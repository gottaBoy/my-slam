#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <limits>
#include <memory>
#include <string>

// Jazzy 把 Humble 的 nav2_core/exceptions.hpp 拆成了多个头文件
// （planner_exceptions.hpp / controller_exceptions.hpp / smoother_exceptions.hpp
// / route_exceptions.hpp）。本文件抛 PlannerException，用 planner 那个。
#include "nav2_core/planner_exceptions.hpp"
// 用 LETHAL_OBSTACLE / INSCRIBED_INFLATED_OBSTACLE 这两个常量
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_custom_planner/nav2_custom_planner.hpp"

namespace nav2_custom_planner
{

    void CustomPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        tf_ = tf;
        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        // 参数初始化
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".interpolation_resolution",
                             interpolation_resolution_);

        // 【新行为 L4-3】贴墙走：目标离墙距离，默认 0 = 关闭（保持原行为）
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".wall_clearance", rclcpp::ParameterValue(0.0));
        node_->get_parameter(name_ + ".wall_clearance", wall_clearance_);
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".search_radius", rclcpp::ParameterValue(2.0));
        node_->get_parameter(name_ + ".search_radius", search_radius_);

        RCLCPP_INFO(node_->get_logger(),
                    "CustomPlanner 配置: interpolation_resolution=%.3f  "
                    "wall_clearance=%.3f  search_radius=%.2f",
                    interpolation_resolution_, wall_clearance_, search_radius_);
    }

    void CustomPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "正在清理类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    void CustomPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在激活类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    void CustomPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在停用类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    nav_msgs::msg::Path
    CustomPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                              const geometry_msgs::msg::PoseStamped &goal,
                              std::function<bool()> cancel_checker)
    {
        // 1.声明并初始化 global_path
        nav_msgs::msg::Path global_path;
        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        // 2.检查目标和起始状态是否在全局坐标系中
        if (start.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(node_->get_logger(), "规划器仅接受来自 %s 坐标系的起始位置",
                         global_frame_.c_str());
            return global_path;
        }

        if (goal.header.frame_id != global_frame_)
        {
            RCLCPP_INFO(node_->get_logger(), "规划器仅接受来自 %s 坐标系的目标位置",
                        global_frame_.c_str());
            return global_path;
        }

        // 3.计算当前插值分辨率 interpolation_resolution_ 下的循环次数和步进值
        int total_number_of_loop =
            std::hypot(goal.pose.position.x - start.pose.position.x,
                       goal.pose.position.y - start.pose.position.y) /
            interpolation_resolution_;
        double x_increment =
            (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
        double y_increment =
            (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

        // 4. 生成路径
        for (int i = 0; i < total_number_of_loop; ++i)
        {
            // 新增的 cancel_checker 参数：如果上层（行为树）中途取消了这次规划
            // （比如收到了新目标），及时退出，而不是把整条路径算完。
            if (cancel_checker && cancel_checker())
            {
                throw nav2_core::PlannerException("规划被取消");
            }
            geometry_msgs::msg::PoseStamped pose; // 生成一个点
            pose.pose.position.x = start.pose.position.x + x_increment * i;
            pose.pose.position.y = start.pose.position.y + y_increment * i;
            pose.pose.position.z = 0.0;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            // 将该点放到路径中
            global_path.poses.push_back(pose);
        }

        // 4.5 【新行为 L4-3】「尽量贴墙走」
        //     把中间点朝「最近的障碍」推，直到离它 wall_clearance_ 米。
        //     wall_clearance_ <= 0 时整段跳过 —— 此时与改动前逐位一致。
        if (wall_clearance_ > 0.0 && global_path.poses.size() > 2)
        {
            int hugged = 0, reverted = 0;
            for (size_t i = 1; i + 1 < global_path.poses.size(); ++i)
            {
                auto &p = global_path.poses[i].pose.position;
                double dist = 0.0, dir_x = 0.0, dir_y = 0.0;
                if (!nearestObstacle(p.x, p.y, dist, dir_x, dir_y))
                {
                    continue; // 这个点周围没有墙，保持原样
                }
                // 已经比目标还近就不动（不把车从墙上拽回来）
                if (dist <= wall_clearance_)
                {
                    continue;
                }
                // 朝障碍方向走过去（贴上去）
                double nx = p.x + dir_x * (dist - wall_clearance_);
                double ny = p.y + dir_y * (dist - wall_clearance_);

                // 安全兜底：新位置不能落进内切膨胀区(253)或致命格(254)，
                // 否则宁可退回原位 —— 保证「贴墙」永远不会贴成穿墙。
                unsigned int mx = 0, my = 0;
                if (!costmap_->worldToMap(nx, ny, mx, my) ||
                    costmap_->getCost(mx, my) >=
                        nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                {
                    ++reverted;
                    continue;
                }
                p.x = nx;
                p.y = ny;
                ++hugged;
            }
            RCLCPP_INFO(node_->get_logger(),
                        "贴墙走(wall_clearance=%.2f m): 移动 %d 点，兜底退回 %d 点",
                        wall_clearance_, hugged, reverted);
        }

        // 5.使用 costmap 检查该条路径是否经过障碍物
        for (geometry_msgs::msg::PoseStamped pose : global_path.poses)
        {
            unsigned int mx, my; // 将点的坐标转换为栅格坐标
            if (costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
            {
                unsigned char cost = costmap_->getCost(mx, my); // 获取对应栅格的代价值
                // 如果存在致命障碍物则抛出异常
                if (cost == nav2_costmap_2d::LETHAL_OBSTACLE)
                {
                    RCLCPP_WARN(node_->get_logger(),"在(%f,%f)检测到致命障碍物，规划失败。",
                        pose.pose.position.x, pose.pose.position.y);
                    throw nav2_core::PlannerException(
                        "无法创建目标规划: " + std::to_string(goal.pose.position.x) + "," +
                        std::to_string(goal.pose.position.y));
                }
            }
        }

        // 6.收尾，将目标点作为路径的最后一个点并返回路径
        geometry_msgs::msg::PoseStamped goal_pose = goal;
        goal_pose.header.stamp = node_->now();
        goal_pose.header.frame_id = global_frame_;
        global_path.poses.push_back(goal_pose);
        return global_path;
    }

    // 【新行为 L4-3】从 (x,y) 向外按 1 格步长做「环搜索」，
    // 找到最近的致命格就停（同圈的取最近的），返回距离和方向。
    // 为什么不用代价反推距离：那需要知道 inscribed/csf 两个参数；
    // 直接找致命格则是纯几何的，不依赖膨胀层配置。
    bool CustomPlanner::nearestObstacle(double x, double y, double &dist,
                                        double &dir_x, double &dir_y)
    {
        unsigned int cx = 0, cy = 0;
        if (!costmap_->worldToMap(x, y, cx, cy))
        {
            return false;
        }
        const double res = costmap_->getResolution();
        const int max_r = static_cast<int>(std::ceil(search_radius_ / res));
        const long sx = static_cast<long>(costmap_->getSizeInCellsX());
        const long sy = static_cast<long>(costmap_->getSizeInCellsY());

        long bmx = -1, bmy = -1;
        double best = std::numeric_limits<double>::infinity();

        for (int r = 1; r <= max_r; ++r)
        {
            bool found_this_ring = false;
            // 沿正方形环走一圈：上、右、下、左各 2r 格，共 8r 格
            for (int a = 0; a < 8 * r; ++a)
            {
                int ox = 0, oy = 0;
                if (a < 2 * r) { ox = -r + a; oy = -r; }
                else if (a < 4 * r) { ox = r; oy = -r + (a - 2 * r); }
                else if (a < 6 * r) { ox = r - (a - 4 * r); oy = r; }
                else { ox = -r; oy = r - (a - 6 * r); }

                const long mx = static_cast<long>(cx) + ox;
                const long my = static_cast<long>(cy) + oy;
                if (mx < 0 || my < 0 || mx >= sx || my >= sy)
                {
                    continue;
                }
                if (costmap_->getCost(static_cast<unsigned int>(mx),
                                      static_cast<unsigned int>(my)) !=
                    nav2_costmap_2d::LETHAL_OBSTACLE)
                {
                    continue;
                }
                double mmx = 0.0, mmy = 0.0;
                costmap_->mapToWorld(static_cast<unsigned int>(mx),
                                     static_cast<unsigned int>(my), mmx, mmy);
                const double d = std::hypot(mmx - x, mmy - y);
                if (d < best)
                {
                    best = d;
                    bmx = mx;
                    bmy = my;
                    found_this_ring = true;
                }
            }
            if (found_this_ring)
            {
                break; // 这一圈就有障碍，不必再往外找
            }
        }

        if (bmx < 0)
        {
            return false; // search_radius_ 内没有障碍
        }
        dist = best;
        if (best < 1e-9)
        {
            dir_x = 0.0;
            dir_y = 0.0;
            return true;
        }
        double mmx = 0.0, mmy = 0.0;
        costmap_->mapToWorld(static_cast<unsigned int>(bmx),
                             static_cast<unsigned int>(bmy), mmx, mmy);
        dir_x = (mmx - x) / best;
        dir_y = (mmy - y) / best;
        return true;
    }

} // namespace nav2_custom_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner,
                       nav2_core::GlobalPlanner)