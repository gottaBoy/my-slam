"""Gazebo Sim (ros_gz) 版本的 fishbot 仿真启动文件。

书上的 gazebo_sim.launch.py 用的是 Gazebo classic（gazebo_ros / spawn_entity.py），
而 ROS 2 Jazzy 已经不再提供 gazebo_ros（classic 于 2025-01 EOL），所以本文件用
ros_gz_sim 重写，功能与书上对应：

    gazebo_ros/gazebo.launch.py   ->  ros_gz_sim/gz_sim.launch.py
    gazebo_ros/spawn_entity.py    ->  ros_gz_sim create
    （classic 插件直接发 ROS 话题）->  ros_gz_bridge 桥接 gz -> ROS 话题
    ros2 control load_controller  ->  controller_manager spawner

启动后的话题（由 config/fishbot_ros2_controller.yaml 与 gz 传感器定义决定）：

    /cmd_vel        (订阅)   ->  /fishbot_diff_drive_controller/cmd_vel
                                注意类型是 geometry_msgs/msg/TwistStamped，
                                不是 Twist！Jazzy 的 diff_drive_controller 4.x
                                已把 ~/cmd_vel 改成 TwistStamped 并移除了
                                use_stamped_vel 参数，所以教材里那个参数现在无效。
                                驱动示例：
                                  ros2 topic pub -r 20 /cmd_vel \
                                    geometry_msgs/msg/TwistStamped \
                                    "{twist: {linear: {x: 0.2}}}"
                                发 Twist 会一直提示 "Waiting for at least 1
                                matching subscription(s)..."，车不会动。
    /odom           (发布)   <-  /fishbot_diff_drive_controller/odom
    /tf, /tf_static          <-  robot_state_publisher + diff_drive_controller
    /joint_states            <-  fishbot_joint_state_broadcaster
    /scan, /scan/points      <-  gpu_lidar  (laser_link)
    /imu                     <-  imu 传感器 (imu_link)
    /camera/image, /camera/depth_image,
    /camera/camera_info, /camera/points  <-  rgbd_camera (camera_optical_link)
    /clock                   <-  Gazebo（供 use_sim_time 使用）

原文件 gazebo_sim.launch.py 保持不动，两条路并存。
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# ros_gz_bridge 的参数格式：
#
#     <话题>@<ROS 类型><方向符><gz 类型>
#            ^^^^^^^^^^^^^^^^^
#   话题和 ROS 类型之间永远是 @；方向符是插在**两个类型之间**的那个字符：
#     @ = 双向，[ = 只 GZ->ROS，] = 只 ROS->GZ
#
# 位置写错（例如 '/scan[sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'）**不会报任何错**，
# 那条桥被静默忽略，现象是"话题存在但一直没有数据"。—— 这条是实测过的。
#
# ⚠️ 关于 /imu 和 /camera/*：这里有过一段「上游缺陷」的结论，现已撤回
#
# 【2026-09-30 观察到的现象】
#   下面这批传感器桥里会有 4~5 条建不出来，报
#       No template specialization for the pair
#   受影响：/imu、/camera/image、/camera/depth_image、/camera/camera_info、/camera/points
#   当时记录的「事实」是：同一份参数连跑 3 次，失败集合完全相同，不像时好时坏；
#   与类型对/话题名/是否双向都无关，但换参数顺序失败集合会变。
#
# 【2026-10-01 复核 —— 结论撤回】
#   在同一个容器里（没有重建镜像、没有换容器）逐个场景重试，
#   **全部 0 失败**，那个失败一次都没再出现：
#     * 单条桥 /imu（单向 [ 和双向 @ 都试）              -> 成功
#     * 当时记录在案的「失败组合」clock,clock,scan,spts,imu -> 5 条全成功
#     * 原始的 9 条桥 + --ros-args remapping 配置         -> 9 条全成功
#   复现脚本：tools/probe-bridge-types.sh（可重复执行，自带对照与判读说明）
#
#   所以：**原因至今未定位，且当前无法复现**。当初那份「稳定失败」的观察应该是
#   真的，但把它归因成「上游 arm64 构建缺陷」是**推测**，没有证据支持，不成立。
#   此处不再写任何未经证实的解释。
#
# 【现状】gz_sensor_bridge 保留 —— 它工作正常且已逐项验证（见 README 的实测表）。
#   但它的定位从「绕开上游 bug 的必要手段」改成「一个可用的替代实现」。
#   哪天真复现了，probe-bridge-types.sh 的输出可以直接拿去报上游。
#
# ⚠️ 这两个必须用**单向** `[`（GZ->ROS），不能用双向 `@`！
#   原因：gz 的激光/点云本来就发布在这两个话题上，如果桥再建一条 ROS->GZ 的方向，
#   就会形成自激回环：
#       激光(gz /scan) -> 桥 -> ROS /scan -> 桥自己订阅 -> 写回 gz /scan -> 桥再收 -> ...
#   实测后果：/scan 频率从 5 Hz 飙到 ~18000 Hz，数据完全不可用（SLAM 会吃到垃圾），
#   而且白烧 CPU。用 `[` 以后 gz 侧只有激光自己的发布者，ROS 侧只有一个发布者，稳定 5 Hz。
BRIDGE_TOPICS = [
    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
]

# 时钟话题是本移植里唯一需要绕一下的地方。实测（gz sim 8.15.0）：
#
#   $ gz topic -i -t /clock                 -> No publishers on topic [/clock]
#   $ gz topic -i -t /world/default/clock   -> Publishers: ..., gz.msgs.Clock
#   $ gz topic -e -t /world/default/clock   -> system { sec: ... }  有数据
#
# 即 gz 只在「世界命名空间」下发时钟，全局 /clock 上没有发布者
# （它之所以出现在 `gz topic -l`，只是因为 bridge 订阅了它）。
# 所以两种可能都桥一次（单向 GZ->ROS），再统一重映射到 ROS 的 /clock；
# 哪一条没有数据就自然闲置，不会报错。世界名从 world 文件里自动解析，不写死。
CLOCK_TOPIC_TEMPLATES = (
    '/world/{world}/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
)


def generate_launch_description():
    robot_name_in_model = 'fishbot'
    pkg_share = get_package_share_directory('fishbot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_model_path = os.path.join(
        pkg_share, 'urdf', 'fishbot', 'fishbot_gz.urdf.xacro')
    default_world_path = os.path.join(pkg_share, 'world', 'custom_room_gz.world')

    declare_model = DeclareLaunchArgument(
        'model', default_value=default_model_path, description='URDF 的绝对路径')
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world_path, description='Gazebo world 的绝对路径')
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true = 只起 Gazebo server（不开 GUI），适合远程/CI 验证')
    declare_verbose = DeclareLaunchArgument(
        'verbose', default_value='1', description='gz sim 日志级别 0-4')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # 启动 Gazebo Sim：-r 立即运行；headless=true 时加 -s 只起 server
    def start_gz_sim(context):
        headless = LaunchConfiguration('headless').perform(context).lower() in (
            '1', 'true', 'yes', 'on')
        verbose = LaunchConfiguration('verbose').perform(context)
        world = LaunchConfiguration('world').perform(context)
        gz_args = '{} -v {} {}'.format('-s -r' if headless else '-r', verbose, world)
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args}.items(),
        )]

    # 把 /robot_description 里的模型生成到 Gazebo 里
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', robot_name_in_model],
        output='screen',
    )

    # gz 话题 -> ROS 话题。时钟话题需要先解析出 world 名，所以放到
    # OpaqueFunction 里构造。
    def start_bridge(context):
        world = LaunchConfiguration('world').perform(context)
        world_name = 'default'
        try:
            with open(world, encoding='utf-8', errors='replace') as handle:
                match = re.search(r'<world\s+name=[\'"]([^\'"]+)', handle.read())
                if match:
                    world_name = match.group(1)
        except OSError:
            pass

        clock_topics = [t.format(world=world_name) for t in CLOCK_TOPIC_TEMPLATES]

        # 时钟 + 雷达：交给 ros_gz_bridge（实测这几条正常）。
        parameter_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=clock_topics + BRIDGE_TOPICS,
            # gz 的 /world/<world>/clock 变成 ROS 的 /clock
            remappings=[('/world/{}/clock'.format(world_name), '/clock')],
            output='screen',
        )

        # IMU + 相机：交给本仓库自己的节点（gz_sensor_bridge）。
        # 它工作正常且已逐项验证；注意这**不是**「绕开上游 bug 的必要手段」——
        # 那个 bug 现在复现不出来、结论已撤回，详见文件顶部那段说明。
        # 两个 frame_id 参数和 urdf/fishbot/plugins/gz_sensor_plugin.xacro 里的
        # <gz_frame_id> 保持一致；节点会优先用 gz 消息里带的 frame_id，取不到才用这两个值。
        sensor_bridge_node = Node(
            package='gz_sensor_bridge',
            executable='gz_sensor_bridge_node',
            parameters=[{
                'imu_frame_id': 'imu_link',
                'camera_frame_id': 'camera_optical_link',
            }],
            output='screen',
        )

        return [parameter_bridge_node, sensor_bridge_node]

    # gz_ros2_control 会把 controller_manager 跑在 Gazebo 进程内，
    # 所以这里用 spawner 去加载控制器，而不是自己起 ros2_control_node。
    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fishbot_joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )

    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fishbot_diff_drive_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )

    return LaunchDescription([
        declare_model,
        declare_world,
        declare_headless,
        declare_verbose,
        robot_state_publisher_node,
        OpaqueFunction(function=start_gz_sim),
        OpaqueFunction(function=start_bridge),
        # Gazebo server 起来需要一点时间，稍等再生成模型
        TimerAction(period=5.0, actions=[spawn_entity_node]),
        # 模型生成完成后再依次加载控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller],
            )
        ),
    ])
