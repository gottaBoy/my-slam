// 把 Gazebo Sim 的传感器话题直接转成 ROS 话题的小桥节点。
//
// ============================================================================
// 为什么需要这个节点
// ============================================================================
// 本环境（aarch64 / Ubuntu 24.04 / ros_gz_bridge 1.0.24-1noble.20260904）里，
// ros_gz_bridge 的 parameter_bridge 对下面这些话题报：
//     Failed to create a bridge ...: No template specialization for the pair
//   /imu、/camera/image、/camera/depth_image、/camera/camera_info（/camera/points 时好时坏）
//
// 已确认（不是猜测）：
//   * 用 gdb 打断点看到传给 get_factory() 的类型字符串完全正确
//     （ROS=[sensor_msgs/msg.Imu] GZ=[gz.msgs.IMU]），是库内部查表返回了 null；
//   * 这已经是 apt 源里 arm64 的最新版本（和清华镜像对比过），dpkg -V 也显示文件完好；
//   * 结论：上游这个 arm64 构建的问题，在本仓库层面修不了。
// 详见 README「已知问题」一节、tools/exp-bridge-repro.sh、tools/gdb-get-factory.gdb。
//
// 所以本节点自己用 gz-transport 订阅 gz 话题、构造 ROS 消息发布，
// 完全不经过 parameter_bridge。
//
// ============================================================================
// 覆盖的话题（与 urdf/fishbot/plugins/gz_sensor_plugin.xacro 一一对应）
// ============================================================================
//   gz /imu                -> ROS /imu                 sensor_msgs/msg/Imu
//   gz /camera/image       -> ROS /camera/image        sensor_msgs/msg/Image
//   gz /camera/depth_image -> ROS /camera/depth_image  sensor_msgs/msg/Image
//   gz /camera/camera_info -> ROS /camera/camera_info  sensor_msgs/msg/CameraInfo
//   gz /camera/points      -> ROS /camera/points       sensor_msgs/msg/PointCloud2
//
// 仍然由 parameter_bridge 负责的：/scan、/scan/points 和时钟 —— 那几条实测是好的。
//
// ============================================================================
// 消息字段映射：逐条照抄上游 ros_gz_bridge 的转换代码
// ============================================================================
// 来源：ros_gz 仓库 jazzy 分支的 ros_gz_bridge/src/convert/sensor_msgs.cpp
// （容器里只装了声明头文件，实现在 .so 里，所以是从上游源码抄的）。
// 照抄而不是自己发挥的原因：要让下游（rviz / slam_toolbox / nav2 / 教材代码）
// 看到的东西和用官方桥时完全一致 —— 比如像素编码字符串、CameraInfo 的 K/P/R 填充方式。
//
// 两个容易踩的坑（都在上游代码里有明确写法，这里照做）：
//   1. Image 的 step 是**重新算**的（width * channels * bytes_per_channel），
//      不是直接用 gz 消息里的 step。
//   2. PointCloudPacked 的 Field::DataType 枚举编号和 ROS 的 PointField **不一样**
//      （gz: INT8=0…FLOAT64=7；ROS: INT8=1…FLOAT64=8），必须按名字逐一映射，
//      不能做数值转换。
//
// QoS：用 rclcpp 默认（reliable + keep-last-10），和 parameter_bridge 的默认一致。
// 这一点很重要 —— 如果这里用了 best_effort，reliable 的订阅者（rviz、nav2 等）
// 会因为 QoS 不兼容而收不到数据，表现成「话题有数据但下游没反应」。

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>

namespace
{

// ---------------------------------------------------------------------------
// header 转换
// ---------------------------------------------------------------------------
// gz-msgs10 的 Header 里没有独立的 frame_id 字段，frame_id 是塞在
// `repeated Map data` 里的（key="frame_id"）。xacro 里的 <gz_frame_id> 最终就是
// 通过这个袋子传到消息里的。取不到时用参数给的兜底值。
//
// 另外上游会把 frame_id 里的 "::" 换成 "/"（为了 TF 兼容，见 utils.cpp 的
// frame_id_gz_to_ros），这里也照做。
std::string frameIdFromGzHeader(const gz::msgs::Header & gz_header,
                                const std::string & fallback)
{
  for (int i = 0; i < gz_header.data_size(); ++i) {
    const auto & entry = gz_header.data(i);
    if (entry.key() == "frame_id" && entry.value_size() > 0) {
      std::string frame = entry.value(0);
      // 和上游 frame_id_gz_to_ros() 一致：把 "::" 换成 "/"
      std::string::size_type pos = 0;
      while ((pos = frame.find("::", pos)) != std::string::npos) {
        frame.replace(pos, 2, "/");
        ++pos;
      }
      return frame;
    }
  }
  return fallback;
}

void fillRosHeader(const gz::msgs::Header & gz_header,
                   std_msgs::msg::Header & ros_header,
                   const std::string & fallback_frame)
{
  ros_header.frame_id = frameIdFromGzHeader(gz_header, fallback_frame);
  if (gz_header.has_stamp()) {
    ros_header.stamp.sec = static_cast<int32_t>(gz_header.stamp().sec());
    ros_header.stamp.nanosec = static_cast<uint32_t>(gz_header.stamp().nsec());
  }
}

// 9 元协方差：上游只在 gz 明确给了 9 个数时才填，其余情况保持 0。
// （注意：0 在 robot_localization 里表示「完全可信」，这一点与官方桥的行为一致。）
void copyCovarianceIfPresent(const gz::msgs::Float_V & in, std::array<double, 9> & out)
{
  if (in.data_size() != 9) {
    return;
  }
  for (int i = 0; i < 9; ++i) {
    out[i] = in.data(i);
  }
}

}  // namespace

namespace gz_sensor_bridge
{

class GzSensorBridge : public rclcpp::Node
{
public:
  GzSensorBridge()
  : rclcpp::Node("gz_sensor_bridge")
  {
    imu_frame_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    camera_frame_ = declare_parameter<std::string>("camera_frame_id", "camera_optical_link");

    // QoS 与 parameter_bridge 默认值保持一致（reliable，keep last 10）。
    const auto qos_imu = rclcpp::QoS(rclcpp::KeepLast(10));
    const auto qos_image = rclcpp::QoS(rclcpp::KeepLast(10));

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", qos_imu);
    color_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/image", qos_image);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/depth_image", qos_image);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", qos_image);
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/camera/points", qos_image);

    // 话题名和 xacro 里 <topic> 定义的一致（imu / camera）。
    // 这里直接调用而不是包一层模板，避免成员函数指针的模板推导歧义。
    const bool ok_imu = gz_node_.Subscribe("/imu", &GzSensorBridge::onImu, this);
    const bool ok_color = gz_node_.Subscribe("/camera/image", &GzSensorBridge::onColorImage, this);
    const bool ok_depth =
      gz_node_.Subscribe("/camera/depth_image", &GzSensorBridge::onDepthImage, this);
    const bool ok_info =
      gz_node_.Subscribe("/camera/camera_info", &GzSensorBridge::onCameraInfo, this);
    const bool ok_points = gz_node_.Subscribe("/camera/points", &GzSensorBridge::onPoints, this);

    if (!(ok_imu && ok_color && ok_depth && ok_info && ok_points)) {
      RCLCPP_WARN(get_logger(),
                  "有 gz 话题订阅失败（gz-transport 在话题名非法或重复订阅时返回 false）："
                  " imu=%d image=%d depth=%d info=%d points=%d",
                  ok_imu, ok_color, ok_depth, ok_info, ok_points);
    }

    RCLCPP_INFO(get_logger(),
                "gz 传感器桥已就绪（绕开 parameter_bridge）："
                "/imu /camera/image /camera/depth_image /camera/camera_info /camera/points");
  }

private:

  // ---------------- IMU ----------------
  void onImu(const gz::msgs::IMU & msg)
  {
    sensor_msgs::msg::Imu out;
    fillRosHeader(msg.header(), out.header, imu_frame_);

    out.orientation.x = msg.orientation().x();
    out.orientation.y = msg.orientation().y();
    out.orientation.z = msg.orientation().z();
    out.orientation.w = msg.orientation().w();

    out.angular_velocity.x = msg.angular_velocity().x();
    out.angular_velocity.y = msg.angular_velocity().y();
    out.angular_velocity.z = msg.angular_velocity().z();

    out.linear_acceleration.x = msg.linear_acceleration().x();
    out.linear_acceleration.y = msg.linear_acceleration().y();
    out.linear_acceleration.z = msg.linear_acceleration().z();

    copyCovarianceIfPresent(msg.orientation_covariance(), out.orientation_covariance);
    copyCovarianceIfPresent(msg.angular_velocity_covariance(), out.angular_velocity_covariance);
    copyCovarianceIfPresent(msg.linear_acceleration_covariance(),
                            out.linear_acceleration_covariance);

    imu_pub_->publish(out);
  }

  // ---------------- 图像（彩色 / 深度） ----------------
  // 逐条对应上游 convert_gz_to_ros(gz::msgs::Image, sensor_msgs::msg::Image)。
  static bool fillRosImage(const gz::msgs::Image & msg,
                           const std::string & frame,
                           sensor_msgs::msg::Image & out)
  {
    fillRosHeader(msg.header(), out.header, frame);

    out.height = msg.height();
    out.width = msg.width();

    unsigned int num_channels = 0;
    unsigned int octets_per_channel = 0;

    // 映射表照抄上游；不认识的格式上游是打印并 return，这里也一样。
    switch (msg.pixel_format_type()) {
      case gz::msgs::PixelFormatType::L_INT8:
        out.encoding = "mono8";
        num_channels = 1;
        octets_per_channel = 1u;
        break;
      case gz::msgs::PixelFormatType::L_INT16:
        out.encoding = "mono16";
        num_channels = 1;
        octets_per_channel = 2u;
        break;
      case gz::msgs::PixelFormatType::RGB_INT8:
        out.encoding = "rgb8";
        num_channels = 3;
        octets_per_channel = 1u;
        break;
      case gz::msgs::PixelFormatType::RGBA_INT8:
        out.encoding = "rgba8";
        num_channels = 4;
        octets_per_channel = 1u;
        break;
      case gz::msgs::PixelFormatType::BGRA_INT8:
        out.encoding = "bgra8";
        num_channels = 4;
        octets_per_channel = 1u;
        break;
      case gz::msgs::PixelFormatType::RGB_INT16:
        out.encoding = "rgb16";
        num_channels = 3;
        octets_per_channel = 2u;
        break;
      case gz::msgs::PixelFormatType::BGR_INT8:
        out.encoding = "bgr8";
        num_channels = 3;
        octets_per_channel = 1u;
        break;
      case gz::msgs::PixelFormatType::BGR_INT16:
        out.encoding = "bgr16";
        num_channels = 3;
        octets_per_channel = 2u;
        break;
      case gz::msgs::PixelFormatType::R_FLOAT32:
        out.encoding = "32FC1";
        num_channels = 1;
        octets_per_channel = 4u;
        break;
      default:
        return false;
    }

    out.is_bigendian = false;
    // 注意：step 是重算的，不用 gz 消息里的 step（与上游一致）。
    out.step = out.width * num_channels * octets_per_channel;
    out.data.resize(static_cast<size_t>(out.step) * out.height);
    std::memcpy(out.data.data(), msg.data().c_str(), msg.data().size());
    return true;
  }

  void onColorImage(const gz::msgs::Image & msg)
  {
    sensor_msgs::msg::Image out;
    if (!fillRosImage(msg, camera_frame_, out)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "不支持的像素格式（gz enum 值 %d），跳过",
                           static_cast<int>(msg.pixel_format_type()));
      return;
    }
    color_pub_->publish(out);
  }

  void onDepthImage(const gz::msgs::Image & msg)
  {
    sensor_msgs::msg::Image out;
    if (!fillRosImage(msg, camera_frame_, out)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "不支持的像素格式（gz enum 值 %d），跳过",
                           static_cast<int>(msg.pixel_format_type()));
      return;
    }
    depth_pub_->publish(out);
  }

  // ---------------- CameraInfo ----------------
  void onCameraInfo(const gz::msgs::CameraInfo & msg)
  {
    sensor_msgs::msg::CameraInfo out;
    fillRosHeader(msg.header(), out.header, camera_frame_);

    out.height = msg.height();
    out.width = msg.width();

    out.k.fill(0.0);
    out.p.fill(0.0);
    out.r.fill(0.0);

    if (msg.has_distortion()) {
      const auto & distortion = msg.distortion();
      if (distortion.model() ==
        gz::msgs::CameraInfo::Distortion::PLUMB_BOB)
      {
        out.distortion_model = "plumb_bob";
      } else if (distortion.model() ==
        gz::msgs::CameraInfo::Distortion::RATIONAL_POLYNOMIAL)
      {
        out.distortion_model = "rational_polynomial";
      } else if (distortion.model() ==
        gz::msgs::CameraInfo::Distortion::EQUIDISTANT)
      {
        out.distortion_model = "equidistant";
      }

      out.d.resize(distortion.k_size());
      for (int i = 0; i < distortion.k_size(); ++i) {
        out.d[i] = distortion.k(i);
      }
    }

    if (msg.has_intrinsics()) {
      const int count = std::min<int>(msg.intrinsics().k_size(), 9);
      for (int i = 0; i < count; ++i) {
        out.k[i] = msg.intrinsics().k(i);
      }
    }

    if (msg.has_projection()) {
      const int count = std::min<int>(msg.projection().p_size(), 12);
      for (int i = 0; i < count; ++i) {
        out.p[i] = msg.projection().p(i);
      }
    }

    const int r_count = std::min<int>(msg.rectification_matrix_size(), 9);
    for (int i = 0; i < r_count; ++i) {
      out.r[i] = msg.rectification_matrix(i);
    }

    info_pub_->publish(out);
  }

  // ---------------- 点云 ----------------
  // 注意 Field::DataType 的映射必须按名字做：gz 是 INT8=0…FLOAT64=7，
  // ROS 是 INT8=1…FLOAT64=8，差 1，不能直接做数值转换。
  void onPoints(const gz::msgs::PointCloudPacked & msg)
  {
    sensor_msgs::msg::PointCloud2 out;
    fillRosHeader(msg.header(), out.header, camera_frame_);

    out.height = msg.height();
    out.width = msg.width();
    out.is_bigendian = msg.is_bigendian();
    out.point_step = msg.point_step();
    out.row_step = msg.row_step();
    out.is_dense = msg.is_dense();

    out.data.resize(msg.data().size());
    std::memcpy(out.data.data(), msg.data().c_str(), msg.data().size());

    out.fields.reserve(static_cast<size_t>(msg.field_size()));
    for (int i = 0; i < msg.field_size(); ++i) {
      const auto & gz_field = msg.field(i);
      sensor_msgs::msg::PointField pf;
      pf.name = gz_field.name();
      pf.count = gz_field.count();
      pf.offset = gz_field.offset();
      switch (gz_field.datatype()) {
        default:
        case gz::msgs::PointCloudPacked::Field::INT8:
          pf.datatype = sensor_msgs::msg::PointField::INT8;
          break;
        case gz::msgs::PointCloudPacked::Field::UINT8:
          pf.datatype = sensor_msgs::msg::PointField::UINT8;
          break;
        case gz::msgs::PointCloudPacked::Field::INT16:
          pf.datatype = sensor_msgs::msg::PointField::INT16;
          break;
        case gz::msgs::PointCloudPacked::Field::UINT16:
          pf.datatype = sensor_msgs::msg::PointField::UINT16;
          break;
        case gz::msgs::PointCloudPacked::Field::INT32:
          pf.datatype = sensor_msgs::msg::PointField::INT32;
          break;
        case gz::msgs::PointCloudPacked::Field::UINT32:
          pf.datatype = sensor_msgs::msg::PointField::UINT32;
          break;
        case gz::msgs::PointCloudPacked::Field::FLOAT32:
          pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
          break;
        case gz::msgs::PointCloudPacked::Field::FLOAT64:
          pf.datatype = sensor_msgs::msg::PointField::FLOAT64;
          break;
      }
      out.fields.push_back(pf);
    }

    points_pub_->publish(out);
  }

  std::string imu_frame_;
  std::string camera_frame_;

  gz::transport::Node gz_node_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
};

}  // namespace gz_sensor_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gz_sensor_bridge::GzSensorBridge>());
  rclcpp::shutdown();
  return 0;
}
