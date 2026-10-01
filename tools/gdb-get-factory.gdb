# 在运行时打印 ros_gz_bridge 查找桥接工厂时用的两个"类型字符串"。
#
# 用法（容器内，需要仿真正在运行才能复现失败）：
#   gdb -q -batch -x /workspace/my-slam/tools/gdb-get-factory.gdb \
#       --args /opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge \
#              '/imu@sensor_msgs/msg.Imu@gz.msgs.IMU'
#
# 背景：parameter_bridge 为每条桥调用 get_factory(ros_type, gz_type)；
# 查不到就抛 "No template specialization for the pair"，被上层 catch 成
# "Failed to create a bridge for topic [...] ..."。打断点把每次调用的两个字符串
# 打出来，就能直接看到"失败的调用拿到的 key 是什么"，不用再黑盒试参数。
#
# 实现说明：ros_gz_bridge 是 Release 构建、无调试信息，所以不能按参数名取参，
# 只能按 aarch64 ABI 从寄存器取：
#     x0 = const std::string & ros_type_name
#     x1 = const std::string & gz_type_name
# libstdc++ 的 std::string 第一个成员就是数据指针（短字符串走 SSO 时，该指针
# 指向对象内部的缓冲区），所以 *(char **)$x0 就是 C 字符串地址。

set pagination off
set confirm off
set breakpoint pending on

break ros_gz_bridge::get_factory__sensor_msgs
commands
  silent
  printf "[lookup] ROS=[%s]  GZ=[%s]\n", *(char **)$x0, *(char **)$x1
  continue
end

run
