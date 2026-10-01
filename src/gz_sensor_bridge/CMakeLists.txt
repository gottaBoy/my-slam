cmake_minimum_required(VERSION 3.8)
project(gz_sensor_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# gz-transport / gz-msgs 用的是 ROS 的 vendor 包（gz_transport_vendor / gz_msgs_vendor），
# 它们已经把 CMake config 挂进 CMAKE_PREFIX_PATH（source /opt/ros/jazzy/setup.bash 之后），
# 所以这里直接 find_package 就能找到，路径不用写死。
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(gz_sensor_bridge_node src/gz_sensor_bridge_node.cpp)
target_compile_features(gz_sensor_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(gz_sensor_bridge_node rclcpp sensor_msgs)
target_link_libraries(gz_sensor_bridge_node
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)

install(TARGETS gz_sensor_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
