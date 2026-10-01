"""fishbot 仿真一键启动（对应第 9 章 bringup 的「仿真版」）。

为什么要单独有一个：
    第 9 章的 launch/bringup.launch.py 是给**真机**用的，它启动的是
    ydlidar（实体雷达）、micro_ros_agent（单片机）、ros_serial2wifi（串口转
    WiFi）这些节点，依赖三个本仓库没有的包。直接跑会得到：
        PackageNotFoundError: "package 'ydlidar' not found"
    所以那份保持原样（只修了它自己的一个拼写错误），真机用；
    想在本仓库的 Gazebo 仿真里「一条命令拉起整套」，用这个文件。

它做了三件事：
    1. 起 Gazebo 仿真（复用 fishbot_description/gazebo_sim_gz.launch.py）
    2. 12 秒后起 Nav2（等机器人被生成出来；AMCL/map_server 会自己等数据）
    3. 30 秒后可选地设置初始位姿（等 Nav2 激活完）

故意**不**包含 odom2tf：仿真里 odom -> base_footprint 这条 TF 已经由
fishbot_diff_drive_controller（enable_odom_tf: true）在发，再让 odom2tf 发
一遍同一条变换会出现两个发布者互相打架。真机上里程计不发 TF 才需要它。

用法（容器内）：
    ros2 launch fishbot_bringup bringup_sim.launch.py
    ros2 launch fishbot_bringup bringup_sim.launch.py headless:=false rviz:=true
    ros2 launch fishbot_bringup bringup_sim.launch.py initial_pose:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fishbot_description_dir = get_package_share_directory('fishbot_description')
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')

    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    initial_pose = LaunchConfiguration('initial_pose')
    nav2_delay = LaunchConfiguration('nav2_delay')
    pose_delay = LaunchConfiguration('pose_delay')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fishbot_description_dir, 'launch',
                         'gazebo_sim_gz.launch.py')),
        launch_arguments={'headless': headless}.items(),
    )

    nav2 = TimerAction(
        period=LaunchConfiguration('nav2_delay'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fishbot_navigation2_dir, 'launch',
                             'navigation2.launch.py')),
            launch_arguments={'rviz': rviz, 'use_sim_time': 'true'}.items(),
        )],
    )

    # 可选：把机器人位姿告诉 AMCL。书上的流程是手动跑
    # `ros2 run fishbot_application init_robot_pose`，这里作为可选步骤内置。
    # 注意它发的是 (0,0,0)，只适合「刚起仿真、机器人还在原点」的情况；
    # 如果机器人已经被开走了，请用 tools/set_initial_pose.py --from-gz。
    set_initial = TimerAction(
        period=LaunchConfiguration('pose_delay'),
        actions=[Node(
            package='fishbot_application',
            executable='init_robot_pose',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(initial_pose),
            output='screen',
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true',
                              description='只起 Gazebo server 不渲染'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否同时打开 rviz2'),
        DeclareLaunchArgument('initial_pose', default_value='true',
                              description='是否调用 init_robot_pose 设置初始位姿'),
        DeclareLaunchArgument('nav2_delay', default_value='12.0',
                              description='起 Nav2 前等待多少秒（等机器人生成）'),
        DeclareLaunchArgument('pose_delay', default_value='30.0',
                              description='设初始位姿前等待多少秒（等 Nav2 激活）'),
        sim,
        nav2,
        set_initial,
    ])
