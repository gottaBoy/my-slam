import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取与拼接默认路径
    autopatrol_robot_dir = get_package_share_directory(
        'autopatrol_robot')
    patrol_config_path = os.path.join(
        autopatrol_robot_dir, 'config', 'patrol_config.yaml')

    # ------------------------------------------------------------------
    # 本仓库（Gazebo Sim / Jazzy）适配，相对原书改动三处：
    #
    # 1) 相机话题 remap
    #    原书用 /camera_sensor/image_raw（Gazebo classic 相机插件的话题名），
    #    我们的 gz 相机桥出来的是 /camera/image。
    # 2) use_sim_time
    #    原书两个节点都没设 use_sim_time，仿真里必须显式给 true，
    #    否则 BasicNavigator 走系统时钟，等导航激活 / 超时判断都会异常。
    # 3) image_save_path
    #    原书留空，cv2.imwrite 会写进进程的当前工作目录（不固定且容器重建即丢）。
    #    这里固定到工作区目录，并先 mkdir -p。
    #    注意 cv2.imwrite 目录不存在时只返回 False 不抛错，会「静默丢图」。
    # ------------------------------------------------------------------
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    image_save_path = launch.substitutions.LaunchConfiguration(
        'image_save_path', default='/workspace/my-slam/patrol_images/')

    action_node_turtle_control = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='patrol_node',
        parameters=[patrol_config_path,
                    {'use_sim_time': use_sim_time,
                     'image_save_path': image_save_path}],
        remappings=[('/camera_sensor/image_raw', '/camera/image')],
    )
    action_node_patrol_client = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='speaker',
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='使用仿真时钟'),
        launch.actions.DeclareLaunchArgument(
            'image_save_path', default_value=image_save_path,
            description='巡逻拍照保存目录（末尾要带 /）'),
        launch.actions.ExecuteProcess(
            cmd=['mkdir', '-p', image_save_path],
            output='screen'),
        action_node_turtle_control,
        action_node_patrol_client,
    ])
