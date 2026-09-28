# slam-ros2-dev

独立的 ROS 2 Jazzy 开发环境，用于 `/home/my/workspace/slam` 下的多个项目。
本目录只存放这个环境新增的 Docker 配置和启动脚本。

## 特性

- 全新构建，基础镜像为官方 `ros:jazzy-ros-base`，桌面和可视化组件在本镜像中单独安装
- 包含 `rqt`、`rviz2`、Gazebo Sim、SLAM Toolbox、Nav2、Cartographer、ROS 2 Control 和常用编译工具
- 主机目录 `/home/my/workspace/slam` 挂载到容器 `/workspace`，因此容器内可以访问该目录下的多个项目
- 使用独立 Docker bridge network：`slam-ros2-dev-net`
- 默认 `ROS_DOMAIN_ID=49`、`GZ_PARTITION=slam-dev-49`
- 容器用户为 `nvidia`，家目录为 `/home/nvidia`，默认密码为 `nvidia`
- 默认不使用 host network、`privileged`、设备映射或 Docker socket
- 容器内用户 UID/GID 与启动用户一致，避免生成 root 文件

容器密码只用于容器内部的登录或 `su`，不代表主机密码，也不适合生产环境。

## SLAM 开发库

2026-09-28 对容器内的开发包、头文件及编译配置检查结果：

| 库 | 当前状态 | 版本 |
| --- | --- | --- |
| Eigen | 已安装 `libeigen3-dev` | 3.4.0 |
| OpenCV | 已安装 `libopencv-dev` 及模块开发包 | 4.6.0 |
| Ceres | 已安装 `libceres-dev` | 2.2.0 |
| Pangolin | 未安装 | - |
| g2o | 未安装 | - |

Eigen、OpenCV、Ceres 目前由 ROS 相关依赖引入，并非 Dockerfile 中显式指定的包；
Pangolin、g2o 未自动补装，也未修改其他项目自带的依赖。

## 启动

```bash
cd /home/my/workspace/slam/my-slam
./start.sh
```

首次启动会拉取 `ros:jazzy-ros-base` 并安装桌面、Gazebo、SLAM、Nav2 等依赖，再构建
`slam-ros2-dev:jazzy`，耗时取决于网络和 Docker 缓存。

## 构建网络记录

截至 **2026-09-28**，当前 Dockerfile **没有默认使用国内镜像**，APT 使用：

- Ubuntu ARM64：`ports.ubuntu.com/ubuntu-ports`
- ROS 2：`packages.ros.org/ros2/ubuntu`

首次构建实际下载约 836 MB，在当前机器上约耗时 51 分钟；后续启动会复用本地镜像缓存，不应重复下载这层。

容器用户名和 UID/GID 参数定义在依赖安装层之后，修改用户配置不需要重新安装 ROS。
Dockerfile 前部保留首次构建的旧参数默认值，用于复用已完成的依赖层缓存。

已验证可用的国内索引：

- Ubuntu ARM64：阿里云、清华大学的 `ubuntu-ports`
- ROS 2 Jazzy：阿里云、清华大学、中科大、南京大学的 `ros2/ubuntu` `noble` 索引

ROS 2 软件源按 Ubuntu 发行版命名，Jazzy 对应 `noble`，不是
`ros2/ubuntu/dists/jazzy`。当前配置暂不自动切换源，以保持已经构建好的镜像和默认环境稳定；
如需切换，应在重新构建前明确指定并重新验证完整依赖下载。

## 常用入口

```bash
./shell.sh
./rqt.sh
./rviz2.sh
./gazebo.sh
./stop.sh
```

也可以使用 Apollo 风格的入口：

```bash
bash ./docker_run.sh
bash ./docker_into.sh
bash ./docker_stop.sh
```

这三个脚本只操作 Compose 项目 `slam-ros2-dev`，不会操作 OOMWOO 容器。

例如启动 Gazebo 世界：

```bash
./gazebo.sh /workspace/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds/empty_room/world.sdf
```

## 构建具体工作空间

不要在 `/workspace` 根目录直接执行 `colcon build`。进入具体 ROS 2 工作空间后再构建，例如：

```bash
./shell.sh
cd /workspace/ros2bookcode/chapt7/chapt7_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 3d 
```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.1 --y 0.0 --z 0.2 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id base_laser

ros2 run tf2_ros static_transform_publisher \
  --x 0.3 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_laser --child-frame-id wall_point

ros2 run tf2_ros tf2_echo base_link wall_point

# 查看 TF 树结构
ros2 run tf2_tools tf2_monitor

# 查看两个 frame 之间的变换
ros2 run tf2_ros tf2_echo base_link laser_frame

# 查看所有 frame
ros2 topic echo /tf_static

# pdf
ros2 run tf2_tools view_frames
ros2 topic info /tf_static
```

### `tf2_echo` 输出说明

执行：

```bash
ros2 run tf2_ros tf2_echo base_link wall_point
```

启动初期如果 `base_link` 还没有被发布，可能出现：

```text
Waiting for transform base_link -> wall_point:
Invalid frame ID "base_link" passed to canTransform argument target_frame
```

这表示当时 TF 树中还不存在 `base_link`。发布 TF 的节点启动后，若持续输出以下结果，说明变换已经可用：

```text
Translation: [0.400, 0.000, 0.200]
Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]
Rotation: in RPY (radian) [0.000, -0.000, 0.000]
```

即平移为 `(0.4, 0.0, 0.2)`，旋转为单位旋转。若一直等待，应检查 TF 发布节点、frame 名称以及 `ROS_DOMAIN_ID` 是否一致。

## 3d tools
```bash
sudo apt install ros-humble-mrpt2 -y
3d-rotation-converter

sudo apt-get install ros-humble-rqt-tf-tree


sudo apt install ros-$ROS_DISTRO-tf-transformations
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# 欧拉角 → 四元数
q = quaternion_from_euler(0, 0, 1.57)  # 绕 z 轴转 90°

# 四元数 → 欧拉角
roll, pitch, yaw = euler_from_quaternion([0, 0, 0.707, 0.707])
sudo pip3 install transforms3d
import transforms3d as tfs

# 欧拉角 → 旋转矩阵
R = tfs.euler.euler2mat(0, 0, 1.57)

# 旋转矩阵 → 四元数
q = tfs.quaternions.mat2quat(R)

# 轴角 → 四元数
q = tfs.axangles.axangle2quat([0, 0, 1], 1.57)
```

## domainID

停止本环境只执行本目录 Compose project 的 `down`。

如果需要同时运行 ROS 2 仿真，保持本环境默认的 `ROS_DOMAIN_ID=49` 和 `GZ_PARTITION=slam-dev-49`

## 生成项目
```bash
ros2 pkg create --build-type ament_python \
  --dependencies rclpy geometry_msgs tf_ros tf_transformations \
  --license Apache-2.0 \
  my_tf_pkg
```

## 项目结构
```bash
my_tf_pkg/
├── my_tf_pkg/
│   └── __init__.py
├── resource/
│   └── my_tf_pkg
├── test/
├── package.xml       # 包信息（依赖写在这里）
├── setup.py          # Python 包安装配置
└── setup.cfg
```

## 安装插件
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-tf-transformations
sudo apt install ros-jazzy-tf-transformations
sudo pip3 install tf-transformations
source /opt/ros/$ROS_DISTRO/setup.bash
```

## 调试运行-1
```bash
colcon build 
source install/setup.bash
ros2 run my_tf_pkg static_tf_broadcaster
ros2 topic list 
ros2 topic echo /tf_static
```

## 调试运行-2
```bash
colcon build 
source install/setup.bash
ros2 run my_tf_pkg dynamic_tf_broadcaster
ros2 run tf2_ros tf2_echo base_link bottle_link
```