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

## 与 OOMWOO 的隔离

本目录的 Compose 配置不会操作 `oomwoo-feature-dev`、`oomwoo-rviz` 或 `oomwoo-dev-local`，也不使用 OOMWOO 镜像、容器、卷或网络。停止本环境只执行本目录 Compose project 的 `down`。

如果需要同时运行 ROS 2 仿真，保持本环境默认的 `ROS_DOMAIN_ID=49` 和 `GZ_PARTITION=slam-dev-49`，不要改成 OOMWOO 主线使用的值。
