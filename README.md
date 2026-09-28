# slam-ros2-dev

独立的 ROS 2 Jazzy 开发环境，用于 `/home/my/workspace/slam` 下的多个项目。

## 特性

- 全新构建，基础镜像为官方 `ros:jazzy-ros-base`，桌面和可视化组件在本镜像中单独安装
- 包含 `rqt`、`rviz2`、Gazebo Sim、SLAM Toolbox、Nav2、Cartographer、ROS 2 Control 和常用编译工具
- 主机目录只挂载到容器 `/workspace`
- 使用独立 Docker bridge network：`slam-ros2-dev-net`
- 默认 `ROS_DOMAIN_ID=49`、`GZ_PARTITION=slam-dev-49`
- 默认不使用 host network、`privileged`、设备映射或 Docker socket
- 容器内用户 UID/GID 与启动用户一致，避免生成 root 文件

## 启动

```bash
cd /home/my/workspace/slam/my-slam
./start.sh
```

首次启动会拉取 `ros:jazzy-desktop` 并构建 `slam-ros2-dev:jazzy`，耗时取决于网络和 Docker 缓存。

## 常用入口

```bash
./shell.sh
./rqt.sh
./rviz2.sh
./gazebo.sh
./stop.sh
```

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
