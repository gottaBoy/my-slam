# syntax=docker/dockerfile:1

FROM ros:jazzy-ros-base

# Preserve the parameters used to build the existing dependency cache.
ARG USER_NAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------
# 换源层：必须在下面那个大安装层**之前**
#
# 为什么单独提前一层：下面那层要装约 25 个包（首次构建实测下载 836 MB）。
# 换源如果放在它之后，那 836 MB 就全走官方源 —— 实测本机访问
# packages.ros.org 只有 14 KB/s、ports.ubuntu.com 同样慢，而
# mirrors.ustc.edu.cn 约 5.8 MB/s（差约 400 倍）。这是构建慢的唯一原因。
#
# 本层只做 sed、不跑 apt，所以层极小，缓存失效的代价可忽略。
# 要回到官方源： docker compose build --build-arg APT_USE_CN_MIRROR=0
# ---------------------------------------------------------------------------
ARG APT_USE_CN_MIRROR=1
RUN set -eu; \
    if [ "${APT_USE_CN_MIRROR}" = "1" ]; then \
        ubuntu_src=/etc/apt/sources.list.d/ubuntu.sources; \
        ros_src=/usr/share/ros-apt-source/ros2.sources; \
        for f in "${ubuntu_src}" "${ros_src}"; do \
            [ -f "$f" ] || { echo "[apt] 找不到 $f —— 基础镜像的源文件结构变了，请更新本层" >&2; exit 1; }; \
        done; \
        sed -i 's#^Types: deb deb-src#Types: deb#' "${ros_src}"; \
        sed -i 's#http://packages.ros.org/ros2/ubuntu#https://mirrors.ustc.edu.cn/ros2/ubuntu#' "${ros_src}"; \
        sed -i 's#http://ports.ubuntu.com/ubuntu-ports/#https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/#' "${ubuntu_src}"; \
        grep -q 'mirrors.ustc.edu.cn' "${ros_src}" \
            || { echo "[apt] ROS 源替换失败" >&2; exit 1; }; \
        grep -q 'mirrors.tuna.tsinghua.edu.cn' "${ubuntu_src}" \
            || { echo "[apt] Ubuntu 源替换失败" >&2; exit 1; }; \
        echo "[apt] 已切换到国内镜像 (USTC ROS / TUNA Ubuntu) —— 下面这层会快很多"; \
    else \
        echo "[apt] 按 APT_USE_CN_MIRROR=0 使用官方源（会明显更慢）"; \
    fi

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        bash-completion \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        gdb \
        git \
        git-lfs \
        less \
        nano \
        ninja-build \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        python3-venv \
        tmux \
        vim \
        wget \
        ros-jazzy-cartographer-ros \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-nav2-bringup \
        ros-jazzy-nav2-map-server \
        ros-jazzy-nav2-msgs \
        ros-jazzy-navigation2 \
        ros-jazzy-ros-gz \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-rqt \
        ros-jazzy-rqt-common-plugins \
        ros-jazzy-rqt-tf-tree \
        ros-jazzy-rviz2 \
        ros-jazzy-robot-localization \
        ros-jazzy-slam-toolbox \
        ros-jazzy-xacro \
    && git lfs install --system \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 追加依赖层
#
# 为什么单独一层：上面那层 apt 很大（约 836MB / 构建约 51 分钟）。把新包塞进
# 去会让整层缓存失效、重跑一次几十分钟；单独一层则只增量构建。
#
# 换源已在文件开头的「换源层」统一处理（在大安装层之前），这里只装包。
# 需要走官方源时： docker compose build --build-arg APT_USE_CN_MIRROR=0
#
# ros-jazzy-tf-transformations 被以下代码依赖（chapt7 及本仓库原有 my_tf_pkg）：
#   autopatrol_robot/patrol_node.py、mybot_application/get_robot_pose.py、
#   my_tf_pkg/{static_tf_broadcaster,dynamic_tf_broadcaster,tf_listener}.py
#
# ros-jazzy-example-interfaces 被第 10 章的 learn_executor_cpp 依赖
# （example_interfaces/srv/AddTwoInts）。不加的话编译会报：
#   Could not find a package configuration file provided by "example_interfaces"
# ---------------------------------------------------------------------------
RUN apt-get update; \
    apt-get install -y --no-install-recommends \
        ros-jazzy-tf-transformations \
        ros-jazzy-example-interfaces; \
    rm -rf /var/lib/apt/lists/*

ARG CONTAINER_USER=nvidia
ARG HOST_UID=1000
ARG HOST_GID=1000

RUN set -eu; \
    if getent group ubuntu >/dev/null \
        && ! getent group "${CONTAINER_USER}" >/dev/null; then \
        groupmod --new-name "${CONTAINER_USER}" ubuntu; \
    fi; \
    if ! getent group "${HOST_GID}" >/dev/null; then \
        groupadd --gid "${HOST_GID}" "${CONTAINER_USER}-host"; \
    fi; \
    if getent passwd ubuntu >/dev/null \
        && [ "${CONTAINER_USER}" != "ubuntu" ]; then \
        usermod --login "${CONTAINER_USER}" \
            --home "/home/${CONTAINER_USER}" --move-home ubuntu; \
    elif ! getent passwd "${CONTAINER_USER}" >/dev/null; then \
        useradd --uid "${HOST_UID}" --gid "${HOST_GID}" \
            --create-home --shell /bin/bash "${CONTAINER_USER}"; \
    fi; \
    usermod --uid "${HOST_UID}" --gid "${HOST_GID}" \
        --shell /bin/bash "${CONTAINER_USER}"; \
    printf '%s\n' "${CONTAINER_USER}:nvidia" | chpasswd; \
    printf '%s\n' \
        'source /opt/ros/jazzy/setup.bash' \
        'if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi' \
        >> "/home/${CONTAINER_USER}/.bashrc"; \
    chown -R "${HOST_UID}:${HOST_GID}" "/home/${CONTAINER_USER}"

COPY entrypoint.sh /usr/local/bin/slam-ros2-entrypoint
RUN chmod 0755 /usr/local/bin/slam-ros2-entrypoint

WORKDIR /workspace
ENV HOME=/home/${CONTAINER_USER}
USER ${CONTAINER_USER}
ENTRYPOINT ["/usr/local/bin/slam-ros2-entrypoint"]
CMD ["sleep", "infinity"]
