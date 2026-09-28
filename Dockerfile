# syntax=docker/dockerfile:1

FROM ros:jazzy-ros-base

ARG USER_NAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=jazzy

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
        ros-jazzy-rviz2 \
        ros-jazzy-robot-localization \
        ros-jazzy-slam-toolbox \
        ros-jazzy-xacro \
    && git lfs install --system \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_GID}" "${USER_NAME}" \
    && useradd \
        --uid "${USER_UID}" \
        --gid "${USER_GID}" \
        --create-home \
        --shell /bin/bash \
        "${USER_NAME}" \
    && printf '%s\n' \
        'source /opt/ros/jazzy/setup.bash' \
        'if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi' \
        >> "/home/${USER_NAME}/.bashrc" \
    && chown "${USER_UID}:${USER_GID}" "/home/${USER_NAME}/.bashrc"

COPY entrypoint.sh /usr/local/bin/slam-ros2-entrypoint
RUN chmod 0755 /usr/local/bin/slam-ros2-entrypoint

WORKDIR /workspace
USER ${USER_NAME}
ENTRYPOINT ["/usr/local/bin/slam-ros2-entrypoint"]
CMD ["sleep", "infinity"]
