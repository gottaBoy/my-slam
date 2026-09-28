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
