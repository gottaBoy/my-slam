#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"
export CONTAINER_NAME="${CONTAINER_NAME:-slam-ros2-dev}"
export IMAGE_NAME="${IMAGE_NAME:-slam-ros2-dev:jazzy}"
export NETWORK_NAME="${NETWORK_NAME:-slam-ros2-dev-net}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-49}"
export GZ_PARTITION="${GZ_PARTITION:-slam-dev-49}"
export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY_FILE="${XAUTHORITY_FILE:-${XAUTHORITY:-/run/user/${HOST_UID}/gdm/Xauthority}}"

display_number="${DISPLAY#:}"
x11_socket="/tmp/.X11-unix/X${display_number%%.*}"

if [[ ! -S "$x11_socket" ]]; then
    echo "X11 socket not found: $x11_socket" >&2
    echo "Set DISPLAY to an active display or run the container headlessly with:" >&2
    echo "  docker compose up -d --build --no-recreate" >&2
    exit 1
fi

if [[ ! -r "$XAUTHORITY_FILE" ]]; then
    echo "X11 authority file is not readable: $XAUTHORITY_FILE" >&2
    echo "Hint: xauth nlist \"$DISPLAY\" | sed -e 's/^..../ffff/' | xauth -f \"$XAUTHORITY_FILE\" nmerge -" >&2
    exit 1
fi

# 宿主机具备 NVIDIA GPU 直通能力时，自动叠加 GPU 覆盖文件。
# 判定依据：CDI spec 存在，或 Docker 注册了 nvidia runtime。
# 显式设置了 COMPOSE_FILE 的话以用户配置为准。
compose_files=()
if [[ -z "${COMPOSE_FILE:-}" ]] && [[ -f "${SCRIPT_DIR}/compose.nvidia.yaml" ]]; then
    if [[ -e /var/run/cdi/nvidia.yaml ]] \
        || docker info --format '{{range $k, $v := .Runtimes}}{{$k}} {{end}}' 2>/dev/null \
            | grep -qw nvidia; then
        compose_files=(-f compose.yaml -f compose.nvidia.yaml)
        echo "Detected NVIDIA GPU support: GPU passthrough enabled"
        echo "If the host GPU is unusable, rerun with LIBGL_ALWAYS_SOFTWARE=1."
    fi
fi

docker compose "${compose_files[@]}" up -d --build

echo "Started ${CONTAINER_NAME}"
echo "Workspace: ${SCRIPT_DIR}/.. -> /workspace"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "GZ_PARTITION=${GZ_PARTITION}"
echo
echo "Enter shell: ./shell.sh"
echo "Run rqt:     ./rqt.sh"
echo "Run RViz2:   ./rviz2.sh"
echo "Run Gazebo:  ./gazebo.sh"
