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
    exit 1
fi

docker compose up -d --build

echo "Started ${CONTAINER_NAME}"
echo "Workspace: ${SCRIPT_DIR}/.. -> /workspace"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "GZ_PARTITION=${GZ_PARTITION}"
echo
echo "Enter shell: ./shell.sh"
echo "Run rqt:     ./rqt.sh"
echo "Run RViz2:   ./rviz2.sh"
echo "Run Gazebo:  ./gazebo.sh"
