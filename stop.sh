#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"
export CONTAINER_NAME="${CONTAINER_NAME:-slam-ros2-dev}"
export IMAGE_NAME="${IMAGE_NAME:-slam-ros2-dev:jazzy}"
export NETWORK_NAME="${NETWORK_NAME:-slam-ros2-dev-net}"

docker compose down
