#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"
export CONTAINER_NAME="${CONTAINER_NAME:-slam-ros2-dev}"

if ! docker compose ps --status running --services | grep -qx "slam-ros2-dev"; then
    echo "Container ${CONTAINER_NAME} is not running." >&2
    echo "Start it with: bash ./docker_run.sh" >&2
    exit 1
fi

exec docker compose exec slam-ros2-dev \
    /usr/local/bin/slam-ros2-entrypoint bash -i "$@"
