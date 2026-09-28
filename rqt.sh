#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"
export CONTAINER_NAME="${CONTAINER_NAME:-slam-ros2-dev}"

exec docker compose exec slam-ros2-dev bash -lc 'exec rqt "$@"' bash "$@"
