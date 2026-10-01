#!/usr/bin/env bash
# 开一个空的 Gazebo Sim GUI（不带机器人），用来手动摆模型、观察世界。
# 跑 mybot 的完整仿真（带控制器/传感器/桥接）请用 ./scripts/sim.sh。
#
# 例子：
#   ./scripts/gazebo.sh                                   # 空世界
#   ./scripts/gazebo.sh /workspace/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds/empty_room/world.sdf
set -euo pipefail

. "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/container-exec.sh"
cexec gz sim "$@"
