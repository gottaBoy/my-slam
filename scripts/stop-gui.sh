#!/usr/bin/env bash
# 停掉图形化调试工具 rviz2 / rqt（宿主机执行），保留 Gazebo 仿真与 Nav2。
#
# 为什么要有这个脚本：直接写
#   docker compose exec ... bash -lc 'pkill -f "rviz2"'
#   会踩两个坑 ——
#     1) 自匹配：同一条命令行里还出现被匹配的字符串时，pkill 把自己也杀掉（F-7）
#     2) `pkill` 返回 0 只表示信号发出去了，不代表进程已终止；
#        实测 rqt 不理会 SIGTERM，必须 kill -9（F-12）
#   把模式放进容器内的 tools/stop-gui.sh 并用 kill -9，两个坑一起避开。
#
# Gazebo 不在本脚本范围内：`gz sim` 的进程名与 sim.sh 起的完整仿真完全相同，
# 无法区分空世界和 mybot 仿真。
#   只停仿真   -> ./scripts/stop-sim.sh
#   图形 + 仿真 -> ./scripts/stop-gui.sh --with-gazebo
#
# 用法：./scripts/stop-gui.sh [--with-gazebo]
set -euo pipefail

. "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/container-exec.sh"

cexec_require_running
cexec bash /workspace/my-slam/tools/stop-gui.sh "$@"
