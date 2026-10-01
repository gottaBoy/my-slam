#!/usr/bin/env bash
# 停掉 Gazebo 仿真（宿主机执行），保留容器本身。
#
# 对应 sim.sh 的停止操作。如果跑 ./scripts/sim.sh 的那个终端还在，直接在那按 Ctrl-C
# 更省事；这个脚本用于「终端已经关了 / 需要脚本化」的场景。
#
# 为什么不用一行 pkill：见 tools/stop-sim.sh 里的说明（pkill 自匹配问题）。
#
# 用法：./scripts/stop-sim.sh
set -euo pipefail

. "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/container-exec.sh"

cexec_require_running
cexec bash /workspace/my-slam/tools/stop-sim.sh
