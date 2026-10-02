#!/usr/bin/env bash
# 停掉键盘遥控 mybot_teleop.py（宿主机执行），保留仿真 / Nav2 / 图形工具。
#
# 为什么需要它：teleop.sh 的 `trap cleanup INT TERM EXIT` 只在**正常退出**
# （Ctrl-C）时生效。终端被强杀时 SIGKILL 不触发任何 trap，容器里的遥控节点会
# 留下来继续发 /cmd_vel —— 重启仿真后机器人会直接冲出去（docs/问题记录.md E-9）。
# 这个脚本用于「终端已经关了 / 需要脚本化」的场景。
#
# 为什么不用一行 pkill：见 tools/stop-teleop.sh 里的说明
#（F-7 自匹配会让 pkill 杀掉自己，F-12 让退出码变得不可信）。
#
# 用法：./scripts/stop-teleop.sh
set -euo pipefail

. "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/container-exec.sh"

cexec_require_running
cexec bash /workspace/my-slam/tools/stop-teleop.sh
