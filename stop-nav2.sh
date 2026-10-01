#!/usr/bin/env bash
# 停掉 Nav2 + 巡逻（宿主机执行），保留 Gazebo 仿真。
#
# 为什么要有这个脚本：直接写
#   docker compose exec ... bash -lc 'pkill -f "navigation2.launch.py"'
#   有自匹配风险 —— 命令行里同时出现被匹配的字符串时，pkill 会把自己也杀掉。
#   这里把 pkill 模式放进容器内的 tools/stop-nav2-patrol.sh，彻底规避。
#
# 用法：./stop-nav2.sh
set -euo pipefail

. "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/container-exec.sh"

cexec_require_running
cexec bash /workspace/my-slam/tools/stop-nav2-patrol.sh
