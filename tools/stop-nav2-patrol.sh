#!/usr/bin/env bash
# 在容器内停掉 Nav2 / 巡逻相关进程（保留 Gazebo 仿真本身）。
#
# 为什么要把这些 pkill 模式写进脚本文件，而不是直接写在命令行里：
#   `docker compose exec ... bash -lc '... pkill -f "xxx" ...'`
#   一旦同一条命令行里还出现了会被该模式匹配到的字符串，pkill 就会
#   把自己这个 shell 一起杀掉，表现为命令「什么都没输出就退出了」。
#   本项目已经踩过两次：`ruby.*gz` 和 `navigation2.launch.p[y]`。
#   把模式放进脚本文件后，杀进程的那条命令行本身只包含脚本路径，不可能自匹配。
#
# 用法（容器内）：bash /workspace/my-slam/tools/stop-nav2-patrol.sh
# 用法（宿主机）：./scripts/stop-nav2.sh

set -uo pipefail

killed_any=0

kill_pat() {
    local label="$1" pat="$2"
    local pids
    pids="$(pgrep -f -- "$pat" 2>/dev/null || true)"
    if [ -n "$pids" ]; then
        local n
        n="$(printf '%s\n' "$pids" | wc -l | tr -d ' ')"
        echo "  停止 $label（$n 个进程）"
        # shellcheck disable=SC2086
        kill -9 $pids 2>/dev/null || true
        killed_any=1
    fi
}

echo "停止 Nav2 / 巡逻进程（不动 Gazebo 仿真）："

# 先杀 launch 前端（它们会拉起子进程），再杀子进程。
kill_pat 'navigation2.launch' 'navigation2\.launch\.py'
kill_pat 'autopatrol.launch' 'autopatrol\.launch\.py'
kill_pat 'component_container_isolated' 'component_container_isolated'
kill_pat 'nav2 单独节点' 'nav2_container'

# 巡逻节点用精确进程名匹配（-x 比较 comm），避免误伤别的名字里带 patrol 的进程。
for name in patrol_node speaker; do
    if pkill -9 -x "$name" 2>/dev/null; then
        echo "  停止 $name"
        killed_any=1
    fi
done

sleep 2
left="$(pgrep -f -- 'component_container_isolated|patrol_node|navigation2\.launch\.py' 2>/dev/null | wc -l | tr -d ' ')"
if [ "$killed_any" = "0" ]; then
    echo "  （没有发现需要停止的进程）"
fi
echo "剩余 Nav2/巡逻相关进程：$left"
