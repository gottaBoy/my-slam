#!/usr/bin/env bash
# 在容器内停掉键盘遥控节点 mybot_teleop.py（不动仿真、Nav2、图形工具）。
#
# 为什么需要它：teleop.sh 自己写了 `trap cleanup INT TERM EXIT` 兜底，正常用
# Ctrl-C 退出会自己清干净。但 **SIGKILL 不触发任何 trap** —— 终端被强杀 / 被
# 系统回收时，容器里的 mybot_teleop.py 会留下来，继续按最后一条指令发布
# /cmd_vel；重启仿真后机器人会立刻冲出去。详见 docs/问题记录.md E-9。
#（stop-sim.sh 只管仿真进程、stop-gui.sh 只管 rviz2/rqt，都不负责它。）
#
# 把模式写进脚本文件是为了避开 F-7（pkill 的自匹配）；
# 用 kill -9 是为了避开 F-12（pkill 的退出码不代表进程已终止）。
#
# 用法（容器内）：bash /workspace/my-slam/tools/stop-teleop.sh
# 用法（宿主机）：./scripts/stop-teleop.sh

set -uo pipefail

killed_any=0

kill_pat() {
    local label="$1" pat="$2" pids n
    pids="$(pgrep -f -- "$pat" 2>/dev/null || true)"
    if [ -n "$pids" ]; then
        n="$(printf '%s\n' "$pids" | wc -l | tr -d ' ')"
        echo "  停止 $label（$n 个进程）"
        # shellcheck disable=SC2086
        kill -9 $pids 2>/dev/null || true
        killed_any=1
    fi
}

echo "停止键盘遥控（不动仿真 / Nav2 / 图形工具）："
kill_pat 'mybot_teleop.py' 'mybot_teleop\.p[y]'

sleep 1
left="$(pgrep -f -- 'mybot_teleop\.p[y]' 2>/dev/null | wc -l | tr -d ' ')"
[ "$killed_any" = "0" ] && echo "  （没有发现正在运行的 mybot_teleop.py）"
echo "剩余遥控相关进程：$left"
