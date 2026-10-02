#!/usr/bin/env bash
# 在容器内停掉图形化调试工具（rviz2 / rqt），不动 Gazebo 仿真、不动 Nav2。
#
# 和 stop-sim.sh / stop-nav2-patrol.sh 同理：把 pkill 的模式写进脚本文件，
# 避免 `docker compose exec ... bash -lc '... pkill -f "xxx" ...'` 时模式与同一
# 命令行里的其它文本自匹配，把执行清理的 shell 自己杀掉（详见 docs/问题记录.md F-7）。
#
# 为什么直接 kill -9 而不是先发默认的 TERM：`pkill` 只负责把信号发出去，
# **退出码不代表进程已终止**。实测 rqt（Qt 应用）不理会 SIGTERM ——
# `kill -TERM` 返回 0，两秒后进程仍在，必须 -9 才干净。详见 F-12。
#
# 为什么不含 Gazebo：`gz sim` 的进程名与 `sim.sh` 起的完整仿真**完全相同**，
# 无法区分「gazebo.sh 的空世界」和「sim.sh 的 mybot 仿真」。
# 要停 Gazebo 请用 ./scripts/stop-sim.sh；想「一键全停」就加 --with-gazebo，
# 本脚本会代调 tools/stop-sim.sh。
#
# 用法（容器内）：bash /workspace/my-slam/tools/stop-gui.sh [--with-gazebo]
# 用法（宿主机）：./scripts/stop-gui.sh [--with-gazebo]

set -uo pipefail

with_gazebo=0
case "${1:-}" in
    --with-gazebo) with_gazebo=1 ;;
    "")            ;;
    *) echo "未知参数：$1（只支持 --with-gazebo）" >&2; exit 2 ;;
esac

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

echo "停止图形化调试工具（不动 Gazebo 仿真 / Nav2）："

# rqt 是 python 进程（comm 是 python3），不能用 -x 精确匹配 comm，
# 只能按完整命令行匹配，所以写成 bin/rq[t]：既够具体，又不会自匹配。
kill_pat 'rqt'   'bin/rq[t]'
kill_pat 'rviz2' 'rviz[2]'

# 进程名精确匹配兜底（-x 比较 comm，不会误伤命令行里恰好含同样字符串的其它进程）。
# 注意 comm 会被截断到 15 个字符；rqt 不能放进来 —— 它的 comm 是 python3，
# -x python3 会把容器里所有 python 进程一起杀掉。
for name in rviz2; do
    if pkill -9 -x "$name" 2>/dev/null; then
        echo "  停止 $name（精确进程名）"
        killed_any=1
    fi
done

sleep 1
left="$(pgrep -f -- 'bin/rq[t]|rviz[2]' 2>/dev/null | wc -l | tr -d ' ')"
[ "$killed_any" = "0" ] && echo "  （没有发现正在运行的 rviz2 / rqt）"
echo "剩余图形工具进程：$left"

if [ "$with_gazebo" = "1" ]; then
    echo
    echo "--with-gazebo：接着停 Gazebo（代调 tools/stop-sim.sh）"
    bash /workspace/my-slam/tools/stop-sim.sh
fi
