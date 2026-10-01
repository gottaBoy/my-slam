#!/usr/bin/env bash
# 在容器内停掉 Gazebo 仿真相关进程（Gazebo、桥、robot_state_publisher）。
#
# 与 stop-nav2-patrol.sh 同理：把 pkill 的模式写进脚本文件，避免
# `docker compose exec ... bash -lc '... pkill -f "xxx" ...'` 时模式与同一
# 命令行里的其它文本自匹配，把执行清理的 shell 自己杀掉。
#
# 用法（容器内）：bash /workspace/my-slam/tools/stop-sim.sh
# 用法（宿主机）：./scripts/stop-sim.sh

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

echo "停止 Gazebo 仿真进程："

# 先停 launch 前端，再停子进程。
kill_pat 'gazebo_sim_gz.launch' 'gazebo_sim_gz\.launch\.py'
kill_pat 'gz sim (ruby)'        'gz[ ]si[m]'
kill_pat 'ros_gz 桥'            'parameter_bridg[e]'
kill_pat 'gz_sensor_bridge'     'gz_sensor_bridg[e]'
kill_pat 'robot_state_publisher' 'robot_state_publishe[r]'
kill_pat 'gz sim server'        'ruby.*gz[ ]si[m]'

# 进程名精确匹配兜底（-x 比较 comm，不会误伤命令行里出现同样字符串的其它进程）
# 注意 comm 会被截断到 15 个字符。
for name in ruby parameter_bridg gz_sensor_bridg robot_state_pub; do
    if pkill -9 -x "$name" 2>/dev/null; then
        echo "  停止 $name"
        killed_any=1
    fi
done

sleep 2
left="$(pgrep -f -- 'gz[ ]si[m]|parameter_bridg[e]|gz_sensor_bridg[e]' 2>/dev/null | wc -l | tr -d ' ')"
[ "$killed_any" = "0" ] && echo "  （没有发现正在运行的仿真进程）"
echo "剩余仿真相关进程：$left"
