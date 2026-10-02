#!/usr/bin/env bash
# 一键停掉容器内本项目起的全部进程（仿真 / Nav2 / 巡逻 / 图形工具 / 键盘遥控）。
# 容器本身保留 —— 要连容器一起停，在宿主机侧加 --with-container。
#
# 为什么要有它：不用记住「该停哪几个脚本」，也不用记住那条容易出事的扫尾命令
#   docker compose exec ... bash -lc 'pkill -9 -f "ros2 launc[h]"'
#（自匹配会杀掉执行它的 shell，见 docs/问题记录.md F-7）。
#
# 停止顺序是刻意的 —— 先停客户端与指令源，再停依赖它们的栈，最后停地基：
#   1. 图形工具 rviz2 / rqt   纯观察者
#   2. 键盘遥控 mybot_teleop  /cmd_vel 的指令源，要在仿真之前停
#   3. Nav2 / 巡逻            依赖仿真的 /scan /odom /tf
#   4. Gazebo 仿真栈          地基，最后停
#   5. 扫尾：残留的 ros2 launch
#
# 用法（容器内）：bash /workspace/my-slam/tools/stop-all.sh
# 用法（宿主机）：./scripts/stop-all.sh [--with-container]

set -uo pipefail

T=/workspace/my-slam/tools
step() { echo; echo "── $* ──"; }

step '1/5 图形工具（rviz2 / rqt）'
bash "$T/stop-gui.sh"

step '2/5 键盘遥控（mybot_teleop.py）'
bash "$T/stop-teleop.sh"

step '3/5 Nav2 / 巡逻'
bash "$T/stop-nav2-patrol.sh"

step '4/5 Gazebo 仿真栈'
bash "$T/stop-sim.sh"

step '5/5 扫尾：残留的 ros2 launch'
# 模式写在这个文件里（不是命令行里），并且只用 pgrep 取 pid 再 kill ——
# 不调用 pkill，就完全不存在自匹配问题（F-7）。
left_launch="$(pgrep -f -- 'ros2[ ]launc[h]' 2>/dev/null || true)"
if [ -n "$left_launch" ]; then
    n="$(printf '%s\n' "$left_launch" | wc -l | tr -d ' ')"
    # shellcheck disable=SC2086
    kill -9 $left_launch 2>/dev/null || true
    sleep 1
    echo "  已清理残留的 ros2 launch（$n 个）"
else
    echo "  （没有残留的 ros2 launch）"
fi

echo
echo "═══ 汇总（逐项独立数一遍，不看上面各脚本的汇报）═══"
show() {
    local n
    n="$(pgrep -f -- "$2" 2>/dev/null | wc -l | tr -d ' ')"
    printf '  %-22s %s\n' "$1" "$n"
}
show 'rviz2 / rqt'      'rviz[2]|bin/rq[t]'
show 'mybot_teleop'     'mybot_teleop\.p[y]'
show 'Nav2 / 巡逻'      'navigation2\.launch\.py|component_container_isolated|patrol_node|autopatrol\.launch\.py'
show 'Gazebo / 桥'      'gz[ ]si[m]|parameter_bridg[e]|gz_sensor_bridg[e]'
show 'ros2 launch'      'ros2[ ]launc[h]'
show 'robot_state_pub'  'robot_state_publishe[r]'
total="$(pgrep -f -- 'rviz[2]|bin/rq[t]|mybot_teleop\.p[y]|navigation2\.launch\.py|component_container_isolated|gz[ ]si[m]|parameter_bridg[e]|gz_sensor_bridg[e]|ros2[ ]launc[h]' 2>/dev/null | wc -l | tr -d ' ')"
echo "  ──────────────────────────────"
echo "  合计残留：$total"
