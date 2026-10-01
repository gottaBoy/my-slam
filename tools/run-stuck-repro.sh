#!/usr/bin/env bash
# 复现「Nav2 反复 Failed to make progress -> 恢复次数耗尽 -> ABORT」并记录整条速度链。
#
# 为什么要这个脚本：手动往 /cmd_vel 发速度能证明"车物理上能动"，但那条路
# **绕过了 Nav2 的速度链**（controller_server -> velocity_smoother ->
# collision_monitor -> /cmd_vel）。要判断零速指令是谁产生的，必须在仿真正常
# 运行时把链上每一级同时录下来。
#
# 用法（容器内）:
#   bash tools/run-stuck-repro.sh                       # 用默认起点和目标
#   bash tools/run-stuck-repro.sh 2.283 1.760 -4.5 1.5  # 自定义 起点x 起点y 目标x 目标y
#   DURATION=200 bash tools/run-stuck-repro.sh          # 自定义录多久

START_X=${1:-2.283}
START_Y=${2:-1.760}
GOAL_X=${3:--4.5}
GOAL_Y=${4:-1.5}
DURATION=${DURATION:-150}

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source /workspace/my-slam/install/setup.bash >/dev/null 2>&1

echo "=== 1. 把车瞬移回起点 (${START_X}, ${START_Y}) ==="
gz service -s /world/default/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: \"mybot\" position: {x: ${START_X}, y: ${START_Y}, z: 0.001}" 2>&1 | tail -2
sleep 3
echo "   瞬移后真值: $(gz model -m mybot -p 2>/dev/null | tail -2 | tr '\n' ' ')"

echo
echo "=== 2. 按真值重置 AMCL 初始位姿（否则定位和实际对不上）==="
python3 tools/set_initial_pose.py --from-gz 2>&1 | tail -3
sleep 3

echo
echo "=== 3. 启动速度链探针（后台录 ${DURATION}s）==="
python3 tools/probe_cmd_chain.py --seconds "$DURATION" --out /tmp/chain.txt \
  > /tmp/probe_chain.log 2>&1 &
PROBE_PID=$!
sleep 2

echo
echo "=== 4. 下目标 (${GOAL_X}, ${GOAL_Y}) ==="
timeout $((DURATION - 20)) ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | grep -E "number_of_recoveries|distance_remaining|Goal finished|error_code" | tail -4

echo
echo "   结束真值: $(gz model -m mybot -p 2>/dev/null | tail -2 | tr '\n' ' ')"

wait $PROBE_PID 2>/dev/null
cat /tmp/probe_chain.log
