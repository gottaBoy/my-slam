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

# 脚本自己定位所在目录，再用它拼同目录脚本的绝对路径。
# 为什么不能写相对路径 `tools/xxx.py`：本脚本常常是从宿主机用
#   ./scripts/shell.sh -c 'bash /workspace/my-slam/tools/run-stuck-repro.sh'
# 调起来的，而那时 cwd 是 /workspace（挂载根），**不是 /workspace/my-slam**。
# 实测踩过：报 `can't open file '/workspace/tools/set_initial_pose.py'`，
# 于是整个复现“跑”了一遍但什么都没录到 —— 比报错更危险的静默失败。
TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source /workspace/my-slam/install/setup.bash >/dev/null 2>&1

echo "=== 0. 前置检查：Nav2 真的在跑吗 ==="
# 为什么先查这一步：Nav2 没起来时，本脚本**会一路跑完并打印「成功」**
# （目标被拒 → 车没动 → 三块话题都是 0 条消息），却什么都没复现出来。
# 实测踩过：第一次跑就是这样，白等了两分多钟。见 问题记录 F-26。
for node in /planner_server /controller_server /amcl; do
  st=$(ros2 lifecycle get "$node" 2>/dev/null | tail -1)
  printf "  %-20s %s\n" "$node" "${st:-（查不到）}"
  case "$st" in
    *inactive*|"")
      echo
      echo "  ✗ $node 不是 active —— 后面的复现会「跑完但什么都没录到」。"
      echo "    最常见原因：Nav2 起后 60 秒内没设初始位姿，planner_server 拿不到"
      echo "    map 帧 -> 激活失败 -> 整链中止（见 问题记录 E-18）。"
      echo "    修法：重起 Nav2 后立刻再跑一次本脚本（第 2 步会设位姿）；"
      echo "          或调 /lifecycle_manager_navigation/manage_nodes (command: 0)。"
      exit 1
      ;;
  esac
done

echo "=== 1. 把车瞬移回起点 (${START_X}, ${START_Y}) ==="
gz service -s /world/default/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req "name: \"mybot\" position: {x: ${START_X}, y: ${START_Y}, z: 0.001}" 2>&1 | tail -2
sleep 3
echo "   瞬移后真值: $(gz model -m mybot -p 2>/dev/null | tail -2 | tr '\n' ' ')"

echo
echo "=== 2. 按真值重置 AMCL 初始位姿（否则定位和实际对不上）==="
python3 "$TOOLS_DIR/set_initial_pose.py" --from-gz 2>&1 | tail -3
sleep 3

echo
echo "=== 3. 启动速度链探针（后台录 ${DURATION}s）==="
python3 "$TOOLS_DIR/probe_cmd_chain.py" --seconds "$DURATION" --out /tmp/chain.txt \
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

# 守卫 2：三块话题一条非零速度都没有 = 车根本没动 = 这次采样没有意义。
# （宁可报错，也不要让人拿着空数据去下结论。）
nonzero=$(awk 'NR>3 && $6 != "-" && $6 + 0 > 0.001 { n++ } END { print n + 0 }' \
  /tmp/chain.txt 2>/dev/null)
if [ "${nonzero:-0}" -eq 0 ]; then
  echo
echo "⚠️  /cmd_vel 全程没有非零速度 —— 车根本没动，这次采样没有意义。"
  echo "    检查上面第 0 步的生命周期状态；如果有节点不是 active，"
  echo "    先按那里的提示修好再来。"
  exit 1
fi
echo
echo "✓ 校验通过：/cmd_vel 上有 ${nonzero} 个非零采样点，本次复现有效。"
