#!/usr/bin/env bash
# 对比「AMCL 认为自己在哪」和「Gazebo 里的真实位置」。
#
# 用途：Nav2 出现 "Failed to make progress" 反复重试时，用来区分两种原因：
#   (a) 物理上被挡住（AMCL 位置准确，车就是过不去）
#   (b) 定位飘了（AMCL 位置和真值差很多，车在朝"想象中的目标"开）
#
# 用法（在容器内）:
#   bash tools/probe-localization-vs-truth.sh [目标x] [目标y] [采样秒数]
#
# 输出四列：wall时间 | gz真值 x,y,yaw | AMCL 认为的 x,y | 两者偏差
# 偏差列是判据：< 0.3 m 说明定位正常；持续 > 1 m 说明定位飘了。

# 注意：这里不要用 `set -u`。ROS 的 setup.bash 会引用未定义变量，
# 在 `set -u` 下会直接把当前 shell 打掉（而且下面的 2>/dev/null 会把报错吞掉，
# 表现为"脚本跑完没有任何输出、退出码 1"）。已经踩过一次。
GOAL_X=${1:-4.5}
GOAL_Y=${2:-1.5}
SECONDS_TO_RUN=${3:-100}

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source /workspace/my-slam/install/setup.bash >/dev/null 2>&1

AMCL_F=/tmp/probe_amcl.txt
GZ_F=/tmp/probe_gz.txt
: > "$AMCL_F"
: > "$GZ_F"

echo "起点 gz 真值: $(gz model -m fishbot -p 2>/dev/null | tail -2 | tr '\n' ' ')"

# 后台：持续记录 AMCL 的位姿。ros2 topic echo 输出形如
#     x: 1.23
#     y: 4.56
#     z: 0.0
#     ---
# 用 awk 给每一行加 wall 时间戳（秒级），后面按 x/y 配对还原。
timeout "$SECONDS_TO_RUN" ros2 topic echo /amcl_pose --field pose.pose.position 2>/dev/null \
  | awk '{ printf "%d %s\n", systime(), $0; fflush() }' > "$AMCL_F" 2>&1 &

# 后台：每 2 秒记一次 gz 真值（最后两行 = XYZ / RPY）
timeout "$SECONDS_TO_RUN" bash -c '
  while :; do
    printf "%s " "$(date +%s)"
    gz model -m fishbot -p 2>/dev/null | tail -2 | tr "\n" " "
    echo
    sleep 2
  done' > "$GZ_F" 2>&1 &

# 前台：下目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | grep -E "number_of_recoveries|distance_remaining|Goal finished|error_code" | tail -4

wait 2>/dev/null

echo
echo "=== gz 真值采样： wall时间  x y z   roll pitch yaw ==="
cat "$GZ_F"
echo
echo "=== AMCL 认为的 x,y： wall时间  x  y ==="
awk '/^[0-9]+ x:/{t=$1; x=$3} /^[0-9]+ y:/{printf "%s  %8.3f  %8.3f\n", t, x, $3}' "$AMCL_F"
echo
echo "（原始文件：$AMCL_F / $GZ_F，如需更细的对比可自行查看）"
