#!/bin/bash
# 模块走查 · 第 0~5 步（只读，不改任何文件）
#
# 配套文档：docs/模块详解-mybot_description.md 的「第零节：零基础七步」
#
# 这个脚本把前五步一次性跑完，每一步都打印「敲的命令」和「应该看到什么」，
# 方便你对照自己的输出。所有命令都是**只读**的 —— 不会改任何文件、
# 不会改运行中的仿真，随时可以跑。
#
# 前置条件：仿真已经在跑。先执行：
#     ./scripts/sim.sh --headless --clean
#
# 用法（宿主机）：
#     ./scripts/shell.sh -c 'bash /workspace/my-slam/tools/walk_mybot_description.sh'
#
# 第 6 步（改参数）在另一个脚本里，因为它会改文件：
#     tools/walk_mybot_description_step6.sh

# 注意：**不要**用 `set -u`。ROS 的 setup.bash 会引用未定义变量，
# 开启 -u 会让 shell 在 source 时直接退出（而且错误被重定向后看不到，极难排查）。
set -o pipefail

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SELF_DIR")"
PKG_DIR="$PROJ_DIR/src/robot/mybot_description"

# 容器里可能没 source。这两个 source 是幂等的，重复执行无副作用。
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash 2>/dev/null \
  || { echo "!! 找不到 /opt/ros/jazzy/setup.bash，容器里没装 ROS 2？" >&2; exit 1; }
# shellcheck disable=SC1091
source "$PROJ_DIR/install/setup.bash" 2>/dev/null \
  || { echo "!! 找不到 $PROJ_DIR/install/setup.bash，先跑 colcon build" >&2; exit 1; }

say() { echo; echo "======== $* ========"; }

# ---------------------------------------------------------------- 第 0 步
say "第 0 步 · 确认环境"
echo '$ echo $ROS_DISTRO; ros2 pkg prefix mybot_description'
echo "  ${ROS_DISTRO:-<空>}"
ros2 pkg prefix mybot_description 2>&1 | sed 's/^/  /'
echo
echo "  期望：两行都有。只出 ROS_DISTRO = 没 source 工作空间。"

# ---------------------------------------------------------------- 第 1 步
say "第 1 步 · 展开 xacro（472 行 URDF + 结构树）"
cd "$PKG_DIR" || exit 1
echo '$ xacro urdf/mybot/mybot_gz.urdf.xacro > /tmp/m.urdf'
xacro urdf/mybot/mybot_gz.urdf.xacro > /tmp/m.urdf 2>/tmp/xacro.err
if [ -s /tmp/xacro.err ]; then
  echo "  !! xacro 报错："; sed 's/^/    /' /tmp/xacro.err; exit 1
fi
echo "  行数 = $(wc -l < /tmp/m.urdf)"
echo
echo '$ check_urdf /tmp/m.urdf'
check_urdf /tmp/m.urdf 2>&1 | sed 's/^/  /'
echo
echo "  要点：base_link 下面挂了 7 样东西（2 万向轮 / 相机 / IMU / 雷达杆 / 2 驱动轮）。"
echo "  另：/tmp/m.urdf 留着，以后查尺寸 grep 它比重读 xacro 快。"

# ---------------------------------------------------------------- 第 2 步
say "第 2 步 · 确认仿真真的活着（三个判据）"

echo "判据 1 —— 进程在不在:"
if pgrep -af 'gz[ ]si[m]' | grep -q 'gz sim'; then
  pgrep -af 'gz[ ]si[m]' | sed 's/^/    /'
  echo "    -> OK"
else
  echo "    -> 没有 gz sim 进程。先跑 ./scripts/sim.sh --headless --clean"
  exit 1
fi

echo
echo "判据 2 —— 有人在发数据（Publisher count 必须是 1）:"
for t in /scan /imu /odom; do
  n=$(ros2 topic info "$t" 2>/dev/null | grep -oP 'Publisher count: \K\d+' | head -1)
  printf "    %-14s publisher=%s\n" "$t" "${n:-?}"
done

echo
echo "判据 3 —— 机器人生成到 gz 里了没:"
gz model --list 2>/dev/null | grep -E '^\s+- (mybot|room)' | sed 's/^/    /'
echo "    期望看到 mybot。没有 = gz 起了但模型没进去（查 robot_state_publisher）。"

# ---------------------------------------------------------------- 第 3 步
say "第 3 步 · TF 分静态/动态（最关键的一步）"

echo '$ ros2 topic echo /tf_static --once | grep child_frame_id'
ros2 topic echo /tf_static --once 2>/dev/null | grep child_frame_id | sed 's/^/    /'
n_static=$(ros2 topic echo /tf_static --once 2>/dev/null | grep -c child_frame_id)
echo "    -> 静态 $n_static 条"

echo
echo '$ timeout 6 ros2 topic echo /tf | grep child_frame_id | sort -u'
timeout 6 ros2 topic echo /tf 2>/dev/null | grep child_frame_id | sort -u | sed 's/^/    /'

cat <<'EOF'

  三个要知道的点：
    * 两个轮子不在静态里 -> 关节类型是 continuous（会转）
    * base_footprint 不在静态里 -> 它相对 odom 会移动
    * odom 根本不出现 -> 它不是 robot_state_publisher 发的，
      是 mybot_diff_drive_controller 发的（enable_odom_tf: true）

  最容易卡的点：以为「URDF 的结构 = 全部 TF」。不是。
  URDF 只管 base_footprint 往下那一段。
EOF

# ---------------------------------------------------------------- 第 4 步
say "第 4 步 · 不读文件，直接问 TF：雷达在哪"
echo '$ ros2 run tf2_ros tf2_echo base_link laser_link'
timeout 14 ros2 run tf2_ros tf2_echo base_link laser_link 2>&1 \
  | grep -E 'Translation|RPY \(radian\)' | head -2 | sed 's/^/    /'
echo
echo "  期望 Translation: [0.000, 0.000, 0.150]"
echo "  来源：laser_xacro xyz=0 0 0.10（杆）+ laser_joint z=0.05 = 0.15"
echo
echo "  注意：第一次跑常先打印 Invalid frame ID \"base_link\" ... does not exist，"
echo "  那是 TF 数据还没收全，等 1~2 秒会自己出结果，不是错误。"

# ---------------------------------------------------------------- 第 5 步
say "第 5 步 · 两侧话题对照"

echo "ROS 侧传感器话题:"
ros2 topic list 2>/dev/null | grep -E '^/(scan|imu|camera)' | sed 's/^/    /'
echo
echo "GZ 侧传感器话题:"
gz topic -l 2>/dev/null | grep -E '^/(scan|imu|camera)' | sed 's/^/    /'
echo
echo "只在 ROS 侧有（gz 里没有）—— 这些是 ROS 自己算出来的:"
for t in /odom /cmd_vel /joint_states; do
  gz topic -l 2>/dev/null | grep -qx "$t" || echo "    $t"
done

cat <<'EOF'

  两侧都有的（/scan /imu /camera/*）-> 靠桥连通
  只有 ROS 有的（/odom /cmd_vel /joint_states）-> ROS 自己算的
  只有 GZ 有的（/stats 等）-> gz 内部话题，ROS 不需要
EOF

say "第 0~5 步结束"
echo "下一步：tools/walk_mybot_description_step6.sh apply   （第 6 步：改一个数）"
