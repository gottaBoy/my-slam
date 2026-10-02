#!/bin/bash
# 模块走查 · 第 6 步：改一个数（会改源文件，所以单独放一个脚本）
#
# 配套文档：docs/模块详解-mybot_description.md 的「第零节：零基础七步」
#
# 这一步是「真正搞懂」和「只是看过」的分界线：先写下预测，再动手，最后对比。
# 练习内容是：把雷达的 update_rate 从 10 改成 20，看哪些话题变了。
#
# ⚠️ 这个脚本会**临时修改** urdf/mybot/plugins/gz_sensor_plugin.xacro。
#    它自带备份与还原，用完记得 restore。
#
# 前置条件：
#   1) 仿真已经在跑（第 0~5 步那个脚本的判据 1 通过）
#   2) 源文件是干净状态（git status 里没有它）
#
# 用法（宿主机）：
#   ./scripts/shell.sh -c 'bash /workspace/my-slam/tools/walk_mybot_description_step6.sh status'
#   ./scripts/shell.sh -c 'bash /workspace/my-slam/tools/walk_mybot_description_step6.sh apply'
#   #  -> 然后重启仿真，再跑 measure
#   ./scripts/shell.sh -c 'bash /workspace/my-slam/tools/walk_mybot_description_step6.sh measure'
#   ./scripts/shell.sh -c 'bash /workspace/my-slam/tools/walk_mybot_description_step6.sh restore'

# 注意：**不要**用 `set -u`。ROS 的 setup.bash 会引用未定义变量，
# 开启 -u 会让 shell 在 source 时直接退出（而且错误被重定向后看不到，极难排查）。
set -o pipefail

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SELF_DIR")"
F="$PROJ_DIR/src/robot/mybot_description/urdf/mybot/plugins/gz_sensor_plugin.xacro"
BAK="/tmp/gz_sensor_plugin.xacro.walk.bak"

# 容器里可能没 source。这两个 source 是幂等的，重复执行无副作用。
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash 2>/dev/null \
  || { echo "!! 找不到 /opt/ros/jazzy/setup.bash，容器里没装 ROS 2？" >&2; exit 1; }
# shellcheck disable=SC1091
source "$PROJ_DIR/install/setup.bash" 2>/dev/null \
  || { echo "!! 找不到 $PROJ_DIR/install/setup.bash，先跑 colcon build" >&2; exit 1; }

show_rates() {
  echo "  当前三个 update_rate（行号: 值）:"
  grep -n 'update_rate' "$F" | sed 's/^/    /'
  echo "    （第 33 行=雷达 10 Hz / 第 63 行=IMU 100 Hz / 第 128 行=相机 10 Hz）"
}

cmd_status() {
  echo "======== 第 6 步 · 当前状态 ========"
  show_rates
  echo
  if [ -f "$BAK" ]; then
    echo "  备份存在：$BAK"
    echo "  -> 你可能处在「已 apply 未 restore」状态。用完请跑 restore。"
  else
    echo "  没有备份 = 干净状态。"
  fi
}

cmd_apply() {
  echo "======== 第 6 步 · 动手改 ========"
  if [ -f "$BAK" ]; then
    echo "  !! 备份已存在（$BAK），说明上次没 restore。"
    echo "     先跑 restore，再 apply。"
    exit 1
  fi

  echo "--- 改之前 ---"
  show_rates

  cat <<'EOF'

--- 先写预测（改之前！）---
  改：雷达 gpu_lidar 的 update_rate  10 -> 20
  预测1  /scan              10 Hz -> 20 Hz
  预测2  gz 侧 /scan        也应为 20 Hz（数据源头在 gz）
  预测3  /imu               仍 100 Hz（没动）
  预测4  /camera/camera_info 仍 10 Hz（没动）
  预测5  tools/check_topic_health.sh 应该报 /scan 失败（它期望 10 Hz）

EOF

  cp "$F" "$BAK"
  # 只替换 <sensor name="laserscan"> 块里的 update_rate，不动 IMU 和相机
  python3 - "$F" <<'PY'
import re, sys
p = sys.argv[1]
s = open(p).read()
new, n = re.subn(
    r'(<sensor name="laserscan".*?<update_rate>)10(</update_rate>)',
    r'\g<1>20\g<2>', s, flags=re.S)
open(p, 'w').write(new)
print(f"  替换了 {n} 处（应为 1）")
PY

  echo
  echo "--- 改之后 ---"
  show_rates

  echo
  echo "--- 确认没搞坏 XML ---"
  if xacro "$F" > /dev/null 2>&1; then
    echo "  xacro 展开 OK"
  else
    echo "  !! xacro 报错，正在自动还原"
    cp "$BAK" "$F"; rm -f "$BAK"
    exit 1
  fi

  cat <<'EOF'

--- 接下来 ---
  1) 重启仿真，让新参数生效：
       ./scripts/stop-all.sh
       ./scripts/sim.sh --headless --clean
  2) 再跑 measure：
       ./scripts/shell.sh -c 'bash /workspace/my-slam/tools/walk_mybot_description_step6.sh measure'
  3) 对照预测，最后 **一定要** restore：
       ./scripts/shell.sh -c 'bash /workspace/my-slam/tools/walk_mybot_description_step6.sh restore'

  注意：install/ 里是指向源码的软链接，所以**不需要**重新 colcon build。
EOF
}

cmd_measure() {
  echo "======== 第 6 步 · 验证预测 ========"
  echo "--- ROS 侧速率 ---"
  for t in /scan /imu /camera/camera_info; do
    r=$(timeout 8 ros2 topic hz "$t" 2>/dev/null | head -1 \
        | grep -oP 'average rate: \K[0-9.]+')
    printf "  %-22s %s Hz\n" "$t" "${r:-<无数据>}"
  done

  echo
  echo "--- gz 侧 /scan 速率（数据源头）---"
  # ⚠️ 别用 grep -c "ranges:" 数消息！一条 LaserScan 里有 360 个 range 值，
  #    那样数出来是 360 倍虚高（实测会得到 7000 Hz 这种假数字）。
  #    要数「每条消息只出现一次」的字段，比如 header 里的 sec:。
  n=$(timeout 5 gz topic -e -t /scan 2>/dev/null | grep -cE '^ *sec:')
  echo "  5 秒 $n 条 -> 约 $((n / 5)) Hz"

  echo
  echo "--- 健康检查脚本怎么看 ---"
  bash "$SELF_DIR/check_topic_health.sh" 2>&1 \
    | grep -E '^话题|^---|^/scan' | head -6 | sed 's/^/  /'

  cat <<'EOF'

--- 对照预测 ---
  /scan 应约 20 Hz、/imu 应约 100 Hz、/camera/camera_info 应约 10 Hz，
  且健康检查应报 /scan 失败（它期望 10 Hz）。

  如果 /scan 还是 10 Hz：多半是仿真没重启，不是改错了。
EOF
}

cmd_restore() {
  echo "======== 第 6 步 · 还原 ========"
  if [ ! -f "$BAK" ]; then
    echo "  没有备份，无需还原。当前状态："
    show_rates
    exit 0
  fi
  cp "$BAK" "$F"
  rm -f "$BAK"
  echo "  已还原。"
  show_rates
  echo
  echo "  重启仿真即回到原状：./scripts/stop-all.sh && ./scripts/sim.sh --headless --clean"
}

case "${1:-status}" in
  status)  cmd_status ;;
  apply)   cmd_apply ;;
  measure) cmd_measure ;;
  restore) cmd_restore ;;
  *)
    echo "用法: $0 {status|apply|measure|restore}"
    echo "  status   看当前 update_rate 和有没有残留备份"
    echo "  apply    备份 + 把雷达 update_rate 改成 20"
    echo "  measure  重启仿真后跑这个，验证预测"
    echo "  restore  从备份还原"
    exit 1
    ;;
esac
