#!/usr/bin/env bash
# 检查传感器话题是否健康 —— 把 README 里「发布者数量必须是 1」那条维护要求
# 变成可重复执行的检查。
#
# 为什么需要它：这套仿真有两种传感器桥实现（launch 参数 sensor_bridge）：
#     sensor_bridge:=gz_sensor_bridge   （默认）本仓库自研节点
#     sensor_bridge:=parameter_bridge   ros_gz_bridge 内置桥
# 两者**不能同时开**，否则同一个话题会有两个发布者，频率翻倍、数据重复
# （实测过：/imu 曾测出 300 Hz）。切换模式或改完 launch 之后，跑一遍这个脚本
# 就能确认没有两个发布者在抢同一个话题。
#
# 用法（容器内，仿真已在跑）:
#   bash tools/check_topic_health.sh
# 宿主机:
#   ./scripts/shell.sh -c 'bash /workspace/my-slam/tools/check_topic_health.sh'

source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source /workspace/my-slam/install/setup.bash >/dev/null 2>&1

# 话题:期望频率（Hz，取自 gz_sensor_plugin.xacro 的 update_rate）
# 注意：雷达是 10 Hz（不是旧版面 Gazebo classic xacro 里的 5），点云来自同一个
# 雷达，所以也是 10 Hz —— 这两个值踩过坑，别照旧文档写。
TOPICS=(
  "/scan:10"
  "/scan/points:10"
  "/imu:100"
  "/camera/camera_info:10"
  "/camera/image:0"
  "/camera/depth_image:0"
  "/camera/points:0"
)

fail=0

echo "话题健康检查（发布者必须为 1）"
echo "======================================================================"
printf "%-24s %8s %8s   %s\n" "话题" "发布者" "频率(Hz)" "判定"
echo "----------------------------------------------------------------------"

for entry in "${TOPICS[@]}"; do
  topic="${entry%%:*}"
  want="${entry##*:}"

  # 订阅者数量：用 --verbose 里 PUBLISHER 端点的行数来数
  pub=$(ros2 topic info "$topic" -v 2>/dev/null | grep -c "Endpoint type: PUBLISHER")

  # 频率：timeout 截断长驻命令，用 tail 不用 head（head 会让上游报假错误）
  rate=$(timeout 8 ros2 topic hz "$topic" 2>&1 | grep -m1 "average rate" \
         | grep -oE "[0-9]+\.[0-9]+" | tail -1)
  [ -z "$rate" ] && rate="0.00"

  verdict="OK"
  if [ "$pub" != "1" ]; then
    verdict="失败：发布者应为 1"
    fail=1
  elif [ "$want" != "0" ]; then
    # 允许 ±20% 偏差（渲染/CPU 波动会影响实际频率）
    ok=$(awk -v r="$rate" -v w="$want" \
         'BEGIN { d = r - w; if (d < 0) d = -d; print (d <= 0.2 * w) ? 1 : 0 }')
    if [ "$ok" != "1" ]; then
      verdict="失败：期望约 ${want} Hz"
      fail=1
    fi
  fi

  printf "%-24s %8s %8s   %s\n" "$topic" "$pub" "$rate" "$verdict"
done

echo "======================================================================"
echo "提示：/camera/image、/camera/depth_image、/camera/points 的频率瓶颈是 Gazebo"
echo "      离屏渲染，不设期望值；只要发布者为 1 且数据自洽即可。"
echo "      数据自洽性另跑：python3 tools/check_sensor_msgs.py"

if [ "$fail" = "0" ]; then
  echo "结论：全部通过"
else
  echo "结论：有检查项失败（先确认没有两套桥同时在跑）"
fi
exit "$fail"
