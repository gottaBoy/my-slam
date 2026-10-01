#!/usr/bin/env bash
# 穷举测试 ros_gz_bridge 的类型对：哪些 (ROS类型, GZ类型) 能建桥、哪些不能。
#
# 为什么要做这个：报错是
#   Failed to create a bridge ... : No template specialization for the pair
# 这个报错有两种完全不同的可能：
#   (a) 类型串写法不对（只是我们用错了写法）
#   (b) 库内部的工厂注册表确实缺这一项（上游缺陷）
# 光看报错分不出来，必须把候选写法逐个试一遍。
#
# 关键点：建桥只取决于「类型对」能否在 工厂注册表 里查到，**与话题是否真的存在
# 无关**。所以可以对任意话题名建桥，不必去干扰正在跑的仿真。
#
# 用法（容器内）：
#   bash /workspace/my-slam/tools/probe-bridge-types.sh
set -u

TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

run_one() {
    local topic="$1" rtype="$2" gtype="$3"
    # 用 '[' 表示单向 GZ->ROS，避免建出双向桥引入回环
    local spec="${topic}@${rtype}[${gtype}"
    local out="${TMP}/out.txt"
    timeout 8 ros2 run ros_gz_bridge parameter_bridge "${spec}" > "${out}" 2>&1
    if grep -q "Creating GZ->ROS Bridge" "${out}"; then
        printf '  %-26s %-32s %-24s => 成功\n' "${topic}" "${rtype}" "${gtype}"
        return 0
    fi
    local why
    why="$(grep -oE "No template specialization for the pair|Invalid|error.*" "${out}" \
        | head -1)"
    printf '  %-26s %-32s %-24s => 失败  %s\n' \
        "${topic}" "${rtype}" "${gtype}" "${why:-（无明确报错）}"
    return 1
}

# 与 run_one 相同，但用 'v' 的替代：显式双向 '@'，检查方向是否影响建桥
run_bidi() {
    local topic="$1" rtype="$2" gtype="$3"
    local out="${TMP}/out_b.txt"
    timeout 8 ros2 run ros_gz_bridge parameter_bridge \
        "${topic}@${rtype}@${gtype}" > "${out}" 2>&1
    local n_ok n_fail
    n_ok="$(grep -c "Creating GZ->ROS Bridge\|Creating ROS->GZ Bridge" "${out}")"
    n_fail="$(grep -c "No template specialization" "${out}")"
    printf '  %-26s %-32s %-24s => 建桥 %s 条, 失败 %s 条\n' \
        "${topic}" "${rtype}" "${gtype}" "${n_ok}" "${n_fail}"
}

echo "======================================================================"
echo " ros_gz_bridge 版本: $(dpkg-query -W -f='${Version}' ros-jazzy-ros-gz-bridge 2>/dev/null)"
echo " 架构: $(dpkg --print-architecture)"
echo "======================================================================"

echo
echo "--- A. 已知能用的（对照组）---"
run_one /probe_scan     sensor_msgs/msg/LaserScan  gz.msgs.LaserScan
run_one /probe_clock    rosgraph_msgs/msg/Clock    gz.msgs.Clock

echo
echo "--- B. 实际问题：IMU 的各种写法 ---"
run_one /probe_imu1  sensor_msgs/msg/Imu  gz.msgs.IMU
run_one /probe_imu2  sensor_msgs/msg/Imu  gz.msgs.Imu
run_one /probe_imu3  sensor_msgs/msg/Imu  ignition.msgs.IMU

echo
echo "--- C. 实际问题：相机 ---"
run_one /probe_cam1  sensor_msgs/msg/Image       gz.msgs.Image
run_one /probe_cam2  sensor_msgs/msg/CameraInfo  gz.msgs.CameraInfo
run_one /probe_cam3  sensor_msgs/msg/PointCloud2 gz.msgs.PointCloudPacked

echo
echo "--- D. 其它传感器类型，看范围有多大 ---"
run_one /probe_nav  nav_msgs/msg/Odometry   gz.msgs.Odometry
run_one /probe_mag  sensor_msgs/msg/MagneticField gz.msgs.MagneticField
run_one /probe_alt  sensor_msgs/msg/FluidPressure gz.msgs.FluidPressure
run_one /probe_gps  sensor_msgs/msg/NavSatFix gz.msgs.NavSat
run_one /probe_tf   tf2_msgs/msg/TFMessage  gz.msgs.Pose_V

echo
echo "--- E. 双向 '@' vs 单向 '['（方向是否影响建桥）---"
run_bidi /probe_bidi_imu   sensor_msgs/msg/Imu        gz.msgs.IMU
run_bidi /probe_bidi_cam   sensor_msgs/msg/Image      gz.msgs.Image
run_bidi /probe_bidi_scan  sensor_msgs/msg/LaserScan  gz.msgs.LaserScan
run_bidi /probe_bidi_clock rosgraph_msgs/msg/Clock    gz.msgs.Clock

echo
echo "--- F. 一个进程里同时建多条桥（复现原始场景）---"
{
    timeout 10 ros2 run ros_gz_bridge parameter_bridge \
        "/probe_m_clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
        "/probe_m_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" \
        "/probe_m_imu@sensor_msgs/msg/Imu[gz.msgs.IMU" \
        "/probe_m_img@sensor_msgs/msg/Image[gz.msgs.Image" \
        "/probe_m_cinfo@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
        "/probe_m_depth@sensor_msgs/msg/Image[gz.msgs.Image" \
        "/probe_m_points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
        > "${TMP}/multi.txt" 2>&1
} || true
printf '  7 条桥一次性传入 => 建桥成功 %s 条, 失败 %s 条\n' \
    "$(grep -c 'Creating .* Bridge' "${TMP}/multi.txt")" \
    "$(grep -c 'No template specialization' "${TMP}/multi.txt")"
grep -oE "\[WARN\].*" "${TMP}/multi.txt" | head -5

echo
echo "======================================================================"
echo " 判读方法："
echo "   * 如果 B/C 里存在某个写法成功 -> 说明是我们的写法问题，不是上游缺陷"
echo "   * 如果 B/C 所有写法都失败，而 A 全部成功 -> 支持「库注册表缺项」"
echo "   * 再对比 D：如果同族类型（如 Odometry）能成功而 IMU 不能，"
echo "     说明不是「传感器类整体不可用」，而是具体几个类型缺席"
echo "======================================================================"
