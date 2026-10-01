#!/usr/bin/env bash
# 一次性对照实验：同一配置连续启动 N 次，统计 ros_gz_bridge 每条桥的成败。
#
# 目的：先回答「稳定失败还是抖动」，再谈根因。抖动和稳定是两类完全不同的
#       调查方向，不先分清就会像上次一样在错误的方向上打转。
#
# 本脚本**不修改任何配置**，只做三件事：
#   1. 每次启动前彻底清理残留进程（否则上一轮的 parameter_bridge 会污染测量）
#   2. 启动 launch，等固定时间，记录 9 条桥的成功/失败明细
#   3. 同时抓下 parameter_bridge 进程的 environ / argv / 当时的 ROS 话题表
#      —— 这几份数据是事后做对照的原料，必须在运行时就落盘，不能事后补
#
# 用法（容器内执行）：
#   bash /workspace/my-slam/tools/exp-bridge-repro.sh [次数] [每次等待秒数]
#
# 输出：/tmp/bridge-repro/run<N>.log|.env|.argv|.topics
set -u

N="${1:-3}"
WAIT="${2:-35}"
LOG_DIR="${LOG_DIR:-/tmp/bridge-repro}"
LAUNCH_TARGET='ros2 launch fishbot_description gazebo_sim_gz.launch.py headless:=true'

mkdir -p "${LOG_DIR}"

# 记录本次实验所用的配置指纹，避免事后说不清楚测的是哪一版
{
    echo "=== 实验配置指纹 ==="
    echo "时间: $(date -Is)"
    echo "launch 文件: $(readlink -f /workspace/my-slam/install/fishbot_description/share/fishbot_description/launch/gazebo_sim_gz.launch.py)"
    md5sum /workspace/my-slam/install/fishbot_description/share/fishbot_description/launch/gazebo_sim_gz.launch.py
    echo "ros_gz_bridge 版本: $(dpkg -query -W -f='${Version}' ros-jazzy-ros-gz-bridge 2>/dev/null)"
    echo "gz sim 版本: $(gz sim --versions 2>/dev/null | head -1)"
    echo "命令: ${LAUNCH_TARGET}"
} | tee "${LOG_DIR}/fingerprint.txt"

# 彻底清理：上一轮的桥会订阅同名话题，直接污染下一轮
cleanup_all() {
    local pat
    # 注意：不能用含正则元字符的模式（比如 'ruby.*gz'）—— 这段代码本身也包含同样的
    # 字面量，pkill -f 会匹配到正在执行清理的 shell 自己并把它杀掉。同款坑见 sim.sh。
    for pat in 'ros2 launc[h]' 'parameter_bridg[e]' 'gz si[m]' \
               'robot_state_publishe[r]' 'ros_gz_si[m]' 'controller_manage[r]' \
               'gz_sensor_bridge_nod[e]'; do
        pkill -9 -f "${pat}" 2>/dev/null || true
    done
    sleep 3
}

report() {
    local log="$1"
    printf '  桥 INFO(GZ->ROS)  : %s\n' "$(grep -c 'Creating GZ->ROS Bridge' "$log")"
    printf '  桥 INFO(ROS->GZ)  : %s\n' "$(grep -c 'Creating ROS->GZ Bridge' "$log")"
    printf '  创建失败 WARN     : %s\n' "$(grep -c 'Failed to create a bridge' "$log")"
    grep -o 'Creating GZ->ROS Bridge: \[[^]]*\]' "$log" \
        | sed 's/.*\[\(.*\)\].*/    OK   \1/' | sort -u
    grep -o 'Failed to create a bridge for topic \[[^]]*\]' "$log" \
        | sed 's/.*\[\(.*\)\].*/    FAIL \1/' | sort -u
}

for i in $(seq 1 "${N}"); do
    echo "================ 第 ${i}/${N} 次 ================"
    cleanup_all

    log="${LOG_DIR}/run${i}.log"
    # setsid：脱离当前会话，避免 exec 的父进程退出时被一起收走
    setsid bash -c "${LAUNCH_TARGET}" >"${log}" 2>&1 </dev/null &
    sleep "${WAIT}"

    report "${log}"

    # 抓进程级证据（成功和失败都要抓，否则事后无法对照）
    bpid="$(pgrep -f 'ros_gz_bridg[e]/parameter_bridge' | head -1 || true)"
    if [ -n "${bpid}" ]; then
        tr '\0' '\n' <"/proc/${bpid}/environ" 2>/dev/null | sort >"${LOG_DIR}/run${i}.env"
        tr '\0' '\n' <"/proc/${bpid}/cmdline" 2>/dev/null >"${LOG_DIR}/run${i}.argv"
        printf '  采集到桥进程 pid=%s（environ/argv 已落盘）\n' "${bpid}"
    else
        echo '  !! 没找到 parameter_bridge 进程'
    fi

    # 结果侧证据：ROS 侧到底有哪些话题
    ros2 topic list 2>/dev/null | sort >"${LOG_DIR}/run${i}.topics"
    printf '  ROS 话题数: %s\n' "$(wc -l <"${LOG_DIR}/run${i}.topics")"

    cleanup_all
done

echo "全部日志在 ${LOG_DIR}/"
