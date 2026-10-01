#!/usr/bin/env bash
# 从宿主机一键启动 fishbot 的 Gazebo Sim 仿真（含控制器 + 传感器 + ROS 桥接）。
#
# 用法：
#   ./sim.sh                       # 开 GUI（默认）
#   ./sim.sh --headless            # 只起 Gazebo server，不渲染（远程/CI 验证用）
#   ./sim.sh --gz-verbose 4        # gz 日志级别 0-4
#   ./sim.sh --world /容器内/world.sdf
#   ./sim.sh --clean               # 启动前先清掉上一次残留的仿真进程
#   ./sim.sh -- headless:=true     # 原样透传给 ros2 launch
#
# 它做的事等价于进容器执行（overlay 由 entrypoint.sh 自动 source）：
#   ros2 launch fishbot_description gazebo_sim_gz.launch.py <参数...>
#
# 注意：教材原版的 gazebo_sim.launch.py 基于 Gazebo classic（gazebo_ros），
#       Jazzy 已经不提供 gazebo_ros，那条路跑不起来。这里走的是新增的 _gz 版本。
set -euo pipefail

. "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/container-exec.sh"

PKG="${SIM_PKG:-fishbot_description}"
LAUNCH_FILE="${SIM_LAUNCH_FILE:-gazebo_sim_gz.launch.py}"

usage() {
    cat <<'EOF'
用法: ./sim.sh [选项] [-- 额外的 ros2 launch 参数...]

  (无选项)              开 GUI 启动仿真
  --headless            只起 Gazebo server（-s -r），不渲染
  --gz-verbose N        gz sim 日志级别 0-4（默认 1）
  --world PATH          指定 world 文件（容器内绝对路径）
  --clean               启动前清理上一次残留的仿真进程
  -- ros2 launch 参数   原样透传，例如：-- headless:=false
  -h, --help            显示本帮助

环境变量:
  SIM_PKG / SIM_LAUNCH_FILE   换包或换 launch 文件

启动后（另开两个终端）:
  ./teleop.sh                   键盘遥控
  ./shell.sh -c 'ros2 topic list'
EOF
}

launch_args=()
do_clean=0

while [ $# -gt 0 ]; do
    case "$1" in
        --headless)
            launch_args+=('headless:=true'); shift ;;
        --gz-verbose)
            [ $# -ge 2 ] || { echo "--gz-verbose 需要参数" >&2; exit 2; }
            launch_args+=("verbose:=$2"); shift 2 ;;
        --world)
            [ $# -ge 2 ] || { echo "--world 需要参数" >&2; exit 2; }
            launch_args+=("world:=$2"); shift 2 ;;
        --clean)
            do_clean=1; shift ;;
        --)
            shift
            # shellcheck disable=SC2199
            if [ $# -gt 0 ]; then launch_args+=("$@"); fi
            break ;;
        -h|--help)
            usage; exit 0 ;;
        *)
            launch_args+=("$1"); shift ;;
    esac
done

# 想在容器里跑 GUI 但宿主机没有可用的 X11 socket 时，早点说清楚，
# 不要等到 gz 抛 "failed to create drawable" 再回头查。
case " ${launch_args[*]:-} " in
    *" headless:=true "*)
        : ;;
    *)
        display_number="${DISPLAY:-:1}"
        display_number="${display_number#:}"
        x11_socket="/tmp/.X11-unix/X${display_number%%.*}"
        if [ ! -S "${x11_socket}" ]; then
            echo "提示：宿主机找不到 X11 socket ${x11_socket}（DISPLAY=${DISPLAY:-未设置}），" >&2
            echo "      开 GUI 会失败。可以改用： ./sim.sh --headless" >&2
        fi ;;
esac

# ---- 残留进程检查 -----------------------------------------------------
# docker exec 的已知行为：客户端（宿主机上的那个进程）被 Ctrl-C / 关终端 / kill
# 之后，容器里真正的 ros2 launch + gz sim 不会跟着退出，会变成孤儿继续跑。
# 而 GZ_PARTITION 是共用的，两个仿真同时活着会互相抢话题/抢世界，现象极难排查。
# 所以每次启动都先数一遍：--clean 就清掉，否则至少明确警告。
leftover_count() {
    docker compose exec -T "${SERVICE_NAME}" bash -c \
        'pgrep -f "gz si[m]|ros2 launc[h]" 2>/dev/null | wc -l' 2>/dev/null \
        | tr -d '[:space:]'
}

cleanup_leftovers() {
    # 每个 pkill 后面都要 || true：没东西可杀时 pkill 返回 1，
    # 在 set -e 的脚本里会直接把整个流程判成失败。
    #
    # 注意：这里**不能**用含正则元字符的模式（比如 "ruby.*gz"）。
    # 因为这段代码是通过 `bash -c '...'` 传进去的，那条 bash 的命令行里就包含
    # 模式字符串本身，而 "ruby.*gz" 这个正则能匹配 "ruby.*gz" 这段文字 →
    # pkill 会把正在执行清理的 shell 自己杀掉，后面的清理和输出都不会执行。
    # （这个是实测踩到的：清理命令静默中断，什么都不打印。）
    docker compose exec -T "${SERVICE_NAME}" bash -c '
        # 注意：以后往 launch 里**新增常驻节点**时，必须把它的进程名加进这张表。
        # 忘了加的后果实测过：旧实例不会被清掉，ROS 侧同一个话题会出现多个发布者，
        # 频率变成期望值的 2~3 倍（/imu 测出 300 Hz），数据也会重复。
        for pat in "ros2 launc[h]" "gz si[m]" \
                   "robot_state_publishe[r]" "ros_gz_bridg[e]" \
                   "parameter_bridg[e]" "gz_sensor_bridge_nod[e]" \
                   "controller_manage[r]"; do
            pkill -9 -f "$pat" 2>/dev/null || true
        done
        true
    ' >/dev/null 2>&1 || true
    sleep 2
}

running="$(leftover_count || true)"
running="${running:-0}"
if [ "${running}" != "0" ]; then
    if [ "${do_clean}" = "1" ]; then
        echo "== 清理上一次残留的 ${running} 个仿真进程 =="
        cleanup_leftovers
    else
        {
            echo "警告：容器里还有 ${running} 个上一次的仿真进程没退出。"
            echo "      两个仿真共用 GZ_PARTITION，会互相抢话题/抢世界，现象很难查。"
            echo "      建议改用： ./sim.sh --clean ...（或先在旧终端里 Ctrl-C）"
            echo "      现在仍然继续启动。"
        } >&2
    fi
fi

echo "启动仿真：${PKG} / ${LAUNCH_FILE}"
if [ ${#launch_args[@]} -gt 0 ]; then
    echo "  launch 参数： ${launch_args[*]}"
else
    echo "  launch 参数： （默认，带 GUI）"
fi
echo "  停止： 在本终端 Ctrl-C"
echo

if [ ${#launch_args[@]} -gt 0 ]; then
    cexec ros2 launch "${PKG}" "${LAUNCH_FILE}" "${launch_args[@]}"
else
    cexec ros2 launch "${PKG}" "${LAUNCH_FILE}"
fi
