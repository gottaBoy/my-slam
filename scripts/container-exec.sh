#!/usr/bin/env bash
# 宿主机侧 wrapper 的公共逻辑，被下面这些脚本 source：
#
#   shell.sh  docker_into.sh  sim.sh  teleop.sh  rviz2.sh  rqt.sh  gazebo.sh
#
# 它集中解决三个反复踩到的坑：
#
# 1) 工作目录
#    compose.yaml 在仓库根，而本文件在根目录下的 scripts/。docker compose 默认
#    只读当前目录的 compose.yaml，在别处执行会直接报
#      no configuration file provided: not found
#    所以这里永远先 cd 到上一级（仓库根）。
#
# 2) 伪终端
#    docker compose exec 默认申请 TTY。当 stdin 不是终端时（脚本、管道、CI），
#    它会申请失败然后一直挂住，最后被 SIGKILL 掉 —— 表现就是退出码 137。
#    这里检测到 stdin 不是终端就自动加 -T。
#
# 3) 容器入口脚本的版本
#    镜像里烘焙的 /usr/local/bin/slam-ros2-entrypoint 里的 overlay 路径是错的
#    （写死 /workspace/install/setup.bash，那个路径不存在），而 bind mount 进来的
#    /workspace/my-slam/entrypoint.sh 是「活的」：改完立即生效，不用重建镜像。
#    所以优先用挂载进来那份，只有它不在时才退回镜像里那份。

SLAM_PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${SLAM_PROJECT_DIR}"

export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"
export CONTAINER_NAME="${CONTAINER_NAME:-slam-ros2-dev}"
export SERVICE_NAME="${SERVICE_NAME:-slam-ros2-dev}"

SLAM_GUEST_ENTRYPOINT_MOUNTED="/workspace/my-slam/entrypoint.sh"
SLAM_GUEST_ENTRYPOINT_IMAGE="/usr/local/bin/slam-ros2-entrypoint"

# 容器没起来时给出可直接照抄的下一步，而不是抛一堆 docker 报错。
cexec_require_running() {
    if ! docker compose ps --status running --services 2>/dev/null \
            | grep -qx "${SERVICE_NAME}"; then
        {
            echo "容器 ${CONTAINER_NAME} 没有在运行。"
            echo "先执行： cd ${SLAM_PROJECT_DIR} && ./scripts/start.sh"
        } >&2
        exit 1
    fi
}

# cexec <容器内命令...>
#   例：cexec bash -i            -> 交互式 shell（已 source overlay）
#       cexec ros2 topic list    -> 一次性命令
#   默认用 exec 替换当前进程（信号/退出码直接透传，更干净）。
#   需要在自己脚本里维护 trap 的（比如 teleop.sh 退出时要清进程）设
#   CEXEC_NO_EXEC=1，这样命令跑完还会回到调用者。
cexec() {
    cexec_require_running

    local tty_args=()
    if [ ! -t 0 ]; then
        tty_args=(-T)
    fi

    local guest_entrypoint="${SLAM_GUEST_ENTRYPOINT_IMAGE}"
    if [ -f "${SLAM_PROJECT_DIR}/entrypoint.sh" ]; then
        guest_entrypoint="${SLAM_GUEST_ENTRYPOINT_MOUNTED}"
    fi

    if [ "${CEXEC_NO_EXEC:-0}" = "1" ]; then
        docker compose exec "${tty_args[@]}" "${SERVICE_NAME}" \
            "${guest_entrypoint}" "$@"
    else
        exec docker compose exec "${tty_args[@]}" "${SERVICE_NAME}" \
            "${guest_entrypoint}" "$@"
    fi
}
