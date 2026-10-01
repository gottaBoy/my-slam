#!/usr/bin/env bash
# 交互式进入容器。
#
# 进去以后 overlay 已经 source 好了，可以直接用：
#   ros2 launch mybot_description gazebo_sim_gz.launch.py
#   ros2 pkg prefix mybot_description      # 不再报 Package not found
# 想看内部到底 source 了哪几个 overlay：echo "$SLAM_OVERLAY_SETUP"
#
# 非交互用法（不占终端）：./shell.sh -c 'ros2 topic list'
set -euo pipefail

. "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/container-exec.sh"

if [ $# -gt 0 ]; then
    # 带参数时是「一次性命令」，不要加 -i：非终端环境下 bash -i 会刷
    # "cannot set terminal process group" / "no job control" 之类的噪音。
    cexec bash "$@"
else
    cexec bash -i
fi
