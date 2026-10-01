#!/usr/bin/env bash
# 在容器里启动 mybot 键盘遥控（在宿主机上执行）。
#
# 为什么要有这个脚本：`/workspace/...` 是**容器内**路径，在宿主机上不存在。
# 直接照文档在宿主机跑 python3 /workspace/... 会得到
#   python3: can't open file '/workspace/...': [Errno 2] No such file or directory
# 并返回退出码 2。这个包装脚本帮你 cd 到正确位置并在容器里运行。
#
# 用法：
#   ./scripts/teleop.sh                 # 交互式终端：单键模式（w/s/a/d/k/q 不用回车）
#   ./scripts/teleop.sh --line          # 强制行模式（按完回车）
#   ./scripts/teleop.sh --forward       # 不读键盘，直接前进，Ctrl-C 停
#
# 说明：本脚本只在容器内运行，宿主机上不需要装任何 ROS。
#       交互式运行时 docker compose exec 会分配 TTY，脚本自动进入单键模式；
#       如果在管道/非交互环境里调用，会自动加 -T 并退化成行模式。
set -euo pipefail

# 公共逻辑：cd 到脚本目录、非 TTY 自动加 -T、优先用挂载进来的 entrypoint。
. "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/container-exec.sh"

# 关键：docker compose exec 的客户端被 Ctrl-C / kill 掉时，容器里那个
# python 进程不一定会跟着退出。残留的 teleop 会继续按最后一条指令发布
# /cmd_vel，造成「明明没按却还在动」或「新起的遥控行为怪异」。所以这里
# 无论怎么退出都清一次容器内的同名进程。
cleanup() {
    # 这里固定 -T：收尾阶段不需要 TTY，也不该因为 TTY 问题挂住。
    docker compose exec -T "${SERVICE_NAME}" \
        pkill -f 'mybot_teleop.py' >/dev/null 2>&1 || true
}
trap cleanup INT TERM EXIT

# CEXEC_NO_EXEC=1 是必须的：如果用 exec 替换掉进程，EXIT trap 就不会执行，
# 残留的 teleop 会继续发 /cmd_vel。
CEXEC_NO_EXEC=1 cexec python3 \
/workspace/my-slam/src/robot/mybot_description/scripts/mybot_teleop.py "$@"
