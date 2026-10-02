#!/usr/bin/env bash
# 一键停掉本项目起的全部进程（宿主机执行）：仿真 / Nav2 / 巡逻 / 图形工具 / 键盘遥控。
#
# 容器默认**保留**（下次进容器还是现成的）。
# 加 --with-container 则在收尾后连容器一起 down。
#
# 具体停什么、按什么顺序停，见 tools/stop-all.sh 的说明。
#
# 用法：./scripts/stop-all.sh [--with-container]
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

. "${SCRIPT_DIR}/container-exec.sh"

with_container=0
case "${1:-}" in
    --with-container) with_container=1 ;;
    "")               ;;
    *) echo "未知参数：$1（只支持 --with-container）" >&2; exit 2 ;;
esac

cexec_require_running

# CEXEC_NO_EXEC=1：默认的 exec 会替换掉当前进程，后面就没法再跑 stop.sh 了。
CEXEC_NO_EXEC=1 cexec bash /workspace/my-slam/tools/stop-all.sh

if [ "$with_container" = "1" ]; then
    echo
    echo "── --with-container：停掉容器本身 ──"
    exec "${SCRIPT_DIR}/stop.sh"
fi

echo
echo "容器保留未停。要连容器一起停：./scripts/stop.sh，或加 --with-container"
