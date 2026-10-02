#!/usr/bin/env bash
# 启动 rqt（图形化话题 / TF / 参数调试工具）。
#
# 启动前会清掉容器内的 ~/.config/ros.org/rqt_gui.ini。
# 原因：rqt 首次运行会把「插件列表 + 面板布局」写进这个 ini，之后**只读它**，
# 不再重新扫描插件。所以新装一个插件（比如 ros-jazzy-rqt-tf-tree）之后，
# 旧 ini 会让它一直不出现 —— 表现成「插件明明装了，Plugins 菜单里就是找不到」。
#
# 清掉它没有持久代价：这个文件在容器可写层里（$HOME 没有 bind mount），
# 容器一重建本来就没了。
#
# 想保留自己调好的面板布局：加 --keep-config
#
# 用法：./scripts/rqt.sh [--keep-config]
set -euo pipefail

. "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/container-exec.sh"

keep=0
if [ "${1:-}" = "--keep-config" ]; then
    keep=1
    shift
fi

cexec_require_running

if [ "$keep" = "0" ]; then
    # 固定加 -T：这里 stdout 被捕获，带 TTY 的 docker compose exec 会挂住。
    # 只在确实存在时才提示，避免每次启动都刷一行没用的输出。
    removed="$(docker compose exec -T "${SERVICE_NAME}" bash -c '
        f="$HOME/.config/ros.org/rqt_gui.ini"
        if [ -f "$f" ]; then rm -f "$f"; echo yes; fi
    ' 2>/dev/null || true)"
    if [ "${removed//[[:space:]]/}" = "yes" ]; then
        echo "已清掉旧的 rqt_gui.ini（它缓存插件列表，不清会让新装的插件不出现）"
        echo "  想保留面板布局：./scripts/rqt.sh --keep-config"
    fi
fi

cexec rqt "$@"
