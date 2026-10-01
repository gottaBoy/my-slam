#!/usr/bin/env bash
# 启动 rqt（图形化话题/TF/参数调试工具）。
set -euo pipefail

. "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/container-exec.sh"
cexec rqt "$@"
