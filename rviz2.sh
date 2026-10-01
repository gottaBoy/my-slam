#!/usr/bin/env bash
# 启动 rviz2。overlay 已 source，所以可以直接引用包内资源：
#   ./rviz2.sh -d "$(ros2 pkg prefix mybot_description)/share/mybot_description/rviz/xxx.rviz"
# 也可以直接 ./rviz2.sh 后在里面手动 Add -> By topic。
set -euo pipefail

. "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/container-exec.sh"
cexec rviz2 "$@"
