#!/usr/bin/env bash
# 兼容早期「Apollo 风格」的入口命名，与 ./shell.sh 完全等价。
# 保留它只是为了不破坏已有习惯/文档；新脚本请直接写 ./shell.sh。
exec "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/shell.sh" "$@"
