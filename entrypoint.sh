#!/usr/bin/env bash
# 容器入口：先 source ROS 基础环境 + 工作空间 overlay，再 exec 目标命令。
#
# 为什么要自动发现 overlay 路径（这是踩过的坑）：
#   这里原来写死了 /workspace/install/setup.bash，可是本工作空间真正的 overlay 是
#   /workspace/my-slam/install/setup.bash —— 前者从来就不存在。后果很隐蔽：
#     * 非交互 shell（bash -c）还有 BASH_ENV=/workspace/my-slam/ros-env.sh 兜底，
#       看起来一切正常；
#     * 交互式 shell（./shell.sh 进来的那种）只读 ~/.bashrc，于是
#         ros2 pkg prefix mybot_description   ->  Package not found
#       必须每次手打 source /workspace/my-slam/install/setup.bash 才能用。
#   改成自动发现之后，src/ 下新加包、colcon build 完立即生效，不用改这个文件。
#
# 本文件有两种被使用的方式：
#   1) 镜像构建时 COPY 成 /usr/local/bin/slam-ros2-entrypoint；
#   2) 各 wrapper 直接调用 bind mount 进来的 /workspace/my-slam/entrypoint.sh。
#   走 (2) 的好处是改完立即生效、不必重建镜像；重建之后 (1) 也一样是对的。
set -e

# ROS 和 overlay 的 setup.bash 里会引用未定义的变量（典型的是
# $AMENT_TRACE_SETUP_FILES），在 `set -u` 下会直接以
#   setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
# 中断。所以 source 阶段必须关掉 nounset，source 完再自己打开。
set +u

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

# ---- 自动发现并 source 工作空间 overlay -------------------------------
# 顺序：$OVERLAY_SETUP 显式指定（可含多项，用 : 分隔）> /workspace/install
#       > /workspace/*/install
_slam_overlay_list=""
for _cand in "${OVERLAY_SETUP:-}" \
             /workspace/install/setup.bash \
             /workspace/*/install/setup.bash; do
    [ -f "${_cand}" ] || continue
    # 去重（同一路径可能同时命中显式指定和通配）
    case ":${_slam_overlay_list}:" in
        *":${_cand}:"*) continue ;;
    esac
    _slam_overlay_list="${_slam_overlay_list}${_slam_overlay_list:+:}${_cand}"
done

_slam_saved_ifs="${IFS}"
IFS=':'
# shellcheck disable=SC2086   # 这里就是要按 : 分词
for _setup in ${_slam_overlay_list}; do
    # shellcheck disable=SC1090
    . "${_setup}"
done
IFS="${_slam_saved_ifs}"

# 导出去，方便进去以后自查到底 source 了哪几个 overlay：
#   echo "$SLAM_OVERLAY_SETUP"
export SLAM_OVERLAY_SETUP="${_slam_overlay_list}"
unset _cand _setup _slam_saved_ifs _slam_overlay_list

# 本脚本自己的代码重新用上 nounset，方便早发现问题。
set -u

exec "$@"
