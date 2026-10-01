#!/usr/bin/env bash

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

# 本工作空间：/workspace/my-slam（src/ 下是本项目的三个包）。
# 旧的 /workspace/install 从来不存在，所以一直没生效。
for ws in /workspace/my-slam /workspace; do
    if [ -f "${ws}/install/setup.bash" ]; then
        source "${ws}/install/setup.bash"
        break
    fi
done

# 保证 Gazebo Sim 能找到本地模型（幂等，已存在则不重复添加）。
# 注意：条目是「模型目录的父目录」，不是模型目录本身。
# 只加 ~/.gz/models。/workspace/gazebo_models 是同一批模型的另一份副本，
# 两个都加会让 model://ur10 出现重名候选，并让 GUI 重复扫描两份目录。
local_models="${HOME}/.gz/models"
case ":${GZ_SIM_RESOURCE_PATH:-}:" in
    *":${local_models}:"*) ;;
    *) export GZ_SIM_RESOURCE_PATH="${local_models}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}" ;;
esac
