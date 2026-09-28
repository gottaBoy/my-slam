#!/usr/bin/env bash

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"

if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi
