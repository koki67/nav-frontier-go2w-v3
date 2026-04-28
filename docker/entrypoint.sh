#!/bin/bash
# Entry script for the nav-frontier-go2w-v2 docker image.
# Sources ROS 2 Humble + the in-image workspace overlay, then execs the user command.
set -e

source /opt/ros/humble/setup.bash
if [ -f /workspace/humble_ws/install/setup.bash ]; then
    source /workspace/humble_ws/install/setup.bash
fi

exec "$@"
