#!/usr/bin/env bash
# Source inside the desktop devcontainer before replaying local rosbags:
#   source .devcontainer/setup_bag_replay.bash

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "This script must be sourced so it can export ROS environment variables." >&2
    exit 2
fi

source /opt/ros/humble/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI

echo "Configured local rosbag replay: RMW=${RMW_IMPLEMENTATION}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}."
