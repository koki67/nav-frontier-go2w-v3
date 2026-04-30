#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "${SCRIPT_DIR}/setup_bag_replay.bash"

exec rviz2 -d "${SCRIPT_DIR}/frontier_remote.rviz" --ros-args -p use_sim_time:=true
