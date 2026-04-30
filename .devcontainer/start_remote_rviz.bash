#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IFACE="${1:-enp97s0}"

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/setup_remote_viz.bash" "${IFACE}"

exec rviz2 -d "${SCRIPT_DIR}/frontier_remote.rviz"
