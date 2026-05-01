#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/setup_bag_replay.bash"

export PYTHONPATH="${REPO_ROOT}/humble_ws/src/nav_frontier_go2w_planner:${PYTHONPATH:-}"

python3 -m nav_frontier_go2w_planner.map_viz_layers \
    --ros-args \
    --params-file "${REPO_ROOT}/humble_ws/src/nav_frontier_go2w_planner/config/map_viz_layers.yaml" \
    -p use_sim_time:=true &
MAP_VIZ_PID="$!"

cleanup() {
    if kill -0 "${MAP_VIZ_PID}" 2>/dev/null; then
        kill "${MAP_VIZ_PID}" 2>/dev/null || true
        wait "${MAP_VIZ_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

rviz2 -d "${SCRIPT_DIR}/frontier_remote.rviz" --ros-args -p use_sim_time:=true
