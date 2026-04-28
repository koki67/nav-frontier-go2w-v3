#!/bin/bash
# Smoke test: bring up the full stack with the bridge in dry-run, replay a
# pre-recorded bag through it, and assert that /frontier_goal and /cmd_vel
# both publish at least once.
#
# Usage:
#   bash scripts/smoke_test.sh /path/to/bag_directory
#
# Run this inside the container (after docker/run.sh) so the workspace
# overlay is sourced and dependencies are available.
set -e

BAG_PATH="${1:-/external/humble_ws/bags/go2w_office_run}"

if [ ! -e "$BAG_PATH" ]; then
    echo "Bag not found at: $BAG_PATH" >&2
    echo "Pass the bag directory as the first argument." >&2
    exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 not in PATH. Run this inside the docker container (docker/run.sh)." >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
[ -f /workspace/humble_ws/install/setup.bash ] && source /workspace/humble_ws/install/setup.bash

LOG_DIR=/tmp/nav_frontier_smoke_$$
mkdir -p "$LOG_DIR"

echo "[smoke] Launching bringup with bridge in dry-run, logs in $LOG_DIR ..."
ros2 launch nav_frontier_go2w_bringup bringup.launch.py \
    use_rviz:=false \
    bridge_dry_run:=true \
    use_sim_time:=true > "$LOG_DIR/bringup.log" 2>&1 &
BRINGUP_PID=$!

cleanup() {
    echo "[smoke] cleaning up..."
    kill -INT "$BRINGUP_PID" 2>/dev/null || true
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BRINGUP_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[smoke] Waiting 8s for nodes to come up..."
sleep 8

echo "[smoke] Replaying bag..."
ros2 bag play "$BAG_PATH" --clock > "$LOG_DIR/bag_play.log" 2>&1 &
BAG_PID=$!

# Race: wait up to 60s for both /frontier_goal and /cmd_vel to publish at least
# once. Use ros2 topic echo with --once and a timeout.
GOAL_OK=false
CMD_OK=false
for _ in $(seq 1 60); do
    if [ "$GOAL_OK" = "false" ] && timeout 1 ros2 topic echo --once /frontier_goal >/dev/null 2>&1; then
        echo "[smoke] saw /frontier_goal"
        GOAL_OK=true
    fi
    if [ "$CMD_OK" = "false" ] && timeout 1 ros2 topic echo --once /cmd_vel >/dev/null 2>&1; then
        echo "[smoke] saw /cmd_vel"
        CMD_OK=true
    fi
    if [ "$GOAL_OK" = "true" ] && [ "$CMD_OK" = "true" ]; then
        break
    fi
    sleep 1
done

if [ "$GOAL_OK" != "true" ]; then
    echo "FAIL: /frontier_goal never published. See $LOG_DIR/bringup.log." >&2
    exit 1
fi
if [ "$CMD_OK" != "true" ]; then
    echo "FAIL: /cmd_vel never published. See $LOG_DIR/bringup.log." >&2
    exit 1
fi
echo "PASS: /frontier_goal and /cmd_vel both published. Logs in $LOG_DIR."
