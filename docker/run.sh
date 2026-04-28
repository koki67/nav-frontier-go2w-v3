#!/bin/bash
# Launch the nav-frontier-go2w-v2 container with X11, CycloneDDS, host networking,
# and the host repository bind-mounted at /external for live config edits.
#
# Usage:
#   bash docker/run.sh                 # interactive bash shell
#   bash docker/run.sh ros2 launch nav_frontier_go2w_bringup bringup.launch.py bridge_dry_run:=true
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

IMAGE="${NAV_FRONTIER_IMAGE:-nav-frontier-go2w-v2:latest}"

# Allow local X11 connections (best-effort; not fatal in headless environments).
xhost +local:docker 2>/dev/null || true

# Generate / refresh XAUTH so containerized GUI apps can reach the host display.
XAUTH=/tmp/.docker.xauth
if [ ! -f "$XAUTH" ]; then
    touch "$XAUTH"
    if command -v xauth >/dev/null 2>&1 && [ -n "$DISPLAY" ]; then
        xauth_list=$(xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' || true)
        if [ -n "$xauth_list" ]; then
            echo "$xauth_list" | xauth -f "$XAUTH" nmerge - 2>/dev/null || true
        fi
    fi
    chmod a+r "$XAUTH"
fi

# Use nvidia runtime if the daemon advertises it.
RUNTIME=""
if docker info 2>/dev/null | grep -q nvidia; then
    RUNTIME="--runtime=nvidia"
fi

# Default command is an interactive bash; override with arguments to this script.
CMD=("${@:-bash}")

exec docker run -it --rm \
    $RUNTIME \
    --privileged \
    --net=host \
    --env="DISPLAY=${DISPLAY:-:0}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --env="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    --env="CYCLONEDDS_URI=file:///etc/cyclonedds.xml" \
    --env="ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$REPO_ROOT:/external:rw" \
    --name="${NAV_FRONTIER_CONTAINER_NAME:-nav-frontier-go2w-v2}" \
    "$IMAGE" \
    "${CMD[@]}"
