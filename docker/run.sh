#!/bin/bash
# Launch the nav-frontier-go2w-v3 container with X11, CycloneDDS, host networking,
# and the host repository bind-mounted at /external for live config edits.
#
# Usage:
#   bash docker/run.sh
#   bash docker/run.sh --remote-viz
#   bash docker/run.sh --remote-viz --remote-viz-iface wlan0 ros2 launch nav_frontier_go2w_bringup bringup.launch.py bridge_dry_run:=true
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

IMAGE="${NAV_FRONTIER_IMAGE:-nav-frontier-go2w-v3:latest}"
ROBOT_IFACE="${NAV_FRONTIER_ROBOT_IFACE:-eth0}"
REMOTE_VIZ_IFACE="${NAV_FRONTIER_REMOTE_IFACE:-wlan0}"
REMOTE_VIZ="${NAV_FRONTIER_REMOTE_VIZ:-false}"
ROBOT_IFACE_OVERRIDDEN=false
if [ -n "${NAV_FRONTIER_ROBOT_IFACE:-}" ]; then
    ROBOT_IFACE_OVERRIDDEN=true
fi

usage() {
    cat <<'EOF'
Usage:
  bash docker/run.sh [options] [command...]

Options:
  --remote-viz                 Also bind CycloneDDS to the Wi-Fi interface so
                               an external laptop can inspect the ROS graph.
  --remote-viz-iface IFACE     Wi-Fi/interface name used with --remote-viz
                               (default: wlan0 or NAV_FRONTIER_REMOTE_IFACE).
  --robot-iface IFACE          Robot/internal DDS interface (default: eth0 or
                               NAV_FRONTIER_ROBOT_IFACE).
  -h, --help                   Show this help.

Environment:
  ROS_DOMAIN_ID                Forwarded into the container, default 0.
  CYCLONEDDS_URI               If set, used verbatim and option-generated DDS
                               profiles are skipped.
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --remote-viz)
            REMOTE_VIZ=true
            shift
            ;;
        --remote-viz-iface)
            if [ "$#" -lt 2 ]; then
                echo "--remote-viz-iface requires an interface name." >&2
                exit 2
            fi
            REMOTE_VIZ_IFACE="$2"
            shift 2
            ;;
        --robot-iface)
            if [ "$#" -lt 2 ]; then
                echo "--robot-iface requires an interface name." >&2
                exit 2
            fi
            ROBOT_IFACE="$2"
            ROBOT_IFACE_OVERRIDDEN=true
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            break
            ;;
        *)
            break
            ;;
    esac
done

build_cyclonedds_uri() {
    local interfaces
    interfaces="<NetworkInterface name=\"${ROBOT_IFACE}\" priority=\"1\" multicast=\"true\" />"

    if [ "$REMOTE_VIZ" = "true" ]; then
        if [ -d "/sys/class/net/${REMOTE_VIZ_IFACE}" ]; then
            interfaces="${interfaces}<NetworkInterface name=\"${REMOTE_VIZ_IFACE}\" priority=\"2\" multicast=\"true\" />"
            echo "Remote visualization DDS enabled on ${REMOTE_VIZ_IFACE}." >&2
        else
            echo "Remote visualization requested, but ${REMOTE_VIZ_IFACE} was not found; using ${ROBOT_IFACE} only." >&2
        fi
    fi

    printf '<CycloneDDS><Domain><General><Interfaces>%s</Interfaces></General></Domain></CycloneDDS>' "$interfaces"
}

should_generate_cyclonedds_uri() {
    [ "$REMOTE_VIZ" = "true" ] || [ "$ROBOT_IFACE_OVERRIDDEN" = "true" ]
}

if [ -n "${CYCLONEDDS_URI:-}" ]; then
    DDS_URI="$CYCLONEDDS_URI"
elif should_generate_cyclonedds_uri; then
    DDS_URI="$(build_cyclonedds_uri)"
else
    DDS_URI="file:///etc/cyclonedds.xml"
fi

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
    --env="CYCLONEDDS_URI=$DDS_URI" \
    --env="ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$REPO_ROOT:/external:rw" \
    --name="${NAV_FRONTIER_CONTAINER_NAME:-nav-frontier-go2w-v3}" \
    "$IMAGE" \
    "${CMD[@]}"
