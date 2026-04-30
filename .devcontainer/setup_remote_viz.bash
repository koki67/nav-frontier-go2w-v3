#!/usr/bin/env bash
# Source inside the desktop devcontainer:
#   source .devcontainer/setup_remote_viz.bash [wifi_iface]

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "This script must be sourced so it can export DDS environment variables." >&2
    exit 2
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

iface="${1:-${NAV_FRONTIER_DESKTOP_IFACE:-}}"
if [ -z "$iface" ]; then
    unset CYCLONEDDS_URI
    echo "CycloneDDS will auto-select network interfaces. ROS_DOMAIN_ID=${ROS_DOMAIN_ID}."
    return 0
fi

if [[ ! "$iface" =~ ^[A-Za-z0-9_.:-]+$ ]]; then
    echo "Invalid interface name: ${iface}" >&2
    return 2
fi

if [ ! -d "/sys/class/net/${iface}" ]; then
    echo "Warning: ${iface} was not found in /sys/class/net." >&2
fi

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${iface}\" priority=\"1\" multicast=\"true\" /></Interfaces></General></Domain></CycloneDDS>"
echo "CycloneDDS bound to ${iface}. ROS_DOMAIN_ID=${ROS_DOMAIN_ID}."
