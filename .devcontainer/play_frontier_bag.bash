#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <bag_directory> [ros2 bag play args...]" >&2
    echo "Example: $0 /workspaces/nav-frontier-go2w-v3/bags/frontier_results_YYYYMMDD_HHMMSS" >&2
    exit 2
fi

BAG_PATH="$1"
shift

source "${SCRIPT_DIR}/setup_bag_replay.bash"

exec ros2 bag play "${BAG_PATH}" --clock "$@"
