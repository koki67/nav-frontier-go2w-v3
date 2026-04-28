#!/bin/bash
# Open an extra shell inside an already-running nav-frontier-go2w-v2 container.
# Use docker/run.sh first to start the primary container.
set -e

CONTAINER="${NAV_FRONTIER_CONTAINER_NAME:-nav-frontier-go2w-v2}"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "Container '$CONTAINER' is not running. Start it with docker/run.sh first." >&2
    exit 1
fi

exec docker exec -it "$CONTAINER" bash
