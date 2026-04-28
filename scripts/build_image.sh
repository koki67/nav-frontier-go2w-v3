#!/bin/bash
# Build the nav-frontier-go2w-v2 docker image (ARM64 robot target).
# On an x86 dev host this requires QEMU + buildx for cross-build; on the
# Jetson itself a plain `docker build` is sufficient.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

IMAGE="${NAV_FRONTIER_IMAGE:-nav-frontier-go2w-v2:latest}"
DOCKERFILE="$REPO_ROOT/docker/Dockerfile"

cd "$REPO_ROOT"

if docker buildx version >/dev/null 2>&1 && [ "$(uname -m)" != "aarch64" ]; then
    echo "Cross-building $IMAGE for linux/arm64 with buildx..."
    docker buildx build \
        --platform linux/arm64 \
        --load \
        -f "$DOCKERFILE" \
        -t "$IMAGE" \
        .
else
    echo "Building $IMAGE natively..."
    docker build \
        -f "$DOCKERFILE" \
        -t "$IMAGE" \
        .
fi
