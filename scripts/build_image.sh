#!/bin/bash
# Build the nav-frontier-go2w-v3 docker image (ARM64 robot target).
# On an x86 dev host this requires QEMU + buildx for cross-build; on the
# Jetson itself a plain `docker build` is sufficient.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

IMAGE="${NAV_FRONTIER_IMAGE:-nav-frontier-go2w-v3:latest}"
PLATFORM="${NAV_FRONTIER_PLATFORM:-linux/arm64}"
DOCKERFILE="$REPO_ROOT/docker/Dockerfile"

cd "$REPO_ROOT"

if docker buildx version >/dev/null 2>&1; then
    if [ "$PLATFORM" = "linux/arm64" ] && [ "$(uname -m)" != "aarch64" ] &&
       [ ! -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ]; then
        cat >&2 <<EOF
ARM64 cross-build requires QEMU/binfmt on this host.
Install it with:
  docker run --privileged --rm tonistiigi/binfmt --install arm64

Or build on the Go2W/aarch64 target directly. For an amd64 Dockerfile smoke
build on this host, run:
  NAV_FRONTIER_PLATFORM=linux/amd64 bash scripts/build_image.sh
EOF
        exit 1
    fi

    echo "Building $IMAGE for $PLATFORM with buildx..."
    docker buildx build \
        --platform "$PLATFORM" \
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
