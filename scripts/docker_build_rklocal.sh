#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_ROOT="${PRISM_RKLOCAL_SDK_ROOT:-${ROOT_DIR}/third_party/Prism-SDK}"
DISTRO="${1:-humble}"
case "$DISTRO" in
  22|22.04) DISTRO=humble ;;
  24|24.04) DISTRO=jazzy ;;
  humble|jazzy) ;;
  *) echo "usage: $0 [22.04|24.04|humble|jazzy]" >&2; exit 2 ;;
esac
for path in include/prism/rklocal_sdk.hpp cmake/PrismRkLocalSdk.cmake \
    runtime/linux-arm64/libprism_rklocal_sdk.a; do
  test -f "$SDK_ROOT/$path" || {
    echo "Missing SDK file: $SDK_ROOT/$path (git submodule update --init --recursive)" >&2
    exit 1
  }
done
# A native ARM64 machine or registered AArch64 binfmt/QEMU is required.
# ROS_BASE may name a pre-pulled ARM64 image for an offline build.
ROS_BASE="${PRISM_ROS_BASE:-ros:${DISTRO}-ros-base}"
exec docker build --platform linux/arm64 \
  --build-context "prism_sdk=${SDK_ROOT}" \
  --build-arg "ROS_DISTRO=${DISTRO}" --build-arg "ROS_BASE=${ROS_BASE}" \
  -f "${ROOT_DIR}/docker/ros2-rklocal.Dockerfile" \
  -t "prism-ros-adapter:${DISTRO}-rklocal" "${ROOT_DIR}"
