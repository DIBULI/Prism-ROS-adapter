#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_SOURCE="${PRISM_USB_SDK_SOURCE:-/work/projects/Prism-agent/prism-sdk/usb-sdk}"
IMAGE_MIRROR="${PRISM_DOCKER_MIRROR:-docker.1ms.run/library}"
IMAGE_MIRROR_FALLBACK="${PRISM_DOCKER_MIRROR_FALLBACK:-docker.m.daocloud.io/library}"
PULL_TIMEOUT_SECONDS="${PRISM_DOCKER_PULL_TIMEOUT_SECONDS:-900}"

if [[ ! -f "${SDK_SOURCE}/CMakeLists.txt" ]]; then
  echo "Prism USB SDK source not found: ${SDK_SOURCE}" >&2
  exit 1
fi

pull_ros_image() {
  local tag="$1"
  local mirrored="${IMAGE_MIRROR}/ros:${tag}"
  local pulled="${mirrored}"
  if ! timeout "${PULL_TIMEOUT_SECONDS}" docker pull "${mirrored}"; then
    pulled="${IMAGE_MIRROR_FALLBACK}/ros:${tag}"
    echo "Primary mirror failed; trying ${pulled}" >&2
    docker pull "${pulled}"
  fi
  docker tag "${pulled}" "ros:${tag}"
}

build_ros1() {
  pull_ros_image noetic-ros-base
  docker build \
    --build-context "prism_sdk=${SDK_SOURCE}" \
    -f "${ROOT_DIR}/docker/ros1-noetic.Dockerfile" \
    -t prism-ros-adapter:noetic "${ROOT_DIR}"
}

build_ros2() {
  local distro="$1"
  pull_ros_image "${distro}-ros-base"
  docker build \
    --build-context "prism_sdk=${SDK_SOURCE}" \
    --build-arg "ROS_DISTRO=${distro}" \
    -f "${ROOT_DIR}/docker/ros2.Dockerfile" \
    -t "prism-ros-adapter:${distro}" "${ROOT_DIR}"
}

case "${1:-all}" in
  noetic) build_ros1 ;;
  humble|jazzy|kilted|lyrical|rolling) build_ros2 "$1" ;;
  all)
    build_ros1
    for distro in humble jazzy kilted lyrical rolling; do
      build_ros2 "${distro}"
    done
    ;;
  *)
    echo "usage: $0 [all|noetic|humble|jazzy|kilted|lyrical|rolling]" >&2
    exit 2
    ;;
esac
