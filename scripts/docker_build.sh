#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_VERSION="1.0.0"
SDK_REPOSITORY_ROOT="${PRISM_USB_SDK_ROOT:-${ROOT_DIR}/third_party/Prism-SDK}"
SDK_PREFIX_ROOT="${SDK_REPOSITORY_ROOT}/runtime/ros"
SDK_PREFIX_OVERRIDE="${PRISM_USB_SDK_PREFIX:-}"
IMAGE_MIRROR="${PRISM_DOCKER_MIRROR:-docker.1ms.run/library}"
IMAGE_MIRROR_FALLBACK="${PRISM_DOCKER_MIRROR_FALLBACK:-docker.m.daocloud.io/library}"
PULL_TIMEOUT_SECONDS="${PRISM_DOCKER_PULL_TIMEOUT_SECONDS:-900}"

normalize_arch() {
  case "$1" in
    x86_64|amd64|x64) echo x86_64 ;;
    aarch64|arm64) echo arm64 ;;
    *)
      echo "unsupported host architecture: $1" >&2
      return 2
      ;;
  esac
}

SDK_ARCH="$(normalize_arch "${PRISM_ROS_ARCH:-$(uname -m)}")"
if [[ "${SDK_ARCH}" == arm64 ]]; then
  DOCKER_PLATFORM=linux/arm64
else
  DOCKER_PLATFORM=linux/amd64
fi

sdk_platform_for_distro() {
  if [[ "${SDK_ARCH}" == arm64 ]]; then
    echo linux-arm64
    return
  fi
  case "$1" in
    noetic) echo "ubuntu-20.04-x86_64" ;;
    humble) echo "ubuntu-22.04-x86_64" ;;
    jazzy|kilted) echo "ubuntu-24.04-x86_64" ;;
    lyrical|rolling) echo "ubuntu-26.04-x86_64" ;;
    *)
      echo "unsupported ROS distribution: $1" >&2
      return 2
      ;;
  esac
}

validate_sdk_root() {
  local sdk_root="$1"
  local required_path
  local forbidden_path
  local library_description
  local library
  local object_headers
  local -a required_paths=(
    "include/prism/usb_sdk.hpp"
    "include/prism/usb/client.hpp"
    "include/prism/usb/common.hpp"
    "include/prism/usb/configuration.hpp"
    "include/prism/usb/device_info.hpp"
    "include/prism/usb/exposure.hpp"
    "include/prism/usb/runtime_api.hpp"
    "include/prism/usb/streams.hpp"
    "include/prism/usb/telemetry.hpp"
    "include/prism/usb/time_sync.hpp"
    "include/prism/usb/update.hpp"
    "include/prism/usb/wifi.hpp"
    "lib/cmake/PrismUsbSdk/PrismUsbSdkConfig.cmake"
    "lib/cmake/PrismUsbSdk/PrismUsbSdkConfigVersion.cmake"
    "lib/cmake/PrismUsbSdk/PrismUsbSdkTargets.cmake"
    "lib/cmake/PrismUsbSdk/PrismUsbSdkTargets-release.cmake"
    "lib/udev/rules.d/99-prism-usb.rules"
  )

  if [[ "${SDK_ARCH}" == arm64 ]]; then
    library=lib/libprism_usb_sdk.a
  else
    library=lib/libprism_usb_sdk.so
  fi
  required_paths+=("${library}")

  if [[ ! -d "${sdk_root}" ]]; then
    echo "Prism USB SDK binary prefix not found: ${sdk_root}" >&2
    return 1
  fi

  for required_path in "${required_paths[@]}"; do
    if [[ ! -e "${sdk_root}/${required_path}" ]]; then
      echo "Prism USB SDK binary prefix is missing ${required_path}: ${sdk_root}" >&2
      return 1
    fi
  done

  if ! grep -Fq "${SDK_VERSION}" \
      "${sdk_root}/lib/cmake/PrismUsbSdk/PrismUsbSdkConfigVersion.cmake"; then
    echo "Prism USB SDK binary prefix is not version ${SDK_VERSION}: ${sdk_root}" >&2
    return 1
  fi

  forbidden_path="$(find "${sdk_root}" \( -type f -o -type l \) \( \
      -name 'CMakeLists.txt' -o \
      -name '*.c' -o -name '*.C' -o -name '*.cc' -o -name '*.cp' -o \
      -name '*.cpp' -o -name '*.cxx' -o -name '*.c++' -o \
      -name '*.ixx' -o -name '*.cppm' -o -name '*.cu' -o \
      -name '*.m' -o -name '*.mm' \
    \) -print -quit)"
  if [[ -n "${forbidden_path}" ]]; then
    echo "SDK source files are not allowed in the binary prefix: ${forbidden_path}" >&2
    return 1
  fi

  library_description="$(file -b "${sdk_root}/${library}")"
  if [[ "${SDK_ARCH}" == arm64 ]]; then
    object_headers="$(readelf -h "${sdk_root}/${library}")"
    if [[ "${library_description}" != *"current ar archive"* ]] ||
        ! grep -Eq 'Machine:[[:space:]]+AArch64' <<<"${object_headers}"; then
      echo "libprism_usb_sdk.a is not an arm64 static archive: ${library_description}" >&2
      return 1
    fi
  elif [[ "${library_description}" != *ELF* ||
          "${library_description}" != *"shared object"* ]]; then
    echo "libprism_usb_sdk.so is not an ELF dynamic library: ${library_description}" >&2
    return 1
  fi
}

sdk_root_for_distro() {
  local distro="$1"
  local sdk_root

  if [[ -n "${SDK_PREFIX_OVERRIDE}" ]]; then
    sdk_root="${SDK_PREFIX_OVERRIDE}"
  else
    sdk_root="${SDK_PREFIX_ROOT}/$(sdk_platform_for_distro "${distro}")"
  fi

  validate_sdk_root "${sdk_root}"
  (cd "${sdk_root}" && pwd -P)
}

pull_ros_image() {
  local tag="$1"
  local mirrored="${IMAGE_MIRROR}/ros:${tag}"
  local pulled="${mirrored}"
  if ! timeout "${PULL_TIMEOUT_SECONDS}" docker pull \
      --platform "${DOCKER_PLATFORM}" "${mirrored}"; then
    pulled="${IMAGE_MIRROR_FALLBACK}/ros:${tag}"
    echo "Primary mirror failed; trying ${pulled}" >&2
    docker pull --platform "${DOCKER_PLATFORM}" "${pulled}"
  fi
  docker tag "${pulled}" "ros:${tag}"
}

build_ros1() {
  local sdk_root
  sdk_root="$(sdk_root_for_distro noetic)"
  pull_ros_image noetic-ros-base
  docker build \
    --platform "${DOCKER_PLATFORM}" \
    --build-context "prism_sdk=${sdk_root}" \
    -f "${ROOT_DIR}/docker/ros1-noetic.Dockerfile" \
    -t prism-ros-adapter:noetic "${ROOT_DIR}"
}

build_ros2() {
  local distro="$1"
  local sdk_root
  sdk_root="$(sdk_root_for_distro "${distro}")"
  pull_ros_image "${distro}-ros-base"
  docker build \
    --platform "${DOCKER_PLATFORM}" \
    --build-context "prism_sdk=${sdk_root}" \
    --build-arg "ROS_DISTRO=${distro}" \
    -f "${ROOT_DIR}/docker/ros2.Dockerfile" \
    -t "prism-ros-adapter:${distro}" "${ROOT_DIR}"
}

TARGET="${1:-all}"
if [[ "${TARGET}" == "all" && -n "${SDK_PREFIX_OVERRIDE}" ]]; then
  echo "PRISM_USB_SDK_PREFIX names one binary prefix; select one ROS distribution instead of 'all'" >&2
  exit 2
fi

case "${TARGET}" in
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
