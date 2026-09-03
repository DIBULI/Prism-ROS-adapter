#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_VERSION="1.0.0"
SDK_ROOT="${PRISM_USB_SDK_ROOT:-${ROOT_DIR}/third_party/Prism-SDK}"
CHECKSUM_FILE="${SDK_ROOT}/SHA256SUMS"

SUPPORTED_PLATFORMS=(
  ubuntu-20.04-x86_64
  ubuntu-22.04-x86_64
  ubuntu-24.04-x86_64
  ubuntu-26.04-x86_64
  linux-arm64
)

EXPECTED_PREFIX_FILES=(
  include/prism/usb/client.hpp
  include/prism/usb/common.hpp
  include/prism/usb/configuration.hpp
  include/prism/usb/device_info.hpp
  include/prism/usb/exposure.hpp
  include/prism/usb/runtime_api.hpp
  include/prism/usb/streams.hpp
  include/prism/usb/telemetry.hpp
  include/prism/usb/time_sync.hpp
  include/prism/usb/update.hpp
  include/prism/usb/wifi.hpp
  include/prism/usb_sdk.hpp
  lib/cmake/PrismUsbSdk/PrismUsbSdkConfig.cmake
  lib/cmake/PrismUsbSdk/PrismUsbSdkConfigVersion.cmake
  lib/cmake/PrismUsbSdk/PrismUsbSdkTargets-release.cmake
  lib/cmake/PrismUsbSdk/PrismUsbSdkTargets.cmake
  lib/udev/rules.d/99-prism-usb.rules
)

usage() {
  cat >&2 <<'EOF'
usage: scripts/verify_sdk_submodule.sh [all|PLATFORM|ROS_DISTRO]

Platforms:
  ubuntu-20.04-x86_64  ubuntu-22.04-x86_64
  ubuntu-24.04-x86_64  ubuntu-26.04-x86_64
  linux-arm64

ROS distributions:
  noetic  humble  jazzy  kilted  lyrical  rolling

ROS distribution names select linux-arm64 on an arm64 host and the matching
Ubuntu x86_64 prefix on an x86_64 host. Set PRISM_ROS_ARCH to override host
architecture detection.
EOF
}

normalize_arch() {
  case "$1" in
    x86_64|amd64|x64)
      printf '%s\n' x86_64
      ;;
    aarch64|arm64)
      printf '%s\n' arm64
      ;;
    *)
      echo "unsupported host architecture: $1" >&2
      return 2
      ;;
  esac
}

SDK_ARCH="$(normalize_arch "${PRISM_ROS_ARCH:-$(uname -m)}")"

platform_for() {
  case "$1" in
    ubuntu-20.04-x86_64)
      printf '%s\n' ubuntu-20.04-x86_64
      ;;
    ubuntu-22.04-x86_64)
      printf '%s\n' ubuntu-22.04-x86_64
      ;;
    ubuntu-24.04-x86_64)
      printf '%s\n' ubuntu-24.04-x86_64
      ;;
    ubuntu-26.04-x86_64)
      printf '%s\n' ubuntu-26.04-x86_64
      ;;
    linux-arm64)
      printf '%s\n' linux-arm64
      ;;
    noetic|humble|jazzy|kilted|lyrical|rolling)
      if [[ "${SDK_ARCH}" == arm64 ]]; then
        printf '%s\n' linux-arm64
      else
        case "$1" in
          noetic) printf '%s\n' ubuntu-20.04-x86_64 ;;
          humble) printf '%s\n' ubuntu-22.04-x86_64 ;;
          jazzy|kilted) printf '%s\n' ubuntu-24.04-x86_64 ;;
          lyrical|rolling) printf '%s\n' ubuntu-26.04-x86_64 ;;
        esac
      fi
      ;;
    *)
      return 1
      ;;
  esac
}

sha256_file() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    echo "sha256sum or shasum is required" >&2
    return 1
  fi
}

checksum_for() {
  local relative_path="$1"
  awk -v path="${relative_path}" '
    /^[[:space:]]*#/ || /^[[:space:]]*$/ { next }
    {
      file = $2
      sub(/^\*/, "", file)
      sub(/^\.\//, "", file)
      if (file == path) {
        print $1
      }
    }
  ' "${CHECKSUM_FILE}"
}

verify_sdk_repository() {
  if [[ ! -f "${SDK_ROOT}/include/prism/usb_sdk.hpp" ||
        ! -f "${CHECKSUM_FILE}" ]]; then
    echo "Prism SDK submodule is missing or uninitialized: ${SDK_ROOT}" >&2
    echo "Run: git submodule update --init --recursive" >&2
    return 1
  fi

  if ! grep -Fq 'project(PrismSdkExamples VERSION 1.0.0' \
      "${SDK_ROOT}/CMakeLists.txt"; then
    echo "Prism SDK submodule is not version ${SDK_VERSION}: ${SDK_ROOT}" >&2
    return 1
  fi
  if ! grep -Fq 'constexpr uint8_t kProtocolVersion = 1;' \
      "${SDK_ROOT}/include/prism/usb/common.hpp"; then
    echo "Prism SDK submodule does not use USB protocol 1" >&2
    return 1
  fi
}

verify_platform() {
  local platform="$1"
  local prefix="${SDK_ROOT}/runtime/ros/${platform}"
  local file relative_path expected actual count description library
  local object_headers
  local -a expected_prefix_files=("${EXPECTED_PREFIX_FILES[@]}")

  if [[ "${platform}" == linux-arm64 ]]; then
    library=lib/libprism_usb_sdk.a
  else
    library=lib/libprism_usb_sdk.so
  fi
  expected_prefix_files+=("${library}")

  if [[ ! -d "${prefix}" ]]; then
    echo "Missing SDK submodule runtime prefix: ${prefix}" >&2
    return 1
  fi
  if find "${prefix}" -type l -print -quit | grep -q .; then
    echo "Symlinks are not allowed in SDK runtime prefix: ${platform}" >&2
    return 1
  fi
  if find "${prefix}" -type f \( \
      -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' -o \
      -name '*.m' -o -name '*.mm' -o -name 'CMakeLists.txt' \) \
      -print -quit | grep -q .; then
    echo "SDK implementation source is not allowed in runtime prefix: ${platform}" >&2
    return 1
  fi
  if ! diff -u \
      <(printf '%s\n' "${expected_prefix_files[@]}" | LC_ALL=C sort) \
      <(cd "${prefix}" && find . -type f -print | sed 's#^\./##' | LC_ALL=C sort); then
    echo "SDK runtime prefix boundary mismatch: ${platform}" >&2
    return 1
  fi

  if ! grep -Fqx "set(PACKAGE_VERSION \"${SDK_VERSION}\")" \
      "${prefix}/lib/cmake/PrismUsbSdk/PrismUsbSdkConfigVersion.cmake"; then
    echo "SDK runtime prefix is not version ${SDK_VERSION}: ${platform}" >&2
    return 1
  fi

  description="$(file -b "${prefix}/${library}")"
  if [[ "${platform}" == linux-arm64 ]]; then
    object_headers="$(readelf -h "${prefix}/${library}")"
    if [[ "${description}" != *"current ar archive"* ]] ||
        ! grep -Eq 'Machine:[[:space:]]+AArch64' <<<"${object_headers}"; then
      echo "SDK runtime is not an arm64 static archive: ${description}" >&2
      return 1
    fi
  elif [[ "${description}" != *"ELF 64-bit"* ||
          "${description}" != *"shared object"* ||
          ( "${description}" != *"x86-64"* && "${description}" != *"x86_64"* ) ]]; then
    echo "SDK runtime is not an x86_64 ELF shared library: ${description}" >&2
    return 1
  fi

  for file in "${expected_prefix_files[@]}"; do
    relative_path="runtime/ros/${platform}/${file}"
    count="$(checksum_for "${relative_path}" | wc -l | tr -d '[:space:]')"
    if [[ "${count}" != 1 ]]; then
      echo "Expected one SHA256SUMS entry for ${relative_path}, got ${count}" >&2
      return 1
    fi
    expected="$(checksum_for "${relative_path}" | tr '[:upper:]' '[:lower:]')"
    actual="$(sha256_file "${SDK_ROOT}/${relative_path}" | tr '[:upper:]' '[:lower:]')"
    if [[ "${actual}" != "${expected}" ]]; then
      echo "SHA-256 mismatch for ${relative_path}" >&2
      echo "expected: ${expected}" >&2
      echo "actual:   ${actual}" >&2
      return 1
    fi
  done

  echo "Verified Prism SDK submodule ${SDK_VERSION}: ${platform}"
}

selection="${1:-all}"
if [[ "$#" -gt 1 ]]; then
  usage
  exit 2
fi

verify_sdk_repository
if [[ "${selection}" == all ]]; then
  for platform in "${SUPPORTED_PLATFORMS[@]}"; do
    verify_platform "${platform}"
  done
else
  if ! platform="$(platform_for "${selection}")"; then
    usage
    exit 2
  fi
  verify_platform "${platform}"
fi
