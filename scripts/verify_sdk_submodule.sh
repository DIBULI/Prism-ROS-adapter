#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_DISTRIBUTION_VERSION="1.0.0"
SDK_RUNTIME_VERSION="1.0.0"
SDK_ROOT="${PRISM_USB_SDK_ROOT:-${ROOT_DIR}/third_party/Prism-SDK}"
CHECKSUM_FILE="${SDK_ROOT}/SHA256SUMS"

SUPPORTED_PLATFORMS=(
  linux-x64
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
  linux-x64  linux-arm64

ROS distributions:
  noetic  foxy  humble  jazzy  kilted  lyrical  rolling

ROS distribution names select linux-arm64 on an arm64 host and the matching
Ubuntu x86_64 prefix on an x86_64 host. Set PRISM_ROS_ARCH to override host
architecture detection. Every supported ROS distribution uses the same prefix
for a given architecture.
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
    linux-x64)
      printf '%s\n' linux-x64
      ;;
    linux-arm64)
      printf '%s\n' linux-arm64
      ;;
    noetic|foxy|humble|jazzy|kilted|lyrical|rolling)
      if [[ "${SDK_ARCH}" == arm64 ]]; then
        printf '%s\n' linux-arm64
      else
        printf '%s\n' linux-x64
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

  if ! grep -Fq \
      "project(PrismSdkExamples VERSION ${SDK_DISTRIBUTION_VERSION}" \
      "${SDK_ROOT}/CMakeLists.txt"; then
    echo "Prism SDK submodule is not distribution ${SDK_DISTRIBUTION_VERSION}: ${SDK_ROOT}" >&2
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
  local dynamic_entries object_headers
  local -a expected_prefix_files=("${EXPECTED_PREFIX_FILES[@]}")

  if [[ "${platform}" == linux-arm64 ]]; then
    library=lib/libprism_usb_sdk.a
  else
    library=lib/libprism_usb_sdk.so
    expected_prefix_files+=(share/licenses/PrismUsbSdk/openssl-LICENSE.txt)
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

  if ! grep -Fqx "set(PACKAGE_VERSION \"${SDK_RUNTIME_VERSION}\")" \
      "${prefix}/lib/cmake/PrismUsbSdk/PrismUsbSdkConfigVersion.cmake"; then
    echo "SDK runtime prefix is not version ${SDK_RUNTIME_VERSION}: ${platform}" >&2
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
  else
    dynamic_entries="$(readelf -d "${prefix}/${library}")"
    if ! grep -Fq 'Shared library: [libusb-1.0.so.0]' \
        <<<"${dynamic_entries}"; then
      echo "Unified x86_64 SDK does not depend on libusb-1.0.so.0" >&2
      return 1
    fi
    if grep -Eq 'Shared library: \[lib(ssl|crypto)\.so' \
        <<<"${dynamic_entries}"; then
      echo "Unified x86_64 SDK must not depend on dynamic OpenSSL" >&2
      return 1
    fi
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

  echo "Verified Prism SDK distribution ${SDK_DISTRIBUTION_VERSION}, runtime ${SDK_RUNTIME_VERSION}: ${platform}"
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
