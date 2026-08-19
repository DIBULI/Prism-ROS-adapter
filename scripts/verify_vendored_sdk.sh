#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_VERSION="0.11.0"
SDK_ROOT="${ROOT_DIR}/vendor/prism-usb-sdk/${SDK_VERSION}"
CHECKSUM_FILE="${SDK_ROOT}/SHA256SUMS"

SUPPORTED_PLATFORMS=(
  ubuntu-20.04-x86_64
  ubuntu-22.04-x86_64
  ubuntu-24.04-x86_64
  ubuntu-26.04-x86_64
)

EXPECTED_FILES=(
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
  lib/libprism_usb_sdk.so
  lib/udev/rules.d/99-prism-usb.rules
)

usage() {
  cat >&2 <<'EOF'
usage: scripts/verify_vendored_sdk.sh [all|PLATFORM|ROS_DISTRO]

Platforms:
  ubuntu-20.04-x86_64  ubuntu-22.04-x86_64
  ubuntu-24.04-x86_64  ubuntu-26.04-x86_64

ROS distributions:
  noetic  humble  jazzy  kilted  lyrical  rolling
EOF
}

platform_for() {
  case "$1" in
    noetic|ubuntu-20.04-x86_64)
      printf '%s\n' ubuntu-20.04-x86_64
      ;;
    humble|ubuntu-22.04-x86_64)
      printf '%s\n' ubuntu-22.04-x86_64
      ;;
    jazzy|kilted|ubuntu-24.04-x86_64)
      printf '%s\n' ubuntu-24.04-x86_64
      ;;
    lyrical|rolling|ubuntu-26.04-x86_64)
      printf '%s\n' ubuntu-26.04-x86_64
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
      if (file == path) {
        print $1
      }
    }
  ' "${CHECKSUM_FILE}"
}

verify_checksum_manifest_boundary() {
  local line checksum relative_path platform remainder
  local expected_platform expected_file
  local -a manifest_paths=()
  local -a expected_paths=()

  for expected_platform in "${SUPPORTED_PLATFORMS[@]}"; do
    for expected_file in "${EXPECTED_FILES[@]}"; do
      expected_paths+=("${expected_platform}/${expected_file}")
    done
  done

  while IFS= read -r line || [[ -n "${line}" ]]; do
    [[ "${line}" =~ ^[[:space:]]*$ || "${line}" =~ ^[[:space:]]*# ]] && continue

    checksum="${line%%[[:space:]]*}"
    remainder="${line#"${checksum}"}"
    remainder="${remainder#"${remainder%%[![:space:]]*}"}"
    relative_path="${remainder#\*}"

    if [[ ! "${checksum}" =~ ^[0-9a-fA-F]{64}$ || -z "${relative_path}" ]]; then
      echo "Invalid SHA256SUMS entry: ${line}" >&2
      return 1
    fi
    if [[ "${relative_path}" == /* || "${relative_path}" == *".."* ]]; then
      echo "Unsafe SHA256SUMS path: ${relative_path}" >&2
      return 1
    fi

    platform="${relative_path%%/*}"
    remainder="${relative_path#*/}"
    if ! platform_for "${platform}" >/dev/null 2>&1; then
      echo "Unsupported platform in SHA256SUMS: ${relative_path}" >&2
      return 1
    fi
    if ! printf '%s\n' "${EXPECTED_FILES[@]}" | grep -Fqx -- "${remainder}"; then
      echo "Non-public file in SHA256SUMS: ${relative_path}" >&2
      return 1
    fi
    manifest_paths+=("${relative_path}")
  done < "${CHECKSUM_FILE}"

  if ! diff -u \
      <(printf '%s\n' "${expected_paths[@]}" | LC_ALL=C sort) \
      <(printf '%s\n' "${manifest_paths[@]}" | LC_ALL=C sort); then
    echo "SHA256SUMS must contain exactly one entry for every vendored payload file" >&2
    return 1
  fi
}

verify_platform() {
  local platform="$1"
  local prefix="${SDK_ROOT}/${platform}"
  local library="${prefix}/lib/libprism_usb_sdk.so"
  local version_file="${prefix}/lib/cmake/PrismUsbSdk/PrismUsbSdkConfigVersion.cmake"
  local relative_library="${platform}/lib/libprism_usb_sdk.so"
  local expected_checksum actual_checksum checksum_count file_description
  local line checksum relative_path checksum_platform remainder matched_checksums

  if [[ ! -d "${prefix}" ]]; then
    echo "Missing vendored SDK prefix: ${prefix}" >&2
    return 1
  fi

  if find "${prefix}" -type l -print -quit | grep -q .; then
    echo "Symlinks are not allowed in vendored SDK prefix: ${platform}" >&2
    return 1
  fi

  if find "${prefix}" -type f \( \
      -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' -o \
      -name '*.m' -o -name '*.mm' -o -name 'CMakeLists.txt' \) \
      -print -quit | grep -q .; then
    echo "SDK implementation source is not allowed in vendored prefix: ${platform}" >&2
    return 1
  fi

  if ! diff -u \
      <(printf '%s\n' "${EXPECTED_FILES[@]}" | LC_ALL=C sort) \
      <(cd "${prefix}" && find . -type f -print | sed 's#^\./##' | LC_ALL=C sort); then
    echo "Vendored SDK public-file boundary mismatch: ${platform}" >&2
    return 1
  fi

  if ! grep -Fqx "set(PACKAGE_VERSION \"${SDK_VERSION}\")" "${version_file}"; then
    echo "Vendored SDK version is not ${SDK_VERSION}: ${platform}" >&2
    return 1
  fi

  file_description="$(file -b "${library}")"
  if [[ "${file_description}" != *"ELF 64-bit"* || \
        "${file_description}" != *"shared object"* || \
        ( "${file_description}" != *"x86-64"* && "${file_description}" != *"x86_64"* ) ]]; then
    echo "Vendored SDK is not an x86_64 ELF shared library: ${platform}" >&2
    echo "file: ${file_description}" >&2
    return 1
  fi

  checksum_count="$(checksum_for "${relative_library}" | wc -l | tr -d '[:space:]')"
  if [[ "${checksum_count}" != 1 ]]; then
    echo "Expected one SHA256SUMS entry for ${relative_library}, got ${checksum_count}" >&2
    return 1
  fi

  matched_checksums=0
  while IFS= read -r line || [[ -n "${line}" ]]; do
    [[ "${line}" =~ ^[[:space:]]*$ || "${line}" =~ ^[[:space:]]*# ]] && continue

    checksum="${line%%[[:space:]]*}"
    remainder="${line#"${checksum}"}"
    remainder="${remainder#"${remainder%%[![:space:]]*}"}"
    relative_path="${remainder#\*}"
    checksum_platform="${relative_path%%/*}"
    [[ "${checksum_platform}" == "${platform}" ]] || continue

    checksum="$(printf '%s' "${checksum}" | tr '[:upper:]' '[:lower:]')"
    actual_checksum="$(sha256_file "${SDK_ROOT}/${relative_path}" | tr '[:upper:]' '[:lower:]')"
    if [[ "${actual_checksum}" != "${checksum}" ]]; then
      echo "SHA-256 mismatch for ${relative_path}" >&2
      echo "expected: ${checksum}" >&2
      echo "actual:   ${actual_checksum}" >&2
      return 1
    fi
    matched_checksums=$((matched_checksums + 1))
  done < "${CHECKSUM_FILE}"
  if [[ "${matched_checksums}" -eq 0 ]]; then
    echo "SHA256SUMS has no entries for ${platform}" >&2
    return 1
  fi

  expected_checksum="$(checksum_for "${relative_library}" | tr '[:upper:]' '[:lower:]')"
  actual_checksum="$(sha256_file "${library}" | tr '[:upper:]' '[:lower:]')"
  if [[ "${actual_checksum}" != "${expected_checksum}" ]]; then
    echo "SHA-256 mismatch for ${relative_library}" >&2
    echo "expected: ${expected_checksum}" >&2
    echo "actual:   ${actual_checksum}" >&2
    return 1
  fi

  echo "Verified Prism USB SDK ${SDK_VERSION}: ${platform}"
}

selection="${1:-all}"
if [[ "$#" -gt 1 ]]; then
  usage
  exit 2
fi
if [[ ! -f "${CHECKSUM_FILE}" ]]; then
  echo "Missing checksum manifest: ${CHECKSUM_FILE}" >&2
  exit 1
fi

verify_checksum_manifest_boundary

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
