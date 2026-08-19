#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_PREFIX="${1:-/opt/prism-sdk}"
INSTALL_PREFIX="${2:-${HOME}/.local/prism-ros/noetic}"

source /opt/ros/noetic/setup.bash
cd "${ROOT_DIR}/ros1_ws"
catkin_make \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DPrismUsbSdk_DIR="${SDK_PREFIX}/lib/cmake/PrismUsbSdk"
catkin_make install \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DPrismUsbSdk_DIR="${SDK_PREFIX}/lib/cmake/PrismUsbSdk"

echo "Installed. Run: source ${INSTALL_PREFIX}/setup.bash"
