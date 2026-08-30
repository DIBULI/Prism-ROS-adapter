#!/usr/bin/env bash
set -eo pipefail

if [[ -z "${ROS_DISTRO:-}" || ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "Source a supported ROS 2 installation first." >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_PREFIX="${1:-/opt/prism-sdk}"
INSTALL_PREFIX="${2:-${HOME}/.local/prism-ros/${ROS_DISTRO}}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
cd "${ROOT_DIR}/ros2_ws"
colcon build --merge-install \
  --install-base "${INSTALL_PREFIX}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DPrismUsbSdk_DIR="${SDK_PREFIX}/lib/cmake/PrismUsbSdk"

echo "Installed. Run:"
echo "  source ${INSTALL_PREFIX}/setup.bash"
echo "  export AMENT_PREFIX_PATH=${INSTALL_PREFIX}:\${AMENT_PREFIX_PATH}"
