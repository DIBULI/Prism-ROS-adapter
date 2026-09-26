#!/usr/bin/env bash
set -euo pipefail
DISTRO="${1:-humble}"
shift || true
case "$DISTRO" in
  22|22.04) DISTRO=humble ;;
  24|24.04) DISTRO=jazzy ;;
  humble|jazzy) ;;
  *) echo "usage: $0 [22.04|24.04|humble|jazzy] [command ...]" >&2; exit 2 ;;
esac
SOCKET_DIR="${PRISM_SOCKET_DIR:-/run/prism}"
if [[ "$SOCKET_DIR" != /* || ! -S "$SOCKET_DIR/stream.sock" ]]; then
  echo "Agent socket not found: $SOCKET_DIR/stream.sock. Run on RK with Agent running." >&2
  exit 1
fi
prism_ros_image="prism-ros-adapter:${DISTRO}-rklocal"
if ! docker image inspect "$prism_ros_image" >/dev/null 2>&1; then
  echo "Docker image not available: $prism_ros_image. Build on the server, then use docker load on RK." >&2
  exit 1
fi
# Mount the directory, so replacing the socket on Agent restart still works.
# No privileged mode, USB device, host PID namespace or clock capability needed.
exec docker run --pull never --rm --init --network host --ipc host \
  --cap-drop ALL --security-opt no-new-privileges \
  --mount "type=bind,src=${SOCKET_DIR},dst=/run/prism,readonly" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
  "$prism_ros_image" "$@"
