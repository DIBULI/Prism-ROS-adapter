#!/usr/bin/env bash
set -euo pipefail

DISTRO="${1:-jazzy}"
shift || true
exec docker run --rm --privileged --network host --ipc host \
  -v /dev/bus/usb:/dev/bus/usb \
  "prism-ros-adapter:${DISTRO}" "$@"
