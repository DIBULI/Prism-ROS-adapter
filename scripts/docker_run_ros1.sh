#!/usr/bin/env bash
set -euo pipefail

exec docker run --rm --privileged --network host \
  -v /dev/bus/usb:/dev/bus/usb \
  prism-ros-adapter:noetic "$@"
