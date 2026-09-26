#!/usr/bin/env bash
# Run only when no other client is capturing. No persistent settings/time writes.
set -euo pipefail
probe=${1:?usage: run_capture_probe.sh PROBE_BINARY [SECONDS]}
seconds=${2:-60}
test -x "$probe"
/opt/prism-ros2/lib/prism_ros_driver/prism_ros_driver_node --ros-args \
  -p camera_fps:=30 &
prism_probe_driver_pid=$!
cleanup() {
  kill -INT "$prism_probe_driver_pid" 2>/dev/null || true
  wait "$prism_probe_driver_pid" || true
}
trap cleanup EXIT INT TERM
"$probe" "$seconds"
kill -0 "$prism_probe_driver_pid"
