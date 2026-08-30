#!/usr/bin/env bash
set -euo pipefail

DISTRO="${1:-}"

check_linkage() {
  local image="$1"
  local node="$2"
  docker run --rm "${image}" bash -lc "
    set -euo pipefail
    test -x '${node}'
    if ldd '${node}' | grep -q 'not found'; then
      ldd '${node}' >&2
      exit 1
    fi
  "
}

case "${DISTRO}" in
  noetic)
    docker run --rm prism-ros-adapter:noetic bash -lc '
      set -euo pipefail
      rospack find prism_ros_driver >/dev/null
      interface_text="$(rosmsg show prism_ros_msgs/CameraFrameMetadata)"
      grep -q "uint32\[4\] exposure_us" <<<"${interface_text}"
      exposure_service="$(rossrv show prism_ros_msgs/SetCameraExposure)"
      grep -q "uint8 camera_index" <<<"${exposure_service}"
      grep -q "bool automatic" <<<"${exposure_service}"
      time_service="$(rossrv show prism_ros_msgs/SyncSystemTime)"
      grep -q "bool confirm" <<<"${time_service}"
      grep -q "int64 after_offset_us" <<<"${time_service}"
      test -f "$(rospack find prism_ros_driver)/launch/prism.launch"
    '
    check_linkage prism-ros-adapter:noetic \
      /opt/prism-ros1/lib/prism_ros_driver/prism_ros_driver_node
    ;;
  humble|jazzy|kilted|lyrical|rolling)
    docker run --rm "prism-ros-adapter:${DISTRO}" bash -lc '
      set -euo pipefail
      ros2 pkg prefix prism_ros_driver >/dev/null
      interface_text="$(ros2 interface show prism_ros_msgs/msg/CameraFrameMetadata)"
      grep -q "uint32\[4\] exposure_us" <<<"${interface_text}"
      exposure_service="$(ros2 interface show prism_ros_msgs/srv/SetCameraExposure)"
      grep -q "uint8 camera_index" <<<"${exposure_service}"
      grep -q "bool automatic" <<<"${exposure_service}"
      time_service="$(ros2 interface show prism_ros_msgs/srv/SyncSystemTime)"
      grep -q "bool confirm" <<<"${time_service}"
      grep -q "int64 after_offset_us" <<<"${time_service}"
      ros2 launch prism_ros_driver prism.launch.py --show-args >/dev/null
    '
    check_linkage "prism-ros-adapter:${DISTRO}" \
      /opt/prism-ros2/lib/prism_ros_driver/prism_ros_driver_node
    ;;
  *)
    echo "usage: $0 [noetic|humble|jazzy|kilted|lyrical|rolling]" >&2
    exit 2
    ;;
esac
