#!/usr/bin/env bash
set -euo pipefail

DISTRO="${1:-}"
TEST_IMAGE="${PRISM_ROS_TEST_IMAGE:-prism-ros-adapter:${DISTRO}}"

case "${PRISM_ROS_ARCH:-$(uname -m)}" in
  x86_64|amd64|x64) DOCKER_PLATFORM=linux/amd64 ;;
  aarch64|arm64) DOCKER_PLATFORM=linux/arm64 ;;
  *)
    echo "unsupported host architecture: ${PRISM_ROS_ARCH:-$(uname -m)}" >&2
    exit 2
    ;;
esac

check_linkage() {
  local image="$1"
  local node="$2"
  docker run --rm --platform "${DOCKER_PLATFORM}" "${image}" bash -lc "
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
    docker run --rm --platform "${DOCKER_PLATFORM}" \
      "${TEST_IMAGE}" bash -lc '
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
      device_info_service="$(rossrv show prism_ros_msgs/GetDeviceInfo)"
      grep -q "string host_sdk_version" <<<"${device_info_service}"
      grep -q "string sensor_board_version" <<<"${device_info_service}"
      device_config_service="$(rossrv show prism_ros_msgs/SetDeviceConfiguration)"
      grep -q "bool set_camera_fps" <<<"${device_config_service}"
      grep -q "uint32 mjpeg_quality" <<<"${device_config_service}"
      lidar_network_service="$(rossrv show prism_ros_msgs/SetLidarNetwork)"
      grep -q "string host_ip" <<<"${lidar_network_service}"
      grep -q "string lidar_ip" <<<"${lidar_network_service}"
      stream_service="$(rossrv show prism_ros_msgs/ControlStreams)"
      grep -q "string command" <<<"${stream_service}"
      grep -q "bool board_imu" <<<"${stream_service}"
      wifi_service="$(rossrv show prism_ros_msgs/SetWifiHotspot)"
      grep -q "bool enabled" <<<"${wifi_service}"
      grep -q "bool persisted" <<<"${wifi_service}"
      gnss_status="$(rosmsg show prism_ros_msgs/GnssTimingStatus)"
      grep -q "int64 message_pps_offset_us" <<<"${gnss_status}"
      rtk_status="$(rosmsg show prism_ros_msgs/TimeSyncRtkStatus)"
      grep -q "uint8 rtcm_format" <<<"${rtk_status}"
      rtk_navigation="$(rosmsg show prism_ros_msgs/ReceiverPosition)"
      grep -q "float64 latitude_deg" <<<"${rtk_navigation}"
      rover_data="$(rosmsg show prism_ros_msgs/RtcmData)"
      grep -Fq "uint8[] data" <<<"${rover_data}"
      for service in GetGnssTiming GetGnssReception GetRtkModuleStatus GetRtkModuleVersions GetCorsConfiguration SetCorsConfiguration ControlRtk GetReceiverPosition GetTimeSyncPort SetTimeSyncPort SetRoverRtcm SetUnifiedExposure; do
        rossrv show "prism_ros_msgs/${service}" >/dev/null
      done
      grep -q "uint32 gnss_uart_baud" <<<"${device_config_service}"
      test -f "$(rospack find prism_ros_driver)/launch/prism.launch"
    '
    check_linkage "${TEST_IMAGE}" \
      /opt/prism-ros1/lib/prism_ros_driver/prism_ros_driver_node
    ;;
  foxy|humble|jazzy|kilted|lyrical|rolling)
    docker run --rm --platform "${DOCKER_PLATFORM}" \
      "${TEST_IMAGE}" bash -lc '
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
      device_info_service="$(ros2 interface show prism_ros_msgs/srv/GetDeviceInfo)"
      grep -q "DeviceInfo info" <<<"${device_info_service}"
      device_info="$(ros2 interface show prism_ros_msgs/msg/DeviceInfo)"
      grep -q "string host_sdk_version" <<<"${device_info}"
      grep -q "string sensor_board_version" <<<"${device_info}"
      device_config_service="$(ros2 interface show prism_ros_msgs/srv/SetDeviceConfiguration)"
      grep -q "bool set_camera_fps" <<<"${device_config_service}"
      grep -q "uint32 mjpeg_quality" <<<"${device_config_service}"
      lidar_network_service="$(ros2 interface show prism_ros_msgs/srv/SetLidarNetwork)"
      grep -q "string host_ip" <<<"${lidar_network_service}"
      grep -q "string lidar_ip" <<<"${lidar_network_service}"
      stream_service="$(ros2 interface show prism_ros_msgs/srv/ControlStreams)"
      grep -q "string command" <<<"${stream_service}"
      grep -q "bool board_imu" <<<"${stream_service}"
      wifi_service="$(ros2 interface show prism_ros_msgs/srv/SetWifiHotspot)"
      grep -q "bool enabled" <<<"${wifi_service}"
      wifi_status="$(ros2 interface show prism_ros_msgs/msg/WifiHotspotStatus)"
      grep -q "bool persisted" <<<"${wifi_status}"
      gnss_status="$(ros2 interface show prism_ros_msgs/msg/GnssTimingStatus)"
      grep -q "int64 message_pps_offset_us" <<<"${gnss_status}"
      rtk_status="$(ros2 interface show prism_ros_msgs/msg/TimeSyncRtkStatus)"
      grep -q "uint8 rtcm_format" <<<"${rtk_status}"
      rtk_navigation="$(ros2 interface show prism_ros_msgs/msg/ReceiverPosition)"
      grep -q "float64 latitude_deg" <<<"${rtk_navigation}"
      for service in GetGnssTiming GetGnssReception GetRtkModuleStatus GetRtkModuleVersions GetCorsConfiguration SetCorsConfiguration ControlRtk GetReceiverPosition GetTimeSyncPort SetTimeSyncPort SetRoverRtcm SetUnifiedExposure; do
        ros2 interface show "prism_ros_msgs/srv/${service}" >/dev/null
      done
      grep -q "uint32 gnss_uart_baud" <<<"${device_config_service}"
      python3 /opt/src/prism-ros-adapter/scripts/verify_navigation_ros2.py
      ros2 launch prism_ros_driver prism.launch.py --show-args >/dev/null
    '
    check_linkage "${TEST_IMAGE}" \
      /opt/prism-ros2/lib/prism_ros_driver/prism_ros_driver_node
    ;;
  *)
    echo "usage: $0 [noetic|foxy|humble|jazzy|kilted|lyrical|rolling]" >&2
    exit 2
    ;;
esac
