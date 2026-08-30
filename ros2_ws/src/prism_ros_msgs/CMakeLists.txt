cmake_minimum_required(VERSION 3.10)
project(prism_ros_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraFrameMetadata.msg"
  "msg/DeviceConfiguration.msg"
  "msg/DeviceInfo.msg"
  "msg/LidarNetworkStatus.msg"
  "msg/LidarStatus.msg"
  "msg/StreamState.msg"
  "msg/WifiHotspotStatus.msg"
  "srv/ControlStreams.srv"
  "srv/GetDeviceConfiguration.srv"
  "srv/GetDeviceInfo.srv"
  "srv/GetExposure.srv"
  "srv/GetLidarNetwork.srv"
  "srv/GetLidarStatus.srv"
  "srv/GetStreamState.srv"
  "srv/GetWifiHotspot.srv"
  "srv/ProbeLidarNetwork.srv"
  "srv/SetCameraExposure.srv"
  "srv/SetDeviceConfiguration.srv"
  "srv/SetExposureLimits.srv"
  "srv/SetLidarNetwork.srv"
  "srv/SetTargetBrightness.srv"
  "srv/SetWifiHotspot.srv"
  "srv/SyncSystemTime.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
