cmake_minimum_required(VERSION 3.0.2)
project(prism_ros_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(FILES
  CameraFrameMetadata.msg
  DeviceConfiguration.msg
  DeviceInfo.msg
  LidarNetworkStatus.msg
  LidarStatus.msg
  StreamState.msg
  WifiHotspotStatus.msg
)
add_service_files(FILES
  ControlStreams.srv
  GetDeviceConfiguration.srv
  GetDeviceInfo.srv
  GetExposure.srv
  GetLidarNetwork.srv
  GetLidarStatus.srv
  GetStreamState.srv
  GetWifiHotspot.srv
  ProbeLidarNetwork.srv
  SetCameraExposure.srv
  SetDeviceConfiguration.srv
  SetExposureLimits.srv
  SetLidarNetwork.srv
  SetTargetBrightness.srv
  SetWifiHotspot.srv
  SyncSystemTime.srv
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
