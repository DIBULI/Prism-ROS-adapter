cmake_minimum_required(VERSION 3.0.2)
project(prism_ros_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(FILES
  GnssReceptionStatus.msg
  TimeSyncRtkStatus.msg
  TimeSyncRtkVersions.msg
  TimeSyncCorsStatus.msg
  GnssObservations.msg
  GnssRawData.msg
  ReceiverPosition.msg
  GnssTimingStatus.msg
  TimeSyncPortStatus.msg
  RoverRtcmStatus.msg
  RtcmData.msg
  CameraFrameMetadata.msg
  DeviceConfiguration.msg
  DeviceInfo.msg
  LidarNetworkStatus.msg
  LidarStatus.msg
  StreamState.msg
  WifiHotspotStatus.msg
)
add_service_files(FILES
  GetLidarSpeed.srv
  SetLidarSpeed.srv
  GetLidarPower.srv
  SetLidarStandby.srv
  GetGnssReception.srv
  GetRtkModuleStatus.srv
  GetRtkModuleVersions.srv
  GetCorsConfiguration.srv
  SetCorsConfiguration.srv
  ControlRtk.srv
  SetTimeSyncPort.srv
  GetReceiverPosition.srv
  SetUnifiedExposure.srv
  GetGnssTiming.srv
  GetTimeSyncPort.srv
  SetRoverRtcm.srv
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
