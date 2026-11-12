#pragma once
#include <prism/usb/device_info.hpp>
#include <stdexcept>
namespace prism_ros_adapter {
inline void requireSupportedDevice(prism::DeviceModel model) {
  // Older Agents do not report a model; retain their 413GSV0 JPEG protocol.
  if (model != prism::DeviceModel::Prism413GSV0 && model != prism::DeviceModel::Unknown)
    throw std::runtime_error("ROS adapter 1.3.0 supports 413GSV0 only");
}
}
