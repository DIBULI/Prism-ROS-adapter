#include "prism_ros_adapter/supported_device.hpp"
#include <iostream>
int main() {
  using prism::DeviceModel;
  for (auto m : {DeviceModel::Unknown, DeviceModel::Prism413GSV0})
    prism_ros_adapter::requireSupportedDevice(m);
  for (auto m : {DeviceModel::Prism415RSV0, DeviceModel::Prism21315V0, static_cast<DeviceModel>(255)}) {
    bool rejected = false;
    try { prism_ros_adapter::requireSupportedDevice(m); }
    catch (const std::runtime_error&) { rejected = true; }
    if (!rejected) return 1;
  }
  std::cout << "PASS 413GSV0 and legacy unknown accepted; other models rejected\n";
}
