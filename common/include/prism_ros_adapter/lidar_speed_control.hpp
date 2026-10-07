#pragma once

#include "prism_ros_adapter/driver.hpp"
#include <prism/usb/lidar_speed.hpp>
#include <optional>
#include <stdexcept>

namespace prism_ros_adapter {
namespace detail {
// Called on the serialized control worker. Never pause/resume streams or retry
// writes: a timeout can occur after the hardware has already changed mode.
template <typename Client>
LidarSpeedState lidarSpeedOperation(Client& client, LidarModel model,
                                   bool capturing, uint32_t timeout_ms,
                                   std::optional<uint8_t> mode = std::nullopt) {
  if (model != LidarModel::Mid360S)
    throw std::invalid_argument("LiDAR speed control requires lidar_model=mid360s");
  if (!timeout_ms || timeout_ms > 10000)
    throw std::invalid_argument("timeout_ms must be 1..10000");
  if (mode && *mode != 1 && *mode != 2)
    throw std::invalid_argument("mode must be 1 (normal) or 2 (low)");
  if (capturing)
    throw std::logic_error("stop all capture streams before querying or changing LiDAR speed");
  const auto result = mode
      ? client.setLidarSpeedMode(prism::LidarModel::Mid360S,
            static_cast<prism::LidarSpeedMode>(*mode), timeout_ms)
      : client.lidarSpeedStatus(prism::LidarModel::Mid360S, timeout_ms);
  // The SDK validates the hardware readback, including a requested-mode match.
  return {static_cast<uint8_t>(result.model), static_cast<uint8_t>(result.mode),
          result.device_type};
}
} // namespace detail
} // namespace prism_ros_adapter
