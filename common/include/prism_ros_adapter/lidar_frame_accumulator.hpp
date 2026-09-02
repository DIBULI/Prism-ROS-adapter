#pragma once

#include "prism_ros_adapter/driver.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace prism_ros_adapter {

class LidarFrameAccumulator {
 public:
  static constexpr uint64_t kFramePeriodNs = 100000000ULL;

  std::vector<LidarPointBatch> append(LidarPointBatch batch);
  void reset() noexcept;
  size_t pendingPointCount() const noexcept;

 private:
  void beginFrame(uint64_t window_start_ns, uint64_t first_point_timestamp_ns,
                  const LidarPointBatch& source);
  LidarPointBatch finishFrame();

  bool active_ = false;
  uint64_t window_start_ns_ = 0;
  uint64_t window_end_ns_ = 0;
  LidarPointBatch frame_;
};

}  // namespace prism_ros_adapter
