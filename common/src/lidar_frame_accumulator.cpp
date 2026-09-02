#include "prism_ros_adapter/lidar_frame_accumulator.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace prism_ros_adapter {

namespace {

uint64_t pointOffsetNs(size_t point_index, size_t point_count,
                       uint32_t time_interval_100ns) {
  if (point_count <= 1u || time_interval_100ns == 0u) return 0u;
  const uint64_t span_ns =
      static_cast<uint64_t>(time_interval_100ns) * 100ULL;
  const uint64_t intervals = static_cast<uint64_t>(point_count - 1u);
  return (static_cast<uint64_t>(point_index) * span_ns + intervals / 2u) /
         intervals;
}

}  // namespace

std::vector<LidarPointBatch> LidarFrameAccumulator::append(
    LidarPointBatch batch) {
  std::vector<LidarPointBatch> completed;
  if (batch.points.empty()) return completed;

  const size_t point_count = batch.points.size();
  for (size_t index = 0; index < point_count; ++index) {
    const uint64_t source_offset_ns =
        pointOffsetNs(index, point_count, batch.time_interval_100ns);
    if (batch.timestamp_ns >
        std::numeric_limits<uint64_t>::max() - source_offset_ns) {
      throw std::overflow_error("LiDAR point timestamp overflow");
    }
    const uint64_t point_timestamp_ns = batch.timestamp_ns + source_offset_ns;

    if (active_ && point_timestamp_ns < window_start_ns_) {
      // A clock-domain reset must not combine points from two time epochs.
      reset();
    }
    if (!active_) beginFrame(point_timestamp_ns, point_timestamp_ns, batch);

    if (point_timestamp_ns >= window_end_ns_) {
      if (!frame_.points.empty()) completed.push_back(finishFrame());

      const uint64_t elapsed_ns = point_timestamp_ns - window_start_ns_;
      const uint64_t elapsed_periods = elapsed_ns / kFramePeriodNs;
      window_start_ns_ += elapsed_periods * kFramePeriodNs;
      window_end_ns_ = window_start_ns_ + kFramePeriodNs;
      beginFrame(window_start_ns_, point_timestamp_ns, batch);
    }

    LidarPoint point = std::move(batch.points[index]);
    point.offset_time_ns =
        static_cast<uint32_t>(point_timestamp_ns - frame_.timestamp_ns);
    frame_.points.push_back(std::move(point));
  }

  return completed;
}

void LidarFrameAccumulator::reset() noexcept {
  active_ = false;
  window_start_ns_ = 0;
  window_end_ns_ = 0;
  frame_ = {};
}

size_t LidarFrameAccumulator::pendingPointCount() const noexcept {
  return frame_.points.size();
}

void LidarFrameAccumulator::beginFrame(uint64_t window_start_ns,
                                       uint64_t first_point_timestamp_ns,
                                       const LidarPointBatch& source) {
  active_ = true;
  window_start_ns_ = window_start_ns;
  if (window_start_ns_ >
      std::numeric_limits<uint64_t>::max() - kFramePeriodNs) {
    throw std::overflow_error("LiDAR frame timestamp overflow");
  }
  window_end_ns_ = window_start_ns_ + kFramePeriodNs;
  frame_ = {};
  frame_.batch_id = source.batch_id;
  frame_.timestamp_ns = first_point_timestamp_ns;
  frame_.timestamp_raw = source.timestamp_raw;
  frame_.points.reserve(20000u);
}

LidarPointBatch LidarFrameAccumulator::finishFrame() {
  if (!frame_.points.empty()) {
    const uint64_t span_ns = frame_.points.back().offset_time_ns;
    frame_.time_interval_100ns =
        static_cast<uint32_t>((span_ns + 99u) / 100u);
  }
  return std::move(frame_);
}

}  // namespace prism_ros_adapter
