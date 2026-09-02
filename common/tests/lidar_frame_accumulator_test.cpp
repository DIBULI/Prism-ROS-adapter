#include "prism_ros_adapter/lidar_frame_accumulator.hpp"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

using prism_ros_adapter::LidarFrameAccumulator;
using prism_ros_adapter::LidarPoint;
using prism_ros_adapter::LidarPointBatch;

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

LidarPointBatch makeBatch(uint32_t batch_id, uint64_t timestamp_ns,
                          uint32_t first_point_index) {
  LidarPointBatch batch;
  batch.batch_id = batch_id;
  batch.timestamp_ns = timestamp_ns;
  batch.timestamp_raw = timestamp_ns;
  batch.time_interval_100ns = 57550u;
  batch.points.reserve(1152u);
  for (uint32_t index = 0; index < 1152u; ++index) {
    LidarPoint point;
    point.x_m = static_cast<float>(first_point_index + index);
    point.reflectivity = static_cast<uint8_t>(index & 0xffu);
    batch.points.push_back(point);
  }
  return batch;
}

}  // namespace

int main() {
  constexpr uint64_t kBaseTimestampNs = 1780000000000000000ULL;
  constexpr uint64_t kSourceBatchPeriodNs = 5760000ULL;

  LidarFrameAccumulator accumulator;
  std::vector<LidarPointBatch> frames;
  uint32_t source_point_index = 0;
  for (uint32_t batch = 0; batch < 35u; ++batch) {
    auto completed = accumulator.append(makeBatch(
        batch, kBaseTimestampNs + batch * kSourceBatchPeriodNs,
        source_point_index));
    source_point_index += 1152u;
    for (auto& frame : completed) frames.push_back(std::move(frame));
  }

  require(frames.size() == 2u,
          "35 normal source batches must produce two complete 10 Hz frames");
  for (size_t frame_index = 0; frame_index < frames.size(); ++frame_index) {
    const auto& frame = frames[frame_index];
    require(frame.timestamp_ns ==
                kBaseTimestampNs +
                    frame_index * LidarFrameAccumulator::kFramePeriodNs,
            "10 Hz frame timestamp is not on the accumulator timeline");
    require(frame.points.size() == 20000u,
            "a 100 ms frame at 200 kpoints/s must contain 20000 points");
    require(frame.points.front().offset_time_ns == 0u,
            "first point offset must be zero");
    require(frame.points.back().offset_time_ns == 99995000u,
            "last point offset must be 99.995 ms");
    require(frame.time_interval_100ns == 999950u,
            "frame interval must preserve the first-to-last span");
    for (size_t point = 1; point < frame.points.size(); ++point) {
      require(frame.points[point].offset_time_ns ==
                  frame.points[point - 1u].offset_time_ns + 5000u,
              "normal Mid-360 point offsets must advance by 5 us");
    }
  }
  require(accumulator.pendingPointCount() == 320u,
          "points after the second complete frame must remain pending");

  accumulator.reset();
  require(accumulator.pendingPointCount() == 0u,
          "reset must discard a partial frame");

  std::cout << "LiDAR 10 Hz frame accumulator tests passed\n";
  return EXIT_SUCCESS;
}
