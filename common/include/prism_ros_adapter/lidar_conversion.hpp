#pragma once
#include "prism_ros_adapter/driver.hpp"
#include "prism/usb/lidar_points.hpp"
#include <algorithm>
#include <limits>
#include <utility>

namespace prism_ros_adapter {
inline LidarPointBatch convertLidarBatch(const prism::LidarPointBatch& batch,
                                         uint64_t selected_base_ns) {
  LidarPointBatch out;
  out.batch_id = batch.batch_id;
  out.timestamp_raw = batch.timestamp_raw;
  out.timestamp_ns = selected_base_ns;
  out.explicit_point_times = true;
  std::vector<std::pair<uint64_t, size_t>> order;
  order.reserve(batch.points.size());
  for (size_t i = 0; i < batch.points.size(); ++i)
    order.emplace_back(prism::lidarPointTimestampNs(batch, i, selected_base_ns), i);
  // XT32 return blocks need not be ordered by time. Sort stably, retaining
  // return/channel identity and equal-time returns, before 100 ms aggregation.
  std::stable_sort(order.begin(), order.end(), [](const auto& a, const auto& b) {
    return a.first < b.first;
  });
  if (!order.empty()) out.timestamp_ns = order.front().first;
  out.points.reserve(order.size());
  for (const auto& item : order) {
    const auto& p = batch.points[item.second];
    const auto offset = item.first - out.timestamp_ns;
    if (offset > UINT32_MAX) throw std::overflow_error("LiDAR batch time span exceeds UINT32");
    LidarPoint point;
    point.x_m = static_cast<float>(p.x_mm) * .001F;
    point.y_m = static_cast<float>(p.y_mm) * .001F;
    point.z_m = static_cast<float>(p.z_mm) * .001F;
    point.reflectivity = p.reflectivity;
    point.tag = p.tag;
    point.offset_time_ns = static_cast<uint32_t>(offset);
    point.ring = p.ring;
    point.line = p.line;
    point.line_valid = p.line_valid ? 1 : 0;
    point.return_id = p.return_id;
    point.confidence = p.confidence;
    out.points.push_back(point);
  }
  return out;
}
}  // namespace prism_ros_adapter
