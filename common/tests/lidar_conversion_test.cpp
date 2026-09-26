#include "prism_ros_adapter/lidar_conversion.hpp"
#include "prism_ros_adapter/lidar_frame_accumulator.hpp"
#include <iostream>
#include <stdexcept>

void check(bool value) { if (!value) throw std::runtime_error("LiDAR conversion test failed"); }
int main() {
  constexpr uint64_t base = 1800000000000000000ull;
  prism::LidarPointBatch packet;
  packet.model = prism::LidarModel::Xt32;
  packet.version = 3;
  packet.points.resize(3);
  packet.points[0].offset_ns = -1000;
  packet.points[0].ring = 12;
  packet.points[0].return_id = 2;
  packet.points[1].offset_ns = -5000;
  packet.points[1].ring = 5;
  packet.points[2] = packet.points[0];
  packet.points[2].return_id = 1;
  const auto xt = prism_ros_adapter::convertLidarBatch(packet, base);
  check(xt.timestamp_ns == base-5000 && xt.explicit_point_times);
  check(xt.points[0].ring == 5 && xt.points[0].offset_time_ns == 0);
  check(xt.points[1].ring == 12 && xt.points[1].offset_time_ns == 4000);
  check(xt.points[1].return_id == 2 && xt.points[2].return_id == 1);
  check(!xt.points[1].line_valid);
  prism_ros_adapter::LidarFrameAccumulator accumulator;
  check(accumulator.append(xt).empty());
  auto next = xt; next.timestamp_ns += 100000000;
  const auto frames = accumulator.append(next);
  check(frames.size() == 1 && frames[0].points.size() == 3);
  check(frames[0].timestamp_ns + frames[0].points[1].offset_time_ns == base-1000);
  check(frames[0].points[2].ring == 12 && frames[0].points[2].return_id == 1);
  packet.model = prism::LidarModel::Mid360;
  packet.version = 2;
  packet.time_interval_100ns = 100;
  for (size_t i=0;i<3;++i) { packet.points[i] = {}; packet.points[i].line_valid=true; packet.points[i].line=uint8_t(i); }
  const auto mid = prism_ros_adapter::convertLidarBatch(packet, base);
  check(mid.timestamp_ns == base && mid.points[1].offset_time_ns == 5000);
  check(mid.points[2].offset_time_ns == 10000 && mid.points[2].line_valid && mid.points[2].line == 2);
  packet.model = prism::LidarModel::Xt32;
  bool rejected=false;
  try { (void)prism_ros_adapter::convertLidarBatch(packet, base); }
  catch (const std::invalid_argument&) { rejected=true; }
  check(rejected);
  std::cout << "XT32 signed/equal-time returns and Livox N-1/line preservation passed\n";
}
