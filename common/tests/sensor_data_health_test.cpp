#include "prism_ros_adapter/sensor_data_health.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

using Health = prism_ros_adapter::SensorDataHealth;
using S = Health::Stream;
using I = Health::Issue;
constexpr auto sec = Health::kSecond;
void require(bool ok, const char* message) {
  if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}
bool contains(const std::vector<std::string>& logs, const std::string& text) {
  for (const auto& log : logs) if (log.find(text) != std::string::npos) return true;
  return false;
}

int main() {
  Health h;
  h.reset(S::BoardImu0, 0, 800, 5 * sec);
  // Variable host arrivals and nominal ODR drift are not source losses.
  for (uint64_t i = 0; i < 1000; ++i)
    h.observe(S::BoardImu0, (i / 100) * sec / 8, sec + i * 1254000, i, 16);
  require(h.reports(2 * sec).empty(), "burst/drift misclassified as loss");
  h.observe(S::BoardImu0, 2 * sec, sec + 1002 * 1254000, 1002, 16);
  auto logs = h.reports(2 * sec);
  require(contains(logs, "sequence_missing+=2") && contains(logs, "estimated_missing+=2"),
          "missing IMU samples not counted");
  h.observe(S::BoardImu0, 2 * sec + 1, sec + 1002 * 1254000, 1002, 16);
  require(h.reports(3 * sec).empty(), "warning throttle ineffective");
  logs = h.reports(7 * sec);
  require(contains(logs, "sequence_duplicate+=1") && contains(logs, "timestamp_duplicate+=1"),
          "throttled duplicate discarded");
  h.observe(S::BoardImu0, 8 * sec, 1, 1003, 16);
  logs = h.reports(8 * sec, true);
  require(contains(logs, "timestamp_backward+=1") && !contains(logs, "estimated_missing"),
          "backward clock treated as enormous loss");
  h.reset(S::BoardImu0, 9 * sec, 400, 5 * sec);
  h.observe(S::BoardImu0, 9 * sec, 100, 0, 16);
  h.observe(S::BoardImu0, 9 * sec, 2500100, 1, 16);
  require(h.reports(9 * sec, true).empty(), "restart/rate change has a false gap");
  h.disable(S::BoardImu0);
  require(h.reports(30 * sec).empty(), "intentional stop is stale");

  // Every stream is independently observable, including an entirely silent
  // camera while the others still deliver. Unknown rates are never guessed.
  for (size_t n = 0; n < static_cast<size_t>(S::Count); ++n) {
    Health each;
    const auto stream = static_cast<S>(n);
    each.reset(stream, 0, 0, 5 * sec);
    each.observe(stream, 0, sec, 1);
    each.observe(stream, 1, 4 * sec, 3);
    logs = each.reports(1);
    require(contains(logs, "sequence_missing+=1") && !contains(logs, "estimated_missing"),
            "per-stream sequence checking or unknown-rate semantics failed");
    logs = each.reports(6 * sec);
    require(contains(logs, "no_data_ms="), "per-stream silence missing");
    require(each.reports(20 * sec).empty(), "unchanged stale stream floods logs");
    each.touch(stream, 21 * sec);
    require(contains(each.reports(26 * sec), "no_data_ms="), "second stall not detected");
  }
  Health wrap;
  for (unsigned bits : {16u, 32u, 64u}) {
    const uint64_t maximum = bits == 16 ? UINT16_MAX : bits == 32 ? UINT32_MAX : UINT64_MAX;
    wrap.reset(S::BoardImu1, 0);
    wrap.observe(S::BoardImu1, 0, 0, maximum, bits);
    wrap.observe(S::BoardImu1, 1, 0, 0, bits);
    require(wrap.reports(1, true).empty(), "normal sequence rollover is a gap");
    wrap.observe(S::BoardImu1, 2, 0, 2, bits);
    require(contains(wrap.reports(2, true), "sequence_missing+=1"), "rollover gap missing");
    wrap.observe(S::BoardImu1, 3, 0, 1, bits);
    logs = wrap.reports(3, true);
    require(contains(logs, "sequence_reset_or_reorder+=1") && !contains(logs, "sequence_missing"),
            "sequence reset is unsigned underflow");
  }
  Health navigation;
  navigation.session(S::RawDispatch, 1);
  navigation.observe(S::RawDispatch, 0, 0, 100, 64);
  navigation.session(S::RawDispatch, 2);
  navigation.observe(S::RawDispatch, 1, 0, 1, 64);
  logs = navigation.reports(1);
  require(contains(logs, "session_changed+=1") && !contains(logs, "sequence_missing"),
          "GNSS session change has false loss");
  navigation.counter(S::GnssReception, I::FifoOverflow, 100);
  navigation.counter(S::GnssReception, I::FifoOverflow, 103);
  navigation.counter(S::GnssReception, I::FifoOverflow, 0);
  navigation.counter(S::GnssReception, I::FifoOverflow, 2);
  logs = navigation.reports(2);
  require(contains(logs, "fifo_overflow+=5"), "device counter baseline/reset failed");
  navigation.condition(S::GnssReception, I::RawStale, true);
  navigation.condition(S::GnssReception, I::RawStale, true);
  require(contains(navigation.reports(3, true), "raw_input_stale+=1"), "stale edge flooded");
  navigation.condition(S::GnssReception, I::RawStale, false);
  navigation.condition(S::GnssReception, I::RawStale, true);
  require(contains(navigation.reports(4, true), "raw_input_stale+=1"), "new stale edge missing");

  Health queue;
  queue.counter(S::CameraMetadata, I::QueueDrop, 2, true);
  queue.reset(S::CameraMetadata, 1);
  queue.counter(S::CameraMetadata, I::QueueDrop, 2, true);
  logs = queue.reports(1);
  require(contains(logs, "dispatch_drop+=2") && !contains(logs, "dispatch_drop+=4"),
          "restart double-counted drops or discarded pending warnings");
  queue.add(S::CameraMetadata, I::Incomplete, 20);
  queue.add(S::CameraMetadata, I::Incomplete, 30);
  require(queue.reports(2).empty(), "pending counts bypassed throttle");
  require(contains(queue.reports(3, true), "incomplete_frame+=50"), "final flush lost pending counts");

  Health lidar;
  lidar.observe(S::LidarPoints, 1, sec, 1, 32, true);
  lidar.observe(S::LidarPoints, 2, sec, 2, 32, true);
  require(lidar.reports(2).empty(), "equal-time returns classified as duplicates");
  // Losing synchronization breaks the clock baseline, not the sequence baseline.
  lidar.observe(S::LidarImu, 1, sec, 1);
  lidar.observe(S::LidarImu, 2, 0, 2);
  lidar.observe(S::LidarImu, 3, 1, 3);
  require(lidar.reports(3).empty(), "clock domain transition misclassified");
  Health query;
  query.queryError(S::GnssTiming, "synthetic\nquery failure");
  logs = query.reports(1);
  require(contains(logs, "query_error+=1") && contains(logs, "last_error=synthetic query failure"),
          "query error detail not retained as a single bounded line");
  std::cout << "All sensor health checks passed (sequence, clock, lifecycle, counters, silence, throttling)\n";
}
