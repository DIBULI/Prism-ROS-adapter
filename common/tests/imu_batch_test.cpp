#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic ignored "-Wsubobject-linkage"
#endif
#include "../src/driver.cpp"
#include "prism_ros_adapter/imu_batch.hpp"
#include <iostream>

using namespace prism_ros_adapter;
struct Message {
  std::vector<uint64_t> timestamp_ns;
  std::vector<double> acceleration_x, acceleration_y, acceleration_z;
  std::vector<double> angular_velocity_x, angular_velocity_y, angular_velocity_z;
};
static void require(bool ok, const char* reason) {
  if (!ok) throw std::runtime_error(reason);
}
int main() {
  using Clock = std::chrono::steady_clock;
  using Queue = DispatchQueue<BoardImuSample>;
  std::mutex mutex;
  std::condition_variable cv;
  std::vector<std::vector<BoardImuSample>> received;
  Queue queue(1024, [&](const std::vector<BoardImuSample>& batch) {
    std::lock_guard<std::mutex> lock(mutex);
    received.push_back(batch); cv.notify_all();
  }, 16, 20);
  auto wait = [&](size_t n) {
    std::unique_lock<std::mutex> lock(mutex);
    require(cv.wait_for(lock, std::chrono::seconds(2), [&] {return received.size() >= n;}),
            "partial batch never flushed");
  };
  queue.start();
  BoardImuSample value;
  value.timestamp_ns = 1700000000123456789ULL;
  value.acceleration_m_s2 = {1.125, -9.80665, 0.000000123456789};
  value.angular_velocity_rad_s = {-0.0123456789, 0, 3.141592653589793};
  queue.push(value); wait(1); // No later sample is needed to flush a partial batch.
  for (uint32_t i = 0; i < 37; ++i) {
    value.sample_id = i; value.sensor_id = i % 2;
    value.timestamp_ns = 1700000000123456789ULL + i * 1250000ULL;
    queue.push(value);
  }
  auto deadline = Clock::now() + std::chrono::seconds(2);
  queue.beginDrain(deadline); queue.waitUntilIdle(deadline);
  queue.stop();
  require(queue.dropped() == 0 && queue.callbackErrors() == 0, "batch dispatch loss");
  std::vector<BoardImuSample> flat;
  for (size_t i = 1; i < received.size(); ++i) {
    require(!received[i].empty() && received[i].size() <= 16, "unbounded/empty batch");
    flat.insert(flat.end(), received[i].begin(), received[i].end());
  }
  require(flat.size() == 37, "STOP lost the partial tail");
  std::array<Message, 2> messages;
  for (size_t i = 0; i < flat.size(); ++i) {
    require(flat[i].sample_id == i, "sample reordered/duplicated");
    appendImuSample(messages[flat[i].sensor_id], flat[i]);
  }
  for (size_t id = 0; id < 2; ++id) {
    const auto& m = messages[id];
    require(m.timestamp_ns.size() == (id ? 18 : 19), "sensor channels mixed");
    for (size_t i = 0; i < m.timestamp_ns.size(); ++i) {
      require(m.timestamp_ns[i] == 1700000000123456789ULL + (i*2+id)*1250000ULL,
              "timestamp precision lost");
      require(m.acceleration_x[i] == value.acceleration_m_s2[0] &&
              m.acceleration_y[i] == value.acceleration_m_s2[1] &&
              m.acceleration_z[i] == value.acceleration_m_s2[2] &&
              m.angular_velocity_x[i] == value.angular_velocity_rad_s[0] &&
              m.angular_velocity_y[i] == value.angular_velocity_rad_s[1] &&
              m.angular_velocity_z[i] == value.angular_velocity_rad_s[2], "SI values changed");
    }
  }
  const size_t before = received.size();
  queue.start(); value.sample_id = 42; queue.push(value); queue.stop();
  require(received.size() == before + 1 && received.back().size() == 1 &&
          received.back()[0].sample_id == 42, "shutdown/restart tail contamination");
  for (const auto& settings : {std::pair<int,int>{0,20}, {257,20}, {16,0}, {16,101}}) {
    bool rejected = false;
    try { Queue bad(1024, Queue::BatchHandler{}, settings.first, settings.second); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "unbounded batching settings accepted");
  }
  // Clock regressions remain visible to consumers; batching must not repair them.
  Message discontinuous;
  value.timestamp_ns = 900; appendImuSample(discontinuous, value);
  value.timestamp_ns = 1; appendImuSample(discontinuous, value);
  require(discontinuous.timestamp_ns == std::vector<uint64_t>({900,1}), "clock jump hidden");
  std::cout << "PASS IMU batching, timed flush, STOP/restart tail, timestamps and SI values\n";
}
