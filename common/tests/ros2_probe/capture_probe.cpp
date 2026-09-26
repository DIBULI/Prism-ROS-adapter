// Passive ROS subscriber: no SDK connection, control writes or disk recording.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char** argv) {
  const double seconds = argc > 1 ? std::max(1.0, std::atof(argv[1])) : 60.0;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("prism_capture_probe");
  std::array<uint64_t, 4> cameras{};
  std::array<uint64_t, 2> imus{}, last_imu_ns{}, imu_gaps{};
  std::map<std::string, std::string> diagnostics;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  for (size_t i = 0; i < cameras.size(); ++i) {
    subscriptions.push_back(node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/prism/camera" + std::to_string(i) + "/image/compressed",
        rclcpp::QoS(64).reliable(),
        [&, i](sensor_msgs::msg::CompressedImage::ConstSharedPtr) { ++cameras[i]; }));
  }
  for (size_t i = 0; i < imus.size(); ++i) {
    subscriptions.push_back(node->create_subscription<sensor_msgs::msg::Imu>(
        "/prism/imu" + std::to_string(i) + "/data",
        rclcpp::SensorDataQoS().keep_last(4096),
        [&, i](sensor_msgs::msg::Imu::ConstSharedPtr value) {
          const uint64_t stamp = uint64_t(value->header.stamp.sec) * 1000000000ull +
                                 value->header.stamp.nanosec;
          if (last_imu_ns[i] && stamp > last_imu_ns[i] + 2000000ull) ++imu_gaps[i];
          last_imu_ns[i] = stamp;
          ++imus[i];
        }));
  }
  subscriptions.push_back(node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable(),
      [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr value) {
        for (const auto& status : value->status) {
          if (status.name == "Prism ROS adapter") {
            for (const auto& pair : status.values) diagnostics[pair.key] = pair.value;
          }
        }
      }));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // Exclude DDS discovery and the driver's initial capture transition.
  const auto warmup_end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < warmup_end) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  cameras.fill(0);
  imus.fill(0);
  last_imu_ns.fill(0);
  imu_gaps.fill(0);
  const auto start = std::chrono::steady_clock::now();
  auto timer = node->create_wall_timer(std::chrono::milliseconds(int(seconds * 1000)),
                                      [&]() { executor.cancel(); });
  executor.spin();
  const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  std::cout << "duration_s=" << elapsed << '\n';
  for (size_t i = 0; i < cameras.size(); ++i)
    std::cout << "camera" << i << " frames=" << cameras[i] << " hz=" << cameras[i]/elapsed << '\n';
  for (size_t i = 0; i < imus.size(); ++i)
    std::cout << "imu" << i << " samples=" << imus[i] << " hz=" << imus[i]/elapsed
              << " timestamp_gaps_over_2ms=" << imu_gaps[i] << '\n';
  for (const auto& pair : diagnostics)
    std::cout << "driver." << pair.first << '=' << pair.second << '\n';
  rclcpp::shutdown();
  return cameras[0] > 0 && imus[0] > 0 ? 0 : 1;
}
