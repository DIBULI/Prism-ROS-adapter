#include "prism_ros_adapter/device_time_resolver.hpp"

#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {

using prism_ros_adapter::DeviceTimeCandidate;
using prism_ros_adapter::DeviceTimeResolver;

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

}  // namespace

int main() {
  constexpr uint64_t kSecond = 1000000000ull;
  constexpr uint64_t kDevice = 1777326476000000000ull;
  constexpr uint64_t kSteady = 12000000000ull;

  DeviceTimeResolver resolver;
  require(!resolver.hasBoardReference(),
          "empty resolver must not report a board reference");
  require(!resolver.resolveLidar(kDevice, kDevice, kSteady).has_value(),
          "resolution must wait for a device reference");

  resolver.observeBoard(kDevice, kSteady);
  require(resolver.hasBoardReference(),
          "board observation must establish a board reference");
  const auto raw_matches = resolver.resolveLidar(
      kDevice - 37ull * kSecond + 1000, kDevice + 2000, kSteady + 3000);
  require(raw_matches.has_value(), "raw-matching timestamp must resolve");
  require(raw_matches->candidate == DeviceTimeCandidate::LidarRaw,
          "raw timestamp must win when SDK normalized time is 37 seconds off");
  require(raw_matches->timestamp_ns == kDevice + 2000,
          "raw timestamp value must be preserved");

  const auto normalized_matches = resolver.resolveLidar(
      kDevice + 5000, kDevice + 37ull * kSecond, kSteady + 4000);
  require(normalized_matches.has_value(),
          "normalized-matching timestamp must resolve");
  require(normalized_matches->candidate ==
              DeviceTimeCandidate::SdkNormalized,
          "SDK normalized timestamp must win for a true TAI raw clock");

  const auto too_far = resolver.resolveLidar(
      kDevice - 37ull * kSecond, kDevice + 37ull * kSecond,
      kSteady + 1000000);
  require(!too_far.has_value(),
          "candidates outside the device-time tolerance must be rejected");

  resolver.observeCamera(kDevice + kSecond, kSteady + kSecond);
  const auto board_still_wins = resolver.resolveLidar(
      kDevice + 2ull * kSecond, kDevice + 39ull * kSecond,
      kSteady + 2ull * kSecond);
  require(board_still_wins.has_value(),
          "fresh board reference must remain usable");
  require(board_still_wins->candidate == DeviceTimeCandidate::SdkNormalized,
          "camera must not replace a fresh board IMU reference");

  resolver.observeCamera(kDevice + 7ull * kSecond,
                         kSteady + 7ull * kSecond);
  const auto camera_fallback = resolver.resolveLidar(
      kDevice + 8ull * kSecond, kDevice + 45ull * kSecond,
      kSteady + 8ull * kSecond);
  require(camera_fallback.has_value(),
          "camera reference must replace a stale board reference");
  require(camera_fallback->candidate == DeviceTimeCandidate::SdkNormalized,
          "camera fallback must select the matching candidate");

  resolver.reset();
  require(!resolver.hasReference(), "reset must clear the reference");
  require(!resolver.hasBoardReference(),
          "reset must clear the board reference");

  std::cout << "Device time resolver tests passed\n";
  return EXIT_SUCCESS;
}
