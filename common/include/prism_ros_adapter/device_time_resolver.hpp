#pragma once

#include <cstdint>
#include <limits>
#include <optional>

namespace prism_ros_adapter {

enum class DeviceTimeCandidate {
  SdkNormalized,
  LidarRaw,
};

struct DeviceTimeResolution {
  uint64_t timestamp_ns = 0;
  DeviceTimeCandidate candidate = DeviceTimeCandidate::SdkNormalized;
};

// Selects the LiDAR timestamp representation that belongs to the same device
// clock domain as the sensor-board. Host wall-clock time is never consulted.
class DeviceTimeResolver {
 public:
  void reset() noexcept { reference_ = {}; }

  void observeBoard(uint64_t timestamp_ns, uint64_t arrival_steady_ns) noexcept {
    observe(timestamp_ns, arrival_steady_ns, ReferenceSource::BoardImu);
  }

  void observeCamera(uint64_t timestamp_ns,
                     uint64_t arrival_steady_ns) noexcept {
    if (reference_.source == ReferenceSource::BoardImu &&
        arrival_steady_ns >= reference_.arrival_steady_ns &&
        arrival_steady_ns - reference_.arrival_steady_ns <=
            kBoardReferenceHoldNs) {
      return;
    }
    observe(timestamp_ns, arrival_steady_ns, ReferenceSource::Camera);
  }

  bool hasReference() const noexcept {
    return reference_.source != ReferenceSource::None;
  }

  bool hasBoardReference() const noexcept {
    return reference_.source == ReferenceSource::BoardImu;
  }

  std::optional<DeviceTimeResolution> resolveLidar(
      uint64_t sdk_normalized_ns, uint64_t raw_ns,
      uint64_t arrival_steady_ns) const noexcept {
    if (!hasReference() || arrival_steady_ns < reference_.arrival_steady_ns) {
      return std::nullopt;
    }
    const uint64_t elapsed_ns =
        arrival_steady_ns - reference_.arrival_steady_ns;
    if (elapsed_ns > kMaximumReferenceAgeNs ||
        reference_.timestamp_ns >
            std::numeric_limits<uint64_t>::max() - elapsed_ns) {
      return std::nullopt;
    }
    const uint64_t expected_ns = reference_.timestamp_ns + elapsed_ns;

    const uint64_t sdk_delta = candidateDelta(sdk_normalized_ns, expected_ns);
    const uint64_t raw_delta = candidateDelta(raw_ns, expected_ns);
    const bool use_raw = raw_delta < sdk_delta;
    const uint64_t best_delta = use_raw ? raw_delta : sdk_delta;
    if (best_delta > kMaximumCandidateDeltaNs) return std::nullopt;
    return DeviceTimeResolution{
        use_raw ? raw_ns : sdk_normalized_ns,
        use_raw ? DeviceTimeCandidate::LidarRaw
                : DeviceTimeCandidate::SdkNormalized};
  }

 private:
  enum class ReferenceSource {
    None,
    Camera,
    BoardImu,
  };

  struct Reference {
    uint64_t timestamp_ns = 0;
    uint64_t arrival_steady_ns = 0;
    ReferenceSource source = ReferenceSource::None;
  };

  static constexpr uint64_t kNanosecondsPerSecond = 1000000000ull;
  static constexpr uint64_t kBoardReferenceHoldNs =
      2ull * kNanosecondsPerSecond;
  static constexpr uint64_t kMaximumReferenceAgeNs =
      5ull * kNanosecondsPerSecond;
  static constexpr uint64_t kMaximumCandidateDeltaNs =
      2ull * kNanosecondsPerSecond;

  static uint64_t candidateDelta(uint64_t candidate,
                                 uint64_t expected) noexcept {
    if (candidate == 0) return std::numeric_limits<uint64_t>::max();
    return candidate >= expected ? candidate - expected : expected - candidate;
  }

  void observe(uint64_t timestamp_ns, uint64_t arrival_steady_ns,
               ReferenceSource source) noexcept {
    if (timestamp_ns == 0 || arrival_steady_ns == 0) return;
    reference_ = {timestamp_ns, arrival_steady_ns, source};
  }

  Reference reference_;
};

}  // namespace prism_ros_adapter
