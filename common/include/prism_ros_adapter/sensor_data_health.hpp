#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace prism_ros_adapter {

// Used only on the SDK receive/control thread. Dispatcher counters are atomic.
// Observes data; never repairs, re-times or suppresses sensor samples.
class SensorDataHealth {
 public:
  enum class Stream : size_t {
    Camera0, Camera1, Camera2, Camera3, CameraMetadata, BoardImu0, BoardImu1,
    LidarPoints, LidarImu, GnssTiming, GnssReception, GnssObservations,
    GnssRaw, CorsRtcm, Receiver, RtkStatus, RoverRtcm, Transport,
    BoardImuDispatch, RawDispatch, GnssFix, RtkFix, Count
  };
  enum class Issue : size_t {
    SequenceMissing, SequenceDuplicate, SequenceReset, TimestampGap,
    EstimatedMissing, TimestampDuplicate, TimestampBackward, SourceGap,
    Incomplete, CacheGap, LostBytes, Unsynchronized, QueueDrop, CallbackError,
    QueryError, SessionChanged, UartError, FifoOverflow, Rejected, CrcError,
    UpstreamDrop, RawStale, NmeaStale, StatusStale, Count
  };
  static constexpr uint64_t kSecond = 1000000000ULL;
  static constexpr uint64_t kReportInterval = 5 * kSecond;

  void reset(Stream stream, uint64_t now, double rate_hz = 0,
             uint64_t silence_ns = 0) {
    auto& s = state(stream);
    // Keep pending/cumulative warnings across intentional restarts.
    s.sequence.reset(); s.stamp.reset(); s.session.reset();
    s.conditions = {};
    s.last_arrival = now; s.stale_reported = false;
    s.period = std::isfinite(rate_hz) && rate_hz >= 0.001 && rate_hz <= 1000000
                   ? 1e9L / rate_hz : 0;
    s.silence_ns = silence_ns;
    if (s.period > 0 && silence_ns)
      s.silence_ns = std::max(silence_ns, static_cast<uint64_t>(3 * s.period));
  }

  void disable(Stream stream) { state(stream).silence_ns = 0; }
  void touch(Stream stream, uint64_t now) {
    auto& s = state(stream);
    s.last_arrival = now;
    s.stale_reported = false;
  }
  void session(Stream stream, uint64_t session_id) {
    auto& s = state(stream);
    if (s.session && *s.session != session_id) {
      add(stream, Issue::SessionChanged);
      s.sequence.reset(); s.stamp.reset();
      s.conditions = {};
    }
    s.session = session_id;
  }
  void add(Stream stream, Issue issue, uint64_t count = 1) {
    auto& s = state(stream);
    const auto i = static_cast<size_t>(issue);
    s.pending[i] += count; s.total[i] += count;
  }
  void condition(Stream stream, Issue issue, bool active) {
    auto& previous = state(stream).conditions[static_cast<size_t>(issue)];
    if (active && !previous) add(stream, issue);
    previous = active;
  }
  void resetCounter(Stream stream, Issue issue) {
    state(stream).counters[static_cast<size_t>(issue)].reset();
  }
  void queryError(Stream stream, const std::string& detail) {
    add(stream, Issue::QueryError);
    auto& text = state(stream).detail;
    text = detail.substr(0, 256);
    for (auto& c : text) if (static_cast<unsigned char>(c) < 32) c = ' ';
  }
  // A cumulative device counter's initial value is historical, not new loss.
  void counter(Stream stream, Issue issue, uint64_t value, bool initial_is_new = false) {
    auto& previous = state(stream).counters[static_cast<size_t>(issue)];
    if (previous && value >= *previous) add(stream, issue, value - *previous);
    else if (!previous && initial_is_new) add(stream, issue, value);
    previous = value; // A reset is a new baseline, never unsigned underflow.
  }

  void observe(Stream stream, uint64_t now, uint64_t stamp_ns,
               std::optional<uint64_t> sequence = std::nullopt,
               unsigned sequence_bits = 32, bool equal_stamp_valid = false) {
    auto& s = state(stream);
    touch(stream, now);
    bool reset_sequence = false;
    if (sequence) {
      if (s.sequence) {
        const uint64_t raw_delta = *sequence - *s.sequence;
        const uint64_t delta = sequence_bits == 16 ? static_cast<uint16_t>(raw_delta)
            : sequence_bits == 32 ? static_cast<uint32_t>(raw_delta) : raw_delta;
        const uint64_t half = sequence_bits == 16 ? (1ULL << 15)
            : sequence_bits == 32 ? (1ULL << 31) : (1ULL << 63);
        if (!delta) add(stream, Issue::SequenceDuplicate);
        else if (delta >= half) {
          add(stream, Issue::SequenceReset);
          reset_sequence = true;
        } else if (delta > 1) add(stream, Issue::SequenceMissing, delta - 1);
      }
      s.sequence = sequence;
    }
    // Unknown/unsynchronized timestamps break the time baseline. A reset or
    // backwards clock also re-baselines; do not invent millions of lost samples.
    if (stamp_ns && s.stamp && !reset_sequence) {
      if (stamp_ns < *s.stamp) add(stream, Issue::TimestampBackward);
      else if (stamp_ns == *s.stamp) {
        if (!equal_stamp_valid) add(stream, Issue::TimestampDuplicate);
      } else if (s.period > 0 && stamp_ns - *s.stamp > 1.5L * s.period) {
        add(stream, Issue::TimestampGap);
        const long double estimate = std::round((stamp_ns - *s.stamp) / s.period) - 1;
        add(stream, Issue::EstimatedMissing,
            static_cast<uint64_t>(std::min(estimate, 1e9L)));
      }
    }
    s.stamp = stamp_ns ? std::optional<uint64_t>(stamp_ns) : std::nullopt;
  }

  std::vector<std::string> reports(uint64_t now, bool force = false) {
    std::vector<std::string> result;
    for (size_t i = 0; i < states_.size(); ++i) {
      auto& s = states_[i];
      const bool stale = s.silence_ns && now >= s.last_arrival &&
                         now - s.last_arrival >= s.silence_ns;
      const bool pending = std::any_of(s.pending.begin(), s.pending.end(),
                                      [](uint64_t n) { return n != 0; });
      if (!pending && !(stale && !s.stale_reported)) continue;
      if (!force && s.reported && now - s.last_report < kReportInterval) continue;
      std::ostringstream text;
      text << "sensor-data stream=" << names_[i];
      for (size_t j = 0; j < s.pending.size(); ++j) {
        if (s.pending[j]) text << ' ' << issues_[j] << "+=" << s.pending[j]
                              << " total=" << s.total[j];
      }
      if (stale) text << " no_data_ms=" << (now - s.last_arrival) / 1000000;
      if (s.sequence) text << " last_sequence=" << *s.sequence;
      if (!s.detail.empty()) text << " last_error=" << s.detail;
      text << "; driver-side evidence only (not DDS/rosbag delivery)";
      result.push_back(text.str());
      s.pending = {}; s.last_report = now; s.reported = true;
      s.detail.clear();
      s.stale_reported = stale;
    }
    return result;
  }

 private:
  static constexpr size_t kIssues = static_cast<size_t>(Issue::Count);
  struct State {
    std::optional<uint64_t> sequence, stamp, session;
    std::array<std::optional<uint64_t>, kIssues> counters{};
    std::array<bool, kIssues> conditions{};
    std::array<uint64_t, kIssues> pending{}, total{};
    long double period = 0;
    uint64_t silence_ns = 0, last_arrival = 0, last_report = 0;
    bool reported = false, stale_reported = false;
    std::string detail;
  };
  State& state(Stream stream) { return states_[static_cast<size_t>(stream)]; }
  std::array<State, static_cast<size_t>(Stream::Count)> states_{};
  inline static constexpr const char* names_[] = {
    "camera0", "camera1", "camera2", "camera3", "camera/metadata", "imu0", "imu1",
    "lidar/points", "lidar/imu", "gnss/timing", "gnss/reception", "gnss/observations",
    "gnss/raw", "rtk/cors_rtcm", "gnss+rtk/receiver", "rtk/status", "gnss/rover_rtcm", "transport",
    "imu0+imu1/dispatch", "gnss+cors/raw_dispatch", "gnss/receiver+fix", "rtk/receiver+fix"
  };
  inline static constexpr const char* issues_[] = {
    "sequence_missing", "sequence_duplicate", "sequence_reset_or_reorder", "timestamp_gap",
    "estimated_missing", "timestamp_duplicate", "timestamp_backward", "source_gap_flag",
    "incomplete_frame", "cache_gap", "lost_bytes", "unsynchronized_drop", "dispatch_drop",
    "publish_callback_error", "query_error", "session_changed", "uart_error", "fifo_overflow",
    "rejected", "crc_error", "upstream_drop", "raw_input_stale", "nmea_input_stale", "status_stale"
  };
};
}  // namespace prism_ros_adapter
