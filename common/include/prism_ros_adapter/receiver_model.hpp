#pragma once

#include "prism_ros_adapter/navigation_fix.hpp"
#include "prism/usb/gnss_plot.hpp"
#include <algorithm>
#include <limits>

namespace prism_ros_adapter {

// Parse the SDK's CRC-validated receiver messages, not an RK positioning solver.
// Observation times are Agent monotonic milliseconds. Only explicit receiver
// epochs can populate ROS UTC timestamps; never substitute host arrival time.
class ReceiverModel {
 public:
  prism::gnss_plot::Model model;
  GnssTimingStatusState timing;
  int32_t gps_utc_leap_seconds = -1;

  void apply(const prism::GnssObservations& batch) {
    if ((model.session && model.session != batch.session) || batch.gap)
      gst_ = {};
    model.apply(batch);
    for (const auto& record : batch.records) {
      const auto& s = record.sentence;
      if (!prism::gnss_plot::checksum(s) || s.empty() || s[0] != '$') continue;
      const auto f = prism::gnss_plot::split(s.substr(1, s.find('*') - 1));
      if (f.size() < 9 || f[0].size() != 5 || f[0].substr(2) != "GST") continue;
      gst_ = {};
      const auto north = prism::gnss_plot::number(f[6], 0);
      const auto east = prism::gnss_plot::number(f[7], 0);
      const auto up = prism::gnss_plot::number(f[8], 0);
      if (!prism::gnss_plot::utcValid(f[1]) || !north || !east || !up ||
          *north <= 0 || *east <= 0 || *up <= 0) continue;
      gst_ = {true, f[1], record.received_ms, *east, *north, *up};
    }
    // ROS does not retain display trajectories in memory.
    model.clearTracks();
  }

  ReceiverPositionState position(bool rtk, uint64_t now) const {
    const auto& p = rtk ? model.rtk : model.gnss;
    ReceiverPositionState out;
    out.source = rtk ? "ADRNAV" : "GGA";
    out.time_system = rtk ? "GPST" : "UTC";
    out.solution = p.solution;
    out.epoch = p.epoch;
    out.sequence = p.sequence;
    out.session = model.session;
    out.age_ms = p.sequence && now >= p.ms
        ? static_cast<uint32_t>(std::min<uint64_t>(now - p.ms, UINT32_MAX))
        : UINT32_MAX;
    out.valid = p.valid && out.age_ms <= 2000;
    out.quality = static_cast<uint8_t>(std::max(0, p.quality));
    out.satellites = static_cast<uint16_t>(std::max(0, p.satellites));
    out.latitude_deg = p.latitude;
    out.longitude_deg = p.longitude;
    out.height_valid = p.height.has_value();
    out.ellipsoidal_height_m = p.height.value_or(0);
    if (rtk && p.east_sigma && p.north_sigma && p.up_sigma) {
      out.east_std_m = *p.east_sigma;
      out.north_std_m = *p.north_sigma;
      out.up_std_m = *p.up_sigma;
      out.covariance_valid = out.valid && out.east_std_m > 0 &&
                             out.north_std_m > 0 && out.up_std_m > 0;
    } else if (!rtk && gst_.valid && sameUtc(gst_.epoch, p.epoch) &&
               prism::gnss_plot::fresh(now, gst_.ms, 2000)) {
      out.east_std_m = gst_.east;
      out.north_std_m = gst_.north;
      out.up_std_m = gst_.up;
      out.covariance_valid = out.valid;
    }
    if (rtk && gps_utc_leap_seconds >= 0 && gps_utc_leap_seconds <= 64) {
      const auto parts = prism::gnss_plot::split(p.epoch, ':');
      if (parts.size() == 2) {
        const int week = prism::gnss_plot::integer(parts[0], 0, 65535);
        const int tow = prism::gnss_plot::integer(parts[1], 0, 604799999);
        if (week >= 0 && tow >= 0)
          out.epoch_us = 315964800000000ull + uint64_t(week) * 604800000000ull +
                         uint64_t(tow) * 1000 - uint64_t(gps_utc_leap_seconds) * 1000000;
      }
    } else if (!rtk && prism::gnss_plot::utcValid(p.epoch)) {
      auto value = timing;
      const auto h = prism::gnss_plot::integer(p.epoch.substr(0, 2), 0, 23);
      const auto m = prism::gnss_plot::integer(p.epoch.substr(2, 2), 0, 59);
      const auto s = prism::gnss_plot::number(p.epoch.substr(4), 0, 59.999999);
      if (h >= 0 && m >= 0 && s) {
        value.utc_ms_of_day = uint32_t((h * 3600 + m * 60) * 1000 + std::llround(*s * 1000));
        out.epoch_us = gnssEpochUs(value);
      }
    }
    out.timestamp_valid = out.epoch_us > 0 && out.epoch_us / 1000000 < 2147483648ull;
    if (!out.timestamp_valid) out.epoch_us = 0;
    return out;
  }

 private:
  struct Gst {
    bool valid = false;
    std::string epoch;
    uint64_t ms = 0;
    double east = 0, north = 0, up = 0;
  } gst_;
  static bool sameUtc(const std::string& a, const std::string& b) {
    if (!prism::gnss_plot::utcValid(a) || !prism::gnss_plot::utcValid(b)) return false;
    auto av = prism::gnss_plot::number(a), bv = prism::gnss_plot::number(b);
    return av && bv && std::abs(*av - *bv) < 0.0005;
  }
};
}  // namespace prism_ros_adapter
