#pragma once

#include "prism_ros_adapter/navigation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace prism_ros_adapter {

// GGA supplies time-of-day only. Use the synchronized PPS date, choosing the
// nearest day across midnight; never substitute the host's date or wall clock.
inline uint64_t gnssEpochUs(const GnssTimingStatusState& s) {
  constexpr uint64_t day = 86400000000ull;
  if (!s.time_synced || !s.last_pps_epoch_us || s.utc_ms_of_day >= 86400000u)
    return 0;
  uint64_t epoch = s.last_pps_epoch_us / day * day +
                   uint64_t(s.utc_ms_of_day) * 1000u;
  if (epoch + day / 2 < s.last_pps_epoch_us) epoch += day;
  else if (epoch > s.last_pps_epoch_us + day / 2 && epoch >= day) epoch -= day;
  return epoch;
}

template <typename Fix>
void noFix(Fix& fix) {
  fix.status.status = -1;
  // The SDK reports total satellites, not the constellation mask.
  fix.status.service = 0;
  fix.latitude = fix.longitude = fix.altitude =
      std::numeric_limits<double>::quiet_NaN();
  std::fill(fix.position_covariance.begin(), fix.position_covariance.end(), 0.0);
  fix.position_covariance_type = 0;
}

template <typename Fix>
void fillGnssFix(Fix& fix, const GnssTimingStatusState& s) {
  noFix(fix);
  if (!s.nmea_seen || !s.nmea_fix_valid || !s.nmea_position_valid ||
      s.nmea_age_ms > 2000 || s.nmea_fix_quality == 0 ||
      std::abs(int64_t(s.latitude_e7)) > 900000000ll ||
      std::abs(int64_t(s.longitude_e7)) > 1800000000ll) return;
  fix.status.status = (s.nmea_fix_quality == 2 || s.nmea_fix_quality == 4 ||
                       s.nmea_fix_quality == 5) ? 2 : 0;
  fix.latitude = s.latitude_e7 * 1e-7;
  fix.longitude = s.longitude_e7 * 1e-7;
  fix.altitude = (int64_t(s.altitude_mm) + s.geoid_separation_mm) * 1e-3;
  // DOP is unitless: it is not a covariance in square metres.
}

template <typename Fix>
void fillReceiverFix(Fix& fix, const ReceiverPositionState& s) {
  noFix(fix);
  if (!s.valid || s.age_ms > 2000 || !std::isfinite(s.latitude_deg) ||
      !std::isfinite(s.longitude_deg) || std::abs(s.latitude_deg) > 90 ||
      std::abs(s.longitude_deg) > 180) return;
  fix.status.status = (s.quality == 2 || s.quality == 4 || s.quality == 5) ? 2 : 0;
  fix.latitude = s.latitude_deg;
  fix.longitude = s.longitude_deg;
  if (s.height_valid && std::isfinite(s.ellipsoidal_height_m))
    fix.altitude = s.ellipsoidal_height_m;
  if (s.covariance_valid && std::isfinite(s.east_std_m) &&
      std::isfinite(s.north_std_m) && std::isfinite(s.up_std_m) &&
      s.east_std_m > 0 && s.north_std_m > 0 && s.up_std_m > 0) {
    fix.position_covariance[0] = s.east_std_m * s.east_std_m;
    fix.position_covariance[4] = s.north_std_m * s.north_std_m;
    fix.position_covariance[8] = s.up_std_m * s.up_std_m;
    fix.position_covariance_type = 2;
  }
}
}  // namespace prism_ros_adapter
