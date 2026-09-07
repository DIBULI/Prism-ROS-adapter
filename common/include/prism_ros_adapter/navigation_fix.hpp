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
void fillRtkFix(Fix& fix, const RtkNavigationStatusState& s, bool smoothed) {
  noFix(fix);
  if (!(smoothed ? s.smoothed_position_valid : s.solution_valid)) return;
  const auto solution = smoothed ? s.smoothed_solution : s.solution;
  const auto latitude = smoothed ? s.smoothed_latitude_deg : s.latitude_deg;
  const auto longitude = smoothed ? s.smoothed_longitude_deg : s.longitude_deg;
  const auto altitude = smoothed ? s.smoothed_ellipsoidal_height_m : s.ellipsoidal_height_m;
  if (solution == 0 || !std::isfinite(latitude) || !std::isfinite(longitude) ||
      !std::isfinite(altitude) || std::abs(latitude) > 90 || std::abs(longitude) > 180)
    return;
  fix.status.status = (solution >= 2 && solution <= 4) ? 2 : 0;
  fix.latitude = latitude;
  fix.longitude = longitude;
  fix.altitude = altitude;
  const double east = smoothed ? s.smoothed_east_std_m : s.east_std_m;
  const double north = smoothed ? s.smoothed_north_std_m : s.north_std_m;
  const double up = smoothed ? s.smoothed_up_std_m : s.up_std_m;
  if (std::isfinite(east) && std::isfinite(north) && std::isfinite(up) &&
      east > 0 && north > 0 && up > 0) {
    fix.position_covariance[0] = east * east;
    fix.position_covariance[4] = north * north;
    fix.position_covariance[8] = up * up;
    fix.position_covariance_type = 2;  // DIAGONAL_KNOWN, ENU order.
  }
}
}  // namespace prism_ros_adapter
