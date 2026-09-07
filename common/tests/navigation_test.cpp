#include "prism_ros_adapter/navigation_fix.hpp"
#include "prism_ros_adapter/navigation_conversion.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

struct Fix {
  struct { int status = 0; unsigned service = 0; } status;
  double latitude = 0, longitude = 0, altitude = 0;
  std::array<double, 9> position_covariance{};
  unsigned position_covariance_type = 0;
};
void require(bool ok) { if (!ok) throw std::runtime_error("navigation assertion failed"); }
bool close(double a, double b) { return std::abs(a - b) < 1e-8; }

int main() {
  using namespace prism_ros_adapter;
  GnssTimingStatusState gnss;
  Fix fix;
  fillGnssFix(fix, gnss);
  require(fix.status.status == -1 && std::isnan(fix.latitude));
  gnss.nmea_seen = gnss.nmea_fix_valid = gnss.nmea_position_valid = true;
  gnss.nmea_fix_quality = 1;
  gnss.latitude_e7 = 311395000;
  gnss.longitude_e7 = 1215649000;
  gnss.altitude_mm = 25000;
  gnss.geoid_separation_mm = 12000;
  gnss.nmea_age_ms = 99;
  fillGnssFix(fix, gnss);
  require(close(fix.latitude, 31.1395) && close(fix.longitude, 121.5649));
  require(close(fix.altitude, 37) && fix.position_covariance_type == 0);
  require(gnssEpochUs(gnss) == 0);  // Never invent the date from the host.
  gnss.time_synced = true;
  gnss.last_pps_epoch_us = 86400000000ull * 20000;
  gnss.utc_ms_of_day = 86399900;
  require(gnssEpochUs(gnss) == gnss.last_pps_epoch_us - 100000);
  gnss.last_pps_epoch_us -= 1000000;
  gnss.utc_ms_of_day = 100;
  require(gnssEpochUs(gnss) == gnss.last_pps_epoch_us + 1100000);
  gnss.utc_ms_of_day = 86400000;
  require(gnssEpochUs(gnss) == 0);
  gnss.nmea_age_ms = 2001;
  fillGnssFix(fix, gnss);
  require(fix.status.status == -1 && std::isnan(fix.altitude));
  GnssTimingStatusState copy;
  gnss.offset_fresh = true;
  gnss.message_pps_offset_us = 800000;
  gnss.pps_valid = true;
  gnss.pps_high_width_us = 499990;
  copyGnssTimingStatus(copy, gnss);
  require(copy.time_synced && copy.offset_fresh && copy.message_pps_offset_us == 800000);
  require(copy.pps_valid && copy.pps_high_width_us == 499990);

  RtkNavigationStatusState rtk;
  fillRtkFix(fix, rtk, false);
  require(fix.status.status == -1);
  rtk.solution_valid = true;
  rtk.solution = 4;
  rtk.latitude_deg = 31.1;
  rtk.longitude_deg = 121.5;
  rtk.ellipsoidal_height_m = 55.3;
  rtk.east_std_m = .02;
  rtk.north_std_m = .03;
  rtk.up_std_m = .05;
  fillRtkFix(fix, rtk, false);
  require(fix.status.status == 2 && close(fix.altitude, 55.3));
  require(fix.position_covariance_type == 2 && close(fix.position_covariance[0], .0004));
  require(close(fix.position_covariance[4], .0009) && close(fix.position_covariance[8], .0025));
  fillRtkFix(fix, rtk, true);
  require(fix.status.status == -1);  // Never fall back from filtered to raw.
  rtk.smoothed_position_valid = true;
  rtk.smoothed_solution = 3;
  rtk.smoothed_latitude_deg = 31.2;
  rtk.smoothed_longitude_deg = 121.6;
  rtk.smoothed_ellipsoidal_height_m = 56;
  fillRtkFix(fix, rtk, true);
  require(fix.status.status == 2 && close(fix.latitude, 31.2));
  require(fix.position_covariance_type == 0);  // Zero/unknown sigma is not perfect precision.
  RtkNavigationStatusState rtk_copy;
  rtk.solution_epoch_us = 1720000000000000;
  rtk.smoothed_solution_epoch_us = rtk.solution_epoch_us - 100000;
  rtk.smoothing_gated_epoch_count = 30;
  copyRtkNavigationStatus(rtk_copy, rtk);
  require(rtk_copy.solution_epoch_us == rtk.solution_epoch_us &&
          rtk_copy.smoothed_solution_epoch_us == rtk.smoothed_solution_epoch_us &&
          rtk_copy.smoothing_gated_epoch_count == 30);
  rtk.latitude_deg = std::numeric_limits<double>::quiet_NaN();
  fillRtkFix(fix, rtk, false);
  require(fix.status.status == -1);
  std::cout << "navigation conversion / invalid fix / ENU covariance / UTC rollover passed\n";
}
