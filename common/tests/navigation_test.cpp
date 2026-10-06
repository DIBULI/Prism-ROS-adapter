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
  GnssRawState raw, raw_copy;
  raw.session=123; raw.sequence=456; raw.received_ms=789;
  raw.device_monotonic_ms=800; raw.cursor=460;
  raw.channel=2; raw.flags=5; raw.lost_bytes=17; raw.adapter_dropped_chunks=9;
  raw.cache_gap=true; raw.query_failed=true; raw.data={0,0xd3,0xff,0x00,0xaa};
  copyGnssRaw(raw_copy,raw);
  require(raw_copy.session==123 && raw_copy.sequence==456 && raw_copy.received_ms==789);
  require(raw_copy.device_monotonic_ms==800 && raw_copy.cursor==460 && raw_copy.channel==2);
  require(raw_copy.flags==5 && raw_copy.lost_bytes==17 && raw_copy.adapter_dropped_chunks==9);
  require(raw_copy.cache_gap && raw_copy.query_failed && raw_copy.data==raw.data);
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

  ReceiverPositionState rtk;
  fillReceiverFix(fix, rtk);
  require(fix.status.status == -1);
  rtk.valid = rtk.height_valid = rtk.covariance_valid = true;
  rtk.quality = 4;
  rtk.latitude_deg = 31.1;
  rtk.longitude_deg = 121.5;
  rtk.ellipsoidal_height_m = 55.3;
  rtk.east_std_m = .02;
  rtk.north_std_m = .03;
  rtk.up_std_m = .05;
  fillReceiverFix(fix, rtk);
  require(fix.status.status == 2 && close(fix.altitude, 55.3));
  require(fix.position_covariance_type == 2 && close(fix.position_covariance[0], .0004));
  require(close(fix.position_covariance[4], .0009) && close(fix.position_covariance[8], .0025));
  rtk.age_ms = 2001;
  fillReceiverFix(fix, rtk);
  require(fix.status.status == -1 && std::isnan(fix.latitude));
  rtk.age_ms = 10;
  rtk.quality = 5;
  rtk.covariance_valid = false;
  fillReceiverFix(fix, rtk);
  require(fix.status.status == 2 && fix.position_covariance_type == 0);
  ReceiverPositionState rtk_copy;
  rtk.epoch_us = 1720000000000000;
  rtk.solution = "NARROW_FLOAT";
  copyReceiverPosition(rtk_copy, rtk);
  require(rtk_copy.epoch_us == rtk.epoch_us && rtk_copy.solution == "NARROW_FLOAT");
  rtk.latitude_deg = std::numeric_limits<double>::quiet_NaN();
  fillReceiverFix(fix, rtk);
  require(fix.status.status == -1);
  std::cout << "navigation conversion / invalid fix / ENU covariance / UTC rollover passed\n";
}
