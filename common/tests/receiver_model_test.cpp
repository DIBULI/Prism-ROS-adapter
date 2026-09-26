#include "prism_ros_adapter/receiver_model.hpp"
#include <cstdio>
#include <iostream>
#include <stdexcept>

void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
std::string sentence(const std::string& body, bool extended = false) {
  uint32_t crc = 0;
  for (const char c : body) {
    crc ^= uint8_t(c);
    if (extended) for (int i=0;i<8;++i) crc=(crc>>1)^((crc&1)?0xedb88320u:0);
  }
  char suffix[16]; std::snprintf(suffix,sizeof(suffix),extended?"*%08X":"*%02X",crc);
  return std::string(extended?"#":"$")+body+suffix;
}
int main() {
  using namespace prism_ros_adapter;
  ReceiverModel model;
  prism::GnssObservations batch;
  batch.session=7;batch.cursor=2;batch.device_monotonic_ms=1100;
  batch.records={{1,1000,sentence("GNGGA,120000.1,3100.0000,N,12100.0000,E,5,12,0.8,20,M,10,M,,")},
                 {2,1000,sentence("GNGST,120000.100,0.03,0.04,0.02,0,0.03,0.02,0.05")}};
  model.apply(batch);
  auto gnss=model.position(false,1100);
  check(gnss.valid&&gnss.quality==5&&gnss.ellipsoidal_height_m==30,"GGA FLOAT / ellipsoid height");
  check(gnss.covariance_valid&&gnss.east_std_m==.02&&gnss.north_std_m==.03,"epoch-matched GST");
  check(!gnss.timestamp_valid&&gnss.epoch_us==0,"must not invent UTC date");
  check(!model.position(false,3101).valid,"stale fix");
  batch.cursor=3;batch.device_monotonic_ms=1200;
  batch.records={{3,1200,sentence("GNGST,120001.1,0.03,0.04,0.02,0,0.03,0.02,0.05")}};
  model.apply(batch);
  check(!model.position(false,1200).covariance_valid,"GST from another epoch must not leak");
  std::string body="ADRNAVA,COM1,GPS,FINE,2437,388818100,0,0,18,0;SOL_COMPUTED,NARROW_INT,31,121,20,10,WGS84,0.03,0.02,0.05,0,0,0,20,18,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
  batch.cursor=4;batch.device_monotonic_ms=1300;
  batch.records={{4,1300,sentence(body,true)}};
  model.apply(batch);
  auto rtk=model.position(true,1300);
  check(rtk.valid&&rtk.quality==4&&rtk.satellites==18,"native ADRNAV FIX");
  check(rtk.covariance_valid&&rtk.east_std_m==.02,"ADRNAV ENU sigma");
  check(rtk.time_system=="GPST"&&!rtk.timestamp_valid,"GPST must not be mislabeled UTC");
  model.gps_utc_leap_seconds=18;
  rtk=model.position(true,1300);
  check(rtk.timestamp_valid&&rtk.epoch_us==315964800000000ull+2437ull*604800000000ull+388818100000ull-18000000ull,"explicit GPST conversion");
  batch.cursor=5;batch.records={{5,1301,"$GNGGA,invalid*00"}};batch.device_monotonic_ms=1301;
  model.apply(batch);check(model.model.rejected==1,"checksum rejected");
  batch.records.clear();batch.gap=true;batch.device_monotonic_ms=1400;model.apply(batch);
  check(!model.position(true,1400).valid&&!model.position(false,1400).valid,"gap invalidation");
  batch.gap=false;batch.session=8;batch.cursor=0;model.apply(batch);
  check(!model.position(true,1400).valid&&model.position(true,1400).sequence==0,"session reset");
  std::cout << "receiver GGA/GST/ADRNAV, staleness, UTC/GPST, CRC and session tests passed\n";
}
