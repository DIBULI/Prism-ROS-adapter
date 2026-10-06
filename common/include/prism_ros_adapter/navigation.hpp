#pragma once

#include <cstdint>
#include <vector>
#include <string>

namespace prism_ros_adapter {

struct GnssRawState {
  uint64_t host_received_ns=0,session=0,sequence=0,received_ms=0,device_monotonic_ms=0,cursor=0;
  uint8_t channel=0;
  uint32_t flags=0;
  uint64_t lost_bytes=0,adapter_dropped_chunks=0;
  bool cache_gap=false,query_failed=false;
  std::vector<uint8_t> data;
};
template<class M> void copyGnssRaw(M& m,const GnssRawState& v) {
  m.session=v.session;m.sequence=v.sequence;m.received_ms=v.received_ms;
  m.device_monotonic_ms=v.device_monotonic_ms;m.cursor=v.cursor;m.channel=v.channel;
  m.flags=v.flags;m.lost_bytes=v.lost_bytes;m.adapter_dropped_chunks=v.adapter_dropped_chunks;
  m.cache_gap=v.cache_gap;m.query_failed=v.query_failed;m.data=v.data;
}

struct GnssTimingStatusState {
  bool sensor_board_online{};
  bool gnss_input_mode{};
  bool time_synced{};
  bool offset_fresh{};
  int64_t message_pps_offset_us{};
  uint64_t last_pps_epoch_us{};
  bool pps_detected{};
  bool pps_valid{};
  uint32_t pps_high_width_us{};
  uint32_t pps_min_high_us{};
  bool nmea_seen{};
  bool nmea_fix_valid{};
  bool nmea_dop_valid{};
  bool nmea_position_valid{};
  uint8_t nmea_fix_quality{};
  uint8_t nmea_fix_mode{};
  uint16_t satellites{};
  uint32_t pdop_milli{};
  uint32_t hdop_milli{};
  uint32_t vdop_milli{};
  uint32_t nmea_age_ms{};
  uint32_t nmea_update_count{};
  int32_t latitude_e7{};
  int32_t longitude_e7{};
  int32_t altitude_mm{};
  int32_t geoid_separation_mm{};
  uint32_t utc_ms_of_day{};
};

struct TimeSyncPortStatusState {
  uint32_t mode{};
  bool persisted{};
  bool applied{};
  bool sensor_board_online{};
  uint32_t generation{};
  int32_t error_code{};
};

struct RoverRtcmStatusState {
  uint16_t version{};
  bool enabled{};
  uint32_t buffered_bytes{};
  uint32_t maximum_event_bytes{};
  uint64_t dropped_bytes{};
};

struct RtcmData {
  uint64_t host_received_ns = 0;
  uint32_t sequence = 0;
  uint32_t flags = 0;
  uint64_t dropped_bytes = 0;
  uint64_t adapter_dropped_chunks = 0;
  std::vector<uint8_t> data;
};
struct GnssReceptionStatusState {
  bool sensor_board_online{};
  bool reception_available{};
  bool raw_data_seen{};
  bool raw_data_fresh{};
  bool nmea_sentence_seen{};
  bool nmea_sentence_fresh{};
  uint32_t raw_age_ms{};
  uint32_t nmea_sentence_age_ms{};
  uint64_t raw_byte_count{};
  uint64_t nmea_sentence_count{};
  uint64_t nmea_rejected_count{};
  uint64_t uart_frame_error_count{};
  uint64_t fifo_overflow_count{};
};

struct TimeSyncRtkStatusState {
  bool linked{};
  bool device_status_fresh{};
  bool control_status_fresh{};
  bool configuration_saved{};
  bool configuration_applied{};
  int32_t error_code{};
  uint32_t age_ms{};
  uint32_t status_age_ms{};
  uint32_t saved_generation{};
  uint32_t applied_generation{};
  uint32_t control_generation{};
  uint8_t control_state{};
  uint8_t control_error{};
  uint8_t device_flags{};
  uint8_t sim{};
  uint8_t registration{};
  uint8_t fix{};
  uint8_t satellites{};
  uint8_t rtcm_format{};
  uint32_t uptime_ms{};
  uint32_t gnss_age_ms{};
  uint32_t rtcm_age_ms{};
  uint32_t network_bytes{};
  uint32_t transmitted_bytes{};
  uint32_t rtcm_frames{};
  uint32_t rtcm_errors{};
  uint32_t upstream_drops{};
  uint32_t current_cors_generation{};
  uint64_t gnss_drained_bytes{};
  uint64_t rtcm_drained_bytes{};
  uint64_t lost_bytes{};
};

struct TimeSyncRtkVersionsState {
  bool linked{};
  uint32_t age_ms{};
  bool application_valid{};
  bool application_diagnostic{};
  uint16_t application_major{};
  uint16_t application_minor{};
  uint16_t application_patch{};
  bool bootloader_valid{};
  bool bootloader_diagnostic{};
  uint16_t bootloader_major{};
  uint16_t bootloader_minor{};
  uint16_t bootloader_patch{};
};

struct TimeSyncCorsStatusState {
  bool linked{};
  bool device_status_fresh{};
  bool control_status_fresh{};
  bool configuration_saved{};
  bool configuration_applied{};
  uint32_t saved_generation{};
  uint32_t applied_generation{};
  bool enabled{};
  bool credentials_present{};
  std::string ip{};
  uint16_t port{};
  std::string mountpoint{};
  std::string username{};
};

struct GnssObservationsState {
  uint64_t cursor{};
  uint64_t device_monotonic_ms{};
  uint64_t session{};
  bool gap{};
  std::vector<uint64_t> sequences{};
  std::vector<uint64_t> received_ms{};
  std::vector<std::string> sentences{};
};

struct ReceiverPositionState {
  bool valid{};
  bool height_valid{};
  bool covariance_valid{};
  bool timestamp_valid{};
  std::string source{};
  std::string solution{};
  std::string time_system{};
  std::string epoch{};
  uint8_t quality{};
  uint16_t satellites{};
  uint32_t age_ms{};
  uint64_t sequence{};
  uint64_t session{};
  uint64_t epoch_us{};
  double latitude_deg{};
  double longitude_deg{};
  double ellipsoidal_height_m{};
  double east_std_m{};
  double north_std_m{};
  double up_std_m{};
};

struct CorsConfiguration {
  bool enabled = false;
  std::string ip, mountpoint, username, password;
  uint16_t port = 0;
};

}  // namespace prism_ros_adapter
