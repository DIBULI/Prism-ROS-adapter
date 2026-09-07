#pragma once

#include <cstdint>
#include <vector>

namespace prism_ros_adapter {

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

struct RtkCorrectionStatusState {
  uint16_t version{};
  uint32_t flags{};
  int32_t error_code{};
  bool running{};
  bool rover_connected{};
  bool base_connected{};
  bool host_active{};
  bool base_position_valid{};
  bool ntrip_configured{};
  bool ntrip_connected{};
  uint16_t base_source{};
  uint16_t correction_format{};
  uint16_t solution{};
  uint64_t host_correction_bytes{};
  uint64_t rover_bytes{};
  uint64_t base_bytes{};
  uint64_t base_rtcm_messages{};
  uint64_t base_observation_epochs{};
  uint64_t solution_count{};
  uint64_t fix_count{};
  uint64_t float_count{};
  uint64_t decoder_errors{};
};

struct RtkNavigationStatusState {
  uint16_t version{};
  uint32_t flags{};
  int32_t error_code{};
  bool solution_valid{};
  bool base_position_valid{};
  bool confidence_valid{};
  bool position_jump_valid{};
  uint16_t base_source{};
  uint16_t solution{};
  uint16_t confidence{};
  uint16_t satellites{};
  uint16_t confidence_score{};
  uint32_t confidence_reasons{};
  int32_t base_station_id{};
  uint32_t consecutive_fix_epochs{};
  uint32_t consecutive_float_epochs{};
  int64_t solution_epoch_us{};
  double latitude_deg{};
  double longitude_deg{};
  double ellipsoidal_height_m{};
  double east_std_m{};
  double north_std_m{};
  double up_std_m{};
  double differential_age_s{};
  double ambiguity_ratio{};
  double position_jump_m{};
  uint64_t solution_count{};
  uint64_t fix_count{};
  uint64_t float_count{};
  uint64_t rover_observation_epochs{};
  uint64_t base_observation_epochs{};
  uint64_t decoder_errors{};
  bool smoothed_position_valid{};
  uint16_t smoothed_solution{};
  uint32_t smoothing_flags{};
  int64_t smoothed_solution_epoch_us{};
  double smoothed_latitude_deg{};
  double smoothed_longitude_deg{};
  double smoothed_ellipsoidal_height_m{};
  double smoothed_east_std_m{};
  double smoothed_north_std_m{};
  double smoothed_up_std_m{};
  uint64_t smoothing_reset_count{};
  uint64_t smoothing_gated_epoch_count{};
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
}  // namespace prism_ros_adapter
