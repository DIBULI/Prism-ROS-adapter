#pragma once

#include "prism_ros_adapter/navigation.hpp"

namespace prism_ros_adapter {
// Shared ROS1/ROS2 conversion, also used for SDK -> adapter state.
// Explicit copies keep reserved SDK timing fields out of the public ROS API.

template <typename Output, typename Input>
void copyGnssTimingStatus(Output& output, const Input& input) {
  output.sensor_board_online = static_cast<bool>(input.sensor_board_online);
  output.gnss_input_mode = static_cast<bool>(input.gnss_input_mode);
  output.time_synced = static_cast<bool>(input.time_synced);
  output.offset_fresh = static_cast<bool>(input.offset_fresh);
  output.message_pps_offset_us = static_cast<int64_t>(input.message_pps_offset_us);
  output.last_pps_epoch_us = static_cast<uint64_t>(input.last_pps_epoch_us);
  output.pps_detected = static_cast<bool>(input.pps_detected);
  output.pps_valid = static_cast<bool>(input.pps_valid);
  output.pps_high_width_us = static_cast<uint32_t>(input.pps_high_width_us);
  output.pps_min_high_us = static_cast<uint32_t>(input.pps_min_high_us);
  output.nmea_seen = static_cast<bool>(input.nmea_seen);
  output.nmea_fix_valid = static_cast<bool>(input.nmea_fix_valid);
  output.nmea_dop_valid = static_cast<bool>(input.nmea_dop_valid);
  output.nmea_position_valid = static_cast<bool>(input.nmea_position_valid);
  output.nmea_fix_quality = static_cast<uint8_t>(input.nmea_fix_quality);
  output.nmea_fix_mode = static_cast<uint8_t>(input.nmea_fix_mode);
  output.satellites = static_cast<uint16_t>(input.satellites);
  output.pdop_milli = static_cast<uint32_t>(input.pdop_milli);
  output.hdop_milli = static_cast<uint32_t>(input.hdop_milli);
  output.vdop_milli = static_cast<uint32_t>(input.vdop_milli);
  output.nmea_age_ms = static_cast<uint32_t>(input.nmea_age_ms);
  output.nmea_update_count = static_cast<uint32_t>(input.nmea_update_count);
  output.latitude_e7 = static_cast<int32_t>(input.latitude_e7);
  output.longitude_e7 = static_cast<int32_t>(input.longitude_e7);
  output.altitude_mm = static_cast<int32_t>(input.altitude_mm);
  output.geoid_separation_mm = static_cast<int32_t>(input.geoid_separation_mm);
  output.utc_ms_of_day = static_cast<uint32_t>(input.utc_ms_of_day);
}

template <typename Output, typename Input>
void copyRtkCorrectionStatus(Output& output, const Input& input) {
  output.version = static_cast<uint16_t>(input.version);
  output.flags = static_cast<uint32_t>(input.flags);
  output.error_code = static_cast<int32_t>(input.error_code);
  output.running = static_cast<bool>(input.running);
  output.rover_connected = static_cast<bool>(input.rover_connected);
  output.base_connected = static_cast<bool>(input.base_connected);
  output.host_active = static_cast<bool>(input.host_active);
  output.base_position_valid = static_cast<bool>(input.base_position_valid);
  output.ntrip_configured = static_cast<bool>(input.ntrip_configured);
  output.ntrip_connected = static_cast<bool>(input.ntrip_connected);
  output.base_source = static_cast<uint16_t>(input.base_source);
  output.correction_format = static_cast<uint16_t>(input.correction_format);
  output.solution = static_cast<uint16_t>(input.solution);
  output.host_correction_bytes = static_cast<uint64_t>(input.host_correction_bytes);
  output.rover_bytes = static_cast<uint64_t>(input.rover_bytes);
  output.base_bytes = static_cast<uint64_t>(input.base_bytes);
  output.base_rtcm_messages = static_cast<uint64_t>(input.base_rtcm_messages);
  output.base_observation_epochs = static_cast<uint64_t>(input.base_observation_epochs);
  output.solution_count = static_cast<uint64_t>(input.solution_count);
  output.fix_count = static_cast<uint64_t>(input.fix_count);
  output.float_count = static_cast<uint64_t>(input.float_count);
  output.decoder_errors = static_cast<uint64_t>(input.decoder_errors);
}

template <typename Output, typename Input>
void copyRtkNavigationStatus(Output& output, const Input& input) {
  output.version = static_cast<uint16_t>(input.version);
  output.flags = static_cast<uint32_t>(input.flags);
  output.error_code = static_cast<int32_t>(input.error_code);
  output.solution_valid = static_cast<bool>(input.solution_valid);
  output.base_position_valid = static_cast<bool>(input.base_position_valid);
  output.confidence_valid = static_cast<bool>(input.confidence_valid);
  output.position_jump_valid = static_cast<bool>(input.position_jump_valid);
  output.base_source = static_cast<uint16_t>(input.base_source);
  output.solution = static_cast<uint16_t>(input.solution);
  output.confidence = static_cast<uint16_t>(input.confidence);
  output.satellites = static_cast<uint16_t>(input.satellites);
  output.confidence_score = static_cast<uint16_t>(input.confidence_score);
  output.confidence_reasons = static_cast<uint32_t>(input.confidence_reasons);
  output.base_station_id = static_cast<int32_t>(input.base_station_id);
  output.consecutive_fix_epochs = static_cast<uint32_t>(input.consecutive_fix_epochs);
  output.consecutive_float_epochs = static_cast<uint32_t>(input.consecutive_float_epochs);
  output.solution_epoch_us = static_cast<int64_t>(input.solution_epoch_us);
  output.latitude_deg = static_cast<double>(input.latitude_deg);
  output.longitude_deg = static_cast<double>(input.longitude_deg);
  output.ellipsoidal_height_m = static_cast<double>(input.ellipsoidal_height_m);
  output.east_std_m = static_cast<double>(input.east_std_m);
  output.north_std_m = static_cast<double>(input.north_std_m);
  output.up_std_m = static_cast<double>(input.up_std_m);
  output.differential_age_s = static_cast<double>(input.differential_age_s);
  output.ambiguity_ratio = static_cast<double>(input.ambiguity_ratio);
  output.position_jump_m = static_cast<double>(input.position_jump_m);
  output.solution_count = static_cast<uint64_t>(input.solution_count);
  output.fix_count = static_cast<uint64_t>(input.fix_count);
  output.float_count = static_cast<uint64_t>(input.float_count);
  output.rover_observation_epochs = static_cast<uint64_t>(input.rover_observation_epochs);
  output.base_observation_epochs = static_cast<uint64_t>(input.base_observation_epochs);
  output.decoder_errors = static_cast<uint64_t>(input.decoder_errors);
  output.smoothed_position_valid = static_cast<bool>(input.smoothed_position_valid);
  output.smoothed_solution = static_cast<uint16_t>(input.smoothed_solution);
  output.smoothing_flags = static_cast<uint32_t>(input.smoothing_flags);
  output.smoothed_solution_epoch_us = static_cast<int64_t>(input.smoothed_solution_epoch_us);
  output.smoothed_latitude_deg = static_cast<double>(input.smoothed_latitude_deg);
  output.smoothed_longitude_deg = static_cast<double>(input.smoothed_longitude_deg);
  output.smoothed_ellipsoidal_height_m = static_cast<double>(input.smoothed_ellipsoidal_height_m);
  output.smoothed_east_std_m = static_cast<double>(input.smoothed_east_std_m);
  output.smoothed_north_std_m = static_cast<double>(input.smoothed_north_std_m);
  output.smoothed_up_std_m = static_cast<double>(input.smoothed_up_std_m);
  output.smoothing_reset_count = static_cast<uint64_t>(input.smoothing_reset_count);
  output.smoothing_gated_epoch_count = static_cast<uint64_t>(input.smoothing_gated_epoch_count);
}

template <typename Output, typename Input>
void copyTimeSyncPortStatus(Output& output, const Input& input) {
  output.mode = static_cast<uint32_t>(input.mode);
  output.persisted = static_cast<bool>(input.persisted);
  output.applied = static_cast<bool>(input.applied);
  output.sensor_board_online = static_cast<bool>(input.sensor_board_online);
  output.generation = static_cast<uint32_t>(input.generation);
  output.error_code = static_cast<int32_t>(input.error_code);
}

template <typename Output, typename Input>
void copyRoverRtcmStatus(Output& output, const Input& input) {
  output.version = static_cast<uint16_t>(input.version);
  output.enabled = static_cast<bool>(input.enabled);
  output.buffered_bytes = static_cast<uint32_t>(input.buffered_bytes);
  output.maximum_event_bytes = static_cast<uint32_t>(input.maximum_event_bytes);
  output.dropped_bytes = static_cast<uint64_t>(input.dropped_bytes);
}
}  // namespace prism_ros_adapter
