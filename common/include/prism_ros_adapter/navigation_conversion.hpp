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
template <typename Output, typename Input>
void copyGnssReceptionStatus(Output& output, const Input& input) {
  output.sensor_board_online = input.sensor_board_online;
  output.reception_available = input.reception_available;
  output.raw_data_seen = input.raw_data_seen;
  output.raw_data_fresh = input.raw_data_fresh;
  output.nmea_sentence_seen = input.nmea_sentence_seen;
  output.nmea_sentence_fresh = input.nmea_sentence_fresh;
  output.raw_age_ms = input.raw_age_ms;
  output.nmea_sentence_age_ms = input.nmea_sentence_age_ms;
  output.raw_byte_count = input.raw_byte_count;
  output.nmea_sentence_count = input.nmea_sentence_count;
  output.nmea_rejected_count = input.nmea_rejected_count;
  output.uart_frame_error_count = input.uart_frame_error_count;
  output.fifo_overflow_count = input.fifo_overflow_count;
}

template <typename Output, typename Input>
void copyTimeSyncRtkStatus(Output& output, const Input& input) {
  output.linked = input.linked;
  output.device_status_fresh = input.device_status_fresh;
  output.control_status_fresh = input.control_status_fresh;
  output.configuration_saved = input.configuration_saved;
  output.configuration_applied = input.configuration_applied;
  output.error_code = input.error_code;
  output.age_ms = input.age_ms;
  output.status_age_ms = input.status_age_ms;
  output.saved_generation = input.saved_generation;
  output.applied_generation = input.applied_generation;
  output.control_generation = input.control_generation;
  output.control_state = input.control_state;
  output.control_error = input.control_error;
  output.device_flags = input.device_flags;
  output.sim = input.sim;
  output.registration = input.registration;
  output.fix = input.fix;
  output.satellites = input.satellites;
  output.rtcm_format = input.rtcm_format;
  output.uptime_ms = input.uptime_ms;
  output.gnss_age_ms = input.gnss_age_ms;
  output.rtcm_age_ms = input.rtcm_age_ms;
  output.network_bytes = input.network_bytes;
  output.transmitted_bytes = input.transmitted_bytes;
  output.rtcm_frames = input.rtcm_frames;
  output.rtcm_errors = input.rtcm_errors;
  output.upstream_drops = input.upstream_drops;
  output.current_cors_generation = input.current_cors_generation;
  output.gnss_drained_bytes = input.gnss_drained_bytes;
  output.rtcm_drained_bytes = input.rtcm_drained_bytes;
  output.lost_bytes = input.lost_bytes;
}

template <typename Output, typename Input>
void copyTimeSyncRtkVersions(Output& output, const Input& input) {
  output.linked = input.linked;
  output.age_ms = input.age_ms;
  output.application_valid = input.application_valid;
  output.application_diagnostic = input.application_diagnostic;
  output.application_major = input.application_major;
  output.application_minor = input.application_minor;
  output.application_patch = input.application_patch;
  output.bootloader_valid = input.bootloader_valid;
  output.bootloader_diagnostic = input.bootloader_diagnostic;
  output.bootloader_major = input.bootloader_major;
  output.bootloader_minor = input.bootloader_minor;
  output.bootloader_patch = input.bootloader_patch;
}

template <typename Output, typename Input>
void copyTimeSyncCorsStatus(Output& output, const Input& input) {
  output.linked = input.linked;
  output.device_status_fresh = input.device_status_fresh;
  output.control_status_fresh = input.control_status_fresh;
  output.configuration_saved = input.configuration_saved;
  output.configuration_applied = input.configuration_applied;
  output.saved_generation = input.saved_generation;
  output.applied_generation = input.applied_generation;
  output.enabled = input.enabled;
  output.credentials_present = input.credentials_present;
  output.ip = input.ip;
  output.port = input.port;
  output.mountpoint = input.mountpoint;
  output.username = input.username;
}

template <typename Output, typename Input>
void copyGnssObservations(Output& output, const Input& input) {
  output.cursor = input.cursor;
  output.device_monotonic_ms = input.device_monotonic_ms;
  output.session = input.session;
  output.gap = input.gap;
  output.sequences = input.sequences;
  output.received_ms = input.received_ms;
  output.sentences = input.sentences;
}

template <typename Output, typename Input>
void copyReceiverPosition(Output& output, const Input& input) {
  output.valid = input.valid;
  output.height_valid = input.height_valid;
  output.covariance_valid = input.covariance_valid;
  output.timestamp_valid = input.timestamp_valid;
  output.source = input.source;
  output.solution = input.solution;
  output.time_system = input.time_system;
  output.epoch = input.epoch;
  output.quality = input.quality;
  output.satellites = input.satellites;
  output.age_ms = input.age_ms;
  output.sequence = input.sequence;
  output.session = input.session;
  output.epoch_us = input.epoch_us;
  output.latitude_deg = input.latitude_deg;
  output.longitude_deg = input.longitude_deg;
  output.ellipsoidal_height_m = input.ellipsoidal_height_m;
  output.east_std_m = input.east_std_m;
  output.north_std_m = input.north_std_m;
  output.up_std_m = input.up_std_m;
}
}  // namespace prism_ros_adapter
