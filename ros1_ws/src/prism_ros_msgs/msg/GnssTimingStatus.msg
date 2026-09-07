# Live external GNSS sync, not the sensor-board internal clock ownership flag.
# message_pps_offset_us: first valid RMC after PPS; use only if offset_fresh (0..800000 us).
# altitude_mm is MSL height; add geoid_separation_mm for ellipsoidal height.
bool sensor_board_online
bool gnss_input_mode
bool time_synced
bool offset_fresh
int64 message_pps_offset_us
uint64 last_pps_epoch_us
bool pps_detected
bool pps_valid
uint32 pps_high_width_us
uint32 pps_min_high_us
bool nmea_seen
bool nmea_fix_valid
bool nmea_dop_valid
bool nmea_position_valid
uint8 nmea_fix_quality
uint8 nmea_fix_mode
uint16 satellites
uint32 pdop_milli
uint32 hdop_milli
uint32 vdop_milli
uint32 nmea_age_ms
uint32 nmea_update_count
int32 latitude_e7
int32 longitude_e7
int32 altitude_mm
int32 geoid_separation_mm
uint32 utc_ms_of_day
