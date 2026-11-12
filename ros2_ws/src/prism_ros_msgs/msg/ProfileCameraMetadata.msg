# Profile timestamp is source-clock microseconds, not implicitly UTC.
# header.stamp repeats source_timestamp_us in the device clock domain.
# It is NOT UTC unless timestamp_utc_valid. IDs are not interchangeable.
std_msgs/Header header
uint8 camera
uint8 format
uint8 source_bits
uint8 bayer
uint8 timing_mode
uint32 transfer_id
uint32 source_frame_id
uint32 source_frame_step
bool source_frame_valid
uint32 trigger_sequence
uint64 source_timestamp_us
uint64 acquisition_epoch
bool timestamp_utc_valid
bool image_published
uint32 width
uint32 height
uint32 exposure_us
uint32 gain_x1024
