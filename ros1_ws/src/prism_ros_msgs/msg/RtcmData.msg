# header.stamp is HOST receipt time, NOT the GNSS observation epoch.
# corrections input: 1..16384 raw RTCM2.x/RTCM3 bytes; header/metadata ignored.
# rover output: sequence/flags/dropped_bytes come from Agent; data is unchanged RTCM3.
std_msgs/Header header
uint32 sequence
uint32 flags
uint64 dropped_bytes
uint64 adapter_dropped_chunks
uint8[] data
