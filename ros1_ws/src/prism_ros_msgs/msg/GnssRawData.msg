# header.stamp is host receipt time, NOT the receiver measurement epoch.
# data contains byte fragments, not complete protocol frames. Preserve order.
# channel: 0 both (gap/error only), 1 receiver, 2 CRC-accepted CORS RTCM3.
# sequence is global across both channels; gaps between different channels are normal.
# On session/cache/source/adapter gap, discard partial frames before decoding.
# flags: 1 upstream gap, 2 link reset, 4 UART error. lost_bytes=0 means unknown.
std_msgs/Header header
uint64 session
uint64 sequence
uint64 received_ms
uint64 device_monotonic_ms
uint64 cursor
uint8 channel
uint32 flags
uint64 lost_bytes
uint64 adapter_dropped_chunks
bool cache_gap
bool query_failed
uint8[] data
