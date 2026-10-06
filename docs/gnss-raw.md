# Raw GNSS and CORS topics

ROS 1 and ROS 2 publish these additional topics when `gnss_raw_enabled: true`
(default). A matching updated 1.2.0 SDK and device application are required.

| Topic | Type | Content |
|---|---|---|
| `/prism/gnss/raw` | `prism_ros_msgs/GnssRawData` | Original receiver ASCII/binary fragments, including enabled observations and ephemerides |
| `/prism/rtk/cors_rtcm` | `prism_ros_msgs/GnssRawData` | Module-received, CRC-accepted CORS RTCM3 fragments |

The topic prefix remains configurable. Existing GNSS/RTK position, precision,
navigation and rover-RTCM topics are unchanged. These passive queries do not
start RTK/CORS or modify receiver output settings. Query failures publish an
empty `query_failed` marker and do not stop camera, IMU or LiDAR acquisition.

`header.stamp` is host receipt time, not GNSS measurement time. Preserve
`session`, `sequence`, `received_ms`, `flags`, `lost_bytes`, `cache_gap`,
`query_failed` and `adapter_dropped_chunks` with the bytes. Sequence numbers are
global across both channels, so non-consecutive numbers on one topic alone are
not proof of packet loss. Channel-zero gap/error markers appear on both topics.
Reset partial-frame decoders on session changes, gap/error markers, or an
increase in the adapter drop counter. ROS transport/recorder losses are still
possible; reliable QoS and bounded queues do not guarantee disk completeness.

Messages are fragments, not complete frames or parsed measurement arrays.
Decode receiver observation epochs and CRCs before offline processing. CORS
bytes exclude HTTP headers, credentials and corrupt rejected RTCM frames.
Raw GNSS frames can contain exact position; protect recorded bags accordingly.

```bash
# ROS 2
ros2 bag record /prism/gnss/raw /prism/rtk/cors_rtcm /prism/gnss/receiver /prism/rtk/receiver
# ROS 1
rosbag record /prism/gnss/raw /prism/rtk/cors_rtcm /prism/gnss/receiver /prism/rtk/receiver
```

No offline dataset-download service is added to ROS.
