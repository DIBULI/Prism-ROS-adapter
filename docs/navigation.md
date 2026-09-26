# GNSS / RTK interfaces — 1.2.0

The ROS adapter uses Prism-SDK 1.2.0 and Agent 1.2.0. It does not run an RTK
solver. GNSS positions come from receiver GGA; independent receiver RTK
positions come from ADRNAV. There is no raw/smoothed position pair.

ROS 1 message names below use `prism_ros_msgs/Type`; ROS 2 uses
`prism_ros_msgs/msg/Type`, and services use `prism_ros_msgs/srv/Type`.
The default prefix is `/prism`.

## Topics

| Topic | Type | Meaning |
| --- | --- | --- |
| `gnss/timing` | `GnssTimingStatus` | Live external synchronization, PPS presence/validity/width, first-RMC delay, NMEA age and basic fix |
| `gnss/reception` | `GnssReceptionStatus` | Raw UART/NMEA freshness, accepted/rejected counts, UART errors and FIFO overflow |
| `gnss/observations` | `GnssObservations` | Cursor/session/gap and original receiver text sentences, including available RMC/GGA/GST/GSV/GSA/ADRNAV |
| `gnss/receiver` | `ReceiverPosition` | GGA solution, satellites, age, ellipsoidal height and epoch-matched GST standard deviations when available |
| `rtk/receiver` | `ReceiverPosition` | ADRNAV native solution type, satellites, epoch and ENU standard deviations |
| `gnss/fix`, `rtk/fix` | `sensor_msgs/NavSatFix` | Separate receiver positions; covariance diagonal is E², N², U² in m² |
| `rtk/status` | `TimeSyncRtkStatus` | Module link, control state/errors/generations, GNSS, SIM/network, CORS and transfer/drop counters |
| `gnss/rover_rtcm` | `RtcmData` | Optional receiver RTCM3 stream, disabled by default |

Navigation queries are scheduled every 100 ms; module/reception diagnostics
every second. These are polling targets, not a guarantee of new receiver
solutions at that rate. Use `sequence` and `epoch` to identify new solutions.
GGA quality 1/2/4/5 means SINGLE/DGNSS/FIX/FLOAT. `solution` preserves the
receiver's detailed ADRNAV type. NavSatFix alone cannot distinguish FLOAT/FIX.
Positions older than two seconds, invalid receiver solutions, observation gaps
or a new Agent session invalidate the current fix. NavSatFix then reports
`STATUS_NO_FIX` with NaN coordinates, not a stale successful position.

Covariance is reported only with valid positive metre standard deviations;
unknown covariance remains unknown. HDOP is not a metre uncertainty and a
vendor standard deviation is not a calibrated confidence percentage.
GGA GST covariance must match the same UTC epoch; no reuse across epochs.
Constellation service flags are zero because a complete fix constellation mask
is not supplied. Satellite sky clients may parse GSV/GSA from `observations`;
GSA membership does not establish per-satellite RTK participation.

## Timestamp contract

- Observation `received_ms` and `device_monotonic_ms` are Agent monotonic
  milliseconds, not Unix UTC; preserve `session` and `gap` when recording.
- GGA only contains time-of-day. The adapter uses a live synchronized PPS date
  for UTC, including midnight rollover. Without that date, the ROS stamp is
  zero and `timestamp_valid=false`; host wall time is never substituted.
- ADRNAV reports GPS week and millisecond-of-week, retained in `epoch` as
  `week:milliseconds`, with `time_system=GPST`. For a UTC ROS header, explicitly
  configure `gps_utc_leap_seconds` to the verified GPS-minus-UTC offset valid
  for the recording date. Default `-1` leaves its UTC header unknown/zero;
  the native epoch and position remain available. The adapter does not guess
  leap seconds or silently label GPST as UTC.
- `epoch_us`, when `timestamp_valid=true`, is Unix UTC microseconds;
  `time_system` describes the original receiver `epoch` string.
- Rover RTCM `header.stamp` is host receipt time, not an observation epoch.
  The GNSS epochs remain inside the RTCM payload.

## Services

| Path under `/prism` | Service | Request |
| --- | --- | --- |
| `gnss/get_timing` | `GetGnssTiming` | none |
| `gnss/get_reception` | `GetGnssReception` | none |
| `rtk/get_receiver_position` | `GetReceiverPosition` | `rtk`: false=GGA, true=ADRNAV |
| `rtk/get_status` | `GetRtkModuleStatus` | none |
| `rtk/get_versions` | `GetRtkModuleVersions` | none; application/bootloader validity and version components |
| `rtk/get_cors` | `GetCorsConfiguration` | none; password is never returned |
| `rtk/set_cors` | `SetCorsConfiguration` | `confirm`, `enabled`, IPv4 `ip`, `port`, `mountpoint`, `username`, `password`; full replacement |
| `rtk/control` | `ControlRtk` | `confirm`, `command` start/stop, `expected_cors_generation`, `allow_gga`, `timeout_ms` 1..60000 |
| `system/get_timesync_port` | `GetTimeSyncPort` | none |
| `system/set_timesync_port` | `SetTimeSyncPort` | `confirm`, mode 0=input, 1=PPS/NMEA output, 2=RTK |
| `gnss/set_rover_rtcm` | `SetRoverRtcm` | `enable` |
| `camera/set_unified_exposure` | `SetUnifiedExposure` | `enabled`; runtime only, enables all four automatic cameras |

CORS credentials are saved privately on RK by Agent and applied to the module
when it is ready. A successful save does not mean configuration was applied or
that CORS is connected: inspect both generations and status flags. Saving a
new account while RTK is already active can change the active CORS connection.
ROS transports do not encrypt these service arguments by default; use a trusted
network or appropriate ROS security and do not record credential requests.

The module, not this ROS node, logs into NTRIP and feeds corrections to its
GNSS/RTK chip. There is no Host RTCM input service/topic in this release.
To start, first read/display the saved CORS configuration. Pass its exact
`saved_generation` and explicit permission `allow_gga=true` to allow live
position to be sent to that caster. Stale generations are rejected. Stop does
not require GGA permission. Start/stop wait for confirmed terminal control
state; RUNNING does not imply FLOAT/FIX or CORS availability.
Commands are not automatically retried. RTK control and TimeSync mode changes
briefly pause this node's active sensor streams and restore them afterward.
Disconnect external transmitters before selecting output mode.

```bash
ros2 service call /prism/rtk/get_status prism_ros_msgs/srv/GetRtkModuleStatus '{}'
ros2 service call /prism/rtk/get_versions prism_ros_msgs/srv/GetRtkModuleVersions '{}'
ros2 service call /prism/rtk/get_cors prism_ros_msgs/srv/GetCorsConfiguration '{}'
# Only after approving the displayed caster and location disclosure; replace 42:
ros2 service call /prism/rtk/control prism_ros_msgs/srv/ControlRtk \
  '{confirm: true, command: start, expected_cors_generation: 42, allow_gga: true, timeout_ms: 20000}'
ros2 service call /prism/rtk/control prism_ros_msgs/srv/ControlRtk \
  '{confirm: true, command: stop, timeout_ms: 20000}'
ros2 service call /prism/camera/set_unified_exposure prism_ros_msgs/srv/SetUnifiedExposure '{enabled: true}'
ros2 bag record /prism/gnss/observations /prism/gnss/receiver /prism/rtk/receiver \
  /prism/gnss/timing /prism/gnss/reception /prism/rtk/status
```

Use `rosservice call` with the same paths/fields in ROS 1. Connecting does not
start CORS, change TimeSync mode or set device time. Sensor capture retains its
existing default startup behavior. `system/sync_time` remains explicit and is
rejected on RK-local (the container has no independent host UTC reference).

`gnss/observations` is a bounded text-message cache, not an arbitrary binary
stream. `gnss/rover_rtcm` only carries RTCM3 frames the receiver actually emits;
it does not convert proprietary full-frequency observation messages into RTCM.
ROS QoS/queues can also drop messages; these topics are not a lossless raw
serial-port recorder. No firmware-upgrade service is introduced.
