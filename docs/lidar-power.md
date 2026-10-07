# Independent LiDAR standby / wake and MID360S speed

Requires the Agent and SDK LiDAR power extension. Older 1.2.0 builds do not
implement it; an unsupported command must not be treated as success.
Select the actual attached radar using the driver's `lidar_model` parameter
(`mid360`, `mid360s`, or `xt32`). The target address is the saved LiDAR
network configuration. The network must already be configured and ready.

These services never stop/resume streams for you. First explicitly stop all
camera, board-IMU and LiDAR streams, then query or change hardware state:

```sh
ros2 service call /prism/streams/control prism_ros_msgs/srv/ControlStreams \
  "{command: stop, camera: true, board_imu: true, lidar: true}"
ros2 service call /prism/lidar/get_power prism_ros_msgs/srv/GetLidarPower \
  "{timeout_ms: 3000}"
ros2 service call /prism/lidar/set_standby prism_ros_msgs/srv/SetLidarStandby \
  "{confirm: true, standby: true, timeout_ms: 10000}"
# Wake, without starting data capture:
ros2 service call /prism/lidar/set_standby prism_ros_msgs/srv/SetLidarStandby \
  "{confirm: true, standby: false, timeout_ms: 10000}"
```

ROS 1 uses the same service paths and fields:

```sh
rosservice call /prism/lidar/get_power "{timeout_ms: 3000}"
rosservice call /prism/lidar/set_standby "{confirm: true, standby: true, timeout_ms: 10000}"
```

Check `success` first; only then use `state` (0 unknown, 1 running, 2 standby,
3 transitioning, 4 error) and `vendor_state`. MID360 uses IDLE/SAMPLING and
current-work-state readback. XT32 confirms vendor Standby/Operation mode; this
does not independently measure motor RPM or watts. Wake can require additional
rotor startup time. Network electronics remain powered in standby.

An error or timeout does not roll back a command that reached the radar. Query
before retrying; no automatic write retry is performed. Timeout covers the whole
hardware operation (1..30000 ms), not each individual packet.

Normal capture start/stop remains unchanged, including existing Livox startup
initialization. After an explicit XT32 standby, wake explicitly before capture.
Do not invoke other services that pause/resume capture while you intend to keep
the radar in standby. These services do not persist a preferred power mode.

## MID360S normal / low speed

ROS1 and ROS2 provide `lidar/get_speed` and `lidar/set_speed` with both USB and
RK-local transports. Select `lidar_model: mid360s` before launching the driver;
ordinary MID360 and XT32 are rejected. The attached MID360S firmware and device
application must support speed control. An unsupported command is an error.

Stop **all** capture streams first (the command above stops camera, board IMU
and LiDAR). These operations do not stop/resume capture, wake the radar, change
the network, or start CORS. Query and set are serialized with other driver controls.

```sh
# ROS2: query, switch to low speed, then confirm the current hardware state.
ros2 service call /prism/lidar/get_speed prism_ros_msgs/srv/GetLidarSpeed \
  "{timeout_ms: 3000}"
ros2 service call /prism/lidar/set_speed prism_ros_msgs/srv/SetLidarSpeed \
  "{confirm: true, mode: 2, timeout_ms: 5000}"
ros2 service call /prism/lidar/get_speed prism_ros_msgs/srv/GetLidarSpeed \
  "{timeout_ms: 3000}"
# Return to normal speed:
ros2 service call /prism/lidar/set_speed prism_ros_msgs/srv/SetLidarSpeed \
  "{confirm: true, mode: 1, timeout_ms: 5000}"

# ROS1 uses the same fields and service paths:
rosservice call /prism/lidar/get_speed "{timeout_ms: 3000}"
rosservice call /prism/lidar/set_speed "{confirm: true, mode: 2, timeout_ms: 5000}"
```

Only consume `model`, `mode` and `device_type` when `success=true`. Mode `1`
means the standard MID360S motor speed, `2` means low speed; `0` is unknown, not
stopped. Normal speed is not a separate high-speed overclock mode. Both modes
scan; use `set_standby` to stop scanning. This is mode readback, not measured RPM.

`timeout_ms` must be 1..10000 and bounds the hardware operation; the SDK response
wait includes a transport margin and ROS queue time is additional. A failed or
timed-out write may have changed hardware: query before retrying. No automatic
retry, rollback or preferred-speed persistence is provided. No startup YAML
speed setting is applied implicitly; query after a device/radar restart and
explicitly set the required mode while idle. Restart capture explicitly when ready.

Rebuild `prism_ros_msgs`, the driver and clients together after adding these services.
