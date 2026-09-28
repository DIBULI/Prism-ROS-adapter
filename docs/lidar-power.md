# Independent LiDAR standby / wake

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
