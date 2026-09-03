# Prism ROS Adapter

[![ROS release tag test](https://github.com/DIBULI/Prism-ROS-adapter/actions/workflows/release-tag-test.yml/badge.svg)](https://github.com/DIBULI/Prism-ROS-adapter/actions/workflows/release-tag-test.yml)

Prism ROS Adapter exposes a DIBULI Prism connected through the Prism USB SDK
as standard ROS topics. The repository contains one ROS-independent USB driver
core, native wrappers for ROS 1 and ROS 2, and the runtime-only Prism USB SDK
Git submodule needed to build them on the supported Ubuntu releases.

## Supported ROS releases

| ROS generation | Distribution | Base operating system | Status |
| --- | --- | --- | --- |
| ROS 1 | Noetic Ninjemys | Ubuntu 20.04 | Supported |
| ROS 2 | Humble Hawksbill | Ubuntu 22.04 | Supported |
| ROS 2 | Jazzy Jalisco | Ubuntu 24.04 | Supported |
| ROS 2 | Kilted Kaiju | Ubuntu 24.04 | Supported |
| ROS 2 | Lyrical Luth | Ubuntu 26.04 | Supported |
| ROS 2 | Rolling Ridley | Ubuntu 26.04 currently | Supported, continuously changing |

ROS 1 releases other than Noetic and end-of-life ROS 2 releases are not
supported. x86-64 uses a Prism USB SDK shared library built for each base
operating system. ARM64 uses one static SDK archive and resolves OpenSSL,
libusb, libstdc++ and glibc while building in the target ROS environment. This
is the same cross-Ubuntu strategy used by Prism Viewer.

## Prism SDK Git submodule

This repository pins Prism SDK distribution `1.0.2` as the
`third_party/Prism-SDK` Git submodule. That distribution contains the Host SDK
1.0.0 runtime/ABI required by Agent 1.0.0. Clone recursively:

```bash
git clone --recurse-submodules git@github.com:DIBULI/Prism-ROS-adapter.git
```

For an existing checkout, initialize the exact pinned SDK commit with:

```bash
git submodule update --init --recursive
```

The SDK repository contains only:

- public SDK headers;
- the platform-specific shared or static Prism USB SDK library;
- exported CMake package configuration files; and
- the Prism USB udev rule.

The Prism Agent and USB SDK implementation source are not included. Neither
the local Docker build nor release CI checks out or compiles Agent source.
The supported binary mapping is:

| Architecture | ROS distribution | SDK submodule runtime prefix |
| --- | --- | --- |
| x86-64 | Noetic | `runtime/ros/ubuntu-20.04-x86_64` |
| x86-64 | Humble | `runtime/ros/ubuntu-22.04-x86_64` |
| x86-64 | Jazzy, Kilted | `runtime/ros/ubuntu-24.04-x86_64` |
| x86-64 | Lyrical, Rolling | `runtime/ros/ubuntu-26.04-x86_64` |
| ARM64 | All supported distributions | `runtime/ros/linux-arm64` |

All five payloads are part of the pinned Prism SDK `1.0.2` distribution and
include the production Host SDK 1.0.0 runtime and `800 Hz` IMU update. The
build scripts detect x86-64 or ARM64 automatically;
`PRISM_ROS_ARCH=arm64` can be used when building ARM64 through emulation on an
x86-64 host. Verify the submodule and all ROS runtime prefixes with:

```bash
./scripts/verify_sdk_submodule.sh all
```

## Published topics

| Topic | Type | Description |
| --- | --- | --- |
| `/prism/camera0/image/compressed` ... `/prism/camera3/image/compressed` | `sensor_msgs/CompressedImage` | Four original MJPEG images; no decode/re-encode |
| `/prism/camera/metadata` | `prism_ros_msgs/CameraFrameMetadata` | Shared trigger timestamp, frame IDs, exposure and gains for all cameras |
| `/prism/imu0/data`, `/prism/imu1/data` | `sensor_msgs/Imu` | Board IMUs in m/s² and rad/s; only detected IMUs publish data |
| `/prism/lidar/points` | `sensor_msgs/PointCloud2` | Mid-360/Mid-360S 10 Hz clouds with `x`, `y`, `z`, `intensity`, `tag`, and nanosecond `offset_time` fields |
| `/prism/lidar/imu` | `sensor_msgs/Imu` | LiDAR-integrated IMU in m/s² and rad/s |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | USB, sensor-board, stream counters and drop counters |

ROS 2 publishes the large compressed-image topics with reliable, volatile QoS
and depth 2. Camera metadata, both IMU families and point clouds use the normal
best-effort sensor-data profile; diagnostics are reliable. A custom ROS 2
camera subscriber should therefore request reliable QoS. This preserves every
complete MJPEG sample without applying back-pressure to the high-rate IMU and
LiDAR topics.

The camera stamp is the sensor-board trigger edge. The adapter combines the
small Agent LiDAR transport batches into fixed 100 ms windows and publishes
`/prism/lidar/points` at 10 Hz. A cloud's header stamp is its first point time;
each point carries a `uint32 offset_time` in nanoseconds relative to that stamp,
so the first point always has an offset of zero.
The point time inside each source batch is derived from the SDK-provided
first-to-last span, preserving the normal Mid-360 5 us point cadence. The
adapter does not alter point coordinates or perform motion deskew.

LiDAR transports expose both a raw PTP timestamp and an SDK normalized
timestamp; the adapter automatically selects the representation that matches
the camera/board-IMU device clock. This comparison uses monotonic arrival
intervals and never uses the host wall clock. By default, samples that cannot
be placed in the common device time domain are dropped so camera, board IMU,
LiDAR points and LiDAR IMU do not silently mix clock domains.

The adapter intentionally publishes compressed camera images. Consumers that
need raw images can use `image_transport`/`compressed_image_transport`. Camera
calibration is not invented by the driver; publish the calibrated
`CameraInfo` from your calibration package.

## Services

ROS 1 and ROS 2 expose the same device-control services. Every SDK command is
serialized onto the USB receive thread, so a service callback never races the
camera, IMU, or LiDAR stream reader.

| Service | Type | Description |
| --- | --- | --- |
| `/prism/camera/get_exposure` | `prism_ros_msgs/GetExposure` | Read the automatic-camera mask, target brightness, four camera values, and shared limits |
| `/prism/camera/set_target_brightness` | `prism_ros_msgs/SetTargetBrightness` | Set the shared automatic-exposure target brightness (1–255) |
| `/prism/camera/set_exposure` | `prism_ros_msgs/SetCameraExposure` | Select automatic or manual exposure for one camera (index 0–3); manual mode accepts exposure in microseconds and gain in x1024 units |
| `/prism/camera/set_exposure_limits` | `prism_ros_msgs/SetExposureLimits` | Set the runtime automatic-exposure time/gain limits shared by all cameras |
| `/prism/system/sync_time` | `prism_ros_msgs/SyncSystemTime` | Make the host wall clock authoritative for the RK system and hardware clocks, then verify the result |
| `/prism/device/get_info` | `prism_ros_msgs/GetDeviceInfo` | Read device identity, USB link, sensor-board health, detected sensors, and Host SDK/Agent/sensor-board versions |
| `/prism/device/get_configuration` | `prism_ros_msgs/GetDeviceConfiguration` | Read the persisted Camera FPS, board-IMU rate, MJPEG quality, and configuration generation |
| `/prism/device/set_configuration` | `prism_ros_msgs/SetDeviceConfiguration` | Persist any selected subset of Camera FPS, board-IMU rate, and MJPEG quality |
| `/prism/lidar/get_status` | `prism_ros_msgs/GetLidarStatus` | Read the live LiDAR model, connection/receive state, address, serial, packet/point counters, and errors |
| `/prism/lidar/get_network` | `prism_ros_msgs/GetLidarNetwork` | Read the persisted LiDAR network configuration and current interface/link/subnet/reachability state |
| `/prism/lidar/set_network` | `prism_ros_msgs/SetLidarNetwork` | Persist LiDAR enable, host IPv4, netmask, and LiDAR IPv4 settings |
| `/prism/lidar/probe_network` | `prism_ros_msgs/ProbeLidarNetwork` | Apply/check the configured host interface and test whether the LiDAR target is reachable |
| `/prism/streams/get_state` | `prism_ros_msgs/GetStreamState` | Read enabled and active state for camera, board IMU, and LiDAR streams |
| `/prism/streams/control` | `prism_ros_msgs/ControlStreams` | Start, stop, or restart the selected camera, board-IMU, and LiDAR streams |
| `/prism/wifi/get_hotspot` | `prism_ros_msgs/GetWifiHotspot` | Read Wi-Fi interface presence, persisted enable state, AP/DHCP runtime state, SSID, and address |
| `/prism/wifi/set_hotspot` | `prism_ros_msgs/SetWifiHotspot` | Persistently enable or disable the RK Wi-Fi hotspot |

Exposure changes are runtime-only and are not persisted by the Agent. Service
responses contain the values actually accepted by the device, including the
effective maximum exposure after the active FPS headroom is applied. When
switching a camera back to automatic mode, `exposure_time_us: 0` and
`gain_x1024: 0` preserve that camera's stored manual values.

System time synchronization requires `confirm: true`. It temporarily pauses
all active camera, board-IMU, and LiDAR streams because the USB SDK only permits
clock synchronization while idle. The adapter restores the original stream
configuration before returning. Make sure the ROS host clock is correct before
calling it.

Persistent device, LiDAR-network, and Wi-Fi writes also require `confirm: true`.
Device configuration currently accepts Camera FPS 1–30, board-IMU rate
800 Hz, and MJPEG quality 1–99; unsupported values are rejected by the SDK.
The request has one `set_*` selector per device field, so omitted fields remain
unchanged. LiDAR network and Wi-Fi control commands require an idle USB session;
the adapter pauses all active streams, performs the operation, and restores the
requested stream state before responding. The corresponding status messages
report persistence generations and device/Agent error details.

`/prism/streams/control` accepts `command: start`, `stop`, or `restart` and at
least one selected stream. Camera and board IMU share one sensor-board capture
session, so every stream transition is implemented as one coordinated stop and
restart; the final enabled/active state still follows the requested selection.
Stopping all streams leaves the node and its services running.

Firmware inspection or upgrade is intentionally not exposed as a ROS service.

### Service request reference

All responses begin with `success` and `message`. Query responses then contain
an `info`, `configuration`, `status`, or `state` message with the complete
device result. A command can be transported successfully by ROS while the
device rejects it, so callers must check `success` and not only the ROS CLI
exit status.

| Service | Request fields | Usage notes |
| --- | --- | --- |
| `device/get_info` | none | Version strings distinguish Host SDK, Agent, sensor-board, and combined versions. Presence, receiving, synchronization, and initialization masks describe each detected sensor. |
| `device/get_configuration` | none | Returns the current persisted/default Camera FPS, board-IMU rate, MJPEG quality, and generation. |
| `device/set_configuration` | `confirm`; `set_camera_fps` + `camera_fps`; `set_imu_rate_hz` + `imu_rate_hz`; `set_mjpeg_quality` + `mjpeg_quality` | Set at least one selector. Unselected fields are preserved. The currently supported ranges are FPS 1–30, IMU 800 Hz, and MJPEG quality 1–99. |
| `lidar/get_status` | none | Safe during streaming; returns model, live receive state, serial/IP, and counters. |
| `lidar/get_network` | none | Returns saved network values and the current `end0`-side interface/link status. It briefly pauses and restores streams. |
| `lidar/set_network` | `confirm`, `enabled`, `host_ip`, `netmask`, `lidar_ip` | Persists all LiDAR-network fields as one configuration. Select the LiDAR model separately with the `lidar_model` startup parameter. |
| `lidar/probe_network` | none | Applies/checks the host-side configuration and reports `same_subnet` and `target_reachable`. |
| `streams/get_state` | none | `*_enabled` is the requested runtime configuration; `*_active` confirms that the SDK stream actually started. |
| `streams/control` | `command`, `camera`, `board_imu`, `lidar` | `command` is exactly `start`, `stop`, or `restart`; select at least one stream. `restart` requires every selected stream to be enabled already. |
| `wifi/get_hotspot` | none | Returns interface presence, persisted enable, AP/DHCP runtime state, SSID, and address. It briefly pauses and restores streams. |
| `wifi/set_hotspot` | `confirm`, `enabled` | Persists hotspot enable/disable. `present: false` means the device has no controllable Wi-Fi interface. |

The device-configuration, LiDAR-network, and Wi-Fi write services reject
`confirm: false` before sending a write to the device. Errors such as an
unsupported IMU rate or invalid IPv4 address are returned in `message`, and
the adapter attempts to restore the previous active streams before returning.

ROS 2 examples:

```bash
ros2 service call /prism/camera/get_exposure \
  prism_ros_msgs/srv/GetExposure '{}'
ros2 service call /prism/camera/set_target_brightness \
  prism_ros_msgs/srv/SetTargetBrightness '{target_brightness: 35}'
ros2 service call /prism/camera/set_exposure \
  prism_ros_msgs/srv/SetCameraExposure \
  '{camera_index: 0, automatic: false, exposure_time_us: 5000, gain_x1024: 1024}'
ros2 service call /prism/system/sync_time \
  prism_ros_msgs/srv/SyncSystemTime '{confirm: true}'
ros2 service call /prism/device/get_info \
  prism_ros_msgs/srv/GetDeviceInfo '{}'
ros2 service call /prism/device/get_configuration \
  prism_ros_msgs/srv/GetDeviceConfiguration '{}'
ros2 service call /prism/device/set_configuration \
  prism_ros_msgs/srv/SetDeviceConfiguration \
  '{confirm: true, set_camera_fps: true, camera_fps: 20, set_imu_rate_hz: true, imu_rate_hz: 800, set_mjpeg_quality: true, mjpeg_quality: 88}'
ros2 service call /prism/lidar/get_status \
  prism_ros_msgs/srv/GetLidarStatus '{}'
ros2 service call /prism/lidar/get_network \
  prism_ros_msgs/srv/GetLidarNetwork '{}'
ros2 service call /prism/lidar/set_network \
  prism_ros_msgs/srv/SetLidarNetwork \
  '{confirm: true, enabled: true, host_ip: 192.168.1.5, netmask: 255.255.255.0, lidar_ip: 192.168.1.194}'
ros2 service call /prism/lidar/probe_network \
  prism_ros_msgs/srv/ProbeLidarNetwork '{}'
ros2 service call /prism/streams/control \
  prism_ros_msgs/srv/ControlStreams \
  '{command: restart, camera: true, board_imu: true, lidar: true}'
ros2 service call /prism/streams/get_state \
  prism_ros_msgs/srv/GetStreamState '{}'
ros2 service call /prism/streams/control \
  prism_ros_msgs/srv/ControlStreams \
  '{command: stop, camera: false, board_imu: false, lidar: true}'
ros2 service call /prism/streams/control \
  prism_ros_msgs/srv/ControlStreams \
  '{command: start, camera: false, board_imu: false, lidar: true}'
ros2 service call /prism/wifi/get_hotspot \
  prism_ros_msgs/srv/GetWifiHotspot '{}'
ros2 service call /prism/wifi/set_hotspot \
  prism_ros_msgs/srv/SetWifiHotspot '{confirm: true, enabled: true}'
```

Use the corresponding ROS 1 service names without `/srv/`, for example:

```bash
rosservice call /prism/camera/get_exposure
rosservice call /prism/camera/set_target_brightness "target_brightness: 35"
rosservice call /prism/system/sync_time "confirm: true"
rosservice call /prism/device/get_info
rosservice call /prism/streams/get_state
rosservice call /prism/lidar/get_status
rosservice call /prism/lidar/get_network
rosservice call /prism/lidar/set_network \
  "confirm: true
enabled: true
host_ip: '192.168.1.5'
netmask: '255.255.255.0'
lidar_ip: '192.168.1.194'"
rosservice call /prism/lidar/probe_network
rosservice call /prism/device/set_configuration \
  "confirm: true
set_camera_fps: true
camera_fps: 20
set_imu_rate_hz: true
imu_rate_hz: 800
set_mjpeg_quality: true
mjpeg_quality: 88"
rosservice call /prism/streams/control \
  "command: 'restart'
camera: true
board_imu: true
lidar: true"
```

## Prerequisites

1. Prism Agent and Host USB SDK runtime must have exactly the same version.
   This adapter pins Prism SDK distribution `1.0.2`, whose runtime version is
   `1.0.0`, and therefore requires Agent `1.0.0`.
2. Select the SDK submodule runtime prefix for the host using the table above and
   install that complete binary prefix under `/opt/prism-sdk`. For example,
   for ROS 2 Jazzy or Kilted on x86-64:

   ```bash
   PRISM_SDK_PREFIX=third_party/Prism-SDK/runtime/ros/ubuntu-24.04-x86_64
   sudo mkdir -p /opt/prism-sdk
   sudo cp -a "${PRISM_SDK_PREFIX}/." /opt/prism-sdk/
   ```

   On ARM64, use `runtime/ros/linux-arm64` for every supported Ubuntu/ROS
   release. Do not mix a library from one prefix with another prefix's headers
   or CMake files.
3. Install the SDK udev rule and reconnect the USB cable:

   ```bash
   sudo install -m 0644 \
     /opt/prism-sdk/lib/udev/rules.d/99-prism-usb.rules \
     /etc/udev/rules.d/99-prism-usb.rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   sudo usermod -aG plugdev "$USER"
   ```

   Log out and back in after changing group membership. Verify the device is
   on USB 3 before running ROS.

Only one process may own the Prism USB device. Close Prism Viewer and all SDK
CLI tools before starting the ROS adapter.

## Install ROS 1 Noetic

Install ROS Noetic and the required binary packages:

```bash
sudo apt update
sudo apt install libssl-dev libusb-1.0-0-dev pkg-config \
  ros-noetic-ros-base ros-noetic-diagnostic-msgs \
  ros-noetic-message-generation ros-noetic-message-runtime \
  ros-noetic-sensor-msgs
```

Build and install the adapter. The first argument is the SDK prefix and the
second is the adapter install prefix:

```bash
cd /work/projects/prism-ros-adapter
./scripts/install_ros1_noetic.sh /opt/prism-sdk /opt/prism-ros/noetic
source /opt/prism-ros/noetic/setup.bash
export LD_LIBRARY_PATH="/opt/prism-sdk/lib:${LD_LIBRARY_PATH:-}"
```

Start the default camera + board IMU + Mid-360 configuration:

```bash
roslaunch prism_ros_driver prism.launch
```

For Mid-360S:

```bash
roslaunch prism_ros_driver prism.launch lidar_model:=mid360s
```

## Install ROS 2

Install one supported ROS 2 distribution and its build dependencies. Replace
`jazzy` below with `foxy`, `humble`, `kilted`, `lyrical`, or `rolling`. Foxy
runs on Ubuntu 20.04 and is supported for reproducible legacy deployments, but
is end-of-life upstream:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install libssl-dev libusb-1.0-0-dev pkg-config \
  ros-jazzy-diagnostic-msgs ros-jazzy-launch-ros \
  ros-jazzy-builtin-interfaces ros-jazzy-rosidl-default-generators \
  ros-jazzy-rosidl-default-runtime ros-jazzy-sensor-msgs \
  ros-jazzy-std-msgs
```

Build and install:

```bash
cd /work/projects/prism-ros-adapter
source /opt/ros/jazzy/setup.bash
./scripts/install_ros2.sh /opt/prism-sdk /opt/prism-ros/jazzy
source /opt/prism-ros/jazzy/setup.bash
export AMENT_PREFIX_PATH="/opt/prism-ros/jazzy:${AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="/opt/prism-sdk/lib:${LD_LIBRARY_PATH:-}"
```

Start the adapter:

```bash
ros2 launch prism_ros_driver prism.launch.py
```

For Mid-360S:

```bash
ros2 launch prism_ros_driver prism.launch.py lidar_model:=mid360s
```

## Configuration

Defaults are in:

- ROS 1: `ros1_ws/src/prism_ros_driver/config/default.yaml`
- ROS 2: `ros2_ws/src/prism_ros_driver/config/default.yaml`

Important parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `device_serial` | empty | Empty selects the first Prism; otherwise uses the USB serial |
| `camera_enabled` | `true` | Publish all four compressed camera streams |
| `camera_fps` | `0` | `0` uses device configuration; otherwise request 1–30 FPS |
| `board_imu_enabled` | `true` | Publish detected board IMUs |
| `imu_sensor_count` | `0` | `0` uses detected count; normally one or two |
| `imu_rate_hz` | `0` | `0` uses device configuration; production devices use 800 Hz |
| `lidar_enabled` | `true` | Start and publish the configured Livox LiDAR |
| `lidar_model` | `mid360` | Must be `mid360` or `mid360s` |
| `require_synchronized_timestamps` | `true` | Drop samples that are not in the common RK time domain |
| `topic_prefix` | `/prism` | Prefix for all data topics |

`lidar_model` selects the SDK stream implementation when the adapter starts; it
is not a persisted device setting and cannot be changed while the adapter is
running. Pass `lidar_model:=mid360s` at launch for a Mid-360S, or use the
default `mid360` for a Mid-360.

LiDAR network settings can be managed through the ROS Adapter. The
`/prism/lidar/set_network` service persists `enabled`, the RK `end0` address,
netmask, and LiDAR address; `confirm` must be `true`. The adapter temporarily
pauses active streams for the configuration operation and then restores them.
Use `/prism/lidar/get_network` to read the saved and live interface state and
`/prism/lidar/probe_network` to apply/check the host-side configuration and
test target reachability. The adapter does not infer the model or IP address,
so both must match the connected LiDAR.

## Docker build

Docker automatically selects the matching binary SDK prefix from the pinned
`third_party/Prism-SDK` submodule and copies it into the ROS image. It never
needs the Prism Agent repository and never compiles the USB SDK. To test an
alternative binary-only SDK prefix, set `PRISM_USB_SDK_PREFIX`; the directory
must contain the public headers, matching shared or static library, CMake
package files and udev rule:

```bash
PRISM_USB_SDK_PREFIX=/path/to/prism-sdk-prefix \
  ./scripts/docker_build.sh jazzy
```

On an ARM64 host no extra option is needed. To cross-test the ARM64 image on an
x86-64 Docker host with binfmt/QEMU enabled, set:

```bash
PRISM_ROS_ARCH=arm64 ./scripts/docker_build.sh jazzy
PRISM_ROS_ARCH=arm64 ./scripts/docker_smoke_test.sh jazzy
```

The build script first pulls ROS base images through `docker.1ms.run` and then
tags them with the standard `ros:*` name. It falls back to DaoCloud if the
primary pull does not finish within 15 minutes. Override the mirror prefixes
with `PRISM_DOCKER_MIRROR` and `PRISM_DOCKER_MIRROR_FALLBACK` when needed.
For Rolling, pin a tested image digest in production because both its API and
base image move over time.

Build one image:

```bash
cd /work/projects/prism-ros-adapter
./scripts/docker_build.sh noetic
./scripts/docker_build.sh jazzy
```

Run the ROS-independent clock-selection regression test:

```bash
cmake -S . -B build-common -DBUILD_TESTING=ON
cmake --build build-common
ctest --test-dir build-common --output-on-failure
```

Build the complete matrix:

```bash
./scripts/docker_build.sh all
```

Run with USB access:

```bash
./scripts/docker_run_ros1.sh
./scripts/docker_run_ros2.sh jazzy
```

Pass another command after the image/distro to inspect topics:

```bash
./scripts/docker_run_ros2.sh jazzy ros2 topic list
```

The run scripts use host networking and privileged USB access so they are
intended for development and hardware verification. The ROS 2 script also uses
the host IPC namespace; Fast DDS shared-memory transport otherwise discovers
publishers across the container boundary but cannot exchange large image
samples. Any ROS 2 subscriber in another container must use the same
`--network host --ipc host` settings. Production deployments should replace
`--privileged` with a narrower device/cgroup policy appropriate for the host.

## Quick verification

ROS 1:

```bash
rostopic hz /prism/camera0/image/compressed
rostopic hz /prism/imu0/data
rostopic hz /prism/lidar/points
rostopic hz /prism/lidar/imu
```

ROS 2:

```bash
ros2 topic hz /prism/camera0/image/compressed
ros2 topic hz /prism/imu0/data
ros2 topic hz /prism/lidar/points
ros2 topic hz /prism/lidar/imu
```

Inspect metadata and diagnostics:

```bash
ros2 topic echo --once /prism/camera/metadata
ros2 topic echo --once /diagnostics
ros2 service list | grep '^/prism/'
```

A healthy run has monotonically increasing device timestamps, nonzero camera,
IMU and LiDAR counters, no continuously growing dispatch-drop counter, and no
USB permission/version error. If the device opens in Viewer but not in Docker,
confirm no other process owns it and that `/dev/bus/usb` was passed into the
container.

For `/prism/lidar/points`, a healthy Mid-360/Mid-360S run reports approximately
10 Hz, normally contains about 20,000 points per message, and exposes
`offset_time` as `UINT32` at byte offset 16 with values in `[0, 100000000)` ns.
With a running ROS 2 node, validate all of these invariants over 30 messages:

```bash
python3 scripts/verify_lidar_pointcloud_ros2.py
```

## Update notes

- [Version 1.0.2](docs/update/v1.0.2.md)
- [版本 1.0.2（中文）](docs/update/v1.0.2.zh-CN.md)

## Release-tag CI

Pushing a tag matching `v*` runs `.github/workflows/release-tag-test.yml`.
The workflow checks that the tag version matches all four ROS package
manifests, runs the ROS-independent unit tests, then builds and smoke-tests
ROS 1 Noetic and every supported ROS 2 distribution on both x86-64 and native
ARM64 runners. It verifies the pinned SDK submodule checksums and binary-only
runtime-prefix boundary before building. The workflow uses only the public
headers and platform-specific binary libraries in the SDK repository; it never
checks out Agent source.
After every matrix job succeeds, the tag workflow publishes a recursive source
archive containing the exact pinned SDK submodule as a GitHub Release.

GitHub-hosted runners have no Prism USB device or Mid-360. They verify source
compatibility, package installation, message interfaces, launch descriptions
and dynamic-library resolution. Camera, IMU and LiDAR streaming tests remain a
separate hardware qualification step on a controlled machine with the device
attached.
