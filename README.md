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
supported. Each ROS distribution uses the Prism USB SDK binary built for its
base operating system, which avoids mixing incompatible glibc/OpenSSL
binaries.

## Prism SDK Git submodule

This repository pins Prism SDK `1.0.0` as the
`third_party/Prism-SDK` Git submodule. Clone recursively:

```bash
git clone --recurse-submodules git@github.com:DIBULI/Prism-ROS-adapter.git
```

For an existing checkout, initialize the exact pinned SDK commit with:

```bash
git submodule update --init --recursive
```

The SDK repository contains only:

- public SDK headers;
- the platform-specific `libprism_usb_sdk.so` dynamic library;
- exported CMake package configuration files; and
- the Prism USB udev rule.

The Prism Agent and USB SDK implementation source are not included. Neither
the local Docker build nor release CI checks out or compiles Agent source.
The supported binary mapping is:

| ROS distribution | SDK submodule runtime prefix |
| --- | --- |
| Noetic | `runtime/ros/ubuntu-20.04-x86_64` |
| Humble | `runtime/ros/ubuntu-22.04-x86_64` |
| Jazzy, Kilted | `runtime/ros/ubuntu-24.04-x86_64` |
| Lyrical, Rolling | `runtime/ros/ubuntu-26.04-x86_64` |

All four payloads are part of the pinned Prism SDK `1.0.0` release and include
the production `800 Hz` IMU runtime update. Verify the submodule and all ROS
runtime prefixes with:

```bash
./scripts/verify_sdk_submodule.sh all
```

## Published topics

| Topic | Type | Description |
| --- | --- | --- |
| `/prism/camera0/image/compressed` ... `/prism/camera3/image/compressed` | `sensor_msgs/CompressedImage` | Four original MJPEG images; no decode/re-encode |
| `/prism/camera/metadata` | `prism_ros_msgs/CameraFrameMetadata` | Shared trigger timestamp, frame IDs, exposure and gains for all cameras |
| `/prism/imu0/data`, `/prism/imu1/data` | `sensor_msgs/Imu` | Board IMUs in m/s² and rad/s; only detected IMUs publish data |
| `/prism/lidar/points` | `sensor_msgs/PointCloud2` | Mid-360/Mid-360S Cartesian points in metres, with `intensity` and `tag` fields |
| `/prism/lidar/imu` | `sensor_msgs/Imu` | LiDAR-integrated IMU in m/s² and rad/s |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | USB, sensor-board, stream counters and drop counters |

ROS 2 publishes the large compressed-image topics with reliable, volatile QoS
and depth 2. Camera metadata, both IMU families and point clouds use the normal
best-effort sensor-data profile; diagnostics are reliable. A custom ROS 2
camera subscriber should therefore request reliable QoS. This preserves every
complete MJPEG sample without applying back-pressure to the high-rate IMU and
LiDAR topics.

The camera stamp is the sensor-board trigger edge. LiDAR point stamps are the
batch base time. The adapter does not interpolate per-point time and does not
deskew the cloud. LiDAR transports expose both a raw PTP timestamp and an SDK
normalized timestamp; the adapter automatically selects the representation
that matches the camera/board-IMU device clock. This comparison uses monotonic
arrival intervals and never uses the host wall clock. By default, samples that
cannot be placed in the common device time domain are dropped so camera, board
IMU, LiDAR points and LiDAR IMU do not silently mix clock domains.

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
```

Use the corresponding ROS 1 service names without `/srv/`, for example:

```bash
rosservice call /prism/camera/get_exposure
rosservice call /prism/camera/set_target_brightness "target_brightness: 35"
rosservice call /prism/system/sync_time "confirm: true"
```

## Prerequisites

1. Prism Agent and Host USB SDK must have exactly the same version. This
   adapter currently requires Prism USB SDK `1.0.0`.
2. Select the SDK submodule runtime prefix for the host using the table above and
   install that complete binary prefix under `/opt/prism-sdk`. For example,
   for ROS 2 Jazzy or Kilted:

   ```bash
   PRISM_SDK_PREFIX=third_party/Prism-SDK/runtime/ros/ubuntu-24.04-x86_64
   sudo mkdir -p /opt/prism-sdk
   sudo cp -a "${PRISM_SDK_PREFIX}/." /opt/prism-sdk/
   ```

   Do not mix a dynamic library from one Ubuntu release with another
   release's headers or CMake files.
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
sudo apt install ros-noetic-ros-base ros-noetic-diagnostic-msgs \
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
`jazzy` below with `humble`, `kilted`, `lyrical`, or `rolling`:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install ros-jazzy-diagnostic-msgs ros-jazzy-launch-ros \
  ros-jazzy-rosidl-default-generators ros-jazzy-sensor-msgs \
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

The LiDAR network address is configured by Prism Viewer/USB SDK before the
adapter starts. The ROS adapter does not alter `end0` or LiDAR IP settings.

## Docker build

Docker automatically selects the matching binary SDK prefix from the pinned
`third_party/Prism-SDK` submodule and copies it into the ROS image. It never
needs the Prism Agent repository and never compiles the USB SDK. To test an
alternative binary-only SDK prefix, set `PRISM_USB_SDK_PREFIX`; the directory
must contain the public headers, dynamic library, CMake package files and udev
rule:

```bash
PRISM_USB_SDK_PREFIX=/path/to/prism-sdk-prefix \
  ./scripts/docker_build.sh jazzy
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

## Release-tag CI

Pushing a tag matching `v*` runs `.github/workflows/release-tag-test.yml`.
The workflow checks that the tag version matches all four ROS package
manifests, runs the ROS-independent unit tests, then builds and smoke-tests
ROS 1 Noetic and every supported ROS 2 distribution. It verifies the pinned
SDK submodule checksums and binary-only runtime-prefix boundary before
building. The workflow uses only the public headers and platform-specific
dynamic libraries in the SDK repository; it never checks out Agent source.
After every matrix job succeeds, the tag workflow publishes a recursive source
archive containing the exact pinned SDK submodule as a GitHub Release.

GitHub-hosted runners have no Prism USB device or Mid-360. They verify source
compatibility, package installation, message interfaces, launch descriptions
and dynamic-library resolution. Camera, IMU and LiDAR streaming tests remain a
separate hardware qualification step on a controlled machine with the device
attached.
