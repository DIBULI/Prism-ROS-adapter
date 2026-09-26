# ROS 2 on RK: Ubuntu 22.04 / 24.04 Docker

Run the ROS node **on the RK3576**, connected to the existing Agent through
`/run/prism/stream.sock`. The container does not start another Agent, access
USB, flash firmware, or synchronize the device clock automatically.

| Container OS | ROS 2 | Architecture | Image built locally |
| --- | --- | --- | --- |
| Ubuntu 22.04 | Humble | Linux ARM64 | `prism-ros-adapter:humble-rklocal` |
| Ubuntu 24.04 | Jazzy | Linux ARM64 | `prism-ros-adapter:jazzy-rklocal` |

The container OS does not replace the RK host OS/kernel. Both images can run on
the same RK host if its kernel and Docker support ARM64 Linux containers.
These tags are local build outputs, **not images published to Docker Hub**.

## Prerequisites

- RK runs Agent **1.2.0**, with its RK-local socket enabled.
  SDK is pinned to `6ed74f8` / 1.2.0.
- Docker Engine with BuildKit/named build-context support. Native ARM64 builds
  need no emulator; an x86-64 build server needs registered AArch64 binfmt/QEMU.
- On RK, `test -S /run/prism/stream.sock` succeeds. Container access to this
  socket grants device-control access; run only trusted images.
- Close Viewer/USB capture and other RK-local capture clients before testing.
  Use one capture owner; this release does not qualify concurrent capture clients.

### RK image without Docker bridge/NAT support

The minimal Debian RK image may lack the kernel MASQUERADE support needed by
Docker's default bridge. If `journalctl -u docker` reports a MASQUERADE/NAT
failure, use the supplied **host-network-only** configuration for a dedicated
ROS Docker host:

```bash
# Install Docker using the host distribution's signed packages first.
# Refuse to overwrite an existing daemon configuration.
test ! -e /etc/docker/daemon.json && \
  sudo install -m 0644 docker/rk-host-network-daemon.json /etc/docker/daemon.json
sudo systemctl reset-failed docker.service docker.socket
sudo systemctl restart docker
sudo docker info
```

If a daemon configuration already exists, review/merge it manually. This
configuration disables Docker's default bridge, NAT and forwarding management;
use `--network host` (as the run script does), or `--network none`. Bridge
networking and `-p` port publishing are not supported in this mode. Host-network
containers share the RK network namespace: run trusted containers only. The
configuration does not stop or reconfigure the device's PTP services and does
not grant containers permission to set the host clock.

Without GNSS, the RK clock may intentionally be near the Unix epoch. Do not
set an arbitrary host time just to download container packages. Build on the
server and transfer with `docker save` / `docker load` as shown below; offline
installation also avoids HTTPS certificate-date failures on the RK.

## Build (on RK or an ARM64-capable build server)

From the existing `Prism-ROS-adapter` checkout:

```bash
git submodule update --init --recursive
bash scripts/docker_build_rklocal.sh 22.04
bash scripts/docker_build_rklocal.sh 24.04
```

Both scripts use `--platform linux/arm64`, the published RK-local static
archive, and no Agent/SDK implementation source. Ubuntu and ROS APT packages
use the **Tsinghua mirrors**, with signature verification retained.

If the Docker registry is unavailable, pre-pull a trusted ARM64 ROS base image
and pass its tag, e.g. `PRISM_ROS_BASE=prism-ros-base:humble-arm64`. It must match
the selected ROS distribution. `PRISM_RKLOCAL_SDK_ROOT` can override the full
SDK distribution root (not the Host SDK `runtime/ros` prefix).

To move a built image from the server to RK:

```bash
docker save prism-ros-adapter:humble-rklocal | gzip > prism-ros2-humble-rklocal.tar.gz
# Copy this archive to RK, then on RK:
gzip -dc prism-ros2-humble-rklocal.tar.gz | docker load
```

Repeat with `jazzy` for Ubuntu 24.04. No ROS installation is needed on the host.

## Run on RK

Use `sudo` for the run commands if your user cannot access Docker's socket.
Optionally install the launcher once on RK:

```bash
sudo install -m 0755 scripts/docker_run_rklocal.sh /usr/local/bin/prism-ros2-docker
sudo prism-ros2-docker 22.04
# Ubuntu 24.04 / Jazzy, after stopping the first container:
sudo prism-ros2-docker 24.04
```

Or run directly from the repository:

```bash
# Run only one capture container at a time.
bash scripts/docker_run_rklocal.sh 22.04
# Or stop the first with Ctrl-C, then:
bash scripts/docker_run_rklocal.sh 24.04
```

Defaults match the USB node: camera + detected board IMU + navigation enabled;
LiDAR disabled until explicitly enabled. Device settings determine camera and
IMU rate (`camera_fps=0`, `imu_rate_hz=0`). One detected IMU is sufficient.
Camera JPEGs are forwarded without decode/re-encode. Times remain sensor-board
timestamps; this backend does not fix missing hardware PPS/GNSS.

The run script mounts the **directory** `/run/prism` read-only, uses host
network/IPC for DDS, drops Linux capabilities, and does **not** use
`--privileged` or mount `/dev/bus/usb`. Read-only bind does not prevent Unix
socket communication. The default container user is root; for non-root manual
runs, match the socket's owner/group and permissions. Agent restart can replace
the socket without remounting, but an existing ROS session must be restarted.

Custom host socket directory: `PRISM_SOCKET_DIR=/path/to/prism`; it must contain
`stream.sock`, mapped to `/run/prism/stream.sock` inside the container.

To query GNSS/RTK only, without starting capture:

```bash
bash scripts/docker_run_rklocal.sh 22.04 \
  ros2 run prism_ros_driver prism_ros_driver_node --ros-args \
  -p camera_enabled:=false -p board_imu_enabled:=false
```

To enable the existing LiDAR path as part of capture:

```bash
bash scripts/docker_run_rklocal.sh 22.04 \
  ros2 run prism_ros_driver prism_ros_driver_node --ros-args \
  -p lidar_enabled:=true -p lidar_model:=mid360s
```

LiDAR networking must already be configured on the device; enabling the stream
does not silently overwrite its IP or persistent settings.

`ROS_DOMAIN_ID=42 bash scripts/docker_run_rklocal.sh 22.04` selects a DDS domain.
Use the same domain on subscriber hosts. For a ROS CLI shell on RK (does not
open another SDK capture session):

```bash
docker run --rm -it --network host --ipc host \
  prism-ros-adapter:humble-rklocal bash
ros2 topic list
ros2 topic hz /prism/imu0/data
ros2 service call /prism/gnss/get_timing prism_ros_msgs/srv/GetGnssTiming '{}'
```

## Topics, services, and differences from USB

Topic names, message types, units, QoS and service schemas are shared with the
[main README](../README.md#published-topics) and [1.2.0 navigation API](navigation.md).
Camera, IMU, LiDAR, GNSS, RTK and rover RTCM use the RK-local client. Configuration,
exposure, network, Wi-Fi, time-port, CORS configuration and RTK start/stop calls also use it;
the Agent remains authoritative for whether a command is currently permitted.
The old Host raw-RTCM correction input is removed. Firmware upgrade services
are not introduced.

- `device_serial` is **USB-only**. A nonempty value is rejected in an RK-local
  build. `rklocal_socket` selects an absolute Unix socket path, not a TCP endpoint.
- `/prism/system/sync_time` with `confirm=true` returns **failure on RK-local**:
  the container shares the RK clock and has no independent host UTC reference.
  It does not pause capture or send a clock-setting command. Use GNSS or an
  external USB host according to existing Agent time-source rules.
- USB speed reported by DeviceInfo describes the device USB link, not the local
  Unix socket throughput; it is not an RK-local connection prerequisite.
- Camera and board IMU share one aggregate session. Stopping either stops both.
- RK-local SDK automatically ACKs assembled video; ROS must not send a second
  ACK. Its bounded raw event queue stops acquisition on overflow with an error,
  instead of silently accepting missing data.
- ROS 1 remains USB-only. This change does not enable TCP transport or install
  ROS/Docker into the flash image.

## Native build without Docker

On an ARM64 Ubuntu host with Humble/Jazzy installed:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --merge-install --build-base build-rklocal \
  --install-base install-rklocal --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPRISM_TRANSPORT=rklocal
source install-rklocal/setup.bash
ros2 launch prism_ros_driver prism.launch.py
```

Do not reuse the USB build directory for this binary. CMake links only
`Prism::RkLocal`; linking Host and RK-local libraries together is not supported.

## Reproducible tests (no physical device)

From the repository on the Docker host, run each image with a temporary mock
Agent **inside** the container. No host socket or USB device is mounted:

```bash
for distro in humble jazzy; do
  docker run --rm --platform linux/arm64 \
    --mount "type=bind,src=$PWD/scripts,dst=/tests,readonly" \
    prism-ros-adapter:${distro}-rklocal \
    python3 /tests/verify_rklocal_ros2.py
done
```

The mock tests actual SDK handshake/stream parsing, four camera IDs, metadata,
one IMU, GNSS/RTK topics, stop/start/restart, no duplicate video ACKs, and no
same-host time-setting. They do not establish physical capture rates, GNSS
accuracy, CORS availability, or LiDAR hardware correctness.

Run runtime tests on native ARM64 as well: successful QEMU compilation does not
establish DDS discovery or delivery behavior on the RK. For capture-rate checks,
`common/tests/ros2_probe` provides a passive C++ subscriber (no SDK connection,
clock changes, persistent configuration or image recording). Build it against
the matching ROS distribution with CMake, then run the binary alongside one
driver. It waits three seconds for discovery before counting. It reports each
camera/IMU rate, IMU timestamp gaps over 2 ms (for an 800 Hz input), and driver
diagnostics. Timestamp gaps can also reflect clock adjustments; they are not by
themselves proof of missing samples. Best-effort IMU QoS does not guarantee
loss-free recording. Python CLI/subscriber rates under four-camera load can be
lower than the actual driver input rate, so compare with the driver counters
and the C++ probe before concluding that the device is under-sampling.
