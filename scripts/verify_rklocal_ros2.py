#!/usr/bin/env python3
"""Run inside either RK-local Docker image; never opens a real device socket.

Exercises the published ARM64 SDK + ROS node over a temporary Unix socket:
camera identity/metadata, one IMU, GNSS/RTK polling, stream restart, exactly
one video ACK, and rejection of same-host clock synchronization.
Synthetic JPEG markers test transport, not image decoding or sensor accuracy.
"""
import collections
import os
import pathlib
import signal
import socket
import struct
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu
from prism_ros_msgs.msg import CameraFrameMetadata, GnssTimingStatus, TimeSyncRtkStatus
from prism_ros_msgs.srv import (
    ControlStreams, SyncSystemTime, ControlRtk, SetCorsConfiguration,
    SetTimeSyncPort, GetReceiverPosition,
)


def put(data, offset, value, fmt="I"):
    struct.pack_into("<" + fmt, data, offset, value)


def status(size, version=1):
    data = bytearray(size)
    struct.pack_into("<HH", data, 0, version, size)
    return data


def sentence(body, extended=False):
    crc = 0
    for byte in body.encode("ascii"):
        crc ^= byte
        if extended:
            for _ in range(8):
                crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return (("#" if extended else "$") + body +
            (f"*{crc:08X}" if extended else f"*{crc:02X}")).encode("ascii")


def receive(conn, size):
    data = bytearray()
    while len(data) < size:
        part = conn.recv(size - len(data))
        if not part:
            raise EOFError()
        data.extend(part)
    return data


class MockAgent:
    def __init__(self, path):
        self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.listener.bind(path)
        self.listener.listen(1)
        self.acks = collections.Counter()
        self.commands = collections.Counter()
        self.frame_id = 0
        self.error = None
        self.streaming = False
        self.empty_observations = False
        self.observation_sequence = 0
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, kind, sequence, payload):
        self.conn.sendall(struct.pack("<6sBBHII", b"DIBULI", 1, kind, 0,
                                      sequence, len(payload)) + payload)

    def dispatch(self, kind, seq, data):
        self.commands[kind] += 1
        if kind == 1:
            body = bytearray(108)
            struct.pack_into("<HHI", body, 0, 1, 18, 8 * 1024 * 1024)
            body[8:19] = b"prism-agent"
            body[40:45] = b"1.2.0"
            body[92:98] = b"0.4.27"
            self.send(0x81, seq, body)
        elif kind == 4:
            body = status(256, 4)
            put(body, 4, 6)  # online, synchronized; no USB host connected
            body[9:16] = bytes([1, 4, 1, 1, 1, 0, 15])
            struct.pack_into("<HH", body, 20, 800, 30)
            body[32:36] = b"MOCK"
            self.send(0x85, seq, body)
        elif kind in (6, 7):
            self.streaming = kind == 6
            body = bytearray(20)
            struct.pack_into("<BBHIII", body, 0, int(self.streaming), 4, 30,
                             1280, 1024, 44)
            self.send(0x88, seq, body)
        elif kind in (8, 9):
            self.streaming = kind == 8
            body = bytearray(12)
            struct.pack_into("<BBHI", body, 0, int(self.streaming), 1, 800, 44)
            self.send(0x8A, seq, body)
        elif kind == 0x3A:
            body = status(104, 5)
            put(body, 28, 20000)
            self.send(0xB6, seq, body)
        elif kind == 0x44:
            body = status(32)
            now_ms = int(time.monotonic() * 1000)
            put(body, 16, now_ms, "Q")
            put(body, 24, 2 if self.empty_observations else 1, "Q")
            if not self.empty_observations:
                fixtures = [
                    sentence("GNGGA,120000.1,3100.0000,N,12100.0000,E,5,12,0.8,20,M,10,M,,"),
                    sentence("GNGST,120000.1,0.03,0.04,0.02,0,0.03,0.02,0.05"),
                    sentence("ADRNAVA,COM1,GPS,FINE,2437,388818100,0,0,18,0;SOL_COMPUTED,NARROW_INT,31,121,20,10,WGS84,0.03,0.02,0.05,0,0,0,20,18,0,0,0,0,0,0,0,0,0,0,0,0,0,0", True),
                ]
                for value in fixtures:
                    self.observation_sequence += 1
                    body.extend(struct.pack("<QQH6x", self.observation_sequence, now_ms, len(value)))
                    body.extend(value)
                put(body, 8, self.observation_sequence, "Q")
            self.send(0xBB, seq, body)
        elif kind == 0x3D:
            body = status(56)
            put(body, 4, 3)
            put(body, 8, 0xFFFFFFFF)
            put(body, 12, 0xFFFFFFFF)
            self.send(0xB8, seq, body)
        elif kind == 0x40:
            body = status(100)
            put(body, 4, 7)
            body[32] = 4
            self.send(0xB9, seq, body)
        elif kind == 0x0A:
            self.acks[struct.unpack_from("<I", data, 4)[0]] += 1
        elif kind != 0x0C:
            raise AssertionError(f"unexpected command {kind:#x}")

    def samples(self):
        self.frame_id += 1
        timestamp = 1788541200000000 + self.frame_id * 100000
        imu = bytearray(44)
        put(imu, 2, 128, "H")
        put(imu, 4, self.frame_id)
        put(imu, 8, timestamp, "Q")
        put(imu, 24, 1000)
        self.send(0x89, 900, imu)
        meta = bytearray(84)
        meta[0:2] = bytes([1, 4])
        put(meta, 4, self.frame_id)
        put(meta, 8, self.frame_id)
        put(meta, 24, timestamp * 1000, "Q")
        for camera in range(4):
            put(meta, 32 + 4 * camera, 1000 + camera)
        self.send(0x8C, 901, meta)
        for camera in range(4):
            chunk = bytearray(39)
            struct.pack_into("<BBHIIIIIIQ", chunk, 0, camera, 2, 3, 1280,
                             1024, self.frame_id, 3, 0, 3, timestamp)
            chunk[36:] = bytes([255, camera, 217])
            self.send(0x8B, 902 + camera, chunk)

    def run(self):
        try:
            import select
            self.conn, _ = self.listener.accept()
            with self.conn:
                next_sample = time.monotonic()
                while True:
                    readable, _, _ = select.select([self.conn], [], [], 0.01)
                    if readable:
                        header = receive(self.conn, 18)
                        magic, version, kind, _, seq, size = struct.unpack("<6sBBHII", header)
                        assert magic == b"DIBULI" and version == 1 and size < 1048576
                        self.dispatch(kind, seq, receive(self.conn, size))
                    if self.streaming and time.monotonic() >= next_sample:
                        self.samples()
                        next_sample = time.monotonic() + 0.1
        except (EOFError, BrokenPipeError, ConnectionResetError):
            pass
        except BaseException as error:
            self.error = error
        finally:
            self.listener.close()


def main():
    rclpy.init()
    node = rclpy.create_node("prism_rklocal_mock_test")
    counts = collections.Counter()
    subscriptions = []

    def camera_received(camera, message):
        assert bytes(message.data) == bytes([255, camera, 217]), "camera identity swapped"
        counts[f"camera{camera}"] += 1

    def metadata_received(message):
        assert list(message.exposure_us) == [1000, 1001, 1002, 1003]
        assert message.host_frame_id == message.carrier_frame_id
        counts["metadata"] += 1

    subscriptions.append(node.create_subscription(
        CameraFrameMetadata, "/prism/camera/metadata", metadata_received,
        qos_profile_sensor_data))
    for camera in range(4):
        subscriptions.append(node.create_subscription(
            CompressedImage, f"/prism/camera{camera}/image/compressed",
            lambda msg, camera=camera: camera_received(camera, msg), 10))
    for kind, topic, name, qos in (
        (Imu, "/prism/imu0/data", "imu0", qos_profile_sensor_data),
        (GnssTimingStatus, "/prism/gnss/timing", "gnss", 10),
        (TimeSyncRtkStatus, "/prism/rtk/status", "rtk", 10),
    ):
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, key=name: counts.update([key]), qos))

    def spin_until(check, timeout=45):
        deadline = time.monotonic() + timeout
        while not check() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert check(), f"timeout; counts={dict(counts)}"

    def call(kind, name, request):
        client = node.create_client(kind, name)
        assert client.wait_for_service(timeout_sec=20), name
        future = client.call_async(request)
        spin_until(future.done)
        result = future.result()
        node.destroy_client(client)
        return result

    with tempfile.TemporaryDirectory(prefix="prism-ros-mock-") as directory:
        path = str(pathlib.Path(directory) / "stream.sock")
        mock = MockAgent(path)
        command = ["/opt/prism-ros2/lib/prism_ros_driver/prism_ros_driver_node",
                   "--ros-args", "-p", f"rklocal_socket:={path}"]
        # A file avoids blocking the driver on a full subprocess pipe.
        with tempfile.TemporaryFile(mode="w+") as output:
            process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT)
            try:
                keys = [f"camera{i}" for i in range(4)] + ["imu0", "metadata", "gnss", "rtk"]
                spin_until(lambda: all(counts[key] >= 3 for key in keys))
                request = SyncSystemTime.Request()
                request.confirm = True
                result = call(SyncSystemTime, "/prism/system/sync_time", request)
                assert not result.success and "RK-local" in result.message, result.message
                assert mock.commands[0x0D] == 0 and mock.commands[0x0E] == 0
                # Unconfirmed mutating requests must not reach the SDK/Agent.
                for service, name in (
                    (ControlRtk, "/prism/rtk/control"),
                    (SetCorsConfiguration, "/prism/rtk/set_cors"),
                    (SetTimeSyncPort, "/prism/system/set_timesync_port"),
                ):
                    result = call(service, name, service.Request())
                    assert not result.success, name
                for rtk in (False, True):
                    request = GetReceiverPosition.Request()
                    request.rtk = rtk
                    result = call(GetReceiverPosition, "/prism/rtk/get_receiver_position", request)
                    assert result.success, result.message
                    assert result.status.valid and result.status.quality == (4 if rtk else 5)
                    assert result.status.covariance_valid
                    if rtk:
                        assert not result.status.timestamp_valid, "GPST must not become UTC without offset"
                mock.empty_observations = True
                for rtk in (False, True):
                    request = GetReceiverPosition.Request()
                    request.rtk = rtk
                    result = call(GetReceiverPosition, "/prism/rtk/get_receiver_position", request)
                    assert result.success, result.message
                    assert not result.status.valid, "empty receiver data must not invent a fix"
                for action in ("stop", "start", "restart", "stop"):
                    request = ControlStreams.Request()
                    request.command, request.camera, request.board_imu = action, True, True
                    result = call(ControlStreams, "/prism/streams/control", request)
                    assert result.success, result.message
                    if action != "stop":
                        previous = counts["camera0"]
                        spin_until(lambda: counts["camera0"] >= previous + 3)
                assert mock.acks and all(count == 1 for count in mock.acks.values()), mock.acks
                assert mock.error is None, mock.error
                print("PASS RK-local ROS2 mock:", dict(counts),
                      "video ACKs:", len(mock.acks), "no duplicate ACK or host time-set")
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                mock.thread.join(timeout=3)
                print("Mock commands:", dict(mock.commands), "mock error:", repr(mock.error))
                output.seek(0)
                print(output.read()[-6000:])
                node.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    main()
