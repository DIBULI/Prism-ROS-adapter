#!/usr/bin/env python3

import argparse
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


EXPECTED_FIELDS = {
    "x": (0, PointField.FLOAT32),
    "y": (4, PointField.FLOAT32),
    "z": (8, PointField.FLOAT32),
    "intensity": (12, PointField.UINT8),
    "tag": (13, PointField.UINT8),
    "offset_time": (16, PointField.UINT32),
}
FRAME_PERIOD_NS = 100_000_000


class PointCloudVerifier(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("prism_lidar_pointcloud_verifier")
        self.messages = []
        self.error = None
        self.create_subscription(
            PointCloud2, topic, self._handle, qos_profile_sensor_data
        )

    def _handle(self, message: PointCloud2) -> None:
        if self.error is not None:
            return
        try:
            fields = {
                field.name: (field.offset, field.datatype, field.count)
                for field in message.fields
            }
            if set(fields) != set(EXPECTED_FIELDS):
                raise RuntimeError(f"unexpected fields: {fields}")
            for name, (offset, datatype) in EXPECTED_FIELDS.items():
                if fields[name] != (offset, datatype, 1):
                    raise RuntimeError(
                        f"invalid {name} field: {fields[name]}"
                    )
            if message.height != 1 or message.width == 0:
                raise RuntimeError(
                    f"invalid cloud dimensions {message.width}x{message.height}"
                )
            if message.point_step != 20:
                raise RuntimeError(f"invalid point_step {message.point_step}")
            if message.row_step != message.point_step * message.width:
                raise RuntimeError("row_step does not match width * point_step")
            if len(message.data) != message.row_step:
                raise RuntimeError("data length does not match row_step")

            byte_order = ">" if message.is_bigendian else "<"
            unpack_offset = struct.Struct(byte_order + "I").unpack_from
            offsets = [
                unpack_offset(message.data, index * message.point_step + 16)[0]
                for index in range(message.width)
            ]
            if offsets[0] != 0:
                raise RuntimeError(f"first offset_time is {offsets[0]} ns")
            if offsets[-1] >= FRAME_PERIOD_NS:
                raise RuntimeError(
                    f"last offset_time is outside the 100 ms frame: {offsets[-1]}"
                )
            if any(current <= previous for previous, current in zip(offsets, offsets[1:])):
                raise RuntimeError("offset_time is not strictly increasing")

            timestamp_ns = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )
            if self.messages:
                previous_timestamp = self.messages[-1][0]
                timestamp_step_ns = timestamp_ns - previous_timestamp
                if abs(timestamp_step_ns - FRAME_PERIOD_NS) > 1_000_000:
                    raise RuntimeError(
                        "header timestamp step differs from 100 ms by more "
                        f"than 1 ms: {timestamp_step_ns} ns"
                    )
            self.messages.append(
                (timestamp_ns, time.monotonic_ns(), message.width, offsets[-1])
            )
        except Exception as error:  # noqa: BLE001 - report callback failures
            self.error = error


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify Prism's 10 Hz PointCloud2 and offset_time fields."
    )
    parser.add_argument("--topic", default="/prism/lidar/points")
    parser.add_argument("--messages", type=int, default=30)
    parser.add_argument("--timeout", type=float, default=15.0)
    arguments = parser.parse_args()
    if arguments.messages < 2 or arguments.timeout <= 0:
        parser.error("--messages must be >= 2 and --timeout must be > 0")

    rclpy.init()
    node = PointCloudVerifier(arguments.topic)
    deadline = time.monotonic() + arguments.timeout
    try:
        while (
            node.error is None
            and len(node.messages) < arguments.messages
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.error is not None:
        print(f"FAIL: {node.error}", file=sys.stderr)
        return 1
    if len(node.messages) < arguments.messages:
        print(
            f"FAIL: received {len(node.messages)}/{arguments.messages} messages",
            file=sys.stderr,
        )
        return 1

    arrival_duration_ns = node.messages[-1][1] - node.messages[0][1]
    arrival_rate_hz = (
        (len(node.messages) - 1) * 1_000_000_000 / arrival_duration_ns
    )
    widths = [message[2] for message in node.messages]
    last_offsets = [message[3] for message in node.messages]
    print(
        "PASS "
        f"messages={len(node.messages)} "
        f"arrival_rate_hz={arrival_rate_hz:.3f} "
        "stamp_rate_hz=10.000_nominal "
        f"width_min={min(widths)} width_max={max(widths)} "
        f"last_offset_ns_min={min(last_offsets)} "
        f"last_offset_ns_max={max(last_offsets)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
