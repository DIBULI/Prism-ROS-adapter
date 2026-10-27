#!/usr/bin/env python3
"""Published SDK + real ROS2 node against a loopback-only NET/4 fixture.

Synthetic JPEG bytes validate transport and ACKs, not image decoding.
"""
import argparse
import collections
import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Imu
from diagnostic_msgs.msg import DiagnosticArray
from prism_ros_msgs.msg import CameraFrameMetadata
from prism_ros_msgs.srv import ControlStreams
from verify_rklocal_ros2 import MockAgent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('binary')
    cases = ('missing-token', 'bad-token', 'serial-mismatch', 'stream')
    parser.add_argument('--case', choices=cases)
    args = parser.parse_args()
    assert pathlib.Path(args.binary).is_file()
    if args.case is None:
        # Negative nodes expose the same service/topic names as the positive
        # node. Do not reuse their discovery state across authentication cases.
        for case in cases:
            subprocess.run([sys.executable, __file__, args.binary, '--case', case],
                           check=True, timeout=90)
        print('PASS all published SDK1.3.0 NET/4 ROS2 loopback cases')
        return
    rclpy.init()
    node = rclpy.create_node('prism_network_mock_test')
    counts = collections.Counter()
    qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
    subs = []
    ready = False

    def diagnostic(msg):
        nonlocal ready
        # DDS endpoint discovery precedes SDK authentication/identity setup.
        # The driver publishes idle only after enabling its command queue.
        ready = any(item.name == 'Prism ROS adapter' and item.hardware_id == 'MOCK'
                    and item.message.split(';', 1)[0] == 'idle' for item in msg.status)

    diagnostic_sub = node.create_subscription(DiagnosticArray, '/diagnostics', diagnostic, 10)

    def camera(index, msg):
        assert bytes(msg.data) == bytes([255, index, 217]), 'camera mapping changed'
        counts['camera' + str(index)] += 1

    for index in range(4):
        subs.append(node.create_subscription(CompressedImage, f'/prism/camera{index}/image/compressed',
                    lambda msg, index=index: camera(index, msg), qos))
    subs.append(node.create_subscription(Imu, '/prism/imu0/data', lambda _: counts.update(['imu']), qos))
    subs.append(node.create_subscription(CameraFrameMetadata, '/prism/camera/metadata',
                lambda _: counts.update(['meta']), qos))

    def spin_until(check, limit=15):
        deadline = time.monotonic() + limit
        while not check() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert check(), dict(counts)

    password = 'loopback-only mixed CASE password'
    for case in (args.case,):
        mock = MockAgent('', network_token=password)
        port = mock.listener.getsockname()[1]
        env = os.environ.copy()
        env.pop('PRISM_TEST_NETWORK_TOKEN', None)
        if case != 'missing-token':
            env['PRISM_TEST_NETWORK_TOKEN'] = 'wrong' if case == 'bad-token' else password
        command = [args.binary, '--ros-args', '-p', 'transport:=network', '-p', 'network_host:=127.0.0.1',
                   '-p', f'network_port:={port}', '-p', 'network_token_env:=PRISM_TEST_NETWORK_TOKEN',
                   '-p', 'device_serial:=' + ('NOT-MOCK' if case == 'serial-mismatch' else 'MOCK'),
                   '-p', 'navigation_enabled:=true', '-p', 'gnss_raw_enabled:=true',
                   '-p', 'camera_fps:=30', '-p', 'reliable_sensor_qos:=true']
        client = node.create_client(ControlStreams, '/prism/streams/control')
        if case == 'stream':
            command += ['-p', 'camera_enabled:=false', '-p', 'board_imu_enabled:=false']
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
            try:
                if case == 'stream':
                    # Match the test endpoints before sending START, not after
                    # filling the stream queues. Keep spinning while discovering
                    # the service so the observer does not block its callbacks.
                    spin_until(lambda: ready and client.service_is_ready() and
                               all(node.count_publishers(sub.topic_name) == 1 for sub in subs))
                    assert mock.commands[6] == mock.commands[8] == 0
                    start = ControlStreams.Request()
                    start.command, start.camera, start.board_imu = 'start', True, True
                    started = client.call_async(start)
                    spin_until(started.done)
                    assert started.result().success, started.result().message
                    spin_until(lambda: all(counts[key] >= 6 for key in ['camera0', 'camera1', 'camera2', 'camera3', 'meta']) and counts['imu'] >= 480)
                    assert mock.commands[0x3A] >= 2 and mock.commands[0x44] >= 2, 'navigation was disabled instead of fixing starvation'
                    req = ControlStreams.Request()
                    req.command, req.camera, req.board_imu = 'stop', True, True
                    future = client.call_async(req)
                    spin_until(future.done)
                    result = future.result()
                    assert result.success and not result.state.camera_active and not result.state.board_imu_active, result.message
                    assert mock.commands[6] == 1 and mock.commands[8] == 1
                    assert mock.commands[7] == 0 and mock.commands[9] == 1  # aggregate STOP
                    assert mock.acks and all(v == 1 for v in mock.acks.values())
                else:
                    # A ROS node may remain alive to expose the failure diagnostics.
                    deadline = time.monotonic() + 3
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=.05)
                    assert mock.commands[6] == mock.commands[8] == 0, 'negative case reached START'
            except BaseException:
                log.flush()
                log.seek(0)
                print('FAIL', case, 'commands', dict(mock.commands),
                      'counts', dict(counts), 'mock error', repr(mock.error))
                print(log.read()[-6000:].replace(password, '<redacted>'))
                raise
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                process.wait(timeout=15)
                mock.listener.close()
                mock.thread.join(timeout=1)
                log.seek(0)
                text = log.read()
                assert password not in text, 'password leaked into log'
                expected = {'missing-token': 'missing or empty', 'bad-token': 'authentication rejected',
                            'serial-mismatch': 'serial does not match', 'stream': 'Network camera FPS ceiling: 10'}[case]
                assert expected in text, (case, text[-3000:])
                if case == 'stream':
                    assert not any('stream=camera' in line and 'timestamp_gap' in line for line in text.splitlines()), 'camera health uses requested30 instead of negotiated10'
                assert 'Opening USB' not in text, 'network failure fell back to USB'
                assert mock.error is None, repr(mock.error)
                node.destroy_client(client)
            print('PASS', case, 'commands', dict(mock.commands), 'ACKs', len(mock.acks))
    print('PASS published SDK1.3.0 NET/4 ROS2 loopback:', dict(counts))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
