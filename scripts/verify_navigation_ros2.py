#!/usr/bin/env python3
"""ROS2 interface serialization tests; --device adds reversible USB smoke tests.

Run after sourcing ROS and this workspace. Hardware mode starts its own
navigation-only node; use only when no other process owns the Prism USB device.
No clock, persisted configuration, correction bytes or firmware are written.
"""
import argparse
import json
import os
import signal
import subprocess
import time

import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from prism_ros_msgs.msg import GnssTimingStatus, RtkNavigationStatus, RtcmData
from prism_ros_msgs.srv import (GetDeviceInfo, GetDeviceConfiguration,
    GetGnssTiming, GetRtkStatus, GetRtkNavigation, GetTimeSyncPort,
    SetDeviceConfiguration, SetRoverRtcm, ControlRtkCorrections, SyncSystemTime,
    ControlStreams, GetStreamState)
from sensor_msgs.msg import CompressedImage, Imu
from rclpy.qos import qos_profile_sensor_data


def offline():
    gnss = GnssTimingStatus()
    gnss.time_synced = True
    gnss.offset_fresh = True
    gnss.message_pps_offset_us = 800000
    gnss.nmea_age_ms = 455360
    decoded = deserialize_message(serialize_message(gnss), GnssTimingStatus)
    assert decoded.time_synced and decoded.message_pps_offset_us == 800000
    assert decoded.nmea_age_ms == 455360
    rtk = RtkNavigationStatus()
    rtk.solution_valid = rtk.smoothed_position_valid = True
    rtk.solution_epoch_us = 1800000000000000
    rtk.smoothed_solution_epoch_us = rtk.solution_epoch_us - 100000
    rtk.latitude_deg, rtk.smoothed_latitude_deg = 31.1, 31.2
    decoded = deserialize_message(serialize_message(rtk), RtkNavigationStatus)
    assert decoded.latitude_deg != decoded.smoothed_latitude_deg
    assert decoded.solution_epoch_us - decoded.smoothed_solution_epoch_us == 100000
    raw = RtcmData()
    raw.data = list(range(256)) * 64
    assert bytes(deserialize_message(serialize_message(raw), RtcmData).data) == bytes(raw.data)
    print('PASS: ROS2 GNSS/RTK/16384-byte RTCM serialization', flush=True)


def hardware(seconds, capture=False, camera_fps=0):
    rclpy.init()
    node = rclpy.create_node('prism_navigation_smoke_test')
    process = subprocess.Popen([
        'ros2', 'run', 'prism_ros_driver', 'prism_ros_driver_node', '--ros-args',
        '-p', 'camera_enabled:=false', '-p', 'board_imu_enabled:=false',
        '-p', 'lidar_enabled:=false', '-p', 'navigation_enabled:=true',
        '-p', 'camera_fps:=' + str(camera_fps)], start_new_session=True)
    rover_enabled = corrections_enabled = capture_started = False
    counts = dict(gnss=0, navigation=0, rover_chunks=0, rover_bytes=0)
    last = {}
    camera_counts = [0] * 4
    imu_counts = [0] * 2
    def on_sensor(counts, index):
        def receive(_):
            counts[index] += 1
        return receive
    def on_gnss(value):
        counts['gnss'] += 1
        last['gnss'] = value
    def on_navigation(value):
        counts['navigation'] += 1
        last['navigation'] = value
    def on_rtcm(value):
        counts['rover_chunks'] += 1
        counts['rover_bytes'] += len(value.data)
        last['rover'] = value
    subscriptions = [
        node.create_subscription(GnssTimingStatus, '/prism/gnss/timing', on_gnss, 100),
        node.create_subscription(RtkNavigationStatus, '/prism/rtk/navigation', on_navigation, 100),
        node.create_subscription(RtcmData, '/prism/gnss/rover_rtcm', on_rtcm, 1024)]

    for index in range(4):
        subscriptions.append(node.create_subscription(CompressedImage,
            '/prism/camera' + str(index) + '/image/compressed',
            on_sensor(camera_counts, index), 10))
    for index in range(2):
        subscriptions.append(node.create_subscription(Imu,
            '/prism/imu' + str(index) + '/data',
            on_sensor(imu_counts, index), qos_profile_sensor_data))

    def call(service_type, path, **fields):
        client = node.create_client(service_type, '/prism/' + path)
        try:
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError('service unavailable: ' + path)
            request = service_type.Request()
            for key, value in fields.items():
                setattr(request, key, value)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=15)
            if not future.done():
                raise RuntimeError('service timeout: ' + path)
            result = future.result()
            print(path, result.success, result.message, flush=True)
            return result
        finally:
            node.destroy_client(client)

    try:
        info = call(GetDeviceInfo, 'device/get_info')
        assert info.success and info.info.host_sdk_version == '1.1.0'
        assert info.info.agent_version == '1.1.0'
        config = call(GetDeviceConfiguration, 'device/get_configuration')
        assert config.success
        print('Persisted configuration:', config.configuration, flush=True)
        gnss = call(GetGnssTiming, 'gnss/get_timing')
        assert gnss.success
        assert call(GetRtkNavigation, 'rtk/get_navigation').success
        port = call(GetTimeSyncPort, 'system/get_timesync_port')
        assert port.success and port.status.mode == 0
        status = call(GetRtkStatus, 'rtk/get_status')
        assert status.success
        # Guaranteed no-op validation failures, never change persisted settings/time.
        assert not call(SetDeviceConfiguration, 'device/set_configuration',
                        confirm=True, set_gnss_uart_baud=True, gnss_uart_baud=12345).success
        assert not call(SyncSystemTime, 'system/sync_time', confirm=False).success
        if not status.status.host_active:
            corrections_enabled = call(ControlRtkCorrections, 'rtk/control_corrections', enable=True).success
            assert corrections_enabled
            assert call(ControlRtkCorrections, 'rtk/control_corrections', enable=False).success
            corrections_enabled = False
        rover_enabled = call(SetRoverRtcm, 'gnss/set_rover_rtcm', enable=True).success
        assert rover_enabled
        if capture:
            capture_started = call(ControlStreams, 'streams/control',
                                   command='start', camera=True, board_imu=True).success
            assert capture_started
        counts.update(gnss=0, navigation=0, rover_chunks=0, rover_bytes=0)
        camera_counts[:] = [0] * 4
        imu_counts[:] = [0] * 2
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            if process.poll() is not None:
                raise RuntimeError('driver exited: ' + str(process.returncode))
            rclpy.spin_once(node, timeout_sec=.05)
        duration = time.monotonic() - start
        assert counts['gnss'] > 0 and counts['navigation'] > 0
        if capture:
            assert sum(camera_counts) > 0 and sum(imu_counts) > 0
            assert call(ControlStreams, 'streams/control',
                        command='stop', camera=True, board_imu=True).success
            capture_started = False
        streams = call(GetStreamState, 'streams/get_state')
        assert streams.success and not streams.state.camera_active and not streams.state.board_imu_active
        after = call(GetDeviceConfiguration, 'device/get_configuration')
        assert after.success and after.configuration == config.configuration
        report = {**counts, 'duration_s': duration,
                  'gnss_hz': counts['gnss'] / duration,
                  'navigation_hz': counts['navigation'] / duration,
                  'configured_camera_fps': config.configuration.camera_fps,
                  'requested_camera_fps': camera_fps,
                  'camera_frames': camera_counts,
                  'imu_samples': imu_counts,
                  'agent': info.info.agent_version,
                  'sensor_board': info.info.sensor_board_version}
        if 'gnss' in last:
            g = last['gnss']
            report.update(external_time_synced=g.time_synced, satellites=g.satellites,
                          nmea_age_ms=g.nmea_age_ms, pps_valid=g.pps_valid)
        if 'rover' in last:
            report.update(rover_device_dropped_bytes=last['rover'].dropped_bytes,
                          rover_adapter_dropped_chunks=last['rover'].adapter_dropped_chunks)
        print(json.dumps(report, indent=2), flush=True)
        if counts['rover_bytes'] == 0:
            print('NOTE: raw rover service works, but no receiver bytes arrived; check GNSS input.', flush=True)
    finally:
        if capture_started and process.poll() is None:
            try:
                call(ControlStreams, 'streams/control',
                     command='stop', camera=True, board_imu=True)
            except Exception as error:
                print('capture cleanup:', error, flush=True)
        for enabled, service, path in [
            (rover_enabled, SetRoverRtcm, 'gnss/set_rover_rtcm'),
            (corrections_enabled, ControlRtkCorrections, 'rtk/control_corrections')]:
            if enabled and process.poll() is None:
                try:
                    call(service, path, enable=False)
                except Exception as error:
                    print('cleanup:', error, flush=True)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', action='store_true')
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--camera-fps', type=int, default=0, help='temporary camera FPS request; 0 uses persisted rate')
    parser.add_argument('--capture', action='store_true', help='also test camera/IMU; no disk recording')
    args = parser.parse_args()
    offline()
    if args.device:
        hardware(max(1.0, args.seconds), args.capture, max(0, args.camera_fps))
