#!/usr/bin/env python3
"""Offline ROS2 v1.2.0 serialization checks. Does not open a device."""
from rclpy.serialization import deserialize_message, serialize_message
from prism_ros_msgs.msg import (GnssTimingStatus, GnssReceptionStatus,
    ReceiverPosition, GnssObservations, TimeSyncRtkStatus, TimeSyncRtkVersions,
    TimeSyncCorsStatus, RtcmData)
from prism_ros_msgs.srv import (ControlRtk, SetCorsConfiguration,
    SetTimeSyncPort, SetUnifiedExposure)

def roundtrip(value):
    return deserialize_message(serialize_message(value), type(value))

gnss = GnssTimingStatus()
gnss.time_synced, gnss.offset_fresh = True, True
gnss.message_pps_offset_us, gnss.nmea_age_ms = 800000, 455360
assert roundtrip(gnss).message_pps_offset_us == 800000
assert roundtrip(gnss).nmea_age_ms == 455360
position = ReceiverPosition()
position.source, position.solution, position.time_system = 'ADRNAV', 'NARROW_FLOAT', 'GPST'
position.quality, position.valid = 5, True
position.timestamp_valid = False
position.latitude_deg, position.longitude_deg = 31.1, 121.5
assert roundtrip(position).quality == 5 and roundtrip(position).latitude_deg == 31.1
assert not roundtrip(position).timestamp_valid
reception = GnssReceptionStatus()
reception.raw_byte_count, reception.nmea_rejected_count = 2**40, 123
assert roundtrip(reception).raw_byte_count == 2**40
observations = GnssObservations()
observations.cursor, observations.session, observations.gap = 42, 7, True
observations.sequences, observations.received_ms = [41, 42], [1000, 1001]
observations.sentences = ['$GNRMC,fixture', '#ADRNAVA,fixture']
assert roundtrip(observations).sentences == observations.sentences
module = TimeSyncRtkStatus()
module.control_state, module.control_generation, module.network_bytes = 2, 123, 65536
assert roundtrip(module).control_state == 2 and roundtrip(module).control_generation == 123
version = TimeSyncRtkVersions()
version.application_valid, version.application_major = True, 1
assert roundtrip(version).application_valid
cors = TimeSyncCorsStatus()
cors.configuration_saved, cors.configuration_applied = True, False
cors.username, cors.credentials_present = 'TEST_ONLY', True
assert roundtrip(cors).configuration_saved and not roundtrip(cors).configuration_applied
assert 'password' not in cors.get_fields_and_field_types()
request = SetCorsConfiguration.Request()
request.password, request.confirm = 'TEST_ONLY_NOT_A_CREDENTIAL', True
assert roundtrip(request).password == request.password
control = ControlRtk.Request()
control.command, control.confirm, control.expected_cors_generation = 'start', True, 42
control.allow_gga, control.timeout_ms = False, 20000
assert not roundtrip(control).allow_gga and roundtrip(control).expected_cors_generation == 42
mode = SetTimeSyncPort.Request()
mode.mode, mode.confirm = 2, False
assert roundtrip(mode).mode == 2 and not roundtrip(mode).confirm
exposure = SetUnifiedExposure.Request()
exposure.enabled = True
assert roundtrip(exposure).enabled
raw = RtcmData()
raw.data = list(range(256)) * 64
assert bytes(roundtrip(raw).data) == bytes(raw.data)
print('PASS: v1.2.0 receiver/module/CORS/control/RTCM serialization and write-only password')
