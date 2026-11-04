import json
import math
from pathlib import Path
import struct

import pytest

from neo2_control.duml import DumlPacket
from neo2_control.telemetry import TelemetryReceiver, decode_fc_osd,ground_velocity_to_aircraft


def body(*, flags=0x380000 | 0x8000 | 12, extension=0, length=85):
    raw = bytearray(85)
    struct.pack_into('<dd7h', raw, 0, 0.3, -0.4, 123, -45, 67, -89, 100, -200, 300)
    struct.pack_into('<I', raw, 32, flags)
    raw[30], raw[36], raw[68] = 0xFF, 15, extension
    return bytes(raw[:length])


def frame(payload=None, *, sequence=1, sender=3, receiver=2, command_type=0x40):
    return DumlPacket(sender, receiver, sequence, command_type, 3, 0x43,
                      payload if payload is not None else body()).encode()


def test_numeric_values_flags_scales_raw_offsets_and_units():
    decoded = decode_fc_osd(body())
    values = decoded.sdk_values()
    assert values['altitude'] == (123 * 0.1,)
    assert values['velocity'] == tuple(v * 0.1 for v in (-45, 67, -89))
    assert values['attitude'] == tuple(v * 0.1 for v in (100, -200, 300))
    assert values['is_flying'] == values['motors_on'] == values['gps_valid'] == 1
    assert values['flight_mode'] == 127 and values['satellite_count'] == 15
    assert values['location'] == (-0.4 * 180 / math.pi, 0.3 * 180 / math.pi)
    fields = decoded.fields()
    assert fields['altitude']['raw_hex'] == '7b00'
    assert fields['altitude']['unit'] == 'm' and fields['altitude']['reference'] == 'relative'
    assert fields['altitude']['physical_accuracy_verified'] is False
    assert fields['velocity']['unit']=='m/s'
    assert fields['velocity']['reason'] == 'sdk_unit_verified_physical_accuracy_unverified'
    assert fields['velocity']['reference']=='sdk_ground_frame'
    assert fields['velocity']['physical_accuracy_verified'] is False
    assert fields['velocity']['axis_labels_verified'] is False
    assert fields['attitude']['unit'] == 'degree'


def test_sdk_ground_to_aircraft_rotation_does_not_invent_axis_labels_or_position():
    assert ground_velocity_to_aircraft((1.,2.,-.3),0)==(1.,2.,-.3)
    assert ground_velocity_to_aircraft((1.,2.,-.3),90)==pytest.approx((2.,-1.,-.3))
    assert ground_velocity_to_aircraft((1.,2.,-.3),-90)==pytest.approx((-2.,1.,-.3))
    with pytest.raises(ValueError):ground_velocity_to_aircraft((1.,float('nan'),0),0)
    with pytest.raises(ValueError):ground_velocity_to_aircraft((1.,2.,0),True)


@pytest.mark.parametrize('flags', range(16))
def test_flying_and_motor_bits_do_not_become_a_guessed_ground_state(flags):
    values = decode_fc_osd(body(flags=flags)).sdk_values()
    assert values['motors_on'] == (flags >> 3) & 1
    assert values['is_flying'] == int(flags & 14 == 12)


def test_location_availability_and_nonfinite_values_are_separate_from_gps_bit():
    assert decode_fc_osd(body(flags=0)).fields()['location']['value'] is None
    fallback = decode_fc_osd(body(flags=0, extension=1))
    assert not fallback.sdk_values()['gps_valid']
    assert fallback.fields()['location']['valid']
    bad = bytearray(body())
    struct.pack_into('<d', bad, 0, float('nan'))
    fields = decode_fc_osd(bytes(bad)).fields()
    assert fields['location']['value'] is None and not fields['location']['valid']
    assert fields['location']['reason'] == 'invalid_geographic_value'
    json.dumps(fields, allow_nan=False)


@pytest.mark.parametrize('length', [0, 1, 16, 32, 36, 1011])
def test_truncated_or_oversize_body_cannot_invent_values(length):
    with pytest.raises(ValueError):
        decode_fc_osd(bytes(length))


def test_fragmented_tunnel_payload_and_freshness_use_monotonic_pc_receipt():
    receiver = TelemetryReceiver(maximum_age=1)
    outer = DumlPacket(0x27, 0xEE, 0x1234, 0, 0x51, 1, frame()).encode()
    for offset in range(0, len(outer), 3):
        receiver.feed(outer[offset:offset + 3], 2)
    snapshot = receiver.snapshot(2.5)
    assert snapshot['accepted'] == 1 and snapshot['fresh']
    assert snapshot['source']['wifi_tunnel'] and snapshot['flight_phase_code'] == 6
    assert snapshot['fields']['motors_on']['value'] is True
    assert snapshot['fields']['motors_on']['age_seconds'] == 0.5
    assert not snapshot['flight_ready'] and not snapshot['source_clock_verified']
    stale = receiver.snapshot(3.01)
    assert not stale['fresh'] and not stale['fields']['motors_on']['valid']
    assert stale['fields']['motors_on']['reason'] == 'stale'


def test_valid_new_push_can_reuse_duml_sequence():
    receiver = TelemetryReceiver()
    receiver.feed(frame(sequence=0), 1)
    receiver.feed(frame(body(flags=0), sequence=0), 2)
    assert receiver.snapshot(2)['fields']['motors_on']['value'] is False
    assert receiver.accepted == 2


@pytest.mark.parametrize('kwargs', [dict(sender=0x23), dict(receiver=3), dict(command_type=0xC0)])
def test_other_routes_or_responses_cannot_refresh_flight_telemetry(kwargs):
    receiver = TelemetryReceiver()
    receiver.feed(frame(), 1)
    receiver.feed(frame(sequence=2, **kwargs), 4)
    assert receiver.accepted == 1 and receiver.ignored == 1
    assert not receiver.snapshot(4)['fresh']


def test_corrupted_crc_and_recursive_tunnels_cannot_update_store():
    receiver = TelemetryReceiver()
    raw = bytearray(frame())
    raw[-1] ^= 1
    receiver.feed(bytes(raw), 1)
    inner = DumlPacket(0x27, 0xEE, 1, 0, 0x51, 1, frame()).encode()
    outer = DumlPacket(0x27, 0xEE, 2, 0, 0x51, 1, inner).encode()
    receiver.feed(outer, 2)
    unsupported = DumlPacket(0x27, 0xEE, 3, 0, 0x51, 1, frame(), version=2).encode()
    receiver.feed(unsupported, 3)
    corrupted_carrier = bytearray(DumlPacket(0x27, 0xEE, 4, 0, 0x51, 1, frame()).encode())
    corrupted_carrier[-1] ^= 1
    receiver.feed(bytes(corrupted_carrier), 4)
    assert not receiver.snapshot(4)['available'] and receiver.accepted == 0


def test_large_reliable_block_is_streamed_with_bounded_framing():
    receiver = TelemetryReceiver()
    receiver.feed(b''.join(frame(sequence=i) for i in range(80)), 1)
    assert receiver.accepted == 80 and len(receiver.decoder.buffer) == 0


def test_time_cannot_go_backwards_and_new_sessions_start_empty():
    receiver = TelemetryReceiver()
    receiver.feed(frame(), 2)
    receiver.snapshot(4)
    with pytest.raises(ValueError):
        receiver.feed(frame(sequence=2), 3)
    with pytest.raises(ValueError):
        receiver.snapshot(float('nan'))
    assert not TelemetryReceiver().snapshot(0)['available']


def test_saved_synthetic_native_vectors_match_decoder():
    fixture = json.loads((Path(__file__).parent / 'fixtures/fc-osd-native.json').read_text())
    for vector in fixture['vectors']:
        expected = vector['native']
        actual = decode_fc_osd(bytes.fromhex(vector['body_hex'])).sdk_values()
        assert json.loads(json.dumps(actual)) == expected
