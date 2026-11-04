from dataclasses import replace
import json
from pathlib import Path

import pytest

from neo2_control.duml import DumlPacket
from neo2_control.flight import (
    FlightAction, FlightActionReply, flight_action, parse_action_response,
    virtual_joystick,
)


def response(request, status=0):
    return DumlPacket(request.receiver, request.sender, request.seq, 0xC0,
                      request.cmd_set, request.cmd_id, bytes([status]))


@pytest.mark.parametrize('status', [0, 1, 2, 127, 255])
@pytest.mark.parametrize('kind',[0x80,0xc0])
def test_matched_action_status_is_an_acknowledgement_without_execution_proof(status,kind):
    request = flight_action(FlightAction.LAND, 0x8123)
    reply = parse_action_response(replace(response(request, status),cmd_type=kind), request)
    assert reply.status_code == status and reply.accepted == (status == 0)
    assert reply.as_dict()['execution_confirmed'] is False


@pytest.mark.parametrize('change', [
    {'sender': 0x23}, {'receiver': 3}, {'seq': 0x8124}, {'cmd_type': 0x40},
    {'cmd_set': 7}, {'cmd_id': 0x29}, {'version': 2}, {'payload': b''},
    {'cmd_type':0x81}, {'cmd_type':0xc1}, {'cmd_type':0xa0},
])
def test_unrelated_or_empty_response_cannot_accept_pending_action(change):
    request = flight_action(FlightAction.TAKEOFF, 0x8123)
    with pytest.raises(ValueError, match='does not match'):
        parse_action_response(replace(response(request), **change), request)


@pytest.mark.parametrize('change', [
    {'sender': 3}, {'receiver': 2}, {'receiver': 259}, {'seq': -1},
    {'seq': True}, {'cmd_type': 0}, {'cmd_set': 1}, {'cmd_id': 2},
    {'version': 2}, {'payload': b'\x00'}, {'payload': b'\x01\x00'},
])
def test_response_cannot_validate_an_unsupported_request(change):
    request = replace(flight_action(FlightAction.TAKEOFF, 0x8123), **change)
    with pytest.raises(ValueError):
        parse_action_response(response(request), request)


def test_receiver_index_and_result_tail_match_native_contract():
    request = flight_action(FlightAction.LAND, 0xFFFF, receiver_index=5)
    assert request.receiver == 0xA3
    packet = replace(response(request), payload=b'\x00\xff')
    assert parse_action_response(packet, request).accepted


@pytest.mark.parametrize('request_hex,response_hex',[
    ('550e0466020342bb40032a0173d7','550e0466030242bb80032a0049e3'),
    ('550e0466020345bb40032a0239f9','550e0466030245bb80032a0098ff'),
])
def test_first_material_trial_crc_valid_replies_match_exact_requests(request_hex,response_hex):
    # Sanitized flight frames only: no identity, coordinates or credentials.
    request=DumlPacket.decode(bytes.fromhex(request_hex))
    reply=DumlPacket.decode(bytes.fromhex(response_hex))
    assert parse_action_response(reply,request).accepted


@pytest.mark.parametrize('bad', [True, -1, 65536, 1.5])
def test_both_command_builders_require_valid_sequence(bad):
    with pytest.raises(ValueError):
        flight_action(FlightAction.LAND, bad)
    with pytest.raises(ValueError):
        virtual_joystick((0, 0, 0, 0), bad)


def test_actions_and_indexes_must_be_explicit():
    for action in (True, 1, 'TAKEOFF'):
        with pytest.raises(ValueError):
            flight_action(action, 0)
    for index in (True, -1, 8, 1.5):
        with pytest.raises(ValueError):
            flight_action(FlightAction.LAND, 0, receiver_index=index)
    for status in (True, -1, 256, 1.5):
        with pytest.raises(ValueError):
            FlightActionReply(status)


def test_portable_joystick_vectors_match_saved_complete_native_frames():
    fixture = json.loads((Path(__file__).parent / 'fixtures/virtual-joystick-native.json').read_text())
    checked = 0
    for vector in fixture['virtual_vectors']:
        channels = tuple(vector['channels'])
        if any(abs(channel) > 1000 for channel in channels):
            continue  # Native has no clamp; this API deliberately bounds UI inputs.
        cache = vector['cache_Int64Msg_value']
        timestamp = None if cache is None else (vector['synthetic_now_ns'] // 1000 - cache) // 1000
        packet = virtual_joystick(channels, 0x8123, relative_time_ms=timestamp)
        assert packet.encode().hex() == vector['native_frame_hex']
        assert DumlPacket.decode(packet.encode()) == packet
        checked += 1
    assert checked == 3


@pytest.mark.parametrize('channels', [(), (0, 0, 0), (0, 0, 0, 0, 0), None,
                                     (1001, 0, 0, 0), (-1001, 0, 0, 0),
                                     (0, True, 0, 0), (0, 0, 1.5, 0)])
def test_joystick_rejects_invalid_values_instead_of_masking_or_clamping(channels):
    with pytest.raises(ValueError):
        virtual_joystick(channels, 0)


@pytest.mark.parametrize('timestamp', [-1, 1 << 64, True, 1.5])
def test_joystick_clock_requires_uint64(timestamp):
    with pytest.raises(ValueError):
        virtual_joystick((0, 0, 0, 0), 0, relative_time_ms=timestamp)


def test_offline_service_and_authority_codecs_match_independent_complete_native_frames():
    from neo2_control.flight import wifi_control_service, joystick_control_authority
    fixture = json.loads((Path(__file__).parent / 'fixtures/flight-route-native.json').read_text())
    assert not fixture['neo2_runtime_product_selected']
    for vector in fixture['vectors']:
        if vector['kind'] == 'service_mode':
            packet = wifi_control_service(vector['mode'] == 2, fixture['sequence'])
        else:
            packet = joystick_control_authority(vector['value'] == 1, fixture['sequence'])
        assert packet.encode().hex() == vector['frame_hex']
        assert packet.payload.hex() == vector['payload_hex']
        assert DumlPacket.decode(packet.encode()) == packet


@pytest.mark.parametrize('bad', [None, 0, 1, 2, 'true', 1.0])
def test_service_and_authority_require_explicit_selections(bad):
    from neo2_control.flight import wifi_control_service, joystick_control_authority
    for codec in (wifi_control_service, joystick_control_authority):
        with pytest.raises(ValueError, match='explicit boolean'):
            codec(bad, 0)
