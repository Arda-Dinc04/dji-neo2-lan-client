from dataclasses import replace
import json
from pathlib import Path
import struct

import pytest

from neo2_control.duml import DumlPacket
from neo2_control.wifi_control import (
    WifiDeviceList, WifiDeviceRecord, wifi_control_mode,
    wifi_control_response, wifi_device_selection,
)
from neo2_control.wlm_discovery import WlmBeacon


FIXTURE = json.loads((Path(__file__).parent / 'fixtures/wifi-control-native.json').read_text())


def devices(*records, tail=b''):
    return WifiDeviceList.decode(struct.pack('<H', len(records)) + b''.join(records) + tail)


def record(flag=1, timestamp=1, identity=None):
    raw = bytearray.fromhex(FIXTURE['identity_cases'][0]['record_hex'])
    raw[25] = flag
    struct.pack_into('<q', raw, 45, timestamp)
    if identity is not None:
        raw[:23] = identity
    return bytes(raw)


def beacon(identity=None):
    body = bytearray(55)
    body[0] = 5
    body[1:24] = record()[:23] if identity is None else identity
    return WlmBeacon(bytes(body))


def response(request, body):
    return DumlPacket(request.receiver, request.sender, request.seq, 0xc0,
                      request.cmd_set, request.cmd_id, body)


def selection():
    return wifi_device_selection(devices(record()), beacon(), 0x8123)


def test_full_requests_equal_saved_native_assembly_including_crc():
    assert selection().encode().hex() == FIXTURE['selection_frame']['native_frame_hex']
    assert wifi_control_mode(0x8123).encode().hex() == FIXTURE['mode_frame']


def test_identity_projection_preserves_raw_padding_and_native_string_semantics():
    for case in FIXTURE['identity_cases']:
        parsed = devices(bytes.fromhex(case['record_hex'])).selected
        assert parsed.canonical_id.hex() == case['expected_canonical_id_hex']
        assert (parsed.canonical_id == parsed.device_id) == case['raw_id_is_canonical']
        assert parsed.selection_timestamp == case['expected_selection_timestamp']


def test_selection_uses_native_signed_timestamp_and_nonzero_flag_cases():
    for case in FIXTURE['selection_cases']:
        parsed = devices(*(record(flag, stamp) for flag, stamp in case['entries']))
        assert (parsed.selected is not None) == bool(case['operation_control_available_byte222'])
        if parsed.selected is not None:
            assert parsed.selected.selection_timestamp == case['latest_selected_timestamp']
    # Equal timestamps retain the first native-selected record, never a sort
    # that changes its identity according to a secondary criterion.
    first = record(1, 2)
    second = record(2, 2, bytes((0, 0, 36)) + b'other'.ljust(20, b'\0'))
    assert devices(first, second).selected.payload == first
    assert devices(record(1, -(1 << 63)), record(1, 0)).selected is None
    assert devices(record(1, (1 << 63)-1)).selected.selection_timestamp == (1 << 63)-1


def test_distinct_status_byte_positions_match_actual_native_callbacks():
    selected, mode = selection(), wifi_control_mode(0x8124)
    for case in FIXTURE['selection_status_cases']:
        assert wifi_control_response(response(selected, bytes((case['status_byte0'], 0))), selected) == case['sdk_result']
    for case in FIXTURE['mode_status_cases']:
        assert wifi_control_response(response(mode, bytes((0xa5, case['status_byte1']))), mode) == case['sdk_result']
    # Native ignores result tail; the PC also permits it after minimum length.
    assert wifi_control_response(response(selected, b'\0\xff'), selected) == 0
    assert wifi_control_response(response(mode, b'\xff\0\xa5'), mode) == 0
    assert wifi_control_response(replace(response(mode, b'\0\0'), cmd_type=0x80), mode) == 0


@pytest.mark.parametrize('body', [b'', b'\0', b'\1\0', b'\1\0' + bytes(60), b'\xff\xff', bytes(1011)])
def test_truncated_or_oversized_count_body_cannot_create_devices(body):
    with pytest.raises(ValueError):
        WifiDeviceList.decode(body)


def test_native_extra_tail_is_preserved_without_becoming_another_record():
    parsed = devices(record(), tail=b'\x51\x14\xff')
    assert len(parsed.records) == 1
    assert parsed.trailing_bytes == b'\x51\x14\xff'
    assert devices(tail=b'extra').selected is None
    assert len(devices(*([record()] * 16), tail=bytes(32)).records) == 16


@pytest.mark.parametrize('identity', [
    bytes((0, 0, 36)) + bytes(20),
    bytes((0, 0, 36)) + b'one\0two'.ljust(20, b'\0'),
    bytes((0, 1, 36)) + b'id'.ljust(20, b'\0'),
    bytes((1, 0, 36)) + b'id'.ljust(20, b'\0'),
    bytes((0, 0, 35)) + b'id'.ljust(20, b'\0'),
])
def test_selection_rejects_empty_noncanonical_and_other_profile_identifiers(identity):
    with pytest.raises(ValueError):
        wifi_device_selection(devices(record(identity=identity)), beacon(identity), 0x8123)


def test_global_selected_record_must_match_peer_instead_of_falling_back_to_older_match():
    other = bytes((0, 0, 36)) + b'other'.ljust(20, b'\0')
    with pytest.raises(ValueError):
        wifi_device_selection(devices(record(timestamp=1), record(timestamp=2, identity=other)), beacon(), 0x8123)
    with pytest.raises(ValueError):
        wifi_device_selection(devices(), beacon(), 0x8123)


@pytest.mark.parametrize('change', [
    {'sender': 3}, {'receiver': 0xee}, {'seq': 0x8124}, {'cmd_type': 0x40},
    {'cmd_type': 0xa0}, {'cmd_type': 0xe0}, {'cmd_set': 3}, {'cmd_id': 0x14},
    {'version': 2}, {'payload': b''}, {'payload': bytearray(b'\0')},
    {'seq': True}, {'sender': 0x1ee}, {'payload': bytes(1011)},
])
def test_strict_pc_response_policy_rejects_unrelated_or_non_wire_values(change):
    request = selection()
    with pytest.raises(ValueError):
        wifi_control_response(replace(response(request, b'\0'), **change), request)


def test_mode_ack_requires_two_bytes_and_supported_request_body():
    request = wifi_control_mode(0x8123)
    with pytest.raises(ValueError):
        wifi_control_response(response(request, b'\0'), request)
    with pytest.raises(ValueError):
        wifi_control_response(response(request, b'\0\0'), replace(request, payload=bytes(32)))


@pytest.mark.parametrize('bad', [True, -1, 65536, 1.5, None])
def test_builders_require_a_real_uint16_sequence(bad):
    with pytest.raises(ValueError):
        wifi_control_mode(bad)
    with pytest.raises(ValueError):
        wifi_device_selection(devices(record()), beacon(), bad)


def test_direct_data_constructors_cannot_bypass_payload_bounds():
    for raw in (bytes(60), bytearray(61), bytes(62)):
        with pytest.raises(ValueError):
            WifiDeviceRecord(raw)
    for records, tail in (([], b''), ((b'not-a-record',), b''), ((WifiDeviceRecord(record()),)*17, b''), ((), bytearray())):
        with pytest.raises(ValueError):
            WifiDeviceList(records, tail)
