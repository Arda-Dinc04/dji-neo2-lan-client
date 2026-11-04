import copy
import json
from pathlib import Path
import math

import pytest

from neo2_control.duml import DumlPacket
from neo2_control.wifi_status import (
    WifiControlOsd, WifiControlReceiver, age_wifi_control, UNKNOWN_MODE,
)


def body(*, byte20=2, byte15=0, byte24=2, byte26=0, prefix_nonzero=True, length=27):
    raw = bytearray(max(27, length))
    raw[0] = int(prefix_nonzero)
    raw[15], raw[20], raw[24], raw[26] = byte15, byte20, byte24, byte26
    return bytes(raw[:length])


def frame(payload=None, *, sender=0x27, receiver=2, sequence=7, command_type=0x40,
          version=1, command_set=0x51, command_id=4):
    return DumlPacket(sender, receiver, sequence, command_type, command_set, command_id,
                      body() if payload is None else payload, version=version).encode()


def tunnel(payload, *, sender=0x27, receiver=0xEE, sequence=9):
    return DumlPacket(sender, receiver, sequence, 0, 0x51, 1, payload).encode()


def test_mode_and_service_guard_are_independent_not_flight_authority():
    # A mode2 sample may have a false SDK service guard.
    sample = WifiControlOsd(body(byte20=0))
    assert sample.mode_code == 2 and not sample.service_guard
    # A true guard may accompany an unknown mode.
    sample = WifiControlOsd(body(prefix_nonzero=False))
    assert sample.mode_code == UNKNOWN_MODE and sample.service_guard
    receiver = WifiControlReceiver()
    receiver.feed(frame(), 1)
    result = receiver.snapshot(1)
    assert result['available'] and result['observations'][0]['mode_name'] == 'OPERATION_CONTROL'
    assert not result['flight_ready']
    assert not result['runtime_product_verified'] and not result['source_route_verified']
    assert not result['source_clock_verified']


@pytest.mark.parametrize('byte15', [0, 1, 3, 4, 5, 7, 8, 255])
def test_legacy_payload_reads_only_the_legacy_mode_bit(byte15):
    sample = WifiControlOsd(body(byte20=1, byte15=byte15, length=21))
    assert sample.mode_code == (byte15 >> 2) & 1
    assert not sample.service_guard


@pytest.mark.parametrize('length', [0, 15, 20, 1011])
def test_framing_guards_short_and_oversized_bodies(length):
    with pytest.raises(ValueError, match='21..1010'):
        WifiControlOsd(bytes(length))


@pytest.mark.parametrize('length', range(21, 27))
def test_short_nonlegacy_body_retains_guard_without_inventing_mode(length):
    sample = WifiControlOsd(body(length=length))
    assert sample.service_guard and sample.mode_code == UNKNOWN_MODE
    receiver = WifiControlReceiver()
    receiver.feed(frame(body(length=length)), 1)
    fields = receiver.snapshot(1)['observations'][0]['fields']
    assert not fields['wifi_mode']['valid'] and fields['wifi_mode']['value'] is None
    assert fields['service_guard']['valid'] and fields['service_guard']['value']


def test_sources_and_tunnel_carriers_are_separate_and_age_independently():
    receiver = WifiControlReceiver()
    a = frame()
    receiver.feed(a, 1)
    receiver.feed(frame(body(byte20=1, byte15=0), sender=0x47), 1.5)
    receiver.feed(tunnel(a), 1.8)
    receiver.feed(tunnel(a, sender=0x47), 1.9)
    result = receiver.snapshot(2.1)
    assert len(result['observations']) == 4
    assert not result['observations'][0]['fresh']
    assert all(x['fresh'] for x in result['observations'][1:])
    assert [x['mode_code'] for x in result['observations']] == [2, 0, 2, 2]
    assert result['observations'][2]['source']['carrier']['sender'] == 0x27
    assert result['observations'][3]['source']['carrier']['sender'] == 0x47


def test_sources_are_bounded_and_repeated_duml_sequence_refreshes_same_route():
    receiver = WifiControlReceiver()
    for sender in range(16):
        receiver.feed(frame(sender=sender, sequence=0), sender)
    assert len(receiver.observations) == 8 and receiver.evicted == 8
    receiver.feed(frame(sender=15, sequence=0), 16)
    result = receiver.snapshot(16)
    assert len(result['observations']) == 8 and receiver.accepted == 17
    assert result['observations'][-1]['age_seconds'] == 0
    assert result['observations'][-1]['source']['sequence'] == 0
    assert result['observations'][0]['source']['sender'] == 8


def test_fragmented_batched_duml_crc_and_tunnel_framing():
    receiver = WifiControlReceiver()
    packet = frame()
    for start in range(0, len(packet), 3):
        receiver.feed(packet[start:start + 3], 1)
    assert receiver.accepted == 1
    receiver.feed(b''.join(frame(sequence=i) for i in range(80)), 2)
    assert receiver.accepted == 81 and not receiver.decoder.buffer
    bad = bytearray(packet)
    bad[-1] ^= 1
    receiver.feed(bytes(bad), 3)
    receiver.feed(tunnel(tunnel(frame())), 3)
    assert receiver.accepted == 81
    receiver.feed(tunnel(frame() + b'junk'), 4)
    assert receiver.accepted == 81 and receiver.malformed >= 1


@pytest.mark.parametrize('changes', [
    {'command_type': 0x60}, {'command_type': 0xC0}, {'version': 2},
    {'command_set': 3}, {'command_id': 0x1A},
])
def test_unrelated_commands_or_acknowledgments_cannot_refresh_mode(changes):
    receiver = WifiControlReceiver()
    receiver.feed(frame(), 1)
    receiver.feed(frame(**changes), 4)
    assert receiver.accepted == 1
    assert receiver.ignored == 1 or receiver.decoder.discarded_bytes > 0
    assert not receiver.snapshot(4)['observations'][0]['fresh']


def test_age_detached_snapshot_and_session_close_without_changing_owner_copy():
    receiver = WifiControlReceiver()
    receiver.feed(frame(), 1)
    original = receiver.snapshot(1)
    stale = age_wifi_control(copy.deepcopy(original), 1.1, True)
    assert not stale['observations'][0]['fresh']
    assert stale['observations'][0]['fields']['service_guard']['reason'] == 'stale'
    closed = age_wifi_control(copy.deepcopy(original), .1, False)
    assert not closed['observations'][0]['fields']['wifi_mode']['valid']
    assert closed['observations'][0]['fields']['wifi_mode']['reason'] == 'session_inactive'
    assert original['observations'][0]['fields']['wifi_mode']['valid']
    assert not closed['flight_ready']


@pytest.mark.parametrize('bad', [True, 0, -1, math.inf, math.nan, 10 ** 1000])
def test_freshness_limit_must_be_positive_finite_and_not_bool(bad):
    with pytest.raises(ValueError):
        WifiControlReceiver(maximum_age=bad)


@pytest.mark.parametrize('bad', [True, -1, math.inf, math.nan, 10 ** 1000])
def test_elapsed_snapshot_age_is_bounded_to_nonnegative_finite_time(bad):
    with pytest.raises(ValueError):
        age_wifi_control({}, bad, True)


def test_clocks_cannot_move_backwards_and_new_session_starts_empty():
    receiver = WifiControlReceiver()
    receiver.feed(frame(), 2)
    receiver.snapshot(4)
    with pytest.raises(ValueError):
        receiver.feed(frame(), 3)
    with pytest.raises(ValueError):
        receiver.snapshot(math.nan)
    with pytest.raises(ValueError):
        receiver.feed(bytes(1024 * 1024 + 1), 4)
    assert not WifiControlReceiver().snapshot(0)['available']


def test_portable_parser_matches_saved_native_wifi_callback_and_rejects_short_reads():
    fixture = json.loads((Path(__file__).parent / 'fixtures/wifi-control-osd-native.json').read_text())
    compared = rejected = 0
    for row in fixture['vectors']:
        payload = bytes.fromhex(row['body_hex'])
        if len(payload) < 21:
            with pytest.raises(ValueError):
                WifiControlOsd(payload)
            rejected += 1
        elif row['expected']['update_called']:
            actual = WifiControlOsd(payload)
            assert actual.mode_code == row['expected']['mode'], row['id']
            assert int(actual.service_guard) == row['expected']['service_guard_byte'], row['id']
            compared += 1
        # Null/expired object and response fixtures concern SDK lifecycle,
        # not payload parsing in this always-present PC receiver.
    assert compared == 63 and rejected == 8


@pytest.mark.parametrize('command_type', [0, 0x20, 0x40])
def test_plaintext_push_ack_modes_refresh_wifi_battery_and_fc_fields(command_type):
    from neo2_control.battery import BatteryReceiver
    from neo2_control.telemetry import TelemetryReceiver
    from test_battery import frame as battery_frame
    from test_telemetry import frame as fc_frame
    for receiver, packet in (
        (WifiControlReceiver(), frame(command_type=command_type)),
        (BatteryReceiver(), battery_frame(command_type=command_type)),
        (TelemetryReceiver(), fc_frame(command_type=command_type)),
    ):
        receiver.feed(packet, 1)
        assert receiver.accepted == 1
        assert receiver.snapshot(1)['available']


@pytest.mark.parametrize('command_type', [1, 0x21, 0x41, 0x60, 0x80, 0xA0, 0xC0, 0xE0])
def test_response_encrypted_or_reserved_push_headers_do_not_refresh_observations(command_type):
    from neo2_control.battery import BatteryReceiver
    from neo2_control.telemetry import TelemetryReceiver
    from test_battery import frame as battery_frame
    from test_telemetry import frame as fc_frame
    for receiver, packet in (
        (WifiControlReceiver(), frame(command_type=command_type)),
        (BatteryReceiver(), battery_frame(command_type=command_type)),
        (TelemetryReceiver(), fc_frame(command_type=command_type)),
    ):
        receiver.feed(packet, 1)
        assert receiver.accepted == 0 and not receiver.snapshot(1)['available']
