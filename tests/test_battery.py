import copy
import json
from pathlib import Path
import math
import struct

import pytest

from neo2_control.battery import BatteryReceiver, FcBatteryOsd, age_battery
from neo2_control.duml import DumlPacket


def body(*, percent=72, voltage=12345, flags=0, low=25, serious=15, type_byte=1, length=10):
    return (struct.pack('<IHBBBB', flags, voltage, low, serious, percent, type_byte)
            + bytes(max(0, length - 10)))[:length]


def frame(payload=None, *, sender=3, receiver=2, sequence=7, command_type=0x40,
          version=1, command_set=3, command_id=0x55):
    return DumlPacket(sender, receiver, sequence, command_type, command_set, command_id,
                      body() if payload is None else payload, version=version).encode()


def tunnel(payload, *, sender=0x27):
    return DumlPacket(sender, 0xEE, 0, 0, 0x51, 1, payload).encode()


def test_saved_sdk_layout_voltage_type_and_threshold_masks():
    decoded = FcBatteryOsd(body(low=0x80 | 25, serious=0x80 | 15, type_byte=0xFE))
    values = decoded.sdk_values()
    assert values['BatteryVoltage'] == 12345
    assert values['BatteryPowerPercent'] == 72 and values['FCBatteryType'] == 2
    assert values['LowBatteryWarningThreshold'] == 25
    assert values['SeriousLowBatteryWarningThreshold'] == 15
    fields = decoded.fields()
    assert fields['voltage']['value'] == 12345
    assert fields['voltage']['unit'] == 'sdk_voltage'
    assert fields['voltage']['reason'] == 'scale_and_unit_unconfirmed'
    assert fields['percent']['raw_hex'] == '48'
    assert fields['low_warning_percent']['offset_bytes'] == 6


@pytest.mark.parametrize('bit', range(32))
def test_native_alert_bits_remain_distinct_and_unused_bits_do_not_invent_faults(bit):
    values = FcBatteryOsd(body(flags=1 << bit)).sdk_values()
    expected = {
        'RequireGoHomeByLowBatteryVoltage': 4,
        'RequireLandingByLowBatteryVoltage': 5,
        'RequireGoHomeBySmartBatteryPercent': 2,
        'RequireLandingBySmartBatteryPercent': 3,
        'IsBatteryFirstChargeNotFull': 11,
    }
    assert {key: value for key, value in values.items() if key in expected} == {
        key: bit == n for key, n in expected.items()}
    assert values['VoltageWarningInfo'] == (bit == 4, bit == 5)


@pytest.mark.parametrize('percent', [0, 1, 20, 100, 101, 127, 255])
def test_out_of_range_percent_is_unknown_without_clamp_or_sentinel_guess(percent):
    decoded = FcBatteryOsd(body(percent=percent))
    assert decoded.sdk_values()['BatteryPowerPercent'] == percent
    field = decoded.fields()['percent']
    assert field['valid'] == (percent <= 100)
    assert field['value'] == (percent if percent <= 100 else None)
    if percent > 100:
        assert field['reason'] == 'invalid_percent_range'
    json.dumps(decoded.fields(), allow_nan=False)


@pytest.mark.parametrize('length', [0, 1, 4, 8, 9, 1011])
def test_short_or_oversized_payload_cannot_supply_partial_battery_state(length):
    with pytest.raises(ValueError, match='10..1010'):
        FcBatteryOsd(bytes(length))


def test_sources_tunnels_and_invalid_percentage_never_fall_back_to_another_source():
    receiver = BatteryReceiver()
    receiver.feed(frame(), 1)
    receiver.feed(frame(body(percent=255), sender=0x23), 1.2)
    receiver.feed(tunnel(frame()), 1.5)
    result = receiver.snapshot(2.1)
    assert len(result['observations']) == 3 and not result['observations'][0]['fresh']
    assert not result['observations'][1]['fields']['percent']['valid']
    assert result['observations'][1]['fields']['percent']['value'] is None
    assert result['observations'][2]['fields']['percent']['value'] == 72
    assert not result['flight_ready'] and not result['source_route_verified']


def test_bounded_routes_fragmentation_batching_and_reused_sequence():
    receiver = BatteryReceiver()
    packet = frame()
    for offset in range(0, len(packet), 3):
        receiver.feed(packet[offset:offset + 3], 1)
    receiver.feed(frame(body(percent=71)), 2)
    assert receiver.accepted == 2
    assert receiver.snapshot(2)['observations'][0]['fields']['percent']['value'] == 71
    receiver.feed(b''.join(frame(sender=i) for i in range(16)), 3)
    assert receiver.accepted == 18 and receiver.evicted == 8
    assert len(receiver.snapshot(3)['observations']) == 8


@pytest.mark.parametrize('change', [
    {'command_type': 0xC0}, {'command_type': 0x60}, {'version': 2},
    {'command_set': 0x0D}, {'command_id': 0x43},
])
def test_other_battery_family_responses_or_commands_cannot_refresh_fc_sample(change):
    receiver = BatteryReceiver()
    receiver.feed(frame(), 1)
    receiver.feed(frame(**change), 3)
    assert receiver.accepted == 1
    assert not receiver.snapshot(3)['observations'][0]['fields']['percent']['valid']


def test_crc_truncation_and_second_tunnel_are_rejected():
    receiver = BatteryReceiver()
    bad = bytearray(frame())
    bad[-1] ^= 1
    receiver.feed(bytes(bad), 1)
    receiver.feed(tunnel(tunnel(frame())), 2)
    receiver.feed(frame(bytes(9)), 3)
    assert not receiver.snapshot(3)['available']
    assert receiver.malformed == 1


def test_detached_fields_age_and_close_without_revalidating_bad_percent():
    receiver = BatteryReceiver()
    receiver.feed(frame(body(percent=255)), 1)
    original = receiver.snapshot(1)
    aged = age_battery(copy.deepcopy(original), .1, True)
    field = aged['observations'][0]['fields']['percent']
    assert not field['valid'] and field['reason'] == 'invalid_percent_range'
    closed = age_battery(copy.deepcopy(original), .1, False)
    assert closed['observations'][0]['fields']['voltage']['reason'] == 'session_inactive'
    assert not closed['observations'][0]['fields']['voltage']['valid']
    assert original['observations'][0]['fields']['voltage']['valid']


@pytest.mark.parametrize('bad', [True, 0, -1, math.inf, math.nan, 10 ** 1000])
def test_maximum_age_has_a_finite_positive_numeric_contract(bad):
    with pytest.raises(ValueError):
        BatteryReceiver(maximum_age=bad)


def test_clock_input_bounds_and_new_sessions_cannot_reuse_old_data():
    receiver = BatteryReceiver()
    receiver.feed(frame(), 2)
    receiver.snapshot(4)
    for bad in (True, math.inf, math.nan, 10 ** 1000, 3):
        with pytest.raises(ValueError):
            receiver.feed(frame(), bad)
    with pytest.raises(ValueError):
        receiver.feed(bytes(1024 * 1024 + 1), 4)
    for bad in (True, math.nan, -1, 10 ** 1000):
        with pytest.raises(ValueError):
            age_battery({}, bad, True)
    assert not BatteryReceiver().snapshot(0)['available']


def test_portable_fc_battery_values_match_complete_saved_native_callback():
    fixture = json.loads((Path(__file__).parent / 'fixtures/battery-osd-native.json').read_text())
    assert len(fixture['vectors']) == 11
    for vector in fixture['vectors']:
        decoded = FcBatteryOsd(bytes.fromhex(vector['payload_hex']))
        assert json.loads(json.dumps(decoded.sdk_values())) == vector['native'], vector['name']
        percent = decoded.fields()['percent']
        if vector['native']['BatteryPowerPercent'] > 100:
            assert percent['value'] is None and not percent['valid']
