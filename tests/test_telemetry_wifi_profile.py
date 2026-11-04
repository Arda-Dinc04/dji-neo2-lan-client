"""Exact native WLM framing and the two measured FC broadcast recipients."""
from dataclasses import replace
import struct

import pytest

from neo2_control.duml import DumlPacket
from neo2_control.telemetry import TelemetryReceiver
from test_telemetry import body, frame


def attributes(*, source_hash=0x12345678, destination_hash=0xffffffff, category=2):
    return struct.pack('<IBIIBB7x', source_hash, category, 0x76543210,
                       destination_hash, 1, 0x82)


def carrier(inner=None, *, trailer=None, **changes):
    if inner is None:
        inner = frame(receiver=0x0e, command_type=0)
    packet = DumlPacket(0x27, 0xee, 0x4321, 0, 0x51, 1,
                        inner+(attributes() if trailer is None else trailer))
    return replace(packet, **changes).encode()


@pytest.mark.parametrize('recipient', [0x0e, 0xaa])
@pytest.mark.parametrize('kind', [0, 0x20, 0x40])
def test_native_fc_broadcast_retains_real_inner_route_carrier_and_fields(recipient, kind):
    raw = carrier(frame(receiver=recipient, sequence=0x9876, command_type=kind))
    receiver = TelemetryReceiver()
    for start in range(0, len(raw), 3):
        receiver.feed(raw[start:start+3], 2)
    result = receiver.snapshot(2.25)
    assert result['accepted'] == 1 and result['malformed'] == result['discarded_bytes'] == 0
    assert result['selection'] == 'last_received' and len(result['observations']) == 1
    source = result['source']
    assert (source['sender'], source['receiver'], source['sequence']) == (3, recipient, 0x9876)
    assert source['wifi_tunnel'] and source['command_type'] == kind
    assert source['carrier'] == {'sender': 0x27, 'receiver': 0xee,
                                  'sequence': 0x4321, 'command_type': 0}
    assert source['forwarding'] == {'source_hash': 0x12345678,
                                     'destination_hash': 0xffffffff, 'link_category': 2}
    assert result['fields']['altitude']['value'] == 12.3
    assert result['fields']['motors_on']['value'] is True
    assert result['fields']['motors_on']['source'] == source
    assert result['age_seconds'] == .25 and result['fresh']
    assert not result['flight_ready'] and not result['observations'][0]['flight_ready']


@pytest.mark.parametrize('envelope', ['direct', 'bare', 'native'])
def test_legacy_fc_route_and_minimum_payload_remain_compatible(envelope):
    raw = frame(body(length=37), receiver=2)
    if envelope == 'bare':
        raw = carrier(raw, trailer=b'')
    elif envelope == 'native':
        raw = carrier(raw)
    receiver = TelemetryReceiver()
    receiver.feed(raw, 1)
    result = receiver.snapshot(1)
    assert result['accepted'] == 1 and result['malformed'] == 0
    assert result['source']['receiver'] == 2
    assert result['source']['wifi_tunnel'] == (envelope != 'direct')


@pytest.mark.parametrize('change', [
    'direct_new', 'bare_new', 'outer_sender', 'outer_receiver', 'outer_ack',
    'outer_encrypted', 'outer_version', 'category', 'destination_hash',
    'inner_sender', 'inner_receiver', 'inner_ack', 'inner_encrypted', 'inner_version',
])
def test_unproved_sources_or_envelopes_cannot_refresh_existing_observation(change):
    inner = DumlPacket.decode(frame(receiver=0x0e, command_type=0))
    trailer, outer_changes = attributes(), {}
    if change == 'direct_new':
        raw = inner.encode()
    else:
        if change == 'bare_new':
            trailer = b''
        elif change == 'outer_sender':
            outer_changes['sender'] = 0x28
        elif change == 'outer_receiver':
            outer_changes['receiver'] = 2
        elif change == 'outer_ack':
            outer_changes['cmd_type'] = 0xc0
        elif change == 'outer_encrypted':
            outer_changes['cmd_type'] = 1
        elif change == 'outer_version':
            outer_changes['version'] = 2
        elif change == 'category':
            trailer = attributes(category=1)
        elif change == 'destination_hash':
            trailer = attributes(destination_hash=0x12345678)
        elif change == 'inner_sender':
            inner = replace(inner, sender=0x23)
        elif change == 'inner_receiver':
            inner = replace(inner, receiver=0x8e)
        elif change == 'inner_ack':
            inner = replace(inner, cmd_type=0x80)
        elif change == 'inner_encrypted':
            inner = replace(inner, cmd_type=1)
        elif change == 'inner_version':
            inner = replace(inner, version=2)
        raw = carrier(inner.encode(), trailer=trailer, **outer_changes)
    receiver = TelemetryReceiver()
    receiver.feed(frame(sequence=7), 0)
    receiver.feed(raw, 2)
    result = receiver.snapshot(2)
    assert result['accepted'] == 1 and len(result['observations']) == 1
    assert result['source']['sequence'] == 7 and result['source']['receiver'] == 2
    assert not result['fresh'] and not result['fields']['altitude']['valid']


@pytest.mark.parametrize('recipient', [0x0e, 0xaa])
@pytest.mark.parametrize('length', [0, 36, 37, 84, 86])
def test_new_recipient_profile_requires_the_observed_85_byte_payload(recipient, length):
    payload = body().ljust(length, b'\0')[:length]
    receiver = TelemetryReceiver()
    receiver.feed(carrier(frame(payload, receiver=recipient)), 1)
    assert receiver.accepted == 0 and receiver.malformed == 1


@pytest.mark.parametrize('damage', [
    'short_trailer', 'long_trailer', 'inner_crc', 'inner_header_crc',
    'outer_crc', 'prefix', 'two_bare_frames', 'extra_bare_byte',
])
def test_malformed_wrapper_never_falls_back_to_scanning_a_valid_embedded_fc(damage):
    inner, trailer = frame(), attributes()
    if damage == 'short_trailer':
        trailer = trailer[:-1]
    elif damage == 'long_trailer':
        trailer += b'\0'
    elif damage in ('inner_crc', 'inner_header_crc'):
        altered = bytearray(inner)
        altered[-1 if damage == 'inner_crc' else 3] ^= 1
        inner = bytes(altered)
    elif damage == 'prefix':
        inner = b'garbage'+inner
    elif damage == 'two_bare_frames':
        inner += inner
        trailer = b''
    elif damage == 'extra_bare_byte':
        trailer = b'\0'
    raw = carrier(inner, trailer=trailer)
    if damage == 'outer_crc':
        raw = raw[:-1]+bytes([raw[-1] ^ 1])
    receiver = TelemetryReceiver()
    receiver.feed(raw, 1)
    assert not receiver.snapshot(1)['available'] and receiver.accepted == 0
    receiver.feed(frame(sequence=2), 2)
    assert receiver.accepted == 1 and receiver.snapshot(2)['source']['sequence'] == 2


@pytest.mark.parametrize('native', [False, True])
def test_nested_forwarding_cannot_escape_through_native_or_bare_compatibility(native):
    nested = carrier(frame(), trailer=b'')
    receiver = TelemetryReceiver()
    receiver.feed(carrier(nested, trailer=attributes() if native else b''), 1)
    assert receiver.accepted == 0 and not receiver.snapshot(1)['available']


def test_forwarding_trailer_is_opaque_even_when_it_contains_a_valid_duml_frame():
    unrelated = replace(DumlPacket.decode(frame()), cmd_id=0x55).encode()
    trailer = bytearray(attributes())
    trailer[9:] = frame(b'')  # A complete13byte DUML fits inside the22byte attributes.
    receiver = TelemetryReceiver()
    receiver.feed(carrier(unrelated, trailer=bytes(trailer)), 1)
    assert receiver.accepted == 0 and receiver.ignored == 1 and receiver.malformed == 0


def test_recipient_observations_age_independently_and_latest_receipt_is_not_a_merge():
    receiver = TelemetryReceiver(maximum_age=1)
    first = frame(body(flags=0), receiver=0x0e, sequence=1)
    changed = bytearray(body())
    struct.pack_into('<h', changed, 16, 321)
    second = frame(bytes(changed), receiver=0xaa, sequence=2)
    receiver.feed(carrier(first), 1)
    receiver.feed(carrier(second), 2.5)
    result = receiver.snapshot(2.75)
    assert result['selection'] == 'last_received' and result['source']['receiver'] == 0xaa
    assert result['fields']['altitude']['value'] == 32.1
    older, newer = result['observations']
    assert older['source']['receiver'] == 0x0e and newer['source']['receiver'] == 0xaa
    assert older['age_seconds'] == 1.75 and not older['fresh']
    assert older['fields']['motors_on']['value'] is False
    assert older['fields']['motors_on']['reason'] == 'stale'
    assert newer['age_seconds'] == .25 and newer['fresh']
    assert newer['fields']['motors_on']['value'] is True
    result['fields']['altitude']['value'] = -1
    assert newer['fields']['altitude']['value'] == 32.1
    receiver.feed(carrier(first), 3)
    result = receiver.snapshot(3)
    assert result['source']['receiver'] == 0x0e and result['accepted'] == 3
    assert len(result['observations']) == 2
    assert result['observations'][0]['source']['receiver'] == 0xaa
    assert result['observations'][0]['age_seconds'] == .5
    assert not result['flight_ready']


def test_observations_are_bounded_by_route_and_forwarding_source_without_sequence_aliases():
    receiver = TelemetryReceiver()
    for source in range(1, 10):
        receiver.feed(carrier(trailer=attributes(source_hash=source)), source)
    result = receiver.snapshot(9)
    assert len(result['observations']) == result['maximum_sources'] == 8
    assert result['evicted'] == 1 and result['accepted'] == 9
    assert [sample['source']['forwarding']['source_hash'] for sample in result['observations']] == list(range(2, 10))
    receiver.feed(carrier(trailer=attributes(source_hash=2)), 10)
    receiver.feed(carrier(trailer=attributes(source_hash=10)), 11)
    result = receiver.snapshot(11)
    assert result['evicted'] == 2 and result['accepted'] == 11
    hashes = [sample['source']['forwarding']['source_hash'] for sample in result['observations']]
    assert 2 in hashes and 3 not in hashes and hashes[-1] == 10


def test_empty_receiver_has_no_default_or_inferred_source():
    result = TelemetryReceiver().snapshot(0)
    assert result['observations'] == [] and result['fields'] == {}
    assert result['selection'] == 'last_received' and 'source' not in result
    assert not result['available'] and not result['flight_ready']
