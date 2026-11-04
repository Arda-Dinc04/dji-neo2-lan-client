"""Native WLM wire fixtures, bounded observation and owned PC UDP reception."""

from dataclasses import replace
import json
from pathlib import Path

import pytest

from neo2_control.duml import DumlPacket
from neo2_control.identity import WlmIdentity
from neo2_control.session import SwudpSession
from neo2_control.transport import ReliableTransport, describe_transport
from neo2_control.wlm_discovery import AbilityAcknowledgement, AbilityMessage, CapabilityVector, WlmBeacon, WlmDiscovery, age_discovery, unpack_forwarded
from test_session import acceptance, status
from test_observation import LoopbackDrone, observed, until


FIXTURE = json.loads((Path(__file__).parent / 'fixtures/wlm-discovery-native.json').read_text())
EXAMPLES = FIXTURE['beacons']
CARRIER = bytes.fromhex(EXAMPLES[0]['carrier_hex'])
STATE = json.loads((Path(__file__).parent / 'fixtures/wlm-state-native.json').read_text())


@pytest.mark.parametrize('row', json.loads((Path(__file__).parent / 'fixtures/wlm-endpoint-native.json').read_text())['identities'])
def test_own_routing_hash_matches_native_setter_projection(row):
    identity = WlmIdentity(row['synthetic_identification'])
    assert identity.wlm_device_hash == row['hash_u32']
    assert identity.device_sn.encode().ljust(20, b'\0') == bytes.fromhex(row['device_id_hex'])[3:]
    assert str(identity.wlm_device_hash) not in json.dumps(identity.metadata())


@pytest.mark.parametrize('example', EXAMPLES)
def test_native_beacon_forwarding_and_fields_without_leaking_identity(example):
    raw = bytes.fromhex(example['forwarded_hex'])
    forwarded = unpack_forwarded(raw)
    assert forwarded.packet.encode().hex() == example['inner_hex']
    assert forwarded.attributes.hex() == example['attributes_hex']
    assert forwarded.link_category == 2
    beacon = WlmBeacon(forwarded.packet.payload)
    metadata = beacon.metadata()
    assert metadata['format_code'] == 5 and metadata['beacon_counter'] == 100
    assert metadata['native_ssfn'] == 0x22334455 and metadata['native_word_49'] == 0x0102
    assert beacon.device_id == forwarded.packet.payload[1:24]
    assert example['synthetic_serial'] not in repr(beacon) + repr(forwarded) + json.dumps(metadata)
    if example is EXAMPLES[0]:
        assert (metadata['device_type'], metadata['device_index'], metadata['device_model']) == (0, 0, 36)


@pytest.mark.parametrize('example', FIXTURE['ability_requests'])
def test_native_ability_header_and_tlv_boundaries(example):
    message = AbilityMessage.decode(bytes.fromhex(example['payload_hex']))
    assert (message.operation, message.flags, message.subtype, message.reserved) == (1, 0, 1, 0)
    assert message.device_hash == 0x12345678
    assert [kind for kind, _ in message.tlvs] == [6, 10]
    assert [len(value) for _, value in message.tlvs] == [17, 17]
    video, control = [CapabilityVector.decode(value) for _, value in message.tlvs]
    first, second = example['synthetic_flags']
    assert video.word_count == control.word_count == 2
    assert video.mask == control.mask == 0
    assert video.support == 0x1001f
    assert control.support == 0x47d | first << 8 | second << 9


@pytest.mark.parametrize('raw', [b'', bytes([0]), bytes([0x21])+bytes(16),
    bytes([0x10])+bytes(16), bytes([0x20])+bytes(15)])
def test_capability_version_and_banks_are_bounded(raw):
    with pytest.raises(ValueError):
        CapabilityVector.decode(raw)


@pytest.mark.parametrize('mutation', ['short', 'long', 'bad_inner_crc', 'extra_between', 'unknown_version'])
def test_forwarding_is_one_checked_inner_frame_and_exact_trailer(mutation):
    raw = bytes.fromhex(EXAMPLES[0]['forwarded_hex'])
    if mutation == 'short':
        raw = raw[:34]
    elif mutation == 'long':
        raw = bytes(1011)
    elif mutation == 'bad_inner_crc':
        raw = raw[:20] + bytes([raw[20] ^ 1]) + raw[21:]
    elif mutation == 'extra_between':
        raw = raw[:-22] + b'\0' + raw[-22:]
    else:
        packet = replace(unpack_forwarded(raw).packet, version=2)
        raw = packet.encode() + raw[-22:]
    with pytest.raises(ValueError):
        unpack_forwarded(raw)


@pytest.mark.parametrize('raw', [b'', bytes(8), bytes(1011),
    bytes.fromhex('10010078563412010006'), bytes.fromhex('100100785634120300061100')])
def test_ability_length_and_truncated_tlv_cannot_escape(raw):
    with pytest.raises(ValueError):
        AbilityMessage.decode(raw)


@pytest.mark.parametrize('example', STATE['native_ack_header_projection'])
def test_native_acknowledgement_header_is_distinct_from_request(example):
    packet = DumlPacket.decode(bytes.fromhex(example['native_ack_frames'][0]['frame_hex']))
    assert packet.cmd_type == 0xc0
    message = AbilityAcknowledgement.decode(packet.payload)
    assert (message.status, message.operation, message.flags, message.subtype, message.reserved) == (
        0, example['operation'], 0, 2, (0, 0))
    assert message.tlvs == ()
    with pytest.raises(ValueError):
        AbilityMessage.decode(packet.payload)


def test_native_read_only_query_response_contains_both_vectors():
    result = STATE['read_only_query']['native_query_consumer_result']['emitted_frames'][0]
    message = AbilityAcknowledgement.decode(bytes.fromhex(result['payload_hex']))
    assert (message.status, message.operation, message.subtype) == (0, 2, 1)
    assert message.device_hash == WlmIdentity('pc-state-synthetic').wlm_device_hash
    assert [kind for kind, _ in message.tlvs] == [6, 10]
    vectors = [CapabilityVector.decode(raw) for _, raw in message.tlvs]
    assert [v.support for v in vectors] == [0x1001f, 0x47d]


@pytest.mark.parametrize('raw', [b'', bytes(10), bytes(1011),
    bytes.fromhex('002000010078563412010006'),
    bytes.fromhex('0020000100785634120300061100')])
def test_ability_ack_truncation_and_length_rejected(raw):
    with pytest.raises(ValueError):
        AbilityAcknowledgement.decode(raw)


def test_ack_observation_does_not_prove_negotiation_or_refresh_beacon():
    result = STATE['read_only_query']['native_query_consumer_result']['emitted_frames'][0]
    native = DumlPacket.decode(bytes.fromhex(result['frame_hex']))
    # Direction reversal is a synthetic inbound fixture, not a drone response.
    inbound = replace(native, sender=0xe9, receiver=0xee).encode()
    carrier = replace(DumlPacket.decode(CARRIER), payload=inbound+bytes.fromhex(EXAMPLES[0]['attributes_hex']))
    receiver, events = WlmDiscovery(), []
    receiver.feed(CARRIER, 0)
    receiver.feed(carrier.encode(), 1.9, lambda name, **detail: events.append((name, detail)))
    snapshot = receiver.snapshot(2.1)
    assert not snapshot['discovered'] and not snapshot['negotiated'] and not snapshot['application_ready']
    assert snapshot['beacons_received'] == 1 and snapshot['ability_acknowledgements_received'] == 1
    assert snapshot['ability_messages_received'] == 0
    assert events == [('wlm_ability_ack_observed', {'status': 0, 'operation': 2, 'subtype': 1,
        'tlv_types': [6, 10], 'application_ready': False})]


def test_fragmentation_counter_duplicates_expiry_and_detached_snapshot():
    receiver, events = WlmDiscovery(), []
    for byte in CARRIER:
        receiver.feed(bytes([byte]), 1, lambda name, **detail: events.append((name, detail)))
    receiver.feed(CARRIER, 1.5)
    result = receiver.snapshot(2)
    assert result['beacons_received'] == 2 and len(result['peers']) == 1
    assert result['peers'][0]['beacons'] == 2 and result['peers'][0]['age_seconds'] == .5
    assert result['discovered'] and not result['negotiated'] and not result['negotiation_enabled']
    assert len(events) == 1 and events[0][0] == 'wlm_peer_discovered'
    aged = age_discovery(result, 4, session_active=True)
    assert not aged['discovered'] and aged['peers'][0]['age_seconds'] == 2.5
    assert result['peers'][0]['age_seconds'] == .5
    assert not age_discovery(result, 2, session_active=False)['discovered']
    with pytest.raises(ValueError):
        receiver.feed(CARRIER, 1.9)
    private = EXAMPLES[0]['synthetic_serial']
    assert private not in json.dumps(result) + json.dumps(events)


def test_bounded_peer_eviction_preserves_only_latest_native_ids():
    receiver = WlmDiscovery(maximum_peers=2)
    for index, example in enumerate(EXAMPLES):
        receiver.feed(bytes.fromhex(example['carrier_hex']), index)
    snapshot = receiver.snapshot(2)
    assert snapshot['beacons_received'] == 3 and snapshot['evicted_peers'] == 1
    assert len(snapshot['peers']) == 2 and snapshot['maximum_peers'] == 2


@pytest.mark.parametrize('level,changes', [('carrier', {'sender': 0x28}),
    ('carrier', {'receiver': 2}), ('carrier', {'cmd_type': 0xc0}),
    ('carrier', {'cmd_type': 1}), ('inner', {'sender': 0xe8}),
    ('inner', {'receiver': 2}), ('inner', {'cmd_type': 0xc0}),
    ('inner', {'cmd_type': 1}), ('category', {'category': 1})])
def test_unrelated_encrypted_or_ack_messages_cannot_discover_or_refresh_peer(level, changes):
    receiver = WlmDiscovery()
    receiver.feed(CARRIER, 0)
    carrier = DumlPacket.decode(CARRIER)
    if level == 'carrier':
        carrier = replace(carrier, **changes)
    elif level == 'inner':
        forwarded = unpack_forwarded(carrier.payload)
        carrier = replace(carrier, payload=replace(forwarded.packet, **changes).encode() + forwarded.attributes)
    else:
        raw = bytearray(carrier.payload)
        raw[-18] = changes['category']
        carrier = replace(carrier, payload=bytes(raw))
    receiver.feed(carrier.encode(), 1.9)
    snapshot = receiver.snapshot(2.1)
    assert snapshot['beacons_received'] == 1 and not snapshot['discovered']


def test_crc_invalid_carrier_cannot_promote_embedded_beacon():
    receiver = WlmDiscovery()
    raw = bytearray(CARRIER)
    raw[-1] ^= 1
    receiver.feed(bytes(raw), 0)
    assert receiver.snapshot(0)['beacons_received'] == 0
    assert receiver.decoder.discarded_bytes == len(raw)


def test_qos_session_observes_discovery_without_enabling_or_emitting_negotiation():
    client = SwudpSession(transport=ReliableTransport(session=0x1234, base_sequence=0x4000))
    client.start(0)
    client.receive(acceptance(flags=15), 0)
    client.receive(status(payload=CARRIER), .1)
    snapshot = client.snapshot(.1)
    assert snapshot['wlm_discovery']['discovered'] and not snapshot['application_ready']
    for packet in client.tick(.1):
        decoded, _ = describe_transport(packet.raw)
        assert decoded['type'] == 4 and decoded['qos_aggregate_length'] == 0
    client.close(.2)
    assert not client.snapshot(.2)['wlm_discovery']['discovered']


def test_actual_udp_owned_snapshots_age_discovery_while_transport_remains_live(tmp_path):
    peer = LoopbackDrone()
    peer.osd = CARRIER
    try:
        with observed(tmp_path, peer) as (_, manager):
            manager.open({'duration': 5})
            until(lambda: manager.snapshot()['session'].get('wlm_discovery', {}).get('discovered'))
            peer.telemetry = False
            until(lambda: not manager.snapshot()['session']['wlm_discovery']['discovered'], timeout=3)
            state = manager.snapshot()
            assert state['connected'] and state['session']['wlm_discovery']['beacons_received'] >= 1
            manager.close()
            assert not manager.snapshot()['session']['wlm_discovery']['discovered']
            until(lambda: not manager.busy)
            assert manager.result['wlm_discovery']['beacons_received'] >= 1
            assert not manager.result['wlm_discovery']['discovered']
            journal = Path(manager.result['journal_path']).read_text()
            assert 'wlm_peer_discovered' in journal and EXAMPLES[0]['synthetic_serial'] not in journal
    finally:
        peer.close()
