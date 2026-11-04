"""The drone's observed ACK80 is distinct from the APK native ACKc0."""
from dataclasses import replace
import json
from pathlib import Path

import pytest

from neo2_control.duml import DumlPacket
from neo2_control.wlm_discovery import WlmDiscovery, unpack_forwarded
from test_wlm_query import read
from test_wlm_query_session import announce, client
from test_session import status

FIXTURE = json.loads((Path(__file__).parent/'fixtures/wlm-real-capability-reply.json').read_text())
REPLY = DumlPacket.decode(bytes.fromhex(FIXTURE['carrier_hex']))


def carrier(ack_type=0x80, sequence=0x8123):
    forwarded = unpack_forwarded(REPLY.payload)
    return replace(REPLY, payload=replace(forwarded.packet, cmd_type=ack_type,
                   seq=sequence).encode()+forwarded.attributes).encode()


@pytest.mark.parametrize('ack_type', [0x80, 0xc0])
def test_real_capabilities_complete_only_the_matching_read_for_both_plain_ack_types(ack_type):
    query = read()
    query.sent(0)
    for byte in carrier(ack_type):query.feed(bytes([byte]), .1)
    result = query.snapshot(.2)
    assert result['state'] == 'complete'
    assert result['response_status'] == 0
    assert result['capabilities'] == FIXTURE['expected_capabilities']
    assert not result['negotiated'] and not result['application_ready']


@pytest.mark.parametrize('ack_type', [0, 0x40, 0x81, 0xa0, 0xc1, 0xe0])
def test_real_reply_does_not_expand_acceptance_to_requests_or_unverified_ack_variants(ack_type):
    query = read();query.sent(0);query.feed(carrier(ack_type), .1)
    assert query.snapshot(.1)['state'] == 'awaiting_response'
    assert query.snapshot(1.5)['state'] == 'timed_out'


def test_ack80_remains_subject_to_sequence_correlation_and_discovery_counts_it_separately():
    query = read();query.sent(0);query.feed(carrier(sequence=0x8124), .1)
    assert query.snapshot(.1)['state'] == 'awaiting_response'
    discovery = WlmDiscovery();discovery.feed(carrier(), 0)
    state = discovery.snapshot(0)
    assert state['ability_acknowledgements_received'] == 1
    assert state['ability_messages_received'] == 0
    assert not state['negotiated'] and not state['discovered']


def test_session_accepts_observed_ack_after_presence_without_becoming_flight_ready():
    peer = client();announce(peer)
    packet, = [p for p in peer.tick(1.6) if p.application_kind == 'wlm_capability_query']
    peer.datagram_sent(packet, 1.6)
    peer.receive(status(payload=carrier(sequence=packet.duml_sequence)), 1.7)
    state = peer.snapshot(1.7)
    assert state['wlm_capability_read']['state'] == 'complete'
    assert state['wlm_capability_read']['capabilities'] == FIXTURE['expected_capabilities']
    assert state['wlm_discovery']['ability_acknowledgements_received'] == 1
    assert not state['application_ready']
