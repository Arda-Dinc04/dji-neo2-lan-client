"""Native beacon bytes and the finite, socket-driven PC announcement policy."""
import json
from pathlib import Path

import pytest

from neo2_control.duml import DumlPacket
from neo2_control.identity import WlmIdentity
from neo2_control.wlm_discovery import unpack_forwarded
from neo2_control.wlm_presence import WlmPresence, own_presence

IDENTITY = WlmIdentity('pc-state-synthetic')
ORACLE = json.loads((Path(__file__).parent/'fixtures/wlm-own-beacon-native.json').read_text())


def test_owned_beacon_matches_actual_native_producer_and_forwarding_layout():
    packet = own_presence(IDENTITY, peer_format=5, nonce=0x1234, counter=0)
    forwarded = unpack_forwarded(packet.payload)
    assert forwarded.packet.encode().hex() == ORACLE['synthetic_wifi_discovery_candidate']['frame_hex']
    assert (packet.sender, packet.receiver, packet.cmd_set, packet.cmd_id, packet.seq) == (0x3b, 0xe9, 0x51, 1, 1)
    assert forwarded.attributes.hex() == 'e0076c550201000000ffffffff018200000000000000'
    assert DumlPacket.decode(packet.encode()) == packet
    assert not forwarded.packet.payload[51] & 0x50


@pytest.mark.parametrize('fields', [{'peer_format':4}, {'peer_format':True}, {'nonce':0},
    {'nonce':65536}, {'nonce':True}, {'counter':-1}, {'counter':2**32},
    {'forwarding_sequence':2**32}, {'forwarding_sequence':False}])
def test_presence_rejects_profiles_that_cannot_honestly_disable_v3_or_bad_fields(fields):
    arguments = dict(peer_format=5, nonce=1, counter=0)
    arguments.update(fields)
    with pytest.raises(ValueError): own_presence(IDENTITY, **arguments)


def test_actual_socket_completion_controls_spacing_counter_nonce_and_bound():
    presence = WlmPresence(IDENTITY)
    packets = []
    for scheduled, completed in ((0, 2), (2.5, 3), (3.5, 4)):
        packet = presence.next_packet(scheduled)
        packets.append(unpack_forwarded(packet.payload))
        assert presence.next_packet(scheduled) is None
        assert presence.sent_count == len(packets)-1
        presence.sent(completed)
        assert presence.next_packet(completed+.49) is None
    assert presence.snapshot(4.49)['state'] == 'settling'
    assert presence.snapshot(4.5)['state'] == 'complete'
    assert presence.next_packet(100) is None
    assert [int.from_bytes(p.packet.payload[39:43], 'little') for p in packets] == [0, 1, 2]
    nonces = {int.from_bytes(p.packet.payload[49:51], 'little') for p in packets}
    assert len(nonces) == 1 and 0 not in nonces
    assert all(int.from_bytes(p.attributes[5:9], 'little') == 1 for p in packets)
    assert not presence.snapshot(100)['peer_registration_confirmed']


def test_cancellation_stops_unsent_announcement_and_has_monotonic_clock():
    presence = WlmPresence(IDENTITY)
    presence.next_packet(0)
    presence.cancel(1, 'session_closed')
    assert presence.next_packet(2) is None
    assert presence.snapshot(2)['datagrams_sent'] == 0
    with pytest.raises(RuntimeError): presence.sent(2)
    with pytest.raises(ValueError): presence.snapshot(1)
