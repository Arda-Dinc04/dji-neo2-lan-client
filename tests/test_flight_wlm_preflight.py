"""A scheduled flight frame is revalidated before the real socket call."""
from dataclasses import replace

import pytest

from neo2_control.flight import FlightAction
from neo2_control.flight_channel import FlightChannel
from test_wlm_link import IDENTITY, follow, send
from test_wlm_query import beacon
from test_wlm_service_link import ready_link, incoming_prepare
from neo2_control.duml import DumlPacket
from neo2_control.wlm_discovery import unpack_forwarded


def committed_link():
    link,_=ready_link()
    link.flight_channel=FlightChannel(clock=lambda:link.last_time)
    send(link,1.6,'wlm_link_switch_trigger')
    prepare=DumlPacket.decode(incoming_prepare(link))
    forwarded=unpack_forwarded(prepare.payload)
    def frame(payload,sequence,kind):
        inner=replace(forwarded.packet,cmd_id=6,payload=payload,seq=sequence,cmd_type=kind)
        return replace(prepare,payload=inner.encode()+forwarded.attributes).encode()
    link.feed(prepare.encode(),1.7)
    send(link,1.7,'wlm_link_switch_prepare_ack')
    own=send(link,1.8,'wlm_link_switch_completion')
    link.feed(frame(IDENTITY.wlm_device_id+b'\0',own.sequence,0x80),1.9)
    link.feed(frame(link.peer_id+IDENTITY.wlm_device_id+bytes(5),0x1234,0x40),1.9)
    send(link,1.9,'wlm_link_switch_result_ack')
    original=beacon()
    body=bytearray(original.payload)
    body[24:27]=bytes([5,4,4]);body[52:54]=bytes([2,1])
    link.on_beacon(replace(original,payload=bytes(body),
        forwarding_source_hash=link.read.peer_hash,
        forwarding_destination_hash=link.identity.wlm_device_hash),2)
    send(link,2,'wlm_link_presence')
    assert link.flight_channel.snapshot()['route_ready']
    return link


@pytest.mark.parametrize('change',['backend','modes','revoke','deadline','peer_expired','reboot'])
def test_preflight_cancels_unsent_packet_without_advancing_forward_counter(change):
    link=committed_link()
    channel=link.flight_channel
    channel.submit_action(FlightAction.TAKEOFF,expires_at=2.2)
    outgoing=link.next_packet(2.01)
    assert outgoing.kind=='wlm_link_flight'
    previous=link.forwarding_sequence
    now=2.02
    if change=='backend':link.prepare_wifi=lambda modes:False
    elif change=='modes':link.feed(follow(link),now)
    elif change=='revoke':channel.revoke()
    elif change=='deadline':now=2.2
    elif change=='peer_expired':now=4
    else:
        body=bytearray(link.peer_beacon.payload);body[49]^=1
        link.on_beacon(replace(link.peer_beacon,payload=bytes(body)),now)
    assert not link.flight_send_allowed(outgoing.sequence,now)
    assert link.pending is None and link.forwarding_sequence==previous
    assert channel.snapshot()['sent_count']==0


def test_landing_has_priority_over_pending_mode_follow_without_dropping_follow_reply():
    link=committed_link()
    link.feed(follow(link,modes=(5,4,4),service=(2,1)),2.01)
    assert link.follow_reply is not None
    link.flight_channel.submit_action(FlightAction.LAND,expires_at=2.2)
    outgoing=link.next_packet(2.02)
    assert outgoing.kind=='wlm_link_flight'
    assert unpack_forwarded(outgoing.packet.payload).packet.payload==b'\2'
    assert link.flight_send_allowed(outgoing.sequence,2.02)
    link.sent(outgoing.kind,2.02)
    assert link.follow_reply is not None
    send(link,2.03,'wlm_link_follow_reply')


def pending_action(link):
    intent=link.flight_channel.submit_action(FlightAction.TAKEOFF,expires_at=2.2)
    outgoing=link.next_packet(2.01)
    request=unpack_forwarded(outgoing.packet.payload).packet
    assert link.flight_send_allowed(outgoing.sequence,2.01)
    link.sent(outgoing.kind,2.01)
    return intent,request


@pytest.mark.parametrize('carrier',['direct','broadcast','unicast'])
@pytest.mark.parametrize('kind',[0x80,0xc0])
def test_observed_action_response_carriers_still_require_exact_pending_request(carrier,kind):
    link=committed_link();intent,request=pending_action(link)
    response=DumlPacket(request.receiver,request.sender,request.seq,kind,3,0x2a,b'\0')
    raw=response.encode()
    if carrier!='direct':
        outer=DumlPacket.decode(incoming_prepare(link,destination_hash=(
            0xffffffff if carrier=='broadcast' else link.identity.wlm_device_hash)))
        attrs=unpack_forwarded(outer.payload).attributes
        raw=replace(outer,payload=raw+attrs,cmd_type=0).encode()
    link.feed(raw,2.02)
    receipt=link.flight_channel.receipt(intent['intent_id'])
    assert receipt['state']=='acknowledged' and receipt['acknowledged_at']==2.02
    assert receipt['execution_confirmed'] is False


@pytest.mark.parametrize('change',['sequence','sender','receiver','encrypted','empty',
                                 'backend','commit','generation'])
def test_direct_action_reply_cannot_bypass_correlation_or_owned_generation(change):
    link=committed_link();intent,request=pending_action(link)
    response=DumlPacket(request.receiver,request.sender,request.seq,0x80,3,0x2a,b'\0')
    if change=='sequence':response=replace(response,seq=response.seq^1)
    elif change=='sender':response=replace(response,sender=0x23)
    elif change=='receiver':response=replace(response,receiver=0x22)
    elif change=='encrypted':response=replace(response,cmd_type=0x81)
    elif change=='empty':response=replace(response,payload=b'')
    elif change=='backend':link.prepare_wifi=lambda modes:False
    elif change=='commit':link.service_switch.committed=False
    else:
        body=bytearray(link.peer_beacon.payload);body[49]^=1
        link.on_beacon(replace(link.peer_beacon,payload=bytes(body)),2.015)
    link.feed(response.encode(),2.02)
    assert link.flight_channel.receipt(intent['intent_id'])['state']!='acknowledged'


def test_direct_ack_before_actual_socket_send_is_never_saved_for_replay():
    link=committed_link();channel=link.flight_channel
    intent=channel.submit_action(FlightAction.TAKEOFF,expires_at=2.2)
    outgoing=link.next_packet(2.01);request=unpack_forwarded(outgoing.packet.payload).packet
    response=DumlPacket(request.receiver,request.sender,request.seq,0x80,3,0x2a,b'\0').encode()
    link.feed(response,2.015)
    assert channel.receipt(intent['intent_id'])['state']=='scheduled'
    assert link.flight_send_allowed(outgoing.sequence,2.02);link.sent(outgoing.kind,2.02)
    assert channel.receipt(intent['intent_id'])['state']=='sent'
    link.feed(response,2.03)
    assert channel.receipt(intent['intent_id'])['state']=='acknowledged'
