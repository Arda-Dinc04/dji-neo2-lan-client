"""Service transaction ownership, strict peer routing and busy mode follow."""
from dataclasses import replace

import pytest

from neo2_control.duml import DumlPacket, DumlSequence
from neo2_control.wlm_link import WlmWifiLink
from neo2_control.wlm_discovery import CapabilityVector, unpack_forwarded
from test_wlm_link import IDENTITY, follow, send
from test_wlm_query import beacon
from test_wlm_query_session import TEMPLATE, FORWARDED
from test_wlm_real_ack import carrier


def ready_link():
    calls=[]
    def prepare(modes):calls.append(modes);return True
    link=WlmWifiLink(IDENTITY,DumlSequence(current=0x8000).next,
                     switch_wifi_service=True,prepare_wifi=prepare)
    link.on_beacon(beacon(),0)
    send(link,0,'wlm_link_presence')
    link.feed(follow(link),.1);send(link,.1,'wlm_link_follow_reply')
    for when in (.5,1):
        link.on_beacon(beacon(),when);send(link,when,'wlm_link_presence')
    send(link,1.5,'wlm_link_capability_query')
    link.feed(carrier(sequence=link.read.sequence),1.6)
    return link,calls


def incoming_prepare(link,*,source_hash=None,destination_hash=None):
    inner=DumlPacket(sender=0xe9,receiver=0xee,seq=0x1234,cmd_type=0x40,
        cmd_set=0x51,cmd_id=8,payload=link.peer_id+IDENTITY.wlm_device_id+b'\x05\x04\x04')
    attrs=bytearray(FORWARDED.attributes)
    attrs[:4]=(link.read.peer_hash if source_hash is None else source_hash).to_bytes(4,'little')
    attrs[9:13]=(IDENTITY.wlm_device_hash if destination_hash is None else destination_hash).to_bytes(4,'little')
    return replace(TEMPLATE,payload=inner.encode()+attrs).encode()


def test_preparation_waits_for_actual_send_and_busy_follow_preserves_current_modes():
    link,calls=ready_link()
    trigger=send(link,1.6,'wlm_link_switch_trigger')
    f=unpack_forwarded(trigger.packet.payload)
    assert int.from_bytes(f.attributes[9:13],'little')==link.read.peer_hash
    assert f.packet.payload==bytes.fromhex('0504040000000200020000')
    link.feed(incoming_prepare(link),1.7)
    pending=link.next_packet(1.7)
    assert pending.kind=='wlm_link_switch_prepare_ack' and not calls
    assert link.snapshot(1.7)['modes_raw']==[1,1,1,0,0]
    link.sent(pending.kind,1.7)
    assert len(calls)==1
    link.feed(follow(link,modes=(5,4,4),service=(2,1)),1.8)
    reply=send(link,1.8,'wlm_link_follow_reply')
    assert unpack_forwarded(reply.packet.payload).packet.payload[12:17]==b'\x01\x01\x01\x00\x00'
    assert link.snapshot(1.8)['follow_requests_deferred']==1
    assert not link.snapshot(1.8)['service_switch']['committed']


@pytest.mark.parametrize('change',['source_hash','destination_hash'])
def test_other_forwarding_identity_cannot_prepare_local_backend(change):
    link,calls=ready_link();send(link,1.6,'wlm_link_switch_trigger')
    link.feed(incoming_prepare(link,**{change:123}),1.7)
    assert not calls and link.service_switch.prepare_ack is None
    assert link.snapshot(1.7)['service_switch']['state']=='awaiting_prepare'


@pytest.mark.parametrize('change',['expired','closed','capability_masked'])
def test_stale_cancelled_or_masked_service_does_not_emit_trigger(change):
    link,calls=ready_link()
    if change=='expired':
        assert link.next_packet(3) is None
    elif change=='closed':
        link.close(1.6);assert link.next_packet(1.6) is None
    else:
        cap=link.read.capabilities[10]
        link.read.capabilities[10]=CapabilityVector(2,1<<5,cap.support)
        send(link,1.6,'wlm_link_presence')
    assert link.service_switch is None and not calls


def test_cancelled_prepared_switch_cannot_commit_or_send_completion():
    link,calls=ready_link();send(link,1.6,'wlm_link_switch_trigger')
    link.feed(incoming_prepare(link),1.7);send(link,1.7,'wlm_link_switch_prepare_ack')
    link.close(1.8)
    assert link.next_packet(1.8) is None and len(calls)==1
    assert link.snapshot(1.8)['service_switch']['state']=='cancelled'
    assert link.snapshot(1.8)['modes_raw']==[1,1,1,0,0]


def test_later_follow_revokes_current_service_indicator_without_rewriting_switch_history():
    link,calls=ready_link();send(link,1.6,'wlm_link_switch_trigger')
    prepare=DumlPacket.decode(incoming_prepare(link))
    forward=unpack_forwarded(prepare.payload)
    def frame(command,payload,sequence=0x1234,kind=0x40):
        return replace(prepare,payload=replace(forward.packet,cmd_id=command,payload=payload,
                       seq=sequence,cmd_type=kind).encode()+forward.attributes).encode()
    link.feed(prepare.encode(),1.7);send(link,1.7,'wlm_link_switch_prepare_ack')
    own=send(link,1.8,'wlm_link_switch_completion')
    link.feed(frame(6,IDENTITY.wlm_device_id+b'\0',own.sequence,0x80),1.9)
    link.feed(frame(6,link.peer_id+IDENTITY.wlm_device_id+bytes(5)),1.9)
    send(link,1.9,'wlm_link_switch_result_ack')
    assert link.snapshot(1.9)['wifi_service_configured'] and len(calls)==2
    link.feed(follow(link),1.95)
    state=link.snapshot(1.95)
    assert state['service_switch']['committed']
    assert not state['wifi_service_configured'] and state['modes_raw']==[1,1,1,0,0]
