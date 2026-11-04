"""Native-compatible mode follow, persistent presence and actual UDP ownership."""
from dataclasses import replace
import json
from pathlib import Path
import socket
import threading

import pytest

from neo2_control.duml import DumlPacket, DumlSequence
from neo2_control.duml_stream import DumlStreamDecoder
from neo2_control.identity import WlmIdentity
from neo2_control.session import SwudpSession, run_session
from neo2_control.transport import describe_transport
from neo2_control.wlm_discovery import unpack_forwarded
from neo2_control.wlm_link import WlmWifiLink
from test_session import acceptance, status
from test_wlm_query import beacon
from test_wlm_query_session import TEMPLATE, FORWARDED, BEACON
from test_wlm_real_ack import carrier

IDENTITY=WlmIdentity('synthetic-follow-client')


def client():
    link=WlmWifiLink(IDENTITY,DumlSequence(current=0x8000).next)
    link.on_beacon(beacon(),0)
    return link


def follow(link, *, sequence=0, modes=(1,1,1), service=(0,0), version=1, hash_value=None):
    payload=bytes([version,0,0])+(link.read.peer_hash if hash_value is None else hash_value).to_bytes(4,'little')
    payload+=bytes([0,*modes,*service])+bytes(6)
    inner=replace(FORWARDED.packet,seq=sequence,cmd_id=0x1c,cmd_type=0x40,payload=payload)
    return replace(TEMPLATE,payload=inner.encode()+FORWARDED.attributes).encode()


def send(link,when,kind):
    output=link.next_packet(when)
    assert output.kind==kind
    assert link.next_packet(when) is None
    link.sent(kind,when)
    return output


def test_follow_applies_modes_updates_beacons_and_shares_counter_with_one_query():
    link=client()
    first=send(link,0,'wlm_link_presence')
    assert unpack_forwarded(first.packet.payload).packet.payload[24:27]==bytes([4,4,4])
    link.feed(follow(link,sequence=0x1234),.1)
    reply=send(link,.1,'wlm_link_follow_reply')
    frame=unpack_forwarded(reply.packet.payload)
    assert frame.packet.seq==0x1234 and frame.packet.payload[12:17]==bytes([1,1,1,0,0])
    assert frame.attributes[14]==0x82
    for when in (.5,1):
        link.on_beacon(beacon(),when)
        output=send(link,when,'wlm_link_presence')
        inner=unpack_forwarded(output.packet.payload)
        assert inner.packet.payload[24:27]==bytes([1,1,1])
        assert inner.packet.payload[33:39]==bytes([0,0,1,0,0,1])
        assert inner.attributes[5:9]==bytes.fromhex('02000000')
    output=send(link,1.5,'wlm_link_capability_query')
    assert output.packet.seq==2 and link.forwarding_sequence==3
    link.feed(carrier(sequence=link.read.sequence),1.6)
    result=link.snapshot(1.6)
    assert result['capability_read']['state']=='complete'
    assert result['capability_read']['capabilities']['10']['support']==0x26d
    assert result['state']=='capabilities_received' and not result['application_ready']
    assert not result['controlled_wifi_route_available']
    # Presence continues after the three-announcement query experiment.
    assert send(link,1.6,'wlm_link_presence').packet.seq==3
    assert not link.snapshot(1.6)['implemented_backends']['SDR']


@pytest.mark.parametrize('change',['source','destination','category','hash','version','length','outer_crc'])
def test_untrusted_or_malformed_follow_cannot_mutate_or_queue_success(change):
    link=client();raw=follow(link);outer=DumlPacket.decode(raw);f=unpack_forwarded(outer.payload)
    inner=f.packet;attrs=f.attributes
    if change=='source':inner=replace(inner,sender=0xe8)
    elif change=='destination':inner=replace(inner,receiver=2)
    elif change=='category':attrs=attrs[:4]+b'\x01'+attrs[5:]
    elif change=='hash':inner=replace(inner,payload=inner.payload[:3]+bytes(4)+inner.payload[7:])
    elif change=='version':inner=replace(inner,payload=b'\x02'+inner.payload[1:])
    elif change=='length':inner=replace(inner,payload=inner.payload[:-1])
    raw=replace(outer,payload=inner.encode()+attrs).encode()
    if change=='outer_crc':raw=raw[:-1]+bytes([raw[-1]^1])
    link.feed(raw,.1)
    assert link.snapshot(.1)['follow_requests_applied']==0
    assert link.follow_reply is None
    assert link.snapshot(.1)['modes_raw']==[4,4,4,0,0]


def test_latest_follow_queue_is_bounded_rate_limited_and_cancellation_clears_effects():
    link=client()
    for seq in range(20):link.feed(follow(link,sequence=seq),.1)
    assert link.follow_coalesced==19
    packet=send(link,.1,'wlm_link_follow_reply')
    assert packet.sequence==19
    link.feed(follow(link,sequence=20),.11)
    assert send(link,.11,'wlm_link_presence')
    assert link.next_packet(.2) is None
    packet=send(link,.301,'wlm_link_follow_reply')
    assert packet.sequence==20
    link.close(.4,'requested')
    assert link.next_packet(.5) is None
    assert link.snapshot(.5)['capability_read']['state']=='cancelled'
    assert link.snapshot(.5)['follow_replies_sent']==2


def test_only_current_same_peer_beacons_maintain_link_and_no_packet_send_is_inferred():
    link=client();output=link.next_packet(0)
    assert output is not None and link.snapshot(0)['presence_datagrams_sent']==0
    foreign=bytearray(beacon().payload);foreign[4]^=1
    link.on_beacon(type(beacon())(bytes(foreign)),1.5)
    assert link.next_packet(2) is None
    assert link.snapshot(2)['reason']=='peer_beacon_expired'
    with pytest.raises(RuntimeError):link.sent(output.kind,2)


def test_response_timeout_is_reported_while_presence_and_peer_remain_current():
    link=client()
    for when in (0,.5,1):
        link.on_beacon(beacon(),when)
        send(link,when,'wlm_link_presence')
    send(link,1.5,'wlm_link_capability_query')
    for when in (2,2.5,3,3.5):
        link.on_beacon(beacon(),when)
        send(link,when,'wlm_link_presence')
    state=link.snapshot(3.5)
    assert state['state']=='capability_read_failed'
    assert state['capability_read']['state']=='timed_out'
    assert state['wifi_tunnel_active'] and not state['application_ready']
    link.feed(carrier(sequence=link.read.sequence),3.6)
    assert link.snapshot(3.6)['capability_read']['capabilities'] is None


@pytest.mark.parametrize('options',[{'wlm_mode_follow':True},{'wlm_mode_follow':1},
    {'wlm_mode_follow':True,'wlm_capability_query':True}])
def test_bad_opt_in_fails_before_files_or_network(tmp_path,options):
    path=tmp_path/'capture.jsonl'
    with pytest.raises(ValueError):run_session(capture_path=path,**options)
    with pytest.raises(ValueError):SwudpSession(**options)
    assert not path.exists()


@pytest.mark.parametrize('via_cli',[False,True])
def test_actual_udp_follow_reply_and_ongoing_presence_open_correlated_capability_read(tmp_path,capsys,via_cli):
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1',0));sock.settimeout(.02)
    stop=threading.Event();errors=[];seen=[];registered=False;followed=False
    oracle=client()
    request=follow(oracle)
    def peer():
        nonlocal registered,followed
        try:
            while not stop.is_set():
                try:raw,address=sock.recvfrom(65535)
                except socket.timeout:continue
                if raw[6]==0:
                    session=int.from_bytes(raw[2:4],'little');anchor=int.from_bytes(raw[8:10],'little')
                    sock.sendto(acceptance(session,flags=15),address);continue
                decoded,_=describe_transport(raw)
                offset,size=decoded['qos_aggregate_offset'],decoded['qos_aggregate_length']
                outgoing=BEACON
                for packet,_ in DumlStreamDecoder().feed(raw[offset:offset+size]):
                    if (packet.cmd_set,packet.cmd_id)!=(0x51,1):continue
                    forwarded=unpack_forwarded(packet.payload);inner=forwarded.packet
                    seen.append((inner,packet.seq))
                    if inner.cmd_id==0x13:
                        registered=True
                        if followed:assert inner.payload[24:27]==bytes([1,1,1])
                        else:outgoing+=request
                    elif inner.cmd_id==0x1c:
                        assert registered and inner.payload[8:12]==bytes(4)
                        assert inner.payload[12:17]==bytes([1,1,1,0,0])
                        followed=True
                    elif inner.cmd_id==0x41:
                        assert registered and followed
                        assert packet.seq==2
                        outgoing=carrier(sequence=inner.seq)
                    else:raise AssertionError('unexpected application command')
                sock.sendto(status(session,anchor=anchor,ack=anchor,payload=outgoing),address)
        except BaseException as error:errors.append(repr(error))
    worker=threading.Thread(target=peer);worker.start()
    identity=tmp_path/'identity.json'
    identity.write_text(json.dumps({'format':'neo2-control/wlm-identity/v1','identification_id':IDENTITY.identification_id}))
    try:
        if via_cli:
            from neo2_control.cli import main
            assert main(['session','--capture',str(tmp_path/'follow.jsonl'),'--host','127.0.0.1',
                '--port',str(sock.getsockname()[1]),'--duration','2.3',
                '--wlm-identity-file',str(identity),'--wlm-mode-follow'])==0
            result=json.loads(capsys.readouterr().out)
        else:
            result=run_session(capture_path=tmp_path/'follow.jsonl',host='127.0.0.1',port=sock.getsockname()[1],
                duration=2.3,wlm_identity_file=identity,wlm_mode_follow=True).as_dict()
    finally:
        stop.set();worker.join(timeout=1);sock.close()
    assert not worker.is_alive() and not errors,errors
    assert result['reason']=='duration_elapsed' and result['transport_accepted']
    state=result['wlm_mode_follow']
    assert state['capability_read']['state']=='complete'
    assert state['presence_datagrams_sent']>=4 and state['follow_replies_sent']==1
    assert len([p for p,_ in seen if p.cmd_id==0x41])==1
    assert not state['application_ready'] and not result['application_ready']
    assert not state['wifi_tunnel_active'] and not state['controlled_wifi_route_available']
