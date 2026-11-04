"""Opt-in WLM read scheduling, socket ownership and independent completion."""
from dataclasses import replace
import json
from pathlib import Path
import socket
import threading

import pytest

from neo2_control.duml import DumlPacket,DumlSequence
from neo2_control.duml_stream import DumlStreamDecoder
from neo2_control.identity import WlmIdentity
from neo2_control.session import SwudpSession,run_session
from neo2_control.transport import ReliableTransport,describe_transport
from neo2_control.wlm_discovery import unpack_forwarded
from test_session import acceptance,status
from test_wlm_query import beacon,response

TEMPLATE=DumlPacket.decode(bytes.fromhex(json.loads((Path(__file__).parent/
    'fixtures/wlm-discovery-native.json').read_text())['beacons'][0]['carrier_hex']))
FORWARDED=unpack_forwarded(TEMPLATE.payload)
BEACON=replace(TEMPLATE,payload=replace(FORWARDED.packet,payload=beacon().payload).encode()
    +FORWARDED.attributes).encode()
IDENTITY=WlmIdentity('synthetic-query-session')


def client(*,enabled=True,flags=15):
    peer=SwudpSession(wlm_identity=IDENTITY,wlm_capability_query=enabled,
        transport=ReliableTransport(session=0x1234,base_sequence=0x4000),
        duml_sequence=DumlSequence(current=0x8000))
    peer.start(0);peer.receive(acceptance(flags=flags),0)
    return peer


def reply(sequence):
    outer=response();forwarded=unpack_forwarded(outer.payload)
    return replace(outer,payload=replace(forwarded.packet,seq=sequence).encode()+forwarded.attributes).encode()


def announce(peer,start=.1):
    announcements=[]
    for when in (start,start+.5,start+1):
        peer.receive(status(payload=BEACON),when)
        output=peer.tick(when)
        assert not any(p.application_kind=='wlm_capability_query' for p in output)
        packet,=[p for p in output if p.application_kind=='wlm_presence']
        assert packet.tos==128
        peer.datagram_sent(packet,when)
        announcements.append(packet)
    return announcements


@pytest.mark.parametrize('flags',[3,15])
def test_query_schedules_once_and_deadline_starts_at_socket_completion(flags):
    peer=client(flags=flags)
    announcements=announce(peer)
    assert len(announcements)==3
    assert not any(p.application_kind=='wlm_capability_query' for p in peer.tick(1.59))
    output=peer.tick(1.6)
    query,= [p for p in output if p.application_kind=='wlm_capability_query']
    assert query.tos==128
    assert peer.snapshot(1.6)['wlm_capability_read']['state']=='prepared'
    assert not peer.snapshot(1.6)['wlm_capability_read']['datagram_sent']
    assert not any(p.application_kind=='wlm_capability_query' for p in peer.tick(1.7))
    peer.receive(status(),2)
    peer.datagram_sent(query,2)
    peer.receive(status(payload=reply(query.duml_sequence)),2.1)
    read=peer.snapshot(2.2)['wlm_capability_read']
    assert read['state']=='complete' and read['datagram_sent']
    assert not read['application_ready'] and peer.sender.waiting_for_ack is None
    assert read['capabilities']['6']['support']==0x1001f
    assert read['capabilities']['10']['support']==0x47d
    assert read['presence']['datagrams_sent']==3
    assert not read['presence']['peer_registration_confirmed']
    peer.close(2.3)
    assert peer.snapshot(2.3)['wlm_capability_read']['state']=='complete'


def test_read_stays_off_by_default_and_unsupported_direct_profile_never_sends_it():
    for enabled,flags,expected in ((False,15,None),(True,1,'unsupported_transport')):
        peer=client(enabled=enabled,flags=flags)
        peer.receive(status(payload=BEACON),.1)
        assert not any(p.application_kind in ('wlm_capability_query','wlm_presence') for p in peer.tick(.1))
        read=peer.snapshot(.1)['wlm_capability_read']
        if enabled:assert read['state']==expected
        else:assert read=={'enabled':False}


def test_corrupted_carrier_cannot_start_query_and_cancelled_read_cannot_complete_later():
    peer=client()
    corrupted=BEACON[:-1]+bytes([BEACON[-1]^1])
    peer.receive(status(payload=corrupted),.1)
    assert peer.snapshot(.1)['wlm_capability_read']['state']=='awaiting_current_beacon'
    announce(peer,.2)
    query,=[p for p in peer.tick(1.7) if p.application_kind=='wlm_capability_query']
    peer.datagram_sent(query,1.7);peer.close(1.8)
    peer.receive(status(payload=reply(query.duml_sequence)),1.9)
    read=peer.snapshot(1.9)['wlm_capability_read']
    assert read['state']=='cancelled' and read['cancel_reason']=='requested'
    assert read['capabilities'] is None


@pytest.mark.parametrize('change',['expired','downgraded','ability_disabled','cancelled'])
def test_presence_requires_current_same_peer_v5_and_stops_on_cancellation(change):
    peer=client()
    peer.receive(status(payload=BEACON),.1)
    packet,=[p for p in peer.tick(.1) if p.application_kind=='wlm_presence']
    peer.datagram_sent(packet,.1)
    if change=='cancelled':peer.close(.2)
    elif change in ('downgraded','ability_disabled'):
        payload=bytearray(beacon().payload)
        if change=='downgraded':payload[0]=4
        else:payload[51]&=~0x40
        changed=replace(TEMPLATE,payload=replace(FORWARDED.packet,payload=bytes(payload)).encode()
            +FORWARDED.attributes).encode()
        peer.receive(status(payload=changed),.2)
    else:
        # Unrelated traffic maintains SWUDP, but cannot refresh the beacon.
        peer.receive(status(),2)
    assert not any(p.application_kind in ('wlm_presence','wlm_capability_query')
                   for p in peer.tick(2.2))
    read=peer.snapshot(2.2)['wlm_capability_read']
    assert read['state']=='cancelled' and read['presence']['datagrams_sent']==1
    assert not read['datagram_sent']


def test_unsent_presence_is_not_rescheduled_and_cannot_open_query():
    peer=client()
    peer.receive(status(payload=BEACON),.1)
    assert len([p for p in peer.tick(.1) if p.application_kind=='wlm_presence'])==1
    for when in (.6,1.1,1.6):
        peer.receive(status(payload=BEACON),when)
        assert not any(p.application_kind in ('wlm_presence','wlm_capability_query') for p in peer.tick(when))
    read=peer.snapshot(1.6)['wlm_capability_read']
    assert read['presence']['datagrams_sent']==0 and read['state']=='prepared'


@pytest.mark.parametrize('option',[True,'yes',1])
def test_missing_identity_or_wrong_option_fails_before_file_or_socket(tmp_path,option):
    path=tmp_path/'capture.jsonl'
    with pytest.raises(ValueError):run_session(capture_path=path,wlm_capability_query=option)
    assert not path.exists()
    with pytest.raises(ValueError):SwudpSession(wlm_capability_query=option)


@pytest.mark.parametrize('respond',[False,True])
def test_actual_udp_query_receives_correlated_native_fixture_or_expires(tmp_path,respond):
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1',0));sock.settimeout(.025)
    stop=threading.Event();errors=[];queries=[];presences=[]
    def drone():
        session=anchor=None
        try:
            while not stop.is_set():
                try:raw,address=sock.recvfrom(65535)
                except socket.timeout:continue
                if raw[6]==0:
                    session=int.from_bytes(raw[2:4],'little');anchor=int.from_bytes(raw[8:10],'little')
                    sock.sendto(acceptance(session,flags=15),address)
                    continue
                decoded,_=describe_transport(raw);outgoing=BEACON
                offset,size=decoded['qos_aggregate_offset'],decoded['qos_aggregate_length']
                for packet,_ in DumlStreamDecoder().feed(raw[offset:offset+size]):
                    if (packet.cmd_set,packet.cmd_id)==(0x51,1):
                        forwarded=unpack_forwarded(packet.payload);inner=forwarded.packet
                        if inner.cmd_id==0x13:
                            assert inner.payload[4:24].rstrip(b'\0')==IDENTITY.device_sn.encode()
                            assert inner.payload[51]==4
                            presences.append((inner,forwarded.attributes))
                        else:
                            assert inner.cmd_id==0x41
                            assert inner.payload[0]==0x20 and inner.payload[9:]==b'\x06\0\x0a\0'
                            queries.append(inner)
                            # Native admission requires a registered sender hash.
                            registered=presences and presences[-1][1][:4]==inner.payload[3:7]
                            if respond and registered:outgoing=reply(inner.seq)
                sock.sendto(status(session,anchor=anchor,ack=anchor,payload=outgoing),address)
        except BaseException as error:errors.append(error)
    thread=threading.Thread(target=drone);thread.start()
    identity=tmp_path/'identity.json'
    identity.write_text(json.dumps({'format':'neo2-control/wlm-identity/v1',
        'identification_id':IDENTITY.identification_id}))
    snapshots=[]
    try:
        result=run_session(capture_path=tmp_path/'query.jsonl',host='127.0.0.1',
            port=sock.getsockname()[1],duration=1.9 if respond else 3.8,
            wlm_identity_file=identity,wlm_capability_query=True,snapshot_sink=snapshots.append)
    finally:
        stop.set();thread.join(timeout=1);sock.close()
    assert not thread.is_alive() and not errors
    assert result.reason=='duration_elapsed' and result.transport_accepted
    assert len(queries)==1 and result.wlm_capability_read['datagram_sent']
    assert len(presences)==3
    assert [int.from_bytes(p.payload[39:43],'little') for p,_ in presences]==[0,1,2]
    assert result.wlm_capability_read['state']==('complete' if respond else 'timed_out')
    assert snapshots[-1]['wlm_capability_read']==result.wlm_capability_read or (
        snapshots[-1]['wlm_capability_read']['state']==result.wlm_capability_read['state'])
    assert not result.application_ready
    journal=Path(result.journal_path).read_text()
    assert 'wlm_capability_read_state' in journal and IDENTITY.device_sn not in journal
