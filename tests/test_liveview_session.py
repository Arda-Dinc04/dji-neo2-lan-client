"""Direct keyframe requests through the owned session and actual localhost UDP."""
from dataclasses import replace
import json
from pathlib import Path
import socket
import struct
import threading
import time

import pytest

from neo2_control.camera import CameraChannel
from neo2_control.duml import DumlPacket,DumlSequence
from neo2_control.duml_stream import DumlStreamDecoder
from neo2_control.identity import WlmIdentity
from neo2_control.session import SwudpSession,run_session
from neo2_control.transport import ReliableTransport,_build_datagram,describe_transport
from neo2_control.wlm_discovery import WlmBeacon
from test_hevc import nal
from test_observation import until
from test_session import acceptance,status
from test_video_hevc_pipeline import hevc_source
from test_wlm_query import beacon
from test_wlm_query_session import TEMPLATE,FORWARDED


IDENTITY=WlmIdentity('synthetic-keyframe-client')


def wifi_beacon(*,mode=4,ssfn=0x22334455,foreign=False,kind=0):
    body=bytearray(beacon().payload)
    body[1]=kind;body[25]=mode;body[43:47]=ssfn.to_bytes(4,'little')
    if foreign:body[4]^=1
    return WlmBeacon(bytes(body))


def beacon_wire(value):
    return replace(TEMPLATE,payload=replace(FORWARDED.packet,payload=value.payload).encode()
                   +FORWARDED.attributes).encode()


def client(*,flags=15,needed=lambda:True,backend=lambda:True,heartbeat=False):
    value=SwudpSession(heartbeat=heartbeat,wlm_identity=IDENTITY,wlm_wifi_liveview=True,
        camera_channel=CameraChannel(),wifi_backend_ready=backend,video_keyframe_needed=needed,
        transport=ReliableTransport(session=0x1234,base_sequence=0x4000),
        duml_sequence=DumlSequence(current=0x8000))
    value.start(0);value.receive(acceptance(flags=flags),0)
    return value


def test_camera_beacon_grace_does_not_relax_ordinary_observation_liveness():
    assert client().wlm_link.maximum_age==5
    ordinary=SwudpSession(wlm_identity=IDENTITY,wlm_mode_follow=True)
    assert ordinary.wlm_link.maximum_age==2
    assert client().timing.remote_timeout==ordinary.timing.remote_timeout==2


def refresh(value,when,**options):
    value.receive(status(payload=beacon_wire(wifi_beacon(**options))),when)


def requests(value,when):
    return [x for x in value.tick(when) if x.application_kind=='video_keyframe_request']


def packet(datagram):
    d,_=describe_transport(datagram.raw)
    return DumlPacket.decode(datagram.raw[d['qos_aggregate_offset']:
        d['qos_aggregate_offset']+d['qos_aggregate_length']])


@pytest.mark.parametrize('options',[{'video_keyframe_needed':True},
    {'video_keyframe_needed':lambda:True},
    {'video_keyframe_needed':lambda:True,'wlm_identity':IDENTITY,'wlm_wifi_liveview':True,
     'wifi_backend_ready':lambda:True}])
def test_feature_rejects_incomplete_owner_configuration_before_effects(options):
    with pytest.raises(ValueError):SwudpSession(**options)


@pytest.mark.parametrize('blocked',['no_video','no_peer','peer_not_wifi','foreign_outer','not_uav',
                                   'backend','legacy','truthy_callback','revoked'])
def test_native_preconditions_cannot_be_replaced_by_transport_acceptance(blocked):
    value=client(flags=1 if blocked=='legacy' else 15,
                 needed=(lambda:1) if blocked=='truthy_callback' else lambda:True,
                 backend=(lambda:False) if blocked=='backend' else lambda:True)
    if blocked!='no_peer':
        raw=beacon_wire(wifi_beacon(mode=1 if blocked=='peer_not_wifi' else 4,
                                   kind=2 if blocked=='not_uav' else 0))
        if blocked=='foreign_outer':raw=replace(DumlPacket.decode(raw),sender=0x26).encode()
        value.receive(status(payload=raw),.1)
    if blocked!='no_video':value._deliver(2,nal(1),.11)
    if blocked=='revoked':value.wlm_link.camera_channel.revoke()
    assert requests(value,.12)==[]
    assert value.video_keyframes.sent_count==0 and value.video_keyframes.pending is None


@pytest.mark.parametrize('flags,tos',[(3,192),(15,128)])
def test_direct_requests_use_fresh_ssfn_actual_send_cadence_shared_sequence_and_no_wlm_counter(flags,tos):
    value=client(flags=flags,heartbeat=True)
    refresh(value,.1);value._deliver(2,nal(1),.11)
    outgoing=value.tick(.12);first,=[x for x in outgoing if x.application_kind=='video_keyframe_request']
    assert first.tos==tos and packet(first).payload==bytes.fromhex('00085544332201000000')
    counter=value.wlm_link.forwarding_sequence
    assert value.video_keyframes.sent_count==0 and requests(value,.3)==[]
    value.datagram_sent(first,.5)
    # No ACK response is consumed by this type00 action; an old frame cannot
    # shorten cooldown or claim that a reference picture was received.
    old=replace(packet(first),sender=0xe9,receiver=2,cmd_type=0x80,payload=b'\0')
    value.receive(status(payload=old.encode()),.6)
    intermediate=value.tick(1.49)
    assert not any(x.application_kind=='video_keyframe_request' for x in intermediate)
    refresh(value,1.5,ssfn=0xffffffff)
    all_output=value.tick(1.5)
    second,=[x for x in all_output if x.application_kind=='video_keyframe_request']
    assert packet(second).payload==bytes.fromhex('0008ffffffff01000000')
    heartbeat,=[x for x in outgoing+intermediate+all_output if x.application_kind=='heartbeat']
    assert len({first.duml_sequence,heartbeat.duml_sequence,second.duml_sequence})==3
    assert value.wlm_link.forwarding_sequence==counter
    value.datagram_sent(second,1.7)
    refresh(value,2.7)
    third,=requests(value,2.7);value.datagram_sent(third,2.7)
    for when in (3.7,4.7,5.7):
        refresh(value,when)
        assert requests(value,when)==[]
    state=value.video_keyframes.snapshot(5.7)
    assert state['sent_count']==3 and state['state']=='limit_reached'
    assert value.wlm_link.forwarding_sequence==counter
    assert not state['refresh_confirmed'] and not value.snapshot(5.7)['application_ready']


def test_foreign_peer_cannot_refresh_request_eligibility_and_pending_is_cancelled():
    value=client();refresh(value,.1);value._deliver(2,nal(1),.11)
    first,=requests(value,.12);value.datagram_sent(first,.12)
    for when in (1.9,3.8,4.9):refresh(value,when,foreign=True)
    assert requests(value,5.11)==[]
    assert value.wlm_link.reason=='peer_beacon_expired'
    assert value.video_keyframes.sent_count==1 and value.video_keyframes.pending is None


def test_no_longer_needed_and_callback_failure_cancel_only_requests():
    needed=[True]
    value=client(needed=lambda:needed[0]);refresh(value,.1);value._deliver(2,nal(1),.11)
    first,=requests(value,.12)
    needed[0]=False
    assert not value.datagram_send_allowed(first,.2)
    assert requests(value,.2)==[] and value.video_keyframes.pending is None
    assert value.video_keyframes.sent_count==0 and value.state=='open'
    value.close(.3)
    assert value.video_keyframes.closed
    def broken():raise RuntimeError('synthetic local consumer failure')
    other=client(needed=broken);refresh(other,.1);other._deliver(2,nal(1),.11)
    assert requests(other,.12)==[] and other.state=='open'
    assert other.video_keyframes.closed and other.video_keyframes.reason=='callback_failed'


class KeyframePeer:
    """Synthetic warm stream: refresh command releases real generated HEVC."""
    def __init__(self,source):
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1',0));self.socket.settimeout(.025)
        self.source=source;self.orphan=nal(1,b'orphan-picture')+nal(35,b'\x30')
        self.stop=threading.Event();self.received=[];self.errors=[];self.released=False;self.release_at=None
        self.thread=threading.Thread(target=self.run,daemon=True)

    def video(self,payload,address):
        chunks=[payload[i:i+1452] for i in range(0,len(payload),1452)]
        start=(self.anchor+self.blocks*8)&0xffff;end=(self.anchor+(self.blocks+len(chunks))*8)&0xffff
        for chunk in chunks:
            self.blocks+=1
            body=struct.pack('<HHHHI',start,end,0,0,0x101)+chunk
            self.socket.sendto(_build_datagram(self.session,2,body,
                send_sequence=(self.anchor+self.blocks*8)&0xffff),address)

    def run(self):
        try:
            while not self.stop.is_set():
                try:raw,address=self.socket.recvfrom(65535)
                except socket.timeout:continue
                if raw[6]==0:
                    self.session=int.from_bytes(raw[2:4],'little')
                    self.anchor=int.from_bytes(raw[8:10],'little');self.frontier=self.anchor;self.blocks=0
                    self.socket.sendto(acceptance(self.session,flags=15),address)
                    self.socket.sendto(status(self.session,anchor=self.anchor,
                        ack=self.anchor,payload=beacon_wire(wifi_beacon())),address)
                    self.video(self.orphan,address)
                    continue
                d,_=describe_transport(raw)
                self.frontier=d.get('video_window_start',self.frontier)
                offset=d.get('qos_aggregate_offset');length=d.get('qos_aggregate_length',0)
                for frame,_ in DumlStreamDecoder().feed(b'' if offset is None else raw[offset:offset+length]):
                    if (frame.cmd_set,frame.cmd_id)!=(0x18,0x47):continue
                    self.received.append(frame)
                    assert (frame.sender,frame.receiver,frame.cmd_type)==(2,0xe9,0)
                    assert frame.payload==bytes.fromhex('00085544332201000000')
                    if self.release_at is None:self.release_at=time.monotonic()+.15
                if not self.released and self.release_at is not None and time.monotonic()>=self.release_at:
                    self.video(self.source,address);self.released=True
                reply=bytearray(status(self.session,anchor=self.anchor,ack=self.anchor,
                    payload=beacon_wire(wifi_beacon())))
                for offset in (8,10):reply[offset:offset+2]=self.frontier.to_bytes(2,'little')
                self.socket.sendto(reply,address)
        except BaseException as error:self.errors.append(repr(error))

    def __enter__(self):self.thread.start();return self
    def __exit__(self,*args):
        self.stop.set();self.thread.join(timeout=1);self.socket.close()
        assert not self.thread.is_alive() and not self.errors,self.errors


def test_real_udp_hevc_consumer_requests_current_stream_bootstrap_then_stops_requests(tmp_path,hevc_source):
    identity=tmp_path/'identity.json'
    identity.write_text(json.dumps({'format':'neo2-control/wlm-identity/v1',
                                    'identification_id':IDENTITY.identification_id}))
    stop=threading.Event();snapshots=[]
    def observe(state):
        snapshots.append(state)
        if (state['video']['hevc_bootstrap']['ready'] and state['video']['keyframe_requests']['needed'] is False
                and state['video_bytes_delivered']==len(peer.source)+len(peer.orphan)):
            stop.set()
    with KeyframePeer(hevc_source) as peer:
        result=run_session(capture_path=tmp_path/'session.jsonl',host='127.0.0.1',
            port=peer.socket.getsockname()[1],duration=4,heartbeat=True,
            wlm_identity_file=identity,wlm_wifi_liveview=True,camera_channel=CameraChannel(),
            hls_directory=tmp_path/'hls',video_codec='hevc',video_nominal_fps=8,preview_jpeg=True,
            video_path=tmp_path/'video.hevc',snapshot_sink=observe,stop_event=stop)
        assert peer.released and len(peer.received)==1
        expected=peer.orphan+peer.source
    assert result.state=='closed' and result.reason=='requested'
    assert result.video_bytes_delivered==len(expected) and (tmp_path/'video.hevc').read_bytes()==expected
    assert result.video['error'] is None and result.video['frames_encoded']==16
    assert result.video['preview_available'] and result.video['cleanup_verified']
    assert result.video['hevc_bootstrap']['ready']
    requests_state=result.video['keyframe_requests']
    assert requests_state['sent_count']==1 and requests_state['needed'] is False
    assert not requests_state['refresh_confirmed'] and requests_state['state']=='closed'
    assert any(s['video']['state']=='waiting_for_codec_config' for s in snapshots)
    assert not result.application_ready
