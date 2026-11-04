"""Real loopback SWUDP/WLM service negotiation and socket-owner camera intents."""
from dataclasses import replace
import json
import socket
import threading
import time

import pytest

from neo2_control.camera import CameraChannel
from neo2_control.duml import DumlPacket
from neo2_control.duml_stream import DumlStreamDecoder
from neo2_control.session import run_session
from neo2_control.transport import describe_transport
from neo2_control.wlm_discovery import unpack_forwarded
from test_observation import until
from test_session import acceptance,status
from test_wlm_link import follow
from test_wlm_liveview import IDENTITY,OWN,PEER,BEACON,link_client,peer_frame


class CameraPeer:
    """Only synthetic localhost traffic; no BLE, real radio or flight packet."""
    def __init__(self,channel,*,hold_ack=False,withhold_beacons=False):
        self.channel=channel
        self.hold_ack=hold_ack
        self.withhold_beacons=withhold_beacons
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1',0));self.sock.settimeout(.02)
        self.stop=threading.Event();self.release_ack=threading.Event()
        self.errors=[];self.camera=[];self.modes=[];self.camera_wire=[];self.forwarding=[];self.allocated=[]
        self.handshake=[];self.pending_ack=None;self.last_beacon_at=None
        self.own_requests={}
        self.registered=self.followed=self.completed=False
        self.oracle=link_client()
        self.thread=threading.Thread(target=self.run,daemon=True)

    def camera_frame(self,request,*,kind=0x80,status_code=0,**changes):
        base=DumlPacket.decode(peer_frame(self.oracle,request.cmd_id,bytes([status_code]),
                                         sequence=request.seq,kind=kind))
        forwarded=unpack_forwarded(base.payload)
        fields=dict(sender=request.receiver,receiver=request.sender,cmd_set=request.cmd_set)
        fields.update(changes)
        inner=replace(forwarded.packet,**fields)
        return replace(base,payload=inner.encode()+forwarded.attributes).encode()

    def beacon_bytes(self):
        return BEACON

    def additional_packet(self,inner,attributes):
        return None

    def response_datagrams(self,raw,outgoing):
        return [status(self.session,anchor=self.anchor,ack=self.anchor,payload=outgoing)]

    def receive_idle(self):
        pass

    def observe_packet(self,inner):
        pass

    def record_allocation(self,inner):
        previous=self.own_requests.get(inner.seq)
        if previous is not None:
            # A completion retry is the same request, with a new outer
            # forwarding counter. Every other duplicate is a test failure.
            assert previous==inner and (inner.cmd_set,inner.cmd_id)==(0x51,6)
        else:
            self.own_requests[inner.seq]=inner
            self.allocated.append(inner.seq)

    def completion_reply(self,inner):
        return (peer_frame(self.oracle,6,OWN+b'\0',sequence=inner.seq,kind=0x80)
                +peer_frame(self.oracle,6,PEER+OWN+bytes(5),sequence=0x9234))

    def run(self):
        try:
            while not self.stop.is_set():
                try:raw,address=self.sock.recvfrom(65535)
                except socket.timeout:
                    self.receive_idle();continue
                self.peer_address=address
                if raw[6]==0:
                    self.session=int.from_bytes(raw[2:4],'little')
                    self.anchor=int.from_bytes(raw[8:10],'little')
                    self.sock.sendto(acceptance(self.session,flags=15),address)
                    continue
                decoded,_=describe_transport(raw)
                offset=decoded.get('qos_aggregate_offset')
                size=decoded.get('qos_aggregate_length',0)
                aggregate=b'' if offset is None else raw[offset:offset+size]
                outgoing=b'' if self.withhold_beacons and self.completed else self.beacon_bytes()
                for outer,_ in DumlStreamDecoder().feed(aggregate):
                    if (outer.cmd_set,outer.cmd_id)!=(0x51,1):continue
                    forwarded=unpack_forwarded(outer.payload)
                    inner,attrs=forwarded.packet,forwarded.attributes
                    self.observe_packet(inner)
                    assert (outer.sender,outer.receiver)==(0x3b,0xe9)
                    assert attrs[:4]==IDENTITY.wlm_device_hash.to_bytes(4,'little') and attrs[4]==2
                    counter=int.from_bytes(attrs[5:9],'little')
                    assert outer.seq==counter&0xffff
                    if (inner.cmd_set,inner.cmd_id)!=(0x51,0x13):self.forwarding.append(counter)
                    if inner.cmd_set!=0x51:
                        assert self.completed
                        additional=self.additional_packet(inner,attrs)
                        if additional is not None:
                            outgoing+=additional
                            continue
                        assert (inner.cmd_set,inner.cmd_id) in ((4,0x14),(2,1),(2,2),(2,0x10))
                        assert attrs[9:13]==self.oracle.read.peer_hash.to_bytes(4,'little')
                        self.camera_wire.append(inner)
                        self.record_allocation(inner)
                        if (inner.cmd_set,inner.cmd_id)==(2,0x10):
                            assert inner.payload in (b'\0',b'\1')
                            self.modes.append(inner)
                            outgoing+=self.camera_frame(inner)
                            body=bytes(4)+inner.payload+bytes(55)
                            push=DumlPacket(2,1,0x9334,0,2,0x80,b'')
                            outgoing+=self.camera_frame(push,kind=0,payload=body)
                            continue
                        self.camera.append(inner)
                        if (inner.cmd_set,inner.cmd_id)==(4,0x14):continue
                        if self.hold_ack:
                            # Invalid seq, counterpart, command, encrypted ACK,
                            # empty body and unbound routing hashes precede good ACK.
                            invalid=[self.camera_frame(inner,seq=inner.seq^1),
                                     self.camera_frame(inner,sender=inner.receiver^1),
                                     self.camera_frame(inner,receiver=inner.sender^1),
                                     self.camera_frame(inner,cmd_id=inner.cmd_id^1),
                                     self.camera_frame(inner,cmd_set=inner.cmd_set^1),
                                     self.camera_frame(inner,kind=0xc1),
                                     self.camera_frame(inner,payload=b'')]
                            bad=DumlPacket.decode(self.camera_frame(inner))
                            f=unpack_forwarded(bad.payload)
                            for location in (0,9):
                                a=bytearray(f.attributes);a[location]^=1
                                invalid.append(replace(bad,payload=f.packet.encode()+bytes(a)).encode())
                            outgoing+=b''.join(invalid)
                            self.pending_ack=self.camera_frame(inner,kind=0xc0)
                        else:
                            outgoing+=self.camera_frame(inner,kind=0x80 if len(self.camera)%2 else 0xc0)
                        continue
                    self.handshake.append((inner.cmd_id,inner.cmd_type))
                    if inner.cmd_id==0x13:
                        if not self.registered:outgoing+=follow(self.oracle,sequence=0x9050)
                        self.registered=True
                    elif inner.cmd_id==0x1c:
                        assert self.registered and inner.cmd_type==0xc0
                        self.followed=True
                    elif inner.cmd_id==0x41:
                        from test_wlm_real_ack import carrier
                        assert self.followed
                        self.record_allocation(inner)
                        outgoing+=carrier(sequence=inner.seq)
                    elif inner.cmd_id==2:
                        assert self.followed and inner.payload==bytes.fromhex('0504040000000200020000')
                        self.record_allocation(inner)
                        outgoing+=peer_frame(self.oracle,8,PEER+OWN+bytes([5,4,4]))
                    elif inner.cmd_id==8:
                        assert inner.cmd_type==0xc0 and inner.payload==PEER+b'\0'
                    elif inner.cmd_id==6 and inner.cmd_type==0x40:
                        self.record_allocation(inner)
                        outgoing+=self.completion_reply(inner)
                    elif inner.cmd_id==6 and inner.cmd_type==0xc0:
                        assert inner.payload==PEER+b'\0' and inner.seq==0x9234
                        self.completed=True
                    else:raise AssertionError('unexpected WLM command')
                if self.pending_ack is not None and self.release_ack.is_set():
                    outgoing+=self.pending_ack;self.pending_ack=None
                if outgoing.startswith(BEACON):self.last_beacon_at=time.monotonic()
                for response in self.response_datagrams(raw,outgoing):self.sock.sendto(response,address)
        except BaseException as error:self.errors.append(repr(error))

    def __enter__(self):
        self.thread.start();return self

    def __exit__(self,*args):
        self.stop.set();self.thread.join(timeout=1);self.sock.close()
        assert not self.thread.is_alive() and not self.errors,self.errors


def start_session(tmp_path,channel,peer,stop):
    identity=tmp_path/'identity.json'
    identity.write_text(json.dumps({'format':'neo2-control/wlm-identity/v1',
                                    'identification_id':IDENTITY.identification_id}))
    result=[];errors=[]
    def run():
        try:
            result.append(run_session(capture_path=tmp_path/'camera.jsonl',host='127.0.0.1',
                port=peer.sock.getsockname()[1],duration=8,heartbeat=True,
                wlm_identity_file=identity,wlm_wifi_liveview=True,
                camera_channel=channel,stop_event=stop).as_dict())
        except BaseException as error:errors.append(repr(error))
    worker=threading.Thread(target=run,daemon=True);worker.start()
    return worker,result,errors


def test_real_udp_service_then_camera_intents_use_actual_sends_shared_counter_and_both_ack_types(tmp_path):
    channel=CameraChannel();stop=threading.Event()
    with CameraPeer(channel) as peer:
        worker,result,errors=start_session(tmp_path,channel,peer,stop)
        try:
            until(lambda: channel.snapshot()['ready'],timeout=5)
            for action,parameters in [('pitch',{'degrees':-12.5}),('recenter',{}),('photo',{}),('record',{}),('stop_record',{})]:
                accepted=channel.submit(action,parameters)
                terminal='sent_unconfirmed' if action in ('pitch','recenter') else 'acknowledged'
                until(lambda: channel.snapshot()['last_command']['id']==accepted['command_id']
                      and channel.snapshot()['last_command']['state']==terminal)
                assert not channel.snapshot()['last_command']['execution_confirmed']
                assert channel.snapshot()['recording'] in (None,False)
        finally:
            stop.set();worker.join(timeout=3)
        assert not worker.is_alive() and not errors,errors
        assert len(peer.camera)==5
        assert [(x.sender,x.receiver,x.cmd_set,x.cmd_id,x.payload.hex()) for x in peer.camera]==[
            (2,4,4,0x14,'0000000083ff070a'),(2,4,4,0x14,'000000000000070a'),
            (2,1,2,1,'01'),(2,1,2,2,'01'),(2,1,2,2,'00')]
        assert [x.payload for x in peer.modes]==[b'\0',b'\1']
        assert [(x.cmd_set,x.cmd_id) for x in peer.camera_wire]==[
            (4,0x14),(4,0x14),(2,0x10),(2,1),(2,0x10),(2,2),(2,2)]
        assert peer.forwarding==list(range(1,1+len(peer.forwarding)))
        assert len(peer.allocated)==len(set(peer.allocated))
    state=result[0]['wlm_mode_follow']
    assert state['service_switch']['committed'] and state['camera']['sent_count']==7
    assert not state['camera']['ready'] and not state['camera']['execution_confirmed']
    assert state['camera']['recording'] is None
    assert result[0]['reason']=='requested'


def test_real_udp_invalid_acks_cannot_complete_or_replay_camera_command(tmp_path):
    channel=CameraChannel();stop=threading.Event()
    with CameraPeer(channel,hold_ack=True) as peer:
        worker,result,errors=start_session(tmp_path,channel,peer,stop)
        try:
            until(lambda: channel.snapshot()['ready'],timeout=5)
            channel.submit('stop_record',{})
            until(lambda: len(peer.camera)==1 and channel.snapshot()['ignored_frames']>=7)
            assert channel.snapshot()['last_command']['state']=='sent'
            assert channel.snapshot()['sent_count']==1
            peer.release_ack.set()
            until(lambda: channel.snapshot()['last_command']['state']=='acknowledged')
            assert channel.snapshot()['last_command']['status_code']==0
            assert len(peer.camera)==1
            peer.release_ack.clear()
            channel.submit('record',{})
            until(lambda: len(peer.camera)==2 and channel.snapshot()['last_command']['state']=='sent')
        finally:
            stop.set();worker.join(timeout=3)
        assert not worker.is_alive() and not errors,errors
        assert channel.snapshot()['last_command']['state']=='cancelled'
        assert channel.snapshot()['last_command']['reason']=='requested'
        assert channel.snapshot()['sent_count']==3
        assert not channel.snapshot()['execution_confirmed']


def test_real_udp_stale_peer_cancels_camera_channel_despite_transport_status_liveness(tmp_path):
    channel=CameraChannel();stop=threading.Event()
    with CameraPeer(channel,withhold_beacons=True,hold_ack=True) as peer:
        worker,result,errors=start_session(tmp_path,channel,peer,stop)
        try:
            until(lambda: channel.snapshot()['ready'],timeout=5)
            until(lambda: peer.last_beacon_at is not None and time.monotonic()-peer.last_beacon_at>=4.3,timeout=5)
            channel.submit('record',{})
            until(lambda: len(peer.camera)==1)
            until(lambda: not channel.snapshot()['ready'],timeout=5)
            assert channel.snapshot()['last_command']['state']=='cancelled'
            assert channel.snapshot()['last_command']['reason']=='peer_beacon_expired'
            with pytest.raises(RuntimeError,match='camera_not_ready'):channel.submit('record',{})
        finally:
            stop.set();worker.join(timeout=3)
        assert not worker.is_alive() and not errors,errors
        assert len(peer.camera)==1
    assert result[0]['wlm_mode_follow']['reason']=='peer_beacon_expired'
    assert result[0]['reason']=='requested'
