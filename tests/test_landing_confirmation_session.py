"""Normal landing continuation through real localhost UDP, never RF/BLE.

The independent peer encodes03/53 and FC mode12 on the wire. Observations come
from the production session/decoders; no observation-provider callback is used.
"""
from dataclasses import replace
import json
import struct
import threading
import time

import pytest

from neo2_control.control_transport import FlightChannelBackend
from neo2_control.controller import ControlSupervisor, DecisionJournal, IntentRejected
from neo2_control.duml import DumlPacket
from neo2_control.duml_stream import DumlStreamDecoder
from neo2_control.flight_channel import FlightChannel
from neo2_control.session import run_session
from neo2_control.wlm_discovery import unpack_forwarded
from test_control_transport_session import ControllerPeer
from test_flight_channel_session import FlightPeer
from test_wlm_liveview import IDENTITY


class LandingPeer(ControllerPeer):
    """LAND pauses airborne until explicit1e; even its ACK is not touchdown."""
    def __init__(self, channel, *, direct=False, without_gps=False):
        super().__init__(channel,observed_action_replies=True,direct_motion=direct)
        self.direct=direct
        self.without_gps=without_gps
        self.waiting_landing=False
        self.confirmed=False
        self.allow_ground=threading.Event()
        self.confirmations=[]

    def _landing_flag(self):
        needed=self.waiting_landing and not self.confirmed
        packet=DumlPacket(3,0xe if self.direct and self.phase!='ground' else 2,
                          0x9543,0,3,0x53,(0x33 if needed else 0x13).to_bytes(4,'little'))
        if self.direct and self.phase!='ground':return packet.encode()
        request=DumlPacket(2,3,packet.seq,0,3,0x53,b'')
        outer=DumlPacket.decode(self.camera_frame(request,kind=0,payload=packet.payload))
        forwarded=unpack_forwarded(outer.payload)
        attrs=bytearray(forwarded.attributes);attrs[9:13]=b'\xff'*4
        return replace(outer,payload=packet.encode()+bytes(attrs)).encode()

    def beacon_bytes(self):
        if self.allow_ground.is_set():
            self.phase='ground';self.complete_at=None;self.waiting_landing=False
        payload=super().beacon_bytes()
        output=[];has_motion=False
        for outer,_ in DumlStreamDecoder().feed(payload):
            wrapped=(outer.cmd_set,outer.cmd_id)==(0x51,1)
            forwarded=unpack_forwarded(outer.payload) if wrapped else None
            inner=forwarded.packet if wrapped else outer
            if (inner.cmd_set,inner.cmd_id)==(3,0x43):
                has_motion=True
                body=bytearray(inner.payload)
                # AUTO_LANDING is the mode byte, distinct from activity bits.
                body[30]=12 if self.waiting_landing else 0
                if self.waiting_landing:
                    struct.pack_into('<I',body,32,0x388000|12)  # flying + motors
                    struct.pack_into('<h',body,16,10)
                if self.without_gps:
                    body[:16]=bytes(16)
                    flags=struct.unpack_from('<I',body,32)[0]
                    struct.pack_into('<I',body,32,flags&~0x388000)
                    body[36]=0
                inner=replace(inner,payload=bytes(body))
                outer=replace(outer,payload=inner.encode()+forwarded.attributes) if wrapped else inner
            output.append(outer.encode())
        if has_motion:output.append(self._landing_flag())
        return b''.join(output)

    def additional_packet(self,inner,attributes):
        if (inner.cmd_set,inner.cmd_id)==(3,0x2a):
            if inner.payload==b'\2':
                self.waiting_landing=True;self.phase='air';self.complete_at=None
                return FlightPeer.additional_packet(self,inner,attributes)
            if inner.payload==b'\x1e':
                assert self.waiting_landing and not self.confirmed
                self.flight_frames.append(inner);self.record_allocation(inner)
                self.confirmations.append(inner)
                self.confirmed=True
                return DumlPacket(3,2,inner.seq,0x80,3,0x2a,b'\0').encode()
            assert inner.payload==b'\1', 'unexpected or forced flight action'
        return super().additional_packet(inner,attributes)


@pytest.mark.parametrize('direct',[False,True],ids=['wrapped-pushes','direct-on-motors'])
@pytest.mark.parametrize('vertical',[False,True],ids=['ordinary','vertical-without-gps'])
def test_wire_landing_request_requires_explicit_confirmation_and_physical_ground(tmp_path,direct,vertical):
    channel=FlightChannel(vertical_trial=vertical);channel.require_loopback()
    backend=FlightChannelBackend(channel,synthetic=False)
    journal=DecisionJournal(tmp_path/'decisions.jsonl')
    controller=ControlSupervisor(journal,mode='flight',backend=backend,vertical_trial=vertical)
    identity=tmp_path/'identity.json'
    identity.write_text(json.dumps({'format':'neo2-control/wlm-identity/v1',
                                    'identification_id':IDENTITY.identification_id}))
    stop=threading.Event();results=[];errors=[];lease=None;last_keepalive=0.

    def drive(predicate, *, timeout=5):
        nonlocal last_keepalive
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            controller.tick()
            now=time.monotonic()
            if lease and controller.state in ('armed','taking_off','flying') and now-last_keepalive>=.1:
                try:controller.handle('keepalive',{'lease':lease})
                except IntentRejected:
                    if controller.state not in ('landing','fault'):raise
                last_keepalive=now
            if predicate():return
            time.sleep(.01)
        pytest.fail(f'landing UDP fixture did not reach state: {controller.status()}')

    with LandingPeer(channel,direct=direct,without_gps=vertical) as peer:
        def run():
            try:
                results.append(run_session(capture_path=tmp_path/'flight.jsonl',host='127.0.0.1',
                    port=peer.sock.getsockname()[1],duration=15,heartbeat=True,
                    flight_channel=channel,wlm_identity_file=identity,wlm_wifi_liveview=True,
                    stop_event=stop).as_dict())
            except BaseException as error:errors.append(repr(error))
        worker=threading.Thread(target=run);worker.start()
        try:
            controller.handle('open')
            drive(lambda:backend.observation() is not None and backend.observation().control_ready is True)
            ground=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
            controller.handle('confirm_ground',{'confirmation_token':ground})
            drive(lambda:controller.state=='ready')
            if vertical:
                assert not backend.observation().position_valid
                assert backend.observation().distance_m is None
            arm=controller.handle('prepare',{'operation':'arm'})['confirmation_token']
            lease=controller.handle('arm',{'confirmation_token':arm})['lease']
            takeoff=controller.handle('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
            controller.handle('takeoff',{'confirmation_token':takeoff,'lease':lease})
            drive(lambda:controller.state=='flying')
            controller.handle('land')
            drive(lambda:controller.status()['landing_confirmation']['available'])
            assert controller.land_acknowledged and not peer.confirmations
            # Repeated statuses/ticks and fresh03/53 true pushes emit no1e.
            limit=time.monotonic()+.15
            while time.monotonic()<limit:
                controller.tick();controller.status();time.sleep(.01)
            assert not peer.confirmations
            token=controller.handle('prepare',{'operation':'landing'})['confirmation_token']
            assert not peer.confirmations
            controller.handle('confirm_landing',{'confirmation_token':token})
            drive(lambda:controller.pending is None and controller.action_acknowledged)
            assert len(peer.confirmations)==1
            assert controller.flight_started and not controller.landing_confirmed
            assert backend.observation().flying is True and backend.observation().motors_on is True
            assert controller.lease is controller.control is None
            with pytest.raises(IntentRejected):
                controller.handle('confirm_landing',{'confirmation_token':token})
            assert len(peer.confirmations)==1
            peer.allow_ground.set()
            drive(lambda:backend.observation() is not None
                and backend.observation().flying is False and backend.observation().motors_on is False)
            assert controller.flight_started and not controller.landing_confirmed
            ground=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
            controller.handle('confirm_ground',{'confirmation_token':ground})
            assert controller.landing_confirmed and not controller.flight_started
            assert controller.state=='ready'
            controller.handle('close')
            actions=[p for p in peer.flight_frames if (p.cmd_set,p.cmd_id)==(3,0x2a)]
            assert [p.payload for p in actions]==[b'\1',b'\2',b'\x1e']
            assert len({p.seq for p in actions})==3
            assert set(peer.controls)=={(1024,)*4}
            events=[json.loads(line) for line in journal.path.read_text().splitlines()]
            replies=[e for e in events if e['event']=='action_response']
            assert [e['request_action'] for e in replies]==['takeoff','land','confirm_landing']
            assert all(e['accepted'] and not e['execution_confirmed'] for e in replies)
        finally:
            stop.set();worker.join(timeout=3);journal.close()
        assert not worker.is_alive() and not errors,errors
    assert results[0]['reason']=='requested' and results[0]['socket_cleanup_verified']
    wire_events=[json.loads(line) for line in (tmp_path/'flight.jsonl.session.jsonl').read_text().splitlines()]
    flag_updates=[item for event in wire_events if event.get('event')=='landing_confirmation_updated'
                  for item in event['landing_confirmation']['observations']
                  if item['fields']['landing_confirmation_needed']['value'] is True]
    assert flag_updates
    assert any(item['source']['wifi_tunnel'] is (not direct) for item in flag_updates)
    if direct:assert any(item['source'].get('direct_connection',{}).get('generation') for item in flag_updates)
