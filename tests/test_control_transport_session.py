"""The real supervisor drives the real UDP owner against a loopback fixture."""
import json
import struct
import threading
import time

import pytest

from neo2_control.control_transport import FlightChannelBackend
from neo2_control.controller import ControlLimits, ControlSupervisor, DecisionJournal, FlightObservation, IntentRejected
from neo2_control.duml import DumlPacket
from neo2_control.flight_channel import FlightChannel
from neo2_control.session import run_session
from test_flight_channel_session import FlightPeer
from test_wlm_liveview import IDENTITY


class ControllerPeer(FlightPeer):
    """Explicit synthetic motion and navigation, encoded into received pushes."""
    def __init__(self,channel,*,withhold_action_ack=False,observed_action_replies=False,
                 direct_motion=False):
        super().__init__(channel,hold_action_ack=withhold_action_ack,
                         observed_action_replies=observed_action_replies)
        self.phase='ground'
        self.direct_motion=direct_motion
        self.complete_at=None
        self.controls=[]
        self.last_beacon=self.last_telemetry=0.
        self.last_complete=self.last_phase=None

    def beacon_bytes(self):
        now=time.monotonic()
        if self.complete_at is not None and now>=self.complete_at:
            self.phase='air' if self.phase=='taking_off' else 'ground'
            self.complete_at=None
        pushes=[]
        if now-self.last_beacon>=.3 or self.last_complete!=self.completed:
            pushes.append(super().beacon_bytes())
            self.last_beacon=now;self.last_complete=self.completed
        if now-self.last_telemetry<.1 and self.last_phase==self.phase:
            return b''.join(pushes)
        self.last_telemetry=now;self.last_phase=self.phase
        phase_flags={'ground':0,'taking_off':8,'air':12,'landing':14}[self.phase]
        # This fixture defines decimetres and phase0=ground. Those meanings
        # are synthetic input, not new claims about real aircraft telemetry.
        fc=bytearray(85)
        struct.pack_into('<ddh',fc,0,.2,.1,10 if self.phase=='air' else 0)
        struct.pack_into('<I',fc,32,0x388000|phase_flags)
        fc[36]=12
        battery=struct.pack('<IHBBBB',0,12345,25,15,90,1)
        # The SDK's51/04 state is local. This peer sends no such radio push.
        for receiver,command_set,command_id,body in ((3,3,0x43,fc),(3,3,0x55,battery)):
            request=DumlPacket(2,receiver,0x9331,0,command_set,command_id,b'')
            if self.direct_motion and self.phase!='ground' and command_id==0x43:
                pushes.append(DumlPacket(3,0xe,0x9331,0,3,0x43,bytes(body)).encode())
            else:
                pushes.append(self.camera_frame(request,kind=0,payload=bytes(body)))
        return b''.join(pushes)

    def additional_packet(self,inner,attributes):
        if inner.cmd_set in (2,4):return None
        if (inner.cmd_set,inner.cmd_id)==(3,0x2a):
            self.phase='taking_off' if inner.payload==b'\1' else 'landing'
            self.complete_at=time.monotonic()+.12
            return super().additional_packet(inner,attributes)
        assert (inner.sender,inner.receiver,inner.cmd_type,inner.cmd_set,inner.cmd_id)==(2,0xa9,0,1,0xa)
        assert len(inner.payload)==18 and inner.payload[:2]==b'\x01\x0d'
        word=int.from_bytes(inner.payload[2:10],'little')
        channels=tuple((word>>shift)&0x7ff for shift in (8,19,30,41))
        assert channels in ((1057,1024,1024,1024),(1024,)*4)
        self.controls.append(channels)
        self.flight_frames.append(inner)
        self.record_allocation(inner)
        return b''


@pytest.mark.parametrize('scenario',['normal','control-loss','ack-loss'])
@pytest.mark.parametrize('observation_source',['fixture-callback','owned','owned-flight-fixture'])
def test_supervisor_loopback_lifecycle_and_failures_use_wire_packets_and_received_fixture_observations(tmp_path,scenario,observation_source,*,motion_envelope='wrapped'):
    channel=FlightChannel();stop=threading.Event();latest=[None];lock=threading.Lock()
    def publish(snapshot):
        with lock:latest[0]=snapshot
    def observation():
        with lock:snapshot=latest[0]
        if snapshot is None:return None
        fc=snapshot['telemetry'];battery=snapshot['battery']
        if not fc['available'] or not battery['observations']:return None
        source=fc['fields']['altitude']['source']
        percent=battery['observations'][-1]
        # Both actual received carriers must refer to the same fixture peer.
        if source.get('forwarding',{}).get('source_hash')!=percent['source'].get('forwarding',{}).get('source_hash'):
            return None
        return FlightObservation(fc['received_at'],fc['flight_phase_code']==0,
            fc['fields']['is_flying']['value'],fc['fields']['motors_on']['value'],
            percent['fields']['percent']['value'],fc['fields']['gps_valid']['value'],
            fc['fields']['altitude']['value'],0.,'received_loopback_flight_fixture',True)
    owned=observation_source!='fixture-callback'
    synthetic=observation_source!='owned-flight-fixture'
    if not synthetic:
        # Exercise the real supervisor branch with declared fixture input,
        # while latching the transport to loopback before any socket binding.
        channel.require_loopback()
    backend=FlightChannelBackend(channel,None if owned else observation,synthetic=synthetic)
    journal=DecisionJournal(tmp_path/'decisions.jsonl')
    controller=ControlSupervisor(journal,mode='simulate' if synthetic else 'flight',backend=backend)
    identity=tmp_path/'identity.json'
    identity.write_text(json.dumps({'format':'neo2-control/wlm-identity/v1',
                                    'identification_id':IDENTITY.identification_id}))
    results=[];errors=[];lease=None;last_keepalive=0.
    def drive(predicate,*,timeout=5,renew=True):
        nonlocal last_keepalive
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            controller.tick()
            now=time.monotonic()
            if renew and lease and controller.state in ('armed','taking_off','flying') and now-last_keepalive>=.1:
                try:controller.handle('keepalive',{'lease':lease})
                except IntentRejected as error:
                    # The injected ACK deadline can revoke authority between
                    # this state read and handle()'s own watchdog update.
                    if str(error)!='operator_lease_invalid' or controller.state not in ('landing','fault'):raise
                last_keepalive=now
            if predicate():return
            time.sleep(.01)
        pytest.fail(f'supervisor did not reach expected state: {controller.status()}')
    with ControllerPeer(channel,withhold_action_ack=scenario=='ack-loss',
                        direct_motion=motion_envelope=='direct-on-motors') as peer:
        def run():
            try:
                results.append(run_session(capture_path=tmp_path/'flight.jsonl',host='127.0.0.1',
                    port=peer.sock.getsockname()[1],duration=10,heartbeat=True,flight_channel=channel,
                    wlm_identity_file=identity,wlm_wifi_liveview=True,snapshot_sink=publish,
                    stop_event=stop).as_dict())
            except BaseException as error:errors.append(repr(error))
        worker=threading.Thread(target=run);worker.start()
        try:
            controller.handle('open')
            if owned:
                drive(lambda:backend.observation() is not None and backend.observation().control_ready is True)
                assert controller.state=='not_ready' and backend.observation().grounded is None
                token=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
                controller.handle('confirm_ground',{'confirmation_token':token})
            drive(lambda:controller.state=='ready')
            token=controller.handle('prepare',{'operation':'arm'})['confirmation_token']
            lease=controller.handle('arm',{'confirmation_token':token})['lease']
            token=controller.handle('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
            controller.handle('takeoff',{'confirmation_token':token,'lease':lease})
            if scenario=='ack-loss':
                drive(lambda:controller.state=='fault')
                assert controller.fault_reason=='action_ack_timeout' and not controller.landing_confirmed
            else:
                drive(lambda:controller.state=='flying')
                controller.handle('control',{'lease':lease,'sequence':1,'channels':[50,0,0,0],
                    'expires_at':time.monotonic()+.2})
                drive(lambda:(1057,1024,1024,1024) in peer.controls,timeout=1)
                if scenario=='normal':
                    controller.handle('land')
                    if owned:
                        drive(lambda:backend.observation().motors_on is False and controller.action_acknowledged)
                        token=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
                        controller.handle('confirm_ground',{'confirmation_token':token})
                    drive(lambda:controller.state=='ready')
                    assert controller.fault_reason is None
                else:
                    if owned:
                        drive(lambda:controller.state=='landing' and backend.observation().motors_on is False and controller.action_acknowledged)
                        token=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
                        controller.handle('confirm_ground',{'confirmation_token':token})
                    drive(lambda:controller.state=='fault')
                    assert controller.fault_reason=='control_expired'
                assert controller.landing_confirmed
                controller.handle('close')
            action_packets=[p for p in peer.flight_frames if (p.cmd_set,p.cmd_id)==(3,0x2a)]
            assert [p.payload for p in action_packets]==[b'\1',b'\2']
            land_index=peer.flight_frames.index(action_packets[-1])
            assert land_index>0
            preceding=peer.flight_frames[land_index-1]
            assert (preceding.cmd_set,preceding.cmd_id)==(1,0xa)
            word=int.from_bytes(preceding.payload[2:10],'little')
            assert tuple((word>>shift)&0x7ff for shift in (8,19,30,41))==(1024,)*4
            events=[json.loads(line) for line in journal.path.read_text().splitlines()]
            ack_sequences={r['command_sequence'] for r in events if r['event']=='action_response'}
            assert ack_sequences<={p.seq for p in action_packets}
            if scenario!='ack-loss':assert ack_sequences=={p.seq for p in action_packets}
        finally:
            stop.set();worker.join(timeout=3);journal.close()
        assert not worker.is_alive() and not errors,errors
    assert results[0]['reason']=='requested'
    assert results[0]['wifi_control']['accepted']==0
    assert all((p.cmd_set,p.cmd_id) not in ((0x51,0x14),(0x51,0x15),(0x51,0x1a))
               for p in peer.requests)
    assert results[0]['wlm_mode_follow']['flight_channel']['loopback_only']
    assert not results[0]['application_ready']


@pytest.mark.parametrize('scenario',['normal','control-loss','ack-loss'])
def test_motor_start_switch_to_direct_fc_preserves_supervision_and_real_failures(tmp_path,scenario):
    test_supervisor_loopback_lifecycle_and_failures_use_wire_packets_and_received_fixture_observations(
        tmp_path,scenario,'owned-flight-fixture',motion_envelope='direct-on-motors')


def test_explicit_fault_landing_recovery_uses_new_udp_transaction_and_rejects_old_ack(tmp_path):
    class RecoveryPeer(ControllerPeer):
        """ACK the first LAND without motion; withhold the retry's actual ACK."""
        def __init__(self, channel):
            super().__init__(channel, observed_action_replies=True, direct_motion=True)
            self.lands=[]
            self.held_response=None
            self.release_ack=threading.Event()

        def additional_packet(self, inner, attributes):
            if (inner.cmd_set,inner.cmd_id,inner.payload)==(3,0x2a,b'\2'):
                self.lands.append(inner)
                response=FlightPeer.additional_packet(self,inner,attributes)
                if len(self.lands)==2:
                    self.held_response=response
                    old=self.lands[0]
                    return DumlPacket(old.receiver,old.sender,old.seq,0x80,3,0x2a,b'\0').encode()
                return response
            return super().additional_packet(inner,attributes)

        def beacon_bytes(self):
            payload=super().beacon_bytes()
            if self.held_response is not None and self.release_ack.is_set():
                payload+=self.held_response
                self.held_response=None
            return payload

    channel=FlightChannel();channel.require_loopback()
    backend=FlightChannelBackend(channel,synthetic=False)
    journal=DecisionJournal(tmp_path/'recovery-decisions.jsonl')
    controller=ControlSupervisor(journal,mode='flight',backend=backend,
        limits=ControlLimits(transition_seconds=.7))
    identity=tmp_path/'identity.json'
    identity.write_text(json.dumps({'format':'neo2-control/wlm-identity/v1',
                                    'identification_id':IDENTITY.identification_id}))
    stop=threading.Event();results=[];errors=[];lease=None;last_keepalive=0.

    def drive(predicate, *, timeout=4):
        nonlocal last_keepalive
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            controller.tick()
            now=time.monotonic()
            if lease and controller.state in ('armed','taking_off','flying') and now-last_keepalive>=.1:
                controller.handle('keepalive',{'lease':lease})
                last_keepalive=now
            if predicate():return
            time.sleep(.01)
        pytest.fail(f'recovery fixture did not reach expected state: {controller.status()}')

    with RecoveryPeer(channel) as peer:
        def run():
            try:
                results.append(run_session(capture_path=tmp_path/'recovery.jsonl',host='127.0.0.1',
                    port=peer.sock.getsockname()[1],duration=12,heartbeat=True,flight_channel=channel,
                    wlm_identity_file=identity,wlm_wifi_liveview=True,stop_event=stop).as_dict())
            except BaseException as error:errors.append(repr(error))
        worker=threading.Thread(target=run);worker.start()
        try:
            controller.handle('open')
            drive(lambda:backend.observation() is not None and backend.observation().control_ready is True)
            token=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
            controller.handle('confirm_ground',{'confirmation_token':token})
            drive(lambda:controller.state=='ready')
            token=controller.handle('prepare',{'operation':'arm'})['confirmation_token']
            lease=controller.handle('arm',{'confirmation_token':token})['lease']
            token=controller.handle('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
            controller.handle('takeoff',{'confirmation_token':token,'lease':lease})
            drive(lambda:controller.state=='flying')
            controller.handle('land')
            drive(lambda:controller.state=='fault')
            assert controller.fault_reason=='landing_unconfirmed'
            assert controller.flight_started and not controller.landing_confirmed
            assert len(peer.lands)==1
            controller.handle('land')
            retry_intent=controller.pending_intent_id
            drive(lambda:len(peer.lands)==2)
            first,retry=peer.lands
            assert first.seq!=retry.seq
            assert controller.pending is not None and not controller.action_acknowledged
            assert controller.handle('land')['landing_already_pending'] is True
            assert controller.pending_intent_id==retry_intent and len(peer.lands)==2
            assert channel.receipt(retry_intent)['expires_at']-channel.receipt(retry_intent)['created_at']<=.25
            with pytest.raises(IntentRejected,match='operator_lease_invalid'):
                controller.handle('control',{'lease':lease,'sequence':1,'channels':[50,0,0,0],
                    'expires_at':time.monotonic()+.2})
            peer.release_ack.set()
            drive(lambda:controller.pending is None and controller.action_acknowledged)
            assert controller.flight_started and not controller.landing_confirmed
            drive(lambda:controller.state=='fault')
            assert controller.fault_reason=='landing_unconfirmed' and len(peer.lands)==2
            assert controller.lease is controller.control is controller.challenge is None
            assert peer.controls and set(peer.controls)=={(1024,)*4}
            actions=[p for p in peer.flight_frames if (p.cmd_set,p.cmd_id)==(3,0x2a)]
            assert [p.payload for p in actions]==[b'\1',b'\2',b'\2']
            for request in peer.lands:
                previous=peer.flight_frames[peer.flight_frames.index(request)-1]
                assert (previous.cmd_set,previous.cmd_id)==(1,0xa)
            events=[json.loads(line) for line in journal.path.read_text().splitlines()]
            replies=[e for e in events if e['event']=='action_response' and e['request_action']=='land']
            assert [e['command_sequence'] for e in replies]==[first.seq,retry.seq]
            assert all(e['accepted'] and not e['execution_confirmed'] for e in replies)
        finally:
            stop.set();worker.join(timeout=3);journal.close()
        assert not worker.is_alive() and not errors,errors
    assert results[0]['reason']=='requested'
    assert channel.snapshot()['loopback_only']
