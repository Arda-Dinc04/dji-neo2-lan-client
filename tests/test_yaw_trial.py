"""Explicit rotation-only profile with decoded fixtures and no real hardware."""
from dataclasses import replace
import struct
import time

import pytest

from neo2_control.command_ingress import CommandIngress
from neo2_control.control_owner import ProcessControllerRuntime
from neo2_control.control_transport import FlightChannelBackend
from neo2_control.controller import ControlLimits, ControlSupervisor, DecisionJournal, IntentRejected
from neo2_control.duml import DumlPacket
from neo2_control.flight import FlightAction
from neo2_control.flight_channel import FlightChannel
from neo2_control.flight_connection import FlightConfig, OwnedFlightBackend
from test_landing_supervisor import LandingWorld
from test_observation_envelope import carrier


class YawWorld(LandingWorld):
    def __init__(self, *, yaw_trial=True, vertical_trial=False, yaw_channel_limit=200):
        super().__init__()
        self.now = 100.
        self.yaw_raw = 1234
        self.altitude_raw = 10
        self.gps = False
        self.channel = FlightChannel(clock=lambda:self.now, yaw_trial=yaw_trial,
            vertical_trial=vertical_trial,yaw_channel_limit=yaw_channel_limit)
        self.backend = FlightChannelBackend(self.channel, synthetic=True)
        self.channel.bind_transport('127.0.0.1',9003)

    def feed(self):
        body = bytearray(85)
        struct.pack_into('<ddh',body,0,.2,.1,self.altitude_raw if self.phase else 0)
        struct.pack_into('<3h',body,24,111,-222,self.yaw_raw)
        struct.pack_into('<I',body,32,(0x388000 if self.gps else 0)|self.phase)
        body[30],body[36] = self.flight_mode,12
        packet = DumlPacket(3,2,0x9000,0,3,0x43,bytes(body))
        self.fc.feed(carrier(packet.encode(),source=self.route.peer_hash,
            destination=self.route.own_hash).encode(),self.now)
        self.push('battery',percent=self.percent)
        self.project()


@pytest.fixture
def yaw_rig(tmp_path):
    journals=[]
    def make(*, limits=None, yaw_trial=True, yaw_channel_limit=200):
        world = YawWorld(yaw_trial=yaw_trial,yaw_channel_limit=yaw_channel_limit)
        world.feed();world.attest()
        journal = DecisionJournal(tmp_path/f'yaw-{len(journals)}.jsonl')
        journals.append(journal)
        controller = ControlSupervisor(journal,mode='simulate',backend=world.backend,
            clock=lambda:world.now,yaw_trial=yaw_trial,limits=limits)
        controller.handle('open')
        return world,controller,journal
    yield make
    for journal in journals:journal.close()


def queue_takeoff(world,controller):
    token = controller.handle('prepare',{'operation':'arm'})['confirmation_token']
    lease = controller.handle('arm',{'confirmation_token':token})['lease']
    token = controller.handle('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
    controller.handle('takeoff',{'confirmation_token':token,'lease':lease})
    return lease


def airborne(world,controller):
    lease = queue_takeoff(world,controller)
    assert world.send().payload == b'\1'
    world.now += .01;world.phase = 12;world.feed();controller.tick()
    assert controller.state == 'flying'
    return lease


def test_yaw_projection_uses_third_degree_component_and_keeps_navigation_unknown(yaw_rig):
    world,controller,_ = yaw_rig()
    sample = world.backend.observation()
    assert sample.yaw_deg == pytest.approx(123.4)
    assert not sample.position_valid and sample.distance_m is None
    assert world.channel.observer.origin is None
    status = controller.status()
    assert status['yaw_trial'] is True and status['vertical_trial'] is False
    assert status['state'] == 'ready'
    assert status['limits']['maximum_flight_seconds'] == 30
    assert status['limits']['maximum_altitude_m'] == 1.5
    assert status['limits']['maximum_channel'] == 200
    assert status['limits']['acknowledgement_seconds'] == 1
    world.now += .51
    assert world.backend.observation().yaw_deg is None


def test_newer_foreign_attitude_cannot_replace_bound_yaw(yaw_rig):
    world,_,_ = yaw_rig()
    world.now += .1
    world.push('fc',source=99,phase=12)
    world.project()
    sample = world.backend.observation()
    assert sample.yaw_deg == pytest.approx(123.4) and sample.flying is False


def test_only_explicit_yaw_profile_can_be_ready_without_gps(yaw_rig):
    _,ordinary,_ = yaw_rig(yaw_trial=False)
    assert ordinary.state == 'not_ready'
    with pytest.raises(IntentRejected,match='invalid_state'):
        ordinary.handle('prepare',{'operation':'arm'})


@pytest.mark.parametrize('channels',[[1,0,0,0],[0,-1,0,0],[0,0,1,0],[1,0,0,200],
    [0,0,0,201],[0,0,0,-201],[0,0,0,True]])
def test_translation_throttle_and_excess_yaw_are_rejected_in_both_layers(yaw_rig,channels):
    world,controller,_ = yaw_rig()
    lease = airborne(world,controller)
    with pytest.raises(IntentRejected):
        controller.handle('control',{'lease':lease,'sequence':1,
            'expires_at':world.now+.2,'channels':channels})
    with pytest.raises(ValueError):world.channel.submit_control(channels,expires_at=world.now+.2)
    assert world.channel.control is None


@pytest.mark.parametrize('yaw',[-200,0,200])
def test_yaw_only_wire_and_original_ingress_deadline(yaw_rig,yaw):
    world,controller,_ = yaw_rig()
    lease = airborne(world,controller)
    ingress = CommandIngress('a'*48,None,1,world.now,world.now+.15)
    controller.handle('control',{'lease':lease,'sequence':1,'expires_at':world.now+.2,
        'channels':[0,0,0,yaw]},ingress=ingress)
    assert world.channel.control['expires_at'] == ingress.expires_at
    packet = world.channel.next_packet(world.now,world.sequence.next)
    assert packet.cmd_set == 1 and packet.cmd_id == 0x0a
    assert world.channel.can_send(packet.seq,world.now)
    words = int.from_bytes(packet.payload[2:10],'little')
    assert [(words >> shift)&2047 for shift in (8,19,30)] == [1024]*3


@pytest.mark.parametrize('stage',['takeoff','yaw'])
@pytest.mark.parametrize('loss',['yaw','motors','flying','battery','altitude','host','route','expiry','motion'])
def test_final_send_rechecks_yaw_source_and_all_non_navigation_guards(yaw_rig,stage,loss):
    world,controller,_ = yaw_rig()
    if stage == 'takeoff':queue_takeoff(world,controller)
    else:
        lease = airborne(world,controller)
        controller.handle('control',{'lease':lease,'sequence':1,'expires_at':world.now+.2,
            'channels':[0,0,0,200]})
    packet = world.channel.next_packet(world.now,world.sequence.next)
    observer = world.channel.observer
    sample = observer.sample
    if loss == 'yaw':observer.sample = replace(sample,yaw_deg=None)
    elif loss == 'motors':observer.sample = replace(sample,motors_on=stage=='takeoff')
    elif loss == 'flying':observer.sample = replace(sample,flying=stage=='takeoff')
    elif loss == 'battery':observer.sample = replace(sample,battery_percent=19)
    elif loss == 'altitude':observer.sample = replace(sample,altitude_m=1.6)
    elif loss == 'host':observer.route = replace(observer.route,
        host_control=replace(observer.route.host_control,committed_at=None))
    elif loss == 'route':world.channel.set_ready(False,world.now)
    elif loss == 'expiry':world.now += .26
    else:world.now += .51
    assert not world.channel.can_send(packet.seq,100.)


def test_neutral_before_land_remains_available_without_yaw(yaw_rig):
    world,controller,_ = yaw_rig()
    airborne(world,controller)
    world.yaw_raw = 32767
    world.now += .01;world.feed()
    assert world.backend.observation().yaw_deg is None
    controller.handle('land')
    assert world.send().cmd_id == 0x0a
    assert world.send().payload == b'\2'


@pytest.mark.parametrize('change',['axis','nonce'])
def test_final_rotation_guard_rejects_changed_axes_or_peer_generation(yaw_rig,change):
    world,controller,_ = yaw_rig()
    lease = airborne(world,controller)
    controller.handle('control',{'lease':lease,'sequence':1,'expires_at':world.now+.2,
        'channels':[0,0,0,200]})
    packet = world.channel.next_packet(world.now,world.sequence.next)
    if change == 'axis':world.channel.control['channels'][0] = 1
    else:
        world.route = replace(world.route,nonce=world.route.nonce+1,host_control=None)
        world.feed()
    assert not world.channel.can_send(packet.seq,world.now)


def test_stricter_limits_reach_the_final_channel_and_are_not_relaxed(yaw_rig):
    limits = ControlLimits(maximum_flight_seconds=12,maximum_altitude_m=.7,
        maximum_channel=80,acknowledgement_seconds=.3,minimum_battery_percent=35)
    world,controller,_ = yaw_rig(limits=limits)
    assert controller.limits == limits
    queue_takeoff(world,controller)
    assert world.channel.action['yaw_limits'] == {
        'minimum_battery_percent':35,'maximum_altitude_m':.7,'maximum_channel':80}
    packet = world.channel.next_packet(world.now,world.sequence.next)
    world.channel.observer.sample = replace(world.channel.observer.sample,battery_percent=34)
    assert not world.channel.can_send(packet.seq,world.now)


def test_refused_second_takeoff_cannot_loosen_latched_yaw_bounds(yaw_rig):
    world,_,_ = yaw_rig()
    strict={'minimum_battery_percent':35,'maximum_altitude_m':.7,'maximum_channel':80}
    world.channel.submit_action(FlightAction.TAKEOFF,expires_at=world.now+.2,yaw_limits=strict)
    with pytest.raises(RuntimeError,match='flight_action_pending'):
        world.channel.submit_action(FlightAction.TAKEOFF,expires_at=world.now+.2)
    strict['maximum_channel']=200
    assert world.channel._yaw_limits['maximum_channel']==80
    assert world.channel.action['yaw_limits']['maximum_channel']==80


def test_queued_rotation_retains_strict_bounds_at_its_final_send(yaw_rig):
    world,controller,_ = yaw_rig(limits=ControlLimits(maximum_channel=80))
    lease=airborne(world,controller)
    controller.handle('control',{'lease':lease,'sequence':1,'expires_at':world.now+.2,
        'channels':[0,0,0,80]})
    packet=world.channel.next_packet(world.now,world.sequence.next)
    world.channel._yaw_limits['maximum_channel']=200
    world.channel.control['channels'][3]=100
    assert not world.channel.can_send(packet.seq,world.now)


def test_profile_flight_deadline_includes_takeoff_and_preserves_stricter_duration(yaw_rig):
    world,controller,_=yaw_rig(limits=ControlLimits(maximum_flight_seconds=.2))
    airborne(world,controller)
    world.now += .2
    world.feed();controller.tick()
    assert controller.fault_reason=='maximum_flight_time' and controller.state=='landing'
    assert world.send().cmd_id==0x0a and world.send().payload==b'\2'


@pytest.mark.parametrize('invalid',[None,0,1,'true'])
def test_yaw_profile_requires_an_actual_boolean(tmp_path,invalid):
    with pytest.raises(ValueError):FlightChannel(yaw_trial=invalid)
    with pytest.raises(ValueError):FlightConfig('fixture0',tmp_path/'w',tmp_path/'b',yaw_trial=invalid)


def test_profiles_cannot_be_combined_or_changed_remotely(tmp_path):
    with pytest.raises(ValueError,match='exclusive'):FlightChannel(vertical_trial=True,yaw_trial=True)
    with pytest.raises(ValueError,match='exclusive'):
        FlightConfig('fixture0',tmp_path/'w',tmp_path/'b',vertical_trial=True,yaw_trial=True)
    journal = DecisionJournal(tmp_path/'observe')
    try:
        with pytest.raises(ValueError,match='yaw_trial_requires_control_backend'):
            ControlSupervisor(journal,yaw_trial=True)
    finally:journal.close()


def test_owned_configuration_and_real_inert_owner_preserve_profile_without_radio(tmp_path):
    config = FlightConfig('fixture0',tmp_path/'w',tmp_path/'b',yaw_trial=True,
        incident_directory=tmp_path/'incidents')
    backend = OwnedFlightBackend(FlightConfig(**config.as_dict()))
    assert backend.channel.yaw_trial and not backend.channel.vertical_trial
    assert backend.connection.thread is None
    runtime = ProcessControllerRuntime(tmp_path/'owner',mode='flight',flight_config=config)
    try:
        status = runtime.status()
        assert status['yaw_trial'] and not status['vertical_trial']
        assert status['state'] == 'disconnected' and not status['connection']['worker_alive']
    finally:runtime.close()


@pytest.mark.parametrize('ceiling',[400,600])
def test_explicit_yaw_ceiling_reaches_final_wire_without_relaxing_other_guards(yaw_rig,ceiling):
    world,controller,_=yaw_rig(yaw_channel_limit=ceiling)
    assert controller.limits.maximum_channel==ceiling
    assert controller.limits.maximum_flight_seconds==30
    assert controller.limits.maximum_altitude_m==1.5
    lease=airborne(world,controller)
    controller.handle('control',{'lease':lease,'sequence':1,'expires_at':world.now+.2,
        'channels':[0,0,0,ceiling]})
    packet=world.channel.next_packet(world.now,world.sequence.next)
    assert world.channel.can_send(packet.seq,world.now)
    word=int.from_bytes(packet.payload[2:10],'little')
    assert [(word>>shift)&2047 for shift in (8,19,30)]==[1024]*3
    assert (word>>41)&2047==int(1024+ceiling*.66)
    world.channel.observer.sample=replace(world.channel.observer.sample,battery_percent=19)
    assert not world.channel.can_send(packet.seq,world.now)
    for channels in ([1,0,0,ceiling],[0,0,0,ceiling+1]):
        with pytest.raises(IntentRejected):
            controller.handle('control',{'lease':lease,'sequence':2,'expires_at':world.now+.2,'channels':channels})
        with pytest.raises(ValueError):world.channel.submit_control(channels,expires_at=world.now+.2)


def test_higher_profile_ceiling_does_not_relax_stricter_controller_or_queued_bounds(yaw_rig):
    world,controller,_=yaw_rig(yaw_channel_limit=600,limits=ControlLimits(maximum_channel=300))
    assert controller.limits.maximum_channel==300
    lease=airborne(world,controller)
    controller.handle('control',{'lease':lease,'sequence':1,'expires_at':world.now+.2,'channels':[0,0,0,300]})
    packet=world.channel.next_packet(world.now,world.sequence.next)
    world.channel._yaw_limits['maximum_channel']=600
    world.channel.control['channels'][3]=400
    assert not world.channel.can_send(packet.seq,world.now)


@pytest.mark.parametrize('invalid',[True,0,601,1000,400.,'400'])
def test_yaw_ceiling_is_bounded_at_configuration_time(tmp_path,invalid):
    with pytest.raises(ValueError):FlightChannel(yaw_trial=True,yaw_channel_limit=invalid)
    with pytest.raises(ValueError):
        FlightConfig('fixture0',tmp_path/'w',tmp_path/'b',yaw_trial=True,yaw_channel_limit=invalid)


def test_higher_ceiling_requires_yaw_profile_and_survives_inert_owner_start(tmp_path):
    with pytest.raises(ValueError,match='requires_yaw_trial'):FlightChannel(yaw_channel_limit=400)
    with pytest.raises(ValueError,match='requires_yaw_trial'):
        FlightConfig('fixture0',tmp_path/'w',tmp_path/'b',yaw_channel_limit=400)
    config=FlightConfig('fixture0',tmp_path/'w',tmp_path/'b',yaw_trial=True,yaw_channel_limit=400,
        incident_directory=tmp_path/'incidents')
    runtime=ProcessControllerRuntime(tmp_path/'owner-higher',mode='flight',flight_config=FlightConfig(**config.as_dict()))
    try:
        status=runtime.status()
        assert status['limits']['maximum_channel']==400
        assert status['state']=='disconnected' and not status['connection']['worker_alive']
    finally:runtime.close()


@pytest.mark.parametrize('raw',[-1801,1801,32767])
def test_invalid_yaw_refuses_preflight_without_inventing_an_angle(yaw_rig,raw):
    world,controller,_ = yaw_rig()
    world.yaw_raw = raw
    world.now += .01;world.feed();controller.tick()
    assert controller.last_sample.yaw_deg is None and controller.state == 'not_ready'
    with pytest.raises(IntentRejected):controller.handle('prepare',{'operation':'arm'})


def test_yaw_profile_crosses_real_owner_and_udp_without_navigation_or_translation(tmp_path,monkeypatch):
    from test_control_transport_session import ControllerPeer
    from test_flight_owner import Owner

    class Peer(ControllerPeer):
        def camera_frame(self,request,**options):
            if (request.cmd_set,request.cmd_id)==(3,0x43) and 'payload' in options:
                body=bytearray(options['payload'])
                flags=struct.unpack_from('<I',body,32)[0] & ~0x388000
                struct.pack_into('<I',body,32,flags)
                struct.pack_into('<3h',body,24,111,-222,420)
                options['payload']=bytes(body)
            return super().camera_frame(request,**options)

        def additional_packet(self,inner,attributes):
            if (inner.cmd_set,inner.cmd_id)!=(1,0x0a):return super().additional_packet(inner,attributes)
            word=int.from_bytes(inner.payload[2:10],'little')
            channels=tuple((word >> shift)&2047 for shift in (8,19,30,41))
            assert channels[:3]==(1024,)*3 and 892 <= channels[3] <= 1156
            self.controls.append(channels);self.flight_frames.append(inner);self.record_allocation(inner)
            return b''

    initialize=ProcessControllerRuntime.__init__
    def yaw_owner(self,*args,**kwargs):
        kwargs['flight_config']=replace(kwargs['flight_config'],yaw_trial=True)
        initialize(self,*args,**kwargs)
    monkeypatch.setattr(ProcessControllerRuntime,'__init__',yaw_owner)
    with Peer(FlightChannel()) as peer:
        owned=Owner(tmp_path,monkeypatch,peer)
        try:
            owned.takeoff()
            status=owned.runtime.status()
            assert status['yaw_trial'] and status['telemetry']['yaw_deg']==42
            assert status['telemetry']['position_valid'] is False and status['telemetry']['distance_m'] is None
            ingress=CommandIngress(owned.runtime.epoch,status['connection']['generation'],1,
                time.monotonic(),time.monotonic()+.2)
            owned.runtime.handle('control',{'lease':owned.lease,'sequence':1,'channels':[0,0,0,200],
                'expires_at':time.monotonic()+.2},ingress=ingress)
            owned.drive(lambda s:(1024,1024,1024,1156) in peer.controls,timeout=1)
            owned.runtime.handle('land')
            owned.drive(lambda s:s['telemetry']['motors_on'] is False and any(
                e['event']=='action_response' and e.get('request_action')=='land' for e in s['events']))
            owned.ground();owned.drive(lambda s:s['state']=='ready')
            assert [p.payload for p in peer.flight_frames if (p.cmd_set,p.cmd_id)==(3,0x2a)]==[b'\1',b'\2']
        finally:owned.cleanup()


def test_continuous_http_yaw_crosses_real_owner_and_loopback_udp(tmp_path,monkeypatch):
    """Exercise command pacing end to end; preview is outside this fixture."""
    from test_control_transport_session import ControllerPeer
    from test_flight_owner import Owner
    from test_flight_web import server
    from neo2_control.operator_client import FlightOperatorClient

    class Peer(ControllerPeer):
        def camera_frame(self,request,**options):
            if (request.cmd_set,request.cmd_id)==(3,0x43) and 'payload' in options:
                body=bytearray(options['payload'])
                struct.pack_into('<I',body,32,struct.unpack_from('<I',body,32)[0]&~0x388000)
                struct.pack_into('<3h',body,24,0,0,420)
                options['payload']=bytes(body)
            return super().camera_frame(request,**options)
        def additional_packet(self,inner,attributes):
            if (inner.cmd_set,inner.cmd_id)!=(1,0x0a):return super().additional_packet(inner,attributes)
            word=int.from_bytes(inner.payload[2:10],'little')
            channels=tuple((word>>shift)&2047 for shift in (8,19,30,41))
            assert channels[:3]==(1024,)*3 and channels[3] in (1024,1156)
            self.controls.append(channels);self.flight_frames.append(inner);self.record_allocation(inner)
            return b''
    initialize=ProcessControllerRuntime.__init__
    def yaw_owner(self,*args,**kwargs):
        kwargs['flight_config']=replace(kwargs['flight_config'],yaw_trial=True)
        initialize(self,*args,**kwargs)
    monkeypatch.setattr(ProcessControllerRuntime,'__init__',yaw_owner)
    with Peer(FlightChannel()) as peer:
        owned=Owner(tmp_path,monkeypatch,peer)
        try:
            owned.takeoff()
            with server(tmp_path/'http',runtime_factory=lambda *a,**kw:owned.runtime) as (base,app):
                client=FlightOperatorClient(base);client.pin_connection(client.status())
                stop=time.monotonic()+3.;sequence=0
                while time.monotonic()<stop:
                    status=client.status()
                    assert status['state']=='flying' and not status['telemetry']['position_valid']
                    client.intent('control',{'lease':owned.lease,'sequence':sequence,'channels':[0,0,0,200]})
                    sequence+=1
                    time.sleep(.05)
                assert sequence>=15 and len(peer.controls)>=15
                client.intent('land')
                owned.drive(lambda s:s['telemetry']['motors_on'] is False,renew=False)
                owned.ground();client.intent('close')
            assert owned.runtime.cleanup_verified
            assert [p.payload for p in peer.flight_frames if (p.cmd_set,p.cmd_id)==(3,0x2a)]==[b'\1',b'\2']
        finally:owned.cleanup()
