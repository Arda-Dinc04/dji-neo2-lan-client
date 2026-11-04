"""The scan extension retains source-bound wire guards and no horizontal axes."""
from dataclasses import replace
import pytest
from neo2_control.flight_channel import FlightChannel
from neo2_control.control_transport import FlightChannelBackend
from neo2_control.controller import ControlSupervisor,DecisionJournal,IntentRejected
from neo2_control.flight_connection import FlightConfig
from test_yaw_trial import YawWorld,airborne


@pytest.fixture
def rig(tmp_path):
    world=YawWorld(yaw_channel_limit=600)
    world.channel=FlightChannel(clock=lambda:world.now,yaw_trial=True,
        yaw_channel_limit=600,acquisition_trial=True)
    world.backend=FlightChannelBackend(world.channel,synthetic=True)
    world.channel.bind_transport('127.0.0.1',9003)
    world.feed();world.attest()
    journal=DecisionJournal(tmp_path/'scan.jsonl')
    controller=ControlSupervisor(journal,mode='simulate',backend=world.backend,
        clock=lambda:world.now,yaw_trial=True,acquisition_trial=True)
    controller.handle('open')
    yield world,controller
    journal.close()


def test_profile_changes_only_explicit_scan_limits(rig):
    world,controller=rig
    assert controller.status()['acquisition_trial'] is True
    assert controller.limits.maximum_flight_seconds==60
    assert controller.limits.maximum_altitude_m==1.8
    assert controller.limits.maximum_channel==600
    assert not world.backend.observation().position_valid


@pytest.mark.parametrize('up',[-200,-100,-30,0,30,100,200])
def test_vertical_and_yaw_reach_native_packet_without_horizontal_motion(rig,up):
    world,controller=rig;lease=airborne(world,controller)
    controller.handle('control',{'lease':lease,'sequence':1,'expires_at':world.now+.2,
        'channels':[0,0,up,600]})
    packet=world.channel.next_packet(world.now,world.sequence.next)
    assert world.channel.can_send(packet.seq,world.now)
    words=int.from_bytes(packet.payload[2:10],'little')
    assert [(words>>shift)&2047 for shift in (8,19)]==[1024,1024]
    assert ((words>>30)&2047)=={-200:892,-100:958,-30:1004,0:1024,30:1043,100:1090,200:1156}[up]


@pytest.mark.parametrize('channels',[[1,0,0,0],[0,-1,0,0],[0,0,201,0],[0,0,-201,0],[0,0,0,601]])
def test_invalid_scan_axes_are_refused_by_supervisor_and_wire_queue(rig,channels):
    world,controller=rig;lease=airborne(world,controller)
    with pytest.raises(IntentRejected):
        controller.handle('control',{'lease':lease,'sequence':1,'expires_at':world.now+.2,'channels':channels})
    with pytest.raises(ValueError):world.channel.submit_control(channels,expires_at=world.now+.2)


@pytest.mark.parametrize('change',['horizontal','vertical','altitude','stale'])
def test_final_send_rechecks_queued_axes_and_current_height_and_freshness(rig,change):
    world,controller=rig;lease=airborne(world,controller)
    controller.handle('control',{'lease':lease,'sequence':1,'expires_at':world.now+.2,'channels':[0,0,100,600]})
    packet=world.channel.next_packet(world.now,world.sequence.next)
    if change=='horizontal':world.channel.control['channels'][0]=1
    if change=='vertical':world.channel.control['channels'][2]=201
    if change=='altitude':world.channel.observer.sample=replace(world.channel.observer.sample,altitude_m=1.9)
    if change=='stale':world.now+=.51
    assert not world.channel.can_send(packet.seq,world.now)


def test_scan_cannot_be_enabled_without_explicit_yaw_profile():
    with pytest.raises(ValueError):FlightChannel(acquisition_trial=True)
    with pytest.raises(ValueError):FlightConfig('wlan9','a','b',acquisition_trial=True)


def test_independent_owner_receives_scan_profile_without_opening_radio(tmp_path):
    from neo2_control.control_owner import ProcessControllerRuntime
    config=FlightConfig('fixture9',tmp_path/'wlm',tmp_path/'ble',yaw_trial=True,
        acquisition_trial=True,yaw_channel_limit=600,incident_directory=tmp_path/'incidents')
    runtime=ProcessControllerRuntime(tmp_path/'owner.jsonl',mode='flight',flight_config=config)
    try:
        status=runtime.status()
        assert status['acquisition_trial'] is True and status['state']=='disconnected'
        assert status['connection']['worker_alive'] is False and not status['connected']
        assert status['limits']['maximum_altitude_m']==1.8 and status['limits']['maximum_flight_seconds']==60
    finally:runtime.close()


def test_buffered_owner_keeps_vertical_target_during_client_jitter_then_lands_on_source_expiry(rig):
    from neo2_control.remote_target import RemoteTargetIngress,RemoteTargetRunner
    world,controller=rig;lease=airborne(world,controller);started=world.now
    runner=RemoteTargetRunner(controller,'a'*48)
    source=RemoteTargetIngress('a'*48,None,1,started,started+.5)
    runner.handle('control',{'lease':lease,'sequence':0,'channels':[0,0,200,600]},source)
    sent=[]
    for _ in range(6):
        world.now+=.05;world.feed();runner.tick()
        item=world.channel.control
        assert item is not None and item['expires_at']<=min(world.now+.2,source.expires_at)+1e-9
        packet=world.channel.next_packet(world.now,world.sequence.next)
        assert packet is not None and world.channel.can_send(packet.seq,world.now)
        sent.append(packet);world.channel.sent(packet.seq,world.now)
    assert len(sent)==6 and runner.source is source and controller.lease_deadline<=source.expires_at
    world.now=started+.51;world.feed();runner.tick()
    assert runner.source is None and controller.state=='landing'
