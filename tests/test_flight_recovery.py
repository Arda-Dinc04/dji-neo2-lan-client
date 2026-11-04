"""Durable recovery through real files/processes and loopback-only fixtures."""
from dataclasses import replace
import hashlib
import json

import pytest

from neo2_control.controller import ControlSupervisor,DecisionJournal,IntentRejected
from neo2_control.flight import FlightAction
from neo2_control.flight_incident import FlightIncidentStore
from neo2_control.flight_channel import FlightChannel
from neo2_control.flight_connection import FlightConfig
from neo2_control.control_owner import ProcessControllerRuntime,OwnerUnavailable
from test_landing_supervisor import LandingWorld
from test_vertical_trial import queue_takeoff
from test_flight_owner import Owner
from test_control_transport_session import ControllerPeer
from test_wlm_liveview import PEER

ADDRESS='02:00:00:00:00:01'


@pytest.fixture
def rig(tmp_path):
    resources=[]
    def make(*,active=False,world=None,directory=None):
        world=world or LandingWorld()
        world.feed()
        store=FlightIncidentStore(directory or tmp_path/f'incidents-{len(resources)}',ADDRESS).__enter__()
        if active:store.mark_airborne(world.route.peer_id_sha256,world.route.peer_hash)
        journal=DecisionJournal(tmp_path/f'decisions-{len(resources)}.jsonl')
        controller=ControlSupervisor(journal,mode='simulate',backend=world.backend,
            clock=lambda:world.now,incident_store=store)
        resources.append((store,journal))
        return world,controller,store,journal
    yield make
    for store,journal in resources:
        store.close();journal.close()


def attest(controller):
    token=controller.handle('prepare',{'operation':'ground'})['confirmation_token']
    return controller.handle('confirm_ground',{'confirmation_token':token})


def test_incident_is_durable_before_takeoff_enqueue_and_only_physical_attestation_clears(rig,monkeypatch):
    world,controller,store,journal=rig()
    controller.handle('open');attest(controller)
    submit=world.channel.submit_action
    def checked(action,**options):
        if action is FlightAction.TAKEOFF:
            assert store.active['reason']=='takeoff_requested'
            assert json.loads((store.directory/store.filename).read_text())['active']==store.active
            assert options['aircraft_key'] is not None
        return submit(action,**options)
    monkeypatch.setattr(world.channel,'submit_action',checked)
    queue_takeoff(world,controller)
    assert store.active is not None
    assert world.send().payload==b'\1'
    world.now+=.01;world.phase=12;world.feed();controller.tick()
    assert controller.state=='flying'
    controller.handle('land');world.send();assert world.send().payload==b'\2'
    world.now+=.01;world.phase=0;world.feed();controller.tick()
    assert controller.action_acknowledged and not controller.landing_confirmed
    assert controller.flight_started and store.active is not None
    attest(controller)
    assert store.active is None and controller.landing_confirmed and not controller.flight_started
    assert controller.state=='ready'
    assert not any('object at' in line for line in journal.path.read_text().splitlines())


def test_reopen_incident_starts_disconnected_without_authority_and_explicit_land_uses_new_context(rig):
    world,controller,store,_=rig(active=True)
    startup=controller.status()
    assert startup['state']=='disconnected' and startup['possible_airborne']
    assert startup['recovery']['required'] and startup['recovery']['peer_match'] is None
    assert controller.lease is controller.challenge is controller.pending is None
    controller.handle('open')
    assert controller.status()['recovery']['peer_match'] is True
    for action,parameters in [('prepare',{'operation':'arm'}),('arm',{'confirmation_token':'old'}),
        ('takeoff',{'confirmation_token':'old','lease':'old'}),('control',{'lease':'old'}),('reset',{})]:
        with pytest.raises(IntentRejected):controller.handle(action,parameters)
    assert world.channel.next_packet(world.now,world.sequence.next) is None
    controller.handle('land');world.send();assert world.send().payload==b'\2'
    world.now+=.01;world.feed();controller.tick()
    assert store.active is not None and controller.flight_started
    attest(controller)
    assert store.active is None and not controller.flight_started
    assert not controller.status()['recovery']['required']
    assert controller.state=='fault' and controller.lease is None
    controller.handle('reset');assert controller.state=='ready'


@pytest.mark.parametrize('change',[{'peer_id_sha256':'b'*64},{'peer_hash':99}])
def test_wrong_aircraft_cannot_land_or_clear_an_existing_incident(rig,change):
    world,controller,store,_=rig(active=True)
    original=store.active
    world.route=replace(world.route,host_control=None,**change)
    world.feed();controller.handle('open')
    assert controller.status()['recovery']['peer_match'] is False
    with pytest.raises(IntentRejected):controller.handle('land')
    with pytest.raises(IntentRejected):attest(controller)
    assert store.active==original and controller.flight_started
    assert world.channel.action is world.channel.control is None


@pytest.mark.parametrize('change',['slow-sync','new-generation'])
def test_persistence_delay_or_generation_change_never_refreshes_takeoff_authority(rig,monkeypatch,change):
    world,controller,store,_=rig()
    controller.handle('open');attest(controller)
    mark=store.mark_airborne
    def persist(*args,**kwargs):
        value=mark(*args,**kwargs)
        if change=='slow-sync':world.now+=.3
        else:world.route=replace(world.route,nonce=124,host_control=None)
        world.feed()
        return value
    monkeypatch.setattr(store,'mark_airborne',persist)
    with pytest.raises(IntentRejected):queue_takeoff(world,controller)
    assert store.active is not None and controller.flight_started
    assert controller.state=='fault' and controller.lease is None
    assert world.channel.sent_count==0 and world.channel.action is None


@pytest.mark.parametrize('operation',['mark','resolve'])
def test_disk_failure_never_clears_uncertainty_or_restores_a_lease(rig,monkeypatch,operation):
    world,controller,store,_=rig(active=operation=='resolve')
    controller.handle('open')
    def fail(*args,**kwargs):raise OSError('test-only disk failure')
    if operation=='mark':
        attest(controller)
        monkeypatch.setattr(store,'mark_airborne',fail)
        with pytest.raises(IntentRejected,match='incident_storage_unavailable'):queue_takeoff(world,controller)
    else:
        monkeypatch.setattr(store,'resolve_ground',fail)
        with pytest.raises(IntentRejected,match='incident_storage_unavailable'):attest(controller)
    status=controller.status()
    assert status['recovery']['required'] and status['possible_airborne']
    assert status['recovery']['reason']=='incident_storage_unavailable'
    assert not status['landing_confirmed'] and controller.lease is None
    assert status['telemetry']['grounded'] is None
    with pytest.raises(IntentRejected):controller.handle('reset')
    assert world.channel.sent_count==0


@pytest.mark.parametrize('action',['takeoff','land'])
def test_incident_pair_and_local_generation_are_rechecked_at_actual_send(rig,action):
    world,controller,store,_=rig(active=action=='land')
    controller.handle('open')
    if action=='takeoff':attest(controller);queue_takeoff(world,controller)
    else:
        controller.handle('land');world.send() # Internal neutral only.
    world.route=replace(world.route,peer_hash=99,host_control=None)
    world.feed()
    request=world.channel.next_packet(world.now,world.sequence.next)
    assert request is not None
    assert not world.channel.can_send(request.seq,world.now)
    assert store.active is not None
    assert world.channel.sent_count==(0 if action=='takeoff' else 1)


def test_unexpected_motor_activity_is_persisted_without_automatic_commands(rig):
    world,controller,store,_=rig()
    controller.handle('open')
    world.now+=.01;world.phase=8;world.feed();controller.tick()
    assert store.active['reason']=='airborne_observed'
    assert controller.flight_started and controller.state=='fault'
    assert controller.status()['recovery']['required']
    assert world.channel.action is world.channel.control is None
    world.now+=.01;world.phase=0;world.feed();controller.tick()
    assert store.active is not None and controller.flight_started
    attest(controller)
    assert store.active is None and not controller.flight_started


def test_unexpected_activity_is_persisted_for_its_original_atomic_aircraft_context(rig,monkeypatch):
    world,controller,store,_=rig()
    controller.handle('open')
    world.now+=.01;world.phase=8;world.feed()
    snapshot=world.backend.observation_context
    def changed_peer_after_read():
        sample,context=snapshot()
        world.route=replace(world.route,peer_hash=99,host_control=None)
        world.phase=0;world.feed()
        return sample,context
    monkeypatch.setattr(world.backend,'observation_context',changed_peer_after_read)
    controller.tick()
    assert store.active['peer_hash']!=99 and store.active['reason']=='airborne_observed'
    assert controller.flight_started and controller.recovery_required
    with pytest.raises(IntentRejected,match='incident_peer_mismatch'):controller.handle('land')
    assert world.channel.action is None


def test_activity_during_durable_ground_resolution_creates_a_new_incident(rig,monkeypatch):
    world,controller,store,_=rig(active=True)
    controller.handle('open')
    original=store.active['incident_id']
    resolve=store.resolve_ground
    def motor_activity(*args):
        resolve(*args)
        world.now+=.01;world.phase=8;world.feed()
    monkeypatch.setattr(store,'resolve_ground',motor_activity)
    attest(controller)
    assert store.active['incident_id']!=original
    assert store.active['reason']=='airborne_observed'
    assert controller.flight_started and controller.recovery_required
    assert not controller.landing_confirmed and controller.lease is None
    assert not controller.land_acknowledged and controller.land_route_key is None
    assert world.channel.action is world.channel.control is None


def test_ground_sample_before_the_latest_action_cannot_clear_the_durable_incident(rig):
    world,controller,store,_=rig(active=True)
    controller.handle('open');controller.handle('land')
    world.send();world.send();controller.tick()
    with pytest.raises(IntentRejected,match='ground_observation_precedes_last_action'):attest(controller)
    assert store.active is not None and controller.flight_started
    world.now+=.01;world.feed();controller.tick();attest(controller)
    assert store.active is None and not controller.flight_started


@pytest.mark.parametrize('change',['lease-expired','motors-started'])
def test_storage_work_cannot_outlive_authority_or_reuse_an_old_stationary_sample(rig,monkeypatch,change):
    world,controller,store,_=rig()
    controller.handle('open');attest(controller)
    mark=store.mark_airborne
    def persist(*args,**kwargs):
        value=mark(*args,**kwargs)
        world.now+=.1
        if change=='lease-expired':controller.lease_deadline=world.now-.01
        else:world.phase=8
        world.feed()
        return value
    monkeypatch.setattr(store,'mark_airborne',persist)
    with pytest.raises(IntentRejected):queue_takeoff(world,controller)
    assert controller.flight_started and controller.recovery_required and controller.lease is None
    assert store.active is not None and world.channel.action is None and world.channel.sent_count==0


def test_real_dead_owner_is_replaced_inert_then_recovers_on_loopback_only(tmp_path,monkeypatch):
    with ControllerPeer(FlightChannel(),observed_action_replies=True) as peer:
        directory=tmp_path/'incidents'
        with FlightIncidentStore(directory,ADDRESS) as store:
            store.mark_airborne(hashlib.sha256(PEER).hexdigest(),peer.oracle.read.peer_hash)
        owner=Owner(tmp_path,monkeypatch,peer)
        old=owner.runtime
        old_epoch=old.epoch
        try:
            assert old.status()['possible_airborne']
            with pytest.raises(IntentRejected,match='still_running'):old.recover_owner()
            old.process.kill();old.process.wait(timeout=2) # This child has never opened any connection.
            assert old.status()['controller_owner']['restart_available']
            owner.runtime=old.recover_owner()
            new=owner.runtime
            status=new.status()
            assert new.epoch!=old_epoch and new.journal_path!=old.journal_path
            assert status['state']=='disconnected' and status['possible_airborne']
            assert not status['connection']['worker_alive'] and not peer.flight_frames
            new.handle('open')
            owner.drive(lambda s:s['connected'] and s['recovery']['peer_match'] is True,renew=False)
            with pytest.raises(IntentRejected):new.handle('prepare',{'operation':'arm'})
            with pytest.raises(IntentRejected):new.camera_command('recenter',{})
            new.handle('land')
            land_at=max(e['time'] for e in new.status()['events'] if e['event']=='action_intent')
            owner.drive(lambda s:any(e['event']=='action_response' and e.get('request_action')=='land'
                                    for e in s['events']),renew=False)
            ack_at=max(e['time'] for e in new.status()['events'] if e['event']=='action_response')
            owner.drive(lambda s:s['telemetry'] and s['telemetry']['motors_on'] is False
                and s['telemetry']['motion_received_at']>max(land_at,ack_at)
                and peer.phase=='ground' and peer.complete_at is None,renew=False)
            assert new.status()['possible_airborne']
            owner.ground()
            assert not new.status()['possible_airborne']
            assert [p.payload for p in peer.flight_frames if (p.cmd_set,p.cmd_id)==(3,0x2a)]==[b'\2']
            new.handle('close');new.close()
            assert new.cleanup_verified
            with FlightIncidentStore(directory,ADDRESS) as store:assert store.active is None
        finally:owner.cleanup()


def test_real_owner_holds_incident_exclusion_even_while_disconnected(tmp_path):
    config=FlightConfig('fixture0',tmp_path/'wlm.json',tmp_path/'ble.json',
        capture_directory=tmp_path/'sessions',incident_directory=tmp_path/'incidents')
    first=ProcessControllerRuntime(tmp_path/'first.jsonl',mode='flight',flight_config=config)
    try:
        with pytest.raises(OwnerUnavailable):
            ProcessControllerRuntime(tmp_path/'second.jsonl',mode='flight',flight_config=config)
        assert first.status()['state']=='disconnected' and first.status()['controller_owner']['available']
        assert not (tmp_path/'sessions').exists()
    finally:first.close()


def test_corrupt_durable_state_refuses_real_owner_start_without_replacing_evidence(tmp_path):
    directory=tmp_path/'incidents'
    with FlightIncidentStore(directory,ADDRESS) as store:
        store.mark_airborne('a'*64,99)
        path=directory/store.filename
    path.write_bytes(b'{corrupt-but-preserved')
    config=FlightConfig('fixture0',tmp_path/'wlm.json',tmp_path/'ble.json',
        capture_directory=tmp_path/'sessions',incident_directory=directory)
    with pytest.raises(OwnerUnavailable):
        ProcessControllerRuntime(tmp_path/'failed.jsonl',mode='flight',flight_config=config)
    assert path.read_bytes()==b'{corrupt-but-preserved'
    assert not (tmp_path/'sessions').exists() and not (tmp_path/'failed.jsonl').read_text()


def test_orphaned_wifi_profile_is_an_explicit_boundary_without_ble_or_global_cleanup(tmp_path):
    from test_flight_connection import Harness,wait_for
    h=Harness(tmp_path)
    def occupied(*args,**kwargs):raise RuntimeError('interface_must_be_disconnected_wifi')
    h.backend.connection.route_guard=occupied
    h.backend.connection_options['route_guard']=occupied
    with FlightIncidentStore(tmp_path/'incidents',h.config.ble_address) as store:
        original=store.mark_airborne('a'*64,99)
        journal=DecisionJournal(tmp_path/'boundary.jsonl')
        controller=ControlSupervisor(journal,mode='flight',backend=h.backend,incident_store=store)
        try:
            controller.handle('open')
            wait_for(lambda:not h.backend.connection.status()['worker_alive'])
            status=controller.status()
            assert status['connection']['error']=='interface_must_be_disconnected_wifi'
            assert status['possible_airborne'] and status['recovery']['required']
            assert status['recovery']['connection_retry_available']
            assert store.active==original and not h.trace and not h.sessions
            old_channel=h.backend.channel
            h.backend.connection.cleanup['wifi']=False
            assert not controller.status()['recovery']['connection_retry_available']
            with pytest.raises(IntentRejected,match='cleanup_unconfirmed'):controller.handle('reconnect')
            assert h.backend.channel is old_channel
            h.backend.connection.cleanup['wifi']=True
            controller.handle('reconnect')
            wait_for(lambda:not h.backend.connection.status()['worker_alive'])
            assert h.backend.channel is not old_channel and old_channel.snapshot()['revoked']
            assert store.active==original and not h.trace and not h.sessions
            assert controller.flight_started and controller.lease is None
        finally:
            h.backend.close(h.backend.clock());h.backend.finish_cleanup(timeout=2);journal.close()
