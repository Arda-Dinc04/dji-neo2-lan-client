"""Original PC deadlines across controller locks, camera stages and real IPC."""
import threading
import time
from types import SimpleNamespace

import pytest

from neo2_control.camera import CameraChannel
from neo2_control.command_ingress import CommandIngress
from neo2_control.control_owner import ProcessControllerRuntime, _receive, _send
from neo2_control.controller import ControllerMode, IntentRejected
from neo2_control.duml import DumlSequence
from neo2_control.flight_connection import FlightConfig, FlightConnection
from neo2_control.flight_channel import FlightChannel
from neo2_control.flight_incident import FlightIncidentStore
from test_camera_channel import reply, state_push
from test_control_owner import owner, start_raw_owner
from test_controller import factory, authorize, airborne
from test_vertical_trial import rig as vertical_rig

EPOCH = 'a' * 48
GENERATION = 'b' * 24


def context(now, *, sequence=1, generation=None, epoch=EPOCH, duration=.25):
    return CommandIngress(epoch, generation, sequence, now, now+duration)


def test_command_context_is_identity_only_and_never_restamps_status_or_authority():
    runtime=object.__new__(ProcessControllerRuntime)
    runtime.lock=threading.Lock();runtime.epoch=EPOCH
    runtime.last_status={'connection':{'generation':GENERATION},
        'telemetry':{'motion_received_at':1.},'lease_expires_at':2.}
    runtime.last_response=3.
    assert runtime.command_context()=={'owner_epoch':EPOCH,'connection_generation':GENERATION}
    assert runtime.last_response==3. and runtime.last_status['lease_expires_at']==2.
    assert runtime.last_status['telemetry']['motion_received_at']==1.


def test_controller_rechecks_after_waiting_for_its_lock(factory):
    controller, backend, clock, _ = factory()
    ingress = context(clock.now)
    waiting = threading.Event()
    underlying = threading.RLock()
    errors = []

    class Lock:
        def __enter__(self):
            if threading.current_thread().name == 'expired-intent':waiting.set()
            underlying.acquire()
        def __exit__(self, *args):underlying.release()

    controller.lock = Lock()
    def call():
        try:controller.handle('open', ingress=ingress)
        except IntentRejected as error:errors.append(str(error))
    with controller.lock:
        thread = threading.Thread(target=call, name='expired-intent')
        thread.start()
        assert waiting.wait(1)
        clock.now += .3
    thread.join(1)
    assert not thread.is_alive() and errors == ['operator_request_expired']
    assert not backend.connected and not backend.sent


def test_journal_delay_cannot_renew_a_lease(factory, monkeypatch):
    controller, backend, clock, journal = factory()
    lease = authorize(controller)
    original = controller.lease_deadline
    ingress = context(clock.now)
    record = journal.record
    def delayed(value):
        record(value)
        if value['event'] == 'intent_received':clock.now += .3
    monkeypatch.setattr(journal, 'record', delayed)
    with pytest.raises(IntentRejected, match='operator_request_expired'):
        controller.handle('keepalive', {'lease':lease}, ingress=ingress)
    assert controller.lease_deadline == original and not backend.sent


def test_arm_and_keepalive_use_issue_time_instead_of_delayed_arrival(factory):
    controller, _, clock, _ = factory()
    controller.handle('open')
    token = controller.handle('prepare', {'operation':'arm'})['confirmation_token']
    issued = clock.now
    ingress = context(issued)
    clock.now += .2
    lease = controller.handle('arm', {'confirmation_token':token}, ingress=ingress)['lease']
    assert controller.lease_deadline == issued + controller.limits.lease_seconds
    issued = clock.now
    ingress = context(issued, sequence=2)
    clock.now += .2
    controller.handle('keepalive', {'lease':lease}, ingress=ingress)
    assert controller.lease_deadline == issued + controller.limits.lease_seconds


def test_control_preserves_original_deadline_and_issue_time(factory):
    controller, backend, clock, _, lease = airborne(factory)
    controller.handle('keepalive', {'lease':lease})
    issued = clock.now
    ingress = context(issued)
    clock.now += .2
    controller.handle('control', {'lease':lease, 'sequence':1,
        'expires_at':clock.now+.2, 'channels':[50,0,0,0]}, ingress=ingress)
    assert controller.control[1] == ingress.expires_at
    assert controller.lease_deadline == issued + controller.limits.lease_seconds
    assert backend.channels != (0,0,0,0)


def test_changed_backend_generation_rejects_before_authority_mutation(factory):
    controller, backend, clock, _ = factory()
    backend.connection_status = lambda:{'generation':'c'*24}
    with pytest.raises(IntentRejected, match='operator_request_generation_changed'):
        controller.handle('open', ingress=context(clock.now, generation=GENERATION))
    assert not backend.connected


def test_packet_journal_delay_cannot_emit_an_expired_takeoff(factory, monkeypatch):
    controller, backend, clock, journal = factory()
    lease = authorize(controller)
    token = controller.handle('prepare', {'operation':'takeoff','lease':lease})['confirmation_token']
    ingress = context(clock.now)
    record = journal.record
    def delayed(value):
        record(value)
        if value['event'] == 'packet_intent':clock.now += .3
    monkeypatch.setattr(journal, 'record', delayed)
    with pytest.raises(IntentRejected, match='operator_request_expired'):
        controller.handle('takeoff', {'lease':lease,'confirmation_token':token}, ingress=ingress)
    assert not backend.sent


def test_incident_fsync_delay_keeps_incident_but_emits_no_takeoff(vertical_rig, tmp_path, monkeypatch):
    world, controller, _ = vertical_rig()
    token = controller.handle('prepare', {'operation':'arm'})['confirmation_token']
    lease = controller.handle('arm', {'confirmation_token':token})['lease']
    token = controller.handle('prepare', {'operation':'takeoff','lease':lease})['confirmation_token']
    ingress = context(world.now)
    with FlightIncidentStore(tmp_path/'persistent', '00:11:22:33:44:55') as store:
        controller.incident_store = store
        mark = store.mark_airborne
        def delayed(*args, **kwargs):
            result = mark(*args, **kwargs)
            world.now += .3
            return result
        monkeypatch.setattr(store, 'mark_airborne', delayed)
        with pytest.raises(IntentRejected, match='operator_request_expired'):
            controller.handle('takeoff', {'confirmation_token':token,'lease':lease}, ingress=ingress)
        assert store.active and controller.recovery_required and controller.flight_started
        assert controller.lease is None and controller.state == 'fault'
        assert world.channel.action is None and world.channel.sent_count == 0


def test_operator_action_queue_expires_but_autonomous_landing_uses_its_own_deadline(vertical_rig):
    world, controller, _ = vertical_rig()
    token = controller.handle('prepare', {'operation':'arm'})['confirmation_token']
    lease = controller.handle('arm', {'confirmation_token':token})['lease']
    token = controller.handle('prepare', {'operation':'takeoff','lease':lease})['confirmation_token']
    ingress = context(world.now)
    world.now += .15
    world.feed()
    controller.handle('takeoff', {'confirmation_token':token,'lease':lease}, ingress=ingress)
    assert world.channel.action['expires_at'] == ingress.expires_at
    world.now = ingress.expires_at
    assert world.channel.next_packet(world.now, world.sequence.next) is None
    controller._failsafe(world.now, 'synthetic_operator_loss')
    assert world.channel.action['action'] == 'land'
    assert world.channel.action['expires_at'] == world.now + .25


def test_camera_mode_ack_does_not_extend_original_photo_deadline():
    now = [100.]
    camera = CameraChannel(clock=lambda:now[0])
    camera.set_ready(True, now[0])
    ingress = context(now[0])
    camera.submit('photo', {}, ingress=ingress)
    sequence = DumlSequence()
    mode = camera.next_packet(now[0], sequence.next)
    now[0] += .01
    camera.sent(mode.seq, now[0])
    now[0] += .05
    camera.receive(reply(mode), now[0])
    now[0] += .05
    camera.receive(state_push(mode=0), now[0])
    assert camera.pending['deadline'] == ingress.expires_at
    assert camera.pending['overall_deadline'] == ingress.expires_at
    now[0] = ingress.expires_at
    assert camera.next_packet(now[0], sequence.next) is None
    assert camera.sent_count == 1 and camera.snapshot()['last_command']['state'] == 'timed_out'


def test_camera_final_guard_rejects_already_scheduled_expired_packet():
    now = [100.]
    camera = CameraChannel(clock=lambda:now[0])
    camera.set_ready(True, now[0])
    ingress = context(now[0])
    camera.submit('recenter', {}, ingress=ingress)
    packet = camera.next_packet(now[0], DumlSequence().next)
    assert camera.can_send(packet.seq, now[0])
    now[0] = ingress.expires_at
    assert not camera.can_send(packet.seq, now[0])
    assert camera.sent_count == 0 and camera.pending is None


def test_camera_checks_the_clock_after_its_lock():
    now = [100.]
    camera = CameraChannel(clock=lambda:now[0])
    camera.set_ready(True, now[0])
    ingress = context(now[0])
    class Lock:
        def __enter__(self):now[0] += .3
        def __exit__(self, *args):pass
    camera.lock = Lock()
    with pytest.raises(ValueError, match='operator_request_expired'):
        camera.submit('recenter', {}, ingress=ingress)
    assert camera.pending is None


def test_connection_camera_admission_is_bound_to_actual_generation(tmp_path):
    config = FlightConfig('fixture0', tmp_path/'wlm', tmp_path/'ble', camera_preview=True)
    channel = FlightChannel(clock=lambda:100.)
    channel.enable_observations(synthetic=True)
    connection = FlightConnection(config, channel, clock=lambda:100.)
    connection.phase, connection.generation = 'session_open', GENERATION
    connection.camera.set_ready(True,100.)
    with pytest.raises(ValueError, match='operator_request_generation_changed'):
        connection.camera_command('recenter', {}, ingress=context(100., generation='c'*24))
    assert connection.camera.pending is None and connection.thread is None


def test_real_ipc_expired_and_replayed_operator_contexts_leave_owner_alive(tmp_path):
    channel, process, ready = start_raw_owner(tmp_path)
    identity = ready['epoch']
    identifier = 0
    def send(action, ingress, *, operation='action'):
        nonlocal identifier
        identifier += 1
        _send(channel, {'kind':'request','epoch':identity,'id':identifier,
            'expires_at':time.monotonic()+.25, 'operation':operation,
            'parameters':{'action':action,'parameters':{},'ingress':ingress.as_dict()}})
        return _receive(channel)
    try:
        expired = context(time.monotonic()-1, epoch=identity)
        response = send('open', expired)
        assert response['error'] == 'operator_request_expired' and not response['status']['connected']
        assert process.poll() is None
        valid = context(time.monotonic(), epoch=identity)
        assert send('open', valid)['error'] is None
        assert send('prepare', valid)['error'] == 'command_mutation_sequence_replayed'
        # A refused camera request still consumes the shared mutation order.
        camera = context(time.monotonic(), epoch=identity, sequence=2)
        assert send('recenter', camera, operation='camera')['error'] == 'invalid_camera_request'
        assert send('prepare', camera)['error'] == 'command_mutation_sequence_replayed'
        prepare = context(time.monotonic(), epoch=identity, sequence=3)
        identifier += 1
        _send(channel, {'kind':'request','epoch':identity,'id':identifier,
            'expires_at':time.monotonic()+.25, 'operation':'action',
            'parameters':{'action':'prepare','parameters':{'operation':'arm'},'ingress':prepare.as_dict()}})
        assert _receive(channel)['result']['confirmation_token']
        assert process.poll() is None
    finally:
        identifier += 1
        if process.poll() is None:
            _send(channel, {'kind':'request','epoch':identity,'id':identifier,
                'expires_at':time.monotonic()+.25,'operation':'shutdown','parameters':{}})
            process.wait(timeout=3)
        channel.close()
        if process.poll() is None:process.kill()
        process.communicate(timeout=2)


def test_parent_lock_wait_expiry_does_not_detach_real_owner(tmp_path):
    with owner(tmp_path) as runtime:
        ingress = context(time.monotonic(), epoch=runtime.epoch, duration=.04)
        errors = []
        def call():
            try:runtime.handle('open', ingress=ingress)
            except IntentRejected as error:errors.append(str(error))
        with runtime.lock:
            thread = threading.Thread(target=call)
            thread.start()
            time.sleep(.08)
        thread.join(1)
        assert not thread.is_alive() and errors == ['operator_request_expired']
        assert runtime.failure is None and runtime.process.poll() is None
        runtime.handle('open')
        assert runtime.status()['connected']


def test_owner_replacement_wait_revalidates_before_creating_a_process(tmp_path, monkeypatch):
    import neo2_control.control_owner as module
    now = [100.]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda:now[0]))
    runtime = object.__new__(ProcessControllerRuntime)
    runtime.epoch, runtime.last_status = EPOCH, {'connection':{'generation':None}}
    runtime.mode, runtime.replacement_lock = ControllerMode.FLIGHT, threading.Lock()
    runtime.process = SimpleNamespace(poll=lambda:1)
    runtime.stopped = threading.Event()
    runtime.thread = SimpleNamespace(join=lambda **kwargs:now.__setitem__(0,100.3), is_alive=lambda:False)
    closed = []
    runtime.channel = SimpleNamespace(close=lambda:closed.append(True))
    runtime.journal_path = tmp_path/'journal'
    with pytest.raises(IntentRejected, match='operator_request_expired'):
        runtime.recover_owner(ingress=context(100.))
    assert not closed and not runtime.journal_path.exists()


def test_original_context_drives_real_child_and_udp_loopback_without_expired_axis(tmp_path, monkeypatch):
    from test_control_transport_session import ControllerPeer
    from test_flight_owner import Owner
    with ControllerPeer(FlightChannel()) as peer:
        owned = Owner(tmp_path, monkeypatch, peer)
        try:
            owned.takeoff()
            state = owned.runtime.status()
            ingress = context(time.monotonic(), epoch=owned.runtime.epoch,
                generation=state['connection']['generation'])
            owned.runtime.handle('control', {'lease':owned.lease,'sequence':1,
                'channels':[50,0,0,0],'expires_at':time.monotonic()+.2}, ingress=ingress)
            owned.drive(lambda s:(1057,1024,1024,1024) in peer.controls, timeout=1)
            stale = context(time.monotonic()-1, epoch=owned.runtime.epoch, sequence=2,
                generation=state['connection']['generation'])
            with pytest.raises(IntentRejected, match='operator_request_expired'):
                owned.runtime.handle('control', {'lease':owned.lease,'sequence':2,
                    'channels':[250,0,0,0],'expires_at':time.monotonic()+.2}, ingress=stale)
            assert not any(channels[0] == 1189 for channels in peer.controls)
            assert owned.runtime.failure is None and owned.runtime.process.poll() is None
            landing = context(time.monotonic(), epoch=owned.runtime.epoch, sequence=2,
                generation=state['connection']['generation'])
            owned.runtime.handle('land', ingress=landing)
            owned.drive(lambda s:s['telemetry']['motors_on'] is False and any(
                e['event']=='action_response' and e.get('request_action')=='land' for e in s['events']))
            owned.ground()
            owned.drive(lambda s:s['state']=='ready')
            assert [packet.payload for packet in peer.flight_frames
                if (packet.cmd_set,packet.cmd_id)==(3,0x2a)] == [b'\1',b'\2']
        finally:owned.cleanup()
