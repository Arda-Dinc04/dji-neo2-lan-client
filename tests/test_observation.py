from contextlib import contextmanager
import json
from pathlib import Path
import socket
import struct
import threading
import time
from types import SimpleNamespace

import pytest

from neo2_control.controller import ControlSupervisor, DecisionJournal, IntentRejected
from neo2_control.duml import DumlPacket
from neo2_control.observation import ManagedObservation, ObservationConfig, ObservationError
from neo2_control.session import run_session
from neo2_control.telemetry import TelemetryReceiver
from neo2_control.transport import _build_datagram
from neo2_control.wifi_link import WifiLinkError
from test_session import acceptance, status
from test_telemetry import body, frame


def until(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        threading.Event().wait(.01)
    assert predicate(), 'condition did not become true within the bound'


class LoopbackDrone:
    """Actual PC UDP endpoint; all FC fields and video bytes are synthetic."""
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 0))
        self.socket.settimeout(.025)
        self.port = self.socket.getsockname()[1]
        self.stop = threading.Event()
        self.requests, self.errors = [], []
        self.telemetry = self.respond = True
        self.osd = frame(body(flags=8))
        self.video = b'synthetic-video'
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        session = anchor = acknowledged = None
        try:
            while not self.stop.is_set():
                try:
                    raw, peer = self.socket.recvfrom(65535)
                except socket.timeout:
                    continue
                self.requests.append(raw)
                if not self.respond:
                    continue
                if raw[6] == 0:
                    session = int.from_bytes(raw[2:4], 'little')
                    anchor = acknowledged = int.from_bytes(raw[8:10], 'little')
                    self.socket.sendto(acceptance(session), peer)
                    end = (anchor + 8) & 0xFFFF
                    payload = struct.pack('<HHHHI', anchor, end, 0, 0, 0x101) + self.video
                    self.socket.sendto(_build_datagram(session, 2, payload, send_sequence=end), peer)
                elif session is not None:
                    if raw[6] == 5:
                        acknowledged = int.from_bytes(raw[4:6], 'little')
                    packet = bytearray(status(session, ack=acknowledged, anchor=anchor,
                                              payload=self.osd if self.telemetry else b''))
                    packet[10:12] = ((anchor + 8) & 0xFFFF).to_bytes(2, 'little')
                    self.socket.sendto(bytes(packet), peer)
        except BaseException as error:
            self.errors.append(error)

    def runner(self, **options):
        # Only this fixture redirects the fixed production peer to loopback.
        options.update(host='127.0.0.1', port=self.port, interface=None)
        return run_session(**options)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        self.socket.close()
        assert not self.thread.is_alive() and not self.errors


@pytest.fixture
def drone():
    peer = LoopbackDrone()
    yield peer
    peer.close()


@contextmanager
def observed(tmp_path, drone, **options):
    journal = DecisionJournal(tmp_path / 'controller.jsonl')
    controller = ControlSupervisor(journal)
    manager = ManagedObservation(ObservationConfig('testwifi0', maximum_duration=10, **options),
        tmp_path / 'captures', route_guard=lambda interface: None, session_runner=drone.runner,
        event_sink=controller.observation_event)
    controller.attach_observation_source(manager.snapshot)
    try:
        yield controller, manager
    finally:
        manager.shutdown()
        journal.close()


def test_actual_udp_worker_updates_observe_controller_records_video_and_cancels(tmp_path, drone):
    with observed(tmp_path, drone, record_video=True) as (controller, manager):
        manager.open({'duration': 5})
        until(lambda: (manager.snapshot().get('telemetry') or {}).get('available'))
        state = controller.status()
        assert state['mode'] == 'observe' and state['state'] == 'observing' and state['connected']
        assert not state['synthetic'] and not state['flight_ready'] and not state['application_ready']
        assert state['telemetry']['fields']['motors_on']['value'] is True
        assert state['telemetry']['fields']['is_flying']['value'] is False
        assert state['possible_airborne'] is None and not state['landing_confirmed']
        assert set(state['session']['receive_windows']) == {'2', '3'}
        with pytest.raises(IntentRejected, match='observe_mode'):
            controller.handle('takeoff')
        closing = manager.close()
        assert closing['observation']['stop_requested'] and not closing['connected']
        assert not closing['telemetry']['fields']['motors_on']['valid']
        until(lambda: not manager.busy)
        final = manager.snapshot()
        assert final['state'] == 'closed' and final['session']['reason'] == 'requested'
        assert final['session']['video_bytes_delivered'] == len(drone.video)
        paths = final['observation']
        assert Path(paths['video_path']).read_bytes() == drone.video
        for field in ('capture_path', 'journal_path', 'video_path'):
            assert Path(paths[field]).stat().st_mode & 0o777 == 0o600
        assert not any(packet[6] == 5 for packet in drone.requests)
        assert sum(packet[6] == 0 for packet in drone.requests) == 1
        assert manager.result['reason'] == 'requested'


def test_telemetry_ages_while_link_remains_active_and_no_telemetry_arrives(tmp_path, drone):
    with observed(tmp_path, drone) as (_, manager):
        manager.open({'duration': 5})
        until(lambda: (manager.snapshot().get('telemetry') or {}).get('available'))
        drone.telemetry = False
        until(lambda: not manager.snapshot()['telemetry']['fresh'], timeout=2)
        state = manager.snapshot()
        assert state['connected'] and state['observation']['worker_alive']
        field = state['telemetry']['fields']['motors_on']
        assert field['age_seconds'] > 1 and field['reason'] == 'stale' and not field['valid']


def test_preflight_cancellation_prevents_runner_and_cannot_overlap_a_second_worker(tmp_path):
    entered, release = threading.Event(), threading.Event()
    calls = []
    def guard(interface):
        entered.set()
        assert release.wait(2)
    manager = ManagedObservation(ObservationConfig('test0'), tmp_path,
        route_guard=guard, session_runner=lambda **kw: calls.append(kw))
    try:
        manager.open()
        assert entered.wait(1)
        with pytest.raises(ObservationError, match='already_running'):
            manager.open()
        manager.close()
        assert manager.snapshot()['state'] == 'stopping'
        release.set()
        until(lambda: not manager.busy)
        assert not calls and not list(tmp_path.iterdir())
        assert manager.snapshot()['state'] == 'closed'
    finally:
        release.set()
        manager.shutdown()


def test_rejected_route_never_creates_files_or_opens_udp(tmp_path):
    def guard(interface):
        raise WifiLinkError('interface_carries_default_route')
    manager = ManagedObservation(ObservationConfig('test0'), tmp_path, route_guard=guard,
        session_runner=lambda **kw: pytest.fail('route failure must precede packets'))
    manager.open()
    until(lambda: not manager.busy)
    status = manager.snapshot()
    assert status['state'] == 'failed'
    assert status['observation']['error'] == 'interface_carries_default_route'
    assert not status['connected'] and not list(tmp_path.iterdir())
    manager.shutdown()


def test_remote_parameters_cannot_change_peer_interface_or_paths(tmp_path):
    manager = ManagedObservation(ObservationConfig('test0', maximum_duration=10), tmp_path)
    for parameters in ({'host': '127.0.0.1'}, {'interface': 'lo'}, {'capture_path': '/tmp/x'},
                       {'duration': True}, {'duration': 11}, {'duration': float('nan')}, []):
        with pytest.raises(ValueError):
            manager.open(parameters)
    assert not list(tmp_path.iterdir())


def test_readers_age_detached_snapshot_without_a_callback_and_old_generations_cannot_publish(tmp_path):
    clock = [10.0]
    entered = threading.Event()
    def runner(**options):
        snapshot = {'state': 'open', 'observed_at': 10., 'last_received_age_seconds': 0.,
                    'remote_timeout_seconds': 20., 'telemetry': {
                        'available': True, 'age_seconds': 0., 'maximum_age_seconds': 1.,
                        'fields': {'motors_on': {'value': True, 'valid': True, 'age_seconds': 0., 'reason': 'decoded'}}}}
        options['snapshot_sink'](snapshot)
        entered.set()
        options['stop_event'].wait(2)
        return SimpleNamespace(state='closed', reason='requested', as_dict=lambda: {})
    manager = ManagedObservation(ObservationConfig('test0'), tmp_path,
        route_guard=lambda interface: None, session_runner=runner, clock=lambda: clock[0])
    try:
        manager.open()
        assert entered.wait(1)
        before = manager.snapshot()
        before['telemetry']['fields']['motors_on']['value'] = False
        clock[0] = 12.
        after = manager.snapshot()
        assert after['connected'] and not after['telemetry']['fresh']
        assert after['telemetry']['fields']['motors_on']['value'] is True
        assert after['telemetry']['fields']['motors_on']['age_seconds'] == 2
        manager._publish('old-session', {'state': 'failed'})
        assert manager.snapshot()['state'] == 'observing'
    finally:
        manager.shutdown()


def test_fc_route_observations_age_independently_without_merging_or_refreshing_on_reads(tmp_path):
    receiver = TelemetryReceiver(maximum_age=1)
    attributes = struct.pack('<IBIIBB7x', 0x12345678, 2, 1, 0xffffffff, 1, 0x82)
    for recipient, received_at in ((0x0e, 10.), (0xaa, 10.8)):
        inner = frame(receiver=recipient, command_type=0)
        receiver.feed(DumlPacket(0x27, 0xee, 1, 0, 0x51, 1, inner+attributes).encode(), received_at)
    source = receiver.snapshot(10.8)
    clock, entered = [10.8], threading.Event()
    def runner(**options):
        options['snapshot_sink']({'state': 'open', 'observed_at': 10.8,
            'last_received_age_seconds': 0., 'remote_timeout_seconds': 20., 'telemetry': source})
        entered.set()
        options['stop_event'].wait(2)
        return SimpleNamespace(state='closed', reason='requested', as_dict=lambda: {})
    manager = ManagedObservation(ObservationConfig('test0'), tmp_path,
        route_guard=lambda interface: None, session_runner=runner, clock=lambda: clock[0])
    try:
        manager.open()
        assert entered.wait(1)
        clock[0] = 11.2
        snapshot = manager.snapshot()
        assert snapshot['connected'] and snapshot['telemetry']['selection'] == 'last_received'
        old, current = snapshot['telemetry']['observations']
        assert old['source']['receiver'] == 0x0e and current['source']['receiver'] == 0xaa
        assert old['age_seconds'] == pytest.approx(1.2) and not old['fresh']
        assert old['fields']['motors_on']['reason'] == 'stale'
        assert not old['fields']['motors_on']['valid']
        assert current['age_seconds'] == pytest.approx(.4) and current['fresh']
        assert current['fields']['motors_on']['valid']
        assert snapshot['telemetry']['source']['receiver'] == 0xaa
        assert snapshot['telemetry']['age_seconds'] == pytest.approx(.4)
        current['fields']['motors_on']['value'] = False
        again = manager.snapshot()['telemetry']
        assert again['observations'][1]['fields']['motors_on']['value'] is True
        assert again['observations'][0]['age_seconds'] == pytest.approx(1.2)
        assert again['observations'][1]['fields']['motors_on']['age_seconds'] == pytest.approx(.4)
        assert source['observations'][0]['age_seconds'] == pytest.approx(.8)
        closing = manager.close()['telemetry']
        assert not closing['session_active'] and not closing['flight_ready']
        for sample in closing['observations']:
            assert not sample['session_active'] and not sample['flight_ready']
            assert not sample['fields']['motors_on']['valid']
            assert sample['fields']['motors_on']['reason'] == 'session_inactive'
    finally:
        manager.shutdown()


def test_worker_errors_are_sanitized_and_late_callback_cannot_make_terminal_worker_connected(tmp_path):
    def runner(**options):
        options['snapshot_sink']({'state': 'open', 'observed_at': time.monotonic(),
                                 'last_received_age_seconds': 0., 'remote_timeout_seconds': 2.})
        raise OSError('private backend message must not escape')
    manager = ManagedObservation(ObservationConfig('test0'), tmp_path,
        route_guard=lambda interface: None, session_runner=runner)
    manager.open()
    until(lambda: not manager.busy)
    status = manager.snapshot()
    assert status['state'] == 'failed' and not status['connected']
    assert status['observation']['error'] == 'observation_io_error'
    assert 'private backend message' not in json.dumps(status)
    manager.shutdown()


def test_session_precancel_and_snapshot_failure_stop_before_application_packets(tmp_path, drone):
    stop = threading.Event()
    stop.set()
    result = run_session(capture_path=tmp_path / 'cancel.jsonl', host='127.0.0.1', port=drone.port,
                         duration=2, heartbeat=True, stop_event=stop)
    assert result.reason == 'requested' and result.transmitted == 0 and not drone.requests
    def failure(snapshot):
        raise RuntimeError('unavailable consumer')
    result = run_session(capture_path=tmp_path / 'sink.jsonl', host='127.0.0.1', port=drone.port,
                         duration=2, heartbeat=True, snapshot_sink=failure)
    assert result.state == 'failed' and result.reason == 'snapshot_consumer_failed'
    assert result.heartbeats_sent == 0 and result.transmitted == 1


def test_terminal_snapshot_failure_is_reported_instead_of_returning_false_success(tmp_path, drone):
    def failure(snapshot):
        if snapshot['state'] == 'closed':
            raise RuntimeError('unavailable consumer')
    with pytest.raises(RuntimeError, match='snapshot_consumer_failed'):
        run_session(capture_path=tmp_path / 'final-sink.jsonl', host='127.0.0.1', port=drone.port,
                    duration=.1, snapshot_sink=failure)


def test_heartbeat_is_a_locally_selected_option_and_uses_only_existing_nonflight_codec(tmp_path, drone):
    from neo2_control.duml import DumlPacket
    with observed(tmp_path, drone, heartbeat=True) as (_, manager):
        manager.open({'duration': 3})
        until(lambda: manager.snapshot()['session'].get('heartbeats_acknowledged', 0) > 0)
        manager.close()
        until(lambda: not manager.busy)
        packets = [DumlPacket.decode(raw[20:]) for raw in drone.requests if raw[6] == 5]
        assert packets and all((packet.cmd_set, packet.cmd_id, packet.payload) == (0, 0, b'\x00\x02\x01\x00') for packet in packets)


def test_unavailable_thread_does_not_leave_a_phantom_running_session(tmp_path, monkeypatch):
    def failure(thread):
        raise RuntimeError('no threads available')
    monkeypatch.setattr(threading.Thread, 'start', failure)
    manager = ManagedObservation(ObservationConfig('test0'), tmp_path)
    with pytest.raises(ObservationError, match='thread_start_failed'):
        manager.open()
    assert not manager.busy and manager.snapshot()['state'] == 'failed'
    manager.shutdown()


def test_shutdown_timeout_preserves_live_worker_and_cancellation_still_blocks_later_io(tmp_path):
    entered, release = threading.Event(), threading.Event()
    calls = []
    def guard(interface):
        entered.set()
        assert release.wait(2)
    manager = ManagedObservation(ObservationConfig('test0'), tmp_path, route_guard=guard,
                                 session_runner=lambda **kw: calls.append(kw))
    try:
        manager.open()
        assert entered.wait(1)
        with pytest.raises(ObservationError, match='did_not_stop'):
            manager.shutdown(timeout=.01)
        assert manager.thread.is_alive() and manager.snapshot()['observation']['worker_alive']
        with pytest.raises(ObservationError, match='service_stopping'):
            manager.open()
        release.set()
        until(lambda: not manager.busy)
        assert not calls
    finally:
        release.set()
        manager.shutdown()
