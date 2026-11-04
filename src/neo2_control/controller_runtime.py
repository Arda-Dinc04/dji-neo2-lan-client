"""Local watchdog lifetime and reproducible no-network control scenarios."""

from pathlib import Path
import json
import threading
import time

from .controller import ControllerMode, ControlSupervisor, DecisionJournal
from .simulator import SimulationBackend


class ControllerRuntime:
    def __init__(self, journal_path: str | Path, *, mode='observe'):
        mode = ControllerMode(mode)
        if mode is ControllerMode.FLIGHT:
            raise ValueError('live flight adapter is not verified')
        self.journal = DecisionJournal(journal_path)
        self.backend = SimulationBackend() if mode is ControllerMode.SIMULATE else None
        try:
            self.controller = ControlSupervisor(self.journal, mode=mode, backend=self.backend)
        except Exception:
            self.journal.close()
            raise
        self.stopped = threading.Event()
        self.failure = None
        self.thread = threading.Thread(target=self._run, name='neo2-watchdog', daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stopped.wait(.025):
                self.controller.tick()
        except Exception as error:
            self.failure = type(error).__name__
            self.controller.stop()

    def close(self):
        self.controller.stop()
        # During normal service shutdown the local loop still has time to
        # observe a synthetic landing. This is not a SIGKILL/host-loss guarantee.
        deadline = time.monotonic() + 1.5
        while self.controller.state == 'landing' and self.thread.is_alive() and time.monotonic() < deadline:
            self.stopped.wait(.025)
        self.stopped.set()
        self.thread.join(timeout=1)
        if self.thread.is_alive():
            raise RuntimeError('controller watchdog did not stop')
        self.journal.close()


SCENARIOS = ('normal', 'control-loss', 'ack-loss', 'telemetry-loss', 'link-loss')


def run_simulation(journal_path: str | Path, scenario='normal') -> dict:
    """Run real supervisor intents with virtual time; no sleep/socket/device."""
    if scenario not in SCENARIOS:
        raise ValueError('unknown simulation scenario')
    now = 0.0
    backend = SimulationBackend()
    journal = DecisionJournal(journal_path)
    controller = ControlSupervisor(journal, mode='simulate', backend=backend, clock=lambda: now)
    landing_confirmed = False
    try:
        controller.handle('open')
        confirmation = controller.handle('prepare', {'operation': 'arm'})['confirmation_token']
        lease = controller.handle('arm', {'confirmation_token': confirmation})['lease']
        confirmation = controller.handle('prepare', {'operation': 'takeoff', 'lease': lease})['confirmation_token']
        if scenario == 'ack-loss':
            backend.ack_enabled = False
        controller.handle('takeoff', {'lease': lease, 'confirmation_token': confirmation})
        control_sequence = 0
        next_intent = 0.4
        injected = False
        for step in range(1, 281):
            now = step * .025
            controller.tick()
            if controller.state == 'flying' and now >= next_intent - 1e-9:
                if not injected:
                    if scenario == 'telemetry-loss':
                        backend.telemetry_enabled = False
                    elif scenario == 'link-loss':
                        backend.connected = False
                    injected = True
                if scenario == 'normal' and now >= 1.2:
                    controller.handle('land')
                elif scenario == 'normal' or (scenario == 'control-loss' and control_sequence == 0):
                    control_sequence += 1
                    controller.handle('control', {'lease': lease, 'sequence': control_sequence,
                                                  'expires_at': now + .15, 'channels': [50, 0, 0, 0]})
                elif scenario == 'telemetry-loss':
                    controller.handle('keepalive', {'lease': lease})
                next_intent = now + .1
            if controller.state == 'ready':
                landing_confirmed = True
                controller.handle('close')
                break
            if controller.state == 'fault':
                landing_confirmed = controller.landing_confirmed
                break
        result = controller.status()
        expected_reason = {'normal': None, 'control-loss': 'control_expired',
                           'ack-loss': 'action_ack_timeout', 'telemetry-loss': 'telemetry_stale',
                           'link-loss': 'link_unavailable'}[scenario]
        passed = (result['state'] == ('disconnected' if scenario == 'normal' else 'fault')
                  and result['fault_reason'] == expected_reason
                  and landing_confirmed == (scenario in ('normal', 'control-loss')))
        return {'scenario': scenario, 'mode': 'simulate', 'virtual_duration_seconds': now,
                'scenario_passed': passed,
                'live_flight_enabled': False, 'state': result['state'],
                'fault_reason': result['fault_reason'], 'landing_confirmed': landing_confirmed,
                'journal_healthy': result['journal_healthy'], 'journal_path': str(journal.path),
                'synthetic_packets': len(backend.sent), 'telemetry': result['telemetry'],
                'transitions': [event for event in map(json.loads, journal.path.read_text().splitlines())
                                if event['event'] == 'state_changed']}
    finally:
        journal.close()
