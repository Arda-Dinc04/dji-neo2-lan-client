"""Real PC UDP/HTTP integration; device payloads and route are synthetic."""
import json
from pathlib import Path
import subprocess
import sys

from neo2_control.service import ServiceState
from test_observation import LoopbackDrone, observed, until
from test_observation_service import api
from test_wifi_status import body, frame
from test_battery import frame as battery_frame


def test_managed_api_exposes_separate_wifi_sources_without_fc_or_flight_readiness(tmp_path):
    peer = LoopbackDrone()
    peer.osd = frame() + frame(body(byte20=1), sender=0x47)
    try:
        with observed(tmp_path, peer) as (controller, manager):
            state = ServiceState(tmp_path / 'captures', tmp_path, True,
                                 controller=controller, observation=manager)
            with api(state) as request:
                assert request('/v1/drone/session/open', {'duration': 5})[0] == 202
                until(lambda: bool((request('/v1/drone/telemetry')[1].get('wifi_control') or {}).get('available')))
                code, result = request('/v1/drone/status')
                assert code == 200 and result['connected']
                assert not result['telemetry']['available']
                wifi = result['wifi_control']
                assert wifi['accepted'] >= 2
                assert len(wifi['observations']) == 2
                assert {x['source']['sender'] for x in wifi['observations']} == {0x27, 0x47}
                assert {x['mode_code'] for x in wifi['observations']} == {0, 2}
                assert all(x['fields']['wifi_mode']['valid'] for x in wifi['observations'])
                assert not wifi['flight_ready'] and not result['flight_ready']
                assert not result['live_flight_enabled'] and not wifi['runtime_product_verified']
                # All future transport windows remain alive, but no repeated
                # OSD can mask an old observation's age.
                peer.telemetry = False
                until(lambda: not request('/v1/drone/status')[1]['wifi_control']['observations'][0]['fresh'], timeout=2)
                stale = request('/v1/drone/telemetry')[1]['wifi_control']
                assert all(not x['fields']['wifi_mode']['valid'] for x in stale['observations'])
                closing = request('/v1/drone/session/close', {})[1]
                assert not closing['wifi_control']['session_active']
                assert all(x['fields']['service_guard']['reason'] == 'session_inactive'
                           for x in closing['wifi_control']['observations'])
                until(lambda: not manager.busy)
                closed = request('/v1/drone/status')[1]
                assert not closed['connected']
                capture = Path(closed['observation']['capture_path'])
                journal = [json.loads(line) for line in Path(str(capture) + '.session.jsonl').read_text().splitlines()]
                assert any(row['event'] == 'wifi_control_updated' for row in journal)
                assert journal[-1]['event'] == 'session_result'
                assert journal[-1]['wifi_control']['accepted'] >= wifi['accepted']
                assert not any(packet[6] == 5 for packet in peer.requests)
    finally:
        peer.close()


def test_managed_api_ages_battery_sources_and_preserves_invalid_percent(tmp_path):
    from test_battery import body as battery_body
    peer = LoopbackDrone()
    peer.osd = battery_frame(battery_body(flags=1 << 4)) + battery_frame(battery_body(percent=255), sender=0x23)
    try:
        with observed(tmp_path, peer) as (controller, manager):
            state = ServiceState(tmp_path, tmp_path, True, controller=controller, observation=manager)
            with api(state) as request:
                assert request('/v1/drone/session/open', {'duration': 5})[0] == 202
                until(lambda: (request('/v1/drone/telemetry')[1].get('battery') or {}).get('available'))
                battery = request('/v1/drone/telemetry')[1]['battery']
                assert len(battery['observations']) == 2
                sources = {x['source']['sender']: x['fields'] for x in battery['observations']}
                assert sources[3]['percent']['value'] == 72
                assert sources[3]['require_go_home_low_voltage']['value']
                assert sources[0x23]['percent']['value'] is None
                assert not sources[0x23]['percent']['valid']
                assert not battery['flight_ready']
                peer.telemetry = False
                until(lambda: all(not x['fresh'] for x in request('/v1/drone/status')[1]['battery']['observations']), timeout=2)
                assert request('/v1/drone/status')[1]['connected']
                closing = request('/v1/drone/session/close', {})[1]
                assert not closing['battery']['session_active']
                assert all(not field['valid'] for sample in closing['battery']['observations']
                           for field in sample['fields'].values())
                until(lambda: not manager.busy)
                assert manager.result['battery']['accepted'] >= 2
                assert not any(packet[6] == 5 for packet in peer.requests)
    finally:
        peer.close()



def test_actual_cli_session_outputs_wifi_mode_and_audits_without_active_payload(tmp_path):
    peer = LoopbackDrone()
    peer.osd = frame() + battery_frame()
    capture = tmp_path / 'cli.jsonl'
    try:
        result = subprocess.run([
            sys.executable, '-m', 'neo2_control', 'session',
            '--host', '127.0.0.1', '--port', str(peer.port),
            '--duration', '.4', '--capture', str(capture),
        ], capture_output=True, text=True, timeout=5)
        assert result.returncode == 0, result.stderr
        state = json.loads(result.stdout)
        assert state['transport_accepted'] and not state['application_ready']
        assert state['wifi_control']['accepted'] >= 1
        assert state['battery']['observations'][0]['fields']['percent']['value'] == 72
        assert not state['battery']['flight_ready']
        observed_mode = state['wifi_control']['observations'][0]
        assert observed_mode['mode_code'] == 2 and observed_mode['service_guard']
        assert observed_mode['fields']['wifi_mode']['valid']
        assert not state['telemetry']['available']
        assert not any(packet[6] == 5 for packet in peer.requests)
        rows = [json.loads(line) for line in Path(state['journal_path']).read_text().splitlines()]
        assert rows[-1]['wifi_control'] == state['wifi_control']
        assert all(path.stat().st_mode & 0o777 == 0o600
                   for path in (capture, Path(state['journal_path'])))
    finally:
        peer.close()
