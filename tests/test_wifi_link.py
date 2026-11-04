import json
from pathlib import Path
import shutil
import stat
import subprocess
from types import SimpleNamespace

import pytest

from neo2_control import wifi_link as wifi
from neo2_control.ble_bootstrap import WifiCredentials
from neo2_control.cli import main


SSID, PASSWORD = b'NEO2-LAB', b'test-only-password'


class Network:
    def __init__(self, tmp_path):
        self.directory = tmp_path / 'runtime'
        self.calls = []
        self.connected = False
        self.fail = None
        self.default_interface = 'eth0'
        self.device_state = 'wifi\n30 (disconnected)\n'
        self.active_uuid = None
        self.bad_route = self.change_default = self.fail_delete = False
        self.events = []

    def run(self, *args, input_text=None, **kwargs):
        self.calls.append(args)
        assert PASSWORD.decode() not in ' '.join(args) and SSID.decode() not in ' '.join(args)
        if '--offline' in args:
            return SimpleNamespace(stdout=input_text, returncode=0)
        if args[:4] == ('nmcli', '--get-values', 'GENERAL.TYPE,GENERAL.STATE', 'device'):
            return SimpleNamespace(stdout=self.device_state)
        if args[:3] == ('nmcli', '--get-values', 'UUID'):
            return SimpleNamespace(stdout=self.uuid if '--active' in args and self.connected else '')
        if args[:3] == ('nmcli', 'connection', 'load'):
            path = Path(args[3])
            assert stat.S_IMODE(path.stat().st_mode) == 0o600
            self.uuid = path.stem.removeprefix('neo2-control-')
            if self.fail == 'load':
                raise wifi.WifiLinkError('profile_load_failed')
            return SimpleNamespace(stdout='')
        if 'up' in args:
            if self.fail == 'up':
                raise wifi.WifiLinkError('wifi_activation_failed')
            self.connected = True
            return SimpleNamespace(stdout='')
        if 'delete' in args:
            if not hasattr(self, 'uuid'):
                self.uuid = next(self.directory.glob('neo2-control-*.nmconnection')).stem.removeprefix('neo2-control-')
            assert args[-1] == self.uuid
            if self.fail_delete:
                raise wifi.WifiLinkError('wifi_cleanup_failed')
            self.connected = False
            return SimpleNamespace(stdout='')
        if 'down' in args:
            self.connected = False
            return SimpleNamespace(stdout='')
        if args[:3] == ('nmcli', '--get-values', 'GENERAL.CON-UUID'):
            return SimpleNamespace(stdout=self.active_uuid or self.uuid)
        if args[0] == 'ip' and 'get' in args:
            return SimpleNamespace(stdout=json.dumps([{'dst': '192.168.2.1', 'dev': 'eth0' if self.bad_route else 'wlanusb0'}]))
        if args[0] == 'ip' and 'default' in args:
            records = ([{'dev': self.default_interface, 'gateway': '192.0.2.1'}]
                       if '-4' in args else [])
            if self.connected and self.change_default:
                records.append({'dev': 'wlanusb0', 'gateway': '192.168.2.1'})
            return SimpleNamespace(stdout=json.dumps(records))
        raise AssertionError(args)


@pytest.fixture
def network(monkeypatch, tmp_path):
    system = Network(tmp_path)
    monkeypatch.setattr(wifi, '_run', system.run)
    monkeypatch.setattr(wifi.os, 'geteuid', lambda: 0)
    monkeypatch.setattr(wifi, 'RUNTIME_DIRECTORY', system.directory)
    return system


def plan():
    return wifi.prepare_wifi_plan(WifiCredentials(bytes.fromhex('020000000002'), SSID, PASSWORD), 'wlanusb0')


def test_temporary_link_loads_only_owned_profile_and_removes_it(network):
    blueprint = plan()
    with wifi.DedicatedWifiLink(blueprint, lambda name, **kw: network.events.append(name)) as link:
        assert network.connected and link.path.exists()
        assert 'wifi_route_verified' in network.events
        assert SSID.decode() not in repr(blueprint) and PASSWORD.decode() not in repr(blueprint)
    assert link.removed and not network.connected and not link.path.exists()
    assert network.calls[-2][-3:] == ('delete', 'uuid', blueprint.connection_uuid)
    assert 'down' in network.calls[-3] and '--active' in network.calls[-1]
    assert link.cleanup_verified and link.deactivated
    assert not any('modify' in c for c in network.calls if '--offline' not in c)


@pytest.mark.parametrize('failure', ['load', 'up'])
def test_partial_activation_is_removed_without_running_transport(network, failure):
    network.fail = failure
    link = wifi.DedicatedWifiLink(plan(), lambda *a, **k: None)
    with pytest.raises(wifi.WifiLinkError):
        with link:
            pytest.fail('failed activation must not enter session')
    assert link.removed and not link.path.exists() and not network.connected


@pytest.mark.parametrize('state', ['wifi\n100 (connected)\n', 'ethernet\n30 (disconnected)\n', 'wifi\n\n'])
def test_existing_link_or_wrong_device_is_never_switched(network, state):
    network.device_state = state
    with pytest.raises(wifi.WifiLinkError, match='disconnected_wifi'):
        with wifi.DedicatedWifiLink(plan(), lambda *a, **k: None):
            pytest.fail()
    assert not network.directory.exists()
    assert not any('load' in call or 'up' in call or 'delete' in call for call in network.calls)


def test_default_route_interface_is_protected_even_if_reported_idle(network):
    network.default_interface = 'wlanusb0'
    with pytest.raises(wifi.WifiLinkError, match='carries_default_route'):
        with wifi.DedicatedWifiLink(plan(), lambda *a, **k: None):
            pytest.fail()
    assert not network.directory.exists()


@pytest.mark.parametrize('attribute,reason', [('change_default', 'default_routes_changed'),
    ('bad_route', 'drone_route_is_not_direct'), ('active_uuid', 'selected_profile_is_not_active')])
def test_post_activation_checks_fail_before_any_transport(network, attribute, reason):
    setattr(network, attribute, 'other-profile' if attribute == 'active_uuid' else True)
    link = wifi.DedicatedWifiLink(plan(), lambda *a, **k: None)
    with pytest.raises(wifi.WifiLinkError, match=reason):
        with link:
            pytest.fail('route verification must precede transport')
    assert link.removed and not network.connected and not link.path.exists()


@pytest.mark.parametrize('error', [KeyboardInterrupt, OSError])
def test_session_failure_and_interrupt_always_remove_temporary_link(network, error):
    link = wifi.DedicatedWifiLink(plan(), lambda *a, **k: None)
    with pytest.raises(error):
        with link:
            raise error('synthetic failure')
    assert link.removed and not link.path.exists() and not network.connected


def test_journal_failure_cannot_prevent_cleanup(network):
    def event(name, **kwargs):
        if name == 'wifi_profile_removed':
            raise OSError('journal failed')
    link = wifi.DedicatedWifiLink(plan(), event)
    with pytest.raises(OSError):
        with link:
            pass
    assert link.removed and not network.connected and not link.path.exists()


def test_cleanup_failure_tries_deactivation_and_reports_not_removed(network):
    network.fail_delete = True
    link = wifi.DedicatedWifiLink(plan(), lambda *a, **k: None)
    with pytest.raises(wifi.WifiLinkError, match='cleanup_failed'):
        with link:
            pass
    assert not link.removed and not network.connected and not link.path.exists()
    assert any('down' in call for call in network.calls)
    assert link.deactivated and not link.cleanup_verified


def test_existing_runtime_file_is_preserved(network):
    blueprint = plan()
    network.directory.mkdir()
    path = network.directory / f'{blueprint.connection_id}.nmconnection'
    path.write_text('existing')
    with pytest.raises(FileExistsError):
        with wifi.DedicatedWifiLink(blueprint, lambda *a, **k: None):
            pytest.fail()
    assert path.read_text() == 'existing'
    assert not any('delete' in call for call in network.calls)


def test_context_is_single_use_and_successful_cleanup_is_idempotent(network):
    link = wifi.DedicatedWifiLink(plan(), lambda *a, **k: None)
    with link:
        pass
    calls = len(network.calls)
    link.__exit__(None, None, None)
    assert len(network.calls) == calls
    with pytest.raises(wifi.WifiLinkError, match='cannot_be_reused'):
        with link:
            pytest.fail()
    assert len(network.calls) == calls


def test_nonroot_activation_never_writes_runtime_profile(network, monkeypatch):
    monkeypatch.setattr(wifi.os, 'geteuid', lambda: 1000)
    with pytest.raises(wifi.WifiLinkError, match='requires_root'):
        with wifi.DedicatedWifiLink(plan(), lambda *a, **k: None):
            pytest.fail()
    assert not network.directory.exists()


def test_backend_failure_text_and_input_are_never_exposed(monkeypatch):
    monkeypatch.setattr(subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=1, stderr=PASSWORD.decode()))
    with pytest.raises(wifi.WifiLinkError) as error:
        wifi._run('nmcli', input_text=PASSWORD.decode())
    assert PASSWORD.decode() not in str(error.value)


def test_wifi_session_plans_then_connects_runs_and_cleans(network, monkeypatch, tmp_path):
    source, capture = tmp_path / 'wifi.json', tmp_path / 'session.jsonl'
    WifiCredentials(bytes.fromhex('020000000002'), SSID, PASSWORD).save_new(source)
    def session(**kwargs):
        assert network.connected and kwargs['interface'] == 'wlanusb0'
        assert (kwargs['host'], kwargs['port']) == ('192.168.2.1', 9003)
        return SimpleNamespace(state='closed', reason='duration_elapsed', transport_accepted=True,
                               as_dict=lambda: {'state': 'closed', 'transport_accepted': True})
    monkeypatch.setattr(wifi, 'run_session', session)
    args = dict(credentials_file=source, interface='wlanusb0', capture_path=capture)
    planned = wifi.run_wifi_session(**args)
    assert planned['state'] == 'planned' and not network.directory.exists()
    assert not capture.with_name(capture.name + '.wifi.jsonl').exists()
    result = wifi.run_wifi_session(**args, apply=True)
    assert result['state'] == 'closed' and result['wifi_profile_removed']
    assert result['application_ready'] is False and not network.connected
    journal = Path(result['journal_path'])
    assert stat.S_IMODE(journal.stat().st_mode) == 0o600
    output = journal.read_text() + json.dumps(result)
    assert SSID.decode() not in output and PASSWORD.decode() not in output


def test_transport_never_runs_if_link_preflight_fails(network, monkeypatch, tmp_path):
    source = tmp_path / 'wifi.json'
    WifiCredentials(bytes.fromhex('020000000002'), SSID, PASSWORD).save_new(source)
    network.device_state = 'wifi\n100 (connected)\n'
    monkeypatch.setattr(wifi, 'run_session', lambda **kw: pytest.fail('no packets before verified link'))
    result = wifi.run_wifi_session(credentials_file=source, interface='wlanusb0',
                                   capture_path=tmp_path / 'capture.jsonl', apply=True)
    assert result['state'] == 'failed' and result['session'] is None


def test_cli_plan_does_not_mutate_network_or_print_credentials(network, tmp_path, capsys):
    source = tmp_path / 'wifi.json'
    WifiCredentials(bytes.fromhex('020000000002'), SSID, PASSWORD).save_new(source)
    assert main(['wifi-session', '--credentials-file', str(source), '--interface', 'wlanusb0',
                 '--capture', str(tmp_path / 'capture.jsonl')]) == 0
    output = capsys.readouterr().out
    assert json.loads(output)['state'] == 'planned'
    assert SSID.decode() not in output and PASSWORD.decode() not in output
    assert all('--offline' in call for call in network.calls)


@pytest.mark.skipif(shutil.which('nmcli') is None, reason='real NetworkManager offline parser unavailable')
@pytest.mark.parametrize('ssid,password', [(b'NEO2-LAB', b'synthetic-passphrase'),
    (b' 12;34;\\\n', b' a\\b[]#;% '), (b'numeric123', b'0123456789abcdef' * 4)])
def test_real_nmcli_offline_roundtrip_preserves_credentials_and_isolation(ssid, password):
    credentials = WifiCredentials(bytes.fromhex('020000000002'), ssid, password)
    blueprint = wifi.prepare_wifi_plan(credentials, 'wlantest0')
    # Synthetic values only: resetting properties to the expected bytes must
    # leave the real parser's normalized keyfile byte-identical.
    result = subprocess.run(['nmcli', '--offline', 'connection', 'modify',
                             'wifi.ssid', ssid.decode('ascii'), 'wifi-sec.psk', password.decode('ascii')],
                            input=blueprint.keyfile, text=True, capture_output=True, check=True)
    assert result.stdout == blueprint.keyfile
    assert blueprint.keyfile.count('never-default=true') == 2
    assert 'ignore-auto-dns=true' in blueprint.keyfile and 'ignore-auto-routes=true' in blueprint.keyfile
    assert 'route1=192.168.2.1/32' in blueprint.keyfile and 'autoconnect=false' in blueprint.keyfile
