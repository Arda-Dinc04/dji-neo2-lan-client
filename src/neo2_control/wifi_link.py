"""Temporary dedicated NetworkManager link around a bounded PC SWUDP session.

Credentials travel to nmcli's offline parser over stdin, never argv. Only a
new runtime profile is loaded/removed; existing profiles are never edited.
"""

from __future__ import annotations

import configparser
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import uuid

from .ble_bootstrap import WifiCredentials, load_wifi_credentials
from .session import run_session
from .identity import load_or_create_identity, validate_identity_path


RUNTIME_DIRECTORY = Path('/run/NetworkManager/system-connections')
DRONE_HOST = '192.168.2.1'


class WifiLinkError(RuntimeError):
    """A local reason code, without backend output or credentials."""


class WifiLinkCancelled(WifiLinkError):
    pass


def _cancelled(cancel_event):
    if cancel_event is not None and cancel_event.is_set():
        raise WifiLinkCancelled('wifi_connection_cancelled')


def _cancellable_process(args, input_text, timeout, cancel_event):
    """Stop only our command; the owned NM profile still needs cleanup."""
    _cancelled(cancel_event)
    deadline = time.monotonic() + timeout
    with subprocess.Popen(args, stdin=subprocess.PIPE if input_text is not None else subprocess.DEVNULL,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                          env={**os.environ, 'LC_ALL': 'C'}) as process:
        submitted = False
        try:
            while True:
                _cancelled(cancel_event)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise subprocess.TimeoutExpired(args, timeout)
                data = input_text if not submitted else None
                submitted = True
                try:
                    output, error = process.communicate(data, timeout=min(.05, remaining))
                    return subprocess.CompletedProcess(args, process.returncode, output, error)
                except subprocess.TimeoutExpired:
                    continue
        except BaseException:
            if process.poll() is None:
                try:
                    process.terminate()
                except ProcessLookupError:
                    pass
            try:
                process.communicate(timeout=.25)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate(timeout=1)
            raise


def _run(*args: str, input_text: str | None = None, timeout: float = 10,
         reason: str = 'network_command_failed', allowed=(0,), cancel_event=None):
    try:
        if cancel_event is None:
            result = subprocess.run(args, input=input_text, text=True, capture_output=True,
                                    timeout=timeout, env={**os.environ, 'LC_ALL': 'C'})
        else:
            result = _cancellable_process(args, input_text, timeout, cancel_event)
        _cancelled(cancel_event)
    except (OSError, subprocess.SubprocessError, UnicodeError) as error:
        raise WifiLinkError(reason) from error
    if result.returncode not in allowed:
        raise WifiLinkError(reason)
    return result


@dataclass(frozen=True, repr=False)
class WifiPlan:
    interface: str
    connection_uuid: str
    keyfile: str
    network: dict

    @property
    def connection_id(self):
        return f'neo2-control-{self.connection_uuid}'

    def metadata(self):
        return {'interface': self.interface, 'connection_uuid': self.connection_uuid,
                'network': self.network, 'autoconnect': False, 'default_route': False,
                'use_drone_dns': False, 'drone_route': f'{DRONE_HOST}/32'}


def prepare_wifi_plan(credentials: WifiCredentials, interface: str, *, cancel_event=None) -> WifiPlan:
    if not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', interface):
        raise ValueError('invalid Linux network interface name')
    password = credentials.passphrase
    if not ((8 <= len(password) <= 63 and all(32 <= c <= 126 for c in password))
            or (len(password) == 64 and re.fullmatch(rb'[0-9a-fA-F]{64}', password))):
        raise ValueError('unsupported Wi-Fi PSK format')
    identifier = str(uuid.uuid4())
    name = f'neo2-control-{identifier}'
    # Decimal byte list preserves SSIDs containing non-UTF8, whitespace, '\n'
    # or semicolons; nmcli normalizes it to its own current keyfile syntax.
    ssid = ''.join(f'{byte};' for byte in credentials.ssid)
    psk = password.decode('ascii').replace('\\', '\\\\').replace(' ', '\\s')
    keyfile = f'''[connection]
id={name}
uuid={identifier}
type=wifi
interface-name={interface}
autoconnect=false
[wifi]
mode=infrastructure
ssid={ssid}
bssid={credentials.bssid.hex(':')}
[wifi-security]
key-mgmt=wpa-psk
psk={psk}
[ipv4]
method=auto
never-default=true
ignore-auto-dns=true
ignore-auto-routes=true
route1={DRONE_HOST}/32
dhcp-send-hostname=false
may-fail=false
[ipv6]
method=disabled
never-default=true
'''
    normalized = _run('nmcli', '--offline', 'connection', 'modify', 'connection.id', name,
                      input_text=keyfile, reason='offline_profile_validation_failed', cancel_event=cancel_event).stdout
    parsed = configparser.ConfigParser(interpolation=None, strict=True)
    try:
        parsed.read_string(normalized)
        expected = {
            ('connection', 'id'): name, ('connection', 'uuid'): identifier,
            ('connection', 'interface-name'): interface,
            ('connection', 'autoconnect'): 'false',
            ('ipv4', 'method'): 'auto', ('ipv4', 'never-default'): 'true',
            ('ipv4', 'ignore-auto-dns'): 'true', ('ipv4', 'ignore-auto-routes'): 'true',
            ('ipv4', 'route1'): f'{DRONE_HOST}/32',
            ('ipv6', 'method'): 'disabled', ('ipv6', 'never-default'): 'true',
            ('wifi-security', 'key-mgmt'): 'wpa-psk',
        }
        if any(parsed.get(section, key) != value for (section, key), value in expected.items()):
            raise ValueError
    except (configparser.Error, ValueError) as error:
        raise WifiLinkError('offline_profile_changed_required_properties') from error
    return WifiPlan(interface, identifier, normalized, credentials.metadata())


def _routes(*, cancel_event=None):
    routes = []
    for family in ('-4', '-6'):
        output = _run('ip', '-j', family, 'route', 'show', 'table', 'all', 'default',
                      reason='cannot_read_default_routes', cancel_event=cancel_event).stdout
        try:
            records = json.loads(output)
            if not isinstance(records, list) or any(not isinstance(r, dict) for r in records):
                raise ValueError
            # Ignore changing lifetime counters; preserve all routing decisions.
            routes.extend((family, {k: v for k, v in r.items() if k != 'expires'}) for r in records)
        except ValueError as error:
            raise WifiLinkError('invalid_default_routes') from error
    return sorted(routes, key=lambda item: json.dumps(item, sort_keys=True))


def require_dedicated_drone_route(interface: str, *, cancel_event=None) -> None:
    """Read-only guard for an already associated interface, before any SYN."""
    if not isinstance(interface, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', interface):
        raise WifiLinkError('invalid_network_interface')
    for _, route in _routes(cancel_event=cancel_event):
        devices = [route.get('dev')] + [hop.get('dev') for hop in route.get('nexthops', [])]
        if interface in devices:
            raise WifiLinkError('interface_carries_default_route')
    output = _run('ip', '-j', '-4', 'route', 'get', DRONE_HOST,
                  reason='cannot_inspect_drone_route', cancel_event=cancel_event).stdout
    try:
        routes = json.loads(output)
        if (not isinstance(routes, list) or len(routes) != 1 or not isinstance(routes[0], dict)
            or routes[0].get('dst') != DRONE_HOST
            or routes[0].get('dev') != interface or routes[0].get('gateway')
            or routes[0].get('type', 'unicast') != 'unicast'):
            raise ValueError
    except (ValueError, TypeError) as error:
        raise WifiLinkError('drone_route_is_not_direct_on_selected_interface') from error


def _require_idle_wifi_interface(interface: str, *, cancel_event=None):
    output = _run('nmcli', '--get-values', 'GENERAL.TYPE,GENERAL.STATE', 'device',
                  'show', interface, reason='cannot_inspect_interface',
                  cancel_event=cancel_event).stdout.splitlines()
    if (len(output) != 2 or output[0] != 'wifi' or not output[1].split()
        or output[1].split(maxsplit=1)[0] != '30'):
        raise WifiLinkError('interface_must_be_disconnected_wifi')


def require_available_wifi_interface(interface: str, *, cancel_event=None):
    """Read-only preflight, also used before connecting a retained BLE peer."""
    if not isinstance(interface, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', interface):
        raise WifiLinkError('invalid_network_interface')
    _cancelled(cancel_event)
    if os.geteuid() != 0:
        raise WifiLinkError('wifi_activation_requires_root')
    defaults = _routes(cancel_event=cancel_event)
    for _, route in defaults:
        devices = [route.get('dev')] + [hop.get('dev') for hop in route.get('nexthops', [])]
        if interface in devices:
            raise WifiLinkError('interface_carries_default_route')
    _require_idle_wifi_interface(interface, cancel_event=cancel_event)
    return defaults


class DedicatedWifiLink:
    """Load a new temporary profile, verify its route and remove it on exit."""

    def __init__(self, plan: WifiPlan, event, *, runtime_directory: Path | None = None,
                 cancel_event=None):
        self.plan, self.event = plan, event
        directory = RUNTIME_DIRECTORY if runtime_directory is None else runtime_directory
        self.path = directory / f'{plan.connection_id}.nmconnection'
        self.created = self.connected = self.removed = False
        self.ever_created = self.deactivated = self.cleanup_verified = False
        self.entered = False
        self.cancel_event = cancel_event

    def _command(self, *args, **kwargs):
        _cancelled(self.cancel_event)
        return _run(*args, **kwargs, cancel_event=self.cancel_event)

    def _idle_interface(self):
        _require_idle_wifi_interface(self.plan.interface, cancel_event=self.cancel_event)

    def __enter__(self):
        if self.entered:
            raise WifiLinkError('wifi_link_cannot_be_reused')
        self.entered = True
        _cancelled(self.cancel_event)
        self.defaults = require_available_wifi_interface(self.plan.interface, cancel_event=self.cancel_event)
        existing = self._command('nmcli', '--get-values', 'UUID', 'connection', 'show',
                        reason='cannot_inspect_profiles').stdout.splitlines()
        if self.plan.connection_uuid in existing:
            raise WifiLinkError('connection_uuid_already_exists')
        try:
            _cancelled(self.cancel_event)
            self.path.parent.mkdir(mode=0o700, exist_ok=True)
            descriptor = os.open(self.path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
            self.created = self.ever_created = True
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                stream.write(self.plan.keyfile)
                stream.flush()
                os.fsync(stream.fileno())
            self.event('runtime_profile_created', **self.plan.metadata())
            self._command('nmcli', 'connection', 'load', str(self.path), reason='profile_load_failed')
            # Recheck immediately before activation; never switch an active link.
            self._idle_interface()
            if _routes(cancel_event=self.cancel_event) != self.defaults:
                raise WifiLinkError('default_routes_changed_before_activation')
            self._command('nmcli', '--wait', '20', 'connection', 'up', 'uuid',
                 self.plan.connection_uuid, 'ifname', self.plan.interface,
                 timeout=25, reason='wifi_activation_failed')
            self.connected = True
            self.event('wifi_activated')
            active = self._command('nmcli', '--get-values', 'GENERAL.CON-UUID', 'device', 'show',
                          self.plan.interface, reason='cannot_inspect_active_profile').stdout.strip()
            if active != self.plan.connection_uuid:
                raise WifiLinkError('selected_profile_is_not_active')
            if _routes(cancel_event=self.cancel_event) != self.defaults:
                raise WifiLinkError('default_routes_changed_after_activation')
            output = self._command('ip', '-j', 'route', 'get', DRONE_HOST,
                          reason='cannot_inspect_drone_route').stdout
            try:
                route = json.loads(output)
                if len(route) != 1 or route[0]['dev'] != self.plan.interface or route[0].get('gateway'):
                    raise ValueError
            except (ValueError, KeyError, TypeError) as error:
                raise WifiLinkError('drone_route_is_not_direct_on_selected_interface') from error
            self.event('wifi_route_verified', interface=self.plan.interface)
            return self
        except BaseException:
            self._cleanup()
            raise

    def _cleanup(self):
        if not self.created:
            return
        audit_error = None
        try:
            self.event('wifi_cleanup_started')
        except BaseException as error:
            audit_error = error
        # Cancellation never skips cleanup. Each operation names only the
        # UUID we created; existing profiles/device-wide switches are untouched.
        try:
            _run('nmcli', '--wait', '10', 'connection', 'down', 'uuid',
                 self.plan.connection_uuid, timeout=15, reason='wifi_cleanup_failed', allowed=(0, 10))
        except WifiLinkError:
            pass  # Still attempt deletion and verify the actual active list.
        try:
            _run('nmcli', '--wait', '10', 'connection', 'delete', 'uuid',
                 self.plan.connection_uuid, timeout=15, reason='wifi_cleanup_failed', allowed=(0, 10))
            self.removed = True
        except WifiLinkError:
            pass
        try:
            active = _run('nmcli', '--get-values', 'UUID', 'connection', 'show', '--active',
                          timeout=5, reason='wifi_cleanup_failed').stdout.splitlines()
            active = [str(uuid.UUID(value.strip())) for value in active if value.strip()]
            self.deactivated = self.plan.connection_uuid not in active
        except (WifiLinkError, ValueError):
            self.deactivated = False
        file_removed = False
        try:
            self.path.unlink(missing_ok=True)
            file_removed = True
        except OSError:
            pass
        self.cleanup_verified = self.removed and self.deactivated and file_removed
        if self.cleanup_verified:
            self.created = False
        try:
            self.event('wifi_profile_removed' if self.cleanup_verified else 'wifi_cleanup_failed',
                       profile_removed=self.removed, deactivated=self.deactivated,
                       cleanup_verified=self.cleanup_verified)
        except BaseException as error:
            audit_error = audit_error or error
        if not self.cleanup_verified:
            raise WifiLinkError('wifi_cleanup_failed')
        if audit_error:
            raise audit_error

    def __exit__(self, *_):
        self._cleanup()


def run_wifi_session(*, credentials_file: str | Path, interface: str,
                     capture_path: str | Path, duration: float = 30,
                     heartbeat: bool = False, video_path: str | Path | None = None,
                     apply: bool = False, wlm_identity_file: str | Path | None = None,
                     hls_directory: str | Path | None = None, video_nominal_fps: float = 30,
                     video_codec: str = 'h264', publish_config_file: str | Path | None = None) -> dict:
    if not math.isfinite(duration) or not 0 < duration <= 86400:
        raise ValueError('session duration must be finite, positive and at most 86400 seconds')
    capture, source = Path(capture_path), Path(credentials_file)
    journal_path = capture.with_name(capture.name + '.wifi.jsonl')
    outputs = [capture, journal_path, capture.with_name(capture.name + '.session.jsonl')]
    if video_path is not None:
        outputs.append(Path(video_path))
    if hls_directory is None and (video_nominal_fps != 30 or video_codec != 'h264'):
        raise ValueError('video decoding options require an HLS directory')
    if publish_config_file is not None:
        if hls_directory is None:
            raise ValueError('video publication requires an HLS directory')
        from .video_publish import PublishConfig
        PublishConfig.load(publish_config_file, protected_paths=[*outputs, source, hls_directory]
            + ([wlm_identity_file] if wlm_identity_file is not None else []))
    if hls_directory is not None:
        from .video import HlsVideo
        HlsVideo(hls_directory, nominal_fps=video_nominal_fps, input_codec=video_codec,
            protected_paths=[*outputs, source] + ([wlm_identity_file] if wlm_identity_file is not None else []))
    for path in outputs:
        if path.exists() or path.is_symlink():
            raise ValueError('Wi-Fi session outputs must be new files')
    if len({p.resolve() for p in outputs + [source]}) != len(outputs) + 1:
        raise ValueError('credentials, captures, journals and video must use distinct paths')
    if wlm_identity_file is not None:
        validate_identity_path(wlm_identity_file, [*outputs, source])
        if apply or Path(wlm_identity_file).exists():
            load_or_create_identity(wlm_identity_file, protected_paths=[*outputs, source])
    plan = prepare_wifi_plan(load_wifi_credentials(source), interface)
    metadata = {**plan.metadata(), 'application_ready': False, 'capture_path': str(capture),
                'journal_path': str(journal_path), 'duration_seconds': duration,
                'wlm_bootstrap_enabled': wlm_identity_file is not None,
                'hls_enabled': hls_directory is not None,
                'video_publish_enabled': publish_config_file is not None}
    if not apply:
        return {'state': 'planned', 'reason': 'not_applied', **metadata}
    started = time.monotonic()
    journal_path.parent.mkdir(parents=True, exist_ok=True)
    with os.fdopen(os.open(journal_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600),
                   'w', encoding='utf-8') as journal:
        def event(name, **details):
            journal.write(json.dumps({'event': name, 'elapsed_seconds': time.monotonic() - started,
                                      **details}, separators=(',', ':')) + '\n')
            journal.flush()

        event('wifi_session_started', **metadata)
        link = DedicatedWifiLink(plan, event)
        session = None
        try:
            with link:
                session = run_session(capture_path=capture, host=DRONE_HOST, port=9003,
                                      interface=interface, duration=duration,
                                      heartbeat=heartbeat, video_path=video_path,
                                      **({'wlm_identity_file': wlm_identity_file} if wlm_identity_file is not None else {}),
                                      **({'hls_directory': hls_directory, 'video_nominal_fps': video_nominal_fps,
                                          'video_codec': video_codec} if hls_directory is not None else {}),
                                      **({'publish_config_file': publish_config_file} if publish_config_file is not None else {}))
            success = (session.state == 'closed' and session.transport_accepted
                and (hls_directory is None or (not session.video.get('error') and session.video.get('frames_encoded', 0) > 0)))
            if publish_config_file is not None:
                from .video_publish import publication_succeeded
                success = success and publication_succeeded(session.video_publish)
            result = {'state': 'closed' if success else 'failed', 'reason': session.reason,
                      'session': session.as_dict(), 'wifi_profile_removed': link.removed,
                      'wifi_deactivated': link.deactivated, 'wifi_cleanup_verified': link.cleanup_verified, **metadata}
        except WifiLinkError as error:
            result = {'state': 'failed', 'reason': str(error), 'wifi_profile_removed': link.removed,
                      'wifi_deactivated': link.deactivated, 'wifi_cleanup_verified': link.cleanup_verified,
                      'session': session.as_dict() if session is not None else None, **metadata}
        finally:
            event('wifi_session_finished', wifi_profile_removed=link.removed,
                  wifi_deactivated=link.deactivated, wifi_cleanup_verified=link.cleanup_verified)
        event('wifi_session_result', **result)
        return result
