"""Independent controller owner and bounded local intent channel.

Only the owner opens the journal and calls the backend. HTTP process liveness
does not renew the operator lease. This prepares process isolation; it proves
neither real aircraft behavior nor survival of owner/host failure.
"""
from __future__ import annotations

import copy
import json
import math
import os
from pathlib import Path
import secrets
import select
import shutil
import socket
import subprocess
import sys
import threading
import time

from .controller import ControllerMode, ControlSupervisor, DecisionJournal, IntentRejected
from .command_ingress import CommandIngress
from .remote_target import RemoteTargetIngress, RemoteTargetRunner
from .simulator import SimulationBackend

MAX_PACKET = 32768
TICK_SECONDS = .025
HEARTBEAT_SECONDS = .1
PARENT_TIMEOUT = .4
REQUEST_SECONDS = .3
CLOSE_SECONDS = 6.5
FLIGHT_CLOSE_SECONDS = 75


class OwnerUnavailable(RuntimeError):
    pass


def _encode(value):
    data = json.dumps(value, allow_nan=False, separators=(',', ':')).encode('utf-8')
    if len(data) > MAX_PACKET:
        raise ValueError('controller IPC packet limit')
    return data


def _decode(data):
    def constant(_):
        raise ValueError('nonfinite JSON')
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate JSON key')
            result[key] = value
        return result
    if not data or len(data) > MAX_PACKET:
        raise ValueError('controller IPC packet limit')
    result = json.loads(data, parse_constant=constant, object_pairs_hook=pairs)
    if not isinstance(result, dict):
        raise ValueError('controller IPC object required')
    return result


def _receive(channel):
    data, _, flags, _ = channel.recvmsg(MAX_PACKET)
    if flags & socket.MSG_TRUNC:
        raise ValueError('controller IPC packet truncated')
    if not data:
        raise EOFError
    return _decode(data)


def _send(channel, value):
    data = _encode(value)
    if channel.send(data) != len(data):
        raise OSError('controller IPC short send')


def _valid_request(request, epoch, previous_id, now):
    if set(request) != {'kind', 'epoch', 'id', 'expires_at', 'operation', 'parameters'}:
        return False
    expiry = request['expires_at']
    return (request['kind'] == 'request' and request['epoch'] == epoch
            and type(request['id']) is int and request['id'] > previous_id
            and type(expiry) in (int, float)
            and now < expiry <= now + REQUEST_SECONDS + .05 and math.isfinite(expiry)
            and request['operation'] in ('heartbeat', 'status', 'action', 'camera', 'remote_action', 'remote_camera', 'shutdown')
            and isinstance(request['parameters'], dict))


def run_owner(fd: int, journal_path: str | Path, *, flight_config=None, backend_factory=None) -> int:
    """Private owner entry point; optional flight connection opens by intent."""
    channel = socket.socket(fileno=fd)
    channel.setblocking(False)
    journal = None
    controller = None
    incident_store = None
    stopping = False
    channel_open = True
    retention_recorded = False
    try:
        journal = DecisionJournal(journal_path)
        mode='simulate' if flight_config is None else 'flight'
        if flight_config is None:
            backend=SimulationBackend()
        else:
            from .flight_connection import OwnedFlightBackend
            from .flight_incident import FlightIncidentStore
            incident_store = FlightIncidentStore(flight_config.incident_directory,flight_config.ble_address).__enter__()
            backend=(backend_factory or OwnedFlightBackend)(flight_config)
        controller = ControlSupervisor(journal, mode=mode, backend=backend,
            vertical_trial=flight_config.vertical_trial if flight_config is not None else False,
            yaw_trial=flight_config.yaw_trial if flight_config is not None else False,
            acquisition_trial=flight_config.acquisition_trial if flight_config is not None else False,
            incident_store=incident_store)
        epoch = secrets.token_hex(24)
        remote = RemoteTargetRunner(controller,epoch)
        journal.record({'event': 'controller_owner_started', 'owner_pid': os.getpid(),
                        'mode':mode,'live_flight_enabled':mode=='flight',
                        'vertical_trial':controller.vertical_trial,'yaw_trial':controller.yaw_trial})
        _send(channel, {'kind': 'ready', 'epoch': epoch, 'status': controller.status()})
        last_parent = time.monotonic()
        previous_id = 0
        previous_mutation = 0
        previous_remote_mutation = 0
        while True:
            now = time.monotonic()
            if not stopping and now - last_parent >= PARENT_TIMEOUT:
                controller.stop('controller_parent_timeout')
                stopping = True
            remote.tick()
            controller.tick()
            if stopping and controller.state != 'landing':
                retain=(mode=='flight' and controller.flight_started
                        and backend.connection_status()['worker_alive'])
                if not retain:break
                # A fault/expired landing deadline is not ground proof. Keep
                # this bounded connection alive after parent loss; never
                # discard it merely to produce a clean process exit.
                if not retention_recorded:
                    journal.record({'event':'flight_owner_retaining_connection',
                        'owner_pid':os.getpid(),'possible_airborne':True})
                    retention_recorded=True
            ready, _, _ = select.select([channel] if channel_open else [], [], [],
                remote.wait_seconds(TICK_SECONDS))
            if not ready:
                continue
            try:
                request = _receive(channel)
                now = time.monotonic()
                # Check before consuming a newly arrived heartbeat: a suspended
                # parent cannot revive authority after its deadline.
                if not stopping and now - last_parent >= PARENT_TIMEOUT:
                    controller.stop('controller_parent_timeout')
                    stopping = True
                if not _valid_request(request, epoch, previous_id, now):
                    raise ValueError('controller IPC invalid request')
                previous_id = request['id']
                last_parent = now
                operation, parameters = request['operation'], request['parameters']
                if operation not in ('action','camera','remote_action','remote_camera') and parameters:
                    raise ValueError('controller IPC unexpected parameters')
                result, error = None, None
                try:
                    ingress = None
                    if operation in ('action', 'camera', 'remote_action', 'remote_camera'):
                        if set(parameters) not in ({'action', 'parameters'}, {'action', 'parameters', 'ingress'}):
                            raise IntentRejected('invalid_parameters')
                        if 'ingress' in parameters:
                            try:
                                ingress_type=RemoteTargetIngress if operation.startswith('remote_') else CommandIngress
                                ingress = ingress_type.from_dict(parameters['ingress'])
                                status = getattr(backend, 'connection_status', None)
                                generation = status().get('generation') if callable(status) else None
                                ingress.validate(time.monotonic(), owner_epoch=epoch,
                                                 connection_generation=generation)
                            except ValueError as rejected:
                                raise IntentRejected(str(rejected)) from None
                            previous = previous_remote_mutation if operation.startswith('remote_') else previous_mutation
                            if ingress.mutation_sequence <= previous:
                                raise IntentRejected('command_mutation_sequence_replayed')
                            # One sequence spans both mutating endpoints, even
                            # when a subsequent business rule refuses the action.
                            if operation.startswith('remote_'):previous_remote_mutation = ingress.mutation_sequence
                            else:previous_mutation = ingress.mutation_sequence
                        elif operation.startswith('remote_'):
                            raise IntentRejected('invalid_remote_ingress')
                    if operation == 'action':
                        remote.clear('local_operator_action')
                        result = controller.handle(parameters['action'], parameters['parameters'], ingress=ingress)
                    elif operation=='remote_action':
                        result = remote.handle(parameters['action'],parameters['parameters'],ingress)
                    elif operation in ('camera','remote_camera'):
                        if stopping:raise IntentRejected('controller_stopping')
                        if controller.recovery_required:raise IntentRejected('flight_recovery_required')
                        if mode!='flight':
                            raise IntentRejected('invalid_camera_request')
                        try:
                            if operation=='remote_camera':ingress=ingress.local_execution(time.monotonic())
                            options = {} if ingress is None else {'ingress': ingress}
                            result=backend.camera_command(parameters['action'],parameters['parameters'], **options)
                        except (ValueError,RuntimeError) as error:raise IntentRejected(str(error)) from None
                    elif operation == 'shutdown':
                        controller.stop()
                        stopping = True
                    else:
                        result = {}
                except IntentRejected as rejected:
                    error = str(rejected)
                status=controller.status()
                status['remote_control']=remote.status()
                _send(channel, {'kind': 'response', 'id': previous_id, 'error': error,
                                'result': result, 'status': status})
            except (EOFError, OSError, ValueError, TypeError, RecursionError) as error:
                channel_open = False
                channel.close()
                controller.stop('controller_parent_disconnected' if isinstance(error, EOFError)
                                else 'controller_ipc_failed')
                stopping = True
        cleanup_verified=True
        try:
            backend.close(time.monotonic())
            if callable(getattr(backend,'finish_cleanup',None)):backend.finish_cleanup()
        except Exception:cleanup_verified=False
        final = controller.status()
        final['connected'] = False
        journal.record({'event': 'controller_owner_stopped', 'owner_pid': os.getpid(),
                        'mode':mode,'cleanup_verified':cleanup_verified,'landing_confirmed': final['landing_confirmed'],
                        'possible_airborne': final['possible_airborne'],
                        'fault_reason': final['fault_reason']})
        if channel_open:
            try:
                _send(channel, {'kind': 'final', 'status': final})
            except (OSError, ValueError):
                pass
        return 0 if cleanup_verified else 1
    except Exception:
        # No traceback or intent/token contents are emitted by this process.
        return 1
    finally:
        if controller and not controller.stopping:
            controller.stop('controller_ipc_failed')
        if journal:
            journal.close()
        if incident_store is not None:incident_store.close()
        channel.close()


class ProcessControllerRuntime:
    """HTTP-side proxy. A failed/uncertain RPC permanently detaches this epoch."""
    def __init__(self, journal_path: str | Path, *, mode='simulate', flight_config=None):
        self.mode = ControllerMode(mode)
        self.flight_config = flight_config
        self.journal_path = Path(journal_path).absolute()
        self.replacement_lock = threading.Lock()
        if self.mode is ControllerMode.FLIGHT:
            from .flight_connection import FlightConfig
            if not isinstance(flight_config,FlightConfig):raise ValueError('flight mode requires a FlightConfig')
        elif self.mode is not ControllerMode.SIMULATE or flight_config is not None:
            raise ValueError('independent controller owner supports simulation only')
        limiter = shutil.which('prlimit')
        if not limiter:
            raise ValueError('controller process resource limiter is unavailable')
        self.controller = self
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.process = None
        self.channel = None
        self.failure = None
        self.cleanup_verified = False
        self.final_received = False
        self.closed = False
        self.sequence = 0
        self.last_response = None
        self.last_status = {}
        self.possible_airborne = False
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.channel = parent
        parent.settimeout(REQUEST_SECONDS)
        try:
            memory=(1610612736 if self.mode is ControllerMode.FLIGHT and flight_config.camera_preview else 402653184)
            command = [limiter, f'--as={memory}', '--nofile=64', '--', sys.executable,
                       '-m', 'neo2_control.control_owner', str(child.fileno()),
                       str(Path(journal_path).absolute())]
            if self.mode is ControllerMode.FLIGHT:command.append('--flight')
            self.process = subprocess.Popen(command, pass_fds=(child.fileno(),),
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
                env={'PATH': os.environ.get('PATH', '/usr/bin:/bin'), 'LC_ALL': 'C'})
            child.close()
            parent.settimeout(3)
            if self.mode is ControllerMode.FLIGHT:
                _send(parent,{'kind':'configure','mode':'flight','connection':flight_config.as_dict()})
            message = _receive(parent)
            if (set(message) != {'kind', 'epoch', 'status'} or message['kind'] != 'ready'
                or not isinstance(message['epoch'], str) or len(message['epoch']) != 48):
                raise OwnerUnavailable('controller owner startup failed')
            self.epoch = message['epoch']
            self._remember(message['status'])
            parent.settimeout(REQUEST_SECONDS)
        except Exception as error:
            child.close()
            parent.close()
            if self.process:
                # No intents have been accepted before the ready handshake.
                self.process.terminate()
                try:
                    self.process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=1)
            raise OwnerUnavailable('controller owner startup failed') from error
        self.thread = threading.Thread(target=self._heartbeat, name='neo2-owner-heartbeat', daemon=True)
        self.thread.start()

    def _remember(self, status):
        if (not isinstance(status, dict) or status.get('mode') != self.mode.value
                or status.get('synthetic') is not (self.mode is ControllerMode.SIMULATE)):
            raise OwnerUnavailable('controller owner invalid status')
        self.last_status = status
        self.last_response = time.monotonic()
        if type(status.get('possible_airborne')) is bool:
            self.possible_airborne = status['possible_airborne']

    def _detach(self):
        if not self.final_received:
            self.failure = 'controller_owner_unavailable'
        self.stopped.set()
        if self.channel:
            try:
                self.channel.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.channel.close()

    def _accept_final(self, reply, operation):
        if set(reply) != {'kind', 'status'} or reply['kind'] != 'final':
            raise OwnerUnavailable('controller owner invalid final')
        self._remember(reply['status'])
        self.final_received = True
        self.closed = True
        self.stopped.set()
        if operation in ('action','remote_action'):
            raise IntentRejected('controller_stopping')
        return {}

    def _validate_ingress(self, ingress, *, remote=False):
        if ingress is None:return
        expected=RemoteTargetIngress if remote else CommandIngress
        if not isinstance(ingress, expected):raise IntentRejected('invalid_command_ingress')
        try:
            ingress.validate(time.monotonic(), owner_epoch=self.epoch,
                connection_generation=self.last_status.get('connection', {}).get('generation'))
        except ValueError as error:raise IntentRejected(str(error)) from None

    def _request(self, operation, parameters=None, *, ingress=None):
        if self.failure or self.closed:
            raise OwnerUnavailable('controller owner unavailable')
        remote=operation in ('remote_action','remote_camera')
        self._validate_ingress(ingress,remote=remote)
        if not self.lock.acquire(timeout=REQUEST_SECONDS):
            # An ordinary WAN expiry while queued is a refused intention, not
            # evidence that the independent owner or its IPC has failed.
            self._validate_ingress(ingress,remote=remote)
            # A slow caller may not keep a live operator authority indefinitely.
            self._detach()
            raise OwnerUnavailable('controller owner unavailable')
        try:
            if self.failure or self.closed:
                raise OwnerUnavailable('controller owner unavailable')
            self._validate_ingress(ingress,remote=remote)
            # An owner may have completed its stop while the HTTP process was
            # suspended. Read its final proof before writing to a closed peer.
            if select.select([self.channel], [], [], 0)[0]:
                return self._accept_final(_receive(self.channel), operation)
            self.sequence += 1
            if ingress is not None:
                if operation not in ('action', 'camera', 'remote_action', 'remote_camera'):raise IntentRejected('invalid_command_ingress_operation')
                parameters = {**parameters, 'ingress': ingress.as_dict()}
            request = {'kind': 'request', 'epoch': self.epoch, 'id': self.sequence,
                       'expires_at': time.monotonic() + REQUEST_SECONDS,
                       'operation': operation, 'parameters': parameters or {}}
            if operation in ('action','remote_action') and parameters.get('action') == 'takeoff':
                # The child may execute even if its reply is lost.
                self.possible_airborne = True
            try:
                _send(self.channel, request)
            except OSError:
                return self._accept_final(_receive(self.channel), operation)
            reply = _receive(self.channel)
            if set(reply) == {'kind', 'status'} and reply['kind'] == 'final':
                return self._accept_final(reply, operation)
            if time.monotonic() >= request['expires_at']:
                raise OwnerUnavailable('controller owner response expired')
            if (set(reply) != {'kind', 'id', 'error', 'result', 'status'}
                or reply['kind'] != 'response' or reply['id'] != self.sequence
                or (reply['error'] is not None and not isinstance(reply['error'], str))):
                raise OwnerUnavailable('controller owner invalid response')
            self._remember(reply['status'])
            if reply['error']:
                raise IntentRejected(reply['error'])
            return reply['result']
        except IntentRejected:
            raise
        except (OSError, EOFError, ValueError, TypeError, RecursionError, OwnerUnavailable):
            # A suspended parent can resume with an expired in-flight reply
            # before the owner's already queued final. Preserve that terminal
            # proof before detaching; never wait for it or revive this epoch.
            try:
                if (not self.failure and not self.closed
                        and select.select([self.channel], [], [], 0)[0]):
                    return self._accept_final(_receive(self.channel), operation)
            except IntentRejected:
                raise
            except (OSError, EOFError, ValueError, TypeError, RecursionError, OwnerUnavailable):
                pass
            self._detach()
            raise OwnerUnavailable('controller owner unavailable') from None
        finally:
            self.lock.release()

    def _heartbeat(self):
        while not self.stopped.wait(HEARTBEAT_SECONDS):
            try:
                self._request('heartbeat')
            except OwnerUnavailable:
                self._detach()
                return

    def handle(self, action, parameters=None, *, ingress=None):
        try:
            return self._request('action', {'action': action, 'parameters': parameters}, ingress=ingress)
        except OwnerUnavailable:
            raise IntentRejected('controller_owner_unavailable') from None

    def recover_owner(self, *, ingress=None):
        """Replace only an exited owner; never terminate a retained aircraft link."""
        self._validate_ingress(ingress)
        with self.replacement_lock:
            self._validate_ingress(ingress)
            if self.mode is not ControllerMode.FLIGHT or self.process.poll() is None:
                raise IntentRejected('previous_flight_owner_still_running')
            self.stopped.set()
            self.thread.join(timeout=REQUEST_SECONDS*2+.2)
            if self.thread.is_alive():raise IntentRejected('previous_flight_owner_still_running')
            self._validate_ingress(ingress)
            if self.channel is not None:self.channel.close()
            path = self.journal_path.parent/('recovery-'+secrets.token_hex(12)+'.jsonl')
            self._validate_ingress(ingress)
            return type(self)(path,mode='flight',flight_config=self.flight_config)

    def camera_command(self,action,parameters=None, *, ingress=None):
        try:return self._request('camera',{'action':action,'parameters':{} if parameters is None else parameters}, ingress=ingress)
        except OwnerUnavailable:raise IntentRejected('controller_owner_unavailable') from None

    def remote_handle(self,action,parameters,*,ingress):
        try:return self._request('remote_action',{'action':action,'parameters':parameters},ingress=ingress)
        except OwnerUnavailable:raise IntentRejected('controller_owner_unavailable') from None

    def remote_camera_command(self,action,parameters,*,ingress):
        try:return self._request('remote_camera',{'action':action,'parameters':parameters},ingress=ingress)
        except OwnerUnavailable:raise IntentRejected('controller_owner_unavailable') from None

    def _snapshot(self):
        if not self.closed and self.process.poll() is not None:
            self._detach()
        result = copy.deepcopy(self.last_status)
        now = time.monotonic()
        age = None if self.last_response is None else now - self.last_response
        available = not self.failure and not self.closed and self.process.poll() is None
        result.update(now=now)
        if not available:
            result.update(state='fault' if self.failure else result.get('state', 'fault'),
                          connected=False, stopping=True, lease_expires_at=None,
                          pending_command_sequence=None)
            if self.failure:
                result.update(fault_reason=self.failure, landing_confirmed=False,
                              possible_airborne=True if self.possible_airborne else None)
        sample = result.get('telemetry')
        if sample:
            sample['age_seconds'] = now - sample['received_at']
            ages={}
            for name,field in (('motion','motion_received_at'),('battery','battery_received_at'),
                               ('control','control_received_at')):
                stamp=sample.get(field,sample['received_at'])
                # Legacy synthetic input has one explicit fixture clock.
                # An absent real component must remain absent at the proxy.
                if stamp is None and sample.get('synthetic') is True:stamp=sample['received_at']
                ages[name]=None if stamp is None else now-stamp
            sample['component_age_seconds']=ages
            sample['valid'] = bool(available and sample['valid']
                and all(age is not None for age in ages.values())
                and 0<=ages['motion']<=result['limits']['telemetry_age']
                and 0<=ages['battery']<=result['limits']['battery_age']
                and 0<=ages['control']<=result['limits']['telemetry_age'])
        confirmation = result.get('landing_confirmation')
        if confirmation is not None:
            # Confirmation has its own evidence lifetime. Ordinary telemetry
            # validity also requires navigation/battery preflight and cannot
            # stand in for the freshness of this landing request.
            ages = {}
            for name,field in (
                    ('request','received_at'),('motion','motion_received_at'),
                    ('control','control_received_at'),('route','route_seen_at'),
                    ('owner_route','ready_at')):
                stamp = confirmation.get(field)
                age_value = (now-stamp if type(stamp) in (int,float) and math.isfinite(stamp) else None)
                ages[name] = age_value
            fresh = all(value is not None and 0 <= value <= (2. if name == 'route' else .5)
                        for name,value in ages.items())
            # The peer beacon is strictly younger than two seconds in the
            # owner, while motion/flag/evaluation allow their exact boundary.
            fresh = fresh and ages['route'] < 2.
            confirmation['component_age_seconds'] = ages
            confirmation['evidence_available'] = bool(available and fresh and confirmation.get('evidence_available'))
            confirmation['available'] = bool(confirmation.get('available') and confirmation['evidence_available']
                                             and confirmation.get('needed') is True)
            if not available or ages['request'] is None or not 0 <= ages['request'] <= .5:
                confirmation['needed'] = None
            if not available or not fresh:
                confirmation.update(available=False,reason='controller_owner_unavailable'
                    if not available else 'landing_confirmation_evidence_stale')
        if 'connection' in result:
            result['connection']['owner_available']=available
            video=result['connection'].get('video',{})
            for key in ('last_encoded_age_seconds','preview_age_seconds'):
                if video.get(key) is not None and age is not None:video[key]+=age
            video['live']=bool(available and video.get('live') and video.get('last_encoded_age_seconds') is not None
                and video['last_encoded_age_seconds']<=2 and video.get('preview_age_seconds') is not None
                and video['preview_age_seconds']<=3)
            if not available and not self.final_received:
                result['connection'].update(phase='unavailable',cleanup_verified=False,
                    worker_alive=False if self.process.poll() is not None else None)
        result['controller_owner'] = {
            'epoch':self.epoch,
            'restart_available':self.mode is ControllerMode.FLIGHT and self.process.poll() is not None,
            'pid': self.process.pid, 'alive': self.process.poll() is None,
            'available': available, 'independent_watchdog': True,
            'scope':'simulation' if self.mode is ControllerMode.SIMULATE else 'flight',
            'parent_timeout_seconds': PARENT_TIMEOUT, 'last_response_age_seconds': age,
            'cleanup_verified': self.cleanup_verified, 'final_received': self.final_received}
        return result

    def status(self):
        if not self.closed and not self.failure:
            try:
                self._request('status')
            except (OwnerUnavailable, IntentRejected):
                self._detach()
        return self._snapshot()

    def command_context(self):
        """Return only the last acknowledged identity, without a status RPC.

        Connection generations change on serialized owner actions. Their
        replies update last_status before returning; an uncertain RPC detaches
        this epoch. The child still checks its actual generation and deadline
        for every mutation. This metadata never refreshes any observation.
        """
        if not self.lock.acquire(timeout=REQUEST_SECONDS):
            raise OwnerUnavailable('controller owner unavailable')
        try:
            return {'owner_epoch':self.epoch,
                'connection_generation':self.last_status.get('connection',{}).get('generation')}
        finally:self.lock.release()

    def close(self):
        if self.cleanup_verified:
            return
        self.stopped.set()
        self.thread.join(timeout=2 * REQUEST_SECONDS + .2)
        deadline = time.monotonic() + (FLIGHT_CLOSE_SECONDS if self.mode is ControllerMode.FLIGHT else CLOSE_SECONDS)
        if not self.failure and not self.closed:
            try:
                self._request('shutdown')
                if not self.final_received:
                    self.channel.settimeout(max(.01, deadline - time.monotonic()))
                    self._accept_final(_receive(self.channel), 'shutdown')
            except (OSError, EOFError, ValueError, TypeError, RecursionError, OwnerUnavailable, IntentRejected):
                self._detach()
        self.closed = True
        if self.channel:
            self.channel.close()
        try:
            self.process.wait(timeout=max(.01, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            # Closing IPC asks the owner to stop when it resumes. Do not kill
            # a possibly airborne owner merely to report successful cleanup.
            self.failure = 'controller_owner_unavailable'
            raise RuntimeError('controller owner cleanup unverified') from None
        self.cleanup_verified = self.process.returncode == 0 and self.final_received
        if not self.cleanup_verified:
            self.failure = 'controller_owner_unavailable'
            raise RuntimeError('controller owner cleanup unverified')


if __name__ == '__main__':
    if len(sys.argv) not in (3,4) or (len(sys.argv)==4 and sys.argv[3]!='--flight'):
        raise SystemExit(2)
    flight_config=None
    if len(sys.argv)==4:
        from .flight_connection import FlightConfig
        bootstrap=socket.socket(fileno=int(sys.argv[1]));bootstrap.settimeout(3)
        try:
            message=_receive(bootstrap)
            if set(message)!={'kind','mode','connection'} or message['kind']!='configure' or message['mode']!='flight':
                raise ValueError('invalid flight owner configuration')
            flight_config=FlightConfig(**message['connection'])
        except Exception:
            bootstrap.close();raise SystemExit(2)
        bootstrap.detach()
    raise SystemExit(run_owner(int(sys.argv[1]), sys.argv[2],flight_config=flight_config))
