"""PC-side programmatic client for the bounded WAN desired-state protocol.

It deliberately uses the existing loopback-only, no-proxy/no-redirect
transport. The cloud exposes the same protocol behind its authenticated API;
its browser client handles WAN measurement independently. No automatic lease
renewal, POST retry, connection recovery or movement occurs on construction.
"""
from collections import deque
from copy import deepcopy
import math
import re
import threading
import time

from .operator_client import FlightOperatorClient, OperatorError


class BufferedFlightOperatorClient(FlightOperatorClient):
    PREFIX='/api/remote-flight/'
    PREEMPTIBLE=frozenset(('control','keepalive','program_start','program_keepalive'))

    def __init__(self,base_url='http://127.0.0.1:8794',*,timeout=1.5):
        super().__init__(base_url,timeout=timeout)
        self._initialize_protocol()
        self._expected_context=self._identity(self.status())

    def _initialize_protocol(self):
        self._round_trips=deque(maxlen=8)
        self._next_window=None
        self._state_lock=threading.RLock()
        self._mutation=threading.Lock()
        self._landing=threading.Lock()
        self._serial=0
        self._active=None
        self._landing_active=None
        self._last_delivery=None
        self._last_status=None
        self._clock=time.monotonic

    @staticmethod
    def _identity(status):
        if not isinstance(status,dict):raise OperatorError('operator_response_invalid')
        if not isinstance(status.get('controller_owner'),dict):raise OperatorError('operator_response_invalid')
        if 'connection' in status and not isinstance(status['connection'],dict):
            raise OperatorError('operator_response_invalid')
        epoch=status.get('controller_owner',{}).get('epoch')
        generation=status.get('connection',{}).get('generation')
        if (not isinstance(epoch,str) or not re.fullmatch('[0-9a-f]{48}',epoch)
                or (generation is not None and (not isinstance(generation,str)
                    or not re.fullmatch('[0-9a-f]{24}',generation)))):
            raise OperatorError('operator_connection_identity_unavailable')
        return epoch,generation

    def _exchange(self,resource,payload=None,*,serial=None):
        started=self._clock()
        result=self._json(self.PREFIX+resource,payload)
        if serial is not None:self._current(serial)
        received=self._clock();elapsed=received-started
        if not math.isfinite(elapsed) or elapsed<0:raise OperatorError('operator_clock_invalid')
        if elapsed>=2:raise OperatorError('operator_link_too_slow')
        with self._state_lock:self._round_trips.append(elapsed)
        def delivered(status):
            if not isinstance(status,dict):raise OperatorError('operator_response_invalid')
            pc_now=status.get('now');namespace=self._identity(status)
            if type(pc_now) not in (int,float) or not math.isfinite(pc_now) or pc_now<0:
                raise OperatorError('operator_response_invalid')
            with self._state_lock:
                previous=self._last_delivery
                received_at=received;round_trip=elapsed
                if previous is not None and previous[0]==namespace:
                    if pc_now<=previous[1]:
                        # Parallel reads may overtake a command's response.
                        # Keep the newer known observation and its original
                        # evidence age; the command receipt remains separate.
                        reused=deepcopy(self._last_status)
                        reused['_client_observation_reused']=True
                        return reused
                self._last_delivery=namespace,pc_now,received_at,round_trip
                value={**status,'_client_received_at':received_at,'_client_round_trip_seconds':round_trip}
                self._last_status=deepcopy(value)
                return value
        if resource=='status':result=delivered(result)
        elif 'status' in result:result={**result,'status':delivered(result['status'])}
        return result,started

    def status(self):return self._exchange('status')[0]

    def pin_connection(self,status):
        identity=self._identity(status)
        if identity[1] is None:raise OperatorError('operator_connection_identity_unavailable')
        with self._state_lock:
            if self._expected_context is not None and self._expected_context!=identity:
                raise OperatorError('operator_request_generation_changed')
            self._expected_context=identity

    def _hint(self):
        with self._state_lock:return max(self._round_trips,default=None)

    def _current(self,serial):
        with self._state_lock:
            if serial!=self._serial:raise OperatorError('operator_mutation_superseded')

    def _begin(self,endpoint,action):
        with self._state_lock:
            if self._landing_active is not None:raise OperatorError('operator_mutation_busy')
            if self._mutation.acquire(blocking=False):
                lock=self._mutation
            elif (endpoint=='intent' and action=='land' and self._active is not None
                    and (self._active[1]=='camera' or self._active[2] in self.PREEMPTIBLE)
                    and self._landing.acquire(blocking=False)):
                lock=self._landing
            else:raise OperatorError('operator_mutation_busy')
            self._serial+=1;serial=self._serial
            if lock is self._landing:self._landing_active=serial
            else:self._active=(serial,endpoint,action)
            expected=self._expected_context;previous=self._next_window;self._next_window=None
            return lock,serial,expected,previous

    @staticmethod
    def _valid_window(window):
        return (isinstance(window,dict) and isinstance(window.get('ticket'),str)
            and 0<len(window['ticket'])<=2048 and type(window.get('mutation_sequence')) is int
            and 1<=window['mutation_sequence']<(1<<53)
            and type(window.get('expires_in_seconds')) in (int,float)
            and math.isfinite(window['expires_in_seconds']) and 0<window['expires_in_seconds']<=2)

    def _submit(self,endpoint,action,parameters):
        if endpoint not in ('intent','camera') or not isinstance(action,str) or not isinstance(parameters,dict):
            raise ValueError('invalid_operator_intent')
        lock,serial,expected,previous=self._begin(endpoint,action)
        try:
            rtt=self._hint() or 0.
            age=None if previous is None else self._clock()-previous[1]
            if age is not None and age<0:raise OperatorError('operator_clock_invalid')
            if (previous is not None and self._valid_window(previous[0])
                    and age+rtt+.2<previous[0]['expires_in_seconds']):
                window,issued_after=previous
            else:window,issued_after=self._exchange('window',serial=serial)
            self._current(serial)
            if not self._valid_window(window):raise OperatorError('operator_window_invalid')
            current=(window.get('owner_epoch'),window.get('connection_generation'))
            if current!=expected:raise OperatorError('operator_request_generation_changed')
            if not 0<=self._clock()-issued_after<window['expires_in_seconds']:
                raise OperatorError('operator_window_expired_before_submit')
            result,started=self._exchange(endpoint,{'action':action,'parameters':parameters,
                'round_trip_seconds':self._hint(),'ingress':{
                    'ticket':window['ticket'],'mutation_sequence':window['mutation_sequence'],
                    'expected_owner_epoch':expected[0],'expected_connection_generation':expected[1]}},serial=serial)
            with self._state_lock:
                self._current(serial)
                identity=self._identity(result.get('status',{}))
                if identity!=expected:
                    if (endpoint!='intent' or action not in ('open','close','reconnect','recover_owner')
                            or (identity[0]!=expected[0] and action!='recover_owner')):
                        raise OperatorError('operator_request_generation_changed')
                    self._expected_context=identity
                next_window=result.get('next_window')
                if (not self._valid_window(next_window) or
                        (next_window.get('owner_epoch'),next_window.get('connection_generation'))!=identity):
                    raise OperatorError('operator_window_invalid')
                self._next_window=next_window,started
            return result
        except Exception:
            self._current(serial)
            raise
        finally:
            with self._state_lock:
                if self._active is not None and self._active[0]==serial:self._active=None
                if self._landing_active==serial:self._landing_active=None
                lock.release()
