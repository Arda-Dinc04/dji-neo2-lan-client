"""Bounded WAN *desired state*, executed by the independent PC owner.

These envelopes are deliberately not CommandIngress instances. A remote
desired state can last up to two seconds; every local actuator submission
still has its own <=200 ms validity, capped by that original envelope. There
is one target slot, no timed FIFO and no catch-up after a stalled owner.
"""
from dataclasses import asdict, dataclass
import math

from .command_ingress import CommandIngress, CommandWindows, _UNSPECIFIED

MAX_REMOTE_SECONDS = 2.
MIN_REMOTE_SECONDS = .5


def target_horizon(round_trip_seconds=None):
    """Two measured HTTP round trips plus 400 ms scheduling/jitter allowance.

    The first ticket uses the hard maximum. Measurements are untrusted sizing
    hints, never permission to exceed the PC's fixed limit. Each ticket keeps
    its original deadline even when a later RTT estimate increases.
    """
    if round_trip_seconds is None:return MAX_REMOTE_SECONDS
    if (type(round_trip_seconds) not in (float,int)
            or not math.isfinite(round_trip_seconds) or round_trip_seconds < 0):
        raise ValueError('remote_round_trip_invalid')
    return min(MAX_REMOTE_SECONDS,max(MIN_REMOTE_SECONDS,2*round_trip_seconds+.4))


@dataclass(frozen=True)
class RemoteTargetIngress:
    owner_epoch: str
    connection_generation: str | None
    mutation_sequence: int
    issued_at: float
    expires_at: float
    kind: str = 'remote_target_v1'

    def __post_init__(self):
        # Reuse the strict identities/sequence/time checks, without broadening
        # CommandIngress's native 250 ms maximum or allowing cross-domain IPC.
        if (type(self.expires_at) not in (float,int)
                or not math.isfinite(self.expires_at)
                or type(self.issued_at) not in (float,int)
                or not math.isfinite(self.issued_at)
                or not 0 < self.expires_at-self.issued_at <= MAX_REMOTE_SECONDS+1e-9
                or self.kind != 'remote_target_v1'):
            raise ValueError('operator_request_invalid')
        CommandIngress(self.owner_epoch,self.connection_generation,self.mutation_sequence,
            self.issued_at,min(self.expires_at,self.issued_at+.2))

    def validate(self,now,*,owner_epoch=_UNSPECIFIED,connection_generation=_UNSPECIFIED):
        return CommandIngress.validate(self,now,owner_epoch=owner_epoch,
            connection_generation=connection_generation)

    def as_dict(self):return asdict(self)

    @classmethod
    def from_dict(cls,value):
        if not isinstance(value,dict) or set(value)!=set(cls.__dataclass_fields__):
            raise ValueError('operator_request_invalid')
        return cls(**value)

    def local_execution(self,now):
        """A new PC plan step; the source envelope is never extended."""
        self.validate(now)
        return CommandIngress(self.owner_epoch,self.connection_generation,self.mutation_sequence,
            now,min(now+.2,self.expires_at))


class RemoteTargetWindows(CommandWindows):
    context_type = RemoteTargetIngress

    def issue(self,owner_epoch,connection_generation,*,round_trip_seconds=None):
        return self._issue(owner_epoch,connection_generation,target_horizon(round_trip_seconds))


class RemoteTargetRunner:
    """Owner-thread plan runner; HTTP threads cannot pump or renew targets.

    Normal core rules still admit every locally generated step. Source expiry,
    generation changes, local takeover, LAND and parent failure all revoke the
    slot. Keepalive explicitly replaces any nonzero desired state with neutral.
    """
    def __init__(self,controller,epoch):
        self.controller=controller
        self.epoch=epoch
        self.source=self.lease=self.channels=None
        self.operator_sequence=-1
        self.program=None
        self.last_program=None

    def _generation(self):
        status=getattr(self.controller.backend,'connection_status',None)
        return status().get('generation') if callable(status) else None

    def _validate(self,source):
        from .controller import IntentRejected
        if not isinstance(source,RemoteTargetIngress):raise IntentRejected('invalid_remote_ingress')
        generation=self._generation()
        try:return source.validate(self.controller._now(),owner_epoch=self.epoch,
            connection_generation=generation)
        except ValueError as error:raise IntentRejected(str(error)) from None

    def _local(self,source):
        from .controller import IntentRejected
        try:return source.local_execution(self.controller._now())
        except ValueError as error:raise IntentRejected(str(error)) from None

    def _cap_lease(self):
        c=self.controller
        # A handle() call can finish journaling just as _pump() notices a
        # deadline and revokes the lease. Its accepted return is not authority
        # to resurrect a revoked lease or assume a deadline still exists.
        if c.lease!=self.lease or c.lease_deadline is None:
            self.clear('local_authority_revoked');return False
        c.lease_deadline=min(c.lease_deadline,self.source.expires_at)
        return True

    def clear(self,reason):
        self._cancel_program(reason)
        if self.source is not None:
            self.controller._event(self.controller._now(),'remote_target_cleared',reason=reason)
        self.source=self.lease=self.channels=None
        self.operator_sequence=-1

    def _cancel_program(self,reason):
        if self.program is not None:
            self.program.cancel(reason)
            self.last_program=self.program.status()
            self.program=None

    def _program_generation(self):
        generation=self._generation()
        # A simulator has no aircraft session. Its owner epoch supplies an
        # explicitly synthetic binding; real flight always requires the radio
        # generation and cannot fall back to an invented aircraft identity.
        if generation is None and self.controller.mode.value=='simulate':return self.epoch[:24]
        return generation

    def status(self):
        return {'active':self.source is not None,
            'expires_at':None if self.source is None else self.source.expires_at,
            'source_sequence':None if self.source is None else self.source.mutation_sequence,
            'channels':None if self.channels is None else list(self.channels),
            'program':self.program.status() if self.program is not None else self.last_program,
            'maximum_seconds':MAX_REMOTE_SECONDS}

    def wait_seconds(self,maximum):
        """Wake for the next local step, without renewing any authority.

        A source handoff can leave the previous frame only a few milliseconds
        beyond the next rate-limited admission. A fixed sleep after journal
        work must not skip that admission while the new source is still live.
        """
        c=self.controller
        with c.lock:
            if self.source is None:return maximum
            now=c._now();deadlines=[self.source.expires_at]
            if c.lease_deadline is not None and c.lease_deadline<self.source.expires_at:
                deadlines.append(c.lease_deadline-.2)
            if c.state=='flying' and self.channels is not None:
                deadlines.append(now if c.last_control_intent is None else
                    c.last_control_intent+c.limits.control_interval)
            return max(0.,min(maximum,min(deadlines)-now))

    def _admit_program(self,value,now):
        from .controller import IntentRejected
        from .motion_program import MotionProgram, MotionProgramExecution
        c=self.controller
        if self.program is not None:raise IntentRejected('motion_program_active')
        if c.state!='flying':raise IntentRejected('motion_program_requires_flight')
        if c.vertical_trial:raise IntentRejected('vertical_trial_control_disabled')
        try:
            program=MotionProgram.from_dict(value)
            for step in program.steps:c._validate_channels(step.channels)
            if c.flight_deadline is None or now+program.budget_seconds>=c.flight_deadline:
                raise IntentRejected('motion_program_exceeds_flight_deadline')
            return MotionProgramExecution(program,self._program_generation(),now)
        except ValueError as error:raise IntentRejected(str(error)) from None

    def handle(self,action,parameters,source):
        from .controller import IntentRejected
        c=self.controller
        with c.lock:
            c._update(c._now())
            self._validate(source)
            if not isinstance(parameters,dict):raise IntentRejected('invalid_parameters')
            if c.stopping:raise IntentRejected('controller_stopping')
            if not c.journal_healthy:raise IntentRejected('journal_unavailable')
            channels=self.channels
            operator_sequence=self.operator_sequence
            next_program=None
            if action in ('program_start','program_keepalive'):
                fields={'lease','program'} if action=='program_start' else {'lease','program_id'}
                if set(parameters)!=fields:raise IntentRejected('invalid_parameters')
                c._check_lease(parameters['lease'],c._now())
                if action=='program_start':
                    next_program=self._admit_program(parameters['program'],c._now())
                    channels=(0,0,0,0)
                elif (self.program is None or parameters['program_id']!=self.program.program_id):
                    raise IntentRejected('motion_program_changed')
                result={'accepted':True,'mode':c.mode.value,'state':c.state,
                    'program_id':(next_program or self.program).program_id}
            elif action=='control':
                if set(parameters)!={'lease','sequence','channels'}:raise IntentRejected('invalid_parameters')
                c._check_lease(parameters['lease'],c._now())
                if c.state!='flying':raise IntentRejected('invalid_state')
                if c.vertical_trial:raise IntentRejected('vertical_trial_control_disabled')
                channels=parameters['channels'];sequence=parameters['sequence']
                c._validate_channels(channels)
                if (type(sequence) is not int or not 0<=sequence<1<<31
                        or (parameters['lease']==self.lease and sequence<=self.operator_sequence)):
                    raise IntentRejected('control_sequence_replayed_or_invalid')
                # Copy untrusted mutable input before accepting the slot.
                channels=tuple(channels)
                operator_sequence=sequence
                result={'accepted':True,'mode':c.mode.value,'state':c.state}
            else:
                if action=='keepalive' and self.program is not None:
                    raise IntentRejected('motion_program_active')
                if action in ('land','close','disarm','reconnect','reset','confirm_landing','confirm_ground'):
                    self.clear(action)
                result=c.handle(action,parameters,ingress=self._local(source))
                preparing_takeoff=(action=='prepare' and parameters.get('operation')=='takeoff')
                if action not in ('arm','keepalive','takeoff') and not preparing_takeoff:return result
                channels=None if c.vertical_trial else (0,0,0,0)
                if action=='arm':operator_sequence=-1
            self._validate(source)
            if c.lease is None or c.lease_deadline is None:
                self.clear('local_authority_revoked')
                raise IntentRejected('operator_lease_invalid')
            if action=='control':self._cancel_program('manual_control')
            if next_program is not None:
                self.program=next_program
                c._event(c._now(),'motion_program_started',program_id=self.program.program_id,
                    steps=len(self.program.program.steps),budget_seconds=self.program.program.budget_seconds)
            self.source=source
            self.lease=c.lease
            self.channels=channels
            self.operator_sequence=operator_sequence
            # Authority is renewed only while this immutable source is valid.
            # The native lease never extends beyond its remote authorization.
            c.lease_deadline=min(c.lease_deadline,source.expires_at)
            c._event(c._now(),'remote_target_accepted',source_sequence=source.mutation_sequence,
                expires_at=source.expires_at,channels=None if self.channels is None else list(self.channels))
            return result

    def tick(self):
        from .controller import IntentRejected
        c=self.controller
        with c.lock:
            if self.source is None:return
            if c.stopping or c.fault_reason or c.lease!=self.lease or c.state not in ('armed','taking_off','flying'):
                self.clear('local_authority_revoked');return
            try:self._validate(self.source)
            except IntentRejected:
                self.clear('source_expired_or_changed')
                c._failsafe(c._now(),'remote_target_expired')
                return
            now=c._now()
            try:
                if self.program is not None:
                    c._update(now)
                    if c.fault_reason or c.state!='flying' or c.lease!=self.lease:
                        self.clear('local_authority_revoked');return
                    generation=self._program_generation()
                    self.channels=self.program.update(c._now(),c.last_sample,generation=generation)
                    if self.program.phase=='motion_complete':
                        local=self._local(self.source)
                        self.clear('program_motion_complete')
                        c.handle('land',{},ingress=local)
                        return
                    now=c._now()
                # A due control already renews the local lease. Admit it
                # before spending journal flushes on a redundant keepalive:
                # the previous control may be close to its original source
                # deadline even though a newer source is already authorized.
                if (c.state=='flying' and self.channels is not None
                        and (c.last_control_intent is None
                            or c._now()-c.last_control_intent>=c.limits.control_interval-1e-9)):
                    local=self._local(self.source)
                    c.handle('control',{'lease':self.lease,'sequence':c.last_control_sequence+1,
                        'channels':self.channels,'expires_at':local.expires_at},ingress=local)
                    if not self._cap_lease():return
                # Status and parent heartbeats still never renew authority.
                if (c.lease_deadline-c._now()<=.2
                        and c.lease_deadline<self.source.expires_at):
                    c.handle('keepalive',{'lease':self.lease},ingress=self._local(self.source))
                    self._cap_lease()
            except (IntentRejected,ValueError) as error:
                reason=('remote_target_expired' if str(error) in
                    ('operator_request_expired','operator_request_generation_changed')
                    else 'remote_target_step_rejected')
                if self.program is not None:
                    self._cancel_program(str(error))
                    if reason!='remote_target_expired':reason='motion_program_failed'
                self.clear('local_step_rejected')
                c._failsafe(c._now(),reason)
