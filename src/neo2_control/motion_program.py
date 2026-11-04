"""Pure, bounded movement programs; this module has no device or network I/O.

Translations are timed normalized stick targets, not metre waypoints. Yaw
completion requires measured angle progress. The owner must still validate
flight authority and observations before every actuator frame and LAND when
the program finishes, fails or loses its operator authorization.
"""
from dataclasses import dataclass
import math
import re
import secrets

from .yaw_rotation import YawRotation
from .flight import MAX_YAW_CHANNEL_LIMIT


def finite(value,minimum,maximum):
    return type(value) in (int,float) and math.isfinite(value) and minimum<=value<=maximum


@dataclass(frozen=True)
class MotionStep:
    kind: str
    seconds: float
    channels: tuple[int,int,int,int]
    degrees: float | None = None
    minimum_channel: int = 40


@dataclass(frozen=True)
class MotionProgram:
    steps: tuple[MotionStep,...]

    def __post_init__(self):
        if (not isinstance(self.steps,tuple) or not 1<=len(self.steps)<=32
                or any(not isinstance(step,MotionStep) for step in self.steps)):
            raise ValueError('motion_program_invalid')
        for step in self.steps:
            if (not isinstance(step.channels,tuple) or len(step.channels)!=4
                    or any(type(v) is not int or abs(v)>250 for v in step.channels[:3])
                    or type(step.channels[3]) is not int or abs(step.channels[3])>MAX_YAW_CHANNEL_LIMIT):
                raise ValueError('motion_channels_invalid')
            if step.kind=='rotate':
                if (not finite(step.seconds,.2,25.) or not finite(step.degrees,5.,360.)
                        or any(step.channels[:3]) or not 40<=abs(step.channels[3])<=MAX_YAW_CHANNEL_LIMIT
                        or type(step.minimum_channel) is not int
                        or not 40<=step.minimum_channel<=abs(step.channels[3])):
                    raise ValueError('motion_rotation_invalid')
            elif step.kind in ('hold','translate'):
                if (not finite(step.seconds,.2,5.) or step.degrees is not None or step.channels[3]
                        or type(step.minimum_channel) is not int or step.minimum_channel!=40
                        or (step.kind=='hold' and any(step.channels))
                        or (step.kind=='translate' and not any(step.channels[:3]))):
                    raise ValueError('motion_step_invalid')
            else:raise ValueError('motion_step_invalid')
        if self.budget_seconds>30.:raise ValueError('motion_program_too_long')

    @classmethod
    def from_dict(cls,value):
        if (not isinstance(value,dict) or set(value)!={'version','steps','finish'}
                or type(value['version']) is not int or value['version']!=1 or value['finish']!='land'
                or not isinstance(value['steps'],list) or not 1<=len(value['steps'])<=32):
            raise ValueError('motion_program_invalid')
        steps=[]
        for entry in value['steps']:
            if not isinstance(entry,dict):raise ValueError('motion_step_invalid')
            kind=entry.get('kind')
            if kind=='hold' and set(entry)=={'kind','seconds'}:
                if not finite(entry['seconds'],.2,5.):raise ValueError('motion_duration_invalid')
                steps.append(MotionStep(kind,float(entry['seconds']),(0,0,0,0)))
            elif kind=='translate' and set(entry)=={'kind','seconds','right','forward','up'}:
                if not finite(entry['seconds'],.2,5.):raise ValueError('motion_duration_invalid')
                axes=tuple(entry[name] for name in ('right','forward','up'))
                if any(type(v) is not int or abs(v)>250 for v in axes) or not any(axes):
                    raise ValueError('motion_translation_invalid')
                steps.append(MotionStep(kind,float(entry['seconds']),(*axes,0)))
            elif kind=='rotate' and set(entry) in (
                    {'kind','degrees','channel','timeout_seconds'},
                    {'kind','degrees','channel','timeout_seconds','minimum_channel'}):
                if not finite(entry['degrees'],5.,360.):raise ValueError('motion_rotation_invalid')
                channel=entry['channel']
                if type(channel) is not int or not 40<=abs(channel)<=MAX_YAW_CHANNEL_LIMIT:
                    raise ValueError('motion_rotation_channel_invalid')
                if not finite(entry['timeout_seconds'],.2,25.):raise ValueError('motion_duration_invalid')
                steps.append(MotionStep(kind,float(entry['timeout_seconds']),(0,0,0,channel),
                    float(entry['degrees']),entry.get('minimum_channel',40)))
            else:raise ValueError('motion_step_invalid')
        return cls(tuple(steps))

    @property
    def budget_seconds(self):
        # Include sampled transitions in admission, rather than promising an
        # exact-duration path from an asynchronous PC/aircraft control loop.
        return sum(step.seconds+.1 for step in self.steps)

    def as_dict(self):
        entries=[]
        for step in self.steps:
            entry={'kind':step.kind}
            if step.kind=='rotate':
                entry.update(degrees=step.degrees,channel=step.channels[3],timeout_seconds=step.seconds)
                if step.minimum_channel!=40:entry['minimum_channel']=step.minimum_channel
            else:
                entry['seconds']=step.seconds
                if step.kind=='translate':entry.update(zip(('right','forward','up'),step.channels[:3]))
            entries.append(entry)
        return {'version':1,'finish':'land','steps':entries}


class MotionProgramExecution:
    def __init__(self,program,generation,started_at):
        if not isinstance(program,MotionProgram) or not program.steps:raise ValueError('motion_program_invalid')
        if not isinstance(generation,str) or not re.fullmatch('[0-9a-f]{24}',generation):
            raise ValueError('motion_program_generation_required')
        if not finite(started_at,0.,1e12):raise ValueError('motion_program_time_invalid')
        self.program=program
        self.generation=generation
        self.program_id=secrets.token_hex(12)
        self.started_at=self.step_started_at=self.last_tick=started_at
        self.deadline=started_at+program.budget_seconds
        self.index=0
        self.phase='running'
        self.reason=None
        self.rotation=None
        self.last_rotation=None
        self.completed_steps=0

    def cancel(self,reason):
        if self.phase=='running':self.phase='cancelled';self.reason=reason

    def _next(self,now):
        if self.rotation is not None:self.last_rotation=self.rotation.snapshot()
        self.completed_steps+=1
        self.index+=1
        self.step_started_at=now
        self.rotation=None
        if self.index==len(self.program.steps):self.phase='motion_complete'
        return (0,0,0,0)

    def update(self,now,observation,*,generation):
        if self.phase!='running':return (0,0,0,0)
        if generation!=self.generation:raise ValueError('motion_program_generation_changed')
        if not finite(now,self.last_tick,self.last_tick+.2+1e-9):
            raise ValueError('motion_program_tick_gap')
        self.last_tick=now
        if now>=self.deadline:raise ValueError('motion_program_deadline')
        if (observation is None or observation.flying is not True or observation.motors_on is not True):
            raise ValueError('motion_program_requires_airborne_observation')
        received=observation.motion_received_at
        if received is None:received=observation.received_at
        if not finite(received,now-.5,now):raise ValueError('motion_program_observation_stale')
        step=self.program.steps[self.index]
        elapsed=now-self.step_started_at
        if step.kind in ('hold','translate'):
            if elapsed>=step.seconds:return self._next(now)
            return step.channels
        if elapsed>=step.seconds:raise ValueError('motion_rotation_timeout')
        if self.rotation is None:
            self.rotation=YawRotation(self.generation)
            self.rotation.TARGET_DEGREES=step.degrees
        self.rotation.observe(observation.yaw_deg,received,now,generation=generation)
        if self.rotation.complete:return self._next(now)
        amplitude=abs(self.rotation.channels(abs(step.channels[3]),minimum_channel=step.minimum_channel)[3])
        return (0,0,0,amplitude if step.channels[3]>0 else -amplitude)

    def status(self):
        return {'program_id':self.program_id,'phase':self.phase,'reason':self.reason,
            'step_index':self.index,'step_count':len(self.program.steps),
            'completed_steps':self.completed_steps,'started_at':self.started_at,
            'deadline':self.deadline,'step_started_at':self.step_started_at,
            'rotation':None if self.rotation is None else self.rotation.snapshot(),
            'last_rotation':self.last_rotation,
            'translation_units':'normalized_stick_and_seconds','landing_confirmed':False}
