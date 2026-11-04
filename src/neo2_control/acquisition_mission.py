"""Two measured-altitude panoramas; no horizontal commands or implicit I/O.

Altitudes are FC relative-altitude readings, not independently measured camera
heights. The resulting recording does not itself qualify a metric map.
"""
from collections import deque
import math
import time
from urllib.parse import urlsplit

from .operator_client import OperatorError
from .yaw_rotation import YawRotation
from .flight import ACQUISITION_VERTICAL_CHANNEL_LIMIT


def finite(value):
    return type(value) in (int,float) and math.isfinite(value)


class TwoLevelScan:
    TARGETS=(1.,1.5)

    def __init__(self,generation,now,*,buffered_feedback=False):
        self.generation=generation
        self.started=self.phase_started=self.last_tick=now
        self.index=0;self.phase='climb';self.stable=deque(maxlen=32)
        self.rotation=None;self.completed=[];self.height_started=None
        self.maximum_tick_gap=.4 if buffered_feedback else .2

    def update(self,now,altitude,yaw,received):
        if self.phase=='complete':return (0,0,0,0)
        if not all(finite(v) for v in (now,altitude,yaw,received)):
            raise ValueError('acquisition_observation_invalid')
        if not self.last_tick<=now<=self.last_tick+self.maximum_tick_gap+1e-9:
            raise ValueError('acquisition_feedback_gap')
        if not 0<=now-received<=.5:raise ValueError('acquisition_motion_stale')
        if not -180<=yaw<=180 or not 0<=altitude<1.8:raise ValueError('acquisition_observation_invalid')
        self.last_tick=now
        if now-self.started>=53.:raise ValueError('acquisition_motion_deadline')
        target=self.TARGETS[self.index];error=target-altitude
        up=0 if abs(error)<=.07 else int(math.copysign(ACQUISITION_VERTICAL_CHANNEL_LIMIT,error))
        if self.phase=='climb':
            if self.height_started is None:self.height_started=altitude
            if altitude<self.height_started-.15:raise ValueError('acquisition_climb_wrong_direction')
            if now-self.phase_started>=8.:raise ValueError('acquisition_height_timeout')
            if abs(error)<=.11:
                if not self.stable or received>self.stable[-1][0]:self.stable.append((received,altitude))
                if (len(self.stable)>=2 and received-self.stable[0][0]>=.8
                        and max(v for _,v in self.stable)-min(v for _,v in self.stable)<=.1+1e-9):
                    self.phase='rotate';self.phase_started=now
                    self.rotation=YawRotation(self.generation)
                    self.rotation.observe(yaw,received,now,generation=self.generation)
            else:self.stable.clear()
            return (0,0,up,0)
        if now-self.phase_started>=22.:raise ValueError('acquisition_rotation_timeout')
        if abs(error)>.3:raise ValueError('acquisition_height_lost')
        self.rotation.observe(yaw,received,now,generation=self.generation)
        if self.rotation.complete:
            self.completed.append({'target_altitude_m':target,'rotation':self.rotation.snapshot()})
            self.index+=1;self.stable.clear();self.height_started=None
            self.phase_started=now;self.phase='complete' if self.index==len(self.TARGETS) else 'climb'
            return (0,0,0,0)
        return (0,0,up,600 if abs(error)<=.15 else 0)

    def snapshot(self):
        return {'phase':self.phase,'level':self.index,
            'target_altitude_m':None if self.phase=='complete' else self.TARGETS[self.index],
            'rotation':None if self.rotation is None else self.rotation.snapshot(),
            'completed':self.completed,'horizontal_channels':[0,0],
            'altitude_source':'FC relative metres; physical accuracy unverified'}


def run_acquisition_mission(client,*,operator_confirmed_grounded=False,
        operator_confirmed_clear_floor=False,event_sink=None,clock=time.monotonic,wait=time.sleep):
    if operator_confirmed_grounded is not True:raise ValueError('operator_readiness_required')
    if hasattr(client,'base_url'):
        endpoint=urlsplit(client.base_url)
        if endpoint.scheme!='http' or endpoint.hostname not in ('127.0.0.1','localhost'):
            raise ValueError('acquisition_requires_pc_local_client')
    from .buffered_operator_client import BufferedFlightOperatorClient
    buffered_feedback=isinstance(client,BufferedFlightOperatorClient)
    initial=client.status();context=(initial.get('controller_owner',{}).get('epoch'),
        initial.get('connection',{}).get('generation'))
    sample=initial.get('telemetry') or {};limits=initial.get('limits') or {}
    if (initial.get('acquisition_trial') is not True or not initial.get('connected')
            or initial.get('possible_airborne') is not False
            or initial.get('state') not in ('ready','not_ready')
            or sample.get('flying') is not False or sample.get('motors_on') is not False
            or initial.get('connection',{}).get('video',{}).get('live') is not True
            or limits.get('maximum_altitude_m')!=1.8 or limits.get('maximum_flight_seconds')!=60.
            or limits.get('maximum_channel')!=600):
        raise ValueError('acquisition_requires_explicit_profile_and_stopped_aircraft')
    client.pin_connection(initial)
    def same(status):return (status.get('controller_owner',{}).get('epoch'),
        status.get('connection',{}).get('generation'))==context
    sink_failed=False
    def emit(event,**values):
        nonlocal sink_failed
        if event_sink is not None and not sink_failed:
            try:event_sink({'event':event,'monotonic':clock(),**values})
            except Exception:
                sink_failed=True
                raise OperatorError('acquisition_event_sink_failed') from None
    status=initial;lease=None;attempted=land_attempted=False;scan=None;failure=None
    began=None;sequence=0;settle=deque(maxlen=32);confirmation_sent=False
    def land():
        nonlocal status,land_attempted
        if land_attempted:return
        land_attempted=True
        response=client.intent('land');status=response.get('status') or client.status()
        emit('land_requested')
    try:
        token=client.intent('prepare',{'operation':'ground'})['confirmation_token']
        client.intent('confirm_ground',{'confirmation_token':token})
        token=client.intent('prepare',{'operation':'arm'})['confirmation_token']
        lease=client.intent('arm',{'confirmation_token':token})['lease']
        token=client.intent('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
        began=clock();attempted=True
        response=client.intent('takeoff',{'confirmation_token':token,'lease':lease})
        status=response.get('status') or client.status();emit('takeoff_requested')
        while clock()-began<90.:
            now=clock()
            if not same(status):raise OperatorError('acquisition_generation_changed')
            sample=status.get('telemetry') or {};state=status.get('state')
            if state in ('taking_off','flying') and status.get('connection',{}).get('video',{}).get('live') is not True:
                raise ValueError('acquisition_video_lost')
            received=sample.get('motion_received_at')
            if not finite(received) or not 0<=now-received<=.5:raise ValueError('acquisition_motion_stale')
            if state in ('taking_off','flying'):
                if clock()-began>=55.:failure='acquisition_flight_deadline';land()
                elif state=='flying' and sample.get('flying') is True and sample.get('motors_on') is True:
                    altitude=sample.get('altitude_m');yaw=sample.get('yaw_deg')
                    if not finite(altitude):raise ValueError('acquisition_altitude_invalid')
                    if scan is None:
                        if not settle or received>settle[-1][0]:settle.append((received,altitude))
                        while settle and received-settle[0][0]>1.:settle.popleft()
                        if (len(settle)>1 and received-settle[0][0]>=.6 and altitude>=.5
                                and max(v for _,v in settle)-min(v for _,v in settle)<=.15):
                            scan=TwoLevelScan(context[1],clock(),buffered_feedback=buffered_feedback);emit('scan_started')
                        elif clock()-began>=10.:raise ValueError('acquisition_takeoff_not_stabilized')
                    if scan is not None:
                        channels=scan.update(clock(),altitude,yaw,received)
                        emit('scan_progress',altitude_m=altitude,yaw_deg=yaw,channels=list(channels),**scan.snapshot())
                        if scan.phase=='complete':land()
                        else:
                            response=client.intent('control',{'lease':lease,'sequence':sequence,'channels':list(channels)})
                            sequence+=1;status=response.get('status') or client.status()
                    else:status=client.intent('keepalive',{'lease':lease}).get('status') or client.status()
                else:status=client.intent('keepalive',{'lease':lease}).get('status') or client.status()
            elif state in ('landing','fault','ready'):
                if state=='fault':failure=failure or status.get('fault_reason') or 'acquisition_controller_fault'
                if not land_attempted and status.get('possible_airborne'):land()
                if status.get('landing_confirmation',{}).get('available') is True:
                    if not operator_confirmed_clear_floor or confirmation_sent:break
                    confirmation_sent=True
                    token=client.intent('prepare',{'operation':'landing'})['confirmation_token']
                    response=client.intent('confirm_landing',{'confirmation_token':token})
                    status=response.get('status') or client.status();emit('clear_floor_landing_confirmed')
                if sample.get('flying') is False and sample.get('motors_on') is False:
                    emit('motors_stopped_telemetry');break
            else:raise ValueError('acquisition_unexpected_state')
            wait(.055);status=client.status()
        else:failure='acquisition_monitor_timeout'
    except (OperatorError,ValueError,KeyError,OSError,KeyboardInterrupt) as error:
        failure='operator_cancelled' if isinstance(error,KeyboardInterrupt) else str(error)
    finally:
        if attempted and not land_attempted:
            try:
                status=client.status()
                if same(status) and status.get('connected') and status.get('possible_airborne'):land()
            except (OperatorError,ValueError,OSError):pass
    return {'failure':failure,'scan':None if scan is None else scan.snapshot(),
        'motion_complete':scan is not None and scan.phase=='complete','land_requested':land_attempted,
        'landing_confirmation_sent':confirmation_sent,'last_status':status,
        'needs_landing_confirmation':status.get('landing_confirmation',{}).get('available') is True,
        'operator_ground_attestation_sent_after_flight':False,'metric_map_qualified':False}
