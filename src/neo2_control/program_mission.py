"""Explicitly authorized PC mission: takeoff, bounded program, then LAND.

Nothing connects a drone on import or without the readiness argument. The
independent owner executes the motion program; this client only maintains its
bounded authorization and monitors the result. It never attests postflight
ground or confirms the separate final landing descent automatically.
"""
from collections import deque
import json
import math
import threading
import time

from .motion_program import MotionProgram
from .flight import MAX_YAW_CHANNEL_LIMIT
from .operator_client import OperatorError
from .delivery_time import delivery, finite, land_since, motion_stamp, server_now


def read_program(path):
    with open(path,'rb') as stream:body=stream.read(8193)
    if len(body)>8192:raise ValueError('motion_program_file_too_large')
    def pairs(items):
        result={}
        for key,value in items:
            if key in result:raise ValueError('duplicate_json_key')
            result[key]=value
        return result
    def constant(_):raise ValueError('nonfinite_number')
    return MotionProgram.from_dict(json.loads(body,object_pairs_hook=pairs,parse_constant=constant)).as_dict()


def run_program_mission(client,program,*,operator_confirmed_grounded=False,event_sink=None,
                        clock=time.monotonic,wait=None):
    if operator_confirmed_grounded is not True:raise ValueError('operator_readiness_required')
    parsed=MotionProgram.from_dict(program)
    if not finite(clock()):raise ValueError('program_clock_invalid')
    initial=client.status()
    initial_sample=initial.get('telemetry') or {}
    if not isinstance(initial_sample,dict):raise ValueError('program_response_invalid')
    if (initial.get('connected') is not True or initial.get('possible_airborne') is not False
            or initial.get('state') not in ('ready','not_ready') or initial.get('vertical_trial') is True
            or initial.get('connection',{}).get('video',{}).get('live') is not True
            or initial_sample.get('motors_on') is not False
            or initial_sample.get('flying') is not False):
        raise ValueError('program_mission_requires_connected_stopped_aircraft')
    limits=initial.get('limits') or {}
    maximum_flight=limits.get('maximum_flight_seconds')
    maximum_channel=limits.get('maximum_channel')
    if (type(maximum_flight) not in (int,float) or not math.isfinite(maximum_flight)
            or type(maximum_channel) is not int or not 1<=maximum_channel<=(MAX_YAW_CHANNEL_LIMIT if initial.get('yaw_trial') is True else 250)
            or parsed.budget_seconds+3>=min(25.,maximum_flight*.9)):
        raise ValueError('program_mission_exceeds_flight_limits')
    for step in parsed.steps:
        if any(abs(v)>maximum_channel for v in step.channels):raise ValueError('motion_channels_invalid')
        if initial.get('yaw_trial') is True and any(step.channels[:3]):
            raise ValueError('yaw_trial_requires_yaw_only')
    client.pin_connection(initial)
    identity=(initial.get('controller_owner',{}).get('epoch'),initial.get('connection',{}).get('generation'))
    wait=wait or threading.Event().wait
    def same(status):
        return (status.get('controller_owner',{}).get('epoch'),
            status.get('connection',{}).get('generation'))==identity
    sink_failed=False
    def emit(event,**fields):
        nonlocal sink_failed
        if event_sink is not None and not sink_failed:
            try:event_sink({'event':event,'monotonic':clock(),**fields})
            except Exception:
                sink_failed=True
                raise OperatorError('program_event_sink_failed') from None
    started=takeoff_reference=None
    attempted=land_attempted=False
    lease=program_id=None
    status=initial
    failure=None
    stable=deque(maxlen=32)
    def land():
        nonlocal land_attempted,status
        if land_attempted:return
        land_attempted=True
        result=client.intent('land')
        status=result.get('status',status)
        emit('land_requested')
    try:
        token=client.intent('prepare',{'operation':'ground'})['confirmation_token']
        client.intent('confirm_ground',{'confirmation_token':token})
        token=client.intent('prepare',{'operation':'arm'})['confirmation_token']
        lease=client.intent('arm',{'confirmation_token':token})['lease']
        prepared=client.intent('prepare',{'operation':'takeoff','lease':lease})
        token=prepared['confirmation_token']
        takeoff_reference=server_now(prepared.get('status') or client.status())
        started=clock()
        if not finite(started):raise OperatorError('program_clock_invalid')
        attempted=True
        result=client.intent('takeoff',{'confirmation_token':token,'lease':lease})
        status=result.get('status') or client.status()
        emit('takeoff_requested')
        while clock()-started<90.:
            if not same(status):raise OperatorError('program_generation_changed')
            now=clock();sample=status.get('telemetry') or {};state=status.get('state')
            if not finite(now) or now<started:raise OperatorError('program_clock_invalid')
            if land_since(status,takeoff_reference):land_attempted=True
            report=status.get('remote_control',{}).get('program') or {}
            if program_id is not None and report.get('program_id')!=program_id:
                raise OperatorError('motion_program_changed')
            if state in ('taking_off','flying'):
                if now-started>=min(25.,maximum_flight*.9):
                    failure='program_flight_deadline';land()
                elif program_id is None and state=='flying':
                    stamp=motion_stamp(status,now)
                    altitude=sample.get('altitude_m')
                    if type(altitude) not in (int,float) or not math.isfinite(altitude):
                        raise OperatorError('program_observation_stale')
                    if stable and (stamp<stable[-1][0] or (stamp==stable[-1][0] and altitude!=stable[-1][1])):
                        raise OperatorError('program_observation_ambiguous')
                    if not stable or stamp!=stable[-1][0]:stable.append((stamp,altitude))
                    rtt,_=delivery(status,now)
                    history=min(1.5,max(.9,.6+rtt))
                    while stable and stamp-stable[0][0]>history:stable.popleft()
                    settled=(len(stable)>1 and stamp-stable[0][0]>=.6 and altitude>=.5
                        and max(v for _,v in stable)-min(v for _,v in stable)<=.15)
                    if settled:
                        result=client.intent('program_start',{'lease':lease,'program':parsed.as_dict()})
                        program_id=result['program_id']
                        if not isinstance(program_id,str) or not 1<=len(program_id)<=128:
                            raise OperatorError('program_response_invalid')
                        status=result.get('status') or client.status()
                        emit('program_started',program_id=program_id)
                    elif now-started>=10.:failure='program_takeoff_not_stabilized';land()
                    else:status=client.intent('keepalive',{'lease':lease}).get('status') or client.status()
                else:
                    action='keepalive' if program_id is None else 'program_keepalive'
                    parameters={'lease':lease,**({} if program_id is None else {'program_id':program_id})}
                    try:status=client.intent(action,parameters).get('status') or client.status()
                    except OperatorError as error:
                        if str(error) not in ('operator_lease_invalid','motion_program_changed'):raise
                        # Completion may revoke the lease just before this
                        # renewal. Observe once; never replay that mutation.
                        status=client.status()
                        if status.get('state') in ('taking_off','flying'):raise
            elif state in ('landing','fault','ready'):
                if report.get('phase')!='motion_complete':
                    failure=failure or report.get('reason') or status.get('fault_reason') or 'program_interrupted'
                if not land_attempted and status.get('connected') and status.get('possible_airborne'):
                    land()
                if status.get('landing_confirmation',{}).get('available') is True:
                    emit('awaiting_explicit_landing_confirmation');break
                if sample.get('flying') is False and sample.get('motors_on') is False:
                    motion_stamp(status,now)
                    emit('awaiting_operator_ground_attestation');break
                status=client.status()
            else:raise OperatorError('program_unexpected_controller_state')
            wait(.1)
        else:failure='program_monitor_timeout'
    except (OperatorError,ValueError,KeyError,KeyboardInterrupt) as error:
        failure=('operator_cancelled' if isinstance(error,KeyboardInterrupt) else
            str(error) if isinstance(error,(OperatorError,ValueError)) else 'program_response_invalid')
    finally:
        if attempted and not land_attempted:
            try:
                current=client.status()
                if same(current) and current.get('connected') and current.get('possible_airborne'):
                    if not land_since(current,takeoff_reference):land()
            except (OperatorError,ValueError):pass
    report=status.get('remote_control',{}).get('program') or {}
    return {'failure':failure,'program_id':program_id,'program':report,
        'motion_complete':program_id is not None and report.get('program_id')==program_id
            and report.get('phase')=='motion_complete',
        'land_requested':land_attempted,
        'needs_landing_confirmation':status.get('landing_confirmation',{}).get('available') is True,
        'needs_operator_ground_confirmation':status.get('possible_airborne') is True
            and status.get('telemetry',{}).get('motors_on') is False,
        'last_status':status}
