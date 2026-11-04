"""One locally authorized yaw trial; no connection, takeoff or retry on import.

The caller connects first and obtains the operator's physical readiness. The
mission never attests landing or closes an aircraft with unresolved ground.
"""
from collections import deque
import math
import threading
import time

from .operator_client import OperatorError
from .yaw_rotation import YawRotation


def run_yaw_mission(client,*,operator_confirmed_grounded=False,event_sink=None,
                    clock=time.monotonic,wait=None):
    if operator_confirmed_grounded is not True:raise ValueError('operator_readiness_required')
    wait=wait or threading.Event().wait
    sink_failed=False
    def emit(event,**values):
        nonlocal sink_failed
        if event_sink is not None and not sink_failed:
            try:event_sink({'event':event,'monotonic':clock(),**values})
            except Exception:
                sink_failed=True
                raise OperatorError('rotation_event_sink_failed') from None
    initial=client.status()
    connection=initial.get('connection') or {}
    if (initial.get('yaw_trial') is not True or initial.get('vertical_trial') is not False
            or initial.get('state') not in ('ready','not_ready')
            or initial.get('possible_airborne') is not False or not initial.get('connected')
            or not connection.get('video',{}).get('live')):
        raise ValueError('yaw_mission_requires_connected_yaw_profile')
    generation=connection.get('generation')
    limits=initial.get('limits') or {}
    maximum_channel=limits.get('maximum_channel',200)
    maximum_flight=limits.get('maximum_flight_seconds',30.)
    control_interval=limits.get('control_interval',.05)
    if (type(maximum_channel) is not int or not 1<=maximum_channel<=200
            or type(maximum_flight) not in (int,float) or not math.isfinite(maximum_flight)
            or maximum_flight<=0 or type(control_interval) not in (int,float)
            or not math.isfinite(control_interval) or not .05<=control_interval<.2):
        raise ValueError('rotation_mission_limits_invalid')
    land_by=min(25.,maximum_flight*.9)
    owner_pid=initial.get('controller_owner',{}).get('pid')
    owner_epoch=initial.get('controller_owner',{}).get('epoch')
    client.pin_connection(initial)
    rotation=YawRotation(generation)
    def check(status):
        now=clock()
        if (status.get('connection',{}).get('generation')!=generation
                or status.get('controller_owner',{}).get('pid')!=owner_pid
                or status.get('controller_owner',{}).get('epoch')!=owner_epoch):
            raise ValueError('rotation_generation_changed')
        sample=status.get('telemetry') or {}
        for key,maximum in (('motion_received_at',.5),('control_received_at',.5),('battery_received_at',3.)):
            at=sample.get(key)
            if type(at) not in (int,float) or not math.isfinite(at) or not 0<=now-at<=maximum:
                raise ValueError('rotation_observation_stale')
        yaw=sample.get('yaw_deg')
        if type(yaw) not in (int,float) or not math.isfinite(yaw) or not -180<=yaw<=180:
            raise ValueError('rotation_yaw_invalid')
        return sample
    sample=check(initial)
    if sample.get('motors_on') is not False or sample.get('flying') is not False:
        raise ValueError('rotation_requires_stopped_aircraft')
    lease=None
    started=None
    takeoff_attempted=land_attempted=False
    airborne_seen=False
    stable=deque(maxlen=32)
    rotating=False
    sequence=0
    last_status=initial
    failure=None
    def land():
        nonlocal land_attempted
        if land_attempted:return
        land_attempted=True
        client.intent('land')
        emit('land_requested',**rotation.snapshot())
    try:
        ground=client.intent('prepare',{'operation':'ground'})
        client.intent('confirm_ground',{'confirmation_token':ground['confirmation_token']})
        prepared=client.intent('prepare',{'operation':'arm'})
        lease=client.intent('arm',{'confirmation_token':prepared['confirmation_token']})['lease']
        prepared=client.intent('prepare',{'operation':'takeoff','lease':lease})
        started=clock();takeoff_attempted=True
        client.intent('takeoff',{'confirmation_token':prepared['confirmation_token'],'lease':lease})
        emit('takeoff_requested',target_degrees=360.)
        while clock()-started<90.:
            next_poll=clock()+.1
            status=client.status();last_status=status
            sample=check(status);now=clock();state=status.get('state')
            airborne_seen|=sample.get('flying') is True
            if state in ('taking_off','flying'):
                if now-started>=land_by:
                    failure='rotation_flight_deadline';land()
                elif state=='flying' and sample.get('flying') is True and sample.get('motors_on') is True:
                    altitude=sample.get('altitude_m')
                    at=sample['motion_received_at']
                    if type(altitude) not in (int,float) or not math.isfinite(altitude):
                        raise ValueError('rotation_altitude_invalid')
                    if not rotating:
                        if not stable or stable[-1][0]!=at:stable.append((at,altitude))
                        while stable and at-stable[0][0]>.9:stable.popleft()
                        stabilized=(len(stable)>1 and at-stable[0][0]>=.6 and altitude>=.5
                            and max(v for _,v in stable)-min(v for _,v in stable)<=.15)
                        if stabilized:
                            rotation.observe(sample['yaw_deg'],at,now,generation=generation)
                            rotating=True;emit('rotation_started')
                        elif now-started>=10.:
                            failure='rotation_takeoff_not_stabilized';land()
                        else:
                            client.intent('keepalive',{'lease':lease})
                            next_poll=clock()+control_interval
                    if rotating and not land_attempted:
                        rotation.observe(sample['yaw_deg'],at,now,generation=generation)
                        if rotation.complete:land()
                        else:
                            client.intent('control',{'lease':lease,'sequence':sequence,
                                'channels':list(rotation.channels(maximum_channel))})
                            next_poll=clock()+control_interval
                            sequence+=1
                            emit('rotation_progress',**rotation.snapshot())
                else:
                    client.intent('keepalive',{'lease':lease})
                    next_poll=clock()+control_interval
            elif state in ('landing','fault'):
                if not rotation.complete:failure=failure or status.get('fault_reason') or 'rotation_interrupted'
                # A fault alone is not evidence that LAND was requested.
                requested=status.get('land_requested_at')
                previous_land=(type(requested) in (int,float) and math.isfinite(requested)
                    and started<=requested<=now)
                if previous_land:land_attempted=True
                if state=='fault':
                    failure=failure or status.get('fault_reason') or 'rotation_interrupted'
                    if (not land_attempted and status.get('connected')
                            and (sample.get('flying') is True or sample.get('motors_on') is True)):
                        land()
                if status.get('landing_confirmation',{}).get('available'):
                    emit('awaiting_explicit_landing_confirmation');break
                if sample.get('flying') is False and sample.get('motors_on') is False:
                    emit('awaiting_operator_ground_attestation');break
            else:
                failure='rotation_unexpected_controller_state';break
            # Space mutations from the previous completed request, without
            # adding a fixed 100 ms sleep to all HTTP/IPC/journal latency.
            # Each command still obtains its own original PC deadline.
            wait(max(0.,next_poll-clock()))
        else:failure='rotation_monitor_timeout'
    except (OperatorError,ValueError,KeyError) as error:
        failure=str(error) if isinstance(error,(OperatorError,ValueError)) else 'rotation_response_invalid'
        # A lost POST may have executed. One LAND is allowed only in the same
        # still-active context; the independent owner also enforces its lease.
        if takeoff_attempted and not land_attempted:
            try:
                current=client.status()
                requested=current.get('land_requested_at')
                previous_land=(type(requested) in (int,float) and math.isfinite(requested)
                    and started<=requested<=clock())
                if (current.get('connection',{}).get('generation')==generation
                        and current.get('controller_owner',{}).get('pid')==owner_pid
                        and current.get('controller_owner',{}).get('epoch')==owner_epoch
                        and current.get('connected') and current.get('possible_airborne')):
                    if previous_land:land_attempted=True
                    elif current.get('state') in ('taking_off','flying','fault'):land()
            except (OperatorError,ValueError):pass
        try:emit('rotation_interrupted',reason=failure)
        except OperatorError:pass  # A failed client journal must not block LAND.
    result={'rotation':rotation.snapshot(),'airborne_seen':airborne_seen,
        'takeoff_attempted':takeoff_attempted,'land_attempted':land_attempted,
        'failure':failure,'ground_attestation_required':takeoff_attempted,
        'connection_generation':generation,'last_status':last_status}
    try:emit('mission_monitor_finished',rotation_complete=rotation.complete,
        airborne_seen=airborne_seen,failure=failure)
    except OperatorError:result['failure']=result['failure'] or 'rotation_event_sink_failed'
    return result
