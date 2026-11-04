"""WAN desired-state execution with a synthetic aircraft; no RF or motors."""
from dataclasses import FrozenInstanceError
import json

import pytest

from neo2_control.command_ingress import CommandIngress, CommandWindows
from neo2_control.controller import IntentRejected
from neo2_control.flight import FlightAction
from neo2_control.remote_target import (
    RemoteTargetIngress, RemoteTargetRunner, RemoteTargetWindows, target_horizon,
)
from test_controller import factory, advance, actions

EPOCH='a'*48


def carrier(window):
    return {'ticket':window['ticket'],'mutation_sequence':window['mutation_sequence'],
        'expected_owner_epoch':window['owner_epoch'],
        'expected_connection_generation':window['connection_generation']}


@pytest.mark.parametrize('rtt,seconds',[(None,2.),(0,.5),(.183,.766),(.221,.842),
    (.55,1.5),(.624,1.648),(100,2.)])
def test_horizon_uses_full_round_trip_and_remains_bounded(rtt,seconds):
    assert target_horizon(rtt)==pytest.approx(seconds)


@pytest.mark.parametrize('rtt',[True,-.1,float('nan'),float('inf'),'0.2',[],{}])
def test_invalid_latency_is_not_authority(rtt):
    with pytest.raises(ValueError):target_horizon(rtt)


def test_remote_windows_single_use_original_expiry_and_native_domain_isolation():
    now=[100.]
    windows=RemoteTargetWindows(clock=lambda:now[0])
    native=CommandWindows(clock=lambda:now[0])
    first=windows.issue(EPOCH,None,round_trip_seconds=.55)
    assert first['expires_in_seconds']==pytest.approx(1.5)
    now[0]=100.8
    source=windows.consume(carrier(first),owner_epoch=EPOCH,connection_generation=None)
    assert source.issued_at==100 and source.expires_at==pytest.approx(101.5)
    with pytest.raises(ValueError,match='replayed'):
        windows.consume(carrier(first),owner_epoch=EPOCH,connection_generation=None)
    with pytest.raises(ValueError,match='invalid'):
        native.consume(carrier(first),owner_epoch=EPOCH,connection_generation=None)
    with pytest.raises(ValueError):CommandIngress.from_dict(source.as_dict())
    with pytest.raises(ValueError):RemoteTargetIngress.from_dict(source.local_execution(now[0]).as_dict())
    with pytest.raises(FrozenInstanceError):source.expires_at=1000
    later=windows.issue(EPOCH,None,round_trip_seconds=5)
    assert later['expires_in_seconds']==2 and source.expires_at==pytest.approx(101.5)
    now[0]=101.5
    with pytest.raises(ValueError,match='expired'):source.local_execution(now[0])


@pytest.mark.parametrize('values',[
    {'kind':'native'},{'expires_at':103.},{'issued_at':101.},
    {'mutation_sequence':True},{'owner_epoch':'invalid'},
    {'connection_generation':'invalid'}, {'expires_at':float('nan')},
])
def test_malformed_remote_context(values):
    context=dict(owner_epoch=EPOCH,connection_generation=None,mutation_sequence=1,
        issued_at=100.,expires_at=101.)
    with pytest.raises(ValueError):RemoteTargetIngress(**{**context,**values})


def remote_flight(factory):
    c,b,clock,journal=factory()
    runner=RemoteTargetRunner(c,EPOCH)
    windows=RemoteTargetWindows(clock=clock)
    def issue():
        return windows.consume(carrier(windows.issue(EPOCH,None,round_trip_seconds=.55)),
            owner_epoch=EPOCH,connection_generation=None)
    def command(action,p=None):return runner.handle(action,p or {},issue())
    def run(seconds):
        end=clock.now+seconds
        while clock.now<end-1e-9:
            clock.now=min(clock.now+.025,end)
            runner.tick();c.tick()
    command('open')
    token=command('prepare',{'operation':'arm'})['confirmation_token']
    lease=command('arm',{'confirmation_token':token})['lease']
    run(.55)  # A WAN round trip exceeds the native 500 ms lease.
    assert c.state=='armed'
    token=command('prepare',{'operation':'takeoff','lease':lease})['confirmation_token']
    run(.55)  # Preparation also spans one full request on the real cloud path.
    command('takeoff',{'confirmation_token':token,'lease':lease})
    run(.4)
    assert c.state=='flying'
    return c,b,clock,journal,runner,lease,issue,command,run


def test_long_rtt_preserves_cadence_without_lengthening_native_frames(factory):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    source=issue()
    run(.275)  # Return half of the window RTT before the gesture reaches PC.
    runner.handle('control',{'lease':lease,'sequence':0,'channels':[0,0,0,100]},source)
    run(.55)
    assert c.state=='flying' and not c.fault_reason
    assert runner.source is source
    assert c.control[1]<=source.expires_at
    events=list(map(json.loads,journal.path.read_text().splitlines()))
    normal=[e for e in events if e['event']=='packet_intent' and not e['emergency']]
    controls=[p for t,p in b.sent if (p.cmd_set,p.cmd_id)==(1,0x0a)]
    assert len(controls)>=8
    assert normal and runner.status()['channels']==[0,0,0,100]
    assert c.lease_deadline<=source.expires_at
    # Sending a newer target replaces the slot; no queue of old controls.
    command('control',{'lease':lease,'sequence':1,'channels':[0,0,0,-100]})
    run(.1)
    assert runner.status()['channels']==[0,0,0,-100]
    assert c.control[0]==(0,0,0,-100)


def test_source_loss_requests_one_land_and_never_replays_after_resume(factory):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    command('control',{'lease':lease,'sequence':1,'channels':[0,0,0,100]})
    expiry=runner.source.expires_at
    run(expiry-clock.now+.05)
    assert c.state=='landing' and c.lease is None and not runner.status()['active']
    assert c.fault_reason=='remote_target_expired'
    assert actions(b)==[FlightAction.TAKEOFF,FlightAction.LAND]
    before=len(b.sent)
    run(.5)
    assert len(b.sent)==before
    with pytest.raises(IntentRejected):
        command('control',{'lease':lease,'sequence':2,'channels':[0,0,0,100]})


def test_land_flushes_slot_and_keepalive_never_extends_nonzero_target(factory):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    command('control',{'lease':lease,'sequence':1,'channels':[0,0,0,100]});run(.075)
    command('keepalive',{'lease':lease});run(.075)
    assert c.control[0]==(0,0,0,0)
    command('land')
    assert not runner.status()['active'] and c.control is None
    before=len(b.sent);run(.4)
    assert len(b.sent)==before
    assert c.landing_confirmed and c.state=='ready'


@pytest.mark.parametrize('problem',['battery','position','motion','parent','generation','stall'])
def test_remote_target_cannot_override_core_guards(factory,problem):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    command('control',{'lease':lease,'sequence':1,'channels':[0,0,0,100]});run(.075)
    if problem=='battery':b.battery_percent=10
    elif problem=='position':b.position_valid=False
    elif problem=='motion':b.observation=lambda:None
    elif problem=='parent':c.stop('controller_parent_timeout')
    elif problem=='generation':b.connection_status=lambda:{'generation':'b'*24}
    elif problem=='stall':clock.now+=.3
    run(.075)
    assert c.state in ('landing','fault') and c.fault_reason
    assert actions(b)==[FlightAction.TAKEOFF,FlightAction.LAND]
    run(.075)
    assert not runner.status()['active']


@pytest.mark.parametrize('parameters',[
    {'sequence':True,'channels':[0,0,0,100]},
    {'sequence':1,'channels':[0,0,0,251]},
    {'sequence':1,'channels':[True,0,0,0]},
    {'sequence':1,'channels':[0,0,0,100],'expires_at':1e9},
])
def test_invalid_target_does_not_replace_last_desired_state(factory,parameters):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    command('control',{'lease':lease,'sequence':0,'channels':[0,0,0,50]})
    source=runner.source
    with pytest.raises(IntentRejected):command('control',{'lease':lease,**parameters})
    assert runner.source is source and runner.status()['channels']==[0,0,0,50]


def test_replayed_control_and_mutable_caller_cannot_change_slot(factory):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    channels=[0,0,0,100]
    command('control',{'lease':lease,'sequence':5,'channels':channels})
    channels[3]=-100
    with pytest.raises(IntentRejected,match='replayed'):
        command('control',{'lease':lease,'sequence':4,'channels':channels})
    run(.075)
    assert c.control[0]==(0,0,0,100)


@pytest.mark.parametrize('renewal',['control','keepalive'])
def test_deadline_crossed_while_journaling_cannot_revive_lease_or_crash_owner(factory,monkeypatch,renewal):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    command('control',{'lease':lease,'sequence':0,'channels':[0,0,0,100]})
    run(.075)
    # Hold the source boundary just beyond the next keepalive. Its journal
    # flush crosses that boundary before core.handle() pumps the old target.
    clock.now=runner.source.expires_at-.01
    c.lease_deadline=runner.source.expires_at
    c.control=(c.control[0],runner.source.expires_at)
    if renewal=='keepalive':
        # A keepalive is useful only if a newer source can extend the lease.
        # The old local frame still expires while that renewal is journaled.
        runner.source=issue()
        c.last_control_intent=clock.now
    original=journal.record
    def delayed(event):
        original(event)
        if event['event']=='intent_accepted' and event.get('action')==renewal:clock.now+=.02
    monkeypatch.setattr(journal,'record',delayed)
    runner.tick()
    assert c.state=='landing' and c.lease is None and not runner.status()['active']
    assert actions(b)==[FlightAction.TAKEOFF,FlightAction.LAND]
    advance(c,clock,.5)
    assert c.landing_confirmed


def test_new_source_control_precedes_redundant_slow_keepalive(factory,monkeypatch):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    command('control',{'lease':lease,'sequence':0,'channels':[0,0,0,100]})
    run(.075)
    # Match a real handoff: the old local control/lease have 40 ms left,
    # but a new operator source already authorizes the same desired target.
    clock.now+=.05
    # Keep the original control live at admission, while making a new local
    # step due. The source's timestamps are never rewritten by the runner.
    c.control=(c.control[0],clock.now+.04)
    c.lease_deadline=clock.now+.04
    source=issue()
    runner.handle('control',{'lease':lease,'sequence':1,'channels':[0,0,0,100]},source)
    original=journal.record;renewals=[]
    def delayed(event):
        original(event)
        if event['event']=='intent_accepted':
            renewals.append(event.get('action'))
            if event.get('action')=='keepalive':clock.now+=.06
    monkeypatch.setattr(journal,'record',delayed)
    runner.tick()
    assert renewals==['control']
    assert c.state=='flying' and not c.fault_reason
    assert c.control[1]>clock.now+.04 and c.control[1]<=source.expires_at
    assert c.control[1]<=clock.now+.2+1e-9


@pytest.mark.parametrize('fixed_sleep',[False,True])
def test_owner_wakes_for_due_control_before_a_short_previous_source_deadline(factory,fixed_sleep):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    command('control',{'lease':lease,'sequence':0,'channels':[0,0,0,100]})
    run(.075)
    admitted=c.last_control_intent
    # Reproduce the recorded handoff: a newer source exists, but its first
    # local step is rate limited until 50 ms after the preceding admission.
    c.control=(c.control[0],admitted+.068)
    clock.now=admitted+.038
    command('control',{'lease':lease,'sequence':1,'channels':[0,0,0,100]})
    runner.tick()
    clock.now+=.006  # Work already spent in this owner iteration.
    delay=runner.wait_seconds(.025)
    assert delay==pytest.approx(.006)
    clock.now+=.025 if fixed_sleep else delay
    runner.tick();c.tick()
    if fixed_sleep:
        # The previous fixed sleep reproduces the observed premature LAND.
        assert c.state=='landing' and c.fault_reason=='control_expired'
        assert actions(b)==[FlightAction.TAKEOFF,FlightAction.LAND]
        return
    assert c.state=='flying' and not c.fault_reason
    assert c.control[1]>admitted+.068
    assert c.control[1]<=min(clock.now+.2,runner.source.expires_at)+1e-9
    # A genuine owner stall still expires the local frame; no catch-up.
    clock.now+=.201;runner.tick();c.tick()
    assert c.state=='landing' and c.fault_reason=='control_expired'
    assert actions(b)==[FlightAction.TAKEOFF,FlightAction.LAND]


def test_keepalive_does_not_repeat_when_the_original_source_already_caps_the_lease(factory):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    command('control',{'lease':lease,'sequence':0,'channels':[0,0,0,100]})
    run(.075)
    clock.now=runner.source.expires_at-.15
    c.lease_deadline=runner.source.expires_at
    c.control=(c.control[0],runner.source.expires_at)
    c.last_control_intent=clock.now
    before=len(journal.path.read_text().splitlines())
    runner.tick()
    assert len(journal.path.read_text().splitlines())==before
    assert c.lease_deadline==runner.source.expires_at
    assert c.state=='flying' and not c.fault_reason
    clock.now=runner.source.expires_at;runner.tick();c.tick()
    assert c.state=='landing' and c.fault_reason=='remote_target_expired'


def test_source_expiring_before_local_conversion_is_a_refusal(factory,monkeypatch):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    source=issue()
    original=source.local_execution
    def expired(now):
        clock.now=source.expires_at
        return original(clock.now)
    monkeypatch.setattr(RemoteTargetIngress,'local_execution',lambda self,now:expired(now))
    with pytest.raises(IntentRejected,match='expired'):
        runner.handle('keepalive',{'lease':lease},source)


def test_waiting_for_generation_lock_cannot_freeze_source_age(factory,monkeypatch):
    c,b,clock,journal,runner,lease,issue,command,run=remote_flight(factory)
    source=issue()
    def delayed_generation():clock.now=source.expires_at;return None
    monkeypatch.setattr(runner,'_generation',delayed_generation)
    with pytest.raises(IntentRejected,match='expired'):runner._validate(source)
