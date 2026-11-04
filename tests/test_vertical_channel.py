"""Explicit vertical-only transport policy, no real hardware."""
from dataclasses import replace

import pytest

from neo2_control.control_transport import FlightChannelBackend
from neo2_control.flight import FlightAction
from neo2_control.flight_channel import FlightChannel
from neo2_control.flight_connection import FlightConfig,OwnedFlightBackend
from test_landing_supervisor import LandingWorld


def world(*,vertical=True):
    w=LandingWorld()
    w.channel=FlightChannel(clock=lambda:w.now,vertical_trial=vertical)
    w.backend=FlightChannelBackend(w.channel,synthetic=True)
    w.channel.bind_transport('127.0.0.1',9003)
    w.gps=False
    w.feed()
    return w


@pytest.mark.parametrize('vertical',[False,True])
def test_actual_takeoff_gate_distinguishes_explicit_profile_without_inventing_navigation(vertical):
    w=world(vertical=vertical)
    w.attest()
    sample=w.backend.observation()
    assert sample.position_valid is False and sample.distance_m is None
    assert not w.channel.observer.permits_control(w.now)
    intent=w.channel.submit_action(FlightAction.TAKEOFF,expires_at=w.now+.2)
    packet=w.channel.next_packet(w.now,w.sequence.next)
    assert packet.payload==b'\1'
    assert w.channel.can_send(packet.seq,w.now) is vertical
    assert w.backend.observation().position_valid is False
    assert w.backend.observation().distance_m is None
    if vertical:
        w.channel.sent(packet.seq,w.now)
        assert w.channel.receipt(intent['intent_id'])['state']=='sent'
    else:assert w.channel.receipt(intent['intent_id'])['state']=='cancelled'


@pytest.mark.parametrize('loss',['motors','flying','battery','motion','host','altitude','route','expiry'])
def test_vertical_takeoff_rechecks_all_retained_guards_at_send(loss):
    w=world()
    intent=w.channel.submit_action(FlightAction.TAKEOFF,expires_at=w.now+.2)
    packet=w.channel.next_packet(w.now,w.sequence.next)
    sample=w.channel.observer.sample
    if loss=='motors':w.channel.observer.sample=replace(sample,motors_on=True)
    elif loss=='flying':w.channel.observer.sample=replace(sample,flying=True)
    elif loss=='battery':w.channel.observer.sample=replace(sample,battery_percent=19)
    elif loss=='motion':
        w.now+=.51
        w.channel.set_ready(True,w.now)
    elif loss=='host':
        route=w.channel.observer.route
        w.channel.observer.route=replace(route,host_control=replace(route.host_control,committed_at=None))
    elif loss=='altitude':w.channel.observer.sample=replace(sample,altitude_m=1.6)
    elif loss=='route':w.channel.set_ready(False,w.now)
    else:w.now+=.21
    assert not w.channel.can_send(packet.seq,1.)  # owner may have waited for lock
    assert w.channel.sent_count==0
    assert w.channel.receipt(intent['intent_id'])['state'] in ('expired','cancelled')


def test_profile_rejects_all_direct_sticks_but_retains_neutral_before_land():
    w=world()
    for channels in ([0]*4,[1,0,0,0],[0,0,1,0]):
        with pytest.raises(ValueError,match='directional_control_disabled'):
            w.channel.submit_control(channels,expires_at=w.now+.2)
    w.channel.submit_landing(expires_at=w.now+.2)
    neutral=w.channel.next_packet(w.now,w.sequence.next)
    assert (neutral.cmd_set,neutral.cmd_id)==(1,0xa) and w.channel.can_send(neutral.seq,w.now)
    w.channel.sent(neutral.seq,w.now)
    landing=w.channel.next_packet(w.now,w.sequence.next)
    assert landing.payload==b'\2' and w.channel.can_send(landing.seq,w.now)


def test_config_roundtrip_latches_explicit_profile_in_owned_channel_without_opening_hardware(tmp_path):
    config=FlightConfig('fixture0',tmp_path/'wlm.json',tmp_path/'ble.json',vertical_trial=True)
    decoded=FlightConfig(**config.as_dict())
    backend=OwnedFlightBackend(decoded)
    assert backend.channel.vertical_trial is True
    assert backend.connection.thread is None and not backend.connected
    assert not backend.channel.snapshot()['ordinary_control_ready']


@pytest.mark.parametrize('value',[None,0,1,'true'])
def test_config_and_channel_reject_nonboolean_profile(value,tmp_path):
    with pytest.raises(ValueError):FlightChannel(vertical_trial=value)
    with pytest.raises(ValueError):FlightConfig('fixture0',tmp_path/'w',tmp_path/'b',vertical_trial=value)


@pytest.mark.parametrize('field,value',[
    ('minimum_battery_percent',19),('minimum_battery_percent',True),
    ('maximum_altitude_m',1.6),('maximum_altitude_m',float('nan'))])
def test_caller_cannot_loosen_vertical_dispatch_bounds(field,value):
    w=world()
    bounds={'minimum_battery_percent':20.,'maximum_altitude_m':1.5}
    bounds[field]=value
    with pytest.raises(ValueError,match='vertical_trial_limits_invalid'):
        w.channel.submit_action(FlightAction.TAKEOFF,expires_at=w.now+.2,vertical_limits=bounds)


@pytest.mark.parametrize('field,value',[
    ('battery_percent',29),('altitude_m',.6)])
def test_actual_send_retains_stricter_supervisor_bounds(field,value):
    w=world()
    w.channel.submit_action(FlightAction.TAKEOFF,expires_at=w.now+.2,
        vertical_limits={'minimum_battery_percent':30.,'maximum_altitude_m':.5})
    packet=w.channel.next_packet(w.now,w.sequence.next)
    w.channel.observer.sample=replace(w.channel.observer.sample,**{field:value})
    assert not w.channel.can_send(packet.seq,w.now)
