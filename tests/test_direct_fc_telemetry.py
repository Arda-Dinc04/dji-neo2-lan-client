"""Measured direct FC03/43 transition, with explicit socket-owner provenance."""
from dataclasses import replace

import pytest

from neo2_control.battery import BatteryReceiver
from neo2_control.duml import DumlPacket
from neo2_control.flight_observation import FlightObserver, FlightRoute
from neo2_control.telemetry import TelemetryReceiver
from test_host_control import state
from test_observation_envelope import carrier as battery_carrier
from test_telemetry import body, frame
from test_telemetry_wifi_profile import carrier


@pytest.mark.parametrize('recipient',[0xe,0xaa])
@pytest.mark.parametrize('kind',[0,0x20,0x40])
def test_direct_broadcast_keeps_receipt_and_connection_provenance_without_inventing_wlm(recipient,kind):
    receiver=TelemetryReceiver()
    raw=frame(receiver=recipient,command_type=kind)
    host=state()
    for offset in range(0,len(raw),3):
        receiver.feed(raw[offset:offset+3],1.,direct_context=host)
    sample=receiver.snapshot(1.25)
    assert sample['accepted']==1 and sample['received_at']==1.
    assert sample['source']['wifi_tunnel'] is False
    assert sample['source']['direct_connection']=={
        'generation':list(host.generation),'provenance':'owned_connected_swudp_peer_session'}
    assert 'carrier' not in sample['source'] and 'forwarding' not in sample['source']
    assert sample['fields']['motors_on']['value'] is True
    assert not sample['flight_ready']
    assert receiver.snapshot(2.1)['received_at']==1.
    assert not receiver.snapshot(2.1)['fresh']


@pytest.mark.parametrize('changes,now',[
    ({'committed_at':None},1.), ({'beacon_route_verified':False},1.),
    ({'wifi_backend_ready':False},1.), ({},1.501),
    ({'evaluated_at':3.},3.),
])
def test_unready_connection_cannot_admit_direct_fc(changes,now):
    receiver=TelemetryReceiver()
    receiver.feed(frame(receiver=0xe),now,direct_context=state(**changes))
    assert receiver.accepted==0


def test_untyped_context_and_unbound_broadcast_are_rejected():
    receiver=TelemetryReceiver()
    with pytest.raises(ValueError,match='typed owning'):
        receiver.feed(frame(receiver=0xe),1.,direct_context={'generation':state().generation})
    receiver.feed(frame(receiver=0xe),1.)
    assert receiver.accepted==0


@pytest.mark.parametrize('recipient',[0xe,0xaa])
@pytest.mark.parametrize('kind',[1,0x80,0xc0])
def test_binding_does_not_admit_encrypted_frames_or_ack_payloads(recipient,kind):
    receiver=TelemetryReceiver()
    receiver.feed(frame(receiver=recipient,command_type=kind),1.,direct_context=state())
    assert receiver.accepted==0


@pytest.mark.parametrize('length',[37,84,86])
def test_binding_requires_measured_direct_payload_length(length):
    receiver=TelemetryReceiver()
    receiver.feed(frame(body().ljust(length,b'\0')[:length],receiver=0xe),1.,direct_context=state())
    assert receiver.accepted==0 and receiver.malformed==1


def test_direct_context_does_not_escape_into_bare_inner_compatibility():
    receiver=TelemetryReceiver()
    receiver.feed(carrier(frame(receiver=0xe),trailer=b''),1.,direct_context=state())
    assert receiver.accepted==0
    receiver.feed(frame(receiver=2),1.,direct_context=state())
    assert receiver.accepted==1
    assert 'direct_connection' not in receiver.snapshot(1.)['source']


@pytest.mark.parametrize('initial',['unbound','different-generation','lost-backend'])
def test_partial_frame_cannot_cross_binding_boundary(initial):
    receiver=TelemetryReceiver();raw=frame(receiver=0xe);host=state()
    first=None if initial=='unbound' else host
    second=host
    if initial=='different-generation':
        second=replace(host,generation=(*host.generation[:3],host.generation[3]+1))
    if initial=='lost-backend':
        second=replace(host,wifi_backend_ready=False)
    receiver.feed(raw[:30],1.,direct_context=first)
    receiver.feed(raw[30:],1.1,direct_context=second)
    assert receiver.accepted==0 and receiver.decoder.discarded_bytes>=30
    receiver.feed(raw,1.2,direct_context=host)
    assert receiver.accepted==1


def test_fragment_continuation_allows_new_evaluation_of_same_generation():
    receiver=TelemetryReceiver();host=state();raw=frame(receiver=0xe)
    receiver.feed(raw[:30],1.,direct_context=host)
    receiver.feed(raw[30:],1.1,direct_context=replace(host,evaluated_at=1.1))
    assert receiver.accepted==1 and receiver.snapshot(1.1)['received_at']==1.1


def test_real_observer_requires_current_generation_and_preserves_ground_and_freshness_guards():
    host=state();route=FlightRoute(*host.generation,host.beacon_received_at,True,host_control=host)
    observer=FlightObserver();fc=TelemetryReceiver();battery=BatteryReceiver(maximum_age=3)
    power=DumlPacket(3,2,1,0,3,0x55,bytes.fromhex('00000000401f140a5a01'))
    battery.feed(battery_carrier(power.encode(),source=host.generation[1],destination=host.generation[2]).encode(),1.)
    fc.feed(frame(body(flags=0x388000),receiver=0xe),1.,direct_context=host)
    observer.update(route,fc.snapshot(1.),battery.snapshot(1.),{},1.)
    assert observer.sample is not None and observer.sample.control_ready
    assert observer.sample.grounded is None
    observer.confirm_ground(observer.ground_confirmation_key(1.),1.)
    assert observer.sample.grounded is True
    fc.feed(frame(receiver=0xe),1.2,direct_context=host)
    observer.update(route,fc.snapshot(1.2),battery.snapshot(1.2),{},1.2)
    assert observer.sample.motors_on and observer.sample.grounded is False
    observer.update(route,fc.snapshot(1.701),battery.snapshot(1.701),{},1.701)
    assert observer.sample is None and not observer.permits_control(1.701)
    fresh=replace(host,evaluated_at=1.8,beacon_received_at=1.8,
                  generation=(*host.generation[:3],host.generation[3]+1))
    new_route=FlightRoute(*fresh.generation,1.8,True,host_control=fresh)
    observer.update(new_route,fc.snapshot(1.8),battery.snapshot(1.8),{},1.8)
    fc.feed(frame(receiver=0xe),1.9,direct_context=replace(host,evaluated_at=1.9))
    observer.update(new_route,fc.snapshot(1.9),battery.snapshot(1.9),{},1.9)
    assert observer.sample is None
