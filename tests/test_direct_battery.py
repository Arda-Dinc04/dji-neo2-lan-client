"""Measured direct FC03/55 battery broadcasts keep their socket provenance."""
from dataclasses import replace
import time
import threading

import pytest

from neo2_control.battery import BatteryReceiver
from neo2_control.control_transport import FlightChannelBackend
from neo2_control.duml import DumlPacket
from neo2_control.duml_stream import DumlStreamDecoder
from neo2_control.flight import FlightAction
from neo2_control.flight_channel import FlightChannel
from neo2_control.flight_observation import FlightObserver, FlightRoute
from neo2_control.telemetry import TelemetryReceiver
from neo2_control.wlm_discovery import unpack_forwarded
from test_battery import body, frame, tunnel
from test_control_transport_session import ControllerPeer
from test_flight_channel_session import start_flight_session
from test_host_control import state
from test_observation import until
from test_observation_envelope import carrier
from test_telemetry import body as fc_body, frame as fc_frame


@pytest.mark.parametrize('recipient',[2,0x0e])
def test_direct_battery_records_actual_receipt_and_current_connection_without_wlm(recipient):
    battery=BatteryReceiver(maximum_age=3);host=state();raw=frame(receiver=recipient,command_type=0)
    for offset in range(0,len(raw),3):
        battery.feed(raw[offset:offset+3],1.,direct_context=host)
    sample=battery.snapshot(1.2)['observations'][0]
    assert sample['received_at']==1. and sample['fields']['percent']['value']==72
    assert sample['source']['direct_connection']=={
        'generation':list(host.generation),'provenance':'owned_connected_swudp_peer_session'}
    assert sample['source']['wifi_tunnel'] is False
    assert 'carrier' not in sample['source'] and 'forwarding' not in sample['source']
    aged=battery.snapshot(4.01)['observations'][0]
    assert aged['received_at']==1. and not aged['fresh']


@pytest.mark.parametrize('changes,now',[
    ({'committed_at':None},1.),({'beacon_route_verified':False},1.),
    ({'wifi_backend_ready':False},1.),({},1.501),({'evaluated_at':3.},3.),
])
def test_unready_owner_keeps_direct_battery_diagnostic_without_control_provenance(changes,now):
    battery=BatteryReceiver()
    battery.feed(frame(command_type=0),now,direct_context=state(**changes))
    sample=battery.snapshot(now)['observations'][0]
    assert sample['fields']['percent']['value']==72
    assert 'direct_connection' not in sample['source']


@pytest.mark.parametrize('options',[
    {'sender':0x23},{'receiver':0xaa},{'receiver':0xe9},
    {'command_type':0x20},{'command_type':0x40},
    {'payload':body(length=11)},
])
def test_only_exact_measured_direct_battery_profile_gets_owner_binding(options):
    battery=BatteryReceiver();options={'command_type':0,**options}
    payload=options.pop('payload',None)
    battery.feed(frame(payload,**options),1.,direct_context=state())
    assert battery.accepted==1
    assert 'direct_connection' not in battery.snapshot(1.)['observations'][0]['source']


@pytest.mark.parametrize('kind',[1,0x80,0xc0])
def test_owner_binding_cannot_admit_encryption_or_battery_response_as_push(kind):
    battery=BatteryReceiver()
    battery.feed(frame(command_type=kind),1.,direct_context=state())
    assert battery.accepted==0


def test_untyped_binding_is_rejected_and_bare_inner_compatibility_cannot_inherit_binding():
    battery=BatteryReceiver();raw=frame(command_type=0)
    with pytest.raises(ValueError,match='typed owning'):
        battery.feed(raw,1.,direct_context={'generation':state().generation})
    battery.feed(tunnel(raw),1.,direct_context=state())
    sample=battery.snapshot(1.)['observations'][0]
    assert sample['source']['wifi_tunnel'] is True
    assert 'direct_connection' not in sample['source']


@pytest.mark.parametrize('boundary',['unbound','new-generation','lost-backend'])
def test_partial_battery_frame_cannot_acquire_or_cross_connection_binding(boundary):
    battery=BatteryReceiver();raw=frame(command_type=0);host=state()
    first=None if boundary=='unbound' else host
    second=host
    if boundary=='new-generation':second=replace(host,generation=(*host.generation[:3],host.generation[3]+1))
    if boundary=='lost-backend':second=replace(host,wifi_backend_ready=False)
    battery.feed(raw[:12],1.,direct_context=first)
    battery.feed(raw[12:],1.1,direct_context=second)
    assert battery.accepted==0 and battery.decoder.discarded_bytes>=12
    battery.feed(raw,1.2,direct_context=host)
    assert battery.snapshot(1.2)['observations'][0]['source']['direct_connection']['generation']==list(host.generation)


def test_partial_battery_can_finish_after_same_generation_owner_re_evaluation():
    battery=BatteryReceiver();raw=frame(command_type=0);host=state()
    battery.feed(raw[:12],1.,direct_context=host)
    battery.feed(raw[12:],1.1,direct_context=replace(host,evaluated_at=1.1))
    assert battery.accepted==1 and battery.snapshot(1.1)['observations'][0]['received_at']==1.1


def test_bound_direct_battery_and_fc_preserve_generation_and_independent_battery_expiry():
    host=state();route=FlightRoute(*host.generation,host.beacon_received_at,True,host_control=host)
    observer=FlightObserver();fc=TelemetryReceiver();battery=BatteryReceiver(maximum_age=3)
    fc.feed(fc_frame(fc_body(flags=0x388000),receiver=0xe),1.,direct_context=host)
    battery.feed(frame(command_type=0),1.,direct_context=host)
    observer.update(route,fc.snapshot(1.),battery.snapshot(1.),{},1.)
    observer.confirm_ground(observer.ground_confirmation_key(1.),1.)
    assert observer.permits_control(1.)
    assert observer.sources['battery']['direct_connection']['generation']==list(route.generation)
    later=replace(host,evaluated_at=4.01,beacon_received_at=4.01)
    later_route=FlightRoute(*later.generation,4.01,True,host_control=later)
    fc.feed(fc_frame(fc_body(flags=0x388000),receiver=0xe),4.01,direct_context=later)
    observer.update(later_route,fc.snapshot(4.01),battery.snapshot(4.01),{},4.01)
    partial=observer.read(4.01)
    assert partial is not None and partial.motion_received_at==4.01
    assert partial.battery_percent is None and partial.battery_received_at is None
    assert not observer.permits_control(4.01)
    assert observer.snapshot(4.01)['component_age_seconds']['battery'] is None
    assert not observer.snapshot(4.01)['sample_fresh']


def test_direct_battery_from_previous_generation_cannot_fill_current_fc_sample():
    host=state();old=replace(host,generation=(*host.generation[:3],host.generation[3]+1))
    route=FlightRoute(*host.generation,host.beacon_received_at,True,host_control=host)
    battery=BatteryReceiver(maximum_age=3);fc=TelemetryReceiver();observer=FlightObserver()
    battery.feed(frame(command_type=0),1.,direct_context=old)
    fc.feed(fc_frame(receiver=0xe),1.,direct_context=host)
    observer.update(route,fc.snapshot(1.),battery.snapshot(1.),{},1.)
    assert observer.sample is not None and observer.sample.motors_on is True
    assert observer.sample.battery_percent is None and observer.sample.battery_received_at is None
    assert not observer.permits_control(1.)
    assert 'battery' not in observer.sources


class DirectAirborneBatteryPeer(ControllerPeer):
    """Switch both FC and power to the measured direct source after motor start."""
    def __init__(self,channel):
        super().__init__(channel,direct_motion=True,observed_action_replies=True)

    def beacon_bytes(self):
        raw=super().beacon_bytes()
        if self.phase=='ground':return raw
        packets=[]
        for outer,wire in DumlStreamDecoder().feed(raw):
            if (outer.cmd_set,outer.cmd_id)==(0x51,1):
                forwarded=unpack_forwarded(outer.payload);inner=forwarded.packet
                if (inner.cmd_set,inner.cmd_id)==(3,0x55):
                    packets.append(DumlPacket(3,2,inner.seq,0,3,0x55,inner.payload).encode())
                    continue
            packets.append(wire)
        return b''.join(packets)


def test_actual_udp_owner_retains_direct_fc_and_battery_beyond_wrapped_battery_deadline(tmp_path):
    channel=FlightChannel();backend=FlightChannelBackend(channel,synthetic=True)
    stop=threading.Event()
    with DirectAirborneBatteryPeer(channel) as peer:
        worker,result,errors=start_flight_session(tmp_path,channel,peer,stop)
        try:
            until(lambda:backend.observation() is not None and backend.observation().control_ready,timeout=5)
            now=time.monotonic();backend.confirm_ground(backend.ground_confirmation_key(now),now)
            channel.submit_action(FlightAction.TAKEOFF,expires_at=time.monotonic()+.2)
            until(lambda:backend.observation() is not None and backend.observation().flying is True)
            started=time.monotonic()
            until(lambda:time.monotonic()-started>FlightObserver.BATTERY_MAXIMUM_AGE+.2,timeout=4)
            now=time.monotonic();sample=backend.observation();snapshot=channel.observer.snapshot(now)
            assert sample is not None and sample.flying and sample.motors_on
            assert sample.battery_percent==90 and now-sample.battery_received_at<.5
            assert sample.motion_received_at>started and sample.battery_received_at>started
            assert channel.observer.permits_control(now)
            generation=list(channel.observer.route.generation)
            for name in ['flight_controller','battery']:
                source=snapshot['sources'][name]
                assert source['wifi_tunnel'] is False
                assert source['direct_connection']['generation']==generation
                assert 'forwarding' not in source
            channel.submit_landing(expires_at=time.monotonic()+.2)
            until(lambda:backend.observation() is not None and backend.observation().motors_on is False)
            assert [p.payload for p in peer.flight_frames if (p.cmd_set,p.cmd_id)==(3,0x2a)]==[b'\1',b'\2']
        finally:
            stop.set();worker.join(timeout=3)
        assert not worker.is_alive() and not errors,errors
    assert result[0]['reason']=='requested'


def test_context_does_not_relabel_existing_wrapped_battery_provenance():
    host=state();battery=BatteryReceiver(maximum_age=3)
    raw=frame(command_type=0)
    battery.feed(carrier(raw,source=host.generation[1],destination=host.generation[2]).encode(),
                 1.,direct_context=host)
    source=battery.snapshot(1.)['observations'][0]['source']
    assert source['wifi_tunnel'] is True
    assert source['forwarding']=={'source_hash':host.generation[1],
        'destination_hash':host.generation[2],'link_category':2}
    assert 'direct_connection' not in source


def test_recorded_airborne_battery_frame_keeps_native_values_and_requires_owner_for_flight_binding():
    # Recovery swudp.jsonl SHA256 8e08721808583f742968e64f52a34215d9e6bf4b69a3d6b71f6e663048cd82f5,
    # row29, actual PC receipt487990.638418871; no drone access in this replay.
    raw=bytes.fromhex('551704380302385000035500000000201a0f8a22005f9a')
    battery=BatteryReceiver(maximum_age=3)
    battery.feed(raw,1.)
    first=battery.snapshot(1.)['observations'][0]
    assert first['fields']['percent']['value']==34
    assert first['fields']['voltage']['value']==6688
    assert first['fields']['serious_low_warning_percent']['value']==10
    assert 'direct_connection' not in first['source']
    battery.feed(raw,1.1,direct_context=state(evaluated_at=1.1))
    bound=battery.snapshot(1.1)['observations'][-1]
    assert bound['raw_payload_hex']=='00000000201a0f8a2200'
    assert bound['received_at']==1.1
    assert bound['source']['direct_connection']['generation']==list(state().generation)
    assert first['received_at']==1.
