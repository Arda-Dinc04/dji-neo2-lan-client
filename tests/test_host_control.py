from dataclasses import replace
import json
from pathlib import Path

import pytest

from neo2_control.flight_observation import FlightObserver, FlightRoute
from neo2_control.flight import FlightAction
from neo2_control.host_control import HostControlState, canonical_neo2_id
from neo2_control.session import SwudpSession, OutboundDatagram
from neo2_control.wlm_modes import WlmModes
from neo2_control.wlm_switch import WIFI_SERVICE_MODES
from test_flight_observation import World
from test_flight_wlm_preflight import committed_link


GENERATION = ('a'*64, 0x12345678, 0x23456789, 123)
FIXTURE = json.loads((Path(__file__).parent/'fixtures/host-control-native.json').read_text())


def state(**changes):
    initial = HostControlState(GENERATION, 1., .9, .8,
        WIFI_SERVICE_MODES, WIFI_SERVICE_MODES, True, True)
    return replace(initial, **changes)


def test_native_service_classification_vectors_preserve_local_and_peer_distinction():
    for case in FIXTURE['cases']:
        local = replace(WIFI_SERVICE_MODES, liveview_service=case['local_services'][0],
                        download_service=case['local_services'][1])
        peer = replace(WIFI_SERVICE_MODES, liveview_service=case['peer_services'][0],
                       download_service=case['peer_services'][1])
        host = state(local_modes=local, peer_modes=peer, wifi_backend_ready=case['connected'])
        assert host.mode(1.) == case['expected_sdk_mode']
        if local != WIFI_SERVICE_MODES or not case['connected'] or case['peer_services'][0] != 2:
            assert not host.permits_control(1.)


@pytest.mark.parametrize('changes,reason', [
    ({'committed_at':None}, 'local_service_uncommitted'),
    ({'beacon_route_verified':False}, 'beacon_route_unverified'),
    ({'wifi_backend_ready':False}, 'wifi_backend_unavailable'),
    ({'peer_modes':WlmModes(1,1,1,2,1)}, 'command_route_not_wifi'),
    ({'peer_modes':WlmModes(5,4,4,0,1)}, 'peer_operation_mode_unavailable'),
    ({'local_modes':WlmModes(5,4,4,0,0)}, 'local_modes_not_committed_target'),
])
def test_each_owned_requirement_is_independent(changes, reason):
    host = state(**changes)
    assert not host.permits_control(1.)
    assert reason in host.snapshot(1.)['problems']
    assert host.snapshot(1.)['source'] == 'pc_owned_wlm_state'


def test_peer_mode_two_without_a_local_commit_is_not_ready():
    host = state(committed_at=None)
    assert host.mode(1.) == 2 and not host.permits_control(1.)


def test_owner_evaluation_cannot_renew_an_old_beacon_or_backdate_a_commit():
    original = state()
    assert original.permits_control(1.5)
    assert not original.permits_control(1.5001)
    refreshed = replace(original, evaluated_at=2.89)
    assert refreshed.beacon_received_at == .9 and refreshed.permits_control(2.89)
    assert not refreshed.permits_control(2.9)
    with pytest.raises(ValueError):
        state(committed_at=1.01)


@pytest.mark.parametrize('bad', [True, None, float('nan'), float('inf')])
def test_invalid_now_cannot_enable_or_publish_a_host_state(bad):
    host = state()
    assert not host.permits_control(bad) and host.mode(bad) == 65535
    with pytest.raises(ValueError):
        host.snapshot(bad)


@pytest.mark.parametrize('changes', [
    {'generation':('a'*64,1,2,0)}, {'generation':('x'*64,1,2,3)},
    {'evaluated_at':True}, {'beacon_received_at':1.1}, {'committed_at':float('nan')},
    {'wifi_backend_ready':1}, {'beacon_route_verified':None}, {'peer_modes':None},
])
def test_state_construction_requires_typed_clocked_evidence(changes):
    with pytest.raises(ValueError):
        state(**changes)


def test_route_cannot_adopt_other_generation_or_other_receipt_host_state():
    host=state()
    route=FlightRoute(*GENERATION,.9,True,host_control=host)
    with pytest.raises(ValueError):replace(route,nonce=124)
    with pytest.raises(ValueError):replace(route,seen_at=1.)


def test_real_observer_can_use_explicit_owner_state_without_any_radio_mode():
    world=World();world.push('fc');world.push('battery')
    host=state(generation=world.route.generation,beacon_received_at=world.now)
    route=replace(world.route,host_control=host)
    observer=FlightObserver(synthetic=False)
    observer.update(route,world.fc.snapshot(world.now),world.battery.snapshot(world.now),{},world.now)
    assert observer.sample.control_ready and observer.sample.grounded is None
    assert observer.origin is None and not observer.permits_control(world.now)
    observer.confirm_ground(observer.ground_confirmation_key(world.now),world.now)
    assert observer.permits_control(world.now)
    snapshot=observer.snapshot(world.now)
    assert snapshot['control_state_provenance']=='pc_owned_wlm_state'
    assert 'wifi_control' not in snapshot['sources']
    assert snapshot['sources']['host_control']['beacon_received_at']==world.now
    assert observer.read(world.now+.5001).control_ready is False
    assert 'host_control_owner_evaluation_stale' in observer.snapshot(world.now+.5001)['problems']
    assert observer.snapshot(world.now+.5001)['sources']['host_control']['control_ready'] is False


def test_radio_mode_and_new_sensor_frames_cannot_override_owner_service_loss():
    world=World();world.feed(mode=2,guard=3)
    host=state(generation=world.route.generation,beacon_received_at=world.now,committed_at=None)
    observer=FlightObserver(synthetic=False)
    observer.update(replace(world.route,host_control=host),world.fc.snapshot(world.now),
                    world.battery.snapshot(world.now),world.wifi.snapshot(world.now),world.now)
    assert observer.sample.control_ready is False
    assert 'host_control_local_service_uncommitted' in observer.problems


def test_link_projects_only_actual_peer_service_and_real_owner_commit():
    link=committed_link()
    route=link.flight_route(2.01)
    assert route.host_control.permits_control(2.01)
    assert route.host_control.generation == route.generation
    assert route.host_control.beacon_received_at == 2
    assert route.host_control.evaluated_at == 2.01
    assert route.host_control.committed_at == link.service_switch.completed_at
    link.prepare_wifi=lambda modes:False
    after=link.flight_route(2.02)
    assert not after.host_control.permits_control(2.02)
    assert after.seen_at==route.seen_at and after.host_control.beacon_received_at==2


def test_service_commit_from_another_generation_cannot_enable_control():
    link=committed_link()
    link.service_generation=(*link.service_generation[:3],link.service_generation[3]^1)
    host=link.flight_route(2.01).host_control
    assert host.mode(2.01)==2 and host.committed_at is None
    assert not host.permits_control(2.01)


def test_closed_owner_cannot_publish_its_previous_control_state():
    link=committed_link()
    assert link.flight_route(2.01).host_control.permits_control(2.01)
    link.close(2.02)
    assert link.flight_route(2.02) is None
    assert link.flight_channel.snapshot()['revoked']


def test_sparse_discovery_suspends_authority_without_destroying_owned_transport():
    link=committed_link();beacon=link.peer_beacon;committed=link.service_switch.completed_at
    stale=link.flight_route(4)
    assert stale.host_control.beacon_received_at==2
    assert not stale.host_control.permits_control(4)
    assert link.peer_stale and not link.flight_channel.snapshot()['revoked']
    assert not link.flight_channel.snapshot()['ordinary_control_ready']
    # Announcing continues throughout the observed6-9s discovery pauses.
    for now in (4,5,6,7,8):
        outgoing=link.next_packet(now)
        assert outgoing.kind=='wlm_link_presence'
        link.sent(outgoing.kind,now)
    assert link.service_switch.completed_at==committed
    link.on_beacon(beacon,8.1)
    recovered=link.flight_route(8.1)
    assert not link.peer_stale and link.reason is None
    assert recovered.generation==stale.generation
    assert recovered.host_control.beacon_received_at==8.1
    assert recovered.host_control.committed_at==committed
    assert recovered.host_control.permits_control(8.1)


@pytest.mark.parametrize('change',['source','destination','boot'])
def test_discovery_recovery_requires_the_same_bound_generation(change):
    link=committed_link();beacon=link.peer_beacon
    link.flight_route(4)
    if change=='source':beacon=replace(beacon,forwarding_source_hash=beacon.forwarding_source_hash^1)
    elif change=='destination':beacon=replace(beacon,forwarding_destination_hash=link.identity.wlm_device_hash^1)
    else:
        body=bytearray(beacon.payload);body[49]^=1;beacon=replace(beacon,payload=bytes(body))
    link.on_beacon(beacon,4.1)
    assert not link.flight_channel.snapshot()['ordinary_control_ready']
    if change=='boot':assert link.flight_channel.snapshot()['revoked'] and link.reason=='peer_session_changed'
    else:assert link.peer_stale and link.peer_seen_at==2


@pytest.mark.parametrize('change', ['source','destination','canonical-id'])
def test_unbound_beacon_cannot_become_an_owned_control_source(change):
    link=committed_link();old=link.peer_beacon
    if change=='source':link.peer_beacon=replace(old,forwarding_source_hash=old.forwarding_source_hash^1)
    elif change=='destination':link.peer_beacon=replace(old,forwarding_destination_hash=link.identity.wlm_device_hash^1)
    else:
        body=bytearray(old.payload);body[4:24]=b'id\0bad-padding'.ljust(20,b'\0')
        link.peer_beacon=replace(old,payload=bytes(body))
    assert not link.flight_route(2.01).host_control.permits_control(2.01)


def test_host_identity_rule_preserves_full_raw_id_including_nul_padding():
    assert canonical_neo2_id(bytes((0,0,36))+b'id'.ljust(20,b'\0'))
    assert canonical_neo2_id(bytes((0,0,36))+b'x'*20)
    assert not canonical_neo2_id(bytes((0,0,36))+b'id\0tail'.ljust(20,b'\0'))
    assert not canonical_neo2_id(bytes((0,1,36))+b'id'.ljust(20,b'\0'))
    assert not canonical_neo2_id(bytes((0,0,36))+bytes(20))


@pytest.mark.parametrize('change', ['backend','peer-service','beacon-source'])
@pytest.mark.parametrize('landing', [False,True])
def test_socket_preflight_refreshes_owner_state_and_preserves_grouped_landing(change,landing):
    link=committed_link();world=World();world.now=2.01
    world.route=link.flight_route(world.now)
    world.push('fc');world.push('battery')
    ready=[True]
    client=SwudpSession(wlm_identity=link.identity,wlm_wifi_liveview=True,
        wifi_backend_ready=lambda:ready[0],flight_channel=world.channel)
    client.state='open';client.feature_flags=15
    client.telemetry=world.fc;client.battery=world.battery;client.wifi_control=world.wifi
    client.wlm_link=link;link.flight_channel=world.channel
    link.prepare_wifi=client._prepare_wifi_service
    client._project_flight(world.now);world.attest()
    world.channel.set_ready(True,world.now)
    accepted=(world.channel.submit_landing(expires_at=2.2) if landing else
              world.channel.submit_action(FlightAction.TAKEOFF,expires_at=2.2))
    outgoing=link.next_packet(world.now)
    before=link.forwarding_sequence
    datagram=OutboundDatagram(b'', 'fixture; no socket', 'wlm_link_flight',outgoing.sequence)
    world.now=2.02
    if change=='backend':ready[0]=False
    elif change=='peer-service':
        body=bytearray(link.peer_beacon.payload);body[52]=0
        link.on_beacon(replace(link.peer_beacon,payload=bytes(body)),world.now)
    else:
        link.peer_beacon=replace(link.peer_beacon,forwarding_source_hash=link.read.peer_hash^1)
    allowed=client.datagram_send_allowed(datagram,world.now)
    assert allowed is (landing and change!='backend')
    assert not world.backend.observation().control_ready
    assert link.forwarding_sequence==before and world.channel.snapshot()['sent_count']==0
    if not allowed:
        assert world.channel.receipt(accepted['intent_id'])['state']=='cancelled'


def test_unrelated_beacon_cannot_renew_the_selected_peer_lifetime_or_services():
    link=committed_link();before=link.peer_beacon
    body=bytearray(before.payload);body[52]=0
    link.on_beacon(replace(before,payload=bytes(body),forwarding_source_hash=link.read.peer_hash^1),2.1)
    assert link.peer_beacon==before and link.peer_seen_at==2
    assert link.flight_route(2.1).host_control.permits_control(2.1)
    stale=link.flight_route(4.)
    assert stale.seen_at==2 and not stale.host_control.permits_control(4.)
    assert link.peer_stale and not link.flight_channel.snapshot()['ordinary_control_ready']
