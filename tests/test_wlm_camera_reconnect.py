"""Bounded reconnect reads and a real bilateral service contract, no RF."""
from dataclasses import replace

import pytest

from neo2_control.camera import CameraChannel
from neo2_control.duml import DumlPacket, DumlSequence
from neo2_control.flight_channel import FlightChannel
from neo2_control.wlm_discovery import unpack_forwarded
from neo2_control.wlm_link import WlmWifiLink
from neo2_control.wlm_query import _peer_hash
from test_wlm_link import IDENTITY, follow, send
from test_wlm_query import beacon
from test_wlm_real_ack import carrier
from test_wlm_service_link import incoming_prepare


def warm_beacon():
    original=beacon()
    payload=bytearray(original.payload)
    payload[24:27]=bytes([5,4,4])
    payload[52:54]=bytes([2,1])
    return type(original)(bytes(payload))


def reconnect(*,camera=True,flight=False):
    preparations=[]
    link=WlmWifiLink(IDENTITY,DumlSequence(current=0x8000).next,
        maximum_age=5 if camera and not flight else 2,
        switch_wifi_service=True,prepare_wifi=lambda modes:preparations.append(modes) or True,
        camera_channel=CameraChannel(clock=lambda:link.last_time or 0) if camera else None,
        flight_channel=FlightChannel(clock=lambda:link.last_time or 0) if flight else None)
    for now in (0,.5,1):
        refresh(link,now)
        send(link,now,'wlm_link_presence')
    first=send(link,1.5,'wlm_link_capability_query')
    return link,first,preparations


def refresh(link,now):
    current=warm_beacon()
    if link.flight_channel is not None:
        current=replace(current,forwarding_source_hash=_peer_hash(current.device_id),
                        forwarding_destination_hash=IDENTITY.wlm_device_hash)
    link.on_beacon(current,now)


def tick(link,now):
    refresh(link,now)
    outgoing=link.next_packet(now)
    if outgoing is not None:
        link.sent(outgoing.kind,now)
    return outgoing


def test_camera_retry_is_delayed_bounded_and_counts_actual_sends_with_new_shared_counters():
    link,first,calls=reconnect()
    queries=[first]
    query_times=[1.5]
    for quarter in range(7,77):
        outgoing=tick(link,quarter/4)
        if outgoing is not None and outgoing.kind=='wlm_link_capability_query':
            queries.append(outgoing)
            query_times.append(quarter/4)
    state=link.snapshot(19)
    assert len(queries)==3
    assert state['capability_queries_sent']==3 and state['capability_query_attempt_limit']==3
    assert state['capability_query_retry_in_seconds'] is None
    assert state['capability_read']['state']=='timed_out'
    assert state['state']=='capability_read_failed'
    assert link.service_switch is None and not calls
    frames=[unpack_forwarded(q.packet.payload) for q in queries]
    assert [int.from_bytes(frame.attributes[5:9],'little') for frame in frames]==[1,2,3]
    assert [q.packet.seq for q in queries]==[1,2,3]
    assert len({frame.packet.seq for frame in frames})==3
    assert query_times==[1.5,7.5,13.5]
    # Each timeout is5s after the actual send, then1s before a fresh read.
    assert link.queries_sent==3 and link.forwarding_sequence==4
    assert not state['camera']['ready'] and not state['wifi_service_configured']


@pytest.mark.parametrize('camera',[False,True])
def test_actual_camera_response_latency_2064ms_is_accepted_only_by_camera_profile(camera):
    link,first,calls=reconnect(camera=camera)
    for now in (2,2.5,3,3.5):tick(link,now)
    received_at=1.5+2.064282
    link.feed(carrier(sequence=first.sequence),received_at)
    state=link.snapshot(received_at)
    assert state['capability_read']['timeout_seconds']==(5 if camera else 2)
    assert state['capability_read']['state']==('complete' if camera else 'timed_out')
    assert (state['capability_read']['capabilities'] is not None)==camera
    assert state['capability_queries_sent']==1
    assert not state['wifi_service_configured'] and not state['application_ready'] and not calls


@pytest.mark.parametrize('elapsed,accepted',[(4.999,True),(5,False),(5.001,False)])
def test_camera_response_deadline_is_strictly_before_five_seconds(elapsed,accepted):
    link,first,calls=reconnect()
    for quarter in range(8,25):tick(link,quarter/4)
    when=1.5+elapsed
    link.feed(carrier(sequence=first.sequence),when)
    state=link.snapshot(when)
    assert state['capability_read']['timeout_seconds']==5
    assert state['capability_read']['state']==('complete' if accepted else 'timed_out')
    assert (state['capability_read']['capabilities'] is not None)==accepted
    assert state['capability_queries_sent']==1 and not state['wifi_service_configured']
    assert not calls


def test_camera_response_budget_cannot_outlive_the_current_peer():
    link,first,calls=reconnect()
    # Last beacon was at1.0: peer lifetime ends at6.0, before read deadline6.5.
    assert link.next_packet(6) is None
    assert link.reason=='peer_beacon_expired' and link.read.state=='cancelled'
    link.feed(carrier(sequence=first.sequence),6.1)
    state=link.snapshot(6.1)
    assert state['capability_read']['state']=='cancelled'
    assert state['capability_read']['capabilities'] is None
    assert state['capability_query_retry_in_seconds'] is None
    assert state['capability_queries_sent']==1 and not calls


@pytest.mark.parametrize('camera',[False,True])
def test_capability_timeout_stays_within_configured_peer_age(camera):
    link=WlmWifiLink(IDENTITY,DumlSequence(current=0x8000).next,maximum_age=1.25,
        switch_wifi_service=True,prepare_wifi=lambda modes:True,
        camera_channel=CameraChannel() if camera else None)
    link.on_beacon(warm_beacon(),0)
    assert link.read.timeout==1.25


def test_pending_retry_has_no_deadline_and_old_ack_cannot_complete_new_correlation():
    link,first,calls=reconnect()
    first_sequence=first.sequence
    for quarter in range(8,27):tick(link,quarter/4)
    assert link.snapshot(6.5)['capability_query_retry_in_seconds']==1
    tick(link,7)
    refresh(link,7.5)
    pending=link.next_packet(7.5)
    assert pending.kind=='wlm_link_capability_query' and pending.sequence!=first_sequence
    assert link.snapshot(7.5)['capability_queries_sent']==1
    assert link.snapshot(7.5)['capability_read']['state']=='prepared'
    assert link.read.timeout==5
    link.feed(carrier(sequence=first_sequence),7.6)
    assert link.read.state=='prepared' and link.read.sent_at is None
    link.sent(pending.kind,7.7)
    assert link.snapshot(7.7)['capability_queries_sent']==2
    link.feed(carrier(sequence=first_sequence),7.8)
    assert link.read.state=='awaiting_response' and link.read.capabilities is None
    link.feed(carrier(sequence=pending.sequence),7.9)
    assert link.read.state=='complete'
    trigger=send(link,7.9,'wlm_link_switch_trigger')
    assert unpack_forwarded(trigger.packet.payload).packet.cmd_id==2
    assert link.snapshot(7.9)['follow_replies_sent']==0
    assert link.snapshot(7.9)['modes_raw']==[4,4,4,0,0]
    assert not calls and not link.snapshot(7.9)['camera']['ready']


def test_plain_observation_link_still_sends_only_one_query_after_timeout():
    link,first,calls=reconnect(camera=False)
    queries=[first]
    for quarter in range(7,49):
        outgoing=tick(link,quarter/4)
        if outgoing is not None and outgoing.kind=='wlm_link_capability_query':queries.append(outgoing)
    state=link.snapshot(12)
    assert len(queries)==1 and state['capability_queries_sent']==1
    assert state['capability_query_attempt_limit']==1 and state['capability_read']['state']=='timed_out'
    assert state['capability_query_retry_in_seconds'] is None
    assert link.service_switch is None and not calls


@pytest.mark.parametrize('camera',[False,True])
def test_flight_reconnect_keeps_three_bounded_reads_without_widening_peer_deadline(camera):
    link,first,calls=reconnect(camera=camera,flight=True)
    queries=[first];times=[1.5]
    for quarter in range(7,45):
        outgoing=tick(link,quarter/4)
        if outgoing is not None and outgoing.kind=='wlm_link_capability_query':
            queries.append(outgoing);times.append(quarter/4)
    state=link.snapshot(11)
    assert times==[1.5,4.5,7.5] and len(queries)==3
    assert state['capability_query_attempt_limit']==state['capability_queries_sent']==3
    assert state['capability_read']['timeout_seconds']==link.maximum_age==2
    assert state['capability_read']['state']=='timed_out'
    assert state['capability_query_retry_in_seconds'] is None
    assert [q.packet.seq for q in queries]==[1,2,3]
    assert len({q.sequence for q in queries})==3
    assert not calls and link.service_switch is None
    assert not link.flight_channel.snapshot()['route_ready']
    assert link.flight_channel.snapshot()['sent_count']==0


def test_second_flight_only_read_requires_new_ack_and_new_bilateral_commit():
    link,first,calls=reconnect(camera=False,flight=True)
    for quarter in range(7,19):tick(link,quarter/4)
    assert link.queries_sent==2 and link.read.sequence!=first.sequence
    link.feed(carrier(sequence=first.sequence),4.6)
    assert link.read.state=='awaiting_response'
    link.feed(carrier(sequence=link.read.sequence),4.7)
    assert link.read.state=='complete'
    assert not link.flight_channel.snapshot()['route_ready']
    send(link,4.7,'wlm_link_switch_trigger')
    assert link.service_switch is not None and not link.service_switch.committed
    assert not calls and not link.snapshot(4.7)['wifi_service_configured']


@pytest.mark.parametrize('condition',['expired','closed','reboot'])
def test_flight_only_retry_cannot_survive_loss_of_its_selected_generation(condition):
    link,first,calls=reconnect(camera=False,flight=True)
    for quarter in range(7,15):tick(link,quarter/4)
    assert link.query_retry_at==4.5
    if condition=='closed':link.close(3.6,'requested')
    elif condition=='reboot':
        payload=bytearray(link.peer_beacon.payload);payload[49]^=1
        link.on_beacon(replace(link.peer_beacon,payload=bytes(payload)),3.6)
    assert link.next_packet(6) is None
    assert link.queries_sent==1 and link.query_retry_at is None
    assert link.service_switch is None and not calls


@pytest.mark.parametrize('condition',['expired','closed','different_peer','profile_changed'])
def test_retry_requires_current_same_peer_and_open_link(condition):
    link,first,calls=reconnect()
    for quarter in range(8,27):tick(link,quarter/4)
    assert link.query_retry_at==7.5
    if condition=='closed':
        link.close(6.6,'requested')
    elif condition=='profile_changed':
        payload=bytearray(warm_beacon().payload);payload[51]=4
        link.on_beacon(type(beacon())(bytes(payload)),6.6)
    elif condition=='different_peer':
        payload=bytearray(warm_beacon().payload);payload[4]^=1
        # Foreign traffic cannot refresh the selected peer through its expiry.
        link.on_beacon(type(beacon())(bytes(payload)),10.4)
    assert link.next_packet(11.5) is None
    state=link.snapshot(11.5)
    assert state['capability_queries_sent']==1 and state['capability_query_retry_in_seconds'] is None
    assert link.service_switch is None and not calls and not state['camera']['ready']


def test_error_response_is_terminal_and_does_not_schedule_timeout_retries():
    link,_,calls=reconnect()
    outer=DumlPacket.decode(carrier(sequence=link.read.sequence))
    forwarded=unpack_forwarded(outer.payload)
    inner=replace(forwarded.packet,payload=b'\x01'+forwarded.packet.payload[1:])
    link.feed(replace(outer,payload=inner.encode()+forwarded.attributes).encode(),1.6)
    assert link.read.state=='response_error'
    for quarter in range(7,33):tick(link,quarter/4)
    state=link.snapshot(8)
    assert state['capability_queries_sent']==1 and state['capability_query_retry_in_seconds'] is None
    assert link.service_switch is None and not calls


def test_capabilities_without_follow_start_only_a_new_bilateral_transaction_then_commit():
    link,_,calls=reconnect()
    link.feed(carrier(sequence=link.read.sequence),1.6)
    trigger=send(link,1.6,'wlm_link_switch_trigger')
    assert trigger.sequence!=link.read.sequence
    assert link.snapshot(1.6)['follow_requests_applied']==0
    assert link.snapshot(1.6)['modes_raw']==[4,4,4,0,0]
    assert not link.snapshot(1.6)['wifi_service_configured'] and not calls
    link.feed(incoming_prepare(link),1.7)
    send(link,1.7,'wlm_link_switch_prepare_ack')
    own=send(link,1.8,'wlm_link_switch_completion')
    assert len(calls)==1 and not link.snapshot(1.8)['camera']['ready']
    outer=DumlPacket.decode(incoming_prepare(link))
    forwarded=unpack_forwarded(outer.payload)
    def frame(body,sequence,kind):
        inner=replace(forwarded.packet,cmd_id=6,payload=body,seq=sequence,cmd_type=kind)
        return replace(outer,payload=inner.encode()+forwarded.attributes).encode()
    link.feed(frame(IDENTITY.wlm_device_id+b'\0',own.sequence,0x80),1.9)
    assert not link.snapshot(1.9)['wifi_service_configured']
    link.feed(frame(link.peer_id+IDENTITY.wlm_device_id+bytes(5),0x4567,0x40),2)
    pending=link.next_packet(2)
    assert pending.kind=='wlm_link_switch_result_ack'
    assert not link.snapshot(2)['wifi_service_configured']
    link.sent(pending.kind,2)
    link.next_packet(2)
    state=link.snapshot(2)
    assert state['wifi_service_configured'] and state['camera']['ready']
    assert state['modes_raw']==[5,4,4,2,1] and len(calls)==2


def test_pending_follow_is_sent_before_starting_service_even_without_prior_follow_history():
    link,_,calls=reconnect()
    link.feed(carrier(sequence=link.read.sequence),1.6)
    link.feed(follow(link),1.6)
    assert link.service_switch is None and link.follow_reply is not None
    send(link,1.6,'wlm_link_follow_reply')
    assert link.service_switch is None and not calls
    send(link,1.7,'wlm_link_switch_trigger')
    assert link.service_switch is not None and not calls
