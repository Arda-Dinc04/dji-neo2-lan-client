"""Synthetic startup refresh on a committed, current owned preview session."""
from dataclasses import replace

import pytest

from neo2_control.camera import CameraChannel
from neo2_control.duml import DumlSequence
from neo2_control.session import SwudpSession, run_session
from neo2_control.transport import ReliableTransport
from neo2_control.wlm_modes import WlmModes
from test_session import acceptance, status
from test_liveview_session import packet, requests, wifi_beacon
from test_wlm_liveview import IDENTITY, OWN, PEER, capabilities_and_follow, peer_frame
from test_wlm_link import send
from test_wlm_query import beacon


def client(*, enabled=True, preview=True, needed=lambda: True, backend=lambda: True):
    options = {'startup_keyframe_refresh': enabled} if enabled is not None else {}
    value = SwudpSession(heartbeat=False, wlm_identity=IDENTITY, wlm_wifi_liveview=True,
        camera_channel=CameraChannel(), wifi_backend_ready=backend,
        video_keyframe_needed=needed, preview_only=preview,
        transport=ReliableTransport(session=0x1234, base_sequence=0x4000),
        duml_sequence=DumlSequence(current=0x8000), **options)
    value.start(0)
    value.receive(acceptance(flags=15), 0)
    return value


def commit(value):
    link = value.wlm_link
    link.on_beacon(beacon(), 0)
    capabilities_and_follow(link)
    send(link, 1.52, 'wlm_link_switch_trigger')
    link.feed(peer_frame(link, 8, PEER + OWN + bytes((5, 4, 4))), 1.53)
    send(link, 1.53, 'wlm_link_switch_prepare_ack')
    completion = send(link, 1.54, 'wlm_link_switch_completion')
    link.feed(peer_frame(link, 6, OWN + b'\0', sequence=completion.sequence, kind=0xc0), 1.55)
    link.feed(peer_frame(link, 6, PEER + OWN + bytes(5)), 1.56)
    send(link, 1.56, 'wlm_link_switch_result_ack')
    assert link.service_switch.committed


def refresh(value, now, *, ssfn=0, mode=4, mutate=None):
    item = wifi_beacon(ssfn=ssfn, mode=mode)
    if mutate:
        body = bytearray(item.payload)
        mutate(body)
        item = replace(item, payload=bytes(body))
    value.receive(status(payload=peer_frame(value.wlm_link, 0x13, item.payload, kind=0)), now)


def ready(**options):
    value = client(**options)
    commit(value)
    refresh(value, 1.6)
    return value


def test_explicit_startup_refresh_accepts_current_ssfn_zero_without_prior_video():
    value = ready()
    assert value.video_bytes_delivered == 0
    request, = requests(value, 1.61)
    assert packet(request).payload == bytes.fromhex('00080000000001000000')
    assert (packet(request).sender, packet(request).receiver, packet(request).cmd_set,
        packet(request).cmd_id, packet(request).cmd_type) == (2, 0xe9, 0x18, 0x47, 0)
    assert value.datagram_send_allowed(request, 1.62)


@pytest.mark.parametrize('enabled', [None, False])
def test_default_retains_first_video_byte_policy(enabled):
    value = ready(enabled=enabled)
    assert requests(value, 1.61) == []
    assert value.video_keyframes.reason == 'awaiting_video_bytes'


@pytest.mark.parametrize('options', [{'startup_keyframe_refresh': 1},
    {'startup_keyframe_refresh': True},
    {'startup_keyframe_refresh': True, 'preview_only': True}])
def test_invalid_startup_configuration_rejected_before_effects(options):
    with pytest.raises(ValueError):
        SwudpSession(**options)


def test_startup_is_not_allowed_for_nonpreview_session():
    with pytest.raises(ValueError):
        client(preview=False)


@pytest.mark.parametrize('options', [{'startup_keyframe_refresh': 1},
    {'startup_keyframe_refresh': True},
    {'startup_keyframe_refresh': True, 'preview_only': True}])
def test_run_session_rejects_invalid_startup_configuration_before_capture(tmp_path, options):
    path = tmp_path / 'capture.jsonl'
    with pytest.raises(ValueError):
        run_session(capture_path=path, **options)
    assert not path.exists()


def test_no_startup_refresh_before_service_commit_or_postcommit_beacon():
    value = client()
    value.wlm_link.on_beacon(beacon(), 0)
    refresh(value, .1)
    assert requests(value, .11) == []
    other = client()
    commit(other)
    assert requests(other, 1.57) == []


@pytest.mark.parametrize('invalid', ['stale', 'generation', 'rollback', 'need_false',
    'backend', 'revoked', 'peer_mode', 'route_source', 'route_destination', 'recommit', 'ssfn'])
def test_actual_send_rechecks_and_cancels_pending_refresh(invalid):
    needed = [True]
    backend = [True]
    value = ready(needed=lambda: needed[0], backend=lambda: backend[0])
    request, = requests(value, 1.61)
    link = value.wlm_link
    when = 1.7
    if invalid == 'stale':
        when = 6.61
    elif invalid == 'generation':
        refresh(value, when, mutate=lambda body: body.__setitem__(49, body[49] ^ 1))
    elif invalid == 'rollback':
        link.modes = WlmModes(1, 1, 1, 0, 0)
    elif invalid == 'need_false':
        needed[0] = False
    elif invalid == 'backend':
        backend[0] = False
    elif invalid == 'revoked':
        link.camera_channel.revoke()
    elif invalid == 'peer_mode':
        refresh(value, when, mode=1)
    elif invalid == 'route_source':
        link.peer_beacon = replace(link.peer_beacon, forwarding_source_hash=0)
    elif invalid == 'route_destination':
        link.peer_beacon = replace(link.peer_beacon, forwarding_destination_hash=0)
    elif invalid == 'recommit':
        link.service_switch.completed_at = 1.65
        refresh(value, when)
    elif invalid == 'ssfn':
        refresh(value, when, ssfn=1)
    assert not value.datagram_send_allowed(request, when)
    assert value.video_keyframes.pending is None and value.video_keyframes.sent_count == 0


def test_startup_preserves_three_request_budget_one_second_cadence_and_no_flight():
    value = ready()
    first, = requests(value, 1.61)
    value.datagram_sent(first, 1.61)
    refresh(value, 2.6)
    assert requests(value, 2.6) == []
    for when in (2.62, 3.63):
        refresh(value, when)
        request, = requests(value, when)
        value.datagram_sent(request, when)
    refresh(value, 4.64)
    assert requests(value, 4.64) == []
    assert value.video_keyframes.sent_count == 3
    assert value.flight_channel is None
