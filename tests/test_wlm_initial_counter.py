"""Explicit per-session WLM counter experiments, no persistence or RF."""
from dataclasses import replace
import json
from pathlib import Path
import threading

import pytest

from neo2_control import session as session_module
from neo2_control.camera_console import CameraConfig
from neo2_control.duml import DumlPacket,DumlSequence
from neo2_control.duml_stream import DumlStreamDecoder
from neo2_control.session import SwudpSession,run_session
from neo2_control.transport import describe_transport
from neo2_control.wlm_discovery import unpack_forwarded
from neo2_control.wlm_link import WlmWifiLink
from test_camera_console import harness
from test_liveview_session import IDENTITY,KeyframePeer
from test_wlm_link import follow,send
from test_wlm_query import beacon


BAD=[True,False,-1,2**32,1.0,'32',None]


def link(**options):
    value=WlmWifiLink(IDENTITY,DumlSequence(current=0x8000).next,**options)
    value.nonce=0x1234
    value.on_beacon(beacon(),0)
    return value


def assert_counter(transmission,expected):
    forwarded=unpack_forwarded(transmission.packet.payload)
    assert transmission.packet.seq==expected&0xffff
    assert int.from_bytes(forwarded.attributes[5:9],'little')==expected
    return forwarded.packet


@pytest.mark.parametrize('initial',[0,32,0xffffffff])
def test_presence_query_and_follow_share_explicit_uint32_counter_and_actual_send_advance(initial):
    value=link(initial_forwarding_sequence=initial)
    presence=value.next_packet(0)
    assert presence.kind=='wlm_link_presence'
    assert assert_counter(presence,initial).cmd_id==0x13
    assert value.forwarding_sequence==initial
    value.sent(presence.kind,0)
    assert value.forwarding_sequence==initial
    for when in (.5,1):
        value.on_beacon(beacon(),when)
        assert_counter(send(value,when,'wlm_link_presence'),initial)
    query=value.next_packet(1.5)
    assert query.kind=='wlm_link_capability_query'
    assert assert_counter(query,initial).cmd_id==0x41
    assert value.forwarding_sequence==initial
    value.sent(query.kind,1.5)
    after=(initial+1)&0xffffffff
    assert value.forwarding_sequence==after
    value.feed(follow(value,sequence=0x9123),1.6)
    reply=value.next_packet(1.6)
    assert reply.kind=='wlm_link_follow_reply'
    inner=assert_counter(reply,after)
    assert inner.cmd_id==0x1c and inner.seq==0x9123
    value.sent(reply.kind,1.6)
    snapshot=value.snapshot(1.6)
    assert snapshot['forwarding_sequence_initial']==initial
    assert snapshot['forwarding_sequence_next']==(initial+2)&0xffffffff
    assert not snapshot['application_ready'] and not snapshot['negotiation_enabled']


def test_implicit_and_explicit_default_one_preserve_identical_wire_bytes():
    implicit=link();explicit=link(initial_forwarding_sequence=1)
    assert send(implicit,0,'wlm_link_presence').packet.encode()==send(explicit,0,'wlm_link_presence').packet.encode()
    assert implicit.forwarding_sequence==explicit.forwarding_sequence==1
    assert SwudpSession().wlm_link is None


@pytest.mark.parametrize('value',BAD)
def test_exact_uint32_validation_at_link_session_and_camera_configuration(value,tmp_path,monkeypatch):
    with pytest.raises(ValueError,match='uint32'):
        WlmWifiLink(IDENTITY,DumlSequence().next,initial_forwarding_sequence=value)
    with pytest.raises(ValueError,match='uint32'):
        SwudpSession(wlm_identity=IDENTITY,wlm_mode_follow=True,wlm_forwarding_sequence=value)
    with pytest.raises(ValueError,match='uint32'):
        CameraConfig('fake0',tmp_path/'wlm.json',tmp_path/'ble.json',wlm_forwarding_sequence=value)
    monkeypatch.setattr(session_module.socket,'socket',lambda *a,**k:pytest.fail('socket opened before validation'))
    with pytest.raises(ValueError,match='uint32'):
        run_session(capture_path=tmp_path/'capture.jsonl',wlm_identity_file=tmp_path/'wlm.json',
                    wlm_mode_follow=True,wlm_forwarding_sequence=value)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('read_only_query',[False,True])
def test_nondefault_sequence_requires_owned_persistent_mode_before_files_or_socket(tmp_path,monkeypatch,read_only_query):
    with pytest.raises(ValueError,match='persistent WLM'):
        SwudpSession(wlm_identity=IDENTITY,wlm_capability_query=read_only_query,wlm_forwarding_sequence=32)
    monkeypatch.setattr(session_module.socket,'socket',lambda *a,**k:pytest.fail('socket opened for invalid profile'))
    with pytest.raises(ValueError,match='persistent WLM'):
        run_session(capture_path=tmp_path/'capture.jsonl',wlm_identity_file=tmp_path/'owned.json',
                    wlm_capability_query=read_only_query,wlm_forwarding_sequence=32)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('initial',[1,32])
def test_camera_config_passes_only_explicit_nondefault_profile_to_session_owner(harness,initial):
    original=harness.config
    harness.config=replace(original,wlm_forwarding_sequence=initial)
    harness.console.config=harness.config
    call=harness.connect()
    try:
        if initial==1:assert 'wlm_forwarding_sequence' not in call.kwargs
        else:assert call.kwargs['wlm_forwarding_sequence']==32
        assert call.kwargs['wlm_identity_file']==original.wlm_identity_file
        assert harness.ble_calls[0]['wlm_identity_file']==original.wlm_identity_file
        assert harness.ble_calls[0]['identity_file']==original.ble_identity_file
        assert 'wlm_forwarding_sequence' not in harness.ble_calls[0]
    finally:harness.stop()
    assert harness.console.status()['cleanup_verified']


def test_run_session_threads_counter32_to_actual_udp_presence_and_query(tmp_path):
    identity=tmp_path/'owned.json'
    identity.write_text(json.dumps({'format':'neo2-control/wlm-identity/v1',
                                    'identification_id':IDENTITY.identification_id}))
    original_identity=identity.read_bytes();stop=threading.Event()
    def observe(state):
        if state['wlm_mode_follow'].get('capability_queries_sent')==1:stop.set()
    capture=tmp_path/'session.jsonl'
    with KeyframePeer(b'') as peer:
        result=run_session(capture_path=capture,host='127.0.0.1',port=peer.socket.getsockname()[1],
            duration=4,wlm_identity_file=identity,wlm_mode_follow=True,wlm_forwarding_sequence=32,
            stop_event=stop,snapshot_sink=observe)
    assert result.reason=='requested' and result.wlm_mode_follow['forwarding_sequence_initial']==32
    assert result.wlm_mode_follow['forwarding_sequence_next']==33
    assert identity.read_bytes()==original_identity
    frames=[]
    for row in map(json.loads,capture.read_text().splitlines()):
        if row['direction']!='tx':continue
        raw=bytes.fromhex(row['raw_hex']);decoded,_=describe_transport(raw)
        offset=decoded.get('qos_aggregate_offset');length=decoded.get('qos_aggregate_length',0)
        for outer,_ in DumlStreamDecoder().feed(b'' if offset is None else raw[offset:offset+length]):
            if (outer.cmd_set,outer.cmd_id)!=(0x51,1):continue
            forwarded=unpack_forwarded(outer.payload)
            frames.append((outer,forwarded))
    assert sum(f.packet.cmd_id==0x13 for _,f in frames)>=3
    assert sum(f.packet.cmd_id==0x41 for _,f in frames)==1
    expected_counter=32
    for outer,forwarded in frames:
        assert outer.seq==expected_counter
        assert int.from_bytes(forwarded.attributes[5:9],'little')==expected_counter
        if forwarded.packet.cmd_id!=0x13:expected_counter+=1
    assert expected_counter==33
