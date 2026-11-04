"""Real localhost UDP flight forwarding, with a synthetic peer and no RF."""
from dataclasses import replace
import json
from pathlib import Path
import threading
import time

import pytest

from neo2_control.camera import CameraChannel
from neo2_control.capture import CaptureRecorder,load_capture
from neo2_control.duml import DumlPacket
from neo2_control.duml_stream import DumlStreamDecoder
from neo2_control.flight import FlightAction
from neo2_control.flight_channel import FlightChannel
from neo2_control.session import SwudpSession, run_session
from neo2_control.wlm_discovery import unpack_forwarded
from neo2_control.transport import describe_transport
from test_camera_session import CameraPeer
from test_observation import until
from test_wlm_liveview import BEACON, IDENTITY


class FlightPeer(CameraPeer):
    """Independent synthetic service handshake and flight ACKs on loopback."""
    def __init__(self, channel, *, hold_action_ack=False, delay_completion_once=False,
                 observed_action_replies=False):
        super().__init__(channel)
        self.flight_frames = []
        self.hold_action_ack = hold_action_ack
        self.observed_action_replies=observed_action_replies
        self.boot_nonce = 0x2541
        self.delay_completion_once=delay_completion_once
        self.completion_requests=0
        self.requests=[]

    def observe_packet(self,inner):
        if inner.cmd_type in (0,0x40) and (inner.cmd_set,inner.cmd_id)!=(0x51,0x13):
            self.requests.append(inner)

    def completion_reply(self,inner):
        self.completion_requests+=1
        if self.delay_completion_once and self.completion_requests==1:return b''
        return super().completion_reply(inner)

    def beacon_bytes(self):
        outer = DumlPacket.decode(BEACON)
        forwarded = unpack_forwarded(outer.payload)
        body = bytearray(forwarded.packet.payload)
        body[49:51] = self.boot_nonce.to_bytes(2, 'little')
        if self.completed:
            body[24:27] = bytes([5, 4, 4])
            body[52:54] = bytes([2, 1])
        inner = replace(forwarded.packet, payload=bytes(body))
        attributes = bytearray(forwarded.attributes)
        attributes[:4] = self.oracle.read.peer_hash.to_bytes(4, 'little')
        attributes[9:13] = IDENTITY.wlm_device_hash.to_bytes(4, 'little')
        return replace(outer, payload=inner.encode()+bytes(attributes)).encode()

    def additional_packet(self, inner, attributes):
        assert self.completed
        assert attributes[9:13] == self.oracle.read.peer_hash.to_bytes(4, 'little')
        self.flight_frames.append(inner)
        self.record_allocation(inner)
        if (inner.sender, inner.receiver, inner.cmd_type, inner.cmd_set, inner.cmd_id) == (2, 3, 0x40, 3, 0x2a):
            assert inner.payload in (b'\x01', b'\x02')
            if self.observed_action_replies:
                if inner.payload==b'\2':
                    return DumlPacket(inner.receiver,inner.sender,inner.seq,0x80,3,0x2a,b'\0').encode()
                outer=DumlPacket.decode(self.camera_frame(inner,kind=0x80))
                forwarded=unpack_forwarded(outer.payload);attrs=bytearray(forwarded.attributes)
                attrs[9:13]=b'\xff'*4
                return replace(outer,payload=forwarded.packet.encode()+bytes(attrs)).encode()
            valid=self.camera_frame(inner, kind=0xc0)
            if not self.hold_action_ack:return valid
            self.pending_ack=valid
            outer=DumlPacket.decode(valid)
            forwarded=unpack_forwarded(outer.payload)
            invalid=[]
            for offset in (0,9):
                attributes=bytearray(forwarded.attributes)
                attributes[offset]^=1
                invalid.append(replace(outer,payload=forwarded.packet.encode()+bytes(attributes)).encode())
            attributes=bytearray(forwarded.attributes)
            attributes[9:13]=bytes.fromhex('ffffffff')
            attributes[0]^=1
            invalid.append(replace(outer,payload=forwarded.packet.encode()+bytes(attributes)).encode())
            invalid.append(self.camera_frame(inner,kind=0xc1))
            invalid.append(self.camera_frame(inner,kind=0xc0,seq=inner.seq^1))
            return b''.join(invalid)
        assert (inner.sender, inner.receiver, inner.cmd_type, inner.cmd_set, inner.cmd_id) == (2, 0xa9, 0, 1, 0xa)
        fixture = json.loads((Path(__file__).parent/'fixtures/virtual-joystick-native.json').read_text())
        neutral = next(row for row in fixture['virtual_vectors'] if row['channels'] == [0, 0, 0, 0])
        expected = DumlPacket.decode(bytes.fromhex(neutral['native_frame_hex']))
        assert inner.payload == expected.payload
        return b''


def start_flight_session(tmp_path, channel, peer, stop):
    identity = tmp_path/'identity.json'
    identity.write_text(json.dumps({'format': 'neo2-control/wlm-identity/v1',
                                    'identification_id': IDENTITY.identification_id}))
    result, errors = [], []
    def run():
        try:
            result.append(run_session(capture_path=tmp_path/'flight.jsonl', host='127.0.0.1',
                port=peer.sock.getsockname()[1], duration=8, heartbeat=True,
                wlm_identity_file=identity, wlm_wifi_liveview=True,
                flight_channel=channel, stop_event=stop).as_dict())
        except BaseException as error:
            errors.append(repr(error))
    worker = threading.Thread(target=run)
    worker.start()
    return worker, result, errors


def completed(channel, intent_id, state):
    return any(row['id'] == intent_id and row['state'] == state
               for row in channel.snapshot()['history'])


def test_real_udp_takeoff_control_and_land_share_owner_sequences_counters_and_exact_routes(tmp_path):
    channel = FlightChannel()
    stop = threading.Event()
    with FlightPeer(channel,delay_completion_once=True) as peer:
        worker, result, errors = start_flight_session(tmp_path, channel, peer, stop)
        try:
            until(lambda: channel.snapshot()['route_ready'], timeout=5)
            assert peer.completed
            for action in (FlightAction.TAKEOFF, None, FlightAction.LAND):
                expiry = time.monotonic()+.2
                accepted = (channel.submit_control([0]*4, expires_at=expiry) if action is None
                            else channel.submit_action(action, expires_at=expiry))
                until(lambda: completed(channel, accepted['intent_id'],
                    'sent_unconfirmed' if action is None else 'acknowledged'))
        finally:
            stop.set()
            worker.join(timeout=3)
        assert not worker.is_alive() and not errors, errors
        assert [p.cmd_set for p in peer.flight_frames] == [3, 1, 3]
        assert [p.payload for p in (peer.flight_frames[0], peer.flight_frames[-1])] == [b'\1', b'\2']
        assert peer.completion_requests>=2
        by_sequence={}
        for packet in peer.requests:
            previous=by_sequence.setdefault(packet.seq,packet)
            assert previous==packet
        completion=[p for p in peer.requests if (p.cmd_set,p.cmd_id)==(0x51,6)]
        assert len(completion)>=2 and len({p.seq for p in completion})==1
        assert len({p.seq for p in peer.flight_frames})==3
        assert not ({p.seq for p in peer.flight_frames}
                    &{p.seq for p in peer.requests if p.cmd_set==0x51})
        for sequence in by_sequence:
            repeated=[p for p in peer.requests if p.seq==sequence]
            if len(repeated)>1:assert (repeated[0].cmd_set,repeated[0].cmd_id)==(0x51,6)
        assert peer.forwarding == list(range(peer.forwarding[0], peer.forwarding[-1]+1))
    state = result[0]['wlm_mode_follow']
    assert result[0]['reason'] == 'requested' and not result[0]['application_ready']
    assert state['flight_channel']['sent_count'] == 3
    assert state['flight_channel']['revoked']
    assert not state['flight_channel']['flight_ready']
    assert all(row['execution_confirmed'] is False for row in channel.snapshot()['history'])


@pytest.mark.parametrize('failure', ['beacons', 'boot'])
def test_real_udp_discovery_expiry_suspends_control_and_reboot_revokes_generation(tmp_path, failure):
    channel = FlightChannel()
    stop = threading.Event()
    with FlightPeer(channel) as peer:
        worker, result, errors = start_flight_session(tmp_path, channel, peer, stop)
        try:
            until(lambda: channel.snapshot()['route_ready'], timeout=5)
            if failure == 'beacons':
                peer.withhold_beacons = True
                until(lambda:not channel.snapshot()['ordinary_control_ready'],timeout=3)
                assert not channel.snapshot()['revoked']
            else:
                peer.boot_nonce += 1
                until(lambda: channel.snapshot()['revoked'], timeout=3)
            with pytest.raises(RuntimeError, match='not_ready'):
                channel.submit_action(FlightAction.TAKEOFF, expires_at=time.monotonic()+.2)
            assert peer.flight_frames == []
            if failure=='beacons':
                landing=channel.submit_landing(expires_at=time.monotonic()+.2)
                until(lambda:completed(channel,landing['intent_id'],'acknowledged'))
                peer.withhold_beacons=False
                until(lambda:channel.snapshot()['ordinary_control_ready'])
                assert [p.payload[0] for p in peer.flight_frames if (p.cmd_set,p.cmd_id)==(3,0x2a)]==[2]
        finally:
            stop.set()
            worker.join(timeout=3)
        assert not worker.is_alive() and not errors, errors
    state = result[0]['wlm_mode_follow']
    assert state['reason'] == ('requested' if failure == 'beacons' else 'peer_session_changed')
    assert result[0]['received'] > 20 and state['flight_channel']['sent_count'] == (2 if failure=='beacons' else 0)


def test_flight_opt_in_is_typed_and_rejected_before_files_or_sockets(tmp_path):
    path = tmp_path/'must-not-exist'/'capture.jsonl'
    for arguments in ({'flight_channel': object()}, {'flight_channel': FlightChannel()}):
        with pytest.raises(ValueError, match='owned Wi-Fi'):
            run_session(capture_path=path, **arguments)
        assert not path.parent.exists()


def test_real_udp_foreign_hash_foreign_broadcast_encrypted_and_uncorrelated_acks_cannot_complete_action(tmp_path):
    channel=FlightChannel()
    stop=threading.Event()
    with FlightPeer(channel,hold_action_ack=True) as peer:
        worker,result,errors=start_flight_session(tmp_path,channel,peer,stop)
        try:
            until(lambda:channel.snapshot()['route_ready'],timeout=5)
            intent=channel.submit_action(FlightAction.TAKEOFF,expires_at=time.monotonic()+.2)
            until(lambda:channel.snapshot()['ignored_responses']>=2)
            assert channel.snapshot()['action']['state']=='sent'
            assert not completed(channel,intent['intent_id'],'acknowledged')
            peer.release_ack.set()
            until(lambda:completed(channel,intent['intent_id'],'acknowledged'))
            assert len(peer.flight_frames)==1 and channel.snapshot()['sent_count']==1
        finally:
            stop.set()
            worker.join(timeout=3)
        assert not worker.is_alive() and not errors,errors


def test_real_udp_observed_broadcast_takeoff_and_direct_land_acks_complete_sent_actions(tmp_path):
    channel=FlightChannel();stop=threading.Event()
    with FlightPeer(channel,observed_action_replies=True) as peer:
        worker,result,errors=start_flight_session(tmp_path,channel,peer,stop)
        try:
            until(lambda:channel.snapshot()['route_ready'],timeout=5)
            for action in (FlightAction.TAKEOFF,FlightAction.LAND):
                intent=channel.submit_action(action,expires_at=time.monotonic()+.2)
                until(lambda:completed(channel,intent['intent_id'],'acknowledged'))
                receipt=channel.receipt(intent['intent_id'])
                assert receipt['acknowledged_at']-receipt['sent_at']<.5
                assert DumlPacket.decode(bytes.fromhex(receipt['response'])).cmd_type==0x80
        finally:stop.set();worker.join(timeout=3)
        assert not worker.is_alive() and not errors,errors
    assert [p.payload for p in peer.flight_frames]==[b'\1',b'\2']
    assert channel.snapshot()['sent_count']==2


def test_camera_with_flight_channel_keeps_strict_two_second_peer_bound():
    client = SwudpSession(wlm_identity=IDENTITY, wlm_wifi_liveview=True,
                          wifi_backend_ready=lambda: True,
                          camera_channel=CameraChannel(), flight_channel=FlightChannel())
    assert client.wlm_link.maximum_age == 2
    assert client.wlm_link.query_timeout == 2


def test_flight_only_reconnect_retries_lost_first_read_then_lands_over_real_udp(tmp_path):
    class SecondQueryPeer(FlightPeer):
        def __init__(self,channel):
            super().__init__(channel);self.capability_replies=0

        def response_datagrams(self,raw,outgoing):
            kept=[]
            for outer,_ in DumlStreamDecoder().feed(outgoing):
                inner=unpack_forwarded(outer.payload).packet
                if (inner.cmd_set,inner.cmd_id)==(0x51,0x41):
                    self.capability_replies+=1
                    if self.capability_replies==1:continue
                kept.append(outer.encode())
            return super().response_datagrams(raw,b''.join(kept))

    channel=FlightChannel();stop=threading.Event()
    with SecondQueryPeer(channel) as peer:
        worker,result,errors=start_flight_session(tmp_path,channel,peer,stop)
        try:
            until(lambda:channel.snapshot()['route_ready'],timeout=7)
            queries=[p for p in peer.requests if (p.cmd_set,p.cmd_id)==(0x51,0x41)]
            assert len(queries)==2 and queries[0].seq!=queries[1].seq
            assert peer.completed and peer.flight_frames==[]
            landing=channel.submit_landing(expires_at=time.monotonic()+.2)
            until(lambda:completed(channel,landing['intent_id'],'acknowledged'))
        finally:stop.set();worker.join(timeout=3)
        assert not worker.is_alive() and not errors,errors
    assert result[0]['wlm_mode_follow']['capability_query_attempt_limit']==3
    assert result[0]['wlm_mode_follow']['capability_queries_sent']==2
    assert [p.payload for p in peer.flight_frames if (p.cmd_set,p.cmd_id)==(3,0x2a)]==[b'\2']


def test_action_ack_is_published_before_slow_diagnostic_capture_and_keeps_receipt_time(tmp_path,monkeypatch):
    channel=FlightChannel();stop=threading.Event();seen=[]
    original=CaptureRecorder.record
    def record(recorder,**fields):
        if fields['direction']=='rx':
            decoded,_=describe_transport(fields['raw'])
            offset=decoded.get('qos_aggregate_offset',decoded.get('payload_offset'))
            length=decoded.get('qos_aggregate_length',decoded.get('payload_length',0))
            if offset is not None:
                for outer,_ in DumlStreamDecoder().feed(fields['raw'][offset:offset+length]):
                    if (outer.cmd_set,outer.cmd_id)!=(0x51,1):continue
                    inner=unpack_forwarded(outer.payload).packet
                    if (inner.cmd_set,inner.cmd_id,inner.cmd_type)==(3,0x2a,0xc0):
                        # The current intent must already have the wire ACK,
                        # before this deliberately slow diagnostic writer.
                        receipt=channel.receipt(intent['intent_id'])
                        seen.append((receipt,fields.get('monotonic_ns'),time.monotonic()))
                        time.sleep(.06)
        return original(recorder,**fields)
    monkeypatch.setattr(CaptureRecorder,'record',record)
    with FlightPeer(channel) as peer:
        worker,result,errors=start_flight_session(tmp_path,channel,peer,stop)
        try:
            until(lambda:channel.snapshot()['route_ready'],timeout=5)
            intent=channel.submit_action(FlightAction.TAKEOFF,expires_at=time.monotonic()+.2)
            until(lambda:bool(seen))
        finally:
            stop.set();worker.join(timeout=3)
        assert not worker.is_alive() and not errors,errors
    assert len(seen)==1
    receipt,received_ns,diagnostic_started=seen[0]
    assert receipt['state']=='acknowledged'
    assert receipt['acknowledged_at']==received_ns/1e9<=diagnostic_started
    assert any(row['monotonic_ns']==received_ns for row in load_capture(tmp_path/'flight.jsonl'))
