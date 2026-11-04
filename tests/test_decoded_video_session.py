"""Original receipts through the real session, local UDP and native decoder."""
import json
import time

import pytest

from neo2_control.session import SwudpSession,run_session
from neo2_control.transport import ReliableTransport
from neo2_control.video_worker import ManagedHevcDecoder
from test_receiver import stream
from test_session import acceptance
from test_video_lineage import encoded
from test_video_session import VideoPeer

OWNER,GENERATION='a'*48,'b'*24


class ReadyVideoPeer(VideoPeer):
    """Keep the real socket clocks; send the short fixture after initialization."""
    def __init__(self,raw,decoder):
        self.decoder=decoder
        super().__init__(raw)

    def _run(self):
        deadline=time.monotonic()+3
        while not self.stop.is_set() and time.monotonic()<deadline:
            if self.decoder.status()['ready']:
                return super()._run()
            time.sleep(.005)
        if not self.stop.is_set():self.errors.append(TimeoutError('decoder did not initialize'))


def test_session_handoff_has_distinct_original_and_release_times(monkeypatch):
    decoder=ManagedHevcDecoder(OWNER,GENERATION)
    calls=[]
    monkeypatch.setattr(decoder,'feed',lambda raw,**times:calls.append((raw,times)) or True)
    client=SwudpSession(transport=ReliableTransport(session=0x1234,base_sequence=0x4000),decoded_video=decoder)
    client.start(0.);client.receive(acceptance(),0.)
    client.receive(stream(0x4010,b'b'),.1)
    client.receive(stream(0x4010,b'b'),.2)
    client.receive(stream(0x4008,b'a',end=0x4010),.3)
    assert calls==[(b'a',{'delivered_at':.3,'received_at':.3}),(b'b',{'delivered_at':.3,'received_at':.1})]
    client.close(.4)
    assert decoder.current() is None
    assert decoder.status()['error']=='session_closed:requested'


def test_real_udp_handoff_decodes_and_retires_observations_on_session_end(encoded,tmp_path):
    source,expected=encoded
    decoder=ManagedHevcDecoder(OWNER,GENERATION)
    peer=ReadyVideoPeer(source.read_bytes(),decoder)
    observations=[];cleanups=[]
    def snapshot(state):
        frame=decoder.current()
        if frame is not None and (not observations or observations[-1].decoded_index!=frame.decoded_index):
            observations.append(frame)
    try:
        result=run_session(capture_path=tmp_path/'capture.jsonl',host='127.0.0.1',port=peer.port,
            duration=1.,decoded_video=decoder,snapshot_sink=snapshot,decoded_video_cleanup_sink=cleanups.append)
    finally:peer.close()
    assert result.state=='closed' and result.reason=='duration_elapsed'
    assert result.video_bytes_delivered==source.stat().st_size
    assert result.decoded_video['received_frames']>0
    assert result.decoded_video['cleanup_verified'] and result.decoded_video['swudp_attached']
    assert observations
    for frame in observations:
        assert frame.rgb_sha256==expected[frame.decoded_index]
        assert frame.original_packet_receipt_known
        assert frame.pc_first_byte_received_at<=frame.pc_first_byte_delivered_at<=frame.decoded_at
        assert not frame.record()['flight_authorized']
    assert decoder.current() is None
    assert cleanups and all(s['cleanup_verified'] for s in cleanups)
    assert sum(raw[6] in (4,6) for raw in peer.requests)>=2
    assert all(raw[6] not in (5,8) for raw in peer.requests)


def test_decoder_failure_does_not_interrupt_udp_window_acknowledgments(tmp_path):
    pytest.importorskip('av')
    decoder=ManagedHevcDecoder(OWNER,GENERATION)
    peer=ReadyVideoPeer(b'not HEVC',decoder)
    try:
        result=run_session(capture_path=tmp_path/'capture.jsonl',host='127.0.0.1',port=peer.port,
            duration=.7,decoded_video=decoder)
    finally:peer.close()
    assert result.state=='closed' and result.reason=='duration_elapsed'
    assert result.video_bytes_delivered==8
    assert result.decoded_video['error']=='worker_exit_without_complete_stream'
    assert result.decoded_video['cleanup_verified']
    assert sum(raw[6] in (4,6) for raw in peer.requests)>=3
    assert decoder.current() is None


def test_closed_session_cannot_expose_old_pixels_after_new_generation(encoded,tmp_path):
    source,_=encoded
    decoders=[]
    for generation in ('b'*24,'c'*24):
        decoder=ManagedHevcDecoder(OWNER,generation);peer=ReadyVideoPeer(source.read_bytes(),decoder)
        try:
            result=run_session(capture_path=tmp_path/(generation+'.jsonl'),host='127.0.0.1',port=peer.port,
                duration=.5,decoded_video=decoder)
        finally:peer.close()
        assert result.decoded_video['cleanup_verified'] and decoder.current() is None
        decoders.append(decoder)
    assert all(d.latest.peek() is None for d in decoders)
    with pytest.raises(ValueError,match='unused_owned_decoder'):
        run_session(capture_path=tmp_path/'reused.jsonl',decoded_video=decoders[0])
    assert not (tmp_path/'reused.jsonl').exists()
