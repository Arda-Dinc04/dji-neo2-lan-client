"""Kernel queue evidence and bounded owner-cost accounting, localhost only."""
import errno
import json
import socket
import struct
import threading
import time

import pytest

from neo2_control.capture import CaptureRecorder, load_capture
import neo2_control.udp_observation as observation
from neo2_control.udp_observation import OwnerCosts, UdpReceiptObserver


class OptionSocket:
    def __init__(self, *, unsupported=False):
        self.unsupported = unsupported

    def setsockopt(self, *_args):
        if self.unsupported:raise OSError(errno.ENOPROTOOPT, 'fixture')

    def recvmsg(self, *_args):
        raise AssertionError('pure metadata fixture must not receive')


@pytest.fixture
def observed(monkeypatch):
    # Fixed ABI in pure tests, independent of the host's socket constants.
    monkeypatch.setattr(observation,'_TIMESTAMPNS_NEW',64)
    monkeypatch.setattr(observation,'_RXQ_OVFL',40)
    return UdpReceiptObserver(OptionSocket())


def ancillary(stamp, drops=None):
    sec,nsec=divmod(stamp,1_000_000_000)
    values=[(socket.SOL_SOCKET,64,struct.pack('=qq',sec,nsec))]
    if drops is not None:values.append((socket.SOL_SOCKET,40,struct.pack('=I',drops)))
    return values


def test_queue_delay_keeps_clock_domains_and_operational_receipt_distinct(observed):
    value=observed.observe(ancillary(1_000_000_000,3),0,100,1_060_000_000,120)
    assert value['kernel_realtime_ns']==1_000_000_000
    assert value['kernel_to_userspace_ns']==60_000_000
    assert value['clock_pair_span_ns']==20
    assert value['socket_drop_counter']==3
    summary=observed.snapshot()
    assert summary['diagnostic_only'] and summary['operational_clock']=='userspace_monotonic'
    assert summary['reported_socket_drops']==3
    assert summary['kernel_to_userspace_maximum_ns']==60_000_000
    value['kernel_to_userspace_ns']=0
    summary['last']['kernel_to_userspace_ns']=0
    assert observed.snapshot()['last']['kernel_to_userspace_ns']==60_000_000


@pytest.mark.parametrize('metadata,flags,wall,end,reason',[
    ([],0,200,110,'kernel_timestamp_unavailable'),
    (ancillary(100),socket.MSG_CTRUNC,200,110,'ancillary_truncated'),
    ([(socket.SOL_SOCKET,64,b'bad')],0,200,110,'ancillary_malformed'),
    ([(socket.SOL_SOCKET,64,struct.pack('=qq',0,1_000_000_000))],0,200,110,'ancillary_malformed'),
    (ancillary(100)*2,0,200,110,'ancillary_malformed'),
    (ancillary(300),0,200,110,'kernel_timestamp_in_future'),
    (ancillary(100),0,200,20_000_100,'clock_pair_delayed'),
])
def test_incomplete_or_untrustworthy_metadata_cannot_claim_queue_latency(
        observed,metadata,flags,wall,end,reason):
    value=observed.observe(metadata,flags,100,wall,end)
    assert value['kernel_to_userspace_ns'] is None
    assert value['delay_unavailable_reason']==reason
    assert observed.snapshot()['unusable_timestamps']==1


def test_realtime_step_suppresses_estimate_instead_of_changing_monotonic_receipt(observed):
    first=observed.observe(ancillary(4_000),0,1_000,5_000,1_001)
    assert first['kernel_to_userspace_ns']==1_000
    second=observed.observe(ancillary(1_000_004_000),0,2_000,1_000_005_000,2_001)
    assert second['delay_unavailable_reason']=='realtime_discontinuity'
    assert second['kernel_to_userspace_ns'] is None


def test_drop_counter_wrap_is_accumulated_without_inventing_radio_loss(observed):
    observed.observe(ancillary(10,0xfffffffe),0,100,20,101)
    observed.observe(ancillary(20,2),0,110,30,111)
    assert observed.snapshot()['reported_socket_drops']==0x100000002
    assert 'radio_loss' not in observed.snapshot()


def test_unsupported_options_keep_original_bytes_and_userspace_clocks(monkeypatch):
    monkeypatch.setattr(observation,'_TIMESTAMPNS_NEW',64)
    monkeypatch.setattr(observation,'_RXQ_OVFL',40)
    class Fallback(OptionSocket):
        def recv(self, maximum):
            assert maximum==65535
            return b'unchanged packet'
    ticks=iter((123,125))
    observer=UdpReceiptObserver(Fallback(unsupported=True),
        monotonic_ns=lambda:next(ticks),realtime_ns=lambda:1000)
    raw,mono,wall,details=observer.receive()
    assert (raw,mono,wall)==(b'unchanged packet',123,1000)
    assert details['kernel_realtime_ns'] is None
    assert observer.snapshot()['option_errors']=={'timestamp':'ENOPROTOOPT','overflow':'ENOPROTOOPT'}


@pytest.mark.skipif(not observation._GENERIC_LINUX,reason='Linux generic socket ABI')
def test_real_kernel_timestamp_separates_queue_wait_from_protocol_processing():
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as receiver, \
            socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
        receiver.bind(('127.0.0.1',0));receiver.settimeout(1)
        observer=UdpReceiptObserver(receiver)
        assert observer.timestamp_enabled,observer.snapshot()
        sent_wall=time.time_ns()
        sender.sendto(b'queued-before-userspace-read',receiver.getsockname())
        time.sleep(.06)
        before_read=time.monotonic_ns()
        raw,mono,wall,details=observer.receive()
        assert raw==b'queued-before-userspace-read'
        assert before_read<=mono<=time.monotonic_ns()
        assert sent_wall<=details['kernel_realtime_ns']<=wall
        assert details['kernel_to_userspace_ns']>=40_000_000
        assert details['clock_pair_span_ns']<details['kernel_to_userspace_ns']


def test_owner_costs_record_exceptions_and_remain_bounded():
    ticks=iter((100,150,200,280))
    costs=OwnerCosts(clock=lambda:next(ticks))
    with costs.measure('protocol_delivery'):pass
    with pytest.raises(RuntimeError):
        with costs.measure('protocol_delivery'):raise RuntimeError('fixture')
    assert costs.snapshot()['protocol_delivery']=={
        'count':2,'total_ns':130,'maximum_ns':80,'last_ns':80}
    with pytest.raises(ValueError,match='unknown_owner_cost_stage'):
        with costs.measure('arbitrary_unbounded_key'):pass
    assert len(costs.values)==1


@pytest.mark.skipif(not observation._GENERIC_LINUX,reason='Linux generic socket ABI')
def test_real_session_distinguishes_slow_delivery_from_next_packet_queue_wait(tmp_path,monkeypatch):
    from neo2_control.session import SwudpSession, run_session
    from neo2_control.transport import describe_transport
    from test_session import acceptance, status
    stopped=threading.Event()
    errors=[]
    peer=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    peer.bind(('127.0.0.1',0));peer.settimeout(1)

    def serve():
        try:
            syn,address=peer.recvfrom(65535)
            session=describe_transport(syn)[0]['session']
            anchor=int.from_bytes(syn[8:10],'little')
            peer.sendto(acceptance(session),address)
            peer.sendto(status(session,ack=anchor,anchor=anchor),address)
            stopped.wait(2)
        except Exception as error:errors.append(error)

    receive=SwudpSession.receive
    def slow_delivery(self,raw,now):
        receive(self,raw,now)
        time.sleep(.04)
    monkeypatch.setattr(SwudpSession,'receive',slow_delivery)
    thread=threading.Thread(target=serve);thread.start()
    try:
        result=run_session(host='127.0.0.1',port=peer.getsockname()[1],
            capture_path=tmp_path/'capture.jsonl',duration=.2)
    finally:
        stopped.set();thread.join(3);peer.close()
    assert not thread.is_alive() and not errors
    assert result.state=='closed' and result.socket_cleanup_verified
    rx=[r for r in load_capture(result.capture_path) if r['direction']=='rx']
    assert len(rx)==2
    assert rx[0]['observation']['protocol_delivery_ns']>=30_000_000
    assert rx[1]['observation']['kernel_to_userspace_ns']>=30_000_000
    assert rx[0]['monotonic_ns']<rx[1]['monotonic_ns']
    assert result.udp_observation['receive']['receipts']==2
    assert result.udp_observation['receive']['diagnostic_only']
    assert result.udp_observation['owner_costs']['protocol_delivery']['count']==2
    assert result.udp_observation['owner_costs']['capture_append']['count']==result.transmitted+2


def test_capture_retains_diagnostics_without_changing_receipt_or_raw_bytes(tmp_path):
    path=tmp_path/'capture.jsonl'
    metadata={'kernel_realtime_ns':900,'kernel_to_userspace_ns':100}
    recorder=CaptureRecorder(path,decode_annotations=False)
    event=recorder.record(interface='udp',direction='rx',peer='127.0.0.1:1',
        raw=b'\x00\xff',timestamp_ns=1000,monotonic_ns=100,
        observation=metadata)
    metadata['kernel_realtime_ns']=0
    row=load_capture(path)[0]
    assert row['observation']==event.observation=={
        'kernel_realtime_ns':900,'kernel_to_userspace_ns':100}
    assert row['raw_hex']=='00ff' and row['timestamp_ns']==1000 and row['monotonic_ns']==100
    recorder.record(interface='udp',direction='tx',peer='127.0.0.1:1',raw=b'old format')
    assert 'observation' not in json.loads(path.read_text().splitlines()[1])
