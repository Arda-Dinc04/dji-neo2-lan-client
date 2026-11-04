from dataclasses import replace
import json
from pathlib import Path

import pytest

from neo2_control.duml import DumlPacket,DumlSequence
from neo2_control.identity import WlmIdentity
from neo2_control.wlm_modes import WlmModes
from neo2_control.wlm_switch import (WiFiServiceSwitch,WIFI_SERVICE_MODES,SwitchPrepare,
    SwitchCompletion,identity_status,switch_ack,switch_completion,wifi_service_request)

ORACLE=json.loads((Path(__file__).parent/'fixtures/wlm-service-switch-native.json').read_text())
IDENTITY=WlmIdentity('pc-state-synthetic')
PEER=bytes.fromhex(ORACLE['peer_identity_hex'])
OWN=IDENTITY.wlm_device_id
CURRENT=WlmModes(1,1,1,0,0)


def incoming(command,payload,*,sequence=0x8123,kind=0x40):
    return DumlPacket(sender=0xe9,receiver=0xee,seq=sequence,cmd_type=kind,
                      cmd_set=0x51,cmd_id=command,payload=payload)


def prepare_request(**changes):
    return incoming(8,PEER+OWN+bytes([5,4,4]),**changes)


def completion_request(*,status=0,detail=0,**changes):
    return incoming(6,PEER+OWN+bytes([status])+detail.to_bytes(4,'little'),**changes)


def transaction(*,ready=True,commit_ok=True):
    calls=[]
    def prepare(modes):calls.append(('prepare',modes));return ready
    def commit(modes):calls.append(('commit',modes));return commit_ok
    switch=WiFiServiceSwitch(IDENTITY,PEER,CURRENT,DumlSequence(current=0x8000).next,
                            prepare=prepare,commit=commit)
    return switch,calls


def send(switch,when,kind):
    output=switch.next_packet(when)
    assert output is not None and output.kind==kind
    assert switch.next_packet(when) is None
    switch.sent(kind,when)
    return output.packet


def prepared_switch(**options):
    switch,calls=transaction(**options)
    send(switch,0,'trigger')
    switch.receive(prepare_request(),.1)
    send(switch,.1,'prepare_ack')
    return switch,calls


def test_codecs_match_complete_native_inner_frames_without_using_native_uninitialized_sequences():
    expected={f['name']:f for f in ORACLE['source_frames']['frames']}
    packets={'trigger51_02_request':wifi_service_request(CURRENT,0),
        'negotiation51_08_ack':switch_ack(PEER,8,0x8123),
        'completion51_06_request':switch_completion(IDENTITY,PEER,0),
        'completion51_06_ack':switch_ack(PEER,6,0x8123)}
    for name,packet in packets.items():assert packet.encode().hex()==expected[name]['frame_hex']
    assert OWN.hex()==ORACLE['own_identity_hex']
    assert SwitchPrepare.decode(prepare_request().payload).target==OWN
    assert SwitchCompletion.decode(completion_request().payload).status==0
    assert identity_status(packets['negotiation51_08_ack'].payload)==(PEER,0)
    # The last2bytes describe current services, not desired2/1.
    assert wifi_service_request(WlmModes(4,4,4,1,1),7).payload[-2:]==b'\x01\x01'


@pytest.mark.parametrize('decoder,length',[(SwitchPrepare.decode,48),(SwitchPrepare.decode,50),
    (SwitchCompletion.decode,47),(SwitchCompletion.decode,50),(SwitchCompletion.decode,52),
    (identity_status,23),(identity_status,25)])
def test_short_native_unwritten_detail_and_extra_bytes_are_outside_pc_profile(decoder,length):
    with pytest.raises(ValueError):decoder(bytes(length))


@pytest.mark.parametrize('peer_result_first',[False,True])
def test_current_modes_commit_once_only_after_local_prepare_and_both_completions(peer_result_first):
    switch,calls=transaction()
    pending=switch.next_packet(0)
    assert not switch.snapshot(1)['deadline_started'] and not calls
    switch.sent(pending.kind,1)
    switch.receive(prepare_request(),1.1)
    pending=switch.next_packet(1.1)
    assert not calls and not switch.snapshot(1.1)['prepared_local']
    switch.sent(pending.kind,1.2)
    assert calls==[('prepare',WIFI_SERVICE_MODES)]
    own=send(switch,1.3,'completion')
    ack=incoming(6,OWN+b'\0',sequence=own.seq,kind=0x80)
    if peer_result_first:
        switch.receive(completion_request(),1.4)
        send(switch,1.4,'result_ack')
        assert not switch.committed and len(calls)==1
        switch.receive(ack,1.5)
    else:
        switch.receive(ack,1.4)
        assert not switch.committed and len(calls)==1
        switch.receive(completion_request(),1.5)
        pending=switch.next_packet(1.5)
        assert not switch.committed
        switch.sent(pending.kind,1.6)
    assert switch.state=='complete' and switch.committed
    assert calls==[('prepare',WIFI_SERVICE_MODES),('commit',WIFI_SERVICE_MODES)]
    switch.receive(ack,1.7)
    assert switch.next_packet(1.7) is None and len(calls)==2
    assert not switch.snapshot(1.7)['application_ready']


def test_early_peer_completion_is_buffered_until_local_preparation_and_completion_send():
    switch,calls=transaction();send(switch,0,'trigger')
    switch.receive(completion_request(),.05)
    assert switch.next_packet(.05) is None and not calls
    switch.receive(prepare_request(),.1);send(switch,.1,'prepare_ack')
    own=send(switch,.2,'completion');send(switch,.3,'result_ack')
    assert not switch.committed
    switch.receive(incoming(6,OWN+b'\0',sequence=own.seq,kind=0xc0),.4)
    assert switch.committed


def test_trigger_and_completion_retries_use_actual_send_times_and_one_global_deadline():
    switch,calls=transaction()
    first=send(switch,1,'trigger')
    assert switch.next_packet(1.399) is None
    assert send(switch,1.4,'trigger')==first
    switch.receive(prepare_request(),1.5);send(switch,1.5,'prepare_ack')
    complete=send(switch,1.6,'completion')
    assert switch.next_packet(1.799) is None
    assert send(switch,1.801,'completion')==complete
    assert switch.next_packet(4) is None
    assert switch.state=='failed' and switch.reason=='switch_timeout' and not switch.committed
    assert len(calls)==1


@pytest.mark.parametrize('change',['producer','target','modes','source','type','orphan_ack'])
def test_prepare_admission_is_bound_to_current_peer_own_identity_target_modes_and_role(change):
    switch,calls=transaction();send(switch,0,'trigger')
    packet=prepare_request();body=bytearray(packet.payload)
    if change=='producer':body[3]^=1
    elif change=='target':body[26]^=1
    elif change=='modes':body[46:]=b'\x01\x01\x01'
    elif change=='source':packet=replace(packet,sender=0xe8)
    elif change=='type':packet=replace(packet,cmd_type=0x41)
    else:packet=incoming(8,OWN+b'\0',kind=0xc0);body=bytearray(packet.payload)
    switch.receive(replace(packet,payload=bytes(body)),.1)
    assert switch.next_packet(.1) is None and not calls
    assert not switch.prepared_local


@pytest.mark.parametrize('change',['seq','identity','encrypted','error'])
def test_completion_ack_does_not_advance_on_foreign_or_invalid_response(change):
    switch,calls=prepared_switch();own=send(switch,.2,'completion')
    body=bytearray(OWN+b'\0');seq=own.seq;kind=0xc0
    if change=='seq':seq^=1
    elif change=='identity':body[3]^=1
    elif change=='encrypted':kind=0xc1
    else:body[-1]=7
    switch.receive(incoming(6,bytes(body),sequence=seq,kind=kind),.3)
    assert not switch.own_completion_ack and not switch.committed
    assert len(calls)==1
    if change=='error':assert switch.state=='failed'


def test_peer_error_can_be_acknowledged_before_prepare_without_claiming_local_success():
    switch,calls=transaction();send(switch,0,'trigger')
    switch.receive(completion_request(status=7,detail=0x12345678),.1)
    ack=send(switch,.1,'result_ack')
    assert ack.payload==PEER+b'\x07'
    assert switch.state=='failed' and switch.reason=='peer_completion_error'
    assert not calls and not switch.committed


def test_local_prepare_failure_reports_nonzero_completion_and_never_commits():
    switch,calls=prepared_switch(ready=False)
    result=send(switch,.2,'completion')
    assert SwitchCompletion.decode(result.payload).status!=0
    assert switch.state=='failed' and switch.reason=='local_prepare_failed'
    assert len(calls)==1 and not switch.committed


def test_commit_failure_and_cancelled_pending_socket_never_claim_configured_services():
    switch,calls=prepared_switch(commit_ok=False);own=send(switch,.2,'completion')
    switch.receive(incoming(6,OWN+b'\0',sequence=own.seq,kind=0xc0),.3)
    switch.receive(completion_request(),.4);send(switch,.4,'result_ack')
    assert switch.state=='failed' and switch.reason=='local_commit_failed' and not switch.committed
    other,_=transaction();pending=other.next_packet(0);other.cancel(.1,'requested')
    assert other.next_packet(.2) is None
    with pytest.raises(RuntimeError):other.sent(pending.kind,.2)


def test_actual_send_completion_crossing_global_deadline_remains_failed_without_raising():
    switch,calls=prepared_switch()
    send(switch,.2,'completion')
    switch.receive(completion_request(),2.999)
    pending=switch.next_packet(2.999)
    assert pending.kind=='result_ack'
    switch.sent(pending.kind,3)
    assert switch.reason=='switch_timeout' and not switch.committed
    assert len(calls)==1


def test_completed_transaction_replies_to_identical_peer_completion_retries_without_recommitting():
    switch,calls=prepared_switch()
    own=send(switch,.2,'completion')
    switch.receive(incoming(6,OWN+b'\0',sequence=own.seq,kind=0x80),.3)
    switch.receive(completion_request(),.4);send(switch,.4,'result_ack')
    assert switch.committed and len(calls)==2
    switch.receive(completion_request(sequence=0x2345),.5)
    ack=send(switch,.5,'result_ack')
    assert ack.seq==0x2345 and len(calls)==2
    switch.receive(completion_request(detail=123),.6)
    assert switch.next_packet(.6) is None
    switch.receive(completion_request(),3.5)
    assert switch.next_packet(3.5) is None
    switch.receive(completion_request(),3.6)
    switch.cancel(3.6)
    assert switch.next_packet(3.6) is None
