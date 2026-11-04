"""Camera mailbox races, deadlines and ACK admission without any device I/O."""
from dataclasses import replace

import pytest

from neo2_control.camera import CameraChannel
from neo2_control.duml import DumlPacket,DumlSequence


def reply(request,*,kind=0x80,status=0):
    return DumlPacket(request.receiver,request.sender,request.seq,kind,
                      request.cmd_set,request.cmd_id,bytes([status]))


def state_push(*,mode=1,recording=0):
    return DumlPacket(1,2,0x8888,0,2,0x80,
                      (recording<<6).to_bytes(4,'little')+bytes([mode])+bytes(55))


def test_unsent_intent_cannot_be_acknowledged_and_readiness_loss_cancels_scheduling():
    now=[0.0]
    channel=CameraChannel(clock=lambda:now[0])
    channel.set_ready(True,0)
    channel.submit('pitch',{'degrees':-12.5})
    request=channel.next_packet(.1,DumlSequence(current=0x9000).next)
    channel.receive(reply(request),.2)
    assert channel.snapshot()['last_command']['state']=='scheduled'
    assert channel.snapshot()['sent_count']==0 and channel.snapshot()['commanded_pitch'] is None
    channel.set_ready(False,.3)
    channel.receive(reply(request),.4)
    assert channel.next_packet(.4,DumlSequence().next) is None
    assert channel.snapshot()['last_command']['state']=='cancelled'
    assert not channel.snapshot()['execution_confirmed']


def test_queued_expiry_stale_readiness_and_late_ack_never_report_acceptance():
    now=[0.0]
    channel=CameraChannel(clock=lambda:now[0])
    channel.set_ready(True,0)
    channel.submit('recenter',{})
    now[0]=2.01
    assert channel.next_packet(2.01,DumlSequence().next) is None
    assert channel.snapshot()['last_command']['state']=='expired'
    with pytest.raises(RuntimeError,match='camera_not_ready'):channel.submit('record',{})
    channel.set_ready(True,2.01)
    channel.submit('record',{})
    request=channel.next_packet(2.01,DumlSequence().next)
    channel.sent(request.seq,2.02)
    now[0]=3.02
    channel.receive(reply(request,kind=0xc0),3.02)
    state=channel.snapshot()
    assert state['last_command']['state']=='timed_out'
    assert state['sent_count']==1 and state['recording'] is None
    assert not state['execution_confirmed']


def test_strict_ack_identity_status_and_explicit_cancel_have_no_false_execution():
    now=[1.0]
    channel=CameraChannel(clock=lambda:now[0])
    channel.set_ready(True,1)
    channel.submit('record',{})
    request=channel.next_packet(1,DumlSequence(current=0x9123).next)
    channel.sent(request.seq,1)
    valid=reply(request,kind=0xc0,status=7)
    invalid=[replace(valid,seq=valid.seq^1),replace(valid,sender=4),
             replace(valid,receiver=3),replace(valid,cmd_set=4),
             replace(valid,cmd_id=3),replace(valid,cmd_type=0xc1),replace(valid,payload=b'')]
    for packet in invalid:channel.receive(packet,1.1)
    assert channel.snapshot()['last_command']['state']=='sent'
    channel.receive(valid,1.2)
    state=channel.snapshot()
    assert state['ignored_frames']==len(invalid)
    assert state['last_command']['state']=='rejected' and state['last_command']['status_code']==7
    assert not state['last_command']['execution_confirmed'] and state['recording'] is None
    now[0]=1.2
    channel.set_ready(True,1.2);channel.submit('stop_record',{})
    request=channel.next_packet(1.2,DumlSequence().next);channel.sent(request.seq,1.2)
    channel.cancel(1.3,'requested');channel.receive(reply(request),1.4)
    assert channel.snapshot()['last_command']['state']=='cancelled'
    assert channel.snapshot()['last_command']['reason']=='requested'
    assert not channel.snapshot()['ready']


def test_detached_snapshot_does_not_advance_owner_clock_or_renew_readiness():
    now=[10.0]
    channel=CameraChannel(clock=lambda:now[0])
    channel.set_ready(True,10)
    channel.submit('recenter',{})
    now[0]=12.1
    snapshot=channel.snapshot()
    assert snapshot['last_command']['state']=='expired' and not snapshot['ready']
    assert channel.last_time==10 and channel.pending['state']=='queued'
    channel.cancel(12.1,'session_closed')
    assert channel.snapshot()['last_command']['state']=='expired'
    assert channel.pending is None


def test_pitch_only_confirms_actual_send_and_recording_requires_a_fresh_camera_push():
    now=[1.0]
    channel=CameraChannel(clock=lambda:now[0])
    channel.set_ready(True,1)
    channel.submit('pitch',{'degrees':-12.5})
    request=channel.next_packet(1,DumlSequence().next)
    assert channel.snapshot()['commanded_pitch'] is None
    channel.sent(request.seq,1.01)
    snapshot=channel.snapshot()
    assert snapshot['last_command']['state']=='sent_unconfirmed'
    assert snapshot['sent_count']==1 and snapshot['commanded_pitch']==-12.5
    assert not snapshot['pending'] and not snapshot['execution_confirmed']
    channel.receive(reply(request),1.02)
    assert channel.snapshot()['last_command']['state']=='sent_unconfirmed'

    body=(2<<6).to_bytes(4,'little')+bytes(56)
    push=DumlPacket(1,2,0x8888,0,2,0x80,body)
    for invalid in (replace(push,sender=4),replace(push,receiver=3),
                    replace(push,cmd_type=0x80),replace(push,payload=body[:-1])):
        channel.receive(invalid,1.03)
    assert channel.snapshot()['recording'] is None
    channel.receive(push,1.04)
    assert channel.snapshot()['recording'] is True
    assert channel.snapshot()['camera_status']['capture_saved_confirmed'] is False
    channel.receive(replace(push,receiver=0x1c,payload=bytes(60)),1.05)
    assert channel.snapshot()['recording'] is False
    assert len(channel.snapshot()['camera_observations'])==2
    now[0]=2.06
    assert channel.snapshot()['recording'] is None
    assert channel.snapshot()['camera_status']['fresh'] is False


@pytest.mark.parametrize('first',['ack','push'])
def test_mode_change_requires_both_ack_and_post_send_idle_push_before_the_action(first):
    now=[1.0]
    channel=CameraChannel(clock=lambda:now[0]);channel.set_ready(True,1)
    sequence=DumlSequence(current=0x8123)
    channel.submit('photo',{})
    mode=channel.next_packet(1,sequence.next)
    assert (mode.cmd_set,mode.cmd_id,mode.payload)==(2,0x10,b'\0')
    channel.sent(mode.seq,1.01)
    signals={'ack':reply(mode,kind=0xc0),'push':state_push(mode=0)}
    channel.receive(signals[first],1.02)
    assert channel.next_packet(1.03,sequence.next) is None
    assert channel.snapshot()['last_command']['state'] in ('sent','waiting_mode')
    channel.receive(signals['push' if first=='ack' else 'ack'],1.04)
    action=channel.next_packet(1.05,sequence.next)
    assert (action.cmd_set,action.cmd_id,action.payload)==(2,1,b'\1')
    assert action.seq!=mode.seq
    channel.sent(action.seq,1.06)
    channel.receive(reply(mode),1.07)
    assert channel.snapshot()['last_command']['state']=='sent'
    channel.receive(reply(action),1.08)
    assert channel.snapshot()['last_command']['state']=='acknowledged'
    assert channel.snapshot()['sent_count']==2 and not channel.snapshot()['execution_confirmed']


@pytest.mark.parametrize('cached',['unknown','stale'])
def test_unknown_or_stale_matching_work_mode_still_requires_mode_selection(cached):
    now=[0.0]
    channel=CameraChannel(clock=lambda:now[0])
    if cached=='stale':channel.receive(state_push(mode=1),0)
    now[0]=1.1;channel.set_ready(True,1.1);channel.submit('record',{})
    request=channel.next_packet(1.1,DumlSequence().next)
    assert (request.cmd_set,request.cmd_id,request.payload)==(2,0x10,b'\1')
    assert channel.snapshot()['last_command']['stage']=='select_mode'


@pytest.mark.parametrize('busy',[1,2,3])
def test_recording_or_transition_push_blocks_photo_and_record_but_allows_stop(busy):
    channel=CameraChannel(clock=lambda:1)
    channel.set_ready(True,1);channel.receive(state_push(recording=busy),1)
    for action in ('photo','record'):
        with pytest.raises(RuntimeError,match='camera_busy'):channel.submit(action,{})
    channel.submit('stop_record',{})
    request=channel.next_packet(1,DumlSequence().next)
    assert (request.cmd_set,request.cmd_id,request.payload)==(2,2,b'\0')


def test_disconnect_between_mode_confirmation_and_action_prevents_the_action():
    now=[1.0]
    channel=CameraChannel(clock=lambda:now[0]);channel.set_ready(True,1)
    channel.submit('record',{})
    mode=channel.next_packet(1,DumlSequence().next);channel.sent(mode.seq,1.01)
    channel.receive(reply(mode),1.02);channel.receive(state_push(mode=1),1.03)
    assert channel.snapshot()['last_command']['stage']=='action'
    channel.revoke()
    assert channel.next_packet(1.04,DumlSequence().next) is None
    assert channel.snapshot()['last_command']['state']=='cancelled'
    assert channel.snapshot()['sent_count']==1 and not channel.snapshot()['ready']


@pytest.mark.parametrize('change',['stale','different_mode'])
def test_confirmed_work_mode_must_still_be_current_at_actual_action_scheduling(change):
    now=[1.0]
    channel=CameraChannel(clock=lambda:now[0]);channel.set_ready(True,1)
    sequence=DumlSequence(current=0x8123)
    channel.submit('record',{})
    mode=channel.next_packet(1,sequence.next);channel.sent(mode.seq,1.01)
    channel.receive(reply(mode),1.02);channel.receive(state_push(mode=1),1.03)
    if change=='different_mode':channel.receive(state_push(mode=0),1.04)
    else:now[0]=2.1
    channel.set_ready(True,now[0] if change=='stale' else 1.05)
    allocated=[]
    assert channel.next_packet(channel.last_time,lambda:allocated.append(True)) is None
    assert not allocated and channel.snapshot()['sent_count']==1
    assert channel.snapshot()['last_command']['state']=='cancelled'
    assert channel.snapshot()['last_command']['reason']=='camera_state_changed'


def test_stop_record_accepts_correlated_ack_within_native_1500ms_timeout():
    now=[1.0]
    channel=CameraChannel(clock=lambda:now[0]);channel.set_ready(True,1)
    channel.submit('stop_record',{})
    packet=channel.next_packet(1,DumlSequence().next);channel.sent(packet.seq,1)
    now[0]=2.2
    channel.receive(reply(packet),2.2)
    assert channel.snapshot()['last_command']['state']=='acknowledged'
    assert channel.snapshot()['last_command']['status_code']==0
    assert not channel.snapshot()['execution_confirmed']


@pytest.mark.parametrize('path,busy',[
    ('before_mode',1),('before_direct_action',2),('after_mode_confirmation',3)])
def test_busy_push_racing_with_accepted_intent_cancels_before_allocating_another_sequence(path,busy):
    now=[1.0]
    channel=CameraChannel(clock=lambda:now[0]);channel.set_ready(True,1)
    allocated=[]
    def sequence():
        value=0x8123+len(allocated);allocated.append(value);return value
    if path!='after_mode_confirmation':
        channel.receive(state_push(mode=0 if path=='before_direct_action' else 1),1)
    channel.submit('photo',{})
    if path=='after_mode_confirmation':
        mode=channel.next_packet(1,sequence)
        channel.sent(mode.seq,1.01);channel.receive(reply(mode),1.02)
        channel.receive(state_push(mode=0),1.03)
        assert channel.snapshot()['last_command']['stage']=='action'
    channel.receive(state_push(mode=0 if path!='before_mode' else 1,recording=busy),1.04)
    assert channel.next_packet(1.05,sequence) is None
    snapshot=channel.snapshot()
    assert snapshot['last_command']['state']=='cancelled'
    assert snapshot['last_command']['reason']=='camera_busy'
    assert len(allocated)==(1 if path=='after_mode_confirmation' else 0)
    assert snapshot['sent_count']==len(allocated) and not snapshot['pending']
    assert channel.next_packet(1.06,sequence) is None


def test_waiting_mode_deadline_expires_in_snapshot_and_owner_before_late_push():
    now=[1.0]
    channel=CameraChannel(clock=lambda:now[0]);channel.set_ready(True,1)
    sequence=DumlSequence()
    channel.submit('record',{})
    mode=channel.next_packet(1,sequence.next);channel.sent(mode.seq,1.01)
    channel.receive(reply(mode),1.02)
    assert channel.snapshot()['last_command']['state']=='waiting_mode'
    now[0]=3.02
    assert channel.snapshot()['last_command']['state']=='timed_out'
    assert channel.pending['state']=='waiting_mode' and channel.last_time==1.02
    channel.receive(state_push(mode=1),3.02)
    assert channel.pending is None
    assert channel.next_packet(3.02,sequence.next) is None
    assert channel.snapshot()['last_command']['state']=='timed_out'
    assert channel.snapshot()['sent_count']==1


@pytest.mark.parametrize('action_sent',[False,True])
def test_overall_deadline_caps_delayed_mode_handshake_and_action_ack(action_sent):
    now=[0.0]
    channel=CameraChannel(clock=lambda:now[0]);channel.set_ready(True,0)
    sequence=DumlSequence()
    channel.submit('record',{})
    channel.set_ready(True,1.8)
    mode=channel.next_packet(1.8,sequence.next);channel.sent(mode.seq,1.9)
    channel.receive(reply(mode),2.8)
    channel.set_ready(True,4.7);channel.receive(state_push(mode=1),4.7)
    assert channel.snapshot()['last_command']['state']=='queued'
    if action_sent:
        action=channel.next_packet(4.8,sequence.next);channel.sent(action.seq,4.9)
    now[0]=5
    snapshot=channel.snapshot()
    assert snapshot['last_command']['state']=='timed_out'
    assert snapshot['last_command']['deadline']>snapshot['last_command']['overall_deadline']
    assert channel.pending is not None  # UI cannot mutate the socket owner's state.
    if action_sent:channel.receive(reply(action),5)
    else:assert channel.next_packet(5,sequence.next) is None
    assert channel.pending is None and not channel.snapshot()['execution_confirmed']
    assert channel.snapshot()['last_command']['state']=='timed_out'
    assert channel.snapshot()['sent_count']==(2 if action_sent else 1)
