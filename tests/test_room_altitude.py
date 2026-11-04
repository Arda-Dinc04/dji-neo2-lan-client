import json
import pytest
from neo2_control.room_altitude import RecordedAltitude,status_altitude_observation


def event(received,available,height):
    return {'event':'telemetry_updated','elapsed_seconds':available-5.,'telemetry':{'observations':[
        {'received_at':received,'age_seconds':available-received,
         'source':{'sender':3,'receiver':14,'command_set':3,'command_id':67},
         'fields':{'altitude':{'valid':True,'unit':'m','reference':'relative','value':height}}}]}}


def timeline(tmp_path,events):
    path=tmp_path/'events.jsonl';path.write_text(''.join(json.dumps(e)+'\n' for e in events))
    return RecordedAltitude(path)


def test_join_uses_availability_not_just_receipt_and_preserves_replay_deltas(tmp_path):
    data=timeline(tmp_path,[event(10.,10.1,1.),event(10.2,10.4,1.5)])
    assert data.at(9.9) is None
    assert data.at(10.3)['value_m']==1.
    assert data.at(10.4)['value_m']==1.5
    assert data.at(10.8) is None
    replay=data.at(10.3,replayed_at=100.)
    assert replay['value_m']==1. and replay['received_at']==pytest.approx(99.7)
    assert replay['available_at']==pytest.approx(99.8)
    assert replay['original_received_at']==10.


def test_duplicate_logged_snapshots_keep_first_availability(tmp_path):
    data=timeline(tmp_path,[event(10.,10.1,1.),event(10.,10.3,1.)])
    assert len(data.rows)==1 and data.at(10.2)['available_at']==10.1


def test_mismatched_clocks_are_not_joined(tmp_path):
    first=event(10.,10.1,1.);second=event(11.,11.1,1.5);second['elapsed_seconds']+=.2
    with pytest.raises(ValueError,match='pc_clock_mismatch'):timeline(tmp_path,[first,second])


def test_only_measured_relative_fc_height_is_used(tmp_path):
    requested={'event':'scan_progress','monotonic':10.,'altitude_m':1.,'target_altitude_m':1.5}
    wrong=event(10.,10.1,1.);wrong['telemetry']['observations'][0]['source']['sender']=4
    assert not timeline(tmp_path,[requested,wrong]).rows


def test_forwarded_recipient_cannot_override_same_batch_wifi_height(tmp_path):
    wifi=event(10.,10.1,1.);forwarded=event(10.,10.1,.9)
    forwarded['telemetry']['observations'][0]['source']['receiver']=170
    assert timeline(tmp_path,[wifi,forwarded]).at(10.2)['value_m']==1.


def test_status_requires_actual_fresh_component_measurements():
    status={'now':10.,'synthetic':False,'telemetry':{
        'synthetic':False,'altitude_m':1.1,'motion_received_at':9.9}}
    assert status_altitude_observation(status)['value_m']==1.1
    status['telemetry']['motion_received_at']=9.
    assert status_altitude_observation(status) is None
    status['telemetry']['motion_received_at']=9.9;status['synthetic']=True
    assert status_altitude_observation(status) is None


def yaw_event(received,available,height,yaw):
    result=event(received,available,height)
    result['telemetry']['observations'][0]['fields']['attitude']={
        'valid':True,'unit':'degree','value':[0.,0.,yaw]}
    return result


def test_body_yaw_comes_from_the_same_past_packet_and_replay_keeps_its_clock(tmp_path):
    data=timeline(tmp_path,[yaw_event(10.,10.1,1.,20.),yaw_event(10.2,10.4,1.5,40.)])
    assert data.at(10.3)['body_yaw_deg']==20.
    result=data.at(10.3,replayed_at=100.)
    assert result['body_yaw_deg']==20. and result['received_at']==pytest.approx(99.7)
    assert result['available_at']==pytest.approx(99.8)


def test_later_duplicate_cannot_enrich_or_override_earlier_packet_yaw(tmp_path):
    data=timeline(tmp_path,[event(10.,10.1,1.),yaw_event(10.,10.3,1.,20.)])
    assert 'body_yaw_deg' not in data.at(10.2)
    data=timeline(tmp_path,[yaw_event(10.,10.1,1.,20.),yaw_event(10.,10.3,1.,30.)])
    assert data.at(10.2)['body_yaw_deg']==20.


def test_invalid_yaw_keeps_measured_height_without_a_rotation_hint(tmp_path):
    assert 'body_yaw_deg' not in timeline(tmp_path,[yaw_event(10.,10.1,1.,200.)]).at(10.2)
    status={'now':10.,'synthetic':False,'telemetry':{
        'synthetic':False,'altitude_m':1.1,'motion_received_at':9.9,'yaw_deg':40.}}
    assert status_altitude_observation(status)['body_yaw_deg']==40.
