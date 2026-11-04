"""Source-bound optics pushes remain passive, raw and separate from calibration."""
from dataclasses import replace
import hashlib
import struct

import pytest

from neo2_control.camera import CameraChannel,decode_camera_fov
from neo2_control.duml import DumlPacket,DumlSequence


BODY=bytes.fromhex('00190000190e000001000000ea0b0000')
PUSH=DumlPacket(1,2,59783,0,2,0x8a,BODY)


def test_actual_payload_and_signed_limits_do_not_become_a_preview_matrix():
    fields=decode_camera_fov(BODY)
    assert fields['sdk_fields_raw']==dict(width=6400,height=3609,imageRatio=1,lensFocalLength=3050)
    assert fields['payload_hex']==BODY.hex()
    assert fields['payload_sha256']==hashlib.sha256(BODY).hexdigest()
    assert fields['units_verified'] is fields['preview_projection_verified'] is False
    extremes=decode_camera_fov(struct.pack('<iiii',-2**31,2**31-1,-1,0))
    assert list(extremes['sdk_fields_raw'].values())==[-2**31,2**31-1,-1,0]


@pytest.mark.parametrize('payload',[None,bytearray(BODY),BODY[:-1],BODY+b'\0'])
def test_unknown_payload_layout_is_rejected(payload):
    with pytest.raises(ValueError,match='requires_16_bytes'):decode_camera_fov(payload)


@pytest.mark.parametrize('change',[
    {'version':2},{'sender':2},{'receiver':3},{'cmd_type':0x40},
    {'cmd_type':0x80},{'cmd_set':4},{'cmd_id':0x80},{'payload':BODY[:-1]},
])
def test_only_the_observed_push_route_is_admitted(change):
    channel=CameraChannel(clock=lambda:1.)
    channel.receive(replace(PUSH,**change),1.)
    assert channel.snapshot()['fov_status'] is None
    assert channel.snapshot()['fov_observations']==[]


def test_both_routes_age_independently_and_never_make_the_channel_ready_or_emit():
    now=[1.]
    channel=CameraChannel(clock=lambda:now[0])
    channel.receive(PUSH,1.)
    snap=channel.snapshot()
    assert not snap['ready'] and not snap['fov_status']['fresh']
    assert channel.next_packet(1.,DumlSequence().next) is None
    assert snap['sent_count']==0 and snap['camera_status'] is None
    channel.set_ready(True,1.)
    now[0]=1.9
    channel.receive(replace(PUSH,receiver=0x1c,seq=59784),1.9)
    channel.set_ready(True,1.9)
    snap=channel.snapshot()
    assert [r['fresh'] for r in snap['fov_observations']]==[True,True]
    assert snap['fov_status']['source']==dict(version=1,sender=1,receiver=0x1c,
        sequence=59784,cmd_type=0,cmd_set=2,cmd_id=0x8a)
    snap['fov_status']['sdk_fields_raw']['width']=1
    now[0]=2.1
    snap=channel.snapshot()
    assert [r['fresh'] for r in snap['fov_observations']]==[False,True]
    assert snap['fov_status']['sdk_fields_raw']['width']==6400
    assert snap['fov_status']['revision']==1
    assert channel.last_time==1.9
    channel.revoke()
    assert not any(r['fresh'] for r in channel.snapshot()['fov_observations'])
    assert channel.snapshot()['sent_count']==0


def test_optical_push_does_not_acknowledge_a_pending_camera_command():
    channel=CameraChannel(clock=lambda:1.)
    channel.set_ready(True,1.)
    channel.submit('stop_record',{})
    request=channel.next_packet(1.,DumlSequence().next)
    channel.sent(request.seq,1.)
    channel.receive(replace(PUSH,seq=request.seq),1.)
    snap=channel.snapshot()
    assert snap['last_command']['state']=='sent' and snap['pending']
    assert snap['fov_status']['payload_hex']==BODY.hex()


def test_observed_change_and_return_to_same_metadata_remain_detectable_between_polls():
    channel=CameraChannel(clock=lambda:1.)
    channel.receive(PUSH,1.)
    assert channel.snapshot()['fov_status']['revision']==1
    changed=struct.pack('<iiii',6400,3609,2,3050)
    channel.receive(replace(PUSH,payload=changed),1.)
    channel.receive(PUSH,1.)
    snap=channel.snapshot()
    assert snap['fov_status']['payload_hex']==BODY.hex()
    assert snap['fov_status']['revision']==3
    assert channel.snapshot()['sent_count']==0
