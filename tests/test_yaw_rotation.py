import math

import pytest

from neo2_control.yaw_rotation import YawRotation


GENERATION='a'*24


@pytest.mark.parametrize('direction',[1,-1])
@pytest.mark.parametrize('start',[-179.,0.,179.])
def test_one_full_measured_turn_across_signed_wrap(direction,start):
    rotation=YawRotation(GENERATION)
    for step in range(46):
        now=100.+step*.1
        yaw=(start+direction*step*8+180)%360-180
        rotation.observe(yaw,now,now+.01,generation=GENERATION)
        assert rotation.complete is (step==45)
        assert rotation.channels()[:3]==(0,0,0)
        assert 0<=rotation.channels()[3]<=200
    assert rotation.progress_degrees==360 and rotation.direction==direction
    assert rotation.channels()==(0,0,0,0)


def test_repeated_packets_do_not_add_a_turn_or_renew_source_time():
    rotation=YawRotation(GENERATION)
    rotation.observe(10.,100.,100.,generation=GENERATION)
    for elapsed in (.1,.2,.3,.49):rotation.observe(10.,100.,100.+elapsed,generation=GENERATION)
    assert rotation.samples==1 and rotation.progress_degrees==0
    with pytest.raises(ValueError,match='stale'):
        rotation.observe(10.,100.,100.501,generation=GENERATION)


def test_two_half_turns_in_opposite_directions_do_not_count_as_360():
    rotation=YawRotation(GENERATION)
    for i in range(19):rotation.observe(i*10.,100+i*.1,100+i*.1,generation=GENERATION)
    rotation.observe(170.,102.,102.,generation=GENERATION)
    assert rotation.progress_degrees==170 and not rotation.complete
    with pytest.raises(ValueError,match='reversed'):
        rotation.observe(160.,102.1,102.1,generation=GENERATION)


@pytest.mark.parametrize('value',[None,True,math.nan,math.inf,-180.1,180.1])
def test_invalid_yaw_cannot_authorize_rotation(value):
    with pytest.raises(ValueError,match='invalid'):
        YawRotation(GENERATION).observe(value,100.,100.,generation=GENERATION)


def test_jump_old_sample_and_connection_replacement_abort_the_measurement():
    rotation=YawRotation(GENERATION)
    rotation.observe(0.,100.,100.,generation=GENERATION)
    for yaw,received,now,generation,reason in (
        (0.,99.9,100.,GENERATION,'reordered'),
        (1.,100.,100.,GENERATION,'ambiguous'),
        (100.,100.1,100.1,GENERATION,'discontinuity'),
        (0.,100.6,100.6,GENERATION,'gap'),
        (0.,100.1,100.1,'b'*24,'generation_changed'),
    ):
        with pytest.raises(ValueError,match=reason):rotation.observe(yaw,received,now,generation=generation)


def test_yaw_command_slows_near_target_without_commanding_translation():
    rotation=YawRotation(GENERATION)
    for i in range(36):
        yaw=(i*10.+180)%360-180
        rotation.observe(yaw,100.+i*.1,100.+i*.1,generation=GENERATION)
    assert rotation.progress_degrees==350 and rotation.channels()==(0,0,0,40)


def test_explicit_floor_keeps_constant_amplitude_until_observed_full_turn():
    rotation=YawRotation(GENERATION)
    for step in range(46):
        now=100+step*.1;yaw=(step*8+180)%360-180
        rotation.observe(yaw,now,now,generation=GENERATION)
        assert rotation.channels(600,minimum_channel=600)==((0,0,0,600) if step<45 else (0,0,0,0))
    assert rotation.complete and rotation.progress_degrees==360
