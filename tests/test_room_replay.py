import math
import numpy as np
import pytest
from neo2_control.room_replay import unwrap_degrees,axis_rotation


def test_unwrap_records_two_turns_across_signed_heading_boundary():
    physical=list(range(170,170+721,30))
    wrapped=[(v+180)%360-180 for v in physical]
    actual=unwrap_degrees(wrapped)
    assert actual==pytest.approx(physical)
    assert actual[-1]-actual[0]==720


def test_axis_rotation_preserves_geometry_and_has_the_expected_direction():
    rotation=axis_rotation([0,3,0],90)
    assert rotation@np.array([0,0,1])==pytest.approx([1,0,0])
    assert rotation@rotation.T==pytest.approx(np.eye(3))
    assert np.linalg.det(rotation)==pytest.approx(1)
    assert axis_rotation([1,2,3],360)==pytest.approx(np.eye(3))


@pytest.mark.parametrize('values',[[float('nan')],[True],[math.inf]])
def test_nonfinite_or_boolean_heading_is_rejected(values):
    with pytest.raises(ValueError):unwrap_degrees(values)
