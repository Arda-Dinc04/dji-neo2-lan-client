"""Rigid lever-arm correction and refusal of ambiguous camera geometry."""
import copy
from dataclasses import replace

import pytest

np = pytest.importorskip('numpy')
cv = pytest.importorskip('cv2')

from neo2_control.body_pose import CameraBodyProjector, RigidCameraMount


FRAME, MAP = 'a'*64, 'b'*64
MOUNT_R = cv.Rodrigues(np.array([.3, -.2, .06]))[0]
MOUNT = RigidCameraMount(tuple(map(tuple, MOUNT_R.tolist())), (.08, -.02, .04), 'Declared model mount only')


def camera_pose(heading=0., center=(1., .2, -.5), *, body_rotation=None):
    R_wb = cv.Rodrigues(np.array([0., np.radians(heading), 0.]))[0] if body_rotation is None else body_rotation
    R_cw = MOUNT_R@R_wb.T
    C = np.array(center)+R_wb@np.array(MOUNT.camera_center_in_body_m)
    return {'coordinate_frame': FRAME, 'map_sha256': MAP, 'image_sha256': 'c'*64,
        'distance_unit': 'm', 'world_to_camera_R': R_cw.tolist(),
        'world_to_camera_t': (-R_cw@C).tolist(), 'camera_center': C.tolist(),
        'inliers': 50, 'maximum_reprojection_error_px': .7}


def project(value, **kwargs):
    return CameraBodyProjector(FRAME, MAP, MOUNT).project(value, observed_at=100.,
        flying=True, motors_on=True, owner_epoch='d'*48, generation='e'*24, synthetic=True, **kwargs)


@pytest.mark.parametrize('heading', [0., 60., 90., 179., 180., 240., 270., 360., 420.])
def test_rotating_offset_camera_yields_stationary_body_center_and_correct_heading(heading):
    camera = camera_pose(heading)
    result = project(camera)
    assert result.body.position_m == pytest.approx((1., .2, -.5), abs=1e-12)
    assert result.body.heading_deg == pytest.approx((heading+180)%360-180)
    assert result.body.observed_at == 100. and result.body.coordinate_frame == FRAME
    assert np.linalg.norm(np.array(camera['camera_center'])-result.body.position_m) > .08
    assert not result.evidence['flight_authorized'] and not result.evidence['physical_accuracy_verified']


def test_mount_round_trip_preserves_explicit_scope_and_rejects_calibration_claim():
    value = MOUNT.as_dict()
    assert RigidCameraMount.from_dict(value) == MOUNT
    value['physical_accuracy_verified'] = True
    with pytest.raises(ValueError, match='camera_mount_invalid'):
        RigidCameraMount.from_dict(value)


@pytest.mark.parametrize('kind', ['reflection', 'skew', 'nonfinite', 'large_offset', 'empty_note'])
def test_invalid_mount_is_not_a_rigid_body_calibration(kind):
    changes = {}
    if kind == 'reflection':
        changes['body_to_camera_R'] = ((-1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
    elif kind == 'skew':
        changes['body_to_camera_R'] = ((1., .2, 0.), (0., 1., 0.), (0., 0., 1.))
    elif kind == 'nonfinite':
        changes['camera_center_in_body_m'] = (float('nan'), 0., 0.)
    elif kind == 'large_offset':
        changes['camera_center_in_body_m'] = (1., 0., 0.)
    else:
        changes['note'] = ''
    with pytest.raises(ValueError):
        replace(MOUNT, **changes)


@pytest.mark.parametrize('key,value', [
    ('coordinate_frame', 'f'*64), ('map_sha256', 'f'*64), ('distance_unit', 'baseline'),
    ('image_sha256', None), ('inliers', 19), ('maximum_reprojection_error_px', 2.51),
    ('maximum_reprojection_error_px', float('nan')), ('camera_center', [0., 0., 0.]),
    ('world_to_camera_t', [0., 0., 0.]),
])
def test_projector_requires_one_metric_map_and_consistent_supported_geometry(key, value):
    camera = camera_pose()
    camera[key] = value
    with pytest.raises(ValueError):
        project(camera)


def test_body_tilt_is_measured_after_removing_camera_mount_orientation():
    assert project(camera_pose()).evidence['body_tilt_deg'] == pytest.approx(0.)
    rotation = cv.Rodrigues(np.array([np.radians(35.), 0., 0.]))[0]
    with pytest.raises(ValueError, match='body_tilt_outside_model'):
        project(camera_pose(body_rotation=rotation))


def test_input_camera_dictionary_is_not_modified():
    camera = camera_pose(90.)
    original = copy.deepcopy(camera)
    project(camera)
    assert camera == original


def test_board_axes_need_not_be_aligned_with_declared_gravity():
    G = cv.Rodrigues(np.array([.4, .1, -.2]))[0]
    R_wb = G@cv.Rodrigues(np.array([0., np.radians(120.), 0.]))[0]
    value = camera_pose(body_rotation=R_wb)
    projector = CameraBodyProjector(FRAME, MAP, MOUNT,
        zero_forward=(G@np.array([0., 0., 1.])).tolist(), up=(G@np.array([0., -1., 0.])).tolist())
    result = projector.project(value, observed_at=100., flying=True, motors_on=True,
        owner_epoch='d'*48, generation='e'*24, synthetic=True)
    assert result.body.position_m == pytest.approx((1., .2, -.5))
    assert result.body.heading_deg == pytest.approx(120.)
    assert result.evidence['body_tilt_deg'] == pytest.approx(0., abs=1e-5)
