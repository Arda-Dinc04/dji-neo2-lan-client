"""Known rendered depths, occlusions, vertical baselines and PC persistence."""
import copy
import json

import pytest

np = pytest.importorskip('numpy')
cv = pytest.importorskip('cv2')

from neo2_control.depth_scan import (
    _stereo_measurements, densify_scan, depth_geometry, load_depth_scan, save_depth_scan,
)
from neo2_control.metric_reference import anchor_scan
from neo2_control.reconstruction import CameraModel, reconstruct_pair
from neo2_control.scan_report import write_scan_report
from test_metric_reference import MODEL as ROOM_MODEL, detect, render_scene


MODEL = CameraModel(640, 480, 400., 400., 320., 240., (), 'calibrated',
    'Exact synthetic camera, not drone calibration')


def stereo_scene(axis, sign=1, *, flat=False):
    """Two textured fronto-parallel surfaces with analytic 24/48 px disparity."""
    cv.setNumThreads(2)
    rng = np.random.default_rng(84)
    h, w = MODEL.height, MODEL.width
    shape = (h+192, w) if axis else (h, w+192)
    background = rng.integers(0, 256, shape, dtype=np.uint8)
    if flat:
        background[:] = 150
    if axis:
        first = background[96:96+h].copy()
        second = background[96+sign*24:96+sign*24+h].copy()
    else:
        first = background[:, 96:96+w].copy()
        second = background[:, 96+sign*24:96+sign*24+w].copy()
    x, y, width, height = 320, 224 if axis else 180, 100, 40 if axis else 120
    patch = rng.integers(0, 256, (height, width), dtype=np.uint8)
    if flat:
        patch[:] = 150
    first[y:y+height, x:x+width] = patch
    x1, y1 = x-sign*48*(1-axis), y-sign*48*axis
    second[y1:y1+height, x1:x1+width] = patch
    truth = np.full((h, w), 2.)
    truth[y:y+height, x:x+width] = 1.
    translation = np.zeros(3)
    translation[axis] = -sign*.12
    return [cv.cvtColor(frame, cv.COLOR_GRAY2BGR) for frame in (first, second)], truth, translation


@pytest.mark.parametrize('axis,sign', [(0, 1), (0, -1), (1, 1), (1, -1)])
def test_dense_depth_agrees_with_two_known_surfaces_for_both_axes_and_signs(axis, sign):
    frames, truth, t = stereo_scene(axis, sign)
    result = _stereo_measurements(*frames, MODEL, np.eye(3), t)
    valid, depth = result['valid'], result['depth']
    for z in (1., 2.):
        selected = valid & (truth == z)
        assert selected.sum() > 1000
        assert np.median(depth[selected]) == pytest.approx(z, rel=.005)
        assert np.percentile(np.abs(depth[selected]-z), 95) < .025
    assert result['disparity_axis'] == ('vertical' if axis else 'horizontal')
    assert np.isfinite(result['sensitivity'][valid]).all()
    assert result['sensitivity'][valid & (truth == 2.)].mean() > result['sensitivity'][valid & (truth == 1.)].mean()*3.5


def test_occluded_pixels_and_untextured_areas_remain_unknown():
    frames, _, t = stereo_scene(0, 1)
    result = _stereo_measurements(*frames, MODEL, np.eye(3), t)
    # Left background at x=296..319 projects behind the right foreground.
    assert result['valid'][190:290, 298:316].mean() < .05
    assert result['valid'][190:290, 327:400].mean() > .8
    frames, _, t = stereo_scene(0, 1, flat=True)
    result = _stereo_measurements(*frames, MODEL, np.eye(3), t)
    assert not result['valid'].any()


@pytest.mark.parametrize('translation', [[0., 0., 0.], [.001, 0., 0.], [float('nan'), 0., 0.]])
def test_depth_needs_an_actual_finite_translational_baseline(translation):
    frames, _, _ = stereo_scene(0)
    with pytest.raises(ValueError, match='relative_pose'):
        _stereo_measurements(*frames, MODEL, np.eye(3), translation)


@pytest.fixture(scope='module')
def depth_scene(tmp_path_factory):
    folder = tmp_path_factory.mktemp('depth-scene')
    first, second = folder/'first.png', folder/'second.png'
    render_scene(first, [-.09, -.07, .65], texture=True)
    render_scene(second, [-.17, -.07, .65], texture=True)
    cv.setRNGSeed(32)
    scan = reconstruct_pair(first, second, ROOM_MODEL)
    metric = anchor_scan(scan, detect(first), detect(second))
    dense = densify_scan(metric, first, second)
    return dense, metric, first, second


def test_real_matching_pipeline_retains_metric_frame_and_persists_unknown_mask(tmp_path, depth_scene):
    dense, metric, _, _ = depth_scene
    assert dense['coordinate_frame'] == metric['coordinate_frame']
    assert dense['distance_unit'] == 'm' and dense['accepted_sample_count'] > 500
    assert dense['accepted_sample_count'] < dense['grid_width']*dense['grid_height']
    path = tmp_path/'depth.json'
    save_depth_scan(path, dense)
    restored = load_depth_scan(path)
    assert restored == dense
    assert path.stat().st_mode & 0o777 == 0o600
    with pytest.raises(FileExistsError):
        save_depth_scan(path, dense)
    geometry = depth_geometry(restored)
    assert len(geometry['points']) == dense['accepted_sample_count']
    # The board surface occupies z=0 in the independent world reference.
    points = np.array(geometry['points'])
    central = points[(points[:, 0] > .03) & (points[:, 0] < .20)
        & (points[:, 1] > .04) & (points[:, 1] < .14) & (points[:, 2] < .1)]
    assert len(central) > 40
    assert abs(np.median(central[:, 2])) < .025
    assert geometry['physical_accuracy_verified'] is False
    assert geometry['free_space_observed'] is False


def test_dense_acquisition_cannot_use_different_images_for_the_saved_pose(depth_scene):
    _, metric, first, second = depth_scene
    with pytest.raises(ValueError, match='source_images'):
        densify_scan(metric, second, first)


def test_no_admitted_depth_is_retained_as_unknown_not_free(tmp_path, depth_scene):
    scan = copy.deepcopy(depth_scene[0])
    for key in ('depth_m', 'one_pixel_depth_sensitivity_m'):
        scan[key] = [[None]*scan['grid_width'] for _ in range(scan['grid_height'])]
    scan['accepted_sample_count'] = 0
    scan['full_resolution_valid_fraction'] = 0.
    path = tmp_path/'empty-depth.json'
    save_depth_scan(path, scan)
    restored = load_depth_scan(path)
    assert depth_geometry(restored)['points'] == []
    assert restored['free_space_observed'] is False
    result = write_scan_report(restored, tmp_path/'unknown.html')
    assert result['point_count'] == 0
    assert result['free_space_observed'] is False


@pytest.mark.parametrize('field,value', [
    ('distance_unit', 'cm'), ('free_space_observed', True),
    ('physical_accuracy_verified', True), ('acquisition_synchronization_verified', True),
    ('accepted_sample_count', 0), ('stride', 2), ('metres_per_baseline', 2.),
    ('coordinate_frame', 'not-a-frame'),
])
def test_saved_depth_cannot_upgrade_evidence_or_change_shape_and_units(tmp_path, depth_scene, field, value):
    scan = copy.deepcopy(depth_scene[0])
    scan[field] = value
    path = tmp_path/'invalid.json'
    path.write_text(json.dumps(scan))
    with pytest.raises(ValueError, match='depth_scan_invalid'):
        load_depth_scan(path)


def test_unsupported_camera_or_depth_range_does_not_invent_measurements():
    frames, _, t = stereo_scene(0)
    candidate = CameraModel.from_dict(MODEL.as_dict() | {'status': 'candidate'})
    with pytest.raises(ValueError, match='model_or_range'):
        _stereo_measurements(*frames, candidate, np.eye(3), t)
    with pytest.raises(ValueError, match='model_or_range'):
        _stereo_measurements(*frames, MODEL, np.eye(3), t, minimum_m=2., maximum_m=1.)
