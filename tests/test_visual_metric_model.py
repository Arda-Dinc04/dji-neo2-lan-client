"""Rendered pixels drive the full metric feedback loop; no truth-pose input.

The atlas is image-derived. The occupancy volume and rigid camera mount are
explicit fixture assumptions; this is not physical flight/clearance evidence.
"""
import json

import pytest

np = pytest.importorskip('numpy')
cv = pytest.importorskip('cv2')

from neo2_control.body_pose import RigidCameraMount
from neo2_control.metric_model import ModelVolume
from neo2_control.reconstruction import _json_file
from neo2_control.visual_metric_model import run_visual_metric_model
from test_room_coverage import room_atlas


@pytest.fixture(scope='module')
def pixel_fixture(room_atlas):
    room, _, path = room_atlas
    atlas = json.loads(path.read_text())  # The model's tracker validates lineage.
    mount = RigidCameraMount(tuple(map(tuple, room.initial_R.tolist())), (.04, 0., .025),
        'Declared exact renderer geometry; no physical Neo 2 calibration')
    ground = room.center+np.array([0., .65, 0.])
    volume = ModelVolume(origin=ground, coordinate_frame=atlas['coordinate_frame'])

    def render(position, heading):
        R_wb = cv.Rodrigues(np.array([0., np.radians(heading), 0.]))[0]
        center = np.array(position)+R_wb@np.array(mount.camera_center_in_body_m)
        image = room.folder/'feedback-current.png'
        room.render(image, heading, center)
        return image.read_bytes()
    return atlas, mount, ground, volume, render


def test_image_body_pose_feedback_completes_out_and_back_and_two_measured_half_turns(pixel_fixture, tmp_path):
    atlas, mount, ground, volume, render = pixel_fixture
    cv.setRNGSeed(31)
    result = run_visual_metric_model(atlas, render, mount, volume, ground_position=ground)
    _json_file(tmp_path/'image-feedback.json', result)
    assert result['scenario_passed'] and result['feedback']['complete']
    assert not result['model_motors_on'] and result['lost_count'] == 0
    assert result['model_outbound_length_m'] == pytest.approx(1., abs=.05)
    assert result['model_return_length_m'] == pytest.approx(1., abs=.05)
    assert result['model_return_error_m'] < .05
    assert result['maximum_position_error_m'] < .05
    events = {event['stage']: event for event in result['transitions']}
    assert events['inbound']['model_true_heading_deg']-events['outbound']['model_true_heading_deg'] == pytest.approx(180., abs=3.)
    assert events['landing']['model_true_heading_deg']-events['inbound']['model_true_heading_deg'] == pytest.approx(180., abs=3.)
    assert [a['action'] for a in result['actions']] == ['takeoff', 'land']
    assert result['observation_count'] > 200
    assert result['maximum_flow_frames'] == 4
    assert any(row['descriptor_refresh'] for row in result['observations'])
    assert any(row['tracking_method'] == 'optical_flow' for row in result['observations'])
    # Image estimates have nonzero fitting errors: model truth was not used
    # as the controller's fallback pose or as an exact metric measurement.
    assert max(row['model_position_error_m'] for row in result['observations']) > .001
    for row in result['trajectory']:
        observed = result['observations'][row['observation_index']]
        assert .15-1e-9 <= row['at']-observed['observed_at'] <= .25+1e-9
        assert not row['demand']['flight_authorized']
    assert not result['real_time_verified'] and not result['flight_authorized']


@pytest.mark.parametrize('after_takeoff', [False, True])
def test_lost_pixels_have_no_truth_pose_fallback_and_do_not_invent_a_landing(pixel_fixture, after_takeoff):
    atlas, mount, ground, volume, render = pixel_fixture
    ok, encoded = cv.imencode('.png', np.full((720, 1280), 160, np.uint8))
    assert ok
    blank = encoded.tobytes()
    calls = 0

    def source(position, heading):
        nonlocal calls
        calls += 1
        return render(position, heading) if after_takeoff and calls <= 15 else blank

    result = run_visual_metric_model(atlas, source, mount, volume, ground_position=ground)
    assert not result['scenario_passed'] and result['feedback']['stage'] == 'aborted'
    assert result['feedback']['reason'] == 'body_pose_missing'
    assert result['lost_count'] > 0 and result['observations'][-1]['body_pose'] is None
    assert result['model_motors_on'] == after_takeoff
    assert [a['action'] for a in result['actions']] == (['takeoff'] if after_takeoff else [])
    assert result['trajectory'][-1]['demand']['landing_required'] == after_takeoff
