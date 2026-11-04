"""Real ChArUco detection and metric map fits against known rendered geometry.

The fixture is a virtual room, never evidence of physical drone accuracy.
"""
import copy
import json
import subprocess
import sys

import pytest

np = pytest.importorskip('numpy')
cv = pytest.importorskip('cv2')

from neo2_control.metric_reference import (
    ReferenceBoard, _solve_reference, _validate_reference_pose, anchor_scan,
    estimate_reference_pose, load_client_scan, load_metric_scan, localize_metric_scan,
    metric_geometry, read_reference_pose, save_metric_scan, write_reference_board,
)
from neo2_control.reconstruction import (
    CameraModel, _json_file, reconstruct_pair, triangulate_correspondences,
)
from neo2_control.scan_report import write_scan_report
from neo2_control.calibration import calibrate_charuco


MODEL = CameraModel(1280, 720, 800., 800., 640., 360., (), 'calibrated',
    'Synthetic exact pinhole fixture, not a drone calibration')
BOARD = ReferenceBoard('synthetic-room-reference')
RV = np.array([.3, -.2, .06])


def render_scene(path, translation, *, texture=False, board=True, textured_background=False,
        rotation_vector=None, wide_scene=False):
    """Project independently placed, textured 3D tiles and a fixed foreground board."""
    cv.setNumThreads(2)
    canvas = np.full((720, 1280), 180, dtype=np.uint8)
    rv = RV if rotation_vector is None else np.asarray(rotation_vector, dtype=np.float64)
    rotation = cv.Rodrigues(rv)[0]
    translation = np.array(translation, dtype=np.float64)

    def place(image, vertices):
        vertices = np.asarray(vertices, dtype=np.float64)
        if ((vertices@rotation.T+translation)[:, 2] <= .05).any():
            return
        target = cv.projectPoints(vertices, rv,
            translation, MODEL.matrix(), None)[0].reshape(-1, 2).astype(np.float32)
        h, w = image.shape
        transform = cv.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]), target)
        warped = cv.warpPerspective(image, transform, (1280, 720), borderValue=180)
        mask = cv.warpPerspective(np.full_like(image, 255), transform, (1280, 720))
        canvas[mask > 254] = warped[mask > 254]

    if textured_background:
        noise = np.random.default_rng(628).integers(0, 256, (1200, 1600), dtype=np.uint8)
        texture_wall = cv.GaussianBlur(noise, (3, 3), .7)
        place(texture_wall, [[-2, -1.5, 2.], [2., -1.5, 2.], [2., 1.5, 2.], [-2, 1.5, 2.]])
    if texture:
        rng = np.random.default_rng(102)
        tiles = []
        for y in np.linspace(-.35, .45, 9):
            for x in np.linspace(-.45, 2.2 if wide_scene else .7, 29 if wide_scene else 13):
                z = rng.uniform(.12, .65)
                pattern = rng.integers(0, 2, (12, 12), dtype=np.uint8)*200+30
                pattern = cv.resize(pattern, (96, 96), interpolation=cv.INTER_NEAREST)
                tiles.append((z, pattern, [[x, y, z], [x+.065, y, z],
                    [x+.065, y+.065, z], [x, y+.065, z]]))
        for _, pattern, vertices in sorted(tiles, key=lambda item: -item[0]):
            place(pattern, vertices)
    if board:
        place(BOARD.opencv_board(.035).generateImage((980, 700)),
            [[0, 0, 0], [.245, 0, 0], [.245, .175, 0], [0, .175, 0]])
    assert cv.imwrite(str(path), canvas)
    return rotation, translation, -rotation.T@translation


def detect(path, *, model=MODEL, board=BOARD, square_mm=35):
    return estimate_reference_pose(path, model, board, measured_square_mm=square_mm,
        measurement_note='Synthetic known square size; no physical measurement')


def render_calibration_views(folder):
    image = BOARD.opencv_board(.035).generateImage((1400, 1000))
    vertices = np.array([[0., 0, 0], [.245, 0, 0], [.245, .175, 0], [0, .175, 0]])
    source = np.float32([[0, 0], [1400, 0], [1400, 1000], [0, 1000]])
    paths = []
    for i in range(12):
        angle = i*2*np.pi/12
        rv = np.array([.3*np.sin(angle), .4*np.cos(angle), .1*np.sin(2*angle)])
        R = cv.Rodrigues(rv)[0]
        t = np.array([.1*np.sin(angle), .05*np.cos(angle), .48+.05*np.sin(2*angle)])-R@np.array([.1225, .0875, 0])
        target = cv.projectPoints(vertices, rv, t, MODEL.matrix(), None)[0].reshape(-1, 2).astype(np.float32)
        frame = cv.warpPerspective(image, cv.getPerspectiveTransform(source, target),
            (1280, 720), borderValue=180)
        path = folder/f'calibration-{i}.png'
        assert cv.imwrite(str(path), frame)
        paths.append(path)
    return paths


def test_same_reference_calibrates_optics_without_any_metric_size_input(tmp_path):
    cv.setNumThreads(2)
    paths = render_calibration_views(tmp_path)
    model, report = calibrate_charuco(paths, BOARD)
    assert model.fx == pytest.approx(MODEL.fx, rel=.02)
    assert model.fy == pytest.approx(MODEL.fy, rel=.02)
    assert abs(model.cx-MODEL.cx) < 12
    assert abs(model.cy-MODEL.cy) < 12
    assert model.status == 'calibrated'
    assert report['accepted_views'] == 12
    assert report['pattern_type'] == 'charuco'
    assert report['metric_scale_verified'] is False
    with pytest.raises(ValueError, match='duplicate'):
        calibrate_charuco([paths[0]]*12, BOARD)


@pytest.fixture
def anchored_scene(tmp_path):
    paths = [tmp_path/'a.png', tmp_path/'b.png']
    expected = [render_scene(paths[0], [-.09, -.07, .65]),
        render_scene(paths[1], [-.17, -.07, .65])]
    poses = [detect(path) for path in paths]
    # Independent nonplanar 3D geometry; marker image detection supplies the
    # actual metric poses rather than injecting the expected transform.
    rng = np.random.default_rng(16)
    world = rng.uniform([-.15, -.1, .1], [.35, .25, .7], (160, 3))
    pixels = [cv.projectPoints(world, RV, t, MODEL.matrix(), None)[0].reshape(-1, 2)
        for _, t, _ in expected]
    cv.setRNGSeed(16)
    geometry = triangulate_correspondences(*pixels, MODEL)
    selected = geometry.pop('selected_indices')
    hashes = [p['image_sha256'] for p in poses]
    scan = {'format': 'neo2-control/sparse-scan/v1', 'coordinate_frame': hashes[0],
        'axes': 'first_camera_right_down_forward', 'distance_unit': 'baseline',
        'metric_scale_verified': False, 'free_space_observed': False,
        'camera_model': MODEL.as_dict(), 'image_sha256': hashes,
        'rgb': [[90, 140, 200]]*len(selected),
        'reference_descriptors': np.zeros((len(selected), 128)).tolist(), **geometry}
    return scan, poses, world[selected], expected


def test_actual_id_detection_retains_fixed_frame_and_known_camera_position(tmp_path):
    path = tmp_path/'board.png'
    _, _, expected = render_scene(path, [-.09, -.07, .65])
    pose = detect(path)
    assert np.linalg.norm(np.array(pose['camera_center'])-expected) < .003
    assert len(pose['corner_ids']) == 24
    assert pose['rms_reprojection_error_px'] < .6
    assert pose['distance_unit'] == 'm'
    assert pose['physical_accuracy_verified'] is False
    assert pose['free_space_observed'] is False
    # Reversing the camera's roll changes image coordinates but cannot
    # silently reverse a symmetric checkerboard's reference corner ordering.
    image = cv.imread(str(path))
    rolled = tmp_path/'rolled.png'
    assert cv.imwrite(str(rolled), cv.rotate(image, cv.ROTATE_180))
    again = detect(rolled)
    assert again['coordinate_frame'] == pose['coordinate_frame']
    assert np.linalg.norm(np.array(again['camera_center'])-expected) < .004
    saved = tmp_path/'pose.json'
    _json_file(saved, pose)
    assert read_reference_pose(saved) == pose


def test_metric_scale_is_from_declared_measurement_not_nominal_svg(tmp_path):
    path = tmp_path/'board.png'
    render_scene(path, [-.09, -.07, .65])
    nominal, smaller = detect(path), detect(path, square_mm=17.5)
    assert detect(path, square_mm=35.0)['coordinate_frame'] == nominal['coordinate_frame']
    assert smaller['camera_center'] == pytest.approx(np.array(nominal['camera_center'])/2, abs=1e-6)
    assert smaller['coordinate_frame'] != nominal['coordinate_frame']
    assert smaller['physical_accuracy_verified'] is False


def test_wrong_dictionary_ids_and_candidate_optics_cannot_produce_metric_pose(tmp_path):
    path = tmp_path/'board.png'
    render_scene(path, [-.09, -.07, .65])
    candidate = CameraModel.from_dict(MODEL.as_dict() | {'status': 'candidate'})
    with pytest.raises(ValueError, match='calibrated_camera'):
        detect(path, model=candidate)
    with pytest.raises(ValueError, match='not_found'):
        detect(path, board=ReferenceBoard('other', 40))
    with pytest.raises(ValueError, match='measurement'):
        detect(path, square_mm=float('nan'))


def test_collinear_small_or_ambiguous_planar_observations_are_rejected():
    native = BOARD.opencv_board(.035)
    ids = np.arange(24)
    objects = native.getChessboardCorners()
    # A very distant, slightly tilted plane has two materially different
    # poses with virtually indistinguishable image errors.
    pixels = cv.projectPoints(objects, np.array([.01, -.01, 0.]),
        np.array([-.12, -.08, 3.]), MODEL.matrix(), None)[0].reshape(-1, 2)
    with pytest.raises(ValueError, match='ambiguous|coverage'):
        _solve_reference(BOARD, MODEL, 35, ids, pixels)
    with pytest.raises(ValueError, match='corners'):
        _solve_reference(BOARD, MODEL, 35, [0]*12, pixels[:12])


def test_anchor_saves_metric_structure_and_reloads_in_new_process(tmp_path, anchored_scene):
    scan, poses, world, expected = anchored_scene
    anchored = anchor_scan(scan, *poses)
    assert anchored['metres_per_baseline'] == pytest.approx(.08, abs=.002)
    geometry = metric_geometry(anchored)
    assert np.max(np.linalg.norm(np.array(geometry['points'])-world, axis=1)) < .025
    assert geometry['distance_unit'] == 'm'
    assert geometry['first_world_to_camera_R'] == poses[0]['world_to_camera_R']
    assert geometry['second_camera_center'] == pytest.approx(expected[1][2], abs=.004)
    path, report = tmp_path/'metric-map.json', tmp_path/'metric-map.html'
    save_metric_scan(path, anchored)
    assert load_metric_scan(path) == anchored == load_client_scan(path)
    assert path.stat().st_mode & 0o777 == 0o600
    with pytest.raises(FileExistsError):
        save_metric_scan(path, anchored)
    result = subprocess.run([sys.executable, '-m', 'neo2_control.cli', 'scan-report',
        str(path), '--output', str(report)], capture_output=True, text=True, check=True)
    assert json.loads(result.stdout)['distance_unit'] == 'm'
    assert 'declared_measured_charuco_square' in report.read_text()
    assert 'data.distance_unit' in report.read_text()


@pytest.mark.parametrize('mutation', ['image', 'reference', 'optics', 'reversed'])
def test_anchor_requires_same_board_and_exact_same_image_sources(anchored_scene, mutation):
    scan, poses, _, _ = anchored_scene
    scan, poses = copy.deepcopy(scan), copy.deepcopy(poses)
    if mutation == 'image':
        scan['image_sha256'][1] = 'a'*64
    elif mutation == 'reference':
        poses[1]['reference']['board']['reference_id'] = 'moved-or-different-reference'
    elif mutation == 'optics':
        scan['camera_model']['fx'] += 10
    else:
        poses.reverse()
    with pytest.raises(ValueError):
        anchor_scan(scan, *poses)


def test_independently_fitted_reference_cannot_anchor_contradictory_motion(anchored_scene):
    scan, poses, _, _ = anchored_scene
    inconsistent = copy.deepcopy(scan)
    t = -np.array(inconsistent['second_world_to_camera_t'])
    R = np.array(inconsistent['second_world_to_camera_R'])
    inconsistent['second_world_to_camera_t'] = t.tolist()
    inconsistent['second_camera_center'] = (-R.T@t).tolist()
    with pytest.raises(ValueError, match='geometry_inconsistent'):
        anchor_scan(inconsistent, *poses)
    with pytest.raises(ValueError, match='source_mismatch'):
        anchor_scan(scan, poses[0], poses[0])


@pytest.mark.parametrize('field,value', [
    ('metres_per_baseline', 10), ('distance_unit', 'cm'),
    ('free_space_observed', True), ('coordinate_frame', 'a'*64),
    ('physical_accuracy_verified', True),
])
def test_metric_memory_cannot_upgrade_scale_frame_or_observation_claims(tmp_path, anchored_scene, field, value):
    scan, poses, _, _ = anchored_scene
    anchored = anchor_scan(scan, *poses)
    anchored[field] = value
    path = tmp_path/'invalid.json'
    path.write_text(json.dumps(anchored))
    with pytest.raises(ValueError, match='metric_scan_invalid'):
        load_metric_scan(path)


def test_scene_features_relocalize_after_reference_board_leaves_view(tmp_path):
    first, second, third = (tmp_path/f'frame-{i}.png' for i in range(3))
    render_scene(first, [-.09, -.07, .65], texture=True)
    render_scene(second, [-.17, -.07, .65], texture=True)
    _, _, center = render_scene(third, [-.19, -.07, .65], texture=True, board=False)
    cv.setRNGSeed(32)
    scan = reconstruct_pair(first, second, MODEL)
    # The two rendered cameras have identical orientations. Consensus
    # refitting must not retain the biased minimal RANSAC pose (~0.23 deg).
    angle = np.degrees(np.linalg.norm(cv.Rodrigues(np.array(scan['second_world_to_camera_R']))[0]))
    assert angle < .03
    anchored = anchor_scan(scan, detect(first), detect(second))
    path = tmp_path/'memory.json'
    save_metric_scan(path, anchored)
    pose = localize_metric_scan(load_metric_scan(path), third)
    assert pose['coordinate_frame'] == anchored['coordinate_frame']
    assert pose['distance_unit'] == 'm'
    assert np.linalg.norm(np.array(pose['camera_center'])-center) < .025
    assert pose['inliers'] >= 20
    assert pose['physical_accuracy_verified'] is False


def test_svg_pattern_agrees_with_detector_after_actual_svg_rendering(tmp_path):
    import shutil
    if not shutil.which('rsvg-convert'):
        pytest.skip('SVG renderer unavailable')
    svg, png = tmp_path/'board.svg', tmp_path/'board.png'
    write_reference_board(BOARD, svg)
    with pytest.raises(FileExistsError):
        write_reference_board(BOARD, svg)
    subprocess.run(['rsvg-convert', '--width', '1188', '--height', '840',
        '--output', str(png), str(svg)], check=True)
    frame = cv.imread(str(png))
    corners, ids, _, _ = cv.aruco.CharucoDetector(BOARD.opencv_board(.035)).detectBoard(frame)
    assert len(ids) == 24
    actual = np.array([p for _, p in sorted(zip(ids.ravel(), corners.reshape(-1, 2)))])
    expected = BOARD.opencv_board(.035).getChessboardCorners()[:, :2]*4000+np.array([104, 70])
    assert np.max(np.linalg.norm(actual-expected, axis=1)) < 1.5


def test_fitted_pose_cannot_be_replaced_by_an_unrelated_transform(tmp_path):
    path = tmp_path/'frame.png'
    render_scene(path, [-.09, -.07, .65])
    pose = detect(path)
    pose['camera_center'][0] += 1
    with pytest.raises(ValueError, match='reference_pose_invalid'):
        _validate_reference_pose(pose)
