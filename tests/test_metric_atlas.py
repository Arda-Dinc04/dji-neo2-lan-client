"""Incremental, image-derived metric landmarks across a turn, never a flight."""
import copy
import json
import subprocess
import sys

import pytest

np = pytest.importorskip('numpy')
cv = pytest.importorskip('cv2')

from neo2_control.metric_atlas import (
    _assemble, _triangulate, create_atlas, extend_atlas, load_atlas, localize_atlas, save_atlas,
)
from neo2_control.metric_reference import anchor_scan, localize_metric_scan
from neo2_control.reconstruction import reconstruct_pair
from test_metric_reference import MODEL, RV, detect, render_scene


@pytest.fixture(scope='module')
def growing_scene(tmp_path_factory):
    folder = tmp_path_factory.mktemp('atlas-room')
    R0 = cv.Rodrigues(RV)[0]
    centers = [-R0.T@np.array(t) for t in ([-.09, -.07, .65], [-.17, -.07, .65])]
    paths = []
    for i, center in enumerate(centers):
        path = folder/f'seed-{i}.png'
        render_scene(path, -R0@center, texture=True, wide_scene=True)
        paths.append(path)
    cv.setRNGSeed(32)
    seed = anchor_scan(reconstruct_pair(*paths, MODEL), *[detect(p) for p in paths])
    atlas = create_atlas(seed)
    addition = []
    for i, center in enumerate(centers):
        R = cv.Rodrigues(np.array([0., -np.radians(25), 0.]))[0]@R0
        path = folder/f'added-{i}.png'
        render_scene(path, -R@center, texture=True, wide_scene=True,
            rotation_vector=cv.Rodrigues(R)[0])
        addition.append(path)
    grown = extend_atlas(atlas, *addition)
    R = cv.Rodrigues(np.array([0., -np.radians(80), 0.]))[0]@R0
    query = folder/'query-80.png'
    render_scene(query, -R@centers[0], texture=True, wide_scene=True, board=False,
        rotation_vector=cv.Rodrigues(R)[0])
    return seed, atlas, grown, paths, addition, query, centers[0]


def test_known_pose_triangulation_preserves_metres_instead_of_normalizing_the_baseline():
    rng = np.random.default_rng(61)
    points = rng.uniform([-.8, -.5, 3.], [.8, .5, 6.], (80, 3))
    rotations = [np.eye(3), cv.Rodrigues(np.array([.01, -.02, .01]))[0]]
    centers = [np.array([.1, .1, 0.]), np.array([.3, .1, 0.])]
    poses = [{'world_to_camera_R': R.tolist(), 'world_to_camera_t': (-R@c).tolist()}
        for R, c in zip(rotations, centers)]
    pixels = [cv.projectPoints(points, cv.Rodrigues(R)[0], -R@c,
        MODEL.matrix(), None)[0].reshape(-1, 2) for R, c in zip(rotations, centers)]
    recovered, keep, error, _, baseline = _triangulate(*pixels, MODEL, poses)
    assert keep.all() and error.max() < 1e-5
    assert baseline == pytest.approx(.2)
    assert recovered == pytest.approx(points, abs=1e-5)
    same_center = copy.deepcopy(poses)
    same_center[1]['world_to_camera_t'] = (-rotations[1]@centers[0]).tolist()
    with pytest.raises(ValueError, match='translational_baseline'):
        _triangulate(*pixels, MODEL, same_center)


def test_growth_keeps_seed_points_and_adds_pose_supported_scene_landmarks(growing_scene):
    seed, atlas, grown, _, _, _, _ = growing_scene
    before = _assemble(atlas)[1]
    after = _assemble(grown)[1]
    assert len(after) > len(before)+100
    assert np.array_equal(after[:len(before)], before)
    assert grown['seed'] == seed == atlas['seed']
    assert atlas['extensions'] == []
    assert grown['coordinate_frame'] == atlas['coordinate_frame']
    assert grown['distance_unit'] == 'm'
    assert grown['physical_accuracy_verified'] is False
    assert grown['free_space_observed'] is False


def test_new_view_can_use_the_grown_map_after_the_seed_loses_localization(growing_scene):
    seed, _, grown, _, _, query, center = growing_scene
    with pytest.raises(ValueError):
        localize_metric_scan(seed, query)
    pose = localize_atlas(grown, query)
    assert pose['distance_unit'] == 'm'
    assert pose['coordinate_frame'] == grown['coordinate_frame']
    assert pose['inliers'] >= 20
    assert pose['added_landmark_inliers'] >= 20
    assert np.linalg.norm(np.array(pose['camera_center'])-center) < .025
    assert pose['tracking_accuracy_verified'] is False


def test_saved_growth_reloads_and_localizes_in_a_new_process(tmp_path, growing_scene):
    _, _, grown, _, _, query, center = growing_scene
    path, output = tmp_path/'atlas.json', tmp_path/'pose.json'
    saved = save_atlas(path, grown)
    assert load_atlas(path) == grown and saved['extension_count'] == 1
    assert path.stat().st_mode & 0o777 == 0o600
    with pytest.raises(FileExistsError):
        save_atlas(path, grown)
    result = subprocess.run([sys.executable, '-m', 'neo2_control.cli', 'scan-localize',
        str(path), str(query), '--output', str(output)], capture_output=True, text=True, check=True)
    assert np.linalg.norm(np.array(json.loads(result.stdout)['camera_center'])-center) < .025
    assert json.loads(output.read_text())['coordinate_frame'] == grown['coordinate_frame']


@pytest.mark.parametrize('mutation', ['units', 'frame', 'claim', 'point', 'pose', 'lineage', 'pixels'])
def test_persisted_extension_cannot_change_its_frame_units_or_pose_support(tmp_path, growing_scene, mutation):
    value = copy.deepcopy(growing_scene[2])
    entry = value['extensions'][0]
    if mutation == 'units': value['distance_unit'] = 'baseline'
    elif mutation == 'frame': value['coordinate_frame'] = 'a'*64
    elif mutation == 'claim': value['physical_accuracy_verified'] = True
    elif mutation == 'point': entry['points_m'][0][0] += .1
    elif mutation == 'pose': entry['poses'][0]['world_to_camera_t'][0] += .1
    elif mutation == 'lineage': entry['poses'][0]['landmark_indices'][0] = entry['parent_point_count']
    else: entry['first_pixels'][0][0] += 10
    path = tmp_path/'invalid.json'
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match='metric_atlas_invalid'):
        load_atlas(path)


def test_previously_used_pairs_cannot_fabricate_additional_landmarks(growing_scene):
    _, atlas, grown, paths, addition, _, _ = growing_scene
    with pytest.raises(ValueError, match='new_images'):
        extend_atlas(atlas, *paths)
    with pytest.raises(ValueError, match='new_images'):
        extend_atlas(grown, *addition)


def test_rotation_without_translation_is_not_new_metric_depth(tmp_path, growing_scene):
    _, atlas, _, _, addition, _, center = growing_scene
    R = cv.Rodrigues(np.array([0., -np.radians(30), 0.]))[0]@cv.Rodrigues(RV)[0]
    rotated = tmp_path/'rotation-only.png'
    render_scene(rotated, -R@center, texture=True, wide_scene=True, rotation_vector=cv.Rodrigues(R)[0])
    with pytest.raises(ValueError, match='parallax|baseline'):
        extend_atlas(atlas, addition[0], rotated)


@pytest.fixture(scope='module')
def atlas_depth(growing_scene):
    from neo2_control.metric_atlas import densify_atlas
    grown, addition = growing_scene[2], growing_scene[4]
    return densify_atlas(grown, 0, *addition)


def test_new_view_depth_keeps_the_same_frame_and_can_feed_volume_memory(tmp_path, growing_scene, atlas_depth):
    from neo2_control.depth_scan import save_depth_scan, load_depth_scan
    from neo2_control.spatial_memory import build_memory
    assert atlas_depth['format'] == 'neo2-control/depth-scan/v2'
    assert atlas_depth['coordinate_frame'] == growing_scene[2]['coordinate_frame']
    assert atlas_depth['accepted_sample_count'] > 100
    assert atlas_depth['source_extension_index'] == 0
    path = tmp_path/'atlas-depth.json'
    save_depth_scan(path, atlas_depth)
    assert load_depth_scan(path) == atlas_depth
    memory = build_memory([load_depth_scan(path)], lower=[-.5, -.5, -.5], upper=[1.5, 1., 1.5])
    assert memory['coordinate_frame'] == atlas_depth['coordinate_frame']
    assert memory['counts']['surface_or_margin'] > 0
    assert memory['free_space_verified'] is False


@pytest.mark.parametrize('mutation', ['index', 'pose', 'parent'])
def test_depth_cannot_detach_its_pose_from_the_atlas_extension(tmp_path, atlas_depth, mutation):
    from neo2_control.depth_scan import load_depth_scan
    value = copy.deepcopy(atlas_depth)
    if mutation == 'index': value['source_extension_index'] = 1
    elif mutation == 'pose': value['relative_t_m'][0] += .02
    else: value['source_metric_atlas_sha256'] = 'a'*64
    path = tmp_path/'changed-depth.json'
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match='depth_scan_invalid'):
        load_depth_scan(path)
