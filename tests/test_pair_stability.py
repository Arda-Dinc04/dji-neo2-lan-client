"""Conditional two-view sensitivity with real geometry and explicit failures."""
import json

import pytest

cv=pytest.importorskip('cv2')
np=pytest.importorskip('numpy')

from neo2_control import cli,pair_stability,reconstruction
from neo2_control.pair_stability import assess_pair,PairStabilityError
from test_reconstruction import scene,MODEL


def test_well_separated_geometry_is_repeatably_stable_and_does_not_gain_authority():
    _,a,b,_,_=scene()
    first=a.copy();second=b.copy()
    report=assess_pair(a,b,MODEL,trials=8,seed=37)
    assert report['passed'] and report['status']=='stable'
    assert report['valid_trials']>=7
    assert report['translation_p95_degrees']<=5.
    assert report['rotation_p95_degrees']<=2.
    assert report==assess_pair(a,b,MODEL,trials=8,seed=37)
    assert np.array_equal(a,first) and np.array_equal(b,second)
    assert not any(report[k] for k in ('flight_authorized','physical_accuracy_verified','metric_scale_verified','free_space_observed'))


def test_rejected_nominal_rotation_never_gets_perturbed_into_a_success():
    _,a,b,_,_=scene(pure_rotation=True)
    report=assess_pair(a,b,MODEL)
    assert report['status']=='nominal_rejected' and not report['passed']
    assert report['trial_results']==[] and 'nominal' not in report


def test_repeated_sift_orientations_are_removed_before_the_perturbations():
    _,a,b,_,_=scene()
    rejected=assess_pair(np.repeat(a[:10],5,axis=0),np.repeat(b[:10],5,axis=0),MODEL)
    assert not rejected['passed'] and rejected['trial_results']==[]
    assert 'distinct_points' in rejected['reason']
    accepted=assess_pair(np.repeat(a,2,axis=0),np.repeat(b,2,axis=0),MODEL,trials=8)
    assert accepted['passed']
    assert accepted['input_correspondences']==320 and accepted['distinct_correspondences']==160


def test_unstable_directions_fail_even_when_every_trial_triangulates(monkeypatch):
    _,a,b,_,_=scene()
    nominal=reconstruction.triangulate_correspondences(a,b,MODEL)
    def changed(*args):
        return nominal|{'second_world_to_camera_t':[1.,0.,0.]}
    monkeypatch.setattr(pair_stability,'triangulate_correspondences',changed)
    report=assess_pair(a,b,MODEL,trials=8,_nominal=nominal)
    assert report['valid_trials']==8 and not report['passed']
    assert report['translation_p95_degrees']>80
    assert 'translation_direction_unstable' in report['reason']


def test_rejected_trials_cannot_be_hidden_behind_small_surviving_angles(monkeypatch):
    _,a,b,_,_=scene();nominal=reconstruction.triangulate_correspondences(a,b,MODEL)
    calls=[0]
    def intermittent(*args):
        calls[0]+=1
        if calls[0]>6:raise ValueError('insufficient_parallax')
        return nominal
    monkeypatch.setattr(pair_stability,'triangulate_correspondences',intermittent)
    report=assess_pair(a,b,MODEL,trials=8,_nominal=nominal)
    assert report['valid_trials']==6 and not report['passed']
    assert report['translation_p95_degrees']<1e-5
    assert 'insufficient_valid_trials' in report['reason']


@pytest.mark.parametrize('options',[{'trials':True},{'trials':7},{'trials':33},
    {'sigma_px':float('nan')},{'sigma_px':0.},{'seed':True},{'seed':-1}])
def test_work_bounds_and_noise_assumptions_are_explicit(options):
    _,a,b,_,_=scene()
    with pytest.raises(ValueError,match='noise_options_invalid'):assess_pair(a,b,MODEL,**options)


@pytest.fixture
def image_pair(tmp_path,monkeypatch):
    _,a,b,_,_=scene();rng=np.random.default_rng(86)
    descriptors=rng.integers(0,64,(len(a),128)).astype(np.float32)
    paths=[tmp_path/'a.png',tmp_path/'b.png']
    for value,path in enumerate(paths):assert cv.imwrite(str(path),np.full((720,1280,3),value,np.uint8))
    monkeypatch.setattr(reconstruction,'_features',lambda image:(a if image[0,0,0]==0 else b,descriptors.copy()))
    model=tmp_path/'model.json';model.write_text(json.dumps(MODEL.as_dict()))
    return paths,model


def test_cli_keeps_failure_diagnostic_without_saving_unstable_points(image_pair,tmp_path,monkeypatch):
    images,model=image_pair
    rejected={'status':'unstable','passed':False,'reason':'translation_direction_unstable'}
    monkeypatch.setattr(pair_stability,'assess_pair',lambda *a,**kw:rejected.copy())
    output=tmp_path/'map.json';report=tmp_path/'noise.json'
    result=cli.main(['reconstruct-pair',*map(str,images),'--camera-model',str(model),
        '--output',str(output),'--noise-report',str(report)])
    assert result==1 and not output.exists()
    saved=json.loads(report.read_text());assert saved['passed'] is False
    assert len(saved['image_sha256'])==2 and report.stat().st_mode&0o777==0o600


def test_checked_and_explicitly_unchecked_scans_retain_the_same_nominal_geometry(image_pair):
    images,_=image_pair
    checked=reconstruction.reconstruct_pair(*images,MODEL,noise_trials=8)
    unchecked=reconstruction.reconstruct_pair(*images,MODEL,noise_trials=0)
    assert checked['pair_noise_check']['passed']
    assert unchecked['pair_noise_check']['status']=='not_checked'
    assert unchecked['pair_noise_check']['passed'] is False
    assert checked['points']==unchecked['points']
    assert checked['distance_unit']==unchecked['distance_unit']=='baseline'
    assert checked['metric_scale_verified'] is False and checked['free_space_observed'] is False


def test_cli_rejects_conflicting_output_paths_before_any_report_write(image_pair,tmp_path):
    images,model=image_pair;output=tmp_path/'same.json'
    assert cli.main(['reconstruct-pair',*map(str,images),'--camera-model',str(model),
        '--output',str(output),'--noise-report',str(output)])==1
    assert not output.exists()


def test_atlas_cli_preserves_stability_rejection_without_a_partial_map(tmp_path,monkeypatch):
    from neo2_control import metric_atlas
    report={'status':'unstable','passed':False,'reason':'rotation_unstable'}
    monkeypatch.setattr(metric_atlas,'load_atlas',lambda _: {})
    def reject(*args):raise PairStabilityError(report)
    monkeypatch.setattr(metric_atlas,'extend_atlas',reject)
    output=tmp_path/'atlas.json';diagnostic=tmp_path/'noise.json'
    assert cli.main(['atlas-extend',str(tmp_path/'input.json'),'first.png','second.png',
        '--output',str(output),'--noise-report',str(diagnostic)])==1
    assert not output.exists() and json.loads(diagnostic.read_text())==report
