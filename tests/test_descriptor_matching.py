"""Shared distances preserve the original global mutual ratio decisions."""
import pytest

np = pytest.importorskip('numpy')
pytest.importorskip('cv2')

from neo2_control.descriptor_matching import FrozenDescriptorMatcher, MAXIMUM_TILE_BYTES
from neo2_control.reconstruction import _matches


def scene(rows=513, queries=113, seed=12):
    rng = np.random.default_rng(seed)
    descriptors = rng.integers(0, 64, (rows, 128)).astype(np.float32)
    selected = rng.choice(rows, queries, replace=queries > rows)
    features = descriptors[selected]+rng.integers(-2, 3, (queries, 128))
    return descriptors, np.clip(features, 0, 512).astype(np.float32)


@pytest.mark.parametrize('rows,queries', [(513, 71), (512, 137), (769, 521), (257, 5000)])
def test_tiled_global_neighbours_equal_the_original_pairs_across_tile_edges(rows, queries):
    descriptors, features = scene(rows, queries)
    expected = _matches(descriptors, features)
    got, evidence = FrozenDescriptorMatcher(descriptors).match(features)
    assert got == expected
    assert evidence['backend'] == 'tiled_exact_sift'
    assert evidence['exact_mutual_ratio']
    assert 0 < evidence['distance_tile_bytes'] <= MAXIMUM_TILE_BYTES


def test_duplicate_and_zero_distances_never_become_false_ratio_matches():
    descriptors, features = scene()
    descriptors[:4] = 0.
    features[:4] = 0.
    descriptors[4:8] = descriptors[8]
    features[4:8] = descriptors[8]
    got, evidence = FrozenDescriptorMatcher(descriptors).match(features)
    assert got == _matches(descriptors, features)
    assert not any(i < 9 or j < 8 for i, j in got)
    assert evidence['backend'] == 'tiled_exact_sift'


def test_large_query_uses_actual_distance_blocks_within_the_memory_limit(monkeypatch):
    import cv2 as cv
    descriptors, features = scene(513, 5000)
    sizes = []
    original = cv.gemm

    def measured(*args, **kwargs):
        result = original(*args, **kwargs)
        sizes.append(result.nbytes)
        return result

    monkeypatch.setattr(cv, 'gemm', measured)
    got, _ = FrozenDescriptorMatcher(descriptors).match(features)
    assert got == _matches(descriptors, features)
    assert len(sizes) >= 2 and max(sizes) <= MAXIMUM_TILE_BYTES


@pytest.mark.parametrize('side', ['forward', 'reverse'])
def test_ratio_rounding_boundary_keeps_the_original_opencv_decision(side):
    rng = np.random.default_rng(5)
    descriptors = rng.integers(20, 40, (300, 128)).astype(np.float32)
    features = rng.integers(20, 40, (90, 128)).astype(np.float32)
    if side == 'forward':
        descriptors[0] = features[0] = features[1] = 0.
        features[0, 0], features[1, 0] = 72., 100.
    else:
        features[0] = descriptors[0] = descriptors[1] = 0.
        descriptors[0, 0], descriptors[1, 0] = 72., 100.
    got, evidence = FrozenDescriptorMatcher(descriptors).match(features)
    assert got == _matches(descriptors, features)
    assert evidence['reason'] == 'ratio_rounding_boundary'


@pytest.mark.parametrize('kind', ['fractional_map', 'fractional_query', 'large_norm', 'negative'])
def test_arbitrary_float_descriptors_keep_the_general_matching_path(kind):
    descriptors, features = scene()
    if kind == 'fractional_map':
        descriptors[0, 0] += .5
    elif kind == 'fractional_query':
        features[0, 0] += .5
    elif kind == 'large_norm':
        descriptors[0] = 512.
    else:
        features[0, 0] = -1.
    got, evidence = FrozenDescriptorMatcher(descriptors).match(features)
    assert got == _matches(descriptors, features)
    assert evidence['backend'] == 'opencv_bf'
    assert evidence['reason'] == 'non_integral_or_large_norm'


def test_frozen_index_owns_its_copy_and_accepts_noncontiguous_query_storage():
    descriptors, features = scene()
    matcher = FrozenDescriptorMatcher(descriptors)
    padded = np.zeros((len(features), 256), dtype=np.float32)
    padded[:, ::2] = features
    assert not padded[:, ::2].flags.c_contiguous
    expected = _matches(descriptors, features)
    descriptors[:] = 0.
    assert matcher.match(padded[:, ::2])[0] == expected
    with pytest.raises(ValueError):
        matcher._descriptors[0, 0] = 3.


@pytest.mark.parametrize('rows,queries', [(1, 90), (300, 1), (30, 30)])
def test_small_matrices_and_missing_second_neighbours_preserve_baseline(rows, queries):
    descriptors, features = scene(rows, queries)
    got, evidence = FrozenDescriptorMatcher(descriptors).match(features)
    assert got == _matches(descriptors, features)
    assert evidence['backend'] == 'opencv_bf'


@pytest.mark.parametrize('value', [np.empty((0, 128), np.float32), np.zeros((30, 127), np.float32),
    np.zeros((30, 128), np.float64), np.full((30, 128), np.nan, np.float32)])
def test_invalid_descriptor_matrices_are_rejected(value):
    with pytest.raises(ValueError, match='descriptor_matrix_invalid'):
        FrozenDescriptorMatcher(value)


@pytest.mark.parametrize('rows,queries',[(513,217),(30,80)])
@pytest.mark.parametrize('fractional',[False,True])
def test_blas_and_fallback_preserve_the_requested_mutual_ratio(rows,queries,fractional):
    import cv2 as cv
    first,second=scene(rows,queries)
    if fractional:first[0,0]+=.5
    matcher=cv.BFMatcher(cv.NORM_L2)
    forward=matcher.knnMatch(first,second,k=2);reverse=matcher.knnMatch(second,first,k=2)
    allowed={(m[0].trainIdx,m[0].queryIdx) for m in reverse
        if len(m)==2 and m[0].distance<.8*m[1].distance}
    expected=[(m[0].queryIdx,m[0].trainIdx) for m in forward
        if len(m)==2 and m[0].distance<.8*m[1].distance
        and (m[0].queryIdx,m[0].trainIdx) in allowed]
    actual,evidence=FrozenDescriptorMatcher(first,ratio=.8,use_blas=True).match(second)
    assert actual==expected
    assert evidence['backend']==('opencv_bf' if fractional or rows<256 else 'tiled_exact_sift_blas')


def test_custom_ratio_boundary_keeps_the_opencv_decision():
    first,second=scene(300,90)
    first[0]=second[0]=second[1]=0
    second[0,0],second[1,0]=80.,100.
    pairs,evidence=FrozenDescriptorMatcher(first,ratio=.8,use_blas=True).match(second)
    assert (0,0) not in pairs
    assert evidence['reason']=='ratio_rounding_boundary'
