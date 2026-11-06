"""Equivalent offline math, not a relaxed sensor or collision contract."""
from array import array
from dataclasses import replace

import numpy as np
import pytest

from d1max_pct_scan import ray_projection as projection
from test_ray_projection import core, feed, raw_points, decode, project


@pytest.mark.parametrize('offsets', [[], [0], [0, 0, 0], [0, 2, 2, 3, 150000000],
    [100000000, 0, 100000000, 1, 1, 2], list(range(1000)), list(reversed(range(1000)))])
def test_exact_acquisition_groups_match_numpy_without_changing_input_order(offsets):
    values = np.asarray(offsets, dtype=np.uint32)
    original = values.copy()
    unique, indices = projection.acquisition_groups(values)
    expected, expected_indices = np.unique(values, return_inverse=True)
    np.testing.assert_array_equal(unique, expected)
    np.testing.assert_array_equal(indices, expected_indices)
    np.testing.assert_array_equal(unique[indices], original)
    np.testing.assert_array_equal(values, original)


@pytest.mark.parametrize('seed', [1, 3, 107, 4089])
@pytest.mark.parametrize('sensor,ordered,strided', [
    (0, True, False), (1, True, False), (0, False, False), (1, False, True)])
def test_full_projection_matches_original_dense_products_and_preserves_wire_fields(
        monkeypatch, seed, sensor, ordered, strided):
    rng = np.random.default_rng(seed)
    points = raw_points(n=4097, sensor=sensor, origin=(-.73 if sensor else 0., .02, .06))
    for name in ('x', 'y', 'z'):
        points[name] = rng.normal(size=len(points))*10.
    # Do not quantize instants: retain irregular ns offsets and arbitrary order.
    if not ordered:
        order = rng.permutation(len(points))
        for name in ('offset_time', 'timestamp', 'source_timestamp', 'raw_timestamp'):
            points[name] = points[name][order]
    raw = decode(points)
    if strided:
        raw = replace(raw, points=raw.points[::2])
    original = raw.points.tobytes()
    value = core()
    feed(value)
    actual = project(value.projection_snapshot(), raw)

    def original_dense(matrices, indices, data, translations=None):
        endpoints = np.column_stack([data[name] for name in ('x', 'y', 'z')])
        result = np.einsum('nij,nj->ni', matrices[indices], endpoints)
        if translations is not None:
            result += translations[indices]
        return result

    monkeypatch.setattr(projection, 'transform_indexed_endpoints', original_dense)
    monkeypatch.setattr(projection, 'acquisition_groups',
                        lambda offsets: np.unique(offsets, return_inverse=True))
    expected = project(value.projection_snapshot(), raw)
    assert actual.points.tobytes() == expected.points.tobytes()
    assert len(actual.points) == len(raw.points)
    assert (actual.start_ns, actual.end_ns, actual.alignment_ns, actual.sensor_id,
            actual.context, actual.sequence) == (
        expected.start_ns, expected.end_ns, expected.alignment_ns, expected.sensor_id,
        expected.context, expected.sequence)
    assert raw.points.tobytes() == original
    assert not np.shares_memory(actual.points, raw.points)
    assert isinstance(actual.wire_data, array) and actual.wire_data.typecode == 'B'
    assert actual.wire_data.tobytes() == actual.points.tobytes()
    assert np.shares_memory(actual.points, np.frombuffer(actual.wire_data, dtype=np.uint8))
    committed = value.commit_projection(actual)
    assert committed.wire_data is actual.wire_data
    assert committed.points is actual.points
    for name in ('intensity', 'sensor_id', 'ring', 'offset_time', 'source_index',
                 'timestamp', 'source_timestamp', 'raw_timestamp'):
        np.testing.assert_array_equal(actual.points[name], raw.points[name])


@pytest.mark.parametrize('strided', [False, True])
def test_final_wire_buffer_contains_only_existing_mask_retained_rays_without_shortening_time(strided):
    from d1max_pct_scan.preview_ray_exclusion import from_session
    from test_preview_ray_exclusion import configured_core, preview_session

    points = raw_points(n=9, origin=(-.73, .02, 0.))
    points['x'] = [.5, .7, .5, .8, 2., .7, .5, .8, .5]
    raw = decode(points)
    if strided:
        raw = replace(raw, points=raw.points[::2])
    baseline = configured_core()
    filtered = configured_core(from_session(preview_session(), 'body'))
    for value in (baseline, filtered):
        feed(value)
    full = project(baseline, raw)
    result = project(filtered, raw)
    keep = raw.points['x'] != .5
    assert result.points.tobytes() == full.points[keep].tobytes()
    assert result.wire_data.tobytes() == result.points.tobytes()
    assert len(result.wire_data) == len(result.points)*projection.RAY_DTYPE.itemsize
    assert np.shares_memory(result.points, np.frombuffer(result.wire_data, dtype=np.uint8))
    assert not np.shares_memory(result.points, raw.points)
    assert (result.start_ns, result.end_ns, result.alignment_ns) == (
        full.start_ns, full.end_ns, full.alignment_ns)
    assert result.points['offset_time'].min() > 0
    assert result.points['offset_time'].max() < raw.end_ns-raw.start_ns


@pytest.mark.parametrize('shape', [(3,), (0, 3), (1, 3), (7001, 3)])
def test_fixed_quaternion_fast_path_preserves_exact_cross_arithmetic(shape):
    rng = np.random.default_rng(329)
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    vectors = rng.normal(size=shape)
    original = vectors.copy()
    cross = 2.*np.cross(q[:3], vectors)
    expected = vectors+q[3]*cross+np.cross(q[:3], cross)
    actual = projection.rotate_many(q, vectors)
    np.testing.assert_array_equal(actual, expected)
    np.testing.assert_array_equal(vectors, original)


def test_fixed_quaternion_fast_path_supports_noncontiguous_points():
    data = np.arange(180., dtype=float).reshape(60, 3)[::3]
    q = np.asarray([.2, -.1, .3, np.sqrt(.86)])
    cross = 2.*np.cross(q[:3], data)
    np.testing.assert_array_equal(projection.rotate_many(q, data),
                                  data+q[3]*cross+np.cross(q[:3], cross))
