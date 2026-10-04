"""Equivalent offline math, not a relaxed sensor or collision contract."""
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

    def original_dense(matrices, indices, data):
        endpoints = np.column_stack([data[name] for name in ('x', 'y', 'z')])
        return np.einsum('nij,nj->ni', matrices[indices], endpoints)

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
    for name in ('intensity', 'sensor_id', 'ring', 'offset_time', 'source_index',
                 'timestamp', 'source_timestamp', 'raw_timestamp'):
        np.testing.assert_array_equal(actual.points[name], raw.points[name])
