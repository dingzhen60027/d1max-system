"""Independent unsafe-counterexample review of the optional native mesh kernel.

The Python production fallback is the oracle; this suite starts no ROS/SDK.
Set D1MAX_REQUIRE_NATIVE_TEST=1 for the isolated built-extension acceptance run.
"""
from concurrent.futures import ThreadPoolExecutor
import gc
import importlib
import math
import os
from pathlib import Path
import struct
import time

import numpy as np
import pytest

from d1max_pct_scan import robot_self_geometry as geometry


try:
    importlib.import_module('d1max_pct_scan._mesh_ray_native')
except ImportError:
    if os.environ.get('D1MAX_REQUIRE_NATIVE_TEST') == '1':
        raise
    pytest.skip('optional native mesh extension not built', allow_module_level=True)


def make_surface(tmp_path, triangles, scale=(1., 1., 1.)):
    triangles = np.asarray(triangles, dtype=np.float32).reshape(-1, 3, 3)
    path = tmp_path/'review.stl'
    records = b''.join(struct.pack('<12fH', *([0., 0., 0.]+list(t.ravel())), 0)
                       for t in triangles)
    path.write_bytes(bytes(80)+struct.pack('<I', len(triangles))+records)
    urdf = tmp_path/'review.urdf'
    urdf.write_text('<robot name="review"><link name="base"><collision><geometry>'
                    '<mesh filename="review.stl" scale="'+ ' '.join(map(str, scale)) +
                    '"/></geometry></collision></link></robot>')
    model = geometry.RobotSelfModel.from_urdf(urdf)
    surface = geometry._MeshSurface(model.collisions[0])
    assert surface._native is not None
    return surface


def assert_equivalent(surface, origins, endpoints, padding=0.):
    with np.errstate(all='ignore'):
        reference, reference_timeout = surface._first_intersections_python(
            origins, endpoints, padding)
        actual, actual_timeout = surface.first_intersections(origins, endpoints, padding)
    assert reference_timeout is actual_timeout is False
    np.testing.assert_array_equal(np.isfinite(actual), np.isfinite(reference))
    np.testing.assert_allclose(actual, reference, rtol=3e-13, atol=3e-13)
    return actual


def test_native_review_barycentric_and_distance_epsilon_boundaries(tmp_path):
    surface = make_surface(tmp_path, [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]])
    eps = 1e-9
    changes = [-2e-15, 0., 2e-15]
    uv = [[-eps+d, .25] for d in changes]
    uv += [[.25, -eps+d] for d in changes]
    uv += [[.25, .75+eps+d] for d in changes]
    uv += [[0., 0.], [1., 0.], [0., 1.], [.5, .5], [.25, .25]]
    origins = np.column_stack((uv, np.ones(len(uv))))
    endpoints = origins.copy();endpoints[:, 2] = -1.
    assert_equivalent(surface, origins, endpoints)
    for padding in (0., .01):
        for offset in (-2e-15, 0., 2e-15):
            endpoint = np.array([[.25, .25, padding+eps+offset]])
            assert_equivalent(surface, [[.25, .25, 1.]], endpoint, padding)


def test_native_review_parallel_determinant_and_zero_length_boundaries(tmp_path):
    surface = make_surface(tmp_path, [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]])
    rays = []
    for multiplier in (.249999999, .25, .250000001, .499999999, .5, .500000001, 1., 2.):
        height = multiplier*1e-12
        rays.append(([.1, .25, height], [.6, .25, -height]))
    rays += [([.25, .25, 0.], [.75, .25, 0.]),
             ([.25, .25, 1.], [.25, .25, 1.]),
             ([.25, .25, 0.], [.25, .25, 1.]),
             ([math.nan, 0., 1.], [0., 0., 0.]),
             ([0., 0., 1.], [math.inf, 0., 0.])]
    origins, endpoints = map(np.asarray, zip(*rays))
    assert_equivalent(surface, origins, endpoints)


def test_native_review_first_hit_not_last_or_occluded_endpoint(tmp_path):
    triangles = []
    for z in (0., .2, .1):
        triangles += [[[0, 0, z], [1, 0, z], [0, 1, z]]]
    surface = make_surface(tmp_path, triangles)
    actual = assert_equivalent(surface, [[.2, .2, 1.], [.2, .2, -1.]],
                               [[.2, .2, -.5], [.2, .2, .5]])
    np.testing.assert_allclose(actual, [.8, 1.], atol=1e-8)
    # A return on a hidden farther surface is not a first self surface.
    first = assert_equivalent(surface, [[.2, .2, 1.]], [[.2, .2, 0.]])[0]
    assert abs(first-1.) > .01


def test_native_review_random_scaled_degenerate_strided_and_reversed(tmp_path):
    rng = np.random.default_rng(830941)
    triangles = rng.uniform(-.6, .6, (150, 3, 3)).astype(np.float32)
    triangles[::11, 2] = triangles[::11, 1]  # Degenerate is never a ghost surface.
    surface = make_surface(tmp_path, triangles, (.731, 1.379, 2.117))
    origins = rng.uniform(-2., 2., (3000, 3))
    endpoints = rng.uniform(-.9, .9, (3000, 3))
    assert_equivalent(surface, origins, endpoints, .007)
    assert_equivalent(surface, origins[::-3], endpoints[::-3], .007)
    assert_equivalent(surface, origins.astype(np.float32), endpoints.astype(np.float32), .007)
    assert_equivalent(surface, endpoints[:700], origins[:700], .007)


def test_native_review_surface_candidates_is_only_superset_filter(tmp_path):
    rng = np.random.default_rng(529)
    surface = make_surface(tmp_path, rng.uniform(-1., 1., (96, 3, 3)))
    points = rng.uniform(-1.5, 1.5, (2048, 3))
    points[:3] = [[math.nan, 0., 0.], [math.inf, 0., 0.], [0., 0., -math.inf]]
    for sample in (points, points[::-2], points.astype(np.float32)):
        reference, timed = surface._surface_candidates_python(sample, .013)
        actual, timed_actual = surface.surface_candidates(sample, .013)
        assert not timed and not timed_actual
        np.testing.assert_array_equal(actual, reference)


def test_native_review_expired_and_mid_batch_deadlines_erase_all_partial_hits(tmp_path):
    surface = make_surface(tmp_path, [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]])
    origins = np.repeat([[.25, .25, 1.]], 400000, axis=0)
    endpoints = np.repeat([[.25, .25, -1.]], len(origins), axis=0)
    for deadline in (time.monotonic()-1., time.monotonic()+.0001):
        first, timed = surface.first_intersections(origins, endpoints, .01, deadline)
        assert timed and np.isinf(first).all()
    for deadline in (time.monotonic()-1., time.monotonic()+.0001):
        candidates, timed = surface.surface_candidates(endpoints, .01, deadline)
        assert timed and not candidates.any()
    # Pass pre-frozen buffers directly, so copying cannot consume the budget
    # before traversal. Every ray/point is a hit; partial successes must vanish.
    frozen_origins = geometry._readonly_query_array(origins)
    frozen_ends = geometry._readonly_query_array(endpoints)
    points = geometry._readonly_query_array(np.repeat([[.25, .25, 0.]], len(origins), axis=0))
    first, timed = surface._native.first_intersections(
        frozen_origins, frozen_ends, .01, time.monotonic()+.001)
    assert timed and np.isinf(first).all()
    candidates, timed = surface._native.surface_candidates(points, .01, time.monotonic()+.001)
    assert timed and not candidates.any()


def test_native_review_owns_mesh_buffers_after_python_surface_release(tmp_path):
    surface = make_surface(tmp_path, [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]])
    native = surface._native
    origins = np.array([[.25, .25, 1.]])
    endpoints = np.array([[.25, .25, -1.]])
    origins.flags.writeable = endpoints.flags.writeable = False
    del surface;gc.collect()
    first, timed = native.first_intersections(origins, endpoints, 0., math.inf)
    assert not timed
    np.testing.assert_allclose(first, [1.], atol=1e-14)


def test_native_review_shared_mesh_two_readers_do_not_share_partial_state(tmp_path):
    rng = np.random.default_rng(703)
    surface = make_surface(tmp_path, rng.uniform(-.5, .5, (100, 3, 3)))
    origins = rng.uniform(-1., 1., (2000, 3))
    endpoints = rng.uniform(-.7, .7, (2000, 3))
    expected, _ = surface._first_intersections_python(origins, endpoints, .003)
    with ThreadPoolExecutor(max_workers=2) as workers:
        jobs = [workers.submit(surface.first_intersections, origins, endpoints, .003)
                for _ in range(6)]
        for job in jobs:
            actual, timed = job.result(timeout=10.)
            assert not timed
            np.testing.assert_array_equal(np.isfinite(actual), np.isfinite(expected))
            np.testing.assert_allclose(actual, expected, rtol=3e-13, atol=3e-13)


def test_native_review_non_axis_barycentric_rounding_edges(tmp_path):
    rng = np.random.default_rng(913104)
    for case in range(48):
        triangle = rng.uniform(-.8, .8, (3, 3)).astype(np.float32)
        scale = rng.uniform(.4, 2., 3)
        surface = make_surface(tmp_path, [triangle], scale)
        vertices = triangle.astype(float)*scale
        edge1, edge2 = vertices[1]-vertices[0], vertices[2]-vertices[0]
        normal = np.cross(edge1, edge2)
        normal /= np.linalg.norm(normal)
        uv = []
        for perturbation in np.linspace(-2e-15, 2e-15, 41):
            uv += [[-1e-9+perturbation, .291], [.291, -1e-9+perturbation],
                   [.291, .709+1e-9+perturbation]]
        uv = np.asarray(uv)
        points = vertices[0]+uv[:, :1]*edge1+uv[:, 1:]*edge2
        assert_equivalent(surface, points+normal, points-normal)


def test_native_review_query_buffers_own_readonly_copy():
    caller = np.arange(30., dtype=np.float64).reshape(10, 3)
    frozen = geometry._readonly_query_array(caller)
    assert frozen.flags.c_contiguous and not frozen.flags.writeable
    assert not np.shares_memory(caller, frozen)
    expected = frozen.copy()
    caller[:] = math.nan
    np.testing.assert_array_equal(frozen, expected)


def test_native_review_rejects_unsafe_native_shapes_and_dtypes(tmp_path):
    surface = make_surface(tmp_path, [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]])
    writable = np.array([[.25, .25, 1.]])
    with pytest.raises(ValueError, match='read-only'):
        surface._native.first_intersections(writable, writable, 0., math.inf)
    readonly = geometry._readonly_query_array(writable)
    for invalid in (readonly.astype(np.float32), readonly[:, ::-1], readonly[0]):
        invalid.flags.writeable = False
        with pytest.raises((TypeError, ValueError)):
            surface._native.first_intersections(invalid, readonly, 0., math.inf)
    with pytest.raises(ValueError, match='shape'):
        surface._native.first_intersections(readonly[:0], readonly, 0., math.inf)


def test_native_review_rejects_invalid_hierarchy_before_gil_release(tmp_path):
    surface = make_surface(tmp_path, [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]])
    native_module = importlib.import_module('d1max_pct_scan._mesh_ray_native')
    original = [surface.triangles, surface.edges, surface.scale, surface.bvh_lower,
                surface.bvh_upper, surface.bvh_children, surface.bvh_spans,
                surface.tree.indices]
    for index, value, error in ((5, [[0, 0]], 'child'),  # Cycle/root-as-child.
                                (5, [[-1, 9]], 'child'),
                                (6, [[0, 2]], 'span'),
                                (6, [[-1, 1]], 'span'),
                                (7, [-1], 'triangle'),
                                (7, [1], 'triangle'),
                                (3, [[math.nan, 0., 0.]], 'bounds'),
                                (2, [1., math.inf, 1.], 'scale')):
        arrays = list(original)
        invalid = np.asarray(value, dtype=original[index].dtype)
        invalid.flags.writeable = False
        arrays[index] = invalid
        with pytest.raises(ValueError, match=error):
            native_module.Mesh(*arrays)


def test_native_review_missing_extension_uses_same_python_geometry(tmp_path, monkeypatch):
    accelerated = make_surface(tmp_path, [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]])
    collision = geometry.RobotSelfModel.from_urdf(tmp_path/'review.urdf').collisions[0]
    monkeypatch.setattr(geometry, '_NativeMesh', None)
    fallback = geometry._MeshSurface(collision)
    assert fallback._native is None
    origins = np.array([[.2, .2, 1.], [1., 1., 1.], [.2, .2, 1.]])
    ends = np.array([[.2, .2, 0.], [1., 1., 0.], [.2, .2, -1.]])
    expected, timed = accelerated._first_intersections_python(origins, ends, .002)
    actual, timed_fallback = fallback.first_intersections(origins, ends, .002)
    assert not timed and not timed_fallback
    np.testing.assert_array_equal(actual, expected)
    candidates, _ = fallback.surface_candidates(ends, .002)
    expected_candidates, _ = accelerated._surface_candidates_python(ends, .002)
    np.testing.assert_array_equal(candidates, expected_candidates)


def test_native_review_marginal_guard_reuses_oracle_and_original_deadline(tmp_path, monkeypatch):
    surface = make_surface(tmp_path, [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]])
    origins = np.array([[-1e-9, .25, 1.], [.25, .25, 1.]])
    endpoints = origins.copy();endpoints[:, 2] = -1.
    frozen_origins = geometry._readonly_query_array(origins)
    frozen_endpoints = geometry._readonly_query_array(endpoints)
    first, timed, marginal = surface._native.first_intersections(
        frozen_origins, frozen_endpoints, 0., math.inf, checked=True)
    assert not timed and marginal
    inner = surface._first_intersections_python
    calls = []
    def oracle(start, end, padding, deadline):
        calls.append((padding, deadline))
        assert not start.flags.writeable and not end.flags.writeable
        assert not np.shares_memory(start, origins)
        assert not np.shares_memory(end, endpoints)
        return inner(start, end, padding, deadline)
    monkeypatch.setattr(surface, '_first_intersections_python', oracle)
    deadline = time.monotonic()+2.
    actual, timed = surface.first_intersections(origins, endpoints, 0., deadline)
    expected, timed_reference = inner(origins, endpoints, 0., deadline)
    assert not timed and not timed_reference
    assert calls == [(0., deadline)]
    np.testing.assert_array_equal(actual, expected)
    # A clearly interior-only batch should not lose the native fast path.
    _, timed, marginal = surface._native.first_intersections(
        frozen_origins[1:], frozen_endpoints[1:], 0., math.inf, checked=True)
    assert not timed and not marginal


def test_native_review_oracle_cannot_return_success_after_original_deadline(tmp_path, monkeypatch):
    surface = make_surface(tmp_path, [[[0, 0, 0], [1, 0, 0], [0, 1, 0]]])
    origins = np.array([[-1e-9, .25, 1.], [.25, .25, 1.]])
    endpoints = origins.copy();endpoints[:, 2] = -1.
    deadline = time.monotonic()+2.
    calls = []
    def completed_too_late(start, end, padding, supplied_deadline):
        calls.append(supplied_deadline)
        # Simulates the oracle completing its final loop just after expiry.
        monkeypatch.setattr(geometry.time, 'monotonic', lambda: deadline+.001)
        return np.ones(len(end)), False
    monkeypatch.setattr(surface, '_first_intersections_python', completed_too_late)
    actual, timed = surface.first_intersections(origins, endpoints, 0., deadline)
    assert calls == [deadline]
    assert timed and np.isinf(actual).all()
