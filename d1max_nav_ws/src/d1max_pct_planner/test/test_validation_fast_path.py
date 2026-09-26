"""Differential tests against the retained general layer-state validator.

The optimized branch must alter neither support decisions nor audit counters.
No cross-request cache is involved: mutation tests intentionally reuse paths.
"""
import inspect
import textwrap

import numpy as np
import pytest

from d1max_pct_planner.tomogram_map import TomogramError, TomogramMap
from d1max_pct_planner.tomogram_route import curve_partition, real_unit_roots


def original_segment_samples(tomogram, a, b):
    """Vector implementation before the scalar per-segment specialization."""
    for endpoint in (a, b):
        if not tomogram.contains(tuple(tomogram.index(endpoint))):
            raise TomogramError('curve_outside_map', 'Curve endpoint lies outside the tomogram')
    start = (np.asarray(a)-tomogram.center)/tomogram.resolution+tomogram.offset
    end = (np.asarray(b)-tomogram.center)/tomogram.resolution+tomogram.offset
    delta = end-start
    boundaries = {0., 1.}
    for axis in range(2):
        if abs(delta[axis])<1e-12:
            continue
        low, high = sorted((start[axis], end[axis]))
        for integer in range(int(np.ceil(low-.5)), int(np.floor(high-.5))+1):
            t = (integer+.5-start[axis])/delta[axis]
            if 0<t<1:
                boundaries.add(float(t))
    boundaries = sorted(boundaries)
    times = sorted(boundaries+[(a+b)/2 for a,b in zip(boundaries,boundaries[1:])])
    ax, ay = float(a[0]), float(a[1])
    dx, dy = float(b[0])-ax, float(b[1])-ay
    return [(t, tomogram._point_cells_scalar(ax+t*dx, ay+t*dy)) for t in times]


def scene(seed=0, hazards=False):
    rng = np.random.default_rng(seed)
    data = np.zeros((5, 2, 17, 18), dtype=np.float32)
    data[3] = rng.choice([0., .03, .12, .17], data[3].shape) if hazards else 0.
    data[4] = 2.
    if hazards:
        data[0] = rng.choice([0., 5., 20., 21., np.nan], data[0].shape, p=[.77, .1, .05, .06, .02])
        data[4][rng.random(data[4].shape)<.05] = np.nan
        data[4][rng.random(data[4].shape)<.03] = .2
        data[3][rng.random(data[3].shape)<.02] = np.nan
    return TomogramMap({'data': data, 'resolution': .2, 'center': [12.3, -4.2],
                        'slice_h0': .5, 'slice_dh': .5})


@pytest.fixture(scope='module')
def general_validator():
    # Exercise the unchanged general algorithm in the same implementation,
    # bypassing *only* the same-layer specialization. This avoids maintaining
    # an oracle that silently diverges from future clearance-policy changes.
    source = textwrap.dedent(inspect.getsource(TomogramMap.validate_path))
    assert source.count('if first == last:') == 1
    scope = {'np': np, 'TomogramError': TomogramError, 'original_segment_samples': original_segment_samples}
    source = source.replace('if first == last:', 'if False:')
    source = source.replace('self._segment_samples_f64(a, b)',
                            'original_segment_samples(self, a[:2], b[:2])')
    exec(source, scope)
    return scope['validate_path']


def result_or_error(call):
    try:
        return ('ok', call())
    except TomogramError as error:
        return ('error', error.as_dict())


@pytest.mark.parametrize('hazards', [False, True])
@pytest.mark.parametrize('policy', ['allow_unobserved', 'reject'])
def test_fast_same_layer_and_mixed_paths_match_general_state_machine(general_validator, hazards, policy):
    rng = np.random.default_rng(313)
    tomo = scene(729, hazards)
    tomo.unknown_ceiling_policy = policy
    for index in range(180):
        count = int(rng.integers(2, 7))
        relative = rng.uniform(-6.5, 6.5, (count, 2))
        if index % 3 == 0:
            # Corners/edges must check both sides; banker's rounding matters.
            relative = np.floor(relative) + .5
        points = np.c_[tomo.center + relative*tomo.resolution,
                       rng.choice([0., .03, -.04, .12, .18], count)]
        layers = np.zeros(count, dtype=int)
        if index % 5 == 0:
            layers[count//2:] = 1
        limits = {'max_ground_step_m': (.15, .17, .05)[index % 3],
                  'surface_tolerance_m': (.08, .18)[index % 2]}
        actual = result_or_error(lambda: tomo.validate_path(points, layers, **limits))
        expected = result_or_error(lambda: general_validator(tomo, points, layers, **limits))
        assert actual == expected


@pytest.mark.parametrize('mutation', ['cost', 'ceiling', 'ground', 'nan_ground', 'policy'])
def test_repeated_validation_never_reuses_a_stale_clearance_result(general_validator, mutation):
    tomo = scene()
    path = np.array([[*tomo.world([4, 8]), 0.], [*tomo.world([12, 8]), 0.]])
    tomo.validate_path(path, [0, 0])
    if mutation == 'cost':
        tomo.cost[0, 8, 8] = 50
    elif mutation == 'ceiling':
        tomo.ceiling[0, 8, 8] = .2
        tomo.headroom[0, 8, 8] = .2
    elif mutation == 'ground':
        tomo.ground[0, 8, 8] = .6
    elif mutation == 'nan_ground':
        # Even a partial/incorrect external array edit cannot pass the range
        # predicate just because NaN comparisons with > return false.
        tomo.ground[0, 8, 8] = np.nan
    else:
        tomo.ceiling_known[0, 8, 8] = False
        tomo.unknown_ceiling_policy = 'reject'
    actual = result_or_error(lambda: tomo.validate_path(path, [0, 0]))
    assert actual == result_or_error(lambda: general_validator(tomo, path, [0, 0]))
    assert actual[0] == 'error'


def test_lightweight_ground_query_preserves_full_selector_results_and_errors():
    tomo = scene(407, True)
    rng = np.random.default_rng(444)
    points = tomo.center + rng.uniform(-1.8, 1.8, (250, 2))
    boundaries = tomo.center + np.array([[i+.5, j+.5] for i in range(-7, 7)
                                       for j in range(-7, 7)]) * tomo.resolution
    for xy in np.vstack((points, boundaries)):
        for layer in [0, 1]:
            assert result_or_error(lambda: tomo.surface_ground_z(xy, layer)) == result_or_error(
                lambda: tomo._surface(xy, layer)['ground_z'])


@pytest.mark.parametrize('hazards', [False, True])
@pytest.mark.parametrize('policy', ['allow_unobserved', 'reject'])
def test_batch_ground_query_matches_scalar_values_and_ordered_errors(hazards, policy):
    tomo = scene(198, hazards)
    tomo.unknown_ceiling_policy = policy
    rng = np.random.default_rng(455)
    for index in range(100):
        positions = tomo.center+rng.uniform(-1.65, 1.65, (int(rng.integers(1, 40)), 2))
        if index % 3 == 0:
            relative = np.floor((positions-tomo.center)/tomo.resolution)+.5
            positions = tomo.center+relative*tomo.resolution
        layer = index % 2
        actual = result_or_error(lambda: tomo.surface_ground_z_many(positions, layer).tolist())
        expected = result_or_error(lambda: [tomo._surface(xy, layer)['ground_z'] for xy in positions])
        assert actual == expected


def test_batch_ground_query_chunking_empty_and_nonfinite_input():
    tomo = scene()
    points = np.repeat(tomo.center[None, :], 9001, axis=0)
    assert tomo.surface_ground_z_many(points, 0).tolist() == [0.]*len(points)
    assert tomo.surface_ground_z_many(np.empty((0, 2)), 0).shape == (0,)
    for bad in [[np.nan, 0.], [1e12, 1e12], [100., 100.]]:
        points[-1] = bad
        assert result_or_error(lambda: tomo.surface_ground_z_many(points, 0)) == result_or_error(
            lambda: [tomo._surface(xy, 0)['ground_z'] for xy in points])


def test_batch_ground_threshold_arithmetic_remains_double_precision():
    tomo = scene()
    points = np.array([tomo.center, tomo.center+[.1, 0]])
    for threshold in [20., np.nextafter(20., -np.inf), np.nextafter(20., np.inf)]:
        tomo.COST_THRESHOLD = threshold
        tomo.cost[0, 8, 9] = 20.
        actual = result_or_error(lambda: tomo.surface_ground_z_many(points, 0).tolist())
        expected = result_or_error(lambda: [tomo._surface(xy, 0)['ground_z'] for xy in points])
        assert actual == expected


def test_scalar_segment_partitions_preserve_exact_times_and_supercover():
    tomo = scene()
    rng = np.random.default_rng(279)
    segments = tomo.center+rng.uniform(-1.55, 1.55, (500, 2, 2))
    for a, b in segments:
        assert tomo._segment_samples_f64(a, b) == original_segment_samples(tomo, a, b)
    for x in range(-7, 7):
        for epsilon in [0., -1e-10, 1e-10, 1e-8]:
            a = tomo.center+[(x+.5+epsilon)*tomo.resolution, -1.1]
            b = tomo.center+[(x+.5+epsilon)*tomo.resolution, 1.1]
            assert tomo._segment_samples_f64(a, b) == original_segment_samples(tomo, a, b)
    for b in [[1e12, 1e12], [100., 100.]]:
        actual = result_or_error(lambda: tomo._segment_samples_f64(tomo.center, b))
        expected = result_or_error(lambda: original_segment_samples(tomo, tomo.center, b))
        assert actual == expected


def test_linear_roots_equal_original_polyroots_at_and_near_unit_endpoints():
    rng = np.random.default_rng(97)
    for root in [0., 1., np.nextafter(0., 1.), np.nextafter(1., 0.), -1e-9, 1+1e-9,
                 *rng.uniform(-10, 10, 200)]:
        for slope in [.01, -3., 1e8]:
            coefficients = np.array([-root*slope, slope, 0., 0.])
            values = np.polynomial.polynomial.polyroots(np.trim_zeros(coefficients, 'b'))
            expected = [float(v.real) for v in values if abs(v.imag)<=1e-8 and 0<v.real<1]
            assert real_unit_roots(coefficients) == expected


def test_linear_partition_keeps_all_crossings_and_interval_interiors(monkeypatch):
    import d1max_pct_planner.tomogram_route as route
    tomo = scene()
    rng = np.random.default_rng(619)
    for _ in range(100):
        a, b = tomo.center + rng.uniform(-1.2, 1.2, (2, 2))
        coefficients = np.array([a, b-a])
        actual = curve_partition(coefficients, tomo)

        def original_roots(poly):
            poly = np.trim_zeros(poly, 'b')
            if len(poly)<2:
                return []
            return [float(v.real) for v in np.polynomial.polynomial.polyroots(poly)
                    if abs(v.imag)<=1e-8 and 0<v.real<1]

        with monkeypatch.context() as patch:
            patch.setattr(route, 'real_unit_roots', original_roots)
            assert actual == curve_partition(coefficients, tomo)
