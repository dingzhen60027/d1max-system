import json

import numpy as np
import pytest

from d1max_pct_planner.corridor_centering import (
    ClearanceFieldCache, center_corridor, route_clearance_quality,
    MAX_ANCHORS, DISTANCE_SEMANTICS, _cubic_segments,
)
import d1max_pct_planner.corridor_centering as centering
from d1max_pct_planner.corridor_refinement import _curve_peak_curvature
from d1max_pct_planner.tomogram_map import TomogramMap


def scene(allowed=None, layers=1):
    coordinates = (np.arange(161)-80)*.1
    x, y = np.meshgrid(coordinates, coordinates, indexing='ij')
    mask = np.ones(x.shape, bool) if allowed is None else allowed(x, y)
    data = np.zeros((5, layers, 161, 161), np.float32)
    data[0] = np.where(mask, 0., 50.)
    data[4] = 2.
    return TomogramMap(dict(data=data, resolution=.1, center=[0., 0.],
                           slice_h0=.15, slice_dh=.5))


def line(y=-.72, begin=-5., end=5., count=101):
    return np.c_[np.linspace(begin, end, count), np.full(count, y), np.zeros(count)]


def corridor_with_alternating_door_recesses():
    # The long through corridor is straight; recesses outside its two walls
    # alternate every two metres.  They are valid map geometry, not noise to
    # erase, but a through route should not weave into each one.
    x, y = np.meshgrid((np.arange(641)-320)*.1, (np.arange(81)-40)*.1, indexing='ij')
    mask = ((abs(y) <= 1.)
            | ((np.mod(x, 4.) < 1.) & (y > 0.) & (y < 1.8))
            | ((np.mod(x+2., 4.) < 1.) & (y < 0.) & (y > -1.8)))
    data = np.zeros((5, 1, 641, 81), np.float32)
    data[0] = np.where(mask, 0., 50.)
    data[4] = 2.
    return TomogramMap(dict(data=data, resolution=.1, center=[0., 0.],
                           slice_h0=.15, slice_dh=.5))


def test_long_corridor_does_not_follow_each_door_recess_and_still_moves_away_from_wall():
    tomo = corridor_with_alternating_door_recesses()
    path = line(y=-.75, begin=-25., end=25., count=201)
    data = tomo.data.copy()
    result = center_corridor(tomo, path, [0]*len(path))
    assert result['applied'] and result['blend'] == 1., result
    output = np.asarray(result['path'])
    middle = output[abs(output[:, 0]) < 18.]
    # Recentring, not just rejecting all improved candidates, is required.
    assert np.max(abs(middle[:, 1])) < .15
    assert np.ptp(middle[:, 1]) < .02
    assert result['after_quality']['clearance_p10_m'] > .8
    assert result['after_quality']['interior_turn']['total_abs_turn_rad'] < .1
    # Both fixed off-centre endpoints need a gentle S transition.  A nonzero
    # total turn is expected; suppressing it would also suppress recentering.
    assert .5 < result['after_quality']['total_abs_turn_rad'] < 1.5
    assert result['after_quality']['xy_length_m'] < 50.5
    np.testing.assert_array_equal(output[[0, -1]], path[[0, -1]])
    np.testing.assert_array_equal(tomo.data, data)
    tomo.validate_path(output, result['layer_ids'])


@pytest.mark.parametrize('interior_only', [False, True])
def test_door_following_candidate_is_a_quality_fallback_not_a_failed_navigation(interior_only, monkeypatch):
    tomo = corridor_with_alternating_door_recesses()
    path = line(y=-.75, begin=-25., end=25., count=201)
    # Reproduce the old overly local response on the same actual map. Even
    # when it improves the distance score, it must not publish a snake path.
    monkeypatch.setattr(centering, 'OFFSET_SMOOTHING_M', .5)
    if interior_only:
        monkeypatch.setattr(centering, 'MAX_ADDITIONAL_TURN_RAD', 100.)
    result = center_corridor(tomo, path, [0]*len(path))
    assert not result['applied'] and 'path' not in result
    reason = 'excess_interior_turn' if interior_only else 'excess_additional_turn'
    assert all(failure['reason'] == reason for failure in result['candidate_failures'])
    # The original valid route is still available to the caller/selector.
    assert route_clearance_quality(tomo, path, [0]*len(path))['available']
    tomo.validate_path(path, [0]*len(path))


def test_straight_offcentre_route_moves_interior_toward_corridor_centre_without_moving_endpoints():
    tomo = scene(lambda x, y: abs(y) <= 1.)
    path = line()
    original = path.copy()
    data = tomo.data.copy()
    result = center_corridor(tomo, path, np.zeros(len(path), int))
    assert result['applied'], result
    output = np.asarray(result['path'])
    np.testing.assert_array_equal(output[[0, -1]], path[[0, -1]])
    np.testing.assert_array_equal(path, original)
    np.testing.assert_array_equal(tomo.data, data)
    middle = output[np.abs(output[:, 0]) < 2.]
    assert np.max(abs(middle[:, 1])) < .18
    assert result['after_clearance']['interior_excluding_2m']['mean_m'] > .8
    assert result['score_after'] < result['score_before']
    assert result['after_quality']['xy_length_m'] <= result['before_quality']['xy_length_m'] + .8
    assert result['analytic_curvature_peak_per_m'] <= 1.25+1e-6
    assert len(result['segments']) < MAX_ANCHORS
    assert result['curve_partition_points'] > len(result['segments'])
    assert not result['physical_safety_certified']
    tomo.validate_path(output, result['layer_ids'])
    json.dumps(result, allow_nan=False)


def test_open_area_does_not_invent_a_centreline_or_change_an_already_straight_route():
    path = line(y=.3, begin=-4., end=4.)
    result = center_corridor(scene(), path, [0]*len(path))
    assert not result['applied'] and result['reason'] == 'no_supported_centering_shift'
    assert result['sections']['open'] == 17
    assert 'path' not in result


def test_single_wall_moves_away_softly_without_chasing_the_other_side_of_open_space():
    tomo = scene(lambda x, y: y >= -1.)
    path = line(y=-.75)
    result = center_corridor(tomo, path, [0]*len(path))
    assert result['applied'], result
    output = np.asarray(result['path'])
    middle = output[abs(output[:, 0]) < 2.]
    assert np.max(abs(middle[:, 1])) < .25
    assert result['sections']['one_sided'] > 0
    assert result['sections']['two_sided'] == 0


def test_narrow_but_legal_corridor_retains_original_route_when_no_quality_improvement_exists():
    tomo = scene(lambda x, y: abs(y) < .16)
    path = line(y=0.)
    tomo.validate_path(path, [0]*len(path))
    result = center_corridor(tomo, path, [0]*len(path))
    assert not result['applied']
    assert result['reason'] == 'no_supported_centering_shift'
    assert 'path' not in result
    assert route_clearance_quality(tomo, path, [0]*len(path))['available']


def test_l_corridor_complete_cubic_stays_inside_walls_and_retains_exact_endpoints():
    tomo = scene(lambda x, y: ((x >= -6.) & (x <= 1.) & (abs(y) <= 1.))
                 | ((abs(x) <= 1.) & (y >= -1.) & (y <= 6.)))
    horizontal = np.c_[np.linspace(-5., -.7, 60), np.full(60, -.7), np.zeros(60)]
    vertical = np.c_[np.full(60, -.7), np.linspace(-.7, 5., 60), np.zeros(60)]
    path = np.vstack((horizontal, vertical[1:]))
    result = center_corridor(tomo, path, [0]*len(path))
    assert result['applied'], result
    output = np.asarray(result['path'])
    tomo.validate_path(output, result['layer_ids'])
    np.testing.assert_array_equal(output[[0, -1]], path[[0, -1]])
    assert result['curve_validation'].startswith('polynomial_cell_boundary_roots')
    segments = [{**s, 'coefficients_xy': np.asarray(s['coefficients_xy'])} for s in result['segments']]
    assert _curve_peak_curvature(segments) <= result['analytic_curvature_limit_per_m'] + 1e-6


@pytest.mark.parametrize('kind', ['wall', 'unknown_ground', 'headroom'])
def test_invalid_route_is_not_repaired_or_given_a_selectable_quality_score(kind):
    tomo = scene()
    if kind == 'wall':
        tomo.cost[0, 80, :] = 50.
    elif kind == 'unknown_ground':
        tomo.ground[0, 80, :] = np.nan
        tomo.ground_known[0, 80, :] = False
    else:
        tomo.headroom[0, 80, :] = .1
    path = line(y=0.)
    result = center_corridor(tomo, path, [0]*len(path))
    assert not result['applied'] and result['reason'] == 'invalid_input_path'
    assert 'path' not in result
    assert not route_clearance_quality(tomo, path, [0]*len(path))['available']


def test_unknown_ceiling_policy_is_not_relaxed_for_quality():
    tomo = scene()
    tomo.unknown_ceiling_policy = 'reject'
    tomo.ceiling_known[0, 80, :] = False
    path = line(y=0.)
    assert not center_corridor(tomo, path, [0]*len(path))['applied']
    assert not route_clearance_quality(tomo, path, [0]*len(path))['available']


def test_normal_section_stops_at_ground_step_instead_of_borrowing_the_other_surface():
    tomo = scene(lambda x, y: abs(y) <= 1.)
    tomo.ground[0, :, 84:] = .5
    path = line(y=-.7)
    result = center_corridor(tomo, path, [0]*len(path))
    if result['applied']:
        output = np.asarray(result['path'])
        assert np.max(output[:, 1]) < .35
        assert np.max(abs(output[:, 2])) < 1e-8
        tomo.validate_path(output, result['layer_ids'])
    else:
        assert 'path' not in result


def test_multilayer_and_stair_height_profiles_are_quality_noops_not_floor_merges():
    tomo = scene(lambda x, y: abs(y) <= 1., layers=2)
    path = line()
    layers = np.zeros(len(path), int)
    layers[len(path)//2:] = 1
    result = center_corridor(tomo, path, layers)
    assert not result['applied'] and result['reason'] == 'multi_layer_path_not_applicable'
    assert route_clearance_quality(tomo, path, layers)['available']
    ramp = line(y=0., begin=-1., end=1., count=21)
    ramp[:, 2] = .4*(ramp[:, 0]+1.)
    for point in ramp:
        i, j = tomo.index(point[:2])
        tomo.ground[0, i, :] = point[2]
    result = center_corridor(tomo, ramp, [0]*len(ramp))
    assert not result['applied'] and result['reason'] == 'nonfloor_height_profile_not_applicable'


def test_two_layer_lru_does_not_merge_same_xy_floors_and_evicts_bounded_storage():
    tomo = scene(layers=3)
    tomo.cost[1, :, 80] = 50.
    cache = ClearanceFieldCache(tomo)
    first = cache.get(0)
    second = cache.get(1)
    assert first[80, 80] > 2.
    assert second[80, 80] == 0.
    assert not first.flags.writeable and not second.flags.writeable
    assert cache.get(0) is first
    cache.get(2)
    assert cache.stats['entries'] == 2 and cache.stats['evictions'] == 1
    assert cache.stats['hits'] == 1
    assert cache.stats['field_bytes'] == 2*161*161*4
    assert cache.stats['distance_semantics'] == DISTANCE_SEMANTICS


@pytest.mark.parametrize('change', ['cost', 'mask', 'headroom', 'ground'])
def test_mutated_map_mask_invalidates_cached_field(change):
    tomo = scene()
    cache = ClearanceFieldCache(tomo)
    before = cache.get(0)
    if change == 'cost':
        tomo.cost[0, 80, 80] = 50.
    elif change == 'mask':
        tomo.valid[0, 80, 80] = False
    elif change == 'headroom':
        tomo.headroom[0, 80, 80] = .1
    else:
        tomo.ground[0, 80, 80] = np.nan
        tomo.ground_known[0, 80, 80] = False
    after = cache.get(0)
    assert after is not before and after[80, 80] == 0.
    assert cache.stats['invalidations'] == 1
    assert cache.stats['misses'] == 2


def test_cache_cannot_be_reused_for_different_map_and_invalid_capacity_is_rejected():
    tomo = scene()
    for count in [0, -1, True, 2.5, 17]:
        with pytest.raises(ValueError):
            ClearanceFieldCache(tomo, count)
    with pytest.raises(ValueError):
        center_corridor(scene(), line(), [0]*101, cache=ClearanceFieldCache(tomo))


def test_cache_rejects_array_shape_change_instead_of_broadcasting_a_stale_field():
    tomo = scene()
    cache = ClearanceFieldCache(tomo)
    cache.get(0)
    tomo.valid = tomo.valid[:, :1]
    with pytest.raises(ValueError, match='shape_changed'):
        cache.get(0)


def test_new_map_obstacle_after_field_creation_is_checked_by_full_candidate_validator(monkeypatch):
    tomo = scene(lambda x, y: abs(y) <= 1.)
    cache = ClearanceFieldCache(tomo)
    cache.get(0)
    original = centering._expanded_candidate
    def changed_map(*args, **kwargs):
        # The input/section search already completed.  A wall spanning this
        # corridor must reject all fresh cubic candidates, not only their knots.
        tomo.cost[0, 81, :] = 50.
        return original(*args, **kwargs)
    monkeypatch.setattr(centering, '_expanded_candidate', changed_map)
    result = center_corridor(tomo, line(), [0]*101, cache)
    assert not result['applied'] and 'path' not in result
    assert result['reason'] == 'no_valid_clearance_improvement'
    assert len(result['candidate_failures']) == 3


def test_production_route_output_cap_is_enforced_before_accepting_quality_candidate(monkeypatch):
    tomo = scene(lambda x, y: abs(y) <= 1.)
    monkeypatch.setattr(centering, 'MAX_OUTPUT_POINTS', 2)
    result = center_corridor(tomo, line(), [0]*101)
    assert not result['applied'] and 'path' not in result
    assert all(item['reason'] == 'published_point_limit' for item in result['candidate_failures'])


def test_quality_objective_is_resampling_invariant_and_measures_exposure_by_arclength():
    tomo = scene(lambda x, y: abs(y) <= 1.)
    sparse = line(count=2)
    dense = line(count=1001)
    cache = ClearanceFieldCache(tomo)
    a = route_clearance_quality(tomo, sparse, [0]*len(sparse), cache)
    b = route_clearance_quality(tomo, dense, [0]*len(dense), cache)
    assert a['available'] and b['available']
    assert a['score'] == pytest.approx(b['score'], abs=1e-10)
    assert a['clearance_mean_m'] == pytest.approx(b['clearance_mean_m'], abs=1e-10)
    assert a['interior_turn'] == b['interior_turn']
    centre = route_clearance_quality(tomo, line(y=0.), [0]*101, cache)
    assert centre['score'] < a['score']
    assert cache.stats['hits'] == 2


def test_duplicate_vertices_and_selected_endpoint_z_are_preserved():
    tomo = scene(lambda x, y: abs(y) <= 1.)
    path = line()
    path[0, 2], path[-1, 2] = .03, -.02
    path = np.vstack((path[0], path, path[-1]))
    result = center_corridor(tomo, path, [0]*len(path))
    assert result['applied'], result
    np.testing.assert_array_equal(np.asarray(result['path'])[[0, -1]], path[[0, -1]])


def test_cubic_coefficients_are_exact_scipy_station_polynomials_and_xy_c2():
    stations = np.array([0., .7, 1.8, 3.])
    xy = np.array([[0., 0.], [.7, .1], [1.8, -.2], [3., 0.]])
    segments = _cubic_segments(stations, xy)
    for index, segment in enumerate(segments):
        c = segment['coefficients_xy']
        np.testing.assert_allclose(np.polynomial.polynomial.polyval(0., c), xy[index], atol=1e-12)
        np.testing.assert_allclose(np.polynomial.polynomial.polyval(1., c), xy[index+1], atol=1e-12)
        if index:
            previous = segments[index-1]['coefficients_xy']
            for order in (1, 2):
                left = np.polynomial.polynomial.polyval(1., np.polynomial.polynomial.polyder(previous, m=order, axis=0)) / (stations[index]-stations[index-1])**order
                right = np.polynomial.polynomial.polyval(0., np.polynomial.polynomial.polyder(c, m=order, axis=0)) / (stations[index+1]-stations[index])**order
                np.testing.assert_allclose(left, right, atol=1e-12)


def test_invalid_input_and_work_limits_do_not_publish_a_path():
    tomo = scene()
    for path, layers in [([[0., 0., 0.], [np.nan, 0., 0.]], [0, 0]),
                         ([[0., 0., 0.], [1., 0., 0.]], [0, 1.5]),
                         ([[0., 0., 0.], [2000., 0., 0.]], [0, 0])]:
        result = center_corridor(tomo, path, layers)
        assert not result['applied'] and 'path' not in result


def test_local_retreat_keeps_obstacle_and_centres_unaffected_corridor_spans():
    # All globally shifted curves hit this narrow wall notch. Retreat locally
    # toward the original legal path instead of discarding the whole route.
    tomo = scene(lambda x, y: (abs(y) <= 1.) & ~((abs(x) <= .15) & (y > -.5)))
    original_map = tomo.data.copy()
    stations = np.linspace(0., 10., 21)
    xy = np.c_[stations-5., np.full(len(stations), -.6)]
    offsets = np.full(len(stations), .5)
    offsets[[0, -1]] = 0.
    normal = np.tile([0., 1.], (len(stations), 1))
    start, goal = np.r_[xy[0], 0.], np.r_[xy[-1], 0.]
    segments, details = centering._locally_repaired_segments(
        tomo, stations, xy, offsets, normal, 0, start, goal)
    assert segments is not None and details['applied'], details
    assert details['rounds'][0]['blocked_spans']
    assert not details['rounds'][-1]['blocked_spans']
    assert 0 < details['modified_anchor_count'] < len(stations)
    output, _, _ = centering._expanded_candidate(tomo, segments, 0, start, goal)
    tomo.validate_path(output, np.zeros(len(output), int))
    # Unaffected sections keep the useful centering, with exact endpoints and
    # no map clearing, relaxed threshold or synthetic free-space connector.
    far = output[(abs(output[:, 0]) > 2.8) & (abs(output[:, 0]) < 3.5)]
    assert np.min(far[:, 1]) > -.2
    near = output[abs(output[:, 0]) < .15]
    assert np.max(near[:, 1]) < -.5
    np.testing.assert_array_equal(output[[0, -1]], [start, goal])
    np.testing.assert_array_equal(tomo.data, original_map)
    assert tomo.cost[0, 80, 80] == 50.


def test_local_retreat_checks_blockage_between_anchors_and_never_returns_unchecked_curve():
    tomo = scene(lambda x, y: (abs(y) <= 1.) & ~((abs(x-.2) < .06) & (y > -.5)))
    stations = np.linspace(0., 10., 21)
    xy = np.c_[stations-5., np.full(len(stations), -.6)]
    offsets = np.full(len(stations), .5)
    offsets[[0, -1]] = 0.
    normal = np.tile([0., 1.], (len(stations), 1))
    # None of the knots lies on the blocking cell at x=.2. A knot-only check
    # would wrongly admit the unmodified curve.
    for point in xy+offsets[:, None]*normal:
        tomo.surface_ground_z(point, 0)
    start, goal = np.r_[xy[0], 0.], np.r_[xy[-1], 0.]
    segments, details = centering._locally_repaired_segments(
        tomo, stations, xy, offsets, normal, 0, start, goal)
    assert details['rounds'][0]['blocked_spans']
    assert segments is not None
    output, _, _ = centering._expanded_candidate(tomo, segments, 0, start, goal)
    tomo.validate_path(output, np.zeros(len(output), int))


def test_unrepairable_map_conflict_has_bounded_work_and_no_candidate():
    tomo = scene(lambda x, y: (abs(y) <= 1.) & (abs(x) > .15))
    stations = np.linspace(0., 10., 21)
    xy = np.c_[stations-5., np.full(len(stations), -.6)]
    offsets = np.full(len(stations), .5)
    offsets[[0, -1]] = 0.
    normal = np.tile([0., 1.], (len(stations), 1))
    segments, details = centering._locally_repaired_segments(
        tomo, stations, xy, offsets, normal, 0, np.r_[xy[0], 0.], np.r_[xy[-1], 0.])
    assert segments is None and not details['applied']
    assert details['reason'] == 'local_offsets_cannot_repair_curve'
    assert len(details['rounds']) <= centering.MAX_LOCAL_REPAIR_ROUNDS+1
    assert all(item['blocked_spans'] for item in details['rounds'])


def test_previously_valid_centering_never_invokes_local_repair(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('Do not perturb previously successful candidate selection')
    monkeypatch.setattr(centering, '_locally_repaired_segments', forbidden)
    tomo = scene(lambda x, y: abs(y) <= 1.)
    result = center_corridor(tomo, line(), [0]*101)
    assert result['applied'] and result['blend'] in (1., .7, .4)
    assert result['local_repair'] is None


@pytest.mark.parametrize('single_wall', [False, True])
def test_short_wall_side_route_backtracks_amplitude_not_collision_or_curvature(single_wall):
    tomo = scene(lambda x, y: y >= -1. if single_wall else abs(y) <= 1.)
    path = line(y=-.85, begin=-1., end=1.)
    data = tomo.data.copy()
    result = center_corridor(tomo, path, [0]*len(path))
    assert result['applied'] and 0 < result['blend'] < .4, result
    # The old three coarse candidates genuinely failed; this is not a mock
    # scorer or a loosened hard limit admitting a formerly invalid curve.
    assert all(item['reason'] == 'analytic_curvature_limit'
               for item in result['candidate_failures'][:3])
    assert result['analytic_curvature_peak_per_m'] <= 1.25
    assert result['after_quality']['clearance_mean_m'] > result['before_quality']['clearance_mean_m']+.04
    assert result['score_after'] < result['score_before']-.01
    output = np.asarray(result['path'])
    np.testing.assert_array_equal(output[[0,-1]], path[[0,-1]])
    np.testing.assert_array_equal(tomo.data, data)
    tomo.validate_path(output, result['layer_ids'])


def test_short_centred_route_is_unchanged_and_stair_quality_policy_not_extended():
    tomo = scene(lambda x, y: abs(y) <= 1.)
    path = line(y=0., begin=-1., end=1.)
    result = center_corridor(tomo, path, [0]*len(path))
    assert not result['applied'] and result['reason'] == 'no_supported_centering_shift'
    path = line(y=-.85, begin=-1., end=1.)
    stairs = center_corridor(tomo, path, [0]*len(path), allow_graded_support=True)
    assert not stairs['applied']
    assert [item['blend'] for item in stairs['candidate_failures']] == [1., .7, .4]
