"""An explicit static-map prior never becomes a generic obstacle remover."""
import numpy as np
import pytest

from pointcloud_preprocessing import reviewed_flat_corridor as prior
from pointcloud_preprocessing.flat_floor import DEFAULTS


def review():
    return dict(schema=prior.SCHEMA, source_map_sha256='a' * 64,
                review=dict(confirmed_clear_flat_floor=True, statement='Reviewed clear flat floor',
                            scope='static_planning_map_only'),
                regions=[dict(id='central', floor_id='floor1',
                              polygon_xy=[[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]],
                              residual_min_m=.04, residual_max_m=.10)],
                stair_exclusion_xy=[[[5., 5.], [6., 5.], [6., 6.], [5., 6.]]])


def fixture(targets=None):
    seeds = [[x, y, 0.] for x in [-.06, 0., .06] for y in [-.06, .06]]
    points = np.array(seeds + (targets if targets is not None else [[0., 0., .09]]))
    count = len(points)
    result = dict(xyz=points.copy(), config={**DEFAULTS, 'reference_z_m': -.65},
                  floor_z=np.zeros(count), conditioning_mask=np.ones(count, bool),
                  keep_mask=np.ones(count, bool), floor_support_mask=np.arange(count) < 6,
                  floor_snap_mask=np.arange(count) < 6, statistics={})
    return points, result, np.zeros(count, bool)


def run(points, result, wall, config=None, floor='floor1'):
    return prior.apply(result, points, floor_id=floor, review=config or review(), wall_protected=wall)


def test_reviewed_duplicate_layer_corrected_without_inventing_raw_support():
    points, result, wall = fixture()
    original = points.copy()
    support = result['floor_support_mask'].copy()
    assert run(points, result, wall).tolist() == [False] * 6 + [True]
    assert result['xyz'][-1, 2] == -.65
    np.testing.assert_array_equal(points, original)
    np.testing.assert_array_equal(result['xyz'][:, :2], original[:, :2])
    np.testing.assert_array_equal(result['floor_support_mask'], support)
    assert result['floor_snap_mask'][-1]


@pytest.mark.parametrize('guard', ['wall', 'stairs', 'upper', 'other_floor', 'outside',
                                  'removed', 'unconditioned', 'too_high', 'too_low', 'no_lower',
                                  'linear_lower'])
def test_unchanged_structures_or_unproven_returns(guard):
    points, result, wall = fixture()
    config = review()
    floor = 'floor1'
    if guard == 'wall':
        wall[-1] = True
    elif guard == 'stairs':
        config['stair_exclusion_xy'] = [config['regions'][0]['polygon_xy']]
    elif guard == 'upper':
        points, result, wall = fixture([[0., 0., .09], [0., 0., .5], [.02, 0., .5], [0., .02, .5]])
    elif guard == 'other_floor':
        floor = 'floor2'
    elif guard == 'outside':
        config['regions'][0]['polygon_xy'] = [[2., 2.], [3., 2.], [3., 3.], [2., 3.]]
    elif guard == 'removed':
        result['keep_mask'][-1] = False
    elif guard == 'unconditioned':
        result['conditioning_mask'][-1] = False
    elif guard == 'too_high':
        points[-1, 2] = result['xyz'][-1, 2] = .1001
    elif guard == 'too_low':
        points[-1, 2] = result['xyz'][-1, 2] = .0399
    elif guard == 'no_lower':
        points[:6, 2] = .05  # still classified support, but not the lower band
    elif guard == 'linear_lower':
        points[:6, 1] = 0.
    before = result['xyz'].copy()
    assert not run(points, result, wall, config, floor).any()
    np.testing.assert_array_equal(result['xyz'], before)


def test_corrected_layer_cannot_propagate_support_outward():
    points, result, wall = fixture([[0., 0., .09], [.19, 0., .09]])
    assert run(points, result, wall).tolist()[-2:] == [True, False]


@pytest.mark.parametrize('mutation', ['source', 'scope', 'confirmation', 'polygon', 'stairs', 'bounds'])
def test_source_bound_explicit_review_is_required(mutation):
    config = review()
    if mutation == 'source':
        config['source_map_sha256'] = 'b' * 64
    elif mutation == 'scope':
        config['review']['scope'] = 'realtime'
    elif mutation == 'confirmation':
        config['review']['confirmed_clear_flat_floor'] = False
    elif mutation == 'polygon':
        config['regions'][0]['polygon_xy'][2] = [0., -2.]
    elif mutation == 'stairs':
        config['stair_exclusion_xy'] = []
    elif mutation == 'bounds':
        config['regions'][0]['residual_max_m'] = float('nan')
    with pytest.raises(ValueError):
        prior.validate(config, 'a' * 64)


def test_reversed_polygon_and_exact_source_are_valid():
    config = review()
    config['regions'][0]['polygon_xy'].reverse()
    assert prior.validate(config, 'a' * 64) is config
    points, result, wall = fixture()
    assert run(points, result, wall, config)[-1]


def test_bounds_cannot_expand_past_existing_near_ground_model():
    points, result, wall = fixture()
    config = review()
    config['regions'][0]['residual_max_m'] = .5
    with pytest.raises(ValueError, match='existing low-return bounds'):
        run(points, result, wall, config)


def test_reviewed_band_can_meet_existing_snap_boundary_without_a_gap():
    points, result, wall = fixture([[0., 0., .039777], [0., .01, .035], [0., -.01, .03499]])
    config = review()
    config['regions'][0]['residual_min_m'] = result['config']['floor_snap_halfwidth_m']
    assert run(points, result, wall, config).tolist()[-3:] == [True, True, False]


def test_already_flat_and_overlapping_regions_do_not_double_count_change():
    points, result, wall = fixture([[0., 0., .09], [.01, .01, .09]])
    result['xyz'][-1, 2] = -.65
    config = review()
    config['regions'].append({**config['regions'][0], 'id': 'overlap'})
    assert run(points, result, wall, config).tolist()[-2:] == [True, False]
    assert [row['changed'] for row in result['statistics']['reviewed_flat_corridors']] == [1, 0]
