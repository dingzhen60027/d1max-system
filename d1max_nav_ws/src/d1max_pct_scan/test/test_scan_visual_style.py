"""Pure upstream visual geometry checks, without ROS nodes or robot access."""
import math

import numpy as np
import pytest
from scipy.interpolate import BSpline

from d1max_pct_scan.scan_visual_style import official_spline_style


def fixture():
    return {'order': 3, 'knots': np.arange(11, dtype=float)*.2-0.6,
            'points': np.array([[0., 0., .6], [.1, .1, .6], [.3, .15, .62],
                                [.9, .6, .7], [1.8, .8, .8],
                                [2., .6, 1.], [2.1, .5, 1.]])}


def test_matches_upstream_sampling_derivative_and_yellow_red_palette():
    raw = fixture()
    style = official_spline_style(**raw)
    spline = BSpline(raw['knots'], raw['points'], 3)
    begin, end = raw['knots'][3], raw['knots'][len(raw['points'])]
    count = max(2, math.ceil((end-begin)/.1)+1)
    times = np.linspace(begin, end, count)
    speed = np.linalg.norm(spline.derivative()(times), axis=1)
    np.testing.assert_allclose(style['points'], spline(times))
    np.testing.assert_allclose(style['speeds'], speed)
    np.testing.assert_allclose(style['times'], times-begin)
    np.testing.assert_allclose(style['colors'][:, 1],
                               1.-(speed-speed.min())/max(1e-6, speed.max()-speed.min()))
    assert style['line_width'] == .08 and style['point_diameter'] == .08
    assert tuple(style['colors'][speed.argmin()]) == (1., 1., 0., 1.)
    assert tuple(style['colors'][speed.argmax()]) == (1., 0., 0., 1.)
    assert np.isfinite(style['colors']).all()


def test_constant_speed_is_yellow_and_input_is_not_mutated():
    raw = fixture()
    raw['points'] = np.array([[i*.2, 0., 1.] for i in range(7)])
    original_points, original_knots = raw['points'].copy(), raw['knots'].copy()
    style = official_spline_style(**raw)
    np.testing.assert_allclose(style['speeds'], 1.)
    np.testing.assert_allclose(style['colors'], np.tile([1., 1., 0., 1.], (len(style['colors']), 1)), atol=1e-8)
    np.testing.assert_array_equal(raw['points'], original_points)
    np.testing.assert_array_equal(raw['knots'], original_knots)


def test_stationary_native_spline_is_finite_yellow_not_a_fake_path():
    raw = fixture()
    raw['points'][:] = [1.5, 2.3, .51]
    style = official_spline_style(**raw)
    np.testing.assert_allclose(style['points'], np.tile([1.5, 2.3, .51], (len(style['points']), 1)))
    np.testing.assert_allclose(style['speeds'], 0., atol=1e-12)
    np.testing.assert_allclose(style['colors'][:, 1], 1., atol=1e-8)


@pytest.mark.parametrize('field,value', [
    ('order', 2), ('order', True), ('points', [[0., 0., 0.]]),
    ('points', [[math.nan, 0., 0.]]*7), ('knots', list(range(10))),
    ('knots', [0.]*11), ('knots', [math.inf]*11),
])
def test_invalid_geometry_is_not_misrendered(field, value):
    raw = fixture()
    raw[field] = value
    with pytest.raises(ValueError, match='invalid_native_cubic_spline'):
        official_spline_style(**raw)


@pytest.mark.parametrize('step', [.001, 31.])
def test_rendering_budget_is_bounded(step):
    raw = fixture()
    raw['knots'] = np.arange(11)*step
    with pytest.raises(ValueError, match='invalid_native_spline_duration'):
        official_spline_style(**raw)
