"""Explicit near-body preview suppression cannot become a motion self-filter."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml

from d1max_pct_scan.preview_ray_exclusion import from_session


def config():
    path = Path(__file__).resolve().parents[1] / 'config/live_visualization.yaml'
    return yaml.safe_load(path.read_text())


def test_near_body_mask_is_identical_for_both_sensors_and_bounded():
    mask = from_session(config(), 'd1max_loc_base_link')
    points = np.array([[0., 0., 0.], [.5, .3, -.45], [-.5, -.3, -.45],
                       [.60001, 0., 0.], [0., .35001, 0.], [0., 0., -.55001],
                       [0., 0., .40001], [1., 0., 0.]])
    expected = [True, True, True, False, False, False, False, False]
    for sensor in (0, 1):
        np.testing.assert_array_equal(mask.matches(points, sensor), expected)


def test_dangerous_counterexample_real_object_inside_is_hidden_only_in_preview():
    cfg = config()
    mask = from_session(cfg, 'd1max_loc_base_link')
    # A box/foot/person at the same coordinates cannot be classified as self.
    assert mask.matches(np.array([[.5, .3, -.45]]), 0)[0]
    for field, value in [('mode', 'LIVE_NAVIGATION'), ('motion_control_enabled', True),
                         ('motion', {}), ('perception_backend', 'deskewed_cloud')]:
        changed = deepcopy(cfg)
        changed[field] = value
        with pytest.raises(ValueError, match='no_motion'):
            from_session(changed, 'd1max_loc_base_link')


@pytest.mark.parametrize('change', ['scope', 'padding', 'remote', 'different', 'one_sensor',
                                   'verified', 'outside_x', 'outside_y', 'outside_z'])
def test_near_body_mask_cannot_silently_expand_or_claim_calibration(change):
    cfg = config()
    mask = cfg['preview_ray_exclusion']
    if change == 'scope':
        mask['scope'] = 'calibrated_self'
    elif change == 'padding':
        mask['pad_m'] = .01
    elif change == 'remote':
        mask['boxes'][0]['min_m'][0] = .1
    elif change == 'different':
        mask['boxes'][0]['max_m'][0] = .59
    elif change == 'one_sensor':
        mask['boxes'][1]['sensor_id'] = 0
    elif change == 'verified':
        mask['calibration_verified'] = True
    else:
        axis = {'outside_x': 0, 'outside_y': 1, 'outside_z': 2}[change]
        mask['boxes'][0]['max_m'][axis] = [0.65001, 0.40001, 0.55001][axis]
    with pytest.raises(ValueError):
        from_session(cfg, 'd1max_loc_base_link')


def test_motion_prepare_rejects_mask_before_any_preparation():
    from d1max_pct_scan.live_session import prepare_motion, DEFAULT_CONFIG
    with pytest.raises(ValueError, match='exclusion is forbidden'):
        prepare_motion(DEFAULT_CONFIG)
