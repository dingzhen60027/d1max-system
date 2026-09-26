"""Layer ownership cannot flatten a staircase or merge vertically overlapping floors."""
import copy
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from pointcloud_preprocessing import multifloor_runner as runner


def fixture(monkeypatch):
    def condition(points, poses, config):
        assert not np.any((points[:, 0] >= .9) & (points[:, 0] <= 1.1))
        moved = points.copy()
        moved[:, 2] += .01
        return {'xyz': moved, 'keep_mask': np.ones(len(points), bool),
                'conditioning_mask': np.ones(len(points), bool), 'statistics': {},
                'floor_z': points[:, 2], 'floor_support_mask': np.ones(len(points), bool),
                'anchors': {'xy': poses[:, :2], 'z': poses[:, 2]}}
    monkeypatch.setattr(runner, 'condition_flat_floor', condition)
    monkeypatch.setattr(runner, 'structural_cleanup', lambda *_: None)
    points = np.array([[0., 0., -.6], [0., 0., 3.1], [1., 0., 1.3], [1., 0., -.5], [1., 0., 3.1]])
    poses = np.array([[0., 0., 0.], [.2, 0., 0.], [0., 0., 3.6], [.2, 0., 3.6]])
    cfg = {'protected_regions': [{'min': [.9, -.1, -1.], 'max': [1.1, .1, 4.]}],
           'floors': [dict(id='lower', input_z_band=[-1., 1.], trajectory_range_inclusive=[0, 1], reference_z_m=-.6),
                      dict(id='upper', input_z_band=[2., 4.], trajectory_range_inclusive=[2, 3], reference_z_m=3.1)],
           'maximum_z_shift_m': .1}
    return points, poses, cfg, {'conditioning': {}, 'obstacle_cleanup': {}}


def test_same_xy_floors_keep_separate_z_and_stair_records_unchanged(monkeypatch):
    points, poses, cfg, defaults = fixture(monkeypatch)
    before = points.copy()
    result, keep, owner, protected, _, _ = runner.condition_levels(points, poses, cfg, defaults)
    np.testing.assert_array_equal(points, before)
    np.testing.assert_array_equal(result[:, :2], before[:, :2])
    np.testing.assert_array_equal(result[protected], before[protected])
    assert keep[protected].all()
    assert owner.tolist() == [0, 1, -1, -1, -1]
    assert result[1, 2] - result[0, 2] == pytest.approx(3.7)


def test_overlapping_ownership_bands_are_rejected(monkeypatch):
    points, poses, cfg, defaults = fixture(monkeypatch)
    cfg['floors'][1]['input_z_band'] = [.5, 4.]
    with pytest.raises(ValueError, match='must not overlap'):
        runner.condition_levels(points, poses, cfg, defaults)


def test_bounded_floor_shift_cannot_hide_large_mapping_error(monkeypatch):
    points, poses, cfg, defaults = fixture(monkeypatch)
    cfg['maximum_z_shift_m'] = .005
    with pytest.raises(ValueError, match='exceeds'):
        runner.condition_levels(points, poses, cfg, defaults)
