"""Compact offline preset reduces extra cost spread, not terrain classification."""
from pathlib import Path

import numpy as np
import yaml

from d1max_pct_planner.cpu_tomography import _inflate


def test_compact_preset_changes_only_extra_margin_in_pct_parameters():
    directory = Path(__file__).resolve().parents[1] / 'config'
    old = yaml.safe_load((directory / 'official_cleaned_single_floor.yaml').read_text())
    new = yaml.safe_load((directory / 'official_cleaned_compact.yaml').read_text())
    assert old['pct']['traversability']['safe_margin'] == .4
    assert new['pct']['traversability']['safe_margin'] == .2
    old['pct']['traversability']['safe_margin'] = .2
    assert new['pct'] == old['pct']
    for key in ('source_pcd', 'preprocessing', 'frame_id', 'export'):
        assert new[key] == old[key]


def test_compact_inflation_keeps_obstacle_core_and_reduces_only_spread():
    raw = np.zeros((15, 15), dtype=np.float32)
    raw[7, 7] = 50.
    old = _inflate(raw, .2, .4, .2)
    new = _inflate(raw, .2, .2, .2)
    assert np.all(new <= old)
    assert (old > 20).sum() == 21
    assert (new > 20).sum() == 13
    assert (old > 0).sum() == 45
    assert (new > 0).sum() == 25
    assert new[7, 7] == new[7, 8] == new[8, 7] == 50.
    assert old[9, 8] > 20 and 0 < new[9, 8] <= 20
