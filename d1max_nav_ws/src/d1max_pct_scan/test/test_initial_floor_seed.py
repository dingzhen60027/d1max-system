from types import SimpleNamespace
import numpy as np
import pytest
from d1max_pct_scan.live_ui_contract import floor_initial_body_z


def bridge():
    def floor(z):
        return SimpleNamespace(query=lambda xy, limits: (np.array([z+.01*xy[0, 0]]), np.array([.02])))
    return SimpleNamespace(floors={'floor1': floor(-.6), 'floor2': floor(3.)},
                           limits=object(), protected_regions=[])


def test_same_xy_selected_floor_changes_height_in_original_map_only():
    value = bridge()
    assert floor_initial_body_z(value, 'floor1', (2., 3.), .55) == pytest.approx(-.03)
    assert floor_initial_body_z(value, 'floor2', (2., 3.), .55) == pytest.approx(3.57)


def test_height_is_applied_exactly_once_and_xy_field_is_not_flattened():
    value = bridge()
    assert floor_initial_body_z(value, 'floor2', (4., 3.), .50) == pytest.approx(3.54)


def test_missing_floor_or_source_support_does_not_fallback_to_first_floor():
    value = bridge()
    with pytest.raises(ValueError):
        floor_initial_body_z(value, 'floor3', (0., 0.), .55)
    def no_support(*args):
        raise ValueError('unobserved_source_floor')
    value.floors['floor2'].query = no_support
    with pytest.raises(ValueError, match='unobserved_source_floor'):
        floor_initial_body_z(value, 'floor2', (0., 0.), .55)


def test_stair_region_is_not_reseeded_with_a_guessed_level_floor():
    value = bridge()
    value.protected_regions = [dict(min=[-1., -1., -1.], max=[1., 1., 4.])]
    with pytest.raises(ValueError, match='楼梯'):
        floor_initial_body_z(value, 'floor1', (0., 0.), .55)
