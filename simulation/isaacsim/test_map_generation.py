"""Large map inputs remain metric, explicit and bounded."""
import pytest

from map_builder import generation_settings


def test_small_map_compatibility():
    spacing, resolution, limits = generation_settings({})
    assert (spacing, resolution) == (.05, .10)
    assert limits['max_total_cells'] == 1000000


def test_large_map_budget_is_explicit():
    spacing, resolution, limits = generation_settings({'map_generation': {
        'surface_spacing_m': .10, 'pct_resolution_m': .10,
        'pct_max_total_cells': 10000000, 'pct_max_working_bytes': 2000000000}})
    assert (spacing, resolution) == (.10, .10)
    assert limits['max_total_cells'] == 10000000


@pytest.mark.parametrize('field,value', [
    ('surface_spacing_m', float('nan')), ('surface_spacing_m', 1.),
    ('pct_resolution_m', True), ('pct_resolution_m', .01),
    ('pct_max_total_cells', 25000001), ('pct_max_working_bytes', 2500000001),
    ('pct_max_total_cells', 1.5), ('pct_max_working_bytes', 0)])
def test_unsafe_generation_requests_rejected(field, value):
    with pytest.raises(ValueError):
        generation_settings({'map_generation': {field: value}})
