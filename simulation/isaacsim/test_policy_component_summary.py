import math
import pytest
from policy_component_summary import straight_demand_diagnostics


def rows():
    return [dict(source_ns=i*2_000_000,highlevel_command=[.25 if i<5 else 0.,0.],
        position=[i*.001,i*.0001,0.]) for i in range(11)]


def test_source_begin_demand_integral_and_true_zero_preserve_hold_semantics():
    result=straight_demand_diagnostics(rows())
    assert result['terminal']['demand_integral_m']==.0025
    assert result['terminal']['actual_along_path_m']==.01
    assert result['terminal']['signed_integral_error_m']==.0075
    assert result['peak_absolute_spatial_lateral_error_m']==.001
    assert not result['formal_tracking_acceptance']


def test_along_error_is_separate_from_spatial_side_budget():
    values=rows();values[-1]['position']=[.25,.01,100.]
    result=straight_demand_diagnostics(values)
    assert not result['original_dot10_time_integral_budget_met']
    assert result['original_dot10_spatial_side_budget_met']
    assert result['peak_integral_error_witness']['source_ns']==20_000_000


def test_requested_world_heading_is_explicit_and_missing_source_rejected():
    result=straight_demand_diagnostics(rows(),math.pi/2)
    assert result['terminal']['actual_along_path_m']==pytest.approx(.001)
    assert result['peak_absolute_spatial_lateral_error_m']==pytest.approx(.01)
    with pytest.raises(ValueError):straight_demand_diagnostics(rows()[:2]+rows()[3:])
