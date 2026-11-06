"""Same MC evidence and guarded metric used by SDK, BT and legacy handoff."""
import math
from types import SimpleNamespace
import pytest
from d1max_pct_scan.execution_timing import stationary_velocity_within, validate_stationary
from test_sdk_input_mapping import profile

def witness():
    xy=math.hypot(.028366,.001744);z=.013489
    return SimpleNamespace(velocity_metric='planar_xy_with_vertical_guard_v1',
        measured_linear_mps=math.hypot(xy,z),measured_planar_mps=xy,
        measured_vertical_mps=z,measured_angular_radps=.005)

def test_original_3d_norm_is_retained_and_policy_explicit():
    p=validate_stationary(profile());e=witness()
    assert e.measured_linear_mps>.03 and stationary_velocity_within(e,p)
    assert not stationary_velocity_within(e,dict(p,velocity_metric='spatial_norm_3d'))
    e.velocity_metric='spatial_norm_3d'
    assert not stationary_velocity_within(e,p)
    e.measured_linear_mps=.02
    assert stationary_velocity_within(e,p)
    assert profile()['physical_acceptance_verified'] is False

@pytest.mark.parametrize('field,value',[
    ('measured_planar_mps',.031),('measured_vertical_mps',.031),
    ('measured_vertical_mps',float('nan')),('measured_linear_mps',.001),
    ('measured_angular_radps',.051),('velocity_metric','ignore_z')])
def test_bad_stationary_evidence_does_not_pass(field,value):
    e=witness();setattr(e,field,value)
    if field in ('measured_planar_mps','measured_vertical_mps'):
        e.measured_linear_mps=math.hypot(e.measured_planar_mps,e.measured_vertical_mps)
    assert not stationary_velocity_within(e,validate_stationary(profile()))

def test_unknown_profile_metric_rejected():
    record=profile();record['stationary_evidence']['velocity_metric']='ignore_z'
    with pytest.raises(ValueError,match='velocity_metric'):validate_stationary(record)
