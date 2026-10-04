import pytest
from d1max_pct_scan.perception_contract import acquisition_budget, MAX_ACQUISITION_POINTS
from d1max_pct_scan.ray_projection import Limits


def test_safety_range_is_independent_of_registration_range():
    value=acquisition_budget(dict(min_range=.5,max_range=60.),MAX_ACQUISITION_POINTS)
    assert value['min_range']==0. and value['max_range']==1000.
    assert value['return_semantics']=='finite_measured_hits_only_invalid_unknown'


@pytest.mark.parametrize('cap,projector',[(250000,100000),(100000,250000),(10,20),(True,1)])
def test_budget_mismatch_fails_before_projection(cap,projector):
    with pytest.raises(ValueError,match='budget_mismatch'):
        acquisition_budget({'perception_rays.max_input_points':cap},projector)


def test_projection_limit_never_exceeds_native_decoder_capacity():
    assert Limits().max_input_points==100000
    with pytest.raises(ValueError):Limits(max_input_points=100001)
