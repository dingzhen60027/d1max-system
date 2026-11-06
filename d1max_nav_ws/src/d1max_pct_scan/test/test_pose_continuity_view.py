from copy import deepcopy
import pytest
from d1max_pct_scan.pose_continuity_view import PoseContinuityView

IDENTITY=('s',2,'seed')

def sample():
    return dict(schema=1,sequence=1,session_id='s',map_version_id='map',localization_epoch=2,
        localization_seed_id='seed',frame_id='d1max_loc_map',body_frame='d1max_loc_base_link',
        available=True,motion_usable=False,quality='imu_gap_coasting_pose_only',
        estimate_stamp_ns=100_360_000_000,posterior_stamp_ns=100_000_000_000,
        imu_stamp_ns=100_000_000_000,valid_until_ns=100_500_000_000,
        position=[.108,0.,.55],orientation=[0.,0.,0.,1.],pose_covariance=[.1 if i%7==0 else 0. for i in range(36)])

def observe(c,s=None):
    return c.observe(sample() if s is None else s,identity=IDENTITY,now_ns=100_360_000_000,monotonic=1.)

def test_position_lease_expires_at_original_source_bound_no_permission():
    c=PoseContinuityView('s','map');assert observe(c)
    assert c.lease(identity=IDENTITY,now_ns=100_360_000_000,monotonic=1.)==pytest.approx(.08)
    assert c.lease(identity=IDENTITY,now_ns=100_500_000_000,monotonic=1.1)==0.
    assert c.sample['motion_usable'] is False

@pytest.mark.parametrize('change',[{'motion_usable':True},{'localization_epoch':3},{'session_id':'other'},
    {'map_version_id':'other'},{'valid_until_ns':100_750_000_000},{'orientation':[0.,0.,0.,0.]},
    {'estimate_stamp_ns':100_361_000_000},{'posterior_stamp_ns':99_000_000_000}])
def test_foreign_expired_or_unsafe_model_is_not_a_display_sample(change):
    c=PoseContinuityView('s','map');v=sample();v.update(change)
    assert not observe(c,v) and c.sample is None

def test_hard_unavailable_clears_display_and_late_old_result_cannot_revive():
    c=PoseContinuityView('s','map');assert observe(c)
    stop=sample();stop.update(sequence=2,available=False,localization_epoch=3,localization_seed_id='')
    assert observe(c,stop) and c.sample is None
    assert not observe(c) and c.sample is None

def test_identity_fault_withdrawal_blocks_late_positive_until_real_typed_pose():
    c=PoseContinuityView('s','map');assert observe(c)
    c.withdraw()
    late=sample();late['sequence']=2
    assert not observe(c,late) and c.sample is None
    c.bind_real_pose()
    assert observe(c,late)
