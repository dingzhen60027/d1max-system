import math
import pytest
from d1max_localization.lio_fusion import LioMapState, LioLimits
from d1max_localization.math_utils import Pose3, compose

def pose(x=0.,y=0.,yaw=0.):
    return Pose3((x,y,0.),(0.,0.,math.sin(yaw/2),math.cos(yaw/2)))
def ready():
    s=LioMapState();assert s.push_local(100.,pose(),100.)
    s.seed(pose(5.,2.),100.)
    assert s.push_local(100.1,pose(),100.1)
    assert s.accept(100.1,pose(5.,2.),100.1)
    return s

def test_stationary_lio_does_not_integrate_mc_velocity_bias():
    s=ready()
    for i in range(1,500):
        t=100.1+i*.1
        assert s.push_local(t,pose(),t)
        assert s.prediction(t).position==pytest.approx((5.,2.,0.))
        if i%3==0:assert s.accept(t,pose(5.,2.),t)
    assert s.tracking(t)

def test_two_meters_per_second_and_fast_turn_prediction_at_scan_time():
    s=ready();anchor=s.anchor
    for i in range(1,21):
        t=100.1+i*.1;p=pose(i*.2,0.,i*.1)
        assert s.push_local(t,p,t)
        expected=compose(anchor,p)
        assert s.prediction(t).position==pytest.approx(expected.position)
        assert s.prediction(t).orientation==pytest.approx(expected.orientation)
        assert s.accept(t,expected,t)
    assert s.tracking(t)

def test_delayed_correction_is_transported_without_restamping_pose():
    s=ready()
    for i in range(1,6):s.push_local(100.1+i*.1,pose(i*.2),100.1+i*.1)
    assert s.accept(100.3,pose(5.5,2.),100.6)
    assert s.prediction(100.6).position==pytest.approx((6.1,2.,0.))
    assert s.last_correction==100.3

def test_no_extrapolation_or_latest_pose_substitution():
    s=ready()
    assert s.prediction(99.) is None
    assert s.prediction(100.11) is None
    assert not s.accept(100.11,pose(5.,2.),100.11)

def test_future_rejected_message_does_not_poison_sequence():
    s=ready();s.push_local(100.2,pose(),100.2)
    assert not s.accept(500.,pose(5.,2.),100.2)
    assert s.accept(100.2,pose(5.,2.),100.2)

def test_stale_correction_revokes_tracking_even_with_fresh_lio():
    s=ready()
    for i in range(1,12):s.push_local(100.1+i*.1,pose(),100.1+i*.1)
    assert not s.tracking(101.2)

def test_recovery_is_bounded_and_does_not_grant_tracking():
    s=ready()
    for i in range(1,150):
        t=100.1+i*.1;s.push_local(t,pose(),t)
        result=s.begin_recovery(t)
        if result is not None:
            assert not s.tracking(t)
            assert result.position==pytest.approx((5.,2.,0.))
    assert s.recovery_count==3
    assert s.begin_recovery(t) is None

def test_recovery_needs_in_range_verified_measurement():
    s=ready()
    for i in range(1,13):s.push_local(100.1+i*.1,pose(),100.1+i*.1)
    assert s.begin_recovery(101.3) is not None
    s.push_local(101.4,pose(),101.4)
    assert not s.accept(101.4,pose(7.,2.),101.4)
    s.push_local(101.5,pose(),101.5)
    assert s.accept(101.5,pose(5.8,2.),101.5)
    assert s.tracking(101.5) and not s.recovery

@pytest.mark.parametrize('t,p', [(99.,pose()),(100.2,pose(2.)),(100.2,pose(yaw=2.))])
def test_lio_reset_or_jump_latches_fault(t,p):
    s=ready();assert not s.push_local(t,p,100.2 if t<100 else t)
    # A stale packet rejected by age is harmless; an admitted time reversal/jump faults.
    if t>100:assert s.fault
    assert not s.tracking(101.)

def test_large_future_timestamps_never_admitted():
    s=LioMapState();assert not s.push_local(100.2,pose(),100.)
    assert not s.local

@pytest.mark.parametrize('kwargs',[{'max_speed':-1},{'recovery_attempts':0},{'local_timeout':float('nan')}])
def test_invalid_limits_fail_early(kwargs):
    with pytest.raises(ValueError):LioMapState(LioLimits(**kwargs))

def test_map_covariance_rotates_xy_with_map_alignment():
    from d1max_localization.lio_fusion import rotate_pose_covariance
    import numpy as np
    cov=np.diag([1.,4.,9.,.1,.4,.9])
    out=np.array(rotate_pose_covariance(cov,pose(yaw=math.pi/2).orientation)).reshape(6,6)
    assert np.diag(out)==pytest.approx([4.,1.,9.,.4,.1,.9])
    assert np.linalg.eigvalsh(out).min()>0

@pytest.mark.parametrize('attempts',[1.5,11])
def test_recovery_budget_is_small_integer(attempts):
    with pytest.raises(ValueError):LioMapState(LioLimits(recovery_attempts=attempts))

def test_recovery_requires_current_seed_generation_and_confirmations():
    previous={'schema':1,'seed_ns':'1000000000','confirmations':3}
    assert not LioMapState.current_verification(previous,'2000000000')
    assert not LioMapState.current_verification(previous,None)
    current={**previous,'seed_ns':'2000000000','confirmations':1}
    assert not LioMapState.current_verification(current,'2000000000')
    assert LioMapState.current_verification({**current,'confirmations':3},'2000000000')

def test_local_epoch_reset_revokes_map_alignment_and_needs_new_seed():
    s=LioMapState();assert s.accept_epoch(1,True)
    assert s.push_local(100.,pose(),100.);s.seed(pose(5.,2.),100.)
    assert s.push_local(100.1,pose(),100.1);assert s.accept(100.1,pose(5.,2.),100.1)
    assert s.tracking(100.1)
    assert s.accept_epoch(1,False);assert not s.tracking(100.1)
    s.fault='imu_gap';assert s.accept_epoch(2,False)
    assert s.anchor is None and s.seed_time is None and not s.local and s.fault is None
    assert s.accept_epoch(2,True);assert s.push_local(101.,pose(),101.)
    assert not s.tracking(101.) and s.prediction(101.) is None
    assert not s.accept(101.,pose(5.,2.),101.)

def test_old_epoch_and_same_epoch_cannot_clear_a_fault():
    s=LioMapState();assert s.accept_epoch(2,False);s.fault='imu_clock_reset'
    assert not s.accept_epoch(1,True)
    assert s.accept_epoch(2,True);assert s.fault=='imu_clock_reset'
    for epoch in (True,0,-1,1.5,2**64):assert not s.accept_epoch(epoch,True)
