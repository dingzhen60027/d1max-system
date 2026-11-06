"""A dropped lidar packet is not a posterior reset. No ROS graph or SDK."""
from copy import deepcopy
import math

import pytest

from d1max_localization.estimation.causal_prediction import CausalInertialPredictor
from d1max_localization.estimation.prediction import PredictionLimits
from test_navigation_estimation import snapshot


def prepared(epoch=1):
    core=CausalInertialPredictor(PredictionLimits(max_horizon=.35,max_coast=.1))
    value=snapshot(100.,epoch)
    value['inertial']['world_velocity']=[.3,0.,0.]
    assert core.accept(value,100.)
    for index in range(71):
        t=100.+index*.005
        assert core.push_imu(t,(0.,0.,9.81),(0.,0.,0.),t)
    return core


def stale_scan(now=100.31,**changes):
    # Exact metadata emitted by PublishRobustSample(false) after RobustSync
    # discard("stale_scan"); the rejected cloud produced no new posterior.
    value=dict(schema=1,epoch=1,valid=False,fault=False,reason='stale_scan',
        received_at_unix=now,imu_quality_scope='last_admitted_scan',
        imu_quality_scan_end_sec=100.,last_good_posterior_sec=100.,
        finite_filter_state=True,gap_coasting=False)
    value.update(changes)
    return value


def test_stale_scan_notice_keeps_same_real_posterior_not_a_new_estimate():
    core=prepared();control=prepared()
    original=deepcopy((core.snapshot,core.snapshot_publication,core.snapshot_receipt))
    for t in (100.30,100.32,100.34):
        assert core.accept(stale_scan(t),t)
        actual,expected=core.evaluate(t),control.evaluate(t)
        assert actual is not None
        assert actual==expected
        assert actual.pose.position[0]==pytest.approx(.3*(t-100.))
        assert actual.source_stamp==100.
        assert original==(core.snapshot,core.snapshot_publication,core.snapshot_receipt)
    assert core.pending_notices==3
    # No amount of pending notice traffic recharges the original deadline.
    for t in (100.351,100.36,100.50):
        assert core.accept(stale_scan(t),t)
        assert core.evaluate(t) is None
        assert original==(core.snapshot,core.snapshot_publication,core.snapshot_receipt)


@pytest.mark.parametrize('change',[
    {'epoch':2},{'fault':True},{'finite_filter_state':False},{'finite_filter_state':1},
    {'last_good_posterior_sec':100.001},{'last_good_posterior_sec':float('nan')},
    {'last_good_posterior_sec':'100'},{'last_good_posterior_sec':0.},
    {'imu_quality_scope':'unknown'},{'reason':'imu_gap'},{'reason':'initializing'},
    {'reason':'nonfinite_filter_state'},{'reason':'invalid_point_time'},
])
def test_unproven_drop_or_reset_never_preserves(change):
    core=prepared()
    assert core.accept(stale_scan(**change),100.31)
    assert core.evaluate(100.31) is None and core.pending_notices==0


def test_missing_source_identity_cannot_preserve():
    core=prepared();value=stale_scan();del value['last_good_posterior_sec']
    assert core.accept(value,100.31)
    assert core.evaluate(100.31) is None and core.pending_notices==0


def test_old_epoch_notice_is_ignored_without_poisoning_current_source():
    core=prepared(epoch=2);before=core.evaluate(100.31)
    assert not core.accept(stale_scan(),100.31)
    assert core.evaluate(100.31)==before and core.pending_notices==0


def test_bounded_notice_does_not_hide_imu_loss():
    core=prepared()
    while core.imu[-1][0]>100.20:core.imu.pop()
    assert core.accept(stale_scan(),100.31)
    assert core.evaluate(100.31) is None and core.reason=='imu_stale'


def test_fault_or_expired_posterior_cannot_be_revived_by_notice():
    core=prepared()
    assert core.accept(stale_scan(fault=True,reason='imu_gap'),100.31)
    assert core.accept(stale_scan(100.32),100.32)
    assert core.evaluate(100.32) is None and core.fault=='imu_gap'
    assert core.pending_notices==0
    expired=prepared()
    assert expired.accept(stale_scan(100.36),100.36)
    assert expired.accept(stale_scan(100.37),100.37)
    assert expired.evaluate(100.37) is None and expired.pending_notices==0


def test_next_real_posterior_recovers_without_new_epoch():
    core=prepared()
    assert core.accept(stale_scan(),100.31)
    new=snapshot(100.32);new['position']=[.096,0.,0.]
    new['inertial']['world_velocity']=[.3,0.,0.]
    assert core.accept(new,100.34)
    result=core.evaluate(100.34)
    assert result is not None and result.source_stamp==int(new['stamp_ns'])*1e-9 and core.epoch==1
    assert result.pose.position[0]==pytest.approx(.102)


@pytest.mark.parametrize('native',[100.1234567899,1800000000.1234567,1791791791.9999998])
def test_exact_native_wire_identity_no_fuzzy_posterior_matching(native):
    core=CausalInertialPredictor(PredictionLimits(max_horizon=.35,max_coast=.1))
    ns=int(native*1e9);source=ns*1e-9;value=snapshot(source)
    value['stamp_ns']=str(ns)
    assert core.accept(value,source)
    assert core.push_imu(source,(0.,0.,9.81),(0.,0.,0.),source)
    assert core.accept(stale_scan(source+.02,last_good_posterior_sec=native),source+.02)
    assert core.evaluate(source+.02) is not None
    wrong=math.nextafter(native,math.inf)
    while int(wrong*1e9)*1e-9==source:wrong=math.nextafter(wrong,math.inf)
    assert core.accept(stale_scan(source+.03,last_good_posterior_sec=wrong),source+.03)
    assert core.evaluate(source+.03) is None
