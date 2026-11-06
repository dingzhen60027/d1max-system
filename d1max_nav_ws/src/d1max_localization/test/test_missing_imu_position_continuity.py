"""Real predictor + output callback; never constructs a ROS graph or worker."""
from dataclasses import replace
import json
from unittest.mock import Mock
import pytest
from d1max_localization.estimation.causal_prediction import CausalInertialPredictor
from d1max_localization.estimation.prediction import PredictionLimits
from d1max_localization.navigation_output import NavigationOutput
from test_navigation_estimation import snapshot
from test_realtime_navigation_output import realtime_boundary


def predictor():
    p=CausalInertialPredictor(PredictionLimits(max_horizon=.35,max_coast=.1))
    v=snapshot(100.);v['inertial']['world_velocity']=[.3,0.,0.]
    assert p.accept(v,100.)
    assert p.push_imu(100.,(0.,0.,9.81),(0.,0.,.1),100.)
    return p


def test_359994ms_gap_keeps_position_model_but_not_motion_and_covariance_grows():
    p=predictor();saved=p.snapshot
    assert p.evaluate(100.359994) is None
    m=p.evaluate_pose_only(100.359994)
    assert m is not None and m.pose.position[0]==pytest.approx(.1079982)
    assert m.source_stamp==m.imu_stamp==100. and m.stamp==100.359994
    assert m.pose_covariance[0]>saved[-2][0] and p.snapshot is saved
    assert p.push_imu(100.359994,(0.,0.,9.81),(0.,0.,.1),100.359994)
    assert p.evaluate(100.36) is None  # No interpolation of the gap endpoints.
    resumed=p.evaluate_pose_only(100.36)
    assert resumed is not None and resumed.source_stamp==100.
    assert resumed.imu_stamp==100.359994
    assert p.accept(snapshot(100.40),100.40)
    assert p.push_imu(100.40,(0.,0.,9.81),(0.,0.,.1),100.40)
    assert p.evaluate(100.40) is not None and p.epoch==1


@pytest.mark.parametrize('case',['duration','rotation','distance','clock','epoch'])
def test_no_unsafe_or_unbounded_model(case):
    p=predictor();target=100.36
    if case=='duration':target=100.5001
    if case=='rotation':p.imu[-1]=(100.,(0.,0.,9.81),(0.,0.,2.))
    if case=='distance':
        s=list(p.snapshot);s[2]=(2.,0.,0.);p.snapshot=tuple(s)
    if case=='clock':p.fault='imu_clock_reset'
    if case=='epoch':p.accept(dict(schema=1,epoch=2,valid=False,fault=False,reason='imu_reinitializing',received_at_unix=100.1),100.1)
    assert p.evaluate_pose_only(target) is None


def test_returned_high_rotation_endpoint_cannot_sanction_unknown_gap_motion():
    p=predictor()
    assert p.push_imu(100.359994,(0.,0.,9.81),(0.,0.,2.),100.359994)
    assert p.evaluate_pose_only(100.36) is None


def test_received_future_imu_is_not_used_as_pose_model_restart():
    p=predictor()
    assert p.push_imu(100.39,(0.,0.,99.),(0.,0.,2.),100.36)
    model=p.evaluate_pose_only(100.36)
    assert model is not None and model.imu_stamp==100.
    assert model.pose.position[0]==pytest.approx(.108)


def test_native_soft_gap_notice_preserves_exact_posterior_not_its_source_age():
    p=predictor();old=(p.snapshot,p.snapshot_receipt,p.snapshot_publication)
    v=dict(schema=1,epoch=1,valid=False,fault=False,reason='imu_gap_coasting',received_at_unix=100.36,
        imu_quality_scope='last_admitted_scan',last_good_posterior_sec=100.,finite_filter_state=True,gap_coasting=True)
    assert p.accept(v,100.36) and (p.snapshot,p.snapshot_receipt,p.snapshot_publication)==old
    assert p.evaluate_pose_only(100.36) is not None
    v['fault']=True;assert p.accept(v,100.37)
    assert p.evaluate_pose_only(100.38) is None


def test_finite_weak_scan_during_gap_recovery_keeps_only_original_bounded_position_model():
    p=predictor();original=p.snapshot
    notice=dict(schema=1,epoch=1,valid=False,fault=False,reason='weak_geometry',received_at_unix=100.4,
        imu_quality_scope='last_admitted_scan',last_good_posterior_sec=100.,finite_filter_state=True,gap_coasting=True)
    assert p.accept(notice,100.4) and p.snapshot is original
    assert p.evaluate(100.4) is None and p.evaluate_pose_only(100.4) is not None
    p.accept(dict(notice,gap_coasting=False,received_at_unix=100.41),100.41)
    assert p.evaluate_pose_only(100.41) is None


def test_same_source_duplicate_notice_does_not_renew_position_deadline():
    p=predictor()
    for target in (100.2,100.4,100.6,101.):
        p.accept(dict(schema=1,epoch=1,valid=False,fault=False,reason='tracking',received_at_unix=target,
            imu_quality_scope='last_admitted_scan',imu_quality_scan_end_sec=100.),target)
    assert p.evaluate_pose_only(101.) is None and p.snapshot[0]==100.


def test_real_output_pose_only_keeps_tf_axes_but_never_control_wire_or_ekf():
    n=realtime_boundary();output=n.core.output(10.2)
    n.publish_global(*output)
    for pub in (n.motion_pub,n.local_state_pub,n.state_pair_pub,n.local_pub,n.global_pub):pub.reset_mock()
    n.tf.reset_mock()
    n.now_s.return_value=10.36
    p=CausalInertialPredictor(PredictionLimits(max_horizon=.35,max_coast=.1))
    v=snapshot(10.);v['inertial']['world_velocity']=[.3,0.,0.]
    p.accept(v,10.);p.push_imu(10.,(0.,0.,9.81),(0.,0.,0.),10.)
    model=p.evaluate_pose_only(10.36)
    n.publish_pose_continuity(model)
    value=json.loads(n.pose_continuity_pub.publish.call_args.args[0].data)
    assert value['available'] and value['motion_usable'] is False
    assert value['posterior_stamp_ns']==value['imu_stamp_ns']==10_000_000_000
    assert value['valid_until_ns']==10_500_000_000
    assert n.tf.sendTransform.call_count==2
    for pub in (n.motion_pub,n.local_state_pub,n.state_pair_pub,n.local_pub,n.global_pub):pub.publish.assert_not_called()
    assert n.pose_status(10.36)['pose_valid'] and not n.pose_status(10.36)['motion_usable']
    n.core.fault='local_clock_reset';n.publish_pose_continuity(model)
    assert not json.loads(n.pose_continuity_pub.publish.call_args.args[0].data)['available']


def test_returned_endpoint_motion_withdrawal_does_not_delete_same_source_display_anchor():
    from std_msgs.msg import String
    from d1max_localization.realtime_navigation_output import prediction_envelope
    n=realtime_boundary();n.publish_global(*n.core.output(10.2))
    p=CausalInertialPredictor(PredictionLimits(max_horizon=.35,max_coast=.1))
    v=snapshot(10.2);v['inertial']['world_velocity']=[.3,0.,0.]
    p.accept(v,10.2);p.push_imu(10.2,(0.,0.,9.81),(0.,0.,0.),10.2)
    p.push_imu(10.519994,(0.,0.,9.81),(0.,0.,0.),10.519994)
    n.now_s.return_value=10.52
    assert p.evaluate(10.52) is None and p.reason=='imu_gap'
    # The normal envelope legitimately withdraws usable motion. Its soft
    # absence is not a new epoch, seed or coordinate anchor.
    n.on_local(String(data=json.dumps(prediction_envelope(p,None,10.52,1))))
    assert n.core.last_output_key is None
    n.publish_pose_continuity(p.evaluate_pose_only(10.52))
    assert json.loads(n.pose_continuity_pub.publish.call_args.args[0].data)['available']
    assert not n.core.local_ready(10.52)
    n.core.map_contract['seed_id']='new_seed'
    n.publish_pose_continuity(p.evaluate_pose_only(10.52))
    assert not json.loads(n.pose_continuity_pub.publish.call_args.args[0].data)['available']
    assert n.pose_continuity_alignment is None
