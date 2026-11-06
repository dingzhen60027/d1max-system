"""Same production prediction core; no ROS graph, SDK or motion authority."""
from dataclasses import replace
import json
import pytest

from d1max_localization.estimation.causal_prediction import CausalInertialPredictor
from d1max_localization.estimation.prediction import PredictionLimits
from d1max_localization.estimation.navigation import NavigationLimits,NavigationState
from d1max_localization.realtime_navigation_output import (
    predictor_parameters,prediction_envelope,quantiles_ms,causal_output_time,
)
from test_navigation_estimation import snapshot


def test_delayed_10hz_posterior_and_one_missed_scan_keep_genuine_50hz_states():
    predictor=CausalInertialPredictor(PredictionLimits(max_horizon=.35))
    nav=NavigationState(replace(NavigationLimits(),max_prediction_horizon=.35))
    posterior=snapshot(100.)
    posterior['inertial']['world_velocity']=[1.,0.,0.]
    assert predictor.accept(posterior,100.12)
    previous=None;imu_index=0
    for index in range(12):
        target=100.12+index*.02
        while 99.995+imu_index*.005<=target+1e-10:
            stamp=99.995+imu_index*.005
            assert predictor.push_imu(stamp,(0.,0.,9.81),(0.,0.,0.),target)
            imu_index+=1
        state=predictor.evaluate(target)
        assert state is not None and state.stamp==pytest.approx(target)
        envelope=prediction_envelope(predictor,state,target,index+1)
        assert nav.push_local(envelope,target)
        assert state.source_stamp==100. and state.imu_stamp<=state.stamp
        assert state.pose.position[0]==pytest.approx(target-100.)
        if previous is not None: assert state.stamp-previous==pytest.approx(.02)
        previous=state.stamp
    assert predictor.evaluate(100.36) is None and predictor.reason=='lio_stale'


def test_same_realtime_policy_does_not_enlarge_imu_or_rotation_deadlines():
    predictor=CausalInertialPredictor(PredictionLimits(max_horizon=.35,max_coast=.1))
    assert predictor.accept(snapshot(100.),100.)
    assert predictor.push_imu(100.,(0.,0.,9.81),(0.,0.,0.),100.)
    assert predictor.evaluate(100.1) is not None
    assert predictor.evaluate(100.101) is None and predictor.reason=='imu_stale'
    other=CausalInertialPredictor(PredictionLimits(max_horizon=.35,max_coast=.1))
    assert other.accept(snapshot(100.),100.)
    assert other.push_imu(100.,(0.,0.,9.81),(0.,0.,2.),100.)
    assert other.evaluate(100.05) is None and other.reason=='coast_rotation'


def test_unavailable_timer_envelope_never_fabricates_pose_or_source_time():
    predictor=CausalInertialPredictor()
    assert predictor.accept(snapshot(100.),100.)
    value=prediction_envelope(predictor,None,100.2,5)
    assert value['valid'] is False and value['epoch']==1
    assert 'stamp_ns' not in value and 'position' not in value


def test_parameter_resolution_does_not_invent_sensor_observations():
    resolved=predictor_parameters(dict(imu_frame='imu',output_rate_hz=50.,
        odom_frame='odom',tracking_frame='tracking',max_horizon=.35,max_coast=.1))
    assert resolved=={'imu_frame':'imu','prediction.max_horizon':.35,'prediction.max_coast':.1}
    assert quantiles_ms([.02,.02,.04])['maximum']==40.
    assert quantiles_ms([])=={}


def pending_notice(now, epoch=1, reason='tracking', fault=False, scan_end=100.):
    return dict(schema=1,epoch=epoch,valid=False,fault=fault,reason=reason,
        received_at_unix=now,imu_quality_scope='last_admitted_scan',
        imu_quality_scan_end_sec=scan_end)


def test_native_scan_timeout_notice_does_not_revoke_or_renew_predictor_posterior():
    predictor=CausalInertialPredictor(PredictionLimits(max_horizon=.35))
    assert predictor.accept(snapshot(100.),100.12)
    original=(predictor.snapshot,predictor.snapshot_receipt,predictor.snapshot_publication)
    imu_index=0
    for target in (100.30,100.32,100.34,100.36,100.38):
        while 100.+imu_index*.005<=target+1e-10:
            stamp=100.+imu_index*.005
            assert predictor.push_imu(stamp,(0.,0.,9.81),(0.,0.,0.),stamp)
            imu_index+=1
        assert predictor.accept(pending_notice(target),target)
        result=predictor.evaluate(target)
        if target <= 100.35:
            assert result is not None and result.source_stamp==100.
        else:
            assert result is None and predictor.reason=='lio_stale'
        assert (predictor.snapshot,predictor.snapshot_receipt,predictor.snapshot_publication)==original
    assert predictor.pending_notices==5


@pytest.mark.parametrize('change',[
    {'fault':True}, {'epoch':2}, {'reason':'weak_geometry'},
    {'reason':'imu_gap'}, {'reason':'stale_scan'}, {'reason':'initializing'},
    {'imu_quality_scope':'unknown'}, {'imu_quality_scan_end_sec':100.01},
    {'imu_quality_scan_end_sec':float('nan')}, {'imu_quality_scan_end_sec':'100'},
])
def test_pending_preservation_never_ignores_reset_fault_or_unproven_notice(change):
    predictor=CausalInertialPredictor(PredictionLimits(max_horizon=.35))
    assert predictor.accept(snapshot(100.),100.)
    value=pending_notice(100.31);value.update(change)
    assert predictor.accept(value,100.31)
    assert predictor.evaluate(100.31) is None
    assert predictor.pending_notices==0
    # A malformed diagnostic cannot crash the next high-rate JSON tick.
    json.dumps(prediction_envelope(predictor,None,100.31,2),allow_nan=False)


def test_pending_notice_cannot_revive_rejected_state_or_mask_imu_expiry():
    predictor=CausalInertialPredictor(PredictionLimits(max_horizon=.35,max_coast=.1))
    assert predictor.accept(snapshot(100.),100.)
    assert predictor.push_imu(100.,(0.,0.,9.81),(0.,0.,0.),100.)
    assert predictor.accept(pending_notice(100.11),100.11)
    assert predictor.evaluate(100.11) is None and predictor.reason=='imu_stale'
    assert predictor.accept(pending_notice(100.12,reason='weak_geometry'),100.12)
    assert predictor.accept(pending_notice(100.13),100.13)
    assert not predictor.valid


def test_nanosecond_roundtrip_cannot_make_current_prediction_a_future_packet():
    from d1max_localization.estimation_ros import stamp_time
    from d1max_localization.navigation_wire import body_pair_message
    from nav_msgs.msg import Odometry
    for epoch in (100_000_000_000,1772776384379474000,1790958000000000000):
        for index in range(1000):
            clock_ns=epoch+(index*47998279)%45000000000
            target=causal_output_time(clock_ns)
            # Exactly the real JSON -> numerical state -> ROS boundary path.
            decoded=round(target*1e9)*1e-9
            assert decoded==target
            source=stamp_time(decoded)
            wire_ns=source.sec*10**9+source.nanosec
            assert 0<=clock_ns-wire_ns<5000
        local=Odometry();global_=Odometry()
        local.header.stamp=global_.header.stamp=stamp_time(decoded)
        local.header.frame_id='d1max_loc_odom';global_.header.frame_id='d1max_loc_map'
        local.child_frame_id=global_.child_frame_id='d1max_loc_base_link'
        local.pose.pose.orientation.w=global_.pose.pose.orientation.w=1.
        packet=body_pair_message(local=local,global_=global_,session_id='s',map_version_id='m',
            epoch=1,seed_id='seed',posterior_stamp=decoded-.2,imu_stamp=decoded-.01,extrapolation_sec=.01)
        assert packet.source_stamp.sec*10**9+packet.source_stamp.nanosec<=clock_ns


def realtime_boundary():
    """Actual serialized callbacks/tick with message publishers, no ROS graph.

    Only the private EKF's input is supplied by the fixture. Prediction,
    aligned history, body conversion, source stamps and output admission all
    run through the production node implementation, not a copied estimator.
    """
    from collections import deque
    from pathlib import Path
    from unittest.mock import Mock
    import yaml
    from d1max_localization.estimation.configuration import navigation_parameters
    from d1max_localization.math_utils import Pose3
    from d1max_localization.realtime_navigation_output import RealtimeNavigationOutput
    from test_navigation_estimation import locked

    root = Path(__file__).parents[1]
    config = yaml.safe_load((root/'config/localization.yaml').read_text())
    config['navigation_estimation']['ros__parameters'].update(
        yaml.safe_load((root/'config/realtime_navigation.yaml').read_text())
        ['navigation_estimation']['ros__parameters'])
    prediction, output, _ = navigation_parameters(config)
    node = RealtimeNavigationOutput.__new__(RealtimeNavigationOutput)
    node.core = locked()
    node.core.limits = NavigationLimits(**{key[7:]:value for key,value in output.items()
                                         if key.startswith('limits.')})
    node.p = dict(map_frame='d1max_loc_map', odom_frame='d1max_loc_odom',
                  tracking_frame='d1max_loc_tracking', body_frame='d1max_loc_base_link',
                  trajectory_rate_hz=5., extrinsics_verified=False,
                  time_alignment_verified=False)
    node.extrinsic = Pose3((0.,0.,0.),(0.,0.,0.,1.))
    node.wire_session = {'id':'session','version_id':'map'}
    node.rate = output['output_rate_hz']
    node.predictor = CausalInertialPredictor(PredictionLimits(**{
        key:value for key,value in prediction.items() if key in vars(PredictionLimits())}))
    node.now_s = Mock(return_value=10.2)
    node.imu_frame = prediction['imu_frame']
    for name in ('motion_pub','local_pub','local_state_pub','global_pub','state_pair_pub',
                 'pose_pub','path_pub','status_pub','pose_status_pub','map_pub',
                 'prediction_pub','rt_pub','tf'):
        setattr(node,name,Mock())
    for name in ('path','local_times','global_times','local_arrivals','global_arrivals',
                 'rt_durations','rt_intervals'):
        setattr(node,name,deque(maxlen=3000))
    node.reset_client = Mock()
    node.reset_client.service_is_ready.return_value = False
    node.future = node.request_key = node.last_local_state_identity = None
    node.last_local_unavailable = None
    node.predictor_status = {}
    node.last_error = ''
    for name in ('last_local_sent','last_map_sent','last_path','last_status',
                 'last_local_state_event','request_stamp','request_at','prediction_sequence',
                 'rt_last_tick','rt_last_status','rt_overruns','rt_ticks',
                 'clock_target_quantization_ns_max'):
        setattr(node,name,0)
    return node


@pytest.mark.parametrize('global_gap',[None,'map','filter'])
def test_actual_realtime_callbacks_emit_source_timed_50hz_body_states_across_global_gap(global_gap):
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from d1max_localization.estimation_ros import stamp_time
    from test_navigation_estimation import alignment, diagonal

    node = realtime_boundary()
    next_imu = 0
    for tick in range(70):
        now = 10.22+tick*.02
        node.now_s.return_value = now
        if tick % 5 == 0:
            node.on_posterior(String(data=json.dumps(snapshot(now))))
        while 10.195+next_imu*.005 <= now+1e-9:
            acquisition = 10.195+next_imu*.005
            message = Imu()
            message.header.frame_id = node.imu_frame
            message.header.stamp = stamp_time(acquisition)
            message.linear_acceleration.z = 9.81
            node.on_imu(message)
            next_imu += 1
        withholding = global_gap is not None and tick >= 10
        if not withholding or global_gap != 'map':
            correction = .002*tick if global_gap is None else 0.
            contract = alignment(now)
            contract['position'][0] = correction
            contract['anchor']['position'][0] = correction
            node.on_map(String(data=json.dumps(contract)))
        if not withholding or global_gap != 'filter':
            message = Odometry()
            message.header.stamp = stamp_time(now)
            message.header.frame_id = 'd1max_loc_map'
            message.child_frame_id = 'd1max_loc_tracking'
            message.pose.pose.position.x = .002*tick if global_gap is None else 0.
            message.pose.pose.orientation.w = 1.
            message.pose.covariance = diagonal()
            node.on_filtered(message)
        node.tick()
        assert not node.last_error and not node.core.fault

    local = [call.args[0] for call in node.local_state_pub.publish.call_args_list]
    assert len(local) == 70 and all(message.usable for message in local)
    times = [message.source_stamp.sec+message.source_stamp.nanosec*1e-9 for message in local]
    assert times == pytest.approx([10.22+tick*.02 for tick in range(70)])
    for message in local:
        source = message.source_stamp.sec*10**9+message.source_stamp.nanosec
        posterior = message.posterior_stamp.sec*10**9+message.posterior_stamp.nanosec
        imu = message.imu_stamp.sec*10**9+message.imu_stamp.nanosec
        assert message.source_stamp == message.local_odometry.header.stamp
        assert 0 < posterior <= source and 0 < imu <= source
        assert 0 <= source-posterior <= 350_000_000
        assert 0 <= source-imu <= 100_000_000
        assert (message.local_odometry.header.frame_id,
                message.local_odometry.child_frame_id) == ('d1max_loc_odom','d1max_loc_base_link')
    if global_gap is None:
        pairs = [call.args[0] for call in node.state_pair_pub.publish.call_args_list
                 if call.args[0].source_stamp.sec*10**9+call.args[0].source_stamp.nanosec
                 >= 10_220_000_000]
        assert len(pairs) == 70
        assert all(message.local_odometry.header.stamp == message.global_odometry.header.stamp
                   == message.source_stamp for message in pairs)
        assert {(message.localization_epoch,message.localization_seed_id) for message in pairs} == {(1,'seed-a')}
        assert all(abs(message.local_odometry.pose.pose.position.x) < 1e-8 for message in pairs)
        # 13.8 cm cumulative map correction is not a new localization identity,
        # a reset of odom, or a reason to interrupt its real 20 ms cadence.
        assert pairs[-1].global_odometry.pose.pose.position.x > .10
    else:
        # The global correction genuinely expires; it cannot invalidate the
        # independent continuous-odom output or fabricate a global heartbeat.
        assert node.state_pair_pub.publish.call_count < 70
        assert not node.core.output_ready(node.now_s())
