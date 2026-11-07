"""Same production prediction core; no ROS graph, SDK or motion authority."""
from dataclasses import replace
import json
import pytest

from d1max_localization.estimation.causal_prediction import CausalInertialPredictor
from d1max_localization.estimation.prediction import InertialPredictor, PredictionLimits
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


def test_real_predictor_delayed_landing_posterior_keeps_same_navigation_epoch():
    predictor = CausalInertialPredictor(PredictionLimits(max_horizon=.35))
    nav = NavigationState(replace(NavigationLimits(), max_prediction_horizon=.35))
    assert predictor.accept(snapshot(100.), 100.)
    for i in range(43):
        stamp = 99.995+i*.005
        assert predictor.push_imu(stamp, (0., 0., 9.81), (0., 0., 0.), stamp)
    before = predictor.evaluate(100.18)
    assert nav.push_local(prediction_envelope(predictor, before, 100.18, 1), 100.18)
    # A delayed scan corrects the vertical prediction by one 18-cm tread.
    # It corrects a 100-ms posterior interval; receipt is only 20 ms later.
    landed = snapshot(100.10)
    landed['received_at_unix'] = 100.20
    landed['position'][2] = -.18
    assert predictor.accept(landed, 100.20)
    state = predictor.evaluate(100.20)
    assert state is not None
    assert nav.push_local(prediction_envelope(predictor, state, 100.20, 2), 100.20)
    assert nav.epoch == 1 and not nav.fault and nav.local_ready(100.20)
    assert nav.raw_local.pose.position[2] == pytest.approx(-.18)
    assert nav.local[-1].source_stamp == pytest.approx(100.10)
    assert nav.local[-1].imu_stamp <= nav.local[-1].stamp
    assert nav.local_transition['translation_m'] == pytest.approx(.18)
    assert nav.local_transition['reason'] == 'posterior_correction_admitted'


@pytest.mark.parametrize('direction', [-1., 0., 1.])
def test_real_imu_step_motion_and_delayed_scans_preserve_fifty_hz_output(direction):
    import math
    from d1max_localization.math_utils import rotate_vector, quaternion_conjugate
    from d1max_localization.estimation.prediction import rotation_step

    predictor = CausalInertialPredictor(PredictionLimits(max_horizon=.35))
    nav = NavigationState(replace(NavigationLimits(), max_prediction_horizon=.35))

    def physical(t):
        s = min(1., max(0., (t-100.3)/.4))
        h = 10*s**3-15*s**4+6*s**5
        dz = direction*.18*(30*s**2-60*s**3+30*s**4)/.4
        ddz = direction*.18*(60*s-180*s**2+120*s**3)/.4**2
        pitch = direction*.20*math.sin(math.pi*s)
        omega = direction*.20*math.pi*math.cos(math.pi*s)/.4 if 0 < s < 1 else 0.
        return (.3*(t-100.), 0., direction*.18*h), (.3, 0., dz), ddz, pitch, omega

    imu_index = 0
    posterior_stamp = None
    for tick in range(76):
        now = 100.+tick*.02
        while 99.895+imu_index*.005 <= now+1e-9:
            stamp = 99.895+imu_index*.005
            _, _, acc_z, pitch, omega = physical(stamp)
            orientation = rotation_step((0., 1., 0.), pitch)
            acc = rotate_vector(quaternion_conjugate(orientation), (0., 0., 9.81+acc_z))
            assert predictor.push_imu(stamp, acc, (0., omega, 0.), now)
            imu_index += 1
        anchor = 99.9+(tick//5)*.1
        if anchor != posterior_stamp and tick not in (20, 21, 22, 23, 24):
            point, velocity, _, pitch, _ = physical(anchor)
            posterior = snapshot(anchor)
            posterior.update(received_at_unix=now, position=point,
                             orientation=rotation_step((0., 1., 0.), pitch))
            posterior['inertial']['world_velocity'] = list(velocity)
            assert predictor.accept(posterior, now)
            posterior_stamp = anchor
        state = predictor.evaluate(now)
        assert state is not None, (now, predictor.reason)
        assert nav.push_local(prediction_envelope(predictor, state, now, tick+1), now)
        assert nav.local_ready(now) and not nav.fault and nav.epoch == 1
        expected, _, _, _, _ = physical(now)
        assert max(abs(a-b) for a, b in zip(state.pose.position, expected)) < .002
        assert nav.local[-1].stamp == pytest.approx(now)
        assert nav.local[-1].source_stamp == state.source_stamp
    assert nav.local_sequence == 76 and nav.local_rejected_samples == 0


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


def weak_geometry_notice(now, posterior=100., **changes):
    value=pending_notice(now,reason='weak_geometry',scan_end=100.1)
    value.update(last_good_posterior_sec=posterior,finite_filter_state=True)
    value.update(changes)
    return value


def test_finite_weak_scan_preserves_only_original_bounded_posterior_then_recovers():
    limits=PredictionLimits(max_horizon=.35,max_coast=.1)
    predictor=CausalInertialPredictor(limits)
    before=InertialPredictor(limits)  # Former weak notice delegated to this exact core.
    original=snapshot(100.)
    original['inertial']['world_velocity']=[1.,0.,0.]
    assert predictor.accept(original,100.12)
    assert before.accept(original,100.12)
    retained=(predictor.snapshot,predictor.snapshot_receipt,predictor.snapshot_publication)
    imu_index=0
    for target in (100.12,100.14,100.20,100.30,100.34,100.36,100.38):
        while 100.+imu_index*.005<=target+1e-10:
            acquisition=100.+imu_index*.005
            assert predictor.push_imu(acquisition,(0.,0.,9.81),(0.,0.,0.),acquisition)
            assert before.push_imu(acquisition,(0.,0.,9.81),(0.,0.,0.),acquisition)
            imu_index+=1
        assert predictor.accept(weak_geometry_notice(target),target)
        assert before.accept(weak_geometry_notice(target),target)
        assert before.evaluate(target) is None  # Same input used to revoke immediately.
        state=predictor.evaluate(target)
        if target<=100.35:
            assert state is not None and state.source_stamp==100.
            assert state.stamp==target and state.pose.position[0]==pytest.approx(target-100.)
        else:
            assert state is None  # Even endless weak notices cannot recharge it.
        assert (predictor.snapshot,predictor.snapshot_receipt,predictor.snapshot_publication)==retained
        assert predictor.epoch==1 and not predictor.fault
    assert predictor.pending_notices==5
    fresh=snapshot(100.40)
    fresh['inertial']['world_velocity']=[1.,0.,0.]
    assert predictor.accept(fresh,100.40)
    assert predictor.push_imu(100.40,(0.,0.,9.81),(0.,0.,0.),100.40)
    state=predictor.evaluate(100.40)
    assert state is not None and state.source_stamp==100.40 and predictor.epoch==1


@pytest.mark.parametrize('changes',[
    {'finite_filter_state':False},{'finite_filter_state':1},
    {'last_good_posterior_sec':100.001},{'last_good_posterior_sec':float('nan')},
    {'last_good_posterior_sec':'100'},{'last_good_posterior_sec':0.},
    {'epoch':2},{'fault':True},{'reason':'nonfinite_filter_state'},
    {'reason':'initializing'},{'reason':'unknown'}, {'imu_quality_scope':'unknown'},
])
def test_weak_scan_without_exact_finite_posterior_evidence_cannot_preserve(changes):
    predictor=CausalInertialPredictor(PredictionLimits(max_horizon=.35,max_coast=.1))
    assert predictor.accept(snapshot(100.),100.)
    assert predictor.push_imu(100.,(0.,0.,9.81),(0.,0.,0.),100.)
    assert predictor.accept(weak_geometry_notice(100.02,**changes),100.02)
    assert predictor.evaluate(100.02) is None and predictor.pending_notices==0
    json.dumps(prediction_envelope(predictor,None,100.02,2),allow_nan=False)


def test_finite_weak_scan_never_masks_imu_expiry_or_revives_a_bad_source():
    predictor=CausalInertialPredictor(PredictionLimits(max_horizon=.35,max_coast=.1))
    assert predictor.accept(snapshot(100.),100.)
    assert predictor.push_imu(100.,(0.,0.,9.81),(0.,0.,0.),100.)
    assert predictor.accept(weak_geometry_notice(100.11),100.11)
    assert predictor.evaluate(100.11) is None and predictor.reason=='imu_stale'
    # An unrelated invalid source cannot be papered over by a later weak notice.
    assert predictor.accept(pending_notice(100.12,reason='initializing'),100.12)
    assert predictor.accept(weak_geometry_notice(100.13),100.13)
    assert not predictor.valid


@pytest.mark.parametrize('native_sec',[100.1234567899,1800000000.1234567,1791791791.9999998])
def test_weak_posterior_identity_matches_exact_native_wire_time(native_sec):
    import math

    source_ns=int(native_sec*10**9)  # The native odometry uint64 conversion.
    source=source_ns*1e-9
    predictor=CausalInertialPredictor(PredictionLimits(max_horizon=.35,max_coast=.1))
    value=snapshot(source);value['stamp_ns']=str(source_ns)
    assert predictor.accept(value,source)
    assert predictor.push_imu(source,(0.,0.,9.81),(0.,0.,0.),source)
    now=source+.02
    assert predictor.accept(weak_geometry_notice(now,posterior=native_sec),now)
    assert predictor.evaluate(now) is not None and predictor.pending_notices==1
    # One different representable native source cannot be accepted as the
    # same posterior, even at Unix epochs where floats are sub-microsecond.
    other=math.nextafter(native_sec,math.inf)
    while int(other*10**9)==source_ns:
        other=math.nextafter(other,math.inf)
    assert predictor.accept(weak_geometry_notice(now,posterior=other),now)
    assert predictor.evaluate(now) is None and predictor.pending_notices==1


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
    node.get_logger = Mock(return_value=Mock())
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
    node.last_local_state_lease = None
    node.last_local_unavailable = None
    node.pose_continuity=None;node.pose_continuity_alignment=None
    node.pose_continuity_sequence=0;node.pose_continuity_pub=Mock()
    node.predictor_status = {}
    node.last_error = ''
    for name in ('last_local_sent','last_map_sent','last_path','last_status',
                 'last_local_state_event','request_stamp','request_at','prediction_sequence',
                 'rt_last_tick','rt_last_status','rt_overruns','rt_ticks',
                 'clock_target_quantization_ns_max'):
        setattr(node,name,0)
    return node


def test_actual_realtime_weak_scan_keeps_deadlines_then_resumes_same_map_identity():
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from d1max_localization.estimation_ros import stamp_time
    from test_navigation_estimation import alignment, diagonal

    node=realtime_boundary()
    node.on_posterior(String(data=json.dumps(snapshot(10.2))))
    next_imu=0
    for tick in range(20):
        now=10.22+tick*.02
        node.now_s.return_value=now
        while 10.195+next_imu*.005<=now+1e-9:
            acquisition=10.195+next_imu*.005
            message=Imu();message.header.frame_id=node.imu_frame
            message.header.stamp=stamp_time(acquisition);message.linear_acceleration.z=9.81
            node.on_imu(message);next_imu+=1
        if tick==19:
            node.on_posterior(String(data=json.dumps(snapshot(now))))
            node.on_map(String(data=json.dumps(alignment(now))))
        else:
            notice=weak_geometry_notice(now,posterior=10.2,imu_quality_scan_end_sec=10.3)
            node.on_posterior(String(data=json.dumps(notice)))
        filtered=Odometry();filtered.header.stamp=stamp_time(now)
        filtered.header.frame_id='d1max_loc_map';filtered.child_frame_id='d1max_loc_tracking'
        filtered.pose.pose.orientation.w=1.;filtered.pose.covariance=diagonal()
        node.on_filtered(filtered)
        node.tick()
    local=[call.args[0] for call in node.local_state_pub.publish.call_args_list if call.args[0].usable]
    pairs=[call.args[0] for call in node.state_pair_pub.publish.call_args_list]
    assert len(local)==18  # Real 20ms samples, then a truthful 60ms gap.
    local_stamps=[m.source_stamp.sec+m.source_stamp.nanosec*1e-9 for m in local]
    assert local_stamps[:-1]==pytest.approx([10.22+i*.02 for i in range(17)])
    assert local_stamps[-1]==pytest.approx(10.60)
    stamps=[m.source_stamp.sec+m.source_stamp.nanosec*1e-9 for m in pairs]
    # One pre-existing real sample is published when the first EKF update
    # arrives; the remaining pairs keep the exact local sample source times.
    assert len(pairs)==19
    assert stamps[:-1]==pytest.approx([10.20+i*.02 for i in range(18)])
    assert stamps[-1]==pytest.approx(10.60)
    assert node.core.key()==(1,'seed-a') and not node.core.fault and not node.core.filter_fault
    assert node.predictor.pending_notices==17 and node.core.output_ready(10.60)
    for message in local:
        source=message.source_stamp.sec*10**9+message.source_stamp.nanosec
        assert message.valid_until.sec*10**9+message.valid_until.nanosec==source+100_000_000
    unavailable=[call.args[0] for call in node.local_state_pub.publish.call_args_list if not call.args[0].usable]
    assert len(unavailable)==1 and unavailable[0].reason=='local_navigation_unavailable'
    diagnostics=[call.args[0] for call in node.get_logger().info.call_args_list
                 if call.args[0].startswith('LocalStateUnavailable ')]
    assert len(diagnostics)==1  # Repeated unavailable ticks are one episode.
    detail=json.loads(diagnostics[0].split(' ',1)[1])
    unavailable_ns=unavailable[0].source_stamp.sec*10**9+unavailable[0].source_stamp.nanosec
    prior_ns=local[-2].source_stamp.sec*10**9+local[-2].source_stamp.nanosec
    assert detail['event_source_ns']==unavailable_ns==prior_ns+1
    assert detail['reason']==unavailable[0].reason
    assert (detail['epoch'],detail['seed'])==(1,'seed-a')
    assert detail['source_timing']['posterior_stamp_sec']==pytest.approx(10.2)
    assert detail['source_timing']['target_stamp_sec']==pytest.approx(10.54)
    assert node.last_local_unavailable is None  # Genuine recovery ends it.
    assert local[-1].posterior_stamp==stamp_time(10.60)
    assert local[-1].valid_until==stamp_time(10.70)
    # Expiry is explicit, not a repeated-pose heartbeat; the real new posterior
    # restores the same seed/epoch, without reinitialization or a new task.


@pytest.mark.parametrize('recover', [True, False])
def test_actual_realtime_imu_boundary_does_not_revoke_unexpired_typed_state(recover):
    """Production predictor -> output tick, no fabricated packet or extra IMU."""
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from d1max_localization.estimation_ros import stamp_time

    node = realtime_boundary()
    node.on_posterior(String(data=json.dumps(snapshot(10.2))))
    imu = Imu()
    imu.header.frame_id = node.imu_frame
    imu.header.stamp = stamp_time(10.2)
    imu.linear_acceleration.z = 9.81
    node.on_imu(imu)
    for now in (10.2, 10.22, 10.24, 10.26, 10.28):
        node.now_s.return_value = now
        node.tick()
    original = node.local_state_pub.publish.call_args.args[0]
    assert original.usable and original.source_stamp == stamp_time(10.28)
    assert original.imu_stamp == stamp_time(10.2)
    assert original.valid_until == stamp_time(10.38)
    count = node.local_state_pub.publish.call_count
    for now in (10.300204, 10.32):
        node.now_s.return_value = now
        node.tick()
        assert node.predictor.reason == 'imu_stale'
        assert node.core.local_valid and not node.core.local_ready(now)
        assert node.local_state_pub.publish.call_count == count
    if recover:
        node.now_s.return_value = 10.34
        node.on_posterior(String(data=json.dumps(snapshot(10.34))))
        imu.header.stamp = stamp_time(10.34)
        node.on_imu(imu)
        node.tick()
        recovered = node.local_state_pub.publish.call_args.args[0]
        assert recovered.usable and recovered.source_stamp == stamp_time(10.34)
        assert recovered.imu_stamp == stamp_time(10.34)
        assert recovered.valid_until == stamp_time(10.44)
        assert node.core.key() == (1, 'seed-a')
        assert all(call.args[0].usable for call in node.local_state_pub.publish.call_args_list)
    else:
        node.now_s.return_value = 10.38
        node.tick()
        unavailable = node.local_state_pub.publish.call_args.args[0]
        assert not unavailable.usable and unavailable.reason == 'local_navigation_unavailable'
        assert unavailable.source_stamp == stamp_time(10.280000001)
        assert node.local_state_pub.publish.call_count == count+1
        node.now_s.return_value = 10.40
        node.tick()
        assert node.local_state_pub.publish.call_count == count+1


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
