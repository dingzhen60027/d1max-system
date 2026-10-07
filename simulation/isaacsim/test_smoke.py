"""Validate the real BT client request without starting a ROS participant."""
from types import SimpleNamespace
import math

from builtin_interfaces.msg import Time
from d1max_pct_scan.bt_adapter_contract import valid_compute_request
from smoke import (navigation_goal, expected_task_result, drain_imu_witness,
    trace_output_path, cancellation_due, terminal_observation_complete,
    stationarity_window, body_height_evidence, source_observation_expired,
    trace_wire_value)


def test_navigation_goal_survives_unchanged_compute_action_validation():
    session=dict(id='isaac-session', frame_id='d1max_loc_map')
    stamp=Time(sec=123, nanosec=456)
    goal=navigation_goal(session, [-2.8,-3.,0.], stamp)
    forwarded=SimpleNamespace(session_id=session['id'],task_id='isaac-session.1',
        schema_version=goal.schema_version,goal_kind=goal.goal_kind,goal=goal.goal,
        has_goal_yaw=goal.has_goal_yaw,goal_yaw_tolerance_rad=goal.goal_yaw_tolerance_rad)
    assert valid_compute_request(forwarded, session['id'],session['frame_id'])
    assert not goal.has_goal_yaw
    assert goal.goal.header.stamp == stamp
    assert goal.goal.pose.position.z == 0.
    # Default ROS field value caused the observed compute action rejection.
    forwarded.goal_yaw_tolerance_rad=0.
    assert not valid_compute_request(forwarded,session['id'],session['frame_id'])


def test_cancel_timeout_or_failure_is_never_reported_as_cancel_pass():
    result=dict(success=False, reason='action_cancelled')
    assert expected_task_result('cancel', result, True, 5)
    assert expected_task_result('preview_cancel', result, True, 5)
    assert not expected_task_result('cancel', result, False, 5)
    assert not expected_task_result('cancel', result, True, 6)
    assert not expected_task_result('cancel', dict(success=False,
        reason='cancel_ack_timeout_requires_session_restart'), True, 5)
    assert expected_task_result('goal', dict(success=True), False, 4)
    assert not expected_task_result('goal', dict(success=True), False, 6)


def test_final_state_callback_waits_for_exact_imu_without_chasing_new_states():
    observer=SimpleNamespace(samples=[dict(source_stamp_ns=100,imu_stamp_ns=90),
        dict(source_stamp_ns=200,imu_stamp_ns=190)],first_imu_ns=90,actual_imu_times={90})
    calls=[]
    def spin(node, timeout_sec):
        calls.append(timeout_sec)
        # This is the real ordering race: another state arrives while the
        # independent IMU callback for the fenced last state is delivered.
        node.samples.append(dict(source_stamp_ns=300,imu_stamp_ns=290))
        node.actual_imu_times.add(190)
    result=drain_imu_witness(observer,spin)
    assert result['matched']
    assert result['state_count']==2
    assert result['fence_source_stamp_ns']==200
    assert result['missing_imu_source_stamps_ns']==[]
    assert len(calls)==1
    assert 290 not in observer.actual_imu_times
    assert len(observer.samples)==3  # Raw later measurements are retained.


def test_missing_interior_imu_never_passes_when_newer_samples_arrive():
    observer=SimpleNamespace(samples=[dict(source_stamp_ns=100,imu_stamp_ns=90),
        dict(source_stamp_ns=200,imu_stamp_ns=190)],first_imu_ns=90,actual_imu_times={90,290})
    clock=[0.]
    def spin(node, timeout_sec):
        clock[0]+=timeout_sec
        node.actual_imu_times.add(390)  # Cannot replace the missing 190 sample.
    result=drain_imu_witness(observer,spin,budget=.1,monotonic=lambda:clock[0])
    assert not result['matched']
    assert result['missing_imu_source_stamps_ns']==[190]
    assert result['receipt_wait_elapsed_s']>=.1


def test_trace_output_cannot_overwrite_or_cross_session(tmp_path):
    import pytest
    session=tmp_path/'session'
    session.mkdir()
    (session/'goals').mkdir()
    trace=trace_output_path(session,'goals/goal_001.jsonl')
    assert trace==session/'goals/goal_001.jsonl'
    trace.write_text('owned\n')
    with pytest.raises(FileExistsError):
        trace_output_path(session,trace)
    with pytest.raises(ValueError,match='inside_session'):
        trace_output_path(session,'../foreign.jsonl')
    (session/'escape').symlink_to(tmp_path,target_is_directory=True)
    with pytest.raises(ValueError,match='inside_session'):
        trace_output_path(session,'escape/foreign.jsonl')


def test_source_time_cancel_does_not_advance_during_wall_pause():
    values=dict(confirmed=True,requested=False,distance=10.,cancel_after=.25,
        source_origin_ns=1_000_000_000,cancel_source_time_s=12.)
    assert not cancellation_due(**values,source_clock_ns=12_999_999_999)
    assert not cancellation_due(**values,source_clock_ns=12_999_999_999)
    assert cancellation_due(**values,source_clock_ns=13_000_000_000)
    assert not cancellation_due(**dict(values,requested=True),source_clock_ns=20_000_000_000)
    assert cancellation_due(**dict(values,cancel_source_time_s=None),source_clock_ns=0)


def test_optional_source_budget_uses_source_clock_and_preserves_default():
    assert not source_observation_expired(100_000_000_000, 1_000_000_000, None)
    assert not source_observation_expired(100_000_000_000, None, 60.)
    assert not source_observation_expired(60_999_999_999, 1_000_000_000, 60.)
    assert not source_observation_expired(60_999_999_999, 1_000_000_000, 60.)
    assert source_observation_expired(61_000_000_000, 1_000_000_000, 60.)


def test_extended_stationarity_uses_complete_source_window_and_no_lease():
    assert terminal_observation_complete(1.,1.6,0.)  # Original default.
    assert not terminal_observation_complete(2.,100.,2.49)
    assert terminal_observation_complete(2.,100.,2.5)
    samples=[dict(elapsed_s=i*.01,source_stamp_ns=1_000_000_000+i*10_000_000,
        linear_mps=0.,angular_radps=0.) for i in range(301)]
    stationary,evidence=stationarity_window(samples,2.,3.)
    assert stationary and evidence['source_span_s']>=2.
    assert evidence['source_window_complete']
    stationary,_=stationarity_window(samples[-100:],2.,3.)
    assert not stationary
    broken=samples[:100]+samples[130:]
    stationary,_=stationarity_window(broken,2.,3.)
    assert not stationary  # Missing source samples cannot certify parking.


def test_model_height_tolerance_needs_independent_whole_body_certificate():
    import pytest
    assert body_height_evidence({}, {}, 100)[:2]==(.03,True)
    session=dict(static_collision_prior_contract=dict(body_envelope_attestation_required=True,
        full_leg_volume_required=True,max_body_height_error_m=.12,
        body_envelope_registry_sha256='sealed_actual_registry'))
    bridge=dict(body_envelope_verified_samples=100,body_envelope_failures=0,
        body_envelope_registry_sha256='sealed_actual_registry')
    assert body_height_evidence(session,bridge,100)[:2]==(.12,True)
    assert not body_height_evidence(session,{},100)[1]
    assert not body_height_evidence(session,dict(bridge,body_envelope_verified_samples=99),100)[1]
    assert not body_height_evidence(session,dict(bridge,body_envelope_failures=1),100)[1]
    assert not body_height_evidence(session,dict(bridge,body_envelope_registry_sha256='foreign'),100)[1]
    session['static_collision_prior_contract']['max_body_height_error_m']=math.nan
    with pytest.raises(ValueError):
        body_height_evidence(session,bridge,100)


def test_causal_trace_preserves_complete_received_curve_and_exact_entry():
    from d1max_planning_interfaces.msg import TaggedBspline
    from geometry_msgs.msg import Point
    import json
    message = TaggedBspline(session_id='source-session', generation=17,
        frame_id='d1max_loc_odom', valid_start_time=.027123456789,
        valid_start_arc_length=.002031, join_source_stamp=Time(sec=29,nanosec=123456789))
    message.trajectory.order=3
    message.trajectory.traj_id=11
    message.trajectory.knots=[-1.,-.5,0.,.5,1.,1.5,2.,2.5]
    message.trajectory.pos_pts=[Point(x=-7.84,y=-5.01,z=.49964001),
        Point(x=-7.845,y=-5.011,z=.50498002),
        Point(x=-7.78,y=-5.012,z=.51371003),
        Point(x=-7.72,y=-5.01,z=.48900004)]
    message.join_twist.linear.x=-.0108
    message.join_twist.linear.z=.00612
    wire=json.loads(json.dumps(trace_wire_value(message)))
    assert wire['trajectory']['knots']==list(message.trajectory.knots)
    assert [p['z'] for p in wire['trajectory']['pos_pts']]==[p.z for p in message.trajectory.pos_pts]
    assert wire['join_source_stamp']==dict(sec=29,nanosec=123456789)
    assert wire['join_twist']['linear']['x']==-.0108
    assert wire['join_twist']['linear']['z']==.00612
    assert wire['valid_start_time']==.027123456789
    assert message.join_source_stamp.nanosec==123456789
