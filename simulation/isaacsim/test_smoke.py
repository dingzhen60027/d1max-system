"""Validate the real BT client request without starting a ROS participant."""
from types import SimpleNamespace
import math

from builtin_interfaces.msg import Time
from d1max_pct_scan.bt_adapter_contract import valid_compute_request
from smoke import (navigation_goal, expected_task_result, drain_imu_witness,
    trace_output_path, cancellation_due, terminal_observation_complete,
    stationarity_window, body_height_evidence, source_observation_expired,
    trace_wire_value, measured_velocity_components, measured_velocity_domain_evidence)


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


def sealed_spot_velocity_record(tmp_path):
    import hashlib
    import json
    path=tmp_path/'braking_model.json'
    record=dict(schema_version=3,model='reaction_braking_reachable_v1',
        transport_mode='isolated_mock',fixture_only=True,session_id='domain-session',
        measurements=dict(max_speed_mps=.6,max_yaw_radps=.8),
        isolated_platform_model=dict(schema=1,kind='official_spot_physx',
            command_max_speed_mps=.25,command_max_yaw_radps=.3,
            reachable_max_speed_mps=.6,reachable_max_yaw_radps=.8,
            source_scope='isolated_simulation_physx_measured_model'),
        isolated_full_xyz_reference_model=dict(schema=1,kind='official_spot_physx',
            reference_max_speed_mps=.6,measured_travel_max_speed_mps=.6,
            observed_max_full_xyz_speed_mps=.5759470588816599,evidence_sha256='a'*64,
            source_scope='isolated_simulation_physx_measured_model'))
    session=dict(id='domain-session',simulation_backend='isaacsim_physx',
        simulation_clock='isaac_fixed_anchor_v1',transport_mode='isolated_mock',
        physical_acceptance=False,max_speed_mps=.25,max_yaw_radps=.3,
        robot_profile=str(tmp_path/'quadruped_fixture_profile.yaml'),
        execution_braking_model_record=str(path))
    def seal():
        raw=json.dumps(record).encode()
        path.write_bytes(raw)
        digest=hashlib.sha256(raw).hexdigest()
        session.update(execution_braking_model_sha256=digest,input_hashes={str(path):digest})
    seal()
    return session,record,seal


def velocity_domain_check(session, speed, **changes):
    stats=dict(measured_state_count=10,velocity_sample_count=10,
        invalid_velocity_sample_count=0,max_full_xyz_speed_mps=speed,
        max_full_angular_speed_radps=.9)
    return measured_velocity_domain_evidence(session,**dict(stats,**changes))


def test_actual_v39_full_xyz_peak_is_rejected_without_clamping_any_axis(tmp_path):
    session,_,_=sealed_spot_velocity_record(tmp_path)
    measured=measured_velocity_components(
        SimpleNamespace(x=.5760377645492554,y=.1930128037929535,z=.04835313558578491),
        SimpleNamespace(x=-.2184164822101593,y=.013279813341796398,z=.03262309357523918))
    assert measured['linear_mps']==math.hypot(*measured['linear_velocity_xyz'])
    assert measured['linear_velocity_xyz'][1]==.1930128037929535
    verified,evidence=velocity_domain_check(session,measured['linear_mps'])
    assert not verified and evidence['required']
    assert evidence['reason']=='measured_full_xyz_speed_outside_sealed_domain'
    assert evidence['measured_max_full_xyz_speed_mps']>.6
    assert set(evidence['exceeded_domains'])=={
        'reachable_max_speed_mps','reference_max_speed_mps','measured_travel_max_speed_mps'}
    verified,evidence=velocity_domain_check(session,.5759470588816599)
    assert verified
    assert evidence['full_angular_norm_is_diagnostic_only']  # .9 is not compared to yaw .8.
    assert velocity_domain_check(session,.6)[0]
    assert not velocity_domain_check(session,math.nextafter(.6,math.inf))[0]


def test_independent_reference_and_travel_limits_are_both_checked(tmp_path):
    session,record,seal=sealed_spot_velocity_record(tmp_path)
    reference=record['isolated_full_xyz_reference_model']
    reference.update(observed_max_full_xyz_speed_mps=.4,reference_max_speed_mps=.5)
    seal()
    assert velocity_domain_check(session,.49)[0]
    verified,evidence=velocity_domain_check(session,.55)
    assert not verified and evidence['exceeded_domains']==['reference_max_speed_mps']
    reference.update(reference_max_speed_mps=.6,measured_travel_max_speed_mps=.5)
    seal()
    verified,evidence=velocity_domain_check(session,.55)
    assert not verified and evidence['exceeded_domains']==['measured_travel_max_speed_mps']


def test_nonfinite_or_missing_measurements_cannot_hide_in_a_finite_maximum(tmp_path):
    session,_,_=sealed_spot_velocity_record(tmp_path)
    measured=measured_velocity_components(SimpleNamespace(x=math.nan,y=0.,z=0.),
        SimpleNamespace(x=0.,y=0.,z=0.))
    assert not measured['velocity_finite'] and math.isnan(measured['linear_velocity_xyz'][0])
    for changes in (dict(max_full_xyz_speed_mps=math.nan),dict(max_full_angular_speed_radps=math.inf),
            dict(invalid_velocity_sample_count=1),dict(velocity_sample_count=9),
            dict(measured_state_count=0,velocity_sample_count=0)):
        verified,evidence=velocity_domain_check(session,.2,**changes)
        assert not verified
        assert evidence['reason']=='spot_velocity_measurements_missing_or_nonfinite'


def test_wrong_record_hash_session_or_marker_never_reverts_to_wheel_gate(tmp_path):
    session,record,seal=sealed_spot_velocity_record(tmp_path)
    for key,value in (('simulation_backend','foreign'),('simulation_clock','wall'),
            ('transport_mode','live'),('physical_acceptance',True),('id','foreign')):
        assert not velocity_domain_check(dict(session,**{key:value}),.2)[0]
    assert not velocity_domain_check(dict(session,execution_braking_model_sha256='f'*64),.2)[0]
    assert not velocity_domain_check(dict(session,input_hashes={}),.2)[0]
    record['isolated_full_xyz_reference_model']['evidence_sha256']='unsealed'
    seal()
    assert not velocity_domain_check(session,.2)[0]
    record.pop('isolated_full_xyz_reference_model')
    seal()
    verified,evidence=velocity_domain_check(session,.2)
    assert not verified and evidence['reason']=='spot_full_xyz_reference_marker_missing'
    record.pop('isolated_platform_model')
    seal()
    assert not velocity_domain_check(session,.2)[0]  # Sealed quadruped identity still requires a marker.


def test_absent_legacy_wheel_marker_keeps_existing_acceptance_gate(tmp_path):
    verified,evidence=velocity_domain_check({},.8,measured_state_count=0,velocity_sample_count=0)
    assert verified and not evidence['required']
    session,record,seal=sealed_spot_velocity_record(tmp_path)
    session['robot_profile']=str(tmp_path/'wheel_fixture_profile.yaml')
    record.pop('isolated_platform_model')
    record.pop('isolated_full_xyz_reference_model')
    seal()
    verified,evidence=velocity_domain_check(session,.8)
    assert verified and not evidence['required']
