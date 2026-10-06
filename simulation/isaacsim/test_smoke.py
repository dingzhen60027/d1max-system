"""Validate the real BT client request without starting a ROS participant."""
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from d1max_pct_scan.bt_adapter_contract import valid_compute_request
from smoke import navigation_goal, expected_task_result, drain_imu_witness


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
