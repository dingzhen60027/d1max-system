"""No ROS, SDK, network or hardware: explicit execution state-machine regressions."""
from dataclasses import replace
import ast
from pathlib import Path
from unittest.mock import patch
import pytest
import yaml
from d1max_pct_scan.motion_execution import MotionConfig, ExecutionLease
from d1max_pct_scan.motion_stack import parameters
from d1max_pct_scan import live_session


def config(**values):
    data = dict(session_id='session', map_version_id='map', body_height_calibrated=True,
                collision_envelope_validated=True, vertical_envelope_validated=True,
                sdk_speed_mapping_validated=True)
    return MotionConfig(**dict(data, **values))


def reference(core, *, generation=1, z=0., now=100.):
    return core.reference(session='session', generation=generation, frame='d1max_loc_map',
                          stamp=now, points=[(0., 0., 0.), (2., 0., z)], now=now)


def admission(core, now=100., **values):
    value = dict(schema=1, session_id='session', execution_mode='execution',
        motion_authorized=False, frame_id='d1max_loc_map', lease_timeout_sec=.35,
        localization_context=['session', 1, 'seed'], generation=1, trajectory_id=1,
        source_stamp=now, debug_stamp=now, valid=True, reason='accepted',
        sequence=round(now*1000), received_at_unix=now)
    old = core.samples.get('admission', (None,))[0]
    if old is not None and values.get('trajectory_id',1) == old['trajectory_id']:
        for key in ('source_stamp','debug_stamp','predecessor_id','predecessor_safe','predecessor_check_stamp'):
            if key in old:
                value[key] = old[key]
    return core.observe('admission', dict(value, **values), now, now)


def gate(core, now, **values):
    return core.observe('gate', dict(navigation_session_id='session', map_version_id='map',
                        wall_time=now, **values), now, now)


def tracker(core, now, **values):
    value = dict(session_id='session', generation=1, trajectory_id=2, frame_id='d1max_loc_map',
                 stamp=now, active=True, finished=False, reason='tracking',
                 command=dict(x=.2, y=0., yaw=.1), execution_frozen=False)
    return core.observe('tracker', dict(value, **values), now, now)


def running():
    core = ExecutionLease(config())
    assert reference(core)
    assert admission(core)
    assert core.begin(100., 100.)[0]
    gate(core, 100.1, armed=True)
    admission(core, 100.1)
    assert core.step(100.1, 100.1) == ((0., 0., 0.), True)
    admission(core, 100.2, trajectory_id=2)
    gate(core, 100.2, armed=True)
    tracker(core, 100.2)
    assert core.step(100.2, 100.2) == ((.2, 0., .1), False)
    return core


def test_default_acceptance_cannot_be_invented_by_a_valid_plan():
    core = ExecutionLease(MotionConfig('session', 'map'))
    reference(core); admission(core)
    assert not core.begin(100., 100.)[0]
    assert 'sdk_speed_mapping_validated' in core.reason
    assert not core.permit(100., 100.)['allow']


def test_idle_task_does_not_terminally_cancel_candidate_before_operator_execute():
    core = ExecutionLease(config())
    reference(core); admission(core)
    assert core.task()['generation'] == 0
    assert not core.task()['active']
    assert core.begin(100., 100.)[0]
    assert core.task()['generation'] == 1 and core.task()['active']


def test_task_uses_body_z_and_immutable_issue_time():
    core = running()
    assert core.task()['target_xyz'] == [2., 0., .55]
    assert core.task()['issued_at'] == 100.


@pytest.mark.parametrize('z', [.251, 1., -1., 3.4])
def test_whole_reference_height_prevents_stairs_and_same_xy_other_floor(z):
    core = ExecutionLease(config())
    assert not reference(core, z=z)
    admission(core)
    assert not core.begin(100., 100.)[0]
    assert 'cross_floor' in core.reason


@pytest.mark.parametrize('change', [dict(valid=False), dict(generation=2),
    dict(frame_id='map'), dict(localization_context=['session', 2, 'seed2']),
    dict(lease_timeout_sec=10), dict(schema=2), dict(motion_authorized=True),
    dict(source_stamp=97.), dict(debug_stamp=97.)])
def test_admission_loss_latches_and_healthy_data_cannot_resume(change):
    core = running()
    admission(core, 100.3, **change)
    assert core.step(100.3, 100.3) == ((0., 0., 0.), True)
    assert not core.active and core.disarm_requested
    admission(core, 100.4)
    gate(core, 100.4, armed=True)
    tracker(core, 100.4)
    assert core.step(100.4, 100.4)[0] == (0., 0., 0.)
    assert not core.begin(100.4, 100.4)[0]


def test_foreign_or_duplicate_admission_cannot_refresh_lease():
    core = running()
    assert not admission(core, 100.5, sequence=100200)
    assert not admission(core, 100.51, session_id='other')
    assert core.step(100.56, 100.56)[0] == (0., 0., 0.)
    assert not core.active


def test_disarm_requires_new_generation_not_late_task_or_sdk_ack():
    core = running()
    core.stop('operator_stopped')
    assert not core.task()['active']
    assert not core.permit(100.3, 100.3)['allow']
    assert not core.begin(100.3, 100.3)[0]
    assert reference(core, generation=2, now=100.4)
    assert core.task()['generation'] == 1  # cancel old task, do not poison new candidate
    admission(core, 100.4, generation=2)
    assert core.begin(100.4, 100.4)[0]
    assert core.task()['generation'] == 2


def test_gate_loss_revokes_even_with_fresh_plan_and_tracker():
    core = running()
    gate(core, 100.3, armed=False, reason='sdk_control_not_owned')
    admission(core, 100.3)
    core.step(100.3, 100.3)
    assert not core.active and 'gate_revoked' in core.reason


@pytest.mark.parametrize('v', [dict(x=1.51,y=0.,yaw=0.), dict(x=.2,y=.1,yaw=0.),
    dict(x=-.1,y=0.,yaw=0.), dict(x=.2,y=0.,yaw=.51), dict(x=float('nan'),y=0.,yaw=0.)])
def test_invalid_velocity_never_reaches_command_output(v):
    core = running()
    tracker(core, 100.3, command=v)
    assert core.step(100.3, 100.3)[0] == (0., 0., 0.)
    assert not core.active


def test_replan_topic_order_is_zero_bounded_not_old_velocity():
    core = running()
    admission(core, 100.3, trajectory_id=3)
    gate(core, 100.3, armed=True)
    assert core.step(100.3, 100.3)[0] == (0., 0., 0.)
    assert core.active
    tracker(core, 100.35, trajectory_id=3)
    assert core.step(100.35, 100.35)[0] == (.2, 0., .1)


def checked_handover(core, now=100.3, **changes):
    old = core.samples.get('admission', ({},))[0]
    values = dict(trajectory_id=3, predecessor_id=2, predecessor_safe=True,
                  predecessor_check_stamp=old.get('predecessor_check_stamp',now)
                    if old.get('trajectory_id') == 3 else now)
    admission(core, now, **dict(values, **changes))
    tracker(core, now, trajectory_id=2)
    gate(core, now, armed=True)


def test_native_checked_predecessor_bridges_only_fresh_commands_then_commits_new_id():
    core = running()
    checked_handover(core)
    assert core.step(100.3,100.3) == ((.2,0.,.1),False)
    assert core.reason == 'safe_trajectory_handover'
    assert core.matched_admission[0]['trajectory_id'] == 2
    assert core.matched_admission[0]['source_stamp'] == 100.2
    tracker(core,100.35,trajectory_id=2,command=dict(x=.21,y=0.,yaw=.11))
    assert core.step(100.35,100.35)[0] == (.21,0.,.11)  # never replay cached Twist
    tracker(core,100.4,trajectory_id=3,command=dict(x=.22,y=0.,yaw=.12))
    assert core.step(100.4,100.4)[0] == (.22,0.,.12)
    assert core.matched_admission[0]['trajectory_id'] == 3
    assert core.mismatch_since is None


@pytest.mark.parametrize('changes', [dict(predecessor_safe=False), dict(predecessor_safe=1),
    dict(predecessor_id=1), dict(predecessor_id=True), dict(predecessor_id=3),
    dict(predecessor_check_stamp=99.9), dict(predecessor_check_stamp=101.),
    dict(predecessor_check_stamp=None)])
def test_handover_requires_native_exact_predecessor_and_fresh_original_check(changes):
    core = running(); checked_handover(core,**changes)
    assert core.step(100.3,100.3) == ((0.,0.,0.),True)


def test_safe_handover_is_not_extended_by_new_heartbeats():
    core=running(); checked_handover(core)
    assert core.step(100.3,100.3)[0] == (.2,0.,.1)
    checked_handover(core,100.44)
    assert core.step(100.44,100.44)[0] == (.2,0.,.1)
    checked_handover(core,100.451)
    assert core.step(100.451,100.451)[0] == (0.,0.,0.)
    assert not core.active and core.reason == 'tracker_handover_timeout'


@pytest.mark.parametrize('loss', ['admission', 'gate', 'context', 'generation'])
def test_handover_revocation_is_latched_before_a_later_good_callback(loss):
    core=running(); checked_handover(core)
    core.step(100.3,100.3)
    if loss == 'gate':
        gate(core,100.31,armed=False,reason='sdk_control_not_owned')
        gate(core,100.32,armed=True)
    else:
        bad = dict(valid=False,reason='native_failed_final_collision') if loss == 'admission' else (
            dict(localization_context=['session',2,'other']) if loss == 'context' else dict(generation=2))
        admission(core,100.31,trajectory_id=3,**bad)
        checked_handover(core,100.32)
    assert not core.active and core.disarm_requested
    assert core.step(100.32,100.32)[0] == (0.,0.,0.)
    assert not core.permit(100.32,100.32)['allow']


def test_handover_never_refreshes_predecessor_source_or_receipt_lease():
    core=running()
    # Heartbeat renews only evidence availability, not old source/debug age.
    admission(core,102.,trajectory_id=2,source_stamp=100.2,debug_stamp=100.2)
    tracker(core,102.); gate(core,102.,armed=True)
    assert core.step(102.,102.)[0] == (.2,0.,.1)
    checked_handover(core,102.15)
    assert core.step(102.15,102.15)[0] == (.2,0.,.1)
    tracker(core,102.201)
    assert core.step(102.201,102.201)[0] == (0.,0.,0.)
    assert core.matched_admission[0]['source_stamp'] == 100.2
    core=running(); checked_handover(core,100.5)
    assert core.step(100.5,100.5)[0] == (.2,0.,.1)
    tracker(core,100.551)
    assert core.step(100.551,100.551)[0] == (0.,0.,0.)


def test_handover_does_not_use_stale_or_invalid_tracker_command():
    core=running(); checked_handover(core)
    tracker(core,100.31,command=dict(x=.31,y=0.,yaw=0.))
    assert core.step(100.31,100.31)[0] == (0.,0.,0.)
    assert not core.active and core.reason == 'invalid_tracker_velocity'
    core=running(); admission(core,100.46,trajectory_id=3,predecessor_id=2,
        predecessor_safe=True,predecessor_check_stamp=100.46)
    gate(core,100.46,armed=True)  # tracker last receipt was 100.2: stale
    assert core.step(100.46,100.46)[0] == (0.,0.,0.)


def test_regressed_admission_cannot_restore_an_old_id_or_handover():
    core=running(); admission(core,100.3,trajectory_id=1)
    assert not core.active and core.reason == 'admission_trajectory_id_regressed'
    core=running(); checked_handover(core)
    assert not admission(core,100.31,trajectory_id=2,sequence=100200)
    assert core.sample('admission',100.31,100.31)['trajectory_id'] == 3


@pytest.mark.parametrize('field',['source_stamp','debug_stamp'])
def test_same_id_heartbeat_cannot_refresh_original_curve_time(field):
    core=running(); admission(core,100.3,trajectory_id=2,**{field:100.3})
    assert not core.active and core.reason == 'admission_trajectory_identity_mutated'


@pytest.mark.parametrize('changes', [dict(session='foreign'),dict(generation=2),dict(frame='odom'),
    dict(trajectory_id=3),dict(trajectory_id=True),dict(source_stamp=100.1),
    dict(source_stamp=100.2001),dict(source_stamp=101.),dict(source_stamp=float('nan'))])
def test_spline_forwarding_requires_exact_current_admission(changes):
    core=running()
    request=dict(session='session',generation=1,frame='d1max_loc_map',trajectory_id=2,
                 source_stamp=100.2,now=100.3,wall=100.3)
    assert core.spline_admitted(**request)
    assert not core.spline_admitted(**dict(request,**changes))


def test_spline_forwarding_waits_for_matching_admission_and_never_restamps_source():
    core=running()
    request=dict(session='session',generation=1,frame='d1max_loc_map',trajectory_id=3,
                 source_stamp=100.3,now=100.31,wall=100.31)
    assert not core.spline_admitted(**request)
    admission(core,100.32,trajectory_id=3,source_stamp=100.3)
    assert core.spline_admitted(**dict(request,now=100.32,wall=100.32))
    assert not core.spline_admitted(**dict(request,now=102.31,wall=102.31))


def pending_handover(core, now=100.3, candidate=3):
    admission(core,now,trajectory_id=2,handover_pending=True,pending_candidate_id=candidate)
    tracker(core,now,trajectory_id=2); gate(core,now,armed=True)


def test_early_unsafe_debug_pending_cannot_use_old_matching_admission():
    core=running(); pending_handover(core)
    assert core.active
    assert core.step(100.3,100.3) == ((0.,0.,0.),True)
    assert core.reason == 'waiting_native_handover_pair'
    assert not core.spline_admitted(session='session',generation=1,frame='d1max_loc_map',
        trajectory_id=2,source_stamp=100.2,now=100.3,wall=100.3)
    # Even an explicit negative proof does not cancel the new valid curve; it
    # just prohibits the old command while waiting for that curve's tracker ACK.
    admission(core,100.35,trajectory_id=3,predecessor_safe=False,
              predecessor_id=2,predecessor_check_stamp=100.34)
    assert core.step(100.35,100.35)[0] == (0.,0.,0.)
    tracker(core,100.4,trajectory_id=3)
    assert core.step(100.4,100.4)[0] == (.2,0.,.1)


def test_validated_spline_topic_arriving_first_also_stops_old_matching_id():
    core=running()
    core.candidate_pending(1,3,100.3)
    tracker(core,100.3); gate(core,100.3,armed=True)
    assert core.step(100.3,100.3)[0] == (0.,0.,0.)
    assert not core.spline_admitted(session='session',generation=1,frame='d1max_loc_map',
        trajectory_id=2,source_stamp=100.2,now=100.3,wall=100.3)
    # Old heartbeat cannot erase the locally seen newer candidate.
    admission(core,100.31,trajectory_id=2)
    assert core.step(100.31,100.31)[0] == (0.,0.,0.)
    checked_handover(core,100.35)
    assert core.step(100.35,100.35)[0] == (.2,0.,.1)
    tracker(core,100.4,trajectory_id=3)
    assert core.step(100.4,100.4)[0] == (.2,0.,.1)


def test_pending_heartbeats_and_successive_candidates_do_not_refresh_budget():
    core=running(); pending_handover(core)
    assert core.step(100.3,100.3)[0] == (0.,0.,0.)
    pending_handover(core,100.4,candidate=4)
    assert core.step(100.4,100.4)[0] == (0.,0.,0.)
    pending_handover(core,100.451,candidate=5)
    assert core.step(100.451,100.451)[0] == (0.,0.,0.)
    assert not core.active and core.reason == 'native_handover_pair_timeout'


def test_pending_then_safe_pair_share_one_handover_budget():
    core=running(); pending_handover(core)
    core.step(100.3,100.3)
    checked_handover(core,100.4)
    assert core.step(100.4,100.4)[0] == (.2,0.,.1)
    tracker(core,100.451,trajectory_id=3)
    # ACK arrives after the original pending deadline: cannot silently resume.
    assert core.step(100.451,100.451)[0] == (0.,0.,0.)
    assert not core.active and core.reason == 'native_handover_pair_timeout'


def test_pair_and_new_tracker_within_budget_end_pending_state():
    core=running(); pending_handover(core)
    core.step(100.3,100.3)
    checked_handover(core,100.35)
    tracker(core,100.4,trajectory_id=3)
    assert core.step(100.4,100.4)[0] == (.2,0.,.1)
    assert not core.pair_wait_active and core.mismatch_since is None
    admission(core,100.5,trajectory_id=3)
    tracker(core,100.5,trajectory_id=3); gate(core,100.5,armed=True)
    assert core.step(100.5,100.5)[0] == (.2,0.,.1)


def test_native_proof_source_age_is_bounded_and_cannot_be_freshened_by_heartbeat():
    core=running(); checked_handover(core,100.4,predecessor_check_stamp=100.249)
    assert core.step(100.4,100.4)[0] == (0.,0.,0.)
    admission(core,100.41,trajectory_id=3,predecessor_check_stamp=100.41)
    assert not core.active and core.reason == 'admission_trajectory_identity_mutated'
    core=running(); checked_handover(core,100.4,predecessor_check_stamp=100.251)
    assert core.step(100.4,100.4)[0] == (.2,0.,.1)
    admission(core,100.41,trajectory_id=3)
    tracker(core,100.41)
    assert core.step(100.41,100.41)[0] == (0.,0.,0.)


@pytest.mark.parametrize('change',[dict(pending_candidate_id=2),dict(pending_candidate_id=True),
    dict(pending_candidate_id=None),dict(handover_pending=1)])
def test_invalid_pending_identity_latches_stop(change):
    core=running()
    admission(core,100.3,**dict(dict(trajectory_id=2,handover_pending=True,pending_candidate_id=3),**change))
    assert not core.active
    assert core.step(100.3,100.3)[0] == (0.,0.,0.)


def test_finished_tracker_precedes_native_completed_admission():
    core = running()
    admission(core, 100.3, valid=False, reason='native_completed')
    tracker(core, 100.3, finished=True, active=False, reason='goal_reached')
    assert core.step(100.3, 100.3)[0] == (0., 0., 0.)
    assert core.phase == 'finished' and not core.permit(100.3, 100.3)['allow']


def test_native_completed_arrives_first_revokes_motion_and_waits_only_for_measured_arrival():
    core = running()
    admission(core, 100.3, valid=False, reason='native_completed')
    assert core.step(100.3,100.3) == ((0., 0., 0.), True)
    assert core.phase == 'verifying_completion' and core.task()['active']
    assert not core.permit(100.3,100.3)['allow']
    tracker(core, 100.35, finished=True, active=False, reason='goal_reached')
    core.step(100.35,100.35)
    assert core.phase == 'finished'


def test_native_completed_without_real_arrival_never_reports_success():
    core = running()
    admission(core, 100.3, valid=False, reason='native_completed')
    core.step(100.3,100.3)
    tracker(core, 100.55)
    core.step(100.56,100.56)
    assert not core.active and core.phase != 'finished'


def test_normal_local_endpoint_gets_bounded_replan_wait_not_200ms_fault():
    core = running()
    for t in (100.3, 100.5, 100.7):
        tracker(core, t, reason='local_segment_finished_waiting_replan')
        admission(core, t, trajectory_id=2); gate(core, t, armed=True)
        assert core.step(t,t) == ((0., 0., 0.), True)
        assert core.active
    tracker(core, 100.8, trajectory_id=3)
    admission(core, 100.8, trajectory_id=3); gate(core, 100.8, armed=True)
    assert core.step(100.8,100.8)[0] == (.2, 0., .1)


def test_motion_preparation_writes_no_false_acceptance_and_starts_nothing(tmp_path):
    with patch.object(live_session, 'ROOT', tmp_path), \
         patch.object(live_session.subprocess, 'Popen') as popen, \
         patch.object(live_session.subprocess, 'run') as run, \
         patch.object(live_session, 'build_opener') as network:
        directory, session = live_session.prepare_motion()
    popen.assert_not_called(); run.assert_not_called(); network.assert_not_called()
    assert session['mode'] == 'LIVE_NAVIGATION'
    assert session['motion']['sdk_speed_mapping_validated'] is False
    assert session['motion']['max_speed'] == .3
    bridge = yaml.safe_load((directory/'bridge.yaml').read_text())['/**']['ros__parameters']
    assert bridge['execution_mode'] == 'execution'
    assert bridge['execution_tracker_node'] == '/d1max/live_planning/motion_coordinator'
    nodes = yaml.safe_load((directory/'motion.yaml').read_text())
    gate_params = nodes['/d1max/live_planning/navigation_command_gate']['ros__parameters']
    assert gate_params['require_execution_permit'] is True
    assert gate_params['input_topic'].endswith('/cmd_vel_collision_checked')
    track = nodes['/d1max/live_planning/trajectory_tracker']['ros__parameters']
    assert track['frozen_topic'].endswith('/tracker_frozen')
    assert track['trajectory_topic'].endswith('/execution_bspline')
    rviz = yaml.safe_load((directory/'live.rviz').read_text())
    assert rviz['Panels'][0]['Class'].endswith('/MotionControlPanel')


def test_no_sdk_or_automatic_robot_state_changes_in_coordinator():
    source = Path(live_session.__file__).with_name('motion_coordinator.py').read_text()
    assert all(name not in source for name in ('SDKClient(', 'StandUp(', 'SetMode(', 'SoftEmergencyStop('))
    tree = ast.parse(source)
    assert not any(isinstance(n, ast.Call) and ast.unparse(n.func).endswith('exec') for n in ast.walk(tree))
