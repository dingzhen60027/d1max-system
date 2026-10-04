"""Production callbacks, same healthy/gap/recovery samples; no ROS graph or IO.

The before-policy is retained explicitly as a baseline: soft global loss sends
an empty reference, the bridge then cancels/recreates native task generations.
The fixed policy suspends evidence admission without changing owner intent.
"""
from types import MethodType, SimpleNamespace as NS
import json

import pytest
from std_msgs.msg import String

from d1max_pct_scan.live_global_planner import LiveGlobalPlanner
from d1max_pct_scan.live_goal_pause import RetainedGlobalComputation
from d1max_pct_scan.live_scan_bridge import LiveScanBridge
from d1max_pct_scan.live_scan_contract import admissible_tagged_spline
from test_live_scan_soft_loss import harness, body_message, IDENTITY
from test_live_scan_reference_order import path
from test_live_goal_pause import INTENT


def real_contract_harness(monkeypatch):
    node, clock, paths, clears, contexts = harness(monkeypatch)
    node.p.update(collision_policy='official', execution_mode='preview',
                  localization_session_id='session', tracking_frame='d1max_loc_tracking')
    node.current_context = MethodType(LiveScanBridge.current_context, node)
    node.nav_body_ready = MethodType(LiveScanBridge.nav_body_ready, node)
    node.pose_identity_fault = None
    node.reference_seen_stamp = 100.12
    node.counts = dict(references=0, rejected_references=0)
    node.execution_frozen = True
    node.clear_marker = lambda **kw: clears.append(('marker', kw))
    node.gate.accept([[0, 0, 0], [.5, 0, 0], [1, 0, 0]],
        frame_id=node.gate.frame_id, stamp=100.2, now=100.2, body_xyz=[0, 0, .55])
    sample(node, clock, 100.25)
    node.update_gate()
    assert node.gate.ready and node.gate.active
    return node, clock, paths, clears, contexts


def sample(node, clock, wall, *, valid=True):
    clock.wall, clock.mono = wall, wall-90.
    nav = dict(schema=1, epoch=1, seed_id='seed', received_at_unix=wall,
               valid=True, fault='', reset_pending=False)
    node.navigation = nav
    node.localizer = dict(session_id='session', wall_time=wall, local_epoch=1,
        active_seed_ns='seed', confirmed_seed_ns='seed', verified_confirmations=3,
        local_fault='', navigation=dict(nav),
        frames=dict(map='d1max_loc_map', tracking='d1max_loc_tracking'))
    node.pose_status = dict(schema=1, epoch=1, seed_id='seed', received_at_unix=wall,
        output_stamp_sec=wall-.01, pose_timeout_sec=.08, valid=valid, pose_valid=valid,
        fault='', reset_pending=False, motion_control_enabled=False,
        frame_id='d1max_loc_map', body_frame='d1max_loc_base_link')
    node.nav_received = node.localizer_received = node.pose_received = clock.mono
    node.body_received = node.freeze_received = clock.mono
    node.body = body_message(wall-.01)
    node.body_context = IDENTITY
    node.cloud_stamp = wall-.06
    node.cloud_received = clock.mono-.02


def test_short_invalid_pose_preserves_task_but_not_pose_or_curve_admission(monkeypatch):
    node, clock, paths, clears, _ = real_contract_harness(monkeypatch)
    original = (node.gate.generation, node.gate.issued_at, node.gate.digest)
    sample(node, clock, 100.3, valid=False)
    assert node.current_context(clock.wall) is None
    assert not node.nav_body_ready(clock.wall, clock.mono)
    node.update_gate()
    assert node.gate.active and node.gate.preview_paused and not node.gate.ready
    assert not paths and (node.gate.generation, node.gate.issued_at, node.gate.digest) == original
    assert ('admission', 'preview_sensor_paused') in clears
    common = dict(session_id='session', expected_session='session',
        generation=node.gate.generation, frame_id=node.gate.frame_id,
        trajectory_id=2, gate=node.gate, last_id=1, now=100.4)
    assert not admissible_tagged_spline(start_time=100.31, **common)
    sample(node, clock, 100.35)
    node.update_gate()
    assert node.gate.active and node.gate.ready and not node.gate.preview_paused
    assert (node.gate.generation, node.gate.issued_at, node.gate.digest) == original
    assert not admissible_tagged_spline(start_time=100.31, **common)
    assert admissible_tagged_spline(start_time=100.36, **common)
    assert not paths


@pytest.mark.parametrize('fault', ['seed', 'epoch', 'frame', 'reset', 'pose_fault',
                                 'map_fault', 'freeze', 'status_timeout'])
def test_hard_failure_still_cancels_instead_of_retaining(monkeypatch, fault):
    node, clock, paths, _, _ = real_contract_harness(monkeypatch)
    sample(node, clock, 100.3, valid=False)
    node.update_gate()
    old_generation = node.gate.generation
    if fault == 'seed':
        node.pose_status['seed_id'] = 'different'
    elif fault == 'epoch':
        node.pose_status['epoch'] = 2
    elif fault == 'frame':
        node.pose_status['frame_id'] = 'other'
    elif fault == 'reset':
        node.pose_status['reset_pending'] = True
    elif fault == 'pose_fault':
        node.pose_status['fault'] = 'imu_discontinuity'
    elif fault == 'map_fault':
        node.map_context_fault = 'invalid_extrinsic'
    elif fault == 'freeze':
        node.execution_frozen = False
    else:
        clock.wall, clock.mono = 101., 11.
    node.update_gate()
    assert not node.gate.active and not node.gate.preview_paused
    assert node.gate.generation == old_generation+1
    assert len(paths) == 1 and not paths[0].path.poses
    sample(node, clock, 101.1)
    node.update_gate()
    assert not node.gate.active  # no resurrection from returning data


def test_explicit_cancel_or_new_goal_handover_wins_during_pose_pause(monkeypatch):
    node, clock, paths, _, _ = real_contract_harness(monkeypatch)
    sample(node, clock, 100.3, valid=False)
    node.update_gate()
    generation = node.gate.generation
    LiveScanBridge.on_reference(node, path(100.31, empty=True))
    assert not node.gate.active and node.gate.generation == generation+1
    sample(node, clock, 100.4)
    node.update_gate()
    # Delayed old reference cannot resurrect a cancelled goal.
    LiveScanBridge.on_reference(node, path(100.2))
    assert not node.gate.active and len(paths) == 1


def test_wrong_frame_packet_is_hard_invalidation_even_after_parser_erases_payload(monkeypatch):
    node, clock, paths, _, _ = real_contract_harness(monkeypatch)
    node.pose_status_stamp = 100.2
    node.pose_status['frame_id'] = 'other'
    LiveScanBridge.on_pose_status(node, String(data=json.dumps(node.pose_status)))
    assert node.pose_status == {} and node.pose_identity_fault == 'continuous_pose_frame_mismatch'
    assert not node.gate.active and len(paths) == 1


def test_identical_gap_sequence_no_longer_manufactures_cancellations(monkeypatch):
    def replay(*, before):
        node, clock, paths, _, _ = real_contract_harness(monkeypatch)
        start = node.gate.generation
        pauses = 0
        for cycle in range(8):
            wall = 100.3+cycle*.1
            sample(node, clock, wall, valid=False)
            node.update_gate()
            paused = RetainedGlobalComputation()
            paused.install(INTENT)
            owner = NS(p=dict(retain_preview_task_on_soft_loss=True),
                pause=paused, now_s=lambda:wall+.001, active_context=(1, 'seed'),
                cached_result=None, commit_wait_started=None, current=object(),
                publish_empty=lambda reason: LiveScanBridge.on_reference(node,path(wall+.001,empty=True)))
            if before:
                owner.publish_empty('paused:continuous_pose_invalid_or_stale')
            else:
                LiveGlobalPlanner.pause_reference(owner,'continuous_pose_invalid_or_stale')
            pauses += 1
            sample(node, clock, wall+.04)
            node.update_gate()
            # Actual global owner freshly validates same path after recovery.
            LiveScanBridge.on_reference(node,path(wall+.05))
        return dict(cancels=sum(not msg.path.poses for msg in paths),
                    generations=node.gate.generation-start, pauses=pauses,
                    active=node.gate.active)
    before, after = replay(before=True), replay(before=False)
    assert before['cancels'] == 8 and before['generations'] == 16
    assert after == dict(cancels=0, generations=0, pauses=8, active=True)


@pytest.mark.parametrize('policy,mode,backend', [
    ('observed_free', 'preview', 'per_sensor_rays'),
    ('observed_free', 'execution', 'per_sensor_rays'),
    ('official', 'preview', 'deskewed_cloud')])
def test_same_pose_gap_remains_a_cancel_for_strict_execution_and_legacy(monkeypatch, policy, mode, backend):
    node, clock, paths, _, _ = real_contract_harness(monkeypatch)
    node.p.update(collision_policy=policy, execution_mode=mode,
        perception_backend=backend, execution_tracker_node='/tracker')
    node.get_publishers_info_by_topic = lambda _: [NS(node_namespace='', node_name='tracker')]
    generation = node.gate.generation
    sample(node, clock, 100.3, valid=False)
    node.update_gate()
    assert not node.gate.active and not node.gate.ready and not node.gate.preview_paused
    assert node.gate.generation == generation+1 and len(paths) == 1
    assert not paths[0].path.poses
    sample(node, clock, 100.35)
    node.update_gate()
    assert not node.gate.active


@pytest.mark.parametrize('parameters', [{}, {'retain_preview_task_on_soft_loss': False}])
def test_global_default_and_execution_contract_still_withdraw_reference(parameters):
    pause = RetainedGlobalComputation()
    pause.install(INTENT)
    empty = []
    owner = NS(p=parameters, pause=pause, now_s=lambda:100.3, active_context=(1, 'seed'),
        cached_result=None, commit_wait_started=None, current=object(),
        publish_empty=empty.append)
    LiveGlobalPlanner.pause_reference(owner, 'continuous_pose_invalid_or_stale')
    assert empty == ['paused:continuous_pose_invalid_or_stale']


def test_inactive_sensor_flapping_does_not_cancel_refresh_or_invent_generations(monkeypatch):
    node, clock, paths, _, _ = real_contract_harness(monkeypatch)
    # A previous real revocation has left the task inactive. Only the global
    # owner may freshly validate and republish it; sensor recovery cannot.
    node.gate.revoke(100.26, 'input_stale_or_invalid')
    node.gate.ready = False
    node.reference_refresh = dict(context=IDENTITY, source_stamp=100.2,
                                  deadline=20., attempts=0)
    node.reference_refresh_last = -float('inf')
    node.reference_refresh_id = 0
    requests = []
    node.reference_refresh_pub = NS(publish=requests.append)
    node.counts.update(reference_refresh_requests=0, reference_refresh_expired=0)
    generation = node.gate.generation
    for i in range(8):
        wall = 100.3+i*.08
        sample(node, clock, wall)
        node.update_gate()
        assert node.gate.ready and not node.gate.active
        sample(node, clock, wall+.02)
        node.cloud_stamp = wall-.6
        node.update_gate()
        assert not node.gate.ready and not node.gate.active
    assert node.gate.generation == generation and paths == []
    assert node.reference_refresh['source_stamp'] == 100.2
    sample(node, clock, 101.1)
    node.update_gate()
    LiveScanBridge.request_reference_refresh(node, clock.wall, clock.mono)
    assert len(requests) == 1
    assert json.loads(requests[0].data)['rejected_reference_stamp'] == 100.2
    # Fresh owner response crosses the current barrier once; old geometry
    # metadata alone never grants authority.
    LiveScanBridge.on_reference(node, path(101.11))
    assert node.gate.active and node.gate.ready and node.gate.generation == generation+1
    assert len(paths) == 1 and len(paths[0].path.poses) == 3
    assert node.reference_refresh is None
