"""Production RViz callbacks with real messages; no ROS node, GUI or SDK."""
from copy import deepcopy
import json
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from std_msgs.msg import String

from d1max_pct_scan import live_view
from d1max_pct_scan.live_view import LiveView
from d1max_pct_scan.committed_route_view import CommittedRouteView
from d1max_pct_scan.pose_continuity_view import PoseContinuityView
from d1max_pct_scan.typed_view_state import TypedViewState
from test_continuous_reference_transport import state
from test_route_ingress import delivery


def fixture(monkeypatch):
    clock = SimpleNamespace(source=10.1, mono=100.1)
    monkeypatch.setattr(live_view.time, 'monotonic', lambda: clock.mono)
    reports = []
    node = SimpleNamespace(single_floor=True, session={'id': 'session', 'version_id': 'map'},
        typed_view=TypedViewState('session', 'map'),
        pose_continuity_view=PoseContinuityView('session', 'map'),
        committed_route_view=CommittedRouteView('session', 'map'),
        frame='d1max_loc_map', freeze=None, last_state={}, state_at=0., last_seed_id=None,
        global_state={}, scan_state={}, global_at=0., scan_at=0., bt_state={}, bt_at=0.,
        reason='', reason_scope='input', last_snapshot_at=clock.mono,
        marker=SimpleNamespace(publish=lambda _: None),
        diagnostics_pub=SimpleNamespace(publish=lambda m: reports.append(json.loads(m.data))),
        publish_goal_editor=lambda: None, expire_requests=lambda _: None)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=round(clock.source*1e9),
        to_msg=lambda: Time(sec=int(clock.source), nanosec=round(clock.source%1*1e9))))
    message = delivery()
    LiveView.on_atomic_state(node, state(t=.1))
    LiveView.on_committed_route(node, message)
    owner = dict(schema=2, owner='BehaviorTree.CPP', session_id='session',
        task_id=message.task_id, route_id=message.route_id, route_hash=message.route_hash,
        localization_epoch=1, localization_seed_id='seed', phase='following',
        active=True, root_status='RUNNING', received_at_unix=clock.source)
    return node, clock, owner, message, reports


def render(node, clock, owner, reports):
    owner = dict(owner, received_at_unix=clock.source)
    LiveView.on_bt(node, String(data=json.dumps(owner)))
    node.last_snapshot_at = clock.mono  # Do not write Web snapshots in these tests.
    LiveView.tick(node)
    return reports[-1]


def test_real_typed_commit_displays_generated_without_legacy_localization_or_worker(monkeypatch):
    node, clock, owner, message, reports = fixture(monkeypatch)
    result = render(node, clock, owner, reports)
    assert node.last_state == {} and node.global_state == {}
    assert result['stages']['global']['label'] == '路径已生成'
    assert result['stages']['localization']['label'] == '已定位'
    assert result['motion_enabled'] is False and not result['navigation_admission']['ready']
    assert node.committed_route_view.snapshot.path.header.stamp == message.source_stamp


def test_source_gap_and_local_failure_do_not_unplan_committed_route(monkeypatch):
    node, clock, owner, _, reports = fixture(monkeypatch)
    original = render(node, clock, owner, reports)['stages']['global']
    clock.source += .5
    clock.mono += .5
    node.scan_state = dict(session_id='session', received_at_unix=clock.source,
        localization_epoch=1, localization_seed_id='seed', ready=True,
        active_reference=True, local_debug_phase='failed_reference_search',
        owner_task_id=owner['task_id'], owner_route_id=owner['route_id'], owner_route_hash=owner['route_hash'])
    node.scan_at = clock.mono
    result = render(node, clock, owner, reports)
    assert result['stages']['global'] == original
    assert result['stages']['localization']['label'] == '等待定位'
    assert result['stages']['local']['label'] == '未找到绕行通道'
    assert result['plan_id'] is None


@pytest.mark.parametrize('phase', ['canceling', 'retiring_task', 'cancelled', 'failed', 'succeeded'])
def test_terminal_owner_keeps_history_but_not_current_route_green(monkeypatch, phase):
    node, clock, owner, _, reports = fixture(monkeypatch)
    owner['phase'] = phase
    result = render(node, clock, owner, reports)
    assert result['stages']['global']['label'].startswith('历史路径')
    assert result['stages']['global']['tone'] != 'ready'
    assert node.committed_route_view.displayed_path() is not None


def test_inactive_owner_with_retained_binding_is_history_even_if_phase_lags(monkeypatch):
    node, clock, owner, _, reports = fixture(monkeypatch)
    owner.update(active=False, root_status='IDLE')
    result = render(node, clock, owner, reports)
    assert result['stages']['global']['label'].startswith('历史路径')
    assert result['stages']['global']['tone'] != 'ready'


def test_new_task_does_not_borrow_previous_committed_path(monkeypatch):
    node, clock, owner, _, reports = fixture(monkeypatch)
    owner.update(task_id='next-task', route_id='', route_hash='', phase='computing_route')
    result = render(node, clock, owner, reports)
    assert result['stages']['global']['label'] == '计算全局路线'
    assert result['stages']['global']['tone'] != 'ready'


def test_switching_goal_is_not_failure_or_previous_trajectory_success(monkeypatch):
    node, clock, owner, _, reports = fixture(monkeypatch)
    node.scan_state = dict(session_id='session', received_at_unix=clock.source,
        localization_epoch=1, localization_seed_id='seed', ready=True,
        active_reference=True, spline_visual_valid=True, last_spline_id=4,
        owner_task_id=owner['task_id'], owner_route_id=owner['route_id'],
        owner_route_hash=owner['route_hash'], local_validation_sequence=7,
        local_validation_valid_until_ns=10_200_000_000)
    node.scan_at = clock.mono
    owner.update(task_id='next-task', route_id='', route_hash='', phase='switching_goal',
                 root_status='IDLE', worker_reason='retiring_previous_task')
    result = render(node, clock, owner, reports)
    assert result['stages']['global']['label'] == '切换目标'
    assert result['stages']['local']['label'] == '等待新路线'
    assert result['plan_id'] is None
    assert node.committed_route_view.displayed_path() is not None
    assert result['stages']['localization']['label'] == '已定位'


def test_old_worker_and_local_failure_do_not_leak_into_new_task(monkeypatch):
    node, clock, owner, _, reports = fixture(monkeypatch)
    node.global_state = dict(session_id='session', received_at_unix=clock.source,
        state='planning_failed', reason='old-worker-failed')
    node.global_at = clock.mono
    node.scan_state = dict(session_id='session', received_at_unix=clock.source,
        localization_epoch=1, localization_seed_id='seed', ready=True, active_reference=True,
        owner_task_id=owner['task_id'], owner_route_id=owner['route_id'], owner_route_hash=owner['route_hash'],
        local_debug_phase='failed_reference_search')
    node.scan_at = clock.mono
    owner.update(task_id='next-task', route_id='', route_hash='', phase='computing_route')
    result = render(node, clock, owner, reports)
    assert result['stages']['global']['label'] == '计算全局路线'
    assert result['stages']['local']['label'] == '等待参考'


def test_relocalization_identity_change_never_relabels_old_path_as_current(monkeypatch):
    node, clock, owner, _, reports = fixture(monkeypatch)
    LiveView.on_atomic_state(node, state(t=.1, epoch=2))
    result = render(node, clock, owner, reports)
    assert result['stages']['global']['label'] == '路线保留 · 定位已改变'
    assert result['stages']['global']['tone'] != 'ready'


def test_route_retirement_wins_over_a_late_active_owner_heartbeat(monkeypatch):
    node, clock, owner, message, reports = fixture(monkeypatch)
    cancel = deepcopy(message)
    cancel.delivery_sequence, cancel.active = 2, False
    LiveView.on_committed_route(node, cancel)
    result = render(node, clock, owner, reports)
    assert result['stages']['global']['label'].startswith('历史路径')
    assert result['stages']['global']['tone'] != 'ready'


def test_status_loss_reports_unknown_task_not_unplanned_or_live_route(monkeypatch):
    node, clock, owner, _, reports = fixture(monkeypatch)
    render(node, clock, owner, reports)
    clock.source += 1.1
    clock.mono += 1.1
    node.last_snapshot_at = clock.mono
    LiveView.tick(node)
    assert reports[-1]['stages']['global']['label'] == '路线保留 · 等待任务状态'
    assert reports[-1]['stages']['global']['tone'] != 'ready'


@pytest.mark.parametrize('mutation', ['session', 'hash', 'owner'])
def test_foreign_owner_or_binding_cannot_make_cached_path_current(monkeypatch, mutation):
    node, clock, owner, _, reports = fixture(monkeypatch)
    if mutation == 'session': owner['session_id'] = 'other'
    elif mutation == 'hash': owner['route_hash'] = 'd'*64
    else: owner['owner'] = 'another_manager'
    result = render(node, clock, owner, reports)
    assert result['stages']['global']['tone'] != 'ready'


def test_owner_route_and_envelope_can_arrive_in_either_order(monkeypatch):
    node, clock, owner, message, reports = fixture(monkeypatch)
    node.committed_route_view = CommittedRouteView('session', 'map')
    before = render(node, clock, owner, reports)
    assert before['stages']['global']['label'] == '路径已生成 · 等待路线显示'
    LiveView.on_committed_route(node, message)
    after = render(node, clock, owner, reports)
    assert after['stages']['global']['label'] == '路径已生成'


def test_malformed_optional_phase_cannot_crash_display_callback(monkeypatch):
    node, clock, owner, _, reports = fixture(monkeypatch)
    owner.update(phase=[], root_status={})
    result = render(node, clock, owner, reports)
    assert result['stages']['global']['label'] == '路径已生成'


def test_formal_local_status_uses_own_route_binding_and_original_proof_deadline(monkeypatch):
    node, clock, owner, _, reports = fixture(monkeypatch)
    node.scan_state = dict(session_id='session', received_at_unix=clock.source,
        localization_epoch=1, localization_seed_id='seed', ready=True,
        active_reference=True, spline_visual_valid=True, last_spline_id=4,
        owner_task_id=owner['task_id'], owner_route_id=owner['route_id'],
        owner_route_hash=owner['route_hash'], local_validation_sequence=7,
        local_validation_valid_until_ns=10_200_000_000)
    node.scan_at = clock.mono
    result = render(node, clock, owner, reports)
    assert result['stages']['local']['label'] == '轨迹有效' and result['plan_id'] == 4
    node.scan_state['owner_route_hash'] = 'b'*64
    result = render(node, clock, owner, reports)
    assert result['plan_id'] is None and result['stages']['local']['tone'] != 'ready'
    node.scan_state['owner_route_hash'] = owner['route_hash']
    node.scan_state['local_validation_valid_until_ns'] = 10_100_000_000
    result = render(node, clock, owner, reports)
    assert result['plan_id'] is None and result['stages']['global']['label'] == '路径已生成'


def test_global_compute_result_reaches_path_publisher_and_live_view_without_follow(monkeypatch):
    """Real ComputeRoute result wire through both production display callbacks."""
    import asyncio
    from test_bt_action_adapters import (harness as adapter_harness, compute_started,
        candidate, refresh)
    from d1max_pct_scan.committed_route_view import publish_route_snapshot
    from d1max_pct_scan.bt_adapters import stamp_seconds
    adapter, adapter_clock = adapter_harness(monkeypatch)
    handle = compute_started(adapter)
    goal_stamp = adapter.compute['goal_stamp']
    adapter_clock.wall, adapter_clock.mono = 100.1, 10.1
    refresh(adapter, adapter_clock)
    source = candidate(adapter_clock.wall)
    adapter.on_candidate(source)
    adapter.on_worker_status(String(data=json.dumps(dict(session_id='session',
        received_at_unix=adapter_clock.wall, last_goal_stamp=goal_stamp,
        route_committed=True, active_goal=dict(user_stamp=goal_stamp, epoch=1,
            seed_id='seed', map_version_id='map'), state='active', generation=7,
        last_route=dict(path_stamp=stamp_seconds(source.path.header.stamp), route_hash=source.route_hash)))))
    adapter.compute_tick()
    result = asyncio.run(adapter.execute_compute(handle))
    assert result.success and handle.terminal == 'succeeded'
    assert not adapter.route_reference_pub.messages and adapter.follow is None
    wire = result.snapshot  # Exactly what navigator publishes on bt/route_snapshot.
    assert wire.preview_ready and not wire.execution_eligible

    node, clock, owner, _, reports = fixture(monkeypatch)
    node.session['planning_scope'] = 'global_route'
    node.committed_route_view = CommittedRouteView('session', 'map')
    display = CommittedRouteView('session', 'map')
    published = []
    publisher = SimpleNamespace(publish=lambda path: published.append(deepcopy(path)))
    publish_route_snapshot(display, publisher, wire)
    publish_route_snapshot(display, publisher, wire)
    LiveView.on_route_snapshot(node, wire)
    assert len(published) == 1 and published[0] == wire.path
    assert node.committed_route_view.displayed_path() == published[0]
    owner.update(task_id=wire.task_id, route_id=wire.route_id, route_hash=wire.route_hash,
        phase='succeeded', root_status='SUCCESS', active=False)
    report = render(node, clock, owner, reports)
    assert report['stages']['global']['label'] == '全局规划完成'
    assert report['stages']['local']['label'] == '未启用 · 仅全局规划'
    assert report['behavior_tree']['label'] == '全局规划完成'
    assert report['navigation_admission']['ready'] is False
    assert report['motion_enabled'] is False and report['plan_id'] is None
    assert report['local_target'] is None and report['reference_horizon_m'] is None


@pytest.mark.parametrize('mutation', ['missing_snapshot', 'other_task', 'other_epoch', 'other_owner'])
def test_global_completion_needs_its_own_typed_snapshot(monkeypatch, mutation):
    node, clock, owner, message, reports = fixture(monkeypatch)
    node.session['planning_scope'] = 'global_route'
    node.committed_route_view = CommittedRouteView('session', 'map')
    if mutation != 'missing_snapshot':
        LiveView.on_route_snapshot(node, message.snapshot)
    owner.update(phase='succeeded', root_status='SUCCESS', active=False)
    if mutation == 'other_task': owner['task_id'] = 'foreign'
    elif mutation == 'other_epoch': owner['localization_epoch'] = 2
    elif mutation == 'other_owner': owner['owner'] = 'foreign'
    report = render(node, clock, owner, reports)
    assert report['stages']['global']['tone'] != 'ready'
    assert report['stages']['local']['label'] == '未启用 · 仅全局规划'
    assert report['plan_id'] is None


def test_navigation_callback_does_not_consume_uncommitted_snapshot(monkeypatch):
    node, _, _, message, _ = fixture(monkeypatch)
    node.committed_route_view = CommittedRouteView('session', 'map')
    LiveView.on_route_snapshot(node, message.snapshot)
    assert node.committed_route_view.displayed_path() is None
