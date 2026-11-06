"""Drive the real RViz presentation callback; no ROS graph or robot connection."""
import json
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

from builtin_interfaces.msg import Time

from d1max_pct_scan import live_view
from d1max_pct_scan.bt_presentation import tree_presentation
from test_live_goal_action import harness


def view(monkeypatch, tmp_path):
    node, acceptance = harness(monkeypatch)
    monkeypatch.setattr(live_view.time, 'monotonic', lambda: 100.)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=10_000_000_000, to_msg=lambda: Time(sec=10)))
    node.freeze = None
    node.single_floor = False
    node.session['mode'] = 'LIVE_NAVIGATION'
    node.state_at = node.global_at = node.scan_at = node.bt_at = 100.
    node.global_state = node.scan_state = {}
    node.last_seed_id = None
    node.last_snapshot_at = 0.
    node.directory = tmp_path
    node.marker = Mock()
    node.diagnostics_pub = Mock()
    node.publish_goal_editor = Mock()
    node.physical_motion_capable = False
    node.execute_pending = False
    node.reason, node.reason_scope = '', 'input'
    node.expire_requests = lambda now: live_view.LiveView.expire_requests(node, now)
    node.bt_state = owner('A', 'failed')
    return node, acceptance


def owner(task, phase):
    return dict(schema=2, session_id='s', owner='BehaviorTree.CPP', task_id=task,
                phase=phase, root_status='FAILURE' if phase == 'failed' else 'RUNNING',
                active=phase != 'failed', active_node='FollowRoute',
                reason='specific_task_failure' if phase == 'failed' else 'task_tracking',
                received_at_unix=10.)


def render(node):
    live_view.LiveView.tick(node)
    return json.loads(node.diagnostics_pub.publish.call_args.args[0].data)


def test_new_submission_does_not_display_previous_tasks_fresh_failure(monkeypatch, tmp_path):
    node, _ = view(monkeypatch, tmp_path)
    live_view.LiveView.commit_goal(node, None)
    report = render(node)
    assert report['behavior_tree']['label'] == '切换目标'
    assert report['behavior_tree'].get('task_id', '') != 'A'
    assert node.bt_state['task_id'] == 'A'  # presentation does not mutate authority
    snapshot = json.loads((tmp_path / 'view_status.json').read_text())
    assert snapshot['reason'] != '任务失败'


def test_uuid_feedback_uses_current_task_not_previous_heartbeat(monkeypatch, tmp_path):
    node, acceptance = view(monkeypatch, tmp_path)
    live_view.LiveView.commit_goal(node, None)
    acceptance.set_result(Mock(accepted=True))
    feedback = node.navigate_client.send_goal_async.call_args.kwargs['feedback_callback']
    feedback(SimpleNamespace(feedback=SimpleNamespace(task_id='B')))
    report = render(node)
    assert report['behavior_tree']['label'] == '切换目标'
    assert report['behavior_tree']['task_id'] == 'B'
    node.bt_state = owner('B', 'running')
    assert render(node)['behavior_tree']['label'] == '执行任务'
    node.bt_state = owner('B', 'failed')
    failed = render(node)['behavior_tree']
    assert failed['label'] == '任务失败'
    assert failed['task_id'] == 'B'
    assert 'specific_task_failure' in failed['detail']


def test_late_old_action_result_does_not_unmask_old_failed_task(monkeypatch, tmp_path):
    node, first = view(monkeypatch, tmp_path)
    second, old_terminal = Future(), Future()
    node.navigate_client.send_goal_async.side_effect = [first, second]
    live_view.LiveView.commit_goal(node, None)
    first.set_result(Mock(accepted=True, get_result_async=Mock(return_value=old_terminal)))
    live_view.LiveView.commit_goal(node, None)
    second.set_result(Mock(accepted=True))
    feedback = node.navigate_client.send_goal_async.call_args.kwargs['feedback_callback']
    feedback(SimpleNamespace(feedback=SimpleNamespace(task_id='B')))
    old_terminal.set_result(SimpleNamespace(result=SimpleNamespace(success=False)))
    assert node.goal_request_active
    report = render(node)
    assert report['behavior_tree']['label'] == '切换目标'
    assert report['behavior_tree']['task_id'] == 'B'
    assert report['stages']['global']['label'] == '等待当前目标路线'
    assert report['stages']['local']['label'] == '等待新路线'
    assert report['local_target'] is None and report['plan_id'] is None


def test_fast_current_result_before_feedback_still_shows_real_failure(monkeypatch, tmp_path):
    node, acceptance = view(monkeypatch, tmp_path)
    terminal = Future()
    live_view.LiveView.commit_goal(node, None)
    acceptance.set_result(Mock(accepted=True, get_result_async=Mock(return_value=terminal)))
    node.bt_state = owner('B', 'failed')
    terminal.set_result(SimpleNamespace(result=SimpleNamespace(success=False)))
    assert render(node)['behavior_tree']['label'] == '任务失败'


def test_missing_owner_status_is_not_invented_success_or_failure(monkeypatch, tmp_path):
    node, _ = view(monkeypatch, tmp_path)
    live_view.LiveView.commit_goal(node, None)
    node.bt_state = {}
    assert render(node)['behavior_tree']['label'] == '等待当前任务状态'


def test_reopened_view_without_submission_shows_current_authoritative_failure(monkeypatch, tmp_path):
    node, _ = view(monkeypatch, tmp_path)
    assert render(node)['behavior_tree']['label'] == '任务失败'


def test_goal_rejection_does_not_mask_authoritative_task_forever(monkeypatch, tmp_path):
    node, acceptance = view(monkeypatch, tmp_path)
    live_view.LiveView.commit_goal(node, None)
    acceptance.set_result(Mock(accepted=False))
    assert render(node)['behavior_tree']['label'] == '任务失败'


def test_authoritative_switching_goal_is_not_reported_as_idle():
    status = owner('B', 'switching_goal')
    status['root_status'] = 'IDLE'
    assert tree_presentation(status)['label'] == '切换目标'
