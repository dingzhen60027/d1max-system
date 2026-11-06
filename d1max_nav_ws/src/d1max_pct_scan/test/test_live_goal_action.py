from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time
from d1max_pct_scan import live_view


def harness(monkeypatch):
    monkeypatch.setattr(live_view,'localization_preview_state',lambda *a,**kw:(True,''))
    future=Future()
    client=Mock()
    client.server_is_ready.return_value=True
    client.send_goal_async.return_value=future
    node=SimpleNamespace(goal_pose=Pose(),last_state={},session={'id':'s'},
        frame='d1max_loc_map',goal_has_yaw=True,goal_request_sequence=0,
        goal_request_pending=False,navigate_client=client,cancel_pub=Mock(),motion_capable=False)
    node.goal_pose.orientation.w=1.
    node.goal_pose.position.z=3.8
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=10)))
    return node,future


def test_dragged_xyz_and_optional_yaw_use_single_task_action_not_path_publisher(monkeypatch):
    node,future=harness(monkeypatch)
    live_view.LiveView.commit_goal(node,None)
    goal=node.navigate_client.send_goal_async.call_args.args[0]
    assert goal.schema_version == 2 and goal.goal_kind == '3d' and goal.has_goal_yaw
    assert goal.request_execution is False
    assert goal.goal.pose.position.z == 3.8
    node.goal_pose.position.z=7.
    assert goal.goal.pose.position.z == 3.8
    handle=Mock(accepted=True)
    future.set_result(handle)
    assert not node.goal_request_pending
    handle.cancel_goal_async.assert_not_called()


def test_each_explicit_navigation_submission_is_new_intent_without_second_ui_rpc(monkeypatch):
    node,future=harness(monkeypatch)
    second = Future()
    node.navigate_client.send_goal_async.side_effect = [future, second]
    node.motion_capable=True
    node.execute_client=Mock()
    live_view.LiveView.commit_goal(node,None,request_execution=True)
    live_view.LiveView.commit_goal(node,None,request_execution=True)
    assert node.navigate_client.send_goal_async.call_count == 2
    assert node.navigate_client.send_goal_async.call_args.args[0].request_execution is True
    old = Mock(accepted=True)
    future.set_result(old)
    old.cancel_goal_async.assert_called_once()
    second.set_result(Mock(accepted=True))
    node.execute_client.call_async.assert_not_called()
    assert node.reason=='导航目标已接收'


def test_motion_capable_preview_still_requires_separate_explicit_intent(monkeypatch):
    node,_=harness(monkeypatch);node.motion_capable=True
    live_view.LiveView.commit_goal(node,None)
    assert node.navigate_client.send_goal_async.call_args.args[0].request_execution is False


def test_display_only_entry_cannot_send_execution_intent(monkeypatch):
    node,_=harness(monkeypatch)
    live_view.LiveView.commit_goal(node,None,request_execution=True)
    node.navigate_client.send_goal_async.assert_not_called()
    assert not node.goal_request_pending


def test_late_goal_acceptance_after_user_cancel_is_retired_not_ignored(monkeypatch):
    node,future=harness(monkeypatch)
    live_view.LiveView.commit_goal(node,None)
    live_view.LiveView.cancel(node,None)
    handle=Mock(accepted=True)
    future.set_result(handle)
    handle.cancel_goal_async.assert_called_once()
    assert not node.goal_request_pending


def test_display_goal_rejection_does_not_claim_execution(monkeypatch):
    node,future=harness(monkeypatch)
    live_view.LiveView.commit_goal(node,None)
    future.set_result(Mock(accepted=False))
    assert '拒绝' in node.reason
    assert not node.goal_request_pending


def test_send_goal_exception_does_not_wedge_pending(monkeypatch):
    node,_=harness(monkeypatch)
    node.navigate_client.send_goal_async.side_effect=RuntimeError('transport failed')
    live_view.LiveView.commit_goal(node,None)
    assert not node.goal_request_pending and '失败' in node.reason


def test_confirm_result_after_cancel_never_relabels_retired_task_as_authorized(monkeypatch):
    import time
    node,_=harness(monkeypatch)
    node.motion_capable=True
    node.bt_state=dict(schema=2,session_id='s',task_id='t',route_id='r',route_hash='hash')
    node.bt_at=time.monotonic()
    node.execute_pending=False
    node.execute_binding=None
    node.execute_client=Mock()
    node.execute_client.service_is_ready.return_value=True
    response=Future()
    node.execute_client.call_async.return_value=response
    live_view.LiveView.confirm_execution(node,None)
    request=node.execute_client.call_async.call_args.args[0]
    assert request.task_id == 't' and request.route_hash == 'hash'
    live_view.LiveView.cancel(node,None)
    expected=node.reason
    response.set_result(SimpleNamespace(schema_version=2,accepted=True,execution_authorized=True))
    assert node.reason == expected and not node.execute_pending


def test_goal_timeout_does_not_accept_late_handle_or_relabel_new_state(monkeypatch):
    node,future=harness(monkeypatch)
    node.execute_pending=False
    live_view.LiveView.commit_goal(node,None)
    live_view.LiveView.expire_requests(node,node.goal_request_deadline)
    expected=node.reason
    handle=Mock(accepted=True)
    future.set_result(handle)
    handle.cancel_goal_async.assert_called_once()
    assert not node.goal_request_pending and node.reason == expected


def test_confirmation_timeout_keeps_idempotency_key_and_discards_late_ack(monkeypatch):
    import time
    node,_=harness(monkeypatch)
    node.motion_capable=True
    node.bt_state=dict(schema=2,session_id='s',task_id='t',route_id='r',route_hash='hash')
    node.bt_at=time.monotonic()
    node.execute_pending=False
    node.execute_binding=None
    node.execute_client=Mock()
    response,retry=Future(),Future()
    node.execute_client.service_is_ready.return_value=True
    node.execute_client.call_async.side_effect=[response,retry]
    live_view.LiveView.confirm_execution(node,None)
    original=node.execute_request_id
    live_view.LiveView.expire_requests(node,node.execute_deadline)
    assert not node.execute_pending and '未知' in node.reason
    live_view.LiveView.confirm_execution(node,None)
    assert node.execute_request_id == original and node.execute_pending
    response.set_result(SimpleNamespace(schema_version=2,accepted=True,execution_authorized=True))
    assert node.execute_pending  # old response must not clear the newer wait
    retry.set_result(SimpleNamespace(schema_version=2,accepted=False,execution_authorized=False,reason='preview_only'))
    assert not node.execute_pending and '未授权' in node.reason


def test_new_target_replaces_pending_goal_and_late_acceptance_only_cancels_old_handle(monkeypatch):
    node,first=harness(monkeypatch)
    second,third=Future(),Future()
    node.navigate_client.send_goal_async.side_effect=[first,second,third]
    live_view.LiveView.commit_goal(node,None)
    node.goal_pose.position.x=2.
    live_view.LiveView.commit_goal(node,None)
    node.goal_pose.position.x=3.
    live_view.LiveView.commit_goal(node,None)
    assert node.navigate_client.send_goal_async.call_count==3
    expected=node.reason
    for pending in (second,first):
        old=Mock(accepted=True)
        pending.set_result(old)
        old.cancel_goal_async.assert_called_once()
        assert node.reason==expected and node.goal_request_pending
    current=Mock(accepted=True)
    third.set_result(current)
    current.cancel_goal_async.assert_not_called()
    assert not node.goal_request_pending
    node.cancel_pub.publish.assert_not_called()  # no unscoped cancel can kill C


def test_preview_then_navigate_same_xyz_is_new_intent_not_double_click(monkeypatch):
    node,preview=harness(monkeypatch)
    node.motion_capable=True
    execute=Future()
    node.navigate_client.send_goal_async.side_effect=[preview,execute]
    live_view.LiveView.commit_goal(node,None)
    live_view.LiveView.commit_goal(node,None,request_execution=True)
    assert node.navigate_client.send_goal_async.call_count==2
    assert node.navigate_client.send_goal_async.call_args.args[0].request_execution is True


def test_replacement_retires_old_confirmation_and_cannot_confirm_old_route(monkeypatch):
    import time
    node,goal=harness(monkeypatch)
    node.motion_capable=True
    node.bt_state=dict(schema=2,session_id='s',task_id='A',route_id='rA',route_hash='hashA')
    node.bt_at=time.monotonic()
    node.execute_pending=False
    node.execute_binding=None
    node.execute_client=Mock()
    node.execute_client.service_is_ready.return_value=True
    old_ack,new_ack=Future(),Future()
    node.execute_client.call_async.side_effect=[old_ack,new_ack]
    live_view.LiveView.confirm_execution(node,None)
    live_view.LiveView.commit_goal(node,None)
    assert not node.execute_pending
    expected=node.reason
    old_ack.set_result(SimpleNamespace(schema_version=2,accepted=True,execution_authorized=True))
    assert node.reason==expected
    goal.set_result(Mock(accepted=True))
    live_view.LiveView.confirm_execution(node,None)
    assert node.execute_client.call_async.call_count==1
    feedback=node.navigate_client.send_goal_async.call_args.kwargs['feedback_callback']
    feedback(SimpleNamespace(feedback=SimpleNamespace(task_id='B')))
    live_view.LiveView.confirm_execution(node,None)
    assert node.execute_client.call_async.call_count==1  # A's late route is not B
    node.bt_state.update(task_id='B',route_id='rB',route_hash='hashB')
    live_view.LiveView.confirm_execution(node,None)
    assert node.execute_client.call_async.call_count==2
    assert node.execute_client.call_async.call_args.args[0].task_id=='B'


def test_late_feedback_cannot_bind_replacement_to_superseded_task(monkeypatch):
    node,first=harness(monkeypatch)
    second=Future()
    node.navigate_client.send_goal_async.side_effect=[first,second]
    live_view.LiveView.commit_goal(node,None)
    old_feedback=node.navigate_client.send_goal_async.call_args.kwargs['feedback_callback']
    node.goal_pose.position.x=2.
    live_view.LiveView.commit_goal(node,None)
    new_feedback=node.navigate_client.send_goal_async.call_args.kwargs['feedback_callback']
    new_feedback(SimpleNamespace(feedback=SimpleNamespace(task_id='B')))
    old_feedback(SimpleNamespace(feedback=SimpleNamespace(task_id='A')))
    assert node.goal_request_task_id=='B'


def test_same_target_new_submission_replaces_active_task_and_late_result_cannot_clear_it(monkeypatch):
    node,first=harness(monkeypatch)
    terminal=Future()
    handle=Mock(accepted=True)
    handle.get_result_async.return_value=terminal
    second, third = Future(), Future()
    node.navigate_client.send_goal_async.side_effect = [first, second, third]
    live_view.LiveView.commit_goal(node,None)
    first.set_result(handle)
    live_view.LiveView.commit_goal(node,None)
    assert node.navigate_client.send_goal_async.call_count == 2
    terminal.set_result(SimpleNamespace(result=SimpleNamespace(success=True)))
    assert node.goal_request_pending and node.goal_request_active
    live_view.LiveView.commit_goal(node,None)
    assert node.navigate_client.send_goal_async.call_count==3


def test_old_terminal_result_cannot_clear_new_goal_and_same_xyz_can_submit_again(monkeypatch):
    node,first=harness(monkeypatch)
    second,terminal=Future(),Future()
    third = Future()
    node.navigate_client.send_goal_async.side_effect=[first,second,third]
    old=Mock(accepted=True)
    old.get_result_async.return_value=terminal
    live_view.LiveView.commit_goal(node,None)
    first.set_result(old)
    node.goal_pose.position.x=2.
    live_view.LiveView.commit_goal(node,None)
    terminal.set_result(SimpleNamespace(result=SimpleNamespace(success=False)))
    assert node.goal_request_active and node.goal_request_pending
    live_view.LiveView.commit_goal(node,None)
    assert node.navigate_client.send_goal_async.call_count==3


def test_same_target_submitted_during_localization_gap_still_reaches_bt_latest_wins(monkeypatch):
    node, first = harness(monkeypatch)
    second = Future()
    node.navigate_client.send_goal_async.side_effect = [first, second]
    live_view.LiveView.commit_goal(node, None)
    monkeypatch.setattr(live_view, 'localization_preview_state', lambda *a, **kw: (False, 'epoch_reset'))
    live_view.LiveView.commit_goal(node, None)
    assert node.goal_request_sequence == 2 and node.navigate_client.send_goal_async.call_count == 2
    old_handle = Mock(accepted=True)
    first.set_result(old_handle)
    old_handle.cancel_goal_async.assert_called_once()
    assert node.goal_request_pending
    node.cancel_pub.publish.assert_not_called()
