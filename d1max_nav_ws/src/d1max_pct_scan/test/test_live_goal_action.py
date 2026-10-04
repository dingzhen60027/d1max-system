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
    assert goal.goal.pose.position.z == 3.8
    node.goal_pose.position.z=7.
    assert goal.goal.pose.position.z == 3.8
    handle=Mock(accepted=True)
    future.set_result(handle)
    assert not node.goal_request_pending
    handle.cancel_goal_async.assert_not_called()


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
