"""Exercise view construction and BT intent without ROS, SDK or a GUI."""
import json
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose

from d1max_pct_scan import live_view


def session(**updates):
    value = dict(id='s', version_id='m', frame_id='d1max_loc_map',
        pipeline_contract='single_floor_v3', mode='SINGLE_FLOOR_PLANNING_ONLY',
        purpose='planning_only', execution_purpose='planning_only',
        transport_mode='live', motion_control_enabled=False)
    return {**value, **updates}


@pytest.mark.parametrize('updates,expected', [
    ({}, False),
    ({'motion_control_enabled': True}, False),
    ({'purpose': 'execution', 'execution_purpose': 'execution'}, False),
    ({'purpose': 'execution', 'execution_purpose': 'execution',
      'motion_control_enabled': True}, True),
    ({'purpose': 'execution', 'transport_mode': 'isolated_mock'}, True),
    ({'purpose': 'execution', 'transport_mode': 'unknown',
      'motion_control_enabled': True}, False),
    ({'purpose': 'execution', 'motion_control_enabled': 1}, False),
    ({'purpose': 'execution', 'motion_control_enabled': 'true'}, False),
    ({'purpose': None, 'motion_control_enabled': True}, False),
    ({'pipeline_contract': None, 'mode': 'LIVE_NAVIGATION',
      'motion_control_enabled': True}, True),
    ({'pipeline_contract': None, 'mode': 'LIVE_VISUALIZATION_NO_MOTION'}, False),
])
def test_execution_menu_capability_distinguishes_preview_live_and_simulation(updates, expected):
    assert live_view.execution_menu_capable(session(**updates)) is expected


def construct(monkeypatch, tmp_path, value, *, selection_model=None):
    (tmp_path/'session.json').write_text(json.dumps(value))
    publishers, subscriptions = [], []
    monkeypatch.setattr(live_view.Node, '__init__', lambda *a, **kw: None)
    def publisher(_, message_type, topic, qos):
        publishers.append(topic)
        return Mock()
    def subscription(_, message_type, topic, callback, qos):
        subscriptions.append(topic)
        return Mock()
    monkeypatch.setattr(live_view.Node, 'create_publisher', publisher)
    monkeypatch.setattr(live_view.Node, 'create_subscription', subscription)
    monkeypatch.setattr(live_view.Node, 'create_client', lambda *a, **kw: Mock())
    monkeypatch.setattr(live_view.Node, 'create_timer', lambda *a, **kw: Mock())
    monkeypatch.setattr(live_view, 'ActionClient', Mock())
    monkeypatch.setattr(live_view, 'InteractiveMarkerServer', Mock())
    # UI capability fixture supplies explicit geometry authority. A formal
    # entry with missing map artifacts must not silently accept a free goal.
    selection = selection_model or Mock(mode='ground')
    if selection_model is None:
        selection.floor_choices = [dict(floor_id='floor1', label='一楼')]
        selection.feedback.return_value = dict(valid=True, reason='traversable')
    monkeypatch.setattr(live_view.LiveGoalSelection, 'from_session', lambda _: selection)
    handler = Mock()
    handler.insert.side_effect = range(1, 20)
    menu_factory = Mock(return_value=handler, UNCHECKED=0, CHECKED=1)
    monkeypatch.setattr(live_view, 'MenuHandler', menu_factory)
    node = live_view.LiveView(tmp_path)
    labels = [call.args[0] for call in handler.insert.call_args_list]
    return node, publishers, subscriptions, labels


@pytest.mark.parametrize('value', [session(), session(transport_mode='isolated_mock')])
def test_v3_nonmotion_view_keeps_typed_bt_and_does_not_revive_legacy_freeze(
        monkeypatch, tmp_path, value):
    node, publishers, subscriptions, labels = construct(monkeypatch, tmp_path, value)
    assert node.single_floor and node.typed_view is not None
    assert node.motion_capable is False and node.physical_motion_capable is False
    assert node.freeze is None
    assert '/d1max/localization/navigation/state' in subscriptions
    assert '/d1max/localization/odometry/global' not in subscriptions
    assert '/d1max/live_planning/execution_frozen' not in publishers
    assert '规划到此处' in labels
    assert '导航到此处' not in labels and '执行已预览路线' not in labels
    # Planning-only still submits the real Navigate Action to the one BT owner.
    node.goal_pose = Pose()
    node.goal_pose.orientation.w = 1.
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: Time(sec=10), nanoseconds=10_000_000_000))
    node.navigate_client.server_is_ready.return_value = True
    node.commit_goal(None)
    request = node.navigate_client.send_goal_async.call_args.args[0]
    assert request.schema_version == 2 and request.request_execution is False
    node.bt_state = dict(schema=2, session_id='s', task_id='t', route_id='r', route_hash='h')
    node.bt_at = time.monotonic()
    node.execute_client.service_is_ready.return_value = True
    node.confirm_execution(None)
    node.execute_client.call_async.assert_not_called()
    assert node.reason == '当前入口仅提供规划预览'


def test_v3_real_execution_view_offers_intent_but_does_not_self_authorize(monkeypatch, tmp_path):
    value = session(purpose='execution', execution_purpose='execution',
        mode='SINGLE_FLOOR_EXECUTION', motion_control_enabled=True)
    node, publishers, subscriptions, labels = construct(monkeypatch, tmp_path, value)
    assert node.motion_capable and node.physical_motion_capable
    assert node.typed_view is not None and node.freeze is None
    assert '导航到此处' in labels and '仅规划预览' in labels
    assert '执行已预览路线' in labels
    assert '/d1max/live_planning/execution_frozen' not in publishers
    node.execute_client.call_async.assert_not_called()
    node.navigate_client.send_goal_async.assert_not_called()


def test_v3_simulated_execution_keeps_bt_confirmation_without_physical_motion(monkeypatch, tmp_path):
    value = session(purpose='execution', execution_purpose='execution',
        mode='SINGLE_FLOOR_EXECUTION', transport_mode='isolated_mock')
    node, publishers, subscriptions, labels = construct(monkeypatch, tmp_path, value)
    assert node.motion_capable and not node.physical_motion_capable
    assert node.typed_view is not None and node.freeze is None
    assert '执行已预览路线' in labels and '导航到此处' in labels
    assert '/d1max/live_planning/execution_frozen' not in publishers
    node.execute_client.call_async.assert_not_called()
    node.navigate_client.send_goal_async.assert_not_called()
    node.bt_state = dict(schema=2, session_id='s', task_id='t', route_id='r', route_hash='h')
    node.bt_at = time.monotonic()
    node.execute_client.service_is_ready.return_value = True
    node.confirm_execution(None)
    request = node.execute_client.call_async.call_args.args[0]
    assert request.schema_version == 2 and request.route_hash == 'h'
    # A service request is an intent, not a local or physical authorization.
    assert node.execute_pending and not node.physical_motion_capable


def test_legacy_display_only_keeps_its_existing_preview_freeze(monkeypatch, tmp_path):
    value = session(pipeline_contract='legacy', mode='LIVE_VISUALIZATION_NO_MOTION')
    node, publishers, subscriptions, labels = construct(monkeypatch, tmp_path, value)
    assert not node.single_floor and node.typed_view is None
    assert node.freeze is not None and not node.motion_capable
    assert '/d1max/live_planning/execution_frozen' in publishers
    assert '/d1max/localization/odometry/global' in subscriptions
    assert '规划到此处' in labels and '执行已预览路线' not in labels


def test_formal_conditioned_map_exposes_both_goal_and_initial_floors_without_execution_change(
        monkeypatch, tmp_path):
    from test_live_goal_selection import conditioned_model
    model = conditioned_model()
    node, publishers, subscriptions, labels = construct(monkeypatch, tmp_path,
        session(current_floor='floor1'), selection_model=model)
    assert node.single_floor and node.typed_view is not None
    assert node.initial_ground_bridge is model.bridge
    assert labels.count('一楼') == 2 and labels.count('二楼') == 2
    assert node.initial_floor_choices == ('floor1', 'floor2')
    node.select_initial_floor('floor2')
    assert node.initial_floor == 'floor2'
    node.select_initial_floor('floor3')
    assert node.initial_floor == 'floor2'
    assert not node.motion_capable and not node.physical_motion_capable
    assert '导航到此处' not in labels and '执行已预览路线' not in labels
    assert '/d1max/live_planning/execution_frozen' not in publishers
    assert '/d1max/localization/navigation/state' in subscriptions
    # The same Navigate action still receives source-ground, never the
    # conditioned PCT height, and optional yaw is not altered by the bridge.
    report = model.edit([.13, .07, 3.0726])
    node.goal_pose = Pose()
    node.goal_pose.position.x, node.goal_pose.position.y, node.goal_pose.position.z = report['xyz']
    node.goal_pose.orientation.z, node.goal_pose.orientation.w = .6, .8
    node.goal_has_yaw = True
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=10_000_000_000, to_msg=lambda: Time(sec=10)))
    node.commit_goal(None)
    request = node.navigate_client.send_goal_async.call_args.args[0]
    assert request.goal.header.frame_id == 'd1max_loc_map'
    assert request.goal.pose.position.z == pytest.approx(3.0726)
    assert request.goal.pose.orientation == node.goal_pose.orientation
    assert request.has_goal_yaw and request.request_execution is False


def test_formal_second_floor_initial_pose_queries_source_field_adds_body_height_once(
        monkeypatch, tmp_path):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from d1max_pct_scan.initial_pose_client import TRANSACTION
    from test_live_goal_selection import conditioned_model
    model = conditioned_model()
    value = session(current_floor='floor1', body_height=.5,
        initial_pose_transaction=TRANSACTION, map_pcd='/map.pcd',
        input_hashes={'/map.pcd': 'a'*64})
    node, _, _, _ = construct(monkeypatch, tmp_path, value, selection_model=model)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=10_000_000_000, to_msg=lambda: Time(sec=10)))
    node.get_logger = lambda: Mock()
    node.select_initial_floor('floor2')
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'd1max_loc_map'
    pose.header.stamp = Time(sec=10)
    pose.pose.pose.position.x, pose.pose.pose.position.y = .13, .07
    pose.pose.pose.position.z = 0.  # RViz 2D tool does not supply floor height.
    pose.pose.pose.orientation.w = 1.
    node.initial(pose)
    request = node.initial_pose_transaction.pending[0]
    assert request.body_pose.header == pose.header
    assert request.body_pose.pose.pose.position.x == .13
    assert request.body_pose.pose.pose.position.y == .07
    assert request.body_pose.pose.pose.position.z == pytest.approx(3.5726)
    assert pose.pose.pose.position.z == 0.
    assert request.source_map_sha256 == 'a'*64
    node.cancel_pub.publish.assert_not_called()
    node.navigate_client.send_goal_async.assert_not_called()
    assert not (tmp_path/'initial_pose.json').exists()


def test_formal_single_floor_map_only_offers_its_declared_initial_floor(monkeypatch, tmp_path):
    node, _, _, labels = construct(monkeypatch, tmp_path, session())
    assert node.initial_floor_choices == ('floor1',)
    node.select_initial_floor('floor2')
    assert node.initial_floor == 'floor1'
    assert '二楼' not in labels


def test_global_route_view_subscribes_only_to_compute_snapshot(monkeypatch, tmp_path):
    node, _, subscriptions, labels = construct(monkeypatch, tmp_path,
        session(planning_scope='global_route'))
    assert '/d1max/live_planning/bt/route_snapshot' in subscriptions
    assert '/d1max/live_planning/committed_route' not in subscriptions
    assert node.freeze is None and not node.motion_capable
    assert '执行已预览路线' not in labels
