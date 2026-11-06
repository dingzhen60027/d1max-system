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


def construct(monkeypatch, tmp_path, value):
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
