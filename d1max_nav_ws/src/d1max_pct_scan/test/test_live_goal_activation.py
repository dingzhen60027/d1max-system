"""Run the actual RViz activation callback without ROS, GUI or a robot."""
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from d1max_pct_scan.live_view import LiveView


def node(monkeypatch, *, ready=False):
    from d1max_pct_scan import live_view
    monkeypatch.setattr(live_view.time, 'monotonic', lambda: 100.)
    monkeypatch.setattr(LiveView, 'localization_ready', lambda _: ready)
    return SimpleNamespace(
        session={'id': 'v3-session', 'pipeline_contract': 'single_floor_v3', 'body_height': .5},
        frame='d1max_loc_map', goal_pose=None, body=None, body_received=0.,
        map_layer_metadata={}, goal_surfaces=None, reason='', reason_scope='',
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=10))),
        place_goal=Mock())


def metadata(**updates):
    value=dict(session_id='v3-session', frame_id='d1max_loc_map',
               ground_display_only=True, motion_enabled=False, initial_goal_xyz=[4.9, 22.2, -.57])
    return {**value, **updates}


def body(frame='d1max_loc_map'):
    value=Odometry()
    value.header.frame_id=frame
    value.header.stamp=Time(sec=9, nanosec=980_000_000)
    value.pose.pose.position.x=2.
    value.pose.pose.position.y=3.
    value.pose.pose.position.z=.5
    value.pose.pose.orientation.w=1.
    return value


def xyz(value):
    return value.point.x, value.point.y, value.point.z


def test_same_v3_input_old_callback_failed_eager_default_new_uses_map_ground(monkeypatch):
    value=node(monkeypatch)
    value.map_layer_metadata=metadata()
    # The production traceback's exact expression fails even with a real seed.
    with pytest.raises(KeyError, match='initial_body_z'):
        value.map_layer_metadata.get('initial_goal_xyz',
            (0., 0., value.session['initial_body_z']-value.session.get('body_height', .55)))
    LiveView.activate_goal(value, None)
    message=value.place_goal.call_args.args[0]
    assert xyz(message)==(4.9, 22.2, -.57)  # Ground seed is not height-corrected again.
    assert message.header.frame_id=='d1max_loc_map' and message.header.stamp.sec==10
    assert 'initial_body_z' not in value.session


def test_existing_ground_draft_needs_neither_legacy_height_nor_localization(monkeypatch):
    value=node(monkeypatch)
    value.goal_pose=Pose()
    value.goal_pose.position.x=5.
    value.goal_pose.position.y=6.
    value.goal_pose.position.z=3.8
    value.map_layer_metadata=metadata(initial_goal_xyz=[0., 0., -7.])
    LiveView.activate_goal(value, None)
    assert xyz(value.place_goal.call_args.args[0])==(5., 6., 3.8)


def test_verified_same_map_body_subtracts_height_once_and_does_not_restamp_body(monkeypatch):
    value=node(monkeypatch, ready=True)
    value.body=body();value.body_received=99.99
    original=deepcopy(value.body)
    LiveView.activate_goal(value, None)
    assert xyz(value.place_goal.call_args.args[0])==(3.5, 3., 0.)
    assert value.body==original and value.body_received==99.99


@pytest.mark.parametrize('ready,frame,received', [
    (False, 'd1max_loc_map', 99.99),
    (True, 'd1max_loc_odom', 99.99),
    (True, 'd1max_loc_map', 99.4),
    (True, 'd1max_loc_map', 100.01),
])
def test_unusable_wrong_frame_or_stale_body_cannot_seed_goal(monkeypatch, ready, frame, received):
    value=node(monkeypatch, ready=ready)
    value.body=body(frame);value.body_received=received
    LiveView.activate_goal(value, None)
    value.place_goal.assert_not_called()
    assert value.reason=='等待定位或当前地图的可通行地面目标'


@pytest.mark.parametrize('updates', [
    {'session_id': 'another'}, {'frame_id': 'd1max_loc_odom'},
    {'ground_display_only': False}, {'motion_enabled': True},
])
def test_foreign_or_non_ground_metadata_cannot_seed_unlocalized_draft(monkeypatch, updates):
    value=node(monkeypatch);value.map_layer_metadata=metadata(**updates)
    LiveView.activate_goal(value, None)
    value.place_goal.assert_not_called()


@pytest.mark.parametrize('point', [[float('nan'), 0., 0.], [1., 2.], ['bad', 0., 0.]])
def test_bad_seed_reports_input_problem_without_killing_display(monkeypatch, point):
    value=node(monkeypatch);value.map_layer_metadata=metadata(initial_goal_xyz=point)
    LiveView.activate_goal(value, None)
    value.place_goal.assert_not_called()
    assert value.reason=='3D 目标草稿坐标无效'
