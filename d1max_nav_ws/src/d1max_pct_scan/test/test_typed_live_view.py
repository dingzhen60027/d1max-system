"""Actual RViz callback bodies without a ROS graph or GUI."""
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from d1max_pct_scan.live_view import LiveView
from d1max_pct_scan.typed_view_state import TypedViewState,seed_command
from test_continuous_reference_transport import state,BASE_NS
from test_live_goal_action import harness


def test_typed_state_needs_source_posterior_imu_freshness_not_json():
    s=TypedViewState('session','map')
    assert s.observe(state(),now_ns=BASE_NS,monotonic=100.)
    assert s.lease(now_ns=BASE_NS,monotonic=100.)>0
    assert not s.observe(state(),now_ns=BASE_NS+50_000_000,monotonic=100.05)
    assert s.received==100.
    assert s.lease(now_ns=BASE_NS+101_000_000,monotonic=100.101)==0
    old=state();old.map_version_id='other'
    with pytest.raises(ValueError):s.observe(old,now_ns=BASE_NS,monotonic=100.)


def test_actual_atomic_callback_preserves_global_axes_pose_and_source(monkeypatch):
    from d1max_pct_scan import live_view
    monkeypatch.setattr(live_view.time,'monotonic',lambda:100.)
    node=SimpleNamespace(typed_view=TypedViewState('session','map'),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=BASE_NS)))
    message=state(x=.1,correction=.05)
    LiveView.on_atomic_state(node,message)
    assert node.body==message.global_odometry and node.body_received==100.
    assert node.body_sample_context==('session',1,'seed')


def test_v3_goal_never_depends_on_legacy_json_and_sends_once(monkeypatch):
    node,future=harness(monkeypatch)
    node.single_floor=True;node.session={'id':'session','version_id':'map'}
    node.typed_view=TypedViewState('session','map')
    node.last_state={}  # not a made-up legacy confirmation dictionary
    from d1max_pct_scan import live_view
    monkeypatch.setattr(live_view.time,'monotonic',lambda:100.)
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=BASE_NS,to_msg=lambda:Time(sec=10)))
    node.typed_view.observe(state(),now_ns=BASE_NS,monotonic=100.)
    LiveView.commit_goal(node,None);LiveView.commit_goal(node,None)
    node.navigate_client.send_goal_async.assert_called_once()
    assert node.navigate_client.send_goal_async.call_args.args[0].has_goal_yaw


def test_seed_not_circularly_blocked_by_unlocalized_state():
    p=PoseWithCovarianceStamped();p.header.frame_id='d1max_loc_map';p.header.stamp=Time(sec=10)
    p.pose.pose.orientation.w=1.;p.pose.pose.position.x=2.
    command=seed_command(p,now=10.,session_id='session',body_z=.06,command_id='seed')
    assert command['reference']=='body' and command['z']==.06 and command['x']==2.
    p.pose.pose.orientation.w=0.
    with pytest.raises(ValueError):seed_command(p,now=10.,session_id='session',body_z=.06,command_id='seed')


def test_single_floor_cannot_select_second_floor():
    node=SimpleNamespace(single_floor=True,initial_floor='floor1')
    LiveView.select_initial_floor(node,'floor2')
    assert node.initial_floor=='floor1'


def test_view_source_time_uses_ros_bag_clock_not_wall_clock(monkeypatch):
    from d1max_pct_scan import live_view
    monkeypatch.setattr(live_view.time, 'time', lambda: 999999.)
    node=SimpleNamespace(get_clock=lambda:SimpleNamespace(
        now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)))
    assert LiveView.source_now_s(node)==10.
