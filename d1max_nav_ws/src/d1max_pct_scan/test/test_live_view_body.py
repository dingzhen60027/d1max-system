"""Message-only tests of LiveView methods; no ROS context, node or graph."""
from copy import deepcopy
import json
from types import SimpleNamespace
from unittest.mock import patch
import pytest
pytest.importorskip('builtin_interfaces.msg', reason='ROS message-only tests require sourced Humble environment')
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from d1max_pct_scan.live_view import (LiveView, BODY_AXIS_LENGTH_M, BODY_AXIS_WIDTH_M,
                                    body_axes_marker)


def fixture():
    state = {'session_id': 's', 'wall_time': 99.9, 'map_localized': True, 'localized': True,
             'local_epoch': 2, 'active_seed_ns': 'seed', 'confirmed_seed_ns': 'seed',
             'verified_confirmations': 3, 'navigation': {'valid': True, 'epoch': 2,
                 'seed_id': 'seed', 'received_at_unix': 99.9}}
    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = 'd1max_loc_map', 'd1max_loc_base_link'
    odom.header.stamp = Time(sec=99, nanosec=800000000)
    odom.pose.pose.position.x = 1.25
    odom.pose.pose.position.y = -2.5
    odom.pose.pose.position.z = .6
    odom.pose.pose.orientation.w = 1.
    published = []
    node = SimpleNamespace(session={'id': 's'}, frame='d1max_loc_map',
        goal_request_pending=False, execute_pending=False,
        last_state=state, state_at=19.95, body=odom, body_received=19.8,
        body_sample_context=('s', 2, 'seed'), body_context=('s', 2, 'seed', 'seed'),
        body_barrier=99., retired_body_caption=False,
        body_marker=SimpleNamespace(publish=published.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: Time(sec=100, nanosec=0))))
    node.expire_requests=lambda now: LiveView.expire_requests(node,now)
    return node, published


def render(node, wall=100., mono=20.):
    with patch('d1max_pct_scan.live_view.time.time', return_value=wall), \
         patch('d1max_pct_scan.live_view.time.monotonic', return_value=mono):
        LiveView.publish_body(node)


def axes(output):
    return next(marker for marker in reversed(output) if marker.ns == 'measured_body')


def ttl(marker):
    return marker.lifetime.sec + marker.lifetime.nanosec * 1e-9


def status(node, state, wall=100., mono=20.):
    with patch('d1max_pct_scan.live_view.time.time', return_value=wall), \
         patch('d1max_pct_scan.live_view.time.monotonic', return_value=mono):
        LiveView.status(node, String(data=json.dumps(state)))


def test_status_wait_keeps_fresh_marker_header_pose_and_shrinking_source_lease():
    node, output = fixture()
    original = deepcopy(node.body)
    waiting = deepcopy(node.last_state)
    waiting.update(wall_time=100., localized=False, state='output_waiting')
    waiting['navigation']['valid'] = False
    status(node, waiting)
    assert node.body == original  # Status availability is not an identity reset.
    render(node)
    assert axes(output).action == Marker.ADD
    assert axes(output).header == original.header and axes(output).pose == original.pose.pose
    assert output[-1].ns == 'measured_body' and axes(output).type == Marker.LINE_LIST
    assert not any(marker.action == Marker.ADD and marker.text for marker in output)
    first_ttl = ttl(axes(output))
    render(node, wall=100.1, mono=20.1)
    assert axes(output).header == original.header and axes(output).pose == original.pose.pose
    assert ttl(axes(output)) < first_ttl
    render(node, wall=100.31, mono=20.31)
    assert axes(output).action == Marker.DELETE


def test_new_actual_odom_replaces_pose_during_same_identity_output_wait():
    node, output = fixture()
    node.last_state.update(localized=False, map_localized=False,
                           continuous_pose_valid=False, state='output_waiting')
    node.last_state['navigation']['valid'] = False
    message = deepcopy(node.body)
    message.header.stamp = Time(sec=100, nanosec=50000000)
    message.pose.pose.position.x = 1.3
    with patch('d1max_pct_scan.live_view.time.time', return_value=100.1), \
         patch('d1max_pct_scan.live_view.time.monotonic', return_value=20.1):
        LiveView.on_body(node, message)
    render(node, wall=100.1, mono=20.1)
    assert axes(output).action == Marker.ADD
    assert axes(output).header == message.header and axes(output).pose == message.pose.pose


def test_new_epoch_reseed_or_fault_immediately_discards_saved_sample():
    for change in ({'local_epoch': 3}, {'active_seed_ns': 'new'}, {'local_fault': 'imu_fault'},
                   {'navigation': {'fault': 'filter_jump'}}):
        node, output = fixture()
        state = deepcopy(node.last_state)
        state.update(wall_time=100., **change)
        status(node, state)
        assert node.body is None and node.body_sample_context is None
        render(node)
        assert output[-1].action == Marker.DELETE


def test_repeated_or_old_body_packet_cannot_renew_original_marker_lease():
    node, output = fixture()
    original_received = node.body_received
    with patch('d1max_pct_scan.live_view.time.time', return_value=100.1), \
         patch('d1max_pct_scan.live_view.time.monotonic', return_value=20.1):
        LiveView.on_body(node, deepcopy(node.body))
    assert node.body_received == original_received
    render(node, wall=100.31, mono=20.31)
    assert axes(output).action == Marker.DELETE


def test_axes_are_fixed_local_rgb_line_segments_at_unmodified_pose():
    node, _ = fixture()
    # Nontrivial body orientation must be applied by marker.pose, not flattened
    # into map axes or moved above the map just to make the marker visible.
    node.body.pose.pose.orientation.y = .6
    node.body.pose.pose.orientation.w = .8
    marker = body_axes_marker(node.body, .24)
    assert marker.type == Marker.LINE_LIST
    assert marker.header == node.body.header and marker.pose == node.body.pose.pose
    assert marker.scale.x == pytest.approx(BODY_AXIS_WIDTH_M)
    assert [(p.x, p.y, p.z) for p in marker.points] == [
        (0., 0., 0.), (BODY_AXIS_LENGTH_M, 0., 0.),
        (0., 0., 0.), (0., BODY_AXIS_LENGTH_M, 0.),
        (0., 0., 0.), (0., 0., BODY_AXIS_LENGTH_M)]
    assert [(c.r, c.g, c.b, c.a) for c in marker.colors] == [
        (1., 0., 0., 1.), (1., 0., 0., 1.),
        (0., 1., 0., 1.), (0., 1., 0., 1.),
        (0., 0., 1., 1.), (0., 0., 1., 1.)]
    assert marker.color.a == 1. and ttl(marker) == pytest.approx(.24)
    assert marker.frame_locked is False


@pytest.mark.parametrize('quality', ['tracking', 'coasting', 'output_waiting'])
def test_fresh_same_identity_pose_has_fixed_colors_despite_soft_quality(quality):
    node, output = fixture()
    node.last_state['navigation'].update(quality=quality,
        prediction={'degraded': True, 'reason': 'predicting_degraded_imu_gap'})
    node.last_state['continuous_pose_valid'] = quality != 'output_waiting'
    render(node)
    assert axes(output).action == Marker.ADD
    assert [(c.r, c.g, c.b, c.a) for c in axes(output).colors] == [
        (1., 0., 0., 1.), (1., 0., 0., 1.),
        (0., 1., 0., 1.), (0., 1., 0., 1.),
        (0., 0., 1., 1.), (0., 0., 1., 1.)]


def test_caption_retired_once_before_axes_and_never_recreated():
    node, output = fixture()
    render(node)
    assert len(output) == 2
    assert output[0].ns == 'last_known_body' and output[0].action == Marker.DELETE
    assert output[1].ns == 'measured_body' and output[1].action == Marker.ADD
    render(node)
    assert len(output) == 3 and output[-1].ns == 'measured_body'


def test_tick_clears_world_caption_but_keeps_panel_diagnostics():
    node, _ = fixture()
    world, reports, goals = [], [], []
    node.freeze = None
    node.global_state, node.scan_state = {}, {}
    node.global_at = node.scan_at = 0.
    node.last_seed_id = None
    node.reason = '等待定位'
    node.reason_scope = 'input'
    node.motion_capable = False
    node.marker = SimpleNamespace(publish=world.append)
    node.diagnostics_pub = SimpleNamespace(publish=reports.append)
    node.publish_goal_editor = lambda: goals.append(True)
    node.last_snapshot_at = 20.
    with patch('d1max_pct_scan.live_view.time.time', return_value=100.), \
         patch('d1max_pct_scan.live_view.time.monotonic', return_value=20.):
        LiveView.tick(node)
    assert len(world) == 1 and world[0].action == Marker.DELETE
    assert world[0].ns == 'live_state' and not world[0].text
    assert len(reports) == 1 and json.loads(reports[0].data)['session_id'] == 's'
    assert goals == [True]


@pytest.mark.parametrize('source_stamp,source_session,reason,label', [
    (99.9, 's', 'current_floor_requires_unique_measured_ground_support', '目标保留 · 等待可用起点'),
    (99.9, 's', 'total_goal_computation_deadline_expired', '规划未完成'),
    (98., 's', 'current_floor_requires_unique_measured_ground_support', None),
    (99.9, 'other', 'current_floor_requires_unique_measured_ground_support', None),
])
def test_live_view_callback_tick_exposes_current_hold_not_stale_or_other_session(
        tmp_path, source_stamp, source_session, reason, label):
    node, _ = fixture()
    reports = []
    node.freeze = None
    node.global_state, node.scan_state = {}, {}
    node.global_at = node.scan_at = 0.
    node.last_seed_id = None
    node.reason, node.reason_scope = '目标已提交，等待规划', 'planning'
    node.goal_submitted_at = 99.
    node.motion_capable = False
    node.marker = SimpleNamespace(publish=lambda _: None)
    node.diagnostics_pub = SimpleNamespace(publish=reports.append)
    node.publish_goal_editor = lambda: None
    node.last_snapshot_at, node.directory = 18., tmp_path
    node.session['mode'] = 'LIVE_VISUALIZATION_NO_MOTION'
    hold = dict(session_id=source_session, received_at_unix=source_stamp,
        state='goal_retained_waiting_recovery', planning_phase='goal_retained_waiting_recovery',
        goal_retained=True, active_reference=False, recovery_hold=dict(reason=reason, user_stamp=90.),
        active_goal=dict(session_id=source_session, epoch=2, seed_id='seed', user_stamp=90.))
    with patch('d1max_pct_scan.live_view.time.time', return_value=100.), \
         patch('d1max_pct_scan.live_view.time.monotonic', return_value=20.):
        LiveView.on_global(node, String(data=json.dumps(hold)))
        LiveView.tick(node)
    report = json.loads(reports[0].data)
    snapshot = json.loads((tmp_path/'view_status.json').read_text())
    if label is not None:
        assert report['stages']['global']['label'] == snapshot['reason'] == label
        assert report['stages']['local']['label'] in ('等待可用起点', '等待重新规划')
    else:
        assert report['stages']['global']['label'] == '等待目标'
        assert snapshot['global_status'] == {}
    assert report['motion_enabled'] is False and report['plan_id'] is None
    assert report['local_target'] is None
