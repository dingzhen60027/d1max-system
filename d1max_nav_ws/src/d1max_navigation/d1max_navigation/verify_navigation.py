"""Bounded Nav2 software-simulation acceptance; read-only unless opted in.

Even with --exercise-sim this executable cannot target the live Zenoh router.
It does not start nodes, reconnect sensors, modify maps, or use a robot SDK.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

from .simulation import OccupancyWorld, PlanarPose, validate_isolation


def raw_costmap_lethal_cells_near(msg, x, y, radius=.28):
    """Count raw lethal cells near a world point, including rotated origins."""
    meta = msg.metadata
    origin = meta.origin
    yaw = 2*math.atan2(origin.orientation.z, origin.orientation.w)
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x-origin.position.x, y-origin.position.y
    gx, gy = (c*dx+s*dy)/meta.resolution, (-s*dx+c*dy)/meta.resolution
    cells = radius/meta.resolution + math.sqrt(.5)
    count = 0
    for iy in range(max(0, math.floor(gy-cells)), min(meta.size_y, math.ceil(gy+cells)+1)):
        for ix in range(max(0, math.floor(gx-cells)), min(meta.size_x, math.ceil(gx+cells)+1)):
            if (ix+.5-gx)**2+(iy+.5-gy)**2 <= cells*cells and msg.data[iy*meta.size_x+ix] == 254:
                count += 1
    return count


def collision_stop_evidence(checked, safe, stop_events, inside_points, displacement):
    """Humble may intentionally stop publishing zero Twists after its timeout.

    Silence alone is never a pass: require an affirmative current polygon STOP
    event, fresh obstacle geometry, continuous safe-gate zeros and no movement.
    """
    if not stop_events:
        raise RuntimeError('No affirmative current Collision Monitor polygon STOP event')
    if inside_points < 4:
        raise RuntimeError('Near-obstacle scan did not contain enough points inside the stop polygon')
    if any(any(abs(value) > 1e-6 for value in entry[1:]) for entry in checked):
        raise RuntimeError('Collision Monitor leaked a NONZERO near-obstacle command')
    if len(safe) < 5 or any(any(abs(value) > 1e-6 for value in entry[1:]) for entry in safe):
        raise RuntimeError('The downstream command gate did not continuously publish zero during the stop test')
    if any(b[0]-a[0] > .2 for a, b in zip(safe, safe[1:])):
        raise RuntimeError('Safe-command output became stale during the stop test')
    if not math.isfinite(displacement) or displacement > .01:
        raise RuntimeError('Simulator moved during the collision-monitor stop test')
    return 'zero_twists' if checked else 'documented_zero_publication_timeout'


def main(args=None):
    import yaml
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exercise-sim', action='store_true', help='Explicitly allow goals in isolated SOFTWARE SIMULATION only')
    parser.add_argument('--expected-version', default='')
    parser.add_argument('--report', type=Path)
    parser.add_argument('--ready-timeout', type=float, default=25.)
    parser.add_argument('--goal-timeout', type=float, default=70.)
    options, ros_args = parser.parse_known_args(args)
    if not 1 <= options.ready_timeout <= 60 or not 5 <= options.goal_timeout <= 120:
        parser.error('Ready timeout must be 1–60 s and goal timeout 5–120 s')
    config_path = os.environ.get('ZENOH_SESSION_CONFIG_URI', '')
    if not config_path:
        parser.error('Isolated Zenoh session configuration is required, including for read-only inspection')
    validate_isolation(os.environ, yaml.safe_load(Path(config_path).read_text()))

    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from nav2_msgs.srv import GetCostmap
    from std_srvs.srv import SetBool
    from rcl_interfaces.msg import Log
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener

    rclpy.init(args=ros_args)
    node = Node('verify_navigation_simulation')
    observations = {'status': None, 'status_received': None, 'odom': None, 'odom_received': None,
                    'command': None, 'command_received': None, 'maps': {},
                    'scan': None, 'scan_received': None, 'scan_count': 0,
                    'collision_commands': [], 'safe_commands': [], 'collision_stop_events': []}
    report = {'mode': 'SOFTWARE_SIMULATION', 'robot_motion': False,
              'exercise_requested': options.exercise_sim, 'started_at': time.time(), 'checks': {}, 'passed': False}
    subscriptions = []
    active_goal = None
    obstacle_active = False
    namespace = '/d1max/navigation'

    def log(text):
        print(f'[Nav2 SIM verification] {text}', flush=True)

    def on_status(msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                observations['status'] = value
                observations['status_received'] = time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_odom(msg):
        observations['odom'] = msg
        observations['odom_received'] = time.monotonic()

    def on_command(msg):
        observations['command'] = (msg.linear.x, msg.linear.y, msg.angular.z)
        observations['command_received'] = time.monotonic()
        observations['safe_commands'].append((observations['command_received'], *observations['command']))

    def on_log(msg):
        if (msg.name == 'd1max.navigation.collision_monitor'
                and msg.msg == 'Robot to stop due to StopFootprint polygon'):
            observations['collision_stop_events'].append((time.monotonic(), msg.msg))

    def on_scan(msg):
        observations['scan'] = msg
        observations['scan_received'] = time.monotonic()
        observations['scan_count'] += 1

    transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions.append(node.create_subscription(String, namespace+'/runtime_status', on_status, 2))
    subscriptions.append(node.create_subscription(Odometry, '/d1max/localization/odometry/global', on_odom, 2))
    subscriptions.append(node.create_subscription(Twist, namespace+'/cmd_vel_safe', on_command, 2))
    subscriptions.append(node.create_subscription(Log, '/rosout', on_log, 20))
    subscriptions.append(node.create_subscription(Twist, namespace+'/cmd_vel_collision_checked',
        lambda msg: observations['collision_commands'].append(
            (time.monotonic(), msg.linear.x, msg.linear.y, msg.angular.z)), 10))
    subscriptions.append(node.create_subscription(LaserScan, namespace+'/scan', on_scan,
                         QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)))
    for name, topic in [('map', namespace+'/map'),
                        ('global_costmap', namespace+'/global_costmap/costmap'),
                        ('local_costmap', namespace+'/local_costmap/costmap')]:
        subscriptions.append(node.create_subscription(OccupancyGrid, topic,
                             lambda msg, key=name: observations['maps'].__setitem__(key, msg), transient))
    navigator = ActionClient(node, NavigateToPose, namespace+'/navigate_to_pose')
    planner = ActionClient(node, ComputePathToPose, namespace+'/compute_path_to_pose')
    tf = Buffer()
    listener = TransformListener(tf, node)
    obstacle_client = node.create_client(SetBool, namespace+'/simulation/obstacle')
    close_obstacle_client = node.create_client(SetBool, namespace+'/simulation/close_obstacle')
    costmap_clients = {name: node.create_client(GetCostmap, namespace+'/'+name+'/get_costmap')
                       for name in ('global_costmap', 'local_costmap')}
    collision_test_pub = node.create_publisher(Twist, namespace+'/cmd_vel_smoothed', 2)

    def current_simulation():
        status = observations['status']
        received = observations['status_received']
        if received is None or not 0 <= time.monotonic()-received <= 2.:
            raise RuntimeError('No fresh explicit SOFTWARE SIMULATION status')
        required = {'mode': 'SIMULATION', 'robot_connected': False, 'real_motion_enabled': False,
                    'ready': True, 'fault': '', 'router': '127.0.0.1:7460'}
        for key, value in required.items():
            if status.get(key) != value or (isinstance(value, bool) and status.get(key) is not value):
                raise RuntimeError(f'Simulation guard failed: {key}={status.get(key)!r}')
        if not isinstance(status.get('wall_time'), (int, float)) or not -.2 <= time.time()-status['wall_time'] <= 2.:
            raise RuntimeError('Simulation source timestamp is stale')
        if not isinstance(status.get('map_version_id'), str) or not status['map_version_id']:
            raise RuntimeError('Simulation map is not pinned to a version')
        if options.expected_version and status['map_version_id'] != options.expected_version:
            raise RuntimeError('Simulation map version differs from expected version')
        for key in ('commands_received', 'collision_stops', 'synthetic_scans'):
            if not isinstance(status.get(key), int) or status[key] < 0:
                raise RuntimeError('Unexpected simulator status schema: '+key)
        return status

    def progress():
        status = observations['status'] or {}
        p = status.get('pose', {})
        return f"pose=({p.get('x', '?')}, {p.get('y', '?')}), distance={status.get('distance_m', '?')}, safe_cmd={observations['command']}"

    def wait_until(predicate, timeout, description, require_simulation=False):
        deadline, next_log = time.monotonic()+timeout, time.monotonic()+5.
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if require_simulation:
                current_simulation()
            result = predicate()
            if result:
                return result
            if time.monotonic() >= next_log:
                log(description+'; '+progress())
                next_log = time.monotonic()+5.
        raise TimeoutError(f'Timed out: {description}')

    def fresh_odom():
        msg = observations['odom']
        received = observations['odom_received']
        if msg is None or received is None or time.monotonic()-received > .5:
            raise RuntimeError('Global odometry is not fresh')
        if msg.header.frame_id != 'd1max_loc_map' or msg.child_frame_id != 'd1max_loc_base_link':
            raise RuntimeError('Global odometry frames are inconsistent')
        return msg

    def ready():
        try:
            current_simulation()
            fresh_odom()
            if (observations['scan_received'] is None
                    or time.monotonic()-observations['scan_received'] > .5
                    or observations['scan_count'] < 2):
                return False
            if not navigator.server_is_ready() or not planner.server_is_ready():
                return False
            for name, frame in [('map', 'd1max_loc_map'), ('global_costmap', 'd1max_loc_map'),
                                ('local_costmap', 'd1max_loc_odom')]:
                msg = observations['maps'].get(name)
                if (msg is None or msg.header.frame_id != frame or msg.info.width < 1
                        or msg.info.height < 1 or len(msg.data) != msg.info.width*msg.info.height):
                    return False
            return tf.can_transform('d1max_loc_map', 'd1max_loc_base_link', rclpy.time.Time())
        except RuntimeError:
            return False

    def message_pose(goal, value):
        if not isinstance(value, dict) or any(not isinstance(value.get(k), (int, float))
                                             or not math.isfinite(value[k]) for k in ('x', 'y', 'yaw')):
            raise RuntimeError('Simulator suggested goal is missing or invalid')
        goal.header.frame_id = 'd1max_loc_map'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = float(value['x']), float(value['y'])
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(value['yaw']/2), math.cos(value['yaw']/2)

    def send_goal(value):
        nonlocal active_goal
        current_simulation()
        if not options.exercise_sim:
            raise RuntimeError('Navigation goal forbidden without --exercise-sim')
        goal = NavigateToPose.Goal()
        message_pose(goal.pose, value)
        future = navigator.send_goal_async(goal)
        wait_until(future.done, 5., 'waiting for simulated navigation goal acceptance', True)
        active_goal = future.result()
        if active_goal is None or not active_goal.accepted:
            active_goal = None
            raise RuntimeError('Nav2 rejected the simulation goal')
        return active_goal

    def change_obstacle(enabled, close=False):
        nonlocal obstacle_active
        current_simulation()
        if not options.exercise_sim:
            raise RuntimeError('Obstacle injection requires --exercise-sim')
        client = close_obstacle_client if close else obstacle_client
        if not client.wait_for_service(timeout_sec=2.):
            raise RuntimeError('Isolated simulator obstacle service unavailable')
        request = SetBool.Request(); request.data = enabled
        future = client.call_async(request)
        wait_until(future.done, 4., 'waiting for simulated obstacle change', True)
        response = future.result()
        if not response.success:
            raise RuntimeError('Simulator refused obstacle: '+response.message)
        obstacle_active = enabled
        return json.loads(response.message) if enabled else None

    def costmap_snapshots():
        result = {}
        for name, client in costmap_clients.items():
            if not client.wait_for_service(timeout_sec=2.):
                raise RuntimeError(name+' GetCostmap service unavailable')
            future = client.call_async(GetCostmap.Request())
            wait_until(future.done, 4., 'reading '+name, True)
            result[name] = future.result().map
        return result

    def exercise_obstacles():
        before = costmap_snapshots()
        obstacle = change_obstacle(True)
        x, y = obstacle['x'], obstacle['y']
        baseline = {name: raw_costmap_lethal_cells_near(grid, x, y) for name, grid in before.items()}
        report['checks']['dynamic_obstacle'] = {'position': obstacle, 'baseline_lethal': baseline,
            'inserted_lethal': {}, 'cleared_lethal': {}, 'passed': False, 'stage': 'inserting'}
        if any(baseline.values()):
            raise RuntimeError('Dynamic-obstacle test region already contains lethal cost; cannot prove insertion')
        marked = {}
        def has_marks():
            marked.update({name: raw_costmap_lethal_cells_near(grid, x, y)
                           for name, grid in costmap_snapshots().items()})
            report['checks']['dynamic_obstacle']['inserted_lethal'] = dict(marked)
            return all(value > 0 for value in marked.values())
        wait_until(has_marks, 12., 'verifying new obstacle appears in BOTH costmaps', True)
        report['checks']['obstacle_input']['dynamic_obstacle_marking_verified'] = True
        report['checks']['dynamic_obstacle']['stage'] = 'clearing'
        change_obstacle(False)
        cleared = {}
        def has_cleared():
            cleared.update({name: raw_costmap_lethal_cells_near(grid, x, y)
                            for name, grid in costmap_snapshots().items()})
            report['checks']['dynamic_obstacle']['cleared_lethal'] = dict(cleared)
            return all(value == 0 for value in cleared.values())
        wait_until(has_cleared, 12., 'verifying removed obstacle clears from BOTH costmaps', True)
        report['checks']['dynamic_obstacle'].update(passed=True, stage='complete')
        report['checks']['obstacle_input']['dynamic_obstacle_marking_verified'] = True
        report['checks']['obstacle_input']['dynamic_obstacle_clearing_verified'] = True
        report['checks']['obstacle_input']['limitation'] = 'Synthetic scan insertion/clearing only; no physical obstacle or sensor acceptance.'

        # No Nav2 goal is active. Inject a near obstacle into synthetic scan,
        # then exercise the *official* collision monitor with a small private
        # test command. The isolated router guard precludes a live target.
        # Ask the monitor to reevaluate the now-cleared scan with a zero input
        # first, so a STOP event belongs to this injection, not an older test.
        collision_test_pub.publish(Twist())
        settle_deadline = time.monotonic()+.1
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        obstacle_started = time.monotonic()
        close_obstacle = change_obstacle(True, close=True)
        scan_before = observations['scan_count']
        wait_until(lambda: observations['scan_count'] >= scan_before+3, 2.,
                   'waiting for close-obstacle synthetic scans', True)
        pose = fresh_odom().pose.pose.position
        initial_xy = (pose.x, pose.y)
        started = time.monotonic()
        sent = 0
        scan = observations['scan']
        inside_points = sum(1 for i, distance in enumerate(scan.ranges)
            if math.isfinite(distance)
            and -.62 < distance*math.cos(scan.angle_min+i*scan.angle_increment) < .62
            and -.42 < distance*math.sin(scan.angle_min+i*scan.angle_increment) < .42)
        scan_age = node.get_clock().now().nanoseconds*1e-9 - scan.header.stamp.sec - scan.header.stamp.nanosec*1e-9
        if scan.header.frame_id != 'd1max_loc_base_link' or not 0 <= scan_age < .5:
            raise RuntimeError('Near-obstacle scan frame or timestamp invalid')
        test_command = Twist(); test_command.linear.x = .1
        while time.monotonic()-started < .7:
            current_simulation()
            collision_test_pub.publish(test_command)
            sent += 1
            # spin_once(timeout) is NOT a rate limiter when other callbacks are
            # ready; drain callbacks for a whole 50 ms instead of flooding input.
            tick_deadline = time.monotonic()+.05
            while time.monotonic() < tick_deadline:
                rclpy.spin_once(node, timeout_sec=min(.02, max(0., tick_deadline-time.monotonic())))
        collision_test_pub.publish(Twist())
        checked = [entry for entry in observations['collision_commands'] if entry[0] >= started]
        safe = [entry for entry in observations['safe_commands'] if entry[0] >= started]
        stop_events = [event for event in observations['collision_stop_events'] if event[0] >= obstacle_started]
        pose = fresh_odom().pose.pose.position
        drift = math.hypot(pose.x-initial_xy[0], pose.y-initial_xy[1])
        report['checks']['collision_monitor_stop'] = {'obstacle': close_obstacle,
            'input_commands': sent, 'checked_zero_commands': len(checked),
            'checked_commands': checked, 'safe_commands': safe, 'polygon_stop_events': stop_events,
            'points_inside_stop_polygon': inside_points, 'scan_age_sec': scan_age,
            'displacement_m': drift, 'passed': False}
        output_mode = collision_stop_evidence(checked, safe, stop_events, inside_points, drift)
        report['checks']['collision_monitor_stop'].update(passed=True, output_mode=output_mode)
        change_obstacle(False)
        collision_test_pub.publish(Twist())

    try:
        log('READ-ONLY readiness checks; robot disconnected, isolated Zenoh port 7460')
        wait_until(ready, options.ready_timeout, 'waiting for map, both costmaps, TF and Nav2 actions')
        status = current_simulation()
        start = dict(status['pose'])
        report['map_version_id'] = status['map_version_id']
        report['checks']['map_and_costmaps'] = {
            name: {'frame': msg.header.frame_id, 'width': msg.info.width, 'height': msg.info.height,
                   'resolution': msg.info.resolution, 'occupied_cells': sum(v >= 65 for v in msg.data),
                   'lethal_cells': sum(v == 100 for v in msg.data),
                   'inscribed_cells': sum(v == 99 for v in msg.data),
                   'inflated_cells': sum(0 < v < 99 for v in msg.data),
                   'unknown_cells': sum(v == -1 for v in msg.data)}
            for name, msg in observations['maps'].items()}
        scan = observations['scan']
        scan_subscribers = [{'node': info.node_namespace.rstrip('/')+'/'+info.node_name,
                             'topic_type': info.topic_type}
                            for info in node.get_subscriptions_info_by_topic(namespace+'/scan')]
        costmap_subscribers = sorted({info['node'] for info in scan_subscribers
                                     if info['node'].endswith('/global_costmap') or info['node'].endswith('/local_costmap')})
        for name in ('global_costmap', 'local_costmap'):
            if not any(path.endswith('/'+name) for path in costmap_subscribers):
                raise RuntimeError(name+' is not subscribed to the synthetic obstacle scan')
            metrics = report['checks']['map_and_costmaps'][name]
            if metrics['lethal_cells'] < 1 or metrics['inflated_cells'] < 1:
                raise RuntimeError(name+' has no lethal or inflated cells')
        report['checks']['obstacle_input'] = {
            'scan_frame': scan.header.frame_id, 'rays': len(scan.ranges),
            'received_scans': observations['scan_count'],
            'finite_returns': sum(math.isfinite(v) for v in scan.ranges),
            'infinite_clear_rays': sum(math.isinf(v) for v in scan.ranges),
            'unknown_nan_rays': sum(math.isnan(v) for v in scan.ranges),
            'costmap_scan_subscribers': costmap_subscribers,
            'input_wiring_verified': True,
            'dynamic_obstacle_marking_verified': False,
            'limitation': 'Both costmaps also contain the static map. Nonzero costs and scan subscriptions alone do not prove insertion/clearing of a new dynamic obstacle.'}
        report['checks']['actions'] = {'navigate_to_pose_ready': True, 'compute_path_to_pose_ready': True}
        report['checks']['tf_map_to_base'] = True
        report['checks']['fresh_simulation'] = True
        log('Map + global/local costmaps + map→base TF + planner/navigator action servers verified')
        if options.exercise_sim:
            collision_before = status['collision_stops']
            goal_value = status.get('suggested_test_goal')
            goal_source = 'simulator_suggestion'
            if (not isinstance(goal_value, dict)
                    or math.hypot(goal_value.get('x', start['x'])-start['x'],
                                  goal_value.get('y', start['y'])-start['y']) < 1.):
                # A second acceptance run may start at the previous finish.
                # Select a new clear test segment from the pinned map, not a
                # remembered physical-robot pose or a hardcoded destination.
                grid = observations['maps']['map']
                origin = grid.info.origin
                world = OccupancyWorld(grid.data, grid.info.width, grid.info.height, grid.info.resolution,
                                       (origin.position.x, origin.position.y,
                                        2*math.atan2(origin.orientation.z, origin.orientation.w)))
                goal_value = world.suggest_goal(PlanarPose(start['x'], start['y'], start['yaw']), radius=.62)
                goal_source = 'fresh_clear_segment_from_same_map'
                if goal_value is None:
                    raise RuntimeError('No safe 3–5 m simulation test segment from current pose; reset SIM pose first')
            report['checks']['test_goal_source'] = goal_source
            log('EXPLICIT SOFTWARE SIMULATION exercise: planner followed by navigation success')
            path_request = ComputePathToPose.Goal()
            message_pose(path_request.goal, goal_value)
            path_request.use_start = False
            path_request.planner_id = 'GridBased'
            path_future = planner.send_goal_async(path_request)
            wait_until(path_future.done, 5., 'waiting for global planner', True)
            path_handle = path_future.result()
            if not path_handle.accepted:
                raise RuntimeError('Planner rejected simulation path request')
            path_result = path_handle.get_result_async()
            wait_until(path_result.done, 10., 'waiting for global path', True)
            path_response = path_result.result()
            if path_response.status != GoalStatus.STATUS_SUCCEEDED or len(path_response.result.path.poses) < 2:
                raise RuntimeError('Global planner did not return a successful nonempty path')
            report['checks']['global_plan'] = {'status': 'SUCCEEDED', 'poses': len(path_response.result.path.poses)}
            begun = time.monotonic()
            goal_handle = send_goal(goal_value)
            result_future = goal_handle.get_result_async()
            wait_until(result_future.done, options.goal_timeout, 'navigating through simulated closed loop', True)
            response = result_future.result()
            active_goal = None
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'NavigateToPose did not succeed: action status {response.status}')
            odom = fresh_odom()
            error = math.hypot(odom.pose.pose.position.x-goal_value['x'], odom.pose.pose.position.y-goal_value['y'])
            if error >= .3:
                raise RuntimeError(f'Navigation action succeeded but position error is {error:.3f} m')
            if current_simulation()['collision_stops'] != collision_before:
                raise RuntimeError('Simulation collision guard triggered during goal exercise')
            report['checks']['navigate_to_pose'] = {'status': 'SUCCEEDED', 'goal': goal_value,
                                                    'position_error_m': error, 'duration_sec': time.monotonic()-begun}
            log(f'NavigateToPose SUCCEEDED, position error={error:.3f} m; checking cancel + stopped command')
            second = send_goal(start)
            second_result = second.get_result_async()
            moving_after = time.monotonic()
            wait_until(lambda: observations['command_received'] is not None
                       and observations['command_received'] > moving_after
                       and any(abs(v) > .01 for v in observations['command']),
                       20., 'waiting for motion in second simulated goal', True)
            cancellation = second.cancel_goal_async()
            wait_until(cancellation.done, 5., 'waiting for navigation cancel response', True)
            cancellation_response = cancellation.result()
            if not cancellation_response.goals_canceling:
                raise RuntimeError('Nav2 did not accept goal cancellation')
            wait_until(second_result.done, 8., 'waiting for CANCELED action result', True)
            if second_result.result().status != GoalStatus.STATUS_CANCELED:
                raise RuntimeError('Second navigation goal did not finish with CANCELED status')
            active_goal = None
            after_cancel = time.monotonic()
            wait_until(lambda: observations['command_received'] is not None
                       and observations['command_received'] > after_cancel
                       and all(abs(v) < 1e-6 for v in observations['command']),
                       3., 'waiting for zero safe command after cancellation', True)
            stopped = fresh_odom().pose.pose.position
            stopped_xy = (stopped.x, stopped.y)
            until = time.monotonic()+.7
            while rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.1)
                current_simulation()
                if time.monotonic()-observations['command_received'] > .3:
                    raise RuntimeError('Safe-command publication became stale after cancellation')
                if any(abs(v) > 1e-6 for v in observations['command']):
                    raise RuntimeError('A nonzero safe command reappeared after cancellation')
            final_position = fresh_odom().pose.pose.position
            cancel_drift = math.hypot(final_position.x-stopped_xy[0], final_position.y-stopped_xy[1])
            if cancel_drift > .01:
                raise RuntimeError(f'Simulator moved {cancel_drift:.3f} m after safe stop')
            if current_simulation()['collision_stops'] != collision_before:
                raise RuntimeError('Simulation collision guard triggered during navigation or cancellation')
            report['checks']['cancel_and_stop'] = {'status': 'CANCELED', 'safe_command': list(observations['command']),
                                                    'post_stop_displacement_m': cancel_drift}
            log('Cancel stopped motion; checking dynamic obstacle insertion/clearing and official collision stop')
            exercise_obstacles()
            report['checks']['simulation_metrics'] = current_simulation()
        report['passed'] = True
        log('PASSED' + (': planner, navigation success, cancel and zero motion' if options.exercise_sim else ': read-only readiness only; no goal was sent'))
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        log('FAILED: '+report['error'])
    finally:
        if options.exercise_sim:
            collision_test_pub.publish(Twist())
        if obstacle_active:
            try:
                change_obstacle(False)
                collision_test_pub.publish(Twist())
                report['cleanup_obstacle_cleared'] = True
            except Exception as exc:
                report['cleanup_obstacle_error'] = str(exc)
        if active_goal is not None:
            try:
                # Only this verifier's own simulator goal; never cancel others.
                cancellation = active_goal.cancel_goal_async()
                deadline = time.monotonic()+3.
                while rclpy.ok() and not cancellation.done() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.1)
                report['cleanup_cancel_requested'] = True
            except Exception as exc:
                report['cleanup_error'] = str(exc)
        report['finished_at'] = time.time()
        rendered = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
        if options.report:
            # Runtime acceptance artifact, not source/configuration mutation.
            options.report.parent.mkdir(parents=True, exist_ok=True)
            options.report.write_text(rendered+'\n')
        print(rendered, flush=True)
        navigator.destroy()
        planner.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
