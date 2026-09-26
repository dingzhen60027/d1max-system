"""Bounded local Nav2 operator commands; never creates an SDK connection.

Every mutation pins the owned systemd unit, navigation generation and map.
Simulation uses only its loopback router. Acceptance is not goal completion.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
import time

from .command_gate import finite

ACTION_LABELS = {0: 'unknown', 1: 'accepted', 2: 'executing', 3: 'canceling',
                 4: 'succeeded', 5: 'canceled', 6: 'aborted'}
ACTIVE_STATUSES = (1, 2, 3)


def observe_action(actions, status_seen, name, status_list):
    """An older active goal must not be hidden by newer terminal history."""
    status_seen.add(name)
    if status_list:
        status = max(status_list, key=lambda item: (
            item.status in ACTIVE_STATUSES,
            item.goal_info.stamp.sec, item.goal_info.stamp.nanosec))
        actions[name] = {'status': status.status, 'label': ACTION_LABELS.get(status.status, 'unknown'),
                        'goal_id': bytes(status.goal_info.goal_id.uuid).hex(), 'action': name,
                        'stamp': status.goal_info.stamp.sec + status.goal_info.stamp.nanosec*1e-9}
    else:
        # Empty is an observed idle state, not silence and not stale history.
        actions.pop(name, None)
    return max(actions.values(), key=lambda item:
               (item['status'] in ACTIVE_STATUSES, item['stamp']), default=None)


def motion_admission(operation, gate, session, actions, wall):
    """Recheck the latest observations immediately before a motion mutation."""
    if operation not in ('arm', 'goal'):
        raise ValueError('Motion admission is only defined for arm/goal')
    current = validated_gate(gate, session, wall)
    if not current:
        raise ValueError('No fresh command gate; motion remains locked')
    if any(item['status'] in ACTIVE_STATUSES for item in actions.values()):
        raise ValueError('Cancel the existing goal before arming or sending a new goal; an old goal must not resume implicitly')
    if operation == 'goal':
        goal_allowed(current, session)
    return current


def validate_context(unit, session, session_id=None, version_id=None):
    from .navigation_session import OWNER
    ident = session.get('navigation_session_id', '')
    version = session.get('version_id', '')
    mode = session.get('mode')
    if (not re.fullmatch('[a-f0-9]{32}', ident) or mode not in ('sim', 'live')
            or not isinstance(version, str) or not version):
        raise ValueError('Navigation session is missing a current generation; restart navigation')
    expected = OWNER + mode + ':' + version + ':' + ident
    if unit.get('ActiveState') != 'active' or unit.get('Description') != expected:
        raise ValueError('Navigation session is stopped, replaced or not owned')
    if session_id is not None and (session_id != ident or version_id != version):
        raise ValueError('Stale navigation session or map; refresh before issuing commands')
    return ident, version


def validated_gate(payload, session, wall):
    if not isinstance(payload, dict):
        return None
    stamp = payload.get('wall_time')
    if (not finite(stamp) or not 0 <= wall - stamp <= 1.
            or payload.get('navigation_session_id') != session['navigation_session_id']
            or payload.get('map_version_id') != session['version_id']
            or payload.get('mode') != ('simulation' if session['mode'] == 'sim' else 'live')):
        return None
    return payload


def goal_allowed(gate, session):
    if not gate or gate.get('armed') is not True:
        raise ValueError('Motion gate is not armed; initialize localization and explicitly arm first')
    if session['mode'] == 'live' and (session.get('enable_motion') is not True
                                      or gate.get('motion_enabled') is not True):
        raise ValueError('Live motion capability is disabled')
    if gate.get('reason') not in ('simulation_only', 'live_guard_passed', 'command_missing_or_stale'):
        raise ValueError('Navigation health gate refused goal: ' + str(gate.get('reason')))


def validate_goal(x, y, yaw, world):
    if not all(finite(v) for v in (x, y, yaw)):
        raise ValueError('Goal x/y/yaw must be finite SI values')
    # Conservative circumscribed walking envelope, shared with the simulator.
    if not world.clear(x, y, .62):
        raise ValueError('Goal footprint overlaps occupied/unknown cells or lies outside the map')
    return float(x), float(y), math.atan2(math.sin(yaw), math.cos(yaw))


def load_world(map_path):
    import numpy as np
    from PIL import Image
    import yaml
    from .simulation import OccupancyWorld
    path = Path(map_path).resolve(strict=True)
    grid = yaml.safe_load(path.read_text())
    image_path = (path.parent / grid['image']).resolve(strict=True)
    if not image_path.is_relative_to(path.parent) or grid.get('mode', 'trinary') != 'trinary':
        raise ValueError('Only version-contained trinary map images are accepted')
    with Image.open(image_path) as image:
        pixels = np.asarray(image.convert('L'), dtype=float) / 255.
    occupancy = pixels if grid.get('negate', 0) else 1. - pixels
    data = np.where(occupancy > grid['occupied_thresh'], 100,
                    np.where(occupancy < grid['free_thresh'], 0, -1))
    data = np.flipud(data)
    return OccupancyWorld(data.ravel(), data.shape[1], data.shape[0],
                          grid['resolution'], tuple(grid['origin']))


def execute(args, app, nav, root):
    """CLI entry. stdout is one JSON object; errors propagate to session CLI."""
    from .navigation_session import info
    session_path = root / 'last_session.json'
    if not session_path.is_file():
        raise ValueError('No navigation session exists')
    session = json.loads(session_path.read_text())
    mutation = args.operation != 'status'
    ident, version = validate_context(info(), session,
        args.session_id if mutation else None, args.version_id if mutation else None)
    if mutation and (not args.session_id or not args.version_id):
        raise ValueError('A navigation generation and map version are required')
    if args.operation == 'goal':
        target = validate_goal(args.x, args.y, args.yaw, load_world(session['map_yaml']))
    if os.environ.get('ZENOH_CONFIG_OVERRIDE') or os.environ.get('ZENOH_SESSION_CONFIG'):
        raise ValueError('Additional Zenoh configuration overrides are not allowed')
    if session['mode'] == 'sim':
        from ament_index_python.packages import get_package_share_directory
        from .simulation import validate_isolation
        import yaml
        router_config = Path(get_package_share_directory('d1max_navigation')) / 'config/zenoh-offline-session.json5'
        os.environ['ZENOH_SESSION_CONFIG_URI'] = str(router_config)
        validate_isolation(os.environ, yaml.safe_load(router_config.read_text()))
    else:
        os.environ['ZENOH_SESSION_CONFIG_URI'] = str(app / 'foxglove_d1max/config/zenoh-live.json5')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp' or os.environ.get('ROS_DOMAIN_ID') != '24':
        raise ValueError('Navigation commands require the configured Zenoh domain 24')

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from action_msgs.msg import GoalStatusArray
    from action_msgs.srv import CancelGoal
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    rclpy.init(args=[])
    node = rclpy.create_node('d1max_nav_operator_' + str(os.getpid()))
    gate, action = None, None
    actions = {}
    status_seen = set()
    action_names = ('navigate_to_pose', 'navigate_through_poses', 'follow_waypoints')

    def on_gate(message):
        nonlocal gate
        try:
            gate = validated_gate(json.loads(message.data), session, time.time())
        except (ValueError, TypeError):
            gate = None

    def on_action(message, name):
        nonlocal action
        action = observe_action(actions, status_seen, name, message.status_list)

    def wait(future, timeout=3.):
        until = time.monotonic() + timeout
        while not future.done() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        if not future.done():
            raise RuntimeError('Command acknowledgement timed out; execution is UNKNOWN; do not automatically retry')
        return future.result()

    def still_current():
        current = json.loads(session_path.read_text())
        validate_context(info(), current, ident, version)

    def cancel_actions(names):
        clients = {name: node.create_client(CancelGoal, '/d1max/navigation/'+name+'/_action/cancel_goal')
                   for name in names}
        try:
            if not all(client.wait_for_service(timeout_sec=.5) for client in clients.values()):
                raise RuntimeError('Navigation cancellation service unavailable; use motion lock or stop')
            futures = {name: client.call_async(CancelGoal.Request()) for name, client in clients.items()}
            results = {name: wait(future) for name, future in futures.items()}
            if any(result.return_code != 0 for result in results.values()):
                raise RuntimeError('Cancellation was not acknowledged; motion remains blocked')
            return [bytes(g.goal_id.uuid).hex() for result in results.values() for g in result.goals_canceling]
        finally:
            for client in clients.values():
                node.destroy_client(client)

    def ensure_no_unobserved_goal():
        # Humble action servers do not necessarily publish an initial empty
        # status. Silence alone must not authorize movement. For an unobserved
        # action, request cancellation and require an ACK with NO active goals.
        # If any were found, do not arm/start a new goal: the operator must
        # inspect the canceled state and issue a fresh explicit request.
        unknown = [name for name in action_names if name not in status_seen]
        if unknown and cancel_actions(unknown):
            raise RuntimeError('Previously unobserved goals are being canceled; verify stopped state before a new request')

    def refresh_motion_admission():
        # Service discovery and cancellation ACKs can take seconds. Dispatch
        # pending callbacks before rechecking: the pre-wait idle snapshot must
        # never authorize a goal/arm after an active state has been observed.
        until = time.monotonic() + .15
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.01)
        still_current()
        current = motion_admission(args.operation, gate, session, actions, time.time())
        response.update(gate=current, action=action, actions=dict(actions),
                        action_status_seen=sorted(status_seen), wall_time=time.time())

    try:
        node.create_subscription(String, '/d1max/navigation/command_gate/status', on_gate, 5)
        for name in action_names:
            node.create_subscription(GoalStatusArray, '/d1max/navigation/'+name+'/_action/status',
                lambda message, name=name: on_action(message, name),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))
        until = time.monotonic() + 2.
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
            if gate and len(status_seen) == len(action_names):
                break
        gate = validated_gate(gate, session, time.time())
        response = {'navigation_session_id': ident, 'version_id': version,
                    'simulation': session['mode'] == 'sim', 'operation': args.operation,
                    'gate': gate, 'action': action, 'actions': actions, 'runtime_available': gate is not None,
                    'action_status_seen': sorted(status_seen),
                    'wall_time': time.time()}
        still_current()
        if args.operation in ('arm', 'disarm'):
            if args.operation == 'arm':
                motion_admission('arm', gate, session, actions, time.time())
                ensure_no_unobserved_goal()
            client = node.create_client(SetBool, '/d1max/navigation/navigation_command_gate/arm')
            if not client.wait_for_service(timeout_sec=2.):
                raise RuntimeError('Motion gate service is unavailable')
            request = SetBool.Request()
            request.data = args.operation == 'arm'
            if request.data:
                refresh_motion_admission()
            else:
                still_current()
            result = wait(client.call_async(request))
            response.update(accepted=result.success, message=result.message)
            # A queued asynchronous SDK arm is explicitly NOT confirmed armed.
            response['armed_confirmed'] = False
            if not result.success:
                response['error'] = result.message
        elif args.operation == 'cancel':
            # Cancels current Nav2 action(s) in this pinned owned stack, including
            # RViz goals. Empty UUID/zero timestamp is ROS action cancel-all.
            canceled = cancel_actions(action_names)
            response.update(accepted=True, canceling_goals=canceled,
                            canceled_actions=list(action_names),
                            message='Cancellation requested; verify canceled status and zero velocity')
        elif args.operation == 'goal':
            motion_admission('goal', gate, session, actions, time.time())
            ensure_no_unobserved_goal()
            client = ActionClient(node, NavigateToPose, '/d1max/navigation/navigate_to_pose')
            if not client.wait_for_server(timeout_sec=2.):
                raise RuntimeError('NavigateToPose action is unavailable')
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = session['map_frame']
            goal.pose.pose.position.x, goal.pose.pose.position.y = target[:2]
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(target[2]/2), math.cos(target[2]/2)
            refresh_motion_admission()
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            handle = wait(client.send_goal_async(goal))
            response.update(accepted=handle.accepted, goal_id=bytes(handle.goal_id.uuid).hex(),
                            message='Goal accepted, NOT arrived' if handle.accepted else 'Goal rejected',
                            target={'x': target[0], 'y': target[1], 'yaw': target[2]})
        elif args.operation != 'status':
            raise ValueError('Unsupported navigation operation')
        print(json.dumps(response, ensure_ascii=False, allow_nan=False))
        return response
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
