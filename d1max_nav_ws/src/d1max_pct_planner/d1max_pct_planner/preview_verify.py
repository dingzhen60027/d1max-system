"""Bounded ROS acceptance test for the planning-only RViz XYZ workbench.

Only preview activation, selection, marker feedback, Plan and Clear are exercised.
--require-rviz additionally checks the actual RViz interactive-marker graph and
its QoS compatibility, not just the backend. This is not a rendered-screen,
GUI mouse, navigation, physical-robot or localization acceptance test.
"""
import argparse
import json
from pathlib import Path as FilePath
import time
import traceback

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSCompatibility, QoSProfile,
                       ReliabilityPolicy, qos_check_compatible)
from std_msgs.msg import Empty, String
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.srv import GetInteractiveMarkers

from d1max_pct_planner.measured_grid import MeasuredGrid


PREFIX = '/d1max/pct_preview'
FRAME = 'd1max_loc_map'
MARKERS = '/pct_preview_points'


class PreviewVerifier(Node):
    def __init__(self, report):
        super().__init__('pct_preview_verify')
        self.report = report
        self.status = None
        self.path = None
        self.path_count = 0
        self.status_count = 0
        self.events = []
        self.last_state = None
        self.started = time.monotonic()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.latched = latched
        self.create_subscription(String, PREFIX + '/status', self.on_status, latched)
        self.create_subscription(Path, PREFIX + '/path', self.on_path, latched)
        self.points = {role: self.create_publisher(PointStamped, PREFIX + '/' + role, 10)
                       for role in ('start', 'goal')}
        self.commands = {name: self.create_publisher(Empty, PREFIX + '/' + name, 10)
                         for name in ('clear', 'plan', 'snap_goal', 'activate_start', 'activate_goal',
                                      'reset_start_orientation', 'reset_goal_orientation')}
        self.feedback = self.create_publisher(
            InteractiveMarkerFeedback, MARKERS + '/feedback', 10)
        self.marker_client = self.create_client(
            GetInteractiveMarkers, MARKERS + '/get_interactive_markers')

    def on_status(self, message):
        self.status = json.loads(message.data)
        self.status_count += 1
        state = (self.status.get('state'), self.status.get('revision'), self.status.get('reason'))
        if state != self.last_state:
            self.events.append({'elapsed_s': time.monotonic() - self.started,
                                'state': state[0], 'revision': state[1], 'reason': state[2]})
            self.last_state = state

    def on_path(self, message):
        self.path = message
        self.path_count += 1

    def wait(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        raise AssertionError(f'Timeout: {label}; last status={self.status}')

    def hold(self, duration, predicate, label):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
            if not predicate():
                raise AssertionError(f'{label}; last status={self.status}')

    def check(self, name, **details):
        record = {'name': name, 'passed': True, **details}
        self.report['checks'].append(record)
        print(json.dumps(record, allow_nan=False), flush=True)

    def xyz(self, role):
        return None if self.status is None else self.status.get(role + '_xyz')

    def matches(self, role, xyz):
        actual = self.xyz(role)
        return actual is not None and np.allclose(actual, xyz, atol=1e-8, rtol=0)

    def orientation(self, role):
        values = self.status.get(role + '_orientation_xyzw') if self.status else None
        quaternion = np.asarray(values, dtype=float)
        if (quaternion.shape != (4,) or not np.isfinite(quaternion).all()
                or not np.isclose(np.linalg.norm(quaternion), 1.0, atol=1e-8, rtol=0)):
            raise AssertionError(f'{role} editor orientation must be a normalized XYZW quaternion: {values}')
        return quaternion

    def orientation_matches(self, role, expected):
        actual = self.orientation(role)
        return (np.allclose(actual, expected, atol=1e-8, rtol=0)
                or np.allclose(actual, -np.asarray(expected), atol=1e-8, rtol=0))

    def feedback_message(self, role, control, xyz, orientation=None):
        message = InteractiveMarkerFeedback()
        message.header.frame_id = FRAME
        message.client_id = ''
        message.marker_name = role
        message.control_name = control
        quaternion = self.orientation(role) if orientation is None else orientation
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = map(float, quaternion)
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, xyz)
        return message

    def assert_marker_pose(self, role, xyz, orientation):
        marker = self.get_markers()[role]
        point, quaternion = marker.pose.position, marker.pose.orientation
        actual_xyz = [point.x, point.y, point.z]
        actual_q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        if (marker.header.frame_id != FRAME or not np.allclose(actual_xyz, xyz, atol=1e-8, rtol=0)
                or not np.isclose(np.linalg.norm(actual_q), 1.0, atol=1e-8, rtol=0)
                or not (np.allclose(actual_q, orientation, atol=1e-8, rtol=0)
                        or np.allclose(actual_q, -np.asarray(orientation), atol=1e-8, rtol=0))):
            raise AssertionError(f'{role} interactive marker pose does not match editor pose: '
                                 f'xyz={actual_xyz}, xyzw={actual_q.tolist()}')
        return actual_xyz, actual_q.tolist()

    def path_values(self):
        return np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                          pose.pose.orientation.x, pose.pose.orientation.y,
                          pose.pose.orientation.z, pose.pose.orientation.w]
                         for pose in self.path.poses])

    def clear(self):
        revision = self.status.get('revision', -1) if self.status else -1
        self.commands['clear'].publish(Empty())
        self.wait(lambda: self.status is not None
                  and self.status['revision'] > revision
                  and self.status['state'] == 'waiting_start'
                  and self.xyz('start') is None and self.xyz('goal') is None
                  and self.path is not None and not self.path.poses,
                  4.0, 'Clear revokes path and both endpoints')

    def select(self, role, xyz, frame=FRAME):
        message = PointStamped()
        message.header.frame_id = frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.point.x, message.point.y, message.point.z = map(float, xyz)
        self.points[role].publish(message)

    def drag(self, role, axis, xyz, control=None):
        old_revision = self.status['revision']
        old_path_count = self.path_count
        # RViz fixed-frame handles intentionally use zero timestamp. Empty
        # client ID matches a freshly inserted marker's default client and
        # avoids the server's one-second cross-client arbitration window.
        orientation = self.orientation(role)
        message = self.feedback_message(role, control or 'move_' + axis, xyz)
        message.event_type = InteractiveMarkerFeedback.POSE_UPDATE
        self.feedback.publish(message)
        self.wait(lambda: self.matches(role, xyz)
                  and self.status['revision'] > old_revision
                  and self.path_count > old_path_count and not self.path.poses,
                  4.0, f'{role} independent {axis.upper()} drag')
        revision = self.status['revision']
        status_count = self.status_count
        message.event_type = InteractiveMarkerFeedback.MOUSE_UP
        self.feedback.publish(message)
        self.wait(lambda: self.status_count > status_count and self.matches(role, xyz),
                  2.0, f'{role} {axis.upper()} release')
        self.hold(0.25, lambda: self.matches(role, xyz)
                  and self.status['revision'] == revision and not self.path.poses
                  and self.orientation_matches(role, orientation),
                  'Mouse release must not duplicate edits, snap Z or auto-plan')
        self.assert_marker_pose(role, xyz, orientation)
        return revision

    def release_only(self, role, axis, xyz):
        """Exercise a final release pose newer than the last POSE_UPDATE."""
        old_revision = self.status['revision']
        old_path_count = self.path_count
        orientation = self.orientation(role)
        message = self.feedback_message(role, 'move_' + axis, xyz)
        message.event_type = InteractiveMarkerFeedback.MOUSE_UP
        self.feedback.publish(message)
        self.wait(lambda: self.matches(role, xyz)
                  and self.status['revision'] == old_revision + 1
                  and self.path_count > old_path_count and not self.path.poses,
                  4.0, 'Release-only final XYZ accepted and old path revoked')
        marker_xyz, marker_q = self.assert_marker_pose(role, xyz, orientation)
        self.hold(0.25, lambda: self.matches(role, xyz)
                  and self.status['revision'] == old_revision + 1 and not self.path.poses,
                  'Release-only edit must remain stable without implicit planning')
        self.check('release_only_final_xyz_matches_interactive_marker_service_pose',
                   selected_xyz=self.xyz(role), marker_xyz=marker_xyz, marker_orientation_xyzw=marker_q,
                   revision=self.status['revision'])

    def plan(self, expected_start, expected_goal):
        previous_status_count = self.status_count
        previous_path_count = self.path_count
        self.commands['plan'].publish(Empty())
        self.wait(lambda: self.status_count > previous_status_count
                  and self.status['state'] in ('planned', 'failed'),
                  30.0, 'Native PCT planning')
        if self.status['state'] != 'planned':
            raise AssertionError(f'Native PCT did not plan: {self.status}')
        self.wait(lambda: self.path_count > previous_path_count
                  and self.path is not None and len(self.path.poses) >= 2,
                  3.0, 'Planned Path delivery')
        points = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                           for p in self.path.poses])
        if self.path.header.frame_id != FRAME:
            raise AssertionError('Path frame mismatch')
        if not np.allclose(points[[0, -1]], [expected_start, expected_goal], atol=1e-8, rtol=0):
            raise AssertionError(f'Path must preserve selected XYZ: {points[[0, -1]]}')
        if not np.isfinite(points).all():
            raise AssertionError('Path contains nonfinite values')
        if 'upstream PCT' not in self.status['result'].get('algorithm', ''):
            raise AssertionError('Planner result does not identify upstream PCT')
        for index, pose in enumerate(self.path.poses):
            direction = points[min(index + 1, len(points) - 1), :2] - points[max(0, index - 1), :2]
            yaw = np.arctan2(direction[1], direction[0])
            expected = np.array([0., 0., np.sin(yaw / 2), np.cos(yaw / 2)])
            quaternion = pose.pose.orientation
            actual = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            if not (np.allclose(actual, expected, atol=1e-8, rtol=0)
                    or np.allclose(actual, -expected, atol=1e-8, rtol=0)):
                raise AssertionError('PCT Path poses must follow route tangents, not editor attitude constraints')
        self.check('path_orientations_follow_route_tangents_not_editor_pose_constraints',
                   path_points=len(points), orientation_semantics=self.status.get('orientation_semantics'))
        return points

    def get_markers(self):
        self.wait(self.marker_client.service_is_ready, 4.0, 'InteractiveMarker service')
        future = self.marker_client.call_async(GetInteractiveMarkers.Request())
        self.wait(future.done, 4.0, 'InteractiveMarker service reply')
        return {marker.name: marker for marker in future.result().markers}

    def audit_markers(self):
        markers = self.get_markers()
        if set(markers) != {'start', 'goal'}:
            raise AssertionError(f'Expected START/GOAL handles, found {set(markers)}')
        details = {}
        for role, marker in markers.items():
            if marker.header.frame_id != FRAME:
                raise AssertionError('InteractiveMarker frame mismatch')
            controls = {control.name: control for control in marker.controls}
            center = controls.get('move_rotate_3d')
            if (center is None or center.interaction_mode != InteractiveMarkerControl.MOVE_ROTATE_3D
                    or center.orientation_mode != InteractiveMarkerControl.VIEW_FACING
                    or not center.independent_marker_orientation):
                raise AssertionError(f'{role}: center must expose the MoveIt-style view-facing 3D handle')
            details[role] = {}
            for i, axis in enumerate('xyz'):
                details[role][axis] = {}
                for prefix, mode in (('move_', InteractiveMarkerControl.MOVE_AXIS),
                                     ('rotate_', InteractiveMarkerControl.ROTATE_AXIS)):
                    control = controls.get(prefix + axis)
                    if (control is None or control.interaction_mode != mode
                            or control.orientation_mode != InteractiveMarkerControl.FIXED):
                        raise AssertionError(f'{role} {prefix}{axis}: expected fixed-frame control')
                    q = control.orientation
                    norm = np.linalg.norm([q.x, q.y, q.z, q.w])
                    rotated_x = np.array([1 - 2 * (q.y * q.y + q.z * q.z),
                                          2 * (q.x * q.y + q.w * q.z),
                                          2 * (q.x * q.z - q.w * q.y)])
                    if not np.isclose(norm, 1.0) or not np.allclose(rotated_x, np.eye(3)[i], atol=1e-8):
                        raise AssertionError(f'{role} {prefix}{axis}: wrong map axis')
                    # Empty geometry intentionally asks Humble's standard
                    # autoComplete path to generate arrows / rotation rings.
                    if control.markers:
                        raise AssertionError(f'{role} {prefix}{axis}: custom geometry bypasses standard handles')
                    details[role][axis][prefix[:-1]] = {
                        'axis': rotated_x.tolist(), 'geometry': 'rviz_native_auto_complete'}
            self.assert_marker_pose(role, self.xyz(role), self.orientation(role))
        self.check('start_goal_native_moveit_style_center_xyz_translation_and_rotation_controls', handles=details)

    def audit_rviz(self):
        """Require the real RViz node, not this verifier's own marker client."""
        node_name, namespace = 'pct_preview_rviz', '/'
        expected = {
            'subscriptions': (MARKERS + '/update', 'visualization_msgs/msg/InteractiveMarkerUpdate'),
            'publishers': (MARKERS + '/feedback', 'visualization_msgs/msg/InteractiveMarkerFeedback'),
            'clients': (MARKERS + '/get_interactive_markers', 'visualization_msgs/srv/GetInteractiveMarkers'),
        }
        observed = {}

        def ready():
            if (node_name, namespace) not in self.get_node_names_and_namespaces():
                return False
            try:
                observed['subscriptions'] = self.get_subscriber_names_and_types_by_node(node_name, namespace)
                observed['publishers'] = self.get_publisher_names_and_types_by_node(node_name, namespace)
                observed['clients'] = self.get_client_names_and_types_by_node(node_name, namespace)
            except RuntimeError:
                return False  # A node may disappear between graph queries.
            return all(any(topic == want_topic and want_type in types
                           for topic, types in observed[kind])
                       for kind, (want_topic, want_type) in expected.items())

        self.wait(ready, 8.0, 'Real RViz subscribes to marker updates and has feedback publisher/init client')
        links = []
        for topic, publisher_node, subscriber_node in (
                (MARKERS + '/update', 'pct_preview_server', node_name),
                (MARKERS + '/feedback', node_name, 'pct_preview_server')):
            endpoints = {}

            def endpoints_ready():
                endpoints['publishers'] = [item for item in self.get_publishers_info_by_topic(topic)
                                          if item.node_name == publisher_node and item.node_namespace == namespace]
                endpoints['subscriptions'] = [item for item in self.get_subscriptions_info_by_topic(topic)
                                             if item.node_name == subscriber_node and item.node_namespace == namespace]
                return bool(endpoints['publishers'] and endpoints['subscriptions'])

            self.wait(endpoints_ready, 4.0, f'RViz/server topic endpoints: {topic}')
            compatible = []
            for publisher in endpoints['publishers']:
                for subscriber in endpoints['subscriptions']:
                    result, reason = qos_check_compatible(publisher.qos_profile, subscriber.qos_profile)
                    if result == QoSCompatibility.OK and publisher.topic_type == subscriber.topic_type:
                        compatible.append({'publisher': publisher_node, 'subscriber': subscriber_node,
                                           'type': publisher.topic_type, 'compatibility': result.name,
                                           'reason': reason,
                                           'publisher_reliability': publisher.qos_profile.reliability.name,
                                           'subscriber_reliability': subscriber.qos_profile.reliability.name,
                                           'publisher_durability': publisher.qos_profile.durability.name,
                                           'subscriber_durability': subscriber.qos_profile.durability.name})
            if not compatible:
                raise AssertionError(f'No confirmed QoS-compatible RViz/server endpoints for {topic}')
            links.append({'topic': topic, 'compatible_pairs': compatible})
        self.check('actual_rviz_marker_update_feedback_and_init_client_connected',
                   node=node_name, graph=observed, links=links,
                   limitation='Graph and QoS proof only; not a rendered-screen or physical mouse test')

    def activate_without_picking(self):
        """Create both spatial handles without first publishing PointStamped."""
        for role in ('start', 'goal'):
            old_revision = self.status['revision']
            self.commands['activate_' + role].publish(Empty())
            self.wait(lambda: self.xyz(role) is not None
                      and self.status['revision'] == old_revision + 1,
                      4.0, f'{role} activation creates a handle without point picking')
            marker = self.get_markers().get(role)
            if marker is None:
                raise AssertionError(f'{role} activation updated status but did not create a handle')
            pose = marker.pose.position
            if not np.allclose([pose.x, pose.y, pose.z], self.xyz(role), atol=1e-8, rtol=0):
                raise AssertionError(f'{role} activation marker pose differs from selected position')
            self.hold(0.3, lambda: self.path is not None and not self.path.poses
                      and self.status['state'] not in ('planning', 'planned'),
                      'Activating a handle must not auto-plan')
        if np.linalg.norm(np.array(self.xyz('start')) - self.xyz('goal')) < 0.05:
            raise AssertionError('Initial START and GOAL handles overlap')
        self.check('empty_activation_creates_two_distinct_spatial_handles_without_picking',
                   start_xyz=self.xyz('start'), goal_xyz=self.xyz('goal'))
        self.audit_markers()

        edited = np.array(self.xyz('goal'))
        edited[2] += 0.37  # Deliberately floating: activation must not reset/snap it.
        self.drag('goal', 'z', edited)
        self.audit_reactivation()

    def audit_reactivation(self):
        revision = self.status['revision']
        points = {role: self.xyz(role)[:] for role in ('start', 'goal')}
        orientations = {role: self.orientation(role) for role in ('start', 'goal')}
        orientation_revision = self.status['orientation_revision']
        old_state = self.status['state']
        old_result = self.status.get('result')
        old_path_count = self.path_count
        for role in ('start', 'goal'):
            status_count = self.status_count
            self.commands['activate_' + role].publish(Empty())
            self.wait(lambda: self.status_count > status_count, 3.0, f'{role} reactivation acknowledgement')
            self.hold(0.4, lambda: self.status['revision'] == revision
                      and all(self.matches(name, value) for name, value in points.items())
                      and all(self.orientation_matches(name, value) for name, value in orientations.items())
                      and self.status['orientation_revision'] == orientation_revision
                      and self.status['state'] == old_state and self.status.get('result') == old_result
                      and self.path_count == old_path_count,
                      'Reactivation must preserve edited XYZ, revision, state and existing path')
            self.assert_marker_pose(role, points[role], orientations[role])
        self.check('repeated_activation_preserves_user_xyz_revision_and_does_not_plan',
                   revision=revision, orientation_revision=orientation_revision,
                   orientations_xyzw={role: value.tolist() for role, value in orientations.items()},
                   state=old_state, endpoints=points)

    def audit_editor_rotations(self):
        if self.status.get('orientation_semantics') != 'editor_preview_only_xyz_planner':
            raise AssertionError('Editor rotations must explicitly remain outside the XYZ planner contract')
        revision, path_count = self.status['revision'], self.path_count
        points = {role: self.xyz(role)[:] for role in ('start', 'goal')}
        path_values = self.path_values()
        result = self.status['result']

        def route_unchanged():
            return (self.status['state'] == 'planned' and self.status['revision'] == revision
                    and self.path_count == path_count and self.status['result'] == result
                    and all(self.matches(role, xyz) for role, xyz in points.items())
                    and np.array_equal(self.path_values(), path_values))

        details = []
        for role, axes in (('start', 'x'), ('goal', 'xyz')):
            for axis in axes:
                index = 'xyz'.index(axis)
                angle = 0.4 + 0.1 * index
                quaternion = np.zeros(4)
                quaternion[index], quaternion[3] = np.sin(angle / 2), np.cos(angle / 2)
                orientation_revision = self.status['orientation_revision']
                # Accept finite non-unit input only through normalization.
                message = self.feedback_message(role, 'rotate_' + axis, points[role], quaternion * 2)
                message.event_type = InteractiveMarkerFeedback.POSE_UPDATE
                self.feedback.publish(message)
                self.wait(lambda: self.orientation_matches(role, quaternion)
                          and self.status['orientation_revision'] == orientation_revision + 1,
                          4.0, f'{role} rotate_{axis} updates only normalized editor orientation')
                if not route_unchanged():
                    raise AssertionError('Editor rotation must not change selected XYZ, planned route or route revision')
                # A newer final release quaternion must survive rather than
                # being forced back to identity or the preceding drag value.
                angle += 0.07
                quaternion[index], quaternion[3] = np.sin(angle / 2), np.cos(angle / 2)
                message = self.feedback_message(role, 'rotate_' + axis, points[role], quaternion)
                message.event_type = InteractiveMarkerFeedback.MOUSE_UP
                self.feedback.publish(message)
                self.wait(lambda: self.orientation_matches(role, quaternion)
                          and self.status['orientation_revision'] == orientation_revision + 2,
                          4.0, f'{role} rotate_{axis} release preserves final quaternion')
                self.assert_marker_pose(role, points[role], quaternion)
                self.hold(0.25, route_unchanged, 'Rotation release must preserve the original PCT path')
                details.append({'role': role, 'control': 'rotate_' + axis,
                                'orientation_xyzw': quaternion.tolist(),
                                'orientation_revision': self.status['orientation_revision']})
        self.check('rotation_controls_normalize_and_preserve_release_quaternion_without_replanning',
                   controls=details, route_revision=revision)
        self.audit_reactivation()
        for role in ('start', 'goal'):
            expected = self.orientation(role)
            status_count = self.status_count
            self.select(role, points[role])
            self.wait(lambda: self.status_count > status_count, 3.0, 'Unchanged numeric point acknowledgement')
            self.hold(0.25, lambda: route_unchanged() and self.orientation_matches(role, expected),
                      'Numeric XYZ input must preserve existing orientation')
            self.assert_marker_pose(role, points[role], expected)
        self.check('numeric_xyz_reinsertion_keeps_both_editor_orientations_and_existing_route')

    def audit_orientation_reset(self):
        revision, path_count = self.status['revision'], self.path_count
        points = {role: self.xyz(role)[:] for role in ('start', 'goal')}
        previous_path, previous_result = self.path_values(), self.status['result']
        for role in ('start', 'goal'):
            if self.orientation_matches(role, [0., 0., 0., 1.]):
                raise AssertionError('Reset regression requires a non-identity editor orientation')
            orientation_revision = self.status['orientation_revision']
            self.commands['reset_' + role + '_orientation'].publish(Empty())
            self.wait(lambda: self.orientation_matches(role, [0., 0., 0., 1.])
                      and self.status['orientation_revision'] == orientation_revision + 1,
                      4.0, f'Explicit {role} orientation reset')
            self.hold(0.25, lambda: self.status['state'] == 'planned'
                      and self.status['revision'] == revision and self.path_count == path_count
                      and self.status['result'] == previous_result
                      and all(self.matches(name, xyz) for name, xyz in points.items())
                      and np.array_equal(self.path_values(), previous_path),
                      'Explicit orientation reset must not change XYZ or the valid global path')
            self.assert_marker_pose(role, points[role], [0., 0., 0., 1.])
        self.check('explicit_start_goal_orientation_reset_changes_only_editor_attitude',
                   route_revision=revision, orientation_revision=self.status['orientation_revision'])

    def audit_publishers(self):
        publishers = self.get_publisher_names_and_types_by_node('pct_preview_server', '/')
        if not publishers:
            raise AssertionError('Could not inspect preview publisher graph')
        allowed_roots = (PREFIX + '/', MARKERS + '/')
        forbidden = []
        for topic, types in publishers:
            if ('cmd_vel' in topic or topic.startswith('/d1max/pct_scan/')
                    or any(kind in ('geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped',
                                    'd1max_planning_interfaces/msg/ReferencePath') for kind in types)
                    or not (topic.startswith(allowed_roots)
                            or topic in ('/rosout', '/parameter_events'))):
                forbidden.append([topic, types])
        if forbidden:
            raise AssertionError(f'Preview unexpectedly publishes robot/task interfaces: {forbidden}')
        if self.status.get('motion_enabled') is not False or self.status.get('robot_connected') is not False:
            raise AssertionError('Preview must explicitly report no motion and no robot connection')
        self.check('preview_has_no_velocity_navigation_task_or_sdk_publishers', publishers=publishers)

    def run(self, grid, start_xy, goal_xy, hold_seconds, require_rviz=False):
        self.wait(lambda: self.status is not None and self.status.get('ready'),
                  35.0, 'Preview worker readiness')
        self.wait(lambda: all(pub.get_subscription_count() > 0
                             for pub in [*self.points.values(), *self.commands.values(), self.feedback]),
                  5.0, 'Preview command discovery')
        self.audit_publishers()
        if require_rviz:
            self.audit_rviz()
        self.clear()
        self.check('clear_waits_for_start_with_empty_path')
        self.activate_without_picking()
        self.clear()

        start = np.r_[start_xy, grid.validate_point(start_xy)]
        goal = np.r_[goal_xy, grid.validate_point(goal_xy)]
        self.report['requested_start_xyz'] = start.tolist()
        self.report['requested_goal_xyz'] = goal.tolist()
        self.select('start', start)
        self.wait(lambda: self.matches('start', start), 4.0, 'Start selection')
        self.select('goal', goal)
        self.wait(lambda: self.matches('goal', goal), 4.0, 'Goal selection')
        self.hold(0.8, lambda: self.path is not None and not self.path.poses
                  and self.status['state'] not in ('planning', 'planned'),
                  'Selecting points must not implicitly plan')
        self.check('selecting_start_goal_does_not_plan')
        self.audit_markers()

        points = self.plan(start, goal)
        grid.validate_path(points)
        self.report['initial_path'] = points.tolist()
        self.check('explicit_plan_native_pct_preserves_exact_xyz_endpoints',
                   result=self.status['result'], path_points=len(points))
        self.audit_reactivation()

        late_paths = []
        late_subscription = self.create_subscription(Path, PREFIX + '/path', late_paths.append, self.latched)
        self.wait(lambda: any(len(path.poses) == len(points) for path in late_paths),
                  4.0, 'Transient-local late subscriber receives completed path')
        before_count = self.path_count
        self.hold(hold_seconds, lambda: self.status['state'] == 'planned'
                  and self.path is not None and len(self.path.poses) == len(points),
                  'Completed path must persist without auto-motion or timer expiry')
        self.destroy_subscription(late_subscription)
        self.check('completed_path_persists_and_is_delivered_to_late_subscriber',
                   hold_seconds=hold_seconds, path_republications=self.path_count - before_count)

        self.audit_editor_rotations()
        orientation = self.orientation('goal')
        center_xyz = goal + np.array([0.01, 0.02, 0.02])
        self.drag('goal', 'center', center_xyz, control='move_rotate_3d')
        self.check('native_center_handle_translates_xyz_and_preserves_editor_rotation',
                   goal_xyz=self.xyz('goal'), orientation_xyzw=self.orientation('goal').tolist())
        # Numeric XYZ changes are not pose resets. Restore the known route
        # endpoint without implicitly clearing the newly edited orientation.
        self.select('goal', goal)
        self.wait(lambda: self.matches('goal', goal), 4.0, 'Numeric XYZ restores selected goal')
        self.hold(0.25, lambda: self.orientation_matches('goal', orientation) and not self.path.poses,
                  'Numeric XYZ changes must preserve editor rotation and require explicit Plan')
        self.assert_marker_pose('goal', goal, orientation)
        self.check('numeric_xyz_edit_keeps_editor_rotation_after_center_translation')

        edited = goal.copy()
        for index, axis in enumerate('xyz'):
            before = edited.copy()
            edited[index] += 0.02
            revision = self.drag('goal', axis, edited)
            unchanged = [j for j in range(3) if j != index]
            if not np.array_equal(np.asarray(self.xyz('goal'))[unchanged], before[unchanged]):
                raise AssertionError(f'{axis.upper()} drag altered other axes')
            self.check('independent_' + axis + '_drag_and_release_revokes_old_path',
                       goal_xyz=edited.tolist(), revision=revision)

        edited[2] += 0.01
        self.release_only('goal', 'z', edited)

        # Return XY to a known native-PCT route without changing selected Z.
        edited[0] = goal[0]
        self.drag('goal', 'x', edited)
        edited[1] = goal[1]
        self.drag('goal', 'y', edited)
        edited[2] = goal[2] + 0.5
        self.drag('goal', 'z', edited)
        old_count = self.status_count
        self.commands['plan'].publish(Empty())
        self.wait(lambda: self.status_count > old_count and self.status['state'] == 'failed',
                  4.0, 'Off-ground selected Z rejects Plan')
        if not self.matches('goal', edited) or self.path.poses or 'Z differs from ground' not in self.status['reason']:
            raise AssertionError('Off-ground point must remain selected and reject planning, not silently snap')
        self.check('off_ground_z_is_retained_but_plan_is_rejected', reason=self.status['reason'])

        revision = self.status['revision']
        orientation = self.orientation('goal')
        orientation_revision = self.status['orientation_revision']
        self.commands['snap_goal'].publish(Empty())
        self.wait(lambda: self.matches('goal', goal) and self.status['revision'] > revision,
                  4.0, 'Explicit Ground restores measured Z')
        self.hold(0.4, lambda: not self.path.poses and self.status['state'] != 'planned'
                  and self.orientation_matches('goal', orientation)
                  and self.status['orientation_revision'] == orientation_revision,
                  'Ground button must not implicitly plan')
        self.assert_marker_pose('goal', goal, orientation)
        self.check('explicit_ground_restores_measured_z_without_auto_plan', goal_xyz=self.xyz('goal'))
        points = self.plan(start, goal)
        self.report['replanned_path'] = points.tolist()
        self.check('explicit_replan_after_ground_snap_succeeds', result=self.status['result'])

        revision = self.status['revision']
        path_count = self.path_count
        status_count = self.status_count
        previous_result = self.status['result']
        self.commands['snap_goal'].publish(Empty())
        self.wait(lambda: self.status_count > status_count, 3.0, 'Already-grounded Ground acknowledgement')
        self.hold(0.6, lambda: self.status['state'] == 'planned'
                  and self.status['revision'] == revision and self.matches('goal', goal)
                  and self.orientation_matches('goal', orientation)
                  and self.status['orientation_revision'] == orientation_revision
                  and self.path_count == path_count and len(self.path.poses) == len(points)
                  and self.status['result'] == previous_result,
                  'Ground on an already-grounded planned point must retain state, revision and path')
        retained_points = np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                                    for pose in self.path.poses])
        if not np.array_equal(retained_points, points):
            raise AssertionError('Idempotent Ground changed the completed path')
        self.check('already_grounded_snap_preserves_planned_state_revision_and_path',
                   revision=revision, path_points=len(points))
        self.audit_orientation_reset()

        revision = self.status['revision']
        self.select('goal', goal + np.array([0.5, 0.5, 0.5]), frame='invalid_test_frame')
        self.wait(lambda: self.status['state'] == 'input_rejected', 4.0, 'Wrong-frame selection rejected')
        if self.status['revision'] != revision or not self.matches('goal', goal):
            raise AssertionError('Rejected frame must not change endpoint or revision')
        self.check('invalid_frame_rejected_without_coordinate_relabeling', reason=self.status['reason'])

        self.clear()
        self.hold(0.5, lambda: self.status['state'] == 'waiting_start'
                  and self.xyz('start') is None and self.xyz('goal') is None and not self.path.poses,
                  'Clear must revoke completed result persistently')
        self.check('clear_revokes_completed_result_and_returns_to_waiting_start')
        self.audit_publishers()
        if require_rviz:
            self.audit_rviz()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--grid', required=True, type=FilePath)
    parser.add_argument('--output', required=True, type=FilePath)
    parser.add_argument('--start', nargs=2, type=float, default=[-4.100001525878895, 6.9])
    parser.add_argument('--goal', nargs=2, type=float, default=[-12.500001525878895, 23.1])
    parser.add_argument('--hold-seconds', type=float, default=2.0)
    parser.add_argument('--require-rviz', action='store_true',
                        help='Require the actual RViz marker update/feedback/service graph and compatible QoS')
    options = parser.parse_args(args)
    if not np.isfinite(options.hold_seconds) or not 0.1 <= options.hold_seconds <= 15.0:
        parser.error('--hold-seconds must be in [0.1, 15]')
    report = {'passed': False, 'checks': [], 'mode': 'GLOBAL_PATH_PREVIEW_ONLY',
              'scope': 'ROS planning/XYZ interactive-marker protocol; not physical navigation or GUI mouse acceptance',
              'require_rviz': options.require_rviz,
              'grid': str(options.grid.resolve()), 'frame_id': FRAME,
              'robot_connected': False, 'motion_enabled': False}
    node = None
    rclpy.init(args=[])
    try:
        grid = MeasuredGrid(options.grid)
        report['source_grid_sha256'] = grid.sha256
        node = PreviewVerifier(report)
        node.run(grid, options.start, options.goal, options.hold_seconds, options.require_rviz)
        report['passed'] = True
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        report['traceback'] = traceback.format_exc()
        print(report['error'], flush=True)
    finally:
        if node is not None:
            try:
                node.clear()
                report['cleanup'] = 'cleared; waiting_start; no result'
            except Exception as exc:
                report['cleanup_error'] = str(exc)
                report['passed'] = False
            report['events'] = node.events
            report['final_status'] = node.status
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'passed': report['passed'], 'report': str(options.output)}, allow_nan=False), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
