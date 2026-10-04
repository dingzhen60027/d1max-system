"""Isolated cross-floor stage editor around real BT / ComputeRoute / native PCT.

Only the stationary start fixture is simulated, explicitly at the selected
ground plus a configurable body height. No sensor replay or robot navigation
success is claimed. The production source-route builder remains authoritative.
"""
import json
import os
import time

import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from d1max_navigation_bt_interfaces.action import Navigate
from d1max_navigation_bt_interfaces.msg import RouteSnapshot
from d1max_planning_interfaces.msg import NavigationState
from d1max_pct_planner.preview_server import PctPreviewServer
from .source_route_ros import from_message

PREFIX = '/d1max/live_planning/'


class GlobalStageEditor(PctPreviewServer):
    def __init__(self):
        if (os.environ.get('D1MAX_NAV_ISOLATED') != '1'
                or os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp'
                or os.environ.get('ROS_DOMAIN_ID') == '24'):
            raise ValueError('global_stage_requires_private_offline_zenoh_domain')
        self.goal_handle = None
        self.sending = False
        self.retiring = False
        self.task_id = ''
        self.snapshots = {}
        self.bt_status = {}
        self.worker_status = {}
        self.fixture_key = None
        self.fixture_epoch = 0
        self.fixture_body = None
        self.stage_result = None
        super().__init__()
        if not self.settings['external_planner'] or not self.crossfloor_settings:
            raise ValueError('global_stage_requires_external_bt_and_crossfloor_map')
        self.session_id = self.declare_parameter('session_id', '').value
        self.map_version_id = self.declare_parameter('map_version_id', '').value
        manifest = self.declare_parameter('planning_manifest', '').value
        self.body_height = self.declare_parameter('fixture_body_height', .55).value
        from .pointcloud_helpers.ground_path_bridge import GroundPathBridge
        self.bridge = GroundPathBridge.from_artifacts(manifest)
        self.navigation_pub = self.create_publisher(NavigationState,
            '/d1max/localization/navigation/state', 5)
        self.client = ActionClient(self, Navigate, PREFIX+'bt/navigate')
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(RouteSnapshot, PREFIX+'bt/route_snapshot', self.on_snapshot, durable)
        self.create_subscription(String, PREFIX+'bt/status', self.on_bt_status, durable)
        self.create_subscription(String, PREFIX+'global_status', self.on_worker_status, 5)
        self.fixture_timer = self.create_timer(.02, self.publish_fixture)
        # Explicit default cross-floor draft, not a previously restored path.
        for role in ('start', 'goal'):
            anchor = self.crossfloor_settings['anchors'][role]
            self.selection.set_active_layer(int(anchor['layer_id']))
            self.selection.set_point(role, anchor['xyz'])
            self.update_endpoint(role)
        self.selection.set_active_layer(-1)
        self.state, self.reason = 'initializing', '行为树与原生 PCT 预热中'

    def publish_fixture(self):
        try:
            ground = self.selection.validate('start')
            floor = self.selection.floor_for_height(float(ground[2]))
            if floor is None:
                raise ValueError('Start fixture must be on an explicit building floor')
            key = (tuple(ground), floor)
            if key != self.fixture_key:
                source = self.bridge.to_localization_ground(ground[None, :], [floor]).xyz[0]
                self.fixture_body = source + [0., 0., self.body_height]
                self.fixture_key = key
                self.fixture_epoch += 1
            stamp = self.get_clock().now().to_msg()
            state = NavigationState(schema_version=2, session_id=self.session_id,
                map_version_id=self.map_version_id, localization_epoch=self.fixture_epoch,
                localization_seed_id=f'offline_start_{self.fixture_epoch}', usable=True,
                source_stamp=stamp, posterior_stamp=stamp, imu_stamp=stamp,
                reason='OFFLINE_STATIONARY_FIXTURE_NOT_ROBOT_LOCALIZATION')
            for odom, frame in ((state.local_odometry, 'd1max_loc_odom'),
                                (state.global_odometry, 'd1max_loc_map')):
                odom.header.stamp, odom.header.frame_id = stamp, frame
                odom.child_frame_id = 'd1max_loc_base_link'
                odom.pose.pose.orientation.w = 1.
            p = state.global_odometry.pose.pose.position
            p.x, p.y, p.z = map(float, self.fixture_body)
            self.navigation_pub.publish(state)
        except ValueError:
            return  # Invalid draft must not manufacture localization readiness.

    def on_worker_status(self, message):
        try:
            value = json.loads(message.data)
            if value.get('session_id') == self.session_id:
                self.worker_status = value
        except (ValueError, TypeError):
            pass

    def on_bt_status(self, message):
        try:
            value = json.loads(message.data)
            if value.get('session_id') == self.session_id:
                self.bt_status = value
        except (ValueError, TypeError):
            pass

    def invalidate_path(self):
        super().invalidate_path()
        self.stage_result = None
        if self.sending:
            self.retiring = True
        if self.goal_handle is not None:
            self.retiring = True
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(lambda _: None)

    def on_plan(self, _message):
        if self.pending is not None or self.retiring or self.sending or self.stage_result is not None:
            return
        try:
            if not self.ready:
                raise ValueError('等待真实行为树和全局规划器就绪')
            request = self.selection.request()
            goal = Navigate.Goal(schema_version=2, goal_kind='3d')
            goal.goal.header = self.header()
            goal.goal.pose.position.x, goal.goal.pose.position.y, goal.goal.pose.position.z = request['goal_xyz']
            goal.goal.pose.orientation.w = 1.
            goal.goal_yaw_tolerance_rad = .1745329252
            self.pending = {**request, 'requested_at': time.monotonic()}
            self.path_data, self.last_result, self.stage_result, self.task_id = None, None, None, ''
            self.publish_path(None)
            self.state, self.reason = 'planning', '行为树 → ComputeRoute → 原生跨楼层 PCT'
            self.sending = True
            future = self.client.send_goal_async(goal, feedback_callback=lambda event,
                generation=request['generation']: self.on_feedback_action(event, generation))
            future.add_done_callback(self.on_action_accepted)
        except ValueError as exc:
            self.state, self.reason = 'failed', str(exc)
        self.publish_status()

    def on_feedback_action(self, feedback, generation):
        if self.pending is None or self.pending['generation'] != generation:
            return
        self.task_id = feedback.feedback.task_id
        self.reason = feedback.feedback.reason or feedback.feedback.active_node

    def on_action_accepted(self, future):
        self.sending = False
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.pending = None
            self.state, self.reason = 'failed', '行为树拒绝本次规划请求'
            self.goal_handle = None
            self.retiring = False
            return
        self.goal_handle.get_result_async().add_done_callback(self.on_action_result)
        if self.pending is None:
            self.retiring = True
            self.goal_handle.cancel_goal_async()

    def on_snapshot(self, message):
        if message.session_id != self.session_id:
            return
        self.snapshots[message.task_id] = message
        if len(self.snapshots) > 8:
            self.snapshots.pop(next(iter(self.snapshots)))

    def on_action_result(self, future):
        self.goal_handle, self.retiring = None, False
        if self.pending is None:
            return
        request, self.pending = self.pending, None
        result = future.result().result
        try:
            if not result.success or request['generation'] != self.selection.revision:
                raise ValueError(result.reason or 'Draft changed during calculation')
            wire = self.snapshots.get(self.task_id)
            if wire is None:
                # Durable snapshot and Action result are different transports;
                # bounded deferred pairing handles their arrival order.
                self.stage_result = (request, result, time.monotonic())
                return
            self.complete_stage(request, wire)
        except ValueError as exc:
            self.state, self.reason = 'failed', str(exc)
            self.last_error = {'error_code': 'bt_global_stage_failed', 'details': {}}
        self.publish_status()

    def complete_stage(self, request, wire):
        snapshot = from_message(wire)
        value = snapshot.payload()
        if (request['generation'] != self.selection.revision or wire.map_version_id != self.map_version_id
                or not value['preview_ready']):
            raise ValueError('Obsolete draft, map or non-preview-ready route')
        display = self.bridge.to_planning_ground(np.asarray(value['xyz']), value['point_floor_ids']).xyz
        self.grid.validate_path(display, value['layer_ids'])
        if not np.allclose(display[[0, -1]], [request['start_xyz'], request['goal_xyz']], atol=.08, rtol=0):
            raise ValueError('Committed route does not match selected ground endpoints')
        self.path_data = display
        self.last_result = dict(stage='global_planning_only', route_id=wire.route_id,
            route_hash=wire.route_hash, task_id=wire.task_id,
            length_m=float(np.linalg.norm(np.diff(display, axis=0), axis=1).sum()),
            elapsed_s=time.monotonic()-request['requested_at'], layer_ids=value['layer_ids'],
            floors=list(dict.fromkeys(value['point_floor_ids'])), segments=value['segments'],
            source_frame=wire.frame_id, display_frame=self.frame,
            display_projection='labelled_ground_only_not_tf', motion_enabled=False,
            execution_eligible=wire.execution_eligible, eligibility_reason=wire.eligibility_reason,
            simulated_input='stationary_start_fixture_only', navigation_arrival_tested=False)
        self.state, self.reason = 'planned', '行为树全局规划阶段完成'
        self.publish_path(display)
        self.publish_tomogram()
        if self.output:
            artifact = dict(result=self.last_result, source_snapshot=value,
                display_path=display.tolist(), request=request, bt=self.bt_status)
            (self.output/f'bt_route_{request["generation"]:06d}.json').write_text(
                json.dumps(artifact, indent=2, allow_nan=False)+'\n')

    def tick(self):
        if not hasattr(self, 'client'):
            return
        warm = self.worker_status.get('native_warmup', {})
        self.ready = (self.client.server_is_ready() and self.bt_status.get('lifecycle_active') is True
                      and warm.get('phase') == 'ready' and not self.retiring)
        if self.ready and self.state == 'initializing':
            self.state, self.reason = 'editing', '选择楼层与起终点，点击规划'
        if self.stage_result is not None:
            request, result, began = self.stage_result
            wire = self.snapshots.get(self.task_id)
            if wire is not None:
                self.stage_result = None
                try:
                    self.complete_stage(request, wire)
                except ValueError as exc:
                    self.state, self.reason = 'failed', str(exc)
            elif time.monotonic()-began > 2.:
                self.stage_result = None
                self.state, self.reason = 'failed', 'Missing matching committed route snapshot'
        if self.pending and time.monotonic()-self.pending['requested_at'] > 15.:
            self.invalidate_path()
            self.state, self.reason = 'failed', '全局阶段超时，正在退役旧计算'
        self.publish_status()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalStageEditor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except RCLError:
            if rclpy.ok():
                raise


if __name__ == '__main__':
    main()
