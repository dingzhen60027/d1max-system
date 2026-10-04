"""Isolated RViz XYZ workbench. No task/reference/velocity/SDK interfaces."""
from array import array
import json
import math
import multiprocessing
from pathlib import Path as FilePath
import queue
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Empty, Header, Int32, String
from visualization_msgs.msg import InteractiveMarkerFeedback, Marker, MarkerArray

from .measured_grid import MeasuredGrid
from .preview_core import PreviewSelection, finite_xyz, unit_quaternion
from .preview_markers import pose_marker
from .route_engine import worker_main
from .tomogram_display import surface_clouds
from .planner_core import validate_native_parameters


PREFIX = '/d1max/pct_preview'


class PctPreviewServer(Node):
    def __init__(self):
        super().__init__('pct_preview_server')
        defaults = {
            'map_backend': 'legacy_grid', 'tomogram_path': '',
            'minimum_headroom_m': 0.55, 'unknown_ceiling_policy': 'allow_unobserved',
            'selection_mode': 'ground', 'placement_anchor_z': -0.6375,
            'max_follow_height_change_m': 0.3,
            'planning_grid': '', 'map_pcd': '',
            'vendor_root': '',
            'planning_frame': 'd1max_loc_map', 'output_directory': '',
            'max_selection_height_error_m': 0.08, 'selection_max_age_s': 5.0,
            'planning_timeout_s': 20.0, 'handle_scale_m': 1.2,
            'place_endpoints_on_start': True,
            'placement_anchor_xy': [-4.100001525878895, 6.9],
            'map_display_max_points': 600000,
            'cost_margin_m': 0.6, 'minimum_clearance_m': 0.2,
            'optimization_guard_cells': 1, 'max_heading_rate': 10.0,
            # Native cost units, independent of metre-based map inflation.
            'astar_cost_weight': 0.2, 'optimizer_cost_margin': 15.0,
            'path_refinement': 'none', 'refinement_corner_cut_m': 1.5,
            'max_ground_step_m': 0.15,
            'restore_route_file': '', 'restore_tomogram_path': '',
            'crossfloor_route_config': '',
            'external_planner': False,
        }
        self.settings = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        validate_native_parameters(self.settings['astar_cost_weight'], self.settings['optimizer_cost_margin'])
        initial = self.declare_parameter('initial_start_xyz', [],
            ParameterDescriptor(dynamic_typing=True)).value
        self.frame = self.settings['planning_frame']
        if not self.frame:
            raise ValueError('An explicit map frame is required')
        for name in ('selection_max_age_s', 'planning_timeout_s', 'handle_scale_m'):
            if not math.isfinite(self.settings[name]) or self.settings[name] <= 0:
                raise ValueError(f'Invalid {name}')
        self.layered = self.settings['map_backend'] == 'official_tomogram'
        if self.layered:
            from .tomogram_map import TomogramMap
            from .tomogram_selection import TomogramSelection
            from .tomogram_route import worker_main as layered_worker
            grid = TomogramMap(self.settings['tomogram_path'],
                minimum_headroom_m=self.settings['minimum_headroom_m'],
                unknown_ceiling_policy=self.settings['unknown_ceiling_policy'],
                max_ground_step_m=self.settings['max_ground_step_m'])
            grid.verify_source(self.settings['map_pcd'], self.frame)
            self.selection = TomogramSelection(grid, self.settings['max_selection_height_error_m'],
                self.settings['selection_mode'], self.settings['placement_anchor_z'],
                self.settings['max_follow_height_change_m'])
            worker_target = layered_worker
            self.layer_metadata = grid.available_layers()
        elif self.settings['map_backend'] == 'legacy_grid':
            grid = MeasuredGrid(self.settings['planning_grid'], self.settings['cost_margin_m'],
                                self.settings['minimum_clearance_m'], self.settings['optimization_guard_cells'])
            self.selection = PreviewSelection(grid, self.settings['max_selection_height_error_m'])
            worker_target = worker_main
            self.layer_metadata = []
        else:
            raise ValueError('Unknown map_backend; no silent fallback to legacy grid')
        self.grid = grid
        self.crossfloor_settings = None
        if self.settings['crossfloor_route_config']:
            from .crossfloor_preview import load_config
            _, self.crossfloor_settings = load_config(self.settings['crossfloor_route_config'], grid)
            self.selection.floor_ranges = {'floor1': self.crossfloor_settings['floor_z_ranges']['lower'],
                                           'floor2': self.crossfloor_settings['floor_z_ranges']['upper']}
        self.display_layers = None
        self.state, self.reason = 'initializing', 'Loading PCT'
        self.ready = False
        self.native_runtime = None
        self.native_parameters = None
        self.pending = None
        self.last_result = None
        self.last_error = None
        self.path_data = None
        self.received_at = time.monotonic()
        self.output = FilePath(self.settings['output_directory']) if self.settings['output_directory'] else None
        if self.output:
            self.output.mkdir(parents=True, exist_ok=True)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, PREFIX + '/path', latched)
        self.status_pub = self.create_publisher(String, PREFIX + '/status', latched)
        self.marker_pub = self.create_publisher(MarkerArray, PREFIX + '/markers', latched)
        self.cloud_pub = self.create_publisher(PointCloud2, PREFIX + '/map', latched)
        self.tomogram_pub = self.create_publisher(PointCloud2, PREFIX + '/tomogram', latched)
        self.blocked_pub = self.create_publisher(PointCloud2, PREFIX + '/blocked_surfaces', latched)
        self.selected_pubs = {role: self.create_publisher(PointStamped, PREFIX + '/selected_' + role, latched)
                              for role in ('start', 'goal')}
        self.marker_server = InteractiveMarkerServer(self, '/pct_preview_points')
        for role in ('start', 'goal'):
            self.create_subscription(PointStamped, PREFIX + '/' + role,
                lambda msg, role=role: self.on_point(role, msg), 10)
            self.create_subscription(Empty, PREFIX + '/snap_' + role,
                lambda msg, role=role: self.on_snap(role), 10)
            self.create_subscription(Empty, PREFIX + '/activate_' + role,
                lambda msg, role=role: self.on_activate(role), 10)
            self.create_subscription(Empty, PREFIX + '/reset_' + role + '_orientation',
                lambda msg, role=role: self.on_reset_orientation(role), 10)
        self.create_subscription(Empty, PREFIX + '/plan', self.on_plan, 10)
        self.create_subscription(Empty, PREFIX + '/clear', self.on_clear, 10)
        self.create_subscription(String, PREFIX + '/selection_mode', self.on_mode, 10)
        self.create_subscription(Int32, PREFIX + '/layer', self.on_layer, 10)
        self.worker_context = multiprocessing.get_context('spawn')
        self.requests = self.worker_context.Queue(maxsize=2)
        self.results = self.worker_context.Queue(maxsize=4)
        from .compute_budget import budgeted_worker, spawn_with_environment
        from .native_runtime import prepare_native_environment
        self.worker = None
        if not self.settings['external_planner']:
            self.worker = self.worker_context.Process(target=budgeted_worker,
                args=(worker_target, self.settings, self.requests, self.results), daemon=True)
            spawn_with_environment(self.worker, prepare_native_environment(self.settings['vendor_root']))
        self.created_at = time.monotonic()
        self.timer = self.create_timer(0.2, self.tick)
        self.publish_map()
        self.publish_path(None)
        self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        if initial:
            self.selection.set_point('start', initial)
            self.update_endpoint('start')
        if self.settings['place_endpoints_on_start']:
            for role in ('start', 'goal'):
                try:
                    self.selection.activate(role, self.settings['placement_anchor_xy'])
                    self.update_endpoint(role)
                except ValueError as exc:
                    self.get_logger().warning(f'Could not seed {role}: {exc}')
            self.state, self.reason = 'editing', 'Drag the start and goal handles, then click Plan'
        if self.settings['restore_route_file']:
            self.restore_previous_route()
        self.publish_tomogram()
        self.publish_status()

    def restore_previous_route(self):
        """Explicit startup restoration only, never fallback after a failed Plan.

        Identical measured geometry and non-increasing costs must be proved
        before an earlier fully validated native curve is displayed again.
        """
        try:
            if not self.layered:
                raise ValueError('Route restoration requires official layered PCT')
            from .route_restore import restore_verified_route
            if self.settings['crossfloor_route_config']:
                from .crossfloor_preview import restore_crossfloor
                result = restore_crossfloor(self.grid, self.settings['crossfloor_route_config'],
                                           self.settings['restore_route_file'])
            else:
                result = restore_verified_route(self.grid,
                    self.settings['restore_tomogram_path'], self.settings['restore_route_file'])
            self.selection.clear()
            for role in ('start', 'goal'):
                self.selection.set_active_layer(result[role + '_layer'])
                self.selection.set_point(role, result[role + '_xyz'])
                self.selection.validate(role)
                self.update_endpoint(role)
            self.selection.set_active_layer(-1)
            points = np.asarray(result['path'], dtype=float)
            if not np.allclose(points[[0, -1]], [self.selection.validate('start'),
                              self.selection.validate('goal')], atol=1e-8, rtol=0):
                raise ValueError('Restored editor endpoints differ from the verified route')
            self.path_data = points
            self.last_result = {key: value for key, value in result.items() if key != 'path'}
            self.publish_path(points)
            self.state = 'planned'
            self.reason = f'原路线复核通过：{result["length_m"]:.2f} m（非重新规划）'
            if self.output:
                (self.output / 'restored_path.json').write_text(
                    json.dumps(result, indent=2, allow_nan=False) + '\n')
        except (ValueError, OSError) as exc:
            self.invalidate_path()
            self.state, self.reason = 'failed', f'Previous route not restored: {exc}'
            self.get_logger().warning(self.reason)

    def header(self):
        return Header(frame_id=self.frame, stamp=self.get_clock().now().to_msg())

    def validate_header(self, header, allow_zero=False):
        if header.frame_id != self.frame:
            raise ValueError(f'Point frame must be {self.frame}; no coordinate relabeling')
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if (not stamp and not allow_zero) or (stamp and (
                age > self.settings['selection_max_age_s'] or age < -0.1)):
            raise ValueError('Point timestamp is stale or in the future')

    def invalidate_path(self):
        self.pending = None
        self.path_data = None
        self.last_result = None
        self.last_error = None
        self.publish_path(None)

    def edited(self, role, xyz, replace_marker=True):
        changed = self.selection.set_point(role, xyz)
        if changed:
            self.invalidate_path()
        elif self.state in ('planned', 'planning'):
            self.update_endpoint(role, replace_marker=replace_marker)
            self.publish_status()
            return
        self.state, self.reason = 'editing', 'Adjust XYZ, then click Plan'
        try:
            self.selection.validate(role)
        except ValueError as exc:
            self.state, self.reason = 'invalid_selection', str(exc)
        self.update_endpoint(role, replace_marker=replace_marker)
        self.publish_status()

    def on_point(self, role, message):
        try:
            self.validate_header(message.header)
            self.edited(role, [message.point.x, message.point.y, message.point.z])
        except ValueError as exc:
            self.state, self.reason = 'input_rejected', str(exc)
            self.get_logger().warning(self.reason)
            self.publish_status()

    def on_activate(self, role):
        try:
            if self.selection.activate(role, self.settings['placement_anchor_xy']):
                self.invalidate_path()
                self.state, self.reason = 'editing', f'{role} marker placed; drag XYZ to set position'
            self.update_endpoint(role)
        except ValueError as exc:
            self.state, self.reason = 'failed', str(exc)
        self.publish_status()

    def on_feedback(self, feedback):
        if feedback.event_type not in (InteractiveMarkerFeedback.POSE_UPDATE,
                                       InteractiveMarkerFeedback.MOUSE_UP):
            return
        if feedback.marker_name not in self.selection.points or self.selection.points[feedback.marker_name] is None:
            return
        try:
            # Fixed-frame interactive markers use zero stamps intentionally.
            self.validate_header(feedback.header, allow_zero=True)
            point = feedback.pose.position
            quat = feedback.pose.orientation
            # Validate the entire pose before changing either half. Rotation
            # belongs to the editor, not the XYZ planner's generation counter.
            xyz = finite_xyz([point.x, point.y, point.z])
            orientation = unit_quaternion([quat.x, quat.y, quat.z, quat.w])
            self.selection.set_orientation(feedback.marker_name, orientation)
            self.edited(feedback.marker_name, xyz, replace_marker=False)
            if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                # Rebuild only after release: retain full attitude, update the
                # valid/invalid visual, never reset a ring drag to identity.
                self.update_endpoint(feedback.marker_name)
        except ValueError as exc:
            self.state, self.reason = 'input_rejected', str(exc)
            self.update_endpoint(feedback.marker_name)
            self.publish_status()
        self.marker_server.applyChanges()

    def on_reset_orientation(self, role):
        try:
            self.selection.set_orientation(role, [0., 0., 0., 1.])
            self.update_endpoint(role)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
        self.publish_status()

    def on_mode(self, message):
        if not self.layered:
            return
        try:
            self.selection.set_mode(message.data)
            for role in ('start', 'goal'):
                self.update_endpoint(role)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
        self.publish_status()

    def on_layer(self, message):
        if not self.layered:
            return
        try:
            if self.crossfloor_settings and message.data in (-2, -3):
                self.selection.set_active_floor('floor1' if message.data == -2 else 'floor2')
            else:
                self.selection.set_active_layer(message.data)
            self.publish_tomogram()
        except ValueError as exc:
            self.get_logger().warning(str(exc))
        self.publish_status()

    def on_snap(self, role):
        try:
            changed = self.selection.snap_ground(role)
            if changed:
                self.invalidate_path()
            self.update_endpoint(role)
            if changed or self.state not in ('planned', 'planning'):
                self.state, self.reason = 'editing', f'{role} Z explicitly set to measured ground'
        except ValueError as exc:
            self.state, self.reason = 'invalid_selection', str(exc)
        self.publish_status()

    def on_clear(self, _message):
        self.selection.clear()
        self.invalidate_path()
        self.marker_server.clear()
        self.marker_server.applyChanges()
        self.state, self.reason = 'waiting_start', 'Select 3D Start, then 3D Goal'
        self.publish_selection_evidence()
        self.publish_tomogram()
        self.publish_status()

    def on_plan(self, _message):
        if self.pending is not None:
            return  # Button repeat is idempotent while this request is pending.
        try:
            if not self.ready or not self.worker.is_alive():
                raise ValueError('PCT worker is not ready; restart if it failed')
            request = self.selection.request()
            # Edits invalidate results by revision. Drop obsolete queued work;
            # an already executing native solve stays isolated and cannot win.
            try:
                while True:
                    self.requests.get_nowait()
            except queue.Empty:
                pass
            self.requests.put_nowait(request)
            self.publish_path(None)
            self.path_data = None
            self.last_result = None
            self.last_error = None
            self.pending = {**request, 'requested_at': time.monotonic()}
            self.state, self.reason = 'planning', 'PCT search + GPMP'
        except (ValueError, queue.Full) as exc:
            self.state, self.reason = 'failed', str(exc) or 'Planner queue is busy'
        self.publish_status()

    def update_endpoint(self, role, replace_marker=True):
        xyz = self.selection.points[role]
        if xyz is None:
            return
        point = PointStamped(header=self.header())
        point.point.x, point.point.y, point.point.z = map(float, xyz)
        self.selected_pubs[role].publish(point)
        if replace_marker:
            try:
                self.selection.validate(role)
                valid = True
            except ValueError:
                valid = False
            self.marker_server.insert(pose_marker(role, xyz, self.selection.orientations[role],
                self.frame, self.settings['handle_scale_m'], valid=valid,
                ground_follow=self.layered and self.selection.mode == 'ground'),
                feedback_callback=self.on_feedback)
            self.marker_server.applyChanges()
        self.publish_tomogram()
        self.publish_selection_evidence()

    def publish_selection_evidence(self):
        if not self.layered:
            return
        from .selection_markers import selection_evidence_markers
        markers = []
        for index, role in enumerate(('start', 'goal')):
            markers.extend(selection_evidence_markers(role, self.selection.points[role],
                self.selection.validation(role), self.header(), index*3))
        self.marker_pub.publish(MarkerArray(markers=markers))

    def publish_tomogram(self):
        if not self.layered:
            return
        chosen = ({self.selection.active_layer} if self.selection.active_layer >= 0 else
                  {layer for layer in self.selection.layers.values() if layer is not None})
        if self.selection.active_floor is not None:
            lo, hi = self.selection.floor_ranges[self.selection.active_floor]
            chosen.update(item['layer_id'] for item in self.layer_metadata
                if item['ground_min_m'] is not None and item['ground_min_m'] <= hi
                and item['ground_max_m'] >= lo)
        if self.selection.active_layer < 0 and self.last_result:
            chosen.update(self.last_result.get('layer_ids', []))
        if not chosen:
            candidates = [item for item in self.layer_metadata if item['traversable_cells'] > 0]
            if candidates:
                # A missing selection must not default to a roof. Use the
                # configured physical anchor height, as initial placement does.
                try:
                    seed = self.grid.initial_seed([*self.settings['placement_anchor_xy'],
                                                    self.settings['placement_anchor_z']])
                    chosen.add(seed['layer_id'])
                except ValueError:
                    pass
        signature = tuple(sorted(chosen))
        display_key = (signature, self.selection.active_floor)
        if display_key == getattr(self, '_display_key', None):
            return
        self._display_key = display_key
        self.display_layers = signature
        xyz, costs, xyz_blocked = surface_clouds(self.grid, signature)
        if self.crossfloor_settings:
            from .crossfloor_preview import visible_surfaces
            mask = visible_surfaces(xyz, self.crossfloor_settings)
            xyz, costs = xyz[mask], costs[mask]
            xyz_blocked = xyz_blocked[visible_surfaces(xyz_blocked, self.crossfloor_settings)]
            if self.selection.active_floor is not None:
                lo, hi = self.selection.floor_ranges[self.selection.active_floor]
                mask = (xyz[:, 2] >= lo) & (xyz[:, 2] <= hi)
                xyz, costs = xyz[mask], costs[mask]
                xyz_blocked = xyz_blocked[(xyz_blocked[:, 2] >= lo) & (xyz_blocked[:, 2] <= hi)]
        def cloud_message(xyz, costs=None):
            xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
            count = len(xyz)
            values = np.empty(count, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
            for axis, name in enumerate(('x', 'y', 'z')):
                values[name] = xyz[:, axis]
            if costs is None:
                values['rgb'] = np.uint32((220 << 16) | (68 << 8) | 68)
            else:
                weight = np.clip(np.asarray(costs, dtype=float) / 20., 0., 1.)
                red = (45 + 200 * weight).astype(np.uint32)
                green = (225 - 45 * weight).astype(np.uint32)
                values['rgb'] = (red << 16) | (green << 8) | 65
            msg = PointCloud2(header=self.header(), height=1, width=count,
                fields=[PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z', 'rgb'))],
                is_bigendian=False, point_step=16, row_step=count * 16, is_dense=True)
            msg.data = array('B', values.tobytes())
            return msg
        self.tomogram_pub.publish(cloud_message(xyz, costs))
        self.blocked_pub.publish(cloud_message(xyz_blocked))

    def publish_path(self, points):
        message = Path(header=self.header())
        if points is not None:
            for i, point in enumerate(points):
                pose = PoseStamped(header=message.header)
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point)
                direction = points[min(i + 1, len(points) - 1), :2] - points[max(0, i - 1), :2]
                yaw = math.atan2(direction[1], direction[0])
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                message.poses.append(pose)
        self.path_pub.publish(message)

    def publish_map(self):
        import open3d as o3d
        cloud = o3d.io.read_point_cloud(self.settings['map_pcd'])
        xyz = np.asarray(cloud.points, dtype=np.float32)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if not len(xyz):
            raise ValueError('PCD has no finite points')
        limit = int(self.settings['map_display_max_points'])
        if limit < 1:
            raise ValueError('map_display_max_points must be positive')
        # Keep actual measured positions instead of voxel centroid shifts.
        xyz = np.ascontiguousarray(xyz[::max(1, math.ceil(len(xyz) / limit))], dtype='<f4')
        message = PointCloud2(header=self.header(), height=1, width=len(xyz),
            fields=[PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(('x', 'y', 'z'))],
            is_bigendian=False, point_step=12, row_step=len(xyz) * 12, is_dense=True)
        message.data = array('B', xyz.tobytes())
        self.cloud_pub.publish(message)

    def publish_status(self):
        layer_status = {}
        if self.layered:
            floors = ([{'id': -2, 'label': '一楼 · 多切片地面'},
                       {'id': -3, 'label': '二楼 · 多切片地面'}] if self.crossfloor_settings else [])
            layer_status = {'selection_mode': self.selection.mode,
                'active_layer': ({'floor1': -2, 'floor2': -3}.get(self.selection.active_floor,
                                 self.selection.active_layer)),
                'active_floor': self.selection.active_floor,
                'available_layers': [{'id': -1, 'label': '自动跟随当前地面'}] + floors + [
                    {'id': item['layer_id'],
                     'label': f'分层 #{item["layer_id"]} · 切面 {item["slice_height_m"]:.2f} m',
                     **item} for item in self.layer_metadata if item['traversable_cells'] > 0],
                'display_layers': list(self.display_layers or []),
                'start_validation': self.selection.validation('start'),
                'goal_validation': self.selection.validation('goal'),
                'source_tomogram_sha256': self.grid.sha256,
                'map_provenance': self.grid.provenance,
                'map_resolution_m': self.grid.resolution,
                'unknown_ceiling_policy': self.grid.unknown_ceiling_policy}
        status = {'mode': 'GLOBAL_PATH_PREVIEW_ONLY', 'state': self.state, 'reason': self.reason,
                  'frame_id': self.frame, 'ready': self.ready, 'revision': self.selection.revision,
                  **self.selection.snapshot(), **self.selection.orientation_snapshot(),
                  'result': self.last_result,
                  'error_code': self.last_error.get('error_code') if self.last_error else None,
                  'reason_code': self.last_error.get('error_code') if self.last_error else self.reason,
                  'error_details': self.last_error.get('details') if self.last_error else None,
                  'native_runtime': self.native_runtime,
                  'native_parameters': self.native_parameters,
                  'robot_connected': False, 'motion_enabled': False,
                  'map_backend': 'official_pct_tomogram_cpu' if self.layered else 'single_floor_measured_grid_to_native_pct',
                  **layer_status,
                  'height_tolerance_m': self.selection.height_tolerance}
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    def receive_result(self, result):
        if not self.pending or result.get('generation') != self.pending['generation']:
            return
        request, self.pending = self.pending, None
        try:
            if result['kind'] == 'failed':
                self.last_error = {'error_code': result.get('error_code', 'planner_failed'),
                                   'details': result.get('details', {})}
                raise ValueError(result['error'])
            points = self.selection.checked_result(result, request, self.settings['max_ground_step_m'])
            if points is None:
                return
            self.path_data = points
            self.last_error = None
            self.last_result = {key: value for key, value in result.items() if key != 'path'}
            self.last_result['length_m'] = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
            self.last_result['height_semantics'] = 'ground_z_with_exact_selected_endpoints'
            self.publish_path(points)
            self.publish_tomogram()
            self.state, self.reason = 'planned', f'Path ready: {self.last_result["length_m"]:.2f} m'
            if self.output:
                artifact = {**self.last_result, **self.selection.snapshot(), 'path': points.tolist(),
                            **self.selection.orientation_snapshot(),
                            'frame_id': self.frame, 'mode': 'GLOBAL_PATH_PREVIEW_ONLY'}
                path = self.output / f'path_{self.selection.revision:06d}.json'
                path.write_text(json.dumps(artifact, indent=2, allow_nan=False) + '\n')
        except (ValueError, OSError) as exc:
            if self.last_error is None:
                self.last_error = {'error_code': getattr(exc, 'code', 'planner_failed'),
                                   'details': getattr(exc, 'details', {})}
            self.state, self.reason = 'failed', str(exc)
            self.path_data = None
            self.last_result = None
            self.publish_path(None)
            self.get_logger().warning(self.reason)

    def tick(self):
        if self.settings['external_planner']:
            self.publish_status()
            return
        try:
            while True:
                result = self.results.get_nowait()
                if result['kind'] == 'ready':
                    if self.layered and result.get('source_tomogram_sha256') != self.grid.sha256:
                        self.state, self.reason = 'failed', 'Worker and editor tomograms differ; restart preview'
                        self.worker.terminate()
                        continue
                    self.native_runtime = result.get('native_runtime')
                    self.native_parameters = result.get('native_parameters')
                    self.ready = True
                    if self.state == 'initializing':
                        self.state, self.reason = 'waiting_start', 'Select 3D Start, then 3D Goal'
                elif result['kind'] == 'initialization_failed':
                    self.ready = False
                    self.state, self.reason = 'failed', result['error']
                    self.last_error = {'error_code': result.get('error_code', 'initialization_failed'),
                                       'details': result.get('details', {})}
                else:
                    self.receive_result(result)
        except queue.Empty:
            pass
        elapsed = time.monotonic() - (self.pending['requested_at'] if self.pending else self.created_at)
        timeout = ((not self.ready and self.worker.is_alive()) or self.pending is not None) and elapsed > self.settings['planning_timeout_s']
        if timeout or (not self.worker.is_alive() and self.state != 'failed'):
            self.ready = False
            self.invalidate_path()
            self.state, self.reason = 'failed', 'PCT worker timed out/exited; restart preview'
            if self.worker.is_alive():
                self.worker.terminate()
        self.publish_status()

    def destroy_node(self):
        self.marker_server.shutdown()
        if self.worker is not None and self.worker.is_alive():
            self.worker.terminate()
        if self.worker is not None:
            self.worker.join(timeout=2)
        for channel in (self.requests, self.results):
            channel.cancel_join_thread()
            channel.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PctPreviewServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
