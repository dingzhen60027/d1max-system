"""Isolated ROS acceptance for full official layered-PCT preview only.

No rendered RViz, SDK, robot connection or physical navigation is exercised.
Run via tools/verify_preview_isolated.py to avoid competing with user edits.
"""
import argparse
import json
from pathlib import Path
import traceback

import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty, Int32, String
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarkerFeedback

from .preview_verify import FRAME, PREFIX, PreviewVerifier
from .tomogram_map import TomogramMap


def cloud_values(message):
    if (message.header.frame_id != FRAME or message.height != 1 or message.is_bigendian
            or message.row_step != message.width * message.point_step):
        raise AssertionError('Unexpected tomogram cloud frame/layout')
    offsets = {field.name: field.offset for field in message.fields}
    if not all(name in offsets for name in ('x', 'y', 'z', 'rgb')):
        raise AssertionError('Tomogram visualization needs XYZ plus RGB')
    dtype = np.dtype({'names': ['x', 'y', 'z', 'rgb'], 'formats': ['<f4'] * 3 + ['<u4'],
                      'offsets': [offsets[name] for name in ('x', 'y', 'z', 'rgb')],
                      'itemsize': message.point_step})
    values = np.frombuffer(message.data, dtype=dtype, count=message.width)
    xyz = np.column_stack([values[name] for name in ('x', 'y', 'z')])
    if not np.isfinite(xyz).all():
        raise AssertionError('Tomogram display contains nonfinite positions')
    return xyz, values['rgb']


class OfficialPreviewVerifier(PreviewVerifier):
    def __init__(self, report):
        super().__init__(report)
        self.clouds = {}
        for name in ('tomogram', 'blocked_surfaces'):
            self.create_subscription(PointCloud2, PREFIX + '/' + name,
                lambda message, name=name: self.clouds.__setitem__(name, message), self.latched)
        self.mode_pub = self.create_publisher(String, PREFIX + '/selection_mode', 10)
        self.layer_pub = self.create_publisher(Int32, PREFIX + '/layer', 10)

    def validation(self, role):
        return self.status.get(role + '_validation', {})

    def set_mode(self, mode):
        self.mode_pub.publish(String(data=mode))
        self.wait(lambda: self.status.get('selection_mode') == mode, 4.0, f'Mode ACK: {mode}')
        self.hold(0.25, lambda: self.status.get('selection_mode') == mode, 'Mode ACK must persist')
        self.check('selection_mode_ack', selection_mode=mode)

    def set_layer(self, layer):
        before = {role: self.xyz(role) for role in ('start', 'goal')}
        revision = self.status['revision']
        self.layer_pub.publish(Int32(data=layer))
        self.wait(lambda: self.status.get('active_layer') == layer, 4.0, f'Layer ACK: {layer}')
        self.hold(0.25, lambda: self.status['revision'] == revision
                  and all(self.xyz(role) == xyz for role, xyz in before.items()),
                  'Layer selection must not teleport existing XYZ')
        self.check('active_layer_ack_without_endpoint_teleport', active_layer=layer)

    def audit_clouds(self, tomogram, layer):
        self.wait(lambda: self.status.get('display_layers') == [layer]
                  and all(name in self.clouds for name in ('tomogram', 'blocked_surfaces')),
                  5.0, 'True tomogram and blocked-surface publishers')
        # Allow the independent latched cloud deliveries to follow the status ACK.
        expected_counts = {'tomogram': int(tomogram.allowed[layer].sum()),
                           'blocked_surfaces': int((tomogram.ground_known[layer] & ~tomogram.allowed[layer]).sum())}
        self.wait(lambda: all(self.clouds[name].width == count for name, count in expected_counts.items()),
                  4.0, 'Displayed cloud counts correspond to selected authoritative layer')
        result = {}
        for name, count in expected_counts.items():
            xyz, rgb = cloud_values(self.clouds[name])
            if not count:
                raise AssertionError('This real-map regression requires both allowed and blocked cells')
            indices = np.rint((xyz[:, :2] - tomogram.center) / tomogram.resolution).astype(int) + tomogram.offset
            if len(np.unique(indices, axis=0)) != count:
                raise AssertionError('Repeated display cells mask missing authoritative cells')
            mask = (tomogram.allowed[layer] if name == 'tomogram' else
                    tomogram.ground_known[layer] & ~tomogram.allowed[layer])
            if not mask[indices[:, 0], indices[:, 1]].all():
                raise AssertionError('Display disagrees with authoritative planning/selection mask')
            if not np.allclose(xyz[:, 2], tomogram.ground[layer, indices[:, 0], indices[:, 1]], atol=1e-6, rtol=0):
                raise AssertionError('Display Z was flattened or invented')
            if name == 'blocked_surfaces':
                if not np.all(rgb == (220 << 16) | (68 << 8) | 68):
                    raise AssertionError('Blocked surfaces must use the configured red overlay')
            else:
                weight = tomogram.cost[layer, indices[:, 0], indices[:, 1]].astype(float) / 20
                expected = ((45 + 200 * weight).astype(np.uint32) << 16
                            | (225 - 45 * weight).astype(np.uint32) << 8 | 65)
                if not np.array_equal(rgb, expected):
                    raise AssertionError('Traversability color does not correspond to the official cost')
            result[name] = {'points': count, 'authoritative_mask_match': True}
        self.check('true_tomogram_and_blocked_clouds_share_the_planning_mask_and_real_z',
                   layer_id=layer, clouds=result)

    def audit_ground_handles(self):
        for role, marker in self.get_markers().items():
            controls = {control.name: control for control in marker.controls}
            center = controls.get('move_rotate_3d')
            if (set(controls) != {'move_rotate_3d', 'move_x', 'move_y', 'rotate_z'}
                    or center.interaction_mode != InteractiveMarkerControl.MOVE_PLANE
                    or center.orientation_mode != InteractiveMarkerControl.FIXED):
                raise AssertionError(f'{role}: ground mode must use map-plane XY controls, not view-plane Z drift')
            q = center.orientation
            normal = np.array([1 - 2 * (q.y * q.y + q.z * q.z),
                               2 * (q.x * q.y + q.w * q.z), 2 * (q.x * q.z - q.w * q.y)])
            if not np.allclose(normal, [0, 0, 1], atol=1e-8):
                raise AssertionError('Ground handle plane must be normal to map Z')
        self.check('ground_mode_handles_use_fixed_map_xy_plane')

    def reject_plan(self, label):
        count = self.status_count
        self.commands['plan'].publish(Empty())
        self.wait(lambda: self.status_count > count and self.status['state'] == 'failed',
                  5.0, label)
        if self.path is None or self.path.poses or self.status.get('result') is not None:
            raise AssertionError('Invalid endpoint must not retain or publish a successful route')

    def audit_native_runtime(self, runtime):
        if (not isinstance(runtime, dict) or runtime.get('runtime_verified') is not True
                or runtime.get('gtsam_version') != '4.1.1' or runtime.get('missing_libraries')):
            raise AssertionError('Native PCT worker must prove its actual loaded GTSAM 4.1.1 runtime')
        gtsam = runtime.get('gtsam_library', '')
        if ('4.1.1' not in gtsam or gtsam not in runtime.get('loaded_libraries', [])
                or not Path(gtsam).is_relative_to(Path(runtime['vendor_root']))):
            raise AssertionError('Native worker loaded the wrong GTSAM shared library')
        self.check('native_worker_loaded_verified_bundled_gtsam_4_1_1',
                   gtsam_library=gtsam, loaded_libraries=runtime['loaded_libraries'])

    def run_official(self, tomogram):
        self.wait(lambda: self.status is not None and self.status.get('ready'), 35.0, 'Official PCT ready')
        self.wait(lambda: all(pub.get_subscription_count() > 0 for pub in
                  [*self.points.values(), *self.commands.values(), self.feedback, self.mode_pub, self.layer_pub]),
                  5.0, 'Official preview command discovery')
        if (self.status.get('map_backend') != 'official_pct_tomogram_cpu'
                or self.status.get('source_tomogram_sha256') != tomogram.sha256):
            raise AssertionError('Preview must use the real supplied tomogram, without legacy fallback')
        self.check('official_safe_npz_backend_and_hash_match', source_tomogram_sha256=tomogram.sha256)
        self.audit_native_runtime(self.status.get('native_runtime'))
        self.audit_publishers()
        self.clear()
        self.set_mode('ground')
        self.set_layer(2)
        start = tomogram.select([-4.100001525878895, 6.9, -0.617390871],
                                mode='ground_follow', layer_lock=2)['xyz']
        goal = tomogram.select([-6.5, 12.1, -0.632633269], mode='ground_follow', layer_lock=2)['xyz']
        for role, point in (('start', start), ('goal', goal)):
            self.select(role, point)
            self.wait(lambda role=role, point=point: self.matches(role, point)
                      and self.validation(role).get('valid') is True
                      and self.validation(role).get('layer_id') == 2, 4.0, f'Supported {role} layer ACK')
        self.audit_clouds(tomogram, 2)
        self.audit_ground_handles()
        self.hold(0.4, lambda: not self.path.poses and self.status['state'] != 'planned',
                  'Selecting points must not implicitly plan')

        requested = np.asarray(goal) + [0.002, 0.003, 9.0]
        expected = tomogram.select([*requested[:2], goal[2]], mode='ground_follow', layer_lock=2)['xyz']
        revision = self.status['revision']
        message = self.feedback_message('goal', 'move_rotate_3d', requested)
        message.event_type = InteractiveMarkerFeedback.POSE_UPDATE
        self.feedback.publish(message)
        self.wait(lambda: self.matches('goal', expected) and self.status['revision'] > revision,
                  4.0, 'Ground feedback preserves XY and follows only real surface Z')
        message.event_type = InteractiveMarkerFeedback.MOUSE_UP
        self.feedback.publish(message)
        self.hold(0.4, lambda: self.matches('goal', expected), 'Ground release must not restore camera-derived Z')
        self.assert_marker_pose('goal', expected, self.orientation('goal'))
        self.check('ground_feedback_keeps_requested_xy_ignores_camera_z_and_follows_measured_surface',
                   raw_feedback_xyz=requested.tolist(), selected_xyz=expected)
        self.select('goal', goal)
        self.wait(lambda: self.matches('goal', goal), 4.0, 'Restore exact regression goal')
        path = self.plan(start, goal)
        result = dict(self.status['result'])
        self.audit_native_runtime(result.get('native_runtime'))
        tomogram.validate_path(path, result['layer_ids'])
        if result.get('extra_erosion_cells') != 0 or result.get('curve_validation') != 'quintic_cell_boundary_roots_and_interval_interiors':
            raise AssertionError('Official native route must report full curve validation and no extra erosion')
        if not 5.5 <= result['length_m'] <= 7.5:
            raise AssertionError('Regression route length unexpectedly changed')
        self.report['ground_route'] = {'path': path.tolist(), **result}
        self.check('official_layered_native_gpmp_short_route_and_exact_endpoints', result=result)

        self.set_mode('free')
        self.audit_markers()
        high = np.asarray(goal) + [0, 0, .5]
        self.select('goal', high)
        self.wait(lambda: self.matches('goal', high) and not self.validation('goal').get('valid', True),
                  4.0, 'Free mode keeps floating XYZ visible but invalid')
        self.reject_plan('Free high-Z Plan rejected')
        self.check('free_high_z_is_kept_without_snapping_and_plan_is_rejected',
                   xyz=self.xyz('goal'), validation=self.validation('goal'))
        self.commands['snap_goal'].publish(Empty())
        self.wait(lambda: self.matches('goal', goal) and self.validation('goal').get('valid') is True,
                  4.0, 'Explicit Ground restores physical surface')
        if self.path.poses:
            raise AssertionError('Snap Ground must not implicitly plan')
        self.check('explicit_ground_restores_z_only_without_auto_planning')

        free_start, free_goal = np.asarray(start) + [0, 0, .025], np.asarray(goal) + [0, 0, .025]
        for role, point in (('start', free_start), ('goal', free_goal)):
            self.select(role, point)
            self.wait(lambda role=role, point=point: self.matches(role, point)
                      and self.validation(role).get('valid') is True, 4.0, 'Free near-ground endpoint accepted unchanged')
        free_path = self.plan(free_start, free_goal)
        self.report['free_route'] = {'path': free_path.tolist(), **self.status['result']}
        self.check('free_xyz_endpoints_within_tolerance_are_preserved_by_native_result',
                   start_xyz=self.xyz('start'), goal_xyz=self.xyz('goal'))

        # A layer selector affects subsequent edits/display only. It must not
        # teleport the current endpoint to the roof or pretend to re-plan.
        self.set_layer(6)
        self.set_layer(-1)
        invalid = [-12.7454586, 22.6254902, -0.90158]
        self.select('goal', invalid)
        self.wait(lambda: self.matches('goal', invalid) and not self.validation('goal').get('valid', True),
                  4.0, 'Known invalid ground goal must remain invalid in Auto mode')
        self.reject_plan('Known blocked target cannot be promoted to a roof route')
        if self.xyz('goal') != invalid:
            raise AssertionError('Known invalid target was silently relocated')
        self.check('known_blocked_target_is_not_xy_snapped_or_replaced_with_roof_surface',
                   xyz=self.xyz('goal'), validation=self.validation('goal'))
        self.audit_publishers()
        self.clear()
        self.check('final_clear_revokes_path_and_endpoints')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tomogram', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--max-ground-step', type=float, default=.17)
    options = parser.parse_args(args)
    report = {'passed': False, 'checks': [], 'mode': 'OFFICIAL_PCT_ISOLATED_BACKEND_ACCEPTANCE',
              'scope': 'Real layered tomogram, ROS selection/marker/path protocol; no rendered GUI, mouse or physical robot acceptance',
              'robot_connected': False, 'motion_enabled': False, 'frame_id': FRAME}
    node = None
    rclpy.init(args=[])
    try:
        tomogram = TomogramMap(options.tomogram, max_ground_step_m=options.max_ground_step)
        report['tomogram'] = str(options.tomogram.resolve())
        report['source_tomogram_sha256'] = tomogram.sha256
        node = OfficialPreviewVerifier(report)
        node.run_official(tomogram)
        report['passed'] = True
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        report['traceback'] = traceback.format_exc()
        print(report['error'], flush=True)
    finally:
        if node:
            try:
                node.clear()
                report['cleanup'] = 'cleared; waiting_start; no result'
            except Exception as exc:
                report['cleanup_error'], report['passed'] = str(exc), False
            report['events'], report['final_status'] = node.events, node.status
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'passed': report['passed'], 'report': str(options.output)}), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
