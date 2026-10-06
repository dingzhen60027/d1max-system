"""Official Isaac Sim 6 Spot PhysX plant, with measured articulated geometry.

The official Spot observation/forward implementation and matching NVIDIA PhysX
TorchScript/env are used unchanged. A local-only constructor avoids the stock
constructor's unconditional remote asset-root probe. step/reset never write the
root pose or velocity. Policy initialization writes the initial 12 joint states
once, as the official controller requires. This module can be imported without
Kit for configuration/geometry tests; construct the plant after SimulationApp.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np

PHYSICS_DT = .002
POLICY_DECIMATION = 10
MODEL_KIND = 'official_spot_physx'
DEFAULT_ROOT_PATH = '/World/Spot'
BODY_SUFFIX = '/body'
ARTICULATION_SUFFIX = ''
DEFAULT_ASSET_ROOT = Path(os.environ.get('D1MAX_SIM_BUILD',
    str(Path(__file__).resolve().parents[3]/'d1max-build-isaac')))/'quadruped-assets-spot'
USD_RELATIVE = 'Isaac/Robots/BostonDynamics/spot/spot.usd'
POLICY_RELATIVE = 'Isaac/Samples/Policies/Spot_Policies/spot_policy.pt'
ENV_RELATIVE = 'Isaac/Samples/Policies/Spot_Policies/spot_env.yaml'


class VelocityFeedback:
    """Bounded PI velocity servo feeding the official policy's command input.

    Measurements are body-axis PhysX velocity, never command-derived state.
    No pose, velocity or joint-state setters are used by this servo. Integral
    anti-windup and command saturation keep its policy-input bound explicit.
    """
    def __init__(self, max_policy_linear=.3, max_policy_angular=.5, mode='legacy_pi_v1'):
        self.limits = np.asarray([max_policy_linear, max_policy_angular], dtype=float)
        if not np.isfinite(self.limits).all() or np.any(self.limits <= 0):
            raise ValueError('invalid_policy_velocity_limits')
        if mode not in ('legacy_pi_v1', 'spot_nonlinear_measured_v1'):
            raise ValueError('invalid_velocity_feedback_mode')
        self.mode = mode
        self.kp = np.asarray([.35, .5])
        self.ki = np.asarray([1. if mode == 'spot_nonlinear_measured_v1' else .3, .35])
        self.integral_limit = np.asarray([.25 if mode == 'spot_nonlinear_measured_v1' else .1, .15])
        self.reset()

    def reset(self):
        self.integral = np.zeros(2)
        self.filtered = None

    def update(self, dt, desired, measured):
        desired, measured = np.asarray(desired, dtype=float), np.asarray(measured, dtype=float)
        if not math.isfinite(dt) or dt <= 0 or desired.shape != (2,) or measured.shape != (2,) or not np.isfinite(np.r_[desired, measured]).all():
            raise ValueError('invalid_velocity_feedback_sample')
        if np.any(np.abs(desired) > self.limits + 1e-9):
            raise ValueError('desired_outside_policy_velocity_limits')
        if self.filtered is None:
            self.filtered = measured.copy()
        else:
            self.filtered += (1. - math.exp(-dt / .05)) * (measured - self.filtered)
        error = desired - self.filtered
        feedforward = np.zeros(2)
        if self.mode == 'spot_nonlinear_measured_v1':
            # Smooth, signed low-level policy-input compensation. The original
            # desired velocity is unchanged, including arbitrarily small and
            # exactly zero commands. This is not a minimum navigation speed.
            feedforward[0] = .25 * math.tanh(desired[0] / .005) * math.exp(-abs(desired[0]) / .15)
        proposed = np.clip(self.integral + self.ki * error * dt, -self.integral_limit, self.integral_limit)
        raw = desired + feedforward + self.kp * error + proposed
        pushes_saturation = ((raw > self.limits) & (error > 0)) | ((raw < -self.limits) & (error < 0))
        self.integral = np.where(pushes_saturation, self.integral, proposed)
        target = desired + feedforward + self.kp * error + self.integral
        if self.mode == 'spot_nonlinear_measured_v1' and desired[0] == 0.:
            # Withdraw all forward feedforward/history from live drive at zero
            # demand. Source-time decay preserves correction across a brief
            # zero pulse; parking still uses actual measured velocity feedback.
            # Order matches the measured smooth_ff_v1 component experiment:
            # error integration + antiwindup, then decay, then output withdrawal.
            self.integral[0] *= math.exp(-dt / .5)
            target[0] = self.kp[0] * error[0]
        return np.clip(target, -self.limits, self.limits)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def array(value):
    return value.numpy() if hasattr(value, 'numpy') else np.asarray(value)


def rotation_wxyz(q):
    """Column-vector rotation; normalize only for geometry, retain raw source q."""
    q = np.asarray(q, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-10:
        raise ValueError('invalid_measured_quaternion')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def pose_matrix(position, quaternion_wxyz):
    matrix = np.eye(4)
    position = np.asarray(position, dtype=np.float64)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError('invalid_measured_position')
    matrix[:3, :3] = rotation_wxyz(quaternion_wxyz)
    matrix[:3, 3] = position
    return matrix


def validate_step(dt, vx, wz, max_linear=.3, max_angular=.5):
    for label, value in [('dt', dt), ('vx', vx), ('wz', wz), ('max_linear', max_linear), ('max_angular', max_angular)]:
        if isinstance(value, bool) or not isinstance(value, (int, float, np.number)) or not math.isfinite(value):
            raise ValueError('invalid_' + label)
    if not math.isclose(float(dt), PHYSICS_DT, rel_tol=0., abs_tol=1e-8):
        raise ValueError('spot_official_policy_requires_500Hz_physics')
    if max_linear <= 0 or max_angular <= 0 or abs(vx) > max_linear + 1e-9 or abs(wz) > max_angular + 1e-9:
        raise ValueError('spot_command_outside_fixture_contract')


def primitive_contains(points, shape, world_matrix, margin=0.):
    """Exact authored solid test (margin optionally expands only self mask)."""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('invalid_points')
    matrix = np.asarray(world_matrix, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('invalid_shape_transform')
    # Support nonuniform authored scale, rotation and translation.
    inverse = np.linalg.inv(matrix)
    local = points @ inverse[:3, :3].T + inverse[:3, 3]
    epsilon = float(margin) / float(np.linalg.svd(matrix[:3, :3], compute_uv=False).min())
    if epsilon < 0 or not math.isfinite(epsilon):
        raise ValueError('invalid_self_mask_margin')
    kind = shape['type']
    if kind == 'Cube':
        return np.all(np.abs(local) <= shape['size'] / 2 + epsilon, axis=1)
    if kind == 'Sphere':
        return np.sum(local * local, axis=1) <= (shape['radius'] + epsilon) ** 2
    axis = 'XYZ'.index(shape['axis'])
    axial = local[:, axis]
    radial = np.delete(local, axis, axis=1)
    if kind == 'Cylinder':
        return ((np.abs(axial) <= shape['height'] / 2 + epsilon)
                & (np.sum(radial * radial, axis=1) <= (shape['radius'] + epsilon) ** 2))
    if kind == 'Capsule':
        excess = np.maximum(np.abs(axial) - shape['height'] / 2, 0.)
        return (np.sum(radial * radial, axis=1) + excess ** 2 <= (shape['radius'] + epsilon) ** 2)
    raise ValueError('unsupported_authorized_collider')


def primitive_aabb(shape, world_matrix):
    """Exact transformed primitive AABB, not an occupied/free voxel proof."""
    matrix = np.asarray(world_matrix, dtype=np.float64)
    linear, center = matrix[:3, :3], matrix[:3, 3]
    kind = shape['type']
    if kind == 'Cube':
        extent = np.abs(linear).sum(axis=1) * shape['size'] / 2
    elif kind == 'Sphere':
        extent = np.linalg.norm(linear, axis=1) * shape['radius']
    else:
        axis = 'XYZ'.index(shape['axis'])
        axial = np.abs(linear[:, axis]) * shape['height'] / 2
        if kind == 'Cylinder':
            extent = axial + np.linalg.norm(np.delete(linear, axis, axis=1), axis=1) * shape['radius']
        elif kind == 'Capsule':
            extent = axial + np.linalg.norm(linear, axis=1) * shape['radius']
        else:
            raise ValueError('unsupported_authorized_collider')
    return center - extent, center + extent


def validate_initial_floor_clearance(colliders, floor_z=0.):
    """Reject actual authored solid penetration before the first physics step.

    A low spawn cannot be repaired by changing joints after collision impulses
    have already been solved. This guard never moves the asset or expands the
    permitted contact penetration of a running navigation session.
    """
    if isinstance(floor_z, bool) or not math.isfinite(float(floor_z)):
        raise ValueError('invalid_initial_floor_z')
    if not colliders:
        raise ValueError('missing_initial_collision_solids')
    minimum = float('inf')
    for collider in colliders:
        lower, _ = primitive_aabb(collider['shape'], collider['world_matrix'])
        clearance = float(lower[2]) - float(floor_z)
        minimum = min(minimum, clearance)
        if clearance < -1e-8:
            raise ValueError('official_spot_initial_floor_penetration:' + collider['path']
                             + ':depth_m=' + str(-clearance))
    return minimum


def verify_asset_manifest(manifest_path):
    manifest_path = Path(manifest_path).resolve()
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('schema') != 1 or manifest.get('source') != 'official_isaac_6_physx_spot':
        raise ValueError('invalid_spot_asset_manifest')
    paths = set()
    for entry in manifest.get('files', []):
        relative = Path(entry['path'])
        if relative.is_absolute() or '..' in relative.parts or str(relative) in paths:
            raise ValueError('unsafe_spot_asset_path')
        paths.add(str(relative))
        path = manifest_path.parent / relative
        if not path.is_file() or path.stat().st_size != entry['size_bytes'] or hashlib.sha256(path.read_bytes()).hexdigest() != entry['sha256']:
            raise ValueError('spot_asset_hash_mismatch:' + str(relative))
    if not {USD_RELATIVE, POLICY_RELATIVE, ENV_RELATIVE} <= paths:
        raise ValueError('missing_spot_required_assets')
    return manifest


class QuadrupedPlant:
    """500 Hz real joint plant. All returned poses/velocities use PhysX tensors."""
    def __init__(self, stage, config, result_dir):
        import carb
        from isaacsim.core.experimental.utils import app as app_utils
        app_utils.enable_extension('isaacsim.robot.policy.examples')
        from isaacsim.core.deprecation_manager import import_module
        from isaacsim.robot.policy.examples.controllers import PolicyController
        from isaacsim.robot.policy.examples.robots.spot import SpotFlatTerrainPolicy
        from isaacsim.core.simulation_manager import SimulationManager

        if SimulationManager.get_active_physics_engine() != 'physx':
            raise ValueError('official_spot_requires_physx')
        self.stage = stage
        self.config = config.get('robot', config)
        if self.config.get('kind', MODEL_KIND) != MODEL_KIND:
            raise ValueError('unsupported_quadruped_kind')
        self.result_dir = Path(result_dir)
        self.result_dir.mkdir(parents=True, exist_ok=True)
        root = Path(self.config.get('asset_root', os.environ.get('D1MAX_QUADRUPED_ASSET_ROOT', str(DEFAULT_ASSET_ROOT)))).resolve()
        self.asset_manifest_path = Path(self.config.get('asset_manifest_path', root / 'asset_manifest.json')).resolve()
        self.assets = verify_asset_manifest(self.asset_manifest_path)
        self.usd_path = Path(self.config.get('usd_path', root / USD_RELATIVE)).resolve()
        self.policy_path = Path(self.config.get('policy_path', root / POLICY_RELATIVE)).resolve()
        self.env_config_path = Path(self.config.get('env_config_path', root / ENV_RELATIVE)).resolve()
        authorized = {str((self.asset_manifest_path.parent / f['path']).resolve()) for f in self.assets['files']}
        if not {str(self.usd_path), str(self.policy_path), str(self.env_config_path)} <= authorized:
            raise ValueError('unsealed_spot_asset_path')
        self.root_path = self.config.get('prim_path', DEFAULT_ROOT_PATH)
        if not isinstance(self.root_path, str) or not self.root_path.startswith('/World/'):
            raise ValueError('invalid_spot_prim_path')
        self.body_path = self.root_path + BODY_SUFFIX
        self.articulation_path = self.root_path + ARTICULATION_SUFFIX
        self.max_linear_speed = float(self.config.get('max_linear_speed', .25))
        self.max_angular_speed = float(self.config.get('max_angular_speed', .4))
        validate_step(PHYSICS_DT, 0., 0., self.max_linear_speed, self.max_angular_speed)
        position = self.config.get('initial_position', [0., 0., .8])
        yaw = float(self.config.get('initial_yaw', 0.))
        orientation = [math.cos(yaw / 2), 0., 0., math.sin(yaw / 2)]
        pose_matrix(position, orientation)  # finite authoring arguments

        class LocalOnlySpot(SpotFlatTerrainPolicy):
            def __init__(inner):
                # Stock ctor performs get_assets_root_path() even with explicit
                # paths. Only constructor resolution is bypassed; policy math,
                # initialization and every forward action remain official.
                PolicyController.__init__(inner, self.root_path, self.articulation_path,
                                          str(self.usd_path), position, orientation)
                # Assets without a multi-engine variant bypass this in the
                # official base constructor. Ensure its authored PhysX solver
                # settings have their schema before physics starts.
                inner._ensure_physx_articulation_api()
                inner.load_policy(str(self.policy_path), str(self.env_config_path))
                inner._action_scale = inner.policy_env_params.get('action_scale', .2)
                inner._previous_action = inner._current_action = None
                inner._policy_counter = 0

        self.controller = LocalOnlySpot()
        if not math.isclose(self.controller._dt, PHYSICS_DT, abs_tol=1e-12) or self.controller._decimation != POLICY_DECIMATION:
            raise ValueError('unexpected_spot_policy_timing')
        self.robot = self.controller.robot
        self._torch = import_module('torch')
        self._command = self._torch.zeros(3, dtype=self._torch.float32, device=str(self.robot._device))
        self.velocity_feedback_enabled = self.config.get('velocity_feedback', True)
        if type(self.velocity_feedback_enabled) is not bool:
            raise ValueError('velocity_feedback_must_be_explicit_bool')
        self.velocity_feedback = VelocityFeedback(mode=self.config.get('velocity_feedback_mode', 'legacy_pi_v1'))
        self._initialized = False
        self._steps = 0
        self._registry = self._read_registry()
        if len(self._registry) != 13:
            raise ValueError('spot_collider_contract_mismatch')
        self._registry_digest = hashlib.sha256(canonical(self._registry)).hexdigest()
        from pxr import UsdGeom
        cache = UsdGeom.XformCache()
        initial_colliders = [dict(path=record['path'], shape=record['shape'],
            world_matrix=np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(record['path']))).T)
            for record in self._registry]
        self.initial_floor_z = float(self.config.get('floor_z', config.get('floor', {}).get('z', 0.)))
        self.initial_collision_floor_clearance_m = validate_initial_floor_clearance(initial_colliders, self.initial_floor_z)
        # Wrap links before play; authoring rigid APIs while tensor views are
        # active can invalidate PhysX views (including disabled visual shapes).
        from isaacsim.core.experimental.prims import RigidPrim
        self._link_paths = sorted({r['rigid_body_path'] for r in self._registry})
        self._links = RigidPrim(self._link_paths, reset_xform_op_properties=False)
        self._link_index = {str(path): i for i, path in enumerate(self._links.paths)}

    def initialize(self):
        """Call once when the physics tensor view is ready, before settle ticks."""
        from isaacsim.core.experimental.utils import backend
        if self._initialized:
            raise ValueError('spot_already_initialized')
        if not self.robot.is_physics_tensor_entity_valid():
            raise RuntimeError('spot_physics_not_ready')
        with backend.use_backend('tensor', raise_on_unsupported=True, raise_on_fallback=True):
            self.controller.initialize()
        if self.robot.num_dofs != 12:
            raise ValueError('spot_dof_contract_mismatch')
        self.verify_collider_registry()
        if not self._links.is_physics_tensor_entity_valid():
            raise RuntimeError('spot_link_physics_not_ready')
        native_paths = [str(p) for p in self._links._physics_rigid_body_view.prim_paths]
        if set(native_paths) != set(self._link_paths) or len(native_paths) != len(self._link_paths):
            raise RuntimeError('spot_native_link_identity_mismatch')
        self._link_index = {path: i for i, path in enumerate(native_paths)}
        self._initialized = True
        self.reset()
        with backend.use_backend('tensor', raise_on_unsupported=True, raise_on_fallback=True):
            gains = self.robot.get_dof_gains()
            self._actuator_readback = dict(
                dof_names=list(self.robot.dof_names),
                stiffness=array(gains[0]).tolist(), damping=array(gains[1]).tolist(),
                max_efforts=array(self.robot.get_dof_max_efforts()).tolist(),
                max_velocities=array(self.robot.get_dof_max_velocities()).tolist())
        canonical(self._actuator_readback)  # reject nonfinite startup properties
        (self.result_dir / 'quadruped_asset_metadata.json').write_text(json.dumps(self.asset_metadata(), indent=2) + '\n')

    def reset(self):
        """Reset policy memory only; live body/joints are never teleported."""
        if not self._initialized:
            raise RuntimeError('spot_not_initialized')
        self._command.zero_()
        self.controller._previous_action = self.controller._current_action = None
        self.controller._policy_counter = 0
        self.velocity_feedback.reset()

    def step(self, dt, vx, wz):
        from isaacsim.core.experimental.utils import backend
        if not self._initialized:
            raise RuntimeError('spot_not_initialized')
        validate_step(dt, vx, wz, self.max_linear_speed, self.max_angular_speed)
        with backend.use_backend('tensor', raise_on_unsupported=True, raise_on_fallback=True):
            policy_command = np.asarray([vx, wz], dtype=float)
            if self.velocity_feedback_enabled and self.controller._policy_counter % POLICY_DECIMATION == 0:
                _, orientations = self.get_world_poses()
                linear, angular = [array(v)[0] for v in self.get_velocities()]
                inverse_rotation = rotation_wxyz(array(orientations)[0]).T
                linear_body, angular_body = inverse_rotation @ linear, inverse_rotation @ angular
                policy_command = self.velocity_feedback.update(PHYSICS_DT * POLICY_DECIMATION,
                    [vx, wz], [linear_body[0], angular_body[2]])
                self._command[0], self._command[1], self._command[2] = float(policy_command[0]), 0., float(policy_command[1])
            elif not self.velocity_feedback_enabled:
                self._command[0], self._command[1], self._command[2] = float(vx), 0., float(wz)
            self.controller.forward(float(dt), self._command)
        self._steps += 1

    def _measured(self, method):
        from isaacsim.core.experimental.utils import backend
        if not self._initialized or not self.robot.is_physics_tensor_entity_valid():
            raise RuntimeError('spot_physics_not_ready')
        with backend.use_backend('tensor', raise_on_unsupported=True, raise_on_fallback=True):
            return getattr(self.robot, method)()

    def get_world_poses(self):
        return self._measured('get_world_poses')

    def get_velocities(self):
        return self._measured('get_velocities')

    def get_dof_velocities(self):
        return self._measured('get_dof_velocities')

    def get_dof_positions(self):
        return self._measured('get_dof_positions')

    def _read_registry(self):
        from pxr import Usd, UsdGeom, UsdPhysics
        root = self.stage.GetPrimAtPath(self.root_path)
        if not root.IsValid():
            raise ValueError('missing_spot_root')
        records = []
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
                continue
            kind = prim.GetTypeName()
            if kind not in ('Cube', 'Sphere', 'Cylinder', 'Capsule'):
                raise ValueError('unsupported_spot_enabled_collider:' + str(prim.GetPath()))
            body = prim
            while body.IsValid() and not body.HasAPI(UsdPhysics.RigidBodyAPI):
                body = body.GetParent()
            if not body.IsValid() or not str(body.GetPath()).startswith(self.root_path + '/'):
                raise ValueError('unowned_spot_collider')
            # Compose only fixed descendants up to this rigid link. Taking two
            # world matrices then cancelling introduces roundoff dependent on
            # each measured articulated pose and would corrupt stable identity.
            relative = np.eye(4)
            current = prim
            while current != body:
                xf = UsdGeom.Xformable(current)
                if xf.GetResetXformStack():
                    raise ValueError('spot_collider_resets_rigid_body_transform')
                relative = np.asarray(xf.GetLocalTransformation()).T @ relative
                current = current.GetParent()
            shape = {'type': kind}
            for name in ('size',) if kind == 'Cube' else (('radius',) if kind == 'Sphere' else ('radius', 'height', 'axis')):
                value = prim.GetAttribute(name).Get()
                shape[name] = str(value) if name == 'axis' else float(value)
            if any(not math.isfinite(v) or v <= 0 for k, v in shape.items() if isinstance(v, float)):
                raise ValueError('invalid_spot_shape')
            records.append(dict(path=str(prim.GetPath()), rigid_body_path=str(body.GetPath()),
                                collision_enabled=True, shape=shape,
                                local_matrix=np.round(relative, 12).tolist()))
        return sorted(records, key=lambda r: r['path'])

    def authorized_collider_identities(self):
        if not self._initialized:
            raise RuntimeError('spot_not_initialized')
        # Copy prevents a consumer from mutating the authorization registry.
        return json.loads(canonical(self._registry))

    def verify_collider_registry(self):
        if hashlib.sha256(canonical(self._read_registry())).hexdigest() != self._registry_digest:
            raise ValueError('spot_collider_registry_changed')
        return self._registry_digest

    def collider_snapshot(self, *, verify=True):
        """Actual PhysX link transforms + fixed exact local shape, no USD pose."""
        from isaacsim.core.experimental.utils import backend
        if not self._initialized or not self._links.is_physics_tensor_entity_valid():
            raise RuntimeError('spot_link_physics_not_ready')
        if verify:
            self.verify_collider_registry()
        with backend.use_backend('tensor', raise_on_unsupported=True, raise_on_fallback=True):
            positions, orientations = [array(v) for v in self._links.get_world_poses()]
        matrices = {path: pose_matrix(positions[i], orientations[i]) for path, i in self._link_index.items()}
        result = []
        for record in self._registry:
            matrix = matrices[record['rigid_body_path']] @ np.asarray(record['local_matrix'])
            lower, upper = primitive_aabb(record['shape'], matrix)
            result.append(dict(path=record['path'], rigid_body_path=record['rigid_body_path'], shape=dict(record['shape']),
                               world_matrix=matrix.tolist(), aabb_min=lower.tolist(), aabb_max=upper.tolist()))
        return result

    def external_hit_mask(self, points_body, *, measured_snapshot=None,
                          body_position=None, body_orientation_wxyz=None):
        """Filter exact self solids using one actual acquisition phase.

        A saved snapshot and its measured body pose must be supplied together
        when LiDAR is acquired before a physics step. That path reads no later
        PhysX poses: unchanged body points are transformed with the saved pose
        and tested against the saved link matrices. Identity remains the exact
        immutable enrolled collider list, rather than a robot-path prefix.
        Omitting all three keywords preserves the current-measurement API.
        """
        points = np.asarray(points_body, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('invalid_points')
        supplied = [measured_snapshot is not None, body_position is not None,
                    body_orientation_wxyz is not None]
        if any(supplied) and not all(supplied):
            raise ValueError('self_filter_requires_same_phase_snapshot_and_body_pose')
        if not any(supplied):
            body_position, body_orientation_wxyz = [array(v)[0] for v in self.get_world_poses()]
            measured_snapshot = self.collider_snapshot()
        position = np.asarray(body_position, dtype=np.float64)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('invalid_self_filter_body_position')
        world = points @ rotation_wxyz(body_orientation_wxyz).T + position
        if (hashlib.sha256(canonical(self._registry)).hexdigest() != self._registry_digest
                or not isinstance(measured_snapshot, (list, tuple))
                or len(measured_snapshot) != len(self._registry)):
            raise ValueError('self_filter_incomplete_or_changed_collider_registry')
        enrolled = {record['path']: record for record in self._registry}
        if len(enrolled) != len(self._registry):
            raise ValueError('self_filter_duplicate_enrolled_collider')
        inside = np.zeros(len(points), dtype=bool)
        seen = set()
        for collider in measured_snapshot:
            path = collider.get('path')
            record = enrolled.get(path)
            if (record is None or path in seen
                    or collider.get('rigid_body_path') != record['rigid_body_path']
                    or canonical(collider.get('shape')) != canonical(record['shape'])):
                raise ValueError('self_filter_foreign_duplicate_or_changed_collider')
            seen.add(path)
            matrix = np.asarray(collider['world_matrix'], dtype=np.float64)
            if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                    or not np.array_equal(matrix[3], [0., 0., 0., 1.])):
                raise ValueError('invalid_self_filter_world_matrix')
            # Measured link motion is rigid. Its transform cannot change the
            # fixed collider scale/shear metric and enlarge the self exclusion.
            local = np.asarray(record['local_matrix'], dtype=np.float64)[:3, :3]
            linear = matrix[:3, :3]
            determinant, local_determinant = np.linalg.det(linear), np.linalg.det(local)
            if (not math.isfinite(determinant) or abs(determinant) < 1e-12
                    or determinant * local_determinant <= 0.
                    or not np.allclose(linear.T @ linear, local.T @ local,
                                       rtol=1e-9, atol=1e-12)):
                raise ValueError('self_filter_changed_collider_transform_metric')
            inside |= primitive_contains(world, collider['shape'], matrix, margin=.002)
        return ~inside

    def asset_metadata(self):
        return dict(schema=1, kind='official_spot_physx', source='NVIDIA Isaac Sim 6.0 matching PhysX Spot model/policy/env',
                    root_path=self.root_path, body_path=self.body_path, articulation_path=self.articulation_path, physics_dt_s=PHYSICS_DT,
                    physics_frequency_hz=500, policy_decimation=POLICY_DECIMATION, policy_frequency_hz=50,
                    max_linear_speed_mps=self.max_linear_speed, max_angular_speed_radps=self.max_angular_speed,
                    velocity_feedback=dict(enabled=self.velocity_feedback_enabled,
                        mode=self.velocity_feedback.mode,
                        type=('bounded_measured_body_velocity_PI_v1' if self.velocity_feedback.mode == 'legacy_pi_v1'
                              else 'bounded_smooth_nonlinear_measured_body_velocity_PI_v1'),
                        policy_command_limits=self.velocity_feedback.limits.tolist(), kp=self.velocity_feedback.kp.tolist(),
                        ki=self.velocity_feedback.ki.tolist(), integral_limit=self.velocity_feedback.integral_limit.tolist(),
                        measurement_lowpass_time_constant_s=.05,
                        nonlinear_linear_feedforward=(dict(amplitude_mps=.25, transition_mps=.005, fade_mps=.15,
                            formula='.25*tanh(vx_des/.005)*exp(-abs(vx_des)/.15)', angular_feedforward=0.)
                            if self.velocity_feedback.mode == 'spot_nonlinear_measured_v1' else None),
                        zero_linear_demand=(dict(history_decay_time_constant_s=.5,
                            output='kp_linear*(zero_desired-measured_filtered_vx); forward feedforward and integral excluded',
                            update_order='measured_filter,error,integral_proposal,antiwindup,history_decay,output_withdrawal')
                            if self.velocity_feedback.mode == 'spot_nonlinear_measured_v1' else None)),
                    usd_path=str(self.usd_path), policy_path=str(self.policy_path), env_config_path=str(self.env_config_path),
                    asset_manifest_path=str(self.asset_manifest_path),
                    asset_manifest_sha256=hashlib.sha256(self.asset_manifest_path.read_bytes()).hexdigest(),
                    assets=self.assets['files'], collider_registry_sha256=self._registry_digest,
                    authorized_colliders=self._registry, policy_steps=self._steps,
                    actuator_tensor_readback=getattr(self, '_actuator_readback', None),
                    initial_floor_z_m=self.initial_floor_z,
                    initial_collision_floor_clearance_m=self.initial_collision_floor_clearance_m,
                    measured_state='PhysX_tensor_no_USD_fallback', motor_model='official_implicit_PD_deployment_not_DelayedPD_or_RemotizedPD_equivalence',
                    source_url='https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_simulation/ext_isaacsim_robot_policy_example.html')
