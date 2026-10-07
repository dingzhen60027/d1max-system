"""Pure bounded per-ray motion projection. No ROS, TF lookup, SDK or I/O.

One scan uses one exact-pair map_from_odom correction; only LOCAL tracking
motion is interpolated per point. Endpoint and its sensor origin share the
same transform. There is no ground removal or invented free-space evidence.
"""
from collections import deque
from dataclasses import dataclass, fields as dataclass_fields, replace
import math

import numpy as np

from d1max_localization.math_utils import Pose3, compose, inverse, pose_innovation
from .perception_contract import MAX_ACQUISITION_POINTS, acquisition_budget


FIELDS = (
    ('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1), ('intensity', 12, 7, 1),
    ('origin_x', 16, 7, 1), ('origin_y', 20, 7, 1), ('origin_z', 24, 7, 1),
    ('sensor_id', 28, 4, 1), ('ring', 30, 4, 1), ('offset_time', 32, 6, 1),
    ('source_index', 36, 6, 1), ('timestamp', 40, 8, 1),
    ('source_timestamp', 48, 8, 1), ('raw_timestamp', 56, 8, 1))
RAY_DTYPE = np.dtype({'names': [x[0] for x in FIELDS],
                     'formats': ['<f4'] * 7 + ['<u2', '<u2', '<u4', '<u4'] + ['<f8'] * 3,
                     'offsets': [x[1] for x in FIELDS], 'itemsize': 64})
# Opt-in isolated native-hit identity. The original 64 bytes retain exactly
# the same acquisition fields, offsets and types. Zero is never actor proof.
ISAAC_FIELDS = FIELDS + (('isaac_actor_id', 64, 4, 1),)
ISAAC_RAY_DTYPE = np.dtype({'names': list(RAY_DTYPE.names) + ['isaac_actor_id'],
    'formats': [RAY_DTYPE.fields[name][0] for name in RAY_DTYPE.names] + ['<u2'],
    'offsets': [RAY_DTYPE.fields[name][1] for name in RAY_DTYPE.names] + [64], 'itemsize': 72})


class ProjectionError(ValueError):
    """Invalid observation: drop it, never use it to clear occupancy."""


class AwaitingCoverage(ProjectionError):
    """An input may wait only until its original bounded receive deadline."""


class PreviewExcludedScan(ProjectionError):
    """Diagnostic-only outcome: no observation may renew a sensor lease."""
    def __init__(self, count, digest):
        super().__init__('preview_exclusion_removed_entire_scan')
        self.count, self.digest = count, digest


@dataclass(frozen=True)
class MapContext:
    session_id: str
    epoch: int
    seed_id: str
    sequence: int
    barrier_ns: int

    @classmethod
    def parse(cls, value):
        if (not isinstance(value, dict) or type(value.get('schema')) is not int or value.get('schema') != 1
                or type(value.get('session_id')) is not str or not 1 <= len(value['session_id']) <= 128
                or type(value.get('seed_id')) is not str or not 1 <= len(value['seed_id']) <= 128
                or any(type(value.get(key)) is not int or not 1 <= value[key] < 2**64
                       for key in ('epoch', 'sequence'))
                or type(value.get('barrier_ns')) is not int or not 0 <= value['barrier_ns'] < 2**63):
            raise ProjectionError('invalid_map_context')
        return cls(value['session_id'], value['epoch'], value['seed_id'],
                   value['sequence'], value['barrier_ns'])


@dataclass(frozen=True)
class Limits:
    history_sec: float = 2.0
    max_history_samples: int = 256
    max_input_points: int = MAX_ACQUISITION_POINTS
    max_pose_gap_sec: float = .06
    input_timeout_sec: float = .5
    max_scan_duration_sec: float = .15
    alignment_wait_sec: float = .10
    max_alignment_translation_step_m: float = .20
    max_alignment_rotation_step_rad: float = .10
    # Independent software rebuild triggers, not physical clearance guarantees.
    # The native map is not reprojected when map_from_odom changes; comparing
    # only adjacent updates permits an unbounded accumulated history mismatch.
    max_alignment_translation_from_anchor_m: float = .10
    max_alignment_rotation_from_anchor_rad: float = .05

    def __post_init__(self):
        if (not .5 <= self.history_sec <= 3.0
                or type(self.max_history_samples) is not int or not 20 <= self.max_history_samples <= 512
                or type(self.max_input_points) is not int or not 1 <= self.max_input_points <= MAX_ACQUISITION_POINTS
                or not .02 <= self.max_pose_gap_sec <= .10
                or not .1 <= self.input_timeout_sec <= .5
                or not .01 <= self.max_scan_duration_sec <= .15
                or not .02 <= self.alignment_wait_sec <= .15
                or not .01 <= self.max_alignment_translation_step_m <= .25
                or not .01 <= self.max_alignment_rotation_step_rad <= .20
                or not .001 <= self.max_alignment_translation_from_anchor_m <= .10
                or not .001 <= self.max_alignment_rotation_from_anchor_rad <= .05):
            raise ValueError('invalid_projection_limits')


@dataclass(frozen=True)
class RawRays:
    points: np.ndarray
    start_ns: int
    end_ns: int
    sensor_id: int


@dataclass(frozen=True)
class Projected:
    points: np.ndarray
    start_ns: int
    end_ns: int
    alignment_ns: int
    sensor_id: int
    context: MapContext
    sequence: int
    exclusion_digest: str = ''
    exclusion_matched: int = 0
    exclusion_dropped: int = 0
    projection_frame: str = 'map'


def session_settings(session, localization):
    """Explicit session opt-in only; do not modify the shared LIO profile."""
    snapshot_contract = session.get('perception_acquisition')
    if snapshot_contract is not None and (
            snapshot_contract != 'isaac_physx_snapshot_v1'
            or session.get('transport_mode') != 'isolated_mock'
            or session.get('simulation_backend') != 'isaacsim_physx'
            or session.get('simulation_clock') != 'isaac_fixed_anchor_v1'
            or session.get('perception_backend') != 'per_sensor_rays'):
        raise ProjectionError('snapshot_acquisition_requires_isolated_isaac_session')
    if 'preview_ray_exclusion' in session and session.get('perception_backend') != 'per_sensor_rays':
        raise ProjectionError('preview_exclusion_requires_per_sensor_rays')
    if session.get('perception_backend', 'deskewed_cloud') != 'per_sensor_rays':
        return {'enabled': False}
    identifier = session.get('id')
    if not isinstance(identifier, str) or not 1 <= len(identifier) <= 128:
        raise ProjectionError('invalid_projector_session')
    front = localization.get('lio_localizer', {}).get('ros__parameters', {})
    nav = localization.get('navigation_estimation', {}).get('ros__parameters', {})
    adapter = localization.get('dual_lidar_adapter', {}).get('ros__parameters', {})
    ray_config = adapter.get('perception_rays', {})
    enabled = adapter.get('perception_rays.enabled', ray_config.get('enabled', False))
    if enabled is not True or nav.get('enabled') is not True:
        raise ProjectionError('projector_requires_explicit_rays_and_navigation_output')
    result = dict(enabled=True, session_id=identifier, map_frame=front.get('map_frame'),
                  odom_frame=front.get('odom_frame'), tracking_frame=front.get('tracking_frame'),
                  body_frame=nav.get('body_frame', 'd1max_loc_base_link'),
                  ray_frame=adapter.get('target_frame'),
                  raw_topic=adapter.get('perception_rays.output_topic',
                      ray_config.get('output_topic', '/d1max/localization/perception/rays_raw')),
                  output_topic='/d1max/live_planning/rays_map')
    local_frame = session.get('local_planning_frame', result['map_frame'])
    if local_frame not in (result['map_frame'], result['odom_frame']):
        raise ProjectionError('unsupported_local_planning_frame')
    result['projection_frame'] = 'odom' if local_frame == result['odom_frame'] else 'map'
    result['output_frame'] = local_frame
    if result['projection_frame'] == 'odom':
        result['output_topic'] = '/d1max/live_planning/rays_odom'
    result['typed_navigation_state'] = session.get('pipeline_contract') == 'single_floor_v3'
    result['independent_local_state'] = session.get('local_state_contract') == 'continuous_odom_v1'
    result['map_version_id'] = session.get('version_id', '')
    if result['typed_navigation_state'] and (result['projection_frame'] != 'odom'
            or not result['map_version_id'] or 'preview_ray_exclusion' in session):
        raise ProjectionError('single_floor_requires_odom_atomic_state_without_preview_exclusion')
    if (result['map_frame'] != session.get('frame_id')
            or any(not isinstance(result[key], str) or not result[key]
                   for key in ('map_frame', 'odom_frame', 'tracking_frame', 'body_frame', 'ray_frame'))
            or len({result[key] for key in ('map_frame', 'odom_frame', 'tracking_frame', 'body_frame')}) != 4
            or result['ray_frame'] in (result['map_frame'], result['odom_frame'], result['body_frame'])):
        raise ProjectionError('inconsistent_projector_frames')
    options = session.get('perception_projector', {})
    if not isinstance(options, dict) or set(options) - {f.name for f in dataclass_fields(Limits)}:
        raise ProjectionError('unknown_projector_limit')
    result['limits'] = Limits(**options)
    result['allow_simulation_snapshot'] = snapshot_contract == 'isaac_physx_snapshot_v1'
    result['maximum_isaac_actor_id'] = 0
    dynamic = session.get('dynamic_oracle_contract', {})
    if 'native_hit_provenance' in dynamic:
        proof = dynamic['native_hit_provenance']
        actors = dynamic.get('actor_ids')
        digest = dynamic.get('registry_sha256')
        if (not result['allow_simulation_snapshot'] or not isinstance(proof, dict)
                or proof != dict(schema=1, kind='physx_exact_hit_prim_ordinal_v1', point_step=72,
                    registry_sha256=digest) or not isinstance(digest, str) or len(digest) != 64
                or any(c not in '0123456789abcdef' for c in digest)
                or not isinstance(actors, list) or len(actors) > 64
                or any(not isinstance(name, str) or not name for name in actors)
                or actors != sorted(set(actors))):
            raise ProjectionError('native_hit_provenance_requires_sealed_isaac_registry')
        result['maximum_isaac_actor_id'] = len(actors)
    result['acquisition_contract'] = acquisition_budget(adapter, result['limits'].max_input_points)
    mode = adapter.get('lidar_mode', 'dual')
    if mode not in ('dual', 'front', 'rear'):
        raise ProjectionError('invalid_projector_sensor_selection')
    result['sensor_ids'] = (0, 1) if mode == 'dual' else (0,) if mode == 'front' else (1,)
    from .preview_ray_exclusion import from_session
    result['preview_exclusion'] = from_session(session, result['body_frame'])
    return result


def checked_pose(position, orientation):
    p, q = np.asarray(position, dtype=float), np.asarray(orientation, dtype=float)
    if (p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all()
            or not np.isfinite(q).all() or np.max(np.abs(p)) > 1e6
            or abs(float(np.linalg.norm(q)) - 1.) > .01):
        raise ProjectionError('invalid_pose_geometry')
    q /= np.linalg.norm(q)
    return Pose3(tuple(p), tuple(q))


def decode_rays(*, data, fields, point_step, row_step, width, height, bigendian,
                header_ns, frame_id, expected_frame, max_points=MAX_ACQUISITION_POINTS,
                maximum_isaac_actor_id=0):
    if type(max_points) is not int or not 1 <= max_points <= MAX_ACQUISITION_POINTS:
        raise ProjectionError('invalid_acquisition_point_budget')
    if type(maximum_isaac_actor_id) is not int or not 0 <= maximum_isaac_actor_id <= 64:
        raise ProjectionError('invalid_native_hit_actor_domain')
    native = point_step == 72 and maximum_isaac_actor_id > 0
    expected_fields, dtype = (ISAAC_FIELDS, ISAAC_RAY_DTYPE) if native else (FIELDS, RAY_DTYPE)
    if (type(header_ns) is not int or not 0 < header_ns < 2**63
            or frame_id != expected_frame or bigendian
            or type(width) is not int or not 1 <= width <= max_points or height != 1
            or point_step != dtype.itemsize or row_step != width * dtype.itemsize or len(data) != row_step
            or len(fields) != len(expected_fields) or set(fields) != set(expected_fields)):
        raise ProjectionError('invalid_raw_ray_schema_or_frame')
    points = np.frombuffer(data, dtype=dtype, count=width)
    if native and np.any(points['isaac_actor_id'] > maximum_isaac_actor_id):
        raise ProjectionError('native_hit_actor_outside_registry')
    # Do not reinterpret invalid endpoints/times as max-range clearing rays.
    for name in ('x', 'y', 'z', 'origin_x', 'origin_y', 'origin_z',
                 'timestamp', 'source_timestamp', 'raw_timestamp'):
        if not np.isfinite(points[name]).all():
            raise ProjectionError('nonfinite_raw_ray')
    sensor = int(points['sensor_id'][0])
    if sensor not in (0, 1) or np.any(points['sensor_id'] != sensor):
        raise ProjectionError('mixed_or_invalid_sensor_identity')
    if len(points) > 1 and np.any(np.diff(points['source_index'].astype(np.int64)) <= 0):
        raise ProjectionError('nonmonotonic_source_indices')
    offsets = points['offset_time'].astype(np.int64)
    source = points['source_timestamp']
    mapped = points['timestamp']
    if (int(offsets.min()) != 0 or int(offsets.max()) > 150000000
            or np.min(mapped) <= 0 or np.max(mapped) >= 2147483647.
            or np.max(np.abs((mapped - source) - (mapped[0] - source[0]))) > 1e-6
            or np.max(np.abs((source - np.min(source)) * 1e9 - offsets)) > 1000.
            or np.max(np.abs(mapped * 1e9 - (header_ns + offsets))) > 1000.):
        raise ProjectionError('inconsistent_ray_timestamps')
    # The raw branch's origin is a fixed configured extrinsic per source.
    for name in ('origin_x', 'origin_y', 'origin_z'):
        if np.max(np.abs(points[name] - points[name][0])) > 1e-6:
            raise ProjectionError('raw_origin_not_fixed_extrinsic')
    return RawRays(points, header_ns, header_ns + int(offsets.max()), sensor)


def rotate_many(quaternions, vectors):
    """Unit quaternion rotation, vectorized without allocating N 3x3 matrices."""
    q = np.asarray(quaternions, dtype=float)
    vectors = np.asarray(vectors, dtype=float)
    t = 2. * np.cross(q[..., :3], vectors)
    return vectors + q[..., 3:4] * t + np.cross(q[..., :3], t)


def transform_many(pose, points):
    return rotate_many(np.asarray(pose.orientation), points) + np.asarray(pose.position)


def rotation_matrices(quaternions):
    q = np.asarray(quaternions, dtype=float)
    x, y, z, w = [q[..., i] for i in range(4)]
    output = np.empty(q.shape[:-1]+(3, 3), dtype=float)
    output[..., 0, 0] = 1-2*(y*y+z*z)
    output[..., 0, 1] = 2*(x*y-z*w)
    output[..., 0, 2] = 2*(x*z+y*w)
    output[..., 1, 0] = 2*(x*y+z*w)
    output[..., 1, 1] = 1-2*(x*x+z*z)
    output[..., 1, 2] = 2*(y*z-x*w)
    output[..., 2, 0] = 2*(x*z-y*w)
    output[..., 2, 1] = 2*(y*z+x*w)
    output[..., 2, 2] = 1-2*(x*x+y*y)
    return output


def acquisition_groups(offsets):
    """Exact same-instant groups, without sorting an already ordered scan."""
    offsets = np.asarray(offsets)
    if len(offsets) and np.all(offsets[1:] >= offsets[:-1]):
        first = np.empty(len(offsets), dtype=bool)
        first[0] = True
        first[1:] = offsets[1:] != offsets[:-1]
        return offsets[first], np.cumsum(first, dtype=np.int64)-1
    # Acquisition order is not a schema requirement. Never sort the points or
    # quantize acquisition instants just to take the fast path.
    return np.unique(offsets, return_inverse=True)


def transform_indexed_endpoints(matrices, indices, points):
    """Same three-term dot products without an N x 3 x 3 matrix expansion."""
    output = np.empty((len(points), 3), dtype=float)
    scratch = np.empty(len(points), dtype=float)
    for axis in range(3):
        np.take(matrices[:, axis, 0], indices, out=scratch)
        np.multiply(scratch, points['x'], out=output[:, axis])
        for column, name in ((1, 'y'), (2, 'z')):
            np.take(matrices[:, axis, column], indices, out=scratch)
            np.multiply(scratch, points[name], out=scratch)
            np.add(output[:, axis], scratch, out=output[:, axis])
    return output


def interpolate_many(history, times_ns, max_gap_ns):
    """Linear XYZ + shortest-arc SLERP, no extrapolation or nearest-pose fallback."""
    if len(history) < 2:
        raise AwaitingCoverage('waiting_local_pose_history')
    stamps = np.asarray([value[0] for value in history], dtype=np.int64)
    if np.any(np.diff(stamps) <= 0):
        raise ProjectionError('nonmonotonic_pose_history')
    if np.min(times_ns) < stamps[0]:
        raise ProjectionError('ray_before_pose_history')
    if np.max(times_ns) > stamps[-1]:
        raise AwaitingCoverage('waiting_local_pose_coverage')
    hi = np.searchsorted(stamps, times_ns, side='left')
    hi = np.minimum(hi, len(stamps) - 1)
    exact = stamps[hi] == times_ns
    lo = np.where(exact, hi, np.maximum(0, hi - 1))
    gaps = stamps[hi] - stamps[lo]
    if np.any(gaps > max_gap_ns):
        raise ProjectionError('local_pose_bracketing_gap')
    alpha = np.divide(times_ns - stamps[lo], gaps, out=np.zeros(len(times_ns)), where=gaps > 0)
    positions = np.asarray([pose.position for _, pose in history], dtype=float)
    quaternions = np.asarray([pose.orientation for _, pose in history], dtype=float)
    p = positions[lo] + alpha[:, None] * (positions[hi] - positions[lo])
    q0, q1 = quaternions[lo], quaternions[hi].copy()
    dot = np.sum(q0 * q1, axis=1)
    q1[dot < 0] *= -1
    dot = np.clip(np.abs(dot), -1., 1.)
    q = q0 + alpha[:, None] * (q1 - q0)
    spherical = dot <= .9995
    angle = np.arccos(dot[spherical])
    if angle.size:
        a = alpha[spherical]
        q[spherical] = ((np.sin((1-a)*angle)[:, None] * q0[spherical]
                         + np.sin(a*angle)[:, None] * q1[spherical]) / np.sin(angle)[:, None])
    q /= np.linalg.norm(q, axis=1)[:, None]
    return p, q


class RayProjectorCore:
    def __init__(self, limits=Limits(), preview_exclusion=None, *, projection_frame='map',
                 allow_simulation_snapshot=False, maximum_isaac_actor_id=0):
        if projection_frame not in ('map', 'odom'):
            raise ValueError('projection_frame must explicitly be map or odom')
        self.limits = limits
        self.projection_frame = projection_frame
        self.preview_exclusion = preview_exclusion
        if type(allow_simulation_snapshot) is not bool:
            raise ValueError('simulation_snapshot_flag_must_be_boolean')
        self.allow_simulation_snapshot = allow_simulation_snapshot
        if (type(maximum_isaac_actor_id) is not int or not 0 <= maximum_isaac_actor_id <= 64
                or maximum_isaac_actor_id and not allow_simulation_snapshot):
            raise ValueError('native_hit_provenance_requires_isolated_snapshot')
        self.maximum_isaac_actor_id = maximum_isaac_actor_id
        self.context = None
        self.local = deque(maxlen=limits.max_history_samples)
        self.global_pending = {}
        self.alignments = deque(maxlen=limits.max_history_samples)
        self.alignment_anchor = None
        self.alignment_displacement = None
        self.body_to_tracking = self.ray_to_tracking = None
        self.sequence = 0
        self.last_input = [-1, -1]
        self.last_local_ns = self.last_global_ns = 0
        self.fault = None

    def reset(self, context):
        if context == self.context:
            return
        self.context = context
        self.local.clear()
        self.global_pending.clear()
        self.alignments.clear()
        self.alignment_anchor = None
        self.alignment_displacement = None
        self.last_local_ns = self.last_global_ns = 0
        self.last_input = [-1, -1]
        self.fault = None
        self.body_to_tracking = self.ray_to_tracking = None
        # sequence never rewinds within a projector process.

    def set_extrinsics(self, *, body_to_tracking, ray_to_tracking):
        body = checked_pose(body_to_tracking.position, body_to_tracking.orientation)
        ray = checked_pose(ray_to_tracking.position, ray_to_tracking.orientation)
        if self.body_to_tracking is not None and self.context is not None:
            for old, new in ((self.body_to_tracking, body), (self.ray_to_tracking, ray)):
                innovation = pose_innovation(old, new)
                if (max(innovation.translation_xy, innovation.translation_z) > 1e-6
                        or innovation.rotation > 1e-6):
                    self.fault = 'static_extrinsic_changed_requires_context_reset'
                    raise ProjectionError(self.fault)
        self.body_to_tracking, self.ray_to_tracking = body, ray

    def projection_snapshot(self):
        """Small immutable-value history copy; geometry may run off the ROS thread."""
        snapshot = RayProjectorCore(self.limits, self.preview_exclusion,
                                    projection_frame=self.projection_frame,
                                    allow_simulation_snapshot=self.allow_simulation_snapshot,
                                    maximum_isaac_actor_id=self.maximum_isaac_actor_id)
        snapshot.context = self.context
        snapshot.local = deque(self.local, maxlen=self.limits.max_history_samples)
        snapshot.alignments = deque(self.alignments, maxlen=self.limits.max_history_samples)
        snapshot.alignment_anchor = self.alignment_anchor
        snapshot.alignment_displacement = self.alignment_displacement
        snapshot.body_to_tracking, snapshot.ray_to_tracking = self.body_to_tracking, self.ray_to_tracking
        snapshot.sequence = self.sequence
        snapshot.last_input = list(self.last_input)
        snapshot.fault = self.fault
        return snapshot

    def commit_projection(self, projected):
        """Main-thread commit only after current context/time/status are rechecked."""
        if (self.context is None or projected.context != self.context or self.fault
                or projected.projection_frame != self.projection_frame
                or projected.start_ns <= self.last_input[projected.sensor_id]
                or projected.exclusion_digest != (self.preview_exclusion.digest
                                                  if self.preview_exclusion else '')):
            raise ProjectionError('projection_context_changed_or_already_committed')
        self.sequence += 1
        self.last_input[projected.sensor_id] = projected.start_ns
        return replace(projected, sequence=self.sequence)

    def _admit_pose(self, stamp_ns, context):
        if (self.context is None or context != self.context or self.fault
                or self.body_to_tracking is None or self.ray_to_tracking is None
                or type(stamp_ns) is not int or stamp_ns <= context.barrier_ns):
            raise ProjectionError(self.fault or 'pose_context_or_extrinsic_unavailable')

    def add_local_body(self, stamp_ns, body_pose, context):
        self._admit_pose(stamp_ns, context)
        if stamp_ns <= self.last_local_ns:
            return False
        body_pose = checked_pose(body_pose.position, body_pose.orientation)
        tracking = compose(body_pose, self.body_to_tracking)
        self.local.append((stamp_ns, tracking))
        self.last_local_ns = stamp_ns
        cutoff = stamp_ns - round(self.limits.history_sec * 1e9)
        while len(self.local) > 2 and self.local[1][0] < cutoff:
            self.local.popleft()
        self.global_pending = {t: p for t, p in self.global_pending.items() if t >= cutoff}
        if stamp_ns in self.global_pending:
            self._pair(stamp_ns, tracking, self.global_pending.pop(stamp_ns))
        return True

    def add_global_tracking(self, stamp_ns, pose, context):
        self._admit_pose(stamp_ns, context)
        if stamp_ns <= self.last_global_ns:
            return False
        pose = checked_pose(pose.position, pose.orientation)
        self.last_global_ns = stamp_ns
        local = next((value for t, value in reversed(self.local) if t == stamp_ns), None)
        if local is None:
            self.global_pending[stamp_ns] = pose
            while len(self.global_pending) > 16:
                del self.global_pending[min(self.global_pending)]
        else:
            self._pair(stamp_ns, local, pose)
        return True

    def _pair(self, stamp_ns, local, global_pose):
        if self.alignments and stamp_ns <= self.alignments[-1][0]:
            return
        correction = compose(global_pose, inverse(local))
        if self.alignment_anchor is None:
            self.alignment_anchor = (stamp_ns, correction)
        anchor_ns, anchor = self.alignment_anchor
        cumulative = pose_innovation(anchor, correction)
        translation = math.hypot(cumulative.translation_xy, cumulative.translation_z)
        self.alignment_displacement = dict(anchor_stamp_ns=anchor_ns, stamp_ns=stamp_ns,
            translation_m=translation, rotation_rad=cumulative.rotation)
        if self.alignments:
            innovation = pose_innovation(self.alignments[-1][1], correction)
            if (math.hypot(innovation.translation_xy, innovation.translation_z)
                    > self.limits.max_alignment_translation_step_m
                    or innovation.rotation > self.limits.max_alignment_rotation_step_rad):
                self.fault = 'map_alignment_jump_requires_context_reset'
                raise ProjectionError(self.fault)
        if (self.projection_frame == 'map'
                and (translation > self.limits.max_alignment_translation_from_anchor_m
                     or cumulative.rotation > self.limits.max_alignment_rotation_from_anchor_rad)):
            self.fault = 'map_alignment_accumulation_requires_context_reset'
            raise ProjectionError(self.fault)
        self.alignments.append((stamp_ns, correction))

    def project(self, raw, context, *, now_ns, authorized_pose_ns):
        if self.context is None or context != self.context or self.fault:
            raise ProjectionError(self.fault or 'projection_context_mismatch')
        if self.ray_to_tracking is None:
            raise AwaitingCoverage('waiting_static_extrinsics')
        if (raw.sensor_id not in (0, 1) or type(now_ns) is not int
                or type(authorized_pose_ns) is not int
                or raw.points.dtype not in ((RAY_DTYPE, ISAAC_RAY_DTYPE) if self.maximum_isaac_actor_id else (RAY_DTYPE,))
                or raw.points.ndim != 1 or not 1 <= len(raw.points) <= self.limits.max_input_points):
            raise ProjectionError('invalid_projection_input')
        if raw.points.dtype == ISAAC_RAY_DTYPE and np.any(raw.points['isaac_actor_id'] > self.maximum_isaac_actor_id):
            raise ProjectionError('native_hit_actor_outside_registry')
        duration_ns = raw.end_ns - raw.start_ns
        # PhysX LiDAR returns one simultaneous measured snapshot. It has no
        # measured per-beam sweep offsets; never fabricate a rotating scan's
        # duration. This exception is opt-in for a sealed isolated fixture.
        valid_duration = (1000000 <= duration_ns <= round(self.limits.max_scan_duration_sec * 1e9)
            or self.allow_simulation_snapshot and duration_ns == 0
                and np.all(raw.points['offset_time'] == 0))
        if (raw.start_ns <= context.barrier_ns or raw.start_ns <= self.last_input[raw.sensor_id]
                or not valid_duration
                or not -10000000 <= now_ns - raw.start_ns <= round(self.limits.input_timeout_sec * 1e9)
                or raw.end_ns > now_ns + 10000000):
            raise ProjectionError('ray_stale_duplicate_future_or_before_context')
        points = raw.points
        # Airy channels share acquisition instants. Interpolate and compose
        # their transforms once per EXACT time, without binning or downsampling.
        offsets, time_index = acquisition_groups(points['offset_time'])
        times = raw.start_ns + offsets.astype(np.int64)
        translation, rotation = interpolate_many(
            self.local, times, round(self.limits.max_pose_gap_sec * 1e9))
        if self.projection_frame == 'odom':
            # Geometry is tied solely to continuous local motion. Global
            # corrections update the route anchor, never this occupied map.
            # Still require an actually authorized measured watermark: changing
            # coordinates cannot manufacture a fresh observation or reset epoch.
            alignment = next(((t, Pose3((0., 0., 0.), (0., 0., 0., 1.)))
                              for t, _ in self.local
                              if raw.end_ns <= t <= min(authorized_pose_ns,
                                 raw.end_ns + round(self.limits.alignment_wait_sec * 1e9))), None)
        else:
            alignment = next(((t, pose) for t, pose in self.alignments
                              if raw.end_ns <= t <= min(authorized_pose_ns,
                                 raw.end_ns + round(self.limits.alignment_wait_sec * 1e9))), None)
        if alignment is None:
            raise AwaitingCoverage('waiting_same_stamp_map_alignment_pair')
        alignment_ns, correction = alignment
        origin = np.asarray([points[name][0] for name in ('origin_x', 'origin_y', 'origin_z')])
        if any(np.any(points[name] != points[name][0])
               for name in ('origin_x', 'origin_y', 'origin_z')):
            raise ProjectionError('raw_origin_not_fixed_extrinsic')
        local_rotation = rotation_matrices(rotation)
        map_rotation = rotation_matrices(correction.orientation)
        ray_rotation = rotation_matrices(self.ray_to_tracking.orientation)
        matrices = map_rotation @ local_rotation @ ray_rotation
        tracking_origin = (np.einsum('nij,j->ni', local_rotation,
                                    np.asarray(self.ray_to_tracking.position)) + translation)
        translations = tracking_origin @ map_rotation.T + np.asarray(correction.position)
        world_endpoints = transform_indexed_endpoints(matrices, time_index, points)
        world_endpoints += translations[time_index]
        # Origins are identical for beams acquired at the exact same instant.
        # Keep that exact-time table compact until writing each wire field.
        world_origins = np.einsum('nij,j->ni', matrices, origin) + translations
        # Experimental preview mask runs only AFTER coverage of all original
        # acquisition times. Never shorten the scan's temporal admission window.
        exclusion = self.preview_exclusion
        matched = np.zeros(len(points), dtype=bool)
        if exclusion is not None:
            body_from_ray = compose(self.body_to_tracking, self.ray_to_tracking)
            endpoints = np.column_stack([points[name] for name in ('x', 'y', 'z')])
            matched = exclusion.matches(transform_many(body_from_ray, endpoints), raw.sensor_id)
        dropped = matched if exclusion is not None and exclusion.mode == 'preview_drop_rays' else np.zeros(len(points), dtype=bool)
        if dropped.all():
            # No observation => no sequence/last-input update or source lease.
            raise PreviewExcludedScan(len(points), exclusion.digest)
        # Structured ndarray.copy performs a separate strided copy per field.
        # The decoded wire buffer is contiguous: one byte copy preserves every
        # acquisition field exactly and owns its memory independently of input.
        output = (points.view(np.uint8).copy().view(points.dtype)
                  if points.flags.c_contiguous else points.copy())
        for world, names, indices in ((world_endpoints, ('x', 'y', 'z'), None),
                (world_origins, ('origin_x', 'origin_y', 'origin_z'), time_index)):
            if not np.isfinite(world).all() or np.max(np.abs(world)) > 1e6:
                raise ProjectionError('invalid_projected_geometry')
            for axis, name in enumerate(names):
                output[name] = world[:, axis] if indices is None else world[indices, axis]
        if np.any(dropped):
            output = output[~dropped]
        self.sequence += 1
        self.last_input[raw.sensor_id] = raw.start_ns
        return Projected(output, raw.start_ns, raw.end_ns, alignment_ns,
                         raw.sensor_id, context, self.sequence,
                         exclusion.digest if exclusion else '', int(matched.sum()), int(dropped.sum()),
                         self.projection_frame)
