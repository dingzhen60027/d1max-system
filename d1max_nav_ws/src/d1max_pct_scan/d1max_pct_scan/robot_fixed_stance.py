"""A frozen engineering wheel-mode stance over the official collision CAD.

The SDK need not supply joints for this explicitly selected fixed model.  Its
angles are an engineering construction, not measured/source-stamped poses or
physical calibration.  They stay private to geometry; no JointState, TF, SDK
connection, motion permission, or free-space evidence is produced here.

The measured approximate body box is descriptive only.  Collision queries use
the source model's actual triangles and first-return policy, including the
space between the legs and CAD concavities.  Meshes are prepared by warmup,
never by the classification path.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import hashlib
import json
import math
from types import MappingProxyType
from typing import Mapping
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares

from .robot_self_geometry import RobotSelfModel, _axis_rotation, _transform


SCHEMA = 'd1max_fixed_stance_self_model/v1'
POSTURE_SOURCE = 'engineering_nominal_not_measured'
_LEGS = (('FAR', 1.), ('FBL', 1.), ('RAR', -1.), ('RBL', -1.))


@dataclass(frozen=True)
class FixedStanceProfile:
    """Explicit engineering dimensions in metres, not deletion padding.

    Width/height describe the user's approximate body/collision proxy.  They
    are not substituted for the collision CAD.  Wheel thickness is recorded
    for cross-checking, while the actual wheel surface remains the CAD mesh.
    There is deliberately no measured outer stance width constraint.
    """
    lidar_ground_height_m: float = .50
    wheelbase_m: float = .75
    wheel_diameter_m: float = .19
    wheel_thickness_m: float = .05
    body_box_width_m: float = .30
    body_box_height_m: float = .50

    def __post_init__(self):
        limits = {
            'lidar_ground_height_m': (.2, 1.2),
            'wheelbase_m': (.3, 1.5),
            'wheel_diameter_m': (.1, .3),
            'wheel_thickness_m': (.01, .12),
            'body_box_width_m': (.1, .8),
            'body_box_height_m': (.1, 1.2),
        }
        for name, (lower, upper) in limits.items():
            value = getattr(self, name)
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                raise ValueError('invalid fixed stance dimension: '+name) from None
            if (isinstance(value, (bool, str)) or not math.isfinite(numeric)
                    or not lower <= numeric <= upper):
                raise ValueError('invalid fixed stance dimension: '+name)
            object.__setattr__(self, name, numeric)

    @classmethod
    def coerce(cls, value):
        if value is None:
            return cls()
        if isinstance(value, cls):
            return value
        if not isinstance(value, Mapping):
            raise ValueError('fixed stance profile must be named dimensions')
        try:
            return cls(**dict(value))
        except TypeError as error:
            raise ValueError('unknown fixed stance profile dimension') from error


def _joint_limits(model):
    """The sealed URDF provides limits; an IK result cannot override them."""
    elements = ET.parse(model.urdf_path).getroot().findall('joint')
    result = {}
    for element in elements:
        limit = element.find('limit')
        if element.get('type') != 'revolute':
            continue
        if limit is None:
            raise ValueError('nominal stance requires explicit revolute limits')
        lower, upper = float(limit.get('lower')), float(limit.get('upper'))
        if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
            raise ValueError('invalid nominal stance joint limits')
        result[element.get('name')] = (lower, upper)
    return result


def _nominal_stance(model, profile):
    joints = {joint.name: joint for joint in model.joints}
    required = {prefix+'_'+part+'_JOINT' for prefix, _ in _LEGS
                for part in ('ABAD', 'HIP', 'KNEE', 'FOOT')}
    if set(model.joint_types) != required:
        raise ValueError('fixed stance requires the official four wheel-leg chains')
    limits = _joint_limits(model)
    poses, _ = model.forward_kinematics()
    sensor = poses.get('R_AIRY_LINK')
    front_sensor = poses.get('F_AIRY_LINK')
    if sensor is None or front_sensor is None:
        raise ValueError('fixed stance requires fixed official LiDAR origins')
    if not math.isclose(sensor[2, 3], front_sensor[2, 3], abs_tol=1e-6):
        raise ValueError('fixed stance requires one nominal LiDAR height')
    # h_lidar = h_root + z_root_lidar; this is a datum, not a changed TF.
    root_height = profile.lidar_ground_height_m-float(sensor[2, 3])
    wheel_z = -root_height+profile.wheel_diameter_m/2
    values = {name: 0. for name in required}
    residuals, wheel_centers, branches = {}, {}, {}
    for prefix, foreaft_sign in _LEGS:
        names = tuple(prefix+'_'+part+'_JOINT'
                      for part in ('ABAD', 'HIP', 'KNEE', 'FOOT'))
        chain = tuple(joints[name] for name in names)
        if (chain[0].parent != model.root_link or
                any(a.child != b.parent for a, b in zip(chain, chain[1:])) or
                tuple(joint.kind for joint in chain) !=
                ('revolute', 'revolute', 'revolute', 'continuous')):
            raise ValueError('unsupported fixed stance leg chain: '+prefix)
        if (not np.allclose(chain[0].axis, (1., 0., 0.)) or
                any(not np.allclose(joint.axis, (0., 1., 0.)) for joint in chain[1:])):
            raise ValueError('unsupported fixed stance joint axes: '+prefix)
        if not limits[names[0]][0] <= 0. <= limits[names[0]][1]:
            raise ValueError('upright wheel stance violates abduction limits')
        collision = tuple(c for c in model.collisions if c.link == chain[-1].child)
        if len(collision) != 1:
            raise ValueError('one official wheel collision required: '+prefix)
        collision = collision[0]
        # Preserve the real asymmetric link-local wheel offset.  CAD mesh
        # bounds locate its centre; they are NEVER a filled collision shape.
        local_center = (np.asarray(collision.lower)+collision.upper)/2
        local_center = (_transform(collision.xyz, collision.rpy) @
                        np.r_[local_center, 1.])[:3]

        def center_at(hip_knee):
            transform = np.eye(4)
            for joint, angle in zip(chain, (0., *hip_knee, 0.)):
                motion = np.eye(4)
                motion[:3, :3] = _axis_rotation(joint.axis, float(angle))
                transform = transform @ _transform(joint.xyz, joint.rpy) @ motion
            return transform[:3, :3] @ local_center+transform[:3, 3]

        target_xz = np.array([foreaft_sign*profile.wheelbase_m/2, wheel_z])
        lower = np.array([limits[name][0] for name in names[1:3]])
        upper = np.array([limits[name][1] for name in names[1:3]])
        solutions = []
        # Both permitted bending branches are examined.  The smallest joint
        # norm chooses a reproducible engineering branch, not a measured knee.
        for seed in ((.5*foreaft_sign, -1.2*foreaft_sign),
                     (-.7*foreaft_sign, 1.2*foreaft_sign)):
            seed = np.clip(seed, lower+1e-8, upper-1e-8)
            result = least_squares(
                lambda q: center_at(q)[[0, 2]]-target_xz, seed,
                bounds=(lower, upper), xtol=1e-13, ftol=1e-13, gtol=1e-13,
                max_nfev=100)
            error = float(np.linalg.norm(center_at(result.x)[[0, 2]]-target_xz))
            if result.success and error <= 1e-7:
                solutions.append((float(np.linalg.norm(result.x)), result.x, error))
        if not solutions:
            raise ValueError('fixed stance dimensions are unreachable: '+prefix)
        solutions.sort(key=lambda entry: (entry[0], tuple(entry[1])))
        _, selected, error = solutions[0]
        values[names[1]], values[names[2]] = map(float, selected)
        residuals[prefix] = error
        wheel_centers[prefix] = tuple(map(float, center_at(selected)))
        branches[prefix] = len(solutions)
    fixed_poses, missing = model.forward_kinematics(values)
    if missing:
        raise ValueError('fixed stance leaves an unsupported collision chain')
    return values, fixed_poses, root_height, residuals, wheel_centers, branches


@dataclass(frozen=True)
class FixedStanceModel:
    """Read-only adapter: every source CAD link is fixed in the root frame."""
    base_model: RobotSelfModel
    profile: FixedStanceProfile = field(default_factory=FixedStanceProfile)
    _positions: Mapping = field(init=False, repr=False)
    _poses: Mapping = field(init=False, repr=False)
    _root_height: float = field(init=False, repr=False)
    _residuals: Mapping = field(init=False, repr=False)
    _wheel_centers: Mapping = field(init=False, repr=False)
    _branches: Mapping = field(init=False, repr=False)
    _collision_frames: tuple = field(init=False, repr=False)

    def __post_init__(self):
        if not isinstance(self.base_model, RobotSelfModel):
            raise ValueError('fixed stance requires an asset-backed collision model')
        profile = FixedStanceProfile.coerce(self.profile)
        values, poses, height, residuals, centers, branches = _nominal_stance(self.base_model, profile)
        object.__setattr__(self, 'profile', profile)
        object.__setattr__(self, '_positions', MappingProxyType(values))
        object.__setattr__(self, '_poses', MappingProxyType(poses))
        object.__setattr__(self, '_root_height', height)
        object.__setattr__(self, '_residuals', MappingProxyType(residuals))
        object.__setattr__(self, '_wheel_centers', MappingProxyType(centers))
        object.__setattr__(self, '_branches', MappingProxyType(branches))
        object.__setattr__(self, '_collision_frames', self.base_model._prepare_collision_frames(self._poses))

    @classmethod
    def from_base_model(cls, model, *, profile=None):
        return cls(model, FixedStanceProfile.coerce(profile))

    @classmethod
    def from_urdf(cls, path, *, package_roots=None, profile=None):
        return cls.from_base_model(RobotSelfModel.from_urdf(path, package_roots=package_roots),
                                   profile=profile)

    @classmethod
    def load(cls, path, *, profile=None, warm=False):
        result = cls.from_base_model(RobotSelfModel.load(path), profile=profile)
        if warm:
            result.warmup()
        return result

    @property
    def root_link(self):
        return self.base_model.root_link

    @property
    def links(self):
        return self.base_model.links

    @property
    def collisions(self):
        return self.base_model.collisions

    @property
    def asset_paths(self):
        return self.base_model.asset_paths

    @property
    def joint_types(self):
        # No live joint source exists or is required by this model adapter.
        return {}

    @property
    def required_joint_names(self):
        return ()

    @property
    def nominal_joint_positions(self):
        """Read-only engineering metadata; never a source JointState."""
        return self._positions

    @property
    def nominal_wheel_centers(self):
        return self._wheel_centers

    def to_dict(self):
        return {
            'schema': SCHEMA,
            'root_link': self.root_link,
            'posture_source': POSTURE_SOURCE,
            'posture_calibrated': False,
            'extrinsics_calibrated': False,
            'grants_motion_authority': False,
            'uses_visual_geometry': False,
            'requires_joint_source': False,
            'extends_free_rays': False,
            'mesh_policy': 'actual_triangle_first_surface_only',
            'applicability': {
                'posture': 'nominal_wheel_standing',
                'posture_evidence': 'engineering_profile_not_live_measurement',
                'live_posture_verified': False,
                'lidar_ground_height_m': self.profile.lidar_ground_height_m,
                'outer_stance_width_measured': False,
                'unsupported_posture_changes': ['crouch', 'articulated_leg_motion', 'stairs'],
                'on_unmatched_return': 'preserve_original_ray',
                'does_not_clear_occupied_or_unknown_voxels': True,
            },
            'profile': asdict(self.profile),
            'body_proxy_used_as_collision': False,
            'nominal_root_ground_height_m': self._root_height,
            'nominal_floor_z_root_m': -self._root_height,
            'nominal_joint_positions_rad': dict(self._positions),
            'branch_policy': 'smallest_joint_norm_within_official_limits',
            'feasible_bending_branch_counts': dict(self._branches),
            'wheel_xz_fit_residual_m': dict(self._residuals),
            'nominal_wheel_centers_root_m': dict(self._wheel_centers),
            'nominal_link_transforms_root': {
                name: pose.tolist() for name, pose in self._poses.items()},
            'source_model': self.base_model.to_dict(),
        }

    @property
    def content_hash(self):
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True,
            separators=(',', ':'), allow_nan=False).encode()).hexdigest()

    def warmup(self, *, include_dynamic=True, sensor_origins=None):
        # All articulated assets are static in THIS adapter.  The underlying
        # immutable cache still checks original mesh hashes before loading.
        report = self.base_model.warmup(include_dynamic=True, sensor_origins=sensor_origins)
        return dict(report, posture_source=POSTURE_SOURCE, requires_joint_source=False,
                    fixed_stance_links=len(self.links))

    def classify(self, endpoints_body, origins_body, *, include_static=True,
                 include_dynamic=True, config=None):
        # include_dynamic is accepted for the common model protocol.  All
        # frozen legs/wheels belong to this adapter's static entity set.
        return self.base_model._classify_at_poses(
            endpoints_body, origins_body, self._poses,
            include_static=include_static, include_dynamic=include_static, config=config,
            prepared_frames=self._collision_frames)


Model = FixedStanceModel
