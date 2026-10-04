"""Offline specification of an immutable map -> continuous-odom control anchor.

This module is deliberately not motion admission. The current TaggedBspline and
localization transport do not carry this contract. A YAML acceptance flag cannot
invent that missing integration. Source nanoseconds, geometry and task identity
are never refreshed by a coordinate conversion.
"""
from dataclasses import dataclass
import hashlib
import json
import math


def control_frame_blockers():
    return ('control_anchor_transport_missing',
            'tracker_task_curve_state_not_bound_to_continuous_odom',
            'map_correction_clearance_revalidation_not_integrated')


def _vector(value, size):
    value = tuple(value)
    if len(value) != size or any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError('finite vector required')
    return value


def _positive_int(value):
    return type(value) is int and value > 0


@dataclass(frozen=True)
class Context:
    session: str
    epoch: int
    seed: str
    map_version: str

    def __post_init__(self):
        if not all(isinstance(v, str) and v for v in (self.session, self.seed, self.map_version)) or not _positive_int(self.epoch):
            raise ValueError('complete localization and map identity required')


@dataclass(frozen=True)
class Rigid:
    """Parent <- child transform; quaternion order is x,y,z,w."""
    xyz: tuple
    xyzw: tuple

    def __post_init__(self):
        object.__setattr__(self, 'xyz', _vector(self.xyz, 3))
        q = _vector(self.xyzw, 4)
        if abs(sum(v*v for v in q)-1.) > 1e-6:
            raise ValueError('unit quaternion required; do not silently calibrate input')
        object.__setattr__(self, 'xyzw', q)

    def rotate(self, value):
        x, y, z = _vector(value, 3)
        a, b, c, w = self.xyzw
        tx, ty, tz = 2*(b*z-c*y), 2*(c*x-a*z), 2*(a*y-b*x)
        return (x+w*tx+b*tz-c*ty, y+w*ty+c*tx-a*tz, z+w*tz+a*ty-b*tx)

    def point(self, value):
        return tuple(a+b for a, b in zip(self.rotate(value), self.xyz))

    def inverse(self):
        a, b, c, w = self.xyzw
        rotation = Rigid((0., 0., 0.), (-a, -b, -c, w))
        return Rigid(rotation.rotate(tuple(-v for v in self.xyz)), rotation.xyzw)

    def compose(self, child):
        x, y, z, w = self.xyzw
        a, b, c, d = child.xyzw
        return Rigid(self.point(child.xyz),
                     (w*a+x*d+y*c-z*b, w*b-x*c+y*d+z*a,
                      w*c+x*b-y*a+z*d, w*d-x*a-y*b-z*c))


@dataclass(frozen=True)
class BodySample:
    context: Context
    frame: str
    child: str
    source_ns: int
    pose: Rigid

    def __post_init__(self):
        if not isinstance(self.context, Context) or not isinstance(self.pose, Rigid) or not _positive_int(self.source_ns):
            raise ValueError('typed, original-time body sample required')


@dataclass(frozen=True)
class ControlAnchor:
    context: Context
    revision: int
    source_ns: int
    map_from_odom: Rigid
    body_in_odom: Rigid
    anchor_id: str


def create_anchor(global_body, local_body, revision):
    if (not isinstance(global_body, BodySample) or not isinstance(local_body, BodySample)
            or global_body.context != local_body.context
            or global_body.source_ns != local_body.source_ns
            or global_body.frame != 'd1max_loc_map' or local_body.frame != 'd1max_loc_odom'
            or global_body.child != 'd1max_loc_base_link' or local_body.child != global_body.child
            or not _positive_int(revision)):
        raise ValueError('anchor needs same-source-time, same-context map/local body poses')
    transform = global_body.pose.compose(local_body.pose.inverse())
    identity = (global_body.context.__dict__, revision, global_body.source_ns,
                transform.xyz, transform.xyzw, local_body.pose.xyz, local_body.pose.xyzw)
    anchor_id = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    return ControlAnchor(global_body.context, revision, global_body.source_ns,
                         transform, local_body.pose, anchor_id)


@dataclass(frozen=True)
class MapSpline:
    context: Context
    generation: int
    trajectory_id: int
    source_ns: int
    points: tuple
    knots: tuple
    order: int = 3

    def __post_init__(self):
        points = tuple(_vector(p, 3) for p in self.points)
        knots = _vector(self.knots, len(points)+4)
        if (not isinstance(self.context, Context) or not _positive_int(self.generation)
                or type(self.trajectory_id) is not int or self.trajectory_id < 0
                or not _positive_int(self.source_ns) or self.order != 3
                or not 4 <= len(points) <= 10000
                or any(b-a <= 1e-9 for a, b in zip(knots, knots[1:]))):
            raise ValueError('invalid immutable cubic spline contract')
        object.__setattr__(self, 'points', points)
        object.__setattr__(self, 'knots', knots)


@dataclass(frozen=True)
class AnchoredControlBundle:
    """One indivisible goal+curve+initial body-state conversion, not an execution permit."""
    anchor: ControlAnchor
    original: MapSpline
    goal_map: tuple
    goal_odom: tuple
    points_odom: tuple
    frame: str = 'd1max_loc_odom'


def anchor_control_bundle(anchor, spline, goal_map):
    if (not isinstance(anchor, ControlAnchor) or not isinstance(spline, MapSpline)
            or anchor.context != spline.context or anchor.source_ns != spline.source_ns):
        raise ValueError('curve needs an exact original-time anchor; latest TF is not a substitute')
    goal = _vector(goal_map, 3)  # already a body-center goal; never add height again
    inverse = anchor.map_from_odom.inverse()
    return AnchoredControlBundle(anchor, spline, goal, inverse.point(goal),
                                 tuple(inverse.point(point) for point in spline.points))


def correction_review(bundle, updated_anchor, *, reserved_clearance_m, body_radius_m):
    """Conservative bound on the complete cubic convex hull and body rotation.

    Both geometric budgets must come from the actual collision certificate/body
    model, never an arbitrary tolerance. Even an in-budget new version requires
    native revalidation before a replacement is admitted; this function grants
    no permission and never mutates the incumbent anchor.
    """
    for value in (reserved_clearance_m, body_radius_m):
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('positive measured body radius and reserved clearance required')
    old = bundle.anchor
    if updated_anchor.context != old.context:
        return {'reason': 'localization_or_map_context_changed', 'requires_stop': True}
    if updated_anchor.anchor_id == old.anchor_id:
        return {'reason': 'anchor_unchanged', 'requires_stop': False, 'displacement_bound_m': 0.}
    if updated_anchor.revision <= old.revision or updated_anchor.source_ns <= old.source_ns:
        return {'reason': 'anchor_version_or_source_not_newer', 'requires_stop': True}
    inverse = updated_anchor.map_from_odom.inverse()
    max_shift = max(math.dist(a, inverse.point(p)) for a, p in zip(bundle.points_odom, bundle.original.points))
    qa, qb = old.map_from_odom.xyzw, updated_anchor.map_from_odom.xyzw
    cosine = min(1., abs(sum(a*b for a, b in zip(qa, qb))))
    bound = max_shift + 2*body_radius_m*math.sqrt(max(0., 1-cosine*cosine))
    exceeded = bound > reserved_clearance_m
    return {'reason': 'correction_budget_exceeded' if exceeded else 'native_recheck_required',
            'requires_stop': exceeded, 'requires_native_recheck': True,
            'displacement_bound_m': bound}
