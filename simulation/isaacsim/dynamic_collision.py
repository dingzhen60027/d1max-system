"""Bounded simulation truth vetoes, independent of laser FREE evidence.

Enabled actors are present for the whole session. Registration, shape, world
identity and source time are explicit; absent/unregistered colliders fail closed.
Reachable boxes cover any actor orientation plus bounded centre translation.
This oracle is a simulation test facility, not a deployable perception sensor.
"""
from __future__ import annotations

import hashlib
import json
import math
import re

KIND = "isaac_dynamic_occupancy_oracle_v1"
MAX_HORIZON_NS = 300_000_000
MAX_SOURCE_AGE_NS = 200_000_000
MAX_REACHABLE_HORIZON_NS = 8_000_000_000
MAX_ACTORS = 64


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def vector(value, label, length=3):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("invalid_" + label)
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in value):
        raise ValueError("invalid_" + label)
    return tuple(float(v) for v in value)


def number(value, label, positive=False):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError("invalid_" + label)
    return float(value)


def actor_registry(spec):
    """Stable explicit actor/shape identity, excluding changing world poses."""
    values = spec.get("dynamic_actors", [])
    if not isinstance(values, list) or len(values) > MAX_ACTORS:
        raise ValueError("invalid_dynamic_actor_registry")
    actors, ids = [], set()
    for actor in values:
        actor_id = actor.get("id")
        if not isinstance(actor_id, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]{0,127}", actor_id) or actor_id in ids:
            raise ValueError("invalid_dynamic_actor_id")
        ids.add(actor_id)
        if type(actor.get("enabled")) is not bool:
            raise ValueError("dynamic_actor_enabled_must_be_explicit")
        if any(key in actor for key in ("lifetime", "spawn_time_s", "despawn_time_s")):
            raise ValueError("dynamic_actor_birth_requires_new_session")
        if not actor["enabled"]:
            continue
        shapes, shape_ids = [], set()
        for shape in actor.get("collision_shapes", []):
            shape_id, kind = shape.get("id"), shape.get("type")
            if not isinstance(shape_id, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]{0,127}", shape_id) or shape_id in shape_ids:
                raise ValueError("invalid_dynamic_shape_id")
            shape_ids.add(shape_id)
            if shape.get("collision") is not True:
                raise ValueError("dynamic_actor_missing_collision")
            item = dict(id=shape_id, path=f"/World/Dynamic/{actor_id}/{shape_id}",
                        type=kind, center=list(vector(shape["center"], "shape_center")))
            if kind == "box":
                item["size"] = [number(v, "shape_size", True) for v in vector(shape["size"], "shape_size")]
            elif kind == "cylinder":
                item.update(radius=number(shape["radius"], "shape_radius", True), height=number(shape["height"], "shape_height", True), axis="Z")
            else:
                raise ValueError("unsupported_dynamic_shape")
            shapes.append(item)
        if not shapes or len(shapes) > 64:
            raise ValueError("invalid_dynamic_shape_registry")
        actors.append(dict(id=actor_id, path=f"/World/Dynamic/{actor_id}", shapes=sorted(shapes, key=lambda s: s["id"]),
                           max_linear_speed_mps=number(actor["max_linear_speed_mps"], "actor_speed"),
                           max_linear_acceleration_mps2=number(actor["max_linear_acceleration_mps2"], "actor_acceleration")))
    return sorted(actors, key=lambda a: a["id"])


def registry_digest(registry):
    return hashlib.sha256(canonical(registry)).hexdigest()


def shape_radius(shape):
    # Sphere about actor root encloses the complete local shape for all yaw,
    # roll and pitch; rotation never sweeps outside this enclosure.
    center = vector(shape["center"], "shape_center")
    if shape["type"] == "box":
        half = tuple(v / 2 for v in vector(shape["size"], "shape_size"))
        return max(math.sqrt(sum((center[d] + sign[d] * half[d]) ** 2 for d in range(3)))
                   for sign in ((x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)))
    if shape["type"] == "cylinder":
        # Enclosing box is conservative for a cylinder and all pivot offsets.
        half = (number(shape["radius"], "radius", True),) * 2 + (number(shape["height"], "height", True) / 2,)
        return math.sqrt(sum((abs(center[d]) + half[d]) ** 2 for d in range(3)))
    raise ValueError("unsupported_dynamic_shape")


def reachable_actor_region(actor, position, velocity, horizon_ns=MAX_HORIZON_NS, margin_m=.02):
    """Full actor reachable envelope; no thin beam/corner free inference."""
    if type(horizon_ns) is not int or not 0 < horizon_ns <= MAX_REACHABLE_HORIZON_NS:
        raise ValueError("invalid_dynamic_horizon")
    position, velocity = vector(position, "actor_position"), vector(velocity, "actor_velocity")
    speed = math.sqrt(sum(v * v for v in velocity))
    bound = number(actor["max_linear_speed_mps"], "actor_speed")
    acceleration = number(actor["max_linear_acceleration_mps2"], "actor_acceleration")
    if speed > bound + 1e-7:
        raise ValueError("dynamic_actor_speed_outside_contract")
    margin = number(margin_m, "dynamic_geometry_margin")
    radius = max(shape_radius(shape) for shape in actor["shapes"])
    dt = horizon_ns * 1e-9
    # Absolute speed bound remains valid even at a trajectory turn. The
    # acceleration term supplies extra enclosure and is never a relaxation.
    radius += min(bound * dt, speed * dt + .5 * acceleration * dt * dt) + margin
    return dict(state=2, min=[v - radius for v in position], max=[v + radius for v in position])


def oracle_payload(registry, samples, *, session_id, epoch, seed_id, context_sequence,
                   sequence, source_stamp_ns, frame_id="d1max_loc_odom", horizon_ns=MAX_HORIZON_NS,
                   reachable_horizon_ns=MAX_HORIZON_NS):
    """Consume same-step actual actor-root poses, never commanded positions."""
    for value in (epoch, context_sequence, sequence, source_stamp_ns):
        if type(value) is not int or value <= 0:
            raise ValueError("invalid_dynamic_source_identity")
    if not session_id or not seed_id or not frame_id or type(horizon_ns) is not int or not 0 < horizon_ns <= MAX_HORIZON_NS:
        raise ValueError("invalid_dynamic_source_identity")
    if type(reachable_horizon_ns) is not int or not 0 < reachable_horizon_ns <= MAX_REACHABLE_HORIZON_NS:
        raise ValueError("invalid_dynamic_reachable_horizon")
    if set(samples) != {actor["id"] for actor in registry}:
        raise ValueError("missing_or_unregistered_dynamic_actor")
    actors = []
    for actor in registry:
        sample = samples[actor["id"]]
        if sample.get("source_stamp_ns") != source_stamp_ns or sample.get("present") is not True:
            raise ValueError("dynamic_actor_not_present_at_body_source")
        actors.append(dict(actor_id=actor["id"], regions=[reachable_actor_region(actor,
            sample["position"], sample["linear_velocity"], reachable_horizon_ns)]))
    return dict(schema=1, kind=KIND, session_id=session_id, epoch=epoch, seed_id=seed_id,
        context_sequence=context_sequence, sequence=sequence, frame_id=frame_id,
        registry_sha256=registry_digest(registry), source_stamp_ns=source_stamp_ns,
        valid_until_ns=source_stamp_ns + horizon_ns, complete=True, actors=actors,
        reachable_horizon_ns=reachable_horizon_ns, reachable_until_ns=source_stamp_ns + reachable_horizon_ns,
        source_scope="isolated_simulation_same_step_dynamic_collision_truth_not_lidar")


def closed_aabb_intersects(a_lower, a_upper, b_lower, b_upper):
    return all(a_lower[d] <= b_upper[d] and b_lower[d] <= a_upper[d] for d in range(3))


def voxel_veto(payload, cell, resolution=.05):
    """Offline geometry audit only; production also validates identity/leases."""
    if len(cell) != 3 or any(type(v) is not int for v in cell):
        raise ValueError("invalid_voxel_index")
    resolution = number(resolution, "resolution", True)
    lower, upper = [v * resolution for v in cell], [(v + 1) * resolution for v in cell]
    state = 0
    for actor in payload["actors"]:
        for region in actor["regions"]:
            if closed_aabb_intersects(lower, upper, region["min"], region["max"]):
                if region["state"] == 1:
                    return 1
                state = 2
    return state


def certify_robot_in_body_envelope(collider_world_bounds, *, body_position, body_orientation_xyzw,
                                   radius, offset, below, above, floor_z, support_paths=(), tolerance_m=1e-6):
    """Prove every measured link AABB is enclosed by the query double cylinder.

    This is deliberately stronger than checking only nominal roll/pitch/height.
    Each transformed primitive bounding box is clipped at body-local x=0.
    Every vertex of each convex half must fit the corresponding cylinder;
    checking only vertices against the union could miss its narrow waist.
    Declared support colliders may intersect the certified flat floor by at
    most 2 cm; their complete bounds remain checked. Low-obstacle cells are
    never cleared by this function.
    """
    p = vector(body_position, "body_position")
    q = vector(body_orientation_xyzw, "body_orientation", 4)
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-8:
        raise ValueError("invalid_body_orientation")
    x, y, z, w = (v / norm for v in q)
    heading_x, heading_y = 1 - 2 * (y * y + z * z), 2 * (x * y + z * w)
    heading_norm = math.hypot(heading_x, heading_y)
    if heading_norm < 1e-8:
        raise ValueError("vertical_body_heading")
    cosine, sine = heading_x / heading_norm, heading_y / heading_norm
    radius = number(radius, "body_radius", True)
    offset, below, above = (number(v, label) for v, label in ((offset, "offset"), (below, "below"), (above, "above")))
    floor_z = float(floor_z)
    if not math.isfinite(floor_z) or not collider_world_bounds:
        raise ValueError("missing_robot_collision_bounds")
    support_paths = set(support_paths)
    checked = []
    for item in collider_world_bounds:
        lower, upper = vector(item["min"], "link_lower"), vector(item["max"], "link_upper")
        if any(a > b for a, b in zip(lower, upper)):
            raise ValueError("invalid_robot_collision_bounds")
        points = item.get("corners_world")
        if points is None:
            points = [(vx, vy, vz) for vx in (lower[0], upper[0]) for vy in (lower[1], upper[1]) for vz in (lower[2], upper[2])]
        if not isinstance(points, (list, tuple)) or len(points) != 8:
            raise ValueError("invalid_robot_primitive_corners")
        points = [vector(point, "robot_primitive_corner") for point in points]
        lower = [min(point[d] for point in points) for d in range(3)]
        upper = [max(point[d] for point in points) for d in range(3)]
        # Only an explicitly authorized support collider can extend to the
        # support plane. This does not exempt any whole navigation voxel.
        if lower[2] < floor_z - (.02 if item["path"] in support_paths else 0.) - tolerance_m:
            raise ValueError("robot_link_below_certified_support_slab:" + item["path"])
        minimum_z = lower[2]
        if minimum_z < p[2] - below - tolerance_m or upper[2] > p[2] + above + tolerance_m:
            raise ValueError("robot_link_outside_vertical_query_envelope:" + item["path"])
        xy = [((point[0] - p[0]) * cosine + (point[1] - p[1]) * sine,
               -(point[0] - p[0]) * sine + (point[1] - p[1]) * cosine) for point in points]
        crossings = []
        # Every clipped convex-hull vertex is either an original vertex or
        # an edge/plane intersection. All pairs include every hull edge and
        # extra interior crossings; testing the extras only strengthens proof.
        for i, a in enumerate(xy):
            for b in xy[i + 1:]:
                if a[0] * b[0] < 0:
                    fraction = -a[0] / (b[0] - a[0])
                    crossings.append((0., a[1] + fraction * (b[1] - a[1])))
        for sign in (-1, 1):
            half = [point for point in xy if sign * point[0] >= 0.] + crossings
            if any((vx - sign * offset) ** 2 + vy ** 2 > (radius + tolerance_m) ** 2 for vx, vy in half):
                raise ValueError("robot_link_outside_horizontal_query_envelope:" + item["path"])
        checked.append(item["path"])
    return dict(valid=True, checked_paths=checked, source_scope="actual_full_link_world_AABBs_inside_yaw_query_envelope",
                nominal_tilt_not_used_as_geometry_proof=True, floor_support_does_not_clear_obstacle_voxels=True)
