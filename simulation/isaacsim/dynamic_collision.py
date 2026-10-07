"""Bounded simulation truth vetoes, independent of laser FREE evidence.

Enabled actors are present for the whole session. Registration, shape, world
identity and source time are explicit; absent/unregistered colliders fail closed.
Reachable spheres cover every actor orientation and bounded centre translation;
their outer boxes are only a broad phase. Legacy box messages keep their veto.
This oracle is a simulation test facility, not a deployable perception sensor.
"""
from __future__ import annotations

import hashlib
import copy
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
        registered = dict(id=actor_id, path=f"/World/Dynamic/{actor_id}", shapes=sorted(shapes, key=lambda s: s["id"]),
                           max_linear_speed_mps=number(actor["max_linear_speed_mps"], "actor_speed"),
                           max_linear_acceleration_mps2=number(actor["max_linear_acceleration_mps2"], "actor_acceleration"))
        if "future_motion_contract" in actor:
            registered["scripted_motion"] = scripted_motion_contract(actor)
        actors.append(registered)
    return sorted(actors, key=lambda a: a["id"])


def registry_digest(registry):
    return hashlib.sha256(canonical(registry)).hexdigest()


def native_hit_actor_ids(hit_paths, registry):
    """Attribute only exact native hit prims in the sealed collision registry.

    Zero preserves static/unregistered/unknown hits as blocking observations.
    PhysX may return the enrolled rigid-body prim instead of a collider leaf.
    Both exact identities refer to the complete registered actor; stage audit
    rejects any unregistered collider and its future veto covers every shape.
    No XYZ proximity or namespace-prefix match is used. Ordinals match the
    sorted registry shared by the original projector.
    """
    if registry != sorted(registry, key=lambda actor: actor['id']):
        raise ValueError('native_hit_actor_registry_must_be_sorted')
    paths = {}
    for index, actor in enumerate(registry):
        for path in [actor['path'], *(shape['path'] for shape in actor['shapes'])]:
            if path in paths:
                raise ValueError('ambiguous_native_hit_actor_registry')
            paths[path] = index + 1
    return [paths.get(str(path), 0) for path in hit_paths]


def scripted_motion_contract(actor):
    """Seal this fixture's enforced kinematic script, never infer human intent."""
    contract = copy.deepcopy(actor["future_motion_contract"])
    expected = {"schema", "kind", "source_scope", "actual_pose_error_bound_m",
                "actual_tilt_error_bound_rad", "physics_dt_ns"}
    if (not isinstance(contract, dict) or set(contract) != expected or type(contract["schema"]) is not int
            or contract["schema"] != 1 or contract["kind"] != "sealed_c1_smoothstep_kinematic_v1"
            or contract["source_scope"] != "isolated_simulation_enforced_kinematic_script"
            or type(contract["physics_dt_ns"]) is not int or not 0 < contract["physics_dt_ns"] <= 2_000_000
            or not 0 < number(contract["actual_pose_error_bound_m"], "script_pose_error", True) <= .0001
            or not 0 < number(contract["actual_tilt_error_bound_rad"], "script_tilt_error", True) <= .000001):
        raise ValueError("invalid_scripted_motion_contract")
    trajectory = copy.deepcopy(actor["trajectory"])
    if (trajectory.get("mode") not in ("loop", "once") or trajectory.get("interpolation") != "c1_smoothstep"
            or not isinstance(trajectory.get("waypoints"), list) or not 2 <= len(trajectory["waypoints"]) <= 64):
        raise ValueError("invalid_scripted_motion_trajectory")
    previous = -1.
    for waypoint in trajectory["waypoints"]:
        time_s = number(waypoint["time_s"], "script_time")
        vector(waypoint["position"], "script_position")
        if any(abs(v) > 100_000. for v in waypoint["position"]):
            raise ValueError("script_position_outside_numeric_domain")
        yaw = waypoint["yaw"]
        if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw):
            raise ValueError("invalid_script_yaw")
        if time_s <= previous:
            raise ValueError("nonmonotone_script_waypoints")
        previous = time_s
    points = trajectory["waypoints"]
    if points[0]["time_s"] != 0. or points[-1]["time_s"] <= 0.:
        raise ValueError("invalid_script_time_domain")
    start, end = number(actor["start_time_s"], "script_start"), number(actor["end_time_s"], "script_end")
    if end <= start or (trajectory["mode"] == "loop" and
            (points[0]["position"] != points[-1]["position"] or points[0]["yaw"] != points[-1]["yaw"])):
        raise ValueError("discontinuous_script_seam")
    periods = (end-start)/points[-1]["time_s"]
    if ((trajectory["mode"] == "loop" and abs(periods-round(periods)) > 1e-9)
            or (trajectory["mode"] == "once" and end-start < points[-1]["time_s"])):
        raise ValueError("script_ends_before_continuous_parking_pose")
    contract.update(trajectory=trajectory, start_time_s=start, end_time_s=end)
    return contract


def scripted_center_bounds(motion, simulation_source_ns, horizon_ns):
    """Continuous XYZ enclosure from monotone segments, including every seam.

    Each C1 smoothstep coordinate is monotone on one segment. Endpoints and
    every intervening waypoint therefore bound all intervening times, rather
    than just a sampled collection of predicted poses. One original physical
    step on both ends covers the native BEGIN/END phase and float clock margin.
    """
    if type(simulation_source_ns) is not int or not 0 <= simulation_source_ns <= 1_000_000_000_000_000:
        raise ValueError("scripted_motion_requires_original_simulation_source")
    if type(horizon_ns) is not int or not 0 < horizon_ns <= MAX_REACHABLE_HORIZON_NS:
        raise ValueError("invalid_dynamic_horizon")
    from world_builder import actor_pose
    actor = dict(enabled=True, start_time_s=motion["start_time_s"],
                 end_time_s=motion["end_time_s"], trajectory=motion["trajectory"])
    dt = motion["physics_dt_ns"] * 1e-9
    lower_t = max(0., simulation_source_ns * 1e-9 - dt)
    upper_t = simulation_source_ns * 1e-9 + horizon_ns * 1e-9 + dt
    times = {lower_t, upper_t}
    for edge in (motion["start_time_s"], motion["end_time_s"]):
        if lower_t <= edge <= upper_t:
            times.add(edge)
    points = motion["trajectory"]["waypoints"]
    period = points[-1]["time_s"]
    if motion["trajectory"]["mode"] == "loop":
        first = max(0, math.floor((lower_t - motion["start_time_s"]) / period))
        last = max(0, math.floor((upper_t - motion["start_time_s"]) / period))
    else:
        first = last = 0
    if (last - first + 1) * len(points) > 4096:
        raise ValueError("scripted_future_enclosure_budget_exhausted")
    for loop in range(first, last + 1):
        for point in points:
            stamp = motion["start_time_s"] + loop * period + point["time_s"]
            if lower_t <= stamp <= upper_t and stamp <= motion["end_time_s"]:
                times.add(stamp)
    poses = [actor_pose(actor, stamp)["position"] for stamp in sorted(times)]
    return ([min(p[d] for p in poses) for d in range(3)],
            [max(p[d] for p in poses) for d in range(3)])


def scripted_actor_regions(actor, sample, simulation_source_ns, horizon_ns, margin_m=.02):
    """Full six-second swept solids for an independently sealed actor script.

    Same-step *measured* PhysX position, velocity and orientation are retained
    and checked against that script. A deviation revokes this stronger future
    model. Unscripted actors keep the original unrestricted reachable sphere.
    """
    motion = actor["scripted_motion"]
    low, high = scripted_center_bounds(motion, simulation_source_ns, horizon_ns)
    from world_builder import actor_pose
    expected = actor_pose(dict(enabled=True, start_time_s=motion["start_time_s"],
        end_time_s=motion["end_time_s"], trajectory=motion["trajectory"]), simulation_source_ns * 1e-9)
    actual = vector(sample["position"], "actor_position")
    q = vector(sample["orientation_xyzw"], "actor_orientation", 4)
    if abs(math.fsum(v*v for v in q) - 1.) > 1e-6:
        raise ValueError("scripted_actor_orientation_invalid")
    norm = math.sqrt(math.fsum(v*v for v in q));q = [v/norm for v in q]
    tilt = 2. * math.asin(min(1., math.hypot(q[0], q[1])))
    if (math.dist(actual, expected["position"]) > motion["actual_pose_error_bound_m"]
            or tilt > motion["actual_tilt_error_bound_rad"]):
        raise ValueError("measured_actor_outside_sealed_script")
    low = [min(low[d], actual[d]) for d in range(3)]
    high = [max(high[d], actual[d]) for d in range(3)]
    regions = []
    for shape in actor["shapes"]:
        cx, cy, cz = vector(shape["center"], "shape_center")
        if shape["type"] == "box":
            hx, hy, hz = [v/2. for v in vector(shape["size"], "shape_size")]
            xy = math.hypot(abs(cx) + hx, abs(cy) + hy)
        else:
            xy = math.hypot(cx, cy) + shape["radius"];hz = shape["height"]/2.
        # All yaw angles are covered, including offset shape pivots. The tilt
        # margin covers the allowed native numerical deviation in every XYZ.
        pad = number(margin_m, "dynamic_geometry_margin") + motion["actual_pose_error_bound_m"]
        pad += 2. * shape_radius(shape) * math.sin(motion["actual_tilt_error_bound_rad"] / 2.)
        minimum = [low[0]-xy-pad, low[1]-xy-pad, low[2]+cz-hz-pad]
        maximum = [high[0]+xy+pad, high[1]+xy+pad, high[2]+cz+hz+pad]
        regions.append(dict(state=2, min=[math.nextafter(v, -math.inf) for v in minimum],
                            max=[math.nextafter(v, math.inf) for v in maximum]))
    return regions


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
    # The unchanged radius proves a sphere, not every corner of its enclosing
    # cube. Retain that cube as a broad phase and for conservative old readers.
    return dict(state=2, min=[v - radius for v in position], max=[v + radius for v in position],
                enclosure="sphere_v1", center=list(position), radius=radius)


def oracle_payload(registry, samples, *, session_id, epoch, seed_id, context_sequence,
                   sequence, source_stamp_ns, frame_id="d1max_loc_odom", horizon_ns=MAX_HORIZON_NS,
                   reachable_horizon_ns=MAX_HORIZON_NS, simulation_source_stamp_ns=None):
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
        reachable = reachable_actor_region(actor, sample["position"], sample["linear_velocity"], reachable_horizon_ns)
        regions = (scripted_actor_regions(actor, sample, simulation_source_stamp_ns, reachable_horizon_ns)
                   if "scripted_motion" in actor else [reachable])
        actors.append(dict(actor_id=actor["id"], regions=regions))
    return dict(schema=1, kind=KIND, session_id=session_id, epoch=epoch, seed_id=seed_id,
        context_sequence=context_sequence, sequence=sequence, frame_id=frame_id,
        registry_sha256=registry_digest(registry), source_stamp_ns=source_stamp_ns,
        valid_until_ns=source_stamp_ns + horizon_ns, complete=True, actors=actors,
        reachable_horizon_ns=reachable_horizon_ns, reachable_until_ns=source_stamp_ns + reachable_horizon_ns,
        source_scope="isolated_simulation_same_step_dynamic_collision_truth_not_lidar")


def closed_aabb_intersects(a_lower, a_upper, b_lower, b_upper):
    return all(a_lower[d] <= b_upper[d] and b_lower[d] <= a_upper[d] for d in range(3))


def region_voxel_intersects(region, lower, upper):
    """Closed full XYZ narrow phase; legacy boxes retain their original veto."""
    region_lower, region_upper = vector(region["min"], "region_min"), vector(region["max"], "region_max")
    if any(a >= b for a, b in zip(region_lower, region_upper)):
        raise ValueError("invalid_dynamic_region_bounds")
    if "enclosure" not in region:
        if "center" in region or "radius" in region:
            raise ValueError("incomplete_dynamic_sphere_enclosure")
        return closed_aabb_intersects(lower, upper, region_lower, region_upper)
    enclosure = region["enclosure"]
    if enclosure != "sphere_v1":
        raise ValueError("unsupported_dynamic_enclosure")
    center = vector(region["center"], "sphere_center")
    radius = number(region["radius"], "sphere_radius", True)
    if any(a != c - radius or b != c + radius for a, b, c in zip(region_lower, region_upper, center)):
        raise ValueError("sphere_broad_phase_mismatch")
    # Outward voxel faces and a rounding allowance keep tangency occupied.
    # They do not change the declared physical/prediction radius.
    lower = [math.nextafter(v, -math.inf) for v in lower]
    upper = [math.nextafter(v, math.inf) for v in upper]
    if not closed_aabb_intersects(lower, upper, region_lower, region_upper):
        return False
    distance_squared = math.fsum(max(a - c, 0., c - b) ** 2 for a, b, c in zip(lower, upper, center))
    radius_squared = radius * radius
    guard = 16 * math.ulp(max(1., distance_squared, radius_squared))
    return distance_squared <= radius_squared + guard


def voxel_veto(payload, cell, resolution=.05):
    """Offline geometry audit only; production also validates identity/leases."""
    if len(cell) != 3 or any(type(v) is not int for v in cell):
        raise ValueError("invalid_voxel_index")
    resolution = number(resolution, "resolution", True)
    lower, upper = [v * resolution for v in cell], [(v + 1) * resolution for v in cell]
    state = 0
    for actor in payload["actors"]:
        for region in actor["regions"]:
            if region_voxel_intersects(region, lower, upper):
                if region["state"] == 1:
                    return 1
                state = 2
    return state


def certify_robot_in_body_envelope(collider_world_bounds, *, body_position, body_orientation_xyzw,
                                   radius, offset, below, above, floor_z, support_paths=(), tolerance_m=1e-6):
    """Prove every complete link enclosure fits the query double cylinder.

    This is deliberately stronger than checking only nominal roll/pitch/height.
    Each complete primitive support box is clipped at body-local x=0.
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
    return dict(valid=True, checked_paths=checked, source_scope="actual_full_link_enclosures_inside_yaw_query_envelope",
                nominal_tilt_not_used_as_geometry_proof=True, floor_support_does_not_clear_obstacle_voxels=True)
