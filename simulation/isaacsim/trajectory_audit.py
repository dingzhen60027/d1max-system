"""Independent sampled PhysX trajectory/static-geometry audit.

Chassis OBB and caster spheres are tested against certified static AABBs.
Drive wheels use conservative oriented enclosing boxes; an enclosure overlap
is only a possible wheel conflict. Floor-plane contact is allowed. This module
does not determine navigation success or supply controller/movement authority.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from truth_map import StaticPrior, geometry_from_spec

TOLERANCE_M = 1e-6
MAX_EVENTS = 200


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def vec(values, length=3):
    if not isinstance(values, (tuple, list)) or len(values) != length or any(
            type(x) not in (int, float) or not math.isfinite(x) for x in values):
        raise ValueError("invalid_finite_geometry_vector")
    return tuple(float(x) for x in values)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def rotation_xyzw(quaternion):
    x, y, z, w = vec(quaternion, 4)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if abs(norm - 1.) > .01:
        raise ValueError("invalid_measured_quaternion")
    x, y, z, w = (c / norm for c in (x, y, z, w))
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def transform_center(position, rotation, local):
    return tuple(position[i] + dot(rotation[i], local) for i in range(3))


def obb_aabb_signed_axis_gap(center, rotation, half_size, lower, upper):
    """Exact 15-axis box intersection test, with a signed SAT gap in metres.

    Positive maximum projection gap is a lower bound on Euclidean separation,
    NOT an exact shortest distance. A negative value is minus the minimum
    separating-axis translation depth for the two boxes. Zero is touching.
    """
    center, half_size, lower, upper = map(vec, (center, half_size, lower, upper))
    if any(h <= 0 for h in half_size) or any(lo >= hi for lo, hi in zip(lower, upper)):
        raise ValueError("invalid_box_extents")
    world_axes = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
    box_axes = tuple(tuple(rotation[row][column] for row in range(3)) for column in range(3))
    other_center = tuple((lo + hi) / 2 for lo, hi in zip(lower, upper))
    other_half = tuple((hi - lo) / 2 for lo, hi in zip(lower, upper))
    difference = tuple(b - a for a, b in zip(center, other_center))
    axes = [*world_axes, *box_axes]
    axes.extend(cross(a, b) for a in world_axes for b in box_axes)
    best = -math.inf
    for axis in axes:
        norm = math.sqrt(dot(axis, axis))
        if norm <= 1e-12:  # Parallel edges add no new separating axis.
            continue
        unit = tuple(v / norm for v in axis)
        radius_a = sum(h * abs(dot(unit, basis)) for h, basis in zip(half_size, box_axes))
        radius_b = sum(h * abs(c) for h, c in zip(other_half, unit))
        best = max(best, abs(dot(difference, unit)) - radius_a - radius_b)
    return best


def sphere_aabb_signed_clearance(center, radius, lower, upper):
    """Exact signed sphere/AABB clearance (negative geometric exit depth)."""
    center, lower, upper = map(vec, (center, lower, upper))
    if type(radius) not in (float, int) or not math.isfinite(radius) or radius <= 0 or any(lo >= hi for lo, hi in zip(lower, upper)):
        raise ValueError("invalid_sphere_or_box_extents")
    outside = [max(lo - c, 0., c - hi) for c, lo, hi in zip(center, lower, upper)]
    distance = math.sqrt(dot(outside, outside))
    if distance > 0:
        return distance - radius
    return -(radius + min(min(c - lo, hi - c) for c, lo, hi in zip(center, lower, upper)))


def fixture_model(spec, urdf_path):
    """Verify the five authored fixture solids before deriving body-frame shapes."""
    root = ET.parse(urdf_path).getroot()
    expected_names = {"base_link", "left_wheel_link", "right_wheel_link", "front_caster_link", "rear_caster_link"}
    links = {item.attrib["name"]: item for item in root.findall("link")}
    if set(links) != expected_names or len(root.findall("link")) != 5:
        raise ValueError("unsupported_fixture_link_set")
    robot = spec["robot"]
    joint_by_child = {}
    for joint in root.findall("joint"):
        child, parent = joint.find("child"), joint.find("parent")
        if child is None or parent is None or parent.attrib.get("link") != "base_link" or child.attrib.get("link") in joint_by_child:
            raise ValueError("unsupported_fixture_joint_tree")
        joint_by_child[child.attrib["link"]] = joint
    if set(joint_by_child) != expected_names - {"base_link"}:
        raise ValueError("unsupported_fixture_joint_tree")

    def numbers(text, count=3):
        try:
            return vec([float(x) for x in text.split()], count)
        except (TypeError, ValueError, AttributeError) as error:
            raise ValueError("invalid_urdf_geometry") from error

    def origin(item):
        value = item.find("origin")
        return (numbers(value.get("xyz", "0 0 0")), numbers(value.get("rpy", "0 0 0"))) if value is not None else ((0., 0., 0.), (0., 0., 0.))

    def same(a, b):
        a, b = vec(a, len(a)), vec(b, len(b))
        if len(a) != len(b) or any(abs(x - y) > 1e-8 for x, y in zip(a, b)):
            raise ValueError("fixture_spec_urdf_geometry_mismatch")

    solids = []
    for name in sorted(expected_names):
        collisions = links[name].findall("collision")
        if len(collisions) != 1 or collisions[0].find("geometry") is None:
            raise ValueError("unsupported_fixture_collision_set")
        collision = collisions[0]
        geometry = collision.find("geometry")
        if len(geometry) != 1:
            raise ValueError("unsupported_fixture_collision_set")
        local, rpy = origin(collision)
        same(local, (0., 0., 0.))
        if name == "base_link":
            box = geometry.find("box")
            if box is None:
                raise ValueError("unsupported_chassis_geometry")
            size = numbers(box.attrib["size"])
            same(size, vec(robot["body_size"]))
            same(rpy, (0., 0., 0.))
            solids.append(dict(name="chassis", type="obb_exact", center=(0., 0., 0.), half_size=tuple(v / 2 for v in size)))
            continue
        joint = joint_by_child[name]
        joint_center, joint_rpy = origin(joint)
        same(joint_rpy, (0., 0., 0.))
        if "wheel" in name:
            cylinder = geometry.find("cylinder")
            if cylinder is None or joint.attrib["type"] != "continuous":
                raise ValueError("unsupported_wheel_geometry")
            radius, width = float(cylinder.attrib["radius"]), float(cylinder.attrib["length"])
            same((radius, width), (robot["wheel_radius"], robot["wheel_width"]))
            same(rpy, (math.pi / 2, 0., 0.))
            axis = joint.find("axis")
            same(numbers(axis.attrib["xyz"]), (0., 1., 0.))
            sign = 1 if name.startswith("left") else -1
            same(joint_center, (0., sign * robot["wheel_base"] / 2, robot["wheel_axle_z"]))
            solids.append(dict(name=name, type="wheel_obb_enclosure", center=joint_center,
                half_size=(radius, width / 2, radius), cylinder_radius=radius,
                maximum_enclosure_excess_distance_m=(math.sqrt(2.) - 1.) * radius))
        else:
            sphere = geometry.find("sphere")
            if sphere is None or joint.attrib["type"] != "fixed":
                raise ValueError("unsupported_caster_geometry")
            radius = float(sphere.attrib["radius"])
            same((radius,), (robot["caster_radius"],))
            same(rpy, (0., 0., 0.))
            index = 0 if name.startswith("front") else 1
            same(joint_center, vec(robot["caster_origins"][index]))
            solids.append(dict(name=name, type="sphere_exact", center=joint_center, radius=radius))
    return solids


def evaluate_pose(pose, solids, boxes, tolerance_m=TOLERANCE_M):
    """Return sampled exact/enclosure relations; floor is not among these boxes."""
    pose = vec(pose, 7)
    if type(tolerance_m) not in (float, int) or not math.isfinite(tolerance_m) or tolerance_m < 0:
        raise ValueError("invalid_overlap_tolerance")
    rotation = rotation_xyzw(pose[3:])
    result = []
    for solid in solids:
        center = transform_center(pose[:3], rotation, solid["center"])
        for box in boxes:
            gap = sphere_aabb_signed_clearance(center, solid["radius"], box["min"], box["max"]) if solid["type"] == "sphere_exact" else obb_aabb_signed_axis_gap(center, rotation, solid["half_size"], box["min"], box["max"])
            classification = "penetration" if gap < -tolerance_m else "contact_within_tolerance" if gap <= tolerance_m else "separated"
            result.append(dict(solid=solid["name"], solid_model=solid["type"], static_collider=box["path"],
                signed_gap_m=gap, classification=classification,
                gap_semantics="exact_sphere_clearance" if solid["type"] == "sphere_exact" else "SAT_axis_gap_positive_distance_lower_bound_negative_translation_depth"))
    return result


def audit(trajectory_path, spec_path, prior_manifest_path, output, urdf_path=None, tolerance_m=TOLERANCE_M):
    trajectory_path, spec_path, prior_manifest_path = map(Path, (trajectory_path, spec_path, prior_manifest_path))
    prior = StaticPrior(prior_manifest_path).manifest
    spec = json.loads(spec_path.read_text())
    if sha(spec_path) != prior["spec_sha256"]:
        raise ValueError("trajectory_audit_spec_hash_mismatch")
    geometry = geometry_from_spec(spec)
    if geometry != prior["collision_geometry"]:
        raise ValueError("trajectory_audit_static_geometry_mismatch")
    urdf_path = Path(urdf_path) if urdf_path else spec_path.parent / "wheel_fixture.urdf"
    if prior.get("robot_urdf_sha256") != sha(urdf_path):
        raise ValueError("trajectory_audit_urdf_hash_mismatch")
    solids = fixture_model(spec, urdf_path)
    data = trajectory_path.read_bytes()
    rows = data.splitlines()
    if not rows or len(rows) > 1_000_000:
        raise ValueError("empty_or_excessive_trajectory")
    counts, sampled_conflicts, contacts = Counter(), [], []
    samples_with_conflict = {"exact": set(), "wheel_enclosure": set()}
    minima = {"exact": None, "wheel_enclosure": None}
    first_time = previous_time = None
    max_gap_ns = 0
    for index, line in enumerate(rows):
        value = json.loads(line)
        source = value.get("sim_time_ns")
        if type(source) is not int or source < 0 or (previous_time is not None and source <= previous_time):
            raise ValueError("nonmonotonic_actual_trajectory_source_time")
        if first_time is None:
            first_time = source
        if previous_time is not None:
            max_gap_ns = max(max_gap_ns, source - previous_time)
        previous_time = source
        for relation in evaluate_pose(value["pose"], solids, geometry["boxes"], tolerance_m):
            category = "wheel_enclosure" if relation["solid_model"] == "wheel_obb_enclosure" else "exact"
            counts[category + ":" + relation["classification"]] += 1
            record = dict(sample_index=index, sim_time_ns=source, pose=value["pose"], **relation)
            if minima[category] is None or relation["signed_gap_m"] < minima[category]["signed_gap_m"]:
                minima[category] = record
            if relation["classification"] == "penetration":
                samples_with_conflict[category].add(index)
                if len(sampled_conflicts) < MAX_EVENTS:
                    sampled_conflicts.append(record)
            elif relation["classification"] == "contact_within_tolerance" and len(contacts) < MAX_EVENTS:
                contacts.append(record)
    report = dict(schema=1, kind="independent_sampled_static_collision_audit", completed=True,
        trajectory_sha256=hashlib.sha256(data).hexdigest(), spec_sha256=sha(spec_path),
        robot_urdf_sha256=sha(urdf_path), prior_manifest_sha256=sha(prior_manifest_path),
        collider_sha256=prior["collider_sha256"], map_version=prior["map_version"],
        measured_pose_format="world xyz + ROS xyzw; normalized measured quaternion; all roll/pitch/yaw retained",
        sample_count=len(rows), first_sim_time_ns=first_time, last_sim_time_ns=previous_time,
        longest_source_sample_gap_ns=max_gap_ns, overlap_tolerance_m=tolerance_m,
        relation_counts=dict(counts), sampled_exact_penetration_samples=len(samples_with_conflict["exact"]),
        sampled_wheel_enclosure_penetration_samples=len(samples_with_conflict["wheel_enclosure"]),
        sampled_chassis_and_casters_clear=not samples_with_conflict["exact"],
        sampled_wheel_enclosures_clear=not samples_with_conflict["wheel_enclosure"],
        clear_result_semantics="no sampled non-floor penetration deeper than overlap_tolerance_m; contacts and tiny overlaps counted separately",
        minimum_signed_gaps=minima, conflict_events=sampled_conflicts, contact_events=contacts,
        event_limit=MAX_EVENTS, events_truncated=sum(counts[k] for k in counts if k.endswith(":penetration")) > len(sampled_conflicts),
        floor_plane_contact_allowed=True, floor_plane_excluded_from_obstacle_tests=True,
        static_boxes_include_walls_cabinets_low_obstacles_and_ceiling=True,
        collider_models=solids,
        assumptions=["five sealed URDF collision solids; joint centers rigidly fixed in the measured base frame",
            "wheel cylinder axis is local Y; spin around this axis does not change cylinder volume",
            "robot joint solver drift is not independently measured by trajectory.jsonl",
            "positive OBB SAT gap is a Euclidean clearance lower bound, not exact minimum distance",
            "wheel enclosing boxes may report false positives; maximum excess distance is (sqrt(2)-1)*radius",
            "sampled poses only: no continuous/inter-sample collision certificate or PhysX contact-event measurement"],
        navigation_success_evaluated=False, controller_modified=False, movement_authority=False)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--spec", required=True, type=Path)
    parser.add_argument("--prior", required=True, type=Path)
    parser.add_argument("--robot-urdf", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--overlap-tolerance", type=float, default=TOLERANCE_M)
    args = parser.parse_args()
    result = audit(args.trajectory, args.spec, args.prior, args.output, args.robot_urdf, args.overlap_tolerance)
    print(json.dumps({k: result[k] for k in ("completed", "sample_count", "sampled_exact_penetration_samples",
        "sampled_wheel_enclosure_penetration_samples", "navigation_success_evaluated")}))


if __name__ == "__main__":
    main()
