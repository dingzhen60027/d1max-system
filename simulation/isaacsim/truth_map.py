"""Certified static voxel prior from this fixture's *actual* collision stage.

No simulator, ROS node, ray synthesis, or measured-free timestamps are involved.
OpenUSD is needed only by build()/verify_stage(); loading/querying uses stdlib.
The source is a declared complete, closed simulation room. It is not a way to
certify empty real-world space from obstacle surfaces or a robot self mask.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from typing import Sequence

FREE, OCCUPIED, UNKNOWN = 0, 1, 2
KIND = "certified_static_occupancy_prior"
PROVENANCE = "isaac_closed_collision_geometry_v1"
NUMERICAL_TOLERANCE = 1e-8
MAX_VOXELS = 20_000_000


def _canonical(value) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _vector(value, label, length=3):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("invalid_" + label)
    if any(not isinstance(v, (float, int)) or isinstance(v, bool) for v in value):
        raise ValueError("invalid_" + label)
    result = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in result):
        raise ValueError("nonfinite_" + label)
    return result


def _positive(value, label):
    if not isinstance(value, (float, int)) or isinstance(value, bool):
        raise ValueError("invalid_" + label)
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError("invalid_" + label)
    return result


def _close(a, b):
    return abs(a - b) <= NUMERICAL_TOLERANCE


def geometry_from_spec(spec: dict) -> dict:
    """Declare a bounded closed room; verify_stage must corroborate its solids."""
    if type(spec.get("schema")) is not int or spec.get("schema") != 1 or spec.get("units") != "m":
        raise ValueError("unsupported_scene_spec")
    room, floor, ceiling = spec["room"], spec["floor"], spec["ceiling"]
    lower = _vector([room["x_min"], room["y_min"], floor["z"]], "room_min")
    upper = _vector([room["x_max"], room["y_max"], ceiling["z"]], "room_max")
    if any(b <= a for a, b in zip(lower, upper)):
        raise ValueError("invalid_room_bounds")
    if not (_close(_positive(room["width"], "room_width"), upper[0] - lower[0])
            and _close(_positive(room["depth"], "room_depth"), upper[1] - lower[1])
            and _close(_positive(room["height"], "room_height"), upper[2] - lower[2])):
        raise ValueError("inconsistent_room_dimensions")
    # The current scene authors its ceiling at x=y=0. Do not extrapolate that
    # authoring rule to an offset room or an uncorroborated floor extent.
    if not (_close(lower[0] + upper[0], 0.) and _close(lower[1] + upper[1], 0.)):
        raise ValueError("unsupported_offset_room")
    if _vector(floor["bounds"], "floor_bounds", 4) != (lower[0], upper[0], lower[1], upper[1]):
        raise ValueError("inconsistent_floor_domain")
    thickness = _positive(ceiling["thickness"], "ceiling_thickness")
    boxes = []
    names = set()
    for item in spec["static_boxes"]:
        name = item["name"]
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name) or name in names:
            raise ValueError("invalid_or_duplicate_static_name")
        names.add(name)
        center, size = _vector(item["center"], "box_center"), _vector(item["size"], "box_size")
        if any(s <= 0 for s in size):
            raise ValueError("invalid_box_size")
        boxes.append(dict(path="/World/Indoor/" + name, type="Cube",
            min=[c - s / 2 for c, s in zip(center, size)],
            max=[c + s / 2 for c, s in zip(center, size)]))
    if "ceiling" in names:
        raise ValueError("duplicate_ceiling")
    boxes.append(dict(path="/World/Indoor/ceiling", type="Cube",
        min=[lower[0], lower[1], upper[2]], max=[upper[0], upper[1], upper[2] + thickness]))
    # A complete environment claim needs actual enclosing walls, not just room
    # numbers in JSON. Require their inner faces, full height and full spans.
    by_name = {b["path"].rsplit("/", 1)[-1]: b for b in boxes}
    for name, axis, side in (("west_wall", 0, 0), ("east_wall", 0, 1),
                             ("south_wall", 1, 0), ("north_wall", 1, 1)):
        wall = by_name.get(name)
        if wall is None:
            raise ValueError("missing_room_enclosure")
        inner = wall["max" if side == 0 else "min"][axis]
        if not _close(inner, (lower if side == 0 else upper)[axis]):
            raise ValueError("incomplete_room_enclosure")
        for other in set(range(3)) - {axis}:
            if wall["min"][other] > lower[other] + NUMERICAL_TOLERANCE or wall["max"][other] < upper[other] - NUMERICAL_TOLERANCE:
                raise ValueError("incomplete_room_enclosure")
    return dict(boxes=sorted(boxes, key=lambda b: b["path"]),
        floor=dict(path="/World/GroundPlane/collisionPlane", type="Plane", axis="Z", z=lower[2], extent="infinite"),
        closed_world_bounds=dict(min=list(lower), max=list(upper), semantics="whole_closed_cell_strictly_inside"))


def _audit_stage_geometry(stage, geometry) -> dict:
    """Check every actual collider; only the named dynamic fixture is excluded."""
    from pxr import Gf, Usd, UsdGeom
    if UsdGeom.GetStageUpAxis(stage) != "Z" or not _close(UsdGeom.GetStageMetersPerUnit(stage), 1.):
        raise ValueError("unsupported_stage_units_or_up_axis")
    root = stage.GetRootLayer()
    expected = {b["path"]: b for b in geometry["boxes"]}
    expected[geometry["floor"]["path"]] = geometry["floor"]
    static, dynamic = set(), []
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if "PhysicsCollisionAPI" not in prim.GetAppliedSchemas():
            continue
        path = str(prim.GetPath())
        if path.startswith("/World/WheelFixture/"):
            # Require the excluded fixture to be explicitly dynamic, not an
            # unexamined static collider merely hidden under a convenient name.
            ancestor, rigid = prim, False
            while ancestor and not ancestor.IsPseudoRoot():
                rigid |= "PhysicsRigidBodyAPI" in ancestor.GetAppliedSchemas()
                ancestor = ancestor.GetParent()
            if not rigid:
                raise ValueError("unverified_dynamic_robot_exclusion")
            dynamic.append(dict(path=path, type=prim.GetTypeName()))
            continue
        if path not in expected:
            raise ValueError("unlisted_static_collider:" + path)
        item = expected[path]
        if prim.GetTypeName() != item["type"]:
            raise ValueError("static_collider_type_mismatch:" + path)
        enabled = prim.GetAttribute("physics:collisionEnabled").Get()
        if enabled is False:
            raise ValueError("disabled_static_collider:" + path)
        ancestor = prim
        while ancestor and not ancestor.IsPseudoRoot():
            # Imported robot layers may change, but static collision geometry
            # and its ancestor transforms must be authored in the current root.
            # No unresolved/external static composition can certify free space.
            if any(item.layer != root for item in ancestor.GetPrimStack()):
                raise ValueError("static_geometry_has_external_dependencies:" + path)
            for key in ("references", "payload"):
                opinion = ancestor.GetMetadata(key)
                if opinion and any(item.assetPath for item in opinion.GetAppliedItems()):
                    raise ValueError("static_geometry_has_external_dependencies:" + path)
            if "PhysicsRigidBodyAPI" in ancestor.GetAppliedSchemas():
                raise ValueError("dynamic_static_collider:" + path)
            if any(a.GetNumTimeSamples() for a in ancestor.GetAttributes() if a.GetName().startswith("xformOp:") or a.GetName() in ("size", "physics:collisionEnabled")):
                raise ValueError("time_varying_static_collider:" + path)
            ancestor = ancestor.GetParent()
        matrix = cache.GetLocalToWorldTransform(prim)
        if any(abs(matrix[i][j]) > NUMERICAL_TOLERANCE for i in range(3) for j in range(3) if i != j) or any(matrix[i][i] <= 0 for i in range(3)):
            raise ValueError("tilted_or_reflected_static_collider:" + path)
        if any(abs(matrix[i][3]) > NUMERICAL_TOLERANCE for i in range(3)) or not _close(matrix[3][3], 1.):
            raise ValueError("nonaffine_static_collider:" + path)
        if item["type"] == "Cube":
            size = float(UsdGeom.Cube(prim).GetSizeAttr().Get())
            points = [matrix.Transform(Gf.Vec3d(x * size / 2, y * size / 2, z * size / 2))
                      for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
            for axis in range(3):
                if not (_close(min(p[axis] for p in points), item["min"][axis]) and _close(max(p[axis] for p in points), item["max"][axis])):
                    raise ValueError("static_collider_geometry_mismatch:" + path)
        elif prim.GetAttribute("axis").Get() != "Z" or not _close(matrix[3][2], item["z"]):
            raise ValueError("floor_plane_mismatch")
        static.add(path)
    if static != set(expected):
        raise ValueError("missing_static_colliders")
    if len(dynamic) != 5:
        raise ValueError("unexpected_robot_collider_count")
    return dict(validation="OpenUSD_composed_collision_stage", static_collider_count=len(static),
                excluded_dynamic_robot_colliders=dynamic, up_axis="Z", meters_per_unit=1.,
                geometric_comparison_tolerance_m=NUMERICAL_TOLERANCE)


def verify_stage_geometry(stage, spec) -> str:
    """Runtime static geometry attestation; never renews a sensor timestamp.

    Normal robot motion/imported robot layers are permitted. Any additional
    non-robot collider, including a new dynamic obstacle, invalidates this closed
    world assumption. Callers revoke their localization/session epoch on error.
    """
    geometry = geometry_from_spec(spec)
    _audit_stage_geometry(stage, geometry)
    return hashlib.sha256(_canonical(geometry)).hexdigest()


def verify_stage(usd_path, geometry) -> dict:
    """Audit a flattened source artifact without creating SimulationApp."""
    from pxr import Sdf, Usd
    usd_path = Path(usd_path).resolve()
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise ValueError("cannot_open_collision_stage")
    root = stage.GetRootLayer()
    if root.subLayerPaths or any(layer.realPath and Path(layer.realPath).resolve() != usd_path for layer in stage.GetUsedLayers()):
        raise ValueError("stage_has_external_dependencies")
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        for key in ("references", "payload"):
            opinion = prim.GetMetadata(key)
            if opinion and any(item.assetPath for item in opinion.GetAppliedItems()):
                raise ValueError("stage_has_external_dependencies")
        if any(isinstance(a.Get(), Sdf.AssetPath) and a.Get().path for a in prim.GetAttributes()):
            raise ValueError("stage_has_external_dependencies")
    return _audit_stage_geometry(stage, geometry)


def _intersect_range(lower, upper, resolution):
    # Closed voxel boundaries intersect a closed solid even at zero area.
    return (math.ceil((lower - NUMERICAL_TOLERANCE) / resolution) - 1,
            math.floor((upper + NUMERICAL_TOLERANCE) / resolution))


def rasterize(geometry, resolution=.05, margin=.005):
    """Return conservative full-cell labels; byte layout is x,y,z (z fastest)."""
    resolution = _positive(resolution, "voxel_resolution")
    if not isinstance(margin, (float, int)) or isinstance(margin, bool):
        raise ValueError("invalid_geometry_margin")
    margin = float(margin)
    if not math.isfinite(margin) or margin < 0:
        raise ValueError("invalid_geometry_margin")
    bounds, boxes = geometry["closed_world_bounds"], geometry["boxes"]
    lower = [min([bounds["min"][i]] + [b["min"][i] for b in boxes]) for i in range(3)]
    upper = [max([bounds["max"][i]] + [b["max"][i] for b in boxes]) for i in range(3)]
    ranges = [_intersect_range(a, b, resolution) for a, b in zip(lower, upper)]
    origin = [r[0] for r in ranges]
    shape = [r[1] - r[0] + 1 for r in ranges]
    if math.prod(shape) > MAX_VOXELS:
        raise ValueError("voxel_prior_too_large")
    data = bytearray([UNKNOWN]) * math.prod(shape)

    def paint(region, code):
        starts = [max(region[i][0], origin[i]) - origin[i] for i in range(3)]
        stops = [min(region[i][1], origin[i] + shape[i] - 1) - origin[i] + 1 for i in range(3)]
        if any(a >= b for a, b in zip(starts, stops)):
            return
        segment = bytes([code]) * (stops[2] - starts[2])
        for x in range(starts[0], stops[0]):
            for y in range(starts[1], stops[1]):
                address = (x * shape[1] + y) * shape[2] + starts[2]
                data[address:address + len(segment)] = segment

    free_region = []
    for lo, hi in zip(bounds["min"], bounds["max"]):
        lo, hi = lo + margin + NUMERICAL_TOLERANCE, hi - margin - NUMERICAL_TOLERANCE
        # lo < cell.min AND cell.max < hi, not just a free center/corners.
        free_region.append((math.floor(lo / resolution) + 1, math.ceil(hi / resolution) - 2))
    paint(free_region, FREE)
    for box in boxes:
        paint([_intersect_range(a - margin, b + margin, resolution) for a, b in zip(box["min"], box["max"])], UNKNOWN)
    z = geometry["floor"]["z"]
    paint([ranges[0], ranges[1], _intersect_range(z - margin, z + margin, resolution)], UNKNOWN)
    for box in boxes:
        paint([_intersect_range(a, b, resolution) for a, b in zip(box["min"], box["max"])], OCCUPIED)
    paint([ranges[0], ranges[1], _intersect_range(z, z, resolution)], OCCUPIED)
    return data, origin, shape


def build(spec_path, usd_path, output, frame_id="d1max_loc_map", resolution=.05, margin=.005, map_version=None):
    """Build JSON manifest + relative uint8 binary; return their absolute paths."""
    spec_path, usd_path, output = Path(spec_path).resolve(), Path(usd_path).resolve(), Path(output).resolve()
    if output.suffix != ".json":
        raise ValueError("manifest_output_must_be_json")
    if not isinstance(frame_id, str) or not frame_id or frame_id.startswith("/"):
        raise ValueError("invalid_map_frame")
    if map_version is not None and (not isinstance(map_version, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,200}", map_version)):
        raise ValueError("invalid_map_version")
    spec = json.loads(spec_path.read_text())
    geometry = geometry_from_spec(spec)
    audit = verify_stage(usd_path, geometry)
    data, origin, shape = rasterize(geometry, resolution, margin)
    collider_sha = hashlib.sha256(_canonical(geometry)).hexdigest()
    grid_sha = hashlib.sha256(_canonical(dict(resolution=resolution, margin=margin, origin=origin, shape=shape))).hexdigest()
    data_path = output.with_suffix(".bin")
    urdf = spec_path.parent / "wheel_fixture.urdf"
    manifest = dict(schema=1, kind=KIND, provenance=PROVENANCE, frame_id=frame_id,
        voxel_resolution=float(resolution), origin_index=origin, shape=shape,
        storage_order="C_xyz_z_fastest", dtype="uint8", state_codes=dict(free=FREE, occupied=OCCUPIED, unknown=UNKNOWN),
        data_file=data_path.name, data_sha256=hashlib.sha256(data).hexdigest(), data_size_bytes=len(data),
        scene_sha256=_sha(usd_path), spec_sha256=_sha(spec_path), collider_sha256=collider_sha,
        source_sha256=collider_sha, robot_urdf_sha256=_sha(urdf) if urdf.is_file() else None,
        map_version=map_version or "isaac-static-" + collider_sha[:16] + "-" + grid_sha[:16],
        geometry_version=collider_sha, grid_version=grid_sha,
        closed_world_bounds=geometry["closed_world_bounds"], geometry_margin_m=float(margin),
        up_axis="Z", meters_per_unit=1., map_from_odom=dict(transform_contract="fixed_identity_map_from_odom_v1",
            from_frame="d1max_loc_odom", to_frame=frame_id,
            translation=[0., 0., 0.], rotation_xyzw=[0., 0., 0., 1.]),
        identity_contract="fixed_sim_world_equals_odom_equals_map; consumer_binds_session_seed_map_version_localization_epoch",
        occupancy_semantics="any_closed_full_cell_intersection_or_boundary_contact_with_static_solid_or_floor_plane",
        free_semantics="whole_closed_cell_inside_declared_complete_room_and_disjoint_from_all_static_colliders_plus_margin",
        unknown_semantics="outside_authorized_closed_world_or_uncertainty_margin; never_clear_from_robot_ownership",
        overlay_policy="live_hit_or_uncertain_evidence_precedes_prior_free; static_occupied_never_erased_by_live_free",
        source_scope="isolated_simulation_complete_static_geometry_only; not_measured_lidar_free",
        collision_geometry=geometry, stage_audit=audit,
        counts={str(code): data.count(code) for code in (FREE, OCCUPIED, UNKNOWN)})
    output.parent.mkdir(parents=True, exist_ok=True)
    data_path.write_bytes(data)
    output.write_text(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return dict(manifest=str(output), data=str(data_path), map_version=manifest["map_version"],
                collider_sha256=collider_sha, counts=manifest["counts"])


class StaticPrior:
    """Verified file lookup in global world indices, unrelated to ring addresses."""
    def __init__(self, manifest_path):
        path = Path(manifest_path).resolve()
        self.manifest = m = json.loads(path.read_text())
        if m.get("schema") != 1 or m.get("kind") != KIND or m.get("provenance") != PROVENANCE:
            raise ValueError("unsupported_static_prior")
        if m.get("storage_order") != "C_xyz_z_fastest" or m.get("dtype") != "uint8" or m.get("state_codes") != dict(free=0, occupied=1, unknown=2):
            raise ValueError("unsupported_static_prior_storage")
        identity = dict(transform_contract="fixed_identity_map_from_odom_v1", from_frame="d1max_loc_odom",
            to_frame=m.get("frame_id"), translation=[0., 0., 0.], rotation_xyzw=[0., 0., 0., 1.])
        if m.get("map_from_odom") != identity or not isinstance(m.get("frame_id"), str) or not m["frame_id"] or m["frame_id"].startswith("/") or m.get("up_axis") != "Z" or m.get("meters_per_unit") != 1.:
            raise ValueError("unsupported_static_prior_coordinates")
        if any(not isinstance(m.get(k), str) or not re.fullmatch(r"[0-9a-f]{64}", m[k]) for k in ("data_sha256", "scene_sha256", "spec_sha256", "collider_sha256")):
            raise ValueError("invalid_static_prior_hash")
        if hashlib.sha256(_canonical(m["collision_geometry"])).hexdigest() != m["collider_sha256"] or m["closed_world_bounds"] != m["collision_geometry"]["closed_world_bounds"]:
            raise ValueError("static_prior_geometry_hash_mismatch")
        if not isinstance(m.get("map_version"), str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,200}", m["map_version"]):
            raise ValueError("invalid_map_version")
        self.resolution = _positive(m["voxel_resolution"], "voxel_resolution")
        if any(not isinstance(m.get(k), list) or len(m[k]) != 3 for k in ("origin_index", "shape")) or any(type(v) is not int for v in m["origin_index"] + m["shape"]) or any(v <= 0 for v in m["shape"]):
            raise ValueError("invalid_static_prior_dimensions")
        self.origin, self.shape = tuple(m["origin_index"]), tuple(m["shape"])
        if math.prod(self.shape) > MAX_VOXELS:
            raise ValueError("voxel_prior_too_large")
        relative = Path(m["data_file"])
        if len(relative.parts) != 1 or relative.is_absolute() or ".." in relative.parts or (path.parent / relative).resolve().parent != path.parent:
            raise ValueError("unsafe_static_prior_data_path")
        self.data = (path.parent / relative).read_bytes()
        if len(self.data) != math.prod(self.shape) or len(self.data) != m["data_size_bytes"]:
            raise ValueError("static_prior_size_mismatch")
        if hashlib.sha256(self.data).hexdigest() != m["data_sha256"]:
            raise ValueError("static_prior_hash_mismatch")
        if any(v not in (FREE, OCCUPIED, UNKNOWN) for v in set(self.data)):
            raise ValueError("invalid_static_prior_state")

    def status(self, global_index: Sequence[int]) -> int:
        if len(global_index) != 3 or any(not isinstance(v, int) or isinstance(v, bool) for v in global_index):
            raise ValueError("invalid_global_voxel_index")
        relative = [i - o for i, o in zip(global_index, self.origin)]
        if any(i < 0 or i >= n for i, n in zip(relative, self.shape)):
            return UNKNOWN
        return self.data[(relative[0] * self.shape[1] + relative[1]) * self.shape[2] + relative[2]]


def query_stats(manifest_path, native_query_path, output):
    """Compare certified prior with every exported actual native query voxel."""
    prior = StaticPrior(manifest_path)
    replies = json.loads(Path(native_query_path).read_text())
    reports = []
    for reply in replies:
        for query in reply.get("queries", []):
            if query.get("export_truncated") or len(query["voxels"]) != query["unique_voxel_count"]:
                raise ValueError("native_query_export_truncated")
            counts, never_counts = Counter(), Counter()
            for voxel in query["voxels"]:
                state = prior.status(voxel["global_index"])
                counts[state] += 1
                if voxel["classification"] == "never_observed":
                    never_counts[state] += 1
            reports.append(dict(label=query["label"], native_diagnostic_counts=query["diagnostic_counts"],
                prior_state_counts={str(k): counts[k] for k in (0, 1, 2)},
                native_never_observed_prior_counts={str(k): never_counts[k] for k in (0, 1, 2)},
                voxel_count=len(query["voxels"])))
    result = dict(schema=1, kind="offline_prior_query_comparison", native_query_sha256=_sha(Path(native_query_path)),
        prior_manifest_sha256=_sha(Path(manifest_path)), map_version=prior.manifest["map_version"],
        map_labels_written_to_native=False, measured_cloud_modified=False, live_overlay_evaluated=False, queries=reports)
    Path(output).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    generate = sub.add_parser("build")
    generate.add_argument("--spec", type=Path, required=True)
    generate.add_argument("--usd", type=Path, required=True)
    generate.add_argument("--output", type=Path, required=True)
    generate.add_argument("--frame-id", default="d1max_loc_map")
    generate.add_argument("--resolution", type=float, default=.05)
    generate.add_argument("--margin", type=float, default=.005)
    generate.add_argument("--map-version", default=None)
    compare = sub.add_parser("query-stats")
    compare.add_argument("--manifest", type=Path, required=True)
    compare.add_argument("--native-query", type=Path, required=True)
    compare.add_argument("--output", type=Path, required=True)
    # Also allow the build-facing CLI requested by the candidate assembler:
    # truth_map.py --spec ... --usd ... --output ...
    arguments = sys.argv[1:]
    if arguments and arguments[0].startswith("--"):
        arguments.insert(0, "build")
    args = parser.parse_args(arguments)
    if args.command == "build":
        result = build(args.spec, args.usd, args.output, args.frame_id, args.resolution, args.margin, args.map_version)
    else:
        result = query_stats(args.manifest, args.native_query, args.output)
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
