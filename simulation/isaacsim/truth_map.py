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
import copy
import json
import math
from pathlib import Path
import re
import sys
from typing import Sequence

FREE, OCCUPIED, UNKNOWN, SUPPORT_CONTACT = 0, 1, 2, 3
KIND = "certified_static_occupancy_prior"
PROVENANCE = "isaac_closed_collision_geometry_v1"
NUMERICAL_TOLERANCE = 1e-8
DEFAULT_FLOOR_ENDPOINT_ERROR_BOUND_M = 1e-5
# Match the native loader's bounded uint8 volume (256 MiB). Live rolling maps
# stay small; this immutable complete-world artifact is shared by snapshots.
MAX_VOXELS = 256 * 1024 * 1024


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


def _floor_endpoint_error_bound(contact, resolution=None):
    """Bound a declared plane-endpoint measurement error, never voxel geometry."""
    if not isinstance(contact, dict):
        raise ValueError("invalid_flat_support_contact_contract")
    value = _positive(contact.get("floor_endpoint_error_bound_m", DEFAULT_FLOOR_ENDPOINT_ERROR_BOUND_M),
                      "floor_endpoint_error_bound")
    maximum = .001 if resolution is None else min(.001, resolution / 20.)
    if value < DEFAULT_FLOOR_ENDPOINT_ERROR_BOUND_M or value > maximum:
        raise ValueError("floor_endpoint_error_bound_out_of_range")
    return value


def _legacy_robot_registry(spec):
    """Exact legacy paths, never a namespace-wide collider exception."""
    robot = spec["robot"]
    base = "/World/WheelFixture/Geometry/base_link"
    items = [dict(path=base + "/box_1", type="Cube", local_geometry=dict(size=1.),
                  world_scale=list(_vector(robot["body_size"], "robot_body_size")))]
    for name in ("left_wheel_link", "right_wheel_link"):
        items.append(dict(path=base + "/" + name + "/cylinder_1", type="Cylinder",
            local_geometry=dict(radius=robot["wheel_radius"], height=robot["wheel_width"], axis="Z"), world_scale=[1., 1., 1.]))
    for name in ("front_caster_link", "rear_caster_link"):
        items.append(dict(path=base + "/" + name + "/sphere_1", type="Sphere",
            local_geometry=dict(radius=robot["caster_radius"]), world_scale=[1., 1., 1.]))
    return dict(root="/World/WheelFixture", colliders=items)


def _plain(value):
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    return [_plain(v) for v in value]


def _robot_matrix_scale(matrix):
    if any(not math.isfinite(matrix[d][j]) for d in range(4) for j in range(4)):
        raise ValueError("nonfinite_registered_robot_transform")
    scale = [math.sqrt(sum(matrix[d][j] ** 2 for j in range(3))) for d in range(3)]
    if any(v <= 0 for v in scale) or any(abs(sum(matrix[d][j] * matrix[e][j] for j in range(3)) / (scale[d] * scale[e])) > 1e-6
        for d in range(3) for e in range(d + 1, 3)):
        raise ValueError("registered_robot_shear_or_singular_transform")
    return scale


def _local_shape(prim):
    """Intrinsic shape, separate from the changing rigid-link world pose."""
    from pxr import UsdGeom
    kind = prim.GetTypeName()
    names = {"Cube": ("size",), "Sphere": ("radius",), "Cylinder": ("radius", "height", "axis"),
             "Capsule": ("radius", "height", "axis"), "Mesh": ("points", "faceVertexCounts", "faceVertexIndices")}.get(kind)
    if names is None:
        raise ValueError("unsupported_registered_collision_shape:" + str(prim.GetPath()))
    if any(prim.GetAttribute(name).GetNumTimeSamples() for name in names):
        raise ValueError("time_varying_registered_collision_geometry:" + str(prim.GetPath()))
    result = {name: _plain(prim.GetAttribute(name).Get()) for name in names}
    if any(result[name] is None for name in names):
        raise ValueError("missing_registered_collision_geometry:" + str(prim.GetPath()))
    if kind == "Cube":
        half = _positive(result["size"], "cube_size") / 2
        lower, upper = [-half] * 3, [half] * 3
    elif kind == "Sphere":
        radius = _positive(result["radius"], "sphere_radius")
        lower, upper = [-radius] * 3, [radius] * 3
    elif kind in ("Cylinder", "Capsule"):
        radius, height = _positive(result["radius"], "radius"), _positive(result["height"], "height")
        axis = result["axis"]
        if axis not in ("X", "Y", "Z"):
            raise ValueError("unsupported_collision_axis")
        half = [radius] * 3
        half[("X", "Y", "Z").index(axis)] = height / 2 + (radius if kind == "Capsule" else 0.)
        lower, upper = [-v for v in half], half
    else:
        points = result["points"]
        if not points or len(points) > 1_000_000:
            raise ValueError("invalid_collision_mesh_points")
        points = [_vector(p, "mesh_point") for p in points]
        lower, upper = [min(p[d] for p in points) for d in range(3)], [max(p[d] for p in points) for d in range(3)]
    return result, dict(min=lower, max=upper)


def robot_registry_from_stage(stage, robot_root, collider_paths=None):
    """Capture finite geometry identity for the explicitly authorized robot root.

    Capture once immediately after importing the approved robot asset; future
    added paths are never automatically registered. Runtime transforms may
    articulate its links, but registered intrinsic shapes/scale stay identical.
    """
    from pxr import Usd, UsdGeom
    root = stage.GetPrimAtPath(robot_root)
    if not root or not isinstance(robot_root, str) or robot_root == "/World":
        raise ValueError("invalid_authorized_robot_root")
    actual = {str(p.GetPath()): p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
              if "PhysicsCollisionAPI" in p.GetAppliedSchemas() and p.GetAttribute("physics:collisionEnabled").Get() is not False}
    allowed = set(actual) if collider_paths is None else set(collider_paths)
    if not allowed or allowed != set(actual) or len(allowed) > 256:
        raise ValueError("robot_collision_registry_not_complete")
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    colliders = []
    for path in sorted(allowed):
        prim = actual[path]
        ancestor, rigid = prim, False
        while ancestor and (str(ancestor.GetPath()).startswith(robot_root + "/") or ancestor == root):
            rigid |= "PhysicsRigidBodyAPI" in ancestor.GetAppliedSchemas()
            ancestor = ancestor.GetParent()
        if not rigid or prim.GetAttribute("physics:collisionEnabled").Get() is False:
            raise ValueError("unverified_registered_robot_collider:" + path)
        shape, bounds = _local_shape(prim)
        matrix = cache.GetLocalToWorldTransform(prim)
        scale = _robot_matrix_scale(matrix)
        colliders.append(dict(path=path, type=prim.GetTypeName(), local_geometry=shape,
                              local_bounds=bounds, world_scale=scale))
    return dict(root=robot_root, colliders=colliders)


def registered_robot_world_bounds(stage, registry):
    """Actual full-link conservative world bounds, including roll/pitch/joints."""
    from pxr import Gf, Usd, UsdGeom
    registry = _validate_robot_registry(registry)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    result = []
    for item in registry["colliders"]:
        prim = stage.GetPrimAtPath(item["path"])
        if not prim or prim.GetTypeName() != item["type"] or prim.GetAttribute("physics:collisionEnabled").Get() is False:
            raise ValueError("registered_robot_collision_identity_changed:" + item["path"])
        shape, bounds = _local_shape(prim)
        if shape != item["local_geometry"]:
            raise ValueError("registered_robot_shape_changed:" + item["path"])
        matrix = cache.GetLocalToWorldTransform(prim)
        scale = _robot_matrix_scale(matrix)
        if any(abs(a - b) > 1e-6 for a, b in zip(scale, item["world_scale"])):
            raise ValueError("registered_robot_scale_changed:" + item["path"])
        points = [matrix.Transform(Gf.Vec3d(x, y, z)) for x in (bounds["min"][0], bounds["max"][0])
                  for y in (bounds["min"][1], bounds["max"][1]) for z in (bounds["min"][2], bounds["max"][2])]
        result.append(dict(path=item["path"], type=item["type"],
            min=[min(p[d] for p in points) for d in range(3)], max=[max(p[d] for p in points) for d in range(3)],
            corners_world=[[float(p[d]) for d in range(3)] for p in points]))
    return result


def _validate_robot_registry(value):
    if not isinstance(value, dict) or not isinstance(value.get("root"), str) or not value["root"].startswith("/World/"):
        raise ValueError("invalid_robot_collision_registry")
    colliders = value.get("colliders")
    if not isinstance(colliders, list) or not 1 <= len(colliders) <= 256:
        raise ValueError("invalid_robot_collision_registry")
    paths = set()
    for item in colliders:
        path = item.get("path")
        if not isinstance(path, str) or not path.startswith(value["root"] + "/") or path in paths:
            raise ValueError("invalid_robot_collision_path")
        paths.add(path)
        if not isinstance(item.get("local_geometry"), dict) or item.get("type") not in ("Cube", "Sphere", "Cylinder", "Capsule", "Mesh"):
            raise ValueError("invalid_robot_collision_shape")
        if any(v <= 0 for v in _vector(item["world_scale"], "robot_world_scale")):
            raise ValueError("invalid_robot_collision_scale")
    return value


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
    from dynamic_collision import actor_registry, registry_digest
    robot_registry = _validate_robot_registry(spec.get("robot_collision_registry") or _legacy_robot_registry(spec))
    dynamics = actor_registry(spec)
    contact = spec.get("flat_support_contact")
    if contact is not None:
        if not isinstance(contact, dict) or contact.get("enabled") is not True or contact.get("floor_path") != "/World/GroundPlane/collisionPlane" or isinstance(contact.get("floor_z"), bool) or not isinstance(contact.get("floor_z"), (float, int)) or not _close(float(contact["floor_z"]), lower[2]):
            raise ValueError("invalid_flat_support_contact_contract")
        penetration = _positive(contact.get("penetration_allowance_m"), "support_penetration")
        if penetration > .02 or "dynamic_actors" not in spec:
            raise ValueError("flat_support_contact_requires_bounded_dynamic_oracle")
        endpoint_error_bound = _floor_endpoint_error_bound(contact)
        explicit_endpoint_error_bound = "floor_endpoint_error_bound_m" in contact
        contact = dict(schema=1, kind="flat_plane_support_contact_v1", floor_path=contact["floor_path"],
            floor_z=lower[2], penetration_allowance_m=penetration,
            semantics="whole_closed_cell_floor_intersection_inside_authorized_xy_and_disjoint_from_nonfloor_static_solids_plus_margin")
        if explicit_endpoint_error_bound:
            contact["floor_endpoint_error_bound_m"] = endpoint_error_bound
    floor_geometry = dict(path="/World/GroundPlane/collisionPlane", type="Plane", axis="Z", z=lower[2], extent="infinite")
    material = spec["floor"].get("physics_material")
    if material is not None:
        if not isinstance(material, dict) or type(material.get("schema")) is not int or material["schema"] != 1 or material.get("binding_purpose") != "physics" or not isinstance(material.get("path"), str) or not material["path"].startswith("/World/PhysicsMaterials/"):
            raise ValueError("invalid_floor_physics_material_contract")
        values = {}
        for name, maximum in (("static_friction", 10.), ("dynamic_friction", 10.), ("restitution", 1.)):
            value = material.get(name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= maximum:
                raise ValueError("invalid_floor_physics_material_contract")
            values[name] = float(value)
        floor_geometry["physics_material"] = dict(schema=1, path=material["path"], binding_purpose="physics", **values)
    return dict(boxes=sorted(boxes, key=lambda b: b["path"]),
        floor=floor_geometry,
        robot_collision_registry=robot_registry, dynamic_actor_registry=dynamics,
        dynamic_actor_registry_sha256=registry_digest(dynamics),
        flat_support_contact=contact,
        closed_world_bounds=dict(min=list(lower), max=list(upper), semantics="whole_closed_cell_strictly_inside"))


def _audit_stage_geometry(stage, geometry) -> dict:
    """Check all colliders against exact static/robot/actor registrations."""
    from pxr import Gf, Usd, UsdGeom, UsdShade
    if UsdGeom.GetStageUpAxis(stage) != "Z" or not _close(UsdGeom.GetStageMetersPerUnit(stage), 1.):
        raise ValueError("unsupported_stage_units_or_up_axis")
    root = stage.GetRootLayer()
    expected = {b["path"]: b for b in geometry["boxes"]}
    expected[geometry["floor"]["path"]] = geometry["floor"]
    robot_expected = {item["path"]: item for item in geometry["robot_collision_registry"]["colliders"]}
    actor_expected = {shape["path"]: (actor, shape) for actor in geometry["dynamic_actor_registry"] for shape in actor["shapes"]}
    static, dynamic, actors = set(), [], set()
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if "PhysicsCollisionAPI" not in prim.GetAppliedSchemas():
            continue
        path = str(prim.GetPath())
        # Explicitly disabled visual collision APIs contribute no collision
        # geometry. Enabling one later makes it unregistered and revokes the
        # certificate; disabling an EXPECTED collider still fails below.
        if path not in expected and path not in robot_expected and path not in actor_expected and prim.GetAttribute("physics:collisionEnabled").Get() is False:
            continue
        if path in robot_expected:
            # Require the excluded fixture to be explicitly dynamic, not an
            # unexamined static collider merely hidden under a convenient name.
            ancestor, rigid = prim, False
            robot_root = geometry["robot_collision_registry"]["root"]
            while ancestor and (str(ancestor.GetPath()).startswith(robot_root + "/") or str(ancestor.GetPath()) == robot_root):
                rigid |= "PhysicsRigidBodyAPI" in ancestor.GetAppliedSchemas()
                ancestor = ancestor.GetParent()
            if not rigid:
                raise ValueError("unverified_dynamic_robot_exclusion")
            item = robot_expected[path]
            if prim.GetTypeName() != item["type"] or prim.GetAttribute("physics:collisionEnabled").Get() is False:
                raise ValueError("registered_robot_collision_identity_changed:" + path)
            shape, _ = _local_shape(prim)
            if shape != item["local_geometry"]:
                raise ValueError("registered_robot_shape_changed:" + path)
            matrix = cache.GetLocalToWorldTransform(prim)
            scale = _robot_matrix_scale(matrix)
            if any(abs(a - b) > 1e-6 for a, b in zip(scale, item["world_scale"])):
                raise ValueError("registered_robot_scale_changed:" + path)
            dynamic.append(dict(path=path, type=prim.GetTypeName()))
            continue
        if path in actor_expected:
            actor, shape = actor_expected[path]
            if prim.GetTypeName() != ("Cube" if shape["type"] == "box" else "Cylinder") or prim.GetAttribute("physics:collisionEnabled").Get() is False:
                raise ValueError("registered_actor_collision_identity_changed:" + path)
            actor_prim = stage.GetPrimAtPath(actor["path"])
            if not actor_prim or "PhysicsRigidBodyAPI" not in actor_prim.GetAppliedSchemas() or actor_prim.GetAttribute("physics:kinematicEnabled").Get() is not True or actor_prim.GetAttribute("physics:rigidBodyEnabled").Get() is False:
                raise ValueError("registered_actor_not_dynamic:" + path)
            ancestor = prim
            while ancestor and not ancestor.IsPseudoRoot():
                if any(stack.layer != root for stack in ancestor.GetPrimStack()):
                    raise ValueError("actor_geometry_has_external_dependencies:" + path)
                ancestor = ancestor.GetParent()
            local = UsdGeom.Xformable(prim).GetLocalTransformation()
            if any(op.GetAttr().GetNumTimeSamples() for op in UsdGeom.Xformable(prim).GetOrderedXformOps()):
                raise ValueError("time_varying_actor_local_shape:" + path)
            dimensions = shape.get("size", [1., 1., 1.])
            if any(abs(local[d][j] - (dimensions[d] if d == j else 0.)) > 1e-6 for d in range(3) for j in range(3)) or any(abs(local[3][d] - shape["center"][d]) > 1e-6 for d in range(3)):
                raise ValueError("registered_actor_local_transform_changed:" + path)
            intrinsic, _ = _local_shape(prim)
            expected_shape = dict(size=1.) if shape["type"] == "box" else dict(radius=shape["radius"], height=shape["height"], axis="Z")
            if intrinsic != expected_shape:
                raise ValueError("registered_actor_shape_changed:" + path)
            actor_matrix = cache.GetLocalToWorldTransform(actor_prim)
            if any(op.GetAttr().GetNumTimeSamples() for op in UsdGeom.Xformable(actor_prim).GetOrderedXformOps()):
                raise ValueError("time_varying_actor_root_outside_same_step_contract:" + actor["path"])
            if any(abs(sum(actor_matrix[d][j] * actor_matrix[e][j] for j in range(3)) - (1. if d == e else 0.)) > 1e-6 for d in range(3) for e in range(3)):
                raise ValueError("registered_actor_scale_or_shear_changed:" + path)
            actors.add(path)
            continue
        if path not in expected:
            raise ValueError("unlisted_static_collider:" + path)
        item = expected[path]
        material_contract = item.get("physics_material")
        if material_contract:
            relation = prim.GetRelationship("material:binding:physics")
            if not relation or [str(target) for target in relation.GetTargets()] != [material_contract["path"]]:
                raise ValueError("floor_physics_material_binding_changed")
            bound_material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(materialPurpose="physics")
            if not bound_material or str(bound_material.GetPath()) != material_contract["path"]:
                raise ValueError("floor_physics_material_resolved_binding_changed")
            material_prim = bound_material.GetPrim()
            if "PhysicsMaterialAPI" not in material_prim.GetAppliedSchemas() or any(stack.layer != root for stack in material_prim.GetPrimStack()):
                raise ValueError("floor_physics_material_identity_changed")
            fields = {"physics:staticFriction": "static_friction", "physics:dynamicFriction": "dynamic_friction", "physics:restitution": "restitution"}
            for name, field in fields.items():
                attribute = material_prim.GetAttribute(name);value = attribute.Get()
                if attribute.GetNumTimeSamples() or isinstance(value, bool) or not isinstance(value, (int, float)) or not _close(float(value), material_contract[field]):
                    raise ValueError("floor_physics_material_value_changed:" + name)
            if any(a.HasAuthoredValueOpinion() and (a.GetName().startswith("physics:") or a.GetName().startswith("physxMaterial:")) and a.GetName() not in fields for a in material_prim.GetAttributes()):
                raise ValueError("unsealed_floor_physical_material_attribute")
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
    if {item["path"] for item in dynamic} != set(robot_expected):
        raise ValueError("missing_registered_robot_colliders")
    if actors != set(actor_expected):
        raise ValueError("missing_registered_dynamic_actor_colliders")
    return dict(validation="OpenUSD_composed_collision_stage", static_collider_count=len(static),
                excluded_dynamic_robot_colliders=dynamic, up_axis="Z", meters_per_unit=1.,
                registered_dynamic_actor_colliders=sorted(actors), dynamic_veto_required=bool(actors),
                geometric_comparison_tolerance_m=NUMERICAL_TOLERANCE)


def verify_stage_geometry(stage, spec) -> str:
    """Runtime static geometry attestation; never renews a sensor timestamp.

    Only finite registered robot and actor geometry may move. Every actor must
    additionally produce a fresh complete truth-veto snapshot at motion-query
    time. Any missing/extra collider or changed intrinsic shape revokes this
    static identity; the static prior alone cannot certify dynamic emptiness.
    """
    geometry = geometry_from_spec(spec)
    _audit_stage_geometry(stage, geometry)
    return hashlib.sha256(_canonical(geometry)).hexdigest()


class StageGeometryVerifier:
    """Event-invalidated static identity audit; never a periodic blind cache.

    Exact enrolled rigid-link translate/orient/rotation changes and registered
    actor-root translate/rotateZ changes preserve static identity. Existing
    typed robot velocity, joint state and noncollision LiDAR enable attributes
    are measurements, not collision geometry. Only their default-value updates
    are exempt; every other USD change requires a complete audit before reuse.
    Actual full-body corners and same-step actor oracle still need validation
    on every source step.
    """
    def __init__(self, stage, spec):
        from pxr import Sdf, Tf, Usd, UsdGeom, UsdPhysics
        self.stage = stage
        self.geometry = geometry_from_spec(copy.deepcopy(spec))
        self._digest = hashlib.sha256(_canonical(self.geometry)).hexdigest()
        self._dirty, self._serial, self._closed = True, 0, False
        self.full_audits, self.cache_hits = 0, 0
        self._robot_pose_nodes = {self.geometry["robot_collision_registry"]["root"]}
        root = stage.GetPrimAtPath(self.geometry["robot_collision_registry"]["root"])
        robot_prims = list(Usd.PrimRange(root, Usd.TraverseInstanceProxies())) if root else []
        rigid_nodes = {str(prim.GetPath()) for prim in robot_prims
                       if "PhysicsRigidBodyAPI" in prim.GetAppliedSchemas()}
        self._robot_pose_nodes.update(rigid_nodes)
        self._actor_roots = {actor["path"] for actor in self.geometry["dynamic_actor_registry"]}
        # Existing camera/decor poses cannot influence collision if neither
        # that prim nor any descendant carries a CollisionAPI. API additions,
        # prim additions/deletions and composition changes are never exempt.
        collision_ancestors = set()
        for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
            if "PhysicsCollisionAPI" not in prim.GetAppliedSchemas():
                continue
            ancestor = prim
            while ancestor and not ancestor.IsPseudoRoot():
                collision_ancestors.add(str(ancestor.GetPath()));ancestor = ancestor.GetParent()
        self._visual_pose_nodes = {str(prim.GetPath()) for prim in stage.Traverse(Usd.TraverseInstanceProxies())
            if UsdGeom.Xformable(prim) and str(prim.GetPath()) not in collision_ancestors}
        # Freeze exact existing property identities once. A subsequent new
        # attribute/prim/API is never enrolled by a successful full recheck.
        self._measurement_attributes = {}
        lidar_names = {"Lidar_" + str(index) for index, _ in enumerate(spec.get("lidar", {}).get("origins", []))}
        for prim in robot_prims:
            path = str(prim.GetPath())
            if path in rigid_nodes:
                # OpenUSD Physics schemas use vector3f; some Isaac authoring
                # paths use float3. Freeze the exact original type in either
                # case, rather than permitting any three-component alias.
                vector_types = (Sdf.ValueTypeNames.Float3, Sdf.ValueTypeNames.Vector3f)
                candidates = (("physics:velocity", vector_types, "rigid"),
                              ("physics:angularVelocity", vector_types, "rigid"))
            elif prim.IsA(UsdPhysics.Joint):
                candidates = (("state:angular:physics:position", (Sdf.ValueTypeNames.Float,), "joint"),
                              ("state:angular:physics:velocity", (Sdf.ValueTypeNames.Float,), "joint"))
            elif prim.GetName() in lidar_names and str(prim.GetParent().GetPath()) in rigid_nodes and path not in collision_ancestors:
                # The native sensor may be a PhysX Lidar or an RTX Camera.
                # Its exact existing configured link/path/type is the identity.
                candidates = (("enabled", (Sdf.ValueTypeNames.Bool,), "lidar"),)
            else:
                continue
            for name, value_types, role in candidates:
                attribute = prim.GetAttribute(name)
                authored_types = tuple(str(item.typeName) for item in attribute.GetPropertyStack()) if attribute else ()
                if (attribute and attribute.GetTypeName() in value_types and not attribute.GetNumTimeSamples()
                        and all(kind in {str(value_type) for value_type in value_types} for kind in authored_types)):
                    self._measurement_attributes[str(attribute.GetPath())] = (attribute.GetTypeName(), prim.GetTypeName(), role, authored_types)
        self._listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_change, stage)

    def _measurement_update_allowed(self, path):
        from pxr import Usd, UsdPhysics
        identity = self._measurement_attributes.get(str(path))
        if identity is None:
            return False
        value_type, prim_type, role, authored_types = identity
        prim = self.stage.GetPrimAtPath(path.GetPrimPath())
        attribute = prim.GetAttribute(path.name) if prim else None
        if (not attribute or prim.GetTypeName() != prim_type or attribute.GetTypeName() != value_type or attribute.GetNumTimeSamples()
                or tuple(str(item.typeName) for item in attribute.GetPropertyStack()) != authored_types):
            return False
        if role == "rigid":
            return "PhysicsRigidBodyAPI" in prim.GetAppliedSchemas()
        if role == "joint":
            return prim.IsA(UsdPhysics.Joint)
        return not any("PhysicsCollisionAPI" in child.GetAppliedSchemas()
                       for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()))

    def _on_change(self, notice, sender):
        if self._closed:
            return
        changed = bool(notice.GetResyncedPaths())
        if not changed:
            for path in notice.GetChangedInfoOnlyPaths():
                prim_path, name = str(path.GetPrimPath()), path.name
                robot_pose = prim_path in self._robot_pose_nodes and name in (
                    "xformOp:translate", "xformOp:orient", "xformOp:rotateXYZ", "xformOp:rotateX", "xformOp:rotateY", "xformOp:rotateZ")
                actor_pose = prim_path in self._actor_roots and name in ("xformOp:translate", "xformOp:rotateZ")
                visual_pose = prim_path in self._visual_pose_nodes and name.startswith("xformOp:")
                fields = set(notice.GetChangedFields(path))
                measurement = path.IsPropertyPath() and self._measurement_update_allowed(path)
                if not path.IsPropertyPath() or not (robot_pose or actor_pose or visual_pose or measurement) or fields != {"default"}:
                    changed = True
                    break
        if changed:
            self._dirty = True
            self._serial += 1

    def verify(self):
        if self._closed:
            raise ValueError("geometry_verifier_closed")
        if not self._dirty:
            self.cache_hits += 1
            return self._digest
        serial = self._serial
        self.full_audits += 1
        _audit_stage_geometry(self.stage, self.geometry)
        if serial != self._serial:
            raise ValueError("stage_changed_during_geometry_verification")
        self._dirty = False
        return self._digest

    def close(self):
        self._closed, self._dirty = True, True
        self._listener.Revoke()


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
        visual_asset_only = prim.GetTypeName() in ("Shader", "DomeLight") and "PhysicsCollisionAPI" not in prim.GetAppliedSchemas()
        if not visual_asset_only and any(isinstance(a.Get(), Sdf.AssetPath) and a.Get().path for a in prim.GetAttributes()):
            raise ValueError("stage_has_external_dependencies")
    return _audit_stage_geometry(stage, geometry)


def _intersect_range(lower, upper, resolution):
    # Closed voxel boundaries intersect a closed solid even at zero area.
    return (math.ceil((lower - NUMERICAL_TOLERANCE) / resolution) - 1,
            math.floor((upper + NUMERICAL_TOLERANCE) / resolution))


def rasterize(geometry, resolution=.05, margin=.005):
    """Return conservative full-cell labels; byte layout is x,y,z (z fastest)."""
    resolution = _positive(resolution, "voxel_resolution")
    if geometry.get("flat_support_contact"):
        _floor_endpoint_error_bound(geometry["flat_support_contact"], resolution)
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
    if any(n > 100000 for n in shape) or any(o < -(2 ** 31) or o + n > 2 ** 31 - 1 for o, n in zip(origin, shape)) or math.prod(shape) > MAX_VOXELS:
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
    z = geometry["floor"]["z"]
    contact = geometry.get("flat_support_contact")
    if contact:
        # This labels permission to contact this ONE plane; it is never FREE.
        # Painting obstacle uncertainty/OCC after this makes shared low
        # obstacle/floor cells fail closed, including all boundary contacts.
        paint([free_region[0], free_region[1], _intersect_range(z, z, resolution)], SUPPORT_CONTACT)
    for box in boxes:
        paint([_intersect_range(a - margin, b + margin, resolution) for a, b in zip(box["min"], box["max"])], UNKNOWN)
    if not contact:
        paint([ranges[0], ranges[1], _intersect_range(z - margin, z + margin, resolution)], UNKNOWN)
    for box in boxes:
        paint([_intersect_range(a, b, resolution) for a, b in zip(box["min"], box["max"])], OCCUPIED)
    if not contact:
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
    state_codes = dict(free=FREE, occupied=OCCUPIED, unknown=UNKNOWN)
    if geometry.get("flat_support_contact"):
        state_codes["support_contact"] = SUPPORT_CONTACT
    manifest = dict(schema=1, kind=KIND, provenance=PROVENANCE, frame_id=frame_id,
        voxel_resolution=float(resolution), origin_index=origin, shape=shape,
        storage_order="C_xyz_z_fastest", dtype="uint8", state_codes=state_codes,
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
        flat_support_contact=geometry.get("flat_support_contact"), dynamic_actor_registry_sha256=geometry["dynamic_actor_registry_sha256"],
        counts={str(code): data.count(code) for code in state_codes.values()})
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
        state_codes = dict(free=0, occupied=1, unknown=2)
        if m.get("flat_support_contact"):
            state_codes["support_contact"] = 3
            if m["flat_support_contact"] != m["collision_geometry"].get("flat_support_contact") or not isinstance(m["collision_geometry"].get("dynamic_actor_registry"), list):
                raise ValueError("invalid_flat_support_contact_contract")
        if m.get("storage_order") != "C_xyz_z_fastest" or m.get("dtype") != "uint8" or m.get("state_codes") != state_codes:
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
        self.floor_endpoint_error_bound_m = (_floor_endpoint_error_bound(m["flat_support_contact"], self.resolution)
                                            if m.get("flat_support_contact") else None)
        if any(not isinstance(m.get(k), list) or len(m[k]) != 3 for k in ("origin_index", "shape")) or any(type(v) is not int for v in m["origin_index"] + m["shape"]) or any(v <= 0 for v in m["shape"]):
            raise ValueError("invalid_static_prior_dimensions")
        self.origin, self.shape = tuple(m["origin_index"]), tuple(m["shape"])
        if math.prod(self.shape) > MAX_VOXELS or any(n > 100000 for n in self.shape) or any(o < -(2**31) or o > 2**31 - 1 - n for o, n in zip(self.origin, self.shape)):
            raise ValueError("voxel_prior_too_large")
        relative = Path(m["data_file"])
        if len(relative.parts) != 1 or relative.is_absolute() or ".." in relative.parts or (path.parent / relative).resolve().parent != path.parent:
            raise ValueError("unsafe_static_prior_data_path")
        self.data = (path.parent / relative).read_bytes()
        if len(self.data) != math.prod(self.shape) or len(self.data) != m["data_size_bytes"]:
            raise ValueError("static_prior_size_mismatch")
        if hashlib.sha256(self.data).hexdigest() != m["data_sha256"]:
            raise ValueError("static_prior_hash_mismatch")
        allowed = (FREE, OCCUPIED, UNKNOWN, SUPPORT_CONTACT) if m.get("flat_support_contact") else (FREE, OCCUPIED, UNKNOWN)
        if any(v not in allowed for v in set(self.data)):
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
