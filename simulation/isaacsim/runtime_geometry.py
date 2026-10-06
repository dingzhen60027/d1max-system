"""Actual PhysX primitive transforms to a same-source whole-body certificate."""
import hashlib
import itertools

import numpy as np

from dynamic_collision import canonical, certify_robot_in_body_envelope


def floor_endpoint_certificate(points_body, pose, hit_paths, contact):
    """Audit raw native floor hits against a sealed numerical error bound.

    Native prim identity is checked at the same acquisition. Neither endpoints
    nor their timestamps are changed; this is separate from laser FREE.
    """
    points = np.asarray(points_body, dtype=float)
    paths = np.asarray(hit_paths, dtype=object)
    pose = np.asarray(pose, dtype=float)
    bound = contact['floor_endpoint_error_bound_m']
    if (points.ndim != 2 or points.shape[1] != 3 or paths.shape != (len(points),)
            or pose.shape != (7,) or not np.isfinite(points).all()
            or not np.isfinite(pose).all() or any(not isinstance(p, str) or not p for p in paths)
            or type(bound) not in (int, float) or not 0 < bound <= .001):
        raise ValueError('invalid_native_floor_hit_identity')
    q = pose[3:]
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError('invalid_native_floor_hit_pose')
    x, y, z, w = q / norm
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    floor = paths == contact['floor_path']
    heights = points[floor] @ rotation[2] + pose[2] - contact['floor_z']
    error = float(np.max(np.abs(heights))) if len(heights) else 0.
    if error > bound:
        raise ValueError('native_floor_endpoint_error_exceeds_sealed_bound')
    return dict(native_floor_hits=int(floor.sum()), floor_endpoint_max_abs_error_m=error)


def measured_bounds(snapshot):
    """Enclose each complete measured solid in its world-axis tight AABB.

    Transforming a sphere's or capsule's *local AABB* rotates artificial box
    corners outside the solid. Use the primitive support function after the
    measured affine transform instead: row norms for the ball, plus the axial
    segment for a capsule, and row-wise absolute sums for a cube. Those bounds
    include rotation, nonuniform scale and shear. Their eight world corners
    conservatively enclose the complete solid for the body-envelope proof.
    """
    from quadruped import primitive_aabb
    result = []
    for item in snapshot:
        matrix = np.asarray(item['world_matrix'], dtype=float)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError('invalid_actual_link_transform')
        lower, upper = primitive_aabb(item['shape'], matrix)
        corners = np.asarray(list(itertools.product(*zip(lower, upper))))
        result.append(dict(path=item['path'], min=lower.tolist(),
            max=upper.tolist(), corners_world=corners.tolist()))
    return result


def body_certificate(snapshot, pose, registry, envelope, source_ns):
    bounds = measured_bounds(snapshot)
    if {v['path'] for v in bounds} != {v['path'] for v in registry['colliders']}:
        raise ValueError('actual_link_registry_incomplete')
    # Explicit sphere support solids from this sealed model only. Capsules and
    # body boxes retain the strict above-plane requirement.
    supports = [v['path'] for v in registry['colliders'] if v['type'] == 'Sphere']
    floor = envelope['support_floor_z']
    certify_robot_in_body_envelope(bounds, body_position=pose[:3],
        body_orientation_xyzw=pose[3:], radius=envelope['radius'],
        offset=envelope['offset'], below=pose[2]-floor+envelope['support_penetration_m'],
        above=envelope['above'], floor_z=floor, support_paths=supports)
    return dict(body_envelope_valid=True,
        body_envelope_checked_sim_time_ns=source_ns,
        body_envelope_registry_sha256=hashlib.sha256(canonical(registry)).hexdigest(),
        body_envelope_fault='')
