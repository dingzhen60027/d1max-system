"""Collision-asset-backed self returns, independent of navigation envelopes.

Pure geometry: no ROS, SDK, free-space clearing or posture guessing.  A caller
must transform both endpoints and their real ray origins into ``root_link`` at
acquisition time, and supply joint angles from a time-matched named snapshot.
Missing articulated poses leave those returns untouched.

Meshes use their *actual triangle surface*, not the filled convex hull.  The
conservative surface-only test may miss noisy self returns; it must not erase
points in a CAD concavity or an external obstacle merely hidden by the robot.
The JSON manifest seals collision origins and original mesh hashes, not visual
approximations.  Cache/prewarm is optional and never changes classification.
"""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from functools import cached_property
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import threading
import time
from typing import Mapping
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree

try:
    from ._mesh_ray_native import Mesh as _NativeMesh
except ImportError:
    _NativeMesh = None

# A startup diagnostic, also included in the existing warmup report. Missing
# optional acceleration changes cost only; the exact Python oracle remains.
MESH_QUERY_BACKEND = 'native_exact_bvh' if _NativeMesh is not None else 'python_exact_bvh_fallback'


def _readonly_query_array(value):
    # The native kernel releases the GIL. Freeze this small query batch rather
    # than retaining a read-only view whose caller-owned base is still mutable.
    # Large immutable meshes are retained by the native handle, never copied.
    result = np.array(value, dtype=np.float64, order='C', copy=True)
    result.flags.writeable = False
    return result


SCHEMA = 'd1max_collision_self_model/v1'


def _numbers(text, size, default):
    values = tuple(float(v) for v in (text or default).split())
    if len(values) != size or not all(math.isfinite(v) for v in values):
        raise ValueError('invalid finite geometry vector')
    return values


def _rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                             math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def _transform(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = _rotation(rpy)
    result[:3, 3] = xyz
    return result


def _inside_bounds(points, lower, upper):
    """Exact inclusive AABB test without allocating/reducing N x 3 masks."""
    points, lower, upper = np.asarray(points), np.asarray(lower), np.asarray(upper)
    return ((points[:, 0] >= lower[0]) & (points[:, 0] <= upper[0]) &
            (points[:, 1] >= lower[1]) & (points[:, 1] <= upper[1]) &
            (points[:, 2] >= lower[2]) & (points[:, 2] <= upper[2]))


def _axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0:
        raise ValueError('joint axis is zero')
    x, y, z = axis / norm
    k = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle)*k + (1-math.cos(angle))*(k @ k)


def _sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def _stl_triangles(path):
    """Compact read-only binary view; ASCII STL is supported for small assets."""
    path = Path(path)
    with path.open('rb') as stream:
        head = stream.read(84)
    count = struct.unpack('<I', head[80:84])[0] if len(head) == 84 else 0
    if path.stat().st_size == 84 + 50*count:
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)),
                          ('attribute', '<u2')])
        records = np.memmap(path, dtype=dtype, mode='r', offset=84, shape=count)
        triangles = records['vertices']
    else:
        # Do not interpret a corrupt large binary file as unbounded text.
        if path.stat().st_size > 16 << 20:
            raise ValueError('non-binary STL exceeds bounded ASCII loader')
        values = re.findall(r'\bvertex\s+(\S+)\s+(\S+)\s+(\S+)',
                            path.read_text(encoding='ascii'))
        if not values or len(values) % 3:
            raise ValueError('invalid STL triangles')
        triangles = np.asarray(values, dtype=np.float32).reshape(-1, 3, 3)
    if not len(triangles):
        raise ValueError('empty collision mesh')
    for start in range(0, len(triangles), 65536):
        if not np.isfinite(triangles[start:start+65536]).all():
            raise ValueError('non-finite collision mesh')
    triangles.flags.writeable = False
    return triangles


def _mesh_bounds(path, scale):
    triangles = _stl_triangles(path)
    lower, upper = np.full(3, np.inf), np.full(3, -np.inf)
    scale = np.asarray(scale)
    for start in range(0, len(triangles), 65536):
        block = triangles[start:start+65536]
        lower = np.minimum(lower, block.min(axis=(0, 1))*scale)
        upper = np.maximum(upper, block.max(axis=(0, 1))*scale)
    return tuple(lower), tuple(upper)


@dataclass(frozen=True)
class Joint:
    name: str
    parent: str
    child: str
    kind: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]


@dataclass(frozen=True)
class Collision:
    link: str
    index: int
    kind: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    lower: tuple[float, float, float]
    upper: tuple[float, float, float]
    dimensions: tuple[float, ...] = ()
    mesh_path: str = ''
    mesh_sha256: str = ''
    scale: tuple[float, float, float] = (1., 1., 1.)


@dataclass(frozen=True)
class SelfFilterConfig:
    # Measurement uncertainty, NOT safety clearance or a near-body clear box.
    measurement_padding_m: float = 0.
    max_candidate_points: int = 2048
    max_classification_ms: float = 20.

    def __post_init__(self):
        if (isinstance(self.measurement_padding_m, bool) or
                not math.isfinite(self.measurement_padding_m) or
                not 0 <= self.measurement_padding_m <= .02):
            raise ValueError('measurement padding must be finite and within 0..0.02 m')
        if (type(self.max_candidate_points) is not int or
                not 1 <= self.max_candidate_points <= 100000):
            raise ValueError('invalid bounded candidate count')
        if (isinstance(self.max_classification_ms, bool) or
                not math.isfinite(self.max_classification_ms) or
                not 0 < self.max_classification_ms <= 1000):
            raise ValueError('invalid classification deadline')


@dataclass(frozen=True)
class Classification:
    matched_mask: np.ndarray
    per_link_counts: Mapping[str, int]
    missing_links: tuple[str, ...]
    origin_inside_links: tuple[str, ...]
    timed_out: bool
    candidate_limit_reached: bool
    processed_candidate_points: int
    # Mesh AABB containment is ambiguity, not evidence of solid containment.
    origin_ambiguous_links: tuple[str, ...] = ()
    unloaded_links: tuple[str, ...] = ()
    unsupported_occluder_links: tuple[str, ...] = ()
    # Never use this result to mark rays past self returns free.
    extends_free_rays: bool = False


class _MeshSurface:
    def __init__(self, collision):
        if _sha(collision.mesh_path) != collision.mesh_sha256:
            raise ValueError('collision mesh changed before warmup')
        # One compact initialization copy freezes geometry even if somebody
        # edits the source STL later. Never copy meshes per ray or snapshot.
        self.triangles = np.array(_stl_triangles(collision.mesh_path),
                                  dtype=np.float32, order='C', copy=True)
        self.triangles.flags.writeable = False
        self.scale = np.asarray(collision.scale)
        self.scale.flags.writeable = False
        centers = np.empty((len(self.triangles), 3), dtype=np.float64)
        # Exact float64 differences of the frozen float32 source vertices.
        # Reusing these edges avoids copying and subtracting entire triangles
        # for every ray/leaf pair.  All bytes remain within the cache limit.
        self.edges = np.empty((len(self.triangles), 2, 3), dtype=np.float64)
        max_radius = 0.
        for start in range(0, len(self.triangles), 65536):
            block = self.triangles[start:start+65536].astype(np.float64)*self.scale
            center = block.mean(axis=1)
            centers[start:start+len(block)] = center
            self.edges[start:start+len(block), 0] = block[:, 1]-block[:, 0]
            self.edges[start:start+len(block), 1] = block[:, 2]-block[:, 0]
            max_radius = max(max_radius, float(np.linalg.norm(
                block-center[:, None, :], axis=2).max()))
        centers.flags.writeable = False
        self.edges.flags.writeable = False
        self.tree = cKDTree(centers, copy_data=False, leafsize=32)
        self.max_radius = max_radius
        # Use the centre tree's subdivision only to build an exact triangle
        # AABB hierarchy.  Centre spheres are far too broad for short body
        # rays: they used to fetch most of a CAD link for every single beam.
        # The immutable hierarchy is compiled once, before live scans.
        lower, upper, children, spans = [], [], [], []

        def compile_node(node):
            index = len(lower)
            lower.append(None); upper.append(None)
            children.append((-1, -1))
            spans.append((node.start_idx, node.end_idx))
            if node.split_dim < 0:
                block = self.triangles[self.tree.indices[node.start_idx:node.end_idx]]
                vertices = block.astype(float)*self.scale
                lower[index], upper[index] = vertices.min(axis=(0, 1)), vertices.max(axis=(0, 1))
            else:
                left, right = compile_node(node.lesser), compile_node(node.greater)
                children[index] = (left, right)
                lower[index] = np.minimum(lower[left], lower[right])
                upper[index] = np.maximum(upper[left], upper[right])
            return index

        compile_node(self.tree.tree)
        self.bvh_lower = np.asarray(lower)
        self.bvh_upper = np.asarray(upper)
        self.bvh_children = np.asarray(children, dtype=np.int32)
        self.bvh_spans = np.asarray(spans, dtype=np.int32)
        for value in (self.bvh_lower, self.bvh_upper, self.bvh_children, self.bvh_spans):
            value.flags.writeable = False
        self.tree.indices.flags.writeable = False
        self._native = (_NativeMesh(self.triangles, self.edges, self.scale,
            self.bvh_lower, self.bvh_upper, self.bvh_children, self.bvh_spans,
            self.tree.indices) if _NativeMesh is not None else None)
        # Center/tree estimate includes data, indices and bounded node storage.
        self.estimated_bytes = (self.triangles.nbytes + self.edges.nbytes + centers.nbytes*2 +
                                self.bvh_lower.nbytes + self.bvh_upper.nbytes +
                                self.bvh_children.nbytes + self.bvh_spans.nbytes)

    def first_intersection(self, origin, endpoint, padding, deadline=math.inf):
        first, timed_out = self.first_intersections(
            np.asarray(origin)[None, :], np.asarray(endpoint)[None, :], padding, deadline)
        return None if timed_out or not np.isfinite(first[0]) else float(first[0])

    def _leaf_pairs(self, rays, spans):
        """Bound pairing storage even for a degenerate oversized centre leaf."""
        for start in range(0, len(rays), 256):
            selected_rays, selected_spans = rays[start:start+256], spans[start:start+256]
            maximum = int((selected_spans[:, 1]-selected_spans[:, 0]).max())
            for offset in range(0, maximum, 32):
                slots = selected_spans[:, 0, None]+np.arange(offset, min(offset+32, maximum))
                used = slots < selected_spans[:, 1, None]
                yield (self.tree.indices[slots[used]],
                       np.broadcast_to(selected_rays[:, None], slots.shape)[used])

    def surface_candidates(self, points, padding, deadline=math.inf):
        if self._native is not None:
            return self._native.surface_candidates(_readonly_query_array(points), padding, deadline)
        return self._surface_candidates_python(points, padding, deadline)

    def _surface_candidates_python(self, points, padding, deadline=math.inf):
        """Necessary endpoint-near-surface test, never a self-hit decision.

        A real triangle hit within the range tolerance must have its endpoint
        within that tolerance of a triangle AABB.  Leaf AABBs are a superset
        of those boxes, so this cheap query cannot discard a supported hit.
        Concavity interiors away from any surface avoid a full ray query.
        """
        points = np.asarray(points, dtype=float)
        possible = np.zeros(len(points), dtype=bool)
        point_ids = np.flatnonzero(np.isfinite(points).all(axis=1))
        node_ids = np.zeros(len(point_ids), dtype=np.int32)
        while len(point_ids):
            if time.monotonic() > deadline:
                return np.zeros(len(points), dtype=bool), True
            next_points, next_nodes = [], []
            for start in range(0, len(point_ids), 8192):
                if time.monotonic() > deadline:
                    return np.zeros(len(points), dtype=bool), True
                ids, nodes = point_ids[start:start+8192], node_ids[start:start+8192]
                keep = (~possible[ids] &
                        (points[ids] >= self.bvh_lower[nodes]-padding-1e-9).all(axis=1) &
                        (points[ids] <= self.bvh_upper[nodes]+padding+1e-9).all(axis=1))
                ids, nodes = ids[keep], nodes[keep]
                leaf = self.bvh_children[nodes, 0] < 0
                possible[ids[leaf]] = True
                if (~leaf).any():
                    next_points.append(np.repeat(ids[~leaf], 2))
                    next_nodes.append(self.bvh_children[nodes[~leaf]].ravel())
            point_ids = np.concatenate(next_points) if next_points else np.empty(0, dtype=np.int64)
            node_ids = np.concatenate(next_nodes) if next_nodes else np.empty(0, dtype=np.int32)
        return possible, False

    def first_intersections(self, origins, endpoints, padding, deadline=math.inf):
        if self._native is not None:
            origins, endpoints = _readonly_query_array(origins), _readonly_query_array(endpoints)
            first, timed_out, marginal = self._native.first_intersections(
                origins, endpoints, padding, deadline, checked=True)
            if timed_out or not marginal:
                return first, timed_out
            # Near a floating-point decision boundary, use the original exact
            # NumPy implementation. The original deadline is not renewed; any
            # exhausted oracle batch retains all rays, including native hits.
            first, timed_out = self._first_intersections_python(origins, endpoints, padding, deadline)
            if timed_out or time.monotonic() > deadline:
                return np.full(len(endpoints), np.inf), True
            return first, False
        return self._first_intersections_python(origins, endpoints, padding, deadline)

    def _first_intersections_python(self, origins, endpoints, padding, deadline=math.inf):
        """Exact triangle first hits for a batch; no mesh/tree construction.

        AABB nodes only prune impossible triangle intersections.  Neither an
        AABB entry nor a hull/inside test becomes surface evidence.  Work is
        chunked for the caller's cooperative deadline; an incomplete batch
        never returns partial surface evidence as a completed query.
        """
        origins, endpoints = np.asarray(origins, dtype=float), np.asarray(endpoints, dtype=float)
        delta = endpoints-origins
        lengths = np.linalg.norm(delta, axis=1)
        directions = np.divide(delta, lengths[:, None], out=np.zeros_like(delta),
                               where=lengths[:, None] > 1e-12)
        first = np.full(len(endpoints), np.inf)
        ray_ids = np.flatnonzero((lengths > 1e-12) & np.isfinite(lengths))
        node_ids = np.zeros(len(ray_ids), dtype=np.int32)
        while len(ray_ids):
            if time.monotonic() > deadline:
                return np.full(len(endpoints), np.inf), True
            next_rays, next_nodes = [], []
            # Native vectorized work stays bounded even for many parallel rays
            # intersecting the same highly tessellated patch.
            for start in range(0, len(ray_ids), 8192):
                if time.monotonic() > deadline:
                    return np.full(len(endpoints), np.inf), True
                rays, nodes = ray_ids[start:start+8192], node_ids[start:start+8192]
                origin, direction = origins[rays], directions[rays]
                lo, hi = self.bvh_lower[nodes], self.bvh_upper[nodes]
                parallel = np.abs(direction) < 1e-12
                with np.errstate(divide='ignore', invalid='ignore'):
                    a, b = (lo-origin)/direction, (hi-origin)/direction
                valid = (~parallel | ((origin >= lo-1e-9) & (origin <= hi+1e-9))).all(axis=1)
                a[parallel], b[parallel] = -math.inf, math.inf
                entry = np.maximum(np.minimum(a, b).max(axis=1), 0.)
                exit = np.maximum(a, b).min(axis=1)
                valid &= ((exit >= entry-1e-9) &
                          (entry <= np.minimum(lengths[rays]+padding, first[rays])+1e-9))
                rays, nodes = rays[valid], nodes[valid]
                if not len(rays):
                    continue
                leaf = self.bvh_children[nodes, 0] < 0
                if (~leaf).any():
                    next_rays.append(np.repeat(rays[~leaf], 2))
                    next_nodes.append(self.bvh_children[nodes[~leaf]].ravel())
                if not leaf.any():
                    continue
                leaf_rays, spans = rays[leaf], self.bvh_spans[nodes[leaf]]
                for triangle_ids, paired_rays in self._leaf_pairs(leaf_rays, spans):
                    if time.monotonic() > deadline:
                        return np.full(len(endpoints), np.inf), True
                    ids, beam = triangle_ids, paired_rays
                    direction = directions[beam]
                    v0 = self.triangles[ids, 0].astype(float)*self.scale
                    e1, e2 = self.edges[ids, 0], self.edges[ids, 1]
                    h = np.cross(direction, e2)
                    determinant = np.einsum('ij,ij->i', e1, h)
                    hit = np.abs(determinant) > 1e-12
                    inverse = np.divide(1., determinant, out=np.zeros(len(ids)), where=hit)
                    s = origins[beam]-v0
                    u = inverse*np.einsum('ij,ij->i', s, h)
                    q = np.cross(s, e1)
                    v = inverse*np.einsum('ij,ij->i', direction, q)
                    distance = inverse*np.einsum('ij,ij->i', e2, q)
                    hit &= ((u >= -1e-9) & (v >= -1e-9) & (u+v <= 1+1e-9) &
                            (distance >= 0.) & (distance <= lengths[beam]+padding+1e-9))
                    if hit.any():
                        np.minimum.at(first, beam[hit], distance[hit])
            ray_ids = np.concatenate(next_rays) if next_rays else np.empty(0, dtype=np.int64)
            node_ids = np.concatenate(next_nodes) if next_nodes else np.empty(0, dtype=np.int32)
        return first, False


class _MeshCache:
    """Bounded process cache; manifest/model state is immutable and copy-free."""
    def __init__(self):
        self.entries = OrderedDict()
        self.bytes = 0
        self.max_bytes = 192 << 20
        self.lock = threading.RLock()

    def get(self, collision):
        with self.lock:
            return self._get_locked(collision)

    def _get_locked(self, collision):
        key = (collision.mesh_sha256, collision.scale)
        if key in self.entries:
            return self.entries[key]
        # Binary records use 50 bytes; centres/tree plus exact AABB hierarchy
        # and cached float64 edges are conservatively bounded by 3.2 source
        # bytes per STL byte. Reject
        # before building, never thrash an
        # existing cache to rebuild missed meshes in a live callback.
        estimate = math.ceil(Path(collision.mesh_path).stat().st_size*3.2)
        if self.bytes+estimate > self.max_bytes or len(self.entries) >= 32:
            return None
        result = _MeshSurface(collision)
        if self.bytes+result.estimated_bytes <= self.max_bytes:
            self.entries[key] = result
            self.bytes += result.estimated_bytes
            return result
        return None

    def peek(self, collision):
        """Non-loading lookup for live classification."""
        with self.lock:
            return self.entries.get((collision.mesh_sha256, collision.scale))


_MESH_CACHE = _MeshCache()


def _inside_primitive(collision, points, margin=0.):
    points = np.asarray(points)
    if collision.kind == 'box':
        return ((points >= np.asarray(collision.lower)-margin).all(axis=-1) &
                (points <= np.asarray(collision.upper)+margin).all(axis=-1))
    radius, length = collision.dimensions
    if collision.kind == 'sphere':
        return np.linalg.norm(points, axis=-1) <= radius+margin
    return ((np.linalg.norm(points[..., :2], axis=-1) <= radius+margin) &
            (np.abs(points[..., 2]) <= length/2+margin))


def _bounds_ray_entry(origin, endpoint, lower, upper, padding=0.):
    """Slab entry distance; AABB is only broad phase, never entity proof."""
    delta = endpoint-origin
    length = float(np.linalg.norm(delta))
    if length <= 1e-12: return None
    direction = delta/length
    lo, hi = 0., length+padding
    for axis in range(3):
        if abs(direction[axis]) < 1e-12:
            if not lower[axis]-1e-9 <= origin[axis] <= upper[axis]+1e-9:
                return None
        else:
            a, b = ((lower[axis]-origin[axis])/direction[axis],
                    (upper[axis]-origin[axis])/direction[axis])
            lo, hi = max(lo, min(a, b)), min(hi, max(a, b))
            if hi < lo-1e-9: return None
    return lo


def _bounds_ray_entries(origins, endpoints, lower, upper, padding=0.):
    """Vectorized slab broad phase; an entry is never surface evidence."""
    delta = endpoints-origins
    lengths = np.linalg.norm(delta, axis=1)
    direction = np.divide(delta, lengths[:, None], out=np.zeros_like(delta),
                          where=lengths[:, None] > 1e-12)
    parallel = np.abs(direction) < 1e-12
    with np.errstate(divide='ignore', invalid='ignore'):
        a, b = (np.asarray(lower)-origins)/direction, (np.asarray(upper)-origins)/direction
    valid = (~parallel | ((origins >= np.asarray(lower)-1e-9) &
                          (origins <= np.asarray(upper)+1e-9))).all(axis=1)
    a[parallel], b[parallel] = -math.inf, math.inf
    entry = np.maximum(np.minimum(a, b).max(axis=1), 0.)
    exit = np.maximum(a, b).min(axis=1)
    valid &= ((lengths > 1e-12) & np.isfinite(lengths) &
              (exit >= entry-1e-9) & (entry <= lengths+padding+1e-9))
    return np.where(valid, entry, np.inf)


def _primitive_first_surface(collision, origin, endpoint, padding=0.):
    delta = endpoint-origin
    length = float(np.linalg.norm(delta))
    if length <= 1e-12 or _inside_primitive(collision, origin): return None
    direction = delta/length
    if collision.kind == 'box':
        return _bounds_ray_entry(origin, endpoint, collision.lower, collision.upper, padding)
    radius, height = collision.dimensions
    if collision.kind == 'sphere':
        b = float(origin @ direction)
        discriminant = b*b-float(origin @ origin)+radius*radius
        if discriminant < 0: return None
        hits = (-b-math.sqrt(discriminant), -b+math.sqrt(discriminant))
    else:
        hits = []
        a = float(direction[:2] @ direction[:2])
        b = float(origin[:2] @ direction[:2])
        disc = b*b-a*(float(origin[:2] @ origin[:2])-radius*radius)
        if a > 1e-12 and disc >= 0:
            for distance in ((-b-math.sqrt(disc))/a, (-b+math.sqrt(disc))/a):
                if abs(origin[2]+distance*direction[2]) <= height/2+1e-9:
                    hits.append(distance)
        if abs(direction[2]) > 1e-12:
            for z in (-height/2, height/2):
                distance = (z-origin[2])/direction[2]
                if np.linalg.norm(origin[:2]+distance*direction[:2]) <= radius+1e-9:
                    hits.append(distance)
    valid = [distance for distance in hits if 0 <= distance <= length+padding+1e-7]
    return min(valid) if valid else None


@dataclass(frozen=True)
class RobotSelfModel:
    root_link: str
    links: tuple[str, ...]
    joints: tuple[Joint, ...]
    collisions: tuple[Collision, ...]
    urdf_path: str
    urdf_sha256: str

    @property
    def joint_types(self):
        return {joint.name: joint.kind for joint in self.joints if joint.kind != 'fixed'}

    @cached_property
    def _dependencies(self):
        result = {self.root_link: ()}
        for joint in self.joints:
            result[joint.child] = result[joint.parent] + (
                (joint.name,) if joint.kind != 'fixed' else ())
        return result

    @property
    def required_joint_names(self):
        return tuple(self.joint_types)

    @property
    def asset_paths(self):
        """Exact original files to freeze alongside the small manifest."""
        return tuple(dict.fromkeys([Path(self.urdf_path)] +
                     [Path(c.mesh_path) for c in self.collisions if c.kind == 'mesh']))

    @classmethod
    def from_urdf(cls, path, *, package_roots=None):
        path = Path(path).resolve()
        robot = ET.parse(path).getroot()
        if robot.tag != 'robot':
            raise ValueError('not a URDF robot')
        links = tuple(link.attrib['name'] for link in robot.findall('link'))
        if len(set(links)) != len(links):
            raise ValueError('duplicate URDF links')
        pending = []
        for element in robot.findall('joint'):
            kind = element.attrib['type']
            if kind not in ('fixed', 'revolute', 'continuous', 'prismatic'):
                raise ValueError('unsupported joint type: '+kind)
            if element.find('mimic') is not None:
                raise ValueError('mimic requires explicit pose mapping')
            origin, axis = element.find('origin'), element.find('axis')
            pending.append(Joint(element.attrib['name'], element.find('parent').attrib['link'],
                                 element.find('child').attrib['link'], kind,
                                 _numbers(None if origin is None else origin.get('xyz'), 3, '0 0 0'),
                                 _numbers(None if origin is None else origin.get('rpy'), 3, '0 0 0'),
                                 _numbers(None if axis is None else axis.get('xyz'), 3, '1 0 0')))
        children = [joint.child for joint in pending]
        if len({joint.name for joint in pending}) != len(pending):
            raise ValueError('duplicate joint names cannot map source poses')
        if any(joint.kind != 'fixed' and np.linalg.norm(joint.axis) == 0 for joint in pending):
            raise ValueError('active joint axis is zero')
        roots = set(links)-set(children)
        if len(roots) != 1 or len(set(children)) != len(children):
            raise ValueError('URDF must have one unambiguous rooted tree')
        root = roots.pop()
        ordered, available = [], {root}
        while pending:
            ready = [joint for joint in pending if joint.parent in available]
            if not ready:
                raise ValueError('disconnected or cyclic URDF')
            for joint in ready:
                if joint.child not in links or joint.parent not in links:
                    raise ValueError('joint references missing link')
                ordered.append(joint); available.add(joint.child); pending.remove(joint)
        package_roots = dict(package_roots or {})
        # Normal ROS package layout package/urdf/model.urdf is unambiguous.
        package_roots.setdefault(path.parent.parent.name, path.parent.parent)
        collisions = []
        for link in robot.findall('link'):
            for index, element in enumerate(link.findall('collision')):
                origin = element.find('origin')
                xyz = _numbers(None if origin is None else origin.get('xyz'), 3, '0 0 0')
                rpy = _numbers(None if origin is None else origin.get('rpy'), 3, '0 0 0')
                geometry = element.find('geometry')
                if geometry is None or len(geometry) != 1:
                    raise ValueError('one explicit collision geometry required')
                shape = list(geometry)[0]
                base = dict(link=link.attrib['name'], index=index, kind=shape.tag, xyz=xyz, rpy=rpy)
                if shape.tag == 'mesh':
                    name = shape.attrib['filename']
                    if name.startswith('package://'):
                        package, relative = name[len('package://'):].split('/', 1)
                        if package not in package_roots:
                            raise ValueError('missing mesh package root: '+package)
                        mesh = Path(package_roots[package])/relative
                    elif name.startswith('file://'):
                        mesh = Path(name[len('file://'):])
                    else:
                        mesh = path.parent/name
                    mesh = mesh.resolve()
                    scale = _numbers(shape.get('scale'), 3, '1 1 1')
                    if any(value <= 0 for value in scale):
                        raise ValueError('positive mesh scale required')
                    lower, upper = _mesh_bounds(mesh, scale)
                    collisions.append(Collision(**base, lower=lower, upper=upper, scale=scale,
                                                mesh_path=str(mesh), mesh_sha256=_sha(mesh)))
                elif shape.tag == 'box':
                    size = _numbers(shape.get('size'), 3, '')
                    if min(size) <= 0: raise ValueError('positive box required')
                    collisions.append(Collision(**base, lower=tuple(-v/2 for v in size),
                                                upper=tuple(v/2 for v in size), dimensions=size))
                elif shape.tag in ('sphere', 'cylinder'):
                    radius = float(shape.attrib['radius'])
                    length = float(shape.attrib['length']) if shape.tag == 'cylinder' else 2*radius
                    if not all(math.isfinite(v) and v > 0 for v in (radius, length)):
                        raise ValueError('positive finite primitive required')
                    collisions.append(Collision(**base, lower=(-radius, -radius, -length/2),
                                                upper=(radius, radius, length/2),
                                                dimensions=(radius, length)))
                else:
                    raise ValueError('unsupported collision geometry: '+shape.tag)
        return cls(root, links, tuple(ordered), tuple(collisions), str(path), _sha(path))

    def to_dict(self):
        from dataclasses import asdict
        value = {'schema': SCHEMA, 'root_link': self.root_link, 'links': list(self.links),
                'joints': [asdict(joint) for joint in self.joints],
                'collisions': [asdict(collision) for collision in self.collisions],
                'urdf_path': self.urdf_path, 'urdf_sha256': self.urdf_sha256,
                'mesh_policy': 'actual_triangle_first_surface_only',
                'uses_visual_geometry': False, 'grants_motion_authority': False,
                'extrinsics_calibrated': False, 'posture_calibrated': False}
        return json.loads(json.dumps(value, allow_nan=False))

    @property
    def content_hash(self):
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True,
                                        separators=(',', ':'), allow_nan=False).encode()).hexdigest()

    @classmethod
    def from_dict(cls, value):
        if (value.get('schema') != SCHEMA or value.get('uses_visual_geometry') is not False or
                value.get('grants_motion_authority') is not False or
                value.get('mesh_policy') != 'actual_triangle_first_surface_only'):
            raise ValueError('incompatible collision manifest')
        # Recompile the declared collision assets and compare every frozen field;
        # a JSON edit cannot silently turn collision meshes into clear boxes.
        actual = cls.from_urdf(value['urdf_path'])
        if actual.to_dict() != json.loads(json.dumps(value, allow_nan=False)):
            raise ValueError('collision manifest or source asset hash changed')
        return actual

    @classmethod
    def load(cls, path, *, warm=False):
        model = cls.from_dict(json.loads(Path(path).read_text()))
        if warm:
            model.warmup()
        return model

    def forward_kinematics(self, joint_positions=None):
        values = dict(joint_positions or {})
        if any(name not in self.joint_types for name in values):
            raise ValueError('joint snapshot has unmapped names')
        if any(isinstance(v, bool) or not math.isfinite(float(v)) for v in values.values()):
            raise ValueError('joint angles must be finite SI values')
        poses = {self.root_link: np.eye(4)}
        missing = []
        for joint in self.joints:
            if joint.parent not in poses or (joint.kind != 'fixed' and joint.name not in values):
                missing.append(joint.child)
                continue
            motion = np.eye(4)
            if joint.kind in ('revolute', 'continuous'):
                motion[:3, :3] = _axis_rotation(joint.axis, float(values[joint.name]))
            elif joint.kind == 'prismatic':
                motion[:3, 3] = np.asarray(joint.axis)/np.linalg.norm(joint.axis)*float(values[joint.name])
            poses[joint.child] = poses[joint.parent] @ _transform(joint.xyz, joint.rpy) @ motion
        for value in poses.values(): value.flags.writeable = False
        return poses, tuple(missing)

    def _joint_columns(self, snapshots, count):
        """Freeze exact per-ray angles; absent/nonfinite values have no pose."""
        names = tuple(self.joint_types)
        known = set(names)
        if isinstance(snapshots, Mapping):
            if set(snapshots)-known:
                raise ValueError('joint snapshot has unmapped names')
            result = {}
            for name in names:
                if name not in snapshots: continue
                raw = np.asarray(snapshots[name])
                if raw.dtype.kind == 'b':
                    raise ValueError('joint angles cannot be booleans')
                if raw.shape != (count,):
                    raise ValueError('per-joint angle arrays must be N values')
                if raw.dtype.kind == 'O' and any(isinstance(v, bool) for v in raw):
                    raise ValueError('joint angles cannot be booleans')
                values = np.array(raw, dtype=float, copy=True)
                values.flags.writeable = False
                result[name] = values
            return result
        rows = tuple(snapshots)
        if len(rows) != count or any(not isinstance(row, Mapping) for row in rows):
            raise ValueError('one named joint snapshot per ray required')
        if set().union(*(row.keys() for row in rows))-known:
            raise ValueError('joint snapshot has unmapped names')
        result = {}
        for name in names:
            def entries():
                for row in rows:
                    value = row.get(name, math.nan)
                    if isinstance(value, bool):
                        raise ValueError('joint angles cannot be booleans')
                    yield value
            values = np.fromiter(entries(), dtype=float, count=count)
            # Entirely absent source joints need no Nx3x3 allocation below.
            if np.isfinite(values).any():
                values.flags.writeable = False
                result[name] = values
        return result

    def _fk_per_ray(self, columns, count, deadline):
        """O(links*rays), not one complete scalar FK for every timestamp."""
        valid_root = np.ones(count, dtype=bool)
        poses = {self.root_link: (np.eye(3), np.zeros(3), valid_root)}
        missing = []
        for joint in self.joints:
            if time.monotonic() > deadline:
                return poses, tuple(missing), True
            if joint.parent not in poses:
                missing.append(joint.child); continue
            rotation, translation, valid_parent = poses[joint.parent]
            valid = valid_parent
            if joint.kind != 'fixed':
                values = columns.get(joint.name)
                if values is None:
                    missing.append(joint.child); continue
                valid = valid_parent & np.isfinite(values)
                if not valid.all(): missing.append(joint.child)
                if not valid.any(): continue
            before_rotation = rotation @ _rotation(joint.rpy)
            before_translation = np.einsum('...ij,j->...i', rotation, joint.xyz) + translation
            constant = (joint.kind != 'fixed' and np.isfinite(values).all() and
                        (not len(values) or (values == values[0]).all()))
            if joint.kind in ('revolute', 'continuous'):
                if constant:
                    # Exact identical source values, not rounded timestamps
                    # or poses. Wheel-mode hips often really remain constant.
                    motion = _axis_rotation(joint.axis, float(values[0]))
                    result_rotation = before_rotation @ motion
                    poses[joint.child] = (result_rotation, before_translation, valid)
                    continue
                # NaN is only an invalid placeholder, never a nominal q=0.
                angle = np.where(valid, values, np.nan)
                axis = np.asarray(joint.axis)/np.linalg.norm(joint.axis)
                x, y, z = axis
                k = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
                motion = (np.eye(3) + np.sin(angle)[:, None, None]*k +
                          (1-np.cos(angle))[:, None, None]*(k @ k))
                result_rotation = before_rotation @ motion
                result_translation = before_translation
            elif joint.kind == 'prismatic':
                amount = values[0] if constant else np.where(valid, values, np.nan)
                displacement = (amount*(np.asarray(joint.axis)/np.linalg.norm(joint.axis)) if constant
                    else amount[:, None]*(np.asarray(joint.axis)/np.linalg.norm(joint.axis)))
                result_rotation = before_rotation
                result_translation = before_translation + np.einsum(
                    '...ij,...j->...i', before_rotation, displacement)
            else:
                result_rotation, result_translation = before_rotation, before_translation
            poses[joint.child] = (result_rotation, result_translation, valid)
        return poses, tuple(missing), False

    def _first_supported_surface(self, target, first, origin_body, endpoint_body,
                                 pose_at, deadline, unloaded, unsupported):
        """No later supported body surface can masquerade as a first return.

        Missing/ambiguous entities remain explicit diagnostics, not a global
        gate on separately supported parts. This never creates free evidence.
        """
        for other in self.collisions:
            if other is target: continue
            if time.monotonic() > deadline: return False
            pose = pose_at(other.link)
            if pose is None:
                # No guessed articulation or whole-map missing-joint gate.
                unsupported.add(other.link); continue
            rotation, translation = pose
            collision_rotation = rotation @ _rotation(other.rpy)
            collision_translation = rotation @ np.asarray(other.xyz)+translation
            origin = (origin_body-collision_translation) @ collision_rotation
            endpoint = (endpoint_body-collision_translation) @ collision_rotation
            entry = _bounds_ray_entry(origin, endpoint, other.lower, other.upper)
            if entry is None or entry >= first-1e-7: continue
            if other.kind == 'mesh':
                if ((origin >= other.lower).all() and (origin <= other.upper).all()):
                    unsupported.add(other.link); continue
                mesh = _MESH_CACHE.peek(other)
                if mesh is None:
                    unloaded.add(other.link); unsupported.add(other.link); continue
                earlier = mesh.first_intersection(origin, endpoint, 0., deadline)
            else:
                if _inside_primitive(other, origin):
                    unsupported.add(other.link); continue
                earlier = _primitive_first_surface(other, origin, endpoint)
            if earlier is not None and earlier < first-1e-7:
                return False
        return True

    def classify_articulated(self, endpoints_body, origins_body,
                             joint_positions_per_ray, *, include_static=False, config=None):
        """Batch articulated self returns at EXACT supplied per-beam poses.

        Angles are either Sequence[named dict] or Mapping[joint, N-angle array].
        NaN/missing means unsupported for that chain/ray, never zero. No time
        binning, pose interpolation or latest-pose fallback occurs here.
        The default matches articulated parts only. With include_static=True,
        one full-model pass uses the same supported per-beam poses for both
        static and moving entities, including their cross-link occlusions.
        """
        from types import MappingProxyType
        config = config or SelfFilterConfig()
        deadline = time.monotonic()+config.max_classification_ms/1000
        points = np.asarray(endpoints_body, dtype=float)
        origins = np.asarray(origins_body, dtype=float)
        if points.ndim != 2 or points.shape[1:] != (3,):
            raise ValueError('endpoints must be N x 3')
        if origins.shape == (3,): origins = np.broadcast_to(origins, points.shape)
        if origins.shape != points.shape:
            raise ValueError('origins must be 3 or N x 3')
        columns = self._joint_columns(joint_positions_per_ray, len(points))
        poses, missing, timed_out = self._fk_per_ray(columns, len(points), deadline)
        matched = np.zeros(len(points), dtype=bool)
        valid_input = np.isfinite(points).all(axis=1) & np.isfinite(origins).all(axis=1)
        counts, inside, ambiguous, unloaded, unsupported = {}, set(), set(), set(), set()
        processed, limit = 0, False
        padding = config.measurement_padding_m
        for collision in self.collisions:
            if ((not include_static and not self._dependencies[collision.link]) or
                    collision.link not in poses):
                continue
            if time.monotonic() > deadline:
                timed_out = True; break
            rotation, translation, valid_pose = poses[collision.link]
            collision_rotation = rotation @ _rotation(collision.rpy)
            collision_translation = np.einsum('...ij,j->...i', rotation, collision.xyz)+translation
            if collision_rotation.ndim == 2:
                local = (points-collision_translation) @ collision_rotation
            else:
                local = np.einsum('...ji,...j->...i', collision_rotation, points-collision_translation)
            selected = np.flatnonzero(valid_input & valid_pose & ~matched &
                (local >= np.asarray(collision.lower)-padding-1e-9).all(axis=1) &
                (local <= np.asarray(collision.upper)+padding+1e-9).all(axis=1))
            for index in selected:
                if time.monotonic() > deadline:
                    timed_out = True; break
                if processed >= config.max_candidate_points:
                    limit = True; break
                processed += 1
                r = collision_rotation if collision_rotation.ndim == 2 else collision_rotation[index]
                t = collision_translation if collision_translation.ndim == 1 else collision_translation[index]
                origin = (origins[index]-t) @ r
                endpoint = local[index]
                if collision.kind == 'mesh':
                    if ((origin >= collision.lower).all() and (origin <= collision.upper).all()):
                        ambiguous.add(collision.link); continue
                    mesh = _MESH_CACHE.peek(collision)
                    if mesh is None:
                        unloaded.add(collision.link); continue
                    first = mesh.first_intersection(origin, endpoint, padding, deadline)
                    hit = (first is not None and
                           abs(first-float(np.linalg.norm(endpoint-origin))) <= padding+1e-7)
                else:
                    if _inside_primitive(collision, origin):
                        inside.add(collision.link); continue
                    first = _primitive_first_surface(collision, origin, endpoint, padding)
                    hit = bool(_inside_primitive(collision, endpoint, padding) and first is not None)
                if hit:
                    def pose_at(link):
                        if link not in poses: return None
                        link_r, link_t, link_valid = poses[link]
                        if not link_valid[index]: return None
                        return (link_r if link_r.ndim == 2 else link_r[index],
                                link_t if link_t.ndim == 1 else link_t[index])
                    hit = self._first_supported_surface(collision, first, origins[index],
                        points[index], pose_at, deadline, unloaded, unsupported)
                if time.monotonic() > deadline:
                    timed_out = True; break
                if hit:
                    matched[index] = True
                    counts[collision.link] = counts.get(collision.link, 0)+1
            if timed_out or limit: break
        if timed_out or limit:
            matched[:] = False; counts.clear()
        matched.flags.writeable = False
        return Classification(matched, MappingProxyType(counts), missing, tuple(sorted(inside)),
                              timed_out, limit, processed, tuple(sorted(ambiguous)),
                              tuple(sorted(unloaded)), tuple(sorted(unsupported)))

    def prewarm(self, *, include_dynamic=False, sensor_origins=None):
        """Compatibility name for the explicit initialization stage."""
        return self.warmup(include_dynamic=include_dynamic, sensor_origins=sensor_origins)

    def warmup(self, *, include_dynamic=False, sensor_origins=None):
        """Load at node initialization, before receiving any live scan.

        Manifest load never constructs triangle trees. The shared cache is
        bounded at 192 MiB of conservative estimated source/tree data. Dynamic
        assets are not prepared by default when no articulated source exists.
        """
        started = time.monotonic()
        origins = None if sensor_origins is None else np.asarray(sensor_origins, dtype=float)
        if origins is not None and (origins.ndim != 2 or origins.shape[1] != 3 or
                                    not len(origins) or not np.isfinite(origins).all()):
            raise ValueError('sensor_origins must be finite K x 3 in root_link')
        fixed_poses, _ = self.forward_kinematics()
        unloaded, skipped, prepared = [], [], []
        for collision in self.collisions:
            if collision.kind != 'mesh': continue
            dynamic = bool(self._dependencies[collision.link])
            if dynamic and not include_dynamic:
                unloaded.append(collision.link); continue
            if not dynamic and origins is not None:
                pose = fixed_poses[collision.link] @ _transform(collision.xyz, collision.rpy)
                local = (origins-pose[:3, 3]) @ pose[:3, :3]
                if ((local >= collision.lower) & (local <= collision.upper)).all():
                    # Every source origin has the same ambiguity as classify;
                    # loading a huge unused torso tree cannot fix that fact.
                    skipped.append(collision.link); continue
            if _MESH_CACHE.get(collision) is None:
                unloaded.append(collision.link)
            else:
                prepared.append(collision.link)
        return {'elapsed_ms': (time.monotonic()-started)*1000,
                'mesh_query_backend': MESH_QUERY_BACKEND,
                'mesh_cache_estimated_bytes': _MESH_CACHE.bytes,
                'mesh_cache_limit_bytes': _MESH_CACHE.max_bytes,
                'dynamic_assets_requested': bool(include_dynamic),
                'prepared_links': tuple(prepared), 'unloaded_links': tuple(unloaded),
                'origin_ambiguous_skipped_links': tuple(skipped)}

    def _prepare_collision_frames(self, poses):
        """Immutable exact collision frames; no mesh loads or policy/padding.

        Only a frozen engineering stance reuses these across scans. Measured
        articulated callers prepare them from their own acquisition-time FK.
        """
        from types import MappingProxyType
        records, unposed = [], set()
        for collision in self.collisions:
            if collision.link not in poses:
                unposed.add(collision.link)
                continue
            pose = poses[collision.link] @ _transform(collision.xyz, collision.rpy)
            rotation, translation = pose[:3, :3], pose[:3, 3]
            center = rotation @ ((np.asarray(collision.lower)+collision.upper)/2)+translation
            extent = np.abs(rotation) @ ((np.asarray(collision.upper)-collision.lower)/2)
            values = dict(collision=collision, rotation=rotation, translation=translation,
                          world_lower=center-extent, world_upper=center+extent,
                          padding_axes=np.abs(rotation).sum(axis=1))
            for value in values.values():
                if isinstance(value, np.ndarray):
                    value.setflags(write=False)
            records.append(MappingProxyType(values))
        return tuple(records), frozenset(unposed)

    def _classify_at_poses(self, endpoints_body, origins_body, poses, *, missing=(),
                           include_static=True, include_dynamic=True, config=None,
                           prepared_frames=None):
        """Batch exact surfaces and occluders at already-frozen link poses.

        A fixed engineering adapter supplies its computed FK.  This does not
        assign a nominal pose to missing source articulation.  Per-beam FK
        retains its separate acquisition-time joint coverage policy.
        """
        from types import MappingProxyType
        config = config or SelfFilterConfig()
        deadline = time.monotonic()+config.max_classification_ms/1000
        points = np.asarray(endpoints_body, dtype=float)
        origins = np.asarray(origins_body, dtype=float)
        one_origin = origins.shape == (3,)
        if points.ndim != 2 or points.shape[1:] != (3,):
            raise ValueError('endpoints must be N x 3')
        if origins.shape == (3,): origins = np.broadcast_to(origins, points.shape)
        if origins.shape != points.shape:
            raise ValueError('origins must be 3 or N x 3')
        matched = np.zeros(len(points), dtype=bool)
        valid_input = np.isfinite(points).all(axis=1) & np.isfinite(origins).all(axis=1)
        counts, origin_inside, origin_ambiguous, unloaded, unsupported = {}, set(), set(), set(), set()
        processed, timed_out, limit = 0, False, False
        pad = config.measurement_padding_m
        records = []
        frames, unposed = (self._prepare_collision_frames(poses)
                           if prepared_frames is None else prepared_frames)
        for frame in frames:
            collision = frame['collision']
            rotation, translation = frame['rotation'], frame['translation']
            local_origins = ((origins[0]-translation) @ rotation if one_origin and len(points)
                             else (origins-translation) @ rotation)
            if local_origins.shape == (3,):
                # A fixed source origin has one inside/ambiguity result, not
                # N identical box/primitive queries. This does not substitute
                # the CAD/configured origin for the actual input ray origin.
                inside = (_inside_bounds(local_origins[None, :], collision.lower, collision.upper)
                          if collision.kind == 'mesh' else _inside_primitive(collision, local_origins[None, :]))
                inside = np.broadcast_to(inside, (len(points),))
                local_origins = np.broadcast_to(local_origins, points.shape)
            else:
                inside = (_inside_bounds(local_origins, collision.lower, collision.upper)
                          if collision.kind == 'mesh' else _inside_primitive(collision, local_origins))
            records.append(dict(frame, world_pad=pad*frame['padding_axes'], origins=local_origins,
                                inside=inside, first=np.full(len(points), np.nan),
                                mesh=_MESH_CACHE.peek(collision) if collision.kind == 'mesh' else None))

        def query(record, indices):
            """Overlapping target/occlusion queries reuse real triangle hits."""
            nonlocal timed_out
            first = record['first']
            unknown = indices[np.isnan(first[indices])]
            if not len(unknown):
                return first[indices]
            if time.monotonic() > deadline:
                timed_out = True
                return np.full(len(indices), np.inf)
            collision = record['collision']
            first[unknown] = np.inf
            usable = unknown[~record['inside'][unknown]]
            if not len(usable):
                return first[indices]
            local = (points[usable]-record['translation']) @ record['rotation']
            if collision.kind == 'mesh':
                if record['mesh'] is None:
                    unloaded.add(collision.link)
                    return first[indices]
                distances, incomplete = record['mesh'].first_intersections(
                    record['origins'][usable], local, pad, deadline)
                timed_out |= incomplete
                first[usable] = distances
            else:
                for slot, index in enumerate(usable):
                    if time.monotonic() > deadline:
                        timed_out = True; break
                    distance = _primitive_first_surface(collision, record['origins'][index],
                        local[slot], pad)
                    if distance is not None:
                        first[index] = distance
            return first[indices]

        for record in records:
            collision = record['collision']
            dynamic = bool(self._dependencies[collision.link])
            if (dynamic and not include_dynamic) or (not dynamic and not include_static):
                continue
            if time.monotonic() > deadline:
                timed_out = True; break
            broad = np.flatnonzero(valid_input & ~matched &
                _inside_bounds(points, record['world_lower']-record['world_pad']-1e-9,
                               record['world_upper']+record['world_pad']+1e-9))
            if not len(broad):
                continue
            local = (points[broad]-record['translation']) @ record['rotation']
            exact = _inside_bounds(local, np.asarray(collision.lower)-pad-1e-9,
                                   np.asarray(collision.upper)+pad+1e-9)
            candidates, local = broad[exact], local[exact]
            if not len(candidates):
                continue
            if record['inside'][candidates].any():
                (origin_ambiguous if collision.kind == 'mesh' else origin_inside).add(collision.link)
            if collision.kind == 'mesh' and record['mesh'] is not None:
                near, incomplete = record['mesh'].surface_candidates(local, pad+1e-7, deadline)
                timed_out |= incomplete
                if timed_out:
                    break
                # Inside-origin diagnostics remain; only externally originated
                # endpoint queries need this necessary near-surface test.
                near |= record['inside'][candidates]
                candidates, local = candidates[near], local[near]
                if not len(candidates):
                    continue
            available = config.max_candidate_points-processed
            if len(candidates) > available:
                processed += available
                limit = True; break
            processed += len(candidates)
            first = query(record, candidates)
            if timed_out:
                break
            lengths = np.linalg.norm(points[candidates]-origins[candidates], axis=1)
            if collision.kind == 'mesh':
                hit = np.isfinite(first) & (np.abs(first-lengths) <= pad+1e-7)
            else:
                hit = np.isfinite(first) & _inside_primitive(collision, local, pad)
            selected, target_first = candidates[hit], first[hit]
            if not len(selected):
                continue
            unsupported.update(unposed)
            alive = np.ones(len(selected), dtype=bool)
            for other in records:
                if other is record or not alive.any():
                    continue
                if time.monotonic() > deadline:
                    timed_out = True; break
                indices, positions = selected[alive], np.flatnonzero(alive)
                entity = other['collision']
                world_entry = _bounds_ray_entries(origins[indices], points[indices],
                                                 other['world_lower'], other['world_upper'])
                possible = world_entry < target_first[alive]-1e-7
                indices, positions = indices[possible], positions[possible]
                if not len(indices):
                    continue
                local = (points[indices]-other['translation']) @ other['rotation']
                entry = _bounds_ray_entries(other['origins'][indices], local,
                                           entity.lower, entity.upper)
                earlier = entry < target_first[positions]-1e-7
                if not earlier.any():
                    continue
                indices, positions = indices[earlier], positions[earlier]
                ambiguous = other['inside'][indices]
                if ambiguous.any():
                    unsupported.add(entity.link)
                indices, positions = indices[~ambiguous], positions[~ambiguous]
                if not len(indices):
                    continue
                if entity.kind == 'mesh' and other['mesh'] is None:
                    unloaded.add(entity.link); unsupported.add(entity.link)
                    continue
                distances = query(other, indices)
                if timed_out:
                    break
                # Occluders use the real beam segment.  A padded intersection
                # beyond the endpoint cannot become an earlier source return.
                lengths = np.linalg.norm(points[indices]-origins[indices], axis=1)
                segment_tolerance = 1e-9 if entity.kind == 'mesh' else 1e-7
                blocked = ((distances <= lengths+segment_tolerance) &
                           (distances < target_first[positions]-1e-7))
                alive[positions[blocked]] = False
            if timed_out:
                break
            confirmed = selected[alive]
            matched[confirmed] = True
            if len(confirmed):
                counts[collision.link] = counts.get(collision.link, 0)+len(confirmed)
        if time.monotonic() > deadline:
            timed_out = True
        if timed_out or limit:
            matched[:] = False; counts.clear()
        matched.flags.writeable = False
        return Classification(matched, MappingProxyType(counts), tuple(missing),
                              tuple(sorted(origin_inside)), timed_out, limit, processed,
                              tuple(sorted(origin_ambiguous)), tuple(sorted(unloaded)),
                              tuple(sorted(unsupported)))

    def classify(self, endpoints_body, origins_body, joint_positions=None, *,
                 include_static=True, include_dynamic=True, config=None):
        """Return a self-hit mask only; invalid/unknown/over-budget rays stay intact.

        FK direction: p_root = T_root_link @ T_link_collision @ p_collision.
        Caller handles per-beam source time, epoch and calibrated lidar/body TF.
        This function has no implicit clock conversion or latest-pose fallback.
        """
        from types import MappingProxyType
        config = config or SelfFilterConfig()
        points = np.asarray(endpoints_body, dtype=float)
        origins = np.asarray(origins_body, dtype=float)
        if points.ndim != 2 or points.shape[1:] != (3,):
            raise ValueError('endpoints must be N x 3')
        if origins.shape == (3,): origins = np.broadcast_to(origins, points.shape)
        if origins.shape != points.shape:
            raise ValueError('origins must be 3 or N x 3')
        poses, missing = self.forward_kinematics(joint_positions)
        matched = np.zeros(len(points), dtype=bool)
        valid_input = np.isfinite(points).all(axis=1) & np.isfinite(origins).all(axis=1)
        counts, origin_inside, origin_ambiguous, unloaded, unsupported = {}, set(), set(), set(), set()
        deadline = time.monotonic() + config.max_classification_ms/1000
        processed, timed_out, limit = 0, False, False
        pad = config.measurement_padding_m
        for collision in self.collisions:
            dynamic = bool(self._dependencies[collision.link])
            if ((dynamic and not include_dynamic) or (not dynamic and not include_static) or
                    collision.link not in poses):
                continue
            if time.monotonic() > deadline:
                timed_out = True; break
            pose = poses[collision.link] @ _transform(collision.xyz, collision.rpy)
            rotation, translation = pose[:3, :3], pose[:3, 3]
            local = (points-translation) @ rotation
            candidates = np.flatnonzero(valid_input & ~matched &
                (local >= np.asarray(collision.lower)-pad-1e-9).all(axis=1) &
                (local <= np.asarray(collision.upper)+pad+1e-9).all(axis=1))
            for index in candidates:
                if time.monotonic() > deadline:
                    timed_out = True; break
                if processed >= config.max_candidate_points:
                    limit = True; break
                processed += 1
                point = local[index]
                origin = (origins[index]-translation) @ rotation
                # Origin inside mesh AABB is ambiguous (e.g. sensor housing).
                # Skip, never let a guessed optical origin erase outside hits.
                if collision.kind == 'mesh':
                    if ((origin >= collision.lower).all() and (origin <= collision.upper).all()):
                        origin_ambiguous.add(collision.link); continue
                    mesh = _MESH_CACHE.peek(collision)
                    if mesh is None:
                        unloaded.add(collision.link); continue
                    first = mesh.first_intersection(origin, point, pad, deadline)
                    if time.monotonic() > deadline:
                        timed_out = True; break
                    # Binary CAD vertices are float32; 0.1 micrometre is only
                    # numeric representation tolerance, not physical padding.
                    hit = first is not None and abs(first-float(np.linalg.norm(point-origin))) <= pad+1e-7
                else:
                    def inside(value, margin):
                        if collision.kind == 'box':
                            return bool((value >= np.asarray(collision.lower)-margin).all() and
                                        (value <= np.asarray(collision.upper)+margin).all())
                        radius, length = collision.dimensions
                        if collision.kind == 'sphere':
                            return np.linalg.norm(value) <= radius+margin
                        return (np.linalg.norm(value[:2]) <= radius+margin and
                                abs(value[2]) <= length/2+margin)
                    if inside(origin, 0.):
                        origin_inside.add(collision.link); continue
                    first = _primitive_first_surface(collision, origin, point, pad)
                    hit = inside(point, pad) and first is not None
                if hit:
                    def pose_at(link):
                        if link not in poses: return None
                        return poses[link][:3, :3], poses[link][:3, 3]
                    hit = self._first_supported_surface(collision, first, origins[index],
                        points[index], pose_at, deadline, unloaded, unsupported)
                if time.monotonic() > deadline:
                    timed_out = True; break
                if hit:
                    matched[index] = True
                    counts[collision.link] = counts.get(collision.link, 0)+1
            if timed_out or limit: break
        if timed_out or limit:
            # All-or-nothing bounded frame/group outcome. Caller retains the
            # original ray evidence, never treats deadline as no observation.
            matched[:] = False
            counts.clear()
        matched.flags.writeable = False
        return Classification(matched, MappingProxyType(counts), missing,
                              tuple(sorted(origin_inside)), timed_out, limit, processed,
                              tuple(sorted(origin_ambiguous)), tuple(sorted(unloaded)),
                              tuple(sorted(unsupported)))


# Short public alias for adapters which do not expose implementation terminology.
Model = RobotSelfModel
