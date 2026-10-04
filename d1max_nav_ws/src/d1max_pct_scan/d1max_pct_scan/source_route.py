"""Immutable, source-map ground routes; never a TF or a motion permit.

PCT's conditioned space is a proposal space.  A route is converted once through
the audited non-rigid ground bridge, with layer/segment ownership retained.  The
source-return audit below measures support; it deliberately does not manufacture
a certified geometric error bound, foot-placement proof, or collision envelope.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

SCHEMA = 2
MAX_POINTS = 20000
MAX_AUDIT_SAMPLES = 50000
MAX_JSON_BYTES = 8_000_000
SOURCE_FRAME = 'd1max_loc_map'
PLANNING_FRAME = 'd1max_multifloor_planning'


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def digest_file(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def _hash(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def _digest(value):
    return (isinstance(value, str) and len(value) == 64
            and all(c in '0123456789abcdef' for c in value))


def _points(value, count=None):
    p = np.asarray(value, dtype=float)
    if (p.ndim != 2 or p.shape[1] != 3 or not 2 <= len(p) <= MAX_POINTS
            or count is not None and len(p) != count
            or not np.isfinite(p).all() or np.max(np.abs(p)) > 100000):
        raise ValueError('route_points_invalid')
    steps = np.linalg.norm(np.diff(p, axis=0), axis=1)
    if steps.max(initial=0) > 1. or not .05 <= steps.sum() <= 2000.:
        raise ValueError('route_discontinuity_or_length_invalid')
    return p


def _layers(value, count):
    if (not isinstance(value, list) or len(value) != count
            or any(type(v) is not int or not 0 <= v < 2**31 for v in value)):
        raise ValueError('route_layer_ownership_invalid')


def _native_layers(value, count):
    values = np.asarray(value)
    if (values.shape != (count,) or not np.issubdtype(values.dtype, np.integer)
            or np.any(values < 0) or np.any(values >= 2**31)):
        raise ValueError('route_layer_ownership_invalid')
    return values.tolist()


@dataclass(frozen=True)
class RouteSnapshot:
    """Canonical JSON is the immutable owned value, not a mutable dict/array."""
    encoded: str
    route_hash: str

    @classmethod
    def create(cls, value):
        encoded = canonical(value)
        if len(encoded.encode()) > MAX_JSON_BYTES:
            raise ValueError('route_snapshot_size_exceeded')
        validate_snapshot(value)
        return cls(encoded, hashlib.sha256(encoded.encode()).hexdigest())

    def payload(self):
        return json.loads(self.encoded)

    def with_goal(self, *, has_goal_yaw=False, goal_yaw=0., goal_yaw_tolerance_rad=.15):
        value = self.payload()
        value.update(has_goal_yaw=has_goal_yaw, goal_yaw=goal_yaw if has_goal_yaw else 0.,
                     goal_yaw_tolerance_rad=goal_yaw_tolerance_rad)
        return self.create(value)


def validate_snapshot(v):
    required = {'schema_version', 'map_version_id', 'source_map_sha256', 'tomogram_sha256',
        'conditioning_sha256', 'frame_id', 'point_reference', 'direction', 'xyz',
        'source_layer_ids', 'layer_ids', 'point_floor_ids', 'edge_segments', 'segments',
        'geometry_evidence', 'preview_ready', 'execution_eligible', 'eligibility_reason',
        'has_goal_yaw', 'goal_yaw', 'goal_yaw_tolerance_rad'}
    if (not isinstance(v, dict) or set(v) != required
            or type(v.get('schema_version')) is not int or v.get('schema_version') != SCHEMA):
        raise ValueError('route_snapshot_schema_2_required')
    if (v['frame_id'] != SOURCE_FRAME or v['point_reference'] != 'ground'
            or not isinstance(v['map_version_id'], str) or not v['map_version_id']
            or len(v['map_version_id']) > 128
            or any(not _digest(v[k]) for k in
                   ('source_map_sha256', 'tomogram_sha256', 'conditioning_sha256'))):
        raise ValueError('route_snapshot_provenance_invalid')
    points = _points(v['xyz'])
    for key in ('source_layer_ids', 'layer_ids'):
        _layers(v[key], len(points))
    if (len(v['point_floor_ids']) != len(points)
            or any(x not in ('floor1', 'floor2', 'stair_lower', 'stair_upper', 'stairs')
                   for x in v['point_floor_ids'])
            or len(v['edge_segments']) != len(points)-1
            or not 1 <= len(v['segments']) <= 64
            or v['direction'] not in ('same_floor', 'lower_to_upper', 'upper_to_lower')):
        raise ValueError('route_snapshot_segment_ownership_invalid')
    cursor = 0
    segment_ids = set()
    for segment in v['segments']:
        keys = {'segment_id', 'kind', 'floor_id', 'begin_index', 'end_index',
            'source_layer_id', 'target_layer_id', 'entry_portal_id', 'exit_portal_id',
            'required_mode', 'direction', 'execution_eligible', 'eligibility_reason'}
        if not isinstance(segment, dict) or set(segment) != keys:
            raise ValueError('route_snapshot_segment_schema_invalid')
        a, b, sid = segment['begin_index'], segment['end_index'], segment['segment_id']
        if (type(a) is not int or type(b) is not int or a != cursor
                or not a < b < len(points) or sid in segment_ids
                or not isinstance(sid, str) or not 0 < len(sid) <= 96
                or v['edge_segments'][a:b] != [sid]*(b-a)
                or segment['source_layer_id'] != v['source_layer_ids'][a]
                or segment['target_layer_id'] != v['source_layer_ids'][b]
                or segment['kind'] not in ('floor', 'stair_up', 'stair_down', 'landing', 'unknown')
                or segment['required_mode'] not in ('general', 'stair', 'unverified')
                or segment['direction'] != v['direction']):
            raise ValueError('route_snapshot_segment_coverage_invalid')
        if (v['point_floor_ids'][a:b] != [segment['floor_id']]*(b-a)
                or (b == len(points)-1 and v['point_floor_ids'][b] != segment['floor_id'])
                or segment['kind'] == 'floor' and (
                    segment['floor_id'] not in ('floor1', 'floor2') or segment['required_mode'] != 'general')
                or segment['kind'].startswith('stair_') and (
                    segment['floor_id'] not in ('stair_lower', 'stair_upper', 'stairs')
                    or segment['required_mode'] != 'stair')):
            raise ValueError('route_snapshot_floor_segment_disagrees')
        if type(segment['execution_eligible']) is not bool or not segment['eligibility_reason']:
            raise ValueError('uncertified_segment_cannot_authorize_execution')
        segment_ids.add(sid)
        cursor = b
    if cursor != len(points)-1:
        raise ValueError('route_snapshot_uncovered_edges')
    evidence = v['geometry_evidence']
    if (not isinstance(evidence, dict) or evidence.get('schema') != 1
            or evidence.get('source_map_sha256') != v['source_map_sha256']
            or evidence.get('projection_kind') not in ('non_rigid_ground_only_not_tf','source_identity')
            or evidence.get('certified_geometric_error_bound_m') is not None
            or v['preview_ready'] is not True or type(v['execution_eligible']) is not bool
            or not isinstance(v['eligibility_reason'], str) or not v['eligibility_reason']):
        raise ValueError('route_snapshot_geometry_authority_invalid')
    eligible = v['execution_eligible']
    if any(s['execution_eligible'] != eligible for s in v['segments']):
        raise ValueError('route_snapshot_geometry_authority_segment_disagrees')
    if eligible and (evidence.get('projection_kind') != 'source_identity'
            or evidence.get('coordinate_transfer_error_bound_m') != 0.
            or evidence.get('static_route_checked') is not True
            or evidence.get('measured_source_support',{}).get('all_samples_supported') is not True
            or evidence.get('measured_ceiling_checked') is not True
            or evidence.get('physical_execution_authorized') is not False
            or v['direction'] != 'same_floor' or len(set(v['point_floor_ids'])) != 1):
        raise ValueError('uncertified_segment_cannot_authorize_execution')
    if (type(v['has_goal_yaw']) is not bool
            or type(v['goal_yaw']) not in (int, float) or not math.isfinite(v['goal_yaw'])
            or type(v['goal_yaw_tolerance_rad']) not in (int, float)
            or not .01 <= v['goal_yaw_tolerance_rad'] <= .5):
        raise ValueError('route_snapshot_goal_yaw_invalid')


class SourceSupport:
    """Bounded source-return proximity audit, not traversability certification."""
    def __init__(self, points_by_label, source_map_sha256, *, radius_m=.12):
        if not _digest(source_map_sha256) or not .01 <= radius_m <= .3:
            raise ValueError('source_support_provenance_invalid')
        self.source_hash, self.radius = source_map_sha256, float(radius_m)
        self.trees = {}
        for label, points in points_by_label.items():
            p = np.asarray(points, dtype=float).reshape(-1, 3)
            if not np.isfinite(p).all():
                raise ValueError('nonfinite_source_support')
            if len(p):
                self.trees[label] = cKDTree(p.copy())

    def audit(self, points, labels, *, spacing=.05):
        # Audit interiors as well as selected PCT vertices. A supported endpoint
        # on each side of a hole is not evidence for the missing ground inside.
        samples, owners = [], []
        for i, distance in enumerate(np.linalg.norm(np.diff(points, axis=0), axis=1)):
            n = max(1, int(math.ceil(distance/spacing)))
            if len(samples)+n+1 > MAX_AUDIT_SAMPLES:
                raise ValueError('source_support_audit_budget_exceeded')
            for j in range(n):
                samples.append(points[i]+(points[i+1]-points[i])*(j/n))
                owners.append(labels[i])
        samples.append(points[-1]); owners.append(labels[-1])
        values = np.asarray(samples)
        distances = np.full(len(values), np.inf)
        for label in set(owners):
            ids = np.flatnonzero(np.asarray(owners) == label)
            tree = self.trees.get(label)
            if tree is not None:
                distances[ids] = tree.query(values[ids], workers=1)[0]
        missing = distances > self.radius
        finite = distances[np.isfinite(distances)]
        unsupported_ids = np.flatnonzero(missing)
        return dict(available=True, source_map_sha256=self.source_hash,
            sample_spacing_m=spacing, support_radius_m=self.radius,
            samples=len(samples), unsupported_samples=int(missing.sum()),
            maximum_nearest_return_distance_m=float(finite.max()) if len(finite) else None,
            all_samples_supported=not bool(missing.any()),
            unsupported_by_floor={label:int(np.count_nonzero(missing & (np.asarray(owners)==label)))
                                  for label in sorted(set(owners))},
            first_unsupported=[dict(xyz=values[i].tolist(),floor_id=owners[i],
                distance_m=float(distances[i]) if np.isfinite(distances[i]) else None)
                for i in unsupported_ids[:16]],
            support_semantics='measured_returns_not_free_space_or_foot_placement',
            whole_swept_volume_verified=False)


class SourceRouteBuilder:
    def __init__(self, *, bridge, source_map_sha256, conditioning_sha256,
                 tomogram_sha256, support=None, tomogram=None):
        if any(not _digest(v) for v in (source_map_sha256, conditioning_sha256, tomogram_sha256)):
            raise ValueError('source_route_provenance_required')
        identity = getattr(bridge,'projection_kind',None) == 'source_identity'
        if bridge.source_frame != SOURCE_FRAME or bridge.planning_frame != (SOURCE_FRAME if identity else PLANNING_FRAME):
            raise ValueError('source_route_frame_contract_invalid')
        if support is not None and support.source_hash != source_map_sha256:
            raise ValueError('source_support_hash_mismatch')
        self.bridge, self.support, self.tomogram = bridge, support, tomogram
        self.provenance = dict(source_map_sha256=source_map_sha256,
            conditioning_sha256=conditioning_sha256, tomogram_sha256=tomogram_sha256)

    @classmethod
    def from_artifacts(cls, manifest_path, tomogram, bridge):
        if getattr(bridge,'projection_kind',None) == 'source_identity':
            from .source_identity import identity_builder
            return identity_builder(manifest_path,tomogram,bridge)
        final_path = Path(manifest_path).resolve(strict=True)
        final = json.loads(final_path.read_text())
        source = Path(final['source_path']).resolve(strict=True)
        if digest_file(source) != final['source_sha256']:
            raise ValueError('source_route_original_pcd_changed')
        floor_path = Path(final['floor_processing_manifest']).resolve(strict=True)
        if digest_file(floor_path) != final['floor_processing_manifest_sha256']:
            raise ValueError('source_route_floor_manifest_changed')
        audit_path = floor_path.parent/'audit/floor_evidence.npz'
        if digest_file(audit_path) != final['configuration']['floor_evidence_sha256']:
            raise ValueError('source_route_floor_evidence_changed')
        from .pointcloud_helpers.pcd_io import read_pcd
        xyz = read_pcd(source).xyz
        by_label = {}
        with np.load(audit_path, allow_pickle=False) as audit:
            for floor in bridge.floors:
                ids, keep = audit[floor+'_source_indices'], audit[floor+'_support']
                if (ids.ndim != 1 or keep.shape != ids.shape or keep.dtype != np.bool_
                        or not np.issubdtype(ids.dtype, np.integer)
                        or np.any(ids < 0) or np.any(ids >= len(xyz))):
                    raise ValueError('source_route_support_indices_invalid')
                by_label[floor] = xyz[ids[keep]]
        protected = np.zeros(len(xyz), dtype=bool)
        for region in bridge.protected_regions:
            protected |= np.all((xyz >= region['min']) & (xyz <= region['max']), axis=1)
        # Stair points retain original coordinates. Their proximity evidence
        # has NOT classified support planes and therefore cannot certify gait.
        stair = xyz[protected]
        by_label['stairs'] = stair
        for label, floor in (('stair_lower', 'floor1'), ('stair_upper', 'floor2')):
            by_label[label] = np.concatenate((stair, by_label[floor]))
        return cls(bridge=bridge, source_map_sha256=final['source_sha256'],
            conditioning_sha256=digest_file(final_path), tomogram_sha256=tomogram.sha256,
            support=SourceSupport(by_label, final['source_sha256']))

    def build(self, result, points, labels, *, map_version_id):
        points = _points(points)
        converted = self.bridge.to_localization_ground(points, labels)
        xyz = _points(converted.xyz, len(points))
        layers = _native_layers(result['layer_ids'], len(points))
        source_layers = result.get('source_layer_ids')
        if source_layers is None:
            raise ValueError('source_route_missing_original_layer_ids')
        source_layers = _native_layers(source_layers, len(points))
        direction = result.get('direction')
        if result.get('route_type') == 'same_floor':
            raw_segments = [dict(name='floor', first_index=0, last_index=len(points)-1,
                                 **{'from': 'start', 'to': 'goal'})]
            direction = 'same_floor'
        else:
            if direction not in ('lower_to_upper', 'upper_to_lower'):
                raise ValueError('source_route_explicit_crossfloor_direction_required')
            raw_segments = result.get('segments', [])
        support = (self.support.audit(xyz, labels) if self.support is not None else
                   dict(available=False, all_samples_supported=False,
                        reason='original_measured_support_unavailable'))
        reason = ('source_support_missing' if not support['all_samples_supported'] else
                  'source_geometry_error_bound_and_swept_volume_unverified')
        identity = getattr(self.bridge,'projection_kind',None) == 'source_identity'
        ceiling_checked = False
        if identity and self.tomogram is not None:
            checks=self.tomogram.validate_path(points,layers)
            # Use the production exhaustive supercover, not only vertices.
            ceiling_checked = (checks['unobserved_ceiling_cells']==0
                and checks['minimum_measured_headroom_m'] is not None
                and not np.any(self.tomogram.open_sky_verified))
        eligible = bool(identity and support['all_samples_supported'] and ceiling_checked)
        if identity:
            reason = ('source_identity_static_geometry_checked' if eligible else
                      'source_support_missing' if not support['all_samples_supported'] else 'unobserved_overhead')
        edge_ids, segments = ['']*(len(points)-1), []
        for segment in raw_segments:
            a, b, name = segment['first_index'], segment['last_index'], segment['name']
            if (type(a) is not int or type(b) is not int or not 0 <= a < b < len(points)
                    or not isinstance(name, str)):
                raise ValueError('source_route_segment_range_invalid')
            is_stair = name.startswith('stair_')
            kind = ('stair_down' if direction == 'upper_to_lower' else 'stair_up') if is_stair else 'floor'
            sid = str(name)
            segments.append(dict(segment_id=sid, kind=kind,
                floor_id=labels[a] if not is_stair else name,
                begin_index=a, end_index=b, source_layer_id=source_layers[a],
                target_layer_id=source_layers[b], entry_portal_id=segment['from'],
                exit_portal_id=segment['to'], required_mode='stair' if is_stair else 'general',
                direction=direction, execution_eligible=eligible, eligibility_reason=reason))
            edge_ids[a:b] = [sid]*(b-a)
        evidence = dict(schema=1, source_map_sha256=self.provenance['source_map_sha256'],
            projection_kind='source_identity' if identity else 'non_rigid_ground_only_not_tf',
            certified_geometric_error_bound_m=None, projection=converted.diagnostics,
            measured_source_support=support, planning_xyz=points.tolist(),
            layer_transitions=result.get('layer_transitions', []),
            native_segments=raw_segments, portals=result.get('anchors', {}),
            ground_to_body_height_applications=0,
            source_swept_volume_verified=False,
            blockers=[reason, 'physical_body_height_and_collision_envelope_unverified'])
        if identity:
            evidence.update(coordinate_transfer_error_bound_m=0.,static_route_checked=self.tomogram is not None,
                measured_ceiling_checked=ceiling_checked,physical_execution_authorized=False,
                eligibility_scope='static_ground_route_only_requires_live_collision_and_execution_permit')
            if self.support and len(set(labels))==1 and labels[0] in self.support.trees:
                tree=self.support.trees[labels[0]]
                ids=tree.query(xyz,workers=1)[1]
                evidence['observed_source_support_xyz']=tree.data[np.unique(ids)].tolist()
        return RouteSnapshot.create(dict(schema_version=SCHEMA, map_version_id=map_version_id,
            **self.provenance, frame_id=SOURCE_FRAME, point_reference='ground',
            direction=direction, xyz=xyz.tolist(), layer_ids=layers,
            source_layer_ids=source_layers, point_floor_ids=list(labels),
            edge_segments=edge_ids, segments=segments, geometry_evidence=evidence,
            preview_ready=True, execution_eligible=eligible, eligibility_reason=reason,
            has_goal_yaw=False, goal_yaw=0., goal_yaw_tolerance_rad=.15))
