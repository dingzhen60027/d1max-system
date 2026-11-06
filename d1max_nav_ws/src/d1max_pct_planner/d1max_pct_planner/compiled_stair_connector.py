"""Current-map verified, offline-compiled native PCT stair polylines.

The bundle is a build artifact of the existing planner, not a new search or
motion permission. Old polynomial certificates are never reused: every saved
polyline chord is checked against the current original and stair-masked map.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

SCHEMA = 'd1max.compiled_stair_connector/v1'
PROOF = 'compiled_polyline_current_tomogram_supercover_and_layer_overlap'
MAX_POINTS = 12_000
MAX_BYTES = 8 * 1024 * 1024
REFINEMENT_SCHEMA = 'd1max.measured_stair_centering/v1'
MODEL_DERIVATION_SCHEMA = 'd1max.compiled_stair_parametric_geometry/v1'
IDENTITY_KEYS = ('planning', 'limits', 'vendor_root', 'floor_z_ranges',
                 'stair_roi', 'unknown_ceiling_policy')


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False,
        default=lambda item: item.tolist() if isinstance(item, np.ndarray) else item.item())


def _digest(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def map_settings_sha256(settings):
    """Stable normalized map-query policy identity, excluding bundle/file paths."""
    return hashlib.sha256(_canonical({key: settings[key] for key in IDENTITY_KEYS}).encode()).hexdigest()


def polyline_sha256(points, layers):
    """Geometry identity independent of package relocation and JSON whitespace."""
    return hashlib.sha256(_canonical(dict(path=np.asarray(points, dtype=float),
        layer_ids=np.asarray(layers, dtype=int))).encode()).hexdigest()


def derive_model_ground(points, model):
    """Explicit ground-Z derivative; XY and points outside model stay literal."""
    original=np.asarray(points,dtype=float)
    if original.ndim!=2 or original.shape[1]!=3 or not np.isfinite(original).all():
        raise ValueError('compiled_stair_model_input_invalid')
    result=original.copy()
    for i,point in enumerate(original):
        surface=model.surface_at(point[:2],height_hint=point[2])
        if surface is not None: result[i,2]=model.ground_at(surface,point[:2])
    return result


def _model_derivation(payload, tomogram, directory):
    """Validate a separately bound model-Z derivative before current-map checks.

    The base bundle retains its original native/centering lineage. It is not
    recursively loaded as a current-map motion proof or a route cache hit.
    """
    value=payload.get('origin',{}).get('parametric_geometry')
    if value is None:return None,{}
    keys={'schema','base_file','base_sha256','model_file','model_sha256',
          'input_geometry_sha256','output_geometry_sha256','changed_vertices'}
    if not isinstance(value,dict) or set(value)!=keys or value.get('schema')!=MODEL_DERIVATION_SCHEMA:
        raise ValueError('compiled_stair_model_derivation_invalid')
    dependencies={}
    def read(name):
        relative=Path(value[name+'_file'])
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('compiled_stair_model_dependency_escapes_package')
        path=(directory/relative).resolve(strict=True)
        if not path.is_relative_to(directory) or not path.is_file() or path.stat().st_size>MAX_BYTES:
            raise ValueError('compiled_stair_model_dependency_invalid')
        raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
        if not _digest(value[name+'_sha256']) or digest!=value[name+'_sha256']:
            raise ValueError('compiled_stair_model_dependency_hash_mismatch')
        dependencies[str(path)]=digest
        return json.loads(raw)
    base,record=read('base'),read('model')
    if (base.get('schema')!=SCHEMA or base.get('origin',{}).get('parametric_geometry') is not None
            or any(base.get(k)!=payload.get(k) for k in ('source_pcd_sha256','original_source_sha256',
                'frame_id','source_route_sha256','origin_tomogram_sha256','landing_index','source_layer_ids'))
            or base.get('origin')!={k:v for k,v in payload.get('origin',{}).items() if k!='parametric_geometry'}):
        raise ValueError('compiled_stair_model_base_identity_mismatch')
    from .stair_model_geometry import ParametricStairModel
    model=ParametricStairModel(record)
    if (value['model_sha256']!=tomogram.provenance.get('parametric_stair_model_sha256')
            or record['source_map_sha256']!=payload['original_source_sha256']
            or record['planning_map_sha256']!=payload['source_pcd_sha256']):
        raise ValueError('compiled_stair_model_current_identity_mismatch')
    original=np.asarray(base.get('path'),dtype=float);points=np.asarray(payload['path'],dtype=float)
    layers=np.asarray(payload['layer_ids']);old_layers=np.asarray(base.get('layer_ids'))
    expected=derive_model_ground(original,model)
    if (original.shape!=points.shape or not np.array_equal(old_layers,layers)
            or not np.array_equal(expected,points)
            or value['input_geometry_sha256']!=polyline_sha256(original,old_layers)
            or value['output_geometry_sha256']!=polyline_sha256(points,layers)
            or value['changed_vertices']!=np.flatnonzero(np.any(original!=points,axis=1)).tolist()):
        raise ValueError('compiled_stair_model_derived_geometry_mismatch')
    return dict(path=original.tolist(),layer_ids=old_layers.tolist()),dependencies


def validate_centering_origin(evidence, original, points, layers, landing_index, origin_sha256):
    """Check the explicit derivation, not pretend a centered line is native output.

    Current support/occupancy checks remain in load_compiled_stair_connector.
    This contract binds the old literal route and the new quality artifact and
    prevents a changed platform, Z, layer or portal from being called centering.
    """
    old = np.asarray(original.get('path', original.get('path_xyz')), dtype=float)
    old_layers = np.asarray(original.get('layer_ids'))
    new = np.asarray(points, dtype=float)
    ids = np.asarray(layers)
    if (not isinstance(evidence, dict) or evidence.get('schema') != REFINEMENT_SCHEMA
            or evidence.get('execution_authorized') is not False
            or evidence.get('source_route_sha256') != origin_sha256
            or old.ndim != 2 or old.shape[1] != 3 or old.shape != new.shape
            or not np.isfinite(old).all() or not np.isfinite(new).all()
            or old_layers.shape != (len(old),) or not np.array_equal(old_layers, ids)
            or not np.array_equal(old[:, 2], new[:, 2])
            or evidence.get('input_geometry_sha256') != polyline_sha256(old, old_layers)
            or evidence.get('output_geometry_sha256') != polyline_sha256(new, ids)):
        raise ValueError('compiled_stair_centering_identity_or_geometry_invalid')
    changed = np.flatnonzero(np.any(old != new, axis=1)).tolist()
    spans = evidence.get('flight_spans')
    if (not changed or evidence.get('changed_vertices') != changed or not isinstance(spans, list)
            or not 1 <= len(spans) <= 2 or any(i in changed for i in (0, landing_index, len(old)-1))):
        raise ValueError('compiled_stair_centering_scope_invalid')
    allowed = set()
    for span in spans:
        first, last = span.get('first_index'), span.get('last_index')
        leg = span.get('leg')
        if (type(first) is not int or type(last) is not int or not 0 < first < last < len(old)-1
                or leg not in ('stair_lower', 'stair_upper')
                or (leg == 'stair_lower' and last >= landing_index)
                or (leg == 'stair_upper' and first <= landing_index)):
            raise ValueError('compiled_stair_centering_flight_span_invalid')
        selected = set(range(first, last+1))
        if allowed.intersection(selected):
            raise ValueError('compiled_stair_centering_overlapping_spans')
        allowed.update(selected)
    if not set(changed).issubset(allowed):
        raise ValueError('compiled_stair_centering_changed_outside_flights')


def validate_reference_config(value):
    if (not isinstance(value, dict) or set(value) != {'file', 'sha256'}
            or not isinstance(value.get('file'), str) or not Path(value['file']).is_absolute()
            or not _digest(value.get('sha256'))):
        raise ValueError('compiled_stair_connector_reference_invalid')
    return dict(value)


def load_compiled_stair_connector(tomogram, settings):
    """Load once for an immutable map; return independent prepared leg values.

    A configured corrupt/foreign bundle is an early configuration error, never
    an alleged cache hit or a silent runtime change to another stair geometry.
    No bundle leaves all existing native routes unchanged.
    """
    reference = settings.get('compiled_stair_connector')
    if reference is None:
        return None
    reference = validate_reference_config(reference)
    file = Path(reference['file']).resolve(strict=True)
    if not file.is_file() or file.stat().st_size > MAX_BYTES:
        raise ValueError('compiled_stair_connector_file_size_invalid')
    with file.open('rb') as stream:
        encoded = stream.read(MAX_BYTES + 1)
    if len(encoded) > MAX_BYTES or hashlib.sha256(encoded).hexdigest() != reference['sha256']:
        raise ValueError('compiled_stair_connector_file_hash_mismatch')
    payload = json.loads(encoded)
    required = {'schema', 'tomogram_sha256', 'source_pcd_sha256', 'original_source_sha256',
        'frame_id', 'map_settings_sha256', 'minimum_headroom_m', 'source_layer_ids',
        'portals', 'path', 'layer_ids', 'landing_index', 'source_route_sha256', 'origin_tomogram_sha256'}
    if (not isinstance(payload, dict) or required-set(payload) or set(payload)-required-{'origin'}
            or payload.get('schema') != SCHEMA):
        raise ValueError('compiled_stair_connector_schema_invalid')
    for key in ('tomogram_sha256', 'source_pcd_sha256', 'original_source_sha256',
                'map_settings_sha256', 'source_route_sha256', 'origin_tomogram_sha256'):
        if not _digest(payload[key]):
            raise ValueError('compiled_stair_connector_identity_invalid:' + key)
    expected = dict(tomogram_sha256=tomogram.sha256,
        source_pcd_sha256=tomogram.provenance.get('source_sha256'),
        original_source_sha256=tomogram.provenance.get('original_source_sha256'),
        frame_id=settings['frame_id'], map_settings_sha256=map_settings_sha256(settings))
    if any(payload[key] != value for key, value in expected.items()):
        raise ValueError('compiled_stair_connector_map_identity_mismatch')
    if (isinstance(payload['minimum_headroom_m'], bool)
            or payload['minimum_headroom_m'] != tomogram.minimum_headroom_m):
        raise ValueError('compiled_stair_connector_headroom_mismatch')
    source_layers = np.asarray(payload['source_layer_ids'])
    if (not np.issubdtype(source_layers.dtype, np.integer)
            or not np.array_equal(source_layers, tomogram.source_layers)):
        raise ValueError('compiled_stair_connector_source_layers_mismatch')
    portals = {key: settings['anchors'][key] for key in ('entry', 'landing', 'exit')}
    if _canonical(payload['portals']) != _canonical(portals):
        raise ValueError('compiled_stair_connector_portals_mismatch')
    if 'origin' in payload and not isinstance(payload['origin'], dict):
        raise ValueError('compiled_stair_connector_origin_invalid')
    points = np.asarray(payload['path'], dtype=float)
    layers = np.asarray(payload['layer_ids'])
    split = payload['landing_index']
    if (points.ndim != 2 or points.shape[1] != 3 or not 3 <= len(points) <= MAX_POINTS
            or not np.isfinite(points).all() or layers.shape != (len(points),)
            or not np.issubdtype(layers.dtype, np.integer)
            or type(split) is not int or not 0 < split < len(points)-1):
        raise ValueError('compiled_stair_connector_polyline_invalid')
    layers = layers.astype(int)
    model_base,model_dependencies=_model_derivation(payload,tomogram,file.parent)
    for name, index in (('entry', 0), ('landing', split), ('exit', len(points)-1)):
        anchor = portals[name]
        if (not np.array_equal(points[index], anchor['xyz']) or layers[index] != anchor['layer_id']):
            raise ValueError('compiled_stair_connector_exact_endpoint_mismatch:' + name)
    # This source hash is not accepted merely because it appears in a JSON file.
    tomogram.verify_source(settings['source_pcd'], settings['frame_id'])
    from .crossfloor_route import _masked_tomogram, _check_leg, _check_joined
    masked = _masked_tomogram(tomogram, 'stair_lower', settings)
    legs = {}
    identity = dict(file=str(file), sha256=reference['sha256'], **expected,
        source_route_sha256=payload['source_route_sha256'],
        origin_tomogram_sha256=payload['origin_tomogram_sha256'],
        schema=SCHEMA, proof=PROOF)
    if model_base is not None:
        identity['parametric_geometry']=deepcopy(payload['origin']['parametric_geometry'])
    for name, begin, end, selection in (
            ('stair_lower', 'entry', 'landing', slice(None, split+1)),
            ('stair_upper', 'landing', 'exit', slice(split, None))):
        values, ids = points[selection], layers[selection]
        _, _, checks = _check_leg(dict(path=values, layer_ids=ids), tomogram, masked,
            name, portals[begin], portals[end], settings)
        legs[name] = dict(path=values.tolist(), layer_ids=ids.tolist(),
            source_layer_ids=tomogram.source_layers[ids].tolist(),
            algorithm='offline compiled native PCT measured stair connector',
            curve_validation=PROOF, quintic_segments=0, curve_partition_points=len(values),
            compiled_stair_connector=deepcopy(identity), checks=checks)
    edge_legs = ['stair_lower']*split + ['stair_upper']*(len(points)-1-split)
    whole, transitions, profile = _check_joined(points, layers, edge_legs, tomogram, settings)
    from .path_quality import path_quality
    from .stair_refinement import _clearance_quality
    from .corridor_centering import ClearanceFieldCache
    quality = path_quality(points)
    clearance = _clearance_quality(masked, points, layers,
        ClearanceFieldCache(masked, max_layers=min(16, len(set(layers)))))
    diagnostics = dict(applied=False, reason='current_map_verified_compiled_connector_kept',
        algorithm='offline_compiled_native_pct_polyline', physical_safety_certified=False,
        continuity='piecewise polyline; no inherited polynomial or dynamic continuity claim',
        curve_validation=PROOF, curve_partition_points=len(points), landing_index=split,
        before_quality=quality, after_quality=quality, before_clearance=clearance,
        after_clearance=clearance, compiled_stair_connector=deepcopy(identity),
        requested_landing_xyz=list(portals['landing']['xyz']),
        effective_landing_xyz=list(portals['landing']['xyz']), landing_displacement_m=0.,
        span_records=[], checks=whole, stair_profile=profile, layer_transitions=transitions,
        geometry_evidence_direction='lower_to_upper', output_direction='lower_to_upper')
    result=dict(legs=legs, diagnostics=diagnostics,
        dependency_files={str(file): reference['sha256'],**model_dependencies}, origin=deepcopy(payload.get('origin')))
    if model_base is not None:result['model_base_geometry']=model_base
    return result
