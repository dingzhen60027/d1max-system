"""Build a separate, source-indexed stair-support planning candidate.

No service, selector or installed map is changed. The original PCD, current
planning PCD and record lineage are hashed, then an explicitly non-rigid
planning-only Z derivative is generated and the same production PCT equations
are rerun. This is an offline preprocessing tool, not a navigation entrypoint.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
import yaml
from scipy.spatial import cKDTree

from .pcd_io import read_pcd, write_derived_z
from .stair_regularization import (
    RegularizationLimits, StairSurface, regularize_stair_support, extend_measured_stair_support,
)


def _sha(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1048576), b''):
            value.update(chunk)
    return value.hexdigest()


def _json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False,
                                   allow_nan=False)+'\n')


def classified_support_mask(points, tomogram, *, surface_tolerance_m=.05):
    """Same cell/XYZ association as the existing production source support."""
    keep = np.zeros(len(points), dtype=bool)
    for begin in range(0, len(points), 8192):
        batch = points[begin:begin+8192]
        cells = np.rint((batch[:, :2]-tomogram.center)/tomogram.resolution).astype(np.int64)
        cells += tomogram.offset
        inside = ((cells[:, 0] >= 0) & (cells[:, 0] < tomogram.nx)
                  & (cells[:, 1] >= 0) & (cells[:, 1] < tomogram.ny))
        ids = np.flatnonzero(inside)
        x, y = cells[ids, 0], cells[ids, 1]
        ground = tomogram.ground[:, x, y]
        keep[begin+ids] = np.any(
            tomogram.valid[:, x, y] & tomogram.ground_known[:, x, y]
            & (abs(ground-batch[ids, 2]) <= surface_tolerance_m+1e-9), axis=0)
    return keep


def preserve_point_slice_membership(original_z, proposed_z, slice_h0, slice_dh, layers):
    """Undo only edits that move a measured point between PCT height slices.

    Production tomography uses float32 ``point_z <= slice_z``. Even a small
    correction across that boundary changes which surfaces are visible to an
    entire slice, not merely the height noise within its observed support.
    Keep the original membership; do not clamp to an artificial boundary.
    """
    original = np.asarray(original_z, dtype=np.float32)
    corrected = np.asarray(proposed_z, dtype=np.float32).copy()
    boundaries = np.asarray([slice_h0+i*slice_dh for i in range(layers)], np.float32)
    old = np.searchsorted(boundaries, original, side='left')
    new = np.searchsorted(boundaries, corrected, side='left')
    changed_membership = old != new
    corrected[changed_membership] = original[changed_membership]
    return corrected, dict(rolled_back_points=int(changed_membership.sum()),
        record_indices=np.flatnonzero(changed_membership).tolist(),
        policy='preserve_float32_point_below_slice_membership')


def preserve_observed_pct_support(before_npz, original_xyz, proposed_z, pct_config,
                                  preprocessing, *, maximum_passes=6):
    """Reject local proposals that lose existing PCT support; never paint costs.

    PCT stores the highest input point below each slice, not a surface median.
    Correcting just one of several overlapping returns can increase an adjacent
    step gradient. Evaluate the actual production CPU equations and undo only
    proposal records near affected columns. This is an offline geometric edit
    transaction, not a new runtime navigation gate or an altered PCT equation.
    """
    from d1max_pct_planner.cpu_tomography import tomogram_from_points
    from d1max_pct_planner.official_pipeline import preprocess

    with np.load(before_npz, allow_pickle=False) as archive:
        old = archive['data']
        center = archive['center'].astype(float)
    original_xyz = np.asarray(original_xyz, dtype=np.float32)
    # Quantize now exactly as the PCD writer will. A float64-only check would
    # miss a highest-point or slice-boundary change after writing float32.
    corrected = np.asarray(proposed_z, dtype=np.float32).copy()
    resolution = float(pct_config['resolution'])
    old_known = np.isfinite(old[3])
    old_valid = old_known & (old[0] <= 20.)
    old_x, old_y = old.shape[2:]
    changed_ids = np.flatnonzero(corrected != original_xyz[:, 2])
    if not len(changed_ids):
        return corrected, dict(passes=[], proposed_changed_points=0, retained_changed_points=0)
    relative = (original_xyz[changed_ids, :2]-center)/resolution
    cells = np.copysign(np.floor(abs(relative)+.5), relative).astype(np.int64)
    column_xy = cells*resolution+center
    tree = cKDTree(column_xy)
    passes = []
    for iteration in range(maximum_passes+1):
        candidate = original_xyz.copy()
        candidate[:, 2] = corrected
        used, _ = preprocess(candidate, preprocessing)
        payload, _ = tomogram_from_points(used, pct_config)
        new = payload['data']
        if new.shape != old.shape or not np.array_equal(payload['center'], center):
            raise ValueError('stair_candidate_changed_pct_grid_geometry')
        known_lost = old_known & ~np.isfinite(new[3])
        valid_lost = old_valid & (~np.isfinite(new[3]) | (new[0] > 20.))
        lost = known_lost | valid_lost
        bad_columns = np.argwhere(np.any(lost, axis=0))
        row = dict(pass_index=iteration, lost_slice_cells=int(lost.sum()),
                   lost_known_slice_cells=int(known_lost.sum()),
                   lost_valid_slice_cells=int(valid_lost.sum()),
                   lost_xy_columns=len(bad_columns),
                   changed_points=int(np.count_nonzero(corrected != original_xyz[:, 2])))
        passes.append(row)
        if not len(bad_columns):
            return corrected, dict(passes=passes, proposed_changed_points=len(changed_ids),
                retained_changed_points=row['changed_points'],
                original_valid_support_cells_preserved=True,
                original_known_support_cells_preserved=True,
                maximum_passes=maximum_passes, rollback_radius_m=3.*resolution,
                production_equations_unchanged=True)
        if iteration == maximum_passes:
            break
        bad_xy = (bad_columns-[old_x//2, old_y//2])*resolution+center
        nearby = tree.query_ball_point(bad_xy, 3.*resolution+1e-9, p=np.inf, workers=1)
        offsets = np.unique(np.concatenate([np.asarray(item, dtype=np.int64) for item in nearby]))
        restore = changed_ids[offsets]
        row['rolled_back_points'] = int(np.count_nonzero(corrected[restore] != original_xyz[restore, 2]))
        if not row['rolled_back_points']:
            break
        corrected[restore] = original_xyz[restore, 2]
    raise ValueError('stair_regularization_would_lose_existing_pct_support')


def rebind_internal_portal_heights(route, before, after, maximum_displacement_m):
    """Keep portal XY/layer, update stale ground Z from that exact new cell.

    The internal landing is map data, not a requested robot body pose. Carrying
    its old ground Z into a regularized map can create a little vertical hairpin
    even though the new observed cell is traversable. No nearest-cell search,
    new floor selection, endpoint relocation or validity override is used.
    """
    if (not np.array_equal(before.center, after.center)
            or not np.array_equal(before.offset, after.offset)
            or before.resolution != after.resolution
            or before.ground.shape != after.ground.shape):
        raise ValueError('portal_rebinding_requires_same_pct_grid')
    records = []
    for name in ('entry', 'landing', 'exit'):
        portal = route['anchors'][name]
        xyz = np.asarray(portal['xyz'], float)
        layer = int(portal['layer_id'])
        cell = np.rint((xyz[:2]-before.center)/before.resolution).astype(int)+before.offset
        x, y = cell
        if (not 0 <= layer < len(before.ground) or not 0 <= x < before.nx
                or not 0 <= y < before.ny):
            raise ValueError('portal_outside_original_pct_domain')
        old, new = float(before.ground[layer, x, y]), float(after.ground[layer, x, y])
        if old == new:
            continue
        if (not np.isfinite([old, new]).all() or not before.valid[layer, x, y]
                or not after.valid[layer, x, y]
                or abs(xyz[2]-old) > 1e-6
                or abs(new-xyz[2]) > maximum_displacement_m+1e-6):
            raise ValueError('portal_rebinding_lacks_bounded_observed_ground')
        portal['xyz'] = [float(xyz[0]), float(xyz[1]), new]
        records.append(dict(name=name, cell=[int(x), int(y)], layer_id=layer,
            old_xyz=xyz.tolist(), new_xyz=portal['xyz'],
            old_observed_ground_m=old, new_observed_ground_m=new,
            displacement_m=abs(new-old),
            policy='same_xy_same_retained_layer_observed_ground'))
    return records


def merge_support_extension(existing, addition, source_xyz):
    """Append new original rows; never recompute or overwrite old corrections."""
    keys = ('source_indices', 'source_xyz', 'planning_xyz', 'surface_id')
    old = {key: np.asarray(existing[key]) for key in keys}
    added = {key: np.asarray(getattr(addition, key)) for key in keys}
    ids = added['source_indices']
    if (np.intersect1d(old['source_indices'], ids).size
            or len(np.unique(ids)) != len(ids) or np.any(ids < 0) or np.any(ids >= len(source_xyz))
            or not np.array_equal(added['source_xyz'], source_xyz[ids])
            or not np.array_equal(added['planning_xyz'][:, :2], source_xyz[ids, :2])):
        raise ValueError('stair_extension_would_overwrite_or_invent_source')
    merged = {key: np.concatenate((old[key], added[key]), axis=0) for key in keys}
    order = np.argsort(merged['source_indices'], kind='stable')
    merged = {key: value[order] for key, value in merged.items()}
    restored = np.searchsorted(merged['source_indices'], old['source_indices'])
    if any(not np.array_equal(merged[key][restored], old[key]) for key in keys):
        raise ValueError('stair_extension_changed_previous_mapping')
    return merged


def checked_compiled_origin_artifacts(record, origin_map_directory=None):
    """Explicit relocation of a sealed historical map, never hash-based search."""
    if origin_map_directory is None:
        pcd = Path(record['source_pcd']).resolve(strict=True)
        grid = Path(record['tomogram_path']).resolve(strict=True)
    else:
        directory = Path(origin_map_directory).resolve(strict=True)
        if not directory.is_dir():
            raise ValueError('compiled_stair_origin_override_requires_map_directory')
        pcd = (directory/'processed_map.pcd').resolve(strict=True)
        grid = (directory/'pct/tomogram.npz').resolve(strict=True)
        if not pcd.is_relative_to(directory) or not grid.is_relative_to(directory):
            raise ValueError('compiled_stair_origin_override_escapes_map_directory')
    if (_sha(pcd) != record.get('source_pcd_sha256')
            or _sha(grid) != record.get('tomogram_sha256')
            or record.get('source_tomogram_sha256') != record.get('tomogram_sha256')):
        raise ValueError('compiled_stair_origin_map_identity_invalid')
    return pcd, grid


def extension_route_config(route, output):
    """A changed map never inherits a compiled descriptor for the old map."""
    from copy import deepcopy
    copied = deepcopy(route)
    copied.pop('compiled_stair_connector', None)
    copied.update(source_pcd=str(output/'processed_map.pcd'),
                  tomogram_path=str(output/'pct/tomogram.npz'), output_directory=str(output/'route'))
    return copied


def saved_landing_request(current, saved):
    """A rebuild may already name the saved effective platform connector.

    Accept only the literal saved requested OR effective portal. Geometry of
    the historical requested-to-effective connector is still rechecked below;
    arbitrary new portals cannot inherit an old route's proof.
    """
    requested = dict(xyz=saved.get('requested_xyz', saved['xyz']),
                     layer_id=saved.get('requested_layer_id', saved['layer_id']))
    effective = dict(xyz=saved['xyz'], layer_id=saved['layer_id'])
    if current != requested and current != effective:
        raise ValueError('compiled_stair_original_requested_landing_changed')
    return requested


def compile_stair_connector(package, source_route_path, *, origin_map_directory=None):
    """Seal a saved native PCT stair polyline against this exact new map.

    The old polynomial is NOT claimed to remain proven. Its published polyline
    is independently checked in the new original/masked cells and mapped back
    to literal original support before being offered to the existing cache.
    This offline operation does not run PCT search, SCAN or robot control.
    """
    from copy import deepcopy
    from d1max_pct_planner.compiled_stair_connector import SCHEMA, PROOF, map_settings_sha256
    from d1max_pct_planner.crossfloor_route import (
        LEGS, _masked_tomogram, _check_leg, _check_joined, checked_landing_adjustment,
    )
    from d1max_pct_planner.tomogram_map import TomogramMap
    from d1max_pct_scan.navigation_map_package import validate_package, package_version
    from d1max_pct_scan.source_route import SourceRouteBuilder
    from d1max_pct_scan.static_route_validation import validate_static_route

    package = Path(package).resolve(strict=True)
    source_route_path = Path(source_route_path).resolve(strict=True)
    if source_route_path.stat().st_size > 8*1024*1024:
        raise ValueError('compiled_stair_origin_route_too_large')
    for name in ('compiled_stair_connector.json', 'compiled_stair_origin_route.json'):
        if (package/name).exists():
            raise ValueError('compiled_stair_artifact_already_exists')
    origin_digest = _sha(source_route_path)
    record = json.loads(source_route_path.read_text())
    if (record.get('status') != 'validated_offline_route'
            or record.get('execution_authorized') is not False
            or record.get('direction') != 'lower_to_upper'):
        raise ValueError('compiled_stair_requires_saved_lower_to_upper_planning_route')
    segments = record.get('segments', [])
    if [segment.get('name') for segment in segments] != ['stair_lower', 'stair_upper']:
        raise ValueError('compiled_stair_requires_stair_only_route')
    points = np.asarray(record.get('path', record.get('path_xyz')), float)
    layers = np.asarray(record.get('layer_ids'))
    if (points.ndim != 2 or points.shape[1] != 3 or not 3 <= len(points) <= 12000
            or not np.isfinite(points).all() or layers.shape != (len(points),)
            or not np.issubdtype(layers.dtype, np.integer)):
        raise ValueError('compiled_stair_origin_geometry_invalid')
    split = segments[0].get('last_index')
    if (type(split) is not int or not 0 < split < len(points)-1
            or segments[0].get('first_index') != 0
            or segments[1].get('first_index') != split
            or segments[1].get('last_index') != len(points)-1
            or [(item.get('from'), item.get('to')) for item in segments]
                != [('entry', 'landing'), ('landing', 'exit')]
            or record.get('edge_legs') != ['stair_lower']*split+['stair_upper']*(len(points)-split-1)):
        raise ValueError('compiled_stair_origin_segment_ownership_invalid')
    checked = validate_package(package)
    grid, settings, bridge = checked['tomogram'], checked['route'], checked['bridge']
    if settings.get('compiled_stair_connector') is not None:
        raise ValueError('compiled_stair_existing_descriptor_requires_new_package')
    origin_pcd, origin_grid_path = checked_compiled_origin_artifacts(record, origin_map_directory)
    if record.get('frame_id') != settings['frame_id']:
        raise ValueError('compiled_stair_origin_map_identity_invalid')
    frozen = dict(checked['dependency_files']) | {str(source_route_path): origin_digest,
        str(origin_pcd): _sha(origin_pcd), str(origin_grid_path): _sha(origin_grid_path)}
    origin_grid = TomogramMap(origin_grid_path,
        minimum_headroom_m=grid.minimum_headroom_m,
        unknown_ceiling_policy=grid.unknown_ceiling_policy,
        max_ground_step_m=grid.max_ground_step_m)
    if (origin_grid.provenance.get('original_source_sha256') != checked['manifest']['source_sha256']
            or origin_grid.resolution != grid.resolution
            or not np.array_equal(origin_grid.center, grid.center)
            or not np.array_equal(origin_grid.source_layers, grid.source_layers)
            or origin_grid.data.shape != grid.data.shape
            or origin_grid.slice_h0 != grid.slice_h0 or origin_grid.slice_dh != grid.slice_dh):
        raise ValueError('compiled_stair_origin_source_or_grid_changed')
    origin_geometry_check = origin_grid.validate_path(points, layers)
    masked = _masked_tomogram(grid, 'stair_lower', settings)
    landing = record['anchors']['landing']
    requested = saved_landing_request(dict(xyz=settings['anchors']['landing']['xyz'].tolist(),
                     layer_id=settings['anchors']['landing']['layer_id']), landing)
    historical_settings = deepcopy(settings)
    historical_settings['anchors']['landing'] = dict(xyz=np.asarray(requested['xyz'], float),
                                                    layer_id=requested['layer_id'])
    adjustment = checked_landing_adjustment(grid, masked, historical_settings, landing)
    selected = deepcopy(settings)
    selected['anchors']['landing'] = dict(xyz=np.asarray(landing['xyz'], float),
                                         layer_id=int(landing['layer_id']))
    checks = {}
    for segment in segments:
        name = segment['name']
        _, begin, end = next(item for item in LEGS if item[0] == name)
        first, last = segment['first_index'], segment['last_index']
        _, _, checks[name] = _check_leg(dict(path=points[first:last+1], layer_ids=layers[first:last+1]),
            grid, masked, name, selected['anchors'][begin], selected['anchors'][end], selected)
    whole, transitions, profile = _check_joined(points, layers, record['edge_legs'], grid, selected)
    # Ground mapping/support uses the current complete source correspondence,
    # not old source XYZ or a ramp surface manufactured for SCAN.
    builder = SourceRouteBuilder.from_artifacts(checked['manifest_path'], grid, bridge)
    proposed = dict(record, source_tomogram_sha256=grid.sha256, execution_authorized=False)
    validated = validate_static_route(proposed, grid, bridge, builder=builder,
                                      map_version_id=package_version(checked))
    geometry = validated['route_snapshot'].payload()['geometry_evidence']
    if not geometry['measured_source_support']['all_samples_supported']:
        raise ValueError('compiled_stair_current_original_support_missing')
    portals = {name: dict(xyz=selected['anchors'][name]['xyz'].tolist(),
                         layer_id=selected['anchors'][name]['layer_id'])
               for name in ('entry', 'landing', 'exit')}
    bundle = dict(schema=SCHEMA, tomogram_sha256=grid.sha256,
        source_pcd_sha256=checked['manifest']['output_sha256'],
        original_source_sha256=checked['manifest']['source_sha256'], frame_id=settings['frame_id'],
        map_settings_sha256=map_settings_sha256(selected), minimum_headroom_m=grid.minimum_headroom_m,
        source_layer_ids=grid.source_layers.astype(int).tolist(), portals=portals,
        path=points.tolist(), layer_ids=layers.astype(int).tolist(), landing_index=split,
        source_route_sha256=origin_digest, origin_tomogram_sha256=origin_grid.sha256,
        origin=dict(route_file='compiled_stair_origin_route.json', source_route_sha256=origin_digest,
            requested_landing=requested, landing_adjustment=adjustment,
            policy='saved_native_pct_published_polyline_revalidated_not_old_quintic',
            curve_validation=PROOF, execution_authorized=False,
            compilation_evidence=dict(previous_map_polyline_check=origin_geometry_check,
                current_map_leg_checks=checks, current_map_whole_check=whole,
                current_layer_transitions=transitions, current_stair_profile=profile,
                original_source_support=geometry['measured_source_support'],
                source_ground_projection=geometry['projection'],
                current_source_route_hash=validated['route_snapshot'].route_hash,
                source_route_hash_scope='offline geometry audit before compiled descriptor publication')))
    if any(_sha(path) != digest for path, digest in frozen.items()):
        raise ValueError('compiled_stair_input_changed_during_validation')
    shutil.copy2(source_route_path, package/'compiled_stair_origin_route.json')
    _json(package/'compiled_stair_connector.json', bundle)
    route = yaml.safe_load((package/'route.yaml').read_text())
    route['anchors']['landing'] = portals['landing']
    route['compiled_stair_connector'] = dict(file=str(package/'compiled_stair_connector.json'),
        sha256=_sha(package/'compiled_stair_connector.json'))
    (package/'route.yaml').write_text(yaml.safe_dump(route, sort_keys=False))
    sealed = validate_package(package)
    return dict(schema=SCHEMA, file='compiled_stair_connector.json',
        sha256=route['compiled_stair_connector']['sha256'], source_route_sha256=origin_digest,
        published_points=len(points), landing_index=split, original_requested_landing=requested,
        effective_landing=portals['landing'], curve_validation=PROOF,
        source_support=geometry['measured_source_support'], projection=geometry['projection'],
        current_map_revalidated=True, saved_polyline_geometry_unchanged=True,
        native_search_run=False, original_source_unchanged=True,
        dependency_files=sealed['dependency_files'])


def build_support_extension(package, output, settings_path=None, *, build_pct=True, surface_ids=None,
                            compiled_stair_route=None, compiled_stair_origin_map=None,
                            structured_corridor=False):
    """Extend a sealed regularized map from still-unmodified measured records.

    Existing model coefficients, planning corrections and source correspondences
    are immutable. This never reruns fitting on an already conditioned cloud.
    PCT is rebuilt from the resulting literal PCD, not cost-painted.
    """
    from .ground_path_bridge import StairGroundMapping
    from d1max_pct_planner.tomogram_map import TomogramMap
    from d1max_pct_scan.navigation_map_package import validate_package

    package, output = Path(package).resolve(strict=True), Path(output).resolve()
    if output.exists() or output == package or output.is_relative_to(package):
        raise ValueError('candidate_must_be_a_new_separate_directory')
    if (not isinstance(surface_ids, (list, tuple)) or not surface_ids
            or any(type(value) is not int for value in surface_ids)):
        raise ValueError('stair_extension_requires_explicit_reviewed_surfaces')
    if compiled_stair_route is not None and not build_pct:
        raise ValueError('compiled_stair_requires_complete_production_pct')
    if compiled_stair_origin_map is not None and compiled_stair_route is None:
        raise ValueError('compiled_stair_origin_override_requires_saved_route')
    bound = validate_package(package)
    manifest = json.loads((package/'manifest.json').read_text())
    if manifest.get('stair_cost_prior') is not None:
        raise ValueError('stair_extension_requires_original_pct_not_cost_prior')
    descriptor = manifest.get('stair_regularization')
    if not isinstance(descriptor, dict):
        raise ValueError('stair_extension_requires_existing_mapping')
    settings_path = Path(settings_path or package/'stair_regularization_settings.json').resolve(strict=True)
    settings = json.loads(settings_path.read_text())
    original_settings = json.loads((package/'stair_regularization_settings.json').read_text())
    if (settings.get('schema') != 'd1max.stair_regularization_settings.v1'
            or settings.get('source_sha256') != manifest['source_sha256']):
        raise ValueError('stair_extension_settings_source_mismatch')
    limits = RegularizationLimits(**settings.get('limits', {}))
    if (limits != RegularizationLimits(**original_settings.get('limits', {}))
            or settings['observed_region'] != original_settings['observed_region']
            or limits.maximum_displacement_m != descriptor['max_displacement_m']):
        raise ValueError('stair_extension_must_preserve_original_limits')
    current_path = package/manifest['output_file']
    source_path = Path(manifest['source_path']).resolve(strict=True)
    inputs = dict(bound['dependency_files'])
    for path in (settings_path, package/'stair_regularization_settings.json',
                 package/'record_lineage.npz', package/'pct/pipeline.yaml'):
        inputs[str(path)] = _sha(path)
    current, source = read_pcd(current_path), read_pcd(source_path)
    mapping = StairGroundMapping.from_artifact(descriptor, base_dir=package,
        source_xyz=source.xyz, source_sha256=manifest['source_sha256'],
        output_path=current_path, output_sha256=manifest['output_sha256'])
    old = dict(source_indices=mapping.source_indices, source_xyz=mapping.source_xyz,
               planning_xyz=mapping.planning_xyz, surface_id=mapping.surface_ids)
    with np.load(package/'record_lineage.npz', allow_pickle=False) as archive:
        retained = archive['retained_source_indices']
    if (retained.dtype != np.int64 or len(retained) > len(current.xyz)
            or np.any(retained < 0) or np.any(retained >= len(source.xyz))
            or len(np.unique(retained)) != len(retained)):
        raise ValueError('invalid_original_record_lineage')
    region = settings['observed_region']
    region_points = source.xyz[retained]
    selected = np.all((region_points >= region['min']) & (region_points <= region['max']), axis=1)
    rows = np.flatnonzero(selected)
    ids = retained[rows]
    owner_by_source = np.full(len(source.xyz), -1, np.int32)
    owner_by_source[old['source_indices']] = old['surface_id']
    structured = None
    if structured_corridor:
        from .stair_structured_support import build_structured_stair_support
        flight_ids = [model['id'] for model in descriptor['surfaces']
                      if model['kind'] == 'flight_support_envelope']
        if set(surface_ids) != set(flight_ids):
            raise ValueError('structured_corridor_requires_both_measured_flights')
        structured = build_structured_stair_support(source.xyz, current.xyz, retained, old,
            descriptor['surfaces'], source_sha256=manifest['source_sha256'],
            planning_sha256=manifest['output_sha256'])
        addition = structured['addition']
    else:
        addition = extend_measured_stair_support(source.xyz[ids], ids, owner_by_source[ids],
            descriptor['surfaces'], limits=limits, surface_ids=surface_ids)
    if not len(addition.source_indices) or not addition.diagnostics['changed_points']:
        raise ValueError('no_additional_connected_measured_support')
    source_to_row = np.full(len(source.xyz), -1, np.int64)
    source_to_row[retained] = np.arange(len(retained))
    added_rows = source_to_row[addition.source_indices]
    if (np.any(added_rows < 0)
            or not np.array_equal(current.records[added_rows], source.records[addition.source_indices])):
        raise ValueError('stair_extension_requires_still_literal_source_records')
    proposed = current.xyz[:, 2].copy()
    proposed[added_rows] = addition.planning_xyz[:, 2]
    tomogram = TomogramMap(package/'pct/tomogram.npz', max_ground_step_m=.17)
    pct = yaml.safe_load((package/'pct/pipeline.yaml').read_text())
    proposed, slice_guard = preserve_point_slice_membership(current.xyz[:, 2], proposed,
        tomogram.slice_h0, tomogram.slice_dh, len(tomogram.ground))
    proposed, pct_guard = preserve_observed_pct_support(package/'pct/tomogram.npz', current.xyz,
        proposed, pct['pct'], pct.get('preprocessing', {}))
    if not np.any(proposed != current.xyz[:, 2]):
        raise ValueError('no_stair_extension_preserves_existing_pct_support')
    if any(_sha(path) != digest for path, digest in inputs.items()):
        raise ValueError('stair_extension_input_changed_during_compute')
    output.mkdir(parents=True)
    # Keep the exact current input and old mapping as replayable evidence.
    shutil.copy2(current_path, output/'original_planning.pcd')
    shutil.copy2(package/descriptor['mapping_file'], output/'previous_stair_ground_mapping.npz')
    shutil.copy2(package/'manifest.json', output/'previous_map_manifest.json')
    shutil.copy2(settings_path, output/'stair_regularization_settings.json')
    shutil.copytree(package/'floor_stage', output/'floor_stage')
    stage_path = output/'floor_stage/manifest.json'
    stage = json.loads(stage_path.read_text())
    trajectory = Path(stage['trajectory_path']).resolve(strict=True)
    copied_trajectory = output/'floor_stage'/trajectory.name
    shutil.copy2(trajectory, copied_trajectory)
    stage['trajectory_path'] = str(copied_trajectory)
    stage['configuration']['trajectory_path'] = str(copied_trajectory)
    _json(stage_path, stage)
    for name in ('record_lineage.npz', 'reviewed_flat_corridors.json', 'support_repair_audit.json'):
        if (package/name).is_file():
            shutil.copy2(package/name, output/name)
    # Runtime measured-restore evidence stays immutable; make it self-contained.
    configuration = manifest['configuration']
    for section, key in [('visibility', 'evidence'), *[(name, key)
            for name in ('measured_restore', 'conditioned_measured_restore') for key in ('evidence', 'sources')]]:
        link = configuration[section][key]
        original = Path(link['path']).resolve(strict=True)
        destination = output/'stage_evidence'/(section+'_'+key+original.suffix)
        destination.parent.mkdir(exist_ok=True)
        shutil.copy2(original, destination)
        if _sha(destination) != link['sha256']:
            raise ValueError('stair_extension_restore_evidence_changed')
        link['path'] = str(destination)
    write_derived_z(output/'processed_map.pcd', current, np.arange(len(current.xyz)), proposed)
    written = read_pcd(output/'processed_map.pcd')
    addition.planning_xyz[:, 2] = written.xyz[added_rows, 2]
    merged = merge_support_extension(old, addition, source.xyz)
    changed = written.records['z'] != current.records['z']
    if (np.any(changed & ~np.isin(np.arange(len(changed)), added_rows))
            or not np.array_equal(written.xyz[:, :2], current.xyz[:, :2])
            or any(not np.array_equal(written.records[field], current.records[field])
                   for field in current.fields if field != 'z')):
        raise ValueError('stair_extension_changed_previous_or_unrelated_records')
    np.savez_compressed(output/'stair_ground_mapping.npz', **merged)
    models = [dict(model) for model in (structured['widened_models'] if structured is not None
                                       else descriptor['surfaces'])]
    for model in models:
        owned = merged['surface_id'] == model['id']
        model['associated_points'] = int(owned.sum())
        model['changed_points'] = int(np.count_nonzero(
            merged['planning_xyz'][owned, 2] != merged['source_xyz'][owned, 2]))
        model['connected_extension_points'] = int(np.count_nonzero(addition.surface_id == model['id']))
        if model['kind'] != 'unmodified_observed_support':
            before, after_points = merged['source_xyz'][owned], merged['planning_xyz'][owned]
            target = (np.full(len(before), model['height_m']) if model['kind'] == 'landing_plane'
                else (before[:, :2]-model['center_xy']) @ np.asarray(model['uphill_axis_xy'])
                    * model['grade']+model['intercept_m'])
            model['maximum_displacement_m'] = float(np.max(abs(before[:, 2]-after_points[:, 2]), initial=0.))
            model['residual_before_rms_m'] = float(np.sqrt(np.mean((target-before[:, 2])**2)))
            model['residual_after_rms_m'] = float(np.sqrt(np.mean((target-after_points[:, 2])**2)))
    updated = dict(descriptor, input_map_file='original_planning.pcd', input_map_sha256=_sha(current_path),
        mapping_file='stair_ground_mapping.npz', mapping_sha256=_sha(output/'stair_ground_mapping.npz'),
        output_map_file='processed_map.pcd', output_map_sha256=_sha(output/'processed_map.pcd'), surfaces=models)
    manifest.update(output_file='processed_map.pcd', output_sha256=updated['output_map_sha256'],
        stair_regularization=updated, floor_processing_manifest=str(output/'floor_stage/manifest.json'),
        floor_processing_manifest_sha256=_sha(output/'floor_stage/manifest.json'))
    manifest.setdefault('processing_steps', []).append(dict(
        operation=('extend_structure_bounded_original_stair_support' if structured is not None
                   else 'extend_connected_original_stair_support'), descriptor='stair_regularization',
        previous_mapping_sha256=descriptor['mapping_sha256'], previous_output_sha256=descriptor['output_map_sha256'],
        selected_surface_ids=addition.diagnostics['selected_surface_ids'],
        model_refit=False, existing_rows_unchanged=True, preserves_original_localization=True))
    if structured is not None:
        # The sampled model is evidence, never a synthetic cloud or live FREE
        # observation. Only the separately checked literal additions enter PCD.
        _json(output/'stair_structured_model.json', structured['model'])
        np.savez_compressed(output/'stair_structured_evidence.npz', **structured['arrays'])
        manifest['stair_structured_support'] = dict(
            schema='d1max.stair_structured_support_binding/v1',
            model_file='stair_structured_model.json', model_sha256=_sha(output/'stair_structured_model.json'),
            evidence_file='stair_structured_evidence.npz', evidence_sha256=_sha(output/'stair_structured_evidence.npz'),
            source_sha256=manifest['source_sha256'],
            input_map_sha256=manifest['stair_regularization']['input_map_sha256'],
            support_semantics='literal_source_rows_only_model_grid_not_observations')
        manifest['stair_structured_cost'] = dict(schema='d1max.stair_structured_cost/v1',
            policy='source_supported_terrain_seeds_with_retained_entities',
            model_sha256=manifest['stair_structured_support']['model_sha256'],
            evidence_sha256=manifest['stair_structured_support']['evidence_sha256'])
    _json(output/'manifest.json', manifest)
    route = extension_route_config(yaml.safe_load((package/'route.yaml').read_text()), output)
    (output/'route.yaml').write_text(yaml.safe_dump(route, sort_keys=False))
    pct.update(source_pcd=str(output/'processed_map.pcd'), source_processing_manifest=str(output/'manifest.json'),
               output_directory=str(output/'pct'))
    (output/'pct_pipeline.yaml').write_text(yaml.safe_dump(pct, sort_keys=False))
    report = dict(schema='d1max.stair_measured_support_extension/v1', inputs=inputs,
        diagnostics=addition.diagnostics, settings=settings,
        previous_mapping_rows=len(old['source_indices']), mapping_rows=len(merged['source_indices']),
        added_source_indices=addition.source_indices.tolist(), changed_source_indices=retained[np.flatnonzero(changed)].tolist(),
        changed_records=int(changed.sum()), existing_mapping_rows_unchanged=True,
        existing_corrected_points_unchanged=True, original_localization_unchanged=True,
        unchanged_non_z_fields=True, source_model_refit=False, production_pct_equations_unchanged=True,
        pct_support_preservation=pct_guard, slice_membership_preservation=slice_guard,
        build_pct=build_pct, output_directory=str(output))
    _json(output/'stair_regularization_report.json', report)
    if build_pct:
        from d1max_pct_planner.official_pipeline import build
        build(output/'pct_pipeline.yaml')
        after = TomogramMap(output/'pct/tomogram.npz', max_ground_step_m=.17)
        report['internal_portal_height_updates'] = rebind_internal_portal_heights(
            route, tomogram, after, limits.maximum_displacement_m)
        (output/'route.yaml').write_text(yaml.safe_dump(route, sort_keys=False))
        checked = validate_package(output)
        report['runtime_dependency_files'] = checked['dependency_files']
        if compiled_stair_route is not None:
            report['compiled_stair_connector'] = compile_stair_connector(output, compiled_stair_route,
                origin_map_directory=compiled_stair_origin_map)
            report['runtime_dependency_files'] = report['compiled_stair_connector']['dependency_files']
        _json(output/'stair_regularization_report.json', report)
    if any(_sha(path) != digest for path, digest in inputs.items()):
        raise ValueError('stair_extension_input_changed_during_export')
    return report


def build_candidate(package, output, settings_path, *, build_pct=True):
    from d1max_pct_planner.tomogram_map import TomogramMap

    package, output = Path(package).resolve(strict=True), Path(output).resolve()
    settings_path = Path(settings_path).resolve(strict=True)
    if output.exists() or output == package or output.is_relative_to(package):
        raise ValueError('candidate_must_be_a_new_separate_directory')
    settings = json.loads(settings_path.read_text())
    if settings.get('schema') != 'd1max.stair_regularization_settings.v1':
        raise ValueError('unsupported_stair_settings_schema')
    manifest = json.loads((package/'manifest.json').read_text())
    current_pcd = package/manifest['output_file']
    source_pcd = Path(manifest['source_path']).resolve(strict=True)
    inputs = {str(path): _sha(path) for path in
              (package/'manifest.json', current_pcd, source_pcd,
               package/'pct/tomogram.npz', package/'pct/pipeline.yaml',
               package/'route.yaml', package/'record_lineage.npz', settings_path)}
    if (inputs[str(current_pcd)] != manifest['output_sha256']
            or inputs[str(source_pcd)] != manifest['source_sha256']
            or settings['source_sha256'] != manifest['source_sha256']
            or settings['input_map_sha256'] != manifest['output_sha256']):
        raise ValueError('stair_regularization_input_hash_mismatch')
    current, source = read_pcd(current_pcd), read_pcd(source_pcd)
    with np.load(package/'record_lineage.npz', allow_pickle=False) as archive:
        retained = archive['retained_source_indices']
    if (retained.dtype != np.int64 or len(np.unique(retained)) != len(retained)
            or np.any(retained < 0) or np.any(retained >= len(source.xyz))
            or len(retained) > len(current.xyz)):
        raise ValueError('invalid_original_record_lineage')
    region = settings['observed_region']
    selected = np.all((current.xyz >= region['min']) & (current.xyz <= region['max']), axis=1)
    if selected[len(retained):].any():
        raise ValueError('stair_region_contains_unbound_restored_records')
    rows = np.flatnonzero(selected)
    ids = retained[rows]
    if not np.array_equal(current.records[rows], source.records[ids]):
        raise ValueError('protected_stair_records_are_not_original')
    tomogram = TomogramMap(package/'pct/tomogram.npz', max_ground_step_m=.17)
    seeds = classified_support_mask(current.xyz[rows], tomogram,
                                    surface_tolerance_m=min(.06, tomogram.resolution*.5))
    limits = RegularizationLimits(**settings.get('limits', {}))
    surfaces = [StairSurface(**item) for item in settings['surfaces']]
    result = regularize_stair_support(source.xyz[ids], ids, seeds, surfaces, limits=limits)
    if not result.diagnostics['changed_points']:
        raise ValueError('no_proven_stair_support_correction')
    # Keep all previously classified stair support in the mapping even where
    # it needs no correction. A partial correction must not erase its original
    # source support domain from downstream reference construction.
    stage = json.loads(Path(manifest['floor_processing_manifest']).read_text())
    old_domain = np.zeros(len(source.xyz), dtype=bool)
    for protected in stage['configuration']['protected_regions']:
        old_domain |= np.all((source.xyz >= protected['min'])
                             & (source.xyz <= protected['max']), axis=1)
    old_source_ids = np.flatnonzero(old_domain)
    old_source_support = classified_support_mask(
        source.xyz[old_source_ids], tomogram,
        surface_tolerance_m=min(.06, tomogram.resolution*.5))
    old_unchanged = np.setdiff1d(old_source_ids[old_source_support], result.source_indices)
    mapping_ids = np.r_[old_unchanged, result.source_indices].astype(np.int64)
    mapping_xyz = np.r_[source.xyz[old_unchanged], result.planning_xyz]
    mapping_surface = np.r_[np.zeros(len(old_unchanged), np.int32), result.surface_id]
    order = np.argsort(mapping_ids)
    mapping_ids, mapping_xyz, mapping_surface = mapping_ids[order], mapping_xyz[order], mapping_surface[order]
    source_to_row = np.full(len(source.xyz), -1, np.int64)
    source_to_row[retained] = np.arange(len(retained))
    mapping_rows = source_to_row[mapping_ids]
    present = mapping_rows >= 0
    if np.any((~present) & (mapping_surface != 0)):
        raise ValueError('changed_source_mapping_row_not_in_current_planning_map')
    # Existing support association uses the original PCD, including original
    # records omitted by later visibility filtering. Preserve that identity
    # domain without restoring those records into the planning PCD.
    derived_z = current.xyz[:, 2].copy()
    derived_z[mapping_rows[present]] = mapping_xyz[present, 2]
    pct = yaml.safe_load((package/'pct/pipeline.yaml').read_text())
    derived_z, slice_guard = preserve_point_slice_membership(
        current.xyz[:, 2], derived_z, tomogram.slice_h0, tomogram.slice_dh,
        len(tomogram.ground))
    derived_z, pct_guard = preserve_observed_pct_support(
        package/'pct/tomogram.npz', current.xyz, derived_z,
        pct['pct'], pct.get('preprocessing', {}))
    mapping_xyz[present, 2] = derived_z[mapping_rows[present]]
    if not np.any(derived_z != current.xyz[:, 2]):
        raise ValueError('no_stair_correction_preserves_existing_pct_support')
    output.mkdir(parents=True)
    # Keep exact before records beside the new derivative for displacement
    # verification. Dependency rebasing is explicit and does not change the
    # older floor-conditioning operation or its measured-source evidence.
    shutil.copy2(current_pcd, output/'original_planning.pcd')
    shutil.copy2(settings_path, output/'stair_regularization_settings.json')
    for name in ('floor_stage',):
        shutil.copytree(package/name, output/name)
    for name in ('record_lineage.npz', 'reviewed_flat_corridors.json', 'support_repair_audit.json'):
        if (package/name).is_file():
            shutil.copy2(package/name, output/name)
    write_derived_z(output/'processed_map.pcd', current,
                    np.arange(len(current.xyz)), derived_z)
    written = read_pcd(output/'processed_map.pcd')
    # Persist the values actually representable in the PCD (normally float32),
    # not an unrounded float64 ideal that the PCT builder never saw.
    mapping_xyz[present, 2] = written.xyz[mapping_rows[present], 2]
    np.savez_compressed(output/'stair_ground_mapping.npz',
                        source_indices=mapping_ids, source_xyz=source.xyz[mapping_ids],
                        planning_xyz=mapping_xyz, surface_id=mapping_surface)
    changed = written.records['z'] != current.records['z']
    if (np.any(changed & ~selected) or np.any(changed[len(retained):])
            or not np.array_equal(written.xyz[:, :2], current.xyz[:, :2])
            or any(not np.array_equal(written.records[f], current.records[f])
                   for f in current.fields if f != 'z')):
        raise ValueError('stair_candidate_changed_unrelated_records')
    models = [dict(m) for m in result.models if m['associated_points'] > 0]
    for model in models:
        mask = mapping_surface == model['id']
        before, after = source.xyz[mapping_ids[mask]], mapping_xyz[mask]
        if model['kind'] == 'flight_support_envelope':
            target = ((before[:, :2]-model['center_xy']) @ model['uphill_axis_xy']
                      * model['grade']+model['intercept_m'])
        else:
            target = np.full(len(before), model['height_m'])
        model['proposed_changed_points'] = model['changed_points']
        model['changed_points'] = int(np.count_nonzero(before[:, 2] != after[:, 2]))
        model['maximum_displacement_m'] = float(np.max(abs(after[:, 2]-before[:, 2])))
        model['residual_after_rms_m'] = float(np.sqrt(np.mean((target-after[:, 2])**2)))
    if len(old_unchanged):
        models.insert(0, dict(id=0, kind='unmodified_observed_support',
                             name='unchanged_original_classified_stair_support',
                             associated_points=len(old_unchanged), changed_points=0))
    descriptor = dict(schema='d1max.stair_regularization.v1',
        mapping_file='stair_ground_mapping.npz', mapping_sha256=_sha(output/'stair_ground_mapping.npz'),
        source_sha256=manifest['source_sha256'],
        input_map_file='original_planning.pcd', input_map_sha256=manifest['output_sha256'],
        output_map_file='processed_map.pcd', output_map_sha256=_sha(output/'processed_map.pcd'),
        surfaces=models, max_displacement_m=limits.maximum_displacement_m,
        association_max_distance_m=.12,
        source_points_are_literal=True, xy_unchanged=True, planning_only=True,
        synthetic_points=0, physical_tread_dimensions_recovered=False)
    manifest.update(output_sha256=descriptor['output_map_sha256'],
                    output_file='processed_map.pcd', stair_regularization=descriptor,
                    floor_processing_manifest=str(output/'floor_stage/manifest.json'))
    manifest['floor_processing_manifest_sha256'] = _sha(output/'floor_stage/manifest.json')
    steps = list(manifest.get('processing_steps', []))
    steps.append(dict(operation='observed_stair_support_structure_regularization',
                      descriptor='stair_regularization', preserves_original_localization=True))
    manifest['processing_steps'] = steps
    # The copied floor stage's own hashes/configuration describe the unchanged
    # earlier operation; it is intentionally not regenerated with these stairs.
    _json(output/'manifest.json', manifest)
    route = yaml.safe_load((package/'route.yaml').read_text())
    route.update(source_pcd=str(output/'processed_map.pcd'),
                 tomogram_path=str(output/'pct/tomogram.npz'), output_directory=str(output/'route'))
    (output/'route.yaml').write_text(yaml.safe_dump(route, sort_keys=False))
    pct.update(source_pcd=str(output/'processed_map.pcd'),
               source_processing_manifest=str(output/'manifest.json'),
               output_directory=str(output/'pct'))
    (output/'pct_pipeline.yaml').write_text(yaml.safe_dump(pct, sort_keys=False))
    report = dict(schema='d1max.stair_regularization_report.v1', inputs=inputs,
        settings=settings, diagnostics=result.diagnostics, models=models,
        mapping_rows=len(mapping_ids), changed_records=int(changed.sum()),
        original_classified_support_points=int(old_source_support.sum()),
        identity_source_support_not_restored_into_planning_pcd=int(np.count_nonzero(~present)),
        output_points=len(written.xyz), original_localization_unchanged=True,
        unchanged_non_z_fields=True, unchanged_outside_stair_region=True,
        maximum_z_displacement_m=float(np.max(abs(written.xyz[:, 2]-current.xyz[:, 2]))),
        pct_support_preservation=pct_guard, slice_membership_preservation=slice_guard,
        build_pct=build_pct, output_directory=str(output))
    changed_rows = np.flatnonzero(changed)
    dh = tomogram.slice_dh
    h0 = tomogram.slice_h0-dh
    old_slice = np.ceil((current.xyz[changed_rows, 2]-h0)/dh).astype(np.int64)-1
    new_slice = np.ceil((written.xyz[changed_rows, 2]-h0)/dh).astype(np.int64)-1
    crossing = old_slice != new_slice
    report['slice_boundary_crossings'] = dict(count=int(crossing.sum()),
        changed_record_indices=changed_rows[crossing].tolist(),
        source_indices=retained[changed_rows[crossing]].tolist(),
        old_first_ground_slice=old_slice[crossing].tolist(),
        new_first_ground_slice=new_slice[crossing].tolist(),
        ground_height_m=h0, slice_dh_m=dh)
    _json(output/'stair_regularization_report.json', report)
    print(json.dumps({'stage': 'stair_candidate_generated',
                      'changed_records': int(changed.sum()), 'mapping_rows': len(mapping_ids)}, ensure_ascii=False), flush=True)
    if build_pct:
        from d1max_pct_planner.official_pipeline import build
        build(output/'pct_pipeline.yaml')
        new_tomogram = TomogramMap(output/'pct/tomogram.npz', max_ground_step_m=.17)
        report['internal_portal_height_updates'] = rebind_internal_portal_heights(
            route, tomogram, new_tomogram, limits.maximum_displacement_m)
        (output/'route.yaml').write_text(yaml.safe_dump(route, sort_keys=False))
        _json(output/'stair_regularization_report.json', report)
    for path, digest in inputs.items():
        if _sha(path) != digest:
            raise ValueError('input_changed_while_building_stair_candidate:' + path)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--settings', type=Path)
    parser.add_argument('--extend-measured-support', action='store_true',
        help='Extend existing measured support components; preserve all previous corrections and fixed models')
    parser.add_argument('--structured-corridor', action='store_true',
        help='Use retained bilateral wall/rail boundaries to extend original stair support')
    parser.add_argument('--extension-surface-id', type=int, action='append',
        help='Reviewed existing surface to extend; repeat for more than one surface')
    parser.add_argument('--compiled-stair-route', type=Path,
        help='Saved validated native stair-only up route to revalidate as a fixed published polyline')
    parser.add_argument('--compiled-stair-origin-map', type=Path,
        help='Explicit sealed old map directory after relocation; original route PCD/NPZ hashes must match')
    parser.add_argument('--skip-pct', action='store_true')
    args = parser.parse_args()
    if args.extend_measured_support:
        if not args.extension_surface_id:
            parser.error('--extension-surface-id is required to state the reviewed extension scope')
        build_support_extension(args.package, args.output, args.settings, build_pct=not args.skip_pct,
            surface_ids=args.extension_surface_id, compiled_stair_route=args.compiled_stair_route,
            compiled_stair_origin_map=args.compiled_stair_origin_map,
            structured_corridor=args.structured_corridor)
    else:
        if args.structured_corridor:
            parser.error('--structured-corridor requires --extend-measured-support')
        if args.compiled_stair_route is not None or args.compiled_stair_origin_map is not None:
            parser.error('--compiled-stair-route is only supported with --extend-measured-support')
        if args.settings is None:
            parser.error('--settings is required for the initial regularization build')
        build_candidate(args.package, args.output, args.settings, build_pct=not args.skip_pct)


if __name__ == '__main__':
    main()
