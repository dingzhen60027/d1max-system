"""Read-only map capabilities and actual runtime dependencies, not motion authority.

Conditioned multi-floor geometry is a ground-only planning proposal. It is
never a control TF or an execution-eligible source-identity map. This loader
does not start a native planner, ROS graph, SDK client, or rewrite artifacts.
Copied packages must explicitly rebase their deployment metadata and reseal;
matching bytes elsewhere do not justify opening a foreign package's paths.
"""
import hashlib
import json
from pathlib import Path

import yaml

from .source_route import SOURCE_FRAME, PLANNING_FRAME, digest_file

CONDITIONED_GEOMETRY = 'independent_floor_conditioning_visibility_filter_and_measured_return_restore'


def package_version(package):
    """Real artifact identity; relocation is a new sealed deployment version."""
    dependencies=package['dependency_files']
    if package['capabilities']['geometry_kind']=='source_identity':
        manifest_path=package['manifest_path']
        value=('source_identity\n'+package['manifest']['source_sha256']+'\n'+
               dependencies[str(manifest_path)]+'\n'+dependencies[str(package['tomogram_path'])])
    else:
        value='conditioned_multifloor_ground\n'+json.dumps(dependencies,sort_keys=True,separators=(',',':'))
    return hashlib.sha256(value.encode()).hexdigest()


def _digest(value):
    return (isinstance(value, str) and len(value) == 64
            and all(c in '0123456789abcdef' for c in value))


def _path(value):
    if not isinstance(value, (str, Path)) or not Path(value).is_absolute():
        raise ValueError('navigation_map_dependency_requires_absolute_path')
    result = Path(value).resolve(strict=True)
    if not result.is_file():
        raise ValueError('navigation_map_dependency_not_a_file:' + str(result))
    return result


def _local(directory, value):
    relative = Path(value)
    if relative.is_absolute() or '..' in relative.parts or not relative.parts:
        raise ValueError('navigation_map_artifact_must_be_package_relative')
    result = (directory / relative).resolve(strict=True)
    if not result.is_relative_to(directory) or not result.is_file():
        raise ValueError('navigation_map_artifact_escapes_package')
    return result


def load_parametric_stair_model(manifest_path, manifest, *, checked=None):
    """Compatibility entry to the shared model-aware ground bridge loader."""
    from .pointcloud_helpers.ground_path_bridge import load_parametric_stair_model as load
    return load(manifest_path, manifest, checked=checked)


def _compiled_stair_origin_dependency(directory, connector, checked):
    """Bind the literal native-origin polyline; never inherit its old proof."""
    origin = connector.get('origin')
    if (not isinstance(origin, dict) or not isinstance(origin.get('route_file'), str)
            or not _digest(origin.get('source_route_sha256'))):
        raise ValueError('compiled_stair_origin_dependency_invalid')
    source = _local(directory, origin['route_file'])
    if source.stat().st_size > 8*1024*1024:
        raise ValueError('compiled_stair_origin_dependency_too_large')
    checked(source, origin['source_route_sha256'])
    record = json.loads(source.read_text())
    identity = connector['diagnostics']['compiled_stair_connector']
    if (origin['source_route_sha256'] != identity['source_route_sha256']
            or record.get('tomogram_sha256') != identity['origin_tomogram_sha256']
            or record.get('execution_authorized') is not False
            or record.get('status') != 'validated_offline_route'
            or record.get('direction') != 'lower_to_upper'):
        raise ValueError('compiled_stair_origin_identity_mismatch')
    lower, upper = (connector['legs'][name] for name in ('stair_lower', 'stair_upper'))
    points, layers = lower['path']+upper['path'][1:], lower['layer_ids']+upper['layer_ids'][1:]
    if 'model_base_geometry' in connector:
        # The loader has separately checked the bound model-Z derivative and
        # the whole current-map curve. Historical native/centering evidence
        # describes its immutable BASE, not the later modeled heights.
        import numpy as np
        base = connector['model_base_geometry']
        if (not isinstance(origin.get('parametric_geometry'), dict)
                or not isinstance(base, dict) or set(base) != {'path', 'layer_ids'}):
            raise ValueError('compiled_stair_origin_model_base_invalid')
        current_xyz, base_xyz = np.asarray(points, dtype=float), np.asarray(base['path'], dtype=float)
        if (base_xyz.shape != current_xyz.shape or not np.isfinite(base_xyz).all()
                or not np.array_equal(base_xyz[:, :2], current_xyz[:, :2])
                or not np.array_equal(base['layer_ids'], layers)):
            raise ValueError('compiled_stair_origin_model_base_geometry_mismatch')
        points, layers = base['path'], base['layer_ids']
    if (len(record.get('segments', [])) != 2
            or record['segments'][0].get('last_index') != len(lower['path'])-1
            or record['segments'][1].get('first_index') != len(lower['path'])-1):
        raise ValueError('compiled_stair_origin_published_geometry_mismatch')
    refinement = origin.get('centering_refinement')
    if refinement is None:
        if record.get('path', record.get('path_xyz')) != points or record.get('layer_ids') != layers:
            raise ValueError('compiled_stair_origin_published_geometry_mismatch')
    else:
        if (not isinstance(refinement, dict) or set(refinement) != {'file', 'sha256'}
                or not _digest(refinement['sha256'])):
            raise ValueError('compiled_stair_origin_centering_reference_invalid')
        evidence_path = _local(directory, refinement['file'])
        if evidence_path.stat().st_size > 8*1024*1024:
            raise ValueError('compiled_stair_origin_centering_evidence_too_large')
        checked(evidence_path, refinement['sha256'])
        from d1max_pct_planner.compiled_stair_connector import validate_centering_origin
        validate_centering_origin(json.loads(evidence_path.read_text()), record,
            points, layers, len(lower['path'])-1, origin['source_route_sha256'])


def _structured_stair_dependencies(directory, manifest, mapping, checked):
    """Bind offline model evidence; it never supplies observations to SCAN."""
    structure = manifest.get('stair_structured_support')
    if structure is None:
        return
    if (not isinstance(structure, dict) or mapping is None
            or structure.get('schema') != 'd1max.stair_structured_support_binding/v1'
            or structure.get('source_sha256') != manifest['source_sha256']
            or structure.get('input_map_sha256') != manifest['stair_regularization']['input_map_sha256']
            or structure.get('support_semantics') != 'literal_source_rows_only_model_grid_not_observations'):
        raise ValueError('conditioned_multifloor_structured_stair_binding_invalid')
    model_path, evidence_path = [_local(directory, structure[name+'_file']) for name in ('model', 'evidence')]
    if model_path.stat().st_size > 8*1024*1024 or evidence_path.stat().st_size > 128*1024*1024:
        raise ValueError('structured_stair_evidence_budget')
    checked(model_path, structure['model_sha256'])
    checked(evidence_path, structure['evidence_sha256'])
    model = json.loads(model_path.read_text())
    if (model.get('schema') != 'd1max.structured_stair_support.v1'
            or model.get('source_sha256') != structure['source_sha256']
            or model.get('planning_sha256') != structure['input_map_sha256']
            or model.get('source_records_are_literal') is not True
            or model.get('model_samples_are_observations') is not False
            or model.get('original_source_modified') is not False):
        raise ValueError('structured_stair_model_identity_or_semantics')
    import numpy as np
    import zipfile
    with zipfile.ZipFile(evidence_path) as archive:
        if sum(item.file_size for item in archive.infolist()) > 128*1024*1024:
            raise ValueError('structured_stair_evidence_budget')
    with np.load(evidence_path, allow_pickle=False) as archive:
        ids, xyz = archive['source_indices'], archive['source_xyz']
        grid = archive['grid_model_xyz']
        if (ids.ndim != 1 or ids.dtype.kind not in 'iu' or len(ids) > 1_000_000
                or len(np.unique(ids)) != len(ids) or xyz.shape != (len(ids), 3)
                or not np.isfinite(xyz).all() or grid.ndim != 2 or grid.shape[1:] != (3,)
                or len(grid) > 500_000 or not np.isfinite(grid).all()):
            raise ValueError('structured_stair_evidence_geometry')
        order = np.argsort(mapping.source_indices)
        rows = np.searchsorted(mapping.source_indices[order], ids)
        if (np.any(rows >= len(order)) or not np.array_equal(mapping.source_indices[order[rows]], ids)
                or not np.array_equal(mapping.source_xyz[order[rows]], xyz)):
            raise ValueError('structured_stair_evidence_not_literal_mapping_rows')
        from .structured_stair_evidence import validate_structured_evidence
        return validate_structured_evidence(model, {key: archive[key] for key in archive.files}, mapping)


def _structured_cost_dependencies(directory, manifest, mapping, grid, source_points, checked):
    """Load a derived static terrain tensor without mistaking it for lidar data.

    The offline builder owns terrain classification and retains its original
    tensor. Loading checks the bounded edit, literal witnesses and complete
    hash closure; it does not run tomography on the navigation/control lane.
    """
    import numpy as np
    prior = manifest.get('stair_structured_cost')
    provenance = grid.provenance
    if prior is None:
        if 'stair_structured_cost_schema' in provenance:
            raise ValueError('structured_cost_descriptor_missing')
        # Support-model geometry alone lacks the retained-entity exclusion.
        return None
    structure = manifest.get('stair_structured_support', {})
    if (source_points is None or mapping is None or not isinstance(prior, dict)
            or prior.get('schema') != 'd1max.stair_structured_cost/v1'
            or prior.get('policy') != 'source_supported_terrain_seeds_with_retained_entities'
            or prior.get('model_sha256') != structure.get('model_sha256')
            or prior.get('evidence_sha256') != structure.get('evidence_sha256')
            or provenance.get('stair_structured_cost_schema') != prior['schema']
            or provenance.get('stair_structured_model_sha256') != prior['model_sha256']
            or provenance.get('stair_structured_evidence_sha256') != prior['evidence_sha256']):
        raise ValueError('structured_cost_binding_invalid')
    base_path = checked(_local(directory, 'pct/structured_cost_base.npz'),
                        provenance.get('stair_structured_base_sha256'))
    evidence_path = checked(_local(directory, 'pct/structured_cost_evidence.npz'),
                            provenance.get('stair_structured_cost_evidence_sha256'))
    report_path = checked(_local(directory, 'pct/structured_cost_report.json'))
    report = json.loads(report_path.read_text())
    if (report.get('input_tomogram_sha256') != provenance['stair_structured_base_sha256']
            or report.get('current_planning_sha256') != manifest['output_sha256']
            or report.get('original_source_sha256') != manifest['source_sha256']
            or report.get('model_is_explicit_planning_prior_not_observation') is not True
            or report.get('original_ground_ceiling_unchanged') is not True):
        raise ValueError('structured_cost_report_binding_invalid')
    if hashlib.sha256(np.ascontiguousarray(grid.data).tobytes()).hexdigest() != provenance.get('stair_structured_data_sha256'):
        raise ValueError('structured_cost_tensor_changed')
    if (np.any(grid.gradient_x[:, (0, -1), :]) or np.any(grid.gradient_y[:, :, (0, -1)])
            or not np.array_equal(grid.gradient_x[:, 1:-1, :], grid.cost[:, 2:, :]-grid.cost[:, :-2, :])
            or not np.array_equal(grid.gradient_y[:, :, 1:-1], grid.cost[:, :, 2:]-grid.cost[:, :, :-2])):
        raise ValueError('structured_cost_gradient_not_cost_difference')
    with np.load(base_path, allow_pickle=False) as archive:
        before = archive['data']
        if (before.dtype != np.float32 or before.shape != grid.data.shape
                or not np.array_equal(before[3:].view(np.uint32), grid.data[3:].view(np.uint32))
                or not np.array_equal(archive['center'], grid.center)
                or float(archive['resolution']) != grid.resolution
                or float(archive['slice_h0']) != grid.slice_h0
                or float(archive['slice_dh']) != grid.slice_dh
                or np.any(grid.cost > before[0])):
            raise ValueError('structured_cost_changed_surface_or_frame')
    with np.load(evidence_path, allow_pickle=False) as archive:
        values = {name: archive[name] for name in archive.files}
    eligible, changed, raw_changed, witness = [values[name] for name in
        ('eligible_terrain', 'changed_cost', 'modified_raw_seed', 'literal_source_witness')]
    if (any(v.dtype.kind != 'b' or v.shape != grid.cost.shape for v in (eligible, changed, raw_changed))
            or witness.dtype.kind not in 'iu' or witness.shape != grid.cost.shape
            or not np.array_equal(changed, grid.cost != before[0])
            or np.any((changed | raw_changed) & ~eligible)
            or np.any(eligible & (~grid.ground_known | ~grid.ceiling_known |
                                 (grid.headroom < grid.minimum_headroom_m-1e-6)))):
        raise ValueError('structured_cost_edit_scope_invalid')
    ids, xyz, owners = [values[name] for name in
        ('qualified_source_indices', 'qualified_source_xyz', 'qualified_source_surface_id')]
    if (ids.ndim != 1 or ids.dtype.kind not in 'iu' or len(ids) > 20000
            or len(np.unique(ids)) != len(ids) or xyz.shape != (len(ids), 3)
            or owners.shape != ids.shape or owners.dtype.kind not in 'iu'
            or not np.isfinite(xyz).all()):
        raise ValueError('structured_cost_qualified_source_invalid')
    order = np.argsort(mapping.source_indices)
    rows = np.searchsorted(mapping.source_indices[order], ids)
    if (np.any(rows >= len(order)) or not np.array_equal(mapping.source_indices[order[rows]], ids)
            or not np.array_equal(mapping.source_xyz[order[rows]], xyz)
            or not set(map(tuple, xyz)) <= set(map(tuple, source_points))):
        raise ValueError('structured_cost_qualified_source_not_literal')
    with np.load(_local(directory, structure['evidence_file']), allow_pickle=False) as archive:
        original_ids, original_owners = archive['source_indices'], archive['surface_id']
    original_order = np.argsort(original_ids)
    original_rows = np.searchsorted(original_ids[original_order], ids)
    if (np.any(original_rows >= len(original_order))
            or not np.array_equal(original_ids[original_order[original_rows]], ids)
            or not np.array_equal(original_owners[original_order[original_rows]], owners)):
        raise ValueError('structured_cost_qualified_source_owner')
    # Witnesses are exact original rows, never sampled model points. Check the
    # model/domain against every changed layer cell, including overlapping XY.
    ordered = np.argsort(ids)
    selected = np.argwhere(eligible)
    witness_ids = witness[eligible]
    near = np.searchsorted(ids[ordered], witness_ids)
    if np.any(near >= len(ids)) or not np.array_equal(ids[ordered[near]], witness_ids):
        raise ValueError('structured_cost_witness_not_qualified')
    indices = ordered[near]
    xy = grid.center+(selected[:, 1:]-grid.offset)*grid.resolution
    queries = np.c_[xy, grid.ground[eligible]]
    if np.any(np.linalg.norm(queries-xyz[indices], axis=1) > .12+1e-8):
        raise ValueError('structured_cost_source_support_missing')
    model = json.loads(_local(directory, structure['model_file']).read_text())
    covered = np.zeros(len(selected), bool)
    for flight in model['flights']:
        use = owners[indices] == flight['surface_id']
        delta = xy[use]-np.asarray(flight['frame']['center_xy'])
        axis = np.asarray(flight['frame']['uphill_axis_xy'])
        s, t = delta@axis, delta@np.array([-axis[1], axis[0]])
        low, high = flight['frame']['along_range_m']
        sides = flight['retained_structure']['sides']
        negative = np.polyval(sides['negative']['lateral_slope_intercept'], s)
        positive = np.polyval(sides['positive']['lateral_slope_intercept'], s)
        guard = flight['wall_boundary_guard_m']
        expected = s*flight['height_model']['grade']+flight['height_model']['intercept_m']
        valid = ((s >= low) & (s <= high) & (t > negative+guard) & (t < positive-guard)
                 & (abs(queries[use, 2]-expected) <= .08+1e-8))
        covered[use] = valid
    if not covered.all():
        raise ValueError('structured_cost_witness_outside_flight')
    xyz = xyz.copy()
    xyz.flags.writeable = False
    return xyz


def _parametric_geometry_dependencies(directory, grid, model, model_hash, checked, *, planning_cloud_path=None):
    """Verify the explicit geometry stage and return its old-stage input.

    The old cost prior still validates against its own preserved output, not
    against a later tensor with deliberately modeled ground. Neither stage
    can overwrite the other's evidence or pretend its model was observed.
    """
    import numpy as np
    from d1max_pct_planner.tomogram_map import TomogramMap
    provenance = grid.provenance
    if model is None:
        if any(key.startswith('parametric_stair_') for key in provenance):
            raise ValueError('parametric_stair_model_descriptor_missing')
        return grid
    if (provenance.get('parametric_stair_geometry_schema') != model.record['schema']
            or provenance.get('parametric_stair_model_sha256') != model_hash
            or provenance.get('parametric_stair_data_sha256') != hashlib.sha256(grid.data.tobytes()).hexdigest()):
        raise ValueError('parametric_stair_tensor_binding_invalid')
    base_path = checked(_local(directory, 'pct/parametric_geometry_base.npz'),
                        provenance.get('parametric_stair_base_sha256'))
    evidence_path = checked(_local(directory, 'pct/parametric_geometry_evidence.npz'),
                            provenance.get('parametric_stair_evidence_sha256'))
    report_path = checked(_local(directory, 'pct/parametric_geometry_report.json'))
    before = TomogramMap(base_path, minimum_headroom_m=grid.minimum_headroom_m,
        unknown_ceiling_policy=grid.unknown_ceiling_policy, max_ground_step_m=grid.max_ground_step_m)
    report = json.loads(report_path.read_text())
    if (before.data.shape != grid.data.shape or not np.array_equal(before.center, grid.center)
            or before.resolution != grid.resolution or before.slice_h0 != grid.slice_h0
            or before.slice_dh != grid.slice_dh or not np.array_equal(before.source_layers, grid.source_layers)
            or not np.array_equal(before.ground_known, grid.ground_known)
            or report.get('input_data_sha256') != hashlib.sha256(before.data.tobytes()).hexdigest()
            or report.get('output_data_sha256') != provenance['parametric_stair_data_sha256']
            or report.get('model_points_are_observations') is not False
            or report.get('original_points_unchanged') is not True):
        raise ValueError('parametric_stair_geometry_scope_invalid')
    with np.load(evidence_path, allow_pickle=False) as archive:
        masks = {key: archive[key] for key in ('model_domain', 'modeled_ground', 'changed_ground',
            'changed_cost', 'derivative_halo', 'structural_or_entity_domain',
            'terrain_ceiling_reclassified')}
        owner = archive['surface_index']
    if (any(v.dtype != np.bool_ or v.shape != grid.cost.shape for v in masks.values())
            or owner.dtype.kind not in 'iu' or owner.shape != grid.cost.shape):
        raise ValueError('parametric_stair_evidence_shape')
    selected, halo = masks['modeled_ground'], masks['derivative_halo']
    changed_ground = grid.ground.view(np.uint32) != before.ground.view(np.uint32)
    changed_cost = grid.cost != before.cost
    changed_ceiling = grid.ceiling.view(np.uint32) != before.ceiling.view(np.uint32)
    blocked = masks['structural_or_entity_domain']
    reclassified = masks['terrain_ceiling_reclassified']
    if np.any(blocked & (~np.isfinite(grid.cost) | (grid.cost <= grid.COST_THRESHOLD))):
        raise ValueError('parametric_stair_structure_cost_not_blocked')
    if (not np.array_equal(changed_ground, masks['changed_ground'])
            or not np.array_equal(changed_cost, masks['changed_cost'])
            or np.any(selected & ~masks['model_domain'])
            or np.any(blocked & ~masks['model_domain']) or np.any(blocked & selected)
            or np.any(changed_ground & ~selected)
            or np.any(changed_cost & ~(selected | blocked))
            or np.any(reclassified & ~selected) or np.any(changed_ceiling & ~reclassified)
            or np.any(owner[~selected] != -1) or np.any(owner[selected] < 0)
            or np.any(owner[selected] >= len(model.surfaces))
            or not np.array_equal(grid.data[1:3, ~halo], before.data[1:3, ~halo])):
        raise ValueError('parametric_stair_evidence_edit_scope')
    if (np.any(grid.gradient_x[:, (0, -1), :]) or np.any(grid.gradient_y[:, :, (0, -1)])
            or not np.array_equal(grid.gradient_x[:, 1:-1, :], grid.cost[:, 2:, :]-grid.cost[:, :-2, :])
            or not np.array_equal(grid.gradient_y[:, :, 1:-1], grid.cost[:, :, 2:]-grid.cost[:, :, :-2])):
        raise ValueError('parametric_stair_gradient_not_cost_difference')
    points = point_cells = None
    if np.any(reclassified):
        if planning_cloud_path is None:
            raise ValueError('parametric_stair_ceiling_source_required')
        from .pointcloud_helpers.pcd_io import read_pcd
        from d1max_pct_planner.cpu_tomography import round_half_away_from_zero
        points = read_pcd(planning_cloud_path).xyz
        if not np.isfinite(points).all():
            raise ValueError('parametric_stair_ceiling_source_invalid')
        point_cells = round_half_away_from_zero((points[:, :2]-grid.center)/grid.resolution)+grid.offset
        inside = ((point_cells >= 0) & (point_cells < grid.ground.shape[1:])).all(axis=1)
        points, point_cells = points[inside], point_cells[inside]
    for index, surface in enumerate(model.surfaces):
        cells = np.argwhere(selected & (owner == index))
        if not len(cells):
            continue
        xyz_index = tuple(cells.T)
        xy = grid.center + grid.resolution*(cells[:, 1:]-grid.offset)
        z = model.ground_at(surface, xy)
        if (not model.contains(surface, xy).all()
                or np.any(np.abs(before.ground[xyz_index]-z) > surface['ground_replacement_band_m'])
                or not np.array_equal(grid.ground[xyz_index], z.astype(np.float32))
                or model.obstacle_mask(xy, z, xy_padding=0.,
                                       height=grid.minimum_headroom_m).any()
                or np.any(grid.ceiling_known[xyz_index] & (grid.headroom[xyz_index] < grid.minimum_headroom_m-1e-6))):
            raise ValueError('parametric_stair_geometry_not_model_surface')
        ceiling_cells = np.argwhere(reclassified & (owner == index))
        if len(ceiling_cells):
            ids = tuple(ceiling_cells.T)
            xy = grid.center+grid.resolution*(ceiling_cells[:, 1:]-grid.offset)
            z = model.ground_at(surface, xy)
            if (not np.isfinite(before.ceiling[ids]).all()
                    or np.any(before.ceiling[ids] > z+surface['static_terrain_band_m'])):
                raise ValueError('parametric_stair_ceiling_not_terrain')
            # Reclassify a near-terrain old ceiling only against the actual
            # retained PCD column. A model must not erase a real higher return.
            above = points[:, 2]-model.ground_at(surface, points[:, :2]) > surface['static_terrain_band_m']
            nx, ny = grid.ground.shape[1:]
            minimum = np.full(nx*ny, np.inf)
            np.minimum.at(minimum, point_cells[above, 0]*ny+point_cells[above, 1], points[above, 2])
            expected = minimum[ceiling_cells[:, 1]*ny+ceiling_cells[:, 2]]
            expected = np.where(np.isfinite(expected), expected, np.nan).astype(np.float32)
            if not np.array_equal(grid.ceiling[ids], expected, equal_nan=True):
                raise ValueError('parametric_stair_ceiling_not_retained_column')
    return before


def validate_package(map_directory, *, purpose='planning_only'):
    """Return audited artifacts plus a complete, hash-bound runtime copy list.

    ``dependency_files`` maps absolute actual paths to SHA256. It deliberately
    contains files read by the ground bridge/route builder and configuration /
    restoration evidence, not all raw keyframes needed to rebuild a map.
    """
    if purpose not in ('execution', 'planning_only'):
        raise ValueError('navigation_map_purpose_invalid')
    directory = Path(map_directory).resolve(strict=True)
    manifest_path = _local(directory, 'manifest.json')
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('geometry_operation') == 'source_identity':
        # Keep the existing original-record, floor and stair exclusion checks.
        from .source_identity import validate_package as validate_identity
        result = validate_identity(directory)
        floor = result['bridge'].floor_id
        dependencies = [manifest_path, directory/'route.yaml', directory/'tomogram.npz',
                        _path(manifest['source_path']),
                        _local(directory, manifest['output_file']),
                        _local(directory, manifest['source_indices_file'])]
        return dict(result, manifest=manifest, manifest_path=manifest_path,tomogram_path=directory/'tomogram.npz',
            dependency_files={str(p.resolve()): digest_file(p) for p in dependencies},
            capabilities=dict(geometry_kind='source_identity', floor_ids=[floor],
                stairs_planning_enabled=False, execution_geometry_eligible=True,
                planning_frame=SOURCE_FRAME))

    # Loading a planning map into the existing navigation graph is not a
    # motion grant. Its conditioned provenance and ineligible execution
    # geometry stay literal for both purposes; task authorization is separate.
    if (manifest.get('schema') != 1
            or manifest.get('processing_schema') != 'd1max.crossfloor_planning/v1'
            or manifest.get('status') != 'complete' or manifest.get('planning_only') is not True
            or manifest.get('geometry_operation') != CONDITIONED_GEOMETRY
            or manifest.get('frame_id') != PLANNING_FRAME):
        raise ValueError('conditioned_multifloor_manifest_invalid')

    dependencies = {}

    def checked(path, expected=None):
        path = _path(path)
        actual = digest_file(path)
        if expected is not None and (not _digest(expected) or actual != expected):
            raise ValueError('navigation_map_dependency_changed:' + str(path))
        dependencies[str(path)] = actual
        return path

    def linked(value):
        if not isinstance(value, dict) or set(value) != {'path', 'sha256'}:
            raise ValueError('conditioned_multifloor_evidence_binding_invalid')
        return checked(value['path'], value['sha256'])

    checked(manifest_path)
    source = checked(manifest['source_path'], manifest['source_sha256'])
    source_manifest = checked(source.parent/'manifest.json')
    if json.loads(source_manifest.read_text()).get('frame_id') != SOURCE_FRAME:
        raise ValueError('conditioned_multifloor_source_frame_invalid')
    output = checked(_local(directory, manifest['output_file']), manifest['output_sha256'])
    stair_model, stair_model_hash, _ = load_parametric_stair_model(manifest_path, manifest, checked=checked)
    route_path = checked(_local(directory, 'route.yaml'))
    tomogram_path = checked(_local(directory, 'pct/tomogram.npz'))
    floor_path = checked(manifest['floor_processing_manifest'], manifest['floor_processing_manifest_sha256'])
    stage = json.loads(floor_path.read_text())
    if (stage.get('status') != 'complete' or stage.get('planning_only') is not True
            or stage.get('frame_id') != PLANNING_FRAME
            or _path(stage['source_path']) != source
            or stage.get('source_sha256') != manifest['source_sha256']):
        raise ValueError('conditioned_multifloor_floor_stage_binding_invalid')
    trajectory = checked(stage['trajectory_path'], stage['trajectory_sha256'])
    configuration = manifest['configuration']
    floor_evidence = checked(floor_path.parent/'audit/floor_evidence.npz',
                             configuration['floor_evidence_sha256'])
    floors = stage['configuration']['floors']
    if ([floor['id'] for floor in floors] != ['floor1', 'floor2']
            or _path(stage['configuration']['source_pcd']) != source
            or _path(stage['configuration']['trajectory_path']) != trajectory):
        raise ValueError('conditioned_multifloor_floor_ownership_invalid')
    # The bridge consumes the resolved defaults and overrides embedded in the
    # hashed floor manifest, NOT those historical rebuild YAML files. Preserve
    # their original lineage verbatim, without claiming a changed current YAML
    # still matches the old build hash or making it a live runtime dependency.
    defaults = stage['configuration']['floor_stage_defaults']
    defaults_hash = stage['input_hashes'].get(defaults)
    if not isinstance(defaults, str) or not Path(defaults).is_absolute() or not _digest(defaults_hash):
        raise ValueError('conditioned_multifloor_rebuild_lineage_invalid')
    rebuild_references = [dict(path=defaults, sha256=defaults_hash),
                          configuration['floor_conditioning_config'],
                          configuration['route_template']]
    if any(not isinstance(v, dict) or set(v) != {'path', 'sha256'}
           or not isinstance(v['path'], str) or not Path(v['path']).is_absolute()
           or not _digest(v['sha256']) for v in rebuild_references):
        raise ValueError('conditioned_multifloor_rebuild_lineage_invalid')
    linked(configuration['visibility']['evidence'])
    for key in ('measured_restore', 'conditioned_measured_restore'):
        linked(configuration[key]['evidence'])
        linked(configuration[key]['sources'])

    from .pointcloud_helpers.ground_path_bridge import GroundPathBridge
    from .live_map_geometry import validate_map_binding
    from d1max_pct_planner.paths import expand_tree
    from d1max_pct_planner.tomogram_map import TomogramMap
    from d1max_pct_planner.crossfloor_preview import load_config
    bridge = GroundPathBridge.from_artifacts(manifest_path)
    if (bridge.source_frame != SOURCE_FRAME or bridge.planning_frame != PLANNING_FRAME
            or list(bridge.floors) != ['floor1', 'floor2']):
        raise ValueError('conditioned_multifloor_bridge_capability_invalid')
    # The optional stair derivative is part of this same map, not a runtime
    # coordinate transform. Freeze its observed-source correspondence and
    # pre-regularization input alongside the PCD so a copied package cannot
    # accidentally combine new planning heights with old local support.
    stair_mapping = getattr(bridge, 'stair_mapping', None)
    if stair_mapping is not None:
        for path, expected in stair_mapping.dependency_files.items():
            checked(path, expected)
    structured_source_points = _structured_stair_dependencies(directory, manifest, stair_mapping, checked)
    raw = expand_tree(yaml.safe_load(route_path.read_text()))
    if (_path(raw['source_pcd']) != output or _path(raw['tomogram_path']) != tomogram_path):
        raise ValueError('conditioned_multifloor_foreign_route_artifact')
    tomogram = TomogramMap(tomogram_path,
        minimum_headroom_m=raw.get('minimum_headroom_m'),
        unknown_ceiling_policy=raw.get('unknown_ceiling_policy', 'allow_unobserved'),
        max_ground_step_m=raw['limits']['max_ground_step_m'])
    _, settings = load_config(route_path, tomogram)
    validate_map_binding(manifest_path, bridge, settings, source_frame=SOURCE_FRAME)
    provenance = tomogram.provenance
    if stair_mapping is not None and (
            provenance.get('stair_regularization_schema') != stair_mapping.SCHEMA
            or provenance.get('stair_mapping_sha256') != stair_mapping.mapping_hash):
        raise ValueError('conditioned_multifloor_stair_tomogram_binding_invalid')
    if stair_mapping is None and ('stair_regularization_schema' in provenance
                                 or 'stair_mapping_sha256' in provenance):
        raise ValueError('conditioned_multifloor_stair_correspondence_missing')
    if (provenance.get('planning_only') is not True
            or provenance.get('geometry_operation') != CONDITIONED_GEOMETRY
            or provenance.get('processing_schema') != manifest['processing_schema']
            or provenance.get('original_source_sha256') != manifest['source_sha256']
            or provenance.get('source_processing_manifest_sha256') != dependencies[str(manifest_path)]
            or _path(provenance.get('source_processing_manifest', '')) != manifest_path
            or _path(provenance.get('source_pcd', '')) != output
            or _path(provenance.get('original_source_pcd', '')) != source):
        raise ValueError('conditioned_multifloor_tomogram_binding_invalid')
    observed_cost_output = _parametric_geometry_dependencies(directory, tomogram,
        stair_model, stair_model_hash, checked, planning_cloud_path=output)
    structured_source_points = _structured_cost_dependencies(directory, manifest, stair_mapping,
        observed_cost_output, structured_source_points, checked)
    # The same production builder must be able to reconstruct literal source
    # support; its nonidentity output remains execution_eligible=False.
    from .source_route import SourceRouteBuilder, original_stair_record_lineage
    lineage = original_stair_record_lineage(manifest_path, manifest, manifest.get('input_points', 0))
    if lineage is not None:
        _, lineage_path, lineage_sha256 = lineage
        checked(lineage_path, lineage_sha256)
    SourceRouteBuilder.from_artifacts(manifest_path, tomogram, bridge)
    if settings.get('compiled_stair_connector') is not None:
        from d1max_pct_planner.compiled_stair_connector import load_compiled_stair_connector
        connector_file = _path(settings['compiled_stair_connector']['file'])
        if not connector_file.is_relative_to(directory):
            raise ValueError('compiled_stair_connector_escapes_package')
        compiled = load_compiled_stair_connector(tomogram, settings)
        for path, expected in compiled['dependency_files'].items():
            checked(path, expected)
        _compiled_stair_origin_dependency(directory, compiled, checked)
    return dict(manifest=manifest, manifest_path=manifest_path,bridge=bridge, tomogram=tomogram, route=settings,
        tomogram_path=tomogram_path, dependency_files=dependencies,
        structured_stair_source_points=structured_source_points,
        stair_model=stair_model, stair_model_sha256=stair_model_hash,
        rebuild_input_references=rebuild_references,
        capabilities=dict(geometry_kind='conditioned_multifloor_ground',
            floor_ids=['floor1', 'floor2'], stairs_planning_enabled=True,
            execution_geometry_eligible=False, planning_frame=PLANNING_FRAME))
