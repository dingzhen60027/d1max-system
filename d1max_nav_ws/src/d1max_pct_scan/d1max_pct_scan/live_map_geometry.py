"""Display-only measured terrain in the original localization coordinates.

Never warp live sensor/body data or publish a planning-frame TF. Unsupported
ground inverse estimates are omitted, not filled or marked traversable.
"""
from pathlib import Path
import numpy as np
import yaml


def validate_map_binding(manifest_path, bridge, settings, *, source_frame=None):
    """Bind separately audited PCT/bridge artifacts to the same map version.

    Matching frame names do not establish provenance. ``load_config`` checks
    the tomogram against its PCD; the bridge checks its own manifest. This
    additional edge ensures they refer to the very same conditioned PCD.
    """
    if (settings['frame_id'] != bridge.planning_frame
            or Path(settings['source_pcd']).resolve()
            != (Path(manifest_path).resolve().parent / 'processed_map.pcd').resolve()
            or (source_frame is not None and bridge.source_frame != source_frame)):
        raise ValueError('PCT source/frame differs from audited bridge')


def initial_goal_from_candidates(planning_xyz, original_xyz, source_indices, candidates, anchor_xyz):
    """Select against the planning anchor, then return the paired original point."""
    chosen = candidates[np.argmin(np.linalg.norm(
        planning_xyz[source_indices[candidates], :2]-np.asarray(anchor_xyz)[:2], axis=1))]
    return original_xyz[chosen].tolist()


def _display_converter(points, labels, bridge):
    """Prepare model membership once, without changing the route bridge.

    Display cells are independent. The legacy bridge may reject a batch for
    one unsupported cell; recursively splitting it must not repeat thousands
    of scalar model/polygon queries. Keep its exact conversions and failures,
    including contiguous outside-model runs, but reuse point-local model work.
    """
    from .pointcloud_helpers.ground_path_bridge import ParametricGroundPathBridge
    if not isinstance(bridge, ParametricGroundPathBridge) or not np.isfinite(points).all():
        return lambda ids: bridge.to_localization_ground(points[ids], labels[ids]).xyz
    model = bridge.stair_model
    owner = np.full(len(points), -1, dtype=int)
    overlaps = np.zeros(len(points), dtype=bool)
    heights = np.zeros(len(points), dtype=float)
    invalid = np.zeros(len(points), dtype=bool)
    for number, surface in enumerate(model.surfaces):
        ids = np.flatnonzero(model.contains(surface, points[:, :2]))
        if not len(ids):
            continue
        ground = model.ground_at(surface, points[ids, :2])
        keep = np.abs(ground-points[ids, 2]) <= surface['ground_replacement_band_m']
        ids, ground = ids[keep], ground[keep]
        overlaps[ids] |= owner[ids] >= 0
        owner[ids], heights[ids] = number, ground
    # Equal-height seams and ambiguous overlapping interiors retain the
    # production scalar tie-break/error contract, not a new display heuristic.
    for index in np.flatnonzero(overlaps):
        try:
            surface = bridge.model_surface_at(points[index])
            heights[index] = model.ground_at(surface, points[index, :2])
        except ValueError:
            invalid[index] = True

    def convert(ids):
        if invalid[ids].any():
            raise ValueError('ambiguous_stair_model_surface')
        modeled = owner[ids] >= 0
        if not modeled.any():
            return bridge.original.to_localization_ground(points[ids], labels[ids]).xyz
        result = points[ids].copy()
        result[modeled, 2] = heights[ids[modeled]]
        outside = np.flatnonzero(~modeled)
        for run in np.split(outside, np.flatnonzero(np.diff(outside) > 1)+1):
            if len(run):
                selected = ids[run]
                result[run] = bridge.original.to_localization_ground(
                    points[selected], labels[selected]).xyz
        return result
    return convert


def map_surfaces(xyz, bridge, settings, *, surface_kind='traversable'):
    if surface_kind not in ('traversable', 'blocked'):
        raise ValueError('Unsupported terrain display surface kind')
    points = np.asarray(xyz, dtype=float).reshape(-1, 3)
    if getattr(bridge,'projection_kind',None)=='source_identity':
        # Display real source-coordinate surfaces; no plane or inverse field.
        low,high=bridge.input_z_band
        keep=(points[:,2]>=low)&(points[:,2]<=high)
        keep &= ~np.all((points[:,:2]>=bridge.forbidden[0])&(points[:,:2]<=bridge.forbidden[1]),axis=1)
        ids=np.flatnonzero(keep)
        return points[ids].astype('<f4'),ids
    labels = np.full(len(points), '', dtype='<U16')
    protected = np.zeros(len(points), dtype=bool)
    for region in bridge.protected_regions:
        protected |= np.all((points >= region['min']) & (points <= region['max']), axis=1)
    labels[protected] = 'stairs'
    for name, floor in bridge.floors.items():
        near = np.abs(points[:, 2]-floor.reference_z_m) <= bridge.limits.max_ground_residual_m
        labels[(labels == '') & near] = name
    lo, hi = settings['stair_roi']
    stair = np.all((points >= lo) & (points <= hi), axis=1) & (labels == '')
    midpoint = (bridge.floors['floor1'].reference_z_m + bridge.floors['floor2'].reference_z_m)/2
    labels[stair & (points[:, 2] <= midpoint)] = 'stair_lower'
    labels[stair & (points[:, 2] > midpoint)] = 'stair_upper'
    output, indices = [], []
    mapping = getattr(bridge, 'stair_mapping', None)
    if mapping is not None and surface_kind == 'blocked':
        # Regularization changes only measured stair support rows, not walls
        # or railings in the protected ROI. Those blocked display cells retain
        # their original coordinates; requiring a ground correspondence would
        # silently hide the surrounding structure. This never admits free or
        # unknown cells, and does not change any planner/collision input.
        ids = np.flatnonzero(protected)
        if len(ids):
            distances = mapping._planning_tree.query(points[ids], workers=1)[0]
            unchanged = ids[distances > mapping.radius+1e-9]
            if len(unchanged):
                output.append(points[unchanged].copy())
                indices.append(unchanged)
                labels[unchanged] = ''
    prepared_convert = _display_converter(points, labels, bridge)
    def convert(ids):
        if not len(ids):
            return
        try:
            result = prepared_convert(ids)
        except ValueError:
            # Grid samples are independent display cells, not a new route.
            # Split to isolate unsupported samples without weakening the bridge.
            if len(ids) > 1:
                middle = len(ids)//2
                convert(ids[:middle]); convert(ids[middle:])
            return
        output.append(result)
        indices.append(ids)
    for label in np.unique(labels[labels != '']):
        ids = np.flatnonzero(labels == label)
        for begin in range(0, len(ids), 8192):
            convert(ids[begin:begin+8192])
    return ((np.concatenate(output).astype('<f4'), np.concatenate(indices)) if output
            else (np.empty((0, 3), dtype='<f4'), np.empty(0, dtype=int)))


def build_layers(session, *, on_traversable=None):
    from .pointcloud_helpers.ground_path_bridge import GroundPathBridge
    from d1max_pct_planner.tomogram_map import TomogramMap
    from d1max_pct_planner.tomogram_display import surface_clouds
    from d1max_pct_planner.crossfloor_preview import load_config, visible_surfaces
    import json
    identity=json.loads(Path(session['planning_manifest']).read_text()).get('geometry_operation')=='source_identity'
    if identity:
        from .source_identity import SourceIdentityBridge
        from d1max_pct_planner.singlefloor_route import load_config
        bridge=SourceIdentityBridge.from_artifacts(session['planning_manifest'])
    else:
        bridge = GroundPathBridge.from_artifacts(session['planning_manifest'])
    if bridge.source_frame != session['frame_id']:
        raise ValueError('Terrain visualization frame does not match localization')
    raw = yaml.safe_load(Path(session['crossfloor_route_config']).read_text())
    tomogram = TomogramMap(session['tomogram_npz'],
        minimum_headroom_m=raw.get('minimum_headroom_m'),
        unknown_ceiling_policy=raw.get('unknown_ceiling_policy', 'allow_unobserved'),
        max_ground_step_m=raw['limits']['max_ground_step_m'])
    _, settings = load_config(session['crossfloor_route_config'], tomogram)
    validate_map_binding(session['planning_manifest'], bridge, settings,
                         source_frame=session['frame_id'])
    xyz, costs, blocked = surface_clouds(tomogram, range(tomogram.layers))
    visible=(lambda values: (values[:,2]>=settings['floor_z_ranges']['lower'][0]) &
             (values[:,2]<=settings['floor_z_ranges']['lower'][1])) if identity else (
             lambda values:visible_surfaces(values,settings))
    keep = visible(xyz)
    xyz, costs = xyz[keep], costs[keep]
    blocked = blocked[visible(blocked)]
    if len(xyz)+len(blocked) > 1000000:
        raise ValueError('Terrain visualization exceeds one-million-cell display budget')
    original, ids = map_surfaces(xyz, bridge, settings)
    metadata = {'frame_id': bridge.source_frame, 'planning_frame': bridge.planning_frame,
        'resolution': tomogram.resolution, 'traversable_cells': len(original),
        'unsupported_traversable_cells': len(xyz)-len(ids),
        'source_tomogram_sha256': tomogram.sha256, 'ground_display_only': True,
        'estimated_inverse_not_tf': True, 'motion_enabled': False}
    floor = session.get('current_floor', 'floor1')
    if identity:
        # Bind what RViz publishes to the exact planning package. A same-frame
        # cloud or a cleaned PCD elsewhere is not evidence of deployment.
        from .source_route import digest_file
        cleanup = bridge.manifest.get('source_record_cleanup', {})
        metadata.update(estimated_inverse_not_tf=False,geometry_operation='source_identity',
            planning_manifest=str(bridge.manifest_path),
            planning_manifest_sha256=digest_file(bridge.manifest_path),
            original_source_sha256=bridge.manifest['source_sha256'],
            retained_source_records=bridge.manifest.get('retained_points'),
            reviewed_cleanup_enabled=cleanup.get('enabled', False),
            reviewed_removed_source_records=cleanup.get('removed_points', 0))
        if len(original):
            metadata['initial_goal_xyz']=original[len(original)//2].tolist()
    else:
        candidates = np.flatnonzero(np.abs(xyz[ids, 2]-bridge.floors[floor].reference_z_m)
                                   <= bridge.limits.max_ground_residual_m)
        if len(candidates):
            anchor = raw['anchors']['start' if floor == 'floor1' else 'goal']['xyz']
            metadata['initial_goal_xyz'] = initial_goal_from_candidates(xyz, original, ids, candidates, anchor)
    if on_traversable is not None:
        # Send the complete green layer before building the optional red one.
        # Unknown red counts are omitted; final metadata below stays unchanged.
        on_traversable(original, costs[ids], dict(metadata))
    red, red_ids = map_surfaces(blocked, bridge, settings, surface_kind='blocked')
    metadata.update(blocked_cells=len(red),
        unsupported_omitted_cells=len(xyz)+len(blocked)-len(ids)-len(red_ids),
        unsupported_blocked_cells=len(blocked)-len(red_ids))
    return original, costs[ids], red, metadata


def colored_records(xyz, costs=None):
    points = np.asarray(xyz, dtype='<f4').reshape(-1, 3)
    if len(points) > 1000000 or not np.isfinite(points).all():
        raise ValueError('Invalid bounded terrain display geometry')
    values = np.empty(len(points), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    for axis, name in enumerate(('x', 'y', 'z')):
        values[name] = points[:, axis]
    if costs is None:
        values['rgb'] = np.uint32((220 << 16) | (68 << 8) | 68)
    else:
        costs = np.asarray(costs, dtype=float)
        if costs.shape != (len(points),) or not np.isfinite(costs).all() or np.any((costs < 0) | (costs > 20)):
            raise ValueError('Display colors require the true traversable PCT costs')
        weight = costs/20.
        values['rgb'] = ((45+200*weight).astype(np.uint32) << 16) | ((225-45*weight).astype(np.uint32) << 8) | 65
    return values
