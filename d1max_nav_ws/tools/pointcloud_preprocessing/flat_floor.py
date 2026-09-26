"""Observed-support, single-floor conditioning for a *derived planning map*.

This is deliberately not a SLAM correction or a semantic dynamic-object filter.
No points are synthesized and XY never changes. A bounded, observed floor field
translates complete local columns before small floor residuals are snapped.
The result must not be silently substituted for the original localization map.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


DEFAULTS = {
    "anchor_radius_m": 0.60,
    "anchor_offset_min_m": -0.80,
    "anchor_offset_max_m": -0.15,
    "anchor_mode_bin_m": 0.025,
    "anchor_mode_halfwidth_m": 0.045,
    "anchor_min_points": 12,
    "anchor_residual_m": 0.025,
    "anchor_max_tilt_deg": 8.0,
    "anchor_min_xy_spread_m": 0.035,
    "field_neighbors": 6,
    "field_max_distance_m": 2.50,
    "field_idw_softening_m": 0.15,
    "field_max_disagreement_m": 0.20,
    "reference_z_m": -0.50,
    "planning_min_relative_z_m": -0.08,
    "support_cell_m": 0.20,
    "support_halfwidth_m": 0.055,
    "support_min_points": 5,
    "support_min_xy_spread_m": 0.020,
    "conditioning_support_radius_m": 0.40,
    "floor_snap_halfwidth_m": 0.035,
    "near_ground_cleanup_enabled": True,
    "near_ground_min_m": 0.045,
    "near_ground_max_m": 0.12,
    "structure_protection_radius_m": 0.18,
    "structure_above_min_m": 0.14,
    "structure_above_max_m": 0.90,
    "structure_min_points": 3,
    "low_object_neighbor_radius_m": 0.15,
    "low_object_min_other_neighbors": 3,
    "fragment_cleanup_enabled": False,
    "fragment_min_height_m": 0.10,
    "fragment_connection_radius_m": 0.15,
    "fragment_max_points": 20,
    "fragment_max_xy_extent_m": 0.25,
    "fragment_max_z_extent_m": 0.15,
    "fragment_planar_min_points": 8,
    "fragment_planar_min_xy_spread_m": 0.025,
    "fragment_planar_max_residual_m": 0.012,
    "fragment_planar_max_tilt_deg": 10.0,
    "workers": 2,
    "maximum_points": 5_000_000,
}


def _config(supplied):
    cfg = dict(DEFAULTS)
    unknown = set(supplied or {}) - set(cfg)
    if unknown:
        raise ValueError(f"Unknown flat-floor parameters: {sorted(unknown)}")
    cfg.update(supplied or {})
    positive = (
        "anchor_radius_m", "anchor_mode_bin_m", "anchor_mode_halfwidth_m",
        "anchor_residual_m", "anchor_max_tilt_deg", "anchor_min_xy_spread_m",
        "field_max_distance_m", "field_idw_softening_m", "field_max_disagreement_m",
        "support_cell_m", "support_halfwidth_m", "support_min_xy_spread_m",
        "conditioning_support_radius_m", "floor_snap_halfwidth_m", "near_ground_min_m",
        "near_ground_max_m", "structure_protection_radius_m",
        "structure_above_min_m", "structure_above_max_m", "low_object_neighbor_radius_m",
        "fragment_min_height_m", "fragment_connection_radius_m", "fragment_max_xy_extent_m",
        "fragment_max_z_extent_m", "fragment_planar_min_xy_spread_m",
        "fragment_planar_max_residual_m", "fragment_planar_max_tilt_deg",
    )
    for name in positive:
        if isinstance(cfg[name], bool) or not np.isfinite(cfg[name]) or cfg[name] <= 0:
            raise ValueError(f"{name} must be finite and positive")
    for name in ("anchor_min_points", "field_neighbors", "support_min_points",
                 "structure_min_points", "low_object_min_other_neighbors", "workers", "maximum_points",
                 "fragment_max_points", "fragment_planar_min_points"):
        value = cfg[name]
        if isinstance(value, bool) or not np.isfinite(value) or int(value) != value or value < 1:
            raise ValueError(f"{name} must be a positive integer")
        cfg[name] = int(value)
    for name in ("anchor_offset_min_m", "anchor_offset_max_m", "reference_z_m", "planning_min_relative_z_m"):
        if isinstance(cfg[name], bool) or not np.isfinite(cfg[name]):
            raise ValueError(f"{name} must be finite")
    if cfg['anchor_offset_max_m'] <= cfg['anchor_offset_min_m']:
        raise ValueError("anchor offset interval is empty")
    if cfg['planning_min_relative_z_m'] >= 0:
        raise ValueError("planning_min_relative_z_m must be negative to preserve observed floor")
    if cfg['anchor_max_tilt_deg'] >= 45:
        raise ValueError("Single-floor anchor tilt must be less than 45 degrees")
    if cfg['near_ground_max_m'] <= cfg['near_ground_min_m']:
        raise ValueError("near-ground interval is empty")
    if cfg['structure_above_max_m'] <= cfg['structure_above_min_m']:
        raise ValueError("structure interval is empty")
    if cfg['structure_above_min_m'] <= cfg['near_ground_max_m']:
        raise ValueError("structure_above_min_m must exceed near_ground_max_m; candidates cannot protect themselves")
    if cfg['floor_snap_halfwidth_m'] > cfg['support_halfwidth_m']:
        raise ValueError("floor snap width must not exceed observed support width")
    if cfg['workers'] > 8 or cfg['field_neighbors'] > 32:
        raise ValueError("Bounded CPU budget: workers<=8, field_neighbors<=32")
    if not isinstance(cfg['near_ground_cleanup_enabled'], bool):
        raise ValueError("near_ground_cleanup_enabled must be boolean")
    if not isinstance(cfg['fragment_cleanup_enabled'], bool):
        raise ValueError("fragment_cleanup_enabled must be boolean")
    if cfg['fragment_cleanup_enabled'] and not cfg['near_ground_cleanup_enabled']:
        raise ValueError("fragment cleanup requires near-ground cleanup")
    if cfg['fragment_cleanup_enabled'] and not cfg['near_ground_min_m'] <= cfg['fragment_min_height_m'] < cfg['near_ground_max_m']:
        raise ValueError("fragment_min_height_m must be inside the near-ground interval")
    if cfg['fragment_planar_max_tilt_deg'] >= 45:
        raise ValueError("fragment_planar_max_tilt_deg must be below 45")
    return cfg


def _points(value, name):
    result = np.asarray(value, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3 or len(result) == 0:
        raise ValueError(f"{name} must be a nonempty Nx3 array")
    if not np.isfinite(result).all():
        raise ValueError(f"{name} must contain only finite coordinates")
    return result


def _anchors(points, trajectory, cfg):
    tree = cKDTree(points[:, :2])
    bins = np.arange(cfg['anchor_offset_min_m'],
                     cfg['anchor_offset_max_m'] + cfg['anchor_mode_bin_m'] * .5,
                     cfg['anchor_mode_bin_m'])
    records = []
    failures = {'insufficient_points': 0, 'degenerate_xy': 0, 'tilted_or_noisy': 0}
    for index, pose in enumerate(trajectory):
        ids = np.asarray(tree.query_ball_point(pose[:2], cfg['anchor_radius_m']), dtype=int)
        local = points[ids]
        offset = local[:, 2] - pose[2]
        hist, _ = np.histogram(offset, bins)
        if not len(hist) or hist.max(initial=0) == 0:
            failures['insufficient_points'] += 1
            continue
        peak = int(hist.argmax())
        mode = (bins[peak] + bins[peak + 1]) / 2
        chosen = (np.abs(offset - mode) <= cfg['anchor_mode_halfwidth_m'])
        chosen &= (offset >= cfg['anchor_offset_min_m']) & (offset <= cfg['anchor_offset_max_m'])
        patch = local[chosen]
        if len(patch) < cfg['anchor_min_points']:
            failures['insufficient_points'] += 1
            continue
        design = np.column_stack((patch[:, :2] - pose[:2], np.ones(len(patch))))
        keep = np.ones(len(patch), dtype=bool)
        for _ in range(3):
            coef, *_ = np.linalg.lstsq(design[keep], patch[keep, 2], rcond=None)
            residual = patch[:, 2] - design @ coef
            new_keep = np.abs(residual - np.median(residual[keep])) <= cfg['anchor_residual_m']
            if new_keep.sum() < cfg['anchor_min_points']:
                break
            keep = new_keep
        scatter = np.cov(patch[keep, :2], rowvar=False)
        spread = float(np.sqrt(max(0, np.linalg.eigvalsh(scatter)[0])))
        if spread < cfg['anchor_min_xy_spread_m']:
            failures['degenerate_xy'] += 1
            continue
        tilt = float(np.degrees(np.arctan(np.linalg.norm(coef[:2]))))
        residual = patch[keep, 2] - design[keep] @ coef
        if tilt > cfg['anchor_max_tilt_deg'] or np.quantile(np.abs(residual), .9) > cfg['anchor_residual_m']:
            failures['tilted_or_noisy'] += 1
            continue
        records.append((index, *pose[:2], float(coef[2]), *coef[:2], int(keep.sum()),
                        tilt, float(np.sqrt(np.mean(residual ** 2)))))
    if len(records) < 3:
        raise ValueError(f"Insufficient reliable floor anchors ({len(records)}); refusing to invent a floor")
    values = np.asarray(records, dtype=np.float64)
    return {
        'pose_indices': values[:, 0].astype(int), 'xy': values[:, 1:3],
        'z': values[:, 3], 'slopes': values[:, 4:6], 'point_counts': values[:, 6].astype(int),
        'tilt_deg': values[:, 7], 'rms_m': values[:, 8], 'failures': failures,
    }


def _field(xy, anchors, cfg):
    tree = cKDTree(anchors['xy'])
    field = np.full(len(xy), np.nan)
    k = min(cfg['field_neighbors'], len(anchors['xy']))
    for start in range(0, len(xy), 100_000):
        query = xy[start:start + 100_000]
        distance, index = tree.query(query, k=k, workers=cfg['workers'])
        if k == 1:
            distance, index = distance[:, None], index[:, None]
        valid = distance <= cfg['field_max_distance_m']
        prediction = anchors['z'][index] + np.einsum(
            'nki,nki->nk', query[:, None, :] - anchors['xy'][index], anchors['slopes'][index])
        weight = np.where(valid, 1 / (distance + cfg['field_idw_softening_m']) ** 2, 0)
        weight_sum = weight.sum(axis=1)
        maximum = np.max(np.where(valid, prediction, -np.inf), axis=1)
        minimum = np.min(np.where(valid, prediction, np.inf), axis=1)
        reliable = (weight_sum > 0) & ((maximum - minimum) <= cfg['field_max_disagreement_m'])
        target = field[start:start + len(query)]
        target[reliable] = (prediction * weight).sum(axis=1)[reliable] / weight_sum[reliable]
    return field


def _observed_support(points, field, cfg):
    """Require real, 2D-spread floor samples, never create floor points."""
    origin = np.min(points[:, :2], axis=0)
    cells = np.floor((points[:, :2] - origin) / cfg['support_cell_m']).astype(np.int64)
    shape = cells.max(axis=0) + 1
    if int(shape[0]) * int(shape[1]) > 25_000_000:
        raise ValueError("Support grid exceeds 25 million cells; provide an explicit ROI")
    keys = cells[:, 0] * shape[1] + cells[:, 1]
    low = np.isfinite(field) & (np.abs(points[:, 2] - field) <= cfg['support_halfwidth_m'])
    ids = np.flatnonzero(low)
    size = int(np.prod(shape))
    count = np.bincount(keys[ids], minlength=size)
    # Moments relative to each cell reduce cancellation on large map coordinates.
    local = (points[ids, :2] - origin) - cells[ids] * cfg['support_cell_m']
    denominator = np.maximum(count, 1)
    sx = np.bincount(keys[ids], weights=local[:, 0], minlength=size) / denominator
    sy = np.bincount(keys[ids], weights=local[:, 1], minlength=size) / denominator
    vx = np.bincount(keys[ids], weights=local[:, 0] ** 2, minlength=size) / denominator - sx * sx
    vy = np.bincount(keys[ids], weights=local[:, 1] ** 2, minlength=size) / denominator - sy * sy
    vxy = np.bincount(keys[ids], weights=local[:, 0] * local[:, 1], minlength=size) / denominator - sx * sy
    smaller = .5 * (vx + vy - np.sqrt(np.maximum(0, (vx - vy) ** 2 + 4 * vxy * vxy)))
    reliable = (count >= cfg['support_min_points']) & (smaller >= cfg['support_min_xy_spread_m'] ** 2)
    support_points = low & reliable[keys]
    if not support_points.any():
        raise ValueError("No observed 2D floor support; refusing to condition an unknown surface")
    support_tree = cKDTree(points[support_points, :2])
    distance, _ = support_tree.query(points[:, :2], workers=cfg['workers'])
    domain = np.isfinite(field) & (distance <= cfg['conditioning_support_radius_m'])
    return support_points, reliable[keys], domain, int(reliable.sum())


def _fragment_cleanup(points, relative, candidates, upper_protected, cfg):
    """Small floating components, with measured-floor and structure gates upstream.

    Excluding the floor residual skirt from connectivity avoids joining several
    isolated elevated returns through harmless near-zero floor scatter. Large,
    long, tall, planar or upward-connected objects remain protected. This is an
    explicit scene prior, not a semantic proof that a small object is harmless.
    """
    remove = np.zeros(len(points), dtype=bool)
    stats = {'fragment_components': 0, 'removed_fragment_components': 0,
             'fragment_components_protected_by_structure': 0,
             'fragment_components_protected_by_size': 0,
             'fragment_components_protected_by_plane': 0}
    if not cfg['fragment_cleanup_enabled']:
        return remove, stats
    ids = np.flatnonzero(candidates & (relative >= cfg['fragment_min_height_m']))
    if not len(ids):
        return remove, stats
    xyz = np.column_stack((points[ids, :2], relative[ids]))
    tree = cKDTree(xyz)
    radius = cfg['fragment_connection_radius_m']
    # Refuse an unexpectedly dense graph before query_pairs allocates edges.
    neighbors = tree.query_ball_point(xyz, radius, return_length=True, workers=cfg['workers'])
    if int(neighbors.sum()) > 4_000_000:
        raise ValueError("Fragment connectivity exceeds four million directed neighbor links")
    edges = tree.query_pairs(radius, output_type='ndarray')
    graph = coo_matrix((np.ones(len(edges), dtype=np.uint8), (edges[:, 0], edges[:, 1])),
                       shape=(len(ids), len(ids))).tocsr()
    count, labels = connected_components(graph, directed=False)
    stats['fragment_components'] = int(count)
    order = np.argsort(labels, kind='stable')
    groups = np.split(order, np.flatnonzero(np.diff(labels[order])) + 1)
    for group in groups:
        original = ids[group]
        patch = xyz[group]
        span = np.ptp(patch, axis=0)
        if upper_protected[original].any():
            stats['fragment_components_protected_by_structure'] += 1
            continue
        if (len(group) > cfg['fragment_max_points'] or max(span[:2]) > cfg['fragment_max_xy_extent_m']
                or span[2] > cfg['fragment_max_z_extent_m']):
            stats['fragment_components_protected_by_size'] += 1
            continue
        if len(group) >= cfg['fragment_planar_min_points']:
            xy = patch[:, :2] - np.mean(patch[:, :2], axis=0)
            spread = np.sqrt(max(0, np.linalg.eigvalsh(np.cov(xy, rowvar=False))[0]))
            if spread >= cfg['fragment_planar_min_xy_spread_m']:
                design = np.column_stack((xy, np.ones(len(group))))
                coef, *_ = np.linalg.lstsq(design, patch[:, 2], rcond=None)
                residual = np.sqrt(np.mean((patch[:, 2] - design @ coef) ** 2))
                tilt = np.degrees(np.arctan(np.linalg.norm(coef[:2])))
                if residual <= cfg['fragment_planar_max_residual_m'] and tilt <= cfg['fragment_planar_max_tilt_deg']:
                    stats['fragment_components_protected_by_plane'] += 1
                    continue
        remove[original] = True
        stats['removed_fragment_components'] += 1
    return remove, stats


def _cleanup(points, field, support_cell, domain, cfg):
    relative = points[:, 2] - field
    candidates = (domain & support_cell & (relative >= cfg['near_ground_min_m'])
                  & (relative <= cfg['near_ground_max_m'])
                  & (relative > cfg['floor_snap_halfwidth_m']))
    remove = np.zeros(len(points), dtype=bool)
    protected = np.zeros(len(points), dtype=bool)
    upper_protected = np.zeros(len(points), dtype=bool)
    fragment_removed, fragment_stats = _fragment_cleanup(points, relative, np.zeros(len(points), dtype=bool),
                                                         upper_protected, cfg)
    ids = np.flatnonzero(candidates)
    if not len(ids) or not cfg['near_ground_cleanup_enabled']:
        return remove, candidates, protected, fragment_removed, fragment_stats
    # Coherent low objects are obstacles even without a tall extension. This
    # intentionally also keeps dense human feet: static geometry cannot tell.
    # Real spatial support, not column density: returns at substantially
    # different heights must not mutually protect one another in XY alone.
    tree = cKDTree(points[ids])
    count = tree.query_ball_point(points[ids], cfg['low_object_neighbor_radius_m'],
                                  return_length=True, workers=cfg['workers']) - 1
    protected[ids[count >= cfg['low_object_min_other_neighbors']]] = True
    above = (domain & (relative >= cfg['structure_above_min_m'])
             & (relative <= cfg['structure_above_max_m']))
    if above.any():
        upper_tree = cKDTree(points[above, :2])
        support = upper_tree.query_ball_point(points[ids, :2], cfg['structure_protection_radius_m'],
                                              return_length=True, workers=cfg['workers'])
        upper_protected[ids[support >= cfg['structure_min_points']]] = True
        protected |= upper_protected
    remove = candidates & ~protected
    fragment_removed, fragment_stats = _fragment_cleanup(points, relative, candidates, upper_protected, cfg)
    # The fragment mask is a subset of the total removal mask, not necessarily
    # all additional removals: it includes singleton fragments already sparse.
    remove |= fragment_removed
    protected &= ~remove
    return remove, candidates, protected, fragment_removed, fragment_stats


def condition_flat_floor(xyz, trajectory_xyz, config=None):
    """Return aligned arrays in original row order, without changing input arrays.

    ``floor_z`` estimates the original warped floor, NaN outside the reliable
    conditioning domain. ``floor_support_mask`` denotes observed floor points.
    ``conditioning_mask`` also includes measured structures near this support;
    it never means that a whole column is traversable. ``planning_mask`` is a
    recommended *point subset* for this explicit single-floor derived map.
    ``keep_mask`` applies cleanup only; outside-domain original points stay kept
    and unchanged. Caller must persist the mask/provenance and not mislabel TFs.
    """
    cfg = _config(config)
    points = _points(xyz, 'xyz')
    trajectory = _points(trajectory_xyz, 'trajectory_xyz')
    if len(points) > cfg['maximum_points']:
        raise ValueError("Source exceeds configured maximum_points")
    anchors = _anchors(points, trajectory, cfg)
    field = _field(points[:, :2], anchors, cfg)
    floor_points, support_cell, domain, supported_cells = _observed_support(points, field, cfg)
    removed, candidates, protected, fragment_removed, fragment_stats = _cleanup(points, field, support_cell, domain, cfg)
    result = points.copy()
    shift = cfg['reference_z_m'] - field
    result[domain, 2] += shift[domain]
    snapped = floor_points & domain & (np.abs(points[:, 2] - field) <= cfg['floor_snap_halfwidth_m'])
    result[snapped, 2] = cfg['reference_z_m']
    field[~domain] = np.nan
    keep = ~removed
    below_floor = domain & ((points[:, 2] - field) < cfg['planning_min_relative_z_m'])
    planning = keep & domain & ~below_floor
    statistics = {
        'source_points': len(points), 'trajectory_poses': len(trajectory),
        'reliable_anchors': len(anchors['z']), 'observed_support_cells': supported_cells,
        'floor_support_points': int(floor_points.sum()), 'conditioned_points': int(domain.sum()),
        'unchanged_outside_domain_points': int((~domain).sum()), 'snapped_floor_points': int(snapped.sum()),
        'near_ground_candidates': int(candidates.sum()), 'near_ground_protected_points': int(protected.sum()),
        'removed_near_ground_points': int(removed.sum()), 'planning_output_points': int(planning.sum()),
        'removed_fragment_points': int(fragment_removed.sum()),
        **fragment_stats,
        'excluded_below_floor_points': int(below_floor.sum()),
        'reference_z_m': cfg['reference_z_m'],
        'anchor_floor_z_percentiles': np.percentile(anchors['z'], [0, 1, 50, 99, 100]).tolist(),
        'applied_z_shift_percentiles': np.percentile(shift[domain], [0, 1, 50, 99, 100]).tolist(),
        'original_xy_unchanged': bool(np.array_equal(points[:, :2], result[:, :2])),
        'no_synthetic_points': True,
        'near_ground_semantics': 'Only sparse unsupported returns; coherent low obstacles and dense feet retained',
        'map_semantics': 'Scene-specific non-rigid Z-conditioned planning derivative; not a replacement localization map',
    }
    return {'xyz': result, 'keep_mask': keep, 'floor_z': field,
            'conditioning_mask': domain, 'floor_support_mask': floor_points,
            'floor_snap_mask': snapped, 'near_ground_candidate_mask': candidates,
            'near_ground_protected_mask': protected, 'planning_mask': planning,
            'removed_fragment_mask': fragment_removed,
            'excluded_below_floor_mask': below_floor,
            'anchors': anchors, 'statistics': statistics, 'config': cfg}
