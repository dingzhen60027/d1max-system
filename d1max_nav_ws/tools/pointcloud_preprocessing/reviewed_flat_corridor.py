"""Explicit, source-bound flat-floor prior for a reviewed static-map region.

This is not an obstacle classifier. A human-confirmed clear-floor polygon may
override only the retained *low-object* interpretation of a bounded duplicate
height layer. Walls, stairs, upper structures and unobserved lower ground stay
untouched. Corrected upper returns never become original measured support.
"""
import numpy as np
from scipy.spatial import cKDTree

SCHEMA = 'd1max.reviewed_flat_corridors/v1'


def polygon_mask(xy, vertices):
    polygon = np.asarray(vertices, dtype=float)
    if (polygon.ndim != 2 or polygon.shape[1] != 2 or not 3 <= len(polygon) <= 32
            or not np.isfinite(polygon).all()):
        raise ValueError('Reviewed region requires a finite convex XY polygon')
    edges = np.roll(polygon, -1, axis=0) - polygon
    turns = np.cross(edges, np.roll(edges, -1, axis=0))
    if not ((turns > 1e-9).all() or (turns < -1e-9).all()):
        raise ValueError('Reviewed region must be strictly convex without duplicate vertices')
    relative = np.asarray(xy, dtype=float)[:, None, :] - polygon
    cross = edges[None, :, 0] * relative[:, :, 1] - edges[None, :, 1] * relative[:, :, 0]
    return (cross >= -1e-9).all(axis=1) | (cross <= 1e-9).all(axis=1)


def validate(review, source_sha256):
    required = {'schema', 'source_map_sha256', 'review', 'regions', 'stair_exclusion_xy'}
    if not isinstance(review, dict) or set(review) != required or review['schema'] != SCHEMA:
        raise ValueError('Explicit reviewed-flat-corridor configuration required')
    digest = review['source_map_sha256']
    if (not isinstance(digest, str) or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest)
            or digest != source_sha256):
        raise ValueError('Reviewed corridor original source hash mismatch')
    confirmation = review['review']
    if (not isinstance(confirmation, dict) or confirmation.get('confirmed_clear_flat_floor') is not True
            or not confirmation.get('statement') or confirmation.get('scope') != 'static_planning_map_only'):
        raise ValueError('Human clear-flat-floor confirmation and static-only scope required')
    if not isinstance(review['regions'], list) or not review['regions']:
        raise ValueError('An explicit reviewed region is required')
    for region in review['regions']:
        if set(region) != {'id', 'floor_id', 'polygon_xy', 'residual_min_m', 'residual_max_m'}:
            raise ValueError('Unknown reviewed region parameters')
        if not region['id'] or not region['floor_id']:
            raise ValueError('Reviewed region and floor identities required')
        polygon_mask(np.empty((0, 2)), region['polygon_xy'])
        lo, hi = region['residual_min_m'], region['residual_max_m']
        if any(isinstance(v, bool) for v in (lo, hi)) or not np.isfinite([lo, hi]).all() or not 0 < lo < hi:
            raise ValueError('Reviewed residual interval must be finite, positive and bounded')
    if not isinstance(review['stair_exclusion_xy'], list) or not review['stair_exclusion_xy']:
        raise ValueError('Explicit stair exclusion required')
    for polygon in review['stair_exclusion_xy']:
        polygon_mask(np.empty((0, 2)), polygon)
    return review


def apply(result, original_xyz, *, floor_id, review, wall_protected):
    points = np.asarray(original_xyz, dtype=float)
    cfg = result['config']
    wall = np.asarray(wall_protected)
    if points.shape != result['xyz'].shape or wall.shape != (len(points),) or wall.dtype.kind != 'b':
        raise ValueError('Reviewed prior masks must match original records')
    field = result['floor_z']
    relative = points[:, 2] - field
    domain = result['conditioning_mask'] & result['keep_mask'] & np.isfinite(field)
    stairs = np.zeros(len(points), dtype=bool)
    for polygon in review['stair_exclusion_xy']:
        stairs |= polygon_mask(points[:, :2], polygon)
    upper = (np.isfinite(field) & (relative >= cfg['structure_above_min_m'])
             & (relative <= cfg['structure_above_max_m']))
    upper_protected = np.zeros(len(points), dtype=bool)
    if upper.any():
        counts = cKDTree(points[upper, :2]).query_ball_point(
            points[:, :2], cfg['structure_protection_radius_m'],
            return_length=True, workers=cfg['workers'])
        upper_protected = counts >= cfg['structure_min_points']
    # Only the original near-zero band is evidence of the *lower* surface.
    # A point merely snapped by this reviewed prior cannot seed another one.
    support = (result['floor_support_mask'] & domain & ~wall & ~stairs
               & (np.abs(relative) <= cfg['floor_snap_halfwidth_m']))
    seed_xy = points[support, :2]
    tree = cKDTree(seed_xy) if len(seed_xy) else None
    changed = np.zeros(len(points), dtype=bool)
    region_stats = []
    for region in review['regions']:
        if region['floor_id'] != floor_id:
            continue
        lo, hi = region['residual_min_m'], region['residual_max_m']
        if lo < cfg['floor_snap_halfwidth_m'] or hi > cfg['near_ground_max_m']:
            raise ValueError('Reviewed layer must remain inside existing low-return bounds')
        inside = polygon_mask(points[:, :2], region['polygon_xy'])
        candidates = (inside & domain & (relative >= lo) & (relative <= hi)
                      & (result['xyz'][:, 2] != cfg['reference_z_m']))
        eligible = candidates & ~wall & ~stairs & ~upper_protected & ~changed
        accepted = np.zeros(len(points), dtype=bool)
        if tree is not None:
            target = np.flatnonzero(eligible)
            radius = cfg['support_cell_m'] / np.sqrt(2.)
            for start in range(0, len(target), 4096):
                batch = target[start:start + 4096]
                counts = tree.query_ball_point(points[batch, :2], radius,
                                              return_length=True, workers=cfg['workers'])
                if int(counts.sum()) > 4_000_000:
                    raise ValueError('Reviewed support exceeds four million neighbor links')
                batch = batch[counts >= cfg['support_min_points']]
                neighborhoods = tree.query_ball_point(points[batch, :2], radius, workers=cfg['workers'])
                for index, neighbors in zip(batch, neighborhoods):
                    spread_sq = np.linalg.eigvalsh(np.cov(seed_xy[neighbors], rowvar=False, bias=True))[0]
                    accepted[index] = spread_sq >= cfg['support_min_xy_spread_m'] ** 2
        changed |= accepted
        region_stats.append(dict(id=region['id'], candidates=int(candidates.sum()),
            changed=int(accepted.sum()), protected_wall=int((candidates & wall).sum()),
            protected_stairs=int((candidates & stairs).sum()),
            protected_upper_structure=int((candidates & upper_protected).sum()),
            insufficient_lower_support=int((eligible & ~accepted).sum())))
    result['xyz'][changed, 2] = cfg['reference_z_m']
    # Keep original measured-support membership literal, even when a point is
    # geometrically corrected under the reviewed prior.
    result['reviewed_floor_prior_mask'] = changed
    result['floor_snap_mask'] |= changed
    result['statistics']['reviewed_flat_corridors'] = region_stats
    return changed
