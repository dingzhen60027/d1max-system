"""Scene-specific cleanup of unsupported low returns over an observed flat floor.

This classifies geometric support, not semantic people, and is not a safety
certificate. Every removed return has measured floor beneath it. No point is
created, moved, or marked traversable here. Original obstacle records are kept by
the caller. Tall continuous walls/posts and coherent low horizontal surfaces are
protected independently; a few arbitrary floating points are not wall evidence.
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


DEFAULTS = {
    'enabled': False,
    'floor_reference_z_m': 0.0,
    'floor_halfwidth_m': 0.035,
    'support_cell_m': 0.15,
    'support_cell_min_points': 3,
    'support_mode': 'cell',
    'support_core_radius_m': 0.11,
    'support_core_min_points': 3,
    'support_radius_m': 0.22,
    'support_min_neighbors': 10,
    'candidate_min_height_m': 0.04,
    'candidate_max_height_m': 0.70,
    'structure_min_height_m': 0.04,
    'structure_max_height_m': 1.50,
    'normal_neighbors': 24,
    'normal_min_neighbors': 8,
    'normal_radius_m': 0.30,
    'normal_max_plane_rms_m': 0.018,
    'normal_min_second_spread_m': 0.040,
    'vertical_normal_abs_z_max': 0.30,
    'horizontal_normal_abs_z_min': 0.92,
    'horizontal_plane_min_inlier_ratio': 0.80,
    'vertical_column_cell_m': 0.15,
    'vertical_bin_m': 0.10,
    'vertical_column_max_height_m': 1.24,
    'vertical_min_occupied_bins': 6,
    'vertical_min_consecutive_bins': 5,
    'vertical_bottom_max_height_m': 0.20,
    'thin_post_min_points': 20,
    'thin_post_max_xy_spread_m': 0.035,
    'wall_protection_radius_m': 0.15,
    'low_object_protection_radius_m': 0.10,
    'low_object_top_tolerance_m': 0.035,
    'workers': 2,
    'maximum_points': 5_000_000,
    'normal_batch_points': 40_000,
    'maximum_grid_cells': 4_000_000,
}


def _config(supplied):
    cfg = dict(DEFAULTS)
    unknown = set(supplied or {}) - set(cfg)
    if unknown:
        raise ValueError(f'Unknown structural-obstacle parameters: {sorted(unknown)}')
    cfg.update(supplied or {})
    if not isinstance(cfg['enabled'], bool):
        raise ValueError('enabled must be boolean')
    if cfg['support_mode'] not in ('cell', 'metric_disk'):
        raise ValueError('support_mode must be cell or metric_disk')
    integer_keys = ('support_cell_min_points', 'support_core_min_points', 'support_min_neighbors', 'normal_neighbors',
                    'normal_min_neighbors', 'vertical_min_occupied_bins', 'vertical_min_consecutive_bins',
                    'thin_post_min_points', 'workers', 'maximum_points', 'normal_batch_points',
                    'maximum_grid_cells')
    for name in integer_keys:
        value = cfg[name]
        if isinstance(value, bool) or not np.isfinite(value) or int(value) != value or value < 1:
            raise ValueError(f'{name} must be a positive integer')
        cfg[name] = int(value)
    for name in set(cfg) - set(integer_keys) - {'enabled', 'floor_reference_z_m', 'support_mode'}:
        value = cfg[name]
        if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and positive')
    if isinstance(cfg['floor_reference_z_m'], bool) or not np.isfinite(cfg['floor_reference_z_m']):
        raise ValueError('floor_reference_z_m must be finite')
    if not cfg['floor_halfwidth_m'] < cfg['candidate_min_height_m'] < cfg['candidate_max_height_m']:
        raise ValueError('candidate interval must lie strictly above measured-floor band')
    if cfg['support_core_radius_m'] > cfg['support_radius_m']:
        raise ValueError('Core floor support radius must not exceed surrounding support radius')
    if not cfg['structure_min_height_m'] < cfg['structure_max_height_m']:
        raise ValueError('structure height interval is empty')
    if not 3 <= cfg['normal_min_neighbors'] <= cfg['normal_neighbors'] <= 64:
        raise ValueError('Require 3 <= normal_min_neighbors <= normal_neighbors <= 64')
    if cfg['workers'] > 8 or cfg['normal_batch_points'] > 100_000:
        raise ValueError('Bounded CPU/memory: workers<=8, normal_batch_points<=100000')
    if not 0 < cfg['vertical_normal_abs_z_max'] < cfg['horizontal_normal_abs_z_min'] < 1:
        raise ValueError('Vertical/horizontal normal thresholds must be ordered in (0,1)')
    if cfg['horizontal_plane_min_inlier_ratio'] > 1:
        raise ValueError('horizontal_plane_min_inlier_ratio must be in (0,1]')
    bins = int(np.floor(cfg['vertical_column_max_height_m'] / cfg['vertical_bin_m'])) + 1
    if bins > 63 or cfg['vertical_min_occupied_bins'] > bins or cfg['vertical_min_consecutive_bins'] > bins:
        raise ValueError('Vertical occupancy has invalid or excessive bin count')
    return cfg


def _grid(xy, cell_size, maximum_cells):
    origin = xy.min(axis=0)
    cell = np.floor((xy - origin) / cell_size).astype(np.int64)
    shape = cell.max(axis=0) + 1
    size = int(shape[0]) * int(shape[1])
    if size > maximum_cells:
        raise ValueError('Structural support grid exceeds maximum_grid_cells; provide a bounded ROI')
    keys = cell[:, 0] * shape[1] + cell[:, 1]
    return keys, size, origin, cell


def _local_planes(points, cfg):
    count = len(points)
    vertical, horizontal = np.zeros(count, bool), np.zeros(count, bool)
    if count < cfg['normal_min_neighbors']:
        return vertical, horizontal
    tree = cKDTree(points)
    k = min(cfg['normal_neighbors'], count)
    batch = cfg['normal_batch_points']
    for start in range(0, count, batch):
        query = points[start:start + batch]
        distance, ids = tree.query(query, k=k, workers=cfg['workers'])
        valid = distance <= cfg['normal_radius_m']
        weight = valid.astype(float)
        neighbors = weight.sum(axis=1)
        denominator = np.maximum(neighbors, 1)
        patch = points[ids]
        center = np.einsum('nk,nki->ni', weight, patch) / denominator[:, None]
        delta = patch - center[:, None, :]
        covariance = np.einsum('nk,nki,nkj->nij', weight, delta, delta) / denominator[:, None, None]
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        normal = eigenvectors[:, :, 0]
        query_residual = np.abs(np.einsum('ni,ni->n', query - center, normal))
        plane = ((neighbors >= cfg['normal_min_neighbors'])
                 & (np.sqrt(np.maximum(eigenvalues[:, 0], 0)) <= cfg['normal_max_plane_rms_m'])
                 & (np.sqrt(np.maximum(eigenvalues[:, 1], 0)) >= cfg['normal_min_second_spread_m'])
                 & (query_residual <= cfg['normal_max_plane_rms_m']))
        vertical[start:start + len(query)] = plane & (abs(normal[:, 2]) <= cfg['vertical_normal_abs_z_max'])
        horizontal[start:start + len(query)] = plane & (abs(normal[:, 2]) >= cfg['horizontal_normal_abs_z_min'])
        # A true box top must survive nearby side/base returns. Refit a
        # horizontal hypothesis using its same-height support (two times the
        # configured plane residual tolerance), not a mixed top+side covariance.
        same_height = valid & (abs(patch[:, :, 2] - query[:, None, 2]) <= 2 * cfg['normal_max_plane_rms_m'])
        hw = same_height.astype(float)
        hn = hw.sum(axis=1)
        hd = np.maximum(hn, 1)
        hc = np.einsum('nk,nki->ni', hw, patch) / hd[:, None]
        hdelta = patch - hc[:, None, :]
        hcov = np.einsum('nk,nki,nkj->nij', hw, hdelta, hdelta) / hd[:, None, None]
        he, hv = np.linalg.eigh(hcov)
        hnormal = hv[:, :, 0]
        hr = abs(np.einsum('ni,ni->n', query - hc, hnormal))
        horizontal[start:start + len(query)] |= (
            (hn >= cfg['normal_min_neighbors'])
            & (hn >= cfg['horizontal_plane_min_inlier_ratio'] * neighbors)
            & (np.sqrt(np.maximum(he[:, 0], 0)) <= cfg['normal_max_plane_rms_m'])
            & (np.sqrt(np.maximum(he[:, 1], 0)) >= cfg['normal_min_second_spread_m'])
            & (hr <= cfg['normal_max_plane_rms_m'])
            & (abs(hnormal[:, 2]) >= cfg['horizontal_normal_abs_z_min']))
    return vertical, horizontal


def _continuous_columns(points, relative_z, cfg):
    key, size, origin, cells = _grid(points[:, :2], cfg['vertical_column_cell_m'], cfg['maximum_grid_cells'])
    use = ((relative_z >= cfg['structure_min_height_m'])
           & (relative_z <= cfg['vertical_column_max_height_m']))
    count_bins = int(np.floor(cfg['vertical_column_max_height_m'] / cfg['vertical_bin_m'])) + 1
    bits = np.zeros(size, dtype=np.uint64)
    b = np.floor(relative_z[use] / cfg['vertical_bin_m']).astype(np.uint64)
    np.bitwise_or.at(bits, key[use], np.left_shift(np.uint64(1), b))
    occupancy = (bits[:, None] & np.left_shift(np.uint64(1), np.arange(count_bins, dtype=np.uint64))[None, :]) > 0
    kernel = np.ones(cfg['vertical_min_consecutive_bins'], dtype=np.int8)
    consecutive = ndimage.convolve1d(occupancy.astype(np.int8), kernel, axis=1, mode='constant') >= len(kernel)
    bottom_bin = max(1, int(np.ceil(cfg['vertical_bottom_max_height_m'] / cfg['vertical_bin_m'])))
    continuous = ((occupancy.sum(axis=1) >= cfg['vertical_min_occupied_bins'])
                  & consecutive.any(axis=1) & occupancy[:, :bottom_bin].any(axis=1))
    # Thin posts need not present a large planar patch, but must have dense,
    # continuous vertical occupancy from near the floor, not arbitrary 3 points.
    count = np.bincount(key[use], minlength=size)
    denom = np.maximum(count, 1)
    local = (points[use, :2] - origin) - cells[use] * cfg['vertical_column_cell_m']
    means = [np.bincount(key[use], weights=local[:, axis], minlength=size) / denom for axis in range(2)]
    variances = [np.bincount(key[use], weights=local[:, axis] ** 2, minlength=size) / denom - means[axis] ** 2 for axis in range(2)]
    thin_post = continuous & (count >= cfg['thin_post_min_points'])
    thin_post &= np.maximum(variances[0], variances[1]) <= cfg['thin_post_max_xy_spread_m'] ** 2
    return continuous[key], thin_post[key], int(continuous.sum()), int(thin_post.sum())


def filter_structural_obstacles(xyz, config=None):
    """Return same-length point masks without changing coordinates or row order.

    Input must already be a floor-conditioned planning derivative. Applying this
    fixed-floor contract to the original warped map is an error in caller setup.
    Returned support masks do NOT mean whole cells are traversable: remaining
    structures, missing floor and collisions still require normal PCT checks.
    """
    cfg = _config(config)
    points = np.asarray(xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError('xyz must be a nonempty finite Nx3 array')
    if len(points) > cfg['maximum_points']:
        raise ValueError('Input exceeds maximum_points')
    empty = np.zeros(len(points), dtype=bool)
    if not cfg['enabled']:
        return {'keep_mask': ~empty, 'removed_mask': empty, 'support_mask': empty.copy(),
                'candidate_mask': empty.copy(), 'wall_protected_mask': empty.copy(),
                'low_object_protected_mask': empty.copy(),
                'statistics': {'enabled': False, 'input_points': len(points), 'removed_points': 0}, 'config': cfg}
    relative = points[:, 2] - cfg['floor_reference_z_m']
    floor = abs(relative) <= cfg['floor_halfwidth_m']
    if floor.sum() < cfg['support_min_neighbors']:
        raise ValueError('Insufficient observed flat-floor points; refusing to invent floor support')
    floor_tree = cKDTree(points[floor, :2])
    local_floor_count = floor_tree.query_ball_point(points[:, :2], cfg['support_radius_m'],
                                                   return_length=True, workers=cfg['workers'])
    if cfg['support_mode'] == 'metric_disk':
        # A grid boundary must not split actual floor evidence into unrelated
        # bins. This small metric core is approximately the circumscribed
        # radius of the legacy 15 cm cell; the 22 cm density guard still applies.
        core_count = floor_tree.query_ball_point(points[:, :2], cfg['support_core_radius_m'],
                                                return_length=True, workers=cfg['workers'])
        core_supported = core_count >= cfg['support_core_min_points']
    else:
        key, size, unused_origin, unused_cells = _grid(points[:, :2], cfg['support_cell_m'], cfg['maximum_grid_cells'])
        floor_cell = np.bincount(key[floor], minlength=size)
        core_supported = floor_cell[key] >= cfg['support_cell_min_points']
    support = core_supported & (local_floor_count >= cfg['support_min_neighbors'])
    candidates = support & (relative > cfg['candidate_min_height_m']) & (relative < cfg['candidate_max_height_m'])
    upper = (relative >= cfg['structure_min_height_m']) & (relative <= cfg['structure_max_height_m'])
    upper_ids = np.flatnonzero(upper)
    vplane, hplane = _local_planes(points[upper], cfg)
    column, thin_post, column_count, post_count = _continuous_columns(points, relative, cfg)
    trusted = (vplane & column[upper_ids]) | thin_post[upper_ids]
    wall = np.zeros(len(points), dtype=bool)
    if trusted.any():
        wall = cKDTree(points[upper_ids[trusted], :2]).query(points[:, :2], workers=cfg['workers'])[0] <= cfg['wall_protection_radius_m']
    # A genuine low box top also protects nearby side/base returns below it.
    # High ceilings never provide this low-object evidence.
    top_ids = upper_ids[hplane & (relative[upper_ids] <= cfg['candidate_max_height_m'])]
    low_object = np.zeros(len(points), dtype=bool)
    if len(top_ids):
        distance, nearest = cKDTree(points[top_ids, :2]).query(points[:, :2], workers=cfg['workers'])
        low_object = ((distance <= cfg['low_object_protection_radius_m'])
                      & (relative <= relative[top_ids[nearest]] + cfg['low_object_top_tolerance_m']))
    removed = candidates & ~wall & ~low_object
    statistics = {
        'enabled': True, 'input_points': len(points), 'observed_floor_points': int(floor.sum()),
        'floor_supported_points': int(support.sum()), 'candidate_points': int(candidates.sum()),
        'floor_support_mode': cfg['support_mode'],
        'vertical_plane_points': int(vplane.sum()), 'trusted_structure_points': int(trusted.sum()),
        'continuous_vertical_columns': column_count, 'thin_post_columns': post_count,
        'low_object_top_points': len(top_ids), 'wall_protected_candidate_points': int((candidates & wall).sum()),
        'low_object_protected_candidate_points': int((candidates & low_object).sum()),
        'removed_points': int(removed.sum()), 'output_points': int((~removed).sum()),
        'removed_observed_floor_points': int((removed & floor).sum()), 'no_synthetic_points': True,
        'trajectory_clearing_used': False,
        'semantics': 'Unsupported low geometry over measured flat floor; not semantic human recognition or a live obstacle safety certificate',
    }
    return {'keep_mask': ~removed, 'removed_mask': removed, 'support_mask': support,
            'candidate_mask': candidates, 'wall_protected_mask': wall,
            'low_object_protected_mask': low_object, 'observed_floor_mask': floor,
            'statistics': statistics, 'config': cfg}
