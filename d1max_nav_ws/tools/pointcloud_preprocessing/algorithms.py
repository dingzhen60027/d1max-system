"""Index-only denoising with local measured surface/line protection.

No semantic person classification, coordinate smoothing, ground flattening,
hole filling, synthetic surfaces, or navigation clearance changes happen here.
"""
import numpy as np
from scipy.spatial import cKDTree


def protect_structures(xyz, tree, candidates, config, workers=4):
    """Protect a candidate only if the QUERY itself is supported by a surface.

    A flat neighborhood below an isolated high point does not protect that high
    point. Fit a trimmed local plane/line excluding the query, check residual,
    finite spatial support and that the point does not extrapolate past it.
    Orientation is arbitrary: floors, walls, slopes and thin edges use the same
    local geometry, without forcing any global floor height.
    """
    ids = np.flatnonzero(candidates)
    planar, linear = np.zeros(len(xyz), bool), np.zeros(len(xyz), bool)
    if not len(ids) or not config['enabled']:
        return planar, linear
    k = min(int(config['neighbors']) + 1, len(xyz))
    distances, neighbors = tree.query(xyz[ids], k=k, workers=workers)
    if k == 1:
        return planar, linear
    radius = config['support_radius_m']
    for query_id, distances_row, neighbors_row in zip(ids, distances, neighbors):
        nearby = neighbors_row[(neighbors_row != query_id) & (distances_row <= radius)]
        if len(nearby) < config['minimum_neighbors']:
            continue
        sample = xyz[nearby]
        fit = sample
        # Use actual neighbors only. The query never influences its own fit.
        for _ in range(3):
            center = np.median(fit, axis=0)
            centered = fit - center
            values, vectors = np.linalg.eigh(centered.T @ centered / len(fit))
            residual = np.abs((sample - center) @ vectors[:, 0])
            cutoff = max(config['plane_fit_tolerance_m'],
                         float(np.quantile(residual, config['trim_fraction'])))
            fit = sample[residual <= cutoff]
            if len(fit) < config['minimum_neighbors']:
                break
        if len(fit) < config['minimum_neighbors']:
            continue
        center = np.mean(fit, axis=0)
        centered = fit - center
        values, vectors = np.linalg.eigh(centered.T @ centered / len(fit))
        values = np.maximum(values, 0.)
        total = max(float(values.sum()), 1e-15)
        offset = xyz[query_id] - center
        local = offset @ vectors
        support = centered @ vectors
        # Do not extrapolate beyond local support by more than one small gap.
        allowance = config['edge_allowance_m'] + 1e-9
        within = np.all(local[1:] >= support[:, 1:].min(axis=0) - allowance) and np.all(
            local[1:] <= support[:, 1:].max(axis=0) + allowance)
        planar[query_id] = bool(
            values[0] / total <= config['maximum_surface_variation']
            and values[1] >= config['minimum_plane_spread_m'] ** 2
            and abs(local[0]) <= config['query_to_plane_m'] and within)
        # A narrow edge/rail/pole is protected as a line instead of discarded
        # merely because it does not form a broad two-dimensional patch.
        along = support[:, 2]
        linear[query_id] = bool(
            values[2] / total >= config['minimum_linearity_fraction']
            and np.ptp(along) >= config['minimum_line_span_m']
            and np.linalg.norm(local[:2]) <= config['query_to_line_m']
            and along.min() - allowance <= local[2] <= along.max() + allowance)
    return planar, linear


def filter_indices(xyz, modules, workers=4, progress=None):
    xyz = np.asarray(xyz, dtype=float)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError('Expected Nx3 point coordinates')
    finite = np.isfinite(xyz).all(axis=1)
    original_ids = np.flatnonzero(finite)
    points = xyz[finite]
    if not len(points):
        raise ValueError('No finite points')
    settings = {module['type']: module for module in modules}
    radius_cfg = settings['radius_outlier']
    sor_cfg = settings['statistical_outlier']
    protect_cfg = {**settings['structure_support']['parameters'],
                   'enabled': settings['structure_support']['enabled']}
    tree = cKDTree(points)
    if progress:
        progress('density_candidates')
    radius_counts = tree.query_ball_point(points, radius_cfg['parameters']['radius_m'],
                                         return_length=True, workers=workers) - 1
    # Counts EXCLUDE the query point. No iterative erosion/cascading deletions.
    radius_bad = (radius_counts < radius_cfg['parameters']['minimum_other_neighbors']) & radius_cfg['enabled']
    k = min(sor_cfg['parameters']['neighbors'], len(points) - 1)
    if k:
        distances, _ = tree.query(points, k=k+1, workers=workers)
        mean_distance = distances[:, 1:].mean(axis=1)
        cutoff = float(mean_distance.mean() + sor_cfg['parameters']['std_ratio'] * mean_distance.std())
        statistical_bad = (mean_distance > cutoff) & sor_cfg['enabled']
    else:
        mean_distance, cutoff = np.zeros(len(points)), 0.
        statistical_bad = np.zeros(len(points), bool)
    proposals = radius_bad | statistical_bad
    if progress:
        progress('measured_structure_protection')
    planar, linear = protect_structures(points, tree, proposals, protect_cfg, workers)
    protected = planar | linear
    remove_radius = radius_bad & ~protected
    remove_statistical = statistical_bad & ~protected & ~remove_radius
    keep_local = ~(remove_radius | remove_statistical)
    # Stable order and original record indices preserve ALL binary fields.
    keep = original_ids[keep_local]
    reason = np.zeros(len(xyz), dtype=np.uint8)
    reason[~finite] = 1
    reason[original_ids[remove_radius]] = 2
    reason[original_ids[remove_statistical]] = 3
    stages = [
        {'stage': 'source', 'type': 'input', 'label': '原始输入', 'points': len(xyz)},
        {'stage': 'finite', 'type': 'finite_xyz', 'label': '有效坐标', 'points': len(points)},
        {'stage': 'radius', 'type': 'radius_outlier', 'label': '半径离群清理（含结构保护）',
         'points': int(len(points)-remove_radius.sum()), 'proposed': int(radius_bad.sum()),
         'protected': int((radius_bad & protected).sum()), 'removed': int(remove_radius.sum())},
        {'stage': 'statistical', 'type': 'statistical_outlier', 'label': '统计离群清理（含结构保护）',
         'points': int(keep_local.sum()), 'proposed': int(statistical_bad.sum()),
         'protected': int((statistical_bad & protected).sum()), 'removed': int(remove_statistical.sum())},
    ]
    details = {'statistical_mean_distance_threshold_m': cutoff,
               'radius_count_excludes_self': True, 'fixed_source_neighborhoods': True,
               'candidate_union': int(proposals.sum()),
               'plane_supported_candidates': int(planar.sum()),
               'line_supported_candidates': int(linear.sum()),
               'protected_candidate_union': int(protected.sum()),
               'removed_reason_codes': {'1': 'nonfinite_xyz', '2': 'radius_unsupported',
                                        '3': 'statistical_unsupported'},
               'stage_counts': stages}
    return {'kept_indices': keep, 'removed_indices': np.flatnonzero(reason),
            'removed_reasons': reason[reason != 0],
            'protected_indices': original_ids[protected], 'statistics': details}
