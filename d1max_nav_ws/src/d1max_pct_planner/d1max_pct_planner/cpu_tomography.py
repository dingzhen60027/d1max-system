#!/usr/bin/env python3
"""Bounded NumPy/SciPy port of upstream PCT tomography, not CUDA execution.

Reference: pct_planner_vendor 35cd73fd, tomography/scripts/{tomography,
tomogram,kernels}.py. Stage equations, sentinel behavior, strict standability
test, radial max-cost inflation and layer selection follow those sources.
Unobserved ground/ceiling become NaN in the exported tensor. In particular,
an unobserved ceiling is NOT fabricated at a fixed height or labelled open sky.
"""
import argparse
import pickle
import time
from pathlib import Path

import numpy as np
import yaml
from scipy import ndimage


BACKEND = 'pct-official-equations-numpy-scipy-cpu'
SENTINEL = np.float32(1e6)


def round_half_away_from_zero(values):
    """CUDA/C round(), unlike np.rint's ties-to-even at exact half cells."""
    values = np.asarray(values)
    return np.copysign(np.floor(np.abs(values) + .5), values).astype(np.int64)


def _shifted_maximum(dst, src, dx, dy, weight):
    nx, ny = src.shape
    if abs(dx) >= nx or abs(dy) >= ny:
        return  # The official kernel skips every neighbor outside the map.
    dst_x = slice(max(0, -dx), min(nx, nx - dx))
    dst_y = slice(max(0, -dy), min(ny, ny - dy))
    src_x = slice(max(0, dx), min(nx, nx + dx))
    src_y = slice(max(0, dy), min(ny, ny + dy))
    np.maximum(dst[dst_x, dst_y], src[src_x, src_y] * weight,
               out=dst[dst_x, dst_y])


def _inflate(cost, resolution, safe_margin, inflation):
    radius = int((safe_margin + inflation) / resolution)
    result = np.zeros_like(cost)
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            distance = resolution * np.hypot(dx, dy)
            weight = np.float32(np.clip(
                1.0 - (distance - inflation) / (safe_margin + resolution), 0.0, 1.0))
            if weight > 0.0:
                _shifted_maximum(result, cost, dx, dy, weight)
    return result


def validate_config(cfg):
    """Reject malformed or unexpectedly huge work before array allocation."""
    for name in ('resolution', 'slice_dh'):
        if not np.isfinite(float(cfg[name])) or float(cfg[name]) <= 0:
            raise ValueError(f'{name} must be finite and positive')
    trav = cfg['traversability']
    for name in ('interval_min', 'interval_free', 'step_max', 'cost_barrier'):
        if not np.isfinite(float(trav[name])) or float(trav[name]) <= 0:
            raise ValueError(f'traversability.{name} must be finite and positive')
    for name in ('safe_margin', 'inflation'):
        if not np.isfinite(float(trav[name])) or float(trav[name]) < 0:
            raise ValueError(f'traversability.{name} must be finite and nonnegative')
    slope = float(trav['slope_max_rad'])
    if not 0 < slope < np.pi / 2 or not 0 <= float(trav['standable_ratio']) <= 1:
        raise ValueError('Invalid slope or standable ratio')
    if float(trav['interval_free']) < float(trav['interval_min']):
        raise ValueError('interval_free must not be lower than interval_min')
    kernel = int(trav['kernel_size'])
    if kernel != trav['kernel_size'] or not 1 <= kernel <= 31 or kernel % 2 != 1:
        raise ValueError('kernel_size must be odd and in [1, 31]')
    if float(trav['cost_barrier']) <= 20:
        raise ValueError('cost_barrier must exceed the native PCT rejection threshold 20')
    if int((float(trav['safe_margin']) + float(trav['inflation'])) / float(cfg['resolution'])) > 64:
        raise ValueError('Inflation kernel radius exceeds the bounded CPU budget (64 cells)')
    for key, default in [('max_total_cells', 25000000), ('max_axis_cells', 2000),
                         ('max_layers', 128), ('max_working_bytes', 2500000000)]:
        value = cfg.get('limits', {}).get(key, default)
        if isinstance(value, bool) or int(value) != value or value <= 0:
            raise ValueError(f'limits.{key} must be a positive integer')
    ground = cfg.get('ground_height', 'auto')
    if ground != 'auto' and not np.isfinite(float(ground)):
        raise ValueError('ground_height must be auto or finite')


def mapping_geometry(points, cfg):
    resolution, dh = float(cfg['resolution']), float(cfg['slice_dh'])
    pmin, pmax = np.min(points, axis=0), np.max(points, axis=0)
    ground = cfg.get('ground_height', 'auto')
    ground_h = float(pmin[2]) if ground == 'auto' else float(ground)
    if pmax[2] < ground_h:
        raise ValueError('ground_height is above every source point')
    # Same +4 border and map centre as tomography.py; no recentering of the PCD.
    shape = np.ceil((pmax[:2].astype(float) - pmin[:2]) / resolution).astype(int) + 4
    n_slices = max(1, int(np.ceil((float(pmax[2]) - ground_h) / dh)))
    total = int(np.prod(shape, dtype=np.int64)) * n_slices
    # Conservative peak including gradients, masks, tensor and export copies.
    estimated_bytes = total * 80 + len(points) * 64
    limits = cfg.get('limits', {})
    if (max(shape) > limits.get('max_axis_cells', 2000)
            or n_slices > limits.get('max_layers', 128)
            or total > limits.get('max_total_cells', 25000000)
            or estimated_bytes > limits.get('max_working_bytes', 2500000000)):
        raise ValueError(f'Tomogram exceeds resource limits: shape={[n_slices, *shape]}, '
                         f'cells={total}, estimated_working_bytes={estimated_bytes}; '
                         'configure an explicit ROI/resolution rather than allocating blindly')
    center = ((pmax[:2] + pmin[:2]) / np.float32(2)).astype(np.float32)
    return {'shape': tuple(map(int, shape)), 'n_slices': n_slices,
            'center': center, 'slice_h0': ground_h + dh, 'ground_height': ground_h,
            'total_cells': total, 'estimated_working_bytes': estimated_bytes}


def height_layers(points, geometry, resolution, slice_dh):
    dim_x, dim_y = geometry['shape']
    center = geometry['center']
    # Point insertion uses the CUDA kernel's round convention. The upstream
    # Python planner uses np.rint for queries, differing only at exact ties.
    indices = round_half_away_from_zero((points[:, :2] - center) / resolution)
    indices += np.array([dim_x // 2, dim_y // 2])
    inside = ((indices >= 0) & (indices < [dim_x, dim_y])).all(axis=1)
    ix, iy = indices[inside].T
    pz = points[inside, 2]
    shape = (geometry['n_slices'], dim_x, dim_y)
    ground = np.full(shape, -SENTINEL, dtype=np.float32)
    ceiling = np.full(shape, SENTINEL, dtype=np.float32)
    for layer in range(geometry['n_slices']):
        slice_z = np.float32(geometry['slice_h0'] + layer * slice_dh)
        below = pz <= slice_z
        np.maximum.at(ground[layer], (ix[below], iy[below]), pz[below])
        np.minimum.at(ceiling[layer], (ix[~below], iy[~below]), pz[~below])
    return ground, ceiling


def height_gradients(ground):
    grad_sq, grad_max = np.zeros_like(ground), np.zeros_like(ground)
    dx_sq = np.maximum((ground[:, 1:-1, :] - ground[:, :-2, :]) ** 2,
                       (ground[:, 1:-1, :] - ground[:, 2:, :]) ** 2)
    dy_sq = np.maximum((ground[:, :, 1:-1] - ground[:, :, :-2]) ** 2,
                       (ground[:, :, 1:-1] - ground[:, :, 2:]) ** 2)
    grad_sq[:, 1:-1, 1:-1] = dx_sq[:, :, 1:-1] + dy_sq[:, 1:-1, :]
    grad_max[:, 1:-1, 1:-1] = np.maximum(dx_sq[:, :, 1:-1], dy_sq[:, 1:-1, :])
    return grad_sq, grad_max


def traversal_cost(ground, ceiling, resolution, cfg):
    grad_sq, grad_max = height_gradients(ground)
    step_stand_sq = (1.2 * resolution * np.tan(float(cfg['slope_max_rad']))) ** 2
    step_cross_sq = float(cfg['step_max']) ** 2
    kernel = int(cfg['kernel_size'])
    standable_th = int(float(cfg['standable_ratio']) * kernel**2) - 1
    interval = ceiling - ground
    trav = np.maximum(np.float32(0), np.float32(20) * (float(cfg['interval_free']) - interval))
    flat = grad_sq <= step_stand_sq
    trav[flat] += 15 * grad_sq[flat] / step_stand_sq
    footprint = np.ones((kernel, kernel), dtype=np.int16)
    crossable = np.zeros_like(flat)
    for layer in range(len(ground)):
        # Kernel uses < for neighboring standable cells, but <= for the
        # current flat cell. Their difference at equality is intentional.
        neighbor_flat = ndimage.convolve((grad_sq[layer] < step_stand_sq).astype(np.int16),
                                        footprint, mode='constant', cval=0)
        crossable[layer] = ((~flat[layer]) & (grad_max[layer] <= step_cross_sq)
                            & (neighbor_flat >= standable_th))
    trav[crossable] += 20 * grad_max[crossable] / step_cross_sq
    trav[(interval < float(cfg['interval_min'])) | ((~flat) & (~crossable))] = float(cfg['cost_barrier'])
    return trav.astype(np.float32)


def select_layers(ground, inflated, barrier, simplify=True):
    """Exactly the upstream simplification loop, including its upper endpoint."""
    count = len(ground)
    if not simplify:
        return np.arange(count, dtype=np.int32)
    selected = [0]
    if count > 1:
        lower, middle = 0, 1
        diff_h = ground[1:] - ground[:-1]
        while middle < count - 2:
            unique = (((ground[middle] - ground[lower] > 0)
                       | (inflated[lower] > inflated[middle]))
                      & (diff_h[middle] > 0) & (inflated[middle] < barrier))
            if np.any(unique):
                selected.append(middle)
                lower = middle
            middle += 1
        selected.append(middle)
    return np.asarray(selected, dtype=np.int32)


def layer_statistics(ground, ceiling, cost, threshold=20.0):
    records = []
    for layer in range(len(ground)):
        support = np.isfinite(ground[layer]) & (ground[layer] > -SENTINEL)
        measured_above = np.isfinite(ceiling[layer]) & (ceiling[layer] < SENTINEL)
        valid = support & np.isfinite(cost[layer]) & (cost[layer] <= threshold)
        labels, count = ndimage.label(valid)
        sizes = np.bincount(labels.ravel())[1:]
        records.append({'layer': layer, 'measured_ground_cells': int(support.sum()),
                        'measured_ground_and_ceiling_cells': int((support & measured_above).sum()),
                        'unobserved_above_supported_cells': int((support & ~measured_above).sum()),
                        'traversable_cells': int(valid.sum()), 'components_4_connected': int(count),
                        'largest_component_cells': int(sizes.max()) if sizes.size else 0,
                        'cost_percentiles_on_support': np.percentile(cost[layer][support], [0, 25, 50, 75, 100]).tolist()
                        if support.any() else None})
    return records


def tomogram_from_points(points, cfg):
    """Pure bounded CPU stages; caller owns conservative preprocessing/export."""
    validate_config(cfg)
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError('Tomography requires a nonempty finite Nx3 point array')
    geometry = mapping_geometry(points, cfg)
    started = time.perf_counter()
    ground, ceiling = height_layers(points, geometry, float(cfg['resolution']), float(cfg['slice_dh']))
    map_time = time.perf_counter() - started
    started = time.perf_counter()
    trav = traversal_cost(ground, ceiling, float(cfg['resolution']), cfg['traversability'])
    raw_stats = layer_statistics(ground, ceiling, trav)
    traversability_time = time.perf_counter() - started
    started = time.perf_counter()
    inflated = np.empty_like(trav)
    for layer in range(len(ground)):
        inflated[layer] = _inflate(trav[layer], float(cfg['resolution']),
                                   float(cfg['traversability']['safe_margin']),
                                   float(cfg['traversability']['inflation']))
    del trav
    inflation_time = time.perf_counter() - started
    inflated_stats = layer_statistics(ground, ceiling, inflated)
    started = time.perf_counter()
    selected = select_layers(ground, inflated, float(cfg['traversability']['cost_barrier']),
                             bool(cfg.get('simplify_layers', True)))
    out_cost = inflated[selected]
    out_g = np.where(ground[selected] > -SENTINEL, ground[selected], np.nan)
    out_c = np.where(ceiling[selected] < SENTINEL, ceiling[selected], np.nan)
    gx, gy = np.zeros_like(out_cost), np.zeros_like(out_cost)
    gx[:, 1:-1, :] = out_cost[:, 2:, :] - out_cost[:, :-2, :]
    gy[:, :, 1:-1] = out_cost[:, :, 2:] - out_cost[:, :, :-2]
    data = np.stack((out_cost, gx, gy, out_g, out_c)).astype(np.float32)
    simplification_time = time.perf_counter() - started
    payload = {'data': data, 'resolution': float(cfg['resolution']), 'center': geometry['center'],
               'slice_h0': geometry['slice_h0'], 'slice_dh': float(cfg['slice_dh']),
               'backend': BACKEND, 'selected_source_layers': selected,
               'minimum_headroom_m': float(cfg['traversability']['interval_min']),
               'preferred_headroom_m': float(cfg['traversability']['interval_free']),
               'cost_threshold': 20.0, 'ground_semantics': 'measured_maximum_below_slice',
               'ceiling_semantics': 'measured_minimum_above_slice_or_nan_unobserved'}
    statistics = {'initial_shape': [geometry['n_slices'], *geometry['shape']],
                  'total_cells': geometry['total_cells'],
                  'estimated_working_bytes': geometry['estimated_working_bytes'],
                  'selected_source_layers': selected.tolist(),
                  'selected_slice_heights_m': (geometry['slice_h0'] + selected * float(cfg['slice_dh'])).tolist(),
                  'raw_traversability': raw_stats, 'inflated_traversability': inflated_stats,
                  'selected_traversability': [inflated_stats[i] for i in selected],
                  'seconds': {'height_layers': map_time, 'traversability_and_stats': traversability_time,
                              'inflation': inflation_time, 'simplification_and_export_tensor': simplification_time}}
    return payload, statistics


def _load_config(path):
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)['pct']


def build_tomogram(pcd_path, output_path, cfg):
    """Legacy CLI compatibility; reproducible builds use official_pipeline."""
    import open3d as o3d
    points = np.asarray(o3d.io.read_point_cloud(str(pcd_path)).points, dtype=np.float32)
    points = points[np.isfinite(points).all(axis=1)]
    if cfg.get('height_filter'):
        bounds = cfg['height_filter']
        points = points[(points[:, 2] >= float(bounds['min'])) & (points[:, 2] <= float(bounds['max']))]
    payload, _ = tomogram_from_points(points, cfg)
    payload['source_pcd'] = str(pcd_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('wb') as stream:
        pickle.dump(payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
    return points, payload


def export_preview(payload, output_path, traversable_only=False):
    import open3d as o3d
    data = np.asarray(payload['data'], dtype=np.float32)
    ground, cost = data[3], data[0]
    center, resolution = np.asarray(payload['center']), float(payload['resolution'])
    chunks, colors = [], []
    for layer in range(len(ground)):
        valid = np.isfinite(ground[layer])
        if traversable_only:
            valid &= np.isfinite(cost[layer]) & (cost[layer] <= float(payload.get('cost_threshold', 20)))
        x, y = np.where(valid)
        if not len(x):
            continue
        chunks.append(np.column_stack(((x-ground.shape[1]//2)*resolution+center[0],
                                       (y-ground.shape[2]//2)*resolution+center[1], ground[layer, x, y])))
        normalized = np.clip(cost[layer, x, y]/50, 0, 1)
        colors.append(np.column_stack((normalized, 1-normalized, np.full_like(normalized, .15))))
    cloud = o3d.geometry.PointCloud()
    if chunks:
        cloud.points = o3d.utility.Vector3dVector(np.concatenate(chunks))
        cloud.colors = o3d.utility.Vector3dVector(np.concatenate(colors))
    if not o3d.io.write_point_cloud(str(output_path), cloud, write_ascii=False):
        raise RuntimeError(f'Could not export preview {output_path}')
    return len(cloud.points)


def main():
    parser = argparse.ArgumentParser(description='Build PCT official-equation CPU tomography (not CUDA).')
    parser.add_argument('--pcd', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--preview', type=Path)
    args = parser.parse_args()
    started = time.perf_counter()
    points, payload = build_tomogram(args.pcd, args.output, _load_config(args.config))
    if args.preview:
        export_preview(payload, args.preview)
    print(f'PCT CPU tomogram: {args.output}; points={len(points)}, '
          f'shape={payload["data"].shape}, elapsed={time.perf_counter()-started:.3f}s')


if __name__ == '__main__':
    main()
