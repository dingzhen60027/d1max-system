#!/usr/bin/env python3
import argparse
import pickle
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml
from scipy import ndimage


def _shifted_maximum(dst, src, dx, dy, weight):
    nx, ny = src.shape
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
            weight = np.clip(
                1.0 - (distance - inflation) / (safe_margin + resolution),
                0.0, 1.0)
            if weight > 0.0:
                _shifted_maximum(result, cost, dx, dy, weight)
    return result


def _load_config(path):
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)['pct']


def build_tomogram(pcd_path, output_path, cfg):
    cloud = o3d.io.read_point_cloud(str(pcd_path))
    points = np.asarray(cloud.points, dtype=np.float32)
    points = points[np.isfinite(points).all(axis=1)]
    height_filter = cfg.get('height_filter')
    if height_filter:
        points = points[
            (points[:, 2] >= float(height_filter['min'])) &
            (points[:, 2] <= float(height_filter['max']))]
    if len(points) == 0:
        raise RuntimeError(f'No finite XYZ points in {pcd_path}')

    resolution = float(cfg['resolution'])
    slice_dh = float(cfg['slice_dh'])
    ground_cfg = cfg.get('ground_height', 'auto')
    ground_h = float(np.min(points[:, 2])) if ground_cfg == 'auto' else float(ground_cfg)
    trav_cfg = cfg['traversability']

    pmin = np.min(points, axis=0)
    pmax = np.max(points, axis=0)
    pmin[2] = ground_h
    dim_x = int(np.ceil((pmax[0] - pmin[0]) / resolution)) + 4
    dim_y = int(np.ceil((pmax[1] - pmin[1]) / resolution)) + 4
    n_slices = max(1, int(np.ceil((pmax[2] - ground_h) / slice_dh)))
    center = ((pmax[:2] + pmin[:2]) / 2.0).astype(np.float32)
    slice_h0 = ground_h + slice_dh

    ix = np.rint((points[:, 0] - center[0]) / resolution).astype(np.int32) + dim_x // 2
    iy = np.rint((points[:, 1] - center[1]) / resolution).astype(np.int32) + dim_y // 2
    valid = (ix >= 0) & (ix < dim_x) & (iy >= 0) & (iy < dim_y)
    ix, iy, pz = ix[valid], iy[valid], points[valid, 2]

    layers_g = np.full((n_slices, dim_x, dim_y), -1e6, dtype=np.float32)
    layers_c = np.full((n_slices, dim_x, dim_y), 1e6, dtype=np.float32)
    for layer in range(n_slices):
        height = slice_h0 + layer * slice_dh
        below = pz <= height
        np.maximum.at(layers_g[layer], (ix[below], iy[below]), pz[below])
        above = ~below
        np.minimum.at(layers_c[layer], (ix[above], iy[above]), pz[above])

    grad_sq = np.zeros_like(layers_g)
    grad_max = np.zeros_like(layers_g)
    dx_sq = np.maximum(
        (layers_g[:, 1:-1, :] - layers_g[:, :-2, :]) ** 2,
        (layers_g[:, 1:-1, :] - layers_g[:, 2:, :]) ** 2)
    dy_sq = np.maximum(
        (layers_g[:, :, 1:-1] - layers_g[:, :, :-2]) ** 2,
        (layers_g[:, :, 1:-1] - layers_g[:, :, 2:]) ** 2)
    grad_sq[:, 1:-1, 1:-1] = dx_sq[:, :, 1:-1] + dy_sq[:, 1:-1, :]
    grad_max[:, 1:-1, 1:-1] = np.maximum(dx_sq[:, :, 1:-1], dy_sq[:, 1:-1, :])

    interval = layers_c - layers_g
    barrier = float(trav_cfg['cost_barrier'])
    step_stand = 1.2 * resolution * np.tan(float(trav_cfg['slope_max_rad']))
    step_cross = float(trav_cfg['step_max'])
    kernel_size = int(trav_cfg['kernel_size'])
    standable_th = int(float(trav_cfg['standable_ratio']) * kernel_size ** 2) - 1
    trav = np.maximum(0.0, 20.0 * (float(trav_cfg['interval_free']) - interval))
    blocked_clearance = interval < float(trav_cfg['interval_min'])
    flat = grad_sq <= step_stand ** 2
    trav[flat] += 15.0 * grad_sq[flat] / max(step_stand ** 2, 1e-9)
    neighbor_flat = np.empty_like(grad_sq, dtype=np.int16)
    footprint = np.ones((kernel_size, kernel_size), dtype=np.int16)
    for layer in range(n_slices):
        neighbor_flat[layer] = ndimage.convolve(
            flat[layer].astype(np.int16), footprint, mode='constant', cval=0)
    crossable = (~flat) & (grad_max <= step_cross ** 2) & (neighbor_flat >= standable_th)
    trav[crossable] += 20.0 * grad_max[crossable] / max(step_cross ** 2, 1e-9)
    trav[blocked_clearance | ((~flat) & (~crossable))] = barrier

    inflated = np.empty_like(trav)
    for layer in range(n_slices):
        inflated[layer] = _inflate(
            trav[layer], resolution, float(trav_cfg['safe_margin']),
            float(trav_cfg['inflation']))

    selected = [0]
    if not bool(cfg.get('simplify_layers', True)):
        selected = list(range(n_slices))
    elif n_slices > 1:
        lower, middle = 0, 1
        diff_h = layers_g[1:] - layers_g[:-1]
        while middle < n_slices - 2:
            unique = ((layers_g[middle] - layers_g[lower] > 0) |
                      (inflated[lower] > inflated[middle]))
            unique &= diff_h[middle] > 0
            unique &= inflated[middle] < barrier
            if np.any(unique):
                selected.append(middle)
                lower = middle
            middle += 1
        if middle not in selected:
            selected.append(middle)
    selected = np.asarray(selected, dtype=np.int32)

    trav_gx = np.zeros_like(layers_g[selected])
    trav_gy = np.zeros_like(layers_g[selected])
    trav_gx[:, 1:-1, :] = inflated[selected, 2:, :] - inflated[selected, :-2, :]
    trav_gy[:, :, 1:-1] = inflated[selected, :, 2:] - inflated[selected, :, :-2]
    out_g = np.where(layers_g[selected] > -1e6, layers_g[selected], np.nan)
    out_c = np.where(layers_c[selected] < 1e6, layers_c[selected], np.nan)
    data = np.stack((inflated[selected], trav_gx, trav_gy, out_g, out_c))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        'data': data.astype(np.float16),
        'resolution': resolution,
        'center': center,
        'slice_h0': slice_h0,
        'slice_dh': slice_dh,
        'source_pcd': str(pcd_path),
        'backend': 'numpy-scipy-cpu',
        'selected_source_layers': selected,
    }
    with open(output_path, 'wb') as stream:
        pickle.dump(payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
    return points, payload


def export_preview(payload, output_path):
    data = np.asarray(payload['data'], dtype=np.float32)
    ground, cost = data[3], data[0]
    resolution = float(payload['resolution'])
    center = np.asarray(payload['center'])
    chunks, colors = [], []
    for layer in range(ground.shape[0]):
        gx, gy = np.where(np.isfinite(ground[layer]))
        if len(gx) == 0:
            continue
        xyz = np.column_stack(((gx - ground.shape[1] // 2) * resolution + center[0],
                               (gy - ground.shape[2] // 2) * resolution + center[1],
                               ground[layer, gx, gy]))
        normalized = np.clip(cost[layer, gx, gy] / 50.0, 0.0, 1.0)
        rgb = np.column_stack((normalized, 1.0 - normalized, np.full_like(normalized, 0.15)))
        chunks.append(xyz)
        colors.append(rgb)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.concatenate(chunks, axis=0))
    cloud.colors = o3d.utility.Vector3dVector(np.concatenate(colors, axis=0))
    o3d.io.write_point_cloud(str(output_path), cloud, write_ascii=False)


def main():
    parser = argparse.ArgumentParser(description='Build a PCT tomogram without CUDA/ROS1.')
    parser.add_argument('--pcd', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--preview', type=Path)
    args = parser.parse_args()
    started = time.perf_counter()
    cfg = _load_config(args.config)
    points, payload = build_tomogram(args.pcd, args.output, cfg)
    if args.preview:
        export_preview(payload, args.preview)
    shape = np.asarray(payload['data']).shape
    print(f'PCT tomogram created: {args.output}')
    print(f'Input points: {len(points)}, tensor shape: {shape}, backend: {payload["backend"]}')
    print(f'Elapsed: {time.perf_counter() - started:.3f} s')


if __name__ == '__main__':
    main()
