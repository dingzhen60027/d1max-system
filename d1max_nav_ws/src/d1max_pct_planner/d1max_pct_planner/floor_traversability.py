#!/usr/bin/env python3
import argparse
import pickle
import time
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy import ndimage


def build_floor_map(points, resolution=0.10, ground_min=-1.0, ground_max=0.10,
                    fill_radius=0.45, obstacle_min_height=0.16,
                    obstacle_max_height=0.90, obstacle_min_points=2,
                    robot_radius=0.24, cost_margin=0.22,
                    max_slope_rad=0.40):
    pmin = points.min(axis=0)
    pmax = points.max(axis=0)
    center = ((pmin[:2] + pmax[:2]) / 2.0).astype(np.float32)
    nx = int(np.ceil((pmax[0] - pmin[0]) / resolution)) + 4
    ny = int(np.ceil((pmax[1] - pmin[1]) / resolution)) + 4
    ix = np.rint((points[:, 0] - center[0]) / resolution).astype(np.int32) + nx // 2
    iy = np.rint((points[:, 1] - center[1]) / resolution).astype(np.int32) + ny // 2
    inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    points, ix, iy = points[inside], ix[inside], iy[inside]

    ground = np.full((nx, ny), np.inf, dtype=np.float32)
    candidate = (points[:, 2] >= ground_min) & (points[:, 2] <= ground_max)
    np.minimum.at(ground, (ix[candidate], iy[candidate]), points[candidate, 2])
    observed = np.isfinite(ground)
    if not observed.any():
        raise RuntimeError('No ground candidates found in configured height range')

    distance_cells, nearest = ndimage.distance_transform_edt(
        ~observed, return_indices=True)
    fillable = distance_cells * resolution <= fill_radius
    filled = ground.copy()
    filled[fillable] = ground[nearest[0][fillable], nearest[1][fillable]]
    valid = observed | fillable
    median_source = np.where(valid, filled, np.nanmedian(filled[observed]))
    smooth = ndimage.median_filter(median_source, size=5, mode='nearest')
    smooth = ndimage.gaussian_filter(smooth, sigma=1.0, mode='nearest')

    local_ground = smooth[ix, iy]
    dz = points[:, 2] - local_ground
    obstacle_points = ((dz >= obstacle_min_height) &
                       (dz <= obstacle_max_height))
    obstacle_count = np.zeros((nx, ny), dtype=np.uint16)
    np.add.at(obstacle_count, (ix[obstacle_points], iy[obstacle_points]), 1)
    obstacle = obstacle_count >= obstacle_min_points

    grad_x, grad_y = np.gradient(smooth, resolution, resolution)
    steep = np.hypot(grad_x, grad_y) > np.tan(max_slope_rad)
    obstacle |= steep & valid
    obstacle |= ~valid

    clearance = ndimage.distance_transform_edt(~obstacle) * resolution
    blocked = clearance < robot_radius
    cost = np.clip(
        19.0 * (robot_radius + cost_margin - clearance) / max(cost_margin, 1e-6),
        0.0, 19.0).astype(np.float32)
    cost[blocked] = 50.0
    cost[~valid] = 50.0

    ground_out = np.where(valid, smooth, np.nan).astype(np.float32)
    ceiling = np.where(valid, smooth + 2.0, np.nan).astype(np.float32)
    cost_grad_x = np.zeros_like(cost)
    cost_grad_y = np.zeros_like(cost)
    cost_grad_x[1:-1] = cost[2:] - cost[:-2]
    cost_grad_y[:, 1:-1] = cost[:, 2:] - cost[:, :-2]
    tensor = np.stack((cost[None], cost_grad_x[None], cost_grad_y[None],
                       ground_out[None], ceiling[None]))
    return tensor, center, observed, valid, obstacle, clearance


def export_preview(tensor, center, resolution, output):
    ground = tensor[3, 0]
    cost = tensor[0, 0]
    ix, iy = np.where(np.isfinite(ground))
    xyz = np.column_stack(((ix - ground.shape[0] // 2) * resolution + center[0],
                           (iy - ground.shape[1] // 2) * resolution + center[1],
                           ground[ix, iy]))
    normalized = np.clip(cost[ix, iy] / 50.0, 0.0, 1.0)
    colors = np.column_stack((normalized, 1.0 - normalized,
                              np.full_like(normalized, 0.12)))
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    cloud.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud(str(output), cloud, write_ascii=False)


def main():
    parser = argparse.ArgumentParser(description='Build a robust single-floor PCT map.')
    parser.add_argument('--pcd', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--preview', type=Path)
    parser.add_argument('--resolution', type=float, default=0.10)
    parser.add_argument('--ground-min', type=float, default=-1.0)
    parser.add_argument('--ground-max', type=float, default=0.10)
    parser.add_argument('--fill-radius', type=float, default=0.45)
    parser.add_argument('--robot-radius', type=float, default=0.24)
    parser.add_argument('--cost-margin', type=float, default=0.22)
    args = parser.parse_args()
    started = time.perf_counter()
    cloud = o3d.io.read_point_cloud(str(args.pcd))
    points = np.asarray(cloud.points, dtype=np.float32)
    points = points[np.isfinite(points).all(axis=1)]
    tensor, center, observed, valid, obstacle, clearance = build_floor_map(
        points, resolution=args.resolution, ground_min=args.ground_min,
        ground_max=args.ground_max, fill_radius=args.fill_radius,
        robot_radius=args.robot_radius, cost_margin=args.cost_margin)
    payload = {
        'data': tensor.astype(np.float16),
        'resolution': args.resolution,
        'center': center,
        'slice_h0': args.ground_max,
        'slice_dh': 0.5,
        'source_pcd': str(args.pcd),
        'backend': 'single-floor-elevation-cpu',
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, 'wb') as stream:
        pickle.dump(payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
    if args.preview:
        export_preview(tensor, center, args.resolution, args.preview)
    free = np.isfinite(tensor[3, 0]) & (tensor[0, 0] < 20.0)
    labels, count = ndimage.label(free, np.ones((3, 3), dtype=np.uint8))
    sizes = np.bincount(labels.ravel())
    largest = int(sizes[1:].max()) if len(sizes) > 1 else 0
    print(f'floor map={args.output} grid={tensor.shape[2:]} observed={observed.sum()} '
          f'valid={valid.sum()} obstacle={obstacle.sum()} free={free.sum()} '
          f'components={count} largest={largest}')
    print(f'elapsed={time.perf_counter() - started:.3f}s')


if __name__ == '__main__':
    main()
