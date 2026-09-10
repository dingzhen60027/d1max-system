#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import numpy as np
import open3d as o3d


def main():
    parser = argparse.ArgumentParser(
        description='Conservative structural filtering for accumulated SLAM maps.')
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--z-min', type=float, default=-1.0)
    parser.add_argument('--z-max', type=float, default=9.0)
    parser.add_argument('--voxel', type=float, default=0.04)
    parser.add_argument('--sor-neighbors', type=int, default=24)
    parser.add_argument('--sor-std', type=float, default=1.2)
    parser.add_argument('--radius', type=float, default=0.16)
    parser.add_argument('--radius-points', type=int, default=6)
    args = parser.parse_args()

    started = time.perf_counter()
    cloud = o3d.io.read_point_cloud(str(args.input))
    points = np.asarray(cloud.points)
    finite = np.isfinite(points).all(axis=1)
    height = (points[:, 2] >= args.z_min) & (points[:, 2] <= args.z_max)
    cloud = cloud.select_by_index(np.flatnonzero(finite & height))
    cropped_count = len(cloud.points)

    cloud = cloud.voxel_down_sample(args.voxel)
    voxel_count = len(cloud.points)
    cloud, _ = cloud.remove_statistical_outlier(
        nb_neighbors=args.sor_neighbors, std_ratio=args.sor_std)
    statistical_count = len(cloud.points)
    cloud, _ = cloud.remove_radius_outlier(
        nb_points=args.radius_points, radius=args.radius)
    radius_count = len(cloud.points)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(args.output), cloud, write_ascii=False):
        raise RuntimeError(f'Failed to write {args.output}')

    print(f'input={len(points)} cropped={cropped_count} voxel={voxel_count} '
          f'statistical={statistical_count} radius={radius_count}')
    print(f'removed={len(points) - radius_count} '
          f'({100.0 * (len(points) - radius_count) / len(points):.2f}%)')
    print(f'output={args.output}')
    print(f'elapsed={time.perf_counter() - started:.3f}s')


if __name__ == '__main__':
    main()
