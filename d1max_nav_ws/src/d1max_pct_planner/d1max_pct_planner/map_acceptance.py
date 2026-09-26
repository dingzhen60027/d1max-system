"""Read-only floor connectivity checks, independent of native route smoothing.

4-neighbor connections check the measured/derived floor step. Diagonal corner
cutting and transitions through unknown cells cannot make a map pass this test.
No geometry, cost, obstacle inflation or planner settings are changed.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from .tomogram_map import TomogramMap


def floor_components(allowed, ground, max_step):
    allowed, ground = np.asarray(allowed, dtype=bool), np.asarray(ground, dtype=float)
    if allowed.ndim != 2 or allowed.shape != ground.shape or not np.isfinite(max_step) or max_step <= 0:
        raise ValueError('Expected matching 2D surface arrays and positive step bound')
    allowed = allowed & np.isfinite(ground)
    ids = np.full(allowed.shape, -1, dtype=np.int64)
    ids[allowed] = np.arange(allowed.sum())
    pairs = []
    for axis in (0, 1):
        first = [slice(None), slice(None)]
        last = first.copy()
        first[axis], last[axis] = slice(None, -1), slice(1, None)
        a, b = tuple(first), tuple(last)
        edge = allowed[a] & allowed[b] & (np.abs(ground[a]-ground[b]) <= max_step + 1e-6)
        pairs.append((ids[a][edge], ids[b][edge]))
    row = np.concatenate([item[0] for item in pairs])
    col = np.concatenate([item[1] for item in pairs])
    graph = coo_matrix((np.ones(len(row), dtype=np.uint8), (row, col)),
                      shape=(int(allowed.sum()), int(allowed.sum()))).tocsr()
    count, labels = connected_components(graph, directed=False)
    output = np.zeros(allowed.shape, dtype=np.int32)
    output[allowed] = labels+1
    return output, np.bincount(labels, minlength=count)


def densify_xy(poses, spacing):
    points = np.asarray(poses, dtype=float)
    if points.ndim != 2 or points.shape[1] not in (2, 3) or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('Expected at least two finite trajectory poses')
    if not np.isfinite(spacing) or spacing <= 0:
        raise ValueError('Spacing must be positive')
    result = []
    for a, b in zip(points[:-1, :2], points[1:, :2]):
        n = max(1, int(np.ceil(np.linalg.norm(b-a)/spacing)))
        result.append(a+np.arange(n)[:, None]/n*(b-a))
    return np.vstack([*result, points[-1:, :2]])


def inspect(tomogram, poses, *, layer=0, spacing=.10, association_radius=.60):
    layer = tomogram.layer(layer)
    if not np.isfinite(association_radius) or not 0 < association_radius <= 1:
        raise ValueError('Reference association radius must be in (0,1] m')
    labels, sizes = floor_components(tomogram.allowed[layer], tomogram.ground[layer],
                                     tomogram.max_ground_step_m)
    dense = densify_xy(poses, spacing)
    indices = np.rint((dense-tomogram.center)/tomogram.resolution).astype(int)+tomogram.offset
    inside = ((indices >= 0) & (indices < tomogram.shape)).all(axis=1)
    centre_components = np.zeros(len(dense), dtype=int)
    centre_components[inside] = labels[tuple(indices[inside].T)]
    free_indices = np.argwhere(labels > 0)
    if not len(free_indices):
        raise ValueError('No valid floor cells remain')
    free_xy = tomogram.center+(free_indices-tomogram.offset)*tomogram.resolution
    free_labels = labels[tuple(free_indices.T)]
    distance, nearest = cKDTree(free_xy).query(dense)
    nearest_labels = np.where(distance <= association_radius, free_labels[nearest], 0)
    counts = np.bincount(nearest_labels, minlength=len(sizes)+1)
    primary = int(np.argmax(counts[1:]))+1
    primary_xy = free_xy[free_labels == primary]
    primary_distance, _ = cKDTree(primary_xy).query(dense)
    near = primary_distance <= association_radius
    pose_distance, pose_nearest = cKDTree(primary_xy).query(np.asarray(poses)[:, :2])
    representative_indices = np.unique(np.r_[0, np.arange(0, len(poses), max(1, len(poses)//16)), len(poses)-1])
    landmarks = []
    for index in representative_indices:
        xyz = np.asarray(poses)[index]
        point = primary_xy[pose_nearest[index]]
        landmarks.append({'pose_index': int(index), 'reference_xy': xyz[:2].tolist(),
                          'candidate_xy': point.tolist(), 'distance_m': float(pose_distance[index]),
                          'within_association_radius': bool(pose_distance[index] <= association_radius),
                          'component': primary})
    output = {
        'source_tomogram': tomogram.source, 'source_sha256': tomogram.sha256,
        'frame_id': tomogram.provenance.get('frame_id'),
        'retained_layer': layer, 'source_layer': int(tomogram.source_layers[layer]),
        'graph': '4-neighbor known/allowed floor cells with <= max_ground_step edges',
        'max_ground_step_m': tomogram.max_ground_step_m,
        'component_count': len(sizes), 'component_sizes_descending': sorted(sizes.tolist(), reverse=True),
        'trajectory_samples': len(dense), 'sampling_spacing_m': spacing,
        'reference_xy_total_length_m': float(np.linalg.norm(np.diff(dense, axis=0), axis=1).sum()),
        'exact_reference_cell_allowed': int((centre_components > 0).sum()),
        'exact_reference_cell_components': sorted(set(centre_components[centre_components > 0].tolist())),
        'association_radius_m': association_radius,
        'nearest_floor_component_counts': {str(i): int(n) for i, n in enumerate(counts) if n},
        'primary_component': primary,
        'samples_within_primary_component_radius': int(near.sum()),
        'all_sampled_trajectory_near_one_component': bool(near.all()),
        'primary_component_distance_percentiles_m': np.percentile(primary_distance, [0,50,95,99,100]).tolist(),
        'representative_landmarks': landmarks,
        'caveat': 'Association is an offline geometry coverage test, not a robot footprint/turning clearance certification. '
                  'It does not require every historic pose centre to be a legal new waypoint, and does not modify or snap user endpoints.'}
    return output, {'labels': labels, 'dense_trajectory_xy': dense,
                    'primary_component_distance_m': primary_distance,
                    'exact_reference_component': centre_components}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tomogram', type=Path, required=True)
    parser.add_argument('--trajectory', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--layer', type=int, default=0)
    parser.add_argument('--association-radius', type=float, default=.6)
    args = parser.parse_args()
    poses = np.loadtxt(args.trajectory, ndmin=2)
    if poses.shape[1] != 12:
        raise ValueError('Expected KITTI 3x4 trajectory')
    tomo = TomogramMap(args.tomogram, minimum_headroom_m=.55,
                       unknown_ceiling_policy='allow_unobserved', max_ground_step_m=.17)
    report, arrays = inspect(tomo, poses[:, [3,7,11]], layer=args.layer,
                             association_radius=args.association_radius)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'connectivity.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    np.savez_compressed(args.output/'connectivity.npz', **arrays)
    print(json.dumps({k:v for k,v in report.items() if k not in ('representative_landmarks', 'caveat')}, indent=2))


if __name__ == '__main__':
    main()
