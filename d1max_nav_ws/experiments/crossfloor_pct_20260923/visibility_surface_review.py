"""Offline audit of persistent-surface protection for visibility candidates.

Keeps candidates only when they lie on a locally observed persistent plane,
not merely near any observed surface. Does not alter the source map or config.
"""
import json
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from visibility_clean import PCD

ROOT = Path(__file__).resolve().parent


def main():
    points = np.asarray(o3d.io.read_point_cloud(PCD).points)
    with np.load(ROOT / 'visibility_votes.npz') as v:
        free, hit, roi = v['free'], v['hit'], v['in_roi']
    candidate = roi & (free >= 10 * np.maximum(hit, 1)) & (free >= 20)
    persistent = roi & (hit >= 10) & (free < 3 * hit)
    query = points[candidate]
    stable = points[persistent]
    tree = cKDTree(stable)
    distance, index = tree.query(query, k=32, distance_upper_bound=0.25)
    normals = np.full_like(query, np.nan)
    errors = np.full(len(query), np.inf)
    ratios = np.full(len(query), np.inf)
    counts = np.sum(np.isfinite(distance), axis=1)
    for i in np.flatnonzero(counts >= 8):
        neighbors = stable[index[i, np.isfinite(distance[i])]]
        mean = neighbors.mean(axis=0)
        scatter = (neighbors - mean).T @ (neighbors - mean) / len(neighbors)
        values, vectors = np.linalg.eigh(scatter)
        normal = vectors[:, 0]
        normals[i] = normal
        errors[i] = abs((query[i] - mean) @ normal)
        ratios[i] = values[0] / max(values[1], 1e-10)
    masks = {}
    ids = np.flatnonzero(candidate)
    for name, scale, radius in [
        ('sphere12', [1, 1, 1], .12),
        ('ellipsoid05', [1, 1, .12 / .05], .12),
        ('ellipsoid03', [1, 1, .12 / .03], .12),
    ]:
        near = cKDTree(stable * scale).query_ball_point(query * scale, radius, return_length=True) > 0
        remove = candidate.copy()
        remove[ids[near]] = False
        masks[name] = remove
    for threshold in [.02, .03, .04]:
        protected = (ratios < .08) & (errors <= threshold)
        remove = candidate.copy()
        remove[ids[protected]] = False
        masks[f'plane{round(threshold * 100):02d}'] = remove
    masks['unprotected'] = candidate
    boxes = {
        'wall1': ([-34, 48, -1.4], [-33, 49.5, 4.9]),
        'floor_edge': ([-34.5, 44, 3.02], [-25.5, 45.5, 3.3]),
        'stair_top': ([-31.3, 50.8, 3.18], [-30.7, 51.6, 3.35]),
        'ghost_middle': ([-29.7, 54.5, 1.42], [-29.1, 55.2, 2.8]),
        'curve_problem': ([-29.0, 53.2, .70], [-28.5, 53.65, 1.40]),
        'upper_corridor': ([-27.2, 45.2, 3.1], [-25.4, 46.8, 3.3]),
    }
    report = {'candidate_points': int(candidate.sum()), 'persistent_points': int(persistent.sum()),
              'removed': {key: int(value.sum()) for key, value in masks.items()}, 'boxes': {}}
    for name, (low, high) in boxes.items():
        in_box = np.all((points >= low) & (points <= high), axis=1)
        report['boxes'][name] = {'points': int(in_box.sum()), **{
            key: int(np.sum(value & in_box)) for key, value in masks.items()}}
    # Candidate details at stair head distinguish a 9cm ghost from real floor.
    low, high = boxes['stair_top']
    selected = np.all((query >= low) & (query <= high), axis=1)
    report['stair_top_candidates'] = [
        {'point': query[i].round(5).tolist(), 'free': int(free[ids[i]]), 'hit': int(hit[ids[i]]),
         'plane_distance_m': round(float(errors[i]), 5), 'planarity': round(float(ratios[i]), 5),
         'persistent_neighbors': int(counts[i]), 'normal': normals[i].round(5).tolist(),
         **{key: bool(mask[ids[i]]) for key, mask in masks.items()}}
        for i in np.flatnonzero(selected)]
    print(json.dumps(report, indent=2))
    (ROOT / 'visibility_surface_review.json').write_text(json.dumps(report, indent=2) + '\n')
    np.savez_compressed(ROOT / 'visibility_surface_masks.npz', **masks)


if __name__ == '__main__':
    main()
