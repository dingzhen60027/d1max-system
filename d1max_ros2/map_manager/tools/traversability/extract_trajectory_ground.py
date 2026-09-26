"""Conservative, trajectory-assisted measured ground subset, no flattening.

For a single-floor map whose pose graph retains some vertical deformation.
Not a traversability map, semantic classifier, or multi-level ground model.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
import open3d as o3d
import yaml
from scipy.spatial import cKDTree
from scipy import ndimage
from extract_ground import read_source, write_xyzi


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    args = parser.parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    source = Path(cfg['source']['pcd'])
    out = Path(cfg['output']['directory'])
    if out.exists():
        raise ValueError('Output already exists; select a new directory to preserve results')
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    records = read_source(source)
    assert len(records) == cfg['source']['expected_points']
    valid = np.isfinite(records[:, :3]).all(1)
    original = np.flatnonzero(valid)
    xyz = records[valid, :3]
    poses = np.loadtxt(cfg['source']['poses']).reshape(-1, 3, 4)[:, :, 3]
    assert np.isfinite(poses).all()
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    nc = cfg['normals']; tc = cfg['trajectory']; cc = cfg['cleanup']
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=nc['radius_m'], max_nn=nc['max_neighbors']))
    normals = np.asarray(cloud.normals)
    distance, near = cKDTree(poses[:, :2]).query(xyz[:, :2])
    separation = poses[near, 2] - xyz[:, 2]
    horizontal = abs(normals[:, 2]) >= nc['minimum_abs_z']
    lo, hi = tc['offset_range_m']; step = tc['histogram_bin_m']
    seed = horizontal & (distance < tc['seed_radius_m']) & (separation > lo) & (separation < hi)
    hist, edges = np.histogram(separation[seed], bins=np.arange(lo, hi+step, step))
    peak = int(np.argmax(hist)); initial = (edges[peak]+edges[peak+1])/2
    if hist[peak] < 100:
        raise ValueError('No supported ground-height mode; requires manual review')
    offset = float(np.median(separation[seed & (abs(separation-initial) < step)]))
    mask = horizontal & (distance <= tc['max_lateral_distance_m']) & (abs(separation-offset) <= tc['ground_band_m'])
    ids = np.flatnonzero(mask)
    candidate_count = len(ids)
    candidate = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz[ids]))
    _, good = candidate.remove_radius_outlier(nb_points=cc['minimum_neighbors'], radius=cc['radius_m'])
    ids = ids[np.asarray(good, dtype=int)]
    if not len(ids):
        raise ValueError('No supported ground points')
    cell = np.floor((xyz[ids, :2]-xyz[ids, :2].min(0))/cc['component_grid_m']).astype(int)
    occupied = np.zeros(tuple(cell.max(0)+1), dtype=bool)
    occupied[cell[:, 0], cell[:, 1]] = True
    labels, _ = ndimage.label(occupied, np.ones((3, 3)))
    sizes = np.bincount(labels.ravel())
    ids = ids[sizes[labels[cell[:, 0], cell[:, 1]]] >= cc['minimum_component_cells']]
    indices = original[ids]
    assert len(indices) > 0 and len(np.unique(indices)) == len(indices)
    ground = records[indices]
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(args.config, out/'pipeline.yaml')
    write_xyzi(out/'ground.pcd', ground)
    rest = np.ones(len(records), dtype=bool); rest[indices] = False
    write_xyzi(out/'non_ground.pcd', records[rest])
    g = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(ground[:, :3]))
    o3d.io.write_point_cloud(str(out/'ground.ply'), g)
    np.save(out/'source_indices.npy', indices)
    assert np.array_equal(read_source(out/'ground.pcd'), records[indices])
    assert digest == hashlib.sha256(source.read_bytes()).hexdigest()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(15, 7), facecolor='#101722')
    p = ground[:, :3]
    for ax in axes:
        ax.set_facecolor('#101722'); ax.set_aspect('equal'); ax.tick_params(colors='white')
        ax.set_xlabel('X (m)', color='white'); ax.set_ylabel('Y (m)', color='white')
    axes[0].scatter(xyz[::4, 0], xyz[::4, 1], s=.12, c='#536174', linewidths=0)
    axes[0].scatter(p[:, 0], p[:, 1], s=.15, c='#41d7a1', linewidths=0)
    axes[0].set_title('Source (gray) / extracted ground (green)', color='white')
    plotted = axes[1].scatter(p[:, 0], p[:, 1], c=p[:, 2], cmap='turbo', s=.3, linewidths=0)
    axes[1].set_title('Ground only / original Z coordinates', color='white')
    bar = fig.colorbar(plotted, ax=axes[1], fraction=.03); bar.ax.tick_params(colors='white')
    fig.tight_layout(); fig.savefig(out/'ground_preview.png', dpi=160); plt.close(fig)
    report = {'source': str(source), 'source_sha256': digest, 'input_points': len(records),
              'candidate_points': candidate_count, 'ground_points': len(ground), 'non_ground_points': int(rest.sum()),
              'estimated_trajectory_ground_offset_m': offset, 'ground_bounds': [p.min(0).tolist(), p.max(0).tolist()],
              'source_unchanged': True, 'exact_source_subset_verified': True, 'synthetic_points': 0, 'coordinate_changes': False,
              'limitations': ['Trajectory-assisted single-floor geometry filter; not semantic ground truth.',
                'Farther than 6 m from the recorded route remains unclassified as ground.',
                'No hole filling, flattening, obstacle inflation or traversability guarantee.']}
    (out/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
