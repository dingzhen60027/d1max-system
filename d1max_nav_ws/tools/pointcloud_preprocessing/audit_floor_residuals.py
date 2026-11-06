"""Read-only original-point evidence for reviewing a remaining corridor layer.

The emitted region is a proposal only, never a map-processing configuration.
No point removal or coordinate change is performed by this diagnostic.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .flat_floor import condition_flat_floor
from .flat_floor_runner import structural_cleanup
from .multifloor_runner import protected_mask
from .pcd_io import read_pcd
from .runner import sha256


def audit(package, output):
    from d1max_pct_planner.tomogram_map import TomogramMap
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.patches import Polygon, Rectangle

    package, output = Path(package).resolve(strict=True), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((package / 'manifest.json').read_text())
    stage = json.loads(Path(manifest['floor_processing_manifest']).read_text())
    source = read_pcd(manifest['source_path'])
    cfg, defaults = stage['configuration'], stage['resolved_floor_defaults']
    if sha256(manifest['source_path']) != manifest['source_sha256']:
        raise ValueError('Original source hash changed')
    poses = np.loadtxt(stage['trajectory_path'])[:, [3, 7, 11]]
    floor = cfg['floors'][0]
    lo, hi = floor['input_z_band']
    ids = np.flatnonzero((source.xyz[:, 2] >= lo) & (source.xyz[:, 2] < hi)
                        & ~protected_mask(source.xyz, cfg['protected_regions']))
    a, b = floor['trajectory_range_inclusive']
    floor_poses = poses[a:b + 1]
    floor_poses = floor_poses[~protected_mask(floor_poses, cfg['protected_regions'])]
    result = condition_flat_floor(source.xyz[ids], floor_poses,
        {**defaults['conditioning'], **cfg.get('conditioning_overrides', {}),
         'reference_z_m': floor['reference_z_m']})
    ix, structures = structural_cleanup(result,
        {**defaults['obstacle_cleanup'], **cfg.get('obstacle_cleanup_overrides', {}),
         'floor_reference_z_m': floor['reference_z_m']})
    wall = np.zeros(len(ids), dtype=bool)
    low_object = wall.copy()
    wall[ix] = structures['wall_protected_mask']
    low_object[ix] = structures['low_object_protected_mask']
    relative = source.xyz[ids, 2] - result['floor_z']
    tree = cKDTree(source.xyz[ids, :2])
    records = []
    for source_id in (101667, 101553, 142867, 149245, 71581, 71636):
        index = int(np.flatnonzero(ids == source_id)[0])
        point = source.xyz[source_id]
        near = np.asarray(tree.query_ball_point(point[:2], .3), dtype=int)
        near = near[np.isfinite(relative[near]) & (np.abs(relative[near]) < .2)]
        records.append(dict(source_record=source_id, original_xyz=point.tolist(),
            original_floor_field_z=float(result['floor_z'][index]),
            residual_m=float(relative[index]), floor_support=bool(result['floor_support_mask'][index]),
            near_ground_protected=bool(result['near_ground_protected_mask'][index]),
            wall_protected=bool(wall[index]), low_object_protected=bool(low_object[index]),
            neighborhood_radius_m=.3, nearby_points=int(len(near)),
            lower_band_within_2cm=int((np.abs(relative[near]) <= .02).sum()),
            elevated_4_to_10cm=int(((relative[near] >= .04) & (relative[near] <= .1)).sum()),
            histogram_edges_m=np.arange(-.2, .201, .02).tolist(),
            histogram_counts=np.histogram(relative[near], bins=np.arange(-.2, .201, .02))[0].tolist()))
    polygon = [[-21.5, 45.8], [-17.5, 45.0], [-17.6, 44.5], [-21.6, 45.3]]
    proposal = dict(schema='d1max.reviewed_flat_corridor_proposal/v1',
        status='requires_explicit_region_review_not_applied', source_map_sha256=manifest['source_sha256'],
        floor_id='floor1', coordinate_frame='d1max_loc_map', polygon_xy=polygon,
        semantics='Proposed central corridor patch around the observed duplicate low layer, not a claim that every low object is an artifact.',
        mandatory_exclusions=['all protected stair volumes and stair ROI', 'wall-connected or upper-structure returns',
                              'unobserved lower ground', 'outside reviewed polygon'],
        uncertainty='Static density/flatness cannot distinguish this elevated layer from a real low object; explicit physical flat-floor confirmation is still required.')
    report = dict(package=str(package), manifest_sha256=sha256(package / 'manifest.json'),
                  source_map_sha256=manifest['source_sha256'], samples=records,
                  localization_map_unchanged=True, points_changed=0, region_proposal=proposal)
    (output / 'evidence.json').write_text(json.dumps(report, indent=2) + '\n')
    (output / 'reviewed_flat_corridor.proposal.json').write_text(json.dumps(proposal, indent=2) + '\n')
    tomo = TomogramMap(package / 'pct/tomogram.npz', unknown_ceiling_policy='allow_unobserved', max_ground_step_m=.17)
    cells = np.argwhere(tomo.allowed[4] & (np.abs(tomo.ground[4] + .65) < .3))
    xy = tomo.center + (cells - tomo.offset) * tomo.resolution
    costs = tomo.cost[4, cells[:, 0], cells[:, 1]]
    heights = tomo.ground[4, cells[:, 0], cells[:, 1]] + .65
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    cost_palette = LinearSegmentedColormap.from_list('runtime_cost_rgb',
                                                    [(45/255, 225/255, 65/255), (245/255, 180/255, 65/255)])
    for ax, value, title, cmap, vmax in [(axes[0, 0], costs, 'Current PCT cost: long corridor (runtime RGB palette)', cost_palette, 20),
                                      (axes[0, 1], heights, 'Actual derived ground residual; proposed review patch only', 'coolwarm', .12)]:
        keep = (xy[:, 0] >= -29) & (xy[:, 0] <= 8) & (xy[:, 1] >= 38) & (xy[:, 1] <= 49)
        plot = ax.scatter(*xy[keep].T, c=value[keep], s=6, marker='s', linewidths=0,
                          cmap=cmap, vmin=0 if vmax == 20 else -.12, vmax=vmax)
        ax.plot(floor_poses[:, 0], floor_poses[:, 1], '.', color='black', ms=1, alpha=.5, label='measured mapping trajectory')
        ax.add_patch(Polygon(polygon, closed=True, fill=False, edgecolor='magenta', linewidth=2, label='review proposal A (not applied)'))
        ax.add_patch(Rectangle((-34, 47), 7, 9, fill=False, edgecolor='black', hatch='//', label='stair ROI: excluded'))
        ax.set(xlim=(-29, 8), ylim=(38, 49), title=title, xlabel='source/planning X [m]', ylabel='Y [m]')
        ax.set_aspect('equal'); fig.colorbar(plot, ax=ax, shrink=.65)
    patch = (source.xyz[ids, 0] > -21.7) & (source.xyz[ids, 0] < -17.3) & (source.xyz[ids, 1] > 44.3) & (source.xyz[ids, 1] < 46)
    patch &= np.isfinite(relative) & (relative > -.1) & (relative < .2)
    ax = axes[1, 0]
    plot = ax.scatter(*source.xyz[ids[patch], :2].T, c=relative[patch], s=8, cmap='coolwarm', vmin=-.12, vmax=.12)
    ax.add_patch(Polygon(polygon, closed=True, fill=False, edgecolor='magenta', linewidth=2))
    ax.set(title='Original measured points minus original floor field', xlabel='X [m]', ylabel='Y [m]')
    ax.set_aspect('equal'); fig.colorbar(plot, ax=ax, shrink=.65)
    ax = axes[1, 1]
    first = records[0]
    edges = np.asarray(first['histogram_edges_m'])
    ax.bar((edges[:-1] + edges[1:]) / 2, first['histogram_counts'], width=.018)
    ax.axvspan(-.02, .02, color='green', alpha=.2, label='observed lower floor')
    ax.axvspan(.04, .10, color='orange', alpha=.2, label='distinct elevated returns')
    ax.set(title='0.3 m neighborhood of original point #101667', xlabel='Original height minus floor field [m]', ylabel='Measured records')
    ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(output / 'corridor_review.png', dpi=150); plt.close(fig)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    audit(**vars(parser.parse_args()))
