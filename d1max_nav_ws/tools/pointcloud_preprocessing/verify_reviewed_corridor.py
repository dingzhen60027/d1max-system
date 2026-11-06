"""Offline record, stair, raw-support and native-result checks for a reviewed ROI.

This checks a candidate, not a deployment or a motion authorization.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from .pcd_io import read_pcd
from .runner import sha256
from .reviewed_flat_corridor import polygon_mask, validate
from .flat_floor import condition_flat_floor
from .flat_floor_runner import structural_cleanup
from .multifloor_runner import protected_mask


def added_reviewed_prior(evidence, metric_evidence, floor_id):
    """Return only this repair's prior delta, without forgetting earlier review."""
    ids = evidence[floor_id + '_source_indices']
    np.testing.assert_array_equal(ids, metric_evidence[floor_id + '_source_indices'])
    key = floor_id + '_reviewed_floor_prior'
    after = evidence[key]
    before = metric_evidence.get(key, np.zeros(len(ids), dtype=bool))
    if (after.dtype.kind != 'b' or before.dtype.kind != 'b'
            or after.shape != ids.shape or before.shape != ids.shape
            or np.any(before & ~after)):
        raise ValueError('Reviewed prior must preserve aligned previous membership: ' + floor_id)
    return after & ~before


def verify(before, metric_reference, after, comparison, output):
    from d1max_pct_scan.navigation_map_package import validate_package, package_version
    from d1max_pct_scan.source_route import SourceRouteBuilder
    from d1max_pct_scan.static_route_validation import validate_static_route
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.patches import Polygon

    before, metric_reference, after, comparison, output = map(Path, (before, metric_reference, after, comparison, output))
    output.mkdir(parents=True, exist_ok=False)
    checked = validate_package(after)
    baseline = validate_package(before)
    metric_checked = validate_package(metric_reference)
    manifest = checked['manifest']
    stage = json.loads(Path(manifest['floor_processing_manifest']).read_text())
    review = validate(stage['configuration']['reviewed_flat_corridors'], manifest['source_sha256'])
    cloud = read_pcd(manifest['source_path'])
    old = read_pcd(before / 'processed_map.pcd')
    metric = read_pcd(metric_reference / 'processed_map.pcd')
    new = read_pcd(after / 'processed_map.pcd')
    assert len(old.records) == len(new.records) == len(metric.records)
    for name in old.fields:
        if name != 'z':
            assert old.records[name].tobytes() == new.records[name].tobytes()
    with np.load(after / 'record_lineage.npz', allow_pickle=False) as line:
        retained = line['retained_source_indices']
    assert old.records[len(retained):].tobytes() == new.records[len(retained):].tobytes()
    evidence = np.load(after / 'floor_stage/audit/floor_evidence.npz', allow_pickle=False)
    metric_evidence = np.load(metric_reference / 'floor_stage/audit/floor_evidence.npz', allow_pickle=False)
    prior_source = np.zeros(len(cloud.xyz), bool)
    walls = np.zeros(len(cloud.xyz), bool)
    cfg, defaults = stage['configuration'], stage['resolved_floor_defaults']
    poses = np.loadtxt(stage['trajectory_path'])[:, [3, 7, 11]]
    for floor in cfg['floors']:
        name = floor['id']
        ids = evidence[name + '_source_indices']
        np.testing.assert_array_equal(evidence[name + '_support'], metric_evidence[name + '_support'])
        prior_source[ids] = added_reviewed_prior(evidence, metric_evidence, name)
        a, b = floor['trajectory_range_inclusive']
        trajectory = poses[a:b + 1]
        trajectory = trajectory[~protected_mask(trajectory, cfg['protected_regions'])]
        result = condition_flat_floor(cloud.xyz[ids], trajectory,
            {**defaults['conditioning'], **cfg.get('conditioning_overrides', {}),
             'reference_z_m': floor['reference_z_m']})
        ix, structure = structural_cleanup(result,
            {**defaults['obstacle_cleanup'], **cfg.get('obstacle_cleanup_overrides', {}),
             'floor_reference_z_m': floor['reference_z_m']})
        walls[ids[ix]] = structure['wall_protected_mask']
    delta = new.records['z'] != metric.records['z']
    np.testing.assert_array_equal(delta[:len(retained)], prior_source[retained])
    assert not delta[len(retained):].any()
    assert old.records[:len(retained)][walls[retained]].tobytes() == new.records[:len(retained)][walls[retained]].tobytes()
    stairs = protected_mask(cloud.xyz, cfg['protected_regions'])
    assert old.records[:len(retained)][stairs[retained]].tobytes() == new.records[:len(retained)][stairs[retained]].tobytes()
    inside = np.zeros(len(cloud.xyz), bool)
    for region in review['regions']:
        inside |= polygon_mask(cloud.xyz[:, :2], region['polygon_xy'])
    assert not (prior_source & ~inside).any()
    # Isolate the reviewed prior from the independently audited metric repair.
    # Its PCT stair cells, not only source records, preserve every stored layer.
    t, b = checked['tomogram'], baseline['tomogram']
    cells = np.indices(t.ground.shape[1:]).reshape(2, -1).T
    xy = t.center + (cells - t.offset) * t.resolution
    stair_xy = np.zeros(len(cells), bool)
    for polygon in review['stair_exclusion_xy']:
        stair_xy |= polygon_mask(xy, polygon)
    for key in ('ground', 'ceiling', 'cost', 'allowed'):
        np.testing.assert_array_equal(getattr(t, key).reshape(len(t.ground), -1)[:, stair_xy],
                                      getattr(metric_checked['tomogram'], key).reshape(len(b.ground), -1)[:, stair_xy])
    builder = SourceRouteBuilder.from_artifacts(checked['manifest_path'], t, checked['bridge'])
    typed = []
    for path in sorted(comparison.glob('after_*.json')):
        result = json.loads(path.read_text())
        snapshot = validate_static_route(result, t, checked['bridge'], builder=builder,
                                         map_version_id=package_version(checked))['route_snapshot'].payload()
        support = snapshot['geometry_evidence']['measured_source_support']['all_samples_supported']
        assert support and snapshot['execution_eligible'] is False
        typed.append(dict(result_file=str(path), all_samples_original_supported=support,
                          execution_eligible=snapshot['execution_eligible']))
    assert len(typed) == 6
    route_profiles = {}
    for route_name in ('central_duplicate_layer_patch', 'long_straight_corridor_with_interference'):
        profiles = {}
        for phase in ('before', 'after'):
            route = np.array(json.loads((comparison/(phase+'_'+route_name+'.json')).read_text())['path'])
            stair_path = np.zeros(len(route), bool)
            for polygon in review['stair_exclusion_xy']:
                stair_path |= polygon_mask(route[:, :2], polygon)
            interior = ~stair_path
            interior[[0, -1]] = False
            profiles[phase] = dict(full_z_range_m=float(np.ptp(route[:, 2])),
                non_stair_z_range_m=float(np.ptp(route[~stair_path, 2])),
                non_stair_max_absolute_floor_residual_m=float(np.abs(route[~stair_path, 2]+.65).max()),
                non_stair_interior_z_range_m=float(np.ptp(route[interior, 2])),
                non_stair_interior_max_absolute_floor_residual_m=float(np.abs(route[interior, 2]+.65).max()),
                exact_request_endpoint_z_m=route[[0, -1], 2].tolist(),
                endpoint_semantics='Same original XYZ request endpoints are preserved; their old Z is not a new map ground sample',
                protected_stair_path_samples=int(stair_path.sum()))
        route_profiles[route_name] = profiles
    report = dict(candidate=str(after.resolve()), source_sha256=sha256(manifest['source_path']),
        candidate_manifest_sha256=sha256(after/'manifest.json'),
        candidate_pcd_sha256=sha256(after/'processed_map.pcd'),
        candidate_tomogram_sha256=sha256(after/'pct/tomogram.npz'),
        prior_changed_records=int(delta.sum()), total_changed_records=int((new.records['z'] != old.records['z']).sum()),
        record_count=len(new.records), non_z_fields_exact=True, restored_tail_exact=True,
        original_measured_support_membership_equal_metric_reference=True,
        original_wall_records_exact=int(walls[retained].sum()),
        original_stair_records_exact=int(stairs[retained].sum()),
        reviewed_prior_vs_metric_stair_xy_every_layer_ground_ceiling_cost_allowed_exact=True,
        prior_outside_reviewed_region_changes=0, typed_routes=typed, route_profiles=route_profiles,
        boundary_limit='PCT aggregates whole 0.1m cells; unchanged records just outside the reviewed polygon can still contribute to boundary cells. Other corridor patches are not covered by this prior.',
        physical_acceptance=False, realtime_obstacle_filter_changed=False)
    (output/'verification.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    palette = LinearSegmentedColormap.from_list('runtime_cost_rgb', [(45/255,225/255,65/255),(245/255,180/255,65/255)])
    for ax, tomo, label in zip(axes[0], (b, t), ('Before', 'After')):
        selected = tomo.allowed[4].ravel() & (np.abs(tomo.ground[4].ravel()+.65)<.2)
        selected &= (xy[:,0]>-31.5)&(xy[:,0]<7)&(xy[:,1]>38)&(xy[:,1]<49)
        p=ax.scatter(*xy[selected].T, c=tomo.cost[4].ravel()[selected], marker='s',s=24,
                     cmap=palette, vmin=0, vmax=20, linewidths=0)
        for region in review['regions']:
            ax.add_patch(Polygon(region['polygon_xy'], fill=False, edgecolor='magenta'))
        ax.set(title=label+' PCT cost (same runtime palette)', xlabel='X [m]',ylabel='Y [m]');ax.set_aspect('equal')
        fig.colorbar(p,ax=ax,shrink=.6)
    ax=axes[1,0]
    ax.scatter(*cloud.xyz[prior_source,:2].T,s=8)
    for region in review['regions']:
        ax.add_patch(Polygon(region['polygon_xy'],fill=False,edgecolor='magenta'))
    ax.set(title=f'{int(delta.sum())} corrected original records; XY unchanged',xlabel='X [m]',ylabel='Y [m]');ax.set_aspect('equal')
    ax=axes[1,1]
    for phase in ('before','after'):
        route=np.array(json.loads((comparison/(phase+'_long_straight_corridor_with_interference.json')).read_text())['path'])
        length=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(route[:,:2],axis=0),axis=1))]
        ax.plot(length,route[:,2]+.65,label=phase)
    ax.set(title='Actual native route ground profile',xlabel='XY distance [m]',ylabel='Derived ground above -0.65m [m]');ax.legend()
    fig.tight_layout();fig.savefig(output/'before_after.png',dpi=150);plt.close(fig)
    print(json.dumps(report,ensure_ascii=False))
    return report


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('before','metric-reference','after','comparison','output'):
        parser.add_argument('--'+name,required=True,type=Path)
    verify(**vars(parser.parse_args()))
