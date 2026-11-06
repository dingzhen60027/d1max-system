#!/usr/bin/env python3
"""Same requests, real production PCT coordinator, before/after floor repair.

Offline geometry regression only. No ROS, SDK, simulated plant or motion claim.
"""
import argparse
import gc
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'src/d1max_pct_planner'), str(ROOT / 'src/d1max_pct_scan')]


def distribution(values):
    return {'count': len(values), 'mean': float(np.mean(values)),
            'percentiles_0_25_50_75_90_95_99_100': np.percentile(values, [0, 25, 50, 75, 90, 95, 99, 100]).tolist(),
            'ge4': int((values >= 4).sum()), 'ge12': int((values >= 12).sum())}


def run(before, after, output, extra_requests=None):
    from d1max_pct_planner.cpu_tomography import traversal_cost, SENTINEL
    from d1max_pct_planner.crossfloor_preview import CrossfloorPreviewRoute
    from d1max_pct_planner.tomogram_map import TomogramMap
    from d1max_pct_planner.path_quality import path_quality
    from d1max_pct_scan.navigation_map_package import validate_package

    raw = yaml.safe_load((before / 'route.yaml').read_text())
    anchors = raw['anchors']
    requests = [dict(name='lower_long_start_to_entry', start=anchors['start'], goal=anchors['entry']),
                dict(name='lower_recent_initial_to_start',
                     start={'xyz': [2.4975318908691406, 9.169448852539062, -.65], 'layer_id': 4},
                     goal=anchors['start']),
                dict(name='upper_exit_to_goal', start=anchors['exit'], goal=anchors['goal']),
                dict(name='full_crossfloor', start=anchors['start'], goal=anchors['goal'])]
    if extra_requests is not None:
        requests.extend(json.loads(Path(extra_requests).read_text()))
    report = dict(actual_native=True, ros_graph_used=False, physical_acceptance=False,
                  execution_authorized=False, same_requests=requests, results={})
    output.mkdir(parents=True, exist_ok=False)
    baseline_masks = {}
    candidate = TomogramMap(after / 'pct/tomogram.npz',
                           unknown_ceiling_policy=raw['unknown_ceiling_policy'],
                           max_ground_step_m=raw['limits']['max_ground_step_m'])
    candidate_masks = {floor: (candidate.allowed[layer] & (candidate.ground[layer] >= low)
                               & (candidate.ground[layer] <= high))
                       for floor, layer, low, high in [('floor1', 4, -1., 0.), ('floor2', 12, 2.8, 3.5)]}
    del candidate
    for name, directory in [('before', before), ('after', after)]:
        checked = validate_package(directory)
        t = checked['tomogram']
        manifest = checked['manifest']
        cfg = manifest['configuration']['pct_pipeline']['pct']
        ground = np.where(np.isfinite(t.ground), t.ground, -SENTINEL)
        ceiling = np.where(np.isfinite(t.ceiling), t.ceiling, SENTINEL)
        intrinsic = traversal_cost(ground, ceiling, t.resolution, cfg['traversability'])
        costs = {}
        for floor, layer, low, high in [('floor1', 4, -1., 0.), ('floor2', 12, 2.8, 3.5)]:
            mask = t.allowed[layer] & (t.ground[layer] >= low) & (t.ground[layer] <= high)
            if name == 'before':
                baseline_masks[floor] = mask.copy()
            common = baseline_masks[floor] & candidate_masks[floor]
            costs[floor] = dict(allowed=distribution(t.cost[layer][mask]),
                intrinsic_same_cells=distribution(intrinsic[layer][common]),
                inflated_same_cells=distribution(t.cost[layer][common]),
                newly_allowed=int((mask & ~baseline_masks[floor]).sum()),
                no_longer_allowed=int((baseline_masks[floor] & ~mask).sum()))
        del ground, ceiling, intrinsic
        planner = CrossfloorPreviewRoute(t, directory / 'route.yaml')
        routes = []
        for request in requests:
            record = dict(name=request['name'])
            try:
                a, b = request['start'], request['goal']
                result = planner.plan(a['xyz'], b['xyz'], a['layer_id'], b['layer_id'])
                points = result.get('path', result.get('path_xyz'))
                record.update(success=True, length_m=result['length_m'],
                              path_quality=path_quality(points),
                              native_geometry=result.get('native_geometry'),
                              refinement=result.get('path_refinement'),
                              algorithm=result.get('algorithm'),
                              source_tomogram_sha256=result['source_tomogram_sha256'])
                (output / (name + '_' + request['name'] + '.json')).write_text(json.dumps(result, indent=2) + '\n')
            except Exception as exc:
                record.update(success=False, code=getattr(exc, 'code', type(exc).__name__),
                              message=str(exc), details=getattr(exc, 'details', {}))
            routes.append(record)
            print(json.dumps({'map': name, 'request': request['name'], 'success': record['success'],
                              'length_m': record.get('length_m'), 'code': record.get('code')}), flush=True)
        report['results'][name] = dict(tomogram_sha256=t.sha256, cost=costs, routes=routes)
        (output / 'comparison.json').write_text(json.dumps(report, indent=2) + '\n')
        del planner, t, checked
        gc.collect()
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('before', 'after', 'output'):
        parser.add_argument('--' + key, required=True, type=Path)
    parser.add_argument('--child', action='store_true')
    parser.add_argument('--extra-requests', type=Path)
    args = parser.parse_args()
    if args.child:
        run(args.before, args.after, args.output, args.extra_requests)
    else:
        from d1max_pct_planner.native_runtime import prepare_native_environment
        vendor = yaml.safe_load((args.before / 'route.yaml').read_text())['vendor_root']
        subprocess.run([sys.executable, __file__, *sys.argv[1:], '--child'],
                       env=prepare_native_environment(vendor), check=True)
