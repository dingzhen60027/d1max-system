"""Same-input offline latency/equivalence probe; no ROS graph, SDK or replay.

The baseline is an explicitly selected frozen Python package, not Git HEAD.
Recorded raw endpoints and source stamps are unchanged. Pose brackets are
constructed here, so these results do not claim measured live data age.
"""
import argparse
import hashlib
import importlib
import json
from pathlib import Path
import sys
import time
from types import ModuleType

import numpy as np
import yaml

from d1max_localization.math_utils import Pose3
from d1max_pct_scan import ray_projection as after
from d1max_pct_scan.robot_self_filter import RobotSelfFilter


ROOT = Path(__file__).resolve().parents[3]
GEOMETRY = ('x', 'y', 'z', 'origin_x', 'origin_y', 'origin_z')


def baseline(path):
    name = '_projector_latency_before'
    package = ModuleType(name)
    package.__path__ = [str(path)]
    sys.modules[name] = package
    return (importlib.import_module(name+'.ray_projection'),
            importlib.import_module(name+'.robot_self_filter').RobotSelfFilter)


def fixture(module, entity, points, start, body):
    context = module.MapContext('offline_latency', 1, 'seed', 1, start-1)
    core = module.RayProjectorCore(module.Limits(max_input_points=100000),
                                   projection_frame='odom', entity_filter=entity)
    core.reset(context)
    identity = Pose3((0., 0., 0.), (0., 0., 0., 1.))
    core.set_extrinsics(body_to_tracking=body, ray_to_tracking=identity)
    # Exact 10 ms brackets, no extrapolation or approximate beam-time binning.
    for index in range(17):
        core.add_local_body(start+index*10000000, identity, context)
    end = start+int(points['offset_time'].max())
    raw = module.RawRays(points, start, end, int(points['sensor_id'][0]))
    arguments = dict(now_ns=end+80000000, authorized_pose_ns=start+160000000)
    wire = dict(data=points.tobytes(), fields=module.FIELDS, point_step=64,
                row_step=len(points)*64, width=len(points), height=1, bigendian=False,
                header_ns=start, frame_id='lidar', expected_frame='lidar', max_points=100000)
    return core, raw, arguments, wire


def stats(samples):
    return dict(median_ms=float(np.median(samples)), p95_ms=float(np.percentile(samples, 95)),
                min_ms=min(samples), max_ms=max(samples))


def pair(before, old_filter, points, start, body, settings, repeats):
    old = fixture(before, old_filter, points, start, body)
    new = fixture(after, RobotSelfFilter.load(settings), points, start, body)
    totals = {name: dict(callback=[], geometry=[], complete=[]) for name in ('before', 'after')}
    original_bytes = points.tobytes()
    outputs = {}
    for index in range(repeats+3):
        # Interleave versions, alternating order to reduce host-load bias.
        names = ('before', 'after') if index % 2 else ('after', 'before')
        for name in names:
            module = before if name == 'before' else after
            core, raw, arguments, wire = old if name == 'before' else new
            decode = module.decode_rays if name == 'before' else module.decode_ray_envelope
            began = time.perf_counter()
            decoded = decode(**wire)
            callback_end = time.perf_counter()
            snapshot = core.projection_snapshot()
            if name == 'before':
                output = snapshot.project(decoded, core.context, **arguments)
            else:
                output = module.project_validated_rays(snapshot, decoded, core.context, **arguments)
            ended = time.perf_counter()
            # Separate geometry/CAD time excludes the migrated sample checks.
            began_geometry = time.perf_counter()
            geometry = core.projection_snapshot().project(raw, core.context, **arguments)
            ended_geometry = time.perf_counter()
            assert geometry.points.tobytes() == output.points.tobytes()
            if index >= 3:
                totals[name]['callback'].append((callback_end-began)*1000.)
                totals[name]['geometry'].append((ended_geometry-began_geometry)*1000.)
                totals[name]['complete'].append((ended-began)*1000.)
            outputs[name] = output
    prior, current = outputs['before'], outputs['after']
    assert prior.points.tobytes() == current.points.tobytes()
    assert len(prior.points) == len(current.points)
    assert points.tobytes() == original_bytes
    assert (prior.start_ns, prior.end_ns, prior.alignment_ns, prior.sensor_id, prior.sequence) == (
            current.start_ns, current.end_ns, current.alignment_ns, current.sensor_id, current.sequence)
    reports = [dict(result.entity_filter_report) for result in (prior, current)]
    for report in reports:
        report.pop('classification_ms', None)
    assert reports[0] == reports[1]
    preserved = all(np.array_equal(prior.points[field], current.points[field])
                    for field in points.dtype.names if field not in GEOMETRY)
    return dict(points=len(points), output_points=len(current.points), source_start_ns=start,
                input_sha256=hashlib.sha256(original_bytes).hexdigest(),
                output_sha256=hashlib.sha256(current.points.tobytes()).hexdigest(),
                bytes_identical=True, metadata_identical=preserved,
                source_span_identical=True, entity_report_identical_except_elapsed=True,
                before={key: stats(value) for key, value in totals['before'].items()},
                after={key: stats(value) for key, value in totals['after'].items()})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--before', type=Path, default=ROOT/'experiments/navigation_runtime/'
                        'source_snapshot/nav_ws/src/d1max_pct_scan/d1max_pct_scan')
    parser.add_argument('--capture', type=Path, default=ROOT/'experiments/'
                        'live_scan_repair_20260927/geometry_20260927_063807')
    parser.add_argument('--repeats', type=int, default=40)
    args = parser.parse_args()
    before, filter_class = baseline(args.before)
    settings = yaml.safe_load((ROOT/'src/d1max_pct_scan/config/entity_self_filter.yaml').read_text())
    settings['model_manifest'] = str(ROOT/'src/d1max_pct_scan/config/d1max_entity_model.json')
    old_filter = filter_class.load(settings)
    capture = json.loads((args.capture/'capture.json').read_text())
    edge = capture['static_transforms']['d1max_loc_base_link->d1max_loc_tracking']
    body = Pose3(tuple(edge['position']), tuple(edge['orientation']))
    results = []
    for filename in ('000_raw_0.npy', '005_raw_1.npy'):
        scan = next(scan for scan in capture['scans'] if scan['file'] == filename)
        points = np.load(args.capture/filename, allow_pickle=False)
        result = pair(before, old_filter, points, scan['start_ns'], body, settings, args.repeats)
        result['input_file'] = str(args.capture/filename)
        results.append(result)
    print(json.dumps(dict(kind='offline_same_raw_points_constructed_pose_brackets_not_live_latency',
        repeats=args.repeats, baseline=str(args.before),
        baseline_sha256={name: hashlib.sha256((args.before/(name+'.py')).read_bytes()).hexdigest()
                         for name in ('ray_projection', 'robot_self_geometry', 'robot_fixed_stance')},
        results=results), indent=2))


if __name__ == '__main__':
    main()
