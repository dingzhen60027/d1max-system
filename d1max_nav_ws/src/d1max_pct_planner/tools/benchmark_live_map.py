#!/usr/bin/env python3
"""Bounded offline cold/warm profiling of the real layered navigation map.

No ROS, SDK, robot connection or motion commands. Uses the saved route's exact
endpoints and original PCT validation; never replaces a hard map constraint.
"""
import argparse
import cProfile
import gc
from functools import partial
import hashlib
import json
import os
from pathlib import Path
import pstats
import resource
import subprocess
import sys
import time

WS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS / 'src/d1max_pct_planner'))
DEFAULT_MAP = WS / 'maps/processed/sc_pgo_20260923_crossfloor_complete_001'


def child(args):
    import numpy as np
    import yaml
    from d1max_pct_planner.tomogram_map import TomogramMap
    from d1max_pct_planner.crossfloor_preview import CrossfloorPreviewRoute

    root = args.map.resolve()
    raw = yaml.safe_load((root / 'route.yaml').read_text())
    saved = json.loads((root / 'route_001/audit.json').read_text())
    xyz, layers = np.asarray(saved['path_xyz']), np.asarray(saved['layer_ids'])
    if args.case == 'short':
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))]
        end = int(np.searchsorted(arc, 7.))
    elif args.case == 'floor':
        end = saved['segments'][0]['last_index']
    else:
        end = len(xyz)-1
    times = {}
    started = time.perf_counter()
    tomo = TomogramMap(root / 'pct/tomogram.npz',
        unknown_ceiling_policy=raw['unknown_ceiling_policy'],
        max_ground_step_m=raw['limits']['max_ground_step_m'])
    times['map_load_s'] = time.perf_counter()-started
    profile = cProfile.Profile()
    if not args.no_profile:
        profile.enable()
    began = time.perf_counter()
    planner = CrossfloorPreviewRoute(tomo, root / 'route.yaml')
    times['planner_init_s'] = time.perf_counter()-began
    runs = []
    static_validator = bridge = None
    if args.parent_validation:
        sys.path[:0] = [str(WS),str(WS/'src/d1max_pct_scan')]
        from d1max_pct_scan.static_route_validation import StaticRouteValidator,validate_static_route
        from tools.pointcloud_preprocessing.ground_path_bridge import GroundPathBridge
        bridge = GroundPathBridge.from_artifacts(root/'manifest.json')
        static_validator = StaticRouteValidator()
    for _ in range(args.repeat):
        began = time.perf_counter()
        cpu_began = time.process_time()
        result = planner.plan(xyz[0], xyz[end], int(layers[0]), int(layers[end]))
        points = np.asarray(result['path'], dtype=np.float64)
        identities = np.asarray(result['layer_ids'], dtype=np.int64)
        row = dict(elapsed_s=time.perf_counter()-began, points=len(points),
            process_cpu_s=time.process_time()-cpu_began,
            native_map_cache=result.get('native_map_cache'),
            fixed_stair_cache=result.get('fixed_stair_cache'),
            native_thread_count=len(list(Path('/proc/self/task').iterdir())),
            resident_rss_mib=int(Path('/proc/self/statm').read_text().split()[1])
                *os.sysconf('SC_PAGE_SIZE')/1024**2,
            maximum_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.,
            geometry_sha256=hashlib.sha256(points.tobytes()+identities.tobytes()).hexdigest(),
            length_m=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
            validation=tomo.validate_path(points, identities))
        if static_validator is not None:
            submit_began = time.perf_counter()
            static_validator.submit(len(runs)+1,partial(validate_static_route,result,tomo,bridge))
            row['static_submission_s'] = time.perf_counter()-submit_began
            previous = time.perf_counter()
            gaps = []
            deadline = previous+10.
            while True:
                checked = static_validator.poll()
                current = time.perf_counter()
                gaps.append(current-previous)
                previous = current
                if checked is not None:
                    _,validated,error,row['static_validation_s'] = checked
                    if error is not None:
                        raise error
                    row['source_frame_points'] = len(validated['xyz'])
                    break
                if current >= deadline:
                    raise RuntimeError('Static validation exceeded bounded offline test deadline')
                time.sleep(.005)
            row['static_poll_max_gap_s'] = max(gaps)
        runs.append(row)
    if static_validator is not None:
        static_validator.close()
    reuse_checks = []
    if args.verify_reuse:
        # Multiple different goals, including reverse direction, must be
        # identical to a newly initialized native instance on this snapshot.
        for a,b in ((end,0), (0,max(3,end//2)), (0,end)):
            reused = planner.plan(xyz[a],xyz[b],int(layers[a]),int(layers[b]))
            fresh_planner = CrossfloorPreviewRoute(tomo,root/'route.yaml')
            fresh = fresh_planner.plan(xyz[a],xyz[b],int(layers[a]),int(layers[b]))
            reused_points, fresh_points = np.asarray(reused['path']),np.asarray(fresh['path'])
            if (not np.array_equal(reused_points,fresh_points)
                    or reused['layer_ids'] != fresh['layer_ids']):
                raise RuntimeError('Native reuse changed route geometry or layer identities')
            reuse_checks.append(dict(start_index=a,goal_index=b,
                exact_geometry_and_layers=True,points=len(reused_points),
                native_map_cache=reused['native_map_cache']))
            del fresh_planner,fresh
            gc.collect()
    if not args.no_profile:
        profile.disable()
    rows = []
    if not args.no_profile:
        stats = pstats.Stats(profile)
        for (filename, line, name), (primitive, calls, own, cumulative, _callers) in stats.stats.items():
            rows.append(dict(file=filename, line=line, name=name, calls=calls,
                             primitive_calls=primitive, own_s=own, cumulative_s=cumulative))
    print(json.dumps(dict(case=args.case, timings=times, runs=runs,
        effective_cpu_ids=sorted(os.sched_getaffinity(0)),
        effective_nice=os.getpriority(os.PRIO_PROCESS,0),
        numeric_thread_environment={key:os.environ.get(key) for key in
            ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS')},
        wall_s=time.perf_counter()-started, tomogram_sha256=tomo.sha256,
        map_shape=list(tomo.data.shape), no_ros_or_sdk=True, reuse_checks=reuse_checks,
        profiler_enabled=not args.no_profile,
        profile=sorted(rows, key=lambda row:row['cumulative_s'], reverse=True)[:40])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=DEFAULT_MAP)
    parser.add_argument('--case', choices=('short', 'floor', 'crossfloor'), default='short')
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--verify-reuse', action='store_true',
                        help='Compare three different/reverse goals against fresh native maps')
    parser.add_argument('--no-profile', action='store_true',help='Measure true wall time without cProfile overhead')
    parser.add_argument('--parent-validation',action='store_true',
                        help='Also measure asynchronous original-frame static proof and polling responsiveness')
    parser.add_argument('--cpu-budget',action='store_true',
                        help='Constrain only the benchmark child to the configured planner CPU/nice budget')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--compare', type=Path, help='Require exact geometry and validation equivalence to this report')
    parser.add_argument('--child', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.repeat <= 3:
        parser.error('repeat must be between 1 and 3')
    if args.child:
        child(args)
        return
    import yaml
    from d1max_pct_planner.native_runtime import prepare_native_environment
    cfg = yaml.safe_load((args.map / 'route.yaml').read_text())
    env = prepare_native_environment(cfg['vendor_root'], os.environ)
    command = [sys.executable, __file__, '--child', '--map', str(args.map),
               '--case', args.case, '--repeat', str(args.repeat)]
    if args.verify_reuse:
        command.append('--verify-reuse')
    if args.no_profile:
        command.append('--no-profile')
    if args.parent_validation:
        command.append('--parent-validation')
    if args.cpu_budget:
        # Apply before interpreter/numeric imports, to every future child thread.
        # The invoking shell and unrelated ROS/robot processes remain untouched.
        from d1max_pct_planner.compute_budget import load_budget
        budget = load_budget(env)
        allowed = set(os.sched_getaffinity(0))
        cpus = budget['cpu_ids'] or sorted(allowed)[:budget['max_cpu_cores']]
        if not cpus or not set(cpus).issubset(allowed):
            raise ValueError('Benchmark CPU budget is outside the inherited cpuset')
        lower_priority = max(0, budget['nice']-os.getpriority(os.PRIO_PROCESS,0))
        command = ['taskset', '--cpu-list', ','.join(map(str,cpus)),
                   'nice', '-n', str(lower_priority), *command]
    result = subprocess.run(command, env=env, text=True, capture_output=True, timeout=150)
    if result.returncode:
        raise RuntimeError(result.stderr[-3000:] + result.stdout[-2000:])
    report = json.loads(next(line for line in reversed(result.stdout.splitlines()) if line.startswith('{')))
    if args.compare:
        baseline = json.loads(args.compare.read_text())
        if (baseline['case'] != report['case'] or
                baseline['tomogram_sha256'] != report['tomogram_sha256']):
            raise RuntimeError('Baseline uses a different map or endpoint case')
        expected = baseline['runs'][0]
        for run in report['runs']:
            if (run['geometry_sha256'] != expected['geometry_sha256'] or
                    run['validation'] != expected['validation']):
                raise RuntimeError('Route geometry/layer identities or validation changed')
        report['comparison'] = dict(exact_geometry_and_validation=True,
            baseline=str(args.compare.resolve()), baseline_wall_s=baseline['wall_s'],
            cold_load_init_plan_s=sum(report['timings'].values())+report['runs'][0]['elapsed_s'],
            warm_plan_s=[run['elapsed_s'] for run in report['runs'][1:]])
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key:value for key,value in report.items() if key != 'profile'}))
    for row in report['profile'][:15]:
        print(f"{row['cumulative_s']:8.3f}s cumulative {row['own_s']:8.3f}s own {row['calls']:7d} {row['file']}:{row['line']} {row['name']}")


if __name__ == '__main__':
    main()
