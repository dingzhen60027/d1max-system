#!/usr/bin/env python3
"""Read-only, offline timing of the exact live PCT map and short/cross-floor goals.

No ROS graph, robot SDK, commands, or service is started. Each case uses a
fresh interpreter to capture the per-goal native worker's cold-start cost.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


WS = Path(__file__).resolve().parents[3]
ARTIFACTS = WS / 'maps/processed/sc_pgo_20260923_crossfloor_complete_001'
for source in (WS, WS / 'src/d1max_pct_planner', WS / 'src/d1max_pct_scan'):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))


def child(case):
    import numpy as np
    from d1max_pct_planner.crossfloor_preview import CrossfloorPreviewRoute
    from d1max_pct_planner.tomogram_map import TomogramMap
    from tools.pointcloud_preprocessing.ground_path_bridge import GroundPathBridge
    from d1max_pct_scan.live_global_contract import labels_for_result

    route_file = ARTIFACTS / 'route.yaml'
    audit = json.loads((ARTIFACTS / 'route_001/audit.json').read_text())
    cfg = __import__('yaml').safe_load(route_file.read_text())
    map_options = dict(minimum_headroom_m=cfg.get('minimum_headroom_m'),
                       unknown_ceiling_policy=cfg.get('unknown_ceiling_policy', 'allow_unobserved'),
                       max_ground_step_m=cfg['limits']['max_ground_step_m'])
    route_xyz = np.asarray(audit['path_xyz'], float)
    route_layers = np.asarray(audit['layer_ids'], int)
    if case == 'short':
        distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route_xyz[:, :2], axis=0), axis=1))]
        index = int(np.searchsorted(distance, 3.))
        if index >= audit['segments'][0]['last_index']:
            raise RuntimeError('short goal left the ground-floor leg')
    else:
        index = len(route_xyz)-1
    start, goal = route_xyz[0], route_xyz[index]
    first_layer, last_layer = int(route_layers[0]), int(route_layers[index])
    times = {}
    began = time.perf_counter()
    bridge = GroundPathBridge.from_artifacts(ARTIFACTS / 'manifest.json')
    times['bridge_init_s'] = time.perf_counter()-began
    began = time.perf_counter()
    tomogram = TomogramMap(ARTIFACTS / 'pct/tomogram.npz', **map_options)
    times['tomogram_load_s'] = time.perf_counter()-began
    began = time.perf_counter()
    planner = CrossfloorPreviewRoute(tomogram, route_file)
    times['planner_init_s'] = time.perf_counter()-began
    began = time.perf_counter()
    result = planner.plan(start, goal, first_layer, last_layer)
    times['native_plan_s'] = time.perf_counter()-began
    began = time.perf_counter()
    same_floor = ({'lower': 'floor1', 'upper': 'floor2'}.get(result.get('floor'))
                  if result.get('route_type') == 'same_floor' else None)
    points, labels = labels_for_result(result, same_floor=same_floor)
    layers = np.asarray(result['layer_ids'], int)
    tomogram.validate_path(points, layers)
    times['parent_validate_s'] = time.perf_counter()-began
    began = time.perf_counter()
    converted = bridge.to_localization_ground(points, labels)
    times['ground_bridge_s'] = time.perf_counter()-began
    print(json.dumps({'case': case, 'status': 'passed', 'timings': times,
                      'route_type': result.get('route_type', 'crossfloor'), 'points': len(points),
                      'source_frame': bridge.source_frame,
                      'max_support_distance_m': converted.diagnostics['maximum_support_distance_m']}))


def _unresponsive_test_child(connection):
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    connection.send('ready')
    connection.close()
    time.sleep(10.)


def cancel_benchmark():
    """Compare worst-case old join to timer-reaped cleanup on owned test PIDs."""
    from d1max_pct_scan.live_global_worker_cleanup import RetiredChildren
    context = multiprocessing.get_context('spawn')
    timings = {}
    for variant in ('old_sync', 'new_nonblocking'):
        parent, reply = context.Pipe(duplex=False)
        child = context.Process(target=_unresponsive_test_child, args=(reply,), daemon=True)
        child.start()
        reply.close()
        try:
            if not parent.poll(5.) or parent.recv() != 'ready':
                raise RuntimeError('test child failed to enter SIGTERM-resistant state')
            began = time.perf_counter()
            if variant == 'old_sync':
                parent.close()
                child.terminate()
                child.join(timeout=.5)
                if child.is_alive():
                    child.kill()
                    child.join(timeout=.5)
            else:
                cleanup = RetiredChildren(terminate_grace_s=.5)
                cleanup.retire(child, parent)
            timings[variant + '_callback_s'] = time.perf_counter()-began
            if variant == 'new_nonblocking':
                time.sleep(.55)
                deadline = time.monotonic() + .5
                while cleanup.reap() and time.monotonic() < deadline:
                    time.sleep(.01)
                timings['new_reaped_after_grace'] = not child.is_alive()
        finally:
            parent.close()
            if child.is_alive():
                child.kill()
            child.join(timeout=1.)
    print(json.dumps({'case': 'cancel_unresponsive_owned_test_child', 'status': 'passed',
                      'robot_or_ros_used': False, 'timings': timings}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--child', choices=('short', 'crossfloor'))
    parser.add_argument('--case', choices=('short', 'crossfloor', 'both', 'cancel'), default='both')
    args = parser.parse_args()
    if args.child:
        child(args.child)
        return
    if args.case == 'cancel':
        cancel_benchmark()
        return
    from d1max_pct_planner.native_runtime import prepare_native_environment
    cfg = __import__('yaml').safe_load((ARTIFACTS / 'route.yaml').read_text())
    environment = prepare_native_environment(cfg['vendor_root'], os.environ)
    environment['PYTHONPATH'] = os.pathsep.join((str(WS), str(WS/'src/d1max_pct_planner'),
        str(WS/'src/d1max_pct_scan'), environment.get('PYTHONPATH', '')))
    environment.update(OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    for name in ('short', 'crossfloor') if args.case == 'both' else (args.case,):
        started = time.perf_counter()
        result = subprocess.run([sys.executable, __file__, '--child', name],
                                env=environment, cwd=WS, text=True,
                                capture_output=True, timeout=180)
        if result.returncode:
            print(json.dumps({'case': name, 'status': 'failed', 'error': result.stderr[-2000:],
                              'wall_s': time.perf_counter()-started}))
        else:
            lines = [line for line in result.stdout.splitlines() if line.startswith('{')]
            if not lines:
                print(json.dumps({'case': name, 'status': 'missing_result',
                    'stdout_tail': result.stdout[-2000:], 'stderr_tail': result.stderr[-2000:],
                    'wall_s': time.perf_counter()-started}))
                continue
            output = json.loads(lines[-1])
            output['wall_s'] = time.perf_counter()-started
            print(json.dumps(output))


if __name__ == '__main__':
    main()
