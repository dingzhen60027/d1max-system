"""Read-only offline timing: current map/algorithms, no ROS or SDK.

Method wrappers collect nested wall time, without cProfile's per-call overhead.
Only reports are written; no production code or map is modified.
"""
from collections import defaultdict
from functools import wraps
import hashlib
import ctypes
import json
import os
from pathlib import Path
import resource
import subprocess
import sys
import time

WS = Path('/home/dndx/d1max_nav_ws')
OUT = Path(os.environ.get('PCT_TIMING_OUTPUT_DIR', str(Path(__file__).resolve().parent)))
OUT.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(WS / 'src/d1max_pct_planner'))

if '--child' not in sys.argv:
    from d1max_pct_planner.native_runtime import prepare_native_environment
    env = prepare_native_environment(str(WS / 'src/pct_planner_vendor'), os.environ)
    env.update(OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    result = subprocess.run([sys.executable, __file__, '--child'], env=env,
                            capture_output=True, text=True, timeout=120)
    (OUT / 'native_stdout.log').write_text(result.stdout + '\n' + result.stderr)
    if result.returncode:
        raise RuntimeError(result.stderr[-4000:] + result.stdout[-2000:])
    print(result.stdout.split('REPORT:')[-1])
    sys.exit(0)

import numpy as np
import yaml
from d1max_pct_planner import planner_core, tomogram_route, crossfloor_route
from d1max_pct_planner import crossfloor_preview, corridor_refinement
from d1max_pct_planner.tomogram_map import TomogramMap

phase = 'initialize'
rows, stack = [], []

def measure(name, fn, *args, **kwargs):
    started = time.perf_counter()
    frame = [0.0]
    stack.append(frame)
    try:
        return fn(*args, **kwargs)
    finally:
        elapsed = time.perf_counter() - started
        stack.pop()
        if stack:
            stack[-1][0] += elapsed
        rows.append(dict(phase=phase, name=name, total_s=elapsed,
                         self_s=elapsed-frame[0]))

def wrap(owner, method, name):
    original = getattr(owner, method)
    @wraps(original)
    def timed(*args, **kwargs):
        return measure(name, original, *args, **kwargs)
    setattr(owner, method, timed)

class NativeProxy:
    def __init__(self, native):
        self.native = native
    def __getattr__(self, name):
        return getattr(self.native, name)
    def plan(self, start, goal, optimize):
        name = 'native_search_plus_gpmp' if optimize else 'native_precheck_search'
        return measure(name, self.native.plan, start, goal, optimize)
    def get_path_finder(self):
        return measure('binding_get_path_finder', self.native.get_path_finder)
    def get_trajectory_optimizer_wnoj(self):
        return measure('binding_get_optimizer', self.native.get_trajectory_optimizer_wnoj)

original_load = planner_core.TomogramPlanner.load_payload
def load_and_wrap(self, *args, **kwargs):
    result = measure('native_map_load', original_load, self, *args, **kwargs)
    self.planner = NativeProxy(self.planner)
    return result
planner_core.TomogramPlanner.load_payload = load_and_wrap
wrap(planner_core.TomogramPlanner, 'plan', 'python_native_wrapper')
wrap(TomogramMap, 'validate_path', 'validate_path')
wrap(tomogram_route, 'expand_native_curve', 'expand_quintic')
wrap(tomogram_route.TomogramRoute, 'plan', 'segment_plan')
wrap(corridor_refinement, 'refine_corridor', 'corridor_refinement')
wrap(crossfloor_preview.CrossfloorPreviewRoute, '_resources', 'map_resources')
wrap(crossfloor_route, '_check_leg', 'check_leg')

root = WS / 'maps/processed/sc_pgo_20260923_crossfloor_complete_001'
config = yaml.safe_load((root / 'route.yaml').read_text())
record = json.loads((root / 'route_001/audit.json').read_text())
xyz, layers = np.asarray(record['path_xyz']), np.asarray(record['layer_ids'])
tomogram = measure('map_file_load', TomogramMap, root / 'pct/tomogram.npz',
    unknown_ceiling_policy=config['unknown_ceiling_policy'],
    max_ground_step_m=config['limits']['max_ground_step_m'])
planner = measure('coordinator_init', crossfloor_preview.CrossfloorPreviewRoute,
                  tomogram, root / 'route.yaml')
arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))]
cases = [('crossfloor_cold', len(xyz)-1), ('crossfloor_warm', len(xyz)-1),
         ('short_warm', int(np.searchsorted(arc, 7.)))]
runs = []
for phase, end in cases:
    ctypes.CDLL(None).fflush(None)
    print('BEGIN:', phase, flush=True)
    result = measure('whole_plan', planner.plan, xyz[0], xyz[end],
                     int(layers[0]), int(layers[end]))
    points = np.asarray(result['path'], dtype=np.float64)
    identities = np.asarray(result['layer_ids'], dtype=np.int64)
    times = defaultdict(lambda: dict(calls=0, total_s=0., self_s=0.))
    for row in rows:
        if row['phase'] != phase:
            continue
        bucket = times[row['name']]
        bucket['calls'] += 1
        for key in ('total_s', 'self_s'):
            bucket[key] += row[key]
    runs.append(dict(case=phase, points=len(points), length_m=result['length_m'],
        geometry_sha256=hashlib.sha256(points.tobytes()+identities.tobytes()).hexdigest(),
        native_map_cache=result['native_map_cache'], timings=dict(times)))
    ctypes.CDLL(None).fflush(None)
    print('END:', phase, flush=True)

report = dict(map_shape=list(tomogram.data.shape), resolution_m=tomogram.resolution,
    layers=tomogram.num_layers if hasattr(tomogram, 'num_layers') else len(tomogram.source_layers),
    tomogram_sha256=tomogram.sha256, initialization=[r for r in rows if r['phase']=='initialize'],
    runs=runs, raw_timings=rows,
    maximum_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
    scope='offline_same_map_no_ROS_no_SDK_no_changes_to_algorithm',
    timing_note='nested total_s values overlap; self_s values are exclusive; lightweight wrappers, not cProfile')
(OUT / 'report.json').write_text(json.dumps(report, indent=2)+'\n')
print('REPORT:', json.dumps({k:v for k,v in report.items() if k!='raw_timings'}), flush=True)
