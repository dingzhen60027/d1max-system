"""Temporary integration regression: proven floor cleanup + protected stairs."""
import os
import sys
import subprocess
import json
from pathlib import Path
import numpy as np
import yaml
sys.path.insert(0, '/home/dndx/d1max_nav_ws')
sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
if os.environ.get('SWEEP_CHILD') != '1':
    from d1max_pct_planner.native_runtime import prepare_native_environment
    env = prepare_native_environment('/home/dndx/d1max_nav_ws/src/pct_planner_vendor')
    env['SWEEP_CHILD'] = '1'
    raise SystemExit(subprocess.run([sys.executable, __file__, *sys.argv[1:]], env=env).returncode)
import native_sweep as ns
from tools.pointcloud_preprocessing.pcd_io import read_pcd
from d1max_pct_planner.crossfloor_route import plan_crossfloor

root = Path(__file__).resolve().parent
src = read_pcd(ns.PCD)
base = Path('/home/dndx/d1max_nav_ws/maps/processed/sc_pgo_20260923_multifloor_conditioned_v1')
conditioned = read_pcd(base/'processed_map.pcd')
audit = np.load(base/'audit/floor_evidence.npz', allow_pickle=False)
ids = audit['source_indices']
mode = sys.argv[1] if len(sys.argv) > 1 else 'W'
if mode in ('W', 'Y'):
    args = (10,20,0) if mode == 'W' else (10,20,.12,.05)
    keep = ns.visibility_keep(src.xyz, *args)
else:
    keep = np.load(root/mode, allow_pickle=False)['keep']
points = np.concatenate([conditioned.xyz[keep[ids]], np.load(root/'upper_floor_measured_restore.npz')['points']])
tomo = ns.build(points, .1, .5, ns.CROP, {'slope_max_rad':.60, 'safe_margin':.10, 'inflation':.10})
np.savez_compressed(root/'cache_composed.npz', data=tomo.data, resolution=tomo.resolution,
                    center=tomo.center, slice_h0=tomo.slice_h0, slice_dh=tomo.slice_dh,
                    selected_source_layers=tomo.source_layers)
config = yaml.safe_load(open(ns.ROUTE_CFG))
for key, anchor in config['anchors'].items():
    chosen = ns.snap_anchor(tomo, anchor['xyz'][:2], ns.EXPECTED[key])
    config['anchors'][key] = {k:chosen[k] for k in ('xyz','layer_id')}
try:
    result = plan_crossfloor(config,tomo)
    (root/'composed_route.json').write_text(json.dumps(result,indent=2))
    print('COMPLETE', result['length_m'], result['stair_profile'], flush=True)
except Exception as exc:
    print('FAIL',getattr(exc,'code',''),str(exc),getattr(exc,'details',None),flush=True)
    print('CAUSE',getattr(exc.__cause__,'details',None),flush=True)
