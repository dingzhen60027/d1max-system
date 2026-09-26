"""Read-only native coordinator checks, independent of the active RViz session."""
import json
import os
from pathlib import Path
import subprocess
import sys
sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
from d1max_pct_planner.native_runtime import prepare_native_environment
if os.environ.get('VERIFY_PREVIEW_NATIVE') != '1':
    env = prepare_native_environment('/home/dndx/d1max_nav_ws/src/pct_planner_vendor')
    env['VERIFY_PREVIEW_NATIVE'] = '1'
    raise SystemExit(subprocess.run([sys.executable,__file__],env=env).returncode)
import numpy as np
import yaml
from d1max_pct_planner.crossfloor_preview import CrossfloorPreviewRoute, restore_crossfloor
from d1max_pct_planner.tomogram_map import TomogramMap
root = Path('/home/dndx/d1max_nav_ws/maps/processed/sc_pgo_20260923_crossfloor_complete_001')
cfg = yaml.safe_load((root/'route.yaml').read_text())
grid = TomogramMap(root/'pct/tomogram.npz', max_ground_step_m=.17)
route = CrossfloorPreviewRoute(grid,root/'route.yaml')
report = {}
for name, a, b in [('down',cfg['anchors']['goal'],cfg['anchors']['start']),
                   ('same_floor',cfg['anchors']['start'],cfg['anchors']['entry'])]:
    result = route.plan(a['xyz'],b['xyz'],a['layer_id'],b['layer_id'])
    assert np.array_equal(result['path'][0],a['xyz'])
    assert np.array_equal(result['path'][-1],b['xyz'])
    report[name] = {k:result.get(k) for k in ('length_m','direction','stair_profile','route_type','layer_transition_count')}
    if name == 'down':
        assert np.array_equal(result['anchors']['start']['xyz'],a['xyz'])
        assert isinstance(result['layer_transitions'], list)
        assert result['stair_profile']['direction'] == 'down'
result = restore_crossfloor(grid,root/'route.yaml',root/'route_001/audit.json')
report['restore'] = {'length_m':result['length_m'],'checked_cells':result['checked_cells']}
Path(__file__).with_suffix('.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report),flush=True)
