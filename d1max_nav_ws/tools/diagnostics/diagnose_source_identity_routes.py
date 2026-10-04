#!/usr/bin/env python3
"""Bounded file-only PCT optimizer/connected-component evidence."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/d1max_pct_planner')]


def run(directory):
    from scipy.ndimage import label
    from d1max_pct_planner.tomogram_map import TomogramMap
    from d1max_pct_planner.tomogram_route import TomogramRoute
    p=Path(directory).resolve()
    t=TomogramMap(p/'tomogram.npz',max_ground_step_m=.17)
    queries=json.loads((p/'native_route_audit_lowest_slice.json').read_text())['queries']
    components,count=label(t.allowed.any(axis=0),np.ones((3,3)))
    connectivity=[]
    for q in queries:
        a,b=(tuple(t.index(q[k+'_xyz'][:2])) for k in ('start','goal'))
        connectivity.append(dict(start=q['start_index'],goal=q['goal_index'],
            start_component=int(components[a]),goal_component=int(components[b]),
            connected_even_ignoring_slice=bool(components[a] and components[a]==components[b])))
    records=[]
    for interval in (10,5,2,1):
        planner=TomogramRoute(t,ROOT/'src/pct_planner_vendor',max_heading_rate=1.,
            astar_cost_weight=1.,optimizer_cost_margin=8.,optimizer_sample_interval=interval,
            path_refinement='visibility_c2')
        for q in queries[:2]:
            before=time.monotonic()
            record=dict(start=q['start_index'],goal=q['goal_index'],sample_interval=interval)
            try:
                result=planner.plan(q['start_xyz'],q['goal_xyz'],q['start_layer'],q['goal_layer'])
                record.update(success=True,points=len(result['path']),length_m=result['length_m'],
                    layers=sorted(set(result['layer_ids'])),path_quality=result['path_quality'])
            except Exception as e:
                record.update(success=False,error=str(e),code=getattr(e,'code',type(e).__name__),
                    details=getattr(e,'details',{}))
            record['seconds']=time.monotonic()-before;records.append(record)
            print(json.dumps(record),flush=True)
    report=dict(tomogram_sha256=t.sha256,hard_mask_changed=False,components=count,
        connectivity=connectivity,optimizer_sweep=records)
    (p/'native_failure_diagnosis.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--directory',required=True);parser.add_argument('--child',action='store_true')
    args=parser.parse_args()
    if args.child:run(args.directory)
    else:
        from d1max_pct_planner.native_runtime import prepare_native_environment
        env=prepare_native_environment(ROOT/'src/pct_planner_vendor')
        subprocess.run([sys.executable,__file__,'--directory',args.directory,'--child'],env=env,check=True)
