#!/usr/bin/env python3
"""Exercise production native A*, dangerous corners and actual-map polylines."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src/d1max_pct_planner')]


def run(args):
    from d1max_pct_planner.planner_core import TomogramPlanner
    from d1max_pct_planner.tomogram_map import TomogramMap
    from d1max_pct_planner.tomogram_route import TomogramRoute
    from d1max_pct_planner.corridor_refinement import refine_corridor
    loader=TomogramPlanner(args.vendor_root)
    cases=[]
    for name in ('open_diagonal','blocked_diagonal_corner','blocked_gateway',
                 'overlap_thin_slice_dead_end','same_xy_other_floor'):
        cost=np.full((5,5),50.,dtype=float);cost[1:4,1:4]=0.
        start,goal=np.array([0,1,1],np.int32),np.array([0,3,3],np.int32)
        gateway=np.zeros_like(cost)
        if name=='blocked_diagonal_corner':
            cost[:]=50.;cost[1,1]=cost[2,2]=0.;goal=np.array([0,2,2],np.int32)
        elif name=='blocked_gateway':
            cost[:,2]=25.;gateway[:,2]=2.;goal=np.array([0,3,1],np.int32)
        layers=1;heights=np.zeros_like(cost)
        if name=='overlap_thin_slice_dead_end':
            layers=2;cost=np.full((10,5),50.);cost[7,1:4]=0.;cost[2,1]=0.
            heights=np.zeros_like(cost);gateway=np.zeros_like(cost);gateway[7,1]=-2.
            start=np.array([1,1,2],np.int32);goal=np.array([1,3,2],np.int32)
        elif name=='same_xy_other_floor':
            layers=2;cost=np.full((10,5),50.);cost[2,1:3]=0.;cost[7,2:4]=0.
            heights=np.zeros_like(cost);heights[5:]=3.;gateway=np.zeros_like(cost);gateway[2,2]=2.
            start=np.array([0,1,2],np.int32);goal=np.array([1,3,2],np.int32)
        finder=loader.a_star.Astar();finder.init(20.,layers,.1,1.,cost,heights,gateway)
        found=finder.search(start,goal)
        cases.append(dict(case=name,found=found,expected=name in ('open_diagonal','overlap_thin_slice_dead_end')))
    p=Path(args.directory);settings=yaml.safe_load((p/'route.yaml').read_text())
    t=TomogramMap(p/'tomogram.npz',max_ground_step_m=.17,unknown_ceiling_policy=settings['unknown_ceiling_policy'])
    route=TomogramRoute(t,args.vendor_root,max_heading_rate=1.,astar_cost_weight=1.,optimizer_cost_margin=8.)
    queries=json.loads((p/'native_route_audit_lowest_slice.json').read_text())['queries']
    records=[]
    for q in queries:
        a=np.r_[q['start_layer'],route.planner._position_index(q['start_xyz'][:2])].astype(np.int32)
        b=np.r_[q['goal_layer'],route.planner._position_index(q['goal_xyz'][:2])].astype(np.int32)
        started=time.monotonic();found=route.planner.planner.plan(a,b,False)
        record=dict(start=q['start_index'],goal=q['goal_index'],found=found)
        if found:
            grid=np.asarray(route.planner.planner.get_path_finder().get_result_matrix())
            layers=grid[:,0].astype(int);indices=grid[:,[1,2]].astype(int)
            xyz=np.c_[t.center+(indices-t.offset)*t.resolution,t.ground[layers,indices[:,0],indices[:,1]]]
            xyz=np.vstack([q['start_xyz'],xyz,q['goal_xyz']]);layers=np.r_[q['start_layer'],layers,q['goal_layer']]
            try:
                record['validation']=t.validate_path(xyz,layers)
                record.update(valid=True,points=len(xyz),layers=sorted(set(layers.tolist())))
                refined=refine_corridor(t,xyz,layers,corner_cut_m=1.5)
                record['refinement']={k:v for k,v in refined.items() if k not in ('path','layer_ids','segments')}
            except Exception as e:
                record.update(valid=False,error=str(e),details=getattr(e,'details',{}))
        record['seconds']=time.monotonic()-started;records.append(record)
    report=dict(vendor_root=str(Path(args.vendor_root).resolve()),native=route.native_runtime,
        tomogram_sha256=t.sha256,physical_acceptance=False,negative_cases=cases,actual_map=records)
    Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(negative_cases=cases,actual_map=[{k:r.get(k) for k in ('start','goal','found','valid','seconds')} for r in records])))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('vendor-root','directory','output'):p.add_argument('--'+name,required=True)
    p.add_argument('--child',action='store_true');args=p.parse_args()
    if args.child:run(args)
    else:
        from d1max_pct_planner.native_runtime import prepare_native_environment
        subprocess.run([sys.executable,__file__,*sys.argv[1:],'--child'],check=True,
            env=prepare_native_environment(args.vendor_root))
