#!/usr/bin/env python3
"""File-only native PCT audit; no ROS graph, fake floor or execution permission."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src/d1max_pct_planner'),str(ROOT/'src/d1max_pct_scan')]


def audit(directory,vendor_root=None,output_name='native_route_audit_lowest_slice.json',sample_interval=None,strategy=None,repetitions=1,reverse=False):
    from scipy.spatial import cKDTree
    from d1max_pct_planner.tomogram_map import TomogramMap
    from d1max_pct_planner.singlefloor_route import SinglefloorRoute
    from d1max_pct_scan.source_identity import SourceIdentityBridge
    from d1max_pct_scan.source_route import SourceRouteBuilder,digest_file
    from d1max_pct_scan.static_route_validation import validate_static_route
    directory=Path(directory).resolve()
    route_settings=yaml.safe_load((directory/'route.yaml').read_text())
    tomo=TomogramMap(directory/'tomogram.npz',max_ground_step_m=.17,
        unknown_ceiling_policy=route_settings['unknown_ceiling_policy'])
    bridge=SourceIdentityBridge.from_artifacts(directory/'manifest.json')
    builder=SourceRouteBuilder.from_artifacts(directory/'manifest.json',tomo,bridge)
    if vendor_root:
        from d1max_pct_planner.tomogram_route import TomogramRoute
        from d1max_pct_planner.singlefloor_route import load_config
        _,config=load_config(directory/'route.yaml',tomo)
        settings=dict(config['planning'])
        if sample_interval is not None:settings['optimizer_sample_interval']=sample_interval
        if strategy is not None:settings['planning_strategy']=strategy
        planner=TomogramRoute(tomo,vendor_root,**settings)
    else:
        planner=SinglefloorRoute(tomo,directory/'route.yaml')
        planner.warmup_resources()
    layer,x,y=np.where(tomo.allowed)
    points=np.c_[tomo.center[0]+(x-tomo.offset[0])*tomo.resolution,
        tomo.center[1]+(y-tomo.offset[1])*tomo.resolution,tomo.ground[layer,x,y]]
    tree=cKDTree(points[:,:2])
    trajectory=ROOT/'maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/sc_pgo/optimized_poses.txt'
    poses=np.loadtxt(trajectory)[:,[3,7,11]]
    anchors=[]
    for index in (0,100,250,400,550,700):
        d,indices=tree.query(poses[index,:2],k=64)
        candidates=[i for distance,i in zip(d,indices) if distance<=.6]
        if candidates:
            selected=min(candidates,key=lambda i:(round(float(np.linalg.norm(points[i,:2]-poses[index,:2])),8),
                int(layer[i]),float(tomo.cost[layer[i],x[i],y[i]])))
            anchors.append((index,int(layer[selected]),points[selected],float(np.linalg.norm(points[selected,:2]-poses[index,:2]))))
    queries=[]
    pairs=list(zip(anchors[:-1],anchors[1:]))
    # A local query across a genuine multi-slice overlap is included in
    # addition to historical-route waypoints; no mask/cost is changed.
    diff=tomo.cost[1:]-tomo.cost[:-1]
    equal=np.abs(tomo.ground[1:]-tomo.ground[:-1])<.1
    gateways=np.argwhere(equal & (np.abs(diff)>8) & tomo.allowed[1:] & tomo.allowed[:-1])
    if len(gateways):
        l,ix,iy=map(int,gateways[len(gateways)//2])
        for dx,dy in ((4,0),(-4,0),(0,4),(0,-4)):
            if tomo.contains((ix+dx,iy+dy)) and tomo.allowed[l,ix+dx,iy+dy]:
                p=np.r_[tomo.world((ix+dx,iy+dy)),tomo.ground[l,ix+dx,iy+dy]]
                q=np.r_[tomo.world((ix,iy)),tomo.ground[l+1,ix,iy]]
                pairs.append((('gateway_start',l,p,0.),('gateway_goal',l+1,q,0.)))
                break
    if reverse:pairs=[(b,a) for a,b in pairs]
    for repeat in range(repetitions):
      for a,b in pairs:
        began=time.monotonic()
        record=dict(repeat=repeat,start_index=a[0],goal_index=b[0],start_xyz=a[2].tolist(),goal_xyz=b[2].tolist(),
            start_layer=a[1],goal_layer=b[1],selection_xy_offsets_m=[a[3],b[3]])
        try:
            result=planner.plan(a[2],b[2],a[1],b[1])
            result.update(route_type='same_floor',floor='lower',execution_authorized=False)
            checked=validate_static_route(result,tomo,bridge,builder=builder,map_version_id='0923-floor1-source-identity-v1')
            snapshot=checked['route_snapshot'].payload()
            record.update(success=True,layers=sorted(set(result['layer_ids'])),points=len(result['path']),
                execution_eligible=snapshot['execution_eligible'],eligibility_reason=snapshot['eligibility_reason'],
                support=snapshot['geometry_evidence']['measured_source_support'],
                algorithm=result['algorithm'],planner_elapsed_s=result['elapsed_s'],
                path_quality=result['path_quality'],route_hash=checked['route_snapshot'].route_hash,
                path_refinement={k:v for k,v in result.get('path_refinement',{}).items() if k not in ('segments',)},
                collision_validation={k:result.get(k) for k in ('checked_layer_cells','cost_threshold','unobserved_ceiling_cells')})
        except Exception as exc:
            record.update(success=False,error=str(exc),code=getattr(exc,'code',type(exc).__name__),
                          details=getattr(exc,'details',{}))
        record['seconds']=time.monotonic()-began
        queries.append(record)
        print(json.dumps(record),flush=True)
    report=dict(source_map_sha256=bridge.manifest['source_sha256'],tomogram_sha256=tomo.sha256,
        trajectory_sha256=digest_file(trajectory),actual_native=True,ros_graph_used=False,
        physical_acceptance=False,gateway_cells=len(gateways),queries=queries)
    report['vendor_root']=str(vendor_root or ROOT/'src/pct_planner_vendor')
    report['sample_interval']=sample_interval
    report['strategy']=strategy or 'native_gpmp'
    report['repetitions']=repetitions
    report['reverse']=reverse
    (directory/output_name).write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--directory',required=True); parser.add_argument('--child',action='store_true')
    parser.add_argument('--vendor-root',type=Path);parser.add_argument('--output-name',default='native_route_audit_lowest_slice.json')
    parser.add_argument('--sample-interval',type=int)
    parser.add_argument('--strategy',choices=['native_gpmp','native_astar_checked_smooth'])
    parser.add_argument('--repetitions',type=int,default=1)
    parser.add_argument('--reverse',action='store_true')
    args=parser.parse_args()
    if not 1<=args.repetitions<=10:parser.error('repetitions must be in [1,10]')
    if args.child: audit(args.directory,args.vendor_root,args.output_name,args.sample_interval,args.strategy,args.repetitions,args.reverse)
    else:
        from d1max_pct_planner.native_runtime import prepare_native_environment
        env=prepare_native_environment(args.vendor_root or ROOT/'src/pct_planner_vendor')
        subprocess.run([sys.executable,__file__,*sys.argv[1:],'--child'],env=env,check=True)
