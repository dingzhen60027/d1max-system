#!/usr/bin/env python3
"""Compare identical PCT endpoint requests and dangerous endpoints offline."""
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


def run(before,after,output,vendor):
    from d1max_pct_planner.tomogram_map import TomogramMap
    from d1max_pct_planner.tomogram_route import TomogramRoute
    from d1max_pct_scan.source_identity import SourceIdentityBridge,identity_builder
    from d1max_pct_scan.static_route_validation import validate_static_route
    requests=json.loads((before/'native_route_audit_lowest_slice.json').read_text())['queries'][:5]
    results,paths={},{}
    for name,directory in [('before',before),('after',after)]:
        raw=yaml.safe_load((directory/'route.yaml').read_text())
        t=TomogramMap(directory/'tomogram.npz',unknown_ceiling_policy=raw['unknown_ceiling_policy'],max_ground_step_m=.17)
        b=SourceIdentityBridge.from_artifacts(directory/'manifest.json')
        builder=identity_builder(directory/'manifest.json',t,b)
        settings=dict(raw['planning'],planning_strategy='native_astar_checked_smooth')
        p=TomogramRoute(t,vendor,**settings)
        records=[]
        for request in requests:
            started=time.monotonic()
            record={k:request[k] for k in ('start_index','goal_index','start_xyz','goal_xyz','start_layer','goal_layer')}
            try:
                result=p.plan(request['start_xyz'],request['goal_xyz'],request['start_layer'],request['goal_layer'])
                result.update(route_type='same_floor',floor='lower',execution_authorized=False)
                checked=validate_static_route(result,t,b,builder=builder,map_version_id='reviewed-cleanup-file-only')
                record.update(success=True,path_quality=result['path_quality'],
                    refinement={key:value for key,value in result['path_refinement'].items()
                                if key in ('applied','reason','continuity','prior_candidate_failures')},
                    support=checked['route_snapshot'].payload()['geometry_evidence']['measured_source_support'])
                paths.setdefault(name,[]).append(dict(xyz=result['path'],layers=result['layer_ids']))
            except Exception as exc:
                record.update(success=False,code=getattr(exc,'code',type(exc).__name__),message=str(exc),details=getattr(exc,'details',{}))
            record['seconds']=time.monotonic()-started
            records.append(record)
            print(json.dumps(dict(map=name,start=record['start_index'],goal=record['goal_index'],success=record['success'],seconds=record['seconds'])),flush=True)
        # Native endpoint validation still rejects actual cost-blocked floor
        # cells and never-observed support. These are from the real tomogram.
        danger=[]
        floor_low,floor_high=b.input_z_band
        wall=np.argwhere(t.ground_known & ~t.allowed & (t.ground>=floor_low)&(t.ground<=floor_high))
        unknown=np.argwhere(~t.ground_known[5])
        for label,indices in [('blocked_measured_surface',wall[::max(1,len(wall)//8)][:8]),
                              ('never_observed_support',np.c_[np.full(min(8,len(unknown)),5),unknown[:8]])]:
            for layer,x,y in indices:
                xy=t.world([x,y]);z=float(t.ground[layer,x,y]) if np.isfinite(t.ground[layer,x,y]) else b.reference_z_m
                try:
                    t.validate_endpoint([*xy,z],int(layer))
                    danger.append(dict(kind=label,rejected=False,layer=int(layer),xyz=[*xy,z]))
                except Exception as exc:
                    danger.append(dict(kind=label,rejected=True,code=getattr(exc,'code',type(exc).__name__),layer=int(layer),xyz=[*xy,z]))
        # A request entering the stair exclusion is rejected by the original
        # source-support owner regardless of which slice happens to be cheap.
        try:
            b.query(np.mean(b.forbidden,axis=0).reshape(1,2))
            danger.append(dict(kind='stair_boundary',rejected=False))
        except ValueError as exc:
            danger.append(dict(kind='stair_boundary',rejected=True,code=str(exc)))
        results[name]=dict(tomogram_sha256=t.sha256,queries=records,dangerous_endpoints=danger)
    output.mkdir(parents=True,exist_ok=True)
    report=dict(same_endpoints_and_source_layers=True,actual_native=True,ros_graph_used=False,
        physical_acceptance=False,execution_authorized=False,results=results)
    (output/'same_input_routes.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'route_paths.json').write_text(json.dumps(paths)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from tools.diagnostics.audit_reviewed_source_cleanup import surfaces
    fig,axes=plt.subplots(1,2,figsize=(12,9),sharex=True,sharey=True)
    for ax,name,directory in zip(axes,('before','after'),(before,after)):
        t,b,g,c,r,s,stats=surfaces(directory)
        ax.scatter(g[:,0],g[:,1],s=3,c='#42cf55',marker='s',linewidths=0)
        ax.scatter(r[:,0],r[:,1],s=3,c='#ed5454',marker='s',linewidths=0)
        for item in paths.get(name,[]):
            xyz=np.array(item['xyz']);ax.plot(xyz[:,0],xyz[:,1],c='#2485e4',lw=1.5)
        ax.set_title(name+' / identical endpoint requests');ax.set_aspect('equal');ax.grid(alpha=.15)
    fig.tight_layout();fig.savefig(output/'same_input_routes.png',dpi=150);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    for key in ('before','after','output','vendor'):parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--child',action='store_true')
    args=parser.parse_args()
    if args.child:run(args.before,args.after,args.output,args.vendor)
    else:
        from d1max_pct_planner.native_runtime import prepare_native_environment
        env=prepare_native_environment(args.vendor)
        subprocess.run([sys.executable,__file__,*sys.argv[1:],'--child'],env=env,check=True)
