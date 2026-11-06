#!/usr/bin/env python3
"""ROS/SDK-free check of the selected production map, editor and PCT routes.

Runs native code in its verified GTSAM loader environment. This checks global
planning artifacts only, not BT transport, local planning or robot arrival.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


def check(session_path):
    import numpy as np
    import yaml
    from d1max_pct_scan.navigation_map_package import validate_package, package_version
    from d1max_pct_scan.live_goal_selection import LiveGoalSelection
    from d1max_pct_scan.live_map_geometry import build_layers
    from d1max_pct_scan.source_route import SourceRouteBuilder
    from d1max_pct_scan.static_route_validation import validate_static_route
    from d1max_pct_planner.crossfloor_preview import CrossfloorPreviewRoute
    session=json.loads(session_path.read_text())
    package=validate_package(Path(session['planning_manifest']).parent)
    editor=LiveGoalSelection.from_session(session)
    assert editor.available_floors==('floor1','floor2')
    assert session['map_capabilities']==package['capabilities']
    assert session['version_id']==package_version(package)
    tomogram,bridge=package['tomogram'],package['bridge']
    raw=yaml.safe_load(Path(session['crossfloor_route_config']).read_text())
    anchors=raw['anchors']
    pairs=[('up','start','goal'),('down','goal','start'),('floor2','exit','goal')]
    planner=CrossfloorPreviewRoute(tomogram,session['crossfloor_route_config'])
    builder=SourceRouteBuilder.from_artifacts(session['planning_manifest'],tomogram,bridge)
    records=[]
    for name,first,last in pairs:
        start,goal=anchors[first],anchors[last]
        floor='floor1' if last=='start' else 'floor2'
        source=bridge.to_localization_ground([goal['xyz']],[floor]).xyz[0]
        editor.set_floor(floor)
        draft=editor.edit(source)
        assert draft['valid'] and draft['floor_id']==floor
        assert np.allclose(draft['xyz'],source,atol=1e-6)
        began=time.monotonic()
        result=planner.plan(start['xyz'],goal['xyz'],start['layer_id'],goal['layer_id'])
        checked=validate_static_route(result,tomogram,bridge,builder=builder,map_version_id=session['version_id'])
        snapshot=checked['route_snapshot'].payload()
        assert snapshot['frame_id']=='d1max_loc_map' and snapshot['preview_ready']
        assert not snapshot['execution_eligible']
        xyz=np.asarray(snapshot['xyz'])
        record=dict(case=name,passed=True,elapsed_s=time.monotonic()-began,
            length_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),
            floors=sorted(set(snapshot['point_floor_ids'])),segments=len(snapshot['segments']),
            route_hash=checked['route_snapshot'].route_hash,ground_goal_source_xyz=draft['xyz'])
        records.append(record)
        print(json.dumps(record),flush=True)
    ground,costs,blocked,metadata=build_layers(session)
    assert len(ground)>0 and np.isfinite(ground).all()
    # Both mapped support levels must survive the actual RViz layer builder.
    assert np.any(ground[:,2]<1.) and np.any(ground[:,2]>2.)
    return dict(schema=1,scope='selected_map_editor_native_PCT_and_RViz_layers_only',
        passed=True,session_id=session['id'],map_version_id=session['version_id'],
        capabilities=session['map_capabilities'],cases=records,rviz_layers=metadata,
        started_ROS=False,connected_SDK=False,robot_motion=False,
        BT_transport_tested=False,navigation_arrival_tested=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--native-child',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args()
    if not args.native_child:
        import yaml
        from d1max_pct_planner.native_runtime import prepare_native_environment
        session=json.loads(args.session.read_text())
        route=yaml.safe_load(Path(session['crossfloor_route_config']).read_text())
        environment=prepare_native_environment(route['vendor_root'])
        subprocess.run([sys.executable,str(Path(__file__).resolve()),'--session',str(args.session),
            '--output',str(args.output),'--native-child'],env=environment,check=True,timeout=180)
    else:
        try:
            result=check(args.session)
        except Exception as error:
            result=dict(schema=1,passed=False,error_type=type(error).__name__,error=str(error),
                error_code=getattr(error,'code',None),
                scope='selected_map_editor_native_PCT_and_RViz_layers_only',
                session=str(args.session.resolve()),started_ROS=False,connected_SDK=False,robot_motion=False)
            args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
            raise
        else:
            args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
