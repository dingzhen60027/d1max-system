#!/usr/bin/env python3
"""Read-only local-flow audit against real committed cross-floor route geometry.

No ROS init in the geometry probe. Ideal measured progression and portal stops
are explicit fixtures, NOT controller-driven navigation or physical acceptance.
Native tests use a private router and production native build, not a replacement
occupancy map. No task, reference, permit or command is published to a robot.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import time

import numpy as np


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def record(path):
    path=Path(path).resolve(strict=True)
    return dict(path=str(path),sha256=digest(path))


def runtime():
    from ament_index_python.packages import get_package_prefix
    root=Path(os.environ['D1MAX_NAV_ROOT'])
    result=dict(revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
        dirty_status=subprocess.check_output(['git','status','--short'],cwd=root,text=True),modules={},executables={},sources={})
    for name in ('continuous_reference','continuous_reference_transport','continuous_reference_node',
                 'bt_adapters','single_floor_session'):
        full='d1max_pct_scan.'+name
        result['modules'][full]=record(importlib.util.find_spec(full).origin)
    for package,name in (('scan_planner','scan_planner_node'),('d1max_trajectory_tracker','trajectory_tracker'),
                         ('d1max_navigation_bt','navigator_node')):
        result['executables'][package]=record(Path(get_package_prefix(package))/'lib'/package/name)
    for name in ('src/scan_planner_vendor/plan_manage/src/scan_replan_fsm.cpp',
                 'src/scan_planner_vendor/plan_manage/include/plan_manage/execution_validator.hpp',
                 'src/d1max_trajectory_tracker/include/d1max_trajectory_tracker/tracker_core.hpp',
                 'src/d1max_trajectory_tracker/include/d1max_trajectory_tracker/execution_contract.hpp'):
        result['sources'][name]=record(root/name)
    return result


def geometry_probe(artifact):
    from d1max_pct_scan.source_route import RouteSnapshot
    from d1max_pct_scan.continuous_reference import ContinuousReference,Observation
    from d1max_pct_scan.control_frame_contract import Context,Rigid,BodySample,create_anchor
    snapshot=RouteSnapshot.create(json.loads(artifact.read_text())['source_snapshot'])
    value=snapshot.payload();points=np.asarray(value['xyz'],dtype=float)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    context=Context('local-audit',1,'geometry-fixture',value['map_version_id'])
    base_ns=1_800_000_000_000_000_000
    body_start=points[0]+[0.,0.,.55]
    def observation(position,t):
        return Observation(BodySample(context=context,frame='d1max_loc_odom',child='d1max_loc_base_link',
                           source_ns=base_ns+round(t*1e9),
                           pose=Rigid(tuple(map(float,position)),(0.,0.,0.,1.))),100.+t)
    local=observation(body_start,0.)
    global_=BodySample(context=context,frame='d1max_loc_map',child='d1max_loc_base_link',
                       source_ns=base_ns,pose=local.body.pose)
    ref=ContinuousReference(snapshot,context=context,task_id='crossfloor-reference-audit',
        anchor=create_anchor(global_,local.body,1),body_height_m=.55,body_height_calibration_id='geometry_fixture_only')
    windows=[];transitions=[];timings=[];t=0.;measurements=0
    last_confirmed=0.
    for number,segment in enumerate(value['segments']):
        first,last=segment['begin_index'],segment['end_index']
        count=max(2,int(np.ceil((arc[last]-arc[first])/.015))+1)
        distances=np.linspace(arc[first],arc[last],count)
        last_window=-1e9
        for distance in distances:
            t+=.05
            position=np.array([np.interp(distance,arc,points[:,i]) for i in range(3)])+[0.,0.,.55]
            sample=observation(position,t)
            began=time.perf_counter()
            progress=ref.project(sample,current_source_ns=sample.body.source_ns,now_monotonic=sample.received_monotonic)
            timings.append((time.perf_counter()-began)*1000.);measurements+=1
            if progress.confirmed_arc_m+1e-9<last_confirmed:
                raise AssertionError('Confirmed route progress regressed')
            last_confirmed=progress.confirmed_arc_m
            if distance<arc[last]-.02 and distance-last_window>=1.:
                window=ref.window(current_source_ns=sample.body.source_ns,now_monotonic=sample.received_monotonic,horizon_m=2.)
                assert window.segment_id==segment['segment_id']
                assert window.required_mode==segment['required_mode'] and window.frame_id=='d1max_loc_odom'
                assert max(window.edge_indices)<last
                assert window.ground_to_body_height_applications==1
                windows.append(dict(segment_id=window.segment_id,kind=window.segment_kind,required_mode=window.required_mode,
                    measured_arc_m=progress.measured_arc_m,end_arc_m=window.end_arc_m,points=len(window.body_odom_xyz)))
                last_window=distance
        if number+1<len(value['segments']):
            # Explicit ideal measured dwell at the actual portal, not an SDK
            # mode event and not an automatic runtime segment transition.
            for _ in range(7):
                t+=.05;sample=observation(points[last]+[0.,0.,.55],t)
                ref.project(sample,current_source_ns=sample.body.source_ns,now_monotonic=sample.received_monotonic)
            next_id=ref.advance_segment(current_source_ns=sample.body.source_ns,now_monotonic=sample.received_monotonic)
            transitions.append(dict(from_segment=segment['segment_id'],to_segment=next_id,
                fixture='explicit_ideal_portal_stop_not_runtime_mode_or_control'))
    assert snapshot.route_hash==ref.snapshot.route_hash
    return dict(scope='pure_real_route_geometry_not_ROS_local_planning_or_navigation',passed=True,
        route_hash=snapshot.route_hash,route_length_m=float(arc[-1]),measurements=measurements,
        windows=windows,portal_transitions=transitions,
        projection_ms={q:float(np.percentile(timings,p)) for q,p in (('p50',50),('p95',95),('p99',99))},
        source_execution_eligible=value['execution_eligible'],execution_permission_changed=False,
        closed_loop=False,physical_acceptance=False)


def runtime_boundary_probe(manifest,route_artifact):
    from d1max_pct_scan.continuous_reference_node import ReferenceCallbacks
    value=json.loads(route_artifact.read_text())['source_snapshot']
    params=dict(session_id='local-audit',map_version_id=value['map_version_id'],
        **{'expected_'+key:value[key] for key in ('source_map_sha256','tomogram_sha256','conditioning_sha256')},
        body_height_m=.55,body_height_calibration_id='fixture_only',transport_mode='isolated_mock',
        floor_id='floor1',planning_manifest=str(manifest))
    try:
        ReferenceCallbacks(params,lambda *_:None,lambda:1_800_000_000_000_000_000)
    except ValueError as error:
        return dict(formal_crossfloor_reference_node_constructible=False,error=str(error),
            source_manifest_geometry_operation=json.loads(manifest.read_text())['geometry_operation'],
            explanation='Formal node loads SourceIdentityBridge; conditioned cross-floor manifest is a different contract',
            not_bypassed=True)
    return dict(formal_crossfloor_reference_node_constructible=True)


def native_tests(output):
    from d1max_pct_scan.isolated_zenoh import private_router
    build=Path(os.environ['D1MAX_LOCAL_AUDIT_ROOT'])/'build'
    rows=[]
    with private_router(output/'zenoh') as env:
        for package in ('scan_planner','d1max_trajectory_tracker'):
            command=['ctest','--test-dir',str(build/package),'--output-on-failure',
                     '-E','test_planner_startup|test_waypoint_parameters']
            with (output/(package+'.log')).open('w') as log:
                result=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180)
            rows.append(dict(package=package,exit_code=result.returncode,command=command))
    return dict(passed=all(row['exit_code']==0 for row in rows),tests=rows,
                graph_tests_excluded=True,physical_acceptance=False)


def diagnostic_bundle(output):
    """Copy a self-contained test bundle; never weaken production verification.

    single_floor_session requires all motion-graph executables under the same
    release root. Preserve that requirement instead of allowing arbitrary ELF
    overrides. Only generated diagnostic copies are replaced, not the baseline.
    """
    baseline=Path(os.environ['D1MAX_RELEASE']).resolve(strict=True)
    audit=Path(os.environ['D1MAX_LOCAL_AUDIT_ROOT'])
    descriptor=json.loads((baseline/'release.json').read_text())
    for component in ('interfaces','native','tracker','bt','sdk','rviz','application'):
        target=output/component
        target.mkdir()
        subprocess.run(['cp','-a','--reflink=auto',str(baseline/component/'install'),str(target/'install')],check=True)
    for package,component,source in (
        ('scan_planner','native',audit/'install'),
        ('d1max_trajectory_tracker','tracker',audit/'install'),
        ('d1max_navigation_bt','bt',Path(os.environ['D1MAX_NAV_ROOT'])/'experiments/crossfloor_global_20261002/install')):
        # cp directory contents, including relocatable ament files, over the
        # generated copy. Native dependencies remain the verified baseline.
        subprocess.run(['cp','-a','--reflink=auto',str(source/package)+ '/.',
                        str(output/component/'install'/package)],check=True)
    relative=Path(descriptor['default_map_directory'])
    (output/relative).parent.mkdir(parents=True,exist_ok=True)
    subprocess.run(['cp','-a','--reflink=auto',str(baseline/relative),str(output/relative)],check=True)
    descriptor.update(release_role='isolated_local_planning_diagnostic_only',physical_acceptance=False,
        baseline_release=str(baseline),sealed_manifest='diagnostic_manifest_not_activated.json',
        preflight_session='diagnostic_preflight',change_scope='fresh_native_and_tracker_and_BT_isolated_copies')
    (output/'release.json').write_text(json.dumps(descriptor,indent=2)+'\n')
    return dict(path=str(output),baseline=str(baseline),deployment_selector_changed=False,
        production_verifier_unchanged=True,physical_acceptance=False)


def envelope_probe(output):
    build=Path(os.environ['D1MAX_LOCAL_AUDIT_ROOT'])/'build/scan_planner'
    target=build/'CMakeFiles/test_motion_sweep.dir'
    flags={key:shlex.split(value) for key,value in
        (line.split(' = ',1) for line in (target/'flags.make').read_text().splitlines() if ' = ' in line)}
    link=shlex.split((target/'link.txt').read_text())
    source=Path(os.environ['D1MAX_NAV_ROOT'])/'tools/validation/probe_local_envelope_contract.cpp'
    binary=output/'native_envelope_contract_probe'
    command=[link[0],*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],
             str(source),'-o',str(binary),*link[link.index('-o')+2:]]
    compiled=subprocess.run(command,cwd=build,text=True,capture_output=True,timeout=120)
    (output/'compile.log').write_text(compiled.stdout+compiled.stderr)
    if compiled.returncode:
        raise RuntimeError('diagnostic_envelope_probe_compile_failed:'+str(output/'compile.log'))
    result=subprocess.run([str(binary)],text=True,capture_output=True,timeout=20)
    (output/'probe.json').write_text(result.stdout)
    return dict(passed=result.returncode==0,source=record(source),binary=record(binary),
        compile_command=command,command=[str(binary)],result=json.loads(result.stdout))


def summarize(output):
    root=Path(os.environ['D1MAX_LOCAL_AUDIT_ROOT'])
    cases=[]
    def stats(values):
        return dict(samples=len(values),**{name:float(np.percentile(values,q))
            for name,q in (('p50',50),('p95',95),('p99',99))},max=max(values)) if values else dict(samples=0)
    for directory in ('long_straight_03','short_straight_01'):
        source=root/directory/'graph_report.json'
        value=json.loads(source.read_text())
        log=(root/directory/'scan.log').read_text()
        solves=[list(map(float,m.groups())) for m in re.finditer(
            r'NativeSolve generation=\d+ queue_ms=([\d.]+) snapshot_ms=([\d.]+) solve_ms=([\d.]+)',log)]
        integration=[float(m.group(1)) for m in re.finditer(r'complete ray integration:.*?duration_ms=([\d.]+)',log)]
        initial=next((e['elapsed_s'] for e in value['events'] if e['key']=='bt' and e.get('phase')=='following_route'),None)
        accepted=next((e['elapsed_s'] for e in value['events'] if e['key']=='tracker' and e.get('active')),None)
        cases.append(dict(input=record(source),case=value['case'],scope=value['kind'],outcome=value['outcome'],
            distance_m=value['distance_m'],route_hashes=value['route_hashes'],
            first_local_preparation_delay_from_follow_request_s=accepted-initial if initial is not None and accepted is not None else None,
            native_solve_ms=stats([v[2] for v in solves]),native_queue_ms=stats([v[0] for v in solves]),
            native_snapshot_ms=stats([v[1] for v in solves]),
            logged_map_integration_ms=stats(integration),
            timing=value['timing'],whole_run_message_rates_hz=value['whole_run_message_rates_hz'],
            source_trial=value['source_trial'],sensor_fixture=value['sensor_fixture'],
            runtime_bundle=record(root/directory/'runtime_bundle.json'),
            physical_acceptance=False,continuous_motion_tracking_accepted=False))
    geometry=root/'crossfloor_geometry_02/report.json';envelope=root/'envelope_contract_01/report.json'
    env_result=json.loads(envelope.read_text())['envelope_probe']['result']
    return dict(overall_crossfloor_local_execution_ready=False,production_behavior_changed=False,
        cases=cases,geometry_input=record(geometry),envelope_input=record(envelope),
        confirmed_blockers=[
            dict(reason='vertical_envelope_contract_reversed_in_command_checker',
                 source='src/scan_planner_vendor/plan_manage/include/plan_manage/motion_sweep.hpp',
                 evidence=env_result,
                 required_change='One explicit reflected body-envelope contract for native curve and command checks; do not delete ground or nearby returns'),
            dict(reason='crossfloor_manifest_rejected_by_formal_reference_node',
                 source='src/d1max_pct_scan/d1max_pct_scan/continuous_reference_node.py',
                 evidence=json.loads(geometry.read_text())['formal_runtime_boundary'],
                 required_change='A proven source-support bridge for conditioned cross-floor map; no fake rigid TF'),
            dict(reason='segment_transition_not_wired_and_support_semantics_single_floor',
                 sources=['src/d1max_pct_scan/d1max_pct_scan/continuous_reference_node.py',
                          'src/d1max_pct_scan/d1max_pct_scan/continuous_reference_transport.py'],
                 evidence='Production node hardcodes floor/general SupportReference; request_segment_transition has no production caller',
                 required_change='Explicit BT-owned portal/mode/segment transaction and segment-bound support, without physical capability bypass')],
        additional_risks=[
            'Collision validation sees original ray ages near 400 ms; measured worst case crosses reference freshness limit. Rates alone do not prove latency.',
            'Top-level tracking phase can remain present while actual_command_blocked is reported separately. UI must not equate phase with physical progress.',
            'Source headers are newer than pinned ELF; current test uses rebuilt and individually hashed ELF, not automatic deployment.'],
        not_validated=['Cross-floor local ROS graph','Continuous moving tracking and handover with current geometry',
                       'Real rosbag localization/perception timing in this round','Real SDK or physical motion',
                       'NUC or one-hour realtime acceptance'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('audit','native-tests','bundle','envelope-probe','summarize'))
    parser.add_argument('--route-artifact',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    report=dict(scope='isolated_local_planning_diagnostics',runtime=runtime(),sdk_connected=False,
                motion_enabled=False,production_deployed=False,physical_acceptance=False)
    if args.command=='audit':
        if not args.route_artifact: parser.error('--route-artifact is required')
        artifact=args.route_artifact.resolve(strict=True)
        manifest=Path(os.environ['D1MAX_NAV_ROOT'])/'maps/processed/sc_pgo_20260923_crossfloor_complete_001/manifest.json'
        report['input']=record(artifact)
        report['geometry']=geometry_probe(artifact)
        report['formal_runtime_boundary']=runtime_boundary_probe(manifest,artifact)
    elif args.command=='native-tests':
        report['native_tests']=native_tests(output)
    elif args.command=='bundle':
        report['diagnostic_bundle']=diagnostic_bundle(output)
    elif args.command=='envelope-probe':
        report['envelope_probe']=envelope_probe(output)
    else:
        report['assessment']=summarize(output)
    (output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output),geometry_passed=report.get('geometry',{}).get('passed'),
        native_passed=report.get('native_tests',{}).get('passed'),
        formal_runtime_boundary=report.get('formal_runtime_boundary')),ensure_ascii=False),flush=True)
    return 0 if all(report.get(key,{}).get('passed',True) for key in ('native_tests','envelope_probe')) else 2


if __name__=='__main__':raise SystemExit(main())
