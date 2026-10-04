#!/usr/bin/env python3
"""Run isolated real-component cases sequentially, retaining every failure.

Each child creates its own loopback-only router and cleans only its own graph.
No live SDK is instantiated. These are analytic sensor/plant fixtures, not bag
localization, dynamic-obstacle acceptance, or a one-hour soak.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=65.)
    parser.add_argument('--cases',nargs='+',default=['cancel','rear_dropout','map_correction','goal_yaw','blocked_safe_distance'],
        choices=['normal','cancel','rear_dropout','map_correction','goal_yaw','blocked','blocked_safe_distance','long_straight','long_map_correction','global_gap','local_gap','static_box','dynamic_box','unobserved_start'])
    args=parser.parse_args();root=args.output.resolve()
    if not 20.<=args.duration<=120.:raise ValueError('bounded_suite_duration_required')
    root.mkdir(parents=True,exist_ok=False)
    rows=[]
    for case in args.cases:
        result=subprocess.run([sys.executable,str(Path(__file__).with_name('run_single_floor_graph.py')),
            '--output',str(root/case),'--case',case,'--duration',str(args.duration)])
        path=root/case/'graph_report.json'
        report=json.loads(path.read_text()) if path.exists() else {}
        audit_path=root/case/'handoff_audit.json'
        if path.exists():
            subprocess.run([sys.executable,str(Path(__file__).with_name('audit_execution_handoff_graph.py')),
                str(path),'--output',str(audit_path)],check=False)
        audit=json.loads(audit_path.read_text()) if audit_path.exists() else {}
        statuses={check['status'] for check in audit.get('checks',[])}
        protocol_status=('MISSING' if not statuses else 'FAIL' if 'FAIL' in statuses
                         else 'UNVERIFIED' if statuses!={'PASS'} else 'PASS')
        row=dict(case=case,exit_code=result.returncode,outcome=report.get('outcome'),
            distance_m=report.get('distance_m'),max_speed_mps=report.get('max_speed_mps'),
            elapsed_s=report.get('elapsed_s'),route_hashes=report.get('route_hashes'),
            report=str(path),handoff_audit=str(audit_path),
            protocol_status=protocol_status,
            physical_acceptance=False)
        rows.append(row)
        (root/'suite.json').write_text(json.dumps(rows,indent=2)+'\n')
        print(json.dumps(row),flush=True)
    # No observed CAS is UNVERIFIED, never silently labelled proven. Keep
    # navigation outcome and evidence coverage as independent report fields.
    return 0 if all(row['exit_code']==0 and row['protocol_status'] not in ('FAIL','MISSING')
                    for row in rows) else 2


if __name__=='__main__':sys.exit(main())
