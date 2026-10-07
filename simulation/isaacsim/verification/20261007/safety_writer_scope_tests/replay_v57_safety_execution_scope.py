"""Replay original recorded permit/ACK/grant callbacks, without a ROS graph.

This isolates writer scope bookkeeping. Projected rays, motion proofs and
independent SDK stationary callbacks are deliberately not reconstructed.
Therefore grant acceptance here is not a full motion or formal-run result.
"""
import collections
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

from d1max_pct_scan.execution_safety import ExecutionSafety
from d1max_pct_scan.execution_handoff import ExecutionHandoffAdmission
import d1max_pct_scan.execution_safety as safety_module
import d1max_pct_scan.execution_handoff as current_module

BASE=Path('/home/eric/wjg/d1max-build-isaac')
RUN=BASE/'runs/campus_restart_v57_001'
CANDIDATE=BASE/'isaac-candidate-v57-campus-execution-scope-restart'
OLD=CANDIDATE/'source_snapshot/nav/src/d1max_pct_scan/d1max_pct_scan/execution_handoff.py'
OUTPUT=BASE/'unknown_research/v57_safety_execution_scope_replay.json'

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def message(value):
    if isinstance(value,dict):return SimpleNamespace(**{k:message(v) for k,v in value.items()})
    if isinstance(value,list):return [message(v) for v in value]
    return value

def state(core):
    return dict(ack_highwater=core.last_ack_sequence,grant_highwater=core.last_grant_sequence,
        commit_sequence=core.commit_sequence,applied_trajectory_id=core.applied.trajectory_id if core.applied else None)

spec=importlib.util.spec_from_file_location('d1max_pct_scan.sealed_v57_execution_handoff',OLD)
old_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(old_module)
session=json.loads((RUN/'session.json').read_text())
engines={name:cls(ExecutionSafety(session['id'],'isolated_mock',
    braking_model_sha256=session['execution_braking_model_sha256']),enabled=True)
    for name,cls in [('sealed_v57',old_module.ExecutionHandoffAdmission),('source_fix',ExecutionHandoffAdmission)]}
result=dict(schema=1,scope='Original permit/ACK/grant bookkeeping replay only; no reconstructed sensor or motion proof',
    run=str(RUN),session_id=session['id'],source_sha256=dict(sealed_v57=digest(OLD),
    source_fix=digest(current_module.__file__),shared_safety=digest(safety_module.__file__)),phases=[])
for phase in [0,1]:
    path=RUN/f'scenario_suite/phase_{phase:03d}_trace.jsonl'
    counters={k:collections.Counter() for k in engines};first={};trace_hash=hashlib.sha256()
    with path.open('rb') as stream:
        for line in stream:
            trace_hash.update(line);row=json.loads(line);kind=row['kind']
            if kind not in ('permit','commit_ack','handoff_grant'):continue
            raw=row['data'];wire=raw.get('wire',raw);m=message(wire);now=row['source_clock_ns']
            for name,engine in engines.items():
                before=state(engine)
                if kind=='permit':accepted=engine.on_permit(m,now) if name=='source_fix' else engine.on_permit(m)
                elif kind=='commit_ack':accepted=engine.on_ack(m,now)
                else:accepted=engine.on_grant(m,now)
                counters[name][kind+('_accepted' if accepted else '_rejected')]+=1
                if kind=='commit_ack' and name not in first:
                    first[name]=dict(sequence=m.sequence,commit_sequence=m.commit_sequence,applied=m.applied,
                        before=before,accepted=accepted,after=state(engine))
    result['phases'].append(dict(index=phase,trace_path=str(path),trace_sha256=trace_hash.hexdigest(),
        engines={name:dict(counts=dict(counters[name]),first_ack=first.get(name),end=state(engine)) for name,engine in engines.items()}))
assert result['phases'][0]['engines']['sealed_v57']['end']['ack_highwater']==16
assert result['phases'][0]['engines']['sealed_v57']['end']['grant_highwater']==8
assert not result['phases'][1]['engines']['sealed_v57']['first_ack']['accepted']
assert result['phases'][1]['engines']['source_fix']['first_ack']['accepted']
assert result['phases'][1]['engines']['source_fix']['first_ack']['after']['commit_sequence']==1
OUTPUT.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
