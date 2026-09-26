#!/usr/bin/env python3
"""Add the completed front-internal/historical-rotation run to immutable v2."""
import json
from pathlib import Path
import runpy

import numpy as np

HERE=Path(__file__).resolve().parent
V2=runpy.run_path(str(HERE/'gravity_reference_audit_v2.py'))
EXTRA='internal_historical_paired_front_180s_20260919'


def build():
    report=V2['build']()
    run=V2['BASE']['audit_run'](EXTRA)
    sequence=np.asarray(run.pop('comparison_header_sequence_s'))
    run['gravity_update_events']=V2['updates'](EXTRA)
    report['runs'].append(run)
    report['schema_version']=3
    report['extends_without_overwriting']='gravity_reference_audit_v2.json; includes all six completed diagnostic runs'
    reference=np.genfromtxt(Path(report['runs'][0]['directory'])/'frontend_state.csv',delimiter=',',names=True)['t']
    delta=float(np.max(np.abs(sequence-reference))) if len(sequence)==len(reference) else None
    report['max_header_difference_from_baseline_s'][EXTRA]=delta
    report['state_headers_correspond_in_sequence_within_100us']=all(v is not None and v<1e-4 for v in report['max_header_difference_from_baseline_s'].values())
    report['limitations']=[x for x in report['limitations'] if not x.startswith('Only completed internal_device_paired_front_180s_20260919')]
    report['limitations'].append('The internal_historical run was added only after result.json confirmed completion; prior v1/v2 files preserved.')
    return report


if __name__=='__main__':
    print(json.dumps(build(),ensure_ascii=False,indent=2))
