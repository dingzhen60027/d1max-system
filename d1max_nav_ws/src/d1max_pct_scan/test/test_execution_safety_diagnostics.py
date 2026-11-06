from copy import deepcopy
from d1max_pct_scan.execution_safety import AdmissionDiagnostics, Decision
from test_execution_safety import fixture, motion_proof, NOW, stamp


def test_diagnostics_remain_fixed_capacity_under_arbitrary_wait_reasons():
    diagnostics=AdmissionDiagnostics()
    keys=set(diagnostics.counts)
    for i in range(10000):diagnostics.record(Decision(False,'arbitrary_wait_'+str(i)))
    assert set(diagnostics.counts)==keys
    assert diagnostics.counts['other']==10000
    assert diagnostics.last_nonzero_rejection is None


def test_wait_diagnostic_preserves_original_binding_after_later_zero_stop():
    _,_,demand,*_=fixture();original=deepcopy(demand)
    diagnostics=AdmissionDiagnostics()
    diagnostics.record(Decision(False,'waiting_command_sweep'),demand)
    zero=deepcopy(demand);zero.velocity.linear.x=zero.velocity.angular.z=0.
    diagnostics.record(Decision(False,'task_owner_revoked'),zero)
    diagnostics.record(Decision(True,'zero_stop_request_not_free_space_proof'),zero)
    state=diagnostics.snapshot()
    assert state['reason_counts']['waiting_command_sweep']==1
    assert state['last_nonzero_rejection']['reason']=='waiting_command_sweep'
    assert state['last_nonzero_rejection']['source_stamp_ns']==NOW
    assert state['last_nonzero_rejection']['valid_until_ns']==NOW+200_000_000
    assert demand==original
    state['reason_counts']['waiting_command_sweep']=99
    assert diagnostics.counts['waiting_command_sweep']==1


def test_motion_verdict_diagnostic_uses_original_demand_times_not_check_time():
    _,_,demand,*_=fixture();proof=motion_proof(demand)
    proof.body_source_stamp=stamp(NOW+1_000_000)
    diagnostics=AdmissionDiagnostics()
    diagnostics.record(Decision(False,'command_sweep_rejected:motion_sweep_occupied'),proof)
    state=diagnostics.snapshot()
    assert state['reason_counts']['command_sweep_rejected']==1
    record=state['last_nonzero_rejection']
    assert record['input_kind']=='motion_validation'
    assert record['body_source_stamp_ns']==NOW
    assert record['valid_until_ns']==NOW+200_000_000
    assert record['demand_sequence']==demand.sequence
