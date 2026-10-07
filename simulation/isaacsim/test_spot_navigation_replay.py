import math
import pytest
from spot_navigation_replay import CommandSchedule, StallResetGate, stop_completion


def events():
    return [dict(source_ns=i*2_000_000,command=[0.,0.] if i<2 else [.1,.2]) for i in range(11)]


def test_saved_commands_use_causal_hold_and_preserve_exact_zero():
    schedule=CommandSchedule(events(),20_000_000)
    assert schedule.at(0)==[0.,0.] and schedule.at(3_999_999)==[0.,0.]
    assert schedule.at(4_000_000)==[.1,.2] and schedule.at(20_000_000)==[.1,.2]
    result=schedule.at(4_000_000);result[0]=0.
    assert schedule.at(4_000_000)==[.1,.2]


@pytest.mark.parametrize('change',['rollback','nan','limit','gap','initial_nonzero','duration_gap'])
def test_bad_or_unrecorded_input_fails_closed(change):
    values=events();duration=20_000_000
    if change=='rollback':values[4]['source_ns']=values[3]['source_ns']
    elif change=='nan':values[5]['command'][0]=math.nan
    elif change=='limit':values[5]['command'][1]=math.nextafter(.3,math.inf)
    elif change=='gap':values=values[:1]+[dict(source_ns=24_000_000,command=[.1,0.])];duration=24_000_000
    elif change=='initial_nonzero':values[0]['command']=[.1,0.]
    elif change=='duration_gap':duration=42_000_000
    with pytest.raises(ValueError):CommandSchedule(values,duration)


def run_gate(gate,first,count,command=[.1,0.],linear=[0.,0.,.02],angular=[.04,0.,0.],offset=0):
    return [gate.observe(i*2_000_000,command,linear,angular,i+offset) for i in range(first,first+count)]


def test_full_xyz_and_full_angular_stall_and_existing_inference_boundary():
    gate=StallResetGate();assert not any(run_gate(gate,0,500))
    assert gate.observe(1_000_000_000,[.1,0.],[0.,0.,.02],[.04,0.,0.],503) is None
    event=gate.observe(1_002_000_000,[.1,0.],[0.,0.,.02],[.04,0.,0.],510)
    assert event['continuous_static_duration_ns']==1_002_000_000
    assert event['policy_counter']==510
    assert not any(run_gate(gate,502,600))


@pytest.mark.parametrize('linear,angular',[([0.,0.,.031],[0.,0.,0.]),([0.,0.,0.],[.051,0.,0.])])
def test_xy_or_yaw_only_static_never_qualifies(linear,angular):
    assert not any(run_gate(StallResetGate(),0,600,linear=linear,angular=angular))


def test_motion_gap_or_zero_restarts_window_and_one_reset_per_positive_episode():
    gate=StallResetGate();assert not any(run_gate(gate,0,500))
    assert gate.observe(1_000_000_000,[.1,0.],[.04,0.,0.],[0.,0.,0.],500) is None
    assert not any(run_gate(gate,501,500))
    assert gate.observe(2_002_000_000,[.1,0.],[0.,0.,.02],[.04,0.,0.],1010)
    assert not any(run_gate(gate,1002,20,command=[0.,.2]))
    assert not any(run_gate(gate,1022,500))
    assert gate.observe(3_044_000_000,[.1,0.],[0.,0.,.02],[.04,0.,0.],1530)['episode']==2
    gap=StallResetGate();assert not any(run_gate(gap,0,500))
    assert gap.observe(1_004_000_000,[.1,0.],[0.,0.,0.],[0.,0.,0.],510) is None
    with pytest.raises(ValueError):gap.observe(1_004_000_000,[.1,0.],[0.,0.,0.],[0.,0.,0.],510)


def test_final_true_zero_requires_original_continuous_full_norms():
    rows=[dict(source_ns=i*2_000_000,linear_velocity=[0.,0.,.02],angular_velocity=[.04,0.,0.]) for i in range(1501)]
    assert stop_completion(rows,0)==1.
    rows[499]['linear_velocity']=[0.,0.,.031]
    assert stop_completion(rows,0)==2.


def test_stop_elapsed_uses_original_zero_event_instead_of_first_post_step_row():
    rows=[dict(source_ns=(i+1)*2_000_000,
        linear_velocity=[.04 if i<1000 else 0.,0.,0.],angular_velocity=[0.,0.,0.]) for i in range(1501)]
    assert stop_completion(rows,0)==3.002
    assert stop_completion(rows,0)>3.


def test_missing_physics_measurement_cannot_fill_a_stop_window():
    rows=[dict(source_ns=i*2_000_000,linear_velocity=[0.,0.,0.],angular_velocity=[0.,0.,0.]) for i in range(1000)]
    rows=rows[:499]+rows[501:]
    assert stop_completion(rows,0) is None
