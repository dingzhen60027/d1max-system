import copy
import json
import math
import pytest
from policy_history import PolicyHistory, FrozenCommands, check_component_options, observe_official_return
from full_scene_replay import freeze_history, scene_command

SHA='a'*64


def context(i):
    return dict(phase='source',source_tick=i,source_ns=i*2_000_000,policy_counter=i+1000)


def make_history(path):
    path.mkdir()
    history=PolicyHistory(path,SHA)
    for i in range(20):
        history.command(context(i),[.1 if i>=3 and i<15 else 0.,0.],
            dict(command_sequence=1 if i>=3 else -1,command_source_deadline_ns=250_000_000,
                 command_steady_deadline_ns=123456789,guard_reason='accepted' if i<15 else 'source_deadline_expired'))
    return history.close()


def test_records_exact_tick_changes_and_original_deadlines(tmp_path):
    path=tmp_path/'history';manifest=make_history(path)
    assert manifest['actual_source_ticks']==20 and manifest['duration_ns']==40_000_000
    rows=[json.loads(line) for line in (path/'policy_command_events.jsonl').read_text().splitlines()]
    assert [row['source_tick'] for row in rows]==[0,3,15]
    assert rows[1]['source_ns']==6_000_000 and rows[1]['policy_counter']==1003
    assert rows[1]['authority']['command_steady_deadline_ns']==123456789
    assert rows[2]['authority']['guard_reason']=='source_deadline_expired'
    output=tmp_path/'frozen.json';value=freeze_history(path,output)
    schedule=FrozenCommands.load(output,SHA,20)
    assert schedule.at_tick(2)['command']==[0.,0.]
    assert schedule.at_tick(3)['command']==[.1,0.]
    assert schedule.at_tick(15)['command']==[0.,0.]
    assert schedule.at_tick(19)['authority']==rows[2]['authority']
    assert value['events']==rows
    with pytest.raises(ValueError):schedule.at_tick(20)
    with pytest.raises(ValueError):freeze_history(path,output)


@pytest.mark.parametrize('bad',['gap','rollback','non_grid','missing_first','nonfinite','limit'])
def test_history_missing_actual_tick_or_bad_command_fails(tmp_path,bad):
    history=PolicyHistory(tmp_path,SHA)
    try:
        if bad=='missing_first':
            with pytest.raises(ValueError):history.command(context(1),[0.,0.],{})
            return
        history.command(context(0),[0.,0.],{})
        tick=2 if bad=='gap' else 0 if bad=='rollback' else 1
        c=context(tick)
        if bad=='non_grid':c['source_ns']+=1
        cmd=[math.nan,0.] if bad=='nonfinite' else [.1500001,0.] if bad=='limit' else [0.,0.]
        with pytest.raises(ValueError):history.command(c,cmd,{})
    finally:
        history.close()


def test_observation_original_tensor_called_once_and_return_identity(tmp_path):
    history=PolicyHistory(tmp_path,SHA)
    class Tensor:
        dtype='float32'
        def detach(self):return self
        def cpu(self):return self
        def reshape(self,n):assert n==-1;return self
        def tolist(self):return list(range(48))
    original=Tensor();calls=[];command=object()
    def compute(value):
        assert value is command
        calls.append(value)
        return original
    assert observe_official_return(compute,command,history.observation,1010,context(10)) is original
    assert calls==[command]
    history.close()
    row=json.loads((tmp_path/'policy_observations.jsonl').read_text())
    assert row['observation']==list(range(48)) and row['inference_policy_counter']==1010


@pytest.mark.parametrize('change',['source','rollback','scene','count','limit'])
def test_frozen_domain_or_payload_mismatch_rejected(tmp_path,change):
    path=tmp_path/'history';make_history(path)
    value=freeze_history(path,tmp_path/'frozen.json')
    value=copy.deepcopy(value)
    if change=='source':value['events'][1]['source_ns']+=1
    elif change=='rollback':value['events'][1]['source_tick']=0
    elif change=='scene':value['scene_sha256']='b'*64
    elif change=='count':value['actual_source_ticks']=30
    elif change=='limit':value['events'][1]['command'][0]=.16
    with pytest.raises(ValueError):FrozenCommands(value,SHA,20)


@pytest.mark.parametrize('args',[
    (0,'frozen',None,0,0.,0.,None),(20,'frozen',.3,10,0.,0.,None),
    (20,'frozen',None,0,.1,0.,None),(20,'frozen',None,0,0.,0.,'control'),
    (20,None,.61,10,0.,0.,None),(20,None,.4,20,0.,0.,None),
    (20,None,.4,9,0.,0.,None),(21,None,.4,10,0.,0.,None)])
def test_component_cannot_mix_external_motion_or_omit_stop_tail(args):
    with pytest.raises(ValueError):check_component_options(*args)


def test_internal_policy_stair_options_are_bounded_and_distinct():
    for speed in (.3,.4,.5,.6):check_component_options(20000,None,speed,15000)
    check_component_options(20000,'frozen',None,0)


def test_navigation_servo_calibration_requires_sealed_domain_and_exclusive_four_second_stop():
    check_component_options(6000,None,None,4000,navigation_speed=.25,command_limits=[.25,.3])
    for kwargs in [dict(navigation_speed=.251,command_limits=[.25,.3]),
                   dict(navigation_speed=.25,command_limits=[.15,.3])]:
        with pytest.raises(ValueError):check_component_options(6000,None,None,4000,**kwargs)
    with pytest.raises(ValueError):check_component_options(6000,None,.4,4000,navigation_speed=.25,command_limits=[.25,.3])
    with pytest.raises(ValueError):check_component_options(6000,None,None,4010,navigation_speed=.25,command_limits=[.25,.3])


def test_component_500hz_begin_then_terminal_end_and_full_norms(tmp_path):
    history=PolicyHistory(tmp_path,SHA,component=True)
    for i in range(11):
        history.component_state(dict(context(i),acquisition_phase='BEGIN' if i<10 else 'END'),
            [0.,0.],[.4 if i<5 else 0.,0.,0.],[1.,2.,.5],[1.,0.,0.,0.],[.3,.4,.1],[.1,.2,.3])
    with pytest.raises(ValueError):history.component_state(context(12),[0.,0.],[0.,0.,0.],
        [1.,2.,.5],[1.,0.,0.,0.],[0.,0.,0.],[0.,0.,0.])
    result=history.close()
    assert result['component_measurement_count']==11 and result['component_last_source_ns']==20_000_000
    rows=[json.loads(row) for row in (tmp_path/'policy_component_state_500hz.jsonl').read_text().splitlines()]
    assert rows[-1]['acquisition_phase']=='END'
    assert rows[0]['full_linear_speed']==math.hypot(.3,.4,.1)
    assert rows[0]['official_policy_input']==[.4,0.,0.] and rows[0]['highlevel_command']==[0.,0.]


def test_sealed_larger_navigation_domain_is_explicit_and_cannot_be_replayed_as_old_domain(tmp_path):
    history=PolicyHistory(tmp_path,SHA,command_limits=[.3,.45])
    for i in range(10):history.command(context(i),[.3,.45],{})
    manifest=history.close()
    assert manifest['command_limits']==[.3,.45]
    value=freeze_history(tmp_path,tmp_path/'frozen.json')
    assert FrozenCommands(value,SHA,10,[.3,.45]).at_tick(0)['command']==[.3,.45]
    with pytest.raises(ValueError):FrozenCommands(value,SHA,10,[.15,.3])
    value['command_limits']=[.301,.45]
    with pytest.raises(ValueError):FrozenCommands(value,SHA,10)


def test_internal_override_history_cannot_masquerade_as_navigation_replay(tmp_path):
    history=PolicyHistory(tmp_path,SHA)
    for i in range(10):history.command(context(i),[0.,0.],dict(policy_input_override=[.3,0.,0.]))
    history.close()
    with pytest.raises(ValueError):freeze_history(tmp_path,tmp_path/'frozen.json')


def test_changed_recorded_input_is_not_reconstructed(tmp_path):
    path=tmp_path/'history';make_history(path)
    with (path/'policy_command_events.jsonl').open('a') as stream:stream.write('{}\n')
    with pytest.raises(ValueError):freeze_history(path,tmp_path/'frozen.json')


def test_launcher_preserves_original_full_scene_and_seals(tmp_path):
    import hashlib
    session=tmp_path/'session';session.mkdir()
    scene=session/'scene.json'
    scene.write_text(json.dumps(dict(robot=dict(kind='official_spot_physx',initial_pose=[-8,-5,.8,0]),
        physics=dict(frequency_hz=500),obstacles=[dict(name='wall')]*85,dynamic_actors=[dict(id='person')],
        lidar=dict(frequency_hz=10),imu=dict(frequency_hz=100))))
    sha=hashlib.sha256(scene.read_bytes()).hexdigest()
    record=dict(id='original',isaac_bridge_contract=dict(scene_config=str(scene),scene_sha256=sha,clock_anchor_ns=100),
        static_collision_prior_contract=dict(static_prior_geometry_sha256='c'*64,
            body_envelope_attestation_required=True,body_envelope=dict(radius=.608)))
    (session/'session.json').write_text(json.dumps(record))
    command=scene_command(session,tmp_path/'result',policy_speed=.5,motion_frames=15000,frames=20000)
    assert command[command.index('--scene-file')+1]==str(scene)
    assert '--body-envelope-json' in command and '--static-prior-geometry-sha256' in command
    assert command[command.index('--test-policy-linear-speed')+1]=='.5' or command[command.index('--test-policy-linear-speed')+1]=='0.5'
    assert '--control-file' not in command and '--test-linear-speed' not in command


def test_navigation_native_history_is_explicit_and_rejects_source_gaps(tmp_path):
    from policy_history import full_physics_history_enabled
    assert not full_physics_history_enabled({})
    assert full_physics_history_enabled({}, component=True)
    assert full_physics_history_enabled({'record_full_physics_history': True})
    for value in (1, 'true', None):
        with pytest.raises(ValueError):
            full_physics_history_enabled({'record_full_physics_history': value})
    history = PolicyHistory(tmp_path, SHA, component=True, command_limits=[.25,.3])
    original = dict(context(0), mode='navigation_wire')
    authority = {'command_sequence': 17}
    history.command(original, [.25,.1], authority)
    original = dict(original, authority=authority)
    history.component_state(original, [.25,.1], [.4,0.,.1], [1.,2.,.5],
        [1.,0.,0.,0.], [.57,.19,.05], [.1,.2,.3])
    with pytest.raises(ValueError, match='source_gap'):
        history.component_state(context(2), [0.,0.], [0.,0.,0.], [1.,2.,.5],
            [1.,0.,0.,0.], [0.,0.,0.], [0.,0.,0.])
    manifest = history.close()
    row = json.loads((tmp_path/'policy_component_state_500hz.jsonl').read_text())
    assert row['mode'] == 'navigation_wire' and row['authority'] == original['authority']
    assert row['linear_velocity_world'] == [.57,.19,.05]
    assert row['full_linear_speed'] > .60  # XY/Z peaks remain visible.
    assert manifest['component_measurement_count'] == 1
    assert manifest['component_measurement_cadence'] == 'every real500Hz BEGIN plus final measured END'
