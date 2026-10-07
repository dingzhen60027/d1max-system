"""Pure offline checks; importing scene.py would start Kit, so parse its AST."""
import argparse
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from full_scene_replay import (component_runtime_robot, freeze_history,
    scene_command, validate_action_memory_experiment, validate_velocity_feedback_experiment)
from full_scene_replay import apply_velocity_feedback_experiment
from policy_history import FrozenCommands, PolicyHistory


def original_history(path, count=14):
    path.mkdir()
    history = PolicyHistory(path, 'a'*64)
    for i in range(count):
        history.command(dict(phase='source', source_tick=i, source_ns=i*2_000_000,
            policy_counter=1000+i, native_physics_tick=1000+i), [.1 if i < 12 else 0., 0.],
            dict(policy_input_override=None, replay_original_authority=None))
    return history.close()


def test_explicit_prefix_retains_entire_original_hashes_and_exact_real_events(tmp_path):
    directory=tmp_path/'history'
    original=original_history(directory)
    raw=(directory/'policy_command_events.jsonl').read_bytes()
    manifest=(directory/'policy_history_manifest.json').read_bytes()
    with pytest.raises(ValueError): freeze_history(directory, tmp_path/'full.json')
    value=freeze_history(directory, tmp_path/'prefix.json', source_ticks=10)
    assert value['actual_source_ticks']==10 and value['duration_ns']==20_000_000
    assert value['original_command_events_sha256']==hashlib.sha256(raw).hexdigest()
    assert value['original_history_manifest_sha256']==hashlib.sha256(manifest).hexdigest()
    assert value['events']==[json.loads(line) for line in raw.splitlines() if json.loads(line)['source_tick']<10]
    assert value['prefix_metadata']==dict(kind='original_actual_500hz_source_prefix', explicit_prefix=True,
        original_source_tick_count=14, selected_source_tick_count=10, excluded_tail_tick_count=4,
        source_origin_ns=0, excluded_events_count=1, timestamps_modified=False, original_records_modified=False)
    assert FrozenCommands(value,'a'*64,10).at_tick(9)['command']==[.1,0.]
    assert (directory/'policy_command_events.jsonl').read_bytes()==raw
    assert (directory/'policy_history_manifest.json').read_bytes()==manifest
    assert original['actual_source_ticks']==14


@pytest.mark.parametrize('count',[False,0,-10,11,20,10.])
def test_bad_explicit_prefix_never_fills_or_retimes_original_history(tmp_path,count):
    directory=tmp_path/'history';original_history(directory)
    with pytest.raises(ValueError):freeze_history(directory,tmp_path/'out.json',source_ticks=count)
    assert not (tmp_path/'out.json').exists()


@pytest.mark.parametrize('bad',['unsealed_tail','override_tail','rollback_tail','bad_command_tail','manifest_count'])
def test_excluded_tail_is_still_validated_as_original_evidence(tmp_path,bad):
    directory=tmp_path/'history';original_history(directory)
    manifest_path=directory/'policy_history_manifest.json'
    manifest=json.loads(manifest_path.read_bytes())
    raw_path=directory/'policy_command_events.jsonl'
    rows=[json.loads(line) for line in raw_path.read_bytes().splitlines()]
    if bad=='override_tail':rows[-1]['authority']['policy_input_override']=[.4,0.,0.]
    elif bad=='rollback_tail':rows[-1]['source_tick']=0;rows[-1]['source_ns']=0
    elif bad=='bad_command_tail':rows[-1]['command']=[.150001,0.]
    elif bad=='manifest_count':manifest['last_source_tick']=12
    else:rows[-1]['command']=[.1,0.]
    raw_path.write_text('\n'.join(json.dumps(row) for row in rows)+'\n')
    if bad!='unsealed_tail':manifest['command_events_sha256']=hashlib.sha256(raw_path.read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError):freeze_history(directory,tmp_path/'out.json',source_ticks=10)


def test_aligned_complete_baseline_is_still_the_default(tmp_path):
    directory=tmp_path/'history';original_history(directory,20)
    value=freeze_history(directory,tmp_path/'complete.json')
    assert value['actual_source_ticks']==20
    assert value['prefix_metadata']['kind']=='original_complete_500hz_source_history'
    assert not value['prefix_metadata']['explicit_prefix']
    assert value['prefix_metadata']['excluded_tail_tick_count']==0


def test_memory_experiment_is_runtime_local_copy_and_baseline_defaults_unchanged():
    source=dict(kind='official_spot_physx', max_linear_speed=.25,
        stall_recovery=dict(schema=1,enabled=False,isolated_fixture=False), nested=dict(value=[1,2]))
    before=copy.deepcopy(source)
    baseline=component_runtime_robot(source,frozen_file='frozen.json')
    alternate=component_runtime_robot(source,frozen_file='frozen.json',action_memory_recovery=True)
    assert source==before and baseline==before
    assert alternate['stall_recovery']==dict(schema=1,enabled=True,isolated_fixture=True)
    expected=copy.deepcopy(source);expected['stall_recovery']=alternate['stall_recovery']
    assert alternate==expected
    alternate['nested']['value'][0]=3
    assert source==before and baseline==before


def test_missing_source_recovery_uses_original_false_default_without_rewriting_source():
    source=dict(kind='official_spot_physx')
    alternate=component_runtime_robot(source,frozen_file='frozen.json',action_memory_recovery=True)
    assert source==dict(kind='official_spot_physx') and alternate['stall_recovery']['enabled'] is True


def test_enabled_original_recovery_is_replayed_exactly_without_new_intervention():
    source = dict(kind='official_spot_physx', stall_recovery=dict(schema=1, enabled=True,
        isolated_fixture=True), nested=dict(values=[1, 2]))
    before = copy.deepcopy(source)
    runtime = component_runtime_robot(source, frozen_file='original_frozen.json')
    assert runtime == before and source == before
    runtime['nested']['values'][0] = 3
    assert source == before


@pytest.mark.parametrize('options', [dict(), dict(action_memory_recovery=True, frozen_file='f'),
    dict(policy_speed=.4), dict(navigation_speed=.23),
    dict(frozen_file='f', policy_speed=.4), dict(frozen_file='f', navigation_speed=.23)])
def test_enabled_original_recovery_cannot_be_calibrated_or_intervened_again(options):
    source = dict(kind='official_spot_physx', stall_recovery=dict(schema=1, enabled=True,
        isolated_fixture=True))
    with pytest.raises(ValueError, match='enabled_original_memory_recovery_requires'):
        component_runtime_robot(source, **options)


@pytest.mark.parametrize('enabled', [False, True])
def test_scene_component_manifest_records_original_resolved_configuration(enabled):
    from stall_recovery import enabled_for
    path = Path(__file__).with_name('scene.py')
    tree = ast.parse(path.read_text())
    assignment = next(node for node in tree.body if isinstance(node, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == 'COMPONENT_EXPERIMENT' for t in node.targets))
    robot = dict(kind='official_spot_physx', stall_recovery=dict(schema=1, enabled=enabled,
        isolated_fixture=enabled))
    namespace = dict(CONFIG={'robot':robot}, IS_POLICY_COMPONENT=True, RUNTIME_ROBOT_CONFIG=robot,
        ARGS=SimpleNamespace(test_action_memory_recovery=False), source_stall_recovery_enabled=enabled_for)
    exec(compile(ast.Module(body=[assignment], type_ignores=[]), str(path), 'exec'), namespace)
    manifest = namespace['COMPONENT_EXPERIMENT']
    assert manifest['source_resolved_stall_recovery_enabled'] is enabled
    assert manifest['source_spec_stall_recovery'] == robot['stall_recovery']
    assert manifest['runtime_local_override'] is None
    assert manifest['source_scene_bytes_modified'] is False


def test_cached_actor_hook_preserves_original_source_plus_dt_and_prepolicy_phase():
    path = Path(__file__).with_name('scene.py')
    tree = ast.parse(path.read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    register = next(node for node in calls if any(isinstance(k.value, ast.Call)
        and k.arg == 'on_update' and any(isinstance(a, ast.Constant)
            and a.value == 'dynamic_actor_updates' for a in k.value.args) for k in node.keywords))
    values = {key.arg:key.value for key in register.keywords}
    assert ast.literal_eval(values['pre_step']) is True
    assert ast.literal_eval(values['order']) == -1
    callback = values['on_update'].args[1]
    assert isinstance(callback, ast.Lambda) and callback.body.func.id == 'actor_updater'
    clock = ast.Expression(body=callback.body.args[0])
    manager = SimpleNamespace(get_simulation_time=lambda: 123.5)
    assert eval(compile(clock, str(path), 'eval'), dict(SimulationManager=manager,
        source_start_time=123., step_dt=.002)) == .502


@pytest.mark.parametrize('enabled,frozen,policy,nav',[(True,None,None,None),(True,'f',.4,None),
    (True,'f',None,.25),('true','f',None,None)])
def test_experiment_cannot_mix_external_or_policy_calibration_modes(enabled,frozen,policy,nav):
    with pytest.raises(ValueError):validate_action_memory_experiment(enabled,frozen,policy,nav)


@pytest.mark.parametrize('source',[dict(kind='wheel'),dict(kind='official_spot_physx',
    stall_recovery=dict(schema=1,enabled=True,isolated_fixture=True)),
    dict(kind='official_spot_physx',stall_recovery=dict(schema=1,enabled='false',isolated_fixture=True))])
def test_source_recovery_enabled_or_malformed_is_not_an_ab_baseline(source):
    with pytest.raises(ValueError):component_runtime_robot(source,frozen_file='frozen',action_memory_recovery=True)


def parse_scene_arguments(monkeypatch,arguments):
    path=Path(__file__).with_name('scene.py')
    tree=ast.parse(path.read_text())
    function=next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=='parse_args')
    module=ast.Module(body=[function],type_ignores=[])
    namespace=dict(argparse=argparse,Path=Path,math=math,HERE=path.parent,
        COMMAND_PORT=1234,STATE_PORT=1235,__doc__='test')
    monkeypatch.setattr(sys,'argv',['scene.py',*arguments])
    exec(compile(module,str(path),'exec'),namespace)
    return namespace['parse_args']()


def test_scene_cli_explicit_experiment_is_only_frozen_component(monkeypatch):
    with pytest.raises(SystemExit):parse_scene_arguments(monkeypatch,['--test-action-memory-recovery'])
    with pytest.raises(SystemExit):parse_scene_arguments(monkeypatch,['--test-action-memory-recovery',
        '--test-frames','6000','--test-policy-linear-speed','.4','--test-policy-motion-frames','4000'])
    value=parse_scene_arguments(monkeypatch,['--test-action-memory-recovery',
        '--frozen-command-file','frozen.json','--test-frames','20'])
    assert value.test_action_memory_recovery and value.frozen_command_file==Path('frozen.json')
    baseline=parse_scene_arguments(monkeypatch,['--frozen-command-file','frozen.json','--test-frames','20'])
    assert not baseline.test_action_memory_recovery


def test_launcher_ab_has_exact_scene_clock_commands_and_only_one_memory_factor(tmp_path):
    session=tmp_path/'session';session.mkdir()
    scene=session/'scene.json'
    scene.write_text(json.dumps(dict(robot=dict(kind='official_spot_physx',max_linear_speed=.15,
        max_angular_speed=.3),physics=dict(frequency_hz=500))))
    sha=hashlib.sha256(scene.read_bytes()).hexdigest()
    (session/'session.json').write_text(json.dumps(dict(id='original',isaac_bridge_contract=dict(
        scene_config=str(scene),scene_sha256=sha,clock_anchor_ns=123456))))
    history=tmp_path/'history';original_history(history,20)
    manifest=json.loads((history/'policy_history_manifest.json').read_bytes());manifest['scene_sha256']=sha
    (history/'policy_history_manifest.json').write_text(json.dumps(manifest))
    frozen=tmp_path/'frozen.json';freeze_history(history,frozen)
    original_scene,original_commands=scene.read_bytes(),frozen.read_bytes()
    baseline=scene_command(session,tmp_path/'baseline',frozen_file=frozen)
    alternate=scene_command(session,tmp_path/'alternate',frozen_file=frozen,action_memory_recovery=True)
    assert '--test-action-memory-recovery' not in baseline
    assert alternate[-1]=='--test-action-memory-recovery'
    alternate=alternate[:-1];alternate[alternate.index('--result-dir')+1]=baseline[baseline.index('--result-dir')+1]
    assert alternate==baseline
    assert scene.read_bytes()==original_scene and frozen.read_bytes()==original_commands
    for forbidden in ('--test-policy-linear-speed','--test-navigation-linear-speed','--control-file'):
        assert forbidden not in baseline


def test_v3_experiment_is_one_local_controller_factor_with_original_selected_memory():
    source = dict(kind='official_spot_physx', velocity_feedback_mode='spot_monotone_measured_v2',
        stall_recovery=dict(schema=1, enabled=True, isolated_fixture=True), nested=dict(value=[1,2]))
    before = copy.deepcopy(source)
    baseline = component_runtime_robot(source, frozen_file='frozen.json')
    alternate = component_runtime_robot(source, frozen_file='frozen.json',
        velocity_feedback_mode='spot_monotone_measured_v3')
    assert source == before and baseline == before
    expected = copy.deepcopy(before);expected['velocity_feedback_mode']='spot_monotone_measured_v3'
    assert alternate == expected
    assert alternate['stall_recovery'] == before['stall_recovery']
    alternate['nested']['value'][0]=3
    assert source == before


@pytest.mark.parametrize('options', [dict(frozen_file=None), dict(frozen_file='f',policy_speed=.4),
    dict(frozen_file='f',navigation_speed=.23), dict(frozen_file='f',action_memory_recovery=True),
    dict(frozen_file='f',velocity_feedback_mode='spot_monotone_measured_v2'),
    dict(frozen_file='f',velocity_feedback_mode=True)])
def test_v3_experiment_requires_exclusive_frozen_commands(options):
    values=dict(mode='spot_monotone_measured_v3', frozen_file='f')
    values.update(options)
    if 'velocity_feedback_mode' in values:values['mode']=values.pop('velocity_feedback_mode')
    with pytest.raises(ValueError):validate_velocity_feedback_experiment(**values)


@pytest.mark.parametrize('change',[dict(velocity_feedback_mode='legacy_pi_v1'),
    dict(velocity_feedback_mode='spot_monotone_measured_v3'),dict(velocity_feedback=False),
    dict(velocity_feedback='true')])
def test_v3_replay_rejects_unmatched_or_disabled_original_controller(change):
    source=dict(kind='official_spot_physx',velocity_feedback_mode='spot_monotone_measured_v2')
    source.update(change)
    with pytest.raises(ValueError,match='original_enabled_v2'):
        component_runtime_robot(source,frozen_file='f',velocity_feedback_mode='spot_monotone_measured_v3')


def test_v3_scene_cli_is_opt_in_and_rejects_nonfrozen_or_multi_factor(monkeypatch):
    baseline=parse_scene_arguments(monkeypatch,['--frozen-command-file','f','--test-frames','20'])
    assert baseline.test_velocity_feedback_mode is None
    args=['--test-velocity-feedback-mode','spot_monotone_measured_v3']
    with pytest.raises(SystemExit):parse_scene_arguments(monkeypatch,args)
    with pytest.raises(SystemExit):parse_scene_arguments(monkeypatch,args+[
        '--test-frames','20','--test-policy-linear-speed','.4','--test-policy-motion-frames','10'])
    with pytest.raises(SystemExit):parse_scene_arguments(monkeypatch,args+[
        '--frozen-command-file','f','--test-frames','20','--test-action-memory-recovery'])
    selected=parse_scene_arguments(monkeypatch,args+['--frozen-command-file','f','--test-frames','20'])
    assert selected.test_velocity_feedback_mode=='spot_monotone_measured_v3'


def test_v3_full_scene_replay_preserves_scene_hash_schedule_authority_and_clock(tmp_path):
    session=tmp_path/'session';session.mkdir()
    scene=session/'scene.json'
    scene.write_text(json.dumps(dict(robot=dict(kind='official_spot_physx',max_linear_speed=.15,
        max_angular_speed=.3,velocity_feedback_mode='spot_monotone_measured_v2',
        stall_recovery=dict(schema=1,enabled=True,isolated_fixture=True)),physics=dict(frequency_hz=500))))
    sha=hashlib.sha256(scene.read_bytes()).hexdigest()
    (session/'session.json').write_text(json.dumps(dict(id='original',isaac_bridge_contract=dict(
        scene_config=str(scene),scene_sha256=sha,clock_anchor_ns=123456))))
    history=tmp_path/'history';original_history(history,20)
    manifest=json.loads((history/'policy_history_manifest.json').read_bytes());manifest['scene_sha256']=sha
    (history/'policy_history_manifest.json').write_text(json.dumps(manifest))
    frozen=tmp_path/'frozen.json';freeze_history(history,frozen)
    original_scene,original_commands=scene.read_bytes(),frozen.read_bytes()
    baseline=scene_command(session,tmp_path/'baseline',frozen_file=frozen)
    alternate=scene_command(session,tmp_path/'alternate',frozen_file=frozen,
        velocity_feedback_mode='spot_monotone_measured_v3',velocity_feedback_start_tick=12)
    assert alternate[-4:]==['--test-velocity-feedback-mode','spot_monotone_measured_v3',
                           '--test-velocity-feedback-start-tick','12']
    alternate=alternate[:-4];alternate[alternate.index('--result-dir')+1]=baseline[baseline.index('--result-dir')+1]
    assert alternate==baseline
    assert scene.read_bytes()==original_scene and frozen.read_bytes()==original_commands
    for forbidden in ('--test-policy-linear-speed','--test-navigation-linear-speed','--control-file',
                      '--test-action-memory-recovery'):
        assert forbidden not in baseline


@pytest.mark.parametrize('mode,tick,frames',[(None,1,20),('spot_monotone_measured_v3',-1,20),
    ('spot_monotone_measured_v3',True,20),('spot_monotone_measured_v3',20,20),
    ('spot_monotone_measured_v3',21,20),('spot_monotone_measured_v3',1.0,20)])
def test_feedback_activation_is_an_explicit_existing_source_tick(mode,tick,frames):
    with pytest.raises(ValueError,match='activation_requires'):
        validate_velocity_feedback_experiment(mode,'f',activation_tick=tick,frames=frames)


def test_original_bootstrap_and_all_preceding_steps_unchanged_until_exact_begin_activation():
    from quadruped import VelocityFeedback
    servo=VelocityFeedback(.38,.5,mode='spot_monotone_measured_v2')
    servo.integral[:]=[.07,.04]
    servo.filtered=__import__('numpy').array([.02,-.01])
    before_integral=servo.integral.copy();before_filter=servo.filtered.copy()
    manifest=dict(requested=True,activation_source_tick=36428,applied=False,
        requested_mode='spot_monotone_measured_v3',runtime_mode='spot_monotone_measured_v2')
    for phase,tick in [('bootstrap',36428),('source',0),('source',36427)]:
        context=dict(phase=phase,source_tick=tick,source_ns=tick*2_000_000,
            policy_counter=1000+tick,native_physics_tick=1010+tick,
            acquisition_phase='actual_pre_policy_PhysX_BEGIN')
        apply_velocity_feedback_experiment(servo,context,manifest)
        assert servo.mode=='spot_monotone_measured_v2' and not manifest['applied']
    context=dict(phase='source',source_tick=36428,source_ns=72_856_000_000,
        policy_counter=37428,native_physics_tick=37438,
        acquisition_phase='actual_pre_policy_PhysX_BEGIN')
    before_context=copy.deepcopy(context)
    apply_velocity_feedback_experiment(servo,context,manifest)
    assert servo.mode=='spot_monotone_measured_v3' and manifest['applied']
    assert context==before_context
    assert servo.integral.tolist()==before_integral.tolist() and servo.filtered.tolist()==before_filter.tolist()
    assert manifest['actual_activation_source_ns']==72_856_000_000
    assert manifest['actual_activation_policy_counter']==37428
    assert manifest['actual_activation_native_tick']==37438
    # Applying later is idempotent; withdrawal belongs to normal v3 step.
    apply_velocity_feedback_experiment(servo,context,manifest)
    assert servo.integral.tolist()==before_integral.tolist()


@pytest.mark.parametrize('change',[dict(source_tick=36429,source_ns=72_858_000_000),
    dict(source_ns=72_856_000_001),dict(acquisition_phase='actual_post_physics_END'),
    dict(policy_counter=True),dict(native_physics_tick=-1)])
def test_activation_never_retimes_or_accepts_another_acquisition_phase(change):
    servo=SimpleNamespace(mode='spot_monotone_measured_v2')
    manifest=dict(requested=True,activation_source_tick=36428,applied=False)
    context=dict(phase='source',source_tick=36428,source_ns=72_856_000_000,
        policy_counter=37428,native_physics_tick=37438,
        acquisition_phase='actual_pre_policy_PhysX_BEGIN')
    context.update(change)
    with pytest.raises(RuntimeError):apply_velocity_feedback_experiment(servo,context,manifest)
    assert servo.mode=='spot_monotone_measured_v2' and not manifest['applied']


def test_scene_constructor_defers_component_feedback_mode_until_source_boundary():
    path=Path(__file__).with_name('scene.py')
    tree=ast.parse(path.read_text())
    branch=next(node for node in ast.walk(tree) if isinstance(node,ast.If)
        and isinstance(node.test,ast.Compare)
        and isinstance(node.test.left,ast.Attribute)
        and node.test.left.attr=='test_velocity_feedback_mode'
        and any(isinstance(sub,ast.Subscript) and isinstance(sub.slice,ast.Constant)
            and sub.slice.value=='velocity_feedback_mode' for sub in ast.walk(node)))
    original=dict(velocity_feedback_mode='spot_monotone_measured_v2')
    requested=dict(velocity_feedback_mode='spot_monotone_measured_v3')
    namespace=dict(ARGS=SimpleNamespace(test_velocity_feedback_mode='spot_monotone_measured_v3'),
        CONFIG={'robot':original},quad_config=requested)
    exec(compile(ast.Module(body=[branch],type_ignores=[]),str(path),'exec'),namespace)
    assert requested['velocity_feedback_mode']=='spot_monotone_measured_v2'
    assert original==dict(velocity_feedback_mode='spot_monotone_measured_v2')
