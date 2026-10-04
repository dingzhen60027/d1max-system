"""Static fixtures only: no hooks, ROS imports, SDK, or subprocesses."""
import importlib.util
import json
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    'startup_entry_preflight', Path(__file__).with_name('single_floor_entry_preflight.py'))
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


@pytest.fixture(autouse=True)
def clean_loader_overrides(monkeypatch):
    for name in preflight._STARTUP_OVERRIDE:
        monkeypatch.delenv(name, raising=False)


def write(path, text=''):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


def isolated_fixture(root):
    write(root/'local_setup.bash', '# fixture, never sourced\n')
    write(root/'local_setup.sh', '# matching shell loader, never sourced\n')
    write(root/'_local_setup_util_sh.py', 'raise RuntimeError("never execute")\n')
    write(root/'.colcon_install_layout', 'isolated')
    package = root/'example'
    write(package/'share/colcon-core/packages/example', '')
    write(package/'share/ament_index/resource_index/packages/example', '')
    write(package/'share/example/package.dsv', 'source;share/example/hook/path.sh\n')
    write(package/'share/example/hook/path.sh', 'raise never-execute-this-hook\n')
    return package


def test_only_startup_inventory_not_installed_test_payloads(tmp_path):
    root = tmp_path/'install';package = isolated_fixture(root)
    write(package/'share/example/tests/not_runtime.py', 'ignored')
    write(package/'lib/example/program', 'separately sealed executable')
    first = preflight.capture_startup_closure({'overlay': str(root)})
    names = first['files']
    assert str(root/'local_setup.bash') in names
    assert str(root/'_local_setup_util_sh.py') in names
    assert str(package/'share/example/hook/path.sh') in names
    assert str(package/'lib/example/program') not in names
    assert not any('/tests/' in p for p in names)
    assert first['executed_hooks'] is False and first['portable'] is False


def test_merged_ros_prefix_and_recursively_sourced_dsv_are_bound(tmp_path):
    root = tmp_path/'ros';write(root/'local_setup.bash', '# never execute')
    write(root/'local_setup.sh', '# matching shell loader')
    write(root/'_local_setup_util.py', '# helper')
    write(root/'share/ament_index/resource_index/packages/example')
    write(root/'share/example/local_setup.dsv', 'source;share/example/environment/vars.sh\n')
    write(root/'share/example/environment/vars.sh', '# no execution')
    result = preflight.capture_startup_closure({'ros': str(root)})
    assert str(root/'share/example/environment/vars.sh') in result['files']


@pytest.mark.parametrize('kind', ['content', 'index', 'hook'])
def test_existing_mutation_or_new_loader_injection_changes_closure(tmp_path, kind):
    root=tmp_path/'install';package=isolated_fixture(root)
    first=preflight.capture_startup_closure({'overlay': str(root)})
    if kind=='content':
        write(root/'local_setup.bash', '# mutated')
    elif kind=='index':
        write(package/'share/ament_index/resource_index/packages/injected')
    else:
        write(package/'share/example/hook/new-hook.sh', '# injected')
    assert preflight.capture_startup_closure({'overlay': str(root)}) != first


def test_symlink_alias_retarget_detected_even_if_original_target_remains(tmp_path):
    root=tmp_path/'install';package=isolated_fixture(root)
    target1=write(tmp_path/'external1.sh', 'same bytes')
    target2=write(tmp_path/'external2.sh', 'same bytes')
    alias=package/'share/example/hook/from-source.sh'
    alias.symlink_to(target1)
    first=preflight.capture_startup_closure({'overlay': str(root)})
    assert first['files'][str(alias)]['resolved_path']==str(target1)
    alias.unlink();alias.symlink_to(target2)
    second=preflight.capture_startup_closure({'overlay': str(root)})
    assert second != first and target1.exists()


@pytest.mark.parametrize('source', ['../../outside.sh', '/outside/arbitrary.sh',
                                     'share/example/tests/not-startup.sh'])
def test_dsv_cannot_dispatch_an_unwhitelisted_script(tmp_path, source):
    root=tmp_path/'install';package=isolated_fixture(root)
    write(package/'share/example/package.dsv', 'source;'+source+'\n')
    write(package/'share/example/tests/not-startup.sh', '# exists, not a loader')
    with pytest.raises(ValueError, match='startup_dsv_source_'):
        preflight.capture_startup_closure({'overlay': str(root)})


def test_inactive_platform_shell_may_be_absent_but_active_shell_must_exist(tmp_path):
    root=tmp_path/'install';package=isolated_fixture(root)
    dsv=package/'share/example/package.dsv'
    write(dsv, 'source;share/example/local_setup.ps1\n')
    preflight.capture_startup_closure({'overlay': str(root)})
    write(dsv, 'source;share/example/unknown.sh\n')
    with pytest.raises(ValueError, match='not_whitelisted'):
        preflight.capture_startup_closure({'overlay': str(root)})


def test_colcon_dsv_sibling_override_is_included_without_sourcing_shell(tmp_path):
    root=tmp_path/'install';package=isolated_fixture(root)
    write(package/'share/example/package.dsv', 'source;share/example/hook/vars.sh\n')
    dsv=write(package/'share/example/hook/vars.dsv', 'prepend-non-duplicate;PATH;bin\n')
    result=preflight.capture_startup_closure({'overlay': str(root)})
    assert str(dsv) in result['files']


@pytest.mark.parametrize('name', preflight._STARTUP_OVERRIDE)
def test_unsealed_loader_interpreter_or_prefix_override_rejected(tmp_path, monkeypatch, name):
    root=tmp_path/'install';isolated_fixture(root)
    monkeypatch.setenv(name, '/another/interpreter-or-prefix')
    with pytest.raises(ValueError, match='unsealed_startup_override:'+name):
        preflight.capture_startup_closure({'overlay': str(root)})


def test_directory_symlink_cycle_fails_closed(tmp_path):
    root=tmp_path/'install';package=isolated_fixture(root)
    hook=package/'share/example/hook'
    (hook/'cycle').symlink_to(hook, target_is_directory=True)
    with pytest.raises(ValueError, match='symlink_cycle'):
        preflight.capture_startup_closure({'overlay': str(root)})


def test_pre_source_check_binds_alias_inventory_and_fails_before_hook_execution(tmp_path, monkeypatch):
    release=tmp_path/'release';frozen=release/'snapshot/tools'
    for relative in ('single_floor_entry.sh', 'release/single_floor_entry_preflight.py',
                     'validation/bt_localization_replay_entry.sh', 'validation/run_bt_localization_replay.py'):
        write(frozen/relative, '# trusted fixture, never invoked')
    descriptor=write(release/'release.json', json.dumps(dict(schema=1,sealed_manifest='seal.json',
        template_source_root='snapshot',tools_source_root='snapshot/tools')))
    overlay=tmp_path/'install';package=isolated_fixture(overlay)
    prefixes={'overlay': str(overlay)}
    closure=preflight.capture_startup_closure(prefixes)
    files={value['resolved_path']:value['sha256'] for value in closure['files'].values()}
    files[str(descriptor)]=preflight.sha(descriptor)
    for path in frozen.rglob('*'):
        if path.is_file():files[str(path)]=preflight.sha(path)
    write(release/'seal.json',json.dumps(dict(schema=3,files=files,startup_closure=closure)))
    monkeypatch.setenv('D1MAX_NAV_ROOT',str(tmp_path))
    monkeypatch.setenv('D1MAX_RMW_PREFIX',str(tmp_path))
    monkeypatch.setattr(preflight,'startup_prefixes',lambda *_:prefixes)
    assert preflight.verify_startup_closure(release)==closure
    write(package/'share/example/hook/inserted.sh', 'exit 99 # must never execute')
    with pytest.raises(ValueError, match='startup_loader_closure_changed'):
        preflight.verify_startup_closure(release)


def test_new_candidate_without_seal_fails_closed_before_startup(tmp_path):
    release=tmp_path/'release'
    write(release/'release.json',json.dumps(dict(schema=1,sealed_manifest='absent.json',
        template_source_root='snapshot',tools_source_root='snapshot/tools')))
    with pytest.raises(FileNotFoundError):
        preflight.verify_startup_closure(release)


def test_manual_zenoh_loader_prefix_has_no_sourced_setup(tmp_path):
    prefix=tmp_path/'zenoh'
    write(prefix/'share/ament_index/resource_index/packages/rmw_zenoh_cpp')
    closure=preflight.capture_startup_closure({'zenoh_manual_prefix':str(prefix)})
    assert len(closure['files'])==1
    with pytest.raises(ValueError,match='startup_local_setup_missing'):
        preflight.capture_startup_closure({'sourced_overlay':str(prefix)})


def test_actual_workspace_develop_hook_is_declared_without_scanning_build_payload(tmp_path):
    nav=tmp_path/'nav';base=nav/'install';package=isolated_fixture(base)
    external=nav/'build/example/share/example/hook/pythonpath_develop.sh'
    write(external,'# source-develop loader, never invoked')
    write(external.with_suffix('.dsv'),'prepend-non-duplicate;PYTHONPATH;mutable/source\n')
    write(nav/'build/example/unrelated-test.py','not a startup dependency')
    write(package/'share/example/package.dsv',
        'source;../../build/example/share/example/hook/pythonpath_develop.sh\n')
    closure=preflight.capture_startup_closure({'workspace_base':str(base)})
    assert str(external) in closure['files']
    assert closure['external_develop_hook_roots']==[str(nav/'build/example')]
    assert not any('unrelated-test' in p for p in closure['files'])


@pytest.mark.parametrize('relative',[
    '../../build/example/share/foreign/hook/escape.sh',
    '../../build/example/tests/escape.sh',
    '../../arbitrary/run.sh',
])
def test_workspace_develop_source_is_not_an_unbounded_external_allowance(tmp_path,relative):
    nav=tmp_path/'nav';base=nav/'install';package=isolated_fixture(base)
    write(package/'share/example/package.dsv','source;'+relative+'\n')
    with pytest.raises(ValueError,match='startup_dsv_source_not_whitelisted'):
        preflight.capture_startup_closure({'workspace_base':str(base)})


def test_standard_loader_cannot_seal_without_its_sh_loader(tmp_path):
    root=tmp_path/'install';isolated_fixture(root)
    (root/'local_setup.sh').unlink()
    with pytest.raises(ValueError,match='startup_local_setup_sh_missing'):
        preflight.capture_startup_closure({'overlay':str(root)})
