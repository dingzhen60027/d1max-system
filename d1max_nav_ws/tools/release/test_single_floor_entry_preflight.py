"""No graph, robot, RMW initialization, network, or SDK connection."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location('entry_preflight', Path(__file__).with_name('single_floor_entry_preflight.py'))
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


@pytest.fixture
def bundle(tmp_path, monkeypatch):
    nav = tmp_path / 'nav'
    release = nav / 'experiments/release'
    release.mkdir(parents=True)
    (nav / 'deploy').mkdir()
    descriptor = release / 'release.json'
    descriptor.write_text(json.dumps(dict(schema=1, sealed_manifest='seal.json')))
    rmw = tmp_path / 'rmw/lib/librmw_zenoh_cpp.so'
    rmw.parent.mkdir(parents=True)
    rmw.write_bytes(b'fixture-not-an-ELF')
    files = {str(p): entry.sha(p) for p in (descriptor, rmw)}
    seal = release / 'seal.json'
    seal.write_text(json.dumps(dict(schema=3, files=files)))
    profile = dict(schema_version=1, entry_module='d1max_pct_scan.single_floor_session',
                   motion_purpose='execution', release_directory='experiments/release',
                   release_manifest_sha256=entry.sha(seal))
    (nav / 'deploy/single_floor_release.json').write_text(json.dumps(profile))
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')
    monkeypatch.setattr(entry.importlib.util, 'find_spec', lambda name:
                        SimpleNamespace(origin=str(release / 'application/install/pkg/__init__.py')))
    import ament_index_python.packages
    monkeypatch.setattr(ament_index_python.packages, 'get_package_prefix', lambda name: str(rmw.parents[1]))
    loads = []
    monkeypatch.setattr(entry.ctypes, 'CDLL', lambda *a, **kw: loads.append((a, kw)))
    return nav, release, descriptor, seal, rmw, profile, loads


def test_pinned_official_selection_and_library_load_never_authorize_motion(bundle):
    nav, release, _, _, rmw, _, loads = bundle
    assert entry.select(nav) == release
    result = entry.verify(release)
    assert result['rmw'] == 'rmw_zenoh_cpp'
    assert not result['started_processes'] and not result['sdk_connected'] and not result['motion_authorized']
    assert loads[0][0] == (str(rmw),)


def test_selection_is_not_latest_directory_or_a_silent_fallback(bundle):
    nav, release, _, seal, _, _, _ = bundle
    (nav / 'experiments/newer_unsealed').mkdir()
    assert entry.select(nav) == release
    seal.write_text('{}')
    with pytest.raises(ValueError, match='selection_hash_mismatch'):
        entry.select(nav)


@pytest.mark.parametrize('bad', ['../other', '/tmp/release', 'deploy'])
def test_selection_cannot_escape_explicit_release_area(bundle, bad):
    nav, _, _, _, _, profile, _ = bundle
    profile['release_directory'] = bad
    (nav / 'deploy/single_floor_release.json').write_text(json.dumps(profile))
    with pytest.raises(ValueError, match='release_must_be|outside_experiments'):
        entry.select(nav)


def test_changed_or_missing_library_fails_before_dlopen(bundle):
    _, release, _, _, rmw, _, loads = bundle
    rmw.write_bytes(b'changed')
    with pytest.raises(ValueError, match='release_changed'):
        entry.verify(release)
    assert not loads
    rmw.unlink()
    with pytest.raises(OSError):
        entry.verify(release)
    assert not loads


def test_unsealed_rmw_or_development_import_cannot_pass(bundle, monkeypatch):
    _, release, _, seal, rmw, _, loads = bundle
    data = json.loads(seal.read_text())
    del data['files'][str(rmw)]
    seal.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='zenoh_runtime_not_sealed'):
        entry.verify(release)
    monkeypatch.setattr(entry.importlib.util, 'find_spec', lambda name:
                        SimpleNamespace(origin='/workspace/src/d1max_pct_scan/__init__.py'))
    with pytest.raises(ValueError, match='unsealed_python_import'):
        entry.verify(release)
    assert not loads


def test_dds_fallback_is_forbidden(bundle, monkeypatch):
    _, release, _, _, _, _, loads = bundle
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    with pytest.raises(ValueError, match='unchanged_zenoh_required'):
        entry.verify(release)
    assert not loads


def test_library_paths_with_spaces_are_not_truncated():
    output = ('\tlibzenohc.so => /home/dndx/智元四足机器人D1 Max/local/lib/libzenohc.so (0x00001)\n'
              '\t/lib64/ld-linux-x86-64.so.2 (0x00002)\n'
              '\tlinux-vdso.so.1 (0x00003)\n')
    assert list(entry.elf_dependency_paths(output)) == [
        Path('/home/dndx/智元四足机器人D1 Max/local/lib/libzenohc.so'),
        Path('/lib64/ld-linux-x86-64.so.2')]


def test_copied_localization_component_requires_actual_selected_prefixes(tmp_path):
    release=tmp_path/'release';root=release/'localization_dependencies/install'
    root.mkdir(parents=True)
    descriptor=release/'release.json'
    descriptor.write_text(json.dumps(dict(schema=1,localization_dependencies_directory='localization_dependencies/install')))
    setup=root/'local_setup.bash';setup.write_text('# copied setup\n')
    provenance=root.parent/'runtime_copy_provenance.json';provenance.write_text('{}')
    for name in ('faster_lio','livox_ros_driver2'):(root/name).mkdir()
    files={str(p):entry.sha(p) for p in (setup,provenance)}
    assert entry.localization_dependencies(release)==root
    entry.verify_localization_dependencies(release,files,lambda name:str(root/name))
    mutable=tmp_path/'mutable/install'
    for name in ('faster_lio','livox_ros_driver2'):
        (mutable/name).mkdir(parents=True)
    with pytest.raises(ValueError,match='unreleased_localization_dependency:faster_lio'):
        entry.verify_localization_dependencies(release,files,lambda name:str(mutable/name))
    with pytest.raises(ValueError,match='component_not_sealed'):
        entry.verify_localization_dependencies(release,{},lambda name:str(root/name))


@pytest.mark.parametrize('relative',['../outside','/tmp/component','.'])
def test_localization_component_escape_is_not_legacy_fallback(tmp_path,relative):
    release=tmp_path/'release';release.mkdir()
    (release/'release.json').write_text(json.dumps(dict(localization_dependencies_directory=relative)))
    with pytest.raises(ValueError,match='localization_dependencies'):
        entry.localization_dependencies(release)


def test_legacy_descriptor_does_not_require_a_new_component(tmp_path):
    release=tmp_path/'release';release.mkdir()
    (release/'release.json').write_text('{}')
    assert entry.localization_dependencies(release) is None
    entry.verify_localization_dependencies(release,{},lambda _:(_ for _ in ()).throw(AssertionError('legacy prefix query')))


def frozen_tools(tmp_path):
    release=tmp_path/'release';tools=release/'source_snapshot/nav_ws/tools'
    tools.mkdir(parents=True)
    (release/'release.json').write_text(json.dumps(dict(schema=1,
        template_source_root='source_snapshot/nav_ws',tools_source_root='source_snapshot/nav_ws/tools')))
    for name in ('single_floor_entry.sh','release/single_floor_entry_preflight.py',
                 'validation/bt_localization_replay_entry.sh','validation/run_bt_localization_replay.py'):
        p=tools/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('# actual frozen tool\n')
    return release,tools


def test_candidate_tools_are_real_frozen_actual_paths(tmp_path):
    release,tools=frozen_tools(tmp_path)
    assert entry.tools_root(release)==tools
    (tools/'validation/run_bt_localization_replay.py').unlink()
    with pytest.raises(ValueError,match='frozen_entry_tool_missing'):
        entry.tools_root(release)


@pytest.mark.parametrize('kind',['escape','absolute','root_symlink','file_symlink'])
def test_candidate_tool_path_escape_or_symlink_cannot_fall_back(tmp_path,kind):
    release,tools=frozen_tools(tmp_path)
    d=json.loads((release/'release.json').read_text())
    if kind=='escape':d['tools_source_root']='../tools'
    elif kind=='absolute':d['tools_source_root']=str(tools)
    elif kind=='root_symlink':
        link=release/'alias';link.symlink_to(tools,target_is_directory=True);d['tools_source_root']='alias'
    else:
        p=tools/'single_floor_entry.sh';p.unlink();p.symlink_to(tools/'validation/run_bt_localization_replay.py')
    (release/'release.json').write_text(json.dumps(d))
    with pytest.raises(ValueError,match='tools_source_root|frozen_entry_tool_missing_or_symlink'):
        entry.tools_root(release)


def test_new_template_descriptor_cannot_use_mutable_legacy_tools(tmp_path):
    release,tools=frozen_tools(tmp_path)
    (release/'release.json').write_text(json.dumps(dict(template_source_root='source_snapshot/nav_ws')))
    with pytest.raises(ValueError,match='requires_frozen_tools'):
        entry.tools_root(release,tmp_path)
