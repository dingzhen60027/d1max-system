"""Actual file-only CLI/Web/SDK checks; never launch ROS, SDK or a service.

The single cross-entry matrix also locks the three intentionally stdlib-only
boundary classifiers. A release helper is not imported ahead of verification.
RUNTIME_CACHE_BEFORE may point at an archived before/ for same-input comparison.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import py_compile
import subprocess
import sys
import types

import pytest

ROOT = Path(__file__).resolve().parents[2]
APP = Path(os.environ.get('D1MAX_APP_ROOT',
    '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2'))
BEFORE = os.environ.get('RUNTIME_CACHE_BEFORE')


def load(name, relative, group):
    path = (Path(BEFORE)/group/Path(relative).name if BEFORE else
            (ROOT if group == 'nav' else APP)/relative)
    spec = importlib.util.spec_from_file_location('cache_check_'+name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ENTRY = load('entry', 'tools/release/single_floor_entry_preflight.py', 'nav')
WEB = load('web', 'map_manager/backend/mainline_release.py', 'app')
SDK = load('sdk', 'foxglove_d1max/scripts/sdk_motion_startup.py', 'app')
# Sealer deliberately imports the matching before/after entry checker.
sys.modules['single_floor_entry_preflight'] = ENTRY
SEAL = load('seal', 'tools/release/seal_single_floor_release.py', 'nav')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def bundle(tmp_path, monkeypatch):
    nav = tmp_path/'nav'
    release = nav/'experiments/release'
    release.mkdir(parents=True)
    descriptor = release/'release.json'
    descriptor.write_text(json.dumps(dict(schema=1, sealed_manifest='seal.json',
        entry_module='d1max_pct_scan.single_floor_session')))
    tools = nav/'tools'
    members = [descriptor]
    for relative in ('single_floor_entry.sh', 'release/single_floor_entry_preflight.py'):
        path = tools/relative; path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('# fixture, never executed\n'); members.append(path)
    binary = release/SDK.RELEASE_MONITOR
    binary.parent.mkdir(parents=True); binary.write_bytes(b'\x7fELF-fixture-no-execution')
    rmw = tmp_path/'rmw/lib/librmw_zenoh_cpp.so'
    rmw.parent.mkdir(parents=True); rmw.write_bytes(b'\x7fELF-fixture-no-dlopen')
    source = release/'application/install/pkg/cache_fixture.py'
    source.parent.mkdir(parents=True)
    source.write_text("VALUE='old'\n")
    cache = Path(py_compile.compile(str(source), doraise=True,
        invalidation_mode=py_compile.PycInvalidationMode.CHECKED_HASH))
    # The historical error: install new .py, retain previous checked-hash .pyc.
    source.write_text("VALUE='new reviewed source'\n")
    members.extend((binary, rmw, source, cache))
    manifest = dict(schema=3, files={str(p):digest(p) for p in members})
    seal = release/'seal.json'; seal.write_text(json.dumps(manifest))
    packages = types.ModuleType('ament_index_python.packages')
    packages.get_package_prefix = lambda _: str(rmw.parents[1])
    ament = types.ModuleType('ament_index_python'); ament.packages = packages
    monkeypatch.setitem(sys.modules, 'ament_index_python', ament)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')
    monkeypatch.setattr(ENTRY.importlib.util, 'find_spec', lambda _:
        types.SimpleNamespace(origin=str(source.parent/'__init__.py')))
    monkeypatch.setattr(ENTRY.ctypes, 'CDLL', lambda *a, **k: None)
    return dict(nav=nav, release=release, seal=seal, source=source, cache=cache,
                manifest=manifest, binary=binary, rmw=rmw)


def verify(which, b):
    if which == 'cli':
        return ENTRY.verify(b['release'])
    if which == 'web':
        return WEB.verify_release_files(b['nav'], b['release'], b['seal'])
    return SDK.sealed_release_monitor(str(b['release']))


def refresh_by_real_import(b):
    old = b['cache'].read_bytes()
    environment = dict(os.environ); environment.pop('PYTHONDONTWRITEBYTECODE', None)
    subprocess.run([sys.executable, '-I', '-c',
        'import sys;sys.path.insert(0,sys.argv[1]);import cache_fixture;'
        'assert cache_fixture.VALUE=="new reviewed source"', str(b['source'].parent)],
        check=True, env=environment, timeout=10)
    assert b['cache'].read_bytes() != old
    assert digest(b['source']) == b['manifest']['files'][str(b['source'])]


@pytest.mark.parametrize('which', ['cli', 'web', 'sdk'])
def test_first_start_import_and_reconnect_do_not_reject_reviewed_source(bundle, which):
    verify(which, bundle)
    refresh_by_real_import(bundle)
    result = verify(which, bundle)
    if isinstance(result, dict):
        assert result['ignored_runtime_caches'] == 1
    bundle['cache'].unlink()  # cache cleanup is also not a source deployment.
    verify(which, bundle)


@pytest.mark.parametrize('which', ['cli', 'web', 'sdk'])
@pytest.mark.parametrize('member', ['source', 'binary', 'rmw'])
def test_source_and_native_changes_still_reject_after_cache_refresh(bundle, which, member):
    refresh_by_real_import(bundle)
    bundle[member].write_bytes(bundle[member].read_bytes()+b'\nchanged\n')
    with pytest.raises((ValueError, RuntimeError, OSError)) as error:
        verify(which, bundle)
    # Reject the real changed input, not the harmless refreshed cache.
    assert str(bundle[member]) in str(error.value)


@pytest.mark.parametrize('which', ['cli', 'web', 'sdk'])
@pytest.mark.parametrize('relative', ['orphan.pyc', 'orphan.pyo',
    '__pycache__/native.so', '__pycache__/limits.yaml', '__pycache__/plain-data'])
def test_no_directory_or_sourceless_bytecode_exemption(bundle, which, relative):
    path = bundle['release']/relative; path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'original')
    bundle['manifest']['files'][str(path)] = digest(path)
    bundle['seal'].write_text(json.dumps(bundle['manifest']))
    path.write_bytes(b'changed')
    with pytest.raises((ValueError, RuntimeError, OSError)) as error:
        verify(which, bundle)
    assert str(path) in str(error.value)


@pytest.mark.parametrize('module', [ENTRY, WEB, SDK])
@pytest.mark.parametrize('relative', ['fixture.pyc', 'fixture.pyo',
    '__pycache__/fixture.cpython-310.pyc', '__pycache__/fixture.cpython-310.opt-1.pyc',
    '__pycache__/fixture.cpython-310.pyo'])
def test_classifiers_agree_only_existing_sealed_source_is_derived(tmp_path, module, relative):
    source = tmp_path/'fixture.py'; source.write_text('value=1')
    cache = tmp_path/relative; cache.parent.mkdir(parents=True, exist_ok=True)
    cache.write_bytes(b'cached')
    files = {str(source):digest(source), str(cache):digest(cache)}
    assert module.source_backed_python_cache(cache, files)
    assert not module.source_backed_python_cache(cache, {str(cache):digest(cache)})
    source.unlink()
    assert not module.source_backed_python_cache(cache, files)


@pytest.mark.parametrize('which', ['cli', 'web', 'sdk'])
@pytest.mark.parametrize('kind', ['dotted_launch', 'pytest_rewrite'])
def test_real_dotted_and_pytest_cache_reconnect(bundle, which, kind):
    root = bundle['source'].parent
    source = root/('navigation.launch.py' if kind == 'dotted_launch' else 'test_navigation.py')
    source.write_text('def test_local_only():\n    assert 2 + 2 == 4\n')
    if kind == 'dotted_launch':
        cache = Path(py_compile.compile(str(source), doraise=True))
    else:
        environment = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
        environment.pop('PYTHONDONTWRITEBYTECODE', None)
        subprocess.run([sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
            str(source)], check=True, env=environment, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, timeout=20)
        cache, = (root/'__pycache__').glob('test_navigation.*-pytest-*.pyc')
    bundle['manifest']['files'].update({str(source):digest(source), str(cache):digest(cache)})
    bundle['seal'].write_text(json.dumps(bundle['manifest']))
    verify(which, bundle)
    cache.unlink()  # Pytest/import cache eviction must not change a release.
    verify(which, bundle)
    assert str(cache) not in SEAL.without_python_caches(bundle['manifest']['files'])
    source.write_text('changed reviewed source\n')
    with pytest.raises((ValueError, RuntimeError, OSError)) as error:
        verify(which, bundle)
    assert str(source) in str(error.value)


@pytest.mark.parametrize('module', [ENTRY, WEB, SDK])
@pytest.mark.parametrize('stem,tag', [
    ('navigation.launch', 'cpython-310'),
    ('test_navigation', 'cpython-310-pytest-6.2.5'),
    ('test_navigation.launch', 'cpython-310-pytest-8.3.4'),
    ('navigation.launch', 'cpython-310.opt-1')])
def test_dotted_tag_still_requires_sealed_source(tmp_path, module, stem, tag):
    source = tmp_path/(stem+'.py'); source.write_text('x=1')
    cache = tmp_path/'__pycache__'/(stem+'.'+tag+'.pyc')
    cache.parent.mkdir(); cache.write_bytes(b'bytecode')
    files={str(source):digest(source), str(cache):digest(cache)}
    assert module.source_backed_python_cache(cache, files)
    assert not module.source_backed_python_cache(cache, {str(cache):digest(cache)})
    source.unlink()
    assert not module.source_backed_python_cache(cache, files)


@pytest.mark.parametrize('module', [ENTRY, WEB, SDK])
def test_cache_symlink_is_not_runtime_bytecode_exemption(tmp_path, module):
    source=tmp_path/'fixture.py';source.write_text('value=1')
    target=tmp_path/'native.so';target.write_bytes(b'\x7fELF')
    cache=tmp_path/'fixture.pyc';cache.symlink_to(target)
    assert not module.source_backed_python_cache(cache, {str(source):digest(source)})


def test_sealer_filters_complete_inventory_but_preserves_every_other_file(bundle):
    members = dict(bundle['manifest']['files'])
    unusual = bundle['cache'].parent/'native.so';unusual.write_bytes(b'\x7fELF')
    members[str(unusual)] = digest(unusual)
    result = SEAL.without_python_caches(members)
    assert set(result) == set(members)-{str(bundle['cache'])}
    assert members[str(bundle['cache'])] == bundle['manifest']['files'][str(bundle['cache'])]
    snapshot = set(SEAL.snapshot_source_files(bundle['source'].parent))
    assert snapshot == {bundle['source'], unusual}
