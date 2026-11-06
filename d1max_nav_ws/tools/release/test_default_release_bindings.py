import importlib.util
import json
from pathlib import Path
import stat

import pytest

spec = importlib.util.spec_from_file_location('default_release_bindings',
    Path(__file__).with_name('default_release_bindings.py'))
binding = importlib.util.module_from_spec(spec)
spec.loader.exec_module(binding)


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def fixture(tmp_path):
    nav = tmp_path / 'nav'
    release = nav / 'experiments/navigation_runtime'
    files = {}
    for name in ('navigation_entry.sh', 'single_floor_entry.sh'):
        for path in (nav / 'tools' / name, release / 'source_snapshot/nav_ws/tools' / name):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#!/bin/sh\n# reviewed ' + name + '\n')
            path.chmod(0o755)
        files[str(path)] = binding.digest(path)
    seal = release / 'release_manifest.json'
    write(seal, dict(files=files, physical_acceptance=False))
    expected = binding.digest(seal)
    selector = nav / 'deploy/single_floor_release.json'
    write(selector, dict(release_directory='experiments/navigation_runtime',
                         release_manifest_sha256='before', physical_acceptance=False))
    active = tmp_path / 'private/activation.json'
    write(active, dict(release_root=str(release), release_manifest=str(seal),
        release_manifest_sha256='before', entrypoint=str(nav / 'tools/navigation_entry.sh'),
        entrypoint_sha256='stale', compatibility_entrypoint_sha256='stale',
        launch_policy='navigation', expected_sdk_session='unchanged'))
    active.chmod(0o600)
    return nav, release, selector, active, expected


def test_updates_web_transitive_entry_and_cli_together_preserving_policy(tmp_path):
    nav, release, selector, active, expected = fixture(tmp_path)
    old = binding.read(active)
    with pytest.raises(ValueError, match='derived_bindings_not_synchronized'):
        binding.synchronize(nav, active, expected, previous_manifest='before')
    binding.synchronize(nav, active, expected, previous_manifest='before', backup=tmp_path/'backup')
    binding.synchronize(nav, active, expected)
    assert binding.read(selector)['release_manifest_sha256'] == expected
    new = binding.read(active)
    assert new['compatibility_entrypoint_sha256'] == binding.digest(nav/'tools/single_floor_entry.sh')
    assert new['entrypoint_sha256'] == binding.digest(nav/'tools/navigation_entry.sh')
    assert {k:v for k,v in new.items() if not k.endswith('sha256')} == {
        k:v for k,v in old.items() if not k.endswith('sha256')}
    assert stat.S_IMODE(active.stat().st_mode) == 0o600
    assert binding.read(tmp_path/'backup/1-activation.json') == old


@pytest.mark.parametrize('kind', ['root_changed', 'snapshot_changed', 'manifest_changed',
    'selector_changed', 'web_other_release', 'permissions', 'root_symlink'])
def test_mismatch_never_silently_rebinds_or_writes(tmp_path, kind):
    nav, release, selector, active, expected = fixture(tmp_path)
    if kind == 'root_changed':
        (nav/'tools/single_floor_entry.sh').write_text('unreviewed')
    elif kind == 'snapshot_changed':
        (release/'source_snapshot/nav_ws/tools/single_floor_entry.sh').write_text('unreviewed')
    elif kind == 'manifest_changed':
        (release/'release_manifest.json').write_text('{}')
    elif kind == 'selector_changed':
        write(selector, {**binding.read(selector), 'release_manifest_sha256':'concurrent'})
    elif kind == 'web_other_release':
        write(active, {**binding.read(active), 'release_root':str(tmp_path)})
    elif kind == 'permissions':
        active.chmod(0o644)
    else:
        path = nav/'tools/single_floor_entry.sh'
        path.unlink()
        path.symlink_to(release/'source_snapshot/nav_ws/tools/single_floor_entry.sh')
    before = (selector.read_bytes(), active.read_bytes())
    with pytest.raises(ValueError):
        binding.synchronize(nav, active, expected, previous_manifest='before', backup=tmp_path/'backup')
    assert before == (selector.read_bytes(), active.read_bytes())
    assert not (tmp_path/'backup').exists()


def test_verification_is_read_only_and_repeatable(tmp_path):
    nav, _, selector, active, expected = fixture(tmp_path)
    binding.synchronize(nav, active, expected, previous_manifest='before', backup=tmp_path/'backup')
    before = (selector.stat().st_mtime_ns, active.stat().st_mtime_ns)
    binding.synchronize(nav, active, expected)
    binding.synchronize(nav, active, expected)
    assert before == (selector.stat().st_mtime_ns, active.stat().st_mtime_ns)
