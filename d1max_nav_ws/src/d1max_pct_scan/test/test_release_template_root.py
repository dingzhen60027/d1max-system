"""A release reads its own template bytes, not an editable source checkout."""
import json
from pathlib import Path
import shutil

import pytest

from d1max_pct_planner import paths
from d1max_pct_scan.single_floor_session import prepare, sha

TEMPLATES = (
    'src/d1max_localization/config/localization.yaml',
    'src/d1max_localization/config/global_relocalization.yaml',
    'src/d1max_localization/config/realtime_navigation.yaml',
    'src/d1max_scan_planner/config/d1max_robot.yaml',
    'src/d1max_scan_planner/config/d1max_scan_planner.yaml',
    'src/d1max_navigation_bt/trees/navigate_with_global_relocalization.xml',
    'src/d1max_navigation/rviz/localization_test.rviz',
)


def descriptor(root, **fields):
    root.mkdir(parents=True, exist_ok=True)
    (root/'release.json').write_text(json.dumps(dict(schema=1, **fields)))


def test_source_and_legacy_roots_remain_unchanged(tmp_path, monkeypatch):
    workspace=tmp_path/'workspace';workspace.mkdir()
    monkeypatch.setattr(paths, 'nav_root', lambda:workspace)
    monkeypatch.setattr(paths, 'release_root', lambda:(_ for _ in ()).throw(ValueError('source-only')))
    assert paths.template_root()==workspace
    release=tmp_path/'legacy';descriptor(release)
    assert paths.template_root(release)==workspace


def test_new_root_is_explicit_copied_release_data_not_nav_root(tmp_path, monkeypatch):
    release=tmp_path/'release';descriptor(release, template_source_root='source_snapshot/nav_ws')
    copied=release/'source_snapshot/nav_ws';copied.mkdir(parents=True)
    workspace=tmp_path/'editable';workspace.mkdir()
    monkeypatch.setattr(paths, 'nav_root', lambda:workspace)
    monkeypatch.setenv('D1MAX_RELEASE', str(release))
    assert paths.template_root()==copied
    assert paths.nav_root()==workspace


@pytest.mark.parametrize('relative', ['/tmp/templates', '../outside', 'source_snapshot/../../outside', '.'])
def test_descriptor_cannot_escape_to_checkout(tmp_path, relative):
    release=tmp_path/'release';descriptor(release, template_source_root=relative)
    with pytest.raises(ValueError, match='template_source_root'):
        paths.template_root(release)


def test_missing_or_symlink_escape_never_falls_back(tmp_path, monkeypatch):
    release=tmp_path/'release';descriptor(release, template_source_root='source_snapshot/nav_ws')
    monkeypatch.setattr(paths, 'nav_root', lambda:(_ for _ in ()).throw(AssertionError('mutable fallback')))
    with pytest.raises(FileNotFoundError):paths.template_root(release)
    outside=tmp_path/'outside';outside.mkdir()
    (release/'source_snapshot').mkdir()
    (release/'source_snapshot/nav_ws').symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match='outside_release'):paths.template_root(release)


def test_optional_same_byte_localization_component_root_is_fail_closed(tmp_path):
    release=tmp_path/'release';descriptor(release)
    assert paths.localization_dependencies_root(release) is None
    descriptor(release,localization_dependencies_directory='localization_dependencies/install')
    with pytest.raises(FileNotFoundError):paths.localization_dependencies_root(release)
    root=release/'localization_dependencies/install';root.mkdir(parents=True)
    (root/'local_setup.bash').write_text('# copied loader\n')
    assert paths.localization_dependencies_root(release)==root
    descriptor(release,localization_dependencies_directory='../checkout')
    with pytest.raises(ValueError,match='localization_dependencies_directory'):
        paths.localization_dependencies_root(release)


def test_prepare_pins_all_seven_actual_copied_templates_without_ros(tmp_path, monkeypatch):
    from test_source_identity_package import package
    release, map_directory=package(tmp_path)
    descriptor(release, template_source_root='source_snapshot/nav_ws')
    copied=release/'source_snapshot/nav_ws'
    source=paths.nav_root()
    for relative in TEMPLATES:
        target=copied/relative;target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source/relative, target)
    monkeypatch.setenv('D1MAX_RELEASE', str(release))
    monkeypatch.setattr(paths, 'nav_root', lambda:(_ for _ in ()).throw(AssertionError('prepare reads mutable checkout')))
    def forbidden(*args, **kwargs):raise AssertionError('prepare launched a process')
    monkeypatch.setattr('subprocess.Popen', forbidden)
    session=prepare(tmp_path/'session', map_directory=map_directory, session_id='copied-template-test')
    for relative in TEMPLATES:
        target=copied/relative
        assert not target.is_symlink()
        assert session['input_hashes'][str(target.resolve())]==sha(target)
    assert Path(session['behavior_tree_xml'])==copied/TEMPLATES[5]
    assert Path(session['robot_profile'])==copied/TEMPLATES[3]
    assert Path(session['global_relocalization_template'])==copied/TEMPLATES[1]
    assert Path(session['realtime_navigation_template'])==copied/TEMPLATES[2]
    assert session['physical_acceptance'] is False
