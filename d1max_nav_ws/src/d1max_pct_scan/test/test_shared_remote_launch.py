from pathlib import Path
from d1max_pct_scan.shared_remote_launch import command
from d1max_pct_scan.single_floor_session import component_is_critical


def test_only_existing_live_mainline_opens_shared_panel(tmp_path):
    entry=tmp_path/'gui.sh';entry.touch();entry.chmod(0o700)
    env={'DISPLAY':':0','D1MAX_REMOTE_ROOT':str(tmp_path)}
    session={'transport_mode':'live','execution_architecture':'task_follower_v1'}
    assert command(session,environment=env)==[str(entry),'--shared']
    assert command({**session,'transport_mode':'isolated_mock'},environment=env) is None
    assert command({**session,'execution_architecture':'legacy'},environment=env) is None
    assert command(session,environment={'D1MAX_REMOTE_ROOT':str(tmp_path)}) is None
    assert command({**session,'purpose':'planning_only'},environment=env) is None
    assert command({**session,'execution_purpose':'planning_only'},environment=env) is None
    assert command({**session,'purpose':'execution'},environment=env)==[str(entry),'--shared']


def test_missing_panel_is_not_a_core_failure(tmp_path):
    assert command({'transport_mode':'live','execution_architecture':'task_follower_v1'},
        environment={'DISPLAY':':0','D1MAX_REMOTE_ROOT':str(tmp_path)}) is None
    assert not component_is_critical('manual_remote')
    assert component_is_critical('tracker')


def test_sealed_gui_is_used_without_startup_build_and_explicit_override_wins(tmp_path):
    binary=tmp_path/'release/remote/bin/d1max_remote_gui'
    binary.parent.mkdir(parents=True);binary.touch();binary.chmod(0o700)
    env={'DISPLAY':':0','D1MAX_RELEASE':str(tmp_path/'release')}
    session={'transport_mode':'live','execution_architecture':'task_follower_v1'}
    assert command(session,environment=env)==[str(binary),'--shared']
    override=tmp_path/'override';override.mkdir()
    entry=override/'gui.sh';entry.touch();entry.chmod(0o700)
    assert command(session,environment={**env,'D1MAX_REMOTE_ROOT':str(override)})==[str(entry),'--shared']
    assert command({**session,'purpose':'planning_only'},environment=env) is None


def test_launcher_contains_no_sdk_session_or_robot_commands():
    import d1max_pct_scan.shared_remote_launch as launch
    source=Path(launch.__file__).read_text()
    assert 'SDKClient(' not in source
    assert 'TakeControl(' not in source
    assert 'Move(' not in source
