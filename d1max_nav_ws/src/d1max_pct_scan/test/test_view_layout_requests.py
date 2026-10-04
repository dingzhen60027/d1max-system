"""Filesystem-only display IPC regression, no ROS/SDK or graphical viewer."""
import json
import os

import pytest

from d1max_pct_scan.view_layout_requests import (
    REQUEST_FILE, VIEW_CONTRACT, VIEWER_ENV, viewer_environment, write_layout_request,
)

TOKEN = 'a' * 32


def test_cli_initial_local_is_requested_in_the_same_global_window(tmp_path):
    source = {'ROS_DOMAIN_ID': '147', 'RMW_IMPLEMENTATION': 'rmw_zenoh_cpp'}
    env = viewer_environment(tmp_path, {'id': 'session', 'view_contract': VIEW_CONTRACT},
                             'local', environment=source)
    request = json.loads((tmp_path / REQUEST_FILE).read_text())
    assert request['session_id'] == 'session' and request['layout'] == 'local'
    assert request['viewer_id'] == env[VIEWER_ENV] and len(request['request_id']) == 32
    assert env['ROS_DOMAIN_ID'] == '147' and env['RMW_IMPLEMENTATION'] == 'rmw_zenoh_cpp'
    assert VIEWER_ENV not in source
    assert (tmp_path / REQUEST_FILE).stat().st_mode & 0o777 == 0o600


def test_web_request_is_not_overwritten_when_viewer_loads(tmp_path):
    request = write_layout_request(tmp_path, session_id='session', viewer_id=TOKEN, layout='local')
    env = viewer_environment(tmp_path, {'id': 'session', 'view_contract': VIEW_CONTRACT},
                             'global', environment={VIEWER_ENV: TOKEN})
    assert env[VIEWER_ENV] == TOKEN
    assert json.loads((tmp_path / REQUEST_FILE).read_text()) == request


def test_request_updates_only_presentation_file_and_has_fresh_identity(tmp_path):
    session = tmp_path / 'session.json'; session.write_text('{"sealed":true}')
    first = write_layout_request(tmp_path, session_id='session', viewer_id=TOKEN, layout='global')
    second = write_layout_request(tmp_path, session_id='session', viewer_id=TOKEN, layout='local')
    assert first['request_id'] != second['request_id']
    assert json.loads((tmp_path / REQUEST_FILE).read_text()) == second
    assert session.read_text() == '{"sealed":true}'
    assert not list(tmp_path.glob('.view-layout-*'))


@pytest.mark.parametrize('layout', ['all', '', 'LOCAL', None])
def test_unknown_layout_is_not_a_motion_or_other_command(tmp_path, layout):
    with pytest.raises(ValueError, match='invalid_view_layout_request'):
        write_layout_request(tmp_path, session_id='session', viewer_id=TOKEN, layout=layout)
    assert list(tmp_path.iterdir()) == []


def test_unpinned_old_session_is_not_claimed_to_have_new_plugin(tmp_path):
    with pytest.raises(ValueError, match='single_window_view_contract_required'):
        viewer_environment(tmp_path, {'id': 'legacy'}, 'local', environment={})


def test_foreign_or_symlink_request_is_not_overwritten(tmp_path):
    outside = tmp_path / 'untouched'; outside.write_text('original')
    request = tmp_path / REQUEST_FILE; request.symlink_to(outside)
    with pytest.raises(ValueError, match='view_request_not_private'):
        write_layout_request(tmp_path, session_id='session', viewer_id=TOKEN, layout='local')
    assert outside.read_text() == 'original'
    request.unlink(); request.write_text('public'); request.chmod(0o666)
    with pytest.raises(ValueError, match='view_request_not_private'):
        write_layout_request(tmp_path, session_id='session', viewer_id=TOKEN, layout='global')


def test_invalid_viewer_nonce_is_not_inherited(tmp_path):
    with pytest.raises(ValueError, match='invalid_viewer_identity'):
        viewer_environment(tmp_path, {'id': 'session', 'view_contract': VIEW_CONTRACT},
                           'global', environment={VIEWER_ENV: 'wrong-viewer'})
