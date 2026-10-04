"""Public naming regression: no ROS graph, SDK, release activation or replay."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

import pytest

from d1max_pct_scan import navigation_session, single_floor_session


ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('name', navigation_session.__all__)
def test_public_api_is_the_same_implementation(name):
    assert getattr(navigation_session, name) is getattr(single_floor_session, name)


def test_old_and_new_names_cannot_create_two_task_owners(tmp_path, monkeypatch):
    monkeypatch.setattr(single_floor_session, '_core_lock_root', lambda: tmp_path)
    monkeypatch.setenv('ROS_DOMAIN_ID', '147')
    first = tmp_path / 'old'; second = tmp_path / 'new'
    first.mkdir(); second.mkdir()
    with single_floor_session.runtime_lock(first):
        with pytest.raises(ValueError, match='navigation_core_already_running'):
            with navigation_session.runtime_lock(second):
                pytest.fail('renaming bypassed the existing task owner lock')
    with navigation_session.runtime_lock(second):
        pass


def test_public_dispatcher_transmits_arguments_environment_and_exit(tmp_path):
    tools = tmp_path / 'tools with spaces'; tools.mkdir()
    entry = tools / 'navigation_entry.sh'
    entry.write_bytes((ROOT / 'tools/navigation_entry.sh').read_bytes())
    # Only this inert fixture is executed, never the actual release loader.
    fixture = tools / 'single_floor_entry.sh'
    program = ('import json,os,sys; print(json.dumps(dict(args=sys.argv[1:],'
               'release=os.environ["D1MAX_RELEASE"],'
               'domain=os.environ["ROS_DOMAIN_ID"],'
               'transport=os.environ["D1MAX_NAV_TRANSPORT"]))); sys.exit(7)')
    fixture.write_text('#!/usr/bin/env bash\nexec ' + shlex.quote(sys.executable) +
                       ' -c ' + shlex.quote(program) + ' "$@"\n')
    arguments = ['prepare', '--session', str(tmp_path / 'session with spaces'),
                 '--purpose', 'planning_only', '--expected-sdk-session', 'fixture-only']
    environment = dict(os.environ, D1MAX_RELEASE='/fixture/not-a-release',
                       ROS_DOMAIN_ID='147', D1MAX_NAV_TRANSPORT='isolated_mock')
    result = subprocess.run(['bash', str(entry), *arguments], env=environment,
                            capture_output=True, text=True, timeout=5, check=False)
    assert result.returncode == 7
    assert json.loads(result.stdout) == dict(args=arguments, release='/fixture/not-a-release',
                                            domain='147', transport='isolated_mock')


def test_public_name_does_not_relabel_sealed_contract_or_enable_stairs():
    loader = (ROOT / 'tools/single_floor_entry.sh').read_text()
    assert 'exec /usr/bin/python3 -m d1max_pct_scan.single_floor_session "$@"' in loader
    implementation = (ROOT / 'src/d1max_pct_scan/d1max_pct_scan/single_floor_session.py').read_text()
    assert "route.get('stairs_enabled') is not False" in implementation
    assert "pipeline_contract='single_floor_v3'" in implementation
    assert 'single_floor_session' not in navigation_session.__all__
