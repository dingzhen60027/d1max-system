"""Actual passive entry setup: no session, ROS graph, SDK, or network startup.

The selected sealed bundle is a prerequisite. Use D1MAX_ENTRY_TEST_SOURCE to
compare an older entry with the same exact setup hooks and runtime contract.
"""
import json
import os
from pathlib import Path
import shlex
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
RELEASE = ROOT / 'experiments/navigation_runtime'
APP = Path(os.environ.get('D1MAX_APP_ROOT',
    '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2'))
ENTRY = Path(os.environ.get('D1MAX_ENTRY_TEST_SOURCE', ROOT/'tools/single_floor_entry.sh'))
PREFLIGHT = RELEASE/'source_snapshot/nav_ws/tools/release/single_floor_entry_preflight.py'
SAVED = RELEASE/'release_preflight/runtime_inputs.json'


@pytest.fixture(scope='module')
def setup_code():
    if not (SAVED.is_file() and PREFLIGHT.is_file() and Path('/opt/ros/humble/setup.bash').is_file()):
        pytest.skip('Passive installed-bundle entry test requires the selected sealed ROS bundle')
    text = ENTRY.read_text()
    # Run the exact production setup, stopping BEFORE the session entrypoint.
    setup = text[text.index('export PYTHONPATH='):text.index('/usr/bin/python3 "$ENTRY_PREFLIGHT" verify')]
    assert 'single_floor_session' not in setup
    assert 'source /opt/ros/humble/setup.bash' in setup
    return setup


def passive_run(setup, context, overrides=None):
    env = {key:value for key,value in os.environ.items()
        if not key.startswith(('D1MAX_', 'COLCON_', 'AMENT_', 'CMAKE_', 'ROS_', 'RMW_', 'ZENOH_'))
        and key not in ('PYTHONPATH', 'LD_LIBRARY_PATH')}
    env.update(PYTHONDONTWRITEBYTECODE='1', D1MAX_NAV_ROOT=str(ROOT), D1MAX_RELEASE=str(RELEASE),
        D1MAX_APP_ROOT=str(APP), ENTRY_PREFLIGHT=str(PREFLIGHT))
    env.update(overrides or {})
    preludes = {
        'clean':'',
        'application_local':f'source {shlex.quote(str(RELEASE/"application/install/local_setup.bash"))}\n',
        'application_setup':f'source {shlex.quote(str(RELEASE/"application/install/setup.bash"))}\n',
        'repeat_complete_setup':setup+'\n',
    }
    diagnostic = '''import json,os
from pathlib import Path
from d1max_pct_planner import compute_budget,runtime_freeze
from ament_index_python.packages import get_package_share_directory
print('ENTRY_TEST='+json.dumps(dict(module=compute_budget.__file__,
 share=get_package_share_directory('d1max_pct_planner'),
 explicit_config=os.environ.get('D1MAX_PCT_COMPUTE_CONFIG'),
 frozen_budget=os.environ.get('D1MAX_PCT_COMPUTE_BUDGET'))),flush=True)
'''
    command = ('set -eo pipefail\n'+preludes[context]+setup+
        '\n/usr/bin/python3 -B -c '+shlex.quote(diagnostic)+
        '\n/usr/bin/python3 -B "$ENTRY_PREFLIGHT" verify --release "$D1MAX_RELEASE"')
    run = subprocess.run(['bash', '--noprofile', '--norc', '-c', command],
        env=env, text=True, capture_output=True, timeout=45)
    diagnostic_lines = [line[len('ENTRY_TEST='):] for line in run.stdout.splitlines()
                        if line.startswith('ENTRY_TEST=')]
    assert diagnostic_lines, (run.stdout, run.stderr)
    return run, json.loads(diagnostic_lines[-1])


@pytest.mark.parametrize('context', ['clean','application_local','application_setup','repeat_complete_setup'])
def test_entry_selects_same_sealed_runtime_after_supported_shell_setup(setup_code, context):
    run, result = passive_run(setup_code, context)
    assert run.returncode == 0, (run.stdout, run.stderr)
    expected = RELEASE/'application/install/d1max_pct_planner'
    assert Path(result['module']).is_relative_to(expected)
    assert result['share'] == str(expected/'share/d1max_pct_planner')


def test_entry_preserves_explicit_unsealed_compute_override_rejection(setup_code):
    # Identical bytes are still a deliberate different source, not a supported
    # inherited ament index. The entry must NOT erase the explicit override.
    alternate = ROOT/'src/d1max_pct_planner/config/compute_budget.yaml'
    run, result = passive_run(setup_code, 'application_local',
        {'D1MAX_PCT_COMPUTE_CONFIG':str(alternate)})
    assert result['explicit_config'] == str(alternate)
    assert run.returncode == 2
    assert 'compute_budget_runtime_changed_or_unsealed_override' in run.stderr


def test_entry_preserves_invalid_frozen_budget_rejection(setup_code):
    budget = json.loads(SAVED.read_text())['compute_budget']['effective']
    budget['max_cpu_cores'] += 1
    override = json.dumps(budget)
    run, result = passive_run(setup_code, 'application_local', {'D1MAX_PCT_COMPUTE_BUDGET':override})
    assert result['frozen_budget'] == override
    assert run.returncode == 2
    assert 'compute_budget_frozen_override_differs_from_actual_file' in run.stderr
