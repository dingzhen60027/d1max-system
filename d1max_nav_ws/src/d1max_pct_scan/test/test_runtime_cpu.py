"""CPU placement is optional, bounded and never mutates a running robot graph."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from d1max_pct_scan import runtime_cpu as cpu


def topology(tmp_path, fast='0-3', efficient='4-7', online='0-7'):
    for name, text in (('bus/event_source/devices/cpu_core/cpus', fast),
                       ('bus/event_source/devices/cpu_atom/cpus', efficient),
                       ('devices/system/cpu/online', online)):
        path = tmp_path/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text+'\n')
    for number in range(8):
        path = tmp_path/'devices/system/cpu'/('cpu'+str(number))/'topology'
        path.mkdir(parents=True, exist_ok=True)
        (path/'physical_package_id').write_text('0\n')
        (path/'core_id').write_text(str(number//2)+'\n')
    return tmp_path


@pytest.mark.parametrize('text', ['', '-1', '3-1', '0-4096', 'a', '1,', '1-2-3'])
def test_invalid_lists(text):
    with pytest.raises(ValueError):
        cpu.cpu_list(text)


def test_explicit_hybrid_respects_parent_cpuset_and_offline_cpus(tmp_path):
    root = topology(tmp_path, online='0-2,4-7')
    assert cpu.performance_cpus({1, 2, 3, 5, 6}, sysfs=root) == {1, 2}
    assert cpu.performance_cpus({4, 5}, sysfs=root) is None
    assert cpu.performance_cpus({1, 2}, sysfs=root) is None
    assert cpu.performance_cpus({0, 1, 4, 5}, sysfs=root) is None


def test_no_guess_on_homogeneous_missing_or_conflicting_topology(tmp_path):
    assert cpu.performance_cpus({0, 1}, sysfs=tmp_path) is None
    assert cpu.performance_cpus(set(range(8)), sysfs=topology(tmp_path, efficient='3-7')) is None
    assert cpu.performance_cpus(set(range(8)), sysfs=topology(tmp_path, online='0-3')) is None


def test_denied_placement_keeps_startup_available(tmp_path, monkeypatch):
    monkeypatch.setattr(cpu.os, 'sched_getaffinity', lambda _: set(range(8)))
    def denied(*args):
        raise PermissionError('fixture')
    monkeypatch.setattr(cpu.os, 'sched_setaffinity', denied)
    assert cpu.apply_placement(sysfs=topology(tmp_path))['reason'] == 'placement_unavailable'


def test_only_scan_and_projector_are_selected():
    command = ['/some/program', '--name', 'value with spaces']
    for name in ('sdk', 'tracker', 'localization', 'rviz', 'global', 'navigator'):
        assert cpu.component_command(name, command) is command
    for name in ('scan', 'perception'):
        assert cpu.component_command(name, command)[-len(command):] == command


def test_exec_preserves_arguments_exit_and_parent_affinity():
    before = os.sched_getaffinity(0)
    program = 'import json,os,sys; print(json.dumps(dict(args=sys.argv[1:],cpus=sorted(os.sched_getaffinity(0))))); sys.exit(7)'
    result = subprocess.run(cpu.component_command('perception',
        [sys.executable, '-c', program, 'two words', 'literal;$']),
        capture_output=True, text=True, timeout=10)
    assert result.returncode == 7
    assert os.sched_getaffinity(0) == before
    data = json.loads(result.stdout)
    assert data['args'] == ['two words', 'literal;$']
    assert set(data['cpus']) <= before
    assert 'NavigationCPU ' in result.stderr


def test_production_spawner_uses_same_process_group_and_environment():
    from d1max_pct_scan import single_floor_session
    import inspect
    source = inspect.getsource(single_floor_session._run_locked)
    assert 'subprocess.Popen(component_command(name,cmd)' in source
    assert 'start_new_session=True' in source
    assert 'pass_fds=() if core_fd is None else (core_fd,),env=trace_env' in source
