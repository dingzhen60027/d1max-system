import json
import multiprocessing
import os
import threading
from types import SimpleNamespace

import pytest

from d1max_pct_planner import compute_budget as budget


def test_native_environment_limits_numeric_pools_without_mutating_parent():
    original = {'RMW_IMPLEMENTATION': 'rmw_zenoh_cpp', 'OPENBLAS_NUM_THREADS': '32'}
    result = budget.prepare_environment(original)
    assert original['OPENBLAS_NUM_THREADS'] == '32'
    assert result['RMW_IMPLEMENTATION'] == 'rmw_zenoh_cpp'
    assert result['OPENBLAS_NUM_THREADS'] == result['MKL_NUM_THREADS'] == '1'
    assert result['OMP_NUM_THREADS'] == '2'
    assert result['OMP_DYNAMIC'] == 'FALSE'
    assert result['OMP_MAX_ACTIVE_LEVELS'] == '1'
    assert budget.prepare_environment(result) == result


@pytest.mark.parametrize('change', [dict(max_cpu_cores=0), dict(max_cpu_cores=True),
    dict(cpu_ids=[1, 1]), dict(cpu_ids=[-1]), dict(cpu_ids=[0, 1, 2]),
    dict(omp_threads=3), dict(blas_threads=0), dict(nice=-1), dict(nice=20),
    dict(unknown=1)])
def test_invalid_budget_rejected(change):
    with pytest.raises(ValueError):
        budget.validate_budget({**budget.DEFAULT, **change})


def test_main_process_affinity_and_priority_cannot_be_changed(monkeypatch):
    monkeypatch.setattr(budget.multiprocessing, 'current_process',
                        lambda: SimpleNamespace(name='MainProcess'))
    with pytest.raises(RuntimeError, match='main process'):
        budget.apply_worker_budget()


def test_child_budget_respects_existing_cpuset_and_priority(monkeypatch):
    monkeypatch.setattr(budget.multiprocessing, 'current_process',
                        lambda: SimpleNamespace(name='SpawnProcess-1'))
    monkeypatch.setattr(budget, 'load_budget', lambda: dict(budget.DEFAULT))
    affinity, priority = [{2, 4, 6, 8}], [10]
    monkeypatch.setattr(budget.os, 'sched_getaffinity', lambda _: affinity[0])
    monkeypatch.setattr(budget.os, 'sched_setaffinity', lambda _, cpus: affinity.__setitem__(0,set(cpus)))
    monkeypatch.setattr(budget.os, 'getpriority', lambda *_: priority[0])
    monkeypatch.setattr(budget.os, 'setpriority', lambda _, pid, n: priority.__setitem__(0,n))
    actual = budget.apply_worker_budget()
    assert actual['effective_cpu_ids'] == [2, 4]
    assert actual['effective_nice'] == 10  # Never elevate priority.
    assert actual['exclusive_cores'] is False


def test_unavailable_explicit_cpu_rejected_before_affinity_change(monkeypatch):
    monkeypatch.setattr(budget.multiprocessing, 'current_process',
                        lambda: SimpleNamespace(name='SpawnProcess-1'))
    monkeypatch.setattr(budget, 'load_budget', lambda: {**budget.DEFAULT, 'cpu_ids': [99]})
    monkeypatch.setattr(budget.os, 'sched_getaffinity', lambda _: {0,1})
    monkeypatch.setattr(budget.os, 'sched_setaffinity', lambda *_: pytest.fail('changed affinity'))
    with pytest.raises(RuntimeError, match='cpuset'):
        budget.apply_worker_budget()


def test_explicit_missing_profile_fails_closed():
    with pytest.raises(ValueError, match='absolute YAML'):
        budget.load_budget({'D1MAX_PCT_COMPUTE_CONFIG': '/missing/compute.yaml'})


def test_host_omp_placement_cannot_override_child_budget():
    original = {name: 'host-placement' for name in budget.PLACEMENT_ENV}
    result = budget.prepare_environment(original)
    assert all(name not in result for name in budget.PLACEMENT_ENV)
    assert result['OMP_PROC_BIND'] == 'FALSE'
    assert all(value == 'host-placement' for value in original.values())


def test_spawn_restores_all_environment_keys_on_success_and_failure(monkeypatch):
    monkeypatch.setenv('GOMP_CPU_AFFINITY', '8-16')
    monkeypatch.setenv('OPENBLAS_NUM_THREADS', '32')
    prepared = budget.prepare_environment(os.environ)
    before = dict(os.environ)
    def start():
        assert 'GOMP_CPU_AFFINITY' not in os.environ
        assert os.environ['OPENBLAS_NUM_THREADS'] == '1'
        assert os.environ['OMP_PROC_BIND'] == 'FALSE'
    budget.spawn_with_environment(SimpleNamespace(start=start), prepared)
    assert dict(os.environ) == before
    def fail():
        start()
        raise RuntimeError('spawn failed')
    with pytest.raises(RuntimeError, match='spawn failed'):
        budget.spawn_with_environment(SimpleNamespace(start=fail), prepared)
    assert dict(os.environ) == before


def _report_worker(connection):
    connection.send((sorted(os.sched_getaffinity(0)), os.getpriority(os.PRIO_PROCESS,0)))
    connection.close()


def _report_numeric_worker(connection):
    import numpy as np
    values = np.arange(512*512, dtype=float).reshape(512,512)
    assert np.isfinite(values @ values.T).all()
    threads = {tid: (sorted(os.sched_getaffinity(tid)),
                     os.getpriority(os.PRIO_PROCESS,tid)) for tid in budget._thread_ids()}
    connection.send((threads, {key:os.environ.get(key) for key in budget.THREAD_ENV},
                     {key:os.environ.get(key) for key in budget.PLACEMENT_ENV}))
    connection.close()


def test_new_numeric_threads_inherit_budget_with_host_binding_removed(monkeypatch):
    expected = set(sorted(os.sched_getaffinity(0))[:2])
    monkeypatch.setenv('GOMP_CPU_AFFINITY', ','.join(map(str, sorted(os.sched_getaffinity(0)))))
    environment = budget.prepare_environment(os.environ)
    context = multiprocessing.get_context('spawn')
    parent, child = context.Pipe(duplex=False)
    process = context.Process(target=budget.budgeted_worker, args=(_report_numeric_worker,child))
    budget.spawn_with_environment(process, environment)
    child.close()
    try:
        assert parent.poll(10)
        threads, numerical, placement = parent.recv()
        process.join(5)
        assert process.exitcode == 0
        assert threads and all(set(cpus).issubset(expected) and nice>=5
                               for cpus,nice in threads.values())
        assert numerical['OPENBLAS_NUM_THREADS'] == '1'
        assert numerical['OMP_NUM_THREADS'] == '2'
        assert all(value is None for value in placement.values())
        assert 'GOMP_CPU_AFFINITY' in os.environ  # Restored in parent.
    finally:
        if process.is_alive():
            process.terminate()
            process.join(5)
        parent.close()


def _report_preexisting_thread(connection):
    started, observe = threading.Event(), threading.Event()
    result = {}
    def old_thread():
        started.set()
        if observe.wait(10):
            result.update(cpus=sorted(os.sched_getaffinity(0)),
                          nice=os.getpriority(os.PRIO_PROCESS,0))
    thread = threading.Thread(target=old_thread)
    thread.start()
    assert started.wait(10)
    effective = budget.apply_worker_budget()
    observe.set()
    thread.join(10)
    connection.send((effective, result))
    connection.close()


def test_existing_pool_threads_are_constrained_not_just_main_thread(monkeypatch):
    expected = sorted(os.sched_getaffinity(0))[:2]
    monkeypatch.setenv(budget.FROZEN_ENV, json.dumps(budget.DEFAULT))
    context = multiprocessing.get_context('spawn')
    parent, child = context.Pipe(duplex=False)
    process = context.Process(target=_report_preexisting_thread, args=(child,))
    process.start()
    child.close()
    try:
        assert parent.poll(10)
        effective, previous_thread = parent.recv()
        process.join(5)
        assert process.exitcode == 0
        assert effective['constrained_existing_threads'] >= 2
        assert effective['effective_cpu_ids'] == previous_thread['cpus'] == expected
        assert previous_thread['nice'] >= 5
    finally:
        if process.is_alive():
            process.terminate()
            process.join(5)
        parent.close()


def test_real_spawn_enforces_limit_and_leaves_parent_untouched(monkeypatch):
    expected_affinity = set(os.sched_getaffinity(0))
    expected_nice = os.getpriority(os.PRIO_PROCESS,0)
    monkeypatch.setenv(budget.FROZEN_ENV, json.dumps(budget.DEFAULT))
    context = multiprocessing.get_context('spawn')
    parent, child = context.Pipe(duplex=False)
    process = context.Process(target=budget.budgeted_worker, args=(_report_worker,child))
    process.start()
    child.close()
    try:
        assert parent.poll(10)
        cpus, nice = parent.recv()
        process.join(5)
        assert process.exitcode == 0
        assert cpus == sorted(expected_affinity)[:2]
        assert nice >= 5
        assert set(os.sched_getaffinity(0)) == expected_affinity
        assert os.getpriority(os.PRIO_PROCESS,0) == expected_nice
    finally:
        if process.is_alive():
            process.terminate()
            process.join(5)
        parent.close()
