"""Scoped compute budget for owned global-planning subprocesses only.

No main-process affinity/priority changes; no ROS middleware changes. Native
numeric limits are prepared before spawn so BLAS cannot create an unbounded
pool at import. Affinity is a ceiling, not exclusive reservation of CPU cores.
"""
import json
import multiprocessing
import os
from pathlib import Path
import threading

DEFAULT = dict(max_cpu_cores=2, cpu_ids=[], omp_threads=2, blas_threads=1, nice=5)
FROZEN_ENV = 'D1MAX_PCT_COMPUTE_BUDGET'
THREAD_ENV = ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
              'NUMEXPR_NUM_THREADS', 'OMP_DYNAMIC', 'OMP_MAX_ACTIVE_LEVELS',
              'OMP_PROC_BIND', 'GOTO_NUM_THREADS', 'BLIS_NUM_THREADS',
              'VECLIB_MAXIMUM_THREADS')
PLACEMENT_ENV = ('GOMP_CPU_AFFINITY', 'KMP_AFFINITY', 'KMP_HW_SUBSET', 'OMP_PLACES')
SPAWN_ENV = ('LD_LIBRARY_PATH', 'D1MAX_PCT_EXPECTED_GTSAM', FROZEN_ENV,
             *THREAD_ENV, *PLACEMENT_ENV)
_SPAWN_LOCK = threading.Lock()


def validate_budget(value):
    if not isinstance(value, dict) or set(value) != set(DEFAULT):
        raise ValueError('Global planner compute budget has missing/unknown fields')
    result = dict(value)
    for field in ('max_cpu_cores', 'omp_threads', 'blas_threads'):
        if type(value[field]) is not int or not 1 <= value[field] <= 32:
            raise ValueError(field + ' must be an integer in [1,32]')
    if type(value['nice']) is not int or not 0 <= value['nice'] <= 19:
        raise ValueError('nice must be in [0,19]; real-time priority is not permitted')
    ids = value['cpu_ids']
    if (not isinstance(ids, list) or any(type(x) is not int or x < 0 for x in ids)
            or len(set(ids)) != len(ids) or len(ids) > value['max_cpu_cores']):
        raise ValueError('cpu_ids must be distinct nonnegative IDs within the core budget')
    if max(value['omp_threads'], value['blas_threads']) > value['max_cpu_cores']:
        raise ValueError('Numeric threads must not exceed the worker CPU budget')
    result['cpu_ids'] = list(ids)
    return result


def load_budget(environment=None):
    env = os.environ if environment is None else environment
    if env.get(FROZEN_ENV):
        return validate_budget(json.loads(env[FROZEN_ENV]))
    path = env.get('D1MAX_PCT_COMPUTE_CONFIG')
    if path:
        path = Path(path)
        if not path.is_absolute() or not path.is_file():
            raise ValueError('D1MAX_PCT_COMPUTE_CONFIG must be an existing absolute YAML path')
    else:
        path = Path(__file__).resolve().parents[1] / 'config/compute_budget.yaml'
        if not path.is_file():
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory('d1max_pct_planner')) / 'config/compute_budget.yaml'
    import yaml
    return validate_budget(yaml.safe_load(path.read_text()))


def prepare_environment(environment):
    env = dict(environment)
    budget = load_budget(env)
    env[FROZEN_ENV] = json.dumps(budget, sort_keys=True)
    env.update(OMP_NUM_THREADS=str(budget['omp_threads']),
               OPENBLAS_NUM_THREADS=str(budget['blas_threads']),
               MKL_NUM_THREADS=str(budget['blas_threads']),
               NUMEXPR_NUM_THREADS=str(budget['blas_threads']),
               GOTO_NUM_THREADS=str(budget['blas_threads']),
               BLIS_NUM_THREADS=str(budget['blas_threads']),
               VECLIB_MAXIMUM_THREADS=str(budget['blas_threads']),
               OMP_DYNAMIC='FALSE', OMP_MAX_ACTIVE_LEVELS='1', OMP_PROC_BIND='FALSE')
    # Host OpenMP placement must not re-pin newly created threads outside the
    # worker's inherited cpuset after the budget has been applied.
    for name in PLACEMENT_ENV:
        env.pop(name, None)
    return env


def spawn_with_environment(process, environment):
    """Serialize this package's spawn environment updates, restoring on failure.

    multiprocessing has no per-child env argument. Other code starting children
    concurrently should use this helper too; this lock is not a process-wide
    sandbox for arbitrary third-party subprocess calls.
    """
    with _SPAWN_LOCK:
        previous = {key: os.environ.get(key) for key in SPAWN_ENV}
        try:
            for key in SPAWN_ENV:
                value = environment.get(key)
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value
            process.start()
        finally:
            for key, value in previous.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value


def _thread_ids():
    return {int(path.name) for path in Path('/proc/self/task').iterdir()}


def apply_worker_budget():
    if multiprocessing.current_process().name == 'MainProcess':
        raise RuntimeError('Refusing to change CPU affinity/priority of the main process')
    budget = load_budget()
    allowed = set(os.sched_getaffinity(0))
    cpus = budget['cpu_ids'] or sorted(allowed)[:budget['max_cpu_cores']]
    if not cpus or not set(cpus).issubset(allowed):
        raise RuntimeError('Configured planning CPUs are outside the inherited cpuset')
    # Linux affinity and nice are per-thread, not per-process. Spawn imports can
    # have initialized numeric pools before this target is invoked. Restrict all
    # existing threads, then settle any threads they created concurrently.
    configured = set()
    for _ in range(8):
        for tid in _thread_ids()-configured:
            try:
                os.sched_setaffinity(tid, cpus)
                priority = max(os.getpriority(os.PRIO_PROCESS, tid), budget['nice'])
                os.setpriority(os.PRIO_PROCESS, tid, priority)
                configured.add(tid)
            except ProcessLookupError:
                pass  # Thread exited; it cannot consume resources anymore.
        current = _thread_ids()
        if current.issubset(configured):
            break
    else:
        raise RuntimeError('Worker thread set did not settle while applying compute budget')
    for tid in current:
        try:
            if (not set(os.sched_getaffinity(tid)).issubset(cpus)
                    or os.getpriority(os.PRIO_PROCESS, tid) < budget['nice']):
                raise RuntimeError('Worker thread escaped the configured compute budget')
        except ProcessLookupError:
            pass
    return {**budget, 'effective_cpu_ids': sorted(os.sched_getaffinity(0)),
            'effective_nice': os.getpriority(os.PRIO_PROCESS, 0),
            'constrained_existing_threads': len(current),
            'exclusive_cores': False}


def budgeted_worker(target, *args):
    effective = apply_worker_budget()
    print(json.dumps({'global_planner_compute_budget': effective}), flush=True)
    return target(*args)
