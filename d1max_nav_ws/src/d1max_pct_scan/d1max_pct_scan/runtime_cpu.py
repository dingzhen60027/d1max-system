"""Best-effort CPU placement for the two latency-critical perception workers.

This is an internal exec trampoline, not another navigation entry or a health
gate. Only Intel's explicit hybrid-core topology is used, never a guessed core
number or a frequency threshold. Homogeneous/restricted hosts keep their
existing placement. No scheduler priority, power policy or parent is changed.
"""
import json
import os
from pathlib import Path
import sys


CRITICAL_COMPONENTS = frozenset(('scan', 'perception'))


def cpu_list(text):
    result = set()
    for part in text.strip().split(','):
        edges = part.split('-')
        if len(edges) not in (1, 2) or any(not edge.isdecimal() for edge in edges):
            raise ValueError('invalid_cpu_list')
        begin, end = int(edges[0]), int(edges[-1])
        if not 0 <= begin <= end < 4096:
            raise ValueError('invalid_cpu_range')
        result.update(range(begin, end + 1))
    if not result:
        raise ValueError('empty_cpu_list')
    return result


def performance_cpus(allowed, *, sysfs=Path('/sys')):
    """Return an allowed hybrid performance class, or no placement change."""
    allowed = set(allowed)
    try:
        topology = Path(sysfs)/'bus/event_source/devices'
        fast = cpu_list((topology/'cpu_core/cpus').read_text())
        efficient = cpu_list((topology/'cpu_atom/cpus').read_text())
        online = cpu_list((Path(sysfs)/'devices/system/cpu/online').read_text())
    except (OSError, ValueError):
        return None
    # Both classes must be independently identified. Ordinary homogeneous
    # "cpu" PMUs and partial/conflicting topology are not a reason to pin.
    if fast & efficient or not (efficient & online):
        return None
    selected = fast & online & allowed
    if not selected or selected == allowed:
        return None
    # Do not squeeze two projection workers and SCAN's independent executors
    # into one physical core when a container/cpuset grants only a narrow
    # mixture. SMT siblings are not two independent performance cores.
    try:
        cores = set()
        for number in selected:
            path = Path(sysfs)/'devices/system/cpu'/('cpu'+str(number))/'topology'
            cores.add((int((path/'physical_package_id').read_text()),
                       int((path/'core_id').read_text())))
    except (OSError, ValueError):
        return None
    return selected if len(cores) >= 2 else None


def apply_placement(*, sysfs=Path('/sys')):
    report = dict(policy='hybrid_perception', applied=False, reason='inherited')
    try:
        allowed = os.sched_getaffinity(0)
        report['inherited_cpus'] = sorted(allowed)
        selected = performance_cpus(allowed, sysfs=sysfs)
        if selected is not None:
            os.sched_setaffinity(0, selected)
            report.update(applied=True, reason='explicit_hybrid_performance_class')
        report['effective_cpus'] = sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError, ValueError) as error:
        # Optional placement must not become yet another startup gate.
        report.update(reason='placement_unavailable', error=type(error).__name__)
    return report


def component_command(name, command):
    if name not in CRITICAL_COMPONENTS:
        return command
    return [sys.executable, '-m', 'd1max_pct_scan.runtime_cpu', '--', *command]


def main():
    args = sys.argv[1:]
    if len(args) < 2 or args[0] != '--':
        raise SystemExit('internal exec requires -- command [arguments]')
    report = apply_placement()
    try:
        print('NavigationCPU '+json.dumps(report, sort_keys=True), file=sys.stderr, flush=True)
    except OSError:
        pass
    # Same process/session group, argv, inherited FDs and signal behaviour.
    # Set placement BEFORE the child creates ROS/NumPy/solver worker threads.
    os.execvpe(args[1], args[1:], os.environ)


if __name__ == '__main__':
    main()
