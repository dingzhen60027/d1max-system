#!/usr/bin/env python3
"""Read-only, no-ROS completion/transform audit for one ended extrinsic trial.

The only write is --output, opened exclusively. No process is signalled and no
network connection is made. Run between trials: a different active domain-219
trial or another listener on 7447 deliberately fails the isolation check.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
from datetime import datetime, timezone

import yaml


LIBRARY = Path('/home/dndx/d1max_nav_ws/install/faster_lio/lib/libfaster_lio_lib.so')
EXPECTED_SHA256 = '8f7a66b4a545a24fce4ba36cbc9764e16e8a2b9f5361833a376738831111f069'
DEFAULT_REAR = dict(rotation=[1., 0., 0., 0., -1., 0., 0., 0., -1.],
                    translation=[0., 0., -.7323],
                    parent_frame='rslidar_head', child_frame='rslidar_tail')
ROS_EXECUTABLES = {'run_mapping_online', 'dual_lidar_adapter', 'map_capture_node',
                   'alaserPGO', 'static_transform_publisher', 'rmw_zenohd', 'rviz2'}


def process(pid):
    proc = Path('/proc') / str(pid)
    try:
        stat = (proc / 'stat').read_text()
        state = stat[stat.rfind(')') + 2:].split()[0]
        command = [v.decode(errors='replace') for v in
                   (proc / 'cmdline').read_bytes().split(b'\0') if v]
        return dict(pid=pid, state=state, command=command,
                    running=state not in ('Z', 'X'))
    except FileNotFoundError:
        return None


def same_command(expected, actual):
    """Allow a shebang interpreter prefix without matching arbitrary substrings."""
    if not expected or not actual:
        return False
    name = Path(expected[0]).name
    for i in range(min(3, len(actual))):
        if Path(actual[i]).name == name and actual[i + 1:] == expected[1:]:
            return True
    return False


def recorded_processes(run):
    records = json.loads((run / 'processes.json').read_text())
    evidence, passed = [], True
    for name, record in records.items():
        try:
            live = process(int(record['pid']))
            match = bool(live and live['running'] and
                         same_command(record['command'], live['command']))
            item = dict(name=name, recorded_pid=record['pid'],
                        matching_running_process=match, observed=live)
            if live and live['running'] and not match:
                item['note'] = 'PID exists but command differs: reused or changed; not treated as this child.'
            if match:
                item['note'] = 'Matching live command; no stored start time, so identity cannot be stronger than this.'
            passed = passed and not match
            evidence.append(item)
        except (PermissionError, OSError, KeyError, ValueError) as error:
            evidence.append(dict(name=name, error=str(error)))
            passed = False
    return passed, evidence


def is_trial_command(command):
    if not command:
        return False
    if Path(command[0]).name in ROS_EXECUTABLES:
        return True
    for i in range(min(3, len(command))):
        name = Path(command[i]).name
        if name in ('central_imu_adapter.py', 'cloud_subset.py'):
            return True
        if name == 'run.py' and 'central_imu_fasterlio_20260918' in command[i]:
            return True
        if name == 'ros2' and (command[i + 1:i + 3] == ['bag', 'play'] or
                              command[i + 1:i + 2] == ['launch']):
            return True
    return False


def domain_processes():
    found, errors = [], []
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit() or int(directory.name) == os.getpid():
            continue
        try:
            live = process(int(directory.name))
            if not live or not live['running'] or not is_trial_command(live['command']):
                continue
            # Never emit an environment. Only test this one non-secret setting.
            environment = (directory / 'environ').read_bytes().split(b'\0')
            if b'ROS_DOMAIN_ID=219' in environment:
                found.append(live)
        except FileNotFoundError:
            continue
        except (PermissionError, OSError) as error:
            errors.append(dict(pid=int(directory.name), error=str(error)))
    return not found and not errors, dict(processes=found, inspection_errors=errors)


def port_listeners():
    listeners = []
    for name in ('tcp', 'tcp6'):
        path = Path('/proc/net') / name
        if not path.exists() and name == 'tcp6':
            continue
        for line in path.read_text().splitlines()[1:]:
            fields = line.split()
            address, port = fields[1].rsplit(':', 1)
            if fields[3] == '0A' and int(port, 16) == 7447:
                listeners.append(dict(protocol=name, address_hex=address,
                                      port=7447, inode=fields[9]))
    return not listeners, listeners


def quaternion(flat):
    r = [flat[i:i + 3] for i in (0, 3, 6)]
    trace = sum(r[i][i] for i in range(3))
    if trace > 0:
        s = 2 * math.sqrt(trace + 1)
        return [(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s,
                (r[1][0] - r[0][1]) / s, s / 4]
    i = max(range(3), key=lambda k: r[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + r[i][i] - r[j][j] - r[k][k])
    q = [0., 0., 0., (r[k][j] - r[j][k]) / s]
    q[i], q[j], q[k] = s / 4, (r[j][i] + r[i][j]) / s, (r[k][i] + r[i][k]) / s
    return q


def rear_transform(run):
    calibration = yaml.safe_load((run / 'config/calibration.yaml').read_text())
    rear = calibration.get('lidar_extrinsics', {}).get('rear_to_front', DEFAULT_REAR)
    rotation, translation = rear['rotation'], rear['translation']
    if len(rotation) != 9 or len(translation) != 3 or not all(
            math.isfinite(float(v)) for v in rotation + translation):
        raise ValueError('Invalid expected rear transform dimensions or numbers')
    expected_q = quaternion(rotation)
    lines = (run / 'frontend.log').read_text(errors='replace').splitlines()
    prefixes = []
    for line in lines:
        match = re.match(r'^\[(static_transform_publisher-\d+)\].*'
                         r'\[central_experiment_lidar_extrinsic\]:.*publishing transform', line)
        if match:
            prefixes.append(match.group(1))
    if len(prefixes) != 1:
        raise ValueError(f'Expected exactly one rear static-TF startup, got {prefixes}')
    prefix = '[' + prefixes[0] + '] '
    details = [line[len(prefix):] for line in lines if line.startswith(prefix)]
    values = {}
    for key, count in [('translation', 3), ('rotation', 4)]:
        matches = [line for line in details if line.startswith(key + ':')]
        if len(matches) != 1:
            raise ValueError(f'Expected one {key} line for {prefixes[0]}')
        values[key] = [float(v) for v in re.findall(r"'([^']+)'", matches[0])]
        if len(values[key]) != count or not all(math.isfinite(v) for v in values[key]):
            raise ValueError(f'Invalid logged {key}')
    frames = [re.fullmatch(r"from '([^']+)' to '([^']+)'", line) for line in details]
    frames = [match.groups() for match in frames if match]
    if len(frames) != 1:
        raise ValueError('Expected one frame-direction line for rear static TF')
    t_error = max(abs(a - b) for a, b in zip(translation, values['translation']))
    q_error = min(max(abs(a - sign * b) for a, b in zip(expected_q, values['rotation']))
                  for sign in (1, -1))
    expected_frames = (rear.get('parent_frame'), rear.get('child_frame'))
    passed = (expected_frames == ('rslidar_head', 'rslidar_tail') and
              frames[0] == expected_frames and t_error <= 0.6e-6 and q_error <= 0.6e-6)
    return passed, dict(prefix=prefixes[0], expected_translation=translation,
                        expected_quaternion_xyzw=expected_q, logged=values,
                        logged_frames=frames[0], translation_max_error=t_error,
                        quaternion_sign_invariant_max_error=q_error, tolerance=0.6e-6,
                        boundary='Proves startup publication, not independent per-point consumption.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    run = args.run.resolve(strict=True)
    if not run.is_dir():
        parser.error('--run must be a directory')
    # Reserve before checks, refuse overwrite including any existing symlink.
    with args.output.open('x', encoding='utf-8') as stream:
        checks, evidence = {}, {}
        def check(name, function):
            try:
                checks[name], evidence[name] = function()
            except Exception as error:
                checks[name], evidence[name] = False, dict(error=str(error))
        check('run_marked_complete', lambda: (
            json.loads((run / 'result.json').read_text()).get('complete') is True,
            dict(source=str(run / 'result.json'))))
        check('recorded_children_stopped', lambda: recorded_processes(run))
        check('domain219_clear', domain_processes)
        check('port7447_clear', port_listeners)
        def library_check():
            hasher = hashlib.sha256()
            with LIBRARY.open('rb') as library:
                for block in iter(lambda: library.read(1024 * 1024), b''):
                    hasher.update(block)
            digest = hasher.hexdigest()
            return digest == EXPECTED_SHA256, dict(path=str(LIBRARY), actual_sha256=digest,
                                                   expected_sha256=EXPECTED_SHA256)
        check('default_library_unchanged', library_check)
        check('rear_tf_matches_calibration', lambda: rear_transform(run))
        report = dict(schema_version=1, run=str(run),
                      checked_at_utc=datetime.now(timezone.utc).isoformat(),
                      passed=all(checks.values()), checks=checks, evidence=evidence,
                      read_only=True, ros_initialized=False, signals_sent=False)
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(passed=report['passed'], checks=checks, output=str(args.output))))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
