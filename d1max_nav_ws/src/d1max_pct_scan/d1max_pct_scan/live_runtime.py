"""Bounded, ROS-independent I/O and process cleanup for an owned live session."""
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import time


def file_sha256(path, chunk_bytes=1024 * 1024):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(chunk_bytes), b''):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n')
    os.replace(temporary, path)


def child_failure_reason(directory, name, code):
    """Expose a bounded, allowlisted cause, never an arbitrary log/credential.

    ros2 launch may exit zero after a critical child fails; its exit code is
    insufficient to explain why the owned session was torn down.
    """
    fallback = f'{name} exited with code {code}; stopping owned session'
    if not re.fullmatch(r'[a-z_]{1,32}', name):
        return fallback
    try:
        with (Path(directory)/(name+'.log')).open('rb') as stream:
            stream.seek(0, os.SEEK_END)
            stream.seek(max(0, stream.tell()-65536))
            tail = stream.read(65536).decode('utf-8', errors='replace')
    except OSError:
        return fallback
    missing = re.search(r"ModuleNotFoundError: No module named '([A-Za-z_][A-Za-z0-9_.]{0,159})'", tail)
    if missing:
        return f'{name} 启动失败：缺少 Python 模块 {missing.group(1)}；请重新构建对应软件包'
    return fallback


def group_alive(process):
    # Only use with children created by Popen(start_new_session=True).
    try:
        os.killpg(process.pid, 0)
        return True
    except ProcessLookupError:
        return False


def stop_owned_children(children, *, interrupt_s=6., terminate_s=1., kill_s=1.):
    """Stop ONLY the separate groups created by this supervisor; never pgrep/killall.

    systemd KillMode=control-group remains the final guard for descendants which
    deliberately detach. Return concrete exit evidence, not optimistic success.
    """
    started = time.monotonic()
    phases = []
    for signum, budget in ((signal.SIGINT, interrupt_s), (signal.SIGTERM, terminate_s),
                           (signal.SIGKILL, kill_s)):
        pending = [(name, child) for name, child in children if group_alive(child)]
        if not pending:
            break
        phases.append(signal.Signals(signum).name)
        for _, child in reversed(pending):
            try:
                os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + budget
        while True:
            for _, child in children:
                child.poll()  # reap direct children even when their groups outlive them
            if not any(group_alive(child) for _, child in children):
                break
            if time.monotonic() >= deadline:
                break
            time.sleep(min(.05, max(0., deadline-time.monotonic())))
    results = [{'name': name, 'pid': child.pid, 'returncode': child.poll(),
                'group_alive': group_alive(child)} for name, child in children]
    return {'elapsed_s': time.monotonic()-started, 'signals': phases,
            'children': results,
            'all_direct_children_exited': all(item['returncode'] is not None for item in results),
            'all_owned_groups_exited': not any(item['group_alive'] for item in results)}
