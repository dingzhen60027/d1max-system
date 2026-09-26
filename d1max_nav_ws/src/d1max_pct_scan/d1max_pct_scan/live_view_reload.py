"""Bounded preview UI reload contract. No ROS, SDK, shell or arbitrary commands."""
import json
import math
import os
from pathlib import Path
import re
import stat
import tempfile
import time

VERSION = 1
REQUEST_TTL = 5.
UI_NAMES = frozenset(('view', 'rviz'))
REQUEST_FILE = 'ui_reload_request.json'
RESULT_FILE = 'ui_reload_result.json'


def token(value):
    return isinstance(value, str) and re.fullmatch(r'[0-9a-f]{32}', value) is not None


def preview_reload_session(session):
    if (not isinstance(session, dict) or not token(session.get('id'))
            or session.get('mode') != 'LIVE_VISUALIZATION_NO_MOTION'
            or session.get('motion_control_enabled') is not False):
        raise ValueError('UI reload is forbidden for motion-capable or ambiguous sessions')
    if (type(session.get('ui_reload_supported')) is not int
            or session['ui_reload_supported'] != VERSION
            or session.get('preview_freeze_owner') != 'supervisor_child'):
        raise ValueError('This preview was created before UI-only reload support; restart preview once')


def owned_directory(directory, root):
    """Only a direct, real session directory under the fixed supervisor root."""
    path, root = Path(directory).absolute(), Path(root).resolve()
    info = path.lstat()
    if (not stat.S_ISDIR(info.st_mode) or path.resolve().parent != root
            or path != path.resolve() or info.st_uid != os.getuid()
            or info.st_mode & 0o022):
        raise ValueError('UI reload requires an owned, non-writable-by-others session directory')
    return path


def read_owned_json(path, maximum_bytes=65536):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as stream:
        info = os.fstat(stream.fileno())
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                or info.st_mode & 0o022 or info.st_size > maximum_bytes):
            raise ValueError('Invalid owned reload mailbox file')
        raw = stream.read(maximum_bytes + 1)
        if len(raw) > maximum_bytes:
            raise ValueError('Reload mailbox exceeds its size limit')
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError('Reload mailbox must contain an object')
    return value


def write_owned_text(path, value):
    """Atomic fixed-name mailbox; no reusable/symlink-following temporary path."""
    path = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.' + path.name + '.', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(value)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def write_owned_json(path, value):
    write_owned_text(path, json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n')


def verify_runtime_owner(session, runtime, unit, *, owner_prefix, now):
    preview_reload_session(session)
    capability = runtime.get('ui_reload')
    if (type(runtime.get('ui_reload_supported')) is not int
            or runtime['ui_reload_supported'] != VERSION or not isinstance(capability, dict)):
        raise ValueError('Running supervisor does not support UI-only reload; restart preview once')
    stamp = runtime.get('received_at_unix')
    if (runtime.get('session_id') != session['id'] or runtime.get('phase') != 'running'
            or runtime.get('motion_capable') is not False
            or type(stamp) not in (int, float) or not math.isfinite(stamp)
            or not -.1 <= now-stamp <= 2.5
            or unit.get('ActiveState') != 'active'
            or unit.get('Description') != owner_prefix + session['id']
            or type(capability.get('owner_pid')) is not int or capability['owner_pid'] <= 1
            or unit.get('MainPID') != str(capability['owner_pid'])
            or not token(capability.get('invocation_id'))
            or unit.get('InvocationID') != capability['invocation_id']
            or not token(capability.get('owner_nonce'))):
        raise ValueError('Cannot verify fresh ownership of the running preview supervisor')
    return capability


def validate_request(value, *, session_id, owner_nonce, now):
    keys = {'schema', 'action', 'session_id', 'owner_nonce', 'request_id', 'issued_at', 'expires_at'}
    if (not isinstance(value, dict) or set(value) != keys
            or type(value.get('schema')) is not int or value['schema'] != VERSION
            or value.get('action') != 'reload-view' or value.get('session_id') != session_id
            or value.get('owner_nonce') != owner_nonce or not token(value.get('request_id'))):
        raise ValueError('Invalid UI reload identity, nonce or fixed action')
    issued, expires = value['issued_at'], value['expires_at']
    if (any(type(v) not in (int, float) or not math.isfinite(v) for v in (issued, expires, now))
            or not 0 < expires-issued <= REQUEST_TTL or not -.1 <= now-issued
            or now > expires):
        raise ValueError('UI reload request expired or has an invalid deadline')
    return value['request_id']


def verified_result(value, request, *, owner_pid, invocation_id, now):
    """Ignore another request's ACK; reject malformed or stale matching ACKs."""
    if value.get('session_id') != request['session_id'] or value.get('request_id') != request['request_id']:
        return None
    completed = value.get('completed_at')
    if (type(value.get('schema')) is not int or value['schema'] != VERSION
            or value.get('owner_nonce') != request['owner_nonce']
            or type(value.get('owner_pid')) is not int or value['owner_pid'] != owner_pid
            or value.get('invocation_id') != invocation_id
            or value.get('status') not in ('reloaded', 'failed')
            or type(completed) not in (int, float) or not math.isfinite(completed)
            or completed < request['issued_at']-.1 or not -.1 <= now-completed <= 12.):
        raise ValueError('Invalid source identity or timestamp in UI reload acknowledgement')
    if value['status'] == 'reloaded':
        children = value.get('children')
        if (not isinstance(children, list) or len(children) != 2
                or any(not isinstance(child, dict) for child in children)
                or {child.get('name') for child in children} != UI_NAMES
                or any(type(child.get('pid')) is not int or child['pid'] <= 1 for child in children)
                or len({child['pid'] for child in children}) != 2
                or value.get('processes_running') is not True):
            raise ValueError('UI reload acknowledgement lacks replacement process evidence')
    return value


class RvizClosedDuringReload(RuntimeError):
    pass


def replace_ui_children(children, *, stop_children, spawn, configure, stopping,
                        pause=time.sleep, monotonic=time.monotonic, settle_s=1.,
                        terminated_ui_pids=None):
    """Replace exactly view+rviz, with old process-group death before any spawn.

    `spawn` is an owner-supplied fixed command dispatcher, never request input.
    Backend process objects and their order are retained verbatim. A failure
    leaves them owned/running, and cleans up partial replacement UI processes.
    """
    current = [(name, child) for name, child in children if name in UI_NAMES]
    backend = [(name, child) for name, child in children if name not in UI_NAMES]
    if any(sum(name == target for name, _ in current) > 1 for target in UI_NAMES):
        raise RuntimeError('Duplicate UI ownership; refusing to launch more processes')
    if not any(name == 'preview_freeze' for name, _ in backend):
        raise RuntimeError('No independent preview heartbeat; UI reload would revoke planning')

    def check_backend():
        if stopping():
            raise RuntimeError('Supervisor is stopping; UI reload cancelled')
        if any(child.poll() is not None for _, child in backend):
            raise RuntimeError('A backend exited; UI reload cannot report success')

    check_backend()
    configure()  # Fail before stopping anything if the new fixed RViz config is invalid.
    if terminated_ui_pids is not None:
        terminated_ui_pids.update(child.pid for _, child in current)
    cleanup = stop_children(current, interrupt_s=2., terminate_s=.5, kill_s=.5)
    if not cleanup['all_direct_children_exited'] or not cleanup['all_owned_groups_exited']:
        raise RuntimeError('Old UI process groups remain alive; replacement not started')
    children[:] = backend
    replacement = []
    try:
        check_backend()
        for name in ('view', 'rviz'):
            child = spawn(name)
            replacement.append((name, child))
            children.append((name, child))
        deadline = monotonic() + settle_s
        while True:
            check_backend()
            if any(name == 'rviz' and child.poll() == 0 for name, child in replacement):
                raise RvizClosedDuringReload('RViz closed during UI reload; preview is stopping')
            if any(child.poll() is not None for _, child in replacement):
                raise RuntimeError('A replacement UI process exited during startup')
            if monotonic() >= deadline:
                break
            pause(min(.05, max(0., deadline-monotonic())))
        return {'children': [{'name': name, 'pid': child.pid} for name, child in replacement],
                'preserved_children': [{'name': name, 'pid': child.pid} for name, child in backend],
                'processes_running': True, 'render_verified': False}
    except BaseException:
        if replacement:
            if terminated_ui_pids is not None:
                terminated_ui_pids.update(child.pid for _, child in replacement)
            cleanup = stop_children(replacement, interrupt_s=1., terminate_s=.5, kill_s=.5)
            if cleanup['all_direct_children_exited'] and cleanup['all_owned_groups_exited']:
                children[:] = backend
        raise
