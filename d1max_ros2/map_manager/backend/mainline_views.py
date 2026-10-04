"""Own one RViz window of an existing mainline session, never its task.

Layouts are presentation requests to that same window. The core and SDK
remain outside this one-way dependent systemd group's lifecycle.
"""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import subprocess
import tempfile
import threading
import time
import uuid

from .localization import Systemd


CORE_UNIT = 'd1max-single-floor-navigation.service'
CORE_OWNER = 'D1MAX-SINGLE-FLOOR:'
VIEW_OWNER = 'D1MAX-NAVIGATION-VIEW:'
VIEW_UNIT = 'd1max-navigation-view.service'
VIEW_UNITS = {layout: VIEW_UNIT for layout in ('global', 'local')}
LEGACY_VIEW_UNITS = {layout: 'd1max-navigation-view-' + layout + '.service'
                     for layout in ('global', 'local')}
VIEW_CONTRACT = 'single_window_layout_v1'
LAYOUT_REQUEST_FILE = 'navigation-view-layout-request.json'
LAYOUT_STATE_FILE = 'navigation-view-layout-state.json'
LAYOUT_LIMIT = 4096
ACTIVE = {'active', 'activating', 'deactivating', 'reloading'}
DESKTOP_KEYS = {'DISPLAY', 'XAUTHORITY', 'WAYLAND_DISPLAY', 'XDG_RUNTIME_DIR',
                'DBUS_SESSION_BUS_ADDRESS', 'LANG'}
SHOW_PROPERTIES = ('ActiveState,Description,MainPID,InvocationID,ControlGroup,'
                   'ExecMainStatus,Result')


class OwnedNavigationViews:
    def __init__(self, nav_root, system=None, runner=None):
        self.nav_root = Path(nav_root).resolve()
        self.system = system or Systemd()
        self.runner = runner or subprocess.run
        self.lock = threading.Lock()

    @contextmanager
    def _transaction(self):
        if not self.lock.acquire(False):
            raise ValueError('view_lifecycle_busy')
        try:
            path = self.nav_root / 'log/.navigation-view-switch.lock'
            path.parent.mkdir(parents=True, exist_ok=True)
            fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            try:
                info = os.fstat(fd)
                if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
                    raise ValueError('view_lifecycle_lock_not_private')
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    raise ValueError('view_lifecycle_busy') from None
                yield
            finally:
                os.close(fd)
        finally:
            self.lock.release()

    def _binding(self, state):
        if not isinstance(state, dict):
            raise ValueError('view_session_binding_invalid')
        identifier = state.get('id')
        if not isinstance(identifier, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,128}', identifier):
            raise ValueError('view_session_binding_invalid')
        try:
            release = Path(state['release_root'])
            directory = Path(state['directory'])
            if not release.is_absolute() or not directory.is_absolute():
                raise ValueError('view_session_binding_invalid')
            release = release.resolve(strict=True)
            directory = directory.resolve(strict=True)
            if (not release.is_dir() or not directory.is_dir()
                    or not release.is_relative_to(self.nav_root / 'experiments')):
                raise ValueError('view_session_binding_invalid')
        except (KeyError, TypeError, OSError):
            raise ValueError('view_session_binding_invalid') from None
        expected_session = state.get('session_sha256')
        if expected_session and hashlib.sha256((directory / 'session.json').read_bytes()).hexdigest() != expected_session:
            raise ValueError('view_session_binding_changed')
        # Use this session's admitted dispatcher, never a new Web activation.
        # Older saved sessions have no path field and retain their old entry.
        entry = Path(state.get('entrypoint') or self.nav_root / 'tools/single_floor_entry.sh')
        allowed = {self.nav_root / 'tools/navigation_entry.sh',
                   self.nav_root / 'tools/single_floor_entry.sh'}
        if (entry not in allowed or entry.is_symlink() or not entry.is_file()
                or not os.access(entry, os.X_OK)):
            raise ValueError('view_entrypoint_unavailable')
        expected = state.get('entrypoint_sha256')
        if (entry.name == 'navigation_entry.sh' and not expected
                or expected and hashlib.sha256(entry.read_bytes()).hexdigest() != expected):
            raise ValueError('view_entrypoint_changed')
        if entry.name == 'navigation_entry.sh':
            compatibility_entry = self.nav_root / 'tools/single_floor_entry.sh'
            expected_dependency = state.get('compatibility_entrypoint_sha256')
            if (not expected_dependency or compatibility_entry.is_symlink()
                    or not compatibility_entry.is_file()
                    or hashlib.sha256(compatibility_entry.read_bytes()).hexdigest() != expected_dependency):
                raise ValueError('view_entrypoint_dependency_changed')
        return identifier, directory, release, entry

    def _unit(self, name):
        result = self.runner(['systemctl', '--user', 'show', name,
                              '--property=' + SHOW_PROPERTIES], check=True,
                             capture_output=True, text=True, timeout=3)
        value = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
        if value.get('ActiveState') not in ACTIVE | {'inactive', 'failed'}:
            raise ValueError('view_unit_state_unverified')
        try:
            if int(value.get('MainPID') or 0) < 0:
                raise ValueError('view_unit_identity_unverified')
        except (TypeError, ValueError):
            raise ValueError('view_unit_identity_unverified') from None
        return value

    def _busy(self, unit):
        return unit['ActiveState'] in ACTIVE or self.system.populated(unit)

    def _require_core(self, state, identifier):
        core = self._unit(CORE_UNIT)
        if core['ActiveState'] != 'active' or not int(core.get('MainPID') or 0):
            raise ValueError('navigation_core_not_running')
        if core.get('Description') != CORE_OWNER + identifier:
            raise ValueError('navigation_core_owner_mismatch')
        if not core.get('InvocationID'):
            raise ValueError('navigation_core_identity_unverified')
        if ((state.get('pid') and int(core['MainPID']) != state['pid'])
                or (state.get('invocation_id') and core['InvocationID'] != state['invocation_id'])):
            raise ValueError('navigation_core_owner_mismatch')
        return core

    @staticmethod
    def _description(identifier, viewer_id):
        return VIEW_OWNER + identifier + ':single:' + viewer_id

    def _record(self, state, key, unit, identifier, directory, release, *, allow_stopping=False):
        name = VIEW_UNIT if key == 'single' else LEGACY_VIEW_UNITS[key]
        description = unit.get('Description', '')
        viewer_id = ''
        if key == 'single':
            prefix = VIEW_OWNER + identifier + ':single:'
            if not description.startswith(prefix) or not re.fullmatch(r'[0-9a-f]{32}', description[len(prefix):]):
                raise ValueError('view_owner_mismatch:single')
            viewer_id = description[len(prefix):]
        elif description != VIEW_OWNER + identifier + ':' + key:
            raise ValueError('view_owner_mismatch:' + key)
        pid = int(unit.get('MainPID') or 0)
        invocation = unit.get('InvocationID')
        views = state.get('views', {})
        if not isinstance(views, dict):
            raise ValueError('view_records_invalid')
        previous = views.get(key)
        active_identity = unit['ActiveState'] == 'active' and pid > 0
        # BindsTo may already be closing a view after the core exits.  A saved
        # exact invocation can still own its residual group even after its
        # leader exits; without that record we cannot adopt this partial state.
        if (not invocation or (not active_identity and
                (not allow_stopping or not isinstance(previous, dict)))):
            raise ValueError('view_identity_unverified:' + key)
        if previous is not None:
            if (not isinstance(previous, dict) or previous.get('unit') != name
                    or previous.get('core_id') != identifier
                    or (key != 'single' and previous.get('layout') != key)
                    or (key == 'single' and previous.get('viewer_id') != viewer_id)
                    or previous.get('description') != description
                    or previous.get('directory') != str(directory)
                    or previous.get('release_root') != str(release)
                    or (previous.get('session_sha256') is not None
                        and previous.get('session_sha256') != state.get('session_sha256'))
                    or (pid > 0 and previous.get('pid') != pid)
                    or previous.get('invocation_id') != invocation):
                raise ValueError('view_owner_mismatch:' + key)
        record = dict(unit=name, layout=previous.get('layout') if isinstance(previous, dict) else None,
                    core_id=identifier,
                    directory=str(directory), release_root=str(release),
                    description=description, pid=pid, invocation_id=invocation,
                    phase='running' if active_identity else 'stopping')
        if key == 'single':
            record['viewer_id'] = viewer_id
            if state.get('session_sha256'):
                record['session_sha256'] = state['session_sha256']
            if isinstance(previous, dict) and previous.get('rviz_pid'):
                record['rviz_pid'] = previous['rviz_pid']
        else:
            record['layout'] = key
        return record

    @staticmethod
    def _private_json(path):
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK)
        except FileNotFoundError:
            return None
        with os.fdopen(fd, 'r') as stream:
            info = os.fstat(stream.fileno())
            if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                    or info.st_mode & 0o077 or info.st_size > LAYOUT_LIMIT):
                raise ValueError('view_layout_file_not_private')
            value = json.loads(stream.read(LAYOUT_LIMIT + 1))
        if not isinstance(value, dict):
            raise ValueError('view_layout_file_invalid')
        return value

    @classmethod
    def _write_request(cls, directory, request):
        path = directory / LAYOUT_REQUEST_FILE
        # Do not replace an unsafe file that another process placed here.
        cls._private_json(path)
        data = json.dumps(request, separators=(',', ':')).encode()
        if len(data) > LAYOUT_LIMIT:
            raise ValueError('view_layout_request_too_large')
        fd, temporary = tempfile.mkstemp(prefix='.navigation-view-layout-', dir=directory)
        try:
            with os.fdopen(fd, 'wb') as stream:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def _ack(self, directory, identifier, viewer_id, request=None):
        value = self._private_json(directory / LAYOUT_STATE_FILE)
        if not value:
            return None
        applied = value.get('applied_at_unix')
        stamp = value.get('stamp')
        if (type(value.get('schema')) is not int or value['schema'] != 1
                or value.get('session_id') != identifier
                or value.get('viewer_id') != viewer_id or value.get('phase') != 'applied'
                or value.get('layout') not in VIEW_UNITS
                or not isinstance(value.get('request_id'), str)
                or not re.fullmatch(r'[0-9a-f]{32}', value['request_id'])
                or type(value.get('rviz_pid')) is not int or value['rviz_pid'] <= 0
                or type(applied) not in (int, float) or not math.isfinite(applied)
                or type(stamp) not in (int, float) or not math.isfinite(stamp)
                or applied < stamp - .1
                or applied > time.time() + .1):
            return None
        if request and any(value.get(key) != item for key, item in request.items()):
            return None
        return value

    def _check_identity(self, state, identifier, core, record):
        after_core = self._require_core(state, identifier)
        if any(after_core.get(key) != core.get(key) for key in ('Description', 'MainPID', 'InvocationID')):
            raise ValueError('navigation_core_owner_mismatch')
        after = self._unit(VIEW_UNIT)
        if (after['ActiveState'] != 'active'
                or any(after.get(key) != record[item] for key, item in
                       (('Description', 'description'), ('InvocationID', 'invocation_id')))
                or int(after.get('MainPID') or 0) != record['pid']):
            raise ValueError('view_owner_mismatch:single')

    def _layout(self, state, layout, identifier, directory, core, record):
        self._check_identity(state, identifier, core, record)
        ack = self._ack(directory, identifier, record['viewer_id'])
        if ack and record.get('rviz_pid') and ack['rviz_pid'] != record['rviz_pid']:
            raise ValueError('view_owner_mismatch:single')
        if ack:
            record = {**record, 'layout': ack['layout'], 'rviz_pid': ack['rviz_pid']}
        # A native button may have changed the window since the last ACK.
        # Every Web click needs its own ACK, including repeated layout clicks.
        request = dict(schema=1, session_id=identifier, viewer_id=record['viewer_id'],
                       request_id=uuid.uuid4().hex, layout=layout, stamp=time.time())
        self._write_request(directory, request)
        deadline = time.monotonic() + 2.
        while True:
            ack = self._ack(directory, identifier, record['viewer_id'], request)
            if ack or time.monotonic() >= deadline:
                break
            time.sleep(.05)
        self._check_identity(state, identifier, core, record)
        if ack:
            if record.get('rviz_pid') and ack['rviz_pid'] != record['rviz_pid']:
                raise ValueError('view_owner_mismatch:single')
            return {**record, 'layout': layout, 'layout_state': 'applied',
                    'request_id': request['request_id'], 'rviz_pid': ack['rviz_pid']}
        return {**record, 'layout': record['layout'], 'requested_layout': layout,
                'layout_state': 'pending', 'request_id': request['request_id']}

    @staticmethod
    def _desktop():
        desktop = {key: os.environ[key] for key in DESKTOP_KEYS if os.environ.get(key)}
        # rviz2 uses the available graphical login; a headless start is not a
        # core failure and must not be silently redirected to another display.
        if not desktop.get('DISPLAY'):
            raise ValueError('desktop_unavailable')
        if any(len(value) > 4096 or '\n' in value or '\r' in value or '\x00' in value
               for value in desktop.values()):
            raise ValueError('desktop_environment_invalid')
        return desktop

    def open(self, state, layout):
        if layout not in VIEW_UNITS:
            raise ValueError('unknown_view_layout')
        with self._transaction():
            identifier, directory, release, entry = self._binding(state)
            if layout != 'global' and state.get('view_contract') != VIEW_CONTRACT:
                raise ValueError('view_layout_switch_unsupported')
            core = self._require_core(state, identifier)
            for name in LEGACY_VIEW_UNITS.values():
                if self._busy(self._unit(name)):
                    raise ValueError('legacy_view_conflict_needs_upgrade')
            unit = self._unit(VIEW_UNIT)
            if self._busy(unit):
                record = self._record(state, 'single', unit, identifier, directory, release)
                return self._layout(state, layout, identifier, directory, core, record)
            desktop = self._desktop()
            viewer_id = uuid.uuid4().hex
            command = ['systemd-run', '--user', '--collect', '--unit=' + VIEW_UNIT,
                       '--property=Description=' + self._description(identifier, viewer_id),
                       '--property=Type=exec', '--property=KillMode=control-group',
                       '--property=KillSignal=SIGTERM', '--property=TimeoutStopSec=12',
                       '--property=BindsTo=' + CORE_UNIT, '--property=After=' + CORE_UNIT,
                       '--property=StandardOutput=append:' + str(directory / 'view-single.log'),
                       '--property=StandardError=append:' + str(directory / 'view-single.log'),
                       '--setenv=D1MAX_NAV_ROOT=' + str(self.nav_root),
                       '--setenv=D1MAX_RELEASE=' + str(release),
                       '--setenv=D1MAX_NAV_TRANSPORT=live',
                       '--setenv=D1MAX_NAV_RVIZ_VIEWER_ID=' + viewer_id]
            command += ['--setenv=' + key + '=' + desktop[key] for key in sorted(desktop)]
            command += [str(entry), 'view', '--session', str(directory), '--layout', layout]
            self.runner(command, check=True, capture_output=True, text=True, timeout=10)
            # Reopening a closed view creates a new invocation.  Only the
            # inactive record is superseded; a live foreign invocation above
            # was rejected without stopping or adopting it.
            fresh_state = {**state, 'views': {**state.get('views', {}), 'single': None}}
            record = self._record(fresh_state, 'single', self._unit(VIEW_UNIT), identifier, directory, release)
            return self._layout(state, layout, identifier, directory, core, record)

    def stop(self, state):
        """Close only exact viewers; navigation cancellation is outside this API."""
        with self._transaction():
            identifier, directory, release, _ = self._binding(state)
            owned = []
            # Validate all three before changing any; a foreign or replacement
            # process must never be killed merely because its unit is fixed.
            for layout, name in {'single': VIEW_UNIT, **LEGACY_VIEW_UNITS}.items():
                unit = self._unit(name)
                if self._busy(unit):
                    record = self._record(state, layout, unit, identifier, directory, release,
                                          allow_stopping=True)
                    owned.append((layout, record))
            result = {}
            for layout, record in owned:
                self.runner(['systemctl', '--user', 'stop', record['unit']], check=True,
                            capture_output=True, text=True, timeout=16)
                after = self._unit(record['unit'])
                if self._busy(after):
                    raise ValueError('view_stop_unconfirmed:' + layout)
                result[layout] = {**record, 'phase': 'stopped', 'exit_code': after.get('ExecMainStatus')}
            return result
