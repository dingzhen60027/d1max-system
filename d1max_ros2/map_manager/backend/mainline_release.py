"""Read-only release diagnostics, never service activation or motion authority.

The Web may poll this object frequently. Only small selection/activation/tool
inputs are read on the request thread; large sealed-file checks run in one
bounded worker. Starting the graph must independently revalidate its activation.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import threading
import time


_SMALL_LIMIT = 1048576
_MANIFEST_LIMIT = 33554432
_ENTRY_MODULE = 'd1max_pct_scan.single_floor_session'
_CHECK_TIMEOUT_S = 60.


def root_dispatchers(nav):
    """The two names for the same root launcher; frozen tools are not entries."""
    return (nav/'tools/navigation_entry.sh', nav/'tools/single_floor_entry.sh')


_REASONS = {
    'ready': '发布版本校验通过',
    'checking': '正在核对发布版本',
    'missing_activation': '未配置主线发布版本',
    'selector_missing': '未配置默认发布版本',
    'selector_invalid': '默认版本配置无效',
    'activation_invalid': '主线版本配置无效',
    'activation_private_required': '版本配置权限不正确',
    'entrypoint_changed': '启动入口已变化',
    'release_manifest_missing': '发布版本尚未封存',
    'release_manifest_changed': '发布清单已变化',
    'release_file_changed': '发布文件已变化',
    'release_file_missing': '发布文件缺失',
    'release_descriptor_invalid': '发布描述无效',
    'release_descriptor_not_sealed': '发布描述未封存',
    'release_tools_not_sealed': '启动工具未封存',
    'startup_closure_missing': '启动依赖尚未封存',
    'startup_closure_invalid': '启动依赖校验失败',
    'physical_record_invalid': '验收记录缺失或变化',
    'release_check_timeout': '发布校验超时',
    'release_invalid': '发布版本校验失败',
}


class ReleaseDiagnosticError(ValueError):
    def __init__(self, code, detail=''):
        super().__init__(code + (':'+str(detail) if detail else ''))
        self.code = code
        self.detail = str(detail)


def _digest(path, *, deadline=None):
    digest = hashlib.sha256()
    try:
        with Path(path).open('rb') as stream:
            while True:
                if deadline is not None and time.monotonic() >= deadline:
                    raise ReleaseDiagnosticError('release_check_timeout')
                block = stream.read(1048576)
                if not block:
                    break
                digest.update(block)
    except (FileNotFoundError, NotADirectoryError) as error:
        raise ReleaseDiagnosticError('release_file_missing', path) from error
    return digest.hexdigest()


def _json(path, *, limit=_SMALL_LIMIT, private=False):
    path = Path(path)
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        with os.fdopen(fd, 'rb') as stream:
            info = os.fstat(stream.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_size > limit:
                raise ReleaseDiagnosticError('release_invalid', path)
            if private and (info.st_uid != os.getuid() or info.st_mode & 0o077):
                raise ReleaseDiagnosticError('activation_private_required')
            raw = stream.read(limit+1)
            if len(raw) > limit:
                raise ReleaseDiagnosticError('release_invalid', path)
    except OSError as error:
        if path.is_symlink():
            raise ReleaseDiagnosticError('release_invalid', 'symlink:'+str(path)) from error
        raise
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ReleaseDiagnosticError('release_invalid', path)
    return value, hashlib.sha256(raw).hexdigest()


def _release(nav, value):
    root = Path(value)
    if not root.is_absolute():
        raise ReleaseDiagnosticError('release_descriptor_invalid', 'release_must_be_absolute')
    try:
        root = root.resolve(strict=True)
    except OSError as error:
        raise ReleaseDiagnosticError('release_manifest_missing') from error
    if not root.is_dir() or not root.is_relative_to(nav/'experiments'):
        raise ReleaseDiagnosticError('release_descriptor_invalid', 'outside_experiments')
    return root


def _descriptor(release):
    try:
        value, identity = _json(release/'release.json')
    except OSError as error:
        raise ReleaseDiagnosticError('release_descriptor_invalid', 'missing') from error
    name = Path(value.get('sealed_manifest', ''))
    if (value.get('schema') != 1 or not name.parts or name.is_absolute()
            or len(name.parts) != 1 or name.name in ('.', '..')):
        raise ReleaseDiagnosticError('release_descriptor_invalid')
    if value.get('entry_module', _ENTRY_MODULE) != _ENTRY_MODULE:
        raise ReleaseDiagnosticError('release_descriptor_invalid', 'entry_module')
    return value, identity, release/name


def _tools(nav, release, descriptor):
    if 'tools_source_root' not in descriptor:
        if 'template_source_root' in descriptor:
            raise ReleaseDiagnosticError('release_tools_not_sealed')
        return nav/'tools'
    relative = Path(descriptor['tools_source_root'])
    if relative.is_absolute() or '..' in relative.parts or not relative.parts:
        raise ReleaseDiagnosticError('release_descriptor_invalid', 'tools_source_root')
    cursor = release
    for part in relative.parts:
        cursor = cursor/part
        if cursor.is_symlink():
            raise ReleaseDiagnosticError('release_tools_not_sealed', 'symlink')
    try:
        root = cursor.resolve(strict=True)
    except OSError as error:
        raise ReleaseDiagnosticError('release_tools_not_sealed', 'missing') from error
    if not root.is_relative_to(release) or not root.is_dir():
        raise ReleaseDiagnosticError('release_tools_not_sealed')
    return root


def _rmw_prefix():
    if os.environ.get('D1MAX_RMW_PREFIX'):
        return os.environ['D1MAX_RMW_PREFIX']
    app = Path(os.environ.get('D1MAX_APP_ROOT') or
        str(Path(os.environ.get('D1MAX_VENDOR_ROOT') or
            str(Path.home()/'智元四足机器人D1 Max二次开发文档资料包v0.1.0'))/'d1max_ros2'))
    return str(app/'local/opt/ros/humble')


def verify_release_files(nav, release, seal, *, expected_manifest_sha256=None):
    """Synchronous integrity check for a launch transaction, not authorization.

    No setup script, ROS import, ctypes loader or SDK is executed. The optional
    startup subprocess invokes only the preflight's pure-stdlib startup-check.
    Legacy seals retain their own contract; they are not relabelled as new ones.
    """
    nav = Path(nav).resolve(strict=True)
    release = _release(nav, release)
    descriptor, _, declared_seal = _descriptor(release)
    seal = Path(seal)
    if seal.is_symlink() or not seal.is_absolute() or seal != declared_seal:
        raise ReleaseDiagnosticError('release_descriptor_invalid', 'manifest_binding')
    if not seal.is_file():
        raise ReleaseDiagnosticError('release_manifest_missing')
    manifest, manifest_hash = _json(seal, limit=_MANIFEST_LIMIT)
    if expected_manifest_sha256 is not None and manifest_hash != expected_manifest_sha256:
        raise ReleaseDiagnosticError('release_manifest_changed')
    files = manifest.get('files')
    if manifest.get('schema') != 3 or not isinstance(files, dict) or not files:
        raise ReleaseDiagnosticError('release_invalid', 'manifest_schema')
    deadline = time.monotonic()+_CHECK_TIMEOUT_S
    for name, expected in files.items():
        path = Path(name)
        if not path.is_absolute() or not re.fullmatch(r'[0-9a-f]{64}', str(expected)):
            raise ReleaseDiagnosticError('release_invalid', 'file_identity')
        if _digest(path, deadline=deadline) != expected:
            raise ReleaseDiagnosticError('release_file_changed', path)
    if files.get(str(release/'release.json')) != _digest(release/'release.json', deadline=deadline):
        raise ReleaseDiagnosticError('release_descriptor_not_sealed')
    tools = _tools(nav, release, descriptor)
    for relative in ('single_floor_entry.sh', 'release/single_floor_entry_preflight.py'):
        path = tools/relative
        if path.is_symlink() or not path.is_file() or files.get(str(path.resolve())) != _digest(path, deadline=deadline):
            raise ReleaseDiagnosticError('release_tools_not_sealed', path)
    new_candidate = 'template_source_root' in descriptor
    if new_candidate and not isinstance(manifest.get('startup_closure'), dict):
        raise ReleaseDiagnosticError('startup_closure_missing')
    if 'startup_closure' in manifest:
        environment = dict(os.environ, D1MAX_NAV_ROOT=str(nav),
                           D1MAX_RELEASE=str(release), D1MAX_RMW_PREFIX=_rmw_prefix())
        timeout = min(10., max(0., deadline-time.monotonic()))
        if timeout <= 0:
            raise ReleaseDiagnosticError('release_check_timeout')
        try:
            result = subprocess.run(['/usr/bin/python3', str(tools/'release/single_floor_entry_preflight.py'),
                'startup-check', '--release', str(release)], env=environment,
                capture_output=True, text=True, timeout=timeout, check=False)
        except subprocess.TimeoutExpired as error:
            raise ReleaseDiagnosticError('release_check_timeout') from error
        if result.returncode:
            raise ReleaseDiagnosticError('startup_closure_invalid', result.stderr.strip()[:512])
    return dict(manifest_sha256=manifest_hash, verified_files=len(files),
                release_contract='frozen_startup_schema3' if new_candidate else 'legacy_schema3',
                startup_closure_verified='startup_closure' in manifest,
                motion_authorized=False, sdk_connected=False)


class MainlineReleaseProbe:
    """One in-flight check and one TTL result; polling never queues more jobs."""
    def __init__(self, nav_root, activation_path, *, ttl_s=30., verifier=None, clock=None):
        self.nav = Path(nav_root).resolve()
        self.activation = Path(activation_path)
        self.ttl_s = max(.1, min(300., float(ttl_s)))
        self._verify = verifier or verify_release_files
        self._clock = clock or time.monotonic
        self._lock = threading.Lock()
        self._worker = None
        self._requested_key = None
        self._cached = None

    @staticmethod
    def _invalid(error, default='release_invalid'):
        code = error.code if isinstance(error, ReleaseDiagnosticError) else default
        return {'readiness': 'invalid', 'reason_code': code,
                'reason': _REASONS.get(code, _REASONS['release_invalid'])}

    def _target(self, release, expected):
        release = _release(self.nav, release)
        descriptor, descriptor_hash, seal = _descriptor(release)
        tools = _tools(self.nav, release, descriptor)
        fingerprints = []
        for path in (*root_dispatchers(self.nav),
                     self.nav/'tools/release/single_floor_entry_preflight.py',
                     tools/'single_floor_entry.sh', tools/'release/single_floor_entry_preflight.py'):
            if path.is_file() and path.stat().st_size <= _SMALL_LIMIT:
                fingerprints.append((str(path), _digest(path)))
            else:
                fingerprints.append((str(path), None))
        info = seal.stat() if seal.is_file() else None
        seal_metadata = (info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns) if info else None
        return dict(id=release.name, release=release, seal=seal, expected=expected,
                    key=(str(release), descriptor_hash, seal_metadata, expected, tuple(fingerprints)))

    def _quick(self):
        selector_hash = activation_hash = None
        selected = configured = None
        selected_error = configured_error = None
        try:
            profile, selector_hash = _json(self.nav/'deploy/single_floor_release.json')
            relative = Path(profile.get('release_directory', ''))
            if (profile.get('schema_version') != 1 or profile.get('entry_module') != _ENTRY_MODULE
                    or profile.get('motion_purpose') != 'execution' or relative.is_absolute()
                    or '..' in relative.parts or not relative.parts
                    or not re.fullmatch(r'[0-9a-f]{64}', str(profile.get('release_manifest_sha256', '')))):
                raise ReleaseDiagnosticError('selector_invalid')
            selected = self._target(self.nav/relative, profile['release_manifest_sha256'])
        except FileNotFoundError:
            selected_error = self._invalid(ReleaseDiagnosticError('selector_missing'))
        except (OSError, ValueError, TypeError, KeyError) as error:
            selected_error = self._invalid(error, 'selector_invalid')
        activation = None
        try:
            activation, activation_hash = _json(self.activation, private=True)
            session_policy=activation.get('sdk_session_policy','fixed')
            dynamic_session=(activation.get('purpose')=='planning_only' and
                             session_policy=='bind_current_on_start')
            if (activation.get('schema_version') != 3 or activation.get('profile') != 'single_floor_live'
                    or activation.get('purpose') not in ('planning_only', 'execution')
                    or session_policy not in ('fixed','bind_current_on_start')
                    or session_policy=='bind_current_on_start' and not dynamic_session
                    or not dynamic_session and not re.fullmatch(r'[A-Za-z0-9_.-]{1,128}', str(activation.get('expected_sdk_session', '')))
                    or not re.fullmatch(r'[0-9a-f]{64}', str(activation.get('release_manifest_sha256', '')))):
                raise ReleaseDiagnosticError('activation_invalid')
            configured = self._target(activation['release_root'], activation['release_manifest_sha256'])
            entry = Path(activation['entrypoint'])
            # Both root names dispatch to the same implementation. The
            # descriptor-bound frozen tools remain independently sealed.
            if (entry.is_symlink() or not entry.is_absolute() or entry not in root_dispatchers(self.nav)
                    or entry.resolve(strict=True)!=entry
                    or not entry.is_file() or entry.stat().st_size > _SMALL_LIMIT
                    or not os.access(entry,os.X_OK)
                    or _digest(entry) != activation.get('entrypoint_sha256')):
                raise ReleaseDiagnosticError('entrypoint_changed')
            if entry==self.nav/'tools/navigation_entry.sh':
                legacy_entry=self.nav/'tools/single_floor_entry.sh'
                if (legacy_entry.is_symlink() or not legacy_entry.is_file()
                        or legacy_entry.resolve(strict=True)!=legacy_entry
                        or legacy_entry.stat().st_size > _SMALL_LIMIT
                        or not os.access(legacy_entry,os.X_OK)
                        or _digest(legacy_entry)!=activation.get('compatibility_entrypoint_sha256')):
                    raise ReleaseDiagnosticError('entrypoint_changed')
            if Path(activation['release_manifest']) != configured['seal']:
                raise ReleaseDiagnosticError('activation_invalid', 'manifest_binding')
            configured['activation'] = activation
        except FileNotFoundError:
            configured_error = dict(readiness='missing_activation', reason_code='missing_activation',
                                    reason=_REASONS['missing_activation'])
        except (OSError, ValueError, TypeError, KeyError) as error:
            configured_error = self._invalid(error, 'activation_invalid')
        key = (selector_hash, activation_hash, selected['key'] if selected else None,
               configured['key'] if configured else None,
               json.dumps(selected_error, sort_keys=True), json.dumps(configured_error, sort_keys=True))
        return key, selected, configured, selected_error, configured_error

    def _check(self, target, *, verified=None):
        try:
            result = verified or self._verify(self.nav, target['release'], target['seal'],
                                             expected_manifest_sha256=target['expected'])
            activation = target.get('activation')
            if activation and activation['purpose'] == 'execution':
                record = Path(activation.get('physical_acceptance_record', ''))
                if (not record.is_absolute() or not record.is_file()
                        or _digest(record) != activation.get('physical_acceptance_record_sha256')):
                    raise ReleaseDiagnosticError('physical_record_invalid')
            return dict(result, readiness='ready', reason_code='ready', reason=_REASONS['ready'])
        except Exception as error:
            return self._invalid(error)

    def _work(self, key, selected, configured):
        selection = self._check(selected) if selected else None
        same=selected and configured and selected['key']==configured['key']
        shared=selection if same and selection['readiness']=='ready' else None
        configuration=(dict(selection) if same and selection['readiness']=='invalid' else
                       self._check(configured,verified=shared) if configured else None)
        result = {'selected': selection,
                  'configured': configuration,
                  'checked_at_unix': time.time()}
        with self._lock:
            if key == self._requested_key:
                self._cached = (key, self._clock(), result)
            self._worker = None

    def snapshot(self):
        key, selected, configured, selected_error, configured_error = self._quick()
        now = self._clock()
        with self._lock:
            self._requested_key = key
            cached = self._cached
            fresh = cached is not None and cached[0] == key and now-cached[1] < self.ttl_s
            details = cached[2] if fresh else {}
            if not fresh and self._worker is None and (selected or configured):
                self._worker = threading.Thread(target=self._work,
                    args=(key, selected, configured), name='mainline-release-probe', daemon=True)
                self._worker.start()
        pending = dict(readiness='checking', reason_code='checking', reason=_REASONS['checking'])
        selection = selected_error or details.get('selected') or pending
        configuration = configured_error or details.get('configured') or pending
        return dict(selected_id=selected['id'] if selected else None,
            configured_id=configured['id'] if configured else None,
            readiness=configuration['readiness'], reason_code=configuration['reason_code'],
            reason=configuration['reason'], manifest_sha256=(configuration.get('manifest_sha256')
                or configured['expected'] if configured else None),
            checked_at_unix=details.get('checked_at_unix'),
            selected_readiness=selection['readiness'], selected_reason_code=selection['reason_code'],
            selected_reason=selection['reason'], selected_manifest_sha256=(selection.get('manifest_sha256')
                or selected['expected'] if selected else None),
            release_contract=configuration.get('release_contract'),
            startup_closure_verified=configuration.get('startup_closure_verified', False),
            sdk_connected=False, motion_authorized=False)
