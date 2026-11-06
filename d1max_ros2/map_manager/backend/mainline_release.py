"""Read-only release diagnostics, never service activation or motion authority.

The Web may poll this object frequently. Only small selection/activation/tool
inputs are read on the request thread; large sealed-file checks run in one
bounded worker. Starting the graph must independently revalidate its activation.
"""
import hashlib
import importlib.util
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
_DESCRIPTOR_MODULES = {'d1max_pct_scan.navigation_session', _ENTRY_MODULE}
_CHECK_TIMEOUT_S = 60.


def source_backed_python_cache(path, files):
    """Exclude only source-backed import bytecode, not a __pycache__ subtree.

    Stdlib-only copy of the CLI/SDK boundary rule; cross-entry tests bind its
    behavior without importing unverified runtime code during integrity checks.
    The .py remains a separately checked member of the same manifest.
    """
    path = Path(path)
    if not path.is_absolute() or path.suffix not in ('.pyc', '.pyo') or path.is_symlink():
        return False
    try:
        source = (Path(importlib.util.source_from_cache(str(path.with_suffix('.pyc'))))
                  if path.parent.name == '__pycache__' else path.with_suffix('.py'))
    except ValueError:
        # source_from_cache assumes no dots in the source stem and cannot
        # decode pytest's versioned rewrite tag. Both are real Python caches.
        # Match only those known tags, then retain the same sealed-source test.
        match = re.fullmatch(
            r'(.+)\.cpython-[0-9]+(?:-pytest-[0-9][A-Za-z0-9_.+-]*)?'
            r'(?:\.opt-[A-Za-z0-9]+)?\.py[co]', path.name)
        if path.parent.name != '__pycache__' or match is None:
            return False
        source = path.parent.parent/(match.group(1)+'.py')
    return (str(source) in files and source.is_file() and not source.is_symlink())


def root_dispatchers(nav):
    """The two names for the same root launcher; frozen tools are not entries."""
    return (nav/'tools/navigation_entry.sh', nav/'tools/single_floor_entry.sh')


def activation_launch_config(value):
    """Resolve one navigation launcher without a user-facing preview switch.

    A sealed runtime profile selects the execution graph and binds the current
    SDK session on start. Legacy physical records retain fixed-session binding.
    This is graph capability, never a goal confirmation or movement permit.
    Legacy configurations and already admitted sessions retain their contract.
    No profile or SDK boolean grants an execution permit to a robot task.
    """
    if not isinstance(value, dict):
        raise ValueError('activation_schema')
    result = dict(value)
    if 'launch_policy' not in result:
        if result.get('purpose') not in ('planning_only', 'execution'):
            raise ValueError('explicit_activation_purpose_required')
        if result.get('purpose')=='execution':
            result['motion_profile_record']=result.get('physical_acceptance_record','')
            result['motion_profile_sha256']=result.get('physical_acceptance_record_sha256','')
        return result
    if result['launch_policy'] != 'navigation':
        raise ValueError('navigation_launch_policy_invalid')
    if 'purpose' in result or 'sdk_session_policy' in result:
        raise ValueError('navigation_launch_policy_conflicts_with_legacy_mode')
    runtime_keys=('runtime_motion_profile','runtime_motion_profile_sha256')
    if any(key in result for key in runtime_keys):
        if (not all(key in result for key in runtime_keys)
                or any(key in result for key in ('physical_acceptance_record',
                    'physical_acceptance_record_sha256','expected_sdk_session'))):
            raise ValueError('navigation_motion_profile_binding_incomplete_or_mixed')
        record,record_hash=(result[key] for key in runtime_keys)
        if (not isinstance(record,str) or not Path(record).is_absolute()
                or not isinstance(record_hash,str) or not re.fullmatch(r'[0-9a-f]{64}',record_hash)):
            raise ValueError('navigation_motion_profile_binding_invalid')
        result.update(purpose='execution',sdk_session_policy='bind_current_on_start',
            motion_profile_record=record,motion_profile_sha256=record_hash)
        return result
    keys = ('physical_acceptance_record', 'physical_acceptance_record_sha256',
            'expected_sdk_session')
    present = [key in result for key in keys]
    if any(present):
        if not all(present):
            raise ValueError('navigation_execution_binding_incomplete')
        record, record_hash, session = (result[key] for key in keys)
        if (not isinstance(record, str) or not Path(record).is_absolute()
                or not isinstance(record_hash, str) or not re.fullmatch(r'[0-9a-f]{64}', record_hash)
                or not isinstance(session, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,128}', session)):
            raise ValueError('navigation_execution_binding_invalid')
        result.update(purpose='execution', sdk_session_policy='fixed',
            motion_profile_record=record,motion_profile_sha256=record_hash)
    else:
        result.update(purpose='planning_only', sdk_session_policy='bind_current_on_start')
    return result


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
    'physical_record_invalid': '运动配置缺失或变化',
    'release_check_timeout': '发布校验超时',
    'release_invalid': '发布版本校验失败',
}


class ReleaseDiagnosticError(ValueError):
    def __init__(self, code, detail=''):
        super().__init__(code + (':'+str(detail) if detail else ''))
        self.code = code
        self.detail = str(detail)


def _digest(path, *, deadline=None, check_cancelled=None):
    digest = hashlib.sha256()
    try:
        # A removed/replaced dependency is a failed check, not permission to
        # block the only verifier on a FIFO/device before its timeout applies.
        # Manifest paths may intentionally name regular symlink targets; byte
        # integrity is still checked against the pinned manifest below.
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
        with os.fdopen(fd, 'rb') as stream:
            if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
                raise ReleaseDiagnosticError('release_invalid', 'regular_file_required:'+str(path))
            while True:
                if check_cancelled:check_cancelled()
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
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK)
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
    # Both public names dispatch to the same owner and versioned contract.
    # The selector retains its compatibility schema; the current sealed
    # descriptor uses the generic indoor/outdoor navigation name.
    if value.get('entry_module', _ENTRY_MODULE) not in _DESCRIPTOR_MODULES:
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


def verify_release_files(nav, release, seal, *, expected_manifest_sha256=None,
                         runner=None, check_cancelled=None,require_admitted_tools=False):
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
    ignored_caches = 0
    for name, expected in files.items():
        path = Path(name)
        if not path.is_absolute() or not re.fullmatch(r'[0-9a-f]{64}', str(expected)):
            raise ReleaseDiagnosticError('release_invalid', 'file_identity')
        if source_backed_python_cache(path, files):
            ignored_caches += 1
            continue
        if _digest(path, deadline=deadline, check_cancelled=check_cancelled) != expected:
            raise ReleaseDiagnosticError('release_file_changed', path)
    if files.get(str(release/'release.json')) != _digest(release/'release.json', deadline=deadline,check_cancelled=check_cancelled):
        raise ReleaseDiagnosticError('release_descriptor_not_sealed')
    tools = _tools(nav, release, descriptor)
    if require_admitted_tools and 'tools_source_root' in descriptor and manifest.get('actual_tools_root')!=str(tools):
        raise ReleaseDiagnosticError('release_tools_not_sealed','tools_binding')
    if require_admitted_tools and 'tools_source_root' in descriptor and 'startup_closure' not in manifest:
        raise ReleaseDiagnosticError('startup_closure_missing')
    for relative in ('single_floor_entry.sh', 'release/single_floor_entry_preflight.py'):
        path = tools/relative
        if path.is_symlink() or not path.is_file() or files.get(str(path.resolve())) != _digest(path, deadline=deadline,check_cancelled=check_cancelled):
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
            if check_cancelled:check_cancelled()
            result = (runner or subprocess.run)(['/usr/bin/python3', str(tools/'release/single_floor_entry_preflight.py'),
                'startup-check', '--release', str(release)], env=environment,
                capture_output=True, text=True, timeout=timeout, check=False)
        except subprocess.TimeoutExpired as error:
            raise ReleaseDiagnosticError('release_check_timeout') from error
        if result.returncode:
            raise ReleaseDiagnosticError('startup_closure_invalid', result.stderr.strip()[:512])
    return dict(manifest_sha256=manifest_hash, verified_files=len(files)-ignored_caches,
                ignored_runtime_caches=ignored_caches,
                release_contract='frozen_startup_schema3' if new_candidate else 'legacy_schema3',
                startup_closure_verified='startup_closure' in manifest,
                motion_authorized=False, sdk_connected=False)


class MainlineReleaseProbe:
    """One in-flight refresh of identity-bound diagnostics, never admission."""
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
            activation = activation_launch_config(activation)
            session_policy=activation.get('sdk_session_policy','fixed')
            dynamic_session=(session_policy=='bind_current_on_start' and
                (activation.get('purpose')=='planning_only' or bool(activation.get('runtime_motion_profile'))))
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
                record = Path(activation.get('motion_profile_record', ''))
                if (not record.is_absolute() or not record.is_file()
                        or _digest(record) != activation.get('motion_profile_sha256')):
                    raise ReleaseDiagnosticError('physical_record_invalid')
                if activation.get('runtime_motion_profile'):
                    # Match launch admission: a checksum of an arbitrary
                    # external/physical record is not this sealed runtime
                    # profile, and readiness never grants motion authority.
                    manifest, _ = _json(target['seal'], limit=_MANIFEST_LIMIT)
                    profile, _ = _json(record)
                    if (not record.resolve(strict=True).is_relative_to(target['release'])
                            or manifest.get('files', {}).get(str(record.resolve())) != activation['motion_profile_sha256']
                            or profile.get('record_kind') != 'runtime_motion_profile'
                            or profile.get('physical_acceptance_verified') is not False):
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
            same_key = cached is not None and cached[0] == key
            fresh = same_key and now-cached[1] < self.ttl_s
            # TTL schedules another full check without withdrawing completed
            # diagnostics for unchanged identity. Preserve the original check
            # timestamp; a new identity or completed failure replaces readiness.
            # The actual start transaction always revalidates the files itself.
            details = cached[2] if same_key else {}
            if not fresh and self._worker is None and (selected or configured):
                self._worker = threading.Thread(target=self._work,
                    args=(key, selected, configured), name='mainline-release-probe', daemon=True)
                try:self._worker.start()
                except RuntimeError:
                    self._worker=None
                    details={kind:self._invalid(ReleaseDiagnosticError('release_check_timeout'))
                             for kind in ('selected','configured')}
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
