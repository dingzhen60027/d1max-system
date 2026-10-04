"""Fail-closed runtime contract for the bundled PCT native planner.

GTSAM 4.1 and 4.2 share an ELF SONAME but are not interchangeable here.  ROS
may put 4.2 ahead of PCT's build RUNPATH.  Prepare a *child process* environment
before starting Python, then probe before and after importing native bindings.
This module never loads libraries or changes os.environ / ROS configuration.
"""
import hashlib
import os
from pathlib import Path
import re
import shlex


class NativeRuntimeError(RuntimeError):
    def __init__(self, code, message, **details):
        super().__init__(message)
        self.code, self.details = code, details

    def as_dict(self):
        return {'code': self.code, 'message': str(self), **self.details}


_NATIVE_BUILD_DIRS = {
    'libgpmp_optimizer.so': 'trajectory_optimization',
    'libmap_manager.so': 'map_manager',
    'libele_planner_lib.so': 'ele_planner',
    'liba_star_search.so': 'a_star',
    'libcommon_smoothing.so': 'common/smoothing',
}


def _required(path):
    try:
        path = Path(path).resolve(strict=True)
        if not path.is_file():
            raise OSError('not a regular file')
        return path
    except OSError as exc:
        raise NativeRuntimeError('native_build_unverified',
                                 'Required PCT build evidence or library is missing',
                                 path=str(path)) from exc


def _digest(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def vendorverify(vendor_root):
    """Verify installed binaries against CMake's actual GTSAM link evidence.

    Missing build evidence is an error: version-number guesses or system-library
    fallbacks cannot establish ABI compatibility. Returned paths are absolute.
    """
    from .paths import expand, vendor_checkout
    root = Path(expand(vendor_root) or vendor_checkout()).expanduser().resolve()
    lib_root = root / 'planner/lib'
    headers = sorted((lib_root / '3rdparty').glob('gtsam-*/install/include/gtsam/config.h'))
    if len(headers) != 1:
        raise NativeRuntimeError('native_build_unverified',
                                 'Expected one verifiable bundled GTSAM installation',
                                 vendor_root=str(root), config_headers=list(map(str, headers)))
    header = _required(headers[0])
    install = header.parents[2]
    version_match = re.search(r'^#define\s+GTSAM_VERSION_STRING\s+"([0-9.]+)"',
                              header.read_text(), flags=re.M)
    if not version_match:
        raise NativeRuntimeError('native_build_unverified', 'Missing compiled GTSAM version',
                                 path=str(header))
    version = version_match.group(1)
    gtsam = _required(install / 'lib/libgtsam.so')
    if not gtsam.is_relative_to(install) or gtsam.name != 'libgtsam.so.' + version:
        raise NativeRuntimeError('native_build_unverified',
                                 'Bundled GTSAM library does not match its compile header',
                                 expected_version=version, actual=str(gtsam))
    metis = _required(install / 'lib/libmetis-gtsam.so')
    if not metis.is_relative_to(install):
        raise NativeRuntimeError('native_build_unverified', 'Bundled METIS escapes installation',
                                 actual=str(metis))

    target = lib_root / 'build/src/trajectory_optimization'
    link_file = _required(target / 'CMakeFiles/gpmp_optimizer.dir/link.txt')
    flags_file = _required(target / 'CMakeFiles/gpmp_optimizer.dir/flags.make')
    linked = []
    for token in shlex.split(link_file.read_text()):
        if re.fullmatch(r'libgtsam\.so(?:\.[0-9]+)+', Path(token).name):
            linked.append(_required(target / token))
    if linked != [gtsam] or str(install) not in flags_file.read_text():
        raise NativeRuntimeError('native_build_unverified',
                                 'PCT CMake compile/link evidence does not match bundled GTSAM',
                                 linked=list(map(str, linked)), expected=str(gtsam))

    libraries, hashes = {}, {}
    for name, directory in _NATIVE_BUILD_DIRS.items():
        installed = _required(lib_root / name)
        built = _required(lib_root / 'build/src' / directory / name)
        installed_hash = _digest(installed)
        if installed_hash != _digest(built):
            raise NativeRuntimeError('native_build_unverified',
                                     'Installed PCT binary differs from its verified build',
                                     installed=str(installed), built=str(built))
        libraries[name] = str(installed)
        hashes[name] = installed_hash
    return {
        'vendor_root': str(root), 'gtsam_version': version,
        'gtsam_library': str(gtsam), 'gtsam_lib_dir': str(gtsam.parent),
        'metis_library': str(metis), 'native_lib_dir': str(lib_root),
        'native_libraries': libraries, 'native_sha256': hashes,
        'build_evidence': {'compile_flags': str(flags_file), 'link_command': str(link_file),
                           'gtsam_config_header': str(header)},
    }


def prepare_native_environment(vendor_root, base_env=None):
    """Return environment for subprocess.Popen(..., env=...), before exec.

    Changing LD_LIBRARY_PATH inside an already started interpreter cannot repair
    a previously loaded ABI. A conflicting LD_PRELOAD is rejected, not hidden.
    Existing ROS / Zenoh and other application variables are preserved.
    """
    evidence = vendorverify(vendor_root)
    env = dict(os.environ if base_env is None else base_env)
    allowed = {evidence['gtsam_library'], evidence['metis_library'],
               *evidence['native_libraries'].values()}
    for token in re.split(r'[:\s]+', env.get('LD_PRELOAD', '')):
        if not token:
            continue
        name = Path(token).name
        if ('gtsam' in name or name in _NATIVE_BUILD_DIRS) and str(Path(token).resolve()) not in allowed:
            raise NativeRuntimeError('native_abi_mismatch',
                                     'Conflicting LD_PRELOAD; launch a clean PCT worker',
                                     actual=token, relaunch_required=True)
    # Empty entries mean current directory to the loader; intentionally omit.
    directories = [evidence['gtsam_lib_dir'], evidence['native_lib_dir']]
    directories += [p for p in env.get('LD_LIBRARY_PATH', '').split(':') if p]
    env['LD_LIBRARY_PATH'] = ':'.join(dict.fromkeys(directories))
    env['D1MAX_PCT_EXPECTED_GTSAM'] = evidence['gtsam_library']
    from .compute_budget import prepare_environment
    return prepare_environment(env)


def _mapped_libraries(text):
    entries = {}
    for line in text.splitlines():
        columns = line.split(None, 5)
        if len(columns) != 6 or not columns[5].startswith('/'):
            continue
        raw = columns[5]
        deleted = raw.endswith(' (deleted)')
        filename = raw[:-10] if deleted else raw
        name = Path(filename).name
        if name.startswith('libgtsam.so') or name == 'libmetis-gtsam.so' or name in _NATIVE_BUILD_DIRS:
            try:
                inode = int(columns[4])
            except ValueError as exc:
                raise NativeRuntimeError('native_runtime_unverified',
                                         'Malformed native mapping inode') from exc
            entries[filename] = {'name': name, 'inode': inode, 'deleted': deleted}
    return entries


def probe_native_libraries(vendor_root, *, require_loaded=False, maps_text=None):
    """Check actual /proc/self/maps before/after native binding imports.

    Before import, absence is allowed but any loaded conflicting library fails.
    After import pass require_loaded=True: both GTSAM and all core PCT native
    modules must be present and match this verified vendor build. maps_text is
    an injectable fixture for tests; production callers must omit it.
    """
    evidence = vendorverify(vendor_root)
    if maps_text is None:
        try:
            maps_text = Path('/proc/self/maps').read_text()
        except OSError as exc:
            raise NativeRuntimeError('native_runtime_unverified',
                                     'Cannot inspect actual native library mappings') from exc
    loaded = _mapped_libraries(maps_text)
    seen = set()
    verified = []
    for filename, entry in loaded.items():
        path = Path(filename)
        name = entry['name']
        if name.startswith('libgtsam.so'):
            key, expected = 'gtsam', evidence['gtsam_library']
        elif name == 'libmetis-gtsam.so':
            key, expected = 'metis', evidence['metis_library']
        else:
            key, expected = name, evidence['native_libraries'][name]
        reason = None
        try:
            actual = path.resolve(strict=True)
            if entry['deleted'] or actual.stat().st_ino != entry['inode']:
                reason = 'mapped file was deleted or replaced'
            elif key in ('gtsam', 'metis'):
                if actual != Path(expected):
                    reason = 'wrong dependency instance / ABI'
            elif not actual.is_relative_to(Path(evidence['vendor_root'])):
                reason = 'PCT shared library is outside this vendor build'
            elif _digest(actual) != evidence['native_sha256'][name]:
                reason = 'PCT shared library differs from verified build'
        except OSError:
            reason = 'mapped library cannot be verified'
        if reason:
            raise NativeRuntimeError('native_abi_mismatch',
                                     'PCT native ABI mismatch; relaunch in the prepared child environment',
                                     expected=expected, actual=filename, reason=reason,
                                     relaunch_required=True)
        seen.add(key)
        verified.append(str(actual))
    required = {'gtsam', 'metis', *_NATIVE_BUILD_DIRS}
    missing = sorted(required - seen)
    if require_loaded and missing:
        raise NativeRuntimeError('native_runtime_unverified',
                                 'PCT imports did not load all verified native libraries',
                                 missing=missing, relaunch_required=True)
    return {**evidence, 'loaded_libraries': sorted(verified),
            'runtime_verified': not missing, 'missing_libraries': missing}
