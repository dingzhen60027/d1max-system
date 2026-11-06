#!/usr/bin/env python3
"""Synchronize CLI/Web bindings to a reviewed, sealed default, without launching it.

The runtime must never silently trust changed launch code. Deployment updates
all derived hashes together, after matching both root dispatchers to the sealed
source copies. Existing launch purpose, SDK policy and acceptance fields are
preserved. This is a deployment operation, not a Web startup auto-repair.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import tempfile


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def regular(path):
    path = Path(path)
    require(not path.is_symlink() and stat.S_ISREG(path.stat().st_mode),
            'regular_file_required:' + str(path))
    return path


def read(path):
    return json.loads(regular(path).read_text())


def derive(nav, activation, expected_manifest, previous_manifest=None):
    nav = Path(nav).resolve(strict=True)
    activation = regular(activation)
    require(stat.S_IMODE(activation.stat().st_mode) == 0o600, 'activation_must_remain_private')
    selector = nav / 'deploy/single_floor_release.json'
    selected, active = read(selector), read(activation)
    relative = Path(selected['release_directory'])
    require(not relative.is_absolute() and '..' not in relative.parts, 'release_directory_invalid')
    release = (nav / relative).resolve(strict=True)
    require(release.is_relative_to(nav / 'experiments'), 'release_outside_workspace_experiments')
    seal = release / 'release_manifest.json'
    require(digest(regular(seal)) == expected_manifest, 'reviewed_manifest_changed')
    manifest = read(seal)
    require(active['release_root'] == str(release) and active['release_manifest'] == str(seal),
            'web_and_cli_select_different_release')
    allowed = {expected_manifest, previous_manifest} - {None}
    require(selected['release_manifest_sha256'] in allowed and
            active['release_manifest_sha256'] in allowed, 'selection_changed_since_review')
    for name, expected in manifest['files'].items():
        require(digest(regular(name)) == expected, 'sealed_member_changed:' + name)
    entry = Path(active['entrypoint'])
    require(entry in (nav / 'tools/navigation_entry.sh', nav / 'tools/single_floor_entry.sh'),
            'unknown_root_dispatcher')
    launchers = [entry]
    if entry.name == 'navigation_entry.sh':
        launchers.append(nav / 'tools/single_floor_entry.sh')
    hashes = {}
    for launcher in launchers:
        regular(launcher)
        require(os.access(launcher, os.X_OK), 'dispatcher_not_executable')
        frozen = release / 'source_snapshot/nav_ws' / launcher.relative_to(nav)
        value = digest(launcher)
        require(manifest['files'].get(str(frozen)) == value == digest(regular(frozen)),
                'root_dispatcher_differs_from_reviewed_snapshot:' + str(launcher))
        hashes[launcher.name] = value
    updated_selector = dict(selected, release_manifest_sha256=expected_manifest)
    updated_active = dict(active, release_manifest_sha256=expected_manifest,
                          entrypoint_sha256=hashes[entry.name])
    if entry.name == 'navigation_entry.sh':
        updated_active['compatibility_entrypoint_sha256'] = hashes['single_floor_entry.sh']
    return [(selector, selected, updated_selector), (activation, active, updated_active)]


def atomic_json(path, value):
    path = Path(path)
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name + '.binding-', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, indent=2, ensure_ascii=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, stat.S_IMODE(path.stat().st_mode))
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def synchronize(nav, activation, expected_manifest, *, previous_manifest=None, backup=None):
    records = derive(nav, activation, expected_manifest, previous_manifest)
    if backup is None:
        require(all(old == new for _, old, new in records), 'derived_bindings_not_synchronized')
        return
    backup = Path(backup)
    backup.mkdir(parents=True, exist_ok=False)
    for index, (path, _, _) in enumerate(records):
        shutil.copy2(path, backup / (str(index) + '-' + path.name))
    # Re-check all inputs before changing either pointer. A partial crash remains
    # fail-closed and has exact backups; retry needs an explicit new review.
    require(records == derive(nav, activation, expected_manifest, previous_manifest),
            'binding_inputs_changed_before_commit')
    for path, old, new in records:
        require(read(path) == old, 'binding_changed_during_commit')
        if old != new:
            atomic_json(path, new)
    synchronize(nav, activation, expected_manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nav-root', type=Path, required=True)
    parser.add_argument('--activation', type=Path, required=True)
    parser.add_argument('--expected-manifest', required=True)
    parser.add_argument('--previous-manifest')
    parser.add_argument('--backup', type=Path, help='Explicitly apply; otherwise verify read-only')
    args = parser.parse_args()
    synchronize(args.nav_root, args.activation, args.expected_manifest,
                previous_manifest=args.previous_manifest, backup=args.backup)
    print(json.dumps(dict(bindings_verified=True, manifest=args.expected_manifest,
                          applied=args.backup is not None, services_started=False)))


if __name__ == '__main__':
    main()
