#!/usr/bin/env python3
"""Entry integrity and library loading only; no ROS initialization or SDK calls."""
import argparse
import ctypes
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def elf_dependency_paths(output):
    """ldd addresses delimit paths; whitespace is valid inside a path."""
    for line in output.splitlines():
        target = line.split('=>', 1)[-1].strip().rsplit(' (', 1)[0].strip()
        if target.startswith('/'):
            yield Path(target)


def select(nav_root):
    nav = Path(nav_root).resolve(strict=True)
    profile = json.loads((nav / 'deploy/single_floor_release.json').read_text())
    if (profile.get('schema_version') != 1
            or profile.get('entry_module') != 'd1max_pct_scan.single_floor_session'
            or profile.get('motion_purpose') != 'execution'):
        raise ValueError('official_single_floor_profile_invalid')
    relative = Path(profile['release_directory'])
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('official_release_must_be_workspace_relative')
    release = (nav / relative).resolve(strict=True)
    if not release.is_relative_to(nav / 'experiments'):
        raise ValueError('official_release_outside_experiments')
    descriptor = json.loads((release / 'release.json').read_text())
    name = Path(descriptor['sealed_manifest'])
    if name.is_absolute() or len(name.parts) != 1 or name.name in ('', '.', '..'):
        raise ValueError('release_manifest_must_be_inside_release')
    if sha(release / name) != profile['release_manifest_sha256']:
        raise ValueError('official_release_selection_hash_mismatch')
    return release


def localization_dependencies(release):
    """Resolve the optional copied-runtime component before sourcing overlays."""
    release = Path(release).resolve(strict=True)
    descriptor = json.loads((release/'release.json').read_text())
    if 'localization_dependencies_directory' not in descriptor:
        return None
    relative = Path(descriptor['localization_dependencies_directory'])
    if relative.is_absolute() or '..' in relative.parts or not relative.parts:
        raise ValueError('localization_dependencies_must_be_release_relative')
    root = (release/relative).resolve(strict=True)
    if not root.is_relative_to(release) or not (root/'local_setup.bash').is_file():
        raise ValueError('localization_dependencies_outside_release_or_setup_missing')
    return root


def tools_root(release, nav_root=None):
    release=Path(release).resolve(strict=True)
    descriptor=json.loads((release/'release.json').read_text())
    if 'tools_source_root' not in descriptor:
        if 'template_source_root' in descriptor:
            raise ValueError('candidate_requires_frozen_tools_source_root')
        if nav_root is None:
            raise ValueError('legacy_tools_require_explicit_nav_root')
        return Path(nav_root).resolve(strict=True)/'tools'
    relative=Path(descriptor['tools_source_root'])
    if relative.is_absolute() or '..' in relative.parts or not relative.parts:
        raise ValueError('tools_source_root_must_be_release_relative')
    target=release
    for part in relative.parts:
        target=target/part
        if target.is_symlink():
            raise ValueError('tools_source_root_must_be_real_copied_directory')
    target=target.resolve(strict=True)
    if not target.is_relative_to(release) or not target.is_dir():
        raise ValueError('tools_source_root_outside_release_or_not_directory')
    for name in ('single_floor_entry.sh','release/single_floor_entry_preflight.py',
                 'validation/bt_localization_replay_entry.sh','validation/run_bt_localization_replay.py'):
        path=target/name
        if path.is_symlink() or not path.is_file() or not path.resolve(strict=True).is_relative_to(target):
            raise ValueError('frozen_entry_tool_missing_or_symlink:'+name)
    return target


_STARTUP_NAME = re.compile(r'^(?:local_setup|setup|package)\.(?:bash|sh|zsh|ps1|dsv)$')
_STARTUP_UTIL = re.compile(r'^_(?:local_setup_util(?:_[a-z0-9]+)?|setup_util)\.py$')
_STARTUP_OVERRIDE = ('COLCON_PYTHON_EXECUTABLE', 'AMENT_PYTHON_EXECUTABLE',
                     'COLCON_CURRENT_PREFIX', 'AMENT_CURRENT_PREFIX')


def startup_prefixes(release, nav_root, rmw_prefix):
    """The entry's explicit source list, not an arbitrary ambient prefix scan."""
    release = Path(release).resolve(strict=True)
    result = {'ros_humble': '/opt/ros/humble',
              'workspace_base': str(Path(nav_root).resolve(strict=True)/'install'),
              'zenoh_manual_prefix': str(Path(rmw_prefix).resolve(strict=True))}
    for component in ('interfaces', 'native', 'tracker', 'bt', 'sdk', 'rviz', 'application'):
        result[component] = str(release/component/'install')
    dependencies = localization_dependencies(release)
    if dependencies is not None:
        result['localization_dependencies'] = str(dependencies)
    return result


def _walk_startup_directory(root):
    """Follow only named startup directories and fail closed on link cycles."""
    def visit(path, ancestors):
        resolved = path.resolve(strict=True)
        if resolved in ancestors:
            raise ValueError('startup_directory_symlink_cycle:'+str(path))
        if path.is_file():
            yield path
        elif path.is_dir():
            for child in sorted(path.iterdir()):
                yield from visit(child, ancestors | {resolved})
        else:
            raise ValueError('startup_path_not_regular:'+str(path))
    if root.exists() or root.is_symlink():
        yield from visit(root, set())


def _startup_files(prefix, *, require_setup=True):
    """Enumerate colcon/ament startup code and registries, excluding payloads."""
    prefix = Path(prefix).absolute()
    if not prefix.is_dir():
        raise ValueError('startup_prefix_missing:'+str(prefix))
    # A merged ROS install and an isolated colcon workspace use different
    # package roots. Neither requires executing a setup utility to discover it.
    roots = [prefix]
    roots.extend(p for p in sorted(prefix.iterdir()) if p.is_dir() and (p/'share').is_dir())
    found = set()
    for root in roots:
        for path in root.iterdir():
            if path.is_file() and (_STARTUP_NAME.fullmatch(path.name)
                    or _STARTUP_UTIL.fullmatch(path.name) or path.name == '.colcon_install_layout'):
                found.add(path)
        share = root/'share'
        if not share.is_dir():
            continue
        for directory in (share/'ament_index/resource_index', share/'colcon-core/packages'):
            found.update(_walk_startup_directory(directory))
        for package in sorted(share.iterdir()):
            if not package.is_dir() or package.name in ('ament_index', 'colcon-core'):
                continue
            for path in package.iterdir():
                if path.is_file() and _STARTUP_NAME.fullmatch(path.name):
                    found.add(path)
            for directory in (package/'hook', package/'environment'):
                found.update(_walk_startup_directory(directory))
    if require_setup and prefix/'local_setup.bash' not in found:
        raise ValueError('startup_local_setup_missing:'+str(prefix))
    # Standard ament/colcon Bash loaders call the matching sh loader/helper.
    # The copied two-package localization component deliberately has a custom
    # Bash-only loader, so do not invent a sh requirement for that component.
    if (require_setup and any(p.parent==prefix and _STARTUP_UTIL.fullmatch(p.name) for p in found)
            and prefix/'local_setup.sh' not in found):
        raise ValueError('startup_local_setup_sh_missing:'+str(prefix))
    return found, roots


def capture_startup_closure(prefixes):
    """Static startup inventory; never execute hooks, utilities, or ROS code.

    Both lexical aliases and resolved bytes are bound. Re-enumeration detects
    new registry/hook injection and symlink retargeting, not only old-file edits.
    External base/source-install aliases remain explicit external dependencies.
    """
    for name in _STARTUP_OVERRIDE:
        if os.environ.get(name):
            raise ValueError('unsealed_startup_override:'+name)
    files, roots = set(), []
    normalized = {}
    for name, prefix in sorted(prefixes.items()):
        path = Path(prefix).absolute()
        normalized[name] = str(path)
        # Zenoh is installed as a manual loader prefix, not an independently
        # sourced colcon workspace. Its existing registries are still bound.
        discovered, package_roots = _startup_files(path, require_setup=name!='zenoh_manual_prefix')
        files.update(discovered)
        roots.extend(package_roots)
    # colcon/ament source statements are DSV data, not shell evaluation. A
    # target may be a source-install symlink, but its lexical loader location
    # must be one of the enumerated startup files. No arbitrary external script.
    pending=sorted(files)
    inspected=set()
    external_roots=set()
    while pending:
        path=pending.pop()
        if path in inspected:
            continue
        inspected.add(path)
        if path.suffix != '.dsv':
            continue
        package_roots = [r for r in roots if path.is_relative_to(r)]
        root = max(package_roots, key=lambda p: len(p.parts))
        for line in path.read_text().splitlines():
            if not line.startswith('source;'):
                continue
            relative = Path(line.split(';', 1)[1])
            target=Path(os.path.abspath(relative if relative.is_absolute() else root/relative))
            if target not in files:
                # The actually sourced workspace base is a --symlink-install
                # and its DSV may name ../../build/<pkg>/share/<pkg>/hook.
                # Follow only that exact external develop-hook family and only
                # files reached by a source statement, never the whole build.
                develop_root=(Path(normalized['workspace_base']).parent/'build'
                              if 'workspace_base' in normalized else None)
                develop=None
                if develop_root is not None and target.is_relative_to(develop_root):
                    parts=target.relative_to(develop_root).parts
                    if (len(parts)>=5 and parts[1]=='share' and parts[0]==parts[2]
                            and parts[3] in ('hook','environment')
                            and target.suffix in ('.dsv','.sh','.bash','.ps1','.zsh')):
                        develop=develop_root/parts[0]
                if develop is not None:
                    external_roots.add(develop)
                    if develop not in roots:roots.append(develop)
                    for sibling in (target,target.with_suffix('.dsv')):
                        if sibling.is_file():
                            files.add(sibling);pending.append(sibling)
                # Cross-platform extensions may be absent in a Linux install.
                # Active sh/bash/DSV references must exist, unless colcon uses
                # the existing DSV sibling instead of that shell extension.
                optional = target.suffix in ('.ps1', '.zsh') and not target.exists()
                sibling = target.with_suffix('.dsv') in files
                if target not in files and not optional and not sibling:
                    raise ValueError('startup_dsv_source_not_whitelisted:'+str(target))
    inventory = {}
    for path in sorted(files):
        resolved = path.resolve(strict=True)
        if not resolved.is_file():
            raise ValueError('startup_file_not_regular:'+str(path))
        inventory[str(path)] = {'resolved_path': str(resolved), 'sha256': sha(resolved)}
    return {'schema': 1, 'scope': 'static_explicit_entry_startup_loader_inventory',
            'prefixes': normalized, 'files': inventory,
            'external_develop_hook_roots': sorted(str(p) for p in external_roots),
            'executed_hooks': False, 'portable': False}


def verify_startup_closure(release):
    """Pure-stdlib early check, before the entry executes any setup hook."""
    release=Path(release).resolve(strict=True)
    descriptor_path=release/'release.json'
    descriptor=json.loads(descriptor_path.read_text())
    name=Path(descriptor['sealed_manifest'])
    if name.is_absolute() or len(name.parts)!=1 or name.name in ('', '.', '..'):
        raise ValueError('release_manifest_must_be_inside_release')
    manifest=json.loads((release/name).read_text())
    files=manifest.get('files', {})
    if manifest.get('schema')!=3 or not isinstance(files, dict) or files.get(str(descriptor_path))!=sha(descriptor_path):
        raise ValueError('release_descriptor_not_sealed_or_changed')
    if 'template_source_root' not in descriptor and 'startup_closure' not in manifest:
        return None  # Existing legacy releases keep their published contract.
    if 'startup_closure' not in manifest:
        raise ValueError('candidate_startup_loader_closure_missing')
    nav_root=os.environ.get('D1MAX_NAV_ROOT')
    rmw_prefix=os.environ.get('D1MAX_RMW_PREFIX')
    if not nav_root or not rmw_prefix:
        raise ValueError('startup_closure_requires_explicit_actual_roots')
    frozen=tools_root(release, nav_root)
    for relative in ('single_floor_entry.sh', 'release/single_floor_entry_preflight.py',
                     'validation/bt_localization_replay_entry.sh'):
        path=(frozen/relative).resolve(strict=True)
        if files.get(str(path))!=sha(path):
            raise ValueError('frozen_startup_tool_changed:'+str(path))
    closure=capture_startup_closure(startup_prefixes(release, nav_root, rmw_prefix))
    if closure!=manifest['startup_closure']:
        raise ValueError('startup_loader_closure_changed')
    for value in closure['files'].values():
        if files.get(value['resolved_path'])!=value['sha256']:
            raise ValueError('startup_loader_dependency_not_sealed:'+value['resolved_path'])
    return closure


def verify_localization_dependencies(release, files, get_package_prefix):
    root = localization_dependencies(release)
    if root is None:
        return
    for package in ('faster_lio', 'livox_ros_driver2'):
        selected = Path(get_package_prefix(package)).resolve(strict=True)
        if selected != root/package:
            raise ValueError('unreleased_localization_dependency:'+package)
    for path in (root/'local_setup.bash', root.parent/'runtime_copy_provenance.json'):
        if str(path.resolve(strict=True)) not in files:
            raise ValueError('localization_dependency_component_not_sealed:'+str(path))


def verify(release):
    release = Path(release).resolve(strict=True)
    descriptor_path = release / 'release.json'
    descriptor = json.loads(descriptor_path.read_text())
    name = Path(descriptor['sealed_manifest'])
    if descriptor.get('schema') != 1 or name.is_absolute() or len(name.parts) != 1 or name.name in ('', '.', '..'):
        raise ValueError('release_manifest_must_be_inside_release')
    manifest = json.loads((release / name).read_text())
    files = manifest.get('files')
    if manifest.get('schema') != 3 or not isinstance(files, dict) or not files:
        raise ValueError('release_manifest_schema')
    for path, expected in files.items():
        if not Path(path).is_absolute() or sha(path) != expected:
            raise ValueError('release_changed:' + path)
    if str(descriptor_path) not in files:
        raise ValueError('release_descriptor_not_sealed')
    spec = importlib.util.find_spec('d1max_pct_scan')
    if not spec or not spec.origin or release / 'application/install' not in Path(spec.origin).resolve().parents:
        raise ValueError('unsealed_python_import')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
        raise ValueError('unchanged_zenoh_required')
    if 'template_source_root' in descriptor:
        frozen=tools_root(release)
        for name in ('single_floor_entry.sh','release/single_floor_entry_preflight.py',
                     'validation/bt_localization_replay_entry.sh','validation/run_bt_localization_replay.py'):
            if str((frozen/name).resolve(strict=True)) not in files:
                raise ValueError('actual_frozen_entry_tool_not_sealed:'+name)
    if 'runtime_inputs' in manifest:
        from d1max_pct_planner.runtime_freeze import verify_runtime_inputs,contract_files,scientific_loader_dependencies
        runtime_inputs=verify_runtime_inputs(manifest['runtime_inputs'])
        if any(files.get(p)!=h for p,h in contract_files(runtime_inputs).items()):
            raise ValueError('actual_runtime_input_not_sealed')
        loader=scientific_loader_dependencies(runtime_inputs['scientific'])
        if loader!=manifest.get('scientific_loader_dependencies') or any(files.get(p)!=h for p,h in loader.items()):
            raise ValueError('scientific_actual_loader_dependencies_changed_or_unsealed')
    elif 'template_source_root' in descriptor:
        raise ValueError('candidate_runtime_inputs_contract_missing')
    if 'startup_closure' in manifest:
        verify_startup_closure(release)
    elif 'template_source_root' in descriptor:
        raise ValueError('candidate_startup_loader_closure_missing')
    # dlopen resolves runtime dependencies without creating a ROS context,
    # opening a Zenoh session, invoking rmw_init, or connecting the SDK.
    from ament_index_python.packages import get_package_prefix
    verify_localization_dependencies(release, files, get_package_prefix)
    rmw = Path(get_package_prefix('rmw_zenoh_cpp')).resolve(strict=True) / 'lib/librmw_zenoh_cpp.so'
    if str(rmw.resolve(strict=True)) not in files:
        raise ValueError('zenoh_runtime_not_sealed')
    ctypes.CDLL(str(rmw), mode=os.RTLD_LOCAL | os.RTLD_NOW)
    return dict(release=str(release), entry_module='d1max_pct_scan.single_floor_session',
                verified_files=len(files), rmw='rmw_zenoh_cpp',
                started_processes=False, sdk_connected=False, motion_authorized=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest='action', required=True)
    actions.add_parser('select').add_argument('--nav-root', required=True)
    actions.add_parser('verify').add_argument('--release', required=True)
    actions.add_parser('localization-dependencies').add_argument('--release', required=True)
    actions.add_parser('startup-check').add_argument('--release', required=True)
    frozen=actions.add_parser('tools-root')
    frozen.add_argument('--release', required=True)
    frozen.add_argument('--nav-root')
    args = parser.parse_args()
    try:
        if args.action == 'select':
            print(select(args.nav_root))
        elif args.action == 'verify':
            print(json.dumps(verify(args.release), ensure_ascii=False))
        elif args.action == 'localization-dependencies':
            root = localization_dependencies(args.release)
            print(str(root) if root is not None else '')
        elif args.action == 'startup-check':
            closure=verify_startup_closure(args.release)
            print(json.dumps(dict(verified_startup_files=len(closure['files']) if closure else 0,
                                  executed_hooks=False, started_processes=False)))
        else:
            print(tools_root(args.release,args.nav_root))
    except (OSError, ValueError, KeyError, TypeError, ImportError) as error:
        parser.exit(2, 'single_floor_entry_preflight_failed: ' + str(error) + '\n')


if __name__ == '__main__':
    main()
