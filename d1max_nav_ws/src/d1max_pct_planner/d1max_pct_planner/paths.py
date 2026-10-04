"""Single source of filesystem roots; no other module names a machine path.

Every root resolves from an explicit environment variable first, then from
where this code is installed. A root that cannot be derived raises instead of
guessing another checkout or release. Configuration files reference roots as
``${D1MAX_NAV_ROOT}/...`` and are expanded here, so moving a tree only means
changing the environment, never editing code or YAML.
"""
import json
import os
from pathlib import Path
import re

_HERE = Path(__file__).resolve()
# Vendor documentation bundle that still hosts the SDK bridge, Web and Zenoh
# configs. It is only the default until the data migration relocates it.
_VENDOR_BUNDLE = '智元四足机器人D1 Max二次开发文档资料包v0.1.0'
RELEASE_DESCRIPTOR = 'release.json'


def _env(name):
    value = os.environ.get(name, '').strip()
    return Path(value).expanduser().resolve() if value else None


def nav_root():
    """Navigation workspace: D1MAX_NAV_ROOT, else the checkout holding this code."""
    explicit = _env('D1MAX_NAV_ROOT')
    if explicit:
        return explicit
    # Source, develop and sealed-release installs all live below the checkout.
    for parent in _HERE.parents:
        if (parent / 'src/d1max_pct_scan/package.xml').is_file():
            return parent
    raise ValueError('nav_root_unresolved:set_D1MAX_NAV_ROOT')


def vendor_root():
    """Unpacked vendor bundle (SDK, manuals, d1max_ros2 applications)."""
    return _env('D1MAX_VENDOR_ROOT') or Path.home() / _VENDOR_BUNDLE


def app_root():
    """d1max_ros2 application tree: SDK bridge, Web map manager, Zenoh configs."""
    return _env('D1MAX_APP_ROOT') or vendor_root() / 'd1max_ros2'


def vendor_checkout():
    """PCT planner checkout: PCT_PLANNER_ROOT, else the workspace vendor tree."""
    return _env('PCT_PLANNER_ROOT') or nav_root() / 'src/pct_planner_vendor'


def release_root():
    """Sealed release: D1MAX_RELEASE, else the release this code was imported from."""
    explicit = _env('D1MAX_RELEASE')
    if explicit:
        return explicit
    # A sealed release imports its Python from <release>/application/install.
    for parent in _HERE.parents:
        if parent.name == 'install' and parent.parent.name == 'application':
            return parent.parent.parent
    raise ValueError('release_root_unresolved:set_D1MAX_RELEASE')


def inside(root, relative, *, what):
    """A release-relative path that cannot escape its root."""
    relative = Path(relative)
    if relative.is_absolute() or '..' in relative.parts or not relative.parts:
        raise ValueError(what + '_must_be_relative_inside_root')
    return Path(root) / relative


def release_descriptor(release=None):
    """The release's own description (default map etc.), not a sealed manifest."""
    root = Path(release).resolve() if release else release_root()
    value = json.loads((root / RELEASE_DESCRIPTOR).read_text())
    if not isinstance(value, dict) or value.get('schema') != 1:
        raise ValueError('release_descriptor_schema')
    return root, value


def template_root(release=None):
    """Actual session templates, distinct from tools/data workspace roots.

    New releases explicitly bind a real copied source snapshot. Once that
    binding exists, a missing/escaping root is an error, never a fallback to
    the mutable checkout. Older descriptors retain their existing behavior.
    """
    if release is None:
        try:
            release = release_root()
        except ValueError:
            return nav_root()  # source-only development, not a named release
    root, value = release_descriptor(release)
    if 'template_source_root' not in value:
        return nav_root()
    target = inside(root, value['template_source_root'], what='template_source_root').resolve(strict=True)
    if not target.is_relative_to(root) or not target.is_dir():
        raise ValueError('template_source_root_outside_release_or_not_directory')
    return target


def localization_dependencies_root(release=None):
    """Optional, same-byte copied LIO/Livox runtime selected by this release."""
    root, value = release_descriptor(release)
    if 'localization_dependencies_directory' not in value:
        return None
    target = inside(root, value['localization_dependencies_directory'],
                    what='localization_dependencies_directory').resolve(strict=True)
    if not target.is_relative_to(root) or not (target/'local_setup.bash').is_file():
        raise ValueError('localization_dependencies_outside_release_or_setup_missing')
    return target


def tools_root(release=None):
    """Frozen actual entry/preflight/replay tools for new candidate bundles."""
    root,value=release_descriptor(release)
    if 'tools_source_root' not in value:
        return nav_root()/'tools'
    relative=value['tools_source_root']
    target=inside(root,relative,what='tools_source_root')
    cursor=root
    for part in Path(relative).parts:
        cursor=cursor/part
        if cursor.is_symlink():raise ValueError('tools_source_root_must_be_real_copied_directory')
    target=target.resolve(strict=True)
    if not target.is_relative_to(root) or not target.is_dir():
        raise ValueError('tools_source_root_outside_release_or_not_directory')
    return target


def default_map_directory(release=None):
    """Map package the release was sealed with; the caller may name another."""
    root, value = release_descriptor(release)
    return inside(root, value['default_map_directory'], what='release_map')


_ROOTS = {'D1MAX_NAV_ROOT': nav_root, 'D1MAX_APP_ROOT': app_root,
          'D1MAX_VENDOR_ROOT': vendor_root, 'D1MAX_RELEASE': release_root}
_VARIABLE = re.compile(r'\$\{([A-Z0-9_]+)\}')


def expand(value):
    """Expand ${D1MAX_*} roots in one config string; other values pass through."""
    if not isinstance(value, str):
        return value

    def root(match):
        resolver = _ROOTS.get(match.group(1))
        if resolver is None:
            raise ValueError('unknown_path_variable:' + match.group(1))
        return str(resolver())
    return _VARIABLE.sub(root, value)


def expand_tree(value):
    """Expand every string inside a loaded YAML/JSON configuration."""
    if isinstance(value, dict):
        return {key: expand_tree(item) for key, item in value.items()}
    if isinstance(value, list):
        return [expand_tree(item) for item in value]
    return expand(value)
