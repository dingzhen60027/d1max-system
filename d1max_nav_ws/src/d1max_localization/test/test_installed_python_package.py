"""Offline installation smoke test: import every module without starting ROS.

Run after ``colcon build --packages-select d1max_localization`` via CTest/colcon
test, or source the install environment and run this file with pytest. A source
PYTHONPATH cannot hide a missing installed module (including new subpackages).
"""

import json
import os
from pathlib import Path
import subprocess
import sys


SOURCE_PACKAGE = Path(__file__).absolute().parents[1] / "d1max_localization"

IMPORT_INSTALLED = r'''
import importlib
import json
from pathlib import Path
import sys
from unittest.mock import patch

installed = Path(sys.argv[1]).absolute()
expected = set(json.loads(sys.argv[2]))
actual = {str(path.relative_to(installed)) for path in installed.rglob("*.py")}
assert actual == expected, (
    f"installed module inventory differs: missing={sorted(expected - actual)}, "
    f"unexpected={sorted(actual - expected)}; rebuild d1max_localization"
)

# Deliberately ignore all alternative roots, including a source PYTHONPATH or a
# stale underlay. Keep the installed pathname (not its symlink-resolved target).
sys.path = [str(installed.parent)] + [
    entry for entry in sys.path
    if entry and not (Path(entry) / "d1max_localization").exists()
]
import rclpy
from rclpy.node import Node

def forbidden(*args, **kwargs):
    raise AssertionError("installed import attempted to initialize ROS or a node")

loaded = {}
with patch.object(rclpy, "init", forbidden), \
        patch.object(rclpy, "create_node", forbidden), \
        patch.object(rclpy, "spin", forbidden), \
        patch.object(Node, "__init__", forbidden):
    for relative in sorted(expected):
        parts = list(Path(relative).with_suffix("").parts)
        if parts[-1] == "__init__":
            parts.pop()
        name = ".".join(["d1max_localization"] + parts)
        module = importlib.import_module(name)
        origin = Path(module.__file__).absolute()
        assert origin == installed / relative, f"source/underlay import: {name}: {origin}"
        loaded[name] = str(origin)
assert not rclpy.ok(), "import unexpectedly started a ROS context"
print(json.dumps(loaded, sort_keys=True))
'''


def installed_package():
    configured = os.environ.get("D1MAX_LOCALIZATION_INSTALL_DIR")
    if configured:
        result = Path(configured).absolute()
    else:
        from ament_index_python.packages import get_package_prefix
        prefix = Path(get_package_prefix("d1max_localization"))
        candidates = [
            candidate
            for layout in ("dist-packages", "site-packages")
            for candidate in prefix.glob(f"**/{layout}/d1max_localization")
        ]
        assert len(candidates) == 1, f"expected one installed Python package in {prefix}: {candidates}"
        result = candidates[0].absolute()
    assert result != SOURCE_PACKAGE, "this regression must inspect the installed package"
    assert result.is_dir(), f"missing installed Python package: {result}"
    return result


def import_package(package, expected, working_directory):
    env = os.environ.copy()
    # Test that an otherwise convenient source import cannot mask the defect.
    env["PYTHONPATH"] = str(SOURCE_PACKAGE.parent) + os.pathsep + env.get("PYTHONPATH", "")
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    return subprocess.run(
        [sys.executable, "-c", IMPORT_INSTALLED, str(package), json.dumps(expected)],
        cwd=working_directory,
        env=env,
        capture_output=True,
        text=True,
        timeout=20,
        check=False,
    )


def source_inventory():
    return sorted(str(path.relative_to(SOURCE_PACKAGE)) for path in SOURCE_PACKAGE.rglob("*.py"))


def test_every_module_imports_from_installed_tree_without_ros(tmp_path):
    expected = source_inventory()
    result = import_package(installed_package(), expected, tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    loaded = json.loads(result.stdout)
    assert len(loaded) == len(expected)
    assert "d1max_localization.estimation.pose_status" in loaded


def test_missing_installed_submodule_is_not_masked_by_source_pythonpath(tmp_path):
    incomplete = tmp_path / "incomplete" / "d1max_localization"
    expected = source_inventory()
    for relative in expected:
        if relative == "estimation/pose_status.py":
            continue
        target = incomplete / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.symlink_to(SOURCE_PACKAGE / relative)
    result = import_package(incomplete, expected, tmp_path)
    assert result.returncode != 0
    assert "missing=['estimation/pose_status.py']" in result.stderr
