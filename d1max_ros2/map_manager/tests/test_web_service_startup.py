"""Check Web startup boundaries without importing/starting the production app."""
import ast
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace
from typing import Any


WEB_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = WEB_ROOT.parents[1]
APP_SOURCE = WEB_ROOT / "backend" / "app.py"


class WebStartupTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="d1max-web-startup-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.tree = ast.parse(APP_SOURCE.read_text(encoding="utf-8"))

    def test_http_health_does_not_depend_on_optional_maps(self):
        node = next(node for node in self.tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == "health")
        node.decorator_list = []
        maps = self.root / "missing-nav" / "maps"
        namespace = {"Any": Any, "MAPS_ROOT": maps, "PROJECT_ROOT": self.root,
                     "NAV_ROOT": maps.parent, "PROCESSED_ROOT": maps / "processed"}
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(APP_SOURCE), "exec"), namespace)
        result = namespace["health"]()
        self.assertTrue(result["ok"])
        self.assertFalse(result["maps_available"])
        maps.mkdir(parents=True)
        result = namespace["health"]()
        self.assertTrue(result["ok"])
        self.assertTrue(result["maps_available"])

    def test_web_launcher_accepts_custom_nav_without_map_library(self):
        # Execute the real shell entry but substitute only the HTTP interpreter.
        # No ROS, SDK, uvicorn or managed unit is initialized in this test.
        interpreter = self.root / "inspect-python"
        interpreter.write_text(
            f"#!{sys.executable}\nimport json, os, sys\n"
            "print(json.dumps({'args':sys.argv[1:], 'nav':os.environ['D1MAX_NAV_ROOT'], "
            "'maps':os.environ['D1MAX_MAPS_ROOT']}))\n", encoding="utf-8")
        interpreter.chmod(0o700)
        nav = self.root / "custom nav"
        for maps_override in (None, str(self.root / "external missing maps")):
            with self.subTest(maps_override=maps_override):
                env = dict(os.environ, D1MAX_NAV_ROOT=str(nav),
                           D1MAX_MAP_MANAGER_PYTHON=str(interpreter))
                env.pop("D1MAX_MAPS_ROOT", None)
                if maps_override is not None:
                    env["D1MAX_MAPS_ROOT"] = maps_override
                result = subprocess.run(
                    ["bash", str(PROJECT_ROOT / "start_d1max_map_manager.sh")],
                    env=env, capture_output=True, text=True, timeout=3, check=True)
                report = json.loads(result.stdout.splitlines()[-1])
                self.assertEqual(report["nav"], str(nav))
                self.assertEqual(report["maps"], maps_override or str(nav / "maps"))
                self.assertEqual(report["args"][:3], ["-m", "uvicorn", "backend.app:app"])
                self.assertFalse(nav.exists())

    def test_all_workflows_respect_formal_navigation_ownership(self):
        # Evaluate the production exclusion expressions, rather than importing
        # the app (whose constructors bind production runtime directories).
        def runtime(busy=False):
            return SimpleNamespace(snapshot=lambda: {"busy": busy, "status": "stopped"})

        namespace = {
            "processing_job": {}, "localization_runtime": SimpleNamespace(pinned_id=None),
            "navigation_runtime": runtime(), "navigation_session_runtime": runtime(True),
            "runtime_manager": runtime(), "MAPPING_BUSY": {"starting", "running", "stopping"},
        }
        conditions = {}
        for function_name in ("start_processing", "start_runtime"):
            function = next(node for node in self.tree.body
                            if isinstance(node, ast.FunctionDef) and node.name == function_name)
            condition = next(node.test for node in ast.walk(function)
                             if isinstance(node, ast.If)
                             and "navigation_session_runtime" in ast.unparse(node.test))
            conditions[function_name] = condition
        for call in ast.walk(self.tree):
            if (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                    and call.func.id in {"create_router", "create_bag_router"}):
                conditions[call.func.id] = next(arg.body for arg in call.args
                                               if isinstance(arg, ast.Lambda))
        self.assertEqual(set(conditions), {"start_processing", "start_runtime", "create_router", "create_bag_router"})
        for name, condition in conditions.items():
            with self.subTest(workflow=name):
                code = compile(ast.Expression(body=condition), str(APP_SOURCE), "eval")
                namespace["navigation_session_runtime"] = runtime(True)
                self.assertTrue(eval(code, namespace))
                namespace["navigation_session_runtime"] = runtime(False)
                self.assertFalse(eval(code, namespace))
