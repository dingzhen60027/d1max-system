"""Exercise a temporary static site without importing the production app."""
import ast
import os
from pathlib import Path
import tempfile
import unittest

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.testclient import TestClient


def static_files_class():
    # Importing backend.app creates production runtime objects. Load only the
    # static adapter so these tests cannot initialize ROS, SDK or live services.
    source = Path(__file__).resolve().parents[1] / "backend" / "app.py"
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    adapter = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                   and node.name == "RevalidatingHtmlStaticFiles")
    namespace = {"StaticFiles": StaticFiles, "Path": Path}
    exec(compile(ast.Module(body=[adapter], type_ignores=[]), str(source), "exec"), namespace)
    return namespace[adapter.name]


class StaticCacheTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="d1max-static-cache-")
        self.addCleanup(self.temporary.cleanup)
        self.dist = Path(self.temporary.name)
        self.index = self.dist / "index.html"
        self.index.write_text('<script src="/assets/index-old123.js"></script>', encoding="utf-8")
        (self.dist / "docs").mkdir()
        (self.dist / "docs" / "index.html").write_text("<h1>Docs</h1>", encoding="utf-8")
        (self.dist / "assets").mkdir()
        (self.dist / "assets" / "index-old123.js").write_text("window.version = 1;", encoding="utf-8")
        (self.dist / "assets" / "index-old123.css").write_text("body { color: black; }", encoding="utf-8")
        app = FastAPI()
        app.mount("/", static_files_class()(directory=self.dist, html=True))
        self.client = TestClient(app)
        self.addCleanup(self.client.close)

    def test_html_get_head_and_directory_index_revalidate(self):
        for path in ("/", "/index.html", "/docs/"):
            for method in ("GET", "HEAD"):
                with self.subTest(path=path, method=method):
                    response = self.client.request(method, path)
                    self.assertEqual(response.status_code, 200)
                    self.assertEqual(response.headers["cache-control"], "no-cache")
                    self.assertIn("etag", response.headers)
                    self.assertIn("last-modified", response.headers)

    def test_html_not_modified_still_revalidates(self):
        for path in ("/", "/index.html", "/docs/"):
            initial = self.client.get(path)
            for validator in ("If-None-Match", "If-Modified-Since"):
                value = initial.headers["etag" if validator == "If-None-Match" else "last-modified"]
                with self.subTest(path=path, validator=validator):
                    response = self.client.get(path, headers={validator: value})
                    self.assertEqual(response.status_code, 304)
                    self.assertEqual(response.headers["cache-control"], "no-cache")
                    self.assertEqual(response.content, b"")

    def test_changed_entrypoint_returns_new_asset_reference(self):
        initial = self.client.get("/")
        previous_mtime = self.index.stat().st_mtime
        self.index.write_text('<script src="/assets/index-new456.js"></script>', encoding="utf-8")
        os.utime(self.index, (previous_mtime + 2, previous_mtime + 2))
        response = self.client.get("/", headers={"If-None-Match": initial.headers["etag"]})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.headers["cache-control"], "no-cache")
        self.assertIn("index-new456.js", response.text)
        self.assertNotEqual(response.headers["etag"], initial.headers["etag"])

    def test_hashed_assets_keep_default_conditional_caching(self):
        for path in ("/assets/index-old123.js", "/assets/index-old123.css"):
            with self.subTest(path=path):
                initial = self.client.get(path)
                self.assertEqual(initial.status_code, 200)
                self.assertNotIn("cache-control", initial.headers)
                response = self.client.get(path, headers={"If-None-Match": initial.headers["etag"]})
                self.assertEqual(response.status_code, 304)
                self.assertNotIn("cache-control", response.headers)
