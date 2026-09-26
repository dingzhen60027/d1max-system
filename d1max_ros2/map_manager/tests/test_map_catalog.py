"""Catalog regressions with all backend state isolated before its first import."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


APP_ROOT = Path(__file__).resolve().parents[1]
SCAN_SCRIPT = """
import json
import backend.app as app

files = sorted(str(path.relative_to(app.MAPS_ROOT)) for path in app.iter_cloud_files())
items, _ = app.scan_maps()
print(json.dumps({
    "files": files,
    "catalog": sorted(item["relative_path"] for item in items),
    "roles": {item["relative_path"]: item["role"] for item in items},
}))
"""


class MapCatalogTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="d1max-map-catalog-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.maps = self.root / "maps"
        self.maps.mkdir()

    def cloud(self, relative):
        path = self.maps / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
            "COUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n",
            encoding="ascii",
        )
        return path

    def assert_catalog(self, expected):
        # Importing backend.app creates directories and runtime holders. Perform
        # the import only in a fresh child with every state/data root redirected.
        # No FastAPI lifespan, ROS node, or runtime startup is invoked.
        env = os.environ.copy()
        env.update({
            "D1MAX_PROJECT_ROOT": str(self.root / "project"),
            "D1MAX_NAV_ROOT": str(self.root / "nav"),
            "D1MAX_MAPS_ROOT": str(self.maps),
            "D1MAX_MAP_MANAGER_DATA": str(self.root / "data"),
            "D1MAX_PROCESSED_ROOT": str(self.root / "processed"),
            "D1MAX_BAG_DIR": str(self.root / "bags"),
            "D1MAX_BAG_LIBRARY_ROOTS": str(self.root / "bags"),
            "PYTHONDONTWRITEBYTECODE": "1",
        })
        process = subprocess.run(
            [sys.executable, "-B", "-c", SCAN_SCRIPT],
            cwd=APP_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=30,
        )
        self.assertEqual(process.returncode, 0, process.stderr)
        result = json.loads(process.stdout)
        expected = sorted(expected)
        self.assertEqual(result["files"], expected)
        self.assertEqual(result["catalog"], expected)
        return result

    def lio_manifest(self, run, mode="dual"):
        directory = self.maps / "lio_sam" / run
        (directory / "lio_sam_manifest.json").write_text(json.dumps({
            "schema": "d1max.lio_sam_artifact.v1", "status": "complete",
            "output_file": "GlobalMap.pcd", "sensor_mode": mode,
            "size_bytes": (directory / "GlobalMap.pcd").stat().st_size,
        }))

    def test_lio_sam_only_completed_global_maps_are_visible(self):
        expected = []
        for run, mode in (("20260920_170433_front", "front"), ("20260920_180000_dual", "dual")):
            path = f"lio_sam/{run}/GlobalMap.pcd"
            expected.append(path)
            self.cloud(path)
            self.lio_manifest(run, mode)
            for internal in ("CornerMap.pcd", "SurfMap.pcd", "trajectory.pcd", "transformations.pcd", "Scans/000.pcd"):
                self.cloud(f"lio_sam/{run}/{internal}")
        self.cloud("lio_sam/incomplete/GlobalMap.pcd")
        self.cloud("lio_sam/incomplete/trajectory.pcd")
        self.cloud("lio_sam/loose-map.pcd")
        result = self.assert_catalog(expected)
        self.assertEqual(result["roles"][expected[0]], "lio_sam_front")
        self.assertEqual(result["roles"][expected[1]], "lio_sam_dual")

    def test_lio_sam_manifest_cannot_add_arbitrary_files_or_symlinks(self):
        self.cloud("lio_sam/bad/GlobalMap.pcd")
        self.lio_manifest("bad")
        manifest = self.maps / "lio_sam/bad/lio_sam_manifest.json"
        value = json.loads(manifest.read_text())
        value["output_file"] = "trajectory.pcd"
        manifest.write_text(json.dumps(value))
        self.cloud("lio_sam/link/GlobalMap.pcd")
        self.lio_manifest("link")
        path = self.maps / "lio_sam/link/GlobalMap.pcd"
        outside = self.root / "outside.pcd"
        path.rename(outside)
        path.symlink_to(outside)
        self.assert_catalog([])

    def test_pgo_keyframe_directories_are_excluded_case_insensitively(self):
        for parent, scans in (
            ("sc_pgo", "Scans"),
            ("sc_pgo_final", "Scans"),
            ("SC_PGO", "sCaNs"),
            ("Sc_PgO_FINAL", "SCANS"),
        ):
            self.cloud(f"runs/20260917_190115_test/{parent}/{scans}/000000.pcd")
            self.cloud(f"runs/20260917_190115_test/{parent}/{scans}/nested/000001.pcd")
        self.assert_catalog([])

    def test_complete_frontend_and_optimized_maps_remain_visible(self):
        expected = [
            "runs/20260917_190115_test/d1max_map_20260917_192001.pcd",
            "runs/20260917_190115_test/sc_pgo/optimized_map.pcd",
            "runs/20260917_190115_test/sc_pgo_final/optimized_map.pcd",
        ]
        for relative in expected:
            self.cloud(relative)
        self.cloud("runs/20260917_190115_test/sc_pgo/Scans/000000.pcd")
        self.cloud("runs/20260917_190115_test/sc_pgo_final/Scans/000001.pcd")
        self.assert_catalog(expected)

    def test_user_scans_directories_and_unrelated_prefixes_are_preserved(self):
        expected = [
            "imports/Scans/user-map.pcd",
            "runs/user-run/Scans/manual-map.pcd",
            "runs/user-run/sc_pgold/Scans/user-map.pcd",
            "runs/user-run/sc_pgo/Scans.pcd",
        ]
        for relative in expected:
            self.cloud(relative)
        self.assert_catalog(expected)

    def test_run_body_samples_are_excluded_recursively_case_insensitively(self):
        expected = []
        for run, directory in (
            ("20260917_190115_test", "body_samples"),
            ("arbitrary-user-run", "BODY_SAMPLES"),
            ("another-run", "BoDy_SaMpLeS"),
        ):
            complete_map = f"runs/{run}/d1max_map_20260917_192001.pcd"
            self.cloud(complete_map)
            expected.append(complete_map)
            self.cloud(f"runs/{run}/{directory}/000000.pcd")
            self.cloud(f"runs/{run}/{directory}/nested/deeper/000001.pcd")
        self.assert_catalog(expected)

    def test_user_body_samples_and_same_named_files_are_preserved(self):
        expected = [
            "imports/body_samples/user-map.pcd",
            "body_samples/root-user-map.pcd",
            "runs/user-run/body_samples.pcd",
            "runs/user-run/notes/body_samples/user-map.pcd",
            "runs/body_samples/d1max_map_20260917_192001.pcd",
        ]
        for relative in expected:
            self.cloud(relative)
        self.assert_catalog(expected)

    def test_directory_and_file_symlinks_are_not_cataloged(self):
        self.cloud("kept-map.pcd")
        outside = self.root / "outside-maps"
        outside.mkdir()
        (outside / "outside.pcd").write_bytes(b"must not be scanned")
        (self.maps / "linked-directory").symlink_to(outside, target_is_directory=True)
        (self.maps / "linked-file.pcd").symlink_to(outside / "outside.pcd")
        self.assert_catalog(["kept-map.pcd"])

    def test_workspace_and_cache_exclusions_are_preserved(self):
        self.cloud("kept-map.pcd")
        for relative in (
            "workspace/internal.pcd",
            ".cache/preview.pcd",
            "runs/test/workspace/nested/internal.pcd",
            "runs/test/.cache/nested/preview.pcd",
        ):
            self.cloud(relative)
        self.assert_catalog(["kept-map.pcd"])


if __name__ == "__main__":
    unittest.main()
