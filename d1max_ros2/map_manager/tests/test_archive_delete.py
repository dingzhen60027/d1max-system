import json
import tempfile
import unittest
from pathlib import Path

from fastapi import HTTPException

import backend.app as backend_app


class ArchiveDeleteTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="d1max-archive-delete-")
        self.root = Path(self.temporary.name)
        self.originals = {
            name: getattr(backend_app, name)
            for name in ("MAPS_ROOT", "PROCESSED_ROOT", "CACHE_ROOT", "STATE_PATH", "scan_maps")
        }
        backend_app.MAPS_ROOT = self.root / "maps"
        backend_app.PROCESSED_ROOT = self.root / "processed"
        backend_app.CACHE_ROOT = self.root / "cache"
        backend_app.STATE_PATH = self.root / "state.json"
        backend_app.MAPS_ROOT.mkdir()
        backend_app.PROCESSED_ROOT.mkdir()
        backend_app.CACHE_ROOT.mkdir()

    def tearDown(self):
        for name, value in self.originals.items():
            setattr(backend_app, name, value)
        self.temporary.cleanup()

    def write_state(self, items):
        backend_app.STATE_PATH.write_text(
            json.dumps({"active_id": None, "items": items}),
            encoding="utf-8",
        )

    def test_selected_delete_removes_files_directories_caches_and_state(self):
        raw_id = "1111111111111111"
        processed_id = "2222222222222222"
        raw_path = backend_app.MAPS_ROOT / "kept-run" / "map.pcd"
        raw_path.parent.mkdir()
        raw_path.write_bytes(b"raw-map")
        log_path = raw_path.parent / "mapping.log"
        log_path.write_text("keep this run log", encoding="utf-8")

        result_dir = backend_app.PROCESSED_ROOT / "result-run"
        result_dir.mkdir()
        processed_path = result_dir / "processed_map.pcd"
        processed_path.write_bytes(b"processed-map")
        (result_dir / "manifest.json").write_text('{"status":"complete"}', encoding="utf-8")
        (result_dir / "pipeline.yaml").write_text("modules: []\n", encoding="utf-8")

        raw_cache = backend_app.CACHE_ROOT / f"{raw_id}-preview.ply"
        processed_cache = backend_app.CACHE_ROOT / f"{processed_id}-preview.ply"
        raw_cache.write_bytes(b"raw-preview")
        processed_cache.write_bytes(b"processed-preview")
        untouched_cache = backend_app.CACHE_ROOT / "ffffffffffffffff-preview.ply"
        untouched_cache.write_bytes(b"keep")

        items = [
            {
                "id": raw_id, "name": "归档原图", "path": str(raw_path),
                "category": "maps", "archived": True, "run_id": "kept-run",
            },
            {
                "id": processed_id, "name": "归档处理结果", "path": str(processed_path),
                "category": "processed", "archived": True, "run_id": "result-run",
            },
        ]
        backend_app.scan_maps = lambda: (items, [])
        self.write_state({raw_id: {"archived": True}, processed_id: {"archived": True}})

        response = backend_app.delete_archived(
            backend_app.ArchiveDeleteRequest(item_ids=[raw_id, processed_id])
        )

        self.assertEqual(response["deleted_count"], 2)
        self.assertGreater(response["freed_bytes"], 0)
        self.assertFalse(raw_path.exists())
        self.assertTrue(log_path.exists(), "ordinary run siblings must not be removed")
        self.assertFalse(result_dir.exists(), "processed result owns its complete task directory")
        self.assertFalse(raw_cache.exists())
        self.assertFalse(processed_cache.exists())
        self.assertTrue(untouched_cache.exists())
        state = json.loads(backend_app.STATE_PATH.read_text(encoding="utf-8"))
        self.assertEqual(state["items"], {})

    def test_delete_all_removes_every_archived_item(self):
        item_ids = ("3333333333333333", "4444444444444444")
        items = []
        for index, item_id in enumerate(item_ids):
            path = backend_app.MAPS_ROOT / f"map-{index}.pcd"
            path.write_bytes(b"point-cloud")
            items.append({
                "id": item_id, "name": f"归档 {index}", "path": str(path),
                "category": "maps", "archived": True, "run_id": f"run-{index}",
            })
        backend_app.scan_maps = lambda: (items, [])
        self.write_state({
            item_ids[0]: {"archived": True},
            item_ids[1]: {"archived": True},
            "5555555555555555": {"archived": True},  # stale metadata is cleaned too
        })

        response = backend_app.delete_archived(
            backend_app.ArchiveDeleteRequest(delete_all=True)
        )

        self.assertEqual(response["deleted_count"], 2)
        self.assertTrue(all(not Path(item["path"]).exists() for item in items))
        state = json.loads(backend_app.STATE_PATH.read_text(encoding="utf-8"))
        self.assertEqual(state["items"], {})

    def test_nonarchived_item_is_never_deleted(self):
        item_id = "6666666666666666"
        path = backend_app.MAPS_ROOT / "current-map.pcd"
        path.write_bytes(b"must-stay")
        item = {
            "id": item_id, "name": "未归档地图", "path": str(path),
            "category": "maps", "archived": False, "run_id": "current",
        }
        backend_app.scan_maps = lambda: ([item], [])
        self.write_state({})

        with self.assertRaises(HTTPException) as raised:
            backend_app.delete_archived(
                backend_app.ArchiveDeleteRequest(item_ids=[item_id])
            )

        self.assertEqual(raised.exception.status_code, 409)
        self.assertTrue(path.exists())


if __name__ == "__main__":
    unittest.main()
