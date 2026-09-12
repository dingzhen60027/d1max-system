"""Use temporary manifests and mock runtime; never contact the robot."""
import asyncio
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
import backend.app as app

class WebLifecycleTests(unittest.TestCase):
    def test_shutdown_cancels_worker_closes_runtime_and_recovers_only_incomplete(self):
        with tempfile.TemporaryDirectory(prefix="d1max-web-lifecycle-") as temporary:
            root = Path(temporary)
            for name, status in (("unfinished","running"), ("good","complete")):
                (root / name).mkdir()
                (root / name / "manifest.json").write_text(json.dumps({"status":status,"source_path":"raw.pcd"}))
            outside = root / "outside.json"
            outside.write_text('{"status":"running"}')
            (root / "linked").mkdir()
            (root / "linked/manifest.json").symlink_to(outside)
            original_good = (root / "good/manifest.json").read_bytes()
            runtime = Mock()
            cancel = threading.Event()
            worker = threading.Thread(target=lambda: cancel.wait(2), daemon=True)
            worker.start()
            async def lifecycle():
                async with app.lifespan(app.app):
                    self.assertFalse(cancel.is_set())
                    value=json.loads((root / "unfinished/manifest.json").read_text())
                    self.assertEqual(value["status"],"interrupted")
            with patch.object(app,"PROCESSED_ROOT",root), patch.object(app,"runtime_manager",runtime), patch.object(app,"grid_workspace",Mock()), patch.object(app,"localization_runtime",Mock()), patch.object(app,"processing_cancel_event",cancel), patch.object(app,"processing_thread",worker):
                asyncio.run(lifecycle())
            self.assertTrue(cancel.is_set())
            self.assertFalse(worker.is_alive())
            runtime.close.assert_called_once()
            runtime.recover_stale_state.assert_called_once()
            self.assertEqual((root / "good/manifest.json").read_bytes(),original_good)
            self.assertEqual(json.loads(outside.read_text())["status"],"running")
