import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from backend.mapping.lio_sam_artifacts import completed_artifacts, publish_run


class LioSamArtifactsTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="d1max-lio-artifacts-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.maps = self.root / "maps"
        self.maps.mkdir()
        self.run = self.root / "20260920_170433_trial"
        (self.run / "map").mkdir(parents=True)
        self.pcd = self.run / "map/GlobalMap.pcd"
        self.pcd.write_bytes(b"VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0 1\n")
        self.manifest = {"completion": {"status": "completed full bag", "map": str(self.pcd)},
                         "input": "single front AIRY96 + central IMU", "loop_enabled": True}
        self.save_manifest()

    def save_manifest(self):
        (self.run / "manifest.json").write_text(json.dumps(self.manifest))

    def test_copy_preserves_source_and_refuses_overwrite(self):
        original = self.pcd.read_bytes()
        result = publish_run(self.run, self.maps, "front")
        self.assertEqual(self.pcd.read_bytes(), original)
        self.assertEqual(Path(result["path"]).read_bytes(), original)
        self.assertEqual(result["sha256"], hashlib.sha256(original).hexdigest())
        self.assertEqual(len(list(completed_artifacts(self.maps / "lio_sam"))), 1)
        self.assertFalse((self.maps / "state.json").exists())
        with self.assertRaises(FileExistsError):
            publish_run(self.run, self.maps, "front")

    def test_sensor_mode_must_agree_with_recorded_input(self):
        with self.assertRaisesRegex(ValueError, "sensor_mode disagrees"):
            publish_run(self.run, self.maps, "dual")
        self.manifest["input"] = "dual front + rear AIRY96 + central IMU"
        self.save_manifest()
        result = publish_run(self.run, self.maps, "dual")
        self.assertEqual(result["sensor_mode"], "dual")

    def test_failed_or_running_runs_are_not_importable(self):
        self.manifest["completion"]["status"] = "failed"
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "completed"):
            publish_run(self.run, self.maps, "front")
        self.assertFalse((self.maps / "lio_sam").exists())

    def test_wrong_completion_path_is_rejected(self):
        self.manifest["completion"]["map"] = str(self.run / "map/trajectory.pcd")
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, "Completion map"):
            publish_run(self.run, self.maps, "front")


if __name__ == "__main__":
    unittest.main()
