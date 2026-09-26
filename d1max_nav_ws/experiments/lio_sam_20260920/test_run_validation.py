import copy
import json
from pathlib import Path
import unittest

import yaml

from run import HERE, validate_full_input


class CompletenessTest(unittest.TestCase):
    def setUp(self):
        baseline = json.loads((HERE / "bag_pair_audit.json").read_text())
        self.bag = Path(baseline["bag"])
        metadata = yaml.safe_load((self.bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
        expected = {row["topic_metadata"]["name"]: row["message_count"] for row in metadata["topics_with_message_count"]}
        pairs = baseline["full_bag"]["paired_scans"]
        self.audit = {"counts": {
            "imu_received": expected["/imu_driver/imu_central"], "imu_published": expected["/imu_driver/imu_central"],
            "front_clouds_received": expected["/front_lidar"], "rear_clouds_received": expected["/rear_lidar"],
            "clouds_published": pairs, "paired_scans": pairs,
            "front_points_published": 1000, "rear_points_published": 1000,
            "front_unpaired": baseline["full_bag"]["counts"]["front_unpaired"],
            "rear_unpaired": baseline["full_bag"]["counts"]["rear_unpaired"]},
            "pending_imu_coverage": 0, "pending_unpaired": {"front": 0, "rear": 0},
            "initialization": {"end_corrected_ns": baseline["init_cutoff_ns"]}}
        self.observed = {"odometry": pairs-3, "deskewed_scans": pairs-3}

    def check(self, audit=None, observed=None):
        return validate_full_input(self.bag, audit or self.audit, observed or self.observed, "dual")["passed"]

    def test_complete_baseline_passes(self):
        self.assertTrue(self.check())

    def test_missing_raw_inputs_rejected(self):
        for key in ("imu_received", "imu_published", "front_clouds_received", "rear_clouds_received"):
            with self.subTest(key=key):
                broken = copy.deepcopy(self.audit)
                broken["counts"][key] -= 1
                self.assertFalse(self.check(broken))

    def test_runtime_pair_loss_rejected(self):
        broken = copy.deepcopy(self.audit)
        broken["counts"]["paired_scans"] -= 1
        broken["counts"]["clouds_published"] -= 1
        self.assertFalse(self.check(broken))

    def test_missing_rear_contribution_rejected(self):
        broken = copy.deepcopy(self.audit)
        broken["counts"]["rear_points_published"] = 0
        self.assertFalse(self.check(broken))

    def test_uncovered_scans_rejected(self):
        broken = copy.deepcopy(self.audit)
        broken["pending_imu_coverage"] = 1
        self.assertFalse(self.check(broken))

    def test_downstream_frame_loss_rejected(self):
        broken = dict(self.observed, odometry=self.observed["odometry"]-10)
        self.assertFalse(self.check(observed=broken))


if __name__ == "__main__":
    unittest.main()
