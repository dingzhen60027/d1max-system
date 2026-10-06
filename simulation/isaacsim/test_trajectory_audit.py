"""Independent collision geometry negatives; no simulator or controller."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import trajectory_audit as audit
import truth_map

HERE = Path(__file__).resolve().parent
SPEC_PATH = HERE / "assets/scene_config.json"
URDF_PATH = HERE / "assets/wheel_fixture.urdf"
SPEC = json.loads(SPEC_PATH.read_text())
MODEL = audit.fixture_model(SPEC, URDF_PATH)
BOXES = truth_map.geometry_from_spec(SPEC)["boxes"]
NORMAL = [-4., -3., .35, 0., 0., 0., 1.]


def penetrations(pose, boxes=BOXES, model=MODEL):
    return [r for r in audit.evaluate_pose(pose, model, boxes) if r["classification"] == "penetration"]


class TrajectoryGeometryTests(unittest.TestCase):
    def test_normal_floor_contact_and_turn_have_no_static_overlap(self):
        for angle in (0, .25, math.pi / 2, math.pi):
            pose = NORMAL[:3] + [0., 0., math.sin(angle / 2), math.cos(angle / 2)]
            self.assertFalse(penetrations(pose))
        # Two casters and the wheel cylinder bottoms legitimately reach floor0.
        caster = next(s for s in MODEL if s["name"] == "front_caster_link")
        self.assertAlmostEqual(NORMAL[2] + caster["center"][2] - caster["radius"], 0.)

    def test_real_wall_penetration_is_detected_in_chassis(self):
        pose = [-5.75, -3., .35, 0., 0., 0., 1.]
        hits = penetrations(pose)
        self.assertTrue(any(r["solid"] == "chassis" and r["static_collider"].endswith("west_wall") for r in hits))
        self.assertFalse(penetrations([-5.55, -3., .35, 0., 0., 0., 1.]))

    def test_thin_low_obstacle_on_wheel_only_path_is_not_missed(self):
        thin = dict(path="thin_wheel_obstacle", min=[-3.805, -2.75, .19], max=[-3.795, -2.71, .21])
        hits = penetrations(NORMAL, [thin])
        self.assertTrue(any(r["solid"] == "left_wheel_link" for r in hits))
        self.assertTrue(all(r["solid_model"] == "wheel_obb_enclosure" for r in hits))

    def test_enclosing_wheel_box_false_positive_is_explicit(self):
        # Local x=z_radial=.24 is in the enclosure square, outside R=.26 circle.
        corner = dict(path="square_corner_only", min=[-3.764, -2.734, .496], max=[-3.756, -2.726, .504])
        self.assertGreater(math.hypot(.236, .236), SPEC["robot"]["wheel_radius"])
        hits = penetrations(NORMAL, [corner])
        self.assertTrue(hits)
        self.assertTrue(all(r["solid_model"] == "wheel_obb_enclosure" for r in hits))
        wheel = next(s for s in MODEL if s["name"] == "left_wheel_link")
        self.assertAlmostEqual(wheel["maximum_enclosure_excess_distance_m"], (math.sqrt(2) - 1) * .26)

    def test_caster_sphere_obstacle_penetration_is_exact(self):
        small = dict(path="caster_obstacle", min=[-3.655, -3.005, .045], max=[-3.645, -2.995, .055])
        hits = penetrations(NORMAL, [small])
        self.assertTrue(any(r["solid"] == "front_caster_link" and r["solid_model"] == "sphere_exact" for r in hits))

    def test_all_measured_rotation_components_change_collision_geometry(self):
        chassis = [s for s in MODEL if s["name"] == "chassis"]
        side = dict(path="yaw_sensitive", min=[-3.68, -3.01, .30], max=[-3.66, -2.99, .40])
        self.assertTrue(penetrations(NORMAL, [side], chassis))
        yaw90 = NORMAL[:3] + [0., 0., math.sqrt(.5), math.sqrt(.5)]
        self.assertFalse(penetrations(yaw90, [side], chassis))
        top = dict(path="roll_sensitive", min=[-4.01, -3.01, .51], max=[-3.99, -2.99, .53])
        self.assertFalse(penetrations(NORMAL, [top], chassis))
        roll90 = NORMAL[:3] + [math.sqrt(.5), 0., 0., math.sqrt(.5)]
        self.assertTrue(penetrations(roll90, [top], chassis))

    def test_quaternion_sign_and_normalization_preserve_orientation(self):
        a = audit.rotation_xyzw([0., 0., .6, .8])
        b = audit.rotation_xyzw([0., 0., -.6, -.8])
        self.assertEqual(a, b)
        for q in ([0., 0., 0., 0.], [math.nan, 0., 0., 1.]):
            with self.assertRaises(ValueError):
                audit.rotation_xyzw(q)

    def test_distance_semantics_and_small_penetration(self):
        identity = audit.rotation_xyzw([0, 0, 0, 1])
        self.assertAlmostEqual(audit.obb_aabb_signed_axis_gap([0, 0, 0], identity, [1, 1, 1], [1.1, -1, -1], [2.1, 1, 1]), .1)
        self.assertAlmostEqual(audit.obb_aabb_signed_axis_gap([0, 0, 0], identity, [1, 1, 1], [.999, -1, -1], [2, 1, 1]), -.001)
        # Sphere fully contained in a box requires exit distance plus radius.
        self.assertAlmostEqual(audit.sphere_aabb_signed_clearance([0, 0, 0], .1, [-1, -1, -1], [1, 1, 1]), -1.1)

    def test_fixture_spec_mismatch_fails_before_audit(self):
        changed = copy.deepcopy(SPEC)
        changed["robot"]["wheel_radius"] = .25
        with self.assertRaisesRegex(ValueError, "fixture_spec_urdf_geometry_mismatch"):
            audit.fixture_model(changed, URDF_PATH)


class TrajectoryFileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)
        self.source = self.folder / "source.usda"
        self.source.write_text("pure-file unit source; USD audit tested in truth_map tests")
        self.prior = self.folder / "prior.json"
        with patch.object(truth_map, "verify_stage", return_value={}):
            truth_map.build(SPEC_PATH, self.source, self.prior)
        self.trajectory = self.folder / "trajectory.jsonl"
        self.output = self.folder / "audit.json"

    def tearDown(self):
        self.temp.cleanup()

    def records(self, poses):
        self.trajectory.write_text("".join(json.dumps(dict(sim_time_ns=(i + 1) * 16666667, pose=p, command=[99, 99])) + "\n" for i, p in enumerate(poses)))

    def run_audit(self):
        return audit.audit(self.trajectory, SPEC_PATH, self.prior, self.output)

    def test_independent_report_ignores_command_and_does_not_evaluate_task(self):
        self.records([NORMAL, NORMAL[:3] + [0., 0., math.sqrt(.5), math.sqrt(.5)]])
        result = self.run_audit()
        self.assertEqual(result["sample_count"], 2)
        self.assertTrue(result["sampled_chassis_and_casters_clear"])
        self.assertTrue(result["sampled_wheel_enclosures_clear"])
        self.assertFalse(result["navigation_success_evaluated"])
        self.assertFalse(result["movement_authority"])
        self.assertTrue(result["floor_plane_contact_allowed"])
        self.assertGreater(result["minimum_signed_gaps"]["exact"]["signed_gap_m"], 0.)

    def test_real_chassis_wall_overlap_report_does_not_pass_sampled_geometry(self):
        self.records([NORMAL, [-5.75, -3, .35, 0, 0, 0, 1]])
        result = self.run_audit()
        self.assertEqual(result["sampled_exact_penetration_samples"], 1)
        self.assertFalse(result["sampled_chassis_and_casters_clear"])
        self.assertLess(result["minimum_signed_gaps"]["exact"]["signed_gap_m"], 0.)

    def test_nonmonotonic_source_and_wrong_artifact_hash_rejected(self):
        row = json.dumps(dict(sim_time_ns=1, pose=NORMAL))
        self.trajectory.write_text(row + "\n" + row + "\n")
        with self.assertRaisesRegex(ValueError, "nonmonotonic_actual_trajectory_source_time"):
            self.run_audit()
        self.records([NORMAL])
        changed = self.folder / "changed.urdf"
        changed.write_text(URDF_PATH.read_text() + "\n")
        with self.assertRaisesRegex(ValueError, "trajectory_audit_urdf_hash_mismatch"):
            audit.audit(self.trajectory, SPEC_PATH, self.prior, self.output, changed)


if __name__ == "__main__":
    unittest.main()
