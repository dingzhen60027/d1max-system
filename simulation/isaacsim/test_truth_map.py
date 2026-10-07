"""Full-voxel geometry, source attestation, and file integrity regressions."""
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import truth_map as truth

HERE = Path(__file__).resolve().parent
SPEC_PATH = HERE / "assets/scene_config.json"
USD_PATH = HERE / "assets/indoor_scene.usda"
SPEC = json.loads(SPEC_PATH.read_text())
HAS_USD = importlib.util.find_spec("pxr") is not None


def support_spec(endpoint_error_bound=None):
    spec = copy.deepcopy(SPEC)
    spec["dynamic_actors"] = []
    spec["flat_support_contact"] = dict(enabled=True, floor_path="/World/GroundPlane/collisionPlane",
                                        floor_z=0., penetration_allowance_m=.02)
    if endpoint_error_bound is not None:
        spec["flat_support_contact"]["floor_endpoint_error_bound_m"] = endpoint_error_bound
    return spec


class FullCellGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = truth.geometry_from_spec(SPEC)
        cls.data, cls.origin, cls.shape = truth.rasterize(cls.geometry)

    def status(self, index):
        relative = [i - o for i, o in zip(index, self.origin)]
        if any(i < 0 or i >= n for i, n in zip(relative, self.shape)):
            return truth.UNKNOWN
        return self.data[(relative[0] * self.shape[1] + relative[1]) * self.shape[2] + relative[2]]

    def test_empty_complete_startup_body_volume_is_environment_free(self):
        # These are certified *environment* cells, independent of robot pose or
        # ownership. They include the previously unobserved lidar blind zone.
        for x in range(-88, -71):
            for y in range(-67, -52):
                for z in range(2, 13):
                    self.assertEqual(self.status([x, y, z]), truth.FREE)

    def test_support_contact_is_distinct_and_never_erases_low_obstacle(self):
        from test_dynamic_collision import ACTOR
        spec = copy.deepcopy(SPEC)
        spec["dynamic_actors"] = [ACTOR]
        spec["flat_support_contact"] = dict(enabled=True, floor_path="/World/GroundPlane/collisionPlane", floor_z=0., penetration_allowance_m=.02)
        geometry = truth.geometry_from_spec(spec)
        data, origin, shape = truth.rasterize(geometry)
        def status(cell):
            a, b, c = [i - o for i, o in zip(cell, origin)]
            return data[(a * shape[1] + b) * shape[2] + c]
        self.assertEqual(status([-80, -60, -1]), truth.SUPPORT_CONTACT)
        self.assertEqual(status([-80, -60, 0]), truth.SUPPORT_CONTACT)
        self.assertEqual(status([-76, 64, 0]), truth.OCCUPIED)
        self.assertEqual(status([-120, -60, 0]), truth.OCCUPIED)
        self.assertEqual(status([-80, -60, 1]), truth.FREE)

    def test_floor_endpoint_error_bound_is_optional_hash_bound_and_never_changes_voxels(self):
        legacy = truth.geometry_from_spec(support_spec())
        widened = truth.geometry_from_spec(support_spec(.0002))
        self.assertNotIn("floor_endpoint_error_bound_m", legacy["flat_support_contact"])
        self.assertEqual(truth._floor_endpoint_error_bound(legacy["flat_support_contact"]), 1e-5)
        self.assertEqual(widened["flat_support_contact"]["floor_endpoint_error_bound_m"], .0002)
        self.assertNotEqual(hashlib.sha256(truth._canonical(legacy)).hexdigest(),
                            hashlib.sha256(truth._canonical(widened)).hexdigest())
        self.assertEqual(truth.rasterize(legacy), truth.rasterize(widened))
        self.assertEqual(widened["floor"], legacy["floor"])
        self.assertEqual(widened["boxes"], legacy["boxes"])

    def test_floor_endpoint_error_bound_rejects_invalid_values_or_disabled_contact(self):
        for value in (0., -1., math.nan, math.inf, True, ".0002", 9e-6, .001000001):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "floor_endpoint_error_bound"):
                    truth.geometry_from_spec(support_spec(value))
        disabled = support_spec(.0002);disabled["flat_support_contact"]["enabled"] = False
        with self.assertRaisesRegex(ValueError, "invalid_flat_support_contact_contract"):
            truth.geometry_from_spec(disabled)
        self.assertEqual(truth.geometry_from_spec(support_spec(.001))["flat_support_contact"]["floor_endpoint_error_bound_m"], .001)

    def test_floor_endpoint_error_bound_resolution_cap_is_checked_before_allocation(self):
        for value, resolution in ((.000500001, .01), (.0002, .001), (None, .0001)):
            geometry = truth.geometry_from_spec(support_spec(value))
            with patch.object(truth, "bytearray", side_effect=RuntimeError("allocation_reached"), create=True):
                with self.assertRaisesRegex(ValueError, "floor_endpoint_error_bound_out_of_range"):
                    truth.rasterize(geometry, resolution=resolution)
        geometry = truth.geometry_from_spec(support_spec(.0005))
        geometry["boxes"] = []
        geometry["closed_world_bounds"] = dict(min=[-1., -1., 0.], max=[1., 1., 1.])
        with patch.object(truth, "bytearray", side_effect=RuntimeError("allocation_reached"), create=True):
            with self.assertRaisesRegex(RuntimeError, "allocation_reached"):
                truth.rasterize(geometry, resolution=.01)

    def test_resource_bound_matches_cpp_256mib_and_rejects_indices_before_allocation(self):
        geometry = dict(boxes=[], floor=dict(z=0.), closed_world_bounds=dict(min=[-50., -40., 0.], max=[50., 40., 3.4]))
        self.assertEqual(truth.MAX_VOXELS, 256 * 1024 * 1024)
        # This large legitimate volume exceeds the old 20M cap. Intercept the
        # actual allocation so a resource-bound regression needs no huge RAM.
        with patch.object(truth, "bytearray", side_effect=RuntimeError("allocation_reached"), create=True):
            with self.assertRaisesRegex(RuntimeError, "allocation_reached"):
                truth.rasterize(geometry)
        for upper in ([100., 100., 4.], [1e10, 1., 1.]):
            invalid = copy.deepcopy(geometry);invalid["closed_world_bounds"]["max"] = upper
            with self.assertRaisesRegex(ValueError, "voxel_prior_too_large"):
                truth.rasterize(invalid)

    def test_floor_ceiling_wall_cabinet_and_low_block_are_complete_solids(self):
        for index in ([-80, -60, -1], [-80, -60, 0], [-80, -60, 49],
                      [-80, -60, 50], [-120, -60, 10], [-66, 14, 10], [-76, 64, 4]):
            self.assertEqual(self.status(index), truth.OCCUPIED, index)
        self.assertEqual(self.status([-80, -60, 1]), truth.FREE)

    def test_doorway_is_free_but_outside_room_is_unknown(self):
        self.assertEqual(self.status([-20, -24, 10]), truth.FREE)
        self.assertEqual(self.status([-200, 0, 10]), truth.UNKNOWN)
        self.assertEqual(self.status([-80, -60, 53]), truth.UNKNOWN)

    def test_thin_wall_intersection_is_not_missed_by_eight_empty_corners(self):
        geometry = copy.deepcopy(self.geometry)
        # Interior obstacle narrower than one voxel, with all cell corner x
        # coordinates strictly outside it. A center/corner-only mask is unsafe.
        geometry["boxes"].append(dict(path="thin", type="Cube", min=[.011, -.2, .2], max=[.012, .2, .8]))
        data, origin, shape = truth.rasterize(geometry, margin=0.)
        index = [0, 0, 8]
        local = [i - o for i, o in zip(index, origin)]
        self.assertEqual(data[(local[0] * shape[1] + local[1]) * shape[2] + local[2]], truth.OCCUPIED)

    def test_exact_surface_touch_is_occupied_and_uncertainty_strip_unknown(self):
        geometry = copy.deepcopy(self.geometry)
        geometry["boxes"].append(dict(path="near", type="Cube", min=[.06, -.2, .2], max=[.10, .2, .8]))
        data, origin, shape = truth.rasterize(geometry, margin=.02)
        def read(x):
            a, b, c = [i - o for i, o in zip([x, 0, 8], origin)]
            return data[(a * shape[1] + b) * shape[2] + c]
        self.assertEqual(read(0), truth.UNKNOWN)  # [.0,.05] is margin-only
        self.assertEqual(read(1), truth.OCCUPIED)
        self.assertEqual(read(2), truth.OCCUPIED)  # [.10,.15] touches solid
        self.assertEqual(read(3), truth.FREE)

    def test_missing_enclosure_and_inconsistent_floor_are_rejected(self):
        bad = copy.deepcopy(SPEC)
        bad["static_boxes"] = [b for b in bad["static_boxes"] if b["name"] != "west_wall"]
        with self.assertRaisesRegex(ValueError, "missing_room_enclosure"):
            truth.geometry_from_spec(bad)
        bad = copy.deepcopy(SPEC)
        bad["floor"]["bounds"][0] -= 1
        with self.assertRaisesRegex(ValueError, "inconsistent_floor_domain"):
            truth.geometry_from_spec(bad)

    def test_invalid_voxel_settings_cannot_create_a_free_certificate(self):
        for resolution in (0, math.nan, -1, True, ".05"):
            with self.assertRaises(ValueError):
                truth.rasterize(self.geometry, resolution)
        for margin in (-1, math.inf, math.nan, False):
            with self.assertRaises(ValueError):
                truth.rasterize(self.geometry, margin=margin)

    def test_nonfinite_or_boolean_spec_dimensions_rejected(self):
        for value in (math.nan, math.inf, True, "1.0"):
            bad = copy.deepcopy(SPEC)
            bad["static_boxes"][0]["size"][0] = value
            with self.assertRaises(ValueError):
                truth.geometry_from_spec(bad)


class PriorFileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)
        self.manifest = self.folder / "prior.json"
        self.usd = self.folder / "source.usda"
        self.usd.write_text("source content for pure file-loader unit tests")
        # USD authenticity is tested separately below with real OpenUSD.
        with patch.object(truth, "verify_stage", return_value={}):
            truth.build(SPEC_PATH, self.usd, self.manifest)

    def tearDown(self):
        self.temp.cleanup()

    def mutate(self, changes):
        value = json.loads(self.manifest.read_text())
        value.update(changes)
        self.manifest.write_text(json.dumps(value))

    def build_support_prior(self, endpoint_error_bound=None):
        spec_path = self.folder / "contact_spec.json"
        spec_path.write_text(json.dumps(support_spec(endpoint_error_bound)))
        with patch.object(truth, "verify_stage", return_value={}):
            truth.build(spec_path, self.usd, self.manifest)
        return json.loads(self.manifest.read_text())

    def test_floor_endpoint_loader_keeps_legacy_default_and_exact_sealed_optional_field(self):
        legacy = self.build_support_prior()
        self.assertEqual(truth.StaticPrior(self.manifest).floor_endpoint_error_bound_m, 1e-5)
        self.assertNotIn("floor_endpoint_error_bound_m", legacy["flat_support_contact"])
        sealed = self.build_support_prior(.0002)
        self.assertEqual(truth.StaticPrior(self.manifest).floor_endpoint_error_bound_m, .0002)
        self.assertEqual(sealed["flat_support_contact"], sealed["collision_geometry"]["flat_support_contact"])
        self.assertEqual(sealed["data_sha256"], legacy["data_sha256"])
        self.assertNotEqual(sealed["collider_sha256"], legacy["collider_sha256"])

    def test_floor_endpoint_loader_rejects_contact_copy_mismatch_and_unbound_hash_change(self):
        original = self.build_support_prior(.0002)
        for variant in ("top_only", "geometry_only", "both"):
            with self.subTest(variant=variant):
                changed = copy.deepcopy(original)
                if variant in ("top_only", "both"):
                    changed["flat_support_contact"]["floor_endpoint_error_bound_m"] = .0003
                if variant in ("geometry_only", "both"):
                    changed["collision_geometry"]["flat_support_contact"]["floor_endpoint_error_bound_m"] = .0003
                self.manifest.write_text(json.dumps(changed))
                reason = "static_prior_geometry_hash_mismatch" if variant == "both" else "invalid_flat_support_contact_contract"
                with self.assertRaisesRegex(ValueError, reason):
                    truth.StaticPrior(self.manifest)

    def test_floor_endpoint_loader_enforces_absolute_and_resolution_caps_even_with_matching_hash(self):
        original = self.build_support_prior(.0002)
        for bound, resolution in ((9e-6, .05), (.001000001, .05), (.0002, .001)):
            with self.subTest(bound=bound, resolution=resolution):
                changed = copy.deepcopy(original)
                for contact in (changed["flat_support_contact"], changed["collision_geometry"]["flat_support_contact"]):
                    contact["floor_endpoint_error_bound_m"] = bound
                changed["collider_sha256"] = hashlib.sha256(truth._canonical(changed["collision_geometry"])).hexdigest()
                changed["voxel_resolution"] = resolution
                self.manifest.write_text(json.dumps(changed))
                with self.assertRaisesRegex(ValueError, "floor_endpoint_error_bound_out_of_range"):
                    truth.StaticPrior(self.manifest)

    def test_layout_global_negative_indices_and_outside_unknown(self):
        prior = truth.StaticPrior(self.manifest)
        self.assertEqual(prior.status([-80, -60, 10]), truth.FREE)
        self.assertEqual(prior.status([-80, -60, 0]), truth.OCCUPIED)
        self.assertEqual(prior.status([-1000, 0, 0]), truth.UNKNOWN)
        for index in ([1., 2, 3], [True, 2, 3], [1, 2]):
            with self.assertRaises(ValueError):
                prior.status(index)

    def test_binary_hash_and_length_mismatch_rejected(self):
        binary = self.manifest.with_suffix(".bin")
        data = bytearray(binary.read_bytes())
        data[0] ^= 1
        binary.write_bytes(data)
        with self.assertRaisesRegex(ValueError, "hash_mismatch"):
            truth.StaticPrior(self.manifest)
        binary.write_bytes(data[:-1])
        with self.assertRaisesRegex(ValueError, "size_mismatch"):
            truth.StaticPrior(self.manifest)

    def test_hash_matching_invalid_state_is_still_rejected(self):
        binary = self.manifest.with_suffix(".bin")
        data = bytearray(binary.read_bytes())
        data[0] = 99
        binary.write_bytes(data)
        self.mutate(dict(data_sha256=hashlib.sha256(data).hexdigest()))
        with self.assertRaisesRegex(ValueError, "invalid_static_prior_state"):
            truth.StaticPrior(self.manifest)

    def test_relative_basename_path_and_storage_contract(self):
        self.mutate(dict(data_file="../prior.bin"))
        with self.assertRaisesRegex(ValueError, "unsafe_static_prior_data_path"):
            truth.StaticPrior(self.manifest)

    def test_coordinate_geometry_and_dimensions_contract_rejected(self):
        original = self.manifest.read_text()
        for changed in (dict(map_from_odom={}), dict(up_axis="Y"), dict(origin_index=[0, 0]),
                        dict(shape=[True, 1, 1]), dict(collider_sha256="0" * 64), dict(voxel_resolution=0)):
            self.manifest.write_text(original)
            self.mutate(changed)
            with self.assertRaises(ValueError):
                truth.StaticPrior(self.manifest)

    def test_caller_map_identity_is_preserved_separate_from_geometry_version(self):
        with patch.object(truth, "verify_stage", return_value={}):
            result = truth.build(SPEC_PATH, self.usd, self.manifest, map_version="source-map:approved_epoch_7")
        self.assertEqual(result["map_version"], "source-map:approved_epoch_7")
        manifest = json.loads(self.manifest.read_text())
        self.assertEqual(manifest["geometry_version"], manifest["collider_sha256"])
        for version in ("", "a\nb", "a/b", True):
            with self.assertRaisesRegex(ValueError, "invalid_map_version"):
                truth.build(SPEC_PATH, self.usd, self.manifest, map_version=version)


@unittest.skipUnless(HAS_USD, "OpenUSD required only for source-stage tests")
class CollisionStageTests(unittest.TestCase):
    def stage(self):
        from pxr import Usd
        stage = Usd.Stage.CreateInMemory()
        stage.GetRootLayer().ImportFromString(USD_PATH.read_text())
        return stage

    def test_real_export_matches_and_normal_robot_motion_is_excluded(self):
        from pxr import Gf
        stage = self.stage()
        expected = truth.verify_stage_geometry(stage, SPEC)
        self.assertEqual(expected, hashlib.sha256(truth._canonical(truth.geometry_from_spec(SPEC))).hexdigest())
        stage.GetPrimAtPath("/World/WheelFixture/Geometry/base_link").GetAttribute("xformOp:translate").Set(Gf.Vec3d(1, 2, .35))
        stage.GetPrimAtPath("/World/WheelFixture/Geometry/base_link/left_wheel_link").GetAttribute("xformOp:orient").Set(Gf.Quatf(.8, .6, 0, 0))
        self.assertEqual(truth.verify_stage_geometry(stage, SPEC), expected)

    def test_floor_endpoint_error_bound_does_not_relax_exact_plane_geometry_attestation(self):
        from pxr import Gf
        stage = self.stage();spec = support_spec(.0002)
        verifier = truth.StageGeometryVerifier(stage, spec)
        original = verifier.verify()
        self.assertEqual(original, hashlib.sha256(truth._canonical(truth.geometry_from_spec(spec))).hexdigest())
        stage.GetPrimAtPath("/World/GroundPlane").GetAttribute("xformOp:translate").Set(Gf.Vec3f(0., 0., .0001))
        with self.assertRaisesRegex(ValueError, "floor_plane_mismatch"):
            verifier.verify()
        self.assertEqual(verifier.full_audits, 2);verifier.close()

    def test_new_nonrobot_dynamic_obstacle_invalidates_static_free(self):
        from pxr import UsdGeom, UsdPhysics
        stage = self.stage()
        obstacle = UsdGeom.Cube.Define(stage, "/World/NewObstacle").GetPrim()
        UsdPhysics.CollisionAPI.Apply(obstacle)
        UsdPhysics.RigidBodyAPI.Apply(obstacle)
        with self.assertRaisesRegex(ValueError, "unlisted_static_collider"):
            truth.verify_stage_geometry(stage, SPEC)

    def test_explicitly_disabled_visual_api_allowed_but_enable_revokes(self):
        from pxr import UsdGeom, UsdPhysics
        stage = self.stage()
        visual = UsdGeom.Mesh.Define(stage, "/World/WheelFixture/Visual/disabled_mesh").GetPrim()
        api = UsdPhysics.CollisionAPI.Apply(visual);api.CreateCollisionEnabledAttr(False)
        truth.verify_stage_geometry(stage, SPEC)
        api.CreateCollisionEnabledAttr(True)
        with self.assertRaisesRegex(ValueError, "unlisted_static_collider"):
            truth.verify_stage_geometry(stage, SPEC)

    def test_notice_cache_reuses_only_exact_pose_changes_and_never_hides_static_mutation(self):
        from pxr import Gf, UsdGeom, UsdPhysics
        stage = self.stage();verifier = truth.StageGeometryVerifier(stage, SPEC)
        original = verifier.verify();self.assertEqual(verifier.full_audits, 1)
        root = stage.GetPrimAtPath("/World/WheelFixture/Geometry/base_link")
        for x in [1., 2., 3.]:
            root.GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, 2, .35))
            self.assertEqual(verifier.verify(), original)
        self.assertEqual(verifier.full_audits, 1);self.assertEqual(verifier.cache_hits, 3)
        stage.GetPrimAtPath("/World/Indoor/low_block").GetAttribute("xformOp:translate").Set(Gf.Vec3d(0, 0, .12))
        with self.assertRaisesRegex(ValueError, "static_collider_geometry_mismatch"):
            verifier.verify()
        with self.assertRaises(ValueError):
            verifier.verify() # Cannot return an old certificate after a failed recheck.
        verifier.close()
        with self.assertRaisesRegex(ValueError, "geometry_verifier_closed"):
            verifier.verify()

    def measurement_stage(self):
        from pxr import Gf, Sdf, UsdPhysics
        stage = self.stage()
        body = stage.GetPrimAtPath("/World/WheelFixture/Geometry/base_link")
        api = UsdPhysics.RigidBodyAPI(body)
        api.CreateVelocityAttr(Gf.Vec3f(0.));api.CreateAngularVelocityAttr(Gf.Vec3f(0.))
        joint = UsdPhysics.RevoluteJoint.Define(stage, str(body.GetPath()) + "/MeasuredJoint").GetPrim()
        for name in ("state:angular:physics:position", "state:angular:physics:velocity"):
            joint.CreateAttribute(name, Sdf.ValueTypeNames.Float).Set(0.)
        lidar = stage.GetPrimAtPath(str(body.GetPath()) + "/Lidar_0")
        lidar.CreateAttribute("enabled", Sdf.ValueTypeNames.Bool).Set(True)
        return stage, body, joint, lidar

    def test_native_rigid_joint_and_lidar_measurements_reuse_only_existing_typed_defaults(self):
        from pxr import Gf
        stage, body, joint, lidar = self.measurement_stage()
        verifier = truth.StageGeometryVerifier(stage, SPEC);original = verifier.verify()
        for index in range(50):
            body.GetAttribute("physics:velocity").Set(Gf.Vec3f(index * .001, .02, -.01))
            body.GetAttribute("physics:angularVelocity").Set(Gf.Vec3f(.01, index * .002, .03))
            joint.GetAttribute("state:angular:physics:position").Set(index * .01)
            joint.GetAttribute("state:angular:physics:velocity").Set(index * .02)
            lidar.GetAttribute("enabled").Set(bool(index % 2))
            self.assertEqual(verifier.verify(), original)
        self.assertEqual(verifier.full_audits, 1);self.assertEqual(verifier.cache_hits, 50)
        verifier.close()

    def test_existing_configured_rtx_camera_lidar_enable_is_cached_without_type_prefix_exemption(self):
        from pxr import Sdf, UsdGeom
        stage, body, joint, lidar = self.measurement_stage()
        lidar.SetTypeName("Camera")
        extra = UsdGeom.Camera.Define(stage, str(body.GetPath()) + "/UnconfiguredLidar").GetPrim()
        extra.CreateAttribute("enabled", Sdf.ValueTypeNames.Bool).Set(True)
        verifier = truth.StageGeometryVerifier(stage, SPEC);original = verifier.verify()
        for state in (False, True, False):
            lidar.GetAttribute("enabled").Set(state)
            self.assertEqual(verifier.verify(), original)
        self.assertEqual(verifier.full_audits, 1)
        extra.GetAttribute("enabled").Set(False)
        self.assertTrue(verifier._dirty);verifier.verify();self.assertEqual(verifier.full_audits, 2)
        verifier.close()

    def test_measurement_names_on_unapproved_or_wrong_typed_prim_are_never_exempt(self):
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics
        for variant in ("outside_rigid", "not_joint", "not_lidar", "collision_lidar", "wrong_type"):
            with self.subTest(variant=variant):
                stage, body, joint, lidar = self.measurement_stage()
                if variant == "outside_rigid":
                    prim = UsdGeom.Xform.Define(stage, "/World/UnregisteredBody").GetPrim()
                    attribute = UsdPhysics.RigidBodyAPI.Apply(prim).CreateVelocityAttr(Gf.Vec3f(0.));value = Gf.Vec3f(1.)
                elif variant == "not_joint":
                    prim = UsdGeom.Xform.Define(stage, str(body.GetPath()) + "/NotJoint").GetPrim()
                    attribute = prim.CreateAttribute("state:angular:physics:position", Sdf.ValueTypeNames.Float)
                    attribute.Set(0.);value = 1.
                elif variant == "not_lidar":
                    prim = UsdGeom.Xform.Define(stage, str(body.GetPath()) + "/NotLidar").GetPrim()
                    attribute = prim.CreateAttribute("enabled", Sdf.ValueTypeNames.Bool);attribute.Set(True);value = False
                elif variant == "collision_lidar":
                    UsdPhysics.CollisionAPI.Apply(lidar).CreateCollisionEnabledAttr(False)
                    attribute = lidar.GetAttribute("enabled");value = False
                else:
                    stage.GetRootLayer().GetAttributeAtPath(str(body.GetPath()) + ".physics:velocity").SetInfo("typeName", "double3")
                    attribute = body.GetAttribute("physics:velocity");value = Gf.Vec3d(1.)
                verifier = truth.StageGeometryVerifier(stage, SPEC);original = verifier.verify()
                attribute.Set(value)
                self.assertTrue(verifier._dirty)
                self.assertEqual(verifier.verify(), original);self.assertEqual(verifier.full_audits, 2)
                verifier.close()

    def test_measurement_type_schema_control_scale_and_time_sample_changes_still_invalidate(self):
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
        for variant in ("type", "late_attribute", "new_prim", "api", "gain", "damping", "material", "scale", "time_sample"):
            with self.subTest(variant=variant):
                stage, body, joint, lidar = self.measurement_stage()
                if variant == "late_attribute":
                    joint.RemoveProperty("state:angular:physics:velocity")
                for name in ("drive:angular:physics:stiffness", "drive:angular:physics:damping"):
                    joint.CreateAttribute(name, Sdf.ValueTypeNames.Float).Set(0.)
                verifier = truth.StageGeometryVerifier(stage, SPEC);verifier.verify()
                if variant == "type":
                    stage.GetRootLayer().GetAttributeAtPath(str(body.GetPath()) + ".physics:velocity").SetInfo("typeName", "double3")
                elif variant == "late_attribute":
                    joint.CreateAttribute("state:angular:physics:velocity", Sdf.ValueTypeNames.Float).Set(1.)
                elif variant == "new_prim":
                    added = UsdPhysics.RevoluteJoint.Define(stage, str(body.GetPath()) + "/LateJoint").GetPrim()
                    added.CreateAttribute("state:angular:physics:position", Sdf.ValueTypeNames.Float).Set(1.)
                elif variant == "api":
                    UsdPhysics.CollisionAPI.Apply(lidar).CreateCollisionEnabledAttr(True)
                elif variant == "gain":
                    joint.GetAttribute("drive:angular:physics:stiffness").Set(20.)
                elif variant == "damping":
                    joint.GetAttribute("drive:angular:physics:damping").Set(80.)
                elif variant == "material":
                    body.CreateRelationship("material:binding:physics").SetTargets(["/World/NewMaterial"])
                elif variant == "scale":
                    stage.GetPrimAtPath(str(body.GetPath()) + "/box_1").GetAttribute("xformOp:scale").Set(Gf.Vec3d(2., 2., 2.))
                else:
                    body.GetAttribute("physics:velocity").Set(Gf.Vec3f(1.), Usd.TimeCode(1.))
                self.assertTrue(verifier._dirty)
                if variant in ("api", "scale"):
                    with self.assertRaises(ValueError):
                        verifier.verify()
                else:
                    verifier.verify()
                self.assertEqual(verifier.full_audits, 2)
                # A recheck does not enroll late attrs/prims or a changed type,
                # and time-sampled measurements never become default-only.
                if variant in ("type", "late_attribute", "new_prim", "time_sample"):
                    if variant == "type":
                        body.GetAttribute("physics:velocity").Set(Gf.Vec3d(2.))
                    elif variant == "late_attribute":
                        joint.GetAttribute("state:angular:physics:velocity").Set(2.)
                    elif variant == "new_prim":
                        added.GetAttribute("state:angular:physics:position").Set(2.)
                    else:
                        body.GetAttribute("physics:velocity").Set(Gf.Vec3f(2.))
                    self.assertTrue(verifier._dirty);verifier.verify();self.assertEqual(verifier.full_audits, 3)
                verifier.close()

    def test_notice_cache_revokes_on_disabled_visual_enable_new_collider_scale_and_composition(self):
        from pxr import Gf, UsdGeom, UsdPhysics
        for variant in ("enable", "new", "scale", "delete", "composition"):
            stage = self.stage()
            mesh = UsdGeom.Mesh.Define(stage, "/World/Visual").GetPrim()
            api = UsdPhysics.CollisionAPI.Apply(mesh);api.CreateCollisionEnabledAttr(False)
            verifier = truth.StageGeometryVerifier(stage, SPEC);verifier.verify()
            if variant == "enable":
                api.CreateCollisionEnabledAttr(True)
            if variant == "new":
                UsdPhysics.CollisionAPI.Apply(UsdGeom.Cube.Define(stage, "/World/Unexpected").GetPrim())
            if variant == "scale":
                stage.GetPrimAtPath("/World/WheelFixture/Geometry/base_link/box_1").GetAttribute("xformOp:scale").Set(Gf.Vec3d(10, 10, 10))
            if variant == "delete":
                stage.RemovePrim("/World/Indoor/low_block")
            if variant == "composition":
                stage.GetPrimAtPath("/World/Indoor/low_block").GetReferences().AddInternalReference("/World/Indoor/west_cabinet")
            if variant == "composition":
                verifier.verify() # Harmless geometry-equivalent composition still needs a full recheck.
            else:
                with self.assertRaises(ValueError, msg=variant):
                    verifier.verify()
            self.assertGreater(verifier.full_audits, 1);verifier.close()

    def test_existing_decor_camera_matrix_is_cached_but_collision_api_never_exempt(self):
        from pxr import Gf, UsdGeom, UsdPhysics
        stage = self.stage()
        camera = UsdGeom.Camera.Define(stage, "/World/ExistingFollowCamera").GetPrim()
        transform = UsdGeom.Xformable(camera).AddTransformOp();transform.Set(Gf.Matrix4d(1.))
        verifier = truth.StageGeometryVerifier(stage, SPEC);original = verifier.verify()
        for x in [1., 2., 3.]:
            transform.Set(Gf.Matrix4d(1.).SetTranslate(Gf.Vec3d(x, 2, 3)))
            self.assertEqual(verifier.verify(), original)
        self.assertEqual(verifier.full_audits, 1)
        UsdPhysics.CollisionAPI.Apply(camera).CreateCollisionEnabledAttr(True)
        with self.assertRaisesRegex(ValueError, "unlisted_static_collider"):
            verifier.verify()
        self.assertEqual(verifier.full_audits, 2);verifier.close()

    def test_floor_physics_material_is_in_hash_and_cache_rejects_binding_or_friction_change(self):
        from pxr import Sdf, UsdPhysics, UsdShade
        stage = self.stage();spec = copy.deepcopy(SPEC)
        declaration = dict(schema=1, path="/World/PhysicsMaterials/CampusFloor", static_friction=1.,
            dynamic_friction=1., restitution=0., binding_purpose="physics")
        spec["floor"]["physics_material"] = declaration
        material = UsdShade.Material.Define(stage, declaration["path"])
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(1.);api.CreateDynamicFrictionAttr(1.);api.CreateRestitutionAttr(0.)
        floor = stage.GetPrimAtPath("/World/GroundPlane/collisionPlane")
        binding = UsdShade.MaterialBindingAPI.Apply(floor);binding.Bind(material, materialPurpose="physics")
        verifier = truth.StageGeometryVerifier(stage, spec);original = verifier.verify()
        self.assertNotEqual(original, truth.verify_stage_geometry(stage, SPEC))
        api.CreateStaticFrictionAttr(.9)
        with self.assertRaisesRegex(ValueError, "floor_physics_material_value_changed"):
            verifier.verify()
        api.CreateStaticFrictionAttr(1.);self.assertEqual(verifier.verify(), original)
        alternative = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/Unapproved")
        binding.Bind(alternative, materialPurpose="physics")
        with self.assertRaisesRegex(ValueError, "floor_physics_material_binding_changed"):
            verifier.verify()
        binding.Bind(material, materialPurpose="physics");self.assertEqual(verifier.verify(), original)
        material.GetPrim().CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("min")
        with self.assertRaisesRegex(ValueError, "unsealed_floor_physical_material_attribute"):
            verifier.verify()
        verifier.close()

    def test_finite_quadruped_registry_supports_capsule_and_has_no_wheel_count_exemption(self):
        from pxr import Gf, UsdGeom, UsdPhysics
        stage = self.stage();stage.RemovePrim("/World/WheelFixture")
        root = UsdGeom.Xform.Define(stage, "/World/Spot").GetPrim();UsdPhysics.RigidBodyAPI.Apply(root)
        capsule = UsdGeom.Capsule.Define(stage, "/World/Spot/link/leg")
        capsule.CreateRadiusAttr(.04);capsule.CreateHeightAttr(.2);capsule.CreateAxisAttr("X")
        UsdPhysics.CollisionAPI.Apply(capsule.GetPrim()).CreateCollisionEnabledAttr(True)
        spec = copy.deepcopy(SPEC);spec["robot_collision_registry"] = truth.robot_registry_from_stage(stage, "/World/Spot")
        self.assertEqual(len(spec["robot_collision_registry"]["colliders"]), 1)
        original = truth.verify_stage_geometry(stage, spec)
        UsdGeom.Xformable(root).AddTranslateOp().Set(Gf.Vec3d(2, 3, .5))
        UsdGeom.Xformable(root).AddRotateXYZOp().Set(Gf.Vec3f(8, 6, 20))
        self.assertEqual(truth.verify_stage_geometry(stage, spec), original)
        bounds = truth.registered_robot_world_bounds(stage, spec["robot_collision_registry"])
        self.assertEqual(len(bounds[0]["corners_world"]), 8)
        rogue = UsdGeom.Cube.Define(stage, "/World/Spot/link/unregistered").GetPrim();UsdPhysics.CollisionAPI.Apply(rogue)
        with self.assertRaisesRegex(ValueError, "unlisted_static_collider"):
            truth.verify_stage_geometry(stage, spec)

    def test_registered_dynamic_geometry_can_move_but_cannot_change_or_disappear(self):
        from pxr import Gf, UsdGeom, UsdPhysics
        from test_dynamic_collision import ACTOR
        stage = self.stage();spec = copy.deepcopy(SPEC);spec["dynamic_actors"] = [ACTOR]
        root = UsdGeom.Xform.Define(stage, "/World/Dynamic/person").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(root).CreateKinematicEnabledAttr(True)
        transform = UsdGeom.Xformable(root);translate = transform.AddTranslateOp();translate.Set(Gf.Vec3d(0, 0, 0))
        rotate = transform.AddRotateZOp();rotate.Set(0.)
        cube = UsdGeom.Cube.Define(stage, "/World/Dynamic/person/torso");cube.CreateSizeAttr(1.)
        transform = UsdGeom.Xformable(cube);transform.AddTranslateOp().Set(Gf.Vec3d(0, 0, .9))
        transform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(.5, .5, 1.8))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        original = truth.verify_stage_geometry(stage, spec)
        verifier = truth.StageGeometryVerifier(stage, spec);verifier.verify()
        translate.Set(Gf.Vec3d(-4, -3, 0));rotate.Set(90.)
        self.assertEqual(truth.verify_stage_geometry(stage, spec), original)
        self.assertEqual(verifier.verify(), original);self.assertEqual(verifier.full_audits, 1)
        cube.CreateSizeAttr(2.)
        with self.assertRaisesRegex(ValueError, "registered_actor_shape_changed"):
            truth.verify_stage_geometry(stage, spec)
        with self.assertRaisesRegex(ValueError, "registered_actor_shape_changed"):
            verifier.verify()
        cube.CreateSizeAttr(1.);verifier.verify()
        root.GetAttribute("physics:kinematicEnabled").Set(False)
        with self.assertRaisesRegex(ValueError, "registered_actor_not_dynamic"):
            verifier.verify()
        root.GetAttribute("physics:kinematicEnabled").Set(True);verifier.verify()
        cube.CreateSizeAttr(1.);stage.RemovePrim("/World/Dynamic/person/torso")
        with self.assertRaisesRegex(ValueError, "missing_registered_dynamic_actor"):
            truth.verify_stage_geometry(stage, spec)
        with self.assertRaisesRegex(ValueError, "missing_registered_dynamic_actor"):
            verifier.verify()
        verifier.close()

    def scripted_actor_stage(self):
        from pxr import Gf, UsdGeom, UsdPhysics
        from test_dynamic_collision import ACTOR
        actor = copy.deepcopy(ACTOR)
        actor.update(start_time_s=0., end_time_s=4., trajectory=dict(
            mode="once", interpolation="c1_smoothstep", waypoints=[
                dict(time_s=0., position=[-4., -3., 0.], yaw=0.),
                dict(time_s=2., position=[-2., -3., 0.], yaw=math.pi/2),
                dict(time_s=4., position=[-4., -3., 0.], yaw=0.)]),
            future_motion_contract=dict(schema=1, kind="sealed_c1_smoothstep_kinematic_v1",
                source_scope="isolated_simulation_enforced_kinematic_script",
                actual_pose_error_bound_m=.0001, actual_tilt_error_bound_rad=.000001,
                physics_dt_ns=2_000_000))
        stage = self.stage();spec = copy.deepcopy(SPEC);spec["dynamic_actors"] = [actor]
        UsdGeom.Xform.Define(stage, "/World/Dynamic")
        root = UsdGeom.Xform.Define(stage, "/World/Dynamic/person").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(root).CreateKinematicEnabledAttr(True)
        transform = UsdGeom.Xformable(root)
        translate = transform.AddTranslateOp();translate.Set(Gf.Vec3d(-4., -3., 0.))
        rotate = transform.AddRotateZOp();rotate.Set(0.)
        cube = UsdGeom.Cube.Define(stage, "/World/Dynamic/person/torso");cube.CreateSizeAttr(1.)
        child_transform = UsdGeom.Xformable(cube)
        child_transform.AddTranslateOp().Set(Gf.Vec3d(0., 0., .9))
        child_transform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(.5, .5, 1.8))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        return stage, spec, root, translate, rotate

    def test_scripted_actor_original_world_script_and_pose_notice_cache_remain_valid(self):
        from world_builder import update_actors
        stage, spec, root, _, _ = self.scripted_actor_stage()
        verifier = truth.StageGeometryVerifier(stage, spec)
        original = verifier.verify()
        for source in (0., 1., 2., 3., 4., 6.):
            update_actors(stage, spec, source)
            self.assertEqual(truth.verify_stage_geometry(stage, spec), original)
            self.assertEqual(verifier.verify(), original)
        self.assertEqual(verifier.full_audits, 1)
        verifier.close()

    def test_scripted_actor_parent_pivot_can_preserve_current_pose_but_redirect_future(self):
        from pxr import Gf, UsdGeom
        from world_builder import actor_pose, update_actors
        stage, spec, root, _, _ = self.scripted_actor_stage()
        verifier = truth.StageGeometryVerifier(stage, spec);verifier.verify()
        pivot = Gf.Vec3d(-4., -3., 0.)
        around_pivot = (Gf.Matrix4d(1.).SetTranslate(-pivot)
                        * Gf.Matrix4d(1.).SetRotate(Gf.Rotation(Gf.Vec3d(0., 0., 1.), 90.))
                        * Gf.Matrix4d(1.).SetTranslate(pivot))
        UsdGeom.Xformable(stage.GetPrimAtPath("/World/Dynamic")).AddTransformOp().Set(around_pivot)
        current = UsdGeom.XformCache().GetLocalToWorldTransform(root).Transform(Gf.Vec3d(0.))
        self.assertLess((current-pivot).GetLength(), 1e-12)
        with self.assertRaisesRegex(ValueError, "scripted_actor_parent_world_transform_changed"):
            verifier.verify()
        update_actors(stage, spec, 1.)
        actual_future = UsdGeom.XformCache().GetLocalToWorldTransform(root).Transform(Gf.Vec3d(0.))
        expected_future = Gf.Vec3d(*actor_pose(spec["dynamic_actors"][0], 1.)["position"])
        self.assertGreater((actual_future-expected_future).GetLength(), 1.)
        verifier.close()

    def test_scripted_actor_parent_or_world_time_samples_revoke_even_with_identity_default(self):
        from pxr import Gf, Usd, UsdGeom
        for path in ("/World/Dynamic", "/World"):
            with self.subTest(path=path):
                stage, spec, root, _, _ = self.scripted_actor_stage()
                verifier = truth.StageGeometryVerifier(stage, spec);verifier.verify()
                op = UsdGeom.Xformable(stage.GetPrimAtPath(path)).AddTranslateOp()
                op.Set(Gf.Vec3d(0.));op.Set(Gf.Vec3d(0.), Usd.TimeCode(1.))
                with self.assertRaisesRegex(ValueError, "scripted_actor_ancestor_time_samples"):
                    truth._audit_scripted_actor_transform(root, UsdGeom.XformCache())
                expected = ("time_varying_static_collider" if path == "/World"
                            else "scripted_actor_ancestor_time_samples")
                with self.assertRaisesRegex(ValueError, expected):
                    verifier.verify()
                verifier.close()

    def test_scripted_actor_root_extra_inverse_reset_order_and_time_samples_revoke(self):
        from pxr import Gf, Usd, UsdGeom
        for change in ("extra", "inverse", "reset", "order", "order_time", "pose_time"):
            with self.subTest(change=change):
                stage, spec, root, translate, rotate = self.scripted_actor_stage()
                verifier = truth.StageGeometryVerifier(stage, spec);verifier.verify()
                transform = UsdGeom.Xformable(root)
                if change == "extra":
                    transform.AddRotateZOp(opSuffix="extra").Set(0.)
                elif change == "inverse":
                    transform.AddTranslateOp(isInverseOp=True)
                elif change == "reset":
                    transform.SetResetXformStack(True)
                elif change == "order":
                    transform.SetXformOpOrder([rotate, translate])
                    # At current yaw zero the measured root position still
                    # agrees; the future rotateZ would rotate its translation.
                    current = UsdGeom.XformCache().GetLocalToWorldTransform(root).Transform(Gf.Vec3d(0.))
                    self.assertEqual(current, Gf.Vec3d(-4., -3., 0.))
                elif change == "order_time":
                    transform.GetXformOpOrderAttr().Set(["xformOp:translate", "xformOp:rotateZ"], Usd.TimeCode(1.))
                else:
                    translate.Set(Gf.Vec3d(-4., -3., 0.), Usd.TimeCode(1.))
                with self.assertRaisesRegex(ValueError, "scripted_actor_root_transform_contract_changed"):
                    verifier.verify()
                verifier.close()

    def test_generic_actor_parent_transform_keeps_original_non_scripted_domain(self):
        from pxr import Gf, UsdGeom
        stage, spec, _, _, _ = self.scripted_actor_stage()
        spec["dynamic_actors"][0].pop("future_motion_contract")
        original = truth.verify_stage_geometry(stage, spec)
        UsdGeom.Xformable(stage.GetPrimAtPath("/World/Dynamic")).AddTranslateOp().Set(Gf.Vec3d(1., 2., 0.))
        self.assertEqual(truth.verify_stage_geometry(stage, spec), original)

    def test_scripted_actor_root_new_enabled_collider_revokes_complete_shape_registry(self):
        from pxr import UsdGeom, UsdPhysics
        stage, spec, root, _, _ = self.scripted_actor_stage()
        verifier = truth.StageGeometryVerifier(stage, spec)
        original = verifier.verify()
        rogue = UsdGeom.Cube.Define(stage, str(root.GetPath()) + "/rogue").GetPrim()
        enabled = UsdPhysics.CollisionAPI.Apply(rogue).CreateCollisionEnabledAttr(False)
        self.assertEqual(verifier.verify(), original)
        enabled.Set(True)
        with self.assertRaisesRegex(ValueError, "unlisted_static_collider"):
            verifier.verify()
        verifier.close()

    def test_new_instanced_collision_child_cannot_evade_attestation(self):
        from pxr import Usd, UsdGeom, UsdPhysics
        with tempfile.TemporaryDirectory() as folder:
            asset = Path(folder) / "obstacle.usda"
            obstacle = Usd.Stage.CreateNew(str(asset))
            UsdGeom.Xform.Define(obstacle, "/Obstacle")
            UsdPhysics.CollisionAPI.Apply(UsdGeom.Cube.Define(obstacle, "/Obstacle/Collider").GetPrim())
            obstacle.GetRootLayer().Save()
            stage = self.stage()
            instance = UsdGeom.Xform.Define(stage, "/World/NewInstance").GetPrim()
            instance.GetReferences().AddReference(str(asset), "/Obstacle")
            instance.SetInstanceable(True)
            with self.assertRaisesRegex(ValueError, "unlisted_static_collider"):
                truth.verify_stage_geometry(stage, SPEC)

    def test_moved_resized_missing_disabled_and_tilted_static_body_rejected(self):
        from pxr import Gf
        cases = (("xformOp:translate", Gf.Vec3d(-3.2, .7, .55)),
                 ("xformOp:scale", Gf.Vec3d(.4, .9, .55)),
                 ("xformOp:orient", Gf.Quatd(.999, .01, 0, 0)))
        for attribute, value in cases:
            stage = self.stage()
            stage.GetPrimAtPath("/World/Indoor/west_cabinet").GetAttribute(attribute).Set(value)
            with self.assertRaises(ValueError):
                truth.verify_stage_geometry(stage, SPEC)
        stage = self.stage()
        stage.RemovePrim("/World/Indoor/low_block")
        with self.assertRaisesRegex(ValueError, "missing_static_colliders"):
            truth.verify_stage_geometry(stage, SPEC)
        stage = self.stage()
        stage.GetPrimAtPath("/World/Indoor/low_block").GetAttribute("physics:collisionEnabled").Set(False)
        with self.assertRaisesRegex(ValueError, "disabled_static_collider"):
            truth.verify_stage_geometry(stage, SPEC)

    def test_time_sampled_static_transform_is_rejected(self):
        from pxr import Gf, Usd
        stage = self.stage()
        stage.GetPrimAtPath("/World/Indoor/low_block").GetAttribute("xformOp:translate").Set(Gf.Vec3d(-3.8, 3.2, .12), Usd.TimeCode(1))
        with self.assertRaisesRegex(ValueError, "time_varying_static_collider"):
            truth.verify_stage_geometry(stage, SPEC)

    def test_wrong_plane_height_and_stage_units_rejected(self):
        from pxr import Gf, UsdGeom
        stage = self.stage()
        stage.GetPrimAtPath("/World/GroundPlane").GetAttribute("xformOp:translate").Set(Gf.Vec3f(0, 0, .01))
        with self.assertRaisesRegex(ValueError, "floor_plane_mismatch"):
            truth.verify_stage_geometry(stage, SPEC)
        stage = self.stage()
        UsdGeom.SetStageMetersPerUnit(stage, .01)
        with self.assertRaisesRegex(ValueError, "unsupported_stage_units"):
            truth.verify_stage_geometry(stage, SPEC)

    def test_external_robot_import_layer_permitted_but_static_layer_rejected(self):
        from pxr import Usd, UsdGeom, UsdPhysics
        with tempfile.TemporaryDirectory() as folder:
            asset = Path(folder) / "robot.usda"
            robot = Usd.Stage.CreateNew(str(asset))
            base = UsdGeom.Xform.Define(robot, "/Fixture").GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(base)
            for name in ("body", "left", "right", "front", "rear"):
                UsdPhysics.CollisionAPI.Apply(UsdGeom.Cube.Define(robot, "/Fixture/" + name).GetPrim())
            robot.GetRootLayer().Save()
            stage = self.stage()
            stage.RemovePrim("/World/WheelFixture")
            UsdGeom.Xform.Define(stage, "/World/WheelFixture").GetPrim().GetReferences().AddReference(str(asset), "/Fixture")
            registered_spec = copy.deepcopy(SPEC)
            registered_spec["robot_collision_registry"] = truth.robot_registry_from_stage(stage, "/World/WheelFixture")
            truth.verify_stage_geometry(stage, registered_spec)
            # A static stage sublayer can silently alter or add environment
            # geometry. It must not inherit the robot import exception.
            static_asset = Path(folder) / "static.usda"
            external = Usd.Stage.CreateNew(str(static_asset))
            external.OverridePrim("/World/Indoor/west_cabinet")
            external.GetRootLayer().Save()
            stage.GetRootLayer().subLayerPaths.append(str(static_asset))
            with self.assertRaisesRegex(ValueError, "static_geometry_has_external_dependencies"):
                truth.verify_stage_geometry(stage, registered_spec)


if __name__ == "__main__":
    unittest.main()
