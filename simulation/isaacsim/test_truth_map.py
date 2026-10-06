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

    def test_new_nonrobot_dynamic_obstacle_invalidates_static_free(self):
        from pxr import UsdGeom, UsdPhysics
        stage = self.stage()
        obstacle = UsdGeom.Cube.Define(stage, "/World/NewObstacle").GetPrim()
        UsdPhysics.CollisionAPI.Apply(obstacle)
        UsdPhysics.RigidBodyAPI.Apply(obstacle)
        with self.assertRaisesRegex(ValueError, "unlisted_static_collider"):
            truth.verify_stage_geometry(stage, SPEC)

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
            truth.verify_stage_geometry(stage, SPEC)
            # A static stage sublayer can silently alter or add environment
            # geometry. It must not inherit the robot import exception.
            static_asset = Path(folder) / "static.usda"
            external = Usd.Stage.CreateNew(str(static_asset))
            external.OverridePrim("/World/Indoor/west_cabinet")
            external.GetRootLayer().Save()
            stage.GetRootLayer().subLayerPaths.append(str(static_asset))
            with self.assertRaisesRegex(ValueError, "static_geometry_has_external_dependencies"):
                truth.verify_stage_geometry(stage, SPEC)


if __name__ == "__main__":
    unittest.main()
