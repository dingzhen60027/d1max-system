"""Offline world, actor timing, resource and actual USD collider regressions."""
import copy
import importlib.util
import math
import unittest

import world_builder as world

HAS_USD = importlib.util.find_spec("pxr") is not None


class CampusGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = world.load_world()

    def test_complete_enclosure_and_real_overhead_colliders(self):
        # This independent world module does not invent the loaded official
        # quadruped's runtime collider registry required by truth_map's seal.
        self.assertEqual(self.spec["floor"]["bounds"], [-50., 50., -40., 40.])
        self.assertEqual(self.spec["ceiling"]["z"], 3.4)
        self.assertEqual(len(world.static_primitives(self.spec)), len(self.spec["static_boxes"]) + 1)
        self.assertTrue(world.collision_names(self.spec, [33., -8., .52]))
        # Clearance is three dimensional: the door has a real overhead lintel.
        self.assertFalse(world.collision_names(self.spec, [12., -25., .52]))
        self.assertTrue(world.collision_names(self.spec, [12., -25., 2.2]))
        self.assertTrue(world.collision_names(self.spec, [-35., -11., .52]))
        self.assertTrue(world.collision_names(self.spec, [0., 0., 3.]))

    def test_missing_enclosure_and_invalid_geometry_fail_closed(self):
        bad = copy.deepcopy(self.spec)
        bad["static_boxes"] = [b for b in bad["static_boxes"] if b["name"] != "west_wall"]
        with self.assertRaisesRegex(ValueError, "missing_room_enclosure"):
            world.validate_world(bad)
        bad = copy.deepcopy(self.spec)
        bad["static_boxes"][0]["size"][0] = math.nan
        with self.assertRaises(ValueError):
            world.validate_world(bad)

    def test_all_scenarios_have_static_whole_body_routes(self):
        total = 0.
        for case in world.scenario_matrix(self.spec):
            start = case["start"][:3]
            for goal in case["goals"]:
                route = world.find_route(self.spec, start, goal)
                for a, b in zip(route, route[1:]):
                    # Check the segment, not just route vertices.
                    for u in (0., .25, .5, .75, 1.):
                        point = [x + u * (y - x) for x, y in zip(a, b)]
                        self.assertEqual(world.collision_names(self.spec, point), [], (case["id"], point))
                length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
                if case["id"] == "long_distance_multi_goal":
                    total += length
                start = goal
        self.assertGreater(total, 300.)

    def test_narrow_gate_has_alternate_route_when_physically_blocked(self):
        spec = copy.deepcopy(self.spec)
        spec["static_boxes"].append(dict(name="offline_blocked_gate", kind="obstacle", center=[12., -25., .7], size=[1.2, 1.4, 1.4]))
        route = world.find_route(spec, [5., -25., .52], [20., -25., .52])
        self.assertTrue(any(p[1] > -13.4 or p[1] < -38.5 for p in route))
        self.assertGreater(sum(math.dist(a, b) for a, b in zip(route, route[1:])), 30.)

    def test_route_rejects_colliding_goal_or_unbounded_grid(self):
        with self.assertRaisesRegex(ValueError, "route_endpoint_collision"):
            world.find_route(self.spec, [-8., -5., .52], [33., -8., .52])
        with self.assertRaisesRegex(ValueError, "offline_route_grid_too_large"):
            world.find_route(self.spec, [-8., -5., .52], [8., -5., .52], resolution=.05)

    def test_voxel_budget_is_bounded_without_allocating_grid(self):
        budget = world.voxel_budget(self.spec, .05)
        self.assertEqual(budget["shape"], [2010, 1610, 73])
        self.assertEqual(budget["total_cells"], 236235300)
        self.assertTrue(budget["fits_256_mib_raw"])
        self.assertFalse(budget["allocation_performed"])
        self.assertGreater(budget["estimated_uint8_plus_bool_plus_float32_bytes"], 1_000_000_000)
        self.assertLess(world.voxel_budget(self.spec, .1)["raw_uint8_bytes"], 32 * 2**20)
        self.assertEqual(self.spec["map_generation"]["pct_max_working_bytes"], 2_000_000_000)


class ActorTrajectoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = world.load_world()

    def test_c1_pose_is_deterministic_and_derivative_bounds_hold(self):
        for actor in self.spec["dynamic_actors"]:
            duration = actor["trajectory"]["waypoints"][-1]["time_s"]
            for i in range(251):
                t = actor["start_time_s"] + duration * i / 250
                pose = world.actor_pose(actor, t)
                self.assertEqual(pose, world.actor_pose(actor, t))
                self.assertLessEqual(math.sqrt(sum(v * v for v in pose["linear_velocity"])), actor["max_linear_speed_mps"] + 1e-8)
                self.assertLessEqual(math.sqrt(sum(v * v for v in pose["linear_acceleration"])), actor["max_linear_acceleration_mps2"] + 1e-8)
                self.assertLessEqual(abs(pose["yaw_rate"]), actor["max_yaw_rate_radps"] + 1e-8)
                epsilon = 1e-4
                before, after = world.actor_pose(actor, t - epsilon), world.actor_pose(actor, t + epsilon)
                estimated = [(b - a) / (2 * epsilon) for a, b in zip(before["position"], after["position"])]
                self.assertLess(math.dist(estimated, pose["linear_velocity"]), .001)

    def test_native_velocity_quantization_margin_is_explicit_and_sealed(self):
        for actor in self.spec["dynamic_actors"]:
            self.assertEqual(actor["native_physics_velocity_margin_mps"], .02)
            analytic = max(1.5 * math.dist(a["position"], b["position"]) / (b["time_s"]-a["time_s"])
                for a, b in zip(actor["trajectory"]["waypoints"], actor["trajectory"]["waypoints"][1:]))
            self.assertAlmostEqual(actor["analytic_max_linear_speed_mps"], analytic)
            self.assertAlmostEqual(actor["max_linear_speed_mps"], analytic + .02)
        actors = {a["id"]: a for a in self.spec["dynamic_actors"]}
        self.assertLess(.763893, actors["office_person"]["max_linear_speed_mps"])
        self.assertLess(.600815, actors["warehouse_forklift"]["max_linear_speed_mps"])
        bad = copy.deepcopy(self.spec)
        bad["dynamic_actors"][0]["max_linear_speed_mps"] = bad["dynamic_actors"][0]["analytic_max_linear_speed_mps"]
        with self.assertRaisesRegex(ValueError, "invalid_native_actor_velocity_margin"):
            world.validate_world(bad)

    def test_motion_window_parks_instead_of_hiding_or_teleporting(self):
        for actor in self.spec["dynamic_actors"]:
            enabled = dict(actor, enabled=True)
            before = world.actor_pose(enabled, actor["start_time_s"] - 10.)
            after = world.actor_pose(enabled, actor["end_time_s"] + 10.)
            self.assertTrue(before["active"])
            self.assertTrue(after["active"])
            self.assertFalse(before["motion_active"])
            self.assertFalse(after["motion_active"])
            self.assertEqual(before["linear_velocity"], [0., 0., 0.])
            self.assertEqual(after["linear_velocity"], [0., 0., 0.])
            for boundary in (actor["start_time_s"], actor["end_time_s"]):
                a, b = world.actor_pose(enabled, boundary - 1e-5), world.actor_pose(enabled, boundary + 1e-5)
                self.assertLess(math.dist(a["position"], b["position"]), 1e-6)
                self.assertLess(math.dist(a["linear_velocity"], b["linear_velocity"]), 1e-5)

    def test_all_motion_paths_keep_registered_shapes_out_of_static_solids(self):
        for actor in self.spec["dynamic_actors"]:
            duration = actor["trajectory"]["waypoints"][-1]["time_s"]
            for i in range(501):
                t = actor["start_time_s"] + duration * i / 500
                self.assertEqual(world.actor_static_collisions(self.spec, actor, t), [], (actor["id"], t))

    def test_exact_shape_registry_and_possible_bounds_cover_pose(self):
        for actor in self.spec["dynamic_actors"]:
            enabled = dict(actor, enabled=True)
            bounds = world.actor_possible_centers_bounds(enabled, 0., 10000.)
            self.assertTrue(world.actor_active_during(enabled, 0., 10000.))
            for t in (-1., 0., 11., 39., 79., 10000.):
                pose = world.actor_pose(enabled, t)
                self.assertTrue(all(bounds["min"][i] <= pose["position"][i] <= bounds["max"][i] for i in range(3)))
                paths = [s["path"] for s in world.actor_colliders(enabled, t)]
                self.assertEqual(paths, ["/World/Dynamic/" + actor["id"] + "/" + s["id"] for s in actor["collision_shapes"]])
            self.assertIsNone(world.actor_possible_centers_bounds(dict(actor, enabled=False), 0., 1.))
        with self.assertRaises(ValueError):
            world.actor_active_during(self.spec["dynamic_actors"][0], 10., 0.)

    def test_invalid_bound_loop_or_lifecycle_is_rejected(self):
        for mutation, expected in ((lambda a: a.update(max_linear_speed_mps=0.), "understated_actor_motion_bound"),
                                   (lambda a: a["trajectory"]["waypoints"][-1].update(position=[1., 2., 0.]), "discontinuous_actor_loop"),
                                   (lambda a: a.update(lifetime=[0., 30.]), "unsupported_actor_lifecycle")):
            bad = copy.deepcopy(self.spec)
            mutation(bad["dynamic_actors"][0])
            with self.assertRaisesRegex(ValueError, expected):
                world.validate_world(bad)

    def test_scenario_selection_is_fixed_and_does_not_mutate_baseline(self):
        selected = world.scenario_spec(self.spec, "temporary_door_block")
        self.assertEqual([a["id"] for a in selected["dynamic_actors"] if a["enabled"]], ["door_blocking_cart"])
        self.assertEqual(sum(a["enabled"] for a in self.spec["dynamic_actors"]), 4)
        blocker = next(a for a in selected["dynamic_actors"] if a["id"] == "door_blocking_cart")
        self.assertTrue(world.collision_names(selected, [12., -25., .52], actor_time=55.))
        self.assertFalse(world.collision_names(selected, [12., -25., .52], actor_time=100.))
        self.assertEqual(world.actor_pose(blocker, 55.)["linear_velocity"], [0., 0., 0.])
        self.assertEqual(world.actor_pose(blocker, 45.)["position"], [12., -25., 0.])
        self.assertEqual(world.actor_pose(blocker, 70.)["position"], [12., -25., 0.])


@unittest.skipUnless(HAS_USD, "OpenUSD is not installed in this offline interpreter")
class ActualUsdAuthoringTests(unittest.TestCase):
    def test_actual_stage_has_exact_colliders_visuals_cameras_and_motion(self):
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        spec = world.load_world()
        stage = Usd.Stage.CreateInMemory()
        registry = world.author_world(stage, spec)
        expected = set(registry["static_paths"] + registry["dynamic_paths"] + [registry["floor_path"]])
        actual = {str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)}
        self.assertEqual(actual, expected)
        material = spec["floor"]["physics_material"]
        floor_prim = stage.GetPrimAtPath(registry["floor_path"])
        self.assertEqual([str(p) for p in floor_prim.GetRelationship("material:binding:physics").GetTargets()], [material["path"]])
        material_api = UsdPhysics.MaterialAPI(stage.GetPrimAtPath(material["path"]))
        self.assertEqual(material_api.GetStaticFrictionAttr().Get(), 1.)
        self.assertEqual(material_api.GetDynamicFrictionAttr().Get(), 1.)
        self.assertEqual(material_api.GetRestitutionAttr().Get(), 0.)
        self.assertEqual(len(registry["dynamic_paths"]), 11)
        self.assertEqual(UsdGeom.Imageable(stage.GetPrimAtPath("/World/Indoor/ceiling")).GetVisibilityAttr().Get(), UsdGeom.Tokens.invisible)
        self.assertEqual(UsdGeom.GetStageUpAxis(stage), UsdGeom.Tokens.z)
        for path in registry["camera_paths"]:
            self.assertTrue(stage.GetPrimAtPath(path).IsA(UsdGeom.Camera))
        overview = spec["cameras"]["overview"]
        self.assertEqual(stage.GetPrimAtPath(overview["path"]).GetAttribute("xformOp:rotateZ").Get(), 180.)
        width = overview["horizontal_aperture"] / 10
        height_at_viewport_aspect = width / (1280 / 900)
        self.assertGreaterEqual(width, spec["room"]["width"] + 16)
        self.assertGreaterEqual(height_at_viewport_aspect, spec["room"]["depth"] + 16)
        for box in spec["static_boxes"]:
            prim = stage.GetPrimAtPath("/World/Indoor/" + box["name"])
            self.assertEqual(UsdGeom.Cube(prim).GetSizeAttr().Get(), 1.)
            scale = prim.GetAttribute("xformOp:scale").Get()
            self.assertLess(math.dist(scale, box["size"]), 1e-6)
        self.assertTrue(stage.GetPrimAtPath("/World/Decorations/Signs/plaza_sign").IsA(UsdGeom.Mesh))
        world.update_actors(stage, spec, 19.)
        actor = spec["dynamic_actors"][0]
        prim = stage.GetPrimAtPath("/World/Dynamic/" + actor["id"])
        self.assertEqual(prim.GetAttribute("xformOp:translate").Get(), Gf.Vec3d(*world.actor_pose(actor, 19.)["position"]))
        self.assertTrue(UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get())
        # Motion never changes the static collider registry or its geometry.
        self.assertEqual({str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)}, expected)


if __name__ == "__main__":
    unittest.main()
