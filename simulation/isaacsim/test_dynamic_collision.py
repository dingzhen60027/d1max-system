"""Pure offline regressions for bounded actor truth and full body enclosure."""
import copy
import math
import unittest
from dynamic_collision import actor_registry, oracle_payload, shape_radius, voxel_veto, region_voxel_intersects, certify_robot_in_body_envelope


ACTOR = dict(id="person", enabled=True, max_linear_speed_mps=1., max_linear_acceleration_mps2=2.,
    collision_shapes=[dict(id="torso", type="box", center=[0., 0., .9], size=[.5, .5, 1.8], collision=True)])


def box(lower, upper, path="body", **extra):
    return dict(path=path, min=lower, max=upper, **extra)


class DynamicCollisionTests(unittest.TestCase):
    def payload(self, samples=None):
        registry = actor_registry(dict(dynamic_actors=[ACTOR]))
        samples = samples if samples is not None else dict(person=dict(present=True, source_stamp_ns=1_000_000_000,
            position=[0., 0., 0.], linear_velocity=[1., 0., 0.]))
        return oracle_payload(registry, samples, session_id="session", epoch=1, seed_id="seed", context_sequence=2,
            sequence=3, source_stamp_ns=1_000_000_000)

    def test_hidden_actor_veto_reaches_static_free_without_laser(self):
        payload = self.payload()
        self.assertEqual(voxel_veto(payload, [5, 0, 0]), 2)
        self.assertEqual(voxel_veto(payload, [100, 100, 0]), 0)
        self.assertIn("not_lidar", payload["source_scope"])
        self.assertEqual(payload["valid_until_ns"], 1_300_000_000)

    def test_explicit_empty_actor_set_emits_complete_same_source_heartbeat(self):
        payload = oracle_payload([], {}, session_id="session", epoch=1, seed_id="seed", context_sequence=2,
            sequence=3, source_stamp_ns=1_000_000_000, reachable_horizon_ns=6_000_000_000)
        self.assertEqual(payload["actors"], []);self.assertTrue(payload["complete"])
        self.assertEqual(payload["reachable_until_ns"], 7_000_000_000)
        self.assertEqual(len(payload["registry_sha256"]), 64)
        with self.assertRaisesRegex(ValueError, "reachable_horizon"):
            oracle_payload([], {}, session_id="session", epoch=1, seed_id="seed", context_sequence=2,
                sequence=3, source_stamp_ns=1_000_000_000, reachable_horizon_ns=0)

    def test_rotation_and_short_future_motion_enclose_whole_shape(self):
        payload = self.payload()
        region = payload["actors"][0]["regions"][0]
        shape = actor_registry(dict(dynamic_actors=[ACTOR]))[0]["shapes"][0]
        radius = shape_radius(shape)
        self.assertAlmostEqual(region["max"][0], radius + .3 + .02)
        for yaw in [0., math.pi / 4, math.pi / 2, math.pi]:
            for x in [-.25, .25]:
                for y in [-.25, .25]:
                    for z in [0., 1.8]:
                        point = [.3 + x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw), z]
                        self.assertTrue(all(a <= p <= b for a, p, b in zip(region["min"], point, region["max"])))
                        self.assertLessEqual(math.sqrt(sum(p * p for p in point)), region["radius"])
                        self.assertEqual(voxel_veto(payload, [math.floor(p / .05) for p in point]), 2)

    def test_reachable_duration_covers_complete_reaction_and_stop_without_renewing_lease(self):
        from dynamic_collision import reachable_actor_region
        registry = actor_registry(dict(dynamic_actors=[ACTOR]));actor = registry[0]
        actor["max_linear_acceleration_mps2"] = .1
        region = reachable_actor_region(actor, [0., 0., 0.], [0., 0., 0.], 6_000_000_000)
        # Starting at rest, a=.1 gives 1.8 m over the sealed 6 second bound.
        self.assertAlmostEqual(region["max"][0], shape_radius(actor["shapes"][0]) + 1.8 + .02)
        samples = dict(person=dict(present=True, source_stamp_ns=1_000_000_000, position=[0., 0., 0.], linear_velocity=[0., 0., 0.]))
        payload = oracle_payload(registry, samples, session_id="session", epoch=1, seed_id="seed", context_sequence=2,
            sequence=3, source_stamp_ns=1_000_000_000, reachable_horizon_ns=6_000_000_000)
        self.assertEqual(payload["reachable_until_ns"], 7_000_000_000)
        self.assertEqual(payload["valid_until_ns"], 1_300_000_000) # Lease remains finite and separate.
        with self.assertRaises(ValueError):
            reachable_actor_region(actor, [0., 0., 0.], [0., 0., 0.], 8_000_000_001)

    def test_missing_newborn_stale_source_and_overspeed_fail_closed(self):
        for changed in [{}, dict(person=dict(present=False, source_stamp_ns=1_000_000_000)),
                        dict(person=dict(present=True, source_stamp_ns=999_999_999)),
                        dict(person=dict(present=True, source_stamp_ns=1_000_000_000, position=[0., 0., 0.], linear_velocity=[2., 0., 0.])),
                        dict(newborn=dict(present=True))]:
            with self.assertRaises(ValueError):
                self.payload(changed)
        actor = copy.deepcopy(ACTOR);actor["lifetime"] = [0., 1.]
        with self.assertRaisesRegex(ValueError, "birth_requires_new_session"):
            actor_registry(dict(dynamic_actors=[actor]))

    def test_closed_voxel_contact_is_not_skipped(self):
        packet = dict(actors=[dict(regions=[dict(state=2, min=[0., 0., 0.], max=[.1, .1, 1.])])])
        self.assertEqual(voxel_veto(packet, [-1, 0, 0]), 2)
        self.assertEqual(voxel_veto(packet, [2, 0, 0]), 2)

    def test_sphere_removes_only_unreachable_box_corners_and_keeps_full_xyz_contact(self):
        region = dict(state=2, min=[-1., -1., -1.], max=[1., 1., 1.],
                      enclosure="sphere_v1", center=[0., 0., 0.], radius=1.)
        packet = dict(actors=[dict(regions=[region])])
        for cell in [[20, 0, 0], [-21, 0, 0], [0, 0, 20], [0, 0, -21]]:
            self.assertEqual(voxel_veto(packet, cell), 2, cell)
        self.assertEqual(voxel_veto(packet, [19, 19, 19]), 0)
        self.assertEqual(voxel_veto(packet, [0, 0, 19]), 2)
        legacy = {key: value for key, value in region.items() if key not in ("enclosure", "center", "radius")}
        self.assertEqual(voxel_veto(dict(actors=[dict(regions=[legacy])]), [19, 19, 19]), 2)
        # Negative offset and a non-grid sphere boundary retain closed tangency.
        offset = [-43.123, -17.719, .387]
        shifted = dict(region, center=offset, min=[v - 1. for v in offset], max=[v + 1. for v in offset])
        self.assertTrue(region_voxel_intersects(shifted, [offset[0] + 1., offset[1], offset[2]],
            [offset[0] + 1.05, offset[1] + .05, offset[2] + .05]))

    def test_v31_false_corner_is_outside_original_complete_six_second_sphere(self):
        region = dict(state=2, center=[0., -12.808781623840332, 0.], radius=7.166098359546661,
                      enclosure="sphere_v1")
        region["min"] = [v - region["radius"] for v in region["center"]]
        region["max"] = [v + region["radius"] for v in region["center"]]
        packet = dict(actors=[dict(regions=[region])])
        self.assertEqual(voxel_veto(packet, [-144, -113, -1]), 0)
        self.assertEqual(voxel_veto(packet, [-120, -240, -1]), 2)
        self.assertEqual(voxel_veto(packet, [0, -257, 140]), 2) # Same sphere retains elevated future volume.
        self.assertEqual(voxel_veto(packet, [0, -257, 145]), 0)

    def test_declared_sphere_must_be_complete_and_exactly_match_its_outer_box(self):
        region = dict(state=2, min=[-1., -1., -1.], max=[1., 1., 1.],
                      enclosure="sphere_v1", center=[0., 0., 0.], radius=1.)
        for variant in range(11):
            changed = copy.deepcopy(region)
            if variant == 0: changed.pop("center")
            if variant == 1: changed.pop("radius")
            if variant == 2: changed["enclosure"] = None
            if variant == 3: changed["enclosure"] = "unproved_shape"
            if variant == 4: changed["radius"] = 0.
            if variant == 5: changed["radius"] = math.inf
            if variant == 6: changed["radius"] = True
            if variant == 7: changed["center"] = [0., 0.]
            if variant == 8: changed["max"][0] += .001
            if variant == 9: changed.pop("enclosure")
            if variant == 10: changed["center"][0] = math.nan
            with self.subTest(variant=variant), self.assertRaises((ValueError, KeyError)):
                # Validate malformed geometry even for a spatially distant query.
                region_voxel_intersects(changed, [100., 100., 100.], [101., 101., 101.])


class BodyEnvelopeTests(unittest.TestCase):
    def certify(self, items, **overrides):
        args = dict(body_position=[0., 0., .5], body_orientation_xyzw=[0., 0., 0., 1.],
            radius=.4, offset=.2, below=.52, above=.3, floor_z=0., support_paths=["foot"])
        args.update(overrides)
        return certify_robot_in_body_envelope(items, **args)

    def test_long_body_can_cross_both_cylinders(self):
        result = self.certify([box([-.55, -.15, .3], [.55, .15, .7])])
        self.assertTrue(result["valid"])

    def test_union_corner_only_test_misses_narrow_waist_but_clipped_proof_rejects(self):
        # Both end faces lie in their circles; x=0,y=.35 lies outside both.
        with self.assertRaisesRegex(ValueError, "horizontal_query_envelope"):
            self.certify([box([-.2, -.35, .3], [.2, .35, .7])])

    def test_support_contact_has_fixed_lower_bound_and_never_allows_deep_floor_penetration(self):
        self.assertTrue(self.certify([box([.1, -.04, -.019], [.2, .04, .2], "foot")])["valid"])
        with self.assertRaisesRegex(ValueError, "support_slab"):
            self.certify([box([.1, -.04, -.021], [.2, .04, .2], "foot")])
        with self.assertRaisesRegex(ValueError, "support_slab"):
            self.certify([box([.1, -.04, -.001], [.2, .04, .2], "body")])

    def test_actual_rotated_corners_prevent_world_aabb_artificial_yaw_inflation(self):
        yaw = math.pi / 4
        points = [[x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw), z]
            for x in [-.55, .55] for y in [-.15, .15] for z in [.3, .7]]
        lower = [min(p[d] for p in points) for d in range(3)];upper = [max(p[d] for p in points) for d in range(3)]
        orientation = [0., 0., math.sin(yaw / 2), math.cos(yaw / 2)]
        self.assertTrue(self.certify([box(lower, upper, corners_world=points)], body_orientation_xyzw=orientation)["valid"])
        with self.assertRaisesRegex(ValueError, "horizontal_query_envelope"):
            self.certify([box(lower, upper)], body_orientation_xyzw=orientation)

    def test_nominal_tilt_and_height_tolerance_cannot_authorize_outside_link(self):
        with self.assertRaisesRegex(ValueError, "vertical_query_envelope"):
            self.certify([box([-.1, -.1, .3], [.1, .1, .9])], body_orientation_xyzw=[.05, 0., 0., .998749])
        with self.assertRaisesRegex(ValueError, "horizontal_query_envelope"):
            self.certify([box([.6, -.1, .3], [.7, .1, .6])])


if __name__ == "__main__":
    unittest.main()
