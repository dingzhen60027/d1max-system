"""Pure offline regressions for bounded actor truth and full body enclosure."""
import copy
import math
import unittest
from dynamic_collision import actor_registry, oracle_payload, shape_radius, voxel_veto, certify_robot_in_body_envelope


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
