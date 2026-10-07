"""Pure contract/shape tests: no Kit, ROS, policy inference or root pose writes."""
import json
import hashlib
import copy
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

import quadruped as q


class QuadrupedGeometryTests(unittest.TestCase):
    def self_filter_plant(self):
        # Pure stub contains only immutable exact collider identity. Physics
        # methods fail if an explicit acquisition snapshot rereads later state.
        plant = object.__new__(q.QuadrupedPlant)
        local = np.eye(4)
        local[:3, 3] = [.5, 0., 0.]
        plant._registry = [dict(path='/World/Spot/leg/collision',
            rigid_body_path='/World/Spot/leg', collision_enabled=True,
            shape=dict(type='Sphere', radius=.1), local_matrix=local.tolist())]
        plant._registry_digest = hashlib.sha256(q.canonical(plant._registry)).hexdigest()
        def forbidden_read():
            raise AssertionError('POSTphysics read during explicit BEGIN filter')
        plant.get_world_poses = forbidden_read
        plant.collider_snapshot = forbidden_read
        return plant

    def saved_self_snapshot(self, center):
        matrix = np.eye(4)
        matrix[:3, 3] = center
        return [dict(path='/World/Spot/leg/collision',rigid_body_path='/World/Spot/leg',
                     shape=dict(type='Sphere',radius=.1),world_matrix=matrix.tolist())]

    def test_same_acquisition_self_filter_uses_saved_body_and_moving_leg(self):
        plant = self.self_filter_plant()
        pre_q = [math.sqrt(.5), 0., 0., math.sqrt(.5)]
        points = np.array([[.5, 0., 0.], [1.5, 0., 0.]])
        unchanged = points.copy()
        pre_snapshot = self.saved_self_snapshot([10., 20.5, 1.])
        self.assertEqual(plant.external_hit_mask(points, measured_snapshot=pre_snapshot,
            body_position=[10., 20., 1.], body_orientation_wxyz=pre_q).tolist(), [False, True])
        # A post-step body and articulated leg moved relative to the body.
        # Same body points then classify differently. The explicit pre path
        # above must neither reread nor substitute these later transforms.
        post_snapshot = self.saved_self_snapshot([12., 21.5, 1.])
        self.assertEqual(plant.external_hit_mask(points, measured_snapshot=post_snapshot,
            body_position=[12., 20., 1.], body_orientation_wxyz=pre_q).tolist(), [True, False])
        np.testing.assert_array_equal(points, unchanged)

    def test_self_filter_legacy_current_measurement_api_is_preserved(self):
        plant = self.self_filter_plant()
        plant.get_world_poses = lambda: (np.array([[0.,0.,0.]]),np.array([[1.,0.,0.,0.]]))
        plant.collider_snapshot = lambda: self.saved_self_snapshot([.5,0.,0.])
        self.assertEqual(plant.external_hit_mask([[.5,0.,0.],[1.5,0.,0.]]).tolist(),[False,True])

    def test_saved_self_filter_rejects_partial_and_foreign_registry(self):
        plant = self.self_filter_plant()
        good = self.saved_self_snapshot([.5, 0., 0.])
        foreign = copy.deepcopy(good);foreign[0]['path']='/World/Spot/new_shape/collision'
        other_link = copy.deepcopy(good);other_link[0]['rigid_body_path']='/World/Spot/other_leg'
        changed_shape = copy.deepcopy(good);changed_shape[0]['shape']['radius']=.2
        changed_scale = copy.deepcopy(good);changed_scale[0]['world_matrix'][0][0]=2.
        reflected = copy.deepcopy(good);reflected[0]['world_matrix'][0][0]=-1.
        projective = copy.deepcopy(good);projective[0]['world_matrix'][3][0]=.1
        nonfinite = copy.deepcopy(good);nonfinite[0]['world_matrix'][0][3]=float('nan')
        for snapshot in [[], foreign, other_link, changed_shape, changed_scale, reflected, projective, nonfinite, good+good]:
            with self.subTest(snapshot=snapshot), self.assertRaises(ValueError):
                plant.external_hit_mask([[.5,0.,0.]], measured_snapshot=snapshot,
                    body_position=[0.,0.,0.], body_orientation_wxyz=[1.,0.,0.,0.])
        with self.assertRaises(ValueError):
            plant.external_hit_mask([[.5,0.,0.]], measured_snapshot=good)
        with self.assertRaises(ValueError):
            plant.external_hit_mask([[.5,0.,0.]], body_position=[0.,0.,0.],body_orientation_wxyz=[1.,0.,0.,0.])
        plant._registry[0]['shape']['radius']=.2
        with self.assertRaises(ValueError):
            plant.external_hit_mask([[.5,0.,0.]], measured_snapshot=good,
                body_position=[0.,0.,0.],body_orientation_wxyz=[1.,0.,0.,0.])

    def test_actual_authored_spot_foot_low_spawn_is_rejected_before_physics(self):
        # Official USD foot center is -0.6569999456 below the body with q=0;
        # initial joint positions are set only after the physics view is ready.
        shape = dict(type='Sphere', radius=.03500000014901161)
        def foot(spawn_z):
            matrix = np.eye(4)
            matrix[2, 3] = spawn_z - .656999945640564
            return dict(path='/World/Spot/fl_foot/collisions/mesh_0', shape=shape, world_matrix=matrix)
        with self.assertRaisesRegex(ValueError, 'initial_floor_penetration'):
            q.validate_initial_floor_clearance([foot(.6)], floor_z=0.)
        self.assertAlmostEqual(q.validate_initial_floor_clearance([foot(.8)], floor_z=0.), .10800005421042447)
        # A different floor elevation is checked as actual geometry, too.
        with self.assertRaisesRegex(ValueError, 'initial_floor_penetration'):
            q.validate_initial_floor_clearance([foot(.8)], floor_z=.2)

    def test_feedback_measured_yaw_bias_is_compensated_and_bounded(self):
        servo = q.VelocityFeedback()
        output = None
        for _ in range(500):
            output = servo.update(.02, [0., 0.], [0., -.04])
        self.assertGreater(output[1], .04)
        self.assertLessEqual(output[1], .5)
        servo.reset()
        for _ in range(1000):
            output = servo.update(.02, [.25, .4], [-5., -5.])
        np.testing.assert_allclose(output, [.3, .5])
        np.testing.assert_allclose(servo.integral, [0., 0.])

    def test_legacy_feedback_default_and_explicit_mode_are_unchanged(self):
        default = q.VelocityFeedback()
        explicit = q.VelocityFeedback(mode='legacy_pi_v1')
        for _ in range(20):
            np.testing.assert_array_equal(default.update(.02,[.01,.03],[0.,-.01]),
                                          explicit.update(.02,[.01,.03],[0.,-.01]))
        with self.assertRaises(ValueError):
            q.VelocityFeedback(mode='unsealed')

    def test_nonlinear_feedback_is_smooth_signed_and_preserves_desired(self):
        outputs = []
        for desired_value in [-.01,-1e-8,0.,1e-8,.01]:
            desired = np.array([desired_value,0.])
            unchanged = desired.copy()
            result = q.VelocityFeedback(mode='spot_nonlinear_measured_v1').update(.02,desired,[0.,0.])
            np.testing.assert_array_equal(desired,unchanged)
            outputs.append(result[0])
        self.assertAlmostEqual(outputs[0],-outputs[-1])
        self.assertAlmostEqual(outputs[1],-outputs[3])
        self.assertEqual(outputs[2],0.)
        self.assertLess(abs(outputs[3]),1e-6)  # no minimum policy speed at zero
        self.assertGreater(outputs[-1],.2)
        self.assertLess(outputs[-1],.3)

    def test_nonlinear_feedback_zero_withdraws_history_and_brakes_measured_motion(self):
        servo = q.VelocityFeedback(mode='spot_nonlinear_measured_v1')
        servo.integral[0] = .2
        np.testing.assert_array_equal(servo.update(.02,[0.,0.],[0.,0.]),[0.,0.])
        self.assertAlmostEqual(servo.integral[0],.2*math.exp(-.02/.5))
        moving = q.VelocityFeedback(mode='spot_nonlinear_measured_v1')
        moving.integral[0] = .2
        self.assertLess(moving.update(.02,[0.,0.],[.1,0.])[0],0.)
        # Actual error after a brief zero retains a signed low-speed correction.
        servo.integral[0] = -.1
        servo.update(.02,[0.,0.],[0.,0.])
        retained = servo.update(.02,[.00835,0.],[0.,0.])[0]
        fresh = q.VelocityFeedback(mode='spot_nonlinear_measured_v1').update(.02,[.00835,0.],[0.,0.])[0]
        self.assertLess(retained,fresh)

    def test_nonlinear_feedback_saturation_antiwindup_and_nonfinite_fail_closed(self):
        servo = q.VelocityFeedback(mode='spot_nonlinear_measured_v1')
        for _ in range(100):
            result = servo.update(.02,[.15,.3],[-5.,-5.])
            np.testing.assert_allclose(result,[.3,.5])
        np.testing.assert_array_equal(servo.integral,[0.,0.])
        for dt,desired,measured in [(.02,[.31,0.],[0.,0.]),(.02,[.01,0.],[float('nan'),0.]),(0.,[.01,0.],[0.,0.])]:
            with self.assertRaises(ValueError):
                servo.update(dt,desired,measured)

    def test_feedback_nonfinite_measurement_fails_closed(self):
        with self.assertRaises(ValueError):
            q.VelocityFeedback().update(.02, [0., 0.], [0., float('nan')])

    def test_monotone_candidate_mapping_has_bounded_gain_and_strict_zero(self):
        servo = q.VelocityFeedback(mode='spot_monotone_measured_v2')
        desired = np.linspace(-.15, .15, 301)
        outputs = np.asarray([servo.base_linear_policy_demand(v) for v in desired])
        self.assertTrue(np.all(np.diff(outputs) > 0.))
        np.testing.assert_allclose(np.diff(outputs) / np.diff(desired), 2., atol=1e-12)
        np.testing.assert_allclose(outputs, -outputs[::-1], atol=1e-15)
        self.assertEqual(servo.base_linear_policy_demand(0.), 0.)
        # Existing map's nonmonotone .02/.075 reversal must not recur.
        self.assertLess(servo.base_linear_policy_demand(.02), servo.base_linear_policy_demand(.075))
        self.assertLess(abs(servo.base_linear_policy_demand(1e-9)), 1e-8)

    def test_monotone_candidate_preserves_demand_zero_parking_and_antiwindup(self):
        servo = q.VelocityFeedback(mode='spot_monotone_measured_v2')
        desired = np.array([.075, .1])
        copy_desired = desired.copy()
        result = servo.update(.02, desired, [0., 0.])
        np.testing.assert_array_equal(desired, copy_desired)
        self.assertLess(result[0], .2)
        self.assertGreater(result[0], .15)
        servo.integral[0] = .2
        servo.filtered = None
        self.assertEqual(servo.update(.02, [0., 0.], [0., 0.])[0], 0.)
        self.assertLess(servo.integral[0], .2)
        servo.filtered = None
        self.assertLess(servo.update(.02, [0., 0.], [.1, 0.])[0], 0.)
        servo.reset()
        for _ in range(100):
            np.testing.assert_array_equal(servo.update(.02, [.15, .3], [-5., -5.]), [.3, .5])
        np.testing.assert_array_equal(servo.integral, [0., 0.])
        with self.assertRaises(ValueError):
            servo.update(.02, [.31, 0.], [0., 0.])

    def test_policy_joint_reading_is_same_source_finite_readonly_state(self):
        class TensorRead:
            def __init__(self, value): self.value = np.asarray(value)
            def detach(self): return self
            def cpu(self): return self
            def numpy(self): return self.value
        class ControllerRead:
            _policy_counter = 10
            _current_action = TensorRead(np.arange(12) * .1)
            _previous_action = TensorRead(np.arange(12) * .1)
        plant = object.__new__(q.QuadrupedPlant)
        plant.controller = ControllerRead()
        plant._steps = 10
        plant._last_inference_previous_action = [0.] * 12
        plant.get_dof_positions = lambda: np.ones((1, 12))
        plant.get_dof_velocities = lambda: np.zeros((1, 12))
        original = plant.controller._current_action.value.copy()
        result = plant.policy_joint_reading(20_000_000)
        self.assertEqual(result['source_sim_time_ns'], 20_000_000)
        self.assertEqual(result['joint_positions'], [1.] * 12)
        self.assertEqual(result['last_inference_previous_action'], [0.] * 12)
        np.testing.assert_array_equal(plant.controller._current_action.value, original)
        plant.get_dof_positions = lambda: np.ones((1, 11))
        with self.assertRaisesRegex(ValueError, 'invalid_actual_policy_joint_reading'):
            plant.policy_joint_reading(20_000_000)

    def test_capsule_half_sphere_spine_and_rotated_aabb(self):
        shape = dict(type='Capsule', axis='X', height=.2, radius=.02)
        self.assertEqual(q.primitive_contains([[.12, 0, 0], [.121, 0, 0], [.11, .02, 0]], shape, np.eye(4)).tolist(), [True, False, False])
        _, upper = q.primitive_aabb(shape, np.eye(4))
        np.testing.assert_allclose(upper, [.12, .02, .02])

    def test_policy_training_timing_and_commands_fail_closed(self):
        q.validate_step(.002, .3, -.5)
        for args in [(.008333333, 0, 0), (.002, float('nan'), 0), (.002, .3001, 0), (.002, 0, .51), (.002, True, 0)]:
            with self.assertRaises(ValueError):
                q.validate_step(*args)

    def test_quaternion_wxyz_measured_rotation(self):
        r = q.rotation_wxyz([math.sqrt(.5), 0, 0, math.sqrt(.5)])
        np.testing.assert_allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-14)
        with self.assertRaises(ValueError):
            q.rotation_wxyz([0, 0, 0, 0])

    def test_exact_cylinder_does_not_remove_capsule_ends(self):
        shape = dict(type='Cylinder', axis='Z', height=.12, radius=.012)
        points = np.array([[0, 0, .06], [0, 0, .061], [.013, 0, 0], [.012, 0, 0]])
        self.assertEqual(q.primitive_contains(points, shape, np.eye(4)).tolist(), [True, False, False, True])

    def test_scaled_rotated_box_and_aabb(self):
        transform = q.pose_matrix([1, 2, 3], [math.sqrt(.5), 0, 0, math.sqrt(.5)])
        transform[:3, :3] = transform[:3, :3] @ np.diag([.4, .1, .05])
        shape = dict(type='Cube', size=2.)
        lower, upper = q.primitive_aabb(shape, transform)
        np.testing.assert_allclose(lower, [.9, 1.6, 2.95])
        np.testing.assert_allclose(upper, [1.1, 2.4, 3.05])
        self.assertEqual(q.primitive_contains([[1, 2.3, 3], [1.11, 2, 3]], shape, transform).tolist(), [True, False])

    def test_rotated_cylinder_aabb_support(self):
        transform = q.pose_matrix([0, 0, 0], [math.sqrt(.5), 0, math.sqrt(.5), 0])
        lower, upper = q.primitive_aabb(dict(type='Cylinder', axis='Z', radius=.02, height=.2), transform)
        np.testing.assert_allclose(upper, [.1, .02, .02], atol=1e-14)
        np.testing.assert_allclose(lower, -upper)

    def test_sphere_nonuniform_scale_preserves_exact_membership(self):
        transform = np.diag([2., 1., .5, 1.])
        shape = dict(type='Sphere', radius=.1)
        self.assertEqual(q.primitive_contains([[.19, 0, 0], [0, .11, 0], [0, 0, .051]], shape, transform).tolist(), [True, False, False])
        lower, upper = q.primitive_aabb(shape, transform)
        np.testing.assert_allclose(upper, [.2, .1, .05])

    def test_primitive_filter_rejects_nonfinite(self):
        with self.assertRaises(ValueError):
            q.primitive_contains([[float('nan'), 0, 0]], dict(type='Sphere', radius=.1), np.eye(4))

    def test_asset_manifest_path_escape_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'asset_manifest.json'
            path.write_text(json.dumps(dict(schema=1, source='official_isaac_6_physx_go2', files=[dict(path='../bad', size_bytes=0, sha256='0'*64)])))
            with self.assertRaises(ValueError):
                q.verify_asset_manifest(path)


if __name__ == '__main__':
    unittest.main()
