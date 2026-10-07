"""Small offline evidence tests: no ROS participant, physics or task authority."""
import math
import unittest

from control_audit import ControlAudit, ExactSpline, identity, stamp_ns


VERSION = dict(session_id="s", task_id="t", route_id="r", route_hash="h", map_version_id="m",
               localization_epoch=1, localization_seed_id="seed", reference_generation=2,
               segment_id="floor", anchor_id="a", anchor_revision=1, context_sequence=3,
               map_geometry_revision=4)


def record(kind, data, source=10_000_000_001, receipt=20_000_000_000):
    return dict(kind=kind, data=data, source_clock_ns=source, receipt_monotonic_ns=receipt)


def spline_message():
    tagged = dict(VERSION, generation=VERSION["reference_generation"], frame_id="odom",
                  valid_start_time=0., join_source_stamp=dict(sec=10, nanosec=1))
    tagged.pop("reference_generation")
    tagged["trajectory"] = dict(order=3, traj_id=11, start_time=dict(sec=10, nanosec=1),
        knots=[float(i-3) for i in range(10)],
        pos_pts=[dict(x=i*.1, y=i*.2, z=.5+i*.3) for i in range(6)])
    return tagged


def body_progress(source=10_000_000_001):
    value = dict(VERSION, generation=2, trajectory_id=11, curve_time=.5, valid=True,
                 holding=False, reason="tracking", header=dict(frame_id="odom",
                    stamp=dict(sec=source//10**9, nanosec=source%10**9)),
                 pose=dict(position=dict(x=.15, y=.3, z=.95), orientation=dict(x=0.,y=0.,z=0.,w=1.)),
                 twist=dict(linear=dict(x=.1,y=.2,z=.3), angular=dict(x=0.,y=0.,z=0.)))
    value.pop("reference_generation")
    value.pop("map_geometry_revision") # Original TrackingProgress has no such field.
    return value


class ControlAuditTest(unittest.TestCase):
    def setUp(self):
        self.events=[]
        self.audit=ControlAudit(self.events.append, command_speed_limit=.15, command_yaw_limit=.3)

    def test_fixed_native_math_preserves_xyz_derivative_and_exact_left_knot(self):
        message=spline_message()["trajectory"]
        curve=ExactSpline(message["pos_pts"], message["knots"], 3)
        for time in [-1., 0., .5, 1., 2., 3., 9.]:
            expected=min(max(time, 0.), 3.)+1.
            for got, want in zip(curve.evaluate(time), [.1*expected,.2*expected,.5+.3*expected]):
                self.assertAlmostEqual(got,want,places=13)
            for got,want in zip(curve.derivative().evaluate(time), [.1,.2,.3]):
                self.assertAlmostEqual(got,want,places=13)
        self.assertAlmostEqual(curve.samples_at(.5)["full_xyz_speed_mps"],math.sqrt(.14))
        with self.assertRaisesRegex(ValueError,"native_strict"):
            ExactSpline(message["pos_pts"],[-3.,-2.,-1.,0.,0.,2.,3.,4.,5.,6.],3)

    def test_full_wire_wins_and_source_ns_is_never_receipt_or_float(self):
        self.audit.feed(record("native_spline",spline_message()))
        self.audit.feed(record("tracking_progress",dict(trajectory_id=999, wire=body_progress()),receipt=900_000_000_000))
        event=self.events[-1]
        self.assertEqual(event["identity"]["trajectory_id"],11)
        self.assertEqual(event["body_source_stamp_ns"],10_000_000_001)
        self.assertEqual(event["receipt_monotonic_ns"],900_000_000_000)
        for delta in event["curve_velocity_minus_body_xyz_mps"]:
            self.assertAlmostEqual(delta,0.,places=13)
        self.assertEqual(stamp_ns(dict(sec=1_791_302_666,nanosec=869_687_328)),1_791_302_666_869_687_328)

    def test_nonuniform_knot_derivative_and_exact_knot_uses_left_span(self):
        curve=ExactSpline([[0.,0.,0.],[1.,1.,1.],[3.,2.,4.]],[-1.,0.,1.,2.,3.],1)
        self.assertEqual(curve.derivative().evaluate(1.),[1.,1.,1.])
        self.assertEqual(curve.derivative().evaluate(1.+1e-12),[2.,1.,3.])
        nonuniform=ExactSpline([[0.,0.,0.],[2.,4.,6.],[5.,10.,15.]],[-1.,0.,2.,5.,6.],1)
        self.assertEqual(nonuniform.evaluate(3.),[3.,6.,9.])
        self.assertEqual(nonuniform.derivative().evaluate(3.),[1.,2.,3.])

    def test_spatial_preview_uses_complete_xyz_arc_and_leaves_curve_phase_as_input(self):
        curve=ExactSpline([[i*.1,i*.2,.5+i*.3] for i in range(6)],[float(i-3) for i in range(10)],3)
        self.assertEqual(curve.control_lookahead_time(.5,.15,.8,False),1.3)
        self.assertAlmostEqual(curve.control_lookahead_time(.5,.15,.8,True),.5+.12/math.sqrt(.14))
        for got,expected in zip(curve.evaluate(.5),[.15,.3,.95]):
            self.assertAlmostEqual(got,expected,places=14)

    def test_spatial_terminal_arc_plateau_keeps_original_endpoint_parameter(self):
        curve=ExactSpline([[x,0.,.481] for x in [-.1,0.,.1,.2,.3,.4,.4,.4,.4,.4]],
                          [float(i-3) for i in range(14)],3)
        for time in [5.,6.,7.]:
            self.assertEqual(curve.control_lookahead_time(time,.15,.8,True),curve.duration)
            self.assertEqual(curve.evaluate(time),curve.evaluate(curve.duration))

    def test_geometry_install_and_grant_do_not_imply_writer_application(self):
        self.audit.feed(record("execution_spline",spline_message()))
        self.audit.feed(record("geometry_receipt",dict(version=VERSION,trajectory_id=11,installation_sequence=7,
            installed=True,installed_at=dict(sec=10,nanosec=1),reason="installed")))
        self.audit.feed(record("handoff_grant",dict(handoff_id="grant",candidate=dict(version=VERSION,trajectory_id=11))))
        self.assertIsNone(self.audit.writer_active)
        self.audit.feed(record("commit_ack",dict(applied=True,handoff_id="grant",reason="writer_applied")))
        self.assertIsNone(self.audit.writer_active)
        self.assertEqual(self.events[-1]["pending_evidence"],["full_writer_ack_identity"])
        self.audit.feed(record("commit_ack",dict(applied=True,candidate_version=VERSION,candidate_trajectory_id=11,
            curve_time=.5, commit_sequence=8,write_submitted=True,write_acknowledged=False,reason="uncertain")))
        self.assertEqual(self.audit.writer_active,identity(VERSION,11))
        self.assertEqual(self.events[-1]["scope"],"irreversible_writer_software_selection_not_physical_motion_proof")
        self.assertIn("candidate_at_writer_curve_time",self.events[-1])

    def test_missing_v31_curve_stays_pending_and_never_synthesizes_controls(self):
        self.audit.feed(record("tracking_progress",body_progress()))
        self.assertEqual(self.events[-1]["pending_evidence"],["complete_unambiguous_curve"])
        self.assertEqual(self.audit.summary()["complete_curve_identities"],0)

    def test_late_exact_body_resolves_original_source_and_reverse_heading_without_erasing_z(self):
        self.audit.feed(record("execution_spline",spline_message()))
        data=dict(trajectory_id=11,reason="tracking",control_diagnostic=dict(geometry_available=True,
            installed_trajectory_start_ns=10_000_000_001,body_source_stamp_ns=10_000_000_001,
            projected_curve_time=.5,lookahead_curve_time=1.,curve_duration=3.,step_source_ns=10_000_000_001,
            lookahead_position=[-.2,0.,1.1],lookahead_velocity=[-.1,0.,.3]))
        self.audit.feed(record("tracker",data))
        self.assertIn("exact_original_body_source",self.events[-1]["pending_evidence"])
        self.audit.feed(record("tracking_progress",body_progress()))
        resolved=[e for e in self.events if e.get("resolved_from_later_receipt_same_source")][0]
        self.assertEqual(resolved["trace_line"],2)
        self.assertEqual(resolved["body_source_stamp_ns"],10_000_000_001)
        self.assertEqual(resolved["original_heading_gate"],"outside")
        self.assertEqual(resolved["recorded_lookahead_velocity_xyz_mps"][2],.3)
        self.assertNotEqual(resolved["lookahead_velocity_evaluation_residual_xyz"][0],0.)
        self.assertEqual(self.audit.summary()["unresolved_original_body_diagnostics"],0)

    def test_commands_and_reason_are_observations_not_hold_failures(self):
        self.audit.feed(record("applied",dict(velocity=[.1,0.],source_stamp_ns=10,reason="tracking")))
        self.audit.feed(record("applied",dict(velocity=[0.,0.],source_stamp_ns=11,
            reason="motion_sweep_unknown",hold=True)))
        self.assertEqual(self.events[-1]["flags"],["positive_forward_to_exact_zero"])
        self.assertEqual(self.events[-1]["reason"],"motion_sweep_unknown")
        self.audit.feed(record("applied",dict(velocity=[.31,.51],reason="recorded")))
        self.assertIn("native_model_command_speed_ceiling_exceeded",self.events[-1]["flags"])
        self.assertIn("configured_yaw_limit_exceeded",self.events[-1]["flags"])
        self.assertNotIn("acceptance",self.audit.summary())

    def test_flattened_route_frame_is_not_mislabeled_as_odom_body_frame(self):
        self.audit.feed(record("tracking_progress",body_progress()))
        self.audit.feed(record("route_progress",dict(frame_id="map",body_source_stamp_ns=10_000_000_001,
            position=[.15,.3,.95],orientation_xyzw=[0.,0.,0.,1.])))
        self.assertEqual(self.audit.bodies[10_000_000_001]["frame"],"odom")
        self.assertEqual(self.audit.summary()["conflicting_original_body_source_count"],0)

    def test_same_identity_cannot_silently_change_original_geometry_clock_or_frame(self):
        self.audit.feed(record("native_spline",spline_message()))
        message=spline_message()
        message["trajectory"]["start_time"]["nanosec"]=2
        self.audit.feed(record("execution_spline",message))
        self.assertEqual(self.events[-1]["kind"],"conflicting_curve_identity")
        self.audit.feed(record("tracking_progress",body_progress()))
        self.assertEqual(self.events[-1]["pending_evidence"],["complete_unambiguous_curve"])

    def test_gait_path_distance_does_not_invent_owner_route_credits_or_net_forward_progress(self):
        route=dict(frame_id="odom",reference_generation=2,segment_id="floor",anchor_id="a",anchor_revision=1,
            context_sequence=3,map_geometry_revision=4,map_from_odom_position=[0.,0.,0.],
            map_from_odom_orientation_xyzw=[0.,0.,0.,1.],orientation_xyzw=[0.,0.,0.,1.],confirmed_arc_m=0.)
        for source,x in [(100,0.),(200,.1),(300,0.)]:
            self.audit.feed(record("route_progress",dict(route,body_source_stamp_ns=source,position=[x,0.,.5])))
        self.audit.feed(record("bt_progress",dict(execution_id="e",control_epoch=1,
            execution_progress_credit_samples=0,execution_progress_reason="waiting_measured_motion_progress")))
        self.audit.feed(record("bt_progress",dict(execution_id="e",control_epoch=1,
            execution_progress_credit_samples=1,execution_progress_body_source_ns=300)))
        self.assertEqual(self.events[-1]["observed_credit_increment"],1)
        summary=self.audit.summary()
        self.assertAlmostEqual(summary["fixed_route_domains"][0]["cumulative_xy_m"],.2)
        self.assertEqual(summary["fixed_route_domains"][0]["net_xy_m"],0.)
        self.assertEqual(summary["recorded_owner_credit_counts"][0]["count"],1)


if __name__ == "__main__":
    unittest.main()
