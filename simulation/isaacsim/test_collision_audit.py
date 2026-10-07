"""Focused pure tests for measured primitive separation, without Kit/ROS."""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from collision_audit import (Audit, SCOPE, SPOT_EXPOSURE_PROFILE, SPOT_EXPOSURE_PROFILE_65,
    ENCOUNTER_HISTORY_PROFILE, _EncounterPairs, encounter_contract,
    review_actual_encounter, validate_encounter_contract)
from quadruped import pose_matrix


def spec(actor=True):
    return dict(robot=dict(kind="official_spot_physx"),
        robot_collision_registry=dict(root="/World/Spot", colliders=[dict(
            path="/World/Spot/body/collision", type="Sphere", local_geometry=dict(radius=.1), world_scale=[1.,1.,1.])]),
        room=dict(x_min=-10.,x_max=10.,y_min=-10.,y_max=10.),
        floor=dict(z=0.), ceiling=dict(z=3.,thickness=.1),
        static_boxes=[dict(name="wall", center=[5.,0.,1.],size=[.2,10.,2.])],
        dynamic_actors=[dict(id="person",enabled=True,max_linear_speed_mps=1.,max_linear_acceleration_mps2=1.,
            collision_shapes=[dict(id="body",type="box",center=[.5,0.,.5],size=[.2,.8,1.],collision=True)])] if actor else [])


def snapshot(position=(0.,0.,.5)):
    return [dict(path="/World/Spot/body/collision",shape=dict(type="Sphere",radius=.1),
                 world_matrix=pose_matrix(position,[1.,0.,0.,0.]).tolist())]


def actors(position=(2.,0.,0.), q=(0.,0.,0.,1.), stamp=None):
    value=dict(present=True,position=list(position),orientation_xyzw=list(q))
    if stamp is not None:
        value["source_stamp_ns"]=stamp
    return dict(person=value)


class CollisionAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.path=Path(self.temp.name)/"trajectory.jsonl"
        self.anchor=10**18

    def tearDown(self):
        self.temp.cleanup()

    def audit(self, value=None):
        return Audit(value or spec(),"test_session","a"*64,self.anchor)

    def trajectory(self,times):
        self.path.write_text("".join(json.dumps(dict(sim_time_ns=t,pose=[0.,0.,.5,0.,0.,0.,1.]))+"\n" for t in times))

    def test_complete_disjoint_and_integer_nanosecond_source(self):
        a=self.audit()
        for t in [0,1]:
            a.sample(t,snapshot(),actors(stamp=self.anchor+t))
        self.trajectory([0,1])
        report=a.finish(self.path)
        self.assertTrue(report["completed"])
        self.assertEqual(report["scope"],SCOPE)
        self.assertEqual(report["source_end_ns"],10**18+1)
        self.assertEqual(report["dynamic_penetration_count"],0)
        self.assertEqual(report["non_floor_static_penetration_count"],0)
        self.assertAlmostEqual(report["min_actor_separation_lower_bound_m"],2.299998)
        self.assertEqual(report["checked_pair_counts"],dict(static=4,dynamic=2))

    def test_touching_static_is_unresolved_and_floor_contact_is_excluded(self):
        value=spec(False)
        value["static_boxes"][0].update(center=[1.,0.,.5],size=[.2,1.,1.])
        a=self.audit(value)
        a.sample(0,snapshot((.8,0.,.1)),{})
        self.trajectory([0])
        report=a.finish(self.path)
        self.assertIsNone(report["non_floor_static_penetration_count"])
        self.assertEqual(report["possible_overlap_counts"]["static"],1)
        self.assertEqual(report["dynamic_penetration_count"],0)
        b=self.audit(spec(False)); b.sample(0,snapshot((0.,0.,.1)),{})
        self.assertEqual(b.finish(self.path)["non_floor_static_penetration_count"],0)

    def test_rotated_actor_offset_and_shape_extents_are_measured(self):
        a=self.audit()
        bounds=a._actor_bounds(0,actors((0.,0.,0.),(0.,0.,math.sqrt(.5),math.sqrt(.5))))[0]
        np.testing.assert_allclose(bounds[1],[-.400001,.399999,-.000001],atol=1e-10)
        np.testing.assert_allclose(bounds[2],[.400001,.600001,1.000001],atol=1e-10)
        a.sample(0,snapshot((0.,.5,.5)),actors((0.,0.,0.),(0.,0.,math.sqrt(.5),math.sqrt(.5))))
        self.trajectory([0]); report=a.finish(self.path)
        self.assertIsNone(report["dynamic_penetration_count"])
        self.assertEqual(report["possible_overlap_counts"]["dynamic"],1)

    def test_capsule_tight_support_bounds_and_nonuniform_scale(self):
        value=spec(False)
        value["robot_collision_registry"]["colliders"][0].update(type="Capsule",local_geometry=dict(radius=.1,height=.6,axis="X"),world_scale=[2.,1.,1.])
        a=self.audit(value)
        matrix=pose_matrix([0.,0.,1.],[math.sqrt(.5),0.,0.,math.sqrt(.5)])
        matrix[:3,:3]=matrix[:3,:3]@np.diag([2.,1.,1.])
        item=snapshot()[0];item.update(shape=dict(type="Capsule",radius=.1,height=.6,axis="X"),world_matrix=matrix.tolist())
        _,lower,upper=a._robot_bounds([item])[0]
        np.testing.assert_allclose(lower,[-.100001,-.800001,.899999],atol=1e-10)
        np.testing.assert_allclose(upper,[.100001,.800001,1.100001],atol=1e-10)

    def test_one_leg_overlap_cannot_be_hidden_by_safe_base(self):
        value=spec(False)
        leg=copy.deepcopy(value["robot_collision_registry"]["colliders"][0])
        leg["path"]="/World/Spot/foot/collision";value["robot_collision_registry"]["colliders"].append(leg)
        links=snapshot();foot=snapshot((5.,0.,1.))[0];foot["path"]=leg["path"];links.append(foot)
        a=self.audit(value);a.sample(0,links,{})
        self.trajectory([0]);report=a.finish(self.path)
        self.assertIsNone(report["non_floor_static_penetration_count"])
        self.assertEqual(report["possible_overlap_examples"][0]["robot_path"],leg["path"])

    def test_missing_actor_link_extra_identity_and_changed_shape_fail_closed(self):
        cases=[([],actors()),(snapshot(),{}),([dict(snapshot()[0],path="/World/Spot/new")],actors()),
               ([dict(snapshot()[0],shape=dict(type="Sphere",radius=.2))],actors())]
        self.trajectory([0])
        for robot, actor in cases:
            with self.subTest(robot=robot,actor=actor):
                a=self.audit()
                with self.assertRaises(ValueError):a.sample(0,robot,actor)
                report=a.finish(self.path)
                self.assertFalse(report["completed"])
                self.assertIsNone(report["dynamic_penetration_count"])
                self.assertIsNone(report["non_floor_static_penetration_count"])

    def test_nonfinite_matrix_clock_mismatch_and_duplicate_time_revoke_zero(self):
        a=self.audit();a.sample(0,snapshot(),actors())
        with self.assertRaises(ValueError):a.sample(0,snapshot(),actors())
        self.trajectory([0]);self.assertIsNone(a.finish(self.path)["dynamic_penetration_count"])
        a=self.audit()
        with self.assertRaises(ValueError):a.sample(1,snapshot(),actors(stamp=self.anchor))
        bad=snapshot();bad[0]["world_matrix"][0][0]=float("nan")
        with self.assertRaises(ValueError):self.audit().sample(0,bad,actors())

    def test_empty_partial_coverage_and_caller_incomplete_are_not_proof(self):
        self.trajectory([]);a=self.audit();report=a.finish(self.path)
        self.assertFalse(report["completed"]);self.assertIsNone(report["dynamic_penetration_count"])
        a.sample(0,snapshot(),actors());self.trajectory([0,1])
        self.assertFalse(a.finish(self.path)["completed"])
        self.trajectory([0]);self.assertFalse(a.finish(self.path,completed=False)["completed"])

    def test_raw_trajectory_hash_changes_with_bytes_and_ceiling_is_checked(self):
        a=self.audit();a.sample(0,snapshot((0.,0.,3.)),actors())
        self.trajectory([0]);before=a.finish(self.path)
        self.path.write_text(self.path.read_text()+"\n");after=a.finish(self.path)
        self.assertNotEqual(before["trajectory_sha256"],after["trajectory_sha256"])
        self.assertEqual(after["trajectory_sha256"],hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.assertIsNone(after["non_floor_static_penetration_count"])
        self.assertEqual(after["possible_overlap_examples"][0]["obstacle_path"],"/World/Indoor/ceiling")

    def encounter_spec(self):
        value = spec()
        value["scenario_suite_contract"] = dict(actor_encounter_contract=encounter_contract(["person"]))
        return value

    def measured_encounter(self, value=None, distance=None):
        value = value or self.encounter_spec()
        audit = self.audit(value)
        times = [i*100_000_000 for i in range(31)]
        for ns in times:
            t = ns*1e-9
            d = (3.-1.5*t if t <= 1. else 1.5 if t <= 2. else 1.5+1.5*(t-2.)) if distance is None else distance(t)
            audit.sample(ns, snapshot(), actors((d-.5, 0., 0.), stamp=self.anchor+ns))
        self.trajectory(times)
        return audit, value

    def test_encounter_producer_uses_actual_exact_pair_source_geometry(self):
        audit, value = self.measured_encounter()
        report = audit.finish(self.path)
        encounter = report["actual_actor_encounter"]
        self.assertTrue(encounter["completed"])
        self.assertTrue(encounter["observed"])
        self.assertEqual(encounter["required_actor_ids"], ["person"])
        observed, selected = review_actual_encounter(encounter, value, report, self.anchor, self.anchor+3_000_000_000)
        self.assertTrue(observed)
        self.assertEqual(len(selected), 1)
        witness = selected[0]
        self.assertEqual(witness["robot_path"], "/World/Spot/body/collision")
        self.assertEqual(witness["actor_shape_path"], "/World/Dynamic/person/body")
        for key in ("approach_start", "near_begin", "closest", "near_end", "departure_end"):
            point = witness[key]
            self.assertEqual(point["source_stamp_ns"], self.anchor+point["sim_time_ns"])
            self.assertAlmostEqual(point["center_distance_m"], math.dist(point["robot_center_m"], point["actor_center_m"]))
        self.assertGreaterEqual(witness["near_sample_count"], 6)
        self.assertFalse(report["continuous_collision_proof"])
        self.assertIn("not_route_blocking_or_yield_or_contact_evidence", encounter["limitations"])

    def spot_exposure_spec(self):
        value = self.encounter_spec()
        value["robot"].update(actor_encounter_profile=SPOT_EXPOSURE_PROFILE,
            body_size=[1.1,.6,.25], body_reference_height=.481,
            navigation_envelope=dict(length_m=1.25,width_m=.9,above_body_m=.589),
            body_reference_calibration=dict(scope="unit_test_metadata_not_official_calibration"),
            model_limits=dict(max_linear_speed_mps=.6,max_angular_speed_radps=.8),
            full_xyz_reference_model=dict(reference_max_speed_mps=.6,measured_travel_max_speed_mps=.6,
                evidence_sha256="a"*64))
        value["flat_support_contact"] = dict(enabled=True,floor_z=0.,penetration_allowance_m=.02)
        value["scenario_suite_contract"]["actor_encounter_contract"] = encounter_contract(["person"], value)
        return value

    def test_explicit_spot_exposure_uses_three_metres_without_reinterpreting_legacy(self):
        curve = lambda t: 4.-1.65*t if t <= 1. else 2.35 if t <= 2. else 2.35+1.65*(t-2.)
        legacy = self.encounter_spec()
        original_contract = copy.deepcopy(legacy["scenario_suite_contract"]["actor_encounter_contract"])
        audit, _ = self.measured_encounter(legacy, curve)
        self.assertIs(audit.finish(self.path)["actual_actor_encounter"]["observed"], False)
        value = self.spot_exposure_spec()
        contract = value["scenario_suite_contract"]["actor_encounter_contract"]
        self.assertEqual(contract["near_center_distance_m"], 3.)
        self.assertEqual({k:v for k,v in contract.items() if k not in ("profile","near_center_distance_m")},
                         {k:v for k,v in original_contract.items() if k != "near_center_distance_m"})
        self.assertAlmostEqual(contract["profile"]["body_envelope"]["radius"], .6078651750202781)
        audit, _ = self.measured_encounter(value, curve)
        report = audit.finish(self.path)
        observed, selected = review_actual_encounter(report["actual_actor_encounter"], value, report,
            self.anchor, self.anchor+3_000_000_000)
        self.assertTrue(observed)
        self.assertEqual(report["dynamic_penetration_count"], 0)
        self.assertGreater(selected[0]["closest"]["center_distance_m"], 2.)
        self.assertEqual(report["actual_actor_encounter"]["profile"], contract["profile"])
        self.assertEqual(legacy["scenario_suite_contract"]["actor_encounter_contract"], original_contract)
        with self.assertRaisesRegex(ValueError, "identity_or_contract_mismatch"):
            review_actual_encounter(report["actual_actor_encounter"], legacy, report,
                self.anchor, self.anchor+3_000_000_000)

    def test_new_profile_cannot_weaken_dwell_source_identity_or_unbind_metadata(self):
        good = self.spot_exposure_spec()
        for mutate in (
                lambda s: s["robot"].update(kind="official_go2_physx"),
                lambda s: s["robot"].update(actor_encounter_profile="automatic_near"),
                lambda s: s["robot"]["full_xyz_reference_model"].update(evidence_sha256="unsealed"),
                lambda s: s["robot"]["model_limits"].update(max_linear_speed_mps=.61),
                lambda s: s["robot"]["navigation_envelope"].update(width_m=.8),
                lambda s: s["scenario_suite_contract"]["actor_encounter_contract"].update(minimum_near_source_s=.1),
                lambda s: s["scenario_suite_contract"]["actor_encounter_contract"].pop("profile")):
            value = copy.deepcopy(good); mutate(value)
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_encounter_contract(value["scenario_suite_contract"]["actor_encounter_contract"], ["person"], value)

    def test_recalibrated_exposure_requires_new_exact_token_and_preserves_all_geometry_rules(self):
        old = self.spot_exposure_spec()
        value = copy.deepcopy(old)
        value['robot']['actor_encounter_profile'] = SPOT_EXPOSURE_PROFILE_65
        value['robot']['model_limits']['max_linear_speed_mps'] = .65
        value['robot']['full_xyz_reference_model'].update(reference_max_speed_mps=.65,
            measured_travel_max_speed_mps=.65)
        contract = encounter_contract(['person'], value)
        value['scenario_suite_contract']['actor_encounter_contract'] = contract
        validate_encounter_contract(contract, ['person'], value)
        prior = old['scenario_suite_contract']['actor_encounter_contract']
        self.assertEqual({k:v for k,v in prior.items() if k != 'profile'},
                         {k:v for k,v in contract.items() if k != 'profile'})
        profile = contract['profile']
        self.assertEqual(profile['kind'], SPOT_EXPOSURE_PROFILE_65)
        self.assertEqual([profile[k] for k in ('reference_max_speed_mps',
            'measured_travel_max_speed_mps', 'reachable_max_speed_mps')], [.65]*3)
        for key in ('body_envelope', 'body_envelope_metadata_sha256',
                    'source_reference_evidence_sha256'):
            self.assertEqual(profile[key], prior['profile'][key])
        for token, speed in ((SPOT_EXPOSURE_PROFILE, .65), (SPOT_EXPOSURE_PROFILE_65, .6),
                             ('isolated_spot_reachable66_3m_exposure_v1', .65)):
            bad = copy.deepcopy(value)
            bad['robot']['actor_encounter_profile'] = token
            bad['robot']['model_limits']['max_linear_speed_mps'] = speed
            bad['robot']['full_xyz_reference_model'].update(reference_max_speed_mps=speed,
                measured_travel_max_speed_mps=speed)
            with self.subTest(token=token), self.assertRaises(ValueError):
                encounter_contract(['person'], bad)
        for container, key in (('full_xyz_reference_model', 'reference_max_speed_mps'),
                ('full_xyz_reference_model', 'measured_travel_max_speed_mps'),
                ('model_limits', 'max_linear_speed_mps'), ('model_limits', 'max_angular_speed_radps')):
            bad = copy.deepcopy(value)
            bad['robot'][container][key] = .6
            with self.subTest(key=key), self.assertRaises(ValueError):
                encounter_contract(['person'], bad)

    def test_near_alone_approach_without_departure_or_far_actor_cannot_claim_encounter(self):
        curves = [lambda t: 1.5, lambda t: max(1.5, 3.-1.5*t), lambda t: 4.]
        for curve in curves:
            with self.subTest(curve=curve):
                audit, value = self.measured_encounter(distance=curve)
                report = audit.finish(self.path)
                self.assertIs(report["actual_actor_encounter"]["observed"], False)
                self.assertEqual(review_actual_encounter(report["actual_actor_encounter"], value, report,
                    self.anchor, self.anchor+3_000_000_000), (False, []))

    def test_encounter_time_gap_incomplete_fault_or_missing_trajectory_stays_pending(self):
        audit, value = self.measured_encounter()
        self.assertIsNone(audit.finish(self.path, completed=False)["actual_actor_encounter"]["observed"])
        audit.sample(3_200_000_000, snapshot(), actors())
        self.trajectory(audit.times)
        report = audit.finish(self.path)
        self.assertTrue(report["completed"])
        self.assertIsNone(report["actual_actor_encounter"]["observed"])
        self.assertEqual(review_actual_encounter(report["actual_actor_encounter"], value, report,
            self.anchor, self.anchor+3_000_000_000), (None, []))
        audit, _ = self.measured_encounter()
        with self.assertRaises(ValueError): audit.sample(3_100_000_000, snapshot(), {})
        self.assertIsNone(audit.finish(self.path)["actual_actor_encounter"]["observed"])
        self.path.unlink()
        self.assertIsNone(audit.finish(self.path)["actual_actor_encounter"]["observed"])

    def test_geometry_pair_switch_center_tamper_wrong_id_and_source_shift_are_rejected(self):
        audit, value = self.measured_encounter()
        good = audit.finish(self.path)
        for mutate in (
                lambda e: e["actors"][0]["witnesses"][0]["departure_end"].update(robot_path="/World/Spot/other"),
                lambda e: e["actors"][0]["witnesses"][0]["departure_end"].update(actor_shape_path="/World/Dynamic/other/body"),
                lambda e: e["actors"][0]["witnesses"][0]["near_begin"].update(center_distance_m=.01),
                lambda e: e["actors"][0]["witnesses"][0]["near_begin"].update(source_stamp_ns=self.anchor),
                lambda e: e.update(required_actor_ids=["other"]),
                lambda e: e.update(contract_sha256="0"*64),
                lambda e: e.update(observed=False),
                lambda e: e["actors"][0]["witnesses"][0].update(near_sample_count=2)):
            encounter = copy.deepcopy(good["actual_actor_encounter"]); mutate(encounter)
            with self.subTest(encounter=encounter), self.assertRaises(ValueError):
                review_actual_encounter(encounter, value, good, self.anchor, self.anchor+3_000_000_000)

    def test_encounter_before_action_is_not_evidence_for_that_action(self):
        audit, value = self.measured_encounter()
        # Extend the same measured session; a previous certificate cannot be
        # recycled for an Action that was submitted after the actor departed.
        for i in range(31, 61):
            audit.sample(i*100_000_000, snapshot(), actors((3.,0.,0.)))
        self.trajectory(audit.times)
        report = audit.finish(self.path)
        observed, selected = review_actual_encounter(report["actual_actor_encounter"], value, report,
            self.anchor+4_000_000_000, self.anchor+6_000_000_000)
        self.assertFalse(observed); self.assertEqual(selected, [])

    def test_resource_history_truncation_is_pending_even_after_valid_certificate(self):
        for prior_certificate in (False, True):
            with self.subTest(prior_certificate=prior_certificate):
                if prior_certificate:
                    audit, _ = self.measured_encounter()
                    self.assertTrue(audit.finish(self.path)["actual_actor_encounter"]["observed"])
                    first_ns = audit.times[-1]
                else:
                    audit = self.audit(self.encounter_spec()); first_ns = 0
                # 500 Hz actual source steps overflow the bounded 2048-point
                # history inside its normal 30s window. No geometry is missing.
                for i in range(2050):
                    audit.sample(first_ns+(i+1)*2_000_000, snapshot(), actors())
                self.trajectory(audit.times)
                report = audit.finish(self.path)
                encounter = report["actual_actor_encounter"]
                self.assertTrue(report["completed"])
                self.assertFalse(encounter["completed"])
                self.assertIsNone(encounter["observed"])
                self.assertIsNone(encounter["actors"][0]["observed"])
                self.assertEqual(encounter["truncated_actor_ids"], ["person"])
                if prior_certificate:
                    self.assertTrue(encounter["actors"][0]["witnesses"])

    def test_normal_source_window_history_expiry_is_not_resource_truncation(self):
        audit = self.audit(self.encounter_spec())
        for i in range(1551):
            audit.sample(i*20_000_000, snapshot(), actors())
        self.trajectory(audit.times)
        encounter = audit.finish(self.path)["actual_actor_encounter"]
        self.assertTrue(encounter["completed"])
        self.assertIs(encounter["observed"], False)
        self.assertEqual(encounter["truncated_actor_ids"], [])

    def cadence_history_spec(self):
        value = self.spot_exposure_spec()
        value["robot"]["actor_encounter_history_profile"] = ENCOUNTER_HISTORY_PROFILE
        value["physics"] = dict(frequency_hz=500., state_frequency_hz=50.)
        value["lidar"] = dict(frequency_hz=25.)
        value["scenario_suite_contract"]["actor_encounter_contract"] = encounter_contract(["person"], value)
        return value

    @staticmethod
    def actual_75hz_closed_window():
        # Two distinct actual source lattices; no interpolated snapshots.
        return sorted(set(range(0, 30_000_000_001, 20_000_000))
            | set(range(18_000_000, 30_000_000_001, 40_000_000)))

    def test_history_cadence_binding_is_exact_bounded_and_opt_out_is_unchanged(self):
        original = self.spot_exposure_spec()
        before = json.dumps(original["scenario_suite_contract"]["actor_encounter_contract"], sort_keys=True)
        value = self.cadence_history_spec()
        contract = value["scenario_suite_contract"]["actor_encounter_contract"]
        binding = contract["history_capacity_binding"]
        self.assertEqual(contract["maximum_history_samples_per_pair"], 2254)
        self.assertEqual(binding["maximum_actual_snapshot_rate_hz"], 75.)
        self.assertEqual(binding["derived_history_samples_per_pair"], 2254)
        self.assertEqual(binding["boundary_allowance_samples"], 4)
        validate_encounter_contract(contract, ["person"], value)
        original["physics"], original["lidar"] = value["physics"], value["lidar"]
        self.assertEqual(json.dumps(encounter_contract(["person"], original), sort_keys=True), before)
        legacy_fields = copy.deepcopy(contract)
        legacy_fields.pop("history_capacity_binding")
        legacy_fields["maximum_history_samples_per_pair"] = 2048
        self.assertEqual(legacy_fields, original["scenario_suite_contract"]["actor_encounter_contract"])

    def test_same_75hz_window_overflows_legacy_but_fits_explicit_cadence_history(self):
        times = self.actual_75hz_closed_window()
        self.assertEqual(len(times), 2251)
        for opted_in in (False, True):
            value = self.cadence_history_spec()
            if not opted_in:
                value["robot"].pop("actor_encounter_history_profile")
                value["scenario_suite_contract"]["actor_encounter_contract"] = encounter_contract(["person"], value)
            audit = self.audit(value)
            for ns in times:
                audit.sample(ns, snapshot(), actors((4.5, 0., 0.), stamp=self.anchor+ns))
            self.trajectory(times)
            report = audit.finish(self.path)
            encounter = report["actual_actor_encounter"]
            self.assertTrue(report["completed"])
            self.assertEqual(report["trajectory_sample_count"], len(times))
            self.assertEqual(encounter["completed"], opted_in)
            self.assertEqual(encounter["truncated_actor_ids"], [] if opted_in else ["person"])
            self.assertIs(encounter["observed"], False if opted_in else None)

    def test_history_capacity_change_preserves_original_three_leg_encounter_rule(self):
        value = self.cadence_history_spec()
        audit = self.audit(value)
        times = self.actual_75hz_closed_window()+list(range(30_020_000_000, 33_000_000_001, 20_000_000))
        for ns in times:
            t = ns*1e-9
            d = 5. if t <= 30. else 5.-2.6*(t-30.) if t <= 31. else 2.4 if t <= 32. else 2.4+2.6*(t-32.)
            audit.sample(ns, snapshot(), actors((d-.5, 0., 0.), stamp=self.anchor+ns))
        self.trajectory(times)
        report = audit.finish(self.path)
        encounter = report["actual_actor_encounter"]
        self.assertTrue(encounter["completed"])
        self.assertTrue(encounter["observed"])
        self.assertEqual(encounter["truncated_actor_ids"], [])
        observed, selected = review_actual_encounter(encounter, value, report, self.anchor, self.anchor+33_000_000_000)
        self.assertTrue(observed)
        self.assertGreaterEqual(selected[0]["near_sample_count"], 26)
        self.assertGreaterEqual(selected[0]["closest"]["center_distance_m"], 2.)

    def test_history_profile_rejects_wrong_fixture_missing_exposure_or_bad_actual_cadence(self):
        good = self.cadence_history_spec()
        mutations = [lambda s: s["robot"].update(actor_encounter_history_profile="automatic_capacity"),
            lambda s: s["robot"].update(kind="official_go2_physx"),
            lambda s: s["robot"].pop("actor_encounter_profile"),
            lambda s: s.pop("physics"), lambda s: s.pop("lidar")]
        for container, key, bad in (("physics", "frequency_hz", 400.),
                ("physics", "frequency_hz", True), ("physics", "state_frequency_hz", 60.),
                ("lidar", "frequency_hz", 10.), ("lidar", "frequency_hz", float("nan")),
                ("lidar", "frequency_hz", float("inf")), ("lidar", "frequency_hz", "25")):
            mutations.append(lambda s, c=container, k=key, v=bad: s[c].update({k:v}))
        for mutate in mutations:
            value = copy.deepcopy(good); mutate(value)
            with self.subTest(value=value), self.assertRaises(ValueError):
                encounter_contract(["person"], value)

    def test_history_binding_capacity_and_cadence_metadata_cannot_be_forged(self):
        good = self.cadence_history_spec()
        for mutate in (lambda c: c.update(maximum_history_samples_per_pair=4096),
                lambda c: c["history_capacity_binding"].update(derived_history_samples_per_pair=4096),
                lambda c: c["history_capacity_binding"].update(boundary_allowance_samples=5),
                lambda c: c["history_capacity_binding"].update(state_frequency_hz=100.),
                lambda c: c["history_capacity_binding"].update(source_exposure_profile_sha256="0"*64),
                lambda c: c.pop("history_capacity_binding")):
            value = copy.deepcopy(good); mutate(value["scenario_suite_contract"]["actor_encounter_contract"])
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_encounter_contract(value["scenario_suite_contract"]["actor_encounter_contract"], ["person"], value)

    def test_new_capacity_actual_overflow_remains_pending_even_after_certificate(self):
        value = self.cadence_history_spec()
        audit, _ = self.measured_encounter(value, lambda t: 4.-1.65*t if t <= 1. else 2.35 if t <= 2. else 2.35+1.65*(t-2.))
        self.assertTrue(audit.finish(self.path)["actual_actor_encounter"]["observed"])
        first_ns = audit.times[-1]
        for i in range(2256):
            audit.sample(first_ns+(i+1)*2_000_000, snapshot(), actors((4.5, 0., 0.)))
        self.trajectory(audit.times)
        report = audit.finish(self.path)
        encounter = report["actual_actor_encounter"]
        self.assertTrue(report["completed"])
        self.assertFalse(encounter["completed"])
        self.assertIsNone(encounter["observed"])
        self.assertEqual(encounter["truncated_actor_ids"], ["person"])
        self.assertTrue(encounter["actors"][0]["witnesses"])

    def test_closed_window_endpoint_bound_does_not_require_phase_alignment(self):
        value = self.cadence_history_spec()
        contract = value["scenario_suite_contract"]["actor_encounter_contract"]
        tracker = _EncounterPairs(contract, self.anchor)
        # Worst-case independent closed lattices: 1501+751 observations; use
        # adjacent source instants at coinciding boundaries to retain both.
        times = sorted(list(range(0, 30_000_000_001, 20_000_000))
            + [t+1 for t in range(0, 30_000_000_000, 40_000_000)]+[30_000_000_001])
        self.assertEqual(len(times), 2252)
        for ns in times:
            tracker.observe("person", "/World/Spot/body/collision", "/World/Dynamic/person/body", ns,
                np.array([0., 0., .5]), np.array([5., 0., .5]))
        self.assertEqual(tracker.truncated_actor_ids, set())
        self.assertLessEqual(len(next(iter(tracker.pairs.values()))["history"]), 2254)

    def test_unregistered_intended_actor_and_mutated_sealed_threshold_fail_closed(self):
        value = self.encounter_spec()
        value["scenario_suite_contract"]["actor_encounter_contract"]["required_actor_ids"] = ["foreign"]
        with self.assertRaises(ValueError): self.audit(value)
        value = self.encounter_spec()
        value["scenario_suite_contract"]["actor_encounter_contract"]["near_center_distance_m"] = 500.
        with self.assertRaises(ValueError): self.audit(value)

    def test_different_pairs_cannot_supply_separate_legs_of_one_encounter(self):
        value = self.encounter_spec()
        other = copy.deepcopy(value["robot_collision_registry"]["colliders"][0])
        other["path"] = "/World/Spot/foot/collision"
        value["robot_collision_registry"]["colliders"].append(other)
        audit = self.audit(value)
        for i in range(31):
            t = i*.1
            actor_center = max(1.5, 3.-1.5*t)
            foot_distance = 1.5 if t <= 2. else 1.5+1.5*(t-2.)
            links = snapshot()
            foot = snapshot((actor_center+foot_distance,0.,.5))[0]
            foot["path"] = other["path"]; links.append(foot)
            audit.sample(i*100_000_000, links, actors((actor_center-.5,0.,0.)))
        self.trajectory(audit.times)
        self.assertIs(audit.finish(self.path)["actual_actor_encounter"]["observed"], False)


if __name__ == "__main__":
    unittest.main()
