"""Pure input/evidence regressions; synthetic records are not navigation runs."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import scenario_suite as suite
import world_builder as world
from collision_audit import Audit
from quadruped import pose_matrix


class ScenarioInputTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def prepare(self, case="long_distance_multi_goal"):
        target = self.root / "prepared.json"
        suite.prepare_case(world.DEFAULT_WORLD, case, target)
        return json.loads(target.read_text())

    def test_six_cases_have_metre_lengths_and_no_execution_claim(self):
        rows = suite.list_cases(world.load_world())
        self.assertEqual(len(rows), 6)
        long = next(r for r in rows if r["id"] == "long_distance_multi_goal")
        self.assertGreater(long["static_route_length_m"], 300.)
        self.assertEqual(long["timeout_source_s"], 3600)
        self.assertEqual(long["phase_timeout_source_s"], [618, 1062, 651, 819, 782])
        self.assertTrue(all(r["execution_status"] == "pending" for r in rows))
        self.assertTrue(all(p[2] == 0. for r in rows for p in r["ground_goals"]))

    def test_prepare_freezes_ground_goals_spawn_and_actor_selection(self):
        spec = self.prepare("temporary_door_block")
        contract = suite.verify_prepared(spec, "temporary_door_block")
        self.assertEqual(contract["goals"], [[20., -25., 0.]])
        self.assertEqual(spec["robot"]["initial_pose"][2], .80)
        self.assertEqual(spec["robot"]["body_reference_height"], .52)
        self.assertEqual(contract["enabled_actor_ids"], ["door_blocking_cart"])
        self.assertEqual([a["id"] for a in spec["dynamic_actors"] if a["enabled"]], ["door_blocking_cart"])
        self.assertEqual(contract["fixture_goal_annotations_xyz"], [[20, -25, .52]])
        self.assertTrue(contract["candidate_seal_required"])
        self.assertFalse(contract["runtime_pose_or_actor_switching_allowed"])
        self.assertEqual(contract["execution_status"], "pending")
        encounter = contract["actor_encounter_contract"]
        self.assertEqual(encounter["required_actor_ids"], ["door_blocking_cart"])
        self.assertEqual(encounter["near_center_distance_m"], 2.)
        self.assertEqual(encounter["maximum_source_sampling_gap_s"], .12)
        self.assertIn("collision_audit.py", contract["input_code_sha256"])

    def test_prepare_never_overwrites_or_accepts_modified_input(self):
        spec = self.prepare()
        with self.assertRaises(FileExistsError):
            suite.prepare_case(world.DEFAULT_WORLD, "cancel_and_park", self.root / "prepared.json")
        spec["dynamic_actors"][0]["enabled"] = False
        with self.assertRaisesRegex(ValueError, "configuration_changed"):
            suite.verify_prepared(spec)
        spec = json.loads((self.root / "prepared.json").read_text())
        spec["scenario_suite_contract"]["goals"][0][2] = .52
        with self.assertRaises(ValueError):
            suite.verify_prepared(spec)

    def test_multi_point_plan_stays_one_original_action_sequence(self):
        contract = suite.verify_prepared(self.prepare())
        self.assertEqual(len(contract["phases"]), 5)
        self.assertEqual([p["index"] for p in contract["phases"]], list(range(5)))
        self.assertEqual([p["goal"] for p in contract["phases"]], contract["goals"])
        self.assertTrue(all(p["smoke_case"] == "goal" for p in contract["phases"]))
        self.assertEqual(contract["actor_clock"], "original_simulation_source_time_since_plant_start; never_reset_between_goals")
        self.assertEqual(contract["navigation_command_limit_mps"], .15)
        self.assertTrue(all("watchdogs_and_motion_leases_unchanged" in p["budget_scope"] for p in contract["phases"]))
        self.assertEqual(contract["actor_encounter_contract"]["required_actor_ids"], [])

    def test_dynamic_case_requires_fresh_sealed_encounter_contract_and_exact_intended_actor(self):
        spec = self.prepare("crossing_blocker")
        del spec["scenario_suite_contract"]["actor_encounter_contract"]
        spec["scenario_suite_contract"]["configuration_sha256"] = suite.configuration_digest(spec)
        with self.assertRaisesRegex(ValueError, "fresh_prepare_required"):
            suite.verify_prepared(spec)
        spec = json.loads((self.root/"prepared.json").read_text())
        spec["scenario_suite_contract"]["actor_encounter_contract"]["required_actor_ids"] = []
        spec["scenario_suite_contract"]["configuration_sha256"] = suite.configuration_digest(spec)
        with self.assertRaisesRegex(ValueError, "intended_encounter_actor_ids_changed"):
            suite.verify_prepared(spec)

    def test_phase_budget_is_observation_only_and_resume_uses_fixed_event_times(self):
        self.assertEqual(suite.phase_source_budget(31.38477631085, .15), 435)
        with self.assertRaises(ValueError):
            suite.phase_source_budget(31., 0.)
        spec = self.prepare("resume_after_block")
        phases = suite.verify_prepared(spec)["phases"]
        self.assertEqual(phases[0]["cancel_source_time_s"], 65)
        self.assertEqual(phases[0]["timeout_source_s"], 95.)
        self.assertEqual(phases[1]["submit_after_source_s"], 110)
        self.assertEqual(phases[1]["timeout_source_s"], 435)

    def test_run_plan_rejects_unprepared_candidate_or_reused_session(self):
        spec = self.prepare()
        candidate = self.root / "candidate"
        (candidate / "simulation/assets").mkdir(parents=True)
        (candidate / "simulation/assets/scene_config.json").write_text(json.dumps(spec))
        (candidate / "simulation/run.py").write_text("# --scenario-case\n")
        command = suite.run_command(candidate, self.root / "new_run", "long_distance_multi_goal")
        self.assertIn("--headless", command)
        self.assertIn("--scenario-case", command)
        self.assertEqual(command.count("--session"), 1)
        (self.root / "new_run").mkdir()
        with self.assertRaises(FileExistsError):
            suite.run_command(candidate, self.root / "new_run", "long_distance_multi_goal")
        with self.assertRaises(ValueError):
            suite.run_command(candidate, self.root / "other_run", "cancel_and_park")


class EvidenceEvaluationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.config = self.root / "prepared.json"
        suite.prepare_case(world.DEFAULT_WORLD, "crossing_blocker", self.config)
        self.spec = json.loads(self.config.read_text())
        # Unit-only actual-geometry registry; this is never an official Spot
        # acceptance experiment. Candidate preparation adds its real registry.
        self.spec["robot_collision_registry"] = dict(root="/World/Spot", colliders=[dict(
            path="/World/Spot/body/collision", type="Sphere", local_geometry=dict(radius=.1), world_scale=[1.,1.,1.])])
        self.registry_sha = hashlib.sha256(suite._canonical(self.spec["robot_collision_registry"])).hexdigest()
        self.spec["scenario_suite_contract"]["configuration_sha256"] = suite.configuration_digest(self.spec)
        self.config.write_text(json.dumps(self.spec))
        self.contract = suite.verify_prepared(self.spec)
        self.session = self.root / "unit_synthetic_session"
        (self.session / "scenario_suite").mkdir(parents=True)
        (self.session / "physics").mkdir()

    def tearDown(self):
        self.temp.cleanup()

    def write(self, relative, value):
        path = self.session / relative
        path.write_text(json.dumps(value))
        return path

    def complete_action_fixture(self):
        # Deliberately synthetic evidence, used only to test fail-closed review.
        session_id = "unit_synthetic_session"
        self.write("session.json", dict(id=session_id, simulation_backend="isaacsim_physx", transport_mode="isolated_mock",
            static_collision_prior_contract=dict(body_envelope_registry_sha256=self.registry_sha),
            isaac_bridge_contract=dict(scene_config=str(self.config), scene_sha256=suite._sha(self.config), clock_anchor_ns=1_000_000_000)))
        trace = self.session / "scenario_suite/phase_000_trace.jsonl"
        trace.write_text('{"unit_test_synthetic_trace":true}\n')
        report = dict(passed=True, case="goal", session_id=session_id, goal=[8., -5., 0.],
            result=dict(success=True, reason="goal_reached", retirement_confirmed=True, physical_stop_confirmed=False),
            action_result_status=4, physical_acceptance=False, measured_stationary=True, imu_source_matching=True,
            body_height_consistent=True, whole_body_attestation=True, error="", source_origin_ns=1_000_000_000,
            source_duration_s=self.contract["phases"][0]["timeout_source_s"],
            stop=dict(stop_submitted=True, measured_stop_confirmed=True, physical_acceptance_verified=False, execution_id="unit_task_1"),
            stationarity_evidence=dict(source_window_complete=True, source_span_s=2., window_s=2.),
            measured_final_position=[8., -5., .52], bt_status=dict(task_id="unit_task_1"),
            measured_samples=[dict(source_stamp_ns=t) for t in (2_000_000_000, 3_000_000_000, 4_000_000_000)],
            imu_evidence=dict(matched=True, state_count=3, missing_imu_source_stamps_ns=[]),
            events=[dict(key="goal_sent", source_stamp_ns=1_500_000_000), dict(key="BT_result", source_stamp_ns=4_000_000_000)])
        path = self.write("scenario_suite/phase_000_report.json", report)
        execution = dict(case_id="crossing_blocker", session_id=session_id, configuration_sha256=self.contract["configuration_sha256"],
            status="completed", phases=[dict(index=0, report=str(path.relative_to(self.session)), trace=str(trace.relative_to(self.session)),
                source_timeout_s=self.contract["phases"][0]["timeout_source_s"],
                report_sha256=suite._sha(path), trace_sha256=suite._sha(trace), returncode=0)])
        self.write("scenario_suite/execution.json", execution)
        self.write("run_summary.json", dict(clean_shutdown=True, navigation_shutdown=dict(session_id=session_id, request_accepted=True, software_retired=True)))
        return report, execution

    def collision_fixture(self, exercise_encounter=True):
        trajectory = self.session / "physics/trajectory.jsonl"
        audit = Audit(self.spec, "unit_synthetic_session", suite._sha(self.config), 1_000_000_000)
        times = [i*20_000_000 for i in range(201)]
        for ns in times:
            t = ns*1e-9
            distance = (3. if t <= 1. else 3.-1.5*(t-1.) if t <= 2. else 1.5 if t <= 2.5 else 1.5+3.*(t-2.5)) if exercise_encounter else 6.
            audit.sample(ns, [dict(path="/World/Spot/body/collision", shape=dict(type="Sphere", radius=.1),
                world_matrix=pose_matrix([0.,-5.,.52],[1.,0.,0.,0.]).tolist())],
                dict(plaza_person=dict(present=True, position=[distance,-5.,0.], orientation_xyzw=[0.,0.,0.,1.],
                                       source_stamp_ns=1_000_000_000+ns)))
        trajectory.write_text("".join(json.dumps(dict(sim_time_ns=t,pose=[0.,-5.,.52,0.,0.,0.,1.],unit_test_synthetic=True))+"\n" for t in times))
        return audit.finish(trajectory)

    def test_no_execution_or_only_action_success_cannot_pass(self):
        self.assertEqual(suite.evaluate_case(self.spec, self.session)["status"], "pending")
        self.complete_action_fixture()
        result = suite.evaluate_case(self.spec, self.session)
        self.assertEqual(result["status"], "pending")
        self.assertFalse(result["passed"])
        self.assertTrue(any(c["name"] == "actual_link_static_and_actor_collision" and c["status"] == "pending" for c in result["checks"]))

    def test_real_recorded_failure_is_failed_despite_missing_collision_audit(self):
        report, execution = self.complete_action_fixture()
        report.update(passed=False, error="smoke_wall_timeout")
        path = self.write("scenario_suite/phase_000_report.json", report)
        execution["phases"][0]["report_sha256"] = suite._sha(path)
        self.write("scenario_suite/execution.json", execution)
        self.assertEqual(suite.evaluate_case(self.spec, self.session)["status"], "failed")

    def test_recorded_phase_source_overrun_fails_without_relaxing_watchdogs(self):
        report, execution = self.complete_action_fixture()
        report["events"][-1]["source_stamp_ns"] = round((self.contract["phases"][0]["timeout_source_s"] + 2) * 1e9)
        path = self.write("scenario_suite/phase_000_report.json", report)
        execution["phases"][0]["report_sha256"] = suite._sha(path)
        self.write("scenario_suite/execution.json", execution)
        result = suite.evaluate_case(self.spec, self.session)
        self.assertEqual(result["status"], "failed")
        self.assertTrue(any(c["name"] == "phase_000_test_source_budget" and c["status"] == "failed" for c in result["checks"]))

    def test_changed_hash_foreign_session_or_negative_stop_never_passes(self):
        report, execution = self.complete_action_fixture()
        report["session_id"] = "foreign_session"
        path = self.write("scenario_suite/phase_000_report.json", report)
        # Hash mismatch is an actual evidence-integrity failure.
        self.assertEqual(suite.evaluate_case(self.spec, self.session)["status"], "failed")
        execution["phases"][0]["report_sha256"] = suite._sha(path)
        self.write("scenario_suite/execution.json", execution)
        self.assertNotEqual(suite.evaluate_case(self.spec, self.session)["status"], "passed")
        report["session_id"] = "unit_synthetic_session"
        report["stop"]["measured_stop_confirmed"] = False
        path = self.write("scenario_suite/phase_000_report.json", report)
        execution["phases"][0]["report_sha256"] = suite._sha(path)
        self.write("scenario_suite/execution.json", execution)
        self.assertEqual(suite.evaluate_case(self.spec, self.session)["status"], "failed")

    def test_collision_penetration_is_failure_not_a_missing_evidence_case(self):
        self.complete_action_fixture()
        collision = self.collision_fixture()
        collision["dynamic_penetration_count"] = 1
        self.write("physics/collision_audit.json", collision)
        self.assertEqual(suite.evaluate_case(self.spec, self.session)["status"], "failed")

    def test_old_wheel_audit_and_unexercised_blocker_are_not_full_pass(self):
        _, execution = self.complete_action_fixture()
        collision = self.collision_fixture()
        collision["robot_kind"] = "wheel_fixture"
        self.write("physics/collision_audit.json", collision)
        self.assertEqual(suite.evaluate_case(self.spec, self.session)["status"], "pending")
        execution["scenario_metrics"] = dict(actual_actor_encounter=dict(observed=False))
        self.write("scenario_suite/execution.json", execution)
        # A claimed outcome in execution.json is not the geometry producer.
        self.assertEqual(suite.evaluate_case(self.spec, self.session)["status"], "pending")
        self.write("physics/collision_audit.json", self.collision_fixture(exercise_encounter=False))
        self.assertEqual(suite.evaluate_case(self.spec, self.session)["status"], "failed")

    def test_collision_boolean_counts_or_foreign_robot_registry_cannot_pass(self):
        self.complete_action_fixture()
        collision = self.collision_fixture()
        collision["non_floor_static_penetration_count"] = False
        self.write("physics/collision_audit.json", collision)
        result = suite.evaluate_case(self.spec, self.session)
        self.assertTrue(any(c["name"] == "actual_link_static_and_actor_collision" and c["status"] == "pending" for c in result["checks"]))
        collision["non_floor_static_penetration_count"] = 0
        collision["robot_collision_registry_sha256"] = "foreign_model_registry"
        self.write("physics/collision_audit.json", collision)
        result = suite.evaluate_case(self.spec, self.session)
        self.assertTrue(any(c["name"] == "actual_link_static_and_actor_collision" and c["status"] == "pending" for c in result["checks"]))

    def test_complete_bound_synthetic_fixture_exercises_evaluator_pass_branch(self):
        _, execution = self.complete_action_fixture()
        self.write("physics/collision_audit.json", self.collision_fixture())
        execution["scenario_metrics"] = dict(actual_actor_encounter=dict(observed=True,
            source="actual_registered_link_and_actor_geometry", collision_audit_sha256=suite._sha(self.session / "physics/collision_audit.json")))
        self.write("scenario_suite/execution.json", execution)
        result = suite.evaluate_case(self.spec, self.session)
        self.assertEqual(result["status"], "passed")
        self.assertTrue(all(c["status"] == "passed" for c in result["checks"]))
        self.assertFalse(result["continuous_collision_certificate"])
        self.assertFalse(result["physical_robot_acceptance"])

    def test_execution_goal_claim_cannot_replace_producer_or_incomplete_actual_encounter(self):
        _, execution = self.complete_action_fixture()
        collision = self.collision_fixture()
        del collision["actual_actor_encounter"]
        self.write("physics/collision_audit.json", collision)
        execution["scenario_metrics"] = dict(actual_actor_encounter=dict(observed=True,
            source="actual_registered_link_and_actor_geometry", collision_audit_sha256=suite._sha(self.session/"physics/collision_audit.json")))
        self.write("scenario_suite/execution.json", execution)
        self.assertEqual(suite.evaluate_case(self.spec, self.session)["status"], "pending")
        collision = self.collision_fixture()
        collision["actual_actor_encounter"].update(completed=False,observed=None)
        self.write("physics/collision_audit.json", collision)
        self.assertEqual(suite.evaluate_case(self.spec, self.session)["status"], "pending")

    def test_changed_pair_id_distance_rule_freshness_or_source_cannot_pass(self):
        self.complete_action_fixture()
        good = self.collision_fixture()
        mutations = [lambda c: c["actual_actor_encounter"].update(required_actor_ids=["foreign"]),
            lambda c: c["actual_actor_encounter"].update(contract_sha256="0"*64),
            lambda c: c["actual_actor_encounter"].update(maximum_source_sampling_gap_ns=200_000_000),
            lambda c: c["actual_actor_encounter"]["actors"][0]["witnesses"][0]["departure_end"].update(robot_path="/World/Spot/foreign"),
            lambda c: c["actual_actor_encounter"]["actors"][0]["witnesses"][0]["near_begin"].update(center_distance_m=.01),
            lambda c: c["actual_actor_encounter"]["actors"][0]["witnesses"][0]["near_end"].update(source_stamp_ns=123)]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                collision = copy.deepcopy(good); mutate(collision)
                self.write("physics/collision_audit.json", collision)
                result = suite.evaluate_case(self.spec, self.session)
                self.assertEqual(result["status"], "failed")
                self.assertTrue(any(c["name"] == "intended_dynamic_encounter_observed" and c["status"] == "failed" for c in result["checks"]))

    def test_internally_consistent_shifted_anchor_cannot_replace_session_clock_anchor(self):
        self.complete_action_fixture()
        collision = self.collision_fixture()
        encounter = collision["actual_actor_encounter"]
        # All absolute source stamps and leg durations remain unchanged. The
        # altered sim times only fit the report's private invented anchor.
        encounter["source_anchor_ns"] += 1
        for actor in encounter["actors"]:
            for witness in actor["witnesses"]:
                for key in ("approach_start", "near_begin", "closest", "near_end", "departure_end"):
                    witness[key]["sim_time_ns"] -= 1
        self.write("physics/collision_audit.json", collision)
        result = suite.evaluate_case(self.spec, self.session)
        self.assertEqual(result["status"], "failed")
        check = next(c for c in result["checks"] if c["name"] == "intended_dynamic_encounter_observed")
        self.assertIn("authoritative_session_clock_anchor_mismatch", check["reason"])


if __name__ == "__main__":
    unittest.main()
