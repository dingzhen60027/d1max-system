"""Pure input/evidence regressions; synthetic records are not navigation runs."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import scenario_suite as suite
import world_builder as world
from collision_audit import Audit, SPOT_EXPOSURE_PROFILE, SPOT_EXPOSURE_PROFILE_65, encounter_contract
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
        self.assertEqual(long["phase_timeout_source_s"], [446, 735, 467, 576, 552])
        self.assertTrue(all(r["execution_status"] == "pending" for r in rows))
        self.assertTrue(all(p[2] == 0. for r in rows for p in r["ground_goals"]))

    def test_prepare_freezes_ground_goals_spawn_and_actor_selection(self):
        spec = self.prepare("temporary_door_block")
        contract = suite.verify_prepared(spec, "temporary_door_block")
        self.assertEqual(contract["goals"], [[20., -25., 0.]])
        self.assertEqual(spec["robot"]["initial_pose"][2], .80)
        self.assertEqual(spec["robot"]["body_reference_height"], .481)
        # The calibrated standing reference must propagate through a fresh seal;
        # the real spawn and historical goal annotations remain distinct.
        envelope = spec["robot"]["offline_collision_envelope"]
        self.assertAlmostEqual(envelope["bottom_z_offset_m"] + .481, 0.)
        self.assertAlmostEqual(envelope["top_z_offset_m"] + .481, 1.07)
        self.assertEqual(spec["robot"]["navigation_envelope"]["above_body_m"],
                         envelope["top_z_offset_m"])
        self.assertEqual(contract["enabled_actor_ids"], ["door_blocking_cart"])
        self.assertEqual([a["id"] for a in spec["dynamic_actors"] if a["enabled"]], ["door_blocking_cart"])
        self.assertEqual(contract["fixture_goal_annotations_xyz"], [[20, -25, 0.]])
        self.assertTrue(contract["candidate_seal_required"])
        self.assertFalse(contract["runtime_pose_or_actor_switching_allowed"])
        self.assertEqual(contract["execution_status"], "pending")
        encounter = contract["actor_encounter_contract"]
        self.assertEqual(encounter["required_actor_ids"], ["door_blocking_cart"])
        self.assertEqual(encounter["near_center_distance_m"], 3.)
        self.assertEqual(encounter["maximum_source_sampling_gap_s"], .12)
        self.assertIn("collision_audit.py", contract["input_code_sha256"])

    def background_actor_source(self):
        original = world.load_world()
        case = next(c for c in original['scenarios'] if c['id'] == 'crossing_blocker')
        case['actor_ids'] = ['plaza_person']
        case['background_actor_ids'] = sorted(a['id'] for a in original['dynamic_actors']
            if a['id'] != 'plaza_person')
        return original, case

    def prepare_background_actor_case(self):
        original, _ = self.background_actor_source()
        source = self.root/'background_author.json'; source.write_text(json.dumps(original))
        target = self.root/'background_prepared.json'
        suite.prepare_case(source, 'crossing_blocker', target)
        return original, json.loads(target.read_text())

    def test_background_actors_preserve_all_six_physical_actors_but_require_only_selected_encounter(self):
        from dynamic_collision import actor_registry
        legacy = self.prepare('crossing_blocker')
        legacy_bytes = (self.root/'prepared.json').read_bytes()
        original, spec = self.prepare_background_actor_case()
        contract = suite.verify_prepared(spec)
        all_ids = sorted(a['id'] for a in original['dynamic_actors'])
        self.assertEqual(contract['enabled_actor_ids'], all_ids)
        self.assertEqual(contract['background_actor_ids'], [v for v in all_ids if v != 'plaza_person'])
        self.assertEqual(contract['actor_encounter_contract']['required_actor_ids'], ['plaza_person'])
        self.assertEqual(contract['actor_encounter_contract'], legacy['scenario_suite_contract']['actor_encounter_contract'])
        self.assertTrue(all(a['enabled'] for a in spec['dynamic_actors']))
        enabled_original = copy.deepcopy(original)
        for actor in enabled_original['dynamic_actors']:
            actor['enabled'] = True
        self.assertEqual(spec['dynamic_actors'], enabled_original['dynamic_actors'])
        self.assertEqual(actor_registry(spec), actor_registry(enabled_original))
        row = next(r for r in suite.list_cases(original) if r['id'] == 'crossing_blocker')
        self.assertEqual(row['enabled_actor_ids'], all_ids)
        self.assertEqual(row['actor_ids'], ['plaza_person'])
        self.assertEqual((self.root/'prepared.json').read_bytes(), legacy_bytes)
        self.assertNotIn('background_actor_ids', legacy['scenario_suite_contract'])
        self.assertNotIn('background_actor_ids', next(c for c in legacy['scenarios'] if c['id']=='crossing_blocker'))
        self.assertNotIn('background_actor_ids', suite.list_cases(world.load_world())[1])

    def test_invalid_background_author_identity_is_rejected_before_preparation(self):
        original, case = self.background_actor_source()
        for index, values in enumerate((None, 'office_person', [False], ['unknown'],
                ['office_person', 'office_person'], ['plaza_person'])):
            bad = copy.deepcopy(original)
            selected = next(c for c in bad['scenarios'] if c['id'] == 'crossing_blocker')
            selected['background_actor_ids'] = values
            source = self.root/f'bad_background_{index}.json';source.write_text(json.dumps(bad))
            target = self.root/f'bad_prepared_{index}.json'
            with self.subTest(values=values), self.assertRaises(ValueError):
                suite.prepare_case(source, 'crossing_blocker', target)
            self.assertFalse(target.exists())
        for required in (['plaza_person','plaza_person'], [1], 'plaza_person', None):
            invalid = copy.deepcopy(case); invalid['actor_ids'] = required
            with self.subTest(required=required), self.assertRaises(ValueError):
                suite.case_actor_scope(original, invalid)

    def test_sealed_background_union_and_required_membership_cannot_drift(self):
        _, good = self.prepare_background_actor_case()
        def case(spec):
            return next(c for c in spec['scenarios'] if c['id'] == 'crossing_blocker')
        mutations = (
            lambda s:s['scenario_suite_contract'].pop('background_actor_ids'),
            lambda s:case(s).pop('background_actor_ids'),
            lambda s:case(s)['background_actor_ids'].pop(),
            lambda s:s['scenario_suite_contract']['background_actor_ids'].pop(),
            lambda s:s['scenario_suite_contract']['enabled_actor_ids'].pop(),
            lambda s:next(a for a in s['dynamic_actors'] if a['id']=='office_person').update(enabled=False),
            lambda s:s['scenario_suite_contract']['actor_encounter_contract']['required_actor_ids'].append('office_person'),
            lambda s:case(s).update(actor_ids=['office_person']),
        )
        for mutate in mutations:
            bad = copy.deepcopy(good); mutate(bad)
            bad['scenario_suite_contract']['configuration_sha256'] = suite.configuration_digest(bad)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):suite.verify_prepared(bad)
        bad = copy.deepcopy(good)
        case(bad)['background_actor_ids'].pop()
        with self.assertRaisesRegex(ValueError, 'configuration_changed'):suite.verify_prepared(bad)

    def test_background_actor_absence_still_rejects_full_actual_geometry_audit(self):
        _, spec = self.prepare_background_actor_case()
        spec['robot_collision_registry'] = dict(root='/World/Spot', colliders=[dict(
            path='/World/Spot/body/collision', type='Sphere', local_geometry=dict(radius=.1), world_scale=[1.,1.,1.])])
        audit = Audit(spec, 'synthetic_background_scope_test', 'a'*64, 1_000_000_000)
        robot = [dict(path='/World/Spot/body/collision', shape=dict(type='Sphere',radius=.1),
            world_matrix=pose_matrix([-8.,-5.,.481], [1.,0.,0.,0.]).tolist())]
        selected_only = dict(plaza_person=dict(present=True,position=[0.,-15.,0.],
            orientation_xyzw=[0.,0.,0.,1.],source_stamp_ns=1_000_000_000))
        with self.assertRaises(ValueError):audit.sample(0, robot, selected_only)

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
        self.assertEqual(contract["navigation_command_limit_mps"], .23)
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
        self.assertEqual(phases[1]["timeout_source_s"], 326)

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

    def test_new_input_explicitly_seals_spot_exposure_without_changing_legacy(self):
        original = world.load_world()
        # The campus default now explicitly selects the .65/3m profile. Freeze
        # a separate missing-profile source to keep the historical 2m branch
        # tested; preparing a new input must never mutate that earlier seal.
        original["robot"].pop("actor_encounter_profile")
        legacy_source = self.root / "legacy_source.json"
        legacy_source.write_text(json.dumps(original))
        legacy_target = self.root / "prepared.json"
        suite.prepare_case(legacy_source, "crossing_blocker", legacy_target)
        legacy = json.loads(legacy_target.read_text())
        self.assertEqual(legacy["scenario_suite_contract"]["actor_encounter_contract"]["near_center_distance_m"], 2.)
        original["robot"]["actor_encounter_profile"] = SPOT_EXPOSURE_PROFILE
        original["robot"]["model_limits"]["max_linear_speed_mps"] = .6
        original["robot"]["full_xyz_reference_model"].update(reference_max_speed_mps=.6,
            measured_travel_max_speed_mps=.6,evidence_sha256="b"*64)
        source=self.root/"new_source.json";source.write_text(json.dumps(original))
        target=self.root/"new_prepared.json"
        suite.prepare_case(source,"crossing_blocker",target)
        spec=json.loads(target.read_text())
        contract=suite.verify_prepared(spec)["actor_encounter_contract"]
        self.assertEqual(contract["near_center_distance_m"],3.)
        self.assertEqual(contract["profile"]["source_reference_evidence_sha256"],"b"*64)
        self.assertEqual(contract["minimum_near_source_s"],.5)
        self.assertNotIn("execution_braking_model_sha256",contract["profile"])
        self.assertEqual(json.loads((self.root/"prepared.json").read_text()),legacy)
        spec["robot"]["navigation_envelope"]["width_m"] += .01
        spec["scenario_suite_contract"]["configuration_sha256"]=suite.configuration_digest(spec)
        with self.assertRaisesRegex(ValueError,"invalid_actor_encounter_contract"):
            suite.verify_prepared(spec)


class EvidenceEvaluationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.config = self.root / "prepared.json"
        # These synthetic sampled-record tests explicitly exercise the legacy
        # evidence branch, independent of the new campus full500Hz/.65 defaults.
        legacy = world.load_world()
        legacy["robot"].pop("actor_encounter_profile")
        legacy["robot"]["record_full_physics_history"] = False
        source = self.root / "unit_legacy_source.json"
        source.write_text(json.dumps(legacy))
        suite.prepare_case(source, "crossing_blocker", self.config)
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

    def enable_spot_profile_fixture(self, token=SPOT_EXPOSURE_PROFILE, speed=.6):
        robot=self.spec["robot"]
        robot["actor_encounter_profile"]=token
        robot["model_limits"]["max_linear_speed_mps"]=speed
        robot["full_xyz_reference_model"].update(reference_max_speed_mps=speed,
            measured_travel_max_speed_mps=speed,evidence_sha256="b"*64)
        self.spec["scenario_suite_contract"]["actor_encounter_contract"]=encounter_contract(
            self.contract["enabled_actor_ids"],self.spec)
        self.spec["scenario_suite_contract"]["configuration_sha256"]=suite.configuration_digest(self.spec)
        self.config.write_text(json.dumps(self.spec));self.contract=suite.verify_prepared(self.spec)

    def bind_spot_session_fixture(self, execution):
        profile=self.contract["actor_encounter_contract"]["profile"]
        speed=profile['reachable_max_speed_mps']
        session=json.loads((self.session/"session.json").read_text())
        robot=self.spec["robot"]
        session.update(simulation_clock="isaac_fixed_anchor_v1",physical_acceptance=False,
            max_speed_mps=robot["max_linear_speed"],max_yaw_radps=robot["max_angular_speed"],
            body_height=robot["body_reference_height"])
        session["static_collision_prior_contract"]["body_envelope"]=profile["body_envelope"]
        record=dict(schema_version=3,model="reaction_braking_reachable_v1",fixture_only=True,
            transport_mode="isolated_mock",session_id=session["id"],measurements=dict(max_speed_mps=speed,max_yaw_radps=.8),
            isolated_platform_model=dict(schema=1,kind="official_spot_physx",command_max_speed_mps=session["max_speed_mps"],
                command_max_yaw_radps=session["max_yaw_radps"],reachable_max_speed_mps=speed,reachable_max_yaw_radps=.8,
                source_scope="isolated_simulation_physx_measured_model"),
            isolated_full_xyz_reference_model=dict(schema=1,kind="official_spot_physx",reference_max_speed_mps=speed,
                measured_travel_max_speed_mps=speed,observed_max_full_xyz_speed_mps=.575947,
                evidence_sha256=profile["source_reference_evidence_sha256"],source_scope="isolated_simulation_physx_measured_model"))
        path=self.write("braking_model.json",record)
        session.update(execution_braking_model_record=str(path),execution_braking_model_sha256=suite._sha(path),
            input_hashes={str(path):suite._sha(path)})
        execution["encounter_profile_model_binding"]=suite.encounter_profile_model_binding(self.spec,session)
        self.write("session.json",session);self.write("scenario_suite/execution.json",execution)
        return session,record,path

    def test_recalibrated_profile_binds_exact_65_model_and_rejects_resealed_old_domain(self):
        self.enable_spot_profile_fixture(SPOT_EXPOSURE_PROFILE_65, .65)
        _, execution = self.complete_action_fixture()
        session, record, path = self.bind_spot_session_fixture(execution)
        self.assertEqual(execution['encounter_profile_model_binding']['profile'], SPOT_EXPOSURE_PROFILE_65)
        for marker, field, invalid in (('measurements', 'max_speed_mps', .6),
                ('isolated_platform_model', 'reachable_max_speed_mps', .6),
                ('isolated_platform_model', 'reachable_max_yaw_radps', .7),
                ('isolated_full_xyz_reference_model', 'reference_max_speed_mps', .6),
                ('isolated_full_xyz_reference_model', 'measured_travel_max_speed_mps', .6),
                ('isolated_full_xyz_reference_model', 'evidence_sha256', 'c'*64)):
            bad = copy.deepcopy(record)
            bad[marker][field] = invalid
            self.write('braking_model.json', bad)
            session.update(execution_braking_model_sha256=suite._sha(path), input_hashes={str(path):suite._sha(path)})
            with self.subTest(field=field), self.assertRaises(ValueError):
                suite.encounter_profile_model_binding(self.spec, session)

    def collision_fixture(self, exercise_encounter=True):
        trajectory = self.session / "physics/trajectory.jsonl"
        audit = Audit(self.spec, "unit_synthetic_session", suite._sha(self.config), 1_000_000_000)
        times = [i*20_000_000 for i in range(201)]
        # The 3m fixture also needs its entire 1s approach after goal admission.
        # Keep the legacy trajectory literal unchanged.
        outside = 4.5 if "profile" in self.contract["actor_encounter_contract"] else 3.
        for ns in times:
            t = ns*1e-9
            distance = (outside if t <= 1. else outside-(outside-1.5)*(t-1.) if t <= 2. else 1.5 if t <= 2.5 else 1.5+3.*(t-2.5)) if exercise_encounter else 6.
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

    def test_new_profile_requires_exact_session_model_hash_source_and_actual_body(self):
        self.enable_spot_profile_fixture()
        _,execution=self.complete_action_fixture()
        self.write("physics/collision_audit.json",self.collision_fixture())
        result=suite.evaluate_case(self.spec,self.session)
        self.assertEqual(next(c for c in result["checks"] if c["name"]=="sealed_spot_encounter_profile_model")["status"],"failed")
        session,record,path=self.bind_spot_session_fixture(execution)
        self.assertEqual(suite.evaluate_case(self.spec,self.session)["status"],"passed")
        for mutate in (lambda s:s.update(execution_braking_model_sha256="f"*64),
                lambda s:s.update(input_hashes={}),lambda s:s.update(id="foreign"),
                lambda s:s["static_collision_prior_contract"]["body_envelope"].update(radius=.5),
                lambda s:s["static_collision_prior_contract"].update(body_envelope_registry_sha256="f"*64)):
            bad=copy.deepcopy(session);mutate(bad);self.write("session.json",bad)
            result=suite.evaluate_case(self.spec,self.session)
            check=next(c for c in result["checks"] if c["name"]=="sealed_spot_encounter_profile_model")
            self.assertEqual(check["status"],"failed")
        self.write("session.json",session)
        record["isolated_full_xyz_reference_model"]["evidence_sha256"]="c"*64
        self.write("braking_model.json",record)
        session.update(execution_braking_model_sha256=suite._sha(path),input_hashes={str(path):suite._sha(path)})
        self.write("session.json",session)
        check=next(c for c in suite.evaluate_case(self.spec,self.session)["checks"] if c["name"]=="sealed_spot_encounter_profile_model")
        self.assertEqual(check["status"],"failed")
        self.assertIn("source_domain_mismatch",check["reason"])

    def test_new_execution_cannot_reuse_foreign_model_binding(self):
        self.enable_spot_profile_fixture();_,execution=self.complete_action_fixture()
        self.bind_spot_session_fixture(execution)
        self.write("physics/collision_audit.json",self.collision_fixture())
        execution["encounter_profile_model_binding"]["execution_braking_model_sha256"]="f"*64
        self.write("scenario_suite/execution.json",execution)
        check=next(c for c in suite.evaluate_case(self.spec,self.session)["checks"] if c["name"]=="sealed_spot_encounter_profile_model")
        self.assertEqual(check["status"],"failed")

    def test_explicit_full_physics_record_cannot_pass_on_only_sampled_action_evidence(self):
        self.spec["robot"]["record_full_physics_history"]=True
        self.spec["scenario_suite_contract"]["configuration_sha256"]=suite.configuration_digest(self.spec)
        self.config.write_text(json.dumps(self.spec));self.contract=suite.verify_prepared(self.spec)
        self.complete_action_fixture();self.write("physics/collision_audit.json",self.collision_fixture())
        check=next(c for c in suite.evaluate_case(self.spec,self.session)["checks"] if c["name"]=="independent_full_physics_motion_domain_and_stop")
        self.assertEqual(check["status"],"pending")
        self.assertEqual(suite.evaluate_case(self.spec,self.session)["status"],"pending")

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
