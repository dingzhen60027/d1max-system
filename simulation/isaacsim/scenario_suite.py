#!/usr/bin/env python3
"""Prepare and review campus cases; execute original Actions in one session.

Preparing a case seals its *input contract*, not a navigation acceptance result.
Its new world configuration must be used to build a new immutable candidate,
including maps and actual official robot collision registration, before launch.
The suite never resets the plant, moves obstacles relative to the robot, creates
a second navigation controller, or combines runs into a multi-goal success.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from world_builder import (DEFAULT_WORLD, find_route, load_world, scenario_matrix,
                           scenario_spec, validate_world, voxel_budget)
from collision_audit import encounter_contract, review_actual_encounter, validate_encounter_contract

HERE = Path(__file__).resolve().parent
KIND = "isolated_isaac_scenario_input_v1"
REPORT_KIND = "isolated_isaac_scenario_evaluation_v1"
DYNAMIC_ENCOUNTER_CASES = {"crossing_blocker", "narrow_head_on", "temporary_door_block", "resume_after_block"}


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def configuration_digest(spec):
    value = copy.deepcopy(spec)
    value["scenario_suite_contract"].pop("configuration_sha256", None)
    return hashlib.sha256(_canonical(value)).hexdigest()


def _case(spec, identifier):
    case = next((s for s in spec["scenarios"] if s["id"] == identifier), None)
    if case is None:
        raise ValueError("unknown_scenario:" + identifier)
    return case


def ground_goals(spec, case):
    # Schema1's old fixture XYZ may annotate body reference height. A Navigate
    # ground goal always uses the actual floor plane, never that body height.
    return [[float(p[0]), float(p[1]), float(spec["floor"]["z"])] for p in case["goals"]]


def static_segment_lengths(spec, case):
    body_z = float(spec["floor"]["z"]) + float(spec["robot"]["body_reference_height"])
    start, lengths = [case["start"][0], case["start"][1], body_z], []
    for goal in ground_goals(spec, case):
        body_goal = [goal[0], goal[1], body_z]
        route = find_route(spec, start, body_goal)
        lengths.append(sum(math.dist(a, b) for a, b in zip(route, route[1:])))
        start = body_goal
    return lengths


def phase_source_budget(length_m, command_speed_mps):
    if not math.isfinite(command_speed_mps) or command_speed_mps <= 0:
        raise ValueError("positive_navigation_command_speed_required")
    # Test observation budget only. The original Action, blocked timeout,
    # input watchdogs and signed motion leases are never extended by this.
    return math.ceil(math.ceil(length_m / command_speed_mps) * 1.5 + 120)


def phases_for_case(spec, case, lengths=None):
    goals = ground_goals(spec, case)
    lengths = static_segment_lengths(spec, case) if lengths is None else lengths
    budgets = [phase_source_budget(length, float(spec["robot"]["max_linear_speed"])) for length in lengths]
    def phase(index, smoke_case, goal_index=0, **event):
        budget = budgets[goal_index]
        if smoke_case == "cancel":
            budget = min(case["timeout_source_s"], max(60., event["cancel_source_time_s"] + 30.))
        return dict(index=index, smoke_case=smoke_case, goal=goals[goal_index],
            static_route_length_m=lengths[goal_index], timeout_source_s=budget,
            timeout_clock="seconds_since_phase_first_original_measured_source_stamp",
            budget_scope="test_observation_only; original_BT_watchdogs_and_motion_leases_unchanged", **event)
    if case["id"] == "resume_after_block":
        cancel = next(e["source_time_s"] for e in case["events"] if e["action"] == "cancel_current_task")
        resume = next(e["source_time_s"] for e in case["events"] if e["action"] == "submit_new_task")
        return [phase(0, "cancel", cancel_source_time_s=cancel, submit_after_source_s=0.),
                phase(1, "goal", submit_after_source_s=resume)]
    if case["id"] == "cancel_and_park":
        cancel = next(e["source_time_s"] for e in case["events"] if e["action"] == "cancel_current_task")
        return [phase(0, "cancel", cancel_source_time_s=cancel, submit_after_source_s=0.)]
    return [phase(i, "goal", goal_index=i, submit_after_source_s=0.) for i in range(len(goals))]


def list_cases(spec):
    rows = []
    for case in scenario_matrix(spec):
        lengths = static_segment_lengths(spec, case)
        rows.append(dict(id=case["id"], ground_goals=ground_goals(spec, case), actor_ids=case["actor_ids"],
                         static_route_length_m=sum(lengths), segment_lengths_m=lengths,
                         phase_timeout_source_s=[p["timeout_source_s"] for p in phases_for_case(spec, case, lengths)],
                         length_semantics="conservative_offline_static_whole_body_connectivity; not_executed_path",
                         timeout_source_s=case["timeout_source_s"], execution_status="pending"))
    return rows


def prepare_case(spec_path, identifier, output_config):
    source = Path(spec_path).resolve(strict=True)
    original = load_world(source)
    case = _case(original, identifier)
    prepared = scenario_spec(original, identifier)
    goals = ground_goals(original, case)
    prepared["robot"]["fixture_goal_annotation_xyz"] = list(original["robot"]["goal"])
    prepared["robot"]["fixture_goal_body_reference_height_m"] = original["robot"]["body_reference_height"]
    prepared["robot"]["goal"] = list(goals[0])
    selected = _case(prepared, identifier)
    selected["fixture_goal_annotations_xyz"] = copy.deepcopy(case["goals"])
    selected["goals"] = copy.deepcopy(goals)
    prepared["scenario_suite_contract"] = dict(schema=1, kind=KIND, case_id=identifier,
        world_source_sha256=_sha(source), input_code_sha256={name: _sha(HERE / name) for name in ("world_builder.py", "scenario_suite.py", "collision_audit.py")},
        seed=prepared["seed"], enabled_actor_ids=sorted(case["actor_ids"]),
        start_pose=prepared["robot"]["initial_pose"], goals=goals,
        fixture_goal_annotations_xyz=copy.deepcopy(case["goals"]),
        phases=phases_for_case(prepared, selected), timeout_source_s=case["timeout_source_s"],
        event_clock="seconds_since_first_phase_original_measured_source_stamp",
        actor_clock="original_simulation_source_time_since_plant_start; never_reset_between_goals",
        navigation_command_limit_mps=prepared["robot"]["max_linear_speed"],
        timeout_scope="test_observation_only; original_BT_watchdogs_and_motion_leases_unchanged",
        stationarity_window_source_s=2., goal_tolerance_xy_m=.35,
        actor_encounter_contract=encounter_contract(case["actor_ids"] if identifier in DYNAMIC_ENCOUNTER_CASES else []),
        candidate_seal_required=True, runtime_pose_or_actor_switching_allowed=False,
        input_seal_scope="canonical_configuration_only; new_candidate_static_geometry_robot_and_runtime_seals_required",
        execution_status="pending", physical_robot_acceptance=False)
    prepared["scenario_suite_contract"]["configuration_sha256"] = configuration_digest(prepared)
    validate_world(prepared)
    target = Path(output_config).expanduser().absolute()
    if target.exists() or target.is_symlink():
        raise FileExistsError("output_config_already_exists:" + str(target))
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("x") as stream:
        stream.write(json.dumps(prepared, indent=2, allow_nan=False) + "\n")
    return dict(config=str(target.resolve()), case_id=identifier, configuration_sha256=prepared["scenario_suite_contract"]["configuration_sha256"],
                status="pending", next_step="build_new_candidate_from_this_config_before_run", voxel_budget=voxel_budget(prepared, .05))


def verify_prepared(spec, identifier=None):
    validate_world(spec)
    contract = spec.get("scenario_suite_contract", {})
    if contract.get("kind") != KIND or contract.get("schema") != 1:
        raise ValueError("prepared_scenario_input_required")
    if identifier is not None and contract.get("case_id") != identifier:
        raise ValueError("prepared_scenario_case_mismatch")
    if contract.get("configuration_sha256") != configuration_digest(spec):
        raise ValueError("prepared_scenario_configuration_changed")
    if spec.get("active_scenario") != contract["case_id"]:
        raise ValueError("prepared_active_scenario_mismatch")
    if sorted(a["id"] for a in spec["dynamic_actors"] if a["enabled"]) != contract["enabled_actor_ids"]:
        raise ValueError("prepared_actor_registry_changed")
    encounter = contract.get("actor_encounter_contract")
    required = contract["enabled_actor_ids"] if contract["case_id"] in DYNAMIC_ENCOUNTER_CASES else []
    if encounter is None and required:
        raise ValueError("prepared_actor_encounter_contract_missing; fresh_prepare_required")
    if encounter is not None:
        validate_encounter_contract(encounter, contract["enabled_actor_ids"])
        if encounter["required_actor_ids"] != required:
            raise ValueError("prepared_intended_encounter_actor_ids_changed")
    if any(goal[2] != spec["floor"]["z"] for goal in contract["goals"]):
        raise ValueError("scenario_goals_must_be_ground_xyz")
    for index, phase in enumerate(contract.get("phases", [])):
        budget = phase.get("timeout_source_s")
        if (phase.get("index") != index or isinstance(budget, bool)
                or not isinstance(budget, (int, float)) or not math.isfinite(budget) or budget <= 0):
            raise ValueError("invalid_phase_source_budget")
    return contract


def _new_json(path, value):
    path = Path(path)
    if path.exists() or path.is_symlink():
        raise FileExistsError("report_already_exists:" + str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False) + "\n")


def _read(path):
    return json.loads(Path(path).read_text()) if Path(path).is_file() else None


def _inside(session, path):
    path = Path(path)
    resolved = (session / path).resolve() if not path.is_absolute() else path.resolve()
    if not resolved.is_relative_to(session):
        raise ValueError("evidence_outside_session")
    return resolved


def _event_stamp(report, key):
    stamps = [e.get("source_stamp_ns") for e in report.get("events", []) if e.get("key") == key]
    return next((s for s in stamps if type(s) is int and s > 0), None)


def evaluate_case(prepared_spec, session_dir):
    """Evidence is required individually; missing data stays pending.

    A collision report must bind actual registered official link and actor geometry,
    cover the Action source interval and hash the recorded physical trajectory.
    Offline A*, desired actor poses, a writer ACK, or an old wheel-only audit
    cannot satisfy this check. A pass is a sampled simulation result only.
    """
    contract = verify_prepared(prepared_spec)
    session_dir = Path(session_dir).resolve()
    checks = []
    def check(name, status, reason="", evidence=None):
        checks.append(dict(name=name, status=status, reason=reason, evidence=evidence))
    try:
        session = _read(session_dir / "session.json")
        execution = _read(session_dir / "scenario_suite/execution.json")
        summary = _read(session_dir / "run_summary.json")
        if session is None:
            check("sealed_same_scene", "pending", "session_not_executed")
        else:
            scene_path = Path(session["isaac_bridge_contract"]["scene_config"]).resolve(strict=True)
            actual = json.loads(scene_path.read_text())
            same = (verify_prepared(actual)["configuration_sha256"] == contract["configuration_sha256"]
                and session["isaac_bridge_contract"]["scene_sha256"] == _sha(scene_path)
                and session.get("simulation_backend") == "isaacsim_physx"
                and session.get("transport_mode") == "isolated_mock")
            check("sealed_same_scene", "passed" if same else "failed", "" if same else "foreign_or_changed_sealed_scene", str(scene_path))
        if execution is None:
            check("single_session_ordered_actions", "pending", "no_suite_execution_record")
        else:
            same = (session is not None and execution.get("session_id") == session["id"]
                and execution.get("case_id") == contract["case_id"]
                and execution.get("configuration_sha256") == contract["configuration_sha256"])
            if not same or execution.get("status") == "failed":
                check("single_session_ordered_actions", "failed", execution.get("error") or "execution_identity_mismatch")
            elif execution.get("status") != "completed" or len(execution.get("phases", [])) != len(contract["phases"]):
                check("single_session_ordered_actions", "pending", "not_all_action_phases_completed")
            else:
                check("single_session_ordered_actions", "passed", evidence=str(session_dir / "scenario_suite/execution.json"))
        reports, sent_stamps, terminal_stamps, task_ids = [], [], [], []
        for index, phase in enumerate(contract["phases"]):
            entry = next((p for p in (execution or {}).get("phases", []) if p.get("index") == index), None)
            prefix = "phase_%03d" % index
            if entry is None:
                check(prefix + "_action_stop_source", "pending", "phase_not_executed")
                continue
            report_path = _inside(session_dir, entry["report"])
            report = _read(report_path)
            if report is None:
                check(prefix + "_action_stop_source", "pending", "phase_report_missing")
                continue
            reports.append(report)
            if entry.get("report_sha256") != _sha(report_path):
                check(prefix + "_action_stop_source", "failed", "phase_report_hash_changed")
                continue
            expected_status = 5 if phase["smoke_case"] == "cancel" else 4
            result, stop = report.get("result") or {}, report.get("stop") or {}
            stationary = report.get("stationarity_evidence") or {}
            imu = report.get("imu_evidence") or {}
            sample_stamps = [s.get("source_stamp_ns") for s in report.get("measured_samples", [])]
            measured_source = (len(sample_stamps) >= 3 and all(type(s) is int and s > 0 for s in sample_stamps)
                and all(b > a for a, b in zip(sample_stamps, sample_stamps[1:])))
            sent, terminal = _event_stamp(report, "goal_sent"), _event_stamp(report, "BT_result")
            sent_stamps.append(sent)
            terminal_stamps.append(terminal)
            task_ids.append((report.get("bt_status") or {}).get("task_id"))
            trace_path = _inside(session_dir, entry["trace"])
            required = (report.get("passed") is True and report.get("session_id") == (session or {}).get("id")
                and report.get("goal") == phase["goal"] and report.get("case") == phase["smoke_case"]
                and report.get("action_result_status") == expected_status
                and result.get("success") is (phase["smoke_case"] == "goal")
                and (phase["smoke_case"] != "cancel" or result.get("reason") in ("action_cancelled", "user_cancelled"))
                and result.get("retirement_confirmed") is True and result.get("physical_stop_confirmed") is False
                and report.get("physical_acceptance") is False
                and stop.get("stop_submitted") is True and stop.get("measured_stop_confirmed") is True
                and stop.get("physical_acceptance_verified") is False and bool(stop.get("execution_id"))
                and report.get("measured_stationary") is True and report.get("imu_source_matching") is True
                and report.get("body_height_consistent") is True
                and report.get("whole_body_attestation") is True
                and measured_source and imu.get("matched") is True and imu.get("state_count", 0) > 0
                and imu.get("missing_imu_source_stamps_ns") == []
                and stationary.get("source_window_complete") is True and stationary.get("source_span_s", 0) >= 2.
                and sent is not None and terminal is not None and terminal > sent
                and trace_path.is_file() and entry.get("trace_sha256") == _sha(trace_path))
            negative = (result.get("retirement_confirmed") is False or stop.get("measured_stop_confirmed") is False
                or report.get("measured_stationary") is False or report.get("imu_source_matching") is False
                or report.get("body_height_consistent") is False or stationary.get("source_window_complete") is False
                or report.get("whole_body_attestation") is False
                or report.get("physical_acceptance") is True or stop.get("physical_acceptance_verified") is True
                or (report.get("session_id") is not None and report["session_id"] != (session or {}).get("id"))
                or (report.get("action_result_status") is not None and report["action_result_status"] != expected_status)
                or (report.get("goal") is not None and report["goal"] != phase["goal"]))
            if report.get("passed") is False or report.get("error") or entry.get("returncode") not in (None, 0) or negative:
                check(prefix + "_action_stop_source", "failed", report.get("error") or "original_action_or_measured_stop_failed", str(report_path))
            elif not required:
                check(prefix + "_action_stop_source", "pending", "missing_bound_action_source_imu_or_two_second_stop_evidence", str(report_path))
            else:
                check(prefix + "_action_stop_source", "passed", evidence=str(report_path))
            origin, budget = report.get("source_origin_ns"), entry.get("source_timeout_s")
            if (type(origin) is int and terminal is not None and isinstance(budget, (int, float))
                    and not isinstance(budget, bool) and math.isfinite(budget) and budget > 0
                    and report.get("source_duration_s") == budget):
                elapsed = (terminal-origin)*1e-9
                within = elapsed >= 0 and elapsed <= budget <= phase["timeout_source_s"]
                check(prefix + "_test_source_budget", "passed" if within else "failed",
                    "" if within else "phase_observation_source_budget_exceeded_or_changed",
                    dict(source_elapsed_s=elapsed, applied_source_budget_s=budget,
                        sealed_phase_source_budget_s=phase["timeout_source_s"]))
            else:
                check(prefix + "_test_source_budget", "pending", "phase_source_budget_evidence_missing")
            if phase["smoke_case"] == "goal":
                position = report.get("measured_final_position")
                if not position or len(position) != 3:
                    check(prefix + "_goal", "pending", "measured_final_position_missing")
                else:
                    distance = math.hypot(position[0]-phase["goal"][0], position[1]-phase["goal"][1])
                    check(prefix + "_goal", "passed" if distance <= contract["goal_tolerance_xy_m"] else "failed", "", dict(measured_xy_error_m=distance))
            if phase["smoke_case"] == "cancel":
                cancelled = _event_stamp(report, "cancel_requested")
                origin = report.get("source_origin_ns")
                if cancelled is None or type(origin) is not int:
                    check(prefix + "_source_cancel", "pending", "source_cancel_event_missing")
                else:
                    elapsed = (cancelled-origin)*1e-9
                    check(prefix + "_source_cancel", "passed" if elapsed >= phase["cancel_source_time_s"] else "failed", evidence=dict(source_elapsed_s=elapsed))
        if (len(reports) == len(contract["phases"]) and len(sent_stamps) == len(contract["phases"])
                and len(terminal_stamps) == len(contract["phases"])
                and all(s is not None for s in sent_stamps + terminal_stamps)):
            ordered = all(a < b for a, b in zip(terminal_stamps, sent_stamps[1:]))
            unique = len(set(task_ids)) == len(task_ids) and all(task_ids)
            source_origin = reports[0].get("source_origin_ns")
            timed = (type(source_origin) is int and terminal_stamps[-1]-source_origin <= contract["timeout_source_s"]*1e9)
            resumed = (contract["case_id"] != "resume_after_block" or (type(source_origin) is int
                and sent_stamps[1] >= source_origin+contract["phases"][1]["submit_after_source_s"]*1e9))
            check("fresh_ordered_task_identities_and_source_deadline", "passed" if ordered and unique and timed and resumed else "failed", "" if ordered and unique and timed and resumed else "task_order_identity_or_source_deadline_failed")
        else:
            check("fresh_ordered_task_identities_and_source_deadline", "pending", "original_action_source_events_incomplete")
        collision_path = session_dir / "physics/collision_audit.json"
        collision = _read(collision_path)
        bound = False
        if collision is None:
            check("actual_link_static_and_actor_collision", "pending", "actual_quadruped_collision_audit_missing")
        else:
            trajectory = session_dir / "physics/trajectory.jsonl"
            bound = (collision.get("completed") is True and collision.get("robot_kind") == prepared_spec["robot"]["kind"]
                and collision.get("scope") == "sampled_actual_link_geometry_static_and_actor"
                and collision.get("session_id") == (session or {}).get("id")
                and collision.get("spec_sha256") == (session or {}).get("isaac_bridge_contract", {}).get("scene_sha256")
                and trajectory.is_file() and collision.get("trajectory_sha256") == _sha(trajectory)
                and collision.get("sample_count", 0) > 0
                and bool(collision.get("robot_collision_registry_sha256"))
                and collision.get("robot_collision_registry_sha256") == (session or {}).get(
                    "static_collision_prior_contract", {}).get("body_envelope_registry_sha256")
                and collision.get("actor_ids") == contract["enabled_actor_ids"]
                and sent_stamps and terminal_stamps and all(s is not None for s in sent_stamps + terminal_stamps)
                and type(collision.get("source_begin_ns")) is int and type(collision.get("source_end_ns")) is int
                and collision.get("source_begin_ns", math.inf) <= min(sent_stamps)
                and collision.get("source_end_ns", -1) >= max(terminal_stamps))
            count_static, count_dynamic = collision.get("non_floor_static_penetration_count"), collision.get("dynamic_penetration_count")
            if (type(count_static) is int and count_static > 0) or (type(count_dynamic) is int and count_dynamic > 0):
                check("actual_link_static_and_actor_collision", "failed", "measured_geometry_penetration", str(collision_path))
            elif bound and type(count_static) is int and type(count_dynamic) is int and count_static == 0 and count_dynamic == 0:
                check("actual_link_static_and_actor_collision", "passed", evidence=str(collision_path))
            else:
                check("actual_link_static_and_actor_collision", "pending", "collision_audit_identity_geometry_or_source_coverage_incomplete", str(collision_path))
        if contract["case_id"] in DYNAMIC_ENCOUNTER_CASES:
            if not bound or collision.get("actual_actor_encounter") is None:
                check("intended_dynamic_encounter_observed", "pending", "bound_actual_geometry_encounter_audit_missing; goal_success_is_insufficient")
            else:
                try:
                    witness = collision["actual_actor_encounter"]
                    anchor = (session or {}).get("isaac_bridge_contract", {}).get("clock_anchor_ns")
                    if (not isinstance(witness, dict) or type(anchor) is not int or anchor < 0
                            or type(witness.get("source_anchor_ns")) is not int
                            or witness["source_anchor_ns"] != anchor):
                        raise ValueError("actor_encounter_authoritative_session_clock_anchor_mismatch")
                    observed, selected = review_actual_encounter(witness, prepared_spec,
                        collision, min(sent_stamps), max(terminal_stamps))
                    check("intended_dynamic_encounter_observed", "pending" if observed is None else "passed" if observed else "failed",
                        "encounter_sampling_or_audit_incomplete" if observed is None else "" if observed else "intended_sampled_proximity_encounter_not_exercised_during_original_action_interval",
                        dict(collision_audit_sha256=_sha(collision_path), source="actual_registered_link_and_actor_geometry",
                             scope="sampled_same_step_primitive_center_proximity_only", selected_exact_pair_witnesses=selected))
                except (ValueError, KeyError, TypeError) as exc:
                    check("intended_dynamic_encounter_observed", "failed", "invalid_actual_encounter_evidence:" + str(exc), str(collision_path))
        if summary is None:
            check("owned_graph_retirement", "pending", "run_still_active_or_not_executed")
        else:
            retired = summary.get("navigation_shutdown") or {}
            success = (summary.get("clean_shutdown") is True and retired.get("session_id") == (session or {}).get("id")
                and retired.get("request_accepted") is True and retired.get("software_retired") is True)
            check("owned_graph_retirement", "passed" if success else "failed", summary.get("error", ""), str(session_dir / "run_summary.json"))
    except (ValueError, KeyError, TypeError, OSError) as exc:
        check("evidence_integrity", "failed", str(exc))
    status = "failed" if any(c["status"] == "failed" for c in checks) else "passed" if checks and all(c["status"] == "passed" for c in checks) else "pending"
    return dict(schema=1, kind=REPORT_KIND, case_id=contract["case_id"], configuration_sha256=contract["configuration_sha256"],
                session=str(session_dir), status=status, passed=status == "passed", checks=checks,
                physical_robot_acceptance=False, collision_scope="sampled_actual_geometry_only_when_bound_evidence_present",
                continuous_collision_certificate=False, localization_validation=False)


def run_command(candidate, session, identifier, headless=True):
    candidate = Path(candidate).resolve(strict=True)
    config = candidate / "simulation/assets/scene_config.json"
    verify_prepared(json.loads(config.read_text()), identifier)
    runner = candidate / "simulation/run.py"
    if '--scenario-case' not in runner.read_text():
        raise ValueError("frozen_run_has_no_scenario_interface; build_new_candidate_with_supported_runner")
    if Path(session).exists():
        raise FileExistsError("new_session_required")
    command = [sys.executable, str(runner), "--candidate", str(candidate), "--session", str(Path(session).absolute()), "--scenario-case", identifier]
    if headless:
        command.append("--headless")
    return command


def execute_case(session_dir, identifier):
    """Observer child invoked by run.py inside its existing owned loopback graph."""
    from d1max_pct_scan.isolated_zenoh import validate_environment
    validate_environment()
    session_dir = Path(session_dir).resolve(strict=True)
    session = json.loads((session_dir / "session.json").read_text())
    if session.get("simulation_backend") != "isaacsim_physx" or session.get("transport_mode") != "isolated_mock":
        raise ValueError("isaac_isolated_session_required")
    scene = Path(session["isaac_bridge_contract"]["scene_config"]).resolve(strict=True)
    if _sha(scene) != session["isaac_bridge_contract"]["scene_sha256"]:
        raise ValueError("sealed_scene_changed")
    spec = json.loads(scene.read_text())
    contract = verify_prepared(spec, identifier)
    smoke = HERE / "smoke.py"
    if not all(flag in smoke.read_text() for flag in ("--trace-output", "--cancel-source-time-s", "--stationary-window-s", "--source-duration-s")):
        raise ValueError("frozen_smoke_has_no_multi_goal_source_evidence_interface")
    output = session_dir / "scenario_suite"
    output.mkdir()  # A repeated case requires a new source-time session.
    record = dict(schema=1, case_id=identifier, session_id=session["id"], configuration_sha256=contract["configuration_sha256"],
                  status="running", phases=[], source_origin_ns=None, physical_robot_acceptance=False,
                  event_clock=contract["event_clock"], actor_clock=contract["actor_clock"],
                  scene_clock_anchor_ns=session["isaac_bridge_contract"]["clock_anchor_ns"],
                  timeout_scope=contract["timeout_scope"])
    record_path = output / "execution.json"
    _new_json(record_path, record)
    def save():
        temporary = output / ".execution.tmp"
        temporary.write_text(json.dumps(record, indent=2) + "\n")
        os.replace(temporary, record_path)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    rclpy.init()
    clock = Node("isaac_scenario_source_observer", parameter_overrides=[Parameter("use_sim_time", value=True)])
    source_origin = None
    try:
        clock_deadline = time.monotonic() + 10.
        while clock.get_clock().now().nanoseconds <= 0:
            if time.monotonic() >= clock_deadline:
                raise TimeoutError("original_scene_clock_not_received")
            rclpy.spin_once(clock, timeout_sec=.05)
        for phase in contract["phases"]:
            if phase["submit_after_source_s"]:
                if source_origin is None:
                    raise ValueError("resume_requires_first_phase_measured_source_origin")
                target = source_origin + round(phase["submit_after_source_s"] * 1e9)
                wall_deadline = time.monotonic() + contract["timeout_source_s"] * 10 + 180
                while clock.get_clock().now().nanoseconds < target:
                    if time.monotonic() >= wall_deadline:
                        raise TimeoutError("resume_source_wait_wall_timeout")
                    rclpy.spin_once(clock, timeout_sec=.05)
            phase_launch_ns = clock.get_clock().now().nanoseconds
            source_budget = float(phase["timeout_source_s"])
            if source_origin is not None:
                remaining = contract["timeout_source_s"]-(phase_launch_ns-source_origin)*1e-9
                if remaining <= 0:
                    raise TimeoutError("scenario_source_time_budget_exceeded_before_next_goal")
                source_budget = min(source_budget, remaining)
            index = phase["index"]
            report_path, trace_path = output / ("phase_%03d_report.json" % index), output / ("phase_%03d_trace.jsonl" % index)
            if report_path.exists() or trace_path.exists():
                raise FileExistsError("phase_evidence_already_exists")
            command = [sys.executable, str(smoke), "--session", str(session_dir), "--case", phase["smoke_case"],
                "--goal", *map(str, phase["goal"]), "--output", str(report_path), "--trace-output", str(trace_path),
                "--stationary-window-s", "2", "--source-duration-s", str(source_budget),
                "--duration", str(source_budget * 10 + 180)]
            if phase["smoke_case"] == "cancel":
                command += ["--cancel-source-time-s", str(phase["cancel_source_time_s"])]
            with (output / ("phase_%03d.log" % index)).open("x") as log:
                # Keep the independent original /clock observer live while the
                # normal Action client runs; no controller or task owner added.
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                try:
                    while process.poll() is None:
                        rclpy.spin_once(clock, timeout_sec=.05)
                    returncode = process.returncode
                finally:
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=5.)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait(timeout=5.)
            report = _read(report_path)
            entry = dict(index=index, goal=phase["goal"], smoke_case=phase["smoke_case"], returncode=returncode,
                source_timeout_s=source_budget, timeout_clock=phase["timeout_clock"],
                phase_launch_source_stamp_ns=phase_launch_ns,
                phase_launch_scene_elapsed_s=(phase_launch_ns-record["scene_clock_anchor_ns"])*1e-9,
                phase_event_source_origin_ns=(report or {}).get("source_origin_ns"),
                report=str(report_path.relative_to(session_dir)), trace=str(trace_path.relative_to(session_dir)),
                report_sha256=_sha(report_path) if report_path.is_file() else None,
                trace_sha256=_sha(trace_path) if trace_path.is_file() else None,
                goal_source_stamp_ns=_event_stamp(report or {}, "goal_sent"), terminal_source_stamp_ns=_event_stamp(report or {}, "BT_result"))
            record["phases"].append(entry)
            if source_origin is None and report is not None:
                source_origin = report.get("source_origin_ns")
                record["source_origin_ns"] = source_origin
            save()
            if returncode or report is None or report.get("passed") is not True:
                raise RuntimeError("original_smoke_phase_failed:" + str(index))
            terminal = entry["terminal_source_stamp_ns"]
            if type(source_origin) is not int or type(terminal) is not int:
                raise ValueError("phase_original_source_evidence_missing")
            if terminal-source_origin > contract["timeout_source_s"]*1e9:
                raise TimeoutError("scenario_source_time_budget_exceeded")
            if terminal-report["source_origin_ns"] > source_budget*1e9:
                raise TimeoutError("phase_source_time_budget_exceeded")
        record["status"] = "completed"
        # Actual collision/encounter audits are independent evidence. Completion
        # of Actions cannot fill those fields with fabricated all-clear values.
        save()
        print(json.dumps(dict(case_id=identifier, status="actions_completed_evaluation_pending", execution=str(record_path))))
        return 0
    except Exception as exc:
        record.update(status="failed", error=str(exc))
        save()
        print(json.dumps(dict(case_id=identifier, status="failed", error=str(exc), execution=str(record_path))))
        return 1
    finally:
        clock.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    listing = commands.add_parser("list", help="Offline case matrix and metre lengths; all execution states pending")
    listing.add_argument("--spec", type=Path, default=DEFAULT_WORLD)
    prepare = commands.add_parser("prepare", help="Write new input configuration; build and seal a new candidate afterward")
    prepare.add_argument("--spec", type=Path, default=DEFAULT_WORLD)
    prepare.add_argument("--case", required=True)
    prepare.add_argument("--output-config", required=True, type=Path)
    run = commands.add_parser("run", help="Launch one new session using a frozen prepared candidate")
    run.add_argument("--case", required=True)
    run.add_argument("--candidate", type=Path, required=True)
    run.add_argument("--session", type=Path, required=True)
    run.add_argument("--gui", action="store_true", help="Explicitly request the visible simulator; default is headless")
    run.add_argument("--plan-only", action="store_true")
    execute = commands.add_parser("execute", help="Internal same-session Action observer, invoked by run.py")
    execute.add_argument("--case", required=True)
    execute.add_argument("--session", type=Path, required=True)
    evaluate = commands.add_parser("evaluate", help="Review actual evidence; absent evidence remains pending")
    evaluate.add_argument("--prepared-config", type=Path, required=True)
    evaluate.add_argument("--session", type=Path, required=True)
    evaluate.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.command == "list":
        value = dict(schema=1, kind="offline_scenario_matrix", cases=list_cases(load_world(args.spec)), actual_execution_performed=False)
    elif args.command == "prepare":
        value = prepare_case(args.spec, args.case, args.output_config)
    elif args.command == "run":
        command = run_command(args.candidate, args.session, args.case, not args.gui)
        if args.plan_only:
            value = dict(command=command, status="pending", execution_performed=False)
        else:
            return subprocess.run(command).returncode
    elif args.command == "execute":
        return execute_case(args.session, args.case)
    else:
        value = evaluate_case(json.loads(args.prepared_config.read_text()), args.session)
        if args.output:
            _new_json(args.output, value)
    print(json.dumps(value, indent=2))
    if args.command == "evaluate":
        return 0 if value["status"] == "passed" else 1 if value["status"] == "failed" else 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
