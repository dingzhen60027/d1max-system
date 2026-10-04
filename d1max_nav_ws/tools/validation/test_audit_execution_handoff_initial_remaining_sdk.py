"""Typed report proofs for SDK first-zero remaining geometry.

ROS-free fixtures audit source identities and original deadlines only. They do
not certify sensing, motion, braking, or physical acceptance.
"""
from copy import deepcopy
import importlib.util
import math
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location(
    "initial_remaining_sdk_fixtures",
    Path(__file__).with_name("test_audit_execution_handoff_initial.py"),
)
INITIAL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(INITIAL)
ns, event, version, check = INITIAL.ns, INITIAL.event, INITIAL.version, INITIAL.check


def remaining_report():
    report = INITIAL.exact_initial_command_report(zero=True)
    ack = INITIAL._first(report, "commit_ack")
    ack["valid_until_ns"] = ns(.13)
    whole = INITIAL._first(report, "validation")
    remaining = dict(deepcopy(whole), elapsed_s=.079, sequence=2,
        whole_curve=False, remaining_curve=True, checked_from_time=.10,
        checked_to_time=1., curve_duration=1., valid_start_time=.20,
        reverse_margin_m=.15, collision_policy="observed_free", frame_id="d1max_loc_odom",
        check_begin_ns=ns(.07), check_end_ns=ns(.075), valid_until_ns=ns(.13),
        body_source_stamp_ns=ns(.075), front_ray_source_stamp_ns=ns(.06),
        rear_ray_source_stamp_ns=ns(.06))
    report["events"].append(remaining)
    x = version()
    progress = event("tracking_progress", .088, schema_version=2,
        source_stamp_ns=ns(.085), valid=True, frame_id="d1max_loc_odom",
        generation=x["reference_generation"], trajectory_id=11,
        **{k:x[k] for k in ("session_id","task_id","route_id","route_hash",
            "map_version_id","localization_epoch","localization_seed_id","segment_id",
            "anchor_id","anchor_revision","context_sequence")},
        curve_time=.20, arc_length=.10, position=[1.,2.,.5], quaternion=[0.,0.,0.,1.],
        linear_velocity=[.02,0.,0.], angular_velocity=[0.,0.,0.])
    report["events"].append(progress)
    body = next(s for s in report["state_sources"]["local"] if s["source_ns"] == ack["body_source_stamp_ns"])
    body.update(usable=True, frame_id="d1max_loc_odom", posterior_ns=body["source_ns"],
        imu_ns=body["source_ns"], position=[1.,2.,.5], quaternion=[0.,0.,0.,1.],
        linear_velocity=[.02,0.,0.], angular_velocity=[0.,0.,0.],
        **{k:x[k] for k in ("session_id","map_version_id","localization_epoch","localization_seed_id")})
    report["events"].sort(key=lambda e:e["elapsed_s"])
    return report


def parts(report):
    proof = next(e for e in report["events"] if e["key"] == "validation" and e["sequence"] == 2)
    progress = next(e for e in report["events"] if e["key"] == "tracking_progress")
    ack = INITIAL._first(report, "commit_ack")
    body = next(s for s in report["state_sources"]["local"] if s["source_ns"] == ack["body_source_stamp_ns"])
    return proof,progress,ack,body


def status(report):
    return check(report, "writer_submission_facts_and_idempotence")["status"]


def test_initial_remaining_zero_binds_actual_install_progress_and_original_cap():
    report = remaining_report()
    assert status(report) == "PASS"
    assert check(report,"typed_tracker_installation_facts")["status"] == "PASS"


def test_initial_remaining_velocity_is_rotated_from_body_to_odom():
    report = remaining_report()
    _,progress,_,body = parts(report)
    body["quaternion"] = [0.,0.,math.sqrt(.5),math.sqrt(.5)]
    body["linear_velocity"] = [.06,0.,0.]
    progress["linear_velocity"] = [0.,.06,0.]
    assert status(report) == "PASS"


@pytest.mark.parametrize("missing", ["progress","body","body_geometry","whole_install","receipt","proof"])
def test_initial_remaining_missing_typed_observation_is_unverified(missing):
    report = remaining_report()
    proof,progress,_,body = parts(report)
    if missing == "progress": report["events"].remove(progress)
    elif missing == "body": report["state_sources"]["local"].remove(body)
    elif missing == "body_geometry": del body["position"]
    elif missing == "whole_install":
        report["events"].remove(INITIAL._first(report,"validation"))
    elif missing == "receipt":
        report["events"].remove(INITIAL._first(report,"tracker_geometry_receipt"))
    else:
        report["events"] = [e for e in report["events"] if e["key"] != "validation"]
    assert status(report) == "UNVERIFIED"


@pytest.mark.parametrize("change", ["scope_both","scope_neither","expired","cap_extended",
    "short_range","short_reverse","from_after_progress","domain_nan","unbound_support",
    "missing_snapshot","wrong_frame","wrong_policy","progress_foreign_task","progress_wrong_generation",
    "progress_wrong_curve","progress_wrong_anchor","progress_invalid","progress_old","progress_future","progress_outside",
    "body_foreign_session","body_invalid","body_postdated","position_mismatch","height_mismatch",
    "velocity_mismatch","invalid_quaternion","nonfinite_body","old_front","old_rear","invalid_fence"])
def test_initial_remaining_wrong_fields_or_expired_evidence_fail(change):
    report = remaining_report()
    proof,progress,ack,body = parts(report)
    if change == "scope_both": proof["whole_curve"] = True
    elif change == "scope_neither": proof["remaining_curve"] = False
    elif change == "expired": proof["valid_until_ns"] = ack["applied_at_ns"]-1
    elif change == "cap_extended": ack["valid_until_ns"] += 1
    elif change == "short_range": proof["checked_to_time"] = .99
    elif change == "short_reverse": proof["reverse_margin_m"] = .149
    elif change == "from_after_progress": proof["checked_from_time"] = .201
    elif change == "domain_nan": proof["curve_duration"] = float("nan")
    elif change == "unbound_support": proof["support_hash"] = ""
    elif change == "missing_snapshot": proof["map_snapshot_revision"] = 0
    elif change == "wrong_frame": proof["frame_id"] = "map"
    elif change == "wrong_policy": proof["collision_policy"] = "unknown_allowed"
    elif change == "progress_foreign_task": progress["task_id"] = "foreign"
    elif change == "progress_wrong_generation": progress["generation"] += 1
    elif change == "progress_wrong_curve": progress["trajectory_id"] += 1
    elif change == "progress_wrong_anchor": progress["anchor_revision"] += 1
    elif change == "progress_invalid": progress["valid"] = False
    elif change == "progress_old": progress["source_stamp_ns"] = ack["applied_at_ns"]-100_000_001
    elif change == "progress_future": progress["source_stamp_ns"] = ack["applied_at_ns"]+1
    elif change == "progress_outside": progress["curve_time"] = 1.01
    elif change == "body_foreign_session": body["session_id"] = "foreign"
    elif change == "body_invalid": body["usable"] = False
    elif change == "body_postdated": body["imu_ns"] = body["source_ns"]+1
    elif change == "position_mismatch": progress["position"][0] += .01251
    elif change == "height_mismatch": progress["position"][2] += .01251
    elif change == "velocity_mismatch": progress["linear_velocity"][1] += .05001
    elif change == "invalid_quaternion": body["quaternion"][3] = .5
    elif change == "nonfinite_body": body["position"][2] = float("nan")
    elif change == "old_front": proof["front_ray_source_stamp_ns"] = ack["applied_at_ns"]-600_000_001
    elif change == "old_rear": proof["rear_ray_source_stamp_ns"] = ack["applied_at_ns"]-600_000_001
    else:
        report["events"].append(dict(deepcopy(proof), sequence=3,valid=False,
            check_end_ns=ns(.08),valid_until_ns=ns(.14),elapsed_s=.089))
    assert status(report) == "FAIL"


def test_initial_zero_cap_ambiguity_is_unverified_not_observer_order_guess():
    report = remaining_report()
    proof,_,_,_ = parts(report)
    report["events"].append(dict(deepcopy(proof),sequence=3,elapsed_s=.082))
    assert status(report) == "UNVERIFIED"


def test_initial_remaining_cannot_replace_original_whole_installation_scope():
    report = remaining_report()
    original = INITIAL._first(report, "validation")
    original["whole_curve"] = False
    original["remaining_curve"] = True
    assert status(report) == "FAIL"


def test_initial_selected_original_proof_cap_is_not_renewed_by_later_native_proof():
    report = remaining_report()
    proof,_,ack,_ = parts(report)
    later = dict(deepcopy(proof),sequence=3,valid_until_ns=ns(.15),
        check_begin_ns=ns(.091),check_end_ns=ns(.095),elapsed_s=.096)
    assert later["check_end_ns"] > ack["applied_at_ns"]
    report["events"].append(later)
    assert status(report) == "PASS"


def test_initial_remaining_proof_deadline_equality_is_expired():
    report = remaining_report()
    proof,_,ack,_ = parts(report)
    proof["valid_until_ns"] = ack["applied_at_ns"]
    ack["valid_until_ns"] = proof["valid_until_ns"]
    assert status(report) == "FAIL"


def test_duplicate_native_sequence_cannot_renew_original_geometry_proof():
    report = remaining_report()
    proof,_,_,_ = parts(report)
    report["events"].append(dict(deepcopy(proof),elapsed_s=.082,
        valid_until_ns=ns(.14)))
    assert status(report) == "FAIL"
