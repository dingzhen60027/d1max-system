"""ROS-free graph-audit regressions for desired/installed/applied separation.

These are report fixtures, not navigation component or physical acceptance.
An ordinary initial permission is an intent until an exact SDK writer fact;
tracker installation and SDK application intentionally remain separate facts.
"""
from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location(
    "handoff_graph_initial_fixtures",
    Path(__file__).with_name("test_audit_execution_handoff_graph.py"),
)
FIXTURES = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FIXTURES)

ns = FIXTURES.ns
event = FIXTURES.event
version = FIXTURES.version
writer = FIXTURES.writer
check = FIXTURES.check


def exact_initial_command_report(*, zero=False):
    """Original initial command/sweep witnesses, independent of install receipt.

    The ACK's entry admission is zero for ordinary initial execution. The
    tracker installation has admission 7; these describe different facts.
    """
    report = FIXTURES.installation_report()
    report["events"] = [item for item in report["events"] if not (
        item["key"] in ("demand", "safe_demand") and item.get("sequence") == 10
        or item["key"] == "motion_validation" and item.get("sequence") == 31)]
    first_ack = next(item for item in report["events"]
                     if item["key"] == "commit_ack" and not item["handoff_id"])
    first_ack.update(vx=0. if zero else .1, wz=0.,
                     motion_validation_sequence=0 if zero else 31,
                     entry_admission_sequence=0)
    initial_proof = next(item for item in report["events"]
                         if item["key"] == "validation" and item["sequence"] == 1)
    initial_proof.update(whole_curve=True, remaining_curve=False,
        checked_from_time=0., checked_to_time=1., curve_duration=1.,
        support_reference_id="support-initial", support_hash="c" * 64,
        map_snapshot_revision=5, check_begin_ns=ns(.03), check_end_ns=ns(.035),
        body_source_stamp_ns=ns(.02), front_ray_source_stamp_ns=ns(.02),
        rear_ray_source_stamp_ns=ns(.02))
    original = event("demand", .083, version=version(), trajectory_id=11,
        sequence=10, permit_sequence=1, validation_sequence=1,
        motion_validation_sequence=0, source_stamp_ns=ns(.08),
        body_source_stamp_ns=ns(.075), valid_until_ns=ns(.18),
        vx=0. if zero else .1, wz=0., hold=zero, safety_checked=False, **writer())
    safe = dict(original, key="safe_demand", elapsed_s=.087,
                motion_validation_sequence=0 if zero else 31, safety_checked=True)
    report["events"].extend([original, safe])
    if not zero:
        report["events"].append(event("motion_validation", .086,
            version=version(), trajectory_id=11, sequence=31, demand_sequence=10,
            permit_sequence=1, trajectory_validation_sequence=1, handoff_id="",
            entry_admission_sequence=0, valid=True, frame_id="d1max_loc_odom",
            demand_source_stamp_ns=ns(.08), demand_body_source_stamp_ns=ns(.075),
            demand_valid_until_ns=ns(.18), check_begin_ns=ns(.081),
            check_end_ns=ns(.085), valid_until_ns=ns(.18), map_snapshot_revision=5,
            body_source_stamp_ns=ns(.085), front_ray_source_stamp_ns=ns(.06),
            rear_ray_source_stamp_ns=ns(.06), vx=.1, wz=0., **writer()))
    report["state_sources"]["local"].extend([
        dict(elapsed_s=.075, source_ns=ns(.075)),
        dict(elapsed_s=.085, source_ns=ns(.085)),
    ])
    report["state_sources"]["local"].sort(key=lambda item: item["elapsed_s"])
    report["events"].sort(key=lambda item: item["elapsed_s"])
    return report


def _first(report, key):
    if key == "commit_ack":
        return next(item for item in report["events"] if item["key"] == key and not item["handoff_id"])
    return next(item for item in report["events"] if item["key"] == key)


def _initial_intent(report, *, trajectory, sequence, installation, start):
    """Insert genuinely admitted/installed but not necessarily applied intent."""
    permit = FIXTURES.permit(1, sequence, start, start + .18)
    permit.update(trajectory_id=trajectory, frame_id="d1max_loc_odom")
    report["events"].extend([
        event("validation", start - .006, version=version(),
              trajectory_id=trajectory, sequence=sequence, valid=True,
              check_end_ns=ns(start - .008), valid_until_ns=ns(start + .17)),
        event("trajectory_admission", start - .002, version=version(),
              trajectory_id=trajectory, sequence=sequence + 100,
              validation_sequence=sequence, accepted=True,
              checked_at_ns=ns(start - .003), valid_until_ns=ns(start + .15)),
        event("permit", start, **permit),
        event("tracker_geometry_receipt", start + .004, version=version(),
              trajectory_id=trajectory, schema_version=1, sequence=sequence,
              installation_sequence=installation, installed=True,
              permit_sequence=sequence, admission_sequence=sequence + 100,
              validation_sequence=sequence, installed_at_ns=ns(start + .001),
              body_source_stamp_ns=ns(round(start / .02) * .02),
              frame_id="d1max_loc_odom", transport_mode="isolated_mock",
              reason="tracker_geometry_installed"),
    ])
    return permit


def unapplied_initial_intent_report():
    report = FIXTURES.installation_report()
    original = FIXTURES.one(report, "tracker_geometry_receipt")
    original["installation_sequence"] = 2
    _initial_intent(report, trajectory=33, sequence=50, installation=1, start=.02)
    report["events"].append(event("bt", .018, phase="tracking",
        writer_commit_sequence=0, execution_id=writer()["execution_id"],
        control_epoch=writer()["control_epoch"]))
    report["events"].sort(key=lambda item: item["elapsed_s"])
    return report


def delayed_initial_ack_restore_report():
    report = FIXTURES.installation_report()
    initial_ack = next(item for item in report["events"]
                       if item["key"] == "commit_ack" and not item["handoff_id"])
    # The SDK actually called A at .09; this fact arrives after another
    # desired initial intent was installed. The original writer timestamps
    # and deadlines do not move when delivery is delayed.
    initial_ack["elapsed_s"] = .20
    _initial_intent(report, trajectory=33, sequence=50, installation=2, start=.105)
    report["events"].extend([
        event("bt", .102, phase="tracking", writer_commit_sequence=0,
              execution_id=writer()["execution_id"],
              control_epoch=writer()["control_epoch"]),
        event("bt", .205, phase="holding", reason="waiting_current_collision_and_tracker_proof",
              writer_commit_sequence=1, execution_id=writer()["execution_id"],
              control_epoch=writer()["control_epoch"]),
        event("tracker_geometry_receipt", .21, version=version(),
              trajectory_id=33, schema_version=1, sequence=51,
              installation_sequence=2, installed=False,
              permit_sequence=50, admission_sequence=150, validation_sequence=50,
              installed_at_ns=ns(.106), body_source_stamp_ns=ns(.20),
              frame_id="d1max_loc_odom", transport_mode="isolated_mock",
              reason="tracker_geometry_no_longer_installed"),
        event("motion", .205, version=version(), trajectory_id=11,
              x=0., yaw=0., observer_receipt_ns=ns(.205), source_stamp_ns=ns(.195),
              **writer()),
    ])
    report["events"].sort(key=lambda item: item["elapsed_s"])
    return report


def test_unapplied_initial_intent_does_not_need_an_sdk_application_ack():
    report = unapplied_initial_intent_report()
    assert check(report, "typed_tracker_installation_facts")["status"] == "PASS"
    assert check(report, "active_geometry_only_after_applied_ack")["status"] == "PASS"


def test_late_initial_ack_restores_actual_identity_without_new_install_fact():
    report = delayed_initial_ack_restore_report()
    assert check(report, "writer_submission_facts_and_idempotence")["status"] == "PASS"
    assert check(report, "typed_tracker_installation_facts")["status"] == "PASS"
    assert check(report, "active_geometry_only_after_applied_ack")["status"] == "PASS"


def test_actual_motion_on_unapplied_initial_intent_still_requires_exact_ack():
    report = unapplied_initial_intent_report()
    report["events"].append(event("motion", .03, version=version(), trajectory_id=33,
        x=.1, yaw=0., observer_receipt_ns=ns(.03), source_stamp_ns=ns(.025), **writer()))
    report["events"].sort(key=lambda item: item["elapsed_s"])
    assert check(report, "active_geometry_only_after_applied_ack")["status"] == "FAIL"


def test_sdk_actual_geometry_still_requires_ack_when_initial_intent_exists():
    report = FIXTURES.installation_report()
    report["events"].append(event("bt", .13, phase="tracking", writer_commit_sequence=1,
        execution_id=writer()["execution_id"], control_epoch=writer()["control_epoch"]))
    foreign = _initial_intent(report, trajectory=33, sequence=50, installation=2, start=.14)
    assert foreign["geometry_committed"] and foreign["allowed"]
    report["events"].append(event("sdk", .15, version=version(), applied_trajectory_id=33,
        writer_commit_sequence=2, phase="ready", source_stamp_ns=ns(.15), **writer()))
    report["events"].sort(key=lambda item: item["elapsed_s"])
    assert check(report, "active_geometry_only_after_applied_ack")["status"] == "FAIL"


def test_late_initial_ack_does_not_authorize_fake_reinstallation_with_expired_entry():
    report = delayed_initial_ack_restore_report()
    forged = deepcopy(FIXTURES.one(report, "tracker_geometry_receipt"))
    forged.update(elapsed_s=.23, installation_sequence=3, sequence=52,
                  installed_at_ns=ns(.22), body_source_stamp_ns=ns(.22))
    report["events"].append(forged)
    report["events"].sort(key=lambda item: item["elapsed_s"])
    assert check(report, "typed_tracker_installation_facts")["status"] == "FAIL"


def test_initial_writer_call_binds_original_command_and_exact_native_sweep():
    report = exact_initial_command_report()
    ack = _first(report, "commit_ack")
    receipt = _first(report, "tracker_geometry_receipt")
    assert ack["entry_admission_sequence"] == 0 and receipt["admission_sequence"] == 7
    assert check(report, "writer_submission_facts_and_idempotence")["status"] == "PASS"


def upgraded_initial_command_report():
    report = exact_initial_command_report()
    safe, proof, ack = (_first(report, key) for key in
                        ("safe_demand", "motion_validation", "commit_ack"))
    later_safe = dict(safe, elapsed_s=.09, motion_validation_sequence=32)
    later_proof = dict(proof, elapsed_s=.089, sequence=32)
    report["events"].extend([later_safe, later_proof])
    ack["motion_validation_sequence"] = 32
    report["events"].sort(key=lambda item: item["elapsed_s"])
    return report


def test_initial_writer_binds_later_exact_signature_without_renewing_command():
    assert check(upgraded_initial_command_report(),
                 "writer_submission_facts_and_idempotence")["status"] == "PASS"


@pytest.mark.parametrize("field", ["source_stamp_ns", "body_source_stamp_ns", "valid_until_ns", "vx", "wz", "permit_sequence"])
def test_initial_signature_upgrade_cannot_hide_changed_original_payload(field):
    report = upgraded_initial_command_report()
    _first(report, "safe_demand")[field] += 1
    assert check(report, "writer_submission_facts_and_idempotence")["status"] == "FAIL"


def test_initial_unobserved_signature_is_unverified_not_selected_by_first_receipt():
    report = upgraded_initial_command_report()
    report["events"] = [item for item in report["events"] if not (
        item["key"] == "safe_demand" and item["motion_validation_sequence"] == 32)]
    assert check(report, "writer_submission_facts_and_idempotence")["status"] == "UNVERIFIED"


@pytest.mark.parametrize("field", ["vx", "wz"])
def test_repeated_initial_ack_cannot_change_original_applied_velocity(field):
    report = exact_initial_command_report()
    forged = deepcopy(_first(report, "commit_ack"))
    forged["elapsed_s"] = .12
    forged[field] += .01
    report["events"].append(forged)
    report["events"].sort(key=lambda item: item["elapsed_s"])
    assert check(report, "writer_submission_facts_and_idempotence")["status"] == "FAIL"


@pytest.mark.parametrize("missing", ["demand", "safe_demand", "motion_validation"])
def test_initial_writer_call_without_exact_observation_is_unverified(missing):
    report = exact_initial_command_report()
    report["events"].remove(_first(report, missing))
    assert check(report, "writer_submission_facts_and_idempotence")["status"] == "UNVERIFIED"


@pytest.mark.parametrize("mismatch", [
    "safe_source", "safe_body", "safe_cap", "safe_velocity", "safe_permit",
    "original_source", "proof_permit", "proof_body", "proof_cap", "proof_version",
    "proof_writer", "proof_velocity", "proof_invalid", "proof_expired", "proof_old_front", "proof_old_rear",
    "proof_snapshot_missing", "ack_extends_cap", "ack_velocity", "nonzero_without_sweep",
])
def test_initial_writer_call_mismatched_fields_or_extended_deadline_fails(mismatch):
    report = exact_initial_command_report()
    demand = _first(report, "demand")
    safe = _first(report, "safe_demand")
    proof = _first(report, "motion_validation")
    ack = _first(report, "commit_ack")
    if mismatch == "safe_source": safe["source_stamp_ns"] += 1
    elif mismatch == "safe_body": safe["body_source_stamp_ns"] += 1
    elif mismatch == "safe_cap": safe["valid_until_ns"] += 1
    elif mismatch == "safe_velocity": safe["vx"] += .01
    elif mismatch == "safe_permit": safe["permit_sequence"] += 1
    elif mismatch == "original_source": demand["source_stamp_ns"] += 1
    elif mismatch == "proof_permit": proof["permit_sequence"] += 1
    elif mismatch == "proof_body": proof["demand_body_source_stamp_ns"] += 1
    elif mismatch == "proof_cap": proof["demand_valid_until_ns"] += 1
    elif mismatch == "proof_version": proof["version"]["context_sequence"] += 1
    elif mismatch == "proof_writer": proof["sdk_arm_generation"] += 1
    elif mismatch == "proof_velocity": proof["vx"] += .01
    elif mismatch == "proof_invalid": proof["valid"] = False
    elif mismatch == "proof_expired": proof["valid_until_ns"] = ack["applied_at_ns"] - 1
    elif mismatch == "proof_old_front": proof["front_ray_source_stamp_ns"] = ack["applied_at_ns"] - 500_000_001
    elif mismatch == "proof_old_rear": proof["rear_ray_source_stamp_ns"] = ack["applied_at_ns"] - 500_000_001
    elif mismatch == "proof_snapshot_missing": proof["map_snapshot_revision"] = 0
    elif mismatch == "ack_extends_cap": ack["valid_until_ns"] += 1
    elif mismatch == "ack_velocity": ack["vx"] += .01
    else: ack["motion_validation_sequence"] = 0; safe["motion_validation_sequence"] = 0
    assert check(report, "writer_submission_facts_and_idempotence")["status"] == "FAIL"


def test_initial_zero_writer_call_still_has_exact_command_and_whole_curve_proof():
    report = exact_initial_command_report(zero=True)
    assert check(report, "writer_submission_facts_and_idempotence")["status"] == "PASS"


@pytest.mark.parametrize("mismatch", ["not_whole", "remaining", "short_range", "no_support", "expired", "ack_extends_native_cap", "missing"])
def test_initial_zero_does_not_bypass_full_candidate_geometry(mismatch):
    report = exact_initial_command_report(zero=True)
    proof = _first(report, "validation")
    if mismatch == "not_whole": proof["whole_curve"] = False
    elif mismatch == "remaining": proof["remaining_curve"] = True
    elif mismatch == "short_range": proof["checked_to_time"] = .99
    elif mismatch == "no_support": proof["support_hash"] = ""
    elif mismatch == "expired": proof["valid_until_ns"] = _first(report, "commit_ack")["applied_at_ns"] - 1
    elif mismatch == "ack_extends_native_cap": proof["valid_until_ns"] = ns(.12)
    else: report["events"].remove(proof)
    expected = "UNVERIFIED" if mismatch == "missing" else "FAIL"
    assert check(report, "writer_submission_facts_and_idempotence")["status"] == expected
