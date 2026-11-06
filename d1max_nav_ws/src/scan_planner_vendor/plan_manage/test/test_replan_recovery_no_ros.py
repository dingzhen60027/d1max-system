#!/usr/bin/env python3
"""Compile production native failure/recovery methods; no ROS or substitute map.

The full-curve recheck result is an explicit seam, covered separately by
test_preview_revalidation_no_ros.py. This checks actual FSM use of that result
and real measured boundary/source changes, not obstacle feasibility.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


HARNESS = r'''
#include <plan_manage/input_contract.hpp>
#include <plan_manage/candidate_adoption.hpp>
#include <plan_env/pending_snapshot_request.hpp>
#include <chrono>
#include <memory>
#include <iostream>
#include <limits>
#include <future>
#include <optional>
#include <array>
#define RCLCPP_WARN(...) ((void)0)
struct ClockValue {
  std::int64_t ns=500000000000LL;
  double seconds() const {return ns*1e-9;}
  std::int64_t nanoseconds() const {return ns;}
  ClockValue operator-(ClockValue b) const {return {ns-b.ns};}
  bool operator>(ClockValue b)const{return ns>b.ns;}
};
namespace rclcpp{using Time=ClockValue;}
struct Node {ClockValue stamp; ClockValue now() const {return stamp;}};
struct Map {
  std::uint64_t revision=7; double source=499.8;
  std::array<std::int64_t,2> rays{{499800000000LL,499800000000LL}};
  std::uint64_t occupancyRevision() const {return revision;}
  std::int64_t integratedRaySourceStamp(unsigned i)const{return rays[i];}
  bool integratedCloudFreshAt(std::int64_t at)const{return at>=source*1e9&&at-source*1e9<=500000000.;}
  double latestCloudStamp() const {return source;}
  double getResolution() const {return .08;}
};
struct Manager {
  struct {double max_vel_=.3;} pp_;
  struct {double duration_=4.;std::int64_t traj_id_=1;Eigen::Vector3d start_pos_{0,0,.55};} local_data_;
  bool preview=true;double progress=0.;
  bool hasPreviewHeadingContract() const {return preview;}
  double previewCurveProgressTime() const {return progress;}
  std::shared_ptr<Map> grid_map_=std::make_shared<Map>();
};
namespace scan_planner {
// The exact one-lock ledger predicate is exercised by test_execution_ledger.
// Here only its read-only result and immutable reference identity are seams;
// the FSM's real periodic/event/worker scheduling method is compiled verbatim.
struct ReferenceMetadata {std::uint64_t generation=1;};
inline std::uint64_t executionVersion(const ReferenceMetadata& p){return p.generation;}
class SCANReplanFSM {
public:
  bool deferred_rolling_proposal_=false;
  std::chrono::steady_clock::time_point rolling_retry_priority_until_{};
  enum class PreviewRecheck {Unavailable,Stale,Safe,Unsafe,Uncertified};
  enum State {EXEC_TRAJ,WAIT_ENVIRONMENT,REPLAN_TRAJ,GEN_NEW_TRAJ};
  Node clock; Node *node_=&clock;
  struct Validator {
    struct PendingSpliceDisposition {bool keep=false;};
    struct View {
      struct {std::uint64_t generation=1;struct{std::int64_t traj_id=1;}trajectory;} spline;
      struct Proof {bool valid=true;ClockValue valid_until{500200000000LL};};
      struct Progress {bool valid=true,holding=false;struct{ClockValue stamp{499900000000LL};}header;
        struct{struct{double x=0.,y=0.,z=0.;}linear;}twist;};
      std::optional<Proof> proof{Proof{}};std::optional<Progress>progress{Progress{}};
    };
    std::optional<View> value{View{}};
    bool pending=false,body_stopped=true;
    bool splice_keep=false;
    mutable unsigned scheduling_queries=0;
    std::optional<View> committedView()const{return value;}
    PendingSpliceDisposition pendingSpliceDispositionAt(std::uint64_t,std::int64_t,double)const {
      return {splice_keep}; // exact native predicate is tested by test_local_follower_native.
    }
    bool keepCommittedIdleHorizonAt(std::uint64_t expected,std::int64_t now,double age,double advance)const {
      ++scheduling_queries;
      if(!value||pending||!body_stopped||expected!=value->spline.generation||age<=0.||advance<=0.||
         !value->proof||!value->proof->valid||value->proof->valid_until.ns<=now||
         !value->progress||!value->progress->valid||value->progress->holding||
         now-value->progress->header.stamp.ns>400000000LL)return false;
      return std::abs(value->progress->twist.linear.x)<.01;
    }
  };
  std::shared_ptr<Validator> execution_validator_;
  ReferenceMetadata reference_metadata_;
  bool blocked_entry_replan_pending_=false,force_visible_side_target_=false;
  bool local_follower_=true;
  unsigned ordinary_progress_triggers=0;
  double max_replan_interval_=.2;
  std::chrono::steady_clock::time_point last_solve_submit_{};
  std::uint64_t reference_generation_=1;
  std::future<int> solve_future_;
  bool solveWorkPending()const{return solve_future_.valid();}
  PendingSnapshotRequest pending_snapshot_request_;
  struct Pool {int cancellations=0;void cancelSolverRequest(){++cancellations;}}snapshot_pool_;
  std::shared_ptr<Manager> planner_manager_=std::make_shared<Manager>();
  bool have_odom_=true;
  bool planning_only_=false;
  std::optional<double> planning_curve_time_{0.};
  std::optional<double> planningOnlyCommittedCurveTime()const{return planning_curve_time_;}
  bool go2_execution_frozen_=true,reference_path_guidance_=true,have_target_=true;
  double planning_horizon_=4.,failed_replan_cooldown_=.5,min_replan_interval_=.1;
  ClockValue last_replan_time_{499900000000LL};
  ClockValue last_odom_time_{499900000000LL};
  std::int64_t failed_body_source_ns_=499800000000LL;
  double failed_replan_monotonic_=0,failed_map_stamp_=499.8,odom_timeout_=.5;
  std::uint64_t failed_environment_revision_=7;
  std::array<std::int64_t,2> failed_ray_stamps_{{0,0}};
  double failed_replan_body_distance_=.15;
  Eigen::Vector3d failed_body_position_{0,0,.55},odom_pos_{0,0,.55};
  Eigen::Vector3d failed_body_velocity_{0,0,0},odom_vel_{0,0,0};
  Eigen::Quaterniond failed_body_orientation_=Eigen::Quaterniond::Identity();
  Eigen::Quaterniond odom_orient_=Eigen::Quaterniond::Identity();
  std::string last_attempt_failure_phase_="failed_optimization",published_phase;
  PreviewRecheck result=PreviewRecheck::Safe;
  State exec_state_=EXEC_TRAJ; int rechecks=0, invalidations=0,replan_fail_count_=0;
  PreviewRecheck revalidatePreviewIncumbent() {++rechecks;return result;}
  void publishInvalidLocalPlanDebug(const std::string &phase,bool=false) {
    ++invalidations; published_phase=phase;
  }
  void changeFSMExecState(State next,const std::string &) {exec_state_=next;}
  void waitForChangedEnvironment();
  void cancelSnapshotAcquisition();
  bool snapshotAcquisitionCurrent();
  bool failedAttemptRetryReady(double monotonic_now);
  bool environmentChangedSinceFailure();
  bool keepPreviewIncumbentForPeriodicReplan();
  bool keepPendingSpliceForOrdinaryReplan();
  void ordinaryReplanTrigger();
  void planningDispatchFailure(State mode);
  bool shouldReplanUncertifiedPreview();
};
__METHODS__
__ORDINARY_INGRESS__
__DISPATCH_FAILURE__
}
int main() {
  using FSM=scan_planner::SCANReplanFSM; using R=FSM::PreviewRecheck;
  int checks=0,failures=0;
  const auto expect=[&](bool ok,const char *name) {
    ++checks;failures+=!ok;std::cout<<(ok?"PASS ":"FAIL ")<<name<<"\n";
  };
  for(auto state:{FSM::GEN_NEW_TRAJ,FSM::REPLAN_TRAJ})for(bool guidance:{false,true}) {
    FSM f;f.exec_state_=state;f.reference_path_guidance_=guidance;
    f.last_attempt_failure_phase_="waiting_solve_budget";
    for(int poll=0;poll<25;++poll)f.planningDispatchFailure(state);
    expect(f.replan_fail_count_==0&&f.invalidations==0&&f.rechecks==0&&f.exec_state_==state,
      "production_dispatch_rate_guard_100hz_polls_are_not_failed_attempts");
    f.replan_fail_count_=f.invalidations=f.rechecks=0;f.exec_state_=state;
    std::promise<int> worker;f.solve_future_=worker.get_future();
    f.last_attempt_failure_phase_="failed_optimization";
    for(int poll=0;poll<15;++poll)f.planningDispatchFailure(state);
    expect(f.replan_fail_count_==0&&f.invalidations==0&&f.exec_state_==state,
      "production_dispatch_existing_worker_preserves_old_attempt_count");
    f.replan_fail_count_=f.invalidations=f.rechecks=0;f.exec_state_=state;
    worker.set_value(0);f.solve_future_.get();
    auto now=scan_planner::SolveBudget::Time{};
    f.pending_snapshot_request_.begin(f.reference_generation_,[&]{return now;});
    f.last_attempt_failure_phase_="waiting_snapshot_slot";
    for(int poll=0;poll<15;++poll)f.planningDispatchFailure(state);
    expect(f.replan_fail_count_==0&&f.invalidations==0&&f.exec_state_==state,
      "production_dispatch_pending_snapshot_is_not_repeated_failure");
    f.replan_fail_count_=f.invalidations=f.rechecks=0;f.exec_state_=state;
    f.pending_snapshot_request_.cancel();
    f.last_attempt_failure_phase_="failed_optimization";
    f.planningDispatchFailure(state);
    expect(f.replan_fail_count_==1&&f.exec_state_==(guidance?FSM::WAIT_ENVIRONMENT:state)&&
      f.invalidations==(!guidance?1:0),"production_dispatch_actual_failed_turn_counts_once");
  }
  {
    FSM f;auto now=scan_planner::SolveBudget::Time{};
    const auto budget=f.pending_snapshot_request_.begin(f.reference_generation_,[&]{return now;});
    f.last_attempt_failure_phase_="waiting_snapshot_slot";
    f.waitForChangedEnvironment();
    expect(f.exec_state_==FSM::EXEC_TRAJ&&f.rechecks==0,
      "pending_acquisition_is_not_half_second_environment_failure_or_clear_active");
    now+=std::chrono::milliseconds(120);
    expect(f.snapshotAcquisitionCurrent(),"same_input_slot_release_can_resume_inside_original_deadline");
    expect(f.pending_snapshot_request_.begin(f.reference_generation_,[&]{return now;})==budget&&
      budget->remainingSeconds()<.281,"resource_retry_keeps_original_budget_not_new_four_hundred_ms");
    const auto side=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(80),[&]{return now;},budget);
    expect(side->remainingSeconds()>.079,"side_compute_quota_starts_after_resource_wait");
    now+=std::chrono::milliseconds(280);
    expect(!f.snapshotAcquisitionCurrent()&&!f.pending_snapshot_request_.pending()&&
      f.last_attempt_failure_phase_=="solve_deadline_exceeded"&&f.snapshot_pool_.cancellations==1,
      "expired_original_acquisition_withdraws_arbiter_intent_never_starts_solver");
    expect(!side->allowed(),"side_work_does_not_outlive_original_wait_inclusive_deadline");
    f.waitForChangedEnvironment();
    const double failed=f.failed_replan_monotonic_;
    expect(!f.failedAttemptRetryReady(failed+.499),
      "expired_acquisition_never_restarts_before_existing_half_second_cooldown");
    expect(!f.environmentChangedSinceFailure()&&f.failedAttemptRetryReady(failed+.501),
      "expired_acquisition_recovers_compute_without_moving_stopped_body_or_changing_saturated_map");
    expect(f.exec_state_==FSM::WAIT_ENVIRONMENT&&f.rechecks==1&&f.invalidations==0&&
      f.planner_manager_->local_data_.traj_id_==1,
      "scheduling_timeout_keeps_separately_revalidated_incumbent_not_fake_motion_authority");
    const auto next=f.pending_snapshot_request_.begin(f.reference_generation_,[&]{return now;});
    expect(next!=budget&&next->remainingSeconds()>.399&&!budget->allowed()&&!side->allowed(),
      "retry_is_distinct_turn_old_parent_and_subbudget_remain_expired");
  }
  {
    FSM f;auto now=scan_planner::SolveBudget::Time{};
    const auto worker_budget=f.pending_snapshot_request_.begin(f.reference_generation_,[&]{return now;});
    f.pending_snapshot_request_.release(); // Production transfers this token to one worker.
    std::promise<int> worker;f.solve_future_=worker.get_future();
    now+=std::chrono::milliseconds(400);
    f.last_attempt_failure_phase_=worker_budget->reason();
    f.waitForChangedEnvironment();
    expect(f.exec_state_==FSM::EXEC_TRAJ&&f.rechecks==0&&f.invalidations==0&&
      !scan_planner::formalReseedSubmissionDue(std::chrono::steady_clock::now(),{},f.solve_future_.valid()),
      "deadline_does_not_retire_running_worker_or_allow_second_concurrent_solve");
    worker.set_value(0);f.solve_future_.get(); // Actual poll consumes the retired result once.
    f.waitForChangedEnvironment();
    const double failed=f.failed_replan_monotonic_;
    expect(!f.failedAttemptRetryReady(failed+.499)&&f.failedAttemptRetryReady(failed+.501)&&
      !f.environmentChangedSinceFailure()&&!worker_budget->allowed()&&f.invalidations==0,
      "retired_worker_timeout_recovers_after_cooldown_without_clearing_safe_old_curve");
  }
  for(const auto* phase:{"waiting_observed_space","failed_final_collision","failed_reference_target_occupied",
      "failed_reference_geometry","solve_cancelled","solve_exception","solve_launch_failed"}) {
    FSM f;f.last_attempt_failure_phase_=phase;f.waitForChangedEnvironment();
    expect(!scan_planner::nativeResourceWait(phase)&&
      !f.failedAttemptRetryReady(f.failed_replan_monotonic_+1.),
      "geometry_unknown_cancel_or_unclassified_error_never_borrows_compute_retry");
  }
  for(const auto* phase:{"failed_optimization","failed_rebound_search","failed_dynamics"}) {
    FSM f;f.last_attempt_failure_phase_=phase;f.failed_replan_cooldown_=.1;
    f.waitForChangedEnvironment();const auto failed=f.failed_replan_monotonic_;
    auto& map=*f.planner_manager_->grid_map_;
    expect(!f.failedAttemptRetryReady(failed+.099)&&!f.failedAttemptRetryReady(failed+.101),
      "numerical_miss_does_not_busy_retry_same_old_input_or_skip_configured_cooldown");
    map.source+=.05;map.rays[0]=map.rays[1]=std::llround(map.source*1e9);
    expect(f.failedAttemptRetryReady(failed+.101)&&map.revision==f.failed_environment_revision_&&
      f.odom_pos_.isApprox(f.failed_body_position_)&&f.planner_manager_->local_data_.traj_id_==1&&f.invalidations==0,
      "fresh_real_dual_input_retries_numerical_failure_without_15cm_motion_or_occupancy_change");
    f.clock.stamp.ns+=1000000000;
    expect(!f.failedAttemptRetryReady(failed+1.),
      "numerical_retry_cannot_borrow_expired_planning_input_or_grant_old_motion");
  }
  {
    FSM f;f.execution_validator_=std::make_shared<FSM::Validator>();
    f.execution_validator_->splice_keep=true;
    f.clock.stamp.ns=f.last_replan_time_.ns+50000000;
    // Actual production EXEC prefix runs BEFORE executed-distance handling.
    f.ordinaryReplanTrigger();
    expect(f.ordinary_progress_triggers==0&&f.exec_state_==FSM::EXEC_TRAJ,
      "pending_future_tail_survives_nonperiodic_progress_tick_instead_of_moving_seam_forward");
    const auto after_first=f.ordinary_progress_triggers;
    f.clock.stamp.ns=f.last_replan_time_.ns+201000000;
    f.ordinaryReplanTrigger();
    expect(f.ordinary_progress_triggers==after_first,
      "same_pending_tail_also_coalesces_nominal_periodic_tick");
    f.force_visible_side_target_=true;
    f.ordinaryReplanTrigger();
    expect(f.ordinary_progress_triggers==after_first+1,
      "danger_side_request_is_never_hidden_by_ordinary_pending_coalescing");
    f.force_visible_side_target_=false;f.execution_validator_->splice_keep=false;
    f.ordinaryReplanTrigger();
    expect(f.ordinary_progress_triggers==after_first+2,
      "missed_expired_or_invalid_native_pending_verdict_releases_ordinary_replan");
  }
  {
    FSM f;f.execution_validator_=std::make_shared<FSM::Validator>();
    f.clock.stamp.ns=f.last_replan_time_.ns+201000000;
    f.deferred_rolling_proposal_=true;f.ordinaryReplanTrigger();
    expect(f.exec_state_==FSM::EXEC_TRAJ&&f.ordinary_progress_triggers==0,
      "queued_rolling_window_has_priority_over_old_periodic_solve");
    f.deferred_rolling_proposal_=false;
    f.rolling_retry_priority_until_=std::chrono::steady_clock::now()+std::chrono::milliseconds(650);
    f.ordinaryReplanTrigger();
    expect(f.exec_state_==FSM::EXEC_TRAJ&&f.ordinary_progress_triggers==0,
      "one_fresh_upstream_retry_cannot_be_starved_by_another_old_window_splice");
    f.force_visible_side_target_=true;f.ordinaryReplanTrigger();
    expect(f.exec_state_==FSM::REPLAN_TRAJ,
      "real_collision_side_request_bypasses_rolling_retry_priority");
    f.force_visible_side_target_=false;f.exec_state_=FSM::EXEC_TRAJ;
    f.execution_validator_->body_stopped=false;f.odom_vel_.x()=.3;
    f.rolling_retry_priority_until_=std::chrono::steady_clock::now()-std::chrono::milliseconds(1);
    f.ordinaryReplanTrigger();
    expect(f.exec_state_==FSM::REPLAN_TRAJ,
      "retry_priority_is_bounded_and_does_not_permanently_disable_old_window_replanning");
  }
  {
    FSM f;f.planning_only_=true;
    f.last_attempt_failure_phase_=scan_planner::candidateAcceptanceReason(
      scan_planner::CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed);
    f.waitForChangedEnvironment();const double failed=f.failed_replan_monotonic_;
    expect(!f.environmentChangedSinceFailure()&&!f.failedAttemptRetryReady(failed+.499)&&
      f.failedAttemptRetryReady(failed+.501)&&f.planner_manager_->local_data_.traj_id_==1&&f.invalidations==0,
      "planning_only_connector_miss_can_reseed_unchanged_stopped_body_after_existing_cooldown_with_incumbent_retained");
    f.planning_only_=false;
    expect(!f.failedAttemptRetryReady(failed+.501),
      "motion_environment_retry_cannot_borrow_preview_connector_reseed");
  }
  for(const auto outcome:{scan_planner::CandidateAcceptanceOutcome::PreviewSourceExpired,
      scan_planner::CandidateAcceptanceOutcome::PreviewSourceChanged,
      scan_planner::CandidateAcceptanceOutcome::PreviewGeometryBlocked}) {
    FSM f;f.planning_only_=true;f.last_attempt_failure_phase_=scan_planner::candidateAcceptanceReason(outcome);
    f.waitForChangedEnvironment();const double failed=f.failed_replan_monotonic_;auto& map=*f.planner_manager_->grid_map_;
    expect(!scan_planner::nativeResourceWait(f.last_attempt_failure_phase_)&&
      !f.failedAttemptRetryReady(failed+1.),
      "preview_source_or_geometry_failure_cannot_retry_unchanged_evidence_by_timer");
    map.source+=.1;map.rays[0]+=100000000;
    expect(!f.failedAttemptRetryReady(failed+1.),
      "preview_failure_front_only_evidence_cannot_wake_environment_retry");
    map.rays[1]+=100000000;
    expect(f.failedAttemptRetryReady(failed+1.)&&f.invalidations==0&&f.planner_manager_->local_data_.traj_id_==1,
      "preview_source_or_geometry_failure_retries_with_new_fresh_dual_map_evidence_not_new_trajectory_authority");
    f.clock.stamp.ns+=1000000000;
    expect(!f.failedAttemptRetryReady(failed+2.),
      "preview_failure_already_expired_dual_evidence_cannot_recover");
  }
  for(const auto outcome:{scan_planner::CandidateAcceptanceOutcome::InvalidPreviewCandidate,
      scan_planner::CandidateAcceptanceOutcome::PreviewSegmentBoundary,
      scan_planner::CandidateAcceptanceOutcome::PreviewBudgetExhausted}) {
    FSM f;f.planning_only_=true;f.last_attempt_failure_phase_=scan_planner::candidateAcceptanceReason(outcome);
    f.waitForChangedEnvironment();auto& map=*f.planner_manager_->grid_map_;
    map.source+=.1;for(auto& stamp:map.rays)stamp+=100000000;
    expect(!f.environmentChangedSinceFailure()&&!f.failedAttemptRetryReady(f.failed_replan_monotonic_+1.),
      "preview_unclassified_boundary_or_budget_failure_does_not_expand_source_geometry_retry_scope");
  }
  {
    FSM f;f.last_attempt_failure_phase_="solve_search_deadline_exceeded";
    f.waitForChangedEnvironment();const double failed=f.failed_replan_monotonic_;
    expect(scan_planner::nativeResourceWait(f.last_attempt_failure_phase_) &&
      !f.environmentChangedSinceFailure() && !f.failedAttemptRetryReady(failed+.499) &&
      f.failedAttemptRetryReady(failed+.501),
      "inner_astar_compute_deadline_retries_same_fresh_map_at_existing_two_hz_limit");
    expect(f.invalidations==0 && f.planner_manager_->local_data_.traj_id_==1 && f.rechecks==1,
      "inner_search_timeout_preserves_only_independently_rechecked_incumbent");
  }
  for(int boundary=0;boundary<4;++boundary) {
    FSM f;auto now=scan_planner::SolveBudget::Time{};
    const auto budget=f.pending_snapshot_request_.begin(f.reference_generation_,[&]{return now;});
    if(boundary==0)++f.reference_generation_;
    if(boundary==1)f.have_target_=false;
    if(boundary==2)f.last_odom_time_.ns=f.clock.stamp.ns-501000000;
    if(boundary==3)f.planner_manager_->grid_map_->source=f.clock.stamp.seconds()-.501;
    expect(!f.snapshotAcquisitionCurrent()&&!f.pending_snapshot_request_.pending()&&!budget->allowed()&&
      f.snapshot_pool_.cancellations==1,
      "route_cancel_body_or_map_source_boundary_cannot_resume_old_acquisition");
    expect(f.planner_manager_->local_data_.traj_id_==1&&f.invalidations==0,
      "acquisition_boundary_preserves_installed_geometry_without_faking_safe_proof");
  }
  {
    FSM f;auto now=scan_planner::SolveBudget::Time{};
    const auto old=f.pending_snapshot_request_.begin(f.reference_generation_,[&]{return now;});
    f.cancelSnapshotAcquisition();now+=std::chrono::milliseconds(10);
    expect(!old->allowed()&&!f.snapshotAcquisitionCurrent()&&!f.pending_snapshot_request_.pending(),
      "cancel_wins_even_if_slot_release_races_before_original_deadline");
  }
  {
    using Clock=std::chrono::steady_clock;
    using scan_planner::formalReseedSubmissionDue;
    const auto last=Clock::time_point{}+std::chrono::seconds(10);
    expect(formalReseedSubmissionDue(last,{},false),"event_reseed_without_prior_solve_can_start");
    expect(!formalReseedSubmissionDue(last+std::chrono::milliseconds(99),last,false),
      "event_reseed_cannot_exceed_configured_ten_hz_submission_ceiling");
    expect(formalReseedSubmissionDue(last+std::chrono::milliseconds(100),last,false),
      "entry_failure_need_not_wait_half_second_after_fast_solve");
    expect(!formalReseedSubmissionDue(last+std::chrono::seconds(20),last,true),
      "repeated_entry_events_never_create_a_second_worker_or_queue");
    expect(!formalReseedSubmissionDue(last-std::chrono::seconds(1),last,false),
      "monotonic_clock_regression_does_not_make_event_due");
  }
  {
    FSM f;f.waitForChangedEnvironment();
    expect(f.exec_state_==FSM::WAIT_ENVIRONMENT && f.rechecks==1 && f.invalidations==0,
      "failed_candidate_keeps_only_whole_curve_revalidated_incumbent");
    expect(f.failed_body_source_ns_==f.last_odom_time_.nanoseconds() &&
      f.failed_body_orientation_.isApprox(f.odom_orient_),"failure_records_actual_body_source_and_orientation");
    expect(!f.environmentChangedSinceFailure(),"unchanged_evidence_does_not_busy_retry");
    f.odom_orient_=Eigen::Quaterniond(Eigen::AngleAxisd(.15,Eigen::Vector3d::UnitZ()));
    expect(!f.environmentChangedSinceFailure(),"orientation_without_new_source_is_not_recovery");
    f.last_odom_time_.ns+=1000000;
    expect(f.environmentChangedSinceFailure(),"real_rotation_in_place_retries_without_map_or_translation");
  }
  for(const std::string phase:{"failed_curve_validation","waiting_splice_validation","waiting_splice_incumbent",
      "candidate_measured_join_rejected","candidate_source_expired","candidate_source_changed",
      "waiting_measured_target_heading"}) {
    FSM f;f.last_attempt_failure_phase_=phase;f.failed_replan_cooldown_=.2;
    f.waitForChangedEnvironment();const double failed=f.failed_replan_monotonic_;
    expect(!f.environmentChangedSinceFailure(),"source_pair_failure_cannot_reuse_same_sample");
    f.last_odom_time_.ns+=20000000LL;
    expect(f.environmentChangedSinceFailure(),"fresh_body_pair_retries_without_forcing_robot_to_move_first");
    const bool rolling=scan_planner::nativeLatestStateRetry(phase);
    expect(f.failedAttemptRetryReady(failed+.199)==rolling&&f.failedAttemptRetryReady(failed+.201),
      "rolling_candidate_retry_skips_environment_backoff_legacy_splice_unchanged");
    expect(f.planner_manager_->local_data_.traj_id_==1&&f.invalidations==0,
      "connection_retry_preserves_safe_incumbent");
  }
  {
    FSM f;f.min_replan_interval_=0.;f.failed_replan_cooldown_=.2;
    f.last_attempt_failure_phase_="candidate_measured_join_rejected";
    f.waitForChangedEnvironment();const double failed=f.failed_replan_monotonic_;
    expect(!f.failedAttemptRetryReady(failed+.01),"rolling_retry_needs_new_actual_input");
    f.last_odom_time_.ns+=20000000LL;
    expect(f.failedAttemptRetryReady(failed+.01),"rolling_retry_wakes_next_tick_not_200ms_later");
    f.clock.stamp.ns+=1000000000LL;
    expect(!f.failedAttemptRetryReady(failed+.02),"rolling_retry_cannot_use_expired_body_or_map");
    f.last_odom_time_=f.clock.stamp;
    expect(!f.failedAttemptRetryReady(failed+.03),"fresh_body_cannot_busy_retry_expired_map");
    expect(f.invalidations==0&&f.planner_manager_->local_data_.traj_id_==1,
      "rolling_retry_never_clears_healthy_incumbent");
  }
  {
    FSM f;f.last_attempt_failure_phase_="waiting_measured_target_heading";
    f.waitForChangedEnvironment();const auto failure=f.failed_replan_monotonic_;
    expect(!f.failedAttemptRetryReady(failure+1.),
      "target_heading_wait_does_not_reuse_duplicate_body_or_receipt");
    f.last_odom_time_.ns+=1000000;f.clock.stamp.ns+=1000000000;
    expect(!f.failedAttemptRetryReady(failure+1.),
      "target_heading_wait_does_not_accept_new_but_expired_body");
    f.last_odom_time_=f.clock.stamp;f.have_odom_=false;
    expect(!f.failedAttemptRetryReady(failure+1.),
      "target_heading_wait_does_not_accept_unavailable_body");
    f.have_odom_=true;
    expect(f.failedAttemptRetryReady(failure+1.)&&f.invalidations==0&&
      f.planner_manager_->local_data_.traj_id_==1,
      "target_heading_recovers_same_geometry_only_on_new_current_body_without_motion_authority");
  }
  {
    FSM f;f.planning_only_=true;f.go2_execution_frozen_=false;
    f.execution_validator_=std::make_shared<FSM::Validator>();
    f.execution_validator_->value->progress.reset();
    expect(f.keepPreviewIncumbentForPeriodicReplan(),
      "explicit_planning_only_retains_measured_stationary_preview_without_tracker_or_frozen_topic");
    f.planning_curve_time_.reset();
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),
      "missing_exact_committed_geometry_or_fresh_native_proof_cannot_keep_preview");
    f.planning_curve_time_=.8;f.odom_vel_.x()=.1;
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),
      "actual_body_motion_wakes_planning_only_replanning");
    f.odom_vel_.setZero();f.planning_curve_time_=1.1;
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),
      "real_projected_preview_progress_advances_local_horizon");
    f.planning_curve_time_=0.;f.odom_vel_.x()=std::numeric_limits<double>::quiet_NaN();
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),
      "nonfinite_actual_velocity_never_claims_stopped_preview");
  }
  {
    FSM f;f.execution_validator_=std::make_shared<FSM::Validator>();
    expect(f.keepPreviewIncumbentForPeriodicReplan(),
      "formal_exact_fresh_stationary_applied_horizon_postpones_nominal_periodic_solve");
    expect(f.execution_validator_->value->proof->valid&&f.invalidations==0&&f.rechecks==0,
      "formal_idle_scheduling_does_not_revoke_incumbent_or_run_compatibility_preview_recheck");
    f.execution_validator_->value->progress->twist.linear.x=.1;
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),"formal_moving_body_keeps_normal_periodic_replanning");
    f.execution_validator_->value->progress->twist.linear.x=0.;
    f.execution_validator_->value->proof->valid=false;
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),"invalid_committed_proof_never_suppresses_replan");
  }
  {
    FSM f;f.execution_validator_=std::make_shared<FSM::Validator>();
    auto& active=*f.execution_validator_->value;
    active.progress->holding=true;
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),
      "finished_local_horizon_replans_even_with_fresh_safe_applied_curve_and_zero_actual_speed");
    expect(active.proof->valid && active.progress->valid && f.invalidations==0 &&
      active.spline.trajectory.traj_id==1,
      "local_horizon_reseed_neither_retires_writer_geometry_nor_invalidates_its_real_proof");
    active.progress->holding=false;f.planner_manager_->local_data_.traj_id_=3;
    f.execution_validator_->pending=true;
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),
      "unjoined_55mmps_side_candidate_cannot_freeze_old_safe_applied_curve_at_zero_speed");
    f.clock.stamp.ns+=1000000000;f.last_replan_time_.ns+=1000000000;
    active.proof->valid_until.ns+=1000000000;active.progress->header.stamp.ns+=1000000000;
    expect(!f.keepPreviewIncumbentForPeriodicReplan() && f.invalidations==0,
      "repeated_fresh_evidence_does_not_turn_rejected_pending_geometry_into_idle_preview");
    f.planner_manager_->local_data_.traj_id_=1;
    f.execution_validator_->pending=false;
    expect(f.keepPreviewIncumbentForPeriodicReplan(),
      "formal_matching_fresh_applied_curve_uses_idle_horizon_not_rejected_manager_candidate");
    ++f.reference_metadata_.generation;
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),"ordinary_new_reference_is_not_hidden_by_idle_holding");
    --f.reference_metadata_.generation;
    f.blocked_entry_replan_pending_=true;
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),"confirmed_obstacle_pending_has_priority_over_periodic_holding");
    f.blocked_entry_replan_pending_=false;f.force_visible_side_target_=true;
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),"unconsumed_side_corridor_event_has_priority_over_periodic_holding");
    f.force_visible_side_target_=false;
    std::promise<int> worker;f.solve_future_=worker.get_future();
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),"replacement_worker_is_not_suppressed_or_requeued_by_idle_holding");
    worker.set_value(1);f.solve_future_.get();
    f.pending_snapshot_request_.begin(f.reference_generation_);
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),"replacement_snapshot_acquisition_is_not_hidden_by_idle_holding");
    f.pending_snapshot_request_.cancel();
    f.clock.stamp.ns=active.proof->valid_until.ns;
    expect(!f.keepPreviewIncumbentForPeriodicReplan()&&f.invalidations==0,
      "idle_holding_never_renews_original_proof_deadline_or_cancels_applied_curve");
  }
  for (int mode=0;mode<4;++mode) {
    FSM f;
    f.result=mode==0?R::Stale:mode==1?R::Uncertified:mode==2?R::Unsafe:R::Unavailable;
    f.waitForChangedEnvironment();
    const std::string expected=mode==0?"waiting_sensor_map":mode==1?"waiting_recheck":
      mode==2?"failed_current_validation":"failed_optimization";
    expect(f.rechecks==1 && f.invalidations==1 && f.published_phase==expected,
      "failure_or_missing_proof_is_not_reported_as_safe_incumbent");
  }
  {
    FSM f;f.last_attempt_failure_phase_="waiting_body_pose";f.waitForChangedEnvironment();
    expect(!f.environmentChangedSinceFailure(),"waiting_body_pose_requires_new_source");
    f.last_odom_time_.ns+=1000000;
    expect(f.environmentChangedSinceFailure(),"fresh_new_body_sample_recovers_even_at_same_position");
    f.clock.stamp.ns+=1000000000;
    expect(!f.environmentChangedSinceFailure(),"stale_new_body_sample_does_not_recover");
  }
  {
    FSM f;f.last_attempt_failure_phase_="failed_dynamics";
    f.odom_vel_={.45,0,0};f.waitForChangedEnvironment();f.odom_vel_.setZero();
    expect(!f.environmentChangedSinceFailure(),"dynamics_recovery_requires_new_measurement");
    f.last_odom_time_.ns+=1000000;
    expect(f.environmentChangedSinceFailure(),"measured_speed_recovery_retries");
  }
  {
    FSM f;f.waitForChangedEnvironment();f.last_odom_time_.ns+=1000000;
    f.odom_orient_=Eigen::Quaterniond(Eigen::AngleAxisd(.01,Eigen::Vector3d::UnitZ()));
    expect(!f.environmentChangedSinceFailure(),"attitude_noise_does_not_retry");
    f.odom_orient_=Eigen::Quaterniond(-1,0,0,0);
    expect(!f.environmentChangedSinceFailure(),"quaternion_sign_flip_does_not_retry");
    f.odom_pos_.x()=.16;
    expect(f.environmentChangedSinceFailure(),"existing_translation_trigger_preserved");
  }
  {
    FSM f;f.waitForChangedEnvironment();++f.planner_manager_->grid_map_->revision;
    expect(f.environmentChangedSinceFailure(),"existing_occupancy_revision_trigger_preserved");
  }
  {
    FSM f;f.last_attempt_failure_phase_="waiting_sensor_map";f.waitForChangedEnvironment();
    f.planner_manager_->grid_map_->source+=.1;
    for(auto& stamp:f.planner_manager_->grid_map_->rays)stamp+=100000000;
    expect(f.environmentChangedSinceFailure(),"existing_new_map_source_trigger_preserved");
  }
  for(const auto* phase:{"waiting_recheck","waiting_observed_space","failed_final_collision","failed_splice_prefix_collision",
      "candidate_latest_geometry_rejected"}) {
    FSM f;f.last_attempt_failure_phase_=phase;f.waitForChangedEnvironment();
    auto& map=*f.planner_manager_->grid_map_;
    expect(!f.environmentChangedSinceFailure(),"same_geometric_map_and_same_source_never_busy_retries");
    map.source+=.1;map.rays[0]+=100000000;
    expect(!f.environmentChangedSinceFailure(),"front_only_cannot_wake_failed_observed_space");
    map.rays[1]+=100000000;
    expect(f.environmentChangedSinceFailure(),"new_dual_evidence_wakes_transient_recheck_without_geometric_change");
    f.clock.stamp.ns+=1000000000;
    expect(!f.environmentChangedSinceFailure(),"already_expired_dual_evidence_cannot_wake_recheck");
  }
  {
    FSM f;f.last_attempt_failure_phase_="waiting_snapshot_slot";f.waitForChangedEnvironment();
    const double failed=f.failed_replan_monotonic_;
    // Actual production FSM decision: identical map, body and sensor sources.
    expect(!f.failedAttemptRetryReady(failed+.499),"resource_retry_respects_half_second_bound");
    expect(f.failedAttemptRetryReady(failed+.501),"slot_wait_retries_without_geometric_or_source_change");
    expect(f.planner_manager_->grid_map_->revision==f.failed_environment_revision_ &&
      f.rechecks==1 && f.invalidations==0,"resource_retry_does_not_modify_or_invalidate_safe_incumbent");
    f.failed_replan_cooldown_=.2;
    expect(!f.failedAttemptRetryReady(failed+.199)&&f.failedAttemptRetryReady(failed+.201),
      "shorter_config_uses_bounded_five_hz_resource_retry_not_hidden_half_second_gate");
    f.failed_replan_cooldown_=1.;
    expect(!f.failedAttemptRetryReady(failed+.501) && f.failedAttemptRetryReady(failed+1.001),
      "larger_existing_cooldown_is_preserved");
    f.last_attempt_failure_phase_="waiting_observed_space";
    expect(!f.failedAttemptRetryReady(failed+1.001),"unknown_geometry_still_requires_new_real_evidence");
    expect(!f.failedAttemptRetryReady(std::numeric_limits<double>::quiet_NaN()),
      "invalid_monotonic_time_cannot_retry");
  }
  {
    FSM f;int skipped=0;
    for(int i=0;i<12;++i) {f.clock.stamp.ns+=1000000000;skipped+=f.keepPreviewIncumbentForPeriodicReplan();}
    expect(skipped==12 && f.rechecks==12 && f.invalidations==0,
      "stationary_preview_twelve_periodic_checks_keep_revalidated_incumbent_without_new_solve");
    f.odom_pos_.x()=.2;f.planner_manager_->progress=.8;
    expect(f.keepPreviewIncumbentForPeriodicReplan(),"small_real_progress_retains_safe_incumbent");
    f.planner_manager_->progress=1.1;
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),"measured_curve_fraction_advances_local_horizon");
    f.planner_manager_->progress=0.;f.odom_pos_.x()=1.1;
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),"large_measured_translation_requires_new_segment");
  }
  for(int mode=0;mode<4;++mode) {
    FSM f;
    f.result=mode==0?R::Unsafe:mode==1?R::Stale:mode==2?R::Uncertified:R::Unavailable;
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),"collision_stale_or_missing_proof_cannot_skip_replan");
  }
  {
    FSM f;f.go2_execution_frozen_=false;
    expect(!f.keepPreviewIncumbentForPeriodicReplan() && f.rechecks==0,
      "executing_mode_periodic_replan_unchanged");
  }
  {
    FSM f;f.odom_pos_.x()=.05;
    expect(!f.shouldReplanUncertifiedPreview(),"lost_join_does_not_create_immediate_retry_storm");
    f.clock.stamp.ns+=500000000;
    expect(f.shouldReplanUncertifiedPreview(),"measured_lost_join_replans_after_bounded_half_second");
    f.waitForChangedEnvironment();
    expect(!f.shouldReplanUncertifiedPreview(),"same_failed_body_does_not_blindly_repeat_join_solve");
    f.odom_pos_.x()+=.03;
    expect(f.shouldReplanUncertifiedPreview(),"new_small_real_motion_can_recover_old_failed_join");
    f.go2_execution_frozen_=false;
    expect(!f.shouldReplanUncertifiedPreview(),"lost_join_preview_policy_does_not_change_execution");
  }
  std::cout<<"checks="<<checks<<" failures="<<failures<<"\n";
  return failures?1:0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[1]
    parser.add_argument('--source', type=Path, default=root/'src/scan_replan_fsm.cpp')
    parser.add_argument('--include', type=Path, default=root/'include')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    source = args.source.read_text()
    def production_method(signature):
        # Select the method's own two-space outer brace, not a distant next
        # method name: unrelated owner ingress can be inserted between them.
        # The extracted body remains production code, with no stub/rewrite.
        start=source.index('  '+signature)
        body=source.index('\n  {',start)
        end=source.index('\n  }\n',body)+len('\n  }\n')
        return source[start:end]
    methods='\n'.join(production_method(signature) for signature in (
        'void SCANReplanFSM::waitForChangedEnvironment()',
        'bool SCANReplanFSM::failedAttemptRetryReady(double monotonic_now)',
        'bool SCANReplanFSM::environmentChangedSinceFailure()',
        'bool SCANReplanFSM::keepPendingSpliceForOrdinaryReplan()',
        'bool SCANReplanFSM::keepPreviewIncumbentForPeriodicReplan()',
        'bool SCANReplanFSM::shouldReplanUncertifiedPreview()',
        'void SCANReplanFSM::cancelSnapshotAcquisition()',
        'bool SCANReplanFSM::snapshotAcquisitionCurrent()',
    ))
    exec_start=source.index('    case EXEC_TRAJ:',source.index('  void SCANReplanFSM::execFSMCallback()'))
    prefix_start=source.index('{',exec_start)+1
    prefix_end=source.index('      /* determine if need to replan */',prefix_start)
    # Compile the real production nominal-trigger prefix. The counter marks
    # reaching the later progress branch; no scheduling decision is replaced.
    ordinary=('void SCANReplanFSM::ordinaryReplanTrigger(){\n'
      'using ExecutionValidator=Validator;switch(exec_state_){case EXEC_TRAJ:{\n'+
      source[prefix_start:prefix_end]+'++ordinary_progress_triggers;break;}default:break;}}\n')
    # The exact production GEN/REPLAN false branches, not a second failure FSM.
    dispatch='void SCANReplanFSM::planningDispatchFailure(State mode){switch(mode){\n'
    for state in ('GEN_NEW_TRAJ','REPLAN_TRAJ'):
        case=source.index('    case '+state+':',source.index('  void SCANReplanFSM::execFSMCallback()'))
        otherwise=source.index('      else\n      {',case)
        begin=source.index('{\n',otherwise)+2
        end=source.index('\n      }\n',begin)
        dispatch+='case '+state+':{\n'+source[begin:end]+'\nbreak;}\n'
    dispatch+='default:break;}}\n'
    with tempfile.TemporaryDirectory(prefix='scan-replan-recovery-') as directory:
        cpp, executable = Path(directory)/'recovery.cpp', Path(directory)/'recovery'
        cpp.write_text(HARNESS.replace('__METHODS__', methods).replace('__ORDINARY_INGRESS__',ordinary)
            .replace('__DISPATCH_FAILURE__',dispatch))
        subprocess.run(['c++', '-std=c++17', '-O0', '-Wall', '-Wextra',
            '-I'+str(args.include), '-I'+str(root.parent/'plan_env/include'),
            '-I'+str(root.parent/'bspline_opt/include'),
            '-I/usr/include/eigen3', str(cpp), '-o', str(executable)], check=True)
        result = subprocess.run([str(executable)], check=False, capture_output=True, text=True)
        print(result.stdout, end='')
        print(result.stderr, end='')
        if args.report:
            args.report.write_text(json.dumps(dict(source=str(args.source),
                source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                include_directory=str(args.include.resolve()),
                input_contract_sha256=hashlib.sha256(
                    (args.include/'plan_manage/input_contract.hpp').read_bytes()).hexdigest(),
                harness_sha256=hashlib.sha256(HARNESS.encode()).hexdigest(),
                production_method_sha256=hashlib.sha256(methods.encode()).hexdigest(),
                ordinary_trigger_prefix_sha256=hashlib.sha256(ordinary.encode()).hexdigest(),
                dispatch_failure_branches_sha256=hashlib.sha256(dispatch.encode()).hexdigest(),
                returncode=result.returncode, stdout=result.stdout, stderr=result.stderr,
                ros_initialized=False, robot_connected=False,
                scope='actual native wait/recovery/periodic decisions; full-curve proof supplied as explicit seam'),
                indent=2)+'\n')
        return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
