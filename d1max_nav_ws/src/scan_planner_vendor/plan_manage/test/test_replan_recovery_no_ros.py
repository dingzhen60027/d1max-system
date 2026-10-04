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
class SCANReplanFSM {
public:
  enum class PreviewRecheck {Unavailable,Stale,Safe,Unsafe,Uncertified};
  enum State {EXEC_TRAJ,WAIT_ENVIRONMENT,REPLAN_TRAJ};
  Node clock; Node *node_=&clock;
  struct Validator {
    struct View {
      struct {std::uint64_t generation=1;struct{std::int64_t traj_id=1;}trajectory;} spline;
      struct Proof {bool valid=true;ClockValue valid_until{500200000000LL};};
      struct Progress {bool valid=true,holding=false;struct{ClockValue stamp{499900000000LL};}header;
        struct{struct{double x=0.,y=0.,z=0.;}linear;}twist;};
      std::optional<Proof> proof{Proof{}};std::optional<Progress>progress{Progress{}};
    };
    std::optional<View> value{View{}};
    std::optional<View> committedView()const{return value;}
  };
  std::shared_ptr<Validator> execution_validator_;
  std::uint64_t reference_generation_=1;
  std::future<int> solve_future_;
  PendingSnapshotRequest pending_snapshot_request_;
  struct Pool {int cancellations=0;void cancelSolverRequest(){++cancellations;}}snapshot_pool_;
  std::shared_ptr<Manager> planner_manager_=std::make_shared<Manager>();
  bool have_odom_=true;
  bool go2_execution_frozen_=true,reference_path_guidance_=true,have_target_=true;
  double planning_horizon_=4.,failed_replan_cooldown_=.5;
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
  State exec_state_=EXEC_TRAJ; int rechecks=0, invalidations=0;
  PreviewRecheck revalidatePreviewIncumbent() {++rechecks;return result;}
  void publishInvalidLocalPlanDebug(const std::string &phase,bool) {
    ++invalidations; published_phase=phase;
  }
  void changeFSMExecState(State next,const std::string &) {exec_state_=next;}
  void waitForChangedEnvironment();
  void cancelSnapshotAcquisition();
  bool snapshotAcquisitionCurrent();
  bool failedAttemptRetryReady(double monotonic_now);
  bool environmentChangedSinceFailure();
  bool keepPreviewIncumbentForPeriodicReplan();
  bool shouldReplanUncertifiedPreview();
};
__METHODS__
}
int main() {
  using FSM=scan_planner::SCANReplanFSM; using R=FSM::PreviewRecheck;
  int checks=0,failures=0;
  const auto expect=[&](bool ok,const char *name) {
    ++checks;failures+=!ok;std::cout<<(ok?"PASS ":"FAIL ")<<name<<"\n";
  };
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
    expect(!formalReseedSubmissionDue(last+std::chrono::milliseconds(499),last,false),
      "event_reseed_cannot_exceed_global_two_hz_solve_budget");
    expect(formalReseedSubmissionDue(last+std::chrono::milliseconds(500),last,false),
      "entry_failure_need_not_wait_the_one_second_periodic_preview_clock");
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
  {
    FSM f;f.execution_validator_=std::make_shared<FSM::Validator>();
    expect(f.keepPreviewIncumbentForPeriodicReplan(),"same_applied_stationary_nonholding_curve_avoids_candidate_solve_storm");
    f.execution_validator_->value->progress->twist.linear.x=.1;
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),"measured_motion_wakes_execution_replanning");
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
    expect(!f.keepPreviewIncumbentForPeriodicReplan(),
      "unjoined_55mmps_side_candidate_cannot_freeze_old_safe_applied_curve_at_zero_speed");
    f.clock.stamp.ns+=1000000000;f.last_replan_time_.ns+=1000000000;
    active.proof->valid_until.ns+=1000000000;active.progress->header.stamp.ns+=1000000000;
    expect(!f.keepPreviewIncumbentForPeriodicReplan() && f.invalidations==0,
      "repeated_fresh_evidence_does_not_turn_rejected_pending_geometry_into_idle_preview");
    f.planner_manager_->local_data_.traj_id_=1;
    expect(f.keepPreviewIncumbentForPeriodicReplan(),
      "only_current_applied_nonholding_curve_can_use_stationary_keep_policy");
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
  for(const auto* phase:{"waiting_recheck","waiting_observed_space","failed_final_collision"}) {
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
    expect(!f.failedAttemptRetryReady(failed+.499),"shorter_config_does_not_spin_resource_retries_above_two_hz");
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
    start = source.index('  void SCANReplanFSM::waitForChangedEnvironment()')
    end = source.index('  bool SCANReplanFSM::planFromCurrentTraj()', start)
    acquisition_start=source.index('  void SCANReplanFSM::cancelSnapshotAcquisition()')
    acquisition_end=source.index('  void SCANReplanFSM::cancelSolveWorker()',acquisition_start)
    methods=source[start:end]+source[acquisition_start:acquisition_end]
    with tempfile.TemporaryDirectory(prefix='scan-replan-recovery-') as directory:
        cpp, executable = Path(directory)/'recovery.cpp', Path(directory)/'recovery'
        cpp.write_text(HARNESS.replace('__METHODS__', methods))
        subprocess.run(['c++', '-std=c++17', '-O0', '-Wall', '-Wextra',
            '-I'+str(args.include), '-I'+str(root.parent/'plan_env/include'),
            '-I/usr/include/eigen3', str(cpp), '-o', str(executable)], check=True)
        result = subprocess.run([str(executable)], check=False, capture_output=True, text=True)
        print(result.stdout, end='')
        print(result.stderr, end='')
        if args.report:
            args.report.write_text(json.dumps(dict(source=str(args.source),
                source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                production_method_sha256=hashlib.sha256(methods.encode()).hexdigest(),
                returncode=result.returncode, stdout=result.stdout, stderr=result.stderr,
                ros_initialized=False, robot_connected=False,
                scope='actual native wait/recovery/periodic decisions; full-curve proof supplied as explicit seam'),
                indent=2)+'\n')
        return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
