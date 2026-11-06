#!/usr/bin/env python3
"""Compile production worker completion/cancel methods; no ROS or fake planner.

The candidate/adoption decision is a deliberately passive seam. These tests
prove generation, deadline, cancellation and publication ordering only; native
snapshot/AStar/collision tests separately validate geometry implementation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

HARNESS = r'''
#include <plan_env/solve_budget.hpp>
#include <plan_manage/candidate_adoption.hpp>
#include <plan_manage/owner_solve_completion.hpp>
#include <future>
#include <optional>
#include <iostream>
#include <string>
#include <stdexcept>
#include <new>
#include <thread>
#define RCLCPP_INFO(...) ((void)0)
struct Time {double value=100.;double seconds() const{return value;}
  std::int64_t nanoseconds()const{return static_cast<std::int64_t>(value*1e9);}
  Time operator-(Time rhs) const{return {value-rhs.value};}};
namespace builtin_interfaces {namespace msg {using Time=::Time;}}
struct Node {Time now() const{return {};}};
namespace scan_planner {
struct CollisionSnapshotPool {enum class SolverPublication {Deferred,Unchanged,Published};};
// Passive instrumentation of the binding boundary only. Native scope/map
// tests separately execute the actual OBB collision implementation.
struct Support {int version=1;};
struct StairAttitudeScope {
  static inline int constructions=0,destructions=0,depth=0;
  static inline int* bound_map=nullptr;
  static inline const Support* bound_support=nullptr;
  static inline MeasuredBodyPose bound_body;
  int* previous_map;const Support* previous_support;MeasuredBodyPose previous_body;
  StairAttitudeScope(int& map,const std::shared_ptr<Support>& support,const MeasuredBodyPose& body):
      previous_map(bound_map),previous_support(bound_support),previous_body(bound_body) {
    ++constructions;++depth;bound_map=&map;bound_support=support.get();bound_body=body;
  }
  ~StairAttitudeScope(){++destructions;--depth;bound_map=previous_map;
    bound_support=previous_support;bound_body=previous_body;}
};
struct Splice {struct {int incumbent_version=1,incumbent_trajectory_id=17;} wire;
  int incumbent=17;double measured_time=0.,committed_arc=0.;std::int64_t progress_source_ns=0;};
double curveArcTo(int,double time){return time*.1;}
struct Validator {struct Horizon {int version=1,trajectory_id=17;double measured_time=1.;
  double committed_arc=.1;
  std::int64_t measured_source_ns=100000000000LL;};std::optional<Horizon> current{Horizon{}};
  int snapshot_wakes=0,admission_wakes=0,support_lookups=0,last_support_version=-1;
  std::shared_ptr<Support> support=std::make_shared<Support>();
  std::optional<bool> slow_hint{true};int slow_hint_reads=0;
  auto planningSupport(int version){++support_lookups;last_support_version=version;return support;}
  bool slowSnapshotWanted()const{return true;}
  std::optional<bool> trySlowSnapshotWanted(){++slow_hint_reads;return slow_hint;}
  void snapshotPublished(){++snapshot_wakes;}
  void admissionSnapshotPublished(){++admission_wakes;}
  auto committedHorizonAtBody(int,std::int64_t,const MeasuredBodyPose&,std::int64_t)const{return current;}};
struct Manager {
  std::shared_ptr<int> grid_map_=std::make_shared<int>(1);
  struct SolvedCandidate {int id=18;std::optional<Splice> splice;};
  std::optional<Splice> planned;void setPlannedSplice(std::optional<Splice> s){planned=std::move(s);}
  bool safe=true;int imports=0,adoptions=0,curve=17;
  bool incumbent_proof=true,throw_on_adoption=false;
  int scoped_adoptions=0,scoped_predecessors=0;const Support* adopted_support=nullptr;
  MeasuredBodyPose adopted_body;
  CandidateAcceptanceOutcome outcome=CandidateAcceptanceOutcome::MotionAccepted;
  CandidateAcceptanceDiagnostics diagnostics;
  const CandidateAcceptanceDiagnostics& candidateAcceptanceDiagnostics()const{return diagnostics;}
  std::string lastFailurePhase()const{return candidateAcceptanceReason(outcome);}
  void importAttemptDiagnostics(const Manager&) {++imports;}
  bool adoptSolvedCandidate(SolvedCandidate candidate,const SolveBudget::Ptr &budget) {
    if(StairAttitudeScope::depth>0&&StairAttitudeScope::bound_map==grid_map_.get())++scoped_adoptions;
    adopted_support=StairAttitudeScope::bound_support;adopted_body=StairAttitudeScope::bound_body;
    if(throw_on_adoption)throw std::runtime_error("adoption_failure");
    diagnostics.record(outcome);
    if(!safe || !budget->allowed())return false;
    ++adoptions;curve=candidate.id;return true;
  }
  bool recheckPredecessor(int,double){
    if(StairAttitudeScope::depth>0&&StairAttitudeScope::bound_map==grid_map_.get())++scoped_predecessors;
    return safe;}
};
class SCANReplanFSM {
public:
  struct WorkerResult {std::optional<Manager::SolvedCandidate> candidate;std::string failure;
    std::chrono::steady_clock::time_point finished_at{};std::int64_t body_source_ns=0;};
  std::future<WorkerResult> solve_future_;
  std::optional<CompletedOwnerSolve<WorkerResult>> completed_solve_;
  std::optional<RetiredOwnerSolveFailure> retired_solve_failure_;
  SolveBudget::Ptr active_solve_budget_;
  std::uint64_t solve_generation_=7,reference_generation_=7;
  bool have_target_=true,have_odom_=true,flag_escape_emergency_=false;
  bool planning_only_=false,local_follower_=false;int reference_metadata_=1;
  int executionVersion(int v)const{return v;}
  std::unique_ptr<Validator> execution_validator_=std::make_unique<Validator>();
  Node clock;Node *node_=&clock;
  Eigen::Vector3d odom_pos_=Eigen::Vector3d::Zero();
  Eigen::Quaterniond odom_orient_=Eigen::Quaterniond::Identity();
  std::string self_inflation_frame_id_="odom";
  std::unique_ptr<Manager> planner_manager_=std::make_unique<Manager>();
  std::unique_ptr<Manager> solve_worker_=std::make_unique<Manager>();
  std::shared_ptr<int> grid_map_writer_=planner_manager_->grid_map_;
  bool map_writer_group_=false;
  bool map_validation_publication_pending_=false,map_solver_publication_pending_=false;
  struct Pool {int validation_published=0,solver_published=0;
    bool replica_available=false;int replica_published=0;
    bool validation_available=true,solver_available=true,have_solver_value=false;int validation_value=0,solver_value=0;
    bool publishValidation(int source,std::int64_t,std::chrono::steady_clock::time_point){
      ++validation_published;if(validation_available)validation_value=source;return validation_available;}
    bool refreshOwnerReplica(int,std::int64_t,std::chrono::steady_clock::time_point){
      if(!replica_available)return false;
      replica_available=false;++replica_published;return true;}
    bool publishSolver(int source,std::int64_t,std::chrono::steady_clock::time_point){
      ++solver_published;if(solver_available){solver_value=source;have_solver_value=true;}return solver_available;}
    CollisionSnapshotPool::SolverPublication refreshSolverPublication(int source,std::int64_t,std::chrono::steady_clock::time_point){
      if(have_solver_value&&solver_value==source)return CollisionSnapshotPool::SolverPublication::Unchanged;
      ++solver_published;if(!solver_available)return CollisionSnapshotPool::SolverPublication::Deferred;
      solver_value=source;have_solver_value=true;return CollisionSnapshotPool::SolverPublication::Published;}
  } snapshot_pool_;
  std::string last_attempt_failure_phase_;
  Time last_odom_time_{99.9};double odom_timeout_=.5;
  int solve_attempt_header_=1,replan_fail_count_=0,attempts=0,waits=0,publishes=0,transitions=0;
  std::uint64_t predecessor_id_=0;
  bool predecessor_safe_=false;
  Time predecessor_check_stamp_;
  std::optional<int> solve_predecessor_;
  int solve_predecessor_id_=0;double solve_predecessor_measured_time_=0.;
  enum {EXEC_TRAJ,REPLAN_TRAJ};int state=EXEC_TRAJ;
  void publishAttemptDebug(int){++attempts;}
  void waitForChangedEnvironment(){++waits;}
  void publishCommittedTrajectory(){++publishes;}
  void changeFSMExecState(int next,const std::string&){++transitions;state=next;}
  void cancelSnapshotAcquisition(){} // Separate production acquisition seam is tested in recovery harness.
  void cancelSolveWorker();bool pollSolveWorker();void publishMapSnapshots();
  bool solveWorkPending()const;void collectSolveCompletion();
  bool owner_map_available=true;unsigned unavailable_debug=0,map_entries=0;
  struct OwnerMapScope {bool available;explicit OwnerMapScope(SCANReplanFSM& f):available(f.owner_map_available){
    if(available)++f.map_entries;}explicit operator bool()const{return available;}};
  void drainOwnerInputs(){} // Real per-input ordering has its separate extracted-method test.
  void publishExecutionLedgerDebug(){++unavailable_debug;}
  void execFSMOwnerCallback(){pollSolveWorker();}
  void execFSMCallback();
  int body_refreshes=0;bool fresh_body_available=false;
  MeasuredBodyPose newest_body{{1.,2.,.6},Eigen::Quaterniond(Eigen::AngleAxisd(-.3,Eigen::Vector3d::UnitY())),99.99,"odom"};
  bool refreshPlanningBodyFromInbox(){++body_refreshes;if(!fresh_body_available)return false;
    odom_pos_=newest_body.position;odom_orient_=newest_body.orientation;
    last_odom_time_.value=newest_body.source_stamp;return true;} // Real source pairing has native-ledger tests.
  void completedMapWriterTransaction(){__FUSION_CALLBACK__}
};
__METHODS__

// Passive submission seam, not a planner/map implementation. Compile the
// actual post-acquisition refresh, side-slot borrowing and worker-input
// capture statements. The selector records which acquired state was used;
// native map/optimization feasibility is intentionally not claimed here.
struct SnapshotRefreshProbe {
  struct GridMap {using Ptr=std::shared_ptr<int>;};
  struct Sample {Eigen::Vector3d p,v,a;std::int64_t source;};
  struct Owner {GridMap::Ptr grid_map_=std::make_shared<int>(7);Sample measured;
    std::int64_t reference_source=0;Eigen::Vector3d reference_start;};
  struct Worker {Sample measured;std::int64_t reference_source=0;Eigen::Vector3d reference_start;
    GridMap::Ptr snapshot;
    void prepareSolveWorker(const Owner& owner,const GridMap::Ptr& map) {
      measured=owner.measured;reference_source=owner.reference_source;
      reference_start=owner.reference_start;snapshot=map;
    }
  };
  struct Pool {int acquisitions=0;GridMap::Ptr map=std::make_shared<int>(7);bool available=false;
    GridMap::Ptr acquireSolver(int,std::int64_t,SolveBudget::Clock::time_point) {
      ++acquisitions;return available?map:nullptr;
    }
  } snapshot_pool_;
  Node clock;Node* node_=&clock;
  std::unique_ptr<Owner> planner_manager_=std::make_unique<Owner>();
  std::unique_ptr<Worker> solve_worker_=std::make_unique<Worker>();
  Sample newest{{1.,2.,.55},{.3,.01,0.},{.02,0.,0.},99980000000LL};
  Time last_odom_time_{99.7};bool local_follower_=true,target_valid=true,advance_during_target=false;
  int target_rebuilds=0,cancellations=0,submissions=0;
  Eigen::Vector3d start_pt_{0.,2.,.55},start_vel_{.1,0.,0.},start_acc_=Eigen::Vector3d::Zero();
  Eigen::Vector3d local_target_pt_{4.,2.,.55},local_target_vel_=Eigen::Vector3d::Zero();
  Eigen::Vector3d captured_p,captured_v,captured_a,captured_target;
  std::int64_t captured_source=0;
  const SolveBudget* original_budget=nullptr;
  std::string last_attempt_failure_phase_;
  struct Selection {std::string reason="passive_target_rejected";};Selection local_target_query_debug_;
  void cancelSnapshotAcquisition(){++cancellations;}
  void setStartStateFromOdomOrCurrentTraj() {
    if(newest.source>last_odom_time_.nanoseconds()) {
      last_odom_time_.value=double(newest.source)*1e-9;planner_manager_->measured=newest;
    }
    start_pt_=planner_manager_->measured.p;start_vel_=planner_manager_->measured.v;
    start_acc_=planner_manager_->measured.a;
  }
  bool getLocalTarget(const SolveBudget::Ptr& budget,GridMap::Ptr* reserved_snapshot) {
    ++target_rebuilds;if(budget.get()!=original_budget)throw std::runtime_error("replaced_parent_budget");
    Selection selection;
    __SIDE_SNAPSHOT_ACQUIRE__
    if(!target_valid||!budget->allowed())return false;
    if(advance_during_target){newest.source+=20000000LL;newest.p.x()+=.006;}
    planner_manager_->reference_source=last_odom_time_.nanoseconds();
    planner_manager_->reference_start=start_pt_;
    local_target_pt_=start_pt_+Eigen::Vector3d(4.,0.,0.);
    if(reserved_snapshot)*reserved_snapshot=snapshot;
    return true;
  }
  bool submitAfterSnapshot(const SolveBudget::Ptr& request_budget,GridMap::Ptr snapshot) {
    original_budget=request_budget.get();
    __LATEST_BODY_REFRESH__
    __PREPARE_WORKER__
    __CAPTURE_INPUTS__
    captured_p=start;captured_v=velocity;captured_a=acceleration;captured_target=target;
    captured_source=solve_body_source_ns;(void)target_velocity;++submissions;return true;
  }
};
}
int main(){
  using F=scan_planner::SCANReplanFSM;using B=scan_planner::SolveBudget;
  int checks=0,failures=0;
  auto expect=[&](bool ok,const char *name){++checks;failures+=!ok;
    std::cout<<(ok?"PASS ":"FAIL ")<<name<<"\n";};
  auto setup=[](F&f,std::promise<F::WorkerResult>&p){
    f.active_solve_budget_=std::make_shared<B>();f.solve_future_=p.get_future();};
  {
    using Scope=scan_planner::StairAttitudeScope;
    F f;f.local_follower_=true;f.fresh_body_available=true;f.solve_predecessor_=17;
    f.solve_predecessor_id_=17;std::promise<F::WorkerResult> p;setup(f,p);
    const auto scopes=Scope::constructions,released=Scope::destructions;
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});f.pollSolveWorker();
    expect(f.body_refreshes==1&&f.execution_validator_->support_lookups==1&&
      f.execution_validator_->last_support_version==f.reference_metadata_,
      "adoption_refreshes_acquired_body_once_and_looks_up_current_exact_reference");
    expect(f.planner_manager_->scoped_adoptions==1&&f.planner_manager_->scoped_predecessors==1&&
      f.planner_manager_->adopted_support==f.execution_validator_->support.get()&&
      f.planner_manager_->adopted_body.position.isApprox(f.newest_body.position)&&
      f.planner_manager_->adopted_body.orientation.isApprox(f.newest_body.orientation)&&
      f.planner_manager_->adopted_body.source_stamp==f.newest_body.source_stamp,
      "owner_adoption_and_predecessor_recheck_bind_source_support_with_original_newest_body");
    expect(Scope::constructions==scopes+1&&Scope::destructions==released+1&&
      Scope::depth==0&&!Scope::bound_map&&!Scope::bound_support,
      "adoption_scope_releases_provider_after_success_without_inheriting_worker_binding");
  }
  for(const bool exception:{false,true}) {
    using Scope=scan_planner::StairAttitudeScope;
    F f;f.planner_manager_->safe=false;f.planner_manager_->throw_on_adoption=exception;
    std::promise<F::WorkerResult> p;setup(f,p);p.set_value({scan_planner::Manager::SolvedCandidate{},""});
    const auto scopes=Scope::constructions,released=Scope::destructions;
    bool thrown=false;try{f.pollSolveWorker();}catch(const std::runtime_error&){thrown=true;}
    expect(thrown==exception&&Scope::constructions==scopes+1&&Scope::destructions==released+1&&
      Scope::depth==0&&!Scope::bound_map&&!Scope::bound_support&&f.planner_manager_->curve==17,
      "rejected_or_exceptional_adoption_restores_provider_and_keeps_incumbent");
  }
  {
    F f;f.map_writer_group_=true;f.execution_validator_->slow_hint.reset();
    *f.grid_map_writer_=234567890;
    f.completedMapWriterTransaction();
    expect(!f.map_validation_publication_pending_&&f.map_solver_publication_pending_&&
      f.snapshot_pool_.validation_value==234567890&&f.execution_validator_->snapshot_wakes==1&&
      f.snapshot_pool_.solver_published==0&&f.execution_validator_->slow_hint_reads==1,
      "busy_ledger_hint_does_not_delay_fast_head_or_discard_admission_retry");
    f.publishMapSnapshots();
    expect(f.map_solver_publication_pending_&&f.snapshot_pool_.validation_published==1&&
      f.snapshot_pool_.solver_published==0,
      "busy_hint_retries_neither_republish_fast_head_nor_allocate_a_slow_copy");
    f.execution_validator_->slow_hint=true;f.publishMapSnapshots();
    expect(!f.map_solver_publication_pending_&&f.snapshot_pool_.solver_published==1&&
      f.snapshot_pool_.solver_value==234567890&&f.execution_validator_->admission_wakes==1,
      "released_ledger_delivers_original_pending_map_without_another_fusion");
    f.publishMapSnapshots();
    expect(f.snapshot_pool_.solver_published==1,"completed_advisory_retry_is_not_repeated");
  }
  {
    F f;f.map_writer_group_=true;f.execution_validator_->slow_hint.reset();
    f.completedMapWriterTransaction();
    f.execution_validator_->slow_hint=false;f.publishMapSnapshots();
    expect(!f.map_solver_publication_pending_&&f.snapshot_pool_.solver_published==0,
      "retired_work_clears_advisory_retry_without_copy_or_admission_wake");
    *f.grid_map_writer_=0;f.execution_validator_->slow_hint.reset();
    f.completedMapWriterTransaction();
    expect(f.snapshot_pool_.validation_value==0&&f.execution_validator_->snapshot_wakes==2&&
      f.map_solver_publication_pending_,"reset_head_is_published_while_advisory_ledger_is_busy");
  }
  {
    F f;f.map_writer_group_=true;f.snapshot_pool_.validation_available=false;
    f.snapshot_pool_.solver_available=false;*f.grid_map_writer_=123456789;
    f.completedMapWriterTransaction();
    expect(f.map_validation_publication_pending_&&f.map_solver_publication_pending_&&
      f.execution_validator_->snapshot_wakes==0,"all_slots_borrowed_retains_one_pending_transaction");
    f.snapshot_pool_.validation_available=true;f.publishMapSnapshots();
    expect(!f.map_validation_publication_pending_&&f.map_solver_publication_pending_&&
      f.snapshot_pool_.validation_value==123456789&&f.execution_validator_->snapshot_wakes==1,
      "released_validation_slot_gets_same_integrated_source_without_next_fusion");
    f.snapshot_pool_.solver_available=true;f.publishMapSnapshots();
    expect(!f.map_solver_publication_pending_&&f.snapshot_pool_.solver_value==123456789&&
      f.execution_validator_->admission_wakes==1,"solver_retry_preserves_original_integrated_source");
    const auto attempts=f.snapshot_pool_.validation_published+f.snapshot_pool_.solver_published;
    f.publishMapSnapshots();
    expect(f.snapshot_pool_.validation_published+f.snapshot_pool_.solver_published==attempts,
      "idle_retry_does_not_duplicate_publication_or_extend_evidence");
  }
  {
    F f;f.map_validation_publication_pending_=true;f.publishMapSnapshots();
    expect(!f.map_validation_publication_pending_&&f.execution_validator_->snapshot_wakes==1,
      "published_head_is_available_before_busy_replica_releases");
    f.snapshot_pool_.replica_available=true;f.publishMapSnapshots();
    expect(f.snapshot_pool_.validation_published==1&&f.snapshot_pool_.replica_published==1&&
      f.execution_validator_->snapshot_wakes==1,"released_replica_does_not_wake_current_reader_for_same_geometry");
    f.publishMapSnapshots();
    expect(f.snapshot_pool_.replica_published==1&&f.execution_validator_->snapshot_wakes==1,
      "unchanged_replica_neither_recopies_nor_repeatedly_wakes_reader");
  }
  {
    F f;std::promise<F::WorkerResult> p;setup(f,p);
    expect(f.pollSolveWorker() && f.waits==0 && f.publishes==0,"pending_is_not_failure_or_adoption");
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});
    expect(f.pollSolveWorker() && f.publishes==1 && f.planner_manager_->curve==18,
           "ready_current_valid_candidate_commits_once");
    expect(!f.pollSolveWorker() && f.publishes==1,"completion_is_consumed_once");
  }
  {
    F f;f.local_follower_=true;f.map_writer_group_=true;std::promise<F::WorkerResult> p;setup(f,p);
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});
    f.completedMapWriterTransaction();
    expect(f.publishes==0&&f.solve_future_.valid()&&f.snapshot_pool_.validation_published==1&&
      f.snapshot_pool_.solver_published==1,"formal_writer_only_publishes_snapshots_never_consumes_owner_future");
    f.pollSolveWorker();
    expect(f.publishes==1&&!f.solve_future_.valid(),"formal_owner_harvests_ready_worker_independently");
  }
  {
    F f;f.local_follower_=true;std::promise<F::WorkerResult> p;setup(f,p);
    const auto start=std::chrono::steady_clock::now();
    f.completedMapWriterTransaction();
    expect(std::chrono::steady_clock::now()-start<std::chrono::milliseconds(100)&&
      f.publishes==0&&f.solve_future_.valid()&&f.waits==0,
      "completed_map_transaction_does_not_wait_for_unready_worker");
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});
    f.completedMapWriterTransaction();f.completedMapWriterTransaction();
    expect(f.publishes==1&&f.planner_manager_->adoptions==1&&!f.solve_future_.valid(),
      "completed_map_transaction_harvests_ready_worker_once_without_next_fsm_tick");
    expect(f.snapshot_pool_.validation_published==3&&f.snapshot_pool_.solver_published==1&&
      f.execution_validator_->snapshot_wakes==3&&f.execution_validator_->admission_wakes==1,
      "map_harvest_preserves_head_notifications_without_recopying_identical_admission_geometry");
  }
  for(int mode=0;mode<3;++mode) {
    F f;f.local_follower_=true;std::promise<F::WorkerResult> p;setup(f,p);
    if(mode==0)f.cancelSolveWorker();
    if(mode==1)++f.reference_generation_;
    if(mode==2)f.have_target_=false;
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});f.completedMapWriterTransaction();
    expect(f.publishes==0&&f.planner_manager_->imports==0&&f.planner_manager_->adoptions==0&&
      !f.solve_future_.valid(),"map_completion_cannot_commit_cancelled_replaced_or_retired_task");
  }
  {
    F f;f.local_follower_=true;std::promise<F::WorkerResult> p;setup(f,p);auto now=B::Clock::now();
    f.active_solve_budget_=std::make_shared<B>(std::chrono::milliseconds(400),[&]{return now;});
    now+=std::chrono::milliseconds(400);p.set_value({scan_planner::Manager::SolvedCandidate{},""});
    f.completedMapWriterTransaction();expect(f.publishes==0&&f.planner_manager_->curve==17&&f.waits==1,
      "map_completion_does_not_renew_expired_solve_budget_or_clear_incumbent");
  }
  {
    scan_planner::SnapshotRefreshProbe f;auto now=B::Clock::now();
    auto budget=std::make_shared<B>(std::chrono::milliseconds(400),[&]{return now;});
    const auto deadline=budget->deadline();auto snapshot=f.snapshot_pool_.map;
    now+=std::chrono::milliseconds(70); // Existing target/copy work; no fake state propagation.
    expect(f.submitAfterSnapshot(budget,snapshot)&&f.target_rebuilds==1&&f.submissions==1,
      "post_snapshot_new_body_rebuilds_target_once_then_submits_once");
    expect(f.captured_p.isApprox(f.newest.p)&&f.captured_v.isApprox(f.newest.v)&&
      f.captured_a.isApprox(f.newest.a)&&f.captured_source==f.newest.source&&
      f.solve_worker_->measured.source==f.newest.source&&
      f.solve_worker_->reference_source==f.newest.source&&f.solve_worker_->reference_start.isApprox(f.newest.p)&&
      f.captured_target.isApprox(f.newest.p+Eigen::Vector3d(4.,0.,0.)),
      "worker_boundary_manager_sample_and_rebuilt_reference_share_original_acquired_state");
    expect(f.snapshot_pool_.acquisitions==0&&f.solve_worker_->snapshot==snapshot&&budget->deadline()==deadline,
      "side_reselection_reuses_exclusive_snapshot_and_original_parent_deadline");
  }
  {
    scan_planner::SnapshotRefreshProbe f;auto budget=std::make_shared<B>();
    f.last_odom_time_.value=double(f.newest.source)*1e-9;f.planner_manager_->measured=f.newest;
    f.planner_manager_->reference_source=f.newest.source;f.planner_manager_->reference_start=f.newest.p;
    expect(f.submitAfterSnapshot(budget,f.snapshot_pool_.map)&&f.target_rebuilds==0&&
      f.submissions==1&&f.snapshot_pool_.acquisitions==0,
      "same_acquired_sample_does_not_repeat_target_work_or_compete_for_snapshot");
  }
  {
    scan_planner::SnapshotRefreshProbe f;auto budget=std::make_shared<B>();
    f.advance_during_target=true;const auto selected=f.newest;
    expect(f.submitAfterSnapshot(budget,f.snapshot_pool_.map)&&f.target_rebuilds==1&&
      f.submissions==1&&f.captured_source==selected.source&&f.newest.source>f.captured_source&&
      f.captured_p.isApprox(selected.p)&&f.solve_worker_->reference_source==selected.source,
      "continuous_body_arrivals_do_not_loop_or_mix_newest_source_into_already_selected_geometry");
  }
  {
    scan_planner::SnapshotRefreshProbe f;auto budget=std::make_shared<B>();
    f.original_budget=budget.get();f.snapshot_pool_.available=true;
    scan_planner::SnapshotRefreshProbe::GridMap::Ptr empty;
    expect(f.getLocalTarget(budget,&empty)&&empty==f.snapshot_pool_.map&&f.snapshot_pool_.acquisitions==1,
      "ordinary_side_target_without_reserved_snapshot_still_acquires_one_existing_slot");
  }
  {
    scan_planner::SnapshotRefreshProbe f;auto budget=std::make_shared<B>();
    f.original_budget=budget.get();auto reserved=f.snapshot_pool_.map;
    expect(f.getLocalTarget(budget,&reserved)&&reserved==f.snapshot_pool_.map&&
      f.snapshot_pool_.acquisitions==0,
      "side_target_with_owned_exclusive_slot_does_not_attempt_a_second_unavailable_lease");
  }
  for(int mode=0;mode<3;++mode) {
    scan_planner::SnapshotRefreshProbe f;auto now=B::Clock::now();
    auto budget=std::make_shared<B>(std::chrono::milliseconds(400),[&]{return now;});
    if(mode==0)f.target_valid=false;
    if(mode==1)now+=std::chrono::milliseconds(400);
    if(mode==2)budget->cancel();
    expect(!f.submitAfterSnapshot(budget,f.snapshot_pool_.map)&&f.submissions==0&&
      f.snapshot_pool_.acquisitions==0&&f.cancellations==(mode==0?0:1),
      "failed_or_expired_reselection_cannot_launch_worker_or_renew_snapshot_deadline");
  }
  {
    F f;f.local_follower_=true;std::promise<F::WorkerResult> p;setup(f,p);
    scan_planner::Manager::SolvedCandidate candidate;candidate.splice=scan_planner::Splice{};
    p.set_value({candidate,""});f.pollSolveWorker();
    expect(f.planner_manager_->planned&&f.planner_manager_->planned->measured_time==1.&&
      f.planner_manager_->planned->progress_source_ns==100000000000LL&&f.publishes==1,
      "future_tail_adoption_refreshes_exact_active_measured_progress_not_worker_clock");
  }
  {
    F f;f.local_follower_=true;std::promise<F::WorkerResult> p;setup(f,p);
    f.execution_validator_->current->trajectory_id=19;
    scan_planner::Manager::SolvedCandidate candidate;candidate.splice=scan_planner::Splice{};
    p.set_value({candidate,""});f.pollSolveWorker();
    expect(!f.planner_manager_->planned,
      "replaced_incumbent_cannot_supply_a_foreign_future_tail_adoption_seed");
  }
  for(int mode=0;mode<3;++mode){
    F f;std::promise<F::WorkerResult> p;setup(f,p);
    const auto scopes=scan_planner::StairAttitudeScope::constructions;
    if(mode==0)f.cancelSolveWorker();
    if(mode==1)++f.reference_generation_;
    if(mode==2)f.have_target_=false;
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});
    expect(!f.pollSolveWorker() && f.publishes==0 && f.planner_manager_->imports==0 && f.waits==0,
           "cancelled_old_generation_or_retired_task_has_no_result_side_effect");
    expect(scan_planner::StairAttitudeScope::constructions==scopes&&f.body_refreshes==0&&
      f.execution_validator_->support_lookups==0,
      "retired_worker_cannot_refresh_body_or_create_attitude_binding");
  }
  {
    F f;std::promise<F::WorkerResult> p;setup(f,p);auto now=B::Clock::now();
    f.active_solve_budget_=std::make_shared<B>(std::chrono::milliseconds(400),[&]{return now;});
    now+=std::chrono::milliseconds(400);p.set_value({scan_planner::Manager::SolvedCandidate{},""});
    f.pollSolveWorker();expect(f.waits==1 && f.publishes==0 && f.planner_manager_->curve==17 &&
      f.last_attempt_failure_phase_=="solve_deadline_exceeded","late_success_keeps_incumbent_and_waits");
  }
  for(int mode=0;mode<3;++mode){
    F f;std::promise<F::WorkerResult> p;setup(f,p);
    if(mode==0)f.planner_manager_->safe=false;
    if(mode==1)f.last_odom_time_.value=98.;
    p.set_value({mode==2?std::nullopt:std::optional<scan_planner::Manager::SolvedCandidate>(scan_planner::Manager::SolvedCandidate{}),"failed_optimization"});
    f.pollSolveWorker();expect(f.publishes==0 && f.waits==1 && f.planner_manager_->curve==17,
      "changed_map_stale_pose_or_failed_candidate_never_overwrites_incumbent");
  }
  {
    F f;std::promise<F::WorkerResult> p;setup(f,p);f.planning_only_=true;
    f.planner_manager_->safe=false;
    f.planner_manager_->outcome=scan_planner::CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed;
    const auto budget=f.active_solve_budget_;
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});
    expect(f.pollSolveWorker() && f.state==F::REPLAN_TRAJ && f.transitions==1 &&
      f.waits==0 && f.publishes==0 && f.planner_manager_->curve==17 &&
      f.planner_manager_->incumbent_proof && f.replan_fail_count_==1 && budget->allowed(),
      "planning_only_pending_candidate_requests_measured_reseed_without_waiting_for_environment_or_retiring_incumbent");
    expect(!f.pollSolveWorker() && f.transitions==1 && f.waits==0,
      "planning_only_reseed_consumes_one_candidate_result_not_a_second_worker");
  }
  for(const auto outcome:{scan_planner::CandidateAcceptanceOutcome::PreviewSourceExpired,
      scan_planner::CandidateAcceptanceOutcome::PreviewSourceChanged,
      scan_planner::CandidateAcceptanceOutcome::PreviewGeometryBlocked,
      scan_planner::CandidateAcceptanceOutcome::InvalidPreviewCandidate}) {
    F f;std::promise<F::WorkerResult> p;setup(f,p);f.planning_only_=true;
    f.planner_manager_->safe=false;f.planner_manager_->outcome=outcome;
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});f.pollSolveWorker();
    expect(f.waits==1 && f.transitions==0 && f.publishes==0 && f.planner_manager_->curve==17 &&
      f.planner_manager_->incumbent_proof && f.last_attempt_failure_phase_==scan_planner::candidateAcceptanceReason(outcome),
      "preview_source_geometry_or_invalid_failure_retains_precise_existing_environment_wait_not_measured_reseed");
  }
  {
    F f;std::promise<F::WorkerResult> p;setup(f,p);f.planner_manager_->safe=false;
    f.planner_manager_->outcome=scan_planner::CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed;
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});f.pollSolveWorker();
    expect(f.waits==1 && f.transitions==0 && f.last_attempt_failure_phase_==
      scan_planner::candidateAcceptanceReason(f.planner_manager_->outcome),
      "motion_completion_does_not_inherit_preview_reseed_policy");
  }
  {
    F f;std::promise<F::WorkerResult> p;setup(f,p);f.planning_only_=true;f.last_odom_time_.value=98.;
    f.planner_manager_->diagnostics.record(scan_planner::CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed);
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});f.pollSolveWorker();
    expect(f.waits==1 && f.transitions==0 && f.planner_manager_->adoptions==0 && f.planner_manager_->curve==17 &&
      f.last_attempt_failure_phase_==scan_planner::candidateAcceptanceReason(scan_planner::CandidateAcceptanceOutcome::PreviewSourceExpired),
      "stale_actual_body_does_not_reuse_previous_candidate_reseed_diagnostic");
  }
  {
    F f;std::promise<F::WorkerResult> p;setup(f,p);f.planning_only_=true;auto now=B::Clock::now();
    f.active_solve_budget_=std::make_shared<B>(std::chrono::milliseconds(400),[&]{return now;});
    f.planner_manager_->outcome=scan_planner::CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed;
    now+=std::chrono::milliseconds(400);p.set_value({scan_planner::Manager::SolvedCandidate{},""});f.pollSolveWorker();
    expect(f.waits==1 && f.transitions==0 && f.publishes==0 && f.planner_manager_->curve==17 &&
      f.last_attempt_failure_phase_=="solve_deadline_exceeded",
      "expired_shared_budget_cannot_turn_preview_reseed_outcome_into_new_request");
  }
  {
    F f;std::promise<F::WorkerResult> p;setup(f,p);const auto budget=f.active_solve_budget_;
    p.set_exception(std::make_exception_ptr(std::runtime_error("native solve error")));
    expect(f.pollSolveWorker() && f.waits==1 && f.publishes==0 &&
      f.planner_manager_->curve==17 && !f.active_solve_budget_ && !f.solve_future_.valid() &&
      budget->allowed() && f.last_attempt_failure_phase_=="solve_exception",
      "future_exception_retires_request_and_uses_existing_incumbent_recheck_path");
  }
  for(int mode=0;mode<2;++mode){
    F f;std::promise<F::WorkerResult> p;setup(f,p);f.solve_predecessor_=17;
    if(mode==0)++f.reference_generation_;else f.cancelSolveWorker();
    p.set_exception(std::make_exception_ptr(std::runtime_error("late old exception")));
    expect(!f.pollSolveWorker() && f.waits==0 && f.publishes==0 &&
      f.planner_manager_->imports==0 && !f.active_solve_budget_ && !f.solve_predecessor_,
      "retired_generation_or_cancelled_exception_has_no_diagnostics_or_recovery_side_effect");
  }
  {
    F f;std::promise<F::WorkerResult> p;setup(f,p);const auto budget=f.active_solve_budget_;
    f.solve_predecessor_=17;p.set_exception(std::make_exception_ptr(std::bad_alloc{}));
    bool propagated=false;try{f.pollSolveWorker();}catch(const std::bad_alloc&){propagated=true;}
    expect(propagated && !f.active_solve_budget_ && !f.solve_future_.valid() &&
      !f.solve_predecessor_ && !budget->allowed() && f.publishes==0 && f.waits==0,
      "allocation_failure_retires_resources_and_fail_stops_without_claiming_safe_recovery");
  }
  {
    F f;f.owner_map_available=false;std::promise<F::WorkerResult> p;setup(f,p);
    const auto budget=f.active_solve_budget_;const auto deadline=budget->deadline();
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});f.execFSMCallback();
    expect(!f.solve_future_.valid()&&f.completed_solve_&&f.solveWorkPending()&&
      !f.active_solve_budget_&&f.publishes==0&&f.planner_manager_->adoptions==0&&f.map_entries==0,
      "ready_result_collected_without_map_but_still_reserves_single_worker");
    f.execFSMCallback();
    expect(f.completed_solve_&&f.completed_solve_->budget==budget&&budget->deadline()==deadline&&f.planner_manager_->imports==0,
      "repeated_map_misses_keep_one_value_original_budget_and_no_adoption");
    f.owner_map_available=true;f.execFSMCallback();f.execFSMCallback();
    expect(!f.solveWorkPending()&&f.publishes==1&&f.planner_manager_->adoptions==1&&
      f.planner_manager_->curve==18&&budget->deadline()==deadline,
      "returned_map_adopts_completed_value_once_through_original_admission");
  }
  for(int kind=0;kind<3;++kind) {
    F f;f.owner_map_available=false;std::promise<F::WorkerResult> p;setup(f,p);
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});f.execFSMCallback();
    if(kind==0)f.cancelSolveWorker();else if(kind==1)++f.reference_generation_;else f.have_target_=false;
    f.execFSMCallback();
    expect(!f.solveWorkPending()&&!f.completed_solve_&&!f.solve_predecessor_&&f.publishes==0&&
      f.waits==0&&f.planner_manager_->imports==0,
      "map_missing_cancel_replace_or_task_end_retires_staged_candidate_without_side_effect");
    f.owner_map_available=true;f.execFSMCallback();
    expect(f.publishes==0&&f.waits==0,"later_map_cannot_revive_retired_completion");
  }
  {
    F f;f.owner_map_available=false;std::promise<F::WorkerResult> p;setup(f,p);
    auto now=B::Clock::now();f.active_solve_budget_=std::make_shared<B>(std::chrono::milliseconds(400),[&]{return now;});
    const auto budget=f.active_solve_budget_;const auto deadline=budget->deadline();
    p.set_value({scan_planner::Manager::SolvedCandidate{},""});f.execFSMCallback();
    now+=std::chrono::milliseconds(400);f.execFSMCallback();
    expect(!f.completed_solve_&&!f.solve_future_.valid()&&f.retired_solve_failure_&&
      f.retired_solve_failure_->reason=="solve_deadline_exceeded"&&f.publishes==0&&f.waits==0&&
      budget->deadline()==deadline,"expired_candidate_freed_without_map_no_budget_renewal_or_fake_verdict");
    f.owner_map_available=true;f.execFSMCallback();f.execFSMCallback();
    expect(!f.solveWorkPending()&&f.waits==1&&f.publishes==0&&f.planner_manager_->curve==17&&
      f.last_attempt_failure_phase_=="solve_deadline_exceeded",
      "expired_completion_uses_original_map_scoped_recovery_once_without_adoption");
  }
  for(int kind=0;kind<3;++kind) {
    F f;f.owner_map_available=false;std::promise<F::WorkerResult> p;setup(f,p);
    if(kind==0)p.set_value({{},"failed_search"});
    else p.set_exception(std::make_exception_ptr(std::runtime_error("worker_failed")));
    if(kind==2)f.cancelSolveWorker();
    f.execFSMCallback();
    expect(!f.solve_future_.valid()&&!f.completed_solve_&&f.publishes==0&&f.waits==0,
      "failed_or_cancelled_future_is_consumed_without_map");
    if(kind==2)expect(!f.solveWorkPending()&&f.planner_manager_->imports==0,
      "cancelled_error_has_no_failure_retry_or_diagnostic_import");
    else {
      expect(f.retired_solve_failure_.has_value()&&f.solveWorkPending(),
        "failure_metadata_waits_for_original_environment_capture_not_another_solve");
      f.owner_map_available=true;f.execFSMCallback();f.execFSMCallback();
      expect(f.waits==1&&f.publishes==0&&!f.solveWorkPending()&&f.planner_manager_->curve==17&&
        f.last_attempt_failure_phase_==(kind==0?"failed_search":"solve_exception"),
        "failed_completion_recovers_once_after_map_returns_and_preserves_incumbent");
    }
  }
  {
    F f;f.owner_map_available=false;std::promise<F::WorkerResult> p;setup(f,p);
    p.set_exception(std::make_exception_ptr(std::bad_alloc{}));bool threw=false;
    try{f.execFSMCallback();}catch(const std::bad_alloc&){threw=true;}
    expect(threw&&!f.solveWorkPending()&&!f.active_solve_budget_&&f.map_entries==0&&f.publishes==0,
      "resource_exhaustion_fail_stop_is_not_hidden_behind_missing_map");
  }
  {
    F f;f.owner_map_available=false;std::promise<F::WorkerResult> first;setup(f,first);
    first.set_value({{},"old_failed_search"});
    for(unsigned tick=0;tick<50;++tick)f.execFSMCallback();
    expect(f.retired_solve_failure_&&!f.completed_solve_&&!f.solve_future_.valid()&&
      f.waits==0&&f.planner_manager_->imports==1&&f.map_entries==0,
      "long_map_unavailability_keeps_only_one_failure_metadata_no_repeated_import_or_query");
    f.cancelSolveWorker();++f.reference_generation_;f.solve_generation_=f.reference_generation_;
    expect(!f.solveWorkPending()&&!f.retired_solve_failure_&&f.waits==0,
      "cancel_discards_retired_failure_before_replacement_without_waiting_for_map");
    std::promise<F::WorkerResult> replacement;setup(f,replacement);
    replacement.set_value({scan_planner::Manager::SolvedCandidate{19,{ }},""});f.execFSMCallback();
    expect(f.completed_solve_&&f.completed_solve_->generation==f.reference_generation_&&f.publishes==0,
      "replacement_completion_has_its_own_generation_while_map_still_unavailable");
    f.owner_map_available=true;f.execFSMCallback();
    expect(f.publishes==1&&f.planner_manager_->curve==19&&f.waits==0&&!f.solveWorkPending(),
      "map_return_cannot_apply_retired_failure_to_new_generation");
  }
  std::cout<<"checks="<<checks<<" failures="<<failures<<"\n";return failures?1:0;
}
'''

# Extract the production scope AND production async launch. Only map queries
# and the solve outcome are passive: this is a wiring/lifetime test, not a
# geometry or optimizer simulation. Native GridMap tests cover actual voxels.
WORKER_BINDING_HARNESS = r'''
#include <plan_env/solve_budget.hpp>
#include <plan_manage/worker_lifecycle.hpp>
#include <nlohmann/json.hpp>
#include <Eigen/Geometry>
#include <future>
#include <optional>
#include <iostream>
#include <string>
#include <utility>
#include <array>
#include <functional>
#include <unordered_map>
namespace scan_planner {
struct MeasuredBodyPose {Eigen::Vector3d position;Eigen::Quaterniond orientation;
  double source_stamp;std::string frame;};
enum class CollisionModel {OrientedBox,Other};
struct BodyEnvelope {CollisionModel model=CollisionModel::OrientedBox;};
struct PassiveModel {
  bool hardCollision(const Eigen::Vector3d&,double,double,double,const BodyEnvelope&)const{return false;}
  std::string describeHardCollision(const Eigen::Vector3d&,double,double,double,const BodyEnvelope&)const{return "{}";}
};
struct StairModelGeometry {
  static std::string boundedDiagnosticId(const std::string& value){return value;}
  static std::string diagnosticDump(const nlohmann::json& value){return value.dump();}
};
struct PlanningSupport {int identity=73;bool stairs=true;
  struct Source {std::string frame_id="odom";} source;
  bool isStair()const{return stairs;}
  bool stairClearanceAllowed(const Eigen::Vector3d&)const{return stairs;}
  struct Fit {bool valid=true;double center_residual=.27;};
  Fit terrainFit(const Eigen::Vector3d&)const{return {};}
  std::shared_ptr<const PassiveModel> model()const{return {};}};
// Passive source/ownership instrumentation only. The production classifier's
// terrain and obstacle decisions have separate native geometry regressions.
struct StairContactQuery {
  static inline int calls=0,last_identity=-1;
  std::shared_ptr<const PlanningSupport> support;std::string frame;double resolution;
  StairContactQuery(std::shared_ptr<const PlanningSupport> s,std::string f,double r):
    support(std::move(s)),frame(std::move(f)),resolution(r){}
  bool valid()const{return support&&support->isStair()&&frame==support->source.frame_id&&resolution==.05;}
  bool matchesObservedBounds(const Eigen::Vector3d& lo,const Eigen::Vector3d&)const{
    ++calls;last_identity=support->identity;return valid()&&lo.x()<.5;}
};
struct StairRiserContactQuery:StairContactQuery {
  using StairContactQuery::StairContactQuery;
  bool matchesObservedBounds(const Eigen::Vector3d& lo,const Eigen::Vector3d&)const{
    ++calls;last_identity=support->identity;return valid()&&lo.x()>=.5;}
};
struct GridMap {
  using Ptr=std::shared_ptr<GridMap>;
  using QueryAttitudeProvider=std::function<std::optional<Eigen::Vector2d>(const Eigen::Vector3d&,double)>;
  using QuerySupportHeightProvider=std::function<std::optional<double>(const Eigen::Vector3d&)>;
  using QueryStairClearanceProvider=std::function<bool(const Eigen::Vector3d&)>;
  using QueryStairContactProvider=std::function<bool(const Eigen::Vector3d&,const Eigen::Vector3d&)>;
  using QueryHardBoundaryProvider=std::function<bool(const Eigen::Vector3d&,double,double,double,const BodyEnvelope&)>;
  using QueryHardBoundaryDiagnosticProvider=std::function<std::string(const Eigen::Vector3d&,double,double,double,const BodyEnvelope&)>;
  QueryAttitudeProvider attitude;
  QuerySupportHeightProvider height;
  QueryStairClearanceProvider clearance;
  QueryStairContactProvider contact;
  QueryHardBoundaryProvider boundary;
  QueryHardBoundaryDiagnosticProvider boundary_diagnostic;
  bool stairClearanceConfigured()const{return true;}
  bool stairSubvoxelConfigured()const{return false;}
  bool stairContactConfigured()const{return true;}
  double collisionResolution()const{return .05;}
  std::string collisionFrame()const{return "odom";}
  bool integratedCloudFreshAt(std::int64_t)const{return true;}
  BodyEnvelope planningBodyEnvelope()const{return {};}
  void setQueryAttitudeProvider(QueryAttitudeProvider next){attitude=std::move(next);}
  auto exchangeQueryAttitudeProvider(QueryAttitudeProvider next){return std::exchange(attitude,std::move(next));}
  auto exchangeQuerySupportHeightProvider(QuerySupportHeightProvider next){return std::exchange(height,std::move(next));}
  auto exchangeQueryStairClearanceProvider(QueryStairClearanceProvider next){return std::exchange(clearance,std::move(next));}
  auto exchangeQueryStairContactProvider(QueryStairContactProvider next){return std::exchange(contact,std::move(next));}
  auto exchangeQueryHardBoundaryProvider(QueryHardBoundaryProvider next){return std::exchange(boundary,std::move(next));}
  auto exchangeQueryHardBoundaryDiagnosticProvider(QueryHardBoundaryDiagnosticProvider next){return std::exchange(boundary_diagnostic,std::move(next));}
};
inline GridMap::QueryAttitudeProvider stairAttitudeProvider(
    std::shared_ptr<const PlanningSupport> support,const MeasuredBodyPose& body,bool box=true){
  if(!box||!support||!support->isStair())return {};
  return [support=std::move(support),body](const Eigen::Vector3d&,double)->std::optional<Eigen::Vector2d>{
    return Eigen::Vector2d(support->identity,body.source_stamp);
  };
}
__PRODUCTION_SCOPE__
struct WorkerProbe {
  struct Time {double seconds()const{return 99.97;}std::int64_t nanoseconds()const{return 99970000000LL;}};
  struct Validator {std::shared_ptr<const PlanningSupport> support=std::make_shared<PlanningSupport>();
    std::shared_ptr<const PlanningSupport> planningSupport(int version){
      if(version!=4)throw std::runtime_error("wrong_support_version");
      return support;}};
  struct Owner {};
  struct Candidate {int id=8;};
  struct Worker {
    GridMap::Ptr map;
    int mode=0,queries=0,releases=0;
    bool expect_stair=true,restored_before_release=false,throw_prepare=false;
    std::shared_future<void> proceed;
    void queryBoth(){
      ++queries;
      if(expect_stair){
        if(!map->attitude||!map->height||!map->clearance||!map->contact)throw std::runtime_error("missing_one_provider");
        const auto a=map->attitude(Eigen::Vector3d::Zero(),0.);
        const auto h=map->height(Eigen::Vector3d(0.,0.,1.));
        if(!a||!h||!map->clearance(Eigen::Vector3d::Zero())||(*a)[0]!=73||(*a)[1]!=99.97||std::abs(*h-1.27)>1e-9||
           !map->contact(Eigen::Vector3d::Zero(),Eigen::Vector3d::Constant(.001))||
           !map->contact(Eigen::Vector3d(1.,0.,0.),Eigen::Vector3d(1.001,.001,.001))||
           StairContactQuery::last_identity!=73)
          throw std::runtime_error("provider_identity_or_body_not_paired");
      } else if(map->attitude||map->height||map->clearance||map->contact)throw std::runtime_error("stair_provider_leaked_into_flat");
    }
    void prepareSolveWorker(const Owner&,const GridMap::Ptr& snapshot){
      map=snapshot;queryBoth();if(throw_prepare)throw std::runtime_error("prepare_failure");
    }
    bool reboundReplan(const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&,
        const Eigen::Vector3d&,const Eigen::Vector3d&,bool,bool,const SolveBudget::Ptr& budget){
      if(proceed.valid())proceed.wait();
      queryBoth();
      if(mode==2)throw std::runtime_error("solver_failure");
      if(mode==3)throw std::bad_alloc{};
      return mode==0&&budget->allowed();
    }
    Candidate takeSolvedCandidate(){queryBoth();return {};}
    std::string lastFailurePhase()const{return mode==0?"":"solver_rejected";}
    void releaseSolveSnapshot()noexcept{
      ++releases;restored_before_release=map&&!map->attitude&&!map->height&&!map->clearance&&!map->contact;map.reset();
    }
  };
  struct WorkerResult {std::optional<Candidate> candidate;std::string failure;
    std::int64_t body_source_ns=0;double queue_ms=0.,solve_ms=0.;SolveBudget::Time finished_at;};
  std::unique_ptr<Validator> execution_validator_=std::make_unique<Validator>();
  std::unique_ptr<Owner> planner_manager_=std::make_unique<Owner>();
  std::unique_ptr<Worker> solve_worker_=std::make_unique<Worker>();
  std::future<WorkerResult> solve_future_;
  int reference_metadata_=4;
  int executionVersion(int v){return v;}
  Eigen::Vector3d odom_pos_{0.,0.,.5};Eigen::Quaterniond odom_orient_=Eigen::Quaterniond::Identity();
  Time last_odom_time_;std::string self_inflation_frame_id_="odom";
  void submit(GridMap::Ptr snapshot,const SolveBudget::Ptr& request_budget){
    __PRODUCTION_BINDING__
    const auto submitted=SolveBudget::Clock::now();
    const auto start=odom_pos_,velocity=Eigen::Vector3d::Zero().eval(),acceleration=velocity;
    const auto target=(start+Eigen::Vector3d(1.,0.,.2)).eval(),target_velocity=velocity;
    const bool poly=true,flag_randomPolyTraj=false;
    const auto solve_body_source_ns=last_odom_time_.nanoseconds();
    __PRODUCTION_LAUNCH__
  }
};
// Compile the whole production synchronous solve/adoption tail. Its manager
// queries are passive, exactly like the asynchronous owner above: this checks
// shared support identity and RAII lifetime, not solver feasibility.
struct SynchronousProbe {
  struct Time {double value=99.97;double seconds()const{return value;}
    std::int64_t nanoseconds()const{return static_cast<std::int64_t>(value*1e9);}
    Time operator-(Time other)const{return {value-other.value};}};
  struct Node {Time now()const{return {100.};}} node;
  Node* node_=&node;
  struct Manager:WorkerProbe::Worker {
    GridMap::Ptr grid_map_=std::make_shared<GridMap>();
    struct Data {int traj_id_=18;} local_data_;
    int commits=0,discards=0,predecessors=0;bool commit_safe=true;
    Manager(){map=grid_map_;}
    bool commitPreviewCandidate(const SolveBudget::Ptr& budget){
      queryBoth();++commits;return commit_safe&&budget->allowed();}
    void discardPreviewCandidate(){queryBoth();++discards;}
    bool recheckPredecessor(int,double){queryBoth();++predecessors;return true;}
  };
  std::unique_ptr<Manager> planner_manager_=std::make_unique<Manager>();
  std::unique_ptr<WorkerProbe::Validator> execution_validator_=std::make_unique<WorkerProbe::Validator>();
  struct Pending {void release(){}} pending_snapshot_request_;
  struct Pool {void cancelSolverRequest(){}} snapshot_pool_;
  SolveBudget::Time last_solve_submit_;
  std::optional<int> actual_body_seed_request_{1};
  int reference_metadata_=4,reference_generation_=7;int executionVersion(int v){return v;}
  Eigen::Vector3d odom_pos_{0.,0.,.5},start_pt_=odom_pos_,start_vel_=Eigen::Vector3d::Zero(),
    start_acc_=start_vel_,local_target_pt_{1.,0.,.7},local_target_vel_=start_vel_;
  Eigen::Quaterniond odom_orient_=Eigen::Quaterniond::Identity();
  Time last_odom_time_,predecessor_check_stamp_;
  std::string self_inflation_frame_id_="odom",last_attempt_failure_phase_;
  bool have_new_target_=true,have_odom_=true,reference_path_guidance_=true,planning_only_=true;
  double odom_timeout_=.5;std::uint64_t predecessor_id_=0;bool predecessor_safe_=false;
  int attempts=0,publishes=0;
  void publishAttemptDebug(int){++attempts;}
  void publishCommittedTrajectory(){planner_manager_->queryBoth();++publishes;}
  bool solve(const SolveBudget::Ptr& request_budget){
    const auto submitted=SolveBudget::Clock::now();
    const bool flag_use_poly_init=true,flag_randomPolyTraj=false;
    const int attempt_header=1,preceding_generation=reference_generation_,preceding_id=17;
    const std::optional<int> predecessor{17};const double predecessor_measured_time=.1;
    using std::cout;using std::endl;
    __PRODUCTION_SYNCHRONOUS_TAIL__
  }
};
}
int main(){
  using namespace scan_planner;
  int checks=0,failures=0;
  auto expect=[&](bool ok,const char* label){++checks;failures+=!ok;
    std::cout<<(ok?"PASS ":"FAIL ")<<label<<'\n';};
  for(int mode=0;mode<7;++mode){
    SynchronousProbe f;
    f.planner_manager_->mode=mode==1?1:(mode==2?2:0);
    f.planner_manager_->commit_safe=mode!=3;
    if(mode==4){f.execution_validator_->support=std::make_shared<PlanningSupport>(PlanningSupport{73,false});
      f.planner_manager_->expect_stair=false;}
    if(mode==5){f.execution_validator_.reset();f.planner_manager_->expect_stair=false;}
    if(mode==6){auto foreign=std::make_shared<PlanningSupport>();foreign->source.frame_id="foreign";
      f.execution_validator_->support=foreign;f.planner_manager_->expect_stair=false;}
    bool threw=false,success=false;std::string exception;
    try{success=f.solve(std::make_shared<SolveBudget>());}catch(const std::runtime_error& e){threw=true;exception=e.what();}
    expect(threw==(mode==2)&&success==(mode==0||mode>=4)&&
      (mode!=2||exception=="solver_failure"),
      "synchronous_exact_support_scope_preserves_solve_failure_exception_flat_missing_and_foreign_support");
    expect(f.publishes==(success?1:0)&&f.planner_manager_->predecessors==(success?1:0)&&
      !f.planner_manager_->grid_map_->attitude&&!f.planner_manager_->grid_map_->height&&
      !f.planner_manager_->grid_map_->clearance&&!f.planner_manager_->grid_map_->contact,
      "synchronous_all_providers_span_solve_refine_commit_predecessor_and_restore_on_exit");
  }
  for(int outcome=0;outcome<5;++outcome){
    WorkerProbe f;auto map=std::make_shared<GridMap>();auto budget=std::make_shared<SolveBudget>();
    f.solve_worker_->mode=outcome==4?0:outcome;
    std::promise<void> gate;f.solve_worker_->proceed=gate.get_future().share();
    std::weak_ptr<const PlanningSupport> weak=f.execution_validator_->support;
    try{f.submit(map,budget);}catch(const std::runtime_error& e){
      expect(false,e.what());continue;
    }
    expect(f.solve_worker_->queries==1&&bool(map->attitude)&&bool(map->height)&&bool(map->clearance)&&bool(map->contact),
      "attitude_height_clearance_contact_installed_before_real_worker_preparation");
    f.execution_validator_->support.reset();
    expect(!weak.expired(),"worker_scope_retains_exact_immutable_support_after_owner_retires_it");
    if(outcome==4)budget->cancel();
    gate.set_value();
    bool threw=false;WorkerProbe::WorkerResult result;
    try{result=f.solve_future_.get();}catch(const std::bad_alloc&){threw=true;}
    expect(threw==(outcome==3)&&bool(result.candidate)==(outcome==0)&&
      (outcome!=2||result.failure=="solve_exception"),"production_lambda_preserves_solve_outcome_and_exception_contract");
    expect(f.solve_worker_->queries>1&&f.solve_worker_->releases==1&&
      f.solve_worker_->restored_before_release&&!map->attitude&&!map->height&&!map->clearance&&!map->contact&&weak.expired(),
      "success_failure_cancel_and_exception_restore_all_providers_before_releasing_pool_slot");
    f.execution_validator_->support=std::make_shared<PlanningSupport>(PlanningSupport{73,false});
    f.solve_worker_->mode=0;f.solve_worker_->expect_stair=false;
    f.solve_worker_->proceed={};f.submit(map,std::make_shared<SolveBudget>());
    const auto flat=f.solve_future_.get();
    expect(bool(flat.candidate)&&f.solve_worker_->restored_before_release&&!map->attitude&&!map->height&&!map->contact,
      "same_snapshot_reused_by_flat_reference_has_no_stair_providers");
  }
  {
    WorkerProbe f;auto map=std::make_shared<GridMap>();f.solve_worker_->throw_prepare=true;
    bool threw=false;try{f.submit(map,std::make_shared<SolveBudget>());}catch(const std::runtime_error&){threw=true;}
    expect(threw&&!map->attitude&&!map->height&&!map->clearance&&!map->contact&&!f.solve_future_.valid(),
      "prepare_exception_restores_all_providers_without_starting_worker");
  }
  {
    WorkerProbe f;auto map=std::make_shared<GridMap>();f.execution_validator_.reset();
    f.solve_worker_->expect_stair=false;f.submit(map,std::make_shared<SolveBudget>());
    expect(bool(f.solve_future_.get().candidate)&&f.solve_worker_->restored_before_release,
      "missing_support_keeps_existing_upright_query_behavior");
  }
  {
    GridMap map;auto support=std::make_shared<PlanningSupport>();
    MeasuredBodyPose body{Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),99.97,"odom"};
    map.contact=[](const Eigen::Vector3d&,const Eigen::Vector3d&){return false;};
    const auto before=StairContactQuery::calls;
    {
      StairAttitudeScope scope(map,support,body);
      expect(map.contact(Eigen::Vector3d::Zero(),Eigen::Vector3d::Constant(.001))&&
        StairContactQuery::last_identity==73,"contact_uses_current_immutable_support_identity");
      const auto once=StairContactQuery::calls;
      expect(map.contact(Eigen::Vector3d::Zero(),Eigen::Vector3d::Constant(.001))&&
        StairContactQuery::calls==once&&once==before+1,"contact_cache_is_owned_by_exact_scope_and_bounds");
      {
        auto flat=std::make_shared<PlanningSupport>();flat->stairs=false;
        StairAttitudeScope nested(map,flat,body);
        expect(!map.contact,"nested_flat_scope_removes_contact_not_just_attitude");
      }
      expect(bool(map.contact)&&map.contact(Eigen::Vector3d::Zero(),Eigen::Vector3d::Constant(.001)),
        "nested_scope_restores_exact_outer_contact_binding");
    }
    expect(bool(map.contact)&&!map.contact(Eigen::Vector3d::Zero(),Eigen::Vector3d::Constant(.001)),
      "retired_scope_restores_preexisting_contact_provider");
    support=std::make_shared<PlanningSupport>();support->identity=91;
    {
      StairAttitudeScope scope(map,support,body);
      expect(map.contact(Eigen::Vector3d::Zero(),Eigen::Vector3d::Constant(.001))&&
        StairContactQuery::last_identity==91,"same_bounds_new_support_does_not_reuse_retired_contact_cache");
    }
  }
  std::cout<<"checks="<<checks<<" failures="<<failures<<'\n';return failures?1:0;
}
'''

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',type=Path)
    parser.add_argument('--source',type=Path)
    parser.add_argument('--owner-order-source',type=Path,
                        help='Use another exact production owner callback; all completion/adoption inputs stay identical')
    args=parser.parse_args()
    package=Path(__file__).resolve().parents[1]
    source=args.source or package/'src/scan_replan_fsm.cpp'
    text=source.read_text()
    start=text.index('  void SCANReplanFSM::cancelSolveWorker()')
    end=text.index('  void SCANReplanFSM::publishCommittedTrajectory()',start)
    methods=text[start:end]
    owner_text=args.owner_order_source.read_text() if args.owner_order_source else text
    callback_start=owner_text.index('  void SCANReplanFSM::execFSMCallback()')
    callback_end=owner_text.index('  void SCANReplanFSM::execFSMOwnerCallback()',callback_start)
    methods+='\n'+owner_text[callback_start:callback_end]
    poll=methods[methods.index('  bool SCANReplanFSM::pollSolveWorker()'):]
    ordering=(poll.index('if (completion.generation!=reference_generation_'),
              poll.index('planner_manager_->importAttemptDiagnostics'),
              poll.index('refreshPlanningBodyFromInbox();'),
              poll.index('const auto adoption_support='),
              poll.index('StairAttitudeScope adoption_attitude('),
              poll.index('planner_manager_->recheckPredecessor('),
              poll.index('planner_manager_->adoptSolvedCandidate('))
    assert all(a<b for a,b in zip(ordering,ordering[1:])), 'owner adoption attitude ordering changed'
    assert methods.count('refreshPlanningBodyFromInbox();')==1, 'adoption must use one acquired body sample'
    fusion_start=text.index('setCollisionUpdateCallback([this]{')+len('setCollisionUpdateCallback([this]{')
    fusion_end=text.index('      });',fusion_start)
    fusion_callback=text[fusion_start:fusion_end]
    refresh_comment='      // Target selection (especially a side search)'
    refresh=''
    if refresh_comment in text:
        refresh_start=text.index('      if(local_follower_) {',text.index(refresh_comment))
        refresh_end=text.index('      // A slot miss did not start a solve.',refresh_start)
        refresh=text[refresh_start:refresh_end]
    selection_start=text.index('  bool SCANReplanFSM::getLocalTarget(')
    side_start=text.index('          auto snapshot=',selection_start)
    side_end=text.index('          // Waiting is part of the parent',side_start)
    side_acquisition=text[side_start:side_end]
    prepare='solve_worker_->prepareSolveWorker(*planner_manager_,snapshot);'
    if prepare not in text:
        raise ValueError('production worker preparation boundary missing')
    capture_start=text.index('      const auto start=start_pt_, velocity=start_vel_, acceleration=start_acc_;')
    capture_end=text.index('      const bool poly=',capture_start)
    capture=text[capture_start:capture_end]
    binding_start=text.index('      const auto support=',text.index('      last_solve_submit_=',text.index(refresh_comment)))
    binding_end=text.index(prepare,binding_start)+len(prepare)
    binding=text[binding_start:binding_end]
    launch_start=text.index('        solve_future_=std::async',binding_end)
    launch_end=text.index('      actual_body_seed_request_.reset();',launch_start)
    launch=text[launch_start:launch_end]
    sync_start=text.index('    pending_snapshot_request_.release();',launch_end)
    sync_end=text.index('\n  }\n\n  void SCANReplanFSM::cancelSnapshotAcquisition()',sync_start)
    synchronous=text[sync_start:sync_end]
    scope_text=(package/'include/plan_manage/stair_attitude_scope.hpp').read_text()
    scope=scope_text[scope_text.index('class StairAttitudeScope {'):scope_text.index('} // namespace scan_planner')]
    rendered=(HARNESS.replace('__METHODS__',methods).replace('__FUSION_CALLBACK__',fusion_callback)
        .replace('__LATEST_BODY_REFRESH__',refresh).replace('__SIDE_SNAPSHOT_ACQUIRE__',side_acquisition)
        .replace('__PREPARE_WORKER__',prepare).replace('__CAPTURE_INPUTS__',capture))
    with tempfile.TemporaryDirectory(prefix='native-worker-completion-') as directory:
        cpp,binary=Path(directory)/'worker.cpp',Path(directory)/'worker'
        cpp.write_text(rendered)
        command=['c++','-std=c++17','-pthread','-Wall','-Wextra',
                 '-I'+str(package/'include'),'-I'+str(package.parent/'plan_env/include'),
                 '-I'+str(package.parent/'bspline_opt/include'),'-I/usr/include/eigen3',
                 str(cpp),'-o',str(binary)]
        subprocess.run(command,check=True)
        result=subprocess.run([str(binary)],capture_output=True,text=True)
        print(result.stdout,end='');print(result.stderr,end='')
        binding_cpp,binding_binary=Path(directory)/'worker_binding.cpp',Path(directory)/'worker_binding'
        binding_cpp.write_text(WORKER_BINDING_HARNESS.replace('__PRODUCTION_SCOPE__',scope)
            .replace('__PRODUCTION_BINDING__',binding).replace('__PRODUCTION_LAUNCH__',launch)
            .replace('__PRODUCTION_SYNCHRONOUS_TAIL__',synchronous))
        binding_command=command[:-3]+[str(binding_cpp),'-o',str(binding_binary)]
        subprocess.run(binding_command,check=True)
        binding_result=subprocess.run([str(binding_binary)],capture_output=True,text=True)
        print(binding_result.stdout,end='');print(binding_result.stderr,end='')
        if args.report:
            args.report.write_text(json.dumps(dict(source=str(source),
                owner_order_source=str(args.owner_order_source or source),
                production_method_sha256=hashlib.sha256(methods.encode()).hexdigest(),
                production_wiring_sha256=hashlib.sha256((fusion_callback+refresh+side_acquisition+prepare+capture).encode()).hexdigest(),
                production_worker_binding_sha256=hashlib.sha256((scope+binding+launch).encode()).hexdigest(),
                production_synchronous_binding_sha256=hashlib.sha256(synchronous.encode()).hexdigest(),
                worker_binding_returncode=binding_result.returncode,worker_binding_stdout=binding_result.stdout,
                compile_command=command,returncode=result.returncode,stdout=result.stdout,
                ros_initialized=False,scope='actual native worker completion/cancel/fusion callback and submission wiring; passive map/target/adoption seams, not feasibility evidence'),indent=2)+'\n')
        return result.returncode or binding_result.returncode

if __name__=='__main__':
    raise SystemExit(main())
