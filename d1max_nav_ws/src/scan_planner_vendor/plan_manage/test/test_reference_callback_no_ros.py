#!/usr/bin/env python3
"""Compile the real reference callbacks against passive transport/test doubles.

No rclcpp initialization, ROS graph, map substitute, optimizer or robot commands.
Only clock/messages/publication sinks are doubled. Both callback bodies and the
generation/waypoint helpers are production source, including for baseline runs.
The scope is reference admission and refusal notification, not path feasibility.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path
import subprocess
import tempfile


HARNESS = r'''
#include <plan_manage/input_contract.hpp>
#include <plan_manage/reference_receipt_history.hpp>
#include <plan_manage/rolling_reference.hpp>
#include <plan_manage/reference_target.hpp>
#include <Eigen/Geometry>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <optional>
#include <tuple>
#include <chrono>
#include <functional>
#include <mutex>
#define RCLCPP_WARN(...) ((void)0)
#define RCLCPP_WARN_THROTTLE(...) ((void)0)
#define RCLCPP_ERROR(...) ((void)0)
#define RCLCPP_INFO(...) ((void)0)
struct Header { std::string frame_id;
  bool operator==(const Header&b)const{return frame_id==b.frame_id;} };
struct Position { double x=0., y=0., z=0.;
  bool operator==(const Position&b)const{return std::tie(x,y,z)==std::tie(b.x,b.y,b.z);} };
struct PoseStamped { Header header; struct { Position position; } pose;
  bool operator==(const PoseStamped&b)const{return header==b.header&&pose.position==b.pose.position;} };
struct ClockValue { double value=0.; double seconds() const {return value;}
  std::int64_t nanoseconds()const{return static_cast<std::int64_t>(value*1e9);}
  ClockValue operator-(ClockValue b) const {return {value-b.value};}
  ClockValue operator+(ClockValue b) const {return {value+b.value};}
  bool operator>(ClockValue b) const {return value>b.value;}
  bool operator==(ClockValue b)const{return value==b.value;} };
namespace builtin_interfaces { namespace msg { using Time=ClockValue; }}
namespace rclcpp { using Time=ClockValue;
  struct Duration { static ClockValue from_seconds(double value){return {value};} }; }
namespace nav_msgs { namespace msg {
struct Path { using ConstSharedPtr=std::shared_ptr<const Path>; Header header;
  std::vector<PoseStamped> poses;
  bool operator==(const Path&b)const{return header==b.header&&poses==b.poses;} };
}}
namespace d1max_planning_interfaces { namespace msg {
struct ReferencePath { using ConstSharedPtr=std::shared_ptr<const ReferencePath>;
  std::string session_id; std::uint64_t generation=0; nav_msgs::msg::Path path;
  std::uint32_t schema_version=0;
  std::string point_reference,task_id,route_id,route_hash,map_version_id,segment_id;
  std::string segment_kind,required_mode,anchor_id,localization_seed_id;
  std::uint64_t anchor_revision=0,map_geometry_revision=0,context_sequence=0,localization_epoch=0;
  std::vector<std::string> point_segment_ids,point_segment_kinds,point_required_modes;
  bool operator==(const ReferencePath&b)const{
    return std::tie(session_id,generation,schema_version,point_reference,task_id,route_id,route_hash,
      map_version_id,segment_id,point_segment_ids,point_segment_kinds,point_required_modes)==
      std::tie(b.session_id,b.generation,b.schema_version,b.point_reference,b.task_id,b.route_id,b.route_hash,
      b.map_version_id,b.segment_id,b.point_segment_ids,b.point_segment_kinds,b.point_required_modes)&&path==b.path;
  }
};
struct ExecutionVersion {
  std::string session_id,task_id,route_id,route_hash,map_version_id;
  std::string anchor_id,segment_id,localization_seed_id;
  std::uint64_t reference_generation=0,anchor_revision=0,context_sequence=0,localization_epoch=0,map_geometry_revision=0;
  bool operator==(const ExecutionVersion& b) const {
    return std::tie(session_id,task_id,route_id,route_hash,map_version_id,reference_generation,
      anchor_id,segment_id,localization_seed_id,anchor_revision,context_sequence,localization_epoch,map_geometry_revision)==
      std::tie(b.session_id,b.task_id,b.route_id,b.route_hash,b.map_version_id,b.reference_generation,
      b.anchor_id,b.segment_id,b.localization_seed_id,b.anchor_revision,b.context_sequence,b.localization_epoch,b.map_geometry_revision);
  }
  bool operator!=(const ExecutionVersion&b)const{return !(*this==b);}
};
struct ReferenceProposal {using ConstSharedPtr=std::shared_ptr<const ReferenceProposal>;
  ExecutionVersion version,expected_version;std::string proposal_id,transport_mode;
  std::int64_t expected_trajectory_id=0;ClockValue source_stamp,valid_until;ReferencePath reference;
  Position goal_position;bool has_goal_yaw=false;double goal_yaw=0.,goal_yaw_tolerance_rad=0.;
  bool operator==(const ReferenceProposal&b)const{
    return version==b.version&&expected_version==b.expected_version&&reference==b.reference&&
      goal_position==b.goal_position&&source_stamp==b.source_stamp&&valid_until==b.valid_until&&
      std::tie(proposal_id,transport_mode,expected_trajectory_id,has_goal_yaw,goal_yaw,goal_yaw_tolerance_rad)==
      std::tie(b.proposal_id,b.transport_mode,b.expected_trajectory_id,b.has_goal_yaw,b.goal_yaw,b.goal_yaw_tolerance_rad);
  }
  bool operator!=(const ReferenceProposal&b)const{return !(*this==b);}
};
struct ReferenceReceipt {ExecutionVersion version,expected_version;std::string proposal_id,transport_mode,reason;
  std::int64_t expected_trajectory_id=0;ClockValue source_stamp,valid_until;bool accepted=false;
};
// Wire fields read by production reference admission. This passive fixture
// does not replace the command/SDK validator or grant execution permission.
struct MotionDemand {ExecutionVersion version;std::int64_t trajectory_id=0;
  std::uint64_t sequence=0;ClockValue source_stamp,valid_until;};
}}
struct PassiveNode { mutable double clock=100.; double increment=0.;
  ClockValue now() const {const ClockValue value{clock};clock+=increment;return value;} };
namespace scan_planner {
namespace ew=d1max_planning_interfaces::msg;
// Passive trace sink: production admission and locking are still extracted
// verbatim; tracing cannot grant/deny a reference in this fixture.
struct BoundedExecutionTrace {
  enum class Kind {ReferenceDecision};
  struct Text {void set(const char*){}};
  struct Event {Kind kind{};std::int64_t ros_ns=0;bool accepted=false;Text reason;};
  template<class F>void attempt(F&& f){f();}
};
inline ew::ExecutionVersion executionVersion(const ew::ReferencePath& m) {
  ew::ExecutionVersion v;v.session_id=m.session_id;v.task_id=m.task_id;v.route_id=m.route_id;
  v.route_hash=m.route_hash;v.map_version_id=m.map_version_id;v.reference_generation=m.generation;
  v.anchor_id=m.anchor_id;v.anchor_revision=m.anchor_revision;v.context_sequence=m.context_sequence;
  v.segment_id=m.segment_id;v.localization_epoch=m.localization_epoch;
  v.localization_seed_id=m.localization_seed_id;v.map_geometry_revision=m.map_geometry_revision;return v;
}
inline bool sameExecutionTask(const ew::ExecutionVersion&a,const ew::ExecutionVersion&b) {
  return a.session_id==b.session_id&&a.task_id==b.task_id&&a.route_id==b.route_id&&
    a.route_hash==b.route_hash&&a.map_version_id==b.map_version_id&&
    a.localization_epoch==b.localization_epoch&&a.localization_seed_id==b.localization_seed_id;
}
struct PassiveExecutionLedger {bool accepts=true,retired=false;
  bool commit_in_flight=false;
  bool localCommitInFlight()const{return commit_in_flight;}
  bool replacement_allowed=true;int replacements=0;
  std::int64_t committed_curve=1;unsigned rolling_replacements=0;
  std::optional<ew::ReferenceProposal> pending;
  bool cancelReference(const ew::ExecutionVersion&){if(accepts)retired=true;return accepts;}
  // Only storage/threading is passive. The actual admission function below
  // is extracted verbatim from production, including its strict default and
  // narrowly classified same-reference curve advance. No fake acceptance.
  struct Job {ew::ExecutionVersion version;struct {struct {std::int64_t traj_id=0;} trajectory;} spline;};
  std::optional<Job> active_,prepared_,candidate_,queued_candidate_;
  std::mutex mutex_,command_delivery_mutex_;bool local_follower_=true,task_closed_=false;
  std::optional<int> pending_committed_display_;
  BoundedExecutionTrace execution_trace_;PassiveNode* node_=nullptr;
  struct TraceIdentity {const ew::ExecutionVersion* version;std::int64_t trajectory;std::string proposal;};
  void traceLocked(BoundedExecutionTrace::Event&,TraceIdentity,TraceIdentity){}
  static bool sameJob(const Job&a,const Job&b) {
    return a.version==b.version&&a.spline.trajectory.traj_id==b.spline.trajectory.traj_id;
  }
  std::optional<int> local_prepare_request_;
  __PENDING_LOCAL_DEMAND__
  std::optional<ew::ExecutionVersion> task_domain_,candidate_reference_,rolling_stair_predecessor_;
  std::optional<int> latest_permit_,latest_demand_;std::vector<int> permits_;
  unsigned motion_stop_barrier_=0;
  bool referenceTaskOpenLocked(const ew::ExecutionVersion&v)const {
    return !retired&&(!committed||sameExecutionTask(*committed,v));
  }
  void resetTaskLeaseDomainLocked(){}
  void clearCandidatesLocked(){++replacements;pending.reset();candidate_.reset();queued_candidate_.reset();}
  __REFERENCE_ADMISSION__
  bool replacePendingReference(const ew::ReferenceProposal& p,bool rolling_reference=false,
      bool continuous_stair_portal=false,
      const std::optional<ew::ExecutionVersion>& classified_predecessor=std::nullopt) {
    if(!replacement_allowed)return false;
    active_.reset();if(committed){active_=Job{};active_->version=*committed;
      active_->spline.trajectory.traj_id=committed_curve;}
    const bool accepted=productionReplacePendingReference(p,rolling_reference,continuous_stair_portal,
        classified_predecessor);
    if(accepted&&rolling_reference)++rolling_replacements;
    return accepted;
  }
  std::optional<ew::ExecutionVersion> committed;
  struct CommittedIdentity {ew::ExecutionVersion version;std::int64_t trajectory_id;};
  std::optional<CommittedIdentity> committedIdentity()const{
    if(committed)return CommittedIdentity{*committed,committed_curve};
    return {};
  }
};
struct PassiveReceiptSink {std::vector<ew::ReferenceReceipt> receipts;
  void publish(const ew::ReferenceReceipt& x){receipts.push_back(x);} };
class SCANReplanFSM {
public:
  enum class OwnerInput {Odom,Progress,Proposal,TypedPath,Path,Goal,Count};
  unsigned owner_dispatches_=0;
  bool owner_map_available=true;
  void dispatchOwnerInput(OwnerInput,std::uint64_t,std::function<void()> callback){
    ++owner_dispatches_;if(owner_map_available)callback();}
  std::optional<ew::ExecutionVersion> actual_body_seed_request_;
  enum {WAIT_TARGET, GEN_NEW_TRAJ, EXEC_TRAJ, REPLAN_TRAJ};
  PassiveNode clock; PassiveNode *node_=&clock;
  std::string navigation_session_id_="session", self_inflation_frame_id_="map";
  bool have_reference_generation_=false, have_odom_=false;
  std::uint64_t reference_generation_=0, predecessor_id_=0;
  int accepted_curve_id_=-1, replan_fail_count_=0, exec_state_=WAIT_TARGET;
  bool predecessor_safe_=false, have_target_=false, have_new_target_=false;
  bool trigger_=false, need_hover_stop_=false, flag_escape_emergency_=false;
  bool strict_input_frames_=true;
  bool local_follower_=false,reference_path_guidance_=true,pending_splice_=false;
  bool fixed_stair_reference_continuity_=false;
  struct Busy {bool busy=false;bool valid()const{return busy;}bool pending()const{return busy;}};
  Busy solve_future_,pending_snapshot_request_;
  bool solveWorkPending()const{return solve_future_.valid();}
  bool keepPendingSpliceForOrdinaryReplan(){return pending_splice_;}
  bool require_schema_v2_=false;
  bool force_visible_side_target_=false, blocked_entry_replan_pending_=false;
  d1max_planning_interfaces::msg::ReferencePath reference_metadata_;
  std::optional<int> tracking_progress_;
  std::shared_ptr<PassiveExecutionLedger> execution_validator_;
  std::optional<ew::ReferenceProposal> execution_proposal_;
  ew::ReferenceProposal::ConstSharedPtr deferred_rolling_proposal_;
  std::chrono::steady_clock::time_point rolling_retry_priority_until_{};
  std::string execution_transport_mode_="isolated_mock";
  std::optional<std::pair<ew::ExecutionVersion,std::int64_t>> last_visualized_committed_;
  std::optional<int> last_execution_debug_key_;
  struct Visual {unsigned clears=0;void clearOptimalTraj(int id){if(id==0)++clears;}} visual;
  Visual* visualization_=&visual;
  ReferenceReceiptHistory<ew::ReferenceProposal,ew::ReferenceReceipt> execution_receipt_history_;
  PassiveReceiptSink receipt_sink;PassiveReceiptSink*execution_receipt_pub_=&receipt_sink;
  int cancellation_requests_=0;
  void cancelSolveWorker(){++cancellation_requests_;}
  double odom_timeout_=.5, reference_path_z_offset_=.55, reference_start_tolerance_=1.;
  ClockValue last_odom_time_{0.};
  builtin_interfaces::msg::Time predecessor_check_stamp_;
  ReferenceTargetResult local_target_query_debug_;
  Eigen::Vector3d odom_pos_{0.,0.,.55};
  std::vector<Eigen::Vector3d> local_debug_selected_reference_, active_waypoints_;
  struct Event {std::string phase,session; std::uint64_t generation; bool valid;};
  std::vector<Event> emitted;
  int accepted_calls=0, emergency_stop_calls=0;
  int ordinary_solve_starts=0;
  Header localPlanDebugHeader() {return {};}
  void publishAttemptDebug(Header,bool) {}
  void publishInvalidLocalPlanDebug(const std::string &phase,bool=false) {
    emitted.push_back({phase,navigation_session_id_,reference_generation_,false});
  }
  void changeFSMExecState(int state,const std::string&) {exec_state_=state;}
  void callEmergencyStop(const Eigen::Vector3d&) {++emergency_stop_calls;}
  bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d>&) {
    ++accepted_calls; have_target_=true; have_new_target_=true;return true;
  }
  void typedPathCallback(const d1max_planning_interfaces::msg::ReferencePath::ConstSharedPtr&);
  void typedIngress(ew::ReferencePath::ConstSharedPtr);
  void replaceTypedReference(const d1max_planning_interfaces::msg::ReferencePath::ConstSharedPtr&,
                             const std::vector<Eigen::Vector3d>*);
  void installReferenceWaypoints(const std::vector<Eigen::Vector3d>&);
  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr&);
  void proposalCallback(ew::ReferenceProposal::ConstSharedPtr);
  void receiveReferenceProposal(ew::ReferenceProposal::ConstSharedPtr,const std::string&);
  std::optional<std::vector<Eigen::Vector3d>> prepareRollingReference(const ew::ReferencePath&)const;
  void installRollingReference(const ew::ReferencePath&,const std::vector<Eigen::Vector3d>&);
  bool rollingReferenceBusy();
  void processDeferredRollingProposal();
  void rejectDeferredRollingProposal(const std::string&,bool);
  void ordinaryReplanTick();
};
__CALLBACKS__
}
using scan_planner::SCANReplanFSM;
using Message=d1max_planning_interfaces::msg::ReferencePath;
std::shared_ptr<Message> path(std::uint64_t gen) {
  auto m=std::make_shared<Message>();m->session_id="session";m->generation=gen;
  m->path.header.frame_id="map";
  PoseStamped p;p.header.frame_id="map";m->path.poses.push_back(p);
  p.pose.position.x=1.;m->path.poses.push_back(p);return m;
}
void fresh(SCANReplanFSM &f) {f.have_odom_=true;f.last_odom_time_={f.clock.clock};}
std::shared_ptr<scan_planner::ew::ReferenceProposal> proposal(std::uint64_t gen) {
  auto p=std::make_shared<scan_planner::ew::ReferenceProposal>();p->reference=*path(gen);
  auto& m=p->reference;m.schema_version=2;m.point_reference="body_center";m.task_id="task";
  m.route_id="route";m.route_hash="hash";m.map_version_id="map-v";m.segment_id="floor1";
  m.segment_kind="floor";m.required_mode="walk";
  m.point_segment_ids={"floor1","floor1"};m.point_segment_kinds={"floor","floor"};
  m.point_required_modes={"walk","walk"};
  p->version=scan_planner::executionVersion(m);p->proposal_id="proposal-"+std::to_string(gen);
  p->transport_mode="isolated_mock";p->source_stamp={100.};p->valid_until={100.5};return p;
}
int main() {
  int failures=0,checks=0;
  const auto check=[&](bool value,const char*name) {
    ++checks;std::cout<<(value?"PASS ":"FAIL ")<<name<<"\n";if(!value)++failures;
  };
  const auto phase=[](const SCANReplanFSM&f,const char*wanted) {
    return !f.emitted.empty() && f.emitted.back().phase==wanted &&
      !f.emitted.back().valid && f.emitted.back().session=="session" &&
      f.emitted.back().generation==f.reference_generation_;
  };
  const auto rolling=[](std::uint64_t generation,double start,double stamp) {
    auto p=proposal(generation);auto& r=p->reference;
    r.path.poses.clear();r.point_segment_ids.clear();r.point_segment_kinds.clear();r.point_required_modes.clear();
    for(int i=0;i<=40;++i) {
      PoseStamped pose;pose.header.frame_id="map";pose.pose.position={start+.1*i,0.,.55};
      r.path.poses.push_back(pose);r.point_segment_ids.push_back("floor1");
      r.point_segment_kinds.push_back("floor");r.point_required_modes.push_back("walk");
    }
    p->source_stamp={stamp};p->valid_until={stamp+.4};return p;
  };
  {
    // Hard replacement retires only the exact uncommitted command. It cannot
    // discard a still-current incumbent demand or rewrite a command's lease.
    for(int variant=0;variant<4;++variant) {
      scan_planner::PassiveExecutionLedger ledger;
      auto first=proposal(1);ledger.committed=first->version;ledger.committed_curve=101;
      ledger.prepared_=scan_planner::PassiveExecutionLedger::Job{};
      ledger.prepared_->version=first->version;ledger.prepared_->spline.trajectory.traj_id=102;
      ledger.local_prepare_request_=17;
      scan_planner::ew::MotionDemand demand;demand.version=first->version;demand.trajectory_id=102;
      demand.sequence=71;demand.source_stamp={100.01};demand.valid_until={100.15};
      if(variant==2)demand.trajectory_id=101;
      if(variant==3)demand.version.reference_generation=9;
      ledger.pending_local_demand_=scan_planner::PassiveExecutionLedger::PendingLocalDemand{demand,7,variant!=1};
      auto next=proposal(2);next->expected_version=first->version;next->expected_trajectory_id=101;
      check(ledger.replacePendingReference(*next),"hard_replacement_uses_actual_pending_demand_contract");
      check(!ledger.prepared_&&!ledger.local_prepare_request_&&ledger.active_&&
        ledger.active_->version==first->version&&ledger.active_->spline.trajectory.traj_id==101,
        "hard_replacement_retires_prepared_lane_without_changing_incumbent");
      check(ledger.pending_local_demand_.has_value()==(variant!=0),
        "only_exact_prepared_waiting_commit_demand_is_retired");
      if(ledger.pending_local_demand_)check(ledger.pending_local_demand_->demand.sequence==71&&
        ledger.pending_local_demand_->demand.source_stamp.value==100.01&&
        ledger.pending_local_demand_->demand.valid_until.value==100.15&&
        ledger.pending_local_demand_->stop_barrier==7,
        "nonmatching_pending_command_keeps_original_source_deadline_and_barrier");
    }
    scan_planner::PassiveExecutionLedger ledger;
    auto first=proposal(1);ledger.committed=first->version;ledger.committed_curve=101;
    ledger.prepared_=scan_planner::PassiveExecutionLedger::Job{};
    ledger.prepared_->version=first->version;ledger.prepared_->spline.trajectory.traj_id=102;
    ledger.local_prepare_request_=17;
    auto next=proposal(2);next->expected_version=first->version;next->expected_trajectory_id=101;
    check(!ledger.replacePendingReference(*next,true)&&ledger.prepared_&&ledger.local_prepare_request_,
      "rolling_reference_cannot_cross_inflight_prepared_commit");
  }
  {
    SCANReplanFSM f;f.typedPathCallback(path(1));
    check(phase(f,"reference_rejected_odometry"),"no_odometry_explicit_generation_bound_rejection");
    check(!f.have_target_ && f.exec_state_==f.WAIT_TARGET && f.accepted_calls==0,
          "rejected_generation_has_no_target_or_plan");
    fresh(f);check(f.accepted_calls==0 && !f.have_target_,"body_recovery_does_not_replay_reference");
    f.typedPathCallback(path(1));check(f.accepted_calls==0,"same_generation_remains_consumed");
    f.typedPathCallback(path(2));
    check(f.accepted_calls==1 && f.have_target_ && f.exec_state_==f.GEN_NEW_TRAJ,
          "new_owner_generation_with_fresh_body_can_be_accepted");
  }
  {
    SCANReplanFSM f;fresh(f);f.last_odom_time_.value-=.501;f.typedPathCallback(path(1));
    check(phase(f,"reference_rejected_odometry") && !f.have_target_,"stale_body_explicit_rejection");
  }
  {
    SCANReplanFSM f;fresh(f);auto m=path(1);m->path.header.frame_id="foreign";f.typedPathCallback(m);
    check(phase(f,"reference_rejected_frame") && !f.have_target_,"path_frame_terminal_rejection");
  }
  {
    SCANReplanFSM f;fresh(f);auto m=path(1);m->path.poses[1].header.frame_id="foreign";f.typedPathCallback(m);
    check(phase(f,"reference_rejected_frame") && !f.have_target_,"pose_frame_terminal_rejection");
  }
  {
    SCANReplanFSM f;fresh(f);auto m=path(1);
    m->path.poses[1].pose.position.x=std::numeric_limits<double>::quiet_NaN();f.typedPathCallback(m);
    check(phase(f,"reference_rejected_geometry") && !f.have_target_,"nan_geometry_terminal_rejection");
  }
  {
    SCANReplanFSM f;fresh(f);auto m=path(1);m->path.poses[0].pose.position.x=3.;f.typedPathCallback(m);
    check(phase(f,"reference_rejected_geometry") && !f.have_target_,"far_start_terminal_rejection");
  }
  {
    SCANReplanFSM f;fresh(f);auto m=path(1);m->path.poses[1].pose.position.x=0.;f.typedPathCallback(m);
    check(phase(f,"reference_rejected_geometry") && !f.have_target_,"collapsed_geometry_terminal_rejection");
  }
  {
    SCANReplanFSM f;fresh(f);f.typedPathCallback(path(1));auto cancel=path(2);cancel->path.poses.clear();
    f.actual_body_seed_request_=scan_planner::ew::ExecutionVersion{};
    f.typedPathCallback(cancel);fresh(f);f.typedPathCallback(path(1));
    check(phase(f,"cancelled") && !f.have_target_ && f.accepted_calls==1,
          "explicit_cancel_and_old_generation_never_resurrect");
    check(!f.actual_body_seed_request_,"reference_cancellation_clears_unsubmitted_actual_seed_intent");
    auto foreign=path(3);foreign->session_id="foreign";f.typedPathCallback(foreign);
    check(f.reference_generation_==2 && !f.have_target_,"foreign_session_cannot_take_generation");
  }
  {
    SCANReplanFSM f;fresh(f);f.require_schema_v2_=true;f.typedPathCallback(path(1));
    check(phase(f,"invalid_reference_schema") && !f.have_target_,"v2_session_rejects_legacy_reference");
    auto m=path(2);m->schema_version=2;m->point_reference="ground";m->task_id="task";m->route_id="route";
    m->route_hash="hash";m->map_version_id="map-v";m->segment_id="floor1";
    m->point_segment_ids={"floor1","stairs"};
    m->point_segment_kinds={"floor","stairs"};m->point_required_modes={"walk","climb"};
    f.typedPathCallback(m);
    check(f.have_target_ && f.reference_metadata_.point_segment_ids[1]=="stairs",
          "v2_keeps_multilevel_semantic_point_arrays");
    auto body=std::make_shared<Message>(*m);body->generation=3;body->point_reference="body_center";
    f.typedPathCallback(body);
    check(!f.have_target_ && phase(f,"invalid_reference_schema"),"body_center_rejects_second_height_offset");
    body->generation=4;f.reference_path_z_offset_=0.;f.typedPathCallback(body);
    check(f.have_target_,"body_center_admitted_only_with_zero_offset");
    m=path(5);m->schema_version=2;f.typedPathCallback(m);
    check(!f.have_target_ && f.cancellation_requests_==5,
          "invalid_replacement_still_cancels_worker_and_revokes_prior_route");
  }
  {
    SCANReplanFSM f;fresh(f);f.typedPathCallback(path(1));
    f.execution_validator_=std::make_shared<scan_planner::PassiveExecutionLedger>();
    f.last_visualized_committed_=std::make_pair(scan_planner::executionVersion(*path(1)),1);
    auto m=path(2);m->schema_version=2;m->path.poses.clear();
    f.execution_validator_->accepts=false;f.typedPathCallback(m);
    check(f.have_target_&&f.reference_generation_==1,"foreign_cancel_keeps_native_owner");
    check(f.visual.clears==0&&f.last_visualized_committed_.has_value(),
          "foreign_cancel_keeps_actual_committed_marker");
    f.execution_validator_->accepts=true;f.typedPathCallback(m);
    check(f.execution_validator_->retired&&phase(f,"cancelled")&&!f.have_target_,
          "execution_cancel_retires_ledger_before_fresh_cancel_ack");
    check(f.visual.clears==1&&!f.last_visualized_committed_,
          "exact_task_cancel_clears_only_stable_local_curve_marker_once");
  }
  {
    SCANReplanFSM f;fresh(f);f.reference_path_z_offset_=0.;f.require_schema_v2_=true;
    f.execution_validator_=std::make_shared<scan_planner::PassiveExecutionLedger>();
    f.proposalCallback(proposal(10));
    check(f.have_target_&&f.reference_generation_==10&&f.execution_validator_->replacements==1,
          "fresh_exact_proposal_replaces_pending_and_accepts_real_callback");
    auto incumbent=proposal(10);f.execution_validator_->pending=*incumbent;
    const auto cancellations=f.cancellation_requests_;
    f.proposalCallback(proposal(9));
    check(f.execution_validator_->pending&&f.execution_validator_->replacements==1&&
          f.execution_proposal_->proposal_id=="proposal-10"&&f.cancellation_requests_==cancellations&&
          f.reference_generation_==10&&f.have_target_,
          "late_generation_cannot_cancel_new_pending_or_replace_fsm_proposal");
    check(f.receipt_sink.receipts.size()==1,
          "uncached_old_revision_is_not_mislabeled_never_admitted");
    f.proposalCallback(proposal(10));
    check(f.execution_validator_->pending&&f.execution_validator_->replacements==1&&
          f.cancellation_requests_==cancellations&&f.receipt_sink.receipts.back().accepted,
          "exact_duplicate_replays_original_admission_without_replacement");
    auto foreign=proposal(11);foreign->reference.session_id="foreign";
    foreign->version=scan_planner::executionVersion(foreign->reference);f.proposalCallback(foreign);
    check(f.execution_validator_->pending&&f.execution_validator_->replacements==1&&
          f.cancellation_requests_==cancellations&&f.reference_generation_==10,
          "foreign_session_is_side_effect_free_before_ledger_replace");
    f.execution_validator_->replacement_allowed=false;f.proposalCallback(proposal(12));
    check(f.execution_validator_->pending&&f.execution_validator_->replacements==1&&
          f.execution_proposal_->proposal_id=="proposal-10"&&f.cancellation_requests_==cancellations&&
          f.reference_generation_==10&&!f.receipt_sink.receipts.back().accepted,
          "closed_task_or_changed_predecessor_rejection_preserves_new_pending");
  }
  {
    SCANReplanFSM f;fresh(f);f.reference_path_z_offset_=0.;f.require_schema_v2_=true;
    f.execution_validator_=std::make_shared<scan_planner::PassiveExecutionLedger>();
    auto original=proposal(10);f.proposalCallback(original);
    const auto accepted=f.receipt_sink.receipts.back();
    const auto cancelled=f.cancellation_requests_,replaced=f.execution_validator_->replacements;
    f.clock.clock=104.; // Original 400/500 ms leases are long expired.
    f.proposalCallback(original);
    const auto recovered=f.receipt_sink.receipts.back();
    check(recovered.accepted&&recovered.source_stamp==accepted.source_stamp&&
          recovered.valid_until==accepted.valid_until&&recovered.reason==accepted.reason&&
          f.cancellation_requests_==cancelled&&f.execution_validator_->replacements==replaced,
          "lost_positive_ack_after_four_seconds_returns_original_fact_not_new_lease");
    for(int mutation=0;mutation<8;++mutation) {
      auto conflict=std::make_shared<scan_planner::ew::ReferenceProposal>(*original);
      if(mutation==0)conflict->reference.path.poses[1].pose.position.x+=.01;
      if(mutation==1)conflict->goal_position.y+=.01;
      if(mutation==2)conflict->has_goal_yaw=true;
      if(mutation==3)conflict->expected_trajectory_id++;
      if(mutation==4)conflict->source_stamp.value+=.01;
      if(mutation==5)conflict->valid_until.value+=.01;
      if(mutation==6)conflict->reference.point_segment_kinds[0]="stairs";
      if(mutation==7)conflict->proposal_id="another-id-same-version";
      const auto count=f.receipt_sink.receipts.size();f.proposalCallback(conflict);
      check(f.receipt_sink.receipts.size()==count&&f.execution_validator_->replacements==replaced&&
            f.cancellation_requests_==cancelled,"identity_payload_conflict_has_no_ack_or_side_effect");
    }
    f.proposalCallback(original);check(f.receipt_sink.receipts.back().accepted,
      "conflict_does_not_destroy_original_admission_recovery");
    auto never=proposal(11);f.proposalCallback(never);
    check(!f.receipt_sink.receipts.back().accepted&&f.execution_validator_->replacements==replaced&&
          f.cancellation_requests_==cancelled,"expired_never_admitted_new_revision_has_real_negative");
    fresh(f);auto next=proposal(12);next->source_stamp={104.};next->valid_until={104.4};
    f.proposalCallback(next);check(f.have_target_&&f.reference_generation_==12&&
          f.execution_validator_->replacements==replaced+1,"fresh_replacement_after_negative_can_progress");
    const auto count=f.execution_validator_->replacements;f.proposalCallback(original);
    check(f.execution_validator_->replacements==count&&f.reference_generation_==12&&
          f.receipt_sink.receipts.back().source_stamp==accepted.source_stamp,
          "late_cached_fact_cannot_restore_old_generation");
  }
  // A reanchor is a candidate in the same task, not a destructive goal
  // replacement. Exercise the actual receive/typed/path callbacks and CAS:
  // the 10/23 stair session repeatedly sent a new anchor whose first route
  // point was outside Native's input correspondence bound.
  for(int defect=0;defect<5;++defect) {
    SCANReplanFSM f;fresh(f);f.reference_path_z_offset_=0.;f.require_schema_v2_=true;
    f.local_follower_=true;
    f.execution_validator_=std::make_shared<scan_planner::PassiveExecutionLedger>();
    auto incumbent=proposal(20);f.proposalCallback(incumbent);
    f.execution_validator_->committed=incumbent->version;
    f.execution_validator_->committed_curve=201;
    f.execution_validator_->pending=*incumbent;
    f.accepted_curve_id_=201;f.tracking_progress_=7;f.exec_state_=f.EXEC_TRAJ;
    f.actual_body_seed_request_=incumbent->version;
    const int cancels=f.cancellation_requests_,replaces=f.execution_validator_->replacements;
    auto invalid=proposal(21);invalid->reference.anchor_id="new-anchor";
    invalid->reference.anchor_revision=2;
    if(defect==0)invalid->reference.path.poses[0].pose.position.x=3.;
    if(defect==1)invalid->reference.path.poses[1].pose.position.x=std::numeric_limits<double>::quiet_NaN();
    if(defect==2)invalid->reference.path.poses[1].header.frame_id="foreign";
    if(defect==3)invalid->reference.point_segment_ids.pop_back();
    if(defect==4)f.last_odom_time_.value-=.501;
    invalid->version=scan_planner::executionVersion(invalid->reference);
    invalid->expected_version=incumbent->version;invalid->expected_trajectory_id=201;
    f.proposalCallback(invalid);
    check(!f.receipt_sink.receipts.back().accepted&&f.have_target_&&f.reference_generation_==20&&
      f.reference_metadata_==incumbent->reference&&f.execution_proposal_->version==incumbent->version&&
      f.execution_validator_->pending&&f.execution_validator_->pending->version==incumbent->version&&
      f.execution_validator_->replacements==replaces&&f.cancellation_requests_==cancels&&
      f.accepted_curve_id_==201&&f.tracking_progress_==7&&f.exec_state_==f.EXEC_TRAJ&&
      f.actual_body_seed_request_==incumbent->version&&!f.execution_validator_->retired,
      "invalid_reanchor_is_rejected_before_cas_without_mutating_active_reference_or_task");
    fresh(f);auto retry=proposal(22);retry->reference.anchor_id="new-anchor";
    retry->reference.anchor_revision=2;retry->version=scan_planner::executionVersion(retry->reference);
    retry->expected_version=incumbent->version;retry->expected_trajectory_id=201;
    f.proposalCallback(retry);
    check(f.receipt_sink.receipts.back().accepted&&f.have_target_&&f.reference_generation_==22&&
      f.execution_validator_->replacements==replaces+1&&f.cancellation_requests_==cancels+1,
      "fresh_valid_reanchor_after_rejection_preserves_exact_predecessor_and_can_progress");
  }
  {
    SCANReplanFSM f;fresh(f);f.reference_path_z_offset_=0.;f.require_schema_v2_=true;
    f.execution_validator_=std::make_shared<scan_planner::PassiveExecutionLedger>();
    f.proposalCallback(proposal(25));
    f.last_odom_time_={99.5003};f.clock.increment=.0002;
    auto input=proposal(26);input->reference.anchor_id="new-anchor";
    input->version=scan_planner::executionVersion(input->reference);
    f.proposalCallback(input);
    check(f.receipt_sink.receipts.back().accepted&&f.have_target_&&f.reference_generation_==26&&
      f.clock.clock-f.last_odom_time_.value>f.odom_timeout_&&f.accepted_calls==2,
      "preflight_age_boundary_uses_prepared_geometry_after_cas_without_second_admission");
  }
  {
    // A legitimate sender window starts at its route projection. The two
    // bounds are independent XY/Z correspondence, not a 1 m 3-D sphere.
    SCANReplanFSM f;fresh(f);f.reference_path_z_offset_=0.;f.require_schema_v2_=true;
    f.execution_validator_=std::make_shared<scan_planner::PassiveExecutionLedger>();
    f.odom_pos_={0.,1.39,.84};
    auto input=proposal(30);
    for(auto& pose:input->reference.path.poses)pose.pose.position.z=.55;
    input->reference.path.poses.back().pose.position.x=3.;
    input->version=scan_planner::executionVersion(input->reference);
    check(!f.prepareRollingReference(input->reference),"legacy_one_metre_sphere_rejects_sender_valid_detour");
    f.reference_start_tolerance_=std::hypot(1.4,.3);
    const auto prepared=f.prepareRollingReference(input->reference);
    check(prepared.has_value(),"shared_xy_z_correspondence_accepts_sender_valid_detour");
    if(prepared) {
      scan_planner::DiscreteReference route;
      const double begin=scan_planner::setReferenceFromBody(route,f.odom_pos_,*prepared);
      // This is selector geometry only, deliberately not a map feasibility
      // claim. The production map/optimizer still checks the actual connector.
      const auto target=scan_planner::selectReferenceTarget(route,f.odom_pos_,begin,2.,{},
          [](const Eigen::Vector3d&,double){return 0;});
      check(begin>1.&&target.valid&&target.point.x()>1.&&std::abs(target.point.y())<1e-9,
        "detour_input_uses_real_reference_target_selector_and_never_targets_body_connector");
    }
    f.proposalCallback(input);
    check(f.have_target_&&f.receipt_sink.receipts.back().accepted,
      "sender_valid_detour_enters_real_proposal_and_path_callbacks");
    auto distant=proposal(31);distant->reference.path.poses.front().pose.position.x=4.;
    distant->version=scan_planner::executionVersion(distant->reference);
    const auto cancellations=f.cancellation_requests_;
    f.proposalCallback(distant);
    check(!f.receipt_sink.receipts.back().accepted&&f.have_target_&&f.reference_generation_==30&&
      f.cancellation_requests_==cancellations,"true_remote_reference_stays_rejected_without_clearing_incumbent");
  }
  {
    using History=scan_planner::ReferenceReceiptHistory<scan_planner::ew::ReferenceProposal,
      scan_planner::ew::ReferenceReceipt>;
    History history;const auto active=proposal(1),pending=proposal(2);
    scan_planner::ew::ReferenceReceipt original;original.accepted=true;original.source_stamp={100.};
    check(history.remember(*active,original,{},{}),"bounded_cache_records_active_fact");
    check(history.remember(*pending,original,{pending->version},{active->version}),"bounded_cache_records_pending_fact");
    for(std::uint64_t generation=3;generation<30;++generation) {
      auto p=proposal(generation);history.remember(*p,original,{pending->version},{active->version});
    }
    check(history.size()==History::capacity&&history.lookup(*active).match==History::Match::Exact&&
          history.lookup(*pending).match==History::Match::Exact,"active_and_pending_facts_are_pinned_under_bounded_churn");
    check(history.lookup(*proposal(3)).match==History::Match::Missing,
          "old_unpinned_fact_can_evict_but_is_not_a_negative");
  }
  {
    SCANReplanFSM f;f.local_follower_=true;f.reference_path_z_offset_=0.;fresh(f);
    f.execution_validator_=std::make_shared<scan_planner::PassiveExecutionLedger>();
    auto first=rolling(1,0.,f.clock.clock);f.proposalCallback(first);
    f.actual_body_seed_request_=first->version;
    f.execution_validator_->committed=first->version;f.execution_validator_->committed_curve=101;
    f.accepted_curve_id_=101;f.tracking_progress_=7;f.exec_state_=f.EXEC_TRAJ;
    const auto initial_cancels=f.cancellation_requests_;
    for(int i=1;i<=5;++i) {
      f.clock.clock+=.1;fresh(f);f.odom_pos_.x()=.25*i;
      const auto generation=f.reference_generation_+1;
      auto next=rolling(generation,.25*i,f.clock.clock);
      next->expected_version=*f.execution_validator_->committed;next->expected_trajectory_id=101;
      f.pending_splice_=true;const auto replacements=f.execution_validator_->replacements;
      const auto receipts=f.receipt_sink.receipts.size();f.proposalCallback(next);
      check(f.deferred_rolling_proposal_&&f.reference_generation_==generation-1&&
        f.execution_validator_->replacements==replacements&&f.receipt_sink.receipts.size()==receipts&&
        f.accepted_curve_id_==101&&f.tracking_progress_==7,"rolling_window_does_not_cancel_reserved_old_splice");
      f.proposalCallback(next);
      check(f.receipt_sink.receipts.size()==receipts&&
        f.deferred_rolling_proposal_->source_stamp==next->source_stamp,
        "queued_duplicate_has_one_original_deadline_and_no_fake_ack");
      f.pending_splice_=false;f.execution_validator_->replacement_allowed=false;
      f.clock.clock+=.1;fresh(f);f.processDeferredRollingProposal();
      check(!f.deferred_rolling_proposal_&&!f.receipt_sink.receipts.back().accepted&&
        f.receipt_sink.receipts.back().reason=="rolling_reference_predecessor_changed_retry"&&
        f.rolling_retry_priority_until_>std::chrono::steady_clock::now(),
        "committed_predecessor_change_has_explicit_retry_priority_not_old_window_starvation");
      f.clock.clock+=.5;fresh(f);f.execution_validator_->replacement_allowed=true;
      auto retry=rolling(generation+1,.25*i,f.clock.clock);
      retry->expected_version=*f.execution_validator_->committed;retry->expected_trajectory_id=101;
      f.proposalCallback(retry);
      check(f.receipt_sink.receipts.back().accepted&&f.reference_generation_==generation+1&&
        f.cancellation_requests_==initial_cancels&&f.accepted_curve_id_==101&&f.tracking_progress_==7&&
        f.exec_state_==f.REPLAN_TRAJ&&f.rolling_retry_priority_until_==std::chrono::steady_clock::time_point{},
        "five_rolling_retries_install_without_resetting_committed_curve_or_progress");
      check(f.actual_body_seed_request_.has_value(),
        "same_task_rolling_reference_preserves_unsubmitted_actual_body_reseed");
      f.execution_validator_->committed=retry->version;
    }
    auto next=rolling(f.reference_generation_+1,1.5,f.clock.clock);
    next->expected_version=*f.execution_validator_->committed;next->expected_trajectory_id=101;
    f.pending_splice_=true;f.proposalCallback(next);f.clock.clock+=.401;fresh(f);
    f.processDeferredRollingProposal();
    check(!f.deferred_rolling_proposal_&&!f.receipt_sink.receipts.back().accepted&&
      f.receipt_sink.receipts.back().reason=="rolling_reference_original_deadline_elapsed",
      "busy_splice_cannot_renew_original_proposal_400ms_deadline");
    f.pending_splice_=false;f.proposalCallback(next);
    check(!f.receipt_sink.receipts.back().accepted&&f.reference_generation_==11,
      "late_expired_request_replays_negative_and_never_revives");
  }
  {
    SCANReplanFSM f;f.local_follower_=true;f.reference_path_z_offset_=0.;fresh(f);
    f.execution_validator_=std::make_shared<scan_planner::PassiveExecutionLedger>();
    auto first=rolling(1,0.,f.clock.clock);f.proposalCallback(first);
    f.execution_validator_->pending=*first;
    const auto replacements=f.execution_validator_->replacements;
    f.clock.clock+=.6; // New proposal is fresh; old body is not.
    auto next=rolling(2,.25,f.clock.clock);f.proposalCallback(next);
    check(!f.receipt_sink.receipts.back().accepted&&f.reference_generation_==1&&
      f.execution_proposal_->version==first->version&&f.execution_validator_->pending.has_value()&&
      f.execution_validator_->replacements==replacements,
      "soft_input_rejection_preserves_old_proposal_candidate_and_geometry_atomically");
    // This is the actual publishTrajectory identity prerequisite, not a new
    // admission: the still-running old solve can reach Native with gen1.
    check(f.execution_proposal_->reference.generation==f.reference_generation_&&
      f.execution_proposal_->version==scan_planner::executionVersion(f.reference_metadata_),
      "old_solve_publication_owner_remains_valid_after_soft_rejection");
    fresh(f);f.pending_splice_=true;
    next=rolling(3,.25,f.clock.clock);f.proposalCallback(next);
    auto cancel=std::make_shared<Message>(first->reference);cancel->generation=4;cancel->path.poses.clear();
    f.typedPathCallback(cancel);f.pending_splice_=false;f.processDeferredRollingProposal();
    check(!f.deferred_rolling_proposal_&&f.reference_generation_==4&&!f.have_target_&&
      f.execution_validator_->retired&&f.rolling_retry_priority_until_==std::chrono::steady_clock::time_point{},
      "cancellation_retires_deferred_window_immediately_without_reactivation");
  }
  {
    SCANReplanFSM f;f.local_follower_=true;f.reference_path_z_offset_=0.;fresh(f);
    f.execution_validator_=std::make_shared<scan_planner::PassiveExecutionLedger>();
    auto first=rolling(1,0.,f.clock.clock);f.proposalCallback(first);
    auto& ledger=*f.execution_validator_;
    ledger.committed=first->version;f.accepted_curve_id_=1;f.tracking_progress_=9;
    f.exec_state_=f.EXEC_TRAJ;const auto cancels=f.cancellation_requests_;
    std::uint64_t generation=1;
    // A reference is INPUT geometry, not a request to commit its expected
    // curve. Native may naturally advance that exact reference while this
    // rolling window is deferred. Keep the immutable original expectation;
    // later trajectory commit still uses the production exact-current CAS.
    for(int window=1;window<=5;++window) {
      f.clock.clock+=.5;fresh(f);f.odom_pos_.x()=.25*window;
      auto queued=rolling(++generation,.25*window,f.clock.clock);
      queued->expected_version=*ledger.committed;queued->expected_trajectory_id=ledger.committed_curve;
      f.pending_splice_=true;f.exec_state_=f.EXEC_TRAJ;f.proposalCallback(queued);
      f.ordinaryReplanTick();
      check(f.deferred_rolling_proposal_&&f.ordinary_solve_starts==0,
        "owner_recovered_pending_splice_keeps_literal_window_and_suppresses_old_work");
      ++ledger.committed_curve;f.accepted_curve_id_=ledger.committed_curve;
      f.pending_splice_=false;f.clock.clock+=.1;fresh(f);f.processDeferredRollingProposal();
      f.ordinaryReplanTick();
      check(f.receipt_sink.receipts.back().accepted&&!f.deferred_rolling_proposal_&&
        f.reference_generation_==generation&&f.execution_proposal_->expected_trajectory_id==ledger.committed_curve-1&&
        f.exec_state_==f.REPLAN_TRAJ&&f.tracking_progress_==9&&
        f.cancellation_requests_==cancels&&f.rolling_retry_priority_until_==std::chrono::steady_clock::time_point{},
        "natural_incumbent_curve_advance_admits_five_rolling_windows_without_retry_or_active_reset");
      ledger.committed=queued->version;++ledger.committed_curve;f.accepted_curve_id_=ledger.committed_curve;
    }
    check(ledger.rolling_replacements==5&&f.owner_dispatches_==6,
      "real_rolling_admission_is_classified_and_owner_dispatch_preserves_literal_requests");
    f.clock.clock+=.5;fresh(f);f.odom_pos_.x()=1.5;
    auto future=rolling(++generation,1.5,f.clock.clock);
    future->expected_version=*ledger.committed;future->expected_trajectory_id=ledger.committed_curve+1;
    f.proposalCallback(future);
    check(!f.receipt_sink.receipts.back().accepted&&f.reference_generation_==generation-1&&
      ledger.rolling_replacements==5&&f.cancellation_requests_==cancels,
      "future_curve_predecessor_still_rejects_without_cancelling_incumbent");
    f.exec_state_=f.EXEC_TRAJ;
    f.rolling_retry_priority_until_=std::chrono::steady_clock::now()-std::chrono::milliseconds(1);
    f.ordinaryReplanTick();
    check(f.ordinary_solve_starts==1,"retry_priority_is_finite_not_a_permanent_planning_hold");
  }
  {
    SCANReplanFSM f;f.local_follower_=true;f.reference_path_z_offset_=0.;fresh(f);
    f.execution_validator_=std::make_shared<scan_planner::PassiveExecutionLedger>();
    auto first=rolling(1,0.,f.clock.clock);f.proposalCallback(first);
    f.execution_validator_->committed=first->version;
    const auto dispatched=f.owner_dispatches_;f.owner_map_available=false;
    auto cancel=std::make_shared<Message>(first->reference);cancel->generation=2;cancel->path.poses.clear();
    f.typedIngress(cancel);
    check(f.execution_validator_->retired&&!f.have_target_&&f.reference_generation_==2&&
      f.owner_dispatches_==dispatched,"formal_cancel_retires_task_without_owner_map_or_geometry_dispatch");
    auto replacement=std::make_shared<Message>(first->reference);replacement->generation=3;
    f.typedIngress(replacement);
    check(f.reference_generation_==2&&!f.have_target_&&f.owner_dispatches_==dispatched+1,
      "nonempty_replacement_still_waits_for_map_and_cannot_resurrect_cancelled_task");
  }
  std::cout<<"checks="<<checks<<" failures="<<failures<<"\n";
  return failures?1:0;
}
'''


def callback(source, name, next_name):
    start = source.index('  void SCANReplanFSM::' + name + '(')
    end = source.index('  void SCANReplanFSM::' + next_name + '(', start)
    return source[start:end]


def proposal_callback(source):
    marker = '[this,mode](ew::ReferenceProposal::ConstSharedPtr m){'
    start = source.index(marker) + len(marker)
    end = source.index('          });', start)
    result = ('  void SCANReplanFSM::proposalCallback(ew::ReferenceProposal::ConstSharedPtr m) {\n'
              '    const std::string mode="isolated_mock";\n' + source[start:end] + '\n  }\n')
    if 'void SCANReplanFSM::receiveReferenceProposal(' in source:
        begin = source.index('  std::optional<std::vector<Eigen::Vector3d>> SCANReplanFSM::prepareRollingReference(')
        finish = source.index('  void SCANReplanFSM::typedPathCallback(', begin)
        result += source[begin:finish]
    else:
        # The frozen baseline had no deferred-ingress event to process. This
        # no-op represents absence of that event, not a substitute planner.
        result += 'void SCANReplanFSM::processDeferredRollingProposal() {}\n'
    return result


def reference_admission(include):
    """Run the real locked ingress predicate, not a permissive fake CAS."""
    text=(include/'plan_manage/execution_validator.hpp').read_text()
    start=text.index('  bool replacePendingReference(')
    brace=text.index('{',start)
    depth=1
    end=brace+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}')
        end+=1
    return text[start:end].replace('replacePendingReference(', 'productionReplacePendingReference(',1)


def pending_local_demand(include):
    """Use the actual production pending-slot declaration, not an empty stub."""
    text = (include / 'plan_manage/execution_validator.hpp').read_text()
    match = re.search(r'struct PendingLocalDemand \{ew::MotionDemand demand;'
                      r'std::uint64_t stop_barrier;bool waiting_for_commit;\};\s*'
                      r'std::optional<PendingLocalDemand> pending_local_demand_;', text)
    assert match, 'production pending command contract changed; update behavioral fixture explicitly'
    return match.group(0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[1] / 'src/scan_replan_fsm.cpp')
    parser.add_argument('--include', type=Path, default=Path(__file__).resolve().parents[1] / 'include')
    parser.add_argument('--build-dir', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    source = args.source.read_text()
    extracted = callback(source, 'typedPathCallback', 'publishTrajectory')
    extracted += callback(source, 'pathCallback', 'odometryCallback')
    extracted += proposal_callback(source)
    ingress_start=source.index('[this](ew::ReferencePath::ConstSharedPtr m){',
                              source.index('typed_path_sub_'))+len('[this](ew::ReferencePath::ConstSharedPtr m){')
    ingress_end=source.index('            });',ingress_start)
    extracted+='\nvoid SCANReplanFSM::typedIngress(ew::ReferencePath::ConstSharedPtr m){'+source[ingress_start:ingress_end]+'}\n'
    start=source.index('    case EXEC_TRAJ:',source.index('  void SCANReplanFSM::execFSMCallback()'))
    start=source.index('{',start)+1
    end=source.index('      // Both ordinary triggers',start)
    extracted += ('void SCANReplanFSM::ordinaryReplanTick(){switch(exec_state_){case EXEC_TRAJ:{\n'
        +source[start:end]+'++ordinary_solve_starts;break;}default:break;}}\n')
    with tempfile.TemporaryDirectory(prefix='reference-callback-') as temporary:
        root = args.build_dir or Path(temporary)
        root.mkdir(parents=True, exist_ok=True)
        cpp, binary = root / 'reference_callback.cpp', root / 'reference_callback'
        cpp.write_text(HARNESS.replace('__CALLBACKS__', extracted)
                       .replace('__REFERENCE_ADMISSION__',reference_admission(args.include))
                       .replace('__PENDING_LOCAL_DEMAND__',pending_local_demand(args.include)))
        command = ['c++', '-std=c++17', '-O0', '-Wall', '-Wextra',
                   '-I' + str(args.include), '-I/usr/include/eigen3',
                   '-I' + str(args.include.parent.parent / 'bspline_opt/include'),
                   str(cpp), '-o', str(binary)]
        subprocess.run(command, check=True)
        print('Production callback source:', args.source, flush=True)
        result = subprocess.run([str(binary)], check=False, capture_output=True, text=True)
        print(result.stdout, end='')
        print(result.stderr, end='')
        if args.report:
            report = dict(schema=1, source=str(args.source),
                          source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                          production_helpers=str(args.include / 'plan_manage/input_contract.hpp'),
                          helpers_sha256=hashlib.sha256((args.include / 'plan_manage/input_contract.hpp').read_bytes()).hexdigest(),
                          production_reference_admission=str(args.include/'plan_manage/execution_validator.hpp'),
                          reference_admission_sha256=hashlib.sha256(reference_admission(args.include).encode()).hexdigest(),
                          callback_source=str(cpp), compile_command=command,
                          returncode=result.returncode, stdout=result.stdout, stderr=result.stderr,
                          ros_initialized=False, robot_connected=False,
                          scope='actual extracted reference callbacks and locked reference admission body; clock, ledger storage, transport, publication and planning sink doubled; no map or optimizer validation')
            args.report.write_text(json.dumps(report, indent=2) + '\n')
        return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
