#pragma once

#include <d1max_planning_interfaces/msg/execution_permit.hpp>
#include <d1max_planning_interfaces/msg/trajectory_validation.hpp>
#include <d1max_planning_interfaces/msg/trajectory_admission.hpp>
#include <d1max_planning_interfaces/msg/sdk_execution_state.hpp>
#include <d1max_planning_interfaces/msg/stop_report.hpp>
#include <d1max_planning_interfaces/msg/reference_receipt.hpp>
#include <d1max_planning_interfaces/msg/reference_proposal.hpp>
#include <d1max_planning_interfaces/msg/motion_demand.hpp>
#include <d1max_planning_interfaces/msg/motion_validation.hpp>
#include <d1max_planning_interfaces/msg/navigation_state.hpp>
#include <d1max_planning_interfaces/msg/local_navigation_state.hpp>
#include <d1max_planning_interfaces/msg/execution_handoff_grant.hpp>
#include <d1max_planning_interfaces/msg/execution_commit_ack.hpp>
#include <d1max_planning_interfaces/msg/sdk_stationary_evidence.hpp>
#include <d1max_planning_interfaces/msg/tracker_geometry_receipt.hpp>
#include "d1max_navigation_bt/stable_recovery.hpp"
#include "d1max_navigation_bt/route_progress_supervisor.hpp"
#include <optional>
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <stdexcept>
#include <string>
#include <vector>
#include <type_traits>

namespace d1max_navigation_bt {
namespace exec3 {
using Permit=d1max_planning_interfaces::msg::ExecutionPermit;
using Validation=d1max_planning_interfaces::msg::TrajectoryValidation;
using Admission=d1max_planning_interfaces::msg::TrajectoryAdmission;
using SDKState=d1max_planning_interfaces::msg::SDKExecutionState;
using Stop=d1max_planning_interfaces::msg::StopReport;
using Receipt=d1max_planning_interfaces::msg::ReferenceReceipt;
using MotionDemand=d1max_planning_interfaces::msg::MotionDemand;
using MotionValidation=d1max_planning_interfaces::msg::MotionValidation;
using NavigationState=d1max_planning_interfaces::msg::NavigationState;
using Handoff=d1max_planning_interfaces::msg::ExecutionHandoffGrant;
using CommitAck=d1max_planning_interfaces::msg::ExecutionCommitAck;
using Stationary=d1max_planning_interfaces::msg::SDKStationaryEvidence;
using GeometryReceipt=d1max_planning_interfaces::msg::TrackerGeometryReceipt;
using RouteProgress=d1max_planning_interfaces::msg::RouteProgress;

// Pending proposals are bounded, but cannot evict the immutable final goal of
// the committed anchor. This is task-owned geometry, not a renewable sensor lease.
class GoalLedger {
public:
  using Proposal=d1max_planning_interfaces::msg::ReferenceProposal;
  bool observe(const Proposal&p) {
    if(!complete(p.version)||p.reference.path.header.frame_id.empty()||
       !std::isfinite(p.goal_position.x)||!std::isfinite(p.goal_position.y)||!std::isfinite(p.goal_position.z)||
       (p.has_goal_yaw&&(!std::isfinite(p.goal_yaw)||!std::isfinite(p.goal_yaw_tolerance_rad)||p.goal_yaw_tolerance_rad<=0)))return false;
    if(active_&&sameVersion(active_->version,p.version))return sameGoal(*active_,p);
    for(auto&old:pending_)if(sameVersion(old.version,p.version))return sameGoal(old,p);
    if(pending_.size()==8)pending_.erase(pending_.begin());pending_.push_back(p);return true;
  }
  const Proposal* committed(const Version&v) {
    if(active_&&sameVersion(active_->version,v))return &*active_;
    for(const auto&p:pending_)if(sameVersion(p.version,v)){active_=p;return &*active_;}
    return nullptr;
  }
  const Proposal* find(const Version&v)const {
    if(active_&&sameVersion(active_->version,v))return &*active_;
    for(const auto&p:pending_)if(sameVersion(p.version,v))return &p;
    return nullptr;
  }
private:
  static bool sameGoal(const Proposal&a,const Proposal&b) {
    return a.goal_position==b.goal_position&&a.reference.path.header.frame_id==b.reference.path.header.frame_id&&
      a.has_goal_yaw==b.has_goal_yaw&&(!a.has_goal_yaw||(a.goal_yaw==b.goal_yaw&&a.goal_yaw_tolerance_rad==b.goal_yaw_tolerance_rad));
  }
  std::optional<Proposal>active_;std::vector<Proposal>pending_;
};

// A single ROS publisher orders all tasks in this navigation session. A retired
// old task may emit a revoke while its physical stop drains, but must not keep
// replacing the current task's stream after that stop is confirmed.
class PermitPublication {
public:
  bool prepare(Permit& p,const std::string& current_task,bool confirmed,bool stopped) {
    if(p.version.task_id!=current_task&&(!confirmed||stopped||!p.revoked))return false;
    if(sequence_==std::numeric_limits<uint64_t>::max())throw std::overflow_error("permit_sequence_exhausted");
    p.sequence=++sequence_;return true;
  }
private:
  uint64_t sequence_=0;
};

// The BT's subordinate execution state, not a second goal/task owner. All
// effects (SDK grant request, commit lease, stop) are emitted by Navigator.
struct StationaryLimits {
  double linear=.03,angular=.05,stop_duration=1.,reentry_duration=.6;
  unsigned minimum_samples=3;
  void validate()const {
    if(!std::isfinite(linear)||linear<=0.||linear>.05||!std::isfinite(angular)||angular<=0.||angular>.1||
       !std::isfinite(stop_duration)||stop_duration<1.||stop_duration>5.||
       !std::isfinite(reentry_duration)||reentry_duration<.6||reentry_duration>5.||
       minimum_samples<3||minimum_samples>1000)
      throw std::invalid_argument("bounded_record_stationary_limits_required");
  }
};
class Owner {
  struct InitialGeometry {Permit permit;Validation proof;Admission admission;};
  struct AppliedInstallation {Permit permit;CommitAck ack;std::optional<GeometryReceipt> fact;};
public:
  explicit Owner(std::string transport="live",double blocked_timeout=30.,double recovery_time=1.,StationaryLimits stationary={})
    : transport_(std::move(transport)),stationary_limits_(stationary),blocked_timeout_(blocked_timeout),recovery_time_(recovery_time) {
    stationary_limits_.validate();
    if(!std::isfinite(blocked_timeout_)||blocked_timeout_<1.||blocked_timeout_>300.||
       !std::isfinite(recovery_time_)||recovery_time_<.1||recovery_time_>=blocked_timeout_)
      throw std::invalid_argument("bounded_execution_blocked_and_recovery_times_required");
  }
  Owner(const Owner&)=delete; Owner& operator=(const Owner&)=delete;
  void bind(const Version& task) { if(!confirmed_) binding_=task; }
  bool bindRoute(const d1max_navigation_bt_interfaces::msg::RouteSnapshot&route) {
    return !confirmed_&&route_progress_.bind(route,binding_,transport_);
  }
  bool observe(const RouteProgress&p,SourceClock now){return route_progress_.observe(p,now);}
  template<class State>void observeBody(const State&s){route_progress_.observeBody(s);}
  bool observe(const Validation& v,SourceClock now) {
    if(!sameTask(binding_,v.version)||!complete(v.version)||v.transport_mode!=transport_||
       !fresh(v.source_stamp,now,.35)||!fresh(v.check_end,now,.35)||
       (v.valid&&(compare(v.valid_until,now)<0||elapsed(v.valid_until,v.check_end)>.5))||
       nanoseconds(v.check_begin)>nanoseconds(v.check_end)||
       !fresh(v.body_source_stamp,now,.4)||
       !fresh(v.front_ray_source_stamp,now,.6)||!fresh(v.rear_ray_source_stamp,now,.6)||
       v.sequence==0||v.trajectory_id<0) return false;
    for(const auto& p:proofs_) if(sameVersion(p.version,v.version)&&p.trajectory_id==v.trajectory_id&&v.sequence<=p.sequence)return false;
    if(proofs_.size()==32)proofs_.erase(proofs_.begin());proofs_.push_back(v);return true;
  }
  bool observe(const Admission& a,SourceClock now) {
    if(!sameTask(binding_,a.version)||!complete(a.version)||a.transport_mode!=transport_||
       !fresh(a.checked_at,now,.35)||!fresh(a.body_source_stamp,now,.4)||
       compare(a.valid_until,now)<0||elapsed(a.valid_until,a.checked_at)>.5)return false;
    for(auto& p:admissions_)if(sameVersion(p.version,a.version)&&p.trajectory_id==a.trajectory_id) {
      if(a.validation_sequence<p.validation_sequence||nanoseconds(a.checked_at)<=nanoseconds(p.checked_at))return false;
      p=a;return true;
    }
    if(admissions_.size()==8)admissions_.erase(admissions_.begin());admissions_.push_back(a);return true;
  }
  bool canConfirm(SourceClock now) const {
    // A desired preview is not an installation fact. Only a fresh exact
    // preparation or the actual tracker's source-bound installation receipt
    // can qualify the first geometry; neither is an SDK application receipt.
    return !confirmed_&&(candidate(now)||installedInitialProof(now));
  }
  std::int64_t preparationEvidence(SourceClock now)const {
    if(confirmed_||!active_)return 0;
    const auto* proof=installedInitialProof(now);
    if(!proof)proof=candidate(now);
    if(!proof)return 0;
    return static_cast<std::int64_t>(proof->body_source_stamp.sec)*1000000000LL+proof->body_source_stamp.nanosec;
  }
  bool observe(const Receipt&r,SourceClock now) {
    if(!sameTask(binding_,r.version)||r.transport_mode!=transport_||r.proposal_id.empty()||
       !fresh(r.source_stamp,now,.5)||compare(r.valid_until,now)<0)return false;
    for(auto&old:receipts_)if(old.proposal_id==r.proposal_id){
      if(nanoseconds(r.source_stamp)<=nanoseconds(old.source_stamp))return false;old=r;return true;
    }
    if(receipts_.size()==8)receipts_.erase(receipts_.begin());receipts_.push_back(r);return true;
  }
  bool observe(const GeometryReceipt& r,SourceClock now) {
    if(r.schema_version!=1||!sameTask(binding_,r.version)||!complete(r.version)||
       r.transport_mode!=transport_||r.frame_id.empty()||r.sequence<=geometry_receipt_sequence_||
       r.installation_sequence==0||!fresh(r.body_source_stamp,now,.4))return false;
    // Later CAS installations have a different original entry receipt than
    // the initial ordinary intent. Bind their software fact to the actual
    // writer ACK, never invent an Admission or apply the geometry from here.
    if(applied_installation_&&active_&&applied_commit_sequence_&&
       sameVersion(r.version,active_->version)&&r.trajectory_id==active_->trajectory_id&&
       sameVersion(r.version,applied_installation_->permit.version)&&
       r.trajectory_id==applied_installation_->permit.trajectory_id) {
      auto&installed=*applied_installation_;const auto&p=installed.permit;
      if(r.frame_id!=p.frame_id||r.permit_sequence!=p.sequence||
         r.admission_sequence!=installed.ack.entry_admission_sequence||
         r.validation_sequence<p.validation_sequence||compare(r.installed_at,now)>0||
         nanoseconds(r.installed_at)<nanoseconds(installed.ack.applied_at)||
         (r.installed&&seconds(r.body_source_stamp)<=geometry_body_source_))return false;
      if(installed.fact) {
        const auto&f=*installed.fact;
        if(r.installation_sequence!=f.installation_sequence||r.installed_at!=f.installed_at||
           r.validation_sequence!=f.validation_sequence)return false;
      } else {
        const Validation* original=nullptr;
        for(const auto&v:proofs_)if(sameVersion(v.version,r.version)&&v.trajectory_id==r.trajectory_id&&
            v.sequence==r.validation_sequence)original=&v;
        // This is the original prepared-entry proof identity, not proof of
        // permission at a late installation or callback. The SDK's actual
        // call independently bound a fresh exact motion/geometry proof; it can
        // use a newer revision. Keeping this fact NEVER renews the old floor.
        if(!original||!original->valid||!original->whole_curve||
           nanoseconds(original->check_end)>nanoseconds(installed.ack.applied_at)||
           (installed_receipt_&&r.installation_sequence<=installed_receipt_->installation_sequence))return false;
      }
      installed.fact=r;geometry_receipt_sequence_=r.sequence;
      geometry_body_source_=std::max(geometry_body_source_,seconds(r.body_source_stamp));
      observeTrackerMotionFence(r,now);return true;
    }
    const InitialGeometry* installed=nullptr;
    const auto matches=[&](const InitialGeometry& g) {
      return sameVersion(g.permit.version,r.version)&&g.permit.trajectory_id==r.trajectory_id&&
        g.permit.sequence==r.permit_sequence&&g.permit.frame_id==r.frame_id&&
        g.proof.sequence==r.validation_sequence&&g.admission.sequence==r.admission_sequence&&
        g.admission.validation_sequence==r.validation_sequence;
    };
    if(installed_geometry_&&matches(*installed_geometry_))installed=&*installed_geometry_;
    if(!installed)for(const auto&g:initial_geometries_)if(matches(g))installed=&g;
    if(!installed)return false;
    const double at=seconds(r.installed_at);
    if(at<=0||at>now||at<seconds(installed->permit.source_stamp)||
       at>seconds(installed->permit.valid_until)||at>seconds(installed->proof.valid_until)||
       at>seconds(installed->admission.valid_until)||!installed->admission.accepted)return false;
    if(installed_receipt_) {
      if(r.installation_sequence<installed_receipt_->installation_sequence||
         (r.installed&&seconds(r.body_source_stamp)<=geometry_body_source_))return false;
      if(r.installation_sequence==installed_receipt_->installation_sequence&&
         (r.permit_sequence!=installed_receipt_->permit_sequence||
          r.admission_sequence!=installed_receipt_->admission_sequence||
          r.validation_sequence!=installed_receipt_->validation_sequence||
          r.installed_at!=installed_receipt_->installed_at||
          !sameVersion(r.version,installed_receipt_->version)||
          r.trajectory_id!=installed_receipt_->trajectory_id))return false;
    }
    // Once the SDK writer chose an identity, a different software installation
    // cannot move that actual identity back or replace its handoff contract.
    if(applied_commit_sequence_&&active_&&(!sameVersion(r.version,active_->version)||
       r.trajectory_id!=active_->trajectory_id))return false;
    geometry_receipt_sequence_=r.sequence;
    geometry_body_source_=std::max(geometry_body_source_,seconds(r.body_source_stamp));
    installed_geometry_=*installed;installed_receipt_=r;
    observeTrackerMotionFence(r,now);
    return true;
  }
  bool begin(std::string id,std::string confirmation,uint64_t epoch,SourceClock now) {
    if(confirmed_||id.empty()||confirmation.empty()||epoch==0||!canConfirm(now))return false;
    execution_=std::move(id);confirmation_=std::move(confirmation);epoch_=epoch;
    confirmed_=true;phase_="arming";started_=now;last_now_=now;return true;
  }
  bool observe(const SDKState& s,SourceClock now) {
    if(!confirmed_||s.execution_id!=execution_||s.control_epoch!=epoch_||!sameTask(binding_,s.version)||
       s.transport_mode!=transport_||!fresh(s.source_stamp,now,.5)||s.sdk_session.empty()||
       s.sequence<=sdk_sequence_)return false;
    if(!sdk_session_.empty()&&sdk_session_!=s.sdk_session){stop("sdk_session_changed",now);return false;}
    sdk_session_=s.sdk_session;sdk_sequence_=s.sequence;sdk_stamp_=SourceClock::fromNanoseconds(nanoseconds(s.source_stamp));
    sdk_applied_commit_sequence_=std::max(sdk_applied_commit_sequence_,s.writer_commit_sequence);
    if(sdk_generation_&&s.sdk_arm_generation!=sdk_generation_)stop("sdk_arm_generation_changed",now);
    if(s.sdk_arm_generation)sdk_generation_=s.sdk_arm_generation;
    const bool mc_hold=s.phase=="holding"&&s.reason=="mc_stale"&&s.control_owned&&
      s.general_low_speed_confirmed&&!s.fault_latched;
    if(s.fault_latched||((!s.control_owned||(!s.grant_ready&&!mc_hold))&&phase_!="arming")||
       s.phase=="stopping"||s.phase=="stopped")stop("sdk_grant_withdrawn:"+s.reason,now);
    ready_=s.grant_ready&&s.control_owned&&s.general_low_speed_confirmed&&!s.fault_latched;
    sdk_hold_reason_=mc_hold?"mc_stale":"";
    return true;
  }
  // Only the SDK writer can supply this raw-MC/zero-write witness. Local LIO
  // velocity, zero commands and terminal stop acknowledgements are not this
  // nonterminal, same-execution stopped-state contract.
  bool observe(const Stationary& s,SourceClock now) {
    if(!confirmed_||stopping_||!active_||s.schema_version!=1||
       !sameVersion(s.version,active_->version)||s.applied_trajectory_id!=active_->trajectory_id||
       s.writer_commit_sequence!=applied_commit_sequence_||applied_commit_sequence_==0||
       s.execution_id!=execution_||s.control_epoch!=epoch_||s.sdk_session!=sdk_session_||
       s.sdk_arm_generation!=sdk_generation_||s.transport_mode!=transport_||
       s.sequence<=stationary_sequence_||!fresh(s.source_stamp,now,.25)||
       !fresh(s.received_stamp,now,.25))return false;
    stationary_sequence_=s.sequence;
    if(!stationary_clock_.empty()&&!s.mc_clock_epoch.empty()&&stationary_clock_!=s.mc_clock_epoch) {
      withdrawStationary("sdk_stationary_clock_epoch_changed",now);
      stop("sdk_stationary_clock_epoch_changed",now);return false;
    }
    // A newer unusable/invalid witness fences an older positive even when no
    // new positive can be admitted. This does not reset the blocked budget.
    if(!stationaryGood(s,now)) {
      withdrawStationary("stationary_evidence_withdrawn",now);
      return false;
    }
    // Keep the accepted raw floor independently of the optional usable cache:
    // an invalidation must not allow a second regressed sample to re-enter.
    if(stationary_raw_ns_&&s.mc_raw_stamp_ns<stationary_raw_ns_) {
      withdrawStationary("stationary_raw_time_rollback",now);return false;
    }
    stationary_clock_=s.mc_clock_epoch;stationary_raw_ns_=s.mc_raw_stamp_ns;stationary_=s;return true;
  }
  // This acknowledges a real writer submission, not a candidate preparation
  // receipt, a new permission heartbeat, or physical motion.  In particular a
  // successful submission whose evidence expired in transit still advances
  // the applied identity; fresh evidence is separately required by tick().
  bool observe(const CommitAck& a,SourceClock now) {
    if(!confirmed_||stopping_||!active_||a.schema_version!=1||a.sequence<=commit_ack_sequence_||
       a.execution_id!=execution_||a.control_epoch!=epoch_||a.sdk_session!=sdk_session_||
       a.sdk_arm_generation!=sdk_generation_||a.transport_mode!=transport_||
       !sameTask(binding_,a.candidate_version)||!complete(a.candidate_version)||
       !fresh(a.applied_at,now,.35))return false;
    if(a.handoff_id.empty()) {
      if(applied_commit_sequence_!=0||a.previous_commit_sequence!=0||a.commit_sequence!=1||
         a.grant_sequence!=0||!a.applied||!initial_submission_issued_)return false;
      const InitialGeometry* issued=nullptr;
      for(const auto&g:initial_geometries_)if(g.permit.sequence==a.permit_sequence&&g.permit.allowed&&
          sameVersion(g.permit.version,a.candidate_version)&&g.permit.trajectory_id==a.candidate_trajectory_id&&
          nanoseconds(g.permit.source_stamp)<=nanoseconds(a.applied_at)&&nanoseconds(g.permit.valid_until)>=nanoseconds(a.applied_at)&&
          submissionEvidence(a,g.permit.frame_id))issued=&g;
      if(!issued)return false;
      // A late first submission may name a previously issued initial intent.
      // Its real writer fact wins over the newer desired preview even if its
      // proof expired in transit; tick separately requires current evidence.
      active_storage_=issued->proof;active_=&active_storage_;
    }else {
      if(!handoff_||a.handoff_id!=handoff_->handoff_id||a.grant_sequence!=handoff_->sequence||
         a.previous_commit_sequence!=applied_commit_sequence_||
         a.previous_commit_sequence!=handoff_->expected_commit_sequence||
         !sameVersion(a.incumbent_version,handoff_->incumbent.version)||
         a.incumbent_trajectory_id!=handoff_->incumbent.trajectory_id||
         !sameVersion(a.candidate_version,handoff_->candidate.version)||
         a.candidate_trajectory_id!=handoff_->candidate.trajectory_id)return false;
      if(!a.applied) {
        if(a.commit_sequence!=applied_commit_sequence_||a.permit_sequence!=handoff_->candidate.sequence)return false;
        commit_ack_sequence_=a.sequence;
        handoff_.reset();prepared_.reset();handoff_reason_="writer_handoff_rejected:"+a.reason;
        return true;
      }
      if(a.commit_sequence!=applied_commit_sequence_+1||a.permit_sequence!=handoff_->candidate.sequence||
         a.entry_admission_sequence==0||nanoseconds(a.applied_at)<nanoseconds(handoff_->source_stamp)||
         nanoseconds(a.applied_at)>nanoseconds(handoff_->transition_deadline)||
         (handoff_revoked_at_>0&&seconds(a.applied_at)>handoff_revoked_at_)||
         !submissionEvidence(a,handoff_->candidate.frame_id)||!prepared_)return false;
      const bool stationary_reentry=handoff_->transition_mode==Handoff::STATIONARY_REENTRY;
      applied_installation_=AppliedInstallation{handoff_->candidate,a,{}};
      active_storage_=*prepared_;active_=&active_storage_;
      if(stationary_reentry)tracker_motion_fenced_=false;
      stationary_.reset();
      handoff_.reset();prepared_.reset();handoff_reason_.clear();
    }
    applied_commit_sequence_=a.commit_sequence;commit_ack_sequence_=a.sequence;applied_at_=seconds(a.applied_at);
    if(!a.write_submitted)stop("writer_submission_failed:"+a.reason,now);
    return true;
  }
  // Called only after the navigator has filled the goal/frame and signed the
  // final ordinary permission.  This prevents an unsigned draft from being
  // accepted as the writer's first commit lease.
  void notePublication(const Permit&p,SourceClock now) {
    if(!sameTask(binding_,p.version))return;
    // This is the final signed ordinary lease, not a draft from tick(). Event
    // driven preparation may retain it, but may never re-date its authority.
    published_permit_=p;
    if(!applied_commit_sequence_&&p.geometry_committed&&!p.revoked&&active_&&
       sameVersion(p.version,active_->version)&&p.trajectory_id==active_->trajectory_id&&
       p.validation_sequence==active_->sequence&&!p.frame_id.empty()) {
      const Admission* admission=nullptr;
      for(const auto&a:admissions_)if(a.accepted&&sameVersion(a.version,p.version)&&
          a.trajectory_id==p.trajectory_id&&a.validation_sequence==p.validation_sequence)admission=&a;
      // A proved tracker installation can renew current collision evidence,
      // but the original installation admission is never re-dated.
      if(!admission&&installedGood(now)&&sameVersion(installed_geometry_->proof.version,p.version)&&
         installed_geometry_->proof.trajectory_id==p.trajectory_id)admission=&installed_geometry_->admission;
      if(admission) {
        if(initial_geometries_.size()==8)initial_geometries_.erase(initial_geometries_.begin());
        initial_geometries_.push_back({p,*active_,*admission});
      }
    }
    if(!confirmed_)return;
    if(p.allowed&&p.geometry_committed&&!p.revoked) {
      initial_submission_issued_=true;
    }
    const bool stationary_hold=handoff_&&handoff_->transition_mode==Handoff::STATIONARY_REENTRY&&
      !p.allowed&&!p.revoked&&p.phase=="holding"&&navigation_usable_&&ready_&&
      sameVersion(p.version,handoff_->incumbent.version)&&p.trajectory_id==handoff_->incumbent.trajectory_id&&
      stationaryGood(handoff_->stationary_evidence,now);
    if(handoff_&&!stationary_hold&&(!p.allowed||p.revoked||p.phase!="tracking")) {
      handoff_->revoked=true;handoff_->reason="owner_handoff_withdrawn:"+p.reason;
      if(handoff_revoked_at_<=0)handoff_revoked_at_=now;
    }
  }
  const Version* pendingVersion(SourceClock now)const {
    if(!confirmed_||stopping_||!ready_||!navigation_usable_||
       (phase_!="tracking"&&!stationaryReentryReady(now))||!active_||
       applied_commit_sequence_==0||handoff_)return nullptr;
    const auto*c=candidate(now);
    return c&&(!sameVersion(c->version,active_->version)||c->trajectory_id!=active_->trajectory_id)?&c->version:nullptr;
  }
  const Permit* publishedHandoffIncumbent(SourceClock now)const {
    if(!published_permit_||!confirmed_||stopping_||!ready_||!navigation_usable_||!active_||handoff_||
       applied_commit_sequence_==0||sdk_applied_commit_sequence_>applied_commit_sequence_||
       !fresh(sdk_stamp_,now,.5))return nullptr;
    const auto&p=*published_permit_;
    if(p.revoked||!p.geometry_committed||p.sequence==0||p.frame_id.empty()||
       !sameVersion(p.version,active_->version)||p.trajectory_id!=active_->trajectory_id||
       !fresh(p.source_stamp,now,.35)||compare(p.valid_until,now)<=0)return nullptr;
    if(p.allowed) {
      if(phase_!="tracking"||p.phase!="tracking"||!proofFor(*active_,now))return nullptr;
    } else if(p.phase!="holding"||!stationaryReentryReady(now))return nullptr;
    return &p;
  }
  bool prepareHandoff(const Permit& incumbent,const GoalLedger::Proposal& goal,
      uint64_t candidate_sequence,SourceClock now) {
    const auto*version=pendingVersion(now);const auto*c=candidate(now);
    const bool reentry=!incumbent.allowed&&incumbent.phase=="holding"&&stationaryReentryReady(now);
    if(!version||!c||candidate_sequence==0||incumbent.revoked||
       !incumbent.geometry_committed||(!reentry&&(!incumbent.allowed||incumbent.phase!="tracking"))||
       !sameVersion(incumbent.version,active_->version)||incumbent.trajectory_id!=active_->trajectory_id||
       !sameVersion(goal.version,*version)||goal.reference.path.header.frame_id!=incumbent.frame_id||
       goal.has_goal_yaw!=incumbent.has_goal_yaw||
       (goal.has_goal_yaw&&std::abs(goal.goal_yaw_tolerance_rad-incumbent.goal_yaw_tolerance_rad)>1e-9)||
       incumbent.frame_id.empty()||compare(incumbent.valid_until,now)<=0)return false;
    // A candidate with only a few milliseconds left is geometrically valid,
    // but cannot usefully traverse prepare/sweep/gate + the 20 Hz writer.
    // This is a 100 ms software scheduling target (50 + 50), not a physical
    // stopping guarantee or an extension of any original proof/lease.
    double entry_until=0.;
    for(const auto&a:admissions_)if(a.accepted&&sameVersion(a.version,c->version)&&
        a.trajectory_id==c->trajectory_id&&a.validation_sequence==c->sequence)
      entry_until=seconds(a.valid_until);
    handoff_ready_window_=std::min({seconds(c->valid_until),entry_until,seconds(incumbent.valid_until),
      reentry?seconds(stationary_->valid_until):now+.25})-now;
    if(handoff_ready_window_+1e-9<.100) {
      handoff_reason_="waiting_candidate_usable_window";return false;
    }
    Handoff h;h.schema_version=2;h.transition_mode=reentry?Handoff::STATIONARY_REENTRY:Handoff::CONTINUOUS_REPLACE;
    h.handoff_id=execution_+":handoff:"+std::to_string(++handoff_sequence_);
    h.sequence=handoff_sequence_;h.expected_commit_sequence=applied_commit_sequence_;
    h.incumbent=incumbent;h.candidate=incumbent;h.candidate.version=c->version;
    h.candidate.trajectory_id=c->trajectory_id;h.candidate.validation_sequence=c->sequence;
    h.candidate.sequence=candidate_sequence;h.candidate.geometry_committed=false;
    h.candidate.allowed=true;h.candidate.phase="tracking";
    h.candidate.goal_position=goal.goal_position;h.candidate.has_goal_yaw=goal.has_goal_yaw;
    h.candidate.goal_yaw=goal.goal_yaw;h.candidate.goal_yaw_tolerance_rad=goal.goal_yaw_tolerance_rad;
    h.candidate.source_stamp=h.source_stamp=stamp(now);
    if(reentry) {
      h.stationary_evidence=*stationary_;
      h.retain_incumbent_until=h.source_stamp;
      h.transition_deadline=h.valid_until=earliestStamp({stamp(after(now,.25)),c->valid_until,stationary_->valid_until});
      h.reason="stationary_same_route_writer_cas";
    } else {
      h.transition_deadline=h.retain_incumbent_until=h.valid_until=
        earliestStamp({stamp(after(now,.25)),incumbent.valid_until});
      h.reason="conditional_writer_cas";
    }
    if(compare(h.transition_deadline,now)<=0)return false;
    h.candidate.valid_until=h.valid_until;
    prepared_=*c;handoff_=h;handoff_revoked_at_=0.;handoff_reason_.clear();return true;
  }
  const Handoff* handoff()const{return handoff_?&*handoff_:nullptr;}
  uint64_t appliedCommitSequence()const{return applied_commit_sequence_;}
  const std::string& handoffReason()const{return handoff_reason_;}
  double handoffReadyWindow()const{return handoff_ready_window_;}
  bool observe(const MotionDemand& d,SourceClock now) {
    if(!executionFeedbackIdentity(d)||d.sequence<=demand_sequence_||
       !fresh(d.source_stamp,now,.35)||!fresh(d.body_source_stamp,now,.4)||
       compare(d.valid_until,now)<0||elapsed(d.valid_until,d.source_stamp)>.35)return false;
    demand_sequence_=d.sequence;
    if(safe_demands_.size()==16)safe_demands_.erase(safe_demands_.begin());safe_demands_.push_back(d);return true;
  }
  bool observe(const MotionValidation& m,SourceClock now) {
    if(!executionFeedbackIdentity(m)||m.sequence<=motion_sequence_||
       !fresh(m.check_end,now,.35)||!fresh(m.body_source_stamp,now,.4)||
       !fresh(m.demand_body_source_stamp,now,.4)||
       !fresh(m.demand_source_stamp,now,.35)||
       (m.valid&&(compare(m.valid_until,now)<0||elapsed(m.valid_until,m.check_end)>.5)))return false;
    motion_sequence_=m.sequence;
    if(!m.valid)motion_block_reason_="actual_command_blocked:"+m.reason;
    if(motion_proofs_.size()==16)motion_proofs_.erase(motion_proofs_.begin());motion_proofs_.push_back(m);return true;
  }
  bool observe(const Stop& s,SourceClock now) {
    if(!stopping_||s.execution_id!=execution_||s.control_epoch!=epoch_||s.sdk_session!=sdk_session_||
       s.sdk_arm_generation!=sdk_generation_||s.stop_request_id!=confirmation_||s.sequence<=stop_sequence_||
       s.mc_raw_stamp_ns==0||s.mc_clock_epoch.empty()||seconds(s.source_stamp)-s.stationary_duration_sec<stop_at_||
       s.time_basis!=(transport_=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock")||
       !sameTask(binding_,s.version)||s.transport_mode!=transport_||
       !fresh(s.source_stamp,now,.3)||!s.nonzero_blocked||!s.stop_submitted||
       !s.measured_stop_confirmed||s.stationary_duration_sec<stationary_limits_.stop_duration||s.stationary_samples<stationary_limits_.minimum_samples||
       !std::isfinite(s.measured_linear_mps)||s.measured_linear_mps<0||s.measured_linear_mps>stationary_limits_.linear||
       !std::isfinite(s.measured_angular_radps)||s.measured_angular_radps<0||s.measured_angular_radps>stationary_limits_.angular||
       (transport_=="live"&&!s.physical_acceptance_verified))return false;
    stop_sequence_=s.sequence;
    if(!stopped_) {
      stopped_=true;stop_confirmed_at_=now;stop_source_=seconds(s.source_stamp);
      phase_=goal_stop_?"verifying_arrival":"terminal";
      if(goal_stop_)reason_="waiting_post_stop_measured_arrival";
    }return true;
  }
  void stop(const std::string& why,SourceClock now,bool goal=false) {
    if(!confirmed_||stopping_)return;stopping_=true;goal_stop_=goal;stop_at_=now;
    phase_="stopping";reason_=why;ready_=false;
    if(handoff_){handoff_->revoked=true;handoff_->reason=why;handoff_revoked_at_=now;}
  }
  void measuredGoal(bool position,bool yaw_required,bool yaw_ok,SourceClock now,double body_source=0.) {
    if(stopped_&&goal_stop_&&!arrival_verified_&&!needsReview()) {
      if(!fresh(body_source,now,.4)||body_source<stop_source_)return;
      arrival_verified_=position&&(!yaw_required||yaw_ok);
      phase_=arrival_verified_?"terminal":"needs_review";
      reason_=arrival_verified_?"goal_reached":"arrival_tolerance_lost_after_stop";return;
    }
    if(!confirmed_||stopping_||!ready_)return;
    if(position) {if(yaw_required&&!yaw_ok)phase_="aligning";else stop("goal_reached",now,true);}
    else if(phase_=="aligning")phase_="tracking";
  }
  bool yawProof(double yaw,SourceClock now)const {
    const auto* p=active_?proofFor(*active_,now):nullptr;
    return p&&p->goal_yaw_checked&&std::isfinite(p->checked_goal_yaw)&&std::isfinite(yaw)&&
      std::abs(std::remainder(p->checked_goal_yaw-yaw,2*M_PI))<=1e-6;
  }
  std::string progressReason()const {return blocked_at_>=0?blocked_reason_:"";}
  double blockedAge(SourceClock now)const {return blocked_at_>=0?std::max(0.,now-blocked_at_):0.;}
  // Supervise the FINAL emitted lease, including Navigator's goal/yaw HOLDs.
  // Preview and arming have their own admission lifetimes. A momentary good
  // proof does not restart a blocked episode. Recovery requires both a real
  // nonzero command admitted by the independent safety gate and measured local
  // fixed-route arc progress (or actual final-yaw error reduction); lateral
  // travel and anchor corrections cannot replenish this bounded wait episode.
  std::string supervise(Permit& p,SourceClock now) {return supervise(p,now,static_cast<const NavigationState*>(nullptr));}
  template<class State>
  std::string supervise(Permit& p,SourceClock now,const State* body) {
    if(!confirmed_||stopping_||p.phase=="arming"||p.phase=="preview")return {};
    const auto progress=measuredProgress(p,body,now);
    if(!p.allowed||p.revoked||!progress.empty()) {
      if(blocked_at_<0)blocked_at_=now;
      recovered_at_=-1.;blocked_reason_=(!p.allowed||p.revoked)?p.reason:progress;
    } else if(blocked_at_>=0) {
      if(recovered_at_<0){recovered_at_=now;recovery_progress_samples_=progress_samples_;}
      if(now-recovered_at_>=recovery_time_&&progress_samples_-recovery_progress_samples_>=2) {
        blocked_at_=recovered_at_=-1.;blocked_reason_.clear();motion_block_reason_.clear();}
    }
    if(blocked_at_>=0&&now-blocked_at_>=blocked_timeout_) {
      const auto why="execution_blocked_timeout:"+blocked_reason_;
      stop(why,now);p.allowed=false;p.revoked=true;p.phase=phase_;p.reason=reason_;
      return why;
    }
    return {};
  }
  Permit tick(SourceClock now,bool navigation_ready,std::int64_t evidence_source_ns=0,
      const std::string& evidence_identity={},double monotonic=std::numeric_limits<double>::quiet_NaN()) {
    navigation_usable_=false;
    const Validation* initial=nullptr;
    if(!applied_commit_sequence_) {
      initial=installedInitialProof(now);if(!initial)initial=candidate(now);
      if(initial){active_storage_=*initial;active_=&active_storage_;}
    }
    Permit p;p.version=active_?active_->version:binding_;p.execution_id=execution_;p.control_epoch=epoch_;
    p.confirmation_id=confirmation_;p.sdk_session=sdk_session_;p.sdk_arm_generation=sdk_generation_;
    p.sequence=++sequence_;p.source_stamp=stamp(now);p.valid_until=stamp(after(now,.25));p.transport_mode=transport_;
    // Geometry ownership survives arming or a short evidence HOLD. This is not
    // a motion lease: allowed remains false until every execution proof passes.
    // Dropping geometry here forced consumers to re-admit the same old curve
    // using a now-expired first-preparation receipt after every pause.
    if(active_){p.trajectory_id=active_->trajectory_id;p.validation_sequence=active_->sequence;
      p.geometry_committed=applied_commit_sequence_||initial!=nullptr;}
    if(!confirmed_){
      const auto*c=installedInitialProof(now);
      if(!c)c=candidate(now);
      if(c){active_storage_=*c;active_=&active_storage_;}
      if(active_){p.version=active_->version;p.trajectory_id=active_->trajectory_id;
        p.validation_sequence=active_->sequence;p.geometry_committed=c!=nullptr;}
      p.phase="preview";p.reason="awaiting_execution_confirmation";return p;
    }
    if(now<last_now_)stop("execution_clock_rollback",now);last_now_=now;
    if(stopping_) {
      if(!stopped_&&now-stop_at_>5.&&phase_!="needs_review"){phase_="needs_review";reason_="physical_stop_unconfirmed:"+reason_;}
      if(stopped_&&goal_stop_&&!arrival_verified_&&now-stop_confirmed_at_>1.) {
        phase_="needs_review";reason_="post_stop_arrival_pose_unavailable";
      }
      p.revoked=true;p.phase=phase_;p.reason=reason_;return p;
    }
    if(phase_=="arming") {
      if(now-started_>10.)stop("execution_arm_timeout",now);
      if(!ready_){p.phase=phase_;p.reason="waiting_sdk_grant";p.revoked=stopping_;return p;}
    }
    if(!fresh(sdk_stamp_,now,.5)){stop("sdk_state_stale",now);p.revoked=true;p.phase=phase_;p.reason=reason_;return p;}
    if(!navigation_recovery_.observe(navigation_ready&&ready_,std::isfinite(monotonic)?monotonic:static_cast<double>(now),
         evidence_source_ns,evidence_identity)) {
      phase_="holding";p.phase=phase_;
      p.reason=!ready_?(sdk_hold_reason_.empty()?"waiting_sdk_grant":sdk_hold_reason_):
        (navigation_ready?"confirming_navigation_recovery":"navigation_inputs_unavailable");
      return p;
    }
    navigation_usable_=true;
    // SDK status can reveal an unreceived writer submission but is not the
    // exact commit ACK. Freeze authority until that actual fact is bound to
    // its issued transaction; do not keep signing a newer initial intent.
    if(sdk_applied_commit_sequence_>applied_commit_sequence_) {
      phase_="holding";p.phase=phase_;p.reason="waiting_writer_commit_ack";return p;
    }
    if(tracker_motion_fenced_) {
      phase_="holding";p.phase=phase_;p.reason="braking_envelope_reentry_required";
      return p;
    }
    const auto* c=candidate(now);
    // New failed preparation does not revoke a current still-fresh proof.
    // First commit must carry the exact native sequence the tracker prepared.
    // Refreshing N to N+1 here would silently break the two-phase handshake.
    // Once execution starts, an admitted candidate is only prepared geometry.
    // The old version stays the owner-visible active version until the one SDK
    // writer reports a matching actual submission.  No BT round-trip is needed
    // to allow its conditional CAS; ordinary permits never carry candidates.
    const auto* current=applied_commit_sequence_?(active_?proofFor(*active_,now):nullptr):installedInitialProof(now);
    if(!current&&!applied_commit_sequence_)current=c;
    if(!current|| (handoff_&&handoff_->transition_mode==Handoff::STATIONARY_REENTRY)) {
      phase_="holding";p.phase=phase_;p.reason=handoff_?"waiting_stationary_writer_handoff_outcome":
        (applied_commit_sequence_?"waiting_current_collision_and_tracker_proof":"waiting_initial_tracker_admission");
      return p;
    }
    active_storage_=*current;active_=&active_storage_;
    if(phase_!="aligning")phase_="tracking";
    p.version=active_->version;p.trajectory_id=active_->trajectory_id;p.validation_sequence=active_->sequence;
    // Authority and collision evidence have independent leases. A still-live
    // same-version demand may use a newer native proof than this signing floor.
    // Consumers must bind the exact actual proof and its original deadline;
    // they may never interpret this owner heartbeat as collision evidence.
    p.valid_until=stamp(after(now,.25));
    if(handoff_) {
      p.valid_until=earliestStamp({p.valid_until,handoff_->retain_incumbent_until});
      if(handoff_->revoked||compare(handoff_->transition_deadline,now)<0) {
        p.allowed=false;p.phase=phase_="holding";p.reason="waiting_writer_handoff_outcome";
        handoff_reason_=p.reason;return p;
      }
    }
    p.allowed=true;p.geometry_committed=true;p.phase=phase_;p.reason="committed_execution_lease";return p;
  }
  bool confirmed()const{return confirmed_;} bool stopping()const{return stopping_;}
  bool stopped()const{return stopped_;} bool goalStopped()const{return stopped_&&goal_stop_&&arrival_verified_;}
  bool needsReview()const{return phase_=="needs_review";}
  // An irreversible grant/transport failure must reach the task owner even if
  // FollowRoute's algorithm still reports RUNNING. Stop confirmation is a
  // separate retirement fence, not permission to keep an abandoned action
  // alive. Ordinary evidence HOLD and a normal final stop are not failures.
  std::string failureReason()const {
    if(!confirmed_)return {};
    return (stopping_&&!goal_stop_)||needsReview()?reason_:std::string{};
  }
  const std::string& executionId()const{return execution_;}uint64_t controlEpoch()const{return epoch_;}
  const std::string& phase()const{return phase_;}const std::string& reason()const{return reason_;}
  Version grantVersion(SourceClock now)const {
    const auto*c=installedInitialProof(now);if(!c)c=candidate(now);
    return c?c->version:binding_;
  }
private:
  void withdrawStationary(const std::string& why,SourceClock now) {
    stationary_.reset();
    if(handoff_&&handoff_->transition_mode==Handoff::STATIONARY_REENTRY) {
      handoff_->revoked=true;handoff_->reason=why;
      if(handoff_revoked_at_<=0)handoff_revoked_at_=now;
    }
  }
  bool submissionEvidence(const CommitAck&a,const std::string& frame)const {
    const auto&pose=a.measured_pose.pose;const auto&q=pose.orientation;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    const auto&v=a.applied_velocity;
    const bool moving=std::abs(v.linear.x)+std::abs(v.linear.y)+std::abs(v.linear.z)+
      std::abs(v.angular.x)+std::abs(v.angular.y)+std::abs(v.angular.z)>1e-9;
    return !frame.empty()&&a.measured_pose.header.frame_id==frame&&
      a.measured_pose.header.stamp==a.body_source_stamp&&a.demand_sequence>0&&(!moving||a.motion_validation_sequence>0)&&
      fresh(a.body_source_stamp,a.applied_at,.1)&&
      fresh(a.demand_body_source_stamp,a.applied_at,.1)&&
      fresh(a.demand_source_stamp,a.applied_at,.1)&&
      nanoseconds(a.demand_body_source_stamp)<=nanoseconds(a.demand_source_stamp)&&
      nanoseconds(a.demand_body_source_stamp)<=nanoseconds(a.body_source_stamp)&&
      (a.handoff_id.empty()||a.entry_source_stamp==a.demand_body_source_stamp)&&
      nanoseconds(a.valid_until)>=nanoseconds(a.applied_at)&&std::isfinite(a.curve_time)&&a.curve_time>=0&&
      std::isfinite(v.linear.x)&&std::isfinite(v.linear.y)&&std::isfinite(v.linear.z)&&
      std::isfinite(v.angular.x)&&std::isfinite(v.angular.y)&&std::isfinite(v.angular.z)&&
      std::isfinite(a.measured_twist.linear.x)&&std::isfinite(a.measured_twist.linear.y)&&
      std::isfinite(a.measured_twist.linear.z)&&std::isfinite(a.measured_twist.angular.x)&&
      std::isfinite(a.measured_twist.angular.y)&&std::isfinite(a.measured_twist.angular.z)&&
      std::isfinite(pose.position.x)&&std::isfinite(pose.position.y)&&std::isfinite(pose.position.z)&&
      std::isfinite(norm)&&std::abs(norm-1.)<=.02;
  }
  template<class Message> bool executionFeedbackIdentity(const Message&m)const {
    return confirmed_&&!stopping_&&active_&&sameVersion(active_->version,m.version)&&
      m.trajectory_id==active_->trajectory_id&&m.execution_id==execution_&&m.control_epoch==epoch_&&
      m.sdk_session==sdk_session_&&m.sdk_arm_generation==sdk_generation_&&m.transport_mode==transport_;
  }
  double admittedNonzero(const Permit&p,SourceClock now)const {
    for(auto d=safe_demands_.rbegin();d!=safe_demands_.rend();++d) {
      if(!sameVersion(d->version,p.version)||d->trajectory_id!=p.trajectory_id||d->hold||!d->safety_checked||
         (std::abs(d->velocity.linear.x)<1e-6&&std::abs(d->velocity.angular.z)<1e-6)||
         !fresh(d->source_stamp,now,.35)||compare(d->valid_until,now)<0)continue;
      for(auto m=motion_proofs_.rbegin();m!=motion_proofs_.rend();++m) {
        if(!sameVersion(m->version,p.version)||m->trajectory_id!=p.trajectory_id)continue;
        if(!m->valid)return 0.; // latest real motion rejection fences older admissions
        if(m->sequence!=d->motion_validation_sequence)continue;
        if(m->demand_sequence==d->sequence&&m->permit_sequence==d->permit_sequence&&
           m->trajectory_validation_sequence>=d->validation_sequence&&
           m->demand_source_stamp==d->source_stamp&&m->demand_body_source_stamp==d->body_source_stamp&&
           m->demand_valid_until==d->valid_until&&m->velocity==d->velocity&&
           m->frame_id==p.frame_id&&m->braking_model_sha256==d->braking_model_sha256&&!m->braking_model_sha256.empty()&&
           compare(m->valid_until,now)>=0&&fresh(m->check_end,now,.35))return seconds(d->source_stamp);
      }
    }
    return 0.;
  }
  template<class State>
  std::string measuredProgress(const Permit&p,const State*b,SourceClock now) {
    const auto admitted=admittedNonzero(p,now);
    if(admitted>last_admitted_motion_)last_admitted_motion_=admitted;
    constexpr unsigned schema=std::is_same_v<State,d1max_planning_interfaces::msg::LocalNavigationState>?1u:2u;
    const bool body_ok=b&&b->schema_version==schema&&b->usable&&b->session_id==binding_.session_id&&
      b->map_version_id==binding_.map_version_id&&b->localization_epoch==binding_.localization_epoch&&
      b->localization_seed_id==binding_.localization_seed_id&&
      b->local_odometry.header.frame_id==p.frame_id&&b->local_odometry.header.stamp==b->source_stamp&&
      fresh(b->source_stamp,now,.4);
    if(!body_ok)return "waiting_measured_body_progress";
    route_progress_.observeBody(*b);
    const auto&pose=b->local_odometry.pose.pose;const auto&q=pose.orientation;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if(!std::isfinite(pose.position.x)||!std::isfinite(pose.position.y)||!std::isfinite(norm)||std::abs(norm-1.)>.02)
      return "invalid_measured_body_progress";
    const auto source=static_cast<std::int64_t>(b->source_stamp.sec)*1000000000LL+b->source_stamp.nanosec;
    const auto yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    const auto yaw_error=std::abs(std::remainder(p.goal_yaw-yaw,2*M_PI));
    if(!progress_anchor_){progress_anchor_=true;progress_x_=pose.position.x;progress_y_=pose.position.y;
      progress_yaw_error_=yaw_error;progress_phase_=p.phase;}
    if(p.phase!=progress_phase_){progress_phase_=p.phase;progress_yaw_error_=yaw_error;}
    if(source>progress_source_ns_) {
      progress_source_ns_=source;
      // Recent actual admission is supervisory evidence only, NOT a lease:
      // individual commands retain their original native/SDK expiry checks.
      if(fresh(last_admitted_motion_,now,.5)&&
         ((p.phase=="aligning"&&progress_yaw_error_-yaw_error>=.05)||
          (p.phase!="aligning"&&route_progress_.advance(p.version,p.frame_id,now)))) {
        progress_at_=now;++progress_samples_;progress_x_=pose.position.x;progress_y_=pose.position.y;progress_yaw_error_=yaw_error;
      }
    }
    if(!fresh(last_admitted_motion_,now,.5))return motion_block_reason_.empty()?"waiting_admitted_nonzero_command":motion_block_reason_;
    if(progress_at_<0||now-progress_at_>1.)return "waiting_measured_motion_progress";
    return {};
  }
  bool proofGood(const Validation& v,SourceClock now)const {
    const bool domain=std::isfinite(v.checked_from_time)&&std::isfinite(v.checked_to_time)&&
      std::isfinite(v.curve_duration)&&std::isfinite(v.valid_start_time)&&v.curve_duration>0&&
      v.checked_from_time>=0&&v.checked_from_time<=v.valid_start_time&&v.valid_start_time<=v.checked_to_time&&
      std::abs(v.checked_to_time-v.curve_duration)<=1e-6;
    const bool whole=v.whole_curve&&!v.remaining_curve&&domain&&v.checked_from_time==0.;
    const bool installed=applied_commit_sequence_||
      (installedGood(now)&&sameVersion(v.version,installed_geometry_->proof.version)&&
       v.trajectory_id==installed_geometry_->proof.trajectory_id);
    const bool suffix=!v.whole_curve&&v.remaining_curve&&installed&&active_&&sameVersion(v.version,active_->version)&&
      v.trajectory_id==active_->trajectory_id&&domain&&std::isfinite(v.reverse_margin_m)&&v.reverse_margin_m>=.15;
    return v.valid&&(whole||suffix)&&!v.support_reference_id.empty()&&!v.support_hash.empty()&&compare(v.valid_until,now)>=0&&
      fresh(v.check_end,now,.35)&&fresh(v.source_stamp,now,.35)&&
      fresh(v.front_ray_source_stamp,now,.6)&&fresh(v.rear_ray_source_stamp,now,.6);
  }
  bool stationaryGood(const Stationary& s,SourceClock now)const {
    const double source=seconds(s.source_stamp),received=seconds(s.received_stamp);
    const double lower=seconds(s.capture_lower_bound),upper=seconds(s.capture_upper_bound);
    return s.schema_version==1&&s.usable&&s.nonzero_blocked&&s.sequence>0&&s.zero_write_sequence>0&&
      s.mc_raw_stamp_ns>0&&!s.mc_clock_epoch.empty()&&s.writer_commit_sequence==applied_commit_sequence_&&
      active_&&sameVersion(s.version,active_->version)&&s.applied_trajectory_id==active_->trajectory_id&&
      s.execution_id==execution_&&s.control_epoch==epoch_&&s.sdk_session==sdk_session_&&
      s.sdk_arm_generation==sdk_generation_&&s.transport_mode==transport_&&
      s.time_basis==(transport_=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock")&&
      (transport_!="live"||s.physical_acceptance_verified)&&
      fresh(source,now,.25)&&fresh(received,now,.25)&&
      std::isfinite(s.mc_capture_delay_bound_sec)&&s.mc_capture_delay_bound_sec>0&&s.mc_capture_delay_bound_sec<=.25&&
      seconds(s.zero_ack_at)>=applied_at_&&seconds(s.zero_ack_at)>0&&seconds(s.zero_ack_at)<lower&&lower<=std::min(source,received)&&
      upper==received&&source<=upper&&upper-lower<=s.mc_capture_delay_bound_sec+std::abs(source-received)+1e-6&&
      s.stationary_samples>=stationary_limits_.minimum_samples&&std::isfinite(s.stationary_duration_sec)&&s.stationary_duration_sec>=stationary_limits_.reentry_duration&&
      std::isfinite(s.measured_linear_mps)&&s.measured_linear_mps>=0&&s.measured_linear_mps<=stationary_limits_.linear&&
      std::isfinite(s.measured_angular_radps)&&s.measured_angular_radps>=0&&s.measured_angular_radps<=stationary_limits_.angular&&
      compare(s.valid_until,now)>=0&&seconds(s.valid_until)<=std::min(source,received)+.25+1e-6;
  }
  bool stationaryReentryReady(SourceClock now)const {
    return confirmed_&&!stopping_&&ready_&&navigation_usable_&&phase_=="holding"&&stationary_&&
      stationaryGood(*stationary_,now)&&fresh(sdk_stamp_,now,.5);
  }
  void observeTrackerMotionFence(const GeometryReceipt&r,SourceClock now) {
    if(!r.installed||r.reason!="braking_envelope_reentry_required"||!confirmed_||stopping_||
       applied_commit_sequence_==0||!active_||!sameVersion(r.version,active_->version)||
       r.trajectory_id!=active_->trajectory_id)return;
    tracker_motion_fenced_=true;phase_="holding";
    // A continuous candidate that was prepared before the braking fence must
    // not borrow the incumbent's motion. A genuinely pending stationary CAS
    // retains its pinned raw-MC witness and is not cancelled by a heartbeat.
    if(handoff_&&handoff_->transition_mode!=Handoff::STATIONARY_REENTRY) {
      handoff_->revoked=true;handoff_->reason="tracker_braking_envelope_reentry_required";
      if(handoff_revoked_at_<=0)handoff_revoked_at_=now;
    }
  }
  const Validation* proofFor(const Validation& old,SourceClock now)const {
    for(auto i=proofs_.rbegin();i!=proofs_.rend();++i)if(sameVersion(i->version,old.version)&&i->trajectory_id==old.trajectory_id)
      return proofGood(*i,now)?&*i:nullptr;
    return nullptr;
  }
  bool installedGood(SourceClock now)const {
    return installed_geometry_&&installed_receipt_&&installed_receipt_->installed&&
      fresh(installed_receipt_->body_source_stamp,now,.4);
  }
  const Validation* installedInitialProof(SourceClock now)const {
    return installedGood(now)?proofFor(installed_geometry_->proof,now):nullptr;
  }
  const Validation* candidate(SourceClock now)const {
    const Validation* best=nullptr;
    for(const auto& v:proofs_) {
      if(!proofGood(v,now))continue;
      bool received=false;
      for(const auto&r:receipts_)if(r.accepted&&r.proposal_id==v.proposal_id&&sameVersion(r.version,v.version)&&
        (active_&&sameVersion(active_->version,v.version)||
         (!active_&&r.expected_version.schema_version==0&&r.expected_trajectory_id==-1)||
         (active_&&sameVersion(active_->version,r.expected_version)&&r.expected_trajectory_id==active_->trajectory_id)))received=true;
      if(!received)continue;
      const auto*latest=proofFor(v,now);if(!latest)continue;
      if(active_&&(v.version.reference_generation<active_->version.reference_generation||
          (v.version.reference_generation==active_->version.reference_generation&&v.trajectory_id<active_->trajectory_id)))continue;
      for(const auto&a:admissions_)if(a.accepted&&sameVersion(a.version,v.version)&&a.trajectory_id==v.trajectory_id&&
          a.validation_sequence==v.sequence&&fresh(a.checked_at,now,.35)&&compare(a.valid_until,now)>=0) {
        if(!best||std::tie(v.version.reference_generation,v.trajectory_id,v.sequence)>
            std::tie(best->version.reference_generation,best->trajectory_id,best->sequence))best=&v;
      }
    }
    return best;
  }
  std::string transport_,execution_,confirmation_,sdk_session_,phase_="preview",reason_,blocked_reason_,sdk_hold_reason_;
  Version binding_;std::vector<Validation>proofs_;std::vector<Admission>admissions_;std::vector<Receipt>receipts_;
  std::vector<MotionDemand>safe_demands_;std::vector<MotionValidation>motion_proofs_;
  Validation active_storage_;const Validation*active_=nullptr;
  std::optional<Validation>prepared_;std::optional<Handoff>handoff_;
  std::optional<Permit>published_permit_;
  std::vector<InitialGeometry>initial_geometries_;
  std::optional<InitialGeometry>installed_geometry_;std::optional<GeometryReceipt>installed_receipt_;
  std::optional<AppliedInstallation>applied_installation_;bool tracker_motion_fenced_=false;
  uint64_t geometry_receipt_sequence_=0;
  double geometry_body_source_=0.;
  uint64_t applied_commit_sequence_=0,commit_ack_sequence_=0,handoff_sequence_=0,sdk_applied_commit_sequence_=0;
  StationaryLimits stationary_limits_;
  uint64_t stationary_sequence_=0,stationary_raw_ns_=0;std::optional<Stationary>stationary_;std::string stationary_clock_;
  bool navigation_usable_=false;double applied_at_=0.;
  double handoff_revoked_at_=0.;std::string handoff_reason_;bool initial_submission_issued_=false;
  double handoff_ready_window_=-1.;
  uint64_t epoch_=0,sequence_=0,sdk_sequence_=0,sdk_generation_=0,stop_sequence_=0;
  SourceClock sdk_stamp_;
  double started_=0,stop_at_=0,last_now_=0,stop_source_=0,stop_confirmed_at_=0;
  double blocked_timeout_=30.,recovery_time_=1.,blocked_at_=-1.,recovered_at_=-1.;
  uint64_t demand_sequence_=0,motion_sequence_=0;
  uint64_t progress_samples_=0,recovery_progress_samples_=0;
  RouteProgressSupervisor route_progress_;
  std::int64_t progress_source_ns_=0;
  double last_admitted_motion_=0.,progress_at_=-1.,progress_x_=0.,progress_y_=0.,progress_yaw_error_=0.;
  std::string progress_phase_,motion_block_reason_;bool progress_anchor_=false;
  d1max_navigation_bt::StableRecoveryGate navigation_recovery_;
  bool confirmed_=false,ready_=false,stopping_=false,stopped_=false,goal_stop_=false,arrival_verified_=false;
};
} // namespace exec3
} // namespace d1max_navigation_bt
