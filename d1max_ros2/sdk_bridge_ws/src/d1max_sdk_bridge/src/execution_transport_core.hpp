#pragma once
#include <d1max_planning_interfaces/msg/execution_permit.hpp>
#include <d1max_planning_interfaces/msg/motion_demand.hpp>
#include <d1max_planning_interfaces/msg/motion_validation.hpp>
#include <d1max_planning_interfaces/msg/sdk_execution_state.hpp>
#include <d1max_planning_interfaces/msg/sdk_execution_retirement.hpp>
#include <d1max_planning_interfaces/msg/stop_report.hpp>
#include <d1max_planning_interfaces/msg/execution_handoff_grant.hpp>
#include <d1max_planning_interfaces/msg/prepared_motion_demand.hpp>
#include <d1max_planning_interfaces/msg/execution_commit_ack.hpp>
#include <d1max_planning_interfaces/msg/sdk_stationary_evidence.hpp>
#include <d1max_planning_interfaces/msg/local_navigation_state.hpp>
#include <d1max_planning_interfaces/msg/trajectory_validation.hpp>
#include <d1max_planning_interfaces/msg/tracking_progress.hpp>
#include <d1max_planning_interfaces/msg/tagged_bspline.hpp>
#include <d1max_planning_interfaces/msg/task_execution_lease.hpp>
#include <d1max_planning_interfaces/msg/local_motion_command.hpp>
#include <d1max_planning_interfaces/srv/execution_grant.hpp>
#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <stdexcept>
#include "execution_policy.hpp"
#include "mc_capture_time.hpp"
#include "execution_kinematics.hpp"
#include "task_command_feedback.hpp"
#include "measured_curve_entry.hpp"

namespace d1monitor::execution3 {
using Version=d1max_planning_interfaces::msg::ExecutionVersion;
using Permit=d1max_planning_interfaces::msg::ExecutionPermit;
using Demand=d1max_planning_interfaces::msg::MotionDemand;
using MotionProof=d1max_planning_interfaces::msg::MotionValidation;
using State=d1max_planning_interfaces::msg::SDKExecutionState;
using Retirement=d1max_planning_interfaces::msg::SDKExecutionRetirement;
using Stop=d1max_planning_interfaces::msg::StopReport;
using Grant=d1max_planning_interfaces::srv::ExecutionGrant;
using Handoff=d1max_planning_interfaces::msg::ExecutionHandoffGrant;
using Prepared=d1max_planning_interfaces::msg::PreparedMotionDemand;
using CommitAck=d1max_planning_interfaces::msg::ExecutionCommitAck;
using Stationary=d1max_planning_interfaces::msg::SDKStationaryEvidence;
using Local=d1max_planning_interfaces::msg::LocalNavigationState;
using Tagged=d1max_planning_interfaces::msg::TaggedBspline;
using CurveProof=d1max_planning_interfaces::msg::TrajectoryValidation;
using Progress=d1max_planning_interfaces::msg::TrackingProgress;
using TaskLease=d1max_planning_interfaces::msg::TaskExecutionLease;
using LocalCommand=d1max_planning_interfaces::msg::LocalMotionCommand;
// Read-only provenance of the command selected by THIS tick. These are the
// original wire stamps, never diagnostic receipt or publication timestamps.
struct TaskCommandTiming {
  bool present=false;uint64_t sequence=0;
  using Time=builtin_interfaces::msg::Time;
  Time source=Time(),body_source=Time(),valid_until=Time();
};
struct TaskCommandReorderStats {
  uint64_t deferred=0,completed=0,discarded=0;bool pending=false;
};
// Fixed-size observation of admission attempts, including a deferred command
// retried by its real owner-lease callback. Never used to authorize a command.
struct TaskCommandAdmission {
  uint64_t attempts=0,accepted=0,rejected=0,deferred=0,sequence=0;
  double at=0.;const char* reason="none";
  builtin_interfaces::msg::Time source,body_source,valid_until;
};
// Last expiry fact only, retained for diagnostics after the original command
// is cleared. It cannot be selected, replayed or extend any source deadline.
struct TaskCommandExpiry {
  uint64_t count=0,sequence=0;double observed_at=0.;
  std::string execution_id;
  builtin_interfaces::msg::Time source,body_source,body_valid_until,valid_until;
  uint64_t admission_sequence=0;double admission_at=0.;const char* admission_reason="none";
};
inline double secs(const builtin_interfaces::msg::Time&t){return t.sec+t.nanosec*1e-9;}
inline int64_t sourceNs(const builtin_interfaces::msg::Time&t){return static_cast<int64_t>(t.sec)*1000000000LL+t.nanosec;}
inline builtin_interfaces::msg::Time stamp(double t){builtin_interfaces::msg::Time s;s.sec=static_cast<int32_t>(std::floor(t));s.nanosec=static_cast<uint32_t>((t-s.sec)*1e9);return s;}
inline builtin_interfaces::msg::Time earlier(const builtin_interfaces::msg::Time&a,const builtin_interfaces::msg::Time&b){
  return a.sec<b.sec||(a.sec==b.sec&&a.nanosec<=b.nanosec)?a:b;
}
inline bool fresh(double t,double now,double age){return std::isfinite(t)&&std::isfinite(now)&&t>0&&now>=t&&now-t<=age;}
inline bool canonicalTime(const builtin_interfaces::msg::Time&t){return t.sec>=0&&t.nanosec<1000000000U;}
inline bool task(const Version&a,const Version&b){return a.schema_version==3&&b.schema_version==3&&a.session_id==b.session_id&&
 a.task_id==b.task_id&&a.route_id==b.route_id&&a.route_hash==b.route_hash&&a.map_version_id==b.map_version_id&&
 a.localization_epoch==b.localization_epoch&&a.localization_seed_id==b.localization_seed_id;}
inline bool version(const Version&a,const Version&b){return task(a,b)&&a.reference_generation==b.reference_generation&&
 a.segment_id==b.segment_id&&a.anchor_id==b.anchor_id&&a.anchor_revision==b.anchor_revision&&
 a.context_sequence==b.context_sequence&&a.map_geometry_revision==b.map_geometry_revision;}
inline bool complete(const Version&a){return a.schema_version==3&&!a.session_id.empty()&&!a.task_id.empty()&&
 !a.route_id.empty()&&a.route_hash.size()==64&&!a.map_version_id.empty()&&a.localization_epoch>0&&
 !a.localization_seed_id.empty()&&a.reference_generation>0&&!a.segment_id.empty()&&!a.anchor_id.empty()&&a.anchor_revision>0&&a.context_sequence>0;}
inline bool completeTask(const Version&a){return a.schema_version==3&&!a.session_id.empty()&&!a.task_id.empty()&&
 !a.route_id.empty()&&a.route_hash.size()==64&&a.route_hash.find_first_not_of("0123456789abcdef")==std::string::npos&&
 !a.map_version_id.empty()&&a.localization_epoch>0&&!a.localization_seed_id.empty();}
struct Health {
  bool connected=false,owned=false,general=false,mc_fresh=false,estop=false;
  // Confirmed ownership / the last mode are facts, not renewable telemetry.
  // Missing RobotState is a HOLD; it is not an observed APP takeover or mode
  // switch. Nonzero writes and initial grants still require fresh telemetry.
  bool robot_state_fresh=false;
  // Explicit actual RobotState classification, not a guess from the generic
  // estop bit. Hardware stop/unknown retains the irreversible fault path.
  bool software_estop_only=false;
  std::string motion_mode;
  bool motion_mode_confirmed=false,mode_transition_active=false,mode_zero_hold_allowed=false;
  std::string mode_transition_reason;
  // Fixed actuation mode is independent of a route's terrain segment. The
  // actual GENERAL fact stays truthful; it cannot satisfy fixed STAIR.
  std::string required_motion_mode;
  // Valid runtime parameters are not a physical acceptance certificate.
  bool mode_runtime_parameters_ready=true;
};
inline bool motionModeObserved(const Health&h){return h.required_motion_mode.empty()?
  h.general||h.motion_mode_confirmed:h.motion_mode==h.required_motion_mode&&h.motion_mode_confirmed;}
inline bool motionModeAllowed(const Health&h){return motionModeObserved(h)&&h.mode_runtime_parameters_ready;}
inline bool zeroModeAllowed(const Health&h){return motionModeAllowed(h)||h.mode_zero_hold_allowed;}

// One writer transport. It cannot select goals, replan or grant itself a lease.
// MC raw deltas remain explicit; live proof is labelled approximate, never PTP.
class Transport {
public:
  Transport(std::string session,std::string mode,bool runtime_parameters_valid=false,double max_speed=.3,double max_yaw=.5,std::string model_sha={},std::string kinematic_profile="forward_yaw",std::optional<bool> physically_verified=std::nullopt)
    :session_(std::move(session)),mode_(std::move(mode)),runtime_valid_(runtime_parameters_valid),physically_verified_(runtime_parameters_valid&&physically_verified.value_or(runtime_parameters_valid)),braking_model_sha_(std::move(model_sha)),kinematic_profile_(std::move(kinematic_profile)),max_speed_(max_speed),max_yaw_(max_yaw){
      validateKinematicProfile(kinematic_profile_);
      if(session_.empty()||(mode_!="live"&&mode_!="isolated_mock")||!std::isfinite(max_speed_)||max_speed_<=0||max_speed_>.3||
         !std::isfinite(max_yaw_)||max_yaw_<=0||max_yaw_>.5)throw std::invalid_argument("invalid_execution_transport_profile");}
  void configureBrakingModel(std::string hash,double speed=.3,double yaw=.5,std::string kinematic_profile="forward_yaw") {
    validateKinematicProfile(kinematic_profile);
    if(!execution_.empty()||hash.size()!=64||hash.find_first_not_of("0123456789abcdef")!=std::string::npos||
       !std::isfinite(speed)||speed<=0||speed>.3||!std::isfinite(yaw)||yaw<=0||yaw>.5)
      throw std::invalid_argument("braking_model_must_be_validated_before_execution");
    braking_model_sha_=std::move(hash);max_speed_=speed;max_yaw_=yaw;kinematic_profile_=std::move(kinematic_profile);
  }
  void configureExecutionPolicy(const ExecutionPolicy&policy,double reaction_bound) {
    if(!execution_.empty())throw std::invalid_argument("execution_policy_must_precede_grant");
    policy.validate(reaction_bound);policy_=policy;policy_configured_=true;
  }
  void configureMcCaptureBound(double delay,const std::string&basis) {
    // A host receipt anchor is not an exact acquisition clock. Live requires
    // an explicit profile bound; private mock requires a fixture bound. The
    // declared bound's provenance does not change this runtime time barrier.
    if(!execution_.empty()||!std::isfinite(delay)||delay<0||delay>.25||
       (mode_=="live"&&delay<=0)||basis!=(mode_=="live"?
        "source_delta_host_anchor_approximate":"isolated_simulated_source_clock"))
      throw std::invalid_argument("explicit_mc_capture_time_bound_required");
    mc_delay_bound_=delay;mc_bound_configured_=true;
  }
  // This is a protocol choice for the existing unique writer, not a second
  // transport or another SDK connection. It is immutable after the first
  // task grant. Geometry belongs exclusively to LocalFollower in this mode.
  void configureTaskFollower(bool enabled) {
    if(!execution_.empty())throw std::invalid_argument("writer_protocol_must_precede_grant");
    task_follower_=enabled;
  }
  bool taskFollower()const{return task_follower_;}
  const TaskLease& taskLeaseSnapshot()const{return task_lease_;}
  void beginModeHold(){
    if(!task_follower_||execution_.empty()||stopping_)return;
    // One new request, one original zero/MC barrier. Duplicate request
    // delivery must not keep resetting the evidence accumulation window.
    clearNonterminalStationary();clearTaskCommand();discardPendingTaskCommand();
    task_resume_lease_floor_=permit_seq_;task_leases_.clear();
    permit_.allowed=false;permit_.phase="holding";
    phase_="holding";reason_="mode_transition_pending";
  }
  int64_t taskCommandStopBarrier()const{return task_zero_source_barrier_ns_;}
  // Read-only scheduling hint. It carries ORIGINAL deadlines only and grants
  // no permission: tick() still rechecks every identity, health and time rule.
  // Once expiry clears the latest command this hint disappears, preventing a
  // busy expiration loop. The writer checks strictly after this boundary.
  std::optional<double> taskCommandDeadline()const{
    if(!task_follower_||!task_have_command_||!task_lease_.allowed)return {};
    return std::min({secs(task_command_.valid_until),secs(task_command_.body_valid_until),
      secs(task_lease_.valid_until),secs(task_command_.source_stamp)+commandPipeline()});
  }
  Grant::Response grantResponse(const Grant::Request&r)const {
    Grant::Response out;out.sdk_session=session_;out.sdk_arm_generation=generation_;
    out.version=r.version;out.execution_id=r.execution_id;out.control_epoch=r.control_epoch;
    out.request_id=r.request_id;out.transport_mode=mode_;
    return out;
  }
  Grant::Response rejectGrant(const Grant::Request&r,const std::string&reason)const {
    auto out=grantResponse(r);out.reason=reason;
    // This is a statement about THIS request, not the incumbent. An old
    // unresolved execution can reject a new request without ever installing
    // it. Keep the incumbent and its physical-stop obligation intact.
    out.definitely_not_admitted=r.activate&&(task_follower_?completeTask(r.version):complete(r.version))&&r.sdk_session==session_&&
      r.transport_mode==mode_&&!r.request_id.empty()&&!r.execution_id.empty()&&r.control_epoch>0&&
      r.request_id!=request_&&!seen_.count(r.request_id)&&
      !(r.execution_id==execution_&&task(binding_,r.version));
    return out;
  }
  Grant::Response grant(const Grant::Request&r,double now,const Health&h) {
    auto out=grantResponse(r);
    if(closing_&&r.activate)return rejectGrant(r,"writer_shutting_down");
    if(!(task_follower_?completeTask(r.version):complete(r.version))||r.sdk_session!=session_||r.transport_mode!=mode_||
       !fresh(secs(r.source_stamp),now,.5)||r.request_id.empty()||r.execution_id.empty()||r.control_epoch==0||
       (task_follower_&&r.confirmation_id.empty()))
      return rejectGrant(r,"grant_binding_or_time_invalid");
    if(r.activate&&sourceNs(r.source_stamp)<=software_estop_grant_barrier_ns_)
      return rejectGrant(r,"grant_precedes_software_estop_barrier");
    if(!r.activate){
      if(r.execution_id!=execution_||r.control_epoch!=epoch_){out.reason="stale_revoke";return out;}
      stop("owner_revoked",now);out.accepted=true;out.reason="stopping_not_yet_measured";return out;
    }
    if(r.request_id==request_) {
      out.accepted=task(binding_,r.version)&&r.execution_id==execution_&&r.control_epoch==epoch_&&
        !fault_&&!software_estop_latched_&&!stopping_;
      out.duplicate=true;out.reason=out.accepted?"same_grant_transaction":"request_id_conflict";return out;
    }
    if(mode_=="live"&&!runtime_valid_)return rejectGrant(r,"runtime_motion_profile_unavailable");
    if(!mc_bound_configured_)return rejectGrant(r,"mc_capture_time_bound_unavailable");
    if(!policy_configured_)return rejectGrant(r,"execution_policy_unavailable");
    // Rejections before binding have no side effects and no request-ID cache.
    // Keep unresolved incumbents distinct from temporarily missing telemetry:
    // the BT may retry the latter, but cannot erase the former's stop duty.
    if(!h.connected)return rejectGrant(r,"writer_disconnected");
    tryCompleteSoftwareEstopRelease(now,h);
    if(fault_)return rejectGrant(r,"writer_fault_latched");
    if(software_estop_latched_)return rejectGrant(r,software_estop_release_confirmed_?
        "software_estop_release_waiting_prior_retirement":"software_estop_latched_rearm_required");
    if(!execution_.empty()&&sdk_transactions_tracked_&&
       (pending_sdk_transactions_>0||sdk_transaction_uncertain_))
      return rejectGrant(r,"prior_sdk_transactions_unretired");
    if(!execution_.empty()&&!terminalStopSatisfied(now)&&!noMotionRetired())
      return rejectGrant(r,"prior_stop_unconfirmed");
    if(!h.robot_state_fresh)return rejectGrant(r,"robot_state_unavailable");
    if(h.estop)return rejectGrant(r,"robot_estop_active");
    if(!motionModeAllowed(h))return rejectGrant(r,h.mode_transition_reason.empty()?"robot_mode_unsupported":h.mode_transition_reason);
    if(!h.mc_fresh)return rejectGrant(r,"mc_capture_unavailable");
    // Control epoch belongs to the navigation session; SDK arm generation
    // remains the writer-wide monotonically increasing identity. A new BT
    // process starts its own epoch at one, never resurrecting a retired one.
    if(seen_.count(r.request_id)||retired_sessions_.count(r.version.session_id)||
       (r.version.session_id==binding_.session_id&&r.control_epoch<=epoch_)){
      out.reason="grant_replay";return out;}
    // Keep the complete bounded request-ID replay ledger. Exhaustion is a
    // terminal resource condition requiring a new explicitly bound SDK
    // session, not evidence that this fresh request was a replay.
    if(seen_.size()>=1024){out.reason="grant_request_ledger_exhausted_new_sdk_session_required";return out;}
    if(last_local_envelope_&&!localContext(*last_local_envelope_,r.version)) {
      local_.reset();last_local_envelope_.reset();local_history_.clear();
    }
    if(!binding_.session_id.empty()&&binding_.session_id!=r.version.session_id)
      retired_sessions_.insert(binding_.session_id);
    binding_=r.version;execution_=r.execution_id;epoch_=r.control_epoch;request_=r.request_id;seen_.insert(request_);
    task_feedback_.reset();
    confirmation_=r.confirmation_id;
    // Preview content may precede this grant. Keep its immutable geometry,
    // but never carry other tasks' cache entries into the admitted task.
    geometry_.erase(std::remove_if(geometry_.begin(),geometry_.end(),[this](const auto&g){
      return !task(binding_,taggedVersion(g.message));}),geometry_.end());
    terminal_stop_.reset();
    ++generation_;permit_seq_=demand_seq_=0;started_=last_tick_=now;permit_at_=demand_at_=0;stopping_=stopped_=false;
    stop_submitted_=false;first_stationary_=0;stationary_samples_=0;have_demand_=ever_nonzero_=mc_hold_=robot_hold_=false;
    mc_resume_floor_=robot_resume_floor_=0.;
    permits_.clear();motion_proofs_.clear();pending_demand_.reset();permit_=Permit{};demand_=Demand{};
    task_lease_=TaskLease{};task_command_=LocalCommand{};task_leases_.clear();
    task_have_command_=task_selected_=false;selected_task_timing_=TaskCommandTiming{};discardPendingTaskCommand();
    task_resume_lease_floor_=0;task_command_source_floor_=task_body_source_floor_=0;task_zero_source_barrier_ns_=0;
    handoff_.reset();prepared_.reset();active_entry_.reset();last_handoff_.reset();last_ack_.reset();commit_sequence_=handoff_sequence_=ack_sequence_=prepared_sequence_floor_=0;
    selected_prepared_=selected_active_=ack_dirty_=false;applied_version_=Version{};applied_trajectory_id_=0;
    retained_incumbent_until_=0.;curve_proofs_.clear();progress_.reset();
    clearNonterminalStationary();write_sequence_=last_nonzero_write_sequence_=stationary_sequence_=0;
    mc_order_hold_=false;mc_order_new_samples_=0;mc_order_first_source_=mc_order_barrier_source_=0.;
    takeover_=!h.owned;takeover_taken_=false;ready_=h.owned;ready_at_=ready_?now:0.;phase_=ready_?"ready":"taking_control";reason_.clear();
    out.accepted=true;out.sdk_arm_generation=generation_;out.reason="grant_queued";return out;
  }
  bool takeControlOnce(){if(!takeover_||takeover_taken_||stopping_||closing_)return false;takeover_taken_=true;return true;}
  // Called under the writer mutex using the actual network transaction ledger
  // and correlated ownership FSM, never a guess from an empty command queue.
  void setSDKTransactions(uint32_t pending,bool uncertain){
    sdk_transactions_tracked_=true;pending_sdk_transactions_=pending;sdk_transaction_uncertain_=uncertain;
  }
  // Final SDK input classification is separate from the SI demand. A mapper
  // may send zero for a nonzero demand during startup, reversal or float cast.
  // Record before Move: a failed nonzero call still has an uncertain outcome.
  void moveAttempted(bool actual_sdk_nonzero){
    if(actual_sdk_nonzero)invalidateTerminalStopForNonzero();
    ever_nonzero_=ever_nonzero_||actual_sdk_nonzero;
  }
  void moveAttempted(const geometry_msgs::msg::Twist&command){moveAttempted(planarMotionNonzero(command));}
  bool motionNeverSubmitted()const{return !ever_nonzero_;}
  bool noMotionRetired()const{return !execution_.empty()&&stopping_&&sdk_transactions_tracked_&&
    !ever_nonzero_&&pending_sdk_transactions_==0&&!sdk_transaction_uncertain_;}
  // Explicit manual handoff finishes the old navigation execution before
  // posture/manual movement. It does not reset the replay ledger, control
  // epoch, SDK fault or software estop. The caller retains its final reports.
  bool completeManualHandoff(double now){
    if(fault_||software_estop_latched_||closing_||!sdk_transactions_tracked_||
       pending_sdk_transactions_||sdk_transaction_uncertain_)return false;
    if(execution_.empty())return true;
    if(!stopping_||(!noMotionRetired()&&!terminalStopSatisfied(now)))return false;
    execution_.clear();takeover_=takeover_taken_=ready_=false;stopping_=false;
    terminal_stop_.reset();
    phase_="idle";reason_="manual_handoff_completed";return true;
  }
  void submissionFailed(double now){fault_=true;stop("sdk_write_outcome_unknown",now);}
  void fault(const std::string&why,double now){fault_=true;stop(why,now);}
  // Fence old work BEFORE SDK(true). Releasing this operator latch never
  // erases a hardware/SDK/ownership fault or reactivates destroyed old leases.
  void softwareEstopRequested(double now){latchSoftwareEstop("software_estop_requested",now);}
  void softwareEstopObserved(double now){latchSoftwareEstop("software_estop_observed",now);}
  bool confirmSoftwareEstopRelease(double now) {
    if(!software_estop_latched_||fault_||closing_||!std::isfinite(now)||now<software_estop_requested_at_)return false;
    // Caller supplies matching explicit OFF + new RobotState confirmation.
    // An active estop blocks Move, so wait for the old zero ACK AFTER OFF.
    software_estop_release_confirmed_=true;
    software_estop_grant_barrier_ns_=std::max(software_estop_grant_barrier_ns_,sourceNs(stamp(now)));
    return true;
  }
  bool softwareEstopLatched()const{return software_estop_latched_;}
  bool softwareEstopReleasePending()const{return software_estop_latched_&&software_estop_release_confirmed_;}
  void telemetryRestartBeforeFirstMotion(double now) {
    if(ever_nonzero_){fault("mc_reconfigured_after_motion",now);return;}
    clock_epoch_.clear();last_raw_=0;last_source_=last_received_=0;ready_=false;ready_at_=0;
  }
  bool taskLease(const TaskLease&p,double now) {
    if(!task_follower_||p.schema_version!=1||execution_.empty()||stopping_||!completeTask(p.version)||
       !task(binding_,p.version)||p.execution_id!=execution_||p.control_epoch!=epoch_||
       p.sdk_session!=session_||p.sdk_arm_generation!=generation_||p.transport_mode!=mode_||
       p.confirmation_id!=confirmation_||!canonicalTime(p.source_stamp)||!canonicalTime(p.valid_until)||
       p.sequence<=permit_seq_||!fresh(secs(p.source_stamp),now,.35)||
       sourceNs(p.valid_until)<=sourceNs(p.source_stamp)||secs(p.valid_until)<now||
       sourceNs(p.valid_until)-sourceNs(p.source_stamp)>350000000LL)return false;
    permit_seq_=p.sequence;permit_at_=now;
    if(p.revoked||p.phase=="stopping"||p.phase=="terminal") {
      stop(p.reason.empty()?"owner_revoked":p.reason,now);return true;
    }
    // Explicit owner HOLD and missing data do not create a new task. They do
    // fence every old nonzero command: recovery needs a new lease and source.
    if(!p.allowed){clearTaskCommand();discardPendingTaskCommand();task_resume_lease_floor_=p.sequence;task_leases_.clear();}
    task_lease_=p;
    if(p.allowed&&!mc_hold_&&!robot_hold_&&secs(p.source_stamp)>mc_resume_floor_&&
       secs(p.source_stamp)>robot_resume_floor_){
      task_leases_.push_back(p);if(task_leases_.size()>8)task_leases_.pop_front();
    }
    // The legacy envelope is retained only for compatible SDK state and stop
    // reporting. No reference/anchor/curve field has authority in this mode.
    permit_=Permit{};permit_.version=binding_;permit_.execution_id=execution_;permit_.control_epoch=epoch_;
    permit_.sdk_session=session_;permit_.sdk_arm_generation=generation_;permit_.source_stamp=p.source_stamp;
    permit_.valid_until=p.valid_until;permit_.sequence=p.sequence;permit_.allowed=p.allowed;
    permit_.revoked=p.revoked;permit_.phase=p.phase;permit_.reason=p.reason;permit_.transport_mode=mode_;
    // BT leases and Native commands use independent ROS publications. A
    // command can legitimately arrive first even though Native had already
    // observed this original causal lease. Re-run FULL admission now; do not
    // restamp it, invent an allowed lease, or replay a retired task's command.
    if(p.allowed&&pending_task_command_&&pending_task_command_->lease_sequence==p.sequence){
      const auto waiting=*pending_task_command_;pending_task_command_.reset();
      if(localCommand(waiting,now))++task_command_reorder_completed_;
      else ++task_command_reorder_discarded_;
    }
    return true;
  }
  bool localCommand(const LocalCommand&c,double now) {
    ++task_command_admission_.attempts;task_command_admission_.sequence=c.sequence;
    task_command_admission_.at=now;task_command_admission_.source=c.source_stamp;
    task_command_admission_.body_source=c.body_source_stamp;task_command_admission_.valid_until=c.valid_until;
    const bool accepted=admitLocalCommand(c,now);
    if(accepted)++task_command_admission_.accepted;
    else if(task_command_admission_.reason==std::string_view("lease_deferred"))++task_command_admission_.deferred;
    else ++task_command_admission_.rejected;
    return accepted;
  }
  TaskCommandAdmission taskCommandAdmission()const{return task_command_admission_;}
private:
  bool admitLocalCommand(const LocalCommand&c,double now) {
    const auto reject=[this](const char*why){task_command_admission_.reason=why;return false;};
    if(!task_follower_||c.schema_version!=1||execution_.empty()||stopping_||!completeTask(c.version)||
       !task(binding_,c.version)||c.execution_id!=execution_||c.control_epoch!=epoch_||
       c.sdk_session!=session_||c.sdk_arm_generation!=generation_||c.transport_mode!=mode_)
      return reject("task_identity_or_retired");
    const bool evidence_update=c.sequence>0&&c.sequence==demand_seq_;
    if(c.sequence<demand_seq_||c.sequence==0||c.lease_sequence<=task_resume_lease_floor_)
      return reject("sequence_or_lease_floor");
    if(evidence_update){
      // Native may renew a still-live environmental verdict BETWEEN 20 Hz
      // controller samples. This is the identical original command, not a new
      // velocity or a fresh source. Expired/zero/retired commands cannot return.
      if(!task_have_command_||!task_original_command_||task_command_.hold||
         !planarMotionNonzero(task_command_.velocity)||secs(task_command_.valid_until)<=now||
         sourceNs(c.valid_until)<=sourceNs(task_command_.valid_until))return reject("evidence_update_not_live");
      auto original=c;original.valid_until=task_original_command_->valid_until;
      if(original!=*task_original_command_)return reject("evidence_update_changed_intent");
    }
    if(c.body_frame!="d1max_loc_base_link")return reject("body_frame");
    if(!taskCommandTime(c,now))return reject("original_time_contract");
    if((!evidence_update&&sourceNs(c.source_stamp)<=task_command_source_floor_)||
       sourceNs(c.body_source_stamp)<task_body_source_floor_)
      return reject("source_order");
    if(!twistFinite(c.velocity)||c.velocity.linear.z!=0.||c.velocity.angular.x!=0.||c.velocity.angular.y!=0.||
       (kinematic_profile_!="holonomic_xyyaw"&&(c.velocity.linear.x<0.||c.velocity.linear.y!=0.))||
       (c.hold&&planarMotionNonzero(c.velocity)))return reject("velocity_contract");
    if(mc_hold_||robot_hold_||secs(c.source_stamp)<=mc_resume_floor_||secs(c.source_stamp)<=robot_resume_floor_)
      return reject("telemetry_hold_or_resume_floor");
    // A newer explicit zero/HOLD is a stop fact, not a future permission.
    // It retires older pending/nonzero work even if its referenced lease is
    // still in transit. Invalid/stale/foreign packets above have no effect.
    const bool zero=!planarMotionNonzero(c.velocity);
    // Feedback is a sending observation, not additional motion authority.
    // Native evidence and these original source/identity/zero barriers still
    // decide admission even when a feedback packet is late or lost.
    const auto command_source=sourceNs(c.source_stamp);
    if(!zero&&command_source<=task_zero_source_barrier_ns_)return reject("zero_source_barrier");
    if(zero){
      if(command_source<task_zero_source_barrier_ns_)return reject("zero_source_barrier");
      // An unknown lease must not poison accepted sequence floors, but the
      // already observed zero has its OWN source-time retirement barrier.
      // Otherwise an older delayed positive on an existing lease could revive
      // motion while this zero's new lease is still crossing the network.
      task_zero_source_barrier_ns_=command_source;
      if(pending_task_command_&&sourceNs(pending_task_command_->source_stamp)<=command_source)
        discardPendingTaskCommand();
      if(task_have_command_&&sourceNs(task_command_.source_stamp)<=command_source)clearTaskCommand();
    }
    const bool current_lease=task_lease_.allowed&&fresh(secs(task_lease_.source_stamp),now,.35)&&
      secs(task_lease_.valid_until)>=now;
    if(!current_lease){
      // Only the first lease may be unknown. An explicit owner HOLD, revoke,
      // or expired known owner remains a real barrier, not a reorder guess.
      if(permit_seq_==0&&c.lease_sequence>0)return deferTaskCommand(c);
      return reject("owner_lease_unavailable");
    }
    const auto lease=std::find_if(task_leases_.begin(),task_leases_.end(),[&](const TaskLease&p){
      return p.sequence==c.lease_sequence&&p.allowed&&!p.revoked&&
        fresh(secs(p.source_stamp),now,.35)&&secs(p.valid_until)>=now&&
        sourceNs(c.source_stamp)>=sourceNs(p.source_stamp);
    });
    if(lease==task_leases_.end()){
      if(c.lease_sequence>permit_seq_)return deferTaskCommand(c);
      return reject("causal_owner_lease_missing");
    }
    if(evidence_update){
      task_command_.valid_until=c.valid_until;task_original_command_->valid_until=c.valid_until;
      demand_.valid_until=c.valid_until;
      task_command_admission_.reason="evidence_updated";
      return true;
    }
    demand_seq_=c.sequence;task_command_source_floor_=sourceNs(c.source_stamp);task_body_source_floor_=sourceNs(c.body_source_stamp);
    task_original_command_=c;
    task_command_=c;
    // Final physical-unit saturation is the writer's responsibility. A
    // finite oversized request is limited, not turned into a task failure.
    const auto speed=std::hypot(c.velocity.linear.x,c.velocity.linear.y);
    if(speed>max_speed_){task_command_.velocity.linear.x*=max_speed_/speed;task_command_.velocity.linear.y*=max_speed_/speed;}
    task_command_.velocity.angular.z=std::clamp(c.velocity.angular.z,-max_yaw_,max_yaw_);
    task_have_command_=true;
    if(pending_task_command_&&(pending_task_command_->sequence<=c.sequence||
       sourceNs(pending_task_command_->source_stamp)<=sourceNs(c.source_stamp)))discardPendingTaskCommand();
    demand_=Demand{};demand_.version=binding_;demand_.execution_id=execution_;demand_.control_epoch=epoch_;
    demand_.sdk_session=session_;demand_.sdk_arm_generation=generation_;demand_.transport_mode=mode_;
    demand_.sequence=c.sequence;demand_.permit_sequence=c.lease_sequence;demand_.trajectory_id=c.trajectory_id;
    demand_.source_stamp=c.source_stamp;demand_.body_source_stamp=c.body_source_stamp;demand_.valid_until=c.valid_until;
    demand_.velocity=task_command_.velocity;demand_.hold=c.hold;
    task_command_admission_.reason="accepted";
    return true;
  }
public:
  bool permit(const Permit&p,double now) {
    if(task_follower_)return false;
    if(execution_.empty()||p.execution_id!=execution_||p.control_epoch!=epoch_||!task(binding_,p.version)||
       p.transport_mode!=mode_||p.sdk_session!=session_||p.sdk_arm_generation!=generation_||
       p.sequence<=permit_seq_||!fresh(secs(p.source_stamp),now,.35)||
       secs(p.valid_until)<now||secs(p.valid_until)-secs(p.source_stamp)>.35||
       (p.allowed&&!p.geometry_committed))return false;
    // A pre-gap permission must not revive a pre-gap command. Revocations
    // and HOLD heartbeats still reach the owner even while MC is unavailable.
    if(p.allowed&&!p.revoked&&p.phase!="stopping"&&p.phase!="terminal"&&
       (mc_hold_||robot_hold_||secs(p.source_stamp)<=mc_resume_floor_||
        secs(p.source_stamp)<=robot_resume_floor_))return false;
    // After an irreversible writer CAS, an overtaken incumbent heartbeat is
    // not a rollback instruction. Task-wide HOLD/revoke still reach us.
    if(commit_sequence_>0&&p.allowed&&!p.revoked&&p.phase!="stopping"&&p.phase!="terminal"&&
       (!version(p.version,applied_version_)||p.trajectory_id!=applied_trajectory_id_))return false;
    // A stationary re-entry is deliberately holding the unsafe incumbent.
    // An allowed heartbeat cannot accidentally restart it during this CAS.
    if(handoff_&&handoff_->transition_mode==Handoff::STATIONARY_REENTRY&&p.allowed&&!p.revoked&&
       p.phase!="stopping"&&p.phase!="terminal")return false;
    permit_seq_=p.sequence;permit_at_=now;
    if(p.revoked||p.phase=="stopping"||p.phase=="terminal"){stop(p.reason.empty()?"owner_revoked":p.reason,now);return true;}
    if(stopping_)return false;
    // A lease heartbeat is not a new trajectory. Preserve a command only
    // inside its original proof lease and unchanged execution phase/geometry;
    // an anchor/curve commit or tracking -> final-yaw transition retires it.
    const bool stationary_wait=handoff_&&handoff_->transition_mode==Handoff::STATIONARY_REENTRY&&
      !p.allowed&&p.phase=="holding"&&version(p.version,applied_version_)&&p.trajectory_id==applied_trajectory_id_;
    if((!p.allowed||permit_.phase!=p.phase)&&!stationary_wait)rejectHandoff("owner_hold_or_phase_barrier",now);
    if(!version(permit_.version,p.version)||permit_.trajectory_id!=p.trajectory_id||
       permit_.phase!=p.phase||!p.allowed){have_demand_=false;permits_.clear();pending_demand_.reset();}
    permit_=p;
    if(p.allowed&&!handoff_)retained_incumbent_until_=0.;
    if(!p.allowed){have_demand_=false;phase_="holding";}
    else {permits_.push_back(p);if(permits_.size()>8)permits_.pop_front();}
    return true;
  }
  bool localState(const Local&m,double now) {
    if(task_follower_)return false;
    if(m.schema_version!=2||m.session_id.empty()||m.map_version_id.empty()||m.localization_epoch==0||m.localization_seed_id.empty())return false;
    if(!execution_.empty()&&!localContext(m,binding_))return false;
    if(!fresh(secs(m.source_stamp),now,.1))return false;
    const bool same_context=last_local_envelope_&&localContext(m,*last_local_envelope_);
    if(same_context&&sourceNs(m.source_stamp)<=sourceNs(last_local_envelope_->source_stamp))return false;
    // An unavailable envelope is an ordered event, not permission for a
    // delayed callback to erase newer localization. Retain its source floor
    // after removing the pose, and never interpolate across an outage.
    if(!m.usable){last_local_envelope_=m;local_.reset();local_history_.clear();return false;}
    const auto&o=m.local_odometry;
    if(!localProviderLease(m,now)||!fresh(secs(m.posterior_stamp),secs(m.source_stamp),.4)||
       !fresh(secs(m.imu_stamp),secs(m.source_stamp),.1)||
       secs(m.posterior_stamp)>secs(m.source_stamp)||secs(m.imu_stamp)>secs(m.source_stamp)||
       !std::isfinite(m.extrapolation_sec)||m.extrapolation_sec<0||m.extrapolation_sec>.1||
       std::abs(secs(m.source_stamp)-secs(m.imu_stamp)-m.extrapolation_sec)>1e-5||o.header.stamp!=m.source_stamp||
       o.header.frame_id!="d1max_loc_odom"||o.child_frame_id!="d1max_loc_base_link"||!poseFinite(o.pose.pose)||!twistFinite(o.twist.twist))return false;
    if(!same_context)local_history_.clear();
    local_=m;last_local_envelope_=m;local_history_.push_back(m);
    while(local_history_.size()>64||(!local_history_.empty()&&
        sourceNs(m.source_stamp)-sourceNs(local_history_.front().source_stamp)>1000000000LL))local_history_.pop_front();
    return true;
  }
  bool trajectoryGeometry(const Tagged&m,double now) {
    if(task_follower_)return false;
    (void)now; // Geometry is immutable content, not a renewable authority lease.
    if(m.frame_id!="d1max_loc_odom"||m.segment_kind!="floor"||m.required_mode!="general")return false;
    if(closing_)return false;
    // Content-only pre-cache: the next task's preview can arrive while the
    // previous execution is stopping. Every prepared command still needs
    // the exact admitted task/version, permit and live native evidence.
    // Repeated publication is an immutable-content check, not another curve
    // acquisition. Do not rebuild the bounded arc table on every heartbeat.
    for(const auto&g:geometry_)if(g.message.trajectory.traj_id==m.trajectory.traj_id&&
        g.cache.versionMatches(taggedVersion(m)))return g.cache.matches(m);
    auto built=MeasuredCurveEntryCache::build(m);if(!built)return false;
    if(geometry_.size()==8)geometry_.pop_front();
    geometry_.push_back({m,std::move(*built)});return true;
  }
  bool trajectoryValidation(const CurveProof&p,double now) {
    if(task_follower_)return false;
    if(!complete(p.version)||!task(binding_,p.version)||p.transport_mode!=mode_||p.sequence==0||
       !fresh(secs(p.check_end),now,.25)||secs(p.check_begin)>secs(p.check_end))return false;
    for(const auto&old:curve_proofs_)if(version(old.version,p.version)&&old.trajectory_id==p.trajectory_id&&
        p.sequence<=old.sequence&&(p.valid||p.sequence==old.sequence))return false;
    curve_proofs_.push_back(p);if(curve_proofs_.size()>32)curve_proofs_.pop_front();return true;
  }
  bool trackingProgress(const Progress&p,double now) {
    if(task_follower_)return false;
    if(p.schema_version!=2||!p.valid||p.header.frame_id!="d1max_loc_odom"||!fresh(secs(p.header.stamp),now,.1)||
       p.session_id!=binding_.session_id||p.task_id!=binding_.task_id||p.route_id!=binding_.route_id||
       p.route_hash!=binding_.route_hash||p.map_version_id!=binding_.map_version_id||p.localization_epoch!=binding_.localization_epoch||
       p.localization_seed_id!=binding_.localization_seed_id||!poseFinite(p.pose)||!twistFinite(p.twist)||
       !std::isfinite(p.curve_time)||p.curve_time<0||!std::isfinite(p.arc_length)||p.arc_length<0)return false;
    if(progress_&&secs(p.header.stamp)<=secs(progress_->header.stamp))return false;
    progress_=p;return true;
  }
  bool handoff(const Handoff&g,double now) {
    if(task_follower_)return false;
    if(g.revoked&&last_handoff_&&g.handoff_id==last_handoff_->handoff_id) {
      auto original=g;original.revoked=last_handoff_->revoked;original.reason=last_handoff_->reason;
      if(original!=*last_handoff_)return false;
      rejectHandoff(g.reason.empty()?"handoff_revoked":g.reason,now);
      // A delayed revoke after the call cannot roll the software identity
      // back. It still blocks motion immediately, even before ordinary HOLD.
      have_demand_=false;permits_.clear();permit_.allowed=false;phase_="holding";
      if(last_ack_&&last_ack_->handoff_id==g.handoff_id)ack_dirty_=true;
      return true;
    }
    if(last_handoff_&&g.handoff_id==last_handoff_->handoff_id) {
      if(g!=*last_handoff_)return false;
      if(last_ack_&&last_ack_->handoff_id==g.handoff_id)ack_dirty_=true;
      return true; // identical transaction does not renew any time budget
    }
    const bool reentry=g.transition_mode==Handoff::STATIONARY_REENTRY;
    const bool continuous=g.transition_mode==Handoff::CONTINUOUS_REPLACE;
    if(closing_||stopping_||mc_hold_||robot_hold_||!ready_||!last_health_.robot_state_fresh||!last_health_.mc_fresh||g.schema_version!=2||(!reentry&&!continuous)||g.handoff_id.empty()||
       g.handoff_id.size()>128||g.sequence<=handoff_sequence_||g.expected_commit_sequence!=commit_sequence_||commit_sequence_==0||
       !sameGrantIdentity(g.incumbent)||!sameGrantIdentity(g.candidate)||!version(g.incumbent.version,applied_version_)||
       g.incumbent.trajectory_id!=applied_trajectory_id_||!version(g.incumbent.version,permit_.version)||
       g.incumbent.trajectory_id!=permit_.trajectory_id||permit_.revoked||g.incumbent.revoked||
       (continuous&&(!permit_.allowed||permit_.phase!="tracking"||!g.incumbent.allowed||g.incumbent.phase!="tracking"))||
       (reentry&&(permit_.allowed||permit_.phase!="holding"||g.incumbent.allowed||g.incumbent.phase!="holding"||
          g.retain_incumbent_until!=g.source_stamp||secs(g.valid_until)>secs(g.stationary_evidence.valid_until)||
          secs(g.candidate.valid_until)>secs(g.stationary_evidence.valid_until)||!stationaryProof(g.stationary_evidence,now)))||
       !g.candidate.allowed||g.candidate.revoked||g.candidate.phase!="tracking"||
       g.candidate.sequence<=g.incumbent.sequence||g.candidate.validation_sequence==0||
       !fresh(secs(g.source_stamp),now,.25)||secs(g.valid_until)<=now||secs(g.valid_until)-secs(g.source_stamp)>.250001||
       secs(g.transition_deadline)>secs(g.valid_until)||secs(g.transition_deadline)<=now||
       (continuous&&(secs(g.retain_incumbent_until)>secs(g.incumbent.valid_until)||
         secs(g.retain_incumbent_until)>secs(permit_.valid_until)||secs(g.retain_incumbent_until)<=secs(g.source_stamp)))||
       !fresh(secs(g.candidate.source_stamp),now,.35)||secs(g.candidate.valid_until)<=now||
       secs(g.candidate.valid_until)-secs(g.candidate.source_stamp)>.35)return false;
    if(g.revoked){rejectHandoff("handoff_revoked",now);return false;}
    if(handoff_)return false; // fixed active + one prepared slot, no queue
    handoff_sequence_=g.sequence;last_handoff_=g;handoff_=g;prepared_.reset();selected_prepared_=false;
    // Incumbent and prepared commands are independent roles during this
    // fixed CAS transaction. An incumbent command arriving while native
    // checks the candidate must not make that candidate a replay. Freeze
    // the admission floor here; prepared_ enforces its own monotonic order.
    prepared_sequence_floor_=demand_seq_;
    retained_incumbent_until_=continuous?secs(g.retain_incumbent_until):0.;
    return true;
  }
  bool preparedDemand(const Prepared&p,double now) {
    if(task_follower_)return false;
    if(!handoff_||p.schema_version!=1||p.handoff_id!=handoff_->handoff_id||p.grant_sequence!=handoff_->sequence||
       p.expected_commit_sequence!=commit_sequence_||p.expected_commit_sequence!=handoff_->expected_commit_sequence||
       secs(handoff_->transition_deadline)<=now||!sameGrantIdentity(handoff_->candidate)||
       !version(p.demand.version,handoff_->candidate.version)||p.demand.trajectory_id!=handoff_->candidate.trajectory_id||
       p.demand.permit_sequence!=handoff_->candidate.sequence||p.demand.sequence<=prepared_sequence_floor_||!demandBinding(p.demand)||
       !commandSourceFresh(p.demand.source_stamp,now)||!commandSourceFresh(p.demand.body_source_stamp,now)||
       secs(p.demand.valid_until)<=now||secs(p.demand.valid_until)>secs(p.demand.source_stamp)+commandPipeline()+.000001||
       !preparedEntry(p,now))return false;
    if(prepared_&&p.demand.sequence<prepared_->demand.sequence)return false;
    if(prepared_&&p.demand.sequence==prepared_->demand.sequence&&
       (!sameOriginalDemand(p.demand,prepared_->demand)||p.entry_admission!=prepared_->entry_admission||
        p.measured_pose!=prepared_->measured_pose||p.measured_twist!=prepared_->measured_twist||
        p.entry_source_stamp!=prepared_->entry_source_stamp||p.curve_entry_pose!=prepared_->curve_entry_pose||
        p.curve_entry_twist!=prepared_->curve_entry_twist||p.curve_time!=prepared_->curve_time))return false;
    prepared_=p;return true;
  }
  bool demand(const Demand&d,double now) {
    if(task_follower_)return false;
    if(closing_||!ready_||stopping_||mc_hold_||robot_hold_||!last_health_.robot_state_fresh||!last_health_.mc_fresh||!permit_.allowed||
       secs(d.source_stamp)<=mc_resume_floor_||secs(d.body_source_stamp)<=mc_resume_floor_||
       secs(d.source_stamp)<=robot_resume_floor_||secs(d.body_source_stamp)<=robot_resume_floor_||
       !version(permit_.version,d.version)||d.execution_id!=execution_||
       d.control_epoch!=epoch_||d.sdk_session!=session_||d.sdk_arm_generation!=generation_||d.transport_mode!=mode_||
       d.sequence<demand_seq_||d.trajectory_id!=permit_.trajectory_id||!commandSourceFresh(d.source_stamp,now)||
       !commandSourceFresh(d.body_source_stamp,now)||secs(d.valid_until)<now||secs(d.valid_until)-secs(d.source_stamp)>.5)return false;
    const bool evidence_upgrade=demand_seq_>0&&d.sequence==demand_seq_;
    // A second independent sweep may improve only the evidence binding of
    // this exact original command. It is not a new motion request: source,
    // body source, velocity, owner lease and original expiry stay immutable.
    // In particular, a zero/HOLD or an invalid native verdict cannot be
    // undone by replaying the old sequence with a later positive proof.
    if(evidence_upgrade&&(!sameOriginalDemand(d,demand_)||
       d.motion_validation_sequence<=demand_.motion_validation_sequence||
       invalidated_accepted_demand_||d.hold||
       !planarMotionNonzero(d.velocity)))return false;
    // Same-curve callback ordering may deliver heartbeat N+1 before a fully
    // checked demand for N. Accept only its exact original still-live lease;
    // phase/geometry/HOLD/revoke transitions have already cleared this history.
    const Permit* original=nullptr;
    for(const auto& p:permits_)if(p.sequence==d.permit_sequence)original=&p;
    if(!original||!original->allowed||original->revoked||!version(original->version,d.version)||
       original->trajectory_id!=d.trajectory_id||original->phase!=permit_.phase||
       // The owner's sequence is the signing floor, not a pin to an ageing
       // collision snapshot. The exact newer proof is still required by the
       // actual-demand MotionValidation below; this never renews its deadline.
       d.validation_sequence<original->validation_sequence||
       !fresh(secs(original->source_stamp),now,.35)||secs(original->valid_until)<now)return false;
    const auto&v=d.velocity;
    if(!velocityAllowed(v))return false;
    const bool nonzero=planarMotionNonzero(v);
    const auto* motion=motionProof(d,now);
    if(nonzero&&(d.hold||!d.safety_checked))return false;
    if(nonzero&&!motion) {
      // Proof and the safety gate are distinct publishers. Buffer only one
      // already-bound demand; late proof rechecks every original lease below.
      if(!pending_demand_||d.sequence>pending_demand_->sequence)pending_demand_=d;
      return false;
    }
    if(!evidence_upgrade)invalidated_accepted_demand_=false;
    demand_=d;demand_seq_=d.sequence;
    if(!evidence_upgrade)active_entry_.reset();
    if(!evidence_upgrade)demand_at_=now;
    demand_lease_until_stamp_=earlier(earlier(d.valid_until,original->valid_until),
      earlier(commandDeadline(d.source_stamp),commandDeadline(d.body_source_stamp)));
    if(nonzero)demand_lease_until_stamp_=earlier(demand_lease_until_stamp_,motion->valid_until);
    demand_lease_until_=secs(demand_lease_until_stamp_);
    have_demand_=true;pending_demand_.reset();return true;
  }
  bool motionValidation(const MotionProof&p,double now) {
    if(task_follower_)return false;
    if(execution_.empty()||!task(binding_,p.version)||!complete(p.version)||p.execution_id!=execution_||
       p.control_epoch!=epoch_||p.sdk_session!=session_||p.sdk_arm_generation!=generation_||p.transport_mode!=mode_||
       p.sequence==0||p.demand_sequence==0||!fresh(secs(p.check_end),now,.15)||secs(p.check_begin)>secs(p.check_end)||
       p.braking_model_sha256!=braking_model_sha_||braking_model_sha_.size()!=64)return false;
    for(const auto&old:motion_proofs_)if(version(old.version,p.version)&&old.demand_sequence==p.demand_sequence&&p.sequence<=old.sequence)return false;
    if(!p.valid&&version(p.version,demand_.version)&&p.demand_sequence==demand_.sequence){
      have_demand_=false;invalidated_accepted_demand_=true;
    }
    if(!p.valid&&pending_demand_&&version(p.version,pending_demand_->version)&&p.demand_sequence==pending_demand_->sequence)pending_demand_.reset();
    motion_proofs_.push_back(p);if(motion_proofs_.size()>32)motion_proofs_.pop_front();
    if(p.valid&&pending_demand_){const auto waiting=*pending_demand_;demand(waiting,now);}return true;
  }
  std::optional<geometry_msgs::msg::Twist> tick(double now,const Health&h) {
    task_selected_=false;selected_task_timing_=TaskCommandTiming{};
    if(pending_task_command_&&!taskCommandTime(*pending_task_command_,now))discardPendingTaskCommand();
    selected_active_=selected_prepared_=false;
    selected_curve_until_=selected_motion_until_={};
    last_health_=h;
    if(software_estop_latched_&&!execution_.empty()&&software_estop_had_owned_&&!h.owned)
      fault("control_lost_during_software_estop",now);
    tryCompleteSoftwareEstopRelease(now,h);
    if(execution_.empty())return {};
    if(noMotionRetired()){phase_="retired_without_motion";return {};}
    if(last_tick_>0&&(now<last_tick_||now-last_tick_>.3)){fault_=true;stop("writer_clock_or_executor_gap",now);}last_tick_=now;
    if(!h.connected||(h.estop&&!(software_estop_latched_&&h.software_estop_only))||
       (!motionModeAllowed(h)&&!(task_follower_&&h.mode_transition_active&&zeroModeAllowed(h)))){
      fault_=true;stop("robot_state_or_connection_lost",now);}
    if(ready_&&!h.owned){fault_=true;stop("control_lost_no_auto_retake",now);}
    if(!ready_&&!stopping_&&h.owned&&h.mc_fresh&&h.robot_state_fresh){ready_=true;ready_at_=now;phase_="ready";}
    if(!ready_&&!stopping_&&now-started_>10.){fault_=true;stop("take_control_timeout",now);}
    if(stopping_) {
      // No physical stop is claimed here. With all actual SDK work retired
      // and no nonzero Move ever called, repeatedly sending zeros would only
      // keep creating the transactions whose retirement we are waiting for.
      if(noMotionRetired()){phase_="retired_without_motion";return {};}
      // A correlated successful zero write is the terminal SDK transaction.
      // Keep the nonzero gate closed while MC independently proves standstill.
      // Starting another zero write every tick would prevent an asynchronous
      // writer from ever publishing a fully drained software retirement.
      if(stop_submitted_)return {};
      if(!h.connected||!h.owned||!zeroModeAllowed(h)||h.estop)return {};
      return geometry_msgs::msg::Twist{};
    }
    // A missing owner heartbeat remains terminal even during a sensor gap;
    // MC unavailability is not permission to keep an abandoned writer armed.
    if(!task_follower_&&ready_&&(permit_at_==0?now-ready_at_>.8:!fresh(secs(permit_.source_stamp),now,.35))) {
      stop("permission_expired",now);return geometry_msgs::msg::Twist{};
    }
    if(!h.robot_state_fresh) {
      // The slow RobotState stream timed out, not a new ownership/mode fact.
      // Keep the same explicit grant and stop output. No pre-gap permit,
      // prepared handoff, native sweep or nonzero command can survive it.
      clearTelemetryGapCommands("robot_state_stale");robot_hold_=true;robot_resume_floor_=now;
      phase_="holding";reason_="robot_state_stale";
      return h.owned&&zeroModeAllowed(h)?std::optional<geometry_msgs::msg::Twist>{geometry_msgs::msg::Twist{}}:std::nullopt;
    }
    if(robot_hold_) {
      clearTelemetryGapCommands("robot_state_stale");robot_hold_=false;robot_resume_floor_=now;
      reason_.clear();phase_="holding";
    }
    if(!h.mc_fresh){
      if(permit_seq_==0&&now-started_<=10.){phase_="waiting_mc_after_takeover";return h.owned?std::optional<geometry_msgs::msg::Twist>{geometry_msgs::msg::Twist{}}:std::nullopt;}
      clearTelemetryGapCommands("mc_stale");mc_hold_=true;mc_resume_floor_=now;phase_="holding";reason_="mc_stale";
      return h.owned&&zeroModeAllowed(h)?std::optional<geometry_msgs::msg::Twist>{geometry_msgs::msg::Twist{}}:std::nullopt;
    }
    if(!ready_)return {};
    if(mc_hold_) {
      // Source freshness is genuine again. Keep the same SDK/control grant,
      // but require a new owner lease and new source-bound motion proof. The
      // owner's independent 0.6 s / 3-new-state window governs task recovery.
      clearTelemetryGapCommands("mc_stale");mc_hold_=false;mc_resume_floor_=now;reason_.clear();phase_="holding";
    }
    if(task_follower_&&h.mode_transition_active){
      clearTaskCommand();discardPendingTaskCommand();task_resume_lease_floor_=permit_seq_;task_leases_.clear();
      phase_="holding";reason_=h.mode_transition_reason.empty()?"mode_transition_pending":h.mode_transition_reason;
      return zeroModeAllowed(h)?std::optional<geometry_msgs::msg::Twist>{geometry_msgs::msg::Twist{}}:std::nullopt;
    }
    if(task_follower_)return taskTick(now);
    if(handoff_&&secs(handoff_->transition_deadline)<=now)rejectHandoff("handoff_deadline_expired",now);
    // Only the fully joined candidate may be selected. A call to the unique
    // writer is the subsequent irreversible software commit, never proof of
    // physical execution. Missing/expired candidate evidence leaves old
    // command untouched inside ALL of its original independent deadlines.
    if(handoff_&&prepared_&&preparedEntry(*prepared_,now)) {
      if(const auto* proof=motionProof(prepared_->demand,now,&*prepared_)) {
        if(!curveProofFresh(handoff_->candidate,now,true,&selected_curve_until_))return geometry_msgs::msg::Twist{};
        selected_motion_until_=proof->valid_until;selected_prepared_=true;return prepared_->demand.velocity;
      }
    }
    if(handoff_&&handoff_->transition_mode==Handoff::STATIONARY_REENTRY){phase_="holding";return geometry_msgs::msg::Twist{};}
    if(retained_incumbent_until_>0&&retained_incumbent_until_<=now){phase_="holding";return geometry_msgs::msg::Twist{};}
    if(!commit_sequence_&&(!actualLocalFresh(now)||
        !curveProofFresh(permit_,now,false,&selected_curve_until_))){
      phase_="holding";return geometry_msgs::msg::Twist{};
    }
    // A short collision proof lease expiring is a local zero-speed HOLD, not
    // loss of the independently live BT owner. Never reuse an old demand.
    if(secs(permit_.valid_until)<now){have_demand_=false;phase_="holding";return geometry_msgs::msg::Twist{};}
    if(!permit_.allowed||!have_demand_){phase_="holding";return geometry_msgs::msg::Twist{};}
    if(!version(permit_.version,demand_.version)||permit_.trajectory_id!=demand_.trajectory_id||
       !commandSourceFresh(demand_.source_stamp,now)||demand_lease_until_<now||
       (planarMotionNonzero(demand_.velocity)&&!actualLocalFresh(now))||
       (planarMotionNonzero(demand_.velocity)&&!motionProof(demand_,now))){
      have_demand_=false;phase_="holding";return geometry_msgs::msg::Twist{};
    }
    if(planarMotionNonzero(demand_.velocity))
      selected_motion_until_=motionProof(demand_,now)->valid_until;
    phase_=permit_.phase;
    selected_active_=true;
    return demand_.velocity;
  }
  uint64_t selectedCommitSequence()const{return task_follower_?0:selected_prepared_||(!commit_sequence_&&selected_active_)?commit_sequence_+1:commit_sequence_;}
  TaskCommandTiming selectedTaskCommandTiming()const{return selected_task_timing_;}
  const std::optional<TaskFeedback>& taskCommandFeedback()const{return task_feedback_.latest();}
  TaskFeedback captureTaskWrite(const geometry_msgs::msg::Twist& command,double at,
      std::optional<SdkMoveFractions> arguments={})const {
    TaskFeedback fact;fact.schema_version=1;fact.version=binding_;fact.execution_id=execution_;
    fact.control_epoch=epoch_;fact.sdk_session=session_;fact.sdk_arm_generation=generation_;
    fact.transport_mode=mode_;fact.write_sequence=write_sequence_+1;fact.source_stamp=stamp(at);
    fact.command_selected=selected_task_timing_.present;fact.command_sequence=selected_task_timing_.sequence;
    fact.command_source_stamp=selected_task_timing_.source;
    fact.command_body_source_stamp=selected_task_timing_.body_source;
    fact.command_valid_until=selected_task_timing_.valid_until;fact.velocity=command;fact.reason=reason_;
    fact.sdk_arguments_available=arguments.has_value();
    if(arguments){fact.sdk_forward=static_cast<float>(arguments->forward_back);
      fact.sdk_lateral=static_cast<float>(arguments->left_right);fact.sdk_yaw=static_cast<float>(arguments->yaw);}
    fact.sdk_nonzero=arguments?(fact.sdk_forward!=0.f||fact.sdk_lateral!=0.f||fact.sdk_yaw!=0.f):
      planarMotionNonzero(command);
    return fact;
  }
  TaskCommandExpiry taskCommandExpiry()const{return task_command_expiry_;}
  // Diagnostic-only view; caller must hold the same owner state mutex as tick.
  // Avoid allocating/copying execution_id in the optional per-write trace.
  const TaskCommandExpiry& taskCommandExpiryView()const noexcept{return task_command_expiry_;}
  TaskCommandReorderStats taskCommandReorderStats()const {
    return {task_command_reorder_deferred_,task_command_reorder_completed_,task_command_reorder_discarded_,
      static_cast<bool>(pending_task_command_)};
  }
  bool ordinaryGoalDecelerationSelected()const {
    return task_follower_&&task_selected_&&!stopping_&&
      task_command_.reason=="goal_position_settling"&&planarMotionNonzero(task_command_.velocity);
  }
  bool ordinaryYawCoastingSelected()const {
    return task_follower_&&task_selected_&&!stopping_&&
      task_command_.reason=="tracking_forward_yaw_coasting"&&
      task_command_.velocity.angular.z!=0.&&
      (task_command_.velocity.linear.x!=0.||task_command_.velocity.linear.y!=0.);
  }
  uint64_t nextWriteSequence()const{return write_sequence_+1;}
  // Omission preserves the legacy SI-to-SDK linear mapping contract. The live
  // writer supplies the classification of its final three float arguments.
  void writeCalled(double now,bool submitted,std::optional<bool> actual_sdk_nonzero=std::nullopt,
      std::optional<TaskFeedback> captured={}) {
    // Live captures BEFORE releasing the owner lock for Move. A command/body/
    // revoke arriving during the vendor call must not rename this write.
    if(task_follower_&&!captured)captured=captureTaskWrite(
      task_selected_?task_command_.velocity:geometry_msgs::msg::Twist{},now);
    ++write_sequence_;
    if(task_follower_) {
      if(captured&&task(captured->version,binding_)&&captured->execution_id==execution_&&
         captured->control_epoch==epoch_&&captured->sdk_session==session_&&
         captured->sdk_arm_generation==generation_&&captured->write_sequence==write_sequence_) {
        captured->write_submitted=submitted;
        if(actual_sdk_nonzero)captured->sdk_nonzero=*actual_sdk_nonzero;
        task_feedback_.record(std::move(*captured));
      }
      if(actual_sdk_nonzero.value_or(task_selected_&&planarMotionNonzero(task_command_.velocity))){
        invalidateTerminalStopForNonzero();
        ever_nonzero_=true;last_nonzero_write_sequence_=write_sequence_;clearNonterminalStationary();
      }
      task_selected_=false;selected_task_timing_=TaskCommandTiming{};
      if(!submitted)submissionFailed(now);
      return;
    }
    const auto&selected=selected_prepared_&&prepared_?prepared_->demand:demand_;
    const bool nonzero=actual_sdk_nonzero.value_or((selected_prepared_||selected_active_)&&planarMotionNonzero(selected.velocity));
    if(nonzero){invalidateTerminalStopForNonzero();ever_nonzero_=true;last_nonzero_write_sequence_=write_sequence_;clearNonterminalStationary();}
    if(selected_prepared_&&handoff_&&prepared_) {
      const auto g=*handoff_;const auto prepared=*prepared_;const auto previous=commit_sequence_;
      permit_=g.candidate;permit_.geometry_committed=true;permit_seq_=std::max(permit_seq_,permit_.sequence);permit_at_=now;
      permits_.clear();permits_.push_back(permit_);demand_=prepared.demand;demand_seq_=demand_.sequence;
      // CAS retires the other role. Future ordinary commands compare only
      // against the actually applied candidate; obsolete incumbent messages
      // are rejected by their exact version/curve, not a shared lane counter.
      demand_at_=now;demand_lease_until_stamp_=earlier(earlier(demand_.valid_until,permit_.valid_until),
        earlier(selected_motion_until_,earlier(commandDeadline(demand_.source_stamp),commandDeadline(demand_.body_source_stamp))));
      demand_lease_until_=secs(demand_lease_until_stamp_);
      have_demand_=true;invalidated_accepted_demand_=false;active_entry_=prepared;
      applied_version_=permit_.version;applied_trajectory_id_=permit_.trajectory_id;++commit_sequence_;
      clearNonterminalStationary();
      phase_=permit_.phase;
      ever_nonzero_=ever_nonzero_||nonzero;
      makeCommitAck(g,prepared,previous,now,submitted);handoff_.reset();prepared_.reset();
      retained_incumbent_until_=0.;
    } else if(selected_active_&&!commit_sequence_&&actualLocalFresh(now)&&commandSourceFresh(demand_.source_stamp,now)&&
        commandSourceFresh(demand_.body_source_stamp,now)&&curveProofFresh(permit_,now,false)) {
      applied_version_=permit_.version;applied_trajectory_id_=permit_.trajectory_id;++commit_sequence_;
      Handoff g;g.incumbent.version=binding_;g.candidate=permit_;Prepared prepared;prepared.demand=demand_;
      if(local_){prepared.measured_pose.header=local_->local_odometry.header;prepared.measured_pose.pose=local_->local_odometry.pose.pose;
        prepared.measured_twist=localTwist();}
      makeCommitAck(g,prepared,0,now,submitted);
    }
    selected_active_=selected_prepared_=false;
    if(!submitted)submissionFailed(now);
  }
  void writeAcknowledged(uint64_t commit,double now) {
    (void)now;
    if(last_ack_&&last_ack_->applied&&last_ack_->commit_sequence==commit&&!last_ack_->write_acknowledged){
      last_ack_->write_acknowledged=true;last_ack_->sequence=++ack_sequence_;ack_dirty_=true;
    }
  }
  void zeroAcknowledged(uint64_t write,double at,double now) {
    // This is the actual vendor ACK of an actual zero call, not a BT-local
    // velocity guess or a terminal StopReport. Earlier ACKs cannot cross a
    // subsequent nonzero call or restore retired SDK/control identities.
    if(!write||write>write_sequence_||write<=last_nonzero_write_sequence_||closing_||stopping_||mc_hold_||robot_hold_||fault_||
       !ready_||!last_health_.connected||!last_health_.owned||!zeroModeAllowed(last_health_)||!last_health_.robot_state_fresh||!last_health_.mc_fresh||
       (!task_follower_&&!commit_sequence_)||permit_.allowed||permit_.phase!="holding"||!fresh(at,now,.25)||!mcSourceFresh(now,clock_epoch_))return;
    if(nonterminal_zero_ack_>0)return; // repeated zero writes do not move the barrier
    nonterminal_zero_ack_=at;nonterminal_zero_write_=write;nonterminal_cutoff_=last_raw_;
  }
  Stationary stationaryEvidence(double now) {
    Stationary s;s.schema_version=1;s.version=task_follower_?binding_:applied_version_;s.execution_id=execution_;s.control_epoch=epoch_;
    s.sdk_session=session_;s.sdk_arm_generation=generation_;s.transport_mode=mode_;s.sequence=++stationary_sequence_;
    s.writer_commit_sequence=commit_sequence_;s.applied_trajectory_id=applied_trajectory_id_;
    s.zero_write_sequence=nonterminal_zero_write_;s.zero_ack_at=stamp(nonterminal_zero_ack_);
    s.mc_raw_stamp_ns=last_raw_;s.mc_clock_epoch=clock_epoch_;s.time_basis=mode_=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock";
    s.source_stamp=stamp(last_source_);s.received_stamp=stamp(last_received_);s.capture_lower_bound=stamp(last_capture_lower_);
    s.capture_upper_bound=stamp(last_received_);s.mc_capture_delay_bound_sec=mc_delay_bound_;
    s.valid_until=stamp(std::min(last_source_,last_received_)+.25);s.stationary_samples=nonterminal_stationary_samples_;
    s.stationary_duration_sec=nonterminal_first_stationary_>0?std::max(0.,last_capture_lower_-nonterminal_first_stationary_):0.;
    s.measured_linear_mps=linear_;s.measured_angular_radps=angular_;
    s.velocity_metric=velocity_metric_;s.measured_planar_mps=planar_;s.measured_vertical_mps=vertical_;
    s.physical_acceptance_verified=mode_=="live"&&physically_verified_;
    s.nonzero_blocked=!permit_.allowed&&permit_.phase=="holding";
    s.usable=nonterminalStationary(now)&&s.nonzero_blocked;
    if(s.usable){stationary_history_.push_back(s);if(stationary_history_.size()>8)stationary_history_.pop_front();}
    return s;
  }
  std::optional<CommitAck> takeCommitAck(){if(!ack_dirty_||!last_ack_)return {};ack_dirty_=false;return last_ack_;}
  void stopWritten(double now) {
    if(!stopping_||stop_submitted_)return;
    stop_submitted_=true;submitted_=now;cutoff_=last_raw_;resetStationary();
  }
  void mc(uint64_t raw,double source,double received,double linear,double angular,double now,std::string clock_epoch,
          double planar=0.,double vertical=0.,bool fresh_profile_mode=false) {
    if(!clock_epoch_.empty()&&clock_epoch_!=clock_epoch){fault_=true;stop("mc_clock_epoch_changed",now);resetStationary();return;}
    if(raw>0&&last_raw_>0&&raw<last_raw_){mcSourceRegressed(now,clock_epoch);return;}
    if(raw==0||raw==last_raw_||!mcTimeFresh(source,received,now)||
       !std::isfinite(linear)||!std::isfinite(angular)||linear<0||angular<0||
       !std::isfinite(planar)||!std::isfinite(vertical)||planar<0||vertical<0)return;
    const char* metric=fresh_profile_mode&&policy_.velocity_metric=="planar_xy_with_vertical_guard_v1"?
      "planar_xy_with_vertical_guard_v1":"spatial_norm_3d";
    if(fresh_profile_mode&&policy_.velocity_metric=="planar_xy_with_vertical_guard_v1"&&
       std::abs(std::hypot(planar,vertical)-linear)>1e-6)return;
    // A mode/metric transition cannot combine evidence from unlike contracts.
    if(metric!=velocity_metric_){resetStationary();clearNonterminalStationary();}
    if(last_source_>0&&(source<=last_source_||source-last_source_>.2||std::abs((source-last_source_)-(raw-last_raw_)*1e-9)>.02)){
      resetStationary();clearNonterminalStationary();}
    clock_epoch_=std::move(clock_epoch);last_raw_=raw;last_source_=source;last_received_=received;linear_=linear;angular_=angular;
    velocity_metric_=metric;planar_=planar;vertical_=vertical;
    if(mc_order_hold_) {
      // A source regression is not guessed to be a new clock epoch. Require
      // unique samples ABOVE the retained raw floor and a real source-time
      // stability window, while keeping the same bounded owner HOLD budget.
      if(source<=mc_order_barrier_source_){mc_order_new_samples_=0;mc_order_first_source_=0.;}
      else {if(!mc_order_first_source_)mc_order_first_source_=source;
        ++mc_order_new_samples_;if(mc_order_new_samples_>=3&&source-mc_order_first_source_>=.6)mc_order_hold_=false;}
    }
    // The physical capture time is bounded, not made exact by relabelling the
    // SDK's undocumented uint64 epoch. Host receipt is the interval upper
    // bound; subtract the profile's uncertainty from the earlier host estimate.
    last_capture_lower_=std::min(source,received)-mc_delay_bound_;
    if(!stationaryMotionWithinPolicy())clearNonterminalStationary();
    else if(nonterminal_zero_ack_>0&&raw>nonterminal_cutoff_&&last_capture_lower_>nonterminal_zero_ack_){
      if(nonterminal_first_stationary_==0)nonterminal_first_stationary_=received;
      ++nonterminal_stationary_samples_;
    }
    if(!stopping_||!stop_submitted_||!mc_bound_configured_||raw<=cutoff_||
       last_capture_lower_<=submitted_)return;
    if(!stationaryMotionWithinPolicy()){resetStationary();return;}
    if(first_stationary_==0)first_stationary_=received;
    ++stationary_samples_;stopped_=stationary_samples_>=policy_.minimum_new_samples&&stationaryDuration()>=policy_.stationary_duration_s;
    if(stopped_){phase_="stopped";retainTerminalStop(now);}
  }
  State state(double now){State s;s.version=commit_sequence_?applied_version_:binding_;s.execution_id=execution_;s.control_epoch=epoch_;s.sdk_session=session_;
    s.sdk_arm_generation=generation_;s.sequence=++state_seq_;s.source_stamp=stamp(now);s.grant_ready=ready_&&last_health_.robot_state_fresh&&last_health_.mc_fresh&&!stopping_&&!fault_&&!software_estop_latched_;
    s.control_owned=last_health_.owned;s.general_low_speed_confirmed=last_health_.robot_state_fresh&&last_health_.general;s.fault_latched=fault_||software_estop_latched_;
    s.motion_mode=last_health_.motion_mode.empty()?(last_health_.general?"general":"unknown"):last_health_.motion_mode;
    s.motion_mode_confirmed=last_health_.robot_state_fresh&&motionModeObserved(last_health_);
    s.mode_transition_active=last_health_.mode_transition_active;
    if(s.mode_transition_active||!motionModeAllowed(last_health_))s.grant_ready=false;
    s.transport_mode=mode_;s.phase=phase_;s.reason=reason_;
    if(s.reason.empty()&&!motionModeAllowed(last_health_)&&!last_health_.mode_transition_reason.empty())
      s.reason=last_health_.mode_transition_reason;
    // The session-wide operator event must remain identifiable even while an
    // older task is stopping for another recorded reason. Do not rewrite that
    // task's retirement/physical-stop facts to supply this diagnostic contract.
    if(software_estop_latched_){s.phase="software_estop_latched";s.reason=software_estop_release_confirmed_?
      "software_estop_release_waiting_prior_retirement":software_estop_reason_;}
    s.writer_commit_sequence=commit_sequence_;s.applied_trajectory_id=applied_trajectory_id_;return s;}
  Stop report(double now){
    retainTerminalStop(now);
    // A terminal report is an immutable event, not a live stationary lease.
    // Its original measurement, source time and sequence MUST survive later
    // noise/telemetry gaps. Live health and StationaryEvidence remain separate.
    return terminalStopMatches()?*terminal_stop_:currentStopReport(now);
  }
  Stop currentStopReport(double now){Stop s;s.version=binding_;s.execution_id=execution_;s.control_epoch=epoch_;s.sdk_session=session_;
    s.sdk_arm_generation=generation_;s.stop_request_id=request_;s.sequence=++stop_seq_;
    // StopReport's source is the original accepted MC receipt/capture upper
    // bound, not the approximate mapped point or a timer-generated heartbeat.
    // No accepted MC means no stop evidence timestamp (zero), including boot.
    s.source_stamp=stamp(last_received_);
    s.nonzero_blocked=stopping_||mc_hold_||robot_hold_||(task_follower_&&
      (!task_lease_.allowed||!fresh(secs(task_lease_.source_stamp),now,.35)||secs(task_lease_.valid_until)<now||
       !task_have_command_||!taskCommandTime(task_command_,now)));
    s.stop_submitted=stop_submitted_;s.measured_stop_confirmed=measuredStopped(now);
    s.physical_acceptance_verified=mode_=="live"&&physically_verified_;s.mc_clock_epoch=clock_epoch_;s.mc_raw_stamp_ns=last_raw_;
    s.stationary_samples=stationary_samples_;s.stationary_duration_sec=stationaryDuration();
    s.measured_linear_mps=linear_;s.measured_angular_radps=angular_;
    s.velocity_metric=velocity_metric_;s.measured_planar_mps=planar_;s.measured_vertical_mps=vertical_;
    s.time_basis=mode_=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock";
    s.transport_mode=mode_;s.reason=reason_;return s;}
  Retirement retirement(double now){Retirement s;s.schema_version=1;s.version=binding_;s.execution_id=execution_;
    s.control_epoch=epoch_;s.sdk_session=session_;s.sdk_arm_generation=generation_;s.grant_request_id=request_;
    s.sequence=++retirement_sequence_;s.source_stamp=stamp(now);s.transport_mode=mode_;
    s.nonzero_blocked=stopping_;s.software_retired=!execution_.empty()&&stopping_&&sdk_transactions_tracked_&&
      pending_sdk_transactions_==0&&!sdk_transaction_uncertain_;
    s.motion_never_submitted=!ever_nonzero_;
    s.pending_sdk_transactions=pending_sdk_transactions_;s.reason=reason_;return s;}
  const Demand& lastDemand()const{return selected_prepared_&&prepared_?prepared_->demand:demand_;}const std::string& executionId()const{return execution_;}
  const Version& binding()const{return binding_;}
  uint64_t armGeneration()const{return generation_;}
  uint64_t epoch()const{return epoch_;}bool stopping()const{return stopping_;}
  bool beginShutdown(double now){closing_=true;takeover_=false;stop("writer_shutdown",now);return !execution_.empty()&&!noMotionRetired();}
  double mcCaptureLowerBound()const{return last_capture_lower_;}
  double mcCaptureUpperBound()const{return last_received_;}
  double mcCaptureDelayBound()const{return mc_delay_bound_;}
  bool mcSourceFresh(double now,const std::string&expected_clock_epoch)const {
    // Arrival frequency alone is not usable MC. Bind health to the exact raw
    // sample admitted by this transport, the active SDK telemetry generation,
    // and both original mapped source time and receipt time. No restamping.
    return !mc_order_hold_&&mc_bound_configured_&&last_raw_>0&&!expected_clock_epoch.empty()&&
      clock_epoch_==expected_clock_epoch&&mcTimeFresh(last_source_,last_received_,now);
  }
  void mcSourceRegressed(double now,const std::string&clock_epoch) {
    if(!clock_epoch_.empty()&&clock_epoch_!=clock_epoch){fault("mc_clock_epoch_changed",now);return;}
    clearTelemetryGapCommands("mc_stale");resetStationary();mc_hold_=mc_order_hold_=true;mc_resume_floor_=now;
    mc_order_new_samples_=0;mc_order_first_source_=0.;mc_order_barrier_source_=now;phase_="holding";reason_="mc_stale";
    // Preserve last_raw_/source/receipt. Neither rejection nor recovery may
    // re-anchor the clock, refresh old telemetry or erase the anti-replay floor.
  }
  void stop(std::string why,double now){if(execution_.empty()||stopping_)return;rejectHandoff(why,now);clearNonterminalStationary();clearTaskCommand();discardPendingTaskCommand();task_leases_.clear();stopping_=true;stop_at_=now;ready_=false;have_demand_=false;permits_.clear();pending_demand_.reset();geometry_.clear();phase_="stopping";reason_=std::move(why);}
private:
  void latchSoftwareEstop(const std::string&reason,double now) {
    if(!std::isfinite(now)||now<=0.){fault("software_estop_clock_invalid",now);return;}
    software_estop_latched_=true;software_estop_release_confirmed_=false;software_estop_reason_=reason;
    software_estop_had_owned_=software_estop_had_owned_||(!execution_.empty()&&last_health_.owned);
    software_estop_requested_at_=std::max(software_estop_requested_at_,now);
    software_estop_grant_barrier_ns_=std::max(software_estop_grant_barrier_ns_,sourceNs(stamp(now)));
    takeover_=false;stop(reason,now);
    if(execution_.empty()){phase_="software_estop_latched";reason_=reason;}
  }
  void tryCompleteSoftwareEstopRelease(double now,const Health&h) {
    if(!software_estop_latched_||!software_estop_release_confirmed_||fault_||closing_||
       !h.connected||h.estop||!h.robot_state_fresh||
       (software_estop_had_owned_&&!h.owned)||!std::isfinite(now)||now<software_estop_requested_at_)return;
    const bool retired=execution_.empty()||(stopping_&&sdk_transactions_tracked_&&
        pending_sdk_transactions_==0&&!sdk_transaction_uncertain_&&(noMotionRetired()||terminalStopSatisfied(now)));
    if(!retired)return;
    software_estop_latched_=software_estop_release_confirmed_=false;
    software_estop_had_owned_=false;
    // Request/epoch/SDK-generation replay floors remain intact. Clearing this
    // one latch gives no permission to the old (still stopping) execution.
    if(execution_.empty()){phase_="idle";reason_="software_estop_released_new_grant_required";}
  }
  void clearTaskCommand(){task_have_command_=task_selected_=false;task_original_command_.reset();selected_task_timing_=TaskCommandTiming{};}
  void discardPendingTaskCommand(){if(pending_task_command_){pending_task_command_.reset();++task_command_reorder_discarded_;}}
  bool deferTaskCommand(const LocalCommand&c){
    // ONE latest original sample, not a command queue. Unknown future lease
    // numbers never advance accepted sequence/source floors or block a newer
    // real command. Source order, rather than an untrusted future sequence,
    // determines replacement of this merely deferred sample.
    if(!pending_task_command_||sourceNs(c.source_stamp)>=sourceNs(pending_task_command_->source_stamp)){
      discardPendingTaskCommand();pending_task_command_=c;++task_command_reorder_deferred_;
    }
    task_command_admission_.reason="lease_deferred";
    return false; // Deferred is not accepted or authorized to move.
  }
  bool taskCommandTime(const LocalCommand&c,double now)const {
    const auto body=sourceNs(c.body_source_stamp),body_until=sourceNs(c.body_valid_until),command=sourceNs(c.source_stamp);
    return policy_configured_&&canonicalTime(c.source_stamp)&&canonicalTime(c.body_source_stamp)&&
      canonicalTime(c.body_valid_until)&&canonicalTime(c.valid_until)&&body>0&&body_until>body&&body_until-body<=100000000LL&&
      command>=body&&fresh(secs(c.source_stamp),now,commandPipeline())&&
      now>=secs(c.body_source_stamp)&&now<=secs(c.body_valid_until)&&
      sourceNs(c.valid_until)>command&&secs(c.valid_until)>=now&&
      sourceNs(c.valid_until)<=body_until&&sourceNs(c.valid_until)<=sourceNs(commandDeadline(c.source_stamp));
  }
  std::optional<geometry_msgs::msg::Twist> taskTick(double now) {
    if(!task_lease_.allowed||!fresh(secs(task_lease_.source_stamp),now,.35)||secs(task_lease_.valid_until)<now){
      clearTaskCommand();task_resume_lease_floor_=permit_seq_;task_leases_.clear();
      phase_="holding";reason_=permit_seq_==0?"waiting_task_lease":"task_lease_unavailable";return geometry_msgs::msg::Twist{};
    }
    if(!task_have_command_||!taskCommandTime(task_command_,now)){
      if(task_have_command_){
        ++task_command_expiry_.count;task_command_expiry_.sequence=task_command_.sequence;
        task_command_expiry_.observed_at=now;task_command_expiry_.execution_id=execution_;
        task_command_expiry_.source=task_command_.source_stamp;
        task_command_expiry_.body_source=task_command_.body_source_stamp;
        task_command_expiry_.body_valid_until=task_command_.body_valid_until;
        task_command_expiry_.valid_until=task_command_.valid_until;
        task_command_expiry_.admission_sequence=task_command_admission_.sequence;
        task_command_expiry_.admission_at=task_command_admission_.at;
        task_command_expiry_.admission_reason=task_command_admission_.reason;
      }
      clearTaskCommand();phase_="holding";reason_="waiting_local_command";return geometry_msgs::msg::Twist{};
    }
    phase_=task_lease_.phase.empty()?"tracking":task_lease_.phase;reason_=task_command_.reason;
    task_selected_=true;
    selected_task_timing_={true,task_command_.sequence,task_command_.source_stamp,
      task_command_.body_source_stamp,task_command_.valid_until};
    // The curve ID is diagnostic only. No cache, join, proof, CAS or curve
    // transition participates in sending this current task's velocity.
    applied_trajectory_id_=task_command_.trajectory_id;
    return task_command_.velocity;
  }
  bool mcTimeFresh(double source,double received,double now)const {
    // Mock time is exact; only live receipt-anchored SDK time has uncertainty.
    // Neither path may freshen an old receipt, reset raw order or extend TTL.
    return mc_bound_configured_&&d1monitor::mcCaptureTimeFresh(source,received,now,.25,
      mode_=="live"?mc_delay_bound_:0.);
  }
  static bool twistFinite(const geometry_msgs::msg::Twist&t) {
    return std::isfinite(t.linear.x)&&std::isfinite(t.linear.y)&&std::isfinite(t.linear.z)&&
      std::isfinite(t.angular.x)&&std::isfinite(t.angular.y)&&std::isfinite(t.angular.z);
  }
  bool velocityAllowed(const geometry_msgs::msg::Twist&v)const {
    return twistFinite(v)&&std::hypot(v.linear.x,v.linear.y)<=max_speed_&&std::abs(v.angular.z)<=max_yaw_&&
      v.linear.z==0.&&v.angular.x==0.&&v.angular.y==0.&&
      (kinematic_profile_=="holonomic_xyyaw"||(v.linear.x>=0.&&v.linear.y==0.));
  }
  static bool poseFinite(const geometry_msgs::msg::Pose&p) {
    const auto&q=p.orientation;
    return std::isfinite(p.position.x)&&std::isfinite(p.position.y)&&std::isfinite(p.position.z)&&
      std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.z)&&std::isfinite(q.w)&&
      std::abs(std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)-1.)<=.001;
  }
  static double distance(const geometry_msgs::msg::Point&a,const geometry_msgs::msg::Point&b) {
    return std::hypot(a.x-b.x,a.y-b.y,a.z-b.z);
  }
  static double velocityDistance(const geometry_msgs::msg::Twist&a,const geometry_msgs::msg::Twist&b) {
    return std::hypot(a.linear.x-b.linear.x,a.linear.y-b.linear.y,a.linear.z-b.linear.z);
  }
  template<class Context>static bool localContext(const Local&m,const Context&c) {
    return m.session_id==c.session_id&&m.map_version_id==c.map_version_id&&
      m.localization_epoch==c.localization_epoch&&m.localization_seed_id==c.localization_seed_id;
  }
  static geometry_msgs::msg::Twist localTwist(const Local&m) {
    geometry_msgs::msg::Twist out;
    const auto&q=m.local_odometry.pose.pose.orientation;
    const auto rotate=[&](const geometry_msgs::msg::Vector3&v) {
      geometry_msgs::msg::Vector3 r;
      const double tx=2.*(q.y*v.z-q.z*v.y),ty=2.*(q.z*v.x-q.x*v.z),tz=2.*(q.x*v.y-q.y*v.x);
      r.x=v.x+q.w*tx+q.y*tz-q.z*ty;r.y=v.y+q.w*ty+q.z*tx-q.x*tz;r.z=v.z+q.w*tz+q.x*ty-q.y*tx;return r;
    };
    out.linear=rotate(m.local_odometry.twist.twist.linear);
    out.angular=rotate(m.local_odometry.twist.twist.angular);return out;
  }
  geometry_msgs::msg::Twist localTwist()const {
    return local_?localTwist(*local_):geometry_msgs::msg::Twist{};
  }
  bool progressMatchesSource(const Progress&t)const {
    // Compare acquisition times, not callback order. A 50 Hz state arriving
    // before a 20 Hz progress callback is motion, not a geometric mismatch.
    // Only bounded real neighboring states may interpolate; no extrapolation
    // or renewed source/evidence deadline is introduced here.
    const auto target=sourceNs(t.header.stamp);
    const Local*left=nullptr;
    for(const auto&m:local_history_) {
      const auto at=sourceNs(m.source_stamp);
      if(at==target)return distance(t.pose.position,m.local_odometry.pose.pose.position)<=.0125&&
        velocityDistance(t.twist,localTwist(m))<=.05;
      if(at>target) {
        if(!left)return false;
        const auto begin=sourceNs(left->source_stamp),gap=at-begin;
        if(gap<=0||gap>100000000LL)return false;
        const double ratio=static_cast<double>(target-begin)/gap;
        auto position=left->local_odometry.pose.pose.position;
        const auto&end=m.local_odometry.pose.pose.position;
        position.x+=ratio*(end.x-position.x);position.y+=ratio*(end.y-position.y);position.z+=ratio*(end.z-position.z);
        auto twist=localTwist(*left);const auto next=localTwist(m);
        twist.linear.x+=ratio*(next.linear.x-twist.linear.x);
        twist.linear.y+=ratio*(next.linear.y-twist.linear.y);
        twist.linear.z+=ratio*(next.linear.z-twist.linear.z);
        return distance(t.pose.position,position)<=.0125&&velocityDistance(t.twist,twist)<=.05;
      }
      left=&m;
    }
    return false;
  }
  bool sameGrantIdentity(const Permit&p)const {
    return complete(p.version)&&task(binding_,p.version)&&p.execution_id==execution_&&p.control_epoch==epoch_&&
      p.sdk_session==session_&&p.sdk_arm_generation==generation_&&p.transport_mode==mode_&&p.frame_id=="d1max_loc_odom";
  }
  bool demandBinding(const Demand&d)const {
    const auto&v=d.velocity;
    return complete(d.version)&&task(binding_,d.version)&&d.execution_id==execution_&&d.control_epoch==epoch_&&
      d.sdk_session==session_&&d.sdk_arm_generation==generation_&&d.transport_mode==mode_&&d.sequence>0&&
      velocityAllowed(v)&&(!planarMotionNonzero(v)||(!d.hold&&d.safety_checked));
  }
  bool preparedEntry(const Prepared&p,double now)const {
    if(!handoff_||!actualLocalFresh(now)||!last_health_.mc_fresh||
       stopping_||closing_||mc_hold_||robot_hold_||!ready_||!last_health_.owned||!last_health_.general||!last_health_.robot_state_fresh||last_health_.estop||
       p.expected_commit_sequence!=commit_sequence_||secs(handoff_->transition_deadline)<=now||
       permit_.revoked||
       (handoff_->transition_mode==Handoff::CONTINUOUS_REPLACE&&(!permit_.allowed||permit_.phase!="tracking"))||
       (handoff_->transition_mode==Handoff::STATIONARY_REENTRY&&
          (permit_.allowed||permit_.phase!="holding"||!stationaryProof(handoff_->stationary_evidence,now)))||
       !sameGrantIdentity(handoff_->candidate)||secs(handoff_->candidate.valid_until)<=now||!curveProofFresh(handoff_->candidate,now,true)||
       !commandSourceFresh(p.demand.source_stamp,now)||!commandSourceFresh(p.demand.body_source_stamp,now)||
       secs(p.demand.valid_until)<=now||p.demand.validation_sequence<handoff_->candidate.validation_sequence||
       p.demand.braking_model_sha256!=braking_model_sha_||!p.demand.safety_checked||
       !demandBinding(p.demand)||
       !poseFinite(p.measured_pose.pose)||!poseFinite(p.curve_entry_pose)||!twistFinite(p.measured_twist)||!twistFinite(p.curve_entry_twist)||
       p.measured_pose.header.frame_id!="d1max_loc_odom"||p.measured_pose.header.stamp!=p.entry_source_stamp||
       p.entry_source_stamp!=p.demand.body_source_stamp||!commandSourceFresh(p.entry_source_stamp,now)||
       !std::isfinite(p.curve_time)||p.curve_time<0||p.curve_time>120||
       !std::isfinite(p.position_error_m)||!std::isfinite(p.velocity_error_mps)||p.position_error_m<0||p.velocity_error_mps<0||
       std::abs(distance(p.measured_pose.pose.position,p.curve_entry_pose.position)-p.position_error_m)>1e-6||
       std::abs(velocityDistance(p.measured_twist,p.curve_entry_twist)-p.velocity_error_mps)>1e-6||
       p.position_error_m>.0125||p.velocity_error_mps>.05)return false;
    const auto&a=p.entry_admission;
    if(a.sequence==0||!a.accepted||a.version!=p.demand.version||a.trajectory_id!=p.demand.trajectory_id||
       a.validation_sequence!=p.demand.validation_sequence||a.body_source_stamp!=p.entry_source_stamp||
       a.transport_mode!=mode_||!commandSourceFresh(a.checked_at,now)||secs(a.checked_at)<secs(a.body_source_stamp)||
       secs(a.valid_until)<=now||secs(a.valid_until)-secs(a.checked_at)>.250001)return false;
    Progress original;original.header=p.measured_pose.header;
    original.pose=p.measured_pose.pose;original.twist=p.measured_twist;
    // Bind the original preparation to a real acquisition in the writer's
    // already bounded local history. Delivery order is not acquisition time;
    // progressMatchesSource permits only exact or bracketed real samples.
    if(!progressMatchesSource(original))return false;
    // Local odometry twist is in child frame. Rotate the actual measured
    // twist into odom, then compare to the production-spline entry boundary.
    // Source age alone cannot certify a 1.25 cm moving handoff.
    const auto geometry=std::find_if(geometry_.begin(),geometry_.end(),[&](const auto&g){
      return g.message.trajectory.traj_id==p.demand.trajectory_id&&g.cache.versionMatches(p.demand.version);
    });
    // Strict historical equality is a source-time check, not the present
    // entry point. Real XYZ chooses a bounded connected point on this SAME
    // immutable production spline; original demand/proof/entry stay unchanged.
    if(geometry==geometry_.end())return false;
    const CurveProof* newest=nullptr;
    for(const auto&v:curve_proofs_)if(version(v.version,p.demand.version)&&v.trajectory_id==p.demand.trajectory_id&&
        (!newest||v.sequence>newest->sequence))newest=&v;
    return newest&&std::abs(newest->curve_duration-geometry->cache.duration())<=1e-6&&
      geometry->cache.observe(p,*local_,max_speed_,now,commandPipeline()).has_value();
  }
  bool curveProofFresh(const Permit&p,double now,bool whole,builtin_interfaces::msg::Time* actual_until=nullptr)const {
    const CurveProof* latest=nullptr;
    for(const auto&v:curve_proofs_)if(version(v.version,p.version)&&v.trajectory_id==p.trajectory_id&&
        (!latest||v.sequence>latest->sequence))latest=&v;
    if(!latest)return false;
    if(curveInvalidated(p.version,p.trajectory_id,p.validation_sequence))return false;
    // A nonzero remaining-curve first write needs the exact native command
    // verdict. Its source-measured
    // entry was checked by SCAN; a second independently delivered progress
    // stream cannot grant authority or veto that still-live native verdict.
    // Select the immutable curve evidence actually named by the verdict,
    // not a later positive heartbeat that would silently renew its expiry.
    const CurveProof* selected=latest;
    bool remaining_entry=false;
    if(!whole&&latest->remaining_curve&&!latest->whole_curve) {
      if(!have_demand_||!p.geometry_committed||
         !version(demand_.version,p.version)||demand_.trajectory_id!=p.trajectory_id)return false;
      if(planarMotionNonzero(demand_.velocity)) {
        const auto* motion=motionProof(demand_,now);
        if(!motion)return false;
        selected=nullptr;
        for(const auto&v:curve_proofs_)if(version(v.version,p.version)&&v.trajectory_id==p.trajectory_id&&
            v.sequence==motion->trajectory_validation_sequence){selected=&v;break;}
        if(!selected)return false;
        remaining_entry=true;
      } else if(progress_&&local_) {
        // Preserve the legacy zero-only geometry ACK. It has no native motion
        // verdict and cannot borrow a suffix without its measured XYZ entry.
        // This is not an authorization to submit nonzero motion.
        const auto&t=*progress_;const auto&x=p.version;const auto&v=*latest;
        remaining_entry=fresh(secs(t.header.stamp),now,.1)&&
          t.generation==x.reference_generation&&t.trajectory_id==p.trajectory_id&&
          t.session_id==x.session_id&&t.task_id==x.task_id&&t.route_id==x.route_id&&t.route_hash==x.route_hash&&
          t.map_version_id==x.map_version_id&&t.localization_epoch==x.localization_epoch&&t.localization_seed_id==x.localization_seed_id&&
          t.segment_id==x.segment_id&&t.anchor_id==x.anchor_id&&t.anchor_revision==x.anchor_revision&&t.context_sequence==x.context_sequence&&
          std::isfinite(v.checked_from_time)&&std::isfinite(v.checked_to_time)&&
          v.checked_from_time<=t.curve_time&&t.curve_time<=v.checked_to_time&&
          fresh(secs(local_->source_stamp),now,.1)&&progressMatchesSource(t);
      }
    }
    const auto&v=*selected;
    const bool domain=std::isfinite(v.curve_duration)&&v.curve_duration>0&&v.curve_duration<=120&&
      std::isfinite(v.checked_from_time)&&std::isfinite(v.checked_to_time)&&std::isfinite(v.valid_start_time)&&
      v.checked_from_time>=0&&v.checked_from_time<=v.valid_start_time&&v.valid_start_time<=v.checked_to_time&&
      std::abs(v.checked_to_time-v.curve_duration)<=1e-6;
    const bool complete=domain&&v.whole_curve&&!v.remaining_curve&&v.checked_from_time==0.;
    const bool remaining=!whole&&remaining_entry&&domain&&v.remaining_curve&&!v.whole_curve&&
      std::isfinite(v.reverse_margin_m)&&v.reverse_margin_m>=.15;
    // Geometry only. tick()/preparedEntry's caller must still join the exact
    // SCAN MotionValidation before selecting any nonzero SDK write.
    const bool geometry_policy=v.collision_policy=="observed_free"||v.collision_policy=="official_inflated_geometry";
    const bool valid=(complete||remaining)&&geometry_policy&&v.valid&&v.sequence>=p.validation_sequence&&v.frame_id=="d1max_loc_odom"&&
      v.map_snapshot_revision>0&&!v.support_reference_id.empty()&&v.support_hash.size()==64&&
      // SCAN already caps this original deadline by all perception leases.
      fresh(secs(v.check_end),now,.25)&&
      secs(v.valid_until)>now&&secs(v.valid_until)-secs(v.check_end)<=.250001;
    if(valid&&actual_until)*actual_until=v.valid_until;
    return valid;
  }
  bool curveInvalidated(const Version&v,int64_t trajectory,uint64_t floor)const {
    return std::any_of(curve_proofs_.begin(),curve_proofs_.end(),[&](const auto&p){
      return version(p.version,v)&&p.trajectory_id==trajectory&&!p.valid&&p.sequence>floor;
    });
  }
  void makeCommitAck(const Handoff&g,const Prepared&p,uint64_t previous,double now,bool submitted) {
    CommitAck a;a.schema_version=1;a.handoff_id=g.handoff_id;a.grant_sequence=g.sequence;a.sequence=++ack_sequence_;
    a.previous_commit_sequence=previous;a.commit_sequence=commit_sequence_;a.incumbent_version=g.incumbent.version;
    a.candidate_version=g.candidate.version;a.incumbent_trajectory_id=g.incumbent.trajectory_id;a.candidate_trajectory_id=g.candidate.trajectory_id;
    a.execution_id=execution_;a.sdk_session=session_;a.transport_mode=mode_;a.control_epoch=epoch_;a.sdk_arm_generation=generation_;
    a.permit_sequence=p.demand.permit_sequence;a.demand_sequence=p.demand.sequence;a.motion_validation_sequence=p.demand.motion_validation_sequence;
    a.entry_admission_sequence=p.entry_admission.sequence;a.applied_at=stamp(now);a.demand_source_stamp=p.demand.source_stamp;
    a.demand_body_source_stamp=p.demand.body_source_stamp;a.entry_source_stamp=p.entry_source_stamp;
    a.body_source_stamp=local_?local_->source_stamp:p.demand.body_source_stamp;
    // Submission facts retain original typed source deadlines. Re-stamping
    // an epoch-sized double can extend them by nanoseconds. Every geometry
    // commit is capped by the exact curve evidence selected for the write;
    // nonzero (and conditional zero) writes also retain their native sweep.
    // A later renewal or ACK callback cannot change those original bounds.
    a.valid_until=earlier(demand_lease_until_stamp_,selected_curve_until_);
    if(selected_motion_until_.sec||selected_motion_until_.nanosec)
      a.valid_until=earlier(a.valid_until,selected_motion_until_);
    if(local_){a.measured_pose.header=local_->local_odometry.header;a.measured_pose.pose=local_->local_odometry.pose.pose;
      a.measured_twist=localTwist();}else{a.measured_pose=p.measured_pose;a.measured_twist=p.measured_twist;}
    a.applied_velocity=p.demand.velocity;a.curve_time=p.curve_time;
    a.applied=true;a.write_submitted=submitted;a.write_acknowledged=false;
    a.reason=submitted?"writer_software_commit_not_physical_confirmation":"writer_call_outcome_failed_identity_not_rolled_back";
    last_ack_=a;ack_dirty_=true;
  }
  void rejectHandoff(const std::string&why,double now) {
    if(!handoff_)return;
    const auto&g=*handoff_;CommitAck a;a.schema_version=1;a.handoff_id=g.handoff_id;a.grant_sequence=g.sequence;
    a.sequence=++ack_sequence_;a.previous_commit_sequence=a.commit_sequence=commit_sequence_;
    a.incumbent_version=g.incumbent.version;a.candidate_version=g.candidate.version;a.incumbent_trajectory_id=g.incumbent.trajectory_id;
    a.candidate_trajectory_id=g.candidate.trajectory_id;a.execution_id=execution_;a.sdk_session=session_;a.transport_mode=mode_;
    a.control_epoch=epoch_;a.sdk_arm_generation=generation_;a.permit_sequence=g.candidate.sequence;
    a.applied_at=stamp(now);a.valid_until=g.transition_deadline;a.reason=why;
    last_ack_=a;ack_dirty_=true;handoff_.reset();prepared_.reset();selected_prepared_=false;
  }
  static bool sameOriginalDemand(const Demand&a,const Demand&b) {
    auto normalized=a;
    // These are the independent gate's proof signature, not control payload.
    normalized.motion_validation_sequence=b.motion_validation_sequence;
    normalized.safety_source_stamp=b.safety_source_stamp;
    normalized.reason=b.reason;
    return normalized==b&&secs(a.safety_source_stamp)>=secs(b.safety_source_stamp);
  }
  void clearTelemetryGapCommands(const std::string& reason){rejectHandoff(reason,last_tick_);clearNonterminalStationary();clearTaskCommand();discardPendingTaskCommand();task_resume_lease_floor_=permit_seq_;task_leases_.clear();have_demand_=false;permits_.clear();motion_proofs_.clear();pending_demand_.reset();active_entry_.reset();}
  void clearNonterminalStationary(){nonterminal_zero_ack_=nonterminal_first_stationary_=0.;
    nonterminal_zero_write_=nonterminal_cutoff_=0;nonterminal_stationary_samples_=0;stationary_history_.clear();}
  bool nonterminalStationary(double now)const {
    return !closing_&&!stopping_&&!mc_hold_&&!robot_hold_&&!fault_&&ready_&&last_health_.connected&&last_health_.owned&&zeroModeAllowed(last_health_)&&last_health_.robot_state_fresh&&
      !last_health_.estop&&last_health_.mc_fresh&&mcSourceFresh(now,clock_epoch_)&&nonterminal_zero_ack_>0&&
      policy_configured_&&nonterminal_zero_write_>last_nonzero_write_sequence_&&nonterminal_stationary_samples_>=policy_.minimum_new_samples&&nonterminal_first_stationary_>0&&
      last_capture_lower_-nonterminal_first_stationary_>=policy_.reentry_duration_s&&stationaryMotionWithinPolicy();
  }
  bool stationaryProof(const Stationary&s,double now)const {
    if(s.schema_version!=1||!s.usable||!s.nonzero_blocked||s.writer_commit_sequence!=commit_sequence_||
       s.applied_trajectory_id!=applied_trajectory_id_||!version(s.version,applied_version_)||s.execution_id!=execution_||
       s.control_epoch!=epoch_||s.sdk_session!=session_||s.sdk_arm_generation!=generation_||s.transport_mode!=mode_||
       s.zero_write_sequence!=nonterminal_zero_write_||s.zero_ack_at!=stamp(nonterminal_zero_ack_)||
       s.mc_clock_epoch!=clock_epoch_||s.time_basis!=(mode_=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock")||
       !mcTimeFresh(secs(s.source_stamp),secs(s.received_stamp),now)||secs(s.valid_until)<=now||
       (mode_=="live"&&!runtime_valid_)||!nonterminalStationary(now))return false;
    return std::any_of(stationary_history_.begin(),stationary_history_.end(),[&](const auto&old){return old==s;});
  }
  double stationaryDuration()const{return first_stationary_>0?std::max(0.,last_capture_lower_-first_stationary_):0.;}
  bool stationaryMotionWithinPolicy()const {
    struct Sample {const std::string& velocity_metric;double measured_linear_mps,measured_angular_radps,
      measured_planar_mps,measured_vertical_mps;};
    const Sample sample{velocity_metric_,linear_,angular_,planar_,vertical_};
    return d1max_planning_interfaces::stationaryVelocityWithin(sample,policy_.velocity_metric,
      policy_.linear_threshold_mps,policy_.angular_threshold_radps);
  }
  bool measuredStopped(double now)const{return stopped_&&policy_configured_&&stationaryMotionWithinPolicy()&&
    mc_bound_configured_&&stationary_samples_>=policy_.minimum_new_samples&&first_stationary_>0&&stationaryDuration()>=policy_.stationary_duration_s&&
    mcSourceFresh(now,clock_epoch_);}
  bool terminalStopMatches()const {
    return terminal_stop_&&!execution_.empty()&&terminal_stop_->execution_id==execution_&&
      terminal_stop_->sdk_session==session_&&terminal_stop_->sdk_arm_generation==generation_&&
      terminal_stop_->control_epoch==epoch_&&terminal_stop_->stop_request_id==request_&&
      version(terminal_stop_->version,binding_)&&terminal_stop_->transport_mode==mode_;
  }
  void retainTerminalStop(double now) {
    // Only actual SDK retirement + a measured post-zero stop can become a
    // terminal fact. A successful zero submission alone, an unresolved vendor
    // callback, or an approximate/stale MC sample cannot manufacture one.
    if(terminalStopMatches()||execution_.empty()||!stopping_||!stop_submitted_||
       !sdk_transactions_tracked_||pending_sdk_transactions_||sdk_transaction_uncertain_||!measuredStopped(now))return;
    terminal_stop_=currentStopReport(now);
  }
  bool terminalStopSatisfied(double now) {
    retainTerminalStop(now);
    return terminalStopMatches()||measuredStopped(now);
  }
  void invalidateTerminalStopForNonzero() {
    // Defensive writer-boundary guard: even an unexpected actual nonzero
    // attempt after completion creates a NEW stop obligation. Neither its
    // outcome nor a late ACK may borrow the earlier terminal certificate.
    terminal_stop_.reset();
    if(stopping_){stop_submitted_=false;resetStationary();}
  }
  void resetStationary(){stopped_=false;first_stationary_=0;stationary_samples_=0;
    if(stopping_)phase_=terminalStopMatches()?"stopped":"stopping";}
  const MotionProof* motionProof(const Demand&d,double now,const Prepared*witness=nullptr)const {
    if(curveInvalidated(d.version,d.trajectory_id,d.validation_sequence))return nullptr;
    if(!witness&&active_entry_&&d.sequence==active_entry_->demand.sequence&&version(d.version,active_entry_->demand.version))witness=&*active_entry_;
    const MotionProof* bound=nullptr;
    for(const auto&p:motion_proofs_)if(version(p.version,d.version)&&p.demand_sequence==d.sequence) {
      // SafeDemand names an exact verdict. A later positive recheck cannot
      // silently replace it (or extend its deadline), nor make it disappear.
      // Every later explicit invalid verdict still fences this older binding,
      // including an invalid result followed by yet another positive result.
      if(!p.valid)return nullptr;
      if(p.sequence==d.motion_validation_sequence)bound=&p;
    }
    if(!bound)return nullptr;
    const auto&p=*bound;
    if(witness) {
      if(p.handoff_id!=witness->handoff_id||p.entry_admission_sequence!=witness->entry_admission.sequence||
         p.entry_curve_pose!=witness->curve_entry_pose||p.entry_curve_twist!=witness->curve_entry_twist||
         p.entry_curve_time!=witness->curve_time||p.entry_position_error_m!=witness->position_error_m||
         p.entry_velocity_error_mps!=witness->velocity_error_mps)return nullptr;
    } else if(!p.handoff_id.empty()||p.entry_admission_sequence!=0)return nullptr;
    if(!p.valid||p.sequence!=d.motion_validation_sequence||p.braking_model_sha256!=d.braking_model_sha256||
       p.braking_model_sha256!=braking_model_sha_||p.execution_id!=d.execution_id||p.control_epoch!=d.control_epoch||
       p.sdk_session!=d.sdk_session||p.sdk_arm_generation!=d.sdk_arm_generation||p.transport_mode!=d.transport_mode||
       // The demand binds a minimum curve-proof sequence. Native records the
       // actual fresh same-curve evidence used for THIS command sweep; the
       // independent gate also checks its intervening invalid-evidence fence.
       p.trajectory_id!=d.trajectory_id||p.permit_sequence!=d.permit_sequence||p.trajectory_validation_sequence<d.validation_sequence||
       p.demand_source_stamp!=d.source_stamp||p.demand_body_source_stamp!=d.body_source_stamp||p.velocity!=d.velocity||
       p.frame_id!="d1max_loc_odom"||secs(d.valid_until)>secs(p.demand_valid_until)||
       !commandSourceFresh(d.source_stamp,now)||!commandSourceFresh(d.body_source_stamp,now)||
       !fresh(secs(p.check_end),now,.1)||
       secs(p.valid_until)<now||secs(p.valid_until)>secs(p.demand_valid_until)||
       secs(p.valid_until)-secs(p.check_end)>.25)return nullptr;
    return &p;
  }
  std::string session_,mode_,execution_,request_,confirmation_,phase_="idle",reason_,clock_epoch_;bool runtime_valid_=false,physically_verified_=false;
  bool task_follower_=false;
  TaskLease task_lease_;LocalCommand task_command_;std::deque<TaskLease>task_leases_;
  std::optional<LocalCommand>task_original_command_;
  bool task_have_command_=false,task_selected_=false;uint64_t task_resume_lease_floor_=0;
  TaskCommandTiming selected_task_timing_;
  TaskCommandExpiry task_command_expiry_;
  TaskCommandAdmission task_command_admission_;
  TaskCommandFeedbackLedger task_feedback_;
  std::optional<LocalCommand>pending_task_command_;
  uint64_t task_command_reorder_deferred_=0,task_command_reorder_completed_=0,task_command_reorder_discarded_=0;
  std::int64_t task_zero_source_barrier_ns_=0;
  int64_t task_command_source_floor_=0,task_body_source_floor_=0;
  struct CachedGeometry {Tagged message;MeasuredCurveEntryCache cache;};
  std::deque<CachedGeometry>geometry_;
  static Version taggedVersion(const Tagged&t) {
    Version v;v.schema_version=3;v.session_id=t.session_id;v.task_id=t.task_id;v.route_id=t.route_id;
    v.route_hash=t.route_hash;v.map_version_id=t.map_version_id;v.localization_epoch=t.localization_epoch;
    v.localization_seed_id=t.localization_seed_id;v.reference_generation=t.generation;v.segment_id=t.segment_id;
    v.anchor_id=t.anchor_id;v.anchor_revision=t.anchor_revision;v.context_sequence=t.context_sequence;
    v.map_geometry_revision=t.map_geometry_revision;return v;
  }
  double commandPipeline()const {return policy_configured_?policy_.command_pipeline_bound_s:0.;}
  bool commandSourceFresh(const builtin_interfaces::msg::Time&t,double now)const {
    return policy_configured_&&fresh(secs(t),now,commandPipeline());
  }
  builtin_interfaces::msg::Time commandDeadline(const builtin_interfaces::msg::Time&t)const {
    const auto ns=sourceNs(t)+static_cast<std::int64_t>(std::llround(commandPipeline()*1e9));
    builtin_interfaces::msg::Time out;out.sec=static_cast<std::int32_t>(ns/1000000000LL);
    out.nanosec=static_cast<std::uint32_t>(ns%1000000000LL);return out;
  }
  bool actualLocalFresh(double now)const {
    return local_&&local_->usable&&localContext(*local_,binding_)&&
      localProviderLease(*local_,now);
  }
  static bool localProviderLease(const Local&m,double now) {
    const auto source=sourceNs(m.source_stamp),deadline=sourceNs(m.valid_until);
    return m.schema_version==2&&m.usable&&m.source_stamp.nanosec<1000000000U&&
      m.valid_until.nanosec<1000000000U&&source>0&&deadline>source&&deadline-source<=100000000LL&&
      std::isfinite(now)&&now>=secs(m.source_stamp)&&now<=secs(m.valid_until);
  }
  std::string braking_model_sha_,kinematic_profile_="forward_yaw";
  ExecutionPolicy policy_;bool policy_configured_=false;
  Version binding_;Permit permit_;Demand demand_;std::deque<Permit> permits_;std::deque<MotionProof>motion_proofs_;std::deque<CurveProof>curve_proofs_;std::set<std::string>seen_,retired_sessions_;
  std::optional<Demand>pending_demand_;
  std::optional<Handoff>handoff_,last_handoff_;std::optional<Prepared>prepared_,active_entry_;std::optional<CommitAck>last_ack_;std::optional<Local>local_,last_local_envelope_;std::optional<Progress>progress_;
  std::deque<Local>local_history_;
  Version applied_version_;int64_t applied_trajectory_id_=0;
  uint64_t commit_sequence_=0,handoff_sequence_=0,ack_sequence_=0,prepared_sequence_floor_=0;
  uint64_t write_sequence_=0,last_nonzero_write_sequence_=0,stationary_sequence_=0,nonterminal_zero_write_=0,nonterminal_cutoff_=0;
  uint32_t nonterminal_stationary_samples_=0;
  double nonterminal_zero_ack_=0.,nonterminal_first_stationary_=0.;std::deque<Stationary>stationary_history_;
  bool mc_order_hold_=false;uint32_t mc_order_new_samples_=0;double mc_order_first_source_=0.,mc_order_barrier_source_=0.;
  bool selected_prepared_=false,selected_active_=false,ack_dirty_=false;double retained_incumbent_until_=0;
  builtin_interfaces::msg::Time selected_curve_until_,selected_motion_until_,demand_lease_until_stamp_;
  Health last_health_;
  bool sdk_transactions_tracked_=false,sdk_transaction_uncertain_=false;
  std::optional<Stop> terminal_stop_;
  uint32_t pending_sdk_transactions_=0;uint64_t retirement_sequence_=0;
  bool ready_=false,fault_=false,stopping_=false,stopped_=false,stop_submitted_=false,have_demand_=false,takeover_=false,takeover_taken_=false,ever_nonzero_=false,closing_=false,mc_bound_configured_=false,mc_hold_=false,robot_hold_=false,invalidated_accepted_demand_=false;
  bool software_estop_latched_=false,software_estop_release_confirmed_=false,software_estop_had_owned_=false;
  std::string software_estop_reason_;
  double software_estop_requested_at_=0.;std::int64_t software_estop_grant_barrier_ns_=0;
  uint64_t epoch_=0,generation_=0,permit_seq_=0,demand_seq_=0,state_seq_=0,stop_seq_=0,last_raw_=0,cutoff_=0;
  uint32_t stationary_samples_=0;
  double started_=0,ready_at_=0,stop_at_=0,submitted_=0,last_tick_=0,permit_at_=0,demand_at_=0,demand_lease_until_=0,last_source_=0,last_received_=0,first_stationary_=0,linear_=0,angular_=0;
  std::string velocity_metric_="spatial_norm_3d";
  double planar_=0.,vertical_=0.;
  double max_speed_=.3,max_yaw_=.5,mc_delay_bound_=0.,last_capture_lower_=0.,mc_resume_floor_=0.,robot_resume_floor_=0.;
};
}
