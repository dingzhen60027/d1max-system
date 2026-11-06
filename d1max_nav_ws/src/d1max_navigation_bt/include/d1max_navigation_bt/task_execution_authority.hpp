#pragma once

#include "d1max_navigation_bt/execution_owner.hpp"
#include <d1max_planning_interfaces/msg/task_execution_lease.hpp>
#include <d1max_planning_interfaces/msg/local_execution_status.hpp>

namespace d1max_navigation_bt {
// Task authority deliberately has no curve, native proof, predecessor CAS or
// handoff ledger. The local executor owns those facts; the SDK owns its grant.
class TaskExecutionAuthority {
public:
  using Lease=d1max_planning_interfaces::msg::TaskExecutionLease;
  using Status=d1max_planning_interfaces::msg::LocalExecutionStatus;
  TaskExecutionAuthority(exec3::Version task,std::string transport,double budget,
      exec3::StationaryLimits limits={},double recovery_stable=.6,
      bool keep_committed_route_on_local_failure=false) : binding_(std::move(task)),transport_(std::move(transport)),
      budget_(budget),limits_(limits),recovery_stable_(recovery_stable),
      keep_committed_route_on_local_failure_(keep_committed_route_on_local_failure) {
    limits_.validate();
    if(!exec3::sameTask(binding_,binding_)||binding_.task_id.empty()||binding_.route_id.empty()||
       binding_.route_hash.empty()||binding_.map_version_id.empty()||!binding_.localization_epoch||
       binding_.localization_seed_id.empty()||!std::isfinite(budget)||budget<=0.||
       !std::isfinite(recovery_stable)||recovery_stable<0.||recovery_stable>budget)
      throw std::invalid_argument("complete_task_authority_required");
    // ExecutionVersion's dynamic coordinates are intentionally absent.
    binding_.anchor_id.clear();binding_.reference_generation=0;binding_.context_sequence=0;
    binding_.map_geometry_revision=0;binding_.segment_id.clear();
  }
  const exec3::Version& grantVersion(double=0.)const{return binding_;}
  const std::string&executionId()const{return execution_;}
  uint64_t controlEpoch()const{return epoch_;}
  bool confirmed()const{return !execution_.empty();}
  bool admitted()const{return admitted_;}
  bool stopping()const{return stopping_;}
  bool stopped()const{return stopped_;}
  bool goalStopped()const{return goal_stop_&&stopped_;}
  bool requiresPhysicalStop()const{return submitted_&&!never_submitted_;}
  bool executionSoftwareRetired()const{return local_retired_&&(!submitted_||sdk_retired_);}
  bool needsReview(double now)const{return stopping_&&requiresPhysicalStop()&&!stopped_&&now-stop_at_>=budget_;}
  const std::string&phase()const{return phase_;}
  const std::string&reason()const{return reason_;}
  const std::string&failureReason()const{return failure_;}
  int64_t evidence(double now)const{return statusReady(now)?sourceNs(status_->body_source_stamp):0;}
  bool canConfirm(double now)const {
    return !stopping_&&!local_retired_&&failure_.empty()&&statusReady(now)&&
      !status_->software_retired&&status_->trajectory_id>=0&&status_->commit_sequence>0;
  }
  bool begin(const std::string&execution,const std::string&confirmation,uint64_t epoch,double now,
      const std::string&sdk) {
    if(!canConfirm(now)||execution.empty()||confirmation.empty()||!epoch||sdk.empty()||submitted_)return false;
    if(execution_.empty()){execution_=execution;confirmation_=confirmation;epoch_=epoch;sdk_=sdk;}
    else if(execution_!=execution||confirmation_!=confirmation||epoch_!=epoch||sdk_!=sdk)return false;
    phase_="arming";reason_="waiting_sdk_admission";submitted_=true;return true;
  }
  bool matches(const exec3::Grant::Request&r,const exec3::Grant::Response&a)const {
    return r.version==binding_&&a.version==r.version&&r.execution_id==execution_&&a.execution_id==execution_&&
      r.control_epoch==epoch_&&a.control_epoch==epoch_&&r.confirmation_id==confirmation_&&
      r.request_id==confirmation_&&a.request_id==r.request_id&&r.sdk_session==sdk_&&a.sdk_session==sdk_&&
      r.transport_mode==transport_&&a.transport_mode==transport_;
  }
  bool observeGrant(const exec3::Grant::Request&r,const exec3::Grant::Response&a,double now) {
    if(!matches(r,a))return false;
    if(a.accepted) {
      if(!a.sdk_arm_generation||(arm_&&arm_!=a.sdk_arm_generation))return false;
      arm_=a.sdk_arm_generation;admitted_=true;
      if(stopping_){phase_="stopping";reason_="late_sdk_admission_retiring";}
      return true;
    }
    if(a.definitely_not_admitted&&!stopping_&&!admitted_&&
       (a.reason=="robot_state_unavailable"||a.reason=="mc_capture_unavailable"||
        a.reason=="prior_stop_unconfirmed"||a.reason=="manual_handoff_waiting_task_fence"||
        a.reason=="manual_control_busy")) {
      // The writer explicitly admitted nothing. A preceding task's current
      // stationary window may still be settling (or restart on MC noise).
      // Keep this route/intent and retry through the existing bounded
      // GrantAdmissionWait; do not turn a transient stop wait into task failure.
      // Manual handback is the same definitely-unadmitted wait: the user may
      // submit a fresh goal while releasing the last jog. Preserve that goal,
      // but never infer control ownership or resume the pre-takeover task.
      // A retry does not bypass stopping, acquire ownership or issue motion.
      submitted_=false;phase_="waiting_sdk";reason_=a.reason;return true;
    }
    if(a.definitely_not_admitted&&!admitted_) {submitted_=false;sdk_retired_=true;}
    failure_="sdk_grant_rejected:"+a.reason;stop(failure_,now);return true;
  }
  bool observe(const Status&s,double now) {
    if(s.schema_version!=1||!exec3::sameTask(binding_,s.version)||s.transport_mode!=transport_||
       s.sequence<=status_sequence_||!validStamp(s.source_stamp)||exec3::seconds(s.source_stamp)>now||
       (!s.software_retired&&!exec3::fresh(exec3::seconds(s.source_stamp),now,.5)))return false;
    if(status_&&sourceNs(s.source_stamp)<sourceNs(status_->source_stamp))return false;
    // A preview has no execution. Once bound, old preview feedback may not
    // overwrite execution feedback, even when its sequence is numerically high.
    if(confirmed()&&(s.execution_id!=execution_||s.control_epoch!=epoch_))return false;
    if(!confirmed()&&(!s.execution_id.empty()||s.control_epoch))return false;
    if(s.software_retired&&(!stopping_||exec3::seconds(s.source_stamp)<stop_at_))return false;
    if(s.ready&&(!validStamp(s.body_source_stamp)||!validStamp(s.body_valid_until)||
       exec3::seconds(s.body_source_stamp)>exec3::seconds(s.source_stamp)||
       exec3::seconds(s.body_valid_until)<exec3::seconds(s.source_stamp)||
       sourceNs(s.body_valid_until)-sourceNs(s.body_source_stamp)>100000000LL))return false;
    // Coarse feedback retains the executor's original at-publication fact.
    // A later 10 Hz task callback is not a second body consumption deadline.
    // canConfirm still requires this unchanged body lease to be live NOW;
    // ordinary feedback never renews it or authorizes a motion command.
    status_=s;status_sequence_=s.sequence;local_retired_|=stopping_&&s.software_retired;
    if(confirmed()&&!stopping_&&s.request_stop&&s.at_goal)stop("goal_reached",now,true);
    return true;
  }
  bool observe(const exec3::SDKState&s,double now) {
    if(!confirmed()||s.execution_id!=execution_||s.control_epoch!=epoch_||s.sdk_session!=sdk_||
       !exec3::sameTask(binding_,s.version)||s.transport_mode!=transport_||!s.sdk_arm_generation||
       (arm_&&arm_!=s.sdk_arm_generation)||s.sequence<=sdk_sequence_||
       !exec3::fresh(exec3::seconds(s.source_stamp),now,.5))return false;
    arm_=s.sdk_arm_generation;admitted_=true;sdk_sequence_=s.sequence;sdk_state_=s;
    const bool first_takeover=!ever_owned_&&submitted_&&s.phase=="taking_control"&&s.reason.empty()&&
      !s.control_owned&&!s.grant_ready&&!s.fault_latched&&motionModeReady(s);
    // First accepted grant precedes the SDK's one TakeControl transaction and
    // two RobotState ownership confirmations. This is not a former owner's
    // loss, including a robot already in confirmed STAIR at task admission.
    // Do not fabricate GENERAL readiness for that state. SDK still bounds
    // the transaction and the task wait budget applies.
    ever_owned_|=s.control_owned;
    const bool soft=s.phase=="holding"&&(s.reason=="mc_stale"||s.reason=="robot_state_stale")&&
      s.control_owned&&!s.fault_latched&&!s.grant_ready;
    // A mode transaction belongs to this admitted task. It revokes velocity,
    // not the route or the task; only the writer can confirm the new mode.
    const bool mode_hold=s.mode_transition_active&&s.control_owned&&!s.fault_latched&&!s.grant_ready;
    if(!stopping_&&!first_takeover&&(s.fault_latched||!s.control_owned||(!motionModeReady(s)&&!soft&&!mode_hold))) {
      failure_="sdk_authority_lost:"+s.reason;stop(failure_,now);
    }return true;
  }
  bool observe(const exec3::Retirement&r,double now) {
    if(!stopping_||r.schema_version!=1||!exec3::sameTask(binding_,r.version)||r.execution_id!=execution_||
       r.control_epoch!=epoch_||r.sdk_session!=sdk_||r.sdk_arm_generation!=arm_||
       r.grant_request_id!=confirmation_||r.transport_mode!=transport_||r.sequence<=retirement_sequence_||
       !validStamp(r.source_stamp)||exec3::seconds(r.source_stamp)>now||
       !r.nonzero_blocked||!r.software_retired||r.pending_sdk_transactions)return false;
    retirement_sequence_=r.sequence;sdk_retired_=true;never_submitted_|=r.motion_never_submitted;return true;
  }
  bool observe(const exec3::Stop&s,double now) {
    // This is an immutable completion event for this stopped execution, not
    // a live stationary certificate for the next motion. A delayed delivery
    // keeps its original MC time and interval; identity, stop boundary and
    // measured evidence below still have to match. Live SDK/body freshness
    // is checked independently before any new execution can be authorized.
    if(!stopping_||!exec3::sameTask(binding_,s.version)||s.execution_id!=execution_||s.control_epoch!=epoch_||
       s.sdk_session!=sdk_||s.sdk_arm_generation!=arm_||s.stop_request_id!=confirmation_||
       s.sequence<=stop_sequence_||s.transport_mode!=transport_||
       !validStamp(s.source_stamp)||exec3::seconds(s.source_stamp)>now||
       !s.mc_raw_stamp_ns||s.mc_clock_epoch.empty()||!s.nonzero_blocked||!s.stop_submitted||!s.measured_stop_confirmed||
       !std::isfinite(s.stationary_duration_sec)||s.stationary_duration_sec<limits_.stop_duration||
       exec3::seconds(s.source_stamp)-s.stationary_duration_sec<stop_at_||s.stationary_samples<limits_.minimum_samples||
       !limits_.acceptsMotion(s)||
       s.time_basis!=(transport_=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock"))return false;
    stop_sequence_=s.sequence;stopped_=true;phase_="terminal";reason_=goal_stop_?"goal_reached":"stopped";return true;
  }
  void stop(const std::string&reason,double now,bool goal=false) {
    if(stopping_)return;
    stopping_=true;goal_stop_=goal&&confirmed();stop_at_=now;phase_="stopping";reason_=reason;
  }
  // Only the navigator's validated, same-SDK retained operator event invokes
  // this. A delayed terminal stop is a completed OLD task fact, not evidence
  // that the robot is stationary now or permission for any new task.
  void stopForManualTakeover(double source_time) {
    if(!std::isfinite(source_time)||source_time<=0.)return;
    // A task already cancelled before manual takeover keeps its original
    // stopping boundary. SDK stop() likewise preserves that earlier request;
    // replacing it would reject the genuine stationary interval in its cache.
    const double boundary=stopping_?std::min(stop_at_,source_time):source_time;
    stop("sdk_manual_takeover",source_time);
    goal_stop_=false;stop_at_=boundary;
  }
  // A correlated FollowRoute terminal result can precede the BT tick that
  // calls stop(). Retain that explicit worker proof in either callback order;
  // it does not retire the SDK or prove physical stopping.
  void noteLocalSoftwareRetired(){local_retired_=true;}
  Lease tick(int64_t now_ns,bool inputs_ready,bool authorized) {
    const double now=now_ns*1e-9;
    const bool sdk_ready=sdk_state_&&exec3::fresh(exec3::seconds(sdk_state_->source_stamp),now,.5)&&
      sdk_state_->grant_ready&&sdk_state_->control_owned&&motionModeReady(*sdk_state_)&&
      !sdk_state_->mode_transition_active&&!sdk_state_->fault_latched;
    const bool initial_ready=ever_allowed_||(inputs_ready&&statusReady(now)&&!status_->holding&&!status_->request_stop);
    const bool base_allowed=authorized&&admitted_&&!stopping_&&!local_retired_&&failure_.empty()&&initial_ready&&sdk_ready;
    if(admitted_&&authorized&&!stopping_) {
      // A task lease is operator/SDK authority, not an instantaneous collision
      // or body certificate. Once initially ready, the 20 Hz executor and SDK
      // independently stop on their ORIGINAL body/curve/command deadlines.
      // Feeding each local wait back into task authority turned a one-frame
      // proof/source gap into a mandatory 0.6 s stop, even after local recovery.
      // Only initial readiness or the bound SDK's readiness starts this episode.
      if(!initial_ready||!sdk_ready) {
        if(blocked_at_ns_<0)blocked_at_ns_=now_ns;
        recovered_at_ns_=-1;recovered_samples_=0;
      }
      // The original episode deadline wins over a last-moment third good
      // sample. Neither local feedback nor a short SDK good flash renews it.
      if(!keep_committed_route_on_local_failure_&&blocked_at_ns_>=0&&
         static_cast<double>(now_ns-blocked_at_ns_)>=budget_*1e9) {
        failure_="execution_local_recovery_timeout";stop(failure_,now);
      } else if(initial_ready&&sdk_ready&&blocked_at_ns_>=0) {
        if(!ever_allowed_)blocked_at_ns_=recovered_at_ns_=-1; // initial arming is not a recovery episode
        else {
          // These are new SDK control-session readiness facts, not fabricated
          // body measurements. Repeated source/sequence cannot satisfy recovery.
          const auto source=sourceNs(sdk_state_->source_stamp);
          if(recovered_at_ns_<0){recovered_at_ns_=now_ns;recovered_source_=source;recovered_samples_=1;}
          else if(source>recovered_source_){recovered_source_=source;++recovered_samples_;}
          if(static_cast<double>(now_ns-recovered_at_ns_)>=recovery_stable_*1e9&&recovered_samples_>=3)
            blocked_at_ns_=recovered_at_ns_=-1;
        }
      }
    }
    const bool allowed=base_allowed&&!stopping_&&(blocked_at_ns_<0||!ever_allowed_);
    ever_allowed_|=allowed;
    if(!stopping_) {
      phase_=!confirmed()?"preview":allowed?"tracking":"holding";
      reason_=!confirmed()?"waiting_execution_confirmation":allowed?"task_tracking":
        !admitted_?"waiting_sdk_admission":!sdk_ready?(sdk_state_&&sdk_state_->mode_transition_active?
          "mode_transition:"+sdk_state_->reason:"waiting_sdk_state"):
        !initial_ready?(!inputs_ready?"waiting_local_state":"waiting_local_executor"):"waiting_stable_sdk_state";
    }
    Lease p;p.schema_version=1;p.version=binding_;p.execution_id=execution_;p.control_epoch=epoch_;
    p.confirmation_id=confirmation_;p.sdk_session=sdk_;p.sdk_arm_generation=arm_;p.sequence=++lease_sequence_;
    p.source_stamp=nsStamp(now_ns);p.valid_until=nsStamp(now_ns+350000000LL);
    p.allowed=allowed&&!stopping_;p.revoked=stopping_;p.phase=phase_;p.reason=reason_;p.transport_mode=transport_;
    return p;
  }
private:
  static bool motionModeReady(const exec3::SDKState&s){return s.general_low_speed_confirmed||
    (s.motion_mode=="stair"&&s.motion_mode_confirmed);}
  static bool validStamp(const builtin_interfaces::msg::Time&t){return t.sec>=0&&t.nanosec<1000000000U&&sourceNs(t)>0;}
  static int64_t sourceNs(const builtin_interfaces::msg::Time&t){return int64_t(t.sec)*1000000000LL+t.nanosec;}
  static builtin_interfaces::msg::Time nsStamp(int64_t ns){builtin_interfaces::msg::Time t;t.sec=ns/1000000000LL;t.nanosec=ns%1000000000LL;return t;}
  bool statusFresh(double now)const{return status_&&exec3::fresh(exec3::seconds(status_->source_stamp),now,.5);}
  bool statusReady(double now)const{return statusFresh(now)&&status_->ready&&exec3::seconds(status_->body_valid_until)>=now;}
  exec3::Version binding_;std::string transport_,execution_,confirmation_,sdk_,phase_="preview",reason_,failure_;
  double budget_,stop_at_=0.;exec3::StationaryLimits limits_;double recovery_stable_;
  uint64_t epoch_=0,arm_=0,status_sequence_=0,sdk_sequence_=0,stop_sequence_=0,retirement_sequence_=0,lease_sequence_=0;
  bool admitted_=false,submitted_=false,never_submitted_=false,stopping_=false,stopped_=false,goal_stop_=false;
  bool sdk_retired_=false,local_retired_=false;
  bool ever_allowed_=false,ever_owned_=false;
  int64_t blocked_at_ns_=-1,recovered_at_ns_=-1,recovered_source_=0;unsigned recovered_samples_=0;
  std::optional<Status>status_;std::optional<exec3::SDKState>sdk_state_;
  bool keep_committed_route_on_local_failure_=false;
};
}  // namespace d1max_navigation_bt
