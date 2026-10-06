#pragma once
#include <d1max_planning_interfaces/msg/execution_permit.hpp>
#include <d1max_planning_interfaces/msg/motion_demand.hpp>
#include <d1max_planning_interfaces/msg/motion_validation.hpp>
#include <d1max_planning_interfaces/msg/sdk_execution_state.hpp>
#include <d1max_planning_interfaces/msg/stop_report.hpp>
#include <d1max_planning_interfaces/msg/execution_handoff_grant.hpp>
#include <d1max_planning_interfaces/msg/prepared_motion_demand.hpp>
#include <d1max_planning_interfaces/msg/execution_commit_ack.hpp>
#include <d1max_planning_interfaces/msg/sdk_stationary_evidence.hpp>
#include <d1max_planning_interfaces/msg/local_navigation_state.hpp>
#include <d1max_planning_interfaces/msg/trajectory_validation.hpp>
#include <d1max_planning_interfaces/msg/tracking_progress.hpp>
#include <d1max_planning_interfaces/srv/execution_grant.hpp>
#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <set>
#include <string>
#include <stdexcept>
#include <cstdint>
#include "execution_policy.hpp"

namespace d1monitor::execution3 {
using Version=d1max_planning_interfaces::msg::ExecutionVersion;
using Permit=d1max_planning_interfaces::msg::ExecutionPermit;
using Demand=d1max_planning_interfaces::msg::MotionDemand;
using MotionProof=d1max_planning_interfaces::msg::MotionValidation;
using State=d1max_planning_interfaces::msg::SDKExecutionState;
using Stop=d1max_planning_interfaces::msg::StopReport;
using Grant=d1max_planning_interfaces::srv::ExecutionGrant;
using Handoff=d1max_planning_interfaces::msg::ExecutionHandoffGrant;
using Prepared=d1max_planning_interfaces::msg::PreparedMotionDemand;
using CommitAck=d1max_planning_interfaces::msg::ExecutionCommitAck;
using Stationary=d1max_planning_interfaces::msg::SDKStationaryEvidence;
using Local=d1max_planning_interfaces::msg::LocalNavigationState;
using CurveProof=d1max_planning_interfaces::msg::TrajectoryValidation;
using Progress=d1max_planning_interfaces::msg::TrackingProgress;
// Keep the ROS source clock in integer nanoseconds at real callbacks.  Double
// seconds remain supported for approximate SDK clocks and small-time tests.
// Converting sec + nanosec separately is not equivalent to ROS ns / 1e9 at
// Unix epochs: the same exact source can otherwise look 238 ns in the future.
struct SourceClock {
  double seconds_value=0.;
  std::optional<std::int64_t> exact_ns;
  SourceClock(double value=0.):seconds_value(value){}
  static SourceClock fromNanoseconds(std::int64_t value) {
    SourceClock out(static_cast<double>(value)/1e9);out.exact_ns=value;return out;
  }
  operator double()const{return seconds_value;}
};
inline std::int64_t nanoseconds(const builtin_interfaces::msg::Time&t){
  return static_cast<std::int64_t>(t.sec)*1000000000LL+t.nanosec;
}
inline double secs(const builtin_interfaces::msg::Time&t){return static_cast<double>(nanoseconds(t))/1e9;}
inline builtin_interfaces::msg::Time stamp(double t){
  builtin_interfaces::msg::Time out;out.sec=static_cast<int32_t>(std::floor(t));
  out.nanosec=static_cast<uint32_t>(std::max(0.,(t-out.sec)*1e9));return out;
}
inline builtin_interfaces::msg::Time stamp(SourceClock t){
  if(!t.exact_ns)return stamp(t.seconds_value);
  builtin_interfaces::msg::Time out;out.sec=static_cast<int32_t>(*t.exact_ns/1000000000LL);
  const auto remainder=*t.exact_ns%1000000000LL;
  if(remainder<0){--out.sec;out.nanosec=static_cast<uint32_t>(remainder+1000000000LL);}
  else out.nanosec=static_cast<uint32_t>(remainder);
  return out;
}
inline SourceClock after(SourceClock t,double duration){
  if(!t.exact_ns)return SourceClock(t.seconds_value+duration);
  return SourceClock::fromNanoseconds(*t.exact_ns+static_cast<std::int64_t>(duration*1e9));
}
inline int compare(const builtin_interfaces::msg::Time&t,SourceClock now){
  if(now.exact_ns){const auto value=nanoseconds(t);return value<*now.exact_ns?-1:value>*now.exact_ns?1:0;}
  const double value=secs(t);return value<now.seconds_value?-1:value>now.seconds_value?1:0;
}
inline double elapsed(const builtin_interfaces::msg::Time&end,const builtin_interfaces::msg::Time&begin){
  return static_cast<double>(nanoseconds(end)-nanoseconds(begin))/1e9;
}
inline bool fresh(double source,double now,double age){
  return std::isfinite(source)&&std::isfinite(now)&&source>0&&now>=source&&now-source<=age;
}
inline bool fresh(double source,SourceClock now,double age){return fresh(source,now.seconds_value,age);}
inline bool fresh(SourceClock source,double now,double age){return fresh(source.seconds_value,now,age);}
inline bool fresh(SourceClock source,SourceClock now,double age){
  if(source.exact_ns&&now.exact_ns){
    return *source.exact_ns>0&&*now.exact_ns>=*source.exact_ns&&std::isfinite(age)&&age>=0.&&
      *now.exact_ns-*source.exact_ns<=static_cast<std::int64_t>(age*1e9);
  }
  return fresh(source.seconds_value,now.seconds_value,age);
}
inline bool fresh(const builtin_interfaces::msg::Time&source,SourceClock now,double age){
  if(source.nanosec>=1000000000U)return false;
  return fresh(SourceClock::fromNanoseconds(nanoseconds(source)),now,age);
}
inline bool fresh(const builtin_interfaces::msg::Time&source,const builtin_interfaces::msg::Time&now,double age){
  if(now.nanosec>=1000000000U)return false;
  return fresh(source,SourceClock::fromNanoseconds(nanoseconds(now)),age);
}

inline builtin_interfaces::msg::Time earlier(const builtin_interfaces::msg::Time&a,const builtin_interfaces::msg::Time&b){
  return nanoseconds(a)<=nanoseconds(b)?a:b;
}
inline bool task(const Version&a,const Version&b){return a.schema_version==3&&b.schema_version==3&&a.session_id==b.session_id&&
 a.task_id==b.task_id&&a.route_id==b.route_id&&a.route_hash==b.route_hash&&a.map_version_id==b.map_version_id&&
 a.localization_epoch==b.localization_epoch&&a.localization_seed_id==b.localization_seed_id;}
inline bool version(const Version&a,const Version&b){return task(a,b)&&a.reference_generation==b.reference_generation&&
 a.segment_id==b.segment_id&&a.anchor_id==b.anchor_id&&a.anchor_revision==b.anchor_revision&&
 a.context_sequence==b.context_sequence&&a.map_geometry_revision==b.map_geometry_revision;}
inline bool complete(const Version&a){return a.schema_version==3&&!a.session_id.empty()&&!a.task_id.empty()&&
 !a.route_id.empty()&&a.route_hash.size()==64&&!a.map_version_id.empty()&&a.localization_epoch>0&&
 !a.localization_seed_id.empty()&&a.reference_generation>0&&!a.segment_id.empty()&&!a.anchor_id.empty()&&a.anchor_revision>0&&a.context_sequence>0;}
struct Health {bool connected=false,owned=false,general=false,mc_fresh=false,estop=false;};

// One writer transport. It cannot select goals, replan or grant itself a lease.
// MC raw deltas remain explicit; live proof is labelled approximate, never PTP.
class Transport {
public:
  Transport(std::string session,std::string mode,bool physically_accepted=false,double max_speed=.3,double max_yaw=.5,std::string model_sha={})
    :session_(std::move(session)),mode_(std::move(mode)),accepted_(physically_accepted),braking_model_sha_(std::move(model_sha)),max_speed_(max_speed),max_yaw_(max_yaw){
      if(session_.empty()||(mode_!="live"&&mode_!="isolated_mock")||!std::isfinite(max_speed_)||max_speed_<=0||max_speed_>.3||
         !std::isfinite(max_yaw_)||max_yaw_<=0||max_yaw_>.5)throw std::invalid_argument("invalid_execution_transport_profile");}
  void configureBrakingModel(std::string hash,double speed=.3,double yaw=.5) {
    if(!execution_.empty()||hash.size()!=64||hash.find_first_not_of("0123456789abcdef")!=std::string::npos||
       !std::isfinite(speed)||speed<=0||speed>.3||!std::isfinite(yaw)||yaw<=0||yaw>.5)
      throw std::invalid_argument("braking_model_must_be_validated_before_execution");
    braking_model_sha_=std::move(hash);max_speed_=speed;max_yaw_=yaw;
  }
  void configureExecutionPolicy(const ExecutionPolicy&policy,double reaction_bound) {
    if(!execution_.empty())throw std::invalid_argument("execution_policy_must_precede_grant");
    policy.validate(reaction_bound);policy_=policy;policy_configured_=true;
  }
  void configureMcCaptureBound(double delay,const std::string&basis) {
    // A host receipt anchor is not an exact acquisition clock. Live requires
    // the measured bound; private mock requires an explicit fixture bound.
    if(!execution_.empty()||!std::isfinite(delay)||delay<0||delay>.25||
       (mode_=="live"&&delay<=0)||basis!=(mode_=="live"?
        "source_delta_host_anchor_approximate":"isolated_simulated_source_clock"))
      throw std::invalid_argument("explicit_mc_capture_time_bound_required");
    mc_delay_bound_=delay;mc_bound_configured_=true;
  }
  Grant::Response grant(const Grant::Request&r,SourceClock now,const Health&h) {
    Grant::Response out;out.sdk_session=session_;out.sdk_arm_generation=generation_;
    if(closing_&&r.activate){out.reason="writer_shutting_down";return out;}
    if(!complete(r.version)||r.sdk_session!=session_||r.transport_mode!=mode_||
       !fresh(r.source_stamp,now,.5)||r.request_id.empty()||r.execution_id.empty()||r.control_epoch==0){out.reason="grant_binding_or_time_invalid";return out;}
    if(!r.activate){
      if(r.execution_id!=execution_||r.control_epoch!=epoch_){out.reason="stale_revoke";return out;}
      stop("owner_revoked",now);out.accepted=true;out.reason="stopping_not_yet_measured";return out;
    }
    if(r.request_id==request_) {
      out.accepted=task(binding_,r.version)&&r.execution_id==execution_&&r.control_epoch==epoch_&&!fault_;
      out.duplicate=true;out.reason=out.accepted?"same_grant_transaction":"request_id_conflict";return out;
    }
    if(mode_=="live"&&!accepted_){out.reason="physical_acceptance_pending";return out;}
    if(!mc_bound_configured_){out.reason="mc_capture_time_bound_unavailable";return out;}
    if(!policy_configured_){out.reason="measured_execution_policy_unavailable";return out;}
    if(!h.connected||!h.general||!h.mc_fresh||h.estop||fault_||(!execution_.empty()&&!measuredStopped(now))) {
      out.reason="writer_not_ready_or_prior_not_stopped";return out;
    }
    if(r.control_epoch<=epoch_||seen_.count(r.request_id)||seen_.size()>=1024){out.reason="grant_replay";return out;}
    binding_=r.version;execution_=r.execution_id;epoch_=r.control_epoch;request_=r.request_id;seen_.insert(request_);
    ++generation_;permit_seq_=demand_seq_=0;started_=last_tick_=now;permit_at_=demand_at_=0;stopping_=stopped_=false;
    stop_submitted_=false;first_stationary_=0;stationary_samples_=0;have_demand_=ever_nonzero_=mc_hold_=false;mc_resume_floor_=0.;
    permits_.clear();motion_proofs_.clear();pending_demand_.reset();permit_=Permit{};demand_=Demand{};
    handoff_.reset();prepared_.reset();active_entry_.reset();last_handoff_.reset();last_ack_.reset();commit_sequence_=handoff_sequence_=ack_sequence_=prepared_sequence_floor_=0;
    selected_prepared_=selected_active_=ack_dirty_=false;applied_version_=Version{};applied_trajectory_id_=0;
    retained_incumbent_until_=0.;curve_proofs_.clear();progress_.reset();
    clearNonterminalStationary();write_sequence_=last_nonzero_write_sequence_=stationary_sequence_=0;
    mc_order_hold_=false;mc_order_new_samples_=0;mc_order_first_source_=mc_order_barrier_source_=0.;
    takeover_=!h.owned;takeover_taken_=false;ready_=h.owned;ready_at_=ready_?static_cast<double>(now):0.;phase_=ready_?"ready":"taking_control";reason_.clear();
    out.accepted=true;out.sdk_arm_generation=generation_;out.reason="grant_queued";return out;
  }
  bool takeControlOnce(){if(!takeover_||takeover_taken_||stopping_||closing_)return false;takeover_taken_=true;return true;}
  void submissionFailed(SourceClock now){fault_=true;stop("sdk_write_outcome_unknown",now);}
  void fault(const std::string&why,SourceClock now){fault_=true;stop(why,now);}
  void telemetryRestartBeforeFirstMotion(SourceClock now) {
    if(ever_nonzero_){fault("mc_reconfigured_after_motion",now);return;}
    clock_epoch_.clear();last_raw_=0;last_source_=last_received_=0;ready_=false;ready_at_=0;
  }
  bool permit(const Permit&p,SourceClock now) {
    if(execution_.empty()||p.execution_id!=execution_||p.control_epoch!=epoch_||!task(binding_,p.version)||
       p.transport_mode!=mode_||p.sdk_session!=session_||p.sdk_arm_generation!=generation_||
       p.sequence<=permit_seq_||!fresh(p.source_stamp,now,.35)||
       compare(p.valid_until,now)<0||elapsed(p.valid_until,p.source_stamp)>.35||
       (p.allowed&&!p.geometry_committed))return false;
    // A pre-gap permission must not revive a pre-gap command. Revocations
    // and HOLD heartbeats still reach the owner even while MC is unavailable.
    if(p.allowed&&!p.revoked&&p.phase!="stopping"&&p.phase!="terminal"&&
       (mc_hold_||secs(p.source_stamp)<=mc_resume_floor_))return false;
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
  bool localState(const Local&m,SourceClock now) {
    if(m.schema_version!=1||m.session_id.empty()||m.map_version_id.empty()||m.localization_epoch==0||m.localization_seed_id.empty())return false;
    if(!execution_.empty()&&(m.session_id!=binding_.session_id||m.map_version_id!=binding_.map_version_id||
       m.localization_epoch!=binding_.localization_epoch||m.localization_seed_id!=binding_.localization_seed_id))return false;
    if(!m.usable){local_.reset();return false;}
    const auto&o=m.local_odometry;
    if(!fresh(m.source_stamp,now,.1)||!fresh(m.posterior_stamp,now,.25)||!fresh(m.imu_stamp,now,.25)||
       nanoseconds(m.posterior_stamp)>nanoseconds(m.source_stamp)||nanoseconds(m.imu_stamp)>nanoseconds(m.source_stamp)||
       !std::isfinite(m.extrapolation_sec)||m.extrapolation_sec<0||m.extrapolation_sec>.1||
       std::abs(elapsed(m.source_stamp,m.imu_stamp)-m.extrapolation_sec)>1e-5||o.header.stamp!=m.source_stamp||
       o.header.frame_id!="d1max_loc_odom"||o.child_frame_id!="d1max_loc_base_link"||!poseFinite(o.pose.pose)||!twistFinite(o.twist.twist))return false;
    if(local_&&nanoseconds(m.source_stamp)<=nanoseconds(local_->source_stamp))return false;
    local_=m;return true;
  }
  bool trajectoryValidation(const CurveProof&p,SourceClock now) {
    if(!complete(p.version)||!task(binding_,p.version)||p.transport_mode!=mode_||p.sequence==0||
       !fresh(p.check_end,now,.25)||nanoseconds(p.check_begin)>nanoseconds(p.check_end))return false;
    for(const auto&old:curve_proofs_)if(version(old.version,p.version)&&old.trajectory_id==p.trajectory_id&&
        p.sequence<=old.sequence&&(p.valid||p.sequence==old.sequence))return false;
    curve_proofs_.push_back(p);if(curve_proofs_.size()>32)curve_proofs_.pop_front();return true;
  }
  bool trackingProgress(const Progress&p,SourceClock now) {
    if(p.schema_version!=2||!p.valid||p.header.frame_id!="d1max_loc_odom"||!fresh(p.header.stamp,now,.1)||
       p.session_id!=binding_.session_id||p.task_id!=binding_.task_id||p.route_id!=binding_.route_id||
       p.route_hash!=binding_.route_hash||p.map_version_id!=binding_.map_version_id||p.localization_epoch!=binding_.localization_epoch||
       p.localization_seed_id!=binding_.localization_seed_id||!poseFinite(p.pose)||!twistFinite(p.twist)||
       !std::isfinite(p.curve_time)||p.curve_time<0||!std::isfinite(p.arc_length)||p.arc_length<0)return false;
    if(progress_&&nanoseconds(p.header.stamp)<=nanoseconds(progress_->header.stamp))return false;
    progress_=p;return true;
  }
  bool handoff(const Handoff&g,SourceClock now) {
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
    if(closing_||stopping_||mc_hold_||!ready_||!last_health_.mc_fresh||g.schema_version!=2||(!reentry&&!continuous)||g.handoff_id.empty()||
       g.handoff_id.size()>128||g.sequence<=handoff_sequence_||g.expected_commit_sequence!=commit_sequence_||commit_sequence_==0||
       !sameGrantIdentity(g.incumbent)||!sameGrantIdentity(g.candidate)||!version(g.incumbent.version,applied_version_)||
       g.incumbent.trajectory_id!=applied_trajectory_id_||!version(g.incumbent.version,permit_.version)||
       g.incumbent.trajectory_id!=permit_.trajectory_id||permit_.revoked||g.incumbent.revoked||
       (continuous&&(!permit_.allowed||permit_.phase!="tracking"||!g.incumbent.allowed||g.incumbent.phase!="tracking"))||
       (reentry&&(permit_.allowed||permit_.phase!="holding"||g.incumbent.allowed||g.incumbent.phase!="holding"||
          g.retain_incumbent_until!=g.source_stamp||nanoseconds(g.valid_until)>nanoseconds(g.stationary_evidence.valid_until)||
          nanoseconds(g.candidate.valid_until)>nanoseconds(g.stationary_evidence.valid_until)||!stationaryProof(g.stationary_evidence,now)))||
       !g.candidate.allowed||g.candidate.revoked||g.candidate.phase!="tracking"||
       g.candidate.sequence<=g.incumbent.sequence||g.candidate.validation_sequence==0||
       !fresh(g.source_stamp,now,.25)||compare(g.valid_until,now)<=0||elapsed(g.valid_until,g.source_stamp)>.250001||
       nanoseconds(g.transition_deadline)>nanoseconds(g.valid_until)||compare(g.transition_deadline,now)<=0||
       (continuous&&(nanoseconds(g.retain_incumbent_until)>nanoseconds(g.incumbent.valid_until)||
         nanoseconds(g.retain_incumbent_until)>nanoseconds(permit_.valid_until)||nanoseconds(g.retain_incumbent_until)<=nanoseconds(g.source_stamp)))||
       !fresh(g.candidate.source_stamp,now,.35)||compare(g.candidate.valid_until,now)<=0||
       elapsed(g.candidate.valid_until,g.candidate.source_stamp)>.35)return false;
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
  bool preparedDemand(const Prepared&p,SourceClock now) {
    if(!handoff_||p.schema_version!=1||p.handoff_id!=handoff_->handoff_id||p.grant_sequence!=handoff_->sequence||
       p.expected_commit_sequence!=commit_sequence_||p.expected_commit_sequence!=handoff_->expected_commit_sequence||
       compare(handoff_->transition_deadline,now)<=0||!sameGrantIdentity(handoff_->candidate)||
       !version(p.demand.version,handoff_->candidate.version)||p.demand.trajectory_id!=handoff_->candidate.trajectory_id||
       p.demand.permit_sequence!=handoff_->candidate.sequence||p.demand.sequence<=prepared_sequence_floor_||!demandBinding(p.demand)||
       !fresh(p.demand.source_stamp,now,.1)||!fresh(p.demand.body_source_stamp,now,.1)||
       compare(p.demand.valid_until,now)<=0||elapsed(p.demand.valid_until,p.demand.source_stamp)>.100001||
       !preparedEntry(p,now))return false;
    if(prepared_&&p.demand.sequence<prepared_->demand.sequence)return false;
    if(prepared_&&p.demand.sequence==prepared_->demand.sequence&&
       (!sameOriginalDemand(p.demand,prepared_->demand)||p.entry_admission!=prepared_->entry_admission||
        p.measured_pose!=prepared_->measured_pose||p.measured_twist!=prepared_->measured_twist||
        p.entry_source_stamp!=prepared_->entry_source_stamp||p.curve_entry_pose!=prepared_->curve_entry_pose||
        p.curve_entry_twist!=prepared_->curve_entry_twist||p.curve_time!=prepared_->curve_time))return false;
    prepared_=p;return true;
  }
  bool demand(const Demand&d,SourceClock now) {
    if(closing_||!ready_||stopping_||mc_hold_||!last_health_.mc_fresh||!permit_.allowed||
       secs(d.source_stamp)<=mc_resume_floor_||secs(d.body_source_stamp)<=mc_resume_floor_||
       !version(permit_.version,d.version)||d.execution_id!=execution_||
       d.control_epoch!=epoch_||d.sdk_session!=session_||d.sdk_arm_generation!=generation_||d.transport_mode!=mode_||
       d.sequence<demand_seq_||d.trajectory_id!=permit_.trajectory_id||!fresh(d.source_stamp,now,.25)||
       !fresh(d.body_source_stamp,now,.4)||compare(d.valid_until,now)<0||elapsed(d.valid_until,d.source_stamp)>.3)return false;
    const bool evidence_upgrade=demand_seq_>0&&d.sequence==demand_seq_;
    // A second independent sweep may improve only the evidence binding of
    // this exact original command. It is not a new motion request: source,
    // body source, velocity, owner lease and original expiry stay immutable.
    // In particular, a zero/HOLD or an invalid native verdict cannot be
    // undone by replaying the old sequence with a later positive proof.
    if(evidence_upgrade&&(!sameOriginalDemand(d,demand_)||
       d.motion_validation_sequence<=demand_.motion_validation_sequence||
       invalidated_accepted_demand_||d.hold||
       (d.velocity.linear.x==0&&d.velocity.angular.z==0)))return false;
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
       !fresh(original->source_stamp,now,.35)||compare(original->valid_until,now)<0)return false;
    const auto&v=d.velocity;
    if(!std::isfinite(v.linear.x)||!std::isfinite(v.angular.z)||v.linear.x<0||v.linear.x>max_speed_||std::abs(v.angular.z)>max_yaw_||
       v.linear.y!=0||v.linear.z!=0||v.angular.x!=0||v.angular.y!=0)return false;
    const bool nonzero=v.linear.x!=0||v.angular.z!=0;
    const auto* motion=motionProof(d,now);
    if(nonzero&&(d.hold||!d.safety_checked||!fresh(d.safety_source_stamp,now,rayAgeBound())))return false;
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
    demand_lease_until_stamp_=earlier(d.valid_until,original->valid_until);
    if(nonzero)demand_lease_until_stamp_=earlier(demand_lease_until_stamp_,motion->valid_until);
    demand_lease_until_=secs(demand_lease_until_stamp_);
    have_demand_=true;pending_demand_.reset();return true;
  }
  bool motionValidation(const MotionProof&p,SourceClock now) {
    if(execution_.empty()||!task(binding_,p.version)||!complete(p.version)||p.execution_id!=execution_||
       p.control_epoch!=epoch_||p.sdk_session!=session_||p.sdk_arm_generation!=generation_||p.transport_mode!=mode_||
       p.sequence==0||p.demand_sequence==0||!fresh(p.check_end,now,.15)||nanoseconds(p.check_begin)>nanoseconds(p.check_end)||
       p.braking_model_sha256!=braking_model_sha_||braking_model_sha_.size()!=64)return false;
    for(const auto&old:motion_proofs_)if(version(old.version,p.version)&&old.demand_sequence==p.demand_sequence&&p.sequence<=old.sequence)return false;
    if(!p.valid&&version(p.version,demand_.version)&&p.demand_sequence==demand_.sequence){
      have_demand_=false;invalidated_accepted_demand_=true;
    }
    if(!p.valid&&pending_demand_&&version(p.version,pending_demand_->version)&&p.demand_sequence==pending_demand_->sequence)pending_demand_.reset();
    motion_proofs_.push_back(p);if(motion_proofs_.size()>32)motion_proofs_.pop_front();
    if(p.valid&&pending_demand_){const auto waiting=*pending_demand_;demand(waiting,now);}return true;
  }
  std::optional<geometry_msgs::msg::Twist> tick(SourceClock now,const Health&h) {
    selected_active_=selected_prepared_=false;
    selected_curve_until_=selected_motion_until_={};
    last_health_=h;
    if(execution_.empty())return {};
    if(last_tick_>0&&(now<last_tick_||now-last_tick_>.3)){fault_=true;stop("writer_clock_or_executor_gap",now);}last_tick_=now;
    if(!h.connected||h.estop||!h.general){fault_=true;stop("robot_state_or_connection_lost",now);}
    if(ready_&&!h.owned){fault_=true;stop("control_lost_no_auto_retake",now);}
    if(!ready_&&!stopping_&&h.owned&&h.mc_fresh){ready_=true;ready_at_=now;phase_="ready";}
    if(!ready_&&!stopping_&&now-started_>10.){fault_=true;stop("take_control_timeout",now);}
    if(stopping_) {
      if(!h.connected||!h.owned||!h.general||h.estop)return {};
      return geometry_msgs::msg::Twist{};
    }
    // A missing owner heartbeat remains terminal even during a sensor gap;
    // MC unavailability is not permission to keep an abandoned writer armed.
    if(ready_&&(permit_at_==0?now-ready_at_>.8:!fresh(permit_.source_stamp,now,.35))) {
      stop("permission_expired",now);return geometry_msgs::msg::Twist{};
    }
    if(!h.mc_fresh){
      if(permit_seq_==0&&now-started_<=10.){phase_="waiting_mc_after_takeover";return h.owned?std::optional<geometry_msgs::msg::Twist>{geometry_msgs::msg::Twist{}}:std::nullopt;}
      clearMcGapCommands();mc_hold_=true;mc_resume_floor_=now;phase_="holding";reason_="mc_stale";
      return h.owned&&h.general?std::optional<geometry_msgs::msg::Twist>{geometry_msgs::msg::Twist{}}:std::nullopt;
    }
    if(!ready_)return {};
    if(mc_hold_) {
      // Source freshness is genuine again. Keep the same SDK/control grant,
      // but require a new owner lease and new source-bound motion proof. The
      // owner's independent 0.6 s / 3-new-state window governs task recovery.
      clearMcGapCommands();mc_hold_=false;mc_resume_floor_=now;reason_.clear();phase_="holding";
    }
    if(handoff_&&compare(handoff_->transition_deadline,now)<=0)rejectHandoff("handoff_deadline_expired",now);
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
    if(!commit_sequence_&&(!local_||!fresh(local_->source_stamp,now,.1)||
        !curveProofFresh(permit_,now,false,&selected_curve_until_))){
      phase_="holding";return geometry_msgs::msg::Twist{};
    }
    // A short collision proof lease expiring is a local zero-speed HOLD, not
    // loss of the independently live BT owner. Never reuse an old demand.
    if(compare(permit_.valid_until,now)<0){have_demand_=false;phase_="holding";return geometry_msgs::msg::Twist{};}
    if(!permit_.allowed||!have_demand_){phase_="holding";return geometry_msgs::msg::Twist{};}
    if(!version(permit_.version,demand_.version)||permit_.trajectory_id!=demand_.trajectory_id||
       !fresh(demand_.source_stamp,now,.25)||compare(demand_lease_until_stamp_,now)<0||
       ((demand_.velocity.linear.x!=0||demand_.velocity.angular.z!=0)&&!motionProof(demand_,now))){
      have_demand_=false;phase_="holding";return geometry_msgs::msg::Twist{};
    }
    if(demand_.velocity.linear.x!=0||demand_.velocity.angular.z!=0)
      selected_motion_until_=motionProof(demand_,now)->valid_until;
    phase_=permit_.phase;
    ever_nonzero_=ever_nonzero_||demand_.velocity.linear.x!=0||demand_.velocity.angular.z!=0;
    selected_active_=true;
    return demand_.velocity;
  }
  uint64_t selectedCommitSequence()const{return selected_prepared_||(!commit_sequence_&&selected_active_)?commit_sequence_+1:commit_sequence_;}
  uint64_t nextWriteSequence()const{return write_sequence_+1;}
  void writeCalled(SourceClock now,bool submitted) {
    ++write_sequence_;
    const auto&selected=selected_prepared_&&prepared_?prepared_->demand:demand_;
    const bool nonzero=(selected_prepared_||selected_active_)&&(selected.velocity.linear.x!=0||selected.velocity.angular.z!=0);
    if(nonzero){last_nonzero_write_sequence_=write_sequence_;clearNonterminalStationary();}
    if(selected_prepared_&&handoff_&&prepared_) {
      const auto g=*handoff_;const auto prepared=*prepared_;const auto previous=commit_sequence_;
      permit_=g.candidate;permit_.geometry_committed=true;permit_seq_=std::max(permit_seq_,permit_.sequence);permit_at_=now;
      permits_.clear();permits_.push_back(permit_);demand_=prepared.demand;demand_seq_=demand_.sequence;
      // CAS retires the other role. Future ordinary commands compare only
      // against the actually applied candidate; obsolete incumbent messages
      // are rejected by their exact version/curve, not a shared lane counter.
      demand_at_=now;demand_lease_until_stamp_=earlier(earlier(demand_.valid_until,permit_.valid_until),selected_motion_until_);
      demand_lease_until_=secs(demand_lease_until_stamp_);
      have_demand_=true;invalidated_accepted_demand_=false;active_entry_=prepared;
      applied_version_=permit_.version;applied_trajectory_id_=permit_.trajectory_id;++commit_sequence_;
      clearNonterminalStationary();
      phase_=permit_.phase;
      ever_nonzero_=ever_nonzero_||demand_.velocity.linear.x!=0||demand_.velocity.angular.z!=0;
      makeCommitAck(g,prepared,previous,now,submitted);handoff_.reset();prepared_.reset();
      retained_incumbent_until_=0.;
    } else if(selected_active_&&!commit_sequence_&&local_&&fresh(demand_.source_stamp,now,.1)&&
        fresh(demand_.body_source_stamp,now,.1)&&fresh(local_->source_stamp,now,.1)&&curveProofFresh(permit_,now,false)) {
      applied_version_=permit_.version;applied_trajectory_id_=permit_.trajectory_id;++commit_sequence_;
      Handoff g;g.incumbent.version=binding_;g.candidate=permit_;Prepared prepared;prepared.demand=demand_;
      if(local_){prepared.measured_pose.header=local_->local_odometry.header;prepared.measured_pose.pose=local_->local_odometry.pose.pose;
        prepared.measured_twist=localTwist();}
      makeCommitAck(g,prepared,0,now,submitted);
    }
    selected_active_=selected_prepared_=false;
    if(!submitted)submissionFailed(now);
  }
  void writeAcknowledged(uint64_t commit,SourceClock now) {
    (void)now;
    if(last_ack_&&last_ack_->applied&&last_ack_->commit_sequence==commit&&!last_ack_->write_acknowledged){
      last_ack_->write_acknowledged=true;last_ack_->sequence=++ack_sequence_;ack_dirty_=true;
    }
  }
  void zeroAcknowledged(uint64_t write,SourceClock at,SourceClock now) {
    // This is the actual vendor ACK of an actual zero call, not a BT-local
    // velocity guess or a terminal StopReport. Earlier ACKs cannot cross a
    // subsequent nonzero call or restore retired SDK/control identities.
    if(!write||write>write_sequence_||write<=last_nonzero_write_sequence_||closing_||stopping_||mc_hold_||fault_||
       !ready_||!last_health_.connected||!last_health_.owned||!last_health_.general||!last_health_.mc_fresh||
       !commit_sequence_||permit_.allowed||permit_.phase!="holding"||!fresh(at,now,.25)||!mcSourceFresh(now,clock_epoch_))return;
    if(nonterminal_zero_ack_>0)return; // repeated zero writes do not move the barrier
    nonterminal_zero_ack_=at;nonterminal_zero_write_=write;nonterminal_cutoff_=last_raw_;
  }
  Stationary stationaryEvidence(SourceClock now) {
    Stationary s;s.schema_version=1;s.version=applied_version_;s.execution_id=execution_;s.control_epoch=epoch_;
    s.sdk_session=session_;s.sdk_arm_generation=generation_;s.transport_mode=mode_;s.sequence=++stationary_sequence_;
    s.writer_commit_sequence=commit_sequence_;s.applied_trajectory_id=applied_trajectory_id_;
    s.zero_write_sequence=nonterminal_zero_write_;s.zero_ack_at=stamp(nonterminal_zero_ack_);
    s.mc_raw_stamp_ns=last_raw_;s.mc_clock_epoch=clock_epoch_;s.time_basis=mode_=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock";
    s.source_stamp=stamp(last_source_);s.received_stamp=stamp(last_received_);s.capture_lower_bound=stamp(last_capture_lower_);
    s.capture_upper_bound=stamp(last_received_);s.mc_capture_delay_bound_sec=mc_delay_bound_;
    s.valid_until=stamp(after(nanoseconds(stamp(last_source_))<=nanoseconds(stamp(last_received_))?last_source_:last_received_,.25));s.stationary_samples=nonterminal_stationary_samples_;
    s.stationary_duration_sec=nonterminal_first_stationary_>0?std::max(0.,last_capture_lower_-nonterminal_first_stationary_):0.;
    s.measured_linear_mps=linear_;s.measured_angular_radps=angular_;s.physical_acceptance_verified=mode_=="live"&&accepted_;
    s.nonzero_blocked=!permit_.allowed&&permit_.phase=="holding";
    s.usable=nonterminalStationary(now)&&s.nonzero_blocked;
    if(s.usable){stationary_history_.push_back(s);if(stationary_history_.size()>8)stationary_history_.pop_front();}
    return s;
  }
  std::optional<CommitAck> takeCommitAck(){if(!ack_dirty_||!last_ack_)return {};ack_dirty_=false;return last_ack_;}
  void stopWritten(SourceClock now) {
    if(!stopping_||stop_submitted_)return;
    stop_submitted_=true;submitted_=now;cutoff_=last_raw_;resetStationary();
  }
  void mc(uint64_t raw,SourceClock source,SourceClock received,double linear,double angular,SourceClock now,std::string clock_epoch) {
    if(!clock_epoch_.empty()&&clock_epoch_!=clock_epoch){fault_=true;stop("mc_clock_epoch_changed",now);resetStationary();return;}
    if(raw>0&&last_raw_>0&&raw<last_raw_){mcSourceRegressed(now,clock_epoch);return;}
    if(raw==0||raw==last_raw_||!fresh(source,now,.25)||!fresh(received,now,.25)||
       !std::isfinite(linear)||!std::isfinite(angular)||linear<0||angular<0)return;
    if(last_source_>0&&(source<=last_source_||source-last_source_>.2||std::abs((source-last_source_)-(raw-last_raw_)*1e-9)>.02)){
      resetStationary();clearNonterminalStationary();}
    clock_epoch_=std::move(clock_epoch);last_raw_=raw;last_source_=source;last_received_=received;linear_=linear;angular_=angular;
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
    // bound; subtract the measured uncertainty from the earlier host estimate.
    last_capture_lower_=after(nanoseconds(stamp(source))<=nanoseconds(stamp(received))?source:received,-mc_delay_bound_);
    if(linear>policy_.linear_threshold_mps||angular>policy_.angular_threshold_radps)clearNonterminalStationary();
    else if(nonterminal_zero_ack_>0&&raw>nonterminal_cutoff_&&last_capture_lower_>nonterminal_zero_ack_){
      if(nonterminal_first_stationary_==0)nonterminal_first_stationary_=received;
      ++nonterminal_stationary_samples_;
    }
    if(!stopping_||!stop_submitted_||!mc_bound_configured_||raw<=cutoff_||
       last_capture_lower_<=submitted_)return;
    if(linear>policy_.linear_threshold_mps||angular>policy_.angular_threshold_radps){resetStationary();return;}
    if(first_stationary_==0)first_stationary_=received;
    ++stationary_samples_;stopped_=stationary_samples_>=policy_.minimum_new_samples&&stationaryDuration()>=policy_.stationary_duration_s;
    if(stopped_)phase_="stopped";
  }
  State state(SourceClock now){State s;s.version=commit_sequence_?applied_version_:binding_;s.execution_id=execution_;s.control_epoch=epoch_;s.sdk_session=session_;
    s.sdk_arm_generation=generation_;s.sequence=++state_seq_;s.source_stamp=stamp(now);s.grant_ready=ready_&&last_health_.mc_fresh&&!stopping_&&!fault_;
    s.control_owned=last_health_.owned;s.general_low_speed_confirmed=last_health_.general;s.fault_latched=fault_;
    s.transport_mode=mode_;s.phase=phase_;s.reason=reason_;s.writer_commit_sequence=commit_sequence_;s.applied_trajectory_id=applied_trajectory_id_;return s;}
  Stop report(SourceClock now){Stop s;s.version=binding_;s.execution_id=execution_;s.control_epoch=epoch_;s.sdk_session=session_;
    s.sdk_arm_generation=generation_;s.stop_request_id=request_;s.sequence=++stop_seq_;s.source_stamp=stamp(last_source_>0?last_source_:now);
    s.nonzero_blocked=stopping_||mc_hold_;s.stop_submitted=stop_submitted_;s.measured_stop_confirmed=measuredStopped(now);
    s.physical_acceptance_verified=mode_=="live"&&accepted_;s.mc_clock_epoch=clock_epoch_;s.mc_raw_stamp_ns=last_raw_;
    s.stationary_samples=stationary_samples_;s.stationary_duration_sec=stationaryDuration();
    s.measured_linear_mps=linear_;s.measured_angular_radps=angular_;
    s.time_basis=mode_=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock";
    s.transport_mode=mode_;s.reason=reason_;return s;}
  const Demand& lastDemand()const{return selected_prepared_&&prepared_?prepared_->demand:demand_;}const std::string& executionId()const{return execution_;}
  const Version& binding()const{return binding_;}
  uint64_t armGeneration()const{return generation_;}
  uint64_t epoch()const{return epoch_;}bool stopping()const{return stopping_;}
  bool beginShutdown(SourceClock now){closing_=true;takeover_=false;stop("writer_shutdown",now);return !execution_.empty();}
  double mcCaptureLowerBound()const{return last_capture_lower_;}
  double mcCaptureUpperBound()const{return last_received_;}
  double mcCaptureDelayBound()const{return mc_delay_bound_;}
  bool mcSourceFresh(SourceClock now,const std::string&expected_clock_epoch)const {
    // Arrival frequency alone is not usable MC. Bind health to the exact raw
    // sample admitted by this transport, the active SDK telemetry generation,
    // and both original mapped source time and receipt time. No restamping.
    return !mc_order_hold_&&mc_bound_configured_&&last_raw_>0&&!expected_clock_epoch.empty()&&
      clock_epoch_==expected_clock_epoch&&fresh(last_source_,now,.25)&&fresh(last_received_,now,.25);
  }
  void mcSourceRegressed(SourceClock now,const std::string&clock_epoch) {
    if(!clock_epoch_.empty()&&clock_epoch_!=clock_epoch){fault("mc_clock_epoch_changed",now);return;}
    clearMcGapCommands();resetStationary();mc_hold_=mc_order_hold_=true;mc_resume_floor_=now;
    mc_order_new_samples_=0;mc_order_first_source_=0.;mc_order_barrier_source_=now;phase_="holding";reason_="mc_stale";
    // Preserve last_raw_/source/receipt. Neither rejection nor recovery may
    // re-anchor the clock, refresh old telemetry or erase the anti-replay floor.
  }
  void stop(std::string why,SourceClock now){if(execution_.empty()||stopping_)return;rejectHandoff(why,now);clearNonterminalStationary();stopping_=true;stop_at_=now;ready_=false;have_demand_=false;permits_.clear();pending_demand_.reset();phase_="stopping";reason_=std::move(why);}
private:
  double rayAgeBound()const {
    // The final writer uses the SAME original sensor-age ceiling as native
    // validation and the independent safety gate. Pipeline/reaction budget
    // cannot extend or restamp either sensor's evidence lease.
    return policy_configured_?policy_.sensor_source_age_bound_s:0.;
  }
  static bool twistFinite(const geometry_msgs::msg::Twist&t) {
    return std::isfinite(t.linear.x)&&std::isfinite(t.linear.y)&&std::isfinite(t.linear.z)&&
      std::isfinite(t.angular.x)&&std::isfinite(t.angular.y)&&std::isfinite(t.angular.z);
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
  geometry_msgs::msg::Twist localTwist()const {
    geometry_msgs::msg::Twist out;if(!local_)return out;
    const auto&q=local_->local_odometry.pose.pose.orientation;
    const auto rotate=[&](const geometry_msgs::msg::Vector3&v) {
      geometry_msgs::msg::Vector3 r;
      const double tx=2.*(q.y*v.z-q.z*v.y),ty=2.*(q.z*v.x-q.x*v.z),tz=2.*(q.x*v.y-q.y*v.x);
      r.x=v.x+q.w*tx+q.y*tz-q.z*ty;r.y=v.y+q.w*ty+q.z*tx-q.x*tz;r.z=v.z+q.w*tz+q.x*ty-q.y*tx;return r;
    };
    out.linear=rotate(local_->local_odometry.twist.twist.linear);
    out.angular=rotate(local_->local_odometry.twist.twist.angular);return out;
  }
  bool sameGrantIdentity(const Permit&p)const {
    return complete(p.version)&&task(binding_,p.version)&&p.execution_id==execution_&&p.control_epoch==epoch_&&
      p.sdk_session==session_&&p.sdk_arm_generation==generation_&&p.transport_mode==mode_&&p.frame_id=="d1max_loc_odom";
  }
  bool demandBinding(const Demand&d)const {
    const auto&v=d.velocity;
    return complete(d.version)&&task(binding_,d.version)&&d.execution_id==execution_&&d.control_epoch==epoch_&&
      d.sdk_session==session_&&d.sdk_arm_generation==generation_&&d.transport_mode==mode_&&d.sequence>0&&
      twistFinite(v)&&v.linear.x>=0&&v.linear.x<=max_speed_&&std::abs(v.angular.z)<=max_yaw_&&
      v.linear.y==0&&v.linear.z==0&&v.angular.x==0&&v.angular.y==0;
  }
  bool preparedEntry(const Prepared&p,SourceClock now)const {
    if(!handoff_||!local_||!fresh(local_->source_stamp,now,.1)||!last_health_.mc_fresh||
       stopping_||closing_||mc_hold_||!ready_||!last_health_.owned||!last_health_.general||last_health_.estop||
       p.expected_commit_sequence!=commit_sequence_||compare(handoff_->transition_deadline,now)<=0||
       permit_.revoked||
       (handoff_->transition_mode==Handoff::CONTINUOUS_REPLACE&&(!permit_.allowed||permit_.phase!="tracking"))||
       (handoff_->transition_mode==Handoff::STATIONARY_REENTRY&&
          (permit_.allowed||permit_.phase!="holding"||!stationaryProof(handoff_->stationary_evidence,now)))||
       !sameGrantIdentity(handoff_->candidate)||compare(handoff_->candidate.valid_until,now)<=0||!curveProofFresh(handoff_->candidate,now,true)||
       !fresh(p.demand.source_stamp,now,.1)||!fresh(p.demand.body_source_stamp,now,.1)||
       compare(p.demand.valid_until,now)<=0||p.demand.validation_sequence<handoff_->candidate.validation_sequence||
       p.demand.braking_model_sha256!=braking_model_sha_||!p.demand.safety_checked||
       !fresh(p.demand.safety_source_stamp,now,rayAgeBound())||!demandBinding(p.demand)||
       !poseFinite(p.measured_pose.pose)||!poseFinite(p.curve_entry_pose)||!twistFinite(p.measured_twist)||!twistFinite(p.curve_entry_twist)||
       p.measured_pose.header.frame_id!="d1max_loc_odom"||p.measured_pose.header.stamp!=p.entry_source_stamp||
       p.entry_source_stamp!=p.demand.body_source_stamp||!fresh(p.entry_source_stamp,now,.1)||
       !std::isfinite(p.curve_time)||p.curve_time<0||p.curve_time>120||
       !std::isfinite(p.position_error_m)||!std::isfinite(p.velocity_error_mps)||p.position_error_m<0||p.velocity_error_mps<0||
       std::abs(distance(p.measured_pose.pose.position,p.curve_entry_pose.position)-p.position_error_m)>1e-6||
       std::abs(velocityDistance(p.measured_twist,p.curve_entry_twist)-p.velocity_error_mps)>1e-6||
       p.position_error_m>.0125||p.velocity_error_mps>.05)return false;
    const auto&a=p.entry_admission;
    if(a.sequence==0||!a.accepted||a.version!=p.demand.version||a.trajectory_id!=p.demand.trajectory_id||
       a.validation_sequence!=p.demand.validation_sequence||a.body_source_stamp!=p.entry_source_stamp||
       a.transport_mode!=mode_||!fresh(a.checked_at,now,.15)||nanoseconds(a.checked_at)<nanoseconds(a.body_source_stamp)||
       compare(a.valid_until,now)<=0||elapsed(a.valid_until,a.checked_at)>.250001)return false;
    // Local odometry twist is in child frame. Rotate the actual measured
    // twist into odom, then compare to the production-spline entry boundary.
    // Source age alone cannot certify a 1.25 cm moving handoff.
    return distance(local_->local_odometry.pose.pose.position,p.curve_entry_pose.position)<=.0125&&
      velocityDistance(localTwist(),p.curve_entry_twist)<=.05;
  }
  bool curveProofFresh(const Permit&p,SourceClock now,bool whole,builtin_interfaces::msg::Time* actual_until=nullptr)const {
    const CurveProof* latest=nullptr;
    for(const auto&v:curve_proofs_)if(version(v.version,p.version)&&v.trajectory_id==p.trajectory_id&&
        (!latest||v.sequence>latest->sequence))latest=&v;
    if(!latest)return false;
    const auto&v=*latest;
    if(curveInvalidated(p.version,p.trajectory_id,p.validation_sequence))return false;
    const bool domain=std::isfinite(v.curve_duration)&&v.curve_duration>0&&v.curve_duration<=120&&
      std::isfinite(v.checked_from_time)&&std::isfinite(v.checked_to_time)&&std::isfinite(v.valid_start_time)&&
      v.checked_from_time>=0&&v.checked_from_time<=v.valid_start_time&&v.valid_start_time<=v.checked_to_time&&
      std::abs(v.checked_to_time-v.curve_duration)<=1e-6;
    const bool complete=domain&&v.whole_curve&&!v.remaining_curve&&v.checked_from_time==0.;
    bool remaining=false;
    if(!whole&&v.remaining_curve&&!v.whole_curve&&progress_&&local_) {
      const auto&t=*progress_;const auto&x=p.version;
      remaining=domain&&fresh(t.header.stamp,now,.1)&&t.generation==x.reference_generation&&t.trajectory_id==p.trajectory_id&&
        t.session_id==x.session_id&&t.task_id==x.task_id&&t.route_id==x.route_id&&t.route_hash==x.route_hash&&
        t.map_version_id==x.map_version_id&&t.localization_epoch==x.localization_epoch&&t.localization_seed_id==x.localization_seed_id&&
        t.segment_id==x.segment_id&&t.anchor_id==x.anchor_id&&t.anchor_revision==x.anchor_revision&&t.context_sequence==x.context_sequence&&
        std::isfinite(v.checked_from_time)&&std::isfinite(v.checked_to_time)&&std::isfinite(v.valid_start_time)&&
        std::isfinite(v.reverse_margin_m)&&v.reverse_margin_m>=.15&&v.checked_from_time<=t.curve_time&&
        t.curve_time<=v.checked_to_time&&v.checked_from_time<=v.valid_start_time&&v.valid_start_time<=v.checked_to_time&&
        std::abs(v.checked_to_time-v.curve_duration)<=1e-6&&
        distance(t.pose.position,local_->local_odometry.pose.pose.position)<=.0125&&velocityDistance(t.twist,localTwist())<=.05;
    }
    const bool valid=(complete||remaining)&&v.valid&&v.sequence>=p.validation_sequence&&v.frame_id=="d1max_loc_odom"&&v.collision_policy=="observed_free"&&
      v.map_snapshot_revision>0&&!v.support_reference_id.empty()&&v.support_hash.size()==64&&
      fresh(v.check_end,now,.25)&&fresh(v.body_source_stamp,now,.4)&&
      fresh(v.front_ray_source_stamp,now,rayAgeBound())&&fresh(v.rear_ray_source_stamp,now,rayAgeBound())&&
      compare(v.valid_until,now)>0&&elapsed(v.valid_until,v.check_end)<=.250001;
    if(valid&&actual_until)*actual_until=v.valid_until;
    return valid;
  }
  bool curveInvalidated(const Version&v,int64_t trajectory,uint64_t floor)const {
    return std::any_of(curve_proofs_.begin(),curve_proofs_.end(),[&](const auto&p){
      return version(p.version,v)&&p.trajectory_id==trajectory&&!p.valid&&p.sequence>floor;
    });
  }
  void makeCommitAck(const Handoff&g,const Prepared&p,uint64_t previous,SourceClock now,bool submitted) {
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
  void rejectHandoff(const std::string&why,SourceClock now) {
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
    return normalized==b&&nanoseconds(a.safety_source_stamp)>=nanoseconds(b.safety_source_stamp);
  }
  void clearMcGapCommands(){rejectHandoff("mc_stale",last_tick_);clearNonterminalStationary();have_demand_=false;permits_.clear();motion_proofs_.clear();pending_demand_.reset();active_entry_.reset();}
  void clearNonterminalStationary(){nonterminal_zero_ack_=nonterminal_first_stationary_=0.;
    nonterminal_zero_write_=nonterminal_cutoff_=0;nonterminal_stationary_samples_=0;stationary_history_.clear();}
  bool nonterminalStationary(SourceClock now)const {
    return !closing_&&!stopping_&&!mc_hold_&&!fault_&&ready_&&last_health_.connected&&last_health_.owned&&last_health_.general&&
      !last_health_.estop&&last_health_.mc_fresh&&mcSourceFresh(now,clock_epoch_)&&nonterminal_zero_ack_>0&&
      policy_configured_&&nonterminal_zero_write_>last_nonzero_write_sequence_&&nonterminal_stationary_samples_>=policy_.minimum_new_samples&&nonterminal_first_stationary_>0&&
      last_capture_lower_-nonterminal_first_stationary_>=policy_.reentry_duration_s&&linear_<=policy_.linear_threshold_mps&&angular_<=policy_.angular_threshold_radps;
  }
  bool stationaryProof(const Stationary&s,SourceClock now)const {
    if(s.schema_version!=1||!s.usable||!s.nonzero_blocked||s.writer_commit_sequence!=commit_sequence_||
       s.applied_trajectory_id!=applied_trajectory_id_||!version(s.version,applied_version_)||s.execution_id!=execution_||
       s.control_epoch!=epoch_||s.sdk_session!=session_||s.sdk_arm_generation!=generation_||s.transport_mode!=mode_||
       s.zero_write_sequence!=nonterminal_zero_write_||s.zero_ack_at!=stamp(nonterminal_zero_ack_)||
       s.mc_clock_epoch!=clock_epoch_||s.time_basis!=(mode_=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock")||
       !fresh(s.source_stamp,now,.25)||!fresh(s.received_stamp,now,.25)||compare(s.valid_until,now)<=0||
       (mode_=="live"&&!s.physical_acceptance_verified)||!nonterminalStationary(now))return false;
    return std::any_of(stationary_history_.begin(),stationary_history_.end(),[&](const auto&old){return old==s;});
  }
  double stationaryDuration()const{return first_stationary_>0?std::max(0.,last_capture_lower_-first_stationary_):0.;}
  bool measuredStopped(SourceClock now)const{return stopped_&&policy_configured_&&linear_<=policy_.linear_threshold_mps&&angular_<=policy_.angular_threshold_radps&&
    mc_bound_configured_&&stationary_samples_>=policy_.minimum_new_samples&&first_stationary_>0&&stationaryDuration()>=policy_.stationary_duration_s&&
    mcSourceFresh(now,clock_epoch_);}
  void resetStationary(){stopped_=false;first_stationary_=0;stationary_samples_=0;if(stopping_)phase_="stopping";}
  const MotionProof* motionProof(const Demand&d,SourceClock now,const Prepared*witness=nullptr)const {
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
       p.frame_id!="d1max_loc_odom"||nanoseconds(d.valid_until)>nanoseconds(p.demand_valid_until)||
       !fresh(d.source_stamp,now,.1)||!fresh(d.body_source_stamp,now,.1)||
       !fresh(p.body_source_stamp,now,.1)||!fresh(p.check_end,now,.1)||
       !fresh(p.front_ray_source_stamp,now,rayAgeBound())||!fresh(p.rear_ray_source_stamp,now,rayAgeBound())||
       compare(p.valid_until,now)<0||nanoseconds(p.valid_until)>nanoseconds(p.demand_valid_until)||
       elapsed(p.valid_until,p.check_end)>.25)return nullptr;
    return &p;
  }
  std::string session_,mode_,execution_,request_,phase_="idle",reason_,clock_epoch_;bool accepted_=false;
  std::string braking_model_sha_;
  ExecutionPolicy policy_;bool policy_configured_=false;
  Version binding_;Permit permit_;Demand demand_;std::deque<Permit> permits_;std::deque<MotionProof>motion_proofs_;std::deque<CurveProof>curve_proofs_;std::set<std::string>seen_;
  std::optional<Demand>pending_demand_;
  std::optional<Handoff>handoff_,last_handoff_;std::optional<Prepared>prepared_,active_entry_;std::optional<CommitAck>last_ack_;std::optional<Local>local_;std::optional<Progress>progress_;
  Version applied_version_;int64_t applied_trajectory_id_=0;
  uint64_t commit_sequence_=0,handoff_sequence_=0,ack_sequence_=0,prepared_sequence_floor_=0;
  uint64_t write_sequence_=0,last_nonzero_write_sequence_=0,stationary_sequence_=0,nonterminal_zero_write_=0,nonterminal_cutoff_=0;
  uint32_t nonterminal_stationary_samples_=0;
  SourceClock nonterminal_zero_ack_;double nonterminal_first_stationary_=0.;std::deque<Stationary>stationary_history_;
  bool mc_order_hold_=false;uint32_t mc_order_new_samples_=0;double mc_order_first_source_=0.,mc_order_barrier_source_=0.;
  bool selected_prepared_=false,selected_active_=false,ack_dirty_=false;double retained_incumbent_until_=0;
  builtin_interfaces::msg::Time selected_curve_until_,selected_motion_until_,demand_lease_until_stamp_;
  Health last_health_;
  bool ready_=false,fault_=false,stopping_=false,stopped_=false,stop_submitted_=false,have_demand_=false,takeover_=false,takeover_taken_=false,ever_nonzero_=false,closing_=false,mc_bound_configured_=false,mc_hold_=false,invalidated_accepted_demand_=false;
  uint64_t epoch_=0,generation_=0,permit_seq_=0,demand_seq_=0,state_seq_=0,stop_seq_=0,last_raw_=0,cutoff_=0;
  uint32_t stationary_samples_=0;
  double started_=0,ready_at_=0,stop_at_=0,submitted_=0,last_tick_=0,permit_at_=0,demand_at_=0,demand_lease_until_=0,first_stationary_=0,linear_=0,angular_=0;
  SourceClock last_source_,last_received_,last_capture_lower_;
  double max_speed_=.3,max_yaw_=.5,mc_delay_bound_=0.,mc_resume_floor_=0.;
};
}
