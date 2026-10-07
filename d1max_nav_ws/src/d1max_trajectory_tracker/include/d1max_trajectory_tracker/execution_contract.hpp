#pragma once
#include "d1max_trajectory_tracker/tracker_core.hpp"
#include "d1max_trajectory_tracker/bounded_preparation_worker.hpp"
#include <d1max_planning_interfaces/msg/execution_permit.hpp>
#include <d1max_planning_interfaces/msg/reference_proposal.hpp>
#include <d1max_planning_interfaces/msg/trajectory_validation.hpp>
#include <d1max_planning_interfaces/msg/trajectory_admission.hpp>
#include <d1max_planning_interfaces/msg/support_reference.hpp>
#include <d1max_planning_interfaces/msg/motion_demand.hpp>
#include <d1max_planning_interfaces/msg/motion_validation.hpp>
#include <d1max_planning_interfaces/msg/execution_handoff_grant.hpp>
#include <d1max_planning_interfaces/msg/prepared_motion_demand.hpp>
#include <d1max_planning_interfaces/msg/execution_commit_ack.hpp>
#include <d1max_planning_interfaces/msg/tracker_geometry_receipt.hpp>
#include <optional>
#include <deque>
#include <initializer_list>
#include <type_traits>

namespace d1max_trajectory_tracker {
namespace wire=d1max_planning_interfaces::msg;
// Geometry-only ACK fact check for the explicitly sealed XYZ measurement
// domain. Source elapsed time bounds arc search; it never selects a phase.
inline bool isolatedAppliedEntryValid(const Config& config,const Trajectory& t,double entry_time,
    const Eigen::Vector3d& measured,const Eigen::Vector3d& actual_velocity,double source_dt) {
  if(!config.isolated_reference_model||!measured.allFinite()||!actual_velocity.allFinite()||
      !finite(source_dt)||source_dt<0.||actual_velocity.stableNorm()>config.measuredTravelSpeedLimit()||
      !t.prepared||!t.prepared->failure.empty()||!t.prepared->derivatives_valid||
      !t.prepared->matches(config,t))return false;
  const auto& cache=t.prepared->entry;
  if(!finite(entry_time)||entry_time<0.||entry_time>cache.duration)return false;
  const double travel=std::min(config.projection_max_forward_m,
      config.measuredTravelSpeedLimit()*source_dt+config.join_limit);
  const double seed_arc=curveArcAt(cache.times,cache.arcs,entry_time);
  const auto projected=projectCurveAdmission(*cache.curve_sampler,cache.times,cache.arcs,cache.points,
      measured,entry_time,seed_arc,std::min(config.projection_backtrack_m,travel),travel,config.join_limit);
  return projected&&(cache.velocity_sampler->evaluateDeBoorT(projected->time)-actual_velocity).norm()<=.05;
}
inline double seconds(const builtin_interfaces::msg::Time& s) { return double(s.sec)+double(s.nanosec)*1e-9; }
inline builtin_interfaces::msg::Time stamp(double t) {
  builtin_interfaces::msg::Time s;
  s.sec=static_cast<std::int32_t>(std::floor(t));
  s.nanosec=static_cast<std::uint32_t>(std::llround((t-s.sec)*1e9));
  if(s.nanosec==1000000000u) {++s.sec;s.nanosec=0;}
  return s;
}
inline std::int64_t timeNs(const builtin_interfaces::msg::Time& t) {
  return std::int64_t(t.sec)*1000000000LL+t.nanosec;
}
inline builtin_interfaces::msg::Time stampNs(std::int64_t ns) {
  builtin_interfaces::msg::Time t;
  t.sec=static_cast<std::int32_t>(ns/1000000000LL);
  t.nanosec=static_cast<std::uint32_t>(ns%1000000000LL);return t;
}
inline builtin_interfaces::msg::Time stamp(SourceTime t) {return stampNs(t.nanoseconds());}
inline bool validSourceTime(const builtin_interfaces::msg::Time& t) {
  return t.sec>=0&&t.nanosec<1000000000U&&timeNs(t)>0;
}
inline SourceTime sourceTime(const builtin_interfaces::msg::Time& t) {
  return SourceTime::fromNanoseconds(validSourceTime(t)?timeNs(t):0);
}
inline bool freshStamp(SourceTime now,const builtin_interfaces::msg::Time& t,double maximum,double future=.02) {
  return validSourceTime(t)&&sourceFresh(now,timeNs(t),maximum,future);
}
inline builtin_interfaces::msg::Time originalTimeCap(builtin_interfaces::msg::Time fresh,
    std::initializer_list<builtin_interfaces::msg::Time> caps) {
  // A signed original lease is an integer transport fact. At Unix-scale
  // seconds a double round trip can increase it, even by one nanosecond.
  for(const auto& cap:caps)if(timeNs(cap)<timeNs(fresh))fresh=cap;
  return fresh;
}
inline builtin_interfaces::msg::Time bodySourceStamp(const Odom& odom) {
  return odom.source_stamp_ns>0?stampNs(odom.source_stamp_ns):stamp(odom.stamp);
}
inline ControlIdentity identity(const wire::ExecutionVersion& v) {
  ControlIdentity i;
  i.schema_version=2;i.task_id=v.task_id;i.route_id=v.route_id;i.route_hash=v.route_hash;
  i.segment_id=v.segment_id;i.map_version_id=v.map_version_id;i.anchor_id=v.anchor_id;
  i.anchor_revision=v.anchor_revision;i.context_sequence=v.context_sequence;
  i.map_geometry_revision=v.map_geometry_revision;
  i.localization_epoch=v.localization_epoch;i.localization_seed_id=v.localization_seed_id;return i;
}
inline bool versionValid(const wire::ExecutionVersion& v,const Config& c) {
  return v.schema_version==3 && v.session_id==c.session_id && v.map_version_id==c.map_version_id &&
    v.reference_generation>0 && v.map_geometry_revision>0 && identity(v).valid();
}
inline bool timed(SourceTime now,const builtin_interfaces::msg::Time& source,
    const builtin_interfaces::msg::Time& until,double maximum) {
  return freshStamp(now,source,maximum)&&validSourceTime(until)&&timeNs(until)>now.nanoseconds()&&
    timeNs(until)>timeNs(source)&&timeNs(until)-timeNs(source)<=durationNs(maximum);
}
inline SupportEvidence supportEvidence(const wire::SupportReference& s) {
  SupportEvidence out;
  out.identity=identity(s.version);out.generation=s.version.reference_generation;
  out.id=s.support_reference_id;out.hash=s.support_hash;out.map_hash=s.support_map_sha256;
  out.floor_id=s.floor_id;out.segment_kind=s.segment_kind;out.mode=s.required_mode;out.frame=s.frame_id;
  out.source_stamp=seconds(s.source_stamp);out.xy_radius=s.support_xy_radius_m;
  out.body_height=s.body_reference_height_m;out.max_slope=s.max_support_slope_rad;
  out.max_step=s.max_support_step_m;out.verified=s.verified;
  for(const auto& p:s.support_ground_xyz) out.ground.emplace_back(p.x,p.y,p.z);
  return out;
}

// Sole controller-side transaction gate. Candidate preparation has no effect
// on the committed curve. Only the BT's matching live lease may commit it.
// No ROS graph is needed to exercise this exact production protocol in tests.
class ExecutionContract {
public:
  ExecutionContract(Config c,std::string mode,std::string braking_model_sha256={},bool writer_handoff=false,
      bool asynchronous_preparation=false):
    config_(std::move(c)),mode_(std::move(mode)),braking_model_sha256_(std::move(braking_model_sha256)),core_(config_),writer_handoff_(writer_handoff) {
    if(mode_!="live"&&mode_!="isolated_mock") throw std::invalid_argument("explicit execution transport required");
    if(config_.isolated_reference_model&&(mode_!="isolated_mock"||
       config_.isolated_reference_model->recordSha()!=braking_model_sha256_))
      throw std::invalid_argument("isolated_reference_record_transport_mismatch");
    if(asynchronous_preparation)worker_=std::make_unique<PreparationWorker>(
      [](const PreparationRequest& r,const PreparationWorker::Allowed& allowed) {
        // Capture only request-owned immutable input. No core/lease state is
        // read or written from this thread; actual admission stays below.
        return PreparedGeometry::build(r.config,*r.trajectory,
          r.support?supportEvidence(*r.support):SupportEvidence{},allowed);
      });
  }
  TrackerCore& core(){return core_;}
  const TrackerCore& core()const{return core_;}
  void cancel(const std::string& reason) {
    if(permit_)retired_task_=permit_->version.task_id;
    else if(candidate_)retired_task_=candidate_->identity.task_id;
    candidate_.reset();candidate_entry_curve_.reset();admission_.reset();committed_version_.reset();demands_.clear();
    preparation_failure_.clear();
    blocked_entry_.reset();last_emission_.reset();paused_source_ns_=0;core_.cancel(reason);
    if(worker_)worker_->invalidate();
    preparation_request_.reset();
    if(handoff_)handoff_->retired=true;
  }
  bool receiveOdom(const Odom& state,SourceTime now,double receipt) {
    const bool was_active=core_.active();
    const bool accepted=core_.receiveOdom(state,now,receipt);
    if(was_active&&!core_.active())cancel(core_.reason());
    return accepted;
  }
  void proposal(const wire::ReferenceProposal& p,SourceTime now) {
    if(!versionValid(p.version,config_)||p.transport_mode!=mode_||p.proposal_id.empty()||
       !timed(now,p.source_stamp,p.valid_until,1.)||
       p.reference.path.header.frame_id!=config_.planning_frame||p.reference.point_reference!="body_center") return;
    // Compare with the owner's current lease, even one whose commit failed
    // here: SCAN and the BT already build on it. A stale local view would
    // reject every successor forever while this controller holds.
    if(owner_permit_&&owner_permit_->allowed && !(p.expected_version==owner_permit_->version)) return;
    if(owner_permit_&&owner_permit_->allowed && p.expected_trajectory_id!=owner_permit_->trajectory_id) return;
    if(proposal_&&p.version.reference_generation<proposal_->version.reference_generation) return;
    proposal_=p;
    // An admission stays bound to the proposal it was prepared from; a newer
    // proposal must not re-date the task of an already prepared curve.
    for(const auto& old:proposals_)if(old.version==p.version&&old.proposal_id==p.proposal_id)return;
    proposals_.push_back(p);if(proposals_.size()>8)proposals_.pop_front();
  }
  void support(const wire::SupportReference& s) {
    if(!versionValid(s.version,config_)||s.support_ground_xyz.size()>20000) return;
    if(active_support_&&s.version==active_support_->version) active_support_=s;
    else pending_support_=s;
    schedulePreparation();
  }
  void candidate(Trajectory t) {
    if(t.session_id!=config_.session_id || !t.identity.valid() || t.points.size()>10000) return;
    if(retired_task_&&t.identity.task_id==*retired_task_)return;
    if(candidate_ && t.generation<candidate_->generation) return;
    if(candidate_&&t.generation==candidate_->generation&&t.id<=candidate_->id)return;
    // A curve ID is immutable. Re-delivery cannot silently replace points
    // under a cached ID, and newer candidates do not touch the incumbent.
    candidate_entry_curve_.reset();t.prepared.reset();
    preparation_failure_.clear();
    candidate_=std::move(t);
    if(worker_)worker_->invalidate();
    preparation_request_.reset();
    schedulePreparation();
  }
  bool pollPreparation() {
    if(!worker_)return false;
    const auto done=worker_->take();if(!done)return false;
    if(!preparation_request_||done->token!=preparation_token_||
       !candidate_||done->request.trajectory->generation!=candidate_->generation||
       done->request.trajectory->id!=candidate_->id||
       !(done->request.trajectory->identity==candidate_->identity)||
       (retired_task_&&candidate_->identity.task_id==*retired_task_))return false;
    last_preparation_seconds_=done->elapsed_sec;
    if(!done->failure.empty()||!done->result||!*done->result) {
      preparation_failure_=last_prepare_reason_="candidate_preparation_worker_failed";return true;
    }
    installPreparation(*done->result);return true;
  }
  bool preparationPending()const{return candidate_&&!candidate_->prepared&&preparation_failure_.empty();}
  double preparationWorkerSeconds()const{return last_preparation_seconds_;}
  void validation(const wire::TrajectoryValidation& v,SourceTime now) {
    if(!versionValid(v.version,config_)||v.transport_mode!=mode_) return;
    if(proofFresh(v,now))for(auto& e:emissions_) {
      if(e.demand.version==v.version&&e.demand.trajectory_id==v.trajectory_id&&
         v.sequence>=e.demand.validation_sequence) {
        capTrajectory(e,v);
        if(last_emission_&&last_emission_->demand.sequence==e.demand.sequence)capTrajectory(*last_emission_,v);
      }
    }
    const auto duplicate=std::find_if(proofs_.begin(),proofs_.end(),[&](const auto& old){
      return old.version==v.version&&old.trajectory_id==v.trajectory_id&&old.sequence==v.sequence;
    });
    if(duplicate==proofs_.end()) {proofs_.push_back(v);if(proofs_.size()>32)proofs_.pop_front();}
    if(permit_&&v.version==permit_->version&&v.trajectory_id==permit_->trajectory_id) {
      if(!active_validation_||v.sequence>active_validation_->sequence) {
        // Keep the proof referenced by the currently committed permit until
        // the owner consumes the new sequence. Callback ordering isn't a stop.
        if(active_validation_&&active_validation_->sequence==permit_->validation_sequence)
          leased_validation_=active_validation_;
        active_validation_=v;
      }
    } else if(!pending_validation_||v.version.reference_generation>pending_validation_->version.reference_generation||
      (v.version==pending_validation_->version&&v.sequence>pending_validation_->sequence)) pending_validation_=v;
    (void)now;
  }
  bool motionValidation(const wire::MotionValidation& v,SourceTime now) {
    // This is controller feedback, never permission to move. Only an actual
    // collision on our own still-live forward+turn or in-place turn command
    // can request a different maneuver; unknown/late/foreign evidence cannot.
    observeMotionExpiry(v,now);
    if(v.valid||v.reason!="motion_sweep_occupied"||!permit_||!committed_version_||
       braking_model_sha256_.size()!=64||v.braking_model_sha256!=braking_model_sha256_||
       v.sequence<=last_motion_sequence_||v.version!=*committed_version_||
       v.version!=permit_->version||v.trajectory_id!=core_.trajectoryId()||
       !permit_->allowed||permit_->revoked||permit_->phase!="tracking"||
       !timed(now,permit_->source_stamp,permit_->valid_until,.75)||
       v.transport_mode!=mode_||v.frame_id!=config_.planning_frame||v.map_snapshot_revision==0)return false;
    const auto it=std::find_if(demands_.rbegin(),demands_.rend(),[&](const auto& d){
      return d.sequence==v.demand_sequence&&d.version==v.version;
    });
    if(it==demands_.rend())return false;
    const auto& d=*it;
    if(d.hold||d.velocity.linear.x<0.||std::abs(d.velocity.angular.z)<=1e-9||
       d.execution_id!=v.execution_id||d.control_epoch!=v.control_epoch||d.sdk_session!=v.sdk_session||
       d.sdk_arm_generation!=v.sdk_arm_generation||d.trajectory_id!=v.trajectory_id||
       d.permit_sequence!=v.permit_sequence||v.trajectory_validation_sequence<d.validation_sequence||
       d.source_stamp!=v.demand_source_stamp||d.body_source_stamp!=v.demand_body_source_stamp||
       d.valid_until!=v.demand_valid_until||d.velocity!=v.velocity||
       d.execution_id!=permit_->execution_id||d.control_epoch!=permit_->control_epoch||
       d.sdk_session!=permit_->sdk_session||d.sdk_arm_generation!=permit_->sdk_arm_generation||
       !timed(now,d.source_stamp,d.valid_until,.100001))return false;
    const auto begin=timeNs(v.check_begin),end=timeNs(v.check_end),until=timeNs(v.valid_until);
    const auto fresh=[&](const auto& t,double maximum){
      return freshStamp(now,t,maximum);
    };
    if(!validSourceTime(v.check_begin)||!validSourceTime(v.check_end)||!validSourceTime(v.valid_until)||
       begin<timeNs(d.source_stamp)||end<begin||!sourceFresh(now,end,.1,.02)||until<=now.nanoseconds()||
       until>timeNs(d.valid_until)||!fresh(d.body_source_stamp,.1)||!fresh(v.body_source_stamp,.1)||
       !fresh(v.front_ray_source_stamp,.5)||!fresh(v.rear_ray_source_stamp,.5))return false;
    const auto* current=proofFor(*permit_,now);
    if(!current||current->sequence<v.trajectory_validation_sequence)return false;
    const auto exact=std::find_if(proofs_.rbegin(),proofs_.rend(),[&](const auto& p){
      return p.version==v.version&&p.trajectory_id==v.trajectory_id&&p.sequence==v.trajectory_validation_sequence;
    });
    if(exact==proofs_.rend()||!proofFresh(*exact,now))return false;
    if(!core_.notifyBlockedForwardTurn(d.trajectory_id,sourceTime(d.source_stamp),now,
       d.velocity.linear.x,d.velocity.angular.z))return false;
    last_motion_sequence_=v.sequence;
    if(std::string(core_.maneuverPhase())=="waiting_executable_entry")blocked_entry_=v;
    return true;
  }
  std::optional<wire::MotionValidation> takeBlockedEntry() {
    // Original native result, not a newly stamped certificate. A consumer
    // must match it to its own sweep ledger before changing local planning.
    auto value=std::move(blocked_entry_);blocked_entry_.reset();return value;
  }
  // Diagnostics only: last reasons a preparation / permit commit was refused.
  const std::string& lastPrepareReason()const{return last_prepare_reason_;}
  const std::string& lastPermitReject()const{return last_permit_reject_;}
  const std::string& initialAckReason()const{return initial_ack_reason_;}
  std::optional<wire::TrackerGeometryReceipt> geometryReceipt() {
    if(!installation_)return {};
    auto r=installation_->fact;
    r.sequence=++geometry_receipt_sequence_;
    r.body_source_stamp=bodySourceStamp(core_.odometry());
    r.installed=core_.hasInstalledGeometry()&&core_.trajectoryId()==r.trajectory_id&&
      core_.generation()==r.version.reference_generation&&core_.task().identity==identity(r.version);
    r.reason=r.installed?(core_.requiresMeasuredReentry()?"braking_envelope_reentry_required":
      "tracker_geometry_installed"):"tracker_geometry_no_longer_installed";
    // Repeated source messages retain their exact old source. No current tick
    // can renew installed_at, the original admission/proof, or body evidence.
    return r;
  }
  std::optional<wire::TrajectoryAdmission> prepare(SourceTime now,double receipt) {
    pollPreparation();
    if(!candidate_||!pending_validation_||!pending_support_) return {};
    if(!candidate_->prepared&&preparation_failure_.empty()) {last_prepare_reason_="waiting_candidate_preparation";return {};}
    if(core_.active()&&core_.generation()==candidate_->generation&&core_.trajectoryId()==candidate_->id) return {};
    const auto& v=*pending_validation_;
    const auto* bound=proposalFor(v.version,v.proposal_id);
    if(!bound||
       candidate_->generation!=v.version.reference_generation||candidate_->id!=v.trajectory_id||
       !(candidate_->identity==identity(v.version))||pending_support_->version!=v.version) return {};
    wire::TrajectoryAdmission a;
    a.sequence=++admission_sequence_;
    a.version=v.version;a.trajectory_id=v.trajectory_id;a.validation_sequence=v.sequence;
    a.body_source_stamp=bodySourceStamp(core_.odometry());a.checked_at=stamp(now);
    a.valid_until=originalTimeCap(stampNs(timeNs(a.checked_at)+150000000LL),
      {stampNs(timeNs(v.check_end)+250000000LL),v.valid_until});a.transport_mode=mode_;
    Task task=taskFor(v.version,bound->goal_position,sourceTime(bound->source_stamp));
    // Re-query current measured entry only AFTER the native certificate is
    // known fresh. This does not refresh that certificate or the worker data.
    std::string reason;
    if(!proofFresh(v,now))reason="validation_not_fresh";
    else if(!supportMatches(v,*pending_support_))reason="support_mismatch";
    else {
      const auto observed=observeCandidateEntry(v,now,reason);
      if(!observed)reason="candidate:"+reason;
      else {
        // A late irreversible ACK can restore the explicitly retired task
        // under HOLD. Preparation for its successor is pure: retire that
        // geometry in a copy, retaining the old writer fact in the live core.
        std::optional<TrackerCore> retired_trial;
        auto* admission_core=&core_;
        if(retiredWriterHoldFor(task)) {
          retired_trial.emplace(core_);retired_trial->cancel("retired_writer_applied_hold");
          admission_core=&*retired_trial;
        }
        if(!admission_core->admitRevision(task,*observed,preparedSupport(*pending_support_),now,receipt,false))
          reason="candidate:"+admission_core->candidateReason();
      }
    }
    a.accepted=reason.empty();
    a.reason=a.accepted?"prepared_not_committed":reason;
    last_prepare_reason_=a.reason;
    if(a.accepted) {
      admission_=a;admitted_validation_=v;
      admissions_.push_back(a);if(admissions_.size()>32)admissions_.pop_front();
    }
    return a;
  }
  bool permit(const wire::ExecutionPermit& p,SourceTime now,double receipt) {
    if(!versionValid(p.version,config_)||p.transport_mode!=mode_||p.frame_id!=config_.planning_frame||
       p.sequence<=last_permit_sequence_||!timed(now,p.source_stamp,p.valid_until,.75)) return false;
    if(permit_ && p.control_epoch<permit_->control_epoch) return false;
    permit_history_.push_back(p);while(permit_history_.size()>32)permit_history_.pop_front();
    if(retired_task_&&p.version.task_id==*retired_task_&&!p.revoked)return false;
    const bool new_writer_authority=writer_handoff_&&p.allowed&&writer_authority_&&
      !writerAuthorityMatches(p)&&writerAuthorityTransitionAllowed(p);
    // Owner heartbeat delivery can lag the writer ACK. Never roll the already
    // applied identity back, even when the old heartbeat has a higher sequence.
    if(writer_handoff_&&writer_commit_sequence_&&committed_version_&&!new_writer_authority&&
       (p.version!=*committed_version_||p.trajectory_id!=core_.trajectoryId())&&
       p.version.reference_generation<=committed_version_->reference_generation&&!p.revoked)return false;
    if(p.revoked) { cancel("execution_revoked");retired_task_=p.version.task_id;permit_=owner_permit_=p;last_permit_sequence_=p.sequence;capPermitEmissions(p);return true; }
    if(!p.geometry_committed) {permit_=owner_permit_=p;last_permit_sequence_=p.sequence;capPermitEmissions(p);return true;}
    if(p.allowed&&(p.execution_id.empty()||p.confirmation_id.empty()||p.control_epoch==0||p.sdk_session.empty()||p.sdk_arm_generation==0))
      return false;
    if(writer_handoff_&&writer_commit_sequence_&&committed_version_&&!new_writer_authority&&
       (p.version!=*committed_version_||p.trajectory_id!=core_.trajectoryId())) {
      last_permit_reject_="candidate_requires_writer_applied_ack";return false;
    }
    // The owner's current lease, whether or not the commit below succeeds.
    owner_permit_=p;
    // Arming/holding changes authority, not an already committed geometry.
    // Do not require the expired first admission again after a zero-authority
    // heartbeat. The independent ledger and live controller must both agree.
    if(committed_version_&&p.version==*committed_version_&&core_.active()&&
       p.trajectory_id==core_.trajectoryId()&&p.version.reference_generation==core_.generation()&&
       identity(p.version)==core_.task().identity) {
      if(!proofFor(p,now)) {last_permit_reject_="heartbeat_proof_not_fresh";return false;}
      if(!writerAuthorityTransitionAllowed(p)) {last_permit_reject_="writer_authority_scope_not_retired_or_forward";return false;}
      bindWriterAuthority(p);
      permit_=p;last_permit_sequence_=p.sequence;core_.refreshTaskLease(receipt);capPermitEmissions(p);return true;
    }
    // A first geometry commit still binds the exact two-phase admission.
    // Only an already committed curve may advance beyond the permit's floor.
    const auto* prepared_proof=proofFor(p,now,true);
    const wire::TrajectoryAdmission* admission=nullptr;
    for(const auto& a:admissions_)if(a.version==p.version&&a.trajectory_id==p.trajectory_id&&
      a.validation_sequence==p.validation_sequence)admission=&a;
    const auto rejectPermit=[&](const char* why){last_permit_reject_=why;return false;};
    if(!admission)return rejectPermit("commit_admission_missing");
    if(!candidate_||candidate_->id!=p.trajectory_id||candidate_->generation!=p.version.reference_generation||
       !(candidate_->identity==identity(p.version)))return rejectPermit("commit_candidate_replaced");
    if(!prepared_proof)return rejectPermit("commit_exact_proof_missing_or_stale");
    const auto* bound=proposalFor(p.version,prepared_proof->proposal_id);
    if(!bound)return rejectPermit("commit_proposal_missing");
    if(!pending_support_)return rejectPermit("commit_support_missing");
    if(!admission->accepted||!timed(now,admission->checked_at,admission->valid_until,.25))
      return rejectPermit("commit_admission_expired");
    if(prepared_proof->version!=p.version||prepared_proof->trajectory_id!=p.trajectory_id||!proofFresh(*prepared_proof,now))
      return rejectPermit("commit_proof_not_fresh");
    if(pending_validation_&&pending_validation_->version==p.version&&pending_validation_->trajectory_id==p.trajectory_id&&
       pending_validation_->sequence>=p.validation_sequence&&!pending_validation_->valid)
      return rejectPermit("commit_newer_invalid_proof");
    if(!writerAuthorityTransitionAllowed(p))return rejectPermit("writer_authority_scope_not_retired_or_forward");
    const Task task=taskFor(p.version,p.goal_position,sourceTime(bound->source_stamp));
    std::string entry_reason;const auto observed=observeCandidateEntry(*prepared_proof,now,entry_reason);
    if(!observed) {last_permit_reject_="commit_candidate:"+entry_reason;return false;}
    std::optional<TrackerCore> retired_trial;
    auto* admission_core=&core_;
    if(new_writer_authority&&retiredWriterHoldFor(task)) {
      retired_trial.emplace(core_);retired_trial->cancel("retired_writer_applied_hold");
      admission_core=&*retired_trial;
    }
    if(!admission_core->admitRevision(task,*observed,preparedSupport(*pending_support_),now,receipt,true)) {
      last_permit_reject_="commit_candidate:"+admission_core->candidateReason();return false;
    }
    if(retired_trial)core_=std::move(*retired_trial);
    bindWriterAuthority(p);
    last_permit_reject_.clear();
    active_validation_=*prepared_proof;leased_validation_=active_validation_;active_support_=pending_support_;
    committed_version_=p.version;
    permit_=p;last_permit_sequence_=p.sequence;
    recordInstallation(p,*admission,*prepared_proof,task,stamp(now));
    admission_.reset();capPermitEmissions(p);return true;
  }
  // false means retain the last signed finite output; publish nothing. All
  // faults use the ordinary stop path, including exact 1 ns source regression.
  bool controlTickRequired(SourceTime now,double receipt)const {
    const wire::TrajectoryValidation* proof=nullptr;
    if(!last_emission_||last_emission_->demand.hold||paused_source_ns_||
       now.nanoseconds()!=last_emission_->control_source_ns||
       now.nanoseconds()!=core_.lastControlSourceNs()||
       last_emission_->control_revision!=core_.outputControlRevision()||
       !std::isfinite(receipt)||receipt<last_emission_->emitted_receipt||
       receipt>=last_emission_->steady_until||ordinaryBlockReason(now,proof)||
       !sameAuthority(last_emission_->authority,*permit_)||
       last_emission_->demand.version!=permit_->version||last_emission_->demand.trajectory_id!=core_.trajectoryId()||
       !core_.active()||!core_.hasInstalledGeometry()||core_.holding())return true;
    const auto& p=*permit_;
    return !core_.duplicateControlStateSafe(now,receipt,p.phase=="aligning",p.goal_yaw,p.goal_yaw_tolerance_rad);
  }
  wire::MotionDemand step(SourceTime now,double receipt) {
    wire::MotionDemand d;
    d.source_stamp=stamp(now);d.body_source_stamp=bodySourceStamp(core_.odometry());
    d.valid_until=stampNs(timeNs(d.source_stamp)+100000000LL);d.sequence=++demand_sequence_;d.transport_mode=mode_;
    d.hold=true;d.reason="waiting_execution_permission";
    const wire::TrajectoryValidation* proof=nullptr;
    const auto finish=[&]() {recordEmission(d,receipt,proof,false);return d;};
    if(!permit_)return finish();
    const auto& p=*permit_;
    d.version=p.version;d.execution_id=p.execution_id;d.control_epoch=p.control_epoch;
    d.sdk_session=p.sdk_session;d.sdk_arm_generation=p.sdk_arm_generation;d.permit_sequence=p.sequence;
    d.validation_sequence=p.validation_sequence;d.trajectory_id=p.trajectory_id;
    const char* blocked=ordinaryBlockReason(now,proof);
    if(blocked) {
      if(std::string(blocked)=="handoff_ack_timeout")handoff_->retired=true;
      d.reason=blocked;core_.suspendOutput(now,receipt,d.reason,false);return finish();
    }
    // A skipped tick cannot refresh any source or steady lease. After the
    // earliest original lease expires, this source remains stopped until the
    // actual source clock advances, even if another proof arrives meanwhile.
    if(paused_source_ns_&&now.nanoseconds()!=paused_source_ns_)paused_source_ns_=0;
    if(paused_source_ns_==now.nanoseconds()||
       (last_emission_&&last_emission_->control_source_ns==now.nanoseconds()&&
        (!std::isfinite(receipt)||receipt>=last_emission_->steady_until))) {
      paused_source_ns_=now.nanoseconds();d.reason="source_clock_paused_command_expired";
      core_.suspendOutput(now,receipt,d.reason,false);return finish();
    }
    // The BT owns authority; the validator owns collision evidence. This
    // sequence records the actual evidence used to generate the command and
    // is a minimum revision for its later native sweep. Do not permanently
    // embed that snapshot's nearly expired safety lease into a new control
    // request: the native validator must independently recheck this SAME curve
    // with current evidence and report the actual revision/deadline it used.
    // Neither the original control source time nor the owner's lease renews.
    d.validation_sequence=proof->sequence;
    d.valid_until=originalTimeCap(d.valid_until,{p.valid_until});
    const auto out=p.phase=="aligning" && p.has_goal_yaw?
      core_.align(p.goal_yaw,p.goal_yaw_tolerance_rad,now,receipt,true):core_.step(now,receipt,true);
    d.velocity.linear.x=out.forward;d.velocity.angular.z=out.yaw_rate;
    d.hold=out.forward==0.&&out.yaw_rate==0.;d.reason=out.reason;
    // Deliberately left false: a separate raw-ray guard must sign the demand.
    d.safety_checked=false;
    if(!d.hold) {
      demands_.push_back(d);while(demands_.size()>16)demands_.pop_front();
    }
    return finish();
  }
  std::uint64_t writerCommitSequence()const{return writer_commit_sequence_;}
  const std::string& handoffReason()const{return handoff_reason_;}
  bool handoff(const wire::ExecutionHandoffGrant& g,SourceTime now,double receipt) {
    if(!writer_handoff_||g.schema_version!=2||g.handoff_id.empty()||g.sequence==0||
       !versionValid(g.candidate.version,config_)||g.candidate.transport_mode!=mode_||
       g.candidate.frame_id!=config_.planning_frame||!g.candidate.allowed||g.candidate.geometry_committed||
       g.expected_commit_sequence!=writer_commit_sequence_||
       !writerAuthorityMatches(g.incumbent)||!writerAuthorityMatches(g.candidate))return false;
    if(handoff_&&handoff_->grant.handoff_id==g.handoff_id) {
      // Fixed transaction: a duplicate may revoke, never refresh source/lease
      // or replace the candidate geometry under the same transaction ID.
      if(g.revoked&&g.sequence>=handoff_->grant.sequence) {handoff_->retired=true;return true;}
      return g==handoff_->grant;
    }
    if(handoff_&&!handoff_->applied)return false;
    if(!handoffModeValid(g,now))return false;
    // A locally infeasible braking boundary cannot be resolved by an ordinary
    // moving replacement. Require the Owner's actual SDK stationary witness.
    if(core_.requiresMeasuredReentry()&&g.transition_mode!=wire::ExecutionHandoffGrant::STATIONARY_REENTRY) {
      handoff_reason_="braking_envelope_requires_stationary_reentry";return false;
    }
    if(g.revoked||!timed(now,g.source_stamp,g.valid_until,.75)||
       !validSourceTime(g.transition_deadline)||timeNs(g.transition_deadline)<=now.nanoseconds()||timeNs(g.transition_deadline)>timeNs(g.valid_until)||
       !timed(now,g.candidate.source_stamp,g.candidate.valid_until,.75)||
       !permit_||g.incumbent.version!=permit_->version||g.incumbent.trajectory_id!=permit_->trajectory_id||
       g.incumbent.execution_id!=permit_->execution_id||g.incumbent.control_epoch!=permit_->control_epoch)return false;
    const auto* proof=proofFor(g.candidate,now);
    const auto* bound=proof?proposalFor(g.candidate.version,proof->proposal_id):nullptr;
    const wire::SupportReference* s=nullptr;
    if(pending_support_&&pending_support_->version==g.candidate.version)s=&*pending_support_;
    else if(active_support_&&active_support_->version==g.candidate.version)s=&*active_support_;
    if(!proof||!bound||!s||!supportMatches(*proof,*s)||!candidate_||
       candidate_->id!=g.candidate.trajectory_id||candidate_->generation!=g.candidate.version.reference_generation||
       !(candidate_->identity==identity(g.candidate.version)))return false;
    TrackerCore trial(core_);
    const auto task=taskFor(g.candidate.version,g.candidate.goal_position,sourceTime(bound->source_stamp));
    const bool already=committed_version_&&g.candidate.version==*committed_version_&&
      core_.trajectoryId()==g.candidate.trajectory_id;
    std::string entry_reason;const auto observed=observeCandidateEntry(*proof,now,entry_reason);
    if(!observed) {handoff_reason_="handoff_prepare:"+entry_reason;return false;}
    if(!already&&!trial.admitRevision(task,*observed,preparedSupport(*s),now,receipt,true)) {
      handoff_reason_="handoff_prepare:"+trial.candidateReason();return false;
    }
    if(handoff_)tombstone_=std::move(handoff_);
    handoff_=PendingHandoff{g,*observed,*s,task,std::move(trial),false,false,{}};
    handoff_reason_="prepared_waiting_writer";return true;
  }
  std::optional<wire::PreparedMotionDemand> preparedStep(SourceTime now,double receipt) {
    if(!writer_handoff_||!handoff_||handoff_->retired||handoff_->applied)return {};
    auto& h=*handoff_;const auto& g=h.grant;const auto& p=g.candidate;
    if(timeNs(g.transition_deadline)<=now.nanoseconds()||timeNs(g.valid_until)<=now.nanoseconds()||timeNs(p.valid_until)<=now.nanoseconds()) {
      h.retired=true;handoff_reason_="handoff_ack_timeout";return {};
    }
    const auto* proof=proofFor(p,now);
    if(!proof||!proof->whole_curve||!supportMatches(*proof,h.support))return {};
    const auto entry=h.controller.refreshPreparedState(core_,now,receipt);
    if(!entry) {handoff_reason_="prepared_actual_entry_not_admissible";return {};}
    wire::PreparedMotionDemand out;out.schema_version=1;out.handoff_id=g.handoff_id;
    out.grant_sequence=g.sequence;out.expected_commit_sequence=g.expected_commit_sequence;
    auto& d=out.demand;d.version=p.version;d.execution_id=p.execution_id;d.control_epoch=p.control_epoch;
    d.sdk_session=p.sdk_session;d.sdk_arm_generation=p.sdk_arm_generation;d.permit_sequence=p.sequence;
    d.validation_sequence=proof->sequence;d.trajectory_id=p.trajectory_id;d.sequence=++demand_sequence_;
    d.source_stamp=stamp(now);d.body_source_stamp=bodySourceStamp(core_.odometry());
    d.valid_until=originalTimeCap(stampNs(timeNs(d.source_stamp)+100000000LL),
      {g.valid_until,g.transition_deadline,p.valid_until});
    d.transport_mode=mode_;d.braking_model_sha256=braking_model_sha256_;
    const auto command=h.controller.step(now,receipt,true);
    d.velocity.linear.x=command.forward;d.velocity.angular.z=command.yaw_rate;
    d.hold=command.forward==0.&&command.yaw_rate==0.;d.reason=command.reason;
    out.entry_source_stamp=d.body_source_stamp;out.measured_pose.header.stamp=d.body_source_stamp;
    out.measured_pose.header.frame_id=config_.planning_frame;
    const auto& measured=core_.odometry();out.measured_pose.pose.position.x=measured.position.x();
    out.measured_pose.pose.position.y=measured.position.y();out.measured_pose.pose.position.z=measured.position.z();
    out.measured_pose.pose.orientation.x=measured.orientation.x();out.measured_pose.pose.orientation.y=measured.orientation.y();
    out.measured_pose.pose.orientation.z=measured.orientation.z();out.measured_pose.pose.orientation.w=measured.orientation.w();
    out.measured_twist.linear.x=measured.velocity_in_frame.x();out.measured_twist.linear.y=measured.velocity_in_frame.y();
    out.measured_twist.linear.z=measured.velocity_in_frame.z();out.measured_twist.angular.z=measured.angular_velocity_in_frame.z();
    out.curve_entry_pose.position.x=entry->position.x();out.curve_entry_pose.position.y=entry->position.y();
    out.curve_entry_pose.position.z=entry->position.z();out.curve_entry_pose.orientation=out.measured_pose.pose.orientation;
    out.curve_entry_twist.linear.x=entry->velocity.x();out.curve_entry_twist.linear.y=entry->velocity.y();
    out.curve_entry_twist.linear.z=entry->velocity.z();out.curve_time=entry->time;
    out.position_error_m=entry->position_error;out.velocity_error_mps=entry->velocity_error;
    out.position_tolerance_m=config_.join_limit;out.velocity_tolerance_mps=.05;
    auto& a=out.entry_admission;a.sequence=++admission_sequence_;a.version=p.version;
    a.trajectory_id=p.trajectory_id;a.validation_sequence=proof->sequence;a.body_source_stamp=d.body_source_stamp;
    a.checked_at=stamp(now);a.valid_until=d.valid_until;a.accepted=true;a.reason="actual_entry_prepared_not_applied";a.transport_mode=mode_;
    recordEmission(d,receipt,proof,true,&p);
    if(!emissions_.empty()) {
      capEmission(emissions_.back(),timeNs(g.valid_until));
      capEmission(emissions_.back(),timeNs(g.transition_deadline));
      capEmission(emissions_.back(),timeNs(p.valid_until));
      capEmission(emissions_.back(),timeNs(p.source_stamp)+durationNs(.75));
    }
    h.demands.push_back(out);while(h.demands.size()>8)h.demands.pop_front();
    handoff_reason_="prepared_waiting_writer";return out;
  }
  bool commitAck(const wire::ExecutionCommitAck& a,SourceTime now,double receipt) {
    if(!writer_handoff_||a.schema_version!=1||!writerAuthorityMatches(a)||
       a.sequence<=ack_sequence_||a.transport_mode!=mode_)return false;
    // The initial ordinary writer call records commit 1 without a handoff.
    if(a.handoff_id.empty()) {
      const auto first=std::find_if(permit_history_.begin(),permit_history_.end(),[&](const auto& p){
        return a.permit_sequence==p.sequence&&a.candidate_version==p.version&&a.candidate_trajectory_id==p.trajectory_id&&
          a.execution_id==p.execution_id&&a.control_epoch==p.control_epoch&&a.sdk_session==p.sdk_session&&
          a.sdk_arm_generation==p.sdk_arm_generation&&p.allowed&&p.geometry_committed&&!p.revoked&&
          timeNs(p.source_stamp)<=timeNs(a.applied_at)&&timeNs(p.valid_until)>timeNs(a.applied_at);
      });
      const auto installed=std::find_if(installations_.rbegin(),installations_.rend(),[&](const auto& i){
        return i.fact.version==a.candidate_version&&i.fact.trajectory_id==a.candidate_trajectory_id&&
          timeNs(i.fact.installed_at)<=timeNs(a.applied_at);
      });
      if(!a.applied||a.previous_commit_sequence||a.commit_sequence!=1||first==permit_history_.end()||
         installed==installations_.rend()) {initial_ack_reason_="initial_writer_fact_not_bound_to_installed_geometry";return false;}
      if(writer_commit_sequence_) {
        if(writer_commit_sequence_!=1||!initial_writer_ack_)return false;
        auto expected=*initial_writer_ack_,actual=a;
        expected.sequence=actual.sequence=0;expected.write_acknowledged=actual.write_acknowledged=false;
        expected.reason.clear();actual.reason.clear();
        if(expected!=actual)return false;
        ack_sequence_=a.sequence;initial_ack_reason_="initial_writer_callback_fact_updated";return true;
      }
      // The writer call is an irreversible fact, not an assertion that the
      // owner's latest desired curve was installed. Cross-topic ordering can
      // deliver this ACK after a different initial intent or cancellation.
      const bool current=committed_version_&&a.candidate_version==*committed_version_&&
        core_.hasInstalledGeometry()&&core_.trajectoryId()==a.candidate_trajectory_id;
      writer_commit_sequence_=1;ack_sequence_=a.sequence;initial_writer_ack_=a;
      if(!current||!a.write_submitted||timeNs(a.valid_until)<=now.nanoseconds()||
         !permit_||!timed(now,permit_->source_stamp,permit_->valid_until,.75)) {
        committed_version_=a.candidate_version;permit_=owner_permit_=*first;
        last_permit_sequence_=std::max(last_permit_sequence_,first->sequence);
        if(handoff_)handoff_->retired=true;
        initial_ack_reason_="initial_writer_applied_fact_restored_hold";
        installation_=*installed; // Original installation fact, never re-dated.
        core_.recordAppliedHold(installed->task,a.candidate_trajectory_id,receipt,initial_ack_reason_);
      } else {
        initial_ack_reason_="initial_writer_applied_installed_geometry";
        if(last_emission_&&a.demand_sequence==last_emission_->demand.sequence&&
           a.demand_source_stamp==last_emission_->demand.source_stamp&&
           a.demand_body_source_stamp==last_emission_->demand.body_source_stamp&&
           a.applied_velocity==last_emission_->demand.velocity)
          capEmission(*last_emission_,timeNs(a.valid_until));
      }
      return true;
    }
    PendingHandoff* target=handoff_&&handoff_->grant.handoff_id==a.handoff_id?&*handoff_:
      tombstone_&&tombstone_->grant.handoff_id==a.handoff_id?&*tombstone_:nullptr;
    if(!target)return false;
    auto& h=*target;const auto& g=h.grant;const auto& p=g.candidate;
    if(a.handoff_id!=g.handoff_id||a.grant_sequence!=g.sequence||
       a.previous_commit_sequence!=g.expected_commit_sequence||a.candidate_version!=p.version||
       a.incumbent_version!=g.incumbent.version||a.candidate_trajectory_id!=p.trajectory_id||
       a.incumbent_trajectory_id!=g.incumbent.trajectory_id||a.execution_id!=p.execution_id||
       a.control_epoch!=p.control_epoch||a.sdk_session!=p.sdk_session||a.sdk_arm_generation!=p.sdk_arm_generation||
       a.permit_sequence!=p.sequence)return false;
    if(!a.applied) {
      // A negative writer fact only resolves the transaction when no writer
      // identity advanced. Command evidence may be absent, but the fixed
      // grant's exact permit/owner identity above is still mandatory.
      if(a.write_submitted||a.write_acknowledged||a.commit_sequence!=a.previous_commit_sequence||
         a.commit_sequence!=writer_commit_sequence_)return false;
      h.retired=true;h.applied=true;ack_sequence_=a.sequence;handoff_reason_="writer_rejected_prepared";return true;
    }
    if(a.commit_sequence!=a.previous_commit_sequence+1)return false;
    if(h.applied&&writer_commit_sequence_==a.commit_sequence) {ack_sequence_=a.sequence;return true;}
    if(a.previous_commit_sequence!=writer_commit_sequence_)return false;
    const auto found=std::find_if(h.demands.begin(),h.demands.end(),[&](const auto& x) {
      return x.demand.sequence==a.demand_sequence&&x.entry_admission.sequence==a.entry_admission_sequence&&
        x.demand.source_stamp==a.demand_source_stamp&&x.demand.body_source_stamp==a.demand_body_source_stamp&&
        x.entry_source_stamp==a.entry_source_stamp&&
        x.demand.velocity==a.applied_velocity&&x.curve_time==a.curve_time;
    });
    if(found==h.demands.end())return false;
    // Applied is irreversible even after cancellation/expiry. Promote identity
    // first; uncertain delivery or stale join can only HOLD the new identity.
    const bool was_retired=h.retired;h.applied=true;writer_commit_sequence_=a.commit_sequence;ack_sequence_=a.sequence;
    if(handoff_&&target!=&*handoff_)handoff_->retired=true; // Late applied fact fences the newer conditional transaction too.
    auto promoted=p;promoted.geometry_committed=true;
    committed_version_=p.version;permit_=owner_permit_=promoted;last_permit_sequence_=std::max(last_permit_sequence_,p.sequence);
    active_support_=h.support;
    const auto* proof=proofFor(p,now);if(proof)active_validation_=leased_validation_=*proof;
    const auto entry=h.controller.refreshPreparedState(core_,now,receipt);
    const bool sdk_entry=sdkAppliedEntry(h.trajectory,*found,a);
    if(was_retired||!a.write_submitted||timeNs(a.valid_until)<=now.nanoseconds()||
       timeNs(p.valid_until)<=now.nanoseconds()||!entry||!proof||!sdk_entry) {
      h.retired=true;handoff_reason_="writer_applied_new_identity_hold";
      core_.recordAppliedHold(h.task,p.trajectory_id,receipt,handoff_reason_);return true;
    }
    h.controller.recordAppliedOutput(a.applied_velocity.linear.x,a.applied_velocity.angular.z,now,receipt);
    core_=std::move(h.controller);
    const auto emission=std::find_if(emissions_.rbegin(),emissions_.rend(),[&](const auto& e){
      return e.demand.sequence==found->demand.sequence&&e.demand==found->demand;
    });
    last_emission_.reset();
    if(emission!=emissions_.rend()&&!found->demand.hold) {
      last_emission_=*emission;capEmission(*last_emission_,timeNs(a.valid_until));
      last_emission_->control_source_ns=now.nanoseconds();
      last_emission_->control_revision=core_.outputControlRevision();
    }
    const auto admitted_proof=std::find_if(proofs_.rbegin(),proofs_.rend(),[&](const auto& v){
      return v.version==p.version&&v.trajectory_id==p.trajectory_id&&v.sequence==found->entry_admission.validation_sequence;
    });
    if(admitted_proof!=proofs_.rend())recordInstallation(promoted,found->entry_admission,*admitted_proof,h.task,stamp(now));
    handoff_reason_="writer_applied";return true;
  }
private:
  bool retiredWriterHoldFor(const Task& next)const {
    return writer_handoff_&&writer_authority_&&retired_task_&&
      *retired_task_==writer_authority_->version.task_id&&
      next.identity.task_id!=*retired_task_&&core_.active()&&core_.holding()&&
      !core_.hasInstalledGeometry()&&core_.task().identity.task_id==*retired_task_;
  }
  // SDK commit and ACK counters belong to an armed execution, not this ROS
  // node's lifetime. Cancel keeps the old scope so an irreversible late fact
  // can still be recorded under HOLD. Only a new, freshly proved positive
  // authority for a retired task can replace it; geometry refresh cannot.
  template<class Authority>
  bool writerAuthorityMatches(const Authority& p)const {
    if(!writer_authority_)return false;
    const auto& current=*writer_authority_;
    const auto& version=[&]() -> const wire::ExecutionVersion& {
      if constexpr(std::is_same_v<Authority,wire::ExecutionCommitAck>)return p.candidate_version;
      else return p.version;
    }();
    return version.session_id==current.version.session_id&&version.task_id==current.version.task_id&&
      version.route_id==current.version.route_id&&version.route_hash==current.version.route_hash&&
      version.map_version_id==current.version.map_version_id&&
      version.localization_epoch==current.version.localization_epoch&&
      version.localization_seed_id==current.version.localization_seed_id&&p.transport_mode==current.transport_mode&&
      p.execution_id==current.execution_id&&p.control_epoch==current.control_epoch&&
      p.sdk_session==current.sdk_session&&p.sdk_arm_generation==current.sdk_arm_generation;
  }
  bool writerAuthorityTransitionAllowed(const wire::ExecutionPermit& p)const {
    if(!writer_handoff_||!p.allowed||!writer_authority_||writerAuthorityMatches(p))return true;
    const auto& previous=*writer_authority_;
    return retired_task_&&*retired_task_==previous.version.task_id&&
      p.version.task_id!=previous.version.task_id&&p.execution_id!=previous.execution_id&&
      p.control_epoch>previous.control_epoch;
  }
  void bindWriterAuthority(const wire::ExecutionPermit& p) {
    if(!writer_handoff_||!p.allowed||writerAuthorityMatches(p))return;
    writer_authority_=p;
    writer_commit_sequence_=ack_sequence_=0;initial_writer_ack_.reset();
    handoff_.reset();tombstone_.reset();handoff_reason_.clear();initial_ack_reason_.clear();
    // Historical installations, permits and emitted commands retain their
    // original identities/times. They can never match this new ACK scope.
  }
  struct ControlEmission {
    wire::MotionDemand demand;
    wire::ExecutionPermit authority;
    double emitted_receipt,steady_until;
    std::int64_t control_source_ns;
    std::uint64_t control_revision;
  };
  static void capEmission(ControlEmission& e,std::int64_t until_ns) {
    // Always measured from the ORIGINAL demand emission, never proof receipt.
    e.steady_until=std::min(e.steady_until,e.emitted_receipt+
      sourceDeltaSeconds(until_ns,timeNs(e.demand.source_stamp)));
  }
  static bool sameAuthority(const wire::ExecutionPermit& a,const wire::ExecutionPermit& b) {
    return a.version==b.version&&a.trajectory_id==b.trajectory_id&&a.execution_id==b.execution_id&&
      a.control_epoch==b.control_epoch&&a.sdk_session==b.sdk_session&&a.sdk_arm_generation==b.sdk_arm_generation&&
      a.phase==b.phase&&a.has_goal_yaw==b.has_goal_yaw&&a.goal_yaw==b.goal_yaw&&
      a.goal_yaw_tolerance_rad==b.goal_yaw_tolerance_rad&&a.goal_position==b.goal_position;
  }
  static void capTrajectory(ControlEmission& e,const wire::TrajectoryValidation& proof) {
    capEmission(e,timeNs(proof.valid_until));capEmission(e,timeNs(proof.check_end)+durationNs(.25));
    capEmission(e,timeNs(proof.source_stamp)+durationNs(.25));capEmission(e,timeNs(proof.body_source_stamp)+durationNs(.4));
    capEmission(e,timeNs(proof.front_ray_source_stamp)+durationNs(.5));capEmission(e,timeNs(proof.rear_ray_source_stamp)+durationNs(.5));
  }
  void capPermitEmissions(const wire::ExecutionPermit& p) {
    for(auto& e:emissions_)if(sameAuthority(e.authority,p)) {
      capEmission(e,timeNs(p.valid_until));capEmission(e,timeNs(p.source_stamp)+durationNs(.75));
      if(last_emission_&&last_emission_->demand.sequence==e.demand.sequence) {
        capEmission(*last_emission_,timeNs(p.valid_until));
        capEmission(*last_emission_,timeNs(p.source_stamp)+durationNs(.75));
      }
    }
  }
  void recordEmission(const wire::MotionDemand& d,double receipt,
      const wire::TrajectoryValidation* proof,bool prepared,const wire::ExecutionPermit* authority=nullptr) {
    const auto bound=authority?*authority:(permit_?*permit_:wire::ExecutionPermit{});
    ControlEmission e{d,bound,receipt,receipt,timeNs(d.source_stamp),core_.outputControlRevision()};
    e.steady_until=receipt+sourceDeltaSeconds(timeNs(d.valid_until),timeNs(d.source_stamp));
    if(proof)capTrajectory(e,*proof);
    if(!prepared&&permit_) {
      capEmission(e,timeNs(permit_->valid_until));
      capEmission(e,timeNs(permit_->source_stamp)+durationNs(.75));
    }
    emissions_.push_back(e);while(emissions_.size()>32)emissions_.pop_front();
    if(!prepared) {if(d.hold)last_emission_.reset();else last_emission_=e;}
  }
  void observeMotionExpiry(const wire::MotionValidation& v,SourceTime now) {
    // A positive native observation adds only an expiry cap, never authority.
    if(!v.valid||!v.handoff_id.empty()||!versionValid(v.version,config_)||v.transport_mode!=mode_||
       v.frame_id!=config_.planning_frame||v.map_snapshot_revision==0||v.sequence==0||
       braking_model_sha256_.size()!=64||v.braking_model_sha256!=braking_model_sha256_)return;
    auto e=std::find_if(emissions_.rbegin(),emissions_.rend(),[&](const auto& x){
      const auto& d=x.demand;
      return !d.hold&&d.sequence==v.demand_sequence&&d.version==v.version&&
        d.execution_id==v.execution_id&&d.control_epoch==v.control_epoch&&d.sdk_session==v.sdk_session&&
        d.sdk_arm_generation==v.sdk_arm_generation&&d.trajectory_id==v.trajectory_id&&
        d.permit_sequence==v.permit_sequence&&v.trajectory_validation_sequence>=d.validation_sequence&&
        d.source_stamp==v.demand_source_stamp&&d.body_source_stamp==v.demand_body_source_stamp&&
        d.valid_until==v.demand_valid_until&&d.velocity==v.velocity;
    });
    if(e==emissions_.rend())return;
    const auto begin=timeNs(v.check_begin),end=timeNs(v.check_end),until=timeNs(v.valid_until);
    if(!validSourceTime(v.check_begin)||!validSourceTime(v.check_end)||!validSourceTime(v.valid_until)||
       begin<timeNs(e->demand.source_stamp)||end<begin||!sourceFresh(now,end,.1,.02)||
       until<=end||until>timeNs(e->demand.valid_until)||
       !freshStamp(now,e->demand.body_source_stamp,.1)||!freshStamp(now,v.body_source_stamp,.1)||
       !freshStamp(now,v.front_ray_source_stamp,.5)||!freshStamp(now,v.rear_ray_source_stamp,.5))return;
    for(const auto deadline:{until,end+durationNs(.1),timeNs(v.body_source_stamp)+durationNs(.1),
        timeNs(e->demand.body_source_stamp)+durationNs(.1),
        timeNs(v.front_ray_source_stamp)+durationNs(.5),timeNs(v.rear_ray_source_stamp)+durationNs(.5)}) {
      capEmission(*e,deadline);
      if(last_emission_&&last_emission_->demand.sequence==e->demand.sequence)
        capEmission(*last_emission_,deadline);
    }
  }
  const char* ordinaryBlockReason(SourceTime now,const wire::TrajectoryValidation*& proof)const {
    if(!permit_)return "waiting_execution_permission";
    const auto& p=*permit_;
    if(writer_handoff_&&handoff_&&!handoff_->applied&&
       handoff_->grant.transition_mode==wire::ExecutionHandoffGrant::STATIONARY_REENTRY)
      return "stationary_reentry_waiting_writer";
    if(writer_handoff_&&handoff_&&handoff_->retired&&(!handoff_->applied||
       (p.version==handoff_->grant.candidate.version&&p.trajectory_id==handoff_->grant.candidate.trajectory_id)))
      return "handoff_retired_or_applied_uncertain";
    if(writer_handoff_&&handoff_&&!handoff_->applied&&timeNs(handoff_->grant.transition_deadline)<=now.nanoseconds())
      return "handoff_ack_timeout";
    if(!p.allowed&&!p.revoked)return "geometry_only_no_motion_permission";
    proof=proofFor(p,now);
    if(!p.allowed||p.revoked||!timed(now,p.source_stamp,p.valid_until,.75)||
       core_.trajectoryId()!=p.trajectory_id||core_.generation()!=p.version.reference_generation||
       !(core_.task().identity==identity(p.version))||!proof||!proofFresh(*proof,now)||
       !active_support_||!supportMatches(*proof,*active_support_)||
       (active_validation_&&!active_validation_->valid&&active_validation_->sequence>=p.validation_sequence))
      return "permission_or_native_proof_expired";
    if(p.phase=="holding"||p.phase=="stopping"||p.phase=="terminal")return p.phase.c_str();
    if(p.phase=="aligning"&&(!p.has_goal_yaw||!proof->goal_yaw_checked||
       std::abs(angle(proof->checked_goal_yaw-p.goal_yaw))>1e-6))return "goal_yaw_sweep_not_verified";
    return nullptr;
  }
  struct Installation {wire::TrackerGeometryReceipt fact;Task task;};
  void recordInstallation(const wire::ExecutionPermit& p,const wire::TrajectoryAdmission& a,
      const wire::TrajectoryValidation& v,const Task& task,const builtin_interfaces::msg::Time& at) {
    wire::TrackerGeometryReceipt r;r.schema_version=1;r.version=p.version;
    r.frame_id=config_.planning_frame;r.transport_mode=mode_;r.trajectory_id=p.trajectory_id;
    r.installation_sequence=++installation_sequence_;r.permit_sequence=p.sequence;
    r.admission_sequence=a.sequence;r.validation_sequence=v.sequence;r.installed_at=at;
    r.body_source_stamp=bodySourceStamp(core_.odometry());r.installed=true;r.reason="tracker_geometry_installed";
    installation_=Installation{r,task};installations_.push_back(*installation_);
    while(installations_.size()>8)installations_.pop_front();
  }
  bool handoffModeValid(const wire::ExecutionHandoffGrant& g,SourceTime now)const {
    if(g.transition_mode==wire::ExecutionHandoffGrant::CONTINUOUS_REPLACE)
      return g.incumbent.allowed&&!g.incumbent.revoked&&g.incumbent.geometry_committed&&
        (g.incumbent.phase=="tracking"||g.incumbent.phase=="aligning");
    if(g.transition_mode!=wire::ExecutionHandoffGrant::STATIONARY_REENTRY||
       g.incumbent.allowed||g.incumbent.revoked||!g.incumbent.geometry_committed||g.incumbent.phase!="holding"||
       !owner_permit_||owner_permit_->allowed||owner_permit_->revoked||owner_permit_->phase!="holding"||
       owner_permit_->version!=g.incumbent.version||owner_permit_->trajectory_id!=g.incumbent.trajectory_id)return false;
    const auto& e=g.stationary_evidence;const auto& p=g.incumbent;
    return e.schema_version==1&&e.usable&&e.nonzero_blocked&&e.sequence>0&&e.zero_write_sequence>0&&
      e.version==p.version&&e.execution_id==p.execution_id&&e.control_epoch==p.control_epoch&&
      e.sdk_session==p.sdk_session&&e.sdk_arm_generation==p.sdk_arm_generation&&e.transport_mode==mode_&&
      e.writer_commit_sequence==g.expected_commit_sequence&&e.applied_trajectory_id==p.trajectory_id&&
      e.mc_raw_stamp_ns>0&&!e.mc_clock_epoch.empty()&&!e.time_basis.empty()&&
      e.stationary_samples>=config_.stationary_minimum_samples&&std::isfinite(e.stationary_duration_sec)&&
      e.stationary_duration_sec>=config_.stationary_reentry_duration_s&&
      std::isfinite(e.measured_linear_mps)&&std::isfinite(e.measured_angular_radps)&&
      std::abs(e.measured_linear_mps)<=config_.stationary_linear_threshold_mps&&
      std::abs(e.measured_angular_radps)<=config_.stationary_angular_threshold_radps&&
      validSourceTime(e.zero_ack_at)&&validSourceTime(e.capture_lower_bound)&&validSourceTime(e.capture_upper_bound)&&
      timeNs(e.capture_lower_bound)>timeNs(e.zero_ack_at)&&timeNs(e.capture_upper_bound)>=timeNs(e.capture_lower_bound)&&
      timed(now,e.source_stamp,e.valid_until,.250001)&&
      timeNs(g.source_stamp)>=timeNs(e.source_stamp)&&g.retain_incumbent_until==g.source_stamp&&
      timeNs(g.valid_until)<=timeNs(e.valid_until)&&timeNs(g.candidate.valid_until)<=timeNs(e.valid_until)&&
      timeNs(g.valid_until)-timeNs(g.source_stamp)<=250001000LL;
  }
  struct PendingHandoff {wire::ExecutionHandoffGrant grant;Trajectory trajectory;wire::SupportReference support;
    Task task;TrackerCore controller;bool retired,applied;std::deque<wire::PreparedMotionDemand> demands;};
  bool sdkAppliedEntry(const Trajectory& t,const wire::PreparedMotionDemand& entry,const wire::ExecutionCommitAck& a)const {
    if(a.measured_pose.header.frame_id!=config_.planning_frame||a.measured_pose.header.stamp!=a.body_source_stamp||
       !validSourceTime(a.body_source_stamp)||!validSourceTime(a.entry_source_stamp)||!validSourceTime(a.applied_at)||
       timeNs(a.body_source_stamp)<timeNs(a.entry_source_stamp)||
       !sourceFresh(sourceTime(a.applied_at),timeNs(a.body_source_stamp),.1,.02)||
       t.points.size()<4||t.knots.size()!=t.points.size()+4)return false;
    Eigen::MatrixXd points(3,t.points.size());Eigen::VectorXd knots(t.knots.size());
    for(std::size_t i=0;i<t.points.size();++i)points.col(i)=t.points[i];
    for(std::size_t i=0;i<t.knots.size();++i)knots[i]=t.knots[i];
    scan_planner::UniformBspline curve(points,3,.1);curve.setKnot(knots);const auto velocity=curve.getDerivative();
    const Eigen::Vector3d measured{a.measured_pose.pose.position.x,a.measured_pose.pose.position.y,a.measured_pose.pose.position.z};
    const Eigen::Vector3d actual_velocity{a.measured_twist.linear.x,a.measured_twist.linear.y,a.measured_twist.linear.z};
    if(config_.isolated_reference_model) {
      return isolatedAppliedEntryValid(config_,t,entry.curve_time,measured,actual_velocity,
          sourceDeltaSeconds(timeNs(a.body_source_stamp),timeNs(a.entry_source_stamp)));
    }
    const double travel=config_.max_speed*std::max(0.,sourceDeltaSeconds(timeNs(a.body_source_stamp),timeNs(a.entry_source_stamp)))+config_.join_limit;
    // Bounded projection around the certified original entry. Neither another
    // branch nor a same-XY floor can be selected by this ACK fact check.
    double best=std::numeric_limits<double>::infinity(),time=entry.curve_time;
    const double lo=std::max(0.,time-travel/config_.max_speed),hi=std::min(curve.getTimeSum(),time+travel/config_.max_speed);
    for(unsigned i=0;i<=80;++i){const double q=lo+(hi-lo)*i/80.;const double error=(curve.evaluateDeBoorT(q)-measured).norm();
      if(error<best){best=error;time=q;}}
    return measured.allFinite()&&actual_velocity.allFinite()&&best<=config_.join_limit&&
      (velocity.evaluateDeBoorT(time)-actual_velocity).norm()<=.05;
  }
  const wire::TrajectoryValidation* proofFor(const wire::ExecutionPermit& p,SourceTime now,bool exact_commit=false) const {
    const wire::TrajectoryValidation* exact=nullptr;const wire::TrajectoryValidation* latest=nullptr;
    for(const auto& v:proofs_)if(v.version==p.version&&v.trajectory_id==p.trajectory_id) {
      if(!latest||v.sequence>latest->sequence)latest=&v;
      if(v.sequence==p.validation_sequence)exact=&v;
    }
    // The signed sequence is a minimum evidence revision for this complete
    // version/curve, not a timer-phase lock. Never fall back around a newer
    // invalid/expired observation. New geometry still needs its exact ACK.
    if(!latest||latest->sequence<p.validation_sequence||!proofFresh(*latest,now))return nullptr;
    return exact_commit?(exact&&proofFresh(*exact,now)?exact:nullptr):latest;
  }
  const wire::ReferenceProposal* proposalFor(const wire::ExecutionVersion& v,const std::string& id)const {
    for(const auto& p:proposals_)if(p.version==v&&p.proposal_id==id)return &p;
    return nullptr;
  }
  Task taskFor(const wire::ExecutionVersion& v,const geometry_msgs::msg::Point& goal,SourceTime source) const {
    Task t;t.session_id=v.session_id;t.frame_id=config_.planning_frame;t.generation=v.reference_generation;
    t.identity=identity(v);t.active=true;t.goal={goal.x,goal.y,goal.z};t.issued_at=source;t.issued_at_ns=source.nanoseconds();return t;
  }
  bool proofFresh(const wire::TrajectoryValidation& v,SourceTime now)const {
    const auto end=timeNs(v.check_end),begin=timeNs(v.check_begin);
    const bool domain=finite(v.checked_from_time)&&finite(v.checked_to_time)&&finite(v.curve_duration)&&
      finite(v.valid_start_time)&&finite(v.valid_start_arc_length)&&v.checked_from_time>=0.&&
      v.checked_from_time<=v.valid_start_time&&v.valid_start_time<=v.checked_to_time&&
      v.curve_duration>0.&&v.curve_duration<=120.&&std::abs(v.checked_to_time-v.curve_duration)<=1e-6;
    const bool covered=domain&&((v.whole_curve&&!v.remaining_curve&&v.checked_from_time==0.) || (!v.whole_curve&&v.remaining_curve&&core_.active()&&
      v.trajectory_id==core_.trajectoryId()&&v.version.reference_generation==core_.generation()&&
      identity(v.version)==core_.task().identity&&core_.remainingProofCovers(v.checked_from_time,
        v.valid_start_time,v.valid_start_arc_length,v.checked_to_time,v.curve_duration,v.reverse_margin_m)));
    return v.valid&&v.frame_id==config_.planning_frame&&v.collision_policy=="observed_free"&&covered&&
      v.map_snapshot_revision>0&&v.sequence>0&&v.transport_mode==mode_&&
      validSourceTime(v.check_begin)&&validSourceTime(v.check_end)&&end>=begin&&
      sourceFresh(now,end,.25,.02)&&validSourceTime(v.valid_until)&&timeNs(v.valid_until)>now.nanoseconds()&&
      // Retain the existing lease-length bound; absolute expiry is exact.
      timeNs(v.valid_until)-end<=250001000LL&&freshStamp(now,v.source_stamp,.25)&&
      timeNs(v.source_stamp)>=end&&freshStamp(now,v.body_source_stamp,.4)&&
      freshStamp(now,v.front_ray_source_stamp,.5)&&freshStamp(now,v.rear_ray_source_stamp,.5);
  }
  bool supportMatches(const wire::TrajectoryValidation& v,const wire::SupportReference& s)const {
    return s.version==v.version&&s.verified&&s.support_reference_id==v.support_reference_id&&s.support_hash==v.support_hash;
  }
  std::optional<Trajectory> observeCandidateEntry(const wire::TrajectoryValidation& proof,
      SourceTime now,std::string& reason) {
    // This is a NEW observation of the SAME immutable geometry. A latest
    // whole-curve native query is mandatory; remaining-old-curve or stale
    // certificates cannot justify a new entry. All certificate times stay
    // original and the later prepared sweep still rechecks the latest map.
    if(!preparation_failure_.empty()){reason=preparation_failure_;return {};}
    if(!candidate_||!candidate_entry_curve_) {reason="entry_curve_cache_invalid";return {};}
    if(!candidate_->prepared||!candidate_->prepared->failure.empty()) {
      reason=candidate_->prepared?candidate_->prepared->failure:"waiting_candidate_preparation";return {};
    }
    if(!proof.whole_curve||proof.remaining_curve||!proofFresh(proof,now)||
       proof.trajectory_id!=candidate_->id||proof.version.reference_generation!=candidate_->generation||
       !(identity(proof.version)==candidate_->identity)||
       std::abs(proof.curve_duration-candidate_entry_curve_->duration)>1e-6) {
      reason="entry_whole_curve_proof_missing_or_stale";
      core_.recordJoinDiagnostic(*candidate_,now,candidate_entry_curve_->duration,reason);return {};
    }
    auto observed=candidate_entry_curve_->observe(*candidate_,core_.odometry(),config_,now,
      sourceTime(proof.body_source_stamp),proof.valid_start_time,proof.valid_start_arc_length,reason);
    if(!observed)core_.recordJoinDiagnostic(*candidate_,now,candidate_entry_curve_->duration,reason);
    return observed;
  }
  Config config_;std::string mode_,braking_model_sha256_;TrackerCore core_;
  struct PreparationRequest {
    Config config;
    std::shared_ptr<const Trajectory> trajectory;
    std::shared_ptr<const wire::SupportReference> support;
  };
  using PreparationWorker=BoundedPreparationWorker<PreparationRequest,std::shared_ptr<const PreparedGeometry>>;
  static bool sameSupportGeometry(const wire::SupportReference& a,const wire::SupportReference& b) {
    return a.version==b.version&&a.support_reference_id==b.support_reference_id&&a.support_hash==b.support_hash&&
      a.support_map_sha256==b.support_map_sha256&&a.floor_id==b.floor_id&&a.segment_kind==b.segment_kind&&
      a.required_mode==b.required_mode&&a.frame_id==b.frame_id&&a.verified==b.verified&&
      a.support_ground_xyz==b.support_ground_xyz&&a.support_xy_radius_m==b.support_xy_radius_m&&
      a.body_reference_height_m==b.body_reference_height_m&&a.max_support_slope_rad==b.max_support_slope_rad&&
      a.max_support_step_m==b.max_support_step_m;
  }
  void installPreparation(std::shared_ptr<const PreparedGeometry> p) {
    candidate_->prepared=std::move(p);
    candidate_entry_curve_=std::shared_ptr<const EntryCurveCache>(candidate_->prepared,&candidate_->prepared->entry);
    if(!candidate_->prepared->failure.empty())last_prepare_reason_="candidate:"+candidate_->prepared->failure;
  }
  void schedulePreparation() {
    if(!candidate_)return;
    const wire::SupportReference* s=nullptr;
    for(const auto* support:{&pending_support_,&active_support_})
      if(*support&&(*support)->version.reference_generation==candidate_->generation&&
         identity((*support)->version)==candidate_->identity) {s=&**support;break;}
    if(config_.require_support_reference&&!s)return;
    if(preparation_request_&&preparation_request_->trajectory->id==candidate_->id&&
       preparation_request_->trajectory->generation==candidate_->generation&&
       preparation_request_->trajectory->identity==candidate_->identity&&
       ((!s&&!preparation_request_->support)||(s&&preparation_request_->support&&
         sameSupportGeometry(*preparation_request_->support,*s))))return;
    candidate_->prepared.reset();candidate_entry_curve_.reset();
    preparation_failure_.clear();
    preparation_request_=PreparationRequest{config_,std::make_shared<const Trajectory>(*candidate_),
      s?std::make_shared<const wire::SupportReference>(*s):nullptr};
    if(worker_)preparation_token_=worker_->submit(*preparation_request_);
    else installPreparation(PreparedGeometry::build(config_,*candidate_,s?supportEvidence(*s):SupportEvidence{}));
  }
  const SupportEvidence& preparedSupport(const wire::SupportReference& wire_support) const {
    // This is reachable only after cached preparation and matching proofs.
    // Never rebuild the 20k-point support on the controller's 50 Hz lane.
    static const SupportEvidence missing;
    if(!candidate_||!candidate_->prepared||!candidate_->prepared->support||
       candidate_->prepared->support->hash!=wire_support.support_hash)return missing;
    return *candidate_->prepared->support;
  }
  std::optional<wire::ReferenceProposal> proposal_;
  std::optional<wire::ExecutionPermit> permit_,owner_permit_;
  std::deque<wire::ExecutionPermit> permit_history_;
  std::deque<wire::ReferenceProposal> proposals_;
  std::optional<wire::ExecutionVersion> committed_version_;
  std::optional<Trajectory> candidate_;
  std::shared_ptr<const EntryCurveCache> candidate_entry_curve_;
  std::optional<PreparationRequest> preparation_request_;
  std::uint64_t preparation_token_{0};
  double last_preparation_seconds_{0.};std::string preparation_failure_;
  std::unique_ptr<PreparationWorker> worker_;
  std::optional<wire::TrajectoryValidation> active_validation_,pending_validation_,leased_validation_,admitted_validation_;
  std::optional<wire::SupportReference> active_support_,pending_support_;
  std::optional<wire::TrajectoryAdmission> admission_;
  std::deque<wire::TrajectoryValidation> proofs_;
  std::deque<wire::TrajectoryAdmission> admissions_;
  std::deque<wire::MotionDemand> demands_;
  std::deque<ControlEmission> emissions_;
  std::optional<ControlEmission> last_emission_;
  std::int64_t paused_source_ns_{0};
  std::optional<wire::MotionValidation> blocked_entry_;
  std::optional<std::string> retired_task_;
  std::string last_prepare_reason_,last_permit_reject_;
  std::uint64_t last_permit_sequence_{0},demand_sequence_{0},last_motion_sequence_{0},admission_sequence_{0};
  bool writer_handoff_{false};std::optional<PendingHandoff> handoff_,tombstone_;
  std::optional<wire::ExecutionPermit> writer_authority_;
  std::uint64_t writer_commit_sequence_{0},ack_sequence_{0};std::string handoff_reason_;
  std::optional<Installation> installation_;
  std::deque<Installation> installations_;
  std::optional<wire::ExecutionCommitAck> initial_writer_ack_;
  std::uint64_t installation_sequence_{0},geometry_receipt_sequence_{0};
  std::string initial_ack_reason_;
};
} // namespace d1max_trajectory_tracker
