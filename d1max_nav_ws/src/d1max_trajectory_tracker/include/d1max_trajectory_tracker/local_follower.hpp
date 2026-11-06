#pragma once
#include "d1max_trajectory_tracker/execution_contract.hpp"
#include <d1max_planning_interfaces/msg/local_execution_status.hpp>
#include <d1max_planning_interfaces/msg/local_trajectory_commit.hpp>
#include <d1max_planning_interfaces/msg/task_execution_lease.hpp>
#include <d1max_planning_interfaces/msg/task_command_feedback.hpp>
#include <d1max_planning_interfaces/msg/tagged_bspline.hpp>
#include <d1max_planning_interfaces/msg/tracking_progress.hpp>
#include <limits>

namespace d1max_trajectory_tracker {

inline wire::ExecutionVersion curveVersion(const wire::TaggedBspline& t) {
  wire::ExecutionVersion v;v.schema_version=3;v.session_id=t.session_id;v.task_id=t.task_id;
  v.route_id=t.route_id;v.route_hash=t.route_hash;v.map_version_id=t.map_version_id;
  v.localization_epoch=t.localization_epoch;v.localization_seed_id=t.localization_seed_id;
  v.reference_generation=t.generation;v.segment_id=t.segment_id;v.anchor_id=t.anchor_id;
  v.anchor_revision=t.anchor_revision;v.context_sequence=t.context_sequence;
  v.map_geometry_revision=t.map_geometry_revision;return v;
}
inline Trajectory decodeLocalCurve(const wire::TaggedBspline& t) {
  Trajectory out;out.session_id=t.session_id;out.generation=t.generation;out.frame_id=t.frame_id;
  out.point_reference=t.point_reference;out.identity=identity(curveVersion(t));
  out.id=t.trajectory.traj_id;out.start_time=seconds(t.trajectory.start_time);out.order=t.trajectory.order;
  out.knots=t.trajectory.knots;out.valid_start_time=t.valid_start_time;
  out.valid_start_arc_length=t.valid_start_arc_length;
  out.join_source_stamp=seconds(t.join_source_stamp);out.join_source_stamp_ns=timeNs(t.join_source_stamp);
  const auto& p=t.join_pose.position;const auto& q=t.join_pose.orientation;const auto& v=t.join_twist.linear;
  out.join_position={p.x,p.y,p.z};out.join_orientation={q.w,q.x,q.y,q.z};out.join_velocity={v.x,v.y,v.z};
  const auto& a=t.join_acceleration.linear;out.join_acceleration={a.x,a.y,a.z};
  out.join_acceleration_valid=t.join_acceleration_valid;
  out.strict_splice_admission=t.splice.enabled;
  if(t.body_heading_seed_valid)out.body_heading_seed_yaw=t.body_heading_seed_yaw;
  out.body_heading_sampling_intervals=t.body_heading_sampling_intervals;
  if(!decodeFrozenHeading(t.trajectory,out.frozen_yaw))out.order=-1;
  for(const auto& pt:t.trajectory.pos_pts)out.points.emplace_back(pt.x,pt.y,pt.z);
  return out;
}
inline wire::TrackingProgress trackingProgress(const TrackerCore& core,const Config& config,
    double now,const wire::LocalExecutionStatus& status) {
  const auto p=core.progress(now);wire::TrackingProgress out;
  out.header.stamp=bodySourceStamp(core.odometry());out.header.frame_id=config.planning_frame;
  out.schema_version=2;out.session_id=config.session_id;out.generation=core.generation();out.trajectory_id=core.trajectoryId();
  out.task_id=p.identity.task_id;out.route_id=p.identity.route_id;out.route_hash=p.identity.route_hash;
  out.segment_id=p.identity.segment_id;out.map_version_id=p.identity.map_version_id;
  out.localization_epoch=p.identity.localization_epoch;out.localization_seed_id=p.identity.localization_seed_id;
  out.anchor_id=p.identity.anchor_id;out.anchor_revision=p.identity.anchor_revision;out.context_sequence=p.identity.context_sequence;
  out.curve_time=p.curve_time;out.arc_length=p.arc_length;out.s_committed=p.s_committed;
  out.pose.position.x=p.position.x();out.pose.position.y=p.position.y();out.pose.position.z=p.position.z();
  out.pose.orientation.x=p.orientation.x();out.pose.orientation.y=p.orientation.y();out.pose.orientation.z=p.orientation.z();out.pose.orientation.w=p.orientation.w();
  out.twist.linear.x=p.velocity_in_frame.x();out.twist.linear.y=p.velocity_in_frame.y();out.twist.linear.z=p.velocity_in_frame.z();
  out.twist.angular.x=p.angular_velocity_in_frame.x();out.twist.angular.y=p.angular_velocity_in_frame.y();out.twist.angular.z=p.angular_velocity_in_frame.z();
  out.valid=p.valid;
  out.holding=status.holding;out.reason=p.reason;out.acceleration_valid=false;return out;
}

// Owns only local geometry and control, never goals or SDK authority. Candidate
// preparation is bounded/asynchronous. The accepted controller stays untouched
// until actual entry + Native proof admit a replacement. No SDK curve ACK/CAS.
class LocalFollower {
 public:
  enum class ExchangeClock { InputDelivery, ControlTick };
  LocalFollower(Config c,std::string mode,std::string braking_hash={},bool async=true,
      double control_period_s=.05,double command_pipeline_bound_s=.1,double writer_period_s=.05):
      config_(nativeScanConfig(std::move(c))),mode_(std::move(mode)),braking_hash_(std::move(braking_hash)),core_(config_) {
    if(mode_!="live"&&mode_!="isolated_mock")throw std::invalid_argument("explicit_transport_required");
    if(!finite(control_period_s)||control_period_s<.02||control_period_s>.1||
       !finite(command_pipeline_bound_s)||command_pipeline_bound_s<.1||command_pipeline_bound_s>.5||
       !finite(writer_period_s)||std::abs(writer_period_s-.05)>1e-9)
      throw std::invalid_argument("invalid_local_handoff_timing");
    if(async)worker_=std::make_unique<Worker>([](const Preparation& p,const Worker::Allowed& allowed) {
      return PreparedGeometry::build(p.config,p.curve,supportEvidence(p.support),allowed);
    });
  }
  TrackerCore& controller(){return core_;}
  const TrackerCore& controller()const{return core_;}
  const std::string& candidateReason()const{return candidate_reason_;}
  // Passive classification for the existing wait. It does not replace the
  // admission predicate, extend a proof, or turn a negative result positive.
  struct ValidationDiagnostic {
    std::string failure,native_reason;
    bool persistent_state=false;
    std::uint64_t sequence=0,map_revision=0;
    // Original producer stamps for passive cross-process correlation only.
    std::int64_t source_ns=0,check_begin_ns=0,check_end_ns=0,valid_until_ns=0,
      body_source_ns=0,front_source_ns=0,rear_source_ns=0;
    double age_s=0.,remaining_s=0.;
    double check_duration_s=0.;
    // Missing sources remain missing (JSON null / log nan), not age zero.
    double body_age_s=std::numeric_limits<double>::quiet_NaN();
    double front_age_s=std::numeric_limits<double>::quiet_NaN();
    double rear_age_s=std::numeric_limits<double>::quiet_NaN();
  };
  ValidationDiagnostic validationDiagnostic(double now)const {
    ValidationDiagnostic d;
    if(!active_proof_) {d.failure="no_current_proof";return d;}
    const auto& v=*active_proof_;
    d.sequence=v.sequence;d.native_reason=v.reason;
    d.source_ns=timeNs(v.source_stamp);d.check_begin_ns=timeNs(v.check_begin);
    d.check_end_ns=timeNs(v.check_end);d.valid_until_ns=timeNs(v.valid_until);
    d.body_source_ns=timeNs(v.body_source_stamp);d.front_source_ns=timeNs(v.front_ray_source_stamp);
    d.rear_source_ns=timeNs(v.rear_ray_source_stamp);
    d.persistent_state=v.valid_until.sec==0&&v.valid_until.nanosec==0;
    d.age_s=now-seconds(v.check_end);
    d.remaining_s=d.persistent_state?std::numeric_limits<double>::quiet_NaN():seconds(v.valid_until)-now;
    d.map_revision=v.map_snapshot_revision;
    d.check_duration_s=seconds(v.check_end)-seconds(v.check_begin);
    const auto source_age=[now](const builtin_interfaces::msg::Time& source) {
      return timeNs(source)>0?now-seconds(source):std::numeric_limits<double>::quiet_NaN();
    };
    d.body_age_s=source_age(v.body_source_stamp);
    d.front_age_s=source_age(v.front_ray_source_stamp);
    d.rear_age_s=source_age(v.rear_ray_source_stamp);
    if(!v.valid)d.failure="native_negative";
    else if(!d.persistent_state)d.failure="persistent_collision_state_required";
    else if(!finite(d.age_s)||d.age_s<-.02)d.failure="proof_time_invalid";
    else if(v.remaining_curve&&!v.whole_curve&&!core_.remainingProofCovers(
        v.checked_from_time,v.valid_start_time,v.valid_start_arc_length,
        v.checked_to_time,v.curve_duration,v.reverse_margin_m))d.failure="remaining_curve_coverage";
    else if(!collisionStateReady(v,now,false))d.failure="proof_contract_mismatch";
    return d;
  }
  std::uint64_t commitSequence()const{return commit_sequence_;}
  const std::optional<wire::ExecutionVersion>& activeVersion()const{return active_;}
  bool proposal(const wire::ReferenceProposal& p,double now) {
    if(!versionValid(p.version,config_)||p.transport_mode!=mode_||p.proposal_id.empty()||
       p.source_stamp.sec<0||p.source_stamp.nanosec>=1000000000U||
       p.valid_until.sec<0||p.valid_until.nanosec>=1000000000U||
       timeNs(p.source_stamp)<=0||seconds(p.source_stamp)>now+.02||
       timeNs(p.valid_until)<=timeNs(p.source_stamp)||
       timeNs(p.valid_until)-timeNs(p.source_stamp)>1000000000LL||
       p.reference.path.header.frame_id!=config_.planning_frame||p.reference.point_reference!="body_center"||
       p.version.reference_generation<=retired_generation_)return false;
    if(domain_&&!identity(p.version).sameTask(identity(*domain_))) {
      if(!retired_)return false;
      domain_.reset();active_.reset();active_id_=-1;lease_.reset();last_permit_.reset();active_proposal_.reset();lease_sequence_=0;
      ever_authorized_=false;
      resetWriterFeedback();
    }
    if(retired_task_==p.version.task_id)return false;
    if(!domain_){domain_=p.version;retired_=false;}
    for(const auto& old:proposals_)if(old.version==p.version&&old.proposal_id==p.proposal_id)return old==p;
    proposals_.push_back(p);while(proposals_.size()>8)proposals_.pop_front();return true;
  }
  bool support(const wire::SupportReference& s) {
    if(!versionValid(s.version,config_)||s.support_ground_xyz.size()>20000||
       !d1max_planning_interfaces::supportGeometryAvailable(s)||
       s.version.reference_generation<=retired_generation_||
       !mayStage(s.version))return false;
    for(auto& old:supports_)if(old.version==s.version) {
      // Immutable support geometry under a version, not source-time renewal.
      if(old.support_hash!=s.support_hash||old.support_ground_xyz!=s.support_ground_xyz||
         old.stair_model_hash!=s.stair_model_hash||old.stair_model_surfaces!=s.stair_model_surfaces||
         old.stair_model_obstacles!=s.stair_model_obstacles)return false;
      return true;
    }
    supports_.push_back(s);while(supports_.size()>8)supports_.pop_front();schedule();return true;
  }
  bool candidate(const wire::TaggedBspline& tag) {
    const auto v=curveVersion(tag);
    if(tag.schema_version!=2||!versionValid(v,config_)||tag.point_reference!="body_center"||
       tag.frame_id!=config_.planning_frame||v.reference_generation<=retired_generation_||
       !mayStage(v)||
       tag.trajectory.pos_pts.size()>10000)return false;
    if(active_&&v==*active_&&tag.trajectory.traj_id<=core_.trajectoryId())return false;
    if(exchange_&&v==exchange_->candidate.version&&tag.trajectory.traj_id<=exchange_->candidate.curve.id)
      return tag==exchange_->candidate.tag;
    if(pending_&&v.reference_generation<pending_->version.reference_generation)return false;
    if(pending_&&v==pending_->version&&tag.trajectory.traj_id<=pending_->curve.id) {
      return tag==pending_->tag; // duplicate cannot change immutable geometry
    }
    pending_=Pending{v,tag,decodeLocalCurve(tag),{},0};if(worker_)worker_->invalidate();
    candidate_reason_="preparing_candidate";schedule();return true;
  }
  bool validation(const wire::TrajectoryValidation& v) {
    if(!versionValid(v.version,config_)||v.transport_mode!=mode_||v.sequence==0||
       v.version.reference_generation<=retired_generation_||
       !mayStage(v.version))return false;
    if(exchange_&&v.version==exchange_->candidate.version&&v.trajectory_id==exchange_->candidate.curve.id&&
       (!exchange_->proof||v.sequence>exchange_->proof->sequence))exchange_->proof=v;
    if(active_&&v.version==*active_&&v.trajectory_id==core_.trajectoryId()) {
      if(active_proof_&&v.sequence<=active_proof_->sequence)return false;
      active_proof_=v;return true;
    }
    for(auto& old:proofs_)if(old.version==v.version&&old.trajectory_id==v.trajectory_id) {
      if(v.sequence<=old.sequence)return false;
      old=v;return true;
    }
    proofs_.push_back(v);while(proofs_.size()>8)proofs_.pop_front();return true;
  }
  bool lease(const wire::TaskExecutionLease& p,double now) {
    if(p.schema_version!=1||p.version.schema_version!=3||p.version.session_id!=config_.session_id||
       p.version.map_version_id!=config_.map_version_id||p.transport_mode!=mode_||!domain_||
       !identity(p.version).sameTask(identity(*domain_))||p.sequence<=lease_sequence_||
       !timed(now,seconds(p.source_stamp),seconds(p.valid_until),.75))return false;
    if(p.allowed&&(p.execution_id.empty()||p.control_epoch==0||p.confirmation_id.empty()||
       p.sdk_session.empty()||p.sdk_arm_generation==0))return false;
    if(lease_&&!lease_->execution_id.empty()) {
      const bool awaiting_sdk=!lease_->allowed&&lease_->sdk_arm_generation==0&&
        (lease_->sdk_session.empty()||lease_->sdk_session==p.sdk_session);
      if(p.execution_id!=lease_->execution_id||p.control_epoch!=lease_->control_epoch||
         (!awaiting_sdk&&(p.sdk_session!=lease_->sdk_session||p.sdk_arm_generation!=lease_->sdk_arm_generation)))return false;
    }
    lease_=p;lease_sequence_=p.sequence;
    ever_authorized_|=p.allowed&&!p.revoked;
    if(p.revoked)retire("task_execution_revoked");
    return true;
  }
  bool body(const Odom& o,double now,double receipt) {
    const bool was_active=core_.active();
    const bool accepted=core_.receiveOdom(o,now,receipt);
    if(was_active&&!core_.active()){retire(core_.reason());return accepted;}
    if(domain_&&o.schema_version==2&&o.session_id==config_.session_id&&o.map_version_id==config_.map_version_id&&
       o.localization_epoch>0&&o.stamp<=now+.02&&o.stamp>=core_.odometry().stamp&&
       (o.localization_epoch!=domain_->localization_epoch||o.localization_seed_id!=domain_->localization_seed_id))
      retire("localization_identity_changed");
    return accepted;
  }
  // Sending facts are not collision authority or physical stop evidence.
  // Match the execution binding, not the replaceable curve/anchor version:
  // a delayed sticky zero must survive a normal same-task replan.
  bool commandFeedback(const wire::TaskCommandFeedback& f,double now) {
    ++feedback_diagnostic_.received;
    const auto canonical=[](const auto& t){return t.sec>=0&&t.nanosec<1000000000U;};
    if(retired_||!domain_||!lease_||lease_->revoked||f.schema_version!=1||
       f.version.schema_version!=3||f.version.session_id!=config_.session_id||
       !identity(f.version).sameTask(identity(*domain_))||f.transport_mode!=mode_||
       f.execution_id!=lease_->execution_id||f.control_epoch!=lease_->control_epoch||
       f.sdk_session!=lease_->sdk_session||f.sdk_arm_generation!=lease_->sdk_arm_generation||
       f.execution_id.empty()||f.control_epoch==0||f.sdk_session.empty()||f.sdk_arm_generation==0||
       f.write_sequence<=feedback_write_sequence_||!canonical(f.source_stamp)||
       timeNs(f.source_stamp)<=0||!finite(now)||seconds(f.source_stamp)>now+.02||
       f.discontinuity_write_sequence>f.write_sequence||
       f.discontinuity_write_sequence<pending_discontinuity_write_){++feedback_diagnostic_.rejected;return false;}
    if(f.discontinuity_write_sequence>0&&
       (!canonical(f.discontinuity_stamp)||timeNs(f.discontinuity_stamp)<=0||
        timeNs(f.discontinuity_stamp)>timeNs(f.source_stamp)||
        f.interrupted_command_sequence==0)){++feedback_diagnostic_.rejected;return false;}
    feedback_write_sequence_=f.write_sequence;
    if(f.discontinuity_write_sequence>pending_discontinuity_write_) {
      ++feedback_diagnostic_.gaps;
      const bool already_resumed=!writer_recovering_&&
        last_fresh_command_ns_>timeNs(f.discontinuity_stamp);
      const bool precedes_authority_barrier=timeNs(f.discontinuity_stamp)<=authority_barrier_ns_;
      // A latest-only fact may already describe successful recovery. Do not
      // replay its old zero into the present controller or wait on an ACK.
      // Likewise a delayed zero cannot rewind a newer accepted-state control
      // boundary that has already resumed independently of feedback delivery.
      if(!precedes_authority_barrier&&!already_resumed&&!(f.write_submitted&&f.sdk_nonzero&&f.command_selected)) {
        beginOutputInterruption();
        for(const auto& command:recovery_commands_)if(command.sequence==f.interrupted_command_sequence&&
            command.source_stamp==f.interrupted_command_source_stamp) {
          interruption_history_=command.history;break;
        }
      }
    }
    pending_discontinuity_write_=f.discontinuity_write_sequence;
    if(f.write_submitted&&!f.sdk_nonzero) {
      // A later actual zero supersedes a cached nonzero even if both facts
      // arrive between control ticks, with or without a new sticky gap. Each
      // newly ordered zero is a real unsent boundary; a persistent positive
      // collision state must not accumulate command history through it.
      // The write-sequence floor above rejects a late older zero after an
      // observed newer nonzero. Missing feedback alone never enters here.
      if(timeNs(f.source_stamp)>authority_barrier_ns_)beginOutputInterruption();
      submitted_recovery_.reset();submitted_recovery_velocity_.reset();submitted_recovery_deadline_ns_=0;
    }
    const auto& v=f.velocity;
    if(writer_recovering_&&f.write_submitted&&f.sdk_nonzero&&f.command_selected&&
       f.command_sequence>interruption_command_floor_&&f.command_sequence<=command_sequence_&&
       canonical(f.command_source_stamp)&&timeNs(f.command_source_stamp)>interruption_source_floor_ns_&&
       timeNs(f.command_source_stamp)<=timeNs(f.source_stamp)&&
       finite(v.linear.x)&&finite(v.linear.y)&&finite(v.angular.z)&&v.linear.z==0.&&
       v.angular.x==0.&&v.angular.y==0.&&std::hypot(v.linear.x,v.linear.y)<=config_.max_speed+1e-12&&
       std::abs(v.angular.z)<=config_.max_yaw_rate+1e-12) {
      // Optional sending observation closes a waiting history transaction;
      // current fresh Native evidence can resume independently of its arrival.
      submitted_recovery_.reset();submitted_recovery_velocity_=v;
      submitted_recovery_deadline_ns_=canonical(f.command_valid_until)?timeNs(f.command_valid_until):0;
      for(const auto& c:recovery_commands_)if(c.sequence==f.command_sequence&&
          c.source_stamp==f.command_source_stamp&&c.velocity==f.velocity) {
        submitted_recovery_=c.history;break;
      }
    }
    feedback_diagnostic_.write_sequence=f.write_sequence;
    feedback_diagnostic_.discontinuity=f.discontinuity_write_sequence;
    return true;
  }
  struct FeedbackDiagnostic {
    std::uint64_t received{0},rejected{0},gaps{0},rebases{0},write_sequence{0},discontinuity{0};
    std::int64_t body_source_ns{0};double before_forward{0.},after_forward{0.};
  };
  const FeedbackDiagnostic& feedbackDiagnostic()const{return feedback_diagnostic_;}
  bool retireRoute(const wire::RouteDeliveryReceipt& r,double now) {
    if(r.schema_version!=1||r.active||r.status!=wire::RouteDeliveryReceipt::RETIRED||
       r.session_id!=config_.session_id||r.task_id.empty()||r.route_id.empty()||r.delivery_sequence==0||
       r.route_hash.size()!=64||!std::all_of(r.route_hash.begin(),r.route_hash.end(),[](char c){
         return (c>='0'&&c<='9')||(c>='a'&&c<='f');})||timeNs(r.source_stamp)<=0||
       timeNs(r.receipt_stamp)<=0||seconds(r.source_stamp)>now+.02||seconds(r.receipt_stamp)>now+.02||
       timeNs(r.receipt_stamp)+100000000LL<timeNs(r.source_stamp))return false;
    if(domain_&&(r.task_id!=domain_->task_id||r.route_id!=domain_->route_id||r.route_hash!=domain_->route_hash))return false;
    if(r.native_retirement_required) {
      if(!r.native_retirement_confirmed||r.native_generation==0||
         (domain_&&r.native_generation<=domain_->reference_generation))return false;
    }else if(!domain_||r.native_retirement_confirmed||r.native_generation!=0)return false;
    retired_generation_=std::max(retired_generation_,r.native_generation);retire("route_retired");return true;
  }
  void retire(const std::string& why) {
    if(domain_)retired_task_=domain_->task_id;
    core_.cancel(why);pending_.reset();exchange_.reset();if(worker_)worker_->invalidate();
    proposals_.clear();supports_.clear();proofs_.clear();active_proof_.reset();
    resetWriterFeedback();
    retired_=true;candidate_reason_=why;
  }
  // Called on the control lane. No support index/spline builds run here.
  std::optional<wire::LocalTrajectoryCommit> tryCommit(double now,double receipt,
      builtin_interfaces::msg::Time event_time={}) {
    core_.refreshMeasuredProgress(now);
    poll();if(retired_||!pending_||!pending_->curve.prepared)return {};
    if(!pending_->curve.prepared->failure.empty()) {
      candidate_reason_=pending_->curve.prepared->failure;return {};
    }
    const auto& p=*pending_;const auto* proof=proofFor(p.version,p.curve.id);
    const auto* proposal=proposalFor(p.version,proof?proof->proposal_id:"");
    const auto* support=supportFor(p.version);
    if(!candidateProofReady(proof,now))return {};
    if(!proposal){candidate_reason_="waiting_candidate_proposal";return {};}
    if(!support){candidate_reason_="waiting_candidate_support";return {};}
    if(proof->support_reference_id!=support->support_reference_id||proof->support_hash!=support->support_hash) {
      candidate_reason_="candidate_support_identity_mismatch";return {};
    }
    const auto& expected=proposal->expected_version;
    // Prepared predecessor is a hint. The commit below carries the actual
    // incumbent CAS, and the Native splice proof binds that same incumbent.
    // Another accepted same-reference refresh cannot strand a prepared window.
    if(active_&&!identity(p.version).sameTask(identity(*active_))) {
      candidate_reason_="candidate_task_mismatch";return {};
    }
    if(!active_&&proposal->expected_trajectory_id>0) {
      candidate_reason_="candidate_incumbent_missing";return {};
    }
    if(p.tag.splice.enabled) {
      const auto& splice=p.tag.splice;
      if(!active_||splice.incumbent_version!=*active_||splice.incumbent_trajectory_id!=core_.trajectoryId()||
         proof->splice!=splice||!finite(splice.splice_curve_time)||!finite(splice.splice_arc_length)) {
        candidate_reason_="candidate_splice_contract_mismatch";return {};
      }
      if(!core_.progress(now).valid) {
        candidate_reason_="waiting_measured_splice_progress";return {};
      }
      // Progress is XYZ/current-curve based. Wall clock never switches a tail.
      const double ahead=splice.splice_arc_length-core_.progressArc();
      if(ahead>config_.join_limit) {candidate_reason_="waiting_measured_splice_progress";return {};}
      if(ahead<-.05) {candidate_reason_="splice_missed_reseed_needed";return {};}
    }
    std::string why;
    auto observed=p.curve.prepared->entry.observe(p.curve,core_.odometry(),config_,now,
      seconds(proof->body_source_stamp),proof->valid_start_time,proof->valid_start_arc_length,why);
    if(!observed){candidate_reason_=why;return {};}
    // A replacement can be the way OUT of a negative incumbent verdict.
    // Reconcile its entry against the actual interrupted-motion boundary,
    // before the curvature/braking admission reads last_output_. Waiting for
    // the old curve to become positive first deadlocks a stopped robot behind
    // its pre-stop command. A zero demand alone is not a measured stop, and a
    // repeated pre-interruption body sample cannot establish a new boundary.
    if(writer_recovering_&&timeNs(bodySourceStamp(core_.odometry()))>interruption_source_floor_ns_)
      reconcileInterruptedCommand(now,timeNs(eventStamp(now,event_time)));
    Task task;task.session_id=p.version.session_id;task.frame_id=config_.planning_frame;
    task.generation=p.version.reference_generation;task.identity=identity(p.version);task.active=true;
    const auto& goal=proposal->goal_position;task.goal={goal.x,goal.y,goal.z};
    task.issued_at=seconds(proposal->source_stamp);
    if(!core_.admitRevision(task,*observed,*p.curve.prepared->support,now,receipt,true,true)) {
      candidate_reason_=core_.candidateReason();return {};
    }
    wire::LocalTrajectoryCommit c;c.schema_version=1;c.version=p.version;c.trajectory_id=p.curve.id;
    c.expected_version=active_.value_or(expected);c.expected_trajectory_id=active_?active_id_:-1;
    c.proposal_id=proof->proposal_id;c.commit_sequence=++commit_sequence_;c.source_stamp=eventStamp(now,event_time);
    c.body_source_stamp=bodySourceStamp(core_.odometry());fillBody(c);
    c.curve_time=core_.progressTime();c.arc_length=core_.progressArc();
    c.native_validation_sequence=proof->sequence;c.map_geometry_revision=proof->map_snapshot_revision;
    c.accepted=true;c.disposition="committed";c.reason="local_geometry_committed";c.transport_mode=mode_;
    active_=p.version;active_id_=p.curve.id;active_proof_=*proof;active_proposal_=*proposal;
    pending_.reset();candidate_reason_.clear();retired_=false;return c;
  }
  // Production transport: reserve candidate geometry before entering a moving
  // splice. The old controller continues while messages cross the process
  // boundary. Only the normal latest-body admission below installs geometry.
  std::optional<wire::LocalTrajectoryCommit> exchangeCommit(double now,double receipt,
      builtin_interfaces::msg::Time event_time={},ExchangeClock clock=ExchangeClock::ControlTick) {
    poll();if(retired_)return {};
    if(exchange_) {
      auto& x=*exchange_;
      if(x.phase==CommitPhase::AwaitCommitted)return x.commit;
      if(x.phase==CommitPhase::AwaitPrepared) {
        if(now-seconds(x.request.source_stamp)<=.4)return x.request;
        return abortExchange("native_candidate_prepare_expired");
      }
      if(x.awaiting_entry_refresh&&now-seconds(x.request.source_stamp)>.4)
        return abortExchange("native_candidate_entry_refresh_expired");
      if(!candidateProofReady(x.proof?&*x.proof:nullptr,now)) {
        // Native publishes completed collision states only. Incomplete map
        // work does not replace one; a newer negative or malformed state is
        // not made positive by retaining an older notification.
        return abortExchange(candidate_reason_);
      }
      // Input/ACK callbacks may reserve and refresh geometry immediately, but
      // must not retire an in-flight incumbent command between control ticks.
      // Observe the replacement's measured join on the control lane, then
      // stage it until Native acknowledges that exact geometry. No curve or
      // source is renamed; the live incumbent still owns this turn's demand.
      if(clock==ExchangeClock::InputDelivery)return {};
      core_.refreshMeasuredProgress(now);
      if(x.candidate.tag.splice.enabled) {
        const auto& splice=x.candidate.tag.splice;
        if(!active_||splice.incumbent_version!=*active_||splice.incumbent_trajectory_id!=active_id_)
          return abortExchange("reserved_splice_incumbent_changed");
        if(splice.splice_arc_length-core_.progressArc()>config_.join_limit)return {};
      }
      // Preserve any newer candidate being prepared. It cannot overtake this
      // transaction, and remains available after its final acknowledgement.
      auto newer=std::move(pending_);pending_=x.candidate;
      auto incumbent=core_;const auto old_version=active_;const auto old_proposal=active_proposal_;
      const auto old_proof=active_proof_;const auto old_id=active_id_;
      const auto old_commit_sequence=commit_sequence_;
      const auto accepted=tryCommit(now,receipt,event_time);
      pending_=std::move(newer);
      if(!accepted) {
        if(candidate_reason_=="entry_proof_body_gap_exceeded")return awaitReservedEntry(now);
        if(admissionCanAwaitInput(candidate_reason_))return {};
        // Renewed collision proofs cannot repair an inadmissible immutable
        // curve. Release only this reservation so an already prepared newer
        // candidate can run; keep the incumbent and all controller history.
        // No new timeout, relaxed admission predicate or task cancellation.
        return abortExchange(candidate_reason_,now,event_time);
      }
      // Admission prepares immutable geometry; it is not yet Native's commit.
      // Keep producing fresh incumbent demands until the matching final ACK.
      // Rejected/late transactions must never make us command an uncommitted
      // curve for one tick and rely on rollback to recover afterward.
      x.accepted_core=core_;
      // tryCommit may have reconciled a real interruption against a newer
      // measured boundary. That is LIVE command state, not candidate geometry;
      // restoring the incumbent while awaiting ACK must not undo it.
      if(writer_recovering_)incumbent.reconcileSubmittedCommand(core_.commandHistory());
      if(old_version)core_=incumbent;
      else core_.rollbackGeometry(incumbent,now); // retain first task, no geometry/output
      active_=old_version;active_id_=old_id;
      active_proposal_=old_proposal;active_proof_=old_proof;
      x.commit=*accepted;x.commit.commit_sequence=x.request.commit_sequence;
      commit_sequence_=old_commit_sequence;x.phase=CommitPhase::AwaitCommitted;
      return x.commit;
    }
    if(pending_&&active_&&pending_->version==*active_&&pending_->curve.id<=active_id_)pending_.reset();
    if(!pending_||!pending_->curve.prepared)return {};
    if(!pending_->curve.prepared->failure.empty()) {
      candidate_reason_=pending_->curve.prepared->failure;return {};
    }
    const auto* proof=proofFor(pending_->version,pending_->curve.id);
    const auto* proposal=proposalFor(pending_->version,proof?proof->proposal_id:"");
    const auto* support=supportFor(pending_->version);
    if(!candidateProofReady(proof,now))return {};
    if(!proposal){candidate_reason_="waiting_candidate_proposal";return {};}
    if(!support){candidate_reason_="waiting_candidate_support";return {};}
    wire::LocalTrajectoryCommit request;request.schema_version=1;request.version=pending_->version;
    request.trajectory_id=pending_->curve.id;request.expected_version=active_.value_or(proposal->expected_version);
    request.expected_trajectory_id=active_?active_id_:-1;request.proposal_id=proposal->proposal_id;
    exchange_sequence_=std::max(exchange_sequence_,commit_sequence_)+1;request.commit_sequence=exchange_sequence_;
    request.source_stamp=eventStamp(now,event_time);request.body_source_stamp=bodySourceStamp(core_.odometry());fillBody(request);
    request.native_validation_sequence=proof->sequence;request.map_geometry_revision=proof->map_snapshot_revision;
    request.disposition="prepare";request.reason="reserve_local_candidate";request.transport_mode=mode_;
    exchange_.emplace(CommitExchange{std::move(*pending_),*proposal,*support,*proof,request});pending_.reset();
    candidate_reason_="waiting_native_candidate_reservation";return request;
  }
  bool commitResult(const wire::LocalTrajectoryCommit& result,double now,double receipt) {
    (void)receipt; // ACK delivery never refreshes an observation or control clock.
    if(!exchange_||retired_)return false;
    auto& x=*exchange_;const auto& original=x.phase==CommitPhase::AwaitCommitted?x.commit:x.request;
    auto normalized=result;normalized.accepted=original.accepted;normalized.disposition=original.disposition;
    normalized.reason=original.reason;if(normalized!=original)return false;
    if(!result.accepted&&result.disposition=="rejected") {
      candidate_reason_=result.reason;
      // Nothing to roll back: the incumbent, its newest verdict and all live
      // controller state remained authoritative while this request travelled.
      exchange_.reset();return true;
    }
    if(result.accepted&&result.disposition=="prepared"&&x.phase==CommitPhase::AwaitPrepared) {
      x.phase=CommitPhase::Prepared;candidate_reason_="native_candidate_reserved";return true;
    }
    if(result.accepted&&result.disposition=="committed"&&x.phase==CommitPhase::AwaitCommitted) {
      // Install only geometry, preserving the live odometry, controller clock,
      // limiter and hold state advanced while the ACK crossed processes.
      // The very first admission has no running incumbent yet; its staged
      // core owns the task, while all current observations still stay live.
      if(!x.accepted_core||!core_.rollbackGeometry(*x.accepted_core,now,true)) {
        retire("commit_activation_context_changed");return true;
      }
      active_=x.candidate.version;active_id_=x.candidate.curve.id;
      active_proposal_=x.proposal;active_proof_=x.proof;
      commit_sequence_=x.commit.commit_sequence;
      exchange_.reset();candidate_reason_.clear();return true;
    }
    return false;
  }
  wire::ExecutionPermit geometryPermit(double now,builtin_interfaces::msg::Time event_time={}) {
    wire::ExecutionPermit p;p.sequence=++permit_sequence_;p.source_stamp=eventStamp(now,event_time);
    p.valid_until=stampNs(timeNs(p.source_stamp)+350000000LL);p.frame_id=config_.planning_frame;
    p.transport_mode=mode_;p.geometry_committed=active_.has_value();p.trajectory_id=active_id_;
    if(active_)p.version=*active_;else if(domain_)p.version=*domain_;
    if(active_proof_)p.validation_sequence=active_proof_->sequence;
    if(active_proposal_) {
      p.goal_position=active_proposal_->goal_position;p.has_goal_yaw=active_proposal_->has_goal_yaw;
      p.goal_yaw=active_proposal_->goal_yaw;p.goal_yaw_tolerance_rad=active_proposal_->goal_yaw_tolerance_rad;
    }
    if(lease_) {
      p.execution_id=lease_->execution_id;p.control_epoch=lease_->control_epoch;p.confirmation_id=lease_->confirmation_id;
      p.sdk_session=lease_->sdk_session;p.sdk_arm_generation=lease_->sdk_arm_generation;
      p.valid_until=originalTimeCap(p.valid_until,{lease_->valid_until});
      p.allowed=taskAuthorized(now);
      p.revoked=lease_->revoked;p.phase=lease_->phase;
    }else p.phase="preview";
    // Permit phases are the existing Native command contract; settling and
    // replan waits are status diagnostics, while their zero/braking demand
    // remains admissible under the task's tracking lease.
    if(p.allowed&&sourceEvidenceFailure(core_.odometry(),now,config_)==nullptr&&canAlignGoal()&&!atGoalPose()&&
       goalYawProofReady(now))
      p.phase="aligning";
    p.reason=p.allowed?"local_task_authorized":"local_geometry_preview";last_permit_=p;return p;
  }
  wire::MotionDemand step(double now,double receipt,builtin_interfaces::msg::Time event_time={}) {
    if(pending_discontinuity_write_>acknowledged_discontinuity_write_) {
      acknowledged_discontinuity_write_=pending_discontinuity_write_;
    }
    wire::MotionDemand d;d.source_stamp=eventStamp(now,event_time);d.body_source_stamp=bodySourceStamp(core_.odometry());
    d.acknowledged_discontinuity_write=acknowledged_discontinuity_write_;
    d.sequence=++command_sequence_;d.transport_mode=mode_;d.hold=true;d.reason="waiting_local_trajectory";
    d.valid_until=stampNs(timeNs(d.source_stamp)+200000000LL);
    if(!active_||!last_permit_)return d;
    const auto& p=*last_permit_;d.version=*active_;d.trajectory_id=active_id_;d.permit_sequence=p.sequence;
    d.execution_id=p.execution_id;d.control_epoch=p.control_epoch;d.sdk_session=p.sdk_session;
    d.sdk_arm_generation=p.sdk_arm_generation;d.braking_model_sha256=braking_hash_;
    d.validation_sequence=active_proof_?active_proof_->sequence:0;
    d.valid_until=originalTimeCap(d.valid_until,{p.valid_until});
    if(core_.odometry().provider_contract)d.valid_until=originalTimeCap(d.valid_until,
      {stampNs(core_.odometry().provider_valid_until_ns)});
    core_.refreshTaskLease(receipt);
    core_.refreshMeasuredProgress(now);
    // The latest completed SCAN state is not a timed motion lease. A delayed
    // notification or an in-progress map update cannot create a fictitious
    // writer interruption. Real negative states and actual sender gaps below
    // retain their existing measured-continuity recovery.
    const bool geometry=currentGeometryReady(now);
    if(!p.allowed||p.revoked||seconds(p.valid_until)<=now||!geometry) {
      d.reason=!geometry?"local_collision_state_unavailable":"waiting_task_authority";
      if(!geometry&&p.allowed&&!p.revoked&&seconds(p.valid_until)>now&&core_.needsTerminalReplan(now))
        d.reason=core_.reason()=="braking_envelope_reentry_required"?
          "braking_envelope_reentry_required":"local_segment_finished_waiting_replan";
      if(p.allowed&&!p.revoked&&seconds(p.valid_until)>now)suspendInterruptedOutput(now,receipt,d.reason);
      else {
        // Explicit authority loss is not a transient map gap. Its zero
        // boundary cannot be replaced by delayed pre-HOLD sending history.
        authority_barrier_ns_=std::max(authority_barrier_ns_,timeNs(d.source_stamp));
        last_fresh_command_ns_=0;recovery_commands_.clear();clearOutputInterruption();
        core_.suspendOutput(now,receipt,d.reason,false);
      }
      return d;
    }
    if(needsGoalYawProof()&&!goalYawProofReady(now)) {
      // SCAN already checks this explicit final rotation. Consume its current
      // typed verdict instead of treating a straight-path proof as proof of an
      // arbitrary terminal heading. Keep task/curve and actual-source progress:
      // the independent Native reader can renew the same proof while we wait.
      d.reason="waiting_native_goal_yaw_proof";
      suspendInterruptedOutput(now,receipt,d.reason);return d;
    }
    if(writer_recovering_) {
      reconcileInterruptedCommand(now,timeNs(d.source_stamp));
      // Current completed geometry permits calculating intent, not a claim
      // it was submitted. Native independently gates real runtime health.
      clearOutputInterruption();
    }
    auto out=canAlignGoal()?
      core_.align(active_proposal_->goal_yaw,active_proposal_->goal_yaw_tolerance_rad,now,receipt,true):
      core_.step(now,receipt,true);
    d.velocity.linear.x=out.forward;d.velocity.linear.y=out.lateral;d.velocity.angular.z=out.yaw_rate;
    d.hold=out.forward==0.&&out.lateral==0.&&out.yaw_rate==0.;d.reason=out.reason;
    if(!d.hold) {
      // Record the evidence used by THIS step, not a positive callback that
      // might arrive later beside a delayed zero notification.
      last_fresh_command_ns_=timeNs(d.source_stamp);
      recovery_commands_.push_back({d.sequence,d.source_stamp,d.velocity,core_.commandHistory()});
      while(recovery_commands_.size()>8)recovery_commands_.pop_front();
    }
    return d; // Native binds this command to the current accepted SCAN curve; no second sweep.
  }
  wire::LocalExecutionStatus status(double now,builtin_interfaces::msg::Time event_time={}) {
    wire::LocalExecutionStatus s;s.schema_version=1;s.sequence=++status_sequence_;s.source_stamp=eventStamp(now,event_time);
    s.version=active_.value_or(domain_.value_or(wire::ExecutionVersion{}));
    s.body_source_stamp=bodySourceStamp(core_.odometry());
    if(core_.odometry().provider_contract)s.body_valid_until=stampNs(core_.odometry().provider_valid_until_ns);
    s.trajectory_id=active_id_;s.commit_sequence=commit_sequence_;s.transport_mode=mode_;
    if(lease_){s.execution_id=lease_->execution_id;s.control_epoch=lease_->control_epoch;}
    s.ready=currentGeometryReady(now)&&core_.hasInstalledGeometry()&&
      sourceEvidenceFailure(core_.odometry(),now,config_)==nullptr;
    const bool authorized=taskAuthorized(now);
    // ready describes geometry/source readiness, not motion authority. Before
    // the first grant this must stay eligible for TaskExecutionAuthority's
    // initial admission (which requires !holding), without claiming motion.
    // After a grant, an expired/withheld lease is a real execution hold, even
    // though suspendOutput(false) deliberately preserves controller recovery.
    const bool waiting_authority=ever_authorized_&&!authorized;
    const bool pose_at_goal=atGoalPose();
    s.at_goal=authorized&&s.ready&&!core_.holding()&&pose_at_goal&&measuredSettled();
    const bool waiting_replan=core_.reason()=="local_segment_finished_waiting_replan"||
      core_.reason()=="braking_envelope_reentry_required";
    const bool waiting_goal_yaw=needsGoalYawProof()&&!goalYawProofReady(now);
    s.request_stop=s.at_goal||retired_;s.software_retired=retired_;
    s.holding=!s.ready||core_.holding()||waiting_replan||waiting_goal_yaw||waiting_authority;
    s.phase=retired_?"retired":s.at_goal?"goal_reached":waiting_replan?"waiting_replan":
      waiting_goal_yaw?"waiting_goal_yaw_proof":s.ready?
      (waiting_authority?"waiting_task_authority":!authorized?"preview":core_.holding()?"holding":
        pose_at_goal?"settling":canAlignGoal()?"aligning":"following"):"waiting_local_trajectory";
    // Before the first admitted curve, report this preparation attempt rather
    // than a previous task's retained controller reason (e.g. route_retired).
    // A failed replacement never overwrites the active curve's running status.
    s.reason=s.phase=="waiting_task_authority"?"waiting_task_authority":s.phase=="preview"?"local_geometry_preview":
      !retired_&&!active_&&!candidate_reason_.empty()?candidate_reason_:core_.reason();return s;
  }
 private:
  friend struct LocalFollowerTestAccess;
  bool taskAuthorized(double now)const {
    return lease_&&lease_->allowed&&!lease_->revoked&&!retired_&&seconds(lease_->valid_until)>now;
  }
  static Config nativeScanConfig(Config c) {
    // Fixed by the production component, never exposed as a ROS/UI switch.
    // Native owns actual body-to-curve connector and obstacle validation.
    // Follow its geometry from measured progress; do not require measured
    // velocity to equal the spline derivative or repeat raw-map classification.
    // Version/source checks and the existing command conditioner still apply.
    c.require_support_reference=true;
    c.geometry_authority=GeometryAuthority::NativeScan;
    return c;
  }
  bool mayStage(const wire::ExecutionVersion& v)const {
    // Reference publishes support before proposal. Across topics, a candidate
    // or proof may also arrive first. After explicit retirement, cache bounded
    // immutable inputs for the next task; only its valid proposal can bind the
    // new domain. Retired-task packets can never refill those caches.
    return v.task_id!=retired_task_&&
      (!domain_||retired_||identity(v).sameTask(identity(*domain_)));
  }
  static builtin_interfaces::msg::Time eventStamp(double now,const builtin_interfaces::msg::Time& original) {
    // Event time is not a sensor observation; preserve the host clock's exact
    // integer transport when provided, rather than rounding UNIX seconds twice.
    return original.sec>=0&&original.nanosec<1000000000U&&timeNs(original)>0?original:stamp(now);
  }
  struct Preparation {Config config;Trajectory curve;wire::SupportReference support;};
  using Worker=BoundedPreparationWorker<Preparation,std::shared_ptr<const PreparedGeometry>>;
  struct Pending {wire::ExecutionVersion version;wire::TaggedBspline tag;Trajectory curve;
    std::string support_hash;std::uint64_t token;};
  enum class CommitPhase {AwaitPrepared,Prepared,AwaitCommitted};
  struct CommitExchange {
    CommitExchange(Pending c,wire::ReferenceProposal p,wire::SupportReference s,
        wire::TrajectoryValidation v,wire::LocalTrajectoryCommit r):
        candidate(std::move(c)),proposal(std::move(p)),support(std::move(s)),
        proof(std::move(v)),request(std::move(r)){}
    Pending candidate;wire::ReferenceProposal proposal;wire::SupportReference support;
    std::optional<wire::TrajectoryValidation> proof;wire::LocalTrajectoryCommit request;
    CommitPhase phase{CommitPhase::AwaitPrepared};wire::LocalTrajectoryCommit commit;
    bool awaiting_entry_refresh{false};
    std::optional<TrackerCore> accepted_core;
  };
  static bool admissionCanAwaitInput(const std::string& reason) {
    // These failures concern replaceable observations or a not-yet-reached
    // splice, rather than the reserved curve. Fresh delivery may repair them
    // without rebuilding it; existing proof and task deadlines still apply.
    return reason=="waiting_candidate_proposal"||reason=="waiting_candidate_support"||
      reason=="waiting_measured_splice_progress"||reason=="entry_body_not_fresh"||
      reason=="entry_body_precedes_or_exceeds_proof"||reason=="entry_body_precedes_candidate_source"||
      reason=="join_body_missing"||reason=="join_body_not_fresh"||
      reason=="join_source_in_future"||reason=="join_source_expired"||reason=="join_source_after_body"||
      reason.rfind("local_state_",0)==0||reason.rfind("body_posterior_",0)==0||
      reason.rfind("body_imu_",0)==0||reason.rfind("body_extrapolation_",0)==0;
  }
  std::optional<wire::LocalTrajectoryCommit> abortExchange(const std::string& reason,
      double now=0.,builtin_interfaces::msg::Time event_time={}) {
    auto request=exchange_->request;request.disposition="abort";request.reason=reason;
    if(now>0.&&exchange_->proof) {
      request.source_stamp=eventStamp(now,event_time);
      request.body_source_stamp=bodySourceStamp(core_.odometry());fillBody(request);
      request.native_validation_sequence=exchange_->proof->sequence;
      request.map_geometry_revision=exchange_->proof->map_snapshot_revision;
    }
    exchange_.reset();candidate_reason_=reason;return request;
  }
  std::optional<wire::LocalTrajectoryCommit> awaitReservedEntry(double now) {
    // Reuse the original reservation's finite acquisition budget. Neither a
    // duplicate ACK nor another proof resets it. This releases an unrefreshable
    // candidate for the next solve without cancelling the task/accepted curve.
    if(now-seconds(exchange_->request.source_stamp)>.4)
      return abortExchange("native_candidate_entry_refresh_expired");
    exchange_->awaiting_entry_refresh=true;
    return exchange_->request;
  }
  void schedule() {
    if(!pending_)return;
    const auto* s=supportFor(pending_->version);if(!s)return;
    if(pending_->support_hash==s->support_hash)return;
    pending_->support_hash=s->support_hash;
    Preparation req{config_,pending_->curve,*s};
    if(worker_)pending_->token=worker_->submit(std::move(req));
    else pending_->curve.prepared=PreparedGeometry::build(config_,req.curve,supportEvidence(req.support));
  }
  void poll() {
    if(!worker_||!pending_)return;
    const auto done=worker_->take(pending_->token);if(!done)return;
    if(!done->failure.empty()||!done->result){candidate_reason_="candidate_preparation_failed";return;}
    pending_->curve.prepared=*done->result;
    if(!pending_->curve.prepared->failure.empty())candidate_reason_=pending_->curve.prepared->failure;
  }
  const wire::SupportReference* supportFor(const wire::ExecutionVersion& v)const {
    if(exchange_&&exchange_->candidate.version==v)return &exchange_->support;
    for(const auto& s:supports_)if(s.version==v)return &s;
    return nullptr;
  }
  const wire::ReferenceProposal* proposalFor(const wire::ExecutionVersion& v,const std::string& id)const {
    if(exchange_&&exchange_->candidate.version==v&&exchange_->proposal.proposal_id==id)return &exchange_->proposal;
    for(const auto& p:proposals_)if(p.version==v&&p.proposal_id==id)return &p;
    return nullptr;
  }
  const wire::TrajectoryValidation* proofFor(const wire::ExecutionVersion& v,std::int64_t id)const {
    if(exchange_&&exchange_->candidate.version==v&&exchange_->candidate.curve.id==id&&exchange_->proof)return &*exchange_->proof;
    for(const auto& p:proofs_)if(p.version==v&&p.trajectory_id==id)return &p;
    return nullptr;
  }
  bool candidateProofReady(const wire::TrajectoryValidation* v,double now) {
    // Preparation is finished; missing, negative or malformed completed SCAN
    // state must not still look like a busy geometry worker.
    if(v&&collisionStateReady(*v,now,true))return true;
    std::string detail="proof_contract_mismatch";
    if(!v)detail="missing";
    else if(!v->valid)detail="native_negative:"+(v->reason.empty()?std::string("unspecified"):v->reason);
    else if(v->valid_until.sec!=0||v->valid_until.nanosec!=0)detail="persistent_collision_state_required";
    else if(!v->whole_curve||v->remaining_curve||v->checked_from_time!=0.)detail="whole_curve_required";
    else {
      const double end=seconds(v->check_end);
      if(!finite(end)||end<=0.||end>now+.02)detail="proof_time_invalid";
    }
    candidate_reason_="waiting_current_native_proof:"+detail;return false;
  }
  // Formal task_follower_v1 consumes SCAN's latest completed collision state,
  // not the archived curve_writer_v3 short proof lease. Zero is an explicit
  // wire discriminator, never an infinite deadline. Original sample times
  // remain evidence; Native owns separate input/process health, while this
  // controller still enforces current measured-body and task authority.
  bool collisionStateReady(const wire::TrajectoryValidation& v,double now,bool entry)const {
    const double begin=seconds(v.check_begin),end=seconds(v.check_end);
    const auto canonical=[](const auto& t){return t.sec>=0&&t.nanosec<1000000000U;};
    const bool coverage=entry?(v.whole_curve&&!v.remaining_curve&&v.checked_from_time==0.):
      ((v.whole_curve&&!v.remaining_curve)||(v.remaining_curve&&!v.whole_curve&&
       core_.remainingProofCovers(v.checked_from_time,v.valid_start_time,v.valid_start_arc_length,
         v.checked_to_time,v.curve_duration,v.reverse_margin_m)));
    return v.valid&&coverage&&v.transport_mode==mode_&&v.frame_id==config_.planning_frame&&
      v.collision_policy==config_.trajectory_geometry_policy&&v.map_snapshot_revision>0&&v.sequence>0&&
      canonical(v.source_stamp)&&canonical(v.check_begin)&&canonical(v.check_end)&&
      finite(begin)&&finite(end)&&begin>0.&&end>=begin&&end<=now+.02&&
      v.valid_until.sec==0&&v.valid_until.nanosec==0&&
      seconds(v.source_stamp)>=end&&seconds(v.source_stamp)<=now+.02&&
      finite(v.checked_from_time)&&finite(v.checked_to_time)&&finite(v.curve_duration)&&
      finite(v.valid_start_time)&&finite(v.valid_start_arc_length)&&v.valid_start_arc_length>=0.&&
      v.checked_from_time<=v.valid_start_time&&v.valid_start_time<=v.checked_to_time&&
      v.curve_duration>0.&&v.curve_duration<=120.&&
      v.checked_from_time>=0.&&v.checked_to_time>v.checked_from_time&&
      std::abs(v.checked_to_time-v.curve_duration)<=1e-6;
  }
  bool currentGeometryReady(double now)const {
    if(!active_||!active_proof_||!active_proposal_||active_proof_->version!=*active_||
       active_proof_->trajectory_id!=active_id_||active_proof_->proposal_id!=active_proposal_->proposal_id)
      return false;
    return collisionStateReady(*active_proof_,now,false);
  }
  bool atGoalPosition()const {
    if(!active_proposal_)return false;
    const auto& p=active_proposal_->goal_position;
    const auto& actual=core_.odometry().position;
    return std::hypot(p.x-actual.x(),p.y-actual.y())<=config_.goal_tolerance&&
      std::abs(p.z-actual.z())<=config_.goal_height_tolerance;
  }
  bool atGoalPose()const {
    return atGoalPosition()&&(!active_proposal_->has_goal_yaw||
      std::abs(angle(active_proposal_->goal_yaw-core_.odometry().yaw))<=active_proposal_->goal_yaw_tolerance_rad);
  }
  bool measuredLinearSettled()const {
    const auto& o=core_.odometry();
    return std::max(o.planar_speed,o.velocity_in_frame.head<2>().norm())<=config_.stationary_linear_threshold_mps;
  }
  bool measuredSettled()const {
    return measuredLinearSettled()&&
      std::abs(core_.odometry().angular_velocity_in_frame.z())<=config_.stationary_angular_threshold_radps;
  }
  bool canAlignGoal()const {
    return active_proposal_&&active_proposal_->has_goal_yaw&&atGoalPosition()&&measuredLinearSettled();
  }
  bool needsGoalYawProof()const {return canAlignGoal()&&!atGoalPose();}
  bool goalYawProofReady(double now)const {
    // Match the immutable target modulo 2*pi using the existing execution
    // contract's target-identity tolerance. Actual completion keeps the route's
    // separate goal_yaw_tolerance_rad; no tolerance or map flag is enlarged.
    return active_&&active_proposal_&&active_proposal_->has_goal_yaw&&active_proof_&&
      active_proof_->version==*active_&&active_proof_->trajectory_id==active_id_&&
      active_proof_->proposal_id==active_proposal_->proposal_id&&collisionStateReady(*active_proof_,now,false)&&
      active_proof_->goal_yaw_checked&&finite(active_proof_->checked_goal_yaw)&&
      finite(active_proposal_->goal_yaw)&&finite(active_proposal_->goal_yaw_tolerance_rad)&&
      active_proposal_->goal_yaw_tolerance_rad>=.01&&active_proposal_->goal_yaw_tolerance_rad<=.5&&
      std::abs(angle(active_proof_->checked_goal_yaw-active_proposal_->goal_yaw))<=1e-6;
  }
  void fillBody(wire::LocalTrajectoryCommit& c)const {
    const auto& o=core_.odometry();c.measured_pose.position.x=o.position.x();c.measured_pose.position.y=o.position.y();
    c.measured_pose.position.z=o.position.z();c.measured_pose.orientation.x=o.orientation.x();
    c.measured_pose.orientation.y=o.orientation.y();c.measured_pose.orientation.z=o.orientation.z();c.measured_pose.orientation.w=o.orientation.w();
    c.measured_twist.linear.x=o.velocity_in_frame.x();c.measured_twist.linear.y=o.velocity_in_frame.y();
    c.measured_twist.linear.z=o.velocity_in_frame.z();c.measured_twist.angular.x=o.angular_velocity_in_frame.x();
    c.measured_twist.angular.y=o.angular_velocity_in_frame.y();c.measured_twist.angular.z=o.angular_velocity_in_frame.z();
  }
  void resetWriterFeedback() {
    feedback_write_sequence_=pending_discontinuity_write_=acknowledged_discontinuity_write_=0;
    clearOutputInterruption();recovery_commands_.clear();feedback_diagnostic_={};
    last_fresh_command_ns_=authority_barrier_ns_=0;
  }
  void beginOutputInterruption() {
    if(writer_recovering_)return;
    writer_recovering_=true;interruption_history_=core_.commandHistory();
    interruption_command_floor_=command_sequence_;
    interruption_source_floor_ns_=timeNs(bodySourceStamp(core_.odometry()));
    submitted_recovery_.reset();submitted_recovery_velocity_.reset();
    submitted_recovery_deadline_ns_=0;
  }
  void clearOutputInterruption() {
    writer_recovering_=false;interruption_history_.reset();
    submitted_recovery_.reset();submitted_recovery_velocity_.reset();submitted_recovery_deadline_ns_=0;
  }
  void reconcileInterruptedCommand(double now,std::int64_t source_ns) {
    if(!writer_recovering_)return;
    const bool submitted=submitted_recovery_.has_value()||submitted_recovery_velocity_.has_value();
    if(submitted_recovery_)core_.reconcileSubmittedCommand(*submitted_recovery_);
    else if(submitted_recovery_velocity_) {const auto& v=*submitted_recovery_velocity_;
      core_.reconcileSubmittedVelocity(v.linear.x,v.linear.y,v.angular.z);}
    else if(interruption_history_)core_.reconcileSubmittedCommand(*interruption_history_);
    const auto before=core_.commandHistory().output.forward;
    // A still-live, actually submitted nonzero boundary supports actuator
    // breakaway. Otherwise only fresh original-source measured motion may
    // reduce interrupted history; this grants neither geometry nor authority.
    if(!(submitted&&submitted_recovery_deadline_ns_>source_ns)&&core_.reconcileMeasuredContinuity(now)) {
      ++feedback_diagnostic_.rebases;feedback_diagnostic_.before_forward=before;
      feedback_diagnostic_.after_forward=core_.commandHistory().output.forward;
      feedback_diagnostic_.body_source_ns=timeNs(bodySourceStamp(core_.odometry()));
    }
    interruption_history_=core_.commandHistory();
  }
  void suspendInterruptedOutput(double now,double receipt,const std::string& reason) {
    beginOutputInterruption();core_.suspendOutput(now,receipt,reason,false);
    if(interruption_history_&&!core_.holding())core_.reconcileSubmittedCommand(*interruption_history_);
  }
  struct RecoveryCommand {
    std::uint64_t sequence;
    builtin_interfaces::msg::Time source_stamp;
    geometry_msgs::msg::Twist velocity;
    TrackerCore::CommandHistory history;
  };
  Config config_;std::string mode_,braking_hash_;TrackerCore core_;
  std::unique_ptr<Worker> worker_;std::optional<Pending> pending_;
  std::optional<CommitExchange> exchange_;std::uint64_t exchange_sequence_{0};
  std::optional<wire::ExecutionVersion> domain_,active_;
  std::optional<wire::ReferenceProposal> active_proposal_;
  std::optional<wire::TrajectoryValidation> active_proof_;
  std::optional<wire::TaskExecutionLease> lease_;std::optional<wire::ExecutionPermit> last_permit_;
  std::deque<wire::ReferenceProposal> proposals_;std::deque<wire::SupportReference> supports_;
  std::deque<wire::TrajectoryValidation> proofs_;
  std::int64_t active_id_{-1};std::uint64_t lease_sequence_{0},permit_sequence_{0},command_sequence_{0},
    status_sequence_{0},commit_sequence_{0},retired_generation_{0};
  std::uint64_t feedback_write_sequence_{0},pending_discontinuity_write_{0},acknowledged_discontinuity_write_{0};
  bool writer_recovering_{false};std::deque<RecoveryCommand> recovery_commands_;
  std::optional<TrackerCore::CommandHistory> submitted_recovery_;
  std::optional<geometry_msgs::msg::Twist> submitted_recovery_velocity_;
  std::int64_t submitted_recovery_deadline_ns_{0};
  std::optional<TrackerCore::CommandHistory> interruption_history_;
  std::uint64_t interruption_command_floor_{0};std::int64_t interruption_source_floor_ns_{0};
  std::int64_t last_fresh_command_ns_{0},authority_barrier_ns_{0};
  FeedbackDiagnostic feedback_diagnostic_;
  bool retired_{false},ever_authorized_{false};std::string retired_task_,candidate_reason_;
};
} // namespace d1max_trajectory_tracker
