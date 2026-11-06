#pragma once
#include <plan_env/collision_snapshot_pool.hpp>
#include <plan_manage/trajectory_collision.hpp>
#include <plan_manage/support_connection.hpp>
#include <plan_manage/motion_sweep.hpp>
#include <plan_manage/validation_cycle.hpp>
#include <d1max_planning_interfaces/msg/motion_demand.hpp>
#include <d1max_planning_interfaces/msg/motion_validation.hpp>
#include <d1max_planning_interfaces/msg/reference_proposal.hpp>
#include <d1max_planning_interfaces/msg/reference_receipt.hpp>
#include <d1max_planning_interfaces/msg/trajectory_validation.hpp>
#include <d1max_planning_interfaces/msg/execution_permit.hpp>
#include <d1max_planning_interfaces/msg/support_reference.hpp>
#include <d1max_planning_interfaces/msg/tagged_bspline.hpp>
#include <d1max_planning_interfaces/msg/local_plan_debug.hpp>
#include <d1max_planning_interfaces/msg/tracking_progress.hpp>
#include <d1max_planning_interfaces/msg/execution_handoff_grant.hpp>
#include <d1max_planning_interfaces/msg/prepared_motion_demand.hpp>
#include <d1max_planning_interfaces/msg/execution_commit_ack.hpp>
#include <d1max_planning_interfaces/msg/trajectory_admission.hpp>
#include <nlohmann/json.hpp>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <deque>
#include <iterator>

namespace scan_planner {
namespace ew=d1max_planning_interfaces::msg;
inline ew::ExecutionVersion executionVersion(const ew::ReferencePath& p) {
  ew::ExecutionVersion v;v.schema_version=3;v.session_id=p.session_id;v.task_id=p.task_id;
  v.route_id=p.route_id;v.route_hash=p.route_hash;v.map_version_id=p.map_version_id;
  v.localization_epoch=p.localization_epoch;v.localization_seed_id=p.localization_seed_id;
  v.reference_generation=p.generation;v.segment_id=p.segment_id;v.anchor_id=p.anchor_id;
  v.anchor_revision=p.anchor_revision;v.context_sequence=p.context_sequence;
  v.map_geometry_revision=p.map_geometry_revision;return v;
}
inline bool sameExecutionTask(const ew::ExecutionVersion& a,const ew::ExecutionVersion& b) {
  return a.session_id==b.session_id&&a.task_id==b.task_id&&a.route_id==b.route_id&&a.route_hash==b.route_hash&&
    a.map_version_id==b.map_version_id&&a.localization_epoch==b.localization_epoch&&a.localization_seed_id==b.localization_seed_id;
}
// Independent native-map reader. Fusion and FSM stay serialized on their
// existing writer executor; this thread ONLY accesses immutable native map
// snapshots, value-copied geometry and measured source poses. Never live map.
class ExecutionValidator {
  friend struct ExecutionValidatorTestAccess;
  struct BodySourceSample {
    MeasuredBodyPose pose;
    std::optional<geometry_msgs::msg::Twist> twist;
    std::int64_t source_ns;
  };
public:
  // Passive constructor exercises the exact transaction ledger without a ROS
  // graph or validator thread; no proof can be produced through this seam.
  struct PassiveLedger {};
  ExecutionValidator(PassiveLedger,CollisionSnapshotPool& pool,std::string frame,std::string mode,bool writer_handoff=false):
    node_(nullptr),pool_(pool),frame_(std::move(frame)),mode_(std::move(mode)),extent_(0.),writer_handoff_(writer_handoff){}
  struct CommittedView {ew::TaggedBspline spline;std::optional<ew::LocalPlanDebug> debug;
    std::optional<ew::TrajectoryValidation> proof;std::optional<ew::TrackingProgress> progress;};
  ExecutionValidator(rclcpp::Node* node,CollisionSnapshotPool& pool,std::string frame,
                     std::string mode,double extent,std::optional<BrakingModel> braking={},bool writer_handoff=false,
                     std::string geometry_policy="observed_free"):node_(node),pool_(pool),frame_(std::move(frame)),mode_(std::move(mode)),extent_(extent),braking_(std::move(braking)),writer_handoff_(writer_handoff),geometry_policy_(std::move(geometry_policy)) {
    if(mode_!="live"&&mode_!="isolated_mock") throw std::invalid_argument("invalid execution transport");
    if(geometry_policy_!="observed_free"&&geometry_policy_!="official_inflated_geometry")
      throw std::invalid_argument("invalid trajectory geometry policy");
    pub_=node_->create_publisher<ew::TrajectoryValidation>("/d1max/live_planning/execution/validation",rclcpp::QoS(2));
    committed_pub_=node_->create_publisher<ew::TaggedBspline>("planning/committed_bspline",rclcpp::QoS(1).transient_local());
    motion_pub_=node_->create_publisher<ew::MotionValidation>("/d1max/live_planning/execution/motion_validation",rclcpp::QoS(8));
    pipeline_pub_=node_->create_publisher<std_msgs::msg::String>("/d1max/live_planning/execution/pipeline_timing",rclcpp::QoS(8));
    thread_=std::thread([this]{run();});
    admission_thread_=std::thread([this]{runAdmission();});
  }
  ~ExecutionValidator(){
    stop_=true;wake_.notify_all();admission_wake_.notify_all();
    if(thread_.joinable())thread_.join();
    if(admission_thread_.joinable())admission_thread_.join();
  }
  // Map writer notification only: it shortens the fusion->proof latency and
  // never changes any source stamp, TTL or verdict. The 50 ms ceiling remains.
  void snapshotPublished(){{std::lock_guard<std::mutex> l(wait_mutex_);snapshot_ready_=true;}wake_.notify_all();}
  // Writer decides whether to refresh the third slot between complete fusion
  // transactions. Admission and solver share that ONE slot exclusively; an
  // unavailable slot is a bounded miss, never a fourth allocation or a copy of
  // a live querying map. Validation slots 0/1 remain reserved for fast checks.
  bool slowSnapshotWanted() const {std::lock_guard<std::mutex> l(mutex_);return active_.has_value()||candidate_.has_value()||prepared_.has_value();}
  void admissionSnapshotPublished(){{std::lock_guard<std::mutex> l(admission_wait_mutex_);admission_ready_=true;}
    admission_wake_.notify_all();}
  void body(const MeasuredBodyPose& p,
      const std::optional<geometry_msgs::msg::Twist>& world_twist={},std::int64_t source_ns=0) {
    if(!p.position.allFinite()||!p.orientation.coeffs().allFinite()||p.orientation.norm()<1e-6||
       !std::isfinite(p.source_stamp)||p.source_stamp<=0.||p.frame!=frame_)return;
    if(world_twist)for(double x:{world_twist->linear.x,world_twist->linear.y,world_twist->linear.z,
        world_twist->angular.x,world_twist->angular.y,world_twist->angular.z})if(!std::isfinite(x))return;
    if(source_ns==0)source_ns=static_cast<std::int64_t>(std::llround(p.source_stamp*1e9));
    if(source_ns<=0)return;
    std::lock_guard<std::mutex> l(mutex_);
    auto at=std::lower_bound(body_history_.begin(),body_history_.end(),source_ns,
      [](const BodySourceSample& s,std::int64_t t){return s.source_ns<t;});
    if(at!=body_history_.end()&&at->source_ns==source_ns) {
      // The FSM and independent inbox can deliver the same real odometry.
      // Duplicate callbacks do not manufacture a new sample or source lease.
      if(world_twist&&!at->twist)at->twist=world_twist;
    } else body_history_.insert(at,BodySourceSample{p,world_twist,source_ns});
    while(body_history_.size()>64 || (!body_history_.empty()&&
        body_history_.back().source_ns-body_history_.front().source_ns>1000000000LL))body_history_.pop_front();
    if(p.source_stamp>body_.source_stamp)body_=p;
  }
  void candidate(const ew::TaggedBspline& spline,const ew::ReferenceProposal& proposal,
      const SplineHeadingContract& frozen_heading={}) {
    // The no-motion solver's direction-confidence contract is part of this
    // exact curve. Copy its ORIGINAL heading seed, not the latest/join pose:
    // the independent reader must reproduce the same swept geometry after
    // measured progress advances. It never applies to the SDK writer lane.
    if(frozen_heading.preview_only_enabled&&
       (writer_handoff_||!validPreviewHeadingContract(frozen_heading)))
      throw std::invalid_argument("invalid no-motion candidate heading contract");
    std::lock_guard<std::mutex> l(mutex_);
    if(task_domain_&&((!sameExecutionTask(*task_domain_,proposal.version)&&!task_closed_)||
       (sameExecutionTask(*task_domain_,proposal.version)&&task_closed_)))return;
    if(retired_&&sameExecutionTask(*retired_,proposal.version)&&
       proposal.version.reference_generation<=retired_->reference_generation)return;
    candidate_=Job{spline,proposal.version,proposal.proposal_id,{},spline.valid_start_time,
      proposal.has_goal_yaw,proposal.goal_yaw,{proposal.goal_position.x,proposal.goal_position.y,proposal.goal_position.z},{},{},{},{}};
    candidate_->heading=frozen_heading;
    if(pending_support_&&pending_support_->version==proposal.version) candidate_->support=*pending_support_;
  }
  void support(const ew::SupportReference& s) {
    std::shared_ptr<const MotionSupport> indexed;
    {std::lock_guard<std::mutex> l(mutex_);if(motion_support_&&motion_support_->source.version==s.version&&
        motion_support_->source.support_hash==s.support_hash)indexed=motion_support_;}
    if(!indexed)indexed=std::make_shared<const MotionSupport>(s);
    std::lock_guard<std::mutex> l(mutex_);pending_support_=s;
    motion_support_=std::move(indexed);
    support_history_.push_back(motion_support_);while(support_history_.size()>8)support_history_.pop_front();
    if(candidate_&&candidate_->version==s.version)candidate_->support=s;
    if(active_&&active_->version==s.version)active_->support=s;
    if(prepared_&&prepared_->version==s.version&&prepared_->support&&
       prepared_->support->support_hash==s.support_hash)prepared_->support=s;
  }
  void candidateDebug(const ew::LocalPlanDebug& d) {
    std::lock_guard<std::mutex> l(mutex_);
    if(candidate_&&candidate_->version.reference_generation==d.generation&&
       candidate_->spline.trajectory.traj_id==static_cast<std::int64_t>(d.plan_id))candidate_->debug=d;
  }
  std::shared_ptr<const MotionSupport> planningSupport(const ew::ExecutionVersion& version)const {
    std::lock_guard<std::mutex> l(mutex_);
    for(auto i=support_history_.rbegin();i!=support_history_.rend();++i)
      if((*i)->valid&&(*i)->source.version==version)return *i;
    return {};
  }
  std::optional<BrakingModel> brakingModel()const { return braking_; }
  // A controller may request a DIFFERENT local entry, never authorize motion,
  // by echoing an exact recent native occupied verdict. No free-text trigger,
  // fabricated proof, stale task or duplicate can drive a replan loop.
  bool submitBlockedEntry(const ew::MotionValidation& proof) {
    return submitBlockedEntryAt(proof,node_->now().nanoseconds());
  }
  bool submitBlockedEntryAt(const ew::MotionValidation& proof,std::int64_t now) {
    std::lock_guard<std::mutex> l(mutex_);
    // Forward+turn or in-place turn; a zero or reverse command is never an entry.
    if(proof.valid||proof.reason!="motion_sweep_occupied"||proof.velocity.linear.x<0.||
       (proof.velocity.linear.x<=0.&&std::abs(proof.velocity.angular.z)<=1e-9)||
       proof.sequence<=last_blocked_entry_sequence_||ns(proof.check_end)>now||
       ns(proof.valid_until)<=now||!active_||active_->version!=proof.version||
       active_->spline.trajectory.traj_id!=proof.trajectory_id||!latest_permit_||
       latest_permit_->revoked||!latest_permit_->geometry_committed||
       latest_permit_->version!=proof.version)return false;
    const auto found=std::find(motion_proofs_.begin(),motion_proofs_.end(),proof);
    if(found==motion_proofs_.end())return false;
    last_blocked_entry_sequence_=proof.sequence;blocked_entry_=proof;return true;
  }
  std::optional<ew::MotionValidation> consumeBlockedEntry() {
    std::lock_guard<std::mutex> l(mutex_);
    auto event=std::move(blocked_entry_);blocked_entry_.reset();
    if(!event||!active_||active_->version!=event->version||
       active_->spline.trajectory.traj_id!=event->trajectory_id||!latest_permit_||
       latest_permit_->revoked)return {};
    return event;
  }
  // A real entry failure requests a new solve, not different collision policy
  // or permission. One latest event is retained; a curve is reseeded at most
  // once before a genuinely new solver curve replaces it. Proof/admission and
  // original body times must all name the same current candidate.
  struct FormalReseedRequest {
    ew::ExecutionVersion version;
    std::int64_t trajectory_id;
    builtin_interfaces::msg::Time body_source_stamp;
    std::string reason;
    bool terminal;
  };
  bool submitCandidateEntryRejection(const ew::TrajectoryAdmission& a) {
    return submitCandidateEntryRejectionAt(a,node_->now().nanoseconds());
  }
  bool submitCandidateEntryRejectionAt(const ew::TrajectoryAdmission& a,std::int64_t now) {
    std::lock_guard<std::mutex> l(mutex_);
    if(a.transport_mode!=mode_||a.sequence<=last_entry_admission_sequence_||!candidate_||!active_||
       a.version!=candidate_->version||a.trajectory_id!=candidate_->spline.trajectory.traj_id||
       !sameExecutionTask(active_->version,a.version)||
       (active_->version==a.version&&active_->spline.trajectory.traj_id==a.trajectory_id)||
       !latest_permit_||latest_permit_->revoked)return false;
    last_entry_admission_sequence_=a.sequence;
    if(a.accepted) {
      if(entry_reseed_&&entry_reseed_->version==a.version&&entry_reseed_->trajectory_id==a.trajectory_id)
        entry_reseed_.reset();
      return false;
    }
    if(a.reason!="candidate:entry_measured_body_not_on_candidate"&&
       a.reason!="candidate:entry_measured_velocity_off_candidate")return false;
    if(ns(a.checked_at)>now+20000000LL||now-ns(a.checked_at)>250000000LL||ns(a.valid_until)<=now||
       ns(a.body_source_stamp)>ns(a.checked_at)+20000000LL||now-ns(a.body_source_stamp)>100000000LL||
       ns(a.body_source_stamp)>now+20000000LL||ns(a.body_source_stamp)<=0)return false;
    const auto proof=std::find_if(proofs_.rbegin(),proofs_.rend(),[&](const auto& p){
      return p.sequence==a.validation_sequence&&p.version==a.version&&p.trajectory_id==a.trajectory_id&&
        p.valid&&p.whole_curve&&!p.remaining_curve&&p.frame_id==frame_&&p.transport_mode==mode_&&
        ns(p.check_end)<=ns(a.checked_at)&&ns(p.valid_until)>now;
    });
    if(proof==proofs_.rend()||
       (last_entry_reseed_&&last_entry_reseed_->first==a.version&&last_entry_reseed_->second==a.trajectory_id))return false;
    entry_reseed_=a;return true;
  }
  std::optional<FormalReseedRequest> consumeFormalReseedAt(std::int64_t now) {
    std::lock_guard<std::mutex> l(mutex_);
    if(!active_||!latest_permit_||latest_permit_->revoked) {entry_reseed_.reset();return {};}
    if(entry_reseed_) {
      const auto a=std::move(*entry_reseed_);entry_reseed_.reset();
      if(candidate_&&candidate_->version==a.version&&candidate_->spline.trajectory.traj_id==a.trajectory_id&&
         sameExecutionTask(active_->version,a.version)&&now-ns(a.checked_at)<=250000000LL&&
         ns(a.checked_at)<=now+20000000LL&&ns(a.valid_until)>now) {
        last_entry_reseed_=std::make_pair(a.version,a.trajectory_id);
        return FormalReseedRequest{a.version,a.trajectory_id,a.body_source_stamp,"measured_candidate_entry_changed",false};
      }
    }
    const auto& p=active_->progress;
    if(!p||!p->valid||!p->holding||p->reason!="local_segment_finished_waiting_replan"||
       ns(p->header.stamp)>now+20000000LL||now-ns(p->header.stamp)>400000000LL||
       (last_terminal_reseed_&&last_terminal_reseed_->first==active_->version&&
        last_terminal_reseed_->second==active_->spline.trajectory.traj_id))return {};
    last_terminal_reseed_=std::make_pair(active_->version,active_->spline.trajectory.traj_id);
    return FormalReseedRequest{active_->version,active_->spline.trajectory.traj_id,p->header.stamp,
      "measured_local_horizon_finished",true};
  }
  std::optional<CommittedView> committedView()const {
    std::lock_guard<std::mutex> l(mutex_);if(!active_)return {};
    return CommittedView{active_->spline,active_->debug,active_->proof,active_->progress};
  }
  struct Boundary {Eigen::Vector3d velocity,acceleration;};
  static std::optional<Boundary> measuredBoundary(const ew::TrackingProgress& p,const Eigen::Vector3d& body) {
    const Eigen::Vector3d measured_position{p.pose.position.x,p.pose.position.y,p.pose.position.z};
    if(!measured_position.allFinite()||!body.allFinite()||(measured_position-body).norm()>.0125)return {};
    Boundary result{{p.twist.linear.x,p.twist.linear.y,p.twist.linear.z},Eigen::Vector3d::Zero()};
    if(p.acceleration_valid)result.acceleration={p.acceleration.linear.x,p.acceleration.linear.y,p.acceleration.linear.z};
    if(!result.velocity.allFinite()||!result.acceleration.allFinite())return {};
    return result;
  }
  std::optional<Boundary> committedBoundary(const Eigen::Vector3d& body,
      const Eigen::Vector3d& current_velocity,std::int64_t body_source_ns,double now)const {
    const auto view=committedView();
    if(!view||!view->proof||!view->proof->valid||!view->progress||!view->progress->valid||
       view->progress->holding||rclcpp::Time(view->proof->valid_until).seconds()<=now||
       now-rclcpp::Time(view->progress->header.stamp).seconds()>.4||
       body_source_ns<=0||now-body_source_ns*1e-9<-.02||now-body_source_ns*1e-9>.4||
       !body.allFinite()||!current_velocity.allFinite())return {};
    // An older progress sample still identifies the committed curve, but its
    // speed is NOT the latest physical start boundary. Keep actual odometry
    // velocity; use reported acceleration only at the exact same source time.
    Boundary result{current_velocity,Eigen::Vector3d::Zero()};
    const auto& p=*view->progress;
    if(ns(p.header.stamp)==body_source_ns&&p.acceleration_valid) {
      const auto same_sample=measuredBoundary(p,body);
      if(same_sample)result.acceleration=same_sample->acceleration;
    }
    return result;
  }
  void progress(const ew::TrackingProgress& p) {
    progressAt(p,node_->now().seconds());
  }
  void progressAt(const ew::TrackingProgress& p,double now) {
    std::lock_guard<std::mutex> l(mutex_);if(!active_)return;
    const auto& v=active_->version;
    if(!p.valid||p.schema_version!=2||p.header.frame_id!=frame_||p.session_id!=v.session_id||
       p.task_id!=v.task_id||p.route_id!=v.route_id||p.route_hash!=v.route_hash||
       p.map_version_id!=v.map_version_id||p.localization_epoch!=v.localization_epoch||
       p.localization_seed_id!=v.localization_seed_id||p.generation!=v.reference_generation||
       p.anchor_id!=v.anchor_id||p.anchor_revision!=v.anchor_revision||p.context_sequence!=v.context_sequence||
       p.segment_id!=v.segment_id||p.trajectory_id!=active_->spline.trajectory.traj_id||
       !std::isfinite(p.curve_time)||p.curve_time<0.||!std::isfinite(p.arc_length)||p.arc_length<0.||
       !std::isfinite(p.s_committed)||p.s_committed<p.arc_length||p.arc_length<p.s_committed-.15)return;
    const double age=now-rclcpp::Time(p.header.stamp).seconds();
    if(age<-.02||age>.4)return;
    if(active_->progress&&(rclcpp::Time(p.header.stamp)<=rclcpp::Time(active_->progress->header.stamp)||
       p.s_committed<active_->progress->s_committed))return;
    MeasuredBodyPose measured{{p.pose.position.x,p.pose.position.y,p.pose.position.z},
      Eigen::Quaterniond(p.pose.orientation.w,p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z),
      rclcpp::Time(p.header.stamp).seconds(),p.header.frame_id};
    double yaw=0.;if(!measuredBodyYaw(measured,frame_,now,.4,yaw))return;
    // Progress is a curve projection of an acquired body state, not the
    // independent current-body inbox. It must not move that inbox forward or
    // overwrite the matched odometry velocity with an older tracker sample.
    active_->progress=p;
  }
  bool commit(const ew::ExecutionPermit& p) {
    std::lock_guard<std::mutex> l(mutex_);
    if(p.transport_mode!=mode_||p.sequence<=permit_sequence_)return false;
    if(!enterTaskDomainLocked(p))return false;
    if(!enterWriterDomainLocked(p))return false;
    if(writer_handoff_&&writer_commit_sequence_&&active_&&
       (p.version!=active_->version||p.trajectory_id!=active_->spline.trajectory.traj_id)&&
       p.version.reference_generation<=active_->version.reference_generation&&!p.revoked)return false;
    if(writer_handoff_&&writer_commit_sequence_&&active_&&
       (p.version!=active_->version||p.trajectory_id!=active_->spline.trajectory.traj_id)&&!p.revoked)return false;
    latest_permit_=p;permits_.push_back(p);while(permits_.size()>32)permits_.pop_front();
    if(p.revoked){retired_=p.version;task_closed_=true;active_.reset();candidate_.reset();if(grant_)grant_retired_=true;
      ++motion_stop_barrier_;permit_sequence_=p.sequence;return true;}
    if(!p.geometry_committed){permit_sequence_=p.sequence;return false;}
    if(active_&&active_->version==p.version&&active_->spline.trajectory.traj_id==p.trajectory_id) {permit_sequence_=p.sequence;return true;}
    if(!candidate_||candidate_->version!=p.version||candidate_->spline.trajectory.traj_id!=p.trajectory_id)return false;
    active_=candidate_;permit_sequence_=p.sequence;
    committed_jobs_.push_back(*active_);while(committed_jobs_.size()>8)committed_jobs_.pop_front();
    // A read-only durable geometry view for late UI subscribers. This is not
    // the candidate/admission channel; preserve original spline/source stamps.
    if(committed_pub_)committed_pub_->publish(active_->spline);
    return true;
  }
  std::optional<ew::ExecutionVersion> committedVersion()const {
    std::lock_guard<std::mutex> l(mutex_);if(active_)return active_->version;return {};
  }
  std::int64_t committedTrajectory()const {
    std::lock_guard<std::mutex> l(mutex_);return active_?active_->spline.trajectory.traj_id:-1;
  }
  bool cancelReference(const ew::ExecutionVersion& barrier) {
    std::lock_guard<std::mutex> l(mutex_);
    if(barrier.schema_version!=3||barrier.session_id.empty()||barrier.task_id.empty()||
       barrier.route_id.empty()||barrier.route_hash.empty()||barrier.reference_generation==0)return false;
    for(const auto* job:{&active_,&candidate_})if(*job &&
        (!sameExecutionTask((*job)->version,barrier)||
         barrier.reference_generation<=(*job)->version.reference_generation))return false;
    retired_=barrier;task_closed_=true;active_.reset();candidate_.reset();
    if(grant_)grant_retired_=true;
    ++motion_stop_barrier_;++prepared_stop_barrier_;
    return true;
  }
  void cancelPending(){std::lock_guard<std::mutex> l(mutex_);candidate_.reset();}
  bool handoff(const ew::ExecutionHandoffGrant& g) {return handoffAt(g,node_->now().nanoseconds());}
  bool handoffAt(const ew::ExecutionHandoffGrant& g,std::int64_t now) {
    std::lock_guard<std::mutex> l(mutex_);
    if(!writer_handoff_||g.schema_version!=2||g.handoff_id.empty()||g.sequence==0||
       g.expected_commit_sequence!=writer_commit_sequence_||g.candidate.transport_mode!=mode_)return false;
    if(grant_&&grant_->handoff_id==g.handoff_id) {
      if(g.revoked&&g.sequence>=grant_->sequence){grant_retired_=true;++prepared_stop_barrier_;return true;}
      return g==*grant_;
    }
    if(grant_&&!grant_applied_)return false; // timeout tombstone is not an empty slot.
    if(!handoffModeValid(g,now))return false;
    if(g.revoked||!active_||!latest_permit_||g.incumbent.version!=active_->version||
       g.incumbent.trajectory_id!=active_->spline.trajectory.traj_id||
       g.incumbent.execution_id!=latest_permit_->execution_id||g.incumbent.control_epoch!=latest_permit_->control_epoch||
       !g.candidate.allowed||g.candidate.geometry_committed||g.candidate.revoked||
       !sameExecutionTask(g.incumbent.version,g.candidate.version)||
       ns(g.source_stamp)>now+20000000LL||now-ns(g.source_stamp)>750000000LL||ns(g.valid_until)<=now||
       ns(g.transition_deadline)<=now||ns(g.transition_deadline)>ns(g.valid_until)||
       ns(g.candidate.valid_until)<=now||g.candidate.frame_id!=frame_)return false;
    const auto matches=[&](const std::optional<Job>& j){return j&&j->version==g.candidate.version&&
      j->spline.trajectory.traj_id==g.candidate.trajectory_id;};
    if(grant_) {tombstone_grant_=grant_;tombstone_job_=prepared_;tombstone_retired_=grant_retired_;
      tombstone_motion_proofs_=prepared_motion_proofs_;}
    if(matches(candidate_))prepared_=candidate_;
    else if(matches(active_))prepared_=active_;
    else return false;
    grant_=g;grant_retired_=grant_applied_=false;prepared_demand_.reset();prepared_motion_proofs_.clear();
    ++prepared_stop_barrier_;return true;
  }
  void preparedDemand(const ew::PreparedMotionDemand& d) {
    std::lock_guard<std::mutex> l(mutex_);
    if(!writer_handoff_||!grant_||grant_retired_||grant_applied_||d.schema_version!=1||
       d.handoff_id!=grant_->handoff_id||d.grant_sequence!=grant_->sequence||
       d.expected_commit_sequence!=grant_->expected_commit_sequence||
       d.demand.version!=grant_->candidate.version||d.demand.trajectory_id!=grant_->candidate.trajectory_id||
       d.demand.transport_mode!=mode_||(prepared_demand_&&d.demand.sequence<=prepared_demand_->demand.sequence))return;
    // The pending geometry is fixed. Only its latest measured entry/command is
    // refreshed; zero demands still need a full real proof before writer CAS.
    prepared_demand_=d;
  }
  bool commitAck(const ew::ExecutionCommitAck& a) {return commitAckAt(a,node_->now().nanoseconds());}
  bool commitAckAt(const ew::ExecutionCommitAck& a,std::int64_t now) {
    std::lock_guard<std::mutex> l(mutex_);
    if(!writer_handoff_||a.schema_version!=1||a.sequence<=ack_sequence_||a.transport_mode!=mode_||
       !task_domain_||task_closed_||!sameExecutionTask(*task_domain_,a.candidate_version)||
       !writer_domain_||a.execution_id!=writer_domain_->execution_id||a.control_epoch!=writer_domain_->control_epoch||
       a.sdk_session!=writer_domain_->sdk_session||a.sdk_arm_generation!=writer_domain_->sdk_arm_generation)return false;
    if(a.handoff_id.empty()) {
      const auto matches=[&](const auto& p){
        return a.permit_sequence==p.sequence&&a.candidate_version==p.version&&a.candidate_trajectory_id==p.trajectory_id&&
          a.execution_id==p.execution_id&&a.control_epoch==p.control_epoch&&a.sdk_session==p.sdk_session&&
          a.sdk_arm_generation==p.sdk_arm_generation&&p.allowed&&p.geometry_committed&&!p.revoked&&
          ns(p.source_stamp)<=ns(a.applied_at)&&ns(p.valid_until)>ns(a.applied_at);
      };
      const ew::ExecutionPermit* first=nullptr;const Job* original=nullptr;
      for(const auto& p:permits_)if(matches(p))first=&p;
      for(const auto& j:committed_jobs_)if(j.version==a.candidate_version&&
          j.spline.trajectory.traj_id==a.candidate_trajectory_id)original=&j;
      for(const auto& intent:initial_intents_)if(matches(intent.permit)&&
          intent.geometry.version==a.candidate_version&&
          intent.geometry.spline.trajectory.traj_id==a.candidate_trajectory_id) {
        first=&intent.permit;original=&intent.geometry;
      }
      if(!a.applied||a.previous_commit_sequence||a.commit_sequence!=1||!first||!original)return false;
      if(writer_commit_sequence_) {
        if(writer_commit_sequence_!=1||!initial_writer_ack_)return false;
        auto expected=*initial_writer_ack_,actual=a;
        expected.sequence=actual.sequence=0;expected.write_acknowledged=actual.write_acknowledged=false;
        expected.reason.clear();actual.reason.clear();
        if(expected!=actual)return false;
        ack_sequence_=a.sequence;return true;
      }
      const bool current=active_&&a.candidate_version==active_->version&&
        a.candidate_trajectory_id==active_->spline.trajectory.traj_id;
      writer_commit_sequence_=1;ack_sequence_=a.sequence;initial_writer_ack_=a;
      if(!current||!a.write_submitted||ns(a.valid_until)<=now||!latest_permit_||ns(latest_permit_->valid_until)<=now) {
        // Restore only the original bounded geometry fact. No old source pose,
        // progress or collision lease is a current authorization. The unique
        // Owner must reconcile this ACK before it can authorize another entry.
        active_=*original;active_->proof.reset();active_->progress.reset();candidate_.reset();
        auto held=*first;held.allowed=false;held.phase="holding";latest_permit_=held;
        permit_sequence_=std::max(permit_sequence_,first->sequence);
        if(grant_)grant_retired_=true;
        latest_demand_.reset();prepared_demand_.reset();++motion_stop_barrier_;++prepared_stop_barrier_;
        if(committed_pub_)committed_pub_->publish(active_->spline);
      }
      return true;
    }
    const bool tombstone=(!grant_||grant_->handoff_id!=a.handoff_id)&&tombstone_grant_&&tombstone_grant_->handoff_id==a.handoff_id;
    const auto& selected_grant=tombstone?tombstone_grant_:grant_;
    const auto& selected_job=tombstone?tombstone_job_:prepared_;
    if(!selected_grant||!selected_job)return false;
    const auto& g=*selected_grant;const auto& p=g.candidate;
    if(a.handoff_id!=g.handoff_id||a.grant_sequence!=g.sequence||
       a.previous_commit_sequence!=g.expected_commit_sequence||a.candidate_version!=p.version||
       a.incumbent_version!=g.incumbent.version||a.candidate_trajectory_id!=p.trajectory_id||
       a.incumbent_trajectory_id!=g.incumbent.trajectory_id||a.execution_id!=p.execution_id||
       a.control_epoch!=p.control_epoch||a.sdk_session!=p.sdk_session||a.sdk_arm_generation!=p.sdk_arm_generation||
       a.permit_sequence!=p.sequence)return false;
    if(!a.applied) {
      if(a.write_submitted||a.write_acknowledged||a.commit_sequence!=a.previous_commit_sequence||
         a.commit_sequence!=writer_commit_sequence_)return false;
      if(tombstone)tombstone_retired_=true;
      else{grant_retired_=true;grant_applied_=true;prepared_demand_.reset();++prepared_stop_barrier_;}
      ack_sequence_=a.sequence;return true;
    }
    if(a.commit_sequence!=a.previous_commit_sequence+1)return false;
    if(writer_commit_sequence_==a.commit_sequence&&active_&&active_->version==p.version&&
       active_->spline.trajectory.traj_id==p.trajectory_id) {ack_sequence_=a.sequence;return true;}
    if(a.previous_commit_sequence!=writer_commit_sequence_)return false;
    if(ns(a.applied_at)<ns(g.source_stamp)||ns(a.applied_at)>ns(g.transition_deadline)||
       ns(a.applied_at)<ns(p.source_stamp)||ns(a.applied_at)>ns(p.valid_until))return false;
    const auto& transaction_proofs=tombstone?tombstone_motion_proofs_:prepared_motion_proofs_;
    const auto proof=std::find_if(transaction_proofs.begin(),transaction_proofs.end(),[&](const auto& m){
      return m.handoff_id==g.handoff_id&&m.sequence==a.motion_validation_sequence&&m.valid&&
        m.demand_sequence==a.demand_sequence&&m.entry_admission_sequence==a.entry_admission_sequence&&
        m.demand_source_stamp==a.demand_source_stamp&&m.demand_body_source_stamp==a.demand_body_source_stamp&&
        m.velocity==a.applied_velocity&&m.entry_curve_time==a.curve_time;
    });
    if(proof==transaction_proofs.end())return false;
    // Applied facts do not acquire a new TTL and cannot be rejected merely
    // because the callback arrived after the original proof expired.
    const bool hold=(tombstone?tombstone_retired_:grant_retired_)||!a.write_submitted||
      ns(a.valid_until)<=now||ns(p.valid_until)<=now;
    active_=selected_job;candidate_.reset();
    if(tombstone)grant_retired_=true; // Current conditional expected an obsolete writer identity.
    else {grant_applied_=true;grant_retired_=hold;}
    writer_commit_sequence_=a.commit_sequence;ack_sequence_=a.sequence;auto promoted=p;promoted.geometry_committed=true;latest_permit_=promoted;
    permit_sequence_=std::max(permit_sequence_,p.sequence);permits_.push_back(promoted);
    while(permits_.size()>32)permits_.pop_front();
    ++motion_stop_barrier_;++prepared_stop_barrier_;latest_demand_.reset();prepared_demand_.reset();
    if(committed_pub_)committed_pub_->publish(active_->spline);
    return true;
  }
  void demand(const ew::MotionDemand& d) {
    std::lock_guard<std::mutex> l(mutex_);
    if(d.transport_mode!=mode_||d.version.schema_version!=3)return;
    if(writer_handoff_&&active_&&(d.version!=active_->version||d.trajectory_id!=active_->spline.trajectory.traj_id))return;
    if(latest_demand_&&d.execution_id==latest_demand_->execution_id&&d.sequence<=latest_demand_->sequence)return;
    latest_demand_=d;
    if(!writer_commit_sequence_&&active_)for(auto it=permits_.rbegin();it!=permits_.rend();++it)
      if(it->sequence==d.permit_sequence&&it->version==d.version&&it->trajectory_id==d.trajectory_id) {
        pinInitialIntentLocked(d,*it,*active_);break;
      }
    if(d.hold||std::abs(d.velocity.linear.x)+std::abs(d.velocity.angular.z)<1e-12)++motion_stop_barrier_;
  }
private:
  struct Job {ew::TaggedBspline spline;ew::ExecutionVersion version;std::string proposal;
    std::optional<ew::SupportReference> support;double measured_time{0.};
    bool has_goal_yaw{false};double goal_yaw{0.};Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
    std::optional<ew::LocalPlanDebug> debug;std::optional<ew::TrajectoryValidation> proof;
    std::optional<ew::TrackingProgress> progress;
    SplineHeadingContract heading;};
  struct InitialIntent {ew::ExecutionPermit permit;Job geometry;};
  bool enterWriterDomainLocked(const ew::ExecutionPermit& p) {
    if(p.revoked||p.execution_id.empty())return true;
    if(!writer_domain_) {writer_domain_=p;return true;}
    const auto& old=*writer_domain_;
    if(p.execution_id==old.execution_id&&p.control_epoch==old.control_epoch&&
       p.sdk_session==old.sdk_session&&p.sdk_arm_generation==old.sdk_arm_generation)return true;
    if(p.allowed||p.control_epoch<=old.control_epoch||(p.phase!="arming"&&p.phase!="holding"))return false;
    writer_domain_=p;writer_commit_sequence_=ack_sequence_=0;initial_writer_ack_.reset();initial_intents_.clear();
    grant_.reset();tombstone_grant_.reset();prepared_.reset();tombstone_job_.reset();prepared_demand_.reset();
    prepared_motion_proofs_.clear();tombstone_motion_proofs_.clear();latest_demand_.reset();
    ++motion_stop_barrier_;++prepared_stop_barrier_;return true;
  }
  bool enterTaskDomainLocked(const ew::ExecutionPermit& p) {
    if(p.version.schema_version!=3||p.version.task_id.empty()||p.version.session_id.empty())return false;
    if(task_domain_&&sameExecutionTask(*task_domain_,p.version))return !task_closed_;
    if(task_domain_&&!task_closed_)return false;
    if(retired_&&sameExecutionTask(*retired_,p.version))return false;
    const auto now=node_?node_->now().nanoseconds():ns(p.source_stamp);
    if(ns(p.source_stamp)<=0||ns(p.source_stamp)>now+20000000LL||
       now-ns(p.source_stamp)>750000000LL||ns(p.valid_until)<=now)return false;
    // Only the fresh typed owner ingress establishes a new task. A solve or
    // map correction never resets writer CAS state. Keep publisher-wide
    // permit/motion sequence floors and the acquired body history intact.
    active_.reset();prepared_.reset();tombstone_job_.reset();grant_.reset();tombstone_grant_.reset();
    prepared_demand_.reset();latest_demand_.reset();blocked_entry_.reset();entry_reseed_.reset();
    last_entry_reseed_.reset();last_terminal_reseed_.reset();committed_jobs_.clear();
    permits_.clear();motion_proofs_.clear();prepared_motion_proofs_.clear();tombstone_motion_proofs_.clear();
    initial_writer_ack_.reset();initial_intents_.clear();writer_domain_.reset();
    writer_commit_sequence_=ack_sequence_=0;grant_retired_=grant_applied_=tombstone_retired_=false;
    if(candidate_&&!sameExecutionTask(candidate_->version,p.version))candidate_.reset();
    proofs_.erase(std::remove_if(proofs_.begin(),proofs_.end(),[&](const auto& proof){
      return !sameExecutionTask(proof.version,p.version);}),proofs_.end());
    task_domain_=p.version;task_closed_=false;++motion_stop_barrier_;++prepared_stop_barrier_;
    return true;
  }
  void pinInitialIntentLocked(const ew::MotionDemand& d,const ew::ExecutionPermit& p,const Job& j) {
    if(!writer_handoff_||writer_commit_sequence_||!p.allowed||!p.geometry_committed||p.revoked||
       p.execution_id.empty()||p.sdk_session.empty()||p.version!=j.version||
       p.trajectory_id!=j.spline.trajectory.traj_id||d.execution_id!=p.execution_id||
       d.control_epoch!=p.control_epoch||d.sdk_session!=p.sdk_session||d.sdk_arm_generation!=p.sdk_arm_generation||
       ns(d.source_stamp)<ns(p.source_stamp)||ns(d.source_stamp)>ns(p.valid_until))return;
    for(const auto& old:initial_intents_)if(old.permit==p)return;
    initial_intents_.push_back({p,j});
    if(initial_intents_.size()>8)initial_intents_.erase(std::next(initial_intents_.begin()));
  }
  void run() {
    while(!stop_) {
      const auto begin=std::chrono::steady_clock::now();
      try {
        CollisionSnapshotPool::PublicationTiming timing;
        auto snapshot=pool_.borrowValidation(&timing);
        const auto borrowed=std::chrono::steady_clock::now();
        const auto borrowed_ns=node_->now().nanoseconds();
        std::optional<Job> active;MeasuredBodyPose body;
        {std::lock_guard<std::mutex> l(mutex_);active=active_;body=body_;}
        if(snapshot) {
          snapshot->requireObservedSnapshot();
          // Actual incumbent command first, then at most one prepared command
          // using the same fixed snapshot and SAME 50 ms round. Preparation is
          // not ordinary motion and cannot consume a second deadline budget.
          ValidationCycle::priorityPassWithPrepared(begin,
            [&](double remaining){checkMotion(*snapshot,remaining,begin+ValidationCycle::period);},
            [&](double remaining){return active&&check(*active,body,*snapshot,remaining,true,
                begin+ValidationCycle::period);},
            [&](double remaining){checkPreparedMotion(*snapshot,remaining,begin+ValidationCycle::period);},
            []{return ValidationCycle::Clock::now();});
          if(timing.publication_sequence!=last_timing_publication_) {
            const auto end=std::chrono::steady_clock::now();
            std_msgs::msg::String message;
            message.data=nlohmann::json{{"schema",1},{"transport_mode",mode_},
              {"fusion_sequence",timing.fusion.sequence},{"map_revision",timing.map_revision},
              {"front_ray_begin_ns",timing.fusion.source_stamps[0]},{"rear_ray_begin_ns",timing.fusion.source_stamps[1]},
              {"fusion_begin_ns",timing.fusion.begin_ns},{"fusion_end_ns",timing.fusion.end_ns},
              {"fusion_begin_steady_ns",timing.fusion.begin_steady_ns},{"fusion_end_steady_ns",timing.fusion.end_steady_ns},
              {"snapshot_publication_sequence",timing.publication_sequence},
              {"snapshot_publication_misses",timing.publication_misses},
              {"snapshot_publish_begin_steady_ns",timing.begin_steady_ns},{"snapshot_publish_end_steady_ns",timing.end_steady_ns},
              {"borrow_ns",borrowed_ns},
              {"borrow_steady_ns",std::chrono::duration_cast<std::chrono::nanoseconds>(borrowed.time_since_epoch()).count()},
              {"check_end_ns",node_->now().nanoseconds()},
              {"check_end_steady_ns",std::chrono::duration_cast<std::chrono::nanoseconds>(end.time_since_epoch()).count()}}.dump();
            pipeline_pub_->publish(message);last_timing_publication_=timing.publication_sequence;
          }
        }
      } catch(const std::bad_alloc&) {std::terminate();}
      catch(const std::exception& e){RCLCPP_ERROR_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,"Native validation failed: %s",e.what());}
      // 20 Hz scheduling ceiling, not a claim of 20 Hz map fusion/full checks.
      std::unique_lock<std::mutex> lock(wait_mutex_);
      wake_.wait_until(lock,begin+std::chrono::milliseconds(50),[&]{return stop_.load()||snapshot_ready_;});
      snapshot_ready_=false;
    }
  }
  void runAdmission() {
    while(!stop_) {
      const auto begin=std::chrono::steady_clock::now();
      try {
        // A slow reader cannot block fast command checking or take either of
        // its double buffers. It also cannot query concurrently with SCAN.
        std::optional<Job> active,candidate,prepared;MeasuredBodyPose body;
        {std::lock_guard<std::mutex> l(mutex_);active=active_;candidate=candidate_;
          if(!grant_retired_&&!grant_applied_)prepared=prepared_;
          body=body_;}
        // Register only real work. The fixed third-slot arbiter alternates
        // contending solver/admission grants; retiring all jobs withdraws this
        // request rather than reserving an idle slot indefinitely.
        auto snapshot=(active||candidate||prepared)?pool_.borrowAdmission():GridMap::Ptr{};
        if(!active&&!candidate&&!prepared)pool_.cancelAdmissionRequest();
        if(snapshot) {
          snapshot->requireObservedSnapshot();
          // Incumbent renewal has priority over admitting an optional candidate.
          if(active)check(*active,body,*snapshot,ValidationCycle::slow_curve_budget_s,true);
          const auto& pending=prepared?prepared:candidate;
          if(pending&&(!active||pending->version!=active->version||
              pending->spline.trajectory.traj_id!=active->spline.trajectory.traj_id)) {
            const double remaining=ValidationCycle::slow_curve_budget_s-
              std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
            if(remaining>0.)check(*pending,body,*snapshot,remaining,false);
          }
        }
      } catch(const std::bad_alloc&) {std::terminate();}
      catch(const std::exception& e){RCLCPP_ERROR_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
        "Native admission failed: %s",e.what());}
      std::unique_lock<std::mutex> lock(admission_wait_mutex_);
      admission_wake_.wait_until(lock,begin+std::chrono::milliseconds(200),
        [&]{return stop_.load()||admission_ready_;});admission_ready_=false;
    }
  }
  static std::int64_t ns(const builtin_interfaces::msg::Time& t){return rclcpp::Time(t).nanoseconds();}
  static bool proofSupersedes(const ew::TrajectoryValidation& incoming,const ew::TrajectoryValidation& current) {
    return incoming.sequence>current.sequence&&incoming.map_snapshot_revision>=current.map_snapshot_revision&&
      ns(incoming.body_source_stamp)>=ns(current.body_source_stamp);
  }
  static bool demandPermitMatches(const ew::MotionDemand& d,const ew::ExecutionPermit& p,
      std::uint64_t checked_sequence=0) {
    const auto evidence=checked_sequence?checked_sequence:d.validation_sequence;
    return d.version==p.version&&d.execution_id==p.execution_id&&d.control_epoch==p.control_epoch&&
      d.sdk_session==p.sdk_session&&d.sdk_arm_generation==p.sdk_arm_generation&&d.trajectory_id==p.trajectory_id&&
      evidence>=d.validation_sequence&&evidence>=p.validation_sequence&&
      p.allowed&&p.geometry_committed&&!p.revoked&&(p.phase=="tracking"||p.phase=="aligning");
  }
  // Demand names the evidence known when control was computed, not an exact
  // immutable collision lease. The same geometry can be rechecked before the
  // command is validated. Never carry an old command over a negative result:
  // only a newly generated demand whose floor is AFTER that result can recover.
  static std::optional<ew::TrajectoryValidation> motionProof(
      const std::deque<ew::TrajectoryValidation>& history,const ew::MotionDemand& demand,
      std::int64_t now) {
    if(demand.validation_sequence==0)return {};
    const ew::TrajectoryValidation* newest=nullptr;
    for(const auto& proof:history) {
      if(proof.version!=demand.version||proof.trajectory_id!=demand.trajectory_id||
         proof.sequence<demand.validation_sequence)continue;
      if(!proof.valid)return {};
      if(!newest||proof.sequence>newest->sequence)newest=&proof;
    }
    if(!newest||ns(newest->check_end)>now||ns(newest->valid_until)<=now)return {};
    return *newest;
  }
  bool handoffModeValid(const ew::ExecutionHandoffGrant& g,std::int64_t now)const {
    if(g.transition_mode==ew::ExecutionHandoffGrant::CONTINUOUS_REPLACE)
      return g.incumbent.allowed&&!g.incumbent.revoked&&g.incumbent.geometry_committed&&
        (g.incumbent.phase=="tracking"||g.incumbent.phase=="aligning");
    if(g.transition_mode!=ew::ExecutionHandoffGrant::STATIONARY_REENTRY||g.incumbent.allowed||
       g.incumbent.revoked||!g.incumbent.geometry_committed||g.incumbent.phase!="holding"||
       !latest_permit_||latest_permit_->allowed||latest_permit_->revoked||latest_permit_->phase!="holding"||
       latest_permit_->version!=g.incumbent.version||latest_permit_->trajectory_id!=g.incumbent.trajectory_id)return false;
    const auto& e=g.stationary_evidence;const auto& p=g.incumbent;
    return e.schema_version==1&&e.usable&&e.nonzero_blocked&&e.sequence>0&&e.zero_write_sequence>0&&
      e.version==p.version&&e.execution_id==p.execution_id&&e.control_epoch==p.control_epoch&&
      e.sdk_session==p.sdk_session&&e.sdk_arm_generation==p.sdk_arm_generation&&e.transport_mode==mode_&&
      e.writer_commit_sequence==g.expected_commit_sequence&&e.applied_trajectory_id==p.trajectory_id&&
      e.mc_raw_stamp_ns>0&&!e.mc_clock_epoch.empty()&&!e.time_basis.empty()&&
      e.stationary_samples>=3&&std::isfinite(e.stationary_duration_sec)&&e.stationary_duration_sec>=.6&&
      std::isfinite(e.measured_linear_mps)&&std::isfinite(e.measured_angular_radps)&&
      ns(e.zero_ack_at)>0&&ns(e.capture_lower_bound)>ns(e.zero_ack_at)&&
      ns(e.capture_upper_bound)>=ns(e.capture_lower_bound)&&
      ns(e.source_stamp)>0&&ns(e.source_stamp)<=now+20000000LL&&now-ns(e.source_stamp)<=250000000LL&&
      ns(e.valid_until)>now&&ns(e.valid_until)>ns(e.source_stamp)&&ns(e.valid_until)-ns(e.source_stamp)<=250001000LL&&
      ns(g.source_stamp)>=ns(e.source_stamp)&&g.retain_incumbent_until==g.source_stamp&&
      ns(g.valid_until)<=ns(e.valid_until)&&ns(g.candidate.valid_until)<=ns(e.valid_until)&&
      ns(g.valid_until)-ns(g.source_stamp)<=250001000LL;
  }
  static bool preparedEntryMatches(const ew::PreparedMotionDemand& x,const ew::ExecutionHandoffGrant& g,
      const std::string& frame,std::int64_t now) {
    const auto& d=x.demand;const auto& a=x.entry_admission;const auto& p=g.candidate;
    return g.schema_version==2&&x.schema_version==1&&x.handoff_id==g.handoff_id&&x.grant_sequence==g.sequence&&
      x.expected_commit_sequence==g.expected_commit_sequence&&d.version==p.version&&
      d.execution_id==p.execution_id&&d.control_epoch==p.control_epoch&&d.sdk_session==p.sdk_session&&
      d.sdk_arm_generation==p.sdk_arm_generation&&d.permit_sequence==p.sequence&&d.trajectory_id==p.trajectory_id&&
      d.validation_sequence>=p.validation_sequence&&d.transport_mode==p.transport_mode&&a.transport_mode==p.transport_mode&&
      a.sequence>0&&a.accepted&&a.version==d.version&&a.trajectory_id==d.trajectory_id&&a.validation_sequence==d.validation_sequence&&
      a.body_source_stamp==d.body_source_stamp&&x.entry_source_stamp==d.body_source_stamp&&
      x.measured_pose.header.stamp==x.entry_source_stamp&&x.measured_pose.header.frame_id==frame&&
      motionSourceFresh(ns(d.source_stamp),now)&&motionSourceFresh(ns(d.body_source_stamp),now)&&
      ns(d.valid_until)>now&&ns(d.valid_until)<=ns(d.source_stamp)+100000001LL&&
      ns(a.valid_until)>now&&ns(a.valid_until)<=ns(d.valid_until)&&ns(a.checked_at)>=ns(d.source_stamp)&&
      ns(g.valid_until)>now&&ns(g.transition_deadline)>now&&ns(p.valid_until)>now&&
      std::isfinite(x.curve_time)&&x.curve_time>=0.&&std::isfinite(x.position_tolerance_m)&&
      x.position_tolerance_m>0.&&x.position_tolerance_m<=.0125&&std::isfinite(x.velocity_tolerance_mps)&&
      x.velocity_tolerance_mps>0.&&x.velocity_tolerance_mps<=.05&&
      std::isfinite(x.position_error_m)&&x.position_error_m>=0.&&x.position_error_m<=x.position_tolerance_m&&
      std::isfinite(x.velocity_error_mps)&&x.velocity_error_mps>=0.&&x.velocity_error_mps<=x.velocity_tolerance_mps;
  }
  void checkPreparedMotion(GridMap& map,double budget_s,ValidationCycle::Clock::time_point round_deadline) {
    std::optional<ew::PreparedMotionDemand> request;std::optional<ew::ExecutionHandoffGrant> grant;
    std::optional<Job> job;std::optional<ew::TrajectoryValidation> proof;MeasuredBodyPose current;
    std::optional<BodySourceSample> entry_sample;
    std::shared_ptr<const MotionSupport> support;std::uint64_t barrier=0;
    {std::lock_guard<std::mutex> l(mutex_);
      if(!prepared_demand_||!grant_||grant_retired_||grant_applied_||!prepared_)return;
      request=prepared_demand_;grant=grant_;job=prepared_;current=body_;barrier=prepared_stop_barrier_;
      entry_sample=bodyAtSourceLocked(ns(request->entry_source_stamp));
      proof=motionProof(proofs_,request->demand,node_->now().nanoseconds());
      for(auto i=support_history_.rbegin();i!=support_history_.rend();++i)
        if((*i)->source.version==request->demand.version){support=*i;break;}}
    const auto& x=*request;const auto& d=x.demand;const auto& g=*grant;const auto now=node_->now().nanoseconds();
    if(!proof||!proof->whole_curve||!proof->valid||!job->support||!support||!support->valid||!braking_||
       !entry_sample||!entry_sample->twist||
       !preparedEntryMatches(x,g,frame_,now)||!map.isRawRaySnapshot()||!map.integratedCloudFreshAt(now)||
       map.localizationContextSequence()!=d.version.context_sequence)return;
    const auto& s=job->spline.trajectory;
    if(s.order!=3||s.pos_pts.size()<4||s.knots.size()!=s.pos_pts.size()+4)return;
    Eigen::MatrixXd points(3,s.pos_pts.size());Eigen::VectorXd knots(s.knots.size());
    for(std::size_t i=0;i<s.pos_pts.size();++i)points.col(i)=Eigen::Vector3d(s.pos_pts[i].x,s.pos_pts[i].y,s.pos_pts[i].z);
    for(std::size_t i=0;i<s.knots.size();++i)knots[i]=s.knots[i];
    UniformBspline curve(points,3,.1);curve.setKnot(knots);const auto velocity=curve.getDerivative();
    if(x.curve_time>curve.getTimeSum())return;
    const Eigen::Vector3d position{x.measured_pose.pose.position.x,x.measured_pose.pose.position.y,x.measured_pose.pose.position.z};
    const Eigen::Vector3d measured_velocity{x.measured_twist.linear.x,x.measured_twist.linear.y,x.measured_twist.linear.z};
    const Eigen::Vector3d entry{x.curve_entry_pose.position.x,x.curve_entry_pose.position.y,x.curve_entry_pose.position.z};
    const Eigen::Vector3d entry_velocity{x.curve_entry_twist.linear.x,x.curve_entry_twist.linear.y,x.curve_entry_twist.linear.z};
    const double pe=(position-entry).norm(),ve=(measured_velocity-entry_velocity).norm();
    if(!position.allFinite()||!measured_velocity.allFinite()||!entry.allFinite()||!entry_velocity.allFinite()||
       (curve.evaluateDeBoorT(x.curve_time)-entry).norm()>1e-6||
       (velocity.evaluateDeBoorT(x.curve_time)-entry_velocity).norm()>1e-6||
       pe>x.position_tolerance_m||ve>x.velocity_tolerance_mps||
       std::abs(pe-x.position_error_m)>1e-6||std::abs(ve-x.velocity_error_mps)>1e-6||
       current.source_stamp<ns(x.entry_source_stamp)*1e-9||now*1e-9-current.source_stamp>.1||
       (entry_sample->pose.position-position).norm()>.0125||
       (Eigen::Vector3d(entry_sample->twist->linear.x,entry_sample->twist->linear.y,
                        entry_sample->twist->linear.z)-measured_velocity).norm()>.05)return;
    // Source-time consistency and latest-entry safety are different checks.
    // The request's acquired state is paired with real odometry at that time;
    // a later body sample must not make ordinary motion look like a bad entry.
    // The unique writer still reobserves the actual latest entry before CAS.
    MeasuredBodyPose body{position,Eigen::Quaterniond(x.measured_pose.pose.orientation.w,x.measured_pose.pose.orientation.x,
      x.measured_pose.pose.orientation.y,x.measured_pose.pose.orientation.z),ns(x.entry_source_stamp)*1e-9,frame_};
    double yaw=0.;if(!measuredBodyYaw(body,frame_,now*1e-9,.1,yaw)||
       !measuredConnectionSupported(*job->support,position,entry,frame_,map.getResolution()))return;
    const double remaining=std::min(budget_s,std::chrono::duration<double>(round_deadline-ValidationCycle::Clock::now()).count());
    if(remaining<=0.)return;
    const auto sweep=validateMotionSweep(map,*support,*braking_,position,yaw,x.measured_twist,d.velocity,frame_,remaining);
    ew::MotionValidation out;out.version=d.version;out.execution_id=d.execution_id;out.control_epoch=d.control_epoch;
    out.sdk_session=d.sdk_session;out.sdk_arm_generation=d.sdk_arm_generation;out.trajectory_id=d.trajectory_id;
    out.permit_sequence=d.permit_sequence;out.trajectory_validation_sequence=proof->sequence;out.demand_sequence=d.sequence;
    out.demand_source_stamp=d.source_stamp;out.demand_body_source_stamp=d.body_source_stamp;out.demand_valid_until=d.valid_until;
    out.sequence=++motion_sequence_;out.velocity=d.velocity;out.frame_id=frame_;out.transport_mode=mode_;
    out.braking_model_sha256=braking_->sha256;out.check_begin=rclcpp::Time(now);out.check_end=node_->now();
    out.body_source_stamp=x.entry_source_stamp;out.front_ray_source_stamp=rclcpp::Time(map.integratedRaySourceStamp(0));
    out.rear_ray_source_stamp=rclcpp::Time(map.integratedRaySourceStamp(1));out.map_snapshot_revision=map.occupancyRevision();
    out.valid=sweep.valid;out.reason=sweep.reason;out.handoff_id=g.handoff_id;out.entry_admission_sequence=x.entry_admission.sequence;
    out.entry_curve_pose=x.curve_entry_pose;out.entry_curve_twist=x.curve_entry_twist;out.entry_curve_time=x.curve_time;
    out.entry_position_error_m=pe;out.entry_velocity_error_mps=ve;
    const auto deadline=motionEvidenceDeadline(ns(d.source_stamp),ns(d.body_source_stamp),ns(x.entry_source_stamp),
      ns(d.valid_until),std::min({ns(g.valid_until),ns(g.transition_deadline),ns(g.candidate.valid_until)}),
      ns(proof->valid_until),map.observedProofDeadlineNs(),map.integratedRaySourceStamp(0),map.integratedRaySourceStamp(1),braking_->raySourceAgeNs());
    out.valid_until=rclcpp::Time(std::max<std::int64_t>(0,deadline));
    if(deadline<=ns(out.check_end)||ValidationCycle::Clock::now()>=round_deadline)return;
    {std::lock_guard<std::mutex> l(mutex_);const auto fresh=motionProof(proofs_,d,node_->now().nanoseconds());
      if(!grant_||grant_->handoff_id!=g.handoff_id||grant_retired_||grant_applied_||barrier!=prepared_stop_barrier_||
         !fresh||fresh->sequence!=proof->sequence)return;
      // Fixed signed handoff, maximum 20 Hz prepared checks over its original
      // sub-second deadline. Keep all issued proofs until outcome; ordinary
      // motion heartbeats cannot evict the proof selected by the writer.
      if(prepared_motion_proofs_.size()>=16)return;
      prepared_motion_proofs_.push_back(out);
      motion_proofs_.push_back(out);while(motion_proofs_.size()>32)motion_proofs_.pop_front();}
    motion_pub_->publish(out);
  }
  void checkMotion(GridMap& map,double budget_s=ValidationCycle::motion_budget_s,
      ValidationCycle::Clock::time_point round_deadline=ValidationCycle::Clock::time_point::max()) {
    std::optional<ew::MotionDemand> demand;std::optional<ew::ExecutionPermit> permit,latest;
    std::optional<Job> active;std::shared_ptr<const MotionSupport> support;std::uint64_t barrier;
    std::optional<ew::TrajectoryValidation> proof;
    std::optional<BodySourceSample> measured_state;
    {
      std::lock_guard<std::mutex> l(mutex_);
      if(!latest_demand_||latest_demand_->hold||
         std::abs(latest_demand_->velocity.linear.x)+std::abs(latest_demand_->velocity.angular.z)<1e-12)return;
      demand=latest_demand_;active=active_;latest=latest_permit_;barrier=motion_stop_barrier_;
      if(!body_history_.empty())measured_state=body_history_.back();
      for(auto it=support_history_.rbegin();it!=support_history_.rend();++it)
        if((*it)->source.version==demand->version){support=*it;break;}
      for(auto it=permits_.rbegin();it!=permits_.rend();++it)if(it->sequence==demand->permit_sequence){permit=*it;break;}
      proof=motionProof(proofs_,*demand,node_->now().nanoseconds());
    }
    const auto& d=*demand;
    const auto now=node_->now().nanoseconds();
    // Waiting for fresh same-curve evidence/permission is not a newly discovered
    // collision. Do not publish a false negative against a newer still-safe
    // command. Existing certified demand can live ONLY to its original expiry.
    // A missing dependency can never produce a positive result either.
    if(!permit||!proof||ns(proof->valid_until)<=now||ns(permit->valid_until)<=now)return;
    // Topic scheduling can deliver the command before its independent odom
    // witness. Wait for that acquired state; this is not occupied evidence and
    // does not extend any previously issued command's original expiration.
    if(!measured_state||!measured_state->twist||measured_state->source_ns<ns(d.body_source_stamp)||
       !motionSourceFresh(measured_state->source_ns,now))return;
    ew::MotionValidation out;
    out.version=d.version;out.execution_id=d.execution_id;out.control_epoch=d.control_epoch;
    out.sdk_session=d.sdk_session;out.sdk_arm_generation=d.sdk_arm_generation;out.trajectory_id=d.trajectory_id;
    out.permit_sequence=d.permit_sequence;out.trajectory_validation_sequence=proof->sequence;
    out.demand_sequence=d.sequence;out.demand_source_stamp=d.source_stamp;
    out.demand_valid_until=d.valid_until;out.demand_body_source_stamp=d.body_source_stamp;
    out.sequence=++motion_sequence_;out.velocity=d.velocity;out.frame_id=frame_;out.transport_mode=mode_;
    out.braking_model_sha256=braking_?braking_->sha256:"";
    out.check_begin=node_->now();out.front_ray_source_stamp=rclcpp::Time(map.integratedRaySourceStamp(0));
    out.rear_ray_source_stamp=rclcpp::Time(map.integratedRaySourceStamp(1));
    out.map_snapshot_revision=map.occupancyRevision();out.reason="motion_permission_or_source_unavailable";
    const auto fresh=[&](const builtin_interfaces::msg::Time& t){return motionSourceFresh(ns(t),now);};
    MotionSweepResult sweep;bool queried=false;
    std::int64_t deadline=std::min<std::int64_t>({ns(d.valid_until),ns(d.source_stamp)+100000000LL,ns(d.body_source_stamp)+100000000LL});
    if(braking_&&active&&active->progress&&measured_state&&measured_state->twist&&
       measured_state->source_ns>=ns(d.body_source_stamp)&&motionSourceFresh(measured_state->source_ns,now)&&
       support&&support->source.version==d.version&&
       active->version==d.version&&active->spline.trajectory.traj_id==d.trajectory_id&&
       active->proof&&active->proof->valid&&active->proof->sequence>=d.validation_sequence&&ns(active->proof->valid_until)>now&&proof&&proof->valid&&
       ns(proof->valid_until)>now&&permit&&latest&&demandPermitMatches(d,*permit,proof->sequence)&&demandPermitMatches(d,*latest,proof->sequence)&&
       permit->phase==latest->phase&&(permit->phase!="aligning"||std::abs(d.velocity.linear.x)<1e-12)&&
       ns(permit->valid_until)>now&&ns(latest->valid_until)>now&&
       fresh(d.source_stamp)&&fresh(d.body_source_stamp)&&deadline>now&&
       map.isRawRaySnapshot()&&map.integratedCloudFreshAt(now)&&map.localizationContextSequence()==d.version.context_sequence) {
      // Curve progress carries route identity, not the current safety pose or
      // limiter boundary. Pose and twist come from one latest real odometry.
      const auto& body=measured_state->pose;const auto& position=body.position;
      out.body_source_stamp=rclcpp::Time(measured_state->source_ns);double yaw=0.;
      if(measuredBodyYaw(body,frame_,now*1e-9,.1,yaw)) {
        const double remaining=std::min(budget_s,
          std::chrono::duration<double>(round_deadline-ValidationCycle::Clock::now()).count());
        if(remaining<=0.)return; // CPU expiry is no new free or occupied evidence.
        queried=true;sweep=validateMotionSweep(map,*support,*braking_,position,yaw,*measured_state->twist,d.velocity,frame_,remaining);
        out.valid=sweep.valid;out.reason=sweep.reason;
        deadline=motionEvidenceDeadline(ns(d.source_stamp),ns(d.body_source_stamp),measured_state->source_ns,
          ns(d.valid_until),std::min(ns(permit->valid_until),ns(latest->valid_until)),ns(proof->valid_until),
          map.observedProofDeadlineNs(),map.integratedRaySourceStamp(0),map.integratedRaySourceStamp(1),braking_->raySourceAgeNs());
      }
    }
    out.check_end=node_->now();out.valid_until=rclcpp::Time(std::max<std::int64_t>(0,deadline));
    if(ValidationCycle::Clock::now()>=round_deadline)return;
    if(out.valid&&(!map.integratedCloudFreshAt(ns(out.check_end))||deadline<=ns(out.check_end))) {
      out.valid=false;out.reason="motion_evidence_expired_during_check";
    }
    {
      std::lock_guard<std::mutex> l(mutex_);
      // A later ordinary nonzero request may coexist with this exact proof.
      // A later zero/HOLD/revoke/owner switch may never resurrect old motion.
      const auto final_proof=motionProof(proofs_,d,node_->now().nanoseconds());
      if(!final_proof||final_proof->sequence!=proof->sequence||barrier!=motion_stop_barrier_||!active_||active_->version!=d.version||
         active_->spline.trajectory.traj_id!=d.trajectory_id||!latest_permit_||
         !demandPermitMatches(d,*latest_permit_,proof->sequence)||latest_permit_->phase!=permit->phase||
         !active_->proof||!active_->proof->valid||active_->proof->sequence<d.validation_sequence||
         ns(active_->proof->valid_until)<=node_->now().nanoseconds()||
         ns(latest_permit_->valid_until)<=node_->now().nanoseconds())return;
    }
    if(!out.valid)RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),250,
      "Motion proof demand=%lu visited_unique_voxels=%zu free=%zu occupied=%zu unknown=%zu outside=%zu query_complete=%d length=%.3f yaw_bound=%.3f check_ms=%.3f demand_age_ms=%.3f front_age_ms=%.3f rear_age_ms=%.3f queried=%d reason=%s",
      d.sequence,sweep.unique_voxels,sweep.state_counts[0],sweep.state_counts[1],sweep.state_counts[2],sweep.state_counts[3],
      sweep.query_complete,sweep.length,sweep.heading_bound,(ns(out.check_end)-ns(out.check_begin))*1e-6,
      (ns(out.check_end)-ns(d.source_stamp))*1e-6,(ns(out.check_end)-map.integratedRaySourceStamp(0))*1e-6,
      (ns(out.check_end)-map.integratedRaySourceStamp(1))*1e-6,queried,out.reason.c_str());
    else RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
      "Motion proof demand=%lu snapshot=%lu unique_voxels=%zu check_ms=%.3f source_age_ms=%.3f body_age_ms=%.3f front_age_ms=%.3f rear_age_ms=%.3f lease_ms=%.3f",
      d.sequence,out.map_snapshot_revision,sweep.unique_voxels,(ns(out.check_end)-ns(out.check_begin))*1e-6,
      (ns(out.check_end)-ns(d.source_stamp))*1e-6,(ns(out.check_end)-ns(out.body_source_stamp))*1e-6,
      (ns(out.check_end)-map.integratedRaySourceStamp(0))*1e-6,(ns(out.check_end)-map.integratedRaySourceStamp(1))*1e-6,
      (deadline-ns(out.check_end))*1e-6);
    {std::lock_guard<std::mutex> l(mutex_);motion_proofs_.push_back(out);
      while(motion_proofs_.size()>32)motion_proofs_.pop_front();}
    motion_pub_->publish(out);
  }
  bool check(const Job& job,const MeasuredBodyPose& body,GridMap& map,
      double budget_s=ValidationCycle::slow_curve_budget_s,bool skip_budget_failure=false,
      ValidationCycle::Clock::time_point round_deadline=ValidationCycle::Clock::time_point::max()) {
    const auto check_started=std::chrono::steady_clock::now();
    map.beginObservedProof();
    ew::TrajectoryValidation result;result.version=job.version;result.proposal_id=job.proposal;
    result.trajectory_id=job.spline.trajectory.traj_id;result.sequence=++sequence_;
    result.check_begin=node_->now();result.body_source_stamp=rclcpp::Time(static_cast<std::int64_t>(body.source_stamp*1e9));
    result.front_ray_source_stamp=rclcpp::Time(map.integratedRaySourceStamp(0));
    result.rear_ray_source_stamp=rclcpp::Time(map.integratedRaySourceStamp(1));
    result.map_snapshot_revision=map.occupancyRevision();result.frame_id=frame_;
    // A route-sized geometry check is not an actuation proof. The formal
    // planner uses SCAN's inflated twin-cylinder obstacle model; each actual
    // demand still passes the independent raw observed-free braking sweep.
    const bool official_geometry=geometry_policy_=="official_inflated_geometry";
    result.collision_policy=geometry_policy_;result.transport_mode=mode_;result.whole_curve=true;
    result.reason="missing_fresh_raw_ray_support_or_body_evidence";
    if(job.support) {result.support_reference_id=job.support->support_reference_id;result.support_hash=job.support->support_hash;}
    const auto& s=job.spline.trajectory;
    if(job.support&&job.support->verified&&job.support->version==job.version&&map.isRawRaySnapshot()&&
       map.integratedCloudFreshAt(node_->now().nanoseconds())&&
       (!official_geometry||!map.planningRequiresObservedFree())&&
       map.localizationContextSequence()==job.version.context_sequence&&s.order==3&&s.pos_pts.size()>=4&&
       s.pos_pts.size()<=10000&&s.knots.size()==s.pos_pts.size()+4&&job.spline.frame_id==frame_) {
      Eigen::MatrixXd points(3,s.pos_pts.size());Eigen::VectorXd knots(s.knots.size());
      for(std::size_t i=0;i<s.pos_pts.size();++i)points.col(i)=Eigen::Vector3d(s.pos_pts[i].x,s.pos_pts[i].y,s.pos_pts[i].z);
      for(std::size_t i=0;i<s.knots.size();++i)knots[i]=s.knots[i];
      bool ordered=knots.allFinite();for(int i=1;i<knots.size();++i)ordered=ordered&&knots[i]>knots[i-1];
      if(points.allFinite()&&ordered) {
        UniformBspline curve(points,3,.1);curve.setKnot(knots);
        // The no-motion heading contract validates the complete frozen curve
        // and replays its heading memory to the measured join. A tracker
        // heartbeat must not turn that proof into an incompatible truncated
        // curve, or reset the original heading to the current pose.
        const bool measured_progress=!job.heading.preview_only_enabled&&job.progress&&job.progress->valid&&
          (node_->now()-rclcpp::Time(job.progress->header.stamp)).seconds()<=.4;
        std::optional<MeasuredCurveDomain> domain;
        double join_residual=std::numeric_limits<double>::quiet_NaN();
        if(measured_progress)domain=measuredRemainingCurveDomain(curve,body.position,job.progress->curve_time,
          body.source_stamp-rclcpp::Time(job.progress->header.stamp).seconds(),job.progress->s_committed,.3,.15,
          MeasuredConnectionPolicy::CommittedSweptConnection,&join_residual);
        const auto join=measured_progress?(domain?std::optional<double>(domain->measured_time):std::nullopt):
          measuredPreviewCurveTime(curve,body.position,job.measured_time,map.getResolution());
        bool occupied=false,unknown=false,supported=false;std::size_t queries=0;
        const auto query_begin=std::chrono::steady_clock::now();
        if(join) {
          const auto arc_domain=domain?domain:measuredRemainingCurveDomain(curve,body.position,*join,0.,0.);
          result.valid_start_time=*join;result.valid_start_arc_length=arc_domain?arc_domain->measured_arc:0.;
          result.whole_curve=!domain;result.remaining_curve=domain.has_value();
          result.checked_from_time=domain?domain->checked_from_time:0.;
          result.checked_to_time=result.curve_duration=curve.getTimeSum();
          result.reverse_margin_m=domain?.15:0.;
          supported=measuredConnectionSupported(*job.support,body.position,curve.evaluateDeBoorT(*join),frame_,map.getResolution());
          const double remaining=std::min(
            budget_s-std::chrono::duration<double>(std::chrono::steady_clock::now()-check_started).count(),
            std::chrono::duration<double>(round_deadline-ValidationCycle::Clock::now()).count());
          result.valid=arc_domain.has_value()&&supported&&remaining>0.&&wholeCurveCollisionFree(curve,map.getResolution(),extent_,
            [&](const Eigen::Vector3d& p,double yaw){++queries;const int state=official_geometry?
                map.getPlanningInflateOccupancy(p,yaw):map.getInflateOccupancy(p,yaw);
              occupied=occupied||state==1;unknown=unknown||state==2||state<0;return state;},
            body,frame_,node_->now().seconds(),.4,200000,remaining,*join,job.heading,result.checked_from_time,
            domain?MeasuredConnectionPolicy::CommittedSweptConnection:MeasuredConnectionPolicy::CandidateAdmission);
          if(result.valid&&job.has_goal_yaw&&std::isfinite(job.goal_yaw)&&
             (body.position-job.goal).norm()<=.25) {
            double measured_yaw=0.;
            if(measuredBodyYaw(body,frame_,node_->now().seconds(),.4,measured_yaw)) {
              result.goal_yaw_checked=headingTransitionFree(body.position,body.position,measured_yaw,job.goal_yaw,
                map.getResolution(),extent_,[&](const Eigen::Vector3d& p,double yaw){
                  return ValidationCycle::Clock::now()<round_deadline&&
                    std::chrono::duration<double>(std::chrono::steady_clock::now()-check_started).count()<budget_s&&
                    (official_geometry?map.getPlanningInflateOccupancy(p,yaw):map.getInflateOccupancy(p,yaw))==0;
                });
              if(result.goal_yaw_checked)result.checked_goal_yaw=job.goal_yaw;
            }
          }
        }
        const double query_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-query_begin).count();
        result.reason=result.valid?geometry_policy_:occupied?"occupied":unknown?"unknown_or_unobserved":
          !join?"measured_projection_or_connection_invalid":!supported?"measured_connection_support_invalid":
          (ValidationCycle::Clock::now()>=round_deadline||
           std::chrono::duration<double>(std::chrono::steady_clock::now()-check_started).count()>=budget_s)?
            "curve_check_budget_exhausted":"curve_or_connection_geometry_invalid";
        // check() can run on two independent readers. Do not share a mutable
        // logging-throttle static between them; source leases remain untouched.
        const auto log_now=std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        auto prior_log=proof_log_steady_ns_.load();
        if(!result.valid&&log_now-prior_log>=250000000LL&&
           proof_log_steady_ns_.compare_exchange_strong(prior_log,log_now))RCLCPP_WARN(node_->get_logger(),
          "Execution proof curve=%ld measured_progress=%d residual=%.6f queries=%zu check_ms=%.3f reason=%s",
          result.trajectory_id,measured_progress,join_residual,queries,query_seconds*1000.,result.reason.c_str());
      }
    }
    if(result.valid&&ValidationCycle::Clock::now()>=round_deadline) {
      result.valid=false;result.reason="curve_check_budget_exhausted";
    }
    // The fast lane may run out of scheduling budget. That is neither a fresh
    // collision nor fresh free evidence. Keep the previous proof ONLY until
    // its original expiry; the slow independent reader may renew it normally.
    if(ValidationCycle::inconclusiveRefresh(skip_budget_failure,result.reason)) {
      // Scheduling failure is observable, but cannot manufacture a collision
      // or a new positive lease. In particular an expired prior proof remains
      // expired. The independent fast command sweep still checks real changes.
      bool retained=false;std::int64_t previous_deadline=0;
      const auto now=node_->now().nanoseconds();
      {std::lock_guard<std::mutex> l(mutex_);
        if(active_&&active_->version==job.version&&active_->spline.trajectory.traj_id==result.trajectory_id&&active_->proof) {
          previous_deadline=ns(active_->proof->valid_until);
          retained=ValidationCycle::originalLeaseUsable(active_->proof->valid,previous_deadline,now);
        }
      }
      const auto log_now=std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      auto prior=budget_log_steady_ns_.load();
      if(log_now-prior>=1000000000LL&&budget_log_steady_ns_.compare_exchange_strong(prior,log_now))
        RCLCPP_WARN(node_->get_logger(),"Native current-curve refresh budget exhausted curve=%ld retained_to_original_expiry=%d previous_lease_remaining_ms=%.3f; no new proof issued",
          result.trajectory_id,retained,std::max<std::int64_t>(0,previous_deadline-now)*1e-6);
      return false;
    }
    result.check_end=node_->now();result.source_stamp=result.check_end;
    const auto sensor_age=braking_?braking_->raySourceAgeNs():500000000LL;
    const auto evidence_deadline=official_geometry?std::numeric_limits<std::int64_t>::max():map.observedProofDeadlineNs();
    const auto min_deadline=std::min<std::int64_t>({node_->now().nanoseconds()+250000000LL,
      evidence_deadline,map.integratedRaySourceStamp(0)+sensor_age,
      map.integratedRaySourceStamp(1)+sensor_age,
      static_cast<std::int64_t>(body.source_stamp*1e9)+400000000LL});
    result.valid_until=rclcpp::Time(std::max<std::int64_t>(0,min_deadline));
    // Do not renew a source lease with computation completion.
    if(!map.integratedCloudFreshAt(node_->now().nanoseconds())||node_->now().seconds()-body.source_stamp>.4||
       min_deadline<=node_->now().nanoseconds()) {
      result.valid=false;result.reason="source_expired_during_check";
    }
    // A canceled/replaced job can never publish a late positive proof.
    {std::lock_guard<std::mutex> l(mutex_);
      const auto same=[&](const std::optional<Job>& j){return j&&j->version==job.version&&j->spline.trajectory.traj_id==result.trajectory_id;};
      if(!same(active_)&&!same(candidate_)&&!same(prepared_))return false;
      // A slower earlier check must never resurrect a curve after a later
      // negative proof or overwrite the newest measured-position validation.
      if((same(active_)&&active_->proof&&!proofSupersedes(result,*active_->proof))||
         (same(candidate_)&&candidate_->proof&&!proofSupersedes(result,*candidate_->proof))||
         (same(prepared_)&&prepared_->proof&&!proofSupersedes(result,*prepared_->proof)))return false;
      if(same(active_))active_->proof=result;
      if(same(candidate_))candidate_->proof=result;
      if(same(prepared_))prepared_->proof=result;
      proofs_.push_back(result);while(proofs_.size()>64)proofs_.pop_front();
      if(result.valid) {
        if(same(active_))active_->measured_time=result.valid_start_time;
        if(same(candidate_))candidate_->measured_time=result.valid_start_time;
        if(same(prepared_))prepared_->measured_time=result.valid_start_time;
      }
    }
    pub_->publish(result);
    return result.valid;
  }
  rclcpp::Node*node_;CollisionSnapshotPool&pool_;std::string frame_,mode_;double extent_;
  mutable std::mutex mutex_;std::mutex wait_mutex_;std::condition_variable wake_;bool snapshot_ready_{false};
  std::atomic<bool>stop_{false};std::thread thread_,admission_thread_;
  std::mutex admission_wait_mutex_;std::condition_variable admission_wake_;bool admission_ready_{false};
  // Only dedicated odometry enters this bounded history. A tracker projection
  // cannot attest to itself, and source-time lookup never extrapolates.
  std::optional<BodySourceSample> bodyAtSourceLocked(std::int64_t source_ns)const {
    const auto next=std::lower_bound(body_history_.begin(),body_history_.end(),source_ns,
      [](const BodySourceSample& s,std::int64_t t){return s.source_ns<t;});
    if(next==body_history_.end()||next->pose.frame!=frame_)return {};
    if(next->source_ns==source_ns)return *next;
    if(next==body_history_.begin())return {};
    const auto previous=std::prev(next);const auto gap=next->source_ns-previous->source_ns;
    if(previous->pose.frame!=frame_||gap<=0||gap>100000000LL||!previous->twist||!next->twist)return {};
    const double alpha=static_cast<double>(source_ns-previous->source_ns)/static_cast<double>(gap);
    BodySourceSample result=*previous;result.source_ns=source_ns;
    result.pose.source_stamp=source_ns*1e-9;
    result.pose.position=(1.-alpha)*previous->pose.position+alpha*next->pose.position;
    result.pose.orientation=previous->pose.orientation.normalized().slerp(alpha,next->pose.orientation.normalized());
    auto& t=*result.twist;const auto& a=*previous->twist;const auto& b=*next->twist;
    t.linear.x=(1.-alpha)*a.linear.x+alpha*b.linear.x;
    t.linear.y=(1.-alpha)*a.linear.y+alpha*b.linear.y;
    t.linear.z=(1.-alpha)*a.linear.z+alpha*b.linear.z;
    t.angular.x=(1.-alpha)*a.angular.x+alpha*b.angular.x;
    t.angular.y=(1.-alpha)*a.angular.y+alpha*b.angular.y;
    t.angular.z=(1.-alpha)*a.angular.z+alpha*b.angular.z;
    return result;
  }
  std::deque<BodySourceSample> body_history_;
  MeasuredBodyPose body_;std::optional<Job>active_,candidate_;std::optional<ew::SupportReference>pending_support_;
  // Geometry-only facts: fixed, bounded history for a late first writer ACK.
  // These are not snapshot slots or renewed collision proofs.
  std::deque<Job> committed_jobs_;
  std::optional<ew::ExecutionCommitAck> initial_writer_ack_;
  std::deque<InitialIntent> initial_intents_;
  std::optional<ew::ExecutionVersion> task_domain_;bool task_closed_{false};
  std::optional<ew::ExecutionPermit> writer_domain_;
  std::optional<ew::ExecutionVersion> retired_;
  std::optional<BrakingModel> braking_;
  std::optional<ew::MotionDemand> latest_demand_;
  std::optional<ew::ExecutionPermit> latest_permit_;
  std::deque<ew::ExecutionPermit> permits_;
  std::deque<ew::TrajectoryValidation> proofs_;
  std::deque<ew::MotionValidation> motion_proofs_;
  std::deque<ew::MotionValidation> prepared_motion_proofs_,tombstone_motion_proofs_;
  std::optional<ew::MotionValidation> blocked_entry_;
  std::uint64_t last_blocked_entry_sequence_{0};
  std::optional<ew::TrajectoryAdmission> entry_reseed_;
  std::uint64_t last_entry_admission_sequence_{0};
  std::optional<std::pair<ew::ExecutionVersion,std::int64_t>> last_entry_reseed_,last_terminal_reseed_;
  std::shared_ptr<const MotionSupport> motion_support_;
  std::deque<std::shared_ptr<const MotionSupport>> support_history_;
  std::uint64_t motion_sequence_{0},motion_stop_barrier_{0};
  std::atomic<std::uint64_t> sequence_{0};std::uint64_t permit_sequence_{0};
  std::atomic<std::int64_t> proof_log_steady_ns_{0};
  std::atomic<std::int64_t> budget_log_steady_ns_{0};
  rclcpp::Publisher<ew::TrajectoryValidation>::SharedPtr pub_;
  rclcpp::Publisher<ew::TaggedBspline>::SharedPtr committed_pub_;
  rclcpp::Publisher<ew::MotionValidation>::SharedPtr motion_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pipeline_pub_;
  std::uint64_t last_timing_publication_{0};
  bool writer_handoff_{false},grant_retired_{false},grant_applied_{false},tombstone_retired_{false};
  std::string geometry_policy_{"observed_free"};
  std::optional<Job> prepared_,tombstone_job_;
  std::optional<ew::ExecutionHandoffGrant> grant_,tombstone_grant_;
  std::optional<ew::PreparedMotionDemand> prepared_demand_;
  std::uint64_t writer_commit_sequence_{0},ack_sequence_{0},prepared_stop_barrier_{0};
};
} // namespace scan_planner
