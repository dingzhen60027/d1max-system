#pragma once
#include <plan_env/native_pipeline_health.hpp>
#include <plan_env/collision_snapshot_pool.hpp>
#include <plan_manage/trajectory_collision.hpp>
#include <plan_manage/candidate_adoption.hpp>
#include <bspline_opt/rolling_curve_connection.hpp>
#include <plan_manage/moving_splice.hpp>
#include <plan_manage/support_connection.hpp>
#include <plan_manage/motion_sweep.hpp>
#include <plan_manage/planning_support.hpp>
#include <plan_manage/stair_attitude_scope.hpp>
#include <plan_manage/validation_cycle.hpp>
#include <plan_manage/motion_check_diagnostics.hpp>
#include <plan_manage/bounded_execution_trace.hpp>
#include <d1max_planning_interfaces/bounded_diagnostic_file.hpp>
#include <plan_manage/reference_failure_diagnostics.hpp>
#include <plan_manage/local_navigation_body_input.hpp>
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
#include <d1max_planning_interfaces/msg/local_trajectory_commit.hpp>
#include <d1max_planning_interfaces/msg/task_execution_lease.hpp>
#include <d1max_planning_interfaces/msg/local_motion_command.hpp>
#include <nlohmann/json.hpp>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <deque>
#include <iterator>
#include <type_traits>

namespace scan_planner {
namespace ew=d1max_planning_interfaces::msg;
template<class Reference> inline ew::ExecutionVersion executionVersion(const Reference& p) {
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
// Official geometry and actual commands share the selected native SCAN policy.
// Only an explicitly strict reader needs per-cell observed-FREE expiry.
// Unknown policy names fail closed; the constructor rejects them as well.
inline bool validationReadRequiresObservedSnapshot(const std::string& geometry_policy,
                                                  bool actual_motion_reader,
                                                  const std::string& collision_policy="official") {
  return geometry_policy!="official_inflated_geometry"||
      (actual_motion_reader&&collision_policy!="official");
}
// Independent native-map reader. Fusion and FSM stay serialized on their
// existing writer executor; this thread ONLY accesses immutable native map
// snapshots, value-copied geometry and measured source poses. Never live map.
class ExecutionValidator {
  friend struct ExecutionValidatorTestAccess;
public:
  struct BodySourceSample {
    MeasuredBodyPose pose;
    std::optional<geometry_msgs::msg::Twist> twist;
    std::int64_t source_ns;
    std::optional<LocalBodyIdentity> provider_identity;
    std::int64_t provider_valid_until_ns{0};
  };
public:
  // Passive constructor creates no ROS graph or validator thread. Ledger
  // fixtures may inject facts; native integration fixtures invoke the exact
  // checkAt/checkMotionAt implementations with real GridMap and a test clock.
  struct PassiveLedger {};
  ExecutionValidator(PassiveLedger,CollisionSnapshotPool& pool,std::string frame,std::string mode,
      bool writer_handoff=false,bool planning_only_geometry=false,bool local_follower=false):
    node_(nullptr),pool_(pool),frame_(std::move(frame)),mode_(std::move(mode)),extent_(0.),
    writer_handoff_(writer_handoff),planning_only_geometry_(planning_only_geometry),local_follower_(local_follower) {
    if(planning_only_geometry_&&writer_handoff_)throw std::invalid_argument("preview cannot bind a motion writer");
    if(local_follower_&&(writer_handoff_||planning_only_geometry_))throw std::invalid_argument("local follower cannot bind legacy writer transactions");
  }
  struct CommittedView {ew::TaggedBspline spline;std::optional<ew::LocalPlanDebug> debug;
    std::optional<ew::TrajectoryValidation> proof;std::optional<ew::TrackingProgress> progress;};
  struct CommittedHorizon {
    ew::ExecutionVersion version;
    std::int64_t trajectory_id;
    UniformBspline curve;
    double duration;
    double measured_time;
    std::int64_t measured_source_ns;
    double committed_arc{0.};
  };
  ExecutionValidator(rclcpp::Node* node,CollisionSnapshotPool& pool,std::string frame,
                     std::string mode,double extent,std::optional<BrakingModel> braking={},bool writer_handoff=false,
                     std::string geometry_policy="observed_free",bool planning_only_geometry=false,
                     bool local_follower=false,std::string body_frame="base_link",
                     std::function<NativePipelineHealth()> pipeline_health={},double component_timeout_s=1.5):node_(node),pool_(pool),frame_(std::move(frame)),mode_(std::move(mode)),extent_(extent),braking_(std::move(braking)),writer_handoff_(writer_handoff),planning_only_geometry_(planning_only_geometry),local_follower_(local_follower),body_frame_(std::move(body_frame)),geometry_policy_(std::move(geometry_policy)),pipeline_health_(std::move(pipeline_health)) {
    if(!std::isfinite(component_timeout_s)||component_timeout_s<=0.)throw std::invalid_argument("invalid Native component liveness timeout");
    component_timeout_ns_=static_cast<std::int64_t>(component_timeout_s*1e9);
    if(local_follower_&&(writer_handoff_||planning_only_geometry_))throw std::invalid_argument("local follower cannot bind legacy writer transactions");
    if(planning_only_geometry_&&(writer_handoff_||braking_))throw std::invalid_argument("preview cannot bind actuation evidence");
    if(mode_!="live"&&mode_!="isolated_mock") throw std::invalid_argument("invalid execution transport");
    if(geometry_policy_!="observed_free"&&geometry_policy_!="official_inflated_geometry")
      throw std::invalid_argument("invalid trajectory geometry policy");
    pub_=node_->create_publisher<ew::TrajectoryValidation>("/d1max/live_planning/execution/validation",rclcpp::QoS(2));
    committed_pub_=node_->create_publisher<ew::TaggedBspline>("planning/committed_bspline",rclcpp::QoS(1).transient_local());
    if(local_follower_)selected_candidate_pub_=node_->create_publisher<ew::TaggedBspline>(
      "planning/tagged_bspline",rclcpp::QoS(1).reliable().transient_local());
    if(!local_follower_)motion_pub_=node_->create_publisher<ew::MotionValidation>("/d1max/live_planning/execution/motion_validation",rclcpp::QoS(8));
    if(local_follower_)local_command_pub_=node_->create_publisher<ew::LocalMotionCommand>(
      "/d1max/live_planning/execution/local_command",rclcpp::QoS(1));
    pipeline_pub_=node_->create_publisher<std_msgs::msg::String>("/d1max/live_planning/execution/pipeline_timing",rclcpp::QoS(8));
    motion_diagnostics_pub_=node_->create_publisher<std_msgs::msg::String>(
      "/d1max/live_planning/execution/native_motion_status",rclcpp::QoS(1));
    // Committed geometry is a display input, not the control/commit receipt.
    // A slow reliable UI reader must not hold the execution ledger or its
    // mutually-exclusive body/permit/command callback lane during publish.
    committed_display_group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    committed_display_timer_=node_->create_wall_timer(std::chrono::milliseconds(20),[this] {
      dispatchCommittedDisplay([this](const ew::TaggedBspline& spline){committed_pub_->publish(spline);});
    },committed_display_group_);
    execution_trace_timer_=node_->create_wall_timer(std::chrono::milliseconds(500),[this] {
      // Diagnostics share only the display lane, never the Native collision
      // worker or the mutually-exclusive command/ACK callback group.
      drainExecutionTraces();
    },committed_display_group_);
    if(local_follower_) {
      // Candidate delivery is neither a commit ACK nor a command. Its reliable
      // middleware write may block, so only this independent consumer drains
      // the existing bounded selected slot. No control or collision callback
      // waits for this publication. The process assigns it an exclusive role executor.
      selected_publication_group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
      selected_publication_timer_=node_->create_wall_timer(std::chrono::milliseconds(10),[this] {
        try {publishSelectedCandidate();}
        catch(const std::bad_alloc&) {throw;}
        catch(const std::exception& e) {RCLCPP_ERROR_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
          "Native selected candidate publication failed: %s",e.what());}
      },selected_publication_group_);
    }
    thread_=std::thread([this]{run();});
    admission_thread_=std::thread([this]{runAdmission();});
  }
  ~ExecutionValidator(){
    // The owning executor stops callbacks before destruction, as for all
    // other FSM subscriptions. No additional potentially blocked thread joins.
    if(committed_display_timer_)committed_display_timer_->cancel();
    if(execution_trace_timer_)execution_trace_timer_->cancel();
    if(selected_publication_timer_)selected_publication_timer_->cancel();
    {std::lock_guard<std::mutex> l(mutex_);pending_committed_display_.reset();selected_candidate_publication_.reset();}
    stop_=true;wake_.notify_all();admission_wake_.notify_all();
    if(thread_.joinable())thread_.join();
    if(admission_thread_.joinable())admission_thread_.join();
    // Callbacks have stopped and both producers have joined. Preserve the
    // last partial timer interval without another worker or control-path I/O.
    if(node_)drainExecutionTraces(BoundedExecutionTrace::capacity);
  }
  // Map writer notification only: it shortens the fusion->proof latency and
  // never changes any source stamp, TTL or verdict. The 50 ms scheduling target
  // remains; formal full-curve work has its own bounded completion budget.
  void snapshotPublished(){{std::lock_guard<std::mutex> l(wait_mutex_);snapshot_ready_=true;}wake_.notify_all();}
  rclcpp::CallbackGroup::SharedPtr committedDisplayGroup()const{return committed_display_group_;}
  rclcpp::CallbackGroup::SharedPtr selectedPublicationGroup()const{return selected_publication_group_;}
  // Writer decides whether to refresh the third slot between complete fusion
  // transactions. Admission and solver share that ONE slot exclusively; an
  // unavailable slot is a bounded miss, never a fourth allocation or a copy of
  // a live querying map. Validation slots 0/1 remain reserved for fast checks.
  bool slowSnapshotWanted() const {
    std::lock_guard<std::mutex> l(mutex_);
    return slowSnapshotWantedLocked();
  }
  // This is an advisory request for the third snapshot, not a collision or
  // execution decision. The map writer must never wait behind command
  // publication or another ledger operation before delivering its fast head.
  // An unavailable hint is NOT false: the writer retains one pending retry.
  std::optional<bool> trySlowSnapshotWanted() const {
    std::unique_lock<std::mutex> l(mutex_,std::try_to_lock);
    if(!l.owns_lock()) {++slow_snapshot_hint_misses_;return {};}
    return slowSnapshotWantedLocked();
  }
  std::uint64_t slowSnapshotHintMisses()const {return slow_snapshot_hint_misses_.load();}
private:
  bool slowSnapshotWantedLocked() const {
    if(!local_follower_)return active_.has_value()||candidate_.has_value()||prepared_.has_value();
    const auto& pending=!grant_retired_&&!grant_applied_&&prepared_?prepared_:candidate_;
    const bool distinct=pending&&(!active_||!sameJob(*pending,*active_));
    return ValidationCycle::admissionWorkWanted(local_follower_,active_.has_value(),distinct);
  }
public:
  // A completed map OR a newly published immutable candidate is work for
  // the same bounded reader. Notification does not change snapshot identity,
  // source stamps or proof deadlines, and repeated events coalesce in one bit.
  void admissionWorkPublished(){{std::lock_guard<std::mutex> l(admission_wait_mutex_);admission_ready_=true;}
    admission_wake_.notify_all();}
  void admissionSnapshotPublished(){admissionWorkPublished();}
  void configureLocalBodyInput(const std::string& session,const std::string& map,
      const std::string& child) {
    if(session.empty()||map.empty()||child.empty())throw std::invalid_argument("native local body identity missing");
    std::lock_guard<std::mutex> delivery(command_delivery_mutex_);
    std::lock_guard<std::mutex> lock(mutex_);
    local_body_input_.configure(session,map,frame_,child);
    body_history_.clear();body_={};pending_local_demand_.reset();source_pair_wait_.reset();
    ++motion_stop_barrier_;
  }
  void localBody(const ew::LocalNavigationState& state) {
    bodyWithClock({}, {}, 0,[this]{return node_->now().nanoseconds();},
      [this](const ew::LocalMotionCommand& command){if(local_command_pub_)local_command_pub_->publish(command);},&state);
  }
  void localBodyAt(const ew::LocalNavigationState& state,std::int64_t now,
      const std::function<void(const ew::LocalMotionCommand&)>& emit_command={}) {
    bodyWithClock({}, {}, 0,[=]{return now;},emit_command,&state);
  }
  void body(const MeasuredBodyPose& p,
      const std::optional<geometry_msgs::msg::Twist>& world_twist={},std::int64_t source_ns=0) {
    bodyWithClock(p,world_twist,source_ns,
      [this,&p]{return node_?node_->now().nanoseconds():static_cast<std::int64_t>(std::llround(p.source_stamp*1e9));},
      [this](const ew::LocalMotionCommand& command){if(local_command_pub_)local_command_pub_->publish(command);});
  }
  // Deterministic delivery seam for the production source/command ledger.
  // The acquired source time and callback time remain separate.
  void bodyAt(const MeasuredBodyPose& p,const std::optional<geometry_msgs::msg::Twist>& world_twist,
      std::int64_t source_ns,std::int64_t now,
      const std::function<void(const ew::LocalMotionCommand&)>& emit_command={}) {
    bodyWithClock(p,world_twist,source_ns,[=]{return now;},emit_command);
  }
  // Planning reads the same independent acquired-body inbox as validation.
  // Copy one original pose/twist pair; never extrapolate it, refresh its source
  // time, or turn this scheduling read into a proof/permit/geometry commit.
  // A fresh sample may predate a new goal in the SAME localization domain.
  std::optional<BodySourceSample> latestPlanningBodyAt(const ew::ExecutionVersion& task,
      std::int64_t now,double maximum_age_s)const {
    std::lock_guard<std::mutex> l(mutex_);
    if(task.schema_version!=3||!task_domain_||!sameExecutionTask(*task_domain_,task)||
       !referenceTaskOpenLocked(task)||body_history_.empty()||now<=0||
       !std::isfinite(maximum_age_s)||maximum_age_s<=0.)return {};
    const auto& sample=body_history_.back();
    if(sample.provider_identity&&(!sample.provider_identity->matches(task)||now>=sample.provider_valid_until_ns))return {};
    if(!sample.twist||sample.source_ns<=0||sample.source_ns>now+20000000LL||
       double(now-sample.source_ns)*1e-9>maximum_age_s||
       sample.pose.frame!=frame_||!sample.pose.position.allFinite()||
       !sample.pose.orientation.coeffs().allFinite()||sample.pose.orientation.norm()<1e-6||
       !std::isfinite(sample.pose.source_stamp)||sample.pose.source_stamp<=0.||
       std::abs(sample.pose.source_stamp-double(sample.source_ns)*1e-9)>1e-6)return {};
    const auto& t=*sample.twist;
    for(double value:{t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z})
      if(!std::isfinite(value))return {};
    return sample;
  }
private:
  void bodyWithClock(MeasuredBodyPose p,
      std::optional<geometry_msgs::msg::Twist> world_twist,std::int64_t source_ns,
      const std::function<std::int64_t()>& clock,
      const std::function<void(const ew::LocalMotionCommand&)>& emitter,
      const ew::LocalNavigationState* local=nullptr) {
    if(!local&&(!p.position.allFinite()||!p.orientation.coeffs().allFinite()||p.orientation.norm()<1e-6||
       !std::isfinite(p.source_stamp)||p.source_stamp<=0.||p.frame!=frame_))return;
    if(world_twist)for(double x:{world_twist->linear.x,world_twist->linear.y,world_twist->linear.z,
        world_twist->angular.x,world_twist->angular.y,world_twist->angular.z})if(!std::isfinite(x))return;
    if(!local&&source_ns==0)source_ns=static_cast<std::int64_t>(std::llround(p.source_stamp*1e9));
    if(!local&&source_ns<=0)return;
    CommandBatch commands;
    const auto emit_command=captureCommands(commands,emitter,clock,CommandOrigin::Body);
    bool advanced=false,source_pair_ready=false,source_invalidated=false;
    [&] {std::lock_guard<std::mutex> l(mutex_);
      std::optional<LocalBodyIdentity> provider_identity;
      std::int64_t provider_deadline=0;
      if(local) {
        LocalBodySample sample;
        const auto result=local_body_input_.accept(*local,clock(),sample);
        if(result==LocalNavigationBodyInput::Result::Ignore)return;
        const bool changed=result==LocalNavigationBodyInput::Result::Accepted&&
          !body_history_.empty()&&body_history_.back().provider_identity&&
          !(*body_history_.back().provider_identity==sample.identity);
        if(result==LocalNavigationBodyInput::Result::Invalid||changed) {
          // Source invalidation/epoch change cannot complete a waiting old
          // command. Do not retire the navigation task or manufacture a zero
          // state: the follower/BT keep their existing recovery ownership.
          body_history_.clear();body_={};pending_local_demand_.reset();source_pair_wait_.reset();
          ++motion_stop_barrier_;
          source_invalidated=true;
          if(result==LocalNavigationBodyInput::Result::Invalid)return;
        }
        p=sample.pose;world_twist=sample.world_twist;source_ns=sample.source_ns;
        provider_identity=sample.identity;provider_deadline=sample.valid_until_ns;
      } else if(local_body_input_.enabled()) {
        // Formal control never accepts a reference-manager/FSM relay as an
        // independent state. Legacy bare odometry remains a separate mode.
        return;
      }
      auto at=std::lower_bound(body_history_.begin(),body_history_.end(),source_ns,
        [](const BodySourceSample& s,std::int64_t t){return s.source_ns<t;});
      if(at!=body_history_.end()&&at->source_ns==source_ns) {
        // The FSM and independent inbox can deliver the same real odometry.
        // Duplicate callbacks do not manufacture a new sample or source lease.
        if(world_twist&&!at->twist)at->twist=world_twist;
      } else body_history_.insert(at,BodySourceSample{p,world_twist,source_ns,provider_identity,provider_deadline});
      while(body_history_.size()>64 || (!body_history_.empty()&&
          body_history_.back().source_ns-body_history_.front().source_ns>1000000000LL))body_history_.pop_front();
      if(p.source_stamp>body_.source_stamp) {body_=p;advanced=true;}
      // Independent odometry and the follower command have no cross-topic
      // delivery order. Complete the SAME original command when the real body
      // arrives, rather than waiting for (and possibly dropping) another tick.
      // One latest-only slot, original expiry and the stop barrier bound it.
      if(pending_local_demand_) {
        const auto pending=*pending_local_demand_;
        if(pending.waiting_for_commit) {
          // Body delivery cannot promote candidate geometry. Keep its one
          // original command only until an actual LocalTrajectoryCommit.
          if(!originalLocalDemandFreshLocked(pending.demand,clock())) {
            recordLocalDemandStageLocked(MotionCheckStage::CandidateSourceExpired,pending.demand,clock());
            pending_local_demand_.reset();
          }
        } else {
          pending_local_demand_.reset();
          if(pending.stop_barrier==motion_stop_barrier_&&latest_demand_&&
              pending.demand.execution_id==latest_demand_->execution_id&&
              pending.demand.sequence==latest_demand_->sequence)
            relayOrDeferLocalDemandLocked(pending.demand,clock(),emit_command,true);
        }
      }
      source_pair_ready=consumeSourcePairBodyReadyLocked(clock());
    }();
    // Invalidation is committed before waiting for an already-started send.
    // No stale source/epoch command may remain in flight when it returns.
    deliverCommands(commands,emitter,clock,nullptr,source_invalidated);
    // Keep the condition-variable lock outside the ledger lock. A real source
    // pair completes a previously inconclusive read; it does not certify a
    // curve or change the original body/proof/command expiry.
    if(source_pair_ready)snapshotPublished();
    // Acquired odometry must not wait for a fresh ray snapshot/positive whole
    // curve check. The bounded projection below changes ONLY observation
    // history; no proof, map source, permission or certified seed is renewed.
    if(advanced)observeCommittedPreviewBody(p,source_ns,clock());
  }
public:
  void candidate(const ew::TaggedBspline& spline,const ew::ReferenceProposal& proposal,
      const SplineHeadingContract& frozen_heading={},
      const std::optional<PreviewCandidateGeometryEvidence>& preview_geometry={}) {
    // The solver's direction-confidence geometry contract is part of this
    // exact curve. Copy its ORIGINAL heading seed, not the latest/join pose:
    // the independent reader must reproduce the same swept geometry after
    // measured progress advances. Geometry enables no SDK permission.
    if(frozen_heading.preview_only_enabled&&
       (writer_handoff_||!validPreviewHeadingContract(frozen_heading)))
      throw std::invalid_argument("invalid no-motion candidate heading contract");
    if(frozen_heading.measured_heading_enabled&&!validMeasuredHeadingContract(frozen_heading))
      throw std::invalid_argument("invalid measured-body candidate heading geometry");
    if(local_follower_&&(!spline.body_heading_seed_valid||!measuredHeadingEnabled(frozen_heading)||
       !std::isfinite(spline.body_heading_seed_yaw)||spline.body_heading_sampling_intervals==0||
       spline.body_heading_sampling_intervals>49999||
       spline.body_heading_seed_yaw!=frozen_heading.measured_yaw))
      throw std::invalid_argument("formal candidate frozen body heading seed missing or mismatched");
    if(!frozenHeadingWireMatches(spline.trajectory,frozen_heading))
      throw std::invalid_argument("candidate yaw controls do not match frozen heading geometry");
    if(preview_geometry&&(!planning_only_geometry_||writer_handoff_||
       !preview_geometry->origin.policy.valid()||!preview_geometry->projection.connected()||
       preview_geometry->origin.solve_body.frame!=frame_))
      throw std::invalid_argument("preview candidate geometry cannot be motion admission");
    {
      std::lock_guard<std::mutex> l(mutex_);
      if(!referenceTaskOpenLocked(proposal.version))return;
      if(local_follower_&&((candidate_reference_&&*candidate_reference_!=proposal.version)||
         (task_lease_&&task_lease_->revoked)))return;
      Job incoming{spline,proposal.version,proposal.proposal_id,{},spline.valid_start_time,
        proposal.has_goal_yaw,proposal.goal_yaw,{proposal.goal_position.x,proposal.goal_position.y,proposal.goal_position.z},{},{},{},{},false,{},{},{},0.,0.,{}, {}};
      incoming.heading=frozen_heading;
      incoming.goal_yaw_tolerance=proposal.goal_yaw_tolerance_rad;
      if(local_follower_&&spline.splice.enabled&&(!active_||
          spline.splice.incumbent_version!=active_->version||
          spline.splice.incumbent_trajectory_id!=active_->spline.trajectory.traj_id))return;
      // The manager's connector proof is not this independent reader's proof.
      // Retain only the original acquired solve boundary/policy; do not install
      // its projection as a certified progress seed before our whole check.
      if(preview_geometry)incoming.preview_candidate_origin=preview_geometry->origin;
      // Support arrives on a different topic from its reference. A future
      // reference may be rejected while the current one keeps replanning; the
      // most recently received support is therefore NOT necessarily ours.
      // Reuse only immutable geometry with the exact execution version. Jobs
      // pin their support even after the bounded pending history rolls over.
      if(pending_support_&&pending_support_->version==proposal.version)
        incoming.support=*pending_support_;
      for(const auto* job:{&active_,&prepared_,&candidate_,&queued_candidate_})
        if(!incoming.support&&*job&&(*job)->support&&
           (*job)->support->version==proposal.version)incoming.support=*(*job)->support;
      if(!incoming.support)
        for(auto i=planning_support_history_.rbegin();i!=planning_support_history_.rend();++i)
          if((*i)->valid&&(*i)->source.version==proposal.version) {
            incoming.support=(*i)->source;break;
          }
      for(const auto* job:{&active_,&prepared_,&candidate_,&queued_candidate_})
        if(*job&&(*job)->geometry_support&&(*job)->version==proposal.version)
          incoming.geometry_support=(*job)->geometry_support;
      if(!incoming.geometry_support)
        for(auto i=planning_support_history_.rbegin();i!=planning_support_history_.rend();++i)
          if((*i)->valid&&(*i)->source.version==proposal.version) {
            incoming.geometry_support=*i;break;
          }
      if(!local_follower_)candidate_=std::move(incoming);
      else {
        // Native is the sole selector of formal admission work. Publishing
        // every 100 ms solve directly to the follower used to invalidate a
        // 120 ms in-flight native check (and its preparation worker) forever.
        // Keep one selected candidate and coalesce one newest successor.
        // Neither slot is an accepted curve or a motion permission.
        for(const auto* old:{&active_,&prepared_,&candidate_,&queued_candidate_})
          if(*old&&(*old)->version==incoming.version&&
             (*old)->spline.trajectory.traj_id>=spline.trajectory.traj_id)return;
        const auto now=node_?node_->now().nanoseconds():ns(spline.join_source_stamp);
        const bool selected_owned=candidate_&&(candidate_->spline.splice.enabled||
          (active_&&sameJob(*candidate_,*active_))||(prepared_&&sameJob(*candidate_,*prepared_)));
        const bool released=candidate_&&candidate_->proof&&(!candidate_->proof->valid||
          (candidate_offer_deadline_ns_>0&&now>=candidate_offer_deadline_ns_));
        // Historical future-splice callers already have their own measured-
        // seam coalescing. Preserve that path; the formal current-state path
        // has no future seam and uses this admission selector instead.
        if(spline.splice.enabled||!candidate_||selected_owned||released)selectCandidateLocked(std::move(incoming));
        else {
          queued_candidate_=std::move(incoming);
          BoundedExecutionTrace::Event event;event.kind=BoundedExecutionTrace::Kind::CandidateQueued;
          event.ros_ns=now;event.source_ns=ns(spline.join_source_stamp);
          traceLocked(event,traceIdentity(queued_candidate_));
        }
      }
    }
    // Do not wait up to the next fusion/200 ms maintenance tick while the
    // robot keeps tracking the incumbent and moves away from this new seed.
    admissionWorkPublished();
  }
  void support(const ew::SupportReference& s) {
    // Support is immutable geometry for one exact reference version, not a
    // sensor heartbeat. Duplicate delivery neither rebuilds/pushes indexes
    // nor overwrites an incumbent. A changed support needs a new version.
    const auto already_known=[&] {
      if(!referenceTaskOpenLocked(s.version))return true;
      for(const auto* job:{&active_,&candidate_,&prepared_,&queued_candidate_})
        if(*job&&(*job)->support&&(*job)->support->version==s.version)return true;
      if(pending_support_&&pending_support_->version==s.version)return true;
      for(const auto& previous:support_history_)if(previous->source.version==s.version)return true;
      return false;
    };
    {std::lock_guard<std::mutex> l(mutex_);if(already_known())return;}
    auto indexed=std::make_shared<const MotionSupport>(s);
    auto planning_indexed=std::make_shared<const PlanningSupport>(s);
    std::lock_guard<std::mutex> l(mutex_);if(already_known())return;
    pending_support_=s;
    motion_support_=std::move(indexed);
    support_history_.push_back(motion_support_);while(support_history_.size()>8)support_history_.pop_front();
    planning_support_history_.push_back(planning_indexed);
    while(planning_support_history_.size()>8)planning_support_history_.pop_front();
    if(candidate_&&candidate_->version==s.version)candidate_->support=s;
    if(queued_candidate_&&queued_candidate_->version==s.version)queued_candidate_->support=s;
    if(active_&&active_->version==s.version)active_->support=s;
    if(prepared_&&prepared_->version==s.version&&prepared_->support&&
       prepared_->support->support_hash==s.support_hash)prepared_->support=s;
    for(auto* job:{&active_,&prepared_,&candidate_,&queued_candidate_})
      if(*job&&(*job)->version==s.version&&(*job)->support&&
         (*job)->support->support_hash==s.support_hash)(*job)->geometry_support=planning_indexed;
  }
  void candidateDebug(const ew::LocalPlanDebug& d) {
    std::lock_guard<std::mutex> l(mutex_);
    if(candidate_&&candidate_->version.reference_generation==d.generation&&
       candidate_->spline.trajectory.traj_id==static_cast<std::int64_t>(d.plan_id))candidate_->debug=d;
  }
  std::shared_ptr<const PlanningSupport> planningSupport(const ew::ExecutionVersion& version)const {
    std::lock_guard<std::mutex> l(mutex_);
    if(!referenceTaskOpenLocked(version))return {};
    for(const auto* job:{&active_,&prepared_,&candidate_,&queued_candidate_})
      if(*job&&(*job)->version==version&&(*job)->geometry_support&&
         (*job)->geometry_support->valid)return (*job)->geometry_support;
    for(auto i=planning_support_history_.rbegin();i!=planning_support_history_.rend();++i)
      if((*i)->valid&&(*i)->source.version==version)return *i;
    return {};
  }
  // Display follows the committed support, not a newer uncommitted reference.
  // Return only immutable geometry; never copy curves on each odometry callback
  // or use a visual update to change an execution/collision certificate.
  std::shared_ptr<const PlanningSupport> displayPlanningSupport(
      const ew::ExecutionVersion& pending_version)const {
    std::lock_guard<std::mutex> l(mutex_);
    if(active_&&referenceTaskOpenLocked(active_->version)&&active_->geometry_support&&
       active_->geometry_support->valid)return active_->geometry_support;
    if(!referenceTaskOpenLocked(pending_version))return {};
    for(auto i=planning_support_history_.rbegin();i!=planning_support_history_.rend();++i)
      if((*i)->valid&&(*i)->source.version==pending_version)return *i;
    return {};
  }
  // Immutable goal geometry comes from the proposal whose curve passed the
  // Native whole check and LocalFollower commit. A pending proposal, even in
  // the same task, cannot replace this endpoint before its real commit.
  std::optional<Eigen::Vector3d> committedTaskGoal(const ew::ExecutionVersion& version)const {
    std::lock_guard<std::mutex> l(mutex_);
    if(!local_follower_||task_closed_||!active_||
       !sameExecutionTask(active_->version,version)||!active_->goal.allFinite())return {};
    return active_->goal;
  }
  bool finalEndpointConnectionNeedsReseed(const ew::ExecutionVersion& version,std::int64_t now)const {
    std::lock_guard<std::mutex> l(mutex_);
    if(!local_follower_||task_closed_||!active_||!sameExecutionTask(active_->version,version)||
       body_history_.empty())return false;
    const auto& body=body_history_.back();
    if(body.source_ns<=0||body.source_ns>now+20000000LL||now-body.source_ns>100000000LL)return false;
    const auto& p=active_->progress;
    // A solve is not a motion lease. Keep causal recent same-curve evidence
    // usable when the faster body stream acquires a later sample; the target
    // selector still checks that CURRENT geometry exceeds the connector bound.
    if(p&&p->valid&&p->holding&&ns(p->header.stamp)>0&&ns(p->header.stamp)<=body.source_ns&&
       now-ns(p->header.stamp)<=400000000LL&&
       (p->reason=="local_segment_finished_waiting_replan"||p->reason=="braking_envelope_reentry_required"))return true;
    const auto& proof=active_->proof;
    return proof&&!proof->valid&&proof->version==active_->version&&
      proof->trajectory_id==active_->spline.trajectory.traj_id&&proof->proposal_id==active_->proposal&&
      proof->frame_id==frame_&&proof->transport_mode==mode_&&
      proof->reason=="measured_projection_or_connection_invalid"&&ns(proof->body_source_stamp)>0&&
      ns(proof->body_source_stamp)<=body.source_ns&&now-ns(proof->body_source_stamp)<=400000000LL&&
      curveVerdictCurrent(*proof,now);
  }
  bool measuredGoalSettled(const ew::ExecutionVersion& version,std::int64_t now)const {
    std::lock_guard<std::mutex> l(mutex_);
    if(!local_follower_||task_closed_||!active_||!sameExecutionTask(active_->version,version)||
       body_history_.empty())return false;
    const auto& body=body_history_.back();
    if(!body.twist||now<=0||body.source_ns<=0||body.source_ns>now+20000000LL||
       now-body.source_ns>100000000LL||!body.pose.position.allFinite()||
       !body.pose.orientation.coeffs().allFinite()||!std::isfinite(body.pose.source_stamp)||
       body.pose.source_stamp<=0.||body.pose.frame!=frame_||
       std::abs(body.pose.source_stamp-double(body.source_ns)*1e-9)>1e-6)return false;
    for(double value:{body.twist->linear.x,body.twist->linear.y,body.twist->linear.z,
        body.twist->angular.x,body.twist->angular.y,body.twist->angular.z})
      if(!std::isfinite(value))return false;
    // Arrival cannot shut down the only same-task reseed while this actual
    // sample is outside the committed curve's certified connector domain.
    const auto& proof=active_->proof;
    if(!proof||!proof->valid||proof->version!=active_->version||
       proof->trajectory_id!=active_->spline.trajectory.traj_id||proof->proposal_id!=active_->proposal||
       proof->frame_id!=frame_||proof->transport_mode!=mode_||proof->collision_policy!=geometry_policy_||
       ns(proof->body_source_stamp)<=0||ns(proof->body_source_stamp)>body.source_ns||
       now-ns(proof->body_source_stamp)>400000000LL||!curveVerdictCurrent(*proof,now)||
       (!proof->whole_curve&&!proof->remaining_curve))return false;
    // Completion is an advisory scheduling fact, not a command admission.
    // A later quiet body sample may use the causal fresh proof only while its
    // CURRENT acquired pose still projects inside that certified curve domain
    // and the unchanged 15 cm connector bound. Command relay keeps its own
    // exact original-body/positive-proof binding.
    auto curve=decodeSpliceCurve(active_->spline.trajectory);if(!curve)return false;
    const auto domain=measuredRemainingCurveDomain(*curve,body.pose.position,proof->valid_start_time,
      double(body.source_ns-ns(proof->body_source_stamp))*1e-9,proof->valid_start_arc_length,.3,.15,
      MeasuredConnectionPolicy::CommittedSweptConnection);
    if(!domain||domain->measured_time<proof->checked_from_time-1e-6||
       domain->measured_time>proof->checked_to_time+1e-6)return false;
    // Current shared low-speed thresholds, also used by Native's existing
    // stationary evidence predicate. They are not a physical braking record;
    // LocalFollower separately applies its configured measured thresholds.
    if(std::hypot(body.twist->linear.x,body.twist->linear.y)>.03||
       std::abs(body.twist->angular.z)>.05)return false;
    if(active_->has_goal_yaw) {
      if(!std::isfinite(active_->goal_yaw)||!std::isfinite(active_->goal_yaw_tolerance)||
         active_->goal_yaw_tolerance<.01||active_->goal_yaw_tolerance>.5)return false;
      double yaw=0.;
      if(!measuredBodyYaw(body.pose,frame_,double(now)*1e-9,.1,yaw)||
         std::abs(std::remainder(active_->goal_yaw-yaw,2.*M_PI))>active_->goal_yaw_tolerance)return false;
    }
    return true;
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
    if(local_follower_)return false;
    // Only an exact native nonzero verdict requests a new entry. Signed XY is
    // permitted only by its bound model; a pure lateral command is NOT stop.
    if(proof.valid||proof.reason!="motion_sweep_occupied"||planarDemandIsZero(proof.velocity)||
       ((!braking_||braking_->kinematic_profile==KinematicProfile::ForwardYaw)&&
        (proof.velocity.linear.x<0.||std::abs(proof.velocity.linear.y)>1e-9||
         (proof.velocity.linear.x<=0.&&std::abs(proof.velocity.angular.z)<=1e-9)))||
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
  // An obstacle on the writer-applied curve must reach the planner even when
  // the controller correctly withholds nonzero demands. Waiting for a demand
  // echo here creates a cycle: negative curve proof -> hold -> no blocked
  // demand -> no obstacle-triggered replan. This is only a bounded planning
  // event; the original negative proof remains the execution verdict.
  std::optional<ew::TrajectoryValidation> consumeCommittedCollisionAt(std::int64_t now) {
    std::lock_guard<std::mutex> l(mutex_);
    auto event=std::move(committed_collision_);committed_collision_.reset();
    // Source expiry cannot retract an already observed obstacle. Consume that
    // original bounded event once, without using it as execution authority.
    const bool expiry_follows_obstacle=event&&active_&&active_->proof&&
      !active_->proof->valid&&active_->proof->reason=="source_expired_during_check"&&
      active_->proof->version==event->version&&active_->proof->proposal_id==event->proposal_id&&
      active_->proof->trajectory_id==event->trajectory_id&&active_->proof->splice==event->splice&&
      active_->proof->support_hash==event->support_hash&&
      active_->proof->support_reference_id==event->support_reference_id&&
      active_->proof->frame_id==event->frame_id&&active_->proof->transport_mode==event->transport_mode&&
      active_->proof->collision_policy==event->collision_policy&&
      active_->proof->sequence>=event->sequence&&
      ns(active_->proof->check_end)>=ns(event->check_end)&&
      ns(active_->proof->check_end)<ns(event->valid_until);
    if(!event||!active_||!active_->proof||
       (*active_->proof!=*event&&!expiry_follows_obstacle)||task_closed_||
       !latest_permit_||latest_permit_->revoked||
       !sameExecutionTask(latest_permit_->version,event->version)||
       !committedCollisionMatchesLocked(*event)||ns(event->check_end)<=0||
       ns(event->check_end)>now||!curveVerdictCurrent(*event,now))return {};
    last_committed_collision_=std::make_pair(event->version,event->trajectory_id);
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
    bool actual_body{false};
  };
  bool submitCandidateEntryRejection(const ew::TrajectoryAdmission& a) {
    return submitCandidateEntryRejectionAt(a,node_->now().nanoseconds());
  }
  bool submitCandidateEntryRejectionAt(const ew::TrajectoryAdmission& a,std::int64_t now) {
    std::lock_guard<std::mutex> l(mutex_);
    return submitCandidateEntryRejectionLocked(a,now);
  }
  bool submitCandidateEntryRejectionLocked(const ew::TrajectoryAdmission& a,std::int64_t now) {
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
    if(ns(a.checked_at)>now+20000000LL||now-ns(a.checked_at)>250000000LL||(!local_follower_&&ns(a.valid_until)<=now)||
       ns(a.body_source_stamp)>ns(a.checked_at)+20000000LL||now-ns(a.body_source_stamp)>100000000LL||
       ns(a.body_source_stamp)>now+20000000LL||ns(a.body_source_stamp)<=0)return false;
    const auto proof=std::find_if(proofs_.rbegin(),proofs_.rend(),[&](const auto& p){
      return p.sequence==a.validation_sequence&&p.version==a.version&&p.trajectory_id==a.trajectory_id&&
        p.valid&&p.whole_curve&&!p.remaining_curve&&p.frame_id==frame_&&p.transport_mode==mode_&&
        ns(p.check_end)<=ns(a.checked_at)&&curveVerdictCurrent(p,now);
    });
    if(proof==proofs_.rend()||
       (last_entry_reseed_&&last_entry_reseed_->first==a.version&&last_entry_reseed_->second==a.trajectory_id))return false;
    entry_reseed_=a;return true;
  }
  std::optional<FormalReseedRequest> consumeFormalReseedAt(std::int64_t now,double resolution=.05) {
    std::lock_guard<std::mutex> l(mutex_);
    if(planning_only_geometry_) {
      // A candidate that moved beyond its bounded initial connection can be
      // solved again on the SAME route without a tracker rejection or active
      // permit. Only a real current negative geometry proof supplies this event.
      if(!candidate_||!candidate_->preview_candidate_origin||!candidate_->proof||
         !usablePlanningGeometry(*candidate_)||
         candidate_->support->version!=candidate_->version||
         (latest_permit_&&latest_permit_->revoked)||task_closed_)return {};
      const auto& p=*candidate_->proof;
      if(p.valid||!p.whole_curve||p.remaining_curve||p.version!=candidate_->version||
         p.proposal_id!=candidate_->proposal||p.trajectory_id!=candidate_->spline.trajectory.traj_id||
         p.reason!=candidateAcceptanceReason(CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed)||
         p.support_hash!=candidate_->support->support_hash||
         p.support_reference_id!=candidate_->support->support_reference_id||
         p.frame_id!=frame_||p.transport_mode!=mode_||p.collision_policy!=geometry_policy_||
         ns(p.check_end)<=0||ns(p.check_end)>now+20000000LL||now-ns(p.check_end)>250000000LL||
         ns(p.valid_until)<=now||ns(p.body_source_stamp)<=0||
         ns(p.body_source_stamp)>now+20000000LL||now-ns(p.body_source_stamp)>400000000LL||
         (last_preview_candidate_reseed_&&last_preview_candidate_reseed_->first==candidate_->version&&
          last_preview_candidate_reseed_->second==p.trajectory_id))return {};
      double yaw=0.;if(!measuredBodyYaw(body_,frame_,now*1e-9,.4,yaw)||
          body_.source_stamp*1e9+2000<ns(p.body_source_stamp))return {};
      last_preview_candidate_reseed_=std::make_pair(candidate_->version,p.trajectory_id);
      return FormalReseedRequest{candidate_->version,p.trajectory_id,p.body_source_stamp,p.reason,false};
    }
    if(!active_||!latest_permit_||latest_permit_->revoked) {entry_reseed_.reset();return {};}
    if(entry_reseed_) {
      const auto a=std::move(*entry_reseed_);entry_reseed_.reset();
      if(candidate_&&candidate_->version==a.version&&candidate_->spline.trajectory.traj_id==a.trajectory_id&&
         sameExecutionTask(active_->version,a.version)&&now-ns(a.checked_at)<=250000000LL&&
         ns(a.checked_at)<=now+20000000LL&&(local_follower_||ns(a.valid_until)>now)) {
        last_entry_reseed_=std::make_pair(a.version,a.trajectory_id);
        if(local_follower_)clearCandidatesLocked();
        return FormalReseedRequest{a.version,a.trajectory_id,a.body_source_stamp,"measured_candidate_entry_changed",false,true};
      }
    }
    // Preserve the existing terminal connector semantics. At a measured local
    // endpoint, a connection-negative is part of finishing that endpoint, not
    // an ordinary mid-curve deviation. Repeated polls must not request both.
    const auto& terminal_progress=active_->progress;
    if(terminal_progress&&terminal_progress->valid&&terminal_progress->holding&&
       (terminal_progress->reason=="local_segment_finished_waiting_replan"||
        terminal_progress->reason=="braking_envelope_reentry_required")&&
       ns(terminal_progress->header.stamp)<=now+20000000LL&&
       now-ns(terminal_progress->header.stamp)<=400000000LL) {
      if(last_terminal_reseed_&&last_terminal_reseed_->first==active_->version&&
         last_terminal_reseed_->second==active_->spline.trajectory.traj_id)return {};
      last_terminal_reseed_=std::make_pair(active_->version,active_->spline.trajectory.traj_id);
      return FormalReseedRequest{active_->version,active_->spline.trajectory.traj_id,
        terminal_progress->header.stamp,
        terminal_progress->reason=="braking_envelope_reentry_required"?
          "measured_tracking_braking_boundary_unreachable":"measured_local_horizon_finished",true};
    }
    // Native can discover a lost measured connection BEFORE the follower's
    // next progress update. That negative verdict makes the follower hold, so
    // the !holding deviation path below can no longer schedule its replacement.
    // Consume the actual current-curve evidence directly: this schedules ONE
    // measured-body solve, never turns a negative proof into motion permission.
    const auto& connection_proof=active_->proof;
    if(local_follower_&&!task_closed_&&referenceTaskOpenLocked(active_->version)&&
       connection_proof&&!connection_proof->valid&&
       connection_proof->reason=="measured_projection_or_connection_invalid"&&
       connection_proof->version==active_->version&&
       connection_proof->proposal_id==active_->proposal&&
       connection_proof->trajectory_id==active_->spline.trajectory.traj_id&&
       connection_proof->frame_id==frame_&&connection_proof->transport_mode==mode_&&
       ns(connection_proof->check_end)>0&&ns(connection_proof->check_end)<=now+20000000LL&&
       curveVerdictCurrent(*connection_proof,now)&&
       ns(connection_proof->body_source_stamp)>0&&
       ns(connection_proof->body_source_stamp)<=now+20000000LL&&
       now-ns(connection_proof->body_source_stamp)<=400000000LL&&
       !body_history_.empty()&&body_history_.back().source_ns>=ns(connection_proof->body_source_stamp)&&
       body_history_.back().source_ns<=now+20000000LL&&
       now-body_history_.back().source_ns<=100000000LL&&
       (!last_deviation_reseed_||last_deviation_reseed_->first!=active_->version||
        last_deviation_reseed_->second!=active_->spline.trajectory.traj_id)) {
      last_deviation_reseed_=std::make_pair(active_->version,active_->spline.trajectory.traj_id);
      clearCandidatesLocked();
      return FormalReseedRequest{active_->version,active_->spline.trajectory.traj_id,
        connection_proof->body_source_stamp,"measured_current_connection_changed",false,true};
    }
    // Leaving the old local curve is a reason to solve from the ACTUAL body,
    // not to make the controller return to an obsolete future splice first.
    // This schedules work only: it never clears the incumbent or grants motion.
    if(local_follower_&&active_->progress&&!active_->progress->holding&&
       std::isfinite(resolution)&&resolution>0.) {
      const auto deviation=measuredCurveDeviationLocked(now);
      if(deviation&&*deviation>plannedSpliceEntryDistance(resolution)&&
         (!last_deviation_reseed_||last_deviation_reseed_->first!=active_->version||
          last_deviation_reseed_->second!=active_->spline.trajectory.traj_id)) {
        last_deviation_reseed_=std::make_pair(active_->version,active_->spline.trajectory.traj_id);
        clearCandidatesLocked();
        return FormalReseedRequest{active_->version,active_->spline.trajectory.traj_id,
          active_->progress->header.stamp,"measured_body_left_local_curve",false,true};
      }
    }
    return {};
  }
  std::optional<CommittedView> committedView()const {
    std::lock_guard<std::mutex> l(mutex_);if(!active_)return {};
    return CommittedView{active_->spline,active_->debug,active_->proof,active_->progress};
  }
  struct CommittedDiagnosticKey {
    ew::ExecutionVersion version;std::int64_t trajectory_id;bool accepted;std::string phase;
  };
  // Display scheduling reads identity/status only. A 100 Hz FSM poll must not
  // copy the immutable control points/reference geometry merely to discover
  // that the same diagnostic heartbeat was already sent. This is never used
  // as a collision certificate or command authority.
  std::optional<CommittedDiagnosticKey> committedDiagnosticAt(std::int64_t now,double body_age_s)const {
    std::lock_guard<std::mutex> l(mutex_);if(!active_)return {};
    const auto& p=active_->proof;
    const bool fresh=p&&p->valid&&curveVerdictCurrent(*p,now)&&
      (local_follower_||double(now-ns(p->body_source_stamp))*1e-9<=body_age_s);
    const bool accepted=fresh&&active_->debug.has_value();
    return CommittedDiagnosticKey{active_->version,active_->spline.trajectory.traj_id,accepted,
      accepted?"accepted":p&&!p->valid?p->reason:"waiting_current_validation"};
  }
  struct PendingSpliceDisposition {
    bool keep{false};const char* reason{"no_pending_splice"};std::int64_t trajectory_id{-1};
  };
  // A scheduling hint ONLY for ordinary PERIODIC solves. Publishing another
  // .8 s future tail every .2 s otherwise keeps moving the seam before the
  // follower can reach its .0125 m admission window. Collision/reseed/reference
  // events run first in the FSM; this read grants no motion, changes no Job,
  // and never renews a source, proof or candidate deadline.
  PendingSpliceDisposition pendingSpliceDispositionAt(const ew::ExecutionVersion& expected,
      std::int64_t now,double resolution=.05)const {
    std::lock_guard<std::mutex> l(mutex_);
    const auto& pending=local_prepare_request_&&prepared_?prepared_:candidate_;
    const auto result=[&](bool keep,const char* reason) {
      return PendingSpliceDisposition{keep,reason,pending?pending->spline.trajectory.traj_id:-1};
    };
    if(!local_follower_||planning_only_geometry_||!pending||!pending->spline.splice.enabled)
      return result(false,"no_pending_splice");
    const auto& c=*pending;const auto& splice=c.spline.splice;
    if(now<=0||task_closed_||!referenceTaskOpenLocked(expected)||c.version!=expected||!active_||
       !sameExecutionTask(active_->version,expected)||splice.incumbent_version!=active_->version||
       splice.incumbent_trajectory_id!=active_->spline.trajectory.traj_id||
       active_->version.context_sequence!=expected.context_sequence||
       active_->version.map_geometry_revision!=expected.map_geometry_revision)
      return result(false,"pending_splice_identity_changed");
    if(committed_collision_||blocked_entry_||entry_reseed_||!latest_permit_||latest_permit_->revoked||
       latest_permit_->version!=active_->version||
       latest_permit_->trajectory_id!=active_->spline.trajectory.traj_id)
      return result(false,"pending_splice_incumbent_changed");
    const auto& progress=active_->progress;
    if(!progress||!progress->valid||progress->holding||!std::isfinite(progress->arc_length)||
       !std::isfinite(splice.splice_arc_length)||!std::isfinite(resolution)||resolution<=0.)
      return result(false,"pending_splice_progress_unavailable");
    const auto deviation=measuredCurveDeviationLocked(now);
    if(!deviation||*deviation>plannedSpliceEntryDistance(resolution))
      return result(false,"pending_splice_measured_deviation");
    // Passing the seam along the same tail is not a lateral join error.
    // Reuse Native's original 5 cm / .4 s entry domain, including its unchanged
    // 1.25 cm residual. Pair geometry with this progress source, not a newer
    // inbox pose. This only retains a pending proposal; actual admission still
    // validates P/V, the incumbent prefix, the tail and the original deadlines.
    const double overshoot=progress->arc_length-splice.splice_arc_length;
    if(overshoot>0.) {
      const auto body=bodyAtSourceLocked(ns(progress->header.stamp));
      auto tail=decodeSpliceCurve(c.spline.trajectory);
      if(overshoot>.05||!body||!tail||!measuredSpliceTailEntry(*tail,body->pose.position,resolution))
        return result(false,"pending_splice_missed");
    }
    const auto usable=[&](const Job& job,bool pending) {
      if(!usablePlanningGeometry(job)||!job.proof)return false;
      const auto& p=*job.proof;
      return p.valid&&p.version==job.version&&p.proposal_id==job.proposal&&
        p.trajectory_id==job.spline.trajectory.traj_id&&p.sequence>0&&
        p.frame_id==frame_&&p.transport_mode==mode_&&p.collision_policy==geometry_policy_&&
        p.support_hash==job.support->support_hash&&p.support_reference_id==job.support->support_reference_id&&
        curveVerdictCurrent(p,now)&&ns(p.body_source_stamp)>0&&ns(p.body_source_stamp)<=now+20000000LL&&
        now-ns(p.body_source_stamp)<=400000000LL&&
        (pending?(p.whole_curve&&!p.remaining_curve&&p.splice==splice):(p.whole_curve||p.remaining_curve));
    };
    if(!usable(*active_,false))return result(false,"pending_splice_incumbent_proof_unavailable");
    if(c.proof)return result(usable(c,true),usable(c,true)?
      "pending_splice_waiting_measured_entry":"pending_splice_proof_unavailable");
    // In the formal manager this is updateTrajInfo(candidate, node_->now())
    // at adoption/publication, NOT splice_curve_time or a future body stamp.
    // Allow one existing 400 ms source/admission horizon for the first reader
    // result. Polling or a missing result cannot restart that original budget.
    const auto published=ns(c.spline.trajectory.start_time);
    const bool waiting=published>0&&published<=now+20000000LL&&now-published<400000000LL;
    return result(waiting,waiting?"pending_splice_waiting_first_proof":"pending_splice_first_proof_timeout");
  }
  std::optional<PlannedTrajectorySplice> plannedSpliceAt(const ew::ExecutionVersion& task,
      std::int64_t now,double resolution,double ahead_s=.8)const {
    std::lock_guard<std::mutex> l(mutex_);
    if(!active_||!active_->progress)return {};
    return plannedSpliceLocked(task,now,resolution,ahead_s,*active_->progress);
  }
  // The solver owner can lag the independent progress inbox. Select an
  // ORIGINAL progress sample no newer than its acquired body, rather than
  // combining that old body with a future progress stamp or restamping either.
  std::optional<PlannedTrajectorySplice> plannedSpliceAtBody(const ew::ExecutionVersion& task,
      std::int64_t now,double resolution,const MeasuredBodyPose& actual,
      std::int64_t actual_source_ns,double ahead_s=.8)const {
    std::lock_guard<std::mutex> l(mutex_);
    const auto progress=pairedProgressLocked(task,now,actual,actual_source_ns);
    if(!progress)return {};
    // Pairing an older solver body cannot hide a newer independently measured
    // deviation. If that newer body has not arrived yet, the paired real body
    // below still has to satisfy the unchanged projection and deviation bounds.
    const auto latest_deviation=measuredCurveDeviationLocked(now);
    if(latest_deviation&&*latest_deviation>plannedSpliceEntryDistance(resolution))return {};
    auto splice=plannedSpliceLocked(task,now,resolution,ahead_s,*progress);
    if(!splice)return {};
    double residual=std::numeric_limits<double>::quiet_NaN();
    const auto domain=measuredRemainingCurveDomain(splice->incumbent,actual.position,
      splice->measured_time,(actual_source_ns-splice->progress_source_ns)*1e-9,
      splice->committed_arc,.3,.15,MeasuredConnectionPolicy::CommittedSweptConnection,&residual);
    if(!domain||!std::isfinite(residual)||residual>plannedSpliceEntryDistance(resolution))return {};
    return splice;
  }
private:
  // A future common-prefix tail must remain joinable by the actual follower.
  // Grid resolution is not that entry contract: with 5 cm voxels, a body
  // travelling 2--4 cm beside the old curve could repeatedly reserve tails
  // that the unchanged 1.25 cm measured-entry check can never accept. Use the
  // existing Native candidate admission distance for SCHEDULING only. Beyond
  // it, solve from the acquired body; never loosen entry/collision validation
  // or discard the still-valid incumbent to make the new tail pass.
  static double plannedSpliceEntryDistance(double resolution) {
    return std::min(.0125,resolution*.25);
  }
  std::optional<PlannedTrajectorySplice> plannedSpliceLocked(const ew::ExecutionVersion& task,
      std::int64_t now,double resolution,double ahead_s,const ew::TrackingProgress& progress)const {
    if(!local_follower_||task_closed_||!active_||!sameExecutionTask(active_->version,task)||
        !active_->progress||!active_->progress->valid||active_->progress->holding||
        !active_->proof||!active_->proof->valid||!curveVerdictCurrent(*active_->proof,now)||
        !usablePlanningGeometry(*active_)||!latest_permit_||latest_permit_->revoked||
        !progress.valid||progress.holding||ns(progress.header.stamp)<=0||ns(progress.header.stamp)>now+20000000LL||
        now-ns(progress.header.stamp)>400000000LL||
        !std::isfinite(ahead_s)||ahead_s<=0.||ahead_s>2.||!std::isfinite(resolution)||resolution<=0.)return {};
    auto old=decodeSpliceCurve(active_->spline.trajectory);if(!old)return {};
    const auto deviation=measuredCurveDeviationLocked(now,progress);
    if(!deviation||*deviation>plannedSpliceEntryDistance(resolution))return {};
    const double measured=progress.curve_time,boundary=measured+ahead_s;
    if(!std::isfinite(measured)||measured<0.||boundary>old->getTimeSum()-.05)return {};
    auto velocity=old->getDerivative();auto acceleration=velocity.getDerivative();
    const Eigen::Vector3d p=old->evaluateDeBoorT(boundary),v=velocity.evaluateDeBoorT(boundary),a=acceleration.evaluateDeBoorT(boundary);
    if(!p.allFinite()||!v.allFinite()||!a.allFinite()||v.norm()<.02)return {};
    SplineHeadingContract resolved_heading;
    const auto yaw=immutableHeadingAt(*old,active_->heading,boundary,resolution,&resolved_heading);
    if(!std::isfinite(yaw))return {};
    // P/V/A continuity alone cannot transfer the original backwards-entry
    // heading blend to a new tail whose derivative is already forwards. Until
    // that immutable blend is finished, let the existing FSM solve from the
    // acquired body instead of manufacturing a new heading discontinuity at
    // the seam. This does not hold execution or move the seam into the future.
    if(resolved_heading.forward_entry_profile&&boundary<resolved_heading.forward_entry_end)return {};
    ew::TrajectorySplice w;w.enabled=true;w.incumbent_version=active_->version;
    w.incumbent_trajectory_id=active_->spline.trajectory.traj_id;
    w.measured_progress_stamp=progress.header.stamp;w.splice_curve_time=boundary;
    w.splice_arc_length=curveArcTo(*old,boundary);w.position.x=p.x();w.position.y=p.y();w.position.z=p.z();
    w.velocity.x=v.x();w.velocity.y=v.y();w.velocity.z=v.z();
    w.acceleration.x=a.x();w.acceleration.y=a.y();w.acceleration.z=a.z();w.acceleration_valid=true;
    return PlannedTrajectorySplice{w,std::move(*old),active_->heading,measured,yaw,
      ns(progress.header.stamp),std::max(progress.s_committed,active_->progress->s_committed)};
  }
public:
  // Scheduling reads the follower-selected incumbent, not the optimizer's
  // most recent candidate. A newer reference/anchor within the same task does
  // not move this immutable curve. This snapshot conveys no motion permission
  // and does not refresh progress, collision evidence or any source deadline.
  std::optional<CommittedHorizon> committedHorizonAt(const ew::ExecutionVersion& task,
      std::int64_t now)const {
    std::lock_guard<std::mutex> l(mutex_);
    if(!active_||!active_->progress)return {};
    return committedHorizonLocked(task,now,*active_->progress);
  }
  std::optional<CommittedHorizon> committedHorizonAtBody(const ew::ExecutionVersion& task,
      std::int64_t now,const MeasuredBodyPose& actual,std::int64_t actual_source_ns)const {
    std::lock_guard<std::mutex> l(mutex_);
    const auto progress=pairedProgressLocked(task,now,actual,actual_source_ns);
    return progress?committedHorizonLocked(task,now,*progress):std::nullopt;
  }
private:
  std::optional<CommittedHorizon> committedHorizonLocked(const ew::ExecutionVersion& task,
      std::int64_t now,const ew::TrackingProgress& progress)const {
    if(task_closed_||!active_||!sameExecutionTask(task,active_->version)||
       !progress.valid||now<=0)return {};
    const auto source=ns(progress.header.stamp);
    if(source<=0||source>now+20000000LL||now-source>400000000LL||
       !std::isfinite(progress.curve_time)||progress.curve_time<0.)return {};
    const auto& spline=active_->spline.trajectory;
    if(active_->spline.frame_id!=frame_||spline.order!=3||spline.pos_pts.size()<4||
       spline.pos_pts.size()>10000||spline.knots.size()!=spline.pos_pts.size()+4)return {};
    Eigen::MatrixXd points(3,spline.pos_pts.size());Eigen::VectorXd knots(spline.knots.size());
    for(std::size_t i=0;i<spline.pos_pts.size();++i)
      points.col(i)=Eigen::Vector3d(spline.pos_pts[i].x,spline.pos_pts[i].y,spline.pos_pts[i].z);
    for(std::size_t i=0;i<spline.knots.size();++i)knots[i]=spline.knots[i];
    if(!points.allFinite()||!knots.allFinite())return {};
    for(int i=1;i<knots.size();++i)if(knots[i]<=knots[i-1])return {};
    UniformBspline curve(points,3,.1);curve.setKnot(knots);
    const auto duration=curve.getTimeSum();
    if(!std::isfinite(duration)||duration<=0.||progress.curve_time>duration+.02)return {};
    return CommittedHorizon{active_->version,spline.traj_id,std::move(curve),duration,
      std::min(duration,progress.curve_time),source,
      std::max(progress.s_committed,active_->progress?active_->progress->s_committed:0.)};
  }
public:
  // Scheduling fact only: a stationary body near the beginning of the exact
  // applied curve does not need another nominal solve every second. The fast
  // native readers still refresh collision evidence independently; no source,
  // proof, permit or progress lease is renewed by postponing PERIODIC.
  // Read applied geometry and all its evidence in one lock, never local_data_
  // (which may already contain an unjoined/rejected optimizer candidate).
  bool keepCommittedIdleHorizonAt(const ew::ExecutionVersion& expected,
      std::int64_t now,double maximum_body_age,double advance_limit)const {
    std::lock_guard<std::mutex> l(mutex_);
    if(planning_only_geometry_||task_closed_||now<=0||!active_||active_->version!=expected||
       !usablePlanningGeometry(*active_)||active_->support->version!=expected||
       !active_->proof||!active_->progress||!latest_permit_||
       committed_collision_||blocked_entry_||entry_reseed_||
       (candidate_&&(candidate_->version!=expected||
        candidate_->spline.trajectory.traj_id!=active_->spline.trajectory.traj_id))||
       (grant_&&!grant_retired_&&!grant_applied_)||
       (tombstone_grant_&&!tombstone_retired_)||
       !std::isfinite(maximum_body_age)||maximum_body_age<=0.||
       !std::isfinite(advance_limit)||advance_limit<=0.)return false;
    const auto trajectory=active_->spline.trajectory.traj_id;
    const auto& permit=*latest_permit_;
    const auto& proof=*active_->proof;
    const auto& progress=*active_->progress;
    if(permit.revoked||!permit.geometry_committed||permit.version!=expected||
       permit.trajectory_id!=trajectory||permit.frame_id!=frame_||permit.transport_mode!=mode_||
       ns(permit.source_stamp)<=0||ns(permit.source_stamp)>now+20000000LL||
       now-ns(permit.source_stamp)>750000000LL||ns(permit.valid_until)<=now||
       !proof.valid||(!proof.whole_curve&&!proof.remaining_curve)||proof.sequence==0||
       proof.sequence<permit.validation_sequence||proof.version!=expected||
       proof.proposal_id!=active_->proposal||proof.trajectory_id!=trajectory||
       proof.frame_id!=frame_||proof.transport_mode!=mode_||proof.collision_policy!=geometry_policy_||
       proof.support_hash!=active_->support->support_hash||
       proof.support_reference_id!=active_->support->support_reference_id||
       ns(proof.check_end)<=0||ns(proof.check_end)>now+20000000LL||!curveVerdictCurrent(proof,now)||
       ns(proof.body_source_stamp)<=0||ns(proof.body_source_stamp)>now+20000000LL||
       now-ns(proof.body_source_stamp)>400000000LL)return false;
    // progressAt already admits only this exact curve identity and monotonic
    // real measurements. Recheck freshness here: repeated heartbeats cannot
    // turn old progress into a stationary-body scheduling fact.
    if(!progress.valid||progress.holding||progress.schema_version!=2||
       progress.header.frame_id!=frame_||progress.session_id!=expected.session_id||
       progress.task_id!=expected.task_id||progress.route_id!=expected.route_id||
       progress.route_hash!=expected.route_hash||progress.map_version_id!=expected.map_version_id||
       progress.localization_epoch!=expected.localization_epoch||
       progress.localization_seed_id!=expected.localization_seed_id||
       progress.generation!=expected.reference_generation||progress.segment_id!=expected.segment_id||
       progress.anchor_id!=expected.anchor_id||progress.anchor_revision!=expected.anchor_revision||
       progress.context_sequence!=expected.context_sequence||progress.trajectory_id!=trajectory||
       ns(progress.header.stamp)<=0||ns(progress.header.stamp)>now+20000000LL||
       now-ns(progress.header.stamp)>400000000LL||
       !std::isfinite(progress.curve_time)||progress.curve_time<0.)return false;
    double yaw=0.;
    if(!measuredBodyYaw(body_,frame_,now*1e-9,std::min(.4,maximum_body_age),yaw))return false;
    // Keep the actual header nanoseconds. UNIX seconds in a double lose the
    // low bits; rounding that value back can put a lookup just AFTER the last
    // real sample and incorrectly suppress idle holding in every live tick.
    if(body_history_.empty())return false;
    const auto& sample=body_history_.back();
    if(!sample.twist||sample.pose.source_stamp!=body_.source_stamp||
       sample.pose.frame!=body_.frame||sample.pose.position!=body_.position||
       sample.pose.orientation.coeffs()!=body_.orientation.coeffs()||
       sample.source_ns>now+20000000LL||now-sample.source_ns>400000000LL)return false;
    const auto stopped=[](const geometry_msgs::msg::Twist& twist) {
      const Eigen::Vector3d linear{twist.linear.x,twist.linear.y,twist.linear.z};
      const Eigen::Vector3d angular{twist.angular.x,twist.angular.y,twist.angular.z};
      return linear.allFinite()&&angular.allFinite()&&
        linear.squaredNorm()<.0001&&angular.squaredNorm()<.0001;
    };
    const Eigen::Vector3d measured_progress{progress.pose.position.x,
      progress.pose.position.y,progress.pose.position.z};
    if(!stopped(*sample.twist)||!stopped(progress.twist)||!measured_progress.allFinite()||
       (measured_progress-body_.position).norm()>.0125)return false;
    const auto& spline=active_->spline.trajectory;
    if(active_->spline.frame_id!=frame_||spline.order!=3||spline.pos_pts.size()<4||
       spline.pos_pts.size()>10000||spline.knots.size()!=spline.pos_pts.size()+4)return false;
    Eigen::MatrixXd points(3,spline.pos_pts.size());Eigen::VectorXd knots(spline.knots.size());
    for(std::size_t i=0;i<spline.pos_pts.size();++i)
      points.col(i)=Eigen::Vector3d(spline.pos_pts[i].x,spline.pos_pts[i].y,spline.pos_pts[i].z);
    for(std::size_t i=0;i<spline.knots.size();++i)knots[i]=spline.knots[i];
    if(!points.allFinite()||!knots.allFinite())return false;
    for(int i=1;i<knots.size();++i)if(knots[i]<=knots[i-1])return false;
    UniformBspline curve(points,3,.1);curve.setKnot(knots);
    const double duration=curve.getTimeSum();
    return std::isfinite(duration)&&duration>0.&&progress.curve_time<duration*.25&&
      (body_.position-curve.evaluateDeBoorT(0.)).norm()<advance_limit&&
      (!proof.remaining_curve||
       (std::isfinite(proof.checked_from_time)&&std::isfinite(proof.checked_to_time)&&
        progress.curve_time>=proof.checked_from_time&&progress.curve_time<=proof.checked_to_time));
  }
  // No-motion preview progress is a bounded projection of acquired odometry
  // onto the exact committed geometry. It is not TrackingProgress, an entry
  // admission, a renewed collision lease, or permission to drive the curve.
  // Native whole-curve checks can renew this geometry without a tracker inbox.
  NoMotionPreviewProgress planningOnlyProgress(const ew::ExecutionVersion& expected,
      std::int64_t trajectory,std::uint64_t context,const MeasuredBodyPose& body,
      std::int64_t body_source_ns,std::int64_t now,double resolution)const {
    std::optional<Job> job;std::optional<ew::ExecutionPermit> permit;
    {std::lock_guard<std::mutex> l(mutex_);
      if(writer_handoff_||!planning_only_geometry_)return {};
      job=active_;permit=latest_permit_;}
    if(now<=0||expected.schema_version!=3||!job||!permit||!job->proof||!usablePlanningGeometry(*job)||
       job->version!=expected||job->spline.trajectory.traj_id!=trajectory||
       context==0||context!=expected.context_sequence||job->support->version!=expected||
       permit->version!=expected||permit->trajectory_id!=trajectory||permit->allowed||
       permit->revoked||permit->phase!="preview"||
       permit->frame_id!=frame_||permit->transport_mode!=mode_||
       ns(permit->source_stamp)<=0||ns(permit->source_stamp)>now+20000000LL||
       now-ns(permit->source_stamp)>750000000LL||
       ns(permit->valid_until)<=now)return {};
    const auto& proof=*job->proof;
    if(!proof.whole_curve||proof.remaining_curve||proof.sequence==0||
       proof.version!=expected||proof.proposal_id!=job->proposal||
       proof.support_reference_id!=job->support->support_reference_id||
       proof.support_hash!=job->support->support_hash||
       proof.trajectory_id!=trajectory||proof.frame_id!=frame_||proof.transport_mode!=mode_||
       proof.collision_policy!=geometry_policy_||
       ns(proof.check_end)<=0||ns(proof.check_end)>now+20000000LL||
       ns(proof.body_source_stamp)<=0||ns(proof.body_source_stamp)>now+20000000LL||
       body_source_ns<=0||
       body_source_ns+2000<ns(proof.body_source_stamp)||body_source_ns>now+20000000LL||
       now-body_source_ns>400000000LL||
       std::abs(body.source_stamp-body_source_ns*1e-9)>1e-6)return {};
    double yaw=0.;
    if(!measuredBodyYaw(body,frame_,now*1e-9,.4,yaw))return {};
    const auto& spline=job->spline.trajectory;
    if(spline.order!=3||spline.pos_pts.size()<4||spline.pos_pts.size()>10000||
       spline.knots.size()!=spline.pos_pts.size()+4||job->spline.frame_id!=frame_)return {};
    Eigen::MatrixXd points(3,spline.pos_pts.size());Eigen::VectorXd knots(spline.knots.size());
    for(std::size_t i=0;i<spline.pos_pts.size();++i)
      points.col(i)=Eigen::Vector3d(spline.pos_pts[i].x,spline.pos_pts[i].y,spline.pos_pts[i].z);
    for(std::size_t i=0;i<spline.knots.size();++i)knots[i]=spline.knots[i];
    if(!points.allFinite()||!knots.allFinite())return {};
    for(int i=1;i<knots.size();++i)if(knots[i]<=knots[i-1])return {};
    UniformBspline curve(points,3,.1);curve.setKnot(knots);
    if(!job->no_motion_preview||!job->preview_progress)return {};
    const auto& seed=job->observed_preview_progress?*job->observed_preview_progress:*job->preview_progress;
    auto projected=projectCommittedNoMotionPreview(curve,body.position,body.source_stamp,seed,resolution);
    if(projected.status==NoMotionPreviewStatus::Uncertified)return {};
    if(!proof.valid&&projected.status!=NoMotionPreviewStatus::ReplanFromMeasuredBody) {
      if(proof.reason=="preview_connection_out_of_range"||proof.reason=="occupied"||
         proof.reason=="measured_connection_support_invalid")
        projected.status=NoMotionPreviewStatus::ReplanFromMeasuredBody;
      else return {};
    }
    // An out-of-range acquired pose requests a fresh solve; it makes no safe
    // geometry claim and therefore cannot borrow an expired positive lease.
    const bool needs_replan=projected.status==NoMotionPreviewStatus::ReplanFromMeasuredBody;
    if(!needs_replan&&(!permit->geometry_committed||proof.sequence<permit->validation_sequence||
       ns(proof.valid_until)<=now||now-ns(proof.body_source_stamp)>400000000LL))return {};
    // Projection consumes a bounded amount of time. A revoke, a newer negative
    // certificate, or original source expiry wins over this read-only result.
    const auto end_now=node_?node_->now().nanoseconds():now;
    std::lock_guard<std::mutex> l(mutex_);
    if(!previewReadStillCurrentLocked(*job,*permit,body_source_ns,now,end_now,needs_replan))return {};
    return projected;
  }
  std::optional<double> planningOnlyCurveTime(const ew::ExecutionVersion& expected,
      std::int64_t trajectory,std::uint64_t context,const MeasuredBodyPose& body,
      std::int64_t body_source_ns,std::int64_t now,double resolution)const {
    const auto progress=planningOnlyProgress(expected,trajectory,context,body,body_source_ns,now,resolution);
    return progress.connected()?std::optional<double>(progress.curve_time):std::nullopt;
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
       view->progress->holding||!curveVerdictCurrent(*view->proof,static_cast<std::int64_t>(now*1e9))||
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
    active_->progress_history.push_back(p);
    while(active_->progress_history.size()>64 ||
        ns(p.header.stamp)-ns(active_->progress_history.front().header.stamp)>1000000000LL)
      active_->progress_history.pop_front();
  }
  bool commit(const ew::ExecutionPermit& p) {
    std::lock_guard<std::mutex> delivery(command_delivery_mutex_);
    std::lock_guard<std::mutex> l(mutex_);
    if(planning_only_geometry_&&(p.allowed||(!p.revoked&&p.geometry_committed&&p.phase!="preview")))return false;
    if(p.transport_mode!=mode_||p.sequence<=permit_sequence_)return false;
    if(!enterTaskDomainLocked(p))return false;
    if(!local_follower_&&!enterWriterDomainLocked(p))return false;
    // Once the writer has acknowledged its applied identity, only a writer
    // commit ACK can replace it. A reference-generation comparison adds no
    // authority: an arbitrary newer reference is still only a candidate.
    if(writer_handoff_&&writer_commit_sequence_&&active_&&
       (p.version!=active_->version||p.trajectory_id!=active_->spline.trajectory.traj_id)&&!p.revoked)return false;
    latest_permit_=p;permits_.push_back(p);while(permits_.size()>32)permits_.pop_front();
    if(p.revoked){retired_=p.version;task_closed_=true;active_.reset();local_transition_incumbent_.reset();pending_committed_display_.reset();clearCandidatesLocked();if(grant_)grant_retired_=true;
      if(local_follower_){prepared_.reset();local_prepare_request_.reset();local_commit_fact_.reset();}
      task_lease_history_.clear();latest_demand_.reset();pending_local_demand_.reset();
      ++motion_stop_barrier_;permit_sequence_=p.sequence;return true;}
    if(local_follower_){permit_sequence_=p.sequence;return true;} // Geometry follows only LocalTrajectoryCommit.
    if(!p.geometry_committed){permit_sequence_=p.sequence;return false;}
    if(active_&&active_->version==p.version&&active_->spline.trajectory.traj_id==p.trajectory_id) {permit_sequence_=p.sequence;return true;}
    if(!candidate_||candidate_->version!=p.version||candidate_->spline.trajectory.traj_id!=p.trajectory_id)return false;
    active_=candidate_;active_->no_motion_preview=planning_only_geometry_;permit_sequence_=p.sequence;
    committed_jobs_.push_back(*active_);while(committed_jobs_.size()>8)committed_jobs_.pop_front();
    // A read-only durable geometry view for late UI subscribers. This is not
    // the candidate/admission channel; preserve original spline/source stamps.
    queueCommittedDisplayLocked();
    return true;
  }
  bool taskLeaseAt(const ew::TaskExecutionLease& p,std::int64_t now) {
    std::lock_guard<std::mutex> delivery(command_delivery_mutex_);
    std::lock_guard<std::mutex> l(mutex_);
    if(!local_follower_||p.schema_version!=1||p.transport_mode!=mode_||p.sequence<=task_lease_sequence_||
        !task_domain_||!sameExecutionTask(*task_domain_,p.version)||task_closed_||
        ns(p.source_stamp)<=0||ns(p.source_stamp)>now+20000000LL||
        ns(p.valid_until)<=now||now-ns(p.source_stamp)>750000000LL)return false;
    if(p.allowed&&(p.execution_id.empty()||p.control_epoch==0||p.confirmation_id.empty()||
        p.sdk_session.empty()||p.sdk_arm_generation==0))return false;
    if(task_lease_&&!task_lease_->execution_id.empty()) {
      const auto& old=*task_lease_;
      const bool same_execution=p.execution_id==old.execution_id&&p.control_epoch==old.control_epoch;
      const bool first_binding=!old.allowed&&!old.revoked&&old.sdk_arm_generation==0&&
        (old.sdk_session.empty()||old.sdk_session==p.sdk_session)&&!p.sdk_session.empty()&&p.sdk_arm_generation>0;
      if(!same_execution||(!first_binding&&
          (p.sdk_session!=old.sdk_session||p.sdk_arm_generation!=old.sdk_arm_generation)))return false;
    }
    task_lease_=p;task_lease_sequence_=p.sequence;
    // The direct relay has no cached-command sender. Retain the latest demand
    // only as a sequence high-water/diagnostic fact across HOLD and revocation;
    // clearing it could replay an earlier command after a same-stamp resume.
    if(!p.allowed||p.revoked){task_lease_history_.clear();pending_local_demand_.reset();
      local_transition_incumbent_.reset();++motion_stop_barrier_;}
    if(p.revoked){pending_committed_display_.reset();clearCandidatesLocked();prepared_.reset();local_prepare_request_.reset();local_commit_fact_.reset();}
    else if(p.allowed) {task_lease_history_.push_back(p);while(task_lease_history_.size()>8)task_lease_history_.pop_front();}
    return true;
  }
  bool taskLease(const ew::TaskExecutionLease& p){return taskLeaseAt(p,node_->now().nanoseconds());}
  // Durable software geometry fact, not a renewed actuation lease. The exact
  // original Native proof must have been valid AT commit, while current motion
  // continues to require newly acquired body/rays and positive Native checks.
  bool localCommitAt(const ew::LocalTrajectoryCommit& c,std::int64_t now,
      const std::function<void(const ew::LocalMotionCommand&)>& emitter={}) {
    CommandBatch commands;const auto emit_command=captureCommands(commands,emitter,[=]{return now;},CommandOrigin::Commit);
    std::unique_lock<std::mutex> delivery(command_delivery_mutex_);
    const bool result=[&] {
    std::lock_guard<std::mutex> l(mutex_);
    // Commit, demand and body are separate topics. Preserve either the exact
    // candidate's first demand or the exact retained predecessor's deferred
    // body/proof delivery. Neither command changes identity or source stamp.
    const auto pending=pending_local_demand_;
    const bool completes=pending&&pending->stop_barrier==motion_stop_barrier_&&
      ((pending->waiting_for_commit&&pending->demand.version==c.version&&pending->demand.trajectory_id==c.trajectory_id)||
       (!pending->waiting_for_commit&&pending->demand.version==c.expected_version&&
        pending->demand.trajectory_id==c.expected_trajectory_id))&&latest_demand_&&
      pending->demand.execution_id==latest_demand_->execution_id&&pending->demand.sequence==latest_demand_->sequence;
    if(!localCommitLocked(c,now))return false;
    if(completes) {
      relayOrDeferLocalDemandLocked(pending->demand,now,emit_command);
    }
    return true;
    }();
    deliverCommands(commands,emitter,[=]{return now;},&delivery);
    return result;
  }
private:
  bool localCommitLocked(const ew::LocalTrajectoryCommit& c,std::int64_t now) {
    const auto& selected=prepared_&&local_prepare_request_&&
      local_prepare_request_->commit_sequence==c.commit_sequence&&prepared_->version==c.version&&
      prepared_->spline.trajectory.traj_id==c.trajectory_id?prepared_:candidate_;
    if(!local_follower_||c.schema_version!=1||c.transport_mode!=mode_||!c.accepted||c.retired||
        c.disposition!="committed"||c.commit_sequence<=local_commit_sequence_||task_closed_||
        (task_lease_&&task_lease_->revoked)||
        !task_domain_||!sameExecutionTask(*task_domain_,c.version)||!selected||
        selected->version!=c.version||selected->spline.trajectory.traj_id!=c.trajectory_id||
        selected->proposal!=c.proposal_id||ns(c.source_stamp)<=0||ns(c.source_stamp)>now+20000000LL||
        ns(c.body_source_stamp)<=0||ns(c.body_source_stamp)>ns(c.source_stamp)+20000000LL||
        ns(c.source_stamp)-ns(c.body_source_stamp)>100000000LL)return false;
    if(active_?(c.expected_version!=active_->version||c.expected_trajectory_id!=active_->spline.trajectory.traj_id):
        (!c.expected_version.task_id.empty()||c.expected_trajectory_id!=-1))return false;
    const auto proof=std::find_if(local_proof_facts_.begin(),local_proof_facts_.end(),[&](const auto& p){
      return p.sequence==c.native_validation_sequence&&p.version==c.version&&p.trajectory_id==c.trajectory_id&&
        p.proposal_id==c.proposal_id&&p.valid&&p.whole_curve&&p.map_snapshot_revision==c.map_geometry_revision&&
        ns(p.check_end)<=ns(c.source_stamp)+20000000LL&&curveVerdictCurrent(p,ns(c.source_stamp));});
    if(proof==local_proof_facts_.end())return false;
    const Eigen::Vector3d point{c.measured_pose.position.x,c.measured_pose.position.y,c.measured_pose.position.z};
    const Eigen::Vector3d velocity{c.measured_twist.linear.x,c.measured_twist.linear.y,c.measured_twist.linear.z};
    Eigen::Quaterniond orientation(c.measured_pose.orientation.w,c.measured_pose.orientation.x,
      c.measured_pose.orientation.y,c.measured_pose.orientation.z);
    if(!point.allFinite()||!velocity.allFinite()||!orientation.coeffs().allFinite()||orientation.norm()<1e-6||
        !std::isfinite(c.curve_time)||c.curve_time<0.||!std::isfinite(c.arc_length)||c.arc_length<0.)return false;
    auto curve=decodeSpliceCurve(selected->spline.trajectory);if(!curve||c.curve_time>curve->getTimeSum())return false;
    if(rollingCandidateConnection(*selected)) {
      // This is the follower's geometry receipt, not a collision certificate.
      // Match the same bounded XYZ domain checked by Native; measured velocity
      // need not equal the spline derivative after asynchronous computation.
      // The actual raw sample, current Native verdict and original command
      // deadlines below remain independent evidence for sending motion.
      if(!rollingCommitDomain(*selected,*curve,*proof,c,point))return false;
    } else {
      auto derivative=curve->getDerivative();
      if((point-curve->evaluateDeBoorT(c.curve_time)).norm()>.0125||
          (velocity-derivative.evaluateDeBoorT(c.curve_time)).norm()>.05)return false;
    }
    if(selected->spline.splice.enabled&&(!active_||
        selected->spline.splice.incumbent_version!=active_->version||
        selected->spline.splice.incumbent_trajectory_id!=active_->spline.trajectory.traj_id||
        proof->splice!=selected->spline.splice))return false;
    // LocalFollower is the only committed-geometry owner. This immutable
    // software fact records its actual LocalState join and original Native
    // whole-curve proof; it is NOT a motion permission or a body measurement.
    // The independent body topic may miss this historical stamp. Missing or
    // interpolated evidence cannot become a second geometry-admission gate,
    // nor may this fact be copied into body_ or manufacture a fresh proof.
    // If we have the exact RAW sample, however, contradictory facts are still
    // rejected. The local command relay separately requires current acquired
    // body, exact active Native proof and original command/state deadlines.
    const auto body=std::lower_bound(body_history_.begin(),body_history_.end(),ns(c.body_source_stamp),
      [](const BodySourceSample& s,std::int64_t t){return s.source_ns<t;});
    if(body!=body_history_.end()&&body->source_ns==ns(c.body_source_stamp)) {
      if((point-body->pose.position).norm()>1e-6||
          orientation.normalized().angularDistance(body->pose.orientation.normalized())>1e-6)return false;
      if(body->twist&&(velocity-Eigen::Vector3d(body->twist->linear.x,
          body->twist->linear.y,body->twist->linear.z)).norm()>1e-6)return false;
    }
    // The final ACK and incumbent demands cross different transport queues.
    // Retain exactly this committed predecessor until the follower emits its
    // first successor demand. It keeps its own immutable curve/proof/source
    // deadlines; it is never relabeled as the new curve or given a new TTL.
    local_transition_incumbent_=active_;
    active_=selected;local_commit_sequence_=c.commit_sequence;active_->no_motion_preview=false;
    // Do not replace a newer invalid verdict with the historical positive used
    // by the follower. The historical fact can observe geometry, never motion.
    queueCommittedDisplayLocked();
    pending_local_demand_.reset();++motion_stop_barrier_;return true;
  }
public:
  bool localCommit(const ew::LocalTrajectoryCommit& c){return localCommitAt(c,node_->now().nanoseconds(),
    [this](const ew::LocalMotionCommand& command){if(local_command_pub_)local_command_pub_->publish(command);});}
  // One bounded geometry reservation closes the asynchronous candidate/commit
  // race. Preparing changes no active curve or motion permission. The follower
  // still rechecks the actual measured join when it emits the final commit.
  ew::LocalTrajectoryCommit localCommitExchangeAt(const ew::LocalTrajectoryCommit& c,std::int64_t now,
      const std::function<void(const ew::LocalMotionCommand&)>& emitter={}) {
    CommandBatch commands;const auto emit_command=captureCommands(commands,emitter,[=]{return now;},CommandOrigin::Commit);
    std::unique_lock<std::mutex> delivery(command_delivery_mutex_);
    const auto reply=[&]() {
    std::lock_guard<std::mutex> l(mutex_);
    const auto trace_begin=BoundedExecutionTrace::steadyNowNs();
    const auto trace_before=execution_trace_.capture(traceContextLocked(
      {&c.version,c.trajectory_id,c.proposal_id},{&c.expected_version,c.expected_trajectory_id,{}}));
    const auto decide=[&]() {
    auto result=c;
    const auto reject=[&](const char* reason) {
      result.accepted=false;result.disposition="rejected";result.reason=reason;return result;
    };
    if(!local_follower_||c.schema_version!=1||c.transport_mode!=mode_||task_closed_||
       !task_domain_||!sameExecutionTask(*task_domain_,c.version)||c.retired||
       (task_lease_&&task_lease_->revoked)||c.commit_sequence==0)
      return reject("local_commit_task_unavailable");
    const auto same_reservation=[&] {
      return local_prepare_request_&&c.commit_sequence==local_prepare_request_->commit_sequence&&
        c.version==local_prepare_request_->version&&c.trajectory_id==local_prepare_request_->trajectory_id&&
        c.proposal_id==local_prepare_request_->proposal_id&&c.expected_version==local_prepare_request_->expected_version&&
        c.expected_trajectory_id==local_prepare_request_->expected_trajectory_id;
    };
    if(c.disposition=="abort") {
      if(same_reservation()) {
        // Formal LocalFollower uses the reserved transaction channel, not
        // the legacy admission topic. Entry mismatch requests another solve;
        // it never revokes the incumbent or supplies motion permission.
        if(prepared_&&c.source_stamp!=local_prepare_request_->source_stamp&&
           (c.reason=="entry_measured_velocity_off_candidate"||c.reason=="entry_measured_body_not_on_candidate")) {
          ew::TrajectoryAdmission a;a.version=c.version;a.trajectory_id=c.trajectory_id;
          a.transport_mode=c.transport_mode;a.sequence=last_entry_admission_sequence_+1;
          a.checked_at=c.source_stamp;a.body_source_stamp=c.body_source_stamp;
          a.validation_sequence=c.native_validation_sequence;a.reason="candidate:"+c.reason;
          const auto proof=std::find_if(proofs_.rbegin(),proofs_.rend(),[&](const auto& p){
            return p.sequence==c.native_validation_sequence&&p.version==c.version&&p.trajectory_id==c.trajectory_id;
          });
          if(proof!=proofs_.rend()) {a.valid_until=proof->valid_until;submitCandidateEntryRejectionLocked(a,now);}
        }
        const bool selected_reserved=candidate_&&prepared_&&sameJob(*candidate_,*prepared_);
        prepared_.reset();local_prepare_request_.reset();
        if(selected_reserved){candidate_offer_deadline_ns_=now;advanceCandidateLocked();}
      }
      else if(c.commit_sequence>local_prepare_floor_)return reject("local_abort_reservation_missing");
      result.accepted=true;result.disposition="aborted";result.reason="local_candidate_released";return result;
    }
    if(c.disposition=="prepare") {
      if(same_reservation()) {
        if(c!=*local_prepare_request_)return reject("local_prepare_content_changed");
        result.accepted=true;result.disposition="prepared";result.reason="local_candidate_reserved";return result;
      }
      if(c.commit_sequence<=local_prepare_floor_||c.commit_sequence<=local_commit_sequence_||
         c.accepted||ns(c.source_stamp)<=0||ns(c.source_stamp)>now+20000000LL||
         now-ns(c.source_stamp)>400000000LL)
        return reject("local_prepare_order_or_source_invalid");
      if(active_?(c.expected_version!=active_->version||c.expected_trajectory_id!=active_->spline.trajectory.traj_id):
          (!c.expected_version.task_id.empty()||c.expected_trajectory_id!=-1))
        return reject("local_prepare_incumbent_changed");
      if(!candidate_||candidate_->version!=c.version||candidate_->spline.trajectory.traj_id!=c.trajectory_id||
         candidate_->proposal!=c.proposal_id||!usablePlanningGeometry(*candidate_))
        return reject("local_prepare_candidate_superseded");
      const auto proof=std::find_if(local_proof_facts_.begin(),local_proof_facts_.end(),[&](const auto& p) {
        return p.sequence==c.native_validation_sequence&&p.version==c.version&&p.trajectory_id==c.trajectory_id&&
          p.proposal_id==c.proposal_id&&p.valid&&p.whole_curve&&p.map_snapshot_revision==c.map_geometry_revision&&
          ns(p.check_end)<=ns(c.source_stamp)+20000000LL&&curveVerdictCurrent(p,ns(c.source_stamp));
      });
      if(proof==local_proof_facts_.end())return reject("local_prepare_original_proof_missing");
      prepared_=candidate_;local_prepare_request_=c;local_prepare_floor_=c.commit_sequence;
      advanceCandidateLocked();
      result.accepted=true;result.disposition="prepared";result.reason="local_candidate_reserved";return result;
    }
    if(c.disposition!="committed")return reject("local_commit_disposition_invalid");
    if(local_commit_fact_&&c==*local_commit_fact_) {
      result.accepted=true;result.disposition="committed";result.reason="local_geometry_synchronized";return result;
    }
    if(!same_reservation()||!prepared_)return reject("local_commit_reservation_missing");
    const auto pending=pending_local_demand_;
    const bool completes=pending&&pending->stop_barrier==motion_stop_barrier_&&
      ((pending->waiting_for_commit&&pending->demand.version==c.version&&pending->demand.trajectory_id==c.trajectory_id)||
       (!pending->waiting_for_commit&&pending->demand.version==c.expected_version&&
        pending->demand.trajectory_id==c.expected_trajectory_id))&&latest_demand_&&
      pending->demand.execution_id==latest_demand_->execution_id&&pending->demand.sequence==latest_demand_->sequence;
    if(!localCommitLocked(c,now)) {
      if(candidate_&&prepared_&&sameJob(*candidate_,*prepared_))candidate_offer_deadline_ns_=now;
      prepared_.reset();local_prepare_request_.reset();
      return reject("local_commit_measured_join_rejected");
    }
    local_commit_fact_=c;prepared_.reset();local_prepare_request_.reset();
    if(completes) {
      relayOrDeferLocalDemandLocked(pending->demand,now,emit_command);
    }
    result.accepted=true;result.disposition="committed";result.reason="local_geometry_synchronized";return result;
    }();
    traceCommitLocked(decide,now,trace_begin,trace_before);
    return decide;
    }();
    deliverCommands(commands,emitter,[=]{return now;},&delivery);
    // Return the decided ACK without waiting for candidate publication. Its
    // independent single-slot consumer can deliver a successor before/after
    // this ACK; the reserved predecessor remains owned by this exchange.
    if(reply.accepted&&reply.disposition=="prepared")admissionWorkPublished();
    if(reply.accepted&&reply.disposition=="committed")snapshotPublished();
    return reply;
  }
  ew::LocalTrajectoryCommit localCommitExchange(const ew::LocalTrajectoryCommit& c) {
    return localCommitExchangeAt(c,node_->now().nanoseconds(),
      [this](const ew::LocalMotionCommand& command){if(local_command_pub_)local_command_pub_->publish(command);});
  }
  std::optional<ew::ExecutionVersion> committedVersion()const {
    std::lock_guard<std::mutex> l(mutex_);if(active_)return active_->version;return {};
  }
  struct CommittedIdentity {ew::ExecutionVersion version;std::int64_t trajectory_id;};
  // Reference replacement needs one predecessor, not independently acquired
  // version/curve values. Copy only identity (no control points/proof vectors).
  std::optional<CommittedIdentity> committedIdentity()const {
    std::lock_guard<std::mutex> l(mutex_);if(!active_)return {};
    return CommittedIdentity{active_->version,active_->spline.trajectory.traj_id};
  }
  // Scheduling only: a prepared transaction owns its original identity until
  // its explicit commit/abort. Reference ingress must not rewrite that owner.
  bool localCommitInFlight()const {
    std::lock_guard<std::mutex> l(mutex_);
    return local_follower_&&prepared_.has_value()&&local_prepare_request_.has_value();
  }
  std::int64_t committedTrajectory()const {
    std::lock_guard<std::mutex> l(mutex_);return active_?active_->spline.trajectory.traj_id:-1;
  }
  bool cancelReference(const ew::ExecutionVersion& barrier) {
    std::lock_guard<std::mutex> delivery(command_delivery_mutex_);
    std::lock_guard<std::mutex> l(mutex_);
    if(barrier.schema_version!=3||barrier.session_id.empty()||barrier.task_id.empty()||
       barrier.route_id.empty()||barrier.route_hash.empty()||barrier.reference_generation==0)return false;
    for(const auto* job:{&active_,&candidate_})if(*job &&
        (!sameExecutionTask((*job)->version,barrier)||
         barrier.reference_generation<=(*job)->version.reference_generation))return false;
    retired_=barrier;task_closed_=true;active_.reset();local_transition_incumbent_.reset();pending_committed_display_.reset();clearCandidatesLocked();
    if(local_follower_){prepared_.reset();local_prepare_request_.reset();local_commit_fact_.reset();}
    task_lease_history_.clear();latest_demand_.reset();pending_local_demand_.reset();
    if(grant_)grant_retired_=true;
    ++motion_stop_barrier_;++prepared_stop_barrier_;
    return true;
  }
  void cancelPending(){std::lock_guard<std::mutex> l(mutex_);clearCandidatesLocked();}
  // Reference ingress may replace only the still-open task's candidate. The
  // default remains an exact predecessor check. An explicitly classified
  // rolling extension may have observed an earlier curve of the SAME exact
  // reference while Native naturally refreshed it. Admit that input under
  // this lock without rewriting its original proposal/receipt predecessor.
  // The later localPrepare/localCommit still CAS the actual current curve;
  // input admission neither commits geometry nor grants motion authority.
  bool replacePendingReference(const ew::ReferenceProposal& proposal,bool rolling_reference=false,
      bool continuous_stair_portal=false,
      const std::optional<ew::ExecutionVersion>& classified_predecessor=std::nullopt) {
    std::lock_guard<std::mutex> delivery(command_delivery_mutex_);
    std::lock_guard<std::mutex> l(mutex_);
    const auto decision=[&]() {
    if(continuous_stair_portal&&(!rolling_reference||!local_follower_))return false;
    if(!referenceTaskOpenLocked(proposal.version))return false;
    // Recheck the owner's scheduling condition under the same mutex as the
    // prepare/commit lane; a reservation may begin after its optimistic poll.
    if(rolling_reference&&local_follower_&&prepared_&&local_prepare_request_)return false;
    bool inherited_stair_portal=false;
    if(active_) {
      if(continuous_stair_portal) {
        // Bind the FSM's geometric classification to the admitted reference
        // it actually compared, rather than turning a bool into permission to
        // jump from an unrelated active segment. A pending same-segment slide
        // or an already classified portal can be the geometric predecessor,
        // but neither becomes the committed curve before localCommit.
        if(!classified_predecessor||
            classified_predecessor->reference_generation>=proposal.version.reference_generation)return false;
        auto same_segment_predecessor=active_->version;
        same_segment_predecessor.reference_generation=classified_predecessor->reference_generation;
        const bool pending_matches=candidate_reference_&&*candidate_reference_==*classified_predecessor;
        const bool admitted_lineage=*classified_predecessor==active_->version||
          (pending_matches&&(*classified_predecessor==same_segment_predecessor||
            (rolling_stair_predecessor_&&*rolling_stair_predecessor_==active_->version)));
        if(!admitted_lineage)return false;
      }
      if(rolling_reference&&local_follower_&&rolling_stair_predecessor_&&candidate_reference_&&
          *rolling_stair_predecessor_==active_->version&&
          proposal.version.reference_generation>candidate_reference_->reference_generation) {
        auto previous_pending=*candidate_reference_;
        previous_pending.reference_generation=proposal.version.reference_generation;
        inherited_stair_portal=previous_pending==proposal.version;
      }
      auto rolling_version=active_->version;
      rolling_version.reference_generation=proposal.version.reference_generation;
      // Only the FSM's ordered, XYZ-overlapping fixed-STAIR portal classifier
      // may update this semantic label. Task, route/hash, map, anchor and
      // localization identity still compare exactly below; actual local
      // prepare/commit continues to CAS the immutable incumbent curve.
      if(continuous_stair_portal||inherited_stair_portal)rolling_version.segment_id=proposal.version.segment_id;
      if(rolling_reference&&(!local_follower_||proposal.version!=rolling_version||
          proposal.version.reference_generation<=active_->version.reference_generation))return false;
      if(active_->version!=proposal.expected_version||
         !sameExecutionTask(active_->version,proposal.version))return false;
      const auto current=active_->spline.trajectory.traj_id;
      if(current!=proposal.expected_trajectory_id&&
         !(rolling_reference&&proposal.expected_trajectory_id>0&&
           proposal.expected_trajectory_id<current))return false;
    } else if(rolling_reference||!proposal.expected_version.task_id.empty())return false;
    if(local_follower_&&(!task_domain_||(task_closed_&&!sameExecutionTask(*task_domain_,proposal.version)))) {
      pending_committed_display_.reset();
      task_domain_=proposal.version;task_closed_=false;resetTaskLeaseDomainLocked();
      latest_permit_.reset();permits_.clear();latest_demand_.reset();++motion_stop_barrier_;
    }
    // A portal reference may be acknowledged before any successor curve is
    // committed. Its later same-segment windows still CAS the exact original
    // incumbent, not the pending segment as though it were already active.
    // Only a previously classified portal establishes this lineage; ordinary
    // hard replacements and any changed predecessor/identity cannot inherit it.
    if(continuous_stair_portal&&active_)rolling_stair_predecessor_=active_->version;
    else if(!inherited_stair_portal)rolling_stair_predecessor_.reset();
    // A same-segment sliding window changes search input, not the selected
    // candidate's admission ownership. Dropping that slot here races the
    // independent full-curve check: its final identity guard then discards a
    // completed result, even though the task/anchor/terrain did not change.
    // Keep exactly that one selected job and its ORIGINAL offer deadline;
    // the first new-window solve joins the existing one-latest selector.
    // A semantic portal/reanchor/hard replacement retains its original fence.
    bool keep_selected=false;
    if(rolling_reference&&local_follower_&&candidate_&&
       candidate_->version.reference_generation<proposal.version.reference_generation&&
       (!active_||!sameJob(*candidate_,*active_))) {
      auto selected_version=candidate_->version;
      selected_version.reference_generation=proposal.version.reference_generation;
      keep_selected=selected_version==proposal.version;
    }
    if(keep_selected)queued_candidate_.reset();
    else clearCandidatesLocked();
    if(local_follower_&&!rolling_reference) {
      // A hard reference replacement fences both uncommitted lanes. Leaving
      // the old prepared lane alive lets its delayed commit cross the new
      // reference fence even though the selected candidate was discarded.
      // Keep the actual incumbent and its control/motion evidence unchanged;
      // the follower's existing negative-ACK path rolls back only that stale
      // exchange and preserves a newly delivered candidate.
      if(prepared_&&pending_local_demand_&&pending_local_demand_->waiting_for_commit&&
          pending_local_demand_->demand.version==prepared_->version&&
          pending_local_demand_->demand.trajectory_id==prepared_->spline.trajectory.traj_id)
        pending_local_demand_.reset();
      prepared_.reset();local_prepare_request_.reset();
    }
    if(local_follower_)candidate_reference_=proposal.version;
    return true;
    }();
    execution_trace_.attempt([&] {
      BoundedExecutionTrace::Event event;event.kind=BoundedExecutionTrace::Kind::ReferenceDecision;
      event.ros_ns=node_?node_->now().nanoseconds():0;event.accepted=decision;
      event.reason.set(continuous_stair_portal?"stair_portal":rolling_reference?"rolling":"hard_reference");
      traceLocked(event,{&proposal.version,-1,proposal.proposal_id},
        {&proposal.expected_version,proposal.expected_trajectory_id,{}});
    });
    return decision;
  }
  bool handoff(const ew::ExecutionHandoffGrant& g) {return handoffAt(g,node_->now().nanoseconds());}
  bool handoffAt(const ew::ExecutionHandoffGrant& g,std::int64_t now) {
    std::lock_guard<std::mutex> delivery(command_delivery_mutex_);
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
    std::lock_guard<std::mutex> delivery(command_delivery_mutex_);
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
        queueCommittedDisplayLocked();
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
    queueCommittedDisplayLocked();
    return true;
  }
  void demand(const ew::MotionDemand& d) {
    demandWithClock(d,[this,&d]{return node_?node_->now().nanoseconds():ns(d.source_stamp);},
      [this](const ew::LocalMotionCommand& command){if(local_command_pub_)local_command_pub_->publish(command);});
  }
  // Formal execution stages the follower's exact bounded command under the
  // proof/lease ledger, then sends outside it with ordered retirement fences.
  // No map query or motion proof is performed here; SCAN's current Native
  // curve verdict is the collision gate. Fixtures use the same output seam.
  void demandAt(const ew::MotionDemand& d,std::int64_t now,
      const std::function<void(const ew::LocalMotionCommand&)>& emit_command={}) {
    demandWithClock(d,[=]{return now;},emit_command);
  }
private:
  void demandWithClock(const ew::MotionDemand& d,const std::function<std::int64_t()>& clock,
      const std::function<void(const ew::LocalMotionCommand&)>& emitter) {
    CommandBatch commands;const auto emit_command=captureCommands(commands,emitter,clock);
    [&] {
    std::lock_guard<std::mutex> l(mutex_);
    if(d.transport_mode!=mode_||d.version.schema_version!=3) {
      recordLocalDemandStageLocked(MotionCheckStage::DemandContractInvalid,d,clock());return;
    }
    if(local_follower_&&(task_closed_||(task_lease_&&
        (d.execution_id!=task_lease_->execution_id||d.control_epoch!=task_lease_->control_epoch||
         d.sdk_session!=task_lease_->sdk_session||d.sdk_arm_generation!=task_lease_->sdk_arm_generation)))) {
      recordLocalDemandStageLocked(MotionCheckStage::DemandAuthorityMismatch,d,clock());return;
    }
    if(latest_demand_&&d.execution_id==latest_demand_->execution_id&&d.sequence<=latest_demand_->sequence) {
      recordLocalDemandStageLocked(MotionCheckStage::DemandSequenceNotNew,d,clock());return;
    }
    const bool active_matches=active_&&d.version==active_->version&&
      d.trajectory_id==active_->spline.trajectory.traj_id;
    const bool transition_matches=local_follower_&&local_transition_incumbent_&&
      local_transition_incumbent_->version==d.version&&
      local_transition_incumbent_->spline.trajectory.traj_id==d.trajectory_id;
    if((local_follower_&&!active_matches&&!transition_matches)||(writer_handoff_&&active_&&!active_matches)) {
      const auto now=clock();
      const bool zero=planarDemandIsZero(d.velocity)&&d.velocity.linear.z==0.&&
        d.velocity.angular.x==0.&&d.velocity.angular.y==0.;
      const auto& waiting=local_prepare_request_&&prepared_&&prepared_->version==d.version&&
        prepared_->spline.trajectory.traj_id==d.trajectory_id?prepared_:candidate_;
      const bool candidate_positive=waiting&&waiting->proof&&waiting->proof->valid&&
        waiting->proof->whole_curve&&waiting->proof->sequence>=d.validation_sequence&&
        d.validation_sequence>0&&curveVerdictCurrent(*waiting->proof,now);
      if(local_follower_&&task_domain_&&sameExecutionTask(*task_domain_,d.version)&&
          task_lease_&&task_lease_->allowed&&!task_lease_->revoked&&waiting&&
          waiting->version==d.version&&waiting->spline.trajectory.traj_id==d.trajectory_id&&
          (zero||candidate_positive)&&originalLocalDemandFreshLocked(d,now)) {
        latest_demand_=d;pending_local_demand_.reset();
        if(d.hold||planarDemandIsZero(d.velocity))++motion_stop_barrier_;
        pending_local_demand_=PendingLocalDemand{d,motion_stop_barrier_,true};
        ++local_commit_deferred_commands_;
        // A known candidate's fresh zero retires any older pending motion even
        // if its Native proof has since become negative/expired. It grants no
        // geometry and cannot emit until the real commit is accepted.
        recordLocalDemandStageLocked(MotionCheckStage::CandidateCommitPending,d,now);
      } else recordLocalDemandStageLocked(
        !waiting||waiting->version!=d.version||waiting->spline.trajectory.traj_id!=d.trajectory_id?
          MotionCheckStage::ActiveGeometryMismatch:
        !zero&&!candidate_positive?MotionCheckStage::CandidateProofUnavailable:
        !originalLocalDemandFreshLocked(d,now)?MotionCheckStage::CandidateSourceExpired:
          MotionCheckStage::DemandAuthorityMismatch,d,now);
      return;
    }
    // Seeing an exact successor demand is the irrevocable transport handover.
    // A delayed predecessor, including a later sequence, cannot return after
    // this point. Task revoke/HOLD also discards the overlap independently.
    if(local_follower_&&active_matches)local_transition_incumbent_.reset();
    latest_demand_=d;
    // A newer zero/HOLD or command supersedes the only deferred sample. The
    // sequence high-water above prevents delayed older commands reviving it.
    pending_local_demand_.reset();
    if(!writer_commit_sequence_&&active_)for(auto it=permits_.rbegin();it!=permits_.rend();++it)
      if(it->sequence==d.permit_sequence&&it->version==d.version&&it->trajectory_id==d.trajectory_id) {
        pinInitialIntentLocked(d,*it,*active_);break;
    }
    if(d.hold||planarDemandIsZero(d.velocity))++motion_stop_barrier_;
    if(!local_follower_)return;
    relayOrDeferLocalDemandLocked(d,clock(),emit_command);
    }();
    deliverCommands(commands,emitter,clock);
  }
  void recordLocalDemandStageLocked(MotionCheckStage stage,const ew::MotionDemand& demand,std::int64_t now) {
    const auto* command_job=localCommandGeometryLocked(demand);
    const auto* proof=command_job&&command_job->proof?&*command_job->proof:nullptr;
    motion_diagnostics_.record(stage,demand,now,proof?proof->sequence:0,
      proof?proof->map_snapshot_revision:0,active_?active_->spline.trajectory.traj_id:-1,
      body_history_.empty()?0:body_history_.back().source_ns);
    BoundedExecutionTrace::Event event;event.kind=BoundedExecutionTrace::Kind::DemandRejected;
    event.ros_ns=now;event.sequence=demand.sequence;event.source_ns=ns(demand.source_stamp);
    event.body_ns=ns(demand.body_source_stamp);event.valid_until_ns=ns(demand.valid_until);
    event.validation_floor=demand.validation_sequence;event.hold=demand.hold;
    event.velocity={{demand.velocity.linear.x,demand.velocity.linear.y,demand.velocity.angular.z}};
    event.reason.set(MotionCheckDiagnostics::name(stage));
    traceLocked(event,{&demand.version,demand.trajectory_id,{}});
  }
  bool originalLocalDemandFreshLocked(const ew::MotionDemand& d,std::int64_t now)const {
    return braking_&&ns(d.source_stamp)>0&&ns(d.source_stamp)<=now+20000000LL&&
      ns(d.body_source_stamp)>0&&ns(d.body_source_stamp)<=now+20000000LL&&
      std::min<std::int64_t>({ns(d.valid_until),ns(d.source_stamp)+braking_->commandPipelineNs(),
        ns(d.body_source_stamp)+100000000LL})>now;
  }
  void relayOrDeferLocalDemandLocked(const ew::MotionDemand& d,std::int64_t now,
      const std::function<void(const ew::LocalMotionCommand&)>& emit_command,bool body_delivery=false,
      bool request_refresh=true) {
    const auto begin=ValidationCycle::Clock::now();
    MotionCheckStage stage=MotionCheckStage::Idle;
    const auto command=relayLocalDemandLocked(d,now,&stage);
    const auto* command_job=localCommandGeometryLocked(d);
    if(command){recordLocalCommandLocked(*command,now);
      if(emit_command)publishLocalCommandLocked(*command,now,emit_command);}
    else if((stage==MotionCheckStage::BodyBehindDemand||stage==MotionCheckStage::BodyNotDelivered||
             stage==MotionCheckStage::BodyTwistMissing||
             (stage==MotionCheckStage::ActiveProofUnavailable&&command_job&&
              (!command_job->proof||command_job->proof->valid)))&&originalLocalDemandFreshLocked(d,now)) {
      pending_local_demand_=PendingLocalDemand{d,motion_stop_barrier_,false};
      if(stage==MotionCheckStage::ActiveProofUnavailable) {
        ++local_proof_deferred_commands_;
        // A check on the same ray snapshot cannot renew its source deadline.
        // Only an external delivery requests an early check; completion must
        // wait for a real map update/the normal period, never wake itself.
        if(request_refresh)snapshotPublished();
      } else if(!body_delivery)++local_body_deferred_commands_;
    }
    ++local_motion_passes_;
    local_last_motion_pass_ns_=std::chrono::duration_cast<std::chrono::nanoseconds>(ValidationCycle::Clock::now()-begin).count();
    local_max_motion_pass_ns_=std::max(local_max_motion_pass_ns_,local_last_motion_pass_ns_);
  }
  std::optional<ew::LocalMotionCommand> relayLocalDemandLocked(const ew::MotionDemand& d,std::int64_t now,
      MotionCheckStage* failed_stage=nullptr,
      std::optional<std::uint64_t> original_lease_sequence={}) {
    const auto wait=[&](MotionCheckStage stage) -> std::optional<ew::LocalMotionCommand> {
      if(failed_stage)*failed_stage=stage;
      recordLocalDemandStageLocked(stage,d,now);
      if(stage==MotionCheckStage::CommandSourceExpired||stage==MotionCheckStage::BodySourceExpired)
        ++local_expired_commands_;
      return {};
    };
    const auto* command_job=localCommandGeometryLocked(d);
    if(!command_job)
      return wait(MotionCheckStage::ActiveGeometryUnavailable);
    if(!braking_||!braking_->valid())return wait(MotionCheckStage::BrakingModelUnavailable);
    for(const double value:{d.velocity.linear.x,d.velocity.linear.y,d.velocity.linear.z,
        d.velocity.angular.x,d.velocity.angular.y,d.velocity.angular.z})
      if(!std::isfinite(value))return wait(MotionCheckStage::PermitBindingMismatch);
    if(d.hold&&!planarDemandIsZero(d.velocity))return wait(MotionCheckStage::PermitBindingMismatch);
    const bool hold=d.hold||planarDemandIsZero(d.velocity);
    std::int64_t deadline=ns(d.valid_until);
    if(!hold) {
      // Always read the CURRENT committed verdict. Historical positives may
      // explain a software commit but cannot revive a negative/expired curve.
      const auto& proof=command_job->proof;
      if(!proof||!proof->valid||(!proof->whole_curve&&!proof->remaining_curve)||proof->version!=d.version||
          proof->trajectory_id!=d.trajectory_id||proof->proposal_id!=command_job->proposal||
          proof->frame_id!=frame_||proof->transport_mode!=mode_||
          proof->collision_policy!=geometry_policy_||d.validation_sequence==0||
          proof->sequence<d.validation_sequence||!usablePlanningGeometry(*command_job)||
          command_job->support->version!=d.version||proof->support_reference_id!=command_job->support->support_reference_id||
          proof->support_hash!=command_job->support->support_hash||ns(proof->check_end)<=0||
          ns(proof->check_end)>now||!curveVerdictCurrent(*proof,now))
        return wait(MotionCheckStage::ActiveProofUnavailable);
      // A command computed before a negative curve result cannot cross that
      // result after recovery. This fixed history is bounded at 64 facts.
      if(!motionProof(proofs_,d,now,local_follower_))return wait(MotionCheckStage::CurveProofSuperseded);
      if(!nativePipelineReadyLocked(d.version,now))return wait(MotionCheckStage::NativePipelineFault);
      if(body_history_.empty())return wait(MotionCheckStage::BodyNotDelivered);
      const auto& body=body_history_.back();
      if(body.provider_identity&&!body.provider_identity->matches(d.version))
        return wait(MotionCheckStage::BodyNotDelivered);
      if(body.provider_identity&&now>=body.provider_valid_until_ns)
        return wait(MotionCheckStage::BodySourceExpired);
      if(body.provider_identity)deadline=std::min(deadline,body.provider_valid_until_ns);
      if(!body.twist)return wait(MotionCheckStage::BodyTwistMissing);
      if(body.source_ns<ns(d.body_source_stamp))return wait(MotionCheckStage::BodyBehindDemand);
      if(!motionSourceFresh(body.source_ns,now))return wait(MotionCheckStage::BodySourceExpired);
      double yaw=0.;if(!measuredBodyYaw(body.pose,frame_,now*1e-9,.1,yaw))
        return wait(MotionCheckStage::BodyYawUnavailable);
      const auto& velocity=*body.twist;
      for(const double value:{velocity.linear.x,velocity.linear.y,velocity.linear.z,
          velocity.angular.x,velocity.angular.y,velocity.angular.z})
        if(!std::isfinite(value))return wait(MotionCheckStage::BodyTwistMissing);
      const bool holonomic=braking_->kinematic_profile==KinematicProfile::HolonomicXYYaw;
      const bool translation_invalid=holonomic?
        std::hypot(d.velocity.linear.x,d.velocity.linear.y)>braking_->max_speed+1e-12:
        (d.velocity.linear.x<0.||d.velocity.linear.x>braking_->max_speed||std::abs(d.velocity.linear.y)>1e-9);
      if(d.braking_model_sha256!=braking_->sha256||translation_invalid||std::abs(d.velocity.angular.z)>braking_->max_yaw||
          std::abs(d.velocity.linear.z)+std::abs(d.velocity.angular.x)+std::abs(d.velocity.angular.y)>1e-9)
        return wait(MotionCheckStage::PermitBindingMismatch);
      // max_speed/max_yaw cap REQUESTED commands, not independently measured
      // response. SDK amplitude is not a verified SI response calibration;
      // transient state overshoot must not turn a bounded correction into an
      // artificial missing-command/zero pulse. Finite acquired state, original
      // deadlines and SCAN's actual current-curve verdict remain required.
      deadline=std::min<std::int64_t>(deadline,body.source_ns+100000000LL);
      if(!local_follower_)deadline=std::min(deadline,ns(proof->valid_until));
    }
    const auto command=localCommandLocked(d,deadline,now,original_lease_sequence);
    if(!command)return wait(ns(d.valid_until)<=now||ns(d.body_source_stamp)+100000000LL<=now?
      MotionCheckStage::CommandSourceExpired:MotionCheckStage::SourceLeaseExpired);
    motion_diagnostics_.record(hold?MotionCheckStage::Idle:MotionCheckStage::LocalCommandRelayed,d,now,
      command_job->proof?command_job->proof->sequence:0,command_job->proof?command_job->proof->map_snapshot_revision:0);
    if(active_&&active_->version==d.version&&active_->spline.trajectory.traj_id==d.trajectory_id)
      local_transition_incumbent_.reset();
    return command;
  }
  void recordLocalCommandLocked(const ew::LocalMotionCommand& command,std::int64_t now) {
    (void)now;
    ++local_commands_staged_;
    // Keep at most the exact staged intent. Only deliverCommands records an
    // actual returned send; a staged/expired value is not a publication.
    relayed_local_command_=command;relayed_local_stop_barrier_=motion_stop_barrier_;
  }
  void refreshRelayedLocalCommandLocked(const ew::TrajectoryValidation& proof,std::int64_t now,
      const std::function<void(const ew::LocalMotionCommand&)>& emit_command) {
    if(!emit_command||!proof.valid||!relayed_local_command_||!latest_demand_||
       relayed_local_stop_barrier_!=motion_stop_barrier_||!active_||!active_->proof||
       active_->proof->sequence!=proof.sequence)return;
    const auto previous=*relayed_local_command_;
    const auto& demand=*latest_demand_;
    if(previous.hold||planarDemandIsZero(previous.velocity)||ns(previous.valid_until)<=now||
       previous.version!=proof.version||previous.trajectory_id!=proof.trajectory_id||
       previous.version!=demand.version||previous.trajectory_id!=demand.trajectory_id||
       previous.execution_id!=demand.execution_id||previous.sequence!=demand.sequence||
       previous.source_stamp!=demand.source_stamp||previous.body_source_stamp!=demand.body_source_stamp||
       !originalLocalDemandFreshLocked(demand,now))return;
    // Reuse the production relay, including current ownership, source leases
    // and the negative-proof barrier. Evidence updates preserve the ORIGINAL
    // emitted authorization too: a genuine pre-demand heartbeat delivered
    // late must not rebind this command and silently suppress its refresh.
    // Both that original lease and the current grant must still be live.
    const auto updated=relayLocalDemandLocked(demand,now,nullptr,previous.lease_sequence);
    if(!updated||ns(updated->valid_until)<=ns(previous.valid_until))return;
    auto original=*updated;original.valid_until=previous.valid_until;
    if(original!=previous)return; // ONLY evidence expiry may change.
    recordLocalCommandLocked(*updated,now);
    publishLocalCommandLocked(*updated,now,emit_command);
  }
  std::optional<ew::LocalMotionCommand> localCommandLocked(const ew::MotionDemand& d,
      std::int64_t evidence_deadline,std::int64_t now,
      std::optional<std::uint64_t> original_lease_sequence={})const {
    if(!local_follower_||!task_lease_||!task_lease_->allowed||task_lease_->revoked||
        task_closed_||!localCommandGeometryLocked(d)||!sameExecutionTask(task_lease_->version,d.version)||
        d.execution_id!=task_lease_->execution_id||d.control_epoch!=task_lease_->control_epoch||
        d.sdk_session!=task_lease_->sdk_session||d.sdk_arm_generation!=task_lease_->sdk_arm_generation||
        ns(task_lease_->source_stamp)<=0||ns(task_lease_->source_stamp)>now+20000000LL||
        ns(task_lease_->valid_until)<=now||now-ns(task_lease_->source_stamp)>750000000LL||
        ns(d.source_stamp)<=0||ns(d.body_source_stamp)<=0||ns(d.source_stamp)>now+20000000LL||
        ns(d.body_source_stamp)>now+20000000LL||!braking_)return {};
    // A 10 Hz task heartbeat can arrive while this 20 Hz demand is being
    // checked. It must not retroactively authorize an earlier command. Select
    // the newest ORIGINAL same-grant lease which already existed at d.source.
    // A proof-only refresh pins the exact previously emitted lease instead;
    // it does not borrow authorization or lifetime from a newer heartbeat.
    // The current lease above still fences HOLD/revoke/ownership changes.
    const auto lease=std::find_if(task_lease_history_.rbegin(),task_lease_history_.rend(),[&](const auto& p){
      return (!original_lease_sequence||p.sequence==*original_lease_sequence)&&
        p.allowed&&!p.revoked&&sameExecutionTask(p.version,d.version)&&
        p.execution_id==d.execution_id&&p.control_epoch==d.control_epoch&&
        p.sdk_session==d.sdk_session&&p.sdk_arm_generation==d.sdk_arm_generation&&
        ns(p.source_stamp)<=ns(d.source_stamp)&&ns(p.valid_until)>now&&
        now-ns(p.source_stamp)<=750000000LL;});
    if(lease==task_lease_history_.rend())return {};
    const auto pipeline=braking_->commandPipelineNs();
    const auto deadline=std::min<std::int64_t>({evidence_deadline,ns(d.valid_until),
      ns(lease->valid_until),ns(task_lease_->valid_until),ns(d.source_stamp)+pipeline,
      ns(d.body_source_stamp)+100000000LL});
    if(deadline<=now)return {};
    ew::LocalMotionCommand out;out.schema_version=1;out.version=d.version;out.execution_id=d.execution_id;
    out.control_epoch=d.control_epoch;out.sdk_session=d.sdk_session;out.sdk_arm_generation=d.sdk_arm_generation;
    out.lease_sequence=lease->sequence;out.sequence=d.sequence;out.source_stamp=d.source_stamp;
    // Preserve the controller's reconciliation of an ACTUAL sender gap. This
    // cannot be inferred from SCAN proof publication or a newer body sample.
    out.acknowledged_discontinuity_write=d.acknowledged_discontinuity_write;
    out.body_source_stamp=d.body_source_stamp;out.body_valid_until=rclcpp::Time(
      std::min<std::int64_t>(ns(d.valid_until),ns(d.body_source_stamp)+100000000LL));
    out.valid_until=rclcpp::Time(deadline);out.body_frame=body_frame_;out.velocity=d.velocity;
    out.hold=d.hold||planarDemandIsZero(d.velocity);out.trajectory_id=d.trajectory_id;
    out.reason=d.reason;out.transport_mode=mode_;return out;
  }
  struct Job {ew::TaggedBspline spline;ew::ExecutionVersion version;std::string proposal;
    std::optional<ew::SupportReference> support;double measured_time{0.};
    bool has_goal_yaw{false};double goal_yaw{0.};Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
    std::optional<ew::LocalPlanDebug> debug;std::optional<ew::TrajectoryValidation> proof;
    std::optional<ew::TrackingProgress> progress;
    SplineHeadingContract heading;
    bool no_motion_preview{false};std::optional<NoMotionPreviewProgress> preview_progress;
    std::optional<PreviewCandidateOrigin> preview_candidate_origin;
    // Real body projections and certified geometry deliberately have separate
    // clocks. Ray/proof outages must not manufacture an odometry source gap.
    std::optional<NoMotionPreviewProgress> observed_preview_progress;
    double preview_projection_resolution{0.};double goal_yaw_tolerance{0.};
    // Same immutable job identity; bounded history only for source pairing.
    // A commit/replacement owns a different Job and cannot inherit these.
    std::deque<ew::TrackingProgress> progress_history{};
    std::shared_ptr<const PlanningSupport> geometry_support;};
  const Job* localCommandGeometryLocked(const ew::MotionDemand& demand)const {
    const auto matches=[&](const std::optional<Job>& job) {
      return job&&job->version==demand.version&&
        job->spline.trajectory.traj_id==demand.trajectory_id;
    };
    if(matches(active_))return &*active_;
    // One predecessor only, from an accepted CAS in this task. It can bridge
    // final-ACK transport, not a subsequent task or an arbitrary old curve.
    if(local_follower_&&!task_closed_&&active_&&task_domain_&&
       sameExecutionTask(*task_domain_,demand.version)&&matches(local_transition_incumbent_)&&
       sameExecutionTask(active_->version,demand.version))return &*local_transition_incumbent_;
    return nullptr;
  }
  static BoundedExecutionTrace::IdentityInput traceIdentity(const std::optional<Job>& job) {
    return job?BoundedExecutionTrace::IdentityInput{&job->version,job->spline.trajectory.traj_id,job->proposal}:
      BoundedExecutionTrace::IdentityInput{};
  }
  BoundedExecutionTrace::Context traceContextLocked(BoundedExecutionTrace::IdentityInput request,
      BoundedExecutionTrace::IdentityInput expected={})const {
    return {request,expected,traceIdentity(active_),traceIdentity(prepared_),traceIdentity(candidate_)};
  }
  void traceLocked(BoundedExecutionTrace::Event event,BoundedExecutionTrace::IdentityInput request,
      BoundedExecutionTrace::IdentityInput expected={}) {
    event.retention.set(task_domain_?&*task_domain_:nullptr,!task_closed_);
    event.stop_barrier=motion_stop_barrier_;
    event.native_body_ns=body_history_.empty()?0:body_history_.back().source_ns;
    if(!event.proof&&active_&&active_->proof) {
      event.proof=active_->proof->sequence;event.map_revision=active_->proof->map_snapshot_revision;
      event.proof_until_ns=ns(active_->proof->valid_until);
    }
    if(task_lease_){event.lease_sequence=task_lease_->sequence;event.lease_until_ns=ns(task_lease_->valid_until);}
    execution_trace_.record(event,traceContextLocked(request,expected));
  }
  void traceCommitLocked(const ew::LocalTrajectoryCommit& c,std::int64_t now,std::int64_t begin,
      const std::array<std::uint64_t,5>& before) {
    BoundedExecutionTrace::Event event;event.kind=BoundedExecutionTrace::Kind::CommitDecision;
    event.ros_ns=now;event.sequence=c.commit_sequence;event.begin_steady_ns=begin;event.before_identity=before;
    event.source_ns=ns(c.source_stamp);event.body_ns=ns(c.body_source_stamp);
    event.validation_floor=c.native_validation_sequence;event.accepted=c.accepted;
    event.reason.set(c.reason);
    traceLocked(event,{&c.version,c.trajectory_id,c.proposal_id},{&c.expected_version,c.expected_trajectory_id,{}});
  }
  void traceCommandLocked(const ew::LocalMotionCommand& c,std::int64_t now,BoundedExecutionTrace::Kind kind,
      std::int64_t begin) {
    BoundedExecutionTrace::Event event;event.kind=kind;event.ros_ns=now;event.sequence=c.sequence;
    event.begin_steady_ns=begin;
    event.source_ns=ns(c.source_stamp);event.body_ns=ns(c.body_source_stamp);event.valid_until_ns=ns(c.valid_until);
    event.velocity={{c.velocity.linear.x,c.velocity.linear.y,c.velocity.angular.z}};
    event.hold=c.hold;event.accepted=kind==BoundedExecutionTrace::Kind::CommandPublishReturned;
    event.reason.set(event.accepted?c.reason:"publisher_exception");
    // The command wire has no proposal field. Attach only the exact Native
    // committed binding already verified for this command, not a latest one.
    const std::string_view proposal=active_&&active_->version==c.version&&
      active_->spline.trajectory.traj_id==c.trajectory_id?std::string_view(active_->proposal):std::string_view{};
    traceLocked(event,{&c.version,c.trajectory_id,proposal});
  }
  void publishLocalCommandLocked(const ew::LocalMotionCommand& c,std::int64_t now,
      const std::function<void(const ew::LocalMotionCommand&)>& emit) {
    (void)now;
    // emit is an invocation-local bounded value collector, never RMW I/O.
    emit(c);
  }
  enum class CommandOrigin {Demand,Body,Commit,Proof};
  struct StagedCommand {
    ew::LocalMotionCommand value;
    std::uint64_t stop_barrier;
    std::int64_t captured_ros_ns,captured_steady_ns;
    CommandOrigin origin;
  };
  struct CommandBatch {
    // A check may complete one deferred demand and refresh its evidence once.
    // All other producers emit at most one. Overflow is a programming failure,
    // never silent replacement of a stop/nonzero command.
    std::array<std::optional<StagedCommand>,2> values;
    std::size_t size{0};
  };
  using CommandEmitter=std::function<void(const ew::LocalMotionCommand&)>;
  CommandEmitter captureCommands(CommandBatch& batch,const CommandEmitter& emitter,
      const std::function<std::int64_t()>& clock,CommandOrigin origin=CommandOrigin::Demand) {
    if(!emitter)return {};
    // Capture before ledger/delivery waits, not after them; an injected/static
    // clock must not hide time spent queued behind an earlier reliable send.
    const auto received_ros_ns=clock(),received_steady_ns=BoundedExecutionTrace::steadyNowNs();
    return [this,&batch,received_ros_ns,received_steady_ns,origin](const ew::LocalMotionCommand& value) {
      if(batch.size==batch.values.size())throw std::logic_error("local command batch overflow");
      batch.values[batch.size++]=StagedCommand{value,motion_stop_barrier_,received_ros_ns,received_steady_ns,origin};
    };
  }
  bool stagedCommandCurrentLocked(const StagedCommand& staged,std::int64_t now) {
    const auto& c=staged.value;
    if(staged.stop_barrier!=motion_stop_barrier_||!latest_demand_||
       latest_demand_->version!=c.version||latest_demand_->trajectory_id!=c.trajectory_id||
       latest_demand_->sequence!=c.sequence||latest_demand_->execution_id!=c.execution_id)return false;
    // Recheck the original lease/command/body deadlines, not a new permit or
    // newly stamped command. Monotonic residence also covers a paused clock.
    if(ns(c.valid_until)<=now||BoundedExecutionTrace::steadyNowNs()-staged.captured_steady_ns>=
       ns(c.valid_until)-staged.captured_ros_ns)return false;
    const auto allowed=localCommandLocked(*latest_demand_,ns(c.valid_until),now,c.lease_sequence);
    if(!allowed||ns(allowed->valid_until)<ns(c.valid_until))return false;
    if(!c.hold) {
      const auto* job=localCommandGeometryLocked(*latest_demand_);
      if(!job||!job->proof||!job->proof->valid||!curveVerdictCurrent(*job->proof,now)||
         (!local_follower_&&ns(job->proof->valid_until)<ns(c.valid_until))||
         !nativePipelineReadyLocked(c.version,now))return false;
      // Waiting for an earlier reliable send may span negative -> positive
      // evidence. Preserve the existing relay's negative-history barrier:
      // a fresh positive cannot revive intent generated before the negative.
      if(!motionProof(proofs_,*latest_demand_,now,local_follower_))return false;
    }
    if(last_delivered_local_command_&&last_delivered_local_command_->execution_id==c.execution_id&&
       sameExecutionTask(last_delivered_local_command_->version,c.version)&&
       last_delivered_local_command_->sdk_session==c.sdk_session&&
       last_delivered_local_command_->sdk_arm_generation==c.sdk_arm_generation&&
       last_delivered_local_command_->control_epoch==c.control_epoch) {
      if(c.sequence<last_delivered_local_command_->sequence)return false;
      if(c.sequence==last_delivered_local_command_->sequence) {
        auto original=c;original.valid_until=last_delivered_local_command_->valid_until;
        if(original!=*last_delivered_local_command_||
           ns(c.valid_until)<=ns(last_delivered_local_command_->valid_until))return false;
      }
    }
    return true;
  }
  void deliverCommands(const CommandBatch& batch,const CommandEmitter& emitter,
      const std::function<std::int64_t()>& clock,std::unique_lock<std::mutex>* owned=nullptr,bool fence=false) {
    if(batch.size==0&&!fence)return;
    // The ONLY nested order is delivery -> ledger. No caller retains ledger.
    // Identity-changing ACK paths also own delivery, so an already-started
    // publish finishes before cancel/retire/commit can report completion.
    std::unique_lock<std::mutex> delivery(command_delivery_mutex_,std::defer_lock);
    if(!owned)delivery.lock();
    else if(!owned->owns_lock())throw std::logic_error("missing command delivery ownership");
    for(std::size_t i=0;i<batch.size;++i) {
      const auto& staged=*batch.values[i];const auto& c=staged.value;
      {std::lock_guard<std::mutex> lock(mutex_);
        if(!stagedCommandCurrentLocked(staged,clock()))continue;}
      const auto begin=BoundedExecutionTrace::steadyNowNs();
      try {emitter(c);}
      catch(...) {
        std::lock_guard<std::mutex> lock(mutex_);
        traceCommandLocked(c,clock(),BoundedExecutionTrace::Kind::CommandPublishFailed,begin);throw;
      }
      {std::lock_guard<std::mutex> lock(mutex_);
        const bool refreshed=last_delivered_local_command_&&
          last_delivered_local_command_->execution_id==c.execution_id&&
          sameExecutionTask(last_delivered_local_command_->version,c.version)&&
          last_delivered_local_command_->sequence==c.sequence;
        ++local_commands_published_;
        if(refreshed)++local_evidence_updates_;
        else if(staged.origin==CommandOrigin::Body)++local_body_completed_commands_;
        else if(staged.origin==CommandOrigin::Commit)++local_commit_completed_commands_;
        else if(staged.origin==CommandOrigin::Proof)++local_proof_completed_commands_;
        last_delivered_local_command_=c;
        const auto returned_at=clock();
        local_source_to_command_ns_=returned_at-ns(c.source_stamp);
        local_body_to_command_ns_=returned_at-ns(c.body_source_stamp);
        local_max_source_to_command_ns_=std::max(local_max_source_to_command_ns_,local_source_to_command_ns_);
        local_max_body_to_command_ns_=std::max(local_max_body_to_command_ns_,local_body_to_command_ns_);
        traceCommandLocked(c,clock(),BoundedExecutionTrace::Kind::CommandPublishReturned,begin);}
    }
  }
  static bool sameJob(const Job& a,const Job& b) {
    return a.version==b.version&&a.spline.trajectory.traj_id==b.spline.trajectory.traj_id;
  }
  static bool usablePlanningGeometry(const Job& job) {
    if(!job.support||job.support->version!=job.version)return false;
    const auto& s=*job.support;
    if(s.stair_model_hash.empty()&&s.stair_model_surfaces.empty()&&s.stair_model_obstacles.empty())
      return s.verified; // Exact historical literal-only contract.
    // A checked explicit prior is usable planning geometry, not a declaration
    // that its generated surface was measured. Keep the wire verified flag
    // untouched and pin the immutable compiled model to this exact job.
    const auto& geometry=job.geometry_support;
    return geometry&&geometry->valid&&geometry->model()&&geometry->source.version==job.version&&
      geometry->source.support_hash==s.support_hash&&geometry->source.frame_id==s.frame_id&&
      geometry->source.stair_model_hash==s.stair_model_hash;
  }
  static bool splicePlanningSupportCompatible(const Job& incumbent,const Job& candidate) {
    const auto declares_stair=[](const Job& job) {
      return (job.support&&job.support->required_mode=="stair")||
        (job.geometry_support&&job.geometry_support->isStair());
    };
    // Legacy flat prefix/tail queries share the unchanged upright model.
    if(!declares_stair(incumbent)&&!declares_stair(candidate))return true;
    // One legacy splice query uses one attitude provider for both pieces. It
    // may only do so for the SAME immutable stair source/semantic segment.
    // Ordinary current-state rolling candidates do not enter this branch;
    // rejecting a cross-surface splice does not retire the incumbent.
    const auto& a=incumbent.geometry_support;const auto& b=candidate.geometry_support;
    if(!a||!b||!a->isStair()||!b->isStair()||
       a->source.version!=incumbent.version||b->source.version!=candidate.version)return false;
    return a->source.support_hash==b->source.support_hash&&
      a->source.stair_model_hash==b->source.stair_model_hash&&
      a->source.support_map_sha256==b->source.support_map_sha256&&
      a->source.frame_id==b->source.frame_id&&
      a->source.version.segment_id==b->source.version.segment_id&&
      a->source.floor_id==b->source.floor_id&&
      a->source.segment_kind==b->source.segment_kind&&
      a->source.required_mode==b->source.required_mode&&
      a->source.body_reference_height_m==b->source.body_reference_height_m&&
      a->source.support_xy_radius_m==b->source.support_xy_radius_m;
  }
  void selectCandidateLocked(Job incoming) {
    candidate_=std::move(incoming);queued_candidate_.reset();candidate_offer_deadline_ns_=0;
    if(local_follower_) {
      if(selected_candidate_publication_)execution_trace_.selectedOverwritten();
      selected_candidate_publication_=candidate_->spline;
      selected_candidate_enqueued_steady_ns_=BoundedExecutionTrace::steadyNowNs();
    }
  }
  void advanceCandidateLocked() {
    if(!local_follower_||!queued_candidate_)return;
    auto next=std::move(*queued_candidate_);selectCandidateLocked(std::move(next));
  }
  void clearCandidatesLocked() {
    candidate_.reset();queued_candidate_.reset();selected_candidate_publication_.reset();
    candidate_offer_deadline_ns_=0;
  }
  void expireCandidateOfferLocked(std::int64_t now) {
    // Explicit retirement may mark a selected offer for release. Completed
    // formal geometry does not assign this field a proof/source expiry.
    if(local_follower_&&candidate_offer_deadline_ns_>0&&now>=candidate_offer_deadline_ns_)
      advanceCandidateLocked();
  }
  void publishSelectedCandidate() {
    if(!local_follower_)return;
    // Sole timer consumer; never called by control, solving or map readers.
    // Serialize only outbound delivery, outside the body/proof ledger. Cancel
    // clears the pending slot; an already in-flight old packet is still fenced
    // by its original task/proposal identity and cannot authorize motion.
    std::lock_guard<std::mutex> publication(selected_candidate_publication_mutex_);
    std::optional<ew::TaggedBspline> selected;
    std::int64_t queued_at=0;
    BoundedExecutionTrace::Retention retention;
    std::array<std::uint64_t,5> before{};const auto publish_begin=BoundedExecutionTrace::steadyNowNs();
    {std::lock_guard<std::mutex> lock(mutex_);selected=std::move(selected_candidate_publication_);
      queued_at=selected_candidate_enqueued_steady_ns_;
      selected_candidate_publication_.reset();
      retention.set(task_domain_?&*task_domain_:nullptr,!task_closed_);
      if(selected)execution_trace_.attempt([&] {
        const auto selected_version=executionVersion(*selected);
        before=execution_trace_.capture(traceContextLocked({&selected_version,selected->trajectory.traj_id,{}}));});}
    if(selected) {
      if(selected_candidate_pub_)selected_candidate_pub_->publish(*selected);
      if(selected_candidate_observer_)selected_candidate_observer_(*selected);
      execution_trace_.attempt([&] {
        BoundedExecutionTrace::Event event;event.kind=BoundedExecutionTrace::Kind::CandidatePublishReturned;
        event.retention=retention;
        event.begin_steady_ns=publish_begin;event.before_identity=before;
        event.queue_steady_ns=queued_at;
        event.ros_ns=node_?node_->now().nanoseconds():0;event.source_ns=ns(selected->join_source_stamp);
        const auto selected_version=executionVersion(*selected);
        execution_trace_.record(event,{{&selected_version,selected->trajectory.traj_id,{}},{},{},{},{}});
      });
      admissionWorkPublished();
    }
  }
  struct MeasuredJoin {
    std::optional<double> time;std::optional<MeasuredCurveDomain> remaining;
    NoMotionPreviewProgress preview;double residual{std::numeric_limits<double>::quiet_NaN()};
    MeasuredConnectionPolicy connection{MeasuredConnectionPolicy::CandidateAdmission};
    CandidateAcceptanceOutcome candidate_outcome{CandidateAcceptanceOutcome::NotAttempted};
    CurveCheckCompletion completion{CurveCheckCompletion::Uncertified};
    std::int64_t progress_source_ns{0};bool source_pair_missing{false};
  };
  bool rollingCandidateConnection(const Job& job)const {
    return local_follower_&&!planning_only_geometry_&&!job.spline.splice.enabled;
  }
  // Candidate geometry has one immutable entry, even when a newer real body
  // sample causes its proof to be recomputed. Never ratchet this domain using
  // job.measured_time: repeated checks could otherwise walk to a remote branch.
  static bool rollingOrigin(const Job& job,UniformBspline& curve,MeasuredBodyPose& origin,
      double& origin_time,double& origin_arc) {
    const auto& s=job.spline;
    origin={Eigen::Vector3d(s.join_pose.position.x,s.join_pose.position.y,s.join_pose.position.z),
      Eigen::Quaterniond(s.join_pose.orientation.w,s.join_pose.orientation.x,
        s.join_pose.orientation.y,s.join_pose.orientation.z),ns(s.join_source_stamp)*1e-9,s.frame_id};
    origin_time=s.valid_start_time;origin_arc=s.valid_start_arc_length;
    if(ns(s.join_source_stamp)<=0||origin.frame.empty()||!origin.position.allFinite()||
       !origin.orientation.coeffs().allFinite()||origin.orientation.norm()<1e-8||
       !std::isfinite(origin_time)||origin_time<0.||origin_time>curve.getTimeSum()||
       !std::isfinite(origin_arc)||origin_arc<0.)return false;
    const auto point=curve.evaluateDeBoorT(origin_time);
    return point.allFinite()&&(point-origin.position).norm()<=RollingCurveConnectionPolicy::max_connector_m+1e-9&&
      std::abs(curveArcTo(curve,origin_time)-origin_arc)<=RollingCurveConnectionPolicy::numerical_origin_tolerance_m;
  }
  static std::optional<MeasuredCurveDomain> rollingCandidateDomain(const Job& job,
      UniformBspline& curve,const MeasuredBodyPose& body,double resolution,
      ValidationCycle::Clock::time_point deadline,double& residual,CurveCheckCompletion& completion) {
    completion=CurveCheckCompletion::InvalidInput;
    MeasuredBodyPose origin;double origin_time=0.,origin_arc=0.;
    if(!rollingOrigin(job,curve,origin,origin_time,origin_arc)||body.frame!=origin.frame||
       !body.position.allFinite()||!body.orientation.coeffs().allFinite()||body.orientation.norm()<1e-8||
       !std::isfinite(body.source_stamp)||body.source_stamp<=0.||
       !std::isfinite(resolution)||resolution<=0.)return {};
    // Collision refresh consumes Native's independently acquired body sample.
    // Its topic may trail the solve input: query that real connector while the
    // existing body-age budget is valid, without restamping or waiting on an
    // unrelated topic ordering. The final follower receipt still requires a
    // current sample no older than the immutable candidate origin and proof.
    if(std::abs(body.source_stamp-origin.source_stamp)<=1e-9&&
       ((body.position-origin.position).norm()>1e-8||
        body.orientation.normalized().angularDistance(origin.orientation.normalized())>1e-8))return {};
    if(ValidationCycle::Clock::now()>=deadline) {
      completion=CurveCheckCompletion::BudgetExhausted;return {};
    }
    auto derivative=curve.getDerivative();const double speed=derivativeControlBound(derivative);
    if(!std::isfinite(speed))return {};
    const double duration=curve.getTimeSum();
    const double dt=std::min(.02,resolution*.125/std::max(speed,.01));
    const double extent=std::min(RollingCurveConnectionPolicy::max_forward_arc_m,
      (body.position-origin.position).norm()+2.*resolution);
    Eigen::Vector3d previous=curve.evaluateDeBoorT(origin_time);
    double best_time=origin_time,best_square=(body.position-previous).squaredNorm(),arc=0.;
    std::size_t samples=0;
    completion=CurveCheckCompletion::Uncertified;
    for(double previous_time=origin_time;previous_time<duration&&arc<extent;) {
      if(++samples>10000||ValidationCycle::Clock::now()>=deadline) {
        completion=CurveCheckCompletion::BudgetExhausted;return {};
      }
      const double time=std::min(duration,previous_time+dt);
      if(time==previous_time)return {};
      const Eigen::Vector3d point=curve.evaluateDeBoorT(time),delta=point-previous;
      if(!point.allFinite())return {};
      const double length=delta.norm();
      if(length>1e-12) {
        const double upper=std::clamp((extent-arc)/length,0.,1.);
        const double fraction=std::clamp((body.position-previous).dot(delta)/delta.squaredNorm(),0.,upper);
        const double projected=previous_time+fraction*(time-previous_time);
        const double square=(body.position-curve.evaluateDeBoorT(projected)).squaredNorm();
        if(square<best_square){best_square=square;best_time=projected;}
      }
      arc+=length;previous=point;previous_time=time;
    }
    residual=std::sqrt(best_square);
    if(residual>RollingCurveConnectionPolicy::max_connector_m+1e-9)return {};
    const double measured_arc=curveArcTo(curve,best_time);
    if(ValidationCycle::Clock::now()>=deadline) {
      completion=CurveCheckCompletion::BudgetExhausted;return {};
    }
    if(measured_arc<origin_arc-1e-6||
       measured_arc>origin_arc+RollingCurveConnectionPolicy::max_forward_arc_m+1e-6)return {};
    completion=CurveCheckCompletion::Clear;
    return MeasuredCurveDomain{best_time,measured_arc,0.,duration};
  }
  static bool rollingCommitDomain(const Job& job,UniformBspline& curve,
      const ew::TrajectoryValidation& proof,const ew::LocalTrajectoryCommit& commit,
      const Eigen::Vector3d& body) {
    MeasuredBodyPose origin;double origin_time=0.,origin_arc=0.;
    if(!rollingOrigin(job,curve,origin,origin_time,origin_arc)||ns(proof.body_source_stamp)<=0||
       ns(commit.body_source_stamp)<ns(job.spline.join_source_stamp)||
       ns(commit.body_source_stamp)<ns(proof.body_source_stamp)||
       !std::isfinite(proof.valid_start_time)||proof.valid_start_time<origin_time||
       proof.valid_start_time>curve.getTimeSum()||!std::isfinite(proof.valid_start_arc_length)||
       proof.valid_start_arc_length<origin_arc-1e-6||
       proof.valid_start_arc_length>origin_arc+RollingCurveConnectionPolicy::max_forward_arc_m+1e-6||
       std::abs(curveArcTo(curve,proof.valid_start_time)-proof.valid_start_arc_length)>
         RollingCurveConnectionPolicy::numerical_origin_tolerance_m)return false;
    const double elapsed=(ns(commit.body_source_stamp)-ns(proof.body_source_stamp))*1e-9;
    if(elapsed>RollingCurveConnectionPolicy::max_source_gap_s)return false;
    const double speed=derivativeControlBound(curve.getDerivative());
    if(!std::isfinite(speed))return false;
    const double travel=speed*elapsed+RollingCurveConnectionPolicy::max_connector_m;
    const double low=std::max(origin_arc,proof.valid_start_arc_length-std::min(.15,travel));
    const double high=std::min(origin_arc+RollingCurveConnectionPolicy::max_forward_arc_m,
      proof.valid_start_arc_length+std::min(RollingCurveConnectionPolicy::max_forward_arc_m,travel));
    const double actual_arc=curveArcTo(curve,commit.curve_time);
    return commit.curve_time>=origin_time&&actual_arc>=low-1e-6&&actual_arc<=high+1e-6&&
      std::abs(actual_arc-commit.arc_length)<=RollingCurveConnectionPolicy::numerical_origin_tolerance_m&&
      (body-curve.evaluateDeBoorT(commit.curve_time)).norm()<=RollingCurveConnectionPolicy::max_connector_m+1e-9;
  }
  static const ew::TrackingProgress* progressAtBodySource(const Job& job,
      std::int64_t now,std::int64_t body_source_ns) {
    // Each Job owns exactly one immutable curve/version. Never fall through
    // into another committed job, change a source stamp or lower its high water.
    if(body_source_ns<=0)return nullptr;
    if(job.progress&&ns(job.progress->header.stamp)<=body_source_ns) {
      const auto& p=*job.progress;
      return p.valid&&ns(p.header.stamp)>0&&now-ns(p.header.stamp)<=400000000LL?&p:nullptr;
    }
    const auto p=std::find_if(job.progress_history.rbegin(),job.progress_history.rend(),
      [&](const auto& value){return ns(value.header.stamp)<=body_source_ns;});
    return p!=job.progress_history.rend()&&p->valid&&ns(p->header.stamp)>0&&
      now-ns(p->header.stamp)<=400000000LL?&*p:nullptr;
  }
  void logMissingSourcePair(std::int64_t trajectory,const char* phase,
      std::int64_t body_source_ns,std::int64_t progress_source_ns) {
    const auto log_now=std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    auto previous=proof_log_steady_ns_.load();
    if(node_&&log_now-previous>=1000000000LL&&
       proof_log_steady_ns_.compare_exchange_strong(previous,log_now))
      RCLCPP_WARN(node_->get_logger(),
        "Native source pairing unavailable: curve=%ld phase=%s body_source_ns=%ld progress_source_ns=%ld; original proof expiry unchanged",
        trajectory,phase,body_source_ns,progress_source_ns);
  }
  bool consumeSourcePairBodyReadyLocked(std::int64_t now) {
    if(!source_pair_wait_)return false;
    const auto& wait=*source_pair_wait_;
    const bool current=local_follower_&&!planning_only_geometry_&&!task_closed_&&active_&&
      active_->version==wait.version&&active_->spline.trajectory.traj_id==wait.trajectory_id&&
      motion_stop_barrier_==wait.stop_barrier&&active_->proof&&active_->proof->valid&&
      active_->proof->sequence==wait.proof_sequence&&now<wait.original_deadline_ns;
    if(!current){source_pair_wait_.reset();return false;}
    if(body_history_.empty()||body_history_.back().source_ns<wait.required_body_source_ns)return false;
    const auto source=body_history_.back().source_ns;
    if(source>now+20000000LL||now-source>400000000LL)return false;
    source_pair_wait_.reset();return true;
  }
  void awaitSourcePairBody(const Job& job,std::int64_t acquired_body_source_ns,std::int64_t now) {
    bool ready=false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Only a live committed curve whose projection got ahead of independent
      // odometry can request this one-shot wake. Stale/missing projections,
      // candidates and old copied jobs remain ordinary inconclusive reads.
      if(!local_follower_||planning_only_geometry_||task_closed_||!active_||!job.progress||
          !job.progress->valid||active_->version!=job.version||
          active_->spline.trajectory.traj_id!=job.spline.trajectory.traj_id||
          ns(job.progress->header.stamp)<=acquired_body_source_ns||
          !active_->proof||!active_->proof->valid||!curveVerdictCurrent(*active_->proof,now))return;
      const auto required=ns(job.progress->header.stamp);
      const auto deadline=local_follower_?required+400000000LL:
        std::min<std::int64_t>(ns(active_->proof->valid_until),required+400000000LL);
      if(required<=0||required>now+20000000LL||deadline<=now)return;
      source_pair_wait_=SourcePairBodyWait{job.version,job.spline.trajectory.traj_id,required,
        deadline,active_->proof->sequence,motion_stop_barrier_};
      // The missing body can arrive between the reader's immutable input copy
      // and registration. Consume that already-arrived original sample too.
      ready=consumeSourcePairBodyReadyLocked(now);
    }
    if(ready)snapshotPublished();
  }
  bool committedCollisionMatchesLocked(const ew::TrajectoryValidation& p)const {
    return active_&&active_->version==p.version&&active_->proposal==p.proposal_id&&
      active_->spline.trajectory.traj_id==p.trajectory_id&&active_->support&&
      usablePlanningGeometry(*active_)&&active_->support->version==p.version&&
      active_->support->support_hash==p.support_hash&&
      active_->support->support_reference_id==p.support_reference_id&&
      !p.valid&&p.reason=="occupied"&&(p.whole_curve||p.remaining_curve)&&
      p.frame_id==frame_&&p.transport_mode==mode_&&p.collision_policy==geometry_policy_;
  }
  // Called only after the real snapshot query and its source/order checks.
  // A single latest event is retained until consumed. Repeated occupied
  // refreshes for the same curve do not cancel the worker every 50 ms; a real
  // positive proof rearms the edge, and a replacement curve has a new edge.
  bool recordCommittedProofLocked(const ew::TrajectoryValidation& p) {
    if(!active_||active_->version!=p.version||active_->spline.trajectory.traj_id!=p.trajectory_id||
       (active_->proof&&!proofSupersedes(p,*active_->proof)))return false;
    active_->proof=p;
    if(p.valid) {committed_collision_.reset();last_committed_collision_.reset();}
    else if(committedCollisionMatchesLocked(p)) {
      const auto identity=std::make_pair(p.version,p.trajectory_id);
      if(!last_committed_collision_||*last_committed_collision_!=identity)committed_collision_=p;
    } else {
      // Keep only the pending occupied wake through a pure source-expiry
      // refresh. Its own original lifetime/identity is never renewed, and p
      // remains the current (invalid) execution verdict above.
      const bool pending_obstacle=p.reason=="source_expired_during_check"&&committed_collision_&&
        committedCollisionMatchesLocked(*committed_collision_)&&
        p.proposal_id==committed_collision_->proposal_id&&p.splice==committed_collision_->splice&&
        p.support_hash==committed_collision_->support_hash&&
        p.support_reference_id==committed_collision_->support_reference_id&&
        p.frame_id==committed_collision_->frame_id&&p.transport_mode==committed_collision_->transport_mode&&
        p.collision_policy==committed_collision_->collision_policy&&
        p.sequence>=committed_collision_->sequence&&
        ns(p.check_end)>=ns(committed_collision_->check_end)&&
        ns(p.check_end)<ns(committed_collision_->valid_until);
      if(!pending_obstacle)committed_collision_.reset();
    }
    return true;
  }
  static bool previewCandidateProjectionInconclusive(const MeasuredJoin& connection) {
    return connection.completion==CurveCheckCompletion::BudgetExhausted||
        (connection.connection==MeasuredConnectionPolicy::PreviewCandidateGeometry&&
         connection.candidate_outcome==CandidateAcceptanceOutcome::PreviewBudgetExhausted);
  }
  void observeCommittedPreviewBody(const MeasuredBodyPose& body,std::int64_t source_ns,std::int64_t now) {
    if(!planning_only_geometry_||writer_handoff_||now<=0||source_ns<=0||
       source_ns>now+20000000LL||now-source_ns>400000000LL||
       std::abs(body.source_stamp-source_ns*1e-9)>1e-6)return;
    double yaw=0.;if(!measuredBodyYaw(body,frame_,now*1e-9,.4,yaw))return;
    std::optional<Job> job;
    {std::lock_guard<std::mutex> l(mutex_);
      if(task_closed_||!active_||!active_->no_motion_preview||!active_->preview_progress||
         (latest_permit_&&latest_permit_->revoked))return;
      job=active_;
    }
    const auto& seed=job->observed_preview_progress?*job->observed_preview_progress:*job->preview_progress;
    if(body.source_stamp<=seed.source_stamp+1e-9)return;
    const auto& s=job->spline.trajectory;
    if(s.order!=3||s.pos_pts.size()<4||s.pos_pts.size()>10000||
       s.knots.size()!=s.pos_pts.size()+4||job->spline.frame_id!=frame_||
       !std::isfinite(job->preview_projection_resolution)||job->preview_projection_resolution<=0.)return;
    Eigen::MatrixXd points(3,s.pos_pts.size());Eigen::VectorXd knots(s.knots.size());
    for(std::size_t i=0;i<s.pos_pts.size();++i)
      points.col(i)=Eigen::Vector3d(s.pos_pts[i].x,s.pos_pts[i].y,s.pos_pts[i].z);
    for(std::size_t i=0;i<s.knots.size();++i)knots[i]=s.knots[i];
    if(!points.allFinite()||!knots.allFinite())return;
    for(int i=1;i<knots.size();++i)if(knots[i]<=knots[i-1])return;
    UniformBspline curve(points,3,.1);curve.setKnot(knots);
    // Same production projection limits/budget, on this immutable geometry;
    // no live GridMap read or new reader/slot is involved.
    const auto projected=projectCommittedNoMotionPreview(curve,body.position,body.source_stamp,seed,
        job->preview_projection_resolution);
    if(!advanceNoMotionPreviewObservation(seed,projected))return;
    std::lock_guard<std::mutex> l(mutex_);
    if(task_closed_||!active_||!active_->no_motion_preview||!active_->preview_progress||
       active_->version!=job->version||active_->spline.trajectory.traj_id!=job->spline.trajectory.traj_id||
       (latest_permit_&&latest_permit_->revoked))return;
    const auto& current=active_->observed_preview_progress?*active_->observed_preview_progress:*active_->preview_progress;
    if(!sameNoMotionPreviewSeed(current,seed))return; // cancel/replacement/newer observation wins.
    active_->observed_preview_progress=projected;
  }
  bool positivePreviewCheckStillCurrentLocked(const Job& job)const {
    if(!planning_only_geometry_)return true;
    if(job.no_motion_preview)return
      (active_&&active_->version==job.version&&active_->spline.trajectory.traj_id==job.spline.trajectory.traj_id&&
       active_->no_motion_preview&&active_->preview_progress&&job.preview_progress&&
       sameNoMotionPreviewSeed(*active_->preview_progress,*job.preview_progress));
    if(!job.preview_candidate_origin)return true;
    // Candidate and committed preview are different stages. A concurrent commit
    // or newer certified candidate seed makes this old positive inconclusive;
    // real negative evidence still takes effect in check() below.
    if(active_&&active_->version==job.version&&
       active_->spline.trajectory.traj_id==job.spline.trajectory.traj_id)return false;
    return candidate_&&candidate_->version==job.version&&
      candidate_->spline.trajectory.traj_id==job.spline.trajectory.traj_id&&
      candidate_->preview_candidate_origin&&samePreviewCandidateOrigin(*job.preview_candidate_origin,*candidate_->preview_candidate_origin)&&
      job.support&&usablePlanningGeometry(*candidate_)&&
      job.support->version==candidate_->support->version&&
      job.support->support_hash==candidate_->support->support_hash&&
      job.support->support_reference_id==candidate_->support->support_reference_id&&
      (job.preview_progress&&candidate_->preview_progress?
        sameNoMotionPreviewSeed(*job.preview_progress,*candidate_->preview_progress):
        !job.preview_progress&&!candidate_->preview_progress);
  }
  bool previewReadStillCurrentLocked(const Job& job,const ew::ExecutionPermit& permit,
      std::int64_t body_source_ns,std::int64_t now,std::int64_t end_now,bool needs_replan)const {
    if(!job.support||!job.proof||!positivePreviewCheckStillCurrentLocked(job))return false;
    const auto& expected=job.version;const auto trajectory=job.spline.trajectory.traj_id;
    const auto& proof=*job.proof;
    return !(!active_||active_->version!=expected||active_->spline.trajectory.traj_id!=trajectory||
       !usablePlanningGeometry(*active_)||active_->support->version!=expected||
       active_->support->support_hash!=job.support->support_hash||
       active_->support->support_reference_id!=job.support->support_reference_id||
       !active_->proof||!active_->proof->whole_curve||active_->proof->remaining_curve||
       active_->proof->version!=expected||active_->proof->proposal_id!=job.proposal||
       active_->proof->trajectory_id!=trajectory||active_->proof->frame_id!=frame_||
       active_->proof->transport_mode!=mode_||active_->proof->collision_policy!=geometry_policy_||
       active_->proof->support_reference_id!=job.support->support_reference_id||
       active_->proof->support_hash!=job.support->support_hash||active_->proof->sequence<proof.sequence||
       body_source_ns+2000<ns(active_->proof->body_source_stamp)||
       (!needs_replan&&(!active_->proof->valid||ns(active_->proof->valid_until)<=end_now))||
       !latest_permit_||latest_permit_->version!=expected||latest_permit_->trajectory_id!=trajectory||
       latest_permit_->allowed||latest_permit_->revoked||(!needs_replan&&!latest_permit_->geometry_committed)||
       latest_permit_->phase!="preview"||latest_permit_->frame_id!=frame_||latest_permit_->transport_mode!=mode_||
       (!needs_replan&&active_->proof->sequence<latest_permit_->validation_sequence)||
       ns(latest_permit_->source_stamp)<=0||ns(latest_permit_->source_stamp)>end_now+20000000LL||
       end_now-ns(latest_permit_->source_stamp)>750000000LL||ns(latest_permit_->valid_until)<=end_now||
       ns(permit.valid_until)<=end_now||(!needs_replan&&ns(proof.valid_until)<=end_now)||
       now>end_now||end_now-body_source_ns>400000000LL||
       (!needs_replan&&end_now-ns(proof.body_source_stamp)>400000000LL));
  }
  MeasuredJoin measuredJoin(const Job& job,UniformBspline& curve,const MeasuredBodyPose& body,
      double resolution,double now,
      ValidationCycle::Clock::time_point projection_deadline=ValidationCycle::Clock::time_point::max(),
      std::int64_t acquired_body_source_ns=0)const {
    MeasuredJoin out;
    if(planning_only_geometry_&&job.no_motion_preview) {
      out.connection=MeasuredConnectionPolicy::CommittedNoMotionPreview;
      if(job.preview_progress) {
        const auto& seed=job.observed_preview_progress?*job.observed_preview_progress:*job.preview_progress;
        out.preview=projectCommittedNoMotionPreview(
          curve,body.position,body.source_stamp,seed,resolution,10000,.005,projection_deadline);
      }
      out.residual=out.preview.residual;
      if(out.preview.connected())out.time=out.preview.curve_time;
      if(out.preview.status==NoMotionPreviewStatus::BudgetExhausted)out.completion=CurveCheckCompletion::BudgetExhausted;
    } else if(planning_only_geometry_&&job.preview_candidate_origin) {
      out.connection=MeasuredConnectionPolicy::PreviewCandidateGeometry;
      const auto result=projectPreviewCandidateGeometryWhileAllowed(curve,*job.preview_candidate_origin,
          body,resolution,[&]{return ValidationCycle::Clock::now()<projection_deadline;},
          PreviewCandidateProjectionPhase::CandidateRevalidation);
      out.candidate_outcome=result.outcome;
      if(result.geometry) {
        out.preview=result.geometry->projection;out.residual=out.preview.residual;
        out.time=out.preview.curve_time;
      }
    } else if(job.progress&&job.progress->valid) {
      const auto progress=progressAtBodySource(job,static_cast<std::int64_t>(now*1e9),acquired_body_source_ns);
      if(!progress){out.source_pair_missing=true;return out;}
      out.progress_source_ns=ns(progress->header.stamp);
      out.connection=MeasuredConnectionPolicy::CommittedSweptConnection;
      out.remaining=measuredRemainingCurveDomain(curve,body.position,progress->curve_time,
        (acquired_body_source_ns-out.progress_source_ns)*1e-9,
        std::max(progress->s_committed,job.progress->s_committed),.3,.15,
        out.connection,&out.residual);
      if(out.remaining)out.time=out.remaining->measured_time;
    } else if(rollingCandidateConnection(job)) {
      out.connection=MeasuredConnectionPolicy::RollingCandidateConnection;
      const auto domain=rollingCandidateDomain(job,curve,body,resolution,projection_deadline,
        out.residual,out.completion);
      if(domain) {
        out.time=domain->measured_time;
        // This is a new whole candidate, not an already-committed suffix.
        out.preview.curve_time=domain->measured_time;out.preview.arc_length=domain->measured_arc;
      }
    } else out.time=measuredPreviewCurveTime(curve,body.position,job.measured_time,resolution,10000,.005,&out.completion);
    return out;
  }
  struct InitialIntent {ew::ExecutionPermit permit;Job geometry;};
  void resetTaskLeaseDomainLocked() {
    // BT creates one TaskExecutionAuthority per task; its lease sequence is
    // task-scoped, unlike the follower's publisher-wide command/permit IDs.
    // Call only after admitting a DIFFERENT task domain, never on an ordinary
    // HOLD, source gap, new reference window or same-task lease recovery.
    task_lease_.reset();task_lease_history_.clear();task_lease_sequence_=0;
    current_check_completed_steady_ns_=current_checked_fusion_progress_steady_ns_=0;
    current_checked_fusion_=0;pipeline_fault_=false;
  }
  bool referenceTaskOpenLocked(const ew::ExecutionVersion& version) const {
    if(task_domain_&&((!sameExecutionTask(*task_domain_,version)&&!task_closed_)||
       (sameExecutionTask(*task_domain_,version)&&task_closed_)))return false;
    // A cancellation closes the task, not merely its last reference window.
    // A larger generation cannot resurrect it before/without an owner permit.
    return !retired_||!sameExecutionTask(*retired_,version);
  }
  bool enterWriterDomainLocked(const ew::ExecutionPermit& p) {
    if(p.revoked)return true;
    if(p.execution_id.empty())return !writer_domain_&&(!latest_permit_||
      !sameExecutionTask(latest_permit_->version,p.version)||latest_permit_->execution_id.empty());
    const bool bound=!p.sdk_session.empty()&&p.sdk_arm_generation>0;
    const bool unbound=p.sdk_session.empty()&&p.sdk_arm_generation==0;
    if(!bound&&!unbound)return false;
    // The owner allocates an execution ID/epoch before SDK admission. Its
    // disallowed geometry/arming ingress is not a writer incarnation and
    // must not pin an empty SDK session that rejects the first real grant.
    if(!writer_domain_) {
      if(latest_permit_&&sameExecutionTask(latest_permit_->version,p.version)&&
         !latest_permit_->execution_id.empty()) {
        const auto& pending=*latest_permit_;
        const bool newer_hold=!p.allowed&&p.control_epoch>pending.control_epoch&&
          (p.phase=="arming"||p.phase=="holding");
        const bool same_execution=p.execution_id==pending.execution_id&&p.control_epoch==pending.control_epoch;
        const bool same_geometry=p.version==pending.version&&p.trajectory_id==pending.trajectory_id;
        const bool current_geometry=p.geometry_committed&&active_&&p.version==active_->version&&
          p.trajectory_id==active_->spline.trajectory.traj_id;
        if(pending.allowed||writer_commit_sequence_||(!newer_hold&&
           (!same_execution||(!unbound&&!same_geometry&&!current_geometry))))return false;
      }
      if(unbound)return !p.allowed;
      writer_domain_=p;return true;
    }
    const auto& old=*writer_domain_;
    if(p.execution_id==old.execution_id&&p.control_epoch==old.control_epoch&&
       p.sdk_session==old.sdk_session&&p.sdk_arm_generation==old.sdk_arm_generation)return true;
    if(p.allowed||p.control_epoch<=old.control_epoch||(p.phase!="arming"&&p.phase!="holding"))return false;
    // A newer disallowed arming epoch may retire the old CAS state, but its
    // SDK identity is pinned only once it is actually populated. The latest
    // owner ingress fences delayed permits from the earlier execution.
    writer_domain_.reset();if(bound)writer_domain_=p;
    writer_commit_sequence_=ack_sequence_=0;initial_writer_ack_.reset();initial_intents_.clear();
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
    active_.reset();local_transition_incumbent_.reset();pending_committed_display_.reset();prepared_.reset();tombstone_job_.reset();grant_.reset();tombstone_grant_.reset();
    local_prepare_request_.reset();local_commit_fact_.reset();
    prepared_demand_.reset();latest_demand_.reset();pending_local_demand_.reset();blocked_entry_.reset();entry_reseed_.reset();
    committed_collision_.reset();last_committed_collision_.reset();
    last_entry_reseed_.reset();last_terminal_reseed_.reset();last_deviation_reseed_.reset();committed_jobs_.clear();
    last_preview_candidate_reseed_.reset();
    permits_.clear();motion_proofs_.clear();prepared_motion_proofs_.clear();tombstone_motion_proofs_.clear();
    initial_writer_ack_.reset();initial_intents_.clear();writer_domain_.reset();
    writer_commit_sequence_=ack_sequence_=0;grant_retired_=grant_applied_=tombstone_retired_=false;
    if(candidate_&&!sameExecutionTask(candidate_->version,p.version))clearCandidatesLocked();
    if(candidate_reference_&&!sameExecutionTask(*candidate_reference_,p.version))candidate_reference_.reset();
    proofs_.erase(std::remove_if(proofs_.begin(),proofs_.end(),[&](const auto& proof){
      return !sameExecutionTask(proof.version,p.version);}),proofs_.end());
    task_domain_=p.version;task_closed_=false;++motion_stop_barrier_;++prepared_stop_barrier_;
    resetTaskLeaseDomainLocked();local_proof_facts_.clear();
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
  void queueCommittedDisplayLocked() {
    // Fixed one-slot mailbox: the next committed geometry supersedes an
    // unsent display, never an execution fact. Copy once under the commit
    // lock; no publisher, source restamp, or mutable Job escapes this lock.
    pending_committed_display_=std::make_shared<const ew::TaggedBspline>(active_->spline);
  }
  void dispatchCommittedDisplay(const std::function<void(const ew::TaggedBspline&)>& emit) {
    std::shared_ptr<const ew::TaggedBspline> view;
    {std::lock_guard<std::mutex> l(mutex_);view=std::move(pending_committed_display_);}
    // The independent mutually-exclusive timer is the sole consumer. An
    // already dispatched display can finish after cancellation; execution_view
    // selects only its exact current permit identity and never uses this topic
    // to authorize movement. Never reacquire the ledger around middleware I/O.
    if(view)emit(*view);
  }
  void run() {
    if(local_follower_){runLocalFollowerValidation();return;}
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
          if(validationReadRequiresObservedSnapshot(geometry_policy_,braking_.has_value(),
              snapshot->planningCollisionQueryPolicy()))
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
        } else {
          std::optional<ew::MotionDemand> demand;
          {std::lock_guard<std::mutex> l(mutex_);demand=latest_demand_;}
          motion_diagnostics_.record(MotionCheckStage::SnapshotUnavailable,demand,node_->now().nanoseconds());
        }
        publishMotionDiagnostics();
      } catch(const std::bad_alloc&) {std::terminate();}
      catch(const std::exception& e){RCLCPP_ERROR_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,"Native validation failed: %s",e.what());}
      // 20 Hz scheduling ceiling, not a claim of 20 Hz map fusion/full checks.
      std::unique_lock<std::mutex> lock(wait_mutex_);
      wake_.wait_until(lock,begin+std::chrono::milliseconds(50),[&]{return stop_.load()||snapshot_ready_;});
      snapshot_ready_=false;
    }
  }
  void drainExecutionTraces(std::size_t page_limit=8) noexcept {
    const auto drain=[&](const BoundedExecutionTrace& trace,std::uint64_t& cursor,
        d1max_diagnostics::BoundedTaskDiagnosticFile& file,bool& path_reported,bool& failure_reported) {
      try {
        trace.drainPages(cursor,[&](const std::string& batch,const BoundedExecutionTrace::Retention& retention) {
          const bool complete=retention.session.complete()&&retention.task.complete();
          const auto key=complete?d1max_diagnostics::BoundedTaskDiagnosticFile::taskKey(
            retention.session.str(),retention.task.str()):std::string{};
          const bool written=file.write(batch,key,complete&&retention.active,retention.steady_ns);
          if(written&&!path_reported) {path_reported=true;
            if(node_)RCLCPP_INFO(node_->get_logger(),"Local execution trace: %s",file.directory().c_str());}
          if(!written&&!failure_reported) {failure_reported=true;
            if(node_)RCLCPP_WARN(node_->get_logger(),"Local execution trace sink unavailable path=%s failed=%lu rejected=%lu disabled=%d (navigation unchanged)",
              file.directory().c_str(),file.failedWrites(),file.rejectedBatches(),file.disabled());}
          return written;
        },page_limit);
      } catch(...) {
        if(!failure_reported) {failure_reported=true;
          try {if(node_)RCLCPP_WARN(node_->get_logger(),"Local execution trace output failed (navigation unchanged)");}
          catch(...) {}}
      }
    };
    drain(execution_trace_,last_execution_trace_sequence_,execution_trace_file_,
      execution_trace_path_reported_,execution_trace_failure_reported_);
    drain(curve_check_trace_,last_curve_check_trace_sequence_,curve_check_trace_file_,
      curve_check_trace_path_reported_,curve_check_trace_failure_reported_);
  }
  struct CurveCheckObservation {
    BoundedExecutionTrace::Event event;
    explicit CurveCheckObservation(bool current) {
      event.kind=current?BoundedExecutionTrace::Kind::CurrentCheckFinished:
        BoundedExecutionTrace::Kind::AdmissionCheckFinished;
      event.begin_steady_ns=BoundedExecutionTrace::steadyNowNs();
      event.check.disposition.set("check_exception");
    }
  };
  // Stack-only completion guard: one record even when a check is discarded or
  // output throws. Its independent ring cannot overwrite command/proof events.
  struct CurveCheckTraceScope {
    ExecutionValidator& owner;const Job* job;CurveCheckObservation& observation;
    void finish() noexcept {
      if(!owner.local_follower_||!job)return;
      owner.curve_check_trace_.attempt([&] {
        if(owner.node_)observation.event.ros_ns=owner.node_->now().nanoseconds();
        owner.curve_check_trace_.record(observation.event,
          {{&job->version,job->spline.trajectory.traj_id,job->proposal},{},{},{},{}});
      });
      job=nullptr;
    }
    ~CurveCheckTraceScope() noexcept {finish();}
  };
  void captureCheckObservationLocked(CurveCheckObservation& observation,const std::optional<Job>& job)const noexcept {
    auto& e=observation.event;e.retention.set(task_domain_?&*task_domain_:nullptr,!task_closed_);
    e.body_ns=e.native_body_ns=body_history_.empty()?0:body_history_.back().source_ns;
    if(job&&job->proof){e.validation_floor=job->proof->sequence;e.proof_until_ns=ns(job->proof->valid_until);}
  }
  static void observeSnapshot(CurveCheckObservation* observation,const CollisionSnapshotPool::PublicationTiming& timing) noexcept {
    if(!observation)return;
    auto& e=observation->event;auto& c=e.check;
    e.map_revision=timing.map_revision;
    c.fusion_sequence=timing.fusion.sequence;c.publication_sequence=timing.publication_sequence;
    c.publication_misses=timing.publication_misses;
    c.fusion_begin_ns=timing.fusion.begin_ns;c.fusion_end_ns=timing.fusion.end_ns;
    c.fusion_begin_steady_ns=timing.fusion.begin_steady_ns;c.fusion_end_steady_ns=timing.fusion.end_steady_ns;
    c.publication_begin_steady_ns=timing.begin_steady_ns;c.publication_end_steady_ns=timing.end_steady_ns;
  }
  struct CurrentCurveSnapshotRead {
    CollisionSnapshotPool::PublicationTiming timing;
    std::int64_t front_source_ns{0},rear_source_ns{0};
    std::int64_t lease_begin_steady_ns{0},lease_end_steady_ns{0};
    bool available{false};
  };
  // Confine the exclusive mutable-query lease to the current-curve check.
  // Native query scopes must restore all map-bound providers before calling
  // finish. Ordered proof/command delivery then owns values, not a map lease.
  // A slow output sink must not pin the fast head after geometry is complete.
  template<class Check> CurrentCurveSnapshotRead withCurrentCurveSnapshot(Check&& check,
      CurveCheckObservation* observation=nullptr) {
    CurrentCurveSnapshotRead read;
    if(observation){observation->event.queue_steady_ns=BoundedExecutionTrace::steadyNowNs();
      observation->event.check.disposition.set("snapshot_unavailable");}
    auto snapshot=pool_.borrowValidation(&read.timing);
    if(observation)observation->event.check.borrow_end_steady_ns=BoundedExecutionTrace::steadyNowNs();
    read.available=static_cast<bool>(snapshot);
    if(!snapshot)return read;
    read.front_source_ns=snapshot->integratedRaySourceStamp(0);
    read.rear_source_ns=snapshot->integratedRaySourceStamp(1);
    observeSnapshot(observation,read.timing);
    if(observation){observation->event.source_ns=read.front_source_ns;observation->event.rear_source_ns=read.rear_source_ns;
      observation->event.check.disposition.set("check_not_started");}
    read.lease_begin_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
      ValidationCycle::Clock::now().time_since_epoch()).count();
    if(validationReadRequiresObservedSnapshot(geometry_policy_,false,
        snapshot->planningCollisionQueryPolicy()))snapshot->requireObservedSnapshot();
    // One captured reference keeps this callback inside std::function's
    // small-object buffer. Capturing all three references separately would
    // add a heap allocation to every current-curve check on libstdc++.
    struct FinishContext {GridMap::Ptr& snapshot;CurrentCurveSnapshotRead& read;
      CurveCheckObservation* observation;};
    FinishContext finish_context{snapshot,read,observation};
    const std::function<void()> finish=[&finish_context] {
      if(!finish_context.snapshot)return;
      finish_context.snapshot.reset();
      auto& completed=finish_context.read;
      completed.lease_end_steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
        ValidationCycle::Clock::now().time_since_epoch()).count();
      if(finish_context.observation)
        finish_context.observation->event.check.lease_end_steady_ns=completed.lease_end_steady_ns;
    };
    if constexpr(std::is_invocable_v<Check,GridMap&,const std::function<void()>&>)
      check(*snapshot,finish);
    else check(*snapshot); // Passive/legacy callers retain the whole callback lease.
    finish();
    return read;
  }
  void runLocalFollowerValidation() {
    using Clock=ValidationCycle::Clock;
    auto next_refresh=Clock::now();
    while(!stop_) {
      {
        std::unique_lock<std::mutex> lock(wait_mutex_);
        wake_.wait_until(lock,next_refresh,[&]{return stop_.load()||snapshot_ready_;});
        if(stop_)break;
        snapshot_ready_=false;next_refresh=Clock::now()+ValidationCycle::period;
      }
      const auto begin=Clock::now();
      try {
        // Copy ledger inputs before leasing a map: command publication may
        // briefly own this mutex, but that wait must not pin either snapshot.
        // checkAt still verifies acquired source time and current task identity.
        std::optional<Job> active;MeasuredBodyPose body;CurveCheckObservation observation(true);
        {std::lock_guard<std::mutex> l(mutex_);active=active_;body=body_;
          // Observe true input/component failure even while the follower is
          // holding. An ordinary incomplete map/check does not set this floor.
          if(active)nativePipelineReadyLocked(active->version,node_->now().nanoseconds());
          captureCheckObservationLocked(observation,active);}
        CurveCheckTraceScope trace{*this,active?&*active:nullptr,observation};
        // The pool reserves its newest completed validation head for this
        // reader; the 100 Hz owner must not steal it between renewal rounds.
        // Only an already completed same-context fallback can be borrowed
        // during a writer copy. Neither acquisition renews any source stamp.
        const auto read=withCurrentCurveSnapshot([&](GridMap& snapshot,const std::function<void()>& finish_query) {
          // This independent reader renews SCAN's current curve proof. Commands
          // relay on demand/body delivery and never query a second reaction/
          // braking volume. It also owns complete checks exceeding 50 ms, with
          // the former slow-fallback 120 ms budget. Candidates never queue
          // ahead of current renewal or double its period to 400 ms.
          if(active)ValidationCycle::formalCurrentCurvePass(begin,
            [&](double remaining,Clock::time_point deadline){
              return check(*active,body,snapshot,remaining,true,deadline,finish_query,&observation);},
            []{return Clock::now();});
        },&observation);
        trace.finish();
        if(!read.available) {
          std::optional<ew::MotionDemand> demand;
          {std::lock_guard<std::mutex> l(mutex_);demand=latest_demand_;}
          motion_diagnostics_.record(MotionCheckStage::SnapshotUnavailable,demand,node_->now().nanoseconds());
        }
        // Passive delivery evidence: publication misses alone did not expose
        // the owner/fast-reader contention. Log the selected original stamps
        // and separate borrow misses, including rounds that had no snapshot.
        const auto& timing=read.timing;
        const auto reads=pool_.readerDiagnostics();
        RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
          "NativeSnapshotFlow selected_publication=%lu latest_publication=%lu map_revision=%lu front_source_ns=%ld rear_source_ns=%ld fast_borrows=%lu fast_misses=%lu fast_fallbacks=%lu owner_borrows=%lu owner_misses=%lu publication_misses=%lu solver_newer_publication_reuses=%lu slow_hint_busy=%lu fast_lease_begin_steady_ns=%ld fast_lease_end_steady_ns=%ld",
          timing.publication_sequence,reads.latest_publication_sequence,timing.map_revision,
          read.front_source_ns,read.rear_source_ns,
          reads.validation_borrows,reads.validation_misses,reads.validation_fallback_borrows,
          reads.owner_borrows,reads.owner_misses,reads.publication_misses,reads.solver_newer_publication_reuses,
          slowSnapshotHintMisses(),read.lease_begin_steady_ns,read.lease_end_steady_ns);
        publishMotionDiagnostics();
      } catch(const std::bad_alloc&){std::terminate();}
      catch(const std::exception& e){RCLCPP_ERROR_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
        "Native local curve renewal failed: %s",e.what());}
    }
  }
  void runAdmission() {
    while(!stop_) {
      const auto begin=std::chrono::steady_clock::now();
      try {
        // A slow reader cannot block fast command checking or take either of
        // its double buffers. It also cannot query concurrently with SCAN.
        std::optional<Job> active,candidate,prepared;MeasuredBodyPose body;CurveCheckObservation observation(false);
        {std::lock_guard<std::mutex> l(mutex_);
          expireCandidateOfferLocked(node_->now().nanoseconds());active=active_;candidate=candidate_;
          if(!grant_retired_&&!grant_applied_)prepared=prepared_;
          body=body_;captureCheckObservationLocked(observation,prepared?prepared:candidate);}
        const auto& pending=prepared?prepared:candidate;
        const bool distinct_pending=pending&&(!active||!sameJob(*pending,*active));
        const bool has_work=local_follower_?
          ValidationCycle::admissionWorkWanted(true,active.has_value(),distinct_pending):
          (active.has_value()||candidate.has_value()||prepared.has_value());
        CurveCheckTraceScope trace{*this,has_work&&pending?&*pending:nullptr,observation};
        // Register only real work. The fixed third-slot arbiter alternates
        // contending solver/admission grants; retiring all jobs withdraws this
        // request rather than reserving an idle slot indefinitely.
        withAdmissionSnapshot(has_work,[&](GridMap& snapshot,const std::function<void()>& finish_query) {
          // Formal admission owns only the selected/reserved candidate. The
          // current curve has its own completed snapshot/cache and full check
          // reader; it cannot starve this candidate or wait behind it. Both
          // keep their existing absolute 120 ms capability, never a new lease.
          const auto check_pending=[&](double remaining,ValidationCycle::Clock::time_point deadline){
            if(distinct_pending)check(*pending,body,snapshot,remaining,false,deadline,
              local_follower_?finish_query:std::function<void()>{},local_follower_?&observation:nullptr);};
          if(local_follower_)ValidationCycle::formalAdmissionPass(begin,check_pending,
            []{return ValidationCycle::Clock::now();});
          else ValidationCycle::admissionPass(begin,
            [&](double remaining,ValidationCycle::Clock::time_point deadline){
              if(active)check(*active,body,snapshot,remaining,true,deadline);},check_pending,
            []{return ValidationCycle::Clock::now();});
        },local_follower_?&observation:nullptr);
      } catch(const std::bad_alloc&) {std::terminate();}
      catch(const std::exception& e){RCLCPP_ERROR_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
        "Native admission failed: %s",e.what());}
      std::unique_lock<std::mutex> lock(admission_wait_mutex_);
      admission_wake_.wait_until(lock,begin+std::chrono::milliseconds(200),
        [&]{return stop_.load()||admission_ready_;});admission_ready_=false;
    }
  }
  template<class Check> bool withAdmissionSnapshot(bool has_work,Check&& check,
      CurveCheckObservation* observation=nullptr) {
    if(observation){observation->event.queue_steady_ns=BoundedExecutionTrace::steadyNowNs();
      observation->event.check.disposition.set("snapshot_unavailable");}
    auto snapshot=has_work?pool_.borrowAdmission():GridMap::Ptr{};
    if(observation)observation->event.check.borrow_end_steady_ns=BoundedExecutionTrace::steadyNowNs();
    if(!has_work)pool_.cancelAdmissionRequest();
    if(!snapshot)return false;
    if(observation) {
      // Admission's existing pool API exposes no publication envelope. Read
      // only metadata of the leased map; zero publication means unavailable,
      // never an inferred latest head or another pool-lock acquisition.
      CollisionSnapshotPool::PublicationTiming timing;
      timing.fusion=snapshot->fusionTiming();timing.map_revision=snapshot->occupancyRevision();
      observeSnapshot(observation,timing);
      observation->event.source_ns=snapshot->integratedRaySourceStamp(0);
      observation->event.rear_source_ns=snapshot->integratedRaySourceStamp(1);
      observation->event.check.disposition.set("check_not_started");
    }
    if(validationReadRequiresObservedSnapshot(geometry_policy_,false))snapshot->requireObservedSnapshot();
    // Formal admission has one pending-curve query. checkAt restores its
    // providers and finishes all map reads before invoking this release;
    // ledger waits and proof/candidate publication cannot pin the solver's
    // exclusive third slot. The archived two-query reader keeps the lease
    // until its outer scope ends, including exceptions.
    const std::function<void()> finish=[&]{if(!snapshot)return;snapshot.reset();
      if(observation)observation->event.check.lease_end_steady_ns=BoundedExecutionTrace::steadyNowNs();};
    check(*snapshot,finish);
    if(observation)finish(); // Formal budget-skipped checks still release once.
    return true;
  }
  static std::int64_t ns(const builtin_interfaces::msg::Time& t){return rclcpp::Time(t).nanoseconds();}
  // Formal task-following carries a completed state, not a renewable map
  // lease. A nonzero expiry belongs to the archived protocol and cannot be
  // mixed into the formal command/admission ledger.
  bool curveVerdictCurrent(const ew::TrajectoryValidation& p,std::int64_t now)const {
    return local_follower_?ns(p.valid_until)==0&&ns(p.check_end)>0&&
      ns(p.check_end)<=now+20000000LL:ns(p.valid_until)>now;
  }
  bool inputAndWriterHealthy(const NativePipelineHealth& h,const ew::ExecutionVersion& v,
      std::int64_t now)const {
    if(!h.context||!braking_||h.clock_fault||h.context->session!=v.session_id||
       h.context->seed!=v.localization_seed_id||h.context->epoch!=v.localization_epoch||
       h.context->sequence!=v.context_sequence||h.context->frame!=frame_||
       h.sampled_steady_ns<=0||h.fusion_sequence==0||h.fusion_completed_steady_ns<=0||
       h.fusion_completed_steady_ns>h.sampled_steady_ns||
       h.sampled_steady_ns-h.fusion_completed_steady_ns>component_timeout_ns_)return false;
    for(const auto source:h.source_ns)
      if(!rayStampFresh(source,now,braking_->sensor_source_age))return false;
    return true;
  }
  bool nativePipelineReadyLocked(const ew::ExecutionVersion& version,std::int64_t now) {
    if(!local_follower_)return true;
    const auto h=pipeline_health_?pipeline_health_():NativePipelineHealth{};
    const bool healthy=inputAndWriterHealthy(h,version,now)&&current_check_completed_steady_ns_>0&&
      current_check_completed_steady_ns_<=h.sampled_steady_ns&&
      h.sampled_steady_ns-current_check_completed_steady_ns_<=component_timeout_ns_&&
      current_checked_fusion_progress_steady_ns_>0&&
      current_checked_fusion_progress_steady_ns_<=h.sampled_steady_ns&&
      h.sampled_steady_ns-current_checked_fusion_progress_steady_ns_<=component_timeout_ns_;
    if(!healthy&&!pipeline_fault_) {
      // One fault edge, not a moving floor on every demand/heartbeat. Recovery
      // needs genuinely newer input integrated and checked, never just receipt.
      pipeline_fault_=true;pipeline_fault_sources_=h.source_ns;
      pipeline_fault_fusion_=h.fusion_sequence;
      ++motion_stop_barrier_;pending_local_demand_.reset();relayed_local_command_.reset();
      // Keep latest_demand_ solely as the sequence high-water. No pending or
      // relayed intent survives to be re-captured after quick fault recovery.
    }
    return healthy&&!pipeline_fault_;
  }
  void completedPipelineCheckLocked(const ew::TrajectoryValidation& result,std::uint64_t fusion) {
    if(!local_follower_)return;
    const auto h=pipeline_health_?pipeline_health_():NativePipelineHealth{};
    // The pending-candidate lane must not mask a hung current-curve reader.
    // Before the first commit, an actual completed candidate check bootstraps
    // this clock; with an incumbent only its complete check advances it.
    const bool current=!active_||(active_->version==result.version&&
      active_->spline.trajectory.traj_id==result.trajectory_id);
    if(current) {
      current_check_completed_steady_ns_=h.sampled_steady_ns;
      // Rechecking one immutable map is useful for actual body progress, but
      // cannot disguise a permanently stalled writer-to-reader handoff.
      // This is the same component hang timeout, not per-map admission.
      if(fusion>current_checked_fusion_) {
        current_checked_fusion_=fusion;
        current_checked_fusion_progress_steady_ns_=h.sampled_steady_ns;
      }
    }
    if(current&&pipeline_fault_&&result.valid&&inputAndWriterHealthy(h,result.version,ns(result.check_end))&&
       fusion>pipeline_fault_fusion_&&ns(result.front_ray_source_stamp)>pipeline_fault_sources_[0]&&
       ns(result.rear_ray_source_stamp)>pipeline_fault_sources_[1])pipeline_fault_=false;
  }
  struct LocalMotionDiagnosticSnapshot {
    std::uint64_t passes{0},commands_staged{0},commands_published{0},expired_commands{0};
    std::uint64_t body_deferred{0},body_completed{0},commit_deferred{0},commit_completed{0};
    std::uint64_t proof_deferred{0},proof_completed{0},evidence_updates{0};
    bool pending_body{false},pending_commit{false},lease_allowed{false};
    std::int64_t last_check_ns{0},max_check_ns{0},source_to_command_ns{0},body_to_command_ns{0};
    std::int64_t max_source_to_command_ns{0},max_body_to_command_ns{0};
    std::uint64_t lease_sequence{0},demand_sequence{0};
    std::string task,lease_task,demand_task;
  };
  LocalMotionDiagnosticSnapshot copyLocalMotionDiagnostics()const {
    // Value-copy only. JSON formatting and the potentially blocking logger /
    // ROS sink must not hold the same ledger mutex as body/command delivery.
    std::lock_guard<std::mutex> l(mutex_);
    LocalMotionDiagnosticSnapshot s;
    s.passes=local_motion_passes_;s.commands_published=local_commands_published_;
    s.commands_staged=local_commands_staged_;
    s.expired_commands=local_expired_commands_;
    s.body_deferred=local_body_deferred_commands_;s.body_completed=local_body_completed_commands_;
    s.commit_deferred=local_commit_deferred_commands_;s.commit_completed=local_commit_completed_commands_;
    s.proof_deferred=local_proof_deferred_commands_;s.proof_completed=local_proof_completed_commands_;
    s.evidence_updates=local_evidence_updates_;
    s.pending_body=pending_local_demand_&&!pending_local_demand_->waiting_for_commit;
    s.pending_commit=pending_local_demand_&&pending_local_demand_->waiting_for_commit;
    s.last_check_ns=local_last_motion_pass_ns_;s.max_check_ns=local_max_motion_pass_ns_;
    s.source_to_command_ns=local_source_to_command_ns_;s.body_to_command_ns=local_body_to_command_ns_;
    s.max_source_to_command_ns=local_max_source_to_command_ns_;s.max_body_to_command_ns=local_max_body_to_command_ns_;
    s.task=task_domain_?task_domain_->task_id:"";
    s.lease_task=task_lease_?task_lease_->version.task_id:"";s.lease_sequence=task_lease_sequence_;
    s.lease_allowed=task_lease_&&task_lease_->allowed&&!task_lease_->revoked;
    s.demand_task=latest_demand_?latest_demand_->version.task_id:"";
    s.demand_sequence=latest_demand_?latest_demand_->sequence:0;
    return s;
  }
  void publishMotionDiagnostics() {
    const auto now=std::chrono::steady_clock::now();
    if(now-last_motion_diagnostics_publication_<std::chrono::milliseconds(500))return;
    last_motion_diagnostics_publication_=now;
    const auto status=motion_diagnostics_.snapshot();
    nlohmann::json counts=nlohmann::json::object();
    for(std::size_t i=0;i<status.counts.size();++i)
      counts[MotionCheckDiagnostics::name(static_cast<MotionCheckStage>(i))]=status.counts[i];
    nlohmann::json payload{{"schema",1},{"diagnostic_only",true},{"transport_mode",mode_},
      {"stage",MotionCheckDiagnostics::name(status.stage)},{"observed_at_ns",status.observed_at_ns},
      {"counts",std::move(counts)},{"last_nonzero_demand",nullptr}};
    nlohmann::json events=nlohmann::json::array(),new_events=nlohmann::json::array();
    std::size_t event_log_omitted=0;
    const auto first_event=status.event_sequence>MotionCheckDiagnostics::event_capacity?
      status.event_sequence-MotionCheckDiagnostics::event_capacity+1:1;
    for(std::uint64_t id=first_event;id<=status.event_sequence;++id) {
      const auto& e=status.events[(id-1)%MotionCheckDiagnostics::event_capacity];
      if(e.event_id!=id)continue;
      nlohmann::json event{{"id",id},{"at_ns",e.observed_at_ns},{"task",e.task_id},
        {"sequence",e.command_sequence},{"curve",e.curve},{"active_curve",e.active_curve},
        {"stage",MotionCheckDiagnostics::name(e.stage)},{"hold",e.hold},
        {"proof_sequence",e.proof_sequence},{"validation_floor",e.validation_floor},{"source_ns",e.command_source_ns},
        {"body_source_ns",e.body_source_ns},{"native_body_source_ns",e.native_body_source_ns},
        {"source_age_ms",(e.observed_at_ns-e.command_source_ns)*1e-6},
        {"body_age_ms",(e.observed_at_ns-e.body_source_ns)*1e-6},
        {"native_body_lag_ms",(e.body_source_ns-e.native_body_source_ns)*1e-6},
        {"original_remaining_ms",(e.command_valid_until_ns-e.observed_at_ns)*1e-6}};
      events.push_back(event);
      if(id>last_motion_event_log_sequence_) {
        // The complete trace above owns per-command/lifecycle history. Keep
        // this legacy rejection summary small even during an error burst.
        if(new_events.size()<8)new_events.push_back(std::move(event));
        else ++event_log_omitted;
      }
    }
    payload["command_events"]={{"capacity",MotionCheckDiagnostics::event_capacity},
      {"last_event_id",status.event_sequence},{"events",std::move(events)}};
    // Never log from the 20/50 Hz delivery callbacks. One bounded batch records
    // the exact missing command instead of whichever stage survives to 1 Hz.
    // An overloaded ring explicitly reports omitted diagnostic events.
    if(node_&&!new_events.empty()&&now-last_motion_event_log_>=std::chrono::seconds(1)) {
      const auto overwritten=first_event>last_motion_event_log_sequence_+1?
        first_event-last_motion_event_log_sequence_-1:0;
      RCLCPP_INFO(node_->get_logger(),"LocalCommandEvents retained_first=%lu last=%lu overwritten=%lu omitted_for_batch=%lu events=%s",
        first_event,status.event_sequence,overwritten,event_log_omitted,new_events.dump().c_str());
      last_motion_event_log_=now;last_motion_event_log_sequence_=status.event_sequence;
    }
    if(local_follower_) {
      const auto s=copyLocalMotionDiagnostics();
      payload["local_motion_lane"]={{"event_driven",true},{"latest_capacity",1},
        {"collision_gate","current_native_curve"},{"command_sweep",false},
        {"passes",s.passes},{"commands_staged",s.commands_staged},{"commands_published",s.commands_published},
        {"expired_commands",s.expired_commands},
        {"body_deferred_commands",s.body_deferred},
        {"body_completed_commands",s.body_completed},
        {"commit_deferred_commands",s.commit_deferred},
        {"commit_completed_commands",s.commit_completed},
        {"proof_deferred_commands",s.proof_deferred},
        {"proof_completed_commands",s.proof_completed},
        {"same_command_evidence_updates",s.evidence_updates},
        {"pending_body_delivery",s.pending_body},
        {"pending_commit_delivery",s.pending_commit},
        {"last_check_ms",s.last_check_ns*1e-6},{"max_check_ms",s.max_check_ns*1e-6},
        {"source_to_command_ms",s.source_to_command_ns*1e-6},
        {"body_to_command_ms",s.body_to_command_ns*1e-6},
        {"max_source_to_command_ms",s.max_source_to_command_ns*1e-6},
        {"max_body_to_command_ms",s.max_body_to_command_ns*1e-6}};
      // Persist the already computed transport diagnosis as well as publishing
      // it. Without this, a post-run log cannot distinguish a rejected task
      // lease from an absent command. No extra query or motion gate is added.
      if(node_)RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
        "LocalCommandFlow task=%s lease_task=%s lease_sequence=%lu lease_allowed=%d demand_task=%s demand_sequence=%lu stage=%s relayed=%lu proof_deferred=%lu proof_completed=%lu evidence_updates=%lu",
        s.task.c_str(),s.lease_task.c_str(),s.lease_sequence,s.lease_allowed,
        s.demand_task.c_str(),s.demand_sequence,MotionCheckDiagnostics::name(status.stage),
        s.commands_published,s.proof_deferred,s.proof_completed,s.evidence_updates);
    }
    if(status.last_nonzero_demand) {
      const auto& d=*status.last_nonzero_demand;const auto& v=d.version;
      payload["last_nonzero_demand"]={{"observed_at_ns",status.last_nonzero_observed_at_ns},
        {"stage",MotionCheckDiagnostics::name(status.last_nonzero_stage)},
        {"version_schema",v.schema_version},
        {"session_id",v.session_id},{"task_id",v.task_id},{"route_id",v.route_id},{"route_hash",v.route_hash},
        {"map_version_id",v.map_version_id},{"localization_epoch",v.localization_epoch},
        {"localization_seed_id",v.localization_seed_id},{"reference_generation",v.reference_generation},
        {"segment_id",v.segment_id},{"anchor_id",v.anchor_id},{"anchor_revision",v.anchor_revision},
        {"context_sequence",v.context_sequence},{"map_geometry_revision",v.map_geometry_revision},
        {"execution_id",d.execution_id},{"control_epoch",d.control_epoch},{"sdk_session",d.sdk_session},
        {"sdk_arm_generation",d.sdk_arm_generation},{"trajectory_id",d.trajectory_id},
        {"demand_sequence",d.sequence},{"permit_sequence",d.permit_sequence},
        {"validation_floor",d.validation_sequence},{"native_proof_sequence",status.proof_sequence},
        {"map_snapshot_revision",status.map_snapshot_revision},
        {"source_stamp_ns",ns(d.source_stamp)},{"body_source_stamp_ns",ns(d.body_source_stamp)},
        {"valid_until_ns",ns(d.valid_until)},
        {"velocity",{d.velocity.linear.x,d.velocity.linear.y,d.velocity.linear.z,
          d.velocity.angular.x,d.velocity.angular.y,d.velocity.angular.z}},
        {"reason",d.reason}};
    }
    std_msgs::msg::String message;message.data=payload.dump();motion_diagnostics_pub_->publish(message);
  }
  static bool proofSupersedes(const ew::TrajectoryValidation& incoming,const ew::TrajectoryValidation& current) {
    if(incoming.sequence<=current.sequence||incoming.map_snapshot_revision<current.map_snapshot_revision||
       ns(incoming.body_source_stamp)<ns(current.body_source_stamp))return false;
    const auto front=ns(incoming.front_ray_source_stamp),rear=ns(incoming.rear_ray_source_stamp);
    const auto current_front=ns(current.front_ray_source_stamp),current_rear=ns(current.rear_ray_source_stamp);
    // Occupancy revision changes with geometry, not every acquisition. The
    // third-slot reader can therefore start later with the SAME revision but
    // older rays than the fast reader. A positive result may never regress a
    // source or resurrect geometry from before a newer negative observation.
    if(incoming.valid&&(front<current_front||rear<current_rear))return false;
    // An expired old-source refresh is absence of new evidence, not a newly
    // witnessed collision. A newer occupancy revision can be copied from an
    // older source pair, so revision alone cannot turn that failed acquisition
    // into a geometry verdict. Keep the already certified positive ONLY to
    // its original expiry. Mixed/new sources and witnessed negative geometry
    // still fence immediately; there is no lease extension or free-space claim.
    const auto checked_at=ns(incoming.check_end);
    if(!incoming.valid&&incoming.reason=="source_expired_during_check"&&current.valid&&
       checked_at>0&&
       ns(current.check_end)<=checked_at&&ns(current.valid_until)>checked_at&&
       current_front>0&&current_rear>0&&front<=current_front&&rear<=current_rear&&
       (front<current_front||rear<current_rear))return false;
    return true;
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
      std::int64_t now,bool state_contract=false) {
    if(demand.validation_sequence==0)return {};
    const ew::TrajectoryValidation* newest=nullptr;
    for(const auto& proof:history) {
      if(proof.version!=demand.version||proof.trajectory_id!=demand.trajectory_id||
         proof.sequence<demand.validation_sequence)continue;
      if(!proof.valid)return {};
      if(!newest||proof.sequence>newest->sequence)newest=&proof;
    }
    if(!newest||ns(newest->check_end)>now||
       (state_contract?ns(newest->valid_until)!=0:ns(newest->valid_until)<=now))return {};
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
    // MC has an approximate receipt-anchored clock in live mode. Its original
    // mapped timestamp can lead receipt within the declared capture bound;
    // it must not extend the lease or advance the post-zero capture evidence.
    const auto source=ns(e.source_stamp),received=ns(e.received_stamp);
    const auto anchor=std::min(source,received);
    if(!std::isfinite(e.mc_capture_delay_bound_sec)||e.mc_capture_delay_bound_sec<=0.||
       e.mc_capture_delay_bound_sec>.25)return false;
    const auto bound=static_cast<std::int64_t>(std::llround(e.mc_capture_delay_bound_sec*1e9));
    return e.schema_version==1&&e.usable&&e.nonzero_blocked&&e.sequence>0&&e.zero_write_sequence>0&&
      e.version==p.version&&e.execution_id==p.execution_id&&e.control_epoch==p.control_epoch&&
      e.sdk_session==p.sdk_session&&e.sdk_arm_generation==p.sdk_arm_generation&&e.transport_mode==mode_&&
      e.writer_commit_sequence==g.expected_commit_sequence&&e.applied_trajectory_id==p.trajectory_id&&
      e.mc_raw_stamp_ns>0&&!e.mc_clock_epoch.empty()&&
      e.time_basis==(mode_=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock")&&
      e.stationary_samples>=3&&std::isfinite(e.stationary_duration_sec)&&e.stationary_duration_sec>=.6&&
      std::isfinite(e.measured_linear_mps)&&std::isfinite(e.measured_angular_radps)&&
      ns(e.zero_ack_at)>0&&ns(e.capture_lower_bound)>ns(e.zero_ack_at)&&
      ns(e.capture_lower_bound)<=anchor&&ns(e.capture_upper_bound)==received&&
      ns(e.capture_upper_bound)-ns(e.capture_lower_bound)<=bound+std::abs(source-received)+1000LL&&
      source>0&&received>0&&received<=now&&now-anchor<=250000000LL&&
      (mode_=="live"?source-received<=bound:source<=received)&&
      ns(e.valid_until)>now&&ns(e.valid_until)>anchor&&ns(e.valid_until)-anchor<=250001000LL&&
      ns(g.source_stamp)>=received&&g.retain_incumbent_until==g.source_stamp&&
      ns(g.valid_until)<=ns(e.valid_until)&&ns(g.candidate.valid_until)<=ns(e.valid_until)&&
      ns(g.valid_until)-ns(g.source_stamp)<=250001000LL;
  }
  static bool preparedEntryMatches(const ew::PreparedMotionDemand& x,const ew::ExecutionHandoffGrant& g,
      const std::string& frame,std::int64_t now,std::int64_t command_pipeline_ns=100000000LL) {
    const auto& d=x.demand;const auto& a=x.entry_admission;const auto& p=g.candidate;
    return g.schema_version==2&&x.schema_version==1&&x.handoff_id==g.handoff_id&&x.grant_sequence==g.sequence&&
      x.expected_commit_sequence==g.expected_commit_sequence&&d.version==p.version&&
      d.execution_id==p.execution_id&&d.control_epoch==p.control_epoch&&d.sdk_session==p.sdk_session&&
      d.sdk_arm_generation==p.sdk_arm_generation&&d.permit_sequence==p.sequence&&d.trajectory_id==p.trajectory_id&&
      d.validation_sequence>=p.validation_sequence&&d.transport_mode==p.transport_mode&&a.transport_mode==p.transport_mode&&
      a.sequence>0&&a.accepted&&a.version==d.version&&a.trajectory_id==d.trajectory_id&&a.validation_sequence==d.validation_sequence&&
      a.body_source_stamp==d.body_source_stamp&&x.entry_source_stamp==d.body_source_stamp&&
      x.measured_pose.header.stamp==x.entry_source_stamp&&x.measured_pose.header.frame_id==frame&&
      motionSourceFresh(ns(d.source_stamp),now,command_pipeline_ns)&&motionSourceFresh(ns(d.body_source_stamp),now,command_pipeline_ns)&&
      ns(d.valid_until)>now&&ns(d.valid_until)<=ns(d.source_stamp)+command_pipeline_ns+1LL&&
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
    std::optional<BodySourceSample> actual_sample;
    std::shared_ptr<const MotionSupport> support;std::uint64_t barrier=0;
    {std::lock_guard<std::mutex> l(mutex_);
      if(!prepared_demand_||!grant_||grant_retired_||grant_applied_||!prepared_)return;
      request=prepared_demand_;grant=grant_;job=prepared_;current=body_;barrier=prepared_stop_barrier_;
      entry_sample=bodyAtSourceLocked(ns(request->entry_source_stamp));
      if(!body_history_.empty())actual_sample=body_history_.back();
      proof=motionProof(proofs_,request->demand,node_->now().nanoseconds());
      for(auto i=support_history_.rbegin();i!=support_history_.rend();++i)
        if((*i)->source.version==request->demand.version){support=*i;break;}}
    const auto& x=*request;const auto& d=x.demand;const auto& g=*grant;const auto now=node_->now().nanoseconds();
    if(!proof||!proof->whole_curve||!proof->valid||!job->support||!support||!support->valid||!braking_||
       !entry_sample||!entry_sample->twist||!actual_sample||!actual_sample->twist||
       !motionSourceFresh(actual_sample->source_ns,now)||
       !preparedEntryMatches(x,g,frame_,now,braking_->commandPipelineNs())||!map.isRawRaySnapshot()||!map.integratedCloudFreshAt(now)||
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
    // Historical entry consistency is bound above to the unchanged request.
    // Collision must use the latest real pose/twist, not that ageing boundary.
    // Writer CAS separately rechecks latest actual entry on the fixed curve.
    const auto& body=actual_sample->pose;
    double yaw=0.;if(!measuredBodyYaw(body,frame_,now*1e-9,.1,yaw))return;
    const double remaining=std::min(budget_s,std::chrono::duration<double>(round_deadline-ValidationCycle::Clock::now()).count());
    if(remaining<=0.)return;
    const auto sweep=validateMotionSweep(map,*support,*braking_,body.position,yaw,*actual_sample->twist,d.velocity,frame_,remaining);
    if(sweep.completion==MotionSweepCompletion::BudgetExhausted&&map.collisionSnapshotSourceFresh()&&
       motionSourceFresh(ns(d.source_stamp),node_->now().nanoseconds(),braking_->commandPipelineNs())&&
       motionSourceFresh(ns(d.body_source_stamp),node_->now().nanoseconds(),braking_->commandPipelineNs())&&
       motionSourceFresh(ns(x.entry_source_stamp),node_->now().nanoseconds(),braking_->commandPipelineNs())&&
       map.localizationContextSequence()==d.version.context_sequence)return;
    ew::MotionValidation out;out.version=d.version;out.execution_id=d.execution_id;out.control_epoch=d.control_epoch;
    out.sdk_session=d.sdk_session;out.sdk_arm_generation=d.sdk_arm_generation;out.trajectory_id=d.trajectory_id;
    out.permit_sequence=d.permit_sequence;out.trajectory_validation_sequence=proof->sequence;out.demand_sequence=d.sequence;
    out.demand_source_stamp=d.source_stamp;out.demand_body_source_stamp=d.body_source_stamp;out.demand_valid_until=d.valid_until;
    out.sequence=++motion_sequence_;out.velocity=d.velocity;out.frame_id=frame_;out.transport_mode=mode_;
    out.braking_model_sha256=braking_->sha256;out.check_begin=rclcpp::Time(now);out.check_end=node_->now();
    out.body_source_stamp=rclcpp::Time(actual_sample->source_ns);out.front_ray_source_stamp=rclcpp::Time(map.integratedRaySourceStamp(0));
    out.rear_ray_source_stamp=rclcpp::Time(map.integratedRaySourceStamp(1));out.map_snapshot_revision=map.occupancyRevision();
    out.valid=sweep.valid;out.reason=sweep.reason;out.handoff_id=g.handoff_id;out.entry_admission_sequence=x.entry_admission.sequence;
    out.entry_curve_pose=x.curve_entry_pose;out.entry_curve_twist=x.curve_entry_twist;out.entry_curve_time=x.curve_time;
    out.entry_position_error_m=pe;out.entry_velocity_error_mps=ve;
    const auto deadline=motionEvidenceDeadline(ns(d.source_stamp),ns(d.body_source_stamp),
      actual_sample->source_ns,
      ns(d.valid_until),std::min({ns(g.valid_until),ns(g.transition_deadline),ns(g.candidate.valid_until)}),
      ns(proof->valid_until),map.observedProofDeadlineNs(),map.integratedRaySourceStamp(0),map.integratedRaySourceStamp(1),braking_->raySourceAgeNs(),braking_->commandPipelineNs());
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
    checkMotionAt(map,[this]{return node_->now().nanoseconds();},
      [this](const ew::LocalMotionCommand& command){if(local_command_pub_)local_command_pub_->publish(command);},
      budget_s,round_deadline);
  }
  // Archived v3 sweep fixture with injected clock/output and the production
  // GridMap, exercised without a ROS graph/SDK. Formal LocalFollower execution
  // is excluded below and relays through demandWithClock instead.
  void checkMotionAt(GridMap& map,const std::function<std::int64_t()>& clock,
      const std::function<void(const ew::LocalMotionCommand&)>& /* legacy output seam */,
      double budget_s=ValidationCycle::motion_budget_s,
      ValidationCycle::Clock::time_point round_deadline=ValidationCycle::Clock::time_point::max()) {
    // Archived v3 reader only. Formal task_follower_v1 never evaluates this
    // reaction/braking volume and must never issue a MotionValidation for it.
    if(local_follower_)return;
    std::optional<ew::MotionDemand> demand;std::optional<ew::ExecutionPermit> permit,latest;
    std::optional<Job> active;std::shared_ptr<const MotionSupport> support;std::uint64_t barrier;
    std::optional<ew::TrajectoryValidation> proof;
    std::optional<BodySourceSample> measured_state;
    {
      std::lock_guard<std::mutex> l(mutex_);
      if(!latest_demand_||latest_demand_->hold||planarDemandIsZero(latest_demand_->velocity)) {
        motion_diagnostics_.record(MotionCheckStage::Idle,latest_demand_,clock());return;
      }
      demand=latest_demand_;active=active_;latest=latest_permit_;barrier=motion_stop_barrier_;
      if(!body_history_.empty())measured_state=body_history_.back();
      for(auto it=support_history_.rbegin();it!=support_history_.rend();++it)
        if((*it)->source.version==demand->version){support=*it;break;}
      for(auto it=permits_.rbegin();it!=permits_.rend();++it)if(it->sequence==demand->permit_sequence){permit=*it;break;}
      proof=motionProof(proofs_,*demand,clock());
    }
    const auto& d=*demand;
    const auto now=clock();
    const auto wait=[&](MotionCheckStage stage) {
      motion_diagnostics_.record(stage,demand,clock(),
        proof?proof->sequence:0,map.occupancyRevision());
      if(local_follower_&&(stage==MotionCheckStage::CommandSourceExpired||
          stage==MotionCheckStage::EvidenceExpiredDuringCheck||stage==MotionCheckStage::BodySourceExpired)) {
        std::lock_guard<std::mutex> l(mutex_);++local_expired_commands_;
      }
    };
    // Waiting for fresh same-curve evidence/permission is not a newly discovered
    // collision. Do not publish a false negative against a newer still-safe
    // command. Existing certified demand can live ONLY to its original expiry.
    // A missing dependency can never produce a positive result either.
    if(!permit){wait(MotionCheckStage::PermitNotDelivered);return;}
    if(!proof){wait(MotionCheckStage::CurveProofUnavailable);return;}
    if(ns(proof->valid_until)<=now||ns(permit->valid_until)<=now) {
      wait(MotionCheckStage::SourceLeaseExpired);return;
    }
    // Topic scheduling can deliver the command before its independent odom
    // witness. Wait for that acquired state; this is not occupied evidence and
    // does not extend any previously issued command's original expiration.
    if(!measured_state){wait(MotionCheckStage::BodyNotDelivered);return;}
    if(!measured_state->twist){wait(MotionCheckStage::BodyTwistMissing);return;}
    if(measured_state->source_ns<ns(d.body_source_stamp)){wait(MotionCheckStage::BodyBehindDemand);return;}
    if(!motionSourceFresh(measured_state->source_ns,now)){wait(MotionCheckStage::BodySourceExpired);return;}
    // Missing prerequisites are WAIT facts, not an occupied/free verdict. The
    // original demand/permit/evidence lease still expires normally. Only the
    // actual native sweep evaluation below may issue MotionValidation.
    if(!braking_){wait(MotionCheckStage::BrakingModelUnavailable);return;}
    if(!active){wait(MotionCheckStage::ActiveGeometryUnavailable);return;}
    if(!active->progress){wait(MotionCheckStage::MeasuredProgressUnavailable);return;}
    if(!support||support->source.version!=d.version){wait(MotionCheckStage::SupportUnavailable);return;}
    if(active->version!=d.version||active->spline.trajectory.traj_id!=d.trajectory_id) {
      wait(MotionCheckStage::ActiveGeometryMismatch);return;
    }
    if(!active->proof||!active->proof->valid||active->proof->sequence<d.validation_sequence||
       ns(active->proof->valid_until)<=now){wait(MotionCheckStage::ActiveProofUnavailable);return;}
    if(!latest||!demandPermitMatches(d,*permit,proof->sequence)||
       !demandPermitMatches(d,*latest,proof->sequence)||permit->phase!=latest->phase||
       (permit->phase=="aligning"&&(std::abs(d.velocity.linear.x)>=1e-12||std::abs(d.velocity.linear.y)>=1e-12))) {
      wait(MotionCheckStage::PermitBindingMismatch);return;
    }
    if(ns(latest->valid_until)<=now){wait(MotionCheckStage::SourceLeaseExpired);return;}
    ew::MotionValidation out;
    out.version=d.version;out.execution_id=d.execution_id;out.control_epoch=d.control_epoch;
    out.sdk_session=d.sdk_session;out.sdk_arm_generation=d.sdk_arm_generation;out.trajectory_id=d.trajectory_id;
    out.permit_sequence=d.permit_sequence;out.trajectory_validation_sequence=proof->sequence;
    out.demand_sequence=d.sequence;out.demand_source_stamp=d.source_stamp;
    out.demand_valid_until=d.valid_until;out.demand_body_source_stamp=d.body_source_stamp;
    out.sequence=++motion_sequence_;out.velocity=d.velocity;out.frame_id=frame_;out.transport_mode=mode_;
    out.braking_model_sha256=braking_->sha256;
    out.check_begin=rclcpp::Time(clock());out.front_ray_source_stamp=rclcpp::Time(map.integratedRaySourceStamp(0));
    out.rear_ray_source_stamp=rclcpp::Time(map.integratedRaySourceStamp(1));
    out.map_snapshot_revision=map.occupancyRevision();
    const auto pipeline=braking_->commandPipelineNs();
    const auto fresh=[&](const builtin_interfaces::msg::Time& t){return motionSourceFresh(ns(t),now,pipeline);};
    MotionSweepResult sweep;
    std::int64_t deadline=std::min<std::int64_t>({ns(d.valid_until),ns(d.source_stamp)+pipeline,ns(d.body_source_stamp)+pipeline});
    if(!fresh(d.source_stamp)||!fresh(d.body_source_stamp)||deadline<=now) {
      wait(MotionCheckStage::CommandSourceExpired);return;
    }
    if(!map.isRawRaySnapshot()||!map.integratedCloudFreshAt(now)||
       map.localizationContextSequence()!=d.version.context_sequence) {
      wait(MotionCheckStage::SnapshotSourceUnavailable);return;
    }
    // Curve progress carries route identity, not the current safety pose or
    // limiter boundary. Pose and twist come from one latest real odometry.
    const auto& body=measured_state->pose;const auto& position=body.position;
    out.body_source_stamp=rclcpp::Time(measured_state->source_ns);double yaw=0.;
    if(!measuredBodyYaw(body,frame_,now*1e-9,.1,yaw)){wait(MotionCheckStage::BodyYawUnavailable);return;}
    const double remaining=std::min(budget_s,
      std::chrono::duration<double>(round_deadline-ValidationCycle::Clock::now()).count());
    if(remaining<=0.){wait(MotionCheckStage::QueryBudgetExhausted);return;}
    sweep=validateMotionSweep(map,*support,*braking_,position,yaw,*measured_state->twist,d.velocity,frame_,remaining);
    // An incomplete reader has witnessed neither free space nor a completed
    // negative sweep; expiry/CPU waiting is not a fabricated obstacle.
    if(sweep.completion==MotionSweepCompletion::BudgetExhausted){wait(MotionCheckStage::SweepBudgetExhausted);return;}
    out.valid=sweep.valid;out.reason=sweep.reason;
    deadline=motionEvidenceDeadline(ns(d.source_stamp),ns(d.body_source_stamp),measured_state->source_ns,
      ns(d.valid_until),std::min(ns(permit->valid_until),ns(latest->valid_until)),ns(proof->valid_until),
      map.observedProofDeadlineNs(),map.integratedRaySourceStamp(0),map.integratedRaySourceStamp(1),braking_->raySourceAgeNs(),pipeline);
    out.check_end=rclcpp::Time(clock());out.valid_until=rclcpp::Time(std::max<std::int64_t>(0,deadline));
    if(ValidationCycle::Clock::now()>=round_deadline){wait(MotionCheckStage::PublicationBudgetExhausted);return;}
    if(!map.integratedCloudFreshAt(ns(out.check_end))||deadline<=ns(out.check_end)) {
      wait(MotionCheckStage::EvidenceExpiredDuringCheck);return;
    }
    {
      std::lock_guard<std::mutex> l(mutex_);
      // A later ordinary nonzero request may coexist with this exact proof.
      // A later zero/HOLD/revoke/owner switch may never resurrect old motion.
      const auto final_proof=motionProof(proofs_,d,clock());
      if(!final_proof||final_proof->sequence!=proof->sequence){wait(MotionCheckStage::CurveProofSuperseded);return;}
      if(barrier!=motion_stop_barrier_){wait(MotionCheckStage::StopBarrierChanged);return;}
      if(!active_||active_->version!=d.version||active_->spline.trajectory.traj_id!=d.trajectory_id) {
        wait(MotionCheckStage::ActiveGeometryChanged);return;
      }
      if(!latest_permit_||!demandPermitMatches(d,*latest_permit_,proof->sequence)||
         latest_permit_->phase!=permit->phase||ns(latest_permit_->valid_until)<=clock()) {
        wait(MotionCheckStage::OwnerPermitChanged);return;
      }
      if(!active_->proof||!active_->proof->valid||active_->proof->sequence<d.validation_sequence||
         ns(active_->proof->valid_until)<=clock()) {
        wait(MotionCheckStage::ActiveProofExpired);return;
      }
    }
    if(node_&&!out.valid)RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),250,
      "Motion proof demand=%lu visited_unique_voxels=%zu free=%zu occupied=%zu unknown=%zu outside=%zu query_complete=%d length=%.3f yaw_bound=%.3f check_ms=%.3f demand_age_ms=%.3f front_age_ms=%.3f rear_age_ms=%.3f queried=%d reason=%s",
      d.sequence,sweep.unique_voxels,sweep.state_counts[0],sweep.state_counts[1],sweep.state_counts[2],sweep.state_counts[3],
      sweep.query_complete,sweep.length,sweep.heading_bound,(ns(out.check_end)-ns(out.check_begin))*1e-6,
      (ns(out.check_end)-ns(d.source_stamp))*1e-6,(ns(out.check_end)-map.integratedRaySourceStamp(0))*1e-6,
      (ns(out.check_end)-map.integratedRaySourceStamp(1))*1e-6,1,out.reason.c_str());
    else if(node_)RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
      "Motion proof demand=%lu snapshot=%lu unique_voxels=%zu check_ms=%.3f source_age_ms=%.3f body_age_ms=%.3f front_age_ms=%.3f rear_age_ms=%.3f lease_ms=%.3f",
      d.sequence,out.map_snapshot_revision,sweep.unique_voxels,(ns(out.check_end)-ns(out.check_begin))*1e-6,
      (ns(out.check_end)-ns(d.source_stamp))*1e-6,(ns(out.check_end)-ns(out.body_source_stamp))*1e-6,
      (ns(out.check_end)-map.integratedRaySourceStamp(0))*1e-6,(ns(out.check_end)-map.integratedRaySourceStamp(1))*1e-6,
      (deadline-ns(out.check_end))*1e-6);
    {std::lock_guard<std::mutex> l(mutex_);motion_proofs_.push_back(out);
      while(motion_proofs_.size()>32)motion_proofs_.pop_front();
    }
    if(motion_pub_)motion_pub_->publish(out);
    motion_diagnostics_.record(MotionCheckStage::VerdictPublished,demand,clock(),
      proof->sequence,map.occupancyRevision());
  }
  bool check(const Job& job,const MeasuredBodyPose& body,GridMap& map,
      double budget_s=ValidationCycle::slow_curve_budget_s,bool skip_budget_failure=false,
      ValidationCycle::Clock::time_point round_deadline=ValidationCycle::Clock::time_point::max(),
      const std::function<void()>& finish_map_query={},CurveCheckObservation* observation=nullptr) {
    return checkAt(job,body,map,[this]{return node_->now().nanoseconds();},
      [this](const ew::TrajectoryValidation& p){if(pub_)pub_->publish(p);},budget_s,skip_budget_failure,round_deadline,
      [this](const ew::LocalMotionCommand& c){if(local_command_pub_)local_command_pub_->publish(c);},finish_map_query,observation);
  }
  bool checkAt(const Job& job,const MeasuredBodyPose& body,GridMap& map,
      const std::function<std::int64_t()>& clock,
      const std::function<void(const ew::TrajectoryValidation&)>& emit_proof,
      double budget_s=ValidationCycle::slow_curve_budget_s,bool skip_budget_failure=false,
      ValidationCycle::Clock::time_point round_deadline=ValidationCycle::Clock::time_point::max(),
      const std::function<void(const ew::LocalMotionCommand&)>& emit_command={},
      const std::function<void()>& finish_map_query={},CurveCheckObservation* observation=nullptr) {
    const auto check_started=std::chrono::steady_clock::now();
    if(observation)observation->event.check.disposition.set("check_exception");
    // Candidate, incumbent and its actual connector use the SAME native body
    // geometry. Pin the source index with the job; rolling pending references
    // cannot change the accepted curve's terrain identity.
    std::optional<StairAttitudeScope> attitude(std::in_place,map,job.geometry_support,body);
    const double query_resolution=map.getResolution();
    const auto check_fusion_sequence=map.fusionTiming().sequence;
    const auto finish_query=[&] {
      if(!attitude)return;
      // These providers own per-query caches and close over this exact map.
      // Destroy/restore them BEFORE the writer may recycle the buffer. No map
      // access is permitted after this point, including during ledger commit.
      attitude.reset();
      if(finish_map_query)finish_map_query();
    };
    bool resource_incomplete=false;
    map.beginObservedProof();
    ew::TrajectoryValidation result;result.version=job.version;result.proposal_id=job.proposal;
    result.splice=job.spline.splice;
    result.trajectory_id=job.spline.trajectory.traj_id;result.sequence=++sequence_;
    result.check_begin=rclcpp::Time(clock());
    std::int64_t acquired_body_source_ns=0;
    {std::lock_guard<std::mutex> l(mutex_);
      // UNIX seconds represented as double cannot recover the original ROS
      // nanoseconds. Bind this proof to the exact acquired sample used by the
      // query, not a rounded timestamp or the latest unrelated body callback.
      const auto sample=std::find_if(body_history_.rbegin(),body_history_.rend(),[&](const auto& s){
        return s.pose.source_stamp==body.source_stamp&&s.pose.frame==body.frame&&
          (!s.provider_identity||s.provider_identity->matches(job.version))&&
          (s.pose.position.array()==body.position.array()).all()&&
          (s.pose.orientation.coeffs().array()==body.orientation.coeffs().array()).all();});
      if(sample!=body_history_.rend())acquired_body_source_ns=sample->source_ns;
    }
    result.body_source_stamp=rclcpp::Time(acquired_body_source_ns);
    result.front_ray_source_stamp=rclcpp::Time(map.integratedRaySourceStamp(0));
    result.rear_ray_source_stamp=rclcpp::Time(map.integratedRaySourceStamp(1));
    result.map_snapshot_revision=map.occupancyRevision();result.frame_id=frame_;
    if(observation) {
      auto& e=observation->event;e.proof=result.sequence;e.body_ns=ns(result.body_source_stamp);
      e.source_ns=ns(result.front_ray_source_stamp);e.rear_source_ns=ns(result.rear_ray_source_stamp);
      e.map_revision=result.map_snapshot_revision;e.check.check_begin_ns=ns(result.check_begin);
    }
    // SCAN's Native curve check is the formal collision gate. Actuation still
    // needs its separate current task lease and original command/body expiry.
    const bool official_geometry=geometry_policy_=="official_inflated_geometry";
    result.collision_policy=geometry_policy_;result.transport_mode=mode_;result.whole_curve=true;
    const auto source_check_ns=clock();
    const bool source_fresh_at_entry=acquired_body_source_ns>0&&
        (local_follower_||map.integratedCloudFreshAt(source_check_ns))&&
        source_check_ns*1e-9-body.source_stamp<=.4;
    if(local_follower_&&(!source_fresh_at_entry||!map.isRawRaySnapshot()||
        map.localizationContextSequence()!=job.version.context_sequence)) {
      finish_query();
      if(observation){observation->event.reason.set(source_fresh_at_entry?
          "snapshot_context_unavailable":"body_input_unavailable");
        observation->event.check.disposition.set("input_unavailable_no_verdict");}
      return false;
    }
    result.reason=source_fresh_at_entry?"missing_fresh_raw_ray_support_or_body_evidence":"source_expired_during_check";
    if(job.support) {result.support_reference_id=job.support->support_reference_id;result.support_hash=job.support->support_hash;}
    const auto& s=job.spline.trajectory;
    std::optional<NoMotionPreviewProgress> next_preview_progress;
    if(source_fresh_at_entry&&usablePlanningGeometry(job)&&
       job.support->frame_id==frame_&&!job.support->support_reference_id.empty()&&!job.support->support_hash.empty()&&map.isRawRaySnapshot()&&
       (!official_geometry||!map.planningRequiresObservedFree())&&
       map.localizationContextSequence()==job.version.context_sequence&&s.order==3&&s.pos_pts.size()>=4&&
       s.pos_pts.size()<=10000&&s.knots.size()==s.pos_pts.size()+4&&job.spline.frame_id==frame_) {
      Eigen::MatrixXd points(3,s.pos_pts.size());Eigen::VectorXd knots(s.knots.size());
      for(std::size_t i=0;i<s.pos_pts.size();++i)points.col(i)=Eigen::Vector3d(s.pos_pts[i].x,s.pos_pts[i].y,s.pos_pts[i].z);
      for(std::size_t i=0;i<s.knots.size();++i)knots[i]=s.knots[i];
      bool ordered=knots.allFinite();for(int i=1;i<knots.size();++i)ordered=ordered&&knots[i]>knots[i-1];
      if(points.allFinite()&&ordered) {
        UniformBspline curve(points,3,.1);curve.setKnot(knots);
        auto derivative=curve.getDerivative();double speed_bound=0.;
        const auto derivative_controls=derivative.getControlPoint();
        for(int i=0;i<derivative_controls.cols();++i)
          speed_bound=std::max(speed_bound,derivative_controls.col(i).norm());
        const auto heading_intervals=bodyHeadingSamplingIntervals(curve.getTimeSum(),speed_bound,map.getResolution());
        // The formal controller replays precisely this frozen heading lattice.
        // A well-formed but different count could change low-speed orientation
        // memory, so range checking alone is not a geometry certificate.
        const bool heading_wire_valid=!local_follower_||
          (heading_intervals&&*heading_intervals==job.spline.body_heading_sampling_intervals&&
           job.spline.body_heading_seed_valid&&job.spline.body_heading_seed_yaw==job.heading.measured_yaw);
        if(!heading_wire_valid)result.reason="invalid_frozen_body_heading_sampling_contract";
        else {
        std::optional<Job> predecessor;
        if(local_follower_&&job.spline.splice.enabled) {
          std::lock_guard<std::mutex> l(mutex_);
          if(active_&&(active_->version!=job.version||active_->spline.trajectory.traj_id!=result.trajectory_id)&&
              active_->version==job.spline.splice.incumbent_version&&
              active_->spline.trajectory.traj_id==job.spline.splice.incumbent_trajectory_id)predecessor=active_;
        }
        if(predecessor) {
          bool occupied=false,unknown=false;CurveCheckCompletion completion=CurveCheckCompletion::Uncertified;
          MeasuredCurveDomain tail_entry;
          const auto& p=*predecessor;auto incumbent=decodeSpliceCurve(p.spline.trajectory);
          const auto progress=progressAtBodySource(p,clock(),acquired_body_source_ns);
          if(p.progress&&!progress) {
            finish_query();
            logMissingSourcePair(result.trajectory_id,"splice_predecessor",
              acquired_body_source_ns,ns(p.progress->header.stamp));
            if(observation){observation->event.check.disposition.set("waiting_splice_source_pair");
              observation->event.reason.set("source_pair_missing");
              observation->event.check.progress_source_ns=ns(p.progress->header.stamp);}
            return false;
          }
          const bool compatible_support=splicePlanningSupportCompatible(p,job);
          bool dependencies=compatible_support&&incumbent&&progress&&usablePlanningGeometry(p)&&p.support->version==p.version&&
            p.support->frame_id==frame_&&!p.support->support_reference_id.empty()&&!p.support->support_hash.empty();
          if(dependencies) {
            auto heading=immutableHeadingAt(*incumbent,p.heading,job.spline.splice.splice_curve_time,map.getResolution());
            PlannedTrajectorySplice splice{job.spline.splice,*incumbent,p.heading,progress->curve_time,heading,
              ns(progress->header.stamp),std::max(progress->s_committed,p.progress->s_committed)};
            dependencies=std::isfinite(heading);
            const double remaining=std::min(budget_s-std::chrono::duration<double>(
                std::chrono::steady_clock::now()-check_started).count(),
                std::chrono::duration<double>(round_deadline-ValidationCycle::Clock::now()).count());
            const double geometry_extent=map.queryBodyExtent(official_geometry);
            result.valid=dependencies&&remaining>0.&&commonPrefixTailCollisionFree(curve,splice,map.getResolution(),
              geometry_extent,[&](const Eigen::Vector3d& point,double yaw){const int state=official_geometry?
                map.getPlanningInflateOccupancy(point,yaw):map.getInflateOccupancy(point,yaw);
                occupied=occupied||state==1;unknown=unknown||state==2||state<0;return state;},
              body,frame_,clock()*1e-9,.4,remaining,job.heading,&completion,nullptr,&tail_entry);
            resource_incomplete=completion==CurveCheckCompletion::BudgetExhausted;
          }
          result.whole_curve=true;result.remaining_curve=false;
          result.valid_start_time=tail_entry.measured_time;result.valid_start_arc_length=tail_entry.measured_arc;
          result.checked_from_time=0.;result.checked_to_time=result.curve_duration=curve.getTimeSum();
          result.reason=!compatible_support?"splice_stair_support_changed":result.valid?geometry_policy_:occupied?"occupied":unknown?"unknown_or_unobserved":
            !dependencies?"splice_incumbent_or_support_invalid":resource_incomplete?"curve_check_budget_exhausted":"splice_boundary_or_prefix_invalid";
        } else {
        // Original heading is immutable. Measured progress selects a remaining
        // collision domain, while replaying the original orientation memory;
        // it never reseeds or moves the accepted curve with a newer TF.
        const bool no_motion_preview=planning_only_geometry_&&job.no_motion_preview;
        const bool preview_candidate=planning_only_geometry_&&!job.no_motion_preview&&job.preview_candidate_origin.has_value();
        const auto projection_deadline=std::min(round_deadline,check_started+
            std::chrono::duration_cast<ValidationCycle::Clock::duration>(std::chrono::duration<double>(budget_s)));
        const auto connection=measuredJoin(job,curve,body,map.getResolution(),clock()*1e-9,
          projection_deadline,acquired_body_source_ns);
        if(connection.source_pair_missing) {
          finish_query();
          logMissingSourcePair(result.trajectory_id,"measured_join",
            acquired_body_source_ns,job.progress?ns(job.progress->header.stamp):0);
          awaitSourcePairBody(job,acquired_body_source_ns,clock());
          if(observation){observation->event.check.disposition.set("waiting_measured_source_pair");
            observation->event.reason.set("source_pair_missing");
            observation->event.check.progress_source_ns=job.progress?ns(job.progress->header.stamp):0;}
          return false;
        }
        // A bounded reader that did not finish projection has not witnessed
        // negative geometry. Formal execution retains the last completed
        // state; no new verdict/seed is published or obstacle suppressed.
        if(previewCandidateProjectionInconclusive(connection)&&
           (local_follower_||map.collisionSnapshotSourceFresh())&&
           clock()*1e-9-body.source_stamp<=.4&&
           map.localizationContextSequence()==job.version.context_sequence) {
          finish_query();
          if(observation){observation->event.check.disposition.set("projection_incomplete_no_proof");
            observation->event.reason.set("projection_incomplete");}
          return false;
        }
        const auto& domain=connection.remaining;const auto& preview=connection.preview;
        const auto& join=connection.time;const double join_residual=connection.residual;
        bool occupied=false,unknown=false;std::size_t queries=0;
        std::optional<std::pair<Eigen::Vector3d,double>> first_occupied_query;
        std::size_t first_occupied_index=0;
        auto completion=connection.completion;
        const auto query_begin=std::chrono::steady_clock::now();
        if(join) {
          const double geometry_extent=map.queryBodyExtent(official_geometry);
          const auto arc_domain=(no_motion_preview||preview_candidate||
              connection.connection==MeasuredConnectionPolicy::RollingCandidateConnection)?
              std::optional<MeasuredCurveDomain>(MeasuredCurveDomain{preview.curve_time,preview.arc_length,0.,curve.getTimeSum()}):
              domain?domain:measuredRemainingCurveDomain(curve,body.position,*join,0.,0.);
          result.valid_start_time=*join;result.valid_start_arc_length=arc_domain?arc_domain->measured_arc:0.;
          result.whole_curve=!domain;result.remaining_curve=domain.has_value();
          result.checked_from_time=domain?domain->checked_from_time:0.;
          result.checked_to_time=result.curve_duration=curve.getTimeSum();
          result.reverse_margin_m=domain?.15:0.;
          // SCAN's native geometry is authoritative. Support above supplies
          // identity, not another raw floor-height/slope/coverage classifier.
          const double remaining=std::min(
            budget_s-std::chrono::duration<double>(std::chrono::steady_clock::now()-check_started).count(),
            std::chrono::duration<double>(round_deadline-ValidationCycle::Clock::now()).count());
          result.valid=arc_domain.has_value()&&remaining>0.&&wholeCurveCollisionFree(curve,map.getResolution(),geometry_extent,
            [&](const Eigen::Vector3d& p,double yaw){++queries;const int state=official_geometry?
                map.getPlanningInflateOccupancy(p,yaw):map.getInflateOccupancy(p,yaw);
              if(state==1&&!first_occupied_query) {
                first_occupied_query=std::make_pair(p,yaw);first_occupied_index=queries;
              }
              occupied=occupied||state==1;unknown=unknown||state==2||state<0;return state;},
            body,frame_,clock()*1e-9,.4,200000,remaining,*join,job.heading,result.checked_from_time,
            connection.connection,preview_candidate?job.preview_candidate_origin->policy.connector_m:
              NoMotionPreviewConnectionPolicy::max_connector_m,&completion,
            std::numeric_limits<double>::infinity(),{},
            official_geometry&&!map.planningRequiresObservedFree());
          resource_incomplete=completion==CurveCheckCompletion::BudgetExhausted||
              (arc_domain.has_value()&&remaining<=0.);
          if(result.valid&&planning_only_geometry_) {
            if(no_motion_preview||preview_candidate)next_preview_progress=preview;
            else next_preview_progress=NoMotionPreviewProgress{NoMotionPreviewStatus::Connected,*join,
              result.valid_start_arc_length,result.valid_start_arc_length,0.,body.source_stamp,body.position};
          }
          if(result.valid&&job.has_goal_yaw&&std::isfinite(job.goal_yaw)&&
             (body.position-job.goal).norm()<=.25) {
            double measured_yaw=0.;
            if(measuredBodyYaw(body,frame_,clock()*1e-9,.4,measured_yaw)) {
              const auto goal_turn_query=[&](const Eigen::Vector3d& p,double yaw){
                  return ValidationCycle::Clock::now()<round_deadline&&
                    std::chrono::duration<double>(std::chrono::steady_clock::now()-check_started).count()<budget_s&&
                    (official_geometry?map.getPlanningInflateOccupancy(p,yaw):map.getInflateOccupancy(p,yaw))==0;
                };
              result.goal_yaw_checked=job.heading.align_then_forward?
                bodyEntryHeadingTransitionFree(body.position,body.position,measured_yaw,job.goal_yaw,
                  map.getResolution(),geometry_extent,goal_turn_query):
                headingTransitionFree(body.position,body.position,measured_yaw,job.goal_yaw,
                  map.getResolution(),geometry_extent,goal_turn_query);
              if(result.goal_yaw_checked)result.checked_goal_yaw=job.goal_yaw;
            }
          }
        }
        const double query_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-query_begin).count();
        result.reason=result.valid?geometry_policy_:occupied?"occupied":unknown?"unknown_or_unobserved":
          !join?(preview_candidate?candidateAcceptanceReason(connection.candidate_outcome):
            no_motion_preview&&preview.status==NoMotionPreviewStatus::ReplanFromMeasuredBody?
            "preview_connection_out_of_range":"measured_projection_or_connection_invalid"):
          (completion==CurveCheckCompletion::BudgetExhausted||ValidationCycle::Clock::now()>=round_deadline||
           std::chrono::duration<double>(std::chrono::steady_clock::now()-check_started).count()>=budget_s)?
            "curve_check_budget_exhausted":"curve_or_connection_geometry_invalid";
        // check() can run on two independent readers. Do not share a mutable
        // logging-throttle static between them; source leases remain untouched.
        const auto log_now=std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        auto prior_log=proof_log_steady_ns_.load();
        if(node_&&!result.valid&&log_now-prior_log>=250000000LL&&
           proof_log_steady_ns_.compare_exchange_strong(prior_log,log_now)) {
          RCLCPP_WARN(node_->get_logger(),
          "Execution proof curve=%ld measured_progress=%d residual=%.6f queries=%zu check_ms=%.3f reason=%s completion=%s body_source_ns=%ld progress_source_ns=%ld",
          result.trajectory_id,domain.has_value(),join_residual,queries,query_seconds*1000.,result.reason.c_str(),
          curveCheckCompletionName(completion),acquired_body_source_ns,connection.progress_source_ns);
          if(first_occupied_query) {
            const auto diagnostic_begin=std::chrono::steady_clock::now();
            const auto detail=describeCurveCollisionWitness(map,first_occupied_query->first,
                first_occupied_query->second,body);
            RCLCPP_WARN(node_->get_logger(),
              "ExecutionCollisionWitness curve=%ld query_index=%zu diagnostics_ms=%.3f %s",
              result.trajectory_id,first_occupied_index,
              std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-diagnostic_begin).count(),
              detail.c_str());
          }
        }
        }
        }
      }
    }
    if(result.valid&&ValidationCycle::Clock::now()>=round_deadline) {
      result.valid=false;result.reason="curve_check_budget_exhausted";resource_incomplete=true;
    }
    // An incomplete attempt is neither a collision nor a new free verdict.
    // Formal execution retains its last completed state; only the archived
    // protocol retains a previous proof up to its original expiry.
    if((local_follower_&&(resource_incomplete||
          ValidationCycle::inconclusiveRefresh(skip_budget_failure,result.reason)))||ValidationCycle::incompleteProof(resource_incomplete||
          ValidationCycle::inconclusiveRefresh(skip_budget_failure,result.reason),
       map.collisionSnapshotSourceFresh()&&clock()*1e-9-body.source_stamp<=.4&&
       map.localizationContextSequence()==job.version.context_sequence)) {
      finish_query();
      // Scheduling failure cannot manufacture either a collision or a new
      // positive state. The independent component watchdog still observes
      // the absence of completed current checks.
      bool retained=false;std::int64_t previous_deadline=0;
      const auto now=clock();
      if(observation) {
        auto& e=observation->event;e.ros_ns=now;e.reason.set(result.reason);
        e.check.check_end_ns=now;e.check.disposition.set("incomplete_no_proof");
      }
      {std::lock_guard<std::mutex> l(mutex_);
        if(active_&&active_->version==job.version&&active_->spline.trajectory.traj_id==result.trajectory_id&&active_->proof) {
          previous_deadline=ns(active_->proof->valid_until);
          retained=active_->proof->valid&&curveVerdictCurrent(*active_->proof,now);
        }
      }
      const auto log_now=std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      auto prior=budget_log_steady_ns_.load();
      if(node_&&log_now-prior>=1000000000LL&&budget_log_steady_ns_.compare_exchange_strong(prior,log_now))
        RCLCPP_WARN(node_->get_logger(),"Native current-curve refresh incomplete curve=%ld formal_state=%d retained_previous_positive=%d legacy_lease_remaining_ms=%.3f; no new verdict issued",
          result.trajectory_id,local_follower_,retained,std::max<std::int64_t>(0,previous_deadline-now)*1e-6);
      {std::lock_guard<std::mutex> l(mutex_);
        if(local_follower_&&candidate_&&sameJob(*candidate_,job))advanceCandidateLocked();}
      return false;
    }
    result.check_end=rclcpp::Time(clock());result.source_stamp=result.check_end;
    result.valid_until=rclcpp::Time(0); // Formal completed state, explicitly not a lease.
    if(!local_follower_) {
      const auto sensor_age=braking_?braking_->raySourceAgeNs():500000000LL;
      const auto evidence_deadline=official_geometry?std::numeric_limits<std::int64_t>::max():map.observedProofDeadlineNs();
      // Preserve the archived protocol's exact source-bound lease semantics.
      const auto min_deadline=std::min<std::int64_t>({ns(result.check_end)+250000000LL,
        evidence_deadline,map.integratedRaySourceStamp(0)+sensor_age,
        map.integratedRaySourceStamp(1)+sensor_age,acquired_body_source_ns+400000000LL});
      result.valid_until=rclcpp::Time(std::max<std::int64_t>(0,min_deadline));
      if(!map.integratedCloudFreshAt(clock())||clock()*1e-9-body.source_stamp>.4||min_deadline<=clock()) {
        if(result.valid)result.reason="source_expired_during_check";
        result.valid=false;
      }
    }
    if(observation) {
      auto& e=observation->event;e.ros_ns=ns(result.check_end);e.valid_until_ns=ns(result.valid_until);
      e.reason.set(result.reason);e.check.check_end_ns=ns(result.check_end);e.check.valid=result.valid;
    }
    const auto discarded=[&](std::string_view why) {
      if(observation)observation->event.check.disposition.set(why);
      return false;
    };
    // Collision result, evidence deadline, original source times and immutable
    // query metadata are now complete. Release before ledger waits and any
    // outbound publication; none of the following work queries this snapshot.
    finish_query();
    // A canceled/replaced job can never publish a late positive proof.
    {std::lock_guard<std::mutex> l(mutex_);
      const auto same=[&](const std::optional<Job>& j){return j&&j->version==job.version&&j->spline.trajectory.traj_id==result.trajectory_id;};
      if(!same(active_)&&!same(candidate_)&&!same(prepared_)&&!same(local_transition_incumbent_))
        return discarded("job_retired_or_replaced");
      // A future-tail proof certifies ONE immutable incumbent prefix. If that
      // predecessor was replaced during the native query, this result is not
      // evidence for the new active curve. A committed tail is checked as an
      // ordinary actual curve instead and does not require its retired parent.
      if(result.valid&&job.spline.splice.enabled&&!same(active_)&&!same(local_transition_incumbent_)&&
          (!active_||active_->version!=job.spline.splice.incumbent_version||
           active_->spline.trajectory.traj_id!=job.spline.splice.incumbent_trajectory_id))return discarded("splice_superseded");
      // Both immutable-map readers can start from the same old projection.
      // A newer-source result must not overwrite a subsequently certified
      // high-water seed. Recompute its positive proof on the next round;
      // witnessed negative evidence still fences the old geometry immediately.
      if(result.valid&&!positivePreviewCheckStillCurrentLocked(job))return discarded("projection_superseded");
      // A slower earlier check must never resurrect a curve after a later
      // negative proof or overwrite the newest measured-position validation.
      if((same(active_)&&active_->proof&&!proofSupersedes(result,*active_->proof))||
         (same(local_transition_incumbent_)&&local_transition_incumbent_->proof&&
          !proofSupersedes(result,*local_transition_incumbent_->proof))||
         (same(candidate_)&&candidate_->proof&&!proofSupersedes(result,*candidate_->proof))||
         (same(prepared_)&&prepared_->proof&&!proofSupersedes(result,*prepared_->proof)))return discarded("proof_superseded");
      // Commands already retain every current proof sequence/deadline. Avoid
      // repeating identical-source positive renewals in diagnostics; keep
      // every candidate result, negative and current sensor/map transition.
      const bool trace_result=local_follower_&&(!same(active_)||!active_->proof||
        !active_->proof->valid||!result.valid||
        active_->proof->front_ray_source_stamp!=result.front_ray_source_stamp||
        active_->proof->rear_ray_source_stamp!=result.rear_ray_source_stamp||
        active_->proof->map_snapshot_revision!=result.map_snapshot_revision);
      if(same(active_))recordCommittedProofLocked(result);
      if(same(local_transition_incumbent_))local_transition_incumbent_->proof=result;
      if(same(candidate_))candidate_->proof=result;
      if(same(prepared_))prepared_->proof=result;
      completedPipelineCheckLocked(result,check_fusion_sequence);
      if(observation){observation->event.accepted=true;
        observation->event.check.ledger_steady_ns=BoundedExecutionTrace::steadyNowNs();
        observation->event.check.disposition.set("output_exception");}
      if(trace_result) {
        BoundedExecutionTrace::Event event;event.kind=BoundedExecutionTrace::Kind::CurveProof;
        event.ros_ns=ns(result.check_end);event.source_ns=ns(result.front_ray_source_stamp);
        event.rear_source_ns=ns(result.rear_ray_source_stamp);event.body_ns=ns(result.body_source_stamp);
        event.valid_until_ns=ns(result.valid_until);event.proof_until_ns=ns(result.valid_until);
        event.proof=result.sequence;event.map_revision=result.map_snapshot_revision;
        event.accepted=result.valid;event.reason.set(result.reason);
        traceLocked(event,{&result.version,result.trajectory_id,result.proposal_id});
      }
      proofs_.push_back(result);while(proofs_.size()>64)proofs_.pop_front();
      if(local_follower_&&result.valid) {local_proof_facts_.push_back(result);while(local_proof_facts_.size()>16)local_proof_facts_.pop_front();}
      if(result.valid) {
        for(auto* current:{&active_,&candidate_,&prepared_})if(same(*current)) {
          (*current)->measured_time=result.valid_start_time;
          if(next_preview_progress) {
            (*current)->preview_progress=next_preview_progress;
            (*current)->preview_projection_resolution=query_resolution;
            // A slow proof must not rewind newer acquired-body history. This
            // initialization/advance is purely geometric; the proof retains
            // its own original body/ray source stamps and expiry above.
            if(!(*current)->observed_preview_progress||advanceNoMotionPreviewObservation(
                *(*current)->observed_preview_progress,*next_preview_progress))
              (*current)->observed_preview_progress=next_preview_progress;
          }
        }
      }
      // Formal geometry state has no offer expiry. Supersession, a real
      // negative result or the follower's finite exchange retires a slot.
      if(local_follower_&&same(candidate_)&&!result.valid)advanceCandidateLocked();
    }
    // Proofs, body samples and demands arrive independently. A successful
    // native check must complete a still-live pending demand immediately,
    // not wait for another controller/body callback. One slot, one emission;
    // the original sequence/source/TTL and stop barrier remain authoritative.
    CommandBatch commands;
    const auto collect_command=captureCommands(commands,emit_command,clock,CommandOrigin::Proof);
    if(local_follower_&&collect_command) {
      std::lock_guard<std::mutex> l(mutex_);
      if(active_&&active_->proof&&active_->proof->sequence==result.sequence&&
          pending_local_demand_&&!pending_local_demand_->waiting_for_commit&&
          pending_local_demand_->demand.version==result.version&&
          pending_local_demand_->demand.trajectory_id==result.trajectory_id) {
        const auto pending=*pending_local_demand_;pending_local_demand_.reset();
        if(result.valid&&pending.stop_barrier==motion_stop_barrier_&&latest_demand_&&
            pending.demand.execution_id==latest_demand_->execution_id&&
            pending.demand.sequence==latest_demand_->sequence&&
            originalLocalDemandFreshLocked(pending.demand,clock())) {
          relayOrDeferLocalDemandLocked(pending.demand,clock(),collect_command,false,false);
        }
      }
      refreshRelayedLocalCommandLocked(result,clock(),collect_command);
    }
    deliverCommands(commands,emit_command,clock);
    // The completed Native fact is already in the ledger. Complete/renew the
    // exact still-live original command BEFORE publishing its notification:
    // a reliable proof sink may block past the old ray-derived deadline and
    // otherwise force the SDK to emit zero despite the completed fresh check.
    // Relay remains serialized with cancel/negative proof/lease updates and
    // never changes the command, body source, sequence or authorization.
    if(observation)observation->event.check.output_begin_steady_ns=BoundedExecutionTrace::steadyNowNs();
    if(emit_proof)emit_proof(result);
    if(observation){observation->event.check.output_end_steady_ns=BoundedExecutionTrace::steadyNowNs();
      observation->event.check.disposition.set(emit_proof?"proof_publish_returned":"ledger_only_no_publisher");}
    return result.valid;
  }
  rclcpp::Node*node_;CollisionSnapshotPool&pool_;std::string frame_,mode_;double extent_;
  mutable std::mutex mutex_;std::mutex wait_mutex_;std::condition_variable wake_;
  mutable std::atomic<std::uint64_t> slow_snapshot_hint_misses_{0};
  bool snapshot_ready_{false};
  std::atomic<bool>stop_{false};std::thread thread_,admission_thread_;
  std::mutex admission_wait_mutex_;std::condition_variable admission_wake_;bool admission_ready_{false};
  // Only dedicated odometry enters this bounded history. A tracker projection
  // cannot attest to itself, and source-time lookup never extrapolates.
  std::optional<double> measuredCurveDeviationLocked(std::int64_t now)const {
    if(!active_||!active_->progress||!active_->progress->valid)return {};
    return measuredCurveDeviationLocked(now,*active_->progress);
  }
  std::optional<double> measuredCurveDeviationLocked(std::int64_t now,
      const ew::TrackingProgress& progress)const {
    if(!active_||!progress.valid)return {};
    const auto source=ns(progress.header.stamp);
    if(source<=0||source>now+20000000LL||now-source>400000000LL)return {};
    const auto body=bodyAtSourceLocked(source);
    auto curve=decodeSpliceCurve(active_->spline.trajectory);
    if(!body||!curve||!body->pose.position.allFinite()||
       !std::isfinite(progress.curve_time)||progress.curve_time<0.||
       progress.curve_time>curve->getTimeSum())return {};
    // Compare the independently acquired body at the SAME source timestamp;
    // callback ordering and an older projection must not invent deviation.
    return (body->pose.position-curve->evaluateDeBoorT(progress.curve_time)).norm();
  }
  std::optional<ew::TrackingProgress> pairedProgressLocked(const ew::ExecutionVersion& task,
      std::int64_t now,const MeasuredBodyPose& actual,std::int64_t actual_source_ns)const {
    if(task_closed_||!active_||!sameExecutionTask(task,active_->version)||
       !active_->progress||!active_->progress->valid||active_->progress->holding||
       actual_source_ns<=0||actual_source_ns>now+20000000LL||now-actual_source_ns>400000000LL||
       actual.frame!=frame_||!actual.position.allFinite()||!actual.orientation.coeffs().allFinite()||
       actual.orientation.norm()<1e-6||!std::isfinite(actual.source_stamp))return {};
    // Require the exact raw source sample. Interpolation must not relabel the
    // owner's pose as a different acquired body; no new measurement is made.
    const auto body=std::lower_bound(body_history_.begin(),body_history_.end(),actual_source_ns,
      [](const BodySourceSample& s,std::int64_t stamp){return s.source_ns<stamp;});
    if(body==body_history_.end()||body->source_ns!=actual_source_ns||body->pose.frame!=actual.frame||
       body->pose.source_stamp!=actual.source_stamp||
       (body->pose.position-actual.position).norm()>1e-6||
       body->pose.orientation.normalized().angularDistance(actual.orientation.normalized())>1e-6)return {};
    const auto progress=progressAtBodySource(*active_,now,actual_source_ns);
    if(!progress||progress->holding)return {};
    return *progress;
  }
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
  LocalNavigationBodyInput local_body_input_;
  MeasuredBodyPose body_;std::optional<Job>active_,candidate_,queued_candidate_;std::optional<ew::SupportReference>pending_support_;
  std::optional<Job> local_transition_incumbent_;
  std::optional<ew::ExecutionVersion> candidate_reference_;
  std::optional<ew::ExecutionVersion> rolling_stair_predecessor_;
  std::int64_t candidate_offer_deadline_ns_{0};
  std::optional<ew::TaggedBspline> selected_candidate_publication_;
  std::mutex selected_candidate_publication_mutex_;
  // Passive real-component fixtures observe the same selected publications;
  // production uses only the ROS publisher below.
  std::function<void(const ew::TaggedBspline&)> selected_candidate_observer_;
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
  struct PendingLocalDemand {ew::MotionDemand demand;std::uint64_t stop_barrier;bool waiting_for_commit;};
  std::optional<PendingLocalDemand> pending_local_demand_;
  struct SourcePairBodyWait {
    ew::ExecutionVersion version;std::int64_t trajectory_id,required_body_source_ns,original_deadline_ns;
    std::uint64_t proof_sequence,stop_barrier;
  };
  std::optional<SourcePairBodyWait> source_pair_wait_;
  std::uint64_t local_proof_deferred_commands_{0},local_proof_completed_commands_{0};
  MotionCheckDiagnostics motion_diagnostics_;
  BoundedExecutionTrace execution_trace_;
  BoundedExecutionTrace curve_check_trace_;
  std::int64_t selected_candidate_enqueued_steady_ns_{0};
  d1max_diagnostics::BoundedTaskDiagnosticFile execution_trace_file_{"scan"};
  bool execution_trace_path_reported_{false};
  bool execution_trace_failure_reported_{false};
  std::uint64_t last_execution_trace_sequence_{0};
  d1max_diagnostics::BoundedTaskDiagnosticFile curve_check_trace_file_{"scan_check"};
  bool curve_check_trace_path_reported_{false},curve_check_trace_failure_reported_{false};
  std::uint64_t last_curve_check_trace_sequence_{0};
  std::chrono::steady_clock::time_point last_motion_diagnostics_publication_{};
  std::chrono::steady_clock::time_point last_motion_event_log_{};
  std::uint64_t last_motion_event_log_sequence_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_diagnostics_pub_;
  std::optional<ew::ExecutionPermit> latest_permit_;
  std::deque<ew::ExecutionPermit> permits_;
  std::deque<ew::TrajectoryValidation> proofs_;
  std::deque<ew::TrajectoryValidation> local_proof_facts_;
  std::optional<ew::LocalTrajectoryCommit> local_prepare_request_,local_commit_fact_;
  std::uint64_t local_prepare_floor_{0};
  std::optional<ew::TaskExecutionLease> task_lease_;
  std::deque<ew::TaskExecutionLease> task_lease_history_;
  std::uint64_t task_lease_sequence_{0},local_commit_sequence_{0};
  std::uint64_t local_motion_passes_{0},local_commands_staged_{0},local_commands_published_{0},local_expired_commands_{0};
  std::optional<ew::LocalMotionCommand> relayed_local_command_;
  std::uint64_t relayed_local_stop_barrier_{0},local_evidence_updates_{0};
  std::uint64_t local_body_deferred_commands_{0},local_body_completed_commands_{0};
  std::uint64_t local_commit_deferred_commands_{0},local_commit_completed_commands_{0};
  std::int64_t local_last_motion_pass_ns_{0},local_max_motion_pass_ns_{0};
  std::int64_t local_source_to_command_ns_{0},local_body_to_command_ns_{0};
  std::int64_t local_max_source_to_command_ns_{0},local_max_body_to_command_ns_{0};
  std::deque<ew::MotionValidation> motion_proofs_;
  std::deque<ew::MotionValidation> prepared_motion_proofs_,tombstone_motion_proofs_;
  std::optional<ew::MotionValidation> blocked_entry_;
  std::uint64_t last_blocked_entry_sequence_{0};
  std::optional<ew::TrajectoryValidation> committed_collision_;
  std::optional<std::pair<ew::ExecutionVersion,std::int64_t>> last_committed_collision_;
  std::optional<ew::TrajectoryAdmission> entry_reseed_;
  std::uint64_t last_entry_admission_sequence_{0};
  std::optional<std::pair<ew::ExecutionVersion,std::int64_t>> last_entry_reseed_,last_terminal_reseed_;
  std::optional<std::pair<ew::ExecutionVersion,std::int64_t>> last_deviation_reseed_;
  std::optional<std::pair<ew::ExecutionVersion,std::int64_t>> last_preview_candidate_reseed_;
  std::shared_ptr<const MotionSupport> motion_support_;
  std::deque<std::shared_ptr<const MotionSupport>> support_history_;
  std::deque<std::shared_ptr<const PlanningSupport>> planning_support_history_;
  std::uint64_t motion_sequence_{0},motion_stop_barrier_{0};
  std::mutex command_delivery_mutex_;
  std::optional<ew::LocalMotionCommand> last_delivered_local_command_;
  std::atomic<std::uint64_t> sequence_{0};std::uint64_t permit_sequence_{0};
  std::atomic<std::int64_t> proof_log_steady_ns_{0};
  std::atomic<std::int64_t> budget_log_steady_ns_{0};
  rclcpp::Publisher<ew::TrajectoryValidation>::SharedPtr pub_;
  rclcpp::Publisher<ew::TaggedBspline>::SharedPtr committed_pub_;
  std::shared_ptr<const ew::TaggedBspline> pending_committed_display_;
  rclcpp::CallbackGroup::SharedPtr committed_display_group_;
  rclcpp::TimerBase::SharedPtr committed_display_timer_;
  rclcpp::TimerBase::SharedPtr execution_trace_timer_;
  rclcpp::Publisher<ew::TaggedBspline>::SharedPtr selected_candidate_pub_;
  rclcpp::CallbackGroup::SharedPtr selected_publication_group_;
  rclcpp::TimerBase::SharedPtr selected_publication_timer_;
  rclcpp::Publisher<ew::MotionValidation>::SharedPtr motion_pub_;
  rclcpp::Publisher<ew::LocalMotionCommand>::SharedPtr local_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pipeline_pub_;
  std::uint64_t last_timing_publication_{0};
  bool writer_handoff_{false},grant_retired_{false},grant_applied_{false},tombstone_retired_{false};
  bool planning_only_geometry_{false};
  bool local_follower_{false};
  std::string body_frame_{"base_link"};
  std::string geometry_policy_{"observed_free"};
  std::function<NativePipelineHealth()> pipeline_health_;
  std::int64_t component_timeout_ns_{1500000000LL},current_check_completed_steady_ns_{0};
  std::int64_t current_checked_fusion_progress_steady_ns_{0};
  std::uint64_t current_checked_fusion_{0};
  bool pipeline_fault_{false};
  std::array<std::int64_t,2> pipeline_fault_sources_{{0,0}};
  std::uint64_t pipeline_fault_fusion_{0};
  std::optional<Job> prepared_,tombstone_job_;
  std::optional<ew::ExecutionHandoffGrant> grant_,tombstone_grant_;
  std::optional<ew::PreparedMotionDemand> prepared_demand_;
  std::uint64_t writer_commit_sequence_{0},ack_sequence_{0},prepared_stop_barrier_{0};
};
} // namespace scan_planner
