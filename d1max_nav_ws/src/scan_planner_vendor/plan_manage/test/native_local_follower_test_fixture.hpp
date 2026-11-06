#pragma once
#include <gtest/gtest.h>
#include <plan_manage/execution_validator.hpp>
#include <plan_manage/reference_target.hpp>
#include <plan_manage/final_endpoint_reseed.hpp>
#include <d1max_trajectory_tracker/local_follower.hpp>
#include <thread>

namespace scan_planner {
struct ExecutionValidatorTestAccess {
  template<class Validator> static void overflowCommandBatch(Validator& v) {
    typename Validator::CommandBatch batch;
    const auto capture=v.captureCommands(batch,[](const ew::LocalMotionCommand&){},[]{return 100000000000LL;});
    std::lock_guard<std::mutex> lock(v.mutex_);
    ew::LocalMotionCommand c;capture(c);capture(c);capture(c);
  }
  static auto holdLedger(ExecutionValidator& v){return std::unique_lock<std::mutex>(v.mutex_);}
  template<class Check> static auto currentSnapshotRead(ExecutionValidator& v,Check&& check) {
    return v.withCurrentCurveSnapshot(std::forward<Check>(check));
  }
  // The fallback overload permits an identical-input before/after build of
  // this fixture against the frozen pre-fix header. It does not emulate a map.
  template<class Validator,class Job> static auto checkWithLeaseFinish(Validator& v,const Job& job,
      const MeasuredBodyPose& body,GridMap& map,const std::function<std::int64_t()>& clock,
      const std::function<void(const ew::TrajectoryValidation&)>& proof,
      const std::function<void(const ew::LocalMotionCommand&)>& command,
      const std::function<void()>& finish,int)
      ->decltype(v.checkAt(job,body,map,clock,proof,.12,false,ValidationCycle::Clock::time_point::max(),command,finish)) {
    return v.checkAt(job,body,map,clock,proof,.12,false,ValidationCycle::Clock::time_point::max(),command,finish);
  }
  template<class Validator,class Job> static bool checkWithLeaseFinish(Validator& v,const Job& job,
      const MeasuredBodyPose& body,GridMap& map,const std::function<std::int64_t()>& clock,
      const std::function<void(const ew::TrajectoryValidation&)>& proof,
      const std::function<void(const ew::LocalMotionCommand&)>& command,
      const std::function<void()>&,long) {
    return v.checkAt(job,body,map,clock,proof,.12,false,ValidationCycle::Clock::time_point::max(),command);
  }
  static auto checkPooledCurveClock(ExecutionValidator& v,const std::function<std::int64_t()>& clock,
      const std::function<void(const ew::TrajectoryValidation&)>& on_proof={},
      const std::function<void(const ew::LocalMotionCommand&)>& on_command={},
      const std::function<void()>& on_released={}) {
    std::optional<ew::TrajectoryValidation> out;
    std::optional<ExecutionValidator::Job> job;MeasuredBodyPose body;
    {std::lock_guard<std::mutex> lock(v.mutex_);job=v.active_;body=v.body_;}
    v.withCurrentCurveSnapshot([&](GridMap& map,const std::function<void()>& finish={}) {
      if(job)checkWithLeaseFinish(v,*job,body,map,clock,
        [&](const auto& p){out=p;if(on_proof)on_proof(p);},on_command,
        [&]{if(finish)finish();if(on_released)on_released();},0);
    });
    return out;
  }
  static auto checkPooledCurve(ExecutionValidator& v,std::int64_t now,
      const std::function<void(const ew::TrajectoryValidation&)>& on_proof={},
      const std::function<void(const ew::LocalMotionCommand&)>& on_command={},
      const std::function<void()>& on_released={}) {
    return checkPooledCurveClock(v,[=]{return now;},on_proof,on_command,on_released);
  }
  template<class Validator,class Check>static auto admissionSnapshotRead(Validator& v,Check&& check,int)
      ->decltype(v.withAdmissionSnapshot(true,std::forward<Check>(check))) {
    return v.withAdmissionSnapshot(true,std::forward<Check>(check));
  }
  // Exact pre-fix runAdmission lease scope for the same-input old-header run.
  // The exact complete old/new production loops are exercised separately by
  // test_admission_reader_scope_no_ros.py; this fallback still queries the
  // real native GridMap/pool and never emulates collision geometry.
  template<class Validator,class Check>static bool admissionSnapshotRead(Validator& v,Check&& check,long) {
    auto snapshot=v.pool_.borrowAdmission();
    if(!snapshot)return false;
    if(validationReadRequiresObservedSnapshot(v.geometry_policy_,false))snapshot->requireObservedSnapshot();
    check(*snapshot,std::function<void()>{});return true;
  }
  static auto checkPooledCandidate(ExecutionValidator& v,std::int64_t now,
      const std::function<void(const ew::TrajectoryValidation&)>& on_proof={},
      const std::function<void()>& on_released={}) {
    std::optional<ew::TrajectoryValidation> out;
    std::optional<ExecutionValidator::Job> job;MeasuredBodyPose body;
    {std::lock_guard<std::mutex> lock(v.mutex_);job=v.prepared_?v.prepared_:v.candidate_;body=v.body_;}
    admissionSnapshotRead(v,[&](GridMap& map,const std::function<void()>& finish) {
      if(job)v.checkAt(*job,body,map,[=]{return now;},
        [&](const auto& p){out=p;if(on_proof)on_proof(p);},.12,false,
        ValidationCycle::Clock::time_point::max(),{},
        [&]{if(finish)finish();if(on_released)on_released();});
    },0);
    return out;
  }
  static auto localDiagnosticSnapshot(const ExecutionValidator& v){return v.copyLocalMotionDiagnostics();}
  static void selectedObserver(ExecutionValidator& v,
      std::function<void(const ew::TaggedBspline&)> observer) {v.selected_candidate_observer_=std::move(observer);}
  static void dispatchSelected(ExecutionValidator& v){v.publishSelectedCandidate();}
  static void expireCandidateOffer(ExecutionValidator& v,std::int64_t now) {
    {std::lock_guard<std::mutex> l(v.mutex_);v.expireCandidateOfferLocked(now);}
    v.publishSelectedCandidate();
  }
  static auto queuedCurve(const ExecutionValidator& v) {
    return v.queued_candidate_?std::optional<ew::TaggedBspline>{v.queued_candidate_->spline}:std::nullopt;
  }
  static auto candidateOfferDeadline(const ExecutionValidator& v){return v.candidate_offer_deadline_ns_;}
  static auto pinnedSupport(const ExecutionValidator& v,bool active=false) {
    const auto& job=active?v.active_:v.candidate_;
    return job?job->geometry_support:std::shared_ptr<const PlanningSupport>{};
  }
  static auto pendingSupportCount(const ExecutionValidator& v){return v.planning_support_history_.size();}
  static auto checkCurveRound(ExecutionValidator& v,GridMap& map,
      const std::function<std::int64_t()>& clock,double budget,
      ValidationCycle::Clock::time_point deadline,bool active) {
    std::optional<ew::TrajectoryValidation> out;
    std::optional<ExecutionValidator::Job> job;MeasuredBodyPose acquired_body;
    {std::lock_guard<std::mutex> lock(v.mutex_);
      job=active?v.active_:v.candidate_;acquired_body=v.body_;} // Same copy/lock as production readers.
    if(job)v.checkAt(*job,acquired_body,map,clock,[&](const auto& p){out=p;},budget,false,deadline);
    return out;
  }
  static auto checkCurve(ExecutionValidator& v,GridMap& map,std::int64_t now,bool active=false,
      const std::function<void(const ew::LocalMotionCommand&)>& emit_command={}) {
    std::optional<ew::TrajectoryValidation> out;
    const auto job=active?v.active_:v.candidate_;
    if(job)v.checkAt(*job,v.body_,map,[=]{return now;},[&](const auto& p){out=p;},.12,false,
      ValidationCycle::Clock::time_point::max(),emit_command);
    return out;
  }
  static auto checkCurveClock(ExecutionValidator& v,GridMap& map,
      const std::function<std::int64_t()>& clock,bool active=false) {
    std::optional<ew::TrajectoryValidation> out;
    const auto job=active?v.active_:v.candidate_;
    if(job)v.checkAt(*job,v.body_,map,clock,[&](const auto& p){out=p;},.12);
    return out;
  }
  static auto checkDemand(ExecutionValidator& v,GridMap& map,std::int64_t now) {
    (void)map;
    return v.latest_demand_?v.relayLocalDemandLocked(*v.latest_demand_,now):std::optional<ew::LocalMotionCommand>{};
  }
  static auto latestDemand(const ExecutionValidator& v){return v.latest_demand_;}
  static bool pendingDemand(const ExecutionValidator& v){return v.pending_local_demand_.has_value();}
  static bool consumeRefreshWake(ExecutionValidator& v) {
    std::lock_guard<std::mutex> lock(v.wait_mutex_);
    const bool pending=v.snapshot_ready_;v.snapshot_ready_=false;return pending;
  }
  static auto pendingCurve(const ExecutionValidator& v) {
    return v.candidate_?std::optional<ew::TaggedBspline>{v.candidate_->spline}:std::nullopt;
  }
  static auto checkReserved(ExecutionValidator& v,GridMap& map,std::int64_t now) {
    std::optional<ew::TrajectoryValidation> out;const auto job=v.prepared_;
    if(job)v.checkAt(*job,v.body_,map,[=]{return now;},[&](const auto& p){out=p;},.12);
    return out;
  }
  static void forgetOriginalProofs(ExecutionValidator& v){v.local_proof_facts_.clear();}
  static auto motionProofCount(const ExecutionValidator& v){return v.motion_proofs_.size();}
  static void activeDebug(ExecutionValidator& v) {
    ew::LocalPlanDebug d;d.generation=v.active_->version.reference_generation;
    d.plan_id=v.active_->spline.trajectory.traj_id;v.active_->debug=d;
  }
  static void removeActiveProof(ExecutionValidator& v){v.active_->proof.reset();}
  static void expireCandidateProof(ExecutionValidator& v,std::int64_t deadline){
    v.candidate_->proof->valid_until=rclcpp::Time(deadline);
  }
  static void skipCurveProof(ExecutionValidator& v,GridMap& map,std::int64_t now) {
    v.checkMotionAt(map,[=]{return now;},{});
  }
  static auto canceledCopiedCurve(ExecutionValidator& v,GridMap& map,std::int64_t now) {
    const auto copied=*v.active_;auto barrier=copied.version;++barrier.reference_generation;
    EXPECT_TRUE(v.cancelReference(barrier));
    std::optional<ew::TrajectoryValidation> out;
    v.checkAt(copied,v.body_,map,[=]{return now;},[&](const auto& p){out=p;},.12);
    return out;
  }
  static auto checkCopiedBody(ExecutionValidator& v,GridMap& map,std::int64_t now,
      const MeasuredBodyPose& body,bool active=true) {
    std::optional<ew::TrajectoryValidation> out;
    const auto job=active?v.active_:v.candidate_;
    if(job)v.checkAt(*job,body,map,[=]{return now;},[&](const auto& p){out=p;},.12);
    return out;
  }
  static void braking(ExecutionValidator& v,const BrakingModel& b){v.braking_=b;}
  static void geometryPolicy(ExecutionValidator& v,bool official){v.geometry_policy_=official?"official_inflated_geometry":"observed_free";}
  template<class Validator>static auto pipelineHealthImpl(Validator& v,
      std::function<NativePipelineHealth()> provider,int)
      ->decltype(v.pipeline_health_=std::move(provider),void()) {
    v.pipeline_health_=std::move(provider);
  }
  template<class Validator>static void pipelineHealthImpl(Validator&,
      std::function<NativePipelineHealth()>,long) {} // Exact old-header decision regression only.
  static void pipelineHealth(ExecutionValidator& v,std::function<NativePipelineHealth()> provider) {
    pipelineHealthImpl(v,std::move(provider),0);
  }
  template<class Validator>static bool pipelineFault(const Validator& v){return v.pipeline_fault_;}
  static auto stopBarrier(const ExecutionValidator& v){return v.motion_stop_barrier_;}
  static bool relayedIntent(const ExecutionValidator& v){return bool(v.relayed_local_command_);}
  template<class Validator>static bool pipelineReady(Validator& v,std::int64_t now) {
    std::lock_guard<std::mutex> lock(v.mutex_);
    return v.active_&&v.nativePipelineReadyLocked(v.active_->version,now);
  }
  static auto diagnostics(const ExecutionValidator& v){return v.motion_diagnostics_.snapshot();}
  static auto executionTrace(const ExecutionValidator& v){return v.execution_trace_.snapshot();}
  static auto drainExecutionTrace(const ExecutionValidator& v,std::uint64_t& cursor){return v.execution_trace_.drain(cursor);}
  static auto checkTrace(const ExecutionValidator& v){return v.curve_check_trace_.snapshot();}
  static void flushTraces(ExecutionValidator& v,const std::filesystem::path& directory) {
    v.execution_trace_file_=d1max_diagnostics::BoundedTaskDiagnosticFile("scan",directory);
    v.curve_check_trace_file_=d1max_diagnostics::BoundedTaskDiagnosticFile("scan_check",directory);
    v.drainExecutionTraces(BoundedExecutionTrace::capacity);
  }
  static auto checkObservedPooled(ExecutionValidator& v,std::int64_t now,bool current=true,
      ValidationCycle::Clock::time_point deadline=ValidationCycle::Clock::time_point::max(),
      const std::function<void()>& on_released={},
      const std::function<void(const ew::TrajectoryValidation&)>& on_proof={}) {
    std::optional<ew::TrajectoryValidation> out;std::optional<ExecutionValidator::Job> job;MeasuredBodyPose body;
    ExecutionValidator::CurveCheckObservation observation(current);observation.event.ros_ns=now;
    {std::lock_guard<std::mutex> lock(v.mutex_);job=current?v.active_:v.candidate_;body=v.body_;
      v.captureCheckObservationLocked(observation,job);}
    ExecutionValidator::CurveCheckTraceScope trace{v,job?&*job:nullptr,observation};
    const auto query=[&](GridMap& map,const std::function<void()>& finish) {
      if(job)v.checkAt(*job,body,map,[=]{return now;},
        [&](const auto& p){out=p;if(on_proof)on_proof(p);},.12,current,deadline,{},
        [&]{if(finish)finish();if(on_released)on_released();},&observation);
    };
    if(current)v.withCurrentCurveSnapshot(query,&observation);
    else v.withAdmissionSnapshot(bool(job),query,&observation);
    return out;
  }
  static auto bodySource(const ExecutionValidator& v){return v.body_.source_stamp;}
  static auto bodyHistorySize(const ExecutionValidator& v){return v.body_history_.size();}
  static void corruptLatestTwist(ExecutionValidator& v,const geometry_msgs::msg::Twist& twist) {
    v.body_history_.back().twist=twist;
  }
  static void corruptLatestPosition(ExecutionValidator& v) {
    v.body_history_.back().pose.position.x()=std::numeric_limits<double>::quiet_NaN();
  }
  static auto checkCurvePublicationInterleave(ExecutionValidator& v,GridMap& map,std::int64_t now,
      const std::function<void(const ew::TrajectoryValidation&)>& on_publication,
      const std::function<void(const ew::LocalMotionCommand&)>& emit_command) {
    std::optional<ew::TrajectoryValidation> out;const auto job=v.active_;
    const auto acquired_body=v.body_; // Same immutable input copy as the real reader.
    if(job)v.checkAt(*job,acquired_body,map,[=]{return now;},[&](const auto& result) {
      out=result;on_publication(result);
    },.12,false,ValidationCycle::Clock::time_point::max(),emit_command);
    return out;
  }
  static auto checkCurrentAcrossCommit(ExecutionValidator& v,GridMap& map,std::int64_t now,
      const std::function<void()>& commit_before_result_ledger) {
    std::optional<ew::TrajectoryValidation> out;const auto job=v.active_;
    if(job)v.checkAt(*job,v.body_,map,[=]{return now;},[&](const auto& p){out=p;},.12,false,
      ValidationCycle::Clock::time_point::max(),{},commit_before_result_ledger);
    return out;
  }
};
}
// This is the production GridMap, not an alternate occupancy implementation.
// The buffers represent a completely observed flat room; each test injects
// acquired ray ages/voxels explicitly. No node/ROS context is constructed.
struct GridMapTestAccess {
  static bool anyQueryProvider(const GridMap& m) {
    return bool(m.query_attitude_provider_)||bool(m.query_support_height_provider_)||
      bool(m.query_stair_clearance_provider_)||bool(m.query_stair_contact_provider_)||
      bool(m.query_hard_boundary_provider_);
  }
  static void sourceBudget(GridMap& m,double seconds){m.mp_.cloud_pose_max_age_=seconds;}
  static void configure(GridMap& m,bool official,int x_cells=80) {
    auto& p=m.mp_;auto& d=m.md_;p.resolution_=.05;p.resolution_inv_=20.;
    p.map_voxel_num_={x_cells,80,60};p.map_origin_idx_={0,0,10};m.updateMapBoundaryFromIndex();
    p.use_projected_rays_=p.require_observed_free_=true;p.planning_collision_policy_="observed_free";
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.double_cylinder_radius_=.1;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.1;
    p.collision_model_="oriented_box";p.box_length_=.93;p.box_width_=.30;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    p.cloud_pose_max_age_=.5;p.frame_id_="odom";p.map_sliding_en_=false;
    if(official){p.require_observed_free_=false;p.planning_collision_policy_="official";}
    const auto cells=static_cast<std::size_t>(x_cells)*80*60;
    d.occupancy_buffer_.assign(cells,-1.);d.occupancy_buffer_inflate_.assign(cells,0);
    d.occupancy_buffer_inflate_cnt_.assign(cells,0);m.rebuildInflationOffsets();
    m.free_observation_stamps_.assign(cells,100000000000LL);
    m.collision_snapshot_=m.enforce_free_freshness_=true;m.ray_clock_fault_=false;
    m.localization_context_sequence_=m.localization_epoch_=1;m.occupancy_revision_=1;
    acquire(m,100000000000LL);
  }
  static void acquire(GridMap& m,std::int64_t source) {
    m.ray_integrated_stamps_={source,source};m.integrated_cloud_stamp_ns_=source;
    m.snapshot_clock_ns_=source;m.snapshot_captured_=std::chrono::steady_clock::now();
    for(std::size_t i=0;i<m.free_observation_stamps_.size();++i)
      m.free_observation_stamps_[i]=m.md_.occupancy_buffer_[i]>=m.mp_.clamp_min_log_?source:0;
    ++m.occupancy_revision_;
    ++m.fusion_timing_.sequence;
  }
  static void voxel(GridMap& m,const Eigen::Vector3d& p,double odds) {
    Eigen::Vector3i i;m.posToIndex(p,i);ASSERT_TRUE(m.isInMap(i));
    m.md_.occupancy_buffer_[m.toAddress(i)]=odds;++m.occupancy_revision_;
  }
  static double raw(GridMap& m,const Eigen::Vector3d& p) {
    Eigen::Vector3i i;m.posToIndex(p,i);return m.md_.occupancy_buffer_[m.toAddress(i)];
  }
  static std::int64_t freeStamp(GridMap& m,const Eigen::Vector3d& p) {
    Eigen::Vector3i i;m.posToIndex(p,i);return m.free_observation_stamps_[m.toAddress(i)];
  }
};
namespace {
namespace t=d1max_trajectory_tracker;
namespace w=d1max_planning_interfaces::msg;
using Native=scan_planner::ExecutionValidator;
using Access=scan_planner::ExecutionValidatorTestAccess;
t::Config config(bool official=true){t::Config c;c.session_id="session";c.map_version_id="map";c.planning_frame="odom";
  c.base_frame="base_link";c.map_frame="map_frame";c.motion_model="holonomic";c.holonomic_yaw_policy="hold_heading";
  c.trajectory_geometry_policy=official?"official_inflated_geometry":"observed_free";
  c.require_support_reference=c.external_goal_completion=true;return c;}
w::ExecutionVersion version(){w::ExecutionVersion v;v.schema_version=3;v.session_id="session";
  v.task_id="task";v.route_id="route";v.route_hash=std::string(64,'a');v.map_version_id="map";
  v.localization_epoch=v.anchor_revision=v.context_sequence=v.map_geometry_revision=v.reference_generation=1;
  v.localization_seed_id="seed";v.segment_id="floor1";v.anchor_id="anchor";return v;}
w::SupportReference support(){w::SupportReference s;s.version=version();s.support_reference_id="support";
  s.support_hash=std::string(64,'b');s.support_map_sha256=std::string(64,'c');s.floor_id="floor1";
  s.segment_kind="floor";s.required_mode="general";s.frame_id="odom";s.verified=true;
  s.source_stamp=t::stamp(100.);
  s.support_xy_radius_m=.04;s.body_reference_height_m=.55;s.max_support_slope_rad=.15;s.max_support_step_m=.05;
  for(int x=-30;x<=75;++x)for(int y=-10;y<=10;++y){geometry_msgs::msg::Point p;p.x=x*.025;p.y=y*.025;s.support_ground_xyz.push_back(p);}
  return s;}
w::ReferenceProposal proposal(){w::ReferenceProposal p;p.version=version();p.proposal_id="proposal";
  p.expected_trajectory_id=-1;p.source_stamp=t::stamp(100.);p.valid_until=t::stamp(100.9);
  p.reference.path.header.frame_id="odom";p.reference.point_reference="body_center";
  p.goal_position.x=3.;p.goal_position.z=.55;p.transport_mode="isolated_mock";return p;}
void freezeHeading(w::TaggedBspline& tag,double yaw=0.) {
  auto curve=scan_planner::decodeSpliceCurve(tag.trajectory);ASSERT_TRUE(curve);
  const auto controls=curve->getDerivative().getControlPoint();double bound=0.;
  for(int i=0;i<controls.cols();++i)bound=std::max(bound,controls.col(i).norm());
  const auto n=scan_planner::bodyHeadingSamplingIntervals(curve->getTimeSum(),bound,.05);ASSERT_TRUE(n);
  tag.body_heading_seed_valid=true;tag.body_heading_seed_yaw=yaw;tag.body_heading_sampling_intervals=*n;
}
w::TaggedBspline tag(unsigned id=1,Eigen::Vector3d p={0.,0.,.55},Eigen::Vector3d v=Eigen::Vector3d::Zero(),
    Eigen::Vector3d a={.05,0.,0.}) {
  const auto ver=version();w::TaggedBspline out;out.schema_version=2;out.session_id=ver.session_id;
  out.task_id=ver.task_id;out.route_id=ver.route_id;out.route_hash=ver.route_hash;out.map_version_id=ver.map_version_id;
  out.localization_epoch=ver.localization_epoch;out.localization_seed_id=ver.localization_seed_id;
  out.generation=ver.reference_generation;out.segment_id=ver.segment_id;out.anchor_id=ver.anchor_id;
  out.anchor_revision=ver.anchor_revision;out.context_sequence=ver.context_sequence;out.map_geometry_revision=ver.map_geometry_revision;
  out.frame_id="odom";out.point_reference="body_center";out.trajectory.order=3;out.trajectory.traj_id=id;
  out.trajectory.start_time=out.join_source_stamp=t::stamp(100.);out.join_pose.orientation.w=1.;
  out.join_pose.position.x=p.x();out.join_pose.position.y=p.y();out.join_pose.position.z=p.z();
  out.join_twist.linear.x=v.x();out.join_twist.linear.y=v.y();out.join_twist.linear.z=v.z();
  for(int i=0;i<23;++i){const double dt=(i-1)*.2;
    const Eigen::Vector3d cp=p+v*dt+.5*a*(dt*dt-.04/3.);geometry_msgs::msg::Point point;
    point.x=cp.x();point.y=cp.y();point.z=cp.z();out.trajectory.pos_pts.push_back(point);
    out.trajectory.yaw_pts.push_back(0.);}
  out.trajectory.yaw_dt=.2;for(int i=0;i<27;++i)out.trajectory.knots.push_back((i-3)*.2);
  freezeHeading(out);return out;
}
w::TaskExecutionLease lease(double now,unsigned sequence){w::TaskExecutionLease p;p.schema_version=1;
  p.version=version();p.execution_id="execution";p.control_epoch=1;p.confirmation_id="click";p.sdk_session="sdk";
  p.sdk_arm_generation=1;p.sequence=sequence;p.source_stamp=t::stamp(now);p.valid_until=t::stamp(now+.35);
  p.allowed=true;p.phase="tracking";p.transport_mode="isolated_mock";return p;}
t::Odom body(double now,double x,double vx){t::Odom o;o.schema_version=2;o.session_id="session";
  o.map_version_id="map";o.localization_epoch=1;o.localization_seed_id="seed";o.frame_id="odom";o.child_frame_id="base_link";
  o.stamp=o.posterior_stamp=o.imu_stamp=now;o.position={x,0.,.55};o.velocity_in_frame={vx,0.,0.};o.planar_speed=std::abs(vx);
  o.source_stamp_ns=t::timeNs(t::stamp(now));o.provider_contract=o.provider_usable=true;
  o.provider_valid_until_ns=o.source_stamp_ns+100000000LL;o.provider_posterior_stamp_ns=o.provider_imu_stamp_ns=o.source_stamp_ns;return o;}
struct Integrated {
  scan_planner::CollisionSnapshotPool pool;Native native;
  const bool official;
  t::LocalFollower follower;GridMap map;w::ReferenceProposal reference=proposal();
  std::optional<w::LocalTrajectoryCommit> initial_commit;
  scan_planner::SplineHeadingContract heading;scan_planner::BrakingModel braking;
  scan_planner::NativePipelineHealth pipeline_health;
  explicit Integrated(bool official_policy=true,bool omit_commit_body=false,
      std::optional<w::ReferenceProposal> typed_reference={},std::optional<w::TaggedBspline> initial_curve={},
      bool defer_initial_commit=false,std::optional<w::SupportReference> original_support={}):
      native(Native::PassiveLedger{},pool,"odom","isolated_mock",false,false,true),
      official(official_policy),follower(config(official),"isolated_mock",std::string(64,'d'),false),
      reference(typed_reference.value_or(proposal())),
      braking{.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'d')} {
    braking.kinematic_profile=scan_planner::KinematicProfile::HolonomicXYYaw;braking.lateral_distance=.08;
    braking.command_pipeline_bound=.1;Access::braking(native,braking);Access::geometryPolicy(native,official);
    GridMapTestAccess::configure(map,official);
    auto context=std::make_shared<scan_planner::ProjectedRayContext>();
    context->session=reference.version.session_id;context->seed=reference.version.localization_seed_id;
    context->epoch=reference.version.localization_epoch;context->sequence=reference.version.context_sequence;
    context->frame="odom";pipeline_health.context=context;
    Access::pipelineHealth(native,[this]{return pipeline_health;});
    heading.measured_heading_enabled=true;heading.policy=scan_planner::BodyHeadingPolicy::HoldHeading;
    heading.measured_yaw=0.;heading.body_extent=.53;
    EXPECT_TRUE(native.replacePendingReference(reference));EXPECT_TRUE(follower.proposal(reference,100.));
    auto s=original_support.value_or(support());native.support(s);EXPECT_TRUE(follower.support(s));const auto curve=initial_curve.value_or(tag());
    acquire(100.,curve.join_pose.position.x,curve.join_twist.linear.x);
    native.candidate(curve,reference,heading);EXPECT_TRUE(follower.candidate(curve));
    // Match LocalFollower's formal Native authority instead of accidentally
    // invoking the historical standalone tracker's raw-support classifier.
    auto preparation_config=config(official);preparation_config.geometry_authority=t::GeometryAuthority::NativeScan;
    const auto prepared=t::PreparedGeometry::build(preparation_config,t::decodeLocalCurve(curve),t::supportEvidence(s));
    EXPECT_TRUE(prepared&&prepared->failure.empty())<<(prepared?prepared->failure:"null");
    auto proof=Access::checkCurve(native,map,100000000000LL);EXPECT_TRUE(proof&&proof->valid);
    if(proof){EXPECT_TRUE(follower.validation(*proof));}
    const double commit_at=omit_commit_body?100.01:100.;
    if(omit_commit_body)EXPECT_TRUE(follower.body(body(commit_at,0.,0.),commit_at,10.01));
    auto c=follower.tryCommit(commit_at,commit_at-90.);EXPECT_TRUE(c.has_value())<<follower.candidateReason();
    if(c){initial_commit=c;if(!defer_initial_commit)EXPECT_TRUE(native.localCommitAt(*c,t::timeNs(t::stamp(commit_at))));}
  }
  void acquireBody(double now,double x,double vx) {
    const auto o=body(now,x,vx);EXPECT_TRUE(follower.body(o,now,now-90.));
    geometry_msgs::msg::Twist twist;twist.linear.x=vx;
    native.body({o.position,o.orientation,o.stamp,"odom"},twist,o.source_stamp_ns);
  }
  void acquire(double now,double x,double vx) {
    acquireBody(now,x,vx);const auto o=body(now,x,vx);
    GridMapTestAccess::acquire(map,o.source_stamp_ns);
    pipeline_health.source_ns={o.source_stamp_ns,o.source_stamp_ns};
    pipeline_health.fusion_sequence=map.fusionTiming().sequence;
    pipeline_health.sampled_steady_ns=pipeline_health.fusion_completed_steady_ns=o.source_stamp_ns;
  }
  w::TrackingProgress progress(double now) {
    auto& core=follower.controller();core.refreshMeasuredProgress(now);
    return t::trackingProgress(core,config(official),now,follower.status(now));
  }
  auto command(double now,double x,double vx,unsigned sequence,bool expect_clear=true,
      bool acquire_body=true,bool expect_proof=true) {
    if(acquire_body)acquire(now,x,vx);
    const auto ns=t::timeNs(t::stamp(now));
    native.progressAt(progress(now),now);
    auto proof=Access::checkCurve(native,map,ns,true);
    EXPECT_EQ(proof.has_value(),expect_proof);if(proof){EXPECT_EQ(proof->valid,expect_clear)<<proof->reason;}
    if(proof){EXPECT_TRUE(follower.validation(*proof));}
    const auto grant=lease(now,sequence);EXPECT_TRUE(follower.lease(grant,now));EXPECT_TRUE(native.taskLeaseAt(grant,ns));
    const auto permit=follower.geometryPermit(now);EXPECT_TRUE(native.commit(permit));
    const auto d=follower.step(now,now-90.);
    std::optional<w::LocalMotionCommand> out;
    native.demandAt(d,ns,[&](const auto& command){out=command;});
    native.progressAt(progress(now),now);
    return out;
  }
  bool prepareTail(double now,unsigned id=2,std::size_t control_points=23) {
    const auto splice=native.plannedSpliceAt(version(),t::timeNs(t::stamp(now)),.05,.8);
    if(!splice)return false;
    const auto& s=splice->wire;
    auto tail=tag(id,{s.position.x,s.position.y,s.position.z},
      {s.velocity.x,s.velocity.y,s.velocity.z},{s.acceleration.x,s.acceleration.y,s.acceleration.z});
    tail.trajectory.pos_pts.resize(control_points);tail.trajectory.yaw_pts.resize(control_points);
    tail.trajectory.knots.resize(control_points+4);
    freezeHeading(tail);
    tail.trajectory.start_time=t::stamp(now); // The real manager stamps publication, not the future seam.
    tail.splice=s;tail.join_source_stamp=builtin_interfaces::msg::Time();
    tail.join_pose=geometry_msgs::msg::Pose();tail.join_twist=geometry_msgs::msg::Twist();
    // Native and follower see the same immutable proposal/tail. This is a
    // planned boundary; no acquired body sample is moved into the future.
    native.candidate(tail,reference,heading);return follower.candidate(tail);
  }
  bool commitTailIfReady(double now) {
    const auto ns=t::timeNs(t::stamp(now));
    auto proof=Access::checkCurve(native,map,ns);
    if(!proof||!follower.validation(*proof))return false;
    const auto c=follower.tryCommit(now,now-90.);
    if(!c)return false;
    EXPECT_EQ(c->expected_trajectory_id,1);EXPECT_EQ(c->trajectory_id,2);
    EXPECT_TRUE(native.localCommitAt(*c,ns));return true;
  }
};
}
