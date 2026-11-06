#include <gtest/gtest.h>
#include <string>
#include <future>
#include <plan_manage/execution_validator.hpp>
namespace scan_planner {
struct ExecutionValidatorTestAccess {
  static void dispatchDisplay(ExecutionValidator& v,const std::function<void(const ew::TaggedBspline&)>& emit) {
    v.dispatchCommittedDisplay(emit);
  }
  static std::size_t pendingDisplays(ExecutionValidator& v) {
    std::lock_guard<std::mutex> lock(v.mutex_);return v.pending_committed_display_?1:0;
  }
  static bool consumeAdmissionWake(ExecutionValidator& v) {
    std::unique_lock<std::mutex> l(v.admission_wait_mutex_);
    const bool ready=v.admission_wake_.wait_for(l,std::chrono::milliseconds(2),
      [&]{return v.stop_.load()||v.admission_ready_;});
    v.admission_ready_=false;return ready;
  }
  static bool matches(const ew::MotionDemand& d,const ew::ExecutionPermit& p){return ExecutionValidator::demandPermitMatches(d,p);}
  static bool matches(const ew::MotionDemand& d,const ew::ExecutionPermit& p,std::uint64_t sequence){return ExecutionValidator::demandPermitMatches(d,p,sequence);}
  static auto proof(const std::deque<ew::TrajectoryValidation>& history,const ew::MotionDemand& d,std::int64_t now){return ExecutionValidator::motionProof(history,d,now);}
  static auto barrier(const ExecutionValidator& v){return v.motion_stop_barrier_;}
  static const auto& latest(const ExecutionValidator& v){return v.latest_permit_;}
  static void record(ExecutionValidator& v,const ew::MotionValidation& p){v.motion_proofs_.push_back(p);}
  static void braking(ExecutionValidator& v,const BrakingModel& model){v.braking_=model;}
  // The prepared validation lane records both transaction-scoped evidence
  // and bounded published history. A generic current-motion proof alone is
  // intentionally insufficient for a writer's replacement ACK.
  static void recordPrepared(ExecutionValidator& v,const ew::MotionValidation& p){
    v.prepared_motion_proofs_.push_back(p);v.motion_proofs_.push_back(p);}
  static void recordWhole(ExecutionValidator& v,const ew::TrajectoryValidation& p){v.proofs_.push_back(p);}
  static void recordLocalFact(ExecutionValidator& v,const ew::TrajectoryValidation& p) {
    v.local_proof_facts_.push_back(p);if(v.candidate_)v.candidate_->proof=p;
  }
  static void pipelineHealth(ExecutionValidator& v,std::function<NativePipelineHealth()> provider) {
    v.pipeline_health_=std::move(provider);
  }
  static void completedPipelineCheck(ExecutionValidator& v,const ew::TrajectoryValidation& p,
      std::uint64_t fusion) {v.completedPipelineCheckLocked(p,fusion);}
  static bool pipelineReady(ExecutionValidator& v,const ew::ExecutionVersion& version,std::int64_t now) {
    return v.nativePipelineReadyLocked(version,now);
  }
  static auto localCommand(const ExecutionValidator& v,const ew::MotionDemand& d,std::int64_t deadline,std::int64_t now) {
    return v.localCommandLocked(d,deadline,now);
  }
  static const auto& latestDemand(const ExecutionValidator& v){return v.latest_demand_;}
  static auto taskLeaseHistorySize(const ExecutionValidator& v){return v.task_lease_history_.size();}
  static auto bodySource(const ExecutionValidator& v){return v.body_.source_stamp;}
  static auto bodyHistorySize(const ExecutionValidator& v){return v.body_history_.size();}
  // Explicit evidence ingress seam: the production reader has completed its
  // native query/source/order checks before this exact committed-proof store.
  static bool recordCommitted(ExecutionValidator& v,const ew::TrajectoryValidation& p){
    return v.recordCommittedProofLocked(p);}
  static void pendingSchedulingEvent(ExecutionValidator& v,unsigned kind) {
    if(kind==0)v.blocked_entry_=ew::MotionValidation{};
    if(kind==1)v.entry_reseed_=ew::TrajectoryAdmission{};
    if(kind==2){v.grant_=ew::ExecutionHandoffGrant{};v.grant_applied_=v.grant_retired_=false;}
    if(kind==3){v.tombstone_grant_=ew::ExecutionHandoffGrant{};v.tombstone_retired_=false;}
  }
  static void previewProof(ExecutionValidator& v,const ew::TrajectoryValidation& p){
    if(v.candidate_)v.candidate_->proof=p;
    if(v.active_)v.active_->proof=p;}
  static void previewSeed(ExecutionValidator& v,const NoMotionPreviewProgress& p){
    if(v.candidate_){v.candidate_->preview_progress=p;v.candidate_->preview_projection_resolution=.05;}
    if(v.active_){v.active_->preview_progress=p;v.active_->preview_projection_resolution=.05;}}
  static auto join(const ExecutionValidator& v,UniformBspline& curve,const MeasuredBodyPose& body,
      bool active=true,ValidationCycle::Clock::time_point deadline=ValidationCycle::Clock::time_point::max()) {
    return v.measuredJoin(active?*v.active_:*v.candidate_,curve,body,.05,body.source_stamp,deadline);}
  static bool projectionInconclusive(const ExecutionValidator::MeasuredJoin& connection) {
    return ExecutionValidator::previewCandidateProjectionInconclusive(connection);}
  static auto previewSeed(const ExecutionValidator& v){return v.active_->preview_progress;}
  static auto observedPreviewSeed(const ExecutionValidator& v){return v.active_->observed_preview_progress;}
  static auto readSnapshot(const ExecutionValidator& v){return *v.active_;}
  static auto candidateReadSnapshot(const ExecutionValidator& v){return *v.candidate_;}
  static void candidateMeasuredTime(ExecutionValidator& v,double time){v.candidate_->measured_time=time;}
  static void clearInjectedCandidateProof(ExecutionValidator& v){v.candidate_->proof.reset();v.local_proof_facts_.clear();}
  static bool hasCandidate(const ExecutionValidator& v){return v.candidate_.has_value();}
  static auto candidateOrigin(const ExecutionValidator& v){return v.candidate_->preview_candidate_origin;}
  static std::size_t supportHistorySize(const ExecutionValidator&v){return v.support_history_.size();}
  static void changedSupportReadRace(ExecutionValidator&v,const ew::SupportReference&s){v.active_->support=s;}
  static void changedCandidateSupportReadRace(ExecutionValidator&v,const ew::SupportReference&s){v.candidate_->support=s;}
  static auto candidateSeed(const ExecutionValidator& v){return v.candidate_->preview_progress;}
  static bool endReadCurrent(const ExecutionValidator& v,const ExecutionValidator::Job& original,
      const ew::ExecutionPermit& permit,std::int64_t source,std::int64_t now,bool needs_replan=false) {
    return v.previewReadStillCurrentLocked(original,permit,source,now,now,needs_replan);
  }
  static bool positiveSeedCurrent(const ExecutionValidator& v,const ExecutionValidator::Job& original) {
    return v.positivePreviewCheckStillCurrentLocked(original);
  }
  static bool supersedes(const ew::TrajectoryValidation& a,const ew::TrajectoryValidation& b){return ExecutionValidator::proofSupersedes(a,b);}
  static void officialGeometry(ExecutionValidator& v){v.geometry_policy_="official_inflated_geometry";}
  static auto checkWithClock(ExecutionValidator& v,GridMap& map,
      const std::function<std::int64_t()>& clock,bool active=false) {
    std::optional<ew::TrajectoryValidation> out;
    const auto job=active?v.active_:v.candidate_;
    if(job)v.checkAt(*job,v.body_,map,clock,[&](const auto& p){out=p;},.12);
    return out;
  }
  static auto preparedCurve(const ExecutionValidator& v){return v.prepared_?v.prepared_->spline.trajectory.traj_id:-1;}
  static auto writerSequence(const ExecutionValidator& v){return v.writer_commit_sequence_;}
  static const auto& writerDomain(const ExecutionValidator& v){return v.writer_domain_;}
  static bool activeProof(const ExecutionValidator& v){return v.active_&&v.active_->proof.has_value();}
  static bool activeProgress(const ExecutionValidator& v){return v.active_&&v.active_->progress.has_value();}
  static SplineHeadingContract candidateHeading(const ExecutionValidator& v){return v.candidate_->heading;}
  static SplineHeadingContract activeHeading(const ExecutionValidator& v){return v.active_->heading;}
  static bool entry(const ew::PreparedMotionDemand& x,const ew::ExecutionHandoffGrant& g,std::int64_t now,
      std::int64_t command_pipeline_ns=100000000LL){
    return ExecutionValidator::preparedEntryMatches(x,g,"odom",now,command_pipeline_ns);}
  static bool stationaryMode(ExecutionValidator& v,const ew::ExecutionHandoffGrant& g,std::int64_t now) {
    v.latest_permit_=g.incumbent;return v.handoffModeValid(g,now);}
};
}
// Native GridMap with deterministic inputs. This does not implement another
// collision map or clear any real occupancy; the production inflated query
// is used below, including occupied rotation sweeps.
struct GridMapTestAccess {
  static GridMap::Ptr headingFixture(bool strict=true,bool box=false) {
    auto map=std::make_shared<GridMap>();auto& p=map->mp_;auto& d=map->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={64,64,32};
    p.map_origin_idx_.setZero();
    if(box)p.map_origin_idx_=p.map_voxel_num_/2;
    map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.require_observed_free_=strict;p.planning_collision_policy_="official";
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=.45;p.obstacles_inflation_z_down=.45;
    if(box) {p.collision_model_="oriented_box";p.box_length_=.93;p.box_width_=.30;
      p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;}
    d.occupancy_buffer_.assign(64*64*32,-1.01);
    d.occupancy_buffer_inflate_.assign(64*64*32,0);
    d.occupancy_buffer_inflate_cnt_.assign(64*64*32,0);
    map->rebuildInflationOffsets();return map;
  }
  static const auto& raw(const GridMap& map){return map.md_.occupancy_buffer_;}
  static void acquiredSnapshot(GridMap& map,std::int64_t source,std::int64_t snapshot_now) {
    map.mp_.use_projected_rays_=true;map.mp_.cloud_pose_max_age_=.5;map.mp_.frame_id_="odom";
    map.collision_snapshot_=true;map.ray_clock_fault_=false;
    map.localization_context_sequence_=map.localization_epoch_=1;
    map.ray_integrated_stamps_={source,source};map.integrated_cloud_stamp_ns_=source;
    map.snapshot_clock_ns_=snapshot_now;map.snapshot_captured_=std::chrono::steady_clock::now();
  }
};
namespace {
namespace w=scan_planner::ew;
struct Fixture {
  scan_planner::CollisionSnapshotPool pool;
  scan_planner::ExecutionValidator ledger;
  w::ReferenceProposal proposal;
  w::TaggedBspline spline;
  explicit Fixture(bool planning_only=false,bool local_follower=false):ledger(scan_planner::ExecutionValidator::PassiveLedger{},
      pool,"odom","isolated_mock",false,planning_only,local_follower) {
    auto& v=proposal.version;v.schema_version=3;v.session_id="session";v.task_id="task";v.route_id="route";
    v.route_hash="hash";v.map_version_id="map";v.localization_epoch=1;v.localization_seed_id="seed";
    v.reference_generation=1;v.anchor_id="anchor";v.anchor_revision=1;v.context_sequence=1;v.segment_id="floor1";
    proposal.proposal_id="proposal";spline.generation=1;spline.trajectory.traj_id=1;
  }
  w::ExecutionPermit permit(std::uint64_t sequence=1)const {
    w::ExecutionPermit p;p.version=proposal.version;p.geometry_committed=true;
    p.trajectory_id=spline.trajectory.traj_id;p.sequence=sequence;p.transport_mode="isolated_mock";
    // A typed owner ingress establishes the task only with its real source
    // lease. PassiveLedger injects this clock rather than manufacturing now.
    p.source_stamp=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10500000000LL);
    p.frame_id="odom";p.phase="holding";return p;
  }
  w::TrackingProgress progress()const {
    const auto& v=proposal.version;w::TrackingProgress p;p.schema_version=2;p.valid=true;p.holding=true;
    p.header.stamp=rclcpp::Time(10000000000LL);p.header.frame_id="odom";
    p.session_id=v.session_id;p.task_id=v.task_id;p.route_id=v.route_id;p.route_hash=v.route_hash;
    p.map_version_id=v.map_version_id;p.localization_epoch=v.localization_epoch;p.localization_seed_id=v.localization_seed_id;
    p.generation=v.reference_generation;p.anchor_id=v.anchor_id;p.anchor_revision=v.anchor_revision;
    p.context_sequence=v.context_sequence;p.segment_id=v.segment_id;p.trajectory_id=spline.trajectory.traj_id;
    p.pose.orientation.w=1.;return p;
  }
};
// Explicit proof-injection seam tests only the real preview projection and
// contract gates. The PassiveLedger cannot produce collision certificates.
struct PlanningBodyFixture : Fixture {
  static constexpr std::int64_t now=10000000000LL;
  explicit PlanningBodyFixture():Fixture(false,true) {
    proposal.source_stamp=rclcpp::Time(now);
    if(!ledger.replacePendingReference(proposal))throw std::runtime_error("planning_body_task_failed");
  }
  void sample(std::int64_t source,const Eigen::Vector3d& position={1.,2.,.5},
      const Eigen::Vector3d& velocity={.2,.01,0.},bool with_twist=true) {
    scan_planner::MeasuredBodyPose p{position,Eigen::Quaterniond(Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ())),
      double(source)*1e-9,"odom"};
    geometry_msgs::msg::Twist t;t.linear.x=velocity.x();t.linear.y=velocity.y();t.linear.z=velocity.z();
    t.angular.z=.12;
    ledger.bodyAt(p,with_twist?std::optional<geometry_msgs::msg::Twist>(t):std::nullopt,source,now);
  }
};
TEST(PlanningBodyInbox, FreshOriginalPoseAndTwistNeedNoCurveProofOrMotionPermit) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  PlanningBodyFixture f;f.sample(f.now-20000000LL);
  const auto barrier=A::barrier(f.ledger);
  const auto sample=f.ledger.latestPlanningBodyAt(f.proposal.version,f.now,.4);
  ASSERT_TRUE(sample);ASSERT_TRUE(sample->twist);
  EXPECT_EQ(sample->source_ns,f.now-20000000LL);
  EXPECT_DOUBLE_EQ(sample->pose.source_stamp,double(f.now-20000000LL)*1e-9);
  EXPECT_EQ(sample->pose.position,Eigen::Vector3d(1.,2.,.5));
  EXPECT_NEAR(Eigen::AngleAxisd(sample->pose.orientation).angle(),.4,1e-12);
  EXPECT_DOUBLE_EQ(sample->twist->linear.x,.2);EXPECT_DOUBLE_EQ(sample->twist->angular.z,.12);
  EXPECT_EQ(A::barrier(f.ledger),barrier);EXPECT_FALSE(A::latest(f.ledger));
  EXPECT_FALSE(f.ledger.committedView());EXPECT_FALSE(A::hasCandidate(f.ledger));
}
TEST(PlanningBodyInbox, DuplicateAndOutOfOrderDeliveryDoNotRenewOrReplaceOriginalSample) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  PlanningBodyFixture f;f.sample(f.now);
  f.sample(f.now,{9.,9.,9.},{9.,9.,9.});
  f.sample(f.now-20000000LL,{-1.,-1.,.5},{-.2,0.,0.});
  const auto sample=f.ledger.latestPlanningBodyAt(f.proposal.version,f.now+300000000LL,.4);
  ASSERT_TRUE(sample);EXPECT_EQ(sample->source_ns,f.now);
  EXPECT_EQ(sample->pose.position,Eigen::Vector3d(1.,2.,.5));
  EXPECT_DOUBLE_EQ(sample->twist->linear.x,.2);EXPECT_EQ(A::bodyHistorySize(f.ledger),2u);
  EXPECT_FALSE(f.ledger.latestPlanningBodyAt(f.proposal.version,f.now+400000001LL,.4));
  EXPECT_EQ(A::bodyHistorySize(f.ledger),2u);
}
TEST(PlanningBodyInbox, LatestPoseAndTwistAreCopiedTogetherWithoutInterpolatingOlderState) {
  PlanningBodyFixture f;f.sample(f.now-20000000LL,{1.,2.,.5},{.1,.2,.3});
  f.sample(f.now,{2.,3.,.6},{.4,.5,.6});
  const auto sample=f.ledger.latestPlanningBodyAt(f.proposal.version,f.now,.4);
  ASSERT_TRUE(sample);EXPECT_EQ(sample->pose.position,Eigen::Vector3d(2.,3.,.6));
  EXPECT_DOUBLE_EQ(sample->twist->linear.x,.4);EXPECT_DOUBLE_EQ(sample->twist->linear.y,.5);
  EXPECT_DOUBLE_EQ(sample->twist->linear.z,.6);EXPECT_EQ(sample->source_ns,f.now);
  // A newer incomplete real sample cannot borrow the preceding sample's twist.
  f.sample(f.now+10000000LL,{3.,4.,.7},{},false);
  EXPECT_FALSE(f.ledger.latestPlanningBodyAt(f.proposal.version,f.now+10000000LL,.4));
}
TEST(PlanningBodyInbox, ForeignTaskEpochOrCancelledTaskCannotReadHistoricalBody) {
  PlanningBodyFixture f;f.sample(f.now);
  auto wrong=f.proposal.version;wrong.task_id="other";
  EXPECT_FALSE(f.ledger.latestPlanningBodyAt(wrong,f.now,.4));
  wrong=f.proposal.version;++wrong.localization_epoch;
  EXPECT_FALSE(f.ledger.latestPlanningBodyAt(wrong,f.now,.4));
  auto cancel=f.proposal.version;++cancel.reference_generation;
  ASSERT_TRUE(f.ledger.cancelReference(cancel));
  EXPECT_FALSE(f.ledger.latestPlanningBodyAt(f.proposal.version,f.now,.4));
  // A genuinely new target in the same localization domain may use a fresh
  // sample acquired before its submission; no new task-time source gate.
  auto next=f.proposal;next.version.task_id="next";next.version.route_id="next-route";
  next.source_stamp=rclcpp::Time(f.now+20000000LL);
  ASSERT_TRUE(f.ledger.replacePendingReference(next));
  EXPECT_TRUE(f.ledger.latestPlanningBodyAt(next.version,f.now+20000000LL,.4));
  EXPECT_FALSE(f.ledger.latestPlanningBodyAt(f.proposal.version,f.now+20000000LL,.4));
}
TEST(PlanningBodyInbox, InvalidAgeMissingTwistFutureAndMismatchedOriginalStampAreRejected) {
  PlanningBodyFixture f;f.sample(f.now,{}, {},false);
  EXPECT_FALSE(f.ledger.latestPlanningBodyAt(f.proposal.version,f.now,.4));
  f.sample(f.now); // The same source can fill its missing twist, not renew it.
  EXPECT_TRUE(f.ledger.latestPlanningBodyAt(f.proposal.version,f.now,.4));
  EXPECT_FALSE(f.ledger.latestPlanningBodyAt(f.proposal.version,f.now,0.));
  EXPECT_FALSE(f.ledger.latestPlanningBodyAt(f.proposal.version,f.now,-.1));
  EXPECT_FALSE(f.ledger.latestPlanningBodyAt(f.proposal.version,f.now,std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(f.ledger.latestPlanningBodyAt(f.proposal.version,f.now-20000001LL,.4));
  geometry_msgs::msg::Twist t;t.linear.x=.2;
  scan_planner::MeasuredBodyPose p{{0.,0.,.5},Eigen::Quaterniond::Identity(),10.01,"odom"};
  f.ledger.bodyAt(p,t,f.now+20000000LL,f.now+20000000LL);
  EXPECT_FALSE(f.ledger.latestPlanningBodyAt(f.proposal.version,f.now+20000000LL,.4));
}
TEST(PlanningBodyInbox, InvalidFrameOrVelocityIngressDoesNotPoisonLatestValidSample) {
  PlanningBodyFixture f;f.sample(f.now);
  geometry_msgs::msg::Twist t;t.linear.x=.2;
  scan_planner::MeasuredBodyPose p{{9.,9.,.5},Eigen::Quaterniond::Identity(),10.01,"foreign"};
  f.ledger.bodyAt(p,t,f.now+10000000LL,f.now+10000000LL);
  p.frame="odom";t.angular.z=std::numeric_limits<double>::quiet_NaN();
  f.ledger.bodyAt(p,t,f.now+10000000LL,f.now+10000000LL);
  const auto sample=f.ledger.latestPlanningBodyAt(f.proposal.version,f.now+10000000LL,.4);
  ASSERT_TRUE(sample);EXPECT_EQ(sample->source_ns,f.now);
  EXPECT_EQ(sample->pose.position,Eigen::Vector3d(1.,2.,.5));
}
TEST(ExecutionAdmissionWake, AcceptedCandidateWakesExistingLaneWithoutNewMapOrProof) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  Fixture f;
  EXPECT_FALSE(A::consumeAdmissionWake(f.ledger));
  f.spline.join_source_stamp=rclcpp::Time(9990000000LL);
  f.proposal.source_stamp=rclcpp::Time(9990000000LL);
  f.ledger.candidate(f.spline,f.proposal);
  ASSERT_TRUE(A::hasCandidate(f.ledger));
  const auto before=A::candidateReadSnapshot(f.ledger);
  EXPECT_TRUE(A::consumeAdmissionWake(f.ledger));
  EXPECT_FALSE(A::consumeAdmissionWake(f.ledger));
  const auto after=A::candidateReadSnapshot(f.ledger);
  EXPECT_EQ(before.spline,after.spline);
  EXPECT_EQ(after.spline.join_source_stamp,f.spline.join_source_stamp);
  EXPECT_FALSE(after.proof);
  EXPECT_FALSE(f.ledger.committedView());
}
TEST(ExecutionAdmissionWake, MapAndCandidateWakeCoalesceRatherThanQueueWork) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  Fixture f;
  f.ledger.admissionSnapshotPublished();
  f.ledger.candidate(f.spline,f.proposal);
  ++f.spline.trajectory.traj_id;
  f.ledger.candidate(f.spline,f.proposal);
  EXPECT_TRUE(A::consumeAdmissionWake(f.ledger));
  EXPECT_FALSE(A::consumeAdmissionWake(f.ledger));
  EXPECT_EQ(A::candidateReadSnapshot(f.ledger).spline.trajectory.traj_id,2);
}
TEST(ExecutionAdmissionWake, RevokedTaskCannotWakeOrRecreateCandidateWork) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  Fixture f;f.ledger.candidate(f.spline,f.proposal);
  ASSERT_TRUE(f.ledger.commit(f.permit()));
  A::consumeAdmissionWake(f.ledger);
  auto stop=f.permit(2);stop.revoked=true;
  ASSERT_TRUE(f.ledger.commit(stop));
  f.ledger.candidate(f.spline,f.proposal);
  EXPECT_FALSE(A::hasCandidate(f.ledger));
  EXPECT_FALSE(A::consumeAdmissionWake(f.ledger));
  EXPECT_FALSE(f.ledger.committedView());
}
struct PlanningOnlyFixture : Fixture {
  static constexpr std::int64_t now=10000000000LL;
  w::TrajectoryValidation proof;
  scan_planner::MeasuredBodyPose body;
  scan_planner::UniformBspline curve;
  explicit PlanningOnlyFixture(const Eigen::Vector3d& offset=Eigen::Vector3d::Zero()):Fixture(true) {
    Eigen::MatrixXd points(3,23);
    for(int i=0;i<points.cols();++i)points.col(i)=offset+Eigen::Vector3d((i-1)*.05,0.,.55);
    curve=scan_planner::UniformBspline(points,3,.2);
    spline.frame_id="odom";spline.trajectory.order=3;
    for(int i=0;i<points.cols();++i) {
      geometry_msgs::msg::Point p;p.x=points(0,i);p.y=points(1,i);p.z=points(2,i);
      spline.trajectory.pos_pts.push_back(p);
    }
    const auto knots=curve.getKnot();
    for(int i=0;i<knots.size();++i)spline.trajectory.knots.push_back(knots[i]);
    w::SupportReference support;support.version=proposal.version;support.verified=true;
    support.support_reference_id="support";support.support_hash="support_hash";
    ledger.support(support);ledger.candidate(spline,proposal);
    proof.version=proposal.version;proof.proposal_id=proposal.proposal_id;
    proof.trajectory_id=spline.trajectory.traj_id;proof.sequence=1;proof.valid=true;proof.whole_curve=true;
    proof.frame_id="odom";proof.transport_mode="isolated_mock";proof.collision_policy="observed_free";
    proof.support_reference_id=support.support_reference_id;proof.support_hash=support.support_hash;
    proof.body_source_stamp=rclcpp::Time(now);proof.check_end=rclcpp::Time(now);
    proof.valid_until=rclcpp::Time(now+250000000LL);
    scan_planner::ExecutionValidatorTestAccess::previewProof(ledger,proof);
    body={curve.evaluateDeBoorT(0.),Eigen::Quaterniond::Identity(),now*1e-9,"odom"};
    scan_planner::ExecutionValidatorTestAccess::previewSeed(ledger,
      {scan_planner::NoMotionPreviewStatus::Connected,0.,0.,0.,0.,body.source_stamp,body.position});
    auto p=permit();p.phase="preview";if(!ledger.commit(p))throw std::runtime_error("preview_commit_fixture_failed");
  }
  std::optional<double> project(std::int64_t at=now,std::int64_t source=now)const {
    return ledger.planningOnlyCurveTime(proposal.version,spline.trajectory.traj_id,
      proposal.version.context_sequence,body,source,at,.08);
  }
};
struct CommittedObstacleFixture : Fixture {
  w::TrajectoryValidation proof;
  CommittedObstacleFixture() {
    w::SupportReference s;s.version=proposal.version;s.verified=true;
    s.support_reference_id="support";s.support_hash="support_hash";
    ledger.support(s);ledger.candidate(spline,proposal);
    if(!ledger.commit(permit()))throw std::runtime_error("committed_obstacle_fixture_failed");
    proof.version=proposal.version;proof.proposal_id=proposal.proposal_id;
    proof.trajectory_id=1;proof.sequence=1;proof.valid=false;proof.whole_curve=true;
    proof.frame_id="odom";proof.transport_mode="isolated_mock";proof.collision_policy="observed_free";
    proof.support_reference_id=s.support_reference_id;proof.support_hash=s.support_hash;
    proof.reason="occupied";proof.check_end=rclcpp::Time(10000000000LL);
    proof.valid_until=rclcpp::Time(10250000000LL);
  }
};
// Passive evidence ingress for the real one-lock scheduling predicate. It
// cannot issue collision certificates or authorize motion. Cubic geometry,
// body ingress, progress admission and permit commitment are production code.
struct CommittedIdleFixture : Fixture {
  static constexpr std::int64_t now=10100000000LL;
  scan_planner::UniformBspline curve;
  scan_planner::MeasuredBodyPose body;
  w::TrajectoryValidation proof;
  w::TrackingProgress measured;
  CommittedIdleFixture() {
    Eigen::MatrixXd points(3,23);
    for(int i=0;i<points.cols();++i)points.col(i)=Eigen::Vector3d((i-1)*.05,0.,.55);
    curve=scan_planner::UniformBspline(points,3,.2);
    spline.frame_id="odom";spline.trajectory.order=3;
    for(int i=0;i<points.cols();++i) {
      geometry_msgs::msg::Point p;p.x=points(0,i);p.y=points(1,i);p.z=points(2,i);
      spline.trajectory.pos_pts.push_back(p);
    }
    const auto knots=curve.getKnot();
    for(int i=0;i<knots.size();++i)spline.trajectory.knots.push_back(knots[i]);
    w::SupportReference support;support.version=proposal.version;support.verified=true;
    support.support_reference_id="support";support.support_hash="support_hash";
    ledger.support(support);ledger.candidate(spline,proposal);
    if(!ledger.commit(permit()))throw std::runtime_error("committed_idle_fixture_failed");
    body={curve.evaluateDeBoorT(.1),Eigen::Quaterniond::Identity(),now*1e-9,"odom"};
    ledger.body(body,geometry_msgs::msg::Twist{},now);
    measured=progress();measured.holding=false;measured.curve_time=.1;
    measured.header.stamp=rclcpp::Time(now);measured.pose.position.x=body.position.x();
    measured.pose.position.y=body.position.y();measured.pose.position.z=body.position.z();
    ledger.progressAt(measured,now*1e-9);
    proof.version=proposal.version;proof.proposal_id=proposal.proposal_id;
    proof.trajectory_id=1;proof.sequence=1;proof.valid=true;proof.whole_curve=true;
    proof.frame_id="odom";proof.transport_mode="isolated_mock";proof.collision_policy="observed_free";
    proof.support_reference_id=support.support_reference_id;proof.support_hash=support.support_hash;
    proof.body_source_stamp=rclcpp::Time(now);proof.check_end=rclcpp::Time(now);
    proof.front_ray_source_stamp=proof.rear_ray_source_stamp=rclcpp::Time(now);
    proof.valid_until=rclcpp::Time(now+250000000LL);
    if(!scan_planner::ExecutionValidatorTestAccess::recordCommitted(ledger,proof))
      throw std::runtime_error("committed_idle_proof_failed");
  }
  bool keep(std::int64_t at=now)const {
    return ledger.keepCommittedIdleHorizonAt(proposal.version,at,.4,1.);
  }
};
struct PreviewCandidateFixture : Fixture {
  scan_planner::UniformBspline curve;
  scan_planner::MeasuredBodyPose body;
  scan_planner::PreviewCandidateGeometryEvidence geometry;
  w::SupportReference support;
  explicit PreviewCandidateFixture():Fixture(true) {
    using namespace scan_planner;
    Eigen::MatrixXd points(3,23);
    for(int i=0;i<points.cols();++i)points.col(i)=Eigen::Vector3d(.8+(i-1)*.05,1.,.55);
    curve=UniformBspline(points,3,.2);
    spline.frame_id="odom";spline.trajectory.order=3;
    for(int i=0;i<points.cols();++i) {
      geometry_msgs::msg::Point p;p.x=points(0,i);p.y=points(1,i);p.z=points(2,i);
      spline.trajectory.pos_pts.push_back(p);
    }
    const auto knots=curve.getKnot();
    for(int i=0;i<knots.size();++i)spline.trajectory.knots.push_back(knots[i]);
    MeasuredBodyPose origin{curve.evaluateDeBoorT(0.),Eigen::Quaterniond::Identity(),10.,"odom"};
    body={origin.position+Eigen::Vector3d(.075,.10,0.),Eigen::Quaterniond::Identity(),10.3,"odom"};
    PreviewCandidateOrigin source{origin,{.25,0.,0.},.3,.6,true,{}};
    const auto result=projectPreviewCandidateGeometry(curve,source,body,.05,std::make_shared<SolveBudget>());
    if(!result.geometry)throw std::runtime_error("preview_candidate_fixture_projection_failed");
    geometry=*result.geometry;
    support.version=proposal.version;support.verified=true;support.support_reference_id="support";
    support.floor_id="floor1";support.segment_kind="floor";support.required_mode="general";support.frame_id="odom";
    support.support_hash=std::string(64,'a');support.support_map_sha256=std::string(64,'b');
    support.support_xy_radius_m=.15;support.body_reference_height_m=.55;
    support.max_support_slope_rad=.1;support.max_support_step_m=.05;
    for(int i=0;i<25;++i){geometry_msgs::msg::Point p;p.x=.7+i*.05;p.y=1.;support.support_ground_xyz.push_back(p);}
    ledger.support(support);ledger.candidate(spline,proposal,{},geometry);ledger.body(body);
  }
  w::TrajectoryValidation proof()const {
    w::TrajectoryValidation p;p.version=proposal.version;p.proposal_id=proposal.proposal_id;
    p.trajectory_id=spline.trajectory.traj_id;p.sequence=1;p.valid=false;p.whole_curve=true;
    p.frame_id="odom";p.transport_mode="isolated_mock";p.collision_policy="observed_free";
    p.support_reference_id=support.support_reference_id;p.support_hash=support.support_hash;
    p.body_source_stamp=rclcpp::Time(10300000000LL);p.check_end=rclcpp::Time(10300000000LL);
    p.valid_until=rclcpp::Time(10500000000LL);
    p.reason=scan_planner::candidateAcceptanceReason(scan_planner::CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed);
    return p;
  }
};
}
TEST(PlanningOnlyCandidate, PendingGeometryHasIndependentWideConnectionButNoMotionJoinOrCertifiedSeed) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  PreviewCandidateFixture f;EXPECT_FALSE(f.ledger.committedView());EXPECT_FALSE(A::candidateSeed(f.ledger));
  ASSERT_TRUE(A::candidateOrigin(f.ledger));
  EXPECT_TRUE(samePreviewCandidateOrigin(*A::candidateOrigin(f.ledger),f.geometry.origin));
  EXPECT_EQ(rclcpp::Time(f.spline.join_source_stamp).nanoseconds(),0);EXPECT_EQ(f.spline.valid_start_time,0.);
  const auto projected=A::join(f.ledger,f.curve,f.body,false);ASSERT_TRUE(projected.time);
  EXPECT_EQ(projected.connection,MeasuredConnectionPolicy::PreviewCandidateGeometry);
  EXPECT_NEAR(projected.residual,.10,1e-8);EXPECT_NEAR(*projected.time,.3,1e-6);
  Fixture motion;motion.spline=f.spline;motion.ledger.candidate(motion.spline,motion.proposal);
  EXPECT_FALSE(A::join(motion.ledger,f.curve,f.body,false).time);
  EXPECT_THROW(motion.ledger.candidate(motion.spline,motion.proposal,{},f.geometry),std::invalid_argument);
}
TEST(ExecutionIdentity, ReferencePredecessorIsOneImmutableVersionAndCurvePair) {
  Fixture f(true);EXPECT_FALSE(f.ledger.committedIdentity());
  f.ledger.candidate(f.spline,f.proposal);
  auto first_permit=f.permit();first_permit.phase="preview";
  ASSERT_TRUE(f.ledger.commit(first_permit));
  const auto first=f.ledger.committedIdentity();ASSERT_TRUE(first);
  EXPECT_EQ(first->version,f.proposal.version);EXPECT_EQ(first->trajectory_id,1);
  // Deterministic interleaving: commit a new generation AFTER the acquired
  // predecessor. Its ID must not leak into that earlier identity value.
  ++f.proposal.version.reference_generation;f.proposal.proposal_id="proposal_2";
  f.spline.generation=2;f.spline.trajectory.traj_id=2;
  f.ledger.candidate(f.spline,f.proposal);
  auto second_permit=f.permit(2);second_permit.phase="preview";
  ASSERT_TRUE(f.ledger.commit(second_permit));
  const auto second=f.ledger.committedIdentity();ASSERT_TRUE(second);
  EXPECT_EQ(first->version.reference_generation,1u);EXPECT_EQ(first->trajectory_id,1);
  EXPECT_EQ(second->version.reference_generation,2u);EXPECT_EQ(second->trajectory_id,2);
  EXPECT_NE(first->version,second->version);
  auto cancel=second_permit;cancel.sequence=3;cancel.revoked=true;
  ASSERT_TRUE(f.ledger.commit(cancel));EXPECT_FALSE(f.ledger.committedIdentity());
  EXPECT_EQ(second->trajectory_id,2); // cancellation cannot mutate a copied view
}
TEST(ExecutionDisplay, LatestCommittedGeometryUsesOneImmutableSlotAndOriginalTimes) {
  using A=scan_planner::ExecutionValidatorTestAccess;Fixture f(true);
  for(unsigned i=1;i<=50;++i) {
    f.proposal.version.reference_generation=i;f.spline.generation=i;f.spline.trajectory.traj_id=i;
    f.spline.trajectory.start_time=rclcpp::Time(9000000000LL+i);
    f.ledger.candidate(f.spline,f.proposal);auto p=f.permit(i);p.phase="preview";
    ASSERT_TRUE(f.ledger.commit(p));EXPECT_EQ(A::pendingDisplays(f.ledger),1u);
  }
  const auto original=f.spline;
  f.spline.trajectory.start_time=rclcpp::Time(1LL);f.spline.trajectory.traj_id=999;
  std::vector<w::TaggedBspline> sent;A::dispatchDisplay(f.ledger,[&](const auto& m){sent.push_back(m);});
  ASSERT_EQ(sent.size(),1u);EXPECT_EQ(sent.front(),original);EXPECT_EQ(A::pendingDisplays(f.ledger),0u);
  A::dispatchDisplay(f.ledger,[&](const auto& m){sent.push_back(m);});EXPECT_EQ(sent.size(),1u);
  EXPECT_EQ(f.ledger.committedTrajectory(),50);
}
TEST(ExecutionDisplay, SlowDisplaySinkCannotBlockCommitCancelOrNewTaskLedger) {
  using A=scan_planner::ExecutionValidatorTestAccess;Fixture f(true);
  f.ledger.candidate(f.spline,f.proposal);auto first=f.permit();first.phase="preview";
  ASSERT_TRUE(f.ledger.commit(first));
  std::promise<void> entered,release;auto gate=release.get_future().share();auto started=entered.get_future();
  std::vector<w::TaggedBspline> sent;
  std::thread display([&]{A::dispatchDisplay(f.ledger,[&](const auto& m){
    entered.set_value();gate.wait();sent.push_back(m);
  });});
  const bool publishing=started.wait_for(std::chrono::seconds(2))==std::future_status::ready;
  EXPECT_TRUE(publishing);
  // Run real ledger operations while the middleware sink is deliberately
  // blocked. Always release/join before assertions, even if lock regression.
  auto operations=std::async(std::launch::async,[&] {
    if(f.ledger.committedTrajectory()!=1)return false;
    f.proposal.version.reference_generation=2;f.spline.generation=2;f.spline.trajectory.traj_id=2;
    f.ledger.candidate(f.spline,f.proposal);auto second=f.permit(2);second.phase="preview";
    if(!f.ledger.commit(second)||A::pendingDisplays(f.ledger)!=1)return false;
    auto barrier=f.proposal.version;barrier.reference_generation=3;
    if(!f.ledger.cancelReference(barrier)||A::pendingDisplays(f.ledger)!=0)return false;
    f.proposal.version.task_id="next_task";f.proposal.version.route_id="next_route";
    f.proposal.version.reference_generation=4;f.spline.generation=4;f.spline.trajectory.traj_id=4;
    f.spline.task_id="next_task";f.spline.route_id="next_route";
    f.ledger.candidate(f.spline,f.proposal);auto next=f.permit(3);next.phase="preview";
    return f.ledger.commit(next)&&f.ledger.committedTrajectory()==4&&A::pendingDisplays(f.ledger)==1;
  });
  const bool independent=operations.wait_for(std::chrono::seconds(2))==std::future_status::ready;
  release.set_value();display.join();EXPECT_TRUE(independent);EXPECT_TRUE(operations.get());
  ASSERT_EQ(sent.size(),1u);EXPECT_EQ(sent.front().trajectory.traj_id,1);
  // The already published old identity is display-only. The current permit
  // still owns 4, while cancelled pending display 2 was never emitted.
  EXPECT_EQ(f.ledger.committedTrajectory(),4);
  A::dispatchDisplay(f.ledger,[&](const auto& m){sent.push_back(m);});ASSERT_EQ(sent.size(),2u);
  EXPECT_EQ(sent.back().trajectory.traj_id,4);EXPECT_EQ(sent.back().task_id,"next_task");
  EXPECT_EQ(A::pendingDisplays(f.ledger),0u);
}
TEST(ExecutionDisplay, RevokedPermitDropsPendingDisplayWithoutRevivingItsGeometry) {
  using A=scan_planner::ExecutionValidatorTestAccess;Fixture f(true);
  f.ledger.candidate(f.spline,f.proposal);auto p=f.permit();p.phase="preview";
  ASSERT_TRUE(f.ledger.commit(p));ASSERT_EQ(A::pendingDisplays(f.ledger),1u);
  p.sequence=2;p.revoked=true;ASSERT_TRUE(f.ledger.commit(p));
  EXPECT_EQ(A::pendingDisplays(f.ledger),0u);EXPECT_FALSE(f.ledger.committedIdentity());
  unsigned published=0;A::dispatchDisplay(f.ledger,[&](const auto&){++published;});EXPECT_EQ(published,0u);
  p.sequence=3;p.revoked=false;EXPECT_FALSE(f.ledger.commit(p));EXPECT_EQ(A::pendingDisplays(f.ledger),0u);
}
TEST(PlanningOnlyCandidate, SlowReaderAfterFiveHundredMillisecondsUsesFreshBodyNotImmutableOriginAge) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  PreviewCandidateFixture f;f.body.source_stamp=10.6;
  const auto joined=A::join(f.ledger,f.curve,f.body,false);ASSERT_TRUE(joined.time);
  EXPECT_EQ(joined.candidate_outcome,CandidateAcceptanceOutcome::PreviewGeometryAccepted);
  EXPECT_EQ(joined.preview.source_stamp,10.6);
  EXPECT_EQ(A::candidateOrigin(f.ledger)->solve_body.source_stamp,10.);
  auto map=GridMapTestAccess::headingFixture(false,true);
  const auto check=[&](double now) {return wholeCurveCollisionFree(f.curve,.05,bodyExtent(map->planningBodyEnvelope()),
      [&](const Eigen::Vector3d& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);},
      f.body,"odom",now,.4,200000,0.,*joined.time,{},0.,joined.connection,
      f.geometry.origin.policy.connector_m);};
  EXPECT_TRUE(check(10.61));EXPECT_FALSE(check(11.01)); // same cached acquisition cannot renew data age
  EXPECT_FALSE(A::candidateSeed(f.ledger)); // a projection read is not a native whole-curve certificate
}
TEST(PlanningOnlyCandidate, ActualNativeBoxConnectorWholeCurveAndVerifiedSupportRemainRequired) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  PreviewCandidateFixture f;const auto joined=A::join(f.ledger,f.curve,f.body,false);ASSERT_TRUE(joined.time);
  auto map=GridMapTestAccess::headingFixture(false,true);
  const auto check=[&] {return measuredConnectionSupported(f.support,f.body.position,f.curve.evaluateDeBoorT(*joined.time),"odom",.05)&&
      wholeCurveCollisionFree(f.curve,.05,bodyExtent(map->planningBodyEnvelope()),
        [&](const Eigen::Vector3d& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);},
        f.body,"odom",10.31,.4,200000,0.,*joined.time,{},0.,joined.connection,
        f.geometry.origin.policy.connector_m);};
  EXPECT_TRUE(check());f.support.verified=false;EXPECT_FALSE(check());f.support.verified=true;
  for(auto& p:f.support.support_ground_xyz)p.z=3.;
  EXPECT_FALSE(check());
  for(auto& p:f.support.support_ground_xyz)p.z=0.;
  map->setOccupied({1.025,1.225,.575});
  EXPECT_EQ(map->getPlanningInflateOccupancy(f.curve.evaluateDeBoorT(0.),0.),0);EXPECT_FALSE(check());
  map=GridMapTestAccess::headingFixture(false,true);map->setOccupied({2.125,1.025,.575});EXPECT_FALSE(check());
}
TEST(PlanningOnlyCandidate, PositiveProofCasRequiresExactCandidateOriginSupportSeedAndUncommittedStage) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  for(int mode=0;mode<6;++mode) {
    PreviewCandidateFixture f;const auto read=A::candidateReadSnapshot(f.ledger);
    EXPECT_TRUE(A::positiveSeedCurrent(f.ledger,read));
    if(mode==0)A::previewSeed(f.ledger,f.geometry.projection);
    if(mode==1){auto next=f.geometry;next.origin.policy.projection_window_m=.4;f.ledger.candidate(f.spline,f.proposal,{},next);}
    if(mode==2){auto next=f.support;next.support_hash=std::string(64,'c');A::changedCandidateSupportReadRace(f.ledger,next);}
    if(mode==3){auto next=f.spline;++next.trajectory.traj_id;f.ledger.candidate(next,f.proposal,{},f.geometry);}
    if(mode==4)f.ledger.cancelPending();
    if(mode==5){auto p=f.permit();p.phase="preview";ASSERT_TRUE(f.ledger.commit(p));}
    EXPECT_FALSE(A::positiveSeedCurrent(f.ledger,read))<<mode;
  }
}
TEST(PlanningOnlyCandidate, GeometricOutOfRangeRequestsSameRouteReseedWithoutTrackerOrPermitOnce) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  PreviewCandidateFixture f;f.body.position.y()+=.06;f.body.source_stamp=10.31;f.ledger.body(f.body);
  EXPECT_FALSE(A::join(f.ledger,f.curve,f.body,false).time);
  EXPECT_FALSE(A::latest(f.ledger));EXPECT_FALSE(f.ledger.committedView());
  // Explicit proof-injection seam: tests the existing event lifecycle only;
  // real connector/whole native geometry is exercised independently above.
  auto proof=f.proof();proof.body_source_stamp=rclcpp::Time(10310000000LL);
  proof.check_end=rclcpp::Time(10310000000LL);A::previewProof(f.ledger,proof);
  const auto event=f.ledger.consumeFormalReseedAt(10310000000LL);ASSERT_TRUE(event);
  EXPECT_EQ(event->version,f.proposal.version);EXPECT_EQ(event->trajectory_id,1);
  EXPECT_EQ(rclcpp::Time(event->body_source_stamp).nanoseconds(),10310000000LL);EXPECT_FALSE(event->terminal);
  EXPECT_FALSE(f.ledger.consumeFormalReseedAt(10310000000LL));
}
TEST(PlanningOnlyCandidate, ChangedExpiredCanceledOrNonReseedProofCannotRequestGeometryReseed) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  for(int mode=0;mode<8;++mode) {
    PreviewCandidateFixture f;auto p=f.proof();
    if(mode==0)p.version.context_sequence++;
    if(mode==1)p.support_hash="different";
    if(mode==2)p.valid_until=rclcpp::Time(10310000000LL);
    if(mode==3)p.reason="occupied";
    if(mode==4)p.reason=candidateAcceptanceReason(CandidateAcceptanceOutcome::PreviewSourceExpired);
    if(mode==5)p.remaining_curve=true;
    if(mode==6)p.body_source_stamp=rclcpp::Time(9800000000LL);
    A::previewProof(f.ledger,p);if(mode==7)f.ledger.cancelPending();
    EXPECT_FALSE(f.ledger.consumeFormalReseedAt(10310000000LL))<<mode;
  }
}
TEST(PlanningOnlyCandidate, ReaderProjectionBudgetMissIsInconclusiveNotNegativeGeometryOrRenewedProof) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  PreviewCandidateFixture f;auto proof=f.proof();proof.valid=true;
  A::previewProof(f.ledger,proof);const auto before=A::candidateReadSnapshot(f.ledger);
  const auto missed=A::join(f.ledger,f.curve,f.body,false,ValidationCycle::Clock::now());
  EXPECT_FALSE(missed.time);EXPECT_EQ(missed.candidate_outcome,CandidateAcceptanceOutcome::PreviewBudgetExhausted);
  EXPECT_TRUE(A::projectionInconclusive(missed));
  const auto after=A::candidateReadSnapshot(f.ledger);
  ASSERT_TRUE(after.proof);EXPECT_EQ(*after.proof,*before.proof);EXPECT_FALSE(after.preview_progress);
  EXPECT_EQ(rclcpp::Time(after.proof->valid_until).nanoseconds(),10500000000LL);
  // Only an uncompleted geometric projection is inconclusive. Genuine
  // geometry/source/semantic failures must still produce their negative fact.
  for(const auto outcome:{CandidateAcceptanceOutcome::PreviewGeometryBlocked,
      CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed,CandidateAcceptanceOutcome::PreviewSourceExpired,
      CandidateAcceptanceOutcome::PreviewSourceChanged,CandidateAcceptanceOutcome::PreviewSegmentBoundary}) {
    auto negative=missed;negative.candidate_outcome=outcome;EXPECT_FALSE(A::projectionInconclusive(negative));
  }
  auto motion=missed;motion.connection=MeasuredConnectionPolicy::CandidateAdmission;
  EXPECT_FALSE(A::projectionInconclusive(motion)); // no change to Motion publication policy
}
TEST(PlanningOnlyProjection, CommittedReaderBudgetMissKeepsOnlyExistingProofAndSeed) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  PlanningOnlyFixture f;const auto before=A::readSnapshot(f.ledger);
  const auto missed=A::join(f.ledger,f.curve,f.body,true,ValidationCycle::Clock::now());
  EXPECT_FALSE(missed.time);EXPECT_EQ(missed.completion,CurveCheckCompletion::BudgetExhausted);
  EXPECT_EQ(missed.preview.status,NoMotionPreviewStatus::BudgetExhausted);
  EXPECT_TRUE(A::projectionInconclusive(missed));
  const auto after=A::readSnapshot(f.ledger);
  ASSERT_TRUE(before.proof);ASSERT_TRUE(after.proof);EXPECT_EQ(*after.proof,*before.proof);
  ASSERT_TRUE(before.preview_progress);ASSERT_TRUE(after.preview_progress);
  EXPECT_TRUE(sameNoMotionPreviewSeed(*before.preview_progress,*after.preview_progress));
  const auto deadline=rclcpp::Time(after.proof->valid_until).nanoseconds();
  EXPECT_TRUE(ValidationCycle::originalLeaseUsable(after.proof->valid,deadline,deadline-1));
  EXPECT_FALSE(ValidationCycle::originalLeaseUsable(after.proof->valid,deadline,deadline));
  EXPECT_TRUE(ValidationCycle::incompleteProof(true,true));
  EXPECT_FALSE(ValidationCycle::incompleteProof(true,false));
  EXPECT_FALSE(ValidationCycle::incompleteProof(false,true));
}
TEST(PlanningOnlyProjection, RealBodyWithoutTrackerDoesNotAdvanceByWallClockOrRepeatedReads) {
  PlanningOnlyFixture f;
  EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::activeProgress(f.ledger));
  for(int i=0;i<20;++i) {
    const auto time=f.project(PlanningOnlyFixture::now+100000000LL);
    ASSERT_TRUE(time);EXPECT_DOUBLE_EQ(*time,0.);
  }
  f.body.position=f.curve.evaluateDeBoorT(.8);f.body.source_stamp=10.1;
  const auto measured=f.project(PlanningOnlyFixture::now+100000000LL,PlanningOnlyFixture::now+100000000LL);
  ASSERT_TRUE(measured);EXPECT_NEAR(*measured,.8,.03);
  EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::activeProgress(f.ledger));
  EXPECT_EQ(scan_planner::ExecutionValidatorTestAccess::latest(f.ledger)->phase,"preview");
  EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::latest(f.ledger)->allowed);
}
TEST(PlanningOnlyProjection, ExactCommittedIdentityContextAndNoMotionAuthorityAreRequired) {
  for(int boundary=0;boundary<10;++boundary) {
    PlanningOnlyFixture f;auto expected=f.proposal.version;auto p=f.permit(2);p.phase="preview";
    std::int64_t trajectory=1;std::uint64_t context=1;
    if(boundary==0)++expected.anchor_revision;
    if(boundary==1)++expected.localization_epoch;
    if(boundary==2)++trajectory;
    if(boundary==3)++context;
    if(boundary==4||boundary==5) {
      if(boundary==4)p.allowed=true;else p.phase="holding";
      EXPECT_FALSE(f.ledger.commit(p));EXPECT_TRUE(f.project());continue;
    }
    if(boundary==6){p.revoked=true;ASSERT_TRUE(f.ledger.commit(p));}
    if(boundary==7){expected.task_id="other_task";}
    if(boundary==8){p.validation_sequence=2;ASSERT_TRUE(f.ledger.commit(p));}
    if(boundary==9){p.frame_id="other_frame";ASSERT_TRUE(f.ledger.commit(p));}
    EXPECT_FALSE(f.ledger.planningOnlyCurveTime(expected,trajectory,context,f.body,
      PlanningOnlyFixture::now,PlanningOnlyFixture::now,.08));
  }
}
TEST(PlanningOnlyProjection, StaleInvalidPartialOrMismatchedNativeProofNeverBecomesProgress) {
  for(int boundary=0;boundary<9;++boundary) {
    PlanningOnlyFixture f;
    if(boundary==0)f.proof.valid=false;
    if(boundary==1)f.proof.whole_curve=false;
    if(boundary==2)f.proof.remaining_curve=true;
    if(boundary==3)++f.proof.version.context_sequence;
    if(boundary==4)f.proof.valid_until=rclcpp::Time(PlanningOnlyFixture::now);
    if(boundary==5)f.proof.frame_id="other_frame";
    if(boundary==6)f.proof.support_hash="other_support";
    if(boundary==7)f.proof.proposal_id="other_proposal";
    if(boundary==8)f.proof.sequence=0;
    scan_planner::ExecutionValidatorTestAccess::previewProof(f.ledger,f.proof);
    EXPECT_FALSE(f.project());
  }
  PlanningOnlyFixture f;
  EXPECT_FALSE(f.project(PlanningOnlyFixture::now+250000000LL));
  EXPECT_EQ(rclcpp::Time(f.ledger.committedView()->proof->valid_until).nanoseconds(),
    PlanningOnlyFixture::now+250000000LL);
}
TEST(PlanningOnlyProjection, BodyMustBeFreshFiniteAndConnectedToActualCommittedCurve) {
  for(int boundary=0;boundary<8;++boundary) {
    PlanningOnlyFixture f;auto source=PlanningOnlyFixture::now;
    if(boundary==0){source-=401000000LL;f.body.source_stamp=source*1e-9;}
    if(boundary==1){source+=21000000LL;f.body.source_stamp=source*1e-9;}
    if(boundary==2)f.body.position.y()+=.3;
    if(boundary==3)f.body.position.z()+=3.;
    if(boundary==4)f.body.frame="other_frame";
    if(boundary==5)f.body.orientation.coeffs().setZero();
    if(boundary==6)f.body.position.x()=std::numeric_limits<double>::quiet_NaN();
    if(boundary==7)f.body.source_stamp+=.001;
    EXPECT_FALSE(f.project(PlanningOnlyFixture::now,source));
  }
}
TEST(PlanningOnlyProjection, PendingCandidateCannotOverwriteCommittedGeometryOrGrantWriterMode) {
  PlanningOnlyFixture f;
  auto next=f.spline;next.trajectory.traj_id=2;f.ledger.candidate(next,f.proposal);
  ASSERT_TRUE(f.project());
  EXPECT_FALSE(f.ledger.planningOnlyCurveTime(f.proposal.version,2,1,f.body,
    PlanningOnlyFixture::now,PlanningOnlyFixture::now,.08));
  scan_planner::ExecutionValidator writer{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
  w::SupportReference support;support.version=f.proposal.version;support.verified=true;
  support.support_reference_id="support";support.support_hash="support_hash";
  writer.support(support);writer.candidate(f.spline,f.proposal);
  scan_planner::ExecutionValidatorTestAccess::previewProof(writer,f.proof);
  auto p=f.permit();p.phase="preview";ASSERT_TRUE(writer.commit(p));
  EXPECT_FALSE(writer.planningOnlyCurveTime(f.proposal.version,1,1,f.body,
    PlanningOnlyFixture::now,PlanningOnlyFixture::now,.08));
}
TEST(PlanningOnlyProjection, OnlyExplicitCommittedPreviewUsesTypedSweptConnection) {
  using A=scan_planner::ExecutionValidatorTestAccess;using P=scan_planner::MeasuredConnectionPolicy;
  PlanningOnlyFixture f;f.body.position.y()=.06;f.body.source_stamp=10.1;
  const auto active=A::join(f.ledger,f.curve,f.body);
  ASSERT_TRUE(active.time);EXPECT_EQ(active.connection,P::CommittedNoMotionPreview);
  EXPECT_FALSE(active.remaining); // No tracker suffix becomes a whole-curve proof.
  auto pending=f.spline;pending.trajectory.traj_id=2;f.ledger.candidate(pending,f.proposal);
  const auto candidate=A::join(f.ledger,f.curve,f.body,false);
  EXPECT_FALSE(candidate.time);EXPECT_EQ(candidate.connection,P::CandidateAdmission);
  Fixture execution;execution.ledger.candidate(f.spline,execution.proposal);
  ASSERT_TRUE(execution.ledger.commit(execution.permit()));
  const auto unchanged=A::join(execution.ledger,f.curve,f.body);
  EXPECT_FALSE(unchanged.time);EXPECT_EQ(unchanged.connection,P::CandidateAdmission);
  EXPECT_THROW((scan_planner::ExecutionValidator{scan_planner::ExecutionValidator::PassiveLedger{},
    f.pool,"odom","isolated_mock",true,true}),std::invalid_argument);
}
TEST(PlanningOnlyProjection, OutOfRangeReturnsReseedTypeNotProgressOrPermission) {
  PlanningOnlyFixture f;f.body.position.y()=.16;f.body.source_stamp=10.1;
  const auto read=[&] {return f.ledger.planningOnlyProgress(f.proposal.version,1,1,f.body,
    PlanningOnlyFixture::now+100000000LL,PlanningOnlyFixture::now+100000000LL,.05);};
  EXPECT_EQ(read().status,scan_planner::NoMotionPreviewStatus::ReplanFromMeasuredBody);
  EXPECT_FALSE(f.project(PlanningOnlyFixture::now+100000000LL,PlanningOnlyFixture::now+100000000LL));
  f.proof.valid=false;f.proof.reason="preview_connection_out_of_range";f.proof.sequence=2;
  scan_planner::ExecutionValidatorTestAccess::previewProof(f.ledger,f.proof);
  EXPECT_EQ(read().status,scan_planner::NoMotionPreviewStatus::ReplanFromMeasuredBody);
  auto paused=f.permit(2);paused.phase="preview";paused.geometry_committed=false;
  EXPECT_FALSE(f.ledger.commit(paused));
  EXPECT_EQ(read().status,scan_planner::NoMotionPreviewStatus::ReplanFromMeasuredBody);
  EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::activeProgress(f.ledger));
  EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::latest(f.ledger)->allowed);
}
TEST(PlanningOnlyProjection, SourceGapRequestsReseedButCachedReceiptCannotRenewItsBodyAge) {
  PlanningOnlyFixture f;const auto later=PlanningOnlyFixture::now+600000000LL;
  auto p=f.permit(2);p.phase="preview";p.geometry_committed=false;
  p.source_stamp=rclcpp::Time(later);p.valid_until=rclcpp::Time(later+250000000LL);
  EXPECT_FALSE(f.ledger.commit(p));
  const auto read=[&](std::int64_t source) {return f.ledger.planningOnlyProgress(f.proposal.version,1,1,
    f.body,source,later,.05);};
  EXPECT_EQ(read(PlanningOnlyFixture::now).status,scan_planner::NoMotionPreviewStatus::Uncertified);
  f.body.source_stamp=later*1e-9;
  EXPECT_EQ(read(later).status,scan_planner::NoMotionPreviewStatus::ReplanFromMeasuredBody);
  const auto seed=scan_planner::ExecutionValidatorTestAccess::previewSeed(f.ledger);
  ASSERT_TRUE(seed);EXPECT_DOUBLE_EQ(seed->source_stamp,10.);
  EXPECT_EQ(rclcpp::Time(f.ledger.committedView()->proof->valid_until).nanoseconds(),PlanningOnlyFixture::now+250000000LL);
}
TEST(PlanningOnlyProjection, ContinuousBodyHistorySurvivesExpiredMapProofWithoutRenewingAnyAuthority) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  PlanningOnlyFixture f;const auto original_proof=*f.ledger.committedView()->proof;
  const auto original_permit=*A::latest(f.ledger);const auto original_seed=*A::previewSeed(f.ledger);
  // Reproduce the recorded stationary chain: odometry remains continuous for
  // >.4 s while ray certificates repeatedly expire. No tracker/motion exists.
  for(int i=1;i<=60;++i) {
    const auto source=PlanningOnlyFixture::now+i*20000000LL;
    f.body.source_stamp=source*1e-9;f.ledger.body(f.body,{},source);
  }
  const auto observed=A::observedPreviewSeed(f.ledger);ASSERT_TRUE(observed);
  EXPECT_NEAR(observed->source_stamp,11.2,1e-9);EXPECT_DOUBLE_EQ(observed->curve_time,0.);
  EXPECT_EQ(*f.ledger.committedView()->proof,original_proof);
  EXPECT_EQ(*A::latest(f.ledger),original_permit);
  EXPECT_TRUE(sameNoMotionPreviewSeed(*A::previewSeed(f.ledger),original_seed));
  const auto connected=A::join(f.ledger,f.curve,f.body);
  ASSERT_TRUE(connected.time);EXPECT_EQ(connected.preview.status,NoMotionPreviewStatus::Connected);
  // Projection history alone may not expose a safe/usable preview or motion.
  EXPECT_FALSE(f.project(11200000000LL,11200000000LL));
  EXPECT_FALSE(A::activeProgress(f.ledger));EXPECT_FALSE(A::latest(f.ledger)->allowed);
  // Only a genuinely new native proof restores visibility; its original
  // deadline is injected as a fact, not copied from the expired certificate.
  auto proof=original_proof;proof.sequence=2;proof.body_source_stamp=rclcpp::Time(11200000000LL);
  proof.check_end=proof.source_stamp=proof.body_source_stamp;proof.valid_until=rclcpp::Time(11450000000LL);
  A::previewProof(f.ledger,proof);
  auto permit=f.permit(2);permit.phase="preview";permit.source_stamp=proof.check_end;
  permit.valid_until=rclcpp::Time(11700000000LL);permit.validation_sequence=2;
  ASSERT_TRUE(f.ledger.commit(permit));ASSERT_TRUE(f.project(11200000000LL,11200000000LL));
  EXPECT_EQ(rclcpp::Time(f.ledger.committedView()->proof->valid_until).nanoseconds(),11450000000LL);
  EXPECT_FALSE(f.project(11450000000LL,11200000000LL));
}
TEST(PlanningOnlyProjection, AcquiredMotionAdvancesLocalArcWithoutChangingCertifiedSeedOrProof) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  PlanningOnlyFixture f;const auto before=A::readSnapshot(f.ledger);
  for(int i=1;i<=60;++i) {
    f.body.position=f.curve.evaluateDeBoorT(i*.04);
    const auto source=PlanningOnlyFixture::now+i*20000000LL;
    f.body.source_stamp=source*1e-9;f.ledger.body(f.body,{},source);
  }
  const auto observed=A::observedPreviewSeed(f.ledger);ASSERT_TRUE(observed);
  EXPECT_NEAR(observed->curve_time,2.4,1e-6);EXPECT_NEAR(observed->arc_length,.6,1e-6);
  EXPECT_TRUE(A::positiveSeedCurrent(f.ledger,before)); // full-check CAS is independent of observations.
  EXPECT_EQ(*f.ledger.committedView()->proof,*before.proof);
  const auto latest=A::join(f.ledger,f.curve,f.body);ASSERT_TRUE(latest.time);
  EXPECT_NEAR(*latest.time,2.4,1e-6);EXPECT_FALSE(latest.remaining);
}
TEST(PlanningOnlyProjection, ActualSourceGapRemotePoseDuplicatesAndCancelCannotAdvanceObservedHistory) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  for(int mode=0;mode<7;++mode) {
    PlanningOnlyFixture f;f.body.source_stamp=10.1;f.ledger.body(f.body,{},10100000000LL);
    ASSERT_TRUE(A::observedPreviewSeed(f.ledger));const auto before=*A::observedPreviewSeed(f.ledger);
    auto body=f.body;auto source=10200000000LL;
    if(mode==0){source=10600000000LL;body.source_stamp=10.6;} // real .5 s odometry gap.
    else body.source_stamp=source*1e-9;
    if(mode==1)body.position.z()+=3.;
    if(mode==2){source=10100000000LL;body.source_stamp=10.1;body.position.x()+=.05;}
    if(mode==3){source=10050000000LL;body.source_stamp=10.05;}
    if(mode==4)body.frame="map";
    if(mode==5)body.orientation.coeffs().setZero();
    if(mode==6){auto p=f.permit(2);p.phase="preview";p.revoked=true;ASSERT_TRUE(f.ledger.commit(p));}
    f.ledger.body(body,{},source);
    if(mode==6){EXPECT_FALSE(f.ledger.committedView());continue;}
    const auto after=A::observedPreviewSeed(f.ledger);ASSERT_TRUE(after);
    EXPECT_TRUE(sameNoMotionPreviewSeed(before,*after))<<mode;
    if(mode==0){EXPECT_EQ(A::join(f.ledger,f.curve,body).preview.status,NoMotionPreviewStatus::ReplanFromMeasuredBody);}
  }
}
TEST(PlanningOnlyProjection, FreshObservationNeverOverridesNegativeCollisionOrNewTaskIdentity) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  PlanningOnlyFixture f;f.body.source_stamp=10.1;f.ledger.body(f.body,{},10100000000LL);
  auto negative=f.proof;negative.valid=false;negative.reason="occupied";negative.sequence=2;
  A::previewProof(f.ledger,negative);
  f.body.source_stamp=10.2;f.ledger.body(f.body,{},10200000000LL);
  ASSERT_TRUE(A::observedPreviewSeed(f.ledger));
  EXPECT_EQ(*f.ledger.committedView()->proof,negative);EXPECT_FALSE(f.project(10200000000LL,10200000000LL));
  auto other=f.proposal.version;other.localization_epoch++;
  EXPECT_FALSE(f.ledger.planningOnlyCurveTime(other,1,1,f.body,10200000000LL,10200000000LL,.05));
}
TEST(PlanningOnlyProjection, ConcurrentOwnerFloorOrSupportChangeWinsEndOfProjectionCas) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(int boundary=0;boundary<5;++boundary) {
    PlanningOnlyFixture f;const auto read=A::readSnapshot(f.ledger);const auto original=*A::latest(f.ledger);
    ASSERT_TRUE(A::endReadCurrent(f.ledger,read,original,PlanningOnlyFixture::now,PlanningOnlyFixture::now));
    auto p=f.permit(2);p.phase="preview";
    if(boundary==0)p.validation_sequence=2;
    if(boundary==1)p.revoked=true;
    if(boundary==2)p.geometry_committed=false;
    if(boundary==3) {
      // Adversarial mutation seam: production support ingress now forbids a
      // changed payload at the same version. The final read/CAS still detects
      // any concurrent geometry mutation, rather than relying on that inbox.
      auto support=*read.support;support.support_hash="new_support";
      A::changedSupportReadRace(f.ledger,support);
    }
    if(boundary==4)p.frame_id="other_frame";
    if(boundary!=3)f.ledger.commit(p);
    EXPECT_FALSE(A::endReadCurrent(f.ledger,read,original,PlanningOnlyFixture::now,PlanningOnlyFixture::now));
  }
}
TEST(ExecutionSupportIngress, OriginalSupportDeliveryIsImmutableIdempotentAndTaskScoped) {
 using A=scan_planner::ExecutionValidatorTestAccess;Fixture f;
 w::SupportReference support;support.version=f.proposal.version;support.verified=true;
 support.support_reference_id="support";support.support_hash="hash";support.frame_id="odom";
 support.source_stamp=rclcpp::Time(10000000000LL);support.support_ground_xyz.emplace_back();
 // The normal wire order is support first, proposal next; no owner permit is
 // required to receive static geometry in an empty task domain.
 f.ledger.support(support);ASSERT_EQ(A::supportHistorySize(f.ledger),1u);
 f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
 for(int i=0;i<30;++i)f.ledger.support(support);
 EXPECT_EQ(A::supportHistorySize(f.ledger),1u);
 EXPECT_EQ(*A::readSnapshot(f.ledger).support,support);
 for(int mutation=0;mutation<6;++mutation) {
  auto conflict=support;
  if(mutation==0)conflict.support_ground_xyz[0].z=.01;
  if(mutation==1)conflict.source_stamp=rclcpp::Time(11000000000LL);
  if(mutation==2)conflict.support_hash="different-hash";
  if(mutation==3)conflict.verified=false;
  if(mutation==4)conflict.body_reference_height_m=.5;
  if(mutation==5)conflict.support_reference_id="different-id";
  f.ledger.support(conflict);
  EXPECT_EQ(A::supportHistorySize(f.ledger),1u)<<mutation;
  EXPECT_EQ(*A::readSnapshot(f.ledger).support,support)<<mutation;
 }
 auto foreign=support;foreign.version.task_id="foreign-task";f.ledger.support(foreign);
 EXPECT_EQ(A::supportHistorySize(f.ledger),1u);
 auto next=support;++next.version.reference_generation;next.support_reference_id="support-2";
 f.ledger.support(next);EXPECT_EQ(A::supportHistorySize(f.ledger),2u);
 EXPECT_EQ(*A::readSnapshot(f.ledger).support,support);
 auto revoked=f.permit(2);revoked.revoked=true;ASSERT_TRUE(f.ledger.commit(revoked));
 auto late=support;late.version.reference_generation=3;f.ledger.support(late);
 EXPECT_EQ(A::supportHistorySize(f.ledger),2u); // Closed task cannot resurrect even at a higher revision.
}
TEST(PlanningOnlyProjection, ConcurrentCertifiedSeedCannotReturnOldReadOrRatchetPositiveHighWater) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  PlanningOnlyFixture f;const auto original=A::readSnapshot(f.ledger);const auto permit=*A::latest(f.ledger);
  EXPECT_TRUE(A::positiveSeedCurrent(f.ledger,original));
  auto newer=*A::previewSeed(f.ledger);newer.curve_time=2.4;newer.arc_length=newer.high_water_arc=.6;
  newer.source_stamp=10.1;newer.measured_position=f.curve.evaluateDeBoorT(2.4);
  A::previewSeed(f.ledger,newer);
  EXPECT_FALSE(A::positiveSeedCurrent(f.ledger,original));
  EXPECT_FALSE(A::endReadCurrent(f.ledger,original,permit,PlanningOnlyFixture::now,PlanningOnlyFixture::now));
  // This guard is applied only when result.valid: a later actual occupied
  // certificate is not suppressed just because its copied seed is obsolete.
  EXPECT_DOUBLE_EQ(A::previewSeed(f.ledger)->high_water_arc,.6);
  const auto latest=A::readSnapshot(f.ledger);EXPECT_TRUE(A::positiveSeedCurrent(f.ledger,latest));
}
TEST(PlanningOnlyProjection, ReadOnlyProjectionCannotCertifyOrRenewNewBodyConnector) {
  PlanningOnlyFixture f;f.body.position.y()=.10;f.body.source_stamp=10.1;
  ASSERT_TRUE(f.project(PlanningOnlyFixture::now+100000000LL,PlanningOnlyFixture::now+100000000LL));
  const auto seed=scan_planner::ExecutionValidatorTestAccess::previewSeed(f.ledger);
  ASSERT_TRUE(seed);EXPECT_DOUBLE_EQ(seed->source_stamp,10.);EXPECT_DOUBLE_EQ(seed->measured_position.y(),0.);
  EXPECT_EQ(rclcpp::Time(f.ledger.committedView()->proof->body_source_stamp).nanoseconds(),PlanningOnlyFixture::now);
  EXPECT_EQ(rclcpp::Time(f.ledger.committedView()->proof->valid_until).nanoseconds(),PlanningOnlyFixture::now+250000000LL);
  f.body.position.z()+=3.;
  EXPECT_FALSE(f.project(PlanningOnlyFixture::now+100000000LL,PlanningOnlyFixture::now+100000000LL));
}
TEST(PlanningOnlyProjection, NativeBoxStillChecksActualConnectorAndOriginalWholeCurve) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  PlanningOnlyFixture f({.8,1.,0.});f.body.position.y()+=.10;f.body.source_stamp=10.1;
  const auto projection=A::join(f.ledger,f.curve,f.body);ASSERT_TRUE(projection.time);
  auto map=GridMapTestAccess::headingFixture(false,true);
  const auto check=[&] {return scan_planner::wholeCurveCollisionFree(f.curve,map->getResolution(),
      scan_planner::bodyExtent(map->planningBodyEnvelope()),
      [&](const Eigen::Vector3d& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);},
      f.body,"odom",10.1,.4,200000,0.,*projection.time,{},0.,projection.connection);};
  EXPECT_TRUE(check());
  // Occupancy outside the original narrow box, but inside the acquired body
  // footprint/connector, must deny renewal even when all curve centres clear.
  map->setOccupied({1.025,1.225,.575});
  EXPECT_EQ(map->getPlanningInflateOccupancy(f.curve.evaluateDeBoorT(0.),0.),0);
  EXPECT_FALSE(check());
  map=GridMapTestAccess::headingFixture(false,true);map->setOccupied({2.125,1.025,.575});
  EXPECT_FALSE(check()); // The final original whole-curve domain is retained.
}
TEST(PlanningOnlyProjection, ActualConnectorRequiresVerifiedSameFloorSupportAtEveryPoint) {
  PlanningOnlyFixture f;f.body.position.y()=.10;f.body.source_stamp=10.1;
  const auto projection=scan_planner::ExecutionValidatorTestAccess::join(f.ledger,f.curve,f.body);
  ASSERT_TRUE(projection.time);
  w::SupportReference support;support.verified=true;support.floor_id="floor1";support.segment_kind="floor";
  support.required_mode="general";support.frame_id="odom";support.support_hash=std::string(64,'a');
  support.support_map_sha256=std::string(64,'b');support.support_xy_radius_m=.15;
  support.body_reference_height_m=.55;support.max_support_slope_rad=.1;support.max_support_step_m=.05;
  for(int i=0;i<20;++i){geometry_msgs::msg::Point p;p.x=(i-2)*.05;support.support_ground_xyz.push_back(p);}
  const auto supported=[&] {return scan_planner::measuredConnectionSupported(support,f.body.position,
    f.curve.evaluateDeBoorT(*projection.time),"odom",.05);};
  EXPECT_TRUE(supported());support.floor_id="floor2";EXPECT_TRUE(supported());
  support.floor_id="floor3";EXPECT_FALSE(supported());support.floor_id="floor1";
  support.verified=false;EXPECT_FALSE(supported());support.verified=true;
  for(auto& p:support.support_ground_xyz)p.z=3.;
  EXPECT_FALSE(supported());
}
TEST(ExecutionSnapshotPolicy, NativeReadersShareSelectedPolicyAndStrictIsExplicit) {
  using scan_planner::validationReadRequiresObservedSnapshot;
  EXPECT_FALSE(validationReadRequiresObservedSnapshot("official_inflated_geometry",false));
  EXPECT_FALSE(validationReadRequiresObservedSnapshot("official_inflated_geometry",true));
  EXPECT_TRUE(validationReadRequiresObservedSnapshot("official_inflated_geometry",true,"observed_free"));
  EXPECT_TRUE(validationReadRequiresObservedSnapshot("observed_free",false));
  EXPECT_TRUE(validationReadRequiresObservedSnapshot("observed_free",true));
  EXPECT_TRUE(validationReadRequiresObservedSnapshot("invalid_policy",false));
}

TEST(ExecutionSnapshotPolicy, OfficialPreviewAndActualReaderKeepSameUnknownRawValues) {
  using scan_planner::validationReadRequiresObservedSnapshot;
  const auto writer=GridMapTestAccess::headingFixture(false);
  const auto original=GridMapTestAccess::raw(*writer);
  const Eigen::Vector3d position(.3,0.,0.); // Entire envelope inside the centered fixture map.
  GridMap preview;writer->copyCollisionSnapshotTo(preview,10000000000LL,std::chrono::steady_clock::now());
  if(validationReadRequiresObservedSnapshot("official_inflated_geometry",false))preview.requireObservedSnapshot();
  EXPECT_FALSE(preview.requiresObservedFree());
  EXPECT_EQ(preview.getPlanningInflateOccupancy(position,0.),0);
  EXPECT_EQ(preview.getInflateOccupancy(position,0.),0);
  EXPECT_GT(preview.inspectInflateOccupancy(position,0.,true).classification_counts[
      static_cast<std::size_t>(scan_planner::RawVoxelDiagnostic::NeverObserved)],0U);
  EXPECT_EQ(GridMapTestAccess::raw(preview),original);

  GridMap motion;writer->copyCollisionSnapshotTo(motion,10000000000LL,std::chrono::steady_clock::now());
  if(validationReadRequiresObservedSnapshot("official_inflated_geometry",true))motion.requireObservedSnapshot();
  EXPECT_FALSE(motion.requiresObservedFree());
  EXPECT_EQ(motion.getInflateOccupancy(position,0.),0);
  // Actual-reader selection never changes raw odds or manufactures FREE.
  EXPECT_EQ(motion.getPlanningInflateOccupancy(position,0.),0);
  EXPECT_EQ(GridMapTestAccess::raw(motion),original);
  EXPECT_EQ(GridMapTestAccess::raw(*writer),original);
  EXPECT_FALSE(writer->requiresObservedFree());

  GridMap strict;writer->copyCollisionSnapshotTo(strict,10000000000LL,std::chrono::steady_clock::now());
  if(validationReadRequiresObservedSnapshot("official_inflated_geometry",true,"observed_free"))strict.requireObservedSnapshot();
  EXPECT_TRUE(strict.requiresObservedFree());EXPECT_EQ(strict.getInflateOccupancy(position,0.),2);
  EXPECT_EQ(GridMapTestAccess::raw(strict),original);
}

TEST(ExecutionLedger,FrozenPreviewHeadingIsCopiedWithCurveAndNotReplacedByLatestBodyOrCandidate) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  Fixture f;scan_planner::SplineHeadingContract heading;
  heading.preview_only_enabled=true;heading.measured_yaw=.4;
  f.ledger.candidate(f.spline,f.proposal,heading);
  heading.measured_yaw=2.;  // Caller mutation cannot change the accepted seed.
  EXPECT_DOUBLE_EQ(A::candidateHeading(f.ledger).measured_yaw,.4);
  ASSERT_TRUE(f.ledger.commit(f.permit()));
  scan_planner::MeasuredBodyPose body{{0.,0.,.55},Eigen::Quaterniond(
      Eigen::AngleAxisd(1.2,Eigen::Vector3d::UnitZ())),10.,"odom"};
  f.ledger.body(body);
  EXPECT_DOUBLE_EQ(A::activeHeading(f.ledger).measured_yaw,.4);
  f.spline.trajectory.traj_id=2;
  f.ledger.candidate(f.spline,f.proposal,heading);
  EXPECT_DOUBLE_EQ(A::candidateHeading(f.ledger).measured_yaw,2.);
  EXPECT_DOUBLE_EQ(A::activeHeading(f.ledger).measured_yaw,.4);
  f.ledger.cancelPending();
  EXPECT_EQ(f.ledger.committedTrajectory(),1);
  EXPECT_DOUBLE_EQ(A::activeHeading(f.ledger).measured_yaw,.4);
}
TEST(ExecutionLedger,FrozenHeldYawControlsAndLedgerRemainOneImmutableCurveContract) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  Fixture f;SplineHeadingContract held;held.measured_heading_enabled=true;
  held.policy=BodyHeadingPolicy::HoldHeading;held.measured_yaw=.7;
  f.spline.trajectory.order=3;f.spline.trajectory.pos_pts.resize(7);
  for(int i=0;i<11;++i)f.spline.trajectory.knots.push_back((i-3)*.25);
  EXPECT_THROW(f.ledger.candidate(f.spline,f.proposal,held),std::invalid_argument);
  ASSERT_TRUE(encodeFrozenHeading(f.spline.trajectory,held));
  ASSERT_TRUE(frozenHeadingWireMatches(f.spline.trajectory,held));
  f.ledger.candidate(f.spline,f.proposal,held);ASSERT_TRUE(f.ledger.commit(f.permit()));
  EXPECT_EQ(A::activeHeading(f.ledger).policy,BodyHeadingPolicy::HoldHeading);
  EXPECT_DOUBLE_EQ(A::activeHeading(f.ledger).measured_yaw,.7);
  for(int mode=0;mode<3;++mode) {
    auto bad=f.spline;
    if(mode==0)bad.trajectory.yaw_pts[2]=.8;
    if(mode==1)bad.trajectory.yaw_pts.pop_back();
    if(mode==2)bad.trajectory.yaw_dt=.5;
    EXPECT_THROW(f.ledger.candidate(bad,f.proposal,held),std::invalid_argument);
    EXPECT_DOUBLE_EQ(A::activeHeading(f.ledger).measured_yaw,.7);
  }
  MeasuredBodyPose newest{{0.,0.,.55},Eigen::Quaterniond(Eigen::AngleAxisd(-.9,Eigen::Vector3d::UnitZ())),
      10.1,"odom"};f.ledger.body(newest);
  EXPECT_DOUBLE_EQ(bodyHeadingFromVelocity({0.,.2,0.},-.9,A::activeHeading(f.ledger)),.7);
  SplineHeadingContract tangent;tangent.measured_heading_enabled=true;tangent.measured_yaw=-.9;
  auto next=f.spline;ASSERT_TRUE(encodeFrozenHeading(next.trajectory,tangent));
  EXPECT_TRUE(next.trajectory.yaw_pts.empty());EXPECT_DOUBLE_EQ(next.trajectory.yaw_dt,0.);
  f.ledger.candidate(next,f.proposal,tangent);
  EXPECT_EQ(A::activeHeading(f.ledger).policy,BodyHeadingPolicy::HoldHeading);
}
TEST(ExecutionLedger,PreviewHeadingCannotEnterWriterOrReplaceGoodCandidateWithInvalidSeed) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  Fixture f;scan_planner::SplineHeadingContract heading;heading.preview_only_enabled=true;
  f.ledger.candidate(f.spline,f.proposal,heading);
  heading.measured_yaw=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(f.ledger.candidate(f.spline,f.proposal,heading),std::invalid_argument);
  EXPECT_DOUBLE_EQ(A::candidateHeading(f.ledger).measured_yaw,0.);
  scan_planner::ExecutionValidator writer{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
  heading.measured_yaw=0.;
  EXPECT_THROW(writer.candidate(f.spline,f.proposal,heading),std::invalid_argument);
  writer.candidate(f.spline,f.proposal);
  EXPECT_FALSE(A::candidateHeading(writer).preview_only_enabled);
}
TEST(ExecutionLedger,FormalWriterUsesMeasuredGeometryWithoutInheritingPreviewAuthorization) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  Fixture f;
  scan_planner::ExecutionValidator writer{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
  scan_planner::SplineHeadingContract heading;
  heading.measured_heading_enabled=true;heading.measured_yaw=.7;
  writer.candidate(f.spline,f.proposal,heading);
  EXPECT_TRUE(A::candidateHeading(writer).measured_heading_enabled);
  EXPECT_FALSE(A::candidateHeading(writer).preview_only_enabled);
  EXPECT_DOUBLE_EQ(A::candidateHeading(writer).measured_yaw,.7);
  // Reading a geometry contract is not a commit or a motion authorization.
  EXPECT_EQ(writer.committedTrajectory(),-1);
  heading.measured_yaw=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(writer.candidate(f.spline,f.proposal,heading),std::invalid_argument);
  EXPECT_DOUBLE_EQ(A::candidateHeading(writer).measured_yaw,.7);
}
TEST(ExecutionLedger,IndependentPreviewUsesSolverHeadingButStillRejectsActualRotationAndNewOccupancy) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  Fixture f;scan_planner::SplineHeadingContract heading;heading.preview_only_enabled=true;
  f.ledger.candidate(f.spline,f.proposal,heading);
  auto map=GridMapTestAccess::headingFixture();
  map->setOccupied({.025,.425,.225});
  Eigen::MatrixXd points(3,7);
  for(int i=0;i<points.cols();++i)points.col(i)=Eigen::Vector3d(.025,.025+.009*((i-1)*.25),.55);
  scan_planner::UniformBspline curve(points,3,.25);
  scan_planner::MeasuredBodyPose body{{.025,.025,.55},Eigen::Quaterniond::Identity(),10.,"odom"};
  const auto check=[&](const scan_planner::SplineHeadingContract& h) {
    return scan_planner::wholeCurveCollisionFree(curve,.05,.49,
      [&](const Eigen::Vector3d& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);},
      body,"odom",10.,.5,50000,0.,0.,h);
  };
  EXPECT_FALSE(check({}));  // Old independent reader inferred 90deg from 9mm/s.
  EXPECT_TRUE(check(A::candidateHeading(f.ledger)));
  body.position=curve.evaluateDeBoorT(.2);  // Physical progress does not reseed yaw.
  EXPECT_TRUE(check(A::candidateHeading(f.ledger)));
  body.orientation=Eigen::AngleAxisd(std::acos(-1.)/2.,Eigen::Vector3d::UnitZ());
  EXPECT_FALSE(check(A::candidateHeading(f.ledger)));
  body.orientation=Eigen::Quaterniond::Identity();
  map->setOccupied({.025,.025,.225});
  EXPECT_FALSE(check(A::candidateHeading(f.ledger)));
}
TEST(ExecutionLedger,UnboundGeometryDoesNotPinWriterAndFirstRealGrantUsesLatestGeometry) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
  ledger.candidate(f.spline,f.proposal);
  auto geometry=f.permit();geometry.phase="preview";geometry.execution_id="exec";geometry.control_epoch=1;
  ASSERT_TRUE(ledger.commit(geometry));EXPECT_FALSE(A::writerDomain(ledger));
  // A new preview curve is still geometry, not another writer incarnation.
  f.spline.trajectory.traj_id=2;ledger.candidate(f.spline,f.proposal);
  geometry.sequence=2;geometry.trajectory_id=2;ASSERT_TRUE(ledger.commit(geometry));
  EXPECT_FALSE(A::writerDomain(ledger));
  auto granted=geometry;granted.sequence=3;granted.allowed=true;granted.phase="tracking";
  granted.sdk_session="sdk";granted.sdk_arm_generation=3;
  ASSERT_TRUE(ledger.commit(granted));ASSERT_TRUE(A::writerDomain(ledger));
  EXPECT_EQ(A::writerDomain(ledger)->sdk_session,"sdk");EXPECT_EQ(A::latest(ledger)->sequence,3u);
  auto late=geometry;late.sequence=4;EXPECT_FALSE(ledger.commit(late));
  late.execution_id.clear();late.control_epoch=0;EXPECT_FALSE(ledger.commit(late));
  for(int variant=0;variant<3;++variant) {
    auto changed=granted;changed.sequence=5+variant;
    if(variant==0)changed.sdk_session="foreign";
    if(variant==1)++changed.sdk_arm_generation;
    if(variant==2)changed.execution_id="foreign";
    EXPECT_FALSE(ledger.commit(changed));
  }
  EXPECT_EQ(A::latest(ledger)->sequence,3u);EXPECT_EQ(ledger.committedTrajectory(),2);
}
TEST(ExecutionLedger,FirstWriterBindingCannotChangePendingExecutionOrGeometry) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(int variant=0;variant<7;++variant) {
    Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
    ledger.candidate(f.spline,f.proposal);auto geometry=f.permit();geometry.phase="preview";
    geometry.execution_id="exec";geometry.control_epoch=1;ASSERT_TRUE(ledger.commit(geometry));
    auto bad=geometry;bad.sequence=2;bad.allowed=true;bad.phase="tracking";
    bad.sdk_session="sdk";bad.sdk_arm_generation=3;
    if(variant==0)bad.execution_id="foreign";
    if(variant==1)++bad.control_epoch;
    if(variant==2) {++f.proposal.version.reference_generation;bad.version=f.proposal.version;}
    if(variant==3) {f.spline.trajectory.traj_id=2;bad.trajectory_id=2;}
    if(variant==4)bad.sdk_session.clear();
    if(variant==5)bad.sdk_arm_generation=0;
    if(variant==6) {bad.sdk_session.clear();bad.sdk_arm_generation=0;}
    ledger.candidate(f.spline,f.proposal);
    EXPECT_FALSE(ledger.commit(bad))<<variant;EXPECT_FALSE(A::writerDomain(ledger))<<variant;
    ASSERT_TRUE(A::latest(ledger));EXPECT_EQ(A::latest(ledger)->sequence,1u);
    EXPECT_EQ(ledger.committedTrajectory(),1);
  }
}
TEST(ExecutionLedger,NewUnboundArmingEpochRetiresCasAndFencesDelayedPriorWriter) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
  ledger.candidate(f.spline,f.proposal);auto initial=f.permit();initial.allowed=true;
  initial.execution_id="exec";initial.control_epoch=1;initial.sdk_session="sdk";initial.sdk_arm_generation=3;
  initial.phase="tracking";ASSERT_TRUE(ledger.commit(initial));
  w::ExecutionCommitAck first;first.schema_version=1;first.sequence=1;first.applied=first.write_submitted=true;
  first.commit_sequence=1;first.candidate_version=initial.version;first.candidate_trajectory_id=1;
  first.execution_id=initial.execution_id;first.control_epoch=1;first.sdk_session="sdk";first.sdk_arm_generation=3;
  first.permit_sequence=1;first.transport_mode="isolated_mock";first.applied_at=rclcpp::Time(10010000000LL);
  ASSERT_TRUE(ledger.commitAckAt(first,10020000000LL));EXPECT_EQ(A::writerSequence(ledger),1u);
  auto arming=initial;arming.sequence=2;arming.allowed=arming.geometry_committed=false;
  arming.execution_id="exec_2";arming.control_epoch=2;arming.sdk_session.clear();arming.sdk_arm_generation=0;
  arming.trajectory_id=0;arming.phase="arming";
  EXPECT_FALSE(ledger.commit(arming)); // No geometry commit, but owner ingress is recorded.
  EXPECT_FALSE(A::writerDomain(ledger));EXPECT_EQ(A::writerSequence(ledger),0u);
  ASSERT_TRUE(A::latest(ledger));EXPECT_EQ(A::latest(ledger)->control_epoch,2u);
  auto late=initial;late.sequence=3;EXPECT_FALSE(ledger.commit(late));
  ++first.sequence;EXPECT_FALSE(ledger.commitAckAt(first,10030000000LL));
  EXPECT_EQ(A::latest(ledger)->control_epoch,2u);EXPECT_FALSE(A::writerDomain(ledger));
  auto next=arming;next.sequence=4;next.allowed=next.geometry_committed=true;next.trajectory_id=1;
  next.sdk_session="sdk_2";next.sdk_arm_generation=4;next.phase="tracking";
  ASSERT_TRUE(ledger.commit(next));ASSERT_TRUE(A::writerDomain(ledger));
  EXPECT_EQ(A::writerDomain(ledger)->control_epoch,2u);EXPECT_EQ(A::writerDomain(ledger)->sdk_session,"sdk_2");
  late.sequence=5;EXPECT_FALSE(ledger.commit(late));
  auto late_preview=arming;late_preview.sequence=6;late_preview.geometry_committed=true;late_preview.trajectory_id=1;
  late_preview.phase="preview";EXPECT_FALSE(ledger.commit(late_preview));
  EXPECT_EQ(A::latest(ledger)->sequence,4u);EXPECT_EQ(ledger.committedTrajectory(),1);
}
TEST(ExecutionLedger,ArmingWithoutGeometryMayBindOnlyCurrentCommittedCurve) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
  ledger.candidate(f.spline,f.proposal);auto geometry=f.permit();geometry.phase="preview";
  ASSERT_TRUE(ledger.commit(geometry));
  auto arming=geometry;arming.sequence=2;arming.geometry_committed=false;arming.trajectory_id=0;
  arming.execution_id="exec";arming.control_epoch=1;arming.phase="arming";
  EXPECT_FALSE(ledger.commit(arming));EXPECT_FALSE(A::writerDomain(ledger));
  auto granted=arming;granted.sequence=3;granted.allowed=granted.geometry_committed=true;
  granted.trajectory_id=1;granted.phase="tracking";granted.sdk_session="sdk";granted.sdk_arm_generation=3;
  ASSERT_TRUE(ledger.commit(granted));ASSERT_TRUE(A::writerDomain(ledger));
}
TEST(ExecutionLedger,FirstSdkBindingUsesCurrentGeometryAfterReferenceAdvanceNotOldArmingVersion) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(const std::string phase:{"arming","holding"}) {
    Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
    ledger.candidate(f.spline,f.proposal);auto geometry=f.permit();geometry.phase="preview";
    geometry.execution_id="exec";geometry.control_epoch=1;ASSERT_TRUE(ledger.commit(geometry));
    ++f.proposal.version.reference_generation;++f.proposal.version.anchor_revision;
    f.spline.generation=2;f.spline.trajectory.traj_id=2;ledger.candidate(f.spline,f.proposal);
    auto current=geometry;current.sequence=2;current.version=f.proposal.version;current.trajectory_id=2;
    ASSERT_TRUE(ledger.commit(current));
    auto pending=geometry;pending.sequence=3;pending.geometry_committed=false;pending.trajectory_id=0;pending.phase=phase;
    EXPECT_FALSE(ledger.commit(pending));EXPECT_FALSE(A::writerDomain(ledger));
    auto granted=current;granted.sequence=4;granted.allowed=true;granted.phase="tracking";
    granted.sdk_session="sdk";granted.sdk_arm_generation=3;
    ASSERT_TRUE(ledger.commit(granted));ASSERT_TRUE(A::writerDomain(ledger));
    EXPECT_EQ(A::writerDomain(ledger)->version,current.version);EXPECT_EQ(ledger.committedTrajectory(),2);
  }
}
TEST(ExecutionLedger,UnboundPendingOwnerRejectsDifferentExecutionAndFullyEmptyLatePreview) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
  ledger.candidate(f.spline,f.proposal);auto geometry=f.permit();geometry.phase="preview";
  geometry.execution_id="exec";geometry.control_epoch=1;ASSERT_TRUE(ledger.commit(geometry));
  auto late=geometry;late.sequence=2;late.execution_id="foreign";
  EXPECT_FALSE(ledger.commit(late));EXPECT_EQ(A::latest(ledger)->execution_id,"exec");
  late.execution_id.clear();late.control_epoch=0;EXPECT_FALSE(ledger.commit(late));
  EXPECT_EQ(A::latest(ledger)->sequence,1u);EXPECT_FALSE(A::writerDomain(ledger));
  auto higher=geometry;higher.sequence=3;higher.execution_id="exec_2";higher.control_epoch=2;
  higher.allowed=higher.geometry_committed=false;higher.phase="holding";
  EXPECT_FALSE(ledger.commit(higher));EXPECT_EQ(A::latest(ledger)->control_epoch,2u);
  geometry.sequence=4;EXPECT_FALSE(ledger.commit(geometry));
  auto bound=higher;bound.sequence=5;bound.sdk_session="sdk_2";bound.sdk_arm_generation=4;
  EXPECT_FALSE(ledger.commit(bound)); // arming binding is recorded, not a geometry commit.
  ASSERT_TRUE(A::writerDomain(ledger));EXPECT_EQ(A::writerDomain(ledger)->control_epoch,2u);
  bound.sequence=6;bound.allowed=bound.geometry_committed=true;bound.phase="tracking";
  ASSERT_TRUE(ledger.commit(bound));EXPECT_EQ(A::latest(ledger)->sequence,6u);
}
TEST(ExecutionLedger,WriterCASPinsPreparedUntilMatchedAckAndNeverRollsBack) {
  Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
  f.ledger.candidate(f.spline,f.proposal);ledger.candidate(f.spline,f.proposal);
  auto old=f.permit();old.allowed=true;old.execution_id="exec";old.control_epoch=1;
  old.sdk_session="sdk";old.sdk_arm_generation=1;old.source_stamp=rclcpp::Time(10000000000LL);
  old.valid_until=rclcpp::Time(10500000000LL);old.frame_id="odom";old.phase="tracking";
  ASSERT_TRUE(ledger.commit(old));w::ExecutionCommitAck first;first.schema_version=1;first.sequence=1;
  first.applied=first.write_submitted=true;first.commit_sequence=1;first.candidate_version=old.version;
  first.candidate_trajectory_id=1;first.execution_id=old.execution_id;first.control_epoch=1;first.sdk_session="sdk";
  first.sdk_arm_generation=1;first.permit_sequence=1;first.transport_mode="isolated_mock";
  first.applied_at=rclcpp::Time(10010000000LL);
  auto heartbeat=old;heartbeat.sequence=2;ASSERT_TRUE(ledger.commit(heartbeat));
  // The latest heartbeat may precede receipt of the first actual writer ACK.
  ASSERT_TRUE(ledger.commitAckAt(first,10020000000LL));
  // Real graph replacements can share the reference/anchor while changing
  // only trajectory identity. Ordinary permits must not bypass writer CAS.
  f.spline.trajectory.traj_id=2;ledger.candidate(f.spline,f.proposal);
  w::ExecutionHandoffGrant g;g.schema_version=2;g.handoff_id="handoff";g.sequence=1;g.expected_commit_sequence=1;
  g.incumbent=heartbeat;g.candidate=heartbeat;g.candidate.version=f.proposal.version;g.candidate.trajectory_id=2;
  g.candidate.sequence=3;g.candidate.geometry_committed=false;g.source_stamp=rclcpp::Time(10020000000LL);
  g.valid_until=g.transition_deadline=rclcpp::Time(10300000000LL);
  ASSERT_TRUE(ledger.handoffAt(g,10020000000LL));EXPECT_EQ(ledger.committedTrajectory(),1);
  using A=scan_planner::ExecutionValidatorTestAccess;EXPECT_EQ(A::preparedCurve(ledger),2);
  auto changed=g;changed.candidate.trajectory_id=3;EXPECT_FALSE(ledger.handoffAt(changed,10030000000LL));
  f.spline.trajectory.traj_id=3;ledger.candidate(f.spline,f.proposal);ledger.cancelPending();EXPECT_EQ(A::preparedCurve(ledger),2);
  auto premature=g.candidate;premature.geometry_committed=true;EXPECT_FALSE(ledger.commit(premature));
  w::MotionValidation proof;proof.handoff_id=g.handoff_id;proof.sequence=30;proof.valid=true;
  proof.demand_sequence=7;proof.entry_admission_sequence=8;proof.demand_source_stamp=rclcpp::Time(10040000000LL);
  proof.demand_body_source_stamp=rclcpp::Time(10040000000LL);proof.entry_curve_time=.1;A::recordPrepared(ledger,proof);
  auto ack=first;ack.sequence=2;ack.handoff_id=g.handoff_id;ack.grant_sequence=1;
  ack.previous_commit_sequence=1;ack.commit_sequence=2;ack.incumbent_version=g.incumbent.version;
  ack.incumbent_trajectory_id=1;ack.candidate_version=g.candidate.version;ack.candidate_trajectory_id=2;
  ack.permit_sequence=3;ack.motion_validation_sequence=30;ack.demand_sequence=7;ack.entry_admission_sequence=8;
  ack.demand_source_stamp=proof.demand_source_stamp;ack.demand_body_source_stamp=proof.demand_body_source_stamp;
  ack.curve_time=.1;ack.applied_at=proof.demand_source_stamp;ack.valid_until=rclcpp::Time(10140000000LL);
  ASSERT_TRUE(ledger.commitAckAt(ack,10050000000LL));EXPECT_EQ(ledger.committedTrajectory(),2);
  EXPECT_EQ(A::writerSequence(ledger),2U);EXPECT_TRUE(A::latest(ledger)->geometry_committed);
  heartbeat.sequence=9;EXPECT_FALSE(ledger.commit(heartbeat));EXPECT_EQ(ledger.committedTrajectory(),2);
  ++ack.sequence;ack.write_acknowledged=true;EXPECT_TRUE(ledger.commitAckAt(ack,10200000000LL));
}
TEST(ExecutionLedger,LateInitialAckRestoresOnlyBoundedOriginalGeometryAndHoldsWithoutRenewingProof) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(const auto canceled:{false,true}) {
    Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
    ledger.candidate(f.spline,f.proposal);auto original=f.permit();original.allowed=true;
    original.execution_id="exec";original.control_epoch=original.sdk_arm_generation=1;original.sdk_session="sdk";
    original.source_stamp=rclcpp::Time(10000000000LL);original.valid_until=rclcpp::Time(10500000000LL);
    original.frame_id="odom";original.phase="tracking";ASSERT_TRUE(ledger.commit(original));
    auto next=original;next.sequence=2;
    if(canceled)next.revoked=true;
    else {f.spline.trajectory.traj_id=2;ledger.candidate(f.spline,f.proposal);next.trajectory_id=2;}
    ASSERT_TRUE(ledger.commit(next));
    w::ExecutionCommitAck a;a.schema_version=1;a.sequence=1;a.applied=a.write_submitted=true;
    a.commit_sequence=1;a.candidate_version=original.version;a.candidate_trajectory_id=1;
    a.execution_id=original.execution_id;a.control_epoch=a.sdk_arm_generation=1;a.sdk_session="sdk";
    a.permit_sequence=1;a.transport_mode="isolated_mock";a.applied_at=rclcpp::Time(10010000000LL);
    a.valid_until=rclcpp::Time(10110000000LL);
    if(canceled) {
      // Explicit task revocation is not a soft hold. A late physical fact
      // cannot restore geometry or task ownership in this retired ledger.
      EXPECT_FALSE(ledger.commitAckAt(a,10200000000LL));EXPECT_FALSE(ledger.committedView());
      ++a.sequence;a.write_acknowledged=true;
      EXPECT_FALSE(ledger.commitAckAt(a,10210000000LL));EXPECT_FALSE(ledger.committedView());
      next.revoked=false;next.trajectory_id=2;++next.sequence;EXPECT_FALSE(ledger.commit(next));
      continue;
    }
    ASSERT_TRUE(ledger.commitAckAt(a,10200000000LL));
    EXPECT_EQ(A::writerSequence(ledger),1U);EXPECT_EQ(ledger.committedTrajectory(),1);
    ASSERT_TRUE(A::latest(ledger));EXPECT_FALSE(A::latest(ledger)->allowed);EXPECT_EQ(A::latest(ledger)->phase,"holding");
    EXPECT_EQ(A::latest(ledger)->source_stamp,original.source_stamp);EXPECT_EQ(A::latest(ledger)->valid_until,original.valid_until);
    EXPECT_FALSE(A::activeProof(ledger));EXPECT_FALSE(A::activeProgress(ledger));
    const auto barrier=A::barrier(ledger);++a.sequence;a.write_acknowledged=true;
    ASSERT_TRUE(ledger.commitAckAt(a,10210000000LL));EXPECT_EQ(A::barrier(ledger),barrier);
    EXPECT_FALSE(A::latest(ledger)->allowed);EXPECT_EQ(ledger.committedTrajectory(),1);
    next.revoked=false;next.trajectory_id=2;++next.sequence;EXPECT_FALSE(ledger.commit(next));
  }
}
TEST(ExecutionLedger,InitialAckNeverGuessesMissingJobOrForeignWriterAndHistoryIsBounded) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(const auto variant:{0U,1U,2U}) {
    Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
    ledger.candidate(f.spline,f.proposal);auto p=f.permit();p.allowed=true;p.execution_id="exec";
    p.control_epoch=p.sdk_arm_generation=1;p.sdk_session="sdk";p.frame_id="odom";p.phase="tracking";
    p.source_stamp=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10500000000LL);
    if(variant==0) {auto missing=p;missing.trajectory_id=99;EXPECT_FALSE(ledger.commit(missing));}
    else ASSERT_TRUE(ledger.commit(p));
    if(variant==2)for(int id=2;id<=9;++id) {
      f.spline.trajectory.traj_id=id;ledger.candidate(f.spline,f.proposal);auto next=p;next.sequence=id;next.trajectory_id=id;
      ASSERT_TRUE(ledger.commit(next));
    }
    w::ExecutionCommitAck a;a.schema_version=1;a.sequence=1;a.applied=a.write_submitted=true;
    a.commit_sequence=1;a.candidate_version=p.version;a.candidate_trajectory_id=variant==0?99:1;
    a.execution_id=p.execution_id;a.control_epoch=a.sdk_arm_generation=1;a.sdk_session=variant==1?"foreign":"sdk";
    a.permit_sequence=1;a.transport_mode="isolated_mock";a.applied_at=rclcpp::Time(10010000000LL);
    a.valid_until=rclcpp::Time(10110000000LL);
    EXPECT_FALSE(ledger.commitAckAt(a,10020000000LL));EXPECT_EQ(A::writerSequence(ledger),0U);
  }
}
TEST(ExecutionLedger,ExactNegativeWriterFactReleasesPreparedButNeverAdvancesOrClearsIncumbent) {
  Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
  ledger.candidate(f.spline,f.proposal);auto old=f.permit();old.allowed=true;old.execution_id="exec";
  old.control_epoch=old.sdk_arm_generation=1;old.sdk_session="sdk";old.frame_id="odom";old.phase="tracking";
  old.source_stamp=rclcpp::Time(10000000000LL);old.valid_until=rclcpp::Time(10500000000LL);
  ASSERT_TRUE(ledger.commit(old));w::ExecutionCommitAck a;a.schema_version=1;a.sequence=1;
  a.applied=a.write_submitted=true;a.commit_sequence=1;a.candidate_version=old.version;a.candidate_trajectory_id=1;
  a.execution_id="exec";a.control_epoch=a.sdk_arm_generation=1;a.sdk_session="sdk";a.permit_sequence=1;
  a.transport_mode="isolated_mock";a.applied_at=rclcpp::Time(10010000000LL);
  ASSERT_TRUE(ledger.commitAckAt(a,10020000000LL));
  f.spline.trajectory.traj_id=2;ledger.candidate(f.spline,f.proposal);
  w::ExecutionHandoffGrant g;g.schema_version=2;g.handoff_id="handoff";g.sequence=1;g.expected_commit_sequence=1;
  g.incumbent=old;g.candidate=old;g.candidate.sequence=2;g.candidate.trajectory_id=2;g.candidate.geometry_committed=false;
  g.source_stamp=rclcpp::Time(10020000000LL);g.valid_until=g.transition_deadline=rclcpp::Time(10300000000LL);
  ASSERT_TRUE(ledger.handoffAt(g,10020000000LL));
  a.sequence=2;a.handoff_id=g.handoff_id;a.grant_sequence=1;a.previous_commit_sequence=a.commit_sequence=1;
  a.incumbent_version=old.version;a.incumbent_trajectory_id=1;a.candidate_trajectory_id=2;
  a.applied=a.write_submitted=a.write_acknowledged=false;
  a.permit_sequence=0;EXPECT_FALSE(ledger.commitAckAt(a,10310000000LL));
  a.permit_sequence=2;auto advanced=a;advanced.commit_sequence=2;
  EXPECT_FALSE(ledger.commitAckAt(advanced,10310000000LL));
  auto uncertain=a;uncertain.write_submitted=true;EXPECT_FALSE(ledger.commitAckAt(uncertain,10310000000LL));
  ASSERT_TRUE(ledger.commitAckAt(a,10310000000LL));
  using A=scan_planner::ExecutionValidatorTestAccess;
  EXPECT_EQ(A::writerSequence(ledger),1U);EXPECT_EQ(ledger.committedTrajectory(),1);
  auto heartbeat=old;heartbeat.sequence=3;ASSERT_TRUE(ledger.commit(heartbeat));
  auto retry=g;retry.handoff_id="retry";retry.sequence=2;retry.source_stamp=rclcpp::Time(10310000000LL);
  retry.valid_until=retry.transition_deadline=rclcpp::Time(10450000000LL);
  ASSERT_TRUE(ledger.handoffAt(retry,10310000000LL));
}
TEST(ExecutionLedger,PreparedZeroStillRequiresFreshExactEntryAndStrictBoundary) {
  using A=scan_planner::ExecutionValidatorTestAccess;Fixture f;w::ExecutionHandoffGrant g;
  g.schema_version=2;g.handoff_id="h";g.sequence=1;g.expected_commit_sequence=1;g.candidate=f.permit();
  auto& p=g.candidate;p.allowed=true;p.geometry_committed=false;p.frame_id="odom";p.phase="tracking";
  p.valid_until=rclcpp::Time(10500000000LL);g.valid_until=g.transition_deadline=p.valid_until;
  w::PreparedMotionDemand x;x.schema_version=1;x.handoff_id="h";x.grant_sequence=1;x.expected_commit_sequence=1;
  auto& d=x.demand;d.version=p.version;d.execution_id=p.execution_id;d.control_epoch=p.control_epoch;
  d.sdk_session=p.sdk_session;d.sdk_arm_generation=p.sdk_arm_generation;d.permit_sequence=p.sequence;
  d.trajectory_id=p.trajectory_id;d.transport_mode=p.transport_mode;d.source_stamp=d.body_source_stamp=rclcpp::Time(10000000000LL);
  d.valid_until=rclcpp::Time(10100000000LL);d.hold=true;
  auto& a=x.entry_admission;a.sequence=1;a.version=d.version;a.trajectory_id=d.trajectory_id;
  a.body_source_stamp=d.body_source_stamp;a.checked_at=d.source_stamp;a.valid_until=d.valid_until;a.accepted=true;a.transport_mode=d.transport_mode;
  x.entry_source_stamp=d.body_source_stamp;x.measured_pose.header.stamp=x.entry_source_stamp;x.measured_pose.header.frame_id="odom";
  x.position_tolerance_m=.0125;x.velocity_tolerance_mps=.05;
  EXPECT_TRUE(A::entry(x,g,10010000000LL));
  for(int variant=0;variant<7;++variant){auto bad=x;
    if(variant==0)bad.entry_source_stamp=rclcpp::Time(9990000000LL);
    if(variant==1)bad.entry_admission.sequence=0;if(variant==2)bad.position_tolerance_m=.02;
    if(variant==3)bad.velocity_tolerance_mps=.1;if(variant==4)bad.position_error_m=.0126;
    if(variant==5)bad.measured_pose.header.frame_id="map";if(variant==6)bad.entry_admission.accepted=false;
    EXPECT_FALSE(A::entry(bad,g,10010000000LL))<<variant;}
  EXPECT_FALSE(A::entry(x,g,10100000000LL));
  // Historical entry/command survives only its configured original budget.
  // This is structural matching, not a new collision or current-body proof.
  x.demand.valid_until=x.entry_admission.valid_until=rclcpp::Time(10200000000LL);
  EXPECT_FALSE(A::entry(x,g,10125000000LL));
  EXPECT_TRUE(A::entry(x,g,10125000000LL,200000000LL));
  EXPECT_FALSE(A::entry(x,g,10200000001LL,200000000LL));
  x.entry_admission.valid_until=rclcpp::Time(10124000000LL);
  EXPECT_FALSE(A::entry(x,g,10125000000LL,200000000LL));
}
TEST(ExecutionLedger,StationaryReentryPinsSdkZeroAndMcEvidenceWithoutGrantingIncumbentMotion) {
  for(int variant=0;variant<10;++variant) {
    Fixture f;scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom","isolated_mock",true};
    ledger.candidate(f.spline,f.proposal);auto old=f.permit();old.allowed=true;old.execution_id="exec";
    old.control_epoch=old.sdk_arm_generation=1;old.sdk_session="sdk";old.frame_id="odom";old.phase="tracking";
    old.source_stamp=rclcpp::Time(10000000000LL);old.valid_until=rclcpp::Time(10500000000LL);ASSERT_TRUE(ledger.commit(old));
    w::ExecutionCommitAck first;first.schema_version=1;first.sequence=1;first.applied=first.write_submitted=true;
    first.commit_sequence=1;first.candidate_version=old.version;first.candidate_trajectory_id=1;
    first.execution_id="exec";first.control_epoch=first.sdk_arm_generation=1;first.sdk_session="sdk";
    first.permit_sequence=1;first.transport_mode="isolated_mock";first.applied_at=rclcpp::Time(10010000000LL);
    ASSERT_TRUE(ledger.commitAckAt(first,10020000000LL));
    auto holding=old;holding.allowed=false;holding.phase="holding";holding.sequence=2;ASSERT_TRUE(ledger.commit(holding));
    f.spline.trajectory.traj_id=2;ledger.candidate(f.spline,f.proposal);
    w::ExecutionHandoffGrant g;g.schema_version=2;g.handoff_id="reentry";g.sequence=1;g.expected_commit_sequence=1;
    g.transition_mode=w::ExecutionHandoffGrant::STATIONARY_REENTRY;g.incumbent=holding;g.candidate=old;
    g.candidate.sequence=3;g.candidate.trajectory_id=2;g.candidate.geometry_committed=false;
    g.source_stamp=rclcpp::Time(10040000000LL);g.valid_until=g.transition_deadline=rclcpp::Time(10240000000LL);
    g.retain_incumbent_until=g.source_stamp;g.candidate.source_stamp=g.source_stamp;g.candidate.valid_until=g.valid_until;
    auto& e=g.stationary_evidence;e.schema_version=1;e.version=old.version;e.execution_id="exec";
    e.control_epoch=e.sdk_arm_generation=1;e.sdk_session="sdk";e.transport_mode="isolated_mock";
    e.sequence=e.zero_write_sequence=e.writer_commit_sequence=1;e.applied_trajectory_id=1;
    e.zero_ack_at=rclcpp::Time(9400000000LL);e.mc_raw_stamp_ns=900;e.mc_clock_epoch="clock";
    e.time_basis="isolated_simulated_source_clock";e.source_stamp=e.received_stamp=g.source_stamp;
    e.capture_lower_bound=rclcpp::Time(10030000000LL);e.capture_upper_bound=g.source_stamp;e.valid_until=g.valid_until;
    e.mc_capture_delay_bound_sec=.02;
    e.stationary_samples=3;e.stationary_duration_sec=.64;e.nonzero_blocked=e.usable=true;
    if(variant==1)g.schema_version=1;if(variant==2)e.usable=false;
    if(variant==3)e.zero_write_sequence=0;if(variant==4)e.capture_lower_bound=e.zero_ack_at;
    if(variant==5)e.sdk_session="foreign";if(variant==6)g.incumbent.allowed=true;
    if(variant==7)e.valid_until=rclcpp::Time(10030000000LL);if(variant==8)e.nonzero_blocked=false;
    if(variant==9)g.candidate.valid_until=rclcpp::Time(10250000000LL);
    EXPECT_EQ(ledger.handoffAt(g,10040000000LL),variant==0)<<variant;
    EXPECT_EQ(ledger.committedTrajectory(),1);EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::latest(ledger)->allowed);
  }
}
TEST(ExecutionLedger,StationaryMcClockUsesOriginalReceiptAndBoundedCaptureInterval) {
  Fixture f;
  for(int variant=0;variant<13;++variant) {
    const std::string mode=variant==12?"isolated_mock":"live";
    scan_planner::ExecutionValidator ledger{scan_planner::ExecutionValidator::PassiveLedger{},f.pool,"odom",mode,true};
    w::ExecutionHandoffGrant g;g.transition_mode=w::ExecutionHandoffGrant::STATIONARY_REENTRY;
    auto& p=g.incumbent;p.version=f.permit().version;
    p.execution_id="exec";p.sdk_session="sdk";p.control_epoch=p.sdk_arm_generation=1;
    p.transport_mode=mode;p.trajectory_id=1;p.geometry_committed=true;p.phase="holding";
    g.expected_commit_sequence=1;g.source_stamp=g.retain_incumbent_until=rclcpp::Time(11000000000LL);
    g.valid_until=g.candidate.valid_until=rclcpp::Time(11200000000LL);
    auto& e=g.stationary_evidence;e.schema_version=1;e.version=p.version;e.execution_id=p.execution_id;
    e.control_epoch=e.sdk_arm_generation=1;e.sdk_session=p.sdk_session;e.transport_mode=mode;
    e.sequence=e.zero_write_sequence=e.writer_commit_sequence=1;e.applied_trajectory_id=1;
    e.mc_raw_stamp_ns=900;e.mc_clock_epoch="sdk:0";
    e.time_basis=mode=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock";
    e.source_stamp=rclcpp::Time(11060000000LL);e.received_stamp=e.capture_upper_bound=rclcpp::Time(11000000000LL);
    e.capture_lower_bound=rclcpp::Time(10750000000LL);e.zero_ack_at=rclcpp::Time(10100000000LL);
    e.mc_capture_delay_bound_sec=.25;e.valid_until=g.valid_until;
    e.stationary_samples=35;e.stationary_duration_sec=.65;e.nonzero_blocked=e.usable=true;
    if(variant==1)e.source_stamp=rclcpp::Time(11260000000LL);
    if(variant==2)e.received_stamp=rclcpp::Time(11020000000LL);
    if(variant==3)e.source_stamp=e.received_stamp=e.capture_upper_bound=rclcpp::Time(10600000000LL);
    if(variant==4)e.capture_lower_bound=e.zero_ack_at;
    if(variant==5)e.capture_upper_bound=rclcpp::Time(11010000000LL);
    if(variant==6)e.mc_capture_delay_bound_sec=.26;
    if(variant==7)e.time_basis="unknown";
    if(variant==8)e.valid_until=g.valid_until=g.candidate.valid_until=rclcpp::Time(11300000000LL);
    if(variant==9)e.capture_lower_bound=rclcpp::Time(11010000000LL);
    if(variant==10)e.mc_capture_delay_bound_sec=std::numeric_limits<double>::quiet_NaN();
    if(variant==11)e.valid_until=g.valid_until=g.candidate.valid_until=rclcpp::Time(11000000000LL);
    EXPECT_EQ(scan_planner::ExecutionValidatorTestAccess::stationaryMode(ledger,g,11000000000LL),variant==0)<<variant;
  }
}
TEST(ExecutionLedger,PreparedSweepComesAfterIncumbentRenewalAndSharesFixedRoundBudget) {
  using C=scan_planner::ValidationCycle;const auto begin=C::Clock::time_point{};auto now=begin;
  std::vector<std::string> order;
  C::priorityPassWithPrepared(begin,[&](double){order.push_back("current_sweep");now+=std::chrono::milliseconds(2);},
    [&](double budget){order.push_back("current_curve");EXPECT_LE(budget,.041);now+=std::chrono::milliseconds(30);return true;},
    [&](double budget){order.push_back("prepared_sweep");EXPECT_LE(budget,.005);now+=std::chrono::milliseconds(5);},[&]{return now;});
  EXPECT_EQ(order,(std::vector<std::string>{"current_sweep","current_curve","current_sweep","prepared_sweep"}));
  EXPECT_LT(now,begin+C::period);
  order.clear();now=begin;
  C::priorityPassWithPrepared(begin,[&](double){order.push_back("current_sweep");now=begin+std::chrono::milliseconds(51);},
    [&](double){ADD_FAILURE()<<"expired round must not renew";return true;},
    [&](double){ADD_FAILURE()<<"expired round must not prepare";},[&]{return now;});
  EXPECT_EQ(order,(std::vector<std::string>{"current_sweep"}));
}
TEST(ExecutionLedger,CandidateAndDebugNeverReplaceCommittedOwner) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);EXPECT_FALSE(f.ledger.committedView());
  w::LocalPlanDebug d;d.generation=1;d.plan_id=1;d.valid=true;f.ledger.candidateDebug(d);
  ASSERT_TRUE(f.ledger.commit(f.permit()));
  f.spline.trajectory.traj_id=2;f.ledger.candidate(f.spline,f.proposal);d.plan_id=2;f.ledger.candidateDebug(d);
  auto active=f.ledger.committedView();ASSERT_TRUE(active&&active->debug);
  EXPECT_EQ(active->spline.trajectory.traj_id,1);EXPECT_EQ(active->debug->plan_id,1U);
  f.ledger.cancelPending();EXPECT_EQ(f.ledger.committedTrajectory(),1);
}
TEST(ExecutionLedger,RunningHorizonIgnoresUncommittedCandidateAndItsProgress) {
  CommittedIdleFixture f;
  f.spline.trajectory.traj_id=2;f.spline.trajectory.pos_pts.back().x+=10.;
  f.ledger.candidate(f.spline,f.proposal);
  auto candidate_progress=f.measured;candidate_progress.trajectory_id=2;
  candidate_progress.curve_time=3.;candidate_progress.header.stamp=rclcpp::Time(f.now+10000000LL);
  f.ledger.progressAt(candidate_progress,(f.now+10000000LL)*1e-9);
  const auto horizon=f.ledger.committedHorizonAt(f.proposal.version,f.now+10000000LL);
  ASSERT_TRUE(horizon);EXPECT_EQ(horizon->trajectory_id,1);
  EXPECT_DOUBLE_EQ(horizon->measured_time,.1);EXPECT_EQ(horizon->measured_source_ns,f.now);
  EXPECT_NEAR(horizon->curve.evaluateDeBoorT(horizon->duration).x(),
    f.curve.evaluateDeBoorT(f.curve.getTimeSum()).x(),1e-12);
}
TEST(ExecutionLedger,RunningHorizonKeepsOldAnchorUntilRealCommitButRejectsAnotherTask) {
  CommittedIdleFixture f;auto updated=f.proposal.version;
  ++updated.reference_generation;++updated.anchor_revision;updated.anchor_id="new_anchor";
  const auto horizon=f.ledger.committedHorizonAt(updated,f.now);
  ASSERT_TRUE(horizon);EXPECT_EQ(horizon->version,f.proposal.version);
  updated.task_id="another_task";EXPECT_FALSE(f.ledger.committedHorizonAt(updated,f.now));
  EXPECT_FALSE(f.ledger.committedHorizonAt(f.proposal.version,f.now+400000001LL));
  auto revoked=f.permit(2);revoked.revoked=true;ASSERT_TRUE(f.ledger.commit(revoked));
  EXPECT_FALSE(f.ledger.committedHorizonAt(f.proposal.version,f.now));
}
TEST(ExecutionLedger,SlowProofCannotOverwriteNewerSourceMapOrNegativeEvidence) {
  w::TrajectoryValidation current,incoming;
  current.sequence=10;current.map_snapshot_revision=20;current.body_source_stamp=rclcpp::Time(10000000000LL);
  current.valid=false;incoming=current;incoming.valid=true;
  using A=scan_planner::ExecutionValidatorTestAccess;
  EXPECT_FALSE(A::supersedes(incoming,current));
  incoming.sequence=11;incoming.map_snapshot_revision=19;EXPECT_FALSE(A::supersedes(incoming,current));
  incoming.map_snapshot_revision=20;incoming.body_source_stamp=rclcpp::Time(9999999999LL);
  EXPECT_FALSE(A::supersedes(incoming,current));
  incoming.body_source_stamp=current.body_source_stamp;EXPECT_TRUE(A::supersedes(incoming,current));
  // Validity itself never overrides chronological source / snapshot evidence.
}
TEST(ExecutionLedger,ExpiredOlderRayRefreshCannotReplaceUnexpiredFreshProofAtAnyNewerRevision) {
  for(const bool newer_revision:{false,true}) {
  using A=scan_planner::ExecutionValidatorTestAccess;CommittedIdleFixture f;
  auto current=f.proof;current.sequence=10;current.map_snapshot_revision=20;
  current.check_end=rclcpp::Time(10310000000LL);current.valid_until=rclcpp::Time(10550000000LL);
  current.body_source_stamp=rclcpp::Time(10300000000LL);
  current.front_ray_source_stamp=current.rear_ray_source_stamp=rclcpp::Time(10200000000LL);
  ASSERT_TRUE(A::recordCommitted(f.ledger,current));
  auto slow=current;slow.sequence=11;slow.valid=false;slow.reason="source_expired_during_check";
  slow.body_source_stamp=rclcpp::Time(10350000000LL);slow.check_end=rclcpp::Time(10360000000LL);
  slow.front_ray_source_stamp=slow.rear_ray_source_stamp=rclcpp::Time(9800000000LL);
  if(newer_revision)++slow.map_snapshot_revision;
  slow.valid_until=rclcpp::Time(10300000000LL);
  EXPECT_FALSE(A::recordCommitted(f.ledger,slow));
  ASSERT_TRUE(f.ledger.committedView()->proof);
  EXPECT_TRUE(f.ledger.committedView()->proof->valid);
  EXPECT_EQ(f.ledger.committedView()->proof->valid_until,current.valid_until);
  // No extension: once the original positive expires, an invalid refresh is
  // not suppressed merely because its immutable map slot is older.
  slow.check_end=current.valid_until;EXPECT_TRUE(A::recordCommitted(f.ledger,slow));
  EXPECT_FALSE(f.ledger.committedView()->proof->valid);
  }
}
TEST(ExecutionLedger,RayRefreshOrderingNeverSuppressesCollisionOrNewGeometryInvalidity) {
  using A=scan_planner::ExecutionValidatorTestAccess;w::TrajectoryValidation current;
  current.sequence=10;current.map_snapshot_revision=20;current.valid=true;
  current.body_source_stamp=rclcpp::Time(10300000000LL);current.valid_until=rclcpp::Time(10550000000LL);
  current.front_ray_source_stamp=current.rear_ray_source_stamp=rclcpp::Time(10200000000LL);
  auto incoming=current;incoming.sequence=11;incoming.valid=false;incoming.check_end=rclcpp::Time(10360000000LL);
  incoming.front_ray_source_stamp=incoming.rear_ray_source_stamp=rclcpp::Time(9800000000LL);
  for(const auto* reason:{"occupied","unknown_or_unobserved","measured_connection_support_invalid"}) {
    incoming.reason=reason;EXPECT_TRUE(A::supersedes(incoming,current))<<reason;
  }
  incoming.reason="source_expired_during_check";++incoming.map_snapshot_revision;
  EXPECT_FALSE(A::supersedes(incoming,current)); // No geometry was witnessed from expired older rays.
  --incoming.map_snapshot_revision;incoming.front_ray_source_stamp=rclcpp::Time(10210000000LL);
  EXPECT_TRUE(A::supersedes(incoming,current)); // Mixed/new observations are not an old refresh.
  incoming.front_ray_source_stamp=incoming.rear_ray_source_stamp=current.front_ray_source_stamp;
  EXPECT_TRUE(A::supersedes(incoming,current)); // Actual same-source expiry still fences.
}
TEST(ExecutionLedger,PositiveProofWithRegressedRaySourceCannotReviveNegativeGeometry) {
  using A=scan_planner::ExecutionValidatorTestAccess;w::TrajectoryValidation current;
  current.sequence=10;current.map_snapshot_revision=20;current.valid=false;current.reason="occupied";
  current.body_source_stamp=rclcpp::Time(10300000000LL);
  current.front_ray_source_stamp=current.rear_ray_source_stamp=rclcpp::Time(10200000000LL);
  auto incoming=current;incoming.sequence=11;incoming.valid=true;
  incoming.body_source_stamp=rclcpp::Time(10350000000LL);
  incoming.rear_ray_source_stamp=rclcpp::Time(10199999999LL);
  EXPECT_FALSE(A::supersedes(incoming,current));
  incoming.rear_ray_source_stamp=current.rear_ray_source_stamp;EXPECT_TRUE(A::supersedes(incoming,current));
}
TEST(ExecutionLedger,NativeReaderOldSourceAtSameOrNewerRevisionCannotInvalidateFreshProof) {
  for(const bool newer_revision:{false,true}) {
  using A=scan_planner::ExecutionValidatorTestAccess;PreviewCandidateFixture f;
  A::officialGeometry(f.ledger);f.ledger.body(f.body,geometry_msgs::msg::Twist{},10300000000LL);
  auto old=GridMapTestAccess::headingFixture(false,true);
  GridMapTestAccess::acquiredSnapshot(*old,9800000000LL,10310000000LL);
  GridMap current;old->copyCollisionSnapshotTo(current,10310000000LL,std::chrono::steady_clock::now());
  GridMapTestAccess::acquiredSnapshot(current,10200000000LL,10310000000LL);
  if(newer_revision)old->setOccupied({.875,1.1,.55});
  if(newer_revision)ASSERT_LT(current.occupancyRevision(),old->occupancyRevision());
  else ASSERT_EQ(current.occupancyRevision(),old->occupancyRevision());
  ASSERT_NE(current.integratedRaySourceStamp(0),old->integratedRaySourceStamp(0));
  // A slow reader may already own the third slot when a newer head completes.
  // Admission is now forbidden from NEWLY borrowing an obsolete released
  // solver input behind that head. Acquire the honest in-flight old lease
  // first; its late result must still not overwrite the newer native proof.
  ASSERT_TRUE(f.pool.publishSolver(*old,10310000000LL,std::chrono::steady_clock::now()));
  auto solver=f.pool.borrowSolver();ASSERT_TRUE(solver);
  EXPECT_FALSE(f.pool.publishSolver(current,10310000000LL,std::chrono::steady_clock::now()));
  solver.reset();auto slow=f.pool.borrowAdmission();ASSERT_TRUE(slow);
  ASSERT_TRUE(f.pool.publishValidation(current,10310000000LL,std::chrono::steady_clock::now()));
  EXPECT_FALSE(f.pool.publishSolver(current,10310000000LL,std::chrono::steady_clock::now()));
  auto fast=f.pool.borrowValidation();ASSERT_TRUE(fast);
  auto good=A::checkWithClock(f.ledger,*fast,[]{return 10310000000LL;});
  ASSERT_TRUE(good);ASSERT_TRUE(good->valid)<<good->reason;
  auto permit=f.permit();permit.phase="preview";ASSERT_TRUE(f.ledger.commit(permit));
  if(newer_revision)ASSERT_LT(fast->occupancyRevision(),slow->occupancyRevision());
  else ASSERT_EQ(fast->occupancyRevision(),slow->occupancyRevision());
  ASSERT_GT(fast->integratedRaySourceStamp(0),slow->integratedRaySourceStamp(0));
  f.body.source_stamp=10.35;f.ledger.body(f.body,geometry_msgs::msg::Twist{},10350000000LL);
  // The in-flight slower reader completes later using its unchanged map.
  auto obsolete=A::checkWithClock(f.ledger,*slow,[]{return 10360000000LL;},true);
  EXPECT_FALSE(obsolete); // It must not publish an empty/expired source as a new collision.
  ASSERT_TRUE(f.ledger.committedView()->proof);
  EXPECT_EQ(f.ledger.committedView()->proof->sequence,good->sequence);
  EXPECT_EQ(f.ledger.committedView()->proof->valid_until,good->valid_until);
  // The same production map, once it carries a genuinely fresh acquisition,
  // must expose its occupied geometry. No weak source-expiry reason masks it.
  if(newer_revision) {
    slow.reset();GridMapTestAccess::acquiredSnapshot(*old,10320000000LL,10360000000LL);
    ASSERT_TRUE(f.pool.publishSolver(*old,10360000000LL,std::chrono::steady_clock::now()));
    auto fresh_occupied=f.pool.borrowAdmission();ASSERT_TRUE(fresh_occupied);
    auto danger=A::checkWithClock(f.ledger,*fresh_occupied,[]{return 10360000000LL;},true);
    ASSERT_TRUE(danger);EXPECT_FALSE(danger->valid);EXPECT_EQ(danger->reason,"occupied");
    ASSERT_TRUE(f.ledger.committedView()->proof);EXPECT_FALSE(f.ledger.committedView()->proof->valid);
  }
  }
}
TEST(ExecutionLedger,NativeOccupiedResultSurvivesSourceExpiryDuringFinalization) {
  using A=scan_planner::ExecutionValidatorTestAccess;PreviewCandidateFixture f;
  A::officialGeometry(f.ledger);f.ledger.body(f.body,geometry_msgs::msg::Twist{},10300000000LL);
  auto map=GridMapTestAccess::headingFixture(false,true);
  GridMapTestAccess::acquiredSnapshot(*map,10000000000LL,10310000000LL);
  map->setOccupied({.875,1.1,.55});
  // Geometry is queried with fresh sources; only the final source/deadline
  // recheck crosses the old acquisition's expiry. Occupied is still observed.
  unsigned calls=0;
  auto result=A::checkWithClock(f.ledger,*map,[&]{return ++calls<=5?10310000000LL:10600000000LL;});
  ASSERT_TRUE(result);EXPECT_FALSE(result->valid);EXPECT_EQ(result->reason,"occupied");
}
TEST(ExecutionLedger,FastLaneChecksLatestMotionBeforeIncumbentAndNeverRunsCandidate) {
  const auto begin=scan_planner::ValidationCycle::Clock::now();std::vector<std::string> order;
  scan_planner::ValidationCycle::priorityPass(begin,[&]{order.push_back("motion");},
    [&](double budget){order.push_back("active");EXPECT_GT(budget,0.);EXPECT_LE(budget,.050);});
  EXPECT_EQ(order,(std::vector<std::string>{"motion","active"}));
  EXPECT_EQ(scan_planner::ValidationCycle::activeBudget(begin,begin+std::chrono::milliseconds(55)),0.);
  order.clear();
  scan_planner::ValidationCycle::priorityPass(begin-std::chrono::seconds(1),[&]{order.push_back("motion");},
    [&](double){order.push_back("active");});
  EXPECT_EQ(order,(std::vector<std::string>{"motion"}));
}

TEST(ExecutionLedger,PositiveCurrentRenewalRechecksLatestDemandWithinSameRoundAtMostOnce) {
  using Cycle=scan_planner::ValidationCycle;
  const auto begin=Cycle::Clock::time_point{};auto now=begin;
  std::vector<std::string> order;std::vector<double> motion_budgets;
  const auto count=Cycle::priorityPassWithRenewal(begin,[&](double budget){
      order.push_back("motion");motion_budgets.push_back(budget);now+=std::chrono::milliseconds(1);
    },[&](double budget){
      order.push_back("published_positive_current_renewal");
      EXPECT_NEAR(budget,.044,1e-9);now+=std::chrono::milliseconds(42);return true;
    },[&]{return now;});
  EXPECT_EQ(count,2U);
  EXPECT_EQ(order,(std::vector<std::string>{"motion","published_positive_current_renewal","motion"}));
  EXPECT_DOUBLE_EQ(motion_budgets[0],.025);
  EXPECT_NEAR(motion_budgets[1],.007,1e-9); // original 50 ms end, not another 25 ms.
  EXPECT_LT(now-begin,Cycle::period);
}

TEST(ExecutionLedger,FailedExpiredRetiredOrOverBudgetRenewalCannotRequestPositiveMotionRetry) {
  using Cycle=scan_planner::ValidationCycle;
  for(const auto* reason:{"occupied","source_expired_during_check","retired_job","curve_check_budget_exhausted"}) {
    const auto begin=Cycle::Clock::time_point{};auto now=begin;unsigned motions=0;
    EXPECT_EQ(Cycle::priorityPassWithRenewal(begin,[&](double){++motions;},
      [&](double){now+=std::chrono::milliseconds(10);return false;},[&]{return now;}),1U)<<reason;
    EXPECT_EQ(motions,1U)<<reason; // actual check() returns false for all these dispositions.
  }
  const auto begin=Cycle::Clock::time_point{};auto now=begin;unsigned motions=0;
  EXPECT_EQ(Cycle::priorityPassWithRenewal(begin,[&](double){++motions;},
    [&](double){now=begin+std::chrono::milliseconds(51);return true;},[&]{return now;}),1U);
  EXPECT_EQ(motions,1U); // even a late positive cannot obtain a renewed CPU deadline.
}

TEST(ExecutionLedger,CurrentBudgetMissRetainsOnlyOriginalLeaseAndNeverSuppressesActualOccupied) {
  using Cycle=scan_planner::ValidationCycle;
  const std::int64_t original_deadline=1250000000LL;
  EXPECT_TRUE(Cycle::inconclusiveRefresh(true,"curve_check_budget_exhausted"));
  EXPECT_TRUE(Cycle::originalLeaseUsable(true,original_deadline,1200000000LL));
  EXPECT_FALSE(Cycle::originalLeaseUsable(true,original_deadline,1250000000LL));
  EXPECT_FALSE(Cycle::originalLeaseUsable(true,original_deadline,1300000000LL));
  EXPECT_FALSE(Cycle::originalLeaseUsable(false,original_deadline,1200000000LL));
  // Candidate budget failure is never an admission, and real new occupied /
  // unknown / expired-source evidence is published even for the incumbent.
  EXPECT_FALSE(Cycle::inconclusiveRefresh(false,"curve_check_budget_exhausted"));
  EXPECT_FALSE(Cycle::inconclusiveRefresh(true,"occupied"));
  EXPECT_FALSE(Cycle::inconclusiveRefresh(true,"unknown_or_unobserved"));
  EXPECT_FALSE(Cycle::inconclusiveRefresh(true,"source_expired_during_check"));
  EXPECT_EQ(original_deadline,1250000000LL); // No completion-time extension.
}
TEST(ExecutionLedger,BlockedEntryRequiresExactNativeOccupiedProofAndNeverGrantsMotion) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  w::MotionValidation p;p.version=f.proposal.version;p.trajectory_id=1;p.sequence=5;
  p.reason="motion_sweep_occupied";p.velocity.linear.x=.01;
  p.check_end=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10090000000LL);
  constexpr std::int64_t now=10010000000LL;
  EXPECT_FALSE(f.ledger.submitBlockedEntryAt(p,now)); // Not a native published proof.
  scan_planner::ExecutionValidatorTestAccess::record(f.ledger,p);
  for(int mode=0;mode<5;++mode) {
    auto bad=p;
    if(mode==0)bad.velocity.linear.x=.02;
    if(mode==1)bad.version.task_id="other";
    if(mode==2)bad.valid=true;
    if(mode==3)bad.reason="motion_sweep_unknown_or_expired";
    EXPECT_FALSE(f.ledger.submitBlockedEntryAt(bad,mode==4?10090000000LL:now));
  }
  ASSERT_TRUE(f.ledger.submitBlockedEntryAt(p,now));
  EXPECT_FALSE(f.ledger.submitBlockedEntryAt(p,now)); // One event, no periodic replan loop.
  auto event=f.ledger.consumeBlockedEntry();ASSERT_TRUE(event);EXPECT_EQ(*event,p);
  EXPECT_FALSE(f.ledger.consumeBlockedEntry());
  EXPECT_EQ(f.ledger.committedTrajectory(),1); // Task/curve is not cleared or replaced.
  EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::latest(f.ledger)->allowed);
}
TEST(ExecutionLedger,CommittedOccupiedProofRequestsReplanWithoutNonzeroDemandOrTrackerEcho) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  CommittedObstacleFixture f;ASSERT_TRUE(A::recordCommitted(f.ledger,f.proof));
  const auto event=f.ledger.consumeCommittedCollisionAt(10010000000LL);
  ASSERT_TRUE(event);EXPECT_EQ(*event,f.proof);
  EXPECT_FALSE(f.ledger.consumeBlockedEntry()); // No fabricated motion proof.
  EXPECT_FALSE(f.ledger.consumeCommittedCollisionAt(10020000000LL));
  EXPECT_EQ(f.ledger.committedTrajectory(),1);EXPECT_FALSE(A::latest(f.ledger)->allowed);
  ASSERT_TRUE(f.ledger.committedView()->proof);EXPECT_FALSE(f.ledger.committedView()->proof->valid);
  EXPECT_EQ(rclcpp::Time(event->valid_until).nanoseconds(),10250000000LL);
}
TEST(ExecutionLedger,PendingOccupiedWakeSurvivesPureSourceExpiryWithoutMotionAuthority) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  CommittedObstacleFixture f;ASSERT_TRUE(A::recordCommitted(f.ledger,f.proof));
  auto expired=f.proof;++expired.sequence;expired.reason="source_expired_during_check";
  expired.check_end=rclcpp::Time(10050000000LL);expired.valid_until=rclcpp::Time(10040000000LL);
  ASSERT_TRUE(A::recordCommitted(f.ledger,expired));
  ASSERT_TRUE(f.ledger.committedView()->proof);
  EXPECT_EQ(*f.ledger.committedView()->proof,expired); // Latest execution verdict remains invalid.
  const auto event=f.ledger.consumeCommittedCollisionAt(10060000000LL);
  ASSERT_TRUE(event);EXPECT_EQ(*event,f.proof); // Exact original collision, not a renewed lease.
  EXPECT_FALSE(A::latest(f.ledger)->allowed);EXPECT_FALSE(f.ledger.consumeBlockedEntry());
  EXPECT_FALSE(f.ledger.consumeCommittedCollisionAt(10070000000LL));
  for(unsigned i=0;i<3;++i) {
    auto repeated=f.proof;repeated.sequence=3+i*2;
    repeated.check_end=rclcpp::Time(10080000000LL+i*30000000LL);
    ASSERT_TRUE(A::recordCommitted(f.ledger,repeated));
    expired.sequence=repeated.sequence+1;expired.check_end=repeated.check_end;
    ASSERT_TRUE(A::recordCommitted(f.ledger,expired));
    EXPECT_FALSE(f.ledger.consumeCommittedCollisionAt(10090000000LL+i*30000000LL));
  }
}
TEST(ExecutionLedger,PendingCollisionAfterSourceExpiryKeepsOriginalLifetimeAndOwnership) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(unsigned variant=0;variant<8;++variant) {
    SCOPED_TRACE(variant);CommittedObstacleFixture f;
    ASSERT_TRUE(A::recordCommitted(f.ledger,f.proof));
    auto expired=f.proof;expired.sequence=2;expired.reason="source_expired_during_check";
    expired.check_end=rclcpp::Time(10050000000LL);expired.valid_until=rclcpp::Time(10040000000LL);
    ASSERT_TRUE(A::recordCommitted(f.ledger,expired));
    auto now=10060000000LL;
    if(variant==0)now=10250000000LL; // Collision's own deadline, not the newer negative's.
    if(variant==1){auto clear=f.proof;clear.sequence=3;clear.valid=true;clear.reason="observed_free";
      clear.check_end=rclcpp::Time(10055000000LL);ASSERT_TRUE(A::recordCommitted(f.ledger,clear));}
    if(variant==2){auto revoke=f.permit(2);revoke.revoked=true;ASSERT_TRUE(f.ledger.commit(revoke));}
    if(variant==3){f.spline.trajectory.traj_id=2;f.ledger.candidate(f.spline,f.proposal);
      ASSERT_TRUE(f.ledger.commit(f.permit(2)));}
    if(variant==4){auto revoke=f.permit(2);revoke.revoked=true;
      ASSERT_TRUE(f.ledger.commit(revoke));f.proposal.version.task_id="new_task";
      f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit(3)));}
    if(variant>=5){auto other=expired;++other.sequence;
      if(variant==5)other.reason="measured_projection_or_connection_invalid";
      if(variant==6)other.reason="unknown_or_unobserved";
      if(variant==7)other.support_hash="changed_support";
      ASSERT_TRUE(A::recordCommitted(f.ledger,other));}
    EXPECT_FALSE(f.ledger.consumeCommittedCollisionAt(now));
  }
  CommittedObstacleFixture f;ASSERT_TRUE(A::recordCommitted(f.ledger,f.proof));
  auto expired=f.proof;expired.sequence=2;expired.reason="source_expired_during_check";
  expired.check_end=f.proof.valid_until;
  ASSERT_TRUE(A::recordCommitted(f.ledger,expired));
  EXPECT_FALSE(f.ledger.consumeCommittedCollisionAt(10260000000LL));
}
TEST(ExecutionLedger,RepeatedOccupiedRefreshDoesNotCancelEverySolveButActualClearRearms) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  CommittedObstacleFixture f;ASSERT_TRUE(A::recordCommitted(f.ledger,f.proof));
  ASSERT_TRUE(f.ledger.consumeCommittedCollisionAt(10010000000LL));
  for(int i=1;i<=10;++i) {
    auto p=f.proof;p.sequence+=i;p.check_end=rclcpp::Time(10000000000LL+i*50000000LL);
    p.valid_until=rclcpp::Time(10250000000LL+i*50000000LL);
    ASSERT_TRUE(A::recordCommitted(f.ledger,p));
    EXPECT_FALSE(f.ledger.consumeCommittedCollisionAt(10010000000LL+i*50000000LL));
  }
  auto clear=f.proof;clear.sequence=12;clear.valid=true;clear.reason="observed_free";
  clear.check_end=rclcpp::Time(10600000000LL);clear.valid_until=rclcpp::Time(10850000000LL);
  ASSERT_TRUE(A::recordCommitted(f.ledger,clear));
  EXPECT_FALSE(f.ledger.consumeCommittedCollisionAt(10610000000LL));
  auto reappeared=clear;reappeared.sequence=13;reappeared.valid=false;reappeared.reason="occupied";
  reappeared.check_end=rclcpp::Time(10650000000LL);reappeared.valid_until=rclcpp::Time(10900000000LL);
  ASSERT_TRUE(A::recordCommitted(f.ledger,reappeared));
  ASSERT_TRUE(f.ledger.consumeCommittedCollisionAt(10660000000LL));
  EXPECT_EQ(f.ledger.committedTrajectory(),1);
}
TEST(ExecutionLedger,UnconsumedExpiredCollisionCannotTriggerButFreshNativeRefreshCan) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  CommittedObstacleFixture f;ASSERT_TRUE(A::recordCommitted(f.ledger,f.proof));
  EXPECT_FALSE(f.ledger.consumeCommittedCollisionAt(10250000000LL));
  auto fresh=f.proof;++fresh.sequence;fresh.check_end=rclcpp::Time(10300000000LL);
  fresh.valid_until=rclcpp::Time(10550000000LL);
  ASSERT_TRUE(A::recordCommitted(f.ledger,fresh));
  ASSERT_TRUE(f.ledger.consumeCommittedCollisionAt(10310000000LL));
  EXPECT_EQ(rclcpp::Time(f.proof.valid_until).nanoseconds(),10250000000LL);
}
TEST(ExecutionLedger,CandidateUnknownIncompleteStaleForeignAndRevokedProofsAreNotCommittedCollisionEvents) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(int variant=0;variant<12;++variant) {
    CommittedObstacleFixture f;auto p=f.proof;
    if(variant==0)p.reason="unknown_or_unobserved";
    if(variant==1)p.reason="curve_check_budget_exhausted";
    if(variant==2)p.reason="source_expired_during_check";
    if(variant==3)p.valid=true;
    if(variant==4)p.support_hash="other";
    if(variant==5)p.trajectory_id=2;
    if(variant==6)p.version.task_id="other";
    if(variant==7)p.transport_mode="live";
    if(variant==8)p.whole_curve=p.remaining_curve=false;
    if(variant==9)p.collision_policy="official_inflated_geometry";
    if(variant==10)p.check_end=rclcpp::Time(10020000000LL);
    A::recordCommitted(f.ledger,p);
    if(variant==11){auto revoked=f.permit(2);revoked.revoked=true;ASSERT_TRUE(f.ledger.commit(revoked));}
    EXPECT_FALSE(f.ledger.consumeCommittedCollisionAt(10010000000LL))<<variant;
  }
  CommittedObstacleFixture f;
  auto candidate=f.spline;candidate.trajectory.traj_id=2;f.ledger.candidate(candidate,f.proposal);
  auto p=f.proof;p.trajectory_id=2;EXPECT_FALSE(A::recordCommitted(f.ledger,p));
  EXPECT_FALSE(f.ledger.consumeCommittedCollisionAt(10010000000LL));
  EXPECT_EQ(f.ledger.committedTrajectory(),1);
}
TEST(ExecutionLedger,PositiveOrNewAppliedCurveSupersedesUnconsumedCollisionAndLateReaderCannotRearm) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  CommittedObstacleFixture f;ASSERT_TRUE(A::recordCommitted(f.ledger,f.proof));
  auto clear=f.proof;clear.sequence=2;clear.valid=true;clear.check_end=rclcpp::Time(10010000000LL);
  ASSERT_TRUE(A::recordCommitted(f.ledger,clear));
  EXPECT_FALSE(A::recordCommitted(f.ledger,f.proof));
  EXPECT_FALSE(f.ledger.consumeCommittedCollisionAt(10020000000LL));
  auto p=f.proof;p.sequence=3;p.check_end=rclcpp::Time(10030000000LL);
  ASSERT_TRUE(A::recordCommitted(f.ledger,p));
  f.spline.trajectory.traj_id=2;f.ledger.candidate(f.spline,f.proposal);
  ASSERT_TRUE(f.ledger.commit(f.permit(2)));
  EXPECT_FALSE(f.ledger.consumeCommittedCollisionAt(10040000000LL));
  p.trajectory_id=2;p.sequence=4;p.check_end=rclcpp::Time(10050000000LL);
  ASSERT_TRUE(A::recordCommitted(f.ledger,p));ASSERT_TRUE(f.ledger.consumeCommittedCollisionAt(10060000000LL));
}
TEST(ExecutionLedger,ProductionBoxWholeCurveQueryObstacleClearAndReappearanceDriveSameCommittedEdge) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  PreviewCandidateFixture f;auto permit=f.permit();permit.phase="preview";
  ASSERT_TRUE(f.ledger.commit(permit));auto map=GridMapTestAccess::headingFixture(false,true);
  const auto clear=[&]{return wholeCurveCollisionFree(f.curve,.05,bodyExtent(map->planningBodyEnvelope()),
    [&](const Eigen::Vector3d& point,double yaw){return map->getPlanningInflateOccupancy(point,yaw);},
    f.body,"odom",10.31,.4,200000,0.,.3,{},0.,MeasuredConnectionPolicy::PreviewCandidateGeometry,.15);};
  auto proof=f.proof();proof.reason="occupied";
  proof.check_end=rclcpp::Time(10300000000LL);proof.valid_until=rclcpp::Time(10500000000LL);
  EXPECT_TRUE(clear());map->setOccupied({1.025,1.225,.575});EXPECT_FALSE(clear());
  ASSERT_TRUE(A::recordCommitted(f.ledger,proof));
  ASSERT_TRUE(f.ledger.consumeCommittedCollisionAt(10310000000LL));
  // Only actual removal under the same production body query rearms the
  // planning edge. The source/proof times are unchanged by consuming events.
  map=GridMapTestAccess::headingFixture(false,true);EXPECT_TRUE(clear());
  ++proof.sequence;proof.valid=true;proof.reason="observed_free";
  ASSERT_TRUE(A::recordCommitted(f.ledger,proof));
  EXPECT_FALSE(f.ledger.consumeCommittedCollisionAt(10310000000LL));
  map->setOccupied({2.125,1.025,.575});EXPECT_FALSE(clear());
  ++proof.sequence;proof.valid=false;proof.reason="occupied";
  ASSERT_TRUE(A::recordCommitted(f.ledger,proof));
  ASSERT_TRUE(f.ledger.consumeCommittedCollisionAt(10310000000LL));
  EXPECT_EQ(f.ledger.committedTrajectory(),1);EXPECT_FALSE(A::latest(f.ledger)->allowed);
}
TEST(ExecutionLedger,FreshExactStationaryAppliedCurvePostponesPeriodicOnlyWithoutRenewingEvidence) {
  CommittedIdleFixture f;using A=scan_planner::ExecutionValidatorTestAccess;
  const auto before=f.ledger.committedView();ASSERT_TRUE(before);ASSERT_TRUE(f.keep());
  EXPECT_TRUE(f.keep(f.now+100000000LL));
  const auto after=f.ledger.committedView();ASSERT_TRUE(after);
  EXPECT_EQ(after->spline,before->spline);EXPECT_EQ(after->proof,before->proof);
  EXPECT_EQ(after->progress,before->progress);EXPECT_FALSE(A::latest(f.ledger)->allowed);
  EXPECT_FALSE(f.keep(f.now+250000000LL)); // Original native proof expiry wins.
  EXPECT_EQ(after->proof->valid_until,before->proof->valid_until);
}
TEST(ExecutionLedger,IdleSchedulingUsesOriginalUnixHeaderNanosecondsNotRoundedDoubleLookup) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(const std::int64_t source:{1791786242116000123LL,1791786242116000277LL}) {
    SCOPED_TRACE(source);CommittedIdleFixture f;
    auto body=f.body;body.source_stamp=rclcpp::Time(source).seconds();
    ASSERT_NE(static_cast<std::int64_t>(std::llround(body.source_stamp*1e9)),source);
    f.ledger.body(body,geometry_msgs::msg::Twist{},source);
    auto progress=f.measured;progress.header.stamp=rclcpp::Time(source);
    f.ledger.progressAt(progress,rclcpp::Time(source).seconds());
    auto proof=f.proof;++proof.sequence;proof.check_end=rclcpp::Time(source);
    proof.body_source_stamp=proof.front_ray_source_stamp=proof.rear_ray_source_stamp=proof.check_end;
    proof.valid_until=rclcpp::Time(source+250000000LL);
    ASSERT_TRUE(A::recordCommitted(f.ledger,proof));
    auto permit=f.permit(2);permit.source_stamp=proof.check_end;permit.valid_until=proof.valid_until;
    ASSERT_TRUE(f.ledger.commit(permit));
    EXPECT_TRUE(f.ledger.keepCommittedIdleHorizonAt(f.proposal.version,source+10000000LL,.4,1.));
    EXPECT_EQ(f.ledger.committedView()->progress->header.stamp,progress.header.stamp);
    EXPECT_EQ(f.ledger.committedView()->proof->body_source_stamp,proof.body_source_stamp);
    // Equal floating seconds do not make a different raw sample identical.
    auto changed=body;changed.source_stamp=rclcpp::Time(source+1).seconds();
    ASSERT_EQ(changed.source_stamp,body.source_stamp);changed.position.x()+=.02;
    f.ledger.body(changed,geometry_msgs::msg::Twist{},source+1);
    EXPECT_FALSE(f.ledger.keepCommittedIdleHorizonAt(f.proposal.version,source+10000000LL,.4,1.));
  }
}
TEST(ExecutionLedger,IdleSchedulingRejectsNegativeForeignExpiredProofOrCurrentReferenceChange) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(unsigned kind=0;kind<10;++kind) {
    SCOPED_TRACE(kind);CommittedIdleFixture f;auto p=f.proof;++p.sequence;
    if(kind==0){p.valid=false;p.reason="occupied";}
    if(kind==1){p.valid=false;p.reason="source_expired_during_check";}
    if(kind==2)p.valid_until=rclcpp::Time(f.now);
    if(kind==3)p.check_end=rclcpp::Time(f.now+21000000LL);
    if(kind==4)p.support_hash="other_support";
    if(kind==5)p.proposal_id="other_proposal";
    if(kind==6)p.collision_policy="foreign_policy";
    if(kind==7)p.body_source_stamp=rclcpp::Time(f.now-401000000LL);
    if(kind==8)p.body_source_stamp=rclcpp::Time(f.now+21000000LL);
    if(kind==9){p.whole_curve=false;p.remaining_curve=false;}
    if(kind==7) {
      // The real proof-order CAS rejects an older acquired body before it
      // could replace current evidence. Its original proof expires unchanged.
      EXPECT_FALSE(A::recordCommitted(f.ledger,p));EXPECT_TRUE(f.keep());
      EXPECT_FALSE(f.keep(f.now+401000000LL));continue;
    }
    ASSERT_TRUE(A::recordCommitted(f.ledger,p));EXPECT_FALSE(f.keep());
  }
  CommittedIdleFixture f;auto changed=f.proposal.version;++changed.reference_generation;
  EXPECT_FALSE(f.ledger.keepCommittedIdleHorizonAt(changed,f.now,.4,1.));
  changed=f.proposal.version;++changed.anchor_revision;
  EXPECT_FALSE(f.ledger.keepCommittedIdleHorizonAt(changed,f.now,.4,1.));
  EXPECT_TRUE(f.keep());EXPECT_EQ(f.ledger.committedVersion(),f.proposal.version);
}
TEST(ExecutionLedger,IdleSchedulingRequiresActualOdomAndMeasuredProgressNotHeartbeatOrWallClock) {
  for(unsigned kind=0;kind<6;++kind) {
    SCOPED_TRACE(kind);CommittedIdleFixture f;
    auto body=f.body;auto twist=geometry_msgs::msg::Twist{};
    if(kind==0)twist.linear.x=.02;
    if(kind==1)twist.linear.y=.02;
    if(kind==2)twist.linear.z=.02;
    if(kind==3)twist.angular.z=.02;
    if(kind==4)body.position.z()+=.02;
    if(kind==5)body.position.x()+=1.;
    body.source_stamp+=.001;
    f.ledger.body(body,twist,f.now+1000000LL);
    EXPECT_FALSE(f.keep(f.now+1000000LL));
  }
  for(unsigned kind=0;kind<5;++kind) {
    SCOPED_TRACE(kind);CommittedIdleFixture f;auto p=f.measured;
    p.header.stamp=rclcpp::Time(f.now+1000000LL);
    if(kind==0)p.holding=true;
    if(kind==1)p.curve_time=f.curve.getTimeSum()*.25;
    if(kind==2)p.twist.linear.x=.02;
    if(kind==3)p.twist.angular.z=.02;
    if(kind==4)p.pose.position.z+=.02;
    f.ledger.progressAt(p,(f.now+1000000LL)*1e-9);
    EXPECT_FALSE(f.keep(f.now+1000000LL));
  }
}
TEST(ExecutionLedger,IdleSchedulingDoesNotHidePendingCandidateHandoffObstacleOrReseed) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(unsigned kind=0;kind<4;++kind) {
    SCOPED_TRACE(kind);CommittedIdleFixture f;A::pendingSchedulingEvent(f.ledger,kind);
    EXPECT_FALSE(f.keep());EXPECT_EQ(f.ledger.committedTrajectory(),1);
  }
  CommittedIdleFixture f;auto candidate=f.spline;candidate.trajectory.traj_id=2;
  f.ledger.candidate(candidate,f.proposal);EXPECT_FALSE(f.keep());
  f.ledger.cancelPending();EXPECT_TRUE(f.keep());
  auto cancel=f.proposal.version;++cancel.reference_generation;
  ASSERT_TRUE(f.ledger.cancelReference(cancel));EXPECT_FALSE(f.keep());
}
TEST(ExecutionLedger,LateOrDuplicateProgressCannotKeepExpiredAppliedHorizonAlive) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  CommittedIdleFixture f;const auto original=f.ledger.committedView()->progress;
  auto late=f.measured;late.trajectory_id=99;late.header.stamp=rclcpp::Time(f.now+390000000LL);
  f.ledger.progressAt(late,(f.now+390000000LL)*1e-9);
  auto duplicate=f.measured;duplicate.curve_time=.0;
  f.ledger.progressAt(duplicate,(f.now+390000000LL)*1e-9);
  EXPECT_EQ(f.ledger.committedView()->progress,original);
  // A genuinely refreshed native proof/permit and odom are not a substitute
  // for fresh measured progress from this exact applied curve.
  auto fresh=f.proof;++fresh.sequence;fresh.check_end=rclcpp::Time(f.now+401000000LL);
  fresh.body_source_stamp=fresh.check_end;fresh.valid_until=rclcpp::Time(f.now+600000000LL);
  ASSERT_TRUE(A::recordCommitted(f.ledger,fresh));
  auto permit=f.permit(2);permit.source_stamp=fresh.check_end;permit.valid_until=fresh.valid_until;
  ASSERT_TRUE(f.ledger.commit(permit));
  auto body=f.body;body.source_stamp+=.401;
  f.ledger.body(body,geometry_msgs::msg::Twist{},f.now+401000000LL);
  EXPECT_FALSE(f.keep(f.now+401000000LL));
  late=f.measured;late.header.stamp=fresh.check_end;
  f.ledger.progressAt(late,(f.now+401000000LL)*1e-9);
  EXPECT_TRUE(f.keep(f.now+401000000LL));
}
TEST(ExecutionLedger,RemainingCurveSchedulingRequiresActuallyCheckedProgressRange) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  CommittedIdleFixture f;auto p=f.proof;++p.sequence;p.whole_curve=false;p.remaining_curve=true;
  p.checked_from_time=.08;p.checked_to_time=f.curve.getTimeSum();
  ASSERT_TRUE(A::recordCommitted(f.ledger,p));EXPECT_TRUE(f.keep());
  ++p.sequence;p.checked_from_time=.2;
  ASSERT_TRUE(A::recordCommitted(f.ledger,p));EXPECT_FALSE(f.keep());
}
TEST(ExecutionLedger,OccupiedInPlaceTurnIsBlockedEntryButStillAndReverseAreNot) {
  // Each variant is recorded as a genuine native proof, so only the command
  // shape decides: an in-place rotation is an entry, zero/reverse never are.
  struct Case{double vx,wz;bool accepted;};
  for(const auto c:{Case{0.,.3,true},Case{0.,-.3,true},Case{0.,0.,false},
                    Case{0.,1e-10,false},Case{-.01,.3,false},Case{-.01,0.,false}}) {
    SCOPED_TRACE(std::to_string(c.vx)+","+std::to_string(c.wz));
    Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
    w::MotionValidation p;p.version=f.proposal.version;p.trajectory_id=1;p.sequence=5;
    p.reason="motion_sweep_occupied";p.velocity.linear.x=c.vx;p.velocity.angular.z=c.wz;
    p.check_end=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10090000000LL);
    scan_planner::ExecutionValidatorTestAccess::record(f.ledger,p);
    auto unknown=p;unknown.sequence=4;unknown.reason="motion_sweep_unknown_or_expired";
    scan_planner::ExecutionValidatorTestAccess::record(f.ledger,unknown);
    EXPECT_FALSE(f.ledger.submitBlockedEntryAt(unknown,10010000000LL));
    EXPECT_EQ(f.ledger.submitBlockedEntryAt(p,10010000000LL),c.accepted);
    EXPECT_EQ(static_cast<bool>(f.ledger.consumeBlockedEntry()),c.accepted);
    EXPECT_EQ(f.ledger.committedTrajectory(),1);
    EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::latest(f.ledger)->allowed);
  }
}
TEST(ExecutionLedger,HolonomicLateralOrReverseOccupiedEventIsExactAndDoesNotGrantMotion) {
  using namespace scan_planner;using A=ExecutionValidatorTestAccess;
  for(const auto& xy:{Eigen::Vector2d(0.,.1),Eigen::Vector2d(0.,-.1),Eigen::Vector2d(-.1,.1)}) {
    Fixture f;BrakingModel model{.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'a')};
    model.kinematic_profile=KinematicProfile::HolonomicXYYaw;model.lateral_distance=.08;
    A::braking(f.ledger,model);f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
    w::MotionValidation proof;proof.version=f.proposal.version;proof.trajectory_id=1;proof.sequence=5;
    proof.reason="motion_sweep_occupied";proof.velocity.linear.x=xy.x();proof.velocity.linear.y=xy.y();
    proof.check_end=rclcpp::Time(10000000000LL);proof.valid_until=rclcpp::Time(10090000000LL);
    EXPECT_FALSE(f.ledger.submitBlockedEntryAt(proof,10010000000LL));
    A::record(f.ledger,proof);EXPECT_TRUE(f.ledger.submitBlockedEntryAt(proof,10010000000LL));
    const auto event=f.ledger.consumeBlockedEntry();ASSERT_TRUE(event);EXPECT_EQ(*event,proof);
    EXPECT_EQ(f.ledger.committedTrajectory(),1);EXPECT_FALSE(A::latest(f.ledger)->allowed);
  }
}
TEST(ExecutionLedger,RevokedOrReplacedOwnerCannotConsumeOldBlockedEntry) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  w::MotionValidation p;p.version=f.proposal.version;p.trajectory_id=1;p.sequence=5;
  p.reason="motion_sweep_occupied";p.velocity.linear.x=.01;
  p.check_end=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10090000000LL);
  scan_planner::ExecutionValidatorTestAccess::record(f.ledger,p);
  ASSERT_TRUE(f.ledger.submitBlockedEntryAt(p,10010000000LL));
  auto permit=f.permit(2);permit.revoked=true;ASSERT_TRUE(f.ledger.commit(permit));
  EXPECT_FALSE(f.ledger.consumeBlockedEntry());
}
TEST(ExecutionLedger,OnlyMatchingOwnerGeometryCommitSwitchesCurve) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  f.spline.trajectory.traj_id=2;f.ledger.candidate(f.spline,f.proposal);auto p=f.permit(2);
  p.trajectory_id=3;EXPECT_FALSE(f.ledger.commit(p));EXPECT_EQ(f.ledger.committedTrajectory(),1);
  p.trajectory_id=2;ASSERT_TRUE(f.ledger.commit(p));EXPECT_EQ(f.ledger.committedTrajectory(),2);
}
TEST(ExecutionLedger,HoldingFreshMeasuredProgressIsNotMissingOrVirtualMotion) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  auto p=f.progress();f.ledger.progressAt(p,10.02);auto active=f.ledger.committedView();
  ASSERT_TRUE(active&&active->progress);EXPECT_TRUE(active->progress->holding);
  EXPECT_DOUBLE_EQ(active->progress->curve_time,0.);
  p.trajectory_id=2;p.curve_time=2.;p.arc_length=p.s_committed=.5;f.ledger.progressAt(p,10.02);
  EXPECT_DOUBLE_EQ(f.ledger.committedView()->progress->curve_time,0.);
}
TEST(ExecutionLedger,ExactFreshCandidateEntryRejectionIsOneBoundedReseedNotGeometryRetirement) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(const auto* reason:{"candidate:entry_measured_velocity_off_candidate","candidate:entry_measured_body_not_on_candidate"}) {
    Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
    f.spline.trajectory.traj_id=2;f.ledger.candidate(f.spline,f.proposal);
    w::TrajectoryValidation v;v.version=f.proposal.version;v.trajectory_id=2;v.sequence=50;v.valid=v.whole_curve=true;
    v.frame_id="odom";v.transport_mode="isolated_mock";
    v.check_end=rclcpp::Time(10000000000LL);v.valid_until=rclcpp::Time(10200000000LL);A::recordWhole(f.ledger,v);
    w::TrajectoryAdmission a;a.sequence=1;a.version=v.version;a.trajectory_id=2;a.validation_sequence=50;
    a.transport_mode="isolated_mock";a.reason=reason;a.body_source_stamp=rclcpp::Time(10010000000LL);
    a.checked_at=rclcpp::Time(10020000000LL);a.valid_until=rclcpp::Time(10170000000LL);
    ASSERT_TRUE(f.ledger.submitCandidateEntryRejectionAt(a,10025000000LL));
    EXPECT_FALSE(f.ledger.submitCandidateEntryRejectionAt(a,10025000000LL));
    auto event=f.ledger.consumeFormalReseedAt(10030000000LL);ASSERT_TRUE(event);
    EXPECT_EQ(event->version,a.version);EXPECT_EQ(event->trajectory_id,2);EXPECT_FALSE(event->terminal);
    EXPECT_EQ(event->body_source_stamp,a.body_source_stamp);EXPECT_EQ(f.ledger.committedTrajectory(),1);
    EXPECT_FALSE(f.ledger.consumeFormalReseedAt(10030000000LL));
    ++a.sequence;a.body_source_stamp=rclcpp::Time(10040000000LL);a.checked_at=rclcpp::Time(10050000000LL);
    EXPECT_FALSE(f.ledger.submitCandidateEntryRejectionAt(a,10055000000LL));
    // A NEW real solver curve may schedule its own bounded retry, not a queue
    // of old failures against curve 2 or a new task/global route.
    f.spline.trajectory.traj_id=3;f.ledger.candidate(f.spline,f.proposal);
    v.trajectory_id=3;++v.sequence;A::recordWhole(f.ledger,v);
    a.trajectory_id=3;a.validation_sequence=v.sequence;++a.sequence;
    ASSERT_TRUE(f.ledger.submitCandidateEntryRejectionAt(a,10055000000LL));
    EXPECT_EQ(f.ledger.consumeFormalReseedAt(10060000000LL)->trajectory_id,3);
    EXPECT_EQ(f.ledger.committedTrajectory(),1);
  }
}
TEST(ExecutionLedger,EntryReseedRejectsWrongTaskProofSourceUnknownAndSuccess) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(int variant=0;variant<17;++variant) {
    Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
    f.spline.trajectory.traj_id=2;f.ledger.candidate(f.spline,f.proposal);
    w::TrajectoryValidation v;v.version=f.proposal.version;v.trajectory_id=2;v.sequence=50;v.valid=v.whole_curve=true;
    v.frame_id="odom";v.transport_mode="isolated_mock";
    v.check_end=rclcpp::Time(10000000000LL);v.valid_until=rclcpp::Time(10200000000LL);
    if(variant==9)v.valid=false;if(variant==10)v.valid_until=rclcpp::Time(10020000000LL);
    if(variant==13)v.whole_curve=false;if(variant==14)v.remaining_curve=true;
    if(variant==15)v.frame_id="map";if(variant==16)v.transport_mode="live";
    A::recordWhole(f.ledger,v);
    w::TrajectoryAdmission a;a.sequence=1;a.version=v.version;a.trajectory_id=2;a.validation_sequence=50;
    a.transport_mode="isolated_mock";a.reason="candidate:entry_measured_velocity_off_candidate";
    a.body_source_stamp=rclcpp::Time(10010000000LL);a.checked_at=rclcpp::Time(10020000000LL);
    a.valid_until=rclcpp::Time(10170000000LL);
    if(variant==0)a.version.task_id="other";if(variant==1)++a.version.context_sequence;
    if(variant==2)++a.version.map_geometry_revision;if(variant==3)a.trajectory_id=1;
    if(variant==4)++a.validation_sequence;if(variant==5)a.transport_mode="live";
    if(variant==6)a.body_source_stamp=rclcpp::Time(9900000000LL);
    if(variant==7)a.body_source_stamp=rclcpp::Time(10060000000LL);
    if(variant==8)a.valid_until=rclcpp::Time(10025000000LL);
    if(variant==11)a.reason="candidate:unknown_space";if(variant==12)a.accepted=true;
    EXPECT_FALSE(f.ledger.submitCandidateEntryRejectionAt(a,10025000000LL))<<variant;
    EXPECT_FALSE(f.ledger.consumeFormalReseedAt(10030000000LL))<<variant;
    EXPECT_EQ(f.ledger.committedTrajectory(),1);
  }
}
TEST(ExecutionLedger,QueuedEntryFailureCoalescesAndIsDiscardedForAcceptedReplacedExpiredOrRevokedCandidate) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(int variant=0;variant<4;++variant) {
    Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
    f.spline.trajectory.traj_id=2;f.ledger.candidate(f.spline,f.proposal);
    w::TrajectoryValidation v;v.version=f.proposal.version;v.trajectory_id=2;v.sequence=50;v.valid=v.whole_curve=true;
    v.frame_id="odom";v.transport_mode="isolated_mock";
    v.check_end=rclcpp::Time(10000000000LL);v.valid_until=rclcpp::Time(10200000000LL);A::recordWhole(f.ledger,v);
    w::TrajectoryAdmission a;a.sequence=1;a.version=v.version;a.trajectory_id=2;a.validation_sequence=50;
    a.transport_mode="isolated_mock";a.reason="candidate:entry_measured_velocity_off_candidate";
    a.body_source_stamp=rclcpp::Time(10010000000LL);a.checked_at=rclcpp::Time(10020000000LL);
    a.valid_until=rclcpp::Time(10170000000LL);
    ASSERT_TRUE(f.ledger.submitCandidateEntryRejectionAt(a,10025000000LL));
    if(variant==0){++a.sequence;a.accepted=true;EXPECT_FALSE(f.ledger.submitCandidateEntryRejectionAt(a,10026000000LL));}
    if(variant==1){++f.spline.trajectory.traj_id;f.ledger.candidate(f.spline,f.proposal);}
    if(variant==3){auto p=f.permit(2);p.revoked=true;ASSERT_TRUE(f.ledger.commit(p));}
    EXPECT_FALSE(f.ledger.consumeFormalReseedAt(variant==2?10180000000LL:10030000000LL));
  }
}
TEST(ExecutionLedger,MeasuredTerminalReseedIsOncePerAppliedCurveNotElapsedCandidateClockOrPreviewHold) {
  for(int variant=0;variant<5;++variant) {
    Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
    auto p=f.progress();p.reason="local_segment_finished_waiting_replan";
    if(variant==1)p.reason="braking_envelope_reentry_required";if(variant==2)p.holding=false;
    if(variant==3)p.trajectory_id=2;if(variant==4)p.header.stamp=rclcpp::Time(9500000000LL);
    f.ledger.progressAt(p,10.02);
    auto event=f.ledger.consumeFormalReseedAt(10030000000LL);
    ASSERT_EQ(static_cast<bool>(event),variant<=1)<<variant;
    if(event) {
      EXPECT_TRUE(event->terminal);EXPECT_EQ(event->trajectory_id,1);
      EXPECT_EQ(event->version,f.proposal.version);
      EXPECT_EQ(event->body_source_stamp,p.header.stamp);
      EXPECT_EQ(event->reason,variant==1?"measured_tracking_braking_boundary_unreachable":"measured_local_horizon_finished");
      EXPECT_FALSE(f.ledger.consumeFormalReseedAt(10031000000LL));
      p.header.stamp=rclcpp::Time(10040000000LL);f.ledger.progressAt(p,10.05);
      EXPECT_FALSE(f.ledger.consumeFormalReseedAt(10060000000LL));
      EXPECT_EQ(f.ledger.committedTrajectory(),1);EXPECT_TRUE(f.ledger.committedView()->progress->holding);
    }
  }
}
TEST(ExecutionLedger,WrongFloorAnchorAndExpiredProgressNeverReplaceSource) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  for(int mode=0;mode<4;++mode) {
    auto p=f.progress();if(mode==0)p.anchor_id="other";if(mode==1)p.segment_id="floor2";
    if(mode==2)p.valid=false;
    f.ledger.progressAt(p,mode==3?10.5:10.02);EXPECT_FALSE(f.ledger.committedView()->progress);
  }
}
TEST(ExecutionLedger,RevocationRetiresCandidateAndRejectsLateGeometry) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  auto p=f.permit(2);p.revoked=true;ASSERT_TRUE(f.ledger.commit(p));
  f.ledger.candidate(f.spline,f.proposal);EXPECT_FALSE(f.ledger.commit(f.permit(3)));
  EXPECT_FALSE(f.ledger.committedView());
}
TEST(ExecutionLedger,ReferenceCancellationRetiresOwnerBeforeAcknowledgementButNotForeignTasks) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  auto barrier=f.proposal.version;++barrier.reference_generation;auto foreign=barrier;foreign.task_id="other";
  EXPECT_FALSE(f.ledger.cancelReference(foreign));EXPECT_EQ(f.ledger.committedTrajectory(),1);
  EXPECT_FALSE(f.ledger.cancelReference(f.proposal.version));EXPECT_EQ(f.ledger.committedTrajectory(),1);
  ASSERT_TRUE(f.ledger.cancelReference(barrier));EXPECT_FALSE(f.ledger.committedView());
  f.ledger.candidate(f.spline,f.proposal);EXPECT_FALSE(f.ledger.commit(f.permit(2)));
  // A genuinely new owner is independent of the retired route's generation.
  f.proposal.version.task_id="new-task";f.proposal.version.reference_generation=3;
  f.ledger.candidate(f.spline,f.proposal);EXPECT_TRUE(f.ledger.commit(f.permit(3)));
}
TEST(ExecutionLedger,ReferenceReplacementRechecksExactPredecessorBeforeClearingCandidate) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  auto next=f.proposal;next.version.reference_generation=2;next.proposal_id="next";
  auto spline=f.spline;spline.generation=2;spline.trajectory.traj_id=2;
  f.ledger.candidate(spline,next);
  next.expected_version=f.proposal.version;next.expected_trajectory_id=1;
  auto wrong=next;wrong.expected_trajectory_id=2;
  EXPECT_FALSE(f.ledger.replacePendingReference(wrong));
  EXPECT_EQ(scan_planner::ExecutionValidatorTestAccess::candidateReadSnapshot(f.ledger).proposal,"next");
  wrong=next;wrong.expected_version.reference_generation=2;
  EXPECT_FALSE(f.ledger.replacePendingReference(wrong));
  ASSERT_TRUE(f.ledger.replacePendingReference(next));
  EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::hasCandidate(f.ledger));
  EXPECT_EQ(f.ledger.committedTrajectory(),1); // active curve is not a candidate.
}
TEST(ExecutionLedger,ForeignTaskCannotClearPendingReferenceInAnOpenOwnerDomain) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  auto pending=f.proposal;pending.version.reference_generation=2;pending.proposal_id="pending";
  f.ledger.candidate(f.spline,pending);
  auto late=pending;late.version.task_id="foreign";late.expected_version=f.proposal.version;
  late.expected_trajectory_id=1;
  EXPECT_FALSE(f.ledger.replacePendingReference(late));
  EXPECT_EQ(scan_planner::ExecutionValidatorTestAccess::candidateReadSnapshot(f.ledger).proposal,"pending");
  EXPECT_EQ(f.ledger.committedTrajectory(),1);
}
TEST(ExecutionLedger,RetiredTaskWithHigherGenerationCannotClearANewTaskCandidate) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  auto cancelled=f.proposal;cancelled.version.reference_generation=2;
  ASSERT_TRUE(f.ledger.cancelReference(cancelled.version));
  auto newer=f.proposal;newer.version.task_id="new-task";newer.version.reference_generation=3;
  newer.proposal_id="new-pending";f.ledger.candidate(f.spline,newer);
  cancelled.version.reference_generation=100;
  EXPECT_FALSE(f.ledger.replacePendingReference(cancelled));
  f.ledger.candidate(f.spline,cancelled);
  EXPECT_EQ(scan_planner::ExecutionValidatorTestAccess::candidateReadSnapshot(f.ledger).proposal,"new-pending");
  ASSERT_TRUE(f.ledger.replacePendingReference(newer));
  EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::hasCandidate(f.ledger));
  EXPECT_FALSE(f.ledger.committedView()); // replacement is not an owner commit.
}
TEST(ExecutionLedger,ReferenceCancellationBeforeFirstOwnerPermitStillClosesWholeTask) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);
  auto barrier=f.proposal.version;barrier.reference_generation=2;
  ASSERT_TRUE(f.ledger.cancelReference(barrier));
  auto late=f.proposal;late.version.reference_generation=3;
  EXPECT_FALSE(f.ledger.replacePendingReference(late));
  f.ledger.candidate(f.spline,late);
  EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::hasCandidate(f.ledger));
  auto fresh=late;fresh.version.task_id="new-task";
  EXPECT_TRUE(f.ledger.replacePendingReference(fresh));
}
TEST(ExecutionLedger,ReplanBoundaryUsesActualTwistAndNeverInventsMeasuredAcceleration) {
  Fixture f;auto p=f.progress();p.twist.linear.x=.0648;p.twist.linear.z=.003;
  p.acceleration.linear.z=-.4;p.acceleration_valid=false;
  auto b=scan_planner::ExecutionValidator::measuredBoundary(p,Eigen::Vector3d::Zero());ASSERT_TRUE(b);
  EXPECT_DOUBLE_EQ(b->velocity.z(),.003);EXPECT_TRUE(b->acceleration.isZero());
  p.acceleration_valid=true;b=scan_planner::ExecutionValidator::measuredBoundary(p,Eigen::Vector3d::Zero());
  ASSERT_TRUE(b);EXPECT_DOUBLE_EQ(b->acceleration.z(),-.4);
  EXPECT_FALSE(scan_planner::ExecutionValidator::measuredBoundary(p,Eigen::Vector3d(.02,0,0)));
}
TEST(ExecutionLedger,ActualConnectorRequiresSourceGroundNotImaginaryLineOrAnotherFloor) {
  w::SupportReference s;s.verified=true;s.floor_id="floor1";s.segment_kind="floor";s.required_mode="general";
  s.frame_id="odom";s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');
  s.support_xy_radius_m=.02;s.body_reference_height_m=.55;s.max_support_slope_rad=.15;s.max_support_step_m=.05;
  for(int i=0;i<=12;++i){geometry_msgs::msg::Point p;p.y=i*.01;s.support_ground_xyz.push_back(p);}
  const Eigen::Vector3d a(0,0,.55),b(0,.12,.55);
  EXPECT_TRUE(scan_planner::measuredConnectionSupported(s,a,b,"odom",.05));
  const auto full=s.support_ground_xyz;s.support_ground_xyz={full.front(),full.back()};
  EXPECT_FALSE(scan_planner::measuredConnectionSupported(s,a,b,"odom",.05));
  s.support_ground_xyz=full;s.floor_id="floor2";
  EXPECT_TRUE(scan_planner::measuredConnectionSupported(s,a,b,"odom",.05));
  s.floor_id="floor3";EXPECT_FALSE(scan_planner::measuredConnectionSupported(s,a,b,"odom",.05));
  s.floor_id="floor2";for(auto& p:s.support_ground_xyz)p.z=3.;
  EXPECT_FALSE(scan_planner::measuredConnectionSupported(s,a,b,"odom",.05));
  const Eigen::Vector3d upper_a(0,0,3.55),upper_b(0,.12,3.55);
  EXPECT_TRUE(scan_planner::measuredConnectionSupported(s,upper_a,upper_b,"odom",.05));
  s.verified=false;EXPECT_FALSE(scan_planner::measuredConnectionSupported(s,upper_a,upper_b,"odom",.05));s.verified=true;
  s.segment_kind="stairs";EXPECT_FALSE(scan_planner::measuredConnectionSupported(s,upper_a,upper_b,"odom",.05));
  s.segment_kind="floor";s.required_mode="stair";
  EXPECT_FALSE(scan_planner::measuredConnectionSupported(s,upper_a,upper_b,"odom",.05));
}
TEST(ExecutionLedger,SupportDiagnosticPreservesOriginalPredicate) {
  // Frozen pre-diagnostic predicate, kept only as a regression oracle.
  const auto original=[](const w::SupportReference& s,const Eigen::Vector3d& from,
      const Eigen::Vector3d& to,const std::string& frame,double resolution) {
    if(!s.verified||(s.floor_id!="floor1"&&s.floor_id!="floor2")||(s.segment_kind!="floor"&&s.segment_kind!="landing")||s.required_mode!="general"||
       s.frame_id!=frame||s.support_hash.size()!=64||s.support_map_sha256.size()!=64||
       s.support_ground_xyz.size()<2||s.support_ground_xyz.size()>20000||
       !std::isfinite(s.support_xy_radius_m)||s.support_xy_radius_m<=0.||s.support_xy_radius_m>.5||
       !std::isfinite(s.body_reference_height_m)||s.body_reference_height_m<.2||s.body_reference_height_m>.8||
       !std::isfinite(s.max_support_slope_rad)||s.max_support_slope_rad<=0.||s.max_support_slope_rad>.176||
       !std::isfinite(s.max_support_step_m)||s.max_support_step_m<=0.||s.max_support_step_m>.08||
       !from.allFinite()||!to.allFinite()||(to-from).norm()>.15||!std::isfinite(resolution)||resolution<=0.)return false;
    for(const auto& p:s.support_ground_xyz)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return false;
    const auto count=std::max(1,static_cast<int>(std::ceil((to-from).norm()/std::min(.0125,resolution*.25))));
    if(count>100)return false;
    for(int i=0;i<=count;++i) {
      const Eigen::Vector3d body=from+(double(i)/count)*(to-from);
      double nearest=std::numeric_limits<double>::infinity();const geometry_msgs::msg::Point* anchor=nullptr;
      for(const auto& p:s.support_ground_xyz){const double d=std::hypot(p.x-body.x(),p.y-body.y());if(d<nearest){nearest=d;anchor=&p;}}
      if(!anchor||nearest>s.support_xy_radius_m||std::abs(body.z()-anchor->z-s.body_reference_height_m)>.15)return false;
      for(const auto& p:s.support_ground_xyz){const double distance=std::hypot(p.x-anchor->x,p.y-anchor->y);
        if(distance<=s.support_xy_radius_m&&distance>=1e-6&&
           std::abs(p.z-anchor->z)>s.max_support_step_m+std::tan(s.max_support_slope_rad)*distance)return false;}
    }
    return true;
  };
  w::SupportReference base;base.verified=true;base.floor_id="floor1";base.segment_kind="floor";base.required_mode="general";
  base.frame_id="odom";base.support_hash=std::string(64,'a');base.support_map_sha256=std::string(64,'b');
  base.support_xy_radius_m=.2;base.body_reference_height_m=.5;base.max_support_slope_rad=.15;base.max_support_step_m=.05;
  for(int i=0;i<25;++i){geometry_msgs::msg::Point p;p.x=(i%5-2)*.05;p.y=(i/5-2)*.05;base.support_ground_xyz.push_back(p);}
  for(unsigned i=0;i<256;++i) {
    auto s=base;Eigen::Vector3d a(0,0,.5),b(.10,0,.5);double resolution=.05;std::string frame="odom";
    const unsigned variant=i%16;
    if(variant==0)s.verified=false;
    if(variant==1)s.floor_id="floor3";
    if(variant==2)s.segment_kind="stairs";
    if(variant==3)s.required_mode="stair";
    if(variant==4)frame="map";
    if(variant==5)s.support_hash.clear();
    if(variant==6)s.support_ground_xyz.resize(i%2);
    if(variant==7)s.support_xy_radius_m=(i%3)*.25;
    if(variant==8)s.support_ground_xyz[3].z=NAN;
    if(variant==9)b.x()=.15000001;
    if(variant==10)a.y()=b.y()=.5;
    if(variant==11)a.z()=b.z()=.50+(int(i/16)-8)*.025;
    if(variant==12)s.support_ground_xyz[13].z=(i/16)*.007;
    if(variant==13)resolution=i%2?.0001:NAN;
    if(variant==14)s.body_reference_height_m=(i/16)*.1;
    if(variant==15)s.max_support_slope_rad=(i/16)*.02;
    scan_planner::SupportConnectionDiagnostic d;
    const auto expected=original(s,a,b,frame,resolution);
    EXPECT_EQ(scan_planner::measuredConnectionSupported(s,a,b,frame,resolution,&d),expected)<<i;
    EXPECT_EQ(scan_planner::measuredConnectionSupported(s,a,b,frame,resolution),expected)<<i;
    EXPECT_TRUE(d.checked);EXPECT_EQ(d.supported,expected);EXPECT_EQ(d.source_points,s.support_ground_xyz.size());
  }
}
TEST(ExecutionLedger,SupportDiagnosticIdentifiesSampleHeightAndRawNeighborWithoutChangingLimits) {
  w::SupportReference s;s.verified=true;s.floor_id="floor1";s.segment_kind="floor";s.required_mode="general";
  s.frame_id="odom";s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');
  s.support_xy_radius_m=.2;s.body_reference_height_m=.5;s.max_support_slope_rad=.15;s.max_support_step_m=.05;
  geometry_msgs::msg::Point a,b;b.x=.1;s.support_ground_xyz={a,b};
  scan_planner::SupportConnectionDiagnostic d;const Eigen::Vector3d body(0,0,.5);
  EXPECT_TRUE(scan_planner::measuredConnectionSupported(s,body,body,"odom",.05,&d));EXPECT_STREQ(d.reason,"supported");
  s.support_ground_xyz[1].z=.08;
  EXPECT_FALSE(scan_planner::measuredConnectionSupported(s,body,body,"odom",.05,&d));
  EXPECT_STREQ(d.reason,"support_neighbor_height_discontinuity");EXPECT_DOUBLE_EQ(d.sample.x(),0.);
  EXPECT_EQ(d.anchor,Eigen::Vector3d::Zero());EXPECT_EQ(d.neighbor,Eigen::Vector3d(.1,0,.08));
  EXPECT_DOUBLE_EQ(d.neighbor_height_delta_m,.08);EXPECT_DOUBLE_EQ(d.neighbor_allowed_delta_m,.05+std::tan(.15)*.1);
  s.support_ground_xyz[1].z=0.;
  const Eigen::Vector3d high(0,0,.651);
  EXPECT_FALSE(scan_planner::measuredConnectionSupported(s,high,high,"odom",.05,&d));
  EXPECT_STREQ(d.reason,"body_support_height_mismatch");EXPECT_NEAR(d.body_height_error_m,.151,1e-15);
  const Eigen::Vector3d outside(.4,0,.5);
  EXPECT_FALSE(scan_planner::measuredConnectionSupported(s,outside,outside,"odom",.05,&d));
  EXPECT_STREQ(d.reason,"support_xy_coverage_missing");EXPECT_NEAR(d.nearest_xy_m,.3,1e-15);
  EXPECT_FALSE(scan_planner::measuredConnectionSupported(s,body,body+Eigen::Vector3d(.151,0,0),"odom",.05,&d));
  EXPECT_STREQ(d.reason,"connection_length_exceeded");
}
TEST(ExecutionLedger,CommandProofRequiresExactCurrentOwnerAndCannotAuthorizeHolding) {
  Fixture f;auto p=f.permit();p.allowed=true;p.phase="tracking";p.execution_id="exec";
  p.sdk_session="sdk";p.sdk_arm_generation=2;p.control_epoch=3;
  w::MotionDemand d;d.version=p.version;d.trajectory_id=p.trajectory_id;d.execution_id=p.execution_id;
  d.sdk_session=p.sdk_session;d.sdk_arm_generation=p.sdk_arm_generation;d.control_epoch=p.control_epoch;
  using A=scan_planner::ExecutionValidatorTestAccess;EXPECT_TRUE(A::matches(d,p));
  for(int mode=0;mode<7;++mode){auto bad=p;
    if(mode==0)bad.allowed=false;if(mode==1)bad.phase="holding";if(mode==2)bad.revoked=true;
    if(mode==3)++bad.sdk_arm_generation;if(mode==4)++bad.control_epoch;
    if(mode==5)bad.version.anchor_id="other";if(mode==6)++bad.trajectory_id;
    EXPECT_FALSE(A::matches(d,bad));
  }
}
TEST(ExecutionLedger,LaterZeroAndPermissionRevocationFenceLateMotionProof) {
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  using A=scan_planner::ExecutionValidatorTestAccess;
  w::MotionDemand d;d.version=f.proposal.version;d.transport_mode="isolated_mock";d.sequence=1;d.execution_id="execution";d.velocity.linear.x=.2;
  f.ledger.demand(d);auto barrier=A::barrier(f.ledger);++d.sequence;d.velocity.linear.x=.1;f.ledger.demand(d);
  EXPECT_EQ(A::barrier(f.ledger),barrier);++d.sequence;d.hold=true;f.ledger.demand(d);
  EXPECT_GT(A::barrier(f.ledger),barrier);
  auto pause=f.permit(2);pause.geometry_committed=false;pause.phase="holding";
  EXPECT_FALSE(f.ledger.commit(pause));ASSERT_TRUE(A::latest(f.ledger));EXPECT_EQ(A::latest(f.ledger)->phase,"holding");
  EXPECT_FALSE(f.ledger.commit(f.permit(1)));EXPECT_EQ(A::latest(f.ledger)->phase,"holding");
}
TEST(ExecutionLedger,LateralDemandDoesNotTripStopBarrierButExplicitZeroStillDoes) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  Fixture f;f.ledger.candidate(f.spline,f.proposal);ASSERT_TRUE(f.ledger.commit(f.permit()));
  w::MotionDemand demand;demand.version=f.proposal.version;demand.transport_mode="isolated_mock";
  demand.sequence=1;demand.execution_id="execution";demand.velocity.linear.y=.2;
  const auto initial=A::barrier(f.ledger);f.ledger.demand(demand);EXPECT_EQ(A::barrier(f.ledger),initial);
  demand.sequence=2;demand.velocity.linear.y=-.2;f.ledger.demand(demand);EXPECT_EQ(A::barrier(f.ledger),initial);
  demand.sequence=3;demand.velocity.linear.y=0.;f.ledger.demand(demand);EXPECT_GT(A::barrier(f.ledger),initial);
}
TEST(ExecutionLedger,NativeMotionDemandMayUseNewerEvidenceButNeverBelowOwnerFloor) {
  Fixture f;auto p=f.permit();p.allowed=true;p.phase="tracking";p.execution_id="exec";
  p.sdk_session="sdk";p.sdk_arm_generation=2;p.control_epoch=3;p.validation_sequence=40;
  w::MotionDemand d;d.version=p.version;d.trajectory_id=p.trajectory_id;d.execution_id=p.execution_id;
  d.sdk_session=p.sdk_session;d.sdk_arm_generation=p.sdk_arm_generation;d.control_epoch=p.control_epoch;
  using A=scan_planner::ExecutionValidatorTestAccess;
  d.validation_sequence=41;EXPECT_TRUE(A::matches(d,p));
  d.validation_sequence=40;EXPECT_TRUE(A::matches(d,p));
  d.validation_sequence=39;EXPECT_FALSE(A::matches(d,p));
  d.validation_sequence=41;d.version.anchor_revision++;EXPECT_FALSE(A::matches(d,p));
}
TEST(ExecutionLedger,RefreshedCurveProofDoesNotInheritExpiredTwoHundredMillisecondFloor) {
  Fixture f;w::MotionDemand d;d.version=f.proposal.version;d.trajectory_id=1;d.validation_sequence=40;
  w::TrajectoryValidation old;old.version=d.version;old.trajectory_id=1;old.sequence=40;old.valid=true;
  old.check_end=rclcpp::Time(10000000000LL);old.valid_until=rclcpp::Time(10200000000LL);
  auto newer=old;newer.sequence=41;newer.check_end=rclcpp::Time(10200000000LL);newer.valid_until=rclcpp::Time(10400000000LL);
  std::deque<w::TrajectoryValidation> history{old,newer};
  const auto selected=scan_planner::ExecutionValidatorTestAccess::proof(history,d,10210000000LL);
  ASSERT_TRUE(selected);EXPECT_EQ(selected->sequence,41U);EXPECT_EQ(rclcpp::Time(selected->valid_until).nanoseconds(),10400000000LL);
  // Production motionEvidenceDeadline still caps the ORIGINAL command/source;
  // selecting a new map check does not create another 100 ms command lease.
  EXPECT_EQ(scan_planner::motionEvidenceDeadline(10190000000LL,10180000000LL,10200000000LL,
    10280000000LL,10400000000LL,rclcpp::Time(selected->valid_until).nanoseconds(),10400000000LL,
    10000000000LL,10000000000LL),10280000000LL);
}
TEST(ExecutionLedger,NegativeCurveRevisionFencesOldCommandEvenAfterPositiveRecovery) {
  Fixture f;w::MotionDemand d;d.version=f.proposal.version;d.trajectory_id=1;d.validation_sequence=40;
  w::TrajectoryValidation p;p.version=d.version;p.trajectory_id=1;p.sequence=40;p.valid=true;
  p.check_end=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10400000000LL);
  auto negative=p;negative.sequence=41;negative.valid=false;auto recovered=p;recovered.sequence=42;
  std::deque<w::TrajectoryValidation> history{p,negative,recovered};using A=scan_planner::ExecutionValidatorTestAccess;
  EXPECT_FALSE(A::proof(history,d,10200000000LL));d.validation_sequence=41;EXPECT_FALSE(A::proof(history,d,10200000000LL));
  d.validation_sequence=42;ASSERT_TRUE(A::proof(history,d,10200000000LL));
}
TEST(ExecutionLedger,MotionProofCannotBorrowOtherVersionCurveFutureOrExpiredNewestEvidence) {
  Fixture f;w::MotionDemand d;d.version=f.proposal.version;d.trajectory_id=1;d.validation_sequence=40;
  w::TrajectoryValidation p;p.version=d.version;p.trajectory_id=1;p.sequence=40;p.valid=true;
  p.check_end=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10400000000LL);
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(int mode=0;mode<6;++mode){auto bad=p;
    if(mode==0)++bad.version.anchor_revision;
    if(mode==1)++bad.trajectory_id;
    if(mode==2)bad.sequence=39;
    if(mode==3)bad.valid_until=rclcpp::Time(10200000000LL);
    if(mode==4)bad.check_end=rclcpp::Time(10200000001LL);
    if(mode==5)bad.valid=false;
    EXPECT_FALSE(A::proof({bad},d,10200000000LL));
  }
  auto latest=p;latest.sequence=41;latest.valid_until=rclcpp::Time(10200000000LL);
  EXPECT_FALSE(A::proof({p,latest},d,10200000000LL));
  auto foreign=latest;foreign.valid=false;++foreign.version.anchor_revision;
  ASSERT_TRUE(A::proof({p,foreign},d,10200000000LL));
}
TEST(ExecutionLedger,ActuallyCheckedRevisionCanSatisfyNewerOwnerFloorWithoutChangingAuthority) {
  Fixture f;auto p=f.permit();p.allowed=true;p.phase="tracking";p.execution_id="exec";p.validation_sequence=41;
  w::MotionDemand d;d.version=p.version;d.trajectory_id=p.trajectory_id;d.execution_id=p.execution_id;
  d.validation_sequence=40;using A=scan_planner::ExecutionValidatorTestAccess;
  EXPECT_FALSE(A::matches(d,p));EXPECT_TRUE(A::matches(d,p,41));
  EXPECT_FALSE(A::matches(d,p,39));++p.control_epoch;EXPECT_FALSE(A::matches(d,p,41));
}
namespace {
// This fixture injects only the result of the independent Native reader. It
// exercises the real software ledger, not a claimed raw-ray/physical proof.
struct LocalCommitFixture:Fixture {
  w::TrajectoryValidation proof;w::LocalTrajectoryCommit fact;
  scan_planner::SplineHeadingContract heading;
  scan_planner::UniformBspline curve;scan_planner::MeasuredBodyPose acquired;
  geometry_msgs::msg::Twist velocity;
  scan_planner::NativePipelineHealth pipeline_health;
  void useNotYetDeliveredBody(double curve_time=.05) {
    acquired.position=curve.evaluateDeBoorT(curve_time);acquired.source_stamp=10.+curve_time;
    const auto v=curve.getDerivative().evaluateDeBoorT(curve_time);
    velocity.linear.x=v.x();velocity.linear.y=v.y();velocity.linear.z=v.z();
    fact.source_stamp=rclcpp::Time(10010000000LL+std::llround(curve_time*1e9));
    fact.body_source_stamp=rclcpp::Time(10000000000LL+std::llround(curve_time*1e9));
    fact.measured_pose.position.x=acquired.position.x();fact.measured_pose.position.y=acquired.position.y();
    fact.measured_pose.position.z=acquired.position.z();fact.measured_twist=velocity;
    fact.curve_time=curve_time;fact.arc_length=.25*curve_time;
  }
  explicit LocalCommitFixture(const Eigen::Vector3d& offset=Eigen::Vector3d::Zero()):Fixture(false,true) {
    using A=scan_planner::ExecutionValidatorTestAccess;
    proposal.expected_trajectory_id=-1;
    EXPECT_TRUE(ledger.replacePendingReference(proposal));
    // Formal collision state and actual pipeline health are independent
    // injected facts. This passive ledger runs neither ROS nor map workers.
    auto context=std::make_shared<scan_planner::ProjectedRayContext>();
    context->session=proposal.version.session_id;context->seed=proposal.version.localization_seed_id;
    context->epoch=proposal.version.localization_epoch;context->sequence=proposal.version.context_sequence;
    context->frame="odom";pipeline_health.context=context;
    pipeline_health.source_ns={10000000000LL,10000000000LL};pipeline_health.fusion_sequence=1;
    pipeline_health.sampled_steady_ns=pipeline_health.fusion_completed_steady_ns=10000000000LL;
    A::pipelineHealth(ledger,[this]{return pipeline_health;});
    scan_planner::BrakingModel model{.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'a')};
    model.command_pipeline_bound=.1;A::braking(ledger,model);
    Eigen::MatrixXd points(3,23);
    for(int i=0;i<points.cols();++i)points.col(i)=offset+Eigen::Vector3d((i-1)*.05,0.,.55);
    curve=scan_planner::UniformBspline(points,3,.2);
    spline.frame_id="odom";spline.trajectory.order=3;
    for(int i=0;i<points.cols();++i) {geometry_msgs::msg::Point p;p.x=points(0,i);p.y=points(1,i);p.z=points(2,i);spline.trajectory.pos_pts.push_back(p);}
    const auto knots=curve.getKnot();for(int i=0;i<knots.size();++i)spline.trajectory.knots.push_back(knots[i]);
    heading.measured_heading_enabled=true;heading.measured_yaw=0.;
    const auto controls=curve.getDerivative().getControlPoint();double bound=0.;
    for(int i=0;i<controls.cols();++i)bound=std::max(bound,controls.col(i).norm());
    const auto n=scan_planner::bodyHeadingSamplingIntervals(curve.getTimeSum(),bound,.05);EXPECT_TRUE(n);
    spline.body_heading_seed_valid=true;spline.body_heading_seed_yaw=heading.measured_yaw;
    spline.body_heading_sampling_intervals=n.value_or(0);
    spline.join_source_stamp=rclcpp::Time(10000000000LL);
    spline.join_pose.position.x=offset.x();spline.join_pose.position.y=offset.y();
    spline.join_pose.position.z=.55+offset.z();spline.join_pose.orientation.w=1.;
    ledger.candidate(spline,proposal,heading);
    acquired={curve.evaluateDeBoorT(0.),Eigen::Quaterniond::Identity(),10.,"odom"};
    auto derivative=curve.getDerivative();const auto v=derivative.evaluateDeBoorT(0.);
    velocity.linear.x=v.x();velocity.linear.y=v.y();velocity.linear.z=v.z();
    ledger.body(acquired,velocity,10000000000LL);
    proof.version=proposal.version;proof.proposal_id=proposal.proposal_id;proof.trajectory_id=1;
    proof.sequence=1;proof.valid=true;proof.whole_curve=true;proof.map_snapshot_revision=33;
    proof.body_source_stamp=rclcpp::Time(10000000000LL);
    proof.source_stamp=proof.check_begin=proof.check_end=proof.body_source_stamp;
    proof.front_ray_source_stamp=proof.rear_ray_source_stamp=proof.body_source_stamp;
    proof.valid_until=rclcpp::Time(0LL);proof.frame_id="odom";
    proof.transport_mode="isolated_mock";proof.collision_policy="observed_free";
    A::recordLocalFact(ledger,proof);A::completedPipelineCheck(ledger,proof,pipeline_health.fusion_sequence);
    fact.schema_version=1;fact.version=proposal.version;fact.trajectory_id=1;fact.expected_trajectory_id=-1;
    fact.proposal_id=proposal.proposal_id;fact.commit_sequence=1;fact.source_stamp=rclcpp::Time(10010000000LL);
    fact.body_source_stamp=rclcpp::Time(10000000000LL);fact.measured_pose.position.x=acquired.position.x();
    fact.measured_pose.position.y=acquired.position.y();fact.measured_pose.position.z=acquired.position.z();
    fact.measured_pose.orientation.w=1.;fact.measured_twist=velocity;fact.native_validation_sequence=1;
    fact.map_geometry_revision=33;fact.accepted=true;fact.disposition="committed";fact.transport_mode="isolated_mock";
  }
};

// Use the production geometry commit ledger to reproduce reference preparation
// at curve 1 followed by an ordinary same-reference commit to curve 2.
void advanceLocalReferenceCurve(LocalCommitFixture& f) {
  ASSERT_TRUE(f.ledger.localCommitAt(f.fact,10020000000LL));
  auto next=f.spline;next.trajectory.traj_id=2;
  f.ledger.candidate(next,f.proposal,f.heading);
  auto proof=f.proof;proof.sequence=2;proof.trajectory_id=2;
  scan_planner::ExecutionValidatorTestAccess::recordLocalFact(f.ledger,proof);
  auto commit=f.fact;commit.trajectory_id=2;commit.commit_sequence=2;
  commit.native_validation_sequence=2;commit.expected_version=f.proposal.version;
  commit.expected_trajectory_id=1;
  ASSERT_TRUE(f.ledger.localCommitAt(commit,10020000000LL));
  EXPECT_EQ(f.ledger.committedTrajectory(),2);
}
}

TEST(LocalFollowerLedger,RollingReferenceAdmissionAllowsOnlyNaturalSameReferenceCurveAdvance) {
  LocalCommitFixture f;advanceLocalReferenceCurve(f);
  auto candidate=f.spline;candidate.trajectory.traj_id=3;
  f.ledger.candidate(candidate,f.proposal,f.heading);
  auto rolling=f.proposal;rolling.proposal_id="next-window";rolling.version.reference_generation=2;
  rolling.expected_version=f.proposal.version;rolling.expected_trajectory_id=1;
  // Strict callers still reject an earlier curve and preserve current work.
  EXPECT_FALSE(f.ledger.replacePendingReference(rolling));
  ASSERT_TRUE(scan_planner::ExecutionValidatorTestAccess::hasCandidate(f.ledger));
  EXPECT_EQ(scan_planner::ExecutionValidatorTestAccess::candidateReadSnapshot(f.ledger).spline.trajectory.traj_id,3);
  ASSERT_TRUE(f.ledger.replacePendingReference(rolling,true));
  // A normal moving window must not cancel the current admission check.
  // Rejection/expiry/commit, not input delivery, releases that bounded slot.
  ASSERT_TRUE(scan_planner::ExecutionValidatorTestAccess::hasCandidate(f.ledger));
  EXPECT_EQ(scan_planner::ExecutionValidatorTestAccess::candidateReadSnapshot(f.ledger).spline.trajectory.traj_id,3);
  EXPECT_EQ(f.ledger.committedTrajectory(),2);
  // Input admission must not rewrite the request's historical predecessor.
  EXPECT_EQ(rolling.expected_trajectory_id,1);
  EXPECT_EQ(rolling.expected_version,f.proposal.version);
  // Explicit completed-query evidence rejects candidate 3. This contract-only
  // fixture uses its existing proof ingress seam; the companion admission
  // timing fixture tests real GridMap occupancy and whole-curve queries.
  auto negative=f.proof;negative.sequence=3;negative.trajectory_id=3;
  negative.valid=false;negative.reason="occupied";
  scan_planner::ExecutionValidatorTestAccess::recordLocalFact(f.ledger,negative);
  // The next selected curve still commits against actual current curve 2.
  candidate.generation=2;candidate.trajectory.traj_id=4;
  f.ledger.candidate(candidate,rolling,f.heading);
  auto proof=f.proof;proof.version=rolling.version;proof.proposal_id=rolling.proposal_id;
  proof.sequence=4;proof.trajectory_id=4;
  scan_planner::ExecutionValidatorTestAccess::recordLocalFact(f.ledger,proof);
  auto commit=f.fact;commit.version=rolling.version;commit.proposal_id=rolling.proposal_id;
  commit.trajectory_id=4;commit.commit_sequence=3;commit.native_validation_sequence=4;
  commit.expected_version=f.proposal.version;commit.expected_trajectory_id=1;
  EXPECT_FALSE(f.ledger.localCommitAt(commit,10020000000LL));
  EXPECT_EQ(f.ledger.committedTrajectory(),2);
  commit.expected_trajectory_id=2;
  EXPECT_TRUE(f.ledger.localCommitAt(commit,10020000000LL));
  EXPECT_EQ(f.ledger.committedTrajectory(),4);
}

TEST(LocalFollowerLedger,RollingReferenceInputCannotCrossTaskAnchorContextOrFutureCurve) {
  for(unsigned kind=0;kind<14;++kind) {
    LocalCommitFixture f;advanceLocalReferenceCurve(f);
    auto candidate=f.spline;candidate.trajectory.traj_id=3;
    f.ledger.candidate(candidate,f.proposal,f.heading);
    auto rolling=f.proposal;rolling.proposal_id="next-window";rolling.version.reference_generation=2;
    rolling.expected_version=f.proposal.version;rolling.expected_trajectory_id=1;
    if(kind==0)rolling.version.task_id="another-task";
    if(kind==1)rolling.version.route_id="another-route";
    if(kind==2)rolling.version.route_hash="another-hash";
    if(kind==3)rolling.version.anchor_id="another-anchor";
    if(kind==4)++rolling.version.anchor_revision;
    if(kind==5)++rolling.version.context_sequence;
    if(kind==6)++rolling.version.localization_epoch;
    if(kind==7)rolling.version.localization_seed_id="another-seed";
    if(kind==8)rolling.version.reference_generation=1;
    if(kind==9)rolling.expected_trajectory_id=3;
    if(kind==10)rolling.expected_trajectory_id=-1;
    if(kind==11)rolling.expected_trajectory_id=0;
    if(kind==12)rolling.expected_version.reference_generation=2;
    if(kind==13)rolling.expected_version.anchor_id="another-anchor";
    EXPECT_FALSE(f.ledger.replacePendingReference(rolling,true))<<kind;
    ASSERT_TRUE(scan_planner::ExecutionValidatorTestAccess::hasCandidate(f.ledger))<<kind;
    EXPECT_EQ(scan_planner::ExecutionValidatorTestAccess::candidateReadSnapshot(f.ledger).spline.trajectory.traj_id,3)<<kind;
    EXPECT_EQ(f.ledger.committedTrajectory(),2)<<kind;
  }
}

TEST(LocalFollowerLedger,RollingReferenceInputCannotReopenRetiredOrUninitializedTask) {
  LocalCommitFixture f;advanceLocalReferenceCurve(f);
  auto rolling=f.proposal;rolling.version.reference_generation=3;
  rolling.expected_version=f.proposal.version;rolling.expected_trajectory_id=1;
  auto barrier=f.proposal.version;barrier.reference_generation=2;
  ASSERT_TRUE(f.ledger.cancelReference(barrier));
  EXPECT_FALSE(f.ledger.replacePendingReference(rolling,true));
  EXPECT_EQ(f.ledger.committedTrajectory(),-1);
  Fixture empty(false,true);auto initial=empty.proposal;initial.expected_trajectory_id=-1;
  EXPECT_FALSE(empty.ledger.replacePendingReference(initial,true));
  EXPECT_TRUE(empty.ledger.replacePendingReference(initial));
  Fixture legacy;legacy.ledger.candidate(legacy.spline,legacy.proposal);
  ASSERT_TRUE(legacy.ledger.commit(legacy.permit()));
  auto not_formal=legacy.proposal;not_formal.version.reference_generation=2;
  not_formal.expected_version=legacy.proposal.version;not_formal.expected_trajectory_id=1;
  EXPECT_FALSE(legacy.ledger.replacePendingReference(not_formal,true));
}

TEST(LocalFollowerLedger,ClassifiedStairPortalKeepsIncumbentButStillRequiresExactCoordinateAndTaskIdentity) {
  for(unsigned fault=0;fault<5;++fault) {
    LocalCommitFixture f;advanceLocalReferenceCurve(f);
    auto rolling=f.proposal;rolling.proposal_id="stair-window";rolling.version.reference_generation=2;
    rolling.version.segment_id="stair_lower";
    rolling.expected_version=f.proposal.version;rolling.expected_trajectory_id=1;
    EXPECT_FALSE(f.ledger.replacePendingReference(rolling,true)); // never implicit cross-segment
    if(fault==1)rolling.version.anchor_revision++;
    if(fault==2)rolling.version.task_id="foreign";
    if(fault==3)rolling.expected_trajectory_id=3;
    if(fault==4)rolling.version.map_geometry_revision++;
    EXPECT_FALSE(f.ledger.replacePendingReference(rolling,true,true)); // a flag is not a classified predecessor
    auto unknown=f.proposal.version;unknown.segment_id="unadmitted-segment";
    EXPECT_FALSE(f.ledger.replacePendingReference(rolling,true,true,unknown));
    EXPECT_EQ(f.ledger.replacePendingReference(rolling,true,true,f.proposal.version),fault==0)<<fault;
    EXPECT_EQ(f.ledger.committedTrajectory(),2)<<fault;
    EXPECT_EQ(f.ledger.committedVersion()->segment_id,f.proposal.version.segment_id)<<fault;
  }
}

TEST(LocalFollowerLedger,PendingStairPortalLineageSurvivesFailedCandidateWithoutPromotingPendingVersion) {
  for(bool classified:{false,true}) {
    LocalCommitFixture f;advanceLocalReferenceCurve(f);
    auto first=f.proposal;first.proposal_id="portal";first.version.reference_generation=2;
    first.version.segment_id="stair_lower";first.expected_version=f.proposal.version;first.expected_trajectory_id=2;
    ASSERT_TRUE(f.ledger.replacePendingReference(first,classified,classified,f.proposal.version));
    f.ledger.cancelPending(); // Candidate fails; the old curve remains the actual owner.
    auto next=first;next.proposal_id="sliding-stair";next.version.reference_generation=3;
    auto changed=next;++changed.version.anchor_revision;
    EXPECT_FALSE(f.ledger.replacePendingReference(changed,true));
    changed=next;changed.expected_version=first.version;
    EXPECT_FALSE(f.ledger.replacePendingReference(changed,true)); // pending is never an incumbent
    EXPECT_EQ(f.ledger.replacePendingReference(next,true),classified);
    if(classified)for(unsigned generation=4;generation<8;++generation) {
      f.ledger.cancelPending();next.version.reference_generation=generation;
      next.proposal_id="retry"+std::to_string(generation);
      changed=next;changed.version.task_id="next-task";
      EXPECT_FALSE(f.ledger.replacePendingReference(changed,true));
      ASSERT_TRUE(f.ledger.replacePendingReference(next,true));
      EXPECT_EQ(f.ledger.committedTrajectory(),2);
    }
    EXPECT_EQ(f.ledger.committedTrajectory(),2);
    EXPECT_EQ(f.ledger.committedVersion(),f.proposal.version);
  }
}

TEST(LocalFollowerLedger,StairPortalInputCannotDisplaceAnInFlightPreparedCurve) {
  LocalCommitFixture f;advanceLocalReferenceCurve(f);
  w::SupportReference support;support.version=f.proposal.version;support.verified=true;
  support.support_reference_id="support";support.support_hash="support_hash";f.ledger.support(support);
  auto candidate=f.spline;candidate.trajectory.traj_id=3;
  f.ledger.candidate(candidate,f.proposal,f.heading);
  auto proof=f.proof;proof.sequence=3;proof.trajectory_id=3;
  scan_planner::ExecutionValidatorTestAccess::recordLocalFact(f.ledger,proof);
  auto request=f.fact;request.trajectory_id=3;request.commit_sequence=3;request.native_validation_sequence=3;
  request.expected_version=f.proposal.version;request.expected_trajectory_id=2;
  request.accepted=false;request.disposition="prepare";
  ASSERT_TRUE(f.ledger.localCommitExchangeAt(request,10020000000LL).accepted);
  ASSERT_TRUE(f.ledger.localCommitInFlight());
  auto portal=f.proposal;portal.version.reference_generation=2;portal.version.segment_id="stair_lower";
  portal.expected_version=f.proposal.version;portal.expected_trajectory_id=2;
  EXPECT_FALSE(f.ledger.replacePendingReference(portal,true,true,f.proposal.version));
  EXPECT_TRUE(f.ledger.localCommitInFlight());EXPECT_EQ(f.ledger.committedTrajectory(),2);
  request.accepted=true;request.disposition="committed";
  ASSERT_TRUE(f.ledger.localCommitExchangeAt(request,10020000000LL).accepted);
  EXPECT_EQ(f.ledger.committedTrajectory(),3);
  // Original proposal may now name curve 2; ordinary same-version advance is
  // still accepted, without pretending its uncommitted segment was active.
  EXPECT_TRUE(f.ledger.replacePendingReference(portal,true,true,f.proposal.version));
}

TEST(LocalFollowerLedger,ClassifiedPortalUsesLiteralAdmittedPendingReferenceLineage) {
  LocalCommitFixture f;advanceLocalReferenceCurve(f);
  auto floor=f.proposal;floor.proposal_id="sliding-floor";floor.version.reference_generation=2;
  floor.expected_version=f.proposal.version;floor.expected_trajectory_id=2;
  ASSERT_TRUE(f.ledger.replacePendingReference(floor,true));
  auto stair=floor;stair.proposal_id="floor-stair";stair.version.reference_generation=3;
  stair.version.segment_id="stair_lower";
  auto unadmitted=floor.version;unadmitted.reference_generation=20;
  EXPECT_FALSE(f.ledger.replacePendingReference(stair,true,true,unadmitted));
  ASSERT_TRUE(f.ledger.replacePendingReference(stair,true,true,floor.version));
  f.ledger.cancelPending();
  auto landing=stair;landing.proposal_id="stair-landing";landing.version.reference_generation=4;
  landing.version.segment_id="landing";
  // The caller classified stair->landing. A fabricated or stale predecessor
  // cannot substitute for the literal stair window that was acknowledged.
  unadmitted=stair.version;unadmitted.segment_id="unknown-stair";
  EXPECT_FALSE(f.ledger.replacePendingReference(landing,true,true,unadmitted));
  EXPECT_FALSE(f.ledger.replacePendingReference(landing,true,true,floor.version));
  ASSERT_TRUE(f.ledger.replacePendingReference(landing,true,true,stair.version));
  EXPECT_EQ(f.ledger.committedTrajectory(),2);
  EXPECT_EQ(f.ledger.committedVersion(),f.proposal.version);
}

TEST(LocalFollowerLedger,PermitCannotPromoteCandidateAndHistoricalExactCommitDoesNotRenewState) {
  LocalCommitFixture f;ASSERT_TRUE(f.ledger.commit(f.permit()));EXPECT_EQ(f.ledger.committedTrajectory(),-1);
  ASSERT_TRUE(f.ledger.localCommitAt(f.fact,10610000000LL));EXPECT_EQ(f.ledger.committedTrajectory(),1);
  ASSERT_TRUE(f.ledger.committedView()->proof);
  EXPECT_EQ(rclcpp::Time(f.ledger.committedView()->proof->valid_until).nanoseconds(),0LL);
  EXPECT_EQ(f.ledger.committedView()->proof->check_end,f.proof.check_end);
  EXPECT_FALSE(f.ledger.localCommitAt(f.fact,10620000000LL));
}
namespace {
// The real scheduling ledger receives immutable reader facts here; this is
// not another collision checker or a claim that fabricated rays were observed.
struct PendingSpliceScheduleFixture:LocalCommitFixture {
  static constexpr std::int64_t now=10200000000LL;
  w::TaggedBspline tail;
  w::TrajectoryValidation current,tail_proof;
  w::TrackingProgress measured;
  explicit PendingSpliceScheduleFixture(double beyond=.02,double lateral=0.) {
    using A=scan_planner::ExecutionValidatorTestAccess;
    EXPECT_TRUE(ledger.localCommitAt(fact,10010000000LL));
    w::SupportReference support;support.version=proposal.version;support.verified=true;
    support.support_reference_id="support";support.support_hash="support_hash";ledger.support(support);
    tail=spline;tail.trajectory.traj_id=2;tail.trajectory.start_time=rclcpp::Time(now);
    constexpr double boundary_time=.1;
    const double boundary_x=curve.evaluateDeBoorT(boundary_time).x();
    for(auto& point:tail.trajectory.pos_pts)point.x+=boundary_x;
    auto& splice=tail.splice;splice.enabled=true;splice.incumbent_version=proposal.version;
    splice.incumbent_trajectory_id=1;splice.splice_curve_time=boundary_time;
    splice.splice_arc_length=boundary_x;splice.position.x=boundary_x;splice.position.z=.55;
    splice.velocity.x=.25;
    ledger.candidate(tail,proposal,heading);
    acquired.position={boundary_x+beyond,lateral,.55};acquired.source_stamp=now*1e-9;
    ledger.bodyAt(acquired,velocity,now,now);
    measured=progress();measured.holding=false;measured.curve_time=(boundary_x+beyond)/.25;
    measured.arc_length=measured.s_committed=boundary_x+beyond;measured.header.stamp=rclcpp::Time(now);
    measured.pose.position.x=acquired.position.x();measured.pose.position.y=lateral;measured.pose.position.z=.55;
    measured.twist=velocity;ledger.progressAt(measured,now*1e-9);
    EXPECT_TRUE(ledger.committedView()->progress.has_value());
    auto owner=permit();owner.source_stamp=rclcpp::Time(now);owner.valid_until=rclcpp::Time(now+250000000LL);
    EXPECT_TRUE(ledger.commit(owner));
    current=proof;current.sequence=2;current.check_end=current.body_source_stamp=rclcpp::Time(now);
    current.front_ray_source_stamp=current.rear_ray_source_stamp=current.check_end;
    current.valid_until=rclcpp::Time(0LL);current.frame_id="odom";
    current.transport_mode="isolated_mock";current.collision_policy="observed_free";
    current.support_reference_id=support.support_reference_id;current.support_hash=support.support_hash;
    EXPECT_TRUE(A::recordCommitted(ledger,current));
    tail_proof=current;tail_proof.sequence=3;tail_proof.trajectory_id=2;tail_proof.splice=splice;
    A::recordLocalFact(ledger,tail_proof);
  }
  auto disposition(std::int64_t at=now)const {return ledger.pendingSpliceDispositionAt(proposal.version,at,.05);}
};
}
TEST(LocalFollowerSpliceScheduling,AlongCurveControlTickOvershootKeepsSameNativeAdmissibleTail) {
  for(const double beyond:{.02,.049}) {
    SCOPED_TRACE(beyond);PendingSpliceScheduleFixture f(beyond);
    auto tail=scan_planner::decodeSpliceCurve(f.tail.trajectory);ASSERT_TRUE(tail);
    ASSERT_TRUE(scan_planner::measuredSpliceTailEntry(*tail,f.acquired.position,.05));
    const auto before=f.ledger.committedView();ASSERT_TRUE(before);
    const auto disposition=f.disposition();EXPECT_TRUE(disposition.keep)<<disposition.reason;
    EXPECT_STREQ(disposition.reason,"pending_splice_waiting_measured_entry");
    const auto after=f.ledger.committedView();ASSERT_TRUE(after);
    EXPECT_EQ(before->spline,after->spline);EXPECT_EQ(before->proof,after->proof);
    EXPECT_EQ(before->progress,after->progress);EXPECT_EQ(f.ledger.committedTrajectory(),1);
  }
}
TEST(LocalFollowerSpliceScheduling,ProjectionUsesBodyPairedWithProgressNotLaterInboxPose) {
  PendingSpliceScheduleFixture f;
  auto later=f.acquired;later.source_stamp+=.02;later.position.y()=1.;
  f.ledger.bodyAt(later,f.velocity,f.now+20000000LL,f.now+20000000LL);
  const auto disposition=f.disposition(f.now+20000000LL);
  EXPECT_TRUE(disposition.keep)<<disposition.reason;
  EXPECT_EQ(f.ledger.committedView()->progress->header.stamp,f.measured.header.stamp);
}
TEST(LocalFollowerSpliceScheduling,OutsideOriginalDomainAndRealInvalidationNeverKeepPending) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  for(unsigned variant=0;variant<8;++variant) {
    SCOPED_TRACE(variant);PendingSpliceScheduleFixture f(variant==0?.051:.02,variant==1?.013:0.);
    if(variant==2){auto p=f.tail_proof;p.valid=false;p.reason="occupied";A::recordLocalFact(f.ledger,p);}
    // Nonzero means the incompatible legacy lease, not formal-state expiry.
    if(variant==3){auto p=f.tail_proof;p.valid_until=rclcpp::Time(f.now);A::recordLocalFact(f.ledger,p);}
    if(variant==4){auto p=f.permit(2);p.revoked=true;EXPECT_TRUE(f.ledger.commit(p));}
    if(variant==5){auto v=f.proposal.version;++v.reference_generation;EXPECT_TRUE(f.ledger.cancelReference(v));}
    if(variant==6){auto p=f.current;++p.sequence;p.valid=false;p.reason="occupied";EXPECT_TRUE(A::recordCommitted(f.ledger,p));}
    const auto at=variant==7?f.now+401000000LL:f.now;
    EXPECT_FALSE(f.disposition(at).keep);
  }
}
TEST(LocalFollowerLedger,MissingHistoricalBodyDoesNotGateGeometryOrManufactureState) {
  LocalCommitFixture f;f.useNotYetDeliveredBody();const auto original=f.fact;
  ASSERT_TRUE(f.ledger.localCommitAt(original,10070000000LL));
  ASSERT_EQ(f.ledger.committedTrajectory(),1);ASSERT_TRUE(f.ledger.committedView()->proof);
  EXPECT_EQ(scan_planner::ExecutionValidatorTestAccess::bodySource(f.ledger),10.);
  EXPECT_EQ(scan_planner::ExecutionValidatorTestAccess::bodyHistorySize(f.ledger),1U);
  EXPECT_EQ(f.ledger.committedView()->proof->valid_until,f.proof.valid_until);
  EXPECT_EQ(f.ledger.committedView()->proof->sequence,original.native_validation_sequence);
  // The source is permanently absent, including after a later real sample.
  auto successor=f.acquired;successor.source_stamp=10.10;
  f.ledger.body(successor,f.velocity,10100000000LL);
  EXPECT_EQ(f.ledger.committedTrajectory(),1);
  EXPECT_FALSE(f.ledger.localCommitAt(original,10610000000LL));
}
TEST(LocalFollowerLedger,RollingCommitUsesMeasuredGeometryNotDerivativeVelocityEquality) {
  LocalCommitFixture f;f.useNotYetDeliveredBody(.05);
  f.acquired.position.y()=.04;f.velocity.linear.x=.08;f.velocity.linear.y=.03;
  f.fact.measured_pose.position.y=.04;f.fact.measured_twist=f.velocity;
  f.ledger.body(f.acquired,f.velocity,10050000000LL);
  const auto source=f.fact.body_source_stamp;
  ASSERT_TRUE(f.ledger.localCommitAt(f.fact,10070000000LL));
  ASSERT_TRUE(f.ledger.committedView()->proof);
  EXPECT_EQ(f.ledger.committedView()->proof->valid_until,f.proof.valid_until);
  EXPECT_EQ(f.fact.body_source_stamp,source);
  EXPECT_EQ(scan_planner::ExecutionValidatorTestAccess::bodySource(f.ledger),10.05);
}
TEST(LocalFollowerLedger,RollingCommitCannotUseRemoteArcOtherFloorOrUnprovedSource) {
  for(unsigned variant=0;variant<7;++variant) {
    SCOPED_TRACE(variant);LocalCommitFixture f;f.useNotYetDeliveredBody(.05);
    if(variant==0)f.fact.measured_pose.position.y=.151;
    if(variant==1)f.fact.measured_pose.position.z+=3.;
    if(variant==2) {f.fact.curve_time=2.4;f.fact.arc_length=.6;
      f.fact.measured_pose.position.x=f.curve.evaluateDeBoorT(2.4).x();}
    if(variant==3)f.fact.arc_length+=.1;
    if(variant==4) {f.proof.body_source_stamp=rclcpp::Time(10060000000LL);
      f.fact.native_validation_sequence=2;f.proof.sequence=2;
      scan_planner::ExecutionValidatorTestAccess::recordLocalFact(f.ledger,f.proof);}
    if(variant==5) {f.proof.valid=false;
      // Remove the one historical positive: geometry cannot invent one.
      f.fact.native_validation_sequence=2;f.proof.sequence=2;
      scan_planner::ExecutionValidatorTestAccess::recordLocalFact(f.ledger,f.proof);}
    if(variant==6) {f.proof.valid_start_time=2.4;f.proof.valid_start_arc_length=.6;
      f.fact.native_validation_sequence=2;f.proof.sequence=2;
      scan_planner::ExecutionValidatorTestAccess::recordLocalFact(f.ledger,f.proof);}
    EXPECT_FALSE(f.ledger.localCommitAt(f.fact,10070000000LL));
  }
}
TEST(LocalFollowerLedger,RollingCandidateRecheckUsesFixedOriginAndActualNativeConnector) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  const auto run=[](bool obstacle,double lateral,double age) {
    LocalCommitFixture f({.8,1.,0.});
    w::SupportReference support;support.version=f.proposal.version;support.verified=true;
    support.frame_id="odom";support.support_reference_id="support";support.support_hash="hash";
    f.ledger.support(support);
    A::clearInjectedCandidateProof(f.ledger);
    auto body=f.acquired;body.position.x()+=.025;body.position.y()+=lateral;body.source_stamp=10.+age;
    const auto source=10000000000LL+std::llround(age*1e9);f.ledger.body(body,f.velocity,source);
    auto map=GridMapTestAccess::headingFixture(false,true);
    GridMapTestAccess::acquiredSnapshot(*map,source,source+10000000LL);
    A::officialGeometry(f.ledger);
    if(obstacle)map->setOccupied({.825,1.225,.575});
    return A::checkWithClock(f.ledger,*map,[=]{return source+10000000LL;});
  };
  auto clear=run(false,.12,.05);ASSERT_TRUE(clear);EXPECT_TRUE(clear->valid)<<clear->reason;
  EXPECT_TRUE(clear->whole_curve);EXPECT_FALSE(clear->remaining_curve);
  auto occupied=run(true,.12,.05);ASSERT_TRUE(occupied);EXPECT_FALSE(occupied->valid);
  EXPECT_EQ(occupied->reason,"occupied");
  auto far=run(false,.151,.05);ASSERT_TRUE(far);EXPECT_FALSE(far->valid);
  // Rechecking current geometry does not expire solely because its immutable
  // tagged origin is older. The acquired body/rays here are genuinely fresh.
  auto later=run(false,.04,.6);ASSERT_TRUE(later);EXPECT_TRUE(later->valid)<<later->reason;
}
TEST(LocalFollowerLedger,RollingCandidateRepeatedProofCannotRatchetBeyondOriginalHalfMeter) {
  using A=scan_planner::ExecutionValidatorTestAccess;
  LocalCommitFixture f;auto body=f.acquired;body.source_stamp=10.2;body.position.x()=.7;
  A::candidateMeasuredTime(f.ledger,2.); // A later proof is not a new origin.
  auto first=A::join(f.ledger,f.curve,body,false);EXPECT_FALSE(first.time);
  body.position={.05,0.,3.55};EXPECT_FALSE(A::join(f.ledger,f.curve,body,false).time);
  body=f.acquired;body.position.y()=.01; // changed pose under unchanged source
  EXPECT_FALSE(A::join(f.ledger,f.curve,body,false).time);
}
TEST(LocalFollowerLedger,ExactRawPoseWithoutTwistChecksPoseButDoesNotBlockGeometry) {
  LocalCommitFixture f;f.useNotYetDeliveredBody();const auto source=rclcpp::Time(f.fact.body_source_stamp).nanoseconds();
  f.ledger.body(f.acquired,{},source);ASSERT_TRUE(f.ledger.localCommitAt(f.fact,10070000000LL));
  ASSERT_EQ(f.ledger.committedTrajectory(),1);
  EXPECT_EQ(f.ledger.committedView()->proof->valid_until,f.proof.valid_until);
  LocalCommitFixture contradictory;contradictory.useNotYetDeliveredBody();
  auto raw=contradictory.acquired;raw.position.y()=.002;contradictory.ledger.body(raw,{},source);
  EXPECT_FALSE(contradictory.ledger.localCommitAt(contradictory.fact,10070000000LL));
}
TEST(LocalFollowerLedger,InterpolatedHistoricalPoseIsNotAnExactRawWitnessOrMotionEvidence) {
  LocalCommitFixture f;f.useNotYetDeliveredBody();
  auto successor=f.acquired;successor.source_stamp=10.10;successor.position.y()=.05;
  f.ledger.body(successor,f.velocity,10100000000LL);
  // Interpolation at the missing 10.05 stamp would disagree with the actual
  // LocalFollower fact. It is neither an acquired sample nor a second owner.
  ASSERT_TRUE(f.ledger.localCommitAt(f.fact,10200000000LL));
  EXPECT_EQ(f.ledger.committedTrajectory(),1);
  EXPECT_EQ(scan_planner::ExecutionValidatorTestAccess::bodySource(f.ledger),10.10);
  EXPECT_EQ(scan_planner::ExecutionValidatorTestAccess::bodyHistorySize(f.ledger),2U);
  ASSERT_TRUE(f.ledger.committedView()->proof);
  EXPECT_EQ(f.ledger.committedView()->proof->check_end,f.proof.check_end);
  EXPECT_EQ(f.ledger.committedView()->proof->valid_until,f.proof.valid_until);
}
TEST(LocalFollowerLedger,CommittedFactCannotBeReplacedWithoutActualPredecessorCas) {
  LocalCommitFixture f;f.useNotYetDeliveredBody();const auto original=f.fact;
  ASSERT_TRUE(f.ledger.localCommitAt(original,10070000000LL));
  auto conflict=original;conflict.measured_pose.position.y=.002;
  EXPECT_FALSE(f.ledger.localCommitAt(conflict,10070000000LL));
  ++conflict.commit_sequence;EXPECT_FALSE(f.ledger.localCommitAt(conflict,10070000000LL));
  EXPECT_EQ(f.ledger.committedTrajectory(),1);
  EXPECT_FALSE(f.ledger.localCommitAt(original,10070000000LL));
}
TEST(LocalFollowerLedger,MissingBodyFactStillHonorsCancelReferenceAndTaskRevocation) {
  for(unsigned mode=0;mode<5;++mode) {
    if(mode==1)continue; // Ordinary successors now queue; they are not revocations.
    LocalCommitFixture f;f.useNotYetDeliveredBody();
    if(mode==0){auto barrier=f.proposal.version;++barrier.reference_generation;ASSERT_TRUE(f.ledger.cancelReference(barrier));}
    if(mode==2)f.ledger.cancelPending();
    if(mode==3){ASSERT_TRUE(f.ledger.replacePendingReference(f.proposal));}
    if(mode==4){w::TaskExecutionLease lease;lease.schema_version=1;lease.version=f.proposal.version;
      lease.sequence=1;lease.source_stamp=rclcpp::Time(10070000000LL);lease.valid_until=rclcpp::Time(10400000000LL);
      lease.revoked=true;lease.transport_mode="isolated_mock";ASSERT_TRUE(f.ledger.taskLeaseAt(lease,10070000000LL));}
    EXPECT_FALSE(f.ledger.localCommitAt(f.fact,10070000000LL))<<mode;
    EXPECT_EQ(f.ledger.committedTrajectory(),-1)<<mode;
  }
}
TEST(LocalFollowerLedger,NewCandidateDoesNotRetireSelectedOriginalSoftwareFact) {
  LocalCommitFixture f;f.useNotYetDeliveredBody();
  auto successor=f.spline;successor.trajectory.traj_id=2;
  f.ledger.candidate(successor,f.proposal,f.heading);
  // This commits immutable geometry only. Missing body evidence still cannot
  // authorize motion; the selected original native proof is not rewritten.
  EXPECT_TRUE(f.ledger.localCommitAt(f.fact,10070000000LL));
  EXPECT_EQ(f.ledger.committedTrajectory(),1);
}
TEST(LocalFollowerLedger,ContradictoryExactRawPoseAndTwistStillRejectSoftwareGeometryFact) {
  LocalCommitFixture f;f.useNotYetDeliveredBody();
  auto actual=f.acquired;actual.position.y()=.002;
  f.ledger.body(actual,f.velocity,rclcpp::Time(f.fact.body_source_stamp).nanoseconds());
  EXPECT_FALSE(f.ledger.localCommitAt(f.fact,10070000000LL));
  EXPECT_EQ(f.ledger.committedTrajectory(),-1);
  auto accurate=f.fact;accurate.commit_sequence=2;accurate.measured_pose.position.y=.002;
  EXPECT_TRUE(f.ledger.localCommitAt(accurate,10070000000LL));
  EXPECT_EQ(f.ledger.committedTrajectory(),1);
  LocalCommitFixture wrong_velocity;wrong_velocity.useNotYetDeliveredBody();
  auto raw_velocity=wrong_velocity.velocity;raw_velocity.linear.y=.002;
  wrong_velocity.ledger.body(wrong_velocity.acquired,raw_velocity,rclcpp::Time(wrong_velocity.fact.body_source_stamp).nanoseconds());
  EXPECT_FALSE(wrong_velocity.ledger.localCommitAt(wrong_velocity.fact,10070000000LL));
}
TEST(LocalFollowerLedger,CommitNeedsOriginalNativeEvidenceLegalSourceAndExactCurrentCandidate) {
  for(int mode=0;mode<10;++mode) {
    LocalCommitFixture f;auto bad=f.fact;
    if(mode==0)++bad.native_validation_sequence;
    if(mode==1)++bad.map_geometry_revision;
    if(mode==2)bad.expected_trajectory_id=1;
    if(mode==3)bad.version.task_id="other";
    if(mode==4)bad.measured_pose.position.y=.02;
    if(mode==5)bad.measured_twist.linear.y=.06;
    if(mode==6)bad.source_stamp=rclcpp::Time(10260000000LL);
    if(mode==7)bad.body_source_stamp=rclcpp::Time(10031000000LL);
    if(mode==8)bad.proposal_id="other";
    if(mode==9)bad.retired=true;
    EXPECT_FALSE(f.ledger.localCommitAt(bad,10610000000LL))<<mode;EXPECT_EQ(f.ledger.committedTrajectory(),-1);
    EXPECT_TRUE(f.ledger.localCommitAt(f.fact,10610000000LL))<<mode;
  }
}
TEST(LocalFollowerLedger,CancelledAndLateOldGeometryCannotReviveOrOverwriteCurrentCurve) {
  LocalCommitFixture f;ASSERT_TRUE(f.ledger.localCommitAt(f.fact,10020000000LL));
  auto tag=f.spline;tag.trajectory.traj_id=2;f.ledger.candidate(tag,f.proposal,f.heading);
  auto proof=f.proof;proof.sequence=2;proof.trajectory_id=2;
  scan_planner::ExecutionValidatorTestAccess::recordLocalFact(f.ledger,proof);
  auto replacement=f.fact;replacement.trajectory_id=2;replacement.commit_sequence=2;
  replacement.native_validation_sequence=2;replacement.expected_version=f.proposal.version;replacement.expected_trajectory_id=1;
  EXPECT_FALSE(f.ledger.localCommitAt(f.fact,10020000000LL));EXPECT_EQ(f.ledger.committedTrajectory(),1);
  ASSERT_TRUE(f.ledger.localCommitAt(replacement,10020000000LL));EXPECT_EQ(f.ledger.committedTrajectory(),2);
  auto barrier=f.proposal.version;++barrier.reference_generation;ASSERT_TRUE(f.ledger.cancelReference(barrier));
  ++replacement.commit_sequence;EXPECT_FALSE(f.ledger.localCommitAt(replacement,10030000000LL));
  EXPECT_EQ(f.ledger.committedTrajectory(),-1);
}
TEST(LocalFollowerLedger,HistoricalCommitDoesNotOverwriteLaterNegativeVerdict) {
  LocalCommitFixture f;auto negative=f.proof;negative.sequence=2;negative.valid=false;negative.reason="occupied";
  scan_planner::ExecutionValidatorTestAccess::previewProof(f.ledger,negative);
  ASSERT_TRUE(f.ledger.localCommitAt(f.fact,10610000000LL));ASSERT_TRUE(f.ledger.committedView()->proof);
  EXPECT_FALSE(f.ledger.committedView()->proof->valid);EXPECT_EQ(f.ledger.committedView()->proof->sequence,2U);
}
TEST(LocalFollowerLedger,CommandKeepsOriginalStateTimesAndUsesTaskLeaseSequenceNotGeometrySequence) {
  LocalCommitFixture f;ASSERT_TRUE(f.ledger.localCommitAt(f.fact,10020000000LL));
  EXPECT_TRUE(scan_planner::ExecutionValidatorTestAccess::pipelineReady(f.ledger,f.proposal.version,10020000000LL));
  w::TaskExecutionLease lease;lease.schema_version=1;lease.version=f.proposal.version;lease.sequence=5;
  lease.source_stamp=rclcpp::Time(10000000000LL);lease.valid_until=rclcpp::Time(10500000000LL);
  lease.allowed=true;lease.execution_id="exec";lease.control_epoch=3;lease.confirmation_id="click";
  lease.sdk_session="sdk";lease.sdk_arm_generation=2;lease.transport_mode="isolated_mock";
  ASSERT_TRUE(f.ledger.taskLeaseAt(lease,10020000000LL));
  scan_planner::BrakingModel model{.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'a')};
  model.command_pipeline_bound=.1;scan_planner::ExecutionValidatorTestAccess::braking(f.ledger,model);
  w::MotionDemand d;d.version=f.proposal.version;d.trajectory_id=1;d.sequence=20;d.permit_sequence=77;
  d.execution_id="exec";d.control_epoch=3;d.sdk_session="sdk";d.sdk_arm_generation=2;
  d.source_stamp=rclcpp::Time(10010000000LL);d.body_source_stamp=rclcpp::Time(10000000000LL);
  d.valid_until=rclcpp::Time(10210000000LL);d.velocity.linear.x=.2;d.transport_mode="isolated_mock";
  auto out=scan_planner::ExecutionValidatorTestAccess::localCommand(f.ledger,d,10150000000LL,10020000000LL);
  ASSERT_TRUE(out);EXPECT_EQ(out->lease_sequence,5U);EXPECT_NE(out->lease_sequence,d.permit_sequence);
  EXPECT_EQ(out->source_stamp,d.source_stamp);EXPECT_EQ(out->body_source_stamp,d.body_source_stamp);
  EXPECT_EQ(rclcpp::Time(out->valid_until).nanoseconds(),10100000000LL);
  // A newer task heartbeat arrives DURING native validation. Its timestamp is
  // after the original demand and must not be attached retroactively. SDK's
  // unchanged causal check can still accept lease5/source10.0 + demand10.01.
  auto heartbeat=lease;heartbeat.sequence=6;heartbeat.source_stamp=rclcpp::Time(10030000000LL);
  heartbeat.valid_until=rclcpp::Time(10530000000LL);
  ASSERT_TRUE(f.ledger.taskLeaseAt(heartbeat,10035000000LL));
  out=scan_planner::ExecutionValidatorTestAccess::localCommand(f.ledger,d,10150000000LL,10035000000LL);
  ASSERT_TRUE(out);EXPECT_EQ(out->lease_sequence,5U);EXPECT_EQ(out->source_stamp,d.source_stamp);
  auto future_only=d;future_only.source_stamp=rclcpp::Time(9999999999LL);
  EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::localCommand(f.ledger,future_only,10150000000LL,10035000000LL));
  EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::localCommand(f.ledger,d,10150000000LL,10100000000LL));
  ++d.sdk_arm_generation;EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::localCommand(f.ledger,d,10150000000LL,10020000000LL));
  // A real input clock fault still blocks the independent health gate even
  // though the same completed collision state remains positive and untimed.
  f.pipeline_health.clock_fault=true;
  EXPECT_FALSE(scan_planner::ExecutionValidatorTestAccess::pipelineReady(f.ledger,f.proposal.version,10035000000LL));
  EXPECT_TRUE(f.ledger.committedView()->proof->valid);
  EXPECT_EQ(rclcpp::Time(f.ledger.committedView()->proof->valid_until).nanoseconds(),0LL);
}
TEST(LocalFollowerLedger,TaskLeaseHistoryIsBoundedAndCurrentHoldFencesEveryHistoricalGrant) {
  LocalCommitFixture f;ASSERT_TRUE(f.ledger.localCommitAt(f.fact,10020000000LL));
  w::TaskExecutionLease lease;lease.schema_version=1;lease.version=f.proposal.version;
  lease.valid_until=rclcpp::Time(10700000000LL);lease.allowed=true;lease.execution_id="exec";
  lease.control_epoch=3;lease.confirmation_id="click";lease.sdk_session="sdk";lease.sdk_arm_generation=2;
  lease.transport_mode="isolated_mock";
  for(int i=1;i<=10;++i){lease.sequence=i;lease.source_stamp=rclcpp::Time(10000000000LL+i*1000000LL);
    ASSERT_TRUE(f.ledger.taskLeaseAt(lease,10020000000LL));}
  using A=scan_planner::ExecutionValidatorTestAccess;EXPECT_EQ(A::taskLeaseHistorySize(f.ledger),8U);
  scan_planner::BrakingModel model{.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'a')};
  model.command_pipeline_bound=.1;A::braking(f.ledger,model);
  w::MotionDemand d;d.version=f.proposal.version;d.trajectory_id=1;d.sequence=20;
  d.execution_id="exec";d.control_epoch=3;d.sdk_session="sdk";d.sdk_arm_generation=2;
  d.source_stamp=rclcpp::Time(10015000000LL);d.body_source_stamp=rclcpp::Time(10000000000LL);
  d.valid_until=rclcpp::Time(10100000000LL);d.velocity.linear.x=.2;d.transport_mode="isolated_mock";
  ASSERT_TRUE(A::localCommand(f.ledger,d,10100000000LL,10020000000LL));
  lease.sequence=11;lease.allowed=false;lease.phase="holding";
  ASSERT_TRUE(f.ledger.taskLeaseAt(lease,10020000000LL));EXPECT_EQ(A::taskLeaseHistorySize(f.ledger),0U);
  EXPECT_FALSE(A::localCommand(f.ledger,d,10100000000LL,10020000000LL));
}
TEST(LocalFollowerLedger,DirectDemandRelayStoresOnlyLatestWhileExplicitHoldFencesImmediately) {
  LocalCommitFixture f;using A=scan_planner::ExecutionValidatorTestAccess;
  ASSERT_TRUE(f.ledger.localCommitAt(f.fact,10020000000LL));
  w::MotionDemand d;d.version=f.proposal.version;d.transport_mode="isolated_mock";d.execution_id="execution";
  d.trajectory_id=f.fact.trajectory_id;d.source_stamp=rclcpp::Time(10020000000LL);
  d.body_source_stamp=rclcpp::Time(10000000000LL);d.valid_until=rclcpp::Time(10100000000LL);d.velocity.linear.x=.2;
  for(unsigned i=1;i<=100;++i){d.sequence=i;f.ledger.demand(d);}
  ASSERT_TRUE(A::latestDemand(f.ledger));EXPECT_EQ(A::latestDemand(f.ledger)->sequence,100U);
  const auto barrier=A::barrier(f.ledger);
  d.sequence=101;d.hold=true;d.velocity={};f.ledger.demand(d);
  EXPECT_GT(A::barrier(f.ledger),barrier);EXPECT_TRUE(A::latestDemand(f.ledger)->hold);
  d.sequence=100;d.hold=false;f.ledger.demand(d);EXPECT_TRUE(A::latestDemand(f.ledger)->hold);
}
TEST(LocalFollowerTiming,ShortBudgetBelongsOnlyToCurrentCurveRenewal) {
  using C=scan_planner::ValidationCycle;
  EXPECT_DOUBLE_EQ(C::local_curve_budget_s,.050);
  EXPECT_EQ(C::local_curve_budget,C::period);
}
TEST(LocalFollowerTiming,CurrentNativeCheckUsesUnusedRoundInsteadOfTwelveMillisecondSubLane) {
  using C=scan_planner::ValidationCycle;const auto begin=C::Clock::time_point{};
  auto now=begin+std::chrono::milliseconds(4); // snapshot + acquired-body setup
  unsigned checks=0;
  EXPECT_TRUE(C::currentCurvePass(begin,[&](double budget,C::Clock::time_point deadline){
    ++checks;EXPECT_NEAR(budget,.046,1e-9);EXPECT_EQ(deadline,begin+C::period);
    // Logged native queries take 11.66--11.88 ms; setup is part of the round.
    now+=std::chrono::milliseconds(13);return now<deadline;
  },[&]{return now;}));
  EXPECT_EQ(checks,1U);EXPECT_GT(now-begin,std::chrono::milliseconds(12));
  EXPECT_LT(now-begin,C::period);
  now=begin+C::period;
  EXPECT_FALSE(C::currentCurvePass(begin,[&](double,C::Clock::time_point){++checks;return true;},[&]{return now;}));
  EXPECT_EQ(checks,1U); // No query or fresh deadline after this round expired.
}
TEST(LocalFollowerTiming,LongAdmissionIncludesBorrowAndSharesOneDeadlineWithOptionalCandidate) {
  using C=scan_planner::ValidationCycle;const auto begin=C::Clock::time_point{};
  auto now=begin+std::chrono::milliseconds(10);std::vector<double> budgets;
  C::admissionPass(begin,[&](double budget,C::Clock::time_point deadline){
    budgets.push_back(budget);EXPECT_EQ(deadline,begin+C::slow_curve_budget);
    now+=std::chrono::milliseconds(85);
  },[&](double budget,C::Clock::time_point deadline){
    budgets.push_back(budget);EXPECT_EQ(deadline,begin+C::slow_curve_budget);
    now+=std::chrono::milliseconds(25);
  },[&]{return now;});
  ASSERT_EQ(budgets.size(),2U);EXPECT_NEAR(budgets[0],.110,1e-9);EXPECT_NEAR(budgets[1],.025,1e-9);
  EXPECT_EQ(now,begin+C::slow_curve_budget);
  EXPECT_DOUBLE_EQ(C::admissionBudget(begin,now),0.);
}
TEST(LocalFollowerTiming,ExhaustedAdmissionCannotRenewIncumbentOrStartPendingWork) {
  using C=scan_planner::ValidationCycle;const auto begin=C::Clock::time_point{};
  auto now=begin+std::chrono::milliseconds(119);unsigned active=0,pending=0;
  C::admissionPass(begin,[&](double budget,C::Clock::time_point deadline){
    ++active;EXPECT_NEAR(budget,.001,1e-9);now=deadline;
  },[&](double,C::Clock::time_point){++pending;},[&]{return now;});
  EXPECT_EQ(active,1U);EXPECT_EQ(pending,0U);
  C::admissionPass(begin,[&](double,C::Clock::time_point){++active;},
    [&](double,C::Clock::time_point){++pending;},[&]{return now;});
  EXPECT_EQ(active,1U);EXPECT_EQ(pending,0U);
  const std::int64_t original_deadline=1000000000LL;
  EXPECT_FALSE(C::originalLeaseUsable(true,original_deadline,original_deadline));
}
TEST(LocalFollowerTiming,FormalCurrentAndCandidateHaveIndependentOriginalCompleteBudgets) {
  using C=scan_planner::ValidationCycle;const auto begin=C::Clock::time_point{};
  auto now=begin+std::chrono::milliseconds(10);unsigned active=0,pending=0;
  EXPECT_TRUE(C::formalCurrentCurvePass(begin,[&](double budget,auto deadline) {
    ++active;EXPECT_NEAR(budget,.110,1e-9);EXPECT_EQ(deadline,begin+C::slow_curve_budget);
    now+=std::chrono::milliseconds(100);return now<deadline;
  },[&]{return now;}));
  // Separate existing reader, separate actual snapshot/cache. The candidate
  // does not steal the incumbent's budget or its next refresh opportunity.
  auto candidate_now=begin+std::chrono::milliseconds(10);
  C::formalAdmissionPass(begin,[&](double budget,auto deadline) {
    ++pending;EXPECT_NEAR(budget,.110,1e-9);EXPECT_EQ(deadline,begin+C::slow_curve_budget);
    candidate_now+=std::chrono::milliseconds(100);
  },[&]{return candidate_now;});
  EXPECT_EQ(active,1u);EXPECT_EQ(pending,1u);EXPECT_EQ(C::period,std::chrono::milliseconds(50));
  now=begin+C::slow_curve_budget;
  EXPECT_FALSE(C::formalCurrentCurvePass(begin,[&](double,auto){++active;return true;},[&]{return now;}));
  C::formalAdmissionPass(begin,[&](double,auto){++pending;},[&]{return now;});
  EXPECT_EQ(active,1u);EXPECT_EQ(pending,1u);
}
TEST(LocalFollowerTiming,FormalActiveOnlyNeverClaimsSolverAdmissionSlot) {
  using C=scan_planner::ValidationCycle;
  EXPECT_FALSE(C::admissionWorkWanted(true,true,false));
  EXPECT_FALSE(C::admissionWorkWanted(true,false,false));
  EXPECT_TRUE(C::admissionWorkWanted(true,true,true));
  EXPECT_TRUE(C::admissionWorkWanted(true,false,true));
  EXPECT_TRUE(C::admissionWorkWanted(false,true,false)); // legacy unchanged
}
TEST(LocalFollowerTiming,SeparateCurrentReaderAvoidsAlternatingSlowLaneProofLeaseGap) {
  using C=scan_planner::ValidationCycle;using Ms=std::chrono::milliseconds;
  const auto origin=C::Clock::time_point{};
  // Before the first E14 review, fair A/P ordering fixed starvation but a
  // 200 ms slow cadence doubled current renewal to 400 ms. A 250 ms proof
  // cannot cover that gap. This deterministic time-line preserves that failed
  // design as a regression, not as an OS hard-real-time performance claim.
  auto prior_current_end=origin;Ms rejected_design_gap{0};unsigned pending=0;
  for(unsigned round=0;round<3;++round) {
    const auto begin=origin+Ms(200*round);auto now=begin;
    const auto work=[&](bool active,double budget,auto deadline) {
      if(budget<.100-1e-9){now=deadline;return;}
      now+=Ms(100);
      if(active) {
        rejected_design_gap=std::max(rejected_design_gap,
          std::chrono::duration_cast<Ms>(now-prior_current_end-Ms(250)));
        prior_current_end=now;
      } else ++pending;
    };
    C::admissionPass(begin,[&](double b,auto d){work(round%2==0,b,d);},
      [&](double b,auto d){work(round%2!=0,b,d);},[&]{return now;});
  }
  EXPECT_EQ(rejected_design_gap,Ms(150));EXPECT_EQ(pending,1u);
  prior_current_end=origin;Ms independent_gap{0};pending=0;
  for(unsigned round=0;round<5;++round) {
    const auto begin=origin+Ms(100*round);auto now=begin;
    EXPECT_TRUE(C::formalCurrentCurvePass(begin,[&](double budget,auto deadline) {
      EXPECT_GE(budget,.1);now+=Ms(100);EXPECT_LE(now,deadline);
      independent_gap=std::max(independent_gap,
        std::chrono::duration_cast<Ms>(now-prior_current_end-Ms(250)));
      prior_current_end=now;return true;
    },[&]{return now;}));
    if(round%2==0) {
      auto candidate_now=begin;
      C::formalAdmissionPass(begin,[&](double b,auto deadline) {
        EXPECT_GE(b,.1);candidate_now+=Ms(100);EXPECT_LE(candidate_now,deadline);++pending;
      },[&]{return candidate_now;});
    }
  }
  EXPECT_EQ(independent_gap,Ms(0));EXPECT_EQ(pending,3u);
}
TEST(LocalFollowerLedger,TaskLeaseBindsSdkOnlyOnceAfterUnboundArmingAndRejectsLatePreview) {
  for(bool expected_sdk:{false,true}) {
  LocalCommitFixture f;
  w::TaskExecutionLease p;p.schema_version=1;p.version=f.proposal.version;p.sequence=1;
  p.source_stamp=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10500000000LL);
  p.execution_id="exec";p.control_epoch=3;p.confirmation_id="click";p.phase="arming";p.transport_mode="isolated_mock";
  if(expected_sdk)p.sdk_session="sdk";
  ASSERT_TRUE(f.ledger.taskLeaseAt(p,10020000000LL));
  p.allowed=true;p.sequence=2;p.sdk_session="sdk";p.sdk_arm_generation=1;
  ASSERT_TRUE(f.ledger.taskLeaseAt(p,10020000000LL));
  auto old=p;old.sequence=3;old.allowed=false;old.sdk_session.clear();old.sdk_arm_generation=0;
  EXPECT_FALSE(f.ledger.taskLeaseAt(old,10020000000LL));
  ++p.sequence;p.sdk_session="other";EXPECT_FALSE(f.ledger.taskLeaseAt(p,10020000000LL));
  }
}
TEST(LocalFollowerLedger,NewTaskStartsOwnLeaseSequenceWithoutAcceptingRetiredTaskLease) {
  for(bool owner_first:{false,true}) {
  SCOPED_TRACE(owner_first?"owner_permit_first":"reference_proposal_first");
  LocalCommitFixture f;
  w::TaskExecutionLease old;old.schema_version=1;old.version=f.proposal.version;old.sequence=1000;
  old.source_stamp=rclcpp::Time(10000000000LL);old.valid_until=rclcpp::Time(10500000000LL);
  old.execution_id="old-exec";old.control_epoch=1;old.confirmation_id="old-click";
  old.sdk_session="sdk";old.sdk_arm_generation=1;old.allowed=true;old.transport_mode="isolated_mock";
  ASSERT_TRUE(f.ledger.taskLeaseAt(old,10020000000LL));
  auto barrier=f.proposal.version;++barrier.reference_generation;
  ASSERT_TRUE(f.ledger.cancelReference(barrier));
  auto next=f.proposal;next.version.task_id="next-task";next.version.reference_generation=3;
  next.proposal_id="next-proposal";next.expected_version={};next.expected_trajectory_id=-1;
  auto permit=f.permit(1);permit.version=next.version;
  if(owner_first)ASSERT_TRUE(f.ledger.commit(permit));
  else ASSERT_TRUE(f.ledger.replacePendingReference(next));
  auto fresh=old;fresh.version=next.version;fresh.sequence=1;fresh.execution_id="next-exec";
  fresh.control_epoch=2;fresh.confirmation_id="next-click";fresh.sdk_arm_generation=2;
  // BT constructs a new TaskExecutionAuthority for each task, whose first
  // lease is 1. Native's per-task high-watermark must share that ownership.
  EXPECT_TRUE(f.ledger.taskLeaseAt(fresh,10020000000LL));
  // The other legitimate ingress is not another new-task event and must
  // not reset the just acquired sequence barrier.
  if(owner_first)ASSERT_TRUE(f.ledger.replacePendingReference(next));
  else ASSERT_TRUE(f.ledger.commit(permit));
  ++old.sequence;EXPECT_FALSE(f.ledger.taskLeaseAt(old,10020000000LL));
  EXPECT_FALSE(f.ledger.taskLeaseAt(fresh,10020000000LL)); // same-task duplicate
  ++fresh.sequence;EXPECT_TRUE(f.ledger.taskLeaseAt(fresh,10020000000LL));
  }
}
TEST(LocalFollowerLedger,SameTaskHoldAndRollingReferenceKeepLeaseHighWatermark) {
  LocalCommitFixture f;
  w::TaskExecutionLease p;p.schema_version=1;p.version=f.proposal.version;p.sequence=1000;
  p.source_stamp=rclcpp::Time(10000000000LL);p.valid_until=rclcpp::Time(10500000000LL);
  p.execution_id="exec";p.control_epoch=1;p.confirmation_id="click";
  p.sdk_session="sdk";p.sdk_arm_generation=1;p.allowed=true;p.transport_mode="isolated_mock";
  ASSERT_TRUE(f.ledger.taskLeaseAt(p,10020000000LL));
  auto hold=p;hold.sequence=1001;hold.allowed=false;hold.reason="waiting_sdk_state";
  ASSERT_TRUE(f.ledger.taskLeaseAt(hold,10020000000LL));
  auto rolling=f.proposal;++rolling.version.reference_generation;rolling.proposal_id="rolling";
  ASSERT_TRUE(f.ledger.replacePendingReference(rolling));
  auto permit=f.permit();permit.version=rolling.version;ASSERT_TRUE(f.ledger.commit(permit));
  EXPECT_FALSE(f.ledger.taskLeaseAt(p,10020000000LL));
  EXPECT_FALSE(f.ledger.taskLeaseAt(hold,10020000000LL));
  p.sequence=1002;EXPECT_TRUE(f.ledger.taskLeaseAt(p,10020000000LL));
}
