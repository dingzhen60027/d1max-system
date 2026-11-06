#include "native_local_follower_test_fixture.hpp"
namespace {
// Refresh only the independently acquired body, command and authority. This
// deliberately does not run another geometry check or manufacture map input.
std::optional<w::LocalMotionCommand> directStateDemand(Integrated& f,
    w::MotionDemand demand,double now,unsigned sequence) {
  const auto ns=t::timeNs(t::stamp(now));
  if(Access::bodySource(f.native)<now)f.acquireBody(now,0.,0.);
  f.native.progressAt(f.progress(now),now);
  const auto authority=lease(now,sequence);
  EXPECT_TRUE(f.follower.lease(authority,now));EXPECT_TRUE(f.native.taskLeaseAt(authority,ns));
  EXPECT_TRUE(f.native.commit(f.follower.geometryPermit(now)));
  demand.sequence=sequence;demand.source_stamp=demand.body_source_stamp=t::stamp(now);
  demand.valid_until=t::stamp(now+.1);
  std::optional<w::LocalMotionCommand> out;
  f.native.demandAt(demand,ns,[&](const auto& command){out=command;});return out;
}
}
TEST(LocalFollowerNative,CompletedStateOutlivesOldMapAndCheckLeasesDuringHealthyUpdates) {
  Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
  const auto demand=Access::latestDemand(f.native);ASSERT_TRUE(demand&&!demand->hold);
  const auto original=f.native.committedView();ASSERT_TRUE(original&&original->proof);
  for(unsigned i=2;i<=28;++i) {
    const double now=100.+.05*i;const auto ns=t::timeNs(t::stamp(now));
    // Real accepted input/writer progress. No current check finishes here:
    // 1.35 s remains within the existing 1.5 s component hang timeout.
    f.pipeline_health.source_ns={ns,ns};++f.pipeline_health.fusion_sequence;
    f.pipeline_health.sampled_steady_ns=f.pipeline_health.fusion_completed_steady_ns=ns;
    const auto command=directStateDemand(f,*demand,now,i);ASSERT_TRUE(command)<<i;
    EXPECT_FALSE(command->hold);EXPECT_EQ(t::timeNs(command->valid_until),ns+100000000LL);
    EXPECT_FALSE(Access::pipelineFault(f.native));
  }
  const auto retained=f.native.committedView();ASSERT_TRUE(retained&&retained->proof);
  EXPECT_EQ(*retained->proof,*original->proof);EXPECT_EQ(t::timeNs(retained->proof->valid_until),0);
}
TEST(LocalFollowerNative,OldIntegratedSnapshotDoesNotImposeAnotherSourceLease) {
  Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
  const auto demand=Access::latestDemand(f.native);ASSERT_TRUE(demand&&!demand->hold);
  f.acquire(100.9,0.,0.);f.native.progressAt(f.progress(100.9),100.9);
  GridMapTestAccess::acquire(f.map,100050000000LL);
  const auto complete=Access::checkCurve(f.native,f.map,100900000000LL,true);
  ASSERT_TRUE(complete&&complete->valid)<< (complete?complete->reason:"no verdict");
  EXPECT_EQ(t::timeNs(complete->front_ray_source_stamp),100050000000LL);
  EXPECT_EQ(t::timeNs(complete->valid_until),0);
  const auto command=directStateDemand(f,*demand,100.9,2);ASSERT_TRUE(command);
  EXPECT_FALSE(command->hold);EXPECT_EQ(t::timeNs(command->valid_until),101000000000LL);
}
TEST(LocalFollowerNative,TrueInputLossRequiresNewFusedPairAndCurrentPositiveNotCandidate) {
  Integrated f;f.braking.sensor_source_age=.6;Access::braking(f.native,f.braking);
  ASSERT_TRUE(f.command(100.05,0.,0.,1));const auto demand=Access::latestDemand(f.native);
  ASSERT_TRUE(demand&&!demand->hold);
  f.pipeline_health.sampled_steady_ns=100651000000LL;
  EXPECT_FALSE(directStateDemand(f,*demand,100.651,2));
  ASSERT_TRUE(Access::pipelineFault(f.native));
  EXPECT_EQ(Access::diagnostics(f.native).stage,scan_planner::MotionCheckStage::NativePipelineFault);
  // Receipt alone is not recovery, nor does repeated sampling move the floor.
  f.pipeline_health.source_ns={100660000000LL,100660000000LL};
  f.pipeline_health.sampled_steady_ns=100660000000LL;
  EXPECT_FALSE(directStateDemand(f,*demand,100.66,3));
  f.acquire(100.67,0.,0.);
  f.native.progressAt(f.progress(100.67),100.67);
  auto candidate=tag(2);candidate.join_source_stamp=candidate.trajectory.start_time=t::stamp(100.67);
  f.native.candidate(candidate,f.reference,f.heading);
  const auto admitted=Access::checkCurve(f.native,f.map,100670000000LL);
  ASSERT_TRUE(admitted&&admitted->valid);EXPECT_TRUE(Access::pipelineFault(f.native));
  EXPECT_FALSE(directStateDemand(f,*demand,100.67,4));
  const auto current=Access::checkCurve(f.native,f.map,100680000000LL,true);
  ASSERT_TRUE(current&&current->valid);EXPECT_FALSE(Access::pipelineFault(f.native));
  const auto command=directStateDemand(f,*demand,100.68,5);ASSERT_TRUE(command);EXPECT_FALSE(command->hold);
}
TEST(LocalFollowerNative,FreshInputDoesNotHideHungFusionOrHungCurrentChecker) {
  for(bool writer_hung:{false,true}) {
    SCOPED_TRACE(writer_hung);Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
    const auto demand=Access::latestDemand(f.native);ASSERT_TRUE(demand&&!demand->hold);
    constexpr auto now=101551000000LL;
    f.pipeline_health.source_ns={now,now};f.pipeline_health.sampled_steady_ns=now;
    if(writer_hung) {
      f.acquireBody(101.551,0.,0.);
      f.native.progressAt(f.progress(101.551),101.551);
      // Even real repeated checks of an old snapshot cannot hide a dead writer.
      const auto checked=Access::checkCurve(f.native,f.map,now,true);ASSERT_TRUE(checked&&checked->valid);
    } else {GridMapTestAccess::acquire(f.map,now);
      f.pipeline_health.fusion_sequence=f.map.fusionTiming().sequence;
      f.pipeline_health.fusion_completed_steady_ns=now;}
    EXPECT_FALSE(directStateDemand(f,*demand,101.551,2));EXPECT_TRUE(Access::pipelineFault(f.native));
    f.acquire(101.60,0.,0.);
    f.native.progressAt(f.progress(101.60),101.60);
    EXPECT_FALSE(directStateDemand(f,*demand,101.60,3));
    const auto checked=Access::checkCurve(f.native,f.map,101600000000LL,true);ASSERT_TRUE(checked&&checked->valid);
    EXPECT_FALSE(Access::pipelineFault(f.native));
    const auto resumed=directStateDemand(f,*demand,101.61,4);ASSERT_TRUE(resumed);EXPECT_FALSE(resumed->hold);
  }
}
TEST(LocalFollowerNative,IncompleteCurrentAttemptRetainsCompletedNegativeWithoutRefreshingWatchdog) {
  Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
  const auto demand=Access::latestDemand(f.native);ASSERT_TRUE(demand&&!demand->hold);
  GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.);
  const auto negative=Access::checkCurve(f.native,f.map,100050000000LL,true);
  ASSERT_TRUE(negative&&!negative->valid);EXPECT_EQ(negative->reason,"occupied");
  GridMapTestAccess::voxel(f.map,{.3,0.,.55},-1.);
  for(unsigned i=2;i<=20;++i) {
    const double now=100.+i*.05;f.acquire(now,0.,0.);
    const auto incomplete=Access::checkCurveRound(f.native,f.map,[=]{return t::timeNs(t::stamp(now));},
      .12,scan_planner::ValidationCycle::Clock::now()-std::chrono::milliseconds(1),true);
    EXPECT_FALSE(incomplete);EXPECT_FALSE(directStateDemand(f,*demand,now,i));
    const auto view=f.native.committedView();ASSERT_TRUE(view&&view->proof);EXPECT_EQ(*view->proof,*negative);
  }
}
TEST(LocalFollowerNative,RepeatedOldSnapshotChecksDoNotHidePermanentlyStalledSnapshotDelivery) {
  Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
  const auto demand=Access::latestDemand(f.native);ASSERT_TRUE(demand&&!demand->hold);
  for(unsigned i=2;i<=32;++i) {
    const double now=100.+.05*i;const auto ns=t::timeNs(t::stamp(now));
    f.acquireBody(now,0.,0.);f.native.progressAt(f.progress(now),now);
    f.pipeline_health.source_ns={ns,ns};++f.pipeline_health.fusion_sequence;
    f.pipeline_health.sampled_steady_ns=f.pipeline_health.fusion_completed_steady_ns=ns;
    const auto proof=Access::checkCurve(f.native,f.map,ns,true);ASSERT_TRUE(proof&&proof->valid);
    const auto command=directStateDemand(f,*demand,now,i);
    if(i<=31){ASSERT_TRUE(command)<<i;EXPECT_FALSE(command->hold);}
    else {EXPECT_FALSE(command);EXPECT_TRUE(Access::pipelineFault(f.native));}
  }
}
TEST(LocalFollowerNative,OldSnapshotAndProjectionBudgetExhaustionNeverPublishNegativeState) {
  Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
  f.acquire(100.9,0.,0.);GridMapTestAccess::acquire(f.map,100050000000LL);
  auto candidate=tag(2);candidate.join_source_stamp=candidate.trajectory.start_time=t::stamp(100.9);
  f.native.candidate(candidate,f.reference,f.heading);
  const auto before=f.native.committedView();ASSERT_TRUE(before&&before->proof);
  const auto incomplete=Access::checkCurveRound(f.native,f.map,[]{return 100900000000LL;},.12,
    scan_planner::ValidationCycle::Clock::now()-std::chrono::milliseconds(1),false);
  EXPECT_FALSE(incomplete);
  const auto after=f.native.committedView();ASSERT_TRUE(after&&after->proof);EXPECT_EQ(*before->proof,*after->proof);
  const auto completed=Access::checkCurve(f.native,f.map,100900000000LL);
  ASSERT_TRUE(completed&&completed->valid);EXPECT_EQ(t::timeNs(completed->valid_until),0);
}
TEST(LocalFollowerNative,ExplicitModelOnlyReachesNativeCommitWithoutClaimingLiteralVerification) {
  auto s=support();s.verified=false;s.support_ground_xyz.clear();
  s.floor_id="stairs";s.segment_kind="stair_up";s.required_mode="stair";
  s.max_support_slope_rad=s.max_support_step_m=0.;
  s.stair_model_hash=std::string(64,'d');
  w::StairSurfaceModel surface;surface.surface_id="independent-static-envelope";
  surface.segment_id=s.version.segment_id;surface.kind="continuous_support_envelope";
  surface.pose.orientation.w=1.;surface.support_grade=surface.support_intercept_m=0.;
  surface.height_association_m=.25;
  const Eigen::Quaterniond anchor(Eigen::AngleAxisd(.12,Eigen::Vector3d::UnitY())*
    Eigen::AngleAxisd(-.08,Eigen::Vector3d::UnitX()));
  const Eigen::Vector3d origin=Eigen::Vector3d(0.,0.,s.body_reference_height_m)-
    anchor*Eigen::Vector3d(0.,0.,s.body_reference_height_m);
  surface.pose.position.x=origin.x();surface.pose.position.y=origin.y();surface.pose.position.z=origin.z();
  surface.pose.orientation.x=anchor.x();surface.pose.orientation.y=anchor.y();
  surface.pose.orientation.z=anchor.z();surface.pose.orientation.w=anchor.w();
  for(const auto& xy:{Eigen::Vector2d(-1.,-.8),Eigen::Vector2d(2.,-.8),
                     Eigen::Vector2d(2.,.8),Eigen::Vector2d(-1.,.8)}) {
    geometry_msgs::msg::Point p;p.x=xy.x();p.y=xy.y();surface.polygon.push_back(p);
  }
  s.stair_model_surfaces.push_back(surface);
  Integrated f(true,false,{},{},false,s);
  const auto pinned=Access::pinnedSupport(f.native,true);ASSERT_TRUE(pinned);
  ASSERT_TRUE(pinned->valid);ASSERT_TRUE(pinned->model());EXPECT_FALSE(pinned->source.verified);
  EXPECT_FALSE(pinned->literalStairValid());EXPECT_TRUE(pinned->source.support_ground_xyz.empty());
  const auto at_body=pinned->modeledBodyPosition({0.,0.,s.body_reference_height_m});ASSERT_TRUE(at_body);
  EXPECT_TRUE(at_body->isApprox(Eigen::Vector3d(0.,0.,s.body_reference_height_m),1e-12));
  auto proof=Access::checkCurve(f.native,f.map,100010000000LL,true);
  ASSERT_TRUE(proof);EXPECT_TRUE(proof->valid)<<proof->reason;
  const auto command=f.command(100.05,0.,0.,1);ASSERT_TRUE(command);
  EXPECT_FALSE(command->hold)<<command->reason;
}
TEST(LocalFollowerNative,PlanningSupportSeparatesStairGeometryFromFlatMotionAndRetiresByTask) {
  Integrated f;
  auto s=support();s.version.segment_id="stairs";++s.version.reference_generation;
  s.support_reference_id="literal-stair-source";s.floor_id="stair_lower";
  s.segment_kind="stair_up";s.required_mode="stair";
  s.reason="original_stair_surface_returns_associated";
  s.max_support_slope_rad=s.max_support_step_m=0.;
  EXPECT_FALSE(scan_planner::MotionSupport(s).valid);
  f.native.support(s);const auto first=f.native.planningSupport(s.version);
  ASSERT_TRUE(first);ASSERT_TRUE(first->isStair());ASSERT_TRUE(first->nearest(0.,0.,0.));
  auto wrong=s.version;++wrong.anchor_revision;EXPECT_FALSE(f.native.planningSupport(wrong));
  // Duplicate delivery is immutable and does not re-index/replace geometry.
  auto corrupt=s;corrupt.support_ground_xyz.front().z=5.;f.native.support(corrupt);
  EXPECT_EQ(f.native.planningSupport(s.version),first);
  // The independent legacy flat entry is still available in the same task.
  const auto flat=f.native.planningSupport(version());ASSERT_TRUE(flat);EXPECT_FALSE(flat->isStair());
  ASSERT_TRUE(f.native.cancelReference(s.version));
  EXPECT_FALSE(f.native.planningSupport(s.version));EXPECT_FALSE(f.native.planningSupport(version()));
  ++s.version.reference_generation;f.native.support(s);
  EXPECT_FALSE(f.native.planningSupport(s.version)); // A newer window cannot resurrect a canceled task.
}
TEST(LocalFollowerNative,PlanningSupportHistoryIsBoundedAndCannotBorrowAnotherAnchor) {
  Integrated f;auto first=support();first.version.reference_generation=2;
  first.version.segment_id="stairs";first.support_reference_id="literal-stair-source";
  first.floor_id="stair_upper";first.segment_kind="stair_down";first.required_mode="stair";
  first.reason="original_stair_surface_returns_associated";
  first.max_support_slope_rad=first.max_support_step_m=0.;
  f.native.support(first);ASSERT_TRUE(f.native.planningSupport(first.version));
  for(unsigned i=0;i<8;++i) {auto newer=first;newer.version.reference_generation+=i+1;f.native.support(newer);}
  EXPECT_FALSE(f.native.planningSupport(first.version));
  auto newest=first.version;newest.reference_generation+=8;
  ASSERT_TRUE(f.native.planningSupport(newest));++newest.anchor_revision;
  EXPECT_FALSE(f.native.planningSupport(newest));
}
TEST(LocalFollowerNative,StairSearchWaitsForItsExactSupportWithoutRetiringTheIncumbent) {
  for(const std::string kind:{"stair_up","stair_down","landing"}) {
    SCOPED_TRACE(kind);
    Integrated f;
    auto next=f.reference;next.expected_version=next.version;next.expected_trajectory_id=1;
    ++next.version.reference_generation;next.proposal_id="stair-window";
    ASSERT_TRUE(f.native.replacePendingReference(next,true));
    w::ReferencePath r;r.schema_version=2;r.required_mode="stair";r.segment_kind=kind;
    const auto ready=[&]{return scan_planner::stairReferenceGeometryReady(
      r,next.version,f.native.planningSupport(next.version),"odom");};
    EXPECT_FALSE(ready());
    EXPECT_EQ(f.native.committedTrajectory(),1);
    // An older valid floor support and another anchor cannot fill this gap.
    const auto old=f.native.planningSupport(version());ASSERT_TRUE(old);
    EXPECT_FALSE(scan_planner::stairReferenceGeometryReady(r,next.version,old,"odom"));
    auto exact=support();exact.version=next.version;exact.floor_id="stair_lower";
    exact.segment_kind=kind;exact.required_mode="stair";
    exact.reason="original_stair_surface_returns_associated";
    exact.max_support_slope_rad=exact.max_support_step_m=0.;
    auto other=exact;++other.version.anchor_revision;f.native.support(other);
    EXPECT_FALSE(ready());EXPECT_EQ(f.native.committedTrajectory(),1);
    f.native.support(exact);ASSERT_TRUE(ready());
    const auto pinned=f.native.planningSupport(next.version);ASSERT_TRUE(pinned);
    EXPECT_FALSE(scan_planner::stairReferenceGeometryReady(r,next.version,pinned,"map"));
    // Arrival enables computation only: no curve/command/permit is committed.
    EXPECT_EQ(f.native.committedTrajectory(),1);
    ASSERT_TRUE(f.native.cancelReference(next.version));
    f.native.support(exact);EXPECT_FALSE(ready());
    EXPECT_EQ(f.native.committedTrajectory(),-1);
  }
}
TEST(LocalFollowerNative,MixedStairWindowNeedsGeometryButFlatReferenceRemainsUnchanged) {
  w::ReferencePath r;r.schema_version=2;r.required_mode="general";r.segment_kind="floor";
  const std::shared_ptr<const scan_planner::PlanningSupport> missing;
  EXPECT_TRUE(scan_planner::stairReferenceGeometryReady(r,version(),missing,"odom"));
  r.point_required_modes={"general","stair","general"};
  EXPECT_FALSE(scan_planner::stairReferenceGeometryReady(r,version(),missing,"odom"));
  r.point_required_modes={"general","general"};
  EXPECT_TRUE(scan_planner::stairReferenceGeometryReady(r,version(),missing,"odom"));
  r.schema_version=1;r.required_mode="stair";
  EXPECT_TRUE(scan_planner::stairReferenceGeometryReady(r,version(),missing,"odom"));
}
TEST(LocalFollowerNative,CommittedStairSupportRemainsPinnedAcrossPendingHistoryRolloverAndReplacement) {
  auto s=support();s.floor_id="stair_lower";s.segment_kind="stair_up";s.required_mode="stair";
  s.reason="original_stair_surface_returns_associated";s.max_support_slope_rad=s.max_support_step_m=0.;
  // A flat landing in a stair-mode segment is still original stair support;
  // terrain attitude is derived from those points, not from the mode name.
  Integrated f(true,false,{},{},false,s);
  const auto original=Access::pinnedSupport(f.native,true);
  ASSERT_TRUE(original);ASSERT_TRUE(original->isStair());
  EXPECT_EQ(original,f.native.planningSupport(s.version));
  for(unsigned i=0;i<10;++i) {
    auto future=s;future.version.reference_generation+=i+1;
    future.support_reference_id="future-stair-"+std::to_string(i);
    for(auto& p:future.support_ground_xyz)p.z+=3.;
    f.native.support(future);
  }
  EXPECT_EQ(Access::pendingSupportCount(f.native),8U);
  EXPECT_EQ(Access::pinnedSupport(f.native,true),original);
  EXPECT_EQ(f.native.planningSupport(s.version),original);
  auto next=tag(2);f.native.candidate(next,f.reference,f.heading);
  ASSERT_EQ(Access::pinnedSupport(f.native),original);
  auto proof=Access::checkCurve(f.native,f.map,100010000000LL);
  ASSERT_TRUE(proof);ASSERT_TRUE(proof->valid)<<proof->reason;
  EXPECT_EQ(proof->support_reference_id,s.support_reference_id);
  EXPECT_EQ(Access::pinnedSupport(f.native,true)->source.support_hash,s.support_hash);
  // Retirement prevents the still-owned shared pointer from becoming current
  // geometry again through a late support message or an old candidate.
  auto barrier=s.version;barrier.reference_generation+=20;
  ASSERT_TRUE(f.native.cancelReference(barrier));
  EXPECT_FALSE(f.native.planningSupport(s.version));
  f.native.support(s);f.native.candidate(tag(3),f.reference,f.heading);
  EXPECT_FALSE(f.native.planningSupport(s.version));
  EXPECT_TRUE(original->valid); // Object lifetime is distinct from task validity.
}
TEST(LocalFollowerNative,LateSupportPinsOnlyItsExactCandidateAndDoesNotReplaceActiveGeometry) {
  Integrated f;const auto original=Access::pinnedSupport(f.native,true);ASSERT_TRUE(original);
  auto reference=f.reference;reference.expected_version=reference.version;
  reference.expected_trajectory_id=1;++reference.version.reference_generation;
  ++reference.version.anchor_revision;reference.proposal_id="new-anchor";
  ASSERT_TRUE(f.native.replacePendingReference(reference));
  auto next=tag(2);next.generation=reference.version.reference_generation;
  next.anchor_revision=reference.version.anchor_revision;
  f.native.candidate(next,reference,f.heading);
  EXPECT_FALSE(Access::pinnedSupport(f.native));
  auto foreign=support();foreign.version=reference.version;++foreign.version.anchor_revision;
  f.native.support(foreign);EXPECT_FALSE(Access::pinnedSupport(f.native));
  auto exact=support();exact.version=reference.version;exact.support_reference_id="new-anchor-support";
  f.native.support(exact);const auto pinned=Access::pinnedSupport(f.native);
  ASSERT_TRUE(pinned);EXPECT_EQ(pinned->source.version,reference.version);
  EXPECT_EQ(Access::pinnedSupport(f.native,true),original);
  auto duplicate=exact;duplicate.support_hash=std::string(64,'f');
  for(auto& p:duplicate.support_ground_xyz)p.z+=5.;
  f.native.support(duplicate);EXPECT_EQ(Access::pinnedSupport(f.native),pinned);
  auto proof=Access::checkCurve(f.native,f.map,100010000000LL);
  ASSERT_TRUE(proof);ASSERT_TRUE(proof->valid)<<proof->reason;
  EXPECT_EQ(proof->support_reference_id,exact.support_reference_id);
  EXPECT_EQ(f.native.committedTrajectory(),1);
}
TEST(LocalFollowerNative,ReferenceAdmissionDoesNotWaitForLargeStaticSupportOrExpireItsGeometry) {
  for(const bool reanchor:{false,true})for(const bool retire:{false,true}) {
    SCOPED_TRACE(reanchor);
    SCOPED_TRACE(retire);
    Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},{0,0,0}));
    const auto original=Access::pinnedSupport(f.native,true);ASSERT_TRUE(original);
    auto next=f.reference;next.expected_version=f.reference.version;
    next.expected_trajectory_id=1;++next.version.reference_generation;
    if(reanchor){++next.version.anchor_revision;next.version.anchor_id="new-anchor";}
    next.proposal_id="fresh-small-window";next.source_stamp=t::stamp(100.01);
    next.valid_until=t::stamp(100.41);
    // The small input is admitted while fresh. Geometry preparation may take
    // longer than the admission TTL; that is not a new motion authorization.
    ASSERT_TRUE(f.native.replacePendingReference(next,!reanchor));
    ASSERT_TRUE(f.follower.proposal(next,100.01));
    for(unsigned i=1;i<=12;++i) {
      const double now=100.+i*.05;
      const auto command=f.command(now,(now-100.)*.2,.2,i);
      ASSERT_TRUE(command);EXPECT_FALSE(command->hold);
      EXPECT_GT(command->velocity.linear.x,0.);
      EXPECT_EQ(command->version,f.reference.version);
      EXPECT_EQ(Access::pinnedSupport(f.native,true),original);
    }
    auto late=support();late.version=next.version;late.source_stamp=next.source_stamp;
    late.support_reference_id="delayed-static-window";
    if(retire) {
      auto barrier=next.version;++barrier.reference_generation;
      ASSERT_TRUE(f.native.cancelReference(barrier));
      f.native.support(late);
      EXPECT_FALSE(f.native.planningSupport(next.version));
      EXPECT_EQ(f.native.committedTrajectory(),-1);
      continue;
    }
    // Exact immutable support arrives at 100.65, after the original proposal
    // deadline. No timestamp is renewed; a genuinely NEW body starts the solve.
    f.acquire(100.65,.13,.2);f.native.support(late);
    ASSERT_TRUE(f.follower.support(late));
    auto candidate=tag(2,{.13,0,.55},{.2,0,0},{0,0,0});
    candidate.generation=next.version.reference_generation;
    candidate.anchor_revision=next.version.anchor_revision;candidate.anchor_id=next.version.anchor_id;
    candidate.join_source_stamp=candidate.trajectory.start_time=t::stamp(100.65);
    f.native.candidate(candidate,next,f.heading);ASSERT_TRUE(f.follower.candidate(candidate));
    const auto proof=Access::checkCurve(f.native,f.map,100650000000LL);
    ASSERT_TRUE(proof);ASSERT_TRUE(proof->valid)<<proof->reason;
    EXPECT_EQ(proof->body_source_stamp,t::stamp(100.65));
    EXPECT_EQ(proof->support_reference_id,late.support_reference_id);
    EXPECT_EQ(f.native.planningSupport(next.version)->source.source_stamp,next.source_stamp);
    ASSERT_TRUE(f.follower.validation(*proof));
    const auto commit=f.follower.tryCommit(100.65,10.65);ASSERT_TRUE(commit)<<f.follower.candidateReason();
    EXPECT_EQ(commit->expected_trajectory_id,1);
    EXPECT_EQ(commit->trajectory_id,2);
    EXPECT_TRUE(f.native.localCommitAt(*commit,100650000000LL));
    EXPECT_EQ(f.native.committedTrajectory(),2);
  }
}
TEST(LocalFollowerNative,WholeCurveUsesPinnedStairAttitudeAndStillRejectsNewBodyObstacle) {
  auto s=support();s.floor_id="stair_lower";s.segment_kind="stair_up";s.required_mode="stair";
  s.reason="original_stair_surface_returns_associated";s.max_support_slope_rad=s.max_support_step_m=0.;
  s.support_xy_radius_m=.12;
  for(auto& p:s.support_ground_xyz)p.z=.5*p.x;
  const auto slope_curve=tag(1,{0.,0.,.55},{0.,0.,0.},{.05,0.,.025});
  Integrated f(true,false,{},slope_curve,false,s);
  const auto pinned=Access::pinnedSupport(f.native,true);ASSERT_TRUE(pinned);
  // Same original voxel: an upright box intersects it, but the full-size
  // acquired uphill attitude clears it. No obstacle buffer is removed.
  const Eigen::Vector3d step{.4,0.,.325};GridMapTestAccess::voxel(f.map,step,2.);
  EXPECT_EQ(f.map.getPlanningInflateOccupancy({0.,0.,.55},0.),1);
  const double pitch=-std::atan(.5);const Eigen::Quaterniond orientation(
      Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()));
  const scan_planner::MeasuredBodyPose acquired{{0.,0.,.55},orientation,100.01,"odom"};
  f.native.body(acquired,geometry_msgs::msg::Twist{},100010000000LL);
  GridMapTestAccess::acquire(f.map,100010000000LL);
  for(unsigned i=0;i<10;++i) {
    auto future=s;future.version.reference_generation+=i+1;
    future.support_reference_id="different-floor-"+std::to_string(i);
    for(auto& p:future.support_ground_xyz)p.z+=3.;
    f.native.support(future);
  }
  auto proof=Access::checkCurve(f.native,f.map,100010000000LL,true);
  ASSERT_TRUE(proof);ASSERT_TRUE(proof->valid)<<proof->reason;
  EXPECT_EQ(Access::pinnedSupport(f.native,true),pinned);
  EXPECT_EQ(GridMapTestAccess::raw(f.map,step),2.);
  // Real new geometry in the torso is not a support sample and must block the
  // same production whole-curve query despite stair mode/terrain prediction.
  const Eigen::Vector3d roof=acquired.position+orientation*Eigen::Vector3d(.2,0.,.2);
  GridMapTestAccess::voxel(f.map,roof,2.);
  proof=Access::checkCurve(f.native,f.map,100010000000LL,true);
  ASSERT_TRUE(proof);EXPECT_FALSE(proof->valid);EXPECT_EQ(proof->reason,"occupied");
  EXPECT_EQ(GridMapTestAccess::raw(f.map,step),2.);EXPECT_EQ(GridMapTestAccess::raw(f.map,roof),2.);
}
TEST(LocalFollowerNative,LegacySpliceOnlySharesSameStairSourceAndNeverRetiresIncumbentOnMismatch) {
  // 0/1: unchanged legacy flat behavior, including different flat source.
  // 2: same immutable stair source, next reference version is permitted.
  // 3..9: changed source, stair<->flat, segment/floor or interpretation change.
  for(int variant=0;variant<=9;++variant) {
    auto stair=support();stair.floor_id="stair_lower";stair.segment_kind="stair_up";
    stair.required_mode="stair";stair.reason="original_stair_surface_returns_associated";
    stair.max_support_slope_rad=stair.max_support_step_m=0.;
    const bool source_stair=variant>=2&&variant!=5;
    auto original=source_stair?stair:support();
    Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()),false,original);
    ASSERT_TRUE(f.command(100.05,.005,.1,1));ASSERT_TRUE(f.prepareTail(100.05));
    auto candidate=Access::pendingCurve(f.native);ASSERT_TRUE(candidate);
    auto reference=f.reference;reference.expected_version=reference.version;
    reference.expected_trajectory_id=1;++reference.version.reference_generation;
    if(variant==6)reference.version.segment_id="stair-next-segment";
    ASSERT_TRUE(f.native.replacePendingReference(reference));
    auto next=variant==4?support():variant==5?stair:original;
    next.version=reference.version;next.support_reference_id="new-window";
    if(variant==1||variant==3)next.support_hash=std::string(64,'f');
    if(variant==7)next.floor_id="stair_upper";
    if(variant==8)next.body_reference_height_m+=.05;
    if(variant==9)next.support_xy_radius_m+=.02;
    f.native.support(next);
    candidate->generation=reference.version.reference_generation;
    candidate->segment_id=reference.version.segment_id;
    f.native.candidate(*candidate,reference,f.heading);
    const auto before=f.native.committedView();ASSERT_TRUE(before&&before->proof);
    const auto proof=Access::checkCurve(f.native,f.map,100050000000LL);
    ASSERT_TRUE(proof)<<variant;
    EXPECT_EQ(proof->valid,variant<=2)<<variant<<": "<<proof->reason;
    if(variant>=3)EXPECT_EQ(proof->reason,"splice_stair_support_changed")<<variant;
    const auto after=f.native.committedView();ASSERT_TRUE(after&&after->proof);
    EXPECT_EQ(f.native.committedTrajectory(),1)<<variant;
    EXPECT_EQ(after->proof->sequence,before->proof->sequence)<<variant;
    EXPECT_EQ(after->proof->valid_until,before->proof->valid_until)<<variant;
    EXPECT_EQ(Access::pinnedSupport(f.native,true)->source.support_reference_id,original.support_reference_id);
    // Candidate failure does not turn the incumbent into a collision, hold,
    // canceled task or a different route. Its ordinary fresh proof still runs.
    const auto incumbent=Access::checkCurve(f.native,f.map,100050000000LL,true);
    ASSERT_TRUE(incumbent);EXPECT_TRUE(incumbent->valid)<<variant<<": "<<incumbent->reason;
  }
}
TEST(LocalFollowerNative,UncommittedFutureSupportCannotStarveCurrentReferenceReplanning) {
  // Future support can arrive before its reference, and that reference can be
  // rejected because its predecessor was superseded. The still-current route
  // must remain able to generate/validate a new local curve.
  for(const unsigned future_count:{1U,10U}) {
    Integrated f;
    for(unsigned i=0;i<future_count;++i) {
      auto future=support();future.version.reference_generation+=i+1;
      future.support_reference_id="future-"+std::to_string(i);
      f.native.support(future);
    }
    auto next=tag(2);f.native.candidate(next,f.reference,f.heading);
    ASSERT_TRUE(f.follower.candidate(next));
    auto proof=Access::checkCurve(f.native,f.map,100010000000LL);
    ASSERT_TRUE(proof);ASSERT_TRUE(proof->valid)<<future_count<<": "<<proof->reason;
    EXPECT_EQ(proof->support_reference_id,"support");
    ASSERT_TRUE(f.follower.validation(*proof));
    const auto prepare=f.follower.exchangeCommit(100.02,10.02);
    ASSERT_TRUE(prepare)<<f.follower.candidateReason();
    auto reserved=f.native.localCommitExchangeAt(*prepare,100020000000LL);
    ASSERT_TRUE(reserved.accepted)<<reserved.reason;
    ASSERT_TRUE(f.follower.commitResult(reserved,100.02,10.02));
    auto commit=f.follower.exchangeCommit(100.02,10.02);ASSERT_TRUE(commit);
    auto accepted=f.native.localCommitExchangeAt(*commit,100020000000LL);
    ASSERT_TRUE(accepted.accepted)<<accepted.reason;
    ASSERT_TRUE(f.follower.commitResult(accepted,100.02,10.02));
    EXPECT_EQ(f.native.committedTrajectory(),2);
    EXPECT_EQ(f.follower.controller().trajectoryId(),2);
    EXPECT_EQ(f.native.committedView()->spline.route_hash,version().route_hash);
  }
}
TEST(LocalFollowerNative,SupportLookupNeverBorrowsGeometryFromAnotherReference) {
  Integrated f;
  auto next_reference=f.reference;
  next_reference.expected_version=f.reference.version;
  next_reference.expected_trajectory_id=1;
  ++next_reference.version.reference_generation;
  ASSERT_TRUE(f.native.replacePendingReference(next_reference));
  // The incumbent has support, and the pending slot has even newer support,
  // but neither belongs to this exact candidate.
  auto future=support();future.version.reference_generation+=2;
  f.native.support(future);
  auto next=tag(2);next.generation=next_reference.version.reference_generation;
  f.native.candidate(next,next_reference,f.heading);
  auto proof=Access::checkCurve(f.native,f.map,100010000000LL);
  ASSERT_TRUE(proof);EXPECT_FALSE(proof->valid);
  EXPECT_EQ(proof->reason,"missing_fresh_raw_ray_support_or_body_evidence");
  EXPECT_EQ(f.native.committedTrajectory(),1);
  auto matching=support();matching.version=next_reference.version;
  matching.support_reference_id="matching-current-reference";
  f.native.support(matching);
  proof=Access::checkCurve(f.native,f.map,100020000000LL);
  ASSERT_TRUE(proof);ASSERT_TRUE(proof->valid)<<proof->reason;
  EXPECT_EQ(proof->support_reference_id,matching.support_reference_id);
  EXPECT_EQ(f.native.committedTrajectory(),1); // Valid candidate is not a commit.
}
TEST(LocalFollowerNative,CompletedStateHasNoLeaseAndPreservesOriginalCheckEndAcrossClockReads) {
  for(const auto increment:{1LL,3000LL,40000LL}) {
    Integrated f;
    auto candidate=tag(2);f.native.candidate(candidate,f.reference,f.heading);
    ASSERT_TRUE(f.follower.candidate(candidate));
    std::int64_t clock_ns=100010000000LL;
    const auto proof=Access::checkCurveClock(f.native,f.map,[&]{
      const auto stamp=clock_ns;clock_ns+=increment;return stamp;
    });
    ASSERT_TRUE(proof&&proof->valid);
    EXPECT_EQ(t::timeNs(proof->valid_until),0)<<increment;
    EXPECT_EQ(proof->source_stamp,proof->check_end);
    ASSERT_TRUE(f.follower.validation(*proof));
    const auto request=f.follower.exchangeCommit(100.02,10.02);
    ASSERT_TRUE(request)<<"clock increment="<<increment<<" "<<f.follower.candidateReason();
    const auto prepared=f.native.localCommitExchangeAt(*request,100020000000LL);
    ASSERT_TRUE(prepared.accepted)<<prepared.reason;
    ASSERT_TRUE(f.follower.commitResult(prepared,100.02,10.02));
    const auto commit=f.follower.exchangeCommit(100.02,10.02);
    ASSERT_TRUE(commit);EXPECT_EQ(commit->disposition,"committed");
    const auto accepted=f.native.localCommitExchangeAt(*commit,100020000000LL);
    ASSERT_TRUE(accepted.accepted)<<accepted.reason;
    ASSERT_TRUE(f.follower.commitResult(accepted,100.02,10.02));
    EXPECT_EQ(f.native.committedTrajectory(),2);
    EXPECT_EQ(f.follower.controller().trajectoryId(),2);
  }
}
TEST(LocalFollowerNative,CompletedStatePreservesAcquiredSourcesAndStillRequiresActualCommitBody) {
  for(unsigned source=0;source<2;++source) {
    Integrated f;auto candidate=tag(2);f.native.candidate(candidate,f.reference,f.heading);
    ASSERT_TRUE(f.follower.candidate(candidate));
    if(source==0)GridMapTestAccess::acquire(f.map,99800000000LL);
    std::int64_t clock_ns=100200000000LL;
    const auto proof=Access::checkCurveClock(f.native,f.map,[&]{
      const auto stamp=clock_ns;clock_ns+=40000LL;return stamp;
    });
    ASSERT_TRUE(proof&&proof->valid);
    const auto deadline=source==0?100300000000LL:100400000000LL;
    EXPECT_EQ(t::timeNs(proof->valid_until),0);
    EXPECT_EQ(t::timeNs(proof->front_ray_source_stamp),source==0?99800000000LL:100000000000LL);
    EXPECT_EQ(t::timeNs(proof->body_source_stamp),100000000000LL);
    ASSERT_TRUE(f.follower.validation(*proof));
    EXPECT_TRUE(f.follower.exchangeCommit(deadline*1e-9,deadline*1e-9-90.));
    EXPECT_EQ(f.native.committedTrajectory(),1);
    EXPECT_EQ(f.follower.controller().trajectoryId(),1);
  }
}
TEST(LocalFollowerNative,FrozenHeadingSeedAndSamplingAreMandatoryAndCannotAlterActualCurve) {
  Integrated f;ASSERT_EQ(f.native.committedTrajectory(),1);
  for(unsigned variant=0;variant<4;++variant) {
    auto bad=tag(2);
    if(variant==0)bad.body_heading_seed_valid=false;
    if(variant==1)bad.body_heading_seed_yaw=.1;
    if(variant==2)bad.body_heading_sampling_intervals=0;
    if(variant==3)bad.body_heading_seed_yaw=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(f.native.candidate(bad,f.reference,f.heading),std::invalid_argument)<<variant;
    EXPECT_EQ(f.native.committedTrajectory(),1);
  }
  auto bad=tag(2);++bad.body_heading_sampling_intervals;
  f.native.candidate(bad,f.reference,f.heading);
  auto proof=Access::checkCurve(f.native,f.map,100000000000LL);
  ASSERT_TRUE(proof);EXPECT_FALSE(proof->valid);
  EXPECT_EQ(proof->reason,"invalid_frozen_body_heading_sampling_contract");
  EXPECT_EQ(f.native.committedTrajectory(),1);
  const auto current=f.native.committedView();ASSERT_TRUE(current&&current->proof);
  EXPECT_TRUE(current->proof->valid);
  auto good=tag(3);f.native.candidate(good,f.reference,f.heading);
  proof=Access::checkCurve(f.native,f.map,100000000000LL);
  ASSERT_TRUE(proof);EXPECT_TRUE(proof->valid)<<proof->reason;
  EXPECT_EQ(f.native.committedTrajectory(),1); // Candidate metadata is not a commit or motion permission.
}
TEST(LocalFollowerNative,ControlDrivenPlantRelaysUnderProductionCurveProofAndOriginalStateLease) {
  Integrated f;double x=0.,vx=0.;unsigned commands=0;
  for(unsigned i=1;i<=40;++i){const double now=100.+i*.05;
    auto out=f.command(now,x,vx,i);ASSERT_TRUE(out)<<scan_planner::MotionCheckDiagnostics::name(Access::diagnostics(f.native).stage);
    EXPECT_EQ(out->source_stamp,t::stamp(now));EXPECT_EQ(out->body_source_stamp,t::stamp(now));
    EXPECT_LE(t::timeNs(out->valid_until),t::timeNs(out->body_source_stamp)+100000000LL);
    EXPECT_LE(std::hypot(out->velocity.linear.x,out->velocity.linear.y),.3+1e-9);
    EXPECT_LE(std::abs(out->velocity.linear.x-vx),.35*.05+1e-8);
    vx=out->velocity.linear.x;x+=vx*.05;++commands;
  }
  EXPECT_EQ(commands,40U);EXPECT_GT(x,.05);EXPECT_GT(f.follower.controller().committedArc(),.04);
  EXPECT_EQ(f.native.committedTrajectory(),1);EXPECT_EQ(f.follower.activeVersion()->route_hash,version().route_hash);
  EXPECT_EQ(Access::motionProofCount(f.native),0U);
}
namespace {
std::optional<w::MotionDemand> expiringProofDemand(Integrated& f,bool body_late=false) {
  for(unsigned i=1;i<20;++i) {
    const double now=100.+i*.05;f.acquire(now,0.,0.);
    GridMapTestAccess::acquire(f.map,t::timeNs(t::stamp(std::max(100.,now-.49))));
    EXPECT_TRUE(f.command(now,0.,0.,i,true,false));
  }
  if(!body_late){f.acquire(101.,0.,0.);f.native.progressAt(f.progress(101.),101.);}
  if(body_late)EXPECT_TRUE(f.follower.body(body(101.,0.,0.),101.,11.));
  GridMapTestAccess::acquire(f.map,100510000000LL); // Monotonic ray source, expires at 101.010.
  auto proof=Access::checkCurve(f.native,f.map,101000000000LL,true);
  if(!proof||!proof->valid)return {};
  EXPECT_EQ(t::timeNs(proof->valid_until),0);
  EXPECT_TRUE(f.follower.validation(*proof));
  const auto grant=lease(101.,20);EXPECT_TRUE(f.follower.lease(grant,101.));
  EXPECT_TRUE(f.native.taskLeaseAt(grant,101000000000LL));
  EXPECT_TRUE(f.native.commit(f.follower.geometryPermit(101.)));
  return f.follower.step(101.005,11.005);
}
}
TEST(LocalFollowerNative,FormerMapProofDeadlineDoesNotDeferAnOtherwiseFreshCommand) {
  Integrated f;const auto prepared=expiringProofDemand(f);ASSERT_TRUE(prepared);
  const auto demand=*prepared;ASSERT_FALSE(demand.hold)<<demand.reason;
  std::vector<w::LocalMotionCommand> sent;
  const auto emit=[&](const auto& c){sent.push_back(c);};
  f.native.demandAt(demand,101012000000LL,emit);
  ASSERT_EQ(sent.size(),1U);EXPECT_FALSE(Access::pendingDemand(f.native));
  EXPECT_EQ(Access::diagnostics(f.native).stage,scan_planner::MotionCheckStage::LocalCommandRelayed);
  Access::consumeRefreshWake(f.native);
  // Fresh raw acquisition, real production GridMap curve check. No new body,
  // controller tick, fabricated source time or SDK lease extension is used.
  GridMapTestAccess::acquire(f.map,101000000000LL);
  const auto fresh=Access::checkCurve(f.native,f.map,101020000000LL,true,emit);
  ASSERT_TRUE(fresh&&fresh->valid);ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].source_stamp,demand.source_stamp);
  EXPECT_EQ(sent[0].body_source_stamp,demand.body_source_stamp);
  EXPECT_EQ(sent[0].sequence,demand.sequence);EXPECT_EQ(sent[0].velocity,demand.velocity);
  EXPECT_EQ(t::timeNs(sent[0].valid_until),101100000000LL);
  EXPECT_FALSE(Access::pendingDemand(f.native));
  EXPECT_FALSE(Access::consumeRefreshWake(f.native));
  Access::checkCurve(f.native,f.map,101030000000LL,true,emit);
  EXPECT_EQ(sent.size(),1U); // Same original demand is delivered exactly once.
}
TEST(LocalFollowerNative,IndependentBodyAndProofDeliveryOrdersCompleteOnlyTheOriginalLatestDemand) {
  for(bool proof_first:{false,true})for(bool body_already_delivered:{false,true}) {
    Integrated f;const auto prepared=expiringProofDemand(f,!body_already_delivered);
    ASSERT_TRUE(prepared);const auto demand=*prepared;ASSERT_FALSE(demand.hold)<<demand.reason;
    Access::removeActiveProof(f.native); // No completed state, not an expired state.
    const auto measured=body(101.,0.,0.);
    geometry_msgs::msg::Twist twist;
    const scan_planner::MeasuredBodyPose pose{measured.position,measured.orientation,101.,"odom"};
    std::vector<w::LocalMotionCommand> sent;const auto emit=[&](const auto& c){sent.push_back(c);};
    f.native.demandAt(demand,101012000000LL,emit);ASSERT_TRUE(sent.empty());
    const auto deliver_body=[&]{f.native.bodyAt(pose,twist,101000000000LL,101020000000LL,emit);};
    const auto deliver_proof=[&]{GridMapTestAccess::acquire(f.map,101000000000LL);
      const auto proof=Access::checkCurve(f.native,f.map,101020000000LL,true,emit);
      ASSERT_TRUE(proof&&proof->valid);};
    if(proof_first){deliver_proof();deliver_body();}else{deliver_body();deliver_proof();}
    ASSERT_EQ(sent.size(),1U)<<proof_first<<body_already_delivered;
    EXPECT_EQ(sent[0].sequence,demand.sequence);EXPECT_EQ(sent[0].source_stamp,demand.source_stamp);
    EXPECT_EQ(sent[0].body_source_stamp,demand.body_source_stamp);
    EXPECT_LE(t::timeNs(sent[0].valid_until),101100000000LL);
    deliver_body();deliver_proof();EXPECT_EQ(sent.size(),1U);
  }
}
TEST(LocalFollowerNative,ProofDeliveryCannotReviveCanceledExpiredSupersededOrOccupiedPendingMotion) {
  for(unsigned fault=0;fault<5;++fault) {
    Integrated f;const auto prepared=expiringProofDemand(f);ASSERT_TRUE(prepared);
    const auto d=*prepared;ASSERT_FALSE(d.hold)<<d.reason;
    Access::removeActiveProof(f.native); // Exercise genuinely missing-state delivery.
    unsigned nonzero=0;const auto emit=[&](const auto& c){if(!c.hold)++nonzero;};
    f.native.demandAt(d,101012000000LL,emit);ASSERT_TRUE(Access::pendingDemand(f.native));
    auto now=101020000000LL;
    if(fault==0){auto grant=lease(101.015,21);grant.allowed=false;grant.revoked=true;
      ASSERT_TRUE(f.native.taskLeaseAt(grant,101015000000LL));}
    if(fault==1)now=101101000000LL; // Real original body expiry.
    if(fault==2){auto zero=d;++zero.sequence;zero.hold=true;zero.velocity={};
      f.native.demandAt(zero,101015000000LL,emit);}
    if(fault==3)GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.);
    if(fault==4){auto barrier=version();++barrier.reference_generation;
      ASSERT_TRUE(f.native.cancelReference(barrier));}
    GridMapTestAccess::acquire(f.map,101000000000LL);
    Access::checkCurve(f.native,f.map,now,true,emit);
    EXPECT_EQ(nonzero,0U)<<fault;
  }
}
TEST(LocalFollowerNative,SameProductionCurveAndMapCompleteAfterObsoleteSubLaneWouldExpire) {
  Integrated f;
  const auto wire=tag();
  Eigen::MatrixXd controls(3,wire.trajectory.pos_pts.size());
  Eigen::VectorXd knots(wire.trajectory.knots.size());
  for(std::size_t i=0;i<wire.trajectory.pos_pts.size();++i) {
    const auto&p=wire.trajectory.pos_pts[i];controls.col(i)=Eigen::Vector3d(p.x,p.y,p.z);
  }
  for(std::size_t i=0;i<wire.trajectory.knots.size();++i)knots[i]=wire.trajectory.knots[i];
  scan_planner::UniformBspline curve(controls,3,.2);curve.setKnot(knots);
  const scan_planner::MeasuredBodyPose measured{{0.,0.,.55},Eigen::Quaterniond::Identity(),100.,"odom"};
  const auto certify=[&](double budget,scan_planner::CurveCheckCompletion& completion) {
    f.map.beginObservedProof();
    bool first=true;
    return scan_planner::wholeCurveCollisionFree(curve,.05,
      scan_planner::bodyExtent(f.map.planningBodyEnvelope()),
      [&](const Eigen::Vector3d& point,double yaw) {
        // Model the logged ~12 ms native work with a controlled 13 ms reader
        // scheduling delay. Geometry and production occupied queries are
        // identical on both sides; no alternate map or weakened collision.
        if(first){first=false;std::this_thread::sleep_for(std::chrono::milliseconds(13));}
        return f.map.getPlanningInflateOccupancy(point,yaw);
      },measured,"odom",100.,.4,200000,budget,0.,f.heading,0.,
      scan_planner::MeasuredConnectionPolicy::CommittedSweptConnection,.15,&completion);
  };
  scan_planner::CurveCheckCompletion completion;
  EXPECT_FALSE(certify(.012,completion));
  EXPECT_EQ(completion,scan_planner::CurveCheckCompletion::BudgetExhausted);
  EXPECT_TRUE(certify(scan_planner::ValidationCycle::local_curve_budget_s,completion));
  EXPECT_EQ(completion,scan_planner::CurveCheckCompletion::Clear);
  GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.);
  EXPECT_FALSE(certify(scan_planner::ValidationCycle::local_curve_budget_s,completion));
  EXPECT_EQ(completion,scan_planner::CurveCheckCompletion::Blocked);
}
TEST(LocalFollowerNative,MissingHistoricalCommitSampleSyncsGeometryButFreshIndependentStateStillControlsMotion) {
  Integrated f(true,true);
  ASSERT_EQ(f.native.committedTrajectory(),f.follower.controller().trajectoryId());
  EXPECT_EQ(Access::bodySource(f.native),100.);EXPECT_EQ(Access::bodyHistorySize(f.native),1U);
  // The LocalState at 100.01 was acquired by LocalFollower, but never arrived
  // on Native's independent stream. Geometry synchronized without copying it
  // into Native state or minting a current collision proof.
  const auto before=f.native.committedView();ASSERT_TRUE(before&&before->proof);
  auto blocked=f.command(100.02,0.,0.,1,true,false,false);
  EXPECT_FALSE(blocked);
  EXPECT_EQ(Access::diagnostics(f.native).stage,scan_planner::MotionCheckStage::BodyBehindDemand);
  const auto after=f.native.committedView();ASSERT_TRUE(after&&after->proof);
  EXPECT_EQ(after->proof->sequence,before->proof->sequence);
  EXPECT_EQ(after->proof->body_source_stamp,t::stamp(100.));
  EXPECT_EQ(after->proof->valid_until,before->proof->valid_until);
  // A later real sample (not retransmission of the missing stamp) restores the
  // same task/curve through the actual production map curve verdict.
  auto recovered=f.command(100.05,0.,0.,2);ASSERT_TRUE(recovered);
  EXPECT_GT(recovered->velocity.linear.x,0.);EXPECT_EQ(f.native.committedTrajectory(),1);
  EXPECT_EQ(f.follower.activeVersion()->route_hash,version().route_hash);
  EXPECT_EQ(recovered->body_source_stamp,t::stamp(100.05));
  EXPECT_LE(t::timeNs(recovered->valid_until),t::timeNs(recovered->body_source_stamp)+100000000LL);
  EXPECT_FALSE(Access::checkDemand(f.native,f.map,100151000000LL));
  GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.);
  const auto stopped=f.command(100.10,0.,0.,3,false);
  EXPECT_TRUE(!stopped||stopped->hold);
}
TEST(LocalFollowerNative,RealOccupiedUnknownAndExpiredStateNeverProduceNonzeroCommand) {
  for(int kind=0;kind<3;++kind){Integrated f(false); // opt-in strict comparison
    if(kind<2)GridMapTestAccess::voxel(f.map,{.3,0.,.55},kind==0?2.:-1.01);
    auto out=f.command(100.05,0.,0.,1,kind==2);
    if(kind<2)EXPECT_TRUE(!out||out->hold)<<kind;
    else {ASSERT_TRUE(out);EXPECT_FALSE(Access::checkDemand(f.native,f.map,100151000000LL));}
  }
}
TEST(LocalFollowerNative,FormalOfficialPolicyDoesNotAddSecondUnknownVetoOrClearRawMap) {
  for(int kind=0;kind<2;++kind) {
    Integrated f;const Eigen::Vector3d point{.3,0.,.55};const double raw=kind==0?-1.01:2.;
    GridMapTestAccess::voxel(f.map,point,raw);
    auto out=f.command(100.05,0.,0.,1,kind==0);
    EXPECT_EQ(out&&!out->hold,kind==0);EXPECT_EQ(GridMapTestAccess::raw(f.map,point),raw);
    if(out&&!out->hold){EXPECT_GT(out->velocity.linear.x,0.);EXPECT_EQ(GridMapTestAccess::freeStamp(f.map,point),0);
      EXPECT_FALSE(Access::checkDemand(f.native,f.map,100151000000LL));}
  }
}
TEST(LocalFollowerNative,MovingFutureTailCommitsAtMeasuredBoundaryWithoutSdkCurveAck) {
  Integrated f;double x=0.,vx=0.;bool prepared=false,committed=false;
  double previous=0.;unsigned nonzero=0;
  for(unsigned i=1;i<=50;++i){const double now=100.+i*.05;
    // The plant is driven by the preceding command, not a recorded trajectory.
    f.acquire(now,x,vx);f.native.progressAt(f.progress(now),now);
    auto active=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(now)),true);
    ASSERT_TRUE(active&&active->valid);ASSERT_TRUE(f.follower.validation(*active));
    if(i==10){ASSERT_TRUE(f.prepareTail(now));prepared=true;}
    if(prepared&&!committed)committed=f.commitTailIfReady(now);
    auto out=f.command(now,x,vx,i,true,false);ASSERT_TRUE(out);
    EXPECT_LE(std::abs(out->velocity.linear.x-previous),.35*.05+1e-8);
    if(out->velocity.linear.x>0.)++nonzero;
    previous=vx=out->velocity.linear.x;x+=vx*.05;
  }
  EXPECT_TRUE(committed)<<f.follower.candidateReason();EXPECT_GT(nonzero,40U);
  EXPECT_GT(x,.10);EXPECT_EQ(f.native.committedTrajectory(),2);
  EXPECT_EQ(f.follower.controller().trajectoryId(),2);EXPECT_EQ(f.follower.commitSequence(),2U);
  EXPECT_EQ(f.follower.activeVersion()->route_hash,version().route_hash);
}
TEST(LocalFollowerNative,PeriodicFutureSplicesCoalesceUntilMeasuredEntryInsteadOfChasingNewSeams) {
  for(const bool coalesce:{false,true}) {
    Integrated f(true,false,{},tag(1,{0.,0.,.55},{.3,0.,0.},Eigen::Vector3d::Zero()));
    unsigned publications=0,skipped=0;bool committed=false;double commit_at=0.;
    const auto original_route=version().route_hash;
    for(unsigned i=1;i<=24;++i) {
      const double now=100.+i*.05,x=.3*(now-100.);
      // Acquired-state scheduling fixture, NOT a plant/robot acceptance model.
      // Both schedules receive identical body/ray sources and Native queries.
      ASSERT_TRUE(f.command(now,x,.3,i));
      const auto ns=t::timeNs(t::stamp(now));
      if(Access::pendingCurve(f.native)) {
        const auto proof=Access::checkCurve(f.native,f.map,ns);
        ASSERT_TRUE(proof&&proof->valid)<< (proof?proof->reason:"no proof");
        ASSERT_TRUE(f.follower.validation(*proof));
      }
      if((i-1)%4==0) { // Healthy results arrive every .2 s; each seam is .8 s ahead.
        const auto disposition=f.native.pendingSpliceDispositionAt(version(),ns);
        if(coalesce&&disposition.keep)++skipped;
        else {
          ASSERT_TRUE(f.prepareTail(now,2+publications,13));++publications;
          const auto candidate=Access::pendingCurve(f.native);ASSERT_TRUE(candidate);
          EXPECT_NEAR(candidate->splice.splice_arc_length-f.follower.controller().progressArc(),.24,.001);
          EXPECT_GT(candidate->splice.splice_arc_length-f.follower.controller().progressArc(),config().join_limit);
          const auto proof=Access::checkCurve(f.native,f.map,ns);
          ASSERT_TRUE(proof&&proof->valid)<<(proof?proof->reason:"no proof");
          ASSERT_TRUE(f.follower.validation(*proof));
        }
      }
      const auto commit=f.follower.tryCommit(now,now-90.);
      if(commit) {
        ASSERT_TRUE(f.native.localCommitAt(*commit,ns));committed=true;commit_at=now;break;
      }
    }
    if(coalesce) {
      EXPECT_TRUE(committed)<<f.follower.candidateReason();EXPECT_EQ(publications,1U);EXPECT_GE(skipped,3U);
      EXPECT_GE(commit_at,100.80);EXPECT_LE(commit_at,100.90);
      EXPECT_EQ(f.native.committedTrajectory(),2);EXPECT_EQ(f.follower.commitSequence(),2U);
      EXPECT_FALSE(f.native.pendingSpliceDispositionAt(version(),t::timeNs(t::stamp(commit_at))).keep);
    } else {
      EXPECT_FALSE(committed);EXPECT_EQ(publications,6U);EXPECT_EQ(f.native.committedTrajectory(),1);
    }
    EXPECT_EQ(f.follower.activeVersion()->route_hash,original_route);
  }
}
TEST(LocalFollowerNative,LightweightDiagnosticKeyKeepsCommittedIdentityAndOriginalProofClock) {
  Integrated f;Access::activeDebug(f.native);
  const auto before=f.native.committedView();ASSERT_TRUE(before&&before->proof);
  EXPECT_EQ(scan_planner::executionVersion(before->spline),version());
  for(unsigned i=0;i<100;++i) {
    const auto key=f.native.committedDiagnosticAt(100010000000LL,.4);ASSERT_TRUE(key);
    EXPECT_EQ(key->version,version());EXPECT_EQ(key->trajectory_id,1);
    EXPECT_TRUE(key->accepted);EXPECT_EQ(key->phase,"accepted");
  }
  const auto after=f.native.committedView();ASSERT_TRUE(after&&after->proof);
  EXPECT_EQ(after->proof->sequence,before->proof->sequence);
  EXPECT_EQ(after->proof->valid_until,before->proof->valid_until);
  f.native.candidate(tag(2),f.reference,f.heading);
  auto key=f.native.committedDiagnosticAt(100010000000LL,.4);ASSERT_TRUE(key);
  EXPECT_EQ(key->trajectory_id,1); // A candidate is never the committed visual identity.
  key=f.native.committedDiagnosticAt(t::timeNs(before->proof->valid_until),.4);ASSERT_TRUE(key);
  EXPECT_FALSE(key->accepted);EXPECT_EQ(key->phase,"waiting_current_validation");
  f.acquire(100.05,0.,0.);GridMapTestAccess::voxel(f.map,{0.,0.,.55},2.);
  const auto negative=Access::checkCurve(f.native,f.map,100050000000LL,true);
  ASSERT_TRUE(negative&&!negative->valid);ASSERT_EQ(negative->reason,"occupied");
  key=f.native.committedDiagnosticAt(100050000000LL,.4);ASSERT_TRUE(key);
  EXPECT_FALSE(key->accepted);EXPECT_EQ(key->phase,"occupied");
  auto cancelled=version();++cancelled.reference_generation;
  ASSERT_TRUE(f.native.cancelReference(cancelled));
  EXPECT_FALSE(f.native.committedDiagnosticAt(100050000000LL,.4));
}
TEST(LocalFollowerNative,PendingSpliceFirstProofWaitUsesOriginalPublicationAndNeverRenewsEvidence) {
  Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
  ASSERT_TRUE(f.command(100.05,.005,.1,1));ASSERT_TRUE(f.prepareTail(100.05));
  const auto original=Access::pendingCurve(f.native);ASSERT_TRUE(original);
  for(unsigned i=2;i<=9;++i) {
    const double now=100.+i*.05;ASSERT_TRUE(f.command(now,.1*(now-100.),.1,i));
    const auto before=f.native.committedView();ASSERT_TRUE(before&&before->proof);
    const auto disposition=f.native.pendingSpliceDispositionAt(version(),t::timeNs(t::stamp(now)));
    EXPECT_EQ(disposition.keep,i<9)<<disposition.reason;
    EXPECT_STREQ(disposition.reason,i<9?"pending_splice_waiting_first_proof":"pending_splice_first_proof_timeout");
    EXPECT_EQ(Access::pendingCurve(f.native)->trajectory.start_time,original->trajectory.start_time);
    const auto after=f.native.committedView();ASSERT_TRUE(after&&after->proof);
    EXPECT_EQ(after->proof->sequence,before->proof->sequence);
    EXPECT_EQ(after->proof->valid_until,before->proof->valid_until);
    EXPECT_EQ(f.native.committedTrajectory(),1);
  }
}
TEST(LocalFollowerNative,PendingSpliceNegativeRetiresButCompletedPositiveDoesNotExpire) {
  for(const bool occupied:{false,true}) {
    Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
    ASSERT_TRUE(f.command(100.05,.005,.1,1));ASSERT_TRUE(f.prepareTail(100.05));
    const auto positive=Access::checkCurve(f.native,f.map,100050000000LL);ASSERT_TRUE(positive&&positive->valid);
    ASSERT_TRUE(f.native.pendingSpliceDispositionAt(version(),100050000000LL).keep);
    double now=100.05;
    if(occupied) {
      GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.);
      const auto negative=Access::checkCurve(f.native,f.map,100050000000LL);
      ASSERT_TRUE(negative&&!negative->valid);EXPECT_EQ(negative->reason,"occupied");
    } else {
      // Keep actual 20 Hz body/control progress while no pending check runs.
      // The completed positive must survive the former 250 ms proof lease.
      for(unsigned i=2;i<=7;++i) {
        now=100.+i*.05;
        const auto command=f.command(now,.1*(now-100.),.1,i);
        ASSERT_TRUE(command);EXPECT_FALSE(command->hold)<<command->reason;
      }
      EXPECT_EQ(t::timeNs(positive->valid_until),0);
      const auto incumbent=f.native.committedView();ASSERT_TRUE(incumbent&&incumbent->progress);
      EXPECT_FALSE(incumbent->progress->holding);
    }
    const auto disposition=f.native.pendingSpliceDispositionAt(version(),t::timeNs(t::stamp(now)));
    EXPECT_EQ(disposition.keep,!occupied);
    if(occupied)EXPECT_STREQ(disposition.reason,"pending_splice_proof_unavailable");
    EXPECT_EQ(f.native.committedTrajectory(),1);
    f.native.cancelPending();EXPECT_FALSE(f.native.pendingSpliceDispositionAt(version(),t::timeNs(t::stamp(now))).keep);
    GridMapTestAccess::voxel(f.map,{.3,0.,.55},-1.);
    ASSERT_TRUE(f.prepareTail(now,3)); // Same task/route, a new bounded ordinary solve is not suppressed.
    const auto recovered=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(now)));
    ASSERT_TRUE(recovered&&recovered->valid);EXPECT_TRUE(f.native.pendingSpliceDispositionAt(version(),t::timeNs(t::stamp(now))).keep);
  }
}
TEST(LocalFollowerNative,PendingSpliceCannotHideCancellationReferenceChangesMeasuredDeviationOrMiss) {
  for(unsigned event=0;event<4;++event) {
    Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
    ASSERT_TRUE(f.command(100.05,.005,.1,1));ASSERT_TRUE(f.prepareTail(100.05));
    const auto proof=Access::checkCurve(f.native,f.map,100050000000LL);ASSERT_TRUE(proof&&proof->valid);
    ASSERT_TRUE(f.native.pendingSpliceDispositionAt(version(),100050000000LL).keep);
    if(event==0) {
      auto barrier=version();++barrier.reference_generation;ASSERT_TRUE(f.native.cancelReference(barrier));
      EXPECT_FALSE(f.native.pendingSpliceDispositionAt(version(),100050000000LL).keep);
      EXPECT_EQ(f.native.committedTrajectory(),-1);
    } else if(event==1) {
      for(unsigned identity=0;identity<4;++identity) {
        auto changed=version();
        if(identity==0)++changed.reference_generation;
        if(identity==1)++changed.context_sequence;
        if(identity==2)changed.localization_seed_id="new_seed";
        if(identity==3)changed.route_hash=std::string(64,'f');
        EXPECT_FALSE(f.native.pendingSpliceDispositionAt(changed,100050000000LL).keep);
      }
      EXPECT_TRUE(f.native.pendingSpliceDispositionAt(version(),100050000000LL).keep);
    } else if(event==2) {
      auto progress=f.progress(100.05);progress.header.stamp=t::stamp(100.10);
      geometry_msgs::msg::Twist twist;twist.linear.x=.1;
      f.native.body({{.005,.08,.55},Eigen::Quaterniond::Identity(),100.10,"odom"},twist,100100000000LL);
      f.native.progressAt(progress,100.10);
      const auto disposition=f.native.pendingSpliceDispositionAt(version(),100100000000LL);
      EXPECT_FALSE(disposition.keep);EXPECT_STREQ(disposition.reason,"pending_splice_measured_deviation");
      const auto reseed=f.native.consumeFormalReseedAt(100100000000LL);ASSERT_TRUE(reseed);
      EXPECT_TRUE(reseed->actual_body);EXPECT_EQ(reseed->reason,"measured_body_left_local_curve");
    } else {
      // Miss the original Native 5 cm along-tail domain, not its unrelated
      // 1.25 cm lateral residual (a normal control tick can cross by 2 cm).
      for(unsigned i=2;i<=28;++i)ASSERT_TRUE(f.command(100.+i*.05,.1*i*.05,.1,i));
      const auto disposition=f.native.pendingSpliceDispositionAt(version(),101400000000LL);
      EXPECT_FALSE(disposition.keep);EXPECT_STREQ(disposition.reason,"pending_splice_missed");
    }
    if(event!=0)EXPECT_EQ(f.native.committedTrajectory(),1);
  }
}
TEST(LocalFollowerNative,WholeProofKeepsExactUnixBodySourceAndRejectsUnacquiredPose) {
  Integrated f;
  for(const auto sub:{116000123LL,116000511LL,987654321LL}) {
    const auto ns=1791786242000000000LL+sub;
    scan_planner::MeasuredBodyPose p{{0.,0.,.55},Eigen::Quaterniond::Identity(),ns*1e-9,"odom"};
    geometry_msgs::msg::Twist velocity;f.native.body(p,velocity,ns);GridMapTestAccess::acquire(f.map,ns);
    auto proof=Access::checkCopiedBody(f.native,f.map,ns,p);ASSERT_TRUE(proof);
    EXPECT_TRUE(proof->valid)<<proof->reason;EXPECT_EQ(t::timeNs(proof->body_source_stamp),ns);
    p.position.y()=.001;auto bad=Access::checkCopiedBody(f.native,f.map,ns,p);
    // An unacquired sample cannot publish a new positive or replace the
    // later acquired-source verdict. Inconclusive checks may emit nothing.
    EXPECT_FALSE(bad&&bad->valid);if(bad){EXPECT_EQ(t::timeNs(bad->body_source_stamp),0);}
  }
}
TEST(LocalFollowerNative,ObstacleOnlyInsideFormerBrakingVolumeDoesNotVetoAcceptedScanCurve) {
  Integrated f;const Eigen::Vector3d obstacle{.05,.30,.55};
  GridMapTestAccess::voxel(f.map,obstacle,2.);
  // Show that this acquired occupied voxel really lies in the former second
  // volume, while the production Native SCAN footprint accepts the curve.
  f.acquire(100.05,0.,0.);
  geometry_msgs::msg::Twist desired,measured;desired.linear.x=.02;
  const scan_planner::MotionSupport supported(support());
  const auto former=scan_planner::validateMotionSweep(f.map,supported,f.braking,
    {0.,0.,.55},0.,measured,desired,"odom",.1);
  EXPECT_FALSE(former.valid);EXPECT_EQ(former.reason,"motion_sweep_occupied");
  auto out=f.command(100.05,0.,0.,1,true,false);ASSERT_TRUE(out);
  EXPECT_FALSE(out->hold);EXPECT_GT(out->velocity.linear.x,0.);
  EXPECT_EQ(GridMapTestAccess::raw(f.map,obstacle),2.);
  EXPECT_EQ(Access::motionProofCount(f.native),0U);
  // Even accidental use of the archived checker cannot mint a formal proof.
  Access::skipCurveProof(f.native,f.map,100050000000LL);
  EXPECT_EQ(Access::motionProofCount(f.native),0U);
}
TEST(LocalFollowerNative,CurrentOccupiedScanVerdictImmediatelyFencesOldPositiveCommand) {
  Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
  auto demand=*Access::latestDemand(f.native);ASSERT_FALSE(demand.hold);
  GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.);
  const auto negative=Access::checkCurve(f.native,f.map,100050000000LL,true);
  ASSERT_TRUE(negative);ASSERT_FALSE(negative->valid);EXPECT_EQ(negative->reason,"occupied");
  // Deliberately retain the controller's earlier positive proof: Native's
  // current committed negative must independently prevent a nonzero relay.
  ++demand.sequence;unsigned emitted=0;
  f.native.demandAt(demand,100050000000LL,[&](const auto&){++emitted;});
  EXPECT_EQ(emitted,0U);
  EXPECT_EQ(Access::diagnostics(f.native).stage,scan_planner::MotionCheckStage::ActiveProofUnavailable);
  ASSERT_TRUE(f.follower.validation(*negative));
  f.follower.step(100.05,100.05-90.); // advance past the deliberately injected demand
  auto hold=f.follower.step(100.05,100.05-90.);ASSERT_TRUE(hold.hold);
  f.native.demandAt(hold,100050000000LL,[&](const auto& command){
    ++emitted;EXPECT_TRUE(command.hold);EXPECT_TRUE(scan_planner::planarDemandIsZero(command.velocity));});
  EXPECT_EQ(emitted,1U);EXPECT_EQ(Access::motionProofCount(f.native),0U);
}
TEST(LocalFollowerNative,PositiveRecoveryRequiresFreshDemandAfterNegativeNativeCurveVerdict) {
  Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
  auto old_demand=*Access::latestDemand(f.native);
  GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.);
  const auto negative=Access::checkCurve(f.native,f.map,100050000000LL,true);
  ASSERT_TRUE(negative&&!negative->valid);
  GridMapTestAccess::voxel(f.map,{.3,0.,.55},-1.);f.acquire(100.06,0.,0.);
  const auto recovered=Access::checkCurve(f.native,f.map,100060000000LL,true);
  ASSERT_TRUE(recovered&&recovered->valid);EXPECT_GT(recovered->sequence,negative->sequence);
  ++old_demand.sequence;unsigned emitted=0;
  f.native.demandAt(old_demand,100060000000LL,[&](const auto&){++emitted;});
  EXPECT_EQ(emitted,0U);
  EXPECT_EQ(Access::diagnostics(f.native).stage,scan_planner::MotionCheckStage::CurveProofSuperseded);
  ASSERT_TRUE(f.follower.validation(*recovered));
  f.follower.step(100.06,100.06-90.); // advance past the deliberately injected demand
  const auto fresh=f.follower.step(100.061,100.061-90.);ASSERT_FALSE(fresh.hold)<<fresh.reason;
  EXPECT_EQ(fresh.validation_sequence,recovered->sequence);
  f.native.demandAt(fresh,100061000000LL,[&](const auto& command){++emitted;EXPECT_FALSE(command.hold);});
  EXPECT_EQ(emitted,1U);EXPECT_EQ(Access::motionProofCount(f.native),0U);
}
// Same production follower/Native/geometry input; only callback delivery order
// differs. The real state provider's 100 ms lease is deliberately unchanged.
struct OrderedCommand {
  Integrated f;
  w::MotionDemand next(double source,unsigned sequence) {
    const auto sample=body(source,0.,0.);
    EXPECT_TRUE(f.follower.body(sample,source,source-90.));
    GridMapTestAccess::acquire(f.map,t::timeNs(t::stamp(source)));
    f.pipeline_health.source_ns={t::timeNs(t::stamp(source)),t::timeNs(t::stamp(source))};
    f.pipeline_health.fusion_sequence=f.map.fusionTiming().sequence;
    f.pipeline_health.sampled_steady_ns=f.pipeline_health.fusion_completed_steady_ns=t::timeNs(t::stamp(source));
    const auto proof=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(source)),true);
    EXPECT_TRUE(proof&&proof->valid);if(proof)EXPECT_TRUE(f.follower.validation(*proof));
    const auto grant=lease(source,sequence);
    EXPECT_TRUE(f.follower.lease(grant,source));
    EXPECT_TRUE(f.native.taskLeaseAt(grant,t::timeNs(t::stamp(source))));
    EXPECT_TRUE(f.native.commit(f.follower.geometryPermit(source)));
    auto d=f.follower.step(source,source-90.);EXPECT_FALSE(d.hold)<<d.reason;return d;
  }
  void deliverBody(double source,std::int64_t receipt,
      const std::function<void(const w::LocalMotionCommand&)>& emit) {
    const auto o=body(source,0.,0.);geometry_msgs::msg::Twist twist;
    f.native.bodyAt({o.position,o.orientation,o.stamp,"odom"},twist,o.source_stamp_ns,receipt,emit);
  }
};
TEST(LocalFollowerNative,Every20HzDemandBeforeIndependentBodyRelaysOnArrivalWithoutRestamping) {
  for(const std::int64_t jitter:{1000000LL,9000000LL,27000000LL,49000000LL}) {
    OrderedCommand run;unsigned emitted=0;
    for(unsigned i=1;i<=20;++i) {
      const double source=100.+i*.05;const auto now=t::timeNs(t::stamp(source));
      const auto d=run.next(source,i);
      run.f.native.demandAt(d,now,[&](const auto&){++emitted;});
      EXPECT_EQ(emitted,i-1)<<jitter;EXPECT_TRUE(Access::pendingDemand(run.f.native));
      run.deliverBody(source,now+jitter,[&](const auto& command){
        ++emitted;EXPECT_FALSE(command.hold);EXPECT_EQ(command.sequence,d.sequence);
        EXPECT_EQ(command.source_stamp,d.source_stamp);EXPECT_EQ(command.body_source_stamp,d.body_source_stamp);
        EXPECT_EQ(t::timeNs(command.body_valid_until),now+100000000LL);
        EXPECT_LE(t::timeNs(command.valid_until),now+100000000LL);
        EXPECT_GT(t::timeNs(command.valid_until),now+jitter);
      });
      EXPECT_EQ(emitted,i);EXPECT_FALSE(Access::pendingDemand(run.f.native));
      // A duplicate independent body callback is not a 2nd command send.
      run.deliverBody(source,now+jitter+1,[&](const auto&){++emitted;});EXPECT_EQ(emitted,i);
      run.f.native.progressAt(run.f.progress(source),source);
    }
  }
}
TEST(LocalFollowerNative,DeferredDeliveryNeverExtendsOriginalStateOrShorterProviderDeadline) {
  for(unsigned fault=0;fault<3;++fault) {
    OrderedCommand run;auto d=run.next(100.05,1);const auto original=d;
    const std::int64_t now=100050000000LL;
    if(fault==1)d.valid_until=t::stamp(100.075); // provider may expire earlier than 100 ms
    unsigned emitted=0;run.f.native.demandAt(d,now,[&](const auto&){++emitted;});
    ASSERT_TRUE(Access::pendingDemand(run.f.native));
    const auto receipt=fault==0?now+100000000LL:fault==1?now+25000000LL:now+101000000LL;
    run.deliverBody(100.05,receipt,[&](const auto&){++emitted;});
    EXPECT_EQ(emitted,0U)<<fault;EXPECT_FALSE(Access::pendingDemand(run.f.native));
    EXPECT_EQ(d.source_stamp,original.source_stamp);EXPECT_EQ(d.body_source_stamp,original.body_source_stamp);
  }
}
TEST(LocalFollowerNative,NewerHoldCancelAndNegativeProofCannotReviveDeferredNonzero) {
  for(unsigned fault=0;fault<4;++fault) {
    OrderedCommand run;auto d=run.next(100.05,1);unsigned nonzero=0;
    const auto count=[&](const auto& c){if(!c.hold)++nonzero;};
    run.f.native.demandAt(d,100050000000LL,count);ASSERT_TRUE(Access::pendingDemand(run.f.native));
    if(fault==0) {
      auto hold=d;++hold.sequence;hold.hold=true;hold.velocity={};
      run.f.native.demandAt(hold,100051000000LL,count);
      run.f.native.demandAt(d,100052000000LL,count); // older topic delivery loses to zero high-water
    } else if(fault==1) {
      auto barrier=version();++barrier.reference_generation;ASSERT_TRUE(run.f.native.cancelReference(barrier));
    } else if(fault==2) {
      auto denied=lease(100.051,2);denied.allowed=false;denied.phase="holding";
      ASSERT_TRUE(run.f.native.taskLeaseAt(denied,100051000000LL));
      ASSERT_TRUE(run.f.native.taskLeaseAt(lease(100.052,3),100052000000LL));
    } else {
      GridMapTestAccess::voxel(run.f.map,{.3,0.,.55},2.);
      const auto p=Access::checkCurve(run.f.native,run.f.map,100052000000LL,true);
      ASSERT_TRUE(p&&!p->valid);
    }
    run.deliverBody(100.05,100060000000LL,count);
    EXPECT_EQ(nonzero,0U)<<fault;EXPECT_FALSE(Access::pendingDemand(run.f.native));
  }
}
TEST(LocalFollowerNative,OnlyLatestDeferredDemandSurvivesBodyTopicReordering) {
  OrderedCommand run;auto first=run.next(100.05,1);unsigned emitted=0;
  run.f.native.demandAt(first,100050000000LL,[&](const auto&){++emitted;});
  auto second=first;++second.sequence;second.source_stamp=t::stamp(100.06);second.velocity.linear.x=.02;
  run.f.native.demandAt(second,100060000000LL,[&](const auto&){++emitted;});
  run.f.native.demandAt(first,100061000000LL,[&](const auto&){++emitted;});
  run.deliverBody(100.05,100070000000LL,[&](const auto& command){
    ++emitted;EXPECT_EQ(command.sequence,second.sequence);EXPECT_EQ(command.source_stamp,second.source_stamp);
    EXPECT_EQ(command.velocity,second.velocity);EXPECT_EQ(command.body_source_stamp,first.body_source_stamp);
    EXPECT_LE(t::timeNs(command.valid_until),100150000000LL);
  });
  EXPECT_EQ(emitted,1U);EXPECT_FALSE(Access::pendingDemand(run.f.native));
}
TEST(LocalFollowerNative,AcceptedCurveCommitRetainsExactPendingIncumbentUntilSuccessorDemand) {
  for(const bool successor_first:{false,true}) {
  SCOPED_TRACE(successor_first);
  OrderedCommand run;const auto d=run.next(100.05,1);unsigned emitted=0;
  std::optional<w::LocalMotionCommand> last;
  const auto count=[&](const auto& c){++emitted;last=c;};
  run.f.native.demandAt(d,100050000000LL,count);ASSERT_TRUE(Access::pendingDemand(run.f.native));
  auto replacement=tag(2);replacement.trajectory.start_time=replacement.join_source_stamp=t::stamp(100.05);
  run.f.native.candidate(replacement,run.f.reference,run.f.heading);
  ASSERT_TRUE(run.f.follower.candidate(replacement));
  const auto proof=Access::checkCurve(run.f.native,run.f.map,100052000000LL);
  ASSERT_TRUE(proof&&proof->valid);ASSERT_TRUE(run.f.follower.validation(*proof));
  const auto commit=run.f.follower.tryCommit(100.052,10.052);
  ASSERT_TRUE(commit)<<run.f.follower.candidateReason();ASSERT_EQ(commit->expected_trajectory_id,1);
  ASSERT_TRUE(run.f.native.localCommitAt(*commit,100052000000LL));
  ASSERT_TRUE(run.f.native.commit(run.f.follower.geometryPermit(100.055)));
  const auto successor=run.f.follower.step(100.055,100.055-90.);
  ASSERT_FALSE(successor.hold);ASSERT_EQ(successor.trajectory_id,2);
  if(successor_first)run.f.native.demandAt(successor,100055000000LL,count);
  run.deliverBody(100.05,100060000000LL,count);
  ASSERT_EQ(emitted,1U);ASSERT_TRUE(last);
  EXPECT_EQ(last->trajectory_id,successor_first?2:1);
  EXPECT_EQ(last->sequence,successor_first?successor.sequence:d.sequence);
  EXPECT_EQ(last->source_stamp,successor_first?successor.source_stamp:d.source_stamp);
  EXPECT_EQ(last->body_source_stamp,d.body_source_stamp);
  EXPECT_FALSE(Access::pendingDemand(run.f.native));
  if(!successor_first) {
    run.f.native.demandAt(successor,100061000000LL,count);
    ASSERT_EQ(emitted,2U);ASSERT_TRUE(last);EXPECT_EQ(last->trajectory_id,2);
  }
  const auto after_successor=emitted;
  auto reordered=d;reordered.sequence=successor.sequence+1;
  run.f.native.demandAt(reordered,100062000000LL,count);
  EXPECT_EQ(emitted,after_successor);EXPECT_FALSE(Access::pendingDemand(run.f.native));
  EXPECT_EQ(run.f.native.committedTrajectory(),2);
  }
}
struct DelayedCommit {
  Integrated f;w::LocalTrajectoryCommit commit;w::MotionDemand demand;
  DelayedCommit() {
    EXPECT_TRUE(f.command(100.05,0.,0.,1));
    auto replacement=tag(2);replacement.trajectory.start_time=replacement.join_source_stamp=t::stamp(100.05);
    f.native.candidate(replacement,f.reference,f.heading);EXPECT_TRUE(f.follower.candidate(replacement));
    const auto proof=Access::checkCurve(f.native,f.map,100060000000LL);
    EXPECT_TRUE(proof&&proof->valid);if(proof)EXPECT_TRUE(f.follower.validation(*proof));
    const auto accepted=f.follower.tryCommit(100.06,10.06);
    EXPECT_TRUE(accepted)<<f.follower.candidateReason();if(accepted)commit=*accepted;
    EXPECT_TRUE(f.native.commit(f.follower.geometryPermit(100.06)));
    demand=f.follower.step(100.06,10.06);EXPECT_FALSE(demand.hold)<<demand.reason;
    EXPECT_EQ(demand.trajectory_id,2);EXPECT_EQ(f.native.committedTrajectory(),1);
  }
};
TEST(LocalFollowerNative,KnownCandidateDemandBeforeLocalCommitCompletesSameOriginalCommand) {
  DelayedCommit run;unsigned emitted=0;const auto original=run.demand;
  run.f.native.demandAt(run.demand,100060000000LL,[&](const auto&){++emitted;});
  EXPECT_EQ(emitted,0U);ASSERT_TRUE(Access::pendingDemand(run.f.native));
  // Body delivery is NOT permission to promote uncommitted geometry.
  const auto o=body(100.07,0.,0.);geometry_msgs::msg::Twist twist;
  run.f.native.bodyAt({o.position,o.orientation,o.stamp,"odom"},twist,o.source_stamp_ns,100070000000LL,
    [&](const auto&){++emitted;});
  EXPECT_EQ(emitted,0U);EXPECT_EQ(run.f.native.committedTrajectory(),1);
  ASSERT_TRUE(run.f.native.localCommitAt(run.commit,100075000000LL,[&](const auto& c){
    ++emitted;EXPECT_FALSE(c.hold);EXPECT_EQ(c.trajectory_id,2);EXPECT_EQ(c.sequence,original.sequence);
    EXPECT_EQ(c.source_stamp,original.source_stamp);EXPECT_EQ(c.body_source_stamp,original.body_source_stamp);
    EXPECT_EQ(c.velocity,original.velocity);EXPECT_LE(t::timeNs(c.valid_until),100150000000LL);
  }));
  EXPECT_EQ(emitted,1U);EXPECT_FALSE(Access::pendingDemand(run.f.native));
  EXPECT_EQ(run.f.native.committedTrajectory(),2);
}
struct ReservedCommit {
  Integrated f;w::LocalTrajectoryCommit prepare;
  ReservedCommit() {
    EXPECT_TRUE(f.command(100.05,0.,0.,1));
    auto replacement=tag(2);replacement.trajectory.start_time=replacement.join_source_stamp=t::stamp(100.05);
    f.native.candidate(replacement,f.reference,f.heading);EXPECT_TRUE(f.follower.candidate(replacement));
    const auto proof=Access::checkCurve(f.native,f.map,100060000000LL);
    EXPECT_TRUE(proof&&proof->valid);if(proof)EXPECT_TRUE(f.follower.validation(*proof));
    const auto request=f.follower.exchangeCommit(100.06,10.06);EXPECT_TRUE(request);
    if(request)prepare=*request;
    EXPECT_EQ(prepare.disposition,"prepare");EXPECT_EQ(f.follower.controller().trajectoryId(),1);
  }
  auto reserve() {
    auto reply=f.native.localCommitExchangeAt(prepare,100061000000LL);
    EXPECT_TRUE(reply.accepted)<<reply.reason;EXPECT_EQ(reply.disposition,"prepared");
    EXPECT_TRUE(f.follower.commitResult(reply,100.062,10.062));return reply;
  }
};
TEST(LocalFollowerNative,NewCandidateDoesNotEraseAlreadyPreparedLocalCommit) {
  ReservedCommit run;run.reserve();
  auto newer=tag(3);newer.trajectory.start_time=newer.join_source_stamp=t::stamp(100.065);
  run.f.native.candidate(newer,run.f.reference,run.f.heading);
  EXPECT_TRUE(run.f.follower.candidate(newer));
  const auto commit=run.f.follower.exchangeCommit(100.07,10.07);ASSERT_TRUE(commit);
  ASSERT_EQ(commit->trajectory_id,2);ASSERT_EQ(commit->disposition,"committed");
  EXPECT_EQ(run.f.native.committedTrajectory(),1);
  const auto result=run.f.native.localCommitExchangeAt(*commit,100075000000LL);
  EXPECT_TRUE(result.accepted)<<result.reason;
  EXPECT_TRUE(run.f.follower.commitResult(result,100.076,10.076));
  EXPECT_EQ(run.f.native.committedTrajectory(),2);EXPECT_EQ(run.f.follower.controller().trajectoryId(),2);
}
TEST(LocalFollowerNative,SelectedCandidateCanReserveBeforeNewestWaitingCandidate) {
  ReservedCommit run;auto newer=tag(3);newer.trajectory.start_time=newer.join_source_stamp=t::stamp(100.065);
  run.f.native.candidate(newer,run.f.reference,run.f.heading);ASSERT_TRUE(run.f.follower.candidate(newer));
  auto reserved=run.f.native.localCommitExchangeAt(run.prepare,100066000000LL);
  ASSERT_TRUE(reserved.accepted);ASSERT_TRUE(run.f.follower.commitResult(reserved,100.066,10.066));
  EXPECT_EQ(run.f.follower.controller().trajectoryId(),1);EXPECT_EQ(run.f.native.committedTrajectory(),1);
  const auto first=run.f.follower.exchangeCommit(100.067,10.067);ASSERT_TRUE(first);
  ASSERT_EQ(first->trajectory_id,2);
  const auto committed=run.f.native.localCommitExchangeAt(*first,100068000000LL);
  ASSERT_TRUE(committed.accepted);ASSERT_TRUE(run.f.follower.commitResult(committed,100.069,10.069));
  const auto proof=Access::checkCurve(run.f.native,run.f.map,100070000000LL);
  ASSERT_TRUE(proof&&proof->valid);ASSERT_TRUE(run.f.follower.validation(*proof));
  const auto retry=run.f.follower.exchangeCommit(100.07,10.07);ASSERT_TRUE(retry);
  EXPECT_EQ(retry->trajectory_id,3);EXPECT_GT(retry->commit_sequence,run.prepare.commit_sequence);
  EXPECT_TRUE(run.f.native.localCommitExchangeAt(*retry,100071000000LL).accepted);
}
TEST(LocalFollowerNative,CommitAckRetryIsOriginalIdempotentAndNeverStartsAnotherTransaction) {
  ReservedCommit run;run.reserve();
  const auto commit=run.f.follower.exchangeCommit(100.07,10.07);ASSERT_TRUE(commit);
  const auto first=run.f.native.localCommitExchangeAt(*commit,100075000000LL);ASSERT_TRUE(first.accepted);
  auto newer=tag(3);run.f.native.candidate(newer,run.f.reference,run.f.heading);
  ASSERT_TRUE(run.f.follower.candidate(newer));
  const auto repeat=run.f.follower.exchangeCommit(100.08,10.08);ASSERT_TRUE(repeat);EXPECT_EQ(*repeat,*commit);
  const auto again=run.f.native.localCommitExchangeAt(*repeat,100085000000LL);
  EXPECT_EQ(first,again);EXPECT_EQ(run.f.native.committedTrajectory(),2);
  EXPECT_TRUE(run.f.follower.commitResult(again,100.086,10.086));
  EXPECT_FALSE(run.f.follower.commitResult(first,100.087,10.087));
}
TEST(LocalFollowerNative,PreparationExpiryOrCurrentNegativeCannotReplaceIncumbent) {
  for(const bool negative:{false,true}) {
    ReservedCommit run;if(negative)run.reserve();
    if(negative) {
      GridMapTestAccess::voxel(run.f.map,{.3,0.,.55},2.);
      const auto proof=Access::checkCurve(run.f.native,run.f.map,100070000000LL);
      ASSERT_TRUE(proof&&!proof->valid);ASSERT_TRUE(run.f.follower.validation(*proof));
    }
    const double now=negative?100.075:100.5;
    const auto abort=run.f.follower.exchangeCommit(now,now-90.);ASSERT_TRUE(abort);
    EXPECT_EQ(abort->disposition,"abort");EXPECT_EQ(run.f.follower.controller().trajectoryId(),1);
    const auto released=run.f.native.localCommitExchangeAt(*abort,t::timeNs(t::stamp(now)));
    EXPECT_EQ(released.disposition,negative?"aborted":"rejected");EXPECT_EQ(run.f.native.committedTrajectory(),1);
    EXPECT_FALSE(run.f.native.localCommitExchangeAt(run.prepare,t::timeNs(t::stamp(now+.001))).accepted);
  }
}
TEST(LocalFollowerNative,PreparedCandidateStillRechecksLatestMeasuredEntry) {
  ReservedCommit run;run.reserve();run.f.acquire(100.07,0.03,0.);
  // A real 3 cm advance along the same candidate is a valid rolling geometric
  // join, not a reason to retain the obsolete 1.25 cm/derivative-velocity veto.
  const auto commit=run.f.follower.exchangeCommit(100.07,10.07);
  ASSERT_TRUE(commit)<<run.f.follower.candidateReason();
  ASSERT_EQ(commit->disposition,"committed");
  EXPECT_EQ(commit->body_source_stamp,t::stamp(100.07));
  EXPECT_NEAR(commit->measured_pose.position.x,.03,1e-9);
  EXPECT_EQ(run.f.native.committedTrajectory(),1);
  const auto accepted=run.f.native.localCommitExchangeAt(*commit,100071000000LL);
  ASSERT_TRUE(accepted.accepted)<<accepted.reason;
  ASSERT_TRUE(run.f.follower.commitResult(accepted,100.072,10.072));
  EXPECT_EQ(run.f.follower.controller().trajectoryId(),2);EXPECT_EQ(run.f.native.committedTrajectory(),2);
}
TEST(LocalFollowerNative,PreparedCandidateStillRejectsCurrentBodyOutsideConnectorOrOtherFloor) {
  for(unsigned variant=0;variant<2;++variant) {
    ReservedCommit run;run.reserve();
    auto measured=body(100.07,0.,0.);
    if(variant==0)measured.position.y()=.151;
    else measured.position.z()+=3.;
    ASSERT_TRUE(run.f.follower.body(measured,100.07,10.07));
    geometry_msgs::msg::Twist twist;
    run.f.native.body({measured.position,measured.orientation,measured.stamp,"odom"},twist,measured.source_stamp_ns);
    const auto result=run.f.follower.exchangeCommit(100.07,10.07);
    EXPECT_TRUE(!result||result->disposition=="abort")<<variant;
    EXPECT_EQ(run.f.follower.controller().trajectoryId(),1);EXPECT_EQ(run.f.native.committedTrajectory(),1);
    if(result)EXPECT_TRUE(run.f.native.localCommitExchangeAt(*result,100071000000LL).accepted);
  }
}
TEST(LocalFollowerNative,CandidateCollisionRefreshKeepsIndependentOlderBodyStampAndRejectsActualExpiry) {
  for(bool expired:{false,true}) {
    Integrated f;
    auto candidate=tag(2);
    candidate.trajectory.start_time=candidate.join_source_stamp=t::stamp(100.05);
    f.native.candidate(candidate,f.reference,f.heading);
    // Only the raw map is renewed. Native's real body remains acquired at 100,
    // which may legally precede the solve body's independent topic delivery.
    const auto now=expired?100401000000LL:100060000000LL;
    GridMapTestAccess::acquire(f.map,now);
    const auto proof=Access::checkCurve(f.native,f.map,now);
    if(expired)EXPECT_FALSE(proof); // Missing body is incomplete, not a fake collision.
    else {ASSERT_TRUE(proof);EXPECT_TRUE(proof->valid)<<proof->reason;
      EXPECT_EQ(proof->body_source_stamp,t::stamp(100.));}
    EXPECT_EQ(Access::bodySource(f.native),100.);
  }
}
TEST(LocalFollowerNative,UnknownAbortCannotPoisonReservationSequenceOrAlterReservedGeometry) {
  ReservedCommit run;auto poison=run.prepare;poison.disposition="abort";
  poison.commit_sequence=std::numeric_limits<std::uint64_t>::max();
  EXPECT_FALSE(run.f.native.localCommitExchangeAt(poison,100061000000LL).accepted);
  run.reserve();auto mutated=tag(2);mutated.trajectory.pos_pts[5].y+=1.;
  EXPECT_FALSE(run.f.follower.candidate(mutated));
  const auto commit=run.f.follower.exchangeCommit(100.07,10.07);ASSERT_TRUE(commit);
  EXPECT_TRUE(run.f.native.localCommitExchangeAt(*commit,100075000000LL).accepted);
}
TEST(LocalFollowerNative,RetirementAndForeignAckNeverReviveReservedCandidate) {
  ReservedCommit run;const auto ready=run.reserve();
  auto corrupt=ready;corrupt.version.task_id="other";
  EXPECT_FALSE(run.f.follower.commitResult(corrupt,100.063,10.063));
  auto barrier=version();++barrier.reference_generation;ASSERT_TRUE(run.f.native.cancelReference(barrier));
  run.f.follower.retire("cancelled");
  EXPECT_FALSE(run.f.follower.commitResult(ready,100.07,10.07));
  EXPECT_FALSE(run.f.follower.exchangeCommit(100.07,10.07));
  EXPECT_FALSE(run.f.native.localCommitExchangeAt(run.prepare,100070000000LL).accepted);
  auto next=proposal();next.version.task_id="next_task";next.proposal_id="next_proposal";
  next.version.reference_generation=barrier.reference_generation+1;
  EXPECT_TRUE(run.f.native.replacePendingReference(next));EXPECT_TRUE(run.f.follower.proposal(next,100.08));
  EXPECT_FALSE(run.f.follower.commitResult(ready,100.08,10.08));
  EXPECT_FALSE(run.f.native.localCommitExchangeAt(run.prepare,100080000000LL).accepted);
}
TEST(LocalFollowerNative,RejectedFinalCommitRestoresIncumbentInsteadOfStrandingDifferentCurves) {
  ReservedCommit run;run.reserve();const auto commit=run.f.follower.exchangeCommit(100.07,10.07);
  ASSERT_TRUE(commit);ASSERT_EQ(run.f.follower.controller().trajectoryId(),1);
  Access::forgetOriginalProofs(run.f.native);
  const auto denied=run.f.native.localCommitExchangeAt(*commit,100075000000LL);
  ASSERT_FALSE(denied.accepted);ASSERT_TRUE(run.f.follower.commitResult(denied,100.076,10.076));
  EXPECT_FALSE(Access::checkReserved(run.f.native,run.f.map,100077000000LL));
  EXPECT_EQ(run.f.follower.controller().trajectoryId(),1);EXPECT_EQ(run.f.native.committedTrajectory(),1);
  EXPECT_TRUE(run.f.command(100.08,0.,0.,2));
}
TEST(LocalFollowerNative,RejectedCommitCannotRollBackNewerOdometryFaultOrSourceBarrier) {
  ReservedCommit run;run.reserve();const auto commit=run.f.follower.exchangeCommit(100.07,10.07);
  ASSERT_TRUE(commit);ASSERT_EQ(run.f.follower.controller().trajectoryId(),1);
  auto invalid=body(100.072,0.,0.);invalid.provider_usable=false;
  ASSERT_FALSE(run.f.follower.body(invalid,100.072,10.072));
  ASSERT_TRUE(run.f.follower.controller().holding());
  Access::forgetOriginalProofs(run.f.native);
  const auto denied=run.f.native.localCommitExchangeAt(*commit,100075000000LL);
  ASSERT_FALSE(denied.accepted);ASSERT_TRUE(run.f.follower.commitResult(denied,100.076,10.076));
  EXPECT_EQ(run.f.follower.controller().trajectoryId(),1);
  EXPECT_TRUE(run.f.follower.controller().holding());
  // The rejected observation consumed source ordering, although it did not
  // replace the last usable pose. Neither rollback nor a delayed valid packet
  // may erase that barrier or count as a new recovery sample.
  EXPECT_FALSE(run.f.follower.body(body(100.071,0.,0.),100.077,10.077));
  const auto old=run.f.follower.controller().odometry();
  EXPECT_FALSE(run.f.follower.body(old,100.078,10.078));
  ASSERT_TRUE(run.f.follower.body(body(100.08,0.,0.),100.08,10.08));
  EXPECT_TRUE(run.f.follower.controller().holding());
  const auto d=run.f.follower.step(100.08,10.08);
  EXPECT_TRUE(d.hold);EXPECT_DOUBLE_EQ(d.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(d.velocity.angular.z,0.);
}
TEST(LocalFollowerNative,RejectedCommitPreservesLatestOutputClockAndLimiterHistory) {
  ReservedCommit run;run.reserve();const auto commit=run.f.follower.exchangeCommit(100.07,10.07);
  ASSERT_TRUE(commit);
  run.f.acquire(100.072,.001,.03);
  run.f.follower.controller().recordAppliedOutput(.08,0.,100.072,10.072);
  auto expected=run.f.follower.controller();
  Access::forgetOriginalProofs(run.f.native);
  const auto denied=run.f.native.localCommitExchangeAt(*commit,100075000000LL);
  ASSERT_FALSE(denied.accepted);ASSERT_TRUE(run.f.follower.commitResult(denied,100.076,10.076));
  EXPECT_DOUBLE_EQ(run.f.follower.controller().odometry().stamp,100.072);
  ASSERT_TRUE(expected.receiveOdom(body(100.08,.0012,.03),100.08,10.08));
  ASSERT_TRUE(run.f.follower.body(body(100.08,.0012,.03),100.08,10.08));
  const auto baseline=expected.step(100.08,10.08,true);
  const auto actual=run.f.follower.controller().step(100.08,10.08,true);
  EXPECT_NEAR(actual.forward,baseline.forward,1e-9);
  EXPECT_NEAR(actual.yaw_rate,baseline.yaw_rate,1e-9);
  EXPECT_GT(actual.forward,.07);
}
TEST(LocalFollowerNative,FinalCommitReplyCannotUndoCancellationOrLocalizationReset) {
  for(const bool localization_reset:{false,true}) {
    ReservedCommit run;run.reserve();const auto commit=run.f.follower.exchangeCommit(100.07,10.07);
    ASSERT_TRUE(commit);
    if(localization_reset) {
      auto reset=body(100.072,0.,0.);++reset.localization_epoch;reset.localization_seed_id="new_seed";
      EXPECT_FALSE(run.f.follower.body(reset,100.072,10.072));
    } else run.f.follower.retire("cancelled");
    auto barrier=version();++barrier.reference_generation;ASSERT_TRUE(run.f.native.cancelReference(barrier));
    const auto reply=run.f.native.localCommitExchangeAt(*commit,100075000000LL);
    EXPECT_FALSE(reply.accepted);EXPECT_FALSE(run.f.follower.commitResult(reply,100.076,10.076));
    EXPECT_FALSE(run.f.follower.controller().active());
    EXPECT_FALSE(run.f.follower.exchangeCommit(100.08,10.08));
    EXPECT_TRUE(run.f.follower.step(100.08,10.08).hold);
  }
}
TEST(LocalFollowerNative,FirstAdmissionRollbackKeepsTaskAndRecoveryWithoutGeometry) {
  Integrated run;auto& core=run.follower.controller();const auto identity=core.task().identity;
  t::TrackerCore no_geometry(config());
  auto invalid=body(100.01,0.,0.);invalid.provider_usable=false;
  ASSERT_FALSE(run.follower.body(invalid,100.01,10.01));ASSERT_TRUE(core.holding());
  ASSERT_TRUE(core.rollbackGeometry(no_geometry,100.02));
  EXPECT_TRUE(core.active());EXPECT_TRUE(core.task().identity==identity);
  EXPECT_FALSE(core.hasInstalledGeometry());EXPECT_TRUE(core.holding());
  ASSERT_TRUE(run.follower.body(body(100.03,0.,0.),100.03,10.03));
  EXPECT_TRUE(core.holding());
}
TEST(LocalFollowerNative,ReservedFirstNonzeroDemandSurvivesCandidateReplacementAndOriginalCommitDelivery) {
  ReservedCommit run;run.reserve();const auto commit=run.f.follower.exchangeCommit(100.07,10.07);ASSERT_TRUE(commit);
  auto newer=tag(3);run.f.native.candidate(newer,run.f.reference,run.f.heading);
  ASSERT_TRUE(run.f.native.commit(run.f.follower.geometryPermit(100.07)));
  const auto d=run.f.follower.step(100.07,10.07);ASSERT_FALSE(d.hold);
  unsigned emitted=0;run.f.native.demandAt(d,100070000000LL,[&](const auto&){++emitted;});
  EXPECT_EQ(emitted,1U);EXPECT_EQ(d.trajectory_id,1);ASSERT_FALSE(Access::pendingDemand(run.f.native));
  const auto accepted=run.f.native.localCommitExchangeAt(*commit,100075000000LL,[&](const auto& c){
    ++emitted;EXPECT_EQ(c.sequence,d.sequence);EXPECT_EQ(c.source_stamp,d.source_stamp);
    EXPECT_EQ(c.body_source_stamp,d.body_source_stamp);EXPECT_FALSE(c.hold);
  });
  EXPECT_TRUE(accepted.accepted);EXPECT_EQ(emitted,1U);
  ASSERT_TRUE(run.f.follower.commitResult(accepted,100.076,10.076));
  EXPECT_EQ(run.f.follower.controller().trajectoryId(),2);
}
TEST(LocalFollowerNative,MovingSpliceReservesEarlyAndKeepsCommandContinuousAcrossDelayedAcks) {
  Integrated f;double x=0.,vx=0.,previous=0.;bool prepared=false,committed=false;
  std::optional<w::LocalTrajectoryCommit> delivery;
  unsigned nonzero=0;
  for(unsigned i=1;i<=50;++i) {
    const double now=100.+i*.05;const auto ns=t::timeNs(t::stamp(now));
    f.acquire(now,x,vx);f.native.progressAt(f.progress(now),now);
    auto active=Access::checkCurve(f.native,f.map,ns,true);ASSERT_TRUE(active&&active->valid);
    ASSERT_TRUE(f.follower.validation(*active));
    if(delivery){f.follower.commitResult(*delivery,now,now-90.);delivery.reset();}
    if(i==10){ASSERT_TRUE(f.prepareTail(now));prepared=true;}
    if(prepared&&!committed) {
      auto pending=Access::checkReserved(f.native,f.map,ns);
      if(!pending)pending=Access::checkCurve(f.native,f.map,ns);
      ASSERT_TRUE(pending&&pending->valid);f.follower.validation(*pending);
      const auto event=f.follower.exchangeCommit(now,now-90.);
      if(event) {
        delivery=f.native.localCommitExchangeAt(*event,ns);
        ASSERT_TRUE(delivery->accepted)<<delivery->reason;
        if(event->disposition=="committed")committed=true;
      }
    }
    auto out=f.command(now,x,vx,i,true,false);ASSERT_TRUE(out);
    EXPECT_LE(std::abs(out->velocity.linear.x-previous),.35*.05+1e-8);
    if(out->velocity.linear.x>0.)++nonzero;
    previous=vx=out->velocity.linear.x;x+=vx*.05;
  }
  EXPECT_TRUE(committed)<<f.follower.candidateReason();EXPECT_GT(nonzero,40U);
  EXPECT_EQ(f.native.committedTrajectory(),2);EXPECT_EQ(f.follower.controller().trajectoryId(),2);
}
TEST(LocalFollowerNative,FirstCurveDemandBeforeAnyNativeCommitWaitsForItsActualCommit) {
  Integrated f(true,false,{},{},true);ASSERT_TRUE(f.initial_commit);
  ASSERT_EQ(f.native.committedTrajectory(),-1);f.acquire(100.02,0.,0.);
  const auto grant=lease(100.02,1);ASSERT_TRUE(f.follower.lease(grant,100.02));
  ASSERT_TRUE(f.native.taskLeaseAt(grant,100020000000LL));ASSERT_TRUE(f.native.commit(f.follower.geometryPermit(100.02)));
  const auto d=f.follower.step(100.02,10.02);ASSERT_FALSE(d.hold)<<d.reason;
  unsigned emitted=0;f.native.demandAt(d,100020000000LL,[&](const auto&){++emitted;});
  ASSERT_TRUE(Access::pendingDemand(f.native));EXPECT_EQ(emitted,0U);
  ASSERT_TRUE(f.native.localCommitAt(*f.initial_commit,100025000000LL,[&](const auto& command){
    ++emitted;EXPECT_EQ(command.source_stamp,d.source_stamp);EXPECT_EQ(command.body_source_stamp,d.body_source_stamp);
    EXPECT_EQ(command.trajectory_id,1);EXPECT_FALSE(command.hold);
  }));
  EXPECT_EQ(emitted,1U);EXPECT_FALSE(Access::pendingDemand(f.native));
}
TEST(LocalFollowerNative,KnownCandidateZeroAfterNegativeProofRetiresOlderBodyPendingCommand) {
  for(const bool expired:{false,true}) {
    OrderedCommand run;auto old=run.next(100.05,1);unsigned nonzero=0;
    const auto count=[&](const auto& c){if(!c.hold)++nonzero;};
    run.f.native.demandAt(old,100050000000LL,count);ASSERT_TRUE(Access::pendingDemand(run.f.native));
    auto replacement=tag(2);replacement.trajectory.start_time=replacement.join_source_stamp=t::stamp(100.05);
    run.f.native.candidate(replacement,run.f.reference,run.f.heading);
    const auto proof=Access::checkCurve(run.f.native,run.f.map,100052000000LL);
    ASSERT_TRUE(proof&&proof->valid);
    if(expired) {
      // Only the candidate expires; the old current curve still has a valid
      // original proof, so its stale pending nonzero would otherwise relay.
      Access::expireCandidateProof(run.f.native,100053000000LL);
    } else {
      GridMapTestAccess::voxel(run.f.map,{.3,0.,.55},2.);
      const auto negative=Access::checkCurve(run.f.native,run.f.map,100053000000LL);
      ASSERT_TRUE(negative&&!negative->valid);
    }
    auto zero=old;zero.trajectory_id=2;++zero.sequence;zero.velocity={};zero.hold=true;
    zero.source_stamp=t::stamp(100.054);zero.validation_sequence=proof->sequence;
    run.f.native.demandAt(zero,100054000000LL,count);
    EXPECT_EQ(Access::latestDemand(run.f.native)->sequence,zero.sequence);
    run.deliverBody(100.05,100060000000LL,count);
    EXPECT_EQ(nonzero,0U)<<expired;EXPECT_EQ(run.f.native.committedTrajectory(),1);
  }
}
TEST(LocalFollowerNative,DeferredCandidateCommitCannotSurviveExpiryRevokeWrongIdentityOrNewerZero) {
  for(unsigned fault=0;fault<5;++fault) {
    DelayedCommit run;unsigned nonzero=0;
    const auto count=[&](const auto& c){if(!c.hold)++nonzero;};
    run.f.native.demandAt(run.demand,100060000000LL,count);ASSERT_TRUE(Access::pendingDemand(run.f.native));
    auto commit=run.commit;std::int64_t receipt=100075000000LL;
    if(fault==0)receipt=100150000000LL; // original body lease ends, despite fresh commit callback
    if(fault==1) {auto denied=lease(100.065,2);denied.allowed=false;denied.revoked=true;
      ASSERT_TRUE(run.f.native.taskLeaseAt(denied,100065000000LL));}
    if(fault==2)++commit.trajectory_id;
    if(fault==3) {auto zero=run.demand;++zero.sequence;zero.velocity={};zero.hold=true;
      run.f.native.demandAt(zero,100065000000LL,count);
      run.f.native.demandAt(run.demand,100066000000LL,count);}
    if(fault==4) {auto other=run.demand;++other.sequence;++other.trajectory_id;
      run.f.native.demandAt(other,100065000000LL,count);
      // Unknown geometry cannot poison the valid known candidate high-water.
      EXPECT_EQ(Access::latestDemand(run.f.native)->sequence,run.demand.sequence);}
    const bool accepted=run.f.native.localCommitAt(commit,receipt,count);
    EXPECT_EQ(accepted,fault!=1&&fault!=2)<<fault;
    EXPECT_EQ(nonzero,fault==4?1U:0U)<<fault;
  }
}
TEST(LocalFollowerNative,LeaseHoldAndRevocationCannotReviveHistoricalAllowedLease) {
  for(const bool revoked:{false,true}) {
    Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
    const auto original=*Access::latestDemand(f.native);auto demand=original;auto denied=lease(100.05,2);
    denied.allowed=false;denied.revoked=revoked;denied.phase="holding";
    ASSERT_TRUE(f.native.taskLeaseAt(denied,100050000000LL));
    ++demand.sequence;unsigned emitted=0;
    f.native.demandAt(demand,100050000000LL,[&](const auto&){++emitted;});
    EXPECT_EQ(emitted,0U);EXPECT_EQ(Access::motionProofCount(f.native),0U);
    if(!revoked) {
      // Resume can share the original source time. The held lease cleared
      // causal grant history, but must retain the received demand high-water.
      ASSERT_TRUE(f.native.taskLeaseAt(lease(100.05,3),100050000000LL));
      f.native.demandAt(original,100050000000LL,[&](const auto&){++emitted;});
      f.native.demandAt(demand,100050000000LL,[&](const auto&){++emitted;});
      EXPECT_EQ(emitted,0U);
      ++demand.sequence;
      f.native.demandAt(demand,100050000000LL,[&](const auto& command){
        ++emitted;EXPECT_EQ(command.lease_sequence,3U);EXPECT_EQ(command.source_stamp,original.source_stamp);
        EXPECT_EQ(command.body_source_stamp,original.body_source_stamp);
        EXPECT_LE(t::timeNs(command.valid_until),t::timeNs(original.body_source_stamp)+100000000LL);});
      EXPECT_EQ(emitted,1U);
      ++demand.sequence;f.native.demandAt(demand,100151000000LL,[&](const auto&){++emitted;});
      EXPECT_EQ(emitted,1U); // a higher sequence cannot renew the original body lease
    }
  }
}
TEST(LocalFollowerNative,OriginalBodyExpiryAbsentProofWrongIdentityAndOverspeedNeverRelay) {
  for(unsigned kind=0;kind<10;++kind) {
    Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
    auto demand=*Access::latestDemand(f.native);++demand.sequence;
    std::int64_t now=100050000000LL;
    if(kind==0){f.acquire(100.151,0.,0.);demand.source_stamp=t::stamp(100.151);
      demand.valid_until=t::stamp(100.251);now=100151000000LL;} // original body remains 100.05
    if(kind==1)Access::removeActiveProof(f.native);
    if(kind==2)++demand.trajectory_id;
    if(kind==3)++demand.version.anchor_revision;
    if(kind==4)++demand.sdk_arm_generation;
    if(kind==5)demand.velocity.linear.x=.31;
    if(kind==6)demand.velocity.angular.z=.51;
    if(kind==7)demand.velocity.linear.y=std::numeric_limits<double>::quiet_NaN();
    if(kind==8)demand.braking_model_sha256=std::string(64,'e');
    if(kind==9)demand.hold=true; // contradictory HOLD must never relay nonzero
    unsigned emitted=0;f.native.demandAt(demand,now,[&](const auto&){++emitted;});
    EXPECT_EQ(emitted,0U)<<kind;EXPECT_EQ(Access::motionProofCount(f.native),0U)<<kind;
  }
}
TEST(LocalFollowerNative,MeasuredResponseAboveRequestedCapsDoesNotDropBoundedCommand) {
  for(unsigned scenario=0;scenario<3;++scenario) {
    Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
    geometry_msgs::msg::Twist measured;
    if(scenario!=1)measured.linear.x=.337;
    if(scenario!=0)measured.angular.z=-.701;
    f.native.body({{0.,0.,.55},Eigen::Quaterniond::Identity(),100.06,"odom"},measured,100060000000LL);
    GridMapTestAccess::acquire(f.map,100060000000LL);
    const auto proof=Access::checkCurve(f.native,f.map,100060000000LL,true);ASSERT_TRUE(proof&&proof->valid);
    auto demand=*Access::latestDemand(f.native);++demand.sequence;
    demand.source_stamp=t::stamp(100.06);demand.velocity.linear.x=.284;demand.velocity.angular.z=-.047;
    unsigned emitted=0;f.native.demandAt(demand,100060000000LL,[&](const auto& c){
      ++emitted;EXPECT_EQ(c.velocity,demand.velocity);EXPECT_FALSE(c.hold);
      EXPECT_EQ(c.body_source_stamp,demand.body_source_stamp);
      EXPECT_LE(t::timeNs(c.valid_until),100150000000LL);
    });
    EXPECT_EQ(emitted,1U)<<scenario;
    // This does NOT bypass actual collision or increase requested caps.
    GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.);
    const auto negative=Access::checkCurve(f.native,f.map,100061000000LL,true);ASSERT_TRUE(negative&&!negative->valid);
    ++demand.sequence;f.native.demandAt(demand,100061000000LL,[&](const auto&){++emitted;});
    EXPECT_EQ(emitted,1U);
  }
}
TEST(LocalFollowerNative,NonfiniteIndependentMeasuredResponseStillCannotRelay) {
  for(unsigned axis=0;axis<6;++axis) {
    Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
    auto demand=*Access::latestDemand(f.native);++demand.sequence;
    geometry_msgs::msg::Twist corrupt;
    double* values[]={&corrupt.linear.x,&corrupt.linear.y,&corrupt.linear.z,
      &corrupt.angular.x,&corrupt.angular.y,&corrupt.angular.z};
    *values[axis]=std::numeric_limits<double>::quiet_NaN();Access::corruptLatestTwist(f.native,corrupt);
    unsigned emitted=0;f.native.demandAt(demand,100050000000LL,[&](const auto&){++emitted;});
    EXPECT_EQ(emitted,0U)<<axis;
  }
}
TEST(LocalFollowerNative,CancelRejectsLateCopiedCurveAndLateCommandWithoutMotionEvidence) {
  Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
  auto demand=*Access::latestDemand(f.native);++demand.sequence;
  EXPECT_FALSE(Access::canceledCopiedCurve(f.native,f.map,100050000000LL));
  unsigned emitted=0;f.native.demandAt(demand,100050000000LL,[&](const auto&){++emitted;});
  EXPECT_EQ(emitted,0U);EXPECT_EQ(f.native.committedTrajectory(),-1);
  EXPECT_EQ(Access::motionProofCount(f.native),0U);
}
TEST(LocalFollowerNative,GoalMetadataComesOnlyFromSameTaskCommittedTypedProof) {
 auto p=proposal();p.goal_position.x=.4;Integrated f(true,false,p);
 const auto goal=f.native.committedTaskGoal(version());ASSERT_TRUE(goal);EXPECT_NEAR(goal->x(),.4,1e-12);
 auto newer=version();++newer.reference_generation;
 ASSERT_TRUE(f.native.committedTaskGoal(newer));
 auto foreign=version();foreign.task_id="other";EXPECT_FALSE(f.native.committedTaskGoal(foreign));
 auto queued=p;queued.goal_position.x=9.;queued.proposal_id="queued";
 f.native.candidate(tag(2),queued,f.heading);
 ASSERT_TRUE(f.native.committedTaskGoal(version()));EXPECT_NEAR(f.native.committedTaskGoal(version())->x(),.4,1e-12);
 auto retired=version();++retired.reference_generation;ASSERT_TRUE(f.native.cancelReference(retired));
 EXPECT_FALSE(f.native.committedTaskGoal(version()));
}
TEST(LocalFollowerNative,NativeGoalSettlingUsesFreshActualTwistAndFailsClosedOnMissingOrMalformedState) {
 Integrated f;EXPECT_TRUE(f.native.measuredGoalSettled(version(),100000000000LL));
 EXPECT_FALSE(f.native.measuredGoalSettled(version(),100101000000LL));
 f.acquire(100.05,0.,.031);EXPECT_FALSE(f.native.measuredGoalSettled(version(),100050000000LL));
 const scan_planner::MeasuredBodyPose measured{{0.,0.,.55},Eigen::Quaterniond::Identity(),100.06,"odom"};
 geometry_msgs::msg::Twist twist;twist.linear.x=.021;twist.linear.y=.022;
 f.native.body(measured,twist,100060000000LL);EXPECT_FALSE(f.native.measuredGoalSettled(version(),100060000000LL));
 auto next=measured;next.source_stamp=100.07;twist.linear.x=twist.linear.y=0.;twist.angular.z=-.051;
 f.native.body(next,twist,100070000000LL);EXPECT_FALSE(f.native.measuredGoalSettled(version(),100070000000LL));
 next.source_stamp=100.08;f.native.body(next,{},100080000000LL);
 EXPECT_FALSE(f.native.measuredGoalSettled(version(),100080000000LL));
 next.source_stamp=100.09;twist.linear.x=.03;twist.angular.z=.05;
 f.native.body(next,twist,100090000000LL);
 const auto current=Access::checkCurve(f.native,f.map,100090000000LL,true);ASSERT_TRUE(current&&current->valid);
 EXPECT_TRUE(f.native.measuredGoalSettled(version(),100090000000LL));
 auto other=version();other.task_id="other";EXPECT_FALSE(f.native.measuredGoalSettled(other,100090000000LL));
 for(unsigned field=0;field<6;++field)for(double malformed:{std::numeric_limits<double>::quiet_NaN(),
     std::numeric_limits<double>::infinity()}) {
   geometry_msgs::msg::Twist corrupt;
   double* values[]={&corrupt.linear.x,&corrupt.linear.y,&corrupt.linear.z,
     &corrupt.angular.x,&corrupt.angular.y,&corrupt.angular.z};*values[field]=malformed;
   Access::corruptLatestTwist(f.native,corrupt);
   EXPECT_FALSE(f.native.measuredGoalSettled(version(),100090000000LL))<<field;
 }
 Access::corruptLatestTwist(f.native,geometry_msgs::msg::Twist());Access::corruptLatestPosition(f.native);
 EXPECT_FALSE(f.native.measuredGoalSettled(version(),100090000000LL));
}
TEST(LocalFollowerNative,NativeGoalSettlingKeepsOriginalCommittedGoalYawTolerance) {
 auto p=proposal();p.has_goal_yaw=true;p.goal_yaw=.25;p.goal_yaw_tolerance_rad=.05;Integrated f(true,false,p);
 EXPECT_FALSE(f.native.measuredGoalSettled(version(),100000000000LL));
 geometry_msgs::msg::Twist stopped;
 scan_planner::MeasuredBodyPose measured{{0.,0.,.55},Eigen::Quaterniond(Eigen::AngleAxisd(.21,Eigen::Vector3d::UnitZ())),100.05,"odom"};
 f.native.body(measured,stopped,100050000000LL);
 auto current=Access::checkCurve(f.native,f.map,100050000000LL,true);ASSERT_TRUE(current&&current->valid);
 EXPECT_TRUE(f.native.measuredGoalSettled(version(),100050000000LL));
 measured.source_stamp=100.06;measured.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(.19,Eigen::Vector3d::UnitZ()));
 f.native.body(measured,stopped,100060000000LL);
 current=Access::checkCurve(f.native,f.map,100060000000LL,true);ASSERT_TRUE(current&&current->valid);
 EXPECT_FALSE(f.native.measuredGoalSettled(version(),100060000000LL));
 auto queued=p;queued.goal_yaw_tolerance_rad=.5;f.native.candidate(tag(2),queued,f.heading);
 EXPECT_FALSE(f.native.measuredGoalSettled(version(),100060000000LL));
 p.goal_yaw_tolerance_rad=std::numeric_limits<double>::quiet_NaN();Integrated malformed(true,false,p);
 measured.source_stamp=100.05;measured.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(.25,Eigen::Vector3d::UnitZ()));
 malformed.native.body(measured,stopped,100050000000LL);
 current=Access::checkCurve(malformed.native,malformed.map,100050000000LL,true);ASSERT_TRUE(current&&current->valid);
 EXPECT_FALSE(malformed.native.measuredGoalSettled(version(),100050000000LL));
}
TEST(LocalFollowerNative,FinalEndpointReconnectUsesCommittedGoalAndNewWholeProofFromRealStoppedBody) {
 auto p=proposal();p.goal_position.x=.4;Integrated f(true,false,p);f.acquire(100.05,.7,0.);
 auto old=Access::checkCurve(f.native,f.map,100050000000LL,true);
 ASSERT_TRUE(old);EXPECT_FALSE(old->valid);ASSERT_TRUE(f.follower.validation(*old));
 const auto goal=f.native.committedTaskGoal(version());const auto ground=f.native.planningSupport(version());
 ASSERT_TRUE(goal);ASSERT_TRUE(ground);ASSERT_TRUE(ground->nearest(.7,0.));ASSERT_TRUE(ground->nearest(goal->x(),goal->y()));
 scan_planner::DiscreteReference reference;reference.set({{0.,0.,.55},*goal});
 const Eigen::Vector3d actual{.7,0.,.55};const double progress=reference.project(actual,0.,reference.length());
 ASSERT_NEAR(progress,reference.length(),1e-12);
 scan_planner::ReferenceTargetOptions options;
 options.final_endpoint_reseed=scan_planner::FinalEndpointReseed{*goal,.2,.15,true};
 const auto selected=scan_planner::selectReferenceTarget(reference,actual,progress,2.,options,
   [&](const Eigen::Vector3d& point,double yaw){return f.map.getPlanningInflateOccupancy(point,yaw);});
 ASSERT_TRUE(selected.valid);EXPECT_TRUE(selected.point.isApprox(*goal));
 const auto connector=reference.slice(progress,selected.arc,actual);
 ASSERT_EQ(connector.size(),2U);EXPECT_TRUE(connector.front().isApprox(actual));EXPECT_TRUE(connector.back().isApprox(*goal));
 auto curve=tag(2,actual,Eigen::Vector3d::Zero(),{-.0375,0.,0.});
 curve.join_source_stamp=curve.trajectory.start_time=t::stamp(100.05);
 f.native.candidate(curve,f.reference,f.heading);ASSERT_TRUE(f.follower.candidate(curve));
 auto proof=Access::checkCurve(f.native,f.map,100050000000LL);ASSERT_TRUE(proof);ASSERT_TRUE(proof->valid)<<proof->reason;
 ASSERT_TRUE(f.follower.validation(*proof));const auto commit=f.follower.tryCommit(100.05,10.05);
 ASSERT_TRUE(commit)<<f.follower.candidateReason();ASSERT_TRUE(f.native.localCommitAt(*commit,100050000000LL));
 EXPECT_EQ(f.native.committedTrajectory(),2);EXPECT_EQ(f.follower.commitSequence(),2U);
 double x=.7,v=0.;bool reached=false,returned=false;
 for(unsigned i=1;i<=250;++i) {
   const double now=100.05+i*.05;const auto out=f.command(now,x,v,i);ASSERT_TRUE(out);
   returned|=out->velocity.linear.x<0.;v=out->velocity.linear.x;x+=v*.05;
   if(f.follower.status(now).at_goal){reached=true;break;}
 }
 EXPECT_TRUE(returned);EXPECT_TRUE(reached);EXPECT_LT(x,.7);EXPECT_EQ(f.follower.activeVersion()->task_id,"task");
}
TEST(LocalFollowerNative,BrakingBoundaryReplanPreservesTaskAndFreshStoppedCurveResumes) {
 Integrated f(true,false,{},tag(1,{0.,0.,.55},{.25,0.,0.},Eigen::Vector3d::Zero()));
 double now=100.,x=0.;unsigned sample=0;bool blocked=false;
 for(unsigned i=1;i<=85;++i) {
   now=100.+i*.05;x=i*.0125;sample=i;f.command(now,x,.25,i);
   if(f.follower.controller().reason()=="braking_envelope_reentry_required"){blocked=true;break;}
  }
  ASSERT_TRUE(blocked)<<f.follower.controller().reason();
 // This fixture supplies progress before and after control; Native keeps only
 // one fact per original body source. A later real sample carries the newly
 // acquired braking wait, without re-stamping the previous observation.
 now+=.05;x+=.0125;f.command(now,x,.25,++sample);
  const auto waiting=f.follower.status(now);EXPECT_TRUE(waiting.holding);EXPECT_FALSE(waiting.request_stop);
 EXPECT_EQ(waiting.phase,"waiting_replan");const auto reseed=f.native.consumeFormalReseedAt(t::timeNs(t::stamp(now)));
 ASSERT_TRUE(reseed);EXPECT_TRUE(reseed->terminal);EXPECT_EQ(reseed->version.task_id,"task");
 EXPECT_EQ(reseed->reason,"measured_tracking_braking_boundary_unreachable");
 EXPECT_FALSE(f.native.consumeFormalReseedAt(t::timeNs(t::stamp(now))));
 now+=.05;f.acquire(now,x,0.);auto stopped_current=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(now)),true);
 ASSERT_TRUE(stopped_current);ASSERT_TRUE(stopped_current->valid);ASSERT_TRUE(f.follower.validation(*stopped_current));
 ASSERT_TRUE(f.follower.lease(lease(now,++sample),now));f.follower.geometryPermit(now);
 const auto hold=f.follower.step(now,now-90.);EXPECT_TRUE(hold.hold);EXPECT_EQ(hold.reason,"braking_envelope_reentry_required");
 auto fresh=tag(2,{x,0.,.55});fresh.join_source_stamp=fresh.trajectory.start_time=t::stamp(now);
 f.native.candidate(fresh,f.reference,f.heading);ASSERT_TRUE(f.follower.candidate(fresh));
 auto proof=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(now)));ASSERT_TRUE(proof);ASSERT_TRUE(proof->valid)<<proof->reason;
 ASSERT_TRUE(f.follower.validation(*proof));const auto commit=f.follower.tryCommit(now,now-90.);
 ASSERT_TRUE(commit)<<f.follower.candidateReason();ASSERT_TRUE(f.native.localCommitAt(*commit,t::timeNs(t::stamp(now))));
 const auto resumed=f.command(now+.05,x,0.,++sample);ASSERT_TRUE(resumed);EXPECT_GT(resumed->velocity.linear.x,0.);
 EXPECT_FALSE(f.follower.status(now+.05).software_retired);EXPECT_EQ(f.native.committedTrajectory(),2);
 EXPECT_EQ(f.follower.activeVersion()->task_id,"task");
}
TEST(LocalFollowerNative,ConnectorOutsideNativeDomainWithinGoalToleranceNeedsShortTypedScanReplan) {
 for(double distance:{.16,.20}) {
   auto p=proposal();p.goal_position.x=.4;Integrated f(true,false,p);
   const Eigen::Vector3d actual{distance==.20?.6:.56,0.,.55};double now=100.;
   // Approach through real, bounded acquired progress; teleporting from the
   // initial join beyond the forward projection window is a different fault.
   unsigned sample=0;
   for(double x=.012;x<actual.x()-1e-9;x+=.012) {
     now=100.+(++sample)*.05;f.command(now,x,.24,sample,x-.4<=.15);
   }
   for(unsigned i=1;i<=2;++i) {
     now=100.+(++sample)*.05;const auto held=f.command(now,actual.x(),0.,sample,false);
     if(held){EXPECT_TRUE(held->hold);EXPECT_DOUBLE_EQ(held->velocity.linear.x,0.);EXPECT_DOUBLE_EQ(held->velocity.linear.y,0.);}
   }
   EXPECT_FALSE(f.native.measuredGoalSettled(version(),t::timeNs(t::stamp(now))));
   EXPECT_FALSE(f.follower.status(now).request_stop);
   const auto reseed=f.native.consumeFormalReseedAt(t::timeNs(t::stamp(now)));ASSERT_TRUE(reseed);EXPECT_TRUE(reseed->terminal);
   ASSERT_TRUE(f.native.finalEndpointConnectionNeedsReseed(version(),t::timeNs(t::stamp(now))));
   const auto goal=f.native.committedTaskGoal(version());ASSERT_TRUE(goal);
   const auto ground=f.native.planningSupport(version());ASSERT_TRUE(ground);ASSERT_TRUE(ground->nearest(actual.x(),actual.y()));
   scan_planner::DiscreteReference reference;reference.set({{0.,0.,.55},*goal});
   const double progress=reference.project(actual,0.,reference.length());ASSERT_NEAR(progress,reference.length(),1e-12);
   scan_planner::ReferenceTargetOptions options;
   options.final_endpoint_reseed=scan_planner::FinalEndpointReseed{*goal,.2,.15,true,true,
     scan_planner::NoMotionPreviewConnectionPolicy::max_connector_m};
   const auto selected=scan_planner::selectReferenceTarget(reference,actual,progress,2.,options,
     [&](const Eigen::Vector3d& point,double yaw){return f.map.getPlanningInflateOccupancy(point,yaw);});
   ASSERT_TRUE(selected.valid);EXPECT_EQ(selected.reason,"reference_target_final_endpoint_reseed");
   const auto connector=reference.slice(progress,selected.arc,actual);ASSERT_EQ(connector.size(),2U);
   EXPECT_TRUE(scan_planner::finalEndpointShortConnectionAllowed(true,connector,actual,selected.point,false));
   auto curve=tag(2,actual,Eigen::Vector3d::Zero(),{-((actual-*goal).x())/8.,0.,0.});
   curve.join_source_stamp=curve.trajectory.start_time=t::stamp(now);
   f.native.candidate(curve,f.reference,f.heading);ASSERT_TRUE(f.follower.candidate(curve));
   const auto proof=Access::checkCurve(f.native,f.map,t::timeNs(t::stamp(now)));ASSERT_TRUE(proof&&proof->valid);
   ASSERT_TRUE(f.follower.validation(*proof));const auto commit=f.follower.tryCommit(now,now-90.);
   ASSERT_TRUE(commit)<<f.follower.candidateReason();ASSERT_TRUE(f.native.localCommitAt(*commit,t::timeNs(t::stamp(now))));
   EXPECT_TRUE(f.native.measuredGoalSettled(version(),t::timeNs(t::stamp(now))));
   EXPECT_TRUE(f.follower.status(now).at_goal);EXPECT_TRUE(f.follower.status(now).request_stop);
   EXPECT_EQ(f.follower.activeVersion()->task_id,"task");EXPECT_EQ(f.native.committedTrajectory(),2);
 }
}
TEST(LocalFollowerNative,FinalConnectionReseedUsesCausalRecentEvidenceAcrossFasterActualBodyUpdates) {
 auto p=proposal();p.goal_position.x=.4;Integrated f(true,false,p);
 f.acquire(100.05,.57,0.);const auto negative=Access::checkCurve(f.native,f.map,100050000000LL,true);
 ASSERT_TRUE(negative);ASSERT_FALSE(negative->valid);ASSERT_EQ(negative->reason,"measured_projection_or_connection_invalid");
 ASSERT_TRUE(f.native.finalEndpointConnectionNeedsReseed(version(),100050000000LL));
 f.acquire(100.06,.57,0.);EXPECT_TRUE(f.native.finalEndpointConnectionNeedsReseed(version(),100060000000LL));
 auto foreign=version();foreign.task_id="other";
 EXPECT_FALSE(f.native.finalEndpointConnectionNeedsReseed(foreign,100060000000LL));
 f.acquire(100.50,.57,0.);EXPECT_FALSE(f.native.finalEndpointConnectionNeedsReseed(version(),100500000000LL));
}
TEST(LocalFollowerNative,CompletionUsesCausalPositiveProofAndCurrentQuietConnectedBodyAcrossRates) {
 Integrated f;f.acquire(100.10,0.,0.);const auto proof=Access::checkCurve(f.native,f.map,100100000000LL,true);
 ASSERT_TRUE(proof&&proof->valid);f.acquire(100.11,0.,0.);
 EXPECT_TRUE(f.native.measuredGoalSettled(version(),100110000000LL));
 auto foreign=version();foreign.task_id="other";EXPECT_FALSE(f.native.measuredGoalSettled(foreign,100110000000LL));
 f.acquire(100.12,0.,0.);
 geometry_msgs::msg::Twist stopped;
 f.native.body({{0.,.16,.55},Eigen::Quaterniond::Identity(),100.13,"odom"},stopped,100130000000LL);
 EXPECT_FALSE(f.native.measuredGoalSettled(version(),100130000000LL));
 f.acquire(100.50,0.,0.);EXPECT_TRUE(f.native.measuredGoalSettled(version(),100500000000LL));
}
TEST(LocalFollowerNative,MeasuredOffsetReseedsSameRouteInsteadOfReturningToOldFutureSplice) {
 Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
 ASSERT_TRUE(f.command(100.05,.005,.1,1));
 ASSERT_TRUE(f.native.plannedSpliceAt(version(),100050000000LL,.05));
 auto progress=f.progress(100.05);progress.header.stamp=t::stamp(100.10);
 progress.pose.position.y=.08;
 geometry_msgs::msg::Twist twist;twist.linear.x=.1;
 f.native.body({{.005,.08,.55},Eigen::Quaterniond::Identity(),100.10,"odom"},twist,100100000000LL);
 f.native.progressAt(progress,100.10);
 EXPECT_FALSE(f.native.plannedSpliceAt(version(),100100000000LL,.05));
 const auto event=f.native.consumeFormalReseedAt(100100000000LL,.05);
 ASSERT_TRUE(event);EXPECT_EQ(event->reason,"measured_body_left_local_curve");
 EXPECT_TRUE(event->actual_body);EXPECT_FALSE(event->terminal);
 EXPECT_EQ(event->version.route_hash,version().route_hash);
 EXPECT_FALSE(f.native.consumeFormalReseedAt(100110000000LL,.05));
 EXPECT_EQ(f.native.committedTrajectory(),1); // Scheduling never erases execution geometry.
}
TEST(LocalFollowerNative,CentimeterOffsetReseedsActualBodyInsteadOfHoldingUnjoinableSplice) {
 for(const double offset:{.02,.04}) {
  SCOPED_TRACE(offset);
  Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
  ASSERT_TRUE(f.command(100.05,.005,.1,1));ASSERT_TRUE(f.prepareTail(100.05));
  const auto pending=Access::pendingCurve(f.native);ASSERT_TRUE(pending);
  // Keep source/lease/collision updates at the actual 20 Hz control cadence.
  // Arrive just before the reserved seam with a measured parallel offset.
  double incumbent_speed=0.;
  for(unsigned i=2;i<=16;++i) {
    const double now=100.+i*.05;
    const auto output=f.command(now,.1*(now-100.),.1,i);
    ASSERT_TRUE(output);ASSERT_FALSE(output->hold)<<output->reason;
    incumbent_speed=output->velocity.linear.x;
  }
  constexpr double now=100.84;constexpr auto stamp=100840000000LL;
  auto odom=body(now,pending->splice.position.x-.001,.1);odom.position.y()=offset;
  ASSERT_TRUE(f.follower.body(odom,now,now-90.));
  geometry_msgs::msg::Twist twist;twist.linear.x=.1;
  const scan_planner::MeasuredBodyPose actual{odom.position,odom.orientation,now,"odom"};
  f.native.body(actual,twist,stamp);GridMapTestAccess::acquire(f.map,stamp);
  f.native.progressAt(f.progress(now),now);
  auto active_proof=Access::checkCurve(f.native,f.map,stamp,true);
  ASSERT_TRUE(active_proof&&active_proof->valid);ASSERT_TRUE(f.follower.validation(*active_proof));
  const auto tail_proof=Access::checkCurve(f.native,f.map,stamp);
  ASSERT_TRUE(tail_proof&&tail_proof->valid)<< (tail_proof?tail_proof->reason:"absent");
  ASSERT_TRUE(f.follower.validation(*tail_proof));
  EXPECT_FALSE(f.follower.tryCommit(now,now-90.));
  EXPECT_EQ(f.follower.candidateReason(),"entry_measured_body_not_on_candidate");
  const auto disposition=f.native.pendingSpliceDispositionAt(version(),stamp);
  EXPECT_FALSE(disposition.keep);EXPECT_STREQ(disposition.reason,"pending_splice_measured_deviation");
  EXPECT_FALSE(f.native.plannedSpliceAt(version(),stamp,.05));
  EXPECT_FALSE(f.native.plannedSpliceAtBody(version(),stamp,.05,actual,stamp));
  const auto event=f.native.consumeFormalReseedAt(stamp,.05);
  ASSERT_TRUE(event);EXPECT_TRUE(event->actual_body);EXPECT_FALSE(event->terminal);
  EXPECT_EQ(event->reason,"measured_body_left_local_curve");
  EXPECT_EQ(event->body_source_stamp,t::stamp(now));EXPECT_EQ(event->version.route_hash,version().route_hash);
  EXPECT_FALSE(f.native.consumeFormalReseedAt(stamp,.05));
  EXPECT_EQ(f.native.committedTrajectory(),1); // The scheduling event does not stop/replace it.
  // This is exactly the no-splice branch used by the production FSM: acquired
  // body position/velocity seed the next solve, not the old future curve point.
  auto replacement=tag(3,odom.position,odom.velocity_in_frame,Eigen::Vector3d::Zero());
  replacement.trajectory.start_time=replacement.join_source_stamp=t::stamp(now);
  f.native.candidate(replacement,f.reference,f.heading);ASSERT_TRUE(f.follower.candidate(replacement));
  auto replacement_proof=Access::checkCurve(f.native,f.map,stamp);
  ASSERT_TRUE(replacement_proof&&replacement_proof->valid);
  ASSERT_TRUE(f.follower.validation(*replacement_proof));
  const auto committed=f.follower.tryCommit(now,now-90.);
  ASSERT_TRUE(committed)<<f.follower.candidateReason();
  EXPECT_EQ(committed->expected_trajectory_id,1);EXPECT_EQ(committed->trajectory_id,3);
  ASSERT_TRUE(f.native.localCommitAt(*committed,stamp));
  EXPECT_EQ(f.native.committedTrajectory(),3);
  const auto output=f.command(now,odom.position.x(),.1,17,true,false);
  ASSERT_TRUE(output);EXPECT_FALSE(output->hold)<<output->reason;
  EXPECT_GT(output->velocity.linear.x,0.);
  EXPECT_LE(std::abs(output->velocity.linear.x-incumbent_speed),config().max_acceleration*.04+1e-8);
 }
}
TEST(LocalFollowerNative,JoinableMillimeterOffsetKeepsSpliceWithoutUnnecessaryReseed) {
 Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
 ASSERT_TRUE(f.command(100.05,.005,.1,1));ASSERT_TRUE(f.prepareTail(100.05));
 auto odom=body(100.10,.010,.1);odom.position.y()=.010;
 ASSERT_TRUE(f.follower.body(odom,100.10,10.10));
 geometry_msgs::msg::Twist twist;twist.linear.x=.1;
 const scan_planner::MeasuredBodyPose actual{odom.position,odom.orientation,100.10,"odom"};
 f.native.body(actual,twist,100100000000LL);GridMapTestAccess::acquire(f.map,100100000000LL);
 f.native.progressAt(f.progress(100.10),100.10);
 auto active=Access::checkCurve(f.native,f.map,100100000000LL,true);ASSERT_TRUE(active&&active->valid);
 const auto pending=Access::checkCurve(f.native,f.map,100100000000LL);ASSERT_TRUE(pending&&pending->valid);
 EXPECT_TRUE(f.native.plannedSpliceAt(version(),100100000000LL,.05));
 EXPECT_TRUE(f.native.plannedSpliceAtBody(version(),100100000000LL,.05,actual,100100000000LL));
 EXPECT_TRUE(f.native.pendingSpliceDispositionAt(version(),100100000000LL).keep);
 EXPECT_FALSE(f.native.consumeFormalReseedAt(100100000000LL,.05));
 EXPECT_EQ(f.native.committedTrajectory(),1);
}
TEST(LocalFollowerNative,ActualBodyReseedDoesNotAcceptOccupiedOrStaleBodyReplacement) {
 for(const bool occupied:{false,true}) {
  SCOPED_TRACE(occupied);
  Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
  ASSERT_TRUE(f.command(100.05,.005,.1,1));
  auto odom=body(100.10,.010,.1);odom.position.y()=.03;
  ASSERT_TRUE(f.follower.body(odom,100.10,10.10));
  geometry_msgs::msg::Twist twist;twist.linear.x=.1;
  f.native.body({odom.position,odom.orientation,100.10,"odom"},twist,100100000000LL);
  GridMapTestAccess::acquire(f.map,100100000000LL);f.native.progressAt(f.progress(100.10),100.10);
  const auto event=f.native.consumeFormalReseedAt(100100000000LL,.05);ASSERT_TRUE(event&&event->actual_body);
  auto candidate=tag(2,odom.position,odom.velocity_in_frame,Eigen::Vector3d::Zero());
  candidate.trajectory.start_time=candidate.join_source_stamp=t::stamp(100.10);
  f.native.candidate(candidate,f.reference,f.heading);ASSERT_TRUE(f.follower.candidate(candidate));
  if(occupied)GridMapTestAccess::voxel(f.map,{.25,.03,.55},2.);
  auto proof=Access::checkCurve(f.native,f.map,100100000000LL);ASSERT_TRUE(proof);
  EXPECT_EQ(proof->valid,!occupied);
  if(occupied)EXPECT_EQ(proof->reason,"occupied");
  // A completed positive state does not expire; an actual commit body still
  // must be delivered/fresh. Do not replace this with a fake proof lease.
  const double failed_at=occupied?100.10:100.501;
  ASSERT_TRUE(f.follower.validation(*proof));
  EXPECT_FALSE(f.follower.tryCommit(failed_at,failed_at-90.));
  EXPECT_EQ(f.native.committedTrajectory(),1);
  // The failed actual-entry attempt is retired by its owner. A positive
  // state no longer retires itself merely because a map lease elapsed.
  f.native.cancelPending();
  // The one-shot reseed notification is not a one-shot ability to replan.
  // A failed/expired candidate leaves the same incumbent. The next ordinary
  // solve must still select measured-body start state, not the unjoinable tail.
  const double retry_at=failed_at+.05;const auto retry_ns=t::timeNs(t::stamp(retry_at));
  odom=body(retry_at,.1*(retry_at-100.),.1);odom.position.y()=.03;
  ASSERT_TRUE(f.follower.body(odom,retry_at,retry_at-90.));
  const scan_planner::MeasuredBodyPose next_body{odom.position,odom.orientation,retry_at,"odom"};
  f.native.body(next_body,twist,retry_ns);
  if(occupied)GridMapTestAccess::voxel(f.map,{.25,.03,.55},-1.);
  GridMapTestAccess::acquire(f.map,retry_ns);f.native.progressAt(f.progress(retry_at),retry_at);
  const auto current=Access::checkCurve(f.native,f.map,retry_ns,true);
  if(current)ASSERT_TRUE(f.follower.validation(*current));
  EXPECT_FALSE(f.native.consumeFormalReseedAt(retry_ns,.05));
  EXPECT_FALSE(f.native.plannedSpliceAtBody(version(),retry_ns,.05,next_body,retry_ns));
  EXPECT_FALSE(f.native.pendingSpliceDispositionAt(version(),retry_ns).keep);
  const auto boundary=f.native.committedBoundary(odom.position,odom.velocity_in_frame,retry_ns,retry_at);
  // Waiting for the candidate's real expiry also expires the incumbent's old
  // proof. The formal FSM must still solve from acquired P/V with zero A,
  // rather than require a fresh incumbent boundary to produce its replacement.
  if(occupied) {ASSERT_TRUE(boundary);EXPECT_NEAR(boundary->velocity.x(),.1,1e-12);}
  else EXPECT_FALSE(boundary);
  const Eigen::Vector3d start_v=boundary?boundary->velocity:odom.velocity_in_frame;
  const Eigen::Vector3d start_a=boundary?boundary->acceleration:Eigen::Vector3d::Zero();
  auto retry=tag(3,odom.position,start_v,start_a);
  retry.trajectory.start_time=retry.join_source_stamp=t::stamp(retry_at);
  f.native.candidate(retry,f.reference,f.heading);ASSERT_TRUE(f.follower.candidate(retry));
  const auto recovered=Access::checkCurve(f.native,f.map,retry_ns);
  ASSERT_TRUE(recovered&&recovered->valid);ASSERT_TRUE(f.follower.validation(*recovered));
  const auto committed=f.follower.tryCommit(retry_at,retry_at-90.);ASSERT_TRUE(committed)<<f.follower.candidateReason();
  ASSERT_TRUE(f.native.localCommitAt(*committed,retry_ns));
  EXPECT_EQ(f.native.committedTrajectory(),3);
 }
}
TEST(LocalFollowerNative,DeviationSchedulingUsesIndependentSameSourceBodyNotClaimedProgressPose) {
 Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
 ASSERT_TRUE(f.command(100.05,.005,.1,1));
 auto progress=f.progress(100.05);progress.header.stamp=t::stamp(100.10);
 progress.pose.position.y=.3; // A projection message cannot invent sensor movement.
 f.acquire(100.10,.005,.1);f.native.progressAt(progress,100.10);
 EXPECT_TRUE(f.native.plannedSpliceAt(version(),100100000000LL,.05));
 EXPECT_FALSE(f.native.consumeFormalReseedAt(100100000000LL,.05));
 EXPECT_FALSE(f.native.plannedSpliceAt(version(),100600000000LL,.05));
 EXPECT_FALSE(f.native.consumeFormalReseedAt(100600000000LL,.05));
}
TEST(LocalFollowerNative,FastProgressAndOlderSolverBodyUseOriginalPairedSourceWithoutLoweringHighWater) {
 Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
 ASSERT_TRUE(f.command(100.05,.005,.1,1));
 ASSERT_TRUE(f.prepareTail(100.05));
 const scan_planner::MeasuredBodyPose solver_body{{.005,0.,.55},Eigen::Quaterniond::Identity(),100.05,"odom"};
 f.acquire(100.15,.015,.1);const auto newer=f.progress(100.15);f.native.progressAt(newer,100.15);
 // This is the actual old failure: latest independent progress is 100 ms
 // ahead of the solver owner's acquired body. The original source rule fails.
 auto unpaired=f.native.plannedSpliceAt(version(),100150000000LL,.05);ASSERT_TRUE(unpaired);
 auto paired=f.native.plannedSpliceAtBody(version(),100150000000LL,.05,solver_body,100050000000LL);
 ASSERT_TRUE(paired);EXPECT_EQ(paired->progress_source_ns,100050000000LL);
 EXPECT_EQ(paired->wire.measured_progress_stamp,t::stamp(100.05));
 EXPECT_GE(paired->committed_arc,newer.s_committed);
 const auto horizon=f.native.committedHorizonAtBody(version(),100150000000LL,solver_body,100050000000LL);
 ASSERT_TRUE(horizon);EXPECT_EQ(horizon->measured_source_ns,100050000000LL);
 EXPECT_GE(horizon->committed_arc,newer.s_committed);
 const auto check=[&](scan_planner::PlannedTrajectorySplice splice,scan_planner::SpliceCheckDiagnostics& diagnostic) {
   const auto& s=splice.wire;
   auto wire=tag(2,{s.position.x,s.position.y,s.position.z},
     {s.velocity.x,s.velocity.y,s.velocity.z},{s.acceleration.x,s.acceleration.y,s.acceleration.z});
   auto tail=scan_planner::decodeSpliceCurve(wire.trajectory);EXPECT_TRUE(tail);
   if(!tail)return false;
   scan_planner::CurveCheckCompletion completion=scan_planner::CurveCheckCompletion::Uncertified;
   return scan_planner::commonPrefixTailCollisionFree(*tail,splice,.05,.53,
     [&](const Eigen::Vector3d& p,double yaw){return f.map.getPlanningInflateOccupancy(p,yaw);},
     solver_body,"odom",100.15,.4,.2,f.heading,&completion,&diagnostic);
 };
 scan_planner::SpliceCheckDiagnostics diagnostic;
 EXPECT_FALSE(check(*unpaired,diagnostic));EXPECT_STREQ(diagnostic.phase,"progress_after_body");
 EXPECT_NEAR(diagnostic.source_delta_ms,-100.,1e-5);
 EXPECT_TRUE(check(*paired,diagnostic));EXPECT_STREQ(diagnostic.phase,"clear");
 // The same selector also serves actual incumbent proof renewal, not just a
 // solver helper. Neither the body nor the tracker progress gets restamped.
 const auto proof=Access::checkCopiedBody(f.native,f.map,100150000000LL,solver_body);
 ASSERT_TRUE(proof);EXPECT_TRUE(proof->valid)<<proof->reason;
 EXPECT_EQ(proof->body_source_stamp,t::stamp(100.05));
 const auto tail_proof=Access::checkCopiedBody(f.native,f.map,100150000000LL,solver_body,false);
 ASSERT_TRUE(tail_proof);EXPECT_TRUE(tail_proof->valid)<<tail_proof->reason;
 EXPECT_EQ(tail_proof->body_source_stamp,t::stamp(100.05));
 EXPECT_EQ(f.native.committedTrajectory(),1);
}
TEST(LocalFollowerNative,ActualTailProjectionCarriesNativeSourceDomainIntoCommit) {
 Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},Eigen::Vector3d::Zero()));
 ASSERT_TRUE(f.command(100.05,.01,.2,1));ASSERT_TRUE(f.prepareTail(100.05));
 const auto candidate=Access::pendingCurve(f.native);ASSERT_TRUE(candidate);
 const double x=candidate->splice.position.x+.02;
 for(unsigned i=2;i<=16;++i)ASSERT_TRUE(f.command(100.+i*.05,.2*i*.05,.2,i));
 ASSERT_TRUE(f.command(100.85,x,.2,17));
 const auto proof=Access::checkCurve(f.native,f.map,100850000000LL);ASSERT_TRUE(proof);
 ASSERT_TRUE(proof->valid)<<proof->reason;EXPECT_NEAR(proof->valid_start_arc_length,.02,1e-6);
 EXPECT_GT(proof->valid_start_time,0.);EXPECT_EQ(proof->checked_from_time,0.);
 EXPECT_EQ(proof->body_source_stamp,t::stamp(100.85));EXPECT_TRUE(f.follower.validation(*proof));
 const auto disposition=f.native.pendingSpliceDispositionAt(version(),100850000000LL);
 ASSERT_TRUE(disposition.keep)<<disposition.reason;
 const auto commit=f.follower.tryCommit(100.85,10.85);ASSERT_TRUE(commit)<<f.follower.candidateReason();
 EXPECT_GT(commit->curve_time,0.);EXPECT_TRUE(f.native.localCommitAt(*commit,100850000000LL));
}
TEST(LocalFollowerNative,RawSupportHeightNoiseIsNotAnotherNativeCollisionClassifier) {
 auto original=support();for(auto& point:original.support_ground_xyz)point.z=.30;
 Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},Eigen::Vector3d::Zero()),false,original);
 ASSERT_TRUE(f.initial_commit);ASSERT_TRUE(f.command(100.05,.01,.2,1));
 GridMapTestAccess::voxel(f.map,{.75,0.,.55},2.);
 const auto proof=Access::checkCurve(f.native,f.map,100050000000LL,true);
 ASSERT_TRUE(proof);EXPECT_FALSE(proof->valid);EXPECT_EQ(proof->reason,"occupied");
}
TEST(LocalFollowerNative,ReservedEntrySpeedMismatchRequestsOneActualReseedNotPermanentSpliceWait) {
 Integrated f(true,false,{},tag(1,{0,0,.55},{.2,0,0},Eigen::Vector3d::Zero()));
 ASSERT_TRUE(f.command(100.05,.01,.2,1));ASSERT_TRUE(f.prepareTail(100.05));
 auto proof=Access::checkCurve(f.native,f.map,100050000000LL);ASSERT_TRUE(proof&&proof->valid);
 ASSERT_TRUE(f.follower.validation(*proof));auto request=f.follower.exchangeCommit(100.05,10.05);ASSERT_TRUE(request);
 auto ack=f.native.localCommitExchangeAt(*request,100050000000LL);ASSERT_TRUE(ack.accepted);
 ASSERT_TRUE(f.follower.commitResult(ack,100.06,10.06));
 const auto candidate=Access::pendingCurve(f.native);ASSERT_TRUE(candidate);
 f.acquire(100.85,candidate->splice.position.x+.02,.3);
 f.native.progressAt(f.progress(100.85),100.85);
 proof=Access::checkReserved(f.native,f.map,100850000000LL);ASSERT_TRUE(proof&&proof->valid);
 ASSERT_TRUE(f.follower.validation(*proof));
 auto abort=f.follower.exchangeCommit(100.85,10.85);ASSERT_TRUE(abort);
 EXPECT_EQ(abort->disposition,"abort");EXPECT_EQ(abort->reason,"entry_measured_velocity_off_candidate");
 EXPECT_EQ(f.follower.controller().trajectoryId(),1);EXPECT_EQ(f.native.committedTrajectory(),1);
 EXPECT_TRUE(f.native.localCommitExchangeAt(*abort,100850000000LL).accepted);
 const auto event=f.native.consumeFormalReseedAt(100850000000LL);ASSERT_TRUE(event);
 EXPECT_TRUE(event->actual_body);EXPECT_EQ(event->trajectory_id,2);EXPECT_EQ(event->version,candidate->splice.incumbent_version);
 EXPECT_FALSE(f.native.consumeFormalReseedAt(100860000000LL));
 EXPECT_TRUE(f.native.localCommitExchangeAt(*abort,100860000000LL).accepted);
 EXPECT_FALSE(f.native.consumeFormalReseedAt(100860000000LL));
 auto foreign=*abort;foreign.version.task_id="other";
 EXPECT_FALSE(f.native.localCommitExchangeAt(foreign,100860000000LL).accepted);
 auto cancel=version();++cancel.reference_generation;ASSERT_TRUE(f.native.cancelReference(cancel));
 EXPECT_FALSE(f.native.localCommitExchangeAt(*abort,100870000000LL).accepted);
 EXPECT_FALSE(f.native.consumeFormalReseedAt(100870000000LL));
}
TEST(LocalFollowerNative,SourcePairingRejectsMissingStaleForeignAndReplayedEvidence) {
 Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
 ASSERT_TRUE(f.command(100.05,.005,.1,1));const auto original=f.progress(100.05);
 const scan_planner::MeasuredBodyPose actual{{.005,0.,.55},Eigen::Quaterniond::Identity(),100.05,"odom"};
 f.acquire(100.15,.015,.1);const auto newer=f.progress(100.15);f.native.progressAt(newer,100.15);
 auto replay=original;replay.s_committed=replay.arc_length=.025;replay.curve_time=.25;
 f.native.progressAt(replay,100.15); // Old valid-shaped source cannot move high water.
 replay.header.stamp=t::stamp(100.10);f.native.progressAt(replay,100.15);
 replay.header.stamp=newer.header.stamp;f.native.progressAt(replay,100.15); // Duplicate source also loses.
 auto paired=f.native.plannedSpliceAtBody(version(),100150000000LL,.05,actual,100050000000LL);
 ASSERT_TRUE(paired);EXPECT_DOUBLE_EQ(paired->committed_arc,newer.s_committed);
 EXPECT_EQ(paired->progress_source_ns,100050000000LL);
 auto foreign=version();foreign.task_id="other";
 EXPECT_FALSE(f.native.plannedSpliceAtBody(foreign,100150000000LL,.05,actual,100050000000LL));
 EXPECT_FALSE(f.native.plannedSpliceAtBody(version(),100600000000LL,.05,actual,100050000000LL));
 auto invented=actual;invented.position.y()=.01;
 EXPECT_FALSE(f.native.plannedSpliceAtBody(version(),100150000000LL,.05,invented,100050000000LL));
 invented=actual;invented.source_stamp=100.075;invented.position.x()=.0075;
 EXPECT_FALSE(f.native.plannedSpliceAtBody(version(),100150000000LL,.05,invented,100075000000LL));
 auto barrier=version();++barrier.reference_generation;ASSERT_TRUE(f.native.cancelReference(barrier));
 EXPECT_FALSE(f.native.plannedSpliceAtBody(version(),100150000000LL,.05,actual,100050000000LL));
 EXPECT_FALSE(f.native.committedHorizonAtBody(version(),100150000000LL,actual,100050000000LL));
}
TEST(LocalFollowerNative,MissingCausalProgressDoesNotPublishOrExtendOriginalProof) {
 Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
 const auto before=f.native.committedView();ASSERT_TRUE(before&&before->proof);
 const scan_planner::MeasuredBodyPose older_body{{0.,0.,.55},Eigen::Quaterniond::Identity(),100.,"odom"};
 f.acquire(100.15,.015,.1);f.native.progressAt(f.progress(100.15),100.15);
 EXPECT_FALSE(f.native.plannedSpliceAtBody(version(),100150000000LL,.05,older_body,100000000000LL));
 EXPECT_FALSE(f.native.committedHorizonAtBody(version(),100150000000LL,older_body,100000000000LL));
 EXPECT_FALSE(Access::checkCopiedBody(f.native,f.map,100150000000LL,older_body));
 EXPECT_FALSE(Access::checkCopiedBody(f.native,f.map,100300000000LL,older_body));
 const auto after=f.native.committedView();ASSERT_TRUE(after&&after->proof);
 EXPECT_EQ(after->proof->sequence,before->proof->sequence);
 EXPECT_EQ(after->proof->valid_until,before->proof->valid_until);
 EXPECT_LE(t::timeNs(after->proof->valid_until),100300000000LL);
}
TEST(LocalFollowerNative,ReorderedProgressWakesCurrentReaderOnceWhenOriginalBodyArrives) {
 Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
 f.acquire(100.04,.004,.1);
 // Independent topics deliver the follower's projection 40 ms ahead of
 // Native's body inbox. This is not a collision or a new acquired body.
 ASSERT_TRUE(f.follower.body(body(100.08,.008,.1),100.08,10.08));
 f.native.progressAt(f.progress(100.08),100.08);
 GridMapTestAccess::acquire(f.map,100080000000LL);
 const auto before=f.native.committedView();ASSERT_TRUE(before&&before->proof);
 EXPECT_FALSE(Access::checkCurve(f.native,f.map,100081000000LL,true));
 EXPECT_FALSE(Access::consumeRefreshWake(f.native)); // No input: no self-wake loop.
 const auto waiting=f.native.committedView();ASSERT_TRUE(waiting&&waiting->proof);
 EXPECT_EQ(waiting->proof->sequence,before->proof->sequence);
 EXPECT_EQ(waiting->proof->valid_until,before->proof->valid_until);
 geometry_msgs::msg::Twist twist;twist.linear.x=.1;
 f.native.bodyAt({{.006,0.,.55},Eigen::Quaterniond::Identity(),100.06,"odom"},twist,
   100060000000LL,100082000000LL);
 EXPECT_FALSE(Access::consumeRefreshWake(f.native)); // Still older than projection.
 f.native.bodyAt({{.008,0.,.55},Eigen::Quaterniond::Identity(),100.08,"odom"},twist,
   100080000000LL,100083000000LL);
 EXPECT_TRUE(Access::consumeRefreshWake(f.native)); // Do not wait the 50 ms timer.
 EXPECT_FALSE(Access::consumeRefreshWake(f.native));
 f.native.bodyAt({{.008,0.,.55},Eigen::Quaterniond::Identity(),100.08,"odom"},twist,
   100080000000LL,100084000000LL);
 EXPECT_FALSE(Access::consumeRefreshWake(f.native)); // Duplicate delivery is inert.
 const auto renewed=Access::checkCurve(f.native,f.map,100084000000LL,true);
 ASSERT_TRUE(renewed&&renewed->valid);EXPECT_EQ(t::timeNs(renewed->body_source_stamp),100080000000LL);
 EXPECT_EQ(renewed->trajectory_id,before->proof->trajectory_id);
}
TEST(LocalFollowerNative,SourcePairWakeCannotCrossRetirementNegativeOrOriginalExpiry) {
 for(int barrier=0;barrier<3;++barrier) {
  Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
  f.acquire(100.04,.004,.1);
  ASSERT_TRUE(f.follower.body(body(100.08,.008,.1),100.08,10.08));
  f.native.progressAt(f.progress(100.08),100.08);GridMapTestAccess::acquire(f.map,100080000000LL);
  EXPECT_FALSE(Access::checkCurve(f.native,f.map,100081000000LL,true));
  ASSERT_FALSE(Access::consumeRefreshWake(f.native));
  if(barrier==0){auto cancel=version();++cancel.reference_generation;ASSERT_TRUE(f.native.cancelReference(cancel));}
  if(barrier==1){
   geometry_msgs::msg::Twist paired_twist;paired_twist.linear.x=.1;
   f.native.bodyAt({{.008,0.,.55},Eigen::Quaterniond::Identity(),100.08,"odom"},paired_twist,
     100080000000LL,100082000000LL);
   Access::consumeRefreshWake(f.native);
   GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.);
   const auto negative=Access::checkCurve(f.native,f.map,100082000000LL,true);
   ASSERT_TRUE(negative&&!negative->valid);
  }
  geometry_msgs::msg::Twist twist;twist.linear.x=.1;
  f.native.bodyAt({{.008,0.,.55},Eigen::Quaterniond::Identity(),100.08,"odom"},twist,
    100080000000LL,barrier==2?100481000000LL:100083000000LL);
  EXPECT_FALSE(Access::consumeRefreshWake(f.native))<<barrier;
 }
}
TEST(LocalFollowerNative,SourcePairArrivalBetweenInputCopyAndRegistrationIsNotLost) {
 Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
 f.acquire(100.04,.004,.1);
 const scan_planner::MeasuredBodyPose copied{{.004,0.,.55},Eigen::Quaterniond::Identity(),100.04,"odom"};
 f.acquire(100.08,.008,.1);f.native.progressAt(f.progress(100.08),100.08);
 // The original reader keeps its old acquired body while the independent
 // inbox has already caught up. Registration must not wait for another packet.
 EXPECT_FALSE(Access::checkCopiedBody(f.native,f.map,100081000000LL,copied));
 EXPECT_TRUE(Access::consumeRefreshWake(f.native));
 EXPECT_FALSE(Access::consumeRefreshWake(f.native));
 const auto renewed=Access::checkCurve(f.native,f.map,100082000000LL,true);
 ASSERT_TRUE(renewed&&renewed->valid);
 EXPECT_EQ(t::timeNs(renewed->body_source_stamp),100080000000LL);
}
TEST(LocalFollowerNative,OlderPairCannotHideNewerIndependentDeviation) {
 Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
 ASSERT_TRUE(f.command(100.05,.005,.1,1));
 const scan_planner::MeasuredBodyPose older_body{{.005,0.,.55},Eigen::Quaterniond::Identity(),100.05,"odom"};
 auto progress=f.progress(100.05);progress.header.stamp=t::stamp(100.10);
 progress.pose.position.y=.08;
 geometry_msgs::msg::Twist twist;twist.linear.x=.1;
 f.native.body({{.005,.08,.55},Eigen::Quaterniond::Identity(),100.10,"odom"},twist,100100000000LL);
 f.native.progressAt(progress,100.10);
 EXPECT_FALSE(f.native.plannedSpliceAtBody(version(),100100000000LL,.05,older_body,100050000000LL));
 const auto event=f.native.consumeFormalReseedAt(100100000000LL,.05);
 ASSERT_TRUE(event);EXPECT_EQ(event->reason,"measured_body_left_local_curve");
 EXPECT_EQ(f.native.committedTrajectory(),1);
}
TEST(LocalFollowerNative,ReplacementCommitCannotBorrowOldCurveProgressHistory) {
 Integrated f(true,false,{},tag(1,{0.,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero()));
 ASSERT_TRUE(f.command(100.05,.005,.1,1));const auto old_progress=f.progress(100.05);
 f.acquire(100.10,.01,.1);
 auto replacement=tag(2,{.01,0.,.55},{.1,0.,0.},Eigen::Vector3d::Zero());
 replacement.join_source_stamp=replacement.trajectory.start_time=t::stamp(100.10);
 f.native.candidate(replacement,f.reference,f.heading);ASSERT_TRUE(f.follower.candidate(replacement));
 const auto proof=Access::checkCurve(f.native,f.map,100100000000LL);ASSERT_TRUE(proof&&proof->valid);
 ASSERT_TRUE(f.follower.validation(*proof));const auto commit=f.follower.tryCommit(100.10,10.10);
 ASSERT_TRUE(commit)<<f.follower.candidateReason();ASSERT_TRUE(f.native.localCommitAt(*commit,100100000000LL));
 ASSERT_EQ(f.native.committedTrajectory(),2);
 f.native.progressAt(old_progress,100.10);
 const scan_planner::MeasuredBodyPose actual{{.005,0.,.55},Eigen::Quaterniond::Identity(),100.05,"odom"};
 EXPECT_FALSE(f.native.plannedSpliceAtBody(version(),100100000000LL,.05,actual,100050000000LL));
 EXPECT_FALSE(f.native.committedHorizonAtBody(version(),100100000000LL,actual,100050000000LL));
}
TEST(LocalFollowerNative,OlderNegativePublicationCannotDiscardNewerPositiveBodyLateDemand) {
 Integrated f;ASSERT_TRUE(f.command(100.05,0.,0.,1));
 f.acquire(100.10,0.,0.);f.native.progressAt(f.progress(100.10),100.10);
 GridMapTestAccess::voxel(f.map,{.3,0.,.55},2.);
 std::vector<w::LocalMotionCommand> sent;
 const auto emit=[&](const auto& command){sent.push_back(command);};
 std::optional<w::MotionDemand> newer_demand;std::uint64_t newer_proof_sequence=0;
 const auto negative=Access::checkCurvePublicationInterleave(f.native,f.map,100100000000LL,
   [&](const auto& published) {
     ASSERT_FALSE(published.valid);
     // Reproduce the unlocked publication boundary: a second real reader
     // publishes N+1 and a new control sample arrives before N's post-publish
     // bookkeeping resumes. No proof/occupancy decision is fabricated.
     GridMapTestAccess::voxel(f.map,{.3,0.,.55},-1.);
     GridMapTestAccess::acquire(f.map,100110000000LL);
     const auto fresh=Access::checkCurve(f.native,f.map,100110000000LL,true,emit);
     ASSERT_TRUE(fresh&&fresh->valid);ASSERT_GT(fresh->sequence,published.sequence);
     newer_proof_sequence=fresh->sequence;ASSERT_TRUE(f.follower.validation(*fresh));
     // Only the follower has the new real body sample so far. Native must
     // retain this exact command until the independent body inbox receives it.
     ASSERT_TRUE(f.follower.body(body(100.11,0.,0.),100.11,10.11));
     const auto grant=lease(100.11,2);ASSERT_TRUE(f.follower.lease(grant,100.11));
     ASSERT_TRUE(f.native.taskLeaseAt(grant,100110000000LL));
     ASSERT_TRUE(f.native.commit(f.follower.geometryPermit(100.11)));
     newer_demand=f.follower.step(100.11,10.11);ASSERT_FALSE(newer_demand->hold);
     ASSERT_EQ(newer_demand->validation_sequence,newer_proof_sequence);
     f.native.demandAt(*newer_demand,100111000000LL,emit);
     ASSERT_TRUE(sent.empty());ASSERT_TRUE(Access::pendingDemand(f.native));
     ASSERT_EQ(Access::diagnostics(f.native).stage,scan_planner::MotionCheckStage::BodyBehindDemand);
   },emit);
 ASSERT_TRUE(negative&&!negative->valid);ASSERT_TRUE(newer_demand);
 const auto current=f.native.committedView();ASSERT_TRUE(current&&current->proof);
 EXPECT_EQ(current->proof->sequence,newer_proof_sequence);EXPECT_TRUE(current->proof->valid);
 // The older negative result must not delete the new post-recovery command.
 ASSERT_TRUE(Access::pendingDemand(f.native));EXPECT_TRUE(sent.empty());
 const auto measured=body(100.11,0.,0.);geometry_msgs::msg::Twist twist;
 f.native.bodyAt({measured.position,measured.orientation,100.11,"odom"},twist,
   100110000000LL,100112000000LL,emit);
 ASSERT_EQ(sent.size(),1U);EXPECT_EQ(sent[0].sequence,newer_demand->sequence);
 EXPECT_EQ(sent[0].source_stamp,newer_demand->source_stamp);
 EXPECT_EQ(sent[0].body_source_stamp,newer_demand->body_source_stamp);
 EXPECT_EQ(sent[0].velocity,newer_demand->velocity);EXPECT_FALSE(Access::pendingDemand(f.native));
}
