// Actual cubic projection/adoption transaction and production GridMap queries.
// In-memory native map inputs and injected clocks only: no ROS init or robot.
#include <gtest/gtest.h>
#include <plan_env/grid_map.h>
#include <plan_manage/candidate_adoption.hpp>
#include <plan_manage/forward_motion_boundary.hpp>
#include <plan_manage/input_contract.hpp>
#include <plan_manage/worker_lifecycle.hpp>
#include <plan_env/collision_snapshot_pool.hpp>
#include <future>

struct GridMapTestAccess {
  static GridMap::Ptr make() {
    auto map=std::make_shared<GridMap>();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.08;p.resolution_inv_=12.5;p.map_voxel_num_={64,64,32};
    p.map_origin_idx_.setZero();map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.require_observed_free_=true;p.double_cylinder_radius_=.1;p.double_cylinder_offset_=.1;
    p.obstacles_inflation_z_up=.15;p.obstacles_inflation_z_down=.15;
    p.cloud_pose_max_age_=.5;
    d.occupancy_buffer_.assign(64*64*32,-1.);
    d.occupancy_buffer_inflate_.assign(64*64*32,0);
    d.occupancy_buffer_inflate_cnt_.assign(64*64*32,0);
    map->integrated_cloud_stamp_ns_=10300000000LL;
    map->localization_context_sequence_=7;
    map->rebuildInflationOffsets();return map;
  }
  static void raw(GridMap &map,const Eigen::Vector3d &p,int value) {
    Eigen::Vector3i cell;map.posToIndex(p,cell);
    map.md_.occupancy_buffer_[map.toAddress(cell)]=value==1?2.:-1.01;
    map.observed_cylinder_cache_.clear();map.planning_box_cache_.clear();++map.occupancy_revision_;
  }
  static void source(GridMap &map,std::int64_t time) {map.integrated_cloud_stamp_ns_=time;}
  static void revision(GridMap &map) {++map.occupancy_revision_;}
  static void context(GridMap &map) {++map.localization_context_sequence_;}
  static void box(GridMap& map) {
    auto& p=map.mp_;p.collision_model_="oriented_box";p.box_length_=.93;p.box_width_=.30;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    p.planning_collision_policy_="official";p.require_observed_free_=false;
    map.rebuildInflationOffsets();
  }
};

namespace {
using namespace scan_planner;
UniformBspline line(double speed=.3) {
  Eigen::MatrixXd points(3,23);
  for(int i=0;i<23;++i) points.col(i)=Eigen::Vector3d((i-1)*speed*.2,0.,.55);
  return UniformBspline(points,3,.2);
}
struct Fixture {
  GridMap::Ptr map=GridMapTestAccess::make();
  UniformBspline curve=line();
  MeasuredBodyPose original{{0.,0.,.55},Eigen::Quaterniond::Identity(),10.,"map"};
  MeasuredBodyPose measured{{.09,0.,.55},Eigen::Quaterniond::Identity(),10.3,"map"};
  Eigen::Vector3d velocity{.3,0.,0.};
  SolveBudget::Time steady=SolveBudget::Clock::now();
  SolveBudget::Ptr budget=std::make_shared<SolveBudget>(std::chrono::milliseconds(400),[this]{return steady;});
  double now=10.31,body_max_age=.5;
  bool single_segment=true;
  std::uint64_t candidate_context=7;
  int checks=0;
  MeasuredConnectionPolicy connection=MeasuredConnectionPolicy::CandidateAdmission;
  MotionCandidateAdoptionOutcome outcome=MotionCandidateAdoptionOutcome::NotAttempted;
  std::function<void()> during_check=[]{};
  std::optional<CandidateJoinEvidence> certify() {
    return certifyCandidateAdoption(curve,original,measured,velocity,.08,.3,.6,
        single_segment,candidate_context,budget,[&] {
      return CandidateSourceLease{map->latestCloudStampNs(),map->localizationContextSequence(),
          map->occupancyRevision(),measured.source_stamp};
    },[&] {
      double yaw=0.;
      return std::make_pair(map->integratedCloudFreshAt(std::llround(now*1e9)),
          measuredBodyYaw(measured,"map",now,body_max_age,yaw));
    },[&](double time) {
      ++checks;during_check();
      return wholeCurveCollisionFree(curve,.08,.2,
          [&](const Eigen::Vector3d &p,double yaw){return
              connection==MeasuredConnectionPolicy::RollingCandidateConnection?
                map->getPlanningInflateOccupancy(p,yaw):map->getInflateOccupancy(p,yaw);},
          measured,"map",10.31,.5,200000,budget->remainingSeconds(),time,{},0.,connection);
    },&outcome,connection);
  }
};
struct PreviewFixture : Fixture {
  PreviewFixture() {GridMapTestAccess::box(*map);}
  PreviewCandidateOrigin origin()const {return {original,{.3,0.,0.},.3,.6,single_segment,{}};}
  PreviewCandidateAcceptanceResult certifyPreview() {
    return certifyPreviewCandidateGeometry(curve,origin(),measured,map->getResolution(),
      candidate_context,budget,[&]{return CandidateSourceLease{map->latestCloudStampNs(),
        map->localizationContextSequence(),map->occupancyRevision(),measured.source_stamp};},[&] {
        double yaw=0.;return std::make_pair(map->integratedCloudFreshAt(std::llround(now*1e9)),
          measuredBodyYaw(measured,"map",now,body_max_age,yaw));
      },[&](const PreviewCandidateGeometryEvidence& evidence) {
        ++checks;during_check();
        return wholeCurveCollisionFree(curve,map->getResolution(),bodyExtent(map->planningBodyEnvelope()),
          [&](const Eigen::Vector3d& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);},
          measured,"map",now,body_max_age,200000,budget->remainingSeconds(),
          evidence.projection.curve_time,{},0.,MeasuredConnectionPolicy::PreviewCandidateGeometry,
          evidence.origin.policy.connector_m);
      });
  }
};
}

TEST(ForwardBoundary,ActualNearRestBackwardTraceDoesNotBecomePiTurnBoundary) {
  const Eigen::Vector3d position{0.,0.,.55},measured_velocity{-.017038,-.033369,0.};
  const auto original=measured_velocity;
  const std::vector<Eigen::Vector3d> reference{position,{0.,.4,.55},{0.,.8,.55}};
  const auto selected=selectForwardMotionBoundary(position,measured_velocity,reference,true,false);
  ASSERT_TRUE(selected.regularized);EXPECT_TRUE(selected.velocity.isZero());
  EXPECT_TRUE(measured_velocity.isApprox(original));
  EXPECT_DOUBLE_EQ(selected.velocity_residual_mps,measured_velocity.norm());
  EXPECT_LT(selected.velocity_residual_mps,measured_candidate_join_velocity_tolerance_mps);
  SplineHeadingContract heading;heading.measured_heading_enabled=true;heading.align_then_forward=true;
  heading.measured_yaw=1.658913;
  const double before=bodyHeadingFromVelocity(measured_velocity,heading.measured_yaw,heading);
  const double after=bodyHeadingFromVelocity(selected.velocity,heading.measured_yaw,heading);
  const auto angle=[](double x){return std::atan2(std::sin(x),std::cos(x));};
  EXPECT_GT(std::abs(angle(before-heading.measured_yaw)),2.5);
  EXPECT_DOUBLE_EQ(angle(after-heading.measured_yaw),0.);
  std::vector<Eigen::Vector3d> samples;
  for(int i=0;i<=20;++i)samples.push_back({0.,i*.04,.55});
  CubicMotionBoundary boundary{position,selected.velocity,Eigen::Vector3d::Zero(),
    reference.back(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
  UniformBspline curve(fitCubicWithFixedBoundary(samples,.2,boundary),3,.2);
  ASSERT_TRUE(cubicBoundaryMatches(curve,boundary));
  EXPECT_GT(curve.getDerivative().evaluateDeBoorT(.1).y(),0.);
  std::cout<<"forward_boundary_trace old_heading_error="<<angle(before-heading.measured_yaw)
    <<" selected_heading_error="<<angle(after-heading.measured_yaw)
    <<" actual_velocity_residual="<<selected.velocity_residual_mps<<std::endl;
}
TEST(CandidateAdoption,FiniteInitialSpeedRecoveryStillRequiresActualFreshMeasuredJoin) {
  Fixture f;
  Eigen::MatrixXd points=Eigen::MatrixXd::Zero(3,10);points.row(2).setConstant(.55);
  const double speeds[]={.41698,.41698,.3,.3,.3,.3,.15,0.,0.};
  for(int i=1;i<10;++i)points(0,i)=points(0,i-1)+.5*speeds[i-1];
  f.curve=UniformBspline(points,3,.5);
  f.original.position=f.curve.evaluateDeBoorT(0.);f.measured=f.original;
  f.velocity={.41698,0.,0.};
  ASSERT_TRUE(measuredCandidateJoin(f.curve,f.original,f.measured,f.velocity,.05,.3,.35,true,f.budget));
  const auto original=f.original;
  f.measured.source_stamp+=.02;f.measured.position=f.curve.evaluateDeBoorT(.02);
  auto derivative=f.curve.getDerivative();f.velocity=derivative.evaluateDeBoorT(.02);
  EXPECT_TRUE(measuredCandidateJoin(f.curve,original,f.measured,f.velocity,.05,.3,.35,true,f.budget));
  EXPECT_FALSE(measuredCandidateJoin(f.curve,original,f.measured,-f.velocity,.05,.3,.35,true,f.budget));
  f.measured.source_stamp=original.source_stamp+.401;
  EXPECT_FALSE(measuredCandidateJoin(f.curve,original,f.measured,f.velocity,.05,.3,.35,true,f.budget));
  f.measured=original;f.budget->cancel();
  EXPECT_FALSE(measuredCandidateJoin(f.curve,original,f.measured,f.velocity,.05,.3,.35,true,f.budget));
}
TEST(ForwardBoundary,ForwardMotionSplicesHolonomicPreviewAndLargerActualMotionRemainExact) {
  const Eigen::Vector3d position{0.,0.,.55};
  const std::vector<Eigen::Vector3d> reference{position,{0.,1.,.55}};
  for(const Eigen::Vector3d& v:std::vector<Eigen::Vector3d>{{0.,.04,0.},{.001,.04,.005},{0.,-.051,0.},{.08,0.,0.}}) {
    const auto result=selectForwardMotionBoundary(position,v,reference,true,false);
    EXPECT_FALSE(result.regularized);EXPECT_TRUE(result.velocity.isApprox(v));
  }
  const Eigen::Vector3d small_backward{0.,-.03,0.};
  for(const auto flags:std::vector<std::pair<bool,bool>>{{false,false},{true,true},{false,true}}) {
    const auto result=selectForwardMotionBoundary(position,small_backward,reference,flags.first,flags.second);
    EXPECT_FALSE(result.regularized);EXPECT_TRUE(result.velocity.isApprox(small_backward));
  }
  // Exactly the existing join tolerance is not rounded down to a zero start.
  const Eigen::Vector3d edge{0.,-.05,0.};
  EXPECT_FALSE(selectForwardMotionBoundary(position,edge,reference,true,false).regularized);
}
TEST(ForwardBoundary,OrderedNearestReferenceUsesXYZNotVectorBackToRoute) {
  const Eigen::Vector3d position{.1,.5,3.55},velocity{0.,.03,0.};
  const std::vector<Eigen::Vector3d> reference{{0.,1.,.55},{0.,0.,.55},{0.,0.,3.55},{0.,1.,3.55}};
  const auto result=selectForwardMotionBoundary(position,velocity,reference,true,false);
  EXPECT_FALSE(result.regularized);EXPECT_GT(result.reference_direction.y(),.99);
  EXPECT_TRUE(result.velocity.isApprox(velocity));
  EXPECT_FALSE(selectForwardMotionBoundary(position,velocity,{},true,false).regularized);
  EXPECT_FALSE(selectForwardMotionBoundary(position,velocity,{{0.,0.,.55}},true,false).regularized);
}
TEST(ForwardBoundary,ArtificialBodyProjectionConnectorIsNotTheRouteForwardDirection) {
  const Eigen::Vector3d position{0.,.08,.55},velocity{.03,0.,0.};
  for(const Eigen::Vector3d& projection:std::vector<Eigen::Vector3d>{
      {0.,0.,.55},{.001,0.,.52}}) {
    const std::vector<Eigen::Vector3d> reference{position,projection,
      projection+Eigen::Vector3d(.5,0.,0.),projection+Eigen::Vector3d(1.,0.,0.)};
    const auto boundary=selectForwardMotionBoundary(position,velocity,reference,true,false,0.,true);
    EXPECT_TRUE(boundary.reference_direction.isApprox(Eigen::Vector3d::UnitX()));
    EXPECT_TRUE(boundary.velocity.isApprox(velocity));EXPECT_FALSE(boundary.regularized);
    EXPECT_FALSE(boundary.reverse_transient);
  }
}
TEST(ForwardBoundary,RealReverseTransientIsDiagnosticAndPreservesActualPva) {
  const Eigen::Vector3d position{0.,0.,.55},actual_reverse{.0129,-.120,-.0044};
  const std::vector<Eigen::Vector3d> reference{position,{0.,.4,.55},{0.,.8,.55}};
  const double body_yaw=1.658913;
  const auto reverse=selectForwardMotionBoundary(position,actual_reverse,reference,true,false,body_yaw);
  EXPECT_TRUE(reverse.reverse_transient);EXPECT_FALSE(reverse.regularized);
  EXPECT_TRUE(reverse.velocity.isApprox(actual_reverse)); // NEVER falsify a .12 m/s acquired boundary
  EXPECT_DOUBLE_EQ(reverse.velocity_residual_mps,0.);
  // The existing near-rest policy is unchanged; the diagnostic flag does not
  // create a waiting state or replace a real reverse velocity with fake zero.
  const Eigen::Vector3d near_rest{-.017038,-.033369,0.};
  const auto recovered=selectForwardMotionBoundary(position,near_rest,reference,true,false,body_yaw);
  EXPECT_FALSE(recovered.reverse_transient);EXPECT_TRUE(recovered.regularized);
  const auto healthy=selectForwardMotionBoundary(position,{0.,.12,0.},reference,true,false,body_yaw);
  EXPECT_FALSE(healthy.reverse_transient);EXPECT_TRUE(healthy.velocity.isApprox(Eigen::Vector3d(0.,.12,0.)));
  for(const auto yaw:std::vector<std::optional<double>>{{},{body_yaw+std::acos(-1.)},{0.}}) {
    const auto other=selectForwardMotionBoundary(position,actual_reverse,reference,true,false,yaw);
    EXPECT_FALSE(other.reverse_transient);EXPECT_TRUE(other.velocity.isApprox(actual_reverse));
  }
  for(const auto flags:std::vector<std::pair<bool,bool>>{{false,false},{true,true}})
    EXPECT_FALSE(selectForwardMotionBoundary(position,actual_reverse,reference,flags.first,flags.second,
      body_yaw).reverse_transient);
}
TEST(ForwardBoundary,SelectedBoundaryStillUsesUnchangedActualCandidateJoinAndNativeCollision) {
  for(unsigned scenario=0;scenario<3;++scenario) {
    Fixture f;f.measured=f.original;f.measured.source_stamp=10.01;f.now=10.02;
    f.velocity={-.017038,-.033369,0.};GridMapTestAccess::source(*f.map,10010000000LL);
    const auto measured_before=f.measured;
    std::vector<Eigen::Vector3d> samples;
    for(int i=0;i<=20;++i)samples.push_back({0.,i*.04,.55});
    const auto selected=selectForwardMotionBoundary(f.original.position,f.velocity,samples,true,false);
    ASSERT_TRUE(selected.regularized);
    const CubicMotionBoundary boundary{f.original.position,selected.velocity,Eigen::Vector3d::Zero(),
      samples.back(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
    f.curve=UniformBspline(fitCubicWithFixedBoundary(samples,.2,boundary),3,.2);
    // The planner's actual stationary-boundary retiming still precedes the
    // actual join; raw fixed-dt samples alone exceed the acceleration bound.
    enforceDerivativeBounds(f.curve,.3,.6);
    ASSERT_TRUE(cubicBoundaryMatches(f.curve,boundary));
    if(scenario==1)f.velocity={0.,-.051,0.}; // actual join residual is NOT widened
    if(scenario==2)GridMapTestAccess::raw(*f.map,{0.,.4,.55},1);
    const auto accepted=f.certify();EXPECT_EQ(accepted.has_value(),scenario==0)<<scenario
      <<" "<<motionCandidateAdoptionReason(f.outcome);
    EXPECT_DOUBLE_EQ(f.measured.source_stamp,measured_before.source_stamp);
    EXPECT_TRUE(f.measured.position.isApprox(measured_before.position));
    if(accepted)EXPECT_TRUE(accepted->velocity.isApprox(f.velocity));
  }
}

TEST(PreviewCandidate, SameNativeBoxInputsAcceptMovingLateralThreeToTenCentimetersAndVelocityDrift) {
  for(double lateral:{.03,.06,.10}) {
    PreviewFixture f;f.measured.position.y()=lateral;f.velocity={-.2,.1,0.};
    const auto controls=f.curve.getControlPoint();const auto knots=f.curve.getKnot();
    EXPECT_FALSE(f.certify()); // unchanged actual Motion join/velocity gate
    f.checks=0;const auto result=f.certifyPreview();ASSERT_TRUE(result.geometry)<<lateral;
    EXPECT_EQ(result.outcome,CandidateAcceptanceOutcome::PreviewGeometryAccepted);EXPECT_EQ(f.checks,1);
    EXPECT_NEAR(result.geometry->projection.curve_time,.3,1e-6);
    EXPECT_NEAR(result.geometry->projection.residual,lateral,1e-8);
    EXPECT_EQ(result.geometry->projection.source_stamp,10.3);
    EXPECT_TRUE(f.curve.getControlPoint().isApprox(controls));EXPECT_TRUE(f.curve.getKnot().isApprox(knots));
    EXPECT_TRUE(result.geometry->origin.solve_velocity.isApprox(Eigen::Vector3d(.3,0.,0.)));
  }
}
TEST(PreviewCandidate, ProductionFixedBoundaryAndMovingRetimingPreserveOriginalAcquiredVelocity) {
  PreviewFixture f;const CubicMotionBoundary boundary{f.original.position,{.09,.02,0.},
    Eigen::Vector3d::Zero(),{.8,0.,.55},Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> samples;
  for(int i=0;i<=20;++i)samples.push_back({i*.04,0.,.55});
  f.curve=UniformBspline(fitCubicWithFixedBoundary(samples,.1,boundary),3,.1);
  const auto timing=refineTimingWithFixedBoundary(f.curve,boundary,.3,.6,
      [](Eigen::MatrixXd&,double){return true;});
  ASSERT_TRUE(timing.success)<<timing.reason;ASSERT_GT(timing.refinements,0);
  ASSERT_TRUE(cubicBoundaryMatches(f.curve,boundary));
  auto origin=f.origin();origin.solve_velocity=boundary.start_velocity;
  f.measured.position=f.original.position+Eigen::Vector3d(.04,.07,0.);
  auto result=projectPreviewCandidateGeometry(f.curve,origin,f.measured,.08,f.budget);
  ASSERT_TRUE(result.geometry);
  EXPECT_LT((f.curve.getDerivative().evaluateDeBoorT(0.)-boundary.start_velocity).norm(),1e-7);
  origin.solve_velocity.x()+=.01;
  EXPECT_EQ(projectPreviewCandidateGeometry(f.curve,origin,f.measured,.08,f.budget).outcome,
      CandidateAcceptanceOutcome::InvalidPreviewCandidate);
}
TEST(PreviewCandidate, IndependentRevalidationDoesNotExpireImmutableSolveOriginAfterReaderQueueDelay) {
  PreviewFixture f;f.measured.source_stamp=10.6;f.measured.position.y()=.08;
  const auto original_stamp=f.origin().solve_body.source_stamp;
  EXPECT_EQ(projectPreviewCandidateGeometry(f.curve,f.origin(),f.measured,.08,f.budget).outcome,
      CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed); // bounded adoption still .4 s
  const auto revalidate=[&]{return projectPreviewCandidateGeometryWhileAllowed(f.curve,f.origin(),
      f.measured,.08,[&]{return f.budget->allowed();},PreviewCandidateProjectionPhase::CandidateRevalidation);};
  const auto result=revalidate();ASSERT_TRUE(result.geometry);
  EXPECT_EQ(result.geometry->origin.solve_body.source_stamp,original_stamp);
  EXPECT_EQ(result.geometry->projection.source_stamp,10.6);
  double yaw=0.;EXPECT_TRUE(measuredBodyYaw(f.measured,"map",10.61,.4,yaw));
  EXPECT_FALSE(measuredBodyYaw(f.measured,"map",11.01,.4,yaw)); // reading cannot renew its own source
  f.measured.position.y()=.16;EXPECT_EQ(revalidate().outcome,CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed);
  f.measured.source_stamp=9.9;EXPECT_EQ(revalidate().outcome,CandidateAcceptanceOutcome::PreviewSourceChanged);
}
TEST(PreviewCandidate, ActualConnectorObstacleAndOriginalWholeCurveStillBlock) {
  for(int mode=0;mode<2;++mode) {
    PreviewFixture f;f.measured.position.y()=.10;
    if(mode==0)GridMapTestAccess::raw(*f.map,{.025,.20,.55},1);
    else GridMapTestAccess::raw(*f.map,{.85,0.,.55},1);
    if(mode==0){EXPECT_EQ(f.map->getPlanningInflateOccupancy(f.original.position,0.),0);}
    const auto result=f.certifyPreview();EXPECT_FALSE(result.geometry);EXPECT_EQ(f.checks,1);
    EXPECT_EQ(result.outcome,CandidateAcceptanceOutcome::PreviewGeometryBlocked);
  }
}
TEST(PreviewCandidate, OutOfRangeFloorAndSemanticBranchCannotBecomePreviewOrMotionProgress) {
  for(int mode=0;mode<5;++mode) {
    PreviewFixture f;
    if(mode==0)f.measured.position.y()=.151;
    if(mode==1)f.measured.position.z()+=3.;
    if(mode==2)f.measured.position.x()=.7;
    if(mode==3)f.measured.source_stamp=10.401;
    if(mode==4)f.single_segment=false;
    const auto result=f.certifyPreview();EXPECT_FALSE(result.geometry);EXPECT_EQ(f.checks,0);
    EXPECT_EQ(result.outcome,mode==4?CandidateAcceptanceOutcome::PreviewSegmentBoundary:
        CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed)<<mode;
  }
}
TEST(PreviewCandidate, ExpiredCanceledMapIdentityAndExactActualBodyCasRemainIndependentGates) {
  for(int mode=0;mode<8;++mode) {
    PreviewFixture f;f.measured.position.y()=.06;
    f.during_check=[&]{
      if(mode==0)f.now+=.6;
      if(mode==1)f.budget->cancel();
      if(mode==2)f.steady+=std::chrono::milliseconds(400);
      if(mode==3)GridMapTestAccess::context(*f.map);
      if(mode==4)GridMapTestAccess::revision(*f.map);
      if(mode==5)GridMapTestAccess::source(*f.map,10301000000LL);
      if(mode==6)f.measured.source_stamp+=.001;
      if(mode==7)f.measured.position.y()+=.001; // illegal same-stamp replacement also fenced
    };
    const auto result=f.certifyPreview();EXPECT_FALSE(result.geometry);EXPECT_EQ(f.checks,1);
    EXPECT_EQ(result.outcome,mode==0?CandidateAcceptanceOutcome::PreviewSourceExpired:
        mode<3?CandidateAcceptanceOutcome::PreviewBudgetExhausted:CandidateAcceptanceOutcome::PreviewSourceChanged)<<mode;
  }
}
TEST(PreviewCandidate, OriginalBoundaryPolicyAndStructuredCountersAreNotMotionCertificates) {
  PreviewFixture f;auto origin=f.origin();origin.policy.connector_m=.151;
  EXPECT_FALSE(projectPreviewCandidateGeometry(f.curve,origin,f.measured,.08,f.budget).geometry);
  origin=f.origin();origin.policy.projection_window_m=1.01;EXPECT_FALSE(origin.policy.valid());
  f.measured.source_stamp=f.original.source_stamp;
  EXPECT_EQ(f.certifyPreview().outcome,CandidateAcceptanceOutcome::PreviewSourceChanged);
  CandidateAcceptanceDiagnostics counters;counters.record(CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed);
  EXPECT_EQ(counters.last,CandidateAcceptanceOutcome::PreviewNeedsMeasuredReseed);
  EXPECT_EQ(counters.counts[static_cast<std::size_t>(counters.last)],1u);
  EXPECT_STREQ(candidateAcceptanceReason(counters.last),"preview_candidate_requires_measured_reseed");
  static_assert(!std::is_convertible<PreviewCandidateGeometryEvidence,CandidateJoinEvidence>::value,
      "no-motion geometry must not become motion join evidence");
}

TEST(CandidateAdoption, MovingNineCentimetersJoinsOriginalCurveInsteadOfTZero) {
  Fixture f;const auto controls=f.curve.getControlPoint();const auto knots=f.curve.getKnot();
  EXPECT_FALSE(wholeCurveCollisionFree(f.curve,.08,.2,
      [&](const Eigen::Vector3d &p,double yaw){return f.map->getInflateOccupancy(p,yaw);},
      f.measured,"map",f.now,.5)); // Frozen t=0 production bug.
  const auto result=f.certify();ASSERT_TRUE(result);
  EXPECT_NEAR(result->curve_time,.3,.02);EXPECT_NEAR(result->arc_length,.09,.006);
  EXPECT_TRUE(result->measured.position.isApprox(f.measured.position));
  EXPECT_EQ(result->measured.source_stamp,10.3);EXPECT_TRUE(result->velocity.isApprox(f.velocity));
  EXPECT_FALSE(result->acceleration_valid); // No invented measured acceleration/C2 claim.
  EXPECT_TRUE(f.curve.getControlPoint().isApprox(controls));EXPECT_TRUE(f.curve.getKnot().isApprox(knots));
}

TEST(RollingCandidate, SameMovingInputNoLongerRequiresExactSplinePositionAndVelocity) {
  for(double lateral:{.014532,.03,.08,.10}) {
    Fixture f;GridMapTestAccess::box(*f.map);
    f.measured.position.y()=lateral;f.velocity={.22,.08,0.};
    const auto controls=f.curve.getControlPoint();const auto actual=f.measured;
    ASSERT_FALSE(f.certify());EXPECT_EQ(f.checks,0); // old strict contract, same input
    f.connection=MeasuredConnectionPolicy::RollingCandidateConnection;
    const auto result=f.certify();ASSERT_TRUE(result)<<lateral;
    EXPECT_EQ(f.checks,1);EXPECT_NEAR(result->curve_time,.3,1e-6);
    EXPECT_TRUE(result->measured.position.isApprox(actual.position));
    EXPECT_EQ(result->measured.source_stamp,actual.source_stamp);
    EXPECT_TRUE(result->velocity.isApprox(f.velocity));EXPECT_FALSE(result->acceleration_valid);
    EXPECT_TRUE(f.curve.getControlPoint().isApprox(controls));
  }
}
TEST(RollingCandidate, ActualConnectorObstacleAndRemainderAreStillNativeCollisionQueries) {
  for(int mode=0;mode<2;++mode) {
    Fixture f;GridMapTestAccess::box(*f.map);f.measured.position.y()=.10;
    f.connection=MeasuredConnectionPolicy::RollingCandidateConnection;
    ASSERT_TRUE(f.certify());f.checks=0;
    GridMapTestAccess::raw(*f.map,mode==0?Eigen::Vector3d(.025,.20,.55):Eigen::Vector3d(.85,0.,.55),1);
    if(mode==0)EXPECT_EQ(f.map->getPlanningInflateOccupancy(f.original.position,0.),0);
    EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,1);
    EXPECT_EQ(f.outcome,MotionCandidateAdoptionOutcome::CollisionRejected);
  }
}
TEST(RollingCandidate, NoRemoteBranchWrongFloorChangedIdentityOrInventedSolveOrigin) {
  for(int mode=0;mode<7;++mode) {
    Fixture f;f.connection=MeasuredConnectionPolicy::RollingCandidateConnection;
    if(mode==0)f.measured.position.y()=.151;
    if(mode==1)f.measured.position.z()+=3.;
    if(mode==2)f.measured.position.x()=.7;
    if(mode==3)f.single_segment=false;
    if(mode==4)f.original.position.y()=.02;
    if(mode==5)f.candidate_context=6;
    if(mode==6)f.budget->cancel();
    EXPECT_FALSE(f.certify())<<mode;EXPECT_EQ(f.checks,0)<<mode;
  }
}

TEST(CandidateAdoption, StationaryInputCannotAdvanceWithClock) {
  Fixture f;f.curve=line(0.);f.velocity.setZero();f.measured.position=f.original.position;
  for(int i=0;i<10;++i) {
    const auto result=f.certify();ASSERT_TRUE(result);EXPECT_EQ(result->curve_time,0.);
    EXPECT_EQ(result->arc_length,0.);f.now+=.001;
  }
}
TEST(CandidateAdoption, RejectionDiagnosticDistinguishesJoinSourceCollisionAndBudgetWithoutAdmittingBadCurve) {
  using O=MotionCandidateAdoptionOutcome;
  for(int mode=0;mode<7;++mode) {
    Fixture f;
    if(mode==0)f.velocity.x()=-.3;
    if(mode==1)f.now=11.;
    if(mode==2)f.during_check=[&]{GridMapTestAccess::revision(*f.map);};
    if(mode==3)f.candidate_context=6;
    if(mode==4)GridMapTestAccess::raw(*f.map,{.85,0.,.55},1);
    if(mode==5)f.budget->cancel();
    if(mode==6)f.during_check=[&]{f.steady+=std::chrono::milliseconds(400);};
    EXPECT_FALSE(f.certify())<<mode;
    const auto expected=mode==0?O::MeasuredJoinRejected:mode==1?O::SourceExpired:
      mode==2?O::SourceChanged:mode==3?O::ContextChanged:mode==4?O::CollisionRejected:O::BudgetExhausted;
    EXPECT_EQ(f.outcome,expected)<<mode;
    EXPECT_EQ(f.map->getInflateOccupancy({.85,0.,.55},0.)==1,mode==4);
  }
  Fixture good;ASSERT_TRUE(good.certify());EXPECT_EQ(good.outcome,O::Accepted);
}

TEST(CandidateAdoption, WrongFloorFarLateralAndWrongVelocityCannotJoin) {
  for(int mode=0;mode<4;++mode) {
    Fixture f;
    if(mode==0)f.measured.position.z()+=3.;
    if(mode==1)f.measured.position.x()+=1.;
    if(mode==2)f.measured.position.y()+=.021;
    if(mode==3)f.velocity.x()=-.3;
    EXPECT_FALSE(f.certify())<<mode;EXPECT_EQ(f.checks,0);
  }
}

TEST(CandidateAdoption, EntryCannotSkipSemanticSegmentBoundaryOrContext) {
  Fixture f;f.single_segment=false;EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,0);
  f.single_segment=true;f.candidate_context=6;EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,0);
}

TEST(CandidateAdoption, ArcWindowRejectsLaterOverlappingBranchDespiteNearbyXYZ) {
  Fixture f;
  // A geometrically near branch is not legal progress. The robot only reports
  // 1 ms new source time, but the first compatible point is 9 cm along curve.
  f.measured.source_stamp=10.001;
  EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,0);
}

TEST(CandidateAdoption, RegressedOrOverBudgetSourceCannotLicenseProjection) {
  for(double stamp:{9.99,10.401}) {Fixture f;f.measured.source_stamp=stamp;
    EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,0);}
}

TEST(CandidateAdoption, RealNativeOccupiedAndUnknownRemainderStillBlock) {
  for(int value:{1,2}) {Fixture f;GridMapTestAccess::raw(*f.map,{.65,0.,.55},value);
    EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,1);}
}

TEST(CandidateAdoption, CollisionBehindEntryIsStillCheckedConservatively) {
  Fixture f;GridMapTestAccess::raw(*f.map,{-.13,0.,.55},1);
  EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,1);
}

TEST(CandidateAdoption, FullCurveVelocityAndAccelerationBoundsAreNotRelaxed) {
  Fixture f;f.curve=line(.31);f.velocity.x()=.31;EXPECT_FALSE(f.certify());
  f.curve=line();auto controls=f.curve.getControlPoint();controls(1,9)+=.5;
  f.curve=UniformBspline(controls,3,.2);EXPECT_FALSE(f.certify());
}

TEST(CandidateAdoption, FreshAtStartButExpiredMapAtEndCannotCommit) {
  Fixture f;GridMapTestAccess::source(*f.map,9850000000LL); // age .46 s initially
  f.during_check=[&]{f.now+=.10;};
  EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,1); // Same clear geometry, crossed TTL.
}

TEST(CandidateAdoption, FreshAtStartButExpiredBodyAtEndCannotCommit) {
  Fixture f;f.body_max_age=.05;f.during_check=[&]{f.now+=.10;};
  EXPECT_FALSE(f.certify());EXPECT_EQ(f.checks,1);
}

TEST(CandidateAdoption, SourceContextRevisionAndBodyReplacementDuringCheckCannotCommit) {
  for(int mode=0;mode<4;++mode) {
    Fixture f;f.during_check=[&]{
      if(mode==0)GridMapTestAccess::source(*f.map,10301000000LL);
      if(mode==1)GridMapTestAccess::context(*f.map);
      if(mode==2)GridMapTestAccess::revision(*f.map);
      if(mode==3)f.measured.source_stamp+=.001;
    };
    EXPECT_FALSE(f.certify())<<mode;EXPECT_EQ(f.checks,1);
  }
}

TEST(CandidateAdoption, CanceledOrFourHundredMillisecondLateCheckCannotCommit) {
  for(int mode=0;mode<2;++mode) {
    Fixture f;f.during_check=[&]{if(mode==0)f.budget->cancel();
      else f.steady+=std::chrono::milliseconds(400);};
    EXPECT_FALSE(f.certify())<<mode;EXPECT_EQ(f.checks,1);
  }
}

TEST(WorkerLifecycle, ActualSnapshotLeaseReleasedOnNormalAndExceptionalWorkerExit) {
  for(int mode=0;mode<3;++mode) {
    Fixture f;CollisionSnapshotPool pool;
    ASSERT_TRUE(pool.publish(*f.map,10310000000LL,SolveBudget::Clock::now()));
    auto snapshot=pool.borrowLatest();ASSERT_TRUE(snapshot);
    auto future=std::async(std::launch::async,[&,borrowed=snapshot]() mutable {
      SolveScopeExit release([&]() noexcept {borrowed.reset();});
      if(mode==1)throw std::runtime_error("optimizer");
      if(mode==2)throw std::bad_alloc{};
    });
    snapshot.reset();
    try {future.get();} catch(const std::exception &) {}
    EXPECT_TRUE(pool.borrowLatest()); // Real slot cannot re-lease while still held.
  }
}
