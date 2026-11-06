#include <gtest/gtest.h>
#include <bspline_opt/soft_clearance.hpp>
#include <bspline_opt/whole_spline_collision.hpp>
#include <bspline_opt/trajectory_timing.hpp>
#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <algorithm>
#include <iostream>
#include <thread>

// This fixture is the production GridMap and its native OBB query. Only the
// static synthetic scene is generated here; no substitute occupancy model.
struct GridMapTestAccess {
  static GridMap::Ptr make() {
    auto map=std::make_shared<GridMap>();auto& p=map->mp_;auto& d=map->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={128,96,40};p.map_origin_idx_.setZero();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    p.require_observed_free_=false;p.planning_collision_policy_="official";
    p.collision_model_="oriented_box";p.box_length_=1.03;p.box_width_=.40;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.25;
    const std::size_t size=128U*96U*40U;
    d.occupancy_buffer_.assign(size,-1.);d.occupancy_buffer_inflate_.assign(size,0);
    d.occupancy_buffer_inflate_cnt_.assign(size,0);
    map->updateMapBoundaryFromIndex();map->rebuildInflationOffsets();return map;
  }
  static auto raw(const GridMap& map){return map.md_.occupancy_buffer_;}
  static void diagnostic(GridMap& map){++map.unknown_collision_queries_;}
};
namespace scan_planner {
struct BsplineOptimizerTestAccess {
  static void configure(BsplineOptimizer& optimizer) {
    optimizer.soft_clearance_={.15,30,1.};optimizer.preferred_turn_radius_=1.;
  }
};
}
namespace {
using namespace scan_planner;
UniformBspline straight(double y=.30,double yaw=0.) {
  Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(3,24);
  const Eigen::Matrix2d rotation=Eigen::Rotation2Dd(yaw).toRotationMatrix();
  for(int i=0;i<24;++i){controls.col(i).head<2>()=rotation*Eigen::Vector2d(-1.5+.13*i,y);controls(2,i)=.5;}
  return UniformBspline(controls,3,.65);
}
UniformBspline inwardBend(double yaw=0.) {
  auto controls=straight(.45).getControlPoint();
  for(int i=3;i<controls.cols()-3;++i)
    controls(1,i)-=.15*std::pow(std::sin(M_PI*(i-2)/(controls.cols()-5)),2);
  const Eigen::Matrix2d rotation=Eigen::Rotation2Dd(yaw).toRotationMatrix();
  controls.topRows(2)=rotation*controls.topRows(2);
  return UniformBspline(controls,3,.65);
}
void wall(GridMap& map,double y,double yaw=0.) {
  const Eigen::Matrix2d rotation=Eigen::Rotation2Dd(yaw).toRotationMatrix();
  for(int x=-48;x<=48;++x)for(int z=2;z<=14;++z) {
    const Eigen::Vector2d xy=rotation*Eigen::Vector2d((x+.5)*.05,y);
    map.setOccupied({xy.x(),xy.y(),(z+.5)*.05});
  }
}
SoftClearanceQuery native(const GridMap::Ptr& map){return [map](const Eigen::Vector3d& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);};}
bool clear(UniformBspline& curve,const GridMap::Ptr& map,const SolveBudget::Ptr& budget={}) {
  return checkWholeSplineCollision(curve,.05,[&](const Eigen::Vector3d& p,double yaw){
      return solveAllowed(budget)?map->getPlanningInflateOccupancy(p,yaw):-1;
    }).state==SplineCollisionState::Clear;
}
SoftClearanceAdmission admission(const GridMap::Ptr& map){return [map](UniformBspline& curve,const SolveBudget::Ptr& budget){
  auto velocity=curve.getDerivative();
  return derivativeControlBound(velocity)<=.30 && derivativeControlBound(velocity.getDerivative())<=.35 && clear(curve,map,budget);
};}
SoftClearanceOptions options(){return {.15,30,1.};}
}

TEST(SoftClearance, CollisionFreeWallHuggingCurveGainsNativeSideMarginAfterFullAdmission) {
  auto map=GridMapTestAccess::make();wall(*map,.025);
  auto curve=inwardBend();const auto original=curve;const auto controls=curve.getControlPoint();const auto knots=curve.getKnot();
  const auto raw=GridMapTestAccess::raw(*map);ASSERT_TRUE(clear(curve,map));
  const auto report=improveSoftClearance(curve,.05,options(),{}, {},native(map),admission(map));
  EXPECT_TRUE(report.changed)<<report.reason;EXPECT_LT(report.final_penalty,report.initial_penalty);
  EXPECT_GT(curve.evaluateDeBoorT(curve.getTimeSum()*.5).y(),.32);
  EXPECT_EQ(curve.getControlPoint().leftCols(3),controls.leftCols(3));
  EXPECT_EQ(curve.getControlPoint().rightCols(3),controls.rightCols(3));
  EXPECT_EQ(curve.getControlPoint().row(2),controls.row(2));EXPECT_EQ(curve.getKnot(),knots);
  EXPECT_LE(report.maximum_shift,.15);EXPECT_LE(report.final_curvature_penalty,report.initial_curvature_penalty+1e-7);
  EXPECT_TRUE(clear(curve,map));EXPECT_EQ(GridMapTestAccess::raw(*map),raw);
  EXPECT_LE(report.queries,768U);
  EXPECT_GE(report.final_minimum_side_margin,report.initial_minimum_side_margin);
  auto old=original;auto old_v=old.getDerivative(),new_v=curve.getDerivative();
  EXPECT_NEAR((old_v.evaluateDeBoorT(0.)-new_v.evaluateDeBoorT(0.)).norm(),0.,1e-12);
  EXPECT_NEAR((old_v.evaluateDeBoorT(old.getTimeSum())-new_v.evaluateDeBoorT(curve.getTimeSum())).norm(),0.,1e-12);
}

TEST(SoftClearance, RotatedCorridorUsesPathNormalRatherThanWorldYAxis) {
  auto map=GridMapTestAccess::make();wall(*map,.025,M_PI*.5);auto curve=inwardBend(M_PI*.5);
  ASSERT_TRUE(clear(curve,map));
  const auto report=improveSoftClearance(curve,.05,options(),{}, {},native(map),admission(map));
  EXPECT_TRUE(report.changed)<<report.reason;EXPECT_LT(curve.evaluateDeBoorT(curve.getTimeSum()*.5).x(),-.32);
}

TEST(SoftClearance, SlopedSupportMapsCandidateAndSideProbesBeforeNativeAdmission) {
  auto map=GridMapTestAccess::make();wall(*map,.025);
  auto controls=inwardBend().getControlPoint();
  for(int i=0;i<controls.cols();++i)controls(2,i)=.5+.4*controls(1,i);
  UniformBspline curve(controls,3,.65);ASSERT_TRUE(clear(curve,map));
  BsplineOptimizer optimizer;BsplineOptimizerTestAccess::configure(optimizer);optimizer.setEnvironment(map);
  int samples=0;
  optimizer.setSupportHeightSampler([&](const Eigen::Vector3d& p)->std::optional<SupportHeightSample>{
    ++samples;return SupportHeightSample{.5+.4*p.y(),{0.,.4}};
  });
  bool admitted=false;const auto raw=GridMapTestAccess::raw(*map);
  const auto report=optimizer.improveTrajectoryClearance(curve,
      [&](UniformBspline& candidate,const SolveBudget::Ptr& budget) {
        admitted=true;
        for(int i=0;i<=200;++i) {
          const auto p=candidate.evaluateDeBoorT(candidate.getTimeSum()*i/200.);
          EXPECT_NEAR(p.z(),.5+.4*p.y(),1e-12);
        }
        EXPECT_EQ(candidate.getControlPoint().leftCols(3),controls.leftCols(3));
        EXPECT_EQ(candidate.getControlPoint().rightCols(3),controls.rightCols(3));
        return admission(map)(candidate,budget);
      });
  EXPECT_TRUE(report.changed)<<report.reason;EXPECT_TRUE(admitted);EXPECT_GT(samples,controls.cols());
  EXPECT_GT((curve.getControlPoint().row(2)-controls.row(2)).norm(),1e-4);
  EXPECT_EQ(GridMapTestAccess::raw(*map),raw);EXPECT_TRUE(clear(curve,map));
}

TEST(SoftClearance, MixedLandingOptionalAdjustmentKeepsAlreadyAcceptedBaseline) {
  auto map=GridMapTestAccess::make();wall(*map,.025);auto curve=inwardBend();
  const auto before=curve.getControlPoint();BsplineOptimizer optimizer;
  BsplineOptimizerTestAccess::configure(optimizer);optimizer.setEnvironment(map);
  optimizer.setSupportHeightSampler([](const Eigen::Vector3d& p)->std::optional<SupportHeightSample>{
    return p.x()<0.?SupportHeightSample{.5+.4*p.x(),{.4,0.}}:SupportHeightSample{.5,{0.,0.}};
  });
  const auto report=optimizer.improveTrajectoryClearance(curve,admission(map));
  EXPECT_FALSE(report.changed);EXPECT_EQ(report.reason,"mixed_support_baseline_preserved");
  EXPECT_EQ(curve.getControlPoint(),before);EXPECT_TRUE(clear(curve,map));
}

TEST(SoftClearance, OpenSpaceAndGroundDoNotCreateArtificialRepulsionOrRecentering) {
  auto map=GridMapTestAccess::make();
  for(int x=-42;x<=42;++x)for(int y=-30;y<=30;++y)map->setOccupied({(x+.5)*.05,(y+.5)*.05,.025});
  auto curve=straight(.45);const auto before=curve.getControlPoint();
  const auto report=improveSoftClearance(curve,.05,options(),{}, {},native(map),admission(map));
  EXPECT_FALSE(report.changed);EXPECT_EQ(report.reason,"already_comfortable");EXPECT_EQ(curve.getControlPoint(),before);
}

TEST(SoftClearance, SymmetricNarrowPassageKeepsValidBaselineWithoutRequiringExtraMargin) {
  auto map=GridMapTestAccess::make();wall(*map,-.325);wall(*map,.325);auto curve=straight(0.);
  const auto before=curve.getControlPoint();ASSERT_TRUE(clear(curve,map));
  const auto report=improveSoftClearance(curve,.05,options(),{}, {},native(map),admission(map));
  EXPECT_FALSE(report.changed);EXPECT_EQ(curve.getControlPoint(),before);EXPECT_TRUE(clear(curve,map));
  EXPECT_GT(report.initial_penalty,0.);
}

TEST(SoftClearance, AsymmetricNarrowPassagePreservesBaselineIfNoSmootherClearanceImprovement) {
  auto map=GridMapTestAccess::make();wall(*map,-.375);wall(*map,.325);auto curve=straight(.075);
  const auto before=curve.getControlPoint();
  ASSERT_TRUE(clear(curve,map));
  const auto report=improveSoftClearance(curve,.05,options(),{}, {},native(map),admission(map));
  EXPECT_TRUE(report.attempted);EXPECT_FALSE(report.changed);EXPECT_EQ(curve.getControlPoint(),before);
  EXPECT_TRUE(clear(curve,map));EXPECT_EQ(report.final_penalty,report.initial_penalty);
  std::cout<<"narrow_optional_decline,penalty="<<report.initial_penalty<<",candidates="<<report.candidates
      <<",curvature="<<report.curvature_rejections<<",clearance="<<report.clearance_rejections
      <<",admission="<<report.admission_rejections<<'\n';
}

TEST(SoftClearance, DeclinedDynamicOrPrefixAdmissionPreservesOriginalTrajectoryExactly) {
  auto map=GridMapTestAccess::make();wall(*map,.025);auto curve=inwardBend();const auto before=curve.getControlPoint();
  int admissions=0;const auto report=improveSoftClearance(curve,.05,options(),{}, {},native(map),
    [&](UniformBspline&,const SolveBudget::Ptr&){++admissions;return false;});
  EXPECT_GT(admissions,0);EXPECT_FALSE(report.changed);EXPECT_EQ(curve.getControlPoint(),before);
}

TEST(SoftClearance, DisabledAndInsufficientParentBudgetKeepBaselineWithoutQuerying) {
  auto map=GridMapTestAccess::make();wall(*map,.025);auto curve=straight();const auto before=curve.getControlPoint();
  auto disabled=options();disabled.distance=0.;
  auto report=improveSoftClearance(curve,.05,disabled,{}, {},native(map),admission(map));
  EXPECT_EQ(report.queries,0U);EXPECT_FALSE(report.attempted);
  auto parent=std::make_shared<SolveBudget>(std::chrono::milliseconds(40));
  report=improveSoftClearance(curve,.05,{.15,30,1.},{},parent,native(map),admission(map));
  EXPECT_EQ(report.queries,0U);EXPECT_EQ(report.reason,"publication_budget_reserved");
  EXPECT_TRUE(parent->allowed());EXPECT_EQ(curve.getControlPoint(),before);
}

TEST(SoftClearance, QualityDeadlineDoesNotExpireParentOrDiscardSuccessfulBaseline) {
  auto map=GridMapTestAccess::make();wall(*map,.025);auto curve=straight();const auto before=curve.getControlPoint();
  auto parent=std::make_shared<SolveBudget>(std::chrono::milliseconds(400));
  auto slow=[&](const Eigen::Vector3d& p,double yaw){std::this_thread::sleep_for(std::chrono::milliseconds(2));return native(map)(p,yaw);};
  SoftClearanceWorkContext context;context.remaining_source=[]{return .4;};
  const auto report=improveSoftClearance(curve,.05,{.15,5,1.},{},parent,slow,admission(map),context);
  EXPECT_FALSE(report.changed);EXPECT_TRUE(parent->allowed());EXPECT_EQ(curve.getControlPoint(),before);
  EXPECT_EQ(report.reason,"quality_deadline"); // Constant fresh source is not the expired resource.
  EXPECT_LT(report.elapsed_ms,30.);
}

TEST(SoftClearance, ParentCancellationCannotPromoteTheOptionalCandidate) {
  auto map=GridMapTestAccess::make();wall(*map,.025);auto curve=straight();const auto before=curve.getControlPoint();
  auto parent=std::make_shared<SolveBudget>(std::chrono::milliseconds(400));
  int count=0;const auto report=improveSoftClearance(curve,.05,options(),{},parent,
    [&](const Eigen::Vector3d& p,double yaw){if(++count==6)parent->cancel();return native(map)(p,yaw);},admission(map));
  EXPECT_FALSE(report.changed);EXPECT_FALSE(parent->allowed());EXPECT_EQ(curve.getControlPoint(),before);
}

TEST(SoftClearance, ExistingOwnerAdoptionAndPublicationReserveTakesPriorityOverQuality) {
  auto map=GridMapTestAccess::make();wall(*map,.025);auto curve=inwardBend();
  const auto before=curve.getControlPoint();
  auto parent=std::make_shared<SolveBudget>(std::chrono::milliseconds(60));
  SoftClearanceWorkContext context;context.completion_reserve_s=.120+.020;
  context.remaining_source=[]{return .4;};
  const auto result=improveSoftClearance(curve,.05,options(),{},parent,native(map),admission(map),context);
  EXPECT_FALSE(result.attempted);EXPECT_FALSE(result.changed);EXPECT_EQ(result.queries,0U);
  EXPECT_EQ(result.reason,"publication_budget_reserved");EXPECT_TRUE(parent->allowed());
  EXPECT_EQ(curve.getControlPoint(),before);EXPECT_TRUE(clear(curve,map));
}

TEST(SoftClearance, OldSolveSourceDoesNotSpendAnOtherwiseAvailableParentBudget) {
  auto map=GridMapTestAccess::make();wall(*map,.025);auto curve=inwardBend();
  const auto before=curve.getControlPoint();
  auto parent=std::make_shared<SolveBudget>(std::chrono::milliseconds(400));
  for(double remaining:{.02,.169,std::numeric_limits<double>::quiet_NaN()}) {
    SoftClearanceWorkContext context;context.completion_reserve_s=.140;
    context.remaining_source=[=]{return remaining;};
    const auto result=improveSoftClearance(curve,.05,options(),{},parent,native(map),admission(map),context);
    EXPECT_FALSE(result.attempted);EXPECT_FALSE(result.changed);EXPECT_EQ(result.queries,0U);
    EXPECT_EQ(result.reason,"source_publication_reserved");EXPECT_TRUE(parent->allowed());
    EXPECT_EQ(curve.getControlPoint(),before);
  }
}

TEST(SoftClearance, FirstSourceReadConsumesQualityBudgetAndCannotStartExtraWorkAfterBlocking) {
  auto map=GridMapTestAccess::make();wall(*map,.025);auto curve=inwardBend();
  const auto before=curve.getControlPoint();
  auto parent=std::make_shared<SolveBudget>(std::chrono::milliseconds(400));
  SoftClearanceWorkContext context;context.completion_reserve_s=.140;
  int reads=0,queries=0,admissions=0;
  context.remaining_source=[&] {
    ++reads;std::this_thread::sleep_for(std::chrono::milliseconds(35));return .4;
  };
  const auto result=improveSoftClearance(curve,.05,options(),{},parent,
      [&](const Eigen::Vector3d& p,double yaw){++queries;return map->getPlanningInflateOccupancy(p,yaw);},
      [&](UniformBspline&,const SolveBudget::Ptr&){++admissions;return true;},context);
  EXPECT_FALSE(result.attempted);EXPECT_FALSE(result.changed);EXPECT_EQ(result.reason,"quality_deadline");
  EXPECT_EQ(reads,1);EXPECT_EQ(queries,0);EXPECT_EQ(admissions,0);EXPECT_EQ(result.queries,0U);
  EXPECT_TRUE(parent->allowed());EXPECT_EQ(curve.getControlPoint(),before);EXPECT_TRUE(clear(curve,map));
  EXPECT_GE(result.elapsed_ms,30.);
}

TEST(SoftClearance, SourceClockRollbackOrAdvanceIntoReservedWindowOnlyAbandonsOptionalWork) {
  for(bool rollback:{false,true}) {
    auto map=GridMapTestAccess::make();wall(*map,.025);auto curve=inwardBend();
    const auto before=curve.getControlPoint();
    auto parent=std::make_shared<SolveBudget>(std::chrono::milliseconds(400));
    double remaining=.4;int queries=0;
    SoftClearanceWorkContext context;context.completion_reserve_s=.140;
    context.remaining_source=[&]{return remaining;};
    const auto query=[&](const Eigen::Vector3d& p,double yaw) {
      if(++queries==3)remaining=rollback?.41:.13;
      return map->getPlanningInflateOccupancy(p,yaw);
    };
    const auto result=improveSoftClearance(curve,.05,options(),{},parent,query,admission(map),context);
    EXPECT_FALSE(result.changed);EXPECT_EQ(result.reason,"source_publication_reserved");
    EXPECT_EQ(queries,3);EXPECT_TRUE(parent->allowed());EXPECT_EQ(curve.getControlPoint(),before);
    EXPECT_TRUE(clear(curve,map));
  }
}

TEST(SoftClearance, PausedSourceClockStillAllowsBoundedWorkAndRechecksAfterAdmission) {
  for(bool expire_in_admission:{false,true}) {
    auto map=GridMapTestAccess::make();wall(*map,.025);auto curve=inwardBend();
    const auto before=curve.getControlPoint();
    auto parent=std::make_shared<SolveBudget>(std::chrono::milliseconds(400));
    double remaining=.4;int calls=0;
    SoftClearanceWorkContext context;context.completion_reserve_s=.140;
    context.remaining_source=[&]{return remaining;};
    const auto check=[&](UniformBspline& candidate,const SolveBudget::Ptr& budget) {
      ++calls;const bool result=admission(map)(candidate,budget);
      if(expire_in_admission)remaining=.13;
      return result;
    };
    const auto result=improveSoftClearance(curve,.05,options(),{},parent,native(map),check,context);
    EXPECT_GT(calls,0);EXPECT_TRUE(parent->allowed());EXPECT_TRUE(clear(curve,map));
    if(expire_in_admission) {
      EXPECT_FALSE(result.changed);EXPECT_EQ(result.reason,"source_publication_reserved");
      EXPECT_EQ(curve.getControlPoint(),before);
    } else EXPECT_TRUE(result.changed)<<result.reason;
  }
}

TEST(SoftClearance, AlternatingOneCellWallEvidenceDoesNotSteerASymmetricCorridorBaseline) {
  for(double sign:{-1.,1.}) {
    auto map=GridMapTestAccess::make();wall(*map,-.375);wall(*map,.375);
    for(int x=-3;x<=2;++x)for(int z=2;z<=14;++z)
      map->setOccupied({(x+.5)*.05,sign*.325,(z+.5)*.05});
    auto curve=straight(0.);const auto before=curve.getControlPoint();ASSERT_TRUE(clear(curve,map));
    SplineHeadingContract heading;heading.measured_heading_enabled=true;heading.align_then_forward=true;
    const auto result=improveSoftClearance(curve,.05,options(),heading,{},native(map),admission(map));
    EXPECT_FALSE(result.changed)<<result.reason;EXPECT_EQ(curve.getControlPoint(),before);
    EXPECT_TRUE(clear(curve,map));EXPECT_EQ(result.reason,"balanced_or_fixed_boundary");
  }
}

TEST(SoftClearance, NativeOccupiedInputCannotBeReclassifiedByComfortPass) {
  auto map=GridMapTestAccess::make();wall(*map,.30);auto curve=straight();const auto before=curve.getControlPoint();
  ASSERT_FALSE(clear(curve,map));const auto report=improveSoftClearance(curve,.05,options(),{}, {},native(map),admission(map));
  EXPECT_FALSE(report.changed);EXPECT_EQ(curve.getControlPoint(),before);EXPECT_FALSE(clear(curve,map));
}

TEST(SoftClearance, FormalThirtyMillisecondBudgetRepeatedNativeImprovement) {
  std::vector<double> elapsed;
  for(int run=0;run<30;++run) {
    auto map=GridMapTestAccess::make();wall(*map,.025);
    auto curve=inwardBend();auto parent=std::make_shared<SolveBudget>(std::chrono::milliseconds(400));
    SplineHeadingContract heading;heading.measured_heading_enabled=true;heading.align_then_forward=true;
    const auto report=improveSoftClearance(curve,.05,options(),heading,parent,native(map),admission(map));
    EXPECT_TRUE(report.changed)<<"run="<<run<<" reason="<<report.reason;
    EXPECT_TRUE(parent->allowed());EXPECT_TRUE(clear(curve,map));elapsed.push_back(report.elapsed_ms);
  }
  std::sort(elapsed.begin(),elapsed.end());
  RecordProperty("formal_budget_ms",30);RecordProperty("repeat_count",30);
  RecordProperty("p50_ms",std::to_string(elapsed[14]));RecordProperty("p95_ms",std::to_string(elapsed[28]));RecordProperty("max_ms",std::to_string(elapsed.back()));
  std::cout<<"soft_clearance_native_30ms,n=30,p50="<<elapsed[14]<<",p95="<<elapsed[28]<<",max="<<elapsed.back()<<'\n';
}

TEST(SoftClearance, OptimizerWrapperDoesNotLeakOptionalProbeDiagnostics) {
  auto map=GridMapTestAccess::make();wall(*map,.025);auto curve=inwardBend();const auto before=curve.getControlPoint();
  BsplineOptimizer optimizer;optimizer.setEnvironment(map);BsplineOptimizerTestAccess::configure(optimizer);
  int admissions=0;
  const auto report=optimizer.improveTrajectoryClearance(curve,[&](UniformBspline&,const SolveBudget::Ptr&){
    ++admissions;GridMapTestAccess::diagnostic(*map);return false;
  });
  EXPECT_GT(admissions,0);EXPECT_FALSE(report.changed);EXPECT_EQ(curve.getControlPoint(),before);
  EXPECT_EQ(map->unknownCollisionQueries(),0U);
}

TEST(SoftClearance, ComfortUsesFrozenMeasuredLowSpeedAndReverseEntryHeading) {
  Eigen::MatrixXd q=Eigen::MatrixXd::Zero(3,12);q.row(2).setConstant(.5);
  q.row(0)<<-.05,-.10,-.15,-.14,-.10,0.,.15,.30,.45,.60,.75,.90;
  for(bool low_speed:{false,true}) {
    UniformBspline curve(q,3,low_speed?25.:.65);auto velocity=curve.getDerivative();
    SplineHeadingContract input;input.measured_heading_enabled=true;
    input.measured_yaw=0.;input.align_then_forward=!low_speed;
    auto expected=input;
    const auto intervals=bodyHeadingSamplingIntervals(curve.getTimeSum(),derivativeControlBound(velocity),.05);
    ASSERT_TRUE(intervals);ASSERT_TRUE(resolveForwardEntryHeading(expected,curve.getTimeSum(),*intervals,
      [&](double t){return curve.evaluateDeBoorT(t);},[&](double t){return velocity.evaluateDeBoorT(t);},[]{return true;}));
    if(!low_speed){ASSERT_GT(expected.forward_entry_end,0.);}
    std::vector<std::pair<Eigen::Vector3d,double>> probes;
    const auto report=improveSoftClearance(curve,.05,options(),input,{},
      [&](const Eigen::Vector3d& p,double yaw){probes.emplace_back(p,yaw);return 0;},
      [](UniformBspline&,const SolveBudget::Ptr&){return true;});
    ASSERT_EQ(report.reason,"already_comfortable");
    for(int i=0;i<9;++i) {
      const double time=curve.getTimeSum()*(i+1)/10.;const auto position=curve.evaluateDeBoorT(time);
      const auto yaw=sampledBodyHeadingAt(time,curve.getTimeSum(),*intervals,expected,
          [&](double t){return velocity.evaluateDeBoorT(t);},[]{return true;});
      ASSERT_TRUE(yaw);bool found=false;
      for(const auto& probe:probes)if((probe.first-position).norm()<1e-10) {
        EXPECT_NEAR(std::remainder(probe.second-*yaw,2.*M_PI),0.,1e-12);found=true;break;
      }
      EXPECT_TRUE(found);
    }
    EXPECT_DOUBLE_EQ(input.forward_entry_end,-1.);EXPECT_FALSE(input.forward_entry_profile);
  }
}

TEST(SoftClearance, StationaryAlignEntryReplaysTheFormalNearCurveHeadingSeed) {
  auto q=straight(.3,M_PI*.25).getControlPoint();q.topRows(2)*=.03;
  UniformBspline curve(q,3,.65);SplineHeadingContract input;
  input.measured_heading_enabled=true;input.align_then_forward=true;input.measured_yaw=-.4;
  auto velocity=curve.getDerivative();ASSERT_LT(derivativeControlBound(velocity),input.minimum_direction_speed);
  std::size_t probes=0;
  const auto report=improveSoftClearance(curve,.05,options(),input,{},
      [&](const Eigen::Vector3d&,double yaw){++probes;EXPECT_NEAR(yaw,M_PI*.25,1e-10);return 0;},
      [](UniformBspline&,const SolveBudget::Ptr&){return true;});
  EXPECT_EQ(report.reason,"already_comfortable");EXPECT_GT(probes,0U);
  EXPECT_DOUBLE_EQ(input.measured_yaw,-.4);EXPECT_DOUBLE_EQ(input.forward_entry_end,-1.);
}

TEST(SoftClearance, FormalBodyDoesNotTradeWorstSideMarginForAverageComfort) {
  auto map=GridMapTestAccess::make();wall(*map,.025);auto curve=straight(.35);
  const auto before=curve.getControlPoint();SplineHeadingContract heading;
  heading.measured_heading_enabled=true;heading.align_then_forward=true;
  const auto report=improveSoftClearance(curve,.05,options(),heading,{},native(map),admission(map));
  EXPECT_FALSE(report.changed);EXPECT_EQ(curve.getControlPoint(),before);
  EXPECT_EQ(report.final_minimum_side_margin,report.initial_minimum_side_margin);
  EXPECT_TRUE(clear(curve,map));EXPECT_GT(report.clearance_rejections,0U);
}
