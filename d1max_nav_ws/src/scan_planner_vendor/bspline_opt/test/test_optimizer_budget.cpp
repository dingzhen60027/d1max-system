#include <gtest/gtest.h>
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/trajectory_timing.hpp>
#include <bspline_opt/reference_path.hpp>
#include <bspline_opt/whole_spline_collision.hpp>
#include "rounding_retry_fixture.hpp"
#include "refine_rounding_fixture.hpp"

// Actual optimizer/production GridMap, no ROS initialization or map surrogate.
struct GridMapTestAccess {
  static GridMap::Ptr make(bool strict=true) {
    auto map=std::make_shared<GridMap>();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={160,80,64};
    p.map_origin_idx_.setZero();map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.clamp_max_log_=2.;p.min_occupancy_log_=1.;p.unknown_flag_=.01;
    p.require_observed_free_=true;p.planning_collision_policy_=strict?"observed_free":"official";
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.45;
    const std::size_t size=160U*80U*64U;
    d.occupancy_buffer_.assign(size,-1.);d.occupancy_buffer_inflate_.assign(size,0);
    d.occupancy_buffer_inflate_cnt_.assign(size,0);map->rebuildInflationOffsets();return map;
  }
  static void unknown(GridMap &map,const Eigen::Vector3d &point) {
    Eigen::Vector3i cell;map.posToIndex(point,cell);
    map.md_.occupancy_buffer_[map.toAddress(cell)]=-1.01;
    map.observed_cylinder_cache_.clear();++map.occupancy_revision_;
  }
  static void allUnknown(GridMap &map) {
    std::fill(map.md_.occupancy_buffer_.begin(),map.md_.occupancy_buffer_.end(),-1.01);
    map.observed_cylinder_cache_.clear();++map.occupancy_revision_;
  }
  static GridMap::Ptr productionBox() {
    auto map=make();auto &p=map->mp_;
    p.collision_model_="oriented_box";p.box_length_=1.03;p.box_width_=.4;
    p.body_below_=p.body_above_=.25;
    p.planning_body_below_=p.planning_body_above_=.25;
    map->rebuildInflationOffsets();return map;
  }
  static GridMap::Ptr roundingFixtureBox() {
    auto map=make(false);auto &p=map->mp_;auto &d=map->md_;
    p.map_voxel_num_={600,80,60};p.map_origin_idx_={0,0,10};
    map->updateMapBoundaryFromIndex();p.require_observed_free_=false;
    p.collision_model_="oriented_box";p.box_length_=.93;p.box_width_=.30;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    const std::size_t count=600U*80U*60U;
    d.occupancy_buffer_.assign(count,-1.);d.occupancy_buffer_inflate_.assign(count,0);
    d.occupancy_buffer_inflate_cnt_.assign(count,0);map->rebuildInflationOffsets();return map;
  }
};
namespace scan_planner {
struct BsplineOptimizerTestAccess {
  static void setup(BsplineOptimizer &optimizer,const GridMap::Ptr &map) {
    optimizer.order_=3;optimizer.lambda1_=1.;optimizer.lambda2_=1.;
    optimizer.lambda3_=1.;optimizer.lambda4_=1.;optimizer.dist0_=.2;
    optimizer.max_vel_=1.;optimizer.max_acc_=1.;optimizer.setEnvironment(map);
    optimizer.a_star_=std::make_shared<AStar>();optimizer.a_star_->initGridMap(map,{80,40,32});
  }
  static void searchFailure(BsplineOptimizer &optimizer,ASTAR_RET result) {
    optimizer.recordSearchFailure(result);
  }
  static bool checkRebound(BsplineOptimizer &optimizer) {
    return optimizer.check_collision_and_rebound();
  }
  static void motionLimits(BsplineOptimizer &optimizer,double speed,double acceleration) {
    optimizer.max_vel_=speed;optimizer.max_acc_=acceleration;
  }
  static bool actualClear(BsplineOptimizer &optimizer,const Eigen::MatrixXd &points) {
    return optimizer.actualStairCurveClear(points);
  }
  static void configureRoundingFixture(BsplineOptimizer &optimizer,const GridMap::Ptr &map) {
    setup(optimizer,map);optimizer.lambda3_=.1;optimizer.lambda_reference_=1.;
    optimizer.lambda_planar_curvature_=.2;optimizer.preferred_turn_radius_=1.;
    optimizer.dist0_=.28;optimizer.max_vel_=.3;optimizer.max_acc_=.35;
  }
  struct RefineStep {
    int code;
    Eigen::MatrixXd accepted, callback_trial;
  };
  static RefineStep refineStep(BsplineOptimizer& optimizer,
      const Eigen::MatrixXd& initial, double dt, bool corrected) {
    optimizer.setControlPoints(initial);
    optimizer.setBsplineInterval(dt);
    optimizer.iter_num_=0;
    optimizer.force_stop_type_=BsplineOptimizer::DONT_STOP;
    const int n=3*(initial.cols()-6);
    Eigen::VectorXd q=Eigen::Map<const Eigen::VectorXd>(initial.data()+9,n);
    lbfgs::lbfgs_parameter_t parameters;
    lbfgs::lbfgs_load_default_parameters(&parameters);
    parameters.mem_size=16;parameters.max_iterations=200;parameters.g_epsilon=.001;
    parameters.corrected_cubic_bound=corrected;
    double cost=0.;
    const int code=lbfgs::lbfgs_optimize(n,q.data(),&cost,
        BsplineOptimizer::costFunctionRefine,nullptr,BsplineOptimizer::earlyExit,
        &optimizer,&parameters);
    Eigen::MatrixXd accepted=initial;
    Eigen::Map<Eigen::VectorXd>(accepted.data()+9,n)=q;
    return {code,accepted,optimizer.cps_.points};
  }
};
}
namespace {
Eigen::MatrixXd straightOptimizerControls() {
  Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(3,8);
  for(int i=0;i<controls.cols();++i)controls(0,i)=i*.3;
  controls.row(2).setConstant(.55);return controls;
}
}

TEST(OptimizerBudget, CancelledNativeRefinementCannotReturnCandidate) {
  scan_planner::BsplineOptimizer optimizer;
  auto budget=std::make_shared<scan_planner::SolveBudget>();budget->cancel();
  optimizer.setSolveBudget(budget);
  Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(3,8),result;
  EXPECT_FALSE(optimizer.BsplineOptimizeTrajRefine(controls,.2,result));
}
namespace {
void configureRefineFixture(scan_planner::BsplineOptimizer& optimizer) {
  scan_planner::BsplineOptimizerTestAccess::configureRoundingFixture(
      optimizer,GridMapTestAccess::productionBox());
  optimizer.reference_path_samples_=refine_rounding_fixture::reference();
  auto curve=scan_planner::UniformBspline(refine_rounding_fixture::controls(),3,
      refine_rounding_fixture::interval);
  optimizer.ref_pts_.clear();
  for(int i=0;i<=refine_rounding_fixture::controls().cols()-3;++i)
    optimizer.ref_pts_.push_back(curve.evaluateDeBoorT(i*refine_rounding_fixture::interval));
}
bool successfulLbfgs(int code) {
  return code==lbfgs::LBFGS_CONVERGENCE||code==lbfgs::LBFGSERR_MAXIMUMITERATION||
      code==lbfgs::LBFGS_ALREADY_MINIMIZED||code==lbfgs::LBFGS_STOP;
}
}
TEST(OptimizerBudget, RefineRoundingUsesAcceptedIterateAndNeverClaimsFailedTrial) {
  using Access=scan_planner::BsplineOptimizerTestAccess;
  const auto initial=refine_rounding_fixture::controls();
  constexpr double dt=refine_rounding_fixture::interval;
  scan_planner::BsplineOptimizer oracle;
  configureRefineFixture(oracle);
  const auto first=Access::refineStep(oracle,initial,dt,false);
  ASSERT_EQ(first.code,lbfgs::LBFGSERR_ROUNDING_ERROR);
  EXPECT_GT((first.callback_trial-first.accepted).norm(),0.);
  const auto retry=Access::refineStep(oracle,first.accepted,dt,true);
  RecordProperty("first_lbfgs_result",first.code);
  RecordProperty("corrected_lbfgs_result",retry.code);
  RecordProperty("rejected_trial_distance",(first.callback_trial-first.accepted).norm());
  scan_planner::BsplineOptimizer actual;
  configureRefineFixture(actual);
  Eigen::MatrixXd result=Eigen::MatrixXd::Constant(3,8,42.);
  const auto saved=result;
  const bool ok=actual.BsplineOptimizeTrajRefine(initial,dt,result);
  EXPECT_EQ(ok,successfulLbfgs(retry.code));
  if(ok) {
    EXPECT_TRUE((result.array()==retry.accepted.array()).all());
    EXPECT_TRUE((result.leftCols(3).array()==initial.leftCols(3).array()).all());
    EXPECT_TRUE((result.rightCols(3).array()==initial.rightCols(3).array()).all());
  } else {
    EXPECT_TRUE((result.array()==saved.array()).all());
    EXPECT_FALSE(actual.lastFailurePhase().empty());
  }
}
TEST(OptimizerBudget, RefineCancellationPreservesPriorOutputAndOriginalDeadline) {
  for(bool cancel:{false,true}) {
    scan_planner::BsplineOptimizer optimizer;
    configureRefineFixture(optimizer);
    auto now=scan_planner::SolveBudget::Clock::now();
    auto budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&]{return now;});
    optimizer.setSolveBudget(budget);
    if(cancel)budget->cancel();else now+=std::chrono::milliseconds(400);
    Eigen::MatrixXd output=Eigen::MatrixXd::Constant(3,8,42.);
    const auto before=output;
    EXPECT_FALSE(optimizer.BsplineOptimizeTrajRefine(refine_rounding_fixture::controls(),
        refine_rounding_fixture::interval,output));
    EXPECT_TRUE((before.array()==output.array()).all());
    EXPECT_FALSE(budget->allowed());
  }
}
TEST(OptimizerBudget, SuccessfulRefineMatchesUnchangedSinglePassForFlatAndBothSlopes) {
  using Access=scan_planner::BsplineOptimizerTestAccess;
  for(double slope:{0.,.2,-.2}) {
    auto initial=straightOptimizerControls();
    initial.row(0).array()+=1.;initial.row(1).setConstant(1.);
    for(int i=0;i<initial.cols();++i)initial(2,i)=1.+slope*i*.3;
    const auto configure=[&](scan_planner::BsplineOptimizer& optimizer) {
      Access::setup(optimizer,GridMapTestAccess::make(false));
      auto curve=scan_planner::UniformBspline(initial,3,1.);
      for(int i=0;i<=initial.cols()-3;++i)optimizer.ref_pts_.push_back(curve.evaluateDeBoorT(i));
    };
    scan_planner::BsplineOptimizer oracle,actual;configure(oracle);configure(actual);
    const auto first=Access::refineStep(oracle,initial,1.,false);
    ASSERT_TRUE(successfulLbfgs(first.code));
    Eigen::MatrixXd output;
    ASSERT_TRUE(actual.BsplineOptimizeTrajRefine(initial,1.,output));
    EXPECT_TRUE((first.accepted.array()==output.array()).all());
  }
}
TEST(OptimizerBudget,RealReboundAndRefinementPreserveMeasuredOverspeedAndRecoverToCruise) {
  for(double speed:{.304981,.416980}) {
    auto map=GridMapTestAccess::make();scan_planner::BsplineOptimizer optimizer;
    scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
    scan_planner::BsplineOptimizerTestAccess::motionLimits(optimizer,.3,.35);
    auto budget=std::make_shared<scan_planner::SolveBudget>();optimizer.setSolveBudget(budget);
    scan_planner::DiscreteReference reference;reference.set({{1.,1.,.55},{3.,1.,.55}});
    const auto seed=scan_planner::sampleReferenceSeed(reference,.3,.35,speed,0.,.2);
    const scan_planner::CubicMotionBoundary boundary{{1.,1.,.55},{speed,0,0},{0,0,0},
        {3.,1.,.55},{0,0,0},{0,0,0}};
    auto controls=scan_planner::fitCubicWithFixedBoundary(seed.samples,seed.dt,boundary);
    optimizer.reference_path_samples_=seed.samples;optimizer.initControlPoints(controls);
    ASSERT_TRUE(optimizer.controlPointsInitialized());
    Eigen::MatrixXd optimized;ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(optimized,seed.dt));
    scan_planner::UniformBspline curve(optimized,3,seed.dt);
    const auto result=scan_planner::refineTimingWithFixedBoundary(curve,boundary,.3,.35,
      [&](Eigen::MatrixXd &q,double dt) {
        scan_planner::UniformBspline ref(q,3,dt);optimizer.ref_pts_.clear();
        for(int i=0;i<=q.cols()-3;++i)optimizer.ref_pts_.push_back(ref.evaluateDeBoorT(i*dt));
        Eigen::MatrixXd out;const bool ok=optimizer.BsplineOptimizeTrajRefine(q,dt,out);
        if(ok)q=out;
        const auto check=scan_planner::inspectInitialMotionLimits({q,3,dt},.3,.35);
        std::cout<<"recovery_refine dt="<<dt<<" nominal="<<check.nominal_speed_bound
                 <<" acc="<<check.acceleration_bound<<" reason="<<check.reason<<std::endl;
        return ok;
      },8,std::nullopt,[&]{return budget->allowed();},true);
    ASSERT_TRUE(result.success)<<result.reason<<" speed="<<speed<<" nominal="<<result.nominal_speed_bound;
    EXPECT_TRUE(scan_planner::cubicBoundaryMatches(curve,boundary));
    const auto limits=scan_planner::inspectInitialMotionLimits(curve,.3,.35);
    EXPECT_TRUE(limits.valid);EXPECT_LE(limits.recovery_end,2.*speed/.35);
  }
}
TEST(OptimizerBudget, ExpiredInitializationDoesNotUseUninitializedEnvironment) {
  scan_planner::BsplineOptimizer optimizer;
  auto now=scan_planner::SolveBudget::Clock::now();
  auto budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&] {return now;});
  now+=std::chrono::milliseconds(400);optimizer.setSolveBudget(budget);
  Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(3,8);
  EXPECT_TRUE(optimizer.initControlPoints(controls).empty());
  EXPECT_FALSE(optimizer.controlPointsInitialized());
}
TEST(OptimizerBudget, StairCubicScopeCannotRenewExpiredOrCancelledBudget) {
  for(bool cancelled:{false,true}) {
    auto map=GridMapTestAccess::make();scan_planner::BsplineOptimizer optimizer;
    scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
    auto previous=map->exchangeQuerySupportHeightProvider(
        [](const Eigen::Vector3d&)->std::optional<double>{return .55;});
    ASSERT_TRUE(map->hasQuerySupportHeightProvider());
    auto now=scan_planner::SolveBudget::Clock::now();
    auto budget=std::make_shared<scan_planner::SolveBudget>(
        std::chrono::milliseconds(400),[&]{return now;});
    optimizer.setSolveBudget(budget);
    auto controls=straightOptimizerControls();optimizer.initControlPoints(controls);
    ASSERT_TRUE(optimizer.controlPointsInitialized());
    if(cancelled)budget->cancel();else now+=std::chrono::milliseconds(400);
    Eigen::MatrixXd result;
    EXPECT_FALSE(optimizer.BsplineOptimizeTrajRebound(result,.5));
    EXPECT_FALSE(optimizer.BsplineOptimizeTrajRefine(controls,.5,result));
    EXPECT_FALSE(budget->allowed());
    map->exchangeQuerySupportHeightProvider(std::move(previous));
    EXPECT_FALSE(map->hasQuerySupportHeightProvider());
  }
}
TEST(OptimizerBudget, RemovingStairScopePreservesFlatNumericalResult) {
  Eigen::MatrixXd original,after_stair;
  for(bool scoped:{false,true}) {
    auto map=GridMapTestAccess::make();scan_planner::BsplineOptimizer optimizer;
    scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
    if(scoped) {
      auto previous=map->exchangeQuerySupportHeightProvider(
          [](const Eigen::Vector3d&)->std::optional<double>{return .55;});
      map->exchangeQuerySupportHeightProvider(std::move(previous));
    }
    EXPECT_FALSE(map->hasQuerySupportHeightProvider());
    auto controls=straightOptimizerControls();optimizer.initControlPoints(controls);
    ASSERT_TRUE(optimizer.controlPointsInitialized());
    EXPECT_TRUE(optimizer.BsplineOptimizeTrajRebound(scoped?after_stair:original,1.));
  }
  ASSERT_EQ(original.rows(),after_stair.rows());ASSERT_EQ(original.cols(),after_stair.cols());
  EXPECT_TRUE((original.array()==after_stair.array()).all());
}

namespace {
scan_planner::SplineHeadingContract roundingHeading() {
  scan_planner::SplineHeadingContract heading;
  heading.measured_heading_enabled=true;heading.align_then_forward=true;
  heading.policy=scan_planner::BodyHeadingPolicy::PathTangent;
  heading.measured_yaw=rounding_retry_fixture::measured_yaw;heading.body_extent=.53;
  return heading;
}
void initializeRoundingFixture(scan_planner::BsplineOptimizer& optimizer,const GridMap::Ptr& map,
                               const scan_planner::SolveBudget::Ptr& budget) {
  scan_planner::BsplineOptimizerTestAccess::configureRoundingFixture(optimizer,map);
  optimizer.setSolveBudget(budget);optimizer.setPreviewHeadingContract(roundingHeading());
  optimizer.reference_path_samples_=rounding_retry_fixture::reference();
  auto controls=rounding_retry_fixture::controls();optimizer.initControlPoints(controls);
  ASSERT_TRUE(optimizer.controlPointsInitialized());
}
}

TEST(OptimizerBudget, LegacyRoundingCanRecoverWithoutChangingFixedPvaOrSkippingWholeCurve) {
  auto map=GridMapTestAccess::roundingFixtureBox();scan_planner::BsplineOptimizer optimizer;
  auto budget=std::make_shared<scan_planner::SolveBudget>();
  initializeRoundingFixture(optimizer,map,budget);EXPECT_FALSE(map->hasQuerySupportHeightProvider());
  const double interval=rounding_retry_fixture::interval;
  scan_planner::UniformBspline original(rounding_retry_fixture::controls(),3,interval);
  auto velocity=original.getDerivative(),acceleration=velocity.getDerivative();
  const double duration=original.getTimeSum();
  const scan_planner::CubicMotionBoundary boundary{original.evaluateDeBoorT(0.),
      velocity.evaluateDeBoorT(0.),acceleration.evaluateDeBoorT(0.),original.evaluateDeBoorT(duration),
      velocity.evaluateDeBoorT(duration),acceleration.evaluateDeBoorT(duration)};
  Eigen::MatrixXd result;
  ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(result,interval))<<optimizer.lastFailurePhase();
  scan_planner::UniformBspline curve(result,3,interval);
  EXPECT_TRUE(scan_planner::cubicBoundaryMatches(curve,boundary));
  const auto collision=scan_planner::checkWholeSplineCollision(curve,map->getResolution(),
      [&](const auto& point,double yaw){return map->getPlanningInflateOccupancy(point,yaw);},
      50000,roundingHeading());
  EXPECT_EQ(collision.state,scan_planner::SplineCollisionState::Clear);
  EXPECT_TRUE(budget->allowed());EXPECT_TRUE(optimizer.lastFailurePhase().empty());
}

TEST(OptimizerBudget, RoundingRecoveryCannotRefreshCancelledOrExpiredOriginalBudget) {
  for(bool cancelled:{false,true})for(int allowed_checks:{0,5,30}) {
    SCOPED_TRACE(cancelled);
    SCOPED_TRACE(allowed_checks);
    auto map=GridMapTestAccess::roundingFixtureBox();scan_planner::BsplineOptimizer optimizer;
    const auto origin=scan_planner::SolveBudget::Clock::now();
    int checks=0;bool armed=false;scan_planner::SolveBudget::Ptr budget;
    budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&]{
      if(armed&&++checks>allowed_checks) {
        if(cancelled)budget->cancel();
        return origin+std::chrono::milliseconds(400);
      }
      return origin;
    });
    initializeRoundingFixture(optimizer,map,budget);const auto deadline=budget->deadline();
    armed=true;if(cancelled&&allowed_checks==0)budget->cancel();
    Eigen::MatrixXd result;
    EXPECT_FALSE(optimizer.BsplineOptimizeTrajRebound(result,rounding_retry_fixture::interval));
    EXPECT_FALSE(budget->allowed());EXPECT_EQ(budget->deadline(),deadline);
    EXPECT_EQ(budget->stop(),cancelled?scan_planner::SolveBudget::Stop::Cancelled:
                                    scan_planner::SolveBudget::Stop::Deadline);
  }
}

TEST(OptimizerEvidence, RoundingRecoveryDoesNotAcceptAnOccupiedCurrentBody) {
  auto map=GridMapTestAccess::roundingFixtureBox();scan_planner::BsplineOptimizer optimizer;
  auto budget=std::make_shared<scan_planner::SolveBudget>();
  initializeRoundingFixture(optimizer,map,budget);
  const Eigen::Vector3d occupied(.125,.025,.55);map->setOccupied(occupied);
  Eigen::MatrixXd result;
  EXPECT_FALSE(optimizer.BsplineOptimizeTrajRebound(result,rounding_retry_fixture::interval));
  EXPECT_EQ(optimizer.lastFailurePhase(),"failed_final_collision");
  EXPECT_EQ(map->getOccupancy(occupied),1);
}
TEST(OptimizerEvidence, UnknownOnFinalCurveHasTypedReasonWithoutClearingEvidence) {
  auto map=GridMapTestAccess::make();scan_planner::BsplineOptimizer optimizer;
  scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
  auto controls=straightOptimizerControls();optimizer.initControlPoints(controls);
  ASSERT_TRUE(optimizer.controlPointsInitialized());
  const Eigen::Vector3d unknown(.325,.025,.55);GridMapTestAccess::unknown(*map,unknown);
  Eigen::MatrixXd result;
  EXPECT_FALSE(optimizer.BsplineOptimizeTrajRebound(result,1.));
  EXPECT_EQ(optimizer.lastFailurePhase(),"waiting_observed_space");
  EXPECT_TRUE(map->isUnknown(unknown));
  map=GridMapTestAccess::make();scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
  controls=straightOptimizerControls();optimizer.initControlPoints(controls);
  ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(result,1.));
  EXPECT_TRUE(optimizer.lastFailurePhase().empty());
}
TEST(OptimizerEvidence, ActualOccupiedCurveRemainsRejectedAndDistinctFromUnknown) {
  auto map=GridMapTestAccess::make();scan_planner::BsplineOptimizer optimizer;
  scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
  auto controls=straightOptimizerControls();optimizer.initControlPoints(controls);
  ASSERT_TRUE(optimizer.controlPointsInitialized());
  const Eigen::Vector3d occupied(.325,.025,.55);map->setOccupied(occupied);
  Eigen::MatrixXd result;
  EXPECT_FALSE(optimizer.BsplineOptimizeTrajRebound(result,1.));
  EXPECT_EQ(optimizer.lastFailurePhase(),"failed_final_collision");
  EXPECT_EQ(map->getOccupancy(occupied),1);
}
TEST(OptimizerEvidence, OfficialPlanningClearInflationDoesNotEraseRawUnknown) {
  auto map=GridMapTestAccess::make(false);scan_planner::BsplineOptimizer optimizer;
  scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
  auto controls=straightOptimizerControls();optimizer.initControlPoints(controls);
  ASSERT_TRUE(optimizer.controlPointsInitialized());
  const Eigen::Vector3d unknown(.325,.025,.55);GridMapTestAccess::unknown(*map,unknown);
  Eigen::MatrixXd result;
  EXPECT_TRUE(optimizer.BsplineOptimizeTrajRebound(result,1.));
  EXPECT_TRUE(optimizer.lastFailurePhase().empty());
  EXPECT_TRUE(map->isUnknown(unknown));
  EXPECT_EQ(map->inspectInflateOccupancy(unknown,0.).state(),2);
}
TEST(OptimizerEvidence, NativeUnknownEndpointFailurePreservesTypedEvidence) {
  auto map=GridMapTestAccess::make();scan_planner::BsplineOptimizer optimizer;
  scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
  GridMapTestAccess::unknown(*map,{.3,0.,.55});
  const auto result=optimizer.a_star_->AstarSearch(.05,{.3,0.,.55},{1.5,0.,.55},false);
  ASSERT_EQ(result,ASTAR_RET::INIT_UNOBSERVED);
  scan_planner::BsplineOptimizerTestAccess::searchFailure(optimizer,result);
  EXPECT_EQ(optimizer.lastFailurePhase(),"waiting_observed_space");
  EXPECT_TRUE(map->isUnknown(Eigen::Vector3d(.3,0.,.55)));
}
TEST(OptimizerEvidence, TerminalUnknownInitializationAndReboundStayUncertified) {
  auto map=GridMapTestAccess::make();scan_planner::BsplineOptimizer optimizer;
  scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
  auto controls=straightOptimizerControls();optimizer.initControlPoints(controls);
  ASSERT_TRUE(optimizer.controlPointsInitialized());
  GridMapTestAccess::allUnknown(*map);
  EXPECT_FALSE(scan_planner::BsplineOptimizerTestAccess::checkRebound(optimizer));
  EXPECT_EQ(optimizer.lastFailurePhase(),"waiting_observed_space");
  optimizer.initControlPoints(controls);
  EXPECT_FALSE(optimizer.controlPointsInitialized());
  EXPECT_EQ(optimizer.lastFailurePhase(),"waiting_observed_space");
}

TEST(OptimizerStairCurve, ActualCubicNotControlPolygonDeterminesProxyBypass) {
  auto map=GridMapTestAccess::productionBox();scan_planner::BsplineOptimizer optimizer;
  scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
  auto controls=straightOptimizerControls();
  controls.row(1).setZero();
  controls(1,4)=1.8;
  const Eigen::Vector3d polygon_obstacle=controls.col(4);
  map->setOccupied(polygon_obstacle);
  auto prior=map->exchangeQuerySupportHeightProvider(
      [](const Eigen::Vector3d&)->std::optional<double>{return .55;});
  scan_planner::UniformBspline curve(controls,3,1.);
  auto actual=scan_planner::checkWholeSplineCollision(curve,map->getResolution(),
      [&](const Eigen::Vector3d &p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);});
  ASSERT_EQ(actual.state,scan_planner::SplineCollisionState::Clear);
  ASSERT_EQ(map->getPlanningInflateOccupancy(polygon_obstacle,0.),1);
  optimizer.initControlPointsWithSpline(controls,1.);
  ASSERT_TRUE(optimizer.controlPointsInitialized());
  EXPECT_TRUE(scan_planner::BsplineOptimizerTestAccess::actualClear(optimizer,controls));
  EXPECT_FALSE(scan_planner::BsplineOptimizerTestAccess::checkRebound(optimizer));
  EXPECT_EQ(map->getOccupancy(polygon_obstacle),1);
  // Scope removal makes the new entry exactly the legacy algorithm again.
  map->exchangeQuerySupportHeightProvider(std::move(prior));
  EXPECT_FALSE(scan_planner::BsplineOptimizerTestAccess::actualClear(optimizer,controls));
}

TEST(OptimizerStairCurve, NativeBoxAndUnknownOnActualCurveCannotBeBypassed) {
  for(bool unknown:{false,true}) {
    auto map=GridMapTestAccess::productionBox();scan_planner::BsplineOptimizer optimizer;
    scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
    auto prior=map->exchangeQuerySupportHeightProvider(
        [](const Eigen::Vector3d&)->std::optional<double>{return .55;});
    auto controls=straightOptimizerControls();
    optimizer.initControlPointsWithSpline(controls,1.);
    ASSERT_TRUE(optimizer.controlPointsInitialized());
    const Eigen::Vector3d witness(.325,.025,.55);
    if(unknown)GridMapTestAccess::unknown(*map,witness);else map->setOccupied(witness);
    EXPECT_FALSE(scan_planner::BsplineOptimizerTestAccess::actualClear(optimizer,controls));
    Eigen::MatrixXd output;
    EXPECT_FALSE(optimizer.BsplineOptimizeTrajRebound(output,1.));
    EXPECT_EQ(optimizer.lastFailurePhase(),unknown?"waiting_observed_space":"failed_final_collision");
    if(unknown)EXPECT_TRUE(map->isUnknown(witness));else EXPECT_EQ(map->getOccupancy(witness),1);
    map->exchangeQuerySupportHeightProvider(std::move(prior));
  }
}

TEST(OptimizerStairCurve, ExplicitIntervalPreservesLegacyFlatAndBudgetContracts) {
  Eigen::MatrixXd legacy_result,new_result;
  for(bool use_new:{false,true}) {
    auto map=GridMapTestAccess::make();scan_planner::BsplineOptimizer optimizer;
    scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
    auto controls=straightOptimizerControls();
    if(use_new)optimizer.initControlPointsWithSpline(controls,1.);
    else optimizer.initControlPoints(controls);
    ASSERT_TRUE(optimizer.controlPointsInitialized());
    EXPECT_FALSE(scan_planner::BsplineOptimizerTestAccess::actualClear(optimizer,controls));
    EXPECT_TRUE(optimizer.BsplineOptimizeTrajRebound(use_new?new_result:legacy_result,1.));
  }
  EXPECT_TRUE((legacy_result.array()==new_result.array()).all());
  for(bool cancelled:{false,true}) {
    auto map=GridMapTestAccess::make();scan_planner::BsplineOptimizer optimizer;
    scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
    map->exchangeQuerySupportHeightProvider(
        [](const Eigen::Vector3d&)->std::optional<double>{return .55;});
    auto now=scan_planner::SolveBudget::Clock::now();
    auto budget=std::make_shared<scan_planner::SolveBudget>(
        std::chrono::milliseconds(400),[&]{return now;});
    optimizer.setSolveBudget(budget);
    auto controls=straightOptimizerControls();
    if(cancelled)budget->cancel();else now+=std::chrono::milliseconds(400);
    optimizer.initControlPointsWithSpline(controls,1.);
    EXPECT_FALSE(optimizer.controlPointsInitialized());
    EXPECT_FALSE(scan_planner::BsplineOptimizerTestAccess::actualClear(optimizer,controls));
    EXPECT_FALSE(budget->allowed());
  }
}

TEST(OptimizerStairCurve, InvalidIntervalDoesNotInitializeOrReusePreviousAuthority) {
  auto map=GridMapTestAccess::make();scan_planner::BsplineOptimizer optimizer;
  scan_planner::BsplineOptimizerTestAccess::setup(optimizer,map);
  map->exchangeQuerySupportHeightProvider(
      [](const Eigen::Vector3d&)->std::optional<double>{return .55;});
  auto controls=straightOptimizerControls();
  for(double dt:{0.,-1.,std::numeric_limits<double>::quiet_NaN()}) {
    optimizer.initControlPointsWithSpline(controls,1.);
    ASSERT_TRUE(optimizer.controlPointsInitialized());
    optimizer.initControlPointsWithSpline(controls,dt);
    EXPECT_FALSE(optimizer.controlPointsInitialized());
    EXPECT_EQ(optimizer.lastFailurePhase(),"failed_curve_validation");
  }
  optimizer.initControlPointsWithSpline(controls,1.);
  ASSERT_TRUE(scan_planner::BsplineOptimizerTestAccess::actualClear(optimizer,controls));
  optimizer.initControlPoints(controls);
  EXPECT_FALSE(scan_planner::BsplineOptimizerTestAccess::actualClear(optimizer,controls));
}
