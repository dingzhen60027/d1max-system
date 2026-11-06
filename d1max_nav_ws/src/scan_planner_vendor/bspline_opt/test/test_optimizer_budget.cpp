#include <gtest/gtest.h>
#include <bspline_opt/bspline_optimizer.h>

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
TEST(OptimizerBudget, ExpiredInitializationDoesNotUseUninitializedEnvironment) {
  scan_planner::BsplineOptimizer optimizer;
  auto now=scan_planner::SolveBudget::Clock::now();
  auto budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&] {return now;});
  now+=std::chrono::milliseconds(400);optimizer.setSolveBudget(budget);
  Eigen::MatrixXd controls=Eigen::MatrixXd::Zero(3,8);
  EXPECT_TRUE(optimizer.initControlPoints(controls).empty());
  EXPECT_FALSE(optimizer.controlPointsInitialized());
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
