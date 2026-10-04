#include <gtest/gtest.h>
#include <bspline_opt/bspline_optimizer.h>
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
