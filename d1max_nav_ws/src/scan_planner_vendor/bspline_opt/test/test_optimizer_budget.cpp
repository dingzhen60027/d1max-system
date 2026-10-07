#include <gtest/gtest.h>
#include <bspline_opt/bspline_optimizer.h>
#include <cmath>
#include <limits>
namespace scan_planner {
struct BsplineOptimizerTestAccess {
  static void configured(BsplineOptimizer& optimizer,double speed) {
    optimizer.configured_max_vel_=optimizer.max_vel_=speed;
  }
  static double speed(const BsplineOptimizer& optimizer) {return optimizer.max_vel_;}
};
}
TEST(OptimizerReferenceDomain, ValidatedReferencePointSixDoesNotChangeConfiguredCruiseFallback) {
  scan_planner::BsplineOptimizer optimizer;
  scan_planner::BsplineOptimizerTestAccess::configured(optimizer,.25);
  for(const double reference:{.575,.6}) {
    ASSERT_NO_THROW(optimizer.setReferenceSpeedLimit(reference));
    EXPECT_DOUBLE_EQ(scan_planner::BsplineOptimizerTestAccess::speed(optimizer),reference);
    optimizer.setReferenceSpeedLimit(std::nullopt);
    EXPECT_DOUBLE_EQ(scan_planner::BsplineOptimizerTestAccess::speed(optimizer),.25);
  }
}
TEST(OptimizerReferenceDomain, InvalidDomainCannotChangeThePreviousLimitOrInferOneWhenAbsent) {
  scan_planner::BsplineOptimizer optimizer;
  scan_planner::BsplineOptimizerTestAccess::configured(optimizer,.25);
  for(const double value:{.6500001,0.,-1.,std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(optimizer.setReferenceSpeedLimit(value),std::invalid_argument);
    EXPECT_DOUBLE_EQ(scan_planner::BsplineOptimizerTestAccess::speed(optimizer),.25);
  }
  optimizer.setReferenceSpeedLimit(std::nullopt);
  EXPECT_DOUBLE_EQ(scan_planner::BsplineOptimizerTestAccess::speed(optimizer),.25);
}
TEST(OptimizerReferenceDomain, PointSixFiveBoundaryPreservesCruiseFallbackAndPreviousValidatedLimit) {
  scan_planner::BsplineOptimizer optimizer;
  scan_planner::BsplineOptimizerTestAccess::configured(optimizer,.25);
  for(const double reference:{.616161,.65}) {
    ASSERT_NO_THROW(optimizer.setReferenceSpeedLimit(reference));
    EXPECT_DOUBLE_EQ(scan_planner::BsplineOptimizerTestAccess::speed(optimizer),reference);
  }
  for(const double value:{.6500001,std::nextafter(.65,std::numeric_limits<double>::infinity())}) {
    EXPECT_THROW(optimizer.setReferenceSpeedLimit(value),std::invalid_argument);
    EXPECT_DOUBLE_EQ(scan_planner::BsplineOptimizerTestAccess::speed(optimizer),.65);
  }
  optimizer.setReferenceSpeedLimit(std::nullopt);
  EXPECT_DOUBLE_EQ(scan_planner::BsplineOptimizerTestAccess::speed(optimizer),.25);
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
