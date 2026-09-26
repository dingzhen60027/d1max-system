#include <gtest/gtest.h>
#include <plan_manage/input_contract.hpp>
#include <limits>

using Eigen::Vector3d;
using scan_planner::prepareReferenceWaypoints;

TEST(InputContract, ReverseMeasuredVelocityMustNotBecomeStationaryBoundary) {
  const Eigen::Vector2d goal(3., 0.);
  for (double speed : {.004, .025, .20}) {
    EXPECT_FALSE(scan_planner::shouldSuppressOpposedPrediction(true, goal, {-speed, .01, 0.}));
    EXPECT_TRUE(scan_planner::shouldSuppressOpposedPrediction(false, goal, {-speed, .01, 0.}));
  }
  EXPECT_FALSE(scan_planner::shouldSuppressOpposedPrediction(false, goal, {.1, 0., 0.}));
  EXPECT_FALSE(scan_planner::shouldSuppressOpposedPrediction(false, {0., 0.}, {-.1, 0., 0.}));
}

TEST(InputContract, GroundToBodyShiftOnlyOnceAndDuplicateStartRemoved) {
  const std::vector<Vector3d> points{{0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
  const auto result = prepareReferenceWaypoints(points, {0, 0, .55}, .55);
  ASSERT_EQ(result.size(), 2U);
  EXPECT_TRUE(result[0].isApprox(Vector3d(1, 0, .55)));
  EXPECT_TRUE(result[1].isApprox(Vector3d(1, 1, .55)));
}

TEST(InputContract, CornersAndNonzeroSegmentsAreNotSimplifiedAway) {
  const std::vector<Vector3d> points{{0, 0, 0}, {4, 0, 0}, {4, 4, 0}, {0, 4, 0}};
  const auto result = prepareReferenceWaypoints(points, {0, 0, .55}, .55);
  ASSERT_EQ(result.size(), 3U);
  EXPECT_EQ(result[0].x(), 4);
  EXPECT_EQ(result[1].x(), 4);
  EXPECT_EQ(result[2].x(), 0);
}

TEST(InputContract, RejectInvalidAndDiscontinuousPaths) {
  EXPECT_THROW(prepareReferenceWaypoints({}, {0, 0, .55}, .55), std::invalid_argument);
  EXPECT_THROW(prepareReferenceWaypoints({{5, 0, 0}}, {0, 0, .55}, .55), std::invalid_argument);
  EXPECT_THROW(prepareReferenceWaypoints({{0, 0, 0}}, {0, 0, .55}, .55), std::invalid_argument);
  EXPECT_THROW(prepareReferenceWaypoints({{0, 0, 0}, {std::numeric_limits<double>::quiet_NaN(), 0, 0}},
                                        {0, 0, .55}, .55), std::invalid_argument);
}

TEST(InputContract, StandardBodyTwistRotatesIntoWorld) {
  const Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2, Vector3d::UnitZ()));
  EXPECT_TRUE(scan_planner::odometryVelocityInWorld({1, 0, 0}, q, true).isApprox(Vector3d(0, 1, 0), 1e-8));
  EXPECT_TRUE(scan_planner::odometryVelocityInWorld({1, 0, 0}, q, false).isApprox(Vector3d(1, 0, 0)));
  EXPECT_THROW(scan_planner::odometryVelocityInWorld({1, 0, 0}, Eigen::Quaterniond(0,0,0,0), true),
               std::invalid_argument);
}

TEST(InputContract, SessionAndGenerationMustMatchWithoutReplay) {
  using scan_planner::acceptsReferenceGeneration;
  EXPECT_TRUE(acceptsReferenceGeneration("session_a", "session_a", false, 0, 0));
  EXPECT_TRUE(acceptsReferenceGeneration("session_a", "session_a", true, 3, 4));
  EXPECT_FALSE(acceptsReferenceGeneration("session_a", "session_b", true, 3, 4));
  EXPECT_FALSE(acceptsReferenceGeneration("session_a", "session_a", true, 3, 3));
  EXPECT_FALSE(acceptsReferenceGeneration("session_a", "session_a", true, 3, 2));
  EXPECT_FALSE(acceptsReferenceGeneration("", "", false, 0, 0));
}

TEST(InputContract, PeriodicReplanUsesIndependentTimeAndDefaultsDisabled) {
  using scan_planner::periodicReplanDue;
  EXPECT_FALSE(periodicReplanDue(100, 1, 0));
  EXPECT_FALSE(periodicReplanDue(1.9, 1, 1));
  EXPECT_TRUE(periodicReplanDue(2, 1, 1));
  EXPECT_FALSE(periodicReplanDue(.5, 1, 1));
  EXPECT_FALSE(periodicReplanDue(std::numeric_limits<double>::quiet_NaN(), 1, 1));
}

TEST(InputContract, MeasuredReferenceCompletionUsesXYAndHeightTogether) {
  using scan_planner::measuredReferenceGoalReached;
  EXPECT_TRUE(measuredReferenceGoalReached({.1,.1,.66},{0.,0.,.55},.20,.15));
  EXPECT_FALSE(measuredReferenceGoalReached({.21,0.,.55},{0.,0.,.55},.20,.15));
  EXPECT_FALSE(measuredReferenceGoalReached({0.,0.,.71},{0.,0.,.55},.20,.15));
  // Elapsed spline time is intentionally not an input to completion.
  EXPECT_FALSE(measuredReferenceGoalReached({0.,0.,.55},{1.,0.,.55},.20,.15));
}

TEST(InputContract, DynamicsRecoveryDoesNotRequirePoseOrOccupancyChange) {
  using scan_planner::failedDynamicsBoundaryChanged;
  EXPECT_TRUE(failedDynamicsBoundaryChanged({.45,0,0},{0,0,0},.30));
  EXPECT_TRUE(failedDynamicsBoundaryChanged({.301,0,0},{.299,0,0},.30));
  EXPECT_FALSE(failedDynamicsBoundaryChanged({.45,0,0},{.40,0,0},.30));
  EXPECT_FALSE(failedDynamicsBoundaryChanged({.1,0,0},{.1,0,0},.30));
  EXPECT_FALSE(failedDynamicsBoundaryChanged({.1,0,0},{.101,0,0},.30));
  EXPECT_TRUE(failedDynamicsBoundaryChanged({.2,0,0},{0,0,0},.30));
}
