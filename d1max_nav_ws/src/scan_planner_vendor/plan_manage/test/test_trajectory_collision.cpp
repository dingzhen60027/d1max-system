#include <gtest/gtest.h>
#include <thread>
#include "plan_manage/trajectory_collision.hpp"

static scan_planner::UniformBspline line(double y=0.) {
  Eigen::MatrixXd points(3,23);
  for(int i=0;i<23;++i) points.col(i)=Eigen::Vector3d((i-1)*.05,y,.55);
  return scan_planner::UniformBspline(points,3,.2);
}

TEST(TrajectoryCollision, PredecessorFullCurveIncludesFinalThirdAndEndpoint) {
  auto old=line(),replacement=line(1.);
  const auto occupied=[](const Eigen::Vector3d &p,double) {return p.x()>.85 && p.y()<.2 ? 1:0;};
  EXPECT_TRUE(scan_planner::wholeCurveCollisionFree(replacement,.08,.49,occupied));
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(old,.08,.49,occupied));
}

TEST(TrajectoryCollision, UnknownAndOutsideCannotProduceContinuityProof) {
  auto old=line();
  for(int state:{-1,1,2}) {
    EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(old,.08,.49,
        [state](const Eigen::Vector3d &p,double){return p.x()>.9?state:0;}));
  }
}

TEST(TrajectoryCollision, QueryBudgetNeverConvertsPartialCheckToSuccess) {
  auto old=line(); std::size_t queries=0;
  const auto free=[&](const Eigen::Vector3d &,double){++queries;return 0;};
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(old,.08,.49,free,20));
  EXPECT_LE(queries,20U);
  EXPECT_TRUE(scan_planner::wholeCurveCollisionFree(old,.08,.49,free,5000));
  EXPECT_GT(queries,100U);
}

TEST(TrajectoryCollision, WallBudgetNeverConvertsPartialCheckToSuccess) {
  auto old=line(); std::size_t queries=0;
  EXPECT_FALSE(scan_planner::wholeCurveCollisionFree(old,.08,.49,
      [&](const Eigen::Vector3d &,double){++queries;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));return 0;},5000,.001));
  EXPECT_LE(queries,1U);
}
