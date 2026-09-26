#include <gtest/gtest.h>
#include <plan_manage/collision_reference.hpp>

using scan_planner::buildCollisionAwareReference;
using scan_planner::collisionFreeSegment;
using Point=Eigen::Vector3d;

TEST(CollisionReference, PreservesClearCornerAndHeight) {
  const std::vector<Point> input{{0,0,.55},{1,0,.55},{1,1,.55}};
  std::vector<Point> result;
  std::string reason;
  int searches=0;
  const auto search=[&](const Point &, const Point &) { ++searches; return std::vector<Point>{}; };
  ASSERT_TRUE(buildCollisionAwareReference(input,.1,[](const Point &,double){return 0;},search,result,reason));
  EXPECT_EQ(searches,0);
  EXPECT_TRUE(result.front().isApprox(input.front()));
  EXPECT_TRUE(result.back().isApprox(input.back()));
  bool corner=false;
  for (const auto &p:result) { corner|=p.isApprox(input[1]); EXPECT_NEAR(p.z(),.55,1e-12); }
  EXPECT_TRUE(corner);
}

TEST(CollisionReference, UsesVerifiedDetourNotBlockedGlobalLine) {
  const auto occupied=[](const Point &p,double) {
    return p.x()>=1. && p.x()<=2. && std::abs(p.y())<.3 ? 1:0;
  };
  const auto search=[](const Point &a,const Point &b) {
    return std::vector<Point>{a,Point(a.x(),.6,a.z()),Point(b.x(),.6,b.z()),b};
  };
  std::vector<Point> result;
  std::string reason;
  ASSERT_TRUE(buildCollisionAwareReference({Point(0,0,.55),Point(3,0,.55)},.1,
      occupied,search,result,reason));
  EXPECT_EQ(reason,"reference_detour_ready");
  bool detoured=false;
  for (std::size_t i=1;i<result.size();++i) {
    EXPECT_TRUE(collisionFreeSegment(result[i-1],result[i],.05,occupied));
    EXPECT_NEAR(result[i].z(),.55,1e-12);
    detoured|=result[i].y()>.5;
  }
  EXPECT_TRUE(detoured);
}

TEST(CollisionReference, SearchFailureIsNotAValidSeed) {
  std::vector<Point> result;
  std::string reason;
  const auto occupied=[](const Point &p,double) {return p.x()>.9 && p.x()<1.5 ? 1:0;};
  EXPECT_FALSE(buildCollisionAwareReference({Point(0,0,0),Point(2,0,0)},.1,occupied,
      [](const Point &,const Point &){return std::vector<Point>{};},result,reason));
  EXPECT_EQ(reason,"failed_reference_search");
}

TEST(CollisionReference, RejectsSearchThatCutsBlockedConnector) {
  std::vector<Point> result;
  std::string reason;
  const auto occupied=[](const Point &p,double) {return p.x()>.9 && p.x()<1.5 ? 1:0;};
  EXPECT_FALSE(buildCollisionAwareReference({Point(0,0,0),Point(2,0,0)},.1,occupied,
      [](const Point &a,const Point &b){return std::vector<Point>{a,b};},result,reason));
  EXPECT_EQ(reason,"failed_reference_search_collision");
}

TEST(CollisionReference, UnknownAndTerminalObstacleAreNotMadeTraversable) {
  const auto unknown=[](const Point &p,double){return p.x()>1. ? -1:0;};
  EXPECT_FALSE(collisionFreeSegment(Point(0,0,0),Point(2,0,0),.05,unknown));
  std::vector<Point> result;
  std::string reason;
  EXPECT_FALSE(buildCollisionAwareReference({Point(0,0,0),Point(2,0,0)},.1,unknown,
      [](const Point &a,const Point &b){return std::vector<Point>{a,b};},result,reason));
  EXPECT_EQ(reason,"failed_reference_target_occupied");
}
