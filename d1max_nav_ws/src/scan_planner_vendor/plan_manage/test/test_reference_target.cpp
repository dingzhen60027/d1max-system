#include <gtest/gtest.h>
#include <plan_manage/reference_target.hpp>
#include <plan_manage/collision_reference.hpp>

using Point=Eigen::Vector3d;
using scan_planner::DiscreteReference;
using scan_planner::ReferenceTargetOptions;
using scan_planner::selectReferenceTarget;

namespace {
DiscreteReference line(double length=8.) {
  DiscreteReference path;
  path.set({Point(0,0,.55),Point(length,0,.55)});
  return path;
}
int freeMap(const Point &,double) { return 0; }
}  // namespace

TEST(ReferenceTarget, UnobstructedNominalEndpointIsUnchanged) {
  const auto path=line();
  const auto result=selectReferenceTarget(path,path.points.front(),0.,4.,{},freeMap);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.arc,4.);
  EXPECT_TRUE(result.point.isApprox(Point(4,0,.55)));
  EXPECT_EQ(result.reason,"reference_target_nominal");
  EXPECT_LE(result.queries,4096U);
  EXPECT_EQ(result.queries,result.free_queries);
  EXPECT_EQ(result.unknown_queries+result.occupied_queries+result.outside_queries,0U);
  EXPECT_FALSE(result.has_first_blocked);
  EXPECT_TRUE(result.blocked_points.empty());
}

TEST(ReferenceTarget, HorizonObstacleUsesForwardExitAndVerifiedSideDetour) {
  const auto path=line();
  const auto occupancy=[](const Point &p,double) {
    return p.x()>=2.7 && p.x()<=3.4 && std::abs(p.y())<=.25 ? 1:0;
  };
  const auto result=selectReferenceTarget(path,path.points.front(),0.,3.,{},occupancy);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.arc,3.4);
  EXPECT_LE(result.arc,4.);
  EXPECT_EQ(result.reason,"reference_target_forward");
  int searches=0;
  const auto search=[&](const Point &a,const Point &b) {
    ++searches;
    return std::vector<Point>{a,Point(a.x(),.7,a.z()),Point(b.x(),.7,b.z()),b};
  };
  std::vector<Point> detour;
  std::string reason;
  ASSERT_TRUE(scan_planner::buildCollisionAwareReference(
      path.slice(0.,result.arc,path.points.front()),.08,occupancy,search,detour,reason)) << reason;
  EXPECT_EQ(searches,1);
  EXPECT_EQ(reason,"reference_detour_ready");
  ASSERT_FALSE(detour.empty());
  EXPECT_TRUE(detour.back().isApprox(result.point));
  bool lateral=false;
  for (std::size_t i=1;i<detour.size();++i) {
    EXPECT_TRUE(scan_planner::collisionFreeSegment(detour[i-1],detour[i],.01,occupancy));
    EXPECT_NEAR(detour[i].z(),.55,1e-12);
    lateral|=detour[i].y()>.6;
  }
  EXPECT_TRUE(lateral);
}

TEST(ReferenceTarget, BlockedCorridorCannotBeConvertedToSuccessfulSeed) {
  const auto path=line();
  // A full-width wall blocks every possible XY detour at the allowed Z.
  const auto occupancy=[](const Point &p,double) {
    if (std::abs(p.y())>1.) return -1;
    return p.x()>=1. && p.x()<=2. ? 1:0;
  };
  const auto target=selectReferenceTarget(path,path.points.front(),0.,3.,{},occupancy);
  ASSERT_TRUE(target.valid); // Free target alone never authorizes a trajectory.
  std::vector<Point> detour;
  std::string reason;
  EXPECT_FALSE(scan_planner::buildCollisionAwareReference(
      path.slice(0.,target.arc,path.points.front()),.08,occupancy,
      [](const Point &,const Point &){return std::vector<Point>{};},detour,reason));
  EXPECT_EQ(reason,"failed_reference_search");
}

TEST(ReferenceTarget, RollingMapBoundaryShortensHorizon) {
  const auto path=line();
  const auto result=selectReferenceTarget(path,path.points.front(),0.,4.,{},
      [](const Point &p,double){return p.x()>=3.5 ? -1:0;});
  ASSERT_TRUE(result.valid);
  EXPECT_GE(result.arc,2.);
  EXPECT_LT(result.arc,3.5);
  EXPECT_EQ(result.reason,"reference_target_shortened");
}

TEST(ReferenceTarget, DoesNotCrossOutOfMapGapToReenterLaterFreeReference) {
  const auto path=line();
  const auto result=selectReferenceTarget(path,path.points.front(),0.,4.,{},
      [](const Point &p,double){return p.x()>=1. && p.x()<=2. ? -1:0;});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason,"failed_reference_target_occupied");
}

TEST(ReferenceTarget, StairsRetainOrderedHeightAndEveryTurn) {
  DiscreteReference path;
  path.set({Point(0,0,.55),Point(2,0,.55),Point(2,2,1.55),
            Point(0,2,2.55),Point(0,0,3.55),Point(2,0,3.55)});
  const double nominal=path.arc[2];
  const auto occupancy=[](const Point &p,double) {
    // Same XY on a different floor does not imply the same occupancy.
    return (p-Point(2,2,1.55)).norm()<.18 ? 1:0;
  };
  const auto result=selectReferenceTarget(path,path.points.front(),0.,nominal,{},occupancy);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.arc,nominal);
  EXPECT_LE(result.arc,nominal+1.);
  EXPECT_TRUE(result.point.isApprox(path.sample(result.arc)));
  EXPECT_GT(result.point.z(),1.55);
  EXPECT_LT(result.point.z(),2.55);
  const auto selected=path.slice(0.,result.arc,path.points.front());
  bool lower_turn=false, landing=false;
  for (const auto &point:selected) {
    lower_turn|=point.isApprox(path.points[1]);
    landing|=point.isApprox(path.points[2]);
  }
  EXPECT_TRUE(lower_turn);
  EXPECT_TRUE(landing);
  const auto upper=selectReferenceTarget(path,path.points[4],path.arc[4],1.,{},occupancy);
  ASSERT_TRUE(upper.valid);
  EXPECT_TRUE(upper.point.isApprox(Point(1,0,3.55)));
}

TEST(ReferenceTarget, CannotUseBodyProximityAtCrossingAsSuccessfulAdvance) {
  DiscreteReference path;
  path.set({Point(0,0,.55),Point(2,0,.55),Point(2,2,.55),
            Point(0,2,.55),Point(0,0,.55),Point(1,0,.55)});
  ReferenceTargetOptions options;
  options.forward_margin=0.; options.backward_margin=0.;
  const auto result=selectReferenceTarget(path,path.points.front(),0.,8.,options,freeMap);
  EXPECT_FALSE(result.valid);
}

TEST(ReferenceTarget, FullyBlockedTargetWindowDoesNotRetreatToRobot) {
  const auto path=line();
  ReferenceTargetOptions options;
  options.backward_margin=5.;
  const auto result=selectReferenceTarget(path,path.points.front(),0.,3.,options,
      [](const Point &p,double){return p.x()>.1 ? 1:0;});
  EXPECT_FALSE(result.valid);
}

TEST(ReferenceTarget, ShortFinalLegStillRequiresRealNonHoverDisplacement) {
  const auto short_path=line(.25);
  const auto actual_leg=selectReferenceTarget(short_path,short_path.points.front(),0.,4.,{},freeMap);
  ASSERT_TRUE(actual_leg.valid);
  EXPECT_DOUBLE_EQ(actual_leg.arc,.25);
  const auto reached_path=line(.15);
  EXPECT_FALSE(selectReferenceTarget(reached_path,reached_path.points.front(),0.,4.,{},freeMap).valid);
}

TEST(ReferenceTarget, QueryBudgetFailsClosedWithoutUnboundedWork) {
  const auto path=line(100.);
  ReferenceTargetOptions options;
  options.max_queries=10;
  int queries=0;
  const auto result=selectReferenceTarget(path,path.points.front(),0.,20.,options,
      [&](const Point &,double){++queries;return 0;});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason,"failed_reference_search_budget");
  EXPECT_EQ(queries,0);
}

TEST(ReferenceTarget, InvalidConfigurationCannotAuthorizeEndpoint) {
  const auto path=line();
  ReferenceTargetOptions options;
  options.min_advance=-1.;
  const auto result=selectReferenceTarget(path,path.points.front(),0.,3.,options,freeMap);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason,"failed_reference_geometry");
}

TEST(ReferenceTarget, FailureReportsActualUnknownOccupiedAndBoundaryQueries) {
  const auto path=line();
  ReferenceTargetOptions options;
  options.backward_margin=0.;
  std::vector<std::pair<Point,int>> observed;
  const auto result=selectReferenceTarget(path,path.points.front(),0.,3.,options,
      [&](const Point &p,double) {
        const int value=p.x()<.5 ? 0 : p.x()<1. ? 2 : p.x()<1.5 ? 1 : -1;
        observed.emplace_back(p,value);
        return value;
      });
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason,"failed_reference_target_occupied");
  ASSERT_EQ(result.queries,observed.size());
  EXPECT_GT(result.free_queries,0U);
  EXPECT_GT(result.unknown_queries,0U);
  EXPECT_GT(result.occupied_queries,0U);
  EXPECT_EQ(result.outside_queries,1U); // original first-boundary stop retained
  EXPECT_EQ(result.queries,result.free_queries+result.unknown_queries+
                            result.occupied_queries+result.outside_queries);
  auto first=std::find_if(observed.begin(),observed.end(),[](const auto &v){return v.second!=0;});
  ASSERT_NE(first,observed.end());
  EXPECT_TRUE(result.has_first_blocked);
  EXPECT_TRUE(result.first_blocked.isApprox(first->first));
  EXPECT_EQ(result.first_blocked_value,2);
  ASSERT_EQ(result.blocked_points.size(),result.queries-result.free_queries);
  for (std::size_t i=0;i<result.blocked_points.size();++i)
    EXPECT_TRUE(result.blocked_points[i].isApprox((first+static_cast<long>(i))->first));
  EXPECT_TRUE(result.blocked_points.back().isApprox(observed.back().first));
}

TEST(ReferenceTarget, DiagnosticPointBudgetCannotChangeSelectionOrQueryCounts) {
  const auto path=line(30.);
  ReferenceTargetOptions options;
  const auto result=selectReferenceTarget(path,path.points.front(),0.,20.,options,
      [](const Point &,double){return 2;});
  EXPECT_FALSE(result.valid);
  EXPECT_GT(result.queries,scan_planner::ReferenceTargetResult::max_blocked_points);
  EXPECT_EQ(result.unknown_queries,result.queries);
  EXPECT_EQ(result.blocked_points.size(),scan_planner::ReferenceTargetResult::max_blocked_points);
  EXPECT_DOUBLE_EQ(result.arc,20.);
  EXPECT_TRUE(result.first_blocked.isApprox(path.points.front()));
}

TEST(ReferenceTarget, SuccessfulTargetKeepsMixedQueryStatisticsWithoutChangingChoice) {
  const auto path=line();
  const auto result=selectReferenceTarget(path,path.points.front(),0.,3.,{},
      [](const Point &p,double){return p.x()>=2.7 && p.x()<=3.4 ? 1:0;});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.reason,"reference_target_forward");
  EXPECT_GT(result.arc,3.4);
  EXPECT_EQ(result.occupied_queries,result.blocked_points.size());
  EXPECT_EQ(result.queries,result.free_queries+result.occupied_queries);
  EXPECT_EQ(result.unknown_queries,0U);
}
