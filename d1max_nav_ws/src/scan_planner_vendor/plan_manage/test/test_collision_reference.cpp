#include <gtest/gtest.h>
#include <limits>
#include <plan_manage/collision_reference.hpp>
#include <plan_manage/forward_motion_boundary.hpp>
#include <bspline_opt/trajectory_timing.hpp>
#include <path_searching/dyn_a_star.h>

using scan_planner::buildCollisionAwareReference;
using scan_planner::collisionFreeSegment;
using scan_planner::DetourSearchResult;
using scan_planner::DetourSearchStatus;
using Point=Eigen::Vector3d;

// The production native map and search are exercised without ROS init or SDK.
struct GridMapTestAccess {
 static GridMap::Ptr boxMap() {
   auto map=std::make_shared<GridMap>();auto& p=map->mp_;auto& d=map->md_;
   p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={160,160,40};
   p.map_origin_idx_.setZero();p.map_size_=p.map_voxel_num_.cast<double>()*.05;map->updateMapBoundaryFromIndex();
   p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
   p.planning_collision_policy_="official";p.require_observed_free_=false;
   p.collision_model_="oriented_box";p.box_length_=1.03;p.box_width_=.4;
   p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
   p.double_cylinder_radius_=.2;p.double_cylinder_offset_=.2;
   p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.25;
   const auto n=160U*160U*40U;d.occupancy_buffer_.assign(n,-1.);
   d.occupancy_buffer_inflate_.assign(n,0);d.occupancy_buffer_inflate_cnt_.assign(n,0);
   d.planning_occupancy_buffer_inflate_.assign(n,0);d.planning_occupancy_buffer_inflate_cnt_.assign(n,0);
   map->rebuildInflationOffsets();return map;
 }
};

TEST(CollisionReference, NativeBoxDetourDoesNotRepeatOrientationBlockedDeparture) {
 // The reference tangent is horizontal. A nearby occupied voxel is outside
 // that rectangular footprint but inside it at the detour's diagonal heading.
 auto map=GridMapTestAccess::boxMap();
 map->setOccupied({.6,0.,.5});map->setOccupied({1.25,.25,.5});
 AStar native;native.initGridMapFromEnvironment(map);
 const std::vector<Point> route{{0.,.7,.5},{0.,0.,.5},{2.8,0.,.5}};
 std::vector<Point> result;std::string reason;int target_rejections=0,searches=0;
 const auto occupied=[&](const Point& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);};
 const scan_planner::DetourSearch search=[&](const Point& a,const Point& b) {
   ++searches;const auto ret=native.AstarSearch(.05,a,b,false,true);
   if(ret==ASTAR_RET::INIT_START_OCCUPIED)return DetourSearchResult{DetourSearchStatus::StartEndpointOccupied,{}};
   if(ret==ASTAR_RET::INIT_TARGET_OCCUPIED){++target_rejections;return DetourSearchResult{DetourSearchStatus::TargetEndpointOccupied,{}};}
   return ret==ASTAR_RET::SUCCESS?DetourSearchResult{DetourSearchStatus::Success,native.getPath()}:DetourSearchResult{};
 };
 ASSERT_TRUE(buildCollisionAwareReference(route,.05,occupied,search,result,reason))<<reason;
 EXPECT_GT(target_rejections,0);EXPECT_GT(searches,target_rejections);
 RecordProperty("native_target_rejections",target_rejections);
 RecordProperty("native_search_calls",searches);
 if(!result.empty()) {
   EXPECT_TRUE(result.front().isApprox(route.front()));EXPECT_TRUE(result.back().isApprox(route.back()));
   for(std::size_t i=1;i<result.size();++i)EXPECT_TRUE(collisionFreeSegment(result[i-1],result[i],.025,occupied));
 }
}

TEST(CollisionReference, NativeOccupiedActualStartIsNotMovedOrCleared) {
 auto map=GridMapTestAccess::boxMap();map->setOccupied({0.,0.,.5});
 AStar native;native.initGridMapFromEnvironment(map);
 std::vector<Point> result;std::string reason;int searches=0;
 const auto occupied=[&](const Point& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);};
 const scan_planner::DetourSearch search=[&](const Point& a,const Point& b) {
   ++searches;EXPECT_TRUE(a.isApprox(Point(0,0,.5)));
   const auto ret=native.AstarSearch(.05,a,b,false,true);
   EXPECT_EQ(ret,ASTAR_RET::INIT_START_OCCUPIED);
   return DetourSearchResult{DetourSearchStatus::StartEndpointOccupied,{}};
 };
 EXPECT_FALSE(buildCollisionAwareReference({{0,0,.5},{2.8,0,.5}},.05,occupied,search,result,reason));
 EXPECT_EQ(searches,1);EXPECT_EQ(reason,"failed_reference_search");
 EXPECT_EQ(map->getPlanningInflateOccupancy({0,0,.5},0.),1);
}

TEST(CollisionReference, NativeWallWithoutPassageIsStillRejectedOnce) {
 auto map=GridMapTestAccess::boxMap();
 for(int y=-79;y<80;++y)map->setOccupied({1.2,(y+.5)*.05,.5});
 AStar native;native.initGridMapFromEnvironment(map);
 std::vector<Point> result;std::string reason;int searches=0;
 const auto occupied=[&](const Point& p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);};
 const scan_planner::DetourSearch search=[&](const Point& a,const Point& b) {
   ++searches;const auto ret=native.AstarSearch(.05,a,b,false,true);
   EXPECT_NE(ret,ASTAR_RET::SUCCESS);
   if(ret==ASTAR_RET::INIT_START_OCCUPIED)return DetourSearchResult{DetourSearchStatus::StartEndpointOccupied,{}};
   if(ret==ASTAR_RET::INIT_TARGET_OCCUPIED)return DetourSearchResult{DetourSearchStatus::TargetEndpointOccupied,{}};
   return DetourSearchResult{};
 };
 EXPECT_FALSE(buildCollisionAwareReference({{0,0,.5},{2.8,0,.5}},.05,occupied,search,result,reason));
 EXPECT_EQ(searches,1);EXPECT_EQ(reason,"failed_reference_search");
}

namespace {
int contractObstacle(const Point& p,double) {
 return p.x()>=1.&&p.x()<=2.&&std::abs(p.y())<.3?1:0;
}
DetourSearchResult contractDetour(const Point& a,const Point& b) {
 return {DetourSearchStatus::Success,{a,Point(a.x(),.6,a.z()),Point(b.x(),.6,b.z()),b}};
}
}

TEST(CollisionReference, RejectedDepartureMovesEarlierButNeverMovesActualStart) {
 std::vector<Point> result,starts;std::string reason;
 const scan_planner::DetourSearch search=[&](const Point& a,const Point& b) {
   starts.push_back(a);
   if(a.x()>.2)return DetourSearchResult{DetourSearchStatus::StartEndpointOccupied,{}};
   return contractDetour(a,b);
 };
 ASSERT_TRUE(buildCollisionAwareReference({{0,0,.5},{3,0,.5}},.1,contractObstacle,search,result,reason))<<reason;
 ASSERT_GT(starts.size(),1U);
 for(std::size_t j=1;j<starts.size();++j) {
   EXPECT_LT(starts[j].x(),starts[j-1].x());EXPECT_GE(starts[j].x(),0.);
 }
 EXPECT_TRUE(result.front().isApprox(Point(0,0,.5)));
 EXPECT_TRUE(result.back().isApprox(Point(3,0,.5)));
 for(std::size_t i=1;i<result.size();++i)
   EXPECT_TRUE(collisionFreeSegment(result[i-1],result[i],.05,contractObstacle));
}

TEST(CollisionReference, TargetRetryPreservesOriginalStairXYZAndEndpoint) {
 std::vector<Point> result,ends;std::string reason;
 const std::vector<Point> stairs{{0,0,.5},{1,0,.8},{3,0,1.2}};
 const scan_planner::DetourSearch search=[&](const Point& a,const Point& b) {
   ends.push_back(b);
   EXPECT_NEAR(b.z(),.8+(b.x()-1.)*.2,1e-12);
   if(b.x()<2.65)return DetourSearchResult{DetourSearchStatus::TargetEndpointOccupied,{}};
   return contractDetour(a,b);
 };
 ASSERT_TRUE(buildCollisionAwareReference(stairs,.1,contractObstacle,search,result,reason))<<reason;
 ASSERT_GT(ends.size(),1U);
 for(std::size_t j=1;j<ends.size();++j) {
   EXPECT_GT(ends[j].x(),ends[j-1].x());EXPECT_GT(ends[j].z(),ends[j-1].z());
 }
 EXPECT_TRUE(result.front().isApprox(stairs.front()));EXPECT_TRUE(result.back().isApprox(stairs.back()));
 for(std::size_t i=1;i<result.size();++i) {
   EXPECT_GE(result[i].z(),result[i-1].z()-1e-12);
   EXPECT_TRUE(collisionFreeSegment(result[i-1],result[i],.05,contractObstacle));
 }
}

TEST(CollisionReference, EndpointExhaustionIsBoundedAndDoesNotAcceptRejectedPath) {
 for(const auto status:{DetourSearchStatus::StartEndpointOccupied,DetourSearchStatus::TargetEndpointOccupied}) {
   std::vector<Point> result;std::string reason;int calls=0;
   const scan_planner::DetourSearch search=[&](const Point& a,const Point& b) {
     ++calls;EXPECT_GE(a.x(),0.);EXPECT_LE(b.x(),3.);
     return DetourSearchResult{status,{a,b}}; // A stale path cannot change a reject into success.
   };
   EXPECT_FALSE(buildCollisionAwareReference({{0,0,.5},{3,0,.5}},.1,contractObstacle,search,result,reason));
   EXPECT_GT(calls,1);EXPECT_LE(calls,61);EXPECT_EQ(reason,"failed_reference_search");
 }
}

TEST(CollisionReference, GenericFailureOrDeadlineDoesNotGetAnExtraRetry) {
 for(const bool endpoint_first:{false,true}) {
   std::vector<Point> result;std::string reason;int calls=0;
   const scan_planner::DetourSearch search=[&](const Point&,const Point&) {
     ++calls;
     if(endpoint_first&&calls==1)return DetourSearchResult{DetourSearchStatus::TargetEndpointOccupied,{}};
     return DetourSearchResult{};
   };
   EXPECT_FALSE(buildCollisionAwareReference({{0,0,.5},{3,0,.5}},.1,contractObstacle,search,result,reason));
   EXPECT_EQ(calls,endpoint_first?2:1);EXPECT_EQ(reason,"failed_reference_search");
 }
}

TEST(CollisionReference, IdenticalReferencePointsCannotStartAnAnchorRetryLoop) {
 std::vector<Point> result;std::string reason;int calls=0;
 const scan_planner::DetourSearch search=[&](const Point&,const Point&) {
   ++calls;return DetourSearchResult{DetourSearchStatus::TargetEndpointOccupied,{}};
 };
 EXPECT_FALSE(buildCollisionAwareReference({{0,0,.5},{0,0,.5}},.05,
     [](const Point&,double){return 0;},search,result,reason));
 EXPECT_EQ(calls,0);EXPECT_TRUE(result.empty());EXPECT_EQ(reason,"failed_reference_geometry");
}

TEST(CollisionReference,OffsetStraightApproachGoesForwardRatherThanPerpendicularlyBackToRoute) {
 const std::vector<Point> input{{0,.3,.55},{0,0,.55},{2,0,.55},{3,0,.55}};
 const auto result=scan_planner::measuredForwardApproach(input,.05,
     [](const Point&,double){return 0;},Point::Zero(),0.,true);
 ASSERT_GT(result.size(),4U);EXPECT_TRUE(result.front().isApprox(input.front()));
 const Point initial=result[1]-result[0];
 EXPECT_GT(initial.x(),0.);EXPECT_LT(std::abs(std::atan2(initial.y(),initial.x())),.01);
 EXPECT_LT(result[1].y(),input.front().y());
 EXPECT_TRUE(result.back().isApprox(input.back()));
 EXPECT_TRUE(input[1].isApprox(Point(0,0,.55))); // Task input not mutated.
}
TEST(CollisionReference,ForwardApproachPreservesCornerStairAndBlockedConnector) {
 const auto free=[](const Point&,double){return 0;};
 const std::vector<Point> corner{{0,.3,.55},{0,0,.55},{.5,0,.55},{.5,1,.55}};
 const auto result=scan_planner::measuredForwardApproach(corner,.05,free,Point::Zero(),0.,true);
 ASSERT_GT(result.size(),3U);
 EXPECT_TRUE(std::any_of(result.begin(),result.end(),[&](const Point& p){return p.isApprox(corner[2]);}));
 for(const auto& p:result)EXPECT_LE(p.x(),.5+1e-12);
 EXPECT_TRUE(result.back().isApprox(corner.back()));
 const std::vector<Point> stairs{{0,.3,.55},{0,0,.55},{.5,0,.75},{1,0,1.05}};
 const auto stair=scan_planner::measuredForwardApproach(stairs,.05,free,Point::Zero(),0.,true);
 ASSERT_EQ(stair.size(),stairs.size());for(std::size_t i=0;i<stairs.size();++i)EXPECT_TRUE(stair[i].isApprox(stairs[i]));
 const auto blocked=[](const Point& p,double){return p.x()>.2&&p.x()<.4?1:0;};
 const auto unchanged=scan_planner::measuredForwardApproach(corner,.05,blocked,Point::Zero(),0.,true);
 ASSERT_EQ(unchanged.size(),corner.size());for(std::size_t i=0;i<corner.size();++i)EXPECT_TRUE(unchanged[i].isApprox(corner[i]));
}

namespace {
std::vector<Point> offsetReference(double offset,double z_error=0.) {
 return {{0,offset,.55+z_error},{0,0,.55},{1,0,.55},{2,0,.55},{4,0,.55}};
}
struct FittedSeedMetrics { double curvature{},heading{};bool boundary_matches{};
 double minimum_forward_velocity{std::numeric_limits<double>::infinity()}; };
FittedSeedMetrics fittedSeedMetrics(const std::vector<Point>& original,
                                   const std::vector<Point>& guidance) {
 std::vector<Point> checked;std::string reason;
 const auto free=[](const Point&,double){return 0;};
 if(!buildCollisionAwareReference(guidance,.05,free,
     scan_planner::DetourSearch{[](const Point&,const Point&){return DetourSearchResult{};}},checked,reason))
   return {1e9,1e9,false};
 scan_planner::DiscreteReference path;path.set(checked);
 const auto seed=scan_planner::sampleReferenceSeed(path,.3,.35,.3,.3,.2);
 const scan_planner::CubicMotionBoundary boundary{
     original.front(),Point(.3,0,0),Point::Zero(),original.back(),Point(.3,0,0),Point::Zero()};
 const auto controls=scan_planner::fitCubicWithFixedBoundary(seed.samples,seed.dt,boundary);
 scan_planner::UniformBspline curve(controls,3,seed.dt);
 auto velocity=curve.getDerivative(),acceleration=velocity.getDerivative();
 FittedSeedMetrics result;result.boundary_matches=scan_planner::cubicBoundaryMatches(curve,boundary);
 for(int i=0;i<=2000;++i) {
   const double t=curve.getTimeSum()*i/2000.;
   const Point v=velocity.evaluateDeBoorT(t),a=acceleration.evaluateDeBoorT(t);
   result.minimum_forward_velocity=std::min(result.minimum_forward_velocity,v.x());
   const double speed=v.head<2>().norm();if(speed<.01)continue;
   result.curvature=std::max(result.curvature,std::abs(v.x()*a.y()-v.y()*a.x())/(speed*speed*speed));
   result.heading=std::max(result.heading,std::abs(std::atan2(v.y(),v.x())));
 }
 return result;
}
}

TEST(CollisionReference,SmallOffsetsAndHeightResidualHaveOneContinuousForwardInitializer) {
 const auto free=[](const Point&,double){return 0;};
 double previous_curvature=0.;
 for(const double offset:{.049,.051,-.08,.08})for(const double height:{0.,.03}) {
   const auto original=offsetReference(offset,height);
   const auto result=scan_planner::measuredForwardApproach(original,.05,free,Point(.3,0,0),0.,true);
   ASSERT_GT(result.size(),original.size());
   EXPECT_TRUE(result.front().isApprox(original.front()));
   EXPECT_TRUE(result.back().isApprox(original.back()));
   const Point first=result[1]-result[0];
   EXPECT_GT(first.x(),0.);
   EXPECT_LT(std::abs(std::atan2(first.y(),first.x())),.01);
   for(std::size_t i=1;i<result.size();++i)EXPECT_GE(result[i].x(),result[i-1].x()-1e-12);
   const auto metrics=fittedSeedMetrics(original,result);
   EXPECT_TRUE(metrics.boundary_matches);  // True measured P/V/A is not rounded or replaced.
   EXPECT_LT(metrics.curvature,1.);
   EXPECT_LT(metrics.heading,.2);
   if(offset==.049&&height==0.)previous_curvature=metrics.curvature;
   if(offset==.051&&height==0.)EXPECT_LT(std::abs(metrics.curvature-previous_curvature),.05);
 }
}

TEST(CollisionReference,MeasuredHeadingAndVelocityShapeOnlyTheSeedNotTheSourceRoute) {
 const auto free=[](const Point&,double){return 0;};
 const auto original=offsetReference(.08);
 const double yaw=.25;
 const Point v(.3*std::cos(yaw),.3*std::sin(yaw),0.);
 const auto result=scan_planner::measuredForwardApproach(original,.05,free,v,yaw,true);
 ASSERT_GT(result.size(),3U);
 const Point first=result[1]-result[0];
 EXPECT_NEAR(std::atan2(first.y(),first.x()),yaw,.01);
 EXPECT_TRUE(original[1].isApprox(Point(0,0,.55)));
 EXPECT_TRUE(result.back().isApprox(original.back()));
 const auto stopped=scan_planner::measuredForwardApproach(original,.05,free,Point::Zero(),yaw,true);
 const auto reverse=scan_planner::measuredForwardApproach(original,.05,free,-v,yaw,true);
 ASSERT_EQ(stopped.size(),reverse.size());
 for(std::size_t i=0;i<stopped.size();++i)EXPECT_TRUE(stopped[i].isApprox(reverse[i]));
}

TEST(CollisionReference,BlockedSmoothApproachKeepsNativeDetourInsteadOfCuttingObstacle) {
 const auto input=offsetReference(.08);
 const auto occupied=[](const Point& p,double) {
   return p.x()>.25&&p.x()<.55&&std::abs(p.y())<.2?1:0;
 };
 const auto guidance=scan_planner::measuredForwardApproach(input,.05,occupied,Point(.3,0,0),0.,true);
 ASSERT_EQ(guidance.size(),input.size());
 for(std::size_t i=0;i<input.size();++i)EXPECT_TRUE(guidance[i].isApprox(input[i]));
 int searches=0;std::vector<Point> checked;std::string reason;
 const scan_planner::DetourSearch search=[&](const Point& a,const Point& b) {
   ++searches;
   return DetourSearchResult{DetourSearchStatus::Success,
       {a,Point(a.x(),.4,a.z()),Point(b.x(),.4,b.z()),b}};
 };
 ASSERT_TRUE(buildCollisionAwareReference(guidance,.05,occupied,search,checked,reason))<<reason;
 EXPECT_GT(searches,0);EXPECT_EQ(reason,"reference_detour_ready");
 for(std::size_t i=1;i<checked.size();++i)
   EXPECT_TRUE(collisionFreeSegment(checked[i-1],checked[i],.025,occupied));
}

TEST(CollisionReference,ForwardApproachRetainsOriginalStairEntryAndAllFollowingHeights) {
 const std::vector<Point> input{{0,.08,.58},{0,0,.55},{1,0,.55},{1.5,0,.80},{2,0,1.05}};
 const auto result=scan_planner::measuredForwardApproach(input,.05,
     [](const Point&,double){return 0;},Point(.3,0,0),0.,true);
 ASSERT_GT(result.size(),input.size());
 for(std::size_t i=2;i<input.size();++i)
   EXPECT_TRUE(std::any_of(result.begin(),result.end(),[&](const Point& p){return p.isApprox(input[i]);}));
 EXPECT_TRUE(result.front().isApprox(input.front()));
 EXPECT_TRUE(result.back().isApprox(input.back()));
}

TEST(CollisionReference,MillimetricHeightVertexDoesNotRestoreSidewaysProjection) {
 const auto free=[](const Point&,double){return 0;};
 double worst_curvature=0.,worst_heading=0.,minimum_forward=1.;
 for(double offset:{-.25,-.15,.15,.25})for(double body_height:{-.03,0.,.03}) {
   std::vector<Point> input{{0.,offset,.55+body_height},{0.,0.,.55}};
   for(int i=1;i<=20;++i)input.emplace_back(i*.1,0.,.55+(i==1?.004:0.));
   const auto result=scan_planner::measuredForwardApproach(input,.05,free,Point(.3,0,0),0.,true);
   EXPECT_GT(result.size(),input.size());
   EXPECT_TRUE(result.front().isApprox(input.front()));EXPECT_TRUE(result.back().isApprox(input.back()));
   const Point entry=result[1]-result[0];EXPECT_GT(entry.x(),0.);
   EXPECT_LT(std::abs(std::atan2(entry.y(),entry.x())),.01);
   for(std::size_t i=1;i<result.size();++i) {
     EXPECT_GE(result[i].x(),result[i-1].x()-1e-12);
     if(result[i].x()<1.)EXPECT_LE((result[i]-result[i-1]).norm(),.025+1e-12);
   }
   if(body_height==0.)EXPECT_NEAR(std::max_element(result.begin(),result.end(),
       [](const Point&a,const Point&b){return a.z()<b.z();})->z(),.554,1e-12);
   const auto fit=fittedSeedMetrics(input,result);
   worst_curvature=std::max(worst_curvature,fit.curvature);worst_heading=std::max(worst_heading,fit.heading);
   minimum_forward=std::min(minimum_forward,fit.minimum_forward_velocity);
   EXPECT_TRUE(fit.boundary_matches);EXPECT_LT(fit.curvature,2.);EXPECT_LT(fit.heading,.5);
   EXPECT_GT(fit.minimum_forward_velocity,0.);
 }
 RecordProperty("maximum_fitted_curvature",std::to_string(worst_curvature));
 RecordProperty("maximum_fitted_heading",std::to_string(worst_heading));
 RecordProperty("minimum_fitted_forward_velocity",std::to_string(minimum_forward));
}

TEST(CollisionReference,HeightProfileKeepsShortPeakAndActualCornerWithoutClearingWall) {
 const std::vector<Point> input{{0,.15,.55},{0,0,.55},{.10000,0,.55},{.10002,0,.56},
   {.10004,0,.55},{.5,0,.55},{.5,1,.55}};
 const auto free=[](const Point&,double){return 0;};
 const auto result=scan_planner::measuredForwardApproach(input,.05,free,Point(.3,0,0),0.,true);
 ASSERT_GT(result.size(),input.size());
 EXPECT_TRUE(std::any_of(result.begin(),result.end(),[](const Point&p){return std::abs(p.z()-.56)<1e-12;}));
 EXPECT_TRUE(std::any_of(result.begin(),result.end(),[&](const Point&p){return p.isApprox(input[5]);}));
 EXPECT_TRUE(result.back().isApprox(input.back()));for(const auto&p:result)EXPECT_LE(p.x(),.5+1e-12);
 const auto wall=[](const Point&p,double){return p.z()>.559?1:0;};
 const auto blocked=scan_planner::measuredForwardApproach(input,.05,wall,Point(.3,0,0),0.,true);
 ASSERT_EQ(blocked.size(),input.size());for(std::size_t i=0;i<input.size();++i)EXPECT_TRUE(blocked[i].isApprox(input[i]));
}

TEST(CollisionReference,NativeBoxHeightVertexForwardApproachPreservesWalls) {
 auto map=GridMapTestAccess::boxMap();
 for(int x=-12;x<=55;++x)for(double y:{-.575,.575})map->setOccupied({x*.05,y,.55});
 const auto occupied=[&](const Point&p,double yaw){return map->getPlanningInflateOccupancy(p,yaw);};
 std::vector<Point> input{{0,.15,.55},{0,0,.55}};
 for(int i=1;i<=20;++i)input.emplace_back(i*.1,0.,.55+(i==1?.004:0.));
 ASSERT_EQ(occupied(input.front(),0.),0);ASSERT_EQ(occupied(input.front(),-M_PI/2.),1);
 const auto shaped=scan_planner::measuredForwardApproach(input,.05,occupied,Point(.3,0,0),0.,true);
 ASSERT_GT(shaped.size(),input.size());
 for(std::size_t i=1;i<shaped.size();++i)EXPECT_TRUE(collisionFreeSegment(shaped[i-1],shaped[i],.025,occupied));
 // A real wall remains a wall; this initializer cannot clear or label it free.
 EXPECT_EQ(occupied({1.,.575,.55},0.),1);
 map->setOccupied({.6,0.,.55});
 const auto blocked=scan_planner::measuredForwardApproach(input,.05,occupied,Point(.3,0,0),0.,true);
 ASSERT_EQ(blocked.size(),input.size());for(std::size_t i=0;i<input.size();++i)EXPECT_TRUE(blocked[i].isApprox(input[i]));
}

TEST(CollisionReference,ShortHeightPeakAtJoinDoesNotExhaustPlanarSamplingBudget) {
 const std::vector<Point> input{{0,.15,.55},{0,0,.55},{.1,0,.554},{.2,0,.55},
     {1.09998,0,.55},{1.1,0,.554},{1.10002,0,.55},{2.,0,.55}};
 const auto free=[](const Point&,double){return 0;};
 const auto result=scan_planner::measuredForwardApproach(input,.05,free,Point(.3,0,0),0.,true);
 ASSERT_GT(result.size(),input.size());ASSERT_LT(result.size(),2049U);
 const Point entry=result[1]-result[0];EXPECT_GT(entry.x(),0.);
 EXPECT_LT(std::abs(std::atan2(entry.y(),entry.x())),.01);
 EXPECT_TRUE(std::any_of(result.begin(),result.end(),[&](const Point&p){return p.isApprox(input[5]);}));
 EXPECT_TRUE(result.back().isApprox(input.back()));
}

TEST(CollisionReference,RealSliceOnRouteOrOffsetNeverDropsFirstCornerAsProjection) {
 scan_planner::DiscreteReference route;route.set({{0,0,.55},{.5,0,.55},{.5,1,.55}});
 const auto free=[](const Point&,double){return 0;};
 for(const double lateral:{0.,.08}) {
   const Point actual(0,lateral,.55),velocity(.03,0,0);
   const auto slice=route.slice(0.,route.length(),actual);
   const bool has_connector=(actual-route.sample(0.)).norm()>1e-6;
   EXPECT_EQ(slice.size(),has_connector?4U:3U);
   const auto boundary=scan_planner::selectForwardMotionBoundary(
       actual,velocity,slice,true,false,0.,has_connector);
   EXPECT_TRUE(boundary.reference_direction.isApprox(Point::UnitX()));
   EXPECT_FALSE(boundary.regularized);EXPECT_TRUE(boundary.velocity.isApprox(velocity));
   const auto result=scan_planner::measuredForwardApproach(slice,.05,free,velocity,0.,has_connector);
   EXPECT_TRUE(std::any_of(result.begin(),result.end(),
       [](const Point& p){return p.isApprox(Point(.5,0,.55));}));
   EXPECT_TRUE(result.front().isApprox(actual));EXPECT_TRUE(result.back().isApprox(route.points.back()));
 }
}

TEST(CollisionReference,UnflaggedRealSidePathAndShortEndpointKeepTheirFirstSegments) {
 const Point actual(0,0,.55),velocity(.03,0,0);
 const std::vector<Point> real_side{actual,{.5,0,.55},{.5,1,.55}};
 const auto boundary=scan_planner::selectForwardMotionBoundary(actual,velocity,real_side,true,false,0.);
 EXPECT_TRUE(boundary.reference_direction.isApprox(Point::UnitX()));
 EXPECT_TRUE(boundary.velocity.isApprox(velocity));
 const auto free=[](const Point&,double){return 0;};
 const auto side=scan_planner::measuredForwardApproach(real_side,.05,free,velocity,0.);
 EXPECT_TRUE(std::any_of(side.begin(),side.end(),
     [](const Point& p){return p.isApprox(Point(.5,0,.55));}));
 const std::vector<Point> short_route{actual,{.15,0,.55}};
 const auto short_result=scan_planner::measuredForwardApproach(short_route,.05,free,velocity,0.);
 ASSERT_EQ(short_result.size(),2U);
 EXPECT_TRUE(short_result.front().isApprox(actual));EXPECT_TRUE(short_result.back().isApprox(short_route.back()));
}

TEST(CollisionReference,RealSliceAtStairStartNeverSkipsFirstSlopeForFollowingFlatLanding) {
 scan_planner::DiscreteReference route;route.set({{0,0,.55},{.5,0,.8},{1,0,.8}});
 const Point actual=route.points.front(),velocity(.03,0,.015);
 const auto slice=route.slice(0.,route.length(),actual);
 const auto boundary=scan_planner::selectForwardMotionBoundary(actual,velocity,slice,true,false,0.,false);
 EXPECT_NEAR(boundary.reference_direction.z(),.5,1e-12);
 EXPECT_TRUE(boundary.velocity.isApprox(velocity));
 const auto result=scan_planner::measuredForwardApproach(slice,.05,
     [](const Point&,double){return 0;},velocity,0.,false);
 ASSERT_EQ(result.size(),slice.size());
 for(std::size_t i=0;i<slice.size();++i)EXPECT_TRUE(result[i].isApprox(slice[i]));
}

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

namespace {
// All three points fit in one 10 cm lattice cell. The horizontal reference
// footprint is blocked while the turned connectors are clear. This emulates
// a same-cell search without equating free endpoints with free connectors.
const std::vector<Point> sameCellReference{{0,0,.55},{.02,0,.55},{.02,.04,.55}};
int horizontalFootprintBlocked(const Point &,double yaw) {
  return std::abs(std::sin(yaw))<.25 ? 1:0;
}
}

TEST(CollisionReference, SuccessfulOneCellSearchChecksBothExactConnectors) {
  std::vector<Point> result;
  std::string reason;
  int searches=0;
  const auto search=[&](const Point &a,const Point &b) {
    ++searches;
    EXPECT_TRUE(a.isApprox(sameCellReference.front()));
    EXPECT_TRUE(b.isApprox(sameCellReference[1]));
    return DetourSearchResult{DetourSearchStatus::Success,{Point(0,.02,.55)}};
  };
  ASSERT_TRUE(buildCollisionAwareReference(sameCellReference,.1,horizontalFootprintBlocked,
      search,result,reason,0.)) << reason;
  EXPECT_EQ(searches,1);
  EXPECT_EQ(reason,"reference_detour_ready");
  ASSERT_GE(result.size(),4U);
  EXPECT_TRUE(result.front().isApprox(sameCellReference.front()));
  EXPECT_TRUE(result.back().isApprox(sameCellReference.back()));
  for (std::size_t i=1;i<result.size();++i)
    EXPECT_TRUE(collisionFreeSegment(result[i-1],result[i],.05,horizontalFootprintBlocked));
}

TEST(CollisionReference, OneCellResultDoesNotBypassEntryCollision) {
  std::vector<Point> result;
  std::string reason;
  const auto search=[](const Point &,const Point &) {
    return DetourSearchResult{DetourSearchStatus::Success,{Point(.01,0,.55)}};
  };
  EXPECT_FALSE(buildCollisionAwareReference(sameCellReference,.1,horizontalFootprintBlocked,
      search,result,reason,0.));
  EXPECT_EQ(reason,"failed_reference_search_collision");
}

TEST(CollisionReference, OneCellResultDoesNotBypassExitCollision) {
  std::vector<Point> result;
  std::string reason;
  const auto search=[](const Point &,const Point &) {
    return DetourSearchResult{DetourSearchStatus::Success,{Point(0,.02,.55)}};
  };
  const auto occupied=[](const Point &p,double yaw) {
    if ((p-Point(.02,0,.55)).norm()<1e-9 && yaw<-.1) return 1;
    return horizontalFootprintBlocked(p,yaw);
  };
  EXPECT_FALSE(buildCollisionAwareReference(sameCellReference,.1,occupied,search,result,reason,0.));
  EXPECT_EQ(reason,"failed_reference_search_collision");
}

TEST(CollisionReference, OneCellResultDoesNotBypassUnknownConnector) {
  std::vector<Point> result;
  std::string reason;
  const auto search=[](const Point &,const Point &) {
    return DetourSearchResult{DetourSearchStatus::Success,{Point(0,.02,.55)}};
  };
  const auto occupied=[](const Point &p,double yaw) {
    if ((p-Point(0,.02,.55)).norm()<1e-9) return -1;
    return horizontalFootprintBlocked(p,yaw);
  };
  EXPECT_FALSE(buildCollisionAwareReference(sameCellReference,.1,occupied,search,result,reason,0.));
  EXPECT_EQ(reason,"failed_reference_search_collision");
}

TEST(CollisionReference, FailureStatusCannotReuseNonemptySearchPath) {
  std::vector<Point> result;
  std::string reason;
  const auto search=[](const Point &,const Point &) {
    return DetourSearchResult{DetourSearchStatus::Failure,{Point(0,.02,.55)}};
  };
  EXPECT_FALSE(buildCollisionAwareReference(sameCellReference,.1,horizontalFootprintBlocked,
      search,result,reason,0.));
  EXPECT_EQ(reason,"failed_reference_search");
}

TEST(CollisionReference, SuccessWithEmptyOrOversizedPathIsRejected) {
  for (const std::size_t count : {0U,20001U}) {
    std::vector<Point> result;
    std::string reason;
    const auto search=[count](const Point &,const Point &) {
      return DetourSearchResult{DetourSearchStatus::Success,
          std::vector<Point>(count,Point(0,.02,.55))};
    };
    EXPECT_FALSE(buildCollisionAwareReference(sameCellReference,.1,horizontalFootprintBlocked,
        search,result,reason,0.));
    EXPECT_EQ(reason,"failed_reference_search");
  }
}

TEST(CollisionReference, NonfiniteSearchPointIsRejected) {
  std::vector<Point> result;
  std::string reason;
  const auto search=[](const Point &,const Point &) {
    return DetourSearchResult{DetourSearchStatus::Success,
        {Point(std::numeric_limits<double>::quiet_NaN(),.02,.55)}};
  };
  EXPECT_FALSE(buildCollisionAwareReference(sameCellReference,.1,horizontalFootprintBlocked,
      search,result,reason,0.));
  EXPECT_EQ(reason,"failed_reference_search_collision");
}
