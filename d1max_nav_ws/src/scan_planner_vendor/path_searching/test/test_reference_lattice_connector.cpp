#include <gtest/gtest.h>
#include <path_searching/dyn_a_star.h>
#include <path_searching/search_lattice.hpp>
#include <cmath>

// Native GridMap, its real inflation kernel and the production AStar. No ROS
// init, mock occupancy callback or second/simplified map implementation.
using Point=Eigen::Vector3d;
struct GridMapTestAccess {
  static GridMap::Ptr make(bool strict=false) {
    auto map=std::make_shared<GridMap>();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={120,160,48};
    p.map_origin_idx_={0,40,0};map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.require_observed_free_=strict;p.planning_collision_policy_=strict?"observed_free":"official";
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.45;
    const std::size_t size=120U*160U*48U;
    d.occupancy_buffer_.assign(size,strict?-1.:-1.01);
    d.occupancy_buffer_inflate_.assign(size,0);d.occupancy_buffer_inflate_cnt_.assign(size,0);
    map->rebuildInflationOffsets();return map;
  }
  static void unknown(GridMap &map,const Point &point) {
    Eigen::Vector3i index;map.posToIndex(point,index);
    map.md_.occupancy_buffer_[map.toAddress(index)]=-1.01;
  }
  static GridMap::Ptr d1BoxCorridor(bool walls=true) {
    auto map=make();auto &p=map->mp_;
    p.map_size_=p.map_voxel_num_.cast<double>()*.05;
    p.map_origin_idx_={0,400,0};map->updateMapBoundaryFromIndex();
    p.collision_model_="oriented_box";p.box_length_=.93;p.box_width_=.30;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    std::fill(map->md_.occupancy_buffer_.begin(),map->md_.occupancy_buffer_.end(),-1.);
    map->rebuildInflationOffsets();
    if(walls)for(int y=320;y<480;++y) for(double x:{-.425,.425})
      map->setOccupied({x,(y+.5)*.05,-.125});
    return map;
  }
  static void strict(GridMap &map) {
    map.mp_.require_observed_free_=true;map.mp_.planning_collision_policy_="observed_free";
  }
  static GridMap::Ptr d1NarrowEntrance() {
    auto map=d1BoxCorridor(false);
    map->mp_.box_length_=1.03;map->mp_.box_width_=.40;
    // A deterministic fixture, not a claim about measured site clearance:
    // 60 cm clear entrance opening into a landing, with the actual body model.
    // The 40 cm-wide D1 fits when aligned; a diagonal start-to-goal chord does
    // not. These are actual occupied cells, not a mocked collision callback.
    for(int y=320;y<377;++y)for(double x:{-.325,.325})
      map->setOccupied({x,(y+.5)*.05,-.125});
    return map;
  }
  static GridMap::Ptr recordedStairWindow() {
    auto map=d1BoxCorridor(false);
    // The recorded odom endpoints, with a deterministic observed-free map,
    // not a reconstruction of the unavailable live voxel/ray snapshot.
    map->mp_.map_origin_idx_={820,-940,30};map->updateMapBoundaryFromIndex();
    map->mp_.box_length_=.98;map->mp_.box_width_=.35;
    map->rebuildInflationOffsets();return map;
  }
};

namespace {
struct RecordedEndpoints {Point start,end,obstacle;};
const std::vector<RecordedEndpoints> recorded{
  // scan.log replan 65 / 1790459832.819903615; true body was (.468,1.3,.0288).
  {{.5895,1.9331,.0343},{.5385,3.6349,.0251},{.875,2.125,.025}},
  // scan.log replan 68 / 1790459836.552673583; another reference subsegment.
  {{.5932,1.7957,.0351},{.5389,3.6195,.0253},{.875,2.025,.025}}
};
const std::vector<std::pair<Point,Point>> recorded_short_slopes{
  // 6855a7d.../scan.log:39158-39160, 21:56:23.239-240.
  {{41.1496797,-47.015827,1.54355193},{41.1142381,-47.0091852,1.58992909}},
  // Same session scan.log:39894-39896, 21:56:38.502-503.
  {{41.1495375,-47.0074963,1.54566969},{41.1183636,-47.0028517,1.60574631}}
};

Point snap(const Point &point,const Point &a,const Point &b) {
  const Point centre=(a+b)*.5;
  Eigen::Vector3i index;
  EXPECT_TRUE(scan_planner::nearestSearchIndex(point,centre,.05,{50,50,16},{100,100,32},index));
  return (index-Eigen::Vector3i(50,50,16)).cast<double>()*.05+centre;
}

void checkWholePath(GridMap &map,const std::vector<Point> &path,const Point &start,const Point &end) {
  ASSERT_GE(path.size(),2U); // An exact collision-free straight edge needs no snapping point.
  EXPECT_LT((path.front()-start).norm(),1e-12);
  EXPECT_LT((path.back()-end).norm(),1e-12);
  // Include exact connectors, at four times finer spacing than production.
  for(std::size_t i=1;i<path.size();++i) {
    const Point delta=path[i]-path[i-1];
    if(delta.norm()<1e-9) continue;
    const double yaw=std::atan2(delta.y(),delta.x());
    const int samples=std::max(1,static_cast<int>(std::ceil(delta.norm()/.00625)));
    for(int k=0;k<=samples;++k)
      EXPECT_EQ(map.getInflateOccupancy(path[i-1]+delta*(static_cast<double>(k)/samples),yaw),0)
          <<"segment="<<i<<" sample="<<k;
  }
}
} // namespace

TEST(ReferenceLatticeConnector, RecordedFloatAnchorsRetainExactEndpointsAndRecoverBothDirections) {
  for(const auto &case_:recorded) {
    auto map=GridMapTestAccess::make();map->setOccupied(case_.obstacle);
    const auto &a=case_.start;const auto &b=case_.end;
    const double yaw=std::atan2(b.y()-a.y(),b.x()-a.x());
    ASSERT_EQ(map->getInflateOccupancy(a,yaw),0);
    ASSERT_EQ(map->getInflateOccupancy(snap(a,a,b),yaw),1);
    ASSERT_EQ(map->getInflateOccupancy(b,yaw),0);
    AStar planner;planner.initGridMap(map,{100,100,32});
    // Legacy external mode is unchanged; the production reference opts in.
    EXPECT_EQ(planner.AstarSearch(.05,a,b,false),ASTAR_RET::INIT_LATTICE_OCCUPIED);
    ASSERT_EQ(planner.AstarSearch(.05,a,b,false,true),ASTAR_RET::SUCCESS);
    checkWholePath(*map,planner.getPath(),a,b);
    ASSERT_EQ(planner.AstarSearch(.05,b,a,false,true),ASTAR_RET::SUCCESS);
    checkWholePath(*map,planner.getPath(),b,a);
  }
}

TEST(ReferenceLatticeConnector, ExactOccupiedStartAndTargetCannotBeMovedAway) {
  for(bool start:{true,false}) {
    const auto &c=recorded.front();auto map=GridMapTestAccess::make();
    map->setOccupied(start?c.start:c.end);
    AStar planner;planner.initGridMap(map,{100,100,32});
    EXPECT_EQ(planner.AstarSearch(.05,c.start,c.end,false,true),start?
              ASTAR_RET::INIT_START_OCCUPIED:ASTAR_RET::INIT_TARGET_OCCUPIED);
    EXPECT_TRUE(planner.getPath().empty());
  }
}

TEST(ReferenceLatticeConnector, ExactStrictUnknownAndOutsideRemainRejected) {
  const auto &c=recorded.front();
  auto strict=GridMapTestAccess::make(true);
  GridMapTestAccess::unknown(*strict,c.start);
  AStar planner;planner.initGridMap(strict,{100,100,32});
  EXPECT_EQ(planner.AstarSearch(.05,c.start,c.end,false,true),ASTAR_RET::INIT_UNOBSERVED);
  EXPECT_TRUE(planner.getPath().empty());
  auto official=GridMapTestAccess::make();AStar other;other.initGridMap(official,{100,100,32});
  EXPECT_EQ(other.AstarSearch(.05,{-3.,0.,0.},{-2.,1.,0.},false,true),ASTAR_RET::INIT_OUTSIDE_MAP);
  EXPECT_TRUE(other.getPath().empty());
}

TEST(ReferenceLatticeConnector, RecoveryDoesNotCrossSealedWallOrLeakAnOldSuccessfulPath) {
  const auto &c=recorded.front();auto map=GridMapTestAccess::make();map->setOccupied(c.obstacle);
  AStar planner;planner.initGridMap(map,{100,100,32});
  ASSERT_EQ(planner.AstarSearch(.05,c.start,c.end,false,true),ASTAR_RET::SUCCESS);
  for(int x=-60;x<60;++x) map->setOccupied({(x+.5)*.05,2.775,.025});
  EXPECT_EQ(planner.AstarSearch(.05,c.start,c.end,false,true),ASTAR_RET::SEARCH_ERR);
  EXPECT_TRUE(planner.getPath().empty());
}

TEST(ReferenceLatticeConnector, HeadingUnsafeConnectorIsNotReplacedByFreeEndpointOnlyCheck) {
  const auto &c=recorded.front();auto map=GridMapTestAccess::make();map->setOccupied(c.obstacle);
  // The nearest free lattice cell is to the LEFT. The exact-to-cell connector
  // points almost west, so the rear cylinder swings into the real obstacle.
  const Point nearest_free=snap(c.start,c.start,c.end)-Point(.05,0.,0.);
  const double path_yaw=std::atan2(c.end.y()-c.start.y(),c.end.x()-c.start.x());
  const Point delta=nearest_free-c.start;
  const double connector_yaw=std::atan2(delta.y(),delta.x());
  ASSERT_EQ(map->getInflateOccupancy(nearest_free,path_yaw),0);
  ASSERT_EQ(map->getInflateOccupancy(c.start,connector_yaw),1);
  AStar planner;planner.initGridMap(map,{100,100,32});
  ASSERT_EQ(planner.AstarSearch(.05,c.start,c.end,false,true),ASTAR_RET::SUCCESS);
  const auto path=planner.getPath();ASSERT_GE(path.size(),2U);
  EXPECT_GT((path[1]-nearest_free).norm(),.01);
  checkWholePath(*map,path,c.start,c.end);
}

TEST(ReferenceLatticeConnector, FreeNearestCellWithBlockedLinkUsesTheSameBoundedRecovery) {
  const auto &c=recorded.front();auto map=GridMapTestAccess::make();
  map->setOccupied({.925,1.975,.025});
  const Point nearest=snap(c.start,c.start,c.end);
  const double path_yaw=std::atan2(c.end.y()-c.start.y(),c.end.x()-c.start.x());
  const Point delta=nearest-c.start;
  const double connector_yaw=std::atan2(delta.y(),delta.x());
  ASSERT_EQ(map->getInflateOccupancy(c.start,path_yaw),0);
  ASSERT_EQ(map->getInflateOccupancy(nearest,path_yaw),0);
  ASSERT_EQ(map->getInflateOccupancy(c.start,connector_yaw),1);
  AStar planner;planner.initGridMap(map,{100,100,32});
  ASSERT_EQ(planner.AstarSearch(.05,c.start,c.end,false,true),ASTAR_RET::SUCCESS);
  const auto path=planner.getPath();ASSERT_GE(path.size(),2U);
  EXPECT_GT((path[1]-nearest).norm(),.01);
  checkWholePath(*map,path,c.start,c.end);
  // Reverse direction must validate the incoming connector too.
  ASSERT_EQ(planner.AstarSearch(.05,c.end,c.start,false,true),ASTAR_RET::SUCCESS);
  checkWholePath(*map,planner.getPath(),c.end,c.start);
}

TEST(ReferenceLatticeConnector, StrictUnobservedSnapCellBesideObservedFreeEndpointRecoversOnlyThroughFree) {
  const auto &c=recorded.front();
  const double yaw=std::atan2(c.end.y()-c.start.y(),c.end.x()-c.start.x());
  const Point lattice=snap(c.end,c.start,c.end);
  // Find one real voxel that the snapped cell's body envelope reaches but the
  // exact endpoint's does not: the recorded static_box_08 lattice artifact.
  bool found=false;GridMap::Ptr map;
  for(int dx=-10;dx<=10&&!found;++dx) for(int dy=-10;dy<=10&&!found;++dy) for(int dz=-12;dz<=12&&!found;++dz) {
    map=GridMapTestAccess::make(true);
    GridMapTestAccess::unknown(*map,lattice+Point(dx,dy,dz)*.05);
    found=map->getInflateOccupancy(c.end,yaw)==0&&map->getInflateOccupancy(lattice,yaw)==2;
  }
  ASSERT_TRUE(found);
  AStar planner;planner.initGridMap(map,{100,100,32});
  // Without the reference opt-in the missing observation is still reported.
  EXPECT_EQ(planner.AstarSearch(.05,c.start,c.end,false),ASTAR_RET::INIT_UNOBSERVED);
  ASSERT_EQ(planner.AstarSearch(.05,c.start,c.end,false,true),ASTAR_RET::SUCCESS);
  checkWholePath(*map,planner.getPath(),c.start,c.end);
  // An unobserved EXACT endpoint is never moved away (same map, plus the
  // exact endpoint's own voxel missing).
  auto exact=GridMapTestAccess::make(true);
  GridMapTestAccess::unknown(*exact,c.end);
  AStar strict;strict.initGridMap(exact,{100,100,32});
  EXPECT_NE(exact->getInflateOccupancy(c.end,yaw),0);
  EXPECT_EQ(strict.AstarSearch(.05,c.start,c.end,false,true),ASTAR_RET::INIT_UNOBSERVED);
  EXPECT_TRUE(strict.getPath().empty());
}

TEST(ReferenceLatticeConnector, D1BoxRecordedAnchorsKeepExactEndpointsWithoutSnapTurn) {
  // Actual 21:32:49 endpoint coordinates, with a deterministic synthetic
  // corridor. This reproduces the discretization, not the missing real map.
  auto map=GridMapTestAccess::d1BoxCorridor();
  const Point a(-.0988,19.6841,-.1143),b(.0238,20.2757,-.1475);
  const double yaw=std::atan2(b.y()-a.y(),b.x()-a.x());
  EXPECT_EQ(map->getPlanningInflateOccupancy(a,yaw),0);
  EXPECT_EQ(map->getPlanningInflateOccupancy(b,yaw),0);
  // The 25 mm snapping displacement is NOT the robot's requested heading.
  const Point snapped(-.075,19.675,-.125);
  EXPECT_EQ(map->getPlanningInflateOccupancy(a,std::atan2(snapped.y()-a.y(),snapped.x()-a.x())),1);
  for(bool reverse:{false,true}) {
    AStar search;search.initGridMapFromEnvironment(map);
    const Point start=reverse?b:a,end=reverse?a:b;
    ASSERT_EQ(search.AstarSearch(.05,start,end,false,true),ASTAR_RET::SUCCESS);
    const auto path=search.getPath();ASSERT_GE(path.size(),2U);
    EXPECT_LT((path.front()-start).norm(),1e-12);
    EXPECT_LT((path.back()-end).norm(),1e-12);
    // Every returned edge, including the actual first/last edge, is checked
    // at its own bearing. No forced endpoint yaw can hide a real collision.
    for(std::size_t i=1;i<path.size();++i) {
      const Point delta=path[i]-path[i-1];
      const double edge_yaw=std::atan2(delta.y(),delta.x());
      const int n=std::max(1,int(std::ceil(delta.norm()/.00625)));
      for(int k=0;k<=n;++k)
        EXPECT_EQ(map->getPlanningInflateOccupancy(path[i-1]+delta*(double(k)/n),edge_yaw),0);
    }
  }
}

TEST(ReferenceLatticeConnector, D1BoxSubvoxelReferenceDoesNotDriveToCentreAndBack) {
  auto map=GridMapTestAccess::d1BoxCorridor();
  const Point a(.0238,20.2757,-.1475),b(.0238,20.2837,-.1475);
  for(bool reverse:{false,true}) {
    AStar search;search.initGridMapFromEnvironment(map);
    const Point start=reverse?b:a,end=reverse?a:b;
    ASSERT_EQ(search.AstarSearch(.05,start,end,false,true),ASTAR_RET::SUCCESS);
    const auto path=search.getPath();ASSERT_EQ(path.size(),2U);
    EXPECT_LT((path.front()-start).norm(),1e-12);
    EXPECT_LT((path.back()-end).norm(),1e-12);
    EXPECT_NEAR((path.back()-path.front()).norm(),.008,1e-12);
  }
}

TEST(ReferenceLatticeConnector, D1BoxTrueTurnStillUsesItsOwnBearing) {
  auto map=GridMapTestAccess::d1BoxCorridor();
  const Point a(-.0988,19.6841,-.1143),b(.1238,19.6841,-.1143);
  // Both poses fit along the corridor. The requested real sideways heading
  // does not: the body cannot be frozen along the old reference to pass it.
  EXPECT_EQ(map->getPlanningInflateOccupancy(a,std::acos(-1.)*.5),0);
  EXPECT_EQ(map->getPlanningInflateOccupancy(b,std::acos(-1.)*.5),0);
  EXPECT_EQ(map->getPlanningInflateOccupancy(a,0.),1);
  AStar search;search.initGridMapFromEnvironment(map);
  EXPECT_EQ(search.AstarSearch(.05,a,b,false,true),ASTAR_RET::INIT_START_OCCUPIED);
  EXPECT_TRUE(search.getPath().empty());
}

TEST(ReferenceLatticeConnector, D1BoxMidCorridorObstacleAndSealedWallCannotBeSkipped) {
  const Point a(-.0988,19.6841,-.1143),b(.0238,21.2757,-.1475);
  for(bool sealed:{false,true}) {
    auto map=GridMapTestAccess::d1BoxCorridor();
    if(sealed)for(int x=-8;x<8;++x) map->setOccupied({(x+.5)*.05,20.475,-.125});
    else map->setOccupied({.025,20.475,-.125});
    const double yaw=std::atan2(b.y()-a.y(),b.x()-a.x());
    ASSERT_EQ(map->getPlanningInflateOccupancy(a,yaw),0);
    ASSERT_EQ(map->getPlanningInflateOccupancy(b,yaw),0);
    ASSERT_EQ(map->getPlanningInflateOccupancy({.025,20.475,-.125},yaw),1);
    AStar search;search.initGridMapFromEnvironment(map);
    const auto result=search.AstarSearch(.05,a,b,false,true);
    if(result==ASTAR_RET::SUCCESS) {
      ASSERT_FALSE(sealed);
      // A nonsealed obstacle is allowed only through a real, fully checked
      // detour. In particular the direct edge cannot ignore its middle.
      const auto path=search.getPath();ASSERT_GT(path.size(),2U);
      checkWholePath(*map,path,a,b);
    } else EXPECT_TRUE(search.getPath().empty());
  }
}

TEST(ReferenceLatticeConnector, D1BoxStrictUnknownRemainsUnknownAtExactPose) {
  auto map=GridMapTestAccess::d1BoxCorridor();GridMapTestAccess::strict(*map);
  const Point a(-.0988,19.6841,-.1143),b(.0238,20.2757,-.1475);
  GridMapTestAccess::unknown(*map,a);
  AStar search;search.initGridMapFromEnvironment(map);
  EXPECT_EQ(search.AstarSearch(.05,a,b,false,true),ASTAR_RET::INIT_UNOBSERVED);
  EXPECT_TRUE(search.getPath().empty());
}

TEST(ReferenceLatticeConnector, D1BoxMiddleUnknownCannotBeIgnoredByDirectEdge) {
  auto map=GridMapTestAccess::d1BoxCorridor();GridMapTestAccess::strict(*map);
  const Point a(-.0988,19.6841,-.1143),b(.0238,21.2757,-.1475);
  for(int x=-8;x<8;++x)GridMapTestAccess::unknown(*map,{(x+.5)*.05,20.475,-.125});
  const double yaw=std::atan2(b.y()-a.y(),b.x()-a.x());
  ASSERT_EQ(map->getPlanningInflateOccupancy(a,yaw),0);
  ASSERT_EQ(map->getPlanningInflateOccupancy(b,yaw),0);
  ASSERT_EQ(map->getPlanningInflateOccupancy({.025,20.475,-.125},yaw),2);
  AStar search;search.initGridMapFromEnvironment(map);
  EXPECT_NE(search.AstarSearch(.05,a,b,false,true),ASTAR_RET::SUCCESS);
  EXPECT_TRUE(search.getPath().empty());
}

TEST(ReferenceLatticeConnector, D1BoxBlockedDirectEdgeStillFindsNativeTurningDetour) {
  auto map=GridMapTestAccess::d1BoxCorridor(false);
  const Point a(-1.013,20.011,-.125),b(1.008,20.018,-.125);
  map->setOccupied({.025,20.025,-.125});
  AStar search;search.initGridMapFromEnvironment(map);
  ASSERT_EQ(search.AstarSearch(.05,a,b,false,true),ASTAR_RET::SUCCESS);
  const auto path=search.getPath();ASSERT_GT(path.size(),2U);
  double detour=0.;for(const auto &p:path)detour=std::max(detour,std::abs(p.y()-20.));
  EXPECT_GT(detour,.15);
  checkWholePath(*map,path,a,b);
}

TEST(ReferenceLatticeConnector, D1ExactReferenceCancellationAndDeadlineNeverReusePath) {
  auto map=GridMapTestAccess::d1BoxCorridor();
  const Point a(-.0988,19.6841,-.1143),b(.0238,20.2757,-.1475);
  using Clock=std::chrono::steady_clock;
  const auto base=Clock::now();auto now=base;int calls=0;
  auto budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&]{return now;});
  AStar search([&]{++calls;return now;});search.initGridMapFromEnvironment(map);search.setSolveBudget(budget);
  ASSERT_EQ(search.AstarSearch(.05,a,b,false,true),ASTAR_RET::SUCCESS);
  budget->cancel();
  EXPECT_EQ(search.AstarSearch(.05,a,b,false,true),ASTAR_RET::SEARCH_ERR);
  EXPECT_EQ(search.lastSearchReason(),"solve_cancelled");EXPECT_TRUE(search.getPath().empty());
  const auto deadline=base+std::chrono::milliseconds(400);
  budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&]{return now;});
  search.setSolveBudget(budget);now=deadline;
  EXPECT_EQ(search.AstarSearch(.05,a,b,false,true),ASTAR_RET::SEARCH_ERR);
  EXPECT_TRUE(search.lastSearchTimedOut());EXPECT_TRUE(search.getPath().empty());
  EXPECT_EQ(budget->deadline(),deadline);
}

TEST(ReferenceLatticeConnector, D1SameCellVerticalReferenceIsNotInventedAsTraversable) {
  auto map=GridMapTestAccess::d1BoxCorridor(false);AStar search;search.initGridMapFromEnvironment(map);
  const Point a(.0238,20.2757,-.1475),b(.0238,20.2757,-.1375);
  EXPECT_EQ(search.AstarSearch(.05,a,b,false,true),ASTAR_RET::SEARCH_ERR);
  EXPECT_EQ(search.lastSearchReason(),"vertical_only_search_unsupported");
  EXPECT_TRUE(search.getPath().empty());
}

TEST(ReferenceLatticeConnector, RecordedShortSlopeSameXYCellChecksExactEdgeBothDirections) {
  for(const auto &[a,b]:recorded_short_slopes)for(bool reverse:{false,true}) {
    auto map=GridMapTestAccess::recordedStairWindow();
    Eigen::Vector3i ia,ib;map->posToIndex(a,ia);map->posToIndex(b,ib);
    ASSERT_EQ(ia.head<2>(),ib.head<2>());ASSERT_NE(ia.z(),ib.z());
    ASSERT_GT((a-b).head<2>().norm(),.03);
    const Point start=reverse?b:a,end=reverse?a:b;
    AStar search;search.initGridMapFromEnvironment(map);
    ASSERT_EQ(search.AstarSearch(.05,start,end,false,true),ASTAR_RET::SUCCESS)
        <<"reverse="<<reverse<<" reason="<<search.lastSearchReason();
    ASSERT_EQ(search.getPath().size(),2U);
    checkWholePath(*map,search.getPath(),start,end);
    EXPECT_EQ(search.lastSearchReason(),"success");
    // Legacy snapped search still cannot invent a vertical lattice edge.
    EXPECT_EQ(search.AstarSearch(.05,start,end,false),ASTAR_RET::SEARCH_ERR);
    EXPECT_EQ(search.lastSearchReason(),"vertical_only_search_unsupported");
    EXPECT_TRUE(search.getPath().empty());
  }
}

TEST(ReferenceLatticeConnector, RecordedShortSlopeMissingMiddleSupportCannotAliasIntoSuccess) {
  const auto &[a,b]=recorded_short_slopes.back();
  for(bool reverse:{false,true}) {
    auto map=GridMapTestAccess::recordedStairWindow();
    // The production attitude contract rejects an unavailable support patch.
    // Endpoints alone are valid, so only the real whole-edge query sees it.
    map->setQueryAttitudeProvider([&](const Point &p,double)->std::optional<Eigen::Vector2d> {
      const double fraction=(p.z()-a.z())/(b.z()-a.z());
      if(fraction>.2&&fraction<.8)return std::nullopt;
      return Eigen::Vector2d::Zero();
    });
    const Point start=reverse?b:a,end=reverse?a:b;
    const double yaw=std::atan2(end.y()-start.y(),end.x()-start.x());
    ASSERT_EQ(map->getPlanningInflateOccupancy(start,yaw),0);
    ASSERT_EQ(map->getPlanningInflateOccupancy(end,yaw),0);
    ASSERT_EQ(map->getPlanningInflateOccupancy((a+b)*.5,yaw),2);
    AStar search;search.initGridMapFromEnvironment(map);
    EXPECT_EQ(search.AstarSearch(.05,start,end,false,true),ASTAR_RET::INIT_UNOBSERVED);
    EXPECT_EQ(search.lastSearchReason(),"exact_reference_short_edge_blocked");
    EXPECT_TRUE(search.getPath().empty());
  }
}

TEST(ReferenceLatticeConnector, RecordedShortSlopeOccupiedEndpointsRemainOccupied) {
  const auto &[a,b]=recorded_short_slopes.back();
  for(bool reverse:{false,true}) {
    auto map=GridMapTestAccess::recordedStairWindow();map->setOccupied((a+b)*.5);
    const Point start=reverse?b:a,end=reverse?a:b;
    AStar search;search.initGridMapFromEnvironment(map);
    EXPECT_EQ(search.AstarSearch(.05,start,end,false,true),ASTAR_RET::INIT_START_OCCUPIED);
    EXPECT_TRUE(search.getPath().empty());
  }
}

TEST(ReferenceLatticeConnector, RecordedShortSlopeNativeMiddleBodyCollisionCannotAliasIntoSuccess) {
  const auto &[a,b]=recorded_short_slopes.back();
  const double forward_yaw=std::atan2(b.y()-a.y(),b.x()-a.x());
  const Point heading(std::cos(forward_yaw),std::sin(forward_yaw),0.);
  for(bool reverse:{false,true}) {
    auto map=GridMapTestAccess::recordedStairWindow();
    // A synthetic changing support attitude creates a swept-body collision
    // between free exact endpoints. This uses a real occupied GridMap voxel,
    // not an occupancy callback. Reverse travel keeps the same world tilt.
    map->setQueryAttitudeProvider([&](const Point &p,double yaw)->std::optional<Eigen::Vector2d> {
      const double fraction=std::clamp((p.z()-a.z())/(b.z()-a.z()),0.,1.);
      return Eigen::Vector2d(.65*4.*fraction*(1.-fraction)*std::cos(yaw-forward_yaw),0.);
    });
    map->setOccupied((a+b)*.5-.21*heading+Point(0.,0.,.40));
    const Point start=reverse?b:a,end=reverse?a:b;
    const double yaw=std::atan2(end.y()-start.y(),end.x()-start.x());
    ASSERT_EQ(map->getPlanningInflateOccupancy(start,yaw),0);
    ASSERT_EQ(map->getPlanningInflateOccupancy(end,yaw),0);
    ASSERT_EQ(map->getPlanningInflateOccupancy((a+b)*.5,yaw),1);
    AStar search;search.initGridMapFromEnvironment(map);
    EXPECT_EQ(search.AstarSearch(.05,start,end,false,true),ASTAR_RET::INIT_CONNECTOR_COLLISION);
    EXPECT_EQ(search.lastSearchReason(),"exact_reference_short_edge_blocked");
    EXPECT_TRUE(search.getPath().empty());
  }
}

TEST(ReferenceLatticeConnector, RecordedShortSlopeTrueVerticalStillUnsupportedBothDirections) {
  const Point a=recorded_short_slopes.back().first;
  const Point b(a.x(),a.y(),recorded_short_slopes.back().second.z());
  auto map=GridMapTestAccess::recordedStairWindow();AStar search;search.initGridMapFromEnvironment(map);
  for(bool reverse:{false,true}) {
    EXPECT_EQ(search.AstarSearch(.05,reverse?b:a,reverse?a:b,false,true),ASTAR_RET::SEARCH_ERR);
    EXPECT_EQ(search.lastSearchReason(),"vertical_only_search_unsupported");
    EXPECT_TRUE(search.getPath().empty());
  }
}

TEST(ReferenceLatticeConnector, RecordedShortSlopeSharesCancellationAndOriginalDeadline) {
  using Clock=std::chrono::steady_clock;
  const auto &[a,b]=recorded_short_slopes.back();
  for(bool cancel:{false,true}) {
    auto map=GridMapTestAccess::recordedStairWindow();const auto base=Clock::now();auto now=base;
    auto budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&]{return now;});
    map->setQueryAttitudeProvider([&](const Point &p,double)->std::optional<Eigen::Vector2d> {
      const double fraction=(p.z()-a.z())/(b.z()-a.z());
      if(fraction>.2&&fraction<.8) {
        if(cancel)budget->cancel();else now=base+std::chrono::milliseconds(401);
      }
      return Eigen::Vector2d::Zero();
    });
    AStar search([&]{return now;});search.initGridMapFromEnvironment(map);search.setSolveBudget(budget);
    EXPECT_EQ(search.AstarSearch(.05,a,b,false,true),ASTAR_RET::SEARCH_ERR);
    EXPECT_EQ(search.lastSearchReason(),cancel?"solve_cancelled":"search_deadline_exceeded");
    EXPECT_EQ(search.lastSearchTimedOut(),!cancel);EXPECT_TRUE(search.getPath().empty());
    EXPECT_EQ(budget->deadline(),base+std::chrono::milliseconds(400));
  }
}

TEST(ReferenceLatticeConnector, D1ZeroLengthReferenceCannotReturnAFalseCubicSeed) {
  auto map=GridMapTestAccess::d1BoxCorridor();AStar search;search.initGridMapFromEnvironment(map);
  const Point a(-.0988,19.6841,-.1143),b(.0238,20.2757,-.1475);
  ASSERT_EQ(search.AstarSearch(.05,a,b,false,true),ASTAR_RET::SUCCESS);
  for(const Point &end:{a,Point(a+Point(0.,5e-7,0.))}) {
    EXPECT_EQ(search.AstarSearch(.05,a,end,false,true),ASTAR_RET::INIT_ERR);
    EXPECT_EQ(search.lastSearchReason(),"zero_length_reference");
    EXPECT_TRUE(search.getPath().empty());
  }
}

TEST(ReferenceLatticeConnector, D1ExactDirectEdgeChecksCancellationAndOriginalBudgetWhileSampling) {
  using Clock=std::chrono::steady_clock;
  for(bool cancel:{false,true}) {
    auto map=GridMapTestAccess::d1BoxCorridor();const auto base=Clock::now();auto now=base;int calls=0;
    auto budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&]{return now;});
    AStar search([&] {
      if(++calls==8) {if(cancel)budget->cancel();else now=base+std::chrono::milliseconds(401);}
      return now;
    });
    search.initGridMapFromEnvironment(map);search.setSolveBudget(budget);
    EXPECT_EQ(search.AstarSearch(.05,{-.0988,19.6841,-.1143},{.0238,20.2757,-.1475},false,true),ASTAR_RET::SEARCH_ERR);
    EXPECT_GE(calls,8);EXPECT_TRUE(search.getPath().empty());
    EXPECT_EQ(search.lastSearchTimedOut(),!cancel);
    EXPECT_EQ(budget->deadline(),base+std::chrono::milliseconds(400));
  }
}

TEST(ReferenceLatticeConnector, NarrowEntranceDoesNotRequireDiagonalChordPose) {
  auto map=GridMapTestAccess::d1NarrowEntrance();
  const Point a(.025,17.025,-.125),b(1.025,20.025,-.125);
  const double chord=std::atan2(b.y()-a.y(),b.x()-a.x());
  ASSERT_EQ(map->getPlanningInflateOccupancy(a,std::acos(-1.)*.5),0);
  ASSERT_EQ(map->getPlanningInflateOccupancy(a,chord),1);
  for(bool reverse:{false,true}) {
    AStar search;search.initGridMapFromEnvironment(map);
    const Point start=reverse?b:a,end=reverse?a:b;
    auto budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400));
    search.setSolveBudget(budget);
    ASSERT_EQ(search.AstarSearch(.05,start,end,false,true,{!reverse,reverse}),ASTAR_RET::SUCCESS)
        <<"reverse="<<reverse<<" reason="<<search.lastSearchReason();
    checkWholePath(*map,search.getPath(),start,end);
  }
}

TEST(ReferenceLatticeConnector, NarrowEntranceTrueOccupiedAndUnknownEndpointsRemainRejected) {
  const Point a(.025,17.025,-.125),b(1.025,20.025,-.125);
  for(bool strict:{false,true})for(bool reverse:{false,true}) {
    auto map=GridMapTestAccess::d1NarrowEntrance();
    if(strict) {GridMapTestAccess::strict(*map);GridMapTestAccess::unknown(*map,a);}
    else map->setOccupied(a);
    AStar search;search.initGridMapFromEnvironment(map);
    const Point start=reverse?b:a,end=reverse?a:b;
    EXPECT_NE(search.AstarSearch(.05,start,end,false,true,{true,true}),ASTAR_RET::SUCCESS);
    EXPECT_TRUE(search.getPath().empty());
  }
}

TEST(ReferenceLatticeConnector, NarrowEntranceFrozenBodyHeadingIsNotOverridden) {
  auto map=GridMapTestAccess::d1NarrowEntrance();
  const Point a(.025,17.025,-.125),b(1.025,20.025,-.125);
  for(bool reverse:{false,true}) {
    AStar search;search.initGridMapFromEnvironment(map);
    const Point start=reverse?b:a,end=reverse?a:b;
    search.setFrozenBodyYaw(std::atan2(end.y()-start.y(),end.x()-start.x()));
    EXPECT_NE(search.AstarSearch(.05,start,end,false,true,{true,true}),ASTAR_RET::SUCCESS);
    EXPECT_TRUE(search.getPath().empty());
  }
}

TEST(ReferenceLatticeConnector, NarrowEntranceFailureNamesBlockedEndpointNotAdmissibleChord) {
  auto map=GridMapTestAccess::d1NarrowEntrance();
  const Point a(.025,17.025,-.125),b(1.025,20.025,-.125);
  map->setOccupied(b);AStar search;search.initGridMapFromEnvironment(map);
  EXPECT_EQ(search.AstarSearch(.05,a,b,false,true,{true,true}),ASTAR_RET::INIT_TARGET_OCCUPIED);
  EXPECT_TRUE(search.getPath().empty());
}

TEST(ReferenceLatticeConnector, NarrowEntranceKeepsOriginalEndpointHeightPlane) {
  auto map=GridMapTestAccess::d1NarrowEntrance();
  const Point a(.025,17.025,-.125),b(1.025,20.025,.275);
  for(bool reverse:{false,true}) {
    AStar search;search.initGridMapFromEnvironment(map);
    const Point start=reverse?b:a,end=reverse?a:b;
    search.setSolveBudget(std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400)));
    ASSERT_EQ(search.AstarSearch(.05,start,end,false,true,{!reverse,reverse}),ASTAR_RET::SUCCESS);
    const auto path=search.getPath();checkWholePath(*map,path,start,end);
    // This is native endpoint-plane interpolation, not terrain validation.
    // The admission fix cannot flatten stairs or move either reference end.
    for(const auto& p:path) {
      EXPECT_GE(p.z(),a.z()-.025-1e-12);EXPECT_LE(p.z(),b.z()+.025+1e-12);
    }
  }
}

TEST(ReferenceLatticeConnector, NarrowEntranceAdmissionSharesCancellationAndDeadline) {
  using Clock=std::chrono::steady_clock;
  for(bool cancel:{false,true}) {
    auto map=GridMapTestAccess::d1NarrowEntrance();const auto base=Clock::now();auto now=base;int calls=0;
    auto budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&]{return now;});
    AStar search([&] {
      if(++calls==8) {if(cancel)budget->cancel();else now=base+std::chrono::milliseconds(401);}
      return now;
    });
    search.initGridMapFromEnvironment(map);search.setSolveBudget(budget);
    EXPECT_EQ(search.AstarSearch(.05,{.025,17.025,-.125},{1.025,20.025,-.125},false,true,{true,false}),ASTAR_RET::SEARCH_ERR);
    EXPECT_GE(calls,8);EXPECT_TRUE(search.getPath().empty());
    EXPECT_EQ(search.lastSearchTimedOut(),!cancel);
    EXPECT_EQ(budget->deadline(),base+std::chrono::milliseconds(400));
  }
}

TEST(ReferenceLatticeConnector, NarrowEntranceIntermediateAnchorsKeepLegacyRecoveryOrder) {
  auto map=GridMapTestAccess::d1NarrowEntrance();
  const Point a(.025,17.025,-.125),b(1.025,20.025,-.125);
  for(bool reverse:{false,true}) {
    AStar search;search.initGridMapFromEnvironment(map);
    const Point start=reverse?b:a,end=reverse?a:b;
    const auto expected=reverse?ASTAR_RET::INIT_TARGET_OCCUPIED:ASTAR_RET::INIT_START_OCCUPIED;
    // Only the OTHER endpoint is immutable. This narrow intermediate anchor
    // must still be offered to its owner's retreat/advance recovery first.
    EXPECT_EQ(search.AstarSearch(.05,start,end,false,true,{reverse,!reverse}),expected);
    EXPECT_TRUE(search.getPath().empty());
    // All historical callers also retain the original selection policy.
    EXPECT_EQ(search.AstarSearch(.05,start,end,false,true),expected);
    EXPECT_TRUE(search.getPath().empty());
  }
}
