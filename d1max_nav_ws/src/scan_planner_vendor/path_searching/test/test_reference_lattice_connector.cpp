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
    p.require_observed_free_=strict;p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
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
};

namespace {
struct RecordedEndpoints {Point start,end,obstacle;};
const std::vector<RecordedEndpoints> recorded{
  // scan.log replan 65 / 1790459832.819903615; true body was (.468,1.3,.0288).
  {{.5895,1.9331,.0343},{.5385,3.6349,.0251},{.875,2.125,.025}},
  // scan.log replan 68 / 1790459836.552673583; another reference subsegment.
  {{.5932,1.7957,.0351},{.5389,3.6195,.0253},{.875,2.025,.025}}
};

Point snap(const Point &point,const Point &a,const Point &b) {
  const Point centre=(a+b)*.5;
  Eigen::Vector3i index;
  EXPECT_TRUE(scan_planner::nearestSearchIndex(point,centre,.05,{50,50,16},{100,100,32},index));
  return (index-Eigen::Vector3i(50,50,16)).cast<double>()*.05+centre;
}

void checkWholePath(GridMap &map,const std::vector<Point> &path,const Point &start,const Point &end) {
  ASSERT_GE(path.size(),3U);
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
  const auto path=planner.getPath();ASSERT_GE(path.size(),3U);
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
  const auto path=planner.getPath();ASSERT_GE(path.size(),3U);
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
