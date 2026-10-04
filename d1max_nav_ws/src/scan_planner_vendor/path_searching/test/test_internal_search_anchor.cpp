#include <gtest/gtest.h>
#include <path_searching/dyn_a_star.h>
#include <cmath>
#include <chrono>

// Actual native GridMap inflation and AStar; no ROS init/node/transports.
using Point=Eigen::Vector3d;
struct GridMapTestAccess {
  static GridMap::Ptr make() {
    auto map=std::make_shared<GridMap>();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={120,120,64};
    p.map_origin_idx_={40,0,0};map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.require_observed_free_=false;p.double_cylinder_radius_=.29;
    p.double_cylinder_offset_=.20000000000000007;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.45;
    const std::size_t size=120U*120U*64U;
    d.occupancy_buffer_.assign(size,-1.01);
    d.occupancy_buffer_inflate_.assign(size,0);d.occupancy_buffer_inflate_cnt_.assign(size,0);
    map->rebuildInflationOffsets();return map;
  }
  static void boxFootprint(GridMap &map) {
    // Actual occupied top cells of the 20 cm analytic box [1.5,2.2] x [-.4,.4].
    // Their XY inflation also supplies the tall-box corner counterexample.
    for(int x=30;x<=44;++x) for(int y=-8;y<=8;++y)
      map.setOccupied({(x+.5)*.05,(y+.5)*.05,.225});
  }
};

namespace {
void expectCollisionFree(GridMap &map,const std::vector<Point> &path) {
  ASSERT_GE(path.size(),2U);
  for(std::size_t i=1;i<path.size();++i) {
    const Point delta=path[i]-path[i-1];
    const double yaw=std::atan2(delta.y(),delta.x());
    const int count=std::max(1,static_cast<int>(std::ceil(delta.norm()/.005)));
    for(int k=0;k<=count;++k)
      EXPECT_EQ(map.getInflateOccupancy(path[i-1]+delta*(static_cast<double>(k)/count),yaw),0)
          <<"edge="<<i<<" sample="<<k;
  }
}

void expectRecordedTrapRecovered(const Point &in,const Point &out,const Point &old_start) {
  auto map=GridMapTestAccess::make();GridMapTestAccess::boxFootprint(*map);
  const double yaw=std::atan2(out.y()-in.y(),out.x()-in.x());
  ASSERT_EQ(map->getInflateOccupancy(old_start,yaw),0);
  // This is the observed failure: the first path-heading-free lattice anchor
  // has all eight destinations blocked under their actual edge headings.
  for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) {
    if(dx==0 && dy==0) continue;
    ASSERT_EQ(map->getInflateOccupancy(old_start+Point(dx*.05,dy*.05,0.),std::atan2(dy,dx)),1);
  }
  AStar planner;planner.initGridMap(map,{100,100,32});
  const auto before_forward=std::chrono::steady_clock::now();
  ASSERT_EQ(planner.AstarSearch(.05,in,out,true),ASTAR_RET::SUCCESS);
  ::testing::Test::RecordProperty("forward_search_ms",std::to_string(std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-before_forward).count()));
  const auto forward=planner.getPath();
  ASSERT_GE(forward.size(),2U);
  EXPECT_GT((forward.front()-old_start).norm(),.025);
  expectCollisionFree(*map,forward);
  // Reversal exercises the incoming-edge condition, not only start egress.
  const auto before_reverse=std::chrono::steady_clock::now();
  ASSERT_EQ(planner.AstarSearch(.05,out,in,true),ASTAR_RET::SUCCESS);
  ::testing::Test::RecordProperty("reverse_search_ms",std::to_string(std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-before_reverse).count()));
  expectCollisionFree(*map,planner.getPath());
}
} // namespace

TEST(InternalSearchAnchor, RecordedTallBoxCornerGetsReachableInternalAnchors) {
  expectRecordedTrapRecovered({1.532773,.690226,.55},{1.925345,.700877,.55},
                              {1.529059,.6955515,.55});
}

TEST(InternalSearchAnchor, RecordedLowBoxReboundGetsReachableInternalAnchors) {
  expectRecordedTrapRecovered({2.526163,-.348705,.55},{2.701947,0.,.55},
                              {2.464055,-.4243525,.55});
}

TEST(InternalSearchAnchor, ActualOccupiedExternalStartIsNeverAdjusted) {
  auto map=GridMapTestAccess::make();GridMapTestAccess::boxFootprint(*map);
  AStar planner;planner.initGridMap(map,{100,100,32});
  for(const auto &ends:std::vector<std::pair<Point,Point>>{
      {{1.532773,.690226,.55},{1.925345,.700877,.55}},
      {{2.526163,-.348705,.55},{2.701947,0.,.55}}}) {
    EXPECT_EQ(planner.AstarSearch(.05,ends.first,ends.second,false),ASTAR_RET::INIT_START_OCCUPIED);
    EXPECT_TRUE(planner.getPath().empty());
  }
}

TEST(InternalSearchAnchor, ReachableHelperAnchorsDoNotCrossASealedWall) {
  auto map=GridMapTestAccess::make();
  for(int y=-60;y<60;++y) map->setOccupied({2.025,(y+.5)*.05,.225});
  AStar planner;planner.initGridMap(map,{100,100,32});
  const auto before=std::chrono::steady_clock::now();
  EXPECT_EQ(planner.AstarSearch(.05,{1.,0.,.55},{3.,0.,.55},true),ASTAR_RET::SEARCH_ERR);
  RecordProperty("search_ms",std::to_string(std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-before).count()));
  EXPECT_TRUE(planner.getPath().empty());
}

TEST(InternalSearchAnchor, HeightVaryingSearchUsesTheSamePlaneForAnchorEdges) {
  auto map=GridMapTestAccess::make();
  AStar planner;planner.initGridMap(map,{100,100,32});
  ASSERT_EQ(planner.AstarSearch(.05,{1.,0.,.3},{3.,.2,.7},true),ASTAR_RET::SUCCESS);
  const auto path=planner.getPath();expectCollisionFree(*map,path);
  EXPECT_NEAR(path.front().z(),.3,.026);EXPECT_NEAR(path.back().z(),.7,.026);
}
