#include <gtest/gtest.h>
#include <path_searching/dyn_a_star.h>

// Exercise the actual AStar and GridMap without ROS init/nodes/transports.
struct GridMapTestAccess {
  static GridMap::Ptr make(const Eigen::Vector3i &blocked={999,999,999},int state=0,
                          bool add_unknown=false) {
    auto map=std::make_shared<GridMap>();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.08;p.resolution_inv_=12.5;p.map_voxel_num_={64,64,32};
    p.map_origin_idx_.setZero();map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;
    p.require_observed_free_=true;p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=.45;p.obstacles_inflation_z_down=.45;
    d.occupancy_buffer_.assign(64*64*32,-1.);
    if(state) d.occupancy_buffer_[map->toAddress(blocked)]=state==1 ? 2.:-1.01;
    if(add_unknown) d.occupancy_buffer_[map->toAddress({-7,1,6})]=-1.01;
    map->rebuildInflationOffsets();return map;
  }
};

static ASTAR_RET search(GridMap::Ptr map) {
  AStar astar;astar.initGridMap(map,{64,64,16});
  return astar.AstarSearch(.08,{-.6,0,.5},{.6,0,.5},false);
}

TEST(SearchEndpoint, FullyObservedSpaceFindsPath) {
  EXPECT_EQ(search(GridMapTestAccess::make()),ASTAR_RET::SUCCESS);
}
TEST(SearchEndpoint, StartUnknownIsExplicitButDoesNotAllowSearch) {
  EXPECT_EQ(search(GridMapTestAccess::make({-7,0,6},2)),ASTAR_RET::INIT_UNOBSERVED);
}
TEST(SearchEndpoint, OccupiedStartTakesPrecedenceOverNearbyUnknown) {
  EXPECT_EQ(search(GridMapTestAccess::make({-7,0,6},1,true)),ASTAR_RET::INIT_START_OCCUPIED);
}
TEST(SearchEndpoint, OccupiedTargetHasDistinctFailure) {
  EXPECT_EQ(search(GridMapTestAccess::make({7,0,6},1)),ASTAR_RET::INIT_TARGET_OCCUPIED);
}
