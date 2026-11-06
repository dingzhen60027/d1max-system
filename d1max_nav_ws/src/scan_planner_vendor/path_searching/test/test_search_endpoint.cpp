#include <gtest/gtest.h>
#include <path_searching/dyn_a_star.h>
#include <plan_env/collision_snapshot_pool.hpp>
#include <future>

// Exercise the actual AStar and GridMap without ROS init/nodes/transports.
struct GridMapTestAccess {
  static GridMap::Ptr make(const Eigen::Vector3i &blocked={999,999,999},int state=0,
                          bool add_unknown=false) {
    auto map=std::make_shared<GridMap>();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.08;p.resolution_inv_=12.5;p.map_voxel_num_={64,64,32};
    p.map_origin_idx_.setZero();map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.require_observed_free_=true;p.planning_collision_policy_="observed_free";
    p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=.45;p.obstacles_inflation_z_down=.45;
    d.occupancy_buffer_.assign(64*64*32,-1.);
    if(state) d.occupancy_buffer_[map->toAddress(blocked)]=state==1 ? 2.:-1.01;
    if(add_unknown) d.occupancy_buffer_[map->toAddress({-7,1,6})]=-1.01;
    map->rebuildInflationOffsets();return map;
  }
  static GridMap::Ptr officialUnknown(bool center_only=false) {
    auto map=make();auto &p=map->mp_;auto &d=map->md_;
    p.require_observed_free_=false;p.planning_collision_policy_="official";
    if (center_only) {
      p.double_cylinder_radius_=.01;p.double_cylinder_offset_=0.;
      p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=0.;
    }
    d.occupancy_buffer_.assign(64*64*32,-1.01);
    d.occupancy_buffer_inflate_.assign(64*64*32,0);
    d.occupancy_buffer_inflate_cnt_.assign(64*64*32,0);
    map->rebuildInflationOffsets();return map;
  }
  static void box(GridMap& map,const Eigen::Vector3d& obstacle) {
    auto& p=map.mp_;p.collision_model_="oriented_box";p.box_length_=.94;p.box_width_=.30;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    map.rebuildInflationOffsets();Eigen::Vector3i id;map.posToIndex(obstacle,id);
    map.md_.occupancy_buffer_[map.toAddress(id)]=2.;
  }
};

static ASTAR_RET search(GridMap::Ptr map,const Eigen::Vector3d &start={-.6,0,.5},
                        const Eigen::Vector3d &goal={.6,0,.5}) {
  AStar astar;astar.initGridMap(map,{64,64,16});
  return astar.AstarSearch(.08,start,goal,false);
}

TEST(SearchEndpoint, FullyObservedSpaceFindsPath) {
  EXPECT_EQ(search(GridMapTestAccess::make()),ASTAR_RET::SUCCESS);
}
TEST(SearchEndpoint,FrozenBodyHeadingCanTranslateSidewaysWithoutRotatingRectangle) {
  auto map=GridMapTestAccess::make();GridMapTestAccess::box(*map,{.025,.325,.5});
  AStar astar;astar.initGridMap(map,{64,64,16});
  const Eigen::Vector3d start(0.,0.,.5),goal(0.,-.8,.5);
  EXPECT_EQ(astar.AstarSearch(.08,start,goal,false),ASTAR_RET::INIT_START_OCCUPIED);
  astar.setFrozenBodyYaw(0.);
  ASSERT_EQ(astar.AstarSearch(.08,start,goal,false),ASTAR_RET::SUCCESS);
  const auto path=astar.getPath();ASSERT_GE(path.size(),2U);
  for(const auto& p:path)EXPECT_EQ(map->getPlanningInflateOccupancy(p,0.),0);
  astar.setFrozenBodyYaw(std::nullopt);
  EXPECT_EQ(astar.AstarSearch(.08,start,goal,false),ASTAR_RET::INIT_START_OCCUPIED);
  EXPECT_THROW(astar.setFrozenBodyYaw(std::numeric_limits<double>::quiet_NaN()),std::invalid_argument);
  astar.setFrozenBodyYaw(1.570796326794897);
  EXPECT_EQ(astar.AstarSearch(.08,start,goal,false),ASTAR_RET::INIT_START_OCCUPIED);
  // Holding yaw never ignores an object inside the held physical rectangle.
  GridMapTestAccess::box(*map,{.325,.025,.5});astar.setFrozenBodyYaw(0.);
  EXPECT_EQ(astar.AstarSearch(.08,start,goal,false),ASTAR_RET::INIT_START_OCCUPIED);
}
TEST(SearchEndpoint, LegacyStandaloneDeadlineIsResourceFailureAndSameMapCanRetry) {
  using Clock=std::chrono::steady_clock;
  const auto base=Clock::now();int calls=0;bool advance=true;
  AStar astar([&] {return base+std::chrono::milliseconds(advance?20*calls++:0);});
  astar.initGridMap(GridMapTestAccess::make(),{64,64,16});
  EXPECT_EQ(astar.AstarSearch(.08,{-.6,0,.5},{.6,0,.5},false),ASTAR_RET::SEARCH_ERR);
  EXPECT_TRUE(astar.lastSearchTimedOut());
  EXPECT_TRUE(astar.getPath().empty());
  advance=false;
  ASSERT_EQ(astar.AstarSearch(.08,{-.6,0,.5},{.6,0,.5},false),ASTAR_RET::SUCCESS);
  EXPECT_FALSE(astar.lastSearchTimedOut());
}
TEST(SearchEndpoint, FormalSearchMayUseOriginalBudgetAfterTwoHundredMilliseconds) {
  using Clock=std::chrono::steady_clock;
  const auto base=Clock::now();auto now=base;int calls=0;
  auto budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&]{return now;});
  AStar astar([&] {
    now=base+std::chrono::milliseconds(calls++==0?0:250);
    return now;
  });
  astar.initGridMap(GridMapTestAccess::make(),{64,64,16});astar.setSolveBudget(budget);
  ASSERT_EQ(astar.AstarSearch(.08,{-.6,0,.5},{.6,0,.5},false),ASTAR_RET::SUCCESS);
  EXPECT_FALSE(astar.lastSearchTimedOut());EXPECT_TRUE(budget->allowed());
  EXPECT_GT(now-base,std::chrono::milliseconds(200));
  EXPECT_LT(now,budget->deadline());
  EXPECT_EQ(budget->deadline(),base+std::chrono::milliseconds(400));
}
TEST(SearchEndpoint, ParentTimeConsumedBeforeSearchCannotBeRenewed) {
  using Clock=std::chrono::steady_clock;
  const auto base=Clock::now();auto now=base;
  auto parent=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&]{return now;});
  now+=std::chrono::milliseconds(350);
  auto budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&]{return now;},parent);
  EXPECT_EQ(budget->deadline(),base+std::chrono::milliseconds(400));
  int calls=0;AStar astar([&] {
    now=base+std::chrono::milliseconds(calls++==0?350:401);
    return now;
  });
  astar.initGridMap(GridMapTestAccess::make(),{64,64,16});astar.setSolveBudget(budget);
  EXPECT_EQ(astar.AstarSearch(.08,{-.6,0,.5},{.6,0,.5},false),ASTAR_RET::SEARCH_ERR);
  EXPECT_TRUE(astar.lastSearchTimedOut());EXPECT_TRUE(astar.getPath().empty());
  EXPECT_EQ(parent->stop(),scan_planner::SolveBudget::Stop::Deadline);
  EXPECT_EQ(budget->deadline(),base+std::chrono::milliseconds(400));
}
TEST(SearchEndpoint, CancellationNeverBorrowsInternalTimeoutRecovery) {
  using Clock=std::chrono::steady_clock;
  const auto base=Clock::now();
  AStar astar([&] {return base+std::chrono::seconds(1);});
  astar.initGridMap(GridMapTestAccess::make(),{64,64,16});
  auto budget=std::make_shared<scan_planner::SolveBudget>();budget->cancel();astar.setSolveBudget(budget);
  EXPECT_EQ(astar.AstarSearch(.08,{-.6,0,.5},{.6,0,.5},false),ASTAR_RET::SEARCH_ERR);
  EXPECT_FALSE(astar.lastSearchTimedOut());
  EXPECT_TRUE(astar.getPath().empty());
}
TEST(SearchEndpoint, CancelledSharedBudgetDoesNotRunOrReturnOldPath) {
  AStar astar;astar.initGridMap(GridMapTestAccess::make(),{64,64,16});
  ASSERT_EQ(astar.AstarSearch(.08,{-.6,0,.5},{.6,0,.5},false),ASTAR_RET::SUCCESS);
  auto budget=std::make_shared<scan_planner::SolveBudget>();budget->cancel();
  astar.setSolveBudget(budget);
  EXPECT_EQ(astar.AstarSearch(.08,{-.6,0,.5},{.6,0,.5},false),ASTAR_RET::SEARCH_ERR);
  EXPECT_TRUE(astar.getPath().empty());
}
TEST(SearchEndpoint, ExpiredSharedBudgetIsNotRenewedByNewAStarCall) {
  AStar astar;astar.initGridMap(GridMapTestAccess::make(),{64,64,16});
  auto now=scan_planner::SolveBudget::Clock::now();
  auto budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&] {return now;});
  now+=std::chrono::milliseconds(400);astar.setSolveBudget(budget);
  EXPECT_EQ(astar.AstarSearch(.08,{-.6,0,.5},{.6,0,.5},false),ASTAR_RET::SEARCH_ERR);
  EXPECT_TRUE(astar.lastSearchTimedOut());EXPECT_EQ(astar.lastSearchReason(),"search_deadline_exceeded");
  EXPECT_TRUE(astar.getPath().empty());
}
TEST(SearchEndpoint, AsyncRealAStarUsesPinnedNativeSnapshotNotMutatingWriter) {
  auto writer=GridMapTestAccess::officialUnknown(true);
  scan_planner::CollisionSnapshotPool pool;
  ASSERT_TRUE(pool.publish(*writer,1000000000,std::chrono::steady_clock::now()));
  auto lease=pool.borrowLatest();ASSERT_TRUE(lease);
  std::promise<void> release;
  auto go=release.get_future();
  auto solve=std::async(std::launch::async,[lease,&go] {
    go.wait();
    AStar actual;actual.initGridMap(lease,{64,64,16});
    auto budget=std::make_shared<scan_planner::SolveBudget>();actual.setSolveBudget(budget);
    return actual.AstarSearch(.08,{-.6,0,.5},{.6,0,.5},false);
  });
  writer->setOccupied({-.6,0.,.5});
  EXPECT_EQ(search(writer),ASTAR_RET::INIT_START_OCCUPIED);
  release.set_value();
  EXPECT_EQ(solve.get(),ASTAR_RET::SUCCESS);
  // New map recheck still rejects the old successful candidate start. A safe
  // snapshot solve alone is not current-world adoption or motion authority.
  EXPECT_EQ(writer->getInflateOccupancy({-.6,0.,.5},0.),1);
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
TEST(SearchEndpoint, OfficialUnknownRawEvidenceDoesNotRejectClearInflation) {
  const auto map=GridMapTestAccess::officialUnknown();
  ASSERT_EQ(map->getInflateOccupancy({-.6,0.,.5},0.),0);
  ASSERT_EQ(map->inspectInflateOccupancy({-.6,0.,.5},0.).state(),2);
  EXPECT_EQ(search(map),ASTAR_RET::SUCCESS);
  EXPECT_TRUE(map->isUnknown(Eigen::Vector3d(-.6,0.,.5)));
}
TEST(SearchEndpoint, OfficialOccupiedStartRemainsOccupiedWithUnknownRawNeighbors) {
  const auto map=GridMapTestAccess::officialUnknown();
  map->setOccupied({-.6,0.,.5});
  ASSERT_EQ(map->getInflateOccupancy({-.6,0.,.5},0.),1);
  EXPECT_EQ(search(map),ASTAR_RET::INIT_START_OCCUPIED);
}
TEST(SearchEndpoint, OfficialOccupiedTargetDoesNotRelabelUnknownStartAsBlocked) {
  const auto map=GridMapTestAccess::officialUnknown();
  map->setOccupied({.6,0.,.5});
  ASSERT_EQ(map->getInflateOccupancy({-.6,0.,.5},0.),0);
  ASSERT_EQ(map->inspectInflateOccupancy({-.6,0.,.5},0.).state(),2);
  ASSERT_EQ(map->getInflateOccupancy({.6,0.,.5},0.),1);
  EXPECT_EQ(search(map),ASTAR_RET::INIT_TARGET_OCCUPIED);
}
TEST(SearchEndpoint, OfficialContinuousMapBoundaryDoesNotBecomeRawUnknownFailure) {
  const auto map=GridMapTestAccess::officialUnknown(true);
  // Upstream's continuous map margin rejects this center, although its raw
  // voxel index is still in-map and unknown. Raw diagnostics cannot replace
  // the actual inflated-buffer query's OUTSIDE result.
  const Eigen::Vector3d start(-2.56+5e-5,0.,.5),goal(-1.28+5e-5,0.,.5);
  ASSERT_EQ(map->getInflateOccupancy(start,0.),-1);
  ASSERT_EQ(map->inspectInflateOccupancy(start,0.).state(),2);
  EXPECT_EQ(search(map,start,goal),ASTAR_RET::INIT_OUTSIDE_MAP);
}
TEST(SearchEndpoint, OfficialOutsideQueryDoesNotBecomeRawOccupiedFailure) {
  const auto map=GridMapTestAccess::officialUnknown(true);
  const Eigen::Vector3d start(-2.56+5e-5,0.,.5),goal(-1.28+5e-5,0.,.5);
  // A legitimate hit in the same cell cannot make an out-of-map query into an
  // in-map occupied endpoint; preserve upstream center-boundary semantics.
  map->setOccupied({-2.52,.04,.52});
  ASSERT_EQ(map->getInflateOccupancy(start,0.),-1);
  ASSERT_EQ(map->inspectInflateOccupancy(start,0.).state(),1);
  EXPECT_EQ(search(map,start,goal),ASTAR_RET::INIT_OUTSIDE_MAP);
}
