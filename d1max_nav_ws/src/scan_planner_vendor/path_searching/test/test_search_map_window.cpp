#include <gtest/gtest.h>
#include <path_searching/dyn_a_star.h>
#include <chrono>

using Point=Eigen::Vector3d;
// The production GridMap / production AStar, no substitute occupancy oracle,
// ROS nodes, transports, SDK or motion. Small-cylinder cases isolate search
// coverage; the actual D1 OBB case independently verifies collision rejection.
struct GridMapTestAccess {
  static GridMap::Ptr make(int xy=240,int z=60) {
    auto map=std::make_shared<GridMap>();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={xy,xy,z};
    p.map_size_=p.map_voxel_num_.cast<double>()*.05;
    p.map_origin_idx_.setZero();map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.require_observed_free_=false;p.planning_collision_policy_="official";
    p.double_cylinder_radius_=.01;p.double_cylinder_offset_=0.;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=0.;
    const auto count=static_cast<std::size_t>(xy)*xy*z;
    d.occupancy_buffer_.assign(count,-1.);
    d.occupancy_buffer_inflate_.assign(count,0);d.occupancy_buffer_inflate_cnt_.assign(count,0);
    map->rebuildInflationOffsets();return map;
  }
  static void wall(GridMap &map) {
    for(int y=-61;y<=61;++y) map.setOccupied({2.025,(y+.5)*.05,.525});
  }
  static void moveOrigin(GridMap &map,const Eigen::Vector3i &origin) {
    map.mp_.map_origin_idx_=origin;map.updateMapBoundaryFromIndex();
  }
  static void actualBox(GridMap &map) {
    auto &p=map.mp_;p.collision_model_="oriented_box";
    p.box_length_=.93;p.box_width_=.30;
    p.body_below_=p.body_above_=.25;
    p.planning_body_below_=p.planning_body_above_=.25;
    map.rebuildInflationOffsets();
  }
};

namespace {
void expectFree(GridMap &map,const std::vector<Point> &path) {
  ASSERT_GE(path.size(),2U);
  for(std::size_t i=1;i<path.size();++i) {
    const auto delta=path[i]-path[i-1];const double yaw=std::atan2(delta.y(),delta.x());
    const int samples=std::max(1,static_cast<int>(std::ceil(delta.norm()/.01)));
    for(int k=0;k<=samples;++k)
      EXPECT_EQ(map.getPlanningInflateOccupancy(path[i-1]+delta*(double(k)/samples),yaw),0);
  }
}
}

TEST(SearchMapWindow, SameInputOldMidpointBoxCannotReachRealMapSideCorridor) {
  auto map=GridMapTestAccess::make();GridMapTestAccess::wall(*map);
  const Point start(.025,.025,.525),goal(4.025,.025,.525);
  AStar before;before.initGridMap(map,{100,100,100});
  const auto t0=std::chrono::steady_clock::now();
  EXPECT_EQ(before.AstarSearch(.05,start,goal,false),ASTAR_RET::SEARCH_ERR);
  const double before_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();
  EXPECT_TRUE(before.getPath().empty());
  AStar after;after.initGridMapFromEnvironment(map);
  const auto t1=std::chrono::steady_clock::now();
  ASSERT_EQ(after.AstarSearch(.05,start,goal,false),ASTAR_RET::SUCCESS);
  const double after_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t1).count();
  const auto path=after.getPath();expectFree(*map,path);
  double detour_y=0.;for(const auto &p:path)detour_y=std::max(detour_y,std::abs(p.y()));
  EXPECT_GT(detour_y,3.);EXPECT_LT(detour_y,6.);
  EXPECT_EQ(after.latticeSize(),Eigen::Vector3i(242,242,62));
  EXPECT_EQ(after.nodeCapacity(),242U*242U);
  EXPECT_LT(after.storageBytes(),5U*1024U*1024U);
  EXPECT_LE(after.peakOpenNodes(),after.nodeCapacity());
  ::testing::Test::RecordProperty("before_status","SEARCH_ERR");
  ::testing::Test::RecordProperty("after_status","SUCCESS");
  ::testing::Test::RecordProperty("before_ms",std::to_string(before_ms));
  ::testing::Test::RecordProperty("after_ms",std::to_string(after_ms));
  ::testing::Test::RecordProperty("after_storage_bytes",std::to_string(after.storageBytes()));
  ::testing::Test::RecordProperty("after_peak_open_nodes",std::to_string(after.peakOpenNodes()));
  ::testing::Test::RecordProperty("after_expanded_nodes",std::to_string(after.expandedNodes()));
}

TEST(SearchMapWindow, HeightVaryingPlaneKeepsXYZAndDoesNotClaimVerticalOnlySupport) {
  auto map=GridMapTestAccess::make();AStar search;search.initGridMapFromEnvironment(map);
  ASSERT_EQ(search.AstarSearch(.05,{-.975,.025,.275},{3.025,.525,.875},false),ASTAR_RET::SUCCESS);
  const auto path=search.getPath();expectFree(*map,path);
  EXPECT_NEAR(path.front().z(),.275,1e-9);EXPECT_NEAR(path.back().z(),.875,1e-9);
  double low=path.front().z(),high=low;for(const auto &p:path){low=std::min(low,p.z());high=std::max(high,p.z());}
  EXPECT_GT(high-low,.55);
  EXPECT_EQ(search.AstarSearch(.05,{.025,.025,.275},{.025,.025,.875},false),ASTAR_RET::SEARCH_ERR);
  EXPECT_EQ(search.lastSearchReason(),"vertical_only_search_unsupported");
  EXPECT_TRUE(search.getPath().empty());
}

TEST(SearchMapWindow, D1ProductionOBBRangeAblationSeparatesGeometryFromRealDeadline) {
  auto map=GridMapTestAccess::make();GridMapTestAccess::actualBox(*map);GridMapTestAccess::wall(*map);
  const Point start(.025,.025,.525),goal(4.025,.025,.525);
  // A fixed planning clock isolates reachable geometry, NOT runtime acceptance.
  // The whole call additionally has a real 5 s diagnostic ceiling. The separate
  // real-clock search below retains the ORIGINAL shared 400 ms contract.
  using Clock=std::chrono::steady_clock;
  const auto fixed=Clock::now();auto real_start=Clock::now();
  const auto diagnostic_clock=[&] {
    return Clock::now()-real_start<std::chrono::seconds(5)?fixed:fixed+std::chrono::seconds(1);
  };
  AStar old_domain(diagnostic_clock);old_domain.initGridMap(map,{100,100,100});
  real_start=Clock::now();
  EXPECT_EQ(old_domain.AstarSearch(.05,start,goal,false),ASTAR_RET::SEARCH_ERR);
  const double old_ms=std::chrono::duration<double,std::milli>(Clock::now()-real_start).count();
  AStar real_domain(diagnostic_clock);real_domain.initGridMapFromEnvironment(map);
  real_start=Clock::now();const auto geometry_result=real_domain.AstarSearch(.05,start,goal,false);
  const double geometry_ms=std::chrono::duration<double,std::milli>(Clock::now()-real_start).count();
  // A resource-limited result is reported honestly rather than disabling OBB
  // queries or extending a production solver's shared deadline to pass a test.
  if(geometry_result==ASTAR_RET::SUCCESS) expectFree(*map,real_domain.getPath());
  else EXPECT_TRUE(real_domain.lastSearchTimedOut());
  // Historical standalone calls still have 200 ms. Record their real-clock
  // result separately; they do not represent the formal production contract.
  AStar standalone;standalone.initGridMapFromEnvironment(map);
  real_start=Clock::now();const auto standalone_result=standalone.AstarSearch(.05,start,goal,false);
  const double standalone_ms=std::chrono::duration<double,std::milli>(Clock::now()-real_start).count();
  if(standalone_result==ASTAR_RET::SUCCESS) expectFree(*map,standalone.getPath());
  else EXPECT_TRUE(standalone.lastSearchTimedOut());
  AStar production;production.initGridMapFromEnvironment(map);
  auto budget=std::make_shared<scan_planner::SolveBudget>();production.setSolveBudget(budget);
  real_start=Clock::now();const auto production_result=production.AstarSearch(.05,start,goal,false);
  const double production_ms=std::chrono::duration<double,std::milli>(Clock::now()-real_start).count();
  if(production_result==ASTAR_RET::SUCCESS) expectFree(*map,production.getPath());
  else EXPECT_TRUE(production.lastSearchTimedOut());
  EXPECT_LT(production_ms,450.);EXPECT_LE(production.peakOpenNodes(),production.nodeCapacity());
  ::testing::Test::RecordProperty("obb_model","oriented_box_0.93x0.30x0.50");
  ::testing::Test::RecordProperty("obb_old_domain_status","SEARCH_ERR");
  ::testing::Test::RecordProperty("obb_old_domain_ms",std::to_string(old_ms));
  ::testing::Test::RecordProperty("obb_geometry_status",geometry_result==ASTAR_RET::SUCCESS?"SUCCESS":"DIAGNOSTIC_TIMEOUT");
  ::testing::Test::RecordProperty("obb_geometry_real_ms",std::to_string(geometry_ms));
  ::testing::Test::RecordProperty("obb_geometry_expanded_nodes",std::to_string(real_domain.expandedNodes()));
  ::testing::Test::RecordProperty("obb_geometry_peak_open_nodes",std::to_string(real_domain.peakOpenNodes()));
  ::testing::Test::RecordProperty("obb_geometry_queries",std::to_string(real_domain.planningQueryCount()));
  ::testing::Test::RecordProperty("obb_geometry_deadline_contract","diagnostic_fixed_clock_5s_watchdog_not_production");
  ::testing::Test::RecordProperty("obb_legacy_standalone_status",standalone_result==ASTAR_RET::SUCCESS?"SUCCESS":"SEARCH_TIMEOUT");
  ::testing::Test::RecordProperty("obb_legacy_standalone_real_ms",std::to_string(standalone_ms));
  ::testing::Test::RecordProperty("obb_legacy_standalone_queries",std::to_string(standalone.planningQueryCount()));
  ::testing::Test::RecordProperty("obb_legacy_standalone_deadline_contract","legacy_standalone_200ms");
  ::testing::Test::RecordProperty("obb_production_status",production_result==ASTAR_RET::SUCCESS?"SUCCESS":"SEARCH_TIMEOUT");
  ::testing::Test::RecordProperty("obb_production_real_ms",std::to_string(production_ms));
  ::testing::Test::RecordProperty("obb_production_expanded_nodes",std::to_string(production.expandedNodes()));
  ::testing::Test::RecordProperty("obb_production_peak_open_nodes",std::to_string(production.peakOpenNodes()));
  ::testing::Test::RecordProperty("obb_production_queries",std::to_string(production.planningQueryCount()));
  ::testing::Test::RecordProperty("obb_production_deadline_contract","original_shared_400ms_not_search_renewal");
  ::testing::Test::RecordProperty("obb_scope","search_only_not_end_to_end_solve_or_motion_acceptance");
}

TEST(SearchMapWindow, ActualD1BoxAndMapBoundaryRemainRejected) {
  auto map=GridMapTestAccess::make();GridMapTestAccess::actualBox(*map);
  map->setOccupied({.325,.075,.575});
  AStar search;search.initGridMapFromEnvironment(map);
  EXPECT_EQ(search.AstarSearch(.05,{.025,.025,.525},{1.025,.025,.525},false),ASTAR_RET::INIT_START_OCCUPIED);
  EXPECT_TRUE(search.getPath().empty());
  EXPECT_EQ(search.AstarSearch(.05,{6.1,.025,.525},{4.025,.025,.525},false),ASTAR_RET::INIT_OUTSIDE_MAP);
  EXPECT_EQ(search.AstarSearch(.05,{2.025,.025,.525},{4.025,.025,1.6},false),ASTAR_RET::INIT_OUTSIDE_MAP);
  // The configurable search window never makes a partly out-of-map body safe.
  EXPECT_NE(map->getPlanningInflateOccupancy({5.9,.025,.525},0.),0);
}

TEST(SearchMapWindow, RollingSnapshotChangesAnchorWithoutAllocatingAnotherPool) {
  auto map=GridMapTestAccess::make();AStar search;search.initGridMapFromEnvironment(map);
  const auto bytes=search.storageBytes();
  ASSERT_EQ(search.AstarSearch(.05,{.025,.025,.525},{4.025,.025,.525},false),ASTAR_RET::SUCCESS);
  GridMapTestAccess::moveOrigin(*map,{200,-100,0});
  ASSERT_EQ(search.AstarSearch(.05,{10.025,-4.975,.525},{14.025,-4.975,.525},false),ASTAR_RET::SUCCESS);
  expectFree(*map,search.getPath());EXPECT_EQ(search.storageBytes(),bytes);
  EXPECT_EQ(search.AstarSearch(.05,{.025,.025,.525},{4.025,.025,.525},false),ASTAR_RET::INIT_OUTSIDE_MAP);
  EXPECT_TRUE(search.getPath().empty());
}

TEST(SearchMapWindow, InternalAnchorShiftSharesRealMapAndDoesNotCrossItsBoundary) {
  auto map=GridMapTestAccess::make();
  GridMapTestAccess::actualBox(*map);
  map->setOccupied({.025,.025,.525});AStar search;search.initGridMapFromEnvironment(map);
  ASSERT_EQ(search.AstarSearch(.05,{.025,.025,.525},{1.025,.025,.525},true),ASTAR_RET::SUCCESS);
  const auto path=search.getPath();expectFree(*map,path);
  EXPECT_LT(path.front().x(),.025);
  EXPECT_EQ(search.AstarSearch(.05,{6.1,.025,.525},{4.025,.025,.525},true),ASTAR_RET::INIT_ERR);
  EXPECT_TRUE(search.getPath().empty());
}

TEST(SearchMapWindow, ResourceLimitRefusesFullConfiguredWindowRatherThanTruncatingIt) {
  auto map=GridMapTestAccess::make(512,8);AStar search;search.initGridMapFromEnvironment(map);
  EXPECT_EQ(search.latticeSize(),Eigen::Vector3i(514,514,10));
  EXPECT_EQ(search.nodeCapacity(),0U);EXPECT_EQ(search.storageBytes(),0U);
  EXPECT_EQ(search.AstarSearch(.05,{.025,.025,.125},{1.025,.025,.125},false),ASTAR_RET::SEARCH_ERR);
  EXPECT_TRUE(search.lastSearchResourceLimited());EXPECT_FALSE(search.lastSearchTimedOut());
  EXPECT_EQ(search.lastSearchReason(),"plane_node_resource_limit");EXPECT_TRUE(search.getPath().empty());
}

TEST(SearchMapWindow, CancelledAndExpiredBudgetNeverRenewOrReturnPreviousPath) {
  auto map=GridMapTestAccess::make();AStar search;search.initGridMapFromEnvironment(map);
  ASSERT_EQ(search.AstarSearch(.05,{.025,.025,.525},{1.025,.025,.525},false),ASTAR_RET::SUCCESS);
  auto budget=std::make_shared<scan_planner::SolveBudget>();budget->cancel();search.setSolveBudget(budget);
  EXPECT_EQ(search.AstarSearch(.05,{.025,.025,.525},{1.025,.025,.525},false),ASTAR_RET::SEARCH_ERR);
  EXPECT_EQ(search.lastSearchReason(),"solve_cancelled");EXPECT_FALSE(search.lastSearchTimedOut());
  EXPECT_TRUE(search.getPath().empty());
  auto now=scan_planner::SolveBudget::Clock::now();
  budget=std::make_shared<scan_planner::SolveBudget>(std::chrono::milliseconds(400),[&]{return now;});
  now+=std::chrono::milliseconds(401);search.setSolveBudget(budget);
  EXPECT_EQ(search.AstarSearch(.05,{.025,.025,.525},{1.025,.025,.525},false),ASTAR_RET::SEARCH_ERR);
  EXPECT_TRUE(search.getPath().empty());EXPECT_EQ(budget->stop(),scan_planner::SolveBudget::Stop::Deadline);
}

TEST(SearchOpenHeap, DecreaseKeyIsBoundedAndRestoresOrdering) {
  SearchOpenHeap heap;heap.reserve(3);GridNode a,b,c,too_many;
  a.fScore=3.;b.fScore=5.;c.fScore=4.;too_many.fScore=0.;
  ASSERT_TRUE(heap.pushOrDecrease(&a));ASSERT_TRUE(heap.pushOrDecrease(&b));ASSERT_TRUE(heap.pushOrDecrease(&c));
  EXPECT_FALSE(heap.pushOrDecrease(&too_many));EXPECT_EQ(heap.size(),3U);
  b.fScore=1.;ASSERT_TRUE(heap.pushOrDecrease(&b));EXPECT_EQ(heap.size(),3U);
  EXPECT_EQ(heap.pop(),&b);EXPECT_EQ(heap.pop(),&a);EXPECT_EQ(heap.pop(),&c);EXPECT_TRUE(heap.empty());
  EXPECT_EQ(b.heap_index,std::numeric_limits<std::size_t>::max());
  ASSERT_TRUE(heap.pushOrDecrease(&b));heap.clear();EXPECT_EQ(b.heap_index,std::numeric_limits<std::size_t>::max());
}

TEST(SearchEdgeSampling, ExactEndpointsAvoidDuplicatePoseCacheMissWithoutRemovingSamples) {
  const Point from(-.2,.025,.525),to(.1,.025,.525);
  const Point old_endpoint=from+(to-from)*1.;
  ASSERT_NE(old_endpoint.x(),to.x());
  scan_planner::VoxelStatusCache before,after;
  int old_misses=0,new_misses=0;
  const auto old_query=[&](const Point &p) {
    return before.getPoseExact(1,5,16,p.x(),p.y(),0.,[&]{++old_misses;return 0;});
  };
  const auto new_query=[&](const Point &p) {
    return after.getPoseExact(1,5,16,p.x(),p.y(),0.,[&]{++new_misses;return 0;});
  };
  EXPECT_EQ(old_query(to),0);EXPECT_EQ(old_query(old_endpoint),0);
  EXPECT_EQ(new_query(to),0);EXPECT_EQ(new_query(scan_planner::searchEdgeSample(from,to,12,12)),0);
  EXPECT_EQ(old_misses,2);EXPECT_EQ(new_misses,1);
  EXPECT_EQ(scan_planner::searchEdgeSample(from,to,0,12),from);
  for(int i=1;i<12;++i)
    EXPECT_EQ(scan_planner::searchEdgeSample(from,to,i,12),from+(to-from)*(double(i)/12));
  EXPECT_THROW(scan_planner::searchEdgeSample(from,to,1,0),std::invalid_argument);
}
