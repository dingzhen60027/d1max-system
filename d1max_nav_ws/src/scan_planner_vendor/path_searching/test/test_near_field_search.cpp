// Analytic fixtures exercise the ACTUAL AStar and GridMap; no ROS init/node,
// graph, live goal, SDK or motion output. Fully observed free cells here are
// explicit synthetic test input, not fabricated observations for the real bag.
// This is geometric SEARCH validation, not B-spline dynamics, support or
// trajectory execution/admission validation.
#include <gtest/gtest.h>
#include <path_searching/dyn_a_star.h>
#include <chrono>
#include <cmath>

using Point=Eigen::Vector3d;

struct GridMapTestAccess {
  static GridMap::Ptr observedFree() {
    auto map=std::make_shared<GridMap>();auto &p=map->mp_;auto &d=map->md_;
    p.resolution_=.08;p.resolution_inv_=12.5;p.map_voxel_num_={64,64,32};
    p.map_origin_idx_.setZero();map->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.require_observed_free_=true;p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=.45;p.obstacles_inflation_z_down=.45;
    const std::size_t size=64*64*32;
    d.occupancy_buffer_.assign(size,-1.);
    d.occupancy_buffer_inflate_.assign(size,0);
    d.occupancy_buffer_inflate_cnt_.assign(size,0);
    map->rebuildInflationOffsets();return map;
  }

  static void box(GridMap &map,const Point &low,const Point &high,int state) {
    Eigen::Vector3i first,last;map.posToIndex(low,first);map.posToIndex(high,last);
    for(int x=first.x();x<=last.x();++x) for(int y=first.y();y<=last.y();++y)
      for(int z=first.z();z<=last.z();++z) {
        const Eigen::Vector3i cell(x,y,z);
        if(map.isInMap(cell)) map.md_.occupancy_buffer_[map.toAddress(cell)]=state==1?2.:-1.01;
      }
    map.observed_cylinder_cache_.clear();++map.occupancy_revision_;
  }
};

namespace {
struct Result {ASTAR_RET status;std::vector<Point> path;double milliseconds;};
Result search(const GridMap::Ptr &map,const Point &start={-1.2,0.,.55},
              const Point &goal={1.2,0.,.55}) {
  AStar planner;planner.initGridMap(map,{64,64,16});
  const auto begin=std::chrono::steady_clock::now();
  const auto status=planner.AstarSearch(.08,start,goal,false);
  Result result{status,planner.getPath(),std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-begin).count()};
  return result;
}

// Independent dense resampling calls the production collision query itself;
// no handwritten proxy geometry accepts the output. This uses segment yaw and
// deliberately does not pretend to certify instantaneous heading transitions.
void expectNativeCollisionFree(GridMap &map,const std::vector<Point> &path) {
  ASSERT_GE(path.size(),2U);
  std::size_t samples=0;
  for(std::size_t i=1;i<path.size();++i) {
    const auto delta=path[i]-path[i-1];
    const double yaw=std::atan2(delta.y(),delta.x());
    const auto count=std::max(1,int(std::ceil(delta.norm()/.01)));
    for(int k=0;k<=count;++k) {
      const Point p=path[i-1]+delta*(double(k)/count);
      EXPECT_EQ(map.getInflateOccupancy(p,yaw),0)
          <<"segment="<<i<<" fraction="<<double(k)/count<<" position="<<p.transpose();
      ++samples;
    }
  }
  EXPECT_GT(samples,path.size());
  testing::Test::RecordProperty("dense_native_collision_samples",samples);
}

double maxLateralDeviation(const std::vector<Point> &path) {
  double maximum=0.;for(const auto &p:path) maximum=std::max(maximum,std::abs(p.y()));
  return maximum;
}
} // namespace

TEST(NearFieldNativeSearch, FullyObservedShortStraightRoutePassesNativeResampling) {
  auto map=GridMapTestAccess::observedFree();
  const auto result=search(map);
  ASSERT_EQ(result.status,ASTAR_RET::SUCCESS);
  ASSERT_GE(result.path.size(),2U);
  EXPECT_NEAR(result.path.front().x(),-1.2,.041);
  EXPECT_NEAR(result.path.back().x(),1.2,.041);
  EXPECT_LT(maxLateralDeviation(result.path),.081);
  expectNativeCollisionFree(*map,result.path);
  RecordProperty("search_ms",std::to_string(result.milliseconds));
}

TEST(NearFieldNativeSearch, OccupiedBoxInObservedSpaceProducesGenuineDetour) {
  auto map=GridMapTestAccess::observedFree();
  GridMapTestAccess::box(*map,{-.12,-.2,.12},{.12,.2,1.},1);
  ASSERT_NE(map->getInflateOccupancy({0.,0.,.55},0.),0);
  const auto result=search(map);
  ASSERT_EQ(result.status,ASTAR_RET::SUCCESS);
  EXPECT_GT(maxLateralDeviation(result.path),.49);
  expectNativeCollisionFree(*map,result.path);
  RecordProperty("search_ms",std::to_string(result.milliseconds));
}

TEST(NearFieldNativeSearch, LowExternalObstacleCannotBeRemovedAsGroundOrSelf) {
  auto map=GridMapTestAccess::observedFree();
  GridMapTestAccess::box(*map,{-.025,-.025,.11},{.025,.025,.15},1);
  ASSERT_NE(map->getInflateOccupancy({0.,0.,.55},0.),0);
  const auto result=search(map);
  ASSERT_EQ(result.status,ASTAR_RET::SUCCESS);
  EXPECT_GT(maxLateralDeviation(result.path),.29);
  expectNativeCollisionFree(*map,result.path);
  RecordProperty("search_ms",std::to_string(result.milliseconds));
}

TEST(NearFieldNativeSearch, ThinPoleRemainsBlockingWhileVisibleDetourExists) {
  auto map=GridMapTestAccess::observedFree();
  GridMapTestAccess::box(*map,{-.01,-.01,.12},{.01,.01,1.},1);
  const auto result=search(map);
  ASSERT_EQ(result.status,ASTAR_RET::SUCCESS);
  EXPECT_GT(maxLateralDeviation(result.path),.29);
  expectNativeCollisionFree(*map,result.path);
  RecordProperty("search_ms",std::to_string(result.milliseconds));
}

TEST(NearFieldNativeSearch, FullySealedOccupiedWallDoesNotReturnAPath) {
  auto map=GridMapTestAccess::observedFree();
  GridMapTestAccess::box(*map,{-.04,-3.,-2.},{.04,3.,2.},1);
  const auto result=search(map);
  EXPECT_EQ(result.status,ASTAR_RET::SEARCH_ERR);
  EXPECT_TRUE(result.path.empty());
  // SEARCH_ERR may mean exhausted or bounded timeout. Both deny a candidate;
  // this is not a claim about timing completeness of the search algorithm.
  RecordProperty("search_ms",std::to_string(result.milliseconds));
}

TEST(NearFieldNativeSearch, UnobservedStripCannotBeUsedAsAHiddenDetour) {
  auto map=GridMapTestAccess::observedFree();
  GridMapTestAccess::box(*map,{-.04,-3.,-2.},{.04,3.,2.},2);
  const auto result=search(map);
  EXPECT_EQ(result.status,ASTAR_RET::SEARCH_ERR);
  EXPECT_TRUE(result.path.empty());
  RecordProperty("search_ms",std::to_string(result.milliseconds));
}

TEST(NearFieldNativeSearch, FreeHeadingEndpointsDoNotProveSafeInPlaceTurnSweep) {
  auto map=GridMapTestAccess::observedFree();
  GridMapTestAccess::box(*map,{.35,.35,.5},{.35,.35,.5},1);
  const Point body(0.,0.,.55);
  const double half_pi=std::acos(-1.)/2.;
  ASSERT_EQ(map->getInflateOccupancy(body,0.),0);
  ASSERT_EQ(map->getInflateOccupancy(body,half_pi),0);
  std::size_t collision_samples=0;
  for(unsigned i=0;i<=90;++i)
    collision_samples+=map->getInflateOccupancy(body,half_pi*i/90.)!=0;
  EXPECT_GT(collision_samples,0U);
  RecordProperty("blocked_rotation_samples",collision_samples);
  // This is an execution-design counterexample, not a newly implemented
  // swept-volume admission guarantee. Production motion remains disabled.
}
