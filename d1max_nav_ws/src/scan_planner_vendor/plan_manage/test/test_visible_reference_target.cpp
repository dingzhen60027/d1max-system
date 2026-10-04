#include <gtest/gtest.h>
#include <plan_manage/visible_reference_target.hpp>
#include <path_searching/dyn_a_star.h>

using Point=Eigen::Vector3d;
// Real production GridMap and AStar, initialized in memory. This test does not
// claim raw-ray integration or physical wheel/leg calibration.
struct GridMapTestAccess {
  static GridMap::Ptr make() {
    auto m=std::make_shared<GridMap>();auto& p=m->mp_;auto& d=m->md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={120,100,60};
    p.map_origin_idx_={20,0,10};m->updateMapBoundaryFromIndex();
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.require_observed_free_=true;p.double_cylinder_radius_=.29;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.45;p.frame_id_="odom";
    d.occupancy_buffer_.assign(120*100*60,-1.);
    d.occupancy_buffer_inflate_.assign(120*100*60,0);d.occupancy_buffer_inflate_cnt_.assign(120*100*60,0);
    m->rebuildInflationOffsets();return m;
  }
  static void fill(GridMap& m,double xmin,double xmax,double ymin,double ymax,double odds) {
    for(double x=xmin;x<=xmax;x+=.025)for(double y=ymin;y<=ymax;y+=.025)for(double z=.025;z<=1.15;z+=.025) {
      Eigen::Vector3i id;m.posToIndex({x,y,z},id);if(m.isInMap(id))m.md_.occupancy_buffer_[m.toAddress(id)]=odds;
    }
  }
  static void rawSnapshot(GridMap& m) {
    m.mp_.use_projected_rays_=true;m.mp_.cloud_pose_max_age_=.5;
    m.free_observation_stamps_.assign(m.md_.occupancy_buffer_.size(),100000000000LL);
    m.ray_integrated_stamps_={100000000000LL,100000000000LL};m.integrated_cloud_stamp_ns_=100000000000LL;
    m.snapshot_clock_ns_=100100000000LL;m.snapshot_captured_=std::chrono::steady_clock::now();
    m.collision_snapshot_=true;m.enforce_free_freshness_=true;m.ray_clock_fault_=false;
  }
};
namespace {
scan_planner::DiscreteReference route() {scan_planner::DiscreteReference r;r.set({{0,0,.55},{3.2,0,.55}});return r;}
d1max_planning_interfaces::msg::SupportReference ground() {
  d1max_planning_interfaces::msg::SupportReference s;s.verified=true;s.floor_id="floor1";s.segment_kind="floor";
  s.required_mode="general";s.frame_id="odom";s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');
  s.support_xy_radius_m=.05;s.body_reference_height_m=.55;s.max_support_slope_rad=.15;s.max_support_step_m=.05;
  for(int x=-20;x<=70;++x)for(int y=-40;y<=40;++y) {
    geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;s.support_ground_xyz.push_back(p);
  }
  return s;
}
scan_planner::VisibleReferenceTarget select(GridMap& m,const scan_planner::MotionSupport& support,
    const scan_planner::SolveBudget::Ptr& budget=std::make_shared<scan_planner::SolveBudget>(),std::size_t query_limit=1024) {
  return scan_planner::selectVisibleSideTarget(route(),{.65,0,.55},0.,.65,2.,support,m,"odom",budget,query_limit);
}
}
TEST(VisibleReferenceTarget,ProductionObservedSideStepAndNativeAstarAroundBoxWithoutSeeingBehindIt) {
  auto map=GridMapTestAccess::make();
  GridMapTestAccess::fill(*map,1.4,1.8,-.18,.18,2.);
  GridMapTestAccess::fill(*map,1.825,3.2,-.30,.30,-1.01);
  const auto original=route();
  const auto old=scan_planner::selectReferenceTarget(original,{.65,0,.55},.65,2.,{},
    [&](const Point& p,double yaw){return map->getInflateOccupancy(p,yaw);});
  ASSERT_FALSE(old.valid); // The visible straight route ends before min advance.
  scan_planner::MotionSupport support(ground());const auto found=select(*map,support);
  ASSERT_TRUE(found.target.valid)<<found.target.reason;
  EXPECT_GT(std::abs(found.target.point.y()),.4);EXPECT_LE(found.candidates,40U);
  EXPECT_LE(found.target.queries,1024U);EXPECT_EQ(found.target.reason,"reference_target_visible_side");
  ASSERT_GE(found.support_path.size(),2U);
  for(const auto& p:found.support_path)EXPECT_DOUBLE_EQ(p.z(),.55);
  EXPECT_TRUE(original.points.back().isApprox(Point(3.2,0,.55))); // Destination not replaced.
  AStar planner;planner.initGridMap(map,{100,100,32});
  ASSERT_EQ(planner.AstarSearch(.05,found.support_path.front(),found.support_path.back(),true),ASTAR_RET::SUCCESS);
  const auto path=planner.getPath();ASSERT_GE(path.size(),2U);
  for(std::size_t i=1;i<path.size();++i) {
    const Point delta=path[i]-path[i-1];const double yaw=std::atan2(delta.y(),delta.x());
    const int n=std::max(1,int(std::ceil(delta.norm()/.0125)));
    for(int k=0;k<=n;++k)EXPECT_EQ(map->getInflateOccupancy(path[i-1]+delta*(double(k)/n),yaw),0);
  }
}
TEST(VisibleReferenceTarget,NoObservedSideSpaceDoesNotBecomeAFreeDetour) {
  for(double state:{-1.01,0.,2.}) {
    auto map=GridMapTestAccess::make();
    GridMapTestAccess::fill(*map,-.1,2.5,-1.8,-.35,state);
    GridMapTestAccess::fill(*map,-.1,2.5,.35,1.8,state);
    const auto found=select(*map,scan_planner::MotionSupport(ground()));
    EXPECT_FALSE(found.target.valid);EXPECT_TRUE(found.support_path.empty());
  }
}
TEST(VisibleReferenceTarget,ExecutionSideTargetIncludesSameReactionAndBrakingEnvelope) {
  auto map=GridMapTestAccess::make();GridMapTestAccess::rawSnapshot(*map);
  GridMapTestAccess::fill(*map,1.4,1.8,-.18,.18,2.);
  GridMapTestAccess::fill(*map,1.825,3.2,-.30,.30,-1.01);
  scan_planner::MotionSupport support(ground());
  const scan_planner::BrakingModel braking{.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'a')};
  const auto selected=scan_planner::selectVisibleSideTarget(route(),{.65,0,.55},0.,.65,2.,support,
      *map,"odom",std::make_shared<scan_planner::SolveBudget>(),1024,&braking);
  ASSERT_TRUE(selected.target.valid)<<selected.target.reason;
  const auto corridor=scan_planner::validateStraightBrakingCorridor(*map,support,braking,
      {.65,0,.55},selected.target.point,"odom");
  EXPECT_TRUE(corridor.valid)<<corridor.reason;
}
TEST(VisibleReferenceTarget,MissingSideSupportCannotBorrowCenterlineHeight) {
  auto map=GridMapTestAccess::make();auto s=ground();
  s.support_ground_xyz.erase(std::remove_if(s.support_ground_xyz.begin(),s.support_ground_xyz.end(),
    [](const auto& p){return std::abs(p.y)>.05;}),s.support_ground_xyz.end());
  const auto found=select(*map,scan_planner::MotionSupport(s));EXPECT_FALSE(found.target.valid);
  EXPECT_TRUE(found.support_path.empty());
}
TEST(VisibleReferenceTarget,SideHeightComesFromOriginalSupportInsteadOfFlatRoute) {
  auto map=GridMapTestAccess::make();auto s=ground();
  for(auto& p:s.support_ground_xyz)p.z=.02*p.y;
  const auto found=select(*map,scan_planner::MotionSupport(s));
  ASSERT_TRUE(found.target.valid)<<found.target.reason;
  EXPECT_GT(std::abs(found.target.point.z()-.55),.005);
  EXPECT_NEAR(found.target.point.z(),.55+.02*found.target.point.y(),.001);
}
TEST(VisibleReferenceTarget,StairsWrongFloorOrFrameCannotBeChosenAsSideSupport) {
  for(int mode=0;mode<4;++mode) {
    auto map=GridMapTestAccess::make();auto s=ground();
    if(mode==0)for(auto& p:s.support_ground_xyz)if(std::abs(p.y)>.1)p.z=.35;
    if(mode==1)s.floor_id="floor2";
    if(mode==2)s.segment_kind="stairs";
    if(mode==3)s.frame_id="map";
    EXPECT_FALSE(select(*map,scan_planner::MotionSupport(s)).target.valid);
  }
}
TEST(VisibleReferenceTarget,CancelledAndFiniteQueryBudgetCannotProduceTarget) {
  auto map=GridMapTestAccess::make();scan_planner::MotionSupport support(ground());
  auto budget=std::make_shared<scan_planner::SolveBudget>();budget->cancel();
  EXPECT_FALSE(select(*map,support,budget).target.valid);
  const auto result=select(*map,support,std::make_shared<scan_planner::SolveBudget>(),1);
  EXPECT_FALSE(result.target.valid);EXPECT_LE(result.target.queries,1U);
}
