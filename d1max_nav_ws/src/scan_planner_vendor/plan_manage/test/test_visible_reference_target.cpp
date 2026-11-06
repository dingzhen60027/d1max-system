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
      Eigen::Vector3i id;m.posToIndex({x,y,z},id);
      // Native updates must maintain BOTH official candidate inflation and
      // strict raw evidence. Direct raw-buffer writes left A*'s official
      // obstacle layer empty and could route straight through the fixture box.
      if(m.isInMap(id))m.applyOccupancyUpdate(id,odds);
    }
  }
  static void rawSnapshot(GridMap& m) {
    m.mp_.use_projected_rays_=true;m.mp_.cloud_pose_max_age_=.5;
    m.free_observation_stamps_.assign(m.md_.occupancy_buffer_.size(),100000000000LL);
    m.ray_integrated_stamps_={100000000000LL,100000000000LL};m.integrated_cloud_stamp_ns_=100000000000LL;
    m.snapshot_clock_ns_=100100000000LL;m.snapshot_captured_=std::chrono::steady_clock::now();
    m.collision_snapshot_=true;m.enforce_free_freshness_=true;m.ray_clock_fault_=false;
  }
  static void box(GridMap& m) {
    m.mp_.collision_model_="oriented_box";m.mp_.box_length_=.94;m.mp_.box_width_=.30;
    m.mp_.body_below_=m.mp_.body_above_=m.mp_.planning_body_below_=m.mp_.planning_body_above_=.25;
    m.rebuildInflationOffsets();
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
// Literal sampled surfaces for deterministic stair candidate tests. These are
// software geometry fixtures, not a stair-mode/physical acceptance record.
d1max_planning_interfaces::msg::SupportReference stairs(bool down=false) {
  auto s=ground();s.floor_id="stair_lower";s.segment_kind=down?"stair_down":"stair_up";
  s.required_mode="stair";s.support_reference_id="stair-source";
  s.reason="original_stair_surface_returns_associated";
  s.max_support_slope_rad=s.max_support_step_m=0.;s.support_xy_radius_m=.16;
  for(auto& p:s.support_ground_xyz) {
    const double height=.12*std::floor(std::max(0.,p.x)/.3);
    p.z=down?.6-height:height;
  }
  return s;
}
scan_planner::DiscreteReference stairRoute(bool down=false) {
  scan_planner::DiscreteReference r;
  std::vector<Point> points;
  for(int i=0;i<=6;++i)points.emplace_back(i*.3,0.,.55+(down?.6-i*.12:i*.12));
  r.set(points);return r;
}
}
TEST(PlanningSupport,StairSourceIsNotLegacyFlatMotionEvidence) {
  const auto packet=stairs();const scan_planner::PlanningSupport planning(packet);
  ASSERT_TRUE(planning.valid);EXPECT_TRUE(planning.isStair());
  EXPECT_FALSE(scan_planner::MotionSupport(packet).valid);
  EXPECT_FALSE(planning.nearest(.6,0.)); // Floor identity is required for stairs.
  const auto lower=planning.nearest(.6,0.,.24);ASSERT_TRUE(lower);EXPECT_NEAR(lower->z,.24,1e-9);
  EXPECT_FALSE(planning.nearest(.6,0.,3.24));
  auto multilevel=packet;
  for(const auto& p:packet.support_ground_xyz) {auto upper=p;upper.z+=3.;multilevel.support_ground_xyz.push_back(upper);}
  const scan_planner::PlanningSupport layered(multilevel);ASSERT_TRUE(layered.valid);
  ASSERT_TRUE(layered.nearest(.6,0.,.24));EXPECT_NEAR(layered.nearest(.6,0.,.24)->z,.24,1e-9);
  ASSERT_TRUE(layered.nearest(.6,0.,3.24));EXPECT_NEAR(layered.nearest(.6,0.,3.24)->z,3.24,1e-9);
}
TEST(PlanningSupport,RejectsInventedStairProvenanceAndMalformedGeometry) {
  for(int corruption=0;corruption<9;++corruption) {
    auto s=stairs();
    if(corruption==0)s.reason="";
    if(corruption==1)s.floor_id="floor1";
    if(corruption==2)s.segment_kind="floor";
    if(corruption==3)s.required_mode="general";
    if(corruption==4)s.max_support_step_m=.08;
    if(corruption==5)s.support_reference_id.clear();
    if(corruption==6)s.support_hash=std::string(64,'z');
    if(corruption==7)s.support_ground_xyz.front().z=std::numeric_limits<double>::quiet_NaN();
    if(corruption==8)s.verified=false;
    EXPECT_FALSE(scan_planner::PlanningSupport(s).valid)<<corruption;
  }
}
TEST(PlanningSupport,AscendingDescendingAndStairLandingReachNativeSideSelector) {
  for(bool down:{false,true}) {
    auto map=GridMapTestAccess::make();GridMapTestAccess::box(*map);
    auto packet=stairs(down);const scan_planner::PlanningSupport source(packet);
    const auto reference=stairRoute(down);const auto body=reference.sample(0.);
    const auto found=scan_planner::selectPlanningSideTarget(reference,body,0.,0.,2.,source,*map,"odom",
      std::make_shared<scan_planner::SolveBudget>());
    ASSERT_TRUE(found.target.valid)<<down<<": "<<found.target.reason;
    EXPECT_GT(found.target.arc,.4);EXPECT_GT(std::abs(found.target.point.z()-body.z()),.1);
    EXPECT_TRUE(found.support_path.front().isApprox(body));
    for(const auto& point:found.support_path) {
      const auto actual=source.nearest(point.x(),point.y(),point.z()-packet.body_reference_height_m);
      ASSERT_TRUE(actual);EXPECT_NEAR(point.z()-actual->z,packet.body_reference_height_m,1e-9);
      const auto delta=found.target.point-body;
      EXPECT_EQ(map->getPlanningInflateOccupancy(point,std::atan2(delta.y(),delta.x())),0);
    }
    // These same semantics support a landing while the required robot mode
    // remains stair; a landing must not silently turn into GENERAL evidence.
    packet.segment_kind="landing";for(auto& p:packet.support_ground_xyz)p.z=0.;
    const scan_planner::PlanningSupport landing(packet);
    EXPECT_TRUE(scan_planner::selectPlanningSideTarget(route(),{.65,0.,.55},0.,.65,2.,landing,*map,"odom",
      std::make_shared<scan_planner::SolveBudget>()).target.valid);
    EXPECT_FALSE(scan_planner::MotionSupport(packet).valid);
  }
}
TEST(PlanningSupport,MissingOrOtherFloorSupportAndRealObstaclesStayRejected) {
  for(int failure=0;failure<4;++failure) {
    auto map=GridMapTestAccess::make();GridMapTestAccess::box(*map);auto packet=stairs();
    if(failure==0)for(auto& p:packet.support_ground_xyz)p.z+=3.;
    if(failure==1)packet.support_ground_xyz.erase(std::remove_if(packet.support_ground_xyz.begin(),
      packet.support_ground_xyz.end(),[](const auto& p){return std::abs(p.y)>.05;}),packet.support_ground_xyz.end());
    if(failure==2)GridMapTestAccess::fill(*map,-.1,.1,-.1,.1,2.);
    if(failure==3)packet.frame_id="map";
    const scan_planner::PlanningSupport support(packet);
    const auto found=scan_planner::selectPlanningSideTarget(stairRoute(),{0.,0.,.55},0.,0.,2.,support,*map,"odom",
      std::make_shared<scan_planner::SolveBudget>());
    EXPECT_FALSE(found.target.valid)<<failure;EXPECT_TRUE(found.support_path.empty());
  }
}
TEST(PlanningSupport,FlatSelectionRemainsExactlyTheLegacyGeometryPolicy) {
  auto map=GridMapTestAccess::make();GridMapTestAccess::fill(*map,1.4,1.8,-.18,.18,2.);
  const auto s=ground();const scan_planner::MotionSupport legacy(s);const scan_planner::PlanningSupport current(s);
  const auto before=scan_planner::selectPlanningSideTarget(route(),{.65,0,.55},0.,.65,2.,legacy,*map,"odom",
      std::make_shared<scan_planner::SolveBudget>());
  const auto after=scan_planner::selectPlanningSideTarget(route(),{.65,0,.55},0.,.65,2.,current,*map,"odom",
      std::make_shared<scan_planner::SolveBudget>());
  ASSERT_EQ(before.target.valid,after.target.valid);ASSERT_EQ(before.support_path.size(),after.support_path.size());
  EXPECT_EQ(before.target.queries,after.target.queries);EXPECT_EQ(before.candidates,after.candidates);
  for(std::size_t i=0;i<before.support_path.size();++i)EXPECT_TRUE(before.support_path[i].isApprox(after.support_path[i]));
}
TEST(PlanningSupport,TerrainAttitudeUsesSurfaceNormalAndRosPitchRollConvention) {
  auto s=stairs();s.support_ground_xyz.clear();
  for(int x=-16;x<=16;++x)for(int y=-16;y<=16;++y) {
    geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;p.z=.5*p.x+.1*p.y;
    s.support_ground_xyz.push_back(p);
  }
  const scan_planner::PlanningSupport plane(s);const auto fit=plane.terrainFit({0.,0.,.55});
  ASSERT_TRUE(fit.valid);EXPECT_LT(fit.rms,1e-9);EXPECT_GT(fit.samples,100U);
  EXPECT_TRUE(fit.normal.isApprox(Point(-.5,-.1,1.).normalized()));
  for(double yaw:{0.,1.2,3.141592653589793}) {
    const auto angles=plane.terrainAttitude({0.,0.,.55},yaw);ASSERT_TRUE(angles);
    const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(yaw,Point::UnitZ())*
      Eigen::AngleAxisd((*angles)[0],Point::UnitY())*Eigen::AngleAxisd((*angles)[1],Point::UnitX())).toRotationMatrix();
    EXPECT_TRUE(rotation.col(2).isApprox(fit.normal));
  }
  ASSERT_TRUE(plane.terrainAttitude({0.,0.,.55},0.));
  EXPECT_LT((*plane.terrainAttitude({0.,0.,.55},0.))[0],0.);
  EXPECT_GT((*plane.terrainAttitude({0.,0.,.55},3.141592653589793))[0],0.);
  EXPECT_FALSE(plane.terrainAttitude({0.,0.,3.55},0.)); // Other floor cannot be borrowed.
  const scan_planner::PlanningSupport flat(ground());const auto horizontal=flat.terrainAttitude({0.,0.,.55},.7);
  ASSERT_TRUE(horizontal);EXPECT_TRUE(horizontal->isZero());
}
TEST(PlanningSupport,TreadNormalIsNotIndividualStepDerivativeAndMixedSurfacesFailFit) {
  auto s=stairs();const scan_planner::PlanningSupport steps(s);
  const auto first=steps.terrainFit({.75,0.,.79});const auto second=steps.terrainFit({.76,0.,.79});
  ASSERT_TRUE(first.valid);ASSERT_TRUE(second.valid);EXPECT_LT(first.rms,.06);
  EXPECT_LT((first.normal-second.normal).norm(),.03);
  EXPECT_NEAR(std::atan2(-first.normal.x(),first.normal.z()),std::atan(.4),.06);
  auto mixed=s;for(const auto& p:s.support_ground_xyz){auto other=p;other.z+=.4;mixed.support_ground_xyz.push_back(other);}
  const scan_planner::PlanningSupport ambiguous(mixed);
  EXPECT_FALSE(ambiguous.terrainFit({.75,0.,.99}).valid);
  auto line=s;line.support_ground_xyz.erase(std::remove_if(line.support_ground_xyz.begin(),line.support_ground_xyz.end(),
    [](const auto& p){return std::abs(p.y)>.01;}),line.support_ground_xyz.end());
  EXPECT_FALSE(scan_planner::PlanningSupport(line).terrainFit({.75,0.,.79}).valid);
  EXPECT_FALSE(scan_planner::PlanningSupport::attitudeFromNormal({0.,0.,-1.},0.));
}
TEST(PlanningSupport,StairVelocityFollowsMeanSupportNotTheRiserChord) {
  for(bool down:{false,true}) {
    const scan_planner::PlanningSupport support(stairs(down));
    const Point body(.75,0.,.55+(down?.36:.24));
    const Point riser(.02,0.,down?-.12:.12);
    const auto fit=support.terrainFit(body);ASSERT_TRUE(fit.valid);
    const auto tangent=support.tangentOnSurface(body,riser);ASSERT_TRUE(tangent);
    EXPECT_DOUBLE_EQ(tangent->x(),riser.x());EXPECT_DOUBLE_EQ(tangent->y(),riser.y());
    EXPECT_NEAR(tangent->dot(fit.normal),0.,1e-12);
    EXPECT_NEAR(tangent->z()/tangent->x(),down?-.4:.4,.07);
    EXPECT_LT(std::abs(tangent->z()),std::abs(riser.z())*.2);
    const Point velocity=tangent->normalized()*.3;
    EXPECT_NEAR(velocity.norm(),.3,1e-12);EXPECT_GT(velocity.head<2>().norm(),.26);
    EXPECT_LT(velocity.z()*(down?1.:-1.),0.);
  }
}
TEST(PlanningSupport,SurfaceTangentPreservesFlatAndMissingEvidenceFallback) {
  const scan_planner::PlanningSupport flat(ground()),stair(stairs());
  EXPECT_FALSE(flat.tangentOnSurface({.75,0.,.55},{.1,0.,.01}));
  EXPECT_FALSE(stair.tangentOnSurface({.75,0.,3.79},{.1,0.,.1}));
  EXPECT_FALSE(stair.tangentOnSurface({.75,0.,.79},{0.,0.,.1}));
  EXPECT_FALSE(stair.tangentOnSurface({.75,0.,.79},{NAN,0.,.1}));
  auto bad=stairs();bad.verified=false;
  EXPECT_FALSE(scan_planner::PlanningSupport(bad).tangentOnSurface({.75,0.,.79},{.1,0.,.1}));
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
    // A* is the official SCAN candidate search. Its zero is not strict
    // observed-free certification; the visible support/entry lane and final
    // execution validation remain independent, including unknown behind box.
    for(int k=0;k<=n;++k)EXPECT_EQ(map->getPlanningInflateOccupancy(path[i-1]+delta*(double(k)/n),yaw),0);
  }
  // Query beyond the box's whole reflected cylinder envelope. At x=2.2
  // the rear cylinder still intersects the real box and must report occupied.
  EXPECT_EQ(map->getInflateOccupancy({2.65,0,.55},0.),2); // Occluded evidence stays unknown.
  EXPECT_EQ(map->getPlanningInflateOccupancy({1.6,0,.55},0.),1); // Real box is still blocked.
}
TEST(VisibleReferenceTarget,HeldHeadingSideTargetAndProductionSearchUseTheSameRectangle) {
  auto map=GridMapTestAccess::make();GridMapTestAccess::box(*map);
  // Two points outside the held rectangle lie on the rear/front corner's
  // swept volume for positive/negative turns. A centerline point below the
  // body does not block every tangent target and is not a rotation fixture.
  GridMapTestAccess::fill(*map,.30,.325,-.325,-.3,2.);
  GridMapTestAccess::fill(*map,1.0,1.025,-.325,-.3,2.);
  const scan_planner::MotionSupport support(ground());
  const auto tangent=scan_planner::selectPlanningSideTarget(route(),{.65,0.,.55},0.,.65,2.,support,
      *map,"odom",std::make_shared<scan_planner::SolveBudget>());
  EXPECT_FALSE(tangent.target.valid);
  const auto held=scan_planner::selectPlanningSideTarget(route(),{.65,0.,.55},0.,.65,2.,support,
      *map,"odom",std::make_shared<scan_planner::SolveBudget>(),1024,scan_planner::BodyHeadingPolicy::HoldHeading);
  ASSERT_TRUE(held.target.valid)<<held.target.reason;ASSERT_GE(held.support_path.size(),2U);
  for(const auto& p:held.support_path)EXPECT_EQ(map->getPlanningInflateOccupancy(p,0.),0);
  AStar search;search.initGridMap(map,{100,100,32});search.setFrozenBodyYaw(0.);
  ASSERT_EQ(search.AstarSearch(.05,held.support_path.front(),held.support_path.back(),false,true),ASTAR_RET::SUCCESS);
  for(const auto& p:search.getPath())EXPECT_EQ(map->getPlanningInflateOccupancy(p,0.),0);
  GridMapTestAccess::fill(*map,.625,.65,-.025,.025,2.);
  const auto blocked=scan_planner::selectPlanningSideTarget(route(),{.65,0.,.55},0.,.65,2.,support,
      *map,"odom",std::make_shared<scan_planner::SolveBudget>(),1024,scan_planner::BodyHeadingPolicy::HoldHeading);
  EXPECT_FALSE(blocked.target.valid); // Same held body footprint, no clearing.
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
  for(int mode=0;mode<5;++mode) {
    SCOPED_TRACE(mode);
    auto map=GridMapTestAccess::make();auto s=ground();
    if(mode==0)for(auto& p:s.support_ground_xyz)if(std::abs(p.y)>.1)p.z=.35;
    if(mode==1) {
      // A label is not geometry. Floor2 is supported by the mainline, but its
      // actual +3m ground cannot support this floor1-height measured body.
      s.floor_id="floor2";
      for(auto& p:s.support_ground_xyz)p.z+=3.;
    }
    if(mode==2)s.segment_kind="stairs";
    if(mode==3)s.frame_id="map";
    if(mode==4)s.floor_id="floor3";
    EXPECT_FALSE(select(*map,scan_planner::MotionSupport(s)).target.valid);
  }
}
TEST(VisibleReferenceTarget,SecondFloorOwnLocalGroundIsNotRejectedByHardcodedFloorOne) {
  auto map=GridMapTestAccess::make();auto s=ground();s.floor_id="floor2";
  // Both support and measured body already share the same continuous odom
  // frame. This API must use their real relative height, not hardcode floor1.
  const auto result=select(*map,scan_planner::MotionSupport(s));
  ASSERT_TRUE(result.target.valid)<<result.target.reason;
  ASSERT_GE(result.support_path.size(),2U);
  for(const auto& p:result.support_path)EXPECT_DOUBLE_EQ(p.z(),.55);
}
TEST(VisibleReferenceTarget,CancelledAndFiniteQueryBudgetCannotProduceTarget) {
  auto map=GridMapTestAccess::make();scan_planner::MotionSupport support(ground());
  auto budget=std::make_shared<scan_planner::SolveBudget>();budget->cancel();
  EXPECT_FALSE(select(*map,support,budget).target.valid);
  const auto result=select(*map,support,std::make_shared<scan_planner::SolveBudget>(),1);
  EXPECT_FALSE(result.target.valid);EXPECT_LE(result.target.queries,1U);
}
