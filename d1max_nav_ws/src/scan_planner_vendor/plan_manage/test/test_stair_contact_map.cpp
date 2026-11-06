#include <gtest/gtest.h>
#include <plan_manage/stair_attitude_scope.hpp>
#include <future>

struct GridMapTestAccess {
  static void configure(GridMap& map) {
    auto& p=map.mp_;auto& d=map.md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={40,40,40};p.map_origin_idx_.setZero();
    p.collision_model_="oriented_box";p.box_length_=1.03;p.box_width_=.40;
    p.box_nominal_length_=.93;p.box_nominal_width_=.30;p.stair_planar_clearance_=.025;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    p.stair_subvoxel_collision_=true;p.stair_contact_body_below_=.12;
    p.double_cylinder_radius_=.2;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.25;p.map_sliding_en_=false;
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.prob_hit_log_=3.;p.prob_miss_log_=-.5;p.max_ray_length_=4.;p.local_update_range_={2.,2.,2.};
    p.planning_collision_policy_="official";p.require_observed_free_=false;p.frame_id_="odom";
    map.updateMapBoundaryFromIndex();
    d.occupancy_buffer_.resize(64000);d.occupancy_buffer_inflate_.resize(64000);
    d.occupancy_buffer_inflate_cnt_.resize(64000);d.count_hit_.resize(64000);
    d.count_hit_and_miss_.resize(64000);d.flag_rayend_.resize(64000);d.flag_traverse_.resize(64000);
    map.rebuildInflationOffsets();map.resetAllMapData();
  }
  static auto raw(const GridMap& map){return map.md_.occupancy_buffer_;}
  static auto bounds(GridMap& map,const Eigen::Vector3d& point) {
    Eigen::Vector3i cell;map.posToIndex(point,cell);
    return map.md_.occupied_hit_bounds_[map.toAddress(cell)].world(cell,map.mp_.resolution_);
  }
  static void discardBounds(GridMap& map) {
    map.md_.occupied_hit_bounds_.clear();map.invalidateCollisionQueryCaches();
  }
  static void markCoarseOnly(GridMap& map,const Eigen::Vector3d& point) {
    Eigen::Vector3i cell;map.posToIndex(point,cell);map.markCoarseOnlyOccupancy(map.toAddress(cell));
  }
  static void disableStairSubvoxel(GridMap& map) {
    map.mp_.stair_subvoxel_collision_=false;map.invalidateCollisionQueryCaches();
  }
  static void strictObservedFree(GridMap& map) {
    map.mp_.require_observed_free_=true;map.mp_.planning_collision_policy_="observed_free";
    map.invalidateCollisionQueryCaches();
  }
  static bool rays(GridMap& map,const std::vector<Eigen::Vector3d>& endpoints) {
    map.mp_.use_projected_rays_=true;map.md_.proj_points_=endpoints;
    map.md_.proj_origins_.assign(endpoints.size(),Eigen::Vector3d(-.65,.025,.8));
    map.md_.proj_points_cnt=endpoints.size();map.md_.ray_pos_=map.md_.proj_origins_.front();
    map.projected_ray_stamps_.assign(endpoints.size(),101000000000LL);
    map.projected_diagnostics_.clear();map.beginCollisionQuery();return map.raycastProcess();
  }
};
namespace {
using namespace scan_planner;
using Vec=Eigen::Vector3d;
std::shared_ptr<const PlanningSupport> source(double z=.276,bool qualified=true) {
  d1max_planning_interfaces::msg::SupportReference s;
  s.support_reference_id="qualified-tread";s.support_hash=std::string(64,'a');
  s.support_map_sha256=std::string(64,'b');s.frame_id="odom";
  s.floor_id="stair_lower";s.segment_kind="stair_up";s.required_mode="stair";
  s.reason="original_stair_surface_returns_associated";s.verified=true;
  s.support_xy_radius_m=.12;s.body_reference_height_m=.50;s.version.segment_id="lower";
  for(int x=-5;x<=5;++x)for(int y=-5;y<=5;++y) {
    geometry_msgs::msg::Point p;p.x=x*.02;p.y=y*.02;p.z=z;
    if(qualified)s.stair_contact_indices.push_back(s.support_ground_xyz.size());
    s.support_ground_xyz.push_back(p);
  }
  return std::make_shared<const PlanningSupport>(s);
}
MeasuredBodyPose measured(Vec position={0.,0.,.50}) {
  MeasuredBodyPose m;m.frame="odom";m.position=position;m.orientation=Eigen::Quaterniond::Identity();return m;
}
std::shared_ptr<const PlanningSupport> twoTreads(double gap,const std::string& kind) {
  auto s=source()->source;s.support_ground_xyz.clear();s.stair_contact_indices.clear();s.segment_kind=kind;
  for(int level=0;level<2;++level)for(int x=0;x<9;++x)for(int y=-10;y<=10;++y) {
    geometry_msgs::msg::Point p;p.x=(level?1.:-1.)*(gap*.5+x*.02);p.y=y*.02;p.z=level*.17;
    s.stair_contact_indices.push_back(s.support_ground_xyz.size());s.support_ground_xyz.push_back(p);
  }
  return std::make_shared<const PlanningSupport>(s);
}
std::shared_ptr<const PlanningSupport> independentlyObservedRiser(const std::string& kind,double face=0.) {
  auto s=twoTreads(.04,kind)->source;
  for(int y=-10;y<=10;++y)for(int z=0;z<=17;++z) {
    geometry_msgs::msg::Point p;p.x=face;p.y=.02*y;p.z=.01*z;
    s.stair_riser_xyz.push_back(p);s.stair_riser_owner.push_back(0);
  }
  return std::make_shared<const PlanningSupport>(s);
}
std::shared_ptr<const PlanningSupport> boundarySource(double upper_z,bool qualified=true) {
  // Keep actual support under the body's .50 m reference AND a separate higher
  // tread ahead inside its longitudinal lower envelope. A high tread alone at
  // the body's XY would fail same-level clearance qualification and would not
  // exercise the stair subvoxel branch being tested.
  auto message=source(upper_z,qualified)->source;
  for(auto& point:message.support_ground_xyz)point.x+=.16;
  const auto lower=source(0.);
  for(auto point:lower->source.support_ground_xyz) {
    point.x-=.12;
    message.stair_contact_indices.push_back(message.support_ground_xyz.size());
    message.support_ground_xyz.push_back(point);
  }
  return std::make_shared<const PlanningSupport>(message);
}
}

TEST(StairContactMap,EntryUsesQualifiedHitNotWholeWindowOrBodyFloorLabel) {
  GridMap map;GridMapTestAccess::configure(map);map.setOccupiedHit({.02,.02,.276});
  const auto raw=GridMapTestAccess::raw(map);const Vec body(0.,0.,.50);
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
  {
    StairAttitudeScope scope(map,source(),measured());
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),0);
    EXPECT_EQ(map.getInflateOccupancy(body,0.),0);
    const auto proof=map.inspectInflateOccupancy(body,0.,true);
    EXPECT_EQ(proof.unique_counts[1],1U); // still occupied, not fabricated FREE
    EXPECT_EQ(proof.subvoxels.qualified_support_contacts,1U);
  }
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);
}

TEST(StairContactMap,UnqualifiedSupportAndWrongFrameDoNotExcuseCollision) {
  GridMap map;GridMapTestAccess::configure(map);map.setOccupiedHit({.02,.02,.276});
  const Vec body(0.,0.,.50);
  {StairAttitudeScope scope(map,source(.276,false),measured());
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);}
  auto foreign=source()->source;foreign.frame_id="map";
  {StairAttitudeScope scope(map,std::make_shared<const PlanningSupport>(foreign),measured());
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);}
}

TEST(StairContactMap,NewMixedHitInSameOctantRemainsBlockedAcrossSnapshots) {
  GridMap map;GridMapTestAccess::configure(map);map.setOccupiedHit({.02,.02,.276});
  const Vec body(0.,0.,.50);
  StairAttitudeScope scope(map,source(),measured());
  ASSERT_EQ(map.getPlanningInflateOccupancy(body,0.),0);
  map.setOccupiedHit({.02,.02,.296}); // identical octant, not a supported tread
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
  map.setOccupiedHit({.02,.02,.276}); // ground cannot overwrite mixed history
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
  GridMap reader;map.copyCollisionSnapshotTo(reader,1000000000,std::chrono::steady_clock::now());
  StairAttitudeScope read_scope(reader,source(),measured());
  EXPECT_EQ(reader.getPlanningInflateOccupancy(body,0.),1);
}

TEST(StairContactMap,RigidChassisStillBlocksEvenQualifiedSupportAndLegacyHit) {
  GridMap map;GridMapTestAccess::configure(map);map.setOccupiedHit({.02,.02,.406});
  StairAttitudeScope scope(map,source(.406),measured());
  EXPECT_EQ(map.getPlanningInflateOccupancy({0.,0.,.50},0.),1);
  GridMap legacy;GridMapTestAccess::configure(legacy);legacy.setOccupancy({.02,.02,.276},1.);
  StairAttitudeScope old_scope(legacy,source(),measured());
  EXPECT_EQ(legacy.getPlanningInflateOccupancy({0.,0.,.50},0.),1);
}

TEST(StairContactMap,FlatScopeCannotInheritPreviousStairClassification) {
  GridMap map;GridMapTestAccess::configure(map);map.setOccupiedHit({.02,.02,.276});
  {StairAttitudeScope scope(map,source(),measured());
    ASSERT_EQ(map.getPlanningInflateOccupancy({0.,0.,.50},0.),0);
    {StairAttitudeScope flat(map,{},measured());
      EXPECT_EQ(map.getPlanningInflateOccupancy({0.,0.,.50},0.),1);}
    EXPECT_EQ(map.getPlanningInflateOccupancy({0.,0.,.50},0.),0);}
  EXPECT_EQ(map.getPlanningInflateOccupancy({0.,0.,.50},0.),1);
}

TEST(StairContactMap,RawRaysAtObservedCommonEdgePermitUpDownButNeverChangeFlatOrOccupancy) {
  for(const std::string kind:{"stair_up","stair_down"}) {
    GridMap map;GridMapTestAccess::configure(map);
    // Actual endpoints at the common front of two measured horizontal patches,
    // integrated by the production raycaster. No synthetic occupied/free grid.
    std::vector<Vec> endpoints;
    for(int y=-2;y<=2;++y)for(int z=0;z<4;++z)
      endpoints.emplace_back(0.,.02*y,.0602+.01*z);
    ASSERT_TRUE(GridMapTestAccess::rays(map,endpoints));
    const Vec body(0.,0.,.30);const auto raw=GridMapTestAccess::raw(map);
    StairRiserContactQuery riser(twoTreads(0.,kind),"odom",.05);
    for(const auto& endpoint:endpoints) {
      const auto bounds=GridMapTestAccess::bounds(map,endpoint);
      EXPECT_TRUE(riser.matchesObservedBounds(bounds.first,bounds.second))
        <<bounds.first.transpose()<<" -> "<<bounds.second.transpose();
    }
    ASSERT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
    {
      StairAttitudeScope scope(map,twoTreads(0.,kind),measured(body));
      EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),0)<<kind;
      EXPECT_GT(map.inspectInflateOccupancy(body,0.,true).unique_counts[1],0U);
      {StairAttitudeScope flat(map,{},measured(body));
        EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);}
      EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),0);
    }
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
    EXPECT_EQ(GridMapTestAccess::raw(map),raw);
  }
}

TEST(StairContactMap,ThinObjectInUnobservedTreadEdgeGapRemainsOccupiedAfterRawRays) {
  GridMap map;GridMapTestAccess::configure(map);
  const Vec lo(.0002,.0002,.0602),hi(.0102,.0102,.0992),body(0.,0.,.30);
  std::vector<Vec> endpoints;
  for(int corner=0;corner<8;++corner)
    endpoints.emplace_back(corner&4?hi.x():lo.x(),corner&2?hi.y():lo.y(),corner&1?hi.z():lo.z());
  ASSERT_TRUE(GridMapTestAccess::rays(map,endpoints));const auto raw=GridMapTestAccess::raw(map);
  ASSERT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
  for(const std::string kind:{"stair_up","stair_down"}) {
    StairAttitudeScope scope(map,twoTreads(.04,kind),measured(body));
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1)<<kind;
    EXPECT_GT(map.inspectInflateOccupancy(body,0.,true).unique_counts[1],0U);
  }
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);
}

TEST(StairContactMap,OriginalIndependentFacePermitsObservedRiserRaysForUpDownOnly) {
  for(const std::string kind:{"stair_up","stair_down"}) {
    GridMap map;GridMapTestAccess::configure(map);
    std::vector<Vec> endpoints;
    for(int y=-2;y<=2;++y)for(int z=0;z<4;++z)
      endpoints.emplace_back(0.,.02*y,.0602+.01*z);
    ASSERT_TRUE(GridMapTestAccess::rays(map,endpoints));
    const Vec body(0.,0.,.30);const auto raw=GridMapTestAccess::raw(map);
    {StairAttitudeScope absent(map,twoTreads(.04,kind),measured(body));
      EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);}
    {StairAttitudeScope scope(map,independentlyObservedRiser(kind),measured(body));
      EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),0);
      const auto proof=map.inspectInflateOccupancy(body,0.,true);
      EXPECT_GT(proof.unique_counts[1],0U); // contact is still observed occupancy
      GridMap reader;map.copyCollisionSnapshotTo(reader,1000000000,std::chrono::steady_clock::now());
      {StairAttitudeScope cloned(reader,independentlyObservedRiser(kind),measured(body));
        EXPECT_EQ(reader.getPlanningInflateOccupancy(body,0.),0);}
      {StairAttitudeScope flat(map,{},measured(body));
        EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);}}
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
    EXPECT_EQ(GridMapTestAccess::raw(map),raw);
  }
}

TEST(StairContactMap,NewThinLowAndWholeHeightPlatesCannotBorrowOriginalRiserThickness) {
  for(const double height:{.0992,.1692})for(const double offset:{.005,.01}) {
    GridMap map;GridMapTestAccess::configure(map);
    std::vector<Vec> endpoints;
    for(int y=-2;y<=2;++y)for(double z=.0502;z<=height;z+=.01)
      endpoints.emplace_back(offset,.02*y,z);
    ASSERT_TRUE(GridMapTestAccess::rays(map,endpoints));const auto raw=GridMapTestAccess::raw(map);
    const Vec body(0.,0.,.30);
    for(const std::string kind:{"stair_up","stair_down"}) {
      StairAttitudeScope scope(map,independentlyObservedRiser(kind),measured(body));
      EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1)<<height<<" "<<offset<<" "<<kind;
    }
    EXPECT_EQ(GridMapTestAccess::raw(map),raw);
  }
}

TEST(StairContactMap,ObservedTreadBelowRigidBottomDoesNotInheritWholeVoxelHeight) {
  for(const std::string kind:{"stair_up","stair_down"}) {
    GridMap map;GridMapTestAccess::configure(map);
    const Vec endpoint(.12,.02,.376),body(0.,0.,.50);
    ASSERT_TRUE(GridMapTestAccess::rays(map,{endpoint}));
    auto message=boundarySource(endpoint.z())->source;message.segment_kind=kind;
    auto support=std::make_shared<const PlanningSupport>(message);
    ASSERT_TRUE(support->stairClearanceAllowed(body));
    const auto bounds=GridMapTestAccess::bounds(map,endpoint);
    StairContactQuery contact(support,"odom",.05);
    ASSERT_TRUE(contact.matchesObservedBounds(bounds.first,bounds.second));
    // The 5 cm voxel reaches .40 m, but every recorded hit (outward-rounded)
    // remains below the actual rigid bottom at .38 m. XY stays a whole voxel.
    auto rigid=map.planningBodyEnvelope();rigid.below=.12;
    OrientedBoxVoxel3DQuery chassis(body,0.,0.,0.,.05,rigid);
    ASSERT_TRUE(chassis.intersects({2,0,7}));
    ASSERT_FALSE(chassis.intersectsVerticalBounds({2,0,7},bounds.first.z(),bounds.second.z()));
    const auto raw=GridMapTestAccess::raw(map);
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
    {
      StairAttitudeScope scope(map,support,measured(body));
      EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),0)<<kind;
      EXPECT_EQ(map.getInflateOccupancy(body,0.),0)<<kind;
      const auto diagnostic=map.inspectInflateOccupancy(body,0.,true);
      EXPECT_EQ(diagnostic.unique_counts[1],1U); // Contact is not erased occupancy.
      EXPECT_EQ(diagnostic.subvoxels.qualified_support_contacts,1U);
      GridMap reader;map.copyCollisionSnapshotTo(reader,1000000000,std::chrono::steady_clock::now());
      StairAttitudeScope cloned(reader,support,measured(body));
      EXPECT_EQ(reader.getPlanningInflateOccupancy(body,0.),0);
    }
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
    EXPECT_EQ(GridMapTestAccess::raw(map),raw);
  }
}

TEST(StairContactMap,ActualRigidIntrusionNeverBecomesTreadContact) {
  for(const double height:{.3803,.396})for(const std::string kind:{"stair_up","stair_down"}) {
    GridMap map;GridMapTestAccess::configure(map);
    const Vec endpoint(.12,.02,height),body(0.,0.,.50);
    ASSERT_TRUE(GridMapTestAccess::rays(map,{endpoint}));
    auto message=boundarySource(height)->source;message.segment_kind=kind;
    auto support=std::make_shared<const PlanningSupport>(message);
    ASSERT_TRUE(support->stairClearanceAllowed(body));
    const auto bounds=GridMapTestAccess::bounds(map,endpoint);
    ASSERT_TRUE(StairContactQuery(support,"odom",.05).matchesObservedBounds(bounds.first,bounds.second));
    const auto raw=GridMapTestAccess::raw(map);
    StairAttitudeScope scope(map,support,measured(body));
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1)<<height<<" "<<kind;
    EXPECT_EQ(map.inspectInflateOccupancy(body,0.,true).subvoxels.qualified_support_contacts,0U);
    EXPECT_EQ(GridMapTestAccess::raw(map),raw);
  }
}

TEST(StairContactMap,MixedRawHitAcrossRigidBottomCannotBeOverwrittenByTread) {
  for(const std::string kind:{"stair_up","stair_down"}) {
    GridMap map;GridMapTestAccess::configure(map);
    const Vec tread(.12,.02,.376),obstacle(.12,.02,.396),body(0.,0.,.50);
    ASSERT_TRUE(GridMapTestAccess::rays(map,{tread,obstacle}));
    auto message=boundarySource(tread.z())->source;message.segment_kind=kind;
    auto support=std::make_shared<const PlanningSupport>(message);
    ASSERT_TRUE(support->stairClearanceAllowed(body));
    StairAttitudeScope scope(map,support,measured(body));
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
    ASSERT_TRUE(GridMapTestAccess::rays(map,{tread}));
    const auto bounds=GridMapTestAccess::bounds(map,tread);
    EXPECT_LE(bounds.first.z(),tread.z());EXPECT_GE(bounds.second.z(),obstacle.z());
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
    GridMap reader;map.copyCollisionSnapshotTo(reader,1000000000,std::chrono::steady_clock::now());
    StairAttitudeScope cloned(reader,support,measured(body));
    EXPECT_EQ(reader.getPlanningInflateOccupancy(body,0.),1);
  }
}

TEST(StairContactMap,MissingOrCoarseOnlyBoundaryEvidenceRemainsOccupied) {
  for(const bool missing_bounds:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map);const Vec hit(.12,.02,.376),body(0.,0.,.50);
    ASSERT_TRUE(GridMapTestAccess::rays(map,{hit}));
    if(missing_bounds)GridMapTestAccess::discardBounds(map);
    else GridMapTestAccess::markCoarseOnly(map,hit);
    const auto raw=GridMapTestAccess::raw(map);
    auto support=boundarySource(hit.z());ASSERT_TRUE(support->stairClearanceAllowed(body));
    StairAttitudeScope scope(map,support,measured(body));
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1)<<missing_bounds;
    EXPECT_EQ(GridMapTestAccess::raw(map),raw);
  }
}

TEST(StairContactMap,BoundaryHitNeedsStairScopeFrameAndIndependentQualification) {
  GridMap map;GridMapTestAccess::configure(map);const Vec hit(.12,.02,.376),body(0.,0.,.50);
  ASSERT_TRUE(GridMapTestAccess::rays(map,{hit}));
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
  for(const std::string variant:{"unqualified","wrong_frame","flat"}) {
    auto message=boundarySource(hit.z(),variant!="unqualified")->source;
    if(variant=="wrong_frame")message.frame_id="map";
    if(variant=="flat"){message.segment_kind="floor";message.required_mode="normal";}
    StairAttitudeScope scope(map,std::make_shared<const PlanningSupport>(message),measured(body));
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1)<<variant;
  }
}

TEST(StairContactMap,RigidCheckKeepsWholeCoarseHorizontalFootprint) {
  GridMap map;GridMapTestAccess::configure(map);const Vec hit(.12,.199,.396),body(0.,0.,.50);
  ASSERT_TRUE(GridMapTestAccess::rays(map,{hit}));
  auto message=boundarySource(hit.z())->source;
  for(std::size_t i=0;i<121;++i)message.support_ground_xyz[i].y+=.20;
  auto support=std::make_shared<const PlanningSupport>(message);
  ASSERT_TRUE(support->stairClearanceAllowed(body));
  const auto bounds=GridMapTestAccess::bounds(map,hit);
  ASSERT_TRUE(StairContactQuery(support,"odom",.05).matchesObservedBounds(bounds.first,bounds.second));
  // Endpoint y=.199 is outside the .175 m stair half-width, but its [ .15,.20 ]
  // coarse column overlaps. Refining Z must not also shrink this XY footprint.
  StairAttitudeScope scope(map,support,measured(body));
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
}

TEST(StairContactMap,ExplicitSubvoxelOptOutRetainsCoarseRigidCheck) {
  GridMap map;GridMapTestAccess::configure(map);const Vec hit(.12,.02,.376),body(0.,0.,.50);
  ASSERT_TRUE(GridMapTestAccess::rays(map,{hit}));
  GridMapTestAccess::disableStairSubvoxel(map);
  auto support=boundarySource(hit.z());ASSERT_TRUE(support->stairClearanceAllowed(body));
  StairAttitudeScope scope(map,support,measured(body));
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
}

TEST(StairContactMap,RigidVerticalRefinementUsesSameRotatedSatAsFullBody) {
  for(const double height:{.376,.396})for(const double yaw:{0.,1.2,3.141592653589793})
      for(const double pitch:{-.01,.01})for(const double roll:{-.01,.01}) {
    GridMap map;GridMapTestAccess::configure(map);const Vec hit(.12,.02,height),body(0.,0.,.50);
    ASSERT_TRUE(GridMapTestAccess::rays(map,{hit}));
    auto support=boundarySource(hit.z());ASSERT_TRUE(support->stairClearanceAllowed(body));
    StairAttitudeScope scope(map,support,measured(body));
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,yaw,pitch,roll),height<.38?0:1)
        <<height<<" "<<yaw<<" "<<pitch<<" "<<roll;
  }
}

TEST(StairContactMap,StrictObservedFreePolicyRetainsCoarseRigidCheck) {
  for(const double pitch:{0.,-.02,.02}) {
    GridMap map;GridMapTestAccess::configure(map);const Vec hit(.12,.02,.376),body(0.,0.,.50);
    ASSERT_TRUE(GridMapTestAccess::rays(map,{hit}));
    GridMapTestAccess::strictObservedFree(map);
    auto support=boundarySource(hit.z());ASSERT_TRUE(support->stairClearanceAllowed(body));
    StairAttitudeScope scope(map,support,measured(body));
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.,pitch,0.),1)<<pitch;
    EXPECT_EQ(map.getInflateOccupancy(body,0.,pitch,0.),1)<<pitch;
  }
}

TEST(StairContactMap,TiltedRigidBottomUsesAllHitSeparationBeforeRejectingQualifiedContact) {
  // A known tread/step contact below the rotated rigid chassis must not become
  // a chassis hit merely because its coarse XY column extends uphill through
  // that bottom plane. Keep the complete lower-envelope contact classification
  // and all occupied cells: only the already-proven separating plane is exact.
  const Vec body(0.,0.,.5);
  std::size_t exercised=0;
  for(const double pitch:{-.55,-.20,.20,.55})for(const double yaw:{0.,.7,2.4})
      for(const double roll:{-.10,0.,.10}) {
    SCOPED_TRACE(::testing::Message()<<"yaw="<<yaw<<" pitch="<<pitch<<" roll="<<roll);
    const Eigen::Quaterniond orientation(Eigen::AngleAxisd(yaw,Vec::UnitZ())*
      Eigen::AngleAxisd(pitch,Vec::UnitY())*Eigen::AngleAxisd(roll,Vec::UnitX()));
    const Vec up=orientation*Vec::UnitZ();
    GridMap map;GridMapTestAccess::configure(map);
    auto shape=map.planningBodyEnvelope();shape.below=.12;
    OrientedBoxVoxel3DQuery rigid(body,yaw,pitch,roll,.05,shape);
    std::optional<Vec> selected;
    for(const double x:{-.045,-.02,.005,.02,.045})for(const double y:{-.045,-.02,.005,.02,.045}) {
      const Vec point(x,y,body.z()+(-.12-up.x()*x-up.y()*y)/up.z()-.004);
      const Eigen::Vector3i cell=(point/.05).array().floor().cast<int>();
      if(rigid.intersectsVerticalBounds(cell,point.z(),point.z()))selected=point;
    }
    ASSERT_TRUE(selected);const Vec hit=*selected;
    ASSERT_TRUE(GridMapTestAccess::rays(map,{hit}));
    auto message=source(hit.z())->source;
    for(auto& p:message.support_ground_xyz){p.x+=hit.x();p.y+=hit.y();}
    const auto ground=source(0.);
    for(const auto& p:ground->source.support_ground_xyz) {
      message.stair_contact_indices.push_back(message.support_ground_xyz.size());
      message.support_ground_xyz.push_back(p);
    }
    const auto support=std::make_shared<const PlanningSupport>(message);
    ASSERT_TRUE(support->valid);ASSERT_TRUE(support->stairClearanceAllowed(body));
    const auto bounds=GridMapTestAccess::bounds(map,hit);
    ASSERT_TRUE(rigid.allHitBoundsBelowBody(bounds.first,bounds.second));
    ASSERT_TRUE(StairContactQuery(support,"odom",.05).matchesObservedBounds(bounds.first,bounds.second));
    auto acquired=measured(body);acquired.orientation=orientation;
    const auto raw=GridMapTestAccess::raw(map);
    {
      StairAttitudeScope scope(map,support,acquired);
      EXPECT_EQ(map.getPlanningInflateOccupancy(body,yaw,pitch,roll),0);
      EXPECT_EQ(map.getInflateOccupancy(body,yaw,pitch,roll),0);
      const auto detail=map.inspectInflateOccupancy(body,yaw,pitch,roll,true);
      EXPECT_GT(detail.subvoxels.qualified_support_contacts,0U);
      EXPECT_EQ(GridMapTestAccess::raw(map),raw);
      GridMap reader;map.copyCollisionSnapshotTo(reader,102000000000LL,std::chrono::steady_clock::now());
      {StairAttitudeScope own(reader,support,acquired);
        EXPECT_EQ(reader.getPlanningInflateOccupancy(body,yaw,pitch,roll),0);}
      // The same point without its contact source remains an obstacle; the
      // rigid-bottom separation is not a generic clearance or near-body erase.
      {StairAttitudeScope flat(map,{},acquired);
        EXPECT_EQ(map.getPlanningInflateOccupancy(body,yaw,pitch,roll),1);}
      auto unqualified=message;unqualified.stair_contact_indices.clear();
      {StairAttitudeScope no_contact(map,std::make_shared<const PlanningSupport>(unqualified),acquired);
        EXPECT_EQ(map.getPlanningInflateOccupancy(body,yaw,pitch,roll),1);}
      // A new return above the true rigid bottom, including same-cell mixed
      // history, must remain blocking even if an older tread match exists.
      const Vec intrusion=hit+Vec(0.,0.,.025);
      ASSERT_TRUE(GridMapTestAccess::rays(map,{intrusion}));
      EXPECT_EQ(map.getPlanningInflateOccupancy(body,yaw,pitch,roll),1);
      EXPECT_EQ(map.getInflateOccupancy(body,yaw,pitch,roll),1);
    }
    ++exercised;
  }
  EXPECT_EQ(exercised,36U);
}

TEST(StairContactMap,TiltedRigidContactReadersCannotShareNewHitsOrQueryCaches) {
  const Vec body(0.,0.,.5),hit(.049,.02,.385);
  GridMap writer;GridMapTestAccess::configure(writer);
  ASSERT_TRUE(GridMapTestAccess::rays(writer,{hit}));
  auto message=source(hit.z())->source;
  for(auto& p:message.support_ground_xyz)p.x+=.05;
  const auto ground=source(0.);
  for(const auto& p:ground->source.support_ground_xyz) {
    message.stair_contact_indices.push_back(message.support_ground_xyz.size());
    message.support_ground_xyz.push_back(p);
  }
  const auto support=std::make_shared<const PlanningSupport>(message);
  ASSERT_TRUE(support->stairClearanceAllowed(body));
  auto acquired=measured(body);acquired.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(-.5,Vec::UnitY()));
  GridMap old_reader,new_reader;
  writer.copyCollisionSnapshotTo(old_reader,102000000000LL,std::chrono::steady_clock::now());
  ASSERT_TRUE(GridMapTestAccess::rays(writer,{hit+Vec(0.,0.,.025)}));
  writer.copyCollisionSnapshotTo(new_reader,103000000000LL,std::chrono::steady_clock::now());
  const auto query=[&](GridMap& reader,int expected) {
    StairAttitudeScope own(reader,support,acquired);
    std::size_t wrong=0;
    for(int i=0;i<100;++i)wrong+=reader.getPlanningInflateOccupancy(body,0.,-.5,0.)!=expected;
    return wrong;
  };
  auto previous=std::async(std::launch::async,[&]{return query(old_reader,0);});
  auto current=std::async(std::launch::async,[&]{return query(new_reader,1);});
  EXPECT_EQ(previous.get(),0U);EXPECT_EQ(current.get(),0U);
  // No provider/cached exception can escape either reader's scoped lease.
  EXPECT_EQ(old_reader.getPlanningInflateOccupancy(body,0.,-.5,0.),1);
  EXPECT_EQ(new_reader.getPlanningInflateOccupancy(body,0.,-.5,0.),1);
}

TEST(StairContactMap,TiltedContactDoesNotRewriteUnknownOrWaiveStrictUnknownPolicy) {
  GridMap map;GridMapTestAccess::configure(map);
  const Vec body(0.,0.,.5);const auto raw=GridMapTestAccess::raw(map);
  auto acquired=measured(body);acquired.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(-.5,Vec::UnitY()));
  GridMapTestAccess::strictObservedFree(map);
  {StairAttitudeScope scope(map,boundarySource(.385),acquired);
    EXPECT_EQ(map.getInflateOccupancy(body,0.,-.5,0.),2);
    const auto evidence=map.inspectInflateOccupancy(body,0.,-.5,0.,true);
    EXPECT_GT(evidence.unique_counts[2],0U);
    EXPECT_EQ(evidence.subvoxels.qualified_support_contacts,0U);
    EXPECT_EQ(GridMapTestAccess::raw(map),raw);
  }
  EXPECT_EQ(map.getInflateOccupancy(body,0.,-.5,0.),2);
}
