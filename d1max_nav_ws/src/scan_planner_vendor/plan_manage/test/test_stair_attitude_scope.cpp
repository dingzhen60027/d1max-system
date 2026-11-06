#include <gtest/gtest.h>
#include <plan_manage/stair_attitude_scope.hpp>
#include <d1max_planning_interfaces/msg/reference_path.hpp>

// Real production map, exclusively owned memory fixture. No ROS node/graph,
// alternate collision implementation, synthetic FREE result or SDK activity.
struct GridMapTestAccess {
  static void configure(GridMap& map,double stair_clearance=-1.) {
    auto& p=map.mp_;auto& d=map.md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={80,64,80};p.map_origin_idx_.setZero();
    p.collision_model_="oriented_box";p.box_length_=1.03;p.box_width_=.4;
    p.box_nominal_length_=.93;p.box_nominal_width_=.30;p.stair_planar_clearance_=stair_clearance;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    // The production map also maintains its upstream visualization kernel;
    // memory-only fixtures must initialize those legacy dimensions explicitly.
    p.double_cylinder_radius_=.2;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.25;
    p.map_sliding_en_=false;
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.planning_collision_policy_="official";p.require_observed_free_=false;p.frame_id_="odom";
    map.updateMapBoundaryFromIndex();
    const auto count=80U*64U*80U;d.occupancy_buffer_.assign(count,-1.);
    d.occupancy_buffer_inflate_.assign(count,0);d.occupancy_buffer_inflate_cnt_.assign(count,0);
    d.planning_occupancy_buffer_inflate_.assign(count,0);d.planning_occupancy_buffer_inflate_cnt_.assign(count,0);
    map.rebuildInflationOffsets();
  }
  static bool hasProvider(const GridMap& map){return bool(map.query_attitude_provider_);}
  static auto attitude(const GridMap& map,const Eigen::Vector3d& p,double yaw) {
    return map.query_attitude_provider_?map.query_attitude_provider_(p,yaw):std::optional<Eigen::Vector2d>{};
  }
  static auto raw(const GridMap& map){return map.md_.occupancy_buffer_;}
  static auto cacheSize(const GridMap& map){return map.planning_box_cache_.size();}
  static void subvoxel(GridMap& map){map.mp_.stair_subvoxel_collision_=true;}
  static bool stairRegion(const GridMap& map,const Eigen::Vector3d& point) {
    return map.query_stair_clearance_provider_&&map.query_stair_clearance_provider_(point);
  }
};

namespace {
using namespace scan_planner;
using Vector=Eigen::Vector3d;
std::shared_ptr<const PlanningSupport> support(bool stair=true) {
  d1max_planning_interfaces::msg::SupportReference s;s.support_reference_id="surface";
  s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');
  s.frame_id="odom";s.floor_id=stair?"stair_lower":"floor1";
  s.segment_kind=stair?"stair_up":"floor";s.required_mode=stair?"stair":"general";
  s.reason=stair?"original_stair_surface_returns_associated":"original_floor_support_classified";
  s.verified=true;s.support_xy_radius_m=.2;s.body_reference_height_m=.55;
  s.max_support_step_m=stair?0.:.05;s.max_support_slope_rad=stair?0.:.15;
  for(int x=-30;x<=36;++x)for(int y=-16;y<=16;++y) {
    geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;p.z=stair?.5*p.x:0.;s.support_ground_xyz.push_back(p);
  }
  return std::make_shared<const PlanningSupport>(s);
}
MeasuredBodyPose measured(double pitch=0.,double roll=0.) {
  return {{0.,0.,.55},Eigen::Quaterniond(Eigen::AngleAxisd(pitch,Vector::UnitY())*
      Eigen::AngleAxisd(roll,Vector::UnitX())),100.,"odom"};
}

std::shared_ptr<const PlanningSupport> mixedClearanceSupport() {
  auto s=support()->source;s.support_ground_xyz.clear();s.regions.clear();
  s.version.segment_id="entry";s.floor_id="floor1";s.segment_kind="floor";s.required_mode="general";
  s.reason="original_transition_surface_returns_associated";
  const auto add=[&](const std::string& id,const std::string& floor,const std::string& kind,
      const std::string& mode,int first_x,int last_x,double z,double slope) {
    d1max_planning_interfaces::msg::SupportRegion region;
    region.segment_id=id;region.floor_id=floor;region.segment_kind=kind;region.required_mode=mode;
    region.begin_index=s.support_ground_xyz.size();
    for(int x=first_x;x<=last_x;++x)for(int y=-6;y<=6;++y) {
      geometry_msgs::msg::Point p;p.x=x*.05;p.y=y*.05;p.z=z+slope*p.x;
      s.support_ground_xyz.push_back(p);
    }
    region.end_index=s.support_ground_xyz.size();s.regions.push_back(region);
  };
  add("entry","floor1","floor","general",-30,-10,0.,0.);
  add("flight","stair_lower","stair_up","stair",-9,9,.25,.5);
  add("platform","stair_upper","landing","stair",10,30,.5,0.);
  // Same XY as the flight but a different floor. Its literal Z and region
  // ownership must win over the transition window's overall stair flag.
  add("upper_floor","floor2","floor","general",-9,9,.9,0.);
  return std::make_shared<const PlanningSupport>(s);
}
}

TEST(StairAttitudeScope,FlatAndMissingSupportDoNotInstallProviderOrFlushFlatCache) {
  GridMap map;GridMapTestAccess::configure(map);
  EXPECT_EQ(map.getPlanningInflateOccupancy({0.,0.,.55},0.),0);
  const auto cache=GridMapTestAccess::cacheSize(map);
  EXPECT_DOUBLE_EQ(map.queryBodyExtent(),std::hypot(1.03*.5,.4*.5));
  {
    StairAttitudeScope flat(map,support(false),measured());
    EXPECT_FALSE(GridMapTestAccess::hasProvider(map));EXPECT_EQ(GridMapTestAccess::cacheSize(map),cache);
  }
  EXPECT_FALSE(GridMapTestAccess::hasProvider(map));EXPECT_EQ(GridMapTestAccess::cacheSize(map),cache);
  {StairAttitudeScope absent(map,{},measured());EXPECT_FALSE(GridMapTestAccess::hasProvider(map));}
  EXPECT_EQ(GridMapTestAccess::cacheSize(map),cache);
}

TEST(StairAttitudeScope,SubvoxelBindingIsIndependentOfExtraXYClearanceAndCannotLeakToFloor) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::subvoxel(map);
  ASSERT_FALSE(map.stairClearanceConfigured());ASSERT_TRUE(map.stairSubvoxelConfigured());
  const auto mixed=mixedClearanceSupport();ASSERT_TRUE(mixed->valid);
  const Vector stair(0.,0.,.80),floor(-.9,0.,.55),upper(0.,0.,1.45);
  EXPECT_FALSE(GridMapTestAccess::stairRegion(map,stair));
  {
    StairAttitudeScope outer(map,mixed,measured());
    EXPECT_TRUE(GridMapTestAccess::stairRegion(map,stair));
    EXPECT_FALSE(GridMapTestAccess::stairRegion(map,floor));
    EXPECT_FALSE(GridMapTestAccess::stairRegion(map,upper));
    // Selecting finer occupied geometry does not alter nominal/padded body.
    EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(stair).width,.4);
    {StairAttitudeScope flat(map,support(false),measured());
      EXPECT_FALSE(GridMapTestAccess::stairRegion(map,stair));}
    EXPECT_TRUE(GridMapTestAccess::stairRegion(map,stair));
    auto foreign=mixed->source;foreign.frame_id="other";
    {StairAttitudeScope rejected(map,std::make_shared<const PlanningSupport>(foreign),measured());
      EXPECT_FALSE(GridMapTestAccess::stairRegion(map,stair));}
    EXPECT_TRUE(GridMapTestAccess::stairRegion(map,stair));
  }
  EXPECT_FALSE(GridMapTestAccess::stairRegion(map,stair));
}
TEST(StairAttitudeScope,ExplicitPortalRegionsUseSpatialSupportWithoutRelabellingFloor) {
  auto s=support()->source;
  s.version.segment_id="entry";s.floor_id="floor1";s.segment_kind="floor";s.required_mode="general";
  s.reason="original_transition_surface_returns_associated";
  d1max_planning_interfaces::msg::SupportRegion first,second;
  first.segment_id="entry";first.floor_id="floor1";first.segment_kind="floor";first.required_mode="general";
  first.begin_index=0;first.end_index=s.support_ground_xyz.size()/2;
  second.segment_id="flight";second.floor_id="stair_lower";second.segment_kind="stair_up";second.required_mode="stair";
  second.begin_index=first.end_index;second.end_index=s.support_ground_xyz.size();s.regions={first,second};
  auto mixed=std::make_shared<const PlanningSupport>(s);
  ASSERT_TRUE(mixed->valid);EXPECT_TRUE(mixed->isStair());EXPECT_EQ(mixed->source.required_mode,"general");
  GridMap map;GridMapTestAccess::configure(map);
  {StairAttitudeScope scope(map,mixed,measured(-.1));EXPECT_TRUE(GridMapTestAccess::hasProvider(map));}
  {StairAttitudeScope flat(map,support(false),measured());EXPECT_FALSE(GridMapTestAccess::hasProvider(map));}
  for(int fault=0;fault<5;++fault) {
    auto bad=s;
    if(fault==0)bad.regions[1].begin_index++;
    if(fault==1)bad.regions[1].end_index--;
    if(fault==2)bad.regions[0].segment_id="foreign";
    if(fault==3)bad.regions[1].required_mode="general";
    if(fault==4)bad.max_support_step_m=.05;
    EXPECT_FALSE(PlanningSupport(bad).valid)<<fault;
  }
}
TEST(StairAttitudeScope,NestedFlatAndStairBindingsRestoreExactPreviousProvider) {
  GridMap map;GridMapTestAccess::configure(map);const auto surface=support();
  const Vector p{0.,0.,.55};
  {
    StairAttitudeScope first(map,surface,measured(-.15,.04));
    EXPECT_NEAR(map.queryBodyExtent(),std::sqrt(.515*.515+.2*.2+.25*.25),1e-12);
    ASSERT_TRUE(GridMapTestAccess::hasProvider(map));auto a=GridMapTestAccess::attitude(map,p,0.);
    ASSERT_TRUE(a);EXPECT_NEAR(a->x(),-.15,1e-12);EXPECT_NEAR(a->y(),.04,1e-12);
    {StairAttitudeScope flat(map,support(false),measured());
      EXPECT_FALSE(GridMapTestAccess::hasProvider(map));
      EXPECT_DOUBLE_EQ(map.queryBodyExtent(),std::hypot(.515,.2));}
    a=GridMapTestAccess::attitude(map,p,0.);ASSERT_TRUE(a);EXPECT_NEAR(a->x(),-.15,1e-12);
    {
      StairAttitudeScope second(map,surface,measured(-.30,-.03));
      a=GridMapTestAccess::attitude(map,p,0.);ASSERT_TRUE(a);EXPECT_NEAR(a->x(),-.30,1e-12);
    }
    a=GridMapTestAccess::attitude(map,p,0.);ASSERT_TRUE(a);EXPECT_NEAR(a->x(),-.15,1e-12);
  }
  EXPECT_FALSE(GridMapTestAccess::hasProvider(map));
  EXPECT_DOUBLE_EQ(map.queryBodyExtent(),std::hypot(.515,.2));
}
TEST(StairAttitudeScope,AcquiredAttitudeIsExactAtBodyAndTerrainNormalControlsFuture) {
  StairAttitudeQuery query(support(),measured(-.10,.03));
  const auto actual=query({0.,0.,.55},0.);ASSERT_TRUE(actual);
  EXPECT_NEAR(actual->x(),-.10,1e-12);EXPECT_NEAR(actual->y(),.03,1e-12);
  const auto future=query({.8,0.,.95},0.);ASSERT_TRUE(future);
  EXPECT_NEAR(future->x(),-std::atan(.5),1e-10);EXPECT_NEAR(future->y(),0.,1e-10);
  const auto blend=query({.2,0.,.65},0.);ASSERT_TRUE(blend);
  EXPECT_LT(blend->x(),actual->x());EXPECT_GT(blend->x(),future->x());
  // A different odometry frame cannot supply measured attitude to this map.
  auto foreign=measured(-.10,.03);foreign.frame="other";StairAttitudeQuery isolated(support(),foreign);
  const auto fit=isolated({0.,0.,.55},0.);ASSERT_TRUE(fit);EXPECT_NEAR(fit->x(),-std::atan(.5),1e-10);
}
TEST(StairAttitudeScope,DelayedSupportMustNotProduceAnUprightCollisionForAStairReference) {
  GridMap map;GridMapTestAccess::configure(map);
  const auto terrain=support();const Vector body(0.,0.,.55);
  d1max_planning_interfaces::msg::ReferencePath reference;
  reference.schema_version=2;reference.required_mode="stair";
  const std::shared_ptr<const PlanningSupport> absent;
  // Same native map/input: before the support arrival, the old fallback would
  // report this tread-height cell as occupied using an upright torso.
  map.setOccupied({.425,.025,.375});const auto before=GridMapTestAccess::raw(map);
  EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1);
  EXPECT_FALSE(stairReferenceGeometryReady(reference,terrain->source.version,absent,"odom"));
  ASSERT_TRUE(stairReferenceGeometryReady(reference,terrain->source.version,terrain,"odom"));
  {
    StairAttitudeScope scope(map,terrain,measured(-std::acos(-1.)/6.));
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),0);
    EXPECT_EQ(GridMapTestAccess::raw(map),before); // No cell was cleared.
    map.setOccupied(body);
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1); // A real body intersection remains blocked.
  }
  EXPECT_FALSE(GridMapTestAccess::hasProvider(map));
}
TEST(StairAttitudeScope,CachePredictionIsQueryOrderIndependentAndDoesNotInventMissingSupport) {
  MeasuredBodyPose absent;absent.frame="absent";StairAttitudeQuery a(support(),absent),b(support(),absent);
  const Vector first{.201,.002,.651},second{.225,.024,.674};
  const auto a1=a(first,.4);const auto a2=a(second,.4);
  const auto b2=b(second,.4);const auto b1=b(first,.4);
  ASSERT_TRUE(a1&&a2&&b1&&b2);EXPECT_TRUE(a1->isApprox(*b1));EXPECT_TRUE(a2->isApprox(*b2));
  const auto missing=a({.2,0.,3.},0.);ASSERT_TRUE(missing);EXPECT_TRUE(missing->isZero());
  EXPECT_FALSE(a({NAN,0.,.55},0.));
}
TEST(StairAttitudeScope,FailedCacheCentreDoesNotRejectSupportedActualQuery) {
  const auto surface=support();
  const Vector body(.249,.002,.801),centre(.225,.025,.825);
  // Both are literal queries against the same unchanged source plane. Only
  // quantising the query moves its residual beyond the existing 15 cm limit.
  const auto actual=surface->terrainFit(body),rounded=surface->terrainFit(centre);
  ASSERT_TRUE(actual.valid);ASSERT_FALSE(rounded.valid);
  EXPECT_LT(std::abs(actual.center_residual),.15);
  EXPECT_GT(std::abs(rounded.center_residual),.15);
  MeasuredBodyPose absent;absent.frame="absent";
  StairAttitudeQuery query(surface,absent);
  const auto attitude=query(body,0.);ASSERT_TRUE(attitude);
  EXPECT_NEAR(attitude->x(),-std::atan(.5),1e-10);
  EXPECT_NEAR(attitude->y(),0.,1e-10);
}
TEST(StairAttitudeScope,ExactFallbackIsNotSharedWithAnUnsupportedNeighbourInTheSameCell) {
  const auto surface=support();
  const Vector valid(.249,.002,.801),invalid(.201,.024,.849);
  ASSERT_TRUE(surface->terrainFit(valid).valid);ASSERT_FALSE(surface->terrainFit(invalid).valid);
  ASSERT_TRUE((valid/.05).array().floor().matrix().isApprox((invalid/.05).array().floor().matrix()));
  MeasuredBodyPose absent;absent.frame="absent";
  StairAttitudeQuery forward(surface,absent),reverse(surface,absent);
  const auto a=forward(valid,.4),b=forward(invalid,.4);
  const auto d=reverse(invalid,.4),c=reverse(valid,.4);
  ASSERT_TRUE(a&&b&&c&&d);
  EXPECT_GT(a->norm(),.4);EXPECT_TRUE(b->isZero());
  EXPECT_TRUE((*a).array().isApprox((*c).array(),0.));
  EXPECT_TRUE((*b).array().isApprox((*d).array(),0.));
}
TEST(StairAttitudeScope,ExactFallbackKeepsAcquiredAttitudeAndTheExistingMeasuredBlend) {
  const auto surface=support();const Vector body(.249,.002,.801);
  auto acquired=measured(-.10,.03);acquired.position=body;
  StairAttitudeQuery at_body(surface,acquired);
  const auto actual=at_body(body,0.);ASSERT_TRUE(actual);
  EXPECT_NEAR(actual->x(),-.10,1e-12);EXPECT_NEAR(actual->y(),.03,1e-12);
  acquired.position=body-Vector(.2,0.,0.);
  StairAttitudeQuery partial(surface,acquired);
  const auto blended=partial(body,0.);ASSERT_TRUE(blended);
  const Vector up=(.4*surface->terrainFit(body).normal+
      .6*(acquired.orientation*Vector::UnitZ())).normalized();
  const Eigen::Vector2d expected(std::atan2(up.x(),up.z()),std::atan2(-up.y(),std::hypot(up.x(),up.z())));
  EXPECT_TRUE(blended->isApprox(expected,1e-12));
}
TEST(StairAttitudeScope,ExactFallbackRetainsOccupiedBoxRailAndRoofWithUnchangedVoxels) {
  const auto surface=support();const Vector body(.249,.002,.801);
  const Eigen::Matrix3d rotation=Eigen::AngleAxisd(-std::atan(.5),Vector::UnitY()).toRotationMatrix();
  MeasuredBodyPose absent;absent.frame="absent";
  for(const Vector& local:{Vector(.30,0.,.23),Vector(0.,.19,0.),Vector(0.,0.,0.),Vector(.35,0.,-.23)}) {
    GridMap map;GridMapTestAccess::configure(map);map.setOccupancy(body+rotation*local,1.);
    const auto before=GridMapTestAccess::raw(map);const auto revision=map.occupancyRevision();
    {StairAttitudeScope scope(map,surface,absent);
      const auto attitude=GridMapTestAccess::attitude(map,body,0.);ASSERT_TRUE(attitude);
      EXPECT_NEAR(attitude->x(),-std::atan(.5),1e-10);
      EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1)<<local.transpose();}
    EXPECT_EQ(GridMapTestAccess::raw(map),before);EXPECT_EQ(map.occupancyRevision(),revision);
  }
}
TEST(StairAttitudeScope,OriginalRoofRailAndBoxVoxelsRemainOccupiedWithUnchangedFullEnvelope) {
  const double pitch=-std::atan(.5);const Eigen::Matrix3d rotation=Eigen::AngleAxisd(pitch,Vector::UnitY()).toRotationMatrix();
  const Vector body{0.,0.,.55};
  for(const Vector& local:{Vector(.30,0.,.23),Vector(0.,.19,0.),Vector(0.,0.,0.),Vector(.35,0.,-.23)}) {
    GridMap map;GridMapTestAccess::configure(map);map.setOccupancy(body+rotation*local,1.);
    const auto before=GridMapTestAccess::raw(map);const auto revision=map.occupancyRevision();
    {
      StairAttitudeScope scope(map,support(),measured(pitch));
      EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),1)<<local.transpose();
      EXPECT_DOUBLE_EQ(map.planningBodyEnvelope().length,1.03);
      EXPECT_DOUBLE_EQ(map.planningBodyEnvelope().width,.4);
      EXPECT_DOUBLE_EQ(map.planningBodyEnvelope().below,.25);
      EXPECT_DOUBLE_EQ(map.planningBodyEnvelope().above,.25);
    }
    EXPECT_EQ(GridMapTestAccess::raw(map),before);EXPECT_EQ(map.occupancyRevision(),revision);
  }
}
TEST(StairAttitudeScope,CollisionSnapshotsNeverCopyOrRetainAnotherReadersAttitude) {
  GridMap source,reader;GridMapTestAccess::configure(source);GridMapTestAccess::configure(reader);
  source.setQueryAttitudeProvider(stairAttitudeProvider(support(),measured(-.15)));
  reader.setQueryAttitudeProvider(stairAttitudeProvider(support(),measured(-.30)));
  ASSERT_TRUE(GridMapTestAccess::hasProvider(source));ASSERT_TRUE(GridMapTestAccess::hasProvider(reader));
  source.copyCollisionSnapshotTo(reader,100000000000LL,std::chrono::steady_clock::now());
  EXPECT_TRUE(GridMapTestAccess::hasProvider(source));EXPECT_FALSE(GridMapTestAccess::hasProvider(reader));
  EXPECT_EQ(GridMapTestAccess::raw(source),GridMapTestAccess::raw(reader));
  {
    StairAttitudeScope own(reader,support(),measured(-.25));
    const auto own_attitude=GridMapTestAccess::attitude(reader,{0.,0.,.55},0.);ASSERT_TRUE(own_attitude);
    EXPECT_NEAR(own_attitude->x(),-.25,1e-12);
    const auto original=GridMapTestAccess::attitude(source,{0.,0.,.55},0.);ASSERT_TRUE(original);
    EXPECT_NEAR(original->x(),-.15,1e-12);
  }
  EXPECT_FALSE(GridMapTestAccess::hasProvider(reader));
}

TEST(StairAttitudeScope,ClearanceSelectsLiteralFloorFlightLandingAndSameXYDifferentLevel) {
  const auto mixed=mixedClearanceSupport();ASSERT_TRUE(mixed->valid);ASSERT_TRUE(mixed->isStair());
  GridMap map;GridMapTestAccess::configure(map,.025);
  const Vector floor(-1.,0.,.55),flight(0.,0.,.8),landing(1.,0.,1.05),upper(0.,0.,1.45);
  for(const auto& p:{floor,flight,landing,upper})map.setOccupied(p+Vector(.025,.225,.025));
  const auto raw=GridMapTestAccess::raw(map);
  {
    StairAttitudeScope scope(map,mixed,measured());
    for(const auto& p:{floor,upper}) {
      EXPECT_FALSE(mixed->stairClearanceAllowed(p));
      EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(p).length,1.03);
      EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(p).width,.40);
      EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),1);
    }
    for(const auto& p:{flight,landing}) {
      EXPECT_TRUE(mixed->stairClearanceAllowed(p));
      EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(p).length,.98);
      EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(p).width,.35);
      EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(p,false).width,.35);
      EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(p).below,.25);
      EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(p).above,.25);
      EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),0);
    }
    EXPECT_DOUBLE_EQ(map.queryBodyEnvelope({0.,0.,1.1}).width,.40); // Neither level within support radius.
    EXPECT_DOUBLE_EQ(map.queryBodyEnvelope({4.,0.,.55}).width,.40); // No source support, no extrapolation.
  }
  for(const auto& p:{floor,flight,landing,upper})EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(p).width,.40);
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);
}

TEST(StairAttitudeScope,ClearanceMissingForeignAndFlatNestedScopesRestoreWithoutSnapshotLeak) {
  const Vector body(0.,0.,.55);const auto stairs=support();ASSERT_TRUE(stairs->valid);
  auto foreign_message=stairs->source;foreign_message.frame_id="other";
  const auto foreign=std::make_shared<const PlanningSupport>(foreign_message);ASSERT_TRUE(foreign->valid);
  GridMap map;GridMapTestAccess::configure(map,.025);
  {
    StairAttitudeScope outer(map,stairs,measured());
    EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(body).width,.35);
    for(const auto& inner:{std::shared_ptr<const PlanningSupport>{},support(false),foreign}) {
      {
        StairAttitudeScope scope(map,inner,measured());
        EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(body).width,.40);
        EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(body,false).width,.40);
      }
      EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(body).width,.35);
    }
    GridMap snapshot;map.copyCollisionSnapshotTo(snapshot,100000000000LL,std::chrono::steady_clock::now());
    EXPECT_DOUBLE_EQ(snapshot.queryBodyEnvelope(body).width,.40);
    EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(body).width,.35);
    EXPECT_EQ(GridMapTestAccess::raw(map),GridMapTestAccess::raw(snapshot));
  }
  EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(body).width,.40);
}

TEST(StairAttitudeScope,ClearanceAmbiguousSupportTieRetainsFloorMargin) {
  auto message=mixedClearanceSupport()->source;
  d1max_planning_interfaces::msg::SupportRegion tied;
  tied.segment_id="floor_boundary";tied.floor_id="floor2";tied.segment_kind="landing";
  tied.required_mode="general";tied.begin_index=message.support_ground_xyz.size();
  geometry_msgs::msg::Point p;p.x=0.;p.y=0.;p.z=.25;message.support_ground_xyz.push_back(p);
  tied.end_index=message.support_ground_xyz.size();message.regions.push_back(tied);
  const auto ambiguous=std::make_shared<const PlanningSupport>(message);ASSERT_TRUE(ambiguous->valid);
  GridMap map;GridMapTestAccess::configure(map,.025);
  StairAttitudeScope scope(map,ambiguous,measured());
  EXPECT_FALSE(ambiguous->stairClearanceAllowed({0.,0.,.8}));
  EXPECT_DOUBLE_EQ(map.queryBodyEnvelope({0.,0.,.8}).width,.40);
}

TEST(StairAttitudeScope,ForeignSourceCannotChangeAttitudeSearchHeightOrCollision) {
  GridMap map;GridMapTestAccess::configure(map,.025);GridMapTestAccess::subvoxel(map);
  const Vector body{0.,0.,.55};
  // This floor-level obstacle intersects the upright body. A foreign-frame
  // ramp must not tilt that body away from it or project search Z onto itself.
  map.setOccupied({.475,0.,.325});
  const auto raw=GridMapTestAccess::raw(map);
  const int original=map.getPlanningInflateOccupancy(body,0.);
  ASSERT_EQ(original,1);
  auto source=support()->source;source.frame_id="map_not_odom";
  const auto foreign=std::make_shared<const PlanningSupport>(source);
  ASSERT_TRUE(foreign->valid);
  {
    StairAttitudeScope rejected(map,foreign,measured());
    EXPECT_FALSE(GridMapTestAccess::hasProvider(map));
    EXPECT_FALSE(map.hasQuerySupportHeightProvider());
    EXPECT_FALSE(map.querySupportBodyHeight({.5,0.,.80}));
    EXPECT_FALSE(GridMapTestAccess::stairRegion(map,body));
    EXPECT_EQ(map.getPlanningInflateOccupancy(body,0.),original);
    EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(body).width,.40);
  }
  EXPECT_EQ(raw,GridMapTestAccess::raw(map));
}

TEST(StairAttitudeScope,ForeignNestedScopeSuspendsAndRestoresAllSourceProviders) {
  GridMap map;GridMapTestAccess::configure(map,.025);GridMapTestAccess::subvoxel(map);
  const Vector body{.5,0.,.80};
  auto source=support()->source;source.frame_id="map_not_odom";
  const auto foreign=std::make_shared<const PlanningSupport>(source);
  {
    StairAttitudeScope valid(map,support(),measured(-.15));
    const auto before=GridMapTestAccess::attitude(map,body,0.);
    const auto height_before=map.querySupportBodyHeight(body);
    ASSERT_TRUE(before&&height_before);
    EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(body).width,.35);
    {
      StairAttitudeScope rejected(map,foreign,measured());
      EXPECT_FALSE(GridMapTestAccess::hasProvider(map));
      EXPECT_FALSE(map.hasQuerySupportHeightProvider());
      EXPECT_FALSE(GridMapTestAccess::stairRegion(map,body));
      EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(body).width,.40);
    }
    const auto restored=GridMapTestAccess::attitude(map,body,0.);
    const auto height_restored=map.querySupportBodyHeight(body);
    ASSERT_TRUE(restored&&height_restored);
    EXPECT_TRUE(restored->isApprox(*before,0.));
    EXPECT_DOUBLE_EQ(*height_restored,*height_before);
    EXPECT_TRUE(GridMapTestAccess::stairRegion(map,body));
    EXPECT_DOUBLE_EQ(map.queryBodyEnvelope(body).width,.35);
  }
  EXPECT_FALSE(GridMapTestAccess::hasProvider(map));
  EXPECT_FALSE(map.hasQuerySupportHeightProvider());
  EXPECT_FALSE(GridMapTestAccess::stairRegion(map,body));
}
