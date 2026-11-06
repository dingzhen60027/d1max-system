#include <gtest/gtest.h>
#include <plan_manage/stair_attitude_scope.hpp>
#include <plan_manage/visible_reference_target.hpp>
#include <plan_manage/model_reference_initializer.hpp>

// Production collision implementation, no ROS graph and no cleared inputs.
struct GridMapTestAccess {
  static void configure(GridMap& map) {
    auto& p=map.mp_;auto& d=map.md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={100,80,100};p.map_origin_idx_.setZero();
    p.collision_model_="oriented_box";p.box_length_=1.03;p.box_width_=.4;
    p.box_nominal_length_=.93;p.box_nominal_width_=.30;p.stair_planar_clearance_=.025;
    p.body_below_=p.body_above_=p.planning_body_below_=p.planning_body_above_=.25;
    p.stair_subvoxel_collision_=true;p.stair_contact_body_below_=.12;
    p.double_cylinder_radius_=.2;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.25;p.map_sliding_en_=false;
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.planning_collision_policy_="official";p.require_observed_free_=false;p.frame_id_="odom";
    map.updateMapBoundaryFromIndex();
    const auto count=100U*80U*100U;d.occupancy_buffer_.assign(count,-1.);
    d.occupancy_buffer_inflate_.assign(count,0);d.occupancy_buffer_inflate_cnt_.assign(count,0);
    d.planning_occupancy_buffer_inflate_.assign(count,0);d.planning_occupancy_buffer_inflate_cnt_.assign(count,0);
    d.occupied_hit_subvoxels_.assign(count,0);d.occupied_hit_bounds_.resize(count);
    map.rebuildInflationOffsets();
  }
  static auto raw(const GridMap& map){return map.md_.occupancy_buffer_;}
  static void frame(GridMap& map,const std::string& value){map.mp_.frame_id_=value;}
  static bool bound(const GridMap& map){return bool(map.query_hard_boundary_provider_);}
};

namespace {
using namespace scan_planner;
using Vec=Eigen::Vector3d;
using Wire=d1max_planning_interfaces::msg::SupportReference;
geometry_msgs::msg::Point point(double x,double y,double z=0.) {
  geometry_msgs::msg::Point p;p.x=x;p.y=y;p.z=z;return p;
}
Wire modeled(double grade=.5) {
  Wire s;s.frame_id="odom";s.support_reference_id="model-packet";
  s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');s.stair_model_hash=std::string(64,'c');
  s.body_reference_height_m=.5;s.support_xy_radius_m=.12;
  s.required_mode="stair";s.floor_id="stairs";s.segment_kind="stair_up";s.version.segment_id="lower";
  d1max_planning_interfaces::msg::StairSurfaceModel m;
  m.surface_id="measured-domain-model";m.segment_id="lower";m.kind="continuous_support_envelope";
  m.pose.orientation.w=1.;m.support_grade=grade;m.support_intercept_m=0.;m.contact_tolerance_m=.02;
  m.height_association_m=.25;
  m.polygon={point(-1.,-.7),point(1.,-.7),point(1.,.7),point(-1.,.7)};
  s.stair_model_surfaces.push_back(m);return s;
}
void wall(Wire& s,const Vec& center,const Vec& half) {
  d1max_planning_interfaces::msg::StairBoundaryModel b;b.structure_id="wall";
  b.pose.orientation.w=1.;b.pose.position=point(center.x(),center.y(),center.z());
  b.half_extents.x=half.x();b.half_extents.y=half.y();b.half_extents.z=half.z();
  s.stair_model_obstacles.push_back(b);
}
MeasuredBodyPose absent(){MeasuredBodyPose m;m.frame="absent";return m;}
std::shared_ptr<const PlanningSupport> make(const Wire& s){return std::make_shared<const PlanningSupport>(s);}
void qualifiedPlane(Wire& s,double z) {
  s.verified=true;s.reason="original_stair_surface_returns_associated";s.floor_id="stair_lower";
  for(int x=-5;x<=5;++x)for(int y=-5;y<=5;++y) {
    s.stair_contact_indices.push_back(s.support_ground_xyz.size());
    s.support_ground_xyz.push_back(point(.02*x,.02*y,z));
  }
}
}

TEST(StairModel,IndependentFiniteSupportDoesNotClaimMeasuredReturns) {
  auto s=modeled();PlanningSupport p(s);ASSERT_TRUE(p.valid);ASSERT_TRUE(p.isStair());
  EXPECT_FALSE(p.literalStairValid());EXPECT_FALSE(s.verified);EXPECT_TRUE(s.support_ground_xyz.empty());
  const auto fit=p.terrainFit({.5,0.,.75});ASSERT_TRUE(fit.valid);
  EXPECT_EQ(fit.samples,0U);EXPECT_TRUE(std::isnan(fit.rms));EXPECT_TRUE(std::isnan(fit.minor_variance));
  EXPECT_NEAR(fit.center_residual,0.,1e-15);EXPECT_TRUE(fit.normal.isApprox(Vec(-.5,0.,1.).normalized()));
  auto ground=p.nearest(.5,0.,.25);ASSERT_TRUE(ground);EXPECT_DOUBLE_EQ(ground->z,.25);
  EXPECT_FALSE(p.terrainFit({1.01,0.,1.005}).valid);EXPECT_FALSE(p.nearest(0.,.701,0.));
  EXPECT_EQ(p.stairFlightAt({.5,0.,.75}),std::optional<std::string>("lower"));
  EXPECT_FALSE(StairContactQuery(make(s),"odom",.05).valid());
}

TEST(StairModel,LandingIsGeometricAndDoesNotRequireLiteralRegionName) {
  auto s=modeled();auto& flight=s.stair_model_surfaces[0];
  flight.polygon={point(-1.,-.7),point(0.,-.7),point(0.,.7),point(-1.,.7)};
  auto landing=flight;landing.surface_id="physical-landing";landing.segment_id="upper";
  landing.kind="landing";landing.support_grade=0.;
  landing.polygon={point(0.,-.7),point(1.,-.7),point(1.,.7),point(0.,.7)};
  s.stair_model_surfaces.push_back(landing);ASSERT_TRUE(s.regions.empty());
  PlanningSupport p(s);ASSERT_TRUE(p.valid);
  EXPECT_EQ(p.stairFlightAt({-.5,0.,.25}),std::optional<std::string>("lower"));
  EXPECT_FALSE(p.stairFlightAt({.5,0.,.5}));EXPECT_FALSE(p.stairFlightAt({0.,0.,.5}));
  EXPECT_TRUE(p.terrainFit({.5,0.,.5}).normal.isApprox(Vec::UnitZ()));
  MeasuredBodyPose missing=absent();StairAttitudeQuery query(make(s),missing);
  // Both positions share a legacy 5 cm cache cell; the exact domain wins.
  EXPECT_NEAR((*query({-.001,0.,.4995},0.)).x(),-std::atan(.5),1e-12);
  EXPECT_DOUBLE_EQ((*query({.001,0.,.5},0.)).x(),0.);
}

TEST(StairModel,SameXYDifferentStoreyCannotBorrowModelHeight) {
  auto s=modeled(0.);s.stair_model_surfaces[0].support_intercept_m=1.8;
  PlanningSupport upper(s);ASSERT_TRUE(upper.valid);
  EXPECT_FALSE(upper.terrainFit({0.,0.,.5}).valid);
  EXPECT_FALSE(upper.stairClearanceAllowed({0.,0.,.5}));
  EXPECT_TRUE(upper.terrainFit({0.,0.,2.3}).valid);
  auto lower=s.stair_model_surfaces[0];lower.surface_id="lower-floor-landing";
  lower.kind="landing";lower.segment_id="entry";lower.support_intercept_m=0.;
  s.stair_model_surfaces.push_back(lower);PlanningSupport both(s);
  EXPECT_NEAR(both.terrainFit({0.,0.,.5}).center_residual,0.,1e-12);
  EXPECT_NEAR(both.terrainFit({0.,0.,2.3}).center_residual,0.,1e-12);
  EXPECT_FALSE(both.terrainFit({0.,0.,1.4}).valid);
  s.stair_model_surfaces[0].height_association_m=0.;EXPECT_FALSE(PlanningSupport(s).valid);
}

TEST(StairModel,FullPoseAnchorTransformsHeightNormalAndFiniteObstacleTogether) {
  auto s=modeled();wall(s,{.5,.65,.5},{.5,.03,.4});
  const Eigen::Quaterniond q(Eigen::AngleAxisd(.4,Vec::UnitZ())*
    Eigen::AngleAxisd(.12,Vec::UnitY())*Eigen::AngleAxisd(-.08,Vec::UnitX()));
  const Vec t(.2,-.3,.4);
  for(auto& m:s.stair_model_surfaces){m.pose.position=point(t.x(),t.y(),t.z());
    m.pose.orientation.x=q.x();m.pose.orientation.y=q.y();m.pose.orientation.z=q.z();m.pose.orientation.w=q.w();}
  auto& b=s.stair_model_obstacles[0];const Vec c=t+q*Vec(.5,.65,.5);
  b.pose.position=point(c.x(),c.y(),c.z());b.pose.orientation=s.stair_model_surfaces[0].pose.orientation;
  PlanningSupport support(s);ASSERT_TRUE(support.valid);
  const Vec literal=t+q*Vec(.4,.1,.2),body=t+q*Vec(.4,.1,.2+.5);
  const auto fit=support.terrainFit(body);ASSERT_TRUE(fit.valid);
  EXPECT_NEAR(fit.center_residual,0.,1e-12);EXPECT_TRUE(fit.normal.isApprox(q*Vec(-.5,0.,1.).normalized(),1e-12));
  const auto sampled=support.model()->sampleBody(body,.5);ASSERT_TRUE(sampled);
  EXPECT_TRUE(sampled->ground.isApprox(literal,1e-12));EXPECT_TRUE(sampled->body.isApprox(body,1e-12));
  EXPECT_EQ(support.stairFlightAt(body),std::optional<std::string>("lower"));
  GridMap map;GridMapTestAccess::configure(map);
  {StairAttitudeScope scope(map,make(s),absent());const auto height=map.querySupportBodyHeight(body);
    ASSERT_TRUE(height);EXPECT_NEAR(*height,body.z(),1e-12);
    DiscreteReference route;route.set({t+q*Vec(-.4,0.,-.2+.5),t+q*Vec(.4,0.,.2+.5)});
    const Vec start=route.sample(0.);const Vec heading=q*Vec(1.,0.,.5);
    const auto target=selectPlanningSideTarget(route,start,std::atan2(heading.y(),heading.x()),
      0.,1.,support,map,"odom",std::make_shared<SolveBudget>(),1024);
    ASSERT_TRUE(target.target.valid)<<target.target.reason;
    const auto exact=support.bodyPositionOnSupport(target.target.point);ASSERT_TRUE(exact);
    EXPECT_NEAR(exact->z(),target.target.point.z(),1e-12);
  }
  auto shape=BodyEnvelope{};shape.model=CollisionModel::OrientedBox;shape.length=.8;shape.width=.3;
  shape.below=shape.above=.2;
  EXPECT_TRUE(support.model()->hardCollision(c,.4,.12,-.08,shape));
  EXPECT_FALSE(support.model()->hardCollision(c+q*Vec(0.,0.,1.),.4,.12,-.08,shape));
}

TEST(StairModel,InvalidModelNeverUsesPartialModelOrPromotesLiteralContact) {
  const auto original=modeled();
  for(int fault=0;fault<8;++fault) {
    auto s=original;
    if(fault==0)s.stair_model_hash="invalid";
    if(fault==1)s.stair_model_surfaces[0].pose.orientation.w=.5;
    if(fault==2)std::swap(s.stair_model_surfaces[0].polygon[1],s.stair_model_surfaces[0].polygon[2]);
    if(fault==3)s.stair_model_surfaces[0].kind="measured_points";
    if(fault==4)s.stair_model_surfaces[0].polygon[0].z=.01;
    if(fault==5)s.stair_model_surfaces[0].tread_heights_m={0.};
    if(fault==6)s.stair_model_surfaces.resize(65,s.stair_model_surfaces[0]);
    if(fault==7){wall(s,{0.,0.,0.},{1.,1.,1.});s.stair_model_obstacles[0].half_extents.z=0.;}
    EXPECT_FALSE(PlanningSupport(s).valid)<<fault;
  }
  auto s=original;s.stair_contact_indices={999};s.support_ground_xyz={point(0.,0.,0.)};
  ASSERT_TRUE(PlanningSupport(s).valid);EXPECT_FALSE(StairContactQuery(make(s),"odom",.05).valid());
}

TEST(StairModel,UnsupportedDiscreteStepsCannotSilentlyBecomeAContinuousEnvelope) {
  auto s=modeled();auto& m=s.stair_model_surfaces[0];m.kind="standard_steps";
  m.tread_stations_m={-1.,0.,1.};m.tread_heights_m={-.25,.25};
  EXPECT_FALSE(d1max_planning_interfaces::hasParametricStairGeometry(s));
  PlanningSupport p(s);EXPECT_FALSE(p.valid);EXPECT_FALSE(p.literalStairValid());
  EXPECT_FALSE(StairRiserContactQuery(make(s),"odom",.05).valid());
  m.tread_stations_m[1]=-1.;EXPECT_FALSE(PlanningSupport(s).valid);
}

TEST(StairModel,ModelHeightAndMeasuredAttitudeAreNotDoubleLifted) {
  auto s=modeled();auto p=make(s);GridMap map;GridMapTestAccess::configure(map);
  auto m=absent();m.frame="odom";m.position={0.,0.,.5};
  m.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(-.12,Vec::UnitY())*Eigen::AngleAxisd(.04,Vec::UnitX()));
  StairAttitudeQuery q(p,m);auto now=q(m.position,0.);ASSERT_TRUE(now);
  EXPECT_NEAR(now->x(),-.12,1e-12);EXPECT_NEAR(now->y(),.04,1e-12);
  auto future=q({.8,0.,.9},0.);ASSERT_TRUE(future);EXPECT_NEAR(future->x(),-std::atan(.5),1e-12);
  {StairAttitudeScope scope(map,p,m);auto z=map.querySupportBodyHeight({.8,0.,.9});ASSERT_TRUE(z);EXPECT_DOUBLE_EQ(*z,.9);}
  EXPECT_FALSE(map.querySupportBodyHeight({.8,0.,.9}));
}

TEST(StairModel,HardWallsAreFiniteAndRawEvidenceIsNotFabricated) {
  auto s=modeled(0.);wall(s,{0.,.6,.5},{.5,.03,.4});
  GridMap map;GridMapTestAccess::configure(map);const auto raw=GridMapTestAccess::raw(map);
  ASSERT_EQ(map.getPlanningInflateOccupancy({0.,.5,.5},0.),0);
  {StairAttitudeScope scope(map,make(s),absent());
    EXPECT_EQ(map.getPlanningInflateOccupancy({0.,.5,.5},0.),1);
    EXPECT_EQ(map.getInflateOccupancy({0.,.5,.5},0.),1);
    EXPECT_EQ(map.getPlanningInflateOccupancy({0.,0.,.5},0.),0);
    EXPECT_EQ(map.getPlanningInflateOccupancy({1.3,.5,.5},0.),0);
    EXPECT_EQ(map.getPlanningInflateOccupancy({0.,.5,1.6},0.),0);
    const auto evidence=map.inspectInflateOccupancy({0.,.5,.5},0.,true);
    EXPECT_TRUE(evidence.modeled_hard_boundary_collision);EXPECT_EQ(evidence.counts[1],0U);EXPECT_EQ(evidence.state(),0);
  }
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);EXPECT_EQ(map.getPlanningInflateOccupancy({0.,.5,.5},0.),0);
}

TEST(StairModel,BoundedWitnessRecordsExactTransformedModelAndRestoresWithScope) {
  auto s=modeled(0.);wall(s,{0.,.6,.5},{.5,.03,.4});
  // Deliberately more intersecting structures than the fixed log limit.
  for(int i=1;i<7;++i){auto b=s.stair_model_obstacles[0];b.structure_id="wall-"+std::to_string(i);
    s.stair_model_obstacles.push_back(b);}
  GridMap map;GridMapTestAccess::configure(map);const Vec p(0.,.5,.5);
  const auto raw=GridMapTestAccess::raw(map);
  const auto parse=[&](const std::string& description) {
    const std::string key=" model_hard_witness=";const auto begin=description.find(key);
    if(begin==std::string::npos)return nlohmann::json();
    const auto end=description.find(" body_xy_box=",begin);
    return nlohmann::json::parse(description.substr(begin+key.size(),end-begin-key.size()));
  };
  EXPECT_TRUE(parse(map.describeInflateOccupancy(p,0.,0.,0.)).is_null());
  {StairAttitudeScope scope(map,make(s),absent());
    const auto evidence=map.inspectInflateOccupancy(p,0.,0.,0.,true);
    ASSERT_TRUE(evidence.modeled_hard_boundary_collision);ASSERT_EQ(evidence.counts[1],0U);
    const auto d=parse(map.describeInflateOccupancy(p,0.,0.,0.));ASSERT_TRUE(d.is_object());
    EXPECT_EQ(d["model_hash"],s.stair_model_hash);EXPECT_EQ(d["support_hash"],s.support_hash);
    EXPECT_EQ(d["query_frame"],"odom");EXPECT_EQ(d["support_frame"],"odom");
    EXPECT_EQ(d["query"]["xyz"],nlohmann::json::array({0.,.5,.5}));
    EXPECT_DOUBLE_EQ(d["query"]["half_extents"][0].get<double>(),.49);
    EXPECT_DOUBLE_EQ(d["query"]["half_extents"][1].get<double>(),.175);
    EXPECT_DOUBLE_EQ(d["query"]["half_extents"][2].get<double>(),.25);
    EXPECT_EQ(d["structures"].size(),4U);EXPECT_EQ(d["intersections_total"],7);
    EXPECT_EQ(d["intersections_omitted"],3);EXPECT_EQ(d["structures"][0]["structure_id"],"wall");
    EXPECT_EQ(d["structures"][0]["center"],nlohmann::json::array({0.,.6,.5}));
    EXPECT_EQ(d["structures"][0]["half_extents"],nlohmann::json::array({.5,.03,.4}));
    EXPECT_EQ(d["support_sample"]["surface_id"],s.stair_model_surfaces[0].surface_id);
    EXPECT_EQ(d["support_sample"]["polygon"].size(),4U);EXPECT_FALSE(d["ids_truncated"].get<bool>());
    EXPECT_LT(d.dump().size(),5000U);
    {StairAttitudeScope nested(map,{},absent());EXPECT_TRUE(parse(map.describeInflateOccupancy(p,0.,0.,0.)).is_null());}
    EXPECT_EQ(d,parse(map.describeInflateOccupancy(p,0.,0.,0.)));
    GridMap snapshot;map.copyCollisionSnapshotTo(snapshot,1000000000,std::chrono::steady_clock::now());
    EXPECT_TRUE(parse(snapshot.describeInflateOccupancy(p,0.,0.,0.)).is_null());
  }
  EXPECT_TRUE(parse(map.describeInflateOccupancy(p,0.,0.,0.)).is_null());
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);
}

TEST(StairModel,WitnessUsesFullPoseAndBoundsUntrustedIdentifiers) {
  auto s=modeled();wall(s,{.2,.1,.5},{.1,.2,.3});
  auto& wall=s.stair_model_obstacles[0];wall.structure_id=std::string(4096,'w');
  const Eigen::Quaterniond rotation(Eigen::AngleAxisd(.4,Vec::UnitZ())*
    Eigen::AngleAxisd(.12,Vec::UnitY())*Eigen::AngleAxisd(-.08,Vec::UnitX()));
  wall.pose.orientation.w=rotation.w();wall.pose.orientation.x=rotation.x();
  wall.pose.orientation.y=rotation.y();wall.pose.orientation.z=rotation.z();
  const PlanningSupport support(s);ASSERT_TRUE(support.valid);
  BodyEnvelope body;body.model=CollisionModel::OrientedBox;body.length=.98;body.width=.35;
  body.below=.12;body.above=.25;
  const auto d=nlohmann::json::parse(support.model()->describeHardCollision({.2,.1,.5},.4,.12,-.08,body));
  ASSERT_EQ(d["structures"].size(),1U);EXPECT_TRUE(d["ids_truncated"].get<bool>());
  EXPECT_EQ(d["structures"][0]["structure_id"].get<std::string>().size(),256U);
  Eigen::Matrix3d recorded;for(int i=0;i<3;++i)for(int j=0;j<3;++j)
    recorded(i,j)=d["structures"][0]["rotation_rows"][i][j].get<double>();
  EXPECT_TRUE(recorded.isApprox(rotation.toRotationMatrix(),1e-15));
  EXPECT_TRUE(support.model()->hardCollision({.2,.1,.5},.4,.12,-.08,body));
}

TEST(StairModel,DiagnosticUtf8TruncationAndMalformedLabelsCannotInterruptCollisionReporting) {
  for(bool malformed:{false,true}) {
    auto s=modeled(0.);wall(s,{0.,.6,.5},{.5,.03,.4});
    std::string label;for(int i=0;i<100;++i)label+=u8"楼";
    if(malformed)label=std::string("bad-")+char(0xff)+label;
    s.frame_id=label;s.support_reference_id=label;s.stair_model_obstacles[0].structure_id=label;
    s.stair_model_surfaces[0].surface_id=label;s.stair_model_surfaces[0].segment_id=label;
    const auto support=make(s);ASSERT_TRUE(support->valid);
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::frame(map,label);
    // The scope's query frame and the model's support/structure identifiers
    // must both serialize despite the byte cap or invalid wire label bytes.
    const Vec p(0.,.5,.5);StairAttitudeScope scope(map,support,absent());
    EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.,0.,0.),1);
    std::string description;ASSERT_NO_THROW(description=map.describeInflateOccupancy(p,0.,0.,0.));
    const std::string key=" model_hard_witness=";const auto begin=description.find(key);
    ASSERT_NE(begin,std::string::npos);const auto end=description.find(" body_xy_box=",begin);
    nlohmann::json d;ASSERT_NO_THROW(d=nlohmann::json::parse(description.substr(begin+key.size(),end-begin-key.size())));
    EXPECT_TRUE(d["ids_truncated"].get<bool>());EXPECT_EQ(d["query_frame"],d["support_frame"]);
    const auto recorded=d["structures"][0]["structure_id"].get<std::string>();
    if(!malformed){EXPECT_EQ(recorded.size(),255U);EXPECT_EQ(recorded,label.substr(0,255));}
    else EXPECT_NE(recorded.find(u8"\ufffd"),std::string::npos);
    EXPECT_LT(d.dump().size(),10000U); // Replacement remains bounded, not a dump of the full source.
  }
}

TEST(StairModel,EnvelopeDoesNotExcuseNewLowBoxOrRailOrUnknown) {
  auto s=modeled(0.);s.stair_model_surfaces[0].contact_tolerance_m=.05;
  for(const Vec& hit:{Vec(.02,.02,.276),Vec(.02,.02,.4),Vec(.02,.17,.5)}) {
    GridMap map;GridMapTestAccess::configure(map);map.setOccupiedHit(hit);const auto raw=GridMapTestAccess::raw(map);
    {StairAttitudeScope scope(map,make(s),absent());
      EXPECT_EQ(map.getPlanningInflateOccupancy({0.,0.,.5},0.),1)<<hit.transpose();}
    EXPECT_EQ(GridMapTestAccess::raw(map),raw);
  }
}

TEST(StairModel,OriginalQualifiedContactRemainsIndependentAndMixedHitStillBlocks) {
  auto s=modeled(0.);qualifiedPlane(s,.276);ASSERT_TRUE(PlanningSupport(s).literalStairValid());
  GridMap map;GridMapTestAccess::configure(map);map.setOccupiedHit({.02,.02,.276});
  {StairAttitudeScope scope(map,make(s),absent());
    EXPECT_EQ(map.getPlanningInflateOccupancy({0.,0.,.5},0.),0);
    map.setOccupiedHit({.02,.02,.296});EXPECT_EQ(map.getPlanningInflateOccupancy({0.,0.,.5},0.),1);
  }
  s.verified=false;
  EXPECT_FALSE(StairContactQuery(make(s),"odom",.05).valid());
}

TEST(StairModel,SnapshotAndNestedBindingCannotLeakModelAcrossVersionOrFrame) {
  auto s=modeled(0.);wall(s,{0.,.6,.5},{.5,.03,.4});
  GridMap map;GridMapTestAccess::configure(map);const Vec p(0.,.5,.5);
  {StairAttitudeScope outer(map,make(s),absent());ASSERT_TRUE(GridMapTestAccess::bound(map));
    EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),1);
    {StairAttitudeScope flat(map,{},absent());EXPECT_FALSE(GridMapTestAccess::bound(map));EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),0);}
    EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),1);
    auto foreign=s;foreign.frame_id="foreign";
    {StairAttitudeScope invalid(map,make(foreign),absent());EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),0);}
    GridMap reader;map.copyCollisionSnapshotTo(reader,1000000000,std::chrono::steady_clock::now());
    EXPECT_FALSE(GridMapTestAccess::bound(reader));EXPECT_EQ(reader.getPlanningInflateOccupancy(p,0.),0);
    {StairAttitudeScope rebound(reader,make(s),absent());EXPECT_EQ(reader.getPlanningInflateOccupancy(p,0.),1);}
  }
  EXPECT_FALSE(GridMapTestAccess::bound(map));EXPECT_EQ(map.getPlanningInflateOccupancy(p,0.),0);
}

namespace {
const std::vector<Vec> initializer_route{{-.7,0.,.5},{.7,0.,.5}};
ModelReferenceInitializerResult recoverInitializer(GridMap& map,
    const std::shared_ptr<const PlanningSupport>& support,const SolveBudget::Ptr& budget,
    const OccupancyQuery& occupied,std::vector<Vec>& output,std::string& reason,
    const std::string& failure="open_set_exhausted",const DetourSearch& search={}) {
  return tryModelReferenceInitializer(map,support,budget,initializer_route,occupied,
      search?search:DetourSearch([](const Vec&,const Vec&){return DetourSearchResult{};}),
      "failed_reference_search",failure,output,reason);
}
}

TEST(StairModelInitializer,RestoredNativeModelAcceptsOnlyFullGuideAndKeepsRawBytes) {
  auto s=modeled(0.);wall(s,{0.,.6,.5},{.5,.03,.4});auto support=make(s);
  GridMap map;GridMapTestAccess::configure(map);const auto raw=GridMapTestAccess::raw(map);
  StairAttitudeScope scope(map,support,absent());
  std::vector<Vec> output;std::string reason="first_failure";
  const auto occupancy=[&](const Vec& p,double yaw){return map.getPlanningInflateOccupancy(p,yaw);};
  const auto result=recoverInitializer(map,support,std::make_shared<SolveBudget>(),occupancy,output,reason);
  ASSERT_TRUE(result.attempted);ASSERT_TRUE(result.accepted)<<result.reason;
  ASSERT_GE(output.size(),2U);EXPECT_TRUE(output.front().isApprox(initializer_route.front()));
  EXPECT_TRUE(output.back().isApprox(initializer_route.back()));
  for(std::size_t i=1;i<output.size();++i)EXPECT_TRUE(collisionFreeSegment(output[i-1],output[i],.025,occupancy));
  EXPECT_TRUE(GridMapTestAccess::bound(map));EXPECT_EQ(map.getPlanningInflateOccupancy({0.,.5,.5},0.),1);
  EXPECT_EQ(GridMapTestAccess::raw(map),raw);
}

TEST(StairModelInitializer,RawFreeProposalCannotCrossFiniteModelWallOrReuseRawCache) {
  auto s=modeled(0.);wall(s,{0.,0.,.5},{.03,.7,.4});auto support=make(s);
  GridMap map;GridMapTestAccess::configure(map);StairAttitudeScope scope(map,support,absent());
  ASSERT_EQ(map.getPlanningInflateOccupancy({0.,0.,.5},0.),1);
  bool raw_free=false;const auto occupancy=[&](const Vec& p,double yaw){
    const auto state=map.getPlanningInflateOccupancy(p,yaw);
    if(!GridMapTestAccess::bound(map)&&std::abs(p.x())<.1&&state==0)raw_free=true;
    return state;
  };
  std::vector<Vec> output{{9,9,9}};std::string reason="first_failure";
  const auto result=recoverInitializer(map,support,std::make_shared<SolveBudget>(),occupancy,output,reason);
  EXPECT_TRUE(raw_free);EXPECT_TRUE(result.attempted);EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason,"failed_model_proposal_collision");EXPECT_EQ(output.size(),1U);
  EXPECT_EQ(reason,"first_failure");EXPECT_EQ(map.getPlanningInflateOccupancy({0.,0.,.5},0.),1);
  EXPECT_TRUE(GridMapTestAccess::bound(map));
}

TEST(StairModelInitializer,ActualRawBoxAndUnobservedOutsideMapAreNeverCleared) {
  auto s=modeled(0.);wall(s,{0.,.6,.5},{.5,.03,.4});auto support=make(s);
  for(bool outside:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map);
    if(!outside)map.setOccupiedHit({0.,0.,.5});
    const auto raw=GridMapTestAccess::raw(map);StairAttitudeScope scope(map,support,absent());
    bool raw_blocked=false;int searches=0;
    const auto occupancy=[&](const Vec& p,double yaw){
      const auto state=map.getPlanningInflateOccupancy(outside?Vec(100.,0.,.5):p,yaw);
      if(!GridMapTestAccess::bound(map)&&state!=0)raw_blocked=true;
      return state;
    };
    const DetourSearch search=[&](const Vec&,const Vec&){++searches;return DetourSearchResult{};};
    std::vector<Vec> output;std::string reason;
    const auto result=recoverInitializer(map,support,std::make_shared<SolveBudget>(),occupancy,output,reason,
                                        "open_set_exhausted",search);
    EXPECT_TRUE(result.attempted);EXPECT_FALSE(result.accepted);EXPECT_TRUE(raw_blocked);
    EXPECT_LE(searches,1);EXPECT_EQ(GridMapTestAccess::raw(map),raw);EXPECT_TRUE(GridMapTestAccess::bound(map));
  }
}

TEST(StairModelInitializer,OnlyExactExhaustionWithBoundSameFrameModelCanAttempt) {
  auto s=modeled(0.);wall(s,{0.,.6,.5},{.5,.03,.4});auto support=make(s);
  GridMap map;GridMapTestAccess::configure(map);int queries=0;
  const auto occupancy=[&](const Vec& p,double yaw){++queries;return map.getPlanningInflateOccupancy(p,yaw);};
  std::vector<Vec> output;std::string reason;
  for(const auto& failure:{"deadline_exceeded","cancelled","start_occupied","unobserved","outside_map",""}) {
    StairAttitudeScope scope(map,support,absent());
    EXPECT_FALSE(recoverInitializer(map,support,std::make_shared<SolveBudget>(),occupancy,output,reason,failure).attempted);
  }
  EXPECT_FALSE(recoverInitializer(map,support,std::make_shared<SolveBudget>(),occupancy,output,reason).attempted);
  {StairAttitudeScope scope(map,support,absent());
    EXPECT_FALSE(recoverInitializer(map,{},std::make_shared<SolveBudget>(),occupancy,output,reason).attempted);
    auto foreign=s;foreign.frame_id="foreign";
    EXPECT_FALSE(recoverInitializer(map,make(foreign),std::make_shared<SolveBudget>(),occupancy,output,reason).attempted);
    EXPECT_FALSE(tryModelReferenceInitializer(map,support,std::make_shared<SolveBudget>(),initializer_route,
      occupancy,[](const Vec&,const Vec&){return DetourSearchResult{};},"failed_reference_target_occupied",
      "open_set_exhausted",output,reason).attempted);
  }
  EXPECT_EQ(queries,0);
}

TEST(StairModelInitializer,CancellationAndOriginalDeadlineNeverRenewOrPublishProposal) {
  auto s=modeled(0.);wall(s,{0.,.6,.5},{.5,.03,.4});auto support=make(s);
  for(bool expired:{false,true})for(bool before:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map);StairAttitudeScope scope(map,support,absent());
    auto now=SolveBudget::Clock::now();auto budget=std::make_shared<SolveBudget>(
      std::chrono::milliseconds(400),[&]{return now;});const auto deadline=budget->deadline();
    const auto stop=[&]{if(expired)now=deadline;else budget->cancel();};
    if(before)stop();
    const auto occupancy=[&](const Vec& p,double yaw){stop();return map.getPlanningInflateOccupancy(p,yaw);};
    std::vector<Vec> output;std::string reason;
    const auto result=recoverInitializer(map,support,budget,occupancy,output,reason);
    EXPECT_EQ(result.attempted,!before);EXPECT_FALSE(result.accepted);EXPECT_TRUE(output.empty());
    EXPECT_EQ(budget->deadline(),deadline);EXPECT_TRUE(GridMapTestAccess::bound(map));
    EXPECT_EQ(map.getPlanningInflateOccupancy({0.,.5,.5},0.),1);
  }
}

TEST(StairModelInitializer,ExceptionRestoresOriginalProviderAndInvalidReferenceDoesNotAttempt) {
  auto s=modeled(0.);wall(s,{0.,.6,.5},{.5,.03,.4});auto support=make(s);
  GridMap map;GridMapTestAccess::configure(map);StairAttitudeScope scope(map,support,absent());
  std::vector<Vec> output;std::string reason;
  const OccupancyQuery throwing=[](const Vec&,double)->int{throw std::runtime_error("fixture query failure");};
  EXPECT_THROW(recoverInitializer(map,support,std::make_shared<SolveBudget>(),throwing,output,reason),std::runtime_error);
  EXPECT_TRUE(GridMapTestAccess::bound(map));EXPECT_EQ(map.getPlanningInflateOccupancy({0.,.5,.5},0.),1);
  const DetourSearch search=[](const Vec&,const Vec&){return DetourSearchResult{};};
  const auto occupancy=[&](const Vec& p,double yaw){return map.getPlanningInflateOccupancy(p,yaw);};
  for(const auto& reference:std::vector<std::vector<Vec>>{{},{Vec(0,0,.5)},
      {Vec(0,0,.5),Vec(std::numeric_limits<double>::quiet_NaN(),0,.5)}}) {
    EXPECT_FALSE(tryModelReferenceInitializer(map,support,std::make_shared<SolveBudget>(),reference,
        occupancy,search,"failed_reference_search","open_set_exhausted",output,reason).attempted);
  }
}
