#include <gtest/gtest.h>
#include <plan_manage/motion_sweep.hpp>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <fstream>
#include <filesystem>
#include <iomanip>

// The same production GridMap buffers and strictRawVoxelStatus implementation
// are exercised in memory. No rclcpp context, ROS graph, sensor subscription or
// alternate fake occupancy implementation is used.
struct GridMapTestAccess {
  static void configure(GridMap& m) {
    auto& p=m.mp_;auto& d=m.md_;
    p.resolution_=.05;p.resolution_inv_=20.;p.map_voxel_num_={80,80,60};
    p.map_origin_idx_={0,0,10};m.updateMapBoundaryFromIndex();
    p.use_projected_rays_=p.require_observed_free_=true;
    p.clamp_min_log_=-1.;p.min_occupancy_log_=1.;p.clamp_max_log_=2.;p.unknown_flag_=.01;
    p.double_cylinder_radius_=.1;p.double_cylinder_offset_=.2;
    p.obstacles_inflation_z_up=p.obstacles_inflation_z_down=.1;p.cloud_pose_max_age_=.5;
    p.frame_id_="odom";p.map_sliding_en_=false;
    d.occupancy_buffer_.assign(80*80*60,-1.);d.occupancy_buffer_inflate_.assign(80*80*60,0);
    d.occupancy_buffer_inflate_cnt_.assign(80*80*60,0);m.rebuildInflationOffsets();
    m.free_observation_stamps_.assign(80*80*60,100000000000LL);
    m.ray_integrated_stamps_={100000000000LL,100000000000LL};m.integrated_cloud_stamp_ns_=100000000000LL;
    m.snapshot_clock_ns_=100100000000LL;m.snapshot_captured_=std::chrono::steady_clock::now();
    m.collision_snapshot_=true;m.enforce_free_freshness_=true;m.ray_clock_fault_=false;
  }
  static void voxel(GridMap& m,const Eigen::Vector3d& p,double odds) {
    Eigen::Vector3i id;m.posToIndex(p,id);ASSERT_TRUE(m.isInMap(id));m.md_.occupancy_buffer_[m.toAddress(id)]=odds;
  }
  static void expire(GridMap& m) {
    m.snapshot_clock_ns_=100600000000LL;m.snapshot_captured_=std::chrono::steady_clock::now();
  }
  static void deployedEnvelope(GridMap& m) {
    m.mp_.double_cylinder_radius_=.29;m.mp_.double_cylinder_offset_=.20;
    m.mp_.obstacles_inflation_z_up=m.mp_.obstacles_inflation_z_down=.45;
    m.rebuildInflationOffsets();
  }
  static void asymmetricEnvelope(GridMap& m) {
    m.mp_.double_cylinder_radius_=.29;m.mp_.double_cylinder_offset_=.20;
    m.mp_.obstacles_inflation_z_up=.40;m.mp_.obstacles_inflation_z_down=.45;
    m.rebuildInflationOffsets();
  }
  static void plane(GridMap& m,double z) {
    for(int x=-30;x<=30;++x)for(int y=-30;y<=30;++y)voxel(m,{x*.05,y*.05,z},2.);
  }
  static void stamp(GridMap& m,const Eigen::Vector3d& p,std::int64_t value) {
    Eigen::Vector3i id;m.posToIndex(p,id);m.free_observation_stamps_[m.toAddress(id)]=value;
  }
};
namespace {
using namespace scan_planner;
BrakingModel model() {return {.3,.5,.4,.08,.20,.5,.01,.03,std::string(64,'a')};}
d1max_planning_interfaces::msg::SupportReference ground() {
  d1max_planning_interfaces::msg::SupportReference s;s.verified=true;s.floor_id="floor1";s.segment_kind="floor";
  s.required_mode="general";s.frame_id="odom";s.support_hash=std::string(64,'a');s.support_map_sha256=std::string(64,'b');
  s.support_xy_radius_m=.04;s.body_reference_height_m=.55;s.max_support_slope_rad=.15;s.max_support_step_m=.05;
  for(int x=-40;x<=40;++x)for(int y=-40;y<=40;++y){geometry_msgs::msg::Point p;p.x=x*.025;p.y=y*.025;s.support_ground_xyz.push_back(p);}return s;
}
MotionSweepResult run(GridMap& m,const geometry_msgs::msg::Twist& command,
    const geometry_msgs::msg::Twist& measured={},const BrakingModel& braking=model(),double budget=.025) {
  MotionSupport s(ground());return validateMotionSweep(m,s,braking,{0.,0.,.55},0.,measured,command,"odom",budget);
}
std::string digest(const std::string& bytes) {
  unsigned char out[EVP_MAX_MD_SIZE];unsigned n=0;EVP_Digest(bytes.data(),bytes.size(),out,&n,EVP_sha256(),nullptr);
  std::ostringstream s;for(unsigned i=0;i<n;++i)s<<std::hex<<std::setfill('0')<<std::setw(2)<<int(out[i]);return s.str();
}
}
TEST(MotionSweep,ActualNativeObservedMapProvesMovingAndTurningReachableEnvelope) {
  GridMap m;GridMapTestAccess::configure(m);geometry_msgs::msg::Twist v;v.linear.x=.3;v.angular.z=.5;
  const auto out=run(m,v);EXPECT_TRUE(out.valid)<<out.reason;EXPECT_GT(out.unique_voxels,100U);
  EXPECT_NEAR(out.length,.2,1e-9);EXPECT_NEAR(out.heading_bound,.43,1e-9);
  EXPECT_GT(m.observedProofDeadlineNs(),100100000000LL);
}
TEST(MotionSweep,AsymmetricReflectedEnvelopeDoesNotInventGroundAndCannotMissHeadObstacle) {
  auto g=ground();g.body_reference_height_m=.50;const MotionSupport support(g);
  const Eigen::Vector3d body(0.,0.,.50);
  geometry_msgs::msg::Twist command;command.linear.x=.02;command.angular.z=-.04;
  for(double plane:{.0,.10,.960})for(double yaw:{0.,-1.570796326794897}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::asymmetricEnvelope(map);
    const auto kernel=map.obstacleDilation();const auto envelope=map.bodyEnvelope();
    ASSERT_DOUBLE_EQ(kernel.up,.40);ASSERT_DOUBLE_EQ(kernel.down,.45);
    ASSERT_DOUBLE_EQ(envelope.below,.40);ASSERT_DOUBLE_EQ(envelope.above,.45);
    GridMapTestAccess::plane(map,plane);map.beginObservedProof();
    const int pose=map.getInflateOccupancy(body,yaw);
    const auto motion=validateMotionSweep(map,support,model(),body,yaw,{},command,"odom",.1);
    if(plane==0.) {EXPECT_EQ(pose,0);EXPECT_TRUE(motion.valid)<<motion.reason;}
    else {EXPECT_EQ(pose,1);EXPECT_FALSE(motion.valid);EXPECT_EQ(motion.reason,"motion_sweep_occupied");}
    const double pad=std::tan(.15)*.025+.01;
    EXPECT_NEAR(motion.min_z,.50-.40-pad,1e-12);
    EXPECT_NEAR(motion.max_z,.50+.45+pad,1e-12);
  }
}
TEST(MotionSweep,FailedEnvelopeDiagnosticsSeparateClassesAndUniqueFromRawOverlap) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::asymmetricEnvelope(map);
  GridMapTestAccess::voxel(map,{.025,.025,.50},-1.01);
  GridMapTestAccess::voxel(map,{.025,.025,.60},0.);
  GridMapTestAccess::voxel(map,{.025,.025,.70},2.);
  GridMapTestAccess::stamp(map,{.025,.025,.80},99000000000LL);
  const auto evidence=map.inspectInflateOccupancy({0.,0.,.5},0.,true);
  EXPECT_EQ(evidence.classification_counts[static_cast<std::size_t>(RawVoxelDiagnostic::NeverObserved)],1U);
  EXPECT_EQ(evidence.classification_counts[static_cast<std::size_t>(RawVoxelDiagnostic::Insufficient)],1U);
  EXPECT_EQ(evidence.classification_counts[static_cast<std::size_t>(RawVoxelDiagnostic::Occupied)],1U);
  EXPECT_EQ(evidence.classification_counts[static_cast<std::size_t>(RawVoxelDiagnostic::StaleFree)],1U);
  EXPECT_EQ(evidence.unique_counts[1],1U);EXPECT_EQ(evidence.unique_counts[2],3U);
  EXPECT_GT(evidence.counts[2],evidence.unique_counts[2]);
  const auto detail=map.describeInflateOccupancy({0.,0.,.5},0.);
  EXPECT_NE(detail.find("never_observed=1"),std::string::npos);
  EXPECT_NE(detail.find("observed_insufficient=1"),std::string::npos);
  EXPECT_NE(detail.find("stale_free=1"),std::string::npos);
  EXPECT_NE(detail.find("front_source_ns=100000000000"),std::string::npos);
  EXPECT_NE(detail.find("ray_witness=disabled"),std::string::npos);
  map.nearFieldDiagnostics().configure(true,{0.,0.,.5},{1.,1.,1.},.05);
  Eigen::Vector3i occupied;map.posToIndex({.025,.025,.70},occupied);
  RayWitness witness;witness.hit=witness.contributed_vote=true;witness.metadata.sensor_id=1;
  witness.metadata.scan_stamp_ns=100000000000LL;witness.metadata.acquisition_end_ns=100050000000LL;
  witness.origin={-.4,0.,.5};witness.endpoint={.025,.025,.70};
  map.nearFieldDiagnostics().record(occupied,witness);
  const auto provenance=map.describeInflateOccupancy({0.,0.,.5},0.);
  EXPECT_NE(provenance.find("ray_witness={sensor:1,hit:1,vote:1,scan_ns:100000000000"),std::string::npos);
  EXPECT_NE(provenance.find("origin:[-0.4,0,0.5]"),std::string::npos);
  map.beginObservedProof();EXPECT_NE(map.getInflateOccupancy({0.,0.,.5},0.),0);
  // Diagnostic occupied has decision precedence for reporting only; native
  // first-hit policy and all unknown-space blocking remain unchanged.
}
TEST(MotionSweep,NewBodyOrNewProofDoesNotRenewOriginalDemandBodyOrExpiredRaySource) {
  constexpr std::int64_t t=100000000000LL;
  EXPECT_TRUE(motionSourceFresh(t,t+99999999LL));EXPECT_FALSE(motionSourceFresh(t,t+100000001LL));
  EXPECT_FALSE(motionSourceFresh(t+1,t));EXPECT_FALSE(motionSourceFresh(0,t));
  auto deadline=motionEvidenceDeadline(t,t-50000000LL,t+20000000LL,t+100000000LL,
    t+250000000LL,t+250000000LL,t+500000000LL,t,t);
  EXPECT_EQ(deadline,t+50000000LL); // Old demand-body evidence, not newest body.
  deadline=motionEvidenceDeadline(t,t,t,t+100000000LL,t+250000000LL,t+250000000LL,
    t+10000000LL,t-490000000LL,t);
  EXPECT_EQ(deadline,t+10000000LL);
  EXPECT_FALSE(deadline>t+10000000LL); // Crossing TTL during computation cannot commit.
}
TEST(MotionSweep,FreeCurrentBodyAndShortNominalCurveDoNotProveBrakingDistance) {
  GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::voxel(m,{.5,.025,.55},2.);
  m.beginObservedProof();EXPECT_EQ(m.getInflateOccupancy({0,0,.55},0.),0);
  EXPECT_EQ(m.getInflateOccupancy({.02,0,.55},0.),0);
  geometry_msgs::msg::Twist command,actual;command.linear.x=.05;actual.linear.x=.3;
  auto out=run(m,command,actual);EXPECT_FALSE(out.valid);EXPECT_EQ(out.reason,"motion_sweep_occupied");
}
TEST(MotionSweep,PlanningCorridorIncludesFutureBrakingAndDoesNotReplaceCommandProof) {
  GridMap m;GridMapTestAccess::configure(m);MotionSupport support(ground());
  const Eigen::Vector3d begin(0.,0.,.55),target(.25,0.,.55);
  ASSERT_TRUE(validateStraightBrakingCorridor(m,support,model(),begin,target,"odom").valid);
  // Past the target's pose footprint, still inside its future stop volume.
  GridMapTestAccess::voxel(m,{.70,.025,.55},2.);
  m.beginObservedProof();ASSERT_EQ(m.getInflateOccupancy(target,0.),0);
  auto future=validateStraightBrakingCorridor(m,support,model(),begin,target,"odom");
  EXPECT_FALSE(future.valid);EXPECT_EQ(future.reason,"motion_sweep_occupied");
  geometry_msgs::msg::Twist command;command.linear.x=.3;
  EXPECT_TRUE(run(m,command).valid); // Current body has not reached that obstacle.
}
TEST(MotionSweep,TurningSweepCoversCylinderRotationAwayFromFreeCurrentPose) {
  GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::voxel(m,{.275,.20,.55},2.);
  m.beginObservedProof();ASSERT_EQ(m.getInflateOccupancy({0,0,.55},0.),0);
  geometry_msgs::msg::Twist v;v.angular.z=.5;const auto out=run(m,v);
  EXPECT_FALSE(out.valid);EXPECT_EQ(out.reason,"motion_sweep_occupied");
}
TEST(MotionSweep,NeverObservedAndInsufficientEvidenceAreNotAssumedFree) {
  for(double evidence:{-1.01,0.}) {
    GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::voxel(m,{.5,.025,.55},evidence);
    geometry_msgs::msg::Twist v;v.linear.x=.3;auto out=run(m,v);
    EXPECT_FALSE(out.valid);EXPECT_EQ(out.reason,"motion_sweep_unknown_or_expired");
  }
}
TEST(MotionSweep,TTLExpiryDoesNotClearOccupancyOrRenewFreeEvidence) {
  GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::expire(m);
  geometry_msgs::msg::Twist v;v.linear.x=.2;auto out=run(m,v);EXPECT_FALSE(out.valid);
  EXPECT_EQ(out.reason,"motion_sweep_unknown_or_expired");
}
TEST(MotionSweep,SourceGroundGapWrongFloorOrMissingBodySupportRejects) {
  GridMap m;GridMapTestAccess::configure(m);geometry_msgs::msg::Twist v;v.linear.x=.3;
  for(int which=0;which<3;++which) {
    auto g=ground();if(which==0)g.floor_id="floor2";
    if(which==1)for(auto& p:g.support_ground_xyz)p.z+=3.;
    if(which==2)g.support_ground_xyz.erase(std::remove_if(g.support_ground_xyz.begin(),g.support_ground_xyz.end(),
      [](const auto& p){return p.x>.06;}),g.support_ground_xyz.end());
    MotionSupport s(g);auto out=validateMotionSweep(m,s,model(),{0,0,.55},0.,{},v,"odom");EXPECT_FALSE(out.valid);
  }
}
// These use the deployed geometric envelope and the actual native swept-volume
// query. Only initial raw odds/free timestamps are injected; raw projection is
// covered separately by plan_env/test_projected_rays, not claimed by this test.
TEST(MotionSweep,DeployedEnvelopeRejectsLowObstacleInsideBrakingSweep) {
  GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::deployedEnvelope(m);
  const auto shape=m.collisionShape();ASSERT_DOUBLE_EQ(shape.radius,.29);
  ASSERT_DOUBLE_EQ(shape.offset,.20);ASSERT_DOUBLE_EQ(shape.up,.45);ASSERT_DOUBLE_EQ(shape.down,.45);
  geometry_msgs::msg::Twist command,actual;command.linear.x=.05;actual.linear.x=.3;
  const auto clear=run(m,command,actual);ASSERT_TRUE(clear.valid)<<clear.reason;
  // The obstacle is outside the current body, but inside reaction + stopping
  // reach. Its 10--15 cm voxel intersects the protected lower body envelope.
  GridMapTestAccess::voxel(m,{.65,.025,.12},2.);
  m.beginObservedProof();ASSERT_EQ(m.getInflateOccupancy({0,0,.55},0.),0);
  const auto blocked=run(m,command,actual);
  EXPECT_FALSE(blocked.valid);EXPECT_EQ(blocked.reason,"motion_sweep_occupied");
  EXPECT_LT(blocked.min_z,.12);EXPECT_GT(blocked.max_z,.12);
}
TEST(MotionSweep,DeployedEnvelopeDoesNotClaimContactBelowOrAboveProtectedHeight) {
  geometry_msgs::msg::Twist v;v.linear.x=.3;
  for(double height:{.025,1.225}) {
    GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::deployedEnvelope(m);
    GridMapTestAccess::voxel(m,{.65,.025,height},2.);
    const auto result=run(m,v);EXPECT_TRUE(result.valid)<<result.reason;
    // The occupied voxel is wholly outside this geometric envelope. This is
    // NOT proof that a physical wheel/foot can safely traverse a 2.5 cm object.
    if(height<.55) EXPECT_LT(.05,result.min_z);
    else EXPECT_GT(1.20,result.max_z);
  }
}
TEST(MotionSweep,DeployedEnvelopeRejectsLowUnknownAndInsufficientEvidence) {
  geometry_msgs::msg::Twist v;v.linear.x=.3;
  for(double odds:{-1.01,0.}) {
    GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::deployedEnvelope(m);
    GridMapTestAccess::voxel(m,{.65,.025,.12},odds);
    const auto result=run(m,v);EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.reason,"motion_sweep_unknown_or_expired");
  }
}
TEST(MotionSweep,DeployedEnvelopeRejectsMissingSupportEvenWhenAllRawVoxelsAreFree) {
  GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::deployedEnvelope(m);
  geometry_msgs::msg::Twist v;v.linear.x=.3;
  ASSERT_TRUE(run(m,v).valid);
  auto g=ground();g.support_ground_xyz.erase(std::remove_if(g.support_ground_xyz.begin(),g.support_ground_xyz.end(),
    [](const auto& p){return p.x>.06;}),g.support_ground_xyz.end());
  MotionSupport support(g);const auto result=validateMotionSweep(m,support,model(),{0,0,.55},0.,{},v,"odom");
  EXPECT_FALSE(result.valid);EXPECT_EQ(result.reason,"braking_sweep_support_unverified");
}
TEST(MotionSweep,EitherRawRaySourceIndependentlyCapsCommandEvidenceDeadline) {
  constexpr std::int64_t t=100000000000LL;
  const auto deadline=[&](std::int64_t front,std::int64_t rear) {
    return motionEvidenceDeadline(t,t,t,t+100000000LL,t+250000000LL,t+250000000LL,
      t+500000000LL,front,rear);
  };
  EXPECT_EQ(deadline(t,t),t+100000000LL);
  EXPECT_EQ(deadline(t,t-490000000LL),t+10000000LL);
  EXPECT_EQ(deadline(t-490000000LL,t),t+10000000LL);
  EXPECT_LE(deadline(t,t-500000001LL),t); // Fresh front cannot renew stale rear.
  EXPECT_LE(deadline(t-500000001LL,t),t); // Nor can a fresh rear renew stale front.
  EXPECT_LE(deadline(t,0),t);EXPECT_LE(deadline(0,t),t);
}
TEST(MotionSweep,BoundsBudgetAndNonForwardCommandFailClosed) {
  GridMap m;GridMapTestAccess::configure(m);geometry_msgs::msg::Twist v;v.linear.x=.3;
  auto bad=model();bad.heading_error=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(run(m,v,{},bad).valid);
  EXPECT_FALSE(run(m,v,{},model(),1e-12).valid);
  v.linear.x=-.01;EXPECT_FALSE(run(m,v).valid);v.linear.x=.31;EXPECT_FALSE(run(m,v).valid);
  v.linear.x=.1;v.linear.y=.01;EXPECT_FALSE(run(m,v).valid);
}
TEST(MotionSweep,ExactFileBytesAndTransportAreBoundToMeasuredModel) {
  nlohmann::json j={{"schema_version",3},{"transport_mode","isolated_mock"},{"fixture_only",true},
    {"model","reaction_braking_reachable_v1"},
    {"execution_timing",{{"sensor_source_age_bound_s",.2},{"command_pipeline_bound_s",.1},
      {"writer_period_s",.05},{"source_time_uncertainty_s",.02}}},
    {"measurements",{{"max_speed_mps",.3},{"max_yaw_radps",.5},
    {"reaction_bound_s",.4},{"stopping_distance_m",.08},{"stopping_yaw_rad",.2},{"stop_latency_bound_s",.5},
    {"tracking_error_bound_m",.01},{"heading_error_bound_rad",.03}}}};
  const auto path=std::filesystem::temp_directory_path()/
    ("d1max_braking_model_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".json");
  auto bytes=j.dump();{std::ofstream f(path);f<<bytes;}
  const auto loaded=BrakingModel::load(path.string(),digest(bytes),"isolated_mock");
  EXPECT_TRUE(loaded.valid());EXPECT_EQ(loaded.raySourceAgeNs(),200000000LL);
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"live"),std::invalid_argument);
  {std::ofstream f(path);f<<bytes<<'\n';}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::invalid_argument);
  auto incomplete=j;incomplete.erase("execution_timing");bytes=incomplete.dump();{std::ofstream f(path);f<<bytes;}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::exception);
  auto underbounded=j;underbounded["execution_timing"]["sensor_source_age_bound_s"]=.5;
  bytes=underbounded.dump();{std::ofstream f(path);f<<bytes;}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::invalid_argument);
  j["measurements"].erase("heading_error_bound_rad");bytes=j.dump();{std::ofstream f(path);f<<bytes;}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::exception);
  std::filesystem::remove(path);
}
TEST(MotionSweep,RecordSourceAgeOverridesLegacyDefaultAndCannotRenewEitherSensor) {
  const auto t=100000000000LL;
  EXPECT_EQ(motionEvidenceDeadline(t,t,t,t+100000000LL,t+250000000LL,t+250000000LL,
    t+1000000000LL,t-150000000LL,t,200000000LL),t+50000000LL);
  EXPECT_EQ(motionEvidenceDeadline(t,t,t,t+100000000LL,t+250000000LL,t+250000000LL,
    t+1000000000LL,t,t-150000000LL,200000000LL),t+50000000LL);
  EXPECT_EQ(motionEvidenceDeadline(t,t,t,t+100000000LL,t+250000000LL,t+250000000LL,
    t+1000000000LL,t,t,0LL),0LL);
}
