#include <gtest/gtest.h>
#include <plan_manage/motion_sweep.hpp>
#include <plan_manage/moving_splice.hpp>
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
    p.planning_collision_policy_="observed_free"; // explicit legacy comparison
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
  static void official(GridMap& m) {
    m.mp_.planning_collision_policy_="official";m.mp_.require_observed_free_=false;
    m.rebuildInflationOffsets();
  }
  static void omitFreeStamps(GridMap& m) {m.free_observation_stamps_.clear();}
  static void absentRear(GridMap& m) {m.ray_integrated_stamps_[1]=0;}
  static const auto& raw(const GridMap& m) {return m.md_.occupancy_buffer_;}
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
  static void orientedBox(GridMap& m,double length=.94,double width=.50) {
    m.mp_.collision_model_="oriented_box";
    m.mp_.box_length_=length;m.mp_.box_width_=width;
    m.mp_.body_below_=m.mp_.body_above_=.25;
    m.mp_.planning_body_below_=m.mp_.planning_body_above_=.25;
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
BrakingModel holonomicModel() {
  auto out=model();out.kinematic_profile=KinematicProfile::HolonomicXYYaw;out.lateral_distance=.08;return out;
}
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
namespace {
struct MovingSpliceFixture {
  UniformBspline old,tail;PlannedTrajectorySplice splice;MeasuredBodyPose acquired;
  SplineHeadingContract heading;
  MovingSpliceFixture() {
    Eigen::MatrixXd points(3,23),next(3,23);
    for(int i=0;i<23;++i) {points.col(i)=Eigen::Vector3d((i-1)*.05,0.,.55);
      next.col(i)=points.col(i)+Eigen::Vector3d(.2,0.,0.);}
    old=UniformBspline(points,3,.2);tail=UniformBspline(next,3,.2);
    heading.measured_heading_enabled=true;heading.policy=BodyHeadingPolicy::HoldHeading;
    heading.measured_yaw=0.;heading.body_extent=.53;
    splice.incumbent=old;splice.heading=heading;splice.measured_time=.4;splice.entry_yaw=0.;
    auto& w=splice.wire;w.enabled=true;w.splice_curve_time=.8;w.splice_arc_length=.2;
    w.position.x=.2;w.position.z=.55;w.velocity.x=.25;w.acceleration_valid=true;
    acquired={old.evaluateDeBoorT(.4),Eigen::Quaterniond::Identity(),100.01,"odom"};
  }
  bool run(GridMap& m,CurveCheckCompletion* completion=nullptr,double now=100.01) {
    m.beginObservedProof();return commonPrefixTailCollisionFree(tail,splice,.05,.53,
      [&](const Eigen::Vector3d& p,double yaw){return m.getPlanningInflateOccupancy(p,yaw);},
      acquired,"odom",now,.1,.2,heading,completion);
  }
};
}
TEST(MovingSplice,ProductionNativeMapProvesMovingPublicPrefixAndFutureTailWithoutFakeBody) {
  GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::orientedBox(m,.93,.30);
  MovingSpliceFixture f;const auto before=f.acquired;ASSERT_TRUE(f.run(m));
  EXPECT_TRUE(f.acquired.position.isApprox(before.position));EXPECT_DOUBLE_EQ(f.acquired.source_stamp,before.source_stamp);
  EXPECT_GT((f.tail.evaluateDeBoorT(0.)-f.acquired.position).norm(),.0125);
  // Tail alone cannot borrow the remote current robot's measured admission.
  EXPECT_FALSE(wholeCurveCollisionFree(f.tail,.05,.53,[&](const Eigen::Vector3d& p,double yaw){
      return m.getPlanningInflateOccupancy(p,yaw);},f.acquired,"odom",100.01,.1,200000,.2));
}
TEST(MovingSplice,ProductionMapRejectsOccupiedPrefixOrTailAndNeverErasesUnknown) {
  for(int kind=0;kind<3;++kind) {
    GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::orientedBox(m,.93,.30);
    MovingSpliceFixture f;
    GridMapTestAccess::voxel(m,{kind==1?.9:.55,0.,.55},kind==2?-1.01:2.);
    CurveCheckCompletion completion=CurveCheckCompletion::Uncertified;
    EXPECT_FALSE(f.run(m,&completion))<<kind;
    if(kind<2)EXPECT_EQ(completion,CurveCheckCompletion::Blocked);
  }
}
TEST(MovingSplice,ActualProgressMayAdvanceAlongTailWithoutWideningLateralJoin) {
  for(double over:{.005,.03,.06}) {
    GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::orientedBox(m,.93,.30);
    MovingSpliceFixture f;f.splice.measured_time=.8+over/.25;
    f.acquired.position=f.old.evaluateDeBoorT(f.splice.measured_time);
    EXPECT_EQ(f.run(m),over<=.05)<<over;
  }
}
TEST(MovingSplice,NewBodyAheadOfProgressCannotHideMissedBoundaryInOldConnector) {
  using namespace scan_planner;
  for(double over:{.005,.03,.06}) {
    GridMap m;GridMapTestAccess::configure(m);MovingSpliceFixture f;
    // Last progress is at the boundary, newer acquired body has passed it.
    // There is no permission to clamp the actual projection back to .8 s.
    f.splice.measured_time=.8;f.splice.committed_arc=.2;
    f.acquired.position=f.splice.incumbent.evaluateDeBoorT(.8+over/.25);
    f.acquired.source_stamp=100.1;f.splice.progress_source_ns=100000000000LL;
    EXPECT_EQ(f.run(m,nullptr,100.1),over<=.05)<<over;
  }
}
TEST(MovingSplice,TailProjectionKeepsLateralSourceAndObstacleBounds) {
  for(int bad=0;bad<4;++bad) {
    GridMap m;GridMapTestAccess::configure(m);MovingSpliceFixture f;
    f.splice.measured_time=.92;f.acquired.position=f.old.evaluateDeBoorT(.92);
    if(bad==0)f.acquired.position.y()=.02;
    if(bad==1)f.splice.progress_source_ns=100500000000LL;
    if(bad==2)GridMapTestAccess::voxel(m,{.9,0.,.55},2.);
    if(bad==3)f.acquired.source_stamp=99.;
    EXPECT_FALSE(f.run(m))<<bad;
  }
}
TEST(MovingSplice,BoundaryDerivativeMismatchAndExpiredActualPoseAreRejectedWithoutRelaxedJoin) {
  for(int bad=0;bad<4;++bad) {
    GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::orientedBox(m,.93,.30);
    MovingSpliceFixture f;
    if(bad==0)f.splice.wire.velocity.x=.20;
    if(bad==1)f.splice.wire.position.y=.001;
    if(bad==2)f.splice.wire.acceleration.x=.1;
    if(bad==3)f.acquired.source_stamp=99.;
    EXPECT_FALSE(f.run(m))<<bad;
  }
}
TEST(MovingSplice,DiagnosticsDistinguishSourceOrderBoundaryAndRealCollisionWithoutChangingAdmission) {
 for(int kind=0;kind<3;++kind) {
   GridMap m;GridMapTestAccess::configure(m);GridMapTestAccess::orientedBox(m,.93,.30);
   MovingSpliceFixture f;
   if(kind==0)f.splice.progress_source_ns=100110000000LL;
   if(kind==1)f.splice.wire.velocity.x=.2;
   if(kind==2)GridMapTestAccess::voxel(m,{.55,0.,.55},2.);
   CurveCheckCompletion completion=CurveCheckCompletion::Uncertified;SpliceCheckDiagnostics diagnostic;
   EXPECT_FALSE(commonPrefixTailCollisionFree(f.tail,f.splice,.05,.53,
     [&](const Eigen::Vector3d& p,double yaw){return m.getPlanningInflateOccupancy(p,yaw);},
     f.acquired,"odom",100.11,.4,.2,f.heading,&completion,&diagnostic));
   EXPECT_STREQ(diagnostic.phase,kind==0?"progress_after_body":kind==1?"boundary_mismatch":"prefix_curve");
   EXPECT_STREQ(curveCheckCompletionName(completion),kind==2?"blocked":"invalid_input");
   if(kind==0)EXPECT_NEAR(diagnostic.source_delta_ms,-100.,1e-5);
 }
}
TEST(MotionSweep,VerifiedKnownGeneralFloorsAreSupportedButStairsAndUnknownFloorsRemainDistinct) {
  auto s=ground();s.segment_kind="landing";EXPECT_TRUE(MotionSupport(s).valid);
  s.floor_id="floor2";EXPECT_TRUE(MotionSupport(s).valid);
  for(auto& p:s.support_ground_xyz)p.z=3.;
  MotionSupport upper(s);ASSERT_TRUE(upper.valid);ASSERT_TRUE(upper.nearest(0.,0.));
  EXPECT_DOUBLE_EQ(upper.nearest(0.,0.)->z,3.);
  s.floor_id="floor3";EXPECT_FALSE(MotionSupport(s).valid);
  s.floor_id="floor1";s.segment_kind="stairs";EXPECT_FALSE(MotionSupport(s).valid);
  s.segment_kind="floor";s.required_mode="stair";EXPECT_FALSE(MotionSupport(s).valid);s.required_mode="general";
  s.segment_kind="landing";s.verified=false;EXPECT_FALSE(MotionSupport(s).valid);
}
TEST(MotionSweep,HolonomicRequiresExplicitMeasuredLateralRecordAndNormSpeedLimit) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::orientedBox(map);
  auto missing=model();missing.kinematic_profile=KinematicProfile::HolonomicXYYaw;
  geometry_msgs::msg::Twist command;command.linear.y=.3;
  EXPECT_FALSE(missing.valid());EXPECT_FALSE(run(map,command,{},missing,.1).valid);
  EXPECT_EQ(run(map,command,{},model(),.1).reason,"command_or_measured_speed_outside_record");
  for(const auto& xy:{Eigen::Vector2d(0.,.3),Eigen::Vector2d(0.,-.3),Eigen::Vector2d(-.3,0.),
                     Eigen::Vector2d(.18,.24),Eigen::Vector2d(-.18,-.24)}) {
    command.linear.x=xy.x();command.linear.y=xy.y();command.angular.z=.4;
    EXPECT_TRUE(run(map,command,{},holonomicModel(),.1).valid)<<xy.transpose();
  }
  command.linear.x=.22;command.linear.y=.22;
  EXPECT_EQ(run(map,command,{},holonomicModel(),.1).reason,"command_or_measured_speed_outside_record");
  command.linear.x=0.;command.linear.y=.2;command.linear.z=.001;
  EXPECT_EQ(run(map,command,{},holonomicModel(),.1).reason,"command_or_measured_speed_outside_record");
  command.linear.z=0.;command.angular.z=.51;
  EXPECT_EQ(run(map,command,{},holonomicModel(),.1).reason,"command_or_measured_speed_outside_record");
  command.angular.z=0.;geometry_msgs::msg::Twist measured;measured.linear.x=.22;measured.linear.y=.22;
  EXPECT_EQ(run(map,command,measured,holonomicModel(),.1).reason,"command_or_measured_speed_outside_record");
}
TEST(MotionSweep,HolonomicReachBoundsContainSignedXYRotatingReactionAndBraking) {
  const auto braking=holonomicModel();
  for(double yaw_bound:{.03,.43,1.2,2.})for(const auto& v:{Eigen::Vector2d(.3,0.),
      Eigen::Vector2d(0.,-.3),Eigen::Vector2d(-.18,.24)}) {
    const auto bounds=holonomicReachableBounds(braking,v.x(),v.y(),-v.x(),-v.y(),yaw_bound);
    for(int i=-100;i<=100;++i) {
      const double angle=yaw_bound*i/100.;Eigen::Rotation2Dd rotation(angle);
      for(double direction:{-1.,1.})for(double bx:{-braking.distance,braking.distance})
        for(double by:{-braking.lateral_distance,braking.lateral_distance}) {
          const Eigen::Vector2d p=rotation*(direction*v*braking.reaction+Eigen::Vector2d(bx,by));
          EXPECT_GE(p.x(),bounds.xmin);EXPECT_LE(p.x(),bounds.xmax);
          EXPECT_GE(p.y(),bounds.ymin);EXPECT_LE(p.y(),bounds.ymax);
        }
    }
  }
}
TEST(MotionSweep,HolonomicSweepIncludesLateralBackwardAndRotatedBodyBrakingObstacle) {
  for(double body_yaw:{0.,1.570796326794897})for(bool reverse:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::orientedBox(map);
    geometry_msgs::msg::Twist command,actual;
    if(reverse) {command.linear.x=-.05;actual.linear.x=-.3*std::cos(body_yaw);actual.linear.y=-.3*std::sin(body_yaw);}
    else {command.linear.y=.05;actual.linear.x=-.3*std::sin(body_yaw);actual.linear.y=.3*std::cos(body_yaw);}
    const MotionSupport support(ground());
    const auto clear=validateMotionSweep(map,support,holonomicModel(),{0.,0.,.55},body_yaw,
        actual,command,"odom",.1);ASSERT_TRUE(clear.valid)<<clear.reason;
    const Eigen::Vector2d local=reverse?Eigen::Vector2d(-.60,.025):Eigen::Vector2d(.025,.425);
    const Eigen::Vector2d obstacle=Eigen::Rotation2Dd(body_yaw)*local;
    GridMapTestAccess::voxel(map,{obstacle.x(),obstacle.y(),.55},2.);
    map.beginObservedProof();ASSERT_EQ(map.getInflateOccupancy({0.,0.,.55},body_yaw),0);
    const auto blocked=validateMotionSweep(map,support,holonomicModel(),{0.,0.,.55},body_yaw,
        actual,command,"odom",.1);
    EXPECT_FALSE(blocked.valid);EXPECT_EQ(blocked.reason,"motion_sweep_occupied");
  }
}
TEST(MotionSweep,HolonomicCannotClearUnknownRenewRearOrIgnoreLateralSupportGap) {
  geometry_msgs::msg::Twist command;command.linear.y=.3;
  for(int failure=0;failure<3;++failure) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::orientedBox(map);
    auto original=ground();
    if(failure==0)GridMapTestAccess::voxel(map,{.025,.425,.55},-1.01);
    if(failure==1)GridMapTestAccess::absentRear(map);
    if(failure==2)original.support_ground_xyz.erase(std::remove_if(original.support_ground_xyz.begin(),
      original.support_ground_xyz.end(),[](const auto& p){return p.y>.06;}),original.support_ground_xyz.end());
    const auto raw=GridMapTestAccess::raw(map);const MotionSupport support(original);
    const auto result=validateMotionSweep(map,support,holonomicModel(),{0.,0.,.55},0.,{},command,"odom",.1);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.reason,failure==0?"motion_sweep_unknown_or_expired":
        failure==1?"motion_sensor_source_expired":"braking_sweep_support_unverified");
    EXPECT_EQ(GridMapTestAccess::raw(map),raw);
  }
}
TEST(MotionSweep,PureLateralIsNotZeroAndLegacyProfileParsingCannotGrantIt) {
  geometry_msgs::msg::Twist demand;EXPECT_TRUE(planarDemandIsZero(demand));
  demand.linear.y=.01;EXPECT_FALSE(planarDemandIsZero(demand));
  demand.linear.y=-.01;EXPECT_FALSE(planarDemandIsZero(demand));
  EXPECT_EQ(parseKinematicProfile("forward_yaw"),KinematicProfile::ForwardYaw);
  EXPECT_EQ(parseKinematicProfile("holonomic_xyyaw"),KinematicProfile::HolonomicXYYaw);
  EXPECT_THROW(parseKinematicProfile("omni"),std::invalid_argument);
}
TEST(MotionSweep,OrientedBoxCornerIsCheckedByNativePoseAndRawBrakingEnvelope) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::orientedBox(map,.93,.30);
  const auto shape=map.bodyEnvelope();
  ASSERT_EQ(shape.model,CollisionModel::OrientedBox);
  EXPECT_DOUBLE_EQ(shape.length,.93);EXPECT_DOUBLE_EQ(shape.width,.30);
  EXPECT_NEAR(bodyExtent(shape),std::hypot(.465,.15),1e-12);
  geometry_msgs::msg::Twist command;command.linear.x=.05;
  EXPECT_TRUE(run(map,command).valid);
  // This actual rectangular corner is outside the old fixture's cylinders.
  GridMapTestAccess::voxel(map,{.45,.125,.55},2.);
  map.beginObservedProof();EXPECT_EQ(map.getInflateOccupancy({0.,0.,.55},0.),1);
  EXPECT_EQ(map.getPlanningInflateOccupancy({0.,0.,.55},0.),1);
  const auto blocked=run(map,command);
  EXPECT_FALSE(blocked.valid);EXPECT_EQ(blocked.reason,"motion_sweep_occupied");
}
TEST(MotionSweep,OrientedBoxTurningCoversIntermediateYawCornerBeyondCurrentFootprint) {
  for(double yaw:{0.,1.570796326794897}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::orientedBox(map);
    const double c=std::cos(yaw),s=std::sin(yaw);
    GridMapTestAccess::voxel(map,{c*.325-s*.40,s*.325+c*.40,.55},2.);
    map.beginObservedProof();ASSERT_EQ(map.getInflateOccupancy({0.,0.,.55},yaw),0);
    geometry_msgs::msg::Twist command;command.angular.z=.5;
    const MotionSupport support(ground());
    const auto sweep=validateMotionSweep(map,support,model(),{0.,0.,.55},yaw,{},command,"odom",.1);
    EXPECT_FALSE(sweep.valid);EXPECT_EQ(sweep.reason,"motion_sweep_occupied");
    EXPECT_NEAR(sweep.heading_bound,.43,1e-12);
  }
}
TEST(MotionSweep,OrientedBoxSweepRetainsStrictUnknownAndSourceExpiryBlocking) {
  for(bool expired:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::orientedBox(map);
    if(expired)GridMapTestAccess::expire(map);
    else GridMapTestAccess::voxel(map,{.45,.225,.55},-1.01);
    geometry_msgs::msg::Twist command;command.linear.x=.05;
    const auto sweep=run(map,command,{},model(),.1);
    EXPECT_FALSE(sweep.valid);
    EXPECT_EQ(sweep.reason,expired?"motion_sensor_source_expired":"motion_sweep_unknown_or_expired");
    if(!expired) {EXPECT_GT(sweep.state_counts[2],0U);EXPECT_TRUE(sweep.query_complete);}
  }
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
  // Native and diagnostic occupied now share precedence; the set of blocked
  // cells, original evidence timestamps and unknown-space blocking are unchanged.
}
TEST(MotionSweep,EarlierUnknownCannotHideRealObstacleAndSuppressLocalRecovery) {
  for(double uncertain:{-1.01,0.}) {
    GridMap map;GridMapTestAccess::configure(map);
    // Production sweep order visits negative X before the forward obstacle.
    GridMapTestAccess::voxel(map,{-.125,.025,.55},uncertain);
    GridMapTestAccess::voxel(map,{.50,.025,.55},2.);
    geometry_msgs::msg::Twist command;command.linear.x=.3;
    const auto result=run(map,command);
    EXPECT_FALSE(result.valid);EXPECT_EQ(result.reason,"motion_sweep_occupied");
    EXPECT_GE(result.state_counts[2],1U);EXPECT_EQ(result.state_counts[1],1U);
    EXPECT_FALSE(result.query_complete);
    EXPECT_EQ(result.unique_voxels,result.state_counts[0]+result.state_counts[1]+
        result.state_counts[2]+result.state_counts[3]);
  }
}
TEST(MotionSweep,PureUnknownStillBlocksAndCountsOnlyActuallyCheckedVolume) {
  GridMap map;GridMapTestAccess::configure(map);
  GridMapTestAccess::voxel(map,{-.125,.025,.55},-1.01);
  geometry_msgs::msg::Twist command;command.linear.x=.3;
  const auto result=run(map,command);
  EXPECT_FALSE(result.valid);EXPECT_EQ(result.reason,"motion_sweep_unknown_or_expired");
  EXPECT_TRUE(result.query_complete);EXPECT_EQ(result.state_counts[1],0U);
  EXPECT_EQ(result.state_counts[2],1U);EXPECT_GT(result.state_counts[0],0U);
  EXPECT_EQ(result.unique_voxels,result.state_counts[0]+result.state_counts[2]);
}
TEST(MotionSweep,RawUnknownFrontDoesNotHideOccupiedRearCylinder) {
  GridMap map;GridMapTestAccess::configure(map);
  GridMapTestAccess::voxel(map,{.225,.025,.55},-1.01);
  GridMapTestAccess::voxel(map,{-.225,.025,.55},2.);
  map.beginObservedProof();EXPECT_EQ(map.getInflateOccupancy({0.,0.,.55},0.),1);
  const auto evidence=map.inspectInflateOccupancy({0.,0.,.55},0.,true);
  EXPECT_EQ(evidence.state(),1);EXPECT_EQ(evidence.unique_counts[1],1U);
  EXPECT_EQ(evidence.unique_counts[2],1U);
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
TEST(MotionSweep,HeldHeadingPlanningCorridorExtendsLateralNotBodyForwardAndProvesNoCommand) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::orientedBox(map,.94,.30);
  const MotionSupport support(ground());
  const Eigen::Vector3d body(0.,0.,.55),target(0.,.25,.55);
  ASSERT_TRUE(validateStraightBrakingCorridor(map,support,holonomicModel(),body,target,"odom",.1,0.).valid);
  GridMapTestAccess::voxel(map,{.025,.725,.55},2.);
  map.beginObservedProof();EXPECT_EQ(map.getInflateOccupancy(body,0.),0);
  map.beginObservedProof();EXPECT_EQ(map.getInflateOccupancy(target,0.),0);
  const auto corridor=validateStraightBrakingCorridor(map,support,holonomicModel(),body,target,"odom",.1,0.);
  EXPECT_FALSE(corridor.valid);EXPECT_EQ(corridor.reason,"motion_sweep_occupied");
  geometry_msgs::msg::Twist command;command.linear.y=.3;
  EXPECT_TRUE(run(map,command,{},holonomicModel(),.1).valid);
  EXPECT_EQ(validateStraightBrakingCorridor(map,support,model(),body,target,"odom",.1,0.).reason,
      "planning_heading_profile_mismatch");
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
  EXPECT_EQ(out.reason,"motion_sensor_source_expired");
}
TEST(MotionSweep,OfficialUsesOriginalRawMapWithoutFreeStampsOrUnknownVeto) {
  for(double odds:{-1.01,0.}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::orientedBox(map,.93,.30);
    GridMapTestAccess::official(map);GridMapTestAccess::omitFreeStamps(map);
    GridMapTestAccess::voxel(map,{.45,.125,.55},odds);
    const auto before=GridMapTestAccess::raw(map);
    geometry_msgs::msg::Twist command;command.linear.x=.1;
    const auto out=run(map,command,{},model(),.1);
    ASSERT_TRUE(out.valid)<<out.reason;
    EXPECT_EQ(out.reason,"official_occupied_motion_sweep");
    EXPECT_EQ(out.completion,MotionSweepCompletion::Clear);
    EXPECT_EQ(out.state_counts[2],1U);EXPECT_TRUE(out.query_complete);
    EXPECT_EQ(GridMapTestAccess::raw(map),before); // Never manufacture observed FREE.
    EXPECT_EQ(map.observedProofDeadlineNs(),std::numeric_limits<std::int64_t>::max());
  }
}
TEST(MotionSweep,OfficialUnknownCannotHideOccupiedOrInvalidNumericMapEvidence) {
  for(double obstacle:{2.,std::numeric_limits<double>::quiet_NaN()}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::orientedBox(map,.93,.30);
    GridMapTestAccess::official(map);GridMapTestAccess::omitFreeStamps(map);
    GridMapTestAccess::voxel(map,{-.425,.025,.55},-1.01);
    GridMapTestAccess::voxel(map,{.45,.125,.55},obstacle);
    geometry_msgs::msg::Twist command;command.linear.x=.1;
    const auto out=run(map,command,{},model(),.1);
    EXPECT_FALSE(out.valid);
    EXPECT_EQ(out.reason,std::isfinite(obstacle)?"motion_sweep_occupied":"motion_sweep_unknown_or_expired");
    EXPECT_EQ(out.completion,std::isfinite(obstacle)?MotionSweepCompletion::Blocked:MotionSweepCompletion::Uncertified);
    EXPECT_GT(out.state_counts[2],0U);
  }
}
TEST(MotionSweep,OfficialStillRejectsOBBRotationCornerAndMapExterior) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::orientedBox(map);
  GridMapTestAccess::official(map);GridMapTestAccess::omitFreeStamps(map);
  GridMapTestAccess::voxel(map,{.325,.40,.55},2.);
  geometry_msgs::msg::Twist command;command.angular.z=.5;
  EXPECT_EQ(run(map,command,{},model(),.1).reason,"motion_sweep_occupied");
  GridMap edge;GridMapTestAccess::configure(edge);GridMapTestAccess::orientedBox(edge);
  GridMapTestAccess::official(edge);GridMapTestAccess::omitFreeStamps(edge);
  auto extended=ground();extended.support_ground_xyz.clear();
  for(int x=-100;x<=100;++x)for(int y=-20;y<=20;++y) {
    geometry_msgs::msg::Point p;p.x=x*.025;p.y=y*.025;extended.support_ground_xyz.push_back(p);
  }
  const MotionSupport support(extended);
  EXPECT_EQ(validateMotionSweep(edge,support,model(),{1.5,0.,.55},0.,{},command,"odom",.1).reason,
      "motion_sweep_outside_map");
}
TEST(MotionSweep,OfficialDoesNotRenewExpiredOrAbsentSensorSources) {
  for(bool absent:{false,true}) {
    GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::official(map);
    GridMapTestAccess::omitFreeStamps(map);
    if(absent)GridMapTestAccess::absentRear(map);else GridMapTestAccess::expire(map);
    const auto before=GridMapTestAccess::raw(map);
    geometry_msgs::msg::Twist command;command.linear.x=.1;
    const auto out=run(map,command,{},model(),.1);
    EXPECT_FALSE(out.valid);EXPECT_EQ(out.reason,"motion_sensor_source_expired");
    EXPECT_EQ(out.completion,MotionSweepCompletion::Uncertified);
    EXPECT_EQ(out.unique_voxels,0U);EXPECT_EQ(GridMapTestAccess::raw(map),before);
  }
}
TEST(MotionSweep,ResourceIncompleteIsNotAnOccupiedWitnessOrNewProof) {
  GridMap map;GridMapTestAccess::configure(map);GridMapTestAccess::official(map);
  geometry_msgs::msg::Twist command;command.linear.x=.1;
  const auto out=run(map,command,{},model(),1e-12);
  EXPECT_FALSE(out.valid);EXPECT_EQ(out.completion,MotionSweepCompletion::BudgetExhausted);
  EXPECT_FALSE(out.query_complete);EXPECT_EQ(out.state_counts[1],0U);
}
TEST(MotionSweep,SourceGroundGapWrongFloorOrMissingBodySupportRejects) {
  GridMap m;GridMapTestAccess::configure(m);geometry_msgs::msg::Twist v;v.linear.x=.3;
  for(int which=0;which<4;++which) {
    auto g=ground();
    // Upper-floor literal support cannot support the lower measured body;
    // floor2 itself is a supported ordinary-ground label, not a rejection.
    if(which==0){g.floor_id="floor2";for(auto& p:g.support_ground_xyz)p.z+=3.;}
    if(which==1)for(auto& p:g.support_ground_xyz)p.z+=3.;
    if(which==2)g.support_ground_xyz.erase(std::remove_if(g.support_ground_xyz.begin(),g.support_ground_xyz.end(),
      [](const auto& p){return p.x>.06;}),g.support_ground_xyz.end());
    if(which==3)g.floor_id="floor3";
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
  EXPECT_EQ(loaded.kinematic_profile,KinematicProfile::ForwardYaw);
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"live"),std::invalid_argument);
  {std::ofstream f(path);f<<bytes<<'\n';}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::invalid_argument);
  auto incomplete=j;incomplete.erase("execution_timing");bytes=incomplete.dump();{std::ofstream f(path);f<<bytes;}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::exception);
  auto underbounded=j;underbounded["execution_timing"]["sensor_source_age_bound_s"]=.5;
  bytes=underbounded.dump();{std::ofstream f(path);f<<bytes;}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::invalid_argument);
  auto omni=j;omni["kinematic_profile"]="holonomic_xyyaw";
  bytes=omni.dump();{std::ofstream f(path);f<<bytes;}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::exception);
  omni["measurements"]["stopping_lateral_distance_m"]=.08;
  bytes=omni.dump();{std::ofstream f(path);f<<bytes;}
  const auto lateral=BrakingModel::load(path.string(),digest(bytes),"isolated_mock");
  EXPECT_EQ(lateral.kinematic_profile,KinematicProfile::HolonomicXYYaw);
  EXPECT_DOUBLE_EQ(lateral.lateral_distance,.08);EXPECT_TRUE(lateral.valid());
  for(const auto& invalid:nlohmann::json::array({0.,-.1,1.01,"not-measured"})) {
    omni["measurements"]["stopping_lateral_distance_m"]=invalid;
    bytes=omni.dump();{std::ofstream f(path);f<<bytes;}
    EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::exception);
  }
  omni["kinematic_profile"]="unsupported";bytes=omni.dump();{std::ofstream f(path);f<<bytes;}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::invalid_argument);
  j["measurements"].erase("heading_error_bound_rad");bytes=j.dump();{std::ofstream f(path);f<<bytes;}
  EXPECT_THROW(BrakingModel::load(path.string(),digest(bytes),"isolated_mock"),std::exception);
  std::filesystem::remove(path);
}
TEST(MotionSweep,RuntimeConfiguredRecordLoadsLiveWithoutPhysicalEvidenceClaims) {
  nlohmann::json j={{"schema_version",3},{"transport_mode","live"},{"fixture_only",false},
    {"record_kind","runtime_motion_profile"},{"parameter_basis","vendor_nominal_and_engineering"},
    {"physical_acceptance_verified",false},{"profile","general_low_speed"},
    {"robot_id","D1-Max"},{"sdk_version","0.1.0-charging_v2"},
    {"calibration_sha256",std::string(64,'a')},{"robot_profile_sha256",std::string(64,'b')},
    {"kinematic_profile","holonomic_xyyaw"},
    {"execution_timing",{{"sensor_source_age_bound_s",.5},{"command_pipeline_bound_s",.2},
      {"writer_period_s",.05},{"source_time_uncertainty_s",.25}}},
    {"stationary_evidence",{{"profile","general_low_speed"},{"linear_threshold_mps",.03},
      {"angular_threshold_radps",.05},{"stationary_duration_s",1.},{"reentry_duration_s",.6},
      {"minimum_new_samples",3},{"mc_expected_hz",100.},{"mc_min_hz",10.},{"mc_max_hz",250.},
      {"configured_static_linear_bound_mps",.01},{"configured_static_angular_bound_radps",.02}}},
    {"measurements",{{"model","reaction_braking_reachable_v1"},{"max_speed_mps",.3},{"max_yaw_radps",.5},
      {"forward_scale_mps",1.},{"lateral_metres_per_fraction",1.},{"yaw_scale_radps",2.},
      {"reaction_bound_s",1.},{"stopping_distance_m",.3},{"stopping_lateral_distance_m",.3},
      {"stopping_yaw_rad",.5},{"stop_latency_bound_s",2.},{"tracking_error_bound_m",.05},{"heading_error_bound_rad",.1}}}};
  const auto path=std::filesystem::temp_directory_path()/
    ("d1max_runtime_motion_model_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".json");
  const auto load=[&](const nlohmann::json& record,const std::string& transport="live") {
    const auto bytes=record.dump();{std::ofstream f(path);f<<bytes;}
    return BrakingModel::load(path.string(),digest(bytes),transport);
  };
  const auto loaded=load(j);
  EXPECT_TRUE(loaded.valid());EXPECT_EQ(loaded.record_kind,"runtime_motion_profile");
  EXPECT_EQ(loaded.parameter_basis,"vendor_nominal_and_engineering");
  EXPECT_FALSE(loaded.physical_acceptance_verified);
  EXPECT_EQ(loaded.kinematic_profile,KinematicProfile::HolonomicXYYaw);
  EXPECT_DOUBLE_EQ(loaded.lateral_distance,.3);EXPECT_EQ(loaded.raySourceAgeNs(),500000000LL);
  auto observed=j;observed["parameter_basis"]="vendor_nominal_engineering_and_operator_observation";
  const auto observed_model=load(observed);
  EXPECT_EQ(observed_model.parameter_basis,"vendor_nominal_engineering_and_operator_observation");
  EXPECT_FALSE(observed_model.physical_acceptance_verified);
  EXPECT_DOUBLE_EQ(observed_model.max_speed,loaded.max_speed);
  EXPECT_THROW(BrakingModel::load(path.string(),std::string(64,'0'),"live"),std::invalid_argument);
  EXPECT_THROW(load(j,"isolated_mock"),std::invalid_argument);
  for(const char* key:{"fixture_only","physical_acceptance_verified","speed_mapping_verified",
      "stop_timing_verified","body_envelope_verified","raw_ray_safety_verified"}) {
    auto bad=j;bad[key]=true;EXPECT_THROW(load(bad),std::invalid_argument)<<key;
  }
  for(const char* key:{"record_kind","parameter_basis","calibration_sha256","robot_profile_sha256","sdk_version"}) {
    auto bad=j;bad[key]="";EXPECT_THROW(load(bad),std::exception)<<key;
  }
  for(const char* key:{"configured_static_linear_bound_mps","configured_static_angular_bound_radps"}) {
    auto bad=j;bad["stationary_evidence"].erase(key);EXPECT_THROW(load(bad),std::exception)<<key;
    bad=j;bad["stationary_evidence"][key]=true;EXPECT_THROW(load(bad),std::invalid_argument)<<key;
  }
  auto bad=j;bad["stationary_evidence"]["configured_static_linear_bound_mps"]=.031;
  EXPECT_THROW(load(bad),std::invalid_argument);
  bad=j;bad["stationary_evidence"]["measured_static_linear_bound_mps"]=.009;
  EXPECT_THROW(load(bad),std::invalid_argument);
  bad=j;bad["stationary_evidence"]["mc_min_hz"]=110.;EXPECT_THROW(load(bad),std::invalid_argument);
  bad=j;bad["stationary_evidence"]["minimum_new_samples"]=true;EXPECT_THROW(load(bad),std::invalid_argument);
  bad=j;bad["measurements"]["reaction_bound_s"]=.99;EXPECT_THROW(load(bad),std::invalid_argument);
  bad=j;bad["measurements"]["lateral_metres_per_fraction"]=.29;EXPECT_THROW(load(bad),std::invalid_argument);
  // Removing the runtime kind restores the original physical-record checks.
  auto legacy=j;legacy.erase("record_kind");EXPECT_THROW(load(legacy),std::invalid_argument);
  for(const char* key:{"speed_mapping_verified","stop_timing_verified","body_envelope_verified","raw_ray_safety_verified"})
    legacy[key]=true;
  EXPECT_THROW(load(legacy),std::invalid_argument); // evidence_id still absent
  legacy["evidence_id"]="legacy-evidence";EXPECT_TRUE(load(legacy).valid());
  std::filesystem::remove(path);
}
TEST(MotionSweep,CheckedInRuntimeProfileUsesExactNativeParserAndConfiguredProvenance) {
  const auto path=std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path()/
    "d1max_pct_scan/config/runtime_motion_profile.json";
  ASSERT_TRUE(path.is_absolute());
  std::ifstream stream(path,std::ios::binary);ASSERT_TRUE(stream.good())<<path;
  const std::string bytes((std::istreambuf_iterator<char>(stream)),{});
  ASSERT_FALSE(bytes.empty());
  const auto j=nlohmann::json::parse(bytes);
  const auto expected=digest(bytes);
  const auto loaded=BrakingModel::load(path.string(),expected,"live");
  EXPECT_TRUE(loaded.valid());EXPECT_EQ(loaded.sha256,expected);
  EXPECT_EQ(loaded.record_kind,"runtime_motion_profile");
  EXPECT_EQ(loaded.parameter_basis,"vendor_nominal_engineering_and_operator_observation");
  EXPECT_FALSE(loaded.physical_acceptance_verified);
  EXPECT_EQ(loaded.kinematic_profile,KinematicProfile::HolonomicXYYaw);
  EXPECT_DOUBLE_EQ(loaded.max_speed,j.at("measurements").at("max_speed_mps").get<double>());
  EXPECT_DOUBLE_EQ(loaded.lateral_distance,j.at("measurements").at("stopping_lateral_distance_m").get<double>());
  EXPECT_DOUBLE_EQ(loaded.sensor_source_age,j.at("execution_timing").at("sensor_source_age_bound_s").get<double>());
  EXPECT_FALSE(j.contains("evidence_id"));
  EXPECT_THROW(BrakingModel::load(path.string(),std::string(64,'0'),"live"),std::invalid_argument);
  EXPECT_THROW(BrakingModel::load(path.string(),expected,"isolated_mock"),std::invalid_argument);
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
TEST(MotionSweep,BoundCommandPipelineDoesNotRelaxLatestActualBodyOrNativeEvidence) {
  constexpr std::int64_t t=100000000000LL;
  EXPECT_FALSE(motionSourceFresh(t,t+125000000LL));
  EXPECT_TRUE(motionSourceFresh(t,t+125000000LL,200000000LL));
  EXPECT_FALSE(motionSourceFresh(t,t+200000001LL,200000000LL));
  // Two 50 ms lanes can consume 125 ms of the original command budget.
  // A real actual body sampled at +65 ms still has its unchanged 100 ms cap.
  const auto cap=motionEvidenceDeadline(t,t,t+65000000LL,t+200000000LL,t+250000000LL,
    t+250000000LL,t+400000000LL,t,t,500000000LL,200000000LL);
  EXPECT_EQ(cap,t+165000000LL);EXPECT_GT(cap,t+125000000LL);
  EXPECT_EQ(motionEvidenceDeadline(t,t,t,t+200000000LL,t+250000000LL,
    t+250000000LL,t+400000000LL,t,t,500000000LL,200000000LL),t+100000000LL);
  EXPECT_EQ(motionEvidenceDeadline(t,t-80000000LL,t+65000000LL,t+200000000LL,t+250000000LL,
    t+250000000LL,t+400000000LL,t,t,500000000LL,200000000LL),t+120000000LL);
  EXPECT_EQ(motionEvidenceDeadline(t,t,t+65000000LL,t+200000000LL,t+250000000LL,
    t+124000000LL,t+400000000LL,t,t,500000000LL,200000000LL),t+124000000LL);
}
