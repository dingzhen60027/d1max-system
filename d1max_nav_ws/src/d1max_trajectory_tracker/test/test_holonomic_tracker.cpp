#include <gtest/gtest.h>
#include "d1max_trajectory_tracker/tracker_core.hpp"

using namespace d1max_trajectory_tracker;
namespace {
// Isolated controller geometry fixture. Production identity/proof admission
// remains covered by test_tracker_v2 and is not bypassed in the runtime node.
Config config() {
  Config c;c.session_id="holonomic-test";c.require_versioned_identity=false;
  c.motion_model="holonomic";return c;
}
Task task() {
  Task t;t.session_id="holonomic-test";t.frame_id="d1max_loc_odom";
  t.generation=1;t.active=true;t.issued_at=100.;t.goal={2.,0.,.55};return t;
}
Odom odom(double stamp=100.,double yaw=0.) {
  Odom o;o.frame_id="d1max_loc_odom";o.child_frame_id="d1max_loc_base_link";
  o.stamp=o.posterior_stamp=o.imu_stamp=stamp;o.position={0.,0.,.55};o.yaw=yaw;
  o.orientation=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ());return o;
}
Trajectory curve(std::int64_t id=1) {
  Trajectory t;t.session_id="holonomic-test";t.frame_id="d1max_loc_odom";
  t.generation=1;t.id=id;t.start_time=100.;
  for(int i=0;i<23;++i)t.points.emplace_back((i-1)*.05,0.,.55);
  for(int i=0;i<27;++i)t.knots.push_back((i-3)*.2);
  return t;
}
void prepare(TrackerCore& core,const Odom& o=odom(),const Trajectory& t=curve(),const Task& goal=task()) {
  ASSERT_TRUE(core.receiveTask(goal,100.,10.));
  ASSERT_TRUE(core.receiveOdom(o,100.,10.));
  ASSERT_TRUE(core.receiveTrajectory(t,100.,10.))<<core.candidateReason();
}
Eigen::Vector2d planar(const Output& out){return {out.forward,out.lateral};}
void expectNormalSlewOrMeasuredGoalZero(const Output& out,const Eigen::Vector2d& previous_world,
    const Odom& body,const Task& goal,const Config& c,double dt) {
  if(out.reason=="goal_position_reached") {
    // Completed-position/actual-rest zero has the same priority as the task
    // owner's stop. It is not an ordinary limiter step or proof of rest by
    // itself: independently assert the original measured criteria here.
    EXPECT_LE((goal.goal-body.position).head<2>().norm(),c.goal_tolerance);
    EXPECT_LE(std::abs(goal.goal.z()-body.position.z()),c.goal_height_tolerance);
    EXPECT_LE(std::max(body.planar_speed,body.velocity_in_frame.head<2>().norm()),c.stationary_linear_threshold_mps);
    EXPECT_LE(std::abs(body.angular_velocity_in_frame.z()),c.stationary_angular_threshold_radps);
    EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_DOUBLE_EQ(out.yaw_rate,0.);
  } else {
    const Eigen::Vector2d world=Eigen::Rotation2Dd(body.yaw)*planar(out);
    EXPECT_LE((world-previous_world).norm(),c.max_acceleration*dt+1e-12);
  }
}
Config productionSteeringConfig() {
  auto c=config();c.align_then_forward=true;c.holonomic_yaw_policy="path_tangent";
  c.max_acceleration=.2;c.max_yaw_acceleration=.4;c.max_jerk=.4;c.max_yaw_jerk=.8;
  c.kp_position=.6;c.kp_yaw=.8;c.velocity_damping_gain=c.yaw_damping_gain=.35;
  c.min_lookahead_distance_m=.07;return c;
}
Config authorLikeConfig() {
  auto c=config();c.align_then_forward=false;c.holonomic_yaw_policy="path_tangent";
  c.kp_position=.8;c.kp_yaw=1.5;c.heading_threshold=.8;
  c.max_acceleration=.35;c.max_yaw_acceleration=.8;
  c.velocity_damping_gain=c.yaw_damping_gain=c.max_jerk=c.max_yaw_jerk=0.;
  c.min_lookahead_distance_m=.07;return c;
}
Config forwardNavigationConfig() {
  auto c=authorLikeConfig();c.align_then_forward=true;
  c.kp_yaw=1.2;c.yaw_damping_gain=.15;c.steering_error_scale_rad=.04;
  c.min_lookahead_distance_m=.40;c.max_lookahead_distance_m=.80;
  return c;
}
Config continuousResponseConfig() {
  auto c=forwardNavigationConfig();c.max_jerk=.70;c.max_yaw_jerk=2.;
  c.forward_response={c.max_speed,.31328213472822414,.40,.10};
  c.yaw_response={c.max_yaw_rate,.20,.30,.20};return c;
}
struct CommandHistory {
  Output output;
  double forward_acceleration=0.,yaw_acceleration=0.,forward_input_rate=0.,yaw_input_rate=0.;
};
void expectConditionedStep(const Config& c,const Output& out,CommandHistory& previous) {
  constexpr double dt=.05;
  const double acceleration=(out.forward-previous.output.forward)/dt;
  const double yaw_acceleration=(out.yaw_rate-previous.output.yaw_rate)/dt;
  const double input_rate=(c.forward_response.map(out.forward)-c.forward_response.map(previous.output.forward))/dt;
  const double yaw_input_rate=(c.yaw_response.map(out.yaw_rate)-c.yaw_response.map(previous.output.yaw_rate))/dt;
  EXPECT_DOUBLE_EQ(out.lateral,0.);
  EXPECT_LE(std::abs(acceleration),c.max_acceleration+1e-9);
  EXPECT_LE(std::abs(yaw_acceleration),c.max_yaw_acceleration+1e-9);
  if(c.max_jerk>0.) {EXPECT_LE(std::abs(acceleration-previous.forward_acceleration),c.max_jerk*dt+1e-8);}
  if(c.max_yaw_jerk>0.) {EXPECT_LE(std::abs(yaw_acceleration-previous.yaw_acceleration),c.max_yaw_jerk*dt+1e-8);}
  EXPECT_LE(std::abs(input_rate),c.max_input_rate+1e-8);
  EXPECT_LE(std::abs(yaw_input_rate),c.max_input_rate+1e-8);
  EXPECT_LE(std::abs(input_rate-previous.forward_input_rate),c.max_input_acceleration*dt+1e-8);
  EXPECT_LE(std::abs(yaw_input_rate-previous.yaw_input_rate),c.max_input_acceleration*dt+1e-8);
  previous={out,acceleration,yaw_acceleration,input_rate,yaw_input_rate};
}
Trajectory distantVerticalTail() {
  auto t=curve();for(auto& k:t.knots)k*=5.; // Real derivatives stay within the existing spatial bounds.
  for(std::size_t i=19;i<t.points.size();++i) {
    t.points[i].x()=t.points[18].x()+1e-6*(i-18);
    t.points[i].z()=.55+.01*(i-18);
  }
  return t;
}
}

TEST(ForwardPreviewGeometry, CircularReferenceRecoversCurvatureIncludingHeadingWrap) {
  constexpr double pi=3.14159265358979323846;
  for(double yaw:{-pi,-pi+.01,0.,pi-.01,pi})
    for(double curvature:{-2.,-.5,.5,2.})for(double arc:{.04,.4,.8}) {
      const double theta=curvature*arc;
      const Eigen::Vector2d local(std::sin(theta)/curvature,(1.-std::cos(theta))/curvature);
      const Eigen::Rotation2Dd rotate(yaw);
      EXPECT_NEAR(previewReferenceCurvature(rotate*Eigen::Vector2d::UnitX(),rotate*local,0.),
        curvature,1e-10);
    }
}

TEST(ForwardPreviewGeometry, StraightFiniteTailAndInvalidInputsKeepExplicitSemantics) {
  EXPECT_DOUBLE_EQ(previewReferenceCurvature({1.,0.},{.4,0.},0.),0.);
  // No future chord at a finite endpoint does not fabricate a new curvature
  // or skip the existing stopping envelope.
  EXPECT_DOUBLE_EQ(previewReferenceCurvature({1.,0.},{0.,0.},.7),.7);
  EXPECT_TRUE(std::isnan(previewReferenceCurvature({0.,0.},{.4,0.},0.)));
  EXPECT_TRUE(std::isnan(previewReferenceCurvature({1.,0.},{NAN,0.},0.)));
  EXPECT_TRUE(std::isnan(previewReferenceCurvature({1.,0.},{.4,0.},NAN)));
}

TEST(ForwardPreviewGeometry, MeasuredEntryWithZeroCurrentCurvatureCanSeeFutureReturn) {
  for(double side:{-1.,1.}) {
    const auto c=forwardNavigationConfig();auto t=curve();
    for(std::size_t i=0;i<t.points.size();++i) {
      // First three controls impose the actual straight P/V/A at the new
      // curve's entry. The future curve gently returns to the route.
      const double u=std::clamp((static_cast<double>(i)-2.)/18.,0.,1.);
      t.points[i].y()=side*.08*(1.-u*u*u*(10.-15.*u+6.*u*u));
    }
    auto measured=odom();measured.position.y()=side*.08;
    measured.velocity_in_frame={.25,0.,0.};measured.planar_speed=.25;
    const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
    TrackerCore core(c);prepare(core,measured,t);core.recordAppliedOutput(.25,0.,0.,100.,10.);
    const auto out=core.step(100.05,10.05,true);const auto& d=core.controlDiagnostic();
    EXPECT_NEAR(d.steering_current_curvature,0.,1e-12);
    EXPECT_NEAR(d.heading_error,0.,1e-12);
    EXPECT_LT(side*d.steering_curvature,-.02);
    EXPECT_LT(side*d.yaw_target,-.005);
    EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
    EXPECT_EQ(d.steering_curvature_source,"accepted_curve_spatial_preview");
    EXPECT_GT(d.steering_preview_distance,.35);
    const double speed=std::max(out.forward,d.measured_forward)*d.heading_weight;
    // A measured-start curve has zero current heading derivative, not zero
    // future bend: retain the entire preview anticipation without damping it
    // against that different future reference a second time.
    EXPECT_NEAR(d.steering_yaw_feedforward,d.steering_curvature*speed,1e-12);
    EXPECT_NEAR(d.yaw_target,d.steering_yaw_feedforward+
      d.steering_yaw_feedback+d.steering_yaw_damping,1e-12);
    EXPECT_DOUBLE_EQ(core.progressTime(),0.);
    EXPECT_DOUBLE_EQ(core.odometry().stamp,100.);
  }
}

namespace {
// Prescribed source-state sequences, not a simulated actuator or stair motion
// acceptance. ZYX angle rates are converted to an actual angular vector in
// odom; this distinguishes heading motion from roll while pitched uphill.
Odom tiltedHeadingState(double roll,double pitch,double yaw,
    double roll_rate,double pitch_rate,double yaw_rate) {
  auto o=odom(100.,yaw);
  const Eigen::Quaterniond rz(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond ry(Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()));
  o.orientation=rz*ry*Eigen::Quaterniond(Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX()));
  o.angular_velocity_in_frame=yaw_rate*Eigen::Vector3d::UnitZ()+
    pitch_rate*(rz*Eigen::Vector3d::UnitY())+roll_rate*(rz*ry*Eigen::Vector3d::UnitX());
  o.velocity_in_frame={.25*std::cos(yaw),.25*std::sin(yaw),0.};o.planar_speed=.25;
  return o;
}
void prepareHeadingCase(TrackerCore& core,const Odom& body) {
  auto t=curve();auto goal=task();
  const Eigen::Rotation2Dd rotate(body.yaw);
  for(auto& p:t.points)p.head<2>()=rotate*p.head<2>();
  goal.goal.head<2>()=rotate*goal.goal.head<2>();
  prepare(core,body,t,goal);
  core.recordAppliedOutput(.25,0.,0.,100.,10.);
}
}

TEST(ForwardHeadingRate, PitchedRollDoesNotCreateFalseLeftRightSteering) {
  auto c=forwardNavigationConfig();c.kp_yaw=.9;c.yaw_damping_gain=.4;
  for(double pitch:{-std::atan(.5),std::atan(.5)})
    for(double roll:{-.15,0.,.15})for(double yaw:{-2.7,0.,1.1})
      for(double roll_rate:{-.5,.5}) {
        auto body=tiltedHeadingState(roll,pitch,yaw,roll_rate,0.,0.);
        const auto original_angular=body.angular_velocity_in_frame;
        ASSERT_GT(std::abs(original_angular.z()),.2);
        TrackerCore core(c);prepareHeadingCase(core,body);
        const auto output=core.step(100.05,10.05,true);
        EXPECT_NEAR(core.controlDiagnostic().heading_error,0.,1e-12);
        EXPECT_NEAR(core.controlDiagnostic().yaw_target,0.,1e-10);
        EXPECT_NEAR(output.yaw_rate,0.,1e-10);
        EXPECT_GT(output.forward,0.);EXPECT_DOUBLE_EQ(output.lateral,0.);
        EXPECT_TRUE(core.odometry().angular_velocity_in_frame==original_angular);
      }
}

TEST(ForwardHeadingRate, RealHeadingMotionRemainsDampedOnUpAndDownInclines) {
  auto c=forwardNavigationConfig();c.kp_yaw=.9;c.yaw_damping_gain=.4;
  for(double pitch:{-std::atan(.5),0.,std::atan(.5)})
    for(double yaw_rate:{-.20,.20}) {
      const auto body=tiltedHeadingState(.13,pitch,.0,.5,-.3,yaw_rate);
      TrackerCore core(c);prepareHeadingCase(core,body);
      const auto output=core.step(100.05,10.05,true);
      EXPECT_NEAR(core.controlDiagnostic().yaw_target,-.4*yaw_rate,1e-12);
      EXPECT_LT(output.yaw_rate*yaw_rate,0.);
      EXPECT_GT(output.forward,0.);EXPECT_DOUBLE_EQ(output.lateral,0.);
      EXPECT_TRUE(core.odometry().angular_velocity_in_frame==body.angular_velocity_in_frame);
    }
}

TEST(ForwardHeadingRate, ExplicitGoalAlignmentUsesHeadingMotionNotRollProjection) {
  auto c=forwardNavigationConfig();c.kp_yaw=.9;c.yaw_damping_gain=.4;
  // world omega Z is zero, although actual yaw increases at .2 rad/s: roll
  // while pitched cancels that vector component, not the heading derivative.
  auto body=tiltedHeadingState(0.,std::asin(.4),0.,.5,0.,.2);
  ASSERT_NEAR(body.angular_velocity_in_frame.z(),0.,1e-12);
  TrackerCore core(c);prepareHeadingCase(core,body);
  const auto output=core.align(.06,.01,100.05,10.05,true);
  EXPECT_EQ(output.reason,"aligning");
  EXPECT_LT(output.yaw_rate,0.);
  EXPECT_TRUE(core.odometry().angular_velocity_in_frame==body.angular_velocity_in_frame);
}

TEST(HolonomicConfig, ExplicitModelsAndFrozenHeadingAreRequired) {
  Config legacy;EXPECT_EQ(legacy.motion_model,"forward_yaw");
  auto c=config();EXPECT_NO_THROW(c.validate());
  c.motion_model="car";EXPECT_THROW(c.validate(),std::invalid_argument);
  c=config();c.holonomic_yaw_policy="infer_sideslip";EXPECT_THROW(c.validate(),std::invalid_argument);
  c=config();c.holonomic_yaw_policy="hold_heading";TrackerCore core(c);
  ASSERT_TRUE(core.receiveTask(task(),100.,10.));ASSERT_TRUE(core.receiveOdom(odom(),100.,10.));
  EXPECT_FALSE(core.receiveTrajectory(curve(),100.,10.));
  EXPECT_EQ(core.candidateReason(),"frozen_heading_missing_or_invalid");
  c=config();c.min_lookahead_distance_m=.31;c.max_lookahead_distance_m=.30;
  EXPECT_THROW(c.validate(),std::invalid_argument);
  c=config();c.min_lookahead_distance_m=0.;EXPECT_THROW(c.validate(),std::invalid_argument);
  c=config();c.max_lookahead_distance_m=NAN;EXPECT_THROW(c.validate(),std::invalid_argument);
}

TEST(HolonomicConfig, MeasuredDampingIsOptionalFiniteAndBounded) {
  auto c=config();EXPECT_DOUBLE_EQ(c.velocity_damping_gain,0.);EXPECT_DOUBLE_EQ(c.yaw_damping_gain,0.);
  for(double value:{0.,.35,2.}) {
    c.velocity_damping_gain=c.yaw_damping_gain=value;EXPECT_NO_THROW(c.validate());
  }
  for(double value:{-.01,2.01,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    c=config();c.velocity_damping_gain=value;EXPECT_THROW(c.validate(),std::invalid_argument);
    c=config();c.yaw_damping_gain=value;EXPECT_THROW(c.validate(),std::invalid_argument);
  }
}

TEST(HolonomicConfig, CommandJerkIsOptionalFiniteAndBounded) {
  auto c=config();EXPECT_NO_THROW(c.validate());
  EXPECT_DOUBLE_EQ(c.max_jerk,0.);EXPECT_DOUBLE_EQ(c.max_yaw_jerk,0.);
  c.max_jerk=.4;c.max_yaw_jerk=.8;EXPECT_NO_THROW(c.validate());
  for(double bad:{-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN(),2.01}) {
    auto variant=c;variant.max_jerk=bad;EXPECT_THROW(variant.validate(),std::invalid_argument);
  }
  c.max_yaw_jerk=4.01;EXPECT_THROW(c.validate(),std::invalid_argument);
}

TEST(HolonomicAuthorLaw, CompleteXYErrorSurvivesXYZProjectionOnSlope) {
  auto c=authorLikeConfig();auto t=curve();
  for(auto& point:t.points)point.z()+=.1*point.x();
  const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
  auto measured=odom();measured.position.z()+=.10;measured.velocity_in_frame={.25,0.,0.};measured.planar_speed=.25;
  TrackerCore core(c);prepare(core,measured,t);core.recordAppliedOutput(.25,0.,0.,100.,10.);
  const auto out=core.step(100.05,10.05,true);
  const auto p=g->entry.curve_sampler->evaluateDeBoorT(core.progressTime());
  const auto v=g->entry.velocity_sampler->evaluateDeBoorT(core.progressTime());
  const Eigen::Vector2d expected=v.head<2>()+c.kp_position*(p-measured.position).head<2>();
  ASSERT_GT((p-measured.position).x(),.009); // XYZ nearest point leaves a real XY residual.
  EXPECT_NEAR(out.forward,expected.x(),1e-10);EXPECT_NEAR(out.lateral,expected.y(),1e-10);
  EXPECT_FALSE(out.frozen);EXPECT_EQ(out.reason,"tracking");
  EXPECT_NEAR(core.progressArc(),curveArcAt(g->entry.times,g->entry.arcs,core.progressTime()),1e-10);
  std::cout<<"AuthorXY full_error_x="<<(p-measured.position).x()<<" native_vx="<<v.x()
    <<" command_vx="<<out.forward<<" expected_vx="<<expected.x()<<"\n";
}

TEST(HolonomicAuthorLaw, MinorHeadingUsesWholeBodyXYAndDirectYawWithoutMeasuredRateCoasting) {
  for(double error:{-.79,-.34,-.18,.18,.34,.79})for(double rate:{-.3,0.,.3}) {
    const auto c=authorLikeConfig();auto measured=odom(100.,-error);
    measured.angular_velocity_in_frame.z()=rate;TrackerCore core(c);prepare(core,measured);
    const auto out=core.step(100.05,10.05,true);
    EXPECT_EQ(out.reason,"tracking");EXPECT_FALSE(out.frozen);EXPECT_TRUE(core.active());
    EXPECT_GT(out.forward,0.);EXPECT_GT(out.lateral*error,0.);
    EXPECT_GT(out.yaw_rate*error,0.); // No prediction of an assumed inertial coast.
    const double weight=headingTranslationWeight(error,c.heading_threshold);
    const double speed=std::min(.25*weight,c.max_acceleration*.05);
    EXPECT_NEAR(out.forward,speed*std::cos(error),1e-12);
    EXPECT_NEAR(out.lateral,speed*std::sin(error),1e-12);
    EXPECT_DOUBLE_EQ(core.controlDiagnostic().heading_weight,weight);
    if(std::abs(error)<=.4)EXPECT_DOUBLE_EQ(weight,1.); // Small errors retain the entire author law.
    EXPECT_NEAR(core.controlDiagnostic().yaw_target,std::clamp(c.kp_yaw*error,-c.max_yaw_rate,c.max_yaw_rate),1e-12);
  }
}

TEST(HolonomicAuthorLaw, LargeHeadingOnlyReducesTranslationTargetAndRetainsTaskAndCurve) {
  const auto c=authorLikeConfig();TrackerCore core(c);prepare(core,odom(100.,-1.));
  auto out=core.step(100.05,10.05,true);
  EXPECT_EQ(out.reason,"heading_alignment");EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
  EXPECT_GT(out.yaw_rate,0.);EXPECT_FALSE(out.frozen);EXPECT_TRUE(core.active());EXPECT_TRUE(core.hasInstalledGeometry());
  EXPECT_EQ(core.controlDiagnostic().zero_target_cause,"heading_feasibility");
  // The next valid source inside the single author threshold continues the
  // SAME curve immediately. No .10/.04 yaw latch or .6 s local recovery gate.
  auto measured=odom(100.1,-.79);ASSERT_TRUE(core.receiveOdom(measured,100.1,10.1));core.refreshTaskLease(10.1);
  out=core.step(100.1,10.1,true);EXPECT_EQ(out.reason,"tracking");EXPECT_GT(out.forward,0.);EXPECT_GT(out.lateral,0.);
  EXPECT_EQ(core.trajectoryId(),1);EXPECT_EQ(core.generation(),1u);EXPECT_EQ(core.task().goal,task().goal);
  core.cancel("operator_cancel");out=core.step(100.15,10.15,true);
  EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_DOUBLE_EQ(out.yaw_rate,0.);
}

TEST(HolonomicAuthorLaw, CurrentCertifiedTangentFeedforwardDoesNotPredictMeasuredYaw) {
  auto c=authorLikeConfig();auto t=curve();for(auto& point:t.points)point.y()=.2*point.x()*point.x();
  const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
  auto measured=odom();const auto v=g->entry.velocity_sampler->evaluateDeBoorT(0.);
  measured.position=g->entry.points.front();measured.yaw=std::atan2(v.y(),v.x());
  measured.orientation=Eigen::AngleAxisd(measured.yaw,Eigen::Vector3d::UnitZ());
  measured.angular_velocity_in_frame.z()=-.3;TrackerCore core(c);prepare(core,measured,t);
  const auto out=core.step(100.05,10.05,true);
  const auto a=g->acceleration_sampler->evaluateDeBoorT(0.);
  const double curvature=(v.x()*a.y()-v.y()*a.x())/std::pow(v.head<2>().norm(),3.);
  const double expected=curvature*out.forward;
  EXPECT_NEAR(out.yaw_rate,expected,1e-12);EXPECT_GT(out.forward,0.);EXPECT_FALSE(out.frozen);
  EXPECT_NEAR(core.controlDiagnostic().heading_error,0.,1e-12);
  EXPECT_NEAR(core.controlDiagnostic().yaw_target,expected,1e-12);
}

TEST(HolonomicAuthorLaw, PlanarAccelerationUsesOneFrameAcrossMeasuredYawChanges) {
  // Same real-core inputs before/after: 20 Hz, a 5 cm current-curve residual,
  // and a fresh 0.5 rad/s yaw observation. No plant or SDK dynamics are assumed.
  // A vector derivative cannot compare coordinates from two body orientations.
  for(double initial_yaw:{0.,3.13})for(double sign:{-1.,1.}) {
    SCOPED_TRACE(initial_yaw);
    SCOPED_TRACE(sign);
    const auto c=authorLikeConfig();auto t=curve();const Eigen::Rotation2Dd initial(initial_yaw);
    for(auto& p:t.points)p.head<2>()=initial*p.head<2>();
    auto body=odom(100.,initial_yaw);body.position.head<2>()=initial*Eigen::Vector2d(0.,-sign*.05);
    body.velocity_in_frame.head<2>()=initial*Eigen::Vector2d(.2,0.);body.planar_speed=.2;
    TrackerCore core(c);prepare(core,body,t);core.recordAppliedOutput(.2,0.,0.,100.,10.);
    const Eigen::Vector2d previous_world=initial*Eigen::Vector2d(.2,0.);
    body.yaw=angle(initial_yaw+sign*.025);body.orientation=Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ());
    body.angular_velocity_in_frame.z()=sign*.5;body.stamp=body.posterior_stamp=body.imu_stamp=100.05;
    ASSERT_TRUE(core.receiveOdom(body,100.05,10.05));core.refreshTaskLease(10.05);
    const auto out=core.step(100.05,10.05,true);
    const Eigen::Vector2d world=Eigen::Rotation2Dd(body.yaw)*planar(out);
    const double delta=(world-previous_world).norm();
    EXPECT_EQ(out.reason,"tracking");EXPECT_FALSE(out.frozen);EXPECT_TRUE(core.hasInstalledGeometry());
    EXPECT_LE(delta,c.max_acceleration*.05+1e-12);
    const auto expected=normalPlanarStep(initial*Eigen::Vector2d(.25,sign*.04),previous_world,
      core.controlDiagnostic().speed_cap,c.max_acceleration,.05);
    ASSERT_TRUE(expected);EXPECT_NEAR((world-*expected).norm(),0.,1e-12);
    std::cout<<"FrameAcceleration initial_yaw="<<initial_yaw<<" yaw_delta="<<sign*.025
      <<" world_command_delta="<<delta<<" budget="<<c.max_acceleration*.05<<"\n";
  }
}

TEST(HolonomicAuthorLaw, AlignGuardRestoresOutputBasisAndHardZeroClearsIt) {
  const auto c=authorLikeConfig();auto body=odom();body.position.y()=-.05;
  body.velocity_in_frame.x()=.2;body.planar_speed=.2;
  TrackerCore core(c);prepare(core,body);core.recordAppliedOutput(.2,.01,.1,100.,10.);
  const Eigen::Vector2d previous_world(.2,.01);
  body=odom(100.05,.025);body.position.y()=-.05;body.velocity_in_frame.x()=.2;body.planar_speed=.2;
  ASSERT_TRUE(core.receiveOdom(body,100.05,10.05));core.refreshTaskLease(10.05);
  const auto out=core.align(-.2,.07,100.05,10.05,true);
  const Eigen::Vector2d world=Eigen::Rotation2Dd(body.yaw)*planar(out);
  const auto expected=normalPlanarStep(Eigen::Vector2d::Zero(),previous_world,c.max_speed,c.max_acceleration,.05);
  ASSERT_TRUE(expected);EXPECT_NEAR((world-*expected).norm(),0.,1e-12);
  EXPECT_LE((world-previous_world).norm(),c.max_acceleration*.05+1e-12);
  EXPECT_EQ(out.reason,"aligning");EXPECT_TRUE(core.active());

  // Authority/source suspension and cancel are hard zero, not normal braking.
  core.suspendOutput(100.05,10.05,"waiting_task_authority",false);
  body=odom(100.10,-.2);ASSERT_TRUE(core.receiveOdom(body,100.10,10.10));core.refreshTaskLease(10.10);
  const auto resumed=core.step(100.10,10.10,true);
  const Eigen::Vector2d resumed_world=Eigen::Rotation2Dd(body.yaw)*planar(resumed);
  EXPECT_NEAR(resumed_world.x(),c.max_acceleration*.05,1e-12);EXPECT_NEAR(resumed_world.y(),0.,1e-12);
  core.cancel("operator_cancel");const auto stopped=core.step(100.15,10.15,true);
  EXPECT_DOUBLE_EQ(stopped.forward,0.);EXPECT_DOUBLE_EQ(stopped.lateral,0.);EXPECT_DOUBLE_EQ(stopped.yaw_rate,0.);
  EXPECT_FALSE(core.receiveTrajectory(curve(2),100.15,10.15));
}

TEST(HolonomicAuthorLaw, OptionalJerkHistoryIsTransportedWithoutResetOnCurveRefresh) {
  auto c=authorLikeConfig();c.max_jerk=.4;c.holonomic_yaw_policy="hold_heading";
  auto t=curve();t.frozen_yaw=0.;TrackerCore core(c),fixed(c);prepare(core,odom(),t);prepare(fixed,odom(),t);
  Eigen::Vector2d previous_world=Eigen::Vector2d::Zero(),previous_acceleration=Eigen::Vector2d::Zero();
  for(int i=1;i<=40;++i) {
    const double now=100.+i*.05,receipt=10.+i*.05;auto body=odom(now,.2*std::sin(i*.05));
    ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
    ASSERT_TRUE(fixed.receiveOdom(odom(now),now,receipt));fixed.refreshTaskLease(receipt);
    if(i%3==0) {
      auto next=t;next.id=i;next.start_time=now;
      ASSERT_TRUE(core.receiveTrajectory(next,now,receipt));ASSERT_TRUE(fixed.receiveTrajectory(next,now,receipt));
    }
    const auto out=core.step(now,receipt,true);
    const Eigen::Vector2d world=Eigen::Rotation2Dd(body.yaw)*planar(out);
    const Eigen::Vector2d acceleration=(world-previous_world)/.05;
    EXPECT_NEAR((world-planar(fixed.step(now,receipt,true))).norm(),0.,1e-12);
    EXPECT_LE(acceleration.norm(),c.max_acceleration+1e-10);
    EXPECT_LE((acceleration-previous_acceleration).norm(),c.max_jerk*.05+1e-10);
    EXPECT_FALSE(out.frozen);EXPECT_EQ(out.reason,"tracking");
    previous_world=world;previous_acceleration=acceleration;
  }
}

TEST(HolonomicAuthorLaw, HeadingBoundaryTraceReportsActualCommandDerivatives) {
  // Fixed measured trace, not an assumed inertial plant. Count exact zero
  // edges and measure COMMAND vector derivatives in the common planning frame.
  const auto c=authorLikeConfig();TrackerCore core(c);prepare(core,odom(100.,-.8));
  Eigen::Vector2d previous_world=Eigen::Vector2d::Zero(),previous_acceleration=Eigen::Vector2d::Zero();
  int zero_edges=0,near_zero_edges=0;double peak_speed=0.,peak_acceleration=0.,peak_acceleration_change=0.;
  for(int i=1;i<=40;++i) {
    const double now=100.+i*.05,receipt=10.+i*.05;
    auto body=odom(now,-(.8+(i%2?-.001:.001)));
    body.angular_velocity_in_frame.z()=i%2?.04:-.04;
    ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
    const auto out=core.step(now,receipt,true);const Eigen::Vector2d world=Eigen::Rotation2Dd(body.yaw)*planar(out);
    const Eigen::Vector2d acceleration=(world-previous_world)/.05;
    zero_edges+=(world.norm()==0.)!=(previous_world.norm()==0.);
    near_zero_edges+=(world.norm()<=1e-12)!=(previous_world.norm()<=1e-12);
    peak_speed=std::max(peak_speed,world.norm());peak_acceleration=std::max(peak_acceleration,acceleration.norm());
    peak_acceleration_change=std::max(peak_acceleration_change,(acceleration-previous_acceleration).norm());
    EXPECT_FALSE(out.frozen);EXPECT_TRUE(core.active());EXPECT_EQ(core.trajectoryId(),1);
    EXPECT_LE(acceleration.norm(),c.max_acceleration+1e-12);
    previous_world=world;previous_acceleration=acceleration;
  }
  std::cout<<"HeadingBoundary exact_zero_edges="<<zero_edges<<" near_zero_edges_1e12="<<near_zero_edges<<" peak_command_speed="<<peak_speed
    <<" peak_command_acceleration="<<peak_acceleration<<" peak_acceleration_change="<<peak_acceleration_change<<"\n";
  RecordProperty("zero_edges",zero_edges);RecordProperty("peak_command_speed",peak_speed);
  RecordProperty("near_zero_edges_1e12",near_zero_edges);
  RecordProperty("peak_command_acceleration",peak_acceleration);RecordProperty("peak_acceleration_change",peak_acceleration_change);
}

TEST(HolonomicAuthorLaw, HeadingWeightIsC2BoundedWithNoExtraPhaseOrSmallErrorGate) {
  const double threshold=.8,h=1e-5;
  for(double sign:{-1.,1.}) {
    for(double error:{0.,.18,.34,.4})EXPECT_DOUBLE_EQ(headingTranslationWeight(sign*error,threshold),1.);
    for(double error:{.8,.81,1.2,3.141592653589793})
      EXPECT_DOUBLE_EQ(headingTranslationWeight(sign*error,threshold),0.);
    EXPECT_NEAR(headingTranslationWeight(sign*.6,threshold),.5,1e-12);
  }
  double previous=1.;
  for(int i=0;i<=1000;++i) {
    const double weight=headingTranslationWeight(threshold*i/1000.,threshold);
    EXPECT_GE(weight,0.);EXPECT_LE(weight,previous+1e-12);previous=weight;
  }
  for(double boundary:{.4,.8}) {
    const double left=headingTranslationWeight(boundary-h,threshold),at=headingTranslationWeight(boundary,threshold),
      right=headingTranslationWeight(boundary+h,threshold);
    EXPECT_NEAR((right-left)/(2.*h),0.,1e-7);
    EXPECT_NEAR((right-2.*at+left)/(h*h),0.,.002);
  }
}

TEST(HolonomicAuthorLaw, LowSpeedEntryNoiseCannotReverseTheBoundedSpatialStartup) {
  for(double noise:{-.007,.007}) {
    const auto c=authorLikeConfig();auto t=curve();t.points.clear();t.knots.clear();
    for(int i=0;i<9;++i){const double s=i-1.;t.points.emplace_back(noise*s+.01*s*s,0.,.55);}
    for(int i=0;i<13;++i)t.knots.push_back(i-3.);
    const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());ASSERT_TRUE(g->derivatives_valid);
    auto measured=odom();measured.position=g->entry.points.front();
    measured.velocity_in_frame={noise,0.,0.};measured.planar_speed=std::abs(noise);
    TrackerCore core(c);prepare(core,measured,t);const auto out=core.step(100.05,10.05,true);
    EXPECT_EQ(out.reason,"tracking");EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
    EXPECT_DOUBLE_EQ(out.yaw_rate,0.);EXPECT_FALSE(out.frozen);
    EXPECT_DOUBLE_EQ(core.odometry().velocity_in_frame.x(),noise);
    EXPECT_NEAR(core.progressTime(),0.,1e-12);EXPECT_NEAR(core.progressArc(),0.,1e-12);
  }
}

TEST(HolonomicHeadingSeed, PreparedPrefixExactlyMatchesNativeFullLatticeAndBacktracking) {
  auto c=authorLikeConfig();c.require_body_heading_seed=true;auto t=curve();
  for(auto& point:t.points)point.y()=.2*point.x()*point.x();
  for(std::size_t i=19;i<t.points.size();++i)t.points[i]=t.points[18];
  for(auto& knot:t.knots)knot*=3.;
  t.body_heading_seed_yaw=3.12;t.body_heading_sampling_intervals=617;
  const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
  ASSERT_EQ(g->body_heading_samples.size(),618u);
  scan_planner::SplineHeadingContract native;native.measured_yaw=*t.body_heading_seed_yaw;
  native.measured_heading_enabled=true;
  for(double fraction:{0.,.001,.11,.73,.99,1.,.87,.19,0.}) {
    const double at=g->entry.duration*fraction;
    const auto exact=scan_planner::sampledBodyHeadingAt(at,g->entry.duration,t.body_heading_sampling_intervals,
      native,[&](double time){return g->entry.velocity_sampler->evaluateDeBoorT(time);},[]{return true;});
    ASSERT_TRUE(exact);EXPECT_DOUBLE_EQ(g->bodyHeadingAt(at),*exact);
  }
  auto changed=t;changed.body_heading_seed_yaw=3.11;EXPECT_FALSE(g->matches(c,changed));
  changed=t;++changed.body_heading_sampling_intervals;EXPECT_FALSE(g->matches(c,changed));
  auto changed_policy=c;changed_policy.align_then_forward=true;EXPECT_FALSE(g->matches(changed_policy,t));
  changed=t;changed.body_heading_sampling_intervals=50000;
  EXPECT_EQ(PreparedGeometry::build(c,changed,{})->failure,"body_heading_seed_missing_or_invalid");
  changed=t;changed.body_heading_seed_yaw=NAN;
  EXPECT_EQ(PreparedGeometry::build(c,changed,{})->failure,"body_heading_seed_missing_or_invalid");
  changed=t;changed.body_heading_seed_yaw.reset();changed.body_heading_sampling_intervals=0;
  EXPECT_EQ(PreparedGeometry::build(c,changed,{})->failure,"body_heading_seed_missing_or_invalid");
  const auto cancelled=PreparedGeometry::build(c,t,{},[]{return false;});
  EXPECT_EQ(cancelled->failure,"preparation_cancelled");
  auto held=c;held.holonomic_yaw_policy="hold_heading";auto held_curve=t;held_curve.frozen_yaw=*t.body_heading_seed_yaw;
  auto held_geometry=PreparedGeometry::build(held,held_curve,{});ASSERT_TRUE(held_geometry->failure.empty());
  EXPECT_DOUBLE_EQ(held_geometry->bodyHeadingAt(held_geometry->entry.duration),*t.body_heading_seed_yaw);
  held_curve.frozen_yaw=*t.body_heading_seed_yaw-.1;
  EXPECT_EQ(PreparedGeometry::build(held,held_curve,{})->failure,"body_heading_seed_frozen_yaw_mismatch");
}

TEST(HolonomicHeadingSeed, LowSpeedMovingReplacementUsesNewFrozenSeedNotOldHeadingOrActualJoin) {
  auto c=authorLikeConfig();c.require_body_heading_seed=true;auto t=curve();
  for(auto& knot:t.knots)knot*=25.; // v=.01 throughout: heading must retain the certified seed.
  t.body_heading_seed_yaw=1.2;t.body_heading_sampling_intervals=5000;
  auto measured=odom(100.,1.2);measured.velocity_in_frame={.01,0.,0.};measured.planar_speed=.01;
  TrackerCore core(c);prepare(core,measured,t);const auto first=core.step(100.05,10.05,true);
  EXPECT_GT(planar(first).norm(),0.);EXPECT_DOUBLE_EQ(first.yaw_rate,0.);
  EXPECT_GT(core.controlDiagnostic().forward_target,0.); // Startup is nonzero, but the seed test does not prescribe its speed.
  measured.stamp=measured.posterior_stamp=measured.imu_stamp=100.1;
  ASSERT_TRUE(core.receiveOdom(measured,100.1,10.1));core.refreshTaskLease(10.1);
  auto next=t;next.id=2;next.start_time=100.1;next.body_heading_seed_yaw=-.4;
  next.join_orientation=measured.orientation; // Actual adoption remains 1.2, distinct from immutable seed.
  ASSERT_TRUE(core.receiveTrajectory(next,100.1,10.1));const auto out=core.step(100.1,10.1,true);
  EXPECT_EQ(out.reason,"heading_alignment");EXPECT_FALSE(out.frozen);EXPECT_LT(out.yaw_rate,0.);
  EXPECT_NEAR(core.controlDiagnostic().heading_error,-1.6,1e-12);
  EXPECT_LE((planar(out)-planar(first)).norm(),c.max_acceleration*.05+1e-12);
  EXPECT_LE(std::abs(out.yaw_rate-first.yaw_rate),c.max_yaw_acceleration*.05+1e-12);
  EXPECT_EQ(core.trajectoryId(),2);EXPECT_TRUE(core.active());
  EXPECT_DOUBLE_EQ(core.odometry().yaw,1.2);EXPECT_EQ(core.task().goal,task().goal);
  core.cancel("operator_cancel");const auto zero=core.step(100.15,10.15,true);
  EXPECT_DOUBLE_EQ(zero.forward,0.);EXPECT_DOUBLE_EQ(zero.lateral,0.);EXPECT_DOUBLE_EQ(zero.yaw_rate,0.);
  EXPECT_FALSE(core.receiveTrajectory(t,100.15,10.15));
}

TEST(HolonomicControl, DampingUsesActualWorldVelocityAndMeasuredAngularRate) {
  auto body=odom();body.velocity_in_frame={.45,.2,0.};body.planar_speed=body.velocity_in_frame.head<2>().norm();
  body.angular_velocity_in_frame.z()=.3;
  auto old=config();old.max_acceleration=.8;old.max_yaw_acceleration=1.5;
  auto damped=old;damped.velocity_damping_gain=damped.yaw_damping_gain=.35;
  TrackerCore legacy(old),next(damped);prepare(legacy,body);prepare(next,body);
  legacy.recordAppliedOutput(.2,0.,0.,100.,10.);next.recordAppliedOutput(.2,0.,0.,100.,10.);
  const auto a=legacy.step(100.05,10.05),b=next.step(100.05,10.05);
  EXPECT_GT(a.forward,b.forward);EXPECT_DOUBLE_EQ(a.lateral,0.);EXPECT_LT(b.lateral,0.);
  EXPECT_DOUBLE_EQ(a.yaw_rate,0.);EXPECT_LT(b.yaw_rate,0.);
  EXPECT_LE((planar(b)-Eigen::Vector2d(.2,0.)).norm(),damped.max_acceleration*.05+1e-12);
}

TEST(HolonomicControl, ExplicitYawAlignmentRetainsMeasuredDampingAndSlew) {
  auto c=config();c.kp_yaw=.8;c.yaw_damping_gain=.35;c.max_yaw_acceleration=1.5;
  auto body=odom(100.,-.5);body.angular_velocity_in_frame.z()=.4;
  TrackerCore core(c);prepare(core,body);core.recordAppliedOutput(0.,0.,.25,100.,10.);
  const auto out=core.align(0.,.07,100.05,10.05,true);
  EXPECT_EQ(out.reason,"aligning");EXPECT_NEAR(out.yaw_rate,.26,1e-12);
  EXPECT_LE(std::abs(out.yaw_rate-.25),c.max_yaw_acceleration*.05+1e-12);
}

TEST(HolonomicControl, FinalEndpointCaptureIsBoundedToSameCurveConnector) {
  for(double overshoot:{.10,.30}) {
    auto c=config();c.goal_tolerance=.05;c.kp_position=.6;c.velocity_damping_gain=.35;
    c.holonomic_yaw_policy="hold_heading";auto t=curve();t.frozen_yaw=0.;
    const auto geometry=PreparedGeometry::build(c,t,{});ASSERT_TRUE(geometry->failure.empty());
    auto goal=task();goal.goal=geometry->entry.points.back();auto body=odom();TrackerCore core(c);prepare(core,body,t,goal);
    for(int i=1;i<=7;++i) {
      const double now=100.+i*.05;body.position.x()=std::min(goal.goal.x(),i*.15);
      body.stamp=body.posterior_stamp=body.imu_stamp=now;
      ASSERT_TRUE(core.receiveOdom(body,now,now-90.));core.refreshTaskLease(now-90.);ASSERT_TRUE(core.refreshMeasuredProgress(now));
    }
    core.recordAppliedOutput(0.,0.,0.,100.35,10.35);
    body.position.x()=goal.goal.x()+overshoot;body.stamp=body.posterior_stamp=body.imu_stamp=100.4;
    ASSERT_TRUE(core.receiveOdom(body,100.4,10.4));const auto out=core.step(100.4,10.4,true);
    EXPECT_LE(planar(out).norm(),c.max_speed+1e-12);
    EXPECT_LE(planar(out).norm(),c.max_acceleration*.05+1e-12);
    if(overshoot<.15) {
      EXPECT_EQ(out.reason,"final_endpoint_capture");EXPECT_LT(out.forward,0.);EXPECT_FALSE(core.needsTerminalReplan(100.4));
    }else {
      EXPECT_EQ(out.reason,"local_segment_finished_waiting_replan");EXPECT_DOUBLE_EQ(planar(out).norm(),0.);
      EXPECT_TRUE(core.needsTerminalReplan(100.4));EXPECT_TRUE(core.active());EXPECT_TRUE(core.hasInstalledGeometry());
    }
  }
}

TEST(HolonomicControl, IntermediateEndpointNeverCapturesAnArbitraryTaskGoal) {
  auto c=config();c.velocity_damping_gain=.35;auto t=curve();auto body=odom();TrackerCore core(c);prepare(core,body,t);
  const auto geometry=PreparedGeometry::build(c,t,{});
  for(int i=1;i<=7;++i) {
    const double now=100.+i*.05;body.position.x()=std::min(geometry->entry.points.back().x(),i*.15);
    body.stamp=body.posterior_stamp=body.imu_stamp=now;
    ASSERT_TRUE(core.receiveOdom(body,now,now-90.));core.refreshTaskLease(now-90.);ASSERT_TRUE(core.refreshMeasuredProgress(now));
  }
  core.recordAppliedOutput(0.,0.,0.,100.35,10.35);
  body.position.x()+=.3;body.stamp=body.posterior_stamp=body.imu_stamp=100.4;
  ASSERT_TRUE(core.receiveOdom(body,100.4,10.4));const auto out=core.step(100.4,10.4,true);
  EXPECT_EQ(out.reason,"local_segment_finished_waiting_replan");EXPECT_DOUBLE_EQ(planar(out).norm(),0.);
  EXPECT_TRUE(core.active());EXPECT_EQ(core.task().goal,task().goal);
}

TEST(HolonomicControl, UnreachableBrakingBoundaryRequestsFreshMeasuredCurveAndRecovers) {
  auto c=config();c.max_acceleration=.2;auto t=curve();auto body=odom();TrackerCore core(c);prepare(core,body,t);
  for(int i=1;i<=7;++i) {
    const double now=100.+i*.05;body.position.x()=std::min(.95,i*.15);
    body.stamp=body.posterior_stamp=body.imu_stamp=now;
    ASSERT_TRUE(core.receiveOdom(body,now,now-90.));ASSERT_TRUE(core.refreshMeasuredProgress(now));
    core.refreshTaskLease(now-90.);
  }
  core.recordAppliedOutput(.3,0.,0.,100.35,10.35);
  const auto stopped=core.step(100.4,10.4,true);
  EXPECT_EQ(stopped.reason,"braking_envelope_reentry_required");EXPECT_TRUE(core.requiresMeasuredReentry());
  EXPECT_TRUE(core.needsTerminalReplan(100.4));EXPECT_TRUE(core.active());EXPECT_TRUE(core.hasInstalledGeometry());
  EXPECT_DOUBLE_EQ(planar(stopped).norm(),0.);
  body.stamp=body.posterior_stamp=body.imu_stamp=100.45;
  ASSERT_TRUE(core.receiveOdom(body,100.45,10.45));
  auto replacement=curve(2);replacement.start_time=100.45;
  for(auto& point:replacement.points)point.x()+=body.position.x();
  ASSERT_TRUE(core.receiveTrajectory(replacement,100.45,10.45));
  EXPECT_FALSE(core.requiresMeasuredReentry());EXPECT_FALSE(core.needsTerminalReplan(100.45));
  const auto resumed=core.step(100.5,10.5,true);EXPECT_EQ(resumed.reason,"tracking");EXPECT_GT(resumed.forward,0.);
  EXPECT_EQ(core.task().goal,task().goal);
}

TEST(HolonomicControl, JerkSlowdownTargetCannotInventHardReentryForAcceptedNonfinalCurve) {
  auto c=config();c.max_acceleration=.2;c.max_jerk=.4;c.align_then_forward=true;
  TrackerCore core(c);prepare(core);
  // The candidate stops in 30 cm, although the SAME task is still 2 m away.
  // The acceleration-only entry disk admits .24, but an S stop with the
  // reserved half acceleration/jerk budget needs more than those 30 cm.
  core.recordAppliedOutput(.24,0.,0.,100.,10.);
  auto short_curve=curve(2);for(auto& p:short_curve.points)p.x()*=.3;
  // The existing true cap permits this current command, so do not invent a
  // new candidate veto. Jerk shaping may request earlier deceleration; only
  // the original curve speed/acceleration envelope can demand hard reentry.
  ASSERT_TRUE(core.receiveTrajectory(short_curve,100.,10.));
  EXPECT_EQ(core.trajectoryId(),2);EXPECT_FALSE(core.requiresMeasuredReentry());
  const auto out=core.step(100.05,10.05,true);
  EXPECT_EQ(out.reason,"tracking_forward");EXPECT_GT(out.forward,0.);
  EXPECT_FALSE(out.frozen);EXPECT_FALSE(core.requiresMeasuredReentry());
  EXPECT_LE(out.forward,.24);EXPECT_GE(out.forward,.24-c.max_acceleration*.05-1e-12);
  EXPECT_EQ(core.task().goal,task().goal);
  EXPECT_TRUE(core.controlDiagnostic().valid);EXPECT_FALSE(core.controlDiagnostic().final_endpoint);
  EXPECT_NEAR(core.controlDiagnostic().goal_distance,2.,1e-12);
  EXPECT_LT(core.controlDiagnostic().jerk_target_cap,core.controlDiagnostic().speed_cap);
}

TEST(ForwardNavigation, SteeringDiagnosticKeepsOriginalProjectionAndExactYawComponents) {
  auto c=continuousResponseConfig();TrackerCore core(c);prepare(core);
  SteeringPreviewSample previous;
  for(int i=1;i<=8;++i) {
    const double now=100.+i*.05;auto o=odom(now,-.08);
    o.position.x()=.25*i*.05;o.velocity_in_frame={.25,0.,0.};o.planar_speed=.25;
    o.angular_velocity_in_frame.z()=.03;o.source_stamp_ns=static_cast<std::int64_t>(now*1e9);
    ASSERT_TRUE(core.receiveOdom(o,now,now-90.));
    const auto out=core.step(now,now-90.,true);ASSERT_EQ(out.reason,"tracking_forward");
    const auto& d=core.controlDiagnostic();ASSERT_TRUE(d.steering_sample.valid);
    EXPECT_EQ(d.steering_sample.body_source_ns,o.source_stamp_ns);
    EXPECT_EQ(d.steering_sample.curve,1);
    EXPECT_GE(d.steering_sample.preview_arc,d.steering_sample.projection_arc);
    EXPECT_NEAR(d.steering_sample.target.y(),0.,1e-12);
    EXPECT_NEAR(d.steering_sample.target.x()-o.position.x(),
      d.steering_sample.preview_arc-d.steering_sample.projection_arc,1e-9);
    EXPECT_NEAR(d.steering_yaw_feedforward+d.steering_yaw_feedback+d.steering_yaw_damping,
      d.yaw_target,1e-12);
    EXPECT_EQ(d.previous_steering_sample.valid,previous.valid);
    if(previous.valid) {
      EXPECT_EQ(d.previous_steering_sample.body_source_ns,previous.body_source_ns);
      EXPECT_DOUBLE_EQ(d.previous_steering_sample.preview_arc,previous.preview_arc);
      EXPECT_EQ(d.previous_steering_sample.target,previous.target);
      EXPECT_LT(d.previous_steering_sample.body_source_ns,d.steering_sample.body_source_ns);
    }
    previous=d.steering_sample;
  }
}

TEST(HolonomicControl, DistantVerticalMicroTailCannotScaleDownTheCurrentFlatPath) {
  auto c=productionSteeringConfig();auto t=distantVerticalTail();
  const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());ASSERT_TRUE(g->derivatives_valid);
  // Mathematical geometry fixture, not a robot/SDK plant or a saved field curve.
  // Its future micro-tail has q about 1e-4; current native velocity is flat.
  ASSERT_LT(g->planar_scale,.001);
  auto body=odom();body.velocity_in_frame=g->entry.velocity_sampler->evaluateDeBoorT(0.);
  body.planar_speed=body.velocity_in_frame.head<2>().norm();
  TrackerCore core(c);prepare(core,body,t);
  const auto out=core.step(100.05,10.05,true);const auto& d=core.controlDiagnostic();
  ASSERT_TRUE(d.valid);EXPECT_FALSE(d.final_endpoint);EXPECT_EQ(out.reason,"tracking_forward");
  EXPECT_FALSE(out.frozen);EXPECT_GT(out.forward,0.);EXPECT_FALSE(core.requiresMeasuredReentry());
  EXPECT_GT(d.speed_cap,.29);EXPECT_GT(d.remaining_arc,.84);
  double horizontal=0.;for(std::size_t i=1;i<g->entry.points.size();++i)
    horizontal+=(g->entry.points[i]-g->entry.points[i-1]).head<2>().norm();
  EXPECT_NEAR(d.remaining_arc,horizontal,1e-10);
  EXPECT_NEAR(core.progress(100.05).arc_length,0.,1e-12); // XYZ projection is unchanged.
  std::cout<<"GeometryMicroTail current_cap="<<d.speed_cap<<" remaining_horizontal="<<d.remaining_arc
    <<" old_global_q="<<g->planar_scale<<" native_planar_speed="<<d.curve_speed<<"\n";
}

TEST(HolonomicControl, LocalSlopeUsesTrueHorizontalDistanceAndKeepsSpatialBrakingBudget) {
  auto c=productionSteeringConfig();auto t=curve();for(auto& k:t.knots)k*=5.;
  for(auto& p:t.points)p.z()+=.1*p.x();
  const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
  const double q=1./std::sqrt(1.01);
  auto body=odom();body.position=g->entry.points.front();
  body.velocity_in_frame=g->entry.velocity_sampler->evaluateDeBoorT(0.);
  body.planar_speed=body.velocity_in_frame.head<2>().norm();
  TrackerCore core(c);prepare(core,body,t);Output previous;
  for(int i=1;i<=20;++i) {
    const double now=100.+i*.05;body.stamp=body.posterior_stamp=body.imu_stamp=now;
    ASSERT_TRUE(core.receiveOdom(body,now,now-90.));core.refreshTaskLease(now-90.);
    const auto out=core.step(now,now-90.,true);const auto& d=core.controlDiagnostic();
    ASSERT_TRUE(d.valid);EXPECT_NEAR(d.speed_cap,c.max_speed*q,1e-10);
    EXPECT_NEAR(d.remaining_arc,g->entry.points.back().x()-g->entry.points.front().x(),1e-10);
    EXPECT_LE(planar(out).norm(),c.max_speed*q+1e-12);
    EXPECT_LE((planar(out)-planar(previous)).norm(),c.max_acceleration*q*.05+1e-12);
    previous=out;
  }
  // The curve's final XYZ stopping boundary remains mandatory even though
  // ordinary forward travel elsewhere no longer inherits its smallest q.
  EXPECT_DOUBLE_EQ(g->speed_envelope.back(),0.);
  const auto limits=brakingSpeedEnvelope({0.,.1*q*q},{.3,0.},.5*c.max_acceleration,1.);
  ASSERT_EQ(limits.size(),2u);EXPECT_NEAR(limits.front(),std::sqrt(c.max_acceleration*.1*q*q),1e-12);
}

TEST(HolonomicControl, ApproachingTheVerticalTailStillBrakesToTheTrueLocalStoppingBoundary) {
  auto c=productionSteeringConfig();const auto t=distantVerticalTail();
  const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
  auto body=odom();body.position=g->entry.points.front();TrackerCore core(c);prepare(core,body,t);
  // Kinematic geometry observations, not a physical robot model: all XYZ
  // position/velocity sources sample the SAME real spline at real 20 Hz dt.
  bool saw_small_local_cap=false,waited_for_extension=false;
  const int count=static_cast<int>(std::ceil(g->entry.duration/.05));
  for(int i=1;i<=count;++i) {
    const double time=std::min(g->entry.duration,i*.05),now=100.+i*.05;
    body.position=g->entry.curve_sampler->evaluateDeBoorT(time);
    body.velocity_in_frame=g->entry.velocity_sampler->evaluateDeBoorT(time);
    body.planar_speed=body.velocity_in_frame.head<2>().norm();
    body.stamp=body.posterior_stamp=body.imu_stamp=now;
    ASSERT_TRUE(core.receiveOdom(body,now,now-90.));core.refreshTaskLease(now-90.);
    const auto out=core.step(now,now-90.,true);const auto& d=core.controlDiagnostic();
    EXPECT_TRUE(std::isfinite(out.forward));EXPECT_GE(out.forward,0.);EXPECT_TRUE(core.active());
    if(d.valid&&d.speed_cap<.01)saw_small_local_cap=true;
    if(out.reason=="local_segment_finished_waiting_replan")waited_for_extension=true;
  }
  EXPECT_TRUE(saw_small_local_cap);EXPECT_TRUE(waited_for_extension);
  EXPECT_FALSE(core.controlDiagnostic().final_endpoint);EXPECT_EQ(core.task().goal,task().goal);
  EXPECT_DOUBLE_EQ(g->speed_envelope.back(),0.);
  // Once the existing .02 m XYZ endpoint/replan condition is reached, step
  // waits before populating tracking diagnostics; inspect the immutable
  // near-vertical local envelope rather than expecting a tracking sample there.
  EXPECT_LE(g->speed_envelope[g->speed_envelope.size()-2],c.max_speed*.001);
  EXPECT_NEAR(core.progress(120.).arc_length,g->entry.arcs.back(),1e-10);
  EXPECT_GT(core.progress(120.).arc_length,g->entry.points.back().x()-g->entry.points.front().x());
}

TEST(HolonomicControl, PureVerticalPathHasNoHorizontalBrakingCreditOrInventedForwardMotion) {
  auto c=productionSteeringConfig();auto t=curve();for(auto& k:t.knots)k*=5.;
  for(std::size_t i=0;i<t.points.size();++i)t.points[i]={0.,0.,.55+.005*(double(i)-1.)};
  auto body=odom();TrackerCore core(c);prepare(core,body,t);
  const auto out=core.step(100.05,10.05,true);
  EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_DOUBLE_EQ(out.yaw_rate,0.);
  EXPECT_TRUE(std::isfinite(core.controlDiagnostic().jerk_target_cap));
  EXPECT_DOUBLE_EQ(core.controlDiagnostic().remaining_arc,0.);
  EXPECT_DOUBLE_EQ(core.controlDiagnostic().speed_cap,0.);EXPECT_TRUE(core.active());
  EXPECT_DOUBLE_EQ(jerkBrakingSpeedLimit(0.,0.,0.,0.,0.),0.);
}

TEST(HolonomicControl, LowSpeedEntryNoiseUsesSameReliableNearDirectionAsNative) {
  for(double noise:{-.007,.007}) {
    auto c=config();c.align_then_forward=true;c.max_jerk=.4;
    auto t=curve();t.points.clear();t.knots.clear();
    for(int i=0;i<9;++i) {
      const double s=i-1.;t.points.emplace_back(noise*s+.01*s*s,0.,.55);
    }
    for(int i=0;i<13;++i)t.knots.push_back(i-3.);
    const auto geometry=PreparedGeometry::build(c,t,{});ASSERT_TRUE(geometry->failure.empty());
    auto body=odom();body.position=geometry->entry.points.front();body.velocity_in_frame.x()=noise;
    body.planar_speed=std::abs(noise);TrackerCore core(c);prepare(core,body,t);
    const auto out=core.step(100.05,10.05,true);
    EXPECT_EQ(out.reason,"tracking_forward");EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.yaw_rate,0.);
    EXPECT_FALSE(out.frozen);EXPECT_NEAR(core.controlDiagnostic().heading_error,0.,1e-12);
  }
}

TEST(HolonomicControl, CommandDiagnosticsExposeRealOverspeedFeedbackWithoutPretendingArrival) {
  auto c=config();c.align_then_forward=true;c.velocity_damping_gain=.35;c.kp_position=.6;
  auto t=curve();for(auto& p:t.points)p.x()*=.3;for(auto& knot:t.knots)knot*=10.;
  // Measured .252 along a curve whose current nominal speed is .0075: this
  // can demand zero even far from the goal. It is not terminal settling or a
  // calibrated SDK-speed model, and must be visible for a real run audit.
  auto body=odom();body.velocity_in_frame.x()=.252;body.planar_speed=.252;
  TrackerCore core(c);prepare(core,body,t);const auto out=core.step(100.05,10.05,true);
  EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_FALSE(out.frozen);EXPECT_EQ(out.reason,"tracking_forward");
  EXPECT_EQ(core.controlDiagnostic().zero_target_cause,"nonpositive_forward_after_velocity_feedback");
  EXPECT_FALSE(core.controlDiagnostic().final_endpoint);EXPECT_GT(core.controlDiagnostic().goal_distance,1.);
  core.suspendOutput(100.1,10.1,"waiting_fresh_local_map",false);
  EXPECT_FALSE(core.controlDiagnostic().valid);
}

TEST(HolonomicControl, RecoverySourceDurationIsInclusiveWithoutRelaxingItsConfiguredWindow) {
  TrackerCore core(config());prepare(core);core.hold("fixture_source_fault",10.02);
  for(double now:{100.15,100.2,100.749999}) {
    auto body=odom(now);ASSERT_TRUE(core.receiveOdom(body,now,now-90.));EXPECT_TRUE(core.holding());
  }
  auto body=odom(100.75);ASSERT_TRUE(core.receiveOdom(body,100.75,10.75));
  EXPECT_FALSE(core.holding());EXPECT_TRUE(core.active());EXPECT_TRUE(core.hasInstalledGeometry());
  EXPECT_EQ(core.reason(),"tracking");
}

TEST(HolonomicLimiter, TotalNormAndDirectionChangesShareOneAccelerationBudget) {
  Eigen::Vector2d previous=Eigen::Vector2d::Zero();
  for(int i=0;i<120;++i) {
    const Eigen::Vector2d target=i<40?Eigen::Vector2d(3.,3.):i<80?Eigen::Vector2d(-3.,3.):Eigen::Vector2d(0.,0.);
    const auto next=normalPlanarStep(target,previous,.3,.35,.02);ASSERT_TRUE(next);
    EXPECT_LE(next->norm(),.3+1e-12);EXPECT_LE((*next-previous).norm(),.35*.02+1e-12);previous=*next;
  }
}

TEST(HolonomicLimiter, TightenedCapMustRemainReachableWithoutPostSlewClipping) {
  EXPECT_FALSE(normalPlanarStep({0.,0.},{.3,.0},.1,.35,.02));
  const auto next=normalPlanarStep({0.,.29},{.3,0.},.295,.35,.02);
  ASSERT_TRUE(next);EXPECT_LE(next->norm(),.295+1e-12);
  EXPECT_LE((*next-Eigen::Vector2d(.3,0.)).norm(),.007+1e-12);
  EXPECT_FALSE(normalPlanarStep({NAN,0.},{0.,0.},.3,.35,.02));
}

TEST(HolonomicControl, RotatedBodyTranslatesImmediatelyWithoutWaitingForYaw) {
  for(double yaw:{1.5707963267948966,-1.5707963267948966,3.141592653589793}) {
    auto c=config();c.holonomic_yaw_policy="hold_heading";auto t=curve();t.frozen_yaw=yaw;
    TrackerCore core(c);prepare(core,odom(100.,yaw),t);
    const auto out=core.step(100.05,10.05);EXPECT_FALSE(out.frozen);
    const Eigen::Rotation2Dd rotation(yaw);const auto world=rotation*planar(out);
    EXPECT_GT(world.x(),0.);EXPECT_NEAR(world.y(),0.,1e-12);
    EXPECT_LE(planar(out).norm(),.35*.05+1e-12);EXPECT_LE(std::abs(out.yaw_rate),.8*.05+1e-12);
  }
}

TEST(HolonomicControl, CrossTrackFeedbackMovesSidewaysWithoutInventingPathYaw) {
  for(double offset:{-.3,.3}) {
    auto body=odom();body.position.y()=offset;TrackerCore core(config());prepare(core,body);
    const auto out=core.step(100.05,10.05);
    EXPECT_LT(out.lateral*offset,0.);EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.yaw_rate,0.);
    EXPECT_LE(planar(out).norm(),.35*.05+1e-12);
  }
}

TEST(HolonomicControl, TwentyHzOnCurveUsesMeasuredTangentInsteadOfFutureChord) {
  auto t=curve();for(auto& point:t.points)point.y()=.2*point.x()*point.x();
  const auto cached=PreparedGeometry::build(config(),t,{});ASSERT_TRUE(cached->failure.empty());
  const double time=1.6;
  auto body=odom();body.position=cached->entry.curve_sampler->evaluateDeBoorT(time);
  const Eigen::Vector2d tangent=cached->entry.velocity_sampler->evaluateDeBoorT(time).head<2>();
  body.yaw=std::atan2(tangent.y(),tangent.x());
  body.orientation=Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ());
  TrackerCore core(config());prepare(core,odom(),t);
  // Measured progress advances in bounded source samples, never by timer age.
  for(int i=1;i<=4;++i) {
    body.position=cached->entry.curve_sampler->evaluateDeBoorT(i*.4);
    body.stamp=100.+i*.05;
    ASSERT_TRUE(core.receiveOdom(body,body.stamp,10.+i*.05));
    core.step(body.stamp,10.+i*.05);
  }
  core.recordAppliedOutput(tangent.norm(),0.,0.,100.2,10.2);
  const auto out=core.step(100.25,10.25);
  ASSERT_EQ(out.reason,"tracking");
  // With zero measured cross-track error, preview geometry must not create
  // sideways chord-cutting demand or inflate nominal along-track speed.
  EXPECT_NEAR(out.lateral,0.,1e-4);
  EXPECT_NEAR(out.forward,tangent.norm(),1e-4);
}

TEST(HolonomicControl, ForwardStrategyKeepsMovingThroughBendWithCurvatureFeedforward) {
  auto c=config();c.align_then_forward=true;
  auto t=curve();for(auto& point:t.points)point.y()=.2*point.x()*point.x();
  const auto geometry=PreparedGeometry::build(c,t,{});ASSERT_TRUE(geometry->failure.empty());
  TrackerCore core(c);prepare(core,odom(),t);
  auto body=odom();
  for(int i=1;i<=4;++i) {
    const double time=i*.4,now=100.+i*.05;
    body.position=geometry->entry.curve_sampler->evaluateDeBoorT(time);
    const auto tangent=geometry->entry.velocity_sampler->evaluateDeBoorT(time);
    body.yaw=std::atan2(tangent.y(),tangent.x());
    body.orientation=Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ());
    body.velocity_in_frame=tangent;body.planar_speed=tangent.head<2>().norm();
    body.stamp=body.posterior_stamp=body.imu_stamp=now;
    ASSERT_TRUE(core.receiveOdom(body,now,now-90.));core.refreshTaskLease(now-90.);
    core.step(now,now-90.,true);
  }
  core.recordAppliedOutput(.15,0.,0.,100.2,10.2);
  const auto out=core.step(100.25,10.25,true);
  EXPECT_EQ(out.reason,"tracking_forward");EXPECT_FALSE(out.frozen);
  EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
  // Already tangent-aligned: a bend still needs yaw; don't wait for heading
  // lag to accumulate until the controller unnecessarily stops to realign.
  EXPECT_GT(out.yaw_rate,0.);EXPECT_LE(out.yaw_rate,c.max_yaw_acceleration*.05+1e-12);
}

TEST(HolonomicControl, CorrectMeasuredBendRateKeepsFeedforwardInsteadOfPulsingZero) {
  auto c=config();c.align_then_forward=true;c.kp_yaw=.8;c.yaw_damping_gain=.35;
  auto t=curve();for(auto& point:t.points)point.y()=.2*point.x()*point.x();
  const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
  TrackerCore core(c);prepare(core,odom(),t);auto body=odom();
  for(int i=1;i<=4;++i) {
    const double time=i*.4,now=100.+i*.05;
    body.position=g->entry.curve_sampler->evaluateDeBoorT(time);
    const auto v=g->entry.velocity_sampler->evaluateDeBoorT(time);
    body.yaw=std::atan2(v.y(),v.x());body.orientation=Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ());
    body.velocity_in_frame=v;body.planar_speed=v.head<2>().norm();
    body.stamp=body.posterior_stamp=body.imu_stamp=now;
    ASSERT_TRUE(core.receiveOdom(body,now,now-90.));core.refreshTaskLease(now-90.);core.step(now,now-90.,true);
  }
  const auto v=g->entry.velocity_sampler->evaluateDeBoorT(1.6);
  const auto a=g->acceleration_sampler->evaluateDeBoorT(1.6);
  const double required=(v.x()*a.y()-v.y()*a.x())/v.head<2>().squaredNorm();
  body.angular_velocity_in_frame.z()=required;body.stamp=body.posterior_stamp=body.imu_stamp=100.25;
  ASSERT_TRUE(core.receiveOdom(body,100.25,10.25));core.refreshTaskLease(10.25);
  core.recordAppliedOutput(v.head<2>().norm(),0.,required,100.2,10.2);
  const auto out=core.step(100.25,10.25,true);
  EXPECT_EQ(out.reason,"tracking_forward");EXPECT_FALSE(out.frozen);
  EXPECT_GT(out.yaw_rate,0.);EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
  // An opposite actual rate is state feedback, not a reason to disable the
  // requested bend. Keep steering through the existing angular slew limiter.
  body.angular_velocity_in_frame.z()=-.2;body.stamp=body.posterior_stamp=body.imu_stamp=100.30;
  ASSERT_TRUE(core.receiveOdom(body,100.30,10.30));core.refreshTaskLease(10.30);
  const auto turning=core.step(100.30,10.30,true);
  EXPECT_GT(core.controlDiagnostic().yaw_target,0.);
  EXPECT_EQ(turning.reason,"tracking_forward");EXPECT_GT(turning.yaw_rate,0.);
  EXPECT_LE(std::abs(turning.yaw_rate-out.yaw_rate),c.max_yaw_acceleration*.05+1e-12);
  EXPECT_GT(turning.forward,0.);EXPECT_FALSE(turning.frozen);
}

TEST(HolonomicLimiter, CommandSmootherUsesElapsedTimeAndBoundsAccelerationChanges) {
  const auto run=[](const std::vector<double>&intervals) {
    Eigen::Vector2d value=Eigen::Vector2d::Zero(),acceleration=Eigen::Vector2d::Zero();
    double elapsed=0.;
    for(int i=0;elapsed<2.-1e-12;++i) {
      const double dt=std::min(intervals[i%intervals.size()],2.-elapsed);
      const auto previous=value,previous_acceleration=acceleration;
      value=smoothVelocityTarget<2>(Eigen::Vector2d(.2,.1),value,acceleration,.2,.4,dt);
      EXPECT_LE(acceleration.norm(),.2+1e-12);
      EXPECT_LE((acceleration-previous_acceleration).norm(),.4*dt+1e-12);
      EXPECT_LE((value-previous).norm(),.2*dt+1e-12);elapsed+=dt;
    }
    return value;
  };
  const auto at20=run({.05}),at40=run({.025}),jitter=run({.03,.07});
  EXPECT_LT((at20-at40).norm(),3e-4);EXPECT_LT((at20-jitter).norm(),3e-4);
}

TEST(HolonomicLimiter, JerkBrakingReservesAccelerationRampBeforeVelocityStop) {
  const double ordinary=.3*.3/(2.*.2);
  const double zero_acceleration=jerkStoppingDistance(.3,0.,.2,.4);
  const double accelerating=jerkStoppingDistance(.3,.1,.2,.4);
  EXPECT_GT(zero_acceleration,ordinary);EXPECT_GT(accelerating,zero_acceleration);
  EXPECT_NEAR(jerkBrakingSpeedLimit(accelerating,.1,.2,.4,.3),.3,1e-8);
  const double smaller=jerkBrakingSpeedLimit(ordinary,.1,.2,.4,.3);
  EXPECT_LT(smaller,.3);EXPECT_LE(jerkStoppingDistance(smaller,.1,.2,.4),ordinary+1e-12);
  double velocity=.3,acceleration=.1;
  for(int i=0;i<60;++i) {
    const auto next=jerkStopStep(velocity,acceleration,.2,.4,.05);
    EXPECT_GE(next.velocity,0.);EXPECT_LE(std::abs(next.acceleration),.2+1e-12);
    EXPECT_LE(std::abs(next.acceleration-acceleration),.4*.05+1e-10);
    velocity=next.velocity;acceleration=next.acceleration;
  }
  EXPECT_DOUBLE_EQ(velocity,0.);EXPECT_DOUBLE_EQ(acceleration,0.);
}

TEST(HolonomicControl, HeadingNoiseAroundFormerThresholdNeverTogglesForwardOff) {
  auto c=config();c.align_then_forward=true;c.max_jerk=.4;c.max_yaw_jerk=.8;
  TrackerCore core(c);prepare(core,odom(100.,-.59));CommandHistory history;
  double previous_target=0.;
  int positive_samples=0;
  for(int i=1;i<=40;++i) {
    const double now=100.+i*.05,receipt=10.+i*.05;auto body=odom(now,i%2?-.59:-.61);
    ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
    auto replacement=curve(i+1);replacement.start_time=now;
    ASSERT_TRUE(core.receiveTrajectory(replacement,now,receipt));
    const auto out=core.step(now,receipt,true);
    EXPECT_FALSE(out.frozen);EXPECT_EQ(out.reason,"tracking_forward");EXPECT_GE(out.forward,0.);
    if(out.forward>0.)++positive_samples;
    EXPECT_LE(out.forward,c.max_speed);expectConditionedStep(c,out,history);
    const double target=core.controlDiagnostic().forward_target;
    const auto& diagnostic=core.controlDiagnostic();
    const double nominal=std::min(diagnostic.speed_cap,diagnostic.curve_speed*diagnostic.heading_weight);
    const double turn=c.kp_yaw*softTrackingError(diagnostic.heading_error,c.steering_error_scale_rad);
    EXPECT_NEAR(target,nominal*std::min(1.,c.max_yaw_rate/std::abs(turn)),1e-12);
    EXPECT_GT(target,0.); // A feasible moving twist, not an arbitrary minimum forward speed.
    if(i>1) {EXPECT_LT(std::abs(target-previous_target),.01);}
    previous_target=target;
  }
  EXPECT_EQ(positive_samples,40); // Neither side of the old .6 rad boundary creates a stop target.
  EXPECT_GT(history.output.forward,.10);
}

TEST(HolonomicControl, SameTaskCurveRefreshKeepsCommandAccelerationAndHardStopResetsIt) {
  auto c=config();c.align_then_forward=true;c.max_jerk=.4;c.max_yaw_jerk=.8;
  TrackerCore refreshed(c),unchanged(c);prepare(refreshed,odom());prepare(unchanged,odom());
  Output previous;
  for(int i=1;i<=10;++i) {
    const double now=100.+i*.05,receipt=10.+i*.05;auto body=odom(now);
    ASSERT_TRUE(refreshed.receiveOdom(body,now,receipt));ASSERT_TRUE(unchanged.receiveOdom(body,now,receipt));
    refreshed.refreshTaskLease(receipt);unchanged.refreshTaskLease(receipt);
    auto replacement=curve(i+1);replacement.start_time=now;
    ASSERT_TRUE(refreshed.receiveTrajectory(replacement,now,receipt));
    const auto out=refreshed.step(now,receipt,true),other=unchanged.step(now,receipt,true);
    EXPECT_NEAR(out.forward,other.forward,1e-12);EXPECT_NEAR(out.yaw_rate,other.yaw_rate,1e-12);
    EXPECT_GT(out.forward,previous.forward);EXPECT_FALSE(out.frozen);previous=out;
  }
  refreshed.hold("fixture_emergency",10.55);const auto stopped=refreshed.step(100.55,10.55,true);
  EXPECT_DOUBLE_EQ(stopped.forward,0.);EXPECT_DOUBLE_EQ(stopped.lateral,0.);EXPECT_DOUBLE_EQ(stopped.yaw_rate,0.);
  unchanged.cancel("fixture_cancel");const auto cancelled=unchanged.step(100.55,10.55,true);
  EXPECT_DOUBLE_EQ(cancelled.forward,0.);EXPECT_DOUBLE_EQ(cancelled.lateral,0.);EXPECT_DOUBLE_EQ(cancelled.yaw_rate,0.);
}

TEST(HolonomicControl, ForwardSteeringStartsBeforeFullAlignmentAndDoesNotFreezeTask) {
  for(double error:{-.34,-.18,.18,.34}) {
    auto c=config();c.align_then_forward=true;c.kp_yaw=.8;c.yaw_damping_gain=.35;
    auto body=odom(100.,-error);TrackerCore core(c);prepare(core,body);
    const auto out=core.step(100.05,10.05,true);
    EXPECT_EQ(out.reason,"tracking_forward");EXPECT_FALSE(out.frozen);
    EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
    EXPECT_GT(out.yaw_rate*error,0.);EXPECT_TRUE(core.active());
    EXPECT_LE(out.forward,c.max_acceleration*.05+1e-12);
  }
  for(double yaw:{1.5707963267948966,2.}) {
    auto c=config();c.align_then_forward=true;TrackerCore core(c);prepare(core,odom(100.,yaw));
    const auto out=core.step(100.05,10.05,true);
    EXPECT_NEAR(out.forward,0.,1e-15);EXPECT_DOUBLE_EQ(out.lateral,0.);
    EXPECT_LT(out.yaw_rate,0.);EXPECT_FALSE(out.frozen);
  }
}

TEST(HolonomicControl, ForwardSteeringUsesMeasuredErrorWithoutAssumedYawCoasting) {
  auto c=forwardNavigationConfig();
  auto body=odom(100.,-.22);TrackerCore core(c);prepare(core,body);
  auto out=core.step(100.05,10.05,true);ASSERT_GT(out.forward,0.);ASSERT_GT(out.yaw_rate,0.);
  // Original physical-state observations must not be projected through an
  // assumed .8-second inertial coast to suppress an outstanding error.
  body=odom(100.1,-.18);body.angular_velocity_in_frame.z()=.30;
  ASSERT_TRUE(core.receiveOdom(body,100.1,10.1));core.refreshTaskLease(10.1);
  out=core.step(100.1,10.1,true);
  EXPECT_EQ(out.reason,"tracking_forward");EXPECT_FALSE(out.frozen);EXPECT_GT(out.forward,0.);
  EXPECT_GT(out.yaw_rate,0.);EXPECT_GT(core.controlDiagnostic().yaw_target,0.);
  body=odom(100.15,.03);body.angular_velocity_in_frame.z()=.20;
  ASSERT_TRUE(core.receiveOdom(body,100.15,10.15));core.refreshTaskLease(10.15);
  out=core.step(100.15,10.15,true);EXPECT_GT(out.forward,0.);
  EXPECT_LT(core.controlDiagnostic().yaw_target,0.);
  EXPECT_GT(core.controlDiagnostic().yaw_target,-.04); // Soft error plus measured-rate damping, not a latch.
  body=odom(100.2,-.03);body.angular_velocity_in_frame.z()=-.20;
  ASSERT_TRUE(core.receiveOdom(body,100.2,10.2));core.refreshTaskLease(10.2);
  out=core.step(100.2,10.2,true);EXPECT_GT(out.forward,0.);
  EXPECT_GT(core.controlDiagnostic().yaw_target,0.);EXPECT_LT(core.controlDiagnostic().yaw_target,.04);
}

TEST(HolonomicControl, RepeatedCurveRefreshAndSmallYawNoiseKeepContinuousForwardAndBoundedSteering) {
  const auto c=forwardNavigationConfig();
  TrackerCore core(c);prepare(core);
  double previous_forward=0.;
  for(int i=1;i<=24;++i) {
    const double now=100.+i*.05,heading=i%2?.025:-.025;
    auto body=odom(now,i%2?-.03:.03);body.angular_velocity_in_frame.z()=i%2?.01:-.01;
    ASSERT_TRUE(core.receiveOdom(body,now,now-90.));core.refreshTaskLease(now-90.);
    auto replacement=curve(i+1);replacement.start_time=now;
    for(auto& p:replacement.points)p.head<2>()=Eigen::Rotation2Dd(heading)*p.head<2>();
    ASSERT_TRUE(core.receiveTrajectory(replacement,now,now-90.))<<core.candidateReason();
    const auto out=core.step(now,now-90.,true);
    EXPECT_EQ(out.reason,"tracking_forward");EXPECT_FALSE(out.frozen);EXPECT_TRUE(core.active());
    EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_LT(std::abs(out.yaw_rate),.025);
    EXPECT_LE(std::abs(out.forward-previous_forward),c.max_acceleration*.05+1e-12);
    previous_forward=out.forward;
  }
}

TEST(ForwardNavigation, SmallHeadingErrorsKeepForwardAndNeverRequestSideways) {
  const auto c=forwardNavigationConfig();
  for(double degrees:{-4.,0.,4.}) {
    SCOPED_TRACE(degrees);TrackerCore core(c);
    auto body=odom(100.,degrees*0.017453292519943295);prepare(core,body);
    for(int i=1;i<=8;++i) {
      const double now=100.+i*.05,receipt=now-90.;
      body.stamp=body.posterior_stamp=body.imu_stamp=now;
      ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
      const auto out=core.step(now,receipt,true);
      EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
      EXPECT_LE(std::abs(out.yaw_rate),.04);EXPECT_FALSE(out.frozen);EXPECT_TRUE(core.active());
      if(degrees==0.)EXPECT_DOUBLE_EQ(out.yaw_rate,0.);
      else EXPECT_LT(out.yaw_rate*degrees,0.);
    }
  }
}

TEST(ContinuousSteering, SmallStraightHeadingNoiseDoesNotCauseHighAmplitudeAlternatingTurns) {
  const auto c=continuousResponseConfig();TrackerCore core(c);prepare(core);CommandHistory history;
  for(int i=1;i<=80;++i) {
    const double now=100.+i*.05,receipt=now-90.;auto measured=odom(now,(i%2?-1.:1.)*M_PI/180.);
    ASSERT_TRUE(core.receiveOdom(measured,now,receipt));core.refreshTaskLease(receipt);
    const auto out=core.step(now,receipt,true);
    EXPECT_GT(out.forward,0.);EXPECT_FALSE(out.frozen);expectConditionedStep(c,out,history);
    EXPECT_LT(std::abs(c.yaw_response.map(out.yaw_rate)),.02);
  }
}

TEST(ContinuousSteering, RearTargetKeepsTurnSideAcross179To181NoiseAndCurveRefresh) {
  const auto c=continuousResponseConfig();
  for(double side:{-1.,1.}) {
    SCOPED_TRACE(side);TrackerCore core(c);
    prepare(core,odom(100.,-side*179.*M_PI/180.));CommandHistory history;
    for(int i=1;i<=40;++i) {
      const double now=100.+i*.05,receipt=now-90.;
      const double error=side*(i%2?179.:-179.)*M_PI/180.;
      auto measured=odom(now,-error);
      ASSERT_TRUE(core.receiveOdom(measured,now,receipt));core.refreshTaskLease(receipt);
      if(i%2==0) {auto replacement=curve(i);replacement.start_time=now;
        ASSERT_TRUE(core.receiveTrajectory(replacement,now,receipt))<<core.candidateReason();}
      const auto out=core.step(now,receipt,true);
      EXPECT_GT(side*core.controlDiagnostic().yaw_target,0.);
      EXPECT_GT(side*out.yaw_rate,0.);
      EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
      expectConditionedStep(c,out,history);
    }
  }
}

TEST(ContinuousSteering, RearTurnSideSurvivesObservedLargeSameTaskCurveDirectionChanges) {
  const auto c=continuousResponseConfig();TrackerCore core(c);prepare(core);CommandHistory history;
  // Oct 19 22:16 actual ControlTrace errors. The direction remains behind,
  // but incoming local curves alternate across the atan2 branch cut.
  const std::vector<double> errors{3.003141,-3.001995,-2.532741,-3.058955,2.081980,2.144392,-2.459630};
  for(std::size_t i=0;i<errors.size();++i) {
    const double now=100.+(i+1)*.05,receipt=now-90.;
    ASSERT_TRUE(core.receiveOdom(odom(now),now,receipt));core.refreshTaskLease(receipt);
    auto replacement=curve(i+2);replacement.start_time=now;
    for(auto& p:replacement.points)p.head<2>()=Eigen::Rotation2Dd(errors[i])*p.head<2>();
    ASSERT_TRUE(core.receiveTrajectory(replacement,now,receipt))<<core.candidateReason();
    const auto out=core.step(now,receipt,true);
    EXPECT_GT(core.controlDiagnostic().yaw_target,0.);
    EXPECT_GT(out.yaw_rate,0.);EXPECT_DOUBLE_EQ(out.forward,0.);expectConditionedStep(c,out,history);
  }
}

TEST(ContinuousSteering, RearTurnIntentIsBoundedAndReleasesForOrdinaryFrontCorrections) {
  for(double side:{-1.,1.}) {
    RearTurnIntent intent;
    for(double degrees:{170.,179.,-179.,-140.,140.,100.,80.,20.,0.,-10.}) {
      const double resolved=intent.resolve(side*degrees*M_PI/180.);
      EXPECT_LE(std::abs(resolved),1.5*M_PI);
      if(std::abs(degrees)>90.)EXPECT_GT(side*resolved,0.);
      else EXPECT_NEAR(resolved,side*degrees*M_PI/180.,1e-12);
    }
    // Having reached the front, another rear request chooses its own shortest
    // side; there is no accumulated angle or remembered full-circle demand.
    EXPECT_LT(side*intent.resolve(-side*170.*M_PI/180.),0.);
    intent.reset();EXPECT_GT(side*intent.resolve(side*170.*M_PI/180.),0.);
  }
}

TEST(ContinuousSteering, RearMemoryUsesFrozenPathHemisphereNotLateralFeedbackAtNinetyDegrees) {
  const auto c=continuousResponseConfig();
  for(double side:{-1.,1.})for(double degrees:{89.99,90.,90.01}) {
    SCOPED_TRACE(side);
    SCOPED_TRACE(degrees);
    TrackerCore core(c);CommandHistory history;
    prepare(core,odom(100.,-side*179.*M_PI/180.));
    auto body=odom(100.05,-side*179.*M_PI/180.);
    ASSERT_TRUE(core.receiveOdom(body,100.05,10.05));core.refreshTaskLease(10.05);
    expectConditionedStep(c,core.step(100.05,10.05,true),history);
    ASSERT_GT(side*core.controlDiagnostic().yaw_target,0.);
    body=odom(100.10,side*degrees*M_PI/180.);body.position.y()=side*.06;
    ASSERT_TRUE(core.receiveOdom(body,100.10,10.10));core.refreshTaskLease(10.10);
    const auto out=core.step(100.10,10.10,true);expectConditionedStep(c,out,history);
    // Cross-track feedback makes the final heading error rear even at 89.99
    // deg. Native certifies only the shortest entry for a FRONT path, so the
    // remembered opposite branch must be released without resetting output.
    if(degrees<=90.)EXPECT_LT(side*core.controlDiagnostic().heading_error,0.);
    else EXPECT_GT(side*core.controlDiagnostic().heading_error,0.);
    EXPECT_GE(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
    if(degrees<=90.) {
      body=odom(100.15);ASSERT_TRUE(core.receiveOdom(body,100.15,10.15));core.refreshTaskLease(10.15);
      auto replacement=curve(2);replacement.start_time=100.15;
      for(auto& p:replacement.points)p.head<2>()=Eigen::Rotation2Dd(-side*170.*M_PI/180.)*p.head<2>();
      ASSERT_TRUE(core.receiveTrajectory(replacement,100.15,10.15))<<core.candidateReason();
      const auto changed=core.step(100.15,10.15,true);expectConditionedStep(c,changed,history);
      EXPECT_LT(side*core.controlDiagnostic().yaw_target,0.); // Same task, genuinely new rear direction.
    }
  }
}

TEST(ContinuousSteering, RearTurnProofPausePreservesIntentButCancelAndNewTaskDoNot) {
  const auto c=continuousResponseConfig();
  for(bool cancel_first:{false,true}) {
    TrackerCore core(c);prepare(core,odom(100.,-179.*M_PI/180.));
    auto measured=odom(100.05,-179.*M_PI/180.);
    ASSERT_TRUE(core.receiveOdom(measured,100.05,10.05));core.refreshTaskLease(10.05);
    EXPECT_GT(core.step(100.05,10.05,true).yaw_rate,0.);
    core.suspendOutput(100.10,10.10,"waiting_fresh_local_map",false);
    measured=odom(100.15,179.*M_PI/180.);
    ASSERT_TRUE(core.receiveOdom(measured,100.15,10.15));core.refreshTaskLease(10.15);
    EXPECT_GT(core.step(100.15,10.15,true).yaw_rate,0.);
    EXPECT_GT(core.controlDiagnostic().yaw_target,0.);
    if(cancel_first)core.cancel("user_cancel");
    auto next=task();next.generation=2;next.issued_at=100.20;
    ASSERT_TRUE(core.receiveTask(next,100.20,10.20));
    auto replacement=curve();replacement.generation=2;replacement.start_time=100.20;
    ASSERT_TRUE(core.receiveTrajectory(replacement,100.20,10.20));
    measured=odom(100.25,179.*M_PI/180.);
    ASSERT_TRUE(core.receiveOdom(measured,100.25,10.25));core.refreshTaskLease(10.25);
    EXPECT_LT(core.step(100.25,10.25,true).yaw_rate,0.);
    EXPECT_LT(core.controlDiagnostic().yaw_target,0.);
  }
}

TEST(ContinuousSteering, PreparedRefreshCarriesCurrentRearTurnIntentNotPreparationSnapshot) {
  const auto c=continuousResponseConfig();TrackerCore current(c);auto slow=curve();
  for(auto& knot:slow.knots)knot*=10.; // Within the unchanged measured-velocity splice bound.
  prepare(current,odom(),slow);
  TrackerCore prepared(current); // Prepared before the first rear-turn decision.
  auto measured=odom(100.05,-179.*M_PI/180.);
  ASSERT_TRUE(current.receiveOdom(measured,100.05,10.05));current.refreshTaskLease(10.05);
  EXPECT_GT(current.step(100.05,10.05,true).yaw_rate,0.);
  measured=odom(100.10,179.*M_PI/180.);
  ASSERT_TRUE(current.receiveOdom(measured,100.10,10.10));current.refreshTaskLease(10.10);
  ASSERT_TRUE(prepared.refreshPreparedState(current,100.10,10.10));
  const auto out=prepared.step(100.10,10.10,true);
  EXPECT_GT(out.yaw_rate,0.);EXPECT_GT(prepared.controlDiagnostic().yaw_target,0.);
  EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_GE(out.forward,0.);
}

TEST(ContinuousSteering, ExplicitRearGoalHeadingHasIndependentStableTurnIntent) {
  auto c=continuousResponseConfig();c.external_goal_completion=true;
  auto goal=task();goal.goal.x()=.05;
  for(double side:{-1.,1.}) {
    auto measured=odom(100.,side*M_PI/180.);measured.position.x()=.05;
    TrackerCore core(c);prepare(core,measured,curve(),goal);CommandHistory history;
    core.step(100.,10.,true); // Establish the existing source/control clock before alignment.
    for(int i=1;i<=40;++i) {
      const double now=100.+i*.05,receipt=now-90.;
      measured=odom(now,side*(i%2?1.:-1.)*M_PI/180.);measured.position.x()=.05;
      ASSERT_TRUE(core.receiveOdom(measured,now,receipt));core.refreshTaskLease(receipt);
      const auto out=core.align(side*M_PI,.10,now,receipt,true);
      EXPECT_GT(side*out.yaw_rate,0.);EXPECT_DOUBLE_EQ(out.forward,0.);
      expectConditionedStep(c,out,history);
    }
  }
}

TEST(ContinuousSteering, RearTurnMeasuredYawResponseConvergesFromBothSidesWithoutFullRevolution) {
  const auto c=continuousResponseConfig();
  for(double side:{-1.,1.}) {
    TrackerCore core(c);double yaw=-side*170.*M_PI/180.,travel=0.;
    prepare(core,odom(100.,yaw));CommandHistory history;bool passed100=false,forward=false;
    for(int i=1;i<=360;++i) {
      const double now=100.+i*.05,receipt=now-90.;auto measured=odom(now,yaw);
      measured.angular_velocity_in_frame.z()=history.output.yaw_rate;
      ASSERT_TRUE(core.receiveOdom(measured,now,receipt));core.refreshTaskLease(receipt);
      const auto out=core.step(now,receipt,true);expectConditionedStep(c,out,history);
      EXPECT_GE(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
      passed100|=std::abs(yaw)<100.*M_PI/180.;forward|=out.forward>0.;
      yaw+=out.yaw_rate*.05;travel+=std::abs(out.yaw_rate)*.05;
    }
    EXPECT_TRUE(passed100);EXPECT_TRUE(forward);EXPECT_LT(std::abs(yaw),.05);
    EXPECT_LT(travel,M_PI+.20);
  }
}

TEST(ContinuousSteering, PersistentCrossTrackOffsetUsesTheCorrectDirectionWithoutSidewaysOrFixedAmplitude) {
  const auto c=continuousResponseConfig();double smaller=0.;
  for(double offset:{-.06,.06}) {
    SCOPED_TRACE(offset);TrackerCore core(c);auto measured=odom();measured.position.y()=offset;prepare(core,measured);
    Output out;CommandHistory history;
    for(int i=1;i<=40;++i) {
      const double now=100.+i*.05,receipt=now-90.;measured.stamp=measured.posterior_stamp=measured.imu_stamp=now;
      ASSERT_TRUE(core.receiveOdom(measured,now,receipt));core.refreshTaskLease(receipt);out=core.step(now,receipt,true);
      EXPECT_GT(out.forward,0.);EXPECT_LT(out.yaw_rate*offset,0.);expectConditionedStep(c,out,history);
    }
    if(offset<0.)smaller=std::abs(out.yaw_rate);
    else EXPECT_NEAR(std::abs(out.yaw_rate),smaller,1e-12);
    EXPECT_GT(std::abs(c.yaw_response.map(out.yaw_rate)),0.);
    EXPECT_LT(std::abs(c.yaw_response.map(out.yaw_rate)),c.yaw_response.maximum_fraction);
  }
}

TEST(ContinuousSteering, OppositeRequestCrossesContinuouslyWithoutWaitingForActualTurnRest) {
  const auto c=continuousResponseConfig();TrackerCore core(c);prepare(core,odom(100.,-10.*M_PI/180.));
  int tick=0;
  CommandHistory history;
  const auto step=[&](double error_deg,double measured_rate) {
    const double now=100.+(++tick)*.05,receipt=now-90.;auto measured=odom(now,-error_deg*M_PI/180.);
    measured.angular_velocity_in_frame.z()=measured_rate;
    EXPECT_TRUE(core.receiveOdom(measured,now,receipt));core.refreshTaskLease(receipt);
    const auto out=core.step(now,receipt,true);EXPECT_GT(out.forward,0.);expectConditionedStep(c,out,history);return out;
  };
  for(int i=0;i<12;++i) { EXPECT_GT(step(10.,.10).yaw_rate,0.); }
  bool reversed=false,carried_previous_command=false;
  for(int i=0;i<40;++i) {
    const auto out=step(-10.,.10); // Old measured rotation remains nonzero throughout.
    EXPECT_LT(core.controlDiagnostic().yaw_target,0.);
    carried_previous_command|=out.yaw_rate>0.;reversed|=out.yaw_rate<0.;
  }
  EXPECT_TRUE(carried_previous_command);EXPECT_TRUE(reversed);
}

TEST(ContinuousSteering, SameTaskCurveRefreshPreservesBothCommandDerivativeHistories) {
  const auto c=continuousResponseConfig();TrackerCore core(c),unchanged(c);
  prepare(core,odom(100.,-10.*M_PI/180.));prepare(unchanged,odom(100.,-10.*M_PI/180.));CommandHistory history;
  for(int i=1;i<=30;++i) {
    const double now=100.+i*.05,receipt=now-90.;auto measured=odom(now,-(i<=10?10.:6.)*M_PI/180.);
    for(auto* current:{&core,&unchanged}) {ASSERT_TRUE(current->receiveOdom(measured,now,receipt));current->refreshTaskLease(receipt);}
    if(i>10&&i%2==0) {auto replacement=curve(i);replacement.start_time=now;
      ASSERT_TRUE(core.receiveTrajectory(replacement,now,receipt))<<core.candidateReason();}
    const auto out=core.step(now,receipt,true),baseline=unchanged.step(now,receipt,true);
    EXPECT_GT(out.forward,0.);EXPECT_GT(out.yaw_rate,0.);expectConditionedStep(c,out,history);
    EXPECT_NEAR(out.forward,baseline.forward,1e-12);EXPECT_NEAR(out.yaw_rate,baseline.yaw_rate,1e-12);
  }
}

TEST(ContinuousSteering, MalformedCandidateDoesNotClearIncumbentOrResetCommandHistory) {
  const auto c=continuousResponseConfig();TrackerCore core(c),unchanged(c);
  prepare(core,odom(100.,-.12));prepare(unchanged,odom(100.,-.12));CommandHistory history;
  for(int i=1;i<=30;++i) {
    const double now=100.+i*.05,receipt=now-90.;auto measured=odom(now,-.12);
    for(auto* current:{&core,&unchanged}) {ASSERT_TRUE(current->receiveOdom(measured,now,receipt));current->refreshTaskLease(receipt);}
    if(i==12) {auto malformed=curve(2);malformed.start_time=now;malformed.points[3].x()=NAN;
      EXPECT_FALSE(core.receiveTrajectory(malformed,now,receipt));EXPECT_EQ(core.trajectoryId(),1);}
    const auto out=core.step(now,receipt,true),baseline=unchanged.step(now,receipt,true);
    EXPECT_TRUE(core.active());EXPECT_TRUE(core.hasInstalledGeometry());EXPECT_FALSE(out.frozen);
    EXPECT_GT(out.forward,0.);expectConditionedStep(c,out,history);
    EXPECT_NEAR(out.forward,baseline.forward,1e-12);EXPECT_NEAR(out.yaw_rate,baseline.yaw_rate,1e-12);
  }
}

TEST(ContinuousSteering, RealCurvatureStartsSteeringBeforeHeadingErrorAccumulates) {
  const auto c=continuousResponseConfig();auto trajectory=curve();
  for(auto& p:trajectory.points)p.y()=.2*p.x()*p.x();
  const auto geometry=PreparedGeometry::build(c,trajectory,{});ASSERT_TRUE(geometry->failure.empty());
  auto measured=odom();measured.position=geometry->entry.curve_sampler->evaluateDeBoorT(1.);
  auto v=geometry->entry.velocity_sampler->evaluateDeBoorT(1.);
  measured.yaw=std::atan2(v.y(),v.x());measured.orientation=Eigen::AngleAxisd(measured.yaw,Eigen::Vector3d::UnitZ());
  TrackerCore core(c);prepare(core,measured,trajectory);CommandHistory history;
  for(int i=1;i<=8;++i) {
    const double now=100.+i*.05,receipt=now-90.,at=1.+i*.05;
    v=geometry->entry.velocity_sampler->evaluateDeBoorT(at);measured.position=geometry->entry.curve_sampler->evaluateDeBoorT(at);
    measured.yaw=std::atan2(v.y(),v.x());
    measured.orientation=Eigen::AngleAxisd(measured.yaw,Eigen::Vector3d::UnitZ());
    measured.velocity_in_frame=v;measured.planar_speed=v.head<2>().norm();
    measured.stamp=measured.posterior_stamp=measured.imu_stamp=now;
    ASSERT_TRUE(core.receiveOdom(measured,now,receipt));core.refreshTaskLease(receipt);
    const auto out=core.step(now,receipt,true);EXPECT_GT(out.forward,0.);expectConditionedStep(c,out,history);
    EXPECT_LT(std::abs(core.controlDiagnostic().heading_error),.005);
    EXPECT_GT(core.controlDiagnostic().yaw_target,0.);EXPECT_GT(out.yaw_rate,0.);
  }
}

TEST(ContinuousSteering, ExplicitFinalHeadingUsesSeparateToleranceAndNeverCompletesOutsideIt) {
  auto c=continuousResponseConfig();c.external_goal_completion=true;auto goal=task();goal.goal.x()=.05;
  auto measured=odom();measured.position.x()=.05;measured.angular_velocity_in_frame.z()=.10;
  TrackerCore core(c);prepare(core,measured,curve(),goal);Output out;
  core.recordAppliedOutput(0.,0.,0.,100.,10.);
  for(int i=1;i<=12;++i) {
    const double now=100.+i*.05,receipt=now-90.;measured.stamp=measured.posterior_stamp=measured.imu_stamp=now;
    ASSERT_TRUE(core.receiveOdom(measured,now,receipt));core.refreshTaskLease(receipt);
    out=core.align(6.*M_PI/180.,4.*M_PI/180.,now,receipt,true);
    EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_GT(out.yaw_rate,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
  }
  EXPECT_EQ(out.reason,"aligning");EXPECT_FALSE(out.finished);EXPECT_TRUE(core.active());
  measured=odom(100.65,3.*M_PI/180.);measured.position.x()=.05;
  ASSERT_TRUE(core.receiveOdom(measured,100.65,10.65));core.refreshTaskLease(10.65);
  out=core.align(6.*M_PI/180.,4.*M_PI/180.,100.65,10.65,true);
  EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_DOUBLE_EQ(out.yaw_rate,0.);
}

TEST(ContinuousSteering, CancellationAndNewTaskCannotResurrectOldCommandHistory) {
  const auto c=continuousResponseConfig();TrackerCore core(c);prepare(core,odom(100.,-10.*M_PI/180.));
  for(int i=1;i<=10;++i) {const double now=100.+i*.05;auto measured=odom(now,-10.*M_PI/180.);
    ASSERT_TRUE(core.receiveOdom(measured,now,now-90.));core.refreshTaskLease(now-90.);EXPECT_GT(core.step(now,now-90.,true).yaw_rate,0.);}
  core.cancel("operator_cancel");EXPECT_DOUBLE_EQ(core.step(100.55,10.55,true).yaw_rate,0.);
  auto next=task();next.generation=2;next.issued_at=100.60;
  ASSERT_TRUE(core.receiveTask(next,100.60,10.60));ASSERT_TRUE(core.receiveOdom(odom(100.60),100.60,10.60));
  auto next_curve=curve();next_curve.generation=2;next_curve.start_time=100.60;
  ASSERT_TRUE(core.receiveTrajectory(next_curve,100.60,10.60));
  const auto restarted=core.step(100.65,10.65,true);EXPECT_GT(restarted.forward,0.);EXPECT_DOUBLE_EQ(restarted.yaw_rate,0.);
  TrackerCore fresh(c);ASSERT_TRUE(fresh.receiveTask(next,100.60,10.60));
  ASSERT_TRUE(fresh.receiveOdom(odom(100.60),100.60,10.60));ASSERT_TRUE(fresh.receiveTrajectory(next_curve,100.60,10.60));
  const auto expected=fresh.step(100.65,10.65,true);
  EXPECT_DOUBLE_EQ(restarted.forward,expected.forward);EXPECT_DOUBLE_EQ(restarted.yaw_rate,expected.yaw_rate);
}

TEST(ForwardNavigation, SoftHeadingFeedbackIsOddMonotonicAndHasNoThresholdGainSpike) {
  const auto c=forwardNavigationConfig();
  for(double sign:{-1.,1.}) {
    SCOPED_TRACE(sign);double previous_target=0.,previous_error=0.;
    for(double degrees:{0.,.1,1.,3.,4.,5.,5.001,6.,7.,7.999,8.,10.}) {
      SCOPED_TRACE(degrees);const double error=sign*degrees*0.017453292519943295;
      TrackerCore core(c);prepare(core,odom(100.,-error));
      const auto out=core.step(100.05,10.05,true);
      const double target=sign*core.controlDiagnostic().yaw_target,full=c.kp_yaw*std::abs(error);
      EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_FALSE(out.frozen);
      EXPECT_GE(target,0.);EXPECT_LE(target,full+1e-12);
      EXPECT_GE(target,previous_target);
      EXPECT_LE(target-previous_target,c.kp_yaw*(std::abs(error)-previous_error)+1e-12);
      if(degrees==0.)EXPECT_DOUBLE_EQ(target,0.);
      else EXPECT_GT(target,0.);
      previous_target=target;previous_error=std::abs(error);
    }
  }
}

TEST(ForwardNavigation, NewCurvesAndHeadingNoiseDoNotResetForwardHistoryOrIntroduceSideways) {
  const auto c=forwardNavigationConfig();TrackerCore core(c);prepare(core);
  core.recordAppliedOutput(.15,0.,0.,100.,10.);
  double previous=.15;
  for(int i=1;i<=24;++i) {
    const double now=100.+i*.05,receipt=now-90.;
    const double curve_heading=(i%2?1.:-1.)*0.017453292519943295;
    auto body=odom(now,(i%2?-3.:3.)*0.017453292519943295);
    ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
    auto replacement=curve(i+1);replacement.start_time=now;
    for(auto& p:replacement.points)p.head<2>()=Eigen::Rotation2Dd(curve_heading)*p.head<2>();
    ASSERT_TRUE(core.receiveTrajectory(replacement,now,receipt))<<core.candidateReason();
    const auto out=core.step(now,receipt,true);
    EXPECT_GT(out.forward,.14);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_LT(std::abs(out.yaw_rate),.04);
    EXPECT_LE(std::abs(out.forward-previous),c.max_acceleration*.05+1e-12);
    EXPECT_FALSE(out.frozen);EXPECT_TRUE(core.active());previous=out.forward;
  }
}

TEST(ForwardNavigation, CrossTrackOffsetIsCorrectedBySteeringRatherThanSidewaysVelocity) {
  const auto c=forwardNavigationConfig();
  for(double offset:{-.06,.06}) {
    SCOPED_TRACE(offset);TrackerCore core(c);auto body=odom();body.position.y()=offset;
    prepare(core,body);const auto out=core.step(100.05,10.05,true);
    EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
    EXPECT_LT(out.yaw_rate*offset,0.);EXPECT_FALSE(out.frozen);EXPECT_TRUE(core.active());
  }
}

TEST(ForwardNavigation, RealBendRetainsContinuousYawFeedforwardInsideStraightErrorBand) {
  const auto c=forwardNavigationConfig();auto t=curve();
  for(auto& point:t.points)point.y()=.2*point.x()*point.x();
  const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
  TrackerCore core(c);prepare(core,odom(),t);auto body=odom();
  for(int i=1;i<=8;++i) {
    const double at=i<=4?i*.4:1.6+(i-4)*.05,now=100.+i*.05,receipt=now-90.;
    const auto v=g->entry.velocity_sampler->evaluateDeBoorT(at);
    const auto a=g->acceleration_sampler->evaluateDeBoorT(at);
    const double required=(v.x()*a.y()-v.y()*a.x())/v.head<2>().squaredNorm();
    body.position=g->entry.curve_sampler->evaluateDeBoorT(at);
    body.yaw=std::atan2(v.y(),v.x());body.orientation=Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ());
    body.velocity_in_frame=v;body.planar_speed=v.head<2>().norm();body.angular_velocity_in_frame.z()=required;
    body.stamp=body.posterior_stamp=body.imu_stamp=now;
    ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
    if(i>=5)core.recordAppliedOutput(v.head<2>().norm(),0.,required,now-.05,receipt-.05);
    const auto out=core.step(now,receipt,true);
    EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_FALSE(out.frozen);
    if(i>=5) {
      EXPECT_LT(std::abs(core.controlDiagnostic().heading_error),c.steering_error_scale_rad);
      EXPECT_GT(out.yaw_rate,0.); // A real bend must not wait for eight degrees of heading lag.
      EXPECT_NEAR(out.yaw_rate,required,.01);
    }
  }
}

TEST(ForwardNavigation, CurvatureNoiseAcrossFormerHardThresholdHasContinuousYawTarget) {
  const auto c=forwardNavigationConfig();double previous_target=0.;bool first=true;
  for(double a:{.1995,.1999,.2,.2001,.2005}) {
    SCOPED_TRACE(a);auto t=curve();for(auto& p:t.points)p.y()=a*p.x()*p.x();
    const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
    auto body=odom();body.position=g->entry.curve_sampler->evaluateDeBoorT(0.);
    body.planar_speed=.125;body.velocity_in_frame={.125,0.,0.};
    TrackerCore core(c);prepare(core,body,t);core.recordAppliedOutput(.25,0.,0.,100.,10.);
    const auto out=core.step(100.05,10.05,true);
    const double target=core.controlDiagnostic().yaw_target;
    EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_FALSE(out.frozen);
    EXPECT_GT(target,0.); // The middle of a blend is not a hidden on/off switch.
    if(!first)EXPECT_LT(std::abs(target-previous_target),.005);
    previous_target=target;first=false;
  }
}

TEST(ForwardNavigation, MeasuredYawDampingIsProportionalAndDoesNotInventAnInertiaHorizon) {
  const auto c=forwardNavigationConfig();std::optional<double> no_rate_target;
  for(double measured_rate:{0.,.30,-.30}) {
    SCOPED_TRACE(measured_rate);auto body=odom(100.,-.18);
    body.angular_velocity_in_frame.z()=measured_rate;
    TrackerCore core(c);prepare(core,body);const auto out=core.step(100.05,10.05,true);
    EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_GT(out.yaw_rate,0.);
    EXPECT_GT(core.controlDiagnostic().yaw_target,0.);EXPECT_FALSE(out.frozen);
    if(no_rate_target)EXPECT_NEAR(core.controlDiagnostic().yaw_target,
      *no_rate_target-c.yaw_damping_gain*measured_rate,1e-12);
    else no_rate_target=core.controlDiagnostic().yaw_target;
  }
}

TEST(ForwardNavigation, CombinedTurnDemandScalesForwardInsteadOfClippingYawAlone) {
  const auto c=forwardNavigationConfig();
  for(double bend:{0.,-1.,1.})for(double error:{.1,.65}) {
    SCOPED_TRACE(::testing::Message()<<"bend="<<bend<<" error="<<error);
    auto t=curve();for(auto& p:t.points)p.y()=bend*p.x()*p.x();
    const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
    const auto v=g->entry.velocity_sampler->evaluateDeBoorT(0.);
    const auto preview_arc=g->arcAtHorizontal(c.min_lookahead_distance_m);
    const auto found=std::lower_bound(g->entry.arcs.begin(),g->entry.arcs.end(),preview_arc);
    const auto index=std::distance(g->entry.arcs.begin(),found);
    const double preview_time=index?g->entry.times[index-1]+(g->entry.times[index]-g->entry.times[index-1])*
      (preview_arc-g->entry.arcs[index-1])/(g->entry.arcs[index]-g->entry.arcs[index-1]):0.;
    const Eigen::Vector2d chord=(g->entry.curve_sampler->evaluateDeBoorT(preview_time)-
      g->entry.curve_sampler->evaluateDeBoorT(0.)).head<2>();
    const Eigen::Vector2d direction=v.head<2>().normalized();
    const double curvature=2.*(direction.x()*chord.y()-direction.y()*chord.x())/chord.squaredNorm();
    const double sign=bend<0.?-1.:1.;
    auto body=odom(100.,std::atan2(v.y(),v.x())-sign*error);
    body.position=g->entry.curve_sampler->evaluateDeBoorT(0.);
    TrackerCore core(c);prepare(core,body,t);
    const auto out=core.step(100.05,10.05,true);const auto& diagnostic=core.controlDiagnostic();
    auto raw=forwardSteeringTarget({c.min_lookahead_distance_m,c.steering_error_scale_rad,
      c.kp_yaw,c.yaw_damping_gain},v.head<2>(),Eigen::Vector2d::Zero(),v.head<2>().norm(),
      body.yaw,0.,curvature);
    const auto acceleration=g->acceleration_sampler->evaluateDeBoorT(0.);
    const double current=(v.x()*acceleration.y()-v.y()*acceleration.x())/
      std::pow(v.head<2>().norm(),3);
    raw.damping_curvature_offset=current-curvature;
    ASSERT_TRUE(raw.valid);raw.forward=std::min(raw.forward,diagnostic.speed_cap);
    const double raw_yaw=raw.yawAtForward(raw.forward);
    const double scale=std::abs(raw_yaw)>c.max_yaw_rate?c.max_yaw_rate/std::abs(raw_yaw):1.;
    EXPECT_NEAR(diagnostic.forward_target,raw.forward*scale,1e-12);
    EXPECT_GT(diagnostic.forward_target,0.);EXPECT_GT(out.forward,0.);
    EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_FALSE(out.frozen);EXPECT_TRUE(core.active());
    // The existing one conditioner may still be ramping. Curvature acts on
    // that actual accepted forward output and the same continuous forward
    // feasibility as translation; the corrective turn shares the allocation.
    const double expected_yaw=(curvature+c.yaw_damping_gain*current)*out.forward*raw.heading_weight+
      scale*raw.yaw_feedback;
    EXPECT_NEAR(diagnostic.yaw_target,expected_yaw,1e-12);
    if(error>.5)EXPECT_LT(diagnostic.forward_target,raw.forward);
    else if(bend==0.)EXPECT_DOUBLE_EQ(diagnostic.forward_target,raw.forward);
  }
}

TEST(ForwardNavigation, TurnAllocationAtFiftyPercentForwardKeepsOneConditionerHistoryAndZeroSideways) {
  auto c=continuousResponseConfig();c.forward_response.maximum_fraction=.50;
  TrackerCore core(c);prepare(core);CommandHistory history;
  for(int tick=1;tick<=180;++tick) {
    const double now=100.+tick*.05,receipt=now-90.;
    // A continuous observed heading error passes into and back out of yaw
    // saturation. This is a command-law test, not a robot dynamics claim.
    const double error=.75*std::pow(std::sin(M_PI*tick/180.),2);
    auto body=odom(now,-error);
    ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
    const auto out=core.step(now,receipt,true);
    EXPECT_FALSE(out.frozen);EXPECT_TRUE(core.active());EXPECT_GT(out.forward,0.);
    expectConditionedStep(c,out,history);
    EXPECT_LE(c.forward_response.map(out.forward),.50+1e-12);
    EXPECT_LE(std::abs(c.yaw_response.map(out.yaw_rate)),.30+1e-12);
  }
  core.cancel("fixture_cancel");const auto stopped=core.step(109.05,19.05,true);
  EXPECT_DOUBLE_EQ(stopped.forward,0.);EXPECT_DOUBLE_EQ(stopped.lateral,0.);EXPECT_DOUBLE_EQ(stopped.yaw_rate,0.);
}

TEST(ForwardNavigation, CurvatureUsesAcquiredForwardSpeedWhenAmplitudeOutrunsSiDemand) {
  auto target=forwardSteeringTarget({.5,.04,1.2,.15},{1.,0.},{0.,0.},.12,0.,0.,1.);
  ASSERT_TRUE(target.valid);target.measured_forward=.30;
  EXPECT_NEAR(target.yawAtForward(.12),1.15*.30,1e-12);
  target.measured_forward=0.;
  EXPECT_NEAR(target.yawAtForward(.12),1.15*.12,1e-12);
  // Current accepted demand anticipates startup, not an advancing path clock.
  EXPECT_NEAR(target.yawAtForward(.02),1.15*.02,1e-12);
}

TEST(DampingReference, ZeroOffsetPreservesTheExactLegacyArithmetic) {
  for(double curvature:{-2.,-.27,0.,.25,2.})for(double heading:{-.8,0.,.8,2.})
    for(double accepted:{0.,.01,.2})for(double actual:{-.2,0.,.5})for(double scale:{0.,.3,1.}) {
      auto t=forwardSteeringTarget({.5,.02,.9,.4},{1.,0.},{0.,.03},.3,heading,.1,curvature);
      t.measured_forward=actual;t.correction_scale=scale;
      const double speed=accepted>0.?std::max(accepted,std::max(0.,actual))*t.heading_weight:0.;
      const double feedforward=curvature*speed;
      const double legacy=(1.+t.yaw_damping)*feedforward+
        scale*(t.yaw_feedback-t.yaw_damping*t.measured_yaw_rate);
      EXPECT_DOUBLE_EQ(t.yawAtForward(accepted),legacy);
    }
}

TEST(DampingReference, PreviewAndCurrentRateKeepDistinctReferencesAndOriginalAllocation) {
  for(double preview:{-.8,.8})for(double current:{-.2,0.,.2})
    for(double actual:{0.,.5})for(double allocation:{0.,.4,1.}) {
      auto t=forwardSteeringTarget({.5,.02,.9,.4},{1.,0.},{0.,.03},.3,-.2,.15,preview);
      t.measured_forward=actual;t.correction_scale=allocation;
      t.damping_curvature_offset=current-preview;
      const double speed=std::max(.2,actual)*t.heading_weight;
      const double expected=(preview+.4*current)*speed+allocation*(t.yaw_feedback-.4*.15);
      EXPECT_NEAR(t.yawAtForward(.2),expected,1e-12);
      auto changed=t;changed.yaw_feedback+=.1;
      EXPECT_NEAR(changed.yawAtForward(.2)-t.yawAtForward(.2),allocation*.1,1e-12);
      changed=t;changed.measured_yaw_rate+=.1;
      EXPECT_NEAR(changed.yawAtForward(.2)-t.yawAtForward(.2),-allocation*.04,1e-12);
      // Acquired speed is a fact: twist allocation cannot retroactively
      // scale its curvature term. Only the original correction part scales.
      EXPECT_NEAR(t.curvatureYawAtSpeed(speed),(preview+.4*current)*speed,1e-12);
    }
}

TEST(DampingReference, CircleAndRearFeasibilityRetainTheirOriginalGeometry) {
  for(double curvature:{-2.,-.25,.25,2.})for(double actual:{0.,.5}) {
    const double speed=std::max(.2,actual);
    auto t=forwardSteeringTarget({.5,.02,.9,.4},{1.,0.},{0.,0.},.2,0.,speed*curvature,curvature);
    t.measured_forward=actual;t.damping_curvature_offset=curvature-curvature;
    EXPECT_NEAR(t.yawAtForward(.2),speed*curvature,1e-12);
  }
  auto rear=forwardSteeringTarget({.5,.02,.9,.4},{1.,0.},{0.,0.},.2,2.,.1,-.8);
  rear.measured_forward=.5;const double original=rear.yawAtForward(.2);
  rear.damping_curvature_offset=5.;
  EXPECT_DOUBLE_EQ(rear.yawAtForward(.2),original);
  EXPECT_DOUBLE_EQ(rear.yawAtForward(0.),original);
}

TEST(DampingReference, DirectionAndFlightWeightsHaveC2SeamsWithoutNewParameters) {
  const scan_planner::SplineHeadingContract contract;
  const double m=contract.minimum_direction_speed;
  auto difference=[&](double speed,double flight) {
    return smoothUnitInterval(speed/m-1.)*(1.-smoothUnitInterval(flight))*.4*.5*(.2-(-.8));
  };
  EXPECT_DOUBLE_EQ(difference(0.,0.),0.);EXPECT_DOUBLE_EQ(difference(m,0.),0.);
  EXPECT_DOUBLE_EQ(difference(2.*m,0.),.4*.5*(.2-(-.8)));
  EXPECT_DOUBLE_EQ(difference(2.*m,1.),0.);
  for(double boundary:{m,2.*m}) {
    const double h=m*1e-4;
    EXPECT_NEAR((difference(boundary+h,0.)-difference(boundary-h,0.))/(2.*h),0.,1e-5);
    EXPECT_NEAR((difference(boundary+h,0.)-2.*difference(boundary,0.)+difference(boundary-h,0.))/(h*h),0.,1.);
  }
}

TEST(DampingReference, SlowAcceptedCurveCannotExposeTheDirectionCutoffAsANewYawJump) {
  auto c=continuousResponseConfig();c.kp_yaw=.9;c.yaw_damping_gain=.4;c.steering_error_scale_rad=.02;
  auto t=curve();t.points.clear();t.knots.clear();
  for(int j=0;j<63;++j){const double q=(j-1)*.2;t.points.emplace_back(.019*q,.004*q*q,.55);}
  for(int j=0;j<67;++j)t.knots.push_back((j-3)*.2);
  const auto geometry=PreparedGeometry::build(c,t,{});ASSERT_TRUE(geometry->failure.empty())<<geometry->failure;
  auto body=odom();body.position=geometry->entry.curve_sampler->evaluateDeBoorT(0.);
  body.velocity_in_frame=geometry->entry.velocity_sampler->evaluateDeBoorT(0.);
  body.planar_speed=body.velocity_in_frame.head<2>().norm();
  body.yaw=std::atan2(body.velocity_in_frame.y(),body.velocity_in_frame.x());
  body.orientation=Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ());
  TrackerCore core(c);prepare(core,body,t);double previous_speed=body.planar_speed,previous_target=0.;bool crossed=false;
  const double m=geometry->body_heading_contract.minimum_direction_speed;
  for(int i=1;i<=30;++i) {
    const double at=i*.05,now=100.+at;
    body.position=geometry->entry.curve_sampler->evaluateDeBoorT(at);
    body.velocity_in_frame=geometry->entry.velocity_sampler->evaluateDeBoorT(at);
    const auto a=geometry->acceleration_sampler->evaluateDeBoorT(at);
    body.planar_speed=body.velocity_in_frame.head<2>().norm();
    body.yaw=std::atan2(body.velocity_in_frame.y(),body.velocity_in_frame.x());
    body.orientation=Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ());
    body.angular_velocity_in_frame.z()=(body.velocity_in_frame.x()*a.y()-body.velocity_in_frame.y()*a.x())/
      body.velocity_in_frame.head<2>().squaredNorm();
    body.stamp=body.posterior_stamp=body.imu_stamp=now;
    ASSERT_TRUE(core.receiveOdom(body,now,10.+at));core.refreshTaskLease(10.+at);
    const auto out=core.step(now,10.+at,true);const auto d=core.controlDiagnostic();
    ASSERT_EQ(out.reason,"tracking_forward");EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
    const double speed=geometry->entry.velocity_sampler->evaluateDeBoorT(core.progressTime()).head<2>().norm();
    const double moving=std::max(out.forward,d.measured_forward)*d.heading_weight;
    const double legacy=(1.+c.yaw_damping_gain)*(d.steering_curvature*moving)+d.steering_yaw_feedback+d.steering_yaw_damping;
    const double expected=legacy+c.yaw_damping_gain*smoothUnitInterval(speed/m-1.)*
      (d.steering_current_curvature-d.steering_curvature)*moving;
    EXPECT_NEAR(d.yaw_target,expected,1e-12);
    if(speed<=m)EXPECT_NEAR(d.yaw_target,legacy,1e-12);
    if(previous_speed<m&&speed>=m){crossed=true;EXPECT_LT(std::abs(d.yaw_target-previous_target),.02);}
    previous_speed=speed;previous_target=d.yaw_target;
  }
  EXPECT_TRUE(crossed);
}

TEST(ForwardNavigation, ZeroCommandAndReverseMeasurementDoNotCreateACurvatureTurn) {
  auto target=forwardSteeringTarget({.5,.04,1.2,.15},{1.,0.},{0.,0.},.12,0.,0.,1.);
  for(double measured:{-.30,-.01,0.,.01,.30}) {
    target.measured_forward=measured;EXPECT_DOUBLE_EQ(target.yawAtForward(0.),0.);
    EXPECT_DOUBLE_EQ(target.yawAtForward(-.01),0.);
  }
  target.measured_forward=-.30;
  EXPECT_NEAR(target.yawAtForward(.02),1.15*.02,1e-12);
  target.measured_forward=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(target.withTwistLimits(.30,.50).valid);
}

TEST(ForwardNavigation, RearTargetDecelerationCannotResurrectOppositeCurvatureTurn) {
  // Recorded 1836.10 target shape: heading +2.548 rad, curvature -4.634,
  // positive residual vx .016988 while the target requests zero. No curve
  // control points were logged, so this reproduces the exact command law,
  // not an invented reconstruction of that full accepted spline.
  for(double side:{-1.,1.}) {
    auto target=forwardSteeringTarget({.4,.02,.9,.4},{1.,0.},{0.,0.},.30,
      -side*2.548417,side*.437565,-side*4.634282);
    target.measured_forward=.35;
    ASSERT_DOUBLE_EQ(target.forward,0.);
    const auto limited=target.withTwistLimits(.30,.50);
    ASSERT_GT(side*limited.yawAtForward(0.),0.);
    for(double coast:{.30,.10,.016988,.001,0.}) {
      EXPECT_GT(side*limited.yawAtForward(coast),0.);
      EXPECT_DOUBLE_EQ(limited.yawAtForward(coast),limited.yawAtForward(0.));
    }
  }
}

TEST(ForwardNavigation, CurvatureAllocationIsContinuousAcrossForwardRearBoundaryWhileCoasting) {
  constexpr double pi=3.14159265358979323846;
  for(double side:{-1.,1.})for(double coasting:{.001,.016988,.3}) {
    const auto yaw=[&](double degrees) {
      const double h=side*degrees*pi/180.;
      auto t=forwardSteeringTarget({.5,.02,.9,.4},{std::cos(h),std::sin(h)},
        Eigen::Vector2d::Zero(),.3,0.,side*.437565,-side*4.634282);
      t.measured_forward=.3;
      const auto target=t.withTwistLimits(.3,.5);
      EXPECT_TRUE(target.valid);
      return target.yawAtForward(coasting);
    };
    // The recorded 146-degree case alone missed the discontinuity immediately
    // before 90 degrees. Test both travel/turn sides and actual limiter coast.
    EXPECT_NEAR(yaw(89.99),yaw(90.01),1e-5);
    EXPECT_GT(side*yaw(89.99),0.);EXPECT_GT(side*yaw(90.01),0.);
    EXPECT_NEAR(yaw(90.-1e-6),yaw(90.+1e-6),1e-9);
    EXPECT_NEAR(yaw(90.-2e-6)-yaw(90.-1e-6),0.,1e-9);
  }
}

TEST(ForwardNavigation, AlignedCurvatureAndStraightPathKeepTheirExistingLaws) {
  auto curved=forwardSteeringTarget({.5,.02,.9,.4},{1.,0.},
    Eigen::Vector2d::Zero(),.3,0.,.1,.25);
  curved.measured_forward=.4;
  EXPECT_DOUBLE_EQ(curved.heading_weight,1.);
  EXPECT_NEAR(curved.yawAtForward(.2),1.4*.25*.4-.4*.1,1e-12);
  for(double heading:{-.8,-.2,0.,.2,.8}) {
    auto straight=forwardSteeringTarget({.5,.02,.9,.4},{1.,0.},
      Eigen::Vector2d::Zero(),.3,heading,.1,0.);
    straight.measured_forward=.4;
    EXPECT_NEAR(straight.yawAtForward(.2),straight.yaw_feedback-.4*.1,1e-12);
  }
}

TEST(ForwardNavigation, AcquiredTurnDemandReducesTranslationWithoutFalsifyingMeasuredSpeed) {
  auto target=forwardSteeringTarget({.5,.04,1.2,.15},{1.,0.},{0.,0.},.20,0.,0.,2.);
  target.measured_forward=.45;
  const double raw_yaw=target.yawAtForward(target.forward);
  ASSERT_GT(raw_yaw,.50);const auto limited=target.withTwistLimits(.30,.50);
  ASSERT_TRUE(limited.valid);EXPECT_GT(limited.forward,0.);
  EXPECT_NEAR(limited.forward,target.forward*.50/raw_yaw,1e-12);
  EXPECT_DOUBLE_EQ(limited.measured_forward,.45);
  // Actual speed is still present while braking. Do not fake a lower MC
  // state just to claim the desired twist is already physically achievable.
  EXPECT_NEAR(limited.yawAtForward(limited.forward),raw_yaw,1e-12);
}

TEST(ForwardNavigation, CurrentCurveCurvatureUsesSameAcquiredOdometryThroughTheConditioner) {
  auto c=continuousResponseConfig();c.forward_response.maximum_fraction=.50;
  auto t=curve();for(auto& p:t.points)p.y()=.5*p.x()*p.x();
  const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty())<<g->failure;
  const auto v=g->entry.velocity_sampler->evaluateDeBoorT(0.);
  auto body=odom(100.,std::atan2(v.y(),v.x()));
  body.position=g->entry.curve_sampler->evaluateDeBoorT(0.);
  body.velocity_in_frame.head<2>()=.30*v.head<2>().normalized();body.planar_speed=.30;
  TrackerCore core(c);prepare(core,body,t);const auto out=core.step(100.05,10.05,true);
  const auto& d=core.controlDiagnostic();
  ASSERT_FALSE(out.frozen);EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
  EXPECT_NEAR(d.measured_forward,.30,1e-12);
  EXPECT_NEAR(d.yaw_target,(d.steering_curvature+
    c.yaw_damping_gain*d.steering_current_curvature)*.30,1e-12);
  EXPECT_LE(out.forward,c.max_acceleration*.05+1e-12);
  EXPECT_LE(std::abs(out.yaw_rate),c.max_yaw_acceleration*.05+1e-12);
  EXPECT_EQ(core.odometry().source_stamp_ns,body.source_stamp_ns);
  EXPECT_EQ(core.odometry().stamp,body.stamp);
  EXPECT_TRUE(core.odometry().velocity_in_frame.isApprox(body.velocity_in_frame,1e-12));
}

TEST(ForwardNavigation, AcquiredBackwardEntryDoesNotTurnTheHeadAwayFromTheSameForwardCurve) {
  auto c=continuousResponseConfig();auto t=curve();
  const double prefix[]={.03,0.,-.03,-.03,-.015,.015,.06};
  for(std::size_t i=0;i<t.points.size();++i)
    t.points[i].x()=i<7?prefix[i]:.06+.06*(i-6);
  for(std::size_t i=0;i<t.knots.size();++i)t.knots[i]=(static_cast<double>(i)-3.)*.3;
  t.body_heading_seed_yaw=0.;t.body_heading_sampling_intervals=500;
  const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty())<<g->failure;
  const auto original_velocity=g->entry.velocity_sampler->evaluateDeBoorT(0.);
  ASSERT_NEAR(original_velocity.x(),-.1,1e-12);ASSERT_NEAR(original_velocity.y(),0.,1e-12);
  auto body=odom();body.position=g->entry.curve_sampler->evaluateDeBoorT(0.);
  body.velocity_in_frame=original_velocity;body.planar_speed=original_velocity.head<2>().norm();
  TrackerCore core(c);prepare(core,body,t);
  const auto out=core.step(100.05,10.05,true);
  EXPECT_FALSE(out.frozen);EXPECT_TRUE(core.active());EXPECT_GT(out.forward,0.);
  EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_NEAR(out.yaw_rate,0.,1e-12);
  EXPECT_NEAR(core.controlDiagnostic().heading_error,0.,1e-12);
  // This is the original negative acquired velocity and unchanged geometry,
  // not an invented stationary state or an advanced progress parameter.
  EXPECT_NEAR(core.odometry().velocity_in_frame.x(),-.1,1e-12);
  EXPECT_DOUBLE_EQ(core.progressArc(),0.);
  EXPECT_DOUBLE_EQ(g->entry.velocity_sampler->evaluateDeBoorT(0.).x(),original_velocity.x());
  // Fixed evaluator and Native share the original controls and knots.
  auto native=*g->entry.velocity;
  auto native_position=*g->entry.curve;
  auto native_heading=g->body_heading_contract;
  ASSERT_TRUE(scan_planner::resolveForwardEntryHeading(native_heading,g->entry.duration,
    t.body_heading_sampling_intervals,[&](double at){return native_position.evaluateDeBoorT(at);},
    [&](double at){return native.evaluateDeBoorT(at);},[]{return true;}));
  EXPECT_DOUBLE_EQ(native_heading.forward_entry_end,g->body_heading_contract.forward_entry_end);
  EXPECT_NEAR(native_heading.forward_entry_yaw,g->body_heading_contract.forward_entry_yaw,1e-12);
}

namespace {
Trajectory lateralAcquiredReverseCurve(double lateral_speed,double rotation=0.) {
  auto t=curve();const double prefix[]={.03,0.,-.03,-.03,-.015,.015,.06};
  const Eigen::Rotation2Dd rotate(rotation);
  for(std::size_t i=0;i<t.points.size();++i) {
    const double x=i<7?prefix[i]:.06+.06*(i-6);
    const auto xy=rotate*Eigen::Vector2d(x,lateral_speed*.3*(static_cast<double>(i)-1.));
    t.points[i]={xy.x(),xy.y(),.55};
  }
  for(std::size_t i=0;i<t.knots.size();++i)t.knots[i]=(static_cast<double>(i)-3.)*.3;
  t.body_heading_seed_yaw=rotation;t.body_heading_sampling_intervals=500;return t;
}
double shortestHeadingDifference(double a,double b) {return std::atan2(std::sin(a-b),std::cos(a-b));}
}

TEST(ForwardEntryGeometry, LateralAcquisitionDoesNotRotateForwardBodyToNearSideways) {
  for(double vy:{-.03,.03})for(double rotation:{0.,179.*M_PI/180.,-179.*M_PI/180.}) {
    SCOPED_TRACE(::testing::Message()<<"vy="<<vy<<" rotation="<<rotation);
    const auto c=continuousResponseConfig();const auto t=lateralAcquiredReverseCurve(vy,rotation);
    const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty())<<g->failure;
    ASSERT_TRUE(g->derivatives_valid);ASSERT_GT(g->body_heading_contract.forward_entry_end,0.);
    const auto original_v=g->entry.velocity_sampler->evaluateDeBoorT(0.);
    const Eigen::Vector2d acquired=Eigen::Rotation2Dd(-rotation)*original_v.head<2>();
    EXPECT_NEAR(acquired.x(),-.1,1e-12);EXPECT_NEAR(acquired.y(),vy,1e-12);
    // This is the existing short backwards boundary, NOT a sideways route.
    // A transverse acquired component must not pick the first vx>0 sample's
    // nearly 90-degree tangent as the robot's whole entry orientation.
    EXPECT_LT(std::abs(shortestHeadingDifference(g->bodyHeadingAt(0.),rotation)),.30);
    for(std::size_t i=0;i<t.points.size();++i)EXPECT_TRUE(g->original_controls[i].isApprox(t.points[i],0.));
    EXPECT_EQ(g->original_knots,t.knots);
    EXPECT_TRUE(g->entry.velocity_sampler->evaluateDeBoorT(0.).isApprox(original_v,0.));
  }
}

TEST(ForwardEntryGeometry, YawCapabilityEnvelopeUsesTheSameResolvedBodyHeading) {
  for(double vy:{-.03,.03}) {
    const auto c=continuousResponseConfig();const auto t=lateralAcquiredReverseCurve(vy);
    const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
    // The old first-positive-vx sample is exactly .48 s. Keep this input
    // fixed across implementations, rather than moving the test away from
    // its original 258/m raw-tangent singularity after a new prefix resolves.
    constexpr double sample=.48;
    const auto v=g->entry.velocity_sampler->evaluateDeBoorT(sample);
    const auto a=g->acceleration_sampler->evaluateDeBoorT(sample);
    const double raw_yaw_cap=curvatureForwardLimit(v,a,c.max_yaw_rate,c.max_speed);
    ASSERT_LT(raw_yaw_cap,.0021); // Reproduces the old 1.93 mm/s hidden cap.
    const double arc=curveArcAt(g->entry.times,g->entry.arcs,sample);
    const double accepted_cap=g->brakingLimitAt(arc,.5*c.max_acceleration);
    // Keep the translational-acceleration envelope and finite terminal stop;
    // this only catches a separate, inconsistent raw-tangent yaw constraint.
    EXPECT_GT(accepted_cap,.005)<<"raw_yaw_cap="<<raw_yaw_cap;
    EXPECT_LE(accepted_cap,c.max_speed);
  }
}

TEST(ForwardEntryGeometry, ProjectionAcrossRecoveryBoundaryDoesNotReintroduceCurvatureStep) {
  for(double vy:{-.03,.03}) {
    const auto c=forwardNavigationConfig();const auto t=lateralAcquiredReverseCurve(vy);
    const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
    const double boundary=g->body_heading_contract.forward_entry_end;
    constexpr double offset=.01;
    ASSERT_GT(boundary,offset);ASSERT_LT(boundary,g->entry.duration-offset);
    auto measured=odom();measured.position=g->entry.curve_sampler->evaluateDeBoorT(0.);
    measured.velocity_in_frame=g->entry.velocity_sampler->evaluateDeBoorT(0.);
    measured.planar_speed=measured.velocity_in_frame.head<2>().norm();
    TrackerCore core(c);prepare(core,measured,t);
    std::vector<double> curvatures,targets;int tick=0;
    // Advance actual measured progress through the same original curve.
    // The resolved spatial prefix is longer than the old first-positive-vx
    // instant; teleporting from s=0 to s=.4 would test admission limits, not
    // the heading transition. Do not enlarge the production search window.
    for(double at=.05;at<boundary-offset;at+=.05) {
      const double now=100.+(++tick)*.05;
      measured.stamp=measured.posterior_stamp=measured.imu_stamp=now;
      measured.source_stamp_ns=std::llround(now*1e9);
      measured.position=g->entry.curve_sampler->evaluateDeBoorT(at);
      measured.velocity_in_frame=g->entry.velocity_sampler->evaluateDeBoorT(at);
      measured.planar_speed=measured.velocity_in_frame.head<2>().norm();
      measured.yaw=g->bodyHeadingAt(at);measured.orientation=Eigen::AngleAxisd(measured.yaw,Eigen::Vector3d::UnitZ());
      ASSERT_TRUE(core.receiveOdom(measured,now,now-90.));core.refreshTaskLease(now-90.);
      const auto out=core.step(now,now-90.,true);ASSERT_FALSE(out.frozen)<<out.reason;
    }
    // Two distinct measured positions must survive the real projection
    // resolution; sub-micrometre input changes can project to the same point
    // and would fail to exercise the old discontinuous .48 s branch.
    for(double at:{boundary-offset,boundary+offset,boundary-offset,boundary+offset}) {
      const double now=100.+(++tick)*.05;
      measured.stamp=measured.posterior_stamp=measured.imu_stamp=now;
      measured.source_stamp_ns=std::llround(now*1e9);
      measured.position=g->entry.curve_sampler->evaluateDeBoorT(at);
      measured.velocity_in_frame=g->entry.velocity_sampler->evaluateDeBoorT(at);
      measured.planar_speed=measured.velocity_in_frame.head<2>().norm();
      measured.yaw=g->bodyHeadingAt(boundary);measured.orientation=Eigen::AngleAxisd(measured.yaw,Eigen::Vector3d::UnitZ());
      ASSERT_TRUE(core.receiveOdom(measured,now,now-90.));core.refreshTaskLease(now-90.);
      const auto out=core.step(now,now-90.,true);const auto& d=core.controlDiagnostic();
      ASSERT_FALSE(out.frozen)<<out.reason;ASSERT_TRUE(d.valid);
      ASSERT_EQ(core.progressTime()<boundary,at<boundary)
        <<"the measured projection must actually cross the recovery boundary";
      EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_EQ(core.trajectoryId(),t.id);
      EXPECT_EQ(core.odometry().source_stamp_ns,measured.source_stamp_ns);
      EXPECT_DOUBLE_EQ(core.odometry().stamp,measured.stamp);
      EXPECT_TRUE(core.odometry().velocity_in_frame.isApprox(measured.velocity_in_frame,0.));
      curvatures.push_back(d.steering_curvature);targets.push_back(d.yaw_target);
    }
    for(std::size_t i=1;i<curvatures.size();++i) {
      EXPECT_LT(std::abs(curvatures[i]-curvatures[i-1]),.001)
        <<"a continuous value of heading must also have a continuous spatial derivative";
      EXPECT_LT(std::abs(targets[i]-targets[i-1]),.01);
    }
  }
}

TEST(ForwardEntryGeometry, ControlCurvatureIsTheDerivativeOfNativeCertifiedHeading) {
  for(double vy:{-.03,.03}) {
    const auto c=forwardNavigationConfig();const auto t=lateralAcquiredReverseCurve(vy);
    const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
    auto measured=odom();measured.position=g->entry.curve_sampler->evaluateDeBoorT(0.);
    TrackerCore core(c);prepare(core,measured,t);int tick=0;
    const double end=g->body_heading_contract.forward_entry_end;
    for(double at:{.4,.46,.5,.75*end,end-.02,end+.02}) {
      const double now=100.+(++tick)*.05;
      measured.stamp=measured.posterior_stamp=measured.imu_stamp=now;
      measured.position=g->entry.curve_sampler->evaluateDeBoorT(at);
      measured.velocity_in_frame=g->entry.velocity_sampler->evaluateDeBoorT(at);
      measured.planar_speed=measured.velocity_in_frame.head<2>().norm();
      measured.yaw=g->bodyHeadingAt(at);measured.orientation=Eigen::AngleAxisd(measured.yaw,Eigen::Vector3d::UnitZ());
      ASSERT_TRUE(core.receiveOdom(measured,now,now-90.));core.refreshTaskLease(now-90.);
      const auto out=core.step(now,now-90.,true);ASSERT_FALSE(out.frozen)<<out.reason;
      const double projected=core.progressTime(),epsilon=1e-6;
      ASSERT_GT(projected,epsilon);ASSERT_LT(projected,g->entry.duration-epsilon);
      const auto velocity=g->entry.velocity_sampler->evaluateDeBoorT(projected);
      ASSERT_GT(velocity.head<2>().norm(),.02);
      const double geometric_curvature=shortestHeadingDifference(g->bodyHeadingAt(projected+epsilon),
        g->bodyHeadingAt(projected-epsilon))/(2.*epsilon*velocity.head<2>().norm());
      EXPECT_NEAR(core.controlDiagnostic().steering_curvature,geometric_curvature,2e-4)
        <<"curve_time="<<projected;
    }
  }
}

TEST(ForwardEntryGeometry, SameTaskReplacementKeepsOneConditionerHistoryAndOriginalAcquiredState) {
  const auto c=continuousResponseConfig();const auto t=lateralAcquiredReverseCurve(.03);
  const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
  auto body=odom();body.position=g->entry.curve_sampler->evaluateDeBoorT(0.);
  body.velocity_in_frame=g->entry.velocity_sampler->evaluateDeBoorT(0.);
  body.planar_speed=body.velocity_in_frame.head<2>().norm();
  TrackerCore fixed(c),refreshed(c);prepare(fixed,body,t);prepare(refreshed,body,t);
  CommandHistory history;
  for(int i=1;i<=16;++i) {
    const double now=100.+i*.05;body.stamp=body.posterior_stamp=body.imu_stamp=now;
    body.source_stamp_ns=std::llround(now*1e9);
    ASSERT_TRUE(fixed.receiveOdom(body,now,now-90.));ASSERT_TRUE(refreshed.receiveOdom(body,now,now-90.));
    fixed.refreshTaskLease(now-90.);refreshed.refreshTaskLease(now-90.);
    if(i%2==0) {
      auto replacement=t;replacement.id=i;replacement.start_time=now;
      ASSERT_TRUE(refreshed.receiveTrajectory(replacement,now,now-90.))<<refreshed.candidateReason();
    }
    const auto expected=fixed.step(now,now-90.,true),out=refreshed.step(now,now-90.,true);
    ASSERT_FALSE(out.frozen)<<out.reason;EXPECT_GT(out.forward,0.);
    EXPECT_NEAR(out.forward,expected.forward,1e-12);EXPECT_NEAR(out.yaw_rate,expected.yaw_rate,1e-12);
    expectConditionedStep(c,out,history);
    EXPECT_EQ(refreshed.odometry().source_stamp_ns,body.source_stamp_ns);
    EXPECT_DOUBLE_EQ(refreshed.odometry().stamp,body.stamp);
    EXPECT_TRUE(refreshed.odometry().velocity_in_frame.isApprox(body.velocity_in_frame,0.));
    EXPECT_EQ(refreshed.task().generation,task().generation);
  }
}

TEST(ForwardNavigation, MovingIntoGoalToleranceKeepsNormalAccelerationBound) {
  auto c=forwardNavigationConfig();c.external_goal_completion=true;
  auto goal=task();goal.goal.x()=.05;auto body=odom();body.position.x()=.05;
  body.planar_speed=.25;body.velocity_in_frame.x()=.25;
  TrackerCore core(c);prepare(core,body,curve(),goal);core.recordAppliedOutput(.20,0.,.12,100.,10.);
  const auto out=core.step(100.05,10.05,true);
  EXPECT_EQ(out.reason,"goal_position_settling");EXPECT_TRUE(core.active());EXPECT_FALSE(out.finished);
  EXPECT_NEAR(out.forward,.20-c.max_acceleration*.05,1e-12);
  EXPECT_NEAR(out.yaw_rate,.12-c.max_yaw_acceleration*.05,1e-12);
  EXPECT_DOUBLE_EQ(out.lateral,0.);
}

TEST(ForwardNavigation, TwentyHzGoalDecelerationThenMeasuredRestIsContinuousAndExactZero) {
  auto c=forwardNavigationConfig();c.external_goal_completion=true;
  auto goal=task();goal.goal.x()=.05;auto body=odom();body.position.x()=.05;
  body.planar_speed=.20;body.velocity_in_frame.x()=.20;body.angular_velocity_in_frame.z()=.12;
  TrackerCore core(c);prepare(core,body,curve(),goal);core.recordAppliedOutput(.20,0.,.12,100.,10.);
  Output previous;previous.forward=.20;previous.yaw_rate=.12;bool reached=false;
  for(int i=1;i<=20;++i) {
    const double now=100.+i*.05,receipt=now-90.;
    body.stamp=body.posterior_stamp=body.imu_stamp=now;
    ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
    const auto out=core.step(now,receipt,true);
    EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_TRUE(core.active());EXPECT_FALSE(out.finished);
    if(out.reason=="goal_position_reached") {
      EXPECT_LE(body.planar_speed,c.stationary_linear_threshold_mps);
      EXPECT_LE(std::abs(body.angular_velocity_in_frame.z()),c.stationary_angular_threshold_radps);
      EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_DOUBLE_EQ(out.yaw_rate,0.);reached=true;break;
    }
    EXPECT_EQ(out.reason,"goal_position_settling");
    EXPECT_LE(std::abs(out.forward-previous.forward),c.max_acceleration*.05+1e-12);
    EXPECT_LE(std::abs(out.yaw_rate-previous.yaw_rate),c.max_yaw_acceleration*.05+1e-12);
    EXPECT_GE(out.forward,0.);EXPECT_GE(out.yaw_rate,0.);
    // Kinematic response is explicitly driven by the controller output,
    // rather than a recorded robot motion being relabelled as success.
    body.position.x()+=out.forward*.05;body.yaw+=out.yaw_rate*.05;
    body.orientation=Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ());
    body.planar_speed=out.forward;body.velocity_in_frame={out.forward,0.,0.};
    body.angular_velocity_in_frame.z()=out.yaw_rate;previous=out;
  }
  EXPECT_TRUE(reached);
}

TEST(HolonomicControl, FreshOppositeCurveImmediatelyRequestsNewSteeringWithoutResettingForwardHistory) {
  auto c=config();c.align_then_forward=true;c.kp_yaw=.8;c.yaw_damping_gain=.35;
  TrackerCore core(c);prepare(core,odom(100.,-.25));
  const auto first=core.step(100.05,10.05,true);ASSERT_GT(first.yaw_rate,0.);ASSERT_GT(first.forward,0.);
  auto replacement=curve(2);replacement.start_time=100.1;
  for(auto& p:replacement.points)p.head<2>()=Eigen::Rotation2Dd(-.3)*p.head<2>();
  auto body=odom(100.1,.10);body.angular_velocity_in_frame.z()=.30;
  ASSERT_TRUE(core.receiveOdom(body,100.1,10.1));core.refreshTaskLease(10.1);
  ASSERT_TRUE(core.receiveTrajectory(replacement,100.1,10.1));
  auto out=core.step(100.1,10.1,true);EXPECT_GT(out.forward,0.);
  EXPECT_LT(core.controlDiagnostic().yaw_target,0.);
  EXPECT_LE(std::abs(out.yaw_rate-first.yaw_rate),c.max_yaw_acceleration*.05+1e-12);
  EXPECT_FALSE(out.frozen);EXPECT_LE(std::abs(out.forward-first.forward),c.max_acceleration*.05+1e-12);
  body=odom(100.15,.13);body.angular_velocity_in_frame.z()=.25;
  ASSERT_TRUE(core.receiveOdom(body,100.15,10.15));core.refreshTaskLease(10.15);
  out=core.step(100.15,10.15,true);EXPECT_LT(out.yaw_rate,0.);EXPECT_GT(out.forward,0.);
  EXPECT_LT(core.controlDiagnostic().yaw_target,0.); // Do not wait for measured old rotation to vanish.
  body=odom(100.2,.13);
  ASSERT_TRUE(core.receiveOdom(body,100.2,10.2));core.refreshTaskLease(10.2);
  out=core.step(100.2,10.2,true);EXPECT_LT(out.yaw_rate,0.);EXPECT_GT(out.forward,0.);
  EXPECT_FALSE(out.frozen);EXPECT_LE(std::abs(out.yaw_rate),2.*c.max_yaw_acceleration*.05+1e-12);
  core.cancel("operator_cancel");out=core.step(100.25,10.25,true);
  EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_DOUBLE_EQ(out.yaw_rate,0.);
}

TEST(HolonomicControl, TwentyHzMeasuredAlignmentKeepsJerkHistoryAcrossSameTaskCurvesAndNeverFreezes) {
  const auto c=productionSteeringConfig();TrackerCore unchanged(c),refreshed(c);
  prepare(unchanged,odom(100.,-.22));prepare(refreshed,odom(100.,-.22));
  Output previous;double previous_acceleration=0.;bool observed_coast=false,reached_zero=false;
  for(int i=1;i<=50;++i) {
    const double now=100.+i*.05;auto body=odom(now,i<=10?-.22:0.);
    // Actual heading is aligned. Release steering through the existing
    // conditioner, not an arbitrary angle latch or predicted inertia horizon.
    for(auto* core:{&unchanged,&refreshed}) {
      ASSERT_TRUE(core->receiveOdom(body,now,now-90.));core->refreshTaskLease(now-90.);
    }
    if(i%3==0) {
      auto replacement=curve(i+1);replacement.start_time=now;
      ASSERT_TRUE(refreshed.receiveTrajectory(replacement,now,now-90.))<<refreshed.candidateReason();
    }
    const auto out=refreshed.step(now,now-90.,true),other=unchanged.step(now,now-90.,true);
    EXPECT_NEAR(out.yaw_rate,other.yaw_rate,1e-12);EXPECT_NEAR(out.forward,other.forward,1e-12);
    EXPECT_FALSE(out.frozen);EXPECT_TRUE(refreshed.active());EXPECT_GT(out.forward,0.);
    EXPECT_GE(out.yaw_rate,0.);EXPECT_LE(std::abs(out.yaw_rate-previous.yaw_rate),c.max_yaw_acceleration*.05+1e-12);
    const double acceleration=(out.yaw_rate-previous.yaw_rate)/.05;
    EXPECT_LE(std::abs(acceleration-previous_acceleration),c.max_yaw_jerk*.05+1e-10);
    if(i>10) {
      EXPECT_DOUBLE_EQ(refreshed.controlDiagnostic().yaw_target,0.);
      if(out.yaw_rate!=0.)observed_coast=true;
      else reached_zero=true;
    }
    previous=out;previous_acceleration=acceleration;
  }
  EXPECT_TRUE(observed_coast);EXPECT_TRUE(reached_zero);
  refreshed.cancel("operator_cancel");const auto stop=refreshed.step(102.55,12.55,true);
  EXPECT_DOUBLE_EQ(stop.forward,0.);EXPECT_DOUBLE_EQ(stop.lateral,0.);EXPECT_DOUBLE_EQ(stop.yaw_rate,0.);
}

TEST(HolonomicControl, TwentyHzOppositeYawRequestCrossesContinuouslyWithoutExtraZeroPhase) {
  const auto c=productionSteeringConfig();TrackerCore core(c);prepare(core,odom(100.,-.25));
  Output previous;double previous_acceleration=0.;bool reversed=false,old_sign_decelerated=false;
  for(int i=1;i<=65;++i) {
    const double now=100.+i*.05;auto body=odom(now,i<=10?-.25:.10);
    ASSERT_TRUE(core.receiveOdom(body,now,now-90.));core.refreshTaskLease(now-90.);
    if(i==11) {
      auto replacement=curve(2);replacement.start_time=now;
      for(auto& p:replacement.points)p.head<2>()=Eigen::Rotation2Dd(-.3)*p.head<2>();
      ASSERT_TRUE(core.receiveTrajectory(replacement,now,now-90.));
    }
    const auto out=core.step(now,now-90.,true);
    EXPECT_GT(out.forward,0.);EXPECT_FALSE(out.frozen);EXPECT_TRUE(core.active());
    EXPECT_LE(std::abs(out.yaw_rate-previous.yaw_rate),c.max_yaw_acceleration*.05+1e-12);
    const double acceleration=(out.yaw_rate-previous.yaw_rate)/.05;
    EXPECT_LE(std::abs(acceleration-previous_acceleration),c.max_yaw_jerk*.05+1e-10);
    if(i>10) {
      EXPECT_LT(core.controlDiagnostic().yaw_target,0.);
      if(out.yaw_rate>0.)old_sign_decelerated=true;
      if(out.yaw_rate<0.)reversed=true;
    }
    previous=out;previous_acceleration=acceleration;
  }
  EXPECT_TRUE(old_sign_decelerated);EXPECT_TRUE(reversed);
  core.hold("fixture_source_failure",13.3);const auto stop=core.step(103.3,13.3,true);
  EXPECT_DOUBLE_EQ(stop.forward,0.);EXPECT_DOUBLE_EQ(stop.lateral,0.);EXPECT_DOUBLE_EQ(stop.yaw_rate,0.);
}

TEST(HolonomicControl, FinalYawUsesIndependentToleranceAndNormalSlewWithoutAssumedCoasting) {
  auto c=config();c.align_then_forward=true;c.external_goal_completion=true;
  c.kp_yaw=.8;c.yaw_damping_gain=.35;c.lookahead=.8;
  auto body=odom(100.,-.22);body.angular_velocity_in_frame.z()=.30;
  TrackerCore core(c);prepare(core,body);core.recordAppliedOutput(0.,0.,.08,100.,10.);
  auto out=core.align(0.,.07,100.05,10.05,true);
  EXPECT_EQ(out.reason,"aligning");EXPECT_TRUE(out.frozen);EXPECT_GT(out.yaw_rate,0.);
  EXPECT_LE(std::abs(out.yaw_rate-.08),c.max_yaw_acceleration*.05+1e-12);
  EXPECT_GT(std::abs(body.yaw),.07);EXPECT_TRUE(core.active());EXPECT_FALSE(out.finished);
  body=odom(100.1,.12);body.angular_velocity_in_frame.z()=.25;
  ASSERT_TRUE(core.receiveOdom(body,100.1,10.1));core.refreshTaskLease(10.1);
  const auto previous=out;
  out=core.align(0.,.07,100.1,10.1,true);
  EXPECT_LT(out.yaw_rate,previous.yaw_rate);
  EXPECT_LE(std::abs(out.yaw_rate-previous.yaw_rate),c.max_yaw_acceleration*.05+1e-12);
  body=odom(100.15,.12);ASSERT_TRUE(core.receiveOdom(body,100.15,10.15));core.refreshTaskLease(10.15);
  out=core.align(0.,.07,100.15,10.15,true);EXPECT_LT(out.yaw_rate,0.); // Outside tolerance is not falsely completed.
  body=odom(100.2,.06);body.angular_velocity_in_frame.z()=-.25;
  ASSERT_TRUE(core.receiveOdom(body,100.2,10.2));core.refreshTaskLease(10.2);
  const auto turning=out;out=core.align(0.,.07,100.2,10.2,true);
  EXPECT_LE(std::abs(out.yaw_rate-turning.yaw_rate),c.max_yaw_acceleration*.05+1e-12);
  EXPECT_TRUE(core.active());EXPECT_FALSE(out.finished); // Actual stationary evidence remains outside this helper.
}

TEST(HolonomicControl, TerminalYawNeverRestoresOldPlanarApproachAfterMeasuredTranslationSettled) {
  for(bool forward_strategy:{false,true}) {
    auto c=config();c.align_then_forward=forward_strategy;c.external_goal_completion=true;
    c.kp_yaw=.8;c.max_acceleration=.2;c.max_yaw_acceleration=.4;
    auto goal=task();goal.goal.x()=.05;auto body=odom(100.,-.30);body.position.x()=.05;
    TrackerCore core(c);prepare(core,body,curve(),goal);
    // Prior accepted SI approach command is NOT a measured velocity: fresh
    // odometry above says that physical translation has already settled.
    core.recordAppliedOutput(.10,.04,.08,100.,10.);
    const auto out=core.align(0.,.07,100.05,10.05,true);
    EXPECT_EQ(out.reason,"aligning");EXPECT_GT(out.yaw_rate,0.);
    EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
    EXPECT_TRUE(core.active());EXPECT_FALSE(out.finished);EXPECT_GT(std::abs(body.yaw),.07);
  }
}

TEST(HolonomicAuthorLaw, MeasuredSettledGoalNeverEmitsSavedApproachOrYawTail) {
  auto c=authorLikeConfig();c.external_goal_completion=true;auto goal=task();goal.goal.x()=.05;
  for(bool align_goal:{false,true}) {
    auto body=odom(100.,align_goal?.06:0.);body.position.x()=.05;
    TrackerCore core(c);prepare(core,body,curve(),goal);core.recordAppliedOutput(.10,.04,.20,100.,10.);
    body.stamp=body.posterior_stamp=body.imu_stamp=100.05;
    ASSERT_TRUE(core.receiveOdom(body,100.05,10.05));core.refreshTaskLease(10.05);
    const auto out=align_goal?core.align(0.,.07,100.05,10.05,true):core.step(100.05,10.05,true);
    EXPECT_DOUBLE_EQ(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_DOUBLE_EQ(out.yaw_rate,0.);
    EXPECT_TRUE(out.frozen);EXPECT_TRUE(core.active());EXPECT_TRUE(core.hasInstalledGeometry());EXPECT_FALSE(out.finished);
    EXPECT_EQ(out.reason,align_goal?"aligning":"goal_position_reached");
  }
}

TEST(HolonomicAuthorLaw, GoalPoseWithoutMeasuredRestKeepsNormalDecelerationAndNoFalseCompletion) {
  auto c=authorLikeConfig();c.external_goal_completion=true;auto goal=task();goal.goal.x()=.05;
  for(bool angular_motion:{false,true}) {
    auto body=odom();body.position.x()=.05;
    if(angular_motion)body.angular_velocity_in_frame.z()=c.stationary_angular_threshold_radps+.001;
    else {body.velocity_in_frame.x()=c.stationary_linear_threshold_mps+.001;body.planar_speed=body.velocity_in_frame.x();}
    TrackerCore core(c);prepare(core,body,curve(),goal);core.recordAppliedOutput(.10,0.,.20,100.,10.);
    const auto out=core.step(100.05,10.05,true);
    EXPECT_NEAR(out.forward,.10-c.max_acceleration*.05,1e-12);
    EXPECT_NEAR(out.yaw_rate,.20-c.max_yaw_acceleration*.05,1e-12);
    EXPECT_EQ(out.reason,"goal_position_settling");EXPECT_TRUE(core.active());EXPECT_FALSE(out.finished);
  }
}

TEST(HolonomicAuthorLaw, SettledGoalZeroPrecedesEndpointWaitWithoutClearingReseedFacts) {
  for(bool final_endpoint:{false,true}) {
    auto c=authorLikeConfig();c.external_goal_completion=true;auto goal=task();
    goal.goal.x()=final_endpoint?1.:1.21;auto body=odom();TrackerCore core(c);prepare(core,body,curve(),goal);
    // Bring the actual projection to the current curve's endpoint without
    // advancing it by a timer or changing the task goal/local geometry.
    for(int i=1;i<=7;++i) {
      const double now=100.+i*.05;body=odom(now);body.position.x()=std::min(1.,i*.15);
      body.velocity_in_frame.x()=.1;body.planar_speed=.1;
      ASSERT_TRUE(core.receiveOdom(body,now,now-90.));core.refreshTaskLease(now-90.);
      ASSERT_TRUE(core.refreshMeasuredProgress(now));
    }
    core.recordAppliedOutput(.10,.04,.20,100.35,10.35);
    if(!final_endpoint) {
      body.stamp=body.posterior_stamp=body.imu_stamp=100.4;
      ASSERT_TRUE(core.receiveOdom(body,100.4,10.4));core.refreshTaskLease(10.4);
      EXPECT_EQ(core.step(100.4,10.4,true).reason,"local_segment_finished_waiting_replan");
      EXPECT_TRUE(core.needsTerminalReplan(100.4)); // A real local-end latch.
    }
    const double now=final_endpoint?100.4:100.45;body=odom(now);body.position.x()=final_endpoint?1.16:1.05;
    ASSERT_TRUE(core.receiveOdom(body,now,now-90.));core.refreshTaskLease(now-90.);
    ASSERT_TRUE(core.needsTerminalReplan(now)); // .16 outside connector, or original local-end wait.
    const auto out=core.step(now,now-90.,true);
    EXPECT_EQ(out.reason,"goal_position_reached");EXPECT_DOUBLE_EQ(out.forward,0.);
    EXPECT_DOUBLE_EQ(out.lateral,0.);EXPECT_DOUBLE_EQ(out.yaw_rate,0.);
    EXPECT_TRUE(core.needsTerminalReplan(now)); // Zero does not forge a renewed geometric certificate.
    EXPECT_TRUE(core.active());EXPECT_TRUE(core.hasInstalledGeometry());EXPECT_FALSE(out.finished);
  }
}

TEST(ContinuousSteering, GlobalGoalDecelerationPrecedesLocalEndpointReseedWithoutClaimingRest) {
  for(bool global_goal_near:{false,true}) {
    auto c=continuousResponseConfig();c.external_goal_completion=true;
    auto goal=task();goal.goal.x()=global_goal_near?1.:2.;
    auto body=odom();TrackerCore core(c);prepare(core,body,curve(),goal);
    for(int i=1;i<=7;++i) {
      const double now=100.+i*.05;body=odom(now);body.position.x()=std::min(1.,i*.15);
      body.velocity_in_frame.x()=.05;body.planar_speed=.05;
      ASSERT_TRUE(core.receiveOdom(body,now,now-90.));core.refreshTaskLease(now-90.);
      ASSERT_TRUE(core.refreshMeasuredProgress(now));
    }
    core.recordAppliedOutput(.10,0.,.10,100.35,10.35);
    body=odom(100.4);body.position.x()=1.16;body.velocity_in_frame.x()=.05;
    body.planar_speed=.05;body.angular_velocity_in_frame.z()=.10;
    ASSERT_TRUE(core.receiveOdom(body,100.4,10.4));core.refreshTaskLease(10.4);
    ASSERT_TRUE(core.needsTerminalReplan(100.4)); // Original .15 m connector is unchanged.
    const auto out=core.step(100.4,10.4,true);
    EXPECT_EQ(out.reason,global_goal_near?"goal_position_settling":"local_segment_finished_waiting_replan");
    EXPECT_TRUE(core.active());EXPECT_FALSE(out.finished);
    EXPECT_LE(std::abs(out.forward-.10),c.max_acceleration*.05+1e-12);
    EXPECT_LE(std::abs(out.yaw_rate-.10),c.max_yaw_acceleration*.05+1e-12);
    EXPECT_DOUBLE_EQ(out.lateral,0.);
  }
}

TEST(HolonomicControl, ZeroNativeEntryStartsOnlyAlongTheSameCurveWithoutTimerProgress) {
  for(int direction=0;direction<3;++direction) {
    SCOPED_TRACE(direction);auto t=curve();
    for(std::size_t i=0;i<t.points.size();++i) {
      const double distance=i<3?0.:.001*(i-2)*(i-2);
      t.points[i].x()=direction==0?distance:direction==1?0.:-distance;
      t.points[i].y()=direction==1?distance:0.;
    }
    auto body=odom(100.,1.2);TrackerCore core(config());prepare(core,body,t);
    const auto out=core.step(100.05,10.05);
    ASSERT_EQ(out.reason,"tracking");EXPECT_GT(planar(out).norm(),0.);
    const Eigen::Vector2d world=Eigen::Rotation2Dd(body.yaw)*planar(out);
    EXPECT_NEAR(world[direction==1?0:1],0.,1e-12);
    EXPECT_GT((direction==2?-1.:1.)*world[direction==1?1:0],0.);
    EXPECT_LE(planar(out).norm(),config().max_acceleration*.05+1e-12);
    EXPECT_NEAR(out.yaw_rate,0.,1e-12); // Zero native tangent does not invent a new yaw.
    core.step(100.1,10.1);
    EXPECT_DOUBLE_EQ(core.progressTime(),0.);EXPECT_DOUBLE_EQ(core.progressArc(),0.);
  }
}

TEST(HolonomicControl, TwentyHzZeroStartClosedLoopsActuallyMoveAndReachWithoutSpeedJump) {
  for(int direction=0;direction<3;++direction) {
    SCOPED_TRACE(direction);auto c=config();c.holonomic_yaw_policy="hold_heading";
    c.external_goal_completion=true;auto t=curve();t.frozen_yaw=1.2;
    for(std::size_t i=0;i<t.points.size();++i) {
      const double distance=i<3?0.:.001*(i-2)*(i-2);
      t.points[i].x()=direction==0?distance:direction==1?0.:-distance;
      t.points[i].y()=direction==1?distance:0.;
    }
    auto goal=task();goal.goal=direction==0?Eigen::Vector3d(.4,0.,.55):
      direction==1?Eigen::Vector3d(0.,.4,.55):Eigen::Vector3d(-.4,0.,.55);
    auto body=odom(100.,1.2);TrackerCore core(c);prepare(core,body,t,goal);
    Output previous;bool reached=false;
    for(int i=1;i<=300;++i) {
      const double now=100.+i*.05,receipt=10.+i*.05;
      body.stamp=body.posterior_stamp=body.imu_stamp=now;
      ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
      const auto out=core.step(now,receipt,true);
      EXPECT_LE(planar(out).norm(),c.max_speed+1e-12);
      expectNormalSlewOrMeasuredGoalZero(out,Eigen::Rotation2Dd(body.yaw)*planar(previous),body,goal,c,.05);
      EXPECT_NEAR(out.yaw_rate,0.,1e-12);
      const Eigen::Vector2d world=Eigen::Rotation2Dd(body.yaw)*planar(out);
      body.position.head<2>()+=world*.05;body.velocity_in_frame={world.x(),world.y(),0.};
      body.planar_speed=world.norm();previous=out;
      if(out.reason=="goal_position_reached"&&planar(out).norm()<1e-9){reached=true;break;}
    }
    EXPECT_TRUE(reached);EXPECT_GT(core.progressArc(),.1);
    EXPECT_LE((body.position-goal.goal).norm(),c.goal_tolerance);
  }
}

TEST(HolonomicControl, CurvedZeroEntryUsesLocalTangentNotThePreviewChord) {
  auto t=curve();
  for(std::size_t i=0;i<t.points.size();++i) {
    const double distance=i<3?0.:.001*(i-2)*(i-2);
    t.points[i].x()=distance;t.points[i].y()=distance*distance;
  }
  const auto geometry=PreparedGeometry::build(config(),t,{});ASSERT_TRUE(geometry->failure.empty());
  ASSERT_TRUE(geometry->derivatives_valid);auto body=odom();TrackerCore core(config());prepare(core,body,t);
  const auto out=core.step(100.05,10.05);ASSERT_GT(out.forward,0.);
  EXPECT_GT(out.lateral,0.);EXPECT_LE(out.lateral/out.forward,.01+1e-12);
  EXPECT_NEAR(out.yaw_rate,0.,1e-12);EXPECT_DOUBLE_EQ(core.progressArc(),0.);
  // The future 5 cm chord turns substantially farther than the bounded local
  // prefix (shared reliable direction within 2 cm). It must not become the
  // instantaneous body-yaw target or a far-ahead translation chord.
  const auto preview=geometry->entry.curve_sampler->evaluateDeBoorT(1.6);
  ASSERT_GT(preview.x(),.04);EXPECT_GT(preview.y()/preview.x(),out.lateral/out.forward*5.);
  auto c=config();c.holonomic_yaw_policy="hold_heading";t.frozen_yaw=0.;
  c.external_goal_completion=true;auto goal=task();goal.goal={.4,.16,.55};
  TrackerCore loop(c);prepare(loop,body,t,goal);Output previous;bool reached=false;double max_error=0.;
  for(int i=1;i<=300;++i) {
    const double now=100.+i*.05,receipt=10.+i*.05;
    body.stamp=body.posterior_stamp=body.imu_stamp=now;
    ASSERT_TRUE(loop.receiveOdom(body,now,receipt));loop.refreshTaskLease(receipt);
    const auto output=loop.step(now,receipt,true);
    expectNormalSlewOrMeasuredGoalZero(output,planar(previous),body,goal,c,.05);
    EXPECT_LE(planar(output).norm(),c.max_speed+1e-12);
    const auto position=geometry->entry.curve_sampler->evaluateDeBoorT(loop.progressTime());
    if(output.reason=="tracking")max_error=std::max(max_error,(position-body.position).head<2>().norm());
    body.position.head<2>()+=planar(output)*.05;
    body.velocity_in_frame={output.forward,output.lateral,0.};body.planar_speed=planar(output).norm();previous=output;
    if(output.reason=="goal_position_reached"&&planar(output).norm()<1e-9){reached=true;break;}
  }
  EXPECT_TRUE(reached);EXPECT_LT(max_error,.006);
  RecordProperty("zero_start_bend_peak_cross_track_m",std::to_string(max_error));
}

TEST(HolonomicControl, SpatialStartupDoesNotInventMotionOnZeroCurveOrPastLocalEndpoint) {
  for(bool terminal:{false,true}) {
    SCOPED_TRACE(terminal);auto t=curve();
    if(!terminal)for(auto& point:t.points)point.x()=0.;
    const auto geometry=PreparedGeometry::build(config(),t,{});
    if(!terminal) {
      EXPECT_EQ(geometry->failure,"trajectory_arc_degenerate");
      TrackerCore rejected(config());ASSERT_TRUE(rejected.receiveTask(task(),100.,10.));
      ASSERT_TRUE(rejected.receiveOdom(odom(),100.,10.));EXPECT_FALSE(rejected.receiveTrajectory(t,100.,10.));
      EXPECT_FALSE(rejected.hasInstalledGeometry());EXPECT_DOUBLE_EQ(planar(rejected.step(100.05,10.05)).norm(),0.);
      continue;
    }
    ASSERT_TRUE(geometry->failure.empty());
    auto body=odom();TrackerCore core(config());prepare(core,body,t);
    if(terminal) {
      for(int i=1;i<=6;++i) {
        body.position.x()=std::min(.15*i,geometry->entry.points.back().x());body.stamp=100.+i*.05;
        ASSERT_TRUE(core.receiveOdom(body,body.stamp,10.+i*.05));core.step(body.stamp,10.+i*.05);
      }
      body.position=geometry->entry.points.back();body.stamp=100.35;
      ASSERT_TRUE(core.receiveOdom(body,body.stamp,10.35));core.recordAppliedOutput(0.,0.,0.,100.35,10.35);
    }
    for(int i=1;i<=5;++i) {
      const double now=terminal?100.35+.05*i:100.+.05*i;
      body.stamp=now;ASSERT_TRUE(core.receiveOdom(body,now,now-90.));core.refreshTaskLease(now-90.);
      const auto out=core.step(now,now-90.);
      EXPECT_EQ(out.reason,"local_segment_finished_waiting_replan");EXPECT_DOUBLE_EQ(planar(out).norm(),0.);
      EXPECT_DOUBLE_EQ(out.yaw_rate,0.);EXPECT_FALSE(out.finished);
    }
  }
}

TEST(HolonomicControl, ShortZeroStartCurveUsesInteriorNativeVelocityNotItsStoppingDerivative) {
  auto t=curve();
  for(std::size_t i=0;i<t.points.size();++i)t.points[i].x()=i<3?0.:std::min(.04,.000125*(i-2)*(i-2));
  t.points[20].x()=t.points[21].x()=t.points[22].x()=.04;
  const auto geometry=PreparedGeometry::build(config(),t,{});ASSERT_TRUE(geometry->failure.empty());
  ASSERT_DOUBLE_EQ(geometry->entry.velocity_sampler->evaluateDeBoorT(0.).norm(),0.);
  ASSERT_DOUBLE_EQ(geometry->entry.velocity_sampler->evaluateDeBoorT(4.).norm(),0.);
  TrackerCore core(config());prepare(core,odom(),t);const auto out=core.step(100.05,10.05);
  EXPECT_EQ(out.reason,"tracking");EXPECT_GT(out.forward,0.);
  EXPECT_LE(out.forward,config().kp_position*.02+1e-12);EXPECT_DOUBLE_EQ(core.progressArc(),0.);
}

TEST(HolonomicControl, LocalEndpointDoesNotConsumeTheTaskGoalTolerance) {
  auto body=odom();TrackerCore core(config());prepare(core);
  for(int i=1;i<=4;++i) {
    body.position.x()=i<4?i*.2:.82;body.stamp=100.+i*.05;
    ASSERT_TRUE(core.receiveOdom(body,body.stamp,10.+i*.05));
    core.step(body.stamp,10.+i*.05);
  }
  core.recordAppliedOutput(.20,0.,0.,100.2,10.2);
  const auto out=core.step(100.25,10.25);
  EXPECT_EQ(out.reason,"tracking");EXPECT_GT(out.forward,.19);
  EXPECT_FALSE(out.frozen);
}

TEST(HolonomicControl, NearStationaryTangentsPreserveHeadingWithoutSuppressingTranslation) {
  // Exercise the actual controller, not a second copy of the native heading
  // formula. Opposite mm/s tangents must not make a stationary body spin; a
  // trustworthy tangent above the shared 0.02 m/s boundary still steers yaw.
  for(double speed:{-.019,-.001,.001,.019,.021}) {
    auto t=curve();
    for(auto& point:t.points)point.x()*=speed/.25;
    TrackerCore core(config());prepare(core,odom(100.,.4),t);
    const auto out=core.step(100.05,10.05);
    ASSERT_EQ(out.reason,"tracking")<<speed;
    EXPECT_GT(planar(out).norm(),0.)<<speed;
    const Eigen::Vector2d world=Eigen::Rotation2Dd(.4)*planar(out);
    EXPECT_GT(world.x()*speed,0.)<<speed;
    EXPECT_NEAR(world.y(),0.,1e-12)<<speed;
    if(std::abs(speed)<.02)EXPECT_DOUBLE_EQ(out.yaw_rate,0.)<<speed;
    else EXPECT_LT(out.yaw_rate,0.)<<speed;
  }
}

TEST(HolonomicControl, HoldHeadingConsumesNativeFrozenYawNotLatestBody) {
  auto c=config();c.holonomic_yaw_policy="hold_heading";
  auto t=curve();t.frozen_yaw=.2;TrackerCore core(c);prepare(core,odom(100.,1.),t);
  const auto out=core.step(100.05,10.05);EXPECT_LT(out.yaw_rate,0.);EXPECT_LT(out.lateral,0.);
  auto cached=PreparedGeometry::build(c,t,{});ASSERT_TRUE(cached->failure.empty());
  t.prepared=cached;t.frozen_yaw=.3;EXPECT_FALSE(cached->matches(c,t));
  c.holonomic_yaw_policy="path_tangent";t.frozen_yaw=.2;EXPECT_FALSE(cached->matches(c,t));
}

TEST(HolonomicControl, CurvatureUsesTranslationAccelerationNotBodyYawLimit) {
  const Eigen::Vector3d v(.3,0.,0.),a(0.,.3,0.);
  const double cap=curvaturePlanarLimit(v,a,.175,.3);
  EXPECT_NEAR(cap,std::sqrt(.175*.3),1e-12);EXPECT_LE(cap*cap/.3,.175+1e-12);
  auto t=curve();for(auto& p:t.points)p.y()=.2*p.x()*p.x();
  auto c=config();c.max_yaw_rate=.1;const auto slow=PreparedGeometry::build(c,t,{});
  c.max_yaw_rate=1.;const auto fast=PreparedGeometry::build(c,t,{});
  ASSERT_TRUE(slow->failure.empty());ASSERT_TRUE(fast->failure.empty());
  EXPECT_EQ(slow->speed_envelope,fast->speed_envelope);
}

TEST(ForwardNavigation, TightCurveSpeedRespectsYawRateWithoutChangingHolonomicStrategies) {
  auto t=curve();for(auto& p:t.points)p.y()=2.*p.x()*p.x();
  const auto forward=forwardNavigationConfig();
  const auto limited=PreparedGeometry::build(forward,t,{});ASSERT_TRUE(limited->failure.empty());
  auto author=authorLikeConfig();const auto author_geometry=PreparedGeometry::build(author,t,{});
  ASSERT_TRUE(author_geometry->failure.empty());bool yaw_cap_is_active=false;
  for(std::size_t i=0;i<limited->entry.times.size();++i) {
    const auto v=limited->entry.velocity_sampler->evaluateDeBoorT(limited->entry.times[i]);
    const auto a=limited->acceleration_sampler->evaluateDeBoorT(limited->entry.times[i]);
    const double speed=v.head<2>().norm();if(speed<1e-5)continue;
    const double curvature=std::abs(v.x()*a.y()-v.y()*a.x())/(speed*speed*speed);
    const double cap=limited->brakingLimitAt(limited->entry.arcs[i],.5*forward.max_acceleration);
    EXPECT_LE(curvature*cap,forward.max_yaw_rate+1e-10);
    yaw_cap_is_active|=cap+1e-6<author_geometry->brakingLimitAt(
      author_geometry->entry.arcs[i],.5*author.max_acceleration);
  }
  EXPECT_TRUE(yaw_cap_is_active); // This fixture must actually exercise the new constraint.
  for(bool hold_heading:{false,true}) {
    auto low=authorLikeConfig();low.max_yaw_rate=.10;
    auto high=low;high.max_yaw_rate=1.;auto shape=t;
    if(hold_heading) {
      low.holonomic_yaw_policy=high.holonomic_yaw_policy="hold_heading";shape.frozen_yaw=0.;
    }
    const auto slow=PreparedGeometry::build(low,shape,{}),fast=PreparedGeometry::build(high,shape,{});
    ASSERT_TRUE(slow->failure.empty());ASSERT_TRUE(fast->failure.empty());
    // Holonomic translation and held body yaw are independent. The no-sideways
    // strategy must not impose its curvature/yaw coupling on these profiles.
    EXPECT_EQ(slow->speed_envelope,fast->speed_envelope);
  }
}

TEST(ForwardNavigation, FutureTightBendDeceleratesOnStraightApproachWithoutHardZero) {
  const auto c=forwardNavigationConfig();auto t=curve();
  for(auto& p:t.points) {const double x=std::max(0.,p.x()-.5);p.y()=2.*x*x;}
  const auto g=PreparedGeometry::build(c,t,{});ASSERT_TRUE(g->failure.empty());
  TrackerCore core(c);auto body=odom();prepare(core,body,t);
  double arc=0.;Output previous;bool decelerated_before_bend=false,entered_bend=false;
  for(int i=1;i<=160;++i) {
    const double at=PreparedGeometry::metricAt(g->entry.arcs,g->entry.times,arc);
    const auto p=g->entry.curve_sampler->evaluateDeBoorT(at);
    const auto v=g->entry.velocity_sampler->evaluateDeBoorT(at);
    const auto a=g->acceleration_sampler->evaluateDeBoorT(at);
    const double speed=v.head<2>().norm(),curvature=(v.x()*a.y()-v.y()*a.x())/(speed*speed*speed);
    const double now=100.+i*.05,receipt=now-90.;
    body.position=p;body.yaw=std::atan2(v.y(),v.x());
    body.orientation=Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ());
    body.planar_speed=previous.forward;body.velocity_in_frame=previous.forward*v/speed;
    body.angular_velocity_in_frame.z()=curvature*previous.forward;
    body.stamp=body.posterior_stamp=body.imu_stamp=now;
    ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
    const auto out=core.step(now,receipt,true);
    EXPECT_EQ(out.reason,"tracking_forward");EXPECT_FALSE(out.frozen);EXPECT_TRUE(core.active());
    EXPECT_GT(out.forward,0.);EXPECT_DOUBLE_EQ(out.lateral,0.);
    EXPECT_LE(std::abs(out.forward-previous.forward),c.max_acceleration*.05+1e-12);
    EXPECT_LE(std::abs(out.yaw_rate-previous.yaw_rate),c.max_yaw_acceleration*.05+1e-12);
    if(p.x()<.45&&std::abs(curvature)<1e-6&&out.forward<previous.forward-1e-6)
      decelerated_before_bend=true;
    if(p.x()>.70) {entered_bend=true;break;}
    arc+=out.forward*.05;previous=out;
  }
  EXPECT_TRUE(decelerated_before_bend);EXPECT_TRUE(entered_bend);
}

TEST(HolonomicControl, ReplanAndAppliedWriterOutputPreserveAllLimiterAxes) {
  TrackerCore core(config());prepare(core);
  core.recordAppliedOutput(.12,.10,.2,100.,10.);
  auto t=curve(2);ASSERT_TRUE(core.receiveTrajectory(t,100.,10.));
  const auto out=core.step(100.02,10.02);
  EXPECT_LE((planar(out)-Eigen::Vector2d(.12,.10)).norm(),.35*.02+1e-12);
  EXPECT_LE(std::abs(out.yaw_rate-.2),.8*.02+1e-12);EXPECT_GT(out.lateral,.09);
  core.recordAppliedOutput(.1,.2,100.02,10.02); // old ABI explicitly means vy=0
  const auto legacy=core.step(100.04,10.04);EXPECT_EQ(legacy.lateral,0.);
}

TEST(HolonomicControl, AlignDeceleratesTranslationAndPreservesYawHistory) {
  TrackerCore core(config());prepare(core);core.recordAppliedOutput(.1,-.15,.2,100.,10.);
  const auto out=core.align(-1.,.1,100.02,10.02,true);
  EXPECT_EQ(out.reason,"aligning");EXPECT_LE((planar(out)-Eigen::Vector2d(.1,-.15)).norm(),.007+1e-12);
  EXPECT_LE(std::abs(out.yaw_rate-.2),.016+1e-12);EXPECT_GT(planar(out).norm(),0.);
}

TEST(HolonomicControl, OccupiedSidewaysEntryWaitsForNewCurveNotTurnFirstFallback) {
  TrackerCore core(config());prepare(core);
  ASSERT_TRUE(core.notifyBlockedMotion(1,100.,100.,0.,.1,0.));
  EXPECT_STREQ(core.maneuverPhase(),"waiting_executable_entry");
  auto stopped=core.step(100.02,10.02);EXPECT_EQ(planar(stopped).norm(),0.);
  ASSERT_TRUE(core.receiveTrajectory(curve(2),100.02,10.02));
  EXPECT_STREQ(core.maneuverPhase(),"following");EXPECT_GT(core.step(100.04,10.04).forward,0.);
}

TEST(HolonomicControl, OccupiedFeedbackUsesOriginalCommandBudgetButKeepsLatestBodyLease) {
  for(const std::string model:{"forward_yaw","holonomic"}) {
    SCOPED_TRACE(model);auto c=config();c.motion_model=model;
    const double lateral=model=="holonomic"?.1:0.;
    TrackerCore core(c);prepare(core);
    ASSERT_TRUE(core.receiveOdom(odom(100.125),100.125,10.125));
    // Compatibility callers still use 100 ms. The formal record can allow
    // 200 ms end-to-end without re-stamping this exact original command.
    EXPECT_FALSE(core.notifyBlockedMotion(1,100.,100.125,.1,lateral,.2));
    for(double budget:{.099,.501,std::numeric_limits<double>::quiet_NaN()})
      EXPECT_FALSE(core.notifyBlockedMotion(1,100.,100.125,.1,lateral,.2,budget));
    ASSERT_TRUE(core.notifyBlockedMotion(1,100.,100.125,.1,lateral,.2,.2));
    EXPECT_EQ(core.odometry().stamp,100.125);
    EXPECT_STREQ(core.maneuverPhase(),model=="holonomic"?"waiting_executable_entry":"decelerating_for_turn");
    EXPECT_FALSE(core.notifyBlockedMotion(1,100.,100.126,.1,lateral,.2,.2)); // original replay
    TrackerCore expired(c);prepare(expired);
    ASSERT_TRUE(expired.receiveOdom(odom(100.201),100.201,10.201));
    EXPECT_FALSE(expired.notifyBlockedMotion(1,100.,100.201,.1,lateral,.2,.2));
    EXPECT_STREQ(expired.maneuverPhase(),"following");
    TrackerCore old_body(c);prepare(old_body);
    EXPECT_FALSE(old_body.notifyBlockedMotion(1,100.,100.125,.1,lateral,.2,.2));
    EXPECT_STREQ(old_body.maneuverPhase(),"following");
    // Legacy forward-turn entry points propagate the same explicit budget.
    TrackerCore compatibility(c);prepare(compatibility);
    ASSERT_TRUE(compatibility.receiveOdom(odom(100.125),100.125,10.125));
    EXPECT_TRUE(compatibility.notifyBlockedForwardTurn(1,100.,100.125,.1,.2,.2));
  }
}

TEST(HolonomicControl, TimerDoesNotInventMeasuredArcAndCancellationZerosEveryAxis) {
  TrackerCore core(config());prepare(core,odom(100.,.4));
  const auto first=core.step(100.02,10.02);ASSERT_NE(first.lateral,0.);
  const auto arc=core.progressArc();core.step(100.04,10.04);EXPECT_EQ(core.progressArc(),arc);
  core.cancel("cancelled");const auto zero=core.step(100.06,10.06);
  EXPECT_EQ(zero.forward,0.);EXPECT_EQ(zero.lateral,0.);EXPECT_EQ(zero.yaw_rate,0.);
}

TEST(HolonomicControl, FinalGoalBrakesThenRequiresDistinctMeasuredStationarity) {
  auto goal=task();goal.goal={0.,0.,.55};TrackerCore core(config());auto initial=odom();
  initial.velocity_in_frame={.12,.16,0.};initial.planar_speed=.2;initial.angular_velocity_in_frame.z()=.1;
  prepare(core,initial,curve(),goal); // Actual motion, not just an old saved command, requires normal braking.
  core.recordAppliedOutput(.12,.16,.1,100.,10.);
  auto out=core.step(100.02,10.02);EXPECT_FALSE(out.finished);
  EXPECT_LE((planar(out)-Eigen::Vector2d(.12,.16)).norm(),.007+1e-12);
  // Fresh moving measurements cannot finish, even at the exact goal.
  for(int i=2;i<=30;++i) {
    auto body=odom(100.+i*.02);body.planar_speed=.1;
    ASSERT_TRUE(core.receiveOdom(body,body.stamp,10.+i*.02));core.refreshTaskLease(10.+i*.02);
    EXPECT_FALSE(core.step(body.stamp,10.+i*.02).finished);
  }
  for(int i=31;i<=65;++i) {
    auto body=odom(100.+i*.02);ASSERT_TRUE(core.receiveOdom(body,body.stamp,10.+i*.02));
    core.refreshTaskLease(10.+i*.02);out=core.step(body.stamp,10.+i*.02);
    if(i<61){EXPECT_FALSE(out.finished);}
  }
  EXPECT_TRUE(out.finished);EXPECT_EQ(planar(out).norm(),0.);
}

TEST(HolonomicControl, SlopeRetainsConservativeSpatialSpeedAndVectorAccelerationBudget) {
  auto c=config();auto t=curve();for(auto& p:t.points)p.z()+=.1*p.x();
  TrackerCore core(c);prepare(core,odom(100.,1.),t);Output previous;
  for(int i=1;i<=30;++i) {
    const double now=100.+i*.02,receipt=10.+i*.02;auto body=odom(now,1.);
    ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
    const auto out=core.step(now,receipt);
    EXPECT_LE(planar(out).norm()*std::sqrt(1.01),c.max_speed+1e-12);
    EXPECT_LE((planar(out)-planar(previous)).norm()*std::sqrt(1.01),c.max_acceleration*.02+1e-12);
    previous=out;
  }
}

TEST(HolonomicControl, IdealLateralAndReverseClosedLoopsReachPositionWithoutReorientingBody) {
  for(double yaw:{0.,1.5707963267948966,3.141592653589793}) {
    auto c=config();c.holonomic_yaw_policy="hold_heading";c.external_goal_completion=true;
    auto t=curve();t.frozen_yaw=yaw;auto goal=task();goal.goal.x()=.95;
    TrackerCore core(c);auto body=odom(100.,yaw);prepare(core,body,t,goal);
    Output previous;bool reached=false;
    for(int i=1;i<=300;++i) {
      const double now=100.+i*.02,receipt=10.+i*.02;
      body.stamp=body.posterior_stamp=body.imu_stamp=now;
      ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
      const auto out=core.step(now,receipt,true);
      EXPECT_LE(planar(out).norm(),c.max_speed+1e-12);
      expectNormalSlewOrMeasuredGoalZero(out,Eigen::Rotation2Dd(yaw)*planar(previous),body,goal,c,.02);
      EXPECT_NEAR(out.yaw_rate,0.,1e-12);
      const auto world=Eigen::Rotation2Dd(yaw)*planar(out);
      body.position.head<2>()+=world*.02;body.velocity_in_frame={world.x(),world.y(),0.};
      body.planar_speed=world.norm();previous=out;
      if(out.reason=="goal_position_reached"&&planar(out).norm()<1e-9){reached=true;break;}
    }
    EXPECT_TRUE(reached)<<yaw;EXPECT_LE((goal.goal-body.position).norm(),c.goal_tolerance);
  }
}

TEST(HolonomicControl, TwentyHzMeasuredClosedLoopsStayContinuousThroughBendsSideslipAndTurning) {
  // Production TrackerCore drives an ideal holonomic body at its new 20 Hz
  // rate. This tests the controller, not a duplicated control law. Native
  // certificates, task ownership and SDK writer admission remain separately
  // exercised by the typed execution tests.
  for(int scenario=0;scenario<5;++scenario) {
    SCOPED_TRACE(scenario);
    auto c=authorLikeConfig();auto t=curve();auto body=odom();
    if(scenario==1)for(int i=0;i<23;++i) {
      const double theta=(i-1)*.075;
      t.points[i]={.6*std::sin(theta),.6*(1.-std::cos(theta)),.55};
    }
    if(scenario==2) {
      for(auto& point:t.points)std::swap(point.x(),point.y());
      c.holonomic_yaw_policy="hold_heading";t.frozen_yaw=0.;
    }
    if(scenario==3) {
      body.yaw=3.141592653589793;
      c.holonomic_yaw_policy="hold_heading";t.frozen_yaw=body.yaw;
    }
    if(scenario==4)body.yaw=1.2; // Author large-heading XY target limit, then continuous body XY tracking.
    body.orientation=Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ());
    const auto geometry=PreparedGeometry::build(c,t,{});ASSERT_TRUE(geometry->failure.empty());
    ASSERT_TRUE(geometry->derivatives_valid);
    body.position=geometry->entry.curve_sampler->evaluateDeBoorT(0.);
    TrackerCore core(c);prepare(core,body,t);Output previous;bool stopped=false;
    Eigen::Vector2d previous_world=Eigen::Vector2d::Zero();
    double max_cross_track=0.,settled_yaw_error=0.;
    bool had_lateral=false,had_turn=false,had_reverse=false;
    for(int i=1;i<=180;++i) {
      const double now=100.+i*.05,receipt=10.+i*.05;
      if(scenario==1&&i==45) {
        const auto progress=core.progress(now-.05);
        const Eigen::Vector2d velocity=geometry->entry.velocity_sampler->evaluateDeBoorT(progress.curve_time).head<2>();
        body.position.head<2>()+=Eigen::Vector2d(-velocity.y(),velocity.x()).normalized()*.01;
      }
      body.stamp=body.posterior_stamp=body.imu_stamp=now;
      ASSERT_TRUE(core.receiveOdom(body,now,receipt));core.refreshTaskLease(receipt);
      const auto out=core.step(now,receipt,true);
      ASSERT_TRUE(out.reason=="tracking"||out.reason=="heading_alignment"||
        out.reason=="local_segment_finished_waiting_replan")<<out.reason;
      EXPECT_LE(planar(out).norm(),c.max_speed+1e-12);
      const Eigen::Vector2d world=Eigen::Rotation2Dd(body.yaw)*planar(out);
      // Compare vectors in the same planning frame, not two different body
      // bases across the measured yaw update below.
      EXPECT_LE((world-previous_world).norm(),c.max_acceleration*.05+1e-12);
      EXPECT_LE(std::abs(out.yaw_rate-previous.yaw_rate),c.max_yaw_acceleration*.05+1e-12);
      EXPECT_LE(std::abs(out.yaw_rate),c.max_yaw_rate+1e-12);
      had_lateral|=std::abs(out.lateral)>.03;had_turn|=std::abs(out.yaw_rate)>.03;
      had_reverse|=out.forward<-.03;
      const auto progress=core.progress(now);
      const auto projection=geometry->entry.curve_sampler->evaluateDeBoorT(progress.curve_time);
      max_cross_track=std::max(max_cross_track,(projection-body.position).head<2>().norm());
      if(i>(scenario==4?80:40)&&out.reason=="tracking"&&c.holonomic_yaw_policy=="path_tangent") {
        const auto velocity=geometry->entry.velocity_sampler->evaluateDeBoorT(progress.curve_time);
        settled_yaw_error=std::max(settled_yaw_error,std::abs(angle(std::atan2(velocity.y(),velocity.x())-body.yaw)));
      }
      body.position.head<2>()+=world*.05;body.yaw=angle(body.yaw+out.yaw_rate*.05);
      body.orientation=Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ());
      body.velocity_in_frame={world.x(),world.y(),0.};body.planar_speed=world.norm();
      body.angular_velocity_in_frame.z()=out.yaw_rate;previous=out;previous_world=world;
      if(out.reason=="local_segment_finished_waiting_replan"&&planar(out).norm()<1e-9&&std::abs(out.yaw_rate)<1e-9) {
        stopped=true;break;
      }
    }
    EXPECT_TRUE(stopped);EXPECT_LT(max_cross_track,scenario==1?.012:.006);EXPECT_LT(settled_yaw_error,.06);
    EXPECT_LT((geometry->entry.points.back()-body.position).head<2>().norm(),.01);
    if(scenario==2||scenario==4)EXPECT_TRUE(had_lateral);
    if(scenario==3)EXPECT_TRUE(had_reverse);
    if(scenario==1||scenario==4)EXPECT_TRUE(had_turn);
    if(scenario==2||scenario==3)EXPECT_NEAR(body.yaw,*t.frozen_yaw,1e-12);
    RecordProperty("scenario_"+std::to_string(scenario)+"_peak_cross_track_m",std::to_string(max_cross_track));
    RecordProperty("scenario_"+std::to_string(scenario)+"_settled_yaw_error_rad",std::to_string(settled_yaw_error));
  }
}
