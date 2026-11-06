#include <gtest/gtest.h>
#include <plan_manage/execution_validator.hpp>

namespace scan_planner {
// Explicit scheduling-fact seam only. No fake GridMap, collision query,
// command, or ROS context is constructed. Geometry and source pairing below
// run through the production ledger and spline implementations.
struct ExecutionValidatorTestAccess {
  static void seed(ExecutionValidator& ledger,const ew::TaggedBspline& spline,
      const ew::ExecutionVersion& version,const SplineHeadingContract& heading,std::int64_t now) {
    ExecutionValidator::Job job;job.spline=spline;job.version=version;job.heading=heading;
    job.support=ew::SupportReference{};job.support->verified=true;job.support->version=version;
    job.proof=ew::TrajectoryValidation{};job.proof->valid=true;job.proof->valid_until=rclcpp::Time(now+250000000LL);
    ledger.active_=job;ledger.task_domain_=version;ledger.task_closed_=false;
    ledger.latest_permit_=ew::ExecutionPermit{};ledger.latest_permit_->version=version;
  }
};
}
namespace {
namespace s=scan_planner;
namespace w=d1max_planning_interfaces::msg;
constexpr std::int64_t now_ns=100000000000LL;
constexpr double now=100.;
Eigen::MatrixXd reversePrefix(double lateral) {
  Eigen::MatrixXd p(3,23);const double x[]={.03,0.,-.03,-.03,-.015,.015,.06};
  for(int i=0;i<23;++i)p.col(i)=Eigen::Vector3d(i<7?x[i]:.06+.06*(i-6),lateral*.3*(i-1),.55);
  return p;
}
s::SplineHeadingContract heading() {
  s::SplineHeadingContract h;h.measured_heading_enabled=true;h.align_then_forward=true;return h;
}
std::uint32_t intervals(s::UniformBspline& curve) {
  const auto v=curve.getDerivative().getControlPoint();
  return *s::bodyHeadingSamplingIntervals(curve.getTimeSum(),v.colwise().norm().maxCoeff(),.05);
}
s::SplineHeadingContract resolved(s::UniformBspline& curve,s::SplineHeadingContract h) {
  auto v=curve.getDerivative();
  EXPECT_TRUE(s::resolveForwardEntryHeading(h,curve.getTimeSum(),intervals(curve),
    [&](double at){return curve.evaluateDeBoorT(at);},[&](double at){return v.evaluateDeBoorT(at);},[]{return true;}));
  return h;
}
struct SchedulingFixture {
  s::CollisionSnapshotPool pool;
  s::ExecutionValidator native{s::ExecutionValidator::PassiveLedger{},pool,"odom","isolated_mock",false,false,true};
  s::UniformBspline curve;w::ExecutionVersion version;w::TaggedBspline tag;
  s::MeasuredBodyPose body;Eigen::Vector3d actual_velocity;
  SchedulingFixture(const Eigen::MatrixXd& points,double measured):curve(points,3,.3) {
    version.schema_version=3;version.session_id="session";version.task_id="task";version.route_id="route";
    version.route_hash=std::string(64,'a');version.map_version_id="map";version.localization_epoch=1;
    version.localization_seed_id="seed";version.reference_generation=version.anchor_revision=version.context_sequence=1;
    version.anchor_id="anchor";version.segment_id="floor";
    tag.trajectory.order=3;tag.trajectory.traj_id=7;tag.trajectory.start_time=rclcpp::Time(now_ns);
    for(int i=0;i<points.cols();++i){geometry_msgs::msg::Point p;p.x=points(0,i);p.y=points(1,i);p.z=points(2,i);tag.trajectory.pos_pts.push_back(p);}
    const auto knots=curve.getKnot();for(int i=0;i<knots.size();++i)tag.trajectory.knots.push_back(knots[i]);
    tag.body_heading_seed_valid=true;tag.body_heading_seed_yaw=0.;tag.body_heading_sampling_intervals=intervals(curve);
    s::ExecutionValidatorTestAccess::seed(native,tag,version,heading(),now_ns);
    body={curve.evaluateDeBoorT(measured),Eigen::Quaterniond::Identity(),now,"odom"};
    actual_velocity=curve.getDerivative().evaluateDeBoorT(measured);
    geometry_msgs::msg::Twist twist;twist.linear.x=actual_velocity.x();twist.linear.y=actual_velocity.y();twist.linear.z=actual_velocity.z();
    native.body(body,twist,now_ns);
    w::TrackingProgress p;p.schema_version=2;p.valid=true;p.header.stamp=rclcpp::Time(now_ns);p.header.frame_id="odom";
    p.session_id=version.session_id;p.task_id=version.task_id;p.route_id=version.route_id;p.route_hash=version.route_hash;
    p.map_version_id=version.map_version_id;p.localization_epoch=version.localization_epoch;p.localization_seed_id=version.localization_seed_id;
    p.generation=version.reference_generation;p.anchor_id=version.anchor_id;p.anchor_revision=version.anchor_revision;
    p.context_sequence=version.context_sequence;p.segment_id=version.segment_id;p.trajectory_id=7;p.curve_time=measured;
    p.arc_length=p.s_committed=s::curveArcTo(curve,measured);p.pose.orientation.w=1.;
    p.pose.position.x=body.position.x();p.pose.position.y=body.position.y();p.pose.position.z=body.position.z();
    native.progressAt(p,now);
  }
};
}

TEST(SpliceHeadingContinuity,ExactPositiveVelocityTailCannotDiscardActiveReversePrefix) {
  for(const double lateral:{-.03,.03}) {
    SCOPED_TRACE(lateral);const auto controls=reversePrefix(lateral);
    SchedulingFixture f(controls,.1);auto velocity=f.curve.getDerivative(),acceleration=velocity.getDerivative();
    auto old=resolved(f.curve,heading());ASSERT_TRUE(old.forward_entry_profile);
    constexpr double at=.6;ASSERT_GT(old.forward_entry_end,at);
    s::UniformBspline tail(controls.rightCols(controls.cols()-2),3,.3);
    auto tv=tail.getDerivative(),ta=tv.getDerivative();
    EXPECT_LT((tail.evaluateDeBoorT(0.)-f.curve.evaluateDeBoorT(at)).norm(),1e-12);
    EXPECT_LT((tv.evaluateDeBoorT(0.)-velocity.evaluateDeBoorT(at)).norm(),1e-12);
    EXPECT_LT((ta.evaluateDeBoorT(0.)-acceleration.evaluateDeBoorT(at)).norm(),1e-12);
    const auto old_geometry=s::forwardEntryHeadingGeometryAt(at,velocity.evaluateDeBoorT(at),acceleration.evaluateDeBoorT(at),old);
    ASSERT_TRUE(old_geometry);auto tail_heading=heading();tail_heading.measured_yaw=old_geometry->yaw;
    tail_heading=resolved(tail,tail_heading);ASSERT_FALSE(tail_heading.forward_entry_profile);
    EXPECT_GT(tv.evaluateDeBoorT(0.).x(),0.); // This is not a newly reverse-going spline.
    const double new_yaw=s::bodyHeadingAtCurveTime(tv.evaluateDeBoorT(0.),old_geometry->yaw,tail_heading,0.);
    EXPECT_GT(std::abs(std::atan2(std::sin(new_yaw-old_geometry->yaw),std::cos(new_yaw-old_geometry->yaw))),.8);
    EXPECT_TRUE(s::frozenHeadingWireMatches(f.tag.trajectory,heading()));
    EXPECT_EQ(f.tag.body_heading_seed_yaw,0.);EXPECT_EQ(f.tag.body_heading_sampling_intervals,intervals(f.curve));
    // Refuse only this future boundary. The live curve is retained and the
    // FSM's existing no-splice branch uses the original acquired body P/V.
    EXPECT_FALSE(f.native.plannedSpliceAt(f.version,now_ns,.05,.8));
    EXPECT_FALSE(f.native.plannedSpliceAtBody(f.version,now_ns,.05,f.body,now_ns,.8));
    EXPECT_EQ(f.native.committedTrajectory(),7);
    const auto actual=f.native.committedBoundary(f.body.position,f.actual_velocity,now_ns,now);
    ASSERT_TRUE(actual);EXPECT_LT((actual->velocity-f.actual_velocity).norm(),1e-12);
    EXPECT_LT(actual->velocity.x(),0.); // Not the future seam's positive derivative.
  }
}

TEST(SpliceHeadingContinuity,PrefixEndRestoresNormalFutureSeamsWithoutWaitingOrRetiring) {
  for(const double lateral:{-.03,.03}) {
    SchedulingFixture f(reversePrefix(lateral),2.4);
    const auto h=resolved(f.curve,heading());ASSERT_TRUE(h.forward_entry_profile);ASSERT_LT(h.forward_entry_end,3.2);
    const auto splice=f.native.plannedSpliceAt(f.version,now_ns,.05,.8);ASSERT_TRUE(splice);
    EXPECT_DOUBLE_EQ(splice->wire.splice_curve_time,3.2);EXPECT_GT(splice->wire.velocity.x,0.);
    EXPECT_EQ(splice->wire.incumbent_trajectory_id,7);EXPECT_EQ(splice->progress_source_ns,now_ns);
    EXPECT_TRUE(f.native.plannedSpliceAtBody(f.version,now_ns,.05,f.body,now_ns,.8));
  }
}

TEST(SpliceHeadingContinuity,OrdinaryBendAndGenuineBackwardGeometryKeepExistingPolicy) {
  for(const bool backward:{false,true}) {
    Eigen::MatrixXd p(3,23);
    for(int i=0;i<23;++i){const double u=(i-1)*.05;
      p.col(i)=backward?Eigen::Vector3d(-u,0.,.55):Eigen::Vector3d(std::sin(u),1.-std::cos(u),.55);}
    SchedulingFixture f(p,.1);const auto h=resolved(f.curve,heading());EXPECT_FALSE(h.forward_entry_profile);
    const auto splice=f.native.plannedSpliceAt(f.version,now_ns,.05,.8);ASSERT_TRUE(splice);
    if(backward)EXPECT_NEAR(std::abs(splice->entry_yaw),M_PI,1e-12);
    else {EXPECT_GT(splice->entry_yaw,0.);EXPECT_LT(splice->entry_yaw,M_PI*.5);}
  }
}

TEST(SpliceHeadingContinuity,ForeignTaskAndCancelNeverReuseHistoricalPrefixOrSeam) {
  SchedulingFixture f(reversePrefix(.03),2.4);ASSERT_TRUE(f.native.plannedSpliceAt(f.version,now_ns,.05,.8));
  auto foreign=f.version;foreign.task_id="other";EXPECT_FALSE(f.native.plannedSpliceAt(foreign,now_ns,.05,.8));
  auto barrier=f.version;++barrier.reference_generation;ASSERT_TRUE(f.native.cancelReference(barrier));
  EXPECT_EQ(f.native.committedTrajectory(),-1);
  EXPECT_FALSE(f.native.plannedSpliceAt(f.version,now_ns,.05,.8));
  EXPECT_FALSE(f.native.plannedSpliceAtBody(f.version,now_ns,.05,f.body,now_ns,.8));
}
