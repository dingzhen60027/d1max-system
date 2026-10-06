#include <gtest/gtest.h>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <condition_variable>
#include "d1max_trajectory_tracker/tracker_core.hpp"
#include "d1max_trajectory_tracker/execution_contract.hpp"
using namespace d1max_trajectory_tracker;

namespace {
ControlIdentity identity() {
  ControlIdentity i;
  i.schema_version=2; i.task_id="task"; i.route_id="route"; i.route_hash=std::string(64,'a');
  i.segment_id="floor1"; i.map_version_id="map"; i.anchor_id="anchor";
  i.anchor_revision=i.context_sequence=i.localization_epoch=i.map_geometry_revision=1; i.localization_seed_id="seed";
  return i;
}
Config config() { Config c; c.session_id="session"; c.map_version_id="map";
  c.planning_frame="d1max_loc_odom"; return c; }
Task task() {
  Task t; t.session_id="session"; t.frame_id="d1max_loc_odom"; t.generation=1;
  t.active=true; t.issued_at=100.; t.goal={2.,0.,.55}; t.identity=identity(); return t;
}
Odom odom(double stamp=100., double x=0., double z=.55) {
  Odom o; o.frame_id="d1max_loc_odom"; o.child_frame_id="d1max_loc_base_link";
  o.stamp=stamp; o.position={x,0.,z}; o.localization_epoch=1; o.localization_seed_id="seed";
  o.schema_version=2; o.session_id="session"; o.map_version_id="map";
  o.posterior_stamp=o.imu_stamp=stamp; o.planar_speed=.25; o.velocity_in_frame={.25,0.,0.}; return o;
}
void certify(Trajectory& t,double source=-1.,double parameter=0.) {
  Eigen::MatrixXd points(3,t.points.size());
  for(std::size_t i=0;i<t.points.size();++i)points.col(i)=t.points[i];
  Eigen::VectorXd knots(t.knots.size());
  for(std::size_t i=0;i<t.knots.size();++i)knots[i]=t.knots[i];
  scan_planner::UniformBspline spline(points,t.order,.1); spline.setKnot(knots);
  auto velocity=spline.getDerivative();
  t.valid_start_time=parameter; t.valid_start_arc_length=0.;
  t.join_source_stamp=source<0.?t.start_time:source;
  t.join_position=spline.evaluateDeBoorT(parameter); t.join_velocity=velocity.evaluateDeBoorT(parameter);
  Eigen::Vector3d previous=spline.evaluateDeBoorT(0.);
  const int intervals=std::max(1,static_cast<int>(std::ceil(parameter/.001)));
  for(int i=1;i<=intervals;++i) {
    const Eigen::Vector3d p=spline.evaluateDeBoorT(parameter*i/intervals);
    t.valid_start_arc_length+=(p-previous).norm(); previous=p;
  }
}
Trajectory curve(std::int64_t id=1, double stamp=100.) {
  Trajectory t; t.session_id="session"; t.frame_id="d1max_loc_odom"; t.generation=1;
  t.point_reference="body_center";
  t.id=id; t.start_time=stamp; t.identity=identity();
  for(int i=0;i<23;++i)t.points.emplace_back((i-1)*.05,0.,.55);
  for(int i=0;i<27;++i)t.knots.push_back((i-3)*.2);
  certify(t);
  return t;
}
void prepare(TrackerCore& core) {
  ASSERT_TRUE(core.receiveTask(task(),100.,10.));
  ASSERT_TRUE(core.receiveOdom(odom(),100.,10.));
  ASSERT_TRUE(core.receiveTrajectory(curve(),100.,10.));
}
}

TEST(TrackerV2, LegacyInputsCannotManufactureControlIdentity) {
  TrackerCore core(config()); auto t=task(); t.identity={};
  EXPECT_FALSE(core.receiveTask(t,100.,10.));
  EXPECT_FALSE(core.active());
}
TEST(TrackerV2, MissingOrDifferentMapGeometryRevisionCannotEnterTheAdmittedCurve) {
  for(std::uint64_t revision:{0u,2u}) {
    TrackerCore core(config());prepare(core);auto t=curve(2);t.identity.map_geometry_revision=revision;
    EXPECT_FALSE(core.receiveTrajectory(t,100.,10.));EXPECT_EQ(core.candidateReason(),"identity_mismatch");
    EXPECT_EQ(core.trajectoryId(),1);EXPECT_TRUE(core.hasInstalledGeometry());
    auto wrong_task=task();wrong_task.identity.map_geometry_revision=revision;
    EXPECT_FALSE(core.admitRevision(wrong_task,curve(2),SupportEvidence{},100.,10.,true));
    EXPECT_EQ(core.trajectoryId(),1);
  }
  auto missing=task();missing.identity.map_geometry_revision=0;TrackerCore empty(config());
  EXPECT_FALSE(empty.receiveTask(missing,100.,10.));EXPECT_FALSE(empty.active());
}

TEST(TrackerV2, PredictedStateHeadersDoNotManufactureIndependentTurnRestSamples) {
  TrackerCore core(config());prepare(core);
  ASSERT_TRUE(core.notifyBlockedForwardTurn(1,100.,100.,.1,.2));
  for(int i=1;i<=35;++i) {
    const double t=i*.02;auto state=odom(100+t);
    state.planar_speed=0.;state.velocity_in_frame.setZero();
    const double measured=100.+(i/5)*.1;
    state.posterior_stamp=state.imu_stamp=measured;
    state.extrapolation_sec=std::max(0.,state.stamp-measured);
    ASSERT_TRUE(core.receiveOdom(state,100+t,10+t));core.refreshTaskLease(10+t);
    EXPECT_EQ(core.step(100+t,10+t).forward,0.);
    if(i<35) {EXPECT_STREQ(core.maneuverPhase(),"decelerating_for_turn")<<i;}
  }
  EXPECT_STREQ(core.maneuverPhase(),"aligning_to_curve");
}
TEST(TrackerJoinDiagnostic, EveryEvidencePredicateReportsItsOriginalReasonWithoutChangingVerdict) {
  const auto c=config();const auto good=curve();const auto body=odom();
  EXPECT_EQ(joinEvidenceFailure(good,true,body,100.,4.,c),nullptr);
  EXPECT_STREQ(joinEvidenceFailure(good,false,body,100.,4.,c),"join_body_missing");
  auto stale=body;stale.stamp=99.;
  EXPECT_STREQ(joinEvidenceFailure(good,true,stale,100.,4.,c),"join_body_not_fresh");
  const auto rejected=[&](const std::function<void(Trajectory&)>& alter,const char* why,double now=100.){
    auto t=good;alter(t);EXPECT_STREQ(joinEvidenceFailure(t,true,body,now,4.,c),why);
  };
  const auto nan=std::numeric_limits<double>::quiet_NaN();
  rejected([&](auto& t){t.valid_start_time=nan;},"join_curve_time_nonfinite");
  rejected([](auto& t){t.valid_start_time=-.01;},"join_curve_time_out_of_range");
  rejected([](auto& t){t.valid_start_time=4.01;},"join_curve_time_out_of_range");
  rejected([&](auto& t){t.valid_start_arc_length=nan;},"join_arc_nonfinite");
  rejected([](auto& t){t.valid_start_arc_length=-.01;},"join_arc_negative");
  rejected([&](auto& t){t.join_source_stamp=nan;},"join_source_nonfinite");
  rejected([](auto& t){t.join_source_stamp=0.;},"join_source_missing");
  rejected([](auto& t){t.join_source_stamp=100.001;},"join_source_in_future");
  rejected([](auto& t){t.join_source_stamp=99.;},"join_source_expired");
  rejected([](auto& t){t.join_source_stamp=100.000002;},"join_source_after_body",100.01);
  rejected([&](auto& t){t.join_position.x()=nan;},"join_position_nonfinite");
  rejected([&](auto& t){t.join_velocity.x()=nan;},"join_velocity_nonfinite");
  rejected([&](auto& t){t.join_orientation.x()=nan;},"join_orientation_nonfinite");
  rejected([](auto& t){t.join_orientation.w()=2.;},"join_orientation_not_unit");
  const auto source=[&](const std::function<void(Odom&)>& alter,const char* why,double now=100.){
    auto o=body;alter(o);EXPECT_STREQ(sourceEvidenceFailure(o,now,c),why);
  };
  source([](auto& o){o.posterior_stamp=0.;},"body_posterior_source_missing");
  source([](auto& o){o.imu_stamp=0.;},"body_imu_source_missing");
  source([&](auto& o){o.extrapolation_sec=nan;},"body_extrapolation_nonfinite");
  source([](auto& o){o.extrapolation_sec=-.01;},"body_extrapolation_negative");
  source([&](auto& o){o.extrapolation_sec=c.max_extrapolation+.01;},"body_extrapolation_limit");
  source([](auto& o){o.posterior_stamp=100.01;},"body_posterior_after_state");
  source([](auto& o){o.imu_stamp=100.01;},"body_imu_after_state");
  source([](auto& o){o.imu_stamp=99.99;},"body_extrapolation_source_mismatch");
  source([](auto& o){o.posterior_stamp=99.;},"body_posterior_not_fresh");
  source([&](auto& o){o.stamp=o.posterior_stamp=100.+c.imu_timeout+.01;},"body_extrapolation_source_mismatch");
  source([](auto&){},"body_imu_not_fresh",100.+c.imu_timeout+.01);
}
TEST(TrackerJoinDiagnostic, FailedTrialPreservesOriginalFineNanosecondEvidenceAndSafeIncumbent) {
  TrackerCore core(config());prepare(core);
  auto body=odom(100.1);body.source_stamp_ns=100100000019LL;
  ASSERT_TRUE(core.receiveOdom(body,100.1,10.1));
  auto t=curve(2);t.join_source_stamp=99.6;t.join_source_stamp_ns=99600000017LL;
  EXPECT_FALSE(core.admitRevision(task(),t,SupportEvidence{},100.1,10.1,true));
  EXPECT_EQ(core.candidateReason(),"join_source_expired");
  const auto& d=core.joinDiagnostic();EXPECT_EQ(d.join_source_stamp_ns,99600000017LL);
  EXPECT_EQ(d.body_source_stamp_ns,100100000019LL);EXPECT_DOUBLE_EQ(d.curve_duration,4.);
  EXPECT_TRUE(core.hasInstalledGeometry());EXPECT_EQ(core.trajectoryId(),1);
}
TEST(TrackerV2, WrongEpochOdometryInvalidatesWholeTask) {
  TrackerCore core(config()); prepare(core);
  auto o=odom(100.1); o.localization_epoch=2;
  EXPECT_FALSE(core.receiveOdom(o,100.1,10.1)); EXPECT_FALSE(core.active());
  EXPECT_FALSE(core.progress(100.1).valid);
}

TEST(TrackerV2, RetiredExpiredAndDuplicateObservationsCannotStopNewTaskOrRenewItsBody) {
  TrackerCore core(config());prepare(core);
  ASSERT_TRUE(core.receiveOdom(odom(100.08,.02),100.08,10.08));
  const auto before=core.progress(100.08);const auto before_stamp=core.odometry().stamp;
  auto old=odom(99.);old.imu_stamp=98.;old.localization_seed_id="old-seed";
  EXPECT_FALSE(core.receiveOdom(old,100.1,10.1));
  auto duplicate=odom(100.08);duplicate.localization_seed_id="retired-seed";
  EXPECT_FALSE(core.receiveOdom(duplicate,100.1,10.1));
  EXPECT_TRUE(core.active());EXPECT_FALSE(core.holding());
  EXPECT_DOUBLE_EQ(core.odometry().stamp,before_stamp);
  EXPECT_EQ(core.trajectoryId(),1);EXPECT_EQ(core.progress(100.1).identity.task_id,before.identity.task_id);
  // A genuinely newer identity remains a stop, never a soft smoothing event.
  auto reset=odom(100.12);reset.localization_epoch=2;
  EXPECT_FALSE(core.receiveOdom(reset,100.12,10.12));EXPECT_FALSE(core.active());
}

TEST(TrackerV2, RetiredEpochWithNewerTimestampCannotCancelRelocalizedTask) {
  TrackerCore core(config());auto t=task();t.identity.localization_epoch=2;t.identity.localization_seed_id="seed2";
  ASSERT_TRUE(core.receiveTask(t,100.,10.));auto current=odom();current.localization_epoch=2;current.localization_seed_id="seed2";
  ASSERT_TRUE(core.receiveOdom(current,100.,10.));auto spline=curve();spline.identity=t.identity;
  ASSERT_TRUE(core.receiveTrajectory(spline,100.,10.));
  EXPECT_FALSE(core.receiveOdom(odom(100.1),100.1,10.1));
  EXPECT_TRUE(core.active());EXPECT_FALSE(core.holding());EXPECT_EQ(core.trajectoryId(),1);
  EXPECT_EQ(core.odometry().localization_epoch,2u);
}

TEST(TrackerV2, FreshEvidenceFaultCannotBeClearedByOlderValidPacket) {
  TrackerCore core(config());prepare(core);auto fault=odom(100.08);fault.imu_stamp=99.;
  EXPECT_FALSE(core.receiveOdom(fault,100.08,10.08));EXPECT_TRUE(core.holding());
  EXPECT_FALSE(core.receiveOdom(odom(100.06),100.09,10.09));
  EXPECT_TRUE(core.holding());EXPECT_DOUBLE_EQ(core.odometry().stamp,100.);
  EXPECT_EQ(core.trajectoryId(),1);EXPECT_TRUE(core.active());
}
TEST(TrackerV2, NewEpochWithResetClockRevokesInsteadOfHidingBehindPriorSourceFloor) {
  TrackerCore core(config());prepare(core);auto reset=odom(99.);
  reset.localization_epoch=2;reset.localization_seed_id="reset";
  EXPECT_FALSE(core.receiveOdom(reset,100.1,10.1));
  EXPECT_FALSE(core.active());EXPECT_EQ(core.reason(),"odometry_context_changed");
  auto next=task();next.generation=2;next.issued_at=100.2;next.identity.localization_epoch=2;
  next.identity.localization_seed_id="reset";ASSERT_TRUE(core.receiveTask(next,100.2,10.2));
  auto body=odom(100.2);body.localization_epoch=2;body.localization_seed_id="reset";
  ASSERT_TRUE(core.receiveOdom(body,100.2,10.2));
  EXPECT_FALSE(core.receiveOdom(odom(100.3),100.3,10.3));EXPECT_TRUE(core.active());
  EXPECT_EQ(core.odometry().localization_epoch,2u);
}
TEST(TrackerV2, ForeignAndFutureFaultsCannotPoisonNextExpectedSource) {
  TrackerCore core(config());prepare(core);auto future=odom(5000.);
  EXPECT_FALSE(core.receiveOdom(future,100.1,10.1));EXPECT_TRUE(core.holding());
  ASSERT_TRUE(core.receiveOdom(odom(100.12),100.12,10.12));
  auto foreign=odom(100.17);foreign.session_id="other";
  EXPECT_FALSE(core.receiveOdom(foreign,100.17,10.17));EXPECT_TRUE(core.active());
  EXPECT_TRUE(core.receiveOdom(odom(100.15),100.17,10.18)); // foreign stamp is not our source floor
}
TEST(TrackerV2, NewerForeignSessionMapOrSchemaDoesNotStopFreshTrajectoryOrEraseMeasuredProgress) {
  for(int variant=0;variant<3;++variant) {
    TrackerCore core(config());prepare(core);
    const auto before=core.step(100.02,10.02);ASSERT_GT(before.forward,0.);
    auto foreign=odom(100.04);foreign.localization_epoch=99;foreign.localization_seed_id="other-seed";
    if(variant==0)foreign.session_id="other";
    if(variant==1)foreign.map_version_id="other";
    if(variant==2)foreign.schema_version=99;
    EXPECT_FALSE(core.receiveOdom(foreign,100.04,10.04));
    EXPECT_TRUE(core.active());EXPECT_FALSE(core.holding());EXPECT_EQ(core.trajectoryId(),1);
    EXPECT_DOUBLE_EQ(core.odometry().stamp,100.);
    const auto after=core.step(100.04,10.04);EXPECT_GT(after.forward,before.forward);
    auto genuine=odom(100.03);ASSERT_TRUE(core.receiveOdom(genuine,100.04,10.04));
    auto malformed=odom(100.05);malformed.localization_seed_id.clear();
    EXPECT_FALSE(core.receiveOdom(malformed,100.05,10.05));EXPECT_TRUE(core.holding());
    EXPECT_TRUE(core.active());EXPECT_FALSE(core.receiveOdom(odom(100.04),100.05,10.05));
  }
}
TEST(TrackerV2, MatchingXYZProgressUsesRealSourceTimeNotControlTicks) {
  TrackerCore core(config()); prepare(core);
  core.step(100.05,10.05);
  ASSERT_TRUE(core.receiveOdom(odom(100.1,.2),100.1,10.1)); core.step(100.1,10.1);
  const auto measured=core.progress(100.1);
  ASSERT_TRUE(measured.valid); EXPECT_NEAR(measured.arc_length,.2,.003);
  EXPECT_EQ(measured.source_stamp,100.1);
  for(int i=1;i<=3;++i)core.step(100.1+i*.05,10.1+i*.05);
  EXPECT_DOUBLE_EQ(core.progressTime(),measured.curve_time);
  EXPECT_DOUBLE_EQ(core.progress(100.25).source_stamp,100.1);
}
TEST(TrackerV2, SmallPhysicalBacktrackPreservesOnlyCommittedHighWater) {
  TrackerCore core(config()); prepare(core); core.step(100.05,10.05);
  ASSERT_TRUE(core.receiveOdom(odom(100.1,.2),100.1,10.1)); core.step(100.1,10.1);
  const double high=core.committedArc();
  ASSERT_TRUE(core.receiveOdom(odom(100.15,.1),100.15,10.15)); core.step(100.15,10.15);
  EXPECT_NEAR(core.progressArc(),.1,.003); EXPECT_DOUBLE_EQ(core.committedArc(),high);
  ASSERT_TRUE(core.receiveOdom(odom(100.2,-.2),100.2,10.2));
  EXPECT_EQ(core.step(100.2,10.2).reason,"measured_projection_outside_window");
}
TEST(TrackerV2, VerticalMeasurementIsNotLostByXYProjection) {
  TrackerCore core(config()); ASSERT_TRUE(core.receiveTask(task(),100.,10.));
  auto body=odom(); body.planar_speed=0.; body.velocity_in_frame={0.,0.,.035};
  ASSERT_TRUE(core.receiveOdom(body,100.,10.)); auto t=curve();
  for(std::size_t i=0;i<t.points.size();++i)t.points[i]={0.,0.,.55+(static_cast<double>(i)-1)*.007};
  certify(t);
  ASSERT_TRUE(core.receiveTrajectory(t,100.,10.)); core.step(100.05,10.05);
  ASSERT_TRUE(core.receiveOdom(odom(100.1,0.,.65),100.1,10.1)); core.step(100.1,10.1);
  EXPECT_NEAR(core.progressArc(),.1,.003); EXPECT_GT(core.progressTime(),2.);
}
TEST(TrackerV2, DifferentSegmentOrAnchorNeverReplacesCurve) {
  for(int variant=0;variant<5;++variant) {
    TrackerCore core(config()); prepare(core); auto replacement=curve(2,100.1);
    if(variant==0)replacement.identity.segment_id="upstairs";
    if(variant==1)replacement.identity.anchor_revision=2;
    if(variant==2)replacement.identity.route_hash=std::string(64,'b');
    if(variant==3)replacement.point_reference="ground";
    if(variant==4)replacement.point_reference.clear();
    EXPECT_FALSE(core.receiveTrajectory(replacement,100.1,10.1));
    EXPECT_EQ(core.trajectoryId(),1);
    EXPECT_GT(core.step(100.1,10.1).forward,0.);
  }
}
TEST(TrackerV2, HigherIdCannotHideOlderSolveOrDistantJoin) {
  TrackerCore core(config()); prepare(core);
  ASSERT_TRUE(core.receiveOdom(odom(100.1),100.1,10.1));
  ASSERT_TRUE(core.receiveTrajectory(curve(2,100.1),100.1,10.1));
  auto stale=curve(3,100.05);
  EXPECT_FALSE(core.receiveTrajectory(stale,100.15,10.15)); EXPECT_EQ(core.trajectoryId(),2);
  auto far=curve(4,100.15); for(auto& p:far.points)p.x()+=.10;
  EXPECT_FALSE(core.receiveTrajectory(far,100.15,10.15)); EXPECT_EQ(core.trajectoryId(),2);
}
TEST(TrackerV2, ValidReplacementPreservesLimiterAndDoesNotInventGlobalProgress) {
  TrackerCore core(config()); prepare(core); Output before;
  for(int i=1;i<=8;++i) {
    const double t=i*.05;
    ASSERT_TRUE(core.receiveTask(task(),100+t,10+t));
    ASSERT_TRUE(core.receiveOdom(odom(100+t,.2),100+t,10+t)); before=core.step(100+t,10+t);
  }
  auto next=curve(2,100.4); for(auto& p:next.points)p.x()+=.2;
  certify(next);
  ASSERT_TRUE(core.receiveTrajectory(next,100.4,10.4));
  EXPECT_DOUBLE_EQ(core.committedArc(),0.);  // local new curve, not route progress
  ASSERT_TRUE(core.receiveOdom(odom(100.45,.2),100.45,10.45));
  const auto after=core.step(100.45,10.45);
  EXPECT_LE(std::abs(after.forward-before.forward),.4*.05+1e-8);
  EXPECT_GT(after.forward,.1);
}

TEST(TrackerV2, CancelNeedsSameIdentityNotMerelyMatchingGeneration) {
  TrackerCore core(config()); prepare(core); auto cancel=task(); cancel.active=false;
  cancel.identity.route_id="other";
  EXPECT_FALSE(core.receiveTask(cancel,100.05,10.05)); EXPECT_TRUE(core.active());
  cancel.identity=identity(); ASSERT_TRUE(core.receiveTask(cancel,100.05,10.05));
  EXPECT_FALSE(core.active()); EXPECT_FALSE(core.progress(100.05).valid);
}

TEST(TrackerV2, FreshPublishStampCannotLeaseOldOrFutureMeasurementEvidence) {
  for (int variant=0;variant<5;++variant) {
    TrackerCore core(config()); prepare(core); auto o=odom(100.1);
    if(variant==0) { o.imu_stamp=99.9; o.extrapolation_sec=.2; }
    if(variant==1) o.posterior_stamp=99.5;
    if(variant==2) o.imu_stamp=100.2;
    if(variant==3) o.map_version_id="old-map";
    if(variant==4) o.session_id="old-session";
    EXPECT_FALSE(core.receiveOdom(o,100.1,10.1));
    if(variant<3) {EXPECT_TRUE(core.active());EXPECT_TRUE(core.holding());EXPECT_EQ(core.trajectoryId(),1);}
    else {EXPECT_TRUE(core.active());EXPECT_FALSE(core.holding());EXPECT_EQ(core.trajectoryId(),1);}
  }
}

TEST(TrackerV2, FreshBodyStateCannotHideReorderedInputs) {
  TrackerCore core(config()); prepare(core); auto o=odom(100.05);
  o.posterior_stamp=99.99;
  EXPECT_FALSE(core.receiveOdom(o,100.05,10.05));
  EXPECT_EQ(core.reason(),"odometry_evidence_reordered");
}

TEST(TrackerV2, SourceEvidenceExpiresWithoutAnyNewCallback) {
  TrackerCore core(config()); prepare(core); core.step(100.05,10.05);
  ASSERT_TRUE(core.progress(100.05).valid);
  EXPECT_FALSE(core.progress(100.15).valid);
  EXPECT_EQ(core.step(100.15,10.15).reason,"odometry_stale");
}

TEST(TrackerV2, MovingBodyAdmitsMeasuredNonzeroCurveParameterWithoutRestarting) {
  TrackerCore core(config()); prepare(core); Output before;
  for(int i=1;i<=8;++i) {
    const double dt=i*.05;
    ASSERT_TRUE(core.receiveTask(task(),100.+dt,10.+dt));
    ASSERT_TRUE(core.receiveOdom(odom(100.+dt,.25*dt),100.+dt,10.+dt));
    before=core.step(100.+dt,10.+dt);
  }
  auto candidate=curve(2,100.2); certify(candidate,100.4,.4);
  // A newer physical sample arrives after native validation and before delivery.
  ASSERT_TRUE(core.receiveOdom(odom(100.45,.1125),100.45,10.45));
  ASSERT_TRUE(core.receiveTrajectory(candidate,100.45,10.45));
  EXPECT_NEAR(core.progressTime(),.45,1e-6);
  EXPECT_NEAR(core.progressArc(),.1125,1e-6);
  EXPECT_NEAR(core.committedArc(),.1125,1e-6);
  const auto output=core.step(100.45,10.45);
  EXPECT_LE(std::abs(output.forward-before.forward),.4*.05+1e-8);
  EXPECT_GT(output.forward,.1);
  EXPECT_EQ(core.trajectoryId(),2);
}

TEST(TrackerV2, DeliveryTimeDoesNotAdvanceAStationaryJoin) {
  TrackerCore core(config()); prepare(core);
  ASSERT_TRUE(core.receiveOdom(odom(100.1,.1),100.1,10.1));
  auto candidate=curve(2,100.); certify(candidate,100.1,.4);
  ASSERT_TRUE(core.receiveTrajectory(candidate,100.15,10.15));
  EXPECT_NEAR(core.progressTime(),.4,1e-6); // not receipt-start (.15)
  core.step(100.15,10.15);
  EXPECT_NEAR(core.progressTime(),.4,1e-6);
}

TEST(TrackerV2, InvalidHandoffProofNeverReplacesIncumbent) {
  for(int variant=0;variant<8;++variant) {
    TrackerCore core(config()); prepare(core);
    ASSERT_TRUE(core.receiveOdom(odom(100.1,.025),100.1,10.1));
    auto candidate=curve(2,100.05); certify(candidate,100.1,.1);
    if(variant==0)candidate.join_source_stamp=99.5;
    if(variant==1)candidate.join_source_stamp=100.2;
    if(variant==2)candidate.valid_start_time=10.;
    if(variant==3)candidate.valid_start_arc_length+=.1;
    if(variant==4)candidate.join_position.z()+=.03;
    if(variant==5)candidate.join_velocity.x()+=.06;
    if(variant==6) { candidate.join_acceleration_valid=true; candidate.join_acceleration.x()=.11; }
    if(variant==7)candidate.join_orientation.w()=2.;
    EXPECT_FALSE(core.receiveTrajectory(candidate,100.1,10.1)) << variant;
    EXPECT_TRUE(core.active()); EXPECT_EQ(core.trajectoryId(),1);
  }
}

TEST(TrackerV2, MatchingPositionCannotHideVelocityMismatchOrUnboundedCurveDynamics) {
  for(int variant=0;variant<3;++variant) {
    TrackerCore core(config()); prepare(core); auto body=odom(100.1,.025);
    if(variant==0)body.velocity_in_frame.x()=-.25;
    ASSERT_TRUE(core.receiveOdom(body,100.1,10.1));
    auto candidate=curve(2,100.05);
    if(variant==1)for(auto& p:candidate.points)p.x()*=2.;
    if(variant==2)candidate.points[10].y()=.1;
    certify(candidate,100.1,variant==1?.05:.1);
    EXPECT_FALSE(core.receiveTrajectory(candidate,100.1,10.1)) << variant;
    EXPECT_EQ(core.trajectoryId(),1);
  }
}

TEST(TrackerV2, ActualVendorCurveAllowsDistantBendButRejectsUnbrakeableEntryWithoutResettingLimiter) {
  for(bool immediate:{false,true}) {
    TrackerCore core(config());prepare(core);core.recordAppliedOutput(.296,0.,100.,10.);
    auto replacement=curve(2);const double onset=immediate?0.:.6;
    for(auto& p:replacement.points) {
      const double d=std::max(0.,p.x()-onset);
      p.y()=d<=.1?2.*d*d:.02+.4*(d-.1);
    }
    certify(replacement);
    const bool accepted=core.receiveTrajectory(replacement,100.,10.);
    if(immediate) {
      EXPECT_FALSE(accepted);EXPECT_EQ(core.candidateReason(),"curvature_braking_limit_below_current_command");
      EXPECT_EQ(core.trajectoryId(),1);
    } else {EXPECT_TRUE(accepted)<<core.candidateReason();EXPECT_EQ(core.trajectoryId(),2);}
    ASSERT_TRUE(core.receiveOdom(odom(100.02),100.02,10.02));core.refreshTaskLease(10.02);
    const auto out=core.step(100.02,10.02);
    EXPECT_GE(out.forward,.296-config().max_acceleration*.02-1e-9);
    EXPECT_LE(out.forward,.30); // candidate admission did not zero/restart the limiter
  }
}

TEST(TrackerV2, HandoffCannotJumpPastBoundedArcWindowOrOntoOtherFloor) {
  for(int variant=0;variant<3;++variant) {
    TrackerCore core(config()); prepare(core); auto body=odom(100.1,.8);
    if(variant==1)body.position={.025,0.,.8};
    if(variant==2)body.position={.025,0.,.55};
    ASSERT_TRUE(core.receiveOdom(body,100.1,10.1));
    auto candidate=curve(2,100.05); certify(candidate,100.1,.1);
    if(variant==2)candidate.identity.segment_id="floor2";
    EXPECT_FALSE(core.receiveTrajectory(candidate,100.1,10.1)) << variant;
    EXPECT_EQ(core.trajectoryId(),1);
  }
}

namespace {
wire::ExecutionVersion version(std::uint64_t generation=1) {
  wire::ExecutionVersion v;v.schema_version=3;v.session_id="session";v.task_id="task";
  v.route_id="route";v.route_hash=std::string(64,'a');v.map_version_id="map";
  v.reference_generation=generation;v.segment_id="floor1";v.anchor_id="anchor";
  v.anchor_revision=v.context_sequence=v.localization_epoch=v.map_geometry_revision=1;v.localization_seed_id="seed";return v;
}
wire::SupportReference supportWire(const wire::ExecutionVersion& v=version()) {
  wire::SupportReference s;s.version=v;s.support_reference_id="support";s.support_hash=std::string(64,'b');
  s.support_map_sha256=std::string(64,'c');s.floor_id="floor1";s.segment_kind="floor";
  s.required_mode="general";s.frame_id="d1max_loc_odom";s.source_stamp=stamp(100.);
  s.support_xy_radius_m=.3;s.body_reference_height_m=.55;s.max_support_slope_rad=.15;s.max_support_step_m=.05;s.verified=true;
  for(int i=-2;i<=60;++i){geometry_msgs::msg::Point p;p.x=i*.05;s.support_ground_xyz.push_back(p);}return s;
}
wire::ReferenceProposal proposalWire(const wire::ExecutionVersion& v=version()) {
  wire::ReferenceProposal p;p.proposal_id="proposal";p.version=v;p.expected_trajectory_id=-1;
  p.source_stamp=stamp(100.);p.valid_until=stamp(100.9);p.transport_mode="isolated_mock";
  p.reference.path.header.frame_id="d1max_loc_odom";p.reference.point_reference="body_center";
  p.goal_position.x=2.;p.goal_position.z=.55;return p;
}
wire::TrajectoryValidation proofWire(const wire::ExecutionVersion& v=version()) {
  wire::TrajectoryValidation p;p.version=v;p.proposal_id="proposal";p.trajectory_id=1;p.sequence=1;
  p.source_stamp=p.check_begin=p.check_end=p.body_source_stamp=p.front_ray_source_stamp=p.rear_ray_source_stamp=stamp(100.);
  p.valid_until=stamp(100.25);
  p.checked_from_time=0.;p.checked_to_time=p.curve_duration=4.;
  p.map_snapshot_revision=1;p.support_reference_id="support";p.support_hash=std::string(64,'b');
  p.valid=p.whole_curve=true;p.frame_id="d1max_loc_odom";p.collision_policy="observed_free";p.transport_mode="isolated_mock";return p;
}
wire::ExecutionPermit permitWire(const wire::ExecutionVersion& v=version()) {
  wire::ExecutionPermit p;p.version=v;p.execution_id="exec";p.control_epoch=1;p.confirmation_id="human";
  p.sdk_session="sdk";p.sdk_arm_generation=1;p.sequence=1;p.source_stamp=stamp(100.);p.valid_until=stamp(100.5);
  p.trajectory_id=1;p.validation_sequence=1;p.geometry_committed=p.allowed=true;p.phase="tracking";
  p.transport_mode="isolated_mock";p.frame_id="d1max_loc_odom";p.goal_position.x=2.;p.goal_position.z=.55;return p;
}
Config executionConfig(){auto c=config();c.require_support_reference=true;c.external_goal_completion=true;return c;}
void stage(ExecutionContract& gate,Trajectory candidate=curve()) {
  ASSERT_TRUE(gate.core().receiveOdom(odom(),100.,10.));gate.proposal(proposalWire(),100.);
  gate.candidate(std::move(candidate));gate.support(supportWire());gate.validation(proofWire(),100.);
}
wire::ExecutionCommitAck initialWriterAck(const wire::ExecutionPermit& p) {
  wire::ExecutionCommitAck a;a.schema_version=1;a.sequence=1;a.applied=a.write_submitted=true;
  a.commit_sequence=1;a.candidate_version=p.version;a.candidate_trajectory_id=p.trajectory_id;
  a.execution_id=p.execution_id;a.control_epoch=p.control_epoch;a.sdk_session=p.sdk_session;
  a.sdk_arm_generation=p.sdk_arm_generation;a.permit_sequence=p.sequence;a.transport_mode=p.transport_mode;
  a.applied_at=stamp(100.02);a.valid_until=p.valid_until;return a;
}
wire::ExecutionHandoffGrant stageWriterHandoff(ExecutionContract& gate,std::uint64_t generation=2) {
  stage(gate);EXPECT_TRUE(gate.prepare(100.,10.)->accepted);auto incumbent=permitWire();
  EXPECT_TRUE(gate.permit(incumbent,100.,10.));gate.step(100.02,10.02);
  EXPECT_TRUE(gate.commitAck(initialWriterAck(incumbent),100.02,10.02));
  const auto v=version(generation);auto p=proposalWire(v);p.expected_version=incumbent.version;p.expected_trajectory_id=1;
  gate.proposal(p,100.02);auto t=curve(2);t.generation=generation;t.identity=d1max_trajectory_tracker::identity(v);gate.candidate(t);
  gate.support(supportWire(v));auto proof=proofWire(v);proof.trajectory_id=2;proof.sequence=2;
  gate.validation(proof,100.02);EXPECT_TRUE(gate.prepare(100.02,10.02)->accepted);
  wire::ExecutionHandoffGrant g;g.schema_version=2;g.handoff_id="handoff";g.sequence=1;
  g.expected_commit_sequence=1;g.incumbent=incumbent;g.candidate=permitWire(v);
  g.candidate.sequence=2;g.candidate.trajectory_id=2;g.candidate.validation_sequence=2;g.candidate.geometry_committed=false;
  g.source_stamp=stamp(100.02);g.valid_until=g.transition_deadline=stamp(100.22);
  g.retain_incumbent_until=incumbent.valid_until;return g;
}
wire::ExecutionCommitAck preparedWriterAck(const wire::ExecutionHandoffGrant& g,const wire::PreparedMotionDemand& x) {
  wire::ExecutionCommitAck a;a.schema_version=1;a.handoff_id=g.handoff_id;a.grant_sequence=g.sequence;
  a.sequence=2;a.previous_commit_sequence=g.expected_commit_sequence;a.commit_sequence=g.expected_commit_sequence+1;
  a.incumbent_version=g.incumbent.version;a.candidate_version=g.candidate.version;
  a.incumbent_trajectory_id=g.incumbent.trajectory_id;a.candidate_trajectory_id=g.candidate.trajectory_id;
  a.execution_id=g.candidate.execution_id;a.sdk_session=g.candidate.sdk_session;a.transport_mode=g.candidate.transport_mode;
  a.control_epoch=g.candidate.control_epoch;a.sdk_arm_generation=g.candidate.sdk_arm_generation;
  a.permit_sequence=g.candidate.sequence;a.demand_sequence=x.demand.sequence;
  a.motion_validation_sequence=99;a.entry_admission_sequence=x.entry_admission.sequence;
  a.applied_at=x.demand.source_stamp;a.demand_source_stamp=x.demand.source_stamp;
  a.demand_body_source_stamp=x.demand.body_source_stamp;a.entry_source_stamp=x.entry_source_stamp;
  a.body_source_stamp=x.entry_source_stamp;a.valid_until=x.demand.valid_until;
  a.measured_pose=x.measured_pose;a.measured_twist=x.measured_twist;a.applied_velocity=x.demand.velocity;
  a.curve_time=x.curve_time;a.applied=a.write_submitted=true;return a;
}
}
TEST(TrackerGeometryReceipt, PreparationAndUnconnectedFreshBodyCannotClaimSoftwareInstallation) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true);
  auto t=curve();t.join_source_stamp=99.69;stage(gate,t);
  const auto prepared=gate.prepare(100.,10.);ASSERT_TRUE(prepared&&prepared->accepted);
  EXPECT_FALSE(gate.geometryReceipt());EXPECT_FALSE(gate.core().hasInstalledGeometry());
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.12,-.1),100.12,10.12));
  auto p=permitWire();p.source_stamp=stamp(100.12);p.valid_until=stamp(100.32);
  EXPECT_FALSE(gate.permit(p,100.12,10.12));
  EXPECT_EQ(gate.lastPermitReject(),"commit_candidate:entry_measured_body_not_on_candidate");
  EXPECT_FALSE(gate.geometryReceipt());EXPECT_FALSE(gate.core().hasInstalledGeometry());
}
TEST(TrackerEntryReobservation, FreshWholeProofUsesActualNewBoundaryNotAnExpiredWorkerSample) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true);
  auto original=curve();original.join_source_stamp_ns=100000000017LL;stage(gate,original);
  auto body=odom(100.55,.125);body.source_stamp_ns=100550000019LL;
  ASSERT_TRUE(gate.core().receiveOdom(body,100.55,10.55));
  auto proof=proofWire();proof.sequence=2;
  proof.source_stamp=proof.check_begin=proof.check_end=proof.body_source_stamp=
    proof.front_ray_source_stamp=proof.rear_ray_source_stamp=stamp(100.55);
  proof.valid_until=stamp(100.8);proof.valid_start_time=.5;proof.valid_start_arc_length=.125;
  gate.validation(proof,100.55);
  const auto exact_now=SourceTime::fromNanoseconds(body.source_stamp_ns);
  const auto admission=gate.prepare(exact_now,10.55);ASSERT_TRUE(admission&&admission->accepted);
  EXPECT_EQ(admission->validation_sequence,2u);EXPECT_EQ(timeNs(admission->body_source_stamp),body.source_stamp_ns);
  auto p=permitWire();p.validation_sequence=2;p.source_stamp=stamp(100.55);p.valid_until=stamp(100.75);
  ASSERT_TRUE(gate.permit(p,exact_now,10.55));
  const auto& diagnostic=gate.core().joinDiagnostic();
  EXPECT_TRUE(diagnostic.entry_reobserved);EXPECT_EQ(diagnostic.original_join_source_stamp_ns,100000000017LL);
  EXPECT_EQ(diagnostic.join_source_stamp_ns,body.source_stamp_ns);
  EXPECT_NEAR(gate.core().progressTime(),.5,.005);EXPECT_NEAR(gate.core().progressArc(),.125,.001);
  EXPECT_DOUBLE_EQ(original.join_source_stamp,100.);EXPECT_EQ(original.join_source_stamp_ns,100000000017LL);
  EXPECT_EQ(proof.valid_until,stamp(100.8)); // no original certificate lease was renewed
}
TEST(TrackerEntryReobservation, CurrentMeasurementAndFiniteNativeDomainRemainStrict) {
  auto original=curve();original.join_source_stamp_ns=100000000017LL;
  const auto cache=EntryCurveCache::build(original);ASSERT_TRUE(cache);
  auto body=odom(100.55,.125);body.source_stamp_ns=100550000019LL;std::string reason;
  auto good=cache->observe(original,body,config(),100.55,100.55,.5,.125,reason);
  ASSERT_TRUE(good);EXPECT_TRUE(reason.empty());EXPECT_EQ(good->join_source_stamp_ns,body.source_stamp_ns);
  const auto fail=[&](Odom o,double proof_body,double time,double arc,const char* expected){
    EXPECT_FALSE(cache->observe(original,o,config(),100.55,proof_body,time,arc,reason));EXPECT_EQ(reason,expected);
  };
  auto stale=body;stale.stamp=stale.posterior_stamp=stale.imu_stamp=100.4;
  stale.source_stamp_ns=100400000000LL;
  fail(stale,100.4,.5,.125,"entry_body_not_fresh");
  auto wrong_epoch=body;wrong_epoch.localization_epoch=2;
  fail(wrong_epoch,100.55,.5,.125,"entry_body_identity_mismatch");
  auto off=body;off.position.z()+=.013;
  fail(off,100.55,.5,.125,"entry_measured_body_not_on_candidate");
  auto wrong_velocity=body;wrong_velocity.velocity_in_frame.x()=.199;
  fail(wrong_velocity,100.55,.5,.125,"entry_measured_velocity_off_candidate");
  fail(body,100.56,.5,.125,"entry_body_precedes_or_exceeds_proof");
  fail(body,100.55,.5,.20,"entry_proof_curve_domain_invalid");
  fail(body,100.55,4.01,.125,"entry_proof_curve_domain_invalid");
}
TEST(TrackerEntryReobservation, StaleOrNegativeWholeProofCannotRefreshEntry) {
  for(bool invalid:{false,true}) {
    ExecutionContract gate(executionConfig(),"isolated_mock",{},true);stage(gate);
    ASSERT_TRUE(gate.core().receiveOdom(odom(100.55),100.55,10.55));
    auto proof=proofWire();proof.sequence=2;
    if(invalid) {proof.valid=false;proof.source_stamp=proof.check_begin=proof.check_end=
      proof.body_source_stamp=proof.front_ray_source_stamp=proof.rear_ray_source_stamp=stamp(100.55);
      proof.valid_until=stamp(100.8);}
    gate.validation(proof,100.55);
    const auto admission=gate.prepare(100.55,10.55);ASSERT_TRUE(admission);EXPECT_FALSE(admission->accepted);
    EXPECT_EQ(admission->reason,"validation_not_fresh");EXPECT_FALSE(gate.geometryReceipt());
  }
}
TEST(TrackerGeometryReceipt, InstalledFactsStayFixedAcrossProofRenewalHoldingAndRepeatedBody) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true);stage(gate);
  const auto admission=gate.prepare(100.,10.);ASSERT_TRUE(admission&&admission->accepted);
  auto p=permitWire();p.allowed=false;p.phase="preview";
  ASSERT_TRUE(gate.permit(p,100.,10.));
  const auto installed=gate.geometryReceipt();ASSERT_TRUE(installed&&installed->installed);
  EXPECT_EQ(installed->schema_version,1);EXPECT_EQ(installed->frame_id,"d1max_loc_odom");
  EXPECT_EQ(installed->transport_mode,"isolated_mock");EXPECT_EQ(installed->version,p.version);
  EXPECT_EQ(installed->permit_sequence,p.sequence);EXPECT_EQ(installed->admission_sequence,admission->sequence);
  EXPECT_EQ(installed->validation_sequence,admission->validation_sequence);
  EXPECT_EQ(installed->installed_at,stamp(100.));
  auto body=odom(100.02);body.source_stamp_ns=100020000019LL;
  ASSERT_TRUE(gate.core().receiveOdom(body,100.02,10.02));
  auto v=proofWire();v.sequence=2;gate.validation(v,100.02);
  p.sequence=2;p.validation_sequence=2;p.source_stamp=stamp(100.02);p.valid_until=stamp(100.22);
  ASSERT_TRUE(gate.permit(p,100.02,10.02));EXPECT_TRUE(gate.step(100.02,10.02).hold);
  const auto renewed=gate.geometryReceipt();ASSERT_TRUE(renewed&&renewed->installed);
  EXPECT_EQ(renewed->installation_sequence,installed->installation_sequence);
  EXPECT_EQ(renewed->installed_at,installed->installed_at);EXPECT_EQ(renewed->permit_sequence,installed->permit_sequence);
  EXPECT_EQ(renewed->validation_sequence,installed->validation_sequence);
  EXPECT_EQ(renewed->admission_sequence,installed->admission_sequence);
  EXPECT_EQ(timeNs(renewed->body_source_stamp),body.source_stamp_ns);
  const auto repeated=gate.geometryReceipt();ASSERT_TRUE(repeated&&repeated->installed);
  EXPECT_GT(repeated->sequence,renewed->sequence);EXPECT_EQ(repeated->body_source_stamp,renewed->body_source_stamp);
  gate.cancel("test_cancel");const auto retired=gate.geometryReceipt();ASSERT_TRUE(retired);
  EXPECT_FALSE(retired->installed);EXPECT_EQ(retired->installed_at,installed->installed_at);
  EXPECT_EQ(retired->installation_sequence,installed->installation_sequence);
}
TEST(TrackerGeometryReceipt, MotionFencePreservesActualInstallationAndCannotBorrowMovingHandoff) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true);
  auto t=curve();for(auto& p:t.points) {const double d=std::max(0.,p.x()-.6);p.y()=d<=.1?2.*d*d:.02+.4*(d-.1);}
  stage(gate,t);ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
  const auto p=permitWire();ASSERT_TRUE(gate.permit(p,100.,10.));
  ASSERT_TRUE(gate.commitAck(initialWriterAck(p),100.02,10.02));
  const auto original=gate.geometryReceipt();ASSERT_TRUE(original&&original->installed);
  gate.core().recordAppliedOutput(.296,0.,100.,10.);
  auto body=odom(100.02,.3);ASSERT_TRUE(gate.core().receiveOdom(body,100.02,10.02));
  ASSERT_EQ(gate.step(100.02,10.02).reason,"tracking");
  body=odom(100.04,.57);body.source_stamp_ns=100040000017LL;
  ASSERT_TRUE(gate.core().receiveOdom(body,100.04,10.04));
  const auto demand=gate.step(100.04,10.04);EXPECT_TRUE(demand.hold);
  EXPECT_EQ(demand.reason,"braking_envelope_reentry_required");
  const auto fenced=gate.geometryReceipt();ASSERT_TRUE(fenced&&fenced->installed);
  EXPECT_EQ(fenced->reason,"braking_envelope_reentry_required");EXPECT_EQ(fenced->version,original->version);
  EXPECT_EQ(fenced->installation_sequence,original->installation_sequence);EXPECT_EQ(fenced->installed_at,original->installed_at);
  EXPECT_EQ(fenced->permit_sequence,original->permit_sequence);EXPECT_EQ(fenced->validation_sequence,original->validation_sequence);
  EXPECT_EQ(fenced->admission_sequence,original->admission_sequence);EXPECT_EQ(timeNs(fenced->body_source_stamp),body.source_stamp_ns);
  wire::ExecutionHandoffGrant g;g.schema_version=2;g.handoff_id="moving";g.sequence=1;g.expected_commit_sequence=1;
  g.incumbent=p;g.candidate=permitWire(version(2));g.candidate.geometry_committed=false;g.candidate.trajectory_id=2;
  EXPECT_FALSE(gate.handoff(g,100.04,10.04));EXPECT_EQ(gate.handoffReason(),"braking_envelope_requires_stationary_reentry");
  EXPECT_TRUE(gate.core().requiresMeasuredReentry());EXPECT_EQ(gate.writerCommitSequence(),1u);
  gate.cancel("explicit_cancel");const auto cancelled=gate.geometryReceipt();ASSERT_TRUE(cancelled);
  EXPECT_FALSE(cancelled->installed);EXPECT_EQ(cancelled->installation_sequence,original->installation_sequence);
}
TEST(TrackerGeometryReceipt, UnixOriginalBodyFactsAreExactAtInstallationAndDoNotRenewWithTimer) {
  const auto ns=1791004686079466628LL;const double base=seconds(stampNs(ns));
  const auto now=SourceTime::fromNanoseconds(ns);
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true);
  auto body=odom(base);body.source_stamp_ns=body.posterior_stamp_ns=body.imu_stamp_ns=ns;
  ASSERT_TRUE(gate.core().receiveOdom(body,now,10.));
  auto proposal=proposalWire();proposal.source_stamp=stampNs(ns);proposal.valid_until=stampNs(ns+900000000LL);
  gate.proposal(proposal,base);auto t=curve(1,base);t.join_source_stamp_ns=ns;gate.candidate(t);
  auto s=supportWire();s.source_stamp=stampNs(ns);gate.support(s);
  auto v=proofWire();v.source_stamp=v.check_begin=v.check_end=v.body_source_stamp=
    v.front_ray_source_stamp=v.rear_ray_source_stamp=stampNs(ns);v.valid_until=stampNs(ns+250000000LL);gate.validation(v,base);
  const auto a=gate.prepare(now,10.);ASSERT_TRUE(a&&a->accepted);
  auto p=permitWire();p.source_stamp=stampNs(ns);p.valid_until=stampNs(ns+500000000LL);
  ASSERT_TRUE(gate.permit(p,SourceTime::fromNanoseconds(ns+1000000LL),10.001));const auto installed=gate.geometryReceipt();ASSERT_TRUE(installed&&installed->installed);
  EXPECT_EQ(timeNs(installed->body_source_stamp),ns);EXPECT_EQ(installed->admission_sequence,a->sequence);
  const auto again=gate.geometryReceipt();ASSERT_TRUE(again);EXPECT_EQ(again->body_source_stamp,installed->body_source_stamp);
  EXPECT_EQ(again->installed_at,installed->installed_at);
}
TEST(TrackerWriterCAS, InitialAckBindsActualInstallationNotLatestIntentAndCannotRollback) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true);stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);const auto first=permitWire();
  ASSERT_TRUE(gate.permit(first,100.,10.));const auto original=gate.geometryReceipt();ASSERT_TRUE(original);
  gate.candidate(curve(2));auto proof=proofWire();proof.sequence=2;proof.trajectory_id=2;gate.validation(proof,100.03);
  ASSERT_TRUE(gate.prepare(100.03,10.03)->accepted);
  auto next=first;next.sequence=2;next.trajectory_id=2;next.validation_sequence=2;
  next.source_stamp=stamp(100.03);ASSERT_TRUE(gate.permit(next,100.03,10.03));
  ASSERT_EQ(gate.core().trajectoryId(),2);
  auto a=initialWriterAck(first);ASSERT_TRUE(gate.commitAck(a,100.04,10.04));
  EXPECT_EQ(gate.writerCommitSequence(),1U);EXPECT_EQ(gate.core().trajectoryId(),1);
  EXPECT_EQ(gate.initialAckReason(),"initial_writer_applied_fact_restored_hold");
  EXPECT_FALSE(gate.core().hasInstalledGeometry());EXPECT_TRUE(gate.step(100.04,10.04).hold);
  const auto receipt=gate.geometryReceipt();ASSERT_TRUE(receipt);
  EXPECT_FALSE(receipt->installed);EXPECT_EQ(receipt->installed_at,original->installed_at);
  EXPECT_EQ(receipt->installation_sequence,original->installation_sequence);
  ++next.sequence;EXPECT_FALSE(gate.permit(next,100.05,10.05));
  ++a.sequence;a.write_acknowledged=true;EXPECT_TRUE(gate.commitAck(a,100.05,10.05));
  EXPECT_TRUE(gate.step(100.06,10.06).hold);EXPECT_EQ(gate.core().trajectoryId(),1);
}
TEST(TrackerWriterCAS, InitialFactRequiresSignedLeaseAndRealInstallationButAcceptsExpiredReceiptTime) {
  for(const auto variant:{0U,1U,2U,3U}) {
    ExecutionContract gate(executionConfig(),"isolated_mock",{},true);stage(gate);
    auto p=permitWire();
    if(variant!=0){ASSERT_TRUE(gate.prepare(100.,10.)->accepted);ASSERT_TRUE(gate.permit(p,100.,10.));}
    auto a=initialWriterAck(p);
    if(variant==1)a.permit_sequence=99;
    if(variant==2)a.sdk_session="foreign_sdk";
    if(variant<3){EXPECT_FALSE(gate.commitAck(a,100.03,10.03));EXPECT_EQ(gate.writerCommitSequence(),0U);}
    else {
      a.valid_until=stamp(100.03);ASSERT_TRUE(gate.commitAck(a,100.04,10.04));
      EXPECT_EQ(gate.writerCommitSequence(),1U);EXPECT_TRUE(gate.step(100.04,10.04).hold);
      EXPECT_FALSE(gate.core().hasInstalledGeometry());
    }
  }
}
TEST(TrackerWriterCAS, ConditionalGrantNeverChangesIncumbentBeforeActualWriterFact) {
  for(const auto generation:{1U,2U}) {
  ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'),true);
  const auto g=stageWriterHandoff(gate,generation);ASSERT_TRUE(gate.handoff(g,100.02,10.02));
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.04),100.04,10.04));
  const auto prepared=gate.preparedStep(100.04,10.04);ASSERT_TRUE(prepared);
  EXPECT_EQ(gate.core().trajectoryId(),1);EXPECT_EQ(prepared->demand.trajectory_id,2);
  EXPECT_GT(prepared->demand.velocity.linear.x,0.); // not a zero-dt fake HOLD.
  EXPECT_EQ(prepared->entry_source_stamp,stamp(100.04));
  EXPECT_EQ(gate.step(100.04,10.04).trajectory_id,1);
  auto premature=g.candidate;premature.geometry_committed=true;
  EXPECT_FALSE(gate.permit(premature,100.04,10.04));EXPECT_EQ(gate.core().trajectoryId(),1);
  auto a=preparedWriterAck(g,*prepared);ASSERT_TRUE(gate.commitAck(a,100.04,10.04));
  EXPECT_EQ(gate.core().trajectoryId(),2);EXPECT_EQ(gate.writerCommitSequence(),2U);
  EXPECT_EQ(gate.step(100.06,10.06).trajectory_id,2);
  auto old=g.incumbent;old.sequence=3;EXPECT_FALSE(gate.permit(old,100.06,10.06));
  ++a.sequence;a.write_acknowledged=true;EXPECT_TRUE(gate.commitAck(a,100.07,10.07));
  EXPECT_EQ(gate.core().trajectoryId(),2);
  }
}
TEST(TrackerWriterCAS, FixedGrantRejectsOverwriteAndPreparationUsesActualMovingBoundary) {
  ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'),true);
  auto g=stageWriterHandoff(gate);ASSERT_TRUE(gate.handoff(g,100.02,10.02));
  auto different=g;different.handoff_id="different";EXPECT_FALSE(gate.handoff(different,100.03,10.03));
  different=g;different.candidate.trajectory_id=3;EXPECT_FALSE(gate.handoff(different,100.03,10.03));
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.04,.01,.55),100.04,10.04));
  const auto x=gate.preparedStep(100.04,10.04);ASSERT_TRUE(x);
  EXPECT_GT(x->curve_time,0.);EXPECT_LE(x->position_error_m,.0125);EXPECT_LE(x->velocity_error_mps,.05);
  auto newer=curve(3);newer.generation=3;newer.identity=d1max_trajectory_tracker::identity(version(3));gate.candidate(newer);
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.06,.015,.55),100.06,10.06));
  ASSERT_TRUE(gate.preparedStep(100.06,10.06));EXPECT_EQ(gate.core().trajectoryId(),1);
}
TEST(TrackerWriterCAS, PreparedExpiryCopiesOriginalFineNanosecondCapsWithoutRoundTrip) {
  for(const auto limiting_cap:{0U,1U,2U}) {
    ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'),true);
    auto g=stageWriterHandoff(gate);
    const auto original=stampNs(100220000001LL);
    g.valid_until=g.transition_deadline=g.candidate.valid_until=stampNs(100230000019LL);
    if(limiting_cap==0)g.valid_until=g.transition_deadline=original;
    if(limiting_cap==1)g.transition_deadline=original;
    if(limiting_cap==2)g.candidate.valid_until=original;
    ASSERT_TRUE(gate.handoff(g,100.02,10.02));
    auto measured=odom(100.17,.03,.55);measured.source_stamp_ns=100170000019LL;
    ASSERT_TRUE(gate.core().receiveOdom(measured,100.17,10.17));
    const auto prepared=gate.preparedStep(100.17,10.17);ASSERT_TRUE(prepared);
    EXPECT_EQ(prepared->demand.valid_until,original);
    EXPECT_EQ(prepared->entry_admission.valid_until,original);
    EXPECT_EQ(timeNs(prepared->demand.body_source_stamp),measured.source_stamp_ns);
    EXPECT_EQ(prepared->entry_source_stamp,prepared->demand.body_source_stamp);
    EXPECT_LE(timeNs(prepared->demand.valid_until),timeNs(g.valid_until));
    EXPECT_LE(timeNs(prepared->demand.valid_until),timeNs(g.transition_deadline));
    EXPECT_LE(timeNs(prepared->demand.valid_until),timeNs(g.candidate.valid_until));
  }
}
TEST(TrackerTimeContract, RealUnixLeaseFactsRemainExactAndCanOnlyCapFreshDeadline) {
  // These are the two original deadlines from the local_gap_01 graph. Its
  // prepared commands increased each by 1 ns before this exact-copy repair.
  for(const auto original_ns:{1791002417245872020LL,1791002433646161317LL}) {
    const auto original=stampNs(original_ns);
    const auto fresh=stampNs(original_ns+50000000LL);
    EXPECT_EQ(timeNs(originalTimeCap(fresh,{original})),original_ns);
    const auto earlier=stampNs(original_ns-1LL);
    EXPECT_EQ(timeNs(originalTimeCap(fresh,{original,earlier})),original_ns-1LL);
    EXPECT_EQ(timeNs(originalTimeCap(earlier,{original})),original_ns-1LL);
    const auto generated_source=stamp(seconds(original));
    EXPECT_EQ(timeNs(stampNs(timeNs(generated_source)+100000000LL))-timeNs(generated_source),100000000LL);
    ExecutionContract gate(executionConfig(),"isolated_mock");
    auto measured=odom(seconds(original));measured.source_stamp_ns=measured.posterior_stamp_ns=measured.imu_stamp_ns=original_ns;
    ASSERT_TRUE(gate.core().receiveOdom(measured,seconds(original),10.));
    const auto demand=gate.step(seconds(original)+.02,10.02);
    EXPECT_EQ(timeNs(demand.body_source_stamp),original_ns);
    EXPECT_EQ(timeNs(demand.valid_until)-timeNs(demand.source_stamp),100000000LL);
  }
}
TEST(TrackerTimeContract, V10EqualUnixSourceIsAdmittedAndTrueFutureNanosecondIsRejected) {
  constexpr std::int64_t ns=1791124691902856036LL;
  const auto now=SourceTime::fromNanoseconds(ns);
  auto body=odom(seconds(stampNs(ns)));
  body.source_stamp_ns=body.posterior_stamp_ns=body.imu_stamp_ns=ns;
  auto t=curve(1,seconds(stampNs(ns)));t.start_time_ns=t.join_source_stamp_ns=ns;
  EXPECT_EQ(joinEvidenceFailure(t,true,body,now,4.,config()),nullptr);
  // These adjacent integer sources have the same Unix-scale double value.
  ++t.join_source_stamp_ns;
  EXPECT_STREQ(joinEvidenceFailure(t,true,body,now,4.,config()),"join_source_in_future");
  t.join_source_stamp_ns=ns;
  ++body.posterior_stamp_ns;
  EXPECT_STREQ(sourceEvidenceFailure(body,now,config()),"body_posterior_after_state");
  body.posterior_stamp_ns=ns;++body.imu_stamp_ns;
  EXPECT_STREQ(sourceEvidenceFailure(body,now,config()),"body_imu_after_state");
}
TEST(TrackerTimeContract, IntegerFreshnessUsesOriginalLimitsAtNanosecondBoundary) {
  constexpr std::int64_t ns=1791124691902856036LL;
  auto body=odom(seconds(stampNs(ns)));body.source_stamp_ns=body.posterior_stamp_ns=body.imu_stamp_ns=ns;
  EXPECT_EQ(sourceEvidenceFailure(body,SourceTime::fromNanoseconds(ns+100000000LL),config()),nullptr);
  EXPECT_STREQ(sourceEvidenceFailure(body,SourceTime::fromNanoseconds(ns+100000001LL),config()),"body_imu_not_fresh");
  const auto now=SourceTime::fromNanoseconds(ns);
  EXPECT_TRUE(freshStamp(now,stampNs(ns),.1,0.));
  EXPECT_FALSE(freshStamp(now,stampNs(ns+1),.1,0.));
  EXPECT_TRUE(freshStamp(now,stampNs(ns+20000000LL),.1));
  EXPECT_FALSE(freshStamp(now,stampNs(ns+20000001LL),.1));
  EXPECT_TRUE(timed(now,stampNs(ns),stampNs(ns+1),.25));
  EXPECT_FALSE(timed(SourceTime::fromNanoseconds(ns+1),stampNs(ns),stampNs(ns+1),.25));
}
TEST(TrackerTimeContract, OriginalNanosecondOrderingDoesNotCollapseAdjacentBodySources) {
  constexpr std::int64_t ns=1791124691902856036LL;
  TrackerCore core(config());auto body=odom(seconds(stampNs(ns)));
  body.source_stamp_ns=body.posterior_stamp_ns=body.imu_stamp_ns=ns;
  ASSERT_TRUE(core.receiveOdom(body,SourceTime::fromNanoseconds(ns),10.));
  ++body.source_stamp_ns;++body.posterior_stamp_ns;++body.imu_stamp_ns;
  ASSERT_TRUE(core.receiveOdom(body,SourceTime::fromNanoseconds(ns+1),10.01));
  EXPECT_EQ(core.odometry().source_stamp_ns,ns+1);
  auto reordered=body;reordered.source_stamp_ns=ns;reordered.localization_seed_id="foreign-old-seed";
  EXPECT_FALSE(core.receiveOdom(reordered,SourceTime::fromNanoseconds(ns+2),10.02));
  EXPECT_EQ(core.odometry().source_stamp_ns,ns+1);
}
TEST(TrackerTimeContract, ExactUnixClockFlowsThroughPrepareCommitDemandAndInstallation) {
  constexpr std::int64_t ns=1791124691902856036LL;
  const auto now=SourceTime::fromNanoseconds(ns);
  ExecutionContract gate(executionConfig(),"isolated_mock");
  auto body=odom(seconds(stampNs(ns)));body.source_stamp_ns=body.posterior_stamp_ns=body.imu_stamp_ns=ns;
  ASSERT_TRUE(gate.receiveOdom(body,now,10.));
  auto proposal=proposalWire();proposal.source_stamp=stampNs(ns);proposal.valid_until=stampNs(ns+900000000LL);
  gate.proposal(proposal,now);
  auto t=curve(1,seconds(stampNs(ns)));t.start_time_ns=t.join_source_stamp_ns=ns;gate.candidate(t);
  auto support=supportWire();support.source_stamp=stampNs(ns);gate.support(support);
  auto proof=proofWire();proof.source_stamp=proof.check_begin=proof.check_end=proof.body_source_stamp=
    proof.front_ray_source_stamp=proof.rear_ray_source_stamp=stampNs(ns);
  proof.valid_until=stampNs(ns+250000000LL);gate.validation(proof,now);
  const auto admission=gate.prepare(now,10.);ASSERT_TRUE(admission&&admission->accepted);
  EXPECT_EQ(timeNs(admission->checked_at),ns);EXPECT_EQ(timeNs(admission->body_source_stamp),ns);
  auto permit=permitWire();permit.source_stamp=stampNs(ns);permit.valid_until=stampNs(ns+500000000LL);
  ASSERT_TRUE(gate.permit(permit,now,10.));
  const auto installed=gate.geometryReceipt();ASSERT_TRUE(installed&&installed->installed);
  EXPECT_EQ(timeNs(installed->installed_at),ns);EXPECT_EQ(timeNs(installed->body_source_stamp),ns);
  const auto demand=gate.step(SourceTime::fromNanoseconds(ns+20000001LL),10.02);
  EXPECT_EQ(timeNs(demand.source_stamp),ns+20000001LL);
  EXPECT_EQ(timeNs(demand.body_source_stamp),ns);
  EXPECT_EQ(timeNs(demand.valid_until),ns+120000001LL);
  const auto& diagnostic=gate.core().joinDiagnostic();
  EXPECT_EQ(diagnostic.checked_now_ns,ns);
  EXPECT_EQ(sourceDeltaSeconds(diagnostic.checked_now_ns,diagnostic.join_source_stamp_ns),0.);
  EXPECT_EQ(sourceDeltaSeconds(diagnostic.checked_now_ns,diagnostic.body_source_stamp_ns),0.);
}
TEST(TrackerTimeContract, WholeCurveProofAbsoluteExpiryDoesNotGainOneNanosecond) {
  constexpr std::int64_t ns=1791124691902856036LL;
  ExecutionContract gate(executionConfig(),"isolated_mock");const auto now=SourceTime::fromNanoseconds(ns);
  auto body=odom(seconds(stampNs(ns)));body.source_stamp_ns=body.posterior_stamp_ns=body.imu_stamp_ns=ns;
  ASSERT_TRUE(gate.receiveOdom(body,now,10.));
  auto proposal=proposalWire();proposal.source_stamp=stampNs(ns);proposal.valid_until=stampNs(ns+900000000LL);gate.proposal(proposal,now);
  auto t=curve(1,seconds(stampNs(ns)));t.start_time_ns=t.join_source_stamp_ns=ns;gate.candidate(t);
  auto support=supportWire();support.source_stamp=stampNs(ns);gate.support(support);
  auto proof=proofWire();proof.source_stamp=proof.check_begin=proof.check_end=proof.body_source_stamp=
    proof.front_ray_source_stamp=proof.rear_ray_source_stamp=stampNs(ns);proof.valid_until=stampNs(ns+1);
  gate.validation(proof,now);
  const auto valid=gate.prepare(now,10.);ASSERT_TRUE(valid&&valid->accepted);
  EXPECT_EQ(timeNs(valid->valid_until),ns+1);
  const auto expired=gate.prepare(SourceTime::fromNanoseconds(ns+1),10.01);
  ASSERT_TRUE(expired);EXPECT_FALSE(expired->accepted);EXPECT_EQ(expired->reason,"validation_not_fresh");
}
TEST(TrackerTimeContract, PreparedDemandAndAdmissionKeepActualClockNanoseconds) {
  ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'),true);
  const auto grant=stageWriterHandoff(gate);ASSERT_TRUE(gate.handoff(grant,100.02,10.02));
  constexpr std::int64_t ns=100040000017LL;
  auto body=odom(100.04);body.source_stamp_ns=body.posterior_stamp_ns=body.imu_stamp_ns=ns;
  ASSERT_TRUE(gate.receiveOdom(body,SourceTime::fromNanoseconds(ns),10.04));
  const auto prepared=gate.preparedStep(SourceTime::fromNanoseconds(ns),10.04);ASSERT_TRUE(prepared);
  EXPECT_EQ(timeNs(prepared->demand.source_stamp),ns);
  EXPECT_EQ(timeNs(prepared->demand.body_source_stamp),ns);
  EXPECT_EQ(timeNs(prepared->demand.valid_until),ns+100000000LL);
  EXPECT_EQ(timeNs(prepared->entry_source_stamp),ns);EXPECT_EQ(timeNs(prepared->measured_pose.header.stamp),ns);
  EXPECT_EQ(timeNs(prepared->entry_admission.checked_at),ns);
  EXPECT_EQ(timeNs(prepared->entry_admission.body_source_stamp),ns);
}
TEST(TrackerWriterCAS, LateAppliedFactMovesIdentityForwardAndHoldsWithoutRollback) {
  ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'),true);
  const auto g=stageWriterHandoff(gate);ASSERT_TRUE(gate.handoff(g,100.02,10.02));
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.04),100.04,10.04));
  const auto x=gate.preparedStep(100.04,10.04);ASSERT_TRUE(x);auto a=preparedWriterAck(g,*x);
  EXPECT_TRUE(gate.step(100.24,10.24).hold);EXPECT_FALSE(gate.preparedStep(100.24,10.24));
  ASSERT_TRUE(gate.commitAck(a,100.24,10.24));EXPECT_EQ(gate.core().trajectoryId(),2);
  EXPECT_TRUE(gate.step(100.26,10.26).hold);EXPECT_EQ(gate.writerCommitSequence(),2U);
  auto old=g.incumbent;old.sequence=8;EXPECT_FALSE(gate.permit(old,100.26,10.26));
}
TEST(TrackerWriterCAS, NegativeFactReleasesSlotButLateAppliedTombstoneCannotResume) {
  ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'),true);
  auto g=stageWriterHandoff(gate);ASSERT_TRUE(gate.handoff(g,100.02,10.02));
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.04),100.04,10.04));const auto x=gate.preparedStep(100.04,10.04);ASSERT_TRUE(x);
  auto a=preparedWriterAck(g,*x);a.applied=a.write_submitted=a.write_acknowledged=false;a.commit_sequence=1;
  ASSERT_TRUE(gate.commitAck(a,100.05,10.05));EXPECT_EQ(gate.core().trajectoryId(),1);
  auto retry=g;retry.handoff_id="retry";retry.sequence=2;ASSERT_TRUE(gate.handoff(retry,100.06,10.06));
  a.applied=true;a.commit_sequence=2;++a.sequence;ASSERT_TRUE(gate.commitAck(a,100.07,10.07));
  EXPECT_EQ(gate.core().trajectoryId(),2);EXPECT_TRUE(gate.step(100.08,10.08).hold);
  EXPECT_FALSE(gate.preparedStep(100.08,10.08));
}
TEST(TrackerWriterCAS, SameReferenceDifferentCurveNegativeFactResolvesOnlyExactUnchangedWriter) {
  ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'),true);
  const auto g=stageWriterHandoff(gate,1);ASSERT_EQ(g.incumbent.version,g.candidate.version);
  ASSERT_TRUE(gate.handoff(g,100.02,10.02));
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.04),100.04,10.04));
  const auto x=gate.preparedStep(100.04,10.04);ASSERT_TRUE(x);
  auto premature=g.candidate;premature.geometry_committed=true;
  EXPECT_FALSE(gate.permit(premature,100.04,10.04));EXPECT_EQ(gate.core().trajectoryId(),1);
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.23),100.23,10.23));
  EXPECT_TRUE(gate.step(100.23,10.23).hold);
  auto negative=preparedWriterAck(g,*x);negative.applied=negative.write_submitted=negative.write_acknowledged=false;
  negative.commit_sequence=1;negative.demand_sequence=negative.motion_validation_sequence=negative.entry_admission_sequence=0;
  // This is the malformed negative in the recorded normal_01 graph. No exact
  // fixed permit identity means writer outcome is still unknown, so HOLD.
  auto malformed=negative;malformed.permit_sequence=0;
  EXPECT_FALSE(gate.commitAck(malformed,100.231,10.231));EXPECT_TRUE(gate.step(100.231,10.231).hold);
  auto advanced=negative;advanced.commit_sequence=2;
  EXPECT_FALSE(gate.commitAck(advanced,100.232,10.232));
  auto uncertain=negative;uncertain.write_submitted=true;
  EXPECT_FALSE(gate.commitAck(uncertain,100.232,10.232));
  // Confirmed no-write fact may arrive after the conditional lease expires.
  // Same reference/anchor is not the same curve: incumbent 1 is still valid.
  ASSERT_TRUE(gate.commitAck(negative,100.233,10.233));
  const auto resumed=gate.step(100.234,10.234);EXPECT_FALSE(resumed.hold);
  EXPECT_EQ(resumed.trajectory_id,1);EXPECT_GT(resumed.velocity.linear.x,0.);
  EXPECT_EQ(gate.writerCommitSequence(),1U);
  auto retry=g;retry.handoff_id="retry";retry.sequence=2;retry.source_stamp=stamp(100.234);
  retry.valid_until=retry.transition_deadline=stamp(100.434);
  retry.candidate.source_stamp=retry.source_stamp;retry.candidate.valid_until=retry.valid_until;
  ASSERT_TRUE(gate.handoff(retry,100.234,10.234));
  auto late=preparedWriterAck(g,*x);late.sequence=3;
  ASSERT_TRUE(gate.commitAck(late,100.235,10.235));EXPECT_EQ(gate.core().trajectoryId(),2);
  EXPECT_TRUE(gate.step(100.236,10.236).hold); // Irreversible late fact, no rollback.
  EXPECT_FALSE(gate.preparedStep(100.236,10.236));
}
TEST(TrackerWriterCAS, WrongAppliedOwnerOrChangedCommandNeverReplacesGeometry) {
  for(int variant=0;variant<7;++variant) {
    ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'),true);
    auto g=stageWriterHandoff(gate);ASSERT_TRUE(gate.handoff(g,100.02,10.02));
    ASSERT_TRUE(gate.core().receiveOdom(odom(100.04),100.04,10.04));const auto x=gate.preparedStep(100.04,10.04);ASSERT_TRUE(x);
    auto a=preparedWriterAck(g,*x);
    if(variant==0)++a.grant_sequence;
    if(variant==1)++a.previous_commit_sequence;
    if(variant==2)a.sdk_session="other";
    if(variant==3)++a.sdk_arm_generation;
    if(variant==4)a.applied_velocity.linear.x+=.001;
    if(variant==5)++a.entry_admission_sequence;
    if(variant==6)a.demand_body_source_stamp=stamp(100.041);
    EXPECT_FALSE(gate.commitAck(a,100.04,10.04))<<variant;EXPECT_EQ(gate.core().trajectoryId(),1);
  }
}
TEST(TrackerWriterCAS, StationaryReentryRequiresOriginalSdkWitnessAndNewCurveProofNotOldProof) {
  for(int variant=0;variant<24;++variant) {
    auto c=executionConfig();
    if(variant==18)c.stationary_linear_threshold_mps=.02;
    if(variant==19)c.stationary_angular_threshold_radps=.04;
    if(variant==20)c.stationary_reentry_duration_s=1.;
    if(variant==21)c.stationary_minimum_samples=5;
    if(variant==22) {
      c.stationary_linear_threshold_mps=.02;c.stationary_angular_threshold_radps=.04;
      c.stationary_reentry_duration_s=1.;c.stationary_minimum_samples=5;
    }
    if(variant==23) {c.stationary_linear_threshold_mps=.05;c.stationary_angular_threshold_radps=.1;}
    ExecutionContract gate(c,"isolated_mock",std::string(64,'f'),true);
    auto g=stageWriterHandoff(gate);g.transition_mode=wire::ExecutionHandoffGrant::STATIONARY_REENTRY;
    auto body=odom(100.04);body.velocity_in_frame.setZero();body.planar_speed=0.;
    ASSERT_TRUE(gate.core().receiveOdom(body,100.04,10.04));
    auto invalid=proofWire();invalid.sequence=3;invalid.valid=false;gate.validation(invalid,100.04);
    auto holding=g.incumbent;holding.sequence=2;holding.allowed=false;holding.phase="holding";
    EXPECT_FALSE(gate.permit(holding,100.04,10.04)); // Invalid old geometry is NOT made valid.
    g.incumbent=holding;g.candidate.sequence=3;
    g.source_stamp=stamp(100.04);g.valid_until=g.transition_deadline=stamp(100.24);
    g.retain_incumbent_until=g.source_stamp;g.candidate.source_stamp=g.source_stamp;g.candidate.valid_until=g.valid_until;
    auto successor=curve(3,100.04);g.candidate.trajectory_id=3;
    successor.generation=2;successor.identity=d1max_trajectory_tracker::identity(g.candidate.version);
    for(std::size_t i=0;i<successor.points.size();++i)successor.points[i]={std::max(0.,double(i)-2.)*.05,0.,.55};
    for(std::size_t i=0;i<successor.knots.size();++i)successor.knots[i]=(double(i)-3.)*.5;
    certify(successor);gate.candidate(successor);
    auto fresh=proofWire(g.candidate.version);fresh.trajectory_id=3;fresh.sequence=4;
    fresh.checked_to_time=fresh.curve_duration=10.;gate.validation(fresh,100.04);g.candidate.validation_sequence=4;
    auto& e=g.stationary_evidence;e.schema_version=1;e.version=holding.version;e.execution_id=holding.execution_id;
    e.control_epoch=holding.control_epoch;e.sdk_session=holding.sdk_session;e.sdk_arm_generation=holding.sdk_arm_generation;
    e.transport_mode="isolated_mock";e.sequence=e.zero_write_sequence=e.writer_commit_sequence=1;
    e.applied_trajectory_id=1;e.zero_ack_at=stamp(99.4);e.mc_raw_stamp_ns=900;e.mc_clock_epoch="clock";
    e.time_basis="isolated_simulated_source_clock";e.source_stamp=stamp(100.04);e.received_stamp=stamp(100.04);
    e.capture_lower_bound=stamp(100.03);e.capture_upper_bound=stamp(100.04);e.valid_until=stamp(100.24);
    e.stationary_samples=3;e.stationary_duration_sec=.64;e.nonzero_blocked=e.usable=true;
    if(variant==1)g.schema_version=1;
    if(variant==2)e.usable=false;
    if(variant==3)e.zero_write_sequence=0;
    if(variant==4)e.capture_lower_bound=e.zero_ack_at;
    if(variant==5)e.sdk_session="foreign";
    if(variant==6)g.incumbent.allowed=true;
    if(variant==7)e.valid_until=stamp(100.03);
    if(variant==8)e.nonzero_blocked=false;
    if(variant==9)g.candidate.valid_until=stamp(100.25);
    if(variant==10)e.measured_linear_mps=.031;
    if(variant==11)e.measured_linear_mps=-.031;
    if(variant==12)e.measured_angular_radps=.051;
    if(variant==13)e.measured_angular_radps=-.051;
    if(variant==14)e.stationary_duration_sec=.599;
    if(variant==15)e.stationary_samples=2;
    if(variant==16)e.measured_linear_mps=NAN;
    if(variant==17)e.measured_angular_radps=NAN;
    if(variant==18)e.measured_linear_mps=.025;
    if(variant==19)e.measured_angular_radps=.045;
    if(variant==22) {
      e.measured_linear_mps=.02;e.measured_angular_radps=.04;
      e.stationary_duration_sec=1.1;e.stationary_samples=5;
    }
    if(variant==23) {e.measured_linear_mps=.049;e.measured_angular_radps=.099;}
    const bool accepted=gate.handoff(g,100.04,10.04);
    EXPECT_EQ(accepted,variant==0||variant==22||variant==23)<<variant;
    if(!accepted)continue;
    EXPECT_TRUE(gate.step(100.04,10.04).hold);EXPECT_EQ(gate.core().trajectoryId(),1);
    const auto prepared=gate.preparedStep(100.06,10.06);ASSERT_TRUE(prepared);
    EXPECT_EQ(prepared->demand.trajectory_id,3);EXPECT_LE(prepared->position_error_m,.0125);
    EXPECT_LE(prepared->velocity_error_mps,.05);
    auto applied=preparedWriterAck(g,*prepared);ASSERT_TRUE(gate.commitAck(applied,100.06,10.06));
    EXPECT_EQ(gate.core().trajectoryId(),3);EXPECT_EQ(gate.writerCommitSequence(),2U);
  }
}

TEST(TrackerExecution, CandidatePreparationDoesNotActivateBeforeMatchingOwnerCommit) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
  const auto ack=gate.prepare(100.,10.);ASSERT_TRUE(ack);ASSERT_TRUE(ack->accepted);
  EXPECT_FALSE(gate.core().active());EXPECT_TRUE(gate.step(100.,10.).hold);
  ASSERT_TRUE(gate.permit(permitWire(),100.,10.));
  const auto out=gate.step(100.05,10.05);EXPECT_GT(out.velocity.linear.x,0.);EXPECT_FALSE(out.safety_checked);
}
TEST(TrackerExecution, NewCandidateCannotBeActivatedWithPredecessorAdmissionAndProof) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
  gate.candidate(curve(2));
  EXPECT_FALSE(gate.permit(permitWire(),100.01,10.01));EXPECT_FALSE(gate.core().active());
}
TEST(TrackerExecution, GeometryOnlyCommitNeverProducesMotionAndCanLaterAuthorize) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
  auto p=permitWire();p.allowed=false;p.phase="preview";ASSERT_TRUE(gate.permit(p,100.,10.));
  EXPECT_TRUE(gate.core().active());EXPECT_DOUBLE_EQ(gate.step(100.02,10.02).velocity.linear.x,0.);
  p.allowed=true;p.phase="tracking";++p.sequence;ASSERT_TRUE(gate.permit(p,100.03,10.03));
  EXPECT_GT(gate.step(100.05,10.05).velocity.linear.x,0.);
}
TEST(TrackerExecution, ArmingHeartbeatDoesNotEraseCommittedGeometryOrNeedOldAdmission) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto p=permitWire();
  p.allowed=false;p.phase="preview";p.control_epoch=0;
  ASSERT_TRUE(gate.permit(p,100.,10.));
  auto arming=p;arming.sequence=2;arming.geometry_committed=false;
  arming.trajectory_id=0;arming.control_epoch=1;arming.phase="arming";
  ASSERT_TRUE(gate.permit(arming,100.1,10.1));
  EXPECT_TRUE(gate.step(100.1,10.1).hold);
  // The first admission is long expired; this is not a second geometry commit.
  for(int i=1;i<=15;++i){const double now=100.1+.02*i;
    ASSERT_TRUE(gate.core().receiveOdom(odom(now),now,now-90.));
    EXPECT_TRUE(gate.step(now,now-90.).hold);
  }
  auto proof=proofWire();proof.sequence=2;
  proof.source_stamp=proof.check_begin=proof.check_end=proof.body_source_stamp=
    proof.front_ray_source_stamp=proof.rear_ray_source_stamp=stamp(100.4);
  proof.valid_until=stamp(100.65);gate.validation(proof,100.4);
  p.sequence=3;p.validation_sequence=2;p.control_epoch=1;p.allowed=true;p.phase="tracking";
  p.source_stamp=stamp(100.4);p.valid_until=stamp(100.6);
  ASSERT_TRUE(gate.permit(p,100.4,10.4));
  const auto out=gate.step(100.42,10.42);
  EXPECT_EQ(out.execution_id,p.execution_id);EXPECT_EQ(out.sdk_arm_generation,1U);
  EXPECT_EQ(out.trajectory_id,1);EXPECT_GT(out.velocity.linear.x,0.);
  auto wrong=p;wrong.sequence=4;wrong.version.anchor_id="other";
  EXPECT_FALSE(gate.permit(wrong,100.43,10.43));
}
TEST(TrackerExecution, ProofGapStopsImmediatelyButDoesNotInventOdometryRecoveryDelay) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto p=permitWire();
  ASSERT_TRUE(gate.permit(p,100.,10.));EXPECT_GT(gate.step(100.02,10.02).velocity.linear.x,0.);
  auto failed=proofWire();failed.sequence=2;failed.valid=false;gate.validation(failed,100.03);
  EXPECT_TRUE(gate.step(100.04,10.04).hold);EXPECT_FALSE(gate.core().holding());
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.06),100.06,10.06));
  auto recovered=proofWire();recovered.sequence=3;gate.validation(recovered,100.06);
  p.sequence=2;p.validation_sequence=3;ASSERT_TRUE(gate.permit(p,100.06,10.06));
  EXPECT_GT(gate.step(100.06,10.06).velocity.linear.x,0.);
  // Real source failure is not cleared by a fresh permission/proof heartbeat.
  auto stale=odom(100.08);stale.imu_stamp=99.;
  EXPECT_FALSE(gate.core().receiveOdom(stale,100.08,10.08));
  EXPECT_TRUE(gate.step(100.08,10.08).hold);EXPECT_TRUE(gate.core().holding());
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.1),100.1,10.1));
  EXPECT_TRUE(gate.step(100.1,10.1).hold);EXPECT_TRUE(gate.core().holding());
}
TEST(TrackerExecution, ThirtySecondStationaryPreviewCanExecuteOnlyWithCurrentExactProof) {
  for(bool current_proof:{true,false}) {
    ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
    ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto p=permitWire();p.allowed=false;p.phase="preview";
    ASSERT_TRUE(gate.permit(p,100.,10.));
    for(int i=1;i<=600;++i) {
      const double now=100.+i*.05,receipt=10.+i*.05;
      ASSERT_TRUE(gate.core().receiveOdom(odom(now),now,receipt));
      auto v=proofWire();v.sequence=i+1;
      v.source_stamp=v.check_begin=v.check_end=v.body_source_stamp=v.front_ray_source_stamp=v.rear_ray_source_stamp=stamp(now);
      v.valid_until=stamp(now+.25);gate.validation(v,now);
      p.sequence=i+1;p.validation_sequence=v.sequence;p.source_stamp=stamp(now);p.valid_until=stamp(now+.2);
      ASSERT_TRUE(gate.permit(p,now,receipt))<<i<<" now="<<now;
      EXPECT_TRUE(gate.step(now,receipt).hold);
      EXPECT_DOUBLE_EQ(gate.core().progressTime(),0.);
    }
    const double now=current_proof?130.02:130.3;
    auto body=odom(now);ASSERT_TRUE(gate.core().receiveOdom(body,now,now-90.));
    ++p.sequence;p.allowed=true;p.phase="tracking";p.source_stamp=stamp(now);p.valid_until=stamp(now+.2);
    EXPECT_EQ(gate.permit(p,now,now-90.),current_proof);
    const auto output=gate.step(now+.02,now-90.+.02);
    if(current_proof)EXPECT_GT(output.velocity.linear.x,0.);else EXPECT_TRUE(output.hold);
  }
}
TEST(TrackerExecution, NativeUnknownExpiredProofWrongOwnerAndMockLeakCannotCommit) {
  for(int variant=0;variant<6;++variant) {
    ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);auto proof=proofWire();
    if(variant==0)proof.valid=false;
    if(variant==1)proof.front_ray_source_stamp=stamp(99.);
    if(variant==2)proof.collision_policy="official_inflated_double_cylinder";
    proof.sequence=2;gate.validation(proof,100.);
    auto ack=gate.prepare(100.,10.);
    if(variant<3) {ASSERT_TRUE(ack);EXPECT_FALSE(ack->accepted);continue;}
    ASSERT_TRUE(ack&&ack->accepted);auto p=permitWire();p.validation_sequence=2;
    if(variant==3)p.version.anchor_id="other";
    if(variant==4)p.transport_mode="live";
    if(variant==5)p.validation_sequence=3;
    EXPECT_FALSE(gate.permit(p,100.,10.));EXPECT_FALSE(gate.core().active());
  }
}
TEST(TrackerExecution, FreshNextProofDoesNotRevokeStillFreshLeasedPredecessor) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
  ASSERT_TRUE(gate.permit(permitWire(),100.,10.));gate.step(100.02,10.02);
  auto next=proofWire();next.sequence=2;gate.validation(next,100.03);
  EXPECT_GT(gate.step(100.04,10.04).velocity.linear.x,0.);
  next.valid=false;next.sequence=3;gate.validation(next,100.05);
  // A witnessed invalidation of this exact curve must stop immediately, even
  // if the owner has not yet delivered the revoked permit.
  EXPECT_TRUE(gate.step(100.06,10.06).hold);
}
TEST(TrackerExecution, TwoHundredMillisecondProofRefreshDoesNotWaitForOwnerHeartbeat) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto permit=permitWire();
  ASSERT_TRUE(gate.permit(permit,100.,10.));
  auto next=proofWire();next.sequence=2;
  next.source_stamp=next.check_begin=next.check_end=next.body_source_stamp=
    next.front_ray_source_stamp=next.rear_ray_source_stamp=stamp(100.2);
  next.valid_until=stamp(100.45);gate.validation(next,100.2);
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.2),100.2,10.2));
  const auto before=gate.step(100.22,10.22);
  ASSERT_FALSE(before.hold);EXPECT_EQ(before.validation_sequence,2U);
  // The original proof expires at 100.25. Authority still lives until 100.5;
  // the newer same-curve native proof already exists, so no zero is invented.
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.26),100.26,10.26));
  const auto after=gate.step(100.26,10.26);
  EXPECT_FALSE(after.hold);EXPECT_EQ(after.permit_sequence,permit.sequence);
  EXPECT_EQ(after.validation_sequence,2U);EXPECT_GT(after.velocity.linear.x,before.velocity.linear.x);
  EXPECT_LE(seconds(after.valid_until),seconds(next.valid_until));
  // Fresh body alone does not renew either native evidence or authority.
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.46),100.46,10.46));
  EXPECT_TRUE(gate.step(100.46,10.46).hold);
}
TEST(TrackerExecution, ProofFloorCannotBorrowForeignGeometryOrCrossLatestInvalidFence) {
  for(int variant=0;variant<6;++variant) {
    ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
    ASSERT_TRUE(gate.prepare(100.,10.)->accepted);ASSERT_TRUE(gate.permit(permitWire(),100.,10.));
    auto next=proofWire();next.sequence=2;
    next.source_stamp=next.check_begin=next.check_end=next.body_source_stamp=
      next.front_ray_source_stamp=next.rear_ray_source_stamp=stamp(100.2);
    next.valid_until=stamp(100.45);
    if(variant==0)next.version.anchor_revision=2;
    if(variant==1)next.version.task_id="other";
    if(variant==2)next.trajectory_id=2;
    if(variant==3)next.sequence=0; // below permit floor
    if(variant==4)next.valid=false;
    gate.validation(next,100.2);
    if(variant==5) {
      auto invalid=next;invalid.sequence=3;invalid.valid=false;gate.validation(invalid,100.21);
      gate.validation(next,100.22); // delayed positive must not mask invalid N+1
    }
    ASSERT_TRUE(gate.core().receiveOdom(odom(100.26),100.26,10.26));
    EXPECT_TRUE(gate.step(100.26,10.26).hold)<<variant;
  }
}
TEST(TrackerExecution, FreshCollisionProofCannotRenewAuthorityOrResumeHoldingPhase) {
  for(bool holding:{false,true}) {
    ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
    ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto p=permitWire();p.valid_until=stamp(100.24);
    ASSERT_TRUE(gate.permit(p,100.,10.));auto next=proofWire();next.sequence=2;
    next.source_stamp=next.check_begin=next.check_end=next.body_source_stamp=
      next.front_ray_source_stamp=next.rear_ray_source_stamp=stamp(100.2);
    next.valid_until=stamp(100.45);gate.validation(next,100.2);
    if(holding){++p.sequence;p.phase="holding";p.source_stamp=stamp(100.2);p.valid_until=stamp(100.45);
      ASSERT_TRUE(gate.permit(p,100.2,10.2));}
    ASSERT_TRUE(gate.core().receiveOdom(odom(100.26),100.26,10.26));
    EXPECT_TRUE(gate.step(100.26,10.26).hold);
  }
}
TEST(TrackerExecution, OwnerCanCommitAdmittedProofWhenNextValidSequenceArrivesFirst) {
  for(bool invalid:{false,true}) {
    ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
    ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
    auto next=proofWire();next.sequence=2;next.valid=!invalid;gate.validation(next,100.01);
    if(!invalid){ASSERT_TRUE(gate.prepare(100.01,10.01)->accepted);}
    EXPECT_EQ(gate.permit(permitWire(),100.02,10.02),!invalid);
    EXPECT_EQ(gate.core().active(),!invalid);
  }
}
TEST(TrackerExecution, MissingSupportAndStairSemanticsNeverBecomeFlatFloorByDefault) {
  for(int variant=0;variant<5;++variant) {
    ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);auto s=supportWire();
    if(variant==0)s.verified=false;
    if(variant==1)s.floor_id="floor2";
    if(variant==2)s.segment_kind="stairs";
    if(variant==3)s.required_mode="stair";
    if(variant==4)s.support_ground_xyz[2].z=2.;
    gate.support(s);ASSERT_TRUE(gate.prepare(100.,10.));EXPECT_FALSE(gate.prepare(100.,10.)->accepted);
  }
}
TEST(TrackerExecution, RemainingProofIsOnlyForCommittedCurveAndRecomputesArcCoverage) {
  for(int failure=0;failure<5;++failure) {
    ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
    auto suffix=proofWire();suffix.whole_curve=false;suffix.remaining_curve=true;
    suffix.checked_from_time=0.;suffix.valid_start_time=0.;suffix.valid_start_arc_length=0.;
    suffix.checked_to_time=suffix.curve_duration=4.;suffix.reverse_margin_m=.15;
    if(failure==0){suffix.sequence=2;gate.validation(suffix,100.);EXPECT_FALSE(gate.prepare(100.,10.)->accepted);continue;}
    ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto p=permitWire();ASSERT_TRUE(gate.permit(p,100.,10.));
    suffix.sequence=2;
    if(failure==2)suffix.checked_to_time=3.;
    if(failure==3)suffix.reverse_margin_m=.1;
    if(failure==4)suffix.valid_start_arc_length=.2;
    gate.validation(suffix,100.01);++p.sequence;p.validation_sequence=2;
    EXPECT_EQ(gate.permit(p,100.02,10.02),failure==1);
  }
}
TEST(TrackerV2, RemainingDomainIncludesMeasuredBacktrackingNotHighWaterCutoff) {
  TrackerCore core(config());prepare(core);
  core.step(100.02,10.02);ASSERT_TRUE(core.receiveOdom(odom(100.04,.3),100.04,10.04));core.step(100.04,10.04);
  ASSERT_TRUE(core.receiveOdom(odom(100.06,.2),100.06,10.06));core.step(100.06,10.06);
  EXPECT_NEAR(core.progressArc(),.2,1e-3);EXPECT_NEAR(core.committedArc(),.3,1e-3);
  EXPECT_TRUE(core.remainingProofCovers(.2,.8,.2,4.,4.,.15));
  EXPECT_FALSE(core.remainingProofCovers(.6,.8,.2,4.,4.,.15));
}
TEST(TrackerV2, LocalEndpointRetainsFreshMeasuredCurveWhileSuccessorIsOnlyPrepared) {
  const auto c=executionConfig();TrackerCore core(c);const auto t=task();
  const auto s=supportEvidence(supportWire());auto current=curve();
  auto body=odom();ASSERT_TRUE(core.receiveOdom(body,100.,10.));
  ASSERT_TRUE(core.admitRevision(t,current,s,100.,10.,true));
  Output prior;bool endpoint=false,stopped=false;double now=100.,receipt=10.;
  for(int i=1;i<=400;++i) {
    now=100.+i*.02;receipt=10.+i*.02;
    body.stamp=body.posterior_stamp=body.imu_stamp=now;
    body.velocity_in_frame={prior.forward,0.,0.};body.planar_speed=prior.forward;
    ASSERT_TRUE(core.receiveTask(t,now,receipt));ASSERT_TRUE(core.receiveOdom(body,now,receipt));
    const auto out=core.step(now,receipt,true);
    if(out.reason=="local_segment_finished_waiting_replan") {
      endpoint=true;EXPECT_TRUE(core.progress(now).valid);EXPECT_EQ(core.trajectoryId(),1);
      EXPECT_GE(out.forward,0.);EXPECT_LE(prior.forward-out.forward,c.max_acceleration*.02+1e-10);
      EXPECT_FALSE(out.finished);
      if(out.forward==0.) {stopped=true;prior=out;break;}
    }
    body.position.x()+=out.forward*.02;prior=out;
  }
  ASSERT_TRUE(endpoint&&stopped);
  // Beyond the native 400 ms progress-age limit, distinct real stationary
  // samples still certify the retained identity. No callback restamping.
  for(int i=1;i<=40;++i) {
    now+=.02;receipt+=.02;body.stamp=body.posterior_stamp=body.imu_stamp=now;
    body.velocity_in_frame.setZero();body.planar_speed=0.;
    ASSERT_TRUE(core.receiveTask(t,now,receipt));ASSERT_TRUE(core.receiveOdom(body,now,receipt));
    const auto out=core.step(now,receipt,true);EXPECT_DOUBLE_EQ(out.forward,0.);
    const auto p=core.progress(now);ASSERT_TRUE(p.valid);EXPECT_DOUBLE_EQ(p.source_stamp,body.stamp);
    EXPECT_EQ(p.identity,t.identity);EXPECT_EQ(core.trajectoryId(),1);
  }
  auto next_task=t;next_task.generation=2;next_task.identity.anchor_revision=2;next_task.identity.anchor_id="next-anchor";
  auto next_support=s;next_support.generation=2;next_support.identity=next_task.identity;
  auto successor=curve(2,now);successor.generation=2;successor.identity=next_task.identity;
  for(std::size_t i=0;i<successor.points.size();++i)
    successor.points[i]={body.position.x()+std::max(0.,double(i)-2.)*.05,0.,.55};
  for(std::size_t i=0;i<successor.knots.size();++i)successor.knots[i]=(double(i)-3.)*.5;
  certify(successor);
  ASSERT_TRUE(core.admitRevision(next_task,successor,next_support,now,receipt,false));
  EXPECT_EQ(core.trajectoryId(),1);EXPECT_TRUE(core.progress(now).valid);
  ASSERT_TRUE(core.admitRevision(next_task,successor,next_support,now,receipt,true));
  EXPECT_EQ(core.trajectoryId(),2);EXPECT_TRUE(core.progress(now).valid);
  // Retaining geometry does not turn stale sensor input into valid evidence.
  EXPECT_FALSE(core.progress(now+c.odom_timeout+.01).valid);
}
TEST(TrackerExecution, OlderFreshHeartbeatCanUseExactProofHistoryButNeverMaskNewInvalid) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto p=permitWire();ASSERT_TRUE(gate.permit(p,100.,10.));
  auto next=proofWire();next.sequence=2;gate.validation(next,100.01);
  ++p.sequence;EXPECT_TRUE(gate.permit(p,100.02,10.02));
  next.sequence=3;next.valid=false;gate.validation(next,100.03);
  ++p.sequence;EXPECT_FALSE(gate.permit(p,100.04,10.04));
  EXPECT_TRUE(gate.step(100.04,10.04).hold);
}
TEST(TrackerExecution, GeometryPreviewProjectsOnlyNewMeasuredPositionWhileDemandStaysZero) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto p=permitWire();p.allowed=false;
  ASSERT_TRUE(gate.permit(p,100.,10.));const double start=gate.core().progressTime();
  EXPECT_TRUE(gate.step(100.02,10.02).hold);EXPECT_DOUBLE_EQ(gate.core().progressTime(),start);
  auto body=odom(100.04);body.position.x()+=.01;
  ASSERT_TRUE(gate.core().receiveOdom(body,100.04,10.04));
  const auto demand=gate.step(100.04,10.04);EXPECT_TRUE(demand.hold);EXPECT_EQ(demand.velocity.linear.x,0.);
  EXPECT_GT(gate.core().progressTime(),start);EXPECT_TRUE(gate.core().progress(100.04).valid);
}
TEST(TrackerV2, RecoverableSourceGapRetainsCurveAndNeedsDistinctSamplesOverPointSixSeconds) {
  TrackerCore core(config());prepare(core);auto stale=odom(100.1);stale.imu_stamp=99.;
  EXPECT_FALSE(core.receiveOdom(stale,100.1,10.1));EXPECT_TRUE(core.active());EXPECT_TRUE(core.holding());
  ASSERT_TRUE(core.receiveOdom(odom(100.2),100.2,10.2));
  EXPECT_FALSE(core.receiveOdom(odom(100.2),100.2,10.3));EXPECT_TRUE(core.holding());
  ASSERT_TRUE(core.receiveOdom(odom(100.4),100.4,10.4));EXPECT_TRUE(core.holding());
  ASSERT_TRUE(core.receiveOdom(odom(100.81),100.81,10.81));EXPECT_FALSE(core.holding());EXPECT_EQ(core.trajectoryId(),1);
}
TEST(TrackerV2, SupportRelativeHeightAllowsVerifiedCorridorButNotCrossFloorSupport) {
  auto c=executionConfig();TrackerCore core(c);auto t=task();t.goal.z()=1.;
  auto support=supportEvidence(supportWire());for(auto& p:support.ground)p.z()=.15*p.x();support.max_slope=.16;
  auto candidate=curve();for(auto& p:candidate.points)p.z()=.55+.15*p.x();
  auto body=odom();body.velocity_in_frame={.25,0.,.0375};ASSERT_TRUE(core.receiveOdom(body,100.,10.));
  certify(candidate);ASSERT_TRUE(core.admitRevision(t,candidate,support,100.,10.,true));
  auto new_t=t;new_t.generation=2;new_t.identity.anchor_revision=2;new_t.identity.anchor_id="anchor2";
  auto next=candidate;next.generation=2;next.identity=new_t.identity;next.id=2;
  support.identity=new_t.identity;support.generation=2;
  EXPECT_TRUE(core.admitRevision(new_t,next,support,100.,10.,false));EXPECT_EQ(core.generation(),1);
  EXPECT_TRUE(core.admitRevision(new_t,next,support,100.,10.,true));EXPECT_EQ(core.generation(),2);
  support.floor_id="floor2";next.id=3;EXPECT_FALSE(core.admitRevision(new_t,next,support,100.,10.,true));
}

TEST(TrackerSpatialBudget, FlatSlopeAndReversedOrRotatedTravelUseTheSameSpatialLimit) {
  for(const double yaw:{0.,.6,1.5707963267948966,3.141592653589793}) {
    for(const double grade:{0.,.00275,-.00275,.15,-.15}) {
      const Eigen::Vector3d tangent(std::cos(yaw),std::sin(yaw),grade);
      const double scale=spatialToPlanarScale(tangent);
      EXPECT_NEAR(scale,1./std::sqrt(1.+grade*grade),1e-14);
      EXPECT_LE((.3*scale*tangent).norm(),.3+1e-15);
      EXPECT_NEAR(spatialToPlanarScale(-tangent),scale,1e-15);
      if(grade!=0.) { EXPECT_LT(scale,1.); }
    }
  }
}

TEST(TrackerSpatialBudget, UndefinedAndVerticalDirectionsCannotBecomeFlatGround) {
  EXPECT_EQ(spatialToPlanarScale(Eigen::Vector3d::Zero()),0.);
  EXPECT_EQ(spatialToPlanarScale(Eigen::Vector3d(0.,0.,.1)),0.);
  EXPECT_EQ(spatialToPlanarScale(Eigen::Vector3d(0.,0.,-.1)),0.);
  EXPECT_EQ(spatialToPlanarScale(Eigen::Vector3d(1.,0.,NAN)),0.);
  EXPECT_EQ(spatialToPlanarScale(Eigen::Vector3d(INFINITY,0.,0.)),0.);
}

TEST(TrackerSpatialBudget, ProductionCurveControllerPreservesTrueVerticalVelocityAndAcceleration) {
  for(const double yaw:{0.,1.5707963267948966,-1.5707963267948966}) {
    for(const double grade:{.00275,-.00275,.10}) {
      auto c=executionConfig();TrackerCore core(c);
      const Eigen::Vector2d direction(std::cos(yaw),std::sin(yaw));
      auto t=task();t.goal={2.*direction.x(),2.*direction.y(),.55+2.*grade};
      auto support=supportEvidence(supportWire());
      for(auto& p:support.ground) { const double x=p.x();p={x*direction.x(),x*direction.y(),x*grade}; }
      auto candidate=curve();
      for(auto& p:candidate.points) { const double x=p.x();p={x*direction.x(),x*direction.y(),.55+x*grade}; }
      certify(candidate);
      auto body=odom();body.yaw=yaw;
      body.orientation=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ());
      candidate.join_orientation=body.orientation;
      body.velocity_in_frame={.25*direction.x(),.25*direction.y(),.25*grade};
      ASSERT_TRUE(core.receiveOdom(body,100.,10.));
      ASSERT_TRUE(core.admitRevision(t,candidate,support,100.,10.,true));
      Output previous;
      for(int i=1;i<=25;++i) {
        const double dt=.05,now=100.+i*dt,receipt=10.+i*dt;
        body.stamp=body.posterior_stamp=body.imu_stamp=now;
        ASSERT_TRUE(core.receiveTask(t,now,receipt));
        ASSERT_TRUE(core.receiveOdom(body,now,receipt));
        const auto out=core.step(now,receipt,true);
        EXPECT_EQ(out.reason,"tracking");
        const Eigen::Vector3d actual(out.forward*direction.x(),out.forward*direction.y(),out.forward*grade);
        EXPECT_LE(actual.norm(),c.max_speed+1e-12);
        const double spatial_acceleration=std::abs(out.forward-previous.forward)*std::sqrt(1.+grade*grade)/dt;
        EXPECT_LE(spatial_acceleration,c.max_acceleration+1e-10);
        // These are still the original sensor measurements, not a fabricated
        // planarized twist used to make the native dynamic checks pass.
        EXPECT_DOUBLE_EQ(core.odometry().velocity_in_frame.z(),.25*grade);
        previous=out;
      }
      EXPECT_GT(previous.forward,.29);
      EXPECT_LE(previous.forward,c.max_speed*std::cos(support.max_slope));
    }
  }
}

TEST(TrackerSpatialBudget, UncommittedCandidateCannotChangeTheIncumbentBudgetOrLimiter) {
  TrackerCore core(config());prepare(core);
  Output before;
  for(int i=1;i<=25;++i) {
    const double now=100.+i*.05,receipt=10.+i*.05;
    ASSERT_TRUE(core.receiveTask(task(),now,receipt));
    ASSERT_TRUE(core.receiveOdom(odom(now),now,receipt));before=core.step(now,receipt,true);
  }
  ASSERT_DOUBLE_EQ(before.forward,.3);
  auto candidate=curve(2,101.25);
  for(auto& p:candidate.points)p.z()+=.1*p.x();
  certify(candidate);
  EXPECT_FALSE(core.receiveTrajectory(candidate,101.25,11.25));
  EXPECT_EQ(core.trajectoryId(),1);
  const auto after=core.step(101.30,11.30,true);
  EXPECT_DOUBLE_EQ(after.forward,before.forward);
}

TEST(TrackerExecution, ControlDeadlineDoesNotFreezeThePriorCollisionSnapshotsDeadline) {
  for(double phase:{.01,.02,.03,.04}) {
    ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
    auto evidence=proofWire();evidence.sequence=2;evidence.valid_until=stamp(100.+phase+.001);
    gate.validation(evidence,100.);ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
    auto permit=permitWire();permit.validation_sequence=2;
    ASSERT_TRUE(gate.permit(permit,100.,10.));
    ASSERT_TRUE(gate.core().receiveOdom(odom(100.+phase),100.+phase,10.+phase));
    const auto demand=gate.step(100.+phase,10.+phase);
    EXPECT_FALSE(demand.hold);EXPECT_GT(demand.velocity.linear.x,0.);
    EXPECT_EQ(demand.validation_sequence,2u);
    EXPECT_NEAR(seconds(demand.source_stamp),100.+phase,1e-8);
    EXPECT_NEAR(seconds(demand.valid_until),100.+phase+.1,1e-8);
    // This is not a lease extension on the original proof. Without genuinely
    // new native evidence the very next controller tick still emits zero.
    ASSERT_TRUE(gate.core().receiveOdom(odom(100.+phase+.02),100.+phase+.02,10.+phase+.02));
    const auto stale=gate.step(100.+phase+.02,10.+phase+.02);
    EXPECT_TRUE(stale.hold);EXPECT_EQ(stale.reason,"permission_or_native_proof_expired");
    EXPECT_TRUE(gate.core().active());
  }
}

TEST(TrackerExecution, IndependentControlDeadlineNeverExtendsOriginalOwnerLease) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
  auto permit=permitWire();permit.valid_until=stamp(100.04);
  ASSERT_TRUE(gate.permit(permit,100.,10.));
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.02),100.02,10.02));
  const auto demand=gate.step(100.02,10.02);
  EXPECT_FALSE(demand.hold);EXPECT_EQ(demand.valid_until,permit.valid_until);
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.06),100.06,10.06));
  EXPECT_TRUE(gate.step(100.06,10.06).hold);
}

namespace {
wire::MotionDemand ownTurningDemand(ExecutionContract& gate) {
  stage(gate);const auto prepared=gate.prepare(100.,10.);
  EXPECT_TRUE(prepared&&prepared->accepted);EXPECT_TRUE(gate.permit(permitWire(),100.,10.));
  auto body=odom(100.02);body.yaw=-.1;
  body.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(gate.core().receiveOdom(body,100.02,10.02));
  auto demand=gate.step(100.02,10.02);
  EXPECT_GT(demand.velocity.linear.x,0.);EXPECT_GT(std::abs(demand.velocity.angular.z),1e-9);
  return demand;
}
void refreshTurnProofAndOwner(ExecutionContract& gate,double now,double receipt,std::uint64_t sequence) {
  // A 0.6 s measured rest window outlives one 200 ms collision/owner lease.
  // Supply independent fresh exact-curve evidence rather than extending the
  // original proof or permitting the turn with an expired certificate.
  auto proof=proofWire();proof.sequence=sequence;
  proof.source_stamp=proof.check_begin=proof.check_end=proof.body_source_stamp=
    proof.front_ray_source_stamp=proof.rear_ray_source_stamp=stamp(now);
  proof.valid_until=stamp(now+.2);gate.validation(proof,now);
  auto permit=permitWire();permit.sequence=sequence;permit.validation_sequence=sequence;
  permit.source_stamp=stamp(now);permit.valid_until=stamp(now+.2);
  EXPECT_TRUE(gate.permit(permit,now,receipt))<<gate.lastPermitReject()<<" at "<<now
    <<" active="<<gate.core().active()<<" reason="<<gate.core().reason()
    <<" trajectory="<<gate.core().trajectoryId();
}
wire::MotionValidation occupiedFor(const wire::MotionDemand& d) {
  wire::MotionValidation v;v.version=d.version;v.execution_id=d.execution_id;v.control_epoch=d.control_epoch;
  v.sdk_session=d.sdk_session;v.sdk_arm_generation=d.sdk_arm_generation;v.trajectory_id=d.trajectory_id;
  v.permit_sequence=d.permit_sequence;v.trajectory_validation_sequence=d.validation_sequence;
  v.demand_sequence=d.sequence;v.demand_source_stamp=d.source_stamp;
  v.demand_valid_until=d.valid_until;v.demand_body_source_stamp=d.body_source_stamp;
  v.body_source_stamp=stamp(100.02);v.front_ray_source_stamp=v.rear_ray_source_stamp=stamp(100.);
  v.sequence=1;v.map_snapshot_revision=1;v.check_begin=stamp(100.03);v.check_end=stamp(100.031);
  v.valid_until=d.valid_until;v.velocity=d.velocity;v.frame_id="d1max_loc_odom";
  v.reason="motion_sweep_occupied";v.transport_mode="isolated_mock";v.braking_model_sha256=std::string(64,'f');
  return v;
}
}

TEST(TrackerExecution, OnlyOwnExactFreshBlockedForwardTurnCanRequestDifferentManeuver) {
  ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'));
  const auto demand=ownTurningDemand(gate);auto feedback=occupiedFor(demand);
  EXPECT_STREQ(gate.core().maneuverPhase(),"following");
  ASSERT_TRUE(gate.motionValidation(feedback,100.04));
  EXPECT_STREQ(gate.core().maneuverPhase(),"decelerating_for_turn");
  EXPECT_FALSE(gate.motionValidation(feedback,100.04));
  // Feedback never bypasses the normal proof/permission gate or transforms
  // the rejected command into an automatically approved rotation.
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.30),100.30,10.30));
  const auto no_evidence=gate.step(100.30,10.30);
  EXPECT_TRUE(no_evidence.hold);EXPECT_FALSE(no_evidence.safety_checked);
}

TEST(TrackerExecution, UnknownStaleForeignOrMutatedSweepCannotRequestTurn) {
  for(int variant=0;variant<23;++variant) {
    SCOPED_TRACE(variant);
    ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'));
    const auto demand=ownTurningDemand(gate);auto feedback=occupiedFor(demand);double now=100.04;
    switch(variant) {
      case 0:feedback.reason="motion_sweep_unknown_or_expired";break;
      case 1:feedback.reason="motion_evidence_expired_during_check";break;
      case 2:feedback.reason="motion_sweep_outside_map";break;
      case 3:feedback.valid=true;break;
      case 4:feedback.version.task_id="foreign";break;
      case 5:feedback.version.anchor_revision++;break;
      case 6:feedback.trajectory_id++;break;
      case 7:feedback.demand_sequence++;break;
      case 8:feedback.permit_sequence++;break;
      case 9:feedback.execution_id="other";break;
      case 10:feedback.control_epoch++;break;
      case 11:feedback.sdk_arm_generation++;break;
      case 12:feedback.braking_model_sha256=std::string(64,'a');break;
      case 13:feedback.velocity.angular.z+=.01;break;
      case 14:feedback.demand_source_stamp.nanosec++;break;
      case 15:feedback.demand_body_source_stamp.nanosec++;break;
      case 16:feedback.demand_valid_until.nanosec++;break;
      case 17:feedback.front_ray_source_stamp=stamp(99.);break;
      case 18:feedback.body_source_stamp=stamp(99.);break;
      case 19:feedback.transport_mode="live";break;
      case 20:feedback.trajectory_validation_sequence++;break;
      case 21:feedback.valid_until=stamp(100.035);break;
      case 22:now=100.13;break;
    }
    EXPECT_FALSE(gate.motionValidation(feedback,now));
    EXPECT_STREQ(gate.core().maneuverPhase(),"following");
  }
}

TEST(TrackerExecution, NeverGeneratedOrRevokedDemandCannotRequestTurn) {
  ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'));
  auto demand=ownTurningDemand(gate);auto feedback=occupiedFor(demand);
  gate.cancel("test_cancel");EXPECT_FALSE(gate.motionValidation(feedback,100.04));
  ExecutionContract disabled(executionConfig(),"isolated_mock");
  demand=ownTurningDemand(disabled);feedback=occupiedFor(demand);
  EXPECT_FALSE(disabled.motionValidation(feedback,100.04));
}

TEST(TrackerExecution, BlockedEntryReportsExactOriginalSweepOnceWithoutRestampingOrAuthority) {
  ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'));
  const auto first=ownTurningDemand(gate);auto feedback=occupiedFor(first);
  ASSERT_TRUE(gate.motionValidation(feedback,100.04));
  EXPECT_FALSE(gate.takeBlockedEntry());
  int sample=3;
  for(;sample<=80;++sample) {
    const double now=100.+sample*.02;auto body=odom(now);body.planar_speed=0.;body.velocity_in_frame.setZero();
    body.yaw=-.05;body.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ()));
    ASSERT_TRUE(gate.core().receiveOdom(body,now,10.+sample*.02));
    refreshTurnProofAndOwner(gate,now,10.+sample*.02,100+sample);
    gate.step(now,10.+sample*.02);
    if(std::string(gate.core().maneuverPhase())=="following")break;
  }
  ASSERT_LT(sample,80);++sample;const double now=100.+sample*.02;
  auto body=odom(now);body.planar_speed=0.;body.velocity_in_frame.setZero();body.yaw=-.05;
  body.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ()));
  ASSERT_TRUE(gate.core().receiveOdom(body,now,10.+sample*.02));
  refreshTurnProofAndOwner(gate,now,10.+sample*.02,100+sample);
  const auto second=gate.step(now,10.+sample*.02);
  ASSERT_GT(second.velocity.linear.x,0.);ASSERT_GT(std::abs(second.velocity.angular.z),0.);
  feedback=occupiedFor(second);feedback.sequence=2;
  feedback.body_source_stamp=body.stamp>0.?stamp(body.stamp):stamp(0.);
  feedback.front_ray_source_stamp=feedback.rear_ray_source_stamp=stamp(now);
  feedback.check_begin=stamp(now+.001);feedback.check_end=stamp(now+.002);
  ASSERT_TRUE(gate.motionValidation(feedback,now+.003));
  ASSERT_STREQ(gate.core().maneuverPhase(),"waiting_executable_entry");
  const auto blocked=gate.takeBlockedEntry();ASSERT_TRUE(blocked);
  EXPECT_EQ(*blocked,feedback);EXPECT_FALSE(gate.takeBlockedEntry());
  EXPECT_FALSE(gate.motionValidation(feedback,now+.004));
  // A received candidate is not a commit and cannot unhold the old entry.
  gate.candidate(curve(2,now));
  EXPECT_STREQ(gate.core().maneuverPhase(),"waiting_executable_entry");
  const auto waiting=gate.step(now+.02,10.+sample*.02+.02);
  EXPECT_TRUE(waiting.hold);EXPECT_FALSE(waiting.safety_checked);
  EXPECT_EQ(waiting.reason,"waiting_executable_entry");EXPECT_TRUE(gate.core().active());
}
TEST(TrackerExecution, OccupiedInPlaceAlignTurnReportsBlockedEntryInsteadOfRepeatingIt) {
  for(const bool occupied:{false,true}) {
    SCOPED_TRACE(occupied);
    ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'));
    const auto first=ownTurningDemand(gate);auto feedback=occupiedFor(first);
    ASSERT_TRUE(gate.motionValidation(feedback,100.04));
    ASSERT_STREQ(gate.core().maneuverPhase(),"decelerating_for_turn");
    wire::MotionDemand rotation;auto body=odom();double now=0.;int sample=3;
    for(;sample<=40;++sample) {
      now=100.+sample*.02;body=odom(now);body.planar_speed=0.;body.velocity_in_frame.setZero();
      body.yaw=-.4;body.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(body.yaw,Eigen::Vector3d::UnitZ()));
      ASSERT_TRUE(gate.core().receiveOdom(body,now,10.+sample*.02));
      refreshTurnProofAndOwner(gate,now,10.+sample*.02,100+sample);
      rotation=gate.step(now,10.+sample*.02);
      if(std::string(gate.core().maneuverPhase())=="aligning_to_curve"&&!rotation.hold&&
         std::abs(rotation.velocity.angular.z)>1e-9)break;
    }
    ASSERT_LE(sample,40);
    EXPECT_DOUBLE_EQ(rotation.velocity.linear.x,0.);
    feedback=occupiedFor(rotation);feedback.sequence=2;feedback.body_source_stamp=stamp(body.stamp);
    feedback.front_ray_source_stamp=feedback.rear_ray_source_stamp=stamp(now);
    feedback.check_begin=stamp(now+.001);feedback.check_end=stamp(now+.002);
    if(!occupied) {
      // Unknown space is never a collision verdict and never a new maneuver.
      feedback.reason="motion_sweep_unknown_or_expired";
      EXPECT_FALSE(gate.motionValidation(feedback,now+.003));
      EXPECT_STREQ(gate.core().maneuverPhase(),"aligning_to_curve");
      EXPECT_FALSE(gate.takeBlockedEntry());
      continue;
    }
    ASSERT_TRUE(gate.motionValidation(feedback,now+.003));
    ASSERT_STREQ(gate.core().maneuverPhase(),"waiting_executable_entry");
    const auto blocked=gate.takeBlockedEntry();ASSERT_TRUE(blocked);
    EXPECT_EQ(*blocked,feedback);EXPECT_FALSE(gate.takeBlockedEntry());
    EXPECT_FALSE(gate.motionValidation(feedback,now+.004));
    const auto waiting=gate.step(now+.02,10.+sample*.02+.02);
    EXPECT_TRUE(waiting.hold);EXPECT_DOUBLE_EQ(waiting.velocity.angular.z,0.);
    EXPECT_EQ(waiting.reason,"waiting_executable_entry");EXPECT_TRUE(gate.core().active());
  }
}
TEST(TrackerPreparationWorker, LatestPendingIsBoundedAndOldCompletionCannotWin) {
  std::mutex mutex;std::condition_variable entered,release;bool first_entered=false,leave_first=false;
  std::vector<int> built;
  BoundedPreparationWorker<int,int> worker([&](const int& value,const auto&) {
    std::unique_lock<std::mutex> lock(mutex);built.push_back(value);
    if(value==1) {first_entered=true;entered.notify_one();release.wait_for(lock,std::chrono::seconds(1),[&]{return leave_first;});}
    return value;
  });
  worker.submit(1);
  {std::unique_lock<std::mutex> lock(mutex);ASSERT_TRUE(entered.wait_for(lock,std::chrono::seconds(1),[&]{return first_entered;}));}
  worker.submit(2);const auto latest=worker.submit(3);
  {std::lock_guard<std::mutex> lock(mutex);leave_first=true;}release.notify_one();
  std::optional<BoundedPreparationWorker<int,int>::Completion> out;
  for(int i=0;i<1000&&!out;++i) {out=worker.take();if(!out)std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  ASSERT_TRUE(out&&out->result);EXPECT_EQ(out->token,latest);EXPECT_EQ(*out->result,3);
  {std::lock_guard<std::mutex> lock(mutex);EXPECT_EQ(built,(std::vector<int>{1,3}));}
}

TEST(TrackerPreparationWorker, CancelNeverQueuesBehindBuildAndRetiredResultCannotReturn) {
  std::atomic<bool> entered{false},exited{false};
  BoundedPreparationWorker<int,int> worker([&](const int& value,const auto& allowed) {
    entered=true;while(allowed())std::this_thread::yield();exited=true;return value;
  });
  worker.submit(1);
  for(int i=0;i<1000&&!entered;++i)std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(entered);worker.invalidate();EXPECT_FALSE(worker.take());
  for(int i=0;i<1000&&!exited;++i)std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_TRUE(exited);EXPECT_FALSE(worker.take());
}

TEST(TrackerPreparationWorker, ExceptionsBecomeFailedCompletionNotSharedControllerMutation) {
  BoundedPreparationWorker<int,int> worker([](const int&,const auto&)->int {throw std::runtime_error("bad geometry");});
  worker.submit(1);std::optional<BoundedPreparationWorker<int,int>::Completion> out;
  for(int i=0;i<1000&&!out;++i){out=worker.take();if(!out)std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  ASSERT_TRUE(out);EXPECT_FALSE(out->result);EXPECT_EQ(out->failure,"bad geometry");
}

namespace {
bool waitPreparation(ExecutionContract& gate) {
  for(int i=0;i<1000;++i) {
    gate.pollPreparation();if(!gate.preparationPending())return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}
}

TEST(TrackerPreparationWorker, ProductionGeometryPreparesOffOwnerButOnlyExactPermitCommits) {
  ExecutionContract async(executionConfig(),"isolated_mock",{},false,true);stage(async);
  EXPECT_FALSE(async.core().active());ASSERT_TRUE(waitPreparation(async));
  const auto ack=async.prepare(100.,10.);ASSERT_TRUE(ack&&ack->accepted);
  EXPECT_FALSE(async.core().active());ASSERT_TRUE(async.permit(permitWire(),100.,10.));
  ExecutionContract sync(executionConfig(),"isolated_mock");stage(sync);
  ASSERT_TRUE(sync.prepare(100.,10.)->accepted);ASSERT_TRUE(sync.permit(permitWire(),100.,10.));
  const auto a=async.step(100.02,10.02),b=sync.step(100.02,10.02);
  EXPECT_EQ(a.hold,b.hold);EXPECT_DOUBLE_EQ(a.velocity.linear.x,b.velocity.linear.x);
  EXPECT_DOUBLE_EQ(a.velocity.angular.z,b.velocity.angular.z);
  EXPECT_EQ(async.core().trajectoryId(),1);
}

TEST(TrackerPreparationWorker, CancelAndDuplicateCannotInstallRetiredCandidateOrMutateCachedCurve) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},false,true);stage(gate);
  gate.cancel("owner_cancelled");EXPECT_FALSE(gate.pollPreparation());
  gate.candidate(curve()); // late delivery for the retired task
  EXPECT_FALSE(gate.prepare(100.,10.));EXPECT_FALSE(gate.core().active());
  ExecutionContract immutable(executionConfig(),"isolated_mock",{},false,true);stage(immutable);
  auto same=curve();for(auto& p:same.points)p.x()+=10.;immutable.candidate(same);
  ASSERT_TRUE(waitPreparation(immutable));ASSERT_TRUE(immutable.prepare(100.,10.)->accepted);
  ASSERT_TRUE(immutable.permit(permitWire(),100.,10.));
  EXPECT_NEAR(immutable.core().progress(100.).position.x(),0.,.01);
}

TEST(TrackerPreparationWorker, CachedGeometryCannotBeReusedForMutatedShapeOrRelaxedConfiguration) {
  auto t=curve();t.prepared=PreparedGeometry::build(config(),t,{});
  TrackerCore core(config());prepare(core);t.id=2;
  EXPECT_FALSE(core.receiveTrajectory(t,100.,10.));EXPECT_EQ(core.candidateReason(),"prepared_geometry_identity_mismatch");
  t=curve(2);t.prepared=PreparedGeometry::build(config(),t,{});t.points[4].y()+=.2;
  EXPECT_FALSE(core.receiveTrajectory(t,100.,10.));EXPECT_EQ(core.trajectoryId(),1);
  t=curve(2);auto relaxed=config();relaxed.max_yaw_rate=.4;t.prepared=PreparedGeometry::build(relaxed,t,{});
  EXPECT_FALSE(core.receiveTrajectory(t,100.,10.));EXPECT_EQ(core.trajectoryId(),1);
}

TEST(TrackerPreparationWorker, CachedSupportCannotAuthorizeDifferentEvidenceWithReusedHash) {
  const auto c=executionConfig();TrackerCore core(c);
  ASSERT_TRUE(core.receiveOdom(odom(),100.,10.));auto t=curve();auto support=supportEvidence(supportWire());
  t.prepared=PreparedGeometry::build(c,t,support);
  auto wrong=support;wrong.ground[2].z()=2.;
  EXPECT_FALSE(core.admitRevision(task(),t,wrong,100.,10.,true));
  EXPECT_EQ(core.candidateReason(),"prepared_support_geometry_mismatch");EXPECT_FALSE(core.active());
  ASSERT_TRUE(core.admitRevision(task(),t,support,100.,10.,true));EXPECT_EQ(core.trajectoryId(),1);
}

TEST(TrackerPreparationWorker, RejectedCandidatePreservesIncumbentAndWorkerNeverPredictsActualEntry) {
  for(bool bad_shape:{false,true}) {
    ExecutionContract gate(executionConfig(),"isolated_mock",{},false,true);stage(gate);
    ASSERT_TRUE(waitPreparation(gate));ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
    ASSERT_TRUE(gate.permit(permitWire(),100.,10.));const auto before=gate.step(100.02,10.02);
    auto next=curve(2);if(bad_shape)for(auto& p:next.points)p.z()+=1.;
    gate.candidate(next);ASSERT_TRUE(waitPreparation(gate));
    auto proof=proofWire();proof.trajectory_id=2;proof.sequence=2;gate.validation(proof,100.04);
    // Geometry was prepared without measured state. The owner must detect
    // that actual entry is no longer reachable, not trust a worker prediction.
    auto body=odom(100.04,.15);ASSERT_TRUE(gate.receiveOdom(body,100.04,10.04));
    const auto rejected=gate.prepare(100.04,10.04);ASSERT_TRUE(rejected);EXPECT_FALSE(rejected->accepted);
    EXPECT_EQ(gate.core().trajectoryId(),1);EXPECT_TRUE(gate.core().active());
    const auto after=gate.step(100.04,10.04);EXPECT_FALSE(after.hold);EXPECT_GT(after.velocity.linear.x,0.);
    EXPECT_LE(after.velocity.linear.x-before.velocity.linear.x,executionConfig().max_acceleration*.02+1e-9);
  }
}

TEST(TrackerPreparationWorker, FreshContextResetRetiresPreparedCandidateAndPriorPermit) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},false,true);stage(gate);
  ASSERT_TRUE(waitPreparation(gate));ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
  ASSERT_TRUE(gate.permit(permitWire(),100.,10.));gate.candidate(curve(2));
  auto reset=odom(99.);reset.localization_epoch=2;reset.localization_seed_id="reset";
  EXPECT_FALSE(gate.receiveOdom(reset,100.04,10.04));EXPECT_FALSE(gate.core().active());
  EXPECT_FALSE(gate.pollPreparation());EXPECT_FALSE(gate.prepare(100.04,10.04));
  auto old=permitWire();old.sequence=2;EXPECT_FALSE(gate.permit(old,100.04,10.04));
  EXPECT_TRUE(gate.step(100.04,10.04).hold);
}

TEST(TrackerExecution, NewerProposalCannotRedatePreparedCurveTask) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
  // A successor proposal stamped after the prepared curve's start time.
  auto next=proposalWire(version(2));next.proposal_id="proposal2";
  next.source_stamp=stamp(100.05);next.valid_until=stamp(100.9);gate.proposal(next,100.05);
  ASSERT_TRUE(gate.permit(permitWire(),100.06,10.06));EXPECT_TRUE(gate.core().active());
  EXPECT_EQ(gate.core().trajectoryId(),1);
}
TEST(TrackerExecution, FailedCommitStillFollowsOwnerLeaseForSuccessorProposal) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);ASSERT_TRUE(gate.permit(permitWire(),100.,10.));
  // Owner moves to curve 2 but this controller never admitted that exact proof.
  auto owner=permitWire();owner.sequence=2;owner.trajectory_id=2;owner.validation_sequence=2;
  EXPECT_FALSE(gate.permit(owner,100.02,10.02));EXPECT_EQ(gate.lastPermitReject(),"commit_admission_missing");
  EXPECT_EQ(gate.core().trajectoryId(),1);
  // SCAN and the owner build the successor on curve 2; it must reach preparation.
  auto next=proposalWire(version(2));next.proposal_id="proposal2";
  next.expected_version=version();next.expected_trajectory_id=2;gate.proposal(next,100.03);
  auto c=curve(3);c.generation=2;gate.candidate(c);gate.support(supportWire(version(2)));
  auto v=proofWire(version(2));v.proposal_id="proposal2";v.trajectory_id=3;gate.validation(v,100.04);
  const auto ack=gate.prepare(100.04,10.04);ASSERT_TRUE(ack);EXPECT_TRUE(ack->accepted)<<ack->reason;
  // A proposal built on a curve the owner never leased is still ignored.
  auto foreign=proposalWire(version(3));foreign.proposal_id="proposal3";
  foreign.expected_version=version();foreign.expected_trajectory_id=1;gate.proposal(foreign,100.05);
  auto f=curve(4);f.generation=3;gate.candidate(f);gate.support(supportWire(version(3)));
  auto fv=proofWire(version(3));fv.proposal_id="proposal3";fv.trajectory_id=4;gate.validation(fv,100.05);
  EXPECT_FALSE(gate.prepare(100.05,10.05));
}

namespace {
wire::MotionDemand beginDuplicateTickTest(ExecutionContract& gate) {
  stage(gate);EXPECT_TRUE(gate.prepare(100.,10.)->accepted);
  EXPECT_TRUE(gate.permit(permitWire(),100.,10.));
  const auto d=gate.step(SourceTime::fromNanoseconds(100020000019LL),10.02);
  EXPECT_FALSE(d.hold);return d;
}
wire::MotionValidation finiteSweepFor(const wire::MotionDemand& d,std::int64_t until) {
  auto v=occupiedFor(d);v.valid=true;v.reason="motion_sweep_observed_free";
  v.check_begin=v.check_end=d.source_stamp;v.valid_until=stampNs(until);return v;
}
}
TEST(TrackerDuplicateTick, Skips20msTimersWithoutNewDemandOrLimiterResetAndResumesContinuously) {
  ExecutionContract gate(executionConfig(),"isolated_mock"),control(executionConfig(),"isolated_mock");
  const auto first=beginDuplicateTickTest(gate);beginDuplicateTickTest(control);
  const auto now=sourceTime(first.source_stamp);
  for(double receipt:{10.04,10.06,10.08,10.10}) {
    EXPECT_FALSE(gate.controlTickRequired(now,receipt));
    EXPECT_EQ(gate.core().lastControlSourceNs(),timeNs(first.source_stamp));
  }
  const auto next=SourceTime::fromNanoseconds(timeNs(first.source_stamp)+20000000LL);
  ASSERT_TRUE(gate.controlTickRequired(next,10.10));
  const auto resumed=gate.step(next,10.10),expected=control.step(next,10.10);
  EXPECT_EQ(resumed.sequence,first.sequence+1);EXPECT_FALSE(resumed.hold);
  EXPECT_GT(resumed.velocity.linear.x,first.velocity.linear.x);
  EXPECT_DOUBLE_EQ(resumed.velocity.linear.x,expected.velocity.linear.x);
  EXPECT_DOUBLE_EQ(resumed.velocity.angular.z,expected.velocity.angular.z);
  EXPECT_EQ(timeNs(first.valid_until)-timeNs(first.source_stamp),100000000LL);
}
TEST(TrackerDuplicateTick, OriginalDemandExpiresBySteadyReceiptAndSameSourceCannotRenewIt) {
  ExecutionContract gate(executionConfig(),"isolated_mock");const auto first=beginDuplicateTickTest(gate);
  const auto now=sourceTime(first.source_stamp);
  EXPECT_FALSE(gate.controlTickRequired(now,10.119));EXPECT_TRUE(gate.controlTickRequired(now,10.121));
  const auto stopped=gate.step(now,10.121);
  EXPECT_TRUE(stopped.hold);EXPECT_EQ(stopped.reason,"source_clock_paused_command_expired");
  auto newer=proofWire();newer.sequence=2;gate.validation(newer,now);
  EXPECT_TRUE(gate.controlTickRequired(now,10.122));
  EXPECT_EQ(gate.step(now,10.122).reason,"source_clock_paused_command_expired");
}
TEST(TrackerDuplicateTick, OriginalTwoMicrosecondCurveProofIsNotRenewedByLongerNewProof) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
  auto proof=proofWire();proof.sequence=2;proof.valid_until=stampNs(100020002019LL);
  gate.validation(proof,100.);ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
  auto permit=permitWire();permit.validation_sequence=2;ASSERT_TRUE(gate.permit(permit,100.,10.));
  const auto now=SourceTime::fromNanoseconds(100020000019LL);const auto d=gate.step(now,10.02);
  ASSERT_FALSE(d.hold);EXPECT_FALSE(gate.controlTickRequired(now,10.020001));
  auto longer=proofWire();longer.sequence=3;gate.validation(longer,now);
  EXPECT_TRUE(gate.controlTickRequired(now,10.020003));
  EXPECT_EQ(gate.step(now,10.020003).reason,"source_clock_paused_command_expired");
}
TEST(TrackerDuplicateTick, NewMatchingShorterProofAndPermitPermanentlyShortenOriginalLease) {
  for(bool shorten_permit:{false,true}) {
    SCOPED_TRACE(shorten_permit);ExecutionContract gate(executionConfig(),"isolated_mock");
    const auto first=beginDuplicateTickTest(gate);const auto now=sourceTime(first.source_stamp);
    if(shorten_permit) {
      auto shorter=permitWire();shorter.sequence=2;shorter.valid_until=stampNs(timeNs(first.source_stamp)+2000);
      ASSERT_TRUE(gate.permit(shorter,now,10.0200001));
      auto longer=permitWire();longer.sequence=3;ASSERT_TRUE(gate.permit(longer,now,10.0200002));
    } else {
      auto shorter=proofWire();shorter.sequence=2;shorter.valid_until=stampNs(timeNs(first.source_stamp)+2000);
      gate.validation(shorter,now);auto longer=proofWire();longer.sequence=3;gate.validation(longer,now);
    }
    EXPECT_FALSE(gate.controlTickRequired(now,10.020001));
    EXPECT_TRUE(gate.controlTickRequired(now,10.020003));
    EXPECT_EQ(gate.step(now,10.020003).reason,"source_clock_paused_command_expired");
  }
}
TEST(TrackerDuplicateTick, ExactOwnMotionProofOnlyShortensAndNeverRefreshesEmissionReceipt) {
  ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'));
  const auto first=beginDuplicateTickTest(gate);const auto now=sourceTime(first.source_stamp);
  auto foreign=finiteSweepFor(first,timeNs(first.source_stamp)+1);foreign.velocity.linear.x+=.001;
  EXPECT_FALSE(gate.motionValidation(foreign,now));EXPECT_FALSE(gate.controlTickRequired(now,10.04));
  auto own=finiteSweepFor(first,timeNs(first.source_stamp)+2000);
  EXPECT_FALSE(gate.motionValidation(own,now)); // observation is never permission
  EXPECT_FALSE(gate.controlTickRequired(now,10.020001));
  auto longer=finiteSweepFor(first,timeNs(first.valid_until));longer.sequence=2;
  EXPECT_FALSE(gate.motionValidation(longer,now));
  EXPECT_TRUE(gate.controlTickRequired(now,10.020003));
  EXPECT_EQ(gate.step(now,10.020003).reason,"source_clock_paused_command_expired");
}
TEST(TrackerDuplicateTick, RevocationInvalidProofHoldingAndNewBodyFaultCannotBeSkipped) {
  for(int fault=0;fault<5;++fault) {
    SCOPED_TRACE(fault);ExecutionContract gate(executionConfig(),"isolated_mock");
    const auto first=beginDuplicateTickTest(gate);const auto now=sourceTime(first.source_stamp);
    if(fault==0) {auto p=permitWire();p.sequence=2;p.revoked=true;ASSERT_TRUE(gate.permit(p,now,10.03));}
    if(fault==1) {auto p=proofWire();p.sequence=2;p.valid=false;gate.validation(p,now);}
    if(fault==2)gate.core().hold("source_context_fault",10.03);
    if(fault>=3) {auto o=odom(100.02,fault==3?1.:2.);o.source_stamp_ns=timeNs(first.source_stamp)+1;
      ASSERT_TRUE(gate.receiveOdom(o,now,10.03));}
    EXPECT_TRUE(gate.controlTickRequired(now,10.04));EXPECT_TRUE(gate.step(now,10.04).hold);
  }
}
TEST(TrackerDuplicateTick, ChangedAuthorityAndControlPhaseRequireOrdinaryControlPath) {
  for(int field=0;field<6;++field) {
    SCOPED_TRACE(field);ExecutionContract gate(executionConfig(),"isolated_mock");
    const auto first=beginDuplicateTickTest(gate);const auto now=sourceTime(first.source_stamp);
    auto p=permitWire();p.sequence=2;
    if(field==0)p.execution_id="new_exec";if(field==1)++p.control_epoch;
    if(field==2)p.sdk_session="new_sdk";if(field==3)++p.sdk_arm_generation;
    if(field==4)p.goal_yaw_tolerance_rad=.2;
    if(field==5) {auto proof=proofWire();proof.sequence=2;proof.goal_yaw_checked=true;
      proof.checked_goal_yaw=.2;gate.validation(proof,now);p.phase="aligning";p.has_goal_yaw=true;p.goal_yaw=.2;}
    ASSERT_TRUE(gate.permit(p,now,10.03));EXPECT_TRUE(gate.controlTickRequired(now,10.04));
  }
}
TEST(TrackerDuplicateTick, AppliedAckSameNowUsesOriginalPreparedCommandDeadline) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true);
  const auto g=stageWriterHandoff(gate);ASSERT_TRUE(gate.handoff(g,100.02,10.02));
  EXPECT_FALSE(gate.controlTickRequired(100.02,10.03)); // no prepared demand on duplicate either
  ASSERT_TRUE(gate.receiveOdom(odom(100.04),100.04,10.04));
  const auto prepared=gate.preparedStep(100.04,10.04);ASSERT_TRUE(prepared);ASSERT_FALSE(prepared->demand.hold);
  gate.step(100.04,10.04);auto ack=preparedWriterAck(g,*prepared);
  ack.valid_until=stampNs(100070000002LL);ASSERT_TRUE(gate.commitAck(ack,100.06,10.06));
  EXPECT_EQ(gate.core().lastControlSourceNs(),100060000000LL);
  EXPECT_FALSE(gate.controlTickRequired(100.06,10.069));
  EXPECT_TRUE(gate.controlTickRequired(100.06,10.070000003));
  EXPECT_EQ(gate.step(100.06,10.070000003).reason,"source_clock_paused_command_expired");
}
TEST(TrackerDuplicateTick, OneNanosecondRegressionAtUnixScaleStopsEvenWhenDoubleCannotDistinguishIt) {
  constexpr std::int64_t base=1791124691902856036LL;
  const auto now=SourceTime::fromNanoseconds(base);TrackerCore core(config());
  auto t=task();t.issued_at=now;t.issued_at_ns=base;
  auto o=odom(now);o.source_stamp_ns=o.posterior_stamp_ns=o.imu_stamp_ns=base;
  auto c=curve(1,now);c.start_time_ns=c.join_source_stamp_ns=base;
  ASSERT_TRUE(core.receiveTask(t,now,10.));ASSERT_TRUE(core.receiveOdom(o,now,10.));
  ASSERT_TRUE(core.receiveTrajectory(c,now,10.));
  const auto tick=SourceTime::fromNanoseconds(base+20000000LL);
  ASSERT_GT(core.step(tick,10.02).forward,0.);
  EXPECT_TRUE(core.duplicateControlStateSafe(tick,10.04));
  const auto regression=SourceTime::fromNanoseconds(tick.nanoseconds()-1);
  EXPECT_DOUBLE_EQ(double(regression),double(tick));
  EXPECT_FALSE(core.duplicateControlStateSafe(regression,10.04));
  const auto stopped=core.step(regression,10.04);
  EXPECT_EQ(stopped.forward,0.);EXPECT_EQ(stopped.reason,"clock_or_executor_discontinuity");EXPECT_TRUE(core.holding());
}
