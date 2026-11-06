#include <gtest/gtest.h>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <scan_planner_msgs/msg/bspline.hpp>
#include "d1max_trajectory_tracker/tracker_core.hpp"
#include "d1max_trajectory_tracker/execution_contract.hpp"
using namespace d1max_trajectory_tracker;

namespace d1max_trajectory_tracker {
struct BoundedPreparationWorkerTestAccess {
  template<class Request,class Result>
  static bool completionReady(BoundedPreparationWorker<Request,Result>& worker) {
    std::lock_guard<std::mutex> lock(worker.mutex_);return worker.completed_.has_value();
  }
};
}

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
void prepare(TrackerCore& core,Odom initial=odom()) {
  ASSERT_TRUE(core.receiveTask(task(),100.,10.));
  ASSERT_TRUE(core.receiveOdom(initial,100.,10.));
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
TEST(TrackerLocalProvider, UsableStateIsNotReclassifiedByConsumerImuAge) {
  const auto c=config();auto body=odom();body.source_stamp_ns=100000000000LL;
  body.imu_stamp=99.9054;body.posterior_stamp=99.7943;body.extrapolation_sec=.0946;
  // Exact observed shape: 25.9 ms delivery + 94.6 ms coast = 120.5 ms IMU age.
  EXPECT_STREQ(sourceEvidenceFailure(body,100.0259,c),"body_imu_not_fresh");
  body.provider_contract=body.provider_usable=true;
  body.provider_valid_until_ns=100100000000LL;
  body.provider_posterior_stamp_ns=99794300000LL;body.provider_imu_stamp_ns=99905400000LL;
  EXPECT_EQ(sourceEvidenceFailure(body,100.0259,c),nullptr);
  EXPECT_EQ(body.source_stamp_ns,100000000000LL);
  EXPECT_EQ(body.provider_valid_until_ns,100100000000LL);
  EXPECT_STREQ(sourceEvidenceFailure(body,100.1,c),"local_state_provider_expired");
}
TEST(TrackerLocalProvider, OriginalExpiryAndMalformedOrUnusableEvidenceFailClosed) {
  auto good=odom();good.source_stamp_ns=100000000000LL;
  good.provider_contract=good.provider_usable=true;good.provider_valid_until_ns=100080000017LL;
  good.provider_posterior_stamp_ns=good.provider_imu_stamp_ns=good.source_stamp_ns;
  const auto c=config();EXPECT_EQ(sourceEvidenceFailure(good,100.04,c),nullptr);
  for(int variant=0;variant<8;++variant) {
    auto bad=good;const char* why=nullptr;
    if(variant==0){bad.provider_usable=false;why="local_state_provider_unusable";}
    if(variant==1){bad.source_stamp_ns=0;why="local_state_source_invalid";}
    if(variant==2){bad.provider_valid_until_ns=bad.source_stamp_ns;why="local_state_provider_deadline_invalid";}
    if(variant==3){bad.provider_valid_until_ns=100100000001LL;why="local_state_provider_deadline_invalid";}
    if(variant==4){bad.imu_stamp=100.01;why="local_state_provider_provenance_invalid";}
    if(variant==5){bad.extrapolation_sec=std::numeric_limits<double>::quiet_NaN();why="local_state_provider_provenance_invalid";}
    if(variant==6){bad.posterior_stamp=100.01;why="local_state_provider_provenance_invalid";}
    if(variant==7){bad.imu_stamp=99.9;why="local_state_provider_provenance_invalid";}
    EXPECT_STREQ(sourceEvidenceFailure(bad,100.04,c),why)<<variant;
  }
  EXPECT_STREQ(sourceEvidenceFailure(good,99.97,c),"local_state_source_in_future");
  EXPECT_STREQ(sourceEvidenceFailure(good,100.080001,c),"local_state_provider_expired");
}
TEST(TrackerLocalProvider, RawEstimatorEvidenceMustStillBeValidAtOriginalSource) {
  auto good=odom();good.source_stamp_ns=100000000000LL;
  good.provider_contract=good.provider_usable=true;good.provider_valid_until_ns=100100000000LL;
  good.provider_posterior_stamp_ns=good.provider_imu_stamp_ns=good.source_stamp_ns;
  for(int variant=0;variant<3;++variant) {
    auto bad=good;
    if(variant==0){bad.posterior_stamp=99.599999999;bad.provider_posterior_stamp_ns=99599999999LL;}
    if(variant==1){bad.imu_stamp=99.899999999;bad.provider_imu_stamp_ns=99899999999LL;bad.extrapolation_sec=.100000001;}
    if(variant==2){bad.imu_stamp=99.9;bad.provider_imu_stamp_ns=99900000000LL;bad.extrapolation_sec=.100000001;}
    EXPECT_STREQ(sourceEvidenceFailure(bad,100.0259,config()),"local_state_provider_source_evidence_invalid")<<variant;
  }
}
TEST(TrackerLocalProvider, DuplicateCannotExtendLeaseAndExpiredStateKeepsInstalledGeometry) {
  TrackerCore core(config());auto initial=odom();initial.imu_stamp=99.92;
  initial.posterior_stamp=99.81;initial.extrapolation_sec=.08;prepare(core,initial);
  auto body=odom(100.02,.005);
  body.source_stamp_ns=100020000017LL;body.provider_contract=body.provider_usable=true;
  body.provider_valid_until_ns=100120000017LL;
  body.imu_stamp=99.9254;body.posterior_stamp=99.8143;body.extrapolation_sec=.0946;
  body.provider_posterior_stamp_ns=99814300000LL;body.provider_imu_stamp_ns=99925400000LL;
  ASSERT_TRUE(core.receiveOdom(body,100.0459,10.0459));core.refreshTaskLease(10.0459);
  EXPECT_GT(core.step(100.0459,10.0459).forward,0.);
  auto replay=body;replay.provider_valid_until_ns=100400000000LL;
  EXPECT_FALSE(core.receiveOdom(replay,100.07,10.07));
  EXPECT_EQ(core.odometry().provider_valid_until_ns,100120000017LL);
  core.refreshTaskLease(10.13);EXPECT_EQ(core.step(100.13,10.13).forward,0.);
  EXPECT_TRUE(core.holding());EXPECT_TRUE(core.hasInstalledGeometry());EXPECT_EQ(core.trajectoryId(),1);
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
void stage(ExecutionContract& gate,Trajectory candidate=curve(),Odom initial=odom(),
    const std::string& mode="isolated_mock") {
  ASSERT_TRUE(gate.core().receiveOdom(initial,100.,10.));auto p=proposalWire();p.transport_mode=mode;gate.proposal(p,100.);
  gate.candidate(std::move(candidate));gate.support(supportWire());auto proof=proofWire();proof.transport_mode=mode;
  gate.validation(proof,100.);
}
wire::ExecutionCommitAck initialWriterAck(const wire::ExecutionPermit& p) {
  wire::ExecutionCommitAck a;a.schema_version=1;a.sequence=1;a.applied=a.write_submitted=true;
  a.commit_sequence=1;a.candidate_version=p.version;a.candidate_trajectory_id=p.trajectory_id;
  a.execution_id=p.execution_id;a.control_epoch=p.control_epoch;a.sdk_session=p.sdk_session;
  a.sdk_arm_generation=p.sdk_arm_generation;a.permit_sequence=p.sequence;a.transport_mode=p.transport_mode;
  a.applied_at=stamp(100.02);a.valid_until=p.valid_until;return a;
}
wire::ExecutionHandoffGrant stageWriterHandoff(ExecutionContract& gate,std::uint64_t generation=2,
    const std::string& mode="isolated_mock") {
  stage(gate,curve(),odom(),mode);EXPECT_TRUE(gate.prepare(100.,10.)->accepted);auto incumbent=permitWire();
  incumbent.transport_mode=mode;
  EXPECT_TRUE(gate.permit(incumbent,100.,10.));gate.step(100.02,10.02);
  EXPECT_TRUE(gate.commitAck(initialWriterAck(incumbent),100.02,10.02));
  const auto v=version(generation);auto p=proposalWire(v);p.transport_mode=mode;p.expected_version=incumbent.version;p.expected_trajectory_id=1;
  gate.proposal(p,100.02);auto t=curve(2);t.generation=generation;t.identity=d1max_trajectory_tracker::identity(v);gate.candidate(t);
  gate.support(supportWire(v));auto proof=proofWire(v);proof.transport_mode=mode;proof.trajectory_id=2;proof.sequence=2;
  gate.validation(proof,100.02);EXPECT_TRUE(gate.prepare(100.02,10.02)->accepted);
  wire::ExecutionHandoffGrant g;g.schema_version=2;g.handoff_id="handoff";g.sequence=1;
  g.expected_commit_sequence=1;g.incumbent=incumbent;g.candidate=permitWire(v);g.candidate.transport_mode=mode;
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
TEST(TrackerLocalProvider, StateConsumptionLeaseDoesNotReplaceOriginalCommandPipelineOrInstallationFacts) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},false,false,.2);
  auto initial=odom();initial.imu_stamp=99.92;initial.posterior_stamp=99.81;initial.extrapolation_sec=.08;
  stage(gate,curve(),initial);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);ASSERT_TRUE(gate.permit(permitWire(),100.,10.));
  const auto installed=gate.geometryReceipt();ASSERT_TRUE(installed&&installed->installed);
  auto body=odom(100.02,.005);body.source_stamp_ns=100020000017LL;
  body.provider_contract=body.provider_usable=true;body.provider_valid_until_ns=100120000017LL;
  body.imu_stamp=99.9254;body.posterior_stamp=99.8143;body.extrapolation_sec=.0946;
  body.provider_posterior_stamp_ns=99814300000LL;body.provider_imu_stamp_ns=99925400000LL;
  ASSERT_TRUE(gate.receiveOdom(body,100.0459,10.0459));
  const auto demand=gate.step(100.0459,10.0459);
  EXPECT_FALSE(demand.hold);EXPECT_GT(demand.velocity.linear.x,0.);
  EXPECT_EQ(timeNs(demand.valid_until),timeNs(demand.source_stamp)+200000000LL);
  EXPECT_EQ(timeNs(demand.body_source_stamp),body.source_stamp_ns);
  EXPECT_EQ(gate.core().odometry().provider_valid_until_ns,100120000017LL);
  const auto still_installed=gate.geometryReceipt();ASSERT_TRUE(still_installed&&still_installed->installed);
  EXPECT_EQ(still_installed->installation_sequence,installed->installation_sequence);
  EXPECT_EQ(still_installed->installed_at,installed->installed_at);
  EXPECT_EQ(still_installed->validation_sequence,installed->validation_sequence);
  const auto expired=gate.step(100.13,10.13);EXPECT_TRUE(expired.hold);
  EXPECT_EQ(timeNs(expired.body_source_stamp),body.source_stamp_ns);
  EXPECT_EQ(gate.core().odometry().provider_valid_until_ns,100120000017LL);
  EXPECT_TRUE(gate.core().hasInstalledGeometry());EXPECT_EQ(gate.core().trajectoryId(),1);
}
TEST(TrackerLocalProvider, PreparedCommandUsesFixedTransactionBudgetButExpiredStateCannotProduceNewEntry) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true,false,.2);const auto grant=stageWriterHandoff(gate);
  auto body=odom(100.04,.01);body.source_stamp_ns=100040000019LL;
  body.provider_contract=body.provider_usable=true;body.provider_valid_until_ns=100140000019LL;
  body.posterior_stamp=body.imu_stamp=100.04;
  body.provider_posterior_stamp_ns=body.provider_imu_stamp_ns=body.source_stamp_ns;
  ASSERT_TRUE(gate.receiveOdom(body,100.04,10.04));ASSERT_TRUE(gate.handoff(grant,100.04,10.04));
  const auto prepared=gate.preparedStep(100.0659,10.0659);ASSERT_TRUE(prepared);
  EXPECT_EQ(prepared->demand.valid_until,grant.transition_deadline);
  EXPECT_EQ(prepared->entry_admission.valid_until,prepared->demand.valid_until);
  EXPECT_EQ(timeNs(prepared->entry_source_stamp),body.source_stamp_ns);
  EXPECT_EQ(timeNs(prepared->demand.body_source_stamp),body.source_stamp_ns);
  EXPECT_EQ(grant.transition_deadline,stamp(100.22)); // no signed grant mutation
  EXPECT_EQ(gate.core().odometry().provider_valid_until_ns,100140000019LL);
  EXPECT_FALSE(gate.preparedStep(100.15,10.15));
  EXPECT_EQ(gate.core().trajectoryId(),1);EXPECT_EQ(gate.writerCommitSequence(),1u);
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
  const auto admission=gate.prepare(100.55,10.55);ASSERT_TRUE(admission&&admission->accepted);
  EXPECT_EQ(admission->validation_sequence,2u);EXPECT_EQ(timeNs(admission->body_source_stamp),body.source_stamp_ns);
  auto p=permitWire();p.validation_sequence=2;p.source_stamp=stamp(100.55);p.valid_until=stamp(100.75);
  ASSERT_TRUE(gate.permit(p,100.55,10.55));
  const auto& diagnostic=gate.core().joinDiagnostic();
  EXPECT_TRUE(diagnostic.entry_reobserved);EXPECT_EQ(diagnostic.original_join_source_stamp_ns,100000000017LL);
  EXPECT_EQ(diagnostic.join_source_stamp_ns,body.source_stamp_ns);
  EXPECT_NEAR(gate.core().progressTime(),.5,.005);EXPECT_NEAR(gate.core().progressArc(),.125,.001);
  EXPECT_DOUBLE_EQ(original.join_source_stamp,100.);EXPECT_EQ(original.join_source_stamp_ns,100000000017LL);
  EXPECT_EQ(proof.valid_until,stamp(100.8)); // no original certificate lease was renewed
}
TEST(TrackerEntryReobservation, LiveTypedAdmissionDoesNotUseOldSolveClockAsTaskOwnership) {
  ExecutionContract gate(executionConfig(),"isolated_mock");
  auto original=curve(1,99.);original.join_source_stamp_ns=99000000017LL;
  stage(gate,original);
  const auto proof=proofWire();
  const auto admission=gate.prepare(100.,10.);
  ASSERT_TRUE(admission&&admission->accepted) << gate.lastPrepareReason();
  ASSERT_TRUE(gate.permit(permitWire(),100.,10.)) << gate.lastPermitReject();
  EXPECT_TRUE(gate.core().joinDiagnostic().entry_reobserved);
  EXPECT_EQ(gate.core().joinDiagnostic().original_join_source_stamp_ns,99000000017LL);
  EXPECT_DOUBLE_EQ(original.start_time,99.); // immutable solve source remains old
  EXPECT_EQ(proof.valid_until,stamp(100.25));
  const auto demand=gate.step(100.02,10.02);
  EXPECT_FALSE(demand.hold);EXPECT_GT(demand.velocity.linear.x,0.);
  EXPECT_FALSE(demand.safety_checked); // the original exact motion still needs SCAN's sweep
}
TEST(TrackerEntryReobservation, NewCurveIdentityAndActualEntryOrderRecertifiedReplacement) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
  ASSERT_TRUE(gate.permit(permitWire(),100.,10.));
  gate.step(100.02,10.02);
  ASSERT_TRUE(gate.receiveOdom(odom(100.04,.01),100.04,10.04));
  auto original=curve(2,99.9);original.join_source_stamp_ns=99900000017LL;
  gate.candidate(original);
  auto proof=proofWire();proof.sequence=2;proof.trajectory_id=2;
  proof.source_stamp=proof.check_begin=proof.check_end=proof.body_source_stamp=
    proof.front_ray_source_stamp=proof.rear_ray_source_stamp=stamp(100.04);
  proof.valid_until=stamp(100.29);proof.valid_start_time=.04;proof.valid_start_arc_length=.01;
  gate.validation(proof,100.04);
  const auto admission=gate.prepare(100.04,10.04);
  ASSERT_TRUE(admission&&admission->accepted) << gate.lastPrepareReason();
  auto replacement=permitWire();replacement.sequence=2;replacement.trajectory_id=2;
  replacement.validation_sequence=2;replacement.source_stamp=stamp(100.04);replacement.valid_until=stamp(100.24);
  ASSERT_TRUE(gate.permit(replacement,100.04,10.04)) << gate.lastPermitReject();
  EXPECT_EQ(gate.core().trajectoryId(),2);
  EXPECT_NEAR(gate.core().progressTime(),.04,.005);
  EXPECT_EQ(gate.core().joinDiagnostic().original_join_source_stamp_ns,99900000017LL);
  EXPECT_DOUBLE_EQ(original.start_time,99.9);
  const auto demand=gate.step(100.06,10.06);
  EXPECT_FALSE(demand.hold);EXPECT_EQ(demand.trajectory_id,2);
}
TEST(TrackerEntryReobservation, OldSolveClockCannotOverrideTypedProofIdentityOrFutureClock) {
  for(int variant=0;variant<4;++variant) {
    ExecutionContract gate(executionConfig(),"isolated_mock");auto original=curve(1,99.);
    if(variant==2)original.identity.task_id="retired-task";
    if(variant==3)original.start_time=101.;
    stage(gate,original);
    double now=100.;
    if(variant==0) {auto invalid=proofWire();invalid.sequence=2;invalid.valid=false;gate.validation(invalid,now);}
    if(variant==1) {now=100.3;ASSERT_TRUE(gate.receiveOdom(odom(now),now,10.3));}
    const auto admission=gate.prepare(now,now-90.);
    EXPECT_FALSE(admission&&admission->accepted) << variant;
    if(variant==0||variant==1) {ASSERT_TRUE(admission);EXPECT_EQ(admission->reason,"validation_not_fresh");}
    if(variant==3) {ASSERT_TRUE(admission);EXPECT_EQ(admission->reason,"candidate:trajectory_start_in_future");}
    EXPECT_FALSE(gate.permit(permitWire(),now,now-90.)) << variant;
    EXPECT_FALSE(gate.core().hasInstalledGeometry());EXPECT_TRUE(gate.step(now+.02,now-89.98).hold);
  }
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
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true);
  auto body=odom(base);body.source_stamp_ns=ns;ASSERT_TRUE(gate.core().receiveOdom(body,base,10.));
  auto proposal=proposalWire();proposal.source_stamp=stampNs(ns);proposal.valid_until=stampNs(ns+900000000LL);
  gate.proposal(proposal,base);auto t=curve(1,base);t.join_source_stamp_ns=ns;gate.candidate(t);
  auto s=supportWire();s.source_stamp=stampNs(ns);gate.support(s);
  auto v=proofWire();v.source_stamp=v.check_begin=v.check_end=v.body_source_stamp=
    v.front_ray_source_stamp=v.rear_ray_source_stamp=stampNs(ns);v.valid_until=stampNs(ns+250000000LL);gate.validation(v,base);
  const auto a=gate.prepare(base,10.);ASSERT_TRUE(a&&a->accepted);
  auto p=permitWire();p.source_stamp=stampNs(ns);p.valid_until=stampNs(ns+500000000LL);
  ASSERT_TRUE(gate.permit(p,base+.001,10.001));const auto installed=gate.geometryReceipt();ASSERT_TRUE(installed&&installed->installed);
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
    auto measured=odom(seconds(original));measured.source_stamp_ns=original_ns;
    ASSERT_TRUE(gate.core().receiveOdom(measured,seconds(original),10.));
    const auto demand=gate.step(seconds(original)+.02,10.02);
    EXPECT_EQ(timeNs(demand.body_source_stamp),original_ns);
    EXPECT_EQ(timeNs(demand.valid_until)-timeNs(demand.source_stamp),100000000LL);
  }
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
    // Refreshing an incumbent owner's zero-authority lease is independent
    // from renewing its native collision proof. Admission is not motion.
    ASSERT_TRUE(gate.permit(holding,100.04,10.04));
    const auto held=gate.step(100.04,10.04);
    EXPECT_TRUE(held.hold);EXPECT_FALSE(held.safety_checked);
    EXPECT_DOUBLE_EQ(held.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(held.velocity.angular.z,0.);
    EXPECT_EQ(gate.core().trajectoryId(),1);
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
    e.mc_capture_delay_bound_sec=.01;
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

namespace {
wire::ExecutionHandoffGrant stageStationaryReentry(ExecutionContract& gate,const std::string& mode) {
  auto g=stageWriterHandoff(gate,2,mode);g.transition_mode=wire::ExecutionHandoffGrant::STATIONARY_REENTRY;
  auto body=odom(100.04);body.velocity_in_frame.setZero();body.planar_speed=0.;
  EXPECT_TRUE(gate.core().receiveOdom(body,100.04,10.04));
  auto invalid=proofWire();invalid.transport_mode=mode;invalid.sequence=3;invalid.valid=false;gate.validation(invalid,100.04);
  auto holding=g.incumbent;holding.sequence=2;holding.allowed=false;holding.phase="holding";
  EXPECT_TRUE(gate.permit(holding,100.04,10.04));
  g.incumbent=holding;g.candidate.sequence=3;g.source_stamp=stamp(100.04);
  g.valid_until=g.transition_deadline=stamp(100.24);g.retain_incumbent_until=g.source_stamp;
  g.candidate.source_stamp=g.source_stamp;g.candidate.valid_until=g.valid_until;g.candidate.trajectory_id=3;
  auto successor=curve(3,100.04);successor.generation=2;
  successor.identity=d1max_trajectory_tracker::identity(g.candidate.version);
  for(std::size_t i=0;i<successor.points.size();++i)successor.points[i]={std::max(0.,double(i)-2.)*.05,0.,.55};
  for(std::size_t i=0;i<successor.knots.size();++i)successor.knots[i]=(double(i)-3.)*.5;
  certify(successor);gate.candidate(successor);
  auto proof=proofWire(g.candidate.version);proof.transport_mode=mode;proof.trajectory_id=3;proof.sequence=4;
  proof.checked_to_time=proof.curve_duration=10.;gate.validation(proof,100.04);g.candidate.validation_sequence=4;
  auto& e=g.stationary_evidence;e.schema_version=1;e.version=holding.version;e.execution_id=holding.execution_id;
  e.control_epoch=holding.control_epoch;e.sdk_session=holding.sdk_session;e.sdk_arm_generation=holding.sdk_arm_generation;
  e.transport_mode=mode;e.sequence=e.zero_write_sequence=e.writer_commit_sequence=1;e.applied_trajectory_id=1;
  e.zero_ack_at=stamp(99.4);e.mc_raw_stamp_ns=900;e.mc_clock_epoch="clock";
  e.time_basis=mode=="live"?"source_delta_host_anchor_approximate":"isolated_simulated_source_clock";
  e.source_stamp=stamp(mode=="live"?100.10:100.04);e.received_stamp=stamp(100.04);
  e.mc_capture_delay_bound_sec=.1;e.capture_lower_bound=stamp(99.94);e.capture_upper_bound=e.received_stamp;
  e.valid_until=stamp(100.24);e.stationary_samples=3;e.stationary_duration_sec=.64;e.nonzero_blocked=e.usable=true;
  return g;
}
}
TEST(TrackerStationaryTime, LiveBoundedMappedLeadCanPrepareButCannotCommitWithoutWriterFact) {
  ExecutionContract gate(executionConfig(),"live",std::string(64,'f'),true);
  const auto g=stageStationaryReentry(gate,"live");const auto original=g.stationary_evidence;
  ASSERT_EQ(timeNs(original.source_stamp)-timeNs(original.received_stamp),60000000LL);
  ASSERT_TRUE(gate.handoff(g,100.04,10.04));
  EXPECT_TRUE(gate.step(100.04,10.04).hold);EXPECT_EQ(gate.core().trajectoryId(),1);
  const auto prepared=gate.preparedStep(100.06,10.06);ASSERT_TRUE(prepared);
  EXPECT_EQ(prepared->demand.trajectory_id,3);EXPECT_EQ(g.stationary_evidence,original);
  EXPECT_EQ(gate.writerCommitSequence(),1U);EXPECT_LE(timeNs(prepared->demand.valid_until),timeNs(original.valid_until));
  const auto a=preparedWriterAck(g,*prepared);ASSERT_TRUE(gate.commitAck(a,100.06,10.06));
  EXPECT_EQ(gate.core().trajectoryId(),3);EXPECT_EQ(gate.writerCommitSequence(),2U);
}
TEST(TrackerStationaryTime, ConservativeAnchorAcceptsMeasuredBoundAndPastMappingWithoutRenewal) {
  for(const auto mapped_ns:{100140000000LL,100030000000LL,100040000000LL}) {
    ExecutionContract gate(executionConfig(),"live",std::string(64,'f'),true);
    auto g=stageStationaryReentry(gate,"live");g.stationary_evidence.source_stamp=stampNs(mapped_ns);
    const auto original=g.stationary_evidence;
    ASSERT_TRUE(gate.handoff(g,100.04,10.04))<<mapped_ns;
    const auto prepared=gate.preparedStep(100.06,10.06);ASSERT_TRUE(prepared);
    EXPECT_EQ(g.stationary_evidence,original);
    EXPECT_LE(timeNs(prepared->demand.valid_until),timeNs(original.valid_until));
    EXPECT_EQ(gate.core().trajectoryId(),1);EXPECT_EQ(gate.writerCommitSequence(),1U);
  }
}
TEST(TrackerStationaryTime, BoundReceiptCaptureAndIdentityFailuresCannotEnterReentry) {
  for(int variant=0;variant<25;++variant) {
    SCOPED_TRACE(variant);ExecutionContract gate(executionConfig(),"live",std::string(64,'f'),true);
    auto g=stageStationaryReentry(gate,"live");auto& e=g.stationary_evidence;
    if(variant==0)e.source_stamp=stamp(100.140000001); // measured .1 s bound plus one ns
    if(variant==1)e.mc_capture_delay_bound_sec=.250000001;
    if(variant==2)e.mc_capture_delay_bound_sec=0.;
    if(variant==3)e.mc_capture_delay_bound_sec=NAN;
    if(variant==4)e.received_stamp=stamp(99.7);
    if(variant==5)e.received_stamp=stamp(100.040000001);
    if(variant==6)e.capture_lower_bound=e.zero_ack_at;
    if(variant==7)e.capture_lower_bound=stamp(100.040000001);
    if(variant==8)e.capture_upper_bound=stamp(100.039999999);
    if(variant==9)e.capture_upper_bound=stamp(100.040000001);
    if(variant==10)e.time_basis="unverified_clock";
    if(variant==11)e.time_basis="isolated_simulated_source_clock";
    if(variant==12)e.valid_until=stamp(100.290000001); // may not lease from future mapped source
    if(variant==13){g.source_stamp=stamp(100.03);g.retain_incumbent_until=g.source_stamp;}
    if(variant==14)e.version.task_id="old_task";
    if(variant==15)e.sdk_session="old_sdk";
    if(variant==16)++e.sdk_arm_generation;
    if(variant==17)++e.writer_commit_sequence;
    if(variant==18)++e.applied_trajectory_id;
    if(variant==19)e.transport_mode="isolated_mock";
    if(variant==20){e.source_stamp=stamp(99.7);e.capture_lower_bound=stamp(99.6);}
    if(variant==21)e.source_stamp.nanosec=1000000000u;
    if(variant==22)e.mc_capture_delay_bound_sec=-.1;
    if(variant==23)e.zero_ack_at=e.capture_lower_bound;
    if(variant==24)e.received_stamp.sec=-1;
    EXPECT_FALSE(gate.handoff(g,100.04,10.04));EXPECT_FALSE(gate.preparedStep(100.04,10.04));
    EXPECT_EQ(gate.core().trajectoryId(),1);EXPECT_EQ(gate.writerCommitSequence(),1U);
  }
}
TEST(TrackerStationaryTime, ExactMockClockNeverBorrowsLiveFutureAllowance) {
  for(int variant=0;variant<3;++variant) {
    ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'),true);
    auto g=stageStationaryReentry(gate,"isolated_mock");
    if(variant==1)g.stationary_evidence.source_stamp=stamp(100.040000001);
    if(variant==2)g.stationary_evidence.time_basis="source_delta_host_anchor_approximate";
    EXPECT_EQ(gate.handoff(g,100.04,10.04),variant==0)<<variant;
    EXPECT_EQ(gate.core().trajectoryId(),1);EXPECT_EQ(gate.writerCommitSequence(),1U);
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
TEST(TrackerHolonomicStartup, StationaryNativeEntryProducesDemandWithoutInventingMeasuredProgress) {
  for(const std::string mode:{"live","isolated_mock"}) {
    SCOPED_TRACE(mode);
    auto settings=executionConfig();settings.motion_model="holonomic";
    ExecutionContract gate(settings,mode,{},true);
    auto trajectory=curve();
    for(std::size_t i=0;i<trajectory.points.size();++i)
      trajectory.points[i].x()=i<3?0.:.001*(i-2)*(i-2);
    certify(trajectory);ASSERT_DOUBLE_EQ(trajectory.join_velocity.norm(),0.);
    auto body=odom();body.planar_speed=0.;body.velocity_in_frame.setZero();
    stage(gate,trajectory,body,mode);
    const auto admission=gate.prepare(100.,10.);ASSERT_TRUE(admission&&admission->accepted);
    auto permit=permitWire();permit.transport_mode=mode;permit.allowed=false;permit.phase="preview";
    permit.sdk_session.clear();permit.sdk_arm_generation=0;
    ASSERT_TRUE(gate.permit(permit,100.,10.));
    EXPECT_TRUE(gate.step(100.01,10.01).hold); // Preview remains geometry only.
    permit.allowed=true;permit.phase="tracking";permit.sdk_session="sdk";permit.sdk_arm_generation=1;
    Eigen::Vector2d previous=Eigen::Vector2d::Zero();
    for(int i=1;i<=40;++i) {
      const double now=100.+.05*i;
      body=odom(now);body.planar_speed=0.;body.velocity_in_frame.setZero();
      ASSERT_TRUE(gate.receiveOdom(body,now,now-90.));
      auto proof=proofWire();proof.transport_mode=mode;proof.sequence=i+1;
      proof.source_stamp=proof.check_begin=proof.check_end=proof.body_source_stamp=
        proof.front_ray_source_stamp=proof.rear_ray_source_stamp=stamp(now);
      proof.valid_until=stamp(now+.25);gate.validation(proof,now);
      permit.sequence=i+1;permit.validation_sequence=proof.sequence;
      permit.source_stamp=stamp(now);permit.valid_until=stamp(now+.2);
      ASSERT_TRUE(gate.permit(permit,now,now-90.));
      const auto demand=gate.step(now,now-90.);
      ASSERT_EQ(demand.reason,"tracking");EXPECT_FALSE(demand.hold);
      const Eigen::Vector2d requested(demand.velocity.linear.x,demand.velocity.linear.y);
      EXPECT_GT(requested.x(),0.);EXPECT_NEAR(requested.y(),0.,1e-12);
      EXPECT_LE(requested.norm(),settings.kp_position*settings.min_lookahead_distance_m+1e-12);
      EXPECT_LE((requested-previous).norm(),settings.max_acceleration*.05+1e-12);
      EXPECT_FALSE(demand.safety_checked); // Native still owns the exact demand sweep.
      EXPECT_EQ(demand.sdk_session,permit.sdk_session);EXPECT_EQ(demand.trajectory_id,1);
      EXPECT_DOUBLE_EQ(gate.core().progressTime(),0.);
      EXPECT_DOUBLE_EQ(gate.core().progressArc(),0.);
      EXPECT_DOUBLE_EQ(gate.core().committedArc(),0.);
      if(i==1) {
        auto applied=initialWriterAck(permit);applied.applied_at=stamp(now);
        ASSERT_TRUE(gate.commitAck(applied,now,now-90.));
      }
      EXPECT_EQ(gate.writerCommitSequence(),1U);previous=requested;
    }
  }
}
TEST(TrackerHolonomicStartup, SpatialBootstrapCannotBypassExpiredOrNewerInvalidNativeProof) {
  for(int variant=0;variant<3;++variant) {
    SCOPED_TRACE(variant);
    auto settings=executionConfig();settings.motion_model="holonomic";
    ExecutionContract gate(settings,"isolated_mock",{},true);auto trajectory=curve();
    for(std::size_t i=0;i<trajectory.points.size();++i)
      trajectory.points[i].x()=i<3?0.:.001*(i-2)*(i-2);
    certify(trajectory);auto body=odom();body.planar_speed=0.;body.velocity_in_frame.setZero();
    stage(gate,trajectory,body);ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
    auto permit=permitWire();
    if(variant==0){permit.allowed=false;permit.phase="preview";}
    ASSERT_TRUE(gate.permit(permit,100.,10.));
    const double now=variant==1?100.3:100.05;
    body=odom(now);body.planar_speed=0.;body.velocity_in_frame.setZero();
    ASSERT_TRUE(gate.receiveOdom(body,now,now-90.));
    if(variant==2) {
      auto blocked=proofWire();blocked.sequence=2;blocked.valid=false;
      blocked.source_stamp=blocked.check_begin=blocked.check_end=blocked.body_source_stamp=
        blocked.front_ray_source_stamp=blocked.rear_ray_source_stamp=stamp(now);
      blocked.valid_until=stamp(now+.25);gate.validation(blocked,now);
      // A late positive predecessor must not replace the newer negative fact.
      gate.validation(proofWire(),now);
    }
    const auto demand=gate.step(now,now-90.);
    EXPECT_TRUE(demand.hold);EXPECT_DOUBLE_EQ(demand.velocity.linear.x,0.);
    EXPECT_DOUBLE_EQ(demand.velocity.linear.y,0.);EXPECT_EQ(gate.core().trajectoryId(),1);
    EXPECT_TRUE(gate.core().hasInstalledGeometry());EXPECT_DOUBLE_EQ(gate.core().progressArc(),0.);
  }
}
TEST(TrackerHolonomicStartup, RejectedCandidateKeepsSafeStationaryEntryAndLimiterHistory) {
  auto settings=executionConfig();settings.motion_model="holonomic";
  ExecutionContract gate(settings,"isolated_mock",{},true);auto trajectory=curve();
  for(std::size_t i=0;i<trajectory.points.size();++i)
    trajectory.points[i].x()=i<3?0.:.001*(i-2)*(i-2);
  certify(trajectory);auto body=odom();body.planar_speed=0.;body.velocity_in_frame.setZero();
  stage(gate,trajectory,body);ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
  ASSERT_TRUE(gate.permit(permitWire(),100.,10.));const auto before=gate.step(100.05,10.05);
  ASSERT_GT(before.velocity.linear.x,0.);
  auto bad=trajectory;bad.id=2;for(auto& point:bad.points)point.z()+=.1;certify(bad);
  gate.candidate(bad);auto proof=proofWire();proof.sequence=2;proof.trajectory_id=2;gate.validation(proof,100.05);
  const auto admission=gate.prepare(100.05,10.05);ASSERT_TRUE(admission);EXPECT_FALSE(admission->accepted);
  body=odom(100.1);body.planar_speed=0.;body.velocity_in_frame.setZero();
  ASSERT_TRUE(gate.receiveOdom(body,100.1,10.1));const auto after=gate.step(100.1,10.1);
  EXPECT_EQ(after.trajectory_id,1);EXPECT_EQ(after.reason,"tracking");EXPECT_FALSE(after.hold);
  EXPECT_GE(after.velocity.linear.x,before.velocity.linear.x);
  EXPECT_LE(after.velocity.linear.x-before.velocity.linear.x,settings.max_acceleration*.05+1e-12);
  EXPECT_DOUBLE_EQ(gate.core().progressTime(),0.);EXPECT_EQ(gate.writerCommitSequence(),0U);
}
TEST(TrackerHolonomicStartup, AdmittedMidCurveStationaryBoundaryRestartsWithoutResettingProjection) {
  auto settings=executionConfig();settings.motion_model="holonomic";
  ExecutionContract gate(settings,"isolated_mock",{},true);auto trajectory=curve();
  const double approach[]={-.015,0.,.015,.03,.045,.06,.0725,.08,.0825,.0825,.0825};
  for(std::size_t i=0;i<trajectory.points.size();++i)
    trajectory.points[i].x()=i<11?approach[i]:.0825+.001*(i-10)*(i-10);
  certify(trajectory,100.,1.6);ASSERT_NEAR(trajectory.join_position.x(),.0825,1e-12);
  ASSERT_DOUBLE_EQ(trajectory.join_velocity.norm(),0.);
  auto body=odom(100.,.0825);body.planar_speed=0.;body.velocity_in_frame.setZero();
  stage(gate,trajectory,body);
  auto proof=proofWire();proof.sequence=2;proof.valid_start_time=trajectory.valid_start_time;
  proof.valid_start_arc_length=trajectory.valid_start_arc_length;gate.validation(proof,100.);
  const auto admission=gate.prepare(100.,10.);ASSERT_TRUE(admission&&admission->accepted)<<gate.lastPrepareReason();
  auto permit=permitWire();permit.validation_sequence=2;ASSERT_TRUE(gate.permit(permit,100.,10.));
  const auto command=gate.step(100.05,10.05);EXPECT_GT(command.velocity.linear.x,0.);
  EXPECT_EQ(command.reason,"tracking");EXPECT_NEAR(gate.core().progressTime(),1.6,1e-12);
  EXPECT_NEAR(gate.core().progressArc(),.0825,1e-6); // Neither restarted at 0 nor advanced by time.
}
TEST(TrackerWriterBootstrap, UnboundPreviewAcceptsFirstSdkBindingInSameOwnerEpoch) {
  for(const std::string mode:{"live","isolated_mock"}) {
    ExecutionContract gate(executionConfig(),mode,{},true);stage(gate,curve(),odom(),mode);
    ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
    auto preview=permitWire();preview.transport_mode=mode;preview.allowed=false;preview.phase="preview";
    preview.control_epoch=3;preview.sdk_session.clear();preview.sdk_arm_generation=0;preview.confirmation_id.clear();
    ASSERT_TRUE(gate.permit(preview,100.,10.));
    const auto before=gate.step(100.02,10.02);
    EXPECT_EQ(before.reason,"geometry_only_no_motion_permission");EXPECT_TRUE(before.hold);
    auto executing=preview;executing.sequence=2;executing.allowed=true;executing.phase="tracking";
    executing.sdk_session="current_sdk";executing.sdk_arm_generation=3;executing.confirmation_id="confirmed_route";
    executing.source_stamp=stamp(100.03);executing.valid_until=stamp(100.23);
    ASSERT_TRUE(gate.permit(executing,100.03,10.03))<<mode;
    const auto after=gate.step(100.05,10.05);
    EXPECT_EQ(after.sdk_session,executing.sdk_session);EXPECT_EQ(after.sdk_arm_generation,3U);
    EXPECT_EQ(after.execution_id,preview.execution_id);EXPECT_EQ(after.control_epoch,preview.control_epoch);
    EXPECT_GT(after.velocity.linear.x,0.);EXPECT_FALSE(after.safety_checked);
    EXPECT_EQ(gate.core().trajectoryId(),1);EXPECT_EQ(gate.writerCommitSequence(),0U);
    auto applied=initialWriterAck(executing);applied.applied_at=stamp(100.05);
    ASSERT_TRUE(gate.commitAck(applied,100.05,10.05));EXPECT_EQ(gate.writerCommitSequence(),1U);
    ASSERT_TRUE(gate.receiveOdom(odom(100.07),100.07,10.07));
    const auto continuing=gate.step(100.07,10.07);
    EXPECT_GT(continuing.velocity.linear.x,0.);EXPECT_EQ(continuing.sdk_session,executing.sdk_session);
    EXPECT_EQ(continuing.trajectory_id,1);EXPECT_EQ(gate.writerCommitSequence(),1U);
  }
}
TEST(TrackerWriterBootstrap, EstablishedSdkBindingCannotBeReplacedBySameEpochOrLatePreview) {
  for(int fault=0;fault<7;++fault) {
    ExecutionContract gate(executionConfig(),"isolated_mock",{},true);stage(gate);
    ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto executing=permitWire();
    ASSERT_TRUE(gate.permit(executing,100.,10.));EXPECT_GT(gate.step(100.02,10.02).velocity.linear.x,0.);
    auto late=executing;late.sequence=2;late.source_stamp=stamp(100.03);late.valid_until=stamp(100.23);
    if(fault==0)late.sdk_session="other_sdk";
    if(fault==1)++late.sdk_arm_generation;
    if(fault==2)late.execution_id="other_exec";
    if(fault==3){late.allowed=false;late.phase="preview";late.sdk_session.clear();late.sdk_arm_generation=0;}
    if(fault==4){late.sdk_session.clear();late.sdk_arm_generation=0;}
    if(fault==5){late.allowed=false;late.phase="arming";late.sdk_session="other_sdk";}
    if(fault==6){late.allowed=false;late.phase="preview";late.sdk_session.clear();late.sdk_arm_generation=0;
      late.execution_id.clear();late.control_epoch=0;late.confirmation_id.clear();}
    EXPECT_FALSE(gate.permit(late,100.03,10.03))<<fault;
    const auto retained=gate.step(100.04,10.04);
    EXPECT_EQ(retained.sdk_session,executing.sdk_session);EXPECT_EQ(retained.sdk_arm_generation,executing.sdk_arm_generation);
    EXPECT_EQ(retained.execution_id,executing.execution_id);EXPECT_EQ(retained.control_epoch,executing.control_epoch);
    EXPECT_EQ(gate.core().trajectoryId(),1);EXPECT_EQ(gate.writerCommitSequence(),0U);
  }
}
TEST(TrackerWriterBootstrap, FirstBindingStillRequiresMatchingOwnerAndCompleteAuthority) {
  for(int fault=0;fault<10;++fault) {
    ExecutionContract gate(executionConfig(),"isolated_mock",{},true);stage(gate);
    ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto preview=permitWire();
    preview.allowed=false;preview.phase="preview";preview.sdk_session.clear();preview.sdk_arm_generation=0;
    ASSERT_TRUE(gate.permit(preview,100.,10.));
    auto executing=permitWire();executing.sequence=2;executing.source_stamp=stamp(100.03);executing.valid_until=stamp(100.23);
    if(fault==0)executing.execution_id="other_exec";
    if(fault==1)++executing.control_epoch;
    if(fault==2)executing.sdk_session.clear();
    if(fault==3)executing.sdk_arm_generation=0;
    if(fault==4)executing.confirmation_id.clear();
    if(fault==5)executing.version.task_id="other_task";
    if(fault==6)executing.valid_until=stamp(100.02);
    if(fault==7)executing.version.anchor_revision=2;
    if(fault==8)executing.trajectory_id=2;
    if(fault==9)executing.version.route_hash=std::string(64,'b');
    EXPECT_FALSE(gate.permit(executing,100.03,10.03))<<fault;
    const auto retained=gate.step(100.04,10.04);
    EXPECT_TRUE(retained.hold);EXPECT_TRUE(retained.sdk_session.empty());EXPECT_EQ(retained.sdk_arm_generation,0U);
    EXPECT_EQ(gate.core().trajectoryId(),1);EXPECT_EQ(gate.writerCommitSequence(),0U);
  }
}
TEST(TrackerWriterBootstrap, ReincarnationMustStageFreshDisallowedOwnerBeforeFirstBinding) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true);stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto old=permitWire();
  ASSERT_TRUE(gate.permit(old,100.,10.));gate.step(100.02,10.02);
  ASSERT_TRUE(gate.commitAck(initialWriterAck(old),100.02,10.02));EXPECT_EQ(gate.writerCommitSequence(),1U);
  auto arming=old;arming.sequence=2;++arming.control_epoch;arming.allowed=false;arming.phase="arming";
  arming.sdk_session.clear();arming.sdk_arm_generation=0;arming.source_stamp=stamp(100.03);arming.valid_until=stamp(100.23);
  ASSERT_TRUE(gate.permit(arming,100.03,10.03));EXPECT_EQ(gate.writerCommitSequence(),0U);
  EXPECT_TRUE(gate.step(100.04,10.04).hold);
  auto executing=arming;executing.sequence=3;executing.allowed=true;executing.phase="tracking";
  executing.sdk_session="new_sdk";executing.sdk_arm_generation=2;
  ASSERT_TRUE(gate.permit(executing,100.05,10.05));
  EXPECT_GT(gate.step(100.06,10.06).velocity.linear.x,0.);
  old.sequence=4;old.source_stamp=stamp(100.07);old.valid_until=stamp(100.27);
  EXPECT_FALSE(gate.permit(old,100.07,10.07));
  EXPECT_EQ(gate.step(100.08,10.08).sdk_session,"new_sdk");
}
TEST(TrackerWriterBootstrap, NormalGeometryAdvanceDuringGrantDoesNotBecomeNewTaskOrEpoch) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true);stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto preview=permitWire();
  preview.allowed=false;preview.phase="preview";preview.sdk_session.clear();preview.sdk_arm_generation=0;
  ASSERT_TRUE(gate.permit(preview,100.,10.));
  const auto next_version=version(2);auto proposal=proposalWire(next_version);gate.proposal(proposal,100.02);
  auto next_curve=curve(2);next_curve.generation=2;next_curve.identity=identity(next_version);
  gate.candidate(next_curve);gate.support(supportWire(next_version));auto proof=proofWire(next_version);
  proof.trajectory_id=2;proof.sequence=2;gate.validation(proof,100.02);ASSERT_TRUE(gate.prepare(100.02,10.02)->accepted);
  auto next=preview;next.sequence=2;next.version=next_version;next.trajectory_id=2;next.validation_sequence=2;
  ASSERT_TRUE(gate.permit(next,100.02,10.02));EXPECT_EQ(gate.core().trajectoryId(),2);
  // An arming lease may retain the pre-grant version. It has no authority to
  // move or replace geometry, and must not turn the same task into a new one.
  auto arming=preview;arming.sequence=3;arming.geometry_committed=false;arming.trajectory_id=0;arming.phase="arming";
  ASSERT_TRUE(gate.permit(arming,100.03,10.03));EXPECT_TRUE(gate.step(100.03,10.03).hold);
  next.sequence=4;next.allowed=true;next.phase="tracking";
  next.sdk_session="current_sdk";next.sdk_arm_generation=1;
  ASSERT_TRUE(gate.permit(next,100.04,10.04));
  EXPECT_EQ(gate.step(100.05,10.05).sdk_session,"current_sdk");
  EXPECT_GT(gate.step(100.06,10.06).velocity.linear.x,0.);
  EXPECT_EQ(gate.core().trajectoryId(),2);EXPECT_EQ(gate.core().generation(),2U);
}
TEST(TrackerWriterBootstrap, HigherDisallowedCompleteGrantCanRetireUnboundOwnerWithoutEmptyIntermediate) {
  for(const std::string phase:{"arming","holding"}) {
    ExecutionContract gate(executionConfig(),"isolated_mock",{},true);stage(gate);
    ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto preview=permitWire();
    preview.allowed=false;preview.phase="preview";preview.sdk_session.clear();preview.sdk_arm_generation=0;
    ASSERT_TRUE(gate.permit(preview,100.,10.));
    auto arming=preview;arming.sequence=2;++arming.control_epoch;arming.execution_id="next_exec";arming.phase=phase;
    arming.sdk_session="current_sdk";arming.sdk_arm_generation=2;
    ASSERT_TRUE(gate.permit(arming,100.03,10.03));EXPECT_TRUE(gate.step(100.04,10.04).hold);
    auto executing=arming;executing.sequence=3;executing.allowed=true;executing.phase="tracking";
    ASSERT_TRUE(gate.permit(executing,100.05,10.05));
    const auto demand=gate.step(100.06,10.06);
    EXPECT_EQ(demand.execution_id,"next_exec");EXPECT_EQ(demand.sdk_session,"current_sdk");
    EXPECT_GT(demand.velocity.linear.x,0.);EXPECT_EQ(gate.writerCommitSequence(),0U);
    preview.sequence=4;preview.source_stamp=stamp(100.07);preview.valid_until=stamp(100.27);
    EXPECT_FALSE(gate.permit(preview,100.07,10.07));
  }
}
TEST(TrackerWriterBootstrap, UnboundOwnerCannotBeOverwrittenByForeignOrAllowedEpoch) {
  for(int fault=0;fault<6;++fault) {
    ExecutionContract gate(executionConfig(),"isolated_mock",{},true);stage(gate);
    ASSERT_TRUE(gate.prepare(100.,10.)->accepted);auto preview=permitWire();
    preview.allowed=false;preview.phase="preview";preview.sdk_session.clear();preview.sdk_arm_generation=0;
    ASSERT_TRUE(gate.permit(preview,100.,10.));
    auto wrong=preview;wrong.sequence=2;
    if(fault==0)wrong.execution_id="other_exec";
    if(fault==1){++wrong.control_epoch;wrong.phase="preview";}
    if(fault==2){++wrong.control_epoch;wrong.allowed=true;wrong.sdk_session="sdk";wrong.sdk_arm_generation=2;wrong.phase="tracking";}
    if(fault==3){wrong.sdk_session="sdk";}
    if(fault==4){wrong.sdk_arm_generation=2;}
    if(fault==5){wrong.execution_id.clear();wrong.control_epoch=0;wrong.confirmation_id.clear();}
    EXPECT_FALSE(gate.permit(wrong,100.03,10.03))<<fault;
    auto executing=permitWire();executing.sequence=3;
    ASSERT_TRUE(gate.permit(executing,100.04,10.04))<<fault;
    EXPECT_GT(gate.step(100.05,10.05).velocity.linear.x,0.);
  }
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
    // Both are fresh leases for already committed geometry; only the
    // current exact native proof can produce a nonzero motion demand.
    ASSERT_TRUE(gate.permit(p,now,now-90.));
    const auto output=gate.step(now+.02,now-90.+.02);
    if(current_proof)EXPECT_GT(output.velocity.linear.x,0.);
    else {
      EXPECT_TRUE(output.hold);EXPECT_FALSE(output.safety_checked);
      EXPECT_DOUBLE_EQ(output.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(output.velocity.angular.z,0.);
      EXPECT_EQ(output.reason,"permission_or_native_proof_expired");
    }
  }
}
TEST(TrackerExecution, NativeUnknownExpiredProofWrongOwnerAndMockLeakCannotCommit) {
  for(int variant=0;variant<6;++variant) {
    ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);auto proof=proofWire();
    if(variant==0)proof.valid=false;
    if(variant==1)proof.valid_until=stamp(99.);
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
TEST(TrackerExecution, ScanDeadlineOwnsPerceptionFreshnessWithoutRenewingEvidence) {
  ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
  auto proof=proofWire();proof.sequence=2;
  proof.body_source_stamp=proof.front_ray_source_stamp=proof.rear_ray_source_stamp=stamp(99.);
  proof.valid_until=stamp(100.05);gate.validation(proof,100.);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
  auto permit=permitWire();permit.validation_sequence=2;
  ASSERT_TRUE(gate.permit(permit,100.,10.));
  EXPECT_GT(gate.step(100.02,10.02).velocity.linear.x,0.);
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.06),100.06,10.06));
  EXPECT_TRUE(gate.step(100.06,10.06).hold); // Original SCAN deadline, not metadata rechecked here.
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
    if(variant==1)s.floor_id="floor3";
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
    ASSERT_TRUE(gate.permit(p,100.02,10.02));
    ASSERT_TRUE(gate.core().receiveOdom(odom(100.02),100.02,10.02));
    const auto demand=gate.step(100.02,10.02);
    if(failure==1) {EXPECT_FALSE(demand.hold);EXPECT_GT(demand.velocity.linear.x,0.);}
    else {
      EXPECT_TRUE(demand.hold);EXPECT_FALSE(demand.safety_checked);
      EXPECT_DOUBLE_EQ(demand.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(demand.velocity.angular.z,0.);
      EXPECT_EQ(demand.reason,"permission_or_native_proof_expired");
    }
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
  // A fresh owner heartbeat does not erase the newer invalidation fence.
  ++p.sequence;ASSERT_TRUE(gate.permit(p,100.04,10.04));
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.04),100.04,10.04));
  const auto demand=gate.step(100.04,10.04);
  EXPECT_TRUE(demand.hold);EXPECT_FALSE(demand.safety_checked);
  EXPECT_DOUBLE_EQ(demand.velocity.linear.x,0.);EXPECT_DOUBLE_EQ(demand.velocity.angular.z,0.);
  EXPECT_EQ(demand.reason,"permission_or_native_proof_expired");EXPECT_EQ(gate.core().trajectoryId(),1);
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
  // A known floor label alone is not wrong-floor geometry. Keep the measured
  // body/candidate in the lower corridor and provide actual upper-floor Z.
  support.floor_id="floor2";for(auto& p:support.ground)p.z()+=3.;next.id=3;
  EXPECT_FALSE(core.admitRevision(new_t,next,support,100.,10.,true));
  EXPECT_EQ(core.trajectoryId(),2);EXPECT_EQ(core.generation(),2);
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
wire::MotionDemand ownStraightDemand(ExecutionContract& gate) {
  stage(gate);const auto prepared=gate.prepare(100.,10.);
  EXPECT_TRUE(prepared&&prepared->accepted);EXPECT_TRUE(gate.permit(permitWire(),100.,10.));
  EXPECT_TRUE(gate.core().receiveOdom(odom(100.02),100.02,10.02));
  const auto demand=gate.step(100.02,10.02);
  EXPECT_GT(demand.velocity.linear.x,0.);EXPECT_NEAR(demand.velocity.angular.z,0.,1e-9);
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
TEST(TrackerExecution, ScanOccupiedVerdictIsNotDiscardedByAnotherSensorAgePolicy) {
  ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'));
  const auto demand=ownTurningDemand(gate);auto feedback=occupiedFor(demand);
  feedback.body_source_stamp=feedback.front_ray_source_stamp=feedback.rear_ray_source_stamp=stamp(99.);
  ASSERT_TRUE(gate.motionValidation(feedback,100.04));
  EXPECT_STREQ(gate.core().maneuverPhase(),"decelerating_for_turn");
}

TEST(TrackerExecution, OwnExactFreshStraightOccupiedReachesNativeDifferentEntryFeedback) {
  ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'));
  const auto demand=ownStraightDemand(gate);const auto feedback=occupiedFor(demand);
  ASSERT_TRUE(gate.motionValidation(feedback,100.04));
  EXPECT_STREQ(gate.core().maneuverPhase(),"waiting_executable_entry");
  const auto forwarded=gate.takeBlockedEntry();ASSERT_TRUE(forwarded);
  EXPECT_EQ(*forwarded,feedback); // Original proof, never a re-stamped permission.
  EXPECT_FALSE(gate.takeBlockedEntry());
  EXPECT_FALSE(gate.motionValidation(feedback,100.04));
  ASSERT_TRUE(gate.core().receiveOdom(odom(100.06),100.06,10.06));
  const auto hold=gate.step(100.06,10.06);
  EXPECT_TRUE(hold.hold);EXPECT_EQ(hold.velocity.linear.x,0.);
  EXPECT_EQ(hold.velocity.angular.z,0.);EXPECT_TRUE(gate.core().active());
}

TEST(TrackerExecution, StraightUnknownStaleForeignOrMutatedProofCannotRequestDifferentEntry) {
  for(int variant=0;variant<8;++variant) {
    SCOPED_TRACE(variant);
    ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'));
    const auto demand=ownStraightDemand(gate);auto feedback=occupiedFor(demand);double now=100.04;
    switch(variant) {
      case 0:feedback.reason="motion_sweep_unknown_or_expired";break;
      case 1:feedback.reason="motion_evidence_expired_during_check";break;
      case 2:feedback.version.task_id="foreign";break;
      case 3:feedback.demand_sequence++;break;
      case 4:feedback.velocity.linear.x=0.;break;
      case 5:feedback.velocity.linear.x=-.1;break;
      case 6:feedback.demand_body_source_stamp.nanosec++;break;
      case 7:now=100.13;break;
    }
    EXPECT_FALSE(gate.motionValidation(feedback,now));EXPECT_FALSE(gate.takeBlockedEntry());
    EXPECT_STREQ(gate.core().maneuverPhase(),"following");
  }
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
      case 17:feedback.check_end=stamp(99.);break;
      case 18:feedback.check_begin=stamp(100.032);break;
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

TEST(TrackerPreparationWorker, ReservedOlderConsumerCannotStealLatestCompletedPreparation) {
  BoundedPreparationWorker<int,int> worker([](const int& value,const auto&){return value;});
  const auto original=worker.submit(2);
  const auto latest=worker.submit(3);
  for(int i=0;i<1000&&!BoundedPreparationWorkerTestAccess::completionReady(worker);++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(BoundedPreparationWorkerTestAccess::completionReady(worker));
  for(int i=0;i<50;++i)EXPECT_FALSE(worker.take(original));
  const auto done=worker.take(latest);ASSERT_TRUE(done&&done->result);
  EXPECT_EQ(done->token,latest);EXPECT_EQ(done->request,3);EXPECT_EQ(*done->result,3);
  EXPECT_FALSE(worker.take(latest)); // Exactly one result, not a second queue.
}

TEST(TrackerPreparationWorker, TokenMatchedReadStillHonorsGlobalInvalidationAndLatestOnlySubmission) {
  BoundedPreparationWorker<int,int> worker([](const int& value,const auto&){return value;});
  const auto old=worker.submit(1);
  for(int i=0;i<1000&&!BoundedPreparationWorkerTestAccess::completionReady(worker);++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(BoundedPreparationWorkerTestAccess::completionReady(worker));
  worker.invalidate();EXPECT_FALSE(worker.take(old));
  const auto latest=worker.submit(2);
  for(int i=0;i<1000&&!BoundedPreparationWorkerTestAccess::completionReady(worker);++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(BoundedPreparationWorkerTestAccess::completionReady(worker));
  EXPECT_FALSE(worker.take(old));
  const auto done=worker.take(latest);ASSERT_TRUE(done&&done->result);
  EXPECT_EQ(done->request,2);EXPECT_EQ(*done->result,2);
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
Config holonomicExecutionConfig() {
  auto c=executionConfig();c.motion_model="holonomic";c.holonomic_yaw_policy="hold_heading";return c;
}
Trajectory holonomicCurve(double direction,std::int64_t id=1,std::uint64_t generation=1) {
  auto t=curve(id);t.generation=generation;t.identity=d1max_trajectory_tracker::identity(version(generation));
  const Eigen::Rotation2Dd rotation(direction);
  for(auto& p:t.points)p.head<2>()=rotation*p.head<2>();
  t.frozen_yaw=0.;certify(t);return t;
}
Odom holonomicBody(double direction,double now=100.) {
  auto body=odom(now);body.velocity_in_frame={.25*std::cos(direction),.25*std::sin(direction),0.};return body;
}
wire::SupportReference holonomicSupport(double direction,const wire::ExecutionVersion& v=version()) {
  auto support=supportWire(v);
  for(auto& p:support.support_ground_xyz) {const double x=p.x;p.x=x*std::cos(direction);p.y=x*std::sin(direction);}
  return support;
}
wire::ExecutionPermit holonomicPermit(double direction,const wire::ExecutionVersion& v=version()) {
  auto p=permitWire(v);p.goal_position.x=2.*std::cos(direction);p.goal_position.y=2.*std::sin(direction);return p;
}
void stageHolonomic(ExecutionContract& gate,double direction,Trajectory candidate) {
  ASSERT_TRUE(gate.core().receiveOdom(holonomicBody(direction),100.,10.));
  auto proposal=proposalWire();proposal.goal_position=holonomicPermit(direction).goal_position;
  gate.proposal(proposal,100.);gate.candidate(std::move(candidate));
  gate.support(holonomicSupport(direction));gate.validation(proofWire(),100.);
}
wire::ExecutionHandoffGrant holonomicHandoff(ExecutionContract& gate,double direction,double frozen_yaw=0.) {
  auto first=holonomicCurve(direction);first.frozen_yaw=frozen_yaw;
  stageHolonomic(gate,direction,first);
  EXPECT_TRUE(gate.prepare(100.,10.)->accepted);const auto incumbent=holonomicPermit(direction);
  EXPECT_TRUE(gate.permit(incumbent,100.,10.));gate.step(100.02,10.02);
  EXPECT_TRUE(gate.commitAck(initialWriterAck(incumbent),100.02,10.02));
  const auto v=version(2);auto proposal=proposalWire(v);proposal.expected_version=incumbent.version;
  proposal.expected_trajectory_id=1;proposal.goal_position=incumbent.goal_position;gate.proposal(proposal,100.02);
  auto second=holonomicCurve(direction,2,2);second.frozen_yaw=frozen_yaw;
  gate.candidate(second);gate.support(holonomicSupport(direction,v));
  auto proof=proofWire(v);proof.trajectory_id=2;proof.sequence=2;gate.validation(proof,100.02);
  EXPECT_TRUE(gate.prepare(100.02,10.02)->accepted);
  wire::ExecutionHandoffGrant g;g.schema_version=2;g.handoff_id="holonomic-handoff";g.sequence=1;
  g.expected_commit_sequence=1;g.incumbent=incumbent;g.candidate=holonomicPermit(direction,v);
  g.candidate.sequence=2;g.candidate.trajectory_id=2;g.candidate.validation_sequence=2;g.candidate.geometry_committed=false;
  g.source_stamp=stamp(100.02);g.valid_until=g.transition_deadline=stamp(100.22);
  g.retain_incumbent_until=incumbent.valid_until;return g;
}
}

TEST(TrackerHolonomicExecution, PureLateralDemandIsNotHoldAndProofLossZerosAllAxes) {
  const double direction=1.5707963267948966;
  ExecutionContract gate(holonomicExecutionConfig(),"isolated_mock",std::string(64,'f'));
  stageHolonomic(gate,direction,holonomicCurve(direction));
  auto admission=gate.prepare(100.,10.);ASSERT_TRUE(admission&&admission->accepted)<<gate.lastPrepareReason();
  ASSERT_TRUE(gate.permit(holonomicPermit(direction),100.,10.));
  const auto demand=gate.step(100.02,10.02);
  EXPECT_FALSE(demand.hold);EXPECT_GT(demand.velocity.linear.y,0.);
  EXPECT_NEAR(demand.velocity.linear.x,0.,1e-12);EXPECT_EQ(demand.velocity.angular.z,0.);
  EXPECT_FALSE(demand.safety_checked); // Demand is not a native sweep certificate.
  auto invalid=proofWire();invalid.sequence=2;invalid.valid=false;gate.validation(invalid,100.03);
  const auto hold=gate.step(100.04,10.04);
  EXPECT_TRUE(hold.hold);EXPECT_EQ(hold.velocity.linear.x,0.);EXPECT_EQ(hold.velocity.linear.y,0.);
  EXPECT_EQ(hold.velocity.angular.z,0.);
}

TEST(TrackerHolonomicExecution, ReverseDemandAndPureSidewaysOccupiedFeedbackUseExactNativeSweep) {
  for(double direction:{1.5707963267948966,3.141592653589793}) {
    SCOPED_TRACE(direction);
    ExecutionContract gate(holonomicExecutionConfig(),"isolated_mock",std::string(64,'f'));
    stageHolonomic(gate,direction,holonomicCurve(direction));
    ASSERT_TRUE(gate.prepare(100.,10.)->accepted);ASSERT_TRUE(gate.permit(holonomicPermit(direction),100.,10.));
    const auto demand=gate.step(100.02,10.02);ASSERT_FALSE(demand.hold);
    if(direction>3.) {EXPECT_LT(demand.velocity.linear.x,0.);} else {EXPECT_GT(demand.velocity.linear.y,0.);}
    auto occupied=occupiedFor(demand),mutated=occupied;mutated.velocity.linear.y+=.001;
    EXPECT_FALSE(gate.motionValidation(mutated,100.04));
    ASSERT_TRUE(gate.motionValidation(occupied,100.04));
    EXPECT_STREQ(gate.core().maneuverPhase(),"waiting_executable_entry");
    const auto forwarded=gate.takeBlockedEntry();ASSERT_TRUE(forwarded);EXPECT_EQ(*forwarded,occupied);
    const auto hold=gate.step(100.04,10.04);EXPECT_TRUE(hold.hold);
    EXPECT_EQ(hold.velocity.linear.x,0.);EXPECT_EQ(hold.velocity.linear.y,0.);EXPECT_EQ(hold.velocity.angular.z,0.);
    EXPECT_TRUE(gate.core().active());EXPECT_EQ(gate.core().trajectoryId(),1);
  }
}

TEST(TrackerHolonomicExecution, PreparedDemandAndWriterAckPreserveExactLateralHistory) {
  const double direction=1.5707963267948966;
  ExecutionContract gate(holonomicExecutionConfig(),"isolated_mock",std::string(64,'f'),true);
  const auto grant=holonomicHandoff(gate,direction);ASSERT_TRUE(gate.handoff(grant,100.02,10.02));
  ASSERT_TRUE(gate.core().receiveOdom(holonomicBody(direction,100.04),100.04,10.04));
  const auto prepared=gate.preparedStep(100.04,10.04);ASSERT_TRUE(prepared)<<gate.handoffReason();
  EXPECT_FALSE(prepared->demand.hold);EXPECT_GT(prepared->demand.velocity.linear.y,.01);
  EXPECT_NEAR(prepared->demand.velocity.linear.x,0.,1e-12);EXPECT_EQ(prepared->demand.velocity.angular.z,0.);
  EXPECT_NEAR(prepared->measured_twist.linear.y,.25,1e-12);EXPECT_NEAR(prepared->curve_entry_twist.linear.y,.25,1e-12);
  EXPECT_EQ(gate.core().trajectoryId(),1);EXPECT_EQ(prepared->demand.trajectory_id,2);
  auto ack=preparedWriterAck(grant,*prepared),wrong=ack;wrong.applied_velocity.linear.y=0.;
  EXPECT_FALSE(gate.commitAck(wrong,100.04,10.04));EXPECT_EQ(gate.writerCommitSequence(),1U);
  ASSERT_TRUE(gate.commitAck(ack,100.04,10.04));EXPECT_EQ(gate.writerCommitSequence(),2U);
  ASSERT_TRUE(gate.core().receiveOdom(holonomicBody(direction,100.06),100.06,10.06));
  const auto next=gate.step(100.06,10.06);EXPECT_EQ(next.trajectory_id,2);
  EXPECT_GT(next.velocity.linear.y,prepared->demand.velocity.linear.y);
  EXPECT_LE(std::hypot(next.velocity.linear.x-ack.applied_velocity.linear.x,
    next.velocity.linear.y-ack.applied_velocity.linear.y),holonomicExecutionConfig().max_acceleration*.02+1e-12);
  EXPECT_FALSE(next.hold);
}

TEST(TrackerHolonomicExecution, TwentyHzMovingHandoffAndRejectedCandidateKeepHistoryUntilExactProofLoss) {
  const double direction=1.5707963267948966;
  ExecutionContract gate(holonomicExecutionConfig(),"isolated_mock",std::string(64,'f'),true);
  const auto grant=holonomicHandoff(gate,direction,.2);ASSERT_TRUE(gate.handoff(grant,100.02,10.02));
  auto body=holonomicBody(direction,100.07);body.position.y()=.0125;
  ASSERT_TRUE(gate.receiveOdom(body,100.07,10.07));
  const auto prepared=gate.preparedStep(100.07,10.07);ASSERT_TRUE(prepared)<<gate.handoffReason();
  ASSERT_FALSE(prepared->demand.hold);EXPECT_GT(prepared->curve_time,0.);
  EXPECT_GT(prepared->demand.velocity.linear.y,0.);EXPECT_GT(prepared->demand.velocity.angular.z,0.);
  const auto ack=preparedWriterAck(grant,*prepared);ASSERT_TRUE(gate.commitAck(ack,100.07,10.07));
  EXPECT_EQ(gate.core().trajectoryId(),2);EXPECT_GT(gate.core().progressArc(),0.);
  body=holonomicBody(direction,100.12);body.position.y()=.025;
  ASSERT_TRUE(gate.receiveOdom(body,100.12,10.12));
  const auto next=gate.step(100.12,10.12);ASSERT_FALSE(next.hold);
  const Eigen::Vector2d applied(ack.applied_velocity.linear.x,ack.applied_velocity.linear.y);
  const Eigen::Vector2d command(next.velocity.linear.x,next.velocity.linear.y);
  EXPECT_GT(next.velocity.linear.y,ack.applied_velocity.linear.y);
  EXPECT_LE((command-applied).norm(),.35*.05+1e-12);
  EXPECT_LE(std::abs(next.velocity.angular.z-ack.applied_velocity.angular.z),.8*.05+1e-12);
  // A failed successor is not withdrawal of this independently leased curve.
  auto bad=holonomicCurve(direction,3,2);bad.points.front().x()=NAN;gate.candidate(bad);
  auto candidate_proof=proofWire(version(2));candidate_proof.trajectory_id=3;candidate_proof.sequence=3;
  gate.validation(candidate_proof,100.12);
  const auto rejected=gate.prepare(100.12,10.12);ASSERT_TRUE(rejected);EXPECT_FALSE(rejected->accepted);
  body=holonomicBody(direction,100.17);body.position.y()=.0375;
  ASSERT_TRUE(gate.receiveOdom(body,100.17,10.17));
  const auto retained=gate.step(100.17,10.17);ASSERT_FALSE(retained.hold);EXPECT_EQ(retained.trajectory_id,2);
  EXPECT_GT(retained.velocity.linear.y,next.velocity.linear.y);
  EXPECT_LE(std::hypot(retained.velocity.linear.x-next.velocity.linear.x,
    retained.velocity.linear.y-next.velocity.linear.y),.35*.05+1e-12);
  EXPECT_LE(std::abs(retained.velocity.angular.z-next.velocity.angular.z),.8*.05+1e-12);
  auto lost=proofWire(version(2));lost.trajectory_id=2;lost.sequence=4;lost.valid=false;gate.validation(lost,100.17);
  body=holonomicBody(direction,100.22);body.position.y()=.05;
  ASSERT_TRUE(gate.receiveOdom(body,100.22,10.22));
  const auto hold=gate.step(100.22,10.22);EXPECT_TRUE(hold.hold);
  EXPECT_EQ(hold.velocity.linear.x,0.);EXPECT_EQ(hold.velocity.linear.y,0.);EXPECT_EQ(hold.velocity.angular.z,0.);
  EXPECT_EQ(gate.core().trajectoryId(),2);
}

TEST(TrackerHolonomicExecution, HoldHeadingMetadataIsRequiredAndCannotBeMutatedUnderSameCurveId) {
  for(int variant=0;variant<3;++variant) {
    auto candidate=holonomicCurve(0.);if(variant==0)candidate.frozen_yaw.reset();
    if(variant==1)candidate.frozen_yaw=NAN;
    if(variant==2)candidate.frozen_yaw=.4;
    ExecutionContract gate(holonomicExecutionConfig(),"isolated_mock");stageHolonomic(gate,0.,candidate);
    const auto admission=gate.prepare(100.,10.);ASSERT_TRUE(admission);
    if(variant<2){EXPECT_FALSE(admission->accepted);continue;}
    ASSERT_TRUE(admission->accepted);ASSERT_TRUE(gate.permit(holonomicPermit(0.),100.,10.));
    candidate.frozen_yaw=-.4;gate.candidate(candidate); // Same ID must remain immutable.
    const auto demand=gate.step(100.02,10.02);EXPECT_GT(demand.velocity.angular.z,0.);
    EXPECT_GT(demand.velocity.linear.x,0.);EXPECT_NEAR(demand.velocity.linear.y,0.,1e-12);
    EXPECT_FALSE(demand.hold);
  }
}

namespace {
scan_planner_msgs::msg::Bspline encodedHeldCurve(const Trajectory& t,double yaw) {
  scan_planner_msgs::msg::Bspline encoded;encoded.order=t.order;encoded.knots=t.knots;
  for(const auto& xyz:t.points) {
    geometry_msgs::msg::Point p;p.x=xyz.x();p.y=xyz.y();p.z=xyz.z();encoded.pos_pts.push_back(p);
  }
  encoded.yaw_dt=t.knots[1]-t.knots[0];encoded.yaw_pts.assign(t.points.size(),yaw);return encoded;
}
}

TEST(TrackerHolonomicEncoding, ProductionDecoderAcceptsLegacyAndConstantHeadingWithoutStaleValueReuse) {
  auto encoded=encodedHeldCurve(holonomicCurve(0.),.4);std::optional<double> yaw=-1.;
  ASSERT_TRUE(decodeFrozenHeading(encoded,yaw));ASSERT_TRUE(yaw);EXPECT_DOUBLE_EQ(*yaw,.4);
  encoded.yaw_pts.back()+=2.*3.141592653589793;
  ASSERT_TRUE(decodeFrozenHeading(encoded,yaw));ASSERT_TRUE(yaw);EXPECT_DOUBLE_EQ(*yaw,.4);
  encoded.yaw_pts.clear();ASSERT_TRUE(decodeFrozenHeading(encoded,yaw));EXPECT_FALSE(yaw);
}

TEST(TrackerHolonomicEncoding, ProductionDecoderRejectsMalformedHeadingAndTiming) {
  for(int variant=0;variant<12;++variant) {
    SCOPED_TRACE(variant);auto encoded=encodedHeldCurve(holonomicCurve(0.),.4);
    switch(variant) {
      case 0:encoded.yaw_pts.pop_back();break;
      case 1:encoded.knots.resize(1);break;
      case 2:encoded.yaw_dt=0.;break;
      case 3:encoded.yaw_dt=-.2;break;
      case 4:encoded.yaw_dt=NAN;break;
      case 5:encoded.yaw_dt=.3;break;
      case 6:encoded.yaw_pts.front()=NAN;break;
      case 7:encoded.yaw_pts.back()+=.01;break;
      case 8:encoded.yaw_pts.back()=INFINITY;break;
      case 9:encoded.knots.front()=NAN;break;
      case 10:encoded.knots[1]=INFINITY;break;
      case 11:encoded.knots[1]=encoded.knots[0];break;
    }
    std::optional<double> yaw=42.;EXPECT_FALSE(decodeFrozenHeading(encoded,yaw));EXPECT_FALSE(yaw);
  }
}

TEST(TrackerHolonomicEncoding, NativeConstantYawEncodingReachesTheActualExecutionContract) {
  auto candidate=holonomicCurve(0.);candidate.frozen_yaw.reset();
  const auto encoded=encodedHeldCurve(candidate,-.4);
  ASSERT_TRUE(decodeFrozenHeading(encoded,candidate.frozen_yaw));
  ExecutionContract gate(holonomicExecutionConfig(),"isolated_mock");stageHolonomic(gate,0.,candidate);
  const auto admission=gate.prepare(100.,10.);ASSERT_TRUE(admission&&admission->accepted);
  ASSERT_TRUE(gate.permit(holonomicPermit(0.),100.,10.));
  const auto demand=gate.step(100.02,10.02);
  EXPECT_GT(demand.velocity.linear.x,0.);EXPECT_NEAR(demand.velocity.linear.y,0.,1e-12);
  EXPECT_LT(demand.velocity.angular.z,0.);EXPECT_FALSE(demand.hold);
}

TEST(CommandPipelineBudget, OrdinaryDemandUsesConfiguredOriginalSourceCapNotLegacy100ms) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},false,false,.2);
  stage(gate);ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
  ASSERT_TRUE(gate.permit(permitWire(),100.,10.));
  const auto demand=gate.step(100.02,10.02);
  EXPECT_EQ(timeNs(demand.valid_until),timeNs(demand.source_stamp)+200000000LL);
  EXPECT_EQ(timeNs(demand.body_source_stamp),100000000000LL);
  // Command generation does not pretend the body was sampled on this tick.
  EXPECT_NE(demand.source_stamp,demand.body_source_stamp);
}
TEST(CommandPipelineBudget, PreparedDemandStillUsesEveryOriginalConditionalCap) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true,false,.2);
  auto grant=stageWriterHandoff(gate);ASSERT_TRUE(gate.handoff(grant,100.02,10.02));
  const auto prepared=gate.preparedStep(100.03,10.03);ASSERT_TRUE(prepared);
  EXPECT_EQ(prepared->demand.valid_until,grant.transition_deadline);
  EXPECT_LE(timeNs(prepared->demand.valid_until),timeNs(prepared->demand.source_stamp)+200000000LL);
  EXPECT_EQ(prepared->entry_source_stamp,prepared->demand.body_source_stamp);
  // Retried control messages may get a new source only from a real new
  // step; an already emitted message is never restamped by the contract.
  const auto copy=*prepared;EXPECT_EQ(copy.demand.valid_until,prepared->demand.valid_until);
}
TEST(CommandPipelineBudget, InvalidUnboundBudgetCannotConstructMotionContract) {
  for(double invalid:{.099,.501,std::numeric_limits<double>::quiet_NaN()})
    EXPECT_THROW(ExecutionContract(executionConfig(),"isolated_mock",{},false,false,invalid),std::invalid_argument);
}
TEST(CommandPipelineBudget, DelayedOccupiedVerdictStillReachesTrackerWithoutRenewingCommand) {
 for(int fault=0;fault<5;++fault){ExecutionContract gate(executionConfig(),"isolated_mock",std::string(64,'f'),false,false,.2);
  const auto demand=ownStraightDemand(gate);auto feedback=occupiedFor(demand);
  auto latest=odom(100.14);ASSERT_TRUE(gate.core().receiveOdom(latest,100.14,10.14));
  feedback.check_begin=stamp(100.13);feedback.check_end=stamp(100.131);
  feedback.body_source_stamp=stamp(100.14);
  double now=100.145;
  if(fault==1){feedback.valid_until=stamp(100.144);}
  if(fault==2){feedback.demand_body_source_stamp=stamp(100.0);}
  if(fault==3){feedback.check_begin=stamp(100.025);feedback.check_end=stamp(100.026);}
  if(fault==4){now=100.221;ASSERT_TRUE(gate.core().receiveOdom(odom(now),now,10.221));}
  EXPECT_EQ(gate.motionValidation(feedback,now),fault==0)<<fault;
  if(fault==0){EXPECT_STREQ(gate.core().maneuverPhase(),"waiting_executable_entry");
   const auto actual=gate.takeBlockedEntry();ASSERT_TRUE(actual);EXPECT_EQ(*actual,feedback);
   EXPECT_EQ(actual->demand_source_stamp,demand.source_stamp);}
  else EXPECT_FALSE(gate.takeBlockedEntry());
 }
}

namespace {
wire::RouteDeliveryReceipt retiredRoute(bool native=true) {
  wire::RouteDeliveryReceipt r;r.schema_version=1;r.session_id="session";
  r.task_id="task";r.route_id="route";r.route_hash=std::string(64,'a');
  r.delivery_sequence=2;r.source_stamp=stamp(100.01);r.receipt_stamp=stamp(100.02);
  r.status=wire::RouteDeliveryReceipt::RETIRED;
  r.native_retirement_required=r.native_retirement_confirmed=native;
  r.native_generation=native?2:0;return r;
}
wire::ExecutionPermit geometryPermit(wire::ExecutionVersion v=version()) {
  auto p=permitWire(v);p.allowed=false;p.phase="preview";
  p.execution_id.clear();p.confirmation_id.clear();p.sdk_session.clear();
  p.control_epoch=p.sdk_arm_generation=0;return p;
}
}
TEST(TrackerTaskRetirement, DurableExactRetirementReleasesNeverMotionDomainWithoutOldPermit) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true);stage(gate);
  ASSERT_TRUE(gate.prepare(100.,10.)->accepted);
  ASSERT_TRUE(gate.permit(geometryPermit(),100.,10.));
  ASSERT_TRUE(gate.core().hasInstalledGeometry());
  auto next_version=version(3);next_version.task_id="next";next_version.route_id="next-route";
  next_version.route_hash=std::string(64,'d');
  auto next=geometryPermit(next_version);next.sequence=2;next.geometry_committed=false;
  // Merely receiving another goal's permit does not seize an active domain.
  ASSERT_FALSE(gate.permit(next,100.02,10.02));
  // The native/reference retirement fact closes the exact old route even when
  // the old depth-one revoked permit was missed or never published.
  ASSERT_TRUE(gate.retireRoute(retiredRoute(),100.02));
  EXPECT_FALSE(gate.core().hasInstalledGeometry());EXPECT_TRUE(gate.step(100.02,10.02).hold);
  EXPECT_TRUE(gate.retireRoute(retiredRoute(),100.03));
  ASSERT_TRUE(gate.permit(next,100.03,10.03));
  EXPECT_FALSE(gate.retireRoute(retiredRoute(),100.04));
  auto old=geometryPermit();old.sequence=3;old.source_stamp=stamp(100.04);old.valid_until=stamp(100.24);
  EXPECT_FALSE(gate.permit(old,100.04,10.04));
  gate.proposal(proposalWire(),100.04);gate.candidate(curve());
  auto late=proofWire();late.sequence=2;late.source_stamp=late.check_begin=late.check_end=stamp(100.04);
  gate.validation(late,100.04);gate.support(supportWire());
  EXPECT_FALSE(gate.prepare(100.04,10.04));
  EXPECT_TRUE(gate.step(100.04,10.04).hold);
}
TEST(TrackerTaskRetirement, UnadmittedRouteFactIsSoftwareOnlyAndRemainsDurable) {
  ExecutionContract gate(executionConfig(),"isolated_mock");
  auto p=geometryPermit();p.geometry_committed=false;
  ASSERT_TRUE(gate.permit(p,100.,10.));
  ASSERT_TRUE(gate.retireRoute(retiredRoute(false),101.));
  auto next_version=version();next_version.task_id="next";
  auto next=geometryPermit(next_version);next.geometry_committed=false;next.sequence=2;
  next.source_stamp=stamp(101.);next.valid_until=stamp(101.2);
  EXPECT_TRUE(gate.permit(next,101.,11.));
  EXPECT_TRUE(gate.step(101.,11.).hold); // Retirement grants no movement.
}
TEST(TrackerTaskRetirement, ForeignNonterminalAndMalformedFactsCannotCloseCurrentDomain) {
  for(int fault=0;fault<18;++fault) {
    ExecutionContract gate(executionConfig(),"isolated_mock");stage(gate);
    ASSERT_TRUE(gate.prepare(100.,10.)->accepted);ASSERT_TRUE(gate.permit(geometryPermit(),100.,10.));
    auto r=retiredRoute();
    switch(fault) {
      case 0:r.schema_version=0;break;
      case 1:r.session_id="foreign";break;
      case 2:r.task_id="foreign";break;
      case 3:r.route_id="foreign";break;
      case 4:r.route_hash=std::string(64,'b');break;
      case 5:r.active=true;break;
      case 6:r.status=wire::RouteDeliveryReceipt::ACCEPTED;break;
      case 7:r.status=wire::RouteDeliveryReceipt::REJECTED;break;
      case 8:r.native_retirement_confirmed=false;break;
      case 9:r.native_generation=0;break;
      case 10:r.native_retirement_required=false;break;
      case 11:r.delivery_sequence=0;break;
      case 12:r.source_stamp.nanosec=1000000000u;break;
      case 13:r.receipt_stamp=stamp(100.2);break;
      case 14:r.source_stamp=stamp(100.2);break;
      case 15:r.receipt_stamp=stamp(99.8);break;
      case 16:r.native_generation=1;break;
      case 17:r.route_hash=std::string(64,'z');break;
    }
    EXPECT_FALSE(gate.retireRoute(r,100.02))<<fault;
    EXPECT_TRUE(gate.core().hasInstalledGeometry())<<fault;
  }
}
TEST(TrackerTaskRetirement, ConfirmedNativeBarrierBeforeIngressCannotReviveQueuedOldTask) {
  ExecutionContract gate(executionConfig(),"isolated_mock",{},true);
  ASSERT_TRUE(gate.retireRoute(retiredRoute(),100.02));
  gate.proposal(proposalWire(),100.02);gate.candidate(curve());
  gate.support(supportWire());gate.validation(proofWire(),100.02);
  EXPECT_FALSE(gate.prepare(100.02,10.02));
  auto old=geometryPermit();old.geometry_committed=false;
  EXPECT_FALSE(gate.permit(old,100.02,10.02));
  auto next_version=version(3);next_version.task_id="next";next_version.route_id="next-route";
  next_version.route_hash=std::string(64,'d');
  auto next=geometryPermit(next_version);next.geometry_committed=false;next.sequence=2;
  ASSERT_TRUE(gate.permit(next,100.03,10.03));
  // A delayed old barrier is neither a new task nor a reason to close next.
  EXPECT_FALSE(gate.retireRoute(retiredRoute(),100.04));
  next.sequence=3;EXPECT_TRUE(gate.permit(next,100.04,10.04));
  EXPECT_TRUE(gate.step(100.04,10.04).hold);
}

TEST(TrackerStairSupport, NativeConsumesSourceAssociationWithoutInventingStepCapability) {
  auto wire_support=supportWire();
  wire_support.floor_id="stair_lower";wire_support.segment_kind="stair_up";
  wire_support.required_mode="stair";
  wire_support.max_support_slope_rad=wire_support.max_support_step_m=0.;
  auto s=supportEvidence(wire_support);
  const SupportIndex index(s.ground);
  auto cfg=executionConfig();cfg.geometry_authority=GeometryAuthority::NativeScan;
  EXPECT_TRUE(supportEvidenceValid(s,index,s.identity,s.generation,cfg));
  cfg.geometry_authority=GeometryAuthority::LegacyTracker;
  EXPECT_FALSE(supportEvidenceValid(s,index,s.identity,s.generation,cfg));
  cfg.geometry_authority=GeometryAuthority::NativeScan;
  for(int fault=0;fault<6;++fault) {
    auto bad=s;
    if(fault==0)bad.verified=false;
    if(fault==1)bad.mode="general";
    if(fault==2)bad.floor_id="floor1";
    if(fault==3)bad.segment_kind="floor";
    if(fault==4)bad.generation++;
    if(fault==5)bad.max_step=.3; // not a provenance-only stair packet
    EXPECT_FALSE(supportEvidenceValid(bad,index,s.identity,s.generation,cfg))<<fault;
  }
}

TEST(TrackerStairSupport, MixedPortalConsumesCompleteSourceRegionsNotFlatStepLimits) {
  auto message=supportWire();
  message.reason="original_transition_surface_returns_associated";
  message.max_support_slope_rad=message.max_support_step_m=0.;
  wire::SupportRegion floor;floor.segment_id=message.version.segment_id;floor.floor_id="floor1";
  floor.segment_kind="floor";floor.required_mode="general";floor.begin_index=0;floor.end_index=20;
  wire::SupportRegion stair;stair.segment_id="stair_lower";stair.floor_id="stair_lower";
  stair.segment_kind="stair_up";stair.required_mode="stair";stair.begin_index=20;
  stair.end_index=message.support_ground_xyz.size();message.regions={floor,stair};
  auto cfg=executionConfig();cfg.geometry_authority=GeometryAuthority::NativeScan;
  for(int fault=-1;fault<9;++fault) {
    auto changed=message;
    if(fault==0)changed.regions.clear();
    if(fault==1)changed.regions[1].begin_index++;
    if(fault==2)changed.regions[1].end_index--;
    if(fault==3)changed.regions[1].required_mode="general";
    if(fault==4)changed.regions[0].segment_id="foreign";
    if(fault==5)changed.reason="verified";
    if(fault==6)changed.max_support_step_m=.3;
    if(fault==7)changed.verified=false;
    if(fault==8)changed.regions[1].segment_id=changed.regions[0].segment_id;
    const auto source=supportEvidence(changed);const SupportIndex index(source.ground);
    EXPECT_EQ(supportEvidenceValid(source,index,source.identity,source.generation,cfg),fault==-1)<<fault;
    auto legacy=cfg;legacy.geometry_authority=GeometryAuthority::LegacyTracker;
    EXPECT_FALSE(supportEvidenceValid(source,index,source.identity,source.generation,legacy))<<fault;
  }
  // Existing positive-threshold floor packets retain their original behavior.
  const auto old=supportEvidence(supportWire());const SupportIndex index(old.ground);
  EXPECT_TRUE(supportEvidenceValid(old,index,old.identity,old.generation,cfg));
}

TEST(TrackerStairSupport, OtherFloorFinalXYDoesNotBrakeAnIntermediateNativeCurve) {
  for(bool forward:{false,true}) {
    auto c=executionConfig();c.geometry_authority=GeometryAuthority::NativeScan;
    c.motion_model="holonomic";c.align_then_forward=forward;c.holonomic_yaw_policy="path_tangent";
    TrackerCore core(c);auto goal=task();goal.goal={0.,0.,3.55};
    ASSERT_TRUE(core.receiveOdom(odom(),100.,10.));
    ASSERT_TRUE(core.admitRevision(goal,curve(),supportEvidence(supportWire()),100.,10.,true))<<core.candidateReason();
    const auto output=core.step(100.05,10.05);
    EXPECT_GT(output.forward,0.);EXPECT_FALSE(core.controlDiagnostic().final_endpoint);
    EXPECT_GT(core.controlDiagnostic().speed_cap,0.);
    EXPECT_LE(output.forward,c.max_acceleration*.05+1e-10);
  }
}

namespace {
wire::SupportReference stairSteeringWire(double begin=-1.,double end=6.,double grade=.3,double elevation=0.) {
  auto s=supportWire();s.stair_model_hash=std::string(64,'d');
  wire::StairSurfaceModel surface;surface.surface_id="flight-surface";surface.segment_id="stair-A";
  surface.kind="continuous_support_envelope";surface.pose.orientation.w=1.;surface.pose.position.z=elevation;
  surface.support_grade=grade;surface.height_association_m=.18;surface.contact_tolerance_m=.02;
  for(auto xy:{Eigen::Vector2d(begin,-.8),Eigen::Vector2d(end,-.8),
      Eigen::Vector2d(end,.8),Eigen::Vector2d(begin,.8)}) {
    geometry_msgs::msg::Point p;p.x=xy.x();p.y=xy.y();surface.polygon.push_back(p);
  }
  s.stair_model_surfaces.push_back(surface);return s;
}
Config stairSteeringConfig() {
  auto c=executionConfig();c.geometry_authority=GeometryAuthority::NativeScan;
  c.motion_model="holonomic";c.align_then_forward=true;c.holonomic_yaw_policy="path_tangent";
  c.min_lookahead_distance_m=.4;c.max_lookahead_distance_m=.8;c.lookahead=1.2;
  c.kp_yaw=.9;c.yaw_damping_gain=.4;c.steering_error_scale_rad=.02;
  c.max_jerk=.7;c.max_yaw_jerk=4.;return c;
}
Trajectory stairSteeringCurve(double grade=.3,double ripple=0.) {
  auto t=curve();t.points.clear();t.knots.clear();
  for(int i=0;i<103;++i) {
    const double x=(i-1)*.05;t.points.emplace_back(x,ripple*std::sin(5.*x),.55+grade*x);
  }
  for(int i=0;i<107;++i)t.knots.push_back((i-3)*.25);
  certify(t);return t;
}
Odom stairSteeringOdom(const Trajectory& t,double yaw=0.) {
  auto body=odom();body.position=t.join_position;body.velocity_in_frame=t.join_velocity;
  body.planar_speed=body.velocity_in_frame.head<2>().norm();body.yaw=yaw;
  body.orientation=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ());return body;
}
}

TEST(TrackerStairSteering, DecoderRetainsFiniteGeometryWithoutChangingObservationOrSource) {
  auto wire=stairSteeringWire();const auto s=supportEvidence(wire);
  ASSERT_TRUE(s.model_geometry_valid);ASSERT_EQ(s.steering_surfaces.size(),1U);
  EXPECT_EQ(s.steering_surfaces[0].segment,"stair-A");
  EXPECT_EQ(s.steering_surfaces[0].polygon.size(),4U);
  EXPECT_DOUBLE_EQ(s.steering_surfaces[0].grade,.3);
  EXPECT_DOUBLE_EQ(s.source_stamp,100.);EXPECT_EQ(s.verified,wire.verified);
  auto changed=s;changed.steering_surfaces[0].origin.x()+=.01;
  EXPECT_FALSE(sameSupportEvidenceGeometry(s,changed));
  wire.stair_model_surfaces[0].pose.orientation.w=0.;
  const auto malformed=supportEvidence(wire);EXPECT_FALSE(malformed.model_geometry_valid);
  EXPECT_TRUE(malformed.steering_surfaces.empty());
}

TEST(TrackerStairSteering, FiniteXYZAssociationDistinguishesFloorsAndRespectsLandingBoundary) {
  auto wire=stairSteeringWire();auto landing=wire.stair_model_surfaces[0];
  landing.surface_id="landing";landing.kind="landing";landing.support_grade=0.;
  landing.polygon[0].x=landing.polygon[3].x=-1.;landing.polygon[1].x=landing.polygon[2].x=0.;
  wire.stair_model_surfaces.push_back(landing);
  auto upper=wire.stair_model_surfaces[0];upper.surface_id="upper";upper.segment_id="stair-B";upper.pose.position.z=3.;
  wire.stair_model_surfaces.push_back(upper);
  const auto s=supportEvidence(wire);ASSERT_TRUE(s.model_geometry_valid);
  const auto query=[&](const Eigen::Vector3d& p){return stairSteeringSurfaceAt(s.steering_surfaces,p,.55);};
  ASSERT_TRUE(query({1.,0.,.85}));EXPECT_EQ(*query({1.,0.,.85}),0U);
  ASSERT_TRUE(query({1.,0.,3.85}));EXPECT_EQ(*query({1.,0.,3.85}),2U);
  EXPECT_FALSE(query({1.,0.,2.}));EXPECT_FALSE(query({1.,.81,.85}));
  ASSERT_TRUE(query({0.,0.,.55}));EXPECT_EQ(*query({0.,0.,.55}),1U);
  ASSERT_TRUE(query({1.,.8,.85})); // The actual polygon edge is retained.
}

TEST(TrackerStairSteering, EntranceAndExitWeightsAreContinuousAndDoNotCrossTheNextFlight) {
  const std::vector<StairFlightRange> ranges{{"A",1.,4.,false},{"B",5.,8.,true}};
  const auto far=stairSteeringPreview(ranges,0.,.4,.98);EXPECT_DOUBLE_EQ(far.weight,0.);
  const auto entry=stairSteeringPreview(ranges,.5,.4,.98);EXPECT_GT(entry.weight,0.);EXPECT_LT(entry.weight,1.);
  for(double boundary:{.02,1.,4.}) {
    const auto before=stairSteeringPreview(ranges,boundary-1e-7,.4,.98);
    const auto after=stairSteeringPreview(ranges,boundary+1e-7,.4,.98);
    EXPECT_NEAR(before.weight,after.weight,1e-5)<<boundary;
  }
  const auto exit=stairSteeringPreview(ranges,3.8,.4,.98);
  EXPECT_LE(exit.horizontal,4.);EXPECT_LT(exit.weight,.1);
  const auto tail=stairSteeringPreview({{"A",0.,3.,true}},2.8,.4,.98);
  EXPECT_DOUBLE_EQ(tail.weight,1.); // Sliding-window tail is not a real stair exit.
  const std::vector<StairFlightRange> narrow_landing{{"A",1.,4.,false},{"B",4.2,8.,true}};
  EXPECT_NEAR(stairSteeringPreview(narrow_landing,4.-1e-7,.4,.98).weight,
    stairSteeringPreview(narrow_landing,4.+1e-7,.4,.98).weight,1e-8);
}

TEST(TrackerStairSteering, WindowEndingAtPhysicalExitDoesNotBecomeAnInteriorFlightTail) {
  const auto s=supportEvidence(stairSteeringWire(-1.,2.,.3));
  for(double end:{1.7,2.}) {
    std::vector<Eigen::Vector3d> points;std::vector<double> horizontal;
    for(int i=0;i<=100;++i){double x=end*i/100.;points.emplace_back(x,0.,.55+.3*x);horizontal.push_back(x);}
    const auto ranges=stairCurveFlightRanges(s.steering_surfaces,points,horizontal,.55);
    ASSERT_EQ(ranges.size(),1U);ASSERT_TRUE(ranges[0].ends_at_curve_tail);
    EXPECT_NEAR(ranges[0].tail_exit_distance,2.-end,1e-10);
    EXPECT_NEAR(stairSteeringPreview(ranges,1.5,.4,.98).weight,stairBlend(.5/.98),1e-10);
  }
  auto surface=s.steering_surfaces[0];surface.kind="unknown";
  EXPECT_TRUE(stairCurveFlightRanges({surface},{{0.,0.,.55},{1.,0.,.85}},{0.,1.},.55).empty());
  int calls=0;
  EXPECT_TRUE(stairCurveFlightRanges(s.steering_surfaces,{{0.,0.,.55},{1.,0.,.85}},
    {0.,1.},.55,[&] {return ++calls<2;}).empty());
  EXPECT_EQ(calls,2);
}

TEST(TrackerStairSteering, UpAndDownUseOnlyTheAcceptedCurveAndLongFlightPreview) {
  for(double grade:{-.3,.3})for(double ripple:{-.025,.025}) {
    const auto c=stairSteeringConfig();auto t=stairSteeringCurve(grade,ripple);
    auto s=supportEvidence(stairSteeringWire(-1.,6.,grade));auto b=stairSteeringOdom(t);
    auto goal=task();goal.goal={7.,0.,.55+grade*7.};TrackerCore core(c);
    ASSERT_TRUE(core.receiveOdom(b,100.,10.));
    ASSERT_TRUE(core.admitRevision(goal,t,s,100.,10.,true))<<core.candidateReason();
    const auto output=core.step(100.05,10.05);const auto& d=core.controlDiagnostic();
    ASSERT_FALSE(output.frozen)<<output.reason;EXPECT_GT(output.forward,0.);EXPECT_DOUBLE_EQ(output.lateral,0.);
    EXPECT_EQ(d.steering_curvature_source,"accepted_stair_flight_preview");
    EXPECT_DOUBLE_EQ(d.steering_curvature,0.);
    EXPECT_LT(std::abs(d.heading_error),.035); // Short tangent is about .12 rad.
    EXPECT_DOUBLE_EQ(core.progressTime(),0.);EXPECT_DOUBLE_EQ(core.odometry().stamp,100.);
  }
}

TEST(TrackerStairSteering, PlainFloorAndOtherFloorModelHaveIdenticalOutputs) {
  const auto c=stairSteeringConfig();auto t=stairSteeringCurve(0.,.025);
  const auto plain=supportEvidence(supportWire());const auto remote=supportEvidence(stairSteeringWire(-1.,6.,.3,3.));
  TrackerCore a(c),b(c);auto goal=task();goal.goal={7.,0.,.55};const auto initial=stairSteeringOdom(t);
  ASSERT_TRUE(a.receiveOdom(initial,100.,10.));ASSERT_TRUE(b.receiveOdom(initial,100.,10.));
  ASSERT_TRUE(a.admitRevision(goal,t,plain,100.,10.,true));ASSERT_TRUE(b.admitRevision(goal,t,remote,100.,10.,true));
  for(int tick=1;tick<=30;++tick) {
    auto state=initial;state.stamp=state.posterior_stamp=state.imu_stamp=100.+tick*.05;
    ASSERT_TRUE(a.receiveOdom(state,state.stamp,state.stamp-90.));ASSERT_TRUE(b.receiveOdom(state,state.stamp,state.stamp-90.));
    a.refreshTaskLease(state.stamp-90.);b.refreshTaskLease(state.stamp-90.);
    const auto oa=a.step(state.stamp,state.stamp-90.),ob=b.step(state.stamp,state.stamp-90.);
    EXPECT_DOUBLE_EQ(oa.forward,ob.forward);EXPECT_DOUBLE_EQ(oa.lateral,ob.lateral);EXPECT_DOUBLE_EQ(oa.yaw_rate,ob.yaw_rate);
    EXPECT_EQ(a.controlDiagnostic().steering_curvature_source,"accepted_curve_spatial_preview");
  }
}

TEST(TrackerStairSteering, LateralRecoveryAndRealBendRemainOnAcceptedCurveWithoutSidewaysCommand) {
  const auto c=stairSteeringConfig();
  for(double side:{-1.,1.}) {
    auto t=stairSteeringCurve();auto support=supportEvidence(stairSteeringWire());
    auto body=stairSteeringOdom(t);auto goal=task();goal.goal={7.,0.,2.65};TrackerCore core(c);
    ASSERT_TRUE(core.receiveOdom(body,100.,10.));
    ASSERT_TRUE(core.admitRevision(goal,t,support,100.,10.,true));
    body.stamp=body.posterior_stamp=body.imu_stamp=100.05;body.position.y()=side*.05;
    ASSERT_TRUE(core.receiveOdom(body,100.05,10.05));core.refreshTaskLease(10.05);
    auto output=core.step(100.05,10.05,true);
    ASSERT_FALSE(output.frozen);EXPECT_GT(output.forward,0.);EXPECT_DOUBLE_EQ(output.lateral,0.);
    EXPECT_LT(side*core.controlDiagnostic().yaw_target,0.);
    EXPECT_LT(std::abs(core.controlDiagnostic().heading_error),.05);
  }
  auto t=stairSteeringCurve();for(auto& p:t.points)p.y()=.045*p.x()*p.x();certify(t);
  auto support=supportEvidence(stairSteeringWire());auto body=stairSteeringOdom(t);
  auto goal=task();goal.goal={7.,2.,2.65};TrackerCore core(c);
  ASSERT_TRUE(core.receiveOdom(body,100.,10.));ASSERT_TRUE(core.admitRevision(goal,t,support,100.,10.,true));
  const auto output=core.step(100.05,10.05,true);
  EXPECT_GT(core.controlDiagnostic().yaw_target,.01);EXPECT_DOUBLE_EQ(output.lateral,0.);
}

TEST(TrackerStairSteering, RetainedPhysicalExitSurvivesExactPrefixRemovalInBothDirections) {
  for(double direction:{-1.,1.}) {
    // Two finite flights and a short landing; reverse the coordinates for
    // downhill travel. No other floor or extrapolated point participates.
    StairFlightRange previous{"A",1.,4.,false},next{"B",4.5,8.,true};
    previous.exit_point={direction*4.,0.,direction*1.2};previous.has_exit_point=true;
    previous.direction=next.direction={direction,0.};
    StairSteeringHistory history;
    const double since=history.precedingExitDistance({previous,next},4.3,.98,
      {direction*4.3,0.,direction*1.2});
    EXPECT_NEAR(since,.3,1e-12);
    const auto full=stairSteeringPreview({previous,next},4.3,.4,.98,since);
    next.begin-=4.2;next.end-=4.2;
    const double after=history.precedingExitDistance({next},.1,.98,
      {direction*4.3,0.,direction*1.2});
    const auto suffix=stairSteeringPreview({next},.1,.4,.98,after);
    EXPECT_NEAR(suffix.weight,full.weight,1e-12);
    EXPECT_NEAR(suffix.horizontal+4.2,full.horizontal,1e-12);
    // The stored exit is not a waypoint: all targets stay on the new curve.
    EXPECT_LE(suffix.horizontal,next.end);
    EXPECT_NEAR(history.precedingExitDistance({next},.4,.98,
      {direction*4.6,0.,direction*1.23}),.6,1e-12);
    // First observation on a landing has no manufactured prior flight.
    StairSteeringHistory fresh;
    EXPECT_TRUE(std::isinf(fresh.precedingExitDistance({next},.1,.98,
      {direction*4.3,0.,direction*1.2})));
    history.reset();
    EXPECT_TRUE(std::isinf(history.precedingExitDistance({next},.1,.98,
      {direction*4.3,0.,direction*1.2})));
  }
}

TEST(TrackerStairSteering, PhysicalExitContextCannotCrossFlightOrReverseDirection) {
  StairFlightRange previous{"A",1.,4.,false},next{"B",4.5,8.,true};
  previous.exit_point={4.,0.,1.2};previous.has_exit_point=true;
  previous.direction=next.direction={1.,0.};
  for(bool reverse:{false,true}) {
    StairSteeringHistory history;
    EXPECT_NEAR(history.precedingExitDistance({previous,next},4.3,.98,{4.3,0.,1.2}),.3,1e-12);
    auto changed=next;changed.begin-=4.2;changed.end-=4.2;
    if(reverse)changed.direction={-1.,0.};else changed.segment="other-flight";
    EXPECT_TRUE(std::isinf(history.precedingExitDistance({changed},.1,.98,{4.3,0.,1.2})));
  }
}

TEST(TrackerStairSteering, ExitPersistsWhenNextFlightWasOutsidePreviousFiniteWindow) {
  StairFlightRange previous{"A",1.,4.,false},next{"B",.3,3.8,true};
  previous.exit_point={4.,0.,1.2};previous.has_exit_point=true;
  previous.direction=next.direction={1.,0.};
  for(bool exact_exit_tail:{false,true}) {
    auto exit=previous;
    if(exact_exit_tail){exit.ends_at_curve_tail=true;exit.tail_exit_distance=0.;}
    StairSteeringHistory history;
    EXPECT_TRUE(std::isinf(history.precedingExitDistance({exit},exact_exit_tail?4.:4.2,.98,
      {exact_exit_tail?4.:4.2,0.,1.2})));
    // A later accepted window contains only the next flight. The already
    // observed predecessor was not part of this new spline at any point.
    EXPECT_NEAR(history.precedingExitDistance({next},.1,.98,{4.3,0.,1.2}),.3,1e-12);
  }
  previous.ends_at_curve_tail=true;previous.tail_exit_distance=.5;
  StairSteeringHistory interior_tail;
  EXPECT_TRUE(std::isinf(interior_tail.precedingExitDistance({previous},4.,.98,{4.,0.,1.2})));
  EXPECT_TRUE(std::isinf(interior_tail.precedingExitDistance({next},.1,.98,{4.3,0.,1.2})));
}

namespace {
SupportEvidence shortLandingSupport() {
  auto s=supportEvidence(stairSteeringWire(-1.,4.,.3));
  auto landing=s.steering_surfaces.front();landing.id="landing";landing.segment="platform";
  landing.kind="landing";landing.grade=0.;landing.intercept=1.2;
  landing.polygon={{4.,-.8},{4.5,-.8},{4.5,.8},{4.,.8}};
  auto next=landing;next.id="second";next.segment="stair-B";
  next.kind="continuous_support_envelope";next.grade=.3;next.intercept=-.15;
  next.polygon={{4.5,-.8},{10.,-.8},{10.,.8},{4.5,.8}};
  s.steering_surfaces.push_back(landing);s.steering_surfaces.push_back(next);return s;
}
Trajectory shortLandingCurve(int first,int id,double stamp,double join) {
  auto t=curve(id,stamp);t.points.clear();t.knots.clear();
  t.body_heading_seed_yaw=0.;t.body_heading_sampling_intervals=1000;
  for(int i=first;i<203;++i) {
    const double x=(i-1)*.05,z=x<=4.?.3*x:(x<=4.5?1.2:1.2+.3*(x-4.5));
    t.points.emplace_back(x,.04*std::sin(3.*x),z+.55);
  }
  for(std::size_t i=0;i<t.points.size()+4;++i)t.knots.push_back((double(i)-3.)*.25);
  certify(t,stamp,join);
  const auto cache=EntryCurveCache::build(t);
  if(cache)t.valid_start_arc_length=curveArcAt(cache->times,cache->arcs,join);
  return t;
}
}

TEST(TrackerStairSteering, ProductionWindowCommitPreservesExitButAnchorModelAndTaskResetIt) {
  const auto c=stairSteeringConfig();auto full=shortLandingCurve(0,1,100.,21.25);
  auto s=shortLandingSupport();auto b=stairSteeringOdom(full);auto goal=task();goal.goal={12.,0.,4.};
  TrackerCore incumbent(c);
  ASSERT_TRUE(incumbent.receiveOdom(b,100.,10.));
  ASSERT_TRUE(incumbent.admitRevision(goal,full,s,100.,10.,true,true))<<incumbent.candidateReason();
  ASSERT_FALSE(incumbent.step(100.05,10.05,true).frozen);
  const double inherited_preview=incumbent.controlDiagnostic().steering_preview_distance;
  ASSERT_LT(inherited_preview,.6);
  for(int variant:{0,1,2,3}) {
    auto core=incumbent;auto candidate=shortLandingCurve(84,2,100.1,.25);
    auto next_goal=goal;auto next_support=s;
    if(variant==1)++next_goal.identity.anchor_revision;
    if(variant==2)next_support.model_hash=std::string(64,'e');
    if(variant==3){core.cancel("new_task");next_goal.identity.task_id="another-task";}
    candidate.identity=next_goal.identity;next_support.identity=next_goal.identity;
    auto body=b;body.stamp=body.posterior_stamp=body.imu_stamp=100.1;
    ASSERT_TRUE(core.receiveOdom(body,100.1,10.1));
    ASSERT_TRUE(core.admitRevision(next_goal,candidate,next_support,100.1,10.1,true,true))<<core.candidateReason();
    core.refreshTaskLease(10.1);core.step(100.1,10.1,true);
    // Cancellation has its independent one-tick stop; neither this scope
    // check nor the stair bias authorizes resurrection of the old command.
    if(variant==3)core.step(100.15,10.15,true);
    const auto preview=core.controlDiagnostic().steering_preview_distance;
    if(variant==0)EXPECT_NEAR(preview,inherited_preview,1e-4);
    else EXPECT_GT(preview,1.);
    EXPECT_DOUBLE_EQ(core.step(100.15,10.15,true).lateral,0.);
  }
}

TEST(TrackerStairSteering, PreparedRefreshAndRejectedGeometryRetainLiveFiniteExitContext) {
  const auto c=stairSteeringConfig();auto full=shortLandingCurve(0,1,100.,21.25);
  auto s=shortLandingSupport();auto b=stairSteeringOdom(full);auto goal=task();goal.goal={12.,0.,4.};
  TrackerCore current(c);
  ASSERT_TRUE(current.receiveOdom(b,100.,10.));
  ASSERT_TRUE(current.admitRevision(goal,full,s,100.,10.,true,true));
  // Stage before the current controller has observed the predecessor exit.
  auto prepared=current;auto candidate=shortLandingCurve(84,2,100.,.25);
  ASSERT_TRUE(prepared.admitRevision(goal,candidate,s,100.,10.,true,true));
  ASSERT_FALSE(current.step(100.05,10.05,true).frozen);
  auto body=b;body.stamp=body.posterior_stamp=body.imu_stamp=100.1;
  ASSERT_TRUE(current.receiveOdom(body,100.1,10.1));
  ASSERT_TRUE(prepared.refreshPreparedState(current,100.1,10.1));
  ASSERT_FALSE(prepared.step(100.1,10.1,true).frozen);
  EXPECT_LT(prepared.controlDiagnostic().steering_preview_distance,.6);
  ASSERT_TRUE(prepared.rollbackGeometry(current,100.1));
  candidate.id=3;candidate.start_time=candidate.join_source_stamp=100.1;
  ASSERT_TRUE(prepared.admitRevision(goal,candidate,s,100.1,10.1,true,true));
  ASSERT_FALSE(prepared.step(100.15,10.15,true).frozen);
  EXPECT_LT(prepared.controlDiagnostic().steering_preview_distance,.6);
}

TEST(TrackerStairSteering, DelayedCommittedAckPromotesGeometryWithoutRewindingLiveExitContext) {
  const auto c=stairSteeringConfig();auto full=shortLandingCurve(0,1,100.,21.25);
  auto s=shortLandingSupport();auto b=stairSteeringOdom(full);auto goal=task();goal.goal={12.,0.,4.};
  TrackerCore live(c);
  ASSERT_TRUE(live.receiveOdom(b,100.,10.));
  ASSERT_TRUE(live.admitRevision(goal,full,s,100.,10.,true,true));
  // Candidate receipt predates the first actual stair control tick. The
  // frozen accepted_core therefore has no exit history yet.
  auto accepted=live;auto candidate=shortLandingCurve(84,2,100.,.25);
  ASSERT_TRUE(accepted.admitRevision(goal,candidate,s,100.,10.,true,true));
  ASSERT_FALSE(live.step(100.05,10.05,true).frozen);
  ASSERT_LT(live.controlDiagnostic().steering_preview_distance,.6);
  auto body=b;body.stamp=body.posterior_stamp=body.imu_stamp=100.1;
  ASSERT_TRUE(live.receiveOdom(body,100.1,10.1));
  ASSERT_TRUE(live.rollbackGeometry(accepted,100.1,true));
  ASSERT_EQ(live.trajectoryId(),2);
  ASSERT_FALSE(live.step(100.1,10.1,true).frozen);
  EXPECT_LT(live.controlDiagnostic().steering_preview_distance,.6);
  EXPECT_DOUBLE_EQ(live.step(100.15,10.15,true).lateral,0.);
}
