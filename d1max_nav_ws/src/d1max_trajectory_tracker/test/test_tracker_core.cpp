#include <gtest/gtest.h>
#include <limits>
#include <string>
#include "d1max_trajectory_tracker/tracker_core.hpp"
#include "d1max_trajectory_tracker/control_contract.hpp"

using namespace d1max_trajectory_tracker;

namespace {
// Legacy fixtures exercise historical controller geometry only; production
// default requires full v2 route/segment/anchor identity and remains blocked.
Config config() { Config c; c.session_id = "test-session"; c.planning_frame="d1max_loc_map";
  c.require_versioned_identity=false; return c; }
Task task(std::uint64_t generation = 1) {
  Task t; t.session_id="test-session"; t.frame_id="d1max_loc_map"; t.generation=generation;
  t.active=true; t.issued_at=100.; t.goal={2.,0.,.55}; return t;
}
Odom odom(double stamp = 100.0) {
  Odom o; o.frame_id="d1max_loc_map"; o.child_frame_id="d1max_loc_base_link";
  o.stamp=stamp; o.position={0.,0.,.55}; return o;
}
Trajectory trajectory(std::uint64_t generation = 1, std::int64_t id = 1) {
  Trajectory t;
  t.session_id = "test-session";
  t.frame_id = "d1max_loc_map";
  t.generation = generation;
  t.id = id;
  t.start_time = 100.0;
  for (int i = 0; i < 23; ++i) t.points.emplace_back((i - 1) * .05, 0, .55);
  for (int i = 0; i < 27; ++i) t.knots.push_back((i - 3) * .2);
  return t;
}
void prepare(TrackerCore &core) {
  ASSERT_TRUE(core.receiveTask(task(), 100, 10));
  ASSERT_TRUE(core.receiveOdom(odom(), 100, 10));
  ASSERT_TRUE(core.receiveTrajectory(trajectory(), 100, 10));
}
}

TEST(Tracker, DoesNotStartWithoutExplicitTask) {
  TrackerCore core(config());
  EXPECT_FALSE(core.receiveTrajectory(trajectory(), 100, 10));
  core.receiveOdom(odom(), 100, 10);
  EXPECT_DOUBLE_EQ(core.step(100, 10).forward, 0);
  EXPECT_FALSE(core.active());
  EXPECT_EQ(core.reason(), "idle");
}

TEST(Tracker, RuntimeRequiresVersionedControlContractEvenIfStandalone) {
  EXPECT_THROW(requireIntegratedControlContract(), std::invalid_argument);
}

TEST(Tracker, OutOfOrderAndDuplicateOdomCannotRewindOrRefreshSource) {
  TrackerCore core(config()); prepare(core);
  ASSERT_TRUE(core.receiveOdom(odom(100.1), 100.1, 10.1));
  auto delayed = odom(100.05);
  delayed.position.x() = 1.5;
  EXPECT_FALSE(core.receiveOdom(delayed, 100.15, 10.15));
  EXPECT_FALSE(core.receiveOdom(odom(100.1), 100.2, 10.2));
  core.step(100.2, 10.2);
  core.receiveTask(task(), 100.3, 10.3);
  EXPECT_GT(core.step(100.3, 10.3).forward, 0.);
  core.receiveTask(task(), 100.51, 10.51);
  EXPECT_EQ(core.step(100.51, 10.51).reason, "odometry_stale");
}

TEST(Tracker, LegacyMapCorrectionChangesCommandWithoutPhysicalMotion) {
  // Evidence for the runtime architecture blocker: both bodies are stationary
  // in local odom, but applying only a map correction to one state creates a
  // spurious steering error against its still-old map trajectory.
  TrackerCore stationary(config()), corrected(config());
  prepare(stationary); prepare(corrected);
  stationary.step(100.05, 10.05); corrected.step(100.05, 10.05);
  auto same = odom(100.1);
  ASSERT_TRUE(stationary.receiveOdom(same, 100.1, 10.1));
  same.position.y() = .2;
  ASSERT_TRUE(corrected.receiveOdom(same, 100.1, 10.1));
  EXPECT_DOUBLE_EQ(stationary.step(100.1, 10.1).yaw_rate, 0.);
  EXPECT_LT(corrected.step(100.1, 10.1).yaw_rate, 0.);
}

TEST(Tracker, UsesVendorCurveAndOutputsBoundedSI) {
  TrackerCore core(config()); prepare(core);
  const auto out = core.step(100.05, 10.05);
  EXPECT_NEAR(out.forward, .0175, 1e-9);  // .35 m/s² × .05s
  EXPECT_NEAR(out.yaw_rate, 0, 1e-9);
  EXPECT_FALSE(out.frozen);
}
TEST(Tracker, ControlDiagnosticsDistinguishClockBranchesAndDoNotAdvanceTheController) {
  TrackerCore core(config());prepare(core);
  const auto initial=core.controlDiagnostic();
  EXPECT_FALSE(initial.step_seen);ASSERT_TRUE(initial.geometry_available);
  const auto output=core.step(100.05,10.05);
  const auto before=core.progress(100.05);
  TrackerCore untouched=core;
  for(int i=0;i<8;++i) {
    const auto d=core.controlDiagnostic();
    EXPECT_TRUE(d.step_seen);EXPECT_FALSE(d.duplicate_source);EXPECT_FALSE(d.before_trajectory_start);
    EXPECT_EQ(d.step_source_ns,100050000000LL);
    EXPECT_NEAR(d.projected_curve_time,before.curve_time,1e-12);
    EXPECT_NEAR(d.lookahead_curve_time,std::min(d.curve_duration,before.curve_time+config().lookahead),1e-12);
    EXPECT_TRUE(d.lookahead_velocity.allFinite());EXPECT_GT(d.planar_speed_limit_mps,0.);
    EXPECT_DOUBLE_EQ(core.progress(100.05).curve_time,before.curve_time);
  }
  const auto next=core.step(100.1,10.1),plain=untouched.step(100.1,10.1);
  EXPECT_DOUBLE_EQ(next.forward,plain.forward);EXPECT_DOUBLE_EQ(next.yaw_rate,plain.yaw_rate);
  EXPECT_GT(output.forward,0.);
  EXPECT_EQ(core.step(100.1,10.12).reason,"waiting_trajectory_clock");
  EXPECT_TRUE(core.controlDiagnostic().duplicate_source);
  EXPECT_FALSE(core.controlDiagnostic().before_trajectory_start);

  TrackerCore future(config());
  ASSERT_TRUE(future.receiveTask(task(),100.,10.));
  auto body=odom();body.velocity_in_frame={.03,.04,.6};
  ASSERT_TRUE(future.receiveOdom(body,100.,10.));
  auto curve=trajectory();curve.start_time=100.1;
  ASSERT_TRUE(future.receiveTrajectory(curve,100.,10.));
  EXPECT_EQ(future.step(100.05,10.05).reason,"waiting_trajectory_clock");
  const auto d=future.controlDiagnostic();
  EXPECT_FALSE(d.duplicate_source);EXPECT_TRUE(d.before_trajectory_start);
  EXPECT_EQ(d.step_trajectory_start_ns,100100000000LL);
  EXPECT_NEAR(d.measured_xy_speed_mps,.05,1e-12); // Z vibration does not become planar progress.
}
TEST(Tracker,AcceptedRouteYawToleranceRangeIsNotSilentlyRejectedAtAlignment) {
  for(double tolerance:{.01,.15,.3,.5}) {
    TrackerCore core(config());prepare(core);core.step(100.02,10.02);
    auto measured=odom(100.04);measured.posterior_stamp=measured.imu_stamp=100.04;
    ASSERT_TRUE(core.receiveOdom(measured,100.04,10.04));
    const auto out=core.align(1.,tolerance,100.04,10.04,true);
    EXPECT_EQ(out.reason,"aligning");EXPECT_GT(out.yaw_rate,0.);EXPECT_EQ(out.forward,0.);
  }
}

TEST(Tracker, SessionGenerationAndIdAreEnforced) {
  TrackerCore core(config()); prepare(core);
  auto stale = trajectory(); stale.session_id = "other";
  EXPECT_FALSE(core.receiveTrajectory(stale, 100.1, 10.1));
  stale = trajectory(2, 2);
  EXPECT_FALSE(core.receiveTrajectory(stale, 100.1, 10.1));
  EXPECT_FALSE(core.receiveTrajectory(trajectory(), 100.1, 10.1));
  EXPECT_GT(core.step(100.1, 10.1).forward, 0);
  auto next = task(2); next.issued_at = 100.1;
  EXPECT_TRUE(core.receiveTask(next, 100.1, 10.1));
  EXPECT_FALSE(core.receiveTrajectory(trajectory(1, 5), 100.15, 10.15));
  EXPECT_DOUBLE_EQ(core.step(100.15, 10.15).forward, 0);
  EXPECT_FALSE(core.receiveTrajectory(trajectory(2, 6), 100.15, 10.15)); // start before new task
  auto current = trajectory(2, 7); current.start_time = 100.1;
  EXPECT_TRUE(core.receiveTrajectory(current, 100.15, 10.15));
}

TEST(Tracker, EmptyInvalidOrWrongFrameCandidateDoesNotWithdrawIncumbent) {
  for (int variant = 0; variant < 7; ++variant) {
    TrackerCore core(config()); prepare(core);
    auto bad = trajectory(1, 2);
    if (variant == 0) bad.points.clear();
    if (variant == 1) bad.points[1].x() = std::numeric_limits<double>::quiet_NaN();
    if (variant == 2) bad.knots.pop_back();
    if (variant == 3) bad.knots[3] = bad.knots[2];
    if (variant == 4) bad.frame_id = "odom";
    if (variant == 5) bad.start_time = 101;
    if (variant == 6) bad.order = 1;
    EXPECT_FALSE(core.receiveTrajectory(bad, 100.05, 10.05));
    EXPECT_GT(core.step(100.05, 10.05).forward, 0);
    EXPECT_EQ(core.trajectoryId(),1);
  }
}

TEST(Tracker, CandidateDiagnosticsSeparateFormatClockAndGeometryWithoutChangingAdmission) {
  struct Case {const char* reason;int variant;};
  const Case cases[]={{"trajectory_clock_nonfinite",0},{"trajectory_clock_nonfinite",1},
    {"trajectory_clock_nonfinite",2},{"trajectory_frame_mismatch",3},
    {"trajectory_predates_task",4},{"trajectory_start_in_future",5},
    {"trajectory_start_expired",6},{"trajectory_order_unsupported",7},
    {"trajectory_control_point_count",8},{"trajectory_control_point_count",9},
    {"trajectory_knot_count",10},{"trajectory_control_point_nonfinite",11},
    {"trajectory_knot_nonfinite",12},{"trajectory_knots_not_increasing",13},
    {"trajectory_duration_out_of_range",14},{"trajectory_duration_out_of_range",15},
    {"trajectory_duration_elapsed",16},{"trajectory_arc_degenerate",17}};
  for(const auto& entry:cases) {
    auto cfg=config();cfg.trajectory_timeout=entry.variant==6?2.:5.;TrackerCore core(cfg);
    ASSERT_TRUE(core.receiveTask(task(),100.,10.));
    ASSERT_TRUE(core.receiveOdom(odom(),100.,10.));
    auto bad=trajectory();double now=100.,receipt=10.;
    const double nan=std::numeric_limits<double>::quiet_NaN();
    switch(entry.variant) {
      case 0:now=nan;break;
      case 1:receipt=nan;break;
      case 2:bad.start_time=nan;break;
      case 3:bad.frame_id="other";break;
      case 4:bad.start_time=99.99;break;
      case 5:bad.start_time=100.201;break;
      case 6:now=102.001;receipt=12.001;break;
      case 7:bad.order=1;break;
      case 8:bad.points.resize(3);break;
      case 9:bad.points.resize(10001);break;
      case 10:bad.knots.pop_back();break;
      case 11:bad.points[1].x()=nan;break;
      case 12:bad.knots[5]=nan;break;
      case 13:bad.knots[5]=bad.knots[4];break;
      case 14:for(auto& knot:bad.knots)knot*=.001;break;
      case 15:for(auto& knot:bad.knots)knot*=100.;break;
      case 16:now=104.001;receipt=14.001;break;
      case 17:for(auto& point:bad.points)point={0.,0.,.55};break;
    }
    EXPECT_FALSE(core.receiveTrajectory(bad,now,receipt))<<entry.variant;
    EXPECT_EQ(core.candidateReason(),entry.reason)<<entry.variant;
    EXPECT_TRUE(core.active())<<entry.variant; // Invalid candidate is not task cancellation.
    EXPECT_EQ(core.trajectoryId(),-1)<<entry.variant;
  }
}

TEST(Tracker, ExpiredPendingCandidateDoesNotWithdrawOrRestampAcceptedCurve) {
  auto cfg=config();cfg.trajectory_timeout=2.;TrackerCore core(cfg);prepare(core);
  const auto accepted_id=core.trajectoryId();
  auto bad=trajectory(1,2);
  EXPECT_FALSE(core.receiveTrajectory(bad,102.001,12.001));
  EXPECT_EQ(core.candidateReason(),"trajectory_start_expired");
  EXPECT_EQ(core.trajectoryId(),accepted_id);
  EXPECT_TRUE(core.active());
  // No replacement was installed and no source was republished to grant it a
  // fresh lease; independent odometry/proof checks still control actual motion.
}

TEST(Tracker, OdomFramesQuaternionAndSpeedCannotBypassSafety) {
  for (int variant = 0; variant < 4; ++variant) {
    TrackerCore core(config()); prepare(core);
    auto bad = odom(100.05); // a distinct fresh fault, not an ignored duplicate
    if (variant == 0) bad.frame_id = "odom";
    if (variant == 1) bad.child_frame_id = "lidar";
    if (variant == 2) bad.yaw = std::numeric_limits<double>::quiet_NaN();
    if (variant == 3) bad.planar_speed = 1.51;
    EXPECT_FALSE(core.receiveOdom(bad, 100.05, 10.05));
    EXPECT_DOUBLE_EQ(core.step(100.05, 10.05).forward, 0);
  }
}

TEST(Tracker, StaleOdometryStopsAndRevokesCurrentTrajectory) {
  TrackerCore core(config()); prepare(core);
  for (int i = 1; i <= 5; ++i) {
    core.receiveTask(task(), 100 + i*.1, 10+i*.1);
    const auto out = core.step(100+i*.1, 10+i*.1);
    if (i == 5) { EXPECT_DOUBLE_EQ(out.forward, 0); EXPECT_EQ(out.reason, "odometry_stale"); }
  }
  EXPECT_TRUE(core.receiveOdom(odom(100.55), 100.55, 10.55));
  EXPECT_DOUBLE_EQ(core.step(100.55, 10.55).forward, 0);
}

TEST(Tracker, HeartbeatTimeoutCannotResumeSameGeneration) {
  TrackerCore core(config()); prepare(core);
  for (int i=1; i<=8; ++i) {
    core.receiveOdom(odom(100+i*.1), 100+i*.1, 10+i*.1);
    core.step(100+i*.1, 10+i*.1);
  }
  EXPECT_FALSE(core.active());
  EXPECT_EQ(core.reason(), "task_heartbeat_stale");
  EXPECT_FALSE(core.receiveTask(task(), 100.85, 10.85));
  EXPECT_DOUBLE_EQ(core.step(100.85, 10.85).forward, 0);
}

TEST(Tracker, CancelCannotResumeSameGeneration) {
  TrackerCore core(config()); prepare(core);
  auto stopped = task(); stopped.active = false;
  EXPECT_TRUE(core.receiveTask(stopped, 100.05, 10.05));
  EXPECT_FALSE(core.receiveTask(task(), 100.1, 10.1));
  EXPECT_FALSE(core.receiveTrajectory(trajectory(1, 2), 100.1, 10.1));
  EXPECT_DOUBLE_EQ(core.step(100.1, 10.1).forward, 0);
}

TEST(Tracker, InvalidMutableHeartbeatIsNotAccepted) {
  TrackerCore core(config()); prepare(core);
  auto changed = task(); changed.goal.x() = 10;
  EXPECT_FALSE(core.receiveTask(changed, 100.1, 10.1));
  changed = task(); changed.issued_at = 100.1;
  EXPECT_FALSE(core.receiveTask(changed, 100.1, 10.1));
  EXPECT_DOUBLE_EQ(core.step(100.1, 10.1).forward, 0);
}

TEST(Tracker, RotatesAndFreezesBeforeForwardMotion) {
  TrackerCore core(config()); prepare(core);
  auto turned = odom(100.01); turned.yaw = 1.5;
  ASSERT_TRUE(core.receiveOdom(turned, 100.01, 10.01));
  const auto out = core.step(100.05, 10.05);
  EXPECT_DOUBLE_EQ(out.forward, 0);
  EXPECT_LT(out.yaw_rate, 0);
  EXPECT_TRUE(out.frozen);
}

TEST(Tracker, OccupiedForwardTurnRequestsControlledStopBeforePureMeasuredTurn) {
  TrackerCore core(config());prepare(core);
  auto pose=odom(100.05);pose.yaw=.4;
  ASSERT_TRUE(core.receiveOdom(pose,100.05,10.05));
  auto previous=core.step(100.05,10.05);ASSERT_GT(previous.forward,0.);
  ASSERT_TRUE(core.notifyBlockedForwardTurn(1,100.05,100.06,previous.forward,previous.yaw_rate));
  EXPECT_STREQ(core.maneuverPhase(),"decelerating_for_turn");
  bool rotated=false;
  for(int i=2;i<=16;++i) {
    const double t=i*.05;pose.stamp=100+t;
    pose.planar_speed=previous.forward;pose.velocity_in_frame={previous.forward,0,0};
    pose.angular_velocity_in_frame.z()=previous.yaw_rate;
    ASSERT_TRUE(core.receiveOdom(pose,100+t,10+t));core.refreshTaskLease(10+t);
    const auto output=core.step(100+t,10+t);
    EXPECT_LE(std::abs(output.forward-previous.forward),.35*.05+1e-9);
    EXPECT_LE(std::abs(output.yaw_rate-previous.yaw_rate),.8*.05+1e-9);
    EXPECT_TRUE(output.frozen);
    if(output.reason=="turn_first_aligning") {
      EXPECT_EQ(output.forward,0.);rotated=rotated||output.yaw_rate<0.;
    }
    previous=output;
  }
  EXPECT_TRUE(rotated);EXPECT_STREQ(core.maneuverPhase(),"aligning_to_curve");
}

TEST(Tracker, TurnFirstNeedsDistinctMeasuredAlignmentSamplesAndHasHysteresis) {
  TrackerCore core(config());prepare(core);
  ASSERT_TRUE(core.notifyBlockedForwardTurn(1,100.,100.,.1,.2));
  for(int i=1;i<=3;++i) {
    const double t=i*.02;ASSERT_TRUE(core.receiveOdom(odom(100+t),100+t,10+t));
    EXPECT_EQ(core.step(100+t,10+t).forward,0.);
  }
  ASSERT_STREQ(core.maneuverPhase(),"decelerating_for_turn"); // 60 ms is not rest.
  // Timer ticks with no new source sample cannot count as observed alignment.
  for(int i=4;i<=6;++i)EXPECT_EQ(core.step(100+i*.02,10+i*.02).forward,0.);
  EXPECT_STREQ(core.maneuverPhase(),"decelerating_for_turn");
  for(int i=7;i<=31;++i) {
    const double t=i*.02;ASSERT_TRUE(core.receiveOdom(odom(100+t),100+t,10+t));
    core.refreshTaskLease(10+t);
    EXPECT_EQ(core.step(100+t,10+t).forward,0.);
  }
  ASSERT_STREQ(core.maneuverPhase(),"aligning_to_curve");
  for(int i=32;i<=34;++i)EXPECT_EQ(core.step(100+i*.02,10+i*.02).forward,0.);
  EXPECT_STREQ(core.maneuverPhase(),"aligning_to_curve");
  for(int i=35;i<=65;++i) {
    const double t=i*.02;ASSERT_TRUE(core.receiveOdom(odom(100+t),100+t,10+t));
    core.refreshTaskLease(10+t);EXPECT_EQ(core.step(100+t,10+t).forward,0.);
  }
  ASSERT_STREQ(core.maneuverPhase(),"following");
  auto pose=odom(101.32);pose.yaw=.15;ASSERT_TRUE(core.receiveOdom(pose,101.32,11.32));
  EXPECT_GT(core.step(101.32,11.32).forward,0.);EXPECT_STREQ(core.maneuverPhase(),"following");
  pose.stamp=101.34;pose.yaw=.21;ASSERT_TRUE(core.receiveOdom(pose,101.34,11.34));
  const auto output=core.step(101.34,11.34);
  EXPECT_STREQ(core.maneuverPhase(),"decelerating_for_turn");EXPECT_TRUE(output.frozen);
}

TEST(Tracker, TurnRestUsesRecordLimitsAndFreshMovingSampleRestartsDuration) {
  auto c=config();c.stationary_linear_threshold_mps=.01;c.stationary_angular_threshold_radps=.02;
  c.stationary_reentry_duration_s=1.;c.stationary_minimum_samples=5;
  TrackerCore core(c);prepare(core);
  ASSERT_TRUE(core.notifyBlockedForwardTurn(1,100.,100.,.1,.2));
  for(int i=1;i<=67;++i) {
    const double t=i*.02;auto state=odom(100+t);
    if(i==16)state.planar_speed=.011; // Fresh MC/fusion noise above accepted threshold.
    ASSERT_TRUE(core.receiveOdom(state,100+t,10+t));core.refreshTaskLease(10+t);
    EXPECT_EQ(core.step(100+t,10+t).forward,0.);
    if(i<67) {EXPECT_STREQ(core.maneuverPhase(),"decelerating_for_turn")<<i;}
  }
  ASSERT_STREQ(core.maneuverPhase(),"aligning_to_curve");
  for(int i=68;i<=118;++i) {
    const double t=i*.02;ASSERT_TRUE(core.receiveOdom(odom(100+t),100+t,10+t));
    core.refreshTaskLease(10+t);EXPECT_EQ(core.step(100+t,10+t).forward,0.);
    if(i<118) {EXPECT_STREQ(core.maneuverPhase(),"aligning_to_curve")<<i;}
  }
  EXPECT_STREQ(core.maneuverPhase(),"following");
  ASSERT_TRUE(core.receiveOdom(odom(102.38),102.38,12.38));
  EXPECT_GT(core.step(102.38,12.38).forward,0.);
}

TEST(Tracker, TurnRestSampleMinimumCannotBeSatisfiedByTimerOrReplayedSource) {
  auto c=config();c.stationary_minimum_samples=100;TrackerCore core(c);prepare(core);
  ASSERT_TRUE(core.notifyBlockedForwardTurn(1,100.,100.,.1,.2));
  for(int i=1;i<=100;++i) {
    const double t=i*.02;auto state=odom(100+t);
    ASSERT_TRUE(core.receiveOdom(state,100+t,10+t));core.refreshTaskLease(10+t);
    EXPECT_FALSE(core.receiveOdom(state,100+t,10+t)); // Same source cannot add an observation.
    EXPECT_EQ(core.step(100+t,10+t).forward,0.);
    if(i<100) {EXPECT_STREQ(core.maneuverPhase(),"decelerating_for_turn")<<i;}
  }
  EXPECT_STREQ(core.maneuverPhase(),"aligning_to_curve");
}

TEST(Tracker, WrongOldZeroFeedbackCannotRequestManeuverAndStaleBodyCannotTurn) {
  TrackerCore core(config());prepare(core);
  EXPECT_FALSE(core.notifyBlockedForwardTurn(2,100.,100.,.1,.2));
  EXPECT_FALSE(core.notifyBlockedForwardTurn(1,99.8,100.,.1,.2));
  EXPECT_FALSE(core.notifyBlockedForwardTurn(1,100.,100.,0.,.2));
  EXPECT_FALSE(core.notifyBlockedForwardTurn(1,100.,100.,.1,0.));
  EXPECT_STREQ(core.maneuverPhase(),"following");
  ASSERT_TRUE(core.notifyBlockedForwardTurn(1,100.,100.,.1,.2));
  EXPECT_FALSE(core.notifyBlockedForwardTurn(1,100.,100.,.1,.2));
  core.refreshTaskLease(10.2);core.step(100.2,10.2);
  core.refreshTaskLease(10.4);core.step(100.4,10.4);
  core.refreshTaskLease(10.5);const auto output=core.step(100.5,10.5);
  EXPECT_EQ(output.forward,0.);EXPECT_EQ(output.yaw_rate,0.);
  EXPECT_TRUE(core.holding());
}

TEST(Tracker, SameTaskCandidateAndTemporaryProofHoldDoNotResetTurnPhase) {
  TrackerCore core(config());prepare(core);core.step(100.02,10.02);
  ASSERT_TRUE(core.notifyBlockedForwardTurn(1,100.02,100.02,.1,.2));
  ASSERT_TRUE(core.receiveTrajectory(trajectory(1,2),100.03,10.03));
  EXPECT_STREQ(core.maneuverPhase(),"decelerating_for_turn");
  EXPECT_FALSE(core.notifyBlockedForwardTurn(1,100.03,100.03,.1,.2));
  core.suspendOutput(100.04,10.04,"waiting_native_proof",false);
  EXPECT_STREQ(core.maneuverPhase(),"decelerating_for_turn");
  EXPECT_EQ(core.step(100.05,10.05).forward,0.);
  core.cancel("cancelled");EXPECT_STREQ(core.maneuverPhase(),"following");
}

namespace {
void alignBlockedEntry(TrackerCore& core) {
  prepare(core);ASSERT_TRUE(core.notifyBlockedForwardTurn(1,100.,100.,.1,.2));
  for(int i=1;i<=62;++i) {
    const double t=i*.02;ASSERT_TRUE(core.receiveOdom(odom(100+t),100+t,10+t));
    core.refreshTaskLease(10+t);EXPECT_EQ(core.step(100+t,10+t).forward,0.);
  }
  ASSERT_STREQ(core.maneuverPhase(),"following");
}
}

TEST(Tracker, OccupiedAlignedEntryWaitsWithoutRepeatedForwardAndRetainsTask) {
  TrackerCore core(config());alignBlockedEntry(core);
  ASSERT_TRUE(core.receiveOdom(odom(101.26),101.26,11.26));
  ASSERT_TRUE(core.notifyBlockedForwardTurn(1,101.26,101.26,.01,.016));
  ASSERT_STREQ(core.maneuverPhase(),"waiting_executable_entry");
  for(int i=64;i<=106;++i) {
    const double t=i*.02;ASSERT_TRUE(core.receiveOdom(odom(100+t),100+t,10+t));
    core.refreshTaskLease(10+t);const auto out=core.step(100+t,10+t);
    EXPECT_EQ(out.forward,0.);EXPECT_EQ(out.yaw_rate,0.);
    EXPECT_EQ(out.reason,"waiting_executable_entry");EXPECT_TRUE(core.active());
    EXPECT_FALSE(core.notifyBlockedForwardTurn(1,100+t,100+t,.01,.016));
  }
}

TEST(Tracker, OccupiedInPlaceAlignTurnWaitsForAnotherEntryInsteadOfRepeatingIt) {
  TrackerCore core(config());prepare(core);
  auto pose=odom(100.01);pose.yaw=.6;ASSERT_TRUE(core.receiveOdom(pose,100.01,10.01));
  ASSERT_TRUE(core.notifyBlockedForwardTurn(1,100.01,100.01,.1,.2));
  double yaw=0.;
  for(int i=1;i<=31&&std::string(core.maneuverPhase())!="aligning_to_curve";++i) {
    const double t=i*.02;pose.stamp=100+t;ASSERT_TRUE(core.receiveOdom(pose,100+t,10+t));
    core.refreshTaskLease(10+t);core.step(100+t,10+t);
  }
  ASSERT_STREQ(core.maneuverPhase(),"aligning_to_curve");
  for(int i=32;i<=37;++i) {
    const double t=i*.02;pose.stamp=100+t;ASSERT_TRUE(core.receiveOdom(pose,100+t,10+t));
    core.refreshTaskLease(10+t);const auto out=core.step(100+t,10+t);
    EXPECT_EQ(out.forward,0.);yaw=out.yaw_rate;
  }
  ASSERT_NE(yaw,0.);
  // Zero-yaw, stale and reverse feedback cannot escalate even while aligning.
  EXPECT_FALSE(core.notifyBlockedForwardTurn(1,100.74,100.74,0.,0.));
  EXPECT_FALSE(core.notifyBlockedForwardTurn(1,100.56,100.74,0.,yaw));
  EXPECT_FALSE(core.notifyBlockedForwardTurn(1,100.74,100.74,-.05,yaw));
  EXPECT_FALSE(core.notifyBlockedForwardTurn(2,100.74,100.74,0.,yaw));
  EXPECT_STREQ(core.maneuverPhase(),"aligning_to_curve");
  ASSERT_TRUE(core.notifyBlockedForwardTurn(1,100.74,100.74,0.,yaw));
  ASSERT_STREQ(core.maneuverPhase(),"waiting_executable_entry");
  for(int i=38;i<=63;++i) {
    const double t=i*.02;pose.stamp=100+t;ASSERT_TRUE(core.receiveOdom(pose,100+t,10+t));
    core.refreshTaskLease(10+t);const auto out=core.step(100+t,10+t);
    EXPECT_EQ(out.forward,0.);EXPECT_EQ(out.yaw_rate,0.);
    EXPECT_EQ(out.reason,"waiting_executable_entry");EXPECT_TRUE(core.active());
  }
  // Only an admitted replacement leaves the wait, and it restarts at a stop.
  auto replacement=trajectory(1,2);replacement.start_time=101.28;
  ASSERT_TRUE(core.receiveTrajectory(replacement,101.28,11.28));
  EXPECT_STREQ(core.maneuverPhase(),"decelerating_for_turn");
}

TEST(Tracker, InPlaceTurnFeedbackOutsideAligningCannotRequestManeuver) {
  TrackerCore core(config());prepare(core);
  EXPECT_FALSE(core.notifyBlockedForwardTurn(1,100.,100.,0.,.3));
  EXPECT_STREQ(core.maneuverPhase(),"following");
  ASSERT_TRUE(core.notifyBlockedForwardTurn(1,100.,100.,.1,.2));
  ASSERT_STREQ(core.maneuverPhase(),"decelerating_for_turn");
  EXPECT_FALSE(core.notifyBlockedForwardTurn(1,100.01,100.01,0.,.3));
  EXPECT_STREQ(core.maneuverPhase(),"decelerating_for_turn");
}

TEST(Tracker, OnlyAdmittedReplacementCanRetryBlockedEntryAndStillMustStopAndAlign) {
  TrackerCore core(config());alignBlockedEntry(core);
  ASSERT_TRUE(core.receiveOdom(odom(101.26),101.26,11.26));
  ASSERT_TRUE(core.notifyBlockedForwardTurn(1,101.26,101.26,.01,.016));
  auto bad=trajectory(1,2);bad.knots.clear();
  ASSERT_FALSE(core.receiveTrajectory(bad,101.26,11.26));
  EXPECT_STREQ(core.maneuverPhase(),"waiting_executable_entry");
  auto valid=trajectory(1,2);valid.start_time=101.26;
  ASSERT_TRUE(core.receiveTrajectory(valid,101.26,11.26));
  EXPECT_STREQ(core.maneuverPhase(),"decelerating_for_turn");
  ASSERT_TRUE(core.receiveOdom(odom(101.28),101.28,11.28));
  EXPECT_EQ(core.step(101.28,11.28).forward,0.);
  core.cancel("operator_cancel");EXPECT_FALSE(core.active());
  EXPECT_STREQ(core.maneuverPhase(),"following");
}

TEST(Tracker, RealProgressAfterAlignedEntryDistinguishesANewObstacle) {
  TrackerCore core(config());alignBlockedEntry(core);
  auto body=odom(101.26);body.position.x()=.06;
  ASSERT_TRUE(core.receiveOdom(body,101.26,11.26));
  ASSERT_TRUE(core.notifyBlockedForwardTurn(1,101.26,101.26,.01,.016));
  EXPECT_STREQ(core.maneuverPhase(),"decelerating_for_turn");
}

TEST(Tracker, RealSplineCurvatureConstrainsForwardSpeedWithoutGuessingLateralAcceleration) {
  EXPECT_NEAR(curvatureForwardLimit({.2,0,0},{0,.08,0},.5,.3),.25,1e-12);
  EXPECT_DOUBLE_EQ(curvatureForwardLimit({.2,0,.01},{0,0,.01},.5,.3),.3);
  EXPECT_DOUBLE_EQ(curvatureForwardLimit({NAN,0,0},{0,0,0},.5,.3),0.);
  auto c=config();c.max_yaw_rate=.1;TrackerCore core(c);
  ASSERT_TRUE(core.receiveTask(task(),100,10));ASSERT_TRUE(core.receiveOdom(odom(),100,10));
  auto curved=trajectory();for(auto& p:curved.points)p.y()=2*p.x()*p.x();
  ASSERT_TRUE(core.receiveTrajectory(curved,100,10));
  double maximum=0.;
  for(int i=1;i<=10;++i) {
    const double t=i*.05;auto pose=odom(100+t);pose.yaw=.5;
    ASSERT_TRUE(core.receiveOdom(pose,100+t,10+t));core.refreshTaskLease(10+t);
    const auto output=core.step(100+t,10+t);maximum=std::max(maximum,output.forward);
    EXPECT_LE(output.forward,.026);EXPECT_LE(std::abs(output.yaw_rate),.1);
  }
  EXPECT_GT(maximum,0.);
}

TEST(Tracker, FutureBendIsReachedThroughBrakingEnvelopeNotWholeCurveMinimum) {
  const std::vector<double> arc{0.,.4,.8,1.2};
  const std::vector<double> cap{.30,.30,.30,.10};
  const auto envelope=brakingSpeedEnvelope(arc,cap,.35,1.);
  ASSERT_EQ(envelope.size(),arc.size());
  EXPECT_DOUBLE_EQ(brakingEnvelopeAt(arc,envelope,0.,.35,1.),.30);
  EXPECT_DOUBLE_EQ(brakingEnvelopeAt(arc,envelope,.8,.35,1.),.30);
  EXPECT_NEAR(brakingEnvelopeAt(arc,envelope,1.19,.35,1.),std::sqrt(.01+.007),1e-12);
  // The low cap is still mandatory at the bend itself. No instantaneous
  // limiter reset or unknown/occupied-space exception is involved.
  EXPECT_DOUBLE_EQ(brakingEnvelopeAt(arc,envelope,1.2,.35,1.),.10);
  EXPECT_LT(brakingEnvelopeAt(arc,envelope,1.19,.35,1.),.30);
  for(std::size_t i=1;i<envelope.size();++i)
    EXPECT_LE(envelope[i-1]*envelope[i-1],envelope[i]*envelope[i]+2.*.35*(arc[i]-arc[i-1])+1e-12);
}

TEST(Tracker, SpatialPlanarProfileRequiresSupportAndDefaultsToLegacy) {
  auto c=config();EXPECT_FALSE(c.spatial_planar_braking_envelope);
  c.spatial_planar_braking_envelope=true;
  EXPECT_THROW(c.validate(),std::invalid_argument);
  c.require_support_reference=true;EXPECT_NO_THROW(c.validate());
}

TEST(Tracker, QuadraticPlanarDirectionBoundCoversInteriorAndZeroSpeedEnds) {
  const std::array<Eigen::Vector3d,3> controls{{{0.,0.,0.},{.05,0.,.01},{.10,.03,-.01}}};
  const double scale=quadraticPlanarScale(controls);ASSERT_GT(scale,.9);
  for(int i=0;i<=1000;++i) {
    const double t=i/1000.;const Eigen::Vector3d v=(1.-t)*(1.-t)*controls[0]+
      2.*t*(1.-t)*controls[1]+t*t*controls[2];
    if(v.norm()>1e-12){EXPECT_LE(scale,spatialToPlanarScale(v)+1e-12);}
  }
  EXPECT_DOUBLE_EQ(quadraticPlanarScale({Eigen::Vector3d(0.,0.,.1),
    Eigen::Vector3d(0.,0.,.2),Eigen::Vector3d(0.,0.,.3)}),0.);
  EXPECT_DOUBLE_EQ(quadraticPlanarScale({Eigen::Vector3d(.1,0.,0.),
    Eigen::Vector3d(-.1,0.,0.),Eigen::Vector3d(.1,0.,0.)}),0.);
}

TEST(Tracker, SpatialPlanarBrakingUsesLocalHorizontalTravelAndPreservesVerticalZeros) {
  const std::vector<double> times{0.,1.,2.,3.,4.};
  const std::vector<Eigen::Vector3d> points{{0.,0.,.48},{.0001,0.,.49},
    {.1,0.,.50},{.3,0.,.50},{.5,0.,.50}};
  const std::vector<Eigen::Vector3d> velocity{{.0001,0.,.01},{.0001,0.,.01},
    {.15,0.,0.},{.15,0.,0.},{.15,0.,0.}};
  const std::vector<Eigen::Vector3d> acceleration(points.size(),Eigen::Vector3d::Zero());
  const auto profile=SpatialPlanarBrakingEnvelope::build(times,points,velocity,acceleration,.3,.35,.5,1.);
  ASSERT_TRUE(profile.valid());EXPECT_LT(profile.speedAt(0.,points[0]),.0031);
  EXPECT_GT(profile.speedAt(3.,points[3]),.29);
  EXPECT_NEAR(profile.horizontal_arcs.back(),.5,1e-12);
  for(std::size_t i=1;i<points.size();++i)
    EXPECT_LE(profile.limits[i-1]*profile.limits[i-1],profile.limits[i]*profile.limits[i]+
      2.*profile.acceleration[i-1]*(profile.horizontal_arcs[i]-profile.horizontal_arcs[i-1])+1e-12);
  auto vertical=points;for(auto& p:vertical)p.x()=0.;
  const std::vector<Eigen::Vector3d> vertical_velocity(points.size(),Eigen::Vector3d(0.,0.,.01));
  const auto zero=SpatialPlanarBrakingEnvelope::build(times,vertical,vertical_velocity,acceleration,.3,.35,.5,1.);
  ASSERT_TRUE(zero.valid());
  for(std::size_t i=0;i<times.size();++i)EXPECT_DOUBLE_EQ(zero.speedAt(times[i],vertical[i]),0.);
  EXPECT_DOUBLE_EQ(zero.accelerationAt(1.5),0.);
}

TEST(Tracker, SlopedCurvedSpatialProfileKeepsLocalSpeedYawAndBrakingBounds) {
  std::vector<double> times;std::vector<Eigen::Vector3d> points,velocity,acceleration;
  for(int i=0;i<=20;++i) {
    const double t=i*.1;times.push_back(t);points.emplace_back(.1*t,.04*t*t,.55+.02*t);
    velocity.emplace_back(.1,.08*t,.02);acceleration.emplace_back(0.,.08,0.);
  }
  const auto profile=SpatialPlanarBrakingEnvelope::build(times,points,velocity,acceleration,.3,.35,.1,.98);
  ASSERT_TRUE(profile.valid());
  for(std::size_t i=0;i<times.size();++i) {
    const double limit=profile.speedAt(times[i],points[i]);EXPECT_TRUE(std::isfinite(limit));EXPECT_GE(limit,0.);
    EXPECT_LE(limit,.3*spatialToPlanarScale(velocity[i])+1e-12);
    EXPECT_LE(limit,curvatureForwardLimit(velocity[i],acceleration[i],.1,.3)+1e-12);
  }
  const Eigen::Vector3d current(.1*1.95,.04*1.95*1.95,.55+.02*1.95);
  const double limit=profile.speedAt(1.95,current);
  EXPECT_LE(limit*limit,profile.limits.back()*profile.limits.back()+
    2.*profile.acceleration.back()*(points.back()-current).head<2>().norm()+1e-12);
}
TEST(Tracker, BrakingEnvelopeCannotBorrowVerticalDistanceOrIgnoreCloseTurn) {
  const std::vector<double> arc{0.,.02};const std::vector<double> cap{.30,.05};
  const auto flat=brakingSpeedEnvelope(arc,cap,.35,1.);
  const auto inclined=brakingSpeedEnvelope(arc,cap,.35*.7,.7);
  ASSERT_EQ(flat.size(),2u);ASSERT_EQ(inclined.size(),2u);
  EXPECT_LT(flat.front(),.30);EXPECT_LT(inclined.front(),flat.front());
  EXPECT_DOUBLE_EQ(inclined.back(),.05);
  EXPECT_TRUE(brakingSpeedEnvelope({0.,-.1},cap,.35,1.).empty());
  EXPECT_TRUE(brakingSpeedEnvelope(arc,{.3,NAN},.35,1.).empty());
  EXPECT_TRUE(brakingSpeedEnvelope(arc,cap,.35,1.01).empty());
}
TEST(Tracker, NormalCurvatureControlNeverHardClipsThroughTheAccelerationLimit) {
  double previous=.30;
  for(double cap:{.30,.297,.295,.293,.291}) {
    const auto next=normalForwardStep(.1,previous,cap,.35,.02);ASSERT_TRUE(next);
    EXPECT_LE(*next,cap+1e-12);EXPECT_LE(std::abs(*next-previous),.35*.02+1e-12);previous=*next;
  }
  EXPECT_FALSE(normalForwardStep(.29,.30,.29,.35,.02));
  EXPECT_FALSE(normalForwardStep(.1,.30,.10,.35,.02));
  EXPECT_FALSE(normalForwardStep(.1,.30,.30,.35,.26));
}
TEST(Tracker, PoseJumpIntoUnreachableCurveSpeedRequestsExplicitStopAndPreservesAppliedGeometry) {
  TrackerCore core(config());prepare(core);
  auto t=trajectory(1,2);
  for(auto& p:t.points) {const double d=std::max(0.,p.x()-.6);p.y()=d<=.1?2.*d*d:.02+.4*(d-.1);}
  ASSERT_TRUE(core.receiveTrajectory(t,100.,10.));core.recordAppliedOutput(.296,0.,100.,10.);
  auto body=odom(100.02);body.position.x()=.3;
  ASSERT_TRUE(core.receiveOdom(body,100.02,10.02));core.refreshTaskLease(10.02);
  ASSERT_EQ(core.step(100.02,10.02).reason,"tracking");
  body=odom(100.04);body.position.x()=.57;
  ASSERT_TRUE(core.receiveOdom(body,100.04,10.04));core.refreshTaskLease(10.04);
  const auto stopped=core.step(100.04,10.04);
  EXPECT_EQ(stopped.reason,"braking_envelope_reentry_required");EXPECT_EQ(stopped.forward,0.);EXPECT_EQ(stopped.yaw_rate,0.);
  EXPECT_TRUE(core.requiresMeasuredReentry());EXPECT_TRUE(core.active());EXPECT_TRUE(core.hasInstalledGeometry());
  EXPECT_EQ(core.trajectoryId(),2);const auto arc=core.progressArc();
  // Fresh samples and permission heartbeats alone cannot reset this fence.
  for(int i=1;i<=40;++i) {
    body.stamp=100.04+i*.02;ASSERT_TRUE(core.receiveOdom(body,body.stamp,10.04+i*.02));
    core.refreshTaskLease(10.04+i*.02);
    const auto held=core.step(body.stamp,10.04+i*.02,true);
    EXPECT_EQ(held.reason,"braking_envelope_reentry_required");EXPECT_EQ(held.forward,0.);
  }
  EXPECT_TRUE(core.requiresMeasuredReentry());EXPECT_TRUE(core.hasInstalledGeometry());EXPECT_GE(core.progressArc(),arc-1e-9);
}

TEST(Tracker, DoesNotChaseAPlanInADifferentHeightOrFarAway) {
  for (int variant = 0; variant < 2; ++variant) {
    TrackerCore core(config()); prepare(core);
    auto plan = trajectory(1, 2);
    for (auto &point : plan.points) {
      if (variant == 0) point.z() += 1;
      else point.x() += 3;
    }
    EXPECT_EQ(core.receiveTrajectory(plan, 100, 10), variant != 0);
    const auto out = core.step(100.05, 10.05);
    if(variant==0) {EXPECT_GT(out.forward,0.);EXPECT_EQ(core.trajectoryId(),1);}
    else EXPECT_DOUBLE_EQ(out.forward, 0);
    EXPECT_EQ(out.reason, variant == 0 ? "tracking" :
                                      "tracking_error_outside_single_floor_envelope");
  }
}

TEST(Tracker, GoalCompletionStopsAndRemainsTerminal) {
  TrackerCore core(config()); prepare(core);
  auto at_goal = odom(100.01); at_goal.position.x() = 1.9;
  ASSERT_TRUE(core.receiveOdom(at_goal, 100.01, 10.01));
  const auto out = core.step(100.05, 10.05);
  EXPECT_DOUBLE_EQ(out.forward, 0);
  EXPECT_TRUE(out.finished);
  EXPECT_EQ(out.reason, "goal_reached");
  EXPECT_FALSE(core.receiveTask(task(), 100.1, 10.1));
}

TEST(Tracker, TrajectoryLeaseStopsEvenWithFreshTaskAndOdom) {
  auto c = config(); c.trajectory_timeout = .5;
  TrackerCore core(c); prepare(core);
  for (int i=1; i<=7; ++i) {
    core.receiveTask(task(), 100+i*.1, 10+i*.1);
    core.receiveOdom(odom(100+i*.1), 100+i*.1, 10+i*.1);
    core.step(100+i*.1, 10+i*.1);
  }
  EXPECT_EQ(core.reason(), "trajectory_stale");
  EXPECT_DOUBLE_EQ(core.step(100.75, 10.75).forward, 0);
}

TEST(Tracker, ClockJumpStopsInsteadOfExtrapolating) {
  TrackerCore core(config()); prepare(core);
  EXPECT_DOUBLE_EQ(core.step(101.0, 11.0).forward, 0);
  EXPECT_EQ(core.reason(), "clock_or_executor_discontinuity");
}

TEST(Tracker, PausedRosClockCannotKeepAdvancingAPlan) {
  TrackerCore core(config()); prepare(core);
  EXPECT_GT(core.step(100.05, 10.05).forward, 0);
  const auto paused = core.step(100.05, 10.1);
  EXPECT_DOUBLE_EQ(paused.forward, 0);
  EXPECT_EQ(paused.reason, "waiting_trajectory_clock");
  EXPECT_TRUE(paused.frozen);
}

TEST(Tracker, FutureTrajectoryDoesNotExecuteEarly) {
  TrackerCore core(config());
  core.receiveTask(task(), 100, 10);
  core.receiveOdom(odom(), 100, 10);
  auto future = trajectory(); future.start_time = 100.15;
  ASSERT_TRUE(core.receiveTrajectory(future, 100, 10));
  EXPECT_DOUBLE_EQ(core.step(100.05, 10.05).forward, 0);
  EXPECT_EQ(core.reason(), "waiting_trajectory_clock");
}

TEST(Tracker, ConfigurationCannotRelaxHardCeiling) {
  auto c = config(); c.max_speed = 1.501;
  EXPECT_THROW(TrackerCore core(c), std::invalid_argument);
  c = config(); c.odom_timeout = 5;
  EXPECT_THROW(TrackerCore core(c), std::invalid_argument);
  c = config(); c.session_id.clear();
  EXPECT_THROW(TrackerCore core(c), std::invalid_argument);
  c = config(); c.single_floor_max_height_change = .301;
  EXPECT_THROW(TrackerCore core(c), std::invalid_argument);
  c = config(); c.goal_height_tolerance = .21;
  EXPECT_THROW(TrackerCore core(c), std::invalid_argument);
}

TEST(Tracker, StationaryRecordThresholdsAreBoundedAndStricterValuesRemainValid) {
  for(int variant=0;variant<10;++variant) {
    auto c=config();
    if(variant==0)c.stationary_linear_threshold_mps=.051;
    if(variant==1)c.stationary_linear_threshold_mps=0.;
    if(variant==2)c.stationary_angular_threshold_radps=.101;
    if(variant==3)c.stationary_angular_threshold_radps=NAN;
    if(variant==4)c.stationary_reentry_duration_s=.59;
    if(variant==5)c.stationary_reentry_duration_s=5.01;
    if(variant==6)c.stationary_reentry_duration_s=NAN;
    if(variant==7)c.stationary_minimum_samples=2;
    if(variant==8)c.stationary_minimum_samples=513;
    if(variant==9)c.stationary_linear_threshold_mps=NAN;
    EXPECT_THROW(TrackerCore core(c),std::invalid_argument)<<variant;
  }
  auto c=config();c.stationary_linear_threshold_mps=.01;c.stationary_angular_threshold_radps=.02;
  c.stationary_reentry_duration_s=1.;c.stationary_minimum_samples=5;
  EXPECT_NO_THROW(TrackerCore core(c));
}

TEST(Tracker, SameXYOnAnotherFloorDoesNotCompleteOrResume) {
  TrackerCore core(config());
  auto target = task(); target.goal = Eigen::Vector3d(0, 0, 3.55);
  ASSERT_TRUE(core.receiveTask(target, 100, 10));
  ASSERT_TRUE(core.receiveOdom(odom(), 100, 10));
  const auto out = core.step(100.05, 10.05);
  EXPECT_FALSE(out.finished);
  EXPECT_DOUBLE_EQ(out.forward, 0);
  EXPECT_EQ(out.reason, "task_outside_single_floor_envelope");
  EXPECT_FALSE(core.active());
  EXPECT_FALSE(core.receiveTask(target, 100.1, 10.1));
}

TEST(Tracker, GoalRequiresHeightToleranceWithinSingleFloorEnvelope) {
  TrackerCore core(config());
  auto target = task(); target.goal = Eigen::Vector3d(0, 0, .75);
  ASSERT_TRUE(core.receiveTask(target, 100, 10));
  ASSERT_TRUE(core.receiveOdom(odom(), 100, 10));
  EXPECT_FALSE(core.step(100.05, 10.05).finished);
  auto arrived = odom(100.1); arrived.position.z() = .72;
  ASSERT_TRUE(core.receiveOdom(arrived, 100.1, 10.1));
  EXPECT_TRUE(core.step(100.1, 10.1).finished);
}

TEST(Tracker, GoalThresholdsMatchMeasuredReferenceXYZCompletion) {
  // Native reference guidance and execution tracking must agree on measured
  // body XYZ; a local spline's elapsed time or endpoint is not global arrival.
  for (const auto &offset : std::vector<Eigen::Vector2d>{
           {.20 - 1e-8, .15 - 1e-8}, {.20 + 1e-8, .15 - 1e-8},
           {.20 - 1e-8, .15 + 1e-8}}) {
    TrackerCore core(config());
    ASSERT_TRUE(core.receiveTask(task(), 100, 10));
    auto pose = odom();
    pose.position = task().goal - Eigen::Vector3d(offset.x(), 0., offset.y());
    ASSERT_TRUE(core.receiveOdom(pose, 100, 10));
    const auto output = core.step(100.05, 10.05);
    EXPECT_EQ(output.finished, offset.x() <= .20 && offset.y() <= .15);
    EXPECT_DOUBLE_EQ(output.forward, 0);
    EXPECT_DOUBLE_EQ(output.yaw_rate, 0);
  }
}

TEST(Tracker, ReplansCannotMoveTheSingleFloorHeightAnchor) {
  TrackerCore core(config()); prepare(core);
  core.step(100.05, 10.05);
  auto raised = odom(100.1); raised.position.z() += .20;
  ASSERT_TRUE(core.receiveOdom(raised, 100.1, 10.1));
  auto plan = trajectory(1, 2); plan.start_time = 100.1;
  for (auto &point : plan.points) point.z() += .20;
  ASSERT_TRUE(core.receiveTrajectory(plan, 100.1, 10.1));
  core.step(100.1, 10.1);
  raised.stamp = 100.15; raised.position.z() += .20;
  ASSERT_TRUE(core.receiveOdom(raised, 100.15, 10.15));
  const auto out = core.step(100.15, 10.15);
  EXPECT_FALSE(out.finished);
  EXPECT_DOUBLE_EQ(out.forward, 0);
  EXPECT_EQ(out.reason, "task_outside_single_floor_envelope");
  EXPECT_FALSE(core.active());
}

TEST(Tracker, EntireSplineHeightHullIsCheckedBeforeExecution) {
  TrackerCore core(config()); prepare(core);
  auto plan = trajectory(1, 2);
  // Reject the later rise without discarding the separate safe incumbent.
  plan.points.back().z() += .4;
  EXPECT_FALSE(core.receiveTrajectory(plan, 100.1, 10.1));
  EXPECT_TRUE(core.active());
  EXPECT_EQ(core.trajectoryId(),1);
  EXPECT_TRUE(core.receiveTask(task(), 100.15, 10.15));
}

TEST(Tracker, IdealSingleFloorClosedLoopReachesGoalWithoutRobot) {
  auto c = config(); TrackerCore core(c);
  auto goal = task(); goal.goal.x() = .95;
  ASSERT_TRUE(core.receiveTask(goal, 100, 10));
  ASSERT_TRUE(core.receiveTrajectory(trajectory(), 100, 10));
  Odom pose = odom();
  bool finished = false;
  for (int i = 1; i <= 95; ++i) {
    const double t = i*.05;
    pose.stamp = 100+t;
    core.receiveTask(goal, 100+t, 10+t);
    core.receiveOdom(pose, 100+t, 10+t);
    const auto out = core.step(100+t, 10+t);
    EXPECT_GE(out.forward, 0);
    EXPECT_LE(out.forward, .30);
    pose.position.x() += out.forward*.05;
    pose.planar_speed = out.forward;
    if (out.finished) { finished = true; break; }
  }
  EXPECT_TRUE(finished);
  EXPECT_GE(pose.position.x(), .75);
  EXPECT_LE(pose.position.x(), 1.15);
}

TEST(Tracker, LocalEndpointIsNotTheGlobalGoal) {
  TrackerCore core(config()); prepare(core);
  Odom pose = odom();
  bool waited = false;
  for (int i = 1; i <= 95; ++i) {
    const double t = i*.05;
    pose.stamp = 100+t;
    core.receiveTask(task(), 100+t, 10+t);
    core.receiveOdom(pose, 100+t, 10+t);
    const auto out = core.step(100+t, 10+t);
    pose.position.x() += out.forward*.05;
    pose.planar_speed = out.forward;
    EXPECT_FALSE(out.finished);
    if (out.reason == "local_segment_finished_waiting_replan") {
      waited = true;
      EXPECT_TRUE(core.active());
      if(out.forward==0.)break; // Segment boundary uses the real limiter, not an abrupt zero.
    }
  }
  EXPECT_TRUE(waited);
}

TEST(Tracker, ValidReplanPreservesLinearAndAngularAccelerationHistory) {
  TrackerCore core(config()); prepare(core);
  auto pose = odom(); pose.yaw = -.2;
  Output previous;
  for (int i = 1; i <= 20; ++i) {
    const double t = i * .05;
    pose.stamp = 100 + t;
    ASSERT_TRUE(core.receiveTask(task(), 100 + t, 10 + t));
    ASSERT_TRUE(core.receiveOdom(pose, 100 + t, 10 + t));
    previous = core.step(100 + t, 10 + t);
  }
  ASSERT_NEAR(previous.forward, .3, 1e-8);
  ASSERT_GT(previous.yaw_rate, .2);
  auto next = trajectory(1, 2); next.start_time = 101;
  for (auto &point : next.points) point.y() = -.1 * point.x();
  ASSERT_TRUE(core.receiveTrajectory(next, 101, 11));
  pose.stamp = 101.05;
  ASSERT_TRUE(core.receiveOdom(pose, 101.05, 11.05));
  const auto output = core.step(101.05, 11.05);
  EXPECT_NEAR(output.forward, .3, 1e-8);
  EXPECT_LE(std::abs(output.forward - previous.forward), .4 * .05 + 1e-8);
  EXPECT_LE(std::abs(output.yaw_rate - previous.yaw_rate), .8 * .05 + 1e-8);
  EXPECT_GT(output.yaw_rate, .1);  // not reset to the first acceleration tick
}

TEST(Tracker, InvalidCandidateRetainsEstablishedCurveAndLimiter) {
  TrackerCore core(config()); prepare(core);
  for (int i = 1; i <= 20; ++i) {
    const double t = i * .05;
    core.receiveTask(task(), 100 + t, 10 + t);
    core.receiveOdom(odom(100 + t), 100 + t, 10 + t);
    core.step(100 + t, 10 + t);
  }
  auto invalid = trajectory(1, 2); invalid.start_time = 101; invalid.knots.clear();
  EXPECT_FALSE(core.receiveTrajectory(invalid, 101, 11));
  const auto output = core.step(101.05, 11.05);
  EXPECT_DOUBLE_EQ(output.forward, config().max_speed);
  EXPECT_DOUBLE_EQ(output.yaw_rate, 0);
  EXPECT_EQ(output.reason, "tracking");
}

TEST(Tracker, ForwardOnlyControllerSteersTowardBothSidesOfParallelPath) {
  for (const double offset : {-.7, -.3, .3, .7}) {
    TrackerCore core(config()); prepare(core);
    auto pose = odom(100.01); pose.position.y() = offset;
    ASSERT_TRUE(core.receiveOdom(pose, 100.01, 10.01));
    const auto output = core.step(100.05, 10.05);
    EXPECT_LT(output.yaw_rate * offset, 0);
    EXPECT_GE(output.forward, 0);
    EXPECT_LE(output.forward, .3);
    if (std::abs(offset) > .6) {
      EXPECT_TRUE(output.frozen);
      EXPECT_DOUBLE_EQ(output.forward, 0);
    }
  }
}

TEST(Tracker, ParallelOffsetsConvergeWithReplansAndBoundedForwardOnlyCommands) {
  // Kinematic closed loop only: this demonstrates controller direction and
  // continuity, not robot dynamics, obstacle clearance or a hardware trial.
  for (const double offset : {-.7, -.3, .3, .7}) {
    TrackerCore core(config());
    auto goal = task(); goal.goal.x() = 4;
    ASSERT_TRUE(core.receiveTask(goal, 100, 10));
    auto pose = odom(); pose.position.y() = offset;
    Output previous;
    bool turned_while_frozen = false, drove = false, converged = false;
    for (int i = 0; i < 300; ++i) {
      const double t = i * .05;
      pose.stamp = 100 + t;
      ASSERT_TRUE(core.receiveTask(goal, 100 + t, 10 + t));
      ASSERT_TRUE(core.receiveOdom(pose, 100 + t, 10 + t));
      if (i % 20 == 0) {
        auto plan = trajectory(1, 1 + i / 20); plan.start_time = 100 + t;
        for (auto &point : plan.points) point.x() += pose.position.x();
        ASSERT_TRUE(core.receiveTrajectory(plan, 100 + t, 10 + t));
      }
      const auto output = core.step(100 + t, 10 + t);
      ASSERT_TRUE(core.active());
      EXPECT_TRUE(std::isfinite(output.forward));
      EXPECT_TRUE(std::isfinite(output.yaw_rate));
      EXPECT_GE(output.forward, 0);
      EXPECT_LE(output.forward, .3);
      EXPECT_LE(std::abs(output.yaw_rate), .5 + 1e-9);
      if (i > 0) {
        EXPECT_LE(std::abs(output.forward - previous.forward), .4 * .05 + 1e-8);
        EXPECT_LE(std::abs(output.yaw_rate - previous.yaw_rate), .8 * .05 + 1e-8);
      }
      turned_while_frozen |= output.frozen && std::abs(output.yaw_rate) > .01;
      drove |= output.forward > .05;
      pose.position.x() += output.forward * std::cos(pose.yaw) * .05;
      pose.position.y() += output.forward * std::sin(pose.yaw) * .05;
      pose.yaw = angle(pose.yaw + output.yaw_rate * .05);
      pose.planar_speed = output.forward;
      previous = output;
      if (std::abs(pose.position.y()) < .04 && pose.position.x() > .5) {
        converged = true;
        break;
      }
    }
    EXPECT_TRUE(drove);
    EXPECT_TRUE(converged) << "initial offset " << offset << ", final y " << pose.position.y();
    if (std::abs(offset) > .6) { EXPECT_TRUE(turned_while_frozen); }
  }
}

TEST(Tracker, BoundedSpatialSearchDoesNotJumpToDistantSplineEnd) {
  TrackerCore core(config()); prepare(core);
  auto pose = odom(); pose.position.x() = .7;
  core.receiveOdom(pose, 100, 10);
  // Only the first 0.8 s (0.2 m) are searchable on this tick, not the
  // entire four-second path. A later branch cannot be selected globally.
  const auto output = core.step(100.05, 10.05);
  EXPECT_FALSE(output.finished);
  EXPECT_LE(core.progressTime(), .8);
  EXPECT_GE(output.forward, 0);
  EXPECT_LE(output.forward, .3);
}
