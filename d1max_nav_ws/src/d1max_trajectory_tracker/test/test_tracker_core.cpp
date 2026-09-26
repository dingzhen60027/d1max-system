#include <gtest/gtest.h>
#include <limits>
#include "d1max_trajectory_tracker/tracker_core.hpp"

using namespace d1max_trajectory_tracker;

namespace {
Config config() { Config c; c.session_id = "test-session"; return c; }
Task task(std::uint64_t generation = 1) {
  return Task{"test-session", "d1max_loc_map", generation, true, 100.0, Eigen::Vector3d(2.0, 0.0, 0.55)};
}
Odom odom(double stamp = 100.0) {
  return Odom{"d1max_loc_map", "d1max_loc_base_link", stamp, 0.0, Eigen::Vector3d(0, 0, .55), 0.0};
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

TEST(Tracker, UsesVendorCurveAndOutputsBoundedSI) {
  TrackerCore core(config()); prepare(core);
  const auto out = core.step(100.05, 10.05);
  EXPECT_NEAR(out.forward, .02, 1e-9);  // .4 m/s² × .05s
  EXPECT_NEAR(out.yaw_rate, 0, 1e-9);
  EXPECT_FALSE(out.frozen);
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

TEST(Tracker, EmptyInvalidOrWrongFramePlanStopsExistingMotion) {
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
    EXPECT_DOUBLE_EQ(core.step(100.05, 10.05).forward, 0);
  }
}

TEST(Tracker, OdomFramesQuaternionAndSpeedCannotBypassSafety) {
  for (int variant = 0; variant < 5; ++variant) {
    TrackerCore core(config()); prepare(core);
    auto bad = odom();
    if (variant == 0) bad.frame_id = "odom";
    if (variant == 1) bad.child_frame_id = "lidar";
    if (variant == 2) bad.yaw = std::numeric_limits<double>::quiet_NaN();
    if (variant == 3) bad.planar_speed = 1.51;
    if (variant == 4) bad.stamp = 99;
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
  auto turned = odom(); turned.yaw = 1.5;
  core.receiveOdom(turned, 100, 10);
  const auto out = core.step(100.05, 10.05);
  EXPECT_DOUBLE_EQ(out.forward, 0);
  EXPECT_LT(out.yaw_rate, 0);
  EXPECT_TRUE(out.frozen);
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
    EXPECT_DOUBLE_EQ(out.forward, 0);
    EXPECT_EQ(out.reason, variant == 0 ? "trajectory_outside_single_floor_envelope" :
                                      "tracking_error_outside_single_floor_envelope");
  }
}

TEST(Tracker, GoalCompletionStopsAndRemainsTerminal) {
  TrackerCore core(config()); prepare(core);
  auto at_goal = odom(); at_goal.position.x() = 1.9;
  core.receiveOdom(at_goal, 100, 10);
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
  // The initial point is safe; a later rise is still a terminal rejection.
  plan.points.back().z() += .4;
  EXPECT_FALSE(core.receiveTrajectory(plan, 100.1, 10.1));
  EXPECT_FALSE(core.active());
  EXPECT_EQ(core.reason(), "trajectory_outside_single_floor_envelope");
  EXPECT_FALSE(core.receiveTask(task(), 100.15, 10.15));
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
      EXPECT_DOUBLE_EQ(out.forward, 0);
      EXPECT_TRUE(core.active());
      break;
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

TEST(Tracker, InvalidReplanStillImmediatelyStopsEstablishedMotion) {
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
  EXPECT_DOUBLE_EQ(output.forward, 0);
  EXPECT_DOUBLE_EQ(output.yaw_rate, 0);
  EXPECT_EQ(output.reason, "invalid_trajectory");
}

TEST(Tracker, ForwardOnlyControllerSteersTowardBothSidesOfParallelPath) {
  for (const double offset : {-.7, -.3, .3, .7}) {
    TrackerCore core(config()); prepare(core);
    auto pose = odom(); pose.position.y() = offset;
    ASSERT_TRUE(core.receiveOdom(pose, 100, 10));
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
