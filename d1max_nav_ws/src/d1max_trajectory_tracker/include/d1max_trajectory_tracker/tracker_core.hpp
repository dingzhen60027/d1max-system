#pragma once

// ROS-independent admission/control core. Trajectory evaluation is SCAN's own
// implementation, not a second interpretation of its knot/time conventions.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <bspline_opt/uniform_bspline.h>

namespace d1max_trajectory_tracker
{
constexpr double HARD_PLANAR_SPEED = 1.5;
inline bool finite(double value) { return std::isfinite(value); }
inline double angle(double value) { return std::atan2(std::sin(value), std::cos(value)); }

struct Config
{
  std::string session_id, planning_frame{"d1max_loc_map"}, base_frame{"d1max_loc_base_link"};
  double max_speed{0.30}, max_yaw_rate{0.50};
  double max_acceleration{0.40}, max_yaw_acceleration{0.80};
  double task_timeout{0.75}, odom_timeout{0.40}, trajectory_timeout{5.0};
  double lookahead{0.8}, kp_position{0.8}, kp_yaw{1.5};
  double heading_threshold{0.6}, goal_tolerance{0.20}, position_freeze_distance{0.60};
  double goal_height_tolerance{0.15}, single_floor_max_height_change{0.25};
  void validate() const
  {
    if (session_id.empty() || planning_frame.empty() || base_frame.empty())
      throw std::invalid_argument("session and frames must be explicit");
    const auto bound = [](double value, double ceiling) {
      return finite(value) && value > 0.0 && value <= ceiling;
    };
    if (!bound(max_speed, HARD_PLANAR_SPEED) || !bound(max_yaw_rate, 1.0) ||
        !bound(max_acceleration, 0.8) || !bound(max_yaw_acceleration, 1.5) ||
        !bound(task_timeout, 1.0) || !bound(odom_timeout, 0.5) ||
        !bound(trajectory_timeout, 10.0) || !bound(lookahead, 2.0) ||
        !bound(kp_position, 3.0) || !bound(kp_yaw, 3.0) ||
        !bound(heading_threshold, 1.0) || !bound(goal_tolerance, 0.3) ||
        !bound(goal_height_tolerance, 0.20) ||
        !bound(single_floor_max_height_change, 0.30) ||
        goal_height_tolerance > single_floor_max_height_change ||
        !bound(position_freeze_distance, 1.0))
      throw std::invalid_argument("unsafe tracker configuration");
  }
};

struct Task
{
  std::string session_id, frame_id;
  std::uint64_t generation{0};
  bool active{false};
  double issued_at{0.0};
  Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
};

struct Trajectory
{
  std::string session_id, frame_id;
  std::uint64_t generation{0};
  std::int64_t id{0};
  double start_time{0.0};
  int order{3};
  std::vector<Eigen::Vector3d> points;
  std::vector<double> knots;
};

struct Odom
{
  std::string frame_id, child_frame_id;
  double stamp{0.0}, yaw{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double planar_speed{0.0};
};

struct Output
{
  double forward{0.0}, yaw_rate{0.0};
  bool frozen{true}, finished{false};
  std::string reason{"idle"};
};

class TrackerCore
{
public:
  explicit TrackerCore(Config config) : config_(std::move(config)) { config_.validate(); }

  bool receiveTask(const Task &task, double ros_now, double received)
  {
    if (task.session_id != config_.session_id || task.generation < task_.generation ||
        task.generation == 0 || !finite(received) || !finite(ros_now)) return false;
    if (!task.active) {
      task_ = task;
      cancel("task_stopped");
      return true;
    }
    if (task.frame_id != config_.planning_frame || !finite(task.issued_at) ||
        task.issued_at <= 0.0 || task.issued_at > ros_now + 0.2 || !task.goal.allFinite()) {
      cancel("invalid_task");
      return false;
    }
    if (task.generation == task_.generation) {
      // A terminal generation can never be revived by a delayed heartbeat.
      if (!active_) return false;
      if (task.issued_at != task_.issued_at || task.frame_id != task_.frame_id ||
          (task.goal - task_.goal).norm() > 1e-9) {
        cancel("task_context_changed_without_generation");
        return false;
      }
    } else {
      if (ros_now - task.issued_at > config_.task_timeout) return false;
      task_ = task;
      trajectory_.reset();
      last_trajectory_id_ = -1;
      active_ = true;
      reason_ = "waiting_trajectory";
      finished_ = false;
      floor_anchor_valid_ = false;
      last_output_ = Output{};
      last_step_ = received;
    }
    task_received_ = received;
    return true;
  }

  bool receiveOdom(const Odom &odom, double ros_now, double received)
  {
    if (!finite(ros_now) || !finite(received) || !finite(odom.stamp) ||
        odom.frame_id != config_.planning_frame || odom.child_frame_id != config_.base_frame ||
        !odom.position.allFinite() || !finite(odom.yaw) || !finite(odom.planar_speed) ||
        odom.planar_speed > HARD_PLANAR_SPEED || odom.planar_speed < 0.0 ||
        ros_now - odom.stamp > config_.odom_timeout || odom.stamp > ros_now + 0.1) {
      have_odom_ = false;
      invalidateTrajectory("invalid_odometry");
      return false;
    }
    odom_ = odom;
    odom_received_ = received;
    have_odom_ = true;
    return true;
  }

  bool receiveTrajectory(const Trajectory &trajectory, double ros_now, double received)
  {
    // Unrelated or obsolete context is not allowed to replace a current plan.
    if (!active_ || trajectory.session_id != config_.session_id ||
        trajectory.generation != task_.generation || trajectory.id <= last_trajectory_id_)
      return false;
    // Validate into temporary curves. A normal replan is not a stop: replacing
    // the curve must retain both command limiters. Invalid replacement input
    // still revokes the old curve and zeros immediately, rather than silently
    // continuing to execute a path which the planner has withdrawn.
    const auto reject = [this]() {
      invalidateTrajectory("invalid_trajectory");
      return false;
    };
    if (!finite(ros_now) || !finite(received) || !finite(trajectory.start_time) ||
        trajectory.frame_id != config_.planning_frame ||
        trajectory.start_time + 1e-6 < task_.issued_at ||
        trajectory.start_time > ros_now + 0.2 ||
        ros_now - trajectory.start_time > config_.trajectory_timeout ||
        trajectory.order != 3 || trajectory.points.size() < 4 ||
        trajectory.points.size() > 10000 ||
        trajectory.knots.size() != trajectory.points.size() + trajectory.order + 1)
      return reject();
    for (const auto &point : trajectory.points) if (!point.allFinite()) return reject();
    // The convex hull of the control points bounds the entire cubic's height.
    // This is a single-floor controller, not an automatic stair controller.
    double min_z = trajectory.points.front().z(), max_z = min_z;
    for (const auto &point : trajectory.points) {
      min_z = std::min(min_z, point.z());
      max_z = std::max(max_z, point.z());
    }
    const double anchor = floor_anchor_valid_ ? floor_anchor_z_ : task_.goal.z();
    if (max_z - min_z > config_.single_floor_max_height_change ||
        min_z < anchor - config_.single_floor_max_height_change ||
        max_z > anchor + config_.single_floor_max_height_change) {
      cancel("trajectory_outside_single_floor_envelope");
      return false;
    }
    // SCAN emits strictly increasing, non-clamped knots. Reject repeated knots
    // before calling the vendor evaluator, which divides by their differences.
    for (std::size_t i = 0; i < trajectory.knots.size(); ++i)
      if (!finite(trajectory.knots[i]) ||
          (i && trajectory.knots[i] - trajectory.knots[i - 1] <= 1e-9)) return reject();
    const double duration = trajectory.knots[trajectory.points.size()] -
                            trajectory.knots[trajectory.order];
    if (!finite(duration) || duration <= 0.01 || duration > 120.0 ||
        ros_now - trajectory.start_time > duration) return reject();
    Eigen::MatrixXd points(3, trajectory.points.size());
    for (std::size_t i = 0; i < trajectory.points.size(); ++i) points.col(i) = trajectory.points[i];
    Eigen::VectorXd knots(trajectory.knots.size());
    for (std::size_t i = 0; i < trajectory.knots.size(); ++i) knots(i) = trajectory.knots[i];
    auto curve = std::make_unique<scan_planner::UniformBspline>(points, trajectory.order, 0.1);
    curve->setKnot(knots);
    auto velocity = std::make_unique<scan_planner::UniformBspline>(curve->getDerivative());
    if (!curve->evaluateDeBoorT(0.0).allFinite() ||
        !curve->evaluateDeBoorT(duration).allFinite() ||
        !velocity->evaluateDeBoorT(0.0).allFinite()) return reject();
    trajectory_ = std::move(curve);
    velocity_ = std::move(velocity);
    trajectory_received_ = received;
    trajectory_start_ = trajectory.start_time;
    duration_ = duration;
    trajectory_min_z_ = min_z;
    trajectory_max_z_ = max_z;
    // A replanned spline starts at the planner's measured body pose, not at a
    // point extrapolated by elapsed wall time while the robot was turning.
    // Its progress will be measured spatially in step(), within a bounded
    // forward window. Original start/receipt times remain the lease clocks.
    execution_time_ = 0.0;
    last_trajectory_id_ = trajectory.id;
    reason_ = "tracking";
    return true;
  }

  Output step(double ros_now, double received)
  {
    // Idle has no execution clock to guard. Establish a baseline instead of
    // reporting a huge "clock jump" on the first timer after process startup.
    if (!active_ && finite(ros_now) && finite(received)) {
      last_step_ = received;
      last_ros_time_ = ros_now;
      return stop();
    }
    const double dt = received - last_step_;
    const double trajectory_dt = last_ros_time_ > 0.0 ? ros_now - last_ros_time_ : dt;
    last_step_ = received;
    if (!finite(received) || !finite(ros_now) || !finite(dt) || dt < 0.0 || dt > 0.25 ||
        !finite(trajectory_dt) || trajectory_dt < 0.0 || trajectory_dt > 0.25) {
      last_ros_time_ = ros_now;
      invalidateTrajectory("clock_or_executor_discontinuity");
      return stop();
    }
    last_ros_time_ = ros_now;
    if (!active_) return stop();
    if (!fresh(received, task_received_, config_.task_timeout)) {
      cancel("task_heartbeat_stale");
      return stop();
    }
    if (!have_odom_ || !fresh(received, odom_received_, config_.odom_timeout) ||
        !fresh(ros_now, odom_.stamp, config_.odom_timeout, 0.1)) {
      invalidateTrajectory("odometry_stale");
      return stop();
    }
    if (!floor_anchor_valid_) {
      floor_anchor_z_ = odom_.position.z();
      floor_anchor_valid_ = true;
    }
    // Pin height to this generation's first fresh measured body pose. Replans
    // cannot walk this anchor up a staircase one small local segment at a time.
    if (std::abs(task_.goal.z() - floor_anchor_z_) > config_.single_floor_max_height_change ||
        std::abs(odom_.position.z() - floor_anchor_z_) > config_.single_floor_max_height_change ||
        (trajectory_ && (trajectory_min_z_ < floor_anchor_z_ - config_.single_floor_max_height_change ||
                         trajectory_max_z_ > floor_anchor_z_ + config_.single_floor_max_height_change))) {
      cancel("task_outside_single_floor_envelope");
      return stop();
    }
    if ((task_.goal.head<2>() - odom_.position.head<2>()).norm() <= config_.goal_tolerance &&
        std::abs(task_.goal.z() - odom_.position.z()) <= config_.goal_height_tolerance) {
      finished_ = true;
      cancel("goal_reached");
      return stop();
    }
    if (!trajectory_) return stop();
    if (trajectory_dt == 0.0 || ros_now < trajectory_start_) {
      reason_ = "waiting_trajectory_clock";
      return stop();
    }
    if (!fresh(received, trajectory_received_, config_.trajectory_timeout) ||
        ros_now - trajectory_start_ > duration_ + config_.trajectory_timeout) {
      invalidateTrajectory("trajectory_stale");
      return stop();
    }
    // Use the same nearest-point/lookahead principle as the upstream adapter,
    // but never scan the entire spline per control tick. Progress is monotonic
    // and the forward search is bounded to lookahead (<= 2 s), at 40 chords.
    // Chord projection avoids quantising a slow trajectory to sample times.
    execution_time_ = nearestTime(execution_time_);
    const auto pos = trajectory_->evaluateDeBoorT(execution_time_);
    const Eigen::Vector2d pos_error = pos.head<2>() - odom_.position.head<2>();
    if (pos_error.norm() > 2.0 || std::abs(pos.z() - odom_.position.z()) > 0.5) {
      invalidateTrajectory("tracking_error_outside_single_floor_envelope");
      return stop();
    }
    const double look_time = std::min(duration_, execution_time_ + config_.lookahead);
    const Eigen::Vector3d desired = trajectory_->evaluateDeBoorT(look_time);
    const Eigen::Vector3d velocity = velocity_->evaluateDeBoorT(look_time);
    const Eigen::Vector2d look_error = desired.head<2>() - odom_.position.head<2>();
    // The upstream adapter can translate sideways. Ours cannot: the complete
    // positional feedback must steer yaw, not be thrown away after projecting
    // velocity onto body X. This is essential on a straight, parallel offset.
    // At the endpoint remove feed-forward so it cannot pull us past the end.
    const Eigen::Vector2d world = config_.kp_position * look_error +
        (look_time < duration_ ? Eigen::Vector2d(velocity.head<2>()) : Eigen::Vector2d::Zero());
    if (!world.allFinite() || !pos.allFinite() || !desired.allFinite()) {
      invalidateTrajectory("nonfinite_controller_output");
      return stop();
    }
    const double desired_yaw = world.norm() > 1e-5 ?
        std::atan2(world.y(), world.x()) : odom_.yaw;
    const double yaw_error = angle(desired_yaw - odom_.yaw);
    Output out;
    out.reason = "tracking";
    out.frozen = std::abs(yaw_error) > config_.heading_threshold ||
                 pos_error.norm() > config_.position_freeze_distance;
    double forward = 0.0;
    if (std::abs(yaw_error) <= config_.heading_threshold) {
      forward = std::clamp(std::cos(odom_.yaw) * world.x() + std::sin(odom_.yaw) * world.y(),
                           0.0, config_.max_speed);
    }
    if (look_time >= duration_ && look_error.norm() <= config_.goal_tolerance) {
      invalidateTrajectory("local_segment_finished_waiting_replan");
      return stop();
    }
    const double yaw = std::clamp(config_.kp_yaw * yaw_error,
                                 -config_.max_yaw_rate, config_.max_yaw_rate);
    out.forward = std::clamp(forward, last_output_.forward - config_.max_acceleration * dt,
                            last_output_.forward + config_.max_acceleration * dt);
    out.yaw_rate = std::clamp(yaw, last_output_.yaw_rate - config_.max_yaw_acceleration * dt,
                             last_output_.yaw_rate + config_.max_yaw_acceleration * dt);
    if (!finite(out.forward) || !finite(out.yaw_rate)) {
      invalidateTrajectory("nonfinite_controller_output");
      return stop();
    }
    out.forward = std::clamp(out.forward, 0.0, std::min(config_.max_speed, HARD_PLANAR_SPEED));
    last_output_ = out;
    reason_ = out.reason;
    return out;
  }

  void cancel(const std::string &reason) {
    active_ = false;
    if (reason != "goal_reached") finished_ = false;
    invalidateTrajectory(reason);
  }
  bool active() const { return active_; }
  std::uint64_t generation() const { return task_.generation; }
  std::int64_t trajectoryId() const { return last_trajectory_id_; }
  double progressTime() const { return execution_time_; }
  const std::string &reason() const { return reason_; }

private:
  double nearestTime(double from) const {
    constexpr int CHORDS = 40;
    const double to = std::min(duration_, from + config_.lookahead);
    double best_time = from;
    Eigen::Vector3d previous = trajectory_->evaluateDeBoorT(from);
    double best_dist = (previous.head<2>() - odom_.position.head<2>()).squaredNorm();
    for (int i = 1; i <= CHORDS && to > from; ++i) {
      const double t = from + (to - from) * static_cast<double>(i) / CHORDS;
      const Eigen::Vector3d point = trajectory_->evaluateDeBoorT(t);
      const Eigen::Vector2d chord = point.head<2>() - previous.head<2>();
      const double denominator = chord.squaredNorm();
      const double fraction = denominator > 1e-12 ? std::clamp(
          (odom_.position.head<2>() - previous.head<2>()).dot(chord) / denominator, 0.0, 1.0) : 0.0;
      const double distance = (previous.head<2>() + fraction * chord -
                               odom_.position.head<2>()).squaredNorm();
      // Strictly nearer only: overlapping/stationary segments cannot advance
      // progress solely because their sample index is later.
      if (distance + 1e-12 < best_dist) {
        best_dist = distance;
        best_time = from + (to - from) * (static_cast<double>(i - 1) + fraction) / CHORDS;
      }
      previous = point;
    }
    return best_time;
  }
  static bool fresh(double now, double then, double limit, double future = 0.0) {
    return finite(now) && finite(then) && now - then >= -future && now - then <= limit;
  }
  void invalidateTrajectory(const std::string &reason) {
    trajectory_.reset(); velocity_.reset(); reason_ = reason;
    last_output_ = Output{};
  }
  Output stop() {
    last_output_ = Output{};
    last_output_.reason = reason_;
    last_output_.finished = finished_;
    return last_output_;
  }
  Config config_;
  Task task_;
  Odom odom_;
  bool active_{false}, finished_{false}, have_odom_{false};
  bool floor_anchor_valid_{false};
  double floor_anchor_z_{0.0}, trajectory_min_z_{0.0}, trajectory_max_z_{0.0};
  double task_received_{-1e10}, odom_received_{-1e10}, trajectory_received_{-1e10};
  double trajectory_start_{0.0}, duration_{0.0}, execution_time_{0.0};
  double last_step_{0.0}, last_ros_time_{0.0};
  std::int64_t last_trajectory_id_{-1};
  std::string reason_{"idle"};
  std::unique_ptr<scan_planner::UniformBspline> trajectory_, velocity_;
  Output last_output_;
};
}  // namespace d1max_trajectory_tracker
