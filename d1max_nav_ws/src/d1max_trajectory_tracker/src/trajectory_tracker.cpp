#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <d1max_planning_interfaces/msg/tagged_bspline.hpp>
#include "d1max_trajectory_tracker/tracker_core.hpp"

namespace d1max_trajectory_tracker
{
using Json = nlohmann::json;
static double monotonicNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static double stampSeconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

class TrackerNode : public rclcpp::Node
{
public:
  TrackerNode() : Node("trajectory_tracker")
  {
    Config c;
    c.session_id = declare_parameter<std::string>("session_id", "");
    c.planning_frame = declare_parameter<std::string>("planning_frame", c.planning_frame);
    c.base_frame = declare_parameter<std::string>("base_frame", c.base_frame);
    c.max_speed = declare_parameter<double>("max_speed", c.max_speed);
    c.max_yaw_rate = declare_parameter<double>("max_yaw_rate", c.max_yaw_rate);
    c.max_acceleration = declare_parameter<double>("max_acceleration", c.max_acceleration);
    c.max_yaw_acceleration = declare_parameter<double>("max_yaw_acceleration", c.max_yaw_acceleration);
    c.task_timeout = declare_parameter<double>("task_timeout", c.task_timeout);
    c.odom_timeout = declare_parameter<double>("odom_timeout", c.odom_timeout);
    c.trajectory_timeout = declare_parameter<double>("trajectory_timeout", c.trajectory_timeout);
    c.lookahead = declare_parameter<double>("lookahead", c.lookahead);
    c.kp_position = declare_parameter<double>("kp_position", c.kp_position);
    c.kp_yaw = declare_parameter<double>("kp_yaw", c.kp_yaw);
    c.heading_threshold = declare_parameter<double>("heading_threshold", c.heading_threshold);
    c.goal_tolerance = declare_parameter<double>("goal_tolerance", c.goal_tolerance);
    c.goal_height_tolerance = declare_parameter<double>("goal_height_tolerance", c.goal_height_tolerance);
    c.single_floor_max_height_change = declare_parameter<double>(
        "single_floor_max_height_change", c.single_floor_max_height_change);
    c.position_freeze_distance = declare_parameter<double>("position_freeze_distance", c.position_freeze_distance);
    config_ = c;
    core_ = std::make_unique<TrackerCore>(c);
    const auto topic = [this](const char *key, const char *value) {
      return declare_parameter<std::string>(key, value);
    };
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        topic("command_topic", "/d1max/pct_scan/cmd_vel_raw"), rclcpp::QoS(1));
    frozen_pub_ = create_publisher<std_msgs::msg::Bool>(
        topic("frozen_topic", "/d1max/pct_scan/execution_frozen"), rclcpp::QoS(1));
    status_pub_ = create_publisher<std_msgs::msg::String>(
        topic("status_topic", "/d1max/pct_scan/tracker_status"), rclcpp::QoS(1));
    task_sub_ = create_subscription<std_msgs::msg::String>(
        topic("task_topic", "/d1max/pct_scan/task"), rclcpp::QoS(1),
        [this](const std_msgs::msg::String::ConstSharedPtr msg) { taskCallback(*msg); });
    spline_sub_ = create_subscription<d1max_planning_interfaces::msg::TaggedBspline>(
        topic("trajectory_topic", "/d1max/pct_scan/planning/tagged_bspline"), rclcpp::QoS(1),
        [this](const d1max_planning_interfaces::msg::TaggedBspline::ConstSharedPtr msg) {
          const auto &spline = msg->trajectory;
          Trajectory input;
          input.session_id = msg->session_id;
          input.generation = msg->generation;
          input.frame_id = msg->frame_id;
          input.id = spline.traj_id;
          input.start_time = stampSeconds(spline.start_time);
          input.order = spline.order;
          input.knots = spline.knots;
          input.points.reserve(spline.pos_pts.size());
          for (const auto &point : spline.pos_pts) input.points.emplace_back(point.x, point.y, point.z);
          if (core_->receiveTrajectory(input, now().seconds(), monotonicNow())) ++accepted_;
          else ++rejected_;
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        topic("odom_topic", "/localization/odometry/global"), rclcpp::SensorDataQoS().keep_last(1),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          Odom input;
          input.frame_id = msg->header.frame_id;
          input.child_frame_id = msg->child_frame_id;
          input.stamp = stampSeconds(msg->header.stamp);
          const auto &p = msg->pose.pose.position;
          const auto &q = msg->pose.pose.orientation;
          const auto &v = msg->twist.twist.linear;
          input.position = Eigen::Vector3d(p.x, p.y, p.z);
          const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
          input.yaw = (finite(norm) && std::abs(norm - 1.0) < 1e-3) ?
              std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z)) :
              std::numeric_limits<double>::quiet_NaN();
          input.planar_speed = std::hypot(v.x, v.y);
          core_->receiveOdom(input, now().seconds(), monotonicNow());
        });
    stop_service_ = create_service<std_srvs::srv::Trigger>(
        topic("stop_service", "/d1max/pct_scan/tracker_stop"),
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          core_->cancel("operator_stopped");
          publish(core_->step(now().seconds(), monotonicNow()));
          response->success = true;
          response->message = "Stopped. A new task generation is required.";
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      publish(core_->step(now().seconds(), monotonicNow()));
    });
    RCLCPP_INFO(get_logger(), "Guarded SCAN tracker ready; idle until an explicit session task. Max %.2f m/s.", c.max_speed);
  }

private:
  void taskCallback(const std_msgs::msg::String &message)
  {
    try {
      const Json data = Json::parse(message.data);
      Task task;
      task.session_id = data.at("session_id").get<std::string>();
      if (!data.at("generation").is_number_unsigned()) throw std::invalid_argument("generation is not uint");
      task.generation = data.at("generation").get<std::uint64_t>();
      task.active = data.at("active").get<bool>();
      if (task.active) {
        task.frame_id = data.at("frame_id").get<std::string>();
        task.issued_at = data.at("issued_at").get<double>();
        const auto &xyz = data.at("target_xyz");
        if (!xyz.is_array() || xyz.size() != 3) throw std::invalid_argument("target_xyz must have three entries");
        task.goal = Eigen::Vector3d(xyz[0].get<double>(), xyz[1].get<double>(), xyz[2].get<double>());
      }
      core_->receiveTask(task, now().seconds(), monotonicNow());
    } catch (const std::exception &) {
      core_->cancel("malformed_task");
      publish(core_->step(now().seconds(), monotonicNow()));
    }
  }

  void publish(const Output &output)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = output.forward;
    command.angular.z = output.yaw_rate;
    command_pub_->publish(command);
    std_msgs::msg::Bool frozen;
    frozen.data = output.frozen;
    frozen_pub_->publish(frozen);
    Json status = {
      {"session_id", config_.session_id}, {"generation", core_->generation()},
      {"active", core_->active()}, {"finished", output.finished}, {"reason", output.reason},
      {"trajectory_id", core_->trajectoryId()}, {"accepted_trajectories", accepted_},
      {"rejected_trajectories", rejected_}, {"frame_id", config_.planning_frame},
      {"command", {{"x", output.forward}, {"y", 0.0}, {"yaw", output.yaw_rate}}},
      {"execution_frozen", output.frozen}, {"max_speed", config_.max_speed},
      {"hard_planar_limit", HARD_PLANAR_SPEED}, {"stamp", now().seconds()}
    };
    std_msgs::msg::String message;
    message.data = status.dump();
    status_pub_->publish(message);
  }
  Config config_;
  std::unique_ptr<TrackerCore> core_;
  std::uint64_t accepted_{0}, rejected_{0};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr frozen_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  rclcpp::Subscription<d1max_planning_interfaces::msg::TaggedBspline>::SharedPtr spline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace d1max_trajectory_tracker

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<d1max_trajectory_tracker::TrackerNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory_tracker"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
