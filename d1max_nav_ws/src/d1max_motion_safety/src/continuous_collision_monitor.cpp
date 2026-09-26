#include "d1max_motion_safety/continuous_collision_monitor.hpp"

#include <utility>

namespace d1max_motion_safety
{

ContinuousCollisionMonitor::ContinuousCollisionMonitor(const rclcpp::NodeOptions & options)
: nav2_collision_monitor::CollisionMonitor(options)
{
}

nav2_util::CallbackReturn ContinuousCollisionMonitor::on_configure(
  const rclcpp_lifecycle::State & state)
{
  const auto result = nav2_collision_monitor::CollisionMonitor::on_configure(state);
  if (result != nav2_util::CallbackReturn::SUCCESS) {
    return result;
  }

  // Native configuration has already validated and declared the topic. Retain
  // its depth-1 subscription and all normal topic remapping / lifecycle behavior.
  const auto topic = get_parameter("cmd_vel_in_topic").as_string();
  cmd_vel_in_sub_.reset();
  cmd_vel_in_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    topic, 1,
    [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {receiveNewInput(std::move(msg));});
  return result;
}

void ContinuousCollisionMonitor::receiveNewInput(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  // A real input, including a zero command, permits the *native* processing path
  // to publish its result. No timer, cached command or direct publisher exists.
  stop_stamp_ = now();
  nav2_collision_monitor::CollisionMonitor::cmdVelInCallback(std::move(msg));
}

}  // namespace d1max_motion_safety
