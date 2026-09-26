#pragma once

#include "nav2_collision_monitor/collision_monitor_node.hpp"

namespace d1max_motion_safety
{

// Humble's native lifecycle, input validation, source processing and collision
// algorithm remain authoritative. Only fresh input renews zero-publication time.
class ContinuousCollisionMonitor : public nav2_collision_monitor::CollisionMonitor
{
public:
  explicit ContinuousCollisionMonitor(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  void receiveNewInput(geometry_msgs::msg::Twist::ConstSharedPtr msg);
};

}  // namespace d1max_motion_safety
