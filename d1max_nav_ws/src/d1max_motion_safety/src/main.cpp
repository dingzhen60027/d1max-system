#include <memory>

#include "d1max_motion_safety/continuous_collision_monitor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto monitor = std::make_shared<d1max_motion_safety::ContinuousCollisionMonitor>();
  // Input callbacks and lifecycle changes remain serialized, as in native Humble.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(monitor->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
