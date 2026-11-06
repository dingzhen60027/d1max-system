#include <memory>
#include <exception>

#include <rclcpp/rclcpp.hpp>
#include <plan_manage/scan_replan_fsm.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("scan_planner_node");

  try
  {
    scan_planner::SCANReplanFSM planner;
    planner.init(node.get());
    // Groups alone do not isolate executor capacity. The map, planning,
    // control, candidate I/O and display I/O each receive one serial role.
    // RoleExecutors joins every callback before planner/node destruction,
    // including partial thread startup and callback exceptions. Archived
    // map/FSM sharing remains one owner when no separate map group exists.
    scan_planner::RoleExecutors executors(node->get_node_base_interface(),planner.executorRoles());
    executors.spin();
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(node->get_logger(), "SCAN-Planner initialization or executor failure: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
