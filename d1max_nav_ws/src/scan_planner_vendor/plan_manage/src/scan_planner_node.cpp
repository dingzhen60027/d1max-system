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
    // Grid fusion, reference solving/adoption and normal FSM callbacks remain
    // in ONE mutually-exclusive default owner group. Only the transaction
    // ledger permit/revocation callback has a separate group and does not
    // access live map/query buffers. Validation has its private snapshot thread.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{},2);
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize SCAN-Planner: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
