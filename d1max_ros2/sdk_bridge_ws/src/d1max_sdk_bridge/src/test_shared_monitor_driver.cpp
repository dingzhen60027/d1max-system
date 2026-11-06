// Non-installed integration target: production Monitor, fake SDK only. No
// vendor shared library and no production SessionLease are linked/acquired.
#define D1MAX_SHARED_MONITOR_TEST_DRIVER
#include "sdk_monitor_bridge.cpp"
int main(int argc,char**argv){
  if(!std::getenv("D1MAX_SHARED_MONITOR_TEST"))return 80;
  rclcpp::init(argc,argv);auto node=std::make_shared<Monitor>();node->start_execution_workers();
  rclcpp::spin(node);node.reset();if(rclcpp::ok())rclcpp::shutdown();return 0;
}
