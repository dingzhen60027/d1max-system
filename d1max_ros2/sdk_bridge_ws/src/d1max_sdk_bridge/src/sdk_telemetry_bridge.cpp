// Receive-only SDK adapter. There are deliberately no command APIs, ROS command
// subscriptions, services, control callbacks, or shutdown movement here.
#include <chrono>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_sdk/sdk_client.hpp"

using Json = nlohmann::json;
using Publisher = rclcpp::Publisher<std_msgs::msg::String>;

static void Publish(const Publisher::SharedPtr& publisher, const Json& value) {
  if (!rclcpp::ok()) return;
  std_msgs::msg::String message;
  message.data = value.dump();  // Non-finite numeric values serialize as null.
  publisher->publish(message);
}

class TelemetryCallback final : public robot_sdk::IDataCallback {
 public:
  TelemetryCallback(Publisher::SharedPtr state, Publisher::SharedPtr faults)
      : state_(std::move(state)), faults_(std::move(faults)) {}
  void OnRobotStateData(const robot_sdk::RobotState& data) override {
    Publish(state_, {
      {"source", "sdk_passive_robot_state"}, {"read_only", true},
      {"received_at_unix", std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch()).count()},
      {"motion_status", static_cast<int>(data.motion_status)},
      {"sport_mode", static_cast<int>(data.sport_mode)},
      {"control_source", static_cast<int>(data.control_source)},
      {"speed_level", static_cast<int>(data.speed_level)},
      {"software_emergency_status", static_cast<int>(data.software_emergency_status)},
      {"hardware_emergency_status", static_cast<int>(data.hardware_emergency_status)},
      {"battery_power_1", data.battery.power1}, {"battery_power_2", data.battery.power2},
      {"forward_speed", data.speed.line}, {"lateral_speed", data.speed.translation},
      {"yaw_speed", data.speed.angle}, {"head_angle", data.head_angle},
      {"head_direction", static_cast<int>(data.head_direction)},
      {"mileage", data.mile_data}, {"joint_temperatures", data.joint_temps}
    });
  }
  void OnFaultData(const robot_sdk::FaultDatas& data) override {
    if (data.empty()) return;
    Json result = Json::array();
    for (const auto& fault : data) result.push_back({
      {"level", static_cast<int>(fault.level)},
      {"code", static_cast<int>(fault.code)}, {"message", fault.message}});
    Publish(faults_, result);
  }
 private:
  Publisher::SharedPtr state_, faults_;
};

class TelemetryBridge final : public rclcpp::Node {
 public:
  TelemetryBridge() : Node("d1max_sdk_telemetry", rclcpp::NodeOptions()
      .start_parameter_services(false).start_parameter_event_publisher(false)
      .enable_rosout(false)) {
    const auto ip = declare_parameter<std::string>("robot_ip", "192.168.168.168");
    const auto port = declare_parameter<int>("robot_port", 8081);
    auto state = create_publisher<std_msgs::msg::String>("/d1max_sdk_bridge/robot_state", 10);
    auto faults = create_publisher<std_msgs::msg::String>("/d1max_sdk_bridge/faults", 10);
    connection_ = create_publisher<std_msgs::msg::String>("/d1max_sdk_bridge/connection_state_text", 2);
    behavior_ = create_publisher<std_msgs::msg::String>("/d1max_sdk_bridge/behavior_state", 2);
    callback_ = std::make_shared<TelemetryCallback>(state, faults);
    robot_sdk::ConnectionConfig config;
    config.auto_reconnect = true;
    config.reconnect_interval_ms = 2000;
    sdk_ = std::make_unique<robot_sdk::SDKClient>([logger=get_logger()](const std::error_code& error) {
      if(error) RCLCPP_WARN(logger, "SDK telemetry connection: %s", error.message().c_str());
    }, config);
    sdk_->SetDataCallback(callback_);
    RCLCPP_INFO(get_logger(), "Passive SDK telemetry only: %s:%ld; no control interfaces", ip.c_str(), static_cast<long>(port));
    const auto error = sdk_->Connect(ip, std::to_string(port), true);
    if (error) RCLCPP_WARN(get_logger(), "Connect: %s", error.message().c_str());
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
      std_msgs::msg::String message;
      message.data = sdk_->IsConnected() ? "connected" : "disconnected";
      connection_->publish(message);
      // This is adapter mode, not a fabricated robot/navigation readiness state.
      Publish(behavior_, {{"fsm_state", "READ_ONLY"}, {"telemetry_only", true},
          {"ready_for_navigation", nullptr}, {"fault_latched", nullptr}});
    });
  }
  ~TelemetryBridge() override {
    timer_->cancel();
    // Only disconnect this observer. Never stop or release another controller.
    sdk_->Disconnect(true);
    sdk_.reset();
    callback_.reset();
  }
 private:
  Publisher::SharedPtr connection_, behavior_;
  std::shared_ptr<TelemetryCallback> callback_;
  std::unique_ptr<robot_sdk::SDKClient> sdk_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TelemetryBridge>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
