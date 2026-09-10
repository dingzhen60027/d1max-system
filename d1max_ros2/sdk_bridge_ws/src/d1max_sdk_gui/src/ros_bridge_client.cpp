#include "d1max_sdk_gui/ros_bridge_client.hpp"

#include <exception>
#include <utility>
#include <vector>

namespace d1max_sdk_gui {

RosBridgeClient::RosBridgeClient(QObject* parent) : QObject(parent) {
  node_ = std::make_shared<rclcpp::Node>("d1max_sdk_gui");
  executor_ =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  CreateInterfaces();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {
    try {
      executor_->spin();
    } catch (const std::exception& error) {
      emit ServiceResult("ros_executor", false,
                         QString::fromStdString(error.what()));
    }
  });
}

RosBridgeClient::~RosBridgeClient() {
  PublishZeroVelocity();
  stopping_.store(true);
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (executor_ && node_) {
    executor_->remove_node(node_);
  }
}

void RosBridgeClient::CreateInterfaces() {
  const std::vector<std::string> trigger_services = {
      "take_control", "release_control", "stand", "lie_down", "crawl",
      "general_mode", "in_place_mode", "stair_mode", "lock",
      "unlock_to_stand", "prepare_navigation", "halt"};
  for (const auto& service : trigger_services) {
    trigger_clients_.emplace(
        service,
        node_->create_client<Trigger>("/d1max_sdk_bridge/" + service));
  }
  estop_client_ = node_->create_client<SetBool>(
      "/d1max_sdk_bridge/soft_estop");
  velocity_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10));

  connection_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/d1max_sdk_bridge/connection_state_text", rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        emit ConnectionChanged(QString::fromStdString(message->data));
      });
  robot_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/d1max_sdk_bridge/robot_state", rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        emit RobotStateReceived(QString::fromStdString(message->data));
      });
  behavior_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/d1max_sdk_bridge/behavior_state", rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        emit BehaviorStateReceived(QString::fromStdString(message->data));
      });
  transition_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/d1max_sdk_bridge/transition_event", rclcpp::QoS(50),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        emit TransitionEventReceived(QString::fromStdString(message->data));
      });
  faults_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/d1max_sdk_bridge/faults", rclcpp::QoS(20),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        emit FaultsReceived(QString::fromStdString(message->data));
      });
  ready_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/d1max_sdk_bridge/ready_for_navigation", rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        emit NavigationReadyChanged(message->data);
      });
}

void RosBridgeClient::CallTrigger(const std::string& service_name) {
  const auto found = trigger_clients_.find(service_name);
  if (found == trigger_clients_.end()) {
    emit ServiceResult(QString::fromStdString(service_name), false,
                       "未知服务");
    return;
  }
  const auto& client = found->second;
  if (!client->service_is_ready()) {
    emit ServiceResult(QString::fromStdString(service_name), false,
                       "桥接服务当前不可用");
    return;
  }
  auto request = std::make_shared<Trigger::Request>();
  client->async_send_request(
      request,
      [this, service_name](rclcpp::Client<Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          emit ServiceResult(QString::fromStdString(service_name),
                             response->success,
                             QString::fromStdString(response->message));
        } catch (const std::exception& error) {
          emit ServiceResult(QString::fromStdString(service_name), false,
                             QString::fromStdString(error.what()));
        }
      });
}

void RosBridgeClient::CallSoftEstop(bool enabled) {
  if (!estop_client_->service_is_ready()) {
    emit ServiceResult("soft_estop", false, "急停服务当前不可用");
    return;
  }
  auto request = std::make_shared<SetBool::Request>();
  request->data = enabled;
  estop_client_->async_send_request(
      request, [this, enabled](rclcpp::Client<SetBool>::SharedFuture future) {
        try {
          const auto response = future.get();
          emit ServiceResult(enabled ? "soft_estop_on" : "soft_estop_off",
                             response->success,
                             QString::fromStdString(response->message));
        } catch (const std::exception& error) {
          emit ServiceResult("soft_estop", false,
                             QString::fromStdString(error.what()));
        }
      });
}

void RosBridgeClient::PublishVelocity(double forward, double lateral,
                                      double yaw) {
  if (stopping_.load() || !velocity_pub_) {
    return;
  }
  geometry_msgs::msg::Twist message;
  message.linear.x = forward;
  message.linear.y = lateral;
  message.angular.z = yaw;
  velocity_pub_->publish(message);
}

void RosBridgeClient::PublishZeroVelocity() {
  PublishVelocity(0.0, 0.0, 0.0);
}

}  // namespace d1max_sdk_gui
