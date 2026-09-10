#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include <QObject>
#include <QString>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace d1max_sdk_gui {

class RosBridgeClient final : public QObject {
  Q_OBJECT

 public:
  explicit RosBridgeClient(QObject* parent = nullptr);
  ~RosBridgeClient() override;

  void CallTrigger(const std::string& service_name);
  void CallSoftEstop(bool enabled);
  void PublishVelocity(double forward, double lateral, double yaw);
  void PublishZeroVelocity();

 signals:
  void ConnectionChanged(const QString& state);
  void RobotStateReceived(const QString& json);
  void BehaviorStateReceived(const QString& json);
  void TransitionEventReceived(const QString& event);
  void FaultsReceived(const QString& json);
  void NavigationReadyChanged(bool ready);
  void ServiceResult(const QString& service, bool success,
                     const QString& message);

 private:
  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;

  void CreateInterfaces();

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> stopping_{false};

  std::unordered_map<std::string,
                     rclcpp::Client<Trigger>::SharedPtr> trigger_clients_;
  rclcpp::Client<SetBool>::SharedPtr estop_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr connection_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr behavior_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr transition_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr faults_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
};

}  // namespace d1max_sdk_gui
