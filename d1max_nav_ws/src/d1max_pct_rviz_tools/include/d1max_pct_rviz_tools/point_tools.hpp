#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <QString>
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/tool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace d1max_pct_rviz_tools
{
class PreviewPointTool : public rviz_common::Tool
{
public:
  PreviewPointTool(QString label, QString topic, char shortcut, QString icon,
    QString status_topic = "/d1max/pct_preview/status",
    QString status_mode = "GLOBAL_PATH_PREVIEW_ONLY");
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel) override;
  void acceptStatus(const QString & json);
protected:
  virtual bool publishActivationRequest();
  virtual void focusAndReturn(const std::array<double, 3> & xyz);
private:
  using WallClock = std::chrono::steady_clock;
  struct Inbox
  {
    std::mutex mutex;
    QString status;
    uint64_t sequence{0};
  };
  void tick();
  void returnToInteraction();
  QString label_;
  QString topic_;
  QString icon_;
  QString role_;
  QString status_topic_;
  QString status_mode_;
  rviz_common::properties::StringProperty * session_property_{nullptr};
  QString selection_mode_{"ground"};
  std::shared_ptr<Inbox> inbox_;
  uint64_t consumed_{0};
  uint64_t status_sequence_{0};
  uint64_t request_status_sequence_{0};
  std::array<double, 3> selected_{};
  bool valid_selection_{false};
  bool active_{false};
  bool request_sent_{false};
  WallClock::time_point started_{};
  WallClock::time_point last_status_{};
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr activation_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
};

class Start3D : public PreviewPointTool
{
public:
  Start3D();
};
class Goal3D : public PreviewPointTool
{
public:
  Goal3D();
};
class LiveGoal3D : public PreviewPointTool
{
public:
  LiveGoal3D();
};
}  // namespace d1max_pct_rviz_tools
