#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <QJsonObject>
#include <QString>
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"

class QLabel;
class QPushButton;

namespace d1max_pct_rviz_tools
{
// Publishes session-scoped operator intent only. Never creates an SDK client
// or calls robot, arm, posture or emergency-stop services.
class MotionControlPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit MotionControlPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;
  void acceptStatus(const QString & json);
  void refresh();

protected:
  virtual bool confirmExecution(double max_speed, double max_yaw);
  virtual bool publishCommand(const QJsonObject & command);
  virtual double wallTime() const;
  virtual double monotonicTime() const;

private:
  struct Inbox {
    std::mutex mutex;
    QString value;
    uint64_t sequence{0};
    double received_at{0.};
  };
  void acceptStatusAt(const QString & json, double received_at);
  bool fresh() const;
  bool canExecute() const;
  void updateButtons();
  void unavailable(const QString & reason);
  void execute();
  void stop();
  bool send(const QString & action);

  std::shared_ptr<Inbox> inbox_;
  uint64_t consumed_{0};
  QString session_;
  uint64_t generation_{0};
  double source_stamp_{0.}, receipt_{0.}, max_speed_{0.}, max_yaw_{0.};
  double command_stamp_{0.};
  bool valid_{false}, can_execute_{false}, armed_{false}, single_floor_{false};
  bool pending_{false}, confirming_{false};
  QString blockers_;
  QLabel * phase_{};
  QLabel * velocity_{};
  QLabel * limits_{};
  QLabel * detail_{};
  QLabel * notice_{};
  QPushButton * execute_{};
  QPushButton * stop_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
}  // namespace d1max_pct_rviz_tools
