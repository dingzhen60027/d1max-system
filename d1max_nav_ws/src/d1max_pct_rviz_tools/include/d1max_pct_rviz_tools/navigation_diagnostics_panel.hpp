#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <QString>
#include "rviz_common/panel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class QLabel;
class QPushButton;
class QDoubleSpinBox;
class QJsonObject;
namespace rviz_common {class DisplayGroup;}

namespace d1max_pct_rviz_tools
{
// Navigation presentation and goal drafts; never navigation/motion authorization.
class NavigationDiagnosticsPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit NavigationDiagnosticsPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;
  void acceptStatus(const QString & json);
  void acceptGoalEditorStatus(const QString & json);
  void showGoalEditor(const QString & session_id);
  bool acceptLayoutRequest(const QString & json);
  void refresh();
  // Presentation only: this never publishes a navigation or motion request.
  static void applyLayoutVisibility(rviz_common::DisplayGroup * root, const QString & layout);

protected:
  virtual void applyLayoutPresentation(const QString & layout);
  virtual bool publishGoalEdit(const QJsonObject & command);

private:
  struct Inbox {std::mutex mutex; QString value; uint64_t sequence{0};};
  std::shared_ptr<Inbox> inbox_;
  std::shared_ptr<Inbox> goal_inbox_;
  uint64_t consumed_{0};
  uint64_t goal_consumed_{0};
  QString session_;
  double source_stamp_{0};
  std::chrono::steady_clock::time_point receipt_{};
  bool valid_{false};
  bool motion_capable_{false};
  QString layout_{"global"};
  QString layout_request_file_;
  QString layout_state_file_;
  QString viewer_id_;
  QString last_layout_request_id_;
  std::array<QPushButton *, 2> layout_buttons_{};
  QLabel * mode_{};
  QLabel * task_{};
  std::array<QLabel *, 3> stages_{};
  QLabel * admission_{};
  QLabel * ages_{};
  QLabel * target_{};
  QLabel * horizon_{};
  QLabel * detail_{};
  std::array<QPushButton *, 2> goal_modes_{};
  std::array<QDoubleSpinBox *, 3> goal_xyz_{};
  QPushButton * goal_apply_{};
  QPushButton * goal_snap_{};
  QLabel * goal_validation_{};
  QString goal_mode_{"ground"};
  QString goal_frame_;
  bool goal_available_{false};
  bool goal_dirty_{false};
  bool goal_pending_{false};
  double goal_source_stamp_{0};
  double goal_edit_stamp_{0};
  std::chrono::steady_clock::time_point goal_receipt_{};
  void updateGoalWidgets();
  void chooseGoalMode(const QString & mode);
  void selectGoalTool();
  void applyGoalEdit(bool snap_ground);
  void resetGoalEditor();
  void unavailable();
  void chooseView(int index);
  void chooseLayout(const QString & layout, bool notify);
  void updateLayoutWidgets();
  void pollLayoutRequest();
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_edit_publisher_;
};
}  // namespace d1max_pct_rviz_tools
