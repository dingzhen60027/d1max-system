#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <vector>
#include <QString>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class QDoubleSpinBox;
class QComboBox;
class QLabel;
class QPushButton;
class QJsonObject;

namespace d1max_pct_rviz_tools
{
class PlanningPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit PlanningPanel(QWidget * parent = nullptr);
  void onInitialize() override;

  // Public UI boundary also supports deterministic tests without a ROS graph or robot.
  void acceptStatus(const QString & json);
  struct OverviewCamera
  {
    std::array<double, 3> center{};
    double distance{8.0};
  };
  static OverviewCamera overviewCamera(
    const std::vector<std::array<double, 3>> & points, double vertical_fov, double aspect);
protected:
  virtual bool publishModeRequest(const QString & mode);
  virtual bool publishLayerRequest(int layer);

private:
  using WallClock = std::chrono::steady_clock;
  struct Endpoint
  {
    std::array<QDoubleSpinBox *, 3> xyz{};
    QPushButton * place{};
    QPushButton * apply{};
    QPushButton * snap{};
    QPushButton * focus{};
    QPushButton * restore{};
    QPushButton * reset_orientation{};
    QLabel * summary{};
    QLabel * orientation{};
    QLabel * validation{};
    QLabel * height_note{};
    std::array<double, 3> canonical{};
    std::array<double, 3> sent{};
    bool selected{false};
    bool dirty{false};
    bool pending{false};
    bool placing{false};
    bool context_conflict{false};
    bool validation_known{false};
    bool valid_location{false};
    bool requires_grounding{false};
    QString sent_mode;
    WallClock::time_point sent_at{};
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr snap_publisher;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr activation_publisher;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_orientation_publisher;
  };
  struct Inbox
  {
    std::mutex mutex;
    QString status;
    uint64_t sequence{0};
    std::vector<std::array<double, 3>> path_bounds;
    QString path_frame;
  };
  QWidget * buildEndpoint(const QString & title, int index);
  void refreshButtons();
  void readInbox();
  void setEndpoint(int index, const QJsonObject & status);
  void applyEndpoint(int index);
  void placeEndpoint(int index);
  void restoreEndpoint(int index);
  void focusEndpoint(int index);
  void showOverview();
  void requestMode(const QString & mode);
  void requestLayer(int layer);
  void updateSelectionContext(const QJsonObject & status);
  void syncSelectionSelectors();
  bool hasDraft() const;
  void sendEmpty(const rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr & publisher);
  std::array<Endpoint, 2> endpoints_;
  QLabel * status_label_{};
  QLabel * frame_label_{};
  QLabel * selection_note_{};
  QLabel * interaction_hint_{};
  QComboBox * mode_selector_{};
  QComboBox * layer_selector_{};
  QPushButton * plan_{};
  QPushButton * clear_{};
  QPushButton * overview_{};
  QString frame_;
  QString state_;
  QString selection_mode_{"ground"};
  int active_layer_{-1};
  bool selection_pending_{false};
  QString requested_mode_;
  int requested_layer_{-1};
  QString selection_error_;
  WallClock::time_point selection_sent_at_{};
  bool server_ready_{false};
  bool valid_status_{false};
  bool clear_pending_{false};
  WallClock::time_point last_status_{};
  WallClock::time_point clear_sent_at_{};
  std::shared_ptr<Inbox> inbox_;
  uint64_t consumed_{0};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr plan_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr layer_publisher_;
};
}  // namespace d1max_pct_rviz_tools
