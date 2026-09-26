#include "d1max_pct_rviz_tools/point_tools.hpp"

#include <cmath>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QKeyEvent>
#include <QTimer>
#include <QDateTime>
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/tool_manager.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"

namespace d1max_pct_rviz_tools
{
PreviewPointTool::PreviewPointTool(QString label, QString topic, char shortcut, QString icon,
  QString status_topic, QString status_mode)
: label_(std::move(label)), topic_(std::move(topic)), icon_(std::move(icon)),
  role_(icon_), status_topic_(std::move(status_topic)), status_mode_(std::move(status_mode)),
  inbox_(std::make_shared<Inbox>())
{
  shortcut_key_ = shortcut;
  if (status_mode_ == "LIVE_GOAL_EDITOR_NO_MOTION") {
    session_property_ = new rviz_common::properties::StringProperty(
      "Session ID", "", "Owned live view session; no offline topic remapping.", getPropertyContainer());
  }
  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &PreviewPointTool::tick);
  timer->start(40);
}

void PreviewPointTool::onInitialize()
{
  setIcon(QIcon(rviz_common::loadPixmap(
    "package://d1max_pct_rviz_tools/icons/" + icon_ + ".svg")));
  setName(label_);
  setDescription(label_ + ": create/focus an independent XYZ marker; no point-cloud click required.");
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  activation_publisher_ = node->create_publisher<std_msgs::msg::Empty>(topic_.toStdString(), 5);
  auto inbox = inbox_;
  status_subscription_ = node->create_subscription<std_msgs::msg::String>(
    status_topic_.toStdString(), rclcpp::QoS(1).reliable().transient_local(),
    [inbox](std_msgs::msg::String::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(inbox->mutex);
      inbox->status = QString::fromStdString(message->data);
      ++inbox->sequence;
    });
}

void PreviewPointTool::activate()
{
  active_ = true;
  request_sent_ = false;
  started_ = WallClock::now();
  if (context_) {setStatus(label_ + ": 正在显示三轴空间标记，无需点击点云。");}
  // ToolManager is still inside setCurrentTool()->activate(): never switch tools here.
  QTimer::singleShot(0, this, &PreviewPointTool::tick);
}

void PreviewPointTool::deactivate()
{
  active_ = false;
}

void PreviewPointTool::acceptStatus(const QString & json)
{
  ++status_sequence_;
  const auto document = QJsonDocument::fromJson(json.toUtf8());
  const auto status = document.object();
  selection_mode_ = status["selection_mode"].toString("ground");
  valid_selection_ = false;
  if (!document.isObject() || status["mode"].toString() != status_mode_) {return;}
  if (session_property_) {
    const auto stamp = status["received_at_unix"];
    const double age = QDateTime::currentMSecsSinceEpoch() * .001 - stamp.toDouble();
    if (session_property_->getString().isEmpty() ||
      status["session_id"].toString() != session_property_->getString() ||
      status["frame_id"].toString() != "d1max_loc_map" ||
      !status["motion_enabled"].isBool() || status["motion_enabled"].toBool() ||
      !stamp.isDouble() || !std::isfinite(stamp.toDouble()) || age < -.1 || age > 2.) {return;}
  }
  const auto point = status[role_ + "_xyz"].toArray();
  if (point.size() != 3) {return;}
  for (int axis = 0; axis < 3; ++axis) {
    if (!point[axis].isDouble() || !std::isfinite(point[axis].toDouble()) ||
      std::abs(point[axis].toDouble()) > 10000) {return;}
    selected_[axis] = point[axis].toDouble();
  }
  valid_selection_ = true;
  last_status_ = WallClock::now();
}

bool PreviewPointTool::publishActivationRequest()
{
  if (!activation_publisher_ || activation_publisher_->get_subscription_count() == 0) {return false;}
  activation_publisher_->publish(std_msgs::msg::Empty{});
  return true;
}

void PreviewPointTool::tick()
{
  QString update;
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    if (consumed_ != inbox_->sequence) {consumed_ = inbox_->sequence; update = inbox_->status;}
  }
  if (!update.isEmpty()) {acceptStatus(update);}
  if (!active_) {return;}
  if (!request_sent_) {
    request_sent_ = publishActivationRequest();
    if (request_sent_) {request_status_sequence_ = status_sequence_;}
  }
  if (request_sent_ && status_sequence_ > request_status_sequence_ && valid_selection_ &&
    WallClock::now() - last_status_ < std::chrono::seconds(2))
  {
    active_ = false;
    focusAndReturn(selected_);
  } else if (WallClock::now() - started_ > std::chrono::seconds(3)) {
    active_ = false;
    if (context_) {setStatus(label_ + ": 未收到空间标记确认，请检查规划服务。");}
    returnToInteraction();
  }
}

void PreviewPointTool::focusAndReturn(const std::array<double, 3> & xyz)
{
  if (context_) {
    if (session_property_) {
      context_->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/Orbit");
    }
    auto * view = context_->getViewManager()->getCurrent();
    if (view) {
      view->lookAt(xyz[0], xyz[1], xyz[2]);
      for (int i = 0; i < view->numChildren(); ++i) {
        auto * property = view->childAt(i);
        if (property->getName() == "Distance") {property->setValue(8.0); break;}
      }
    }
    setStatus(label_ + (selection_mode_ == "ground" ?
      ": 拖动地面目标 · Z 跟随地面；无需选择雷达点。" :
      ": 中心拖移 · XYZ 箭头微调 · 圆环旋转；无需选择雷达点。"));
  }
  returnToInteraction();
}

void PreviewPointTool::returnToInteraction()
{
  if (context_) {
    auto * manager = context_->getToolManager();
    manager->setCurrentTool(manager->getDefaultTool());
  }
}

int PreviewPointTool::processMouseEvent(rviz_common::ViewportMouseEvent &)
{
  return 0;  // Activation itself places the marker; no surface picking, no Z=0 projection.
}

int PreviewPointTool::processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel)
{
  if (event->key() == Qt::Key_Escape) {
    return Finished;
  }
  return Tool::processKeyEvent(event, panel);
}

Start3D::Start3D()
: PreviewPointTool("3D Start", "/d1max/pct_preview/activate_start", 'b', "start") {}

Goal3D::Goal3D()
: PreviewPointTool("3D Goal", "/d1max/pct_preview/activate_goal", 'n', "goal") {}
LiveGoal3D::LiveGoal3D()
: PreviewPointTool("3D 目标", "/d1max/live_planning/activate_goal3d", 'n', "goal",
    "/d1max/live_planning/goal_editor_status", "LIVE_GOAL_EDITOR_NO_MOTION") {}
}  // namespace d1max_pct_rviz_tools

PLUGINLIB_EXPORT_CLASS(d1max_pct_rviz_tools::Start3D, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(d1max_pct_rviz_tools::Goal3D, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(d1max_pct_rviz_tools::LiveGoal3D, rviz_common::Tool)
