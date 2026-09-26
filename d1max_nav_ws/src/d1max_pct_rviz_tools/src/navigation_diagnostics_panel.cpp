#include "d1max_pct_rviz_tools/navigation_diagnostics_panel.hpp"
#include <cmath>
#include <QButtonGroup>
#include <QDateTime>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/display_group.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/view_controller.hpp"

namespace d1max_pct_rviz_tools
{
namespace
{
double wallNow() {return QDateTime::currentMSecsSinceEpoch() / 1000.0;}
bool finite(const QJsonValue & value)
{return value.isDouble() && std::isfinite(value.toDouble());}
QString age(const QJsonValue & value)
{
  return finite(value) && value.toDouble() >= 0 && value.toDouble() < 100 ?
         QString::number(value.toDouble() * 1000, 'f', 0) + " ms" : "—";
}
QLabel * label(QWidget * parent, const QString & text, const char * name)
{
  auto * value = new QLabel(text, parent);
  value->setObjectName(name);
  value->setTextFormat(Qt::PlainText);
  value->setWordWrap(true);
  return value;
}
rviz_common::Display * childDisplay(rviz_common::DisplayGroup * group, const QString & name)
{
  if (!group) {return nullptr;}
  for (int i = 0; i < group->numDisplays(); ++i) {
    auto * display = group->getDisplayAt(i);
    if (display && display->getName() == name) {return display;}
  }
  return nullptr;
}
void enabled(rviz_common::DisplayGroup * group, const QString & name, bool value)
{
  if (auto * display = childDisplay(group, name)) {display->setEnabled(value);}
}
}

NavigationDiagnosticsPanel::NavigationDiagnosticsPanel(QWidget * parent)
: Panel(parent), inbox_(std::make_shared<Inbox>())
{
  setMinimumWidth(275);
  auto * outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  auto * scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  auto * content = new QWidget(scroll);
  scroll->setWidget(content);
  outer->addWidget(scroll);
  auto * layout = new QVBoxLayout(content);
  layout->setContentsMargins(12, 12, 12, 12);
  layout->setSpacing(12);
  auto * title = label(content, "导航监视", "title");
  auto font = title->font(); font.setBold(true); font.setPointSize(font.pointSize() + 2);
  title->setFont(font);
  layout->addWidget(title);
  mode_ = label(content, "仅预览 · 运动关闭", "mode");
  mode_->setStyleSheet("color: #d8a657;");
  layout->addWidget(mode_);
  auto * layouts = new QHBoxLayout;
  auto * exclusive = new QButtonGroup(this);
  exclusive->setExclusive(true);
  for (int i = 0; i < 2; ++i) {
    const QString key = i == 0 ? "global" : "local";
    auto * button = new QPushButton(i == 0 ? "全局规划" : "局部规划", content);
    button->setObjectName("layout_" + key);
    button->setMinimumHeight(36);
    button->setCheckable(true);
    exclusive->addButton(button);
    layout_buttons_[i] = button;
    connect(button, &QPushButton::clicked, this, [this, key] {chooseLayout(key, true);});
    layouts->addWidget(button);
  }
  layout->addLayout(layouts);
  auto * form = new QFormLayout;
  const char * names[] = {"localization", "global", "local"};
  const QString captions[] = {"定位", "全局规划", "局部规划"};
  for (int i = 0; i < 3; ++i) {
    stages_[i] = label(content, "等待数据", names[i]);
    stages_[i]->setMinimumHeight(25);
    form->addRow(captions[i], stages_[i]);
  }
  admission_ = label(content, "状态未知", "navigation_admission");
  admission_->setMinimumHeight(25);
  form->addRow("导航验收", admission_);
  layout->addLayout(form);
  auto * views = new QHBoxLayout;
  auto * spatial = new QToolButton(content);
  spatial->setText("跨层 3D");
  spatial->setObjectName("spatial_view");
  spatial->setToolTip("保留当前图层，切换跨层视角");
  connect(spatial, &QToolButton::clicked, this, [this] {chooseView(2);});
  views->addWidget(spatial);
  views->addStretch();
  layout->addLayout(views);
  const QString legends[][2] = {
    {"#00d2e1", "●  实时点云"}, {"#d7dce4", "╋  机身坐标系 · XYZ / 红绿蓝"},
    {"#4191ff", "━  全局路径"}, {"#ff8c14", "━  当前参考段"},
    {"#ff941f", "●  局部目标"}, {"#33e680", "━  局部轨迹 · 机身"},
    {"#688cff", "●  参考锚点"}, {"#a046c8", "━  历史定位轨迹"},
    {"#aab6c4", "■  滑动占据地图"}};
  // 1 = global, 2 = local; keep each layout's legend limited to its visible layers.
  const int legend_layouts[] = {2, 3, 3, 2, 2, 2, 2, 1, 2};
  auto * legend = new QVBoxLayout;
  legend->setSpacing(4);
  for (int i = 0; i < 9; ++i) {
    const auto & item = legends[i];
    auto * widget = new QWidget(content);
    widget->setObjectName("legend_row");
    widget->setProperty("layout_mask", legend_layouts[i]);
    auto * row = new QHBoxLayout(widget);
    row->setContentsMargins(0, 0, 0, 0);
    auto * swatch = label(widget, item[1].left(1), "legend_swatch");
    swatch->setFixedWidth(18);
    swatch->setStyleSheet("color: " + item[0] + "; font-weight: bold;");
    auto * value = label(widget, item[1].mid(3), "legend");
    row->addWidget(swatch); row->addWidget(value, 1);
    legend->addWidget(widget);
  }
  layout->addLayout(legend);
  ages_ = label(content, "", "ages");
  target_ = label(content, "", "target");
  horizon_ = label(content, "", "horizon");
  layout->addWidget(ages_); layout->addWidget(target_); layout->addWidget(horizon_);
  auto * toggle = new QToolButton(content);
  toggle->setText("诊断详情"); toggle->setCheckable(true); toggle->setArrowType(Qt::RightArrow);
  toggle->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  layout->addWidget(toggle);
  detail_ = label(content, "", "detail");
  detail_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  detail_->hide(); layout->addWidget(detail_);
  connect(toggle, &QToolButton::toggled, this, [this, toggle](bool open) {
      detail_->setVisible(open); toggle->setArrowType(open ? Qt::DownArrow : Qt::RightArrow);
    });
  layout->addStretch();
  updateLayoutWidgets();
  unavailable();
  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &NavigationDiagnosticsPanel::refresh);
  timer->start(100);
}

void NavigationDiagnosticsPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
  config.mapGetString("Session ID", &session_);
  motion_capable_ = false; config.mapGetBool("Motion Capable", &motion_capable_);
  QString requested_layout;
  config.mapGetString("Layout", &requested_layout);
  layout_ = requested_layout == "local" ? "local" : "global";
  updateLayoutWidgets();
  // RViz may load panels before displays and saved cameras. Apply only after
  // that synchronous configuration load has finished, not from a status tick.
  QTimer::singleShot(0, this, [this] {applyLayoutPresentation(layout_);});
  mode_->setText(motion_capable_ ? "定位与规划监视" : "仅预览 · 运动关闭");
  source_stamp_ = 0; valid_ = false; unavailable();
}

void NavigationDiagnosticsPanel::save(rviz_common::Config config) const
{
  Panel::save(config); config.mapSetValue("Session ID", session_);
  config.mapSetValue("Motion Capable", motion_capable_);
  config.mapSetValue("Layout", layout_);
}

void NavigationDiagnosticsPanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  auto inbox = inbox_;
  subscription_ = node->create_subscription<std_msgs::msg::String>(
    "/d1max/live_planning/diagnostics", rclcpp::QoS(1).reliable(),
    [inbox](std_msgs::msg::String::ConstSharedPtr message) {
      if (message->data.size() > 16384) {return;}
      std::lock_guard<std::mutex> lock(inbox->mutex);
      inbox->value = QString::fromStdString(message->data); ++inbox->sequence;
    });
}

void NavigationDiagnosticsPanel::acceptStatus(const QString & json)
{
  if (json.size() > 16384) {return;}
  const auto document = QJsonDocument::fromJson(json.toUtf8());
  const auto data = document.object();
  const auto stamp = data["stamp"];
  if (!document.isObject() || data["schema"].toInt() != 1 || session_.isEmpty() ||
    data["session_id"].toString() != session_ || !finite(stamp) ||
    stamp.toDouble() <= source_stamp_ || wallNow()-stamp.toDouble() > 1.0 ||
    wallNow()-stamp.toDouble() < -.1 || data["motion_enabled"] != QJsonValue(false) ||
    data["mode"].toString() != "LIVE_VISUALIZATION_NO_MOTION" ||
    data["frame_id"].toString() != "d1max_loc_map") {return;}
  const auto stages = data["stages"].toObject();
  const char * keys[] = {"localization", "global", "local"};
  QString detail;
  for (int i = 0; i < 3; ++i) {
    const auto row = stages[keys[i]].toObject();
    const auto tone = row["tone"].toString();
    if (row["label"].toString().isEmpty() || row["label"].toString().size() > 30 ||
      (tone != "ready" && tone != "muted" && tone != "warning" && tone != "error")) {return;}
  }
  source_stamp_ = stamp.toDouble(); receipt_ = std::chrono::steady_clock::now(); valid_ = true;
  for (int i = 0; i < 3; ++i) {
    const auto row = stages[keys[i]].toObject();
    const auto tone = row["tone"].toString();
    const bool light = palette().color(QPalette::Window).lightness() > 128;
    const QString color = tone == "ready" ? (light ? "#197a52" : "#56cb97") :
      tone == "error" ? (light ? "#b33c43" : "#f2777a") :
      tone == "warning" ? (light ? "#976718" : "#e8b75d") : (light ? "#667180" : "#9ba5b3");
    stages_[i]->setText("●  " + row["label"].toString());
    stages_[i]->setStyleSheet("color: " + color + "; font-weight: 600;");
    const QString reason = row["detail"].toString().left(500);
    stages_[i]->setToolTip(reason);
    if (!reason.isEmpty()) {detail += QString::fromLatin1(keys[i]) + ": " + reason + "\n";}
  }
  // Preview success is not hardware admission. Missing/old contracts never
  // inherit a previous green admission indicator.
  const auto admission = data["navigation_admission"].toObject();
  const bool known = admission["ready"].isBool();
  const bool admitted = known && admission["ready"].toBool() &&
    data["preview_ready"] == QJsonValue(true) &&
    admission["blockers"].isArray() && admission["blockers"].toArray().isEmpty();
  const bool light = palette().color(QPalette::Window).lightness() > 128;
  admission_->setText(admitted ? "●  已通过" : known ? "●  未通过" : "●  状态未知");
  admission_->setStyleSheet("color: " + QString(admitted ? (light ? "#197a52" : "#56cb97") :
    (light ? "#976718" : "#e8b75d")) + "; font-weight: 600;");
  const QString admission_detail = known ? admission["detail"].toString().left(600) :
    "尚未收到导航验收状态";
  admission_->setToolTip(admission_detail);
  if (!admission_detail.isEmpty()) {detail += "导航验收: " + admission_detail + "\n";}
  if (finite(admission["global_observed_hz"])) {
    detail += QString("融合输出: %1 Hz\n").arg(admission["global_observed_hz"].toDouble(), 0, 'f', 1);
  }
  ages_->setText("里程计 " + age(data["body_age"]) + "  ·  点云 " + age(data["cloud_age"]));
  const auto xyz = data["local_target"].toArray();
  bool has_target = xyz.size() == 3;
  for (const auto & value : xyz) {has_target = has_target && finite(value) && std::abs(value.toDouble()) < 10000;}
  target_->setText(has_target ? QString("目标  %1, %2, %3 m")
    .arg(xyz[0].toDouble(), 0, 'f', 2).arg(xyz[1].toDouble(), 0, 'f', 2).arg(xyz[2].toDouble(), 0, 'f', 2) : "局部目标 —");
  const auto horizon = data["reference_horizon_m"];
  horizon_->setText(finite(horizon) && horizon.toDouble() >= 0 ?
    QString("参考段 %1 m  ·  轨迹 #%2").arg(horizon.toDouble(), 0, 'f', 2).arg(data["plan_id"].toInt()) : "参考段 —");
  detail_->setText((detail + data["notice"].toString().left(500)).trimmed());
}

void NavigationDiagnosticsPanel::unavailable()
{
  for (auto * value : stages_) {
    value->setText("●  等待数据"); value->setStyleSheet("color: #9ba5b3;"); value->setToolTip("");
  }
  admission_->setText("●  状态未知"); admission_->setStyleSheet("color: #9ba5b3;");
  admission_->setToolTip("");
  ages_->setText("里程计 —  ·  点云 —");
  target_->setText("局部目标 —"); horizon_->setText("参考段 —");
  detail_->setText("状态未连接或已超时");
}

void NavigationDiagnosticsPanel::refresh()
{
  QString value;
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    if (inbox_->sequence != consumed_) {consumed_ = inbox_->sequence; value = inbox_->value;}
  }
  if (!value.isEmpty()) {acceptStatus(value);}
  if (valid_ && (wallNow()-source_stamp_ > 1.0 || wallNow()-source_stamp_ < -.1 ||
    std::chrono::steady_clock::now()-receipt_ > std::chrono::seconds(1)))
  {valid_ = false; unavailable();}
}

void NavigationDiagnosticsPanel::chooseView(int index)
{
  if (!getDisplayContext()) {return;}
  auto * manager = getDisplayContext()->getViewManager();
  if (manager && index >= 0 && index < manager->getNumViews()) {
    manager->setCurrentFrom(manager->getViewAt(index));
  }
}

void NavigationDiagnosticsPanel::updateLayoutWidgets()
{
  const bool local = layout_ == "local";
  for (int i = 0; i < 2; ++i) {
    const QSignalBlocker block(layout_buttons_[i]);
    layout_buttons_[i]->setChecked(i == (local ? 1 : 0));
  }
  for (auto * row : findChildren<QWidget *>("legend_row")) {
    row->setVisible((row->property("layout_mask").toInt() & (local ? 2 : 1)) != 0);
  }
  ages_->setVisible(local); target_->setVisible(local); horizon_->setVisible(local);
}

void NavigationDiagnosticsPanel::chooseLayout(const QString & requested, bool notify)
{
  if (requested != "global" && requested != "local") {return;}
  layout_ = requested;
  updateLayoutWidgets();
  applyLayoutPresentation(layout_);
  if (notify) {Q_EMIT configChanged();}
}

void NavigationDiagnosticsPanel::applyLayoutPresentation(const QString & layout)
{
  if (!getDisplayContext()) {return;}
  auto * root = getDisplayContext()->getRootDisplayGroup();
  applyLayoutVisibility(root, layout);
  chooseView(layout == "local" ? 1 : 2);
  if (layout == "local") {
    auto * manager = getDisplayContext()->getViewManager();
    if (manager && manager->getCurrent()) {
      auto * view = manager->getCurrent();
      for (int i = 0; i < view->numChildren(); ++i) {
        auto * property = view->childAt(i);
        if (property->getName() == "Distance") {property->setValue(10.);}
      }
    }
  }
}

void NavigationDiagnosticsPanel::applyLayoutVisibility(
  rviz_common::DisplayGroup * root, const QString & layout)
{
  if (!root || (layout != "global" && layout != "local")) {return;}
  const bool local = layout == "local";
  auto group = [root](const QString & name) {
      return dynamic_cast<rviz_common::DisplayGroup *>(childDisplay(root, name));
    };
  enabled(root, "定位", true);
  enabled(root, "地图", !local);
  enabled(root, "全局规划", true);
  enabled(root, "局部规划", local);
  enabled(group("定位"), "机器狗坐标系", true);
  enabled(group("定位"), "定位轨迹", !local);
  enabled(group("定位"), "初值预览 · 未确认", !local);
  enabled(group("全局规划"), "全局路径", true);
  enabled(group("全局规划"), "3D 目标手柄", !local);
  if (auto * path = childDisplay(group("全局规划"), "全局路径")) {
    for (int i = 0; i < path->numChildren(); ++i) {
      auto * property = path->childAt(i);
      if (property->getName() == "Line Width") {property->setValue(local ? .035 : .065);}
      if (property->getName() == "Alpha") {property->setValue(local ? .35 : 1.);}
    }
  }
  for (const auto & name : {"实时点云", "滑动占据地图（含地面）", "滑动窗口边界", "跟踪路段与局部目标",
      "搜索尝试与受阻位置", "局部轨迹"})
  {enabled(group("局部规划"), name, true);}
}
}  // namespace d1max_pct_rviz_tools

PLUGINLIB_EXPORT_CLASS(d1max_pct_rviz_tools::NavigationDiagnosticsPanel, rviz_common::Panel)
