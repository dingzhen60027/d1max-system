#include "d1max_pct_rviz_tools/navigation_diagnostics_panel.hpp"
#include <cmath>
#include <cerrno>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <QButtonGroup>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QRegularExpression>
#include <QSaveFile>
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
#include "rviz_common/tool_manager.hpp"
#include "rviz_common/tool.hpp"
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
bool nonce(const QString & value)
{
  static const QRegularExpression pattern("\\A[0-9a-f]{32}\\z");
  return pattern.match(value).hasMatch();
}
bool privateFile(const struct stat & info)
{
  return S_ISREG(info.st_mode) && info.st_uid == getuid() && !(info.st_mode & 0077);
}
QString readLayoutRequest(const QString & path)
{
  if (!QDir::isAbsolutePath(path)) {return {};}
  const auto encoded = QFile::encodeName(path);
  const int fd = open(encoded.constData(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC | O_NONBLOCK);
  if (fd < 0) {return {};}
  struct stat info{};
  if (fstat(fd, &info) || !privateFile(info) || info.st_size < 1 || info.st_size > 4096) {
    close(fd); return {};
  }
  QByteArray data(4097, Qt::Uninitialized);
  const auto size = read(fd, data.data(), static_cast<size_t>(data.size()));
  close(fd);
  if (size < 1 || size > 4096) {return {};}
  data.resize(static_cast<int>(size));
  return QString::fromUtf8(data);
}
bool writeLayoutState(const QString & path, const QString & request_path, const QJsonObject & value)
{
  const QFileInfo target(path), source(request_path);
  if (!target.isAbsolute() || target.fileName() != "navigation-view-layout-state.json" ||
    source.fileName() != "navigation-view-layout-request.json" ||
    target.absolutePath() != source.absolutePath()) {return false;}
  const QFileInfo parent(target.absolutePath());
  struct stat info{};
  const auto parent_path = QFile::encodeName(parent.absoluteFilePath());
  if (parent.canonicalFilePath() != parent.absoluteFilePath() ||
    lstat(parent_path.constData(), &info) || !S_ISDIR(info.st_mode) ||
    info.st_uid != getuid() || (info.st_mode & 0077)) {return false;}
  const auto encoded = QFile::encodeName(path);
  if (!lstat(encoded.constData(), &info)) {
    if (!privateFile(info)) {return false;}
  } else if (errno != ENOENT) {return false;}
  const auto data = QJsonDocument(value).toJson(QJsonDocument::Compact);
  if (data.size() > 4096) {return false;}
  QSaveFile output(path);
  output.setDirectWriteFallback(false);
  if (!output.open(QIODevice::WriteOnly) ||
    !output.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner) ||
    output.write(data) != data.size()) {return false;}
  return output.commit();
}
}

NavigationDiagnosticsPanel::NavigationDiagnosticsPanel(QWidget * parent)
: Panel(parent), inbox_(std::make_shared<Inbox>()), goal_inbox_(std::make_shared<Inbox>())
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
  task_ = label(content, "等待任务状态", "behavior_tree");
  task_->setMinimumHeight(25);
  form->addRow("任务", task_);
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
  auto * goal_heading = new QHBoxLayout;
  auto * goal_title = label(content, "目标", "goal_editor_title");
  auto goal_font = goal_title->font(); goal_font.setBold(true); goal_title->setFont(goal_font);
  auto * goal_select = new QPushButton("选择目标", content);
  goal_select->setObjectName("goal_select");
  connect(goal_select, &QPushButton::clicked, this, [this] {selectGoalTool();});
  goal_heading->addWidget(goal_title); goal_heading->addStretch();
  goal_heading->addWidget(goal_select);
  layout->addLayout(goal_heading);
  auto * goal_modes = new QHBoxLayout;
  auto * goal_exclusive = new QButtonGroup(this);
  goal_exclusive->setExclusive(true);
  for (int i = 0; i < 2; ++i) {
    const QString key = i == 0 ? "ground" : "free";
    auto * button = new QPushButton(i == 0 ? "贴地" : "XYZ", content);
    button->setObjectName("goal_mode_" + key);
    button->setCheckable(true);
    goal_exclusive->addButton(button); goal_modes_[i] = button;
    connect(button, &QPushButton::clicked, this, [this, key] {chooseGoalMode(key);});
    goal_modes->addWidget(button);
  }
  layout->addLayout(goal_modes);
  auto * coordinates = new QHBoxLayout;
  coordinates->setSpacing(4);
  for (int i = 0; i < 3; ++i) {
    const QString axis = QString("xyz").mid(i, 1);
    auto * column = new QVBoxLayout;
    column->setSpacing(2);
    column->addWidget(label(content, axis.toUpper() + " (m)", "goal_axis"));
    auto * value = new QDoubleSpinBox(content);
    value->setObjectName("live_goal_" + axis);
    value->setRange(-9999., 9999.);
    value->setDecimals(3); value->setSingleStep(.01);
    value->setKeyboardTracking(false); value->setMinimumWidth(65);
    goal_xyz_[i] = value;
    connect(value, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
      [this](double) {goal_dirty_ = true; goal_pending_ = false;
        goal_validation_->setText("目标已修改"); updateGoalWidgets();});
    column->addWidget(value); coordinates->addLayout(column);
  }
  layout->addLayout(coordinates);
  auto * goal_actions = new QHBoxLayout;
  goal_apply_ = new QPushButton("应用", content); goal_apply_->setObjectName("goal_apply");
  goal_snap_ = new QPushButton("贴地", content); goal_snap_->setObjectName("goal_snap_ground");
  connect(goal_apply_, &QPushButton::clicked, this, [this] {applyGoalEdit(false);});
  connect(goal_snap_, &QPushButton::clicked, this, [this] {applyGoalEdit(true);});
  goal_actions->addWidget(goal_apply_); goal_actions->addWidget(goal_snap_);
  layout->addLayout(goal_actions);
  goal_validation_ = label(content, "请选择 3D 目标", "goal_validation");
  layout->addWidget(goal_validation_);
  updateGoalWidgets();
  const QString legends[][2] = {
    {"#00d2e1", "●  实时点云"}, {"#d7dce4", "╋  机身坐标系 · XYZ / 红绿蓝"},
    {"#4191ff", "━  全局路径"}, {"#ff8c14", "━  当前参考段"},
    {"#ff941f", "●  局部目标"}, {"#33e680", "━  局部轨迹 · 机身"},
    {"#688cff", "●  参考锚点"}, {"#a046c8", "━  历史定位轨迹"},
    {"#aab6c4", "■  占据 / 膨胀地图 · Z 高度"}};
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
  layout_request_file_.clear(); layout_state_file_.clear(); last_layout_request_id_.clear();
  config.mapGetString("Layout Request File", &layout_request_file_);
  config.mapGetString("Layout State File", &layout_state_file_);
  viewer_id_ = qEnvironmentVariable("D1MAX_NAV_RVIZ_VIEWER_ID");
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
  resetGoalEditor();
}

void NavigationDiagnosticsPanel::save(rviz_common::Config config) const
{
  Panel::save(config); config.mapSetValue("Session ID", session_);
  config.mapSetValue("Motion Capable", motion_capable_);
  config.mapSetValue("Layout", layout_);
  config.mapSetValue("Layout Request File", layout_request_file_);
  config.mapSetValue("Layout State File", layout_state_file_);
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
  auto goal_inbox = goal_inbox_;
  goal_subscription_ = node->create_subscription<std_msgs::msg::String>(
    "/d1max/live_planning/goal_editor_status", rclcpp::QoS(1).reliable().transient_local(),
    [goal_inbox](std_msgs::msg::String::ConstSharedPtr message) {
      if (message->data.size() > 8192) {return;}
      std::lock_guard<std::mutex> lock(goal_inbox->mutex);
      goal_inbox->value = QString::fromStdString(message->data); ++goal_inbox->sequence;
    });
  goal_edit_publisher_ = node->create_publisher<std_msgs::msg::String>(
    "/d1max/live_planning/edit_goal3d", rclcpp::QoS(1).reliable());
}

void NavigationDiagnosticsPanel::resetGoalEditor()
{
  goal_available_ = goal_dirty_ = goal_pending_ = false;
  goal_source_stamp_ = goal_edit_stamp_ = 0.; goal_mode_ = "ground";
  goal_frame_.clear();
  goal_validation_->setText("请选择 3D 目标"); goal_validation_->setToolTip("");
  updateGoalWidgets();
}

void NavigationDiagnosticsPanel::updateGoalWidgets()
{
  for (int i = 0; i < 2; ++i) {
    const QSignalBlocker block(goal_modes_[i]);
    goal_modes_[i]->setChecked(i == (goal_mode_ == "ground" ? 0 : 1));
    goal_modes_[i]->setEnabled(goal_available_);
  }
  for (int i = 0; i < 3; ++i) {
    goal_xyz_[i]->setEnabled(goal_available_ && (i != 2 || goal_mode_ == "free"));
  }
  goal_apply_->setEnabled(goal_available_ && goal_dirty_ && !goal_pending_);
  goal_snap_->setEnabled(goal_available_ && !goal_pending_);
}

bool NavigationDiagnosticsPanel::publishGoalEdit(const QJsonObject & command)
{
  if (!goal_edit_publisher_) {return false;}
  std_msgs::msg::String message;
  message.data = QJsonDocument(command).toJson(QJsonDocument::Compact).toStdString();
  goal_edit_publisher_->publish(message);
  return true;
}

void NavigationDiagnosticsPanel::applyGoalEdit(bool snap_ground)
{
  if (!goal_available_ || session_.isEmpty() || goal_frame_.isEmpty()) {return;}
  QJsonObject command{{"schema", 1}, {"session_id", session_},
    {"frame_id", goal_frame_}, {"selection_mode", snap_ground ? "ground" : goal_mode_}};
  QJsonArray xyz;
  for (auto * value : goal_xyz_) {xyz.append(value->value());}
  // Include the current draft even for snapping: never snap an older handle
  // while the exact XY fields are waiting to be applied.
  command["goal_xyz"] = xyz;
  if (snap_ground) {command["operation"] = "snap_ground";}
  if (publishGoalEdit(command)) {
    if (snap_ground) {goal_mode_ = "ground";}
    goal_pending_ = true; goal_edit_stamp_ = wallNow();
    goal_validation_->setText("目标更新中"); updateGoalWidgets();
  }
}

void NavigationDiagnosticsPanel::chooseGoalMode(const QString & mode)
{
  if (!goal_available_ || (mode != "ground" && mode != "free")) {return;}
  if (goal_mode_ == mode) {return;}
  goal_mode_ = mode; goal_dirty_ = true; goal_pending_ = false;
  applyGoalEdit(mode == "ground");
  updateGoalWidgets();
}

void NavigationDiagnosticsPanel::selectGoalTool()
{
  // The global presentation owns the goal handles. Switching the layout first
  // avoids focusing an InteractiveMarker hidden by the local presentation.
  chooseLayout("global", true);
  QTimer::singleShot(0, this, [this] {
      if (!getDisplayContext()) {return;}
      auto * tools = getDisplayContext()->getToolManager();
      if (!tools) {return;}
      for (int i = 0; i < tools->numTools(); ++i) {
        auto * tool = tools->getTool(i);
        if (tool && (tool->getClassId() == "d1max_pct_rviz_tools/LiveGoal3D" ||
          tool->getName() == "3D 目标")) {
          tools->setCurrentTool(tool); return;
        }
      }
    });
}

void NavigationDiagnosticsPanel::showGoalEditor(const QString & session_id)
{
  // Tools may focus the same draft from the toolbar, not just this panel.
  // A stale viewer/tool session must not alter another session's presentation.
  if (session_.isEmpty() || session_id != session_) {return;}
  chooseLayout("global", true);
}

void NavigationDiagnosticsPanel::acceptGoalEditorStatus(const QString & json)
{
  if (json.toUtf8().size() > 8192 || session_.isEmpty()) {return;}
  const auto document = QJsonDocument::fromJson(json.toUtf8());
  const auto data = document.object();
  const auto stamp = data["received_at_unix"];
  const auto mode = data["selection_mode"].toString();
  const auto frame = data["frame_id"].toString();
  const auto xyz = data["goal_xyz"].toArray();
  const auto validation = data["goal_validation"].toObject();
  if (!document.isObject() || data["schema"] != QJsonValue(1) ||
    data["mode"].toString() != "LIVE_GOAL_EDITOR_NO_MOTION" ||
    data["motion_enabled"] != QJsonValue(false) || data["session_id"].toString() != session_ ||
    frame.isEmpty() || frame.size() > 128 ||
    (mode != "ground" && mode != "free") || !finite(stamp) ||
    stamp.toDouble() <= goal_source_stamp_ || wallNow()-stamp.toDouble() > 1. ||
    wallNow()-stamp.toDouble() < -.1) {return;}
  if (xyz.isEmpty() && data["goal_xyz"].isNull()) {
    goal_source_stamp_ = stamp.toDouble(); goal_receipt_ = std::chrono::steady_clock::now();
    goal_available_ = false; goal_dirty_ = goal_pending_ = false;
    goal_mode_ = mode; goal_frame_ = frame; goal_validation_->setText("请选择 3D 目标");
    updateGoalWidgets(); return;
  }
  if (xyz.size() != 3 || !validation["valid"].isBool() ||
    !validation["reason"].isString()) {return;}
  for (const auto & value : xyz) {
    if (!finite(value) || std::abs(value.toDouble()) >= 10000.) {return;}
  }
  goal_source_stamp_ = stamp.toDouble(); goal_receipt_ = std::chrono::steady_clock::now();
  goal_available_ = true;
  if (goal_frame_ != frame) {goal_dirty_ = goal_pending_ = false;}
  goal_frame_ = frame;
  // A heartbeat cannot overwrite unsubmitted exact coordinates. An applied
  // draft is reconciled only with a newer matching backend selection.
  bool matches = mode == goal_mode_;
  for (int i = 0; i < 3; ++i) {
    if (mode == "ground" && i == 2) {continue;}
    matches = matches && std::abs(xyz[i].toDouble()-goal_xyz_[i]->value()) <= .001;
  }
  if (goal_pending_ && stamp.toDouble() >= goal_edit_stamp_ && matches) {
    goal_pending_ = false; goal_dirty_ = false;
  }
  if (!goal_dirty_ && !goal_pending_) {
    goal_mode_ = mode;
    for (int i = 0; i < 3; ++i) {
      const QSignalBlocker block(goal_xyz_[i]); goal_xyz_[i]->setValue(xyz[i].toDouble());
    }
    const bool valid = validation["valid"].toBool();
    QString text = valid ? "有效" : "无效";
    const auto floor = validation["floor_id"].toString().left(80);
    if (!floor.isEmpty()) {text += " · " + floor;}
    if (finite(validation["layer_id"])) {
      text += QString(" · slice %1").arg(validation["layer_id"].toInt());
    }
    if (finite(validation["height_error_m"])) {
      text += QString(" · ΔZ %1 m").arg(validation["height_error_m"].toDouble(), 0, 'f', 3);
    }
    goal_validation_->setText(text);
    QString detail = validation["reason"].toString().left(300);
    if (finite(validation["ground_z"])) {
      detail += QString(" · 地面 Z %1 m").arg(validation["ground_z"].toDouble(), 0, 'f', 3);
    }
    goal_validation_->setToolTip(detail);
  }
  updateGoalWidgets();
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
  const auto tree = data["behavior_tree"].toObject();
  const auto task_label = tree["label"].toString().left(30);
  task_->setText(task_label.isEmpty() ? "等待任务状态" : task_label);
  task_->setToolTip(tree["detail"].toString().left(500));
  if (!tree.isEmpty()) {
    detail += "任务: " + tree["task_id"].toString().left(128) + "\n";
    detail += "行为树: " + tree["root_status"].toString().left(16) + " · " +
      tree["detail"].toString().left(500) + "\n";
    const auto nodes = tree["nodes"].toArray();
    for (int i = 0; i < nodes.size() && i < 24; ++i) {
      const auto node = nodes[i].toObject();
      detail += "  " + node["name"].toString().left(160) + ": " +
        node["status"].toString().left(16) + "\n";
    }
  }
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
  task_->setText("等待任务状态"); task_->setToolTip("");
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
  pollLayoutRequest();
  QString value;
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    if (inbox_->sequence != consumed_) {consumed_ = inbox_->sequence; value = inbox_->value;}
  }
  if (!value.isEmpty()) {acceptStatus(value);}
  QString goal_value;
  {
    std::lock_guard<std::mutex> lock(goal_inbox_->mutex);
    if (goal_inbox_->sequence != goal_consumed_) {
      goal_consumed_ = goal_inbox_->sequence; goal_value = goal_inbox_->value;
    }
  }
  if (!goal_value.isEmpty()) {acceptGoalEditorStatus(goal_value);}
  if (goal_pending_ && wallNow()-goal_edit_stamp_ > 3.) {
    goal_pending_ = false; goal_dirty_ = true;
    goal_validation_->setText("目标未更新"); updateGoalWidgets();
  }
  if (goal_available_ && (wallNow()-goal_source_stamp_ > 1. ||
    wallNow()-goal_source_stamp_ < -.1 ||
    std::chrono::steady_clock::now()-goal_receipt_ > std::chrono::seconds(1))) {
    goal_available_ = false; goal_pending_ = false;
    goal_validation_->setText("目标状态未连接"); updateGoalWidgets();
  }
  if (valid_ && (wallNow()-source_stamp_ > 1.0 || wallNow()-source_stamp_ < -.1 ||
    std::chrono::steady_clock::now()-receipt_ > std::chrono::seconds(1)))
  {valid_ = false; unavailable();}
}

bool NavigationDiagnosticsPanel::acceptLayoutRequest(const QString & json)
{
  if (json.toUtf8().size() > 4096 || session_.isEmpty() || !nonce(viewer_id_)) {return false;}
  const auto document = QJsonDocument::fromJson(json.toUtf8());
  auto request = document.object();
  const auto id = request["request_id"].toString();
  const auto layout = request["layout"].toString();
  const auto stamp = request["stamp"];
  const auto age = wallNow() - stamp.toDouble();
  if (!document.isObject() || request["schema"] != QJsonValue(1) ||
    request["session_id"].toString() != session_ ||
    request["viewer_id"].toString() != viewer_id_ || !nonce(id) ||
    id == last_layout_request_id_ || (layout != "global" && layout != "local") ||
    !finite(stamp) || age < -.1 || age > 10.) {return false;}
  // This is an explicit presentation request, separate from status heartbeats.
  // Re-selecting the active layout acknowledges it without moving the camera.
  // Consume before acknowledging: an unwritable state file must not make an
  // old request repeatedly undo a later manual camera/layout selection.
  last_layout_request_id_ = id;
  if (layout != layout_) {chooseLayout(layout, true);}
  QJsonObject state;
  for (const auto * key : {"schema", "session_id", "viewer_id", "request_id", "layout", "stamp"}) {
    state[key] = request[key];
  }
  state["phase"] = "applied";
  state["applied_at_unix"] = wallNow();
  state["rviz_pid"] = static_cast<int>(getpid());
  if (!writeLayoutState(layout_state_file_, layout_request_file_, state)) {return false;}
  return true;
}

void NavigationDiagnosticsPanel::pollLayoutRequest()
{
  if (!nonce(viewer_id_) || layout_request_file_.isEmpty() || layout_state_file_.isEmpty()) {return;}
  const auto json = readLayoutRequest(layout_request_file_);
  if (!json.isEmpty()) {acceptLayoutRequest(json);}
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
  // Finish only the active mouse tool. A pending goal/initial-pose gesture
  // must not follow the user into another camera; the task and route stay
  // owned by the navigation graph and are never changed here.
  auto * tools = getDisplayContext()->getToolManager();
  if (tools && tools->getDefaultTool()) {tools->setCurrentTool(tools->getDefaultTool());}
  chooseView(layout == "local" ? 1 : 2);
  // The saved local camera is sized from the actual rolling map. Do not
  // silently overwrite it with a fixed 10 m distance when switching layouts.
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
  // DisplayGroup toggles propagate to children. Each preset explicitly
  // restores raw occupancy to optional/debug-only, after toggling its parent.
  enabled(group("局部规划"), "原始占据地图（调试）", false);
  enabled(group("局部规划"), "滑动占据地图（高度）", false);
  enabled(group("定位"), "机器狗坐标系", true);
  enabled(group("定位"), "定位轨迹", !local);
  enabled(group("定位"), "初值预览 · 未确认", !local);
  enabled(group("全局规划"), "全局路径", true);
  enabled(group("全局规划"), "3D 目标手柄", !local);
  enabled(group("全局规划"), "目标支撑面", !local);
  if (auto * path = childDisplay(group("全局规划"), "全局路径")) {
    for (int i = 0; i < path->numChildren(); ++i) {
      auto * property = path->childAt(i);
      if (property->getName() == "Line Width") {property->setValue(local ? .035 : .065);}
      if (property->getName() == "Alpha") {property->setValue(local ? .35 : 1.);}
    }
  }
  // Restore the current-heading C-space display in either preset's tree.
  for (const auto & name : {"实时点云", "膨胀占据地图（当前朝向 · 高度）", "碰撞膨胀地图（高度）", "滑动窗口边界", "跟踪路段与局部目标",
      "搜索尝试与受阻位置", "局部轨迹", "已提交局部轨迹"})
  {enabled(group("局部规划"), name, true);}
}
}  // namespace d1max_pct_rviz_tools

PLUGINLIB_EXPORT_CLASS(d1max_pct_rviz_tools::NavigationDiagnosticsPanel, rviz_common::Panel)
