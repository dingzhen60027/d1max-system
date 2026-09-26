#include "d1max_pct_rviz_tools/planning_panel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <OgreCamera.h>
#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QStringList>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/tool_manager.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"

namespace d1max_pct_rviz_tools
{
namespace
{
constexpr const char * kPrefix = "/d1max/pct_preview/";
const std::array<QString, 2> kRoles{"start", "goal"};

QString tomogramReason(const QString & reason)
{
  // Stable backend codes are distinct: e.g. unobserved ceiling is not low ceiling.
  static const std::array<std::pair<const char *, const char *>, 16> labels{{
    {"height_off_surface", "高度不在表面"},
    {"no_traversable_surface", "此处无可通行表面"},
    {"pct_cost_blocked", "高代价区"},
    {"insufficient_headroom", "上方净空不足"},
    {"unobserved_ground", "没有地面观测"},
    {"unobserved_ceiling", "净空尚未观测"},
    {"ambiguous_surface", "表面不明确，请选择编辑层"},
    {"surface_jump", "地面层变化过大"},
    {"outside_map", "超出地图范围"},
    {"invalid_curve", "路径经过不可通行格"},
    {"layer_transition", "路径跨层，请核对连接"},
    {"surface_switch_requires_selection", "地面层变化，请选择编辑层"},
    {"nonadjacent_layer_transition", "路径跨过非相邻分层"},
    {"ground_step", "地面高差超过限制"},
    {"endpoints_too_close", "起终点过近，请移至不同网格"},
    {"native_abi_mismatch", "规划运行库不兼容，需修复后端"}
  }};
  for (const auto & label : labels) {
    if (reason == label.first) {return label.second;}
  }
  return {};
}

QString reasonCode(const QJsonObject & object)
{
  for (const auto * field : {"error_code", "reason_code", "reason"}) {
    const auto code = object[field].toString();
    if (!code.isEmpty()) {return code;}
  }
  return {};
}

QString reasonTooltip(const QJsonObject & object)
{
  QStringList parts;
  const auto append = [&parts](const QString & part) {
      if (!part.isEmpty() && !parts.contains(part)) {parts.append(part);}
    };
  append(reasonCode(object));
  append(object["reason"].toString());
  append(object["detail"].toString());
  const auto details = object["error_details"];
  if (details.isObject() && !details.toObject().isEmpty()) {
    append(QString::fromUtf8(QJsonDocument(details.toObject()).toJson(QJsonDocument::Compact)));
  } else if (details.isArray() && !details.toArray().isEmpty()) {
    append(QString::fromUtf8(QJsonDocument(details.toArray()).toJson(QJsonDocument::Compact)));
  } else {
    append(details.toString());
  }
  return parts.join("\n");
}

QString locationReason(const QString & reason)
{
  const auto lower = reason.toLower();
  const auto official = tomogramReason(lower);
  if (!official.isEmpty()) {return official;}
  if (reason.isEmpty()) {return "待检查";}
  if (lower == "ok" || lower == "valid" || lower == "traversable") {return "可通行";}
  if (lower.contains("out_of_bounds") || lower.contains("outside") || lower.contains("out of map")) {
    return "超出地图范围";
  }
  if (lower.contains("no_surface") || lower.contains("unknown") || lower.contains("support") ||
    lower.contains("unobserved")) {return "缺少可通行表面";}
  if (lower.contains("ceiling") || lower.contains("headroom")) {return "上方净高不足";}
  if (lower.contains("clearance") || lower.contains("margin")) {return "安全边距不足";}
  if (lower.contains("obstacle") || lower.contains("occupied") || lower.contains("blocked")) {return "位置在障碍区";}
  if (lower.contains("slope") || lower.contains("step")) {return "坡度或台阶超限";}
  if (lower.contains("height") || lower.contains("z differs") || lower.contains("off_ground")) {
    return "高度不在表面";
  }
  if (lower.contains("layer") || lower.contains("floor")) {return "位置与当前地图层不符";}
  if (lower.contains("not_selected") || lower.contains("not selected")) {return "未放置";}
  // Retain concise backend-provided Chinese reasons; full text is always in the tooltip.
  return reason.size() <= 24 ? reason : reason.left(23) + "…";
}

QString shortStatus(const QJsonObject & status)
{
  const auto state = status["state"].toString();
  const auto reason = reasonCode(status);
  if (state == "planned") {
    const double length = status["result"].toObject()["length_m"].toDouble(-1);
    return length >= 0 ? QString("路径就绪 · %1 m").arg(length, 0, 'f', 2) : "路径就绪";
  }
  if (state == "planning") {return "正在规划…";}
  if (state == "initializing") {return "正在加载地图…";}
  if (state == "failed" || state == "invalid_selection" || state == "input_rejected") {
    const auto official = tomogramReason(reason.toLower());
    if (!official.isEmpty()) {return official;}
    if (reason.contains("Z differs")) {return "位置离地：调整高度或点击贴地";}
    if (reason.contains("clearance") || reason.contains("support") || reason.contains("outside")) {
      return "位置不可通行：请调整起终点";
    }
    return "规划未完成 · 悬停查看原因";
  }
  if (!status["start_xyz"].isArray()) {return "请放置起点";}
  if (!status["goal_xyz"].isArray()) {return "请放置终点";}
  return "可规划 · 拖动手柄调整位置";
}

QString orientationText(const QJsonValue & value)
{
  const auto q = value.toArray();
  if (q.size() != 4) {return "姿态预览：—";}
  std::array<double, 4> p{};
  double norm = 0;
  for (int i = 0; i < 4; ++i) {
    if (!q[i].isDouble() || !std::isfinite(q[i].toDouble())) {return "姿态预览：—";}
    p[i] = q[i].toDouble();
    norm += p[i] * p[i];
  }
  if (!std::isfinite(norm) || norm < 1e-12) {return "姿态预览：—";}
  for (auto & coordinate : p) {coordinate /= std::sqrt(norm);}
  const auto [x, y, z, w] = p;
  constexpr double degrees = 180.0 / 3.14159265358979323846;
  const double roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * degrees;
  const double pitch = std::asin(std::clamp(2 * (w * y - z * x), -1.0, 1.0)) * degrees;
  const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * degrees;
  return QString("姿态预览  R %1°  P %2°  Y %3°")
    .arg(roll, 0, 'f', 1).arg(pitch, 0, 'f', 1).arg(yaw, 0, 'f', 1);
}
}

PlanningPanel::PlanningPanel(QWidget * parent)
: Panel(parent), inbox_(std::make_shared<Inbox>())
{
  setMinimumWidth(310);
  auto * outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  auto * scroll = new QScrollArea(this);
  scroll->setObjectName("panel_scroll");
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setWidgetResizable(true);
  auto * content = new QWidget(scroll);
  scroll->setWidget(content);
  outer->addWidget(scroll);
  auto * layout = new QVBoxLayout(content);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);
  auto * title = new QLabel("全局路径", this);
  title->setObjectName("panel_title");
  QFont font = title->font();
  font.setPointSize(font.pointSize() + 2);
  font.setBold(true);
  title->setFont(font);
  auto * heading = new QHBoxLayout;
  heading->addWidget(title, 1);
  overview_ = new QPushButton("总览", this);
  overview_->setObjectName("overview");
  overview_->setMinimumHeight(32);
  overview_->setToolTip("总览起终点与完整路径");
  heading->addWidget(overview_);
  layout->addLayout(heading);
  connect(overview_, &QPushButton::clicked, this, &PlanningPanel::showOverview);
  frame_label_ = new QLabel("离线预览 · 不下发运动", this);
  frame_label_->setObjectName("preview_context");
  frame_label_->setToolTip("坐标系：等待地图");
  frame_label_->setWordWrap(true);
  layout->addWidget(frame_label_);
  auto * selection_form = new QFormLayout;
  selection_form->setVerticalSpacing(7);
  mode_selector_ = new QComboBox(this);
  mode_selector_->setObjectName("selection_mode");
  mode_selector_->addItem("贴地移动", "ground");
  mode_selector_->addItem("自由 XYZ", "free");
  mode_selector_->setMinimumHeight(32);
  mode_selector_->setToolTip("贴地移动：Z 跟随当前表面；自由 XYZ：三轴独立拖动。XY 不吸附。");
  selection_form->addRow("选点模式", mode_selector_);
  layer_selector_ = new QComboBox(this);
  layer_selector_->setObjectName("active_layer");
  layer_selector_->setMinimumHeight(32);
  layer_selector_->setToolTip("只影响之后的放置和移动，不搬动已选点。分层切片不等于物理楼层；XY 不吸附。");
  selection_form->addRow("编辑层", layer_selector_);
  layout->addLayout(selection_form);
  selection_note_ = new QLabel("高度跟随当前层表面", this);
  selection_note_->setObjectName("selection_note");
  selection_note_->setWordWrap(true);
  layout->addWidget(selection_note_);
  connect(mode_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [this](int index) {if (index >= 0) {requestMode(mode_selector_->itemData(index).toString());}});
  connect(layer_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [this](int index) {if (index >= 0) {requestLayer(layer_selector_->itemData(index).toInt());}});
  layout->addWidget(buildEndpoint("起点 [B]", 0));
  layout->addWidget(buildEndpoint("终点 [N]", 1));
  auto * step_container = new QWidget(this);
  step_container->setObjectName("numeric_steps");
  auto * step_row = new QHBoxLayout(step_container);
  step_row->setContentsMargins(0, 0, 0, 0);
  step_row->addWidget(new QLabel("数值步长", this));
  auto * step = new QComboBox(this);
  step->addItem("0.01 m", 0.01);
  step->addItem("0.05 m", 0.05);
  step->addItem("0.10 m", 0.10);
  step->setMinimumHeight(30);
  step->setToolTip("仅控制精确坐标输入框的加减步长；鼠标拖动不受此限制。");
  step_row->addWidget(step, 1);
  connect(step, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
    [this, step](int) {
      for (auto & endpoint : endpoints_) {
        for (auto * field : endpoint.xyz) {field->setSingleStep(step->currentData().toDouble());}
      }
    });
  layout->addWidget(step_container);
  step_container->hide();
  for (auto * toggle : findChildren<QToolButton *>()) {
    connect(toggle, &QToolButton::toggled, this, [this, step_container](bool) {
      bool editing = false;
      for (auto * item : findChildren<QToolButton *>()) {editing = editing || item->isChecked();}
      step_container->setVisible(editing);
    });
  }
  plan_ = new QPushButton("规划路径", this);
  plan_->setObjectName("plan");
  plan_->setMinimumHeight(36);
  clear_ = new QPushButton("清空", this);
  clear_->setObjectName("clear");
  clear_->setMinimumHeight(36);
  clear_->setToolTip("清空起终点与路径");
  auto * actions = new QHBoxLayout;
  actions->addWidget(plan_, 2);
  actions->addWidget(clear_, 1);
  layout->addLayout(actions);
  status_label_ = new QLabel("等待规划服务", this);
  status_label_->setObjectName("status");
  status_label_->setWordWrap(true);
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  status_label_->setMinimumHeight(28);
  layout->addWidget(status_label_);
  auto * hint = new QLabel("XYZ 拖动 · 姿态仅预览", this);
  interaction_hint_ = hint;
  hint->setWordWrap(true);
  hint->setObjectName("interaction_hint");
  hint->setToolTip(
    "中心左键拖动：在当前视平面平移。\n"
    "Shift + 中心左拖：沿视线前后移动，不是地图 Z。\n"
    "Ctrl + 中心左拖：绕相机水平/竖直轴旋转。\n"
    "Ctrl + Shift + 中心左拖：绕视线旋转。\n"
    "地图轴精确调整：拖动 XYZ 箭头或对应圆环。\n"
    "仅规划预览，不驱动机器狗。");
  layout->addWidget(hint);
  layout->addStretch();
  connect(plan_, &QPushButton::clicked, this, [this]() {
    sendEmpty(plan_publisher_);
  });
  connect(clear_, &QPushButton::clicked, this, [this]() {
    clear_pending_ = true;
    clear_sent_at_ = WallClock::now();
    for (auto & endpoint : endpoints_) {
      endpoint.dirty = false; endpoint.pending = false; endpoint.placing = false;
      endpoint.context_conflict = false;
      for (auto * field : endpoint.xyz) {const QSignalBlocker blocked(field); field->setValue(0);}
    }
    refreshButtons();
    sendEmpty(clear_publisher_);
  });
  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &PlanningPanel::readInbox);
  timer->start(100);
  refreshButtons();
}

bool PlanningPanel::hasDraft() const
{
  return std::any_of(endpoints_.begin(), endpoints_.end(),
    [](const Endpoint & endpoint) {return endpoint.dirty || endpoint.pending;});
}

bool PlanningPanel::publishModeRequest(const QString & mode)
{
  if (!mode_publisher_) {return false;}
  std_msgs::msg::String message;
  message.data = mode.toStdString();
  mode_publisher_->publish(message);
  return true;
}

bool PlanningPanel::publishLayerRequest(int layer)
{
  if (!layer_publisher_) {return false;}
  std_msgs::msg::Int32 message;
  message.data = layer;
  layer_publisher_->publish(message);
  return true;
}

void PlanningPanel::requestMode(const QString & mode)
{
  if (!valid_status_ || clear_pending_ || selection_pending_ || hasDraft() ||
    (mode != "ground" && mode != "free") || mode == selection_mode_)
  {
    syncSelectionSelectors();
    return;
  }
  if (publishModeRequest(mode)) {
    requested_mode_ = mode;
    requested_layer_ = active_layer_;
    selection_pending_ = true;
    selection_sent_at_ = WallClock::now();
    selection_error_.clear();
  }
  syncSelectionSelectors();
  refreshButtons();
}

void PlanningPanel::requestLayer(int layer)
{
  if (!valid_status_ || clear_pending_ || selection_pending_ || hasDraft() ||
    layer_selector_->findData(layer) < 0 || layer == active_layer_)
  {
    syncSelectionSelectors();
    return;
  }
  if (publishLayerRequest(layer)) {
    requested_mode_ = selection_mode_;
    requested_layer_ = layer;
    selection_pending_ = true;
    selection_sent_at_ = WallClock::now();
    selection_error_.clear();
  }
  syncSelectionSelectors();
  refreshButtons();
}

void PlanningPanel::syncSelectionSelectors()
{
  const QSignalBlocker mode_blocked(mode_selector_);
  const QSignalBlocker layer_blocked(layer_selector_);
  mode_selector_->setCurrentIndex(mode_selector_->findData(selection_pending_ ? requested_mode_ : selection_mode_));
  layer_selector_->setCurrentIndex(layer_selector_->findData(selection_pending_ ? requested_layer_ : active_layer_));
}

void PlanningPanel::updateSelectionContext(const QJsonObject & status)
{
  const QString new_mode = status["selection_mode"].toString("ground");
  const int new_layer = status["active_layer"].toInt(-1);
  const bool changed = !frame_.isEmpty() &&
    (new_mode != selection_mode_ || new_layer != active_layer_ || frame_ != status["frame_id"].toString());
  if (changed) {
    for (auto & endpoint : endpoints_) {
      if (endpoint.dirty || endpoint.pending) {
        endpoint.context_conflict = true;
        endpoint.dirty = true;
        endpoint.pending = false;
      }
    }
  }
  selection_mode_ = new_mode;
  active_layer_ = new_layer;
  if (selection_pending_ && requested_mode_ == selection_mode_ && requested_layer_ == active_layer_) {
    selection_pending_ = false;
    selection_error_.clear();
  }
  std::vector<std::pair<int, QString>> layer_items;
  const auto layers = status["available_layers"].toArray();
  for (int i = 0; i < std::min(layers.size(), 512); ++i) {
    const auto layer = layers[i].toObject();
    const auto id = layer["id"];
    if (!id.isDouble() || !std::isfinite(id.toDouble()) ||
      id.toDouble() != std::floor(id.toDouble()) || id.toDouble() < -1 ||
      id.toDouble() > std::numeric_limits<int>::max()) {continue;}
    const int number = id.toInt();
    if (std::any_of(layer_items.begin(), layer_items.end(),
      [number](const auto & item) {return item.first == number;})) {continue;}
    layer_items.emplace_back(number, layer["label"].toString(QString("分层 #%1").arg(number)));
  }
  bool same_items = static_cast<size_t>(layer_selector_->count()) == layer_items.size();
  for (size_t i = 0; same_items && i < layer_items.size(); ++i) {
    same_items = layer_selector_->itemData(i).toInt() == layer_items[i].first &&
      layer_selector_->itemText(i) == layer_items[i].second;
  }
  // A 5 Hz heartbeat must not rebuild an open dropdown while the user chooses a layer.
  if (!same_items) {
    const QSignalBlocker blocked(layer_selector_);
    layer_selector_->clear();
    for (const auto & [number, label] : layer_items) {
      layer_selector_->addItem(label, number);
    }
  }
  syncSelectionSelectors();
}

QWidget * PlanningPanel::buildEndpoint(const QString & title, int index)
{
  auto * box = new QGroupBox(title, this);
  auto * layout = new QVBoxLayout(box);
  layout->setContentsMargins(8, 10, 8, 8);
  layout->setSpacing(5);
  auto & endpoint = endpoints_[index];
  endpoint.place = new QPushButton(index == 0 ? "放置起点" : "放置终点", box);
  endpoint.place->setObjectName(kRoles[index] + "_place");
  endpoint.place->setMinimumHeight(32);
  endpoint.place->setToolTip("直接显示独立三维空间标记；已存在的位置不会重置。无需选中点云。");
  endpoint.focus = new QPushButton("聚焦", box);
  endpoint.focus->setObjectName(kRoles[index] + "_focus");
  endpoint.focus->setMinimumHeight(32);
  auto * placement = new QHBoxLayout;
  placement->addWidget(endpoint.place, 2);
  placement->addWidget(endpoint.focus, 1);
  layout->addLayout(placement);
  endpoint.summary = new QLabel("未放置", box);
  endpoint.summary->setObjectName(kRoles[index] + "_summary");
  endpoint.summary->setWordWrap(true);
  layout->addWidget(endpoint.summary);
  endpoint.validation = new QLabel("未放置", box);
  endpoint.validation->setObjectName(kRoles[index] + "_validation");
  endpoint.validation->setWordWrap(true);
  layout->addWidget(endpoint.validation);
  endpoint.orientation = new QLabel("姿态预览：—", box);
  endpoint.orientation->setObjectName(kRoles[index] + "_orientation");
  endpoint.orientation->setWordWrap(true);
  endpoint.orientation->setToolTip("R/P/Y 为手柄姿态角；当前路径只约束 XYZ，不约束终点姿态。");
  auto * edit_toggle = new QToolButton(box);
  edit_toggle->setObjectName(kRoles[index] + "_edit_toggle");
  edit_toggle->setText("精确坐标");
  edit_toggle->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  edit_toggle->setArrowType(Qt::RightArrow);
  edit_toggle->setCheckable(true);
  edit_toggle->setMinimumHeight(30);
  auto * endpoint_actions = new QHBoxLayout;
  endpoint_actions->addWidget(edit_toggle, 1);
  layout->addLayout(endpoint_actions);
  auto * edit_body = new QWidget(box);
  edit_body->setObjectName(kRoles[index] + "_edit_body");
  auto * edit_layout = new QVBoxLayout(edit_body);
  edit_layout->setContentsMargins(0, 0, 0, 0);
  auto * coordinates = new QFormLayout;
  coordinates->setVerticalSpacing(7);
  layout->addWidget(edit_body);
  edit_body->hide();
  connect(edit_toggle, &QToolButton::toggled, this, [edit_toggle, edit_body](bool expanded) {
    edit_toggle->setArrowType(expanded ? Qt::DownArrow : Qt::RightArrow);
    edit_body->setVisible(expanded);
  });
  connect(endpoint.place, &QPushButton::clicked, this, [this, index]() {placeEndpoint(index);});
  for (int axis = 0; axis < 3; ++axis) {
    auto * field = new QDoubleSpinBox(box);
    endpoint.xyz[axis] = field;
    field->setObjectName(kRoles[index] + "_" + QString("xyz")[axis]);
    field->setRange(-10000, 10000);
    field->setDecimals(3);
    field->setSingleStep(0.01);
    field->setSuffix(" m");
    field->setMinimumSize(160, 32);
    field->setKeyboardTracking(false);
    coordinates->addRow(QString("XYZ")[axis] + QString("  "), field);
    connect(field, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
      [this, index](double) {endpoints_[index].dirty = true; refreshButtons();});
    // Keyboard tracking is off so partially typed numbers are not committed;
    // still mark drafts immediately, before focus loss or the Apply click.
    connect(field->findChild<QLineEdit *>(), &QLineEdit::textEdited, this,
      [this, index](const QString &) {endpoints_[index].dirty = true; refreshButtons();});
  }
  edit_layout->addLayout(coordinates);
  edit_layout->addWidget(endpoint.orientation);
  endpoint.height_note = new QLabel("Z 跟随表面；应用 XY 后更新高度", edit_body);
  endpoint.height_note->setObjectName(kRoles[index] + "_height_note");
  endpoint.height_note->setWordWrap(true);
  edit_layout->addWidget(endpoint.height_note);
  auto * first = new QHBoxLayout;
  endpoint.apply = new QPushButton("应用坐标", box);
  endpoint.apply->setObjectName(kRoles[index] + "_apply");
  endpoint.restore = new QPushButton("恢复", box);
  endpoint.restore->setObjectName(kRoles[index] + "_restore");
  first->addWidget(endpoint.apply, 2);
  first->addWidget(endpoint.restore, 1);
  edit_layout->addLayout(first);
  endpoint.snap = new QPushButton("贴地", box);
  endpoint.snap->setObjectName(kRoles[index] + "_snap");
  endpoint.snap->setToolTip("仅在明确点击时把 Z 调整到已测地面；不改变 XY。");
  endpoint.reset_orientation = new QPushButton("姿态归零", box);
  endpoint.reset_orientation->setObjectName(kRoles[index] + "_reset_orientation");
  endpoint.reset_orientation->setToolTip("仅重置手柄姿态；不改变 XYZ 和已规划路径。");
  endpoint_actions->addWidget(endpoint.snap);
  edit_layout->addWidget(endpoint.reset_orientation);
  for (auto * button : {endpoint.apply, endpoint.restore, endpoint.snap, endpoint.focus, endpoint.reset_orientation}) {
    button->setMinimumHeight(32);
  }
  connect(endpoint.apply, &QPushButton::clicked, this, [this, index]() {applyEndpoint(index);});
  connect(endpoint.restore, &QPushButton::clicked, this, [this, index]() {restoreEndpoint(index);});
  connect(endpoint.snap, &QPushButton::clicked, this, [this, index]() {
    sendEmpty(endpoints_[index].snap_publisher);
  });
  connect(endpoint.focus, &QPushButton::clicked, this, [this, index]() {focusEndpoint(index);});
  connect(endpoint.reset_orientation, &QPushButton::clicked, this, [this, index]() {
    sendEmpty(endpoints_[index].reset_orientation_publisher);
  });
  return box;
}

void PlanningPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  for (int index = 0; index < 2; ++index) {
    auto & endpoint = endpoints_[index];
    endpoint.publisher = node_->create_publisher<geometry_msgs::msg::PointStamped>(
      std::string(kPrefix) + kRoles[index].toStdString(), rclcpp::QoS(5));
    endpoint.snap_publisher = node_->create_publisher<std_msgs::msg::Empty>(
      std::string(kPrefix) + "snap_" + kRoles[index].toStdString(), rclcpp::QoS(5));
    endpoint.activation_publisher = node_->create_publisher<std_msgs::msg::Empty>(
      std::string(kPrefix) + "activate_" + kRoles[index].toStdString(), rclcpp::QoS(5));
    endpoint.reset_orientation_publisher = node_->create_publisher<std_msgs::msg::Empty>(
      std::string(kPrefix) + "reset_" + kRoles[index].toStdString() + "_orientation", rclcpp::QoS(5));
  }
  plan_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(std::string(kPrefix) + "plan", 5);
  clear_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(std::string(kPrefix) + "clear", 5);
  mode_publisher_ = node_->create_publisher<std_msgs::msg::String>(std::string(kPrefix) + "selection_mode", 5);
  layer_publisher_ = node_->create_publisher<std_msgs::msg::Int32>(std::string(kPrefix) + "layer", 5);
  // Status is canonical: stale transient selected_* samples cannot revive points after Clear.
  // ROS callbacks never touch Qt widgets or capture the panel's lifetime.
  auto inbox = inbox_;
  status_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    std::string(kPrefix) + "status", rclcpp::QoS(1).reliable().transient_local(),
    [inbox](std_msgs::msg::String::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(inbox->mutex);
      inbox->status = QString::fromStdString(message->data);
      ++inbox->sequence;
    });
  path_subscription_ = node_->create_subscription<nav_msgs::msg::Path>(
    std::string(kPrefix) + "path", rclcpp::QoS(1).reliable().transient_local(),
    [inbox](nav_msgs::msg::Path::ConstSharedPtr message) {
      std::array<double, 3> low{}, high{};
      bool initialized = false;
      for (const auto & pose : message->poses) {
        const auto & p = pose.pose.position;
        const std::array<double, 3> point{p.x, p.y, p.z};
        if (!std::all_of(point.begin(), point.end(), [](double v) {return std::isfinite(v);})) {continue;}
        if (!initialized) {low = point; high = point; initialized = true;}
        for (int axis = 0; axis < 3; ++axis) {
          low[axis] = std::min(low[axis], point[axis]);
          high[axis] = std::max(high[axis], point[axis]);
        }
      }
      std::lock_guard<std::mutex> lock(inbox->mutex);
      inbox->path_frame = QString::fromStdString(message->header.frame_id);
      inbox->path_bounds.clear();
      if (initialized) {inbox->path_bounds = {low, high};}
    });
}

void PlanningPanel::readInbox()
{
  QString update;
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    if (consumed_ != inbox_->sequence) {
      consumed_ = inbox_->sequence;
      update = inbox_->status;
    }
  }
  if (!update.isEmpty()) {acceptStatus(update);}
  if (valid_status_ && WallClock::now() - last_status_ > std::chrono::seconds(2)) {
    valid_status_ = false;
    status_label_->setText("规划服务状态超时；操作已禁用");
  }
  if (selection_pending_ && WallClock::now() - selection_sent_at_ > std::chrono::seconds(2)) {
    selection_pending_ = false;
    selection_error_ = "切换未确认，保持原设置";
    syncSelectionSelectors();
  }
  refreshButtons();
}

void PlanningPanel::acceptStatus(const QString & json)
{
  const auto document = QJsonDocument::fromJson(json.toUtf8());
  const auto status = document.object();
  if (!document.isObject() || status["mode"].toString() != "GLOBAL_PATH_PREVIEW_ONLY" ||
    status["frame_id"].toString().isEmpty() ||
    (status["selection_mode"].toString("ground") != "ground" && status["selection_mode"].toString() != "free"))
  {
    valid_status_ = false;
    status_label_->setText("规划服务状态格式异常");
    refreshButtons();
    return;
  }
  updateSelectionContext(status);
  frame_ = status["frame_id"].toString();
  frame_label_->setToolTip("坐标系：" + frame_);
  last_status_ = WallClock::now();
  valid_status_ = true;
  server_ready_ = status["ready"].toBool();
  state_ = status["state"].toString();
  if (clear_pending_ &&
    ((status["start_xyz"].isNull() && status["goal_xyz"].isNull()) ||
    WallClock::now() - clear_sent_at_ > std::chrono::seconds(2)))
  {
    clear_pending_ = false;
  }
  if (!clear_pending_) {
    setEndpoint(0, status);
    setEndpoint(1, status);
  }
  status_label_->setText(shortStatus(status));
  if (tomogramReason(reasonCode(status).toLower()).isEmpty() &&
    std::any_of(endpoints_.begin(), endpoints_.end(), [](const Endpoint & endpoint) {
    return endpoint.selected && endpoint.validation_known && !endpoint.valid_location;
  })) {
    status_label_->setText("请修正标记位置，再规划路径");
  }
  status_label_->setToolTip(reasonTooltip(status));
  refreshButtons();
}

void PlanningPanel::setEndpoint(int index, const QJsonObject & status)
{
  auto & endpoint = endpoints_[index];
  const auto value = status[kRoles[index] + "_xyz"];
  const auto xyz = value.toArray();
  bool selected = value.isArray() && xyz.size() == 3;
  if (selected) {
    for (const auto & coordinate : xyz) {
      selected = selected && coordinate.isDouble() && std::isfinite(coordinate.toDouble()) &&
        std::abs(coordinate.toDouble()) <= 10000;
    }
  }
  const bool was_selected = endpoint.selected;
  endpoint.selected = selected;
  const auto validation = status[kRoles[index] + "_validation"].toObject();
  endpoint.validation_known = validation["valid"].isBool();
  endpoint.valid_location = endpoint.validation_known && validation["valid"].toBool();
  QString layer_name = "所属层待定";
  if (validation["layer_id"].isDouble()) {
    const int layer_id = validation["layer_id"].toInt();
    const int layer_index = layer_selector_->findData(layer_id);
    layer_name = layer_index >= 0 ? layer_selector_->itemText(layer_index) : QString("层 %1").arg(layer_id);
  }
  const auto reason = reasonCode(validation);
  const auto lower_reason = reason.toLower();
  endpoint.requires_grounding = selected && endpoint.validation_known && !endpoint.valid_location &&
    (lower_reason.contains("height") || lower_reason.contains("z differs") || lower_reason.contains("off_ground"));
  endpoint.validation->setText(!selected ? "未放置" :
    layer_name + " · " + (endpoint.valid_location ? "可通行" : locationReason(reason)));
  endpoint.validation->setToolTip(reasonTooltip(validation));
  endpoint.validation->setStyleSheet(selected && endpoint.validation_known && !endpoint.valid_location ?
    "color: #b45309;" : "");
  endpoint.orientation->setText(selected ?
    orientationText(status[kRoles[index] + "_orientation_xyzw"]) : "姿态预览：—");
  if (!selected) {
    endpoint.summary->setText("未放置");
    // With no marker yet, users may type an arbitrary space coordinate and
    // Apply to create one. Repeated null heartbeat must not destroy that draft.
    if (was_selected) {
      endpoint.dirty = false;
      endpoint.pending = false;
      endpoint.context_conflict = false;
      for (auto * field : endpoint.xyz) {const QSignalBlocker blocked(field); field->setValue(0);}
    }
    if (endpoint.pending && WallClock::now() - endpoint.sent_at > std::chrono::seconds(2)) {
      endpoint.pending = false;
      endpoint.dirty = true;
    }
    if (endpoint.placing && WallClock::now() - endpoint.sent_at > std::chrono::seconds(3)) {
      endpoint.placing = false;
    }
    endpoint.canonical = {};
    return;
  }
  bool acknowledgement = true;
  bool focused = false;
  for (int axis = 0; axis < 3; ++axis) {
    endpoint.canonical[axis] = xyz[axis].toDouble();
    if (axis < 2 || endpoint.sent_mode != "ground") {
      acknowledgement = acknowledgement && std::abs(endpoint.sent[axis] - endpoint.canonical[axis]) < 0.0005;
    }
    auto * focus = QApplication::focusWidget();
    focused = focused || endpoint.xyz[axis] == focus || endpoint.xyz[axis]->isAncestorOf(focus);
  }
  endpoint.summary->setText(QString("X %1   Y %2   Z %3 m")
    .arg(endpoint.canonical[0], 0, 'f', 2).arg(endpoint.canonical[1], 0, 'f', 2)
    .arg(endpoint.canonical[2], 0, 'f', 2));
  if (endpoint.pending && (acknowledgement || WallClock::now() - endpoint.sent_at > std::chrono::seconds(2))) {
    endpoint.pending = false;
  }
  if (!endpoint.dirty && !endpoint.pending && !focused) {
    restoreEndpoint(index);
  }
  if (endpoint.placing) {
    endpoint.placing = false;
    focusEndpoint(index);
  }
}

void PlanningPanel::refreshButtons()
{
  const bool online = valid_status_ && !clear_pending_ && !selection_pending_;
  for (auto & endpoint : endpoints_) {
    const bool available = online && endpoint.selected;
    endpoint.place->setEnabled(online && !endpoint.dirty && !endpoint.pending && !endpoint.placing);
    for (auto * field : endpoint.xyz) {field->setEnabled(online);}
    endpoint.xyz[2]->setReadOnly(selection_mode_ == "ground");
    endpoint.height_note->setVisible(selection_mode_ == "ground");
    endpoint.apply->setEnabled(online && (endpoint.dirty || !endpoint.selected) && !endpoint.pending &&
      !endpoint.context_conflict);
    endpoint.restore->setEnabled(online && endpoint.dirty && !endpoint.pending);
    endpoint.snap->setVisible(selection_mode_ == "free" || endpoint.requires_grounding);
    endpoint.snap->setText(selection_mode_ == "ground" ? "贴回表面" : "贴地");
    endpoint.snap->setEnabled(available && !endpoint.dirty && !endpoint.pending);
    endpoint.focus->setEnabled(available);
    endpoint.reset_orientation->setEnabled(available && !endpoint.pending);
  }
  plan_->setEnabled(online && server_ready_ && endpoints_[0].selected && endpoints_[1].selected &&
    !endpoints_[0].dirty && !endpoints_[1].dirty && !endpoints_[0].pending && !endpoints_[1].pending &&
    !endpoints_[0].context_conflict && !endpoints_[1].context_conflict &&
    (!endpoints_[0].validation_known || endpoints_[0].valid_location) &&
    (!endpoints_[1].validation_known || endpoints_[1].valid_location));
  const bool draft = endpoints_[0].dirty || endpoints_[1].dirty;
  const bool pending = endpoints_[0].pending || endpoints_[1].pending;
  const bool conflict = endpoints_[0].context_conflict || endpoints_[1].context_conflict;
  plan_->setText(selection_pending_ ? "等待模式 / 层确认" :
    (conflict ? "先恢复坐标后重新编辑" :
    (draft ? "先应用修改后的坐标" : (pending ? "等待坐标确认" : "规划路径"))));
  clear_->setEnabled(valid_status_ && !clear_pending_);
  overview_->setEnabled(online && (endpoints_[0].selected || endpoints_[1].selected));
  mode_selector_->setEnabled(online && !draft && !pending);
  layer_selector_->setEnabled(online && !draft && !pending && layer_selector_->count() > 0);
  if (selection_pending_) {
    selection_note_->setText("正在切换，请等待服务确认");
  } else if (conflict) {
    selection_note_->setText("模式或层已变化，草稿已保留；请恢复后重新编辑");
  } else if (draft || pending) {
    selection_note_->setText("先应用或恢复坐标，再切换模式 / 层");
  } else if (!selection_error_.isEmpty()) {
    selection_note_->setText(selection_error_);
  } else {
    selection_note_->setText(selection_mode_ == "ground" ? "高度跟随所选表面；XY 不吸附" : "XYZ 自由调整；高度由你指定");
  }
  // Routine guidance lives in tooltips; pending changes and conflicts stay visible.
  selection_note_->setVisible(selection_pending_ || conflict || draft || pending || !selection_error_.isEmpty());
  interaction_hint_->setText(selection_mode_ == "ground" ?
    "贴地拖动 · 姿态仅预览" : "XYZ 拖动 · 姿态仅预览");
  interaction_hint_->setToolTip(selection_mode_ == "ground" ?
    "贴地模式：中心沿地图 XY 平面拖动；后端仅在当前地图层计算 Z。\n"
    "不会吸附 XY、越过障碍或自动切换楼层。方向仅预览。" :
    "中心左拖：视平面平移。Shift + 左拖：沿视线前后移动，不是地图 Z。\n"
    "Ctrl + 左拖：绕相机水平/竖直轴旋转；Ctrl + Shift + 左拖：绕视线旋转。\n"
    "精确调整地图轴：拖 XYZ 箭头或对应圆环。");
}

void PlanningPanel::applyEndpoint(int index)
{
  auto & endpoint = endpoints_[index];
  if (!endpoint.publisher || !node_ || !valid_status_ || selection_pending_ || endpoint.context_conflict) {return;}
  geometry_msgs::msg::PointStamped point;
  point.header.frame_id = frame_.toStdString();
  point.header.stamp = node_->now();
  for (int axis = 0; axis < 3; ++axis) {
    endpoint.xyz[axis]->interpretText();
    endpoint.sent[axis] = endpoint.xyz[axis]->value();
  }
  point.point.x = endpoint.sent[0];
  point.point.y = endpoint.sent[1];
  point.point.z = endpoint.sent[2];
  endpoint.publisher->publish(point);
  endpoint.placing = !endpoint.selected;
  endpoint.sent_at = WallClock::now();
  endpoint.sent_mode = selection_mode_;
  endpoint.pending = true;
  endpoint.dirty = false;
  endpoint.context_conflict = false;
  refreshButtons();
}

void PlanningPanel::placeEndpoint(int index)
{
  auto & endpoint = endpoints_[index];
  if (!valid_status_) {return;}
  endpoint.dirty = false;
  endpoint.context_conflict = false;
  endpoint.placing = true;
  endpoint.sent_at = WallClock::now();
  sendEmpty(endpoint.activation_publisher);
  if (endpoint.selected) {endpoint.placing = false; focusEndpoint(index);}
  refreshButtons();
}

void PlanningPanel::restoreEndpoint(int index)
{
  auto & endpoint = endpoints_[index];
  for (int axis = 0; axis < 3; ++axis) {
    const QSignalBlocker blocked(endpoint.xyz[axis]);
    endpoint.xyz[axis]->setValue(endpoint.canonical[axis]);
  }
  endpoint.dirty = false;
  endpoint.context_conflict = false;
  refreshButtons();
}

void PlanningPanel::focusEndpoint(int index)
{
  if (!getDisplayContext() || !endpoints_[index].selected) {return;}
  auto * view = getDisplayContext()->getViewManager()->getCurrent();
  if (!view) {return;}
  const auto & point = endpoints_[index].canonical;
  view->lookAt(point[0], point[1], point[2]);
  for (int i = 0; i < view->numChildren(); ++i) {
    auto * property = view->childAt(i);
    if (property->getName() == "Distance") {property->setValue(8.0); break;}
  }
  auto * tools = getDisplayContext()->getToolManager();
  tools->setCurrentTool(tools->getDefaultTool());
}

PlanningPanel::OverviewCamera PlanningPanel::overviewCamera(
  const std::vector<std::array<double, 3>> & points, double vertical_fov, double aspect)
{
  OverviewCamera camera;
  std::array<double, 3> low{}, high{};
  bool initialized = false;
  for (const auto & point : points) {
    if (!std::all_of(point.begin(), point.end(), [](double v) {return std::isfinite(v);})) {continue;}
    if (!initialized) {low = point; high = point; initialized = true;}
    for (int axis = 0; axis < 3; ++axis) {
      low[axis] = std::min(low[axis], point[axis]);
      high[axis] = std::max(high[axis], point[axis]);
    }
  }
  if (!initialized) {return camera;}
  double radius_squared = 0;
  for (int axis = 0; axis < 3; ++axis) {
    camera.center[axis] = (low[axis] + high[axis]) * 0.5;
    radius_squared += std::pow((high[axis] - low[axis]) * 0.5, 2);
  }
  if (!std::isfinite(vertical_fov) || vertical_fov < 0.1 || vertical_fov > 3.0) {vertical_fov = 0.785398;}
  if (!std::isfinite(aspect) || aspect < 0.05 || aspect > 20.0) {aspect = 1.0;}
  const double horizontal_half = std::atan(std::tan(vertical_fov * 0.5) * aspect);
  const double limiting_half = std::min(vertical_fov * 0.5, horizontal_half);
  camera.distance = std::max(8.0, (std::sqrt(radius_squared) + 1.5) / std::sin(limiting_half));
  return camera;
}

void PlanningPanel::showOverview()
{
  if (!getDisplayContext()) {return;}
  auto * view = getDisplayContext()->getViewManager()->getCurrent();
  if (!view || !view->getCamera()) {return;}
  std::vector<std::array<double, 3>> points;
  for (const auto & endpoint : endpoints_) {
    if (endpoint.selected) {points.push_back(endpoint.canonical);}
  }
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    if (state_ == "planned" && inbox_->path_frame == frame_) {
      points.insert(points.end(), inbox_->path_bounds.begin(), inbox_->path_bounds.end());
    }
  }
  if (points.empty()) {return;}
  const auto camera = overviewCamera(
    points, view->getCamera()->getFOVy().valueRadians(), view->getCamera()->getAspectRatio());
  view->lookAt(camera.center[0], camera.center[1], camera.center[2]);
  for (int i = 0; i < view->numChildren(); ++i) {
    auto * property = view->childAt(i);
    if (property->getName() == "Distance") {property->setValue(camera.distance); break;}
  }
  auto * tools = getDisplayContext()->getToolManager();
  tools->setCurrentTool(tools->getDefaultTool());
}

void PlanningPanel::sendEmpty(const rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr & publisher)
{
  if (publisher && valid_status_) {publisher->publish(std_msgs::msg::Empty{});}
}
}  // namespace d1max_pct_rviz_tools

PLUGINLIB_EXPORT_CLASS(d1max_pct_rviz_tools::PlanningPanel, rviz_common::Panel)
