#include "d1max_pct_rviz_tools/motion_control_panel.hpp"
#include <chrono>
#include <cmath>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QStringList>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace d1max_pct_rviz_tools
{
namespace
{
constexpr double kStatusTimeout = .6;
double monotonicNow()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool finite(const QJsonValue & value)
{return value.isDouble() && std::isfinite(value.toDouble());}
bool boundedText(const QJsonValue & value, int limit)
{return value.isString() && value.toString().size() <= limit;}
QLabel * label(QWidget * parent, const QString & text, const char * name)
{
  auto * result = new QLabel(text, parent);
  result->setObjectName(name); result->setTextFormat(Qt::PlainText);
  result->setWordWrap(true);
  return result;
}
QString phaseLabel(const QString & phase)
{
  if (phase == "idle" || phase == "ready") {return "等待开始";}
  if (phase == "arming" || phase == "starting") {return "正在准备执行";}
  if (phase == "locked") {return "运动锁定";}
  if (phase == "waiting_trajectory") {return "等待新轨迹";}
  if (phase == "verifying_completion") {return "已停速 · 确认到点";}
  if (phase == "executing" || phase == "tracking") {return "正在执行";}
  if (phase == "stopping") {return "正在停止";}
  if (phase == "stopped" || phase == "cancelled") {return "导航已停止";}
  if (phase == "completed" || phase == "finished") {return "导航已完成";}
  if (phase == "blocked" || phase == "fault" || phase == "error") {return "执行受阻";}
  if (phase == "disabled") {return "运动功能未启用";}
  return phase;
}
}

MotionControlPanel::MotionControlPanel(QWidget * parent)
: Panel(parent), inbox_(std::make_shared<Inbox>())
{
  setMinimumWidth(275);
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(12, 12, 12, 12); layout->setSpacing(9);
  auto * title = label(this, "导航执行", "motion_title");
  auto font = title->font(); font.setBold(true); title->setFont(font);
  layout->addWidget(title);
  phase_ = label(this, "等待执行状态", "motion_phase");
  velocity_ = label(this, "指令速度 —", "motion_velocity");
  limits_ = label(this, "仅限低速、平地、同一楼层", "motion_limits");
  detail_ = label(this, "", "motion_detail");
  detail_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  notice_ = label(this, "", "motion_notice");
  layout->addWidget(phase_); layout->addWidget(velocity_); layout->addWidget(limits_);
  layout->addWidget(detail_);
  auto * buttons = new QHBoxLayout;
  execute_ = new QPushButton("开始执行", this); execute_->setObjectName("motion_execute");
  stop_ = new QPushButton("停止导航", this); stop_->setObjectName("motion_stop");
  for (auto * button : {execute_, stop_}) {button->setMinimumHeight(38); buttons->addWidget(button);}
  connect(execute_, &QPushButton::clicked, this, &MotionControlPanel::execute);
  connect(stop_, &QPushButton::clicked, this, &MotionControlPanel::stop);
  stop_->setToolTip("向本次会话发送停止请求；状态断流时仍可使用。需要紧急制动时使用物理急停。");
  layout->addLayout(buttons); layout->addWidget(notice_); layout->addStretch();
  updateButtons();
  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &MotionControlPanel::refresh);
  timer->start(50);
}

double MotionControlPanel::wallTime() const
{return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();}
double MotionControlPanel::monotonicTime() const {return monotonicNow();}

void MotionControlPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
  session_.clear(); config.mapGetString("Session ID", &session_);
  if (session_.size() > 128) {session_.clear();}
  source_stamp_ = 0.; generation_ = 0; pending_ = false;
  notice_->clear(); unavailable("等待本次会话的执行状态");
}

void MotionControlPanel::save(rviz_common::Config config) const
{Panel::save(config); config.mapSetValue("Session ID", session_);}

void MotionControlPanel::onInitialize()
{
  if (!getDisplayContext()) {return;}
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {return;}
  auto node = abstraction->get_raw_node();
  publisher_ = node->create_publisher<std_msgs::msg::String>(
    "/d1max/live_planning/motion_command", rclcpp::QoS(1).reliable());
  auto inbox = inbox_;
  subscription_ = node->create_subscription<std_msgs::msg::String>(
    "/d1max/live_planning/motion_status", rclcpp::QoS(1).reliable(),
    [inbox](std_msgs::msg::String::ConstSharedPtr message) {
      const QString payload = message->data.size() <= 16384 ?
        QString::fromStdString(message->data) : QStringLiteral("{}");
      std::lock_guard<std::mutex> lock(inbox->mutex);
      inbox->value = payload; inbox->received_at = monotonicNow(); ++inbox->sequence;
    });
}

bool MotionControlPanel::fresh() const
{
  const double wall_age = wallTime()-source_stamp_, receipt_age = monotonicTime()-receipt_;
  return valid_ && std::isfinite(wall_age) && std::isfinite(receipt_age) &&
    wall_age >= 0. && wall_age <= kStatusTimeout &&
    receipt_age >= 0. && receipt_age <= kStatusTimeout;
}

bool MotionControlPanel::canExecute() const
{
  return fresh() && can_execute_ && !armed_ && single_floor_ &&
    blockers_.isEmpty() && !session_.isEmpty() && !pending_;
}

void MotionControlPanel::acceptStatus(const QString & json)
{acceptStatusAt(json, monotonicTime());}

void MotionControlPanel::acceptStatusAt(const QString & json, double received_at)
{
  if (json.size() > 16384) {unavailable("执行状态无效"); return;}
  const auto document = QJsonDocument::fromJson(json.toUtf8());
  const auto data = document.object();
  const auto stamp = data["stamp"], generation = data["generation"];
  const auto velocity = data["velocity"].toArray();
  const auto blockers = data["acceptance_blockers"].toArray();
  bool valid = document.isObject() && finite(data["schema"]) && data["schema"].toDouble() == 1. &&
    !session_.isEmpty() && data["session_id"].toString() == session_ &&
    finite(stamp) && stamp.toDouble() > 0. &&
    finite(generation) && generation.toDouble() >= 0. && generation.toDouble() <= 9007199254740991. &&
    std::floor(generation.toDouble()) == generation.toDouble() &&
    boundedText(data["phase"], 64) && !data["phase"].toString().isEmpty() &&
    boundedText(data["reason"], 1000) && boundedText(data["stop_reason"], 1000) &&
    boundedText(data["gate_reason"], 1000) && data["armed"].isBool() &&
    data["can_execute"].isBool() && data["single_floor_only"].isBool() &&
    finite(data["max_speed"]) && data["max_speed"].toDouble() > 0. && data["max_speed"].toDouble() <= 1.5 &&
    finite(data["max_yaw"]) && data["max_yaw"].toDouble() > 0. && data["max_yaw"].toDouble() <= 1.5 &&
    data["velocity"].isArray() && velocity.size() == 3 &&
    data["acceptance_blockers"].isArray() && blockers.size() <= 32;
  for (const auto & value : velocity) {valid = valid && finite(value) && std::abs(value.toDouble()) < 1000.;}
  for (const auto & value : blockers) {valid = valid && boundedText(value, 500);}
  if (!valid) {unavailable("执行状态无效或会话不匹配"); return;}
  // Duplicate or out-of-order source data cannot renew the receipt deadline.
  if (stamp.toDouble() <= source_stamp_) {updateButtons(); return;}
  const double wall_age = wallTime()-stamp.toDouble(), receipt_age = monotonicTime()-received_at;
  if (!std::isfinite(receipt_age) || wall_age < 0. || wall_age > kStatusTimeout ||
    receipt_age < 0. || receipt_age > kStatusTimeout)
  {unavailable("执行状态已过期"); return;}
  source_stamp_ = stamp.toDouble(); receipt_ = received_at; valid_ = true;
  generation_ = static_cast<uint64_t>(generation.toDouble());
  armed_ = data["armed"].toBool(); can_execute_ = data["can_execute"].toBool();
  single_floor_ = data["single_floor_only"].toBool();
  max_speed_ = data["max_speed"].toDouble(); max_yaw_ = data["max_yaw"].toDouble();
  QStringList details, reasons;
  for (const auto & value : blockers) {reasons.append(value.toString());}
  blockers_ = reasons.join("\n");
  for (const char * key : {"reason", "stop_reason", "gate_reason"}) {
    if (!data[key].toString().isEmpty()) {details.append(data[key].toString());}
  }
  details.append(reasons); details.removeDuplicates();
  const QString full_detail = details.join("\n");
  phase_->setText(phaseLabel(data["phase"].toString()) + (armed_ ? " · 已使能" : " · 未使能"));
  phase_->setToolTip(full_detail);
  velocity_->setText(QString("指令  %1 m/s  ·  转向 %2 rad/s")
    .arg(std::hypot(velocity[0].toDouble(), velocity[1].toDouble()), 0, 'f', 2)
    .arg(velocity[2].toDouble(), 0, 'f', 2));
  velocity_->setToolTip(QString("前向 %1 m/s；横向 %2 m/s；转向 %3 rad/s")
    .arg(velocity[0].toDouble(), 0, 'f', 3).arg(velocity[1].toDouble(), 0, 'f', 3)
    .arg(velocity[2].toDouble(), 0, 'f', 3));
  limits_->setText(QString("上限 %1 m/s · %2 rad/s\n%3")
    .arg(max_speed_, 0, 'f', 2).arg(max_yaw_, 0, 'f', 2)
    .arg(single_floor_ ? "仅限低速、平地、同一楼层" : "平地限制未确认 · 禁止开始"));
  const QString short_detail = full_detail.section('\n', 0, 1);
  detail_->setText(short_detail.size() > 180 ? short_detail.left(177) + "…" : short_detail);
  detail_->setToolTip(full_detail);
  if (pending_ && source_stamp_ >= command_stamp_) {pending_ = false; notice_->clear();}
  updateButtons();
}

void MotionControlPanel::unavailable(const QString & reason)
{
  valid_ = false; can_execute_ = false;
  phase_->setText("执行状态不可用"); phase_->setToolTip(reason);
  velocity_->setText("指令速度 —"); velocity_->setToolTip("");
  detail_->setText(reason); detail_->setToolTip(reason);
  updateButtons();
}

void MotionControlPanel::updateButtons()
{
  execute_->setEnabled(canExecute() && !confirming_);
  execute_->setToolTip(canExecute() ? "确认低速、平地条件后，开始本次会话的导航执行" :
    !fresh() ? "等待新鲜的本次会话执行状态" : !blockers_.isEmpty() ? blockers_ :
    pending_ ? "请求已发送，等待状态确认" : "当前尚不满足开始条件；查看执行状态和阻塞原因");
  stop_->setEnabled(!session_.isEmpty());
}

void MotionControlPanel::refresh()
{
  QString value; double received_at = 0.;
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    if (inbox_->sequence != consumed_) {
      consumed_ = inbox_->sequence; value = inbox_->value; received_at = inbox_->received_at;
    }
  }
  if (!value.isEmpty()) {acceptStatusAt(value, received_at);}
  if (valid_ && !fresh()) {unavailable("执行状态断流；开始已锁定，仍可发送停止请求");}
  updateButtons();
}

bool MotionControlPanel::confirmExecution(double max_speed, double max_yaw)
{
  QMessageBox confirmation(QMessageBox::Warning, "确认开始导航",
    QString("确认机器人位于平整地面、同一楼层，行进区域可通行。\n\n"
            "本次速度上限 %1 m/s，转向上限 %2 rad/s。\n"
            "请保持现场监护，并准备随时操作物理急停。")
    .arg(max_speed, 0, 'f', 2).arg(max_yaw, 0, 'f', 2),
    QMessageBox::NoButton, this);
  auto * start = confirmation.addButton("已确认，低速开始", QMessageBox::AcceptRole);
  auto * cancel = confirmation.addButton("取消", QMessageBox::RejectRole);
  confirmation.setDefaultButton(cancel); confirmation.setEscapeButton(cancel);
  confirmation.exec();
  return confirmation.clickedButton() == start;
}

bool MotionControlPanel::publishCommand(const QJsonObject & command)
{
  if (!publisher_) {return false;}
  std_msgs::msg::String message;
  message.data = QJsonDocument(command).toJson(QJsonDocument::Compact).toStdString();
  publisher_->publish(message);
  return true;
}

bool MotionControlPanel::send(const QString & action)
{
  if (session_.isEmpty()) {return false;}
  const double stamp = wallTime();
  if (!std::isfinite(stamp) || stamp <= 0.) {return false;}
  QJsonObject command{{"session_id", session_}, {"stamp", stamp},
    {"id", QUuid::createUuid().toString(QUuid::WithoutBraces).remove('-')}, {"action", action},
    {"generation", static_cast<double>(generation_)}};
  if (!publishCommand(command)) {
    notice_->setText("命令未发送；连接不可用，需要紧急制动时使用物理急停。");
    return false;
  }
  command_stamp_ = stamp; pending_ = true;
  notice_->setText(action == "execute" ? "执行请求已发送 · 等待状态确认" : "停止请求已发送 · 等待停止确认");
  updateButtons(); return true;
}

void MotionControlPanel::execute()
{
  refresh();
  if (!canExecute() || confirming_) {return;}
  const QString session = session_;
  const auto generation = generation_;
  const double max_speed = max_speed_, max_yaw = max_yaw_;
  confirming_ = true; updateButtons();
  const bool accepted = confirmExecution(max_speed, max_yaw);
  confirming_ = false;
  refresh();
  if (!accepted) {return;}
  if (!canExecute() || session != session_ || generation != generation_ ||
    max_speed != max_speed_ || max_yaw != max_yaw_) {
    notice_->setText("确认期间执行条件已变化；请核对状态后重新开始。"); return;
  }
  send("execute");
}

void MotionControlPanel::stop()
{
  // Use the last known generation even after status expiry; the supervisor
  // handles stop for the bound session independently of execution admission.
  send("stop");
}
}  // namespace d1max_pct_rviz_tools

PLUGINLIB_EXPORT_CLASS(d1max_pct_rviz_tools::MotionControlPanel, rviz_common::Panel)
