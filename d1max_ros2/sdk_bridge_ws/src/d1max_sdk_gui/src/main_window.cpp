#include "d1max_sdk_gui/main_window.hpp"

#include <QCheckBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollBar>
#include <QSizePolicy>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "d1max_sdk_gui/ros_bridge_client.hpp"

namespace d1max_sdk_gui {

namespace {

QFrame* Separator() {
  auto* line = new QFrame;
  line->setFrameShape(QFrame::HLine);
  line->setObjectName("separator");
  return line;
}

QLabel* SectionTitle(const QString& title, const QString& subtitle) {
  auto* label = new QLabel(
      QString("<span class='section'>%1</span><br><span class='hint'>%2</span>")
          .arg(title, subtitle));
  label->setTextFormat(Qt::RichText);
  return label;
}

}  // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  setWindowTitle("D1 Max 行为控制中心");
  setMinimumSize(1280, 760);
  resize(1480, 900);

  setStyleSheet(R"(
    QMainWindow, QWidget { background: #f3f0e8; color: #16242a; }
    QWidget { font-family: "Noto Sans CJK SC", "Noto Sans SC"; font-size: 14px; }
    QLabel { background: transparent; }
    QLabel#title { font-size: 27px; font-weight: 800; letter-spacing: 1px; }
    QLabel#eyebrow { color: #d45a2c; font-size: 11px; font-weight: 800; letter-spacing: 2px; }
    QLabel#value { color: #0b5351; font-family: "JetBrains Mono", "DejaVu Sans Mono"; font-weight: 700; }
    QFrame#card, QGroupBox { background: #fffdf7; border: 1px solid #d9d4c6; border-radius: 12px; }
    QGroupBox { margin-top: 13px; padding: 18px 12px 12px 12px; font-weight: 800; }
    QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 7px; color: #344a50; }
    QFrame#separator { color: #ddd7c8; background: #ddd7c8; max-height: 1px; }
    QPushButton { min-height: 40px; padding: 0 14px; border: 1px solid #bfc5bd; border-radius: 8px; background: #f8f6ef; font-weight: 700; }
    QPushButton:hover { border-color: #0b7773; background: #e9f3ef; }
    QPushButton:pressed { background: #cce2db; }
    QPushButton:disabled { color: #9b9d98; background: #ece9e1; border-color: #dedbd3; }
    QPushButton#primary { color: white; background: #0b7773; border-color: #075b58; }
    QPushButton#primary:hover { background: #096965; }
    QPushButton#warning { color: #7a310f; background: #ffe0b8; border-color: #e59b4b; }
    QPushButton#estop { min-height: 62px; color: white; background: #c62f2f; border: 3px solid #8f1f1f; border-radius: 12px; font-size: 18px; font-weight: 900; }
    QPushButton#estop:hover { background: #ae2424; }
    QPushButton#recover { color: #8f1f1f; background: #fff5f0; border-color: #c85b49; }
    QPlainTextEdit { background: #13262b; color: #d8e8e2; border: 0; border-radius: 9px; padding: 9px; font-family: "JetBrains Mono", "DejaVu Sans Mono"; font-size: 12px; }
    QDoubleSpinBox { min-height: 34px; border: 1px solid #c8c4b9; border-radius: 7px; background: white; padding: 0 7px; }
    QCheckBox { spacing: 8px; font-weight: 700; }
    .section { font-size: 18px; font-weight: 800; }
    .hint { color: #738086; font-size: 12px; }
  )");

  auto* central = new QWidget;
  auto* root = new QVBoxLayout(central);
  root->setContentsMargins(24, 20, 24, 20);
  root->setSpacing(16);
  root->addWidget(BuildHeader());

  auto* content = new QHBoxLayout;
  content->setSpacing(16);
  auto* left = new QVBoxLayout;
  left->setSpacing(16);
  left->addWidget(BuildStatusPanel());
  left->addWidget(BuildTeleopPanel());
  content->addLayout(left, 3);
  content->addWidget(BuildBehaviorPanel(), 5);
  content->addWidget(BuildEventPanel(), 4);
  root->addLayout(content, 1);
  setCentralWidget(central);

  ros_ = std::make_unique<RosBridgeClient>();
  ConnectRosSignals();
  SetConnected(false, "等待桥接");
  SetNavigationReady(false);
  AppendEvent("Qt 控制中心已启动；未申请控制权，未发送运动指令");
}

MainWindow::~MainWindow() = default;

QWidget* MainWindow::BuildHeader() {
  auto* frame = new QFrame;
  frame->setObjectName("card");
  auto* layout = new QHBoxLayout(frame);
  layout->setContentsMargins(20, 14, 20, 14);

  auto* titles = new QVBoxLayout;
  auto* eyebrow = new QLabel("D1 MAX / GUARDED ROS BRIDGE");
  eyebrow->setObjectName("eyebrow");
  auto* title = new QLabel("行为状态机控制中心");
  title->setObjectName("title");
  titles->addWidget(eyebrow);
  titles->addWidget(title);
  layout->addLayout(titles);
  layout->addStretch();

  connection_badge_ = new QLabel;
  nav_badge_ = new QLabel;
  fault_badge_ = new QLabel;
  ApplyBadge(connection_badge_, "未连接", "#74412a", "#f6d8c5");
  ApplyBadge(nav_badge_, "导航未就绪", "#67551d", "#f4e5a8");
  ApplyBadge(fault_badge_, "无故障锁存", "#255e4b", "#d9eee5");
  layout->addWidget(connection_badge_);
  layout->addWidget(nav_badge_);
  layout->addWidget(fault_badge_);
  return frame;
}

QWidget* MainWindow::BuildStatusPanel() {
  auto* box = new QGroupBox("实时状态");
  auto* grid = new QGridLayout(box);
  grid->setHorizontalSpacing(16);
  grid->setVerticalSpacing(10);

  const auto add = [this, grid](int row, const QString& name,
                                QLabel** destination) {
    auto* key = new QLabel(name);
    key->setStyleSheet("color:#6b777b;font-weight:700;");
    *destination = MakeValueLabel("--");
    grid->addWidget(key, row, 0);
    grid->addWidget(*destination, row, 1, 1, 2);
  };
  add(0, "机器人状态", &motion_value_);
  add(1, "运动模式", &mode_value_);
  add(2, "控制来源", &control_value_);
  add(3, "FSM", &fsm_value_);
  add(4, "当前目标", &goal_value_);
  add(5, "迁移步骤", &transition_value_);
  add(6, "急停状态", &estop_value_);
  add(7, "双电池", &battery_value_);
  return box;
}

QWidget* MainWindow::BuildBehaviorPanel() {
  auto* frame = new QFrame;
  frame->setObjectName("card");
  auto* root = new QVBoxLayout(frame);
  root->setContentsMargins(18, 18, 18, 18);
  root->setSpacing(12);
  root->addWidget(SectionTitle("行为目标", "按钮提交目标；每一步等待 RobotState 确认后才继续"));
  root->addWidget(Separator());

  auto* ownership = new QGroupBox("控制权");
  auto* ownership_layout = new QHBoxLayout(ownership);
  auto* take = MakeCommandButton("申请 SDK 控制权", "take_control",
      "确认从当前控制源申请 SDK 控制权？");
  take->setObjectName("primary");
  ownership_layout->addWidget(take);
  ownership_layout->addWidget(MakeCommandButton(
      "释放控制权", "release_control", "确认释放 SDK 控制权？"));
  root->addWidget(ownership);

  auto* posture = new QGroupBox("姿态迁移");
  auto* posture_layout = new QGridLayout(posture);
  posture_layout->addWidget(MakeCommandButton(
      "站立", "stand", "确认请求站立？状态机将检查当前姿态。"), 0, 0);
  posture_layout->addWidget(MakeCommandButton(
      "趴下", "lie_down", "确认请求趴下？机器人必须先停止。"), 0, 1);
  posture_layout->addWidget(MakeCommandButton(
      "匍匐", "crawl", "确认进入匍匐状态？"), 0, 2);
  posture_layout->addWidget(MakeCommandButton(
      "锁定", "lock", "确认锁定机器人？导航不会自动解锁。"), 1, 0);
  posture_layout->addWidget(MakeCommandButton(
      "解锁并站立", "unlock_to_stand", "确认从锁定状态解锁并等待站立完成？"), 1, 1, 1, 2);
  root->addWidget(posture);

  auto* modes = new QGroupBox("运动模式");
  auto* mode_layout = new QHBoxLayout(modes);
  mode_layout->addWidget(MakeCommandButton(
      "通用模式", "general_mode", "确认切换通用模式？"));
  mode_layout->addWidget(MakeCommandButton(
      "原地模式", "in_place_mode", "确认切换原地模式？"));
  mode_layout->addWidget(MakeCommandButton(
      "登阶模式", "stair_mode", "确认切换登阶模式？请确认现场安全。"));
  root->addWidget(modes);

  auto* navigation = MakeCommandButton(
      "准备导航", "prepare_navigation",
      "状态机将依次申请控制权、站立、进入通用模式并设置低速。确认继续？");
  navigation->setObjectName("primary");
  root->addWidget(navigation);
  auto* halt = MakeCommandButton("立即发送零速度", "halt");
  halt->setObjectName("warning");
  root->addWidget(halt);

  root->addStretch();
  auto* safety = new QHBoxLayout;
  estop_button_ = new QPushButton("软件急停");
  estop_button_->setObjectName("estop");
  recover_button_ = new QPushButton("解除软件急停");
  recover_button_->setObjectName("recover");
  safety->addWidget(estop_button_, 2);
  safety->addWidget(recover_button_, 1);
  root->addLayout(safety);
  connect(estop_button_, &QPushButton::clicked, this, [this]() {
    EndJog();
    AppendEvent("正在请求软件急停", "WARN");
    ros_->CallSoftEstop(true);
  });
  connect(recover_button_, &QPushButton::clicked, this, [this]() {
    if (QMessageBox::warning(this, "解除急停",
          "仅当现场、机器人和人员均安全时解除软件急停。确认继续？",
          QMessageBox::Yes | QMessageBox::No, QMessageBox::No) ==
        QMessageBox::Yes) {
      ros_->CallSoftEstop(false);
    }
  });
  return frame;
}

QWidget* MainWindow::BuildEventPanel() {
  auto* frame = new QFrame;
  frame->setObjectName("card");
  auto* layout = new QVBoxLayout(frame);
  layout->setContentsMargins(16, 16, 16, 16);
  layout->addWidget(SectionTitle("迁移审计", "服务应答、SDK ACK、状态确认、故障和超时"));
  layout->addWidget(Separator());
  event_log_ = new QPlainTextEdit;
  event_log_->setReadOnly(true);
  event_log_->document()->setMaximumBlockCount(1000);
  layout->addWidget(event_log_, 1);
  auto* clear = new QPushButton("清空显示日志");
  connect(clear, &QPushButton::clicked, event_log_, &QPlainTextEdit::clear);
  layout->addWidget(clear);
  return frame;
}

QWidget* MainWindow::BuildTeleopPanel() {
  auto* box = new QGroupBox("手动速度测试");
  auto* root = new QVBoxLayout(box);
  teleop_enable_ = new QCheckBox("启用按住式手动速度（要求导航已就绪）");
  root->addWidget(teleop_enable_);

  auto* values = new QGridLayout;
  forward_speed_ = new QDoubleSpinBox;
  lateral_speed_ = new QDoubleSpinBox;
  yaw_speed_ = new QDoubleSpinBox;
  forward_speed_->setRange(0.02, 0.45);
  lateral_speed_->setRange(0.02, 0.35);
  yaw_speed_->setRange(0.05, 0.80);
  forward_speed_->setValue(0.12);
  lateral_speed_->setValue(0.10);
  yaw_speed_->setValue(0.20);
  forward_speed_->setSuffix(" m/s");
  lateral_speed_->setSuffix(" m/s");
  yaw_speed_->setSuffix(" rad/s");
  values->addWidget(new QLabel("前后"), 0, 0);
  values->addWidget(forward_speed_, 0, 1);
  values->addWidget(new QLabel("横移"), 1, 0);
  values->addWidget(lateral_speed_, 1, 1);
  values->addWidget(new QLabel("转向"), 2, 0);
  values->addWidget(yaw_speed_, 2, 1);
  root->addLayout(values);

  auto* pad = new QGridLayout;
  const auto jog = [this, pad](const QString& text, int row, int column,
                               double forward, double lateral, double yaw) {
    auto* button = new QPushButton(text);
    button->setMinimumSize(62, 44);
    jog_buttons_.append(button);
    pad->addWidget(button, row, column);
    connect(button, &QPushButton::pressed, this,
            [this, forward, lateral, yaw]() {
              BeginJog(forward, lateral, yaw);
            });
    connect(button, &QPushButton::released, this, &MainWindow::EndJog);
  };
  jog("前", 0, 1, 1, 0, 0);
  jog("左移", 1, 0, 0, 1, 0);
  jog("停", 1, 1, 0, 0, 0);
  jog("右移", 1, 2, 0, -1, 0);
  jog("后", 2, 1, -1, 0, 0);
  jog("左转", 3, 0, 0, 0, 1);
  jog("右转", 3, 2, 0, 0, -1);
  root->addLayout(pad);

  teleop_timer_ = new QTimer(this);
  teleop_timer_->setInterval(50);
  connect(teleop_timer_, &QTimer::timeout, this, [this]() {
    if (!navigation_ready_ || !teleop_enable_->isChecked()) {
      EndJog();
      return;
    }
    ros_->PublishVelocity(jog_forward_, jog_lateral_, jog_yaw_);
  });
  connect(teleop_enable_, &QCheckBox::toggled, this, [this](bool enabled) {
    if (!enabled) {
      EndJog();
    }
    for (auto* button : jog_buttons_) {
      button->setEnabled(navigation_ready_ && enabled);
    }
  });
  return box;
}

QPushButton* MainWindow::MakeCommandButton(
    const QString& text, const QString& service,
    const QString& confirmation) {
  auto* button = new QPushButton(text);
  command_buttons_.append(button);
  connect(button, &QPushButton::clicked, this,
          [this, service, confirmation]() {
            RequestService(service, confirmation);
          });
  return button;
}

QLabel* MainWindow::MakeValueLabel(const QString& initial) {
  auto* label = new QLabel(initial);
  label->setObjectName("value");
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  return label;
}

void MainWindow::ConnectRosSignals() {
  connect(ros_.get(), &RosBridgeClient::ConnectionChanged, this,
          [this](const QString& state) {
            SetConnected(state == "connected", state);
          });
  connect(ros_.get(), &RosBridgeClient::NavigationReadyChanged, this,
          &MainWindow::SetNavigationReady);
  connect(ros_.get(), &RosBridgeClient::TransitionEventReceived, this,
          [this](const QString& event) { AppendEvent(event, "FSM"); });
  connect(ros_.get(), &RosBridgeClient::ServiceResult, this,
          [this](const QString& service, bool success,
                 const QString& message) {
            AppendEvent(QString("%1: %2").arg(service, message),
                        success ? "ACCEPT" : "REJECT");
          });
  connect(ros_.get(), &RosBridgeClient::FaultsReceived, this,
          [this](const QString& json) {
            ApplyBadge(fault_badge_, "故障锁存", "#ffffff", "#b52b2b");
            AppendEvent(json, "FAULT");
            EndJog();
          });
  connect(ros_.get(), &RosBridgeClient::RobotStateReceived, this,
          [this](const QString& json) {
            const auto document = QJsonDocument::fromJson(json.toUtf8());
            if (!document.isObject()) {
              AppendEvent("robot_state JSON 解析失败", "ERROR");
              return;
            }
            const auto object = document.object();
            motion_value_->setText(object.value("motion_status_text").toString("UNKNOWN"));
            const int mode = object.value("sport_mode").toInt();
            mode_value_->setText(mode == 1 ? "GENERAL" : mode == 2 ? "IN_PLACE" : mode == 3 ? "STAIR" : "UNKNOWN");
            const int control = object.value("control_source").toInt();
            control_value_->setText(control == 2 ? "SDK" : control == 1 ? "APP" : control == 3 ? "OTHER" : "UNKNOWN");
            const int soft = object.value("software_emergency_status").toInt();
            const int hard = object.value("hardware_emergency_status").toInt();
            estop_value_->setText(QString("软:%1  硬:%2")
                .arg(soft == 1 ? "恢复" : soft == 2 ? "触发" : "未知",
                     hard == 1 ? "恢复" : hard == 2 ? "触发" : "未知"));
            battery_value_->setText(QString("%1% / %2%")
                .arg(object.value("battery_power_1").toDouble(), 0, 'f', 0)
                .arg(object.value("battery_power_2").toDouble(), 0, 'f', 0));
          });
  connect(ros_.get(), &RosBridgeClient::BehaviorStateReceived, this,
          [this](const QString& json) {
            const auto document = QJsonDocument::fromJson(json.toUtf8());
            if (!document.isObject()) {
              return;
            }
            const auto object = document.object();
            fsm_value_->setText(object.value("fsm_state").toString());
            goal_value_->setText(object.value("goal").toString().isEmpty()
                ? "--" : object.value("goal").toString());
            transition_value_->setText(object.value("active_transition").toString());
            if (object.value("fault_latched").toBool()) {
              ApplyBadge(fault_badge_, "故障锁存", "#ffffff", "#b52b2b");
            }
            const QString error = object.value("last_error").toString();
            if (!error.isEmpty()) {
              AppendEvent(error, "ERROR");
            }
          });
}

void MainWindow::SetConnected(bool connected, const QString& text) {
  connected_ = connected;
  ApplyBadge(connection_badge_, connected ? "SDK 已连接" : text,
             connected ? "#ffffff" : "#74412a",
             connected ? "#0b7773" : "#f6d8c5");
  for (auto* button : command_buttons_) {
    button->setEnabled(connected);
  }
  estop_button_->setEnabled(connected);
  recover_button_->setEnabled(connected);
  if (!connected) {
    SetNavigationReady(false);
  }
}

void MainWindow::SetNavigationReady(bool ready) {
  navigation_ready_ = ready && connected_;
  ApplyBadge(nav_badge_, navigation_ready_ ? "导航已就绪" : "导航未就绪",
             navigation_ready_ ? "#ffffff" : "#67551d",
             navigation_ready_ ? "#0b7773" : "#f4e5a8");
  teleop_enable_->setEnabled(navigation_ready_);
  if (!navigation_ready_) {
    teleop_enable_->setChecked(false);
    EndJog();
  }
  for (auto* button : jog_buttons_) {
    button->setEnabled(navigation_ready_ && teleop_enable_->isChecked());
  }
}

void MainWindow::AppendEvent(const QString& message, const QString& kind) {
  if (!event_log_) {
    return;
  }
  event_log_->appendPlainText(QString("[%1] %2 %3")
      .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"),
           kind.leftJustified(7, ' '), message));
  event_log_->verticalScrollBar()->setValue(
      event_log_->verticalScrollBar()->maximum());
}

void MainWindow::RequestService(const QString& service,
                                const QString& confirmation) {
  if (!connected_) {
    AppendEvent("SDK 未连接，拒绝服务请求", "REJECT");
    return;
  }
  if (!confirmation.isEmpty() &&
      QMessageBox::question(this, "确认行为目标", confirmation,
                            QMessageBox::Yes | QMessageBox::No,
                            QMessageBox::No) != QMessageBox::Yes) {
    return;
  }
  AppendEvent("提交目标: " + service, "REQUEST");
  ros_->CallTrigger(service.toStdString());
}

void MainWindow::BeginJog(double forward_sign, double lateral_sign,
                          double yaw_sign) {
  if (!navigation_ready_ || !teleop_enable_->isChecked()) {
    return;
  }
  jog_forward_ = forward_sign * forward_speed_->value();
  jog_lateral_ = lateral_sign * lateral_speed_->value();
  jog_yaw_ = yaw_sign * yaw_speed_->value();
  ros_->PublishVelocity(jog_forward_, jog_lateral_, jog_yaw_);
  if (jog_forward_ == 0.0 && jog_lateral_ == 0.0 && jog_yaw_ == 0.0) {
    EndJog();
  } else {
    teleop_timer_->start();
  }
}

void MainWindow::EndJog() {
  if (teleop_timer_) {
    teleop_timer_->stop();
  }
  jog_forward_ = 0.0;
  jog_lateral_ = 0.0;
  jog_yaw_ = 0.0;
  if (ros_) {
    ros_->PublishZeroVelocity();
  }
}

void MainWindow::ApplyBadge(QLabel* label, const QString& text,
                            const QString& color,
                            const QString& background) {
  label->setText(text);
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet(QString(
      "QLabel { color:%1; background:%2; border-radius:13px; "
      "padding:6px 12px; font-weight:800; }").arg(color, background));
}

void MainWindow::closeEvent(QCloseEvent* event) {
  EndJog();
  QMainWindow::closeEvent(event);
}

}  // namespace d1max_sdk_gui
