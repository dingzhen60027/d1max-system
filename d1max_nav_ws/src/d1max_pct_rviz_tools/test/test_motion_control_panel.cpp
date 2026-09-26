#include <functional>
#include <vector>
#include <QApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QRegularExpression>
#include <QTimer>
#include <gtest/gtest.h>
#include "d1max_pct_rviz_tools/motion_control_panel.hpp"
#include "d1max_pct_rviz_tools/navigation_diagnostics_panel.hpp"
#include "pluginlib/class_loader.hpp"

using namespace d1max_pct_rviz_tools;

namespace
{
class MotionProbe : public MotionControlPanel
{
public:
  double wall{100.}, monotonic{10.};
  bool confirm{true}, publish_ok{true};
  int confirmations{0};
  std::function<void()> while_confirming;
  std::vector<QJsonObject> commands;
protected:
  double wallTime() const override {return wall;}
  double monotonicTime() const override {return monotonic;}
  bool confirmExecution(double, double) override
  {
    ++confirmations;
    if (while_confirming) {while_confirming();}
    return confirm;
  }
  bool publishCommand(const QJsonObject & command) override
  {
    if (publish_ok) {commands.push_back(command);}
    return publish_ok;
  }
};

void configure(MotionControlPanel & panel, const QString & session = "motion-session")
{
  rviz_common::Config config;
  config.mapSetValue("Session ID", session); panel.load(config);
}

QJsonObject motionStatus(double stamp = 100.)
{
  return {{"schema", 1}, {"session_id", "motion-session"}, {"stamp", stamp},
    {"phase", "idle"}, {"reason", ""}, {"stop_reason", ""}, {"generation", 7},
    {"armed", false}, {"can_execute", true}, {"velocity", QJsonArray{.2, 0., .1}},
    {"max_speed", .3}, {"max_yaw", .5}, {"single_floor_only", true},
    {"gate_reason", ""}, {"acceptance_blockers", QJsonArray{}}};
}

void accept(MotionControlPanel & panel, const QJsonObject & status)
{panel.acceptStatus(QString::fromUtf8(QJsonDocument(status).toJson(QJsonDocument::Compact)));}

QPushButton * start(MotionControlPanel & panel)
{return panel.findChild<QPushButton *>("motion_execute");}
QPushButton * stop(MotionControlPanel & panel)
{return panel.findChild<QPushButton *>("motion_stop");}
}

TEST(MotionControl, LoadsPluginWithoutRosOrRendering)
{
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
  auto panel = loader.createSharedInstance("d1max_pct_rviz_tools/MotionControlPanel");
  ASSERT_NE(panel, nullptr);
  EXPECT_EQ(panel->findChildren<QPushButton *>().size(), 2);
  EXPECT_FALSE(panel->findChild<QPushButton *>("motion_execute")->isEnabled());
  EXPECT_FALSE(panel->findChild<QPushButton *>("motion_stop")->isEnabled());
}

TEST(MotionControl, StopWorksForBoundSessionBeforeAnyStatus)
{
  MotionProbe panel;
  configure(panel);
  EXPECT_FALSE(start(panel)->isEnabled());
  ASSERT_TRUE(stop(panel)->isEnabled());
  stop(panel)->click();
  ASSERT_EQ(panel.commands.size(), 1u);
  EXPECT_EQ(panel.commands[0]["action"].toString(), "stop");
  EXPECT_EQ(panel.commands[0]["session_id"].toString(), "motion-session");
  EXPECT_EQ(panel.commands[0]["generation"].toInt(), 0);
  EXPECT_EQ(panel.confirmations, 0);
}

TEST(MotionControl, SessionConfigRoundTripsWithoutInventingSession)
{
  MotionProbe panel;
  configure(panel, "current-session");
  rviz_common::Config saved;
  panel.save(saved);
  QString session;
  ASSERT_TRUE(saved.mapGetString("Session ID", &session));
  EXPECT_EQ(session, "current-session");
  panel.load(rviz_common::Config());
  EXPECT_FALSE(start(panel)->isEnabled()); EXPECT_FALSE(stop(panel)->isEnabled());
}

TEST(MotionControl, ExecuteRequiresConfirmationAndPublishesOnlyBoundIntent)
{
  MotionProbe panel;
  configure(panel); accept(panel, motionStatus());
  ASSERT_TRUE(start(panel)->isEnabled());
  start(panel)->click();
  EXPECT_EQ(panel.confirmations, 1);
  ASSERT_EQ(panel.commands.size(), 1u);
  const auto command = panel.commands.front();
  EXPECT_EQ(command.size(), 5);
  EXPECT_EQ(command["session_id"].toString(), "motion-session");
  EXPECT_EQ(command["action"].toString(), "execute");
  EXPECT_EQ(command["generation"].toInt(), 7);
  EXPECT_DOUBLE_EQ(command["stamp"].toDouble(), panel.wall);
  EXPECT_TRUE(QRegularExpression("^[a-f0-9]{32}$").match(command["id"].toString()).hasMatch());
  EXPECT_FALSE(start(panel)->isEnabled());
  EXPECT_TRUE(stop(panel)->isEnabled());
  stop(panel)->click();
  ASSERT_EQ(panel.commands.size(), 2u);
  EXPECT_NE(panel.commands[0]["id"], panel.commands[1]["id"]);
}

TEST(MotionControl, CancellingConfirmationSendsNothing)
{
  MotionProbe panel; panel.confirm = false;
  configure(panel); accept(panel, motionStatus()); start(panel)->click();
  EXPECT_EQ(panel.confirmations, 1); EXPECT_TRUE(panel.commands.empty());
  EXPECT_TRUE(start(panel)->isEnabled());
}

TEST(MotionControl, ConfirmationRechecksSourceAndReceiptFreshness)
{
  for (bool wall_expiry : {false, true}) {
    MotionProbe panel; configure(panel); accept(panel, motionStatus());
    panel.while_confirming = [&] {
      if (wall_expiry) {panel.wall += .601;} else {panel.monotonic += .601;}
    };
    start(panel)->click();
    EXPECT_TRUE(panel.commands.empty());
    EXPECT_FALSE(start(panel)->isEnabled()); EXPECT_TRUE(stop(panel)->isEnabled());
  }
}

TEST(MotionControl, ConfirmationRechecksGenerationAndSpeedLimits)
{
  for (const QString & key : {QString("generation"), QString("max_speed"), QString("max_yaw")}) {
    MotionProbe panel; configure(panel); accept(panel, motionStatus());
    panel.while_confirming = [&] {
      panel.wall += .1; panel.monotonic += .1;
      auto changed = motionStatus(panel.wall);
      changed[key] = key == "generation" ? 8. : .25;
      accept(panel, changed);
    };
    start(panel)->click();
    EXPECT_TRUE(panel.commands.empty());
    EXPECT_TRUE(panel.findChild<QLabel *>("motion_notice")->text().contains("条件已变化"));
  }
}

TEST(MotionControl, ConfirmationRechecksBackendAdmission)
{
  MotionProbe panel; configure(panel); accept(panel, motionStatus());
  panel.while_confirming = [&] {
    panel.wall += .1; panel.monotonic += .1;
    auto changed = motionStatus(panel.wall); changed["can_execute"] = false; accept(panel, changed);
  };
  start(panel)->click();
  EXPECT_TRUE(panel.commands.empty()); EXPECT_FALSE(start(panel)->isEnabled());
}

TEST(MotionControl, InvalidStatusCannotKeepPreviousStartAdmission)
{
  const std::vector<std::pair<QString, QJsonValue>> invalid{
    {"schema", 2}, {"schema", true}, {"session_id", "other-session"},
    {"stamp", 100.01}, {"generation", -.1}, {"generation", 1.2}, {"armed", 0},
    {"can_execute", 1}, {"max_speed", 1.50001}, {"max_yaw", 0},
    {"velocity", QJsonArray{0., 0.}}, {"velocity", QJsonArray{0., QJsonValue(), 0.}},
    {"single_floor_only", "true"}, {"acceptance_blockers", QJsonArray{4}},
    {"acceptance_blockers", QJsonObject{}}};
  for (const auto & item : invalid) {
    MotionProbe panel; configure(panel); accept(panel, motionStatus());
    auto value = motionStatus(); value[item.first] = item.second;
    accept(panel, value);
    EXPECT_FALSE(start(panel)->isEnabled()) << item.first.toStdString();
    EXPECT_TRUE(stop(panel)->isEnabled());
  }
  MotionProbe panel; configure(panel); accept(panel, motionStatus());
  panel.acceptStatus(QString(17000, 'x'));
  EXPECT_FALSE(start(panel)->isEnabled());
}

TEST(MotionControl, DuplicateAndOutOfOrderStatusDoNotRenewReceipt)
{
  for (double repeated_stamp : {99.99, 100.}) {
    MotionProbe panel; configure(panel); accept(panel, motionStatus());
    panel.monotonic += .5;
    accept(panel, motionStatus(repeated_stamp));
    panel.monotonic += .101;
    panel.refresh();
    EXPECT_FALSE(start(panel)->isEnabled());
    EXPECT_TRUE(stop(panel)->isEnabled());
    stop(panel)->click();
    ASSERT_EQ(panel.commands.size(), 1u);
    EXPECT_EQ(panel.commands[0]["generation"].toInt(), 7);
  }
}

TEST(MotionControl, BlockersAndUnsafeFloorPreventContradictoryCanExecute)
{
  for (const QString & key : {QString("armed"), QString("single_floor_only"), QString("acceptance_blockers")}) {
    MotionProbe panel; configure(panel); auto status = motionStatus();
    if (key == "acceptance_blockers") {status[key] = QJsonArray{"外参验收未通过"};}
    else {status[key] = key == "armed";}
    accept(panel, status);
    EXPECT_FALSE(start(panel)->isEnabled()); EXPECT_TRUE(stop(panel)->isEnabled());
    if (key == "acceptance_blockers") {EXPECT_TRUE(start(panel)->toolTip().contains("外参"));}
  }
}

TEST(MotionControl, ShowsVelocityAndAllBlockersInPlainTextTooltips)
{
  MotionProbe panel; configure(panel); auto status = motionStatus();
  status["reason"] = "waiting_for_admission";
  status["stop_reason"] = "localization_lost";
  status["gate_reason"] = "not_armed";
  status["acceptance_blockers"] = QJsonArray{"<b>外参待验收</b>", "时间对齐待验收"};
  accept(panel, status);
  EXPECT_TRUE(panel.findChild<QLabel *>("motion_velocity")->text().contains("0.20 m/s"));
  EXPECT_TRUE(panel.findChild<QLabel *>("motion_velocity")->toolTip().contains("横向"));
  auto * detail = panel.findChild<QLabel *>("motion_detail");
  EXPECT_EQ(detail->textFormat(), Qt::PlainText);
  EXPECT_TRUE(detail->toolTip().contains("localization_lost"));
  EXPECT_TRUE(detail->toolTip().contains("时间对齐待验收"));
  EXPECT_TRUE(panel.findChild<QLabel *>("motion_limits")->text().contains("0.30 m/s"));
}

TEST(MotionControl, PublishFailureDoesNotClaimCommandSent)
{
  MotionProbe panel; panel.publish_ok = false;
  configure(panel); accept(panel, motionStatus()); start(panel)->click();
  EXPECT_TRUE(panel.commands.empty());
  EXPECT_TRUE(panel.findChild<QLabel *>("motion_notice")->text().contains("未发送"));
  EXPECT_TRUE(stop(panel)->isEnabled());
}

TEST(MotionControl, ActualConfirmationDefaultsToCancelAndMentionsPhysicalStop)
{
  class DialogProbe : public MotionControlPanel
  {public: using MotionControlPanel::confirmExecution;};
  DialogProbe panel;
  bool observed = false;
  QTimer::singleShot(0, [&] {
    for (auto * widget : QApplication::topLevelWidgets()) {
      auto * dialog = qobject_cast<QMessageBox *>(widget);
      if (!dialog) {continue;}
      observed = true;
      EXPECT_TRUE(dialog->text().contains("平整地面"));
      EXPECT_TRUE(dialog->text().contains("物理急停"));
      EXPECT_TRUE(dialog->text().contains("0.30 m/s"));
      EXPECT_NE(dialog->defaultButton(), nullptr);
      if (dialog->defaultButton()) {EXPECT_EQ(dialog->defaultButton()->text(), "取消");}
      dialog->reject();
    }
  });
  EXPECT_FALSE(panel.confirmExecution(.3, .5));
  EXPECT_TRUE(observed);
}

TEST(MotionControl, RendersCompactOffscreen)
{
  MotionProbe panel; configure(panel); accept(panel, motionStatus());
  panel.resize(330, 370); panel.show(); QApplication::processEvents();
  EXPECT_LE(panel.minimumWidth(), 330);
  EXPECT_TRUE(panel.grab().save("/tmp/d1max-motion-control-panel.png"));
}

TEST(NavigationDiagnostics, MotionCapableChangesLabelOnlyAndRoundTrips)
{
  NavigationDiagnosticsPanel panel;
  rviz_common::Config config;
  config.mapSetValue("Session ID", "motion-session"); config.mapSetValue("Motion Capable", true);
  panel.load(config);
  EXPECT_EQ(panel.findChild<QLabel *>("mode")->text(), "定位与规划监视");
  EXPECT_EQ(panel.findChildren<QPushButton *>().size(), 2);  // Two passive layout selectors.
  // Changing the presentation label never changes the passive wire contract.
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(motionStatus()).toJson()));
  EXPECT_TRUE(panel.findChild<QLabel *>("local")->text().contains("等待"));
  rviz_common::Config saved; panel.save(saved);
  bool capable = false; ASSERT_TRUE(saved.mapGetBool("Motion Capable", &capable)); EXPECT_TRUE(capable);
  panel.load(rviz_common::Config());
  EXPECT_EQ(panel.findChild<QLabel *>("mode")->text(), "仅预览 · 运动关闭");
}
