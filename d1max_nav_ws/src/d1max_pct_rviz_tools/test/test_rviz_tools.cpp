#include <limits>
#include <vector>
#include <sys/stat.h>
#include <QApplication>
#include <QDateTime>
#include <QAbstractItemModel>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEventLoop>
#include <QFile>
#include <QLineEdit>
#include <QLabel>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPushButton>
#include <QSignalBlocker>
#include <QScrollArea>
#include <QScrollBar>
#include <QTimer>
#include <QTemporaryDir>
#include <QToolButton>
#include <gtest/gtest.h>
#include "d1max_pct_rviz_tools/point_tools.hpp"
#include "d1max_pct_rviz_tools/planning_panel.hpp"
#include "d1max_pct_rviz_tools/navigation_diagnostics_panel.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_group.hpp"
#include "rviz_common/yaml_config_reader.hpp"
#include "pluginlib/class_loader.hpp"

using namespace d1max_pct_rviz_tools;

namespace
{
std::vector<QString> icon_messages;
void captureIconMessage(QtMsgType, const QMessageLogContext &, const QString & message)
{
  icon_messages.push_back(message);
}
}

TEST(Icons, QtLoadsBothSvgPathsWithoutWarnings)
{
  icon_messages.clear();
  const auto previous = qInstallMessageHandler(captureIconMessage);
  const auto start = rviz_common::loadPixmap("package://d1max_pct_rviz_tools/icons/start.svg", false);
  const auto goal = rviz_common::loadPixmap("package://d1max_pct_rviz_tools/icons/goal.svg", false);
  qInstallMessageHandler(previous);
  EXPECT_FALSE(start.isNull());
  EXPECT_FALSE(goal.isNull());
  EXPECT_EQ(start.width(), 32);
  EXPECT_EQ(goal.width(), 32);
  EXPECT_TRUE(icon_messages.empty());
  for (const auto & message : icon_messages) {ADD_FAILURE() << message.toStdString();}
}

TEST(Placement, DistinctToolbarShortcuts)
{
  Start3D start;
  Goal3D goal;
  EXPECT_EQ(start.getShortcutKey(), 'b');
  EXPECT_EQ(goal.getShortcutKey(), 'n');
}

const QString selected = R"({"mode":"GLOBAL_PATH_PREVIEW_ONLY","frame_id":"map","ready":true,"state":"selected","reason":"ok","start_xyz":[1,2,-0.53],"goal_xyz":[5,6,-0.4]})";
const QString cleared = R"({"mode":"GLOBAL_PATH_PREVIEW_ONLY","frame_id":"map","ready":true,"state":"cleared","reason":"ok","start_xyz":null,"goal_xyz":null})";

QString selectionStatus(const QString & mode = "ground", int active_layer = -1)
{
  auto status = QJsonDocument::fromJson(selected.toUtf8()).object();
  status["selection_mode"] = mode;
  status["active_layer"] = active_layer;
  status["available_layers"] = QJsonArray{
    QJsonObject{{"id", -1}, {"label", "自动跟随当前地面"}},
    QJsonObject{{"id", 0}, {"label", "分层 #0 · z=-0.5 m"}},
    QJsonObject{{"id", 3}, {"label", "分层 #3 · z=2.5 m"}}};
  status["start_validation"] = QJsonObject{{"valid", true}, {"reason", "ok"}, {"layer_id", 0}};
  status["goal_validation"] = QJsonObject{{"valid", true}, {"reason", "ok"}, {"layer_id", 3}};
  return QString::fromUtf8(QJsonDocument(status).toJson());
}

class SelectionPanelProbe : public PlanningPanel
{
public:
  int mode_requests{0};
  int layer_requests{0};
  QString requested_mode;
  int requested_layer{-100};
protected:
  bool publishModeRequest(const QString & mode) override
  {
    ++mode_requests;
    requested_mode = mode;
    return true;
  }
  bool publishLayerRequest(int layer) override
  {
    ++layer_requests;
    requested_layer = layer;
    return true;
  }
};

class ActivationProbe : public PreviewPointTool
{
public:
  ActivationProbe() : PreviewPointTool("3D Start", "/d1max/pct_preview/activate_start", 'b', "start") {}
  int requests{0};
  int focused{0};
  std::array<double, 3> point{};
protected:
  bool publishActivationRequest() override {++requests; return true;}
  void focusAndReturn(const std::array<double, 3> & xyz) override {++focused; point = xyz;}
};

class LiveActivationProbe : public PreviewPointTool
{
public:
  LiveActivationProbe() : PreviewPointTool("3D 目标", "/d1max/live_planning/activate_goal3d", 'n', "goal",
      "/d1max/live_planning/goal_editor_status", "LIVE_GOAL_EDITOR_NO_MOTION")
  {getPropertyContainer()->subProp("Session ID")->setValue("live-test");}
  int focused{0};
protected:
  bool publishActivationRequest() override {return true;}
  void focusAndReturn(const std::array<double, 3> &) override {++focused;}
};

void spinQt(int milliseconds = 60);

TEST(Placement, LiveGoalRequiresOwnModeSessionFrameAndFreshEditorState)
{
  LiveActivationProbe tool;
  tool.activate();
  QApplication::processEvents();
  tool.acceptStatus(selected);  // Existing offline editor is not the live editor.
  spinQt();
  EXPECT_EQ(tool.focused, 0);
  QJsonObject value{{"mode", "LIVE_GOAL_EDITOR_NO_MOTION"}, {"session_id", "wrong"},
    {"frame_id", "d1max_loc_map"}, {"motion_enabled", false},
    {"received_at_unix", QDateTime::currentMSecsSinceEpoch() * .001},
    {"selection_mode", "free_xyz"}, {"goal_xyz", QJsonArray{1, 2, 3}}};
  tool.acceptStatus(QString::fromUtf8(QJsonDocument(value).toJson()));
  spinQt();
  EXPECT_EQ(tool.focused, 0);
  value["session_id"] = "live-test";
  tool.acceptStatus(QString::fromUtf8(QJsonDocument(value).toJson()));
  spinQt();
  EXPECT_EQ(tool.focused, 1);
}

void spinQt(int milliseconds)
{
  QEventLoop loop;
  QTimer::singleShot(milliseconds, &loop, &QEventLoop::quit);
  loop.exec();
}

TEST(Placement, ToolbarActivateCreatesWithoutAnyMouseOrCloudHit)
{
  ActivationProbe tool;
  tool.acceptStatus(cleared);
  tool.activate();
  EXPECT_EQ(tool.requests, 0);  // Must not reenter ToolManager synchronously.
  QApplication::processEvents();
  EXPECT_EQ(tool.requests, 1);
  EXPECT_EQ(tool.focused, 0);  // Wait for backend confirmation.
  tool.acceptStatus(selected);
  spinQt();
  EXPECT_EQ(tool.requests, 1);
  EXPECT_EQ(tool.focused, 1);
  EXPECT_DOUBLE_EQ(tool.point[0], 1);
  EXPECT_DOUBLE_EQ(tool.point[2], -0.53);
}

TEST(Placement, ExistingPointIsFocusedOnlyAfterFreshConfirmation)
{
  ActivationProbe tool;
  tool.acceptStatus(selected);
  tool.activate();
  QApplication::processEvents();
  EXPECT_EQ(tool.requests, 1);
  EXPECT_EQ(tool.focused, 0);
  tool.acceptStatus(selected);
  spinQt();
  EXPECT_EQ(tool.focused, 1);
  EXPECT_DOUBLE_EQ(tool.point[2], -0.53);
}

TEST(Placement, CancelledActivationCannotFocusLater)
{
  ActivationProbe tool;
  tool.activate();
  QApplication::processEvents();
  tool.deactivate();
  tool.acceptStatus(selected);
  spinQt();
  EXPECT_EQ(tool.requests, 1);
  EXPECT_EQ(tool.focused, 0);
}

TEST(Placement, MalformedPointIsNotFocused)
{
  ActivationProbe tool;
  tool.activate();
  QApplication::processEvents();
  tool.acceptStatus(R"({"mode":"GLOBAL_PATH_PREVIEW_ONLY","start_xyz":[1,2,20000]})");
  spinQt();
  EXPECT_EQ(tool.focused, 0);
}

TEST(Panel, StartsDisabledAndAcceptsCanonicalXYZ)
{
  PlanningPanel panel;
  auto * x = panel.findChild<QDoubleSpinBox *>("start_x");
  auto * z = panel.findChild<QDoubleSpinBox *>("start_z");
  auto * plan = panel.findChild<QPushButton *>("plan");
  ASSERT_NE(x, nullptr);
  EXPECT_FALSE(x->isEnabled());
  EXPECT_FALSE(plan->isEnabled());
  panel.acceptStatus(selected);
  EXPECT_TRUE(x->isEnabled());
  EXPECT_TRUE(plan->isEnabled());
  EXPECT_DOUBLE_EQ(x->value(), 1);
  EXPECT_DOUBLE_EQ(z->value(), -0.53);
  EXPECT_EQ(x->decimals(), 3);
  EXPECT_DOUBLE_EQ(x->singleStep(), 0.01);
}

TEST(Panel, DirtyDraftSurvivesHeartbeatAndDisablesPlan)
{
  PlanningPanel panel;
  panel.acceptStatus(selected);
  auto * x = panel.findChild<QDoubleSpinBox *>("start_x");
  x->setValue(2.345);
  panel.acceptStatus(selected);
  EXPECT_DOUBLE_EQ(x->value(), 2.345);
  EXPECT_FALSE(panel.findChild<QPushButton *>("plan")->isEnabled());
  EXPECT_TRUE(panel.findChild<QPushButton *>("start_apply")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("start_snap")->isEnabled());
  panel.findChild<QPushButton *>("start_restore")->click();
  EXPECT_DOUBLE_EQ(x->value(), 1);
  EXPECT_TRUE(panel.findChild<QPushButton *>("plan")->isEnabled());
}

TEST(Panel, ClearInvalidatesEndpointAndDraft)
{
  PlanningPanel panel;
  panel.acceptStatus(selected);
  panel.findChild<QDoubleSpinBox *>("start_x")->setValue(99);
  panel.acceptStatus(cleared);
  EXPECT_TRUE(panel.findChild<QDoubleSpinBox *>("start_x")->isEnabled());
  EXPECT_DOUBLE_EQ(panel.findChild<QDoubleSpinBox *>("start_x")->value(), 0);
  EXPECT_TRUE(panel.findChild<QPushButton *>("start_apply")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("plan")->isEnabled());
}

TEST(Panel, EmptySelectionCanBePlacedOrEditedWithoutCloudPick)
{
  PlanningPanel panel;
  panel.acceptStatus(cleared);
  EXPECT_TRUE(panel.findChild<QPushButton *>("start_place")->isEnabled());
  EXPECT_TRUE(panel.findChild<QPushButton *>("goal_place")->isEnabled());
  auto * x = panel.findChild<QDoubleSpinBox *>("start_x");
  EXPECT_TRUE(x->isEnabled());
  x->setValue(7.123);
  panel.acceptStatus(cleared);
  EXPECT_DOUBLE_EQ(x->value(), 7.123);
  EXPECT_TRUE(panel.findChild<QPushButton *>("start_apply")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("plan")->isEnabled());
}

TEST(Panel, InvalidStatusCannotEnableActions)
{
  PlanningPanel panel;
  panel.acceptStatus(selected);
  panel.acceptStatus("{}");
  EXPECT_FALSE(panel.findChild<QPushButton *>("plan")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("start_snap")->isEnabled());
}

TEST(Panel, UncommittedTypingAlsoDisablesPlan)
{
  PlanningPanel panel;
  panel.acceptStatus(selected);
  auto * x = panel.findChild<QDoubleSpinBox *>("start_x");
  auto * editor = x->findChild<QLineEdit *>();
  editor->setText("123.456 m");
  QMetaObject::invokeMethod(editor, "textEdited", Qt::DirectConnection, Q_ARG(QString, "123.456 m"));
  panel.acceptStatus(selected);
  EXPECT_EQ(editor->text(), "123.456 m");
  EXPECT_FALSE(panel.findChild<QPushButton *>("plan")->isEnabled());
  EXPECT_TRUE(panel.findChild<QPushButton *>("start_apply")->isEnabled());
}

TEST(Plugins, RegisteredClassesLoadWithoutROSOrRendering)
{
  pluginlib::ClassLoader<rviz_common::Tool> tools("rviz_common", "rviz_common::Tool");
  auto start = tools.createSharedInstance("d1max_pct_rviz_tools/Start3D");
  auto goal = tools.createSharedInstance("d1max_pct_rviz_tools/Goal3D");
  auto live_goal = tools.createSharedInstance("d1max_pct_rviz_tools/LiveGoal3D");
  EXPECT_EQ(start->getShortcutKey(), 'b');
  EXPECT_EQ(goal->getShortcutKey(), 'n');
  EXPECT_EQ(live_goal->getShortcutKey(), 'n');
  rviz_common::Config live_config;
  live_config.mapSetValue("Session ID", "owned-live-session");
  live_goal->load(live_config);
  EXPECT_EQ(live_goal->getPropertyContainer()->subProp("Session ID")->getValue().toString(),
    "owned-live-session");
  pluginlib::ClassLoader<rviz_common::Panel> panels("rviz_common", "rviz_common::Panel");
  auto panel = panels.createSharedInstance("d1max_pct_rviz_tools/PlanningPanel");
  EXPECT_NE(panel, nullptr);
}

TEST(Plugins, HumbleInteractiveMarkersLoadsNamespaceNotUpdateTopic)
{
  pluginlib::ClassLoader<rviz_common::Display> loader("rviz_common", "rviz_common::Display");
  auto display = loader.createSharedInstance("rviz_default_plugins/InteractiveMarkers");
  rviz_common::properties::Property * marker_namespace = nullptr;
  for (int i = 0; i < display->numChildren(); ++i) {
    auto * child = display->childAt(i);
    if (child->getName() == "Interactive Markers Namespace") {marker_namespace = child;}
  }
  ASSERT_NE(marker_namespace, nullptr);
  const QSignalBlocker blocked_property(marker_namespace);
  const QSignalBlocker blocked_display(display.get());
  rviz_common::Config config;
  rviz_common::YamlConfigReader reader;
  reader.readString(config, "Interactive Markers Namespace: /pct_preview_points\n");
  ASSERT_FALSE(reader.error());
  display->load(config);
  EXPECT_EQ(marker_namespace->getValue().toString(), "/pct_preview_points");
}

TEST(Panel, RenderReadableOffscreenPreview)
{
  PlanningPanel panel;
  auto state = QJsonDocument::fromJson(selectionStatus().toUtf8()).object();
  state["start_orientation_xyzw"] = QJsonArray{0, 0, 0, 1};
  state["goal_orientation_xyzw"] = QJsonArray{0, 0, 0, 1};
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(state).toJson()));
  panel.resize(370, 960);
  panel.show();
  QApplication::processEvents();
  EXPECT_TRUE(panel.grab().save("/tmp/d1max_pct_preview_panel.png"));
  panel.findChild<QToolButton *>("start_edit_toggle")->click();
  panel.resize(370, 980);
  QApplication::processEvents();
  EXPECT_TRUE(panel.grab().save("/tmp/d1max_pct_preview_panel_expanded.png"));
}

TEST(Panel, PreciseXYZCanBeExpandedWithoutLosingValues)
{
  PlanningPanel panel;
  panel.acceptStatus(selected);
  auto * body = panel.findChild<QWidget *>("start_edit_body");
  auto * toggle = panel.findChild<QToolButton *>("start_edit_toggle");
  ASSERT_NE(body, nullptr);
  ASSERT_NE(toggle, nullptr);
  EXPECT_TRUE(body->isHidden());
  EXPECT_TRUE(panel.findChild<QWidget *>("numeric_steps")->isHidden());
  toggle->click();
  EXPECT_FALSE(body->isHidden());
  EXPECT_FALSE(panel.findChild<QWidget *>("numeric_steps")->isHidden());
  EXPECT_DOUBLE_EQ(panel.findChild<QDoubleSpinBox *>("start_z")->value(), -0.53);
}

TEST(Panel, CompactDefaultKeepsPrimaryActionsVisibleWithoutAlgorithmBranding)
{
  PlanningPanel panel;
  auto state = QJsonDocument::fromJson(selectionStatus("free").toUtf8()).object();
  state["frame_id"] = "d1max_multifloor_planning";
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(state).toJson()));
  panel.resize(370, 810);
  panel.show();
  QApplication::processEvents();
  EXPECT_EQ(panel.findChild<QLabel *>("panel_title")->text(), "全局路径");
  EXPECT_EQ(panel.findChild<QLabel *>("preview_context")->text(), "离线预览 · 不下发运动");
  EXPECT_TRUE(panel.findChild<QLabel *>("preview_context")->toolTip().contains("d1max_multifloor_planning"));
  EXPECT_TRUE(panel.findChild<QLabel *>("selection_note")->isHidden());
  EXPECT_FALSE(panel.findChild<QLabel *>("start_orientation")->isVisible());
  EXPECT_FALSE(panel.findChild<QPushButton *>("start_reset_orientation")->isVisible());
  EXPECT_TRUE(panel.findChild<QPushButton *>("start_snap")->isVisible());
  EXPECT_TRUE(panel.findChild<QPushButton *>("plan")->isVisible());
  EXPECT_EQ(panel.findChild<QScrollArea *>("panel_scroll")->verticalScrollBar()->maximum(), 0);
  for (auto * label : panel.findChildren<QLabel *>()) {
    EXPECT_FALSE(label->text().contains("PCT"));
  }
  panel.findChild<QToolButton *>("start_edit_toggle")->click();
  QApplication::processEvents();
  EXPECT_TRUE(panel.findChild<QLabel *>("start_orientation")->isVisible());
  EXPECT_TRUE(panel.findChild<QPushButton *>("start_reset_orientation")->isVisible());
}

TEST(Panel, BothExpandedEditorsScrollInsteadOfForcingOversizedDock)
{
  PlanningPanel panel;
  panel.acceptStatus(selected);
  panel.resize(350, 810);
  panel.show();
  panel.findChild<QToolButton *>("start_edit_toggle")->click();
  panel.findChild<QToolButton *>("goal_edit_toggle")->click();
  QApplication::processEvents();
  EXPECT_EQ(panel.height(), 810);
  EXPECT_GT(panel.findChild<QScrollArea *>("panel_scroll")->verticalScrollBar()->maximum(), 0);
}

TEST(Panel, PoseIsExplicitlyPreviewOnlyAndStatusIsCompact)
{
  PlanningPanel panel;
  auto state = QJsonDocument::fromJson(selected.toUtf8()).object();
  state["start_orientation_xyzw"] = QJsonArray{0, 0, std::sqrt(0.5), std::sqrt(0.5)};
  state["selection_mode"] = "free";
  state["goal_orientation_xyzw"] = QJsonArray{0, 0, 0, 1};
  state["state"] = "planned";
  state["reason"] = "A long detailed native planner diagnostic retained only in tooltip";
  QJsonObject result;
  result["length_m"] = 18.453;
  state["result"] = result;
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(state).toJson()));
  EXPECT_TRUE(panel.findChild<QLabel *>("start_orientation")->text().contains("Y 90.0°"));
  EXPECT_TRUE(panel.findChild<QLabel *>("interaction_hint")->text().contains("姿态仅预览"));
  EXPECT_TRUE(panel.findChild<QLabel *>("interaction_hint")->toolTip().contains("不是地图 Z"));
  EXPECT_EQ(panel.findChild<QLabel *>("status")->text(), "路径就绪 · 18.45 m");
  EXPECT_TRUE(panel.findChild<QLabel *>("status")->toolTip().contains("native planner"));
  EXPECT_TRUE(panel.findChild<QPushButton *>("start_reset_orientation")->isEnabled());
  panel.acceptStatus(cleared);
  EXPECT_FALSE(panel.findChild<QPushButton *>("start_reset_orientation")->isEnabled());
}

TEST(SelectionMode, GroundIsDefaultAndFreeRestoresZEditing)
{
  PlanningPanel panel;
  panel.acceptStatus(selectionStatus());
  EXPECT_EQ(panel.findChild<QComboBox *>("selection_mode")->currentData().toString(), "ground");
  EXPECT_TRUE(panel.findChild<QDoubleSpinBox *>("start_z")->isReadOnly());
  EXPECT_TRUE(panel.findChild<QPushButton *>("start_snap")->isHidden());
  EXPECT_FALSE(panel.findChild<QLabel *>("start_height_note")->isHidden());
  panel.acceptStatus(selectionStatus("free"));
  EXPECT_FALSE(panel.findChild<QDoubleSpinBox *>("start_z")->isReadOnly());
  EXPECT_FALSE(panel.findChild<QPushButton *>("start_snap")->isHidden());
  EXPECT_TRUE(panel.findChild<QLabel *>("start_height_note")->isHidden());
}

TEST(SelectionMode, ModeWaitsForAcknowledgementWithoutMovingEndpoints)
{
  SelectionPanelProbe panel;
  panel.acceptStatus(selectionStatus());
  auto * selector = panel.findChild<QComboBox *>("selection_mode");
  selector->setCurrentIndex(selector->findData("free"));
  EXPECT_EQ(panel.mode_requests, 1);
  EXPECT_EQ(panel.requested_mode, "free");
  EXPECT_FALSE(panel.findChild<QPushButton *>("plan")->isEnabled());
  EXPECT_FALSE(selector->isEnabled());
  panel.acceptStatus(selectionStatus());  // Old heartbeat is not an acknowledgement.
  EXPECT_FALSE(selector->isEnabled());
  EXPECT_TRUE(panel.findChild<QDoubleSpinBox *>("start_z")->isReadOnly());
  panel.acceptStatus(selectionStatus("free"));
  EXPECT_TRUE(selector->isEnabled());
  EXPECT_FALSE(panel.findChild<QDoubleSpinBox *>("start_z")->isReadOnly());
  EXPECT_DOUBLE_EQ(panel.findChild<QDoubleSpinBox *>("start_z")->value(), -0.53);
  EXPECT_TRUE(panel.findChild<QPushButton *>("plan")->isEnabled());
}

TEST(SelectionMode, LayerUsesExplicitIdAndWaitsForAcknowledgement)
{
  SelectionPanelProbe panel;
  panel.acceptStatus(selectionStatus());
  auto * selector = panel.findChild<QComboBox *>("active_layer");
  selector->setCurrentIndex(selector->findData(3));
  EXPECT_EQ(panel.layer_requests, 1);
  EXPECT_EQ(panel.requested_layer, 3);
  EXPECT_FALSE(selector->isEnabled());
  panel.acceptStatus(selectionStatus("ground", 3));
  EXPECT_TRUE(selector->isEnabled());
  EXPECT_EQ(selector->currentData().toInt(), 3);
  EXPECT_DOUBLE_EQ(panel.findChild<QDoubleSpinBox *>("start_x")->value(), 1);
  EXPECT_DOUBLE_EQ(panel.findChild<QDoubleSpinBox *>("start_z")->value(), -0.53);
  EXPECT_TRUE(panel.findChild<QLabel *>("start_validation")->text().contains("分层 #0"));
}

TEST(SelectionMode, PhysicalFloorsAreNotNativeSliceIdsAndHeightWitnessIsShown)
{
  SelectionPanelProbe panel;
  auto state = QJsonDocument::fromJson(selectionStatus().toUtf8()).object();
  auto layers = state["available_layers"].toArray();
  layers.prepend(QJsonObject{{"id", -3}, {"label", "二楼 · 多切片地面"}});
  layers.prepend(QJsonObject{{"id", -2}, {"label", "一楼 · 多切片地面"}});
  state["available_layers"] = layers;
  state["start_validation"] = QJsonObject{{"valid", true}, {"layer_id", 0},
    {"floor_id", "floor1"}, {"height_error_m", 0.0}};
  state["goal_validation"] = QJsonObject{{"valid", false}, {"layer_id", 3},
    {"floor_id", "floor2"}, {"height_error_m", 0.4}, {"reason_code", "height_off_surface"}};
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(state).toJson()));
  auto * selector = panel.findChild<QComboBox *>("active_layer");
  EXPECT_GE(selector->findData(-2), 0);
  EXPECT_GE(selector->findData(-3), 0);
  selector->setCurrentIndex(selector->findData(-3));
  EXPECT_EQ(panel.requested_layer, -3);
  auto * start = panel.findChild<QLabel *>("start_validation");
  auto * goal = panel.findChild<QLabel *>("goal_validation");
  EXPECT_TRUE(start->text().contains("一楼"));
  EXPECT_TRUE(goal->text().contains("二楼"));
  EXPECT_TRUE(goal->text().contains("0.40"));
  EXPECT_FALSE(panel.findChild<QPushButton *>("plan")->isEnabled());
}

TEST(SelectionMode, UnappliedDraftBlocksModeAndLayerChanges)
{
  SelectionPanelProbe panel;
  panel.acceptStatus(selectionStatus("free"));
  auto * x = panel.findChild<QDoubleSpinBox *>("start_x");
  x->setValue(2.345);
  auto * mode = panel.findChild<QComboBox *>("selection_mode");
  auto * layer = panel.findChild<QComboBox *>("active_layer");
  EXPECT_FALSE(mode->isEnabled());
  EXPECT_FALSE(layer->isEnabled());
  mode->setCurrentIndex(mode->findData("ground"));  // Programmatic callbacks are guarded too.
  layer->setCurrentIndex(layer->findData(3));
  EXPECT_EQ(panel.mode_requests, 0);
  EXPECT_EQ(panel.layer_requests, 0);
  EXPECT_DOUBLE_EQ(x->value(), 2.345);
  EXPECT_EQ(mode->currentData().toString(), "free");
  EXPECT_EQ(layer->currentData().toInt(), -1);
}

TEST(SelectionMode, ExternalContextChangePreservesDraftUntilExplicitRestore)
{
  SelectionPanelProbe panel;
  panel.acceptStatus(selectionStatus("free"));
  auto * x = panel.findChild<QDoubleSpinBox *>("start_x");
  x->setValue(2.345);
  panel.acceptStatus(selectionStatus("ground", 3));
  EXPECT_DOUBLE_EQ(x->value(), 2.345);
  EXPECT_FALSE(panel.findChild<QPushButton *>("start_apply")->isEnabled());
  EXPECT_FALSE(panel.findChild<QPushButton *>("plan")->isEnabled());
  EXPECT_TRUE(panel.findChild<QLabel *>("selection_note")->text().contains("草稿已保留"));
  EXPECT_FALSE(panel.findChild<QLabel *>("selection_note")->isHidden());
  panel.findChild<QPushButton *>("start_restore")->click();
  EXPECT_DOUBLE_EQ(x->value(), 1);
  EXPECT_TRUE(panel.findChild<QComboBox *>("selection_mode")->isEnabled());
  EXPECT_TRUE(panel.findChild<QPushButton *>("plan")->isEnabled());
}

TEST(SelectionMode, EndpointReasonReplacesGenericInvalidAndBlocksPlan)
{
  PlanningPanel panel;
  auto state = QJsonDocument::fromJson(selectionStatus().toUtf8()).object();
  state["goal_validation"] = QJsonObject{
    {"valid", false}, {"reason", "occupied obstacle cost exceeds threshold"}, {"layer_id", 3}};
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(state).toJson()));
  EXPECT_TRUE(panel.findChild<QLabel *>("goal_validation")->text().contains("位置在障碍区"));
  EXPECT_TRUE(panel.findChild<QLabel *>("goal_validation")->text().contains("分层 #3"));
  EXPECT_FALSE(panel.findChild<QPushButton *>("plan")->isEnabled());
  EXPECT_TRUE(panel.findChild<QLabel *>("status")->text().contains("请修正"));
}

TEST(SelectionMode, HeartbeatDoesNotRebuildOpenLayerChoices)
{
  PlanningPanel panel;
  panel.acceptStatus(selectionStatus());
  auto * selector = panel.findChild<QComboBox *>("active_layer");
  int removed = 0;
  QObject::connect(selector->model(), &QAbstractItemModel::rowsRemoved,
    [&removed](const QModelIndex &, int, int) {++removed;});
  panel.acceptStatus(selectionStatus());
  panel.acceptStatus(selectionStatus());
  EXPECT_EQ(removed, 0);
  EXPECT_EQ(selector->count(), 3);
}

TEST(SelectionMode, FreeToGroundHeightMismatchKeepsExplicitRecovery)
{
  PlanningPanel panel;
  auto state = QJsonDocument::fromJson(selectionStatus().toUtf8()).object();
  state["start_validation"] = QJsonObject{
    {"valid", false}, {"reason", "height_mismatch"}, {"layer_id", 0}};
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(state).toJson()));
  EXPECT_FALSE(panel.findChild<QPushButton *>("start_snap")->isHidden());
  EXPECT_EQ(panel.findChild<QPushButton *>("start_snap")->text(), "贴回表面");
  EXPECT_TRUE(panel.findChild<QPushButton *>("start_snap")->isEnabled());
}

TEST(SelectionMode, TomogramHeightOffSurfaceSurvivesFreeToGroundWithRecovery)
{
  SelectionPanelProbe panel;
  auto state = QJsonDocument::fromJson(selectionStatus("free").toUtf8()).object();
  state["start_xyz"] = QJsonArray{1, 2, 0.8};
  state["start_validation"] = QJsonObject{
    {"valid", false}, {"reason", "height_off_surface"},
    {"reason_code", "height_off_surface"},
    {"detail", "selected z differs from traversable surface by 1.33 m"}, {"layer_id", 0}};
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(state).toJson()));
  auto * mode = panel.findChild<QComboBox *>("selection_mode");
  mode->setCurrentIndex(mode->findData("ground"));
  EXPECT_EQ(panel.mode_requests, 1);
  state["selection_mode"] = "ground";
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(state).toJson()));
  EXPECT_TRUE(panel.findChild<QDoubleSpinBox *>("start_z")->isReadOnly());
  EXPECT_DOUBLE_EQ(panel.findChild<QDoubleSpinBox *>("start_z")->value(), 0.8);
  auto * recovery = panel.findChild<QPushButton *>("start_snap");
  EXPECT_FALSE(recovery->isHidden());
  EXPECT_EQ(recovery->text(), "贴回表面");
  EXPECT_TRUE(recovery->isEnabled());
  auto * validation = panel.findChild<QLabel *>("start_validation");
  EXPECT_TRUE(validation->text().contains("高度不在表面"));
  EXPECT_TRUE(validation->toolTip().contains("1.33 m"));
  EXPECT_FALSE(panel.findChild<QPushButton *>("plan")->isEnabled());
}

TEST(SelectionMode, TomogramMachineCodesHaveDistinctChineseReasonsAndDetails)
{
  const std::vector<std::pair<QString, QString>> cases{
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
    {"native_abi_mismatch", "规划运行库不兼容，需修复后端"}};
  PlanningPanel panel;
  for (const auto & test : cases) {
    SCOPED_TRACE(test.first.toStdString());
    auto state = QJsonDocument::fromJson(selectionStatus().toUtf8()).object();
    state["goal_validation"] = QJsonObject{
      {"valid", false}, {"reason", "legacy reason"}, {"reason_code", test.first},
      {"detail", "exact backend measurement"}, {"layer_id", 3}};
    panel.acceptStatus(QString::fromUtf8(QJsonDocument(state).toJson()));
    auto * validation = panel.findChild<QLabel *>("goal_validation");
    EXPECT_TRUE(validation->text().contains(test.second));
    EXPECT_FALSE(validation->text().contains(test.first));
    EXPECT_TRUE(validation->toolTip().contains("exact backend measurement"));
    EXPECT_FALSE(panel.findChild<QPushButton *>("plan")->isEnabled());
  }
}

TEST(SelectionMode, PathFailureUsesChineseCodeAndRetainsMeasurementTooltip)
{
  PlanningPanel panel;
  auto state = QJsonDocument::fromJson(selectionStatus().toUtf8()).object();
  state["state"] = "failed";
  state["reason"] = "invalid_curve";
  state["reason_code"] = "invalid_curve";
  state["detail"] = "curve segment 5 intersects blocked cell";
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(state).toJson()));
  EXPECT_EQ(panel.findChild<QLabel *>("status")->text(), "路径经过不可通行格");
  EXPECT_TRUE(panel.findChild<QLabel *>("status")->toolTip().contains("curve segment 5"));
}

TEST(SelectionMode, TopLevelPlannerErrorsAreNotMisreportedAsInvalidEndpoints)
{
  const std::vector<std::pair<QString, QString>> cases{
    {"surface_switch_requires_selection", "地面层变化，请选择编辑层"},
    {"nonadjacent_layer_transition", "路径跨过非相邻分层"},
    {"ground_step", "地面高差超过限制"},
    {"endpoints_too_close", "起终点过近，请移至不同网格"},
    {"native_abi_mismatch", "规划运行库不兼容，需修复后端"}};
  PlanningPanel panel;
  for (const auto & test : cases) {
    SCOPED_TRACE(test.first.toStdString());
    auto state = QJsonDocument::fromJson(selectionStatus().toUtf8()).object();
    state["state"] = "failed";
    state["reason"] = "backend diagnostic message";
    state["error_code"] = test.first;
    state["reason_code"] = QJsonValue::Null;  // Compatibility before server alias exists.
    state["error_details"] = QJsonObject{{"step_m", 0.75}, {"limit_m", 0.15}};
    panel.acceptStatus(QString::fromUtf8(QJsonDocument(state).toJson()));
    auto * status = panel.findChild<QLabel *>("status");
    EXPECT_EQ(status->text(), test.second);
    EXPECT_FALSE(status->text().contains("标记位置"));
    EXPECT_TRUE(status->toolTip().contains("backend diagnostic message"));
    EXPECT_TRUE(status->toolTip().contains("0.75"));
    EXPECT_TRUE(panel.findChild<QLabel *>("start_validation")->text().contains("可通行"));
    EXPECT_TRUE(panel.findChild<QLabel *>("goal_validation")->text().contains("可通行"));
  }
}

TEST(SelectionMode, NativeRuntimeFailureTakesPriorityOverExistingInvalidSelection)
{
  PlanningPanel panel;
  auto state = QJsonDocument::fromJson(selectionStatus().toUtf8()).object();
  state["state"] = "failed";
  state["ready"] = false;
  state["error_code"] = "native_abi_mismatch";
  state["reason"] = "GTSAM symbol mismatch";
  state["error_details"] = "worker runtime isolation needed";
  state["start_validation"] = QJsonObject{
    {"valid", false}, {"reason", "height_off_surface"}, {"layer_id", 0}};
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(state).toJson()));
  EXPECT_EQ(panel.findChild<QLabel *>("status")->text(), "规划运行库不兼容，需修复后端");
  EXPECT_TRUE(panel.findChild<QLabel *>("status")->toolTip().contains("runtime isolation"));
  EXPECT_FALSE(panel.findChild<QPushButton *>("plan")->isEnabled());
}
TEST(Overview, FitsEndpointsAndRouteEnvelopeWithMargin)
{
  const double fov = 3.14159265358979323846 / 2;
  const auto camera = PlanningPanel::overviewCamera({{0, 0, 0}, {20, 0, 0}, {10, 12, 2}}, fov, 1);
  EXPECT_DOUBLE_EQ(camera.center[0], 10);
  EXPECT_DOUBLE_EQ(camera.center[1], 6);
  EXPECT_DOUBLE_EQ(camera.center[2], 1);
  EXPECT_GE(camera.distance * std::sin(fov * 0.5), std::sqrt(137.0) + 1.5 - 1e-9);
  const auto narrow = PlanningPanel::overviewCamera({{0, 0, 0}, {20, 0, 0}, {10, 12, 2}}, fov, 0.5);
  EXPECT_GT(narrow.distance, camera.distance);
}

TEST(Overview, EmptyAndNonFiniteInputsAreSafe)
{
  const auto camera = PlanningPanel::overviewCamera({}, 0, 0);
  EXPECT_DOUBLE_EQ(camera.distance, 8);
  const auto malformed = PlanningPanel::overviewCamera(
    {{std::numeric_limits<double>::quiet_NaN(), 0, 0}}, 0, 0);
  EXPECT_TRUE(std::isfinite(malformed.distance));
  EXPECT_DOUBLE_EQ(malformed.center[0], 0);
}

QString liveDiagnostics(double stamp)
{
  return QString::fromUtf8(QJsonDocument(QJsonObject{
    {"schema", 1}, {"mode", "LIVE_VISUALIZATION_NO_MOTION"}, {"motion_enabled", false},
    {"session_id", "test-session"}, {"frame_id", "d1max_loc_map"}, {"stamp", stamp},
    {"preview_ready", true},
    {"navigation_admission", QJsonObject{{"ready", false},
      {"detail", "外参未完成实机验收；传感器时间对齐未完成实机验收"},
      {"blockers", QJsonArray{"外参未完成实机验收", "传感器时间对齐未完成实机验收"}},
      {"global_observed_hz", 50.0}}},
    {"stages", QJsonObject{
      {"localization", QJsonObject{{"label", "已定位"}, {"tone", "ready"}}},
      {"global", QJsonObject{{"label", "路径有效"}, {"tone", "ready"}}},
      {"local", QJsonObject{{"label", "轨迹有效"}, {"tone", "ready"}}}}},
    {"body_age", .02}, {"cloud_age", .10}, {"local_target", QJsonArray{1.25, 2.5, .55}},
    {"reference_horizon_m", 6.}, {"plan_id", 17}}).toJson());
}

void configureDiagnostics(NavigationDiagnosticsPanel & panel)
{
  rviz_common::Config config;
  config.mapSetValue("Session ID", "test-session");
  panel.load(config);
}

TEST(NavigationDiagnostics, LoadsAsPassiveRvizPlugin)
{
  pluginlib::ClassLoader<rviz_common::Panel> panels("rviz_common", "rviz_common::Panel");
  auto panel = panels.createSharedInstance("d1max_pct_rviz_tools/NavigationDiagnosticsPanel");
  ASSERT_NE(panel, nullptr);
  EXPECT_EQ(panel->findChildren<QPushButton *>().size(), 2);  // Presentation layouts only.
  EXPECT_NE(panel->findChild<QToolButton *>("spatial_view"), nullptr);
}

TEST(NavigationDiagnostics, DisplaysActualTargetAndMinimalLegend)
{
  NavigationDiagnosticsPanel panel;
  configureDiagnostics(panel);
  panel.acceptStatus(liveDiagnostics(QDateTime::currentMSecsSinceEpoch() / 1000.));
  EXPECT_TRUE(panel.findChild<QLabel *>("local")->text().contains("轨迹有效"));
  EXPECT_TRUE(panel.findChild<QLabel *>("target")->text().contains("1.25, 2.50, 0.55"));
  EXPECT_TRUE(panel.findChild<QLabel *>("ages")->text().contains("20 ms"));
  EXPECT_TRUE(panel.findChild<QLabel *>("horizon")->text().contains("6.00 m"));
  EXPECT_TRUE(panel.findChild<QLabel *>("mode")->text().contains("运动关闭"));
  EXPECT_EQ(panel.findChildren<QLabel *>("legend").size(), 9);
  QStringList legends;
  for (const auto * item : panel.findChildren<QLabel *>("legend")) {
    legends.append(item->text());
  }
  EXPECT_TRUE(legends.join(" ").contains("机身坐标系 · XYZ / 红绿蓝"));
  EXPECT_FALSE(legends.join(" ").contains("当前机身"));
}

TEST(NavigationDiagnostics, RejectsReplayWrongSessionAndStaleSource)
{
  NavigationDiagnosticsPanel panel;
  configureDiagnostics(panel);
  const auto now = QDateTime::currentMSecsSinceEpoch() / 1000.;
  panel.acceptStatus(liveDiagnostics(now - 4.));
  EXPECT_TRUE(panel.findChild<QLabel *>("local")->text().contains("等待"));
  panel.acceptStatus(liveDiagnostics(now).replace("test-session", "old-session"));
  EXPECT_TRUE(panel.findChild<QLabel *>("local")->text().contains("等待"));
  panel.acceptStatus(liveDiagnostics(now));
  panel.acceptStatus(liveDiagnostics(now).replace("轨迹有效", "错误重放"));
  EXPECT_TRUE(panel.findChild<QLabel *>("local")->text().contains("轨迹有效"));
}

TEST(NavigationDiagnostics, ShowsActualTreeNodeWithoutGrantingMotion)
{
  NavigationDiagnosticsPanel panel;
  configureDiagnostics(panel);
  const auto now = QDateTime::currentMSecsSinceEpoch() / 1000.;
  auto value = QJsonDocument::fromJson(liveDiagnostics(now).toUtf8()).object();
  value["behavior_tree"] = QJsonObject{{"label", "等待输入恢复"},
    {"task_id", "test-session.2"}, {"root_status", "RUNNING"},
    {"detail", "WaitForNavigationInputs · waiting_localization"},
    {"nodes", QJsonArray{QJsonObject{{"name", "ComputeRouteOnce"}, {"status", "SUCCESS"}},
                        QJsonObject{{"name", "FollowRoute"}, {"status", "RUNNING"}}}}};
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(value).toJson()));
  EXPECT_EQ(panel.findChild<QLabel *>("behavior_tree")->text(), "等待输入恢复");
  EXPECT_TRUE(panel.findChild<QLabel *>("behavior_tree")->toolTip().contains("WaitForNavigationInputs"));
  EXPECT_TRUE(panel.findChild<QLabel *>("detail")->text().contains("ComputeRouteOnce: SUCCESS"));
  EXPECT_TRUE(panel.findChild<QLabel *>("mode")->text().contains("运动关闭"));
  EXPECT_TRUE(panel.findChild<QLabel *>("navigation_admission")->text().contains("未通过"));
  value.remove("behavior_tree");
  value["stamp"] = now + .001;
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(value).toJson()));
  EXPECT_EQ(panel.findChild<QLabel *>("behavior_tree")->text(), "等待任务状态");
}

TEST(NavigationDiagnostics, PreviewSuccessDoesNotConcealNavigationAdmission)
{
  NavigationDiagnosticsPanel panel;
  configureDiagnostics(panel);
  panel.acceptStatus(liveDiagnostics(QDateTime::currentMSecsSinceEpoch() / 1000.));
  EXPECT_TRUE(panel.findChild<QLabel *>("localization")->text().contains("已定位"));
  auto * admission = panel.findChild<QLabel *>("navigation_admission");
  ASSERT_NE(admission, nullptr);
  EXPECT_TRUE(admission->text().contains("未通过"));
  EXPECT_TRUE(admission->toolTip().contains("外参未完成"));
  EXPECT_TRUE(panel.findChild<QLabel *>("detail")->text().contains("50.0 Hz"));
}

TEST(NavigationDiagnostics, MissingAdmissionCannotRetainOldGreen)
{
  NavigationDiagnosticsPanel panel;
  configureDiagnostics(panel);
  const auto now = QDateTime::currentMSecsSinceEpoch() / 1000.;
  auto status = QJsonDocument::fromJson(liveDiagnostics(now).toUtf8()).object();
  status["navigation_admission"] = QJsonObject{
    {"ready", true}, {"detail", ""}, {"blockers", QJsonArray{}}};
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(status).toJson()));
  EXPECT_TRUE(panel.findChild<QLabel *>("navigation_admission")->text().contains("已通过"));
  status.remove("navigation_admission");
  status["stamp"] = now + .001;
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(status).toJson()));
  EXPECT_TRUE(panel.findChild<QLabel *>("navigation_admission")->text().contains("状态未知"));
  EXPECT_TRUE(panel.findChild<QLabel *>("localization")->text().contains("已定位"));
}

TEST(NavigationDiagnostics, AdmissionCannotTurnGreenWhenPreviewIsInvalid)
{
  NavigationDiagnosticsPanel panel;
  configureDiagnostics(panel);
  auto status = QJsonDocument::fromJson(liveDiagnostics(
      QDateTime::currentMSecsSinceEpoch() / 1000.).toUtf8()).object();
  status["navigation_admission"] = QJsonObject{
    {"ready", true}, {"detail", ""}, {"blockers", QJsonArray{}}};
  status["preview_ready"] = false;
  panel.acceptStatus(QString::fromUtf8(QJsonDocument(status).toJson()));
  EXPECT_TRUE(panel.findChild<QLabel *>("navigation_admission")->text().contains("未通过"));
}

TEST(NavigationDiagnostics, MalformedModeOrPayloadCannotTurnGreen)
{
  NavigationDiagnosticsPanel panel;
  configureDiagnostics(panel);
  const auto now = QDateTime::currentMSecsSinceEpoch() / 1000.;
  panel.acceptStatus(liveDiagnostics(now).replace("LIVE_VISUALIZATION_NO_MOTION", "live_control"));
  panel.acceptStatus("{}");
  panel.acceptStatus(QString(18000, 'x'));
  EXPECT_TRUE(panel.findChild<QLabel *>("local")->text().contains("等待"));
}

TEST(NavigationDiagnostics, ExpiryClearsHealthyStateAndTarget)
{
  NavigationDiagnosticsPanel panel;
  configureDiagnostics(panel);
  panel.acceptStatus(liveDiagnostics(QDateTime::currentMSecsSinceEpoch() / 1000. - .85));
  EXPECT_TRUE(panel.findChild<QLabel *>("local")->text().contains("轨迹有效"));
  QEventLoop wait;
  QTimer::singleShot(250, &wait, &QEventLoop::quit);
  wait.exec();
  panel.refresh();
  EXPECT_TRUE(panel.findChild<QLabel *>("local")->text().contains("等待"));
  EXPECT_FALSE(panel.findChild<QLabel *>("target")->text().contains("1.25"));
  EXPECT_TRUE(panel.findChild<QLabel *>("navigation_admission")->text().contains("状态未知"));
}

TEST(NavigationDiagnostics, RendersWithoutOversizedDock)
{
  NavigationDiagnosticsPanel panel;
  configureDiagnostics(panel);
  panel.acceptStatus(liveDiagnostics(QDateTime::currentMSecsSinceEpoch() / 1000.));
  panel.resize(310, 710); panel.show(); QApplication::processEvents();
  EXPECT_LE(panel.minimumWidth(), 310);
  const auto image = panel.grab();
  EXPECT_FALSE(image.isNull());
  EXPECT_TRUE(image.save("/tmp/d1max-navigation-diagnostics-panel.png"));
  panel.findChild<QPushButton *>("layout_local")->click();
  QApplication::processEvents();
  auto * scroll = panel.findChild<QScrollArea *>();
  ASSERT_NE(scroll, nullptr);
  EXPECT_EQ(scroll->horizontalScrollBar()->maximum(), 0);
  EXPECT_TRUE(panel.grab().save("/tmp/d1max-navigation-local-panel.png"));
}

class LayoutPanelProbe : public NavigationDiagnosticsPanel
{
public:
  std::vector<QString> layouts;
protected:
  void applyLayoutPresentation(const QString & layout) override {layouts.push_back(layout);}
};

// Exercise RViz's real property tree without constructing an Ogre scene or ROS
// node. The rendering hook is deliberately omitted, not the enabled property.
class HeadlessDisplay : public rviz_common::Display
{
public:
  void onEnableChanged() override {}
};
class HeadlessDisplayGroup : public rviz_common::DisplayGroup
{
public:
  void onEnableChanged() override {}
};

TEST(NavigationLayouts, TwoExclusivePresetsPersistAndDoNotDependOnStatus)
{
  LayoutPanelProbe panel;
  configureDiagnostics(panel);
  QApplication::processEvents();
  auto * global = panel.findChild<QPushButton *>("layout_global");
  auto * local = panel.findChild<QPushButton *>("layout_local");
  ASSERT_NE(global, nullptr); ASSERT_NE(local, nullptr);
  EXPECT_TRUE(global->isChecked()); EXPECT_FALSE(local->isChecked());
  EXPECT_TRUE(panel.findChild<QLabel *>("target")->isHidden());
  local->click();
  EXPECT_TRUE(local->isChecked()); EXPECT_FALSE(global->isChecked());
  EXPECT_FALSE(panel.findChild<QLabel *>("target")->isHidden());
  ASSERT_FALSE(panel.layouts.empty()); EXPECT_EQ(panel.layouts.back(), "local");
  int visible_legends = 0;
  for (auto * row : panel.findChildren<QWidget *>("legend_row")) {
    visible_legends += !row->isHidden();
  }
  EXPECT_EQ(visible_legends, 8);
  const auto applied = panel.layouts.size();
  panel.acceptStatus(liveDiagnostics(QDateTime::currentMSecsSinceEpoch() / 1000.));
  panel.refresh();
  EXPECT_EQ(panel.layouts.size(), applied);  // Heartbeats never reset camera/layers.
  rviz_common::Config saved;
  panel.save(saved);
  QString key;
  EXPECT_TRUE(saved.mapGetString("Layout", &key)); EXPECT_EQ(key, "local");
  LayoutPanelProbe restored;
  restored.load(saved);
  QApplication::processEvents();
  EXPECT_TRUE(restored.findChild<QPushButton *>("layout_local")->isChecked());
  ASSERT_FALSE(restored.layouts.empty()); EXPECT_EQ(restored.layouts.back(), "local");
  global->click();
  EXPECT_TRUE(global->isChecked()); EXPECT_FALSE(local->isChecked());
  visible_legends = 0;
  for (auto * row : panel.findChildren<QWidget *>("legend_row")) {
    visible_legends += !row->isHidden();
  }
  EXPECT_EQ(visible_legends, 3);
}

TEST(NavigationLayouts, InvalidSavedLayoutUsesGlobalWithoutEnablingNavigation)
{
  LayoutPanelProbe panel;
  rviz_common::Config config;
  config.mapSetValue("Layout", "start_motion");
  panel.load(config);
  QApplication::processEvents();
  EXPECT_TRUE(panel.findChild<QPushButton *>("layout_global")->isChecked());
  ASSERT_FALSE(panel.layouts.empty()); EXPECT_EQ(panel.layouts.back(), "global");
  EXPECT_TRUE(panel.findChild<QLabel *>("mode")->text().contains("运动关闭"));
}

class LayoutRequestFixture : public testing::Test
{
protected:
  QTemporaryDir directory;
  QByteArray previous_viewer;
  bool previous_set{false};
  const QString viewer{"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"};
  QString requestPath() const {return directory.path() + "/navigation-view-layout-request.json";}
  QString statePath() const {return directory.path() + "/navigation-view-layout-state.json";}
  void SetUp() override
  {
    ASSERT_TRUE(directory.isValid());
    previous_set = qEnvironmentVariableIsSet("D1MAX_NAV_RVIZ_VIEWER_ID");
    previous_viewer = qgetenv("D1MAX_NAV_RVIZ_VIEWER_ID");
    qputenv("D1MAX_NAV_RVIZ_VIEWER_ID", viewer.toUtf8());
  }
  void TearDown() override
  {
    if (previous_set) {qputenv("D1MAX_NAV_RVIZ_VIEWER_ID", previous_viewer);}
    else {qunsetenv("D1MAX_NAV_RVIZ_VIEWER_ID");}
  }
  void configure(LayoutPanelProbe & panel)
  {
    rviz_common::Config config;
    config.mapSetValue("Session ID", "test-session");
    config.mapSetValue("Layout Request File", requestPath());
    config.mapSetValue("Layout State File", statePath());
    panel.load(config); QApplication::processEvents();
  }
  QJsonObject request(const QString & id = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb") const
  {
    return {{"schema", 1}, {"session_id", "test-session"}, {"viewer_id", viewer},
      {"request_id", id}, {"layout", "local"},
      {"stamp", QDateTime::currentMSecsSinceEpoch() / 1000.}};
  }
  QString encode(const QJsonObject & value) const
  {return QString::fromUtf8(QJsonDocument(value).toJson(QJsonDocument::Compact));}
  void writeRequest(const QJsonObject & value)
  {
    QFile file(requestPath()); ASSERT_TRUE(file.open(QIODevice::WriteOnly | QIODevice::Truncate));
    ASSERT_TRUE(file.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner));
    const auto data = encode(value).toUtf8(); ASSERT_EQ(file.write(data), data.size());
  }
};

TEST_F(LayoutRequestFixture, PrivateRequestSwitchesSamePanelAndAcknowledgesActualProcess)
{
  LayoutPanelProbe panel; configure(panel);
  const auto value = request(); writeRequest(value); panel.refresh();
  ASSERT_TRUE(panel.findChild<QPushButton *>("layout_local")->isChecked());
  ASSERT_EQ(panel.layouts.back(), "local");
  QFile state(statePath()); ASSERT_TRUE(state.open(QIODevice::ReadOnly));
  const auto ack = QJsonDocument::fromJson(state.readAll()).object();
  for (const auto * field : {"schema", "session_id", "viewer_id", "request_id", "layout", "stamp"}) {
    EXPECT_EQ(ack[field], value[field]);
  }
  EXPECT_EQ(ack["phase"].toString(), "applied");
  EXPECT_EQ(ack["rviz_pid"].toInt(), QApplication::applicationPid());
  EXPECT_GT(ack["applied_at_unix"].toDouble(), 0.);
  EXPECT_FALSE(state.permissions() & (QFileDevice::ReadGroup | QFileDevice::WriteGroup |
    QFileDevice::ExeGroup | QFileDevice::ReadOther | QFileDevice::WriteOther | QFileDevice::ExeOther));
  const auto applied = panel.layouts.size();
  panel.refresh(); panel.acceptStatus(liveDiagnostics(QDateTime::currentMSecsSinceEpoch() / 1000.));
  panel.refresh(); EXPECT_EQ(panel.layouts.size(), applied);
  panel.findChild<QPushButton *>("layout_global")->click();
  panel.refresh(); EXPECT_TRUE(panel.findChild<QPushButton *>("layout_global")->isChecked());
  EXPECT_EQ(panel.layouts.size(), applied + 1);  // Old request cannot undo the mouse selection.
}

TEST_F(LayoutRequestFixture, RejectsWrongOwnerContractReplayExpiryAndUnknownLayouts)
{
  LayoutPanelProbe panel; configure(panel);
  const auto applied = panel.layouts.size();
  const auto valid = request();
  for (const auto * field : {"session_id", "viewer_id", "request_id", "layout"}) {
    auto bad = valid; bad[field] = "wrong"; EXPECT_FALSE(panel.acceptLayoutRequest(encode(bad)));
  }
  for (const double offset : {-10.5, .5}) {
    auto bad = valid; bad["stamp"] = valid["stamp"].toDouble() + offset;
    EXPECT_FALSE(panel.acceptLayoutRequest(encode(bad)));
  }
  auto bad = valid; bad["schema"] = 2; EXPECT_FALSE(panel.acceptLayoutRequest(encode(bad)));
  bad = valid; bad["stamp"] = "now"; EXPECT_FALSE(panel.acceptLayoutRequest(encode(bad)));
  EXPECT_FALSE(panel.acceptLayoutRequest(QString(5000, 'x')));
  EXPECT_FALSE(panel.acceptLayoutRequest("[]"));
  EXPECT_EQ(panel.layouts.size(), applied); EXPECT_FALSE(QFile::exists(statePath()));
  EXPECT_TRUE(panel.acceptLayoutRequest(encode(valid)));
  EXPECT_FALSE(panel.acceptLayoutRequest(encode(valid)));
  auto same = request("cccccccccccccccccccccccccccccccc");
  const auto selected = panel.layouts.size(); EXPECT_TRUE(panel.acceptLayoutRequest(encode(same)));
  EXPECT_EQ(panel.layouts.size(), selected);  // A new same-layout request does not reset the camera.
}

TEST_F(LayoutRequestFixture, PollRejectsSymlinkPublicAndOversizedFilesAndUnsafeState)
{
  LayoutPanelProbe panel; configure(panel);
  const auto applied = panel.layouts.size();
  writeRequest(request());
  ASSERT_TRUE(QFile::setPermissions(requestPath(), QFileDevice::ReadOwner |
    QFileDevice::WriteOwner | QFileDevice::ReadOther));
  panel.refresh(); EXPECT_EQ(panel.layouts.size(), applied);
  ASSERT_TRUE(QFile::remove(requestPath()));
  QFile real(directory.path() + "/actual.json"); ASSERT_TRUE(real.open(QIODevice::WriteOnly));
  ASSERT_TRUE(real.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner));
  const auto content = encode(request()).toUtf8(); ASSERT_EQ(real.write(content), content.size());
  real.close(); ASSERT_TRUE(QFile::link(real.fileName(), requestPath()));
  panel.refresh(); EXPECT_EQ(panel.layouts.size(), applied);
  ASSERT_TRUE(QFile::remove(requestPath()));
  QFile large(requestPath()); ASSERT_TRUE(large.open(QIODevice::WriteOnly));
  ASSERT_TRUE(large.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner));
  ASSERT_EQ(large.write(QByteArray(5000, 'x')), 5000); large.close();
  panel.refresh(); EXPECT_EQ(panel.layouts.size(), applied);
  ASSERT_TRUE(QFile::remove(requestPath()));
  ASSERT_EQ(::mkfifo(QFile::encodeName(requestPath()).constData(), 0600), 0);
  panel.refresh(); EXPECT_EQ(panel.layouts.size(), applied);  // No writer: O_NONBLOCK must return.
  ASSERT_TRUE(QFile::link(real.fileName(), statePath()));
  EXPECT_FALSE(panel.acceptLayoutRequest(encode(request())));
  panel.findChild<QPushButton *>("layout_global")->click();
  EXPECT_FALSE(panel.acceptLayoutRequest(encode(request())));
  EXPECT_TRUE(panel.findChild<QPushButton *>("layout_global")->isChecked());
  QFile untouched(real.fileName()); ASSERT_TRUE(untouched.open(QIODevice::ReadOnly));
  EXPECT_EQ(untouched.readAll(), content);  // ACK never writes through an existing symlink.
}

TEST_F(LayoutRequestFixture, MissingViewerNonceDisablesOnlyExternalRequests)
{
  qunsetenv("D1MAX_NAV_RVIZ_VIEWER_ID");
  LayoutPanelProbe panel; configure(panel); writeRequest(request()); panel.refresh();
  EXPECT_TRUE(panel.findChild<QPushButton *>("layout_global")->isChecked());
  EXPECT_FALSE(QFile::exists(statePath()));
  panel.findChild<QPushButton *>("layout_local")->click();
  EXPECT_TRUE(panel.findChild<QPushButton *>("layout_local")->isChecked());
}

TEST(NavigationLayouts, VisibilityTreeAndRouteEmphasisAreReversible)
{
  HeadlessDisplayGroup root;
  auto add_group = [&root](const QString & name) {
      auto * value = new HeadlessDisplayGroup;
      value->setName(name); root.addDisplay(value); return value;
    };
  auto add = [](rviz_common::DisplayGroup * group, const QString & name) {
      auto * value = new HeadlessDisplay;
      value->setName(name); value->setEnabled(true); group->addDisplay(value); return value;
    };
  auto * localization = add_group("定位");
  auto * maps = add_group("地图");
  auto * global = add_group("全局规划");
  auto * local = add_group("局部规划");
  auto * axes = add(localization, "机器狗坐标系");
  auto * history = add(localization, "定位轨迹");
  auto * initial = add(localization, "初值预览 · 未确认");
  auto * route = add(global, "全局路径");
  auto * handle = add(global, "3D 目标手柄");
  auto * width = new rviz_common::properties::Property("Line Width", .065, "", route);
  auto * alpha = new rviz_common::properties::Property("Alpha", 1., "", route);
  auto * occupancy = add(local, "滑动占据地图（高度）");
  auto * inflated = add(local, "碰撞膨胀地图（高度）");
  auto * legacy_trajectory = add(local, "局部轨迹");
  auto * committed_trajectory = add(local, "已提交局部轨迹");
  auto * boundary = add(local, "滑动窗口边界");
  root.setEnabled(true);
  NavigationDiagnosticsPanel::applyLayoutVisibility(&root, "global");
  EXPECT_TRUE(maps->isEnabled()); EXPECT_FALSE(local->isEnabled());
  EXPECT_TRUE(history->isEnabled()); EXPECT_TRUE(initial->isEnabled());
  EXPECT_TRUE(axes->isEnabled()); EXPECT_TRUE(handle->isEnabled());
  EXPECT_DOUBLE_EQ(width->getValue().toDouble(), .065);
  EXPECT_DOUBLE_EQ(alpha->getValue().toDouble(), 1.);
  NavigationDiagnosticsPanel::applyLayoutVisibility(&root, "local");
  EXPECT_FALSE(maps->isEnabled()); EXPECT_TRUE(local->isEnabled());
  EXPECT_FALSE(history->isEnabled()); EXPECT_FALSE(initial->isEnabled());
  EXPECT_TRUE(axes->isEnabled()); EXPECT_FALSE(handle->isEnabled());
  EXPECT_TRUE(occupancy->isEnabled()); EXPECT_TRUE(inflated->isEnabled());
  EXPECT_TRUE(legacy_trajectory->isEnabled()); EXPECT_TRUE(committed_trajectory->isEnabled());
  EXPECT_TRUE(boundary->isEnabled());
  EXPECT_TRUE(route->isEnabled());
  EXPECT_DOUBLE_EQ(width->getValue().toDouble(), .035);
  EXPECT_DOUBLE_EQ(alpha->getValue().toDouble(), .35);
  occupancy->setEnabled(false); inflated->setEnabled(false);
  legacy_trajectory->setEnabled(false); committed_trajectory->setEnabled(false);
  NavigationDiagnosticsPanel::applyLayoutVisibility(&root, "local");
  EXPECT_TRUE(occupancy->isEnabled()); EXPECT_TRUE(inflated->isEnabled());
  EXPECT_TRUE(legacy_trajectory->isEnabled()); EXPECT_TRUE(committed_trajectory->isEnabled());
  NavigationDiagnosticsPanel::applyLayoutVisibility(&root, "invalid");
  EXPECT_TRUE(local->isEnabled());
  NavigationDiagnosticsPanel::applyLayoutVisibility(&root, "global");
  EXPECT_TRUE(maps->isEnabled()); EXPECT_FALSE(local->isEnabled());
  EXPECT_TRUE(axes->isEnabled()); EXPECT_TRUE(handle->isEnabled());
  EXPECT_DOUBLE_EQ(width->getValue().toDouble(), .065);
  NavigationDiagnosticsPanel::applyLayoutVisibility(nullptr, "local");
}

int main(int argc, char ** argv)
{
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
