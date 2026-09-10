#pragma once

#include <memory>

#include <QList>
#include <QMainWindow>
#include <QString>

class QCheckBox;
class QCloseEvent;
class QDoubleSpinBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QTimer;

namespace d1max_sdk_gui {

class RosBridgeClient;

class MainWindow final : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow() override;

 protected:
  void closeEvent(QCloseEvent* event) override;

 private:
  QWidget* BuildHeader();
  QWidget* BuildStatusPanel();
  QWidget* BuildBehaviorPanel();
  QWidget* BuildEventPanel();
  QWidget* BuildTeleopPanel();
  QPushButton* MakeCommandButton(const QString& text,
                                 const QString& service,
                                 const QString& confirmation = QString());
  QLabel* MakeValueLabel(const QString& initial);
  void ConnectRosSignals();
  void SetConnected(bool connected, const QString& text);
  void SetNavigationReady(bool ready);
  void AppendEvent(const QString& message, const QString& kind = "INFO");
  void RequestService(const QString& service, const QString& confirmation);
  void BeginJog(double forward_sign, double lateral_sign, double yaw_sign);
  void EndJog();
  void ApplyBadge(QLabel* label, const QString& text, const QString& color,
                  const QString& background);

  std::unique_ptr<RosBridgeClient> ros_;
  bool connected_{false};
  bool navigation_ready_{false};
  double jog_forward_{0.0};
  double jog_lateral_{0.0};
  double jog_yaw_{0.0};

  QLabel* connection_badge_{nullptr};
  QLabel* nav_badge_{nullptr};
  QLabel* motion_value_{nullptr};
  QLabel* mode_value_{nullptr};
  QLabel* control_value_{nullptr};
  QLabel* fsm_value_{nullptr};
  QLabel* goal_value_{nullptr};
  QLabel* transition_value_{nullptr};
  QLabel* estop_value_{nullptr};
  QLabel* battery_value_{nullptr};
  QLabel* fault_badge_{nullptr};
  QPlainTextEdit* event_log_{nullptr};
  QCheckBox* teleop_enable_{nullptr};
  QDoubleSpinBox* forward_speed_{nullptr};
  QDoubleSpinBox* lateral_speed_{nullptr};
  QDoubleSpinBox* yaw_speed_{nullptr};
  QTimer* teleop_timer_{nullptr};
  QPushButton* estop_button_{nullptr};
  QPushButton* recover_button_{nullptr};
  QList<QPushButton*> command_buttons_;
  QList<QPushButton*> jog_buttons_;
};

}  // namespace d1max_sdk_gui
