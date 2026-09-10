#include <QApplication>
#include <QCoreApplication>
#include <QFont>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"

#include "d1max_sdk_gui/main_window.hpp"

int main(int argc, char** argv) {
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  QApplication application(argc, argv);
  application.setApplicationName("D1 Max Behavior Control Center");
  application.setOrganizationName("D1 Max");
  application.setFont(QFont("Noto Sans CJK SC", 10));

  rclcpp::init(argc, argv);
  QTimer ros_shutdown_monitor;
  QObject::connect(&ros_shutdown_monitor, &QTimer::timeout,
                   &application, [&application]() {
                     if (!rclcpp::ok()) {
                       application.quit();
                     }
                   });
  ros_shutdown_monitor.start(100);

  int result = 0;
  {
    d1max_sdk_gui::MainWindow window;
    window.show();
    result = application.exec();
  }
  rclcpp::shutdown();
  return result;
}
