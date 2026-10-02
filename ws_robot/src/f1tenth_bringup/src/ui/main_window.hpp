// Window frame: top bar (session, driver, battery, link, E-STOP), page rail, pages and the status ticker.
#pragma once

#include <QMainWindow>
#include <QString>

#include <string>
#include <vector>

#include "car_picker.hpp"
#include "pages.hpp"
#include "ros_bridge.hpp"

class QLabel;
class QPushButton;
class QStackedWidget;
class QUdpSocket;
class QTimer;

namespace f1ui
{
class NavButton;
class SkewBadge;
class StatusLed;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(RosBridge * ros, const std::string & rviz_node_name, const CarInfo & car);

protected:
  void showEvent(QShowEvent * event) override;
  void closeEvent(QCloseEvent * event) override;
  bool eventFilter(QObject * watched, QEvent * event) override;

private:
  enum PageIndex {PageHome, PageSession, PageGarage, PageTelemetry, PageSystem, PageParams, PageLog};

  QWidget * makeTopBar();
  void showPage(int index);
  void notify(Level level, const QString & text);
  void onTelemetry(const Telemetry & t);
  void onSupervisor(const SupervisorState & state);
  void onDrive(const DriveState & d);
  void onLink(bool car, bool supervisor);
  void setEstopShown(bool engaged, bool held_here);
  void toggleEstop();
  void screenshot(const QString & path = QString());
  void runTour(const QString & dir);
  void switchCar();
  void watchCarAddress();

  RosBridge * ros_;
  CarInfo car_;
  // Auto reconnect: the same car announcing itself from another address (e.g. after a Wi-Fi change)
  QUdpSocket * beacon_ = nullptr;
  CarInfo moved_;
  qint64 link_lost_ms_ = 0;
  bool reconnecting_ = false;
  QStackedWidget * pages_ = nullptr;
  std::vector<NavButton *> nav_;
  HomePage * home_ = nullptr;
  SessionPage * session_ = nullptr;
  GaragePage * garage_ = nullptr;
  TelemetryPage * telemetry_ = nullptr;
  SystemPage * system_ = nullptr;
  ParamsPage * params_ = nullptr;
  LogPage * log_ = nullptr;

  SkewBadge * mode_badge_ = nullptr;
  QLabel * track_ = nullptr;
  SkewBadge * driver_badge_ = nullptr;
  QLabel * battery_ = nullptr;
  StatusLed * link_led_ = nullptr;
  QLabel * link_ = nullptr;
  QPushButton * estop_ = nullptr;
  QLabel * estop_banner_ = nullptr;
  QTimer * estop_flash_ = nullptr;
  bool estop_engaged_ = false, estop_phase_ = false;
  QLabel * ticker_ = nullptr;
  QLabel * clock_ = nullptr;
  QTimer * ticker_timer_ = nullptr;
  QString last_supervisor_message_;
  bool scene_ready_ = false;
  bool low_battery_warned_ = false;
  QString mode_;
  bool closing_ = false;
};
}  // namespace f1ui
