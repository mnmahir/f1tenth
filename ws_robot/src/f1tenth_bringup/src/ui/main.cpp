// f1tenth_ui: the F1-themed pit wall. Runs on the car's screen or on any computer on the car's network.
//   f1tenth_ui              pick a car (on the car itself: that car)
//   f1tenth_ui --car NAME   connect to NAME (or an address) without asking, if it is found
//   f1tenth_ui --pick       always show the car picker
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <QApplication>
#include <QPalette>
#include <QSocketNotifier>

#include "car_picker.hpp"
#include "main_window.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_bridge.hpp"
#include "rviz_common/logging.hpp"
#include "theme.hpp"

namespace
{
int signal_fds[2] = {-1, -1};

void onSignal(int)
{
  char c = 1;
  ssize_t written = ::write(signal_fds[1], &c, 1);
  (void)written;
}

QPalette darkPalette()
{
  using namespace f1ui::theme;
  QPalette p;
  p.setColor(QPalette::Window, bg0);
  p.setColor(QPalette::WindowText, text);
  p.setColor(QPalette::Base, QColor(15, 15, 21));
  p.setColor(QPalette::AlternateBase, bg1);
  p.setColor(QPalette::Text, text);
  p.setColor(QPalette::Button, bg2);
  p.setColor(QPalette::ButtonText, text);
  p.setColor(QPalette::BrightText, redBright);
  p.setColor(QPalette::Highlight, red);
  p.setColor(QPalette::HighlightedText, text);
  p.setColor(QPalette::ToolTipBase, bg2);
  p.setColor(QPalette::ToolTipText, text);
  p.setColor(QPalette::PlaceholderText, text3);
  p.setColor(QPalette::Link, cyan);
  p.setColor(QPalette::Disabled, QPalette::Text, text3);
  p.setColor(QPalette::Disabled, QPalette::ButtonText, text3);
  p.setColor(QPalette::Disabled, QPalette::WindowText, text3);
  return p;
}

void installSignalHandlers(QApplication & app)
{
  if (::socketpair(AF_UNIX, SOCK_STREAM, 0, signal_fds) != 0) {
    return;
  }
  auto * notifier = new QSocketNotifier(signal_fds[0], QSocketNotifier::Read, &app);
  QObject::connect(notifier, &QSocketNotifier::activated, &app, [&app]() {
      char c;
      ssize_t got = ::read(signal_fds[0], &c, 1);
      (void)got;
      app.quit();
    });
  struct sigaction action {};
  action.sa_handler = onSignal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESTART;
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);
}

// Which car to drive: this one when the UI runs on a car, otherwise the one picked from the network
bool chooseCar(const QStringList & args, f1ui::CarInfo & car)
{
  QString wanted = qEnvironmentVariable("F1TENTH_CAR");
  int at = args.indexOf("--car");
  if (at >= 0 && at + 1 < args.size()) {
    wanted = args[at + 1];
  }
  if (!args.contains("--pick") && wanted.isEmpty() && f1ui::HomePage::isCar()) {
    char host[256] = {0};
    gethostname(host, sizeof(host) - 1);
    car.name = qEnvironmentVariable("F1TENTH_CAR_NAME", QString::fromLocal8Bit(host));
    car.host = QString::fromLocal8Bit(host);
    car.address = "this computer";
    car.domain = qEnvironmentVariableIntValue("ROS_DOMAIN_ID");
    return true;
  }
  f1ui::CarPicker picker;
  if (!wanted.isEmpty()) {
    picker.autoConnect(wanted, 4000);
  }
  if (picker.exec() != QDialog::Accepted) {
    return false;
  }
  car = picker.selected();
  f1ui::CarPicker::rememberCar(car);
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  QApplication app(argc, argv);
  app.setApplicationName("F1TENTH Pit Wall");
  app.setStyle("Fusion");
  f1ui::theme::loadFonts();
  app.setFont(f1ui::theme::font(13));
  app.setPalette(darkPalette());
  app.setStyleSheet(f1ui::theme::styleSheet());
  installSignalHandlers(app);

  f1ui::CarInfo car;
  if (!chooseCar(app.arguments(), car)) {
    return 0;
  }
  // Join only that car's ROS domain (and reach it through its router when it's on another network)
  f1ui::configureMiddleware(car);
  // Qt handles SIGINT/SIGTERM (above) so the window closes cleanly
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  // RViz's messages go to the ROS log; its debug output is dropped
  auto rviz_logger = rclcpp::get_logger("f1tenth_ui.scene");
  rviz_common::set_logging_handlers(
    [](const std::string &, const std::string &, size_t) {},
    [rviz_logger](const std::string & m, const std::string &, size_t) {RCLCPP_INFO(rviz_logger, "%s", m.c_str());},
    [rviz_logger](const std::string & m, const std::string &, size_t) {RCLCPP_WARN(rviz_logger, "%s", m.c_str());},
    [rviz_logger](const std::string & m, const std::string &, size_t) {RCLCPP_ERROR(rviz_logger, "%s", m.c_str());});
  rviz_common::install_rviz_rendering_log_handlers();

  int code = 0;
  {
    f1ui::RosBridge ros;
    f1ui::MainWindow window(&ros, std::string(ros.node()->get_name()) + "_scene", car);
    window.resize(1680, 1000);
    window.show();
    code = app.exec();
  }
  rclcpp::shutdown();
  return code;
}
