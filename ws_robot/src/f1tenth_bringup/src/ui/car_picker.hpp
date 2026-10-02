// Finding and choosing the car, before ROS starts: every car's supervisor announces its name, ROS domain, mode and
// battery on UDP. The UI joins that car's domain only, so it can't see or command any other car.
#pragma once

#include <QDialog>
#include <QString>

#include <map>

class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QTimer;
class QUdpSocket;

namespace f1ui
{
struct CarInfo
{
  QString name;
  QString host;
  QString address;      // where the announcement came from (or the address typed by the user)
  int domain = 0;       // ROS domain the car's software runs in
  QString mode, map, path;
  double battery = -1;  // %, -1 when unknown
  int router_port = 0;  // Zenoh router on the car (for networks without multicast)
  bool remote = false;  // connect through the router instead of multicast discovery
  qint64 seen_ms = 0;

  bool valid() const {return !name.isEmpty();}
  QString key() const {return name + "/" + QString::number(domain);}
};

class CarPicker : public QDialog
{
  Q_OBJECT

public:
  explicit CarPicker(QWidget * parent = nullptr);
  CarInfo selected() const {return selected_;}
  // Connects without asking once a car matching name (or host/address) shows up; shows the picker after timeout
  void autoConnect(const QString & name, int timeout_ms);

  // The last car this computer connected to
  static CarInfo lastCar();
  static void rememberCar(const CarInfo & car);

private:
  void readAnnouncements();
  void readReplies();
  void addCar(CarInfo car);
  void refresh();
  void query();
  void connectSelected();

  QUdpSocket * listener_ = nullptr;
  QUdpSocket * prober_ = nullptr;
  QListWidget * list_ = nullptr;
  QLineEdit * address_ = nullptr;
  QPushButton * find_ = nullptr;
  QPushButton * connect_ = nullptr;
  QLabel * status_ = nullptr;
  QTimer * refresh_timer_ = nullptr;
  std::map<QString, CarInfo> cars_;
  CarInfo selected_;
  QString wanted_;   // for autoConnect
  QString query_host_;
};

// Points this process's ROS middleware (Zenoh) at the car: its domain, plus the car's router when it isn't
// reachable by multicast. Must run before rclcpp::init.
void configureMiddleware(const CarInfo & car);
}  // namespace f1ui
