#include "car_picker.hpp"

#include <QDateTime>
#include <QHBoxLayout>
#include <QHostInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QNetworkDatagram>
#include <QNetworkInterface>
#include <QPainter>
#include <QPushButton>
#include <QSettings>
#include <QTimer>
#include <QUdpSocket>
#include <QVBoxLayout>

#include "theme.hpp"
#include "widgets.hpp"

namespace f1ui
{
namespace
{
constexpr quint16 kQueryPort = 47820;     // the supervisor answers "F1TENTH?" here
constexpr quint16 kAnnouncePort = 47821;  // and broadcasts its announcement here every second

bool parse(const QByteArray & data, const QHostAddress & from, CarInfo & car)
{
  auto doc = QJsonDocument::fromJson(data);
  if (!doc.isObject() || doc.object().value("f1tenth").toInt() != 1) {
    return false;
  }
  auto o = doc.object();
  car.name = o.value("name").toString();
  car.host = o.value("host").toString();
  car.domain = o.value("domain").toInt();
  car.mode = o.value("mode").toString();
  car.map = o.value("map").toString();
  car.path = o.value("path").toString();
  car.battery = o.value("battery").toDouble(-1);
  car.router_port = o.value("router").toInt();
  bool ok = false;
  quint32 v4 = from.toIPv4Address(&ok);
  car.address = ok ? QHostAddress(v4).toString() : from.toString();
  car.seen_ms = QDateTime::currentMSecsSinceEpoch();
  return car.valid();
}

// Multicast discovery only works inside this computer's own networks; elsewhere the car's router is used
bool onLocalNetwork(const QString & address)
{
  QHostAddress host(address);
  for (const auto & iface : QNetworkInterface::allInterfaces()) {
    for (const auto & entry : iface.addressEntries()) {
      if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol && entry.prefixLength() > 0 &&
        host.isInSubnet(entry.ip(), entry.prefixLength()))
      {
        return true;
      }
    }
  }
  return false;
}

// One car in the list
class CarCard : public QWidget
{
public:
  explicit CarCard(const CarInfo & car)
  : car_(car)
  {
    setMinimumHeight(78);
    setAttribute(Qt::WA_TransparentForMouseEvents);
  }
  void setCar(const CarInfo & car)
  {
    car_ = car;
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    qint64 age = (QDateTime::currentMSecsSinceEpoch() - car_.seen_ms) / 1000;
    bool stale = age > 5;
    QColor mode = theme::modeColor(car_.mode);
    p.fillRect(QRectF(0, 10, 4, height() - 20), stale ? theme::text3 : mode);
    QFont name = theme::font(22, QFont::Black, true);
    name.setLetterSpacing(QFont::AbsoluteSpacing, 1.0);
    p.setFont(name);
    p.setPen(stale ? theme::text3 : theme::text);
    p.drawText(QRectF(18, 8, width() * 0.6, 32), Qt::AlignLeft | Qt::AlignVCenter, car_.name.toUpper());
    p.setFont(theme::font(12));
    p.setPen(theme::text2);
    QStringList detail = {car_.host, car_.address, QString("ROS domain %1").arg(car_.domain)};
    if (car_.remote) {
      detail << "via its router";
    }
    if (stale) {
      detail << QString("not seen for %1 s").arg(age);
    }
    p.drawText(QRectF(18, 40, width() - 200, 18), Qt::AlignLeft | Qt::AlignVCenter, detail.join("  ·  "));
    QString track = car_.map.isEmpty() ? QString() : car_.map + (car_.path.isEmpty() ? "" : "  ·  " + car_.path);
    if (!track.isEmpty()) {
      p.setPen(theme::text3);
      p.drawText(QRectF(18, 58, width() - 200, 16), Qt::AlignLeft | Qt::AlignVCenter, track);
    }
    // Mode badge and battery on the right
    QFont badge = theme::font(12, QFont::Black, true);
    badge.setLetterSpacing(QFont::AbsoluteSpacing, 1.0);
    p.setFont(badge);
    QString mode_text = theme::modeTitle(car_.mode);
    double w = p.fontMetrics().horizontalAdvance(mode_text) + 30;
    QRectF box(width() - w - 12, 12, w, 24);
    theme::fillSlanted(p, box, stale ? theme::bg3 : mode, 8);
    p.setPen(car_.mode == "mapping" ? QColor(Qt::black) : theme::text);
    p.drawText(box, Qt::AlignCenter, mode_text);
    if (car_.battery >= 0) {
      QColor level = car_.battery > 50 ? theme::green : (car_.battery > 20 ? theme::yellow : theme::redBright);
      p.setFont(theme::font(16, QFont::Bold));
      p.setPen(level);
      p.drawText(QRectF(width() - 140, 42, 128, 24), Qt::AlignRight | Qt::AlignVCenter,
        QString("%1%").arg(car_.battery, 0, 'f', 0));
      p.setFont(theme::iconFont(18));
      p.drawText(QRectF(width() - 190, 42, 50, 24), Qt::AlignRight | Qt::AlignVCenter, QString(theme::icon::battery));
    }
  }

private:
  CarInfo car_;
};
}  // namespace

CarPicker::CarPicker(QWidget * parent)
: QDialog(parent)
{
  setWindowTitle("F1TENTH Pit Wall: choose a car");
  setMinimumSize(760, 560);
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(28, 24, 28, 22);
  root->setSpacing(12);
  auto * title = new QLabel("CHOOSE YOUR CAR");
  QFont title_font = theme::font(30, QFont::Black, true);
  title_font.setLetterSpacing(QFont::AbsoluteSpacing, 2.0);
  title->setFont(title_font);
  root->addWidget(title);
  auto * subtitle = makeLabel("Cars on this network. Each car runs in its own ROS domain: the pit wall talks only to "
    "the car you pick, never to its neighbours.", 13, QFont::Normal, theme::text2);
  subtitle->setWordWrap(true);
  root->addWidget(subtitle);
  list_ = new QListWidget();
  list_->setSpacing(4);
  root->addWidget(list_, 1);
  status_ = makeLabel("Looking for cars...", 12, QFont::DemiBold, theme::text3);
  root->addWidget(status_);

  auto * address_row = new QHBoxLayout();
  address_row->addWidget(makeCaption("Not listed?"));
  address_ = new QLineEdit();
  address_->setPlaceholderText("Car's address, e.g. 192.168.1.20 or f1.my-vpn.net (other networks, VPNs)");
  find_ = makeButton("FIND", theme::icon::search);
  address_row->addWidget(address_, 1);
  address_row->addWidget(find_);
  root->addLayout(address_row);

  auto * buttons = new QHBoxLayout();
  auto * quit = makeButton("QUIT", QChar(), "ghost");
  connect_ = makeButton("CONNECT", theme::icon::link, "primary");
  connect_->setMinimumSize(200, 44);
  connect_->setFont(theme::font(15, QFont::Black, true));
  connect_->setEnabled(false);
  connect_->setDefault(true);
  buttons->addWidget(quit);
  buttons->addStretch(1);
  buttons->addWidget(makeLabel("PIT WALL  ·  BY MAHIR SEHMI", 10, QFont::Bold, QColor(92, 92, 108)));
  buttons->addSpacing(16);
  buttons->addWidget(connect_);
  root->addLayout(buttons);

  listener_ = new QUdpSocket(this);
  if (!listener_->bind(QHostAddress::AnyIPv4, kAnnouncePort, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
    status_->setText("Can't listen for car announcements (UDP 47821 busy); use the address field.");
  }
  connect(listener_, &QUdpSocket::readyRead, this, &CarPicker::readAnnouncements);
  prober_ = new QUdpSocket(this);
  prober_->bind(QHostAddress(QHostAddress::AnyIPv4), static_cast<quint16>(0));
  connect(prober_, &QUdpSocket::readyRead, this, &CarPicker::readReplies);

  connect(quit, &QPushButton::clicked, this, &QDialog::reject);
  connect(connect_, &QPushButton::clicked, this, &CarPicker::connectSelected);
  connect(list_, &QListWidget::itemDoubleClicked, this, &CarPicker::connectSelected);
  connect(list_, &QListWidget::currentRowChanged, this, [this](int row) {connect_->setEnabled(row >= 0);});
  connect(find_, &QPushButton::clicked, this, [this]() {
      query_host_ = address_->text().trimmed();
      query();
    });
  connect(address_, &QLineEdit::returnPressed, find_, &QPushButton::click);

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, [this]() {
      if (!query_host_.isEmpty()) {
        query();  // remote cars don't reach us by broadcast: keep asking
      }
      refresh();
    });
  refresh_timer_->start(1500);

  // Development aid (see MainWindow::runTour): screenshot the picker, then take the first car
  QString tour = qEnvironmentVariable("F1TENTH_UI_TOUR");
  if (!tour.isEmpty()) {
    QTimer::singleShot(3000, this, [this, tour]() {
        grab().save(tour + "/00_picker.png");
        if (list_->count() > 0) {
          list_->setCurrentRow(0);
          connectSelected();
        }
      });
  }

  // The last car, if it's elsewhere, won't be heard: ask it directly
  CarInfo last = lastCar();
  if (last.valid() && last.remote) {
    query_host_ = last.address;
    address_->setText(last.address);
    query();
  }
}

void CarPicker::autoConnect(const QString & name, int timeout_ms)
{
  wanted_ = name;
  QTimer::singleShot(timeout_ms, this, [this]() {
      if (!wanted_.isEmpty() && result() != QDialog::Accepted) {
        status_->setText("Couldn't find '" + wanted_ + "'. Pick a car:");
        wanted_.clear();
      }
    });
  // An address can be asked directly
  if (!QHostAddress(name).isNull() || name.contains('.')) {
    query_host_ = name;
    query();
  }
}

bool CarPicker::parseAnnouncement(const QByteArray & data, const QHostAddress & from, CarInfo & car)
{
  return parse(data, from, car);
}

quint16 CarPicker::announcePort()
{
  return kAnnouncePort;
}

void CarPicker::readAnnouncements()
{
  while (listener_->hasPendingDatagrams()) {
    QNetworkDatagram datagram = listener_->receiveDatagram();
    CarInfo car;
    if (parse(datagram.data(), datagram.senderAddress(), car)) {
      car.remote = false;
      addCar(car);
    }
  }
}

void CarPicker::readReplies()
{
  while (prober_->hasPendingDatagrams()) {
    QNetworkDatagram datagram = prober_->receiveDatagram();
    CarInfo car;
    if (parse(datagram.data(), datagram.senderAddress(), car)) {
      car.remote = !onLocalNetwork(car.address) && car.router_port > 0;
      if (!wanted_.isEmpty() && query_host_ == wanted_) {
        selected_ = car;  // the address asked for on the command line answered
        accept();
        return;
      }
      addCar(car);
    }
  }
}

void CarPicker::query()
{
  QString host = query_host_;
  if (host.isEmpty()) {
    return;
  }
  QHostInfo::lookupHost(host, this, [this, host](const QHostInfo & info) {
      if (info.error() != QHostInfo::NoError) {
        status_->setText("Can't find the address " + host + ": " + info.errorString());
        return;
      }
      for (const auto & address : info.addresses()) {
        if (address.protocol() == QAbstractSocket::IPv4Protocol) {
          prober_->writeDatagram("F1TENTH?", address, kQueryPort);
        }
      }
    });
}

void CarPicker::addCar(CarInfo car)
{
  // Keep the remote flag of a car first found by address
  auto it = cars_.find(car.key());
  if (it != cars_.end() && it->second.remote && !car.remote && !onLocalNetwork(car.address)) {
    car.remote = true;
  }
  bool is_new = it == cars_.end();
  cars_[car.key()] = car;
  if (!wanted_.isEmpty() && (car.name == wanted_ || car.host == wanted_ || car.address == wanted_)) {
    selected_ = car;
    accept();
    return;
  }
  if (is_new) {
    refresh();
  }
}

void CarPicker::refresh()
{
  // Rebuild only when cars come or go; otherwise update the cards in place (keeps the selection steady)
  QStringList keys, shown;
  for (const auto & [key, car] : cars_) {
    keys << key;
  }
  for (int i = 0; i < list_->count(); ++i) {
    shown << list_->item(i)->data(Qt::UserRole).toString();
  }
  if (keys != shown) {
    QString current = list_->currentItem() ? list_->currentItem()->data(Qt::UserRole).toString() : lastCar().key();
    list_->clear();
    for (const auto & [key, car] : cars_) {
      auto * item = new QListWidgetItem();
      item->setData(Qt::UserRole, key);
      item->setSizeHint(QSize(100, 84));
      list_->addItem(item);
      list_->setItemWidget(item, new CarCard(car));
    }
    int select = keys.indexOf(current);
    if (select < 0 && list_->count() == 1) {
      select = 0;
    }
    if (select >= 0) {
      list_->setCurrentRow(select);
    }
  } else {
    for (int i = 0; i < list_->count(); ++i) {
      if (auto * card = dynamic_cast<CarCard *>(list_->itemWidget(list_->item(i)))) {
        card->setCar(cars_[keys[i]]);
      }
    }
  }
  if (!cars_.empty()) {
    status_->setText(QString("%1 car%2 found. Double-click one, or select it and press CONNECT.")
      .arg(cars_.size()).arg(cars_.size() == 1 ? "" : "s"));
  } else {
    status_->setText("Looking for cars... Check the car is on and on this network. On another network (e.g. a VPN), "
      "type its address below.");
  }
}

void CarPicker::connectSelected()
{
  auto * item = list_->currentItem();
  if (!item) {
    return;
  }
  auto it = cars_.find(item->data(Qt::UserRole).toString());
  if (it == cars_.end()) {
    return;
  }
  selected_ = it->second;
  accept();
}

CarInfo CarPicker::lastCar()
{
  QSettings settings("f1tenth", "pit-wall");
  CarInfo car;
  car.name = settings.value("car/name").toString();
  car.host = settings.value("car/host").toString();
  car.address = settings.value("car/address").toString();
  car.domain = settings.value("car/domain").toInt();
  car.remote = settings.value("car/remote").toBool();
  car.router_port = settings.value("car/router", 7447).toInt();
  return car;
}

void CarPicker::rememberCar(const CarInfo & car)
{
  QSettings settings("f1tenth", "pit-wall");
  settings.setValue("car/name", car.name);
  settings.setValue("car/host", car.host);
  settings.setValue("car/address", car.address);
  settings.setValue("car/domain", car.domain);
  settings.setValue("car/remote", car.remote);
  settings.setValue("car/router", car.router_port);
}

void configureMiddleware(const CarInfo & car)
{
  // The car runs rmw_zenoh_cpp without a router: peers find each other by multicast
  if (qEnvironmentVariableIsEmpty("RMW_IMPLEMENTATION")) {
    qputenv("RMW_IMPLEMENTATION", "rmw_zenoh_cpp");
  }
  if (qEnvironmentVariableIsEmpty("ZENOH_ROUTER_CHECK_ATTEMPTS")) {
    qputenv("ZENOH_ROUTER_CHECK_ATTEMPTS", "-1");
  }
  // Remember the user's own Zenoh settings, so switching cars starts from them again
  if (!qEnvironmentVariableIsSet("F1TENTH_USER_ZENOH_OVERRIDE")) {
    qputenv("F1TENTH_USER_ZENOH_OVERRIDE", qgetenv("ZENOH_CONFIG_OVERRIDE"));
  }
  QByteArray zenoh = qgetenv("F1TENTH_USER_ZENOH_OVERRIDE");
  if (car.router_port > 0 && !QHostAddress(car.address).isNull()) {
    // A client of the Zenoh router the car runs: one TCP connection to the car carries everything, on whatever
    // network the car is reached through and past this computer's firewall. As a peer, this computer would
    // advertise all its addresses for the car's nodes to connect back to, and if the first ones are unreachable
    // from the car (this computer also on another network) no data arrives; multicast discovery uses one
    // interface, which may not be the car's.
    QString host = car.address.contains(':') ? "[" + car.address + "]" : car.address;
    zenoh = QString("mode=\"client\";scouting/multicast/enabled=false;connect/endpoints=[\"tcp/%1:%2\"]")
      .arg(host).arg(car.router_port).toUtf8();
  } else if (zenoh.isEmpty()) {
    zenoh = "listen/endpoints=[\"tcp/0.0.0.0:0\"];scouting/multicast/enabled=true";
  }
  qputenv("ZENOH_CONFIG_OVERRIDE", zenoh);
  if (car.valid()) {
    qputenv("ROS_DOMAIN_ID", QByteArray::number(car.domain));
  }
}
}  // namespace f1ui
