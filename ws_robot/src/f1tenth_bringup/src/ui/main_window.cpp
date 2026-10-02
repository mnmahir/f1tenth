#include "main_window.hpp"

#include <QAbstractSpinBox>
#include <QApplication>
#include <QButtonGroup>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QCloseEvent>
#include <QLineEdit>
#include <QMessageBox>
#include <QMenu>
#include <QNetworkDatagram>
#include <QPainter>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QRegularExpression>
#include <QScreen>
#include <QShortcut>
#include <QStackedWidget>
#include <QStandardPaths>
#include <QTextEdit>
#include <QTimer>
#include <QUdpSocket>
#include <QVBoxLayout>
#include <QWindow>

#include "theme.hpp"
#include "widgets.hpp"

namespace f1ui
{
namespace
{
// "F1TENTH" with speed stripes
class Logo : public QWidget
{
public:
  using QWidget::QWidget;
  QSize sizeHint() const override {return QSize(178, 40);}

protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    for (int i = 0; i < 3; ++i) {
      theme::fillSlanted(p, QRectF(i * 9, 9 + i * 0.0, 13, 22), i == 2 ? theme::red : theme::red.darker(110 + 40 * (2 - i)),
        8);
    }
    QFont f = theme::font(25, QFont::Black, true);
    f.setLetterSpacing(QFont::AbsoluteSpacing, 0.5);
    p.setFont(f);
    p.setPen(theme::text);
    p.drawText(QRectF(36, 0, width() - 36, 30), Qt::AlignLeft | Qt::AlignVCenter, "F1TENTH");
    QFont small = theme::font(9, QFont::Bold);
    small.setLetterSpacing(QFont::AbsoluteSpacing, 3.0);
    p.setFont(small);
    p.setPen(theme::text3);
    p.drawText(QRectF(38, 27, width() - 38, 12), Qt::AlignLeft | Qt::AlignVCenter, "PIT WALL");
  }
};

QIcon appIcon()
{
  QPixmap pm(64, 64);
  pm.fill(Qt::transparent);
  QPainter p(&pm);
  p.setRenderHint(QPainter::Antialiasing);
  theme::fillSlanted(p, QRectF(2, 10, 60, 44), theme::red, 14);
  p.setFont(theme::font(26, QFont::Black, true));
  p.setPen(Qt::white);
  p.drawText(QRectF(2, 10, 60, 44), Qt::AlignCenter, "F1");
  return QIcon(pm);
}

bool typing(QWidget * w)
{
  if (!w) {
    return false;
  }
  if (qobject_cast<QLineEdit *>(w) || qobject_cast<QAbstractSpinBox *>(w) || qobject_cast<QPlainTextEdit *>(w) ||
    qobject_cast<QTextEdit *>(w))
  {
    return true;
  }
  auto * combo = qobject_cast<QComboBox *>(w);
  return combo && combo->isEditable();
}
}  // namespace

MainWindow::MainWindow(RosBridge * ros, const std::string & rviz_node_name, const CarInfo & car)
: ros_(ros), car_(car)
{
  setWindowTitle("F1TENTH Pit Wall: " + car.name);
  setWindowIcon(appIcon());
  setMinimumSize(1280, 760);

  auto * central = new QWidget();
  central->setObjectName("Root");
  auto * root = new QVBoxLayout(central);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);
  root->addWidget(makeTopBar());

  estop_banner_ = new QLabel("EMERGENCY STOP  ·  NOTHING CAN DRIVE THE CAR  ·  RELEASE WITH THE BUTTON WHEN SAFE");
  QFont banner = theme::font(15, QFont::Black, true);
  banner.setLetterSpacing(QFont::AbsoluteSpacing, 1.5);
  estop_banner_->setFont(banner);
  estop_banner_->setAlignment(Qt::AlignCenter);
  estop_banner_->setFixedHeight(34);
  estop_banner_->setStyleSheet(QString("background: %1; color: white;").arg(theme::css(theme::red)));
  estop_banner_->hide();
  root->addWidget(estop_banner_);

  auto * body = new QHBoxLayout();
  body->setSpacing(0);
  auto * rail = new QWidget();
  rail->setObjectName("Rail");
  rail->setAttribute(Qt::WA_StyledBackground);
  rail->setStyleSheet(QString("#Rail { background: #0d0d13; border-right: 1px solid %1; }").arg(theme::css(theme::bg2)));
  rail->setFixedWidth(84);
  auto * rail_layout = new QVBoxLayout(rail);
  rail_layout->setContentsMargins(0, 8, 0, 8);
  rail_layout->setSpacing(2);
  auto * group = new QButtonGroup(this);
  const std::pair<QChar, const char *> entries[] = {
    {theme::icon::home, "Home"}, {theme::icon::speed, "Drive"}, {theme::icon::garage, "Garage"},
    {theme::icon::insights, "Telemetry"}, {theme::icon::monitor, "System"}, {theme::icon::tune, "Params"},
    {theme::icon::terminal, "Log"}};
  int index = 0;
  for (const auto & [glyph, label] : entries) {
    auto * button = new NavButton(glyph, label);
    group->addButton(button);
    rail_layout->addWidget(button);
    nav_.push_back(button);
    connect(button, &NavButton::clicked, this, [this, index]() {showPage(index);});
    ++index;
  }
  rail_layout->addStretch(1);
  body->addWidget(rail);

  pages_ = new QStackedWidget();
  pages_->setObjectName("Pages");
  home_ = new HomePage(ros_);
  session_ = new SessionPage(ros_, rviz_node_name);
  garage_ = new GaragePage(ros_);
  telemetry_ = new TelemetryPage(ros_);
  system_ = new SystemPage(ros_);
  params_ = new ParamsPage(ros_);
  log_ = new LogPage(ros_);
  for (Page * page : {static_cast<Page *>(home_), static_cast<Page *>(session_), static_cast<Page *>(garage_),
      static_cast<Page *>(telemetry_), static_cast<Page *>(system_), static_cast<Page *>(params_),
      static_cast<Page *>(log_)})
  {
    pages_->addWidget(page);
    connect(page, &Page::message, this, &MainWindow::notify);
  }
  body->addWidget(pages_, 1);
  root->addLayout(body, 1);

  auto * status = new QWidget();
  status->setObjectName("StatusBar");
  status->setAttribute(Qt::WA_StyledBackground);
  status->setStyleSheet(QString("#StatusBar { background: #0d0d13; border-top: 1px solid %1; }").arg(theme::css(theme::bg2)));
  status->setFixedHeight(30);
  auto * status_layout = new QHBoxLayout(status);
  status_layout->setContentsMargins(14, 0, 14, 0);
  ticker_ = makeLabel("Connecting to the car...", 12, QFont::DemiBold, theme::text3);
  ticker_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  clock_ = makeLabel("", 12, QFont::Bold, theme::text3);
  status_layout->addWidget(ticker_, 1);
  auto * node_name = makeLabel(QString::fromStdString(ros_->node()->get_name()), 11, QFont::Normal, theme::text3);
  node_name->setToolTip("This UI's ROS node");
  status_layout->addWidget(node_name);
  status_layout->addSpacing(14);
  auto * credit = new QLabel("PIT WALL  ·  BY MAHIR SEHMI");
  QFont credit_font = theme::font(10, QFont::Bold);
  credit_font.setLetterSpacing(QFont::AbsoluteSpacing, 1.2);
  credit->setFont(credit_font);
  credit->setStyleSheet(QString("color: %1;").arg(theme::css(QColor(92, 92, 108))));
  status_layout->addWidget(credit);
  status_layout->addSpacing(14);
  status_layout->addWidget(clock_);
  root->addWidget(status);
  setCentralWidget(central);

  ticker_timer_ = new QTimer(this);
  ticker_timer_->setSingleShot(true);
  connect(ticker_timer_, &QTimer::timeout, this, [this]() {
      ticker_->setStyleSheet(QString("color: %1;").arg(theme::css(theme::text3)));
    });
  auto * second = new QTimer(this);
  connect(second, &QTimer::timeout, this, [this]() {
      clock_->setText(QDateTime::currentDateTime().toString("HH:mm:ss"));
    });
  second->start(1000);
  estop_flash_ = new QTimer(this);
  connect(estop_flash_, &QTimer::timeout, this, [this]() {
      estop_phase_ = !estop_phase_;
      estop_banner_->setStyleSheet(QString("background: %1; color: white;")
        .arg(theme::css(estop_phase_ ? theme::red : QColor(120, 4, 0))));
    });

  connect(home_, &HomePage::sessionStarted, this, [this]() {showPage(PageSession);});
  connect(session_, &SessionPage::titleChanged, this, [this](const QString & title) {nav_[PageSession]->setText(title);});
  connect(session_, &SessionPage::openParameters, this, [this](const QString & node) {
      params_->showNode(node);
      showPage(PageParams);
    });
  connect(session_, &SessionPage::openGarage, this, [this]() {showPage(PageGarage);});
  connect(session_, &SessionPage::openHome, this, [this]() {showPage(PageHome);});
  connect(garage_, &GaragePage::useMap, this, [this](const QString & map) {
      home_->selectMap(map);
      showPage(PageHome);
    });
  connect(garage_, &GaragePage::usePath, this, [this](const QString & path) {
      home_->selectPath(path);
      showPage(PageHome);
    });
  connect(system_, &SystemPage::healthChanged, this, [this](int level) {
      nav_[PageSystem]->setBadge(level == 0 ? QColor() : theme::levelColor(level));
    });
  connect(ros_, &RosBridge::telemetry, this, &MainWindow::onTelemetry);
  connect(ros_, &RosBridge::supervisorState, this, &MainWindow::onSupervisor);
  connect(ros_, &RosBridge::driveState, this, &MainWindow::onDrive);
  connect(ros_, &RosBridge::linkChanged, this, &MainWindow::onLink);
  watchCarAddress();
  connect(ros_, &RosBridge::estopChanged, this, &MainWindow::setEstopShown);
  connect(ros_, &RosBridge::rosout, this, [this](const std::vector<Log> & batch) {
      for (const auto & log : batch) {
        if (log.level >= Log::ERROR) {
          notify(Level::Error, QString("%1: %2").arg(QString::fromStdString(log.name))
            .arg(QString::fromStdString(log.msg).remove(QRegularExpression("\x1b\\[[0-9;]*m"))));
        }
      }
    });

  for (int i = 0; i < static_cast<int>(nav_.size()); ++i) {
    auto * shortcut = new QShortcut(QKeySequence(Qt::CTRL | (Qt::Key_1 + i)), this);
    connect(shortcut, &QShortcut::activated, this, [this, i]() {showPage(i);});
  }
  connect(new QShortcut(QKeySequence(Qt::Key_F11), this), &QShortcut::activated, this, [this]() {
      isFullScreen() ? showNormal() : showFullScreen();
    });
  connect(new QShortcut(QKeySequence(Qt::Key_F12), this), &QShortcut::activated, this, [this]() {screenshot();});
  qApp->installEventFilter(this);

  // The 3D scene initializes on the session page, then HOME is shown
  pages_->setCurrentIndex(PageSession);
  nav_[PageHome]->setChecked(true);
}

QWidget * MainWindow::makeTopBar()
{
  auto * bar = new QWidget();
  bar->setObjectName("TopBar");
  bar->setAttribute(Qt::WA_StyledBackground);
  bar->setStyleSheet(QString("#TopBar { background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #17171f, "
    "stop:1 #0b0b10); border-bottom: 2px solid %1; }").arg(theme::css(theme::red)));
  bar->setFixedHeight(60);
  auto * layout = new QHBoxLayout(bar);
  layout->setContentsMargins(14, 6, 14, 6);
  layout->setSpacing(14);
  layout->addWidget(new Logo());
  layout->addSpacing(4);
  // Which car this pit wall is talking to; click to switch
  auto * car_button = new QPushButton(QString("  %1").arg(car_.name.toUpper()));
  car_button->setObjectName("Car");
  car_button->setIcon(glyphIcon(theme::icon::car, theme::text, 18));
  car_button->setIconSize(QSize(18, 18));
  car_button->setCursor(Qt::PointingHandCursor);
  QFont car_font = theme::font(15, QFont::Black, true);
  car_font.setLetterSpacing(QFont::AbsoluteSpacing, 1.0);
  car_button->setFont(car_font);
  car_button->setToolTip(QString("Connected to %1 (%2, ROS domain %3%4). Click to switch car.")
    .arg(car_.name, car_.address).arg(car_.domain).arg(car_.remote ? ", through its router" : ""));
  car_button->setStyleSheet(QString("QPushButton#Car { background: transparent; border: 1px solid %1; border-radius: 4px;"
    " padding: 4px 12px; color: white; } QPushButton#Car:hover { background: %2; }")
    .arg(theme::css(theme::line), theme::css(theme::bg2)));
  auto * car_menu = new QMenu(car_button);
  car_menu->addAction(QString("%1  ·  ROS domain %2").arg(car_.address).arg(car_.domain))->setEnabled(false);
  car_menu->addSeparator();
  connect(car_menu->addAction("Switch car..."), &QAction::triggered, this, &MainWindow::switchCar);
  car_button->setMenu(car_menu);
  layout->addWidget(car_button);
  layout->addSpacing(4);
  mode_badge_ = new SkewBadge();
  mode_badge_->setPixelSize(14);
  mode_badge_->set("OFFLINE", theme::bg3);
  layout->addWidget(mode_badge_);
  track_ = makeLabel("", 13, QFont::DemiBold, theme::text2);
  layout->addWidget(track_);
  driver_badge_ = new SkewBadge();
  driver_badge_->setPixelSize(13);
  driver_badge_->hide();
  layout->addWidget(driver_badge_);
  layout->addStretch(1);
  battery_ = new QLabel();
  battery_->setTextFormat(Qt::RichText);
  layout->addWidget(battery_);
  link_led_ = new StatusLed(nullptr, 10);
  link_led_->setColor(theme::redBright);
  link_ = makeLabel("NO LINK", 12, QFont::Bold, theme::text2);
  auto * link_row = new QHBoxLayout();
  link_row->setSpacing(4);
  link_row->addWidget(link_led_);
  link_row->addWidget(link_);
  layout->addLayout(link_row);
  layout->addSpacing(6);
  estop_ = new QPushButton("  E-STOP");
  estop_->setObjectName("Estop");
  estop_->setIcon(glyphIcon(theme::icon::emergency, Qt::white, 22, Qt::black));
  estop_->setIconSize(QSize(22, 22));
  estop_->setCursor(Qt::PointingHandCursor);
  estop_->setMinimumSize(170, 44);
  estop_->setToolTip("Emergency stop: nothing can drive the car until released (Space engages it)");
  estop_->setStyleSheet(QString(
    "QPushButton#Estop { background: %1; border: 2px solid #ff5a4f; border-radius: 6px; color: white;"
    " font-size: 17px; font-weight: 900; font-style: italic; padding: 4px 16px; }"
    "QPushButton#Estop:hover { background: %2; }"
    "QPushButton#Estop[engaged=\"true\"] { background: %3; border-color: #fff3b0; color: black; }")
    .arg(theme::css(theme::red), theme::css(theme::redBright), theme::css(theme::yellow)));
  connect(estop_, &QPushButton::clicked, this, &MainWindow::toggleEstop);
  layout->addWidget(estop_);
  return bar;
}

void MainWindow::showEvent(QShowEvent * event)
{
  QMainWindow::showEvent(event);
  if (scene_ready_) {
    return;
  }
  scene_ready_ = true;
  QTimer::singleShot(0, this, [this]() {
      session_->initializeScene();
      showPage(PageHome);
      QString tour = qEnvironmentVariable("F1TENTH_UI_TOUR");
      if (!tour.isEmpty()) {
        runTour(tour);
      }
    });
}

// Zenoh connects to the car's address chosen at start, so when the car moves to another address (another Wi-Fi
// network, a hotspot) the link is gone for good. Its announcements say where it is now: once the link has been down
// a few seconds and the same car is heard elsewhere, start again on the new address.
void MainWindow::watchCarAddress()
{
  if (QHostAddress(car_.address).isNull()) {
    return;  // the UI runs on the car itself
  }
  beacon_ = new QUdpSocket(this);
  beacon_->bind(QHostAddress::AnyIPv4, CarPicker::announcePort(), QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
  connect(beacon_, &QUdpSocket::readyRead, this, [this]() {
      while (beacon_->hasPendingDatagrams()) {
        QNetworkDatagram datagram = beacon_->receiveDatagram();
        CarInfo car;
        if (CarPicker::parseAnnouncement(datagram.data(), datagram.senderAddress(), car) && car.name == car_.name &&
          car.domain == car_.domain && car.address != car_.address)
        {
          moved_ = car;
        }
      }
    });
  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, [this]() {
      qint64 now = QDateTime::currentMSecsSinceEpoch();
      if (ros_->supervisorLinked()) {
        link_lost_ms_ = 0;
        return;
      }
      if (link_lost_ms_ == 0) {
        link_lost_ms_ = now;
      }
      if (!reconnecting_ && now - link_lost_ms_ > 4000 && moved_.valid() && now - moved_.seen_ms < 3000) {
        reconnecting_ = true;
        notify(Level::Warn, QString("%1 is now at %2: reconnecting...").arg(car_.name.toUpper(), moved_.address));
        QTimer::singleShot(1500, this, [this]() {
          QProcess::startDetached(QCoreApplication::applicationFilePath(), {"--car", moved_.address});
          closing_ = true;
          close();
        });
      }
    });
  timer->start(1000);
}

void MainWindow::switchCar()
{
  // A fresh process: the ROS domain is chosen before ROS starts. This car keeps running as it is.
  QProcess::startDetached(QCoreApplication::applicationFilePath(), {"--pick"});
  closing_ = true;
  close();
}

void MainWindow::closeEvent(QCloseEvent * event)
{
  if (closing_ || !ros_->supervisorLinked() || mode_.isEmpty() || mode_ == "idle") {
    event->accept();
    return;
  }
  QMessageBox box(this);
  box.setWindowTitle("Leave the pit wall?");
  box.setText(QString("A %1 session is running on the car.").arg(theme::modeTitle(mode_).toLower()));
  box.setInformativeText("Stop it (standby), or keep it running and reconnect later?");
  auto * stop = box.addButton("STOP THE CAR", QMessageBox::AcceptRole);
  box.addButton("KEEP RUNNING", QMessageBox::DestructiveRole);
  auto * cancel = box.addButton(QMessageBox::Cancel);
  box.setDefaultButton(stop);
  box.exec();
  if (box.clickedButton() == cancel) {
    event->ignore();
    return;
  }
  if (box.clickedButton() == stop) {
    event->ignore();
    ros_->setMode("idle", "", "", [this](bool, const QString &) {
        closing_ = true;
        close();
      });
    return;
  }
  event->accept();
}

bool MainWindow::eventFilter(QObject * watched, QEvent * event)
{
  // Space engages the E-stop from anywhere except text fields (releasing needs the button)
  if (event->type() == QEvent::KeyPress) {
    auto * key = static_cast<QKeyEvent *>(event);
    if (key->key() == Qt::Key_Space && !key->isAutoRepeat() && !typing(QApplication::focusWidget()) &&
      isActiveWindow())
    {
      if (!estop_engaged_) {
        toggleEstop();
      }
      return true;
    }
  }
  return QMainWindow::eventFilter(watched, event);
}

void MainWindow::showPage(int index)
{
  pages_->setCurrentIndex(index);
  if (index >= 0 && index < static_cast<int>(nav_.size())) {
    nav_[index]->setChecked(true);
  }
  session_->setActive(index == PageSession);
  garage_->setActive(index == PageGarage);
}

void MainWindow::notify(Level level, const QString & text)
{
  QColor color = level == Level::Error ? theme::redBright : (level == Level::Warn ? theme::yellow :
    (level == Level::Ok ? theme::green : theme::text));
  QChar glyph = level == Level::Error ? theme::icon::error : (level == Level::Warn ? theme::icon::warning :
    (level == Level::Ok ? theme::icon::check : theme::icon::info));
  ticker_->setText(QString("<span style=\"font-family:'Material Icons'; font-size:15px;\">%1</span>&nbsp; %2")
    .arg(QString(glyph), text.toHtmlEscaped()));
  ticker_->setToolTip(text);
  ticker_->setStyleSheet(QString("color: %1;").arg(theme::css(color)));
  ticker_timer_->start(level == Level::Error ? 15000 : 8000);
}

void MainWindow::onTelemetry(const Telemetry & t)
{
  if (t.vesc_connected) {
    QColor color = t.battery_percent > 50 ? theme::green : (t.battery_percent > 20 ? theme::yellow : theme::redBright);
    battery_->setText(QString("<span style=\"font-family:'Material Icons'; font-size:20px; color:%1;\">%2</span>"
      "<span style=\"font-size:16px; font-weight:700; color:%3;\"> %4%</span>"
      "<span style=\"font-size:12px; color:%5;\">  %6 V</span>")
      .arg(theme::css(color), QString(theme::icon::battery), theme::css(theme::text))
      .arg(t.battery_percent, 0, 'f', 0).arg(theme::css(theme::text2)).arg(t.battery_voltage, 0, 'f', 1));
    if (t.battery_percent < 15 && !low_battery_warned_) {
      low_battery_warned_ = true;
      notify(Level::Warn, QString("Battery low (%1%). Charge it soon: LiPos are damaged below ~3.3 V per cell.")
        .arg(t.battery_percent, 0, 'f', 0));
    } else if (t.battery_percent > 25) {
      low_battery_warned_ = false;
    }
  } else {
    battery_->setText(QString("<span style=\"color:%1;\">VESC OFFLINE</span>").arg(theme::css(theme::text3)));
  }
  bool engaged = t.estop || ros_->estopHeldHere();
  if (engaged != estop_engaged_) {
    setEstopShown(engaged, ros_->estopHeldHere());
  }
  link_->setText(QString("LINK %1 HZ").arg(ros_->telemetryRate(), 0, 'f', 0));
}

void MainWindow::onSupervisor(const SupervisorState & state)
{
  static bool first = true;
  QString mode = QString::fromStdString(state.mode);
  mode_ = mode;
  mode_badge_->set(theme::modeTitle(mode), theme::modeColor(mode),
    mode == "mapping" ? QColor(Qt::black) : QColor(Qt::white));
  QStringList track;
  if (!state.map.empty()) {
    track << QString::fromStdString(state.map);
  }
  if (!state.path.empty()) {
    track << QString::fromStdString(state.path);
  }
  track_->setText(track.join("  ·  "));
  QString message = QString::fromStdString(state.message);
  if (message != last_supervisor_message_) {
    if (!last_supervisor_message_.isEmpty() || first) {
      notify(Level::Info, message);
    }
    last_supervisor_message_ = message;
  }
  if (first) {
    first = false;
    if (mode != "idle" && pages_->currentIndex() == PageHome) {
      showPage(PageSession);  // a session is already running: go straight to it
    }
  }
}

void MainWindow::onDrive(const DriveState & d)
{
  switch (d.source) {
    case DriveState::AUTONOMOUS: driver_badge_->set("AUTONOMOUS", theme::green, Qt::black); break;
    case DriveState::ASSIST: driver_badge_->set("ASSISTED", theme::purple); break;
    case DriveState::MANUAL: driver_badge_->set("MANUAL", theme::bg3); break;
    default:
      driver_badge_->set(d.autonomous_enabled ? "AUTO ARMED" : "NO DRIVER", d.autonomous_enabled ? theme::green :
        theme::bg2, d.autonomous_enabled ? QColor(Qt::black) : theme::text3);
  }
  driver_badge_->show();
}

void MainWindow::onLink(bool car, bool supervisor)
{
  link_led_->setColor(car ? theme::green : (supervisor ? theme::yellow : theme::redBright));
  link_led_->setBlinking(!car);
  if (!car) {
    link_->setText(supervisor ? (mode_ == "idle" ? "STANDBY" : "NO TELEMETRY") : "NO LINK");
    driver_badge_->hide();
    if (!supervisor) {
      mode_badge_->set("OFFLINE", theme::bg3);
    }
    if (supervisor && (mode_.isEmpty() || mode_ == "idle")) {
      notify(Level::Info, "Car on standby: start a session on HOME");
    } else {
      notify(Level::Warn, supervisor ? "No telemetry from the car (is the core running?)" : "Lost the link to the car");
    }
  } else {
    notify(Level::Ok, "Connected to the car");
  }
}

void MainWindow::toggleEstop()
{
  if (estop_engaged_) {
    ros_->setEstop(false);
    notify(Level::Ok, "E-stop released");
  } else {
    ros_->setEstop(true);
    notify(Level::Error, "E-STOP ENGAGED: the car is stopped");
  }
}

void MainWindow::setEstopShown(bool engaged, bool)
{
  estop_engaged_ = engaged || ros_->estopHeldHere();
  estop_->setText(estop_engaged_ ? "  RELEASE" : "  E-STOP");
  estop_->setProperty("engaged", estop_engaged_);
  repolish(estop_);
  estop_banner_->setVisible(estop_engaged_);
  if (estop_engaged_) {
    estop_flash_->start(400);
  } else {
    estop_flash_->stop();
  }
}

void MainWindow::screenshot(const QString & path)
{
  QString file = path;
  if (file.isEmpty()) {
    QString dir = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
    QDir().mkpath(dir);
    file = dir + "/f1tenth_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".png";
  }
  QScreen * screen = windowHandle() ? windowHandle()->screen() : QGuiApplication::primaryScreen();
  QPixmap shot = screen->grabWindow(winId());
  if (shot.save(file)) {
    notify(Level::Ok, "Screenshot saved: " + file);
  } else {
    notify(Level::Error, "Could not save " + file);
  }
}

void MainWindow::runTour(const QString & dir)
{
  // Development aid: visit every page and save a screenshot of each, then quit
  QDir().mkpath(dir);
  struct Step
  {
    int page;
    int view;  // -1: leave the 3D view as it is
    const char * name;
  };
  static const Step steps[] = {
    {PageHome, -1, "01_home"}, {PageSession, 1, "02_drive_chase"}, {PageSession, 0, "03_drive_bird"},
    {PageSession, 2, "04_drive_onboard"}, {PageSession, 3, "05_drive_orbit"}, {PageGarage, -1, "06_garage"},
    {PageTelemetry, -1, "07_telemetry"}, {PageSystem, -1, "08_system"}, {PageParams, -1, "09_params"}, {PageLog, -1, "10_log"}};
  int delay = 7000;
  for (const auto & step : steps) {
    QTimer::singleShot(delay, this, [this, step]() {
        showPage(step.page);
        if (step.view >= 0) {
          session_->scene()->setView(static_cast<SceneView::View>(step.view));
        }
        if (step.page == PageParams) {
          params_->showNode("/autonomous_safety_brake");
        }
      });
    QTimer::singleShot(delay + 3500, this, [this, dir, step]() {
        screenshot(dir + "/" + step.name + ".png");
      });
    delay += 4500;
  }
  QTimer::singleShot(delay + 500, qApp, &QApplication::quit);
}
}  // namespace f1ui
