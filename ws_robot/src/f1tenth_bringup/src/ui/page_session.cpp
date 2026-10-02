#include <QButtonGroup>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QStackedWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <functional>
#include <cmath>

#include "hud.hpp"
#include "pages.hpp"
#include "theme.hpp"
#include "views.hpp"
#include "widgets.hpp"

namespace f1ui
{
namespace
{
constexpr char kSafetyNode[] = "/autonomous_safety_brake";
constexpr char kAvoidanceNode[] = "/avoidance_controller_node";
constexpr char kFollowerNode[] = "/pure_pursuit";
constexpr char kBoostNode[] = "/teleop_speed_multiplier";

QValidator * nameValidator(QObject * parent)
{
  return new QRegularExpressionValidator(QRegularExpression("[A-Za-z0-9_-]{1,64}"), parent);
}

QString defaultName(const QString & prefix)
{
  return prefix + QDateTime::currentDateTime().toString("_yyyyMMdd_HHmm");
}

QWidget * scrollable(QWidget * content)
{
  auto * area = new QScrollArea();
  area->setWidget(content);
  area->setWidgetResizable(true);
  area->setFrameShape(QFrame::NoFrame);
  area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  area->setStyleSheet("QScrollArea, QScrollArea > QWidget > QWidget { background: transparent; }");
  return area;
}

QHBoxLayout * row(std::initializer_list<QWidget *> widgets, int spacing = 8)
{
  auto * layout = new QHBoxLayout();
  layout->setSpacing(spacing);
  for (auto * w : widgets) {
    layout->addWidget(w);
  }
  return layout;
}
}  // namespace

// ---- HudStrip ----

HudStrip::HudStrip(RosBridge * ros, QWidget * parent)
: QWidget(parent), ros_(ros)
{
  setObjectName("Hud");
  setAttribute(Qt::WA_StyledBackground);
  setStyleSheet(QString("#Hud { background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #13131b, stop:1 #09090d);"
    " border-top: 1px solid %1; }").arg(theme::css(theme::bg3)));
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(16, 6, 16, 8);
  root->setSpacing(4);

  auto * top = new QHBoxLayout();
  flags_ = new FlagStrip();
  lights_ = new ShiftLights();
  lights_->setFixedSize(420, 20);
  top->addWidget(flags_, 1);
  top->addWidget(lights_);
  auto * spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  top->addWidget(spacer, 1);
  root->addLayout(top);

  auto * instruments = new QHBoxLayout();
  instruments->setSpacing(18);
  pedals_ = new PedalBars();
  pedals_->setFixedWidth(90);
  speed_ = new SpeedGauge();
  speed_->setMinimumWidth(210);
  wheel_ = new SteeringWheel();
  wheel_->setMinimumWidth(190);
  battery_ = new BatteryGauge();
  battery_->setFixedWidth(190);
  auto * tiles = new QGridLayout();
  tiles->setHorizontalSpacing(18);
  tiles->setVerticalSpacing(6);
  temp_fet_ = new ValueTile("ESC temp", "°C");
  motor_current_ = new ValueTile("Motor", "A");
  brake_current_ = new ValueTile("Brake", "A");
  distance_ = new ValueTile("Odometer", "m");
  for (auto * tile : {temp_fet_, motor_current_, brake_current_, distance_}) {
    tile->setValuePixelSize(22);
    tile->setMinimumWidth(110);
  }
  tiles->addWidget(temp_fet_, 0, 0);
  tiles->addWidget(motor_current_, 0, 1);
  tiles->addWidget(brake_current_, 1, 0);
  tiles->addWidget(distance_, 1, 1);
  g_meter_ = new GForceMeter();
  g_meter_->setFixedWidth(140);
  instruments->addWidget(pedals_);
  instruments->addWidget(speed_, 2);
  instruments->addWidget(wheel_, 2);
  instruments->addWidget(battery_);
  instruments->addLayout(tiles, 2);
  instruments->addWidget(g_meter_);
  root->addLayout(instruments, 1);
  setFixedHeight(214);

  flags_->setFlag("vesc", "VESC", theme::text3, false);
  flags_->setFlag("joy", "JOYSTICK", theme::text3, false);
  flags_->setFlag("deadman", "DEADMAN", theme::green, false);
  flags_->setFlag("kill", "KILL SWITCH", theme::redBright, false);
  flags_->setFlag("auto_lock", "AUTO LOCK", theme::yellow, false);
  flags_->setFlag("estop", "E-STOP", theme::redBright, false);
  flags_->setFlag("loc", "LOCALIZED", theme::green, false);
  connect(ros_, &RosBridge::localization, this, [this](const LocalizationState & l) {
      localization_ms_ = QDateTime::currentMSecsSinceEpoch();
      flags_->setFlag("loc", l.localized ? "LOCALIZED" : "NOT LOCALIZED", l.localized ? theme::green : theme::yellow,
        l.active);
    });

  connect(ros_, &RosBridge::telemetry, this, &HudStrip::onTelemetry);
  connect(ros_, &RosBridge::imu, this, &HudStrip::onImu);
  connect(ros_, &RosBridge::linkChanged, this, [this](bool car, bool) {
      if (!car) {
        Telemetry offline;
        onTelemetry(offline);
      }
    });
  connect(ros_, &RosBridge::parameterEvent, this, [this](const ParameterEvent & e) {
      if (e.node == kBoostNode) {
        for (const auto & p : e.changed_parameters) {
          if (p.name == "max_multiplier") {
            max_boost_ = rclcpp::Parameter::from_parameter_msg(p).as_double();
          }
        }
      }
    });
  auto * poll = new QTimer(this);
  connect(poll, &QTimer::timeout, this, [this]() {
      ros_->getParameters(kBoostNode, {"max_multiplier"}, [this](bool ok, const std::vector<rclcpp::Parameter> & v) {
        if (ok && !v.empty() && v[0].get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          max_boost_ = v[0].as_double();
        }
      });
    });
  poll->start(15000);
}

void HudStrip::setMaxSpeed(double speed)
{
  speed_->setMaxSpeed(speed);
}

void HudStrip::onTelemetry(const Telemetry & t)
{
  bool on = t.vesc_connected;
  speed_->setActive(on);
  speed_->setSpeed(t.speed, t.speed_command);
  speed_->setBoost(1.0 + (max_boost_ - 1.0) * t.boost);
  double demand = std::clamp(std::abs(t.speed_command) / 4.0, 0.0, 1.0);
  double brake = std::max(static_cast<double>(t.brake_trigger), std::abs(t.brake_current) / 60.0);
  pedals_->set(on ? demand : 0.0, brake, t.speed_command < -0.01);
  wheel_->set(t.steering_angle, t.steering_command, 0.4);
  lights_->setFraction(on ? (std::abs(t.duty_cycle) - 0.12) / 0.8 : 0.0);
  int cells = std::clamp(static_cast<int>(std::round(t.battery_voltage / 3.85)), 1, 8);
  battery_->set(t.battery_percent, t.battery_voltage, cells, t.input_current, on);

  QColor fet = t.temp_fet < 60 ? theme::text : (t.temp_fet < 80 ? theme::yellow : theme::redBright);
  temp_fet_->setValue(on ? QString::number(t.temp_fet, 'f', 0) : "--", fet);
  temp_fet_->setBar(on ? t.temp_fet / 100.0 : -1.0, fet == theme::text ? theme::green : fet);
  motor_current_->setValue(on ? QString::number(std::abs(t.motor_current), 'f', 1) : "--");
  motor_current_->setBar(on ? std::abs(t.motor_current) / 60.0 : -1.0, theme::cyan);
  bool braking = std::abs(t.brake_current) > 0.5;
  brake_current_->setValue(on ? QString::number(std::abs(t.brake_current), 'f', 1) : "--",
    braking ? theme::redBright : theme::text);
  brake_current_->setBar(on ? std::abs(t.brake_current) / 60.0 : -1.0, theme::redBright);
  distance_->setValue(on ? QString::number(t.distance, 'f', 0) : "--");
  distance_->setDetail(on ? QString("%1 Wh used").arg(t.energy_drawn, 0, 'f', 1) : QString());

  flags_->setFlag("vesc", t.fault_code != 0 ? QString::fromStdString(t.fault) : "VESC",
    on && t.fault_code == 0 ? theme::green : theme::redBright, true);
  flags_->setFlag("joy", "JOYSTICK", t.joystick_connected ? theme::green : theme::redBright, true);
  flags_->setFlag("deadman", "DEADMAN", theme::green, t.deadman);
  flags_->setFlag("kill", "KILL SWITCH", theme::redBright, t.teleop_lock);
  flags_->setFlag("auto_lock", "AUTO LOCK", theme::yellow, t.autonomous_lock);
  flags_->setFlag("estop", "E-STOP", theme::redBright, t.estop);
  if (QDateTime::currentMSecsSinceEpoch() - localization_ms_ > 2000) {
    flags_->setFlag("loc", "LOCALIZED", theme::green, false);  // no map in this session
  }
  if (!imu_seen_) {
    g_meter_->addSample(0.0, 0.0);
  }
}

void HudStrip::onImu(double ax, double ay, double)
{
  imu_seen_ = true;
  // VESC IMU: x forward, y left. Screen: right is +x, braking (deceleration) pushes up
  g_lat_ = 0.7 * g_lat_ + 0.3 * (-ay / 9.81);
  g_lon_ = 0.7 * g_lon_ + 0.3 * (-ax / 9.81);
  g_meter_->addSample(g_lat_, g_lon_);
}

// One line of the autonomous checklist: LED, what is checked, and its state
class ChecklistRow : public QWidget
{
public:
  explicit ChecklistRow(const QString & title)
  {
    auto * layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);
    led_ = new StatusLed(nullptr, 10);
    auto * label = makeLabel(title, 13, QFont::DemiBold);
    detail_ = makeLabel("", 12, QFont::Normal, theme::text2);
    detail_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    detail_->setWordWrap(true);
    layout->addWidget(led_);
    layout->addWidget(label);
    layout->addWidget(detail_, 1);
  }
  void set(const QColor & color, const QString & detail)
  {
    led_->setColor(color);
    detail_->setText(detail);
  }

private:
  StatusLed * led_ = nullptr;
  QLabel * detail_ = nullptr;
};

// ---- LocalizationControls ----

LocalizationControls::LocalizationControls(RosBridge * ros, QWidget * parent)
: QWidget(parent), ros_(ros)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(6);
  auto * head = new QHBoxLayout();
  head->setSpacing(8);
  led_ = new StatusLed(nullptr, 10);
  status_ = makeLabel("Localized", 13, QFont::DemiBold);
  match_ = makeLabel("", 12, QFont::Bold, theme::text2);
  match_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  head->addWidget(led_);
  head->addWidget(status_, 1);
  head->addWidget(match_);
  root->addLayout(head);
  bar_ = new QProgressBar();
  bar_->setRange(0, 100);
  bar_->setTextVisible(false);
  bar_->setFixedHeight(6);
  bar_->setToolTip("Share of lidar points that land on the map's walls");
  root->addWidget(bar_);
  auto * buttons = new QHBoxLayout();
  buttons->setSpacing(6);
  start_line_ = makeButton("START LINE", theme::icon::flag);
  start_line_->setToolTip("The car stands on the path's start line, facing along the path");
  find_ = makeButton("FIND CAR", theme::icon::search);
  find_->setToolTip("Search the whole map for where the lidar scan fits (takes a second; the car can stand still)");
  set_pose_ = makeButton("SET POSE", theme::icon::pose);
  set_pose_->setToolTip("Click where the car is on the map and drag towards where it faces");
  for (auto * b : {start_line_, find_, set_pose_}) {
    b->setFont(theme::font(12, QFont::DemiBold));
    buttons->addWidget(b, 1);
  }
  root->addLayout(buttons);
  hint_ = makeLabel("", 11, QFont::Normal, theme::text3);
  hint_->setWordWrap(true);
  root->addWidget(hint_);

  auto reply = [this](const QString & what) {
      return [this, what](bool ok, const QString & text) {
               busy_ = false;
               refresh();
               Q_EMIT message(ok ? Level::Ok : Level::Warn, what + ": " + text);
             };
    };
  connect(start_line_, &QPushButton::clicked, this, [this, reply]() {
      busy_ = true;
      refresh();
      ros_->placeAtStartLine(reply("Start line"));
    });
  connect(find_, &QPushButton::clicked, this, [this, reply]() {
      busy_ = true;
      find_->setText("SEARCHING...");
      refresh();
      ros_->locateCar([this, reply](bool ok, const QString & text) {
        find_->setText("FIND CAR");
        reply("Find car")(ok, text);
      });
    });
  connect(set_pose_, &QPushButton::clicked, this, &LocalizationControls::setPoseRequested);
  connect(ros_, &RosBridge::localization, this, &LocalizationControls::onLocalization);
  connect(ros_, &RosBridge::supervisorState, this, [this](const SupervisorState & state) {
      bool has_path = !state.path.empty();
      if (has_path != has_path_) {
        has_path_ = has_path;
        refresh();
      }
    });
  // Reports stop when the session has no map
  auto * stale = new QTimer(this);
  connect(stale, &QTimer::timeout, this, [this]() {
      if (active_ && QDateTime::currentMSecsSinceEpoch() - last_ms_ > 2000) {
        active_ = false;
        localized_ = false;
        refresh();
        Q_EMIT changed();
      }
    });
  stale->start(1000);
  refresh();
}

void LocalizationControls::onLocalization(const LocalizationState & state)
{
  last_ms_ = QDateTime::currentMSecsSinceEpoch();
  bool was = localized_;
  active_ = state.active;
  localized_ = state.active && state.localized;
  match_value_ = state.match;
  state_message_ = QString::fromStdString(state.message);
  refresh();
  if (was != localized_) {
    Q_EMIT changed();
  }
}

void LocalizationControls::refresh()
{
  QColor color = !active_ ? theme::text3 : (localized_ ? theme::green :
    (match_value_ >= 0.25 ? theme::yellow : theme::redBright));
  led_->setColor(color);
  led_->setBlinking(active_ && !localized_);
  status_->setText(!active_ ? "Localized" : (localized_ ? "Localized" : state_message_));
  match_->setText(active_ ? QString("MATCH %1%").arg(match_value_ * 100.0, 0, 'f', 0) : QString());
  bar_->setValue(active_ ? static_cast<int>(match_value_ * 100.0) : 0);
  bar_->setStyleSheet(QString("QProgressBar::chunk { background: %1; border-radius: 2px; }").arg(theme::css(color)));
  start_line_->setEnabled(active_ && has_path_ && !busy_);
  find_->setEnabled(active_ && !busy_);
  set_pose_->setEnabled(active_);
  hint_->setText(!active_ ? "Needs a session with a map." : (localized_ ?
    "The lidar points (red) sit on the map's walls." :
    (has_path_ ? "Put the car on the start line and press START LINE, or let FIND CAR search the map." :
    "Press FIND CAR, or SET POSE and click-drag where the car is on the map.")));
}

// ---- DrivePanel ----

DrivePanel::DrivePanel(RosBridge * ros, QWidget * parent)
: Page(ros, parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(10);

  auto * driver = new Card("Driver");
  source_ = new SkewBadge();
  source_->setPixelSize(15);
  source_->set("IDLE", theme::bg3);
  auto * badge_row = row({source_});
  badge_row->addStretch(1);
  driver->body()->addLayout(badge_row);
  drive_message_ = makeLabel("", 12, QFont::Normal, theme::text2);
  drive_message_->setWordWrap(true);
  driver->body()->addWidget(drive_message_);
  root->addWidget(driver);

  // Autonomous: what must be true before the car may drive itself, then the button
  auto * autonomous = new Card("Autonomous");
  track_row_ = new ChecklistRow("Map and path");
  autonomous->body()->addWidget(track_row_);
  localization_ = new LocalizationControls(ros);
  autonomous->body()->addWidget(localization_);
  follower_row_ = new ChecklistRow("Path follower");
  autonomous->body()->addWidget(follower_row_);
  joystick_row_ = new ChecklistRow("Joystick");
  autonomous->body()->addWidget(joystick_row_);
  autonomous_ = makeButton("ENGAGE AUTONOMOUS", theme::icon::rocket, "go");
  autonomous_->setMinimumHeight(46);
  autonomous_->setFont(theme::font(15, QFont::Black, true));
  autonomous->body()->addWidget(autonomous_);
  autonomous_hint_ = makeLabel("", 11, QFont::Normal, theme::text3);
  autonomous_hint_->setWordWrap(true);
  autonomous->body()->addWidget(autonomous_hint_);
  root->addWidget(autonomous);

  auto * aids = new Card("Driver aids");
  assist_ = new Toggle("Steering assist (follow the line)");
  assist_->setAccent(theme::purple);
  safety_ = new Toggle("Collision brake");
  avoidance_ = new Toggle("Obstacle avoidance");
  for (auto * t : {assist_, safety_, avoidance_}) {
    aids->body()->addWidget(t);
  }
  auto slider_row = [&](const QString & caption, QSlider *& slider, QLabel *& value, int lo, int hi) {
      aids->body()->addWidget(makeCaption(caption));
      slider = new FocusWheel<QSlider>(Qt::Horizontal);
      slider->setRange(lo, hi);
      value = makeLabel("--", 13, QFont::Bold);
      value->setMinimumWidth(48);
      value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
      aids->body()->addLayout(row({slider, value}));
    };
  slider_row("Autonomous speed", auto_speed_, auto_speed_value_, 10, 100);
  slider_row("Boost (LT) up to", boost_, boost_value_, 10, 100);
  root->addWidget(aids);

  auto * timing = new Card("Timing");
  auto * reset = makeButton("", theme::icon::restart, "ghost");
  reset->setToolTip("Reset lap times");
  timing->header()->addWidget(reset);
  timing_ = new TimingTower();
  timing->body()->addWidget(timing_);
  root->addWidget(timing);
  root->addStretch(1);

  connect(autonomous_, &QPushButton::clicked, this, [this]() {
      bool engage = !drive_.autonomous_enabled;
      autonomous_->setEnabled(false);
      ros_->setAutonomous(engage, [this](bool ok, const QString & text) {
        autonomous_->setEnabled(true);
        Q_EMIT message(ok ? Level::Ok : Level::Error, text);
      });
    });
  connect(assist_, &Toggle::clicked, this, [this](bool on) {
      assist_->setPending(true);
      ros_->setAssist(on, [this, on](bool ok, const QString & text) {
        assist_->setPending(false);
        if (!ok) {
          assist_->setChecked(!on);
        }
        Q_EMIT message(ok ? Level::Ok : Level::Error, text);
      });
    });
  auto bind_toggle = [this](Toggle * toggle, const QString & node, const QString & what) {
      connect(toggle, &Toggle::clicked, this, [this, toggle, node, what](bool on) {
        if (!on && QMessageBox::warning(this, "Turn off " + what.toLower() + "?",
          what + " stops the car before it hits something. Turn it off only on a clear track.",
          QMessageBox::Cancel | QMessageBox::Ok, QMessageBox::Cancel) != QMessageBox::Ok)
        {
          toggle->setChecked(true);
          return;
        }
        toggle->setPending(true);
        ros_->setParameter(node, rclcpp::Parameter("enabled", on), [this, toggle, on, what](bool ok, const QString & text) {
          toggle->setPending(false);
          if (!ok) {
            toggle->setChecked(!on);
            Q_EMIT message(Level::Error, text);
          } else {
            Q_EMIT message(on ? Level::Ok : Level::Warn, what + (on ? " on" : " OFF"));
          }
        });
      });
    };
  bind_toggle(safety_, kSafetyNode, "Collision brake");
  bind_toggle(avoidance_, kAvoidanceNode, "Obstacle avoidance");
  // Sliders apply shortly after the last change, whether dragged, scrolled or moved with the keys
  auto debounce = [this](QSlider * slider, std::function<void()> apply) {
      auto * timer = new QTimer(this);
      timer->setSingleShot(true);
      timer->setInterval(350);
      connect(timer, &QTimer::timeout, this, apply);
      connect(slider, &QSlider::valueChanged, timer, [timer]() {timer->start();});
    };
  connect(auto_speed_, &QSlider::valueChanged, this, [this](int v) {auto_speed_value_->setText(QString("%1%").arg(v));});
  debounce(auto_speed_, [this]() {
      ros_->setParameter(kFollowerNode, rclcpp::Parameter("velocity_percentage", auto_speed_->value() / 100.0),
        report("Autonomous speed"));
    });
  connect(boost_, &QSlider::valueChanged, this, [this](int v) {boost_value_->setText(QString("x%1").arg(v / 10.0, 0, 'f', 1));});
  debounce(boost_, [this]() {
      ros_->setParameter(kBoostNode, rclcpp::Parameter("max_multiplier", boost_->value() / 10.0), report("Boost"));
    });
  connect(reset, &QPushButton::clicked, this, [this]() {
      timing_->reset();
      ros_->resetLap(report());
    });
  connect(ros_, &RosBridge::driveState, this, &DrivePanel::onDrive);
  connect(localization_, &LocalizationControls::changed, this, &DrivePanel::updateChecklist);
  connect(localization_, &LocalizationControls::message, this, &Page::message);
  connect(ros_, &RosBridge::supervisorState, this, [this](const SupervisorState & state) {
      QString map = QString::fromStdString(state.map), path = QString::fromStdString(state.path);
      if (map != map_ || path != path_) {
        map_ = map;
        path_ = path;
        updateChecklist();
      }
    });
  connect(ros_, &RosBridge::telemetry, this, [this](const Telemetry & t) {
      bool estop = t.estop || ros_->estopHeldHere();
      if (t.joystick_connected != joystick_ || estop != estop_) {
        joystick_ = t.joystick_connected;
        estop_ = estop;
        updateChecklist();
      }
    });
  connect(ros_, &RosBridge::estopChanged, this, [this](bool engaged, bool held_here) {
      estop_ = engaged || held_here;
      updateChecklist();
    });
  connect(ros_, &RosBridge::lapState, timing_, &TimingTower::setLap);
  connect(ros_, &RosBridge::parameterEvent, this, &DrivePanel::onParameterEvent);
  connect(ros_, &RosBridge::nodesChanged, this, [this]() {refreshParameters();});

  DriveState idle;
  onDrive(idle);
}

void DrivePanel::refreshParameters()
{
  auto fetch = [this](const QString & node, const QStringList & names) {
      ros_->getParameters(node, names, [this, node](bool ok, const std::vector<rclcpp::Parameter> & values) {
        if (ok) {
          for (const auto & p : values) {
            applyParameter(node, p);
          }
        }
      });
    };
  QStringList nodes = ros_->nodes();
  safety_->setEnabled(nodes.contains(kSafetyNode));
  avoidance_->setEnabled(nodes.contains(kAvoidanceNode));
  auto_speed_->setEnabled(nodes.contains(kFollowerNode));
  boost_->setEnabled(nodes.contains(kBoostNode));
  if (nodes.contains(kSafetyNode)) {fetch(kSafetyNode, {"enabled"});}
  if (nodes.contains(kAvoidanceNode)) {fetch(kAvoidanceNode, {"enabled"});}
  if (nodes.contains(kFollowerNode)) {fetch(kFollowerNode, {"velocity_percentage"});}
  if (nodes.contains(kBoostNode)) {fetch(kBoostNode, {"max_multiplier"});}
}

void DrivePanel::applyParameter(const QString & node, const rclcpp::Parameter & p)
{
  auto type = p.get_type();
  if (p.get_name() == "enabled" && type == rclcpp::ParameterType::PARAMETER_BOOL) {
    if (node == kSafetyNode) {safety_->setChecked(p.as_bool());}
    if (node == kAvoidanceNode) {avoidance_->setChecked(p.as_bool());}
  } else if (node == kFollowerNode && p.get_name() == "velocity_percentage" &&
    type == rclcpp::ParameterType::PARAMETER_DOUBLE && !auto_speed_->isSliderDown())
  {
    QSignalBlocker block(auto_speed_);
    auto_speed_->setValue(static_cast<int>(std::round(p.as_double() * 100.0)));
    auto_speed_value_->setText(QString("%1%").arg(auto_speed_->value()));
  } else if (node == kBoostNode && p.get_name() == "max_multiplier" &&
    type == rclcpp::ParameterType::PARAMETER_DOUBLE && !boost_->isSliderDown())
  {
    QSignalBlocker block(boost_);
    boost_->setValue(static_cast<int>(std::round(p.as_double() * 10.0)));
    boost_value_->setText(QString("x%1").arg(boost_->value() / 10.0, 0, 'f', 1));
  }
}

void DrivePanel::onParameterEvent(const ParameterEvent & event)
{
  QString node = QString::fromStdString(event.node);
  for (const auto & p : event.changed_parameters) {
    applyParameter(node, rclcpp::Parameter::from_parameter_msg(p));
  }
}

void DrivePanel::onDrive(const DriveState & d)
{
  drive_ = d;
  switch (d.source) {
    case DriveState::AUTONOMOUS: source_->set("AUTONOMOUS", theme::green, Qt::black); break;
    case DriveState::ASSIST: source_->set("ASSISTED", theme::purple); break;
    case DriveState::MANUAL: source_->set("MANUAL", theme::red); break;
    default: source_->set(d.autonomous_enabled ? "AUTONOMOUS ARMED" : "NO INPUT", theme::bg3); break;
  }
  drive_message_->setText(QString::fromStdString(d.message));
  assist_->setEnabled(d.follower_available);
  if (!assist_->isDown()) {
    assist_->setChecked(d.assist_enabled);
  }
  updateChecklist();
}

void DrivePanel::updateChecklist()
{
  bool track = !map_.isEmpty() && !path_.isEmpty();
  bool localized = localization_->localized();
  bool follower = drive_.follower_available;
  track_row_->set(track ? theme::green : theme::redBright,
    track ? map_ + "  ·  " + path_ : "Start a MANUAL session with both on HOME");
  follower_row_->set(follower ? theme::green : (track ? theme::yellow : theme::text3),
    follower ? "Ready" : (track ? "Starting..." : "Runs with a map and a path"));
  joystick_row_->set(joystick_ ? theme::green : theme::yellow,
    joystick_ ? "RB is your kill switch" : "Not connected: only E-STOP can stop the car");
  bool ready = track && localized && follower && !estop_;
  if (drive_.autonomous_enabled) {
    autonomous_->setText("DISENGAGE");
    autonomous_->setProperty("role", "primary");
    autonomous_->setIcon(glyphIcon(theme::icon::stop, theme::text, 16));
    autonomous_->setEnabled(true);
    autonomous_hint_->setText("Stop: this button, B or RT on the joystick, RB kill switch, or E-STOP.");
  } else {
    autonomous_->setText("ENGAGE AUTONOMOUS");
    autonomous_->setProperty("role", "go");
    autonomous_->setIcon(glyphIcon(theme::icon::rocket, theme::text, 16));
    autonomous_->setEnabled(ready);
    autonomous_hint_->setText(estop_ ? "Release the E-STOP first." : (ready ?
      "Or press A on the joystick. Speed: AUTONOMOUS SPEED below." : "Engage unlocks when everything above is green."));
  }
  repolish(autonomous_);
}

// ---- MappingPanel ----

MappingPanel::MappingPanel(RosBridge * ros, QWidget * parent)
: Page(ros, parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(10);

  auto * status = new Card("Mapping");
  led_ = new StatusLed(nullptr, 11);
  status_ = makeLabel("SLAM NOT RUNNING", 15, QFont::Bold);
  auto * head = row({led_, status_});
  head->addStretch(1);
  status->body()->addLayout(head);
  auto * tiles = new QHBoxLayout();
  size_ = new ValueTile("Map size");
  explored_ = new ValueTile("Known");
  resolution_ = new ValueTile("Cell");
  for (auto * tile : {size_, explored_, resolution_}) {
    tile->setValuePixelSize(20);
    tiles->addWidget(tile);
  }
  status->body()->addLayout(tiles);
  pause_ = makeButton("PAUSE", theme::icon::pause);
  auto * restart = makeButton("START OVER", theme::icon::restart);
  status->body()->addLayout(row({pause_, restart}));
  root->addWidget(status);

  auto * save = new Card("Save map");
  name_ = new QLineEdit(defaultName("track"));
  name_->setValidator(nameValidator(name_));
  save->body()->addWidget(name_);
  save_ = makeButton("SAVE MAP", theme::icon::save, "primary");
  save_->setMinimumHeight(40);
  save->body()->addWidget(save_);
  saved_count_ = makeLabel("", 12, QFont::Normal, theme::text3);
  auto * garage = makeButton("GARAGE", theme::icon::garage, "ghost");
  auto * saved_row = row({saved_count_, garage});
  save->body()->addLayout(saved_row);
  root->addWidget(save);

  auto * tips = new Card("How to map");
  auto * text = makeLabel(
    "1. Drive slowly and smoothly (under ~1.5 m/s).\n"
    "2. Go round the whole track and come back to the start to close the loop.\n"
    "3. PAUSE before picking the car up or carrying it.\n"
    "4. SAVE MAP, then start a PATH session on it.", 12, QFont::Normal, theme::text2);
  text->setWordWrap(true);
  tips->body()->addWidget(text);
  auto * params = makeButton("SLAM PARAMETERS", theme::icon::tune);
  tips->body()->addWidget(params);
  root->addWidget(tips);
  root->addStretch(1);

  connect(pause_, &QPushButton::clicked, this, [this]() {ros_->toggleMappingPause(report());});
  connect(restart, &QPushButton::clicked, this, [this]() {
      if (QMessageBox::question(this, "Start mapping over?", "This throws away the map being built.") ==
        QMessageBox::Yes)
      {
        ros_->componentCommand("slam", "restart", report());
      }
    });
  connect(save_, &QPushButton::clicked, this, [this]() {
      QString name = name_->text().trimmed();
      if (name.isEmpty()) {
        Q_EMIT message(Level::Warn, "Give the map a name");
        return;
      }
      save_->setEnabled(false);
      save_->setText("SAVING...");
      ros_->saveMap(name, [this](bool ok, const QString & text) {
        save_->setEnabled(true);
        save_->setText("SAVE MAP");
        Q_EMIT message(ok ? Level::Ok : Level::Error, text);
        if (ok) {
          name_->setText(defaultName("track"));
        }
      });
    });
  connect(garage, &QPushButton::clicked, this, &MappingPanel::openGarage);
  connect(params, &QPushButton::clicked, this, [this]() {Q_EMIT openParameters("/slam_toolbox");});
  connect(ros_, &RosBridge::supervisorState, this, &MappingPanel::onSupervisor);
}

void MappingPanel::setMapInfo(int width, int height, double resolution, double known_fraction)
{
  size_->setValue(QString("%1x%2").arg(width * resolution, 0, 'f', 0).arg(height * resolution, 0, 'f', 0));
  size_->setDetail("metres");
  explored_->setValue(QString("%1%").arg(known_fraction * 100.0, 0, 'f', 0));
  resolution_->setValue(QString("%1").arg(resolution * 100.0, 0, 'f', 0));
  resolution_->setDetail("cm");
}

void MappingPanel::onSupervisor(const SupervisorState & state)
{
  std::string slam = "stopped";
  for (const auto & c : state.components) {
    if (c.name == "slam") {
      slam = c.status;
    }
  }
  paused_ = state.mapping_paused;
  bool running = state.mode == "mapping" && slam == "running";
  led_->setColor(running ? (paused_ ? theme::yellow : theme::green) : (slam == "starting" ? theme::yellow : theme::text3));
  led_->setBlinking(running && !paused_);
  status_->setText(state.mode != "mapping" ? "NOT IN A MAPPING SESSION" :
    (slam == "running" ? (paused_ ? "PAUSED: SCANS IGNORED" : "MAPPING") : "SLAM " + QString::fromStdString(slam).toUpper()));
  pause_->setText(paused_ ? "RESUME" : "PAUSE");
  pause_->setIcon(glyphIcon(paused_ ? theme::icon::play : theme::icon::pause, theme::text, 16));
  pause_->setEnabled(running);
  save_->setEnabled(running && save_->text() == "SAVE MAP");
  saved_count_->setText(QString("%1 map%2 on the car").arg(state.maps.size()).arg(state.maps.size() == 1 ? "" : "s"));
}

// ---- PathPanel ----

PathPanel::PathPanel(RosBridge * ros, QWidget * parent)
: Page(ros, parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(10);

  auto * where = new Card("Localization");
  localization_ = new LocalizationControls(ros);
  where->body()->addWidget(localization_);
  root->addWidget(where);
  connect(localization_, &LocalizationControls::message, this, &Page::message);

  auto * rec = new Card("Record a lap");
  rec_led_ = new StatusLed(nullptr, 11);
  record_ = makeButton("RECORD", theme::icon::record, "primary");
  record_->setMinimumHeight(38);
  auto * clear = makeButton("CLEAR", theme::icon::trash);
  auto * head = row({rec_led_, record_, clear});
  head->setStretch(1, 1);
  rec->body()->addLayout(head);
  auto * tiles = new QHBoxLayout();
  points_ = new ValueTile("Points");
  length_ = new ValueTile("Length", "m");
  top_speed_ = new ValueTile("Top", "m/s");
  for (auto * tile : {points_, length_, top_speed_}) {
    tile->setValuePixelSize(20);
    tiles->addWidget(tile);
  }
  rec->body()->addLayout(tiles);
  record_name_ = new QLineEdit(defaultName("lap"));
  record_name_->setValidator(nameValidator(record_name_));
  auto * save = makeButton("SAVE", theme::icon::save);
  rec->body()->addLayout(row({record_name_, save}));
  root->addWidget(rec);

  auto * race = new Card("Raceline optimizer");
  source_ = new FocusWheel<QComboBox>();
  race->body()->addWidget(makeCaption("Source path"));
  race->body()->addWidget(source_);
  auto * form = new QGridLayout();
  form->setHorizontalSpacing(10);
  form->setVerticalSpacing(6);
  auto spin = [&](int r, int c, const QString & caption, QDoubleSpinBox *& box, double lo, double hi, double step,
    double value, const QString & suffix) {
      box = new FocusWheel<QDoubleSpinBox>();
      box->setRange(lo, hi);
      box->setSingleStep(step);
      box->setDecimals(2);
      box->setValue(value);
      box->setSuffix(" " + suffix);
      form->addWidget(makeCaption(caption), r * 2, c);
      form->addWidget(box, r * 2 + 1, c);
    };
  spin(0, 0, "Wall margin", margin_, 0.01, 1.0, 0.02, 0.10, "m");
  spin(0, 1, "Top speed", max_speed_, 0.5, 15.0, 0.5, 6.0, "m/s");
  spin(1, 0, "Corner grip", lateral_, 0.5, 15.0, 0.5, 5.0, "m/s²");
  spin(1, 1, "Acceleration", accel_, 0.5, 10.0, 0.5, 3.0, "m/s²");
  spin(2, 0, "Braking", decel_, 0.5, 15.0, 0.5, 5.0, "m/s²");
  race->body()->addLayout(form);
  optimize_ = makeButton("OPTIMIZE", theme::icon::magic, "primary");
  optimize_->setMinimumHeight(38);
  race->body()->addWidget(optimize_);
  result_ = makeLabel("Minimum-curvature line inside the walls of the session's map, with a speed profile.", 12,
    QFont::Normal, theme::text2);
  result_->setWordWrap(true);
  race->body()->addWidget(result_);
  raceline_name_ = new QLineEdit(defaultName("raceline"));
  raceline_name_->setValidator(nameValidator(raceline_name_));
  save_raceline_ = makeButton("SAVE", theme::icon::save);
  save_raceline_->setEnabled(false);
  race->body()->addLayout(row({raceline_name_, save_raceline_}));
  drive_it_ = makeButton("DRIVE THIS RACELINE", theme::icon::flag, "go");
  drive_it_->setEnabled(false);
  race->body()->addWidget(drive_it_);
  root->addWidget(race);
  root->addStretch(1);

  connect(record_, &QPushButton::clicked, this, [this]() {
      if (!recording_ && !localization_->localized() &&
        QMessageBox::question(this, "Record without localization?", "The car isn't localized on the map, so the "
        "recorded path would be wrong. Record anyway?") != QMessageBox::Yes)
      {
        return;
      }
      ros_->recorder(recording_ ? "stop" : "start", report());
    });
  connect(clear, &QPushButton::clicked, this, [this]() {ros_->recorder("clear", report());});
  connect(save, &QPushButton::clicked, this, [this, save]() {
      save->setEnabled(false);
      ros_->saveRecording(record_name_->text().trimmed(), [this, save](bool ok, const QString & text) {
        save->setEnabled(true);
        Q_EMIT message(ok ? Level::Ok : Level::Error, text);
        if (ok) {
          // Offer the new lap as the optimizer's source once the supervisor lists it
          QString saved = record_name_->text().trimmed();
          record_name_->setText(defaultName("lap"));
          QTimer::singleShot(1200, this, [this, saved]() {
            int index = source_->findData(saved);
            if (index >= 0) {
              source_->setCurrentIndex(index);
            }
          });
        }
      });
    });
  connect(optimize_, &QPushButton::clicked, this, &PathPanel::optimize);
  connect(save_raceline_, &QPushButton::clicked, this, [this]() {
      QString name = raceline_name_->text().trimmed();
      ros_->saveRaceline(name, [this, name](bool ok, const QString & text) {
        Q_EMIT message(ok ? Level::Ok : Level::Error, text);
        if (ok) {
          saved_raceline_ = name;
          drive_it_->setEnabled(true);
          drive_it_->setText("DRIVE " + name.toUpper());
        }
      });
    });
  connect(drive_it_, &QPushButton::clicked, this, [this]() {
      ros_->setMode("manual", map_, saved_raceline_, report());
    });
  connect(ros_, &RosBridge::supervisorState, this, &PathPanel::onSupervisor);
  connect(ros_, &RosBridge::recorderState, this, &PathPanel::onRecorder);
}

void PathPanel::onSupervisor(const SupervisorState & state)
{
  map_ = QString::fromStdString(state.map);
  // Only paths made on the session's map (or of unknown map) can be optimized in its walls
  auto paths = pathsForMap(state, map_);
  QStringList wanted, current;
  for (const auto & [name, label] : paths) {
    wanted << label;
  }
  for (int i = 0; i < source_->count(); ++i) {
    current << source_->itemText(i);
  }
  if (current != wanted) {
    QString selected = source_->currentData().toString();
    source_->clear();
    for (const auto & [name, label] : paths) {
      source_->addItem(label, name);
    }
    int index = source_->findData(selected);
    if (index >= 0) {
      source_->setCurrentIndex(index);
    }
  }
  bool tools = false;
  for (const auto & c : state.components) {
    if (c.name == "path_tools" && c.status == "running") {
      tools = true;
    }
  }
  record_->setEnabled(tools);
  optimize_->setEnabled(tools && source_->count() > 0);
}

void PathPanel::onRecorder(const RecorderState & state)
{
  recording_ = state.recording;
  rec_led_->setColor(recording_ ? theme::redBright : theme::text3);
  rec_led_->setBlinking(recording_);
  record_->setText(recording_ ? "PAUSE" : (state.points > 0 ? "CONTINUE" : "RECORD"));
  record_->setIcon(glyphIcon(recording_ ? theme::icon::pause : theme::icon::record, theme::text, 16));
  points_->setValue(QString::number(state.points));
  length_->setValue(QString::number(state.length, 'f', 1));
  top_speed_->setValue(QString::number(state.max_speed, 'f', 1));
}

void PathPanel::optimize()
{
  if (map_.isEmpty()) {
    Q_EMIT message(Level::Warn, "The optimizer needs the session's map");
    return;
  }
  f1tenth_bringup::srv::OptimizeRaceline::Request request;
  request.path = source_->currentData().toString().toStdString();
  request.map = map_.toStdString();
  request.margin = static_cast<float>(margin_->value());
  request.max_speed = static_cast<float>(max_speed_->value());
  request.max_lateral_accel = static_cast<float>(lateral_->value());
  request.max_accel = static_cast<float>(accel_->value());
  request.max_decel = static_cast<float>(decel_->value());
  optimize_->setEnabled(false);
  optimize_->setText("OPTIMIZING...");
  result_->setText("Working...");
  ros_->optimizeRaceline(request, [this](const RacelineResult & r) {
      optimize_->setEnabled(true);
      optimize_->setText("OPTIMIZE");
      if (!r.ok) {
        result_->setText(r.message);
        result_->setStyleSheet(QString("color: %1;").arg(theme::css(theme::redBright)));
        Q_EMIT message(Level::Error, "Raceline: " + r.message);
        return;
      }
      result_->setStyleSheet(QString("color: %1;").arg(theme::css(theme::text)));
      result_->setText(QString("<b>%1 m</b> &nbsp; est. lap <b style='color:%2'>%3 s</b> &nbsp; narrowest %4 m<br>"
        "<span style='color:%5'>%6</span>").arg(r.length, 0, 'f', 1).arg(theme::css(theme::purple))
        .arg(r.lap_time, 0, 'f', 2).arg(r.min_width, 0, 'f', 2).arg(theme::css(theme::text2)).arg(r.message.toHtmlEscaped()));
      save_raceline_->setEnabled(true);
      Q_EMIT message(Level::Ok, "Raceline ready: " + r.message);
      Q_EMIT showRaceline();
    });
}

// ---- SessionPage ----

SessionPage::SessionPage(RosBridge * ros, const std::string & rviz_node_name, QWidget * parent)
: Page(ros, parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);
  auto * main = new QHBoxLayout();
  main->setContentsMargins(12, 10, 12, 10);
  main->setSpacing(12);

  // Left: view controls over the 3D scene (or the big camera)
  auto * left = new QVBoxLayout();
  left->setSpacing(8);
  auto * toolbar = new QHBoxLayout();
  toolbar->setSpacing(0);
  auto * views = new QButtonGroup(this);
  const std::pair<SceneView::View, const char *> view_list[] = {
    {SceneView::View::Bird, "BIRD"}, {SceneView::View::Chase, "CHASE"},
    {SceneView::View::Onboard, "ONBOARD"}, {SceneView::View::Orbit, "ORBIT"}};
  for (const auto & [view, label] : view_list) {
    auto * button = makeButton(label, QChar(), "seg");
    button->setCheckable(true);
    views->addButton(button);
    toolbar->addWidget(button);
    view_buttons_[view] = button;
    SceneView::View v = view;
    connect(button, &QPushButton::clicked, this, [this, v]() {
        if (camera_big_shown_) {
          swapCamera();
        }
        scene_->setView(v);
        follow_->setVisible(v == SceneView::View::Bird);
      });
  }
  view_buttons_[SceneView::View::Chase]->setChecked(true);
  toolbar->addSpacing(8);
  follow_ = makeButton("FOLLOW", theme::icon::navigation, "seg");
  follow_->setCheckable(true);
  follow_->setChecked(true);
  follow_->setVisible(false);
  follow_->setToolTip("Bird view moves with the car");
  toolbar->addWidget(follow_);
  auto * reset = makeButton("", theme::icon::focus, "ghost");
  reset->setToolTip("Reset this view");
  toolbar->addWidget(reset);
  toolbar->addSpacing(8);
  pose_ = makeButton("SET POSE", theme::icon::pose);
  pose_->setCheckable(true);
  pose_->setToolTip("Tell the localization where the car is: click its position and drag towards where it faces");
  measure_ = makeButton("MEASURE", theme::icon::measure);
  measure_->setCheckable(true);
  auto * layers = makeButton("LAYERS", theme::icon::layers);
  auto * menu = new QMenu(layers);
  for (auto layer : SceneView::layers()) {
    auto * action = menu->addAction(QString(SceneView::layerName(layer)).replace("&", "&&"));
    action->setCheckable(true);
    action->setData(static_cast<int>(layer));
    connect(action, &QAction::toggled, this, [this, layer](bool on) {
        user_layers_[layer] = on;
        scene_->setLayerVisible(layer, on);
      });
  }
  connect(menu, &QMenu::aboutToShow, this, [this, menu]() {
      for (auto * action : menu->actions()) {
        action->blockSignals(true);
        action->setChecked(scene_->layerVisible(static_cast<SceneView::Layer>(action->data().toInt())));
        action->blockSignals(false);
      }
    });
  layers->setMenu(menu);
  auto * clear_trail = makeButton("", theme::icon::trash, "ghost");
  clear_trail->setToolTip("Clear the speed trail");
  auto * swap = makeButton("CAMERA", theme::icon::swap);
  swap->setToolTip("Swap the camera and the 3D view");
  for (auto * w : {static_cast<QWidget *>(pose_), static_cast<QWidget *>(measure_), static_cast<QWidget *>(layers),
      static_cast<QWidget *>(clear_trail), static_cast<QWidget *>(swap)})
  {
    toolbar->addWidget(w);
    toolbar->addSpacing(6);
  }
  status_ = new ElidedLabel();
  status_->setFont(theme::font(12));
  status_->setStyleSheet(QString("color: %1;").arg(theme::css(theme::text3)));
  status_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
  status_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  toolbar->addWidget(status_, 1);
  left->addLayout(toolbar);

  big_ = new QStackedWidget();
  scene_ = new SceneView(rviz_node_name);
  camera_big_ = new CameraView();
  big_->addWidget(scene_);
  big_->addWidget(camera_big_);
  left->addWidget(big_, 1);
  main->addLayout(left, 1);

  // Right: camera (or track map) and the panel for the session's mode
  auto * right = new QWidget();
  right->setFixedWidth(392);
  auto * rv = new QVBoxLayout(right);
  rv->setContentsMargins(0, 0, 0, 0);
  rv->setSpacing(10);
  small_ = new QStackedWidget();
  camera_small_ = new CameraView();
  minimap_ = new TrackMap();
  minimap_->setPlaceholder("Track map appears when the car is on a map");
  small_->addWidget(camera_small_);
  small_->addWidget(minimap_);
  small_->setFixedHeight(220);
  rv->addWidget(small_);
  panels_ = new QStackedWidget();
  auto * idle = new QWidget();
  auto * idle_layout = new QVBoxLayout(idle);
  idle_layout->setContentsMargins(0, 0, 0, 0);
  auto * idle_card = new Card("Standby");
  auto * idle_text = makeLabel("Nothing is running on the car. Start a session on HOME: manual driving (with or "
    "without a map), mapping, or path & raceline.", 13, QFont::Normal, theme::text2);
  idle_text->setWordWrap(true);
  idle_card->body()->addWidget(idle_text);
  auto * home = makeButton("CHOOSE A SESSION", theme::icon::home, "primary");
  idle_card->body()->addWidget(home);
  idle_layout->addWidget(idle_card);
  idle_layout->addStretch(1);
  idle_ = idle;
  drive_ = new DrivePanel(ros);
  mapping_ = new MappingPanel(ros);
  path_ = new PathPanel(ros);
  panels_->addWidget(idle_);
  panels_->addWidget(scrollable(drive_));
  panels_->addWidget(scrollable(mapping_));
  panels_->addWidget(scrollable(path_));
  rv->addWidget(panels_, 1);
  main->addWidget(right);
  root->addLayout(main, 1);

  hud_ = new HudStrip(ros);
  root->addWidget(hud_);

  for (Page * page : {static_cast<Page *>(drive_), static_cast<Page *>(mapping_), static_cast<Page *>(path_)}) {
    connect(page, &Page::message, this, &Page::message);
  }
  connect(home, &QPushButton::clicked, this, &SessionPage::openHome);
  connect(mapping_, &MappingPanel::openParameters, this, &SessionPage::openParameters);
  connect(mapping_, &MappingPanel::openGarage, this, &SessionPage::openGarage);
  connect(drive_->localization(), &LocalizationControls::setPoseRequested, this, &SessionPage::startPoseOnMap);
  connect(path_->localization(), &LocalizationControls::setPoseRequested, this, &SessionPage::startPoseOnMap);
  connect(path_, &PathPanel::showRaceline, this, [this]() {
      scene_->setLayerVisible(SceneView::Layer::Raceline, true);
      user_layers_[SceneView::Layer::Raceline] = true;
    });
  connect(follow_, &QPushButton::toggled, this, [this](bool on) {scene_->setFollow(on);});
  connect(reset, &QPushButton::clicked, this, [this]() {scene_->resetView();});
  connect(pose_, &QPushButton::clicked, this, [this](bool on) {
      measure_->setChecked(false);
      if (on) {
        scene_->startPoseTool();
      } else {
        scene_->cancelTool();
      }
    });
  connect(measure_, &QPushButton::clicked, this, [this](bool on) {
      pose_->setChecked(false);
      if (on) {
        scene_->startMeasureTool();
      } else {
        scene_->cancelTool();
      }
    });
  connect(clear_trail, &QPushButton::clicked, this, [this]() {scene_->clearTrail();});
  connect(swap, &QPushButton::clicked, this, &SessionPage::swapCamera);
  connect(camera_small_, &CameraView::clicked, this, &SessionPage::swapCamera);
  connect(camera_big_, &CameraView::clicked, this, &SessionPage::swapCamera);
  connect(scene_, &SceneView::toolFinished, this, [this]() {
      if (pose_->isChecked()) {
        Q_EMIT message(Level::Ok, "Pose sent to the localization");
      }
      pose_->setChecked(false);
      measure_->setChecked(false);
    });
  connect(scene_, &SceneView::statusMessage, this, [this](const QString & text) {status_->setFullText(text);});
  connect(scene_, &SceneView::mapUpdated, this, [this](int w, int h, double res, double known) {
      mapping_->setMapInfo(w, h, res, known);
      QImage image;
      double resolution, ox, oy;
      if (scene_->mapImage(image, resolution, ox, oy)) {
        minimap_->setMap(image, resolution, ox, oy);
      }
    });
  connect(ros_, &RosBridge::cameraFrame, this, [this](const QImage & image, double fps) {
      (camera_big_shown_ ? camera_big_ : camera_small_)->setFrame(image, fps);
    });
  connect(ros_, &RosBridge::telemetry, this, [this](const Telemetry & t) {scene_->setCarSpeed(t.speed);});
  connect(ros_, &RosBridge::supervisorState, this, &SessionPage::onSupervisor);

  // The car on the track map
  auto * pose_timer = new QTimer(this);
  connect(pose_timer, &QTimer::timeout, this, [this]() {
      double x, y, yaw;
      bool valid = active_ && scene_->fixedFrame() == "map" && scene_->carPose(x, y, yaw);
      minimap_->setCar(valid, x, y, yaw);
    });
  pose_timer->start(100);
  setMode("idle");
}

void SessionPage::initializeScene()
{
  scene_->initialize();
  applyLayerDefaults(mode_);
}

QString SessionPage::title() const
{
  if (mode_ == "mapping") {return "MAPPING";}
  if (mode_ == "path") {return "PATH";}
  return "DRIVE";
}

void SessionPage::setActive(bool active)
{
  active_ = active;
  updateCameraWanted();
}

void SessionPage::updateCameraWanted()
{
  bool camera_visible = camera_big_shown_ || small_->currentWidget() == camera_small_;
  ros_->setCameraWanted(active_ && camera_visible, camera_big_shown_ ? QSize(1280, 720) : QSize(640, 360));
}

void SessionPage::startPoseOnMap()
{
  if (camera_big_shown_) {
    swapCamera();
  }
  view_buttons_[SceneView::View::Bird]->setChecked(true);
  follow_->setVisible(true);
  follow_->blockSignals(true);
  follow_->setChecked(false);
  follow_->blockSignals(false);
  scene_->showWholeMap();
  measure_->setChecked(false);
  pose_->setChecked(true);
  scene_->startPoseTool();
}

void SessionPage::swapCamera()
{
  camera_big_shown_ = !camera_big_shown_;
  big_->setCurrentWidget(camera_big_shown_ ? static_cast<QWidget *>(camera_big_) : scene_);
  small_->setCurrentWidget(camera_big_shown_ ? static_cast<QWidget *>(minimap_) : camera_small_);
  updateCameraWanted();
}

void SessionPage::onSupervisor(const SupervisorState & state)
{
  QString mode = QString::fromStdString(state.mode);
  QString map = QString::fromStdString(state.map);
  if (mode != mode_) {
    setMode(mode);
  }
  scene_->setFixedFrame(mode == "mapping" || !map.isEmpty() ? "map" : "odom");
  if (map != map_) {
    map_ = map;
    minimap_->clearMap();
    minimap_->clearPath();
    session_path_.clear();
  }
  // The session's path on the track map too
  QString path = QString::fromStdString(state.path);
  if (path != session_path_) {
    session_path_ = path;
    minimap_->clearPath();
    if (!path.isEmpty()) {
      ros_->path(path, [this, path](const PathData & data) {
          if (data.ok && path == session_path_) {
            minimap_->setPath(data.points, data.speeds);
          }
        });
    }
  }
}

void SessionPage::setMode(const QString & mode)
{
  mode_ = mode;
  if (mode == "manual") {
    panels_->setCurrentIndex(1);
    drive_->refreshParameters();
  } else if (mode == "mapping") {
    panels_->setCurrentIndex(2);
  } else if (mode == "path") {
    panels_->setCurrentIndex(3);
  } else {
    panels_->setCurrentIndex(0);
  }
  applyLayerDefaults(mode);
  Q_EMIT titleChanged(title());
}

void SessionPage::applyLayerDefaults(const QString & mode)
{
  if (!scene_->initialized()) {
    return;
  }
  using L = SceneView::Layer;
  std::map<L, bool> on = {
    {L::Grid, true}, {L::Map, true}, {L::Lidar, true}, {L::Car, true}, {L::Trail, true}, {L::Safety, true},
    {L::Path, mode == "manual" || mode == "path"}, {L::Follower, mode == "manual"},
    {L::Recording, mode == "path"}, {L::Raceline, mode == "path"}};
  for (const auto & [layer, visible] : on) {
    auto user = user_layers_.find(layer);
    scene_->setLayerVisible(layer, user != user_layers_.end() ? user->second : visible);
  }
}
}  // namespace f1ui
