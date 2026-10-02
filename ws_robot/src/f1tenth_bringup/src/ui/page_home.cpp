#include <QAbstractButton>
#include <QButtonGroup>
#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QProcess>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "hud.hpp"
#include "pages.hpp"
#include "theme.hpp"
#include "views.hpp"
#include "widgets.hpp"

namespace f1ui
{
Page::Page(RosBridge * ros, QWidget * parent)
: QWidget(parent), ros_(ros)
{
}

std::vector<std::pair<QString, QString>> pathsForMap(const SupervisorState & state, const QString & map)
{
  std::vector<std::pair<QString, QString>> own, unknown;
  if (map.isEmpty()) {
    return own;
  }
  for (size_t i = 0; i < state.paths.size(); ++i) {
    QString name = QString::fromStdString(state.paths[i]);
    QString made_on = i < state.path_maps.size() ? QString::fromStdString(state.path_maps[i]) : QString();
    if (made_on == map) {
      own.emplace_back(name, name);
    } else if (made_on.isEmpty()) {
      unknown.emplace_back(name, name + "   (map unknown)");
    }
  }
  own.insert(own.end(), unknown.begin(), unknown.end());
  return own;
}

QString pathMap(const SupervisorState & state, const QString & path)
{
  for (size_t i = 0; i < state.paths.size() && i < state.path_maps.size(); ++i) {
    if (QString::fromStdString(state.paths[i]) == path) {
      return QString::fromStdString(state.path_maps[i]);
    }
  }
  return QString();
}

ReplyFn Page::report(const QString & context)
{
  return [this, context](bool ok, const QString & text) {
           Q_EMIT message(ok ? Level::Ok : Level::Error, context.isEmpty() ? text : context + ": " + text);
         };
}

// Big selectable card for a session mode
class ModeTile : public QAbstractButton
{
public:
  ModeTile(QChar glyph, const QString & title, const QString & description, const QColor & color,
    QWidget * parent = nullptr)
  : QAbstractButton(parent), glyph_(glyph), title_(title), description_(description), color_(color)
  {
    setCheckable(true);
    setCursor(Qt::PointingHandCursor);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  }
  void setRunning(bool running) {running_ = running; update();}
  QSize sizeHint() const override {return QSize(280, 138);}
  QSize minimumSizeHint() const override {return QSize(200, 124);}

protected:
  void enterEvent(QEvent *) override {hover_ = true; update();}
  void leaveEvent(QEvent *) override {hover_ = false; update();}
  void paintEvent(QPaintEvent *) override
  {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    QRectF r = QRectF(rect()).adjusted(1, 1, -1, -1);
    QColor fill = isChecked() ? QColor(30, 30, 41) : (hover_ ? theme::bg3 : theme::bg2);
    p.setPen(isChecked() ? QPen(color_, 2.0) : QPen(theme::line, 1.0));
    p.setBrush(fill);
    p.drawRoundedRect(r, 6, 6);
    if (isChecked()) {
      QLinearGradient g(r.topLeft(), r.topRight());
      QColor tint = color_;
      tint.setAlpha(45);
      g.setColorAt(0, tint);
      g.setColorAt(0.6, Qt::transparent);
      p.setPen(Qt::NoPen);
      p.setBrush(g);
      p.drawRoundedRect(r, 6, 6);
      p.fillRect(QRectF(r.left(), r.top() + 10, 4, r.height() - 20), color_);
    }
    p.setPen(isChecked() ? color_ : theme::text2);
    p.setFont(theme::iconFont(30));
    p.drawText(QRectF(18, 14, 40, 36), Qt::AlignLeft | Qt::AlignVCenter, QString(glyph_));
    QFont title = theme::font(19, QFont::Black, true);
    title.setLetterSpacing(QFont::AbsoluteSpacing, 1.0);
    p.setFont(title);
    p.setPen(theme::text);
    p.drawText(QRectF(60, 14, width() - 70, 36), Qt::AlignLeft | Qt::AlignVCenter, title_);
    p.setFont(theme::font(13));
    p.setPen(theme::text2);
    p.drawText(QRectF(18, 56, width() - 34, height() - 64), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap,
      description_);
    if (running_) {
      QFont chip = theme::font(10, QFont::Bold);
      chip.setLetterSpacing(QFont::AbsoluteSpacing, 1.2);
      p.setFont(chip);
      QString text = "RUNNING";
      double w = p.fontMetrics().horizontalAdvance(text) + 16;
      QRectF box(width() - w - 12, 14, w, 20);
      p.setPen(Qt::NoPen);
      p.setBrush(color_);
      p.drawRoundedRect(box, 3, 3);
      p.setPen(color_.lightness() > 150 ? QColor(Qt::black) : theme::text);
      p.drawText(box, Qt::AlignCenter, text);
    }
  }

private:
  QChar glyph_;
  QString title_, description_;
  QColor color_;
  bool hover_ = false, running_ = false;
};

namespace
{
QColor componentColor(const std::string & status)
{
  if (status == "running") {return theme::green;}
  if (status == "starting" || status == "stopping") {return theme::yellow;}
  if (status == "failed") {return theme::redBright;}
  return theme::text3;
}

void fillPathCombo(QComboBox * combo, const std::vector<std::pair<QString, QString>> & paths)
{
  QString current = combo->currentData().toString();
  combo->blockSignals(true);
  combo->clear();
  combo->addItem("None", QString());
  for (const auto & [name, label] : paths) {
    combo->addItem(label, name);
  }
  int index = combo->findData(current);
  combo->setCurrentIndex(index >= 0 ? index : 0);
  combo->blockSignals(false);
}

void fillCombo(QComboBox * combo, const QStringList & names)
{
  QString current = combo->currentData().toString();
  combo->blockSignals(true);
  combo->clear();
  combo->addItem("None", QString());
  for (const auto & name : names) {
    combo->addItem(name, name);
  }
  int index = combo->findData(current);
  combo->setCurrentIndex(index >= 0 ? index : 0);
  combo->blockSignals(false);
}
}  // namespace

HomePage::HomePage(RosBridge * ros, QWidget * parent)
: Page(ros, parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(24, 18, 24, 18);
  root->setSpacing(14);

  // Header
  auto * hero = new QHBoxLayout();
  auto * titles = new QVBoxLayout();
  titles->setSpacing(0);
  auto * title = new QLabel("PIT WALL");
  QFont title_font = theme::font(40, QFont::Black, true);
  title_font.setLetterSpacing(QFont::AbsoluteSpacing, 2.0);
  title->setFont(title_font);
  titles->addWidget(title);
  titles->addWidget(makeLabel("Pick a session and a track, then go racing.", 14, QFont::Normal, theme::text2));
  hero->addLayout(titles);
  hero->addStretch(1);
  auto * car = new Card();
  car->setMinimumWidth(320);
  auto * car_row = new QHBoxLayout();
  link_led_ = new StatusLed(nullptr, 12);
  link_text_ = makeLabel("CAR OFFLINE", 16, QFont::Bold);
  car_row->addWidget(link_led_);
  car_row->addWidget(link_text_);
  car_row->addStretch(1);
  car->body()->addLayout(car_row);
  battery_text_ = makeLabel("Battery --", 13, QFont::Normal, theme::text2);
  battery_text_->setWordWrap(true);
  car->body()->addWidget(battery_text_);
  start_car_ = makeButton("START CAR SOFTWARE", theme::icon::power, "primary");
  start_car_->setVisible(false);
  car->body()->addWidget(start_car_);
  connect(start_car_, &QPushButton::clicked, this, &HomePage::startCarSoftware);
  hero->addWidget(car);
  root->addLayout(hero);

  auto * columns = new QHBoxLayout();
  columns->setSpacing(14);
  auto * left = new QVBoxLayout();
  left->setSpacing(14);

  auto * session = new Card("Session");
  auto * grid = new QGridLayout();
  grid->setSpacing(12);
  auto * group = new QButtonGroup(this);
  struct Spec
  {
    const char * mode;
    QChar glyph;
    const char * title;
    const char * text;
  };
  const Spec specs[] = {
    {"manual", theme::icon::gamepad, "MANUAL DRIVE",
      "Joystick driving with the safety systems on. Add a map and a path and the car can steer along the "
      "line for you, or race it on its own (A to engage, B to stop)."},
    {"mapping", theme::icon::radar, "MAPPING",
      "Build an occupancy map with SLAM while you drive around the track. Close the loop, then save it."},
    {"path", theme::icon::route, "PATH & RACELINE",
      "Localize on a saved map, record a lap, and optimize it into a minimum-curvature raceline."},
    {"idle", theme::icon::power, "STANDBY",
      "Stop everything on the car. Nothing runs until the next session; the car just waits for this UI."},
  };
  int i = 0;
  for (const auto & spec : specs) {
    auto * tile = new ModeTile(spec.glyph, spec.title, spec.text, theme::modeColor(spec.mode));
    group->addButton(tile);
    grid->addWidget(tile, i / 2, i % 2);
    tiles_[spec.mode] = tile;
    QString mode = spec.mode;
    connect(tile, &QAbstractButton::clicked, this, [this, mode]() {selectMode(mode);});
    ++i;
  }
  tiles_["manual"]->setChecked(true);
  session->body()->addLayout(grid);
  left->addWidget(session);

  auto * running = new Card("Running now");
  auto * running_row = new QHBoxLayout();
  running_row->setSpacing(12);
  running_mode_ = new SkewBadge();
  running_mode_->set("OFFLINE", theme::bg3);
  running_detail_ = makeLabel("Waiting for the car...", 13, QFont::Normal, theme::text2);
  running_row->addWidget(running_mode_);
  running_row->addWidget(running_detail_, 1);
  components_ = new QWidget();
  auto * chips = new QHBoxLayout(components_);
  chips->setContentsMargins(0, 0, 0, 0);
  chips->setSpacing(14);
  running_row->addWidget(components_);
  running->body()->addLayout(running_row);
  left->addWidget(running);

  // Joystick cheat sheet
  auto * controls = new Card("Controls");
  auto * keys = new QGridLayout();
  keys->setHorizontalSpacing(10);
  keys->setVerticalSpacing(7);
  const std::pair<const char *, const char *> mapping[] = {
    {"LB", "Hold to drive (deadman); also releases the kill switch"},
    {"L STICK", "Throttle and reverse"},
    {"R STICK", "Steering"},
    {"RT", "Brake (also blocks autonomous)"},
    {"LT", "Boost: up to the BOOST setting on DRIVE"},
    {"RB", "Kill switch: nothing drives until LB"},
    {"A", "Engage autonomous (map + path session)"},
    {"B", "Disengage autonomous"},
    {"X", "Collision brake on/off (buzz: short on, long off)"},
    {"Y", "Obstacle avoidance on/off"},
    {"D-PAD UP", "Steering assist on/off"},
    {"SPACE", "UI emergency stop (keyboard, any page)"},
  };
  int k = 0;
  for (const auto & [key, text] : mapping) {
    auto * chip = new QLabel(key);
    chip->setAlignment(Qt::AlignCenter);
    chip->setFont(theme::font(11, QFont::Black));
    chip->setFixedSize(64, 22);
    chip->setStyleSheet(QString("background: %1; border-radius: 4px; color: %2;")
      .arg(theme::css(QString(key) == "SPACE" ? theme::red : theme::bg3), theme::css(theme::text)));
    keys->addWidget(chip, k / 2, (k % 2) * 2);
    keys->addWidget(makeLabel(text, 12, QFont::Normal, theme::text2), k / 2, (k % 2) * 2 + 1);
    ++k;
  }
  keys->setColumnStretch(1, 1);
  keys->setColumnStretch(3, 1);
  controls->body()->addLayout(keys);
  controls->body()->addStretch(1);
  left->addWidget(controls, 1);
  columns->addLayout(left, 3);

  auto * track = new Card("Track");
  auto * form = new QGridLayout();
  form->setHorizontalSpacing(10);
  form->setVerticalSpacing(8);
  map_combo_ = new QComboBox();
  path_combo_ = new QComboBox();
  fillCombo(map_combo_, {});
  fillCombo(path_combo_, {});
  form->addWidget(makeCaption("Map"), 0, 0);
  form->addWidget(map_combo_, 0, 1);
  form->addWidget(makeCaption("Path"), 1, 0);
  form->addWidget(path_combo_, 1, 1);
  form->setColumnStretch(1, 1);
  track->body()->addLayout(form);
  preview_ = new TrackMap();
  preview_->setPlaceholder("No map selected\n\nFree driving needs no map. Make one in MAPPING.");
  track->body()->addWidget(preview_, 1);
  info_ = makeLabel("", 12, QFont::Normal, theme::text2);
  info_->setWordWrap(true);
  track->body()->addWidget(info_);
  hint_ = makeLabel("", 13, QFont::DemiBold, theme::text2);
  hint_->setWordWrap(true);
  track->body()->addWidget(hint_);
  start_ = makeButton("START SESSION", theme::icon::play, "primary");
  start_->setMinimumHeight(48);
  start_->setFont(theme::font(16, QFont::Black, true));
  track->body()->addWidget(start_);
  columns->addWidget(track, 2);
  root->addLayout(columns, 1);

  connect(map_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
      // Only paths made on this map can go with it
      path_map_filter_ = map_combo_->currentData().toString();
      fillPathCombo(path_combo_, pathsForMap(state_, path_map_filter_));
      refreshPreview();
      validate();
    });
  connect(path_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
      refreshPreview();
      validate();
    });
  connect(start_, &QPushButton::clicked, this, &HomePage::start);
  connect(ros_, &RosBridge::supervisorState, this, &HomePage::onSupervisor);
  connect(ros_, &RosBridge::linkChanged, this, &HomePage::onLink);
  connect(ros_, &RosBridge::telemetry, this, &HomePage::onTelemetry);
  validate();
}

void HomePage::selectMode(const QString & mode)
{
  mode_ = mode;
  if (tiles_.count(mode)) {
    tiles_[mode]->setChecked(true);
  }
  validate();
}

void HomePage::selectMap(const QString & map)
{
  int index = map_combo_->findData(map);
  if (index >= 0) {
    map_combo_->setCurrentIndex(index);
  }
}

void HomePage::selectPath(const QString & path)
{
  int index = path_combo_->findData(path);
  if (index >= 0) {
    path_combo_->setCurrentIndex(index);
  }
}

bool HomePage::isCar()
{
  // The car software is installed on this computer: the UI runs on the car itself
  try {
    ament_index_cpp::get_package_prefix("f1tenth_tools");
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

void HomePage::onLink(bool car, bool supervisor)
{
  link_led_->setColor(supervisor ? (car ? theme::green : theme::cyan) : theme::redBright);
  link_text_->setText(supervisor ? (car ? "CAR ONLINE" : "CAR ON STANDBY") : "CAR NOT FOUND");
  if (!supervisor) {
    battery_text_->setText(isCar() ?
      "The car software isn't running on this car." :
      "Waiting for the car. Is it on, on this network, with the car service installed?");
  } else if (!car) {
    battery_text_->setText("Connected. Nothing is running: start a session.");
  }
  start_car_->setVisible(!supervisor && isCar());
  validate();
}

void HomePage::startCarSoftware()
{
  start_car_->setEnabled(false);
  QTimer::singleShot(15000, start_car_, [this]() {start_car_->setEnabled(true);});
  // The car service if it's installed, otherwise the launch file directly (detached: it outlives the UI)
  if (QFile::exists("/etc/systemd/system/f1tenth.service")) {
    int code = QProcess::execute("sudo", {"-n", "systemctl", "start", "f1tenth.service"});
    Q_EMIT message(code == 0 ? Level::Ok : Level::Error, code == 0 ? "Starting the car service..." :
      "Couldn't start the car service. In a terminal: sudo systemctl start f1tenth");
    return;
  }
  QDir().mkpath(QDir::homePath() + "/.ros/f1tenth");
  bool ok = QProcess::startDetached("/bin/bash", {"-c",
      "exec setsid ros2 launch f1tenth_bringup car_launch.py >> \"$HOME/.ros/f1tenth/car_launch.log\" 2>&1 < /dev/null"});
  Q_EMIT message(ok ? Level::Ok : Level::Error, ok ? "Starting the car software (log: ~/.ros/f1tenth/car_launch.log)" :
    "Couldn't run ros2 launch f1tenth_bringup car_launch.py");
}

void HomePage::onTelemetry(const Telemetry & t)
{
  battery_text_->setText(t.vesc_connected ?
    QString("Battery %1%  ·  %2 V  ·  Joystick %3").arg(t.battery_percent, 0, 'f', 0)
    .arg(t.battery_voltage, 0, 'f', 2).arg(t.joystick_connected ? "connected" : "not connected") :
    "VESC not connected");
}

void HomePage::onSupervisor(const SupervisorState & state)
{
  QStringList maps;
  for (const auto & m : state.maps) {
    maps << QString::fromStdString(m);
  }
  if (maps != maps_) {
    maps_ = maps;
    fillCombo(map_combo_, maps_);
  }
  state_ = state;
  QString map_now = map_combo_->currentData().toString();
  QStringList listed;
  for (int i = 1; i < path_combo_->count(); ++i) {
    listed << path_combo_->itemText(i);
  }
  QStringList wanted;
  for (const auto & [name, label] : pathsForMap(state, map_now)) {
    wanted << label;
  }
  if (listed != wanted || map_now != path_map_filter_) {
    path_map_filter_ = map_now;
    fillPathCombo(path_combo_, pathsForMap(state, map_now));
  }
  QString mode = QString::fromStdString(state.mode);
  // Start from what the car is doing, and follow sessions started elsewhere (DRIVE THIS LAP, another pit wall).
  // Standby keeps the last picks, ready to start again.
  QString session = mode + "|" + QString::fromStdString(state.map) + "|" + QString::fromStdString(state.path);
  if (!have_state_ || (session != running_session_ && mode != "idle")) {
    if (mode != "idle") {
      selectMode(mode);
    }
    selectMap(QString::fromStdString(state.map));
    selectPath(QString::fromStdString(state.path));
  }
  running_session_ = session;
  for (auto & [key, tile] : tiles_) {
    tile->setRunning(key == mode);
  }
  running_mode_->set(theme::modeTitle(mode), theme::modeColor(mode),
    mode == "mapping" ? QColor(Qt::black) : QColor(Qt::white));
  QStringList detail;
  if (!state.map.empty()) {
    detail << "Map " + QString::fromStdString(state.map);
  }
  if (!state.path.empty()) {
    detail << "Path " + QString::fromStdString(state.path);
  }
  if (state.bag_recording) {
    detail << "Recording bag";
  }
  running_detail_->setText(detail.isEmpty() ? QString::fromStdString(state.message) : detail.join("  ·  "));

  // Component chips: LED + name
  auto * layout = components_->layout();
  QStringList names;
  for (const auto & c : state.components) {
    names << QString::fromStdString(c.name);
  }
  QStringList shown;
  for (int k = 0; k < layout->count(); ++k) {
    shown << layout->itemAt(k)->widget()->objectName();
  }
  if (shown != names) {
    while (auto * item = layout->takeAt(0)) {
      delete item->widget();
      delete item;
    }
    for (const auto & name : names) {
      auto * chip = new QWidget();
      chip->setObjectName(name);
      auto * row = new QHBoxLayout(chip);
      row->setContentsMargins(0, 0, 0, 0);
      row->setSpacing(4);
      auto * led = new StatusLed(nullptr, 9);
      led->setObjectName("led");
      row->addWidget(led);
      row->addWidget(makeLabel(name.toUpper(), 11, QFont::Bold, theme::text2));
      layout->addWidget(chip);
    }
  }
  for (const auto & c : state.components) {
    auto * chip = components_->findChild<QWidget *>(QString::fromStdString(c.name));
    if (auto * led = chip ? chip->findChild<StatusLed *>("led") : nullptr) {
      led->setColor(componentColor(c.status));
      led->setBlinking(c.status == "starting" || c.status == "failed");
      chip->setToolTip(QString("%1: %2, %3 restarts").arg(QString::fromStdString(c.name))
        .arg(QString::fromStdString(c.status)).arg(c.restarts));
    }
  }
  state_ = state;
  have_state_ = true;
  validate();
}

void HomePage::refreshPreview()
{
  QString map = map_combo_->currentData().toString();
  QString path = path_combo_->currentData().toString();
  if (map != preview_map_) {
    preview_map_ = map;
    if (map.isEmpty()) {
      preview_->clearMap();
    } else if (preview_cache_.count(map)) {
      const auto & cached = preview_cache_[map];
      preview_->setPreview(cached.image, cached.resolution, cached.origin_x, cached.origin_y);
    } else {
      preview_->clearMap();
      preview_->setPlaceholder("Loading " + map + "...");
      ros_->mapPreview(map, 640, [this, map](const MapPreview & preview) {
          if (!preview.ok) {
            preview_->setPlaceholder("Map preview failed: " + preview.message);
            return;
          }
          preview_cache_[map] = preview;
          if (map_combo_->currentData().toString() == map) {
            preview_->setPreview(preview.image, preview.resolution, preview.origin_x, preview.origin_y);
            info_->setText(QString("%1  ·  %2 x %3 m  ·  %4 m/px").arg(map)
              .arg(preview.image.width() * preview.resolution, 0, 'f', 1)
              .arg(preview.image.height() * preview.resolution, 0, 'f', 1).arg(preview.resolution, 0, 'f', 3));
          }
        });
    }
  }
  if (path != preview_path_) {
    preview_path_ = path;
    preview_->clearPath();
    if (!path.isEmpty()) {
      ros_->path(path, [this, path](const PathData & data) {
          if (path_combo_->currentData().toString() != path) {
            return;
          }
          if (!data.ok) {
            Q_EMIT message(Level::Error, data.message);
            return;
          }
          preview_->setPath(data.points, data.speeds);
          double top = 0.0;
          for (float v : data.speeds) {
            top = std::max(top, static_cast<double>(v));
          }
          info_->setText(QString("%1  ·  %2 m  ·  %3 points  ·  up to %4 m/s").arg(path)
            .arg(data.length(), 0, 'f', 1).arg(data.points.size()).arg(top, 0, 'f', 1));
        });
    }
  }
  if (map.isEmpty() && path.isEmpty()) {
    info_->setText("");
    preview_->setPlaceholder("No map selected\n\nFree driving needs no map. Make one in MAPPING.");
  }
}

void HomePage::validate()
{
  QString map = map_combo_->currentData().toString();
  QString path = path_combo_->currentData().toString();
  bool uses_track = mode_ == "manual" || mode_ == "path";
  map_combo_->setEnabled(uses_track);
  path_combo_->setEnabled(uses_track && !map.isEmpty() && path_combo_->count() > 1);
  bool ok = true;
  QString hint;
  QColor color = theme::text2;
  if (!ros_->supervisorLinked()) {
    ok = false;
    hint = isCar() ? "Start the car software first (button above)." :
      "Can't reach the car. Check it's on and on this network (see SYSTEM and the README).";
    color = theme::yellow;
  } else if (mode_ == "path" && map.isEmpty()) {
    ok = false;
    hint = "Path mode needs a map to localize on.";
    color = theme::yellow;
  } else if (mode_ == "manual" && !path.isEmpty() && map.isEmpty()) {
    ok = false;
    hint = "A path needs the map it was recorded on.";
    color = theme::yellow;
  } else if (!path.isEmpty() && pathMap(state_, path).isEmpty()) {
    hint = "Which map " + path + " was made on isn't recorded: make sure it is " + map + ".";
    color = theme::yellow;
  } else if (uses_track && !map.isEmpty() && path_combo_->count() <= 1) {
    hint = mode_ == "path" ? "Record a lap on " + map + " and optimize it into a raceline." :
      "No paths on " + map + " yet: make one in a PATH session.";
  } else if (mode_ == "manual") {
    hint = map.isEmpty() ? "Free driving: joystick with safety." :
      (path.isEmpty() ? "Localized driving on " + map + "." :
      "Steering assist, lap timing and AUTONOMOUS are available.");
  } else if (mode_ == "mapping") {
    hint = "SLAM builds a new map from scratch. Save it on the MAPPING panel.";
  } else if (mode_ == "path") {
    hint = "Set the car's pose on the map, then record and optimize a lap.";
  } else {
    hint = "Stops every node on the car: motor, sensors and joystick are off.";
  }
  bool same = have_state_ && QString::fromStdString(state_.mode) == mode_ &&
    (!uses_track || (QString::fromStdString(state_.map) == map && QString::fromStdString(state_.path) == path));
  start_->setText(same ? (mode_ == "idle" ? "ON STANDBY" : "SESSION RUNNING") :
    (mode_ == "idle" ? "STOP EVERYTHING" : "START SESSION"));
  start_->setEnabled(ok && !same);
  hint_->setText(hint);
  hint_->setStyleSheet(QString("color: %1;").arg(theme::css(color)));
}

void HomePage::start()
{
  bool uses_track = mode_ == "manual" || mode_ == "path";
  QString map = uses_track ? map_combo_->currentData().toString() : QString();
  QString path = uses_track ? path_combo_->currentData().toString() : QString();
  start_->setEnabled(false);
  start_->setText("STARTING...");
  QString mode = mode_;
  ros_->setMode(mode, map, path, [this, mode](bool ok, const QString & text) {
      Q_EMIT message(ok ? Level::Ok : Level::Error, text);
      validate();
      if (ok) {
        Q_EMIT sessionStarted(mode);
      }
    });
}
}  // namespace f1ui
