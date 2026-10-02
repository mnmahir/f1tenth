#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <map>

#include "pages.hpp"
#include "theme.hpp"
#include "widgets.hpp"

namespace f1ui
{
namespace
{
QColor statusColor(const std::string & status)
{
  if (status == "running") {return theme::green;}
  if (status == "starting" || status == "stopping") {return theme::yellow;}
  if (status == "failed") {return theme::redBright;}
  return theme::text3;
}

QString uptime(double s)
{
  if (s <= 0.0) {
    return "--";
  }
  int total = static_cast<int>(s);
  if (total < 3600) {
    return QString("%1:%2").arg(total / 60).arg(total % 60, 2, 10, QChar('0'));
  }
  return QString("%1h %2m").arg(total / 3600).arg((total % 3600) / 60);
}

QTableWidgetItem * item(const QString & text, const QColor & color = theme::text, bool bold = false)
{
  auto * it = new QTableWidgetItem(text);
  it->setForeground(color);
  if (bold) {
    it->setFont(theme::font(13, QFont::Bold));
  }
  return it;
}

QString value(const diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key)
{
  for (const auto & kv : status.values) {
    if (kv.key == key) {
      return QString::fromStdString(kv.value);
    }
  }
  return QString();
}

// LED centred in a table cell
StatusLed * cellLed(QTableWidget * table, int row, int column)
{
  if (auto * holder = table->cellWidget(row, column)) {
    return holder->findChild<StatusLed *>();
  }
  auto * holder = new QWidget();
  auto * layout = new QHBoxLayout(holder);
  layout->setContentsMargins(8, 0, 4, 0);
  auto * led = new StatusLed(nullptr, 9);
  layout->addWidget(led, 0, Qt::AlignCenter);
  table->setCellWidget(row, column, holder);
  return led;
}

bool internalNode(const QString & node)
{
  return node.startsWith("/_") || node.contains("/_ros2cli") || node.startsWith("/launch_ros_") ||
         node.startsWith("/transform_listener_impl_");
}

QTableWidget * makeTable(const QStringList & headers)
{
  auto * table = new QTableWidget(0, headers.size());
  table->setHorizontalHeaderLabels(headers);
  table->verticalHeader()->hide();
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setSelectionMode(QAbstractItemView::NoSelection);
  table->setShowGrid(false);
  table->setAlternatingRowColors(true);
  table->setFocusPolicy(Qt::NoFocus);
  table->verticalHeader()->setDefaultSectionSize(32);
  return table;
}
}  // namespace

SystemPage::SystemPage(RosBridge * ros, QWidget * parent)
: Page(ros, parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(24, 18, 24, 18);
  root->setSpacing(14);

  auto * supervisor = new Card("Supervisor");
  auto * sup_row = new QHBoxLayout();
  supervisor_led_ = new StatusLed(nullptr, 12);
  supervisor_text_ = makeLabel("Waiting for the car...", 15, QFont::Bold);
  auto_restart_ = new Toggle("Auto-restart crashed parts");
  bag_ = new Toggle("Record rosbag");
  bag_->setAccent(theme::redBright);
  sup_row->addWidget(supervisor_led_);
  sup_row->addWidget(supervisor_text_, 1);
  sup_row->addWidget(auto_restart_);
  sup_row->addSpacing(18);
  sup_row->addWidget(bag_);
  sup_row->addSpacing(18);
  auto * reboot = makeButton("REBOOT CAR", theme::icon::restart);
  auto * poweroff = makeButton("POWER OFF CAR", theme::icon::power);
  sup_row->addWidget(reboot);
  sup_row->addWidget(poweroff);
  auto power = [this](const QString & action, const QString & title, const QString & text) {
      if (QMessageBox::question(this, title, text) == QMessageBox::Yes) {
        ros_->componentCommand("car", action, report());
      }
    };
  connect(reboot, &QPushButton::clicked, this, [power]() {
      power("reboot", "Reboot the car?", "Stops the session and restarts the car's computer. The UI reconnects by "
      "itself in about a minute.");
    });
  connect(poweroff, &QPushButton::clicked, this, [power]() {
      power("poweroff", "Power off the car?", "Stops the session and shuts the car's computer down. Wait for its "
      "lights to go off before unplugging the battery.");
    });
  supervisor->body()->addLayout(sup_row);
  root->addWidget(supervisor);

  auto * middle = new QHBoxLayout();
  middle->setSpacing(14);
  auto * components = new Card("Components");
  components_ = makeTable({"", "COMPONENT", "STATUS", "UPTIME", "RESTARTS", "MISSING NODES", ""});
  components_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  components_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);
  components->body()->addWidget(components_);
  middle->addWidget(components, 3);
  auto * health = new Card("Car health");
  tile_grid_ = new QGridLayout();
  tile_grid_->setHorizontalSpacing(18);
  tile_grid_->setVerticalSpacing(12);
  const std::pair<const char *, const char *> tiles[] = {{"cpu", "CPU"}, {"gpu", "GPU"}, {"power", "Power"},
    {"memory", "Memory"}, {"disk", "Disk"}, {"wifi", "Wi-Fi"}, {"thermal", "Hottest"}};
  for (const auto & [key, caption] : tiles) {
    auto * tile = new ValueTile(caption, QString(), health);
    tile->setValuePixelSize(24);
    // A car reports one of these: the GPU on a Jetson, the power supply on a Raspberry Pi
    tile->setVisible(QString(key) != "gpu" && QString(key) != "power");
    tiles_[key] = tile;
    tile_order_.push_back(key);
  }
  layoutTiles();
  health->body()->addLayout(tile_grid_);
  health->body()->addStretch(1);
  middle->addWidget(health, 2);
  root->addLayout(middle, 3);

  auto * bottom = new QHBoxLayout();
  bottom->setSpacing(14);
  auto * rates = new Card("Sensors & diagnostics");
  rates_ = makeTable({"", "SOURCE", "STATUS", "EXPECTED"});
  rates_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  rates_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  rates->body()->addWidget(rates_);
  bottom->addWidget(rates, 3);
  auto * nodes = new Card("ROS nodes");
  nodes_count_ = makeLabel("", 12, QFont::Bold, theme::text2);
  nodes->header()->addWidget(nodes_count_);
  nodes_ = new QListWidget();
  nodes_->setFont(theme::mono(12));
  nodes->body()->addWidget(nodes_);
  bottom->addWidget(nodes, 2);
  root->addLayout(bottom, 2);

  connect(auto_restart_, &Toggle::clicked, this, [this](bool on) {
      auto_restart_->setPending(true);
      ros_->setAutoRestart(on, [this](bool ok, const QString & text) {
        auto_restart_->setPending(false);
        Q_EMIT message(ok ? Level::Ok : Level::Error, text);
      });
    });
  connect(bag_, &Toggle::clicked, this, [this](bool on) {
      bag_->setPending(true);
      ros_->setBagRecording(on, [this](bool ok, const QString & text) {
        bag_->setPending(false);
        Q_EMIT message(ok ? Level::Ok : Level::Error, text);
      });
    });
  connect(ros_, &RosBridge::supervisorState, this, &SystemPage::onSupervisor);
  connect(ros_, &RosBridge::diagnostics, this, &SystemPage::onDiagnostics);
  connect(ros_, &RosBridge::nodesChanged, this, &SystemPage::onNodes);
  connect(ros_, &RosBridge::linkChanged, this, [this](bool, bool supervisor) {
      if (!supervisor) {
        supervisor_led_->setColor(theme::redBright);
        supervisor_text_->setText("Supervisor not reachable. On the car: ros2 launch f1tenth_bringup car_launch.py");
        component_level_ = 2;
        updateHealth();
      }
    });
}

void SystemPage::onSupervisor(const SupervisorState & state)
{
  standby_ = state.mode == "idle";
  supervisor_led_->setColor(theme::green);
  supervisor_text_->setText(QString("%1  ·  %2").arg(theme::modeTitle(QString::fromStdString(state.mode)))
    .arg(QString::fromStdString(state.message)));
  if (!auto_restart_->isDown()) {
    auto_restart_->setChecked(state.auto_restart);
  }
  if (!bag_->isDown()) {
    bag_->setChecked(state.bag_recording);
  }

  if (components_->rowCount() != static_cast<int>(state.components.size())) {
    components_->setRowCount(static_cast<int>(state.components.size()));
  }
  int worst = 0;
  missing_.clear();
  for (int row = 0; row < static_cast<int>(state.components.size()); ++row) {
    const auto & c = state.components[row];
    QString name = QString::fromStdString(c.name);
    QColor color = statusColor(c.status);
    auto * led = cellLed(components_, row, 0);
    led->setColor(color);
    led->setBlinking(c.status == "starting" || c.status == "failed");
    components_->setItem(row, 1, item(name.toUpper(), theme::text, true));
    components_->setItem(row, 2, item(QString::fromStdString(c.status).toUpper(), color, true));
    components_->setItem(row, 3, item(uptime(c.uptime), theme::text2));
    components_->setItem(row, 4, item(QString::number(c.restarts), c.restarts > 0 ? theme::yellow : theme::text2));
    QStringList missing;
    for (const auto & n : c.missing_nodes) {
      missing << QString::fromStdString(n);
      missing_ << QString::fromStdString(n);
    }
    for (const auto & n : c.unresponsive_nodes) {
      missing << QString::fromStdString(n) + " (not responding)";
    }
    components_->setItem(row, 5, item(missing.join("  "), theme::redBright));
    auto * actions = components_->cellWidget(row, 6);
    if (!actions || actions->objectName() != name) {
      actions = new QWidget();
      actions->setObjectName(name);
      auto * layout = new QHBoxLayout(actions);
      layout->setContentsMargins(4, 0, 4, 0);
      layout->setSpacing(2);
      const std::pair<QChar, const char *> buttons[] = {
        {theme::icon::play, "start"}, {theme::icon::stop, "stop"}, {theme::icon::restart, "restart"}};
      for (const auto & [glyph, action] : buttons) {
        auto * b = makeButton("", glyph, "ghost");
        b->setToolTip(QString("%1 %2").arg(action).arg(name));
        QString act = action;
        connect(b, &QPushButton::clicked, this, [this, name, act]() {
            if (name == "core" && act != "start" && QMessageBox::question(this, act + " core?",
              "The core runs the sensors, the VESC, the joystick and the safety systems. The car stops while it "
              "restarts.") != QMessageBox::Yes)
            {
              return;
            }
            ros_->componentCommand(name, act, report());
          });
        layout->addWidget(b);
      }
      components_->setCellWidget(row, 6, actions);
    }
    if (c.status == "failed") {
      worst = std::max(worst, 2);
    } else if (!c.missing_nodes.empty() || !c.unresponsive_nodes.empty() || c.status == "starting") {
      worst = std::max(worst, 1);
    }
  }
  component_level_ = worst;
  updateHealth();
  onNodes(graph_);
}

// Two columns of the tiles this car reports, without gaps
void SystemPage::layoutTiles()
{
  int i = 0;
  for (const auto & key : tile_order_) {
    ValueTile * tile = tiles_[key];
    tile_grid_->removeWidget(tile);
    if (!tile->isHidden()) {
      tile_grid_->addWidget(tile, i / 2, i % 2);
      ++i;
    }
  }
}

void SystemPage::onDiagnostics(const DiagnosticArray & diagnostics)
{
  int worst = 0;
  for (const auto & status : diagnostics.status) {
    QString name = QString::fromStdString(status.name);
    QString message = QString::fromStdString(status.message);
    int level = status.level;
    if (message.startsWith("No events recorded")) {
      level = 3;  // a frequency monitor that never saw an event: no data rather than an error
      message = "no data";
    }
    QColor color = theme::levelColor(level);
    if (name.startsWith("f1tenth/") && !name.startsWith("f1tenth/topic/")) {
      QString key = name.mid(8);
      auto it = tiles_.find(key);
      if (it == tiles_.end()) {
        continue;
      }
      ValueTile * tile = it->second;
      if (tile->isHidden()) {
        tile->setVisible(true);
        layoutTiles();
      }
      QString usage = value(status, "usage");
      QString temperature = value(status, "temperature");
      if (key == "cpu" || key == "gpu") {
        tile->setValue(usage.isEmpty() ? "--" : QString::number(usage.toDouble(), 'f', 0) + "%", color);
        tile->setDetail(temperature.isEmpty() ? QString() : QString::number(temperature.toDouble(), 'f', 0) + " °C");
        tile->setBar(usage.toDouble() / 100.0, color);
      } else if (key == "memory" || key == "disk") {
        tile->setValue(QString::number(usage.toDouble(), 'f', 0) + "%", color);
        tile->setDetail(message);
        tile->setBar(usage.toDouble() / 100.0, color);
      } else if (key == "power") {
        tile->setValue(message, color);
        tile->setDetail(value(status, "supply"));
        tile->setBar(-1.0, color);
      } else if (key == "wifi") {
        QString dbm = value(status, "signal_dbm");
        tile->setValue(dbm.isEmpty() ? "--" : dbm + " dBm", color);
        tile->setDetail(value(status, "interface"));
        tile->setBar(dbm.isEmpty() ? 0.0 : std::clamp((dbm.toDouble() + 90.0) / 60.0, 0.0, 1.0), color);
      } else {
        tile->setValue(message.replace("max ", ""), color);
        tile->setBar(-1.0, color);
      }
      worst = std::max(worst, level == 3 ? 0 : level);
      continue;
    }
    // Topic rates and any other node's diagnostics
    if (standby_ && name.startsWith("f1tenth/topic/") && level == 2) {
      color = theme::levelColor(3);  // nothing is meant to run in standby: no data isn't a fault
      message = "standby";
    }
    QString source = name.startsWith("f1tenth/topic/") ? name.mid(13) : name;
    QString expected = value(status, "expected");
    int row = -1;
    for (int r = 0; r < rates_->rowCount(); ++r) {
      if (rates_->item(r, 1) && rates_->item(r, 1)->text() == source) {
        row = r;
        break;
      }
    }
    if (row < 0) {
      row = rates_->rowCount();
      rates_->insertRow(row);
    }
    cellLed(rates_, row, 0)->setColor(color);
    rates_->setItem(row, 1, item(source, theme::text, true));
    rates_->setItem(row, 2, item(message, color));
    rates_->setItem(row, 3, item(expected.isEmpty() || expected == "0.0" ? "" : expected + " Hz", theme::text3));
    if (level == 2 && name.startsWith("f1tenth/topic/") && !standby_) {
      worst = std::max(worst, 1);  // a silent sensor is a warning for the car's health
    }
  }
  diagnostic_level_ = worst;
  updateHealth();
}

void SystemPage::onNodes(const QStringList & nodes)
{
  graph_ = nodes;
  nodes_->clear();
  for (const auto & missing : missing_) {
    auto * it = new QListWidgetItem(glyphIcon(theme::icon::error, theme::redBright, 14), missing + "   (missing)");
    it->setForeground(theme::redBright);
    nodes_->addItem(it);
  }
  int count = 0;
  for (const auto & node : nodes) {
    if (internalNode(node)) {
      continue;
    }
    auto * it = new QListWidgetItem(glyphIcon(theme::icon::hub, node.contains("f1tenth_ui") ? theme::cyan :
      theme::green, 14), node);
    nodes_->addItem(it);
    ++count;
  }
  nodes_count_->setText(QString("%1 RUNNING%2").arg(count)
    .arg(missing_.isEmpty() ? QString() : QString("  ·  %1 MISSING").arg(missing_.size())));
}

void SystemPage::updateHealth()
{
  Q_EMIT healthChanged(std::max(component_level_, diagnostic_level_));
}
}  // namespace f1ui
