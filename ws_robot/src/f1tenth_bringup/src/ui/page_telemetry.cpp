#include <QButtonGroup>
#include <QElapsedTimer>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

#include <algorithm>
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
double seconds()
{
  static QElapsedTimer clock = []() {
      QElapsedTimer t;
      t.start();
      return t;
    }();
  return clock.elapsed() / 1000.0;
}

QString lapTime(double s)
{
  int minutes = static_cast<int>(s / 60.0);
  return QString("%1:%2").arg(minutes).arg(s - minutes * 60.0, 6, 'f', 3, QChar('0'));
}
}  // namespace

TelemetryPage::TelemetryPage(RosBridge * ros, QWidget * parent)
: Page(ros, parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(24, 18, 24, 18);
  root->setSpacing(14);

  auto * bar = new QHBoxLayout();
  auto * title = new QLabel("TELEMETRY");
  QFont title_font = theme::font(28, QFont::Black, true);
  title_font.setLetterSpacing(QFont::AbsoluteSpacing, 1.5);
  title->setFont(title_font);
  bar->addWidget(title);
  bar->addStretch(1);
  bar->addWidget(makeCaption("Window"));
  bar->addSpacing(6);
  auto * windows = new QButtonGroup(this);
  for (int s : {10, 30, 60, 120}) {
    auto * b = makeButton(QString("%1 S").arg(s), QChar(), "seg");
    b->setCheckable(true);
    b->setChecked(s == 30);
    windows->addButton(b, s);
    bar->addWidget(b);
  }
  bar->addSpacing(12);
  auto * pause = makeButton("PAUSE", theme::icon::pause);
  pause->setCheckable(true);
  auto * clear = makeButton("CLEAR", theme::icon::trash);
  bar->addWidget(pause);
  bar->addWidget(clear);
  root->addLayout(bar);

  auto * body = new QHBoxLayout();
  body->setSpacing(14);
  auto * charts_card = new Card();
  auto * grid = new QGridLayout();
  grid->setSpacing(16);
  speed_ = new StripChart("Speed", "m/s");
  speed_actual_ = speed_->addSeries("ACT", theme::text);
  speed_cmd_ = speed_->addSeries("CMD", theme::cyan);
  steering_ = new StripChart("Steering", "°");
  steer_actual_ = steering_->addSeries("ACT", theme::text);
  steer_cmd_ = steering_->addSeries("CMD", theme::cyan);
  current_ = new StripChart("Current", "A");
  motor_ = current_->addSeries("MOTOR", theme::orange);
  input_ = current_->addSeries("BATT", theme::yellow);
  brake_ = current_->addSeries("BRAKE", theme::redBright);
  battery_ = new StripChart("Battery", "V");
  volts_ = battery_->addSeries("PACK", theme::green);
  duty_ = new StripChart("Duty cycle", "%");
  duty_series_ = duty_->addSeries("DUTY", theme::purple);
  duty_->setRange(-100, 100);
  temps_ = new StripChart("Temperature", "°C");
  fet_ = temps_->addSeries("ESC", theme::orange);
  motor_temp_ = temps_->addSeries("MOTOR", theme::redBright);
  // Small sensor noise shouldn't fill a whole chart
  steering_->setRange(-30, 30);
  speed_->setMinSpan(2.0);
  current_->setMinSpan(10.0);
  battery_->setMinSpan(1.0);
  temps_->setMinSpan(10.0);
  grid->addWidget(speed_, 0, 0);
  grid->addWidget(steering_, 0, 1);
  grid->addWidget(current_, 1, 0);
  grid->addWidget(battery_, 1, 1);
  grid->addWidget(duty_, 2, 0);
  grid->addWidget(temps_, 2, 1);
  charts_card->body()->addLayout(grid);
  body->addWidget(charts_card, 1);

  auto * side = new QVBoxLayout();
  side->setSpacing(14);
  auto * g_card = new Card("G-force");
  g_meter_ = new GForceMeter();
  g_meter_->setMinimumSize(220, 220);
  g_meter_->setRange(1.5);
  g_card->body()->addWidget(g_meter_, 0, Qt::AlignHCenter);
  side->addWidget(g_card);
  auto * stats = new Card("This session");
  auto * tiles = new QGridLayout();
  tiles->setHorizontalSpacing(16);
  tiles->setVerticalSpacing(10);
  top_speed_ = new ValueTile("Top speed", "m/s");
  distance_ = new ValueTile("Distance", "m");
  energy_ = new ValueTile("Energy", "Wh");
  peak_current_ = new ValueTile("Peak current", "A");
  tiles->addWidget(top_speed_, 0, 0);
  tiles->addWidget(distance_, 0, 1);
  tiles->addWidget(energy_, 1, 0);
  tiles->addWidget(peak_current_, 1, 1);
  stats->body()->addLayout(tiles);
  side->addWidget(stats);
  auto * laps = new Card("Laps");
  laps_ = new QTableWidget(0, 3);
  laps_->setHorizontalHeaderLabels({"LAP", "TIME", "GAP"});
  laps_->verticalHeader()->hide();
  laps_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  laps_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  laps_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  laps_->setSelectionMode(QAbstractItemView::NoSelection);
  laps_->setFont(theme::font(14, QFont::Bold));
  laps_->setShowGrid(false);
  laps->body()->addWidget(laps_);
  side->addWidget(laps, 1);
  auto * side_widget = new QWidget();
  side_widget->setLayout(side);
  side_widget->setFixedWidth(330);
  body->addWidget(side_widget);
  root->addLayout(body, 1);

  connect(windows, QOverload<int>::of(&QButtonGroup::idClicked), this, [this](int s) {
      for (auto * chart : {speed_, steering_, current_, battery_, duty_, temps_}) {
        chart->setWindow(s);
      }
    });
  connect(pause, &QPushButton::toggled, this, [this, pause](bool on) {
      paused_ = on;
      pause->setText(on ? "RESUME" : "PAUSE");
      pause->setIcon(glyphIcon(on ? theme::icon::play : theme::icon::pause, theme::text, 16));
      for (auto * chart : {speed_, steering_, current_, battery_, duty_, temps_}) {
        chart->setPaused(on);
      }
    });
  connect(clear, &QPushButton::clicked, this, [this]() {
      for (auto * chart : {speed_, steering_, current_, battery_, duty_, temps_}) {
        chart->clear();
      }
      top_speed_value_ = peak_current_value_ = 0.0;
      start_distance_ = start_energy_ = -1.0;
    });
  connect(ros_, &RosBridge::telemetry, this, &TelemetryPage::onTelemetry);
  connect(ros_, &RosBridge::lapState, this, &TelemetryPage::onLap);
  connect(ros_, &RosBridge::imu, this, [this](double ax, double ay, double) {
      g_meter_->addSample(-ay / 9.81, -ax / 9.81);
    });
}

void TelemetryPage::onTelemetry(const Telemetry & t)
{
  if (!t.vesc_connected) {
    return;
  }
  double now = seconds();
  speed_->addPoint(speed_actual_, now, t.speed);
  speed_->addPoint(speed_cmd_, now, t.speed_command);
  steering_->addPoint(steer_actual_, now, t.steering_angle * 180.0 / M_PI);
  steering_->addPoint(steer_cmd_, now, t.steering_command * 180.0 / M_PI);
  current_->addPoint(motor_, now, t.motor_current);
  current_->addPoint(input_, now, t.input_current);
  current_->addPoint(brake_, now, std::abs(t.brake_current));
  battery_->addPoint(volts_, now, t.battery_voltage);
  duty_->addPoint(duty_series_, now, t.duty_cycle * 100.0);
  temps_->addPoint(fet_, now, t.temp_fet);
  if (t.temp_motor > 0.0f) {
    temps_->addPoint(motor_temp_, now, t.temp_motor);
  }
  if (isVisible()) {
    for (auto * chart : {speed_, steering_, current_, battery_, duty_, temps_}) {
      chart->update();
    }
  }
  if (paused_) {
    return;
  }
  top_speed_value_ = std::max(top_speed_value_, static_cast<double>(std::abs(t.speed)));
  peak_current_value_ = std::max(peak_current_value_, static_cast<double>(std::abs(t.motor_current)));
  if (start_distance_ < 0.0 || t.distance < start_distance_) {
    start_distance_ = t.distance;
    start_energy_ = t.energy_drawn;
  }
  top_speed_->setValue(QString::number(top_speed_value_, 'f', 2));
  top_speed_->setDetail(QString("%1 km/h").arg(top_speed_value_ * 3.6, 0, 'f', 1));
  distance_->setValue(QString::number(t.distance - start_distance_, 'f', 0));
  energy_->setValue(QString::number(std::max(0.0, t.energy_drawn - start_energy_), 'f', 2));
  peak_current_->setValue(QString::number(peak_current_value_, 'f', 1));
}

void TelemetryPage::onLap(const LapState & lap)
{
  if (lap.lap < lap_count_) {
    laps_->setRowCount(0);  // lap times were reset
    best_lap_ = 0.0;
  }
  if (lap.lap > lap_count_ && lap.last_lap_time > 0.0f) {
    double time = lap.last_lap_time;
    bool best = best_lap_ <= 0.0 || time <= best_lap_;
    double gap = best_lap_ > 0.0 ? time - best_lap_ : 0.0;
    if (best) {
      best_lap_ = time;
    }
    laps_->insertRow(0);
    auto * number = new QTableWidgetItem(QString::number(lap.lap));
    auto * time_item = new QTableWidgetItem(lapTime(time));
    auto * gap_item = new QTableWidgetItem(best ? "BEST" : QString("+%1").arg(gap, 0, 'f', 3));
    time_item->setForeground(best ? theme::purple : theme::text);
    gap_item->setForeground(best ? theme::purple : theme::yellow);
    number->setForeground(theme::text3);
    laps_->setItem(0, 0, number);
    laps_->setItem(0, 1, time_item);
    laps_->setItem(0, 2, gap_item);
  }
  lap_count_ = lap.lap;
}
}  // namespace f1ui
