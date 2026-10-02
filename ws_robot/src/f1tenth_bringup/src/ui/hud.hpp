// Driver instruments, drawn like an F1 steering wheel display and TV graphics.
#pragma once

#include <QColor>
#include <QPointF>
#include <QString>
#include <QVector>
#include <QWidget>

#include <algorithm>
#include <deque>

#include "ros_bridge.hpp"

class QTimer;

namespace f1ui
{
// Speedometer arc with the commanded speed, gear letter and the LT boost (shown like DRS)
class SpeedGauge : public QWidget
{
  Q_OBJECT

public:
  explicit SpeedGauge(QWidget * parent = nullptr);
  void setSpeed(double speed, double command);  // m/s
  void setMaxSpeed(double speed) {max_speed_ = std::max(1.0, speed); update();}
  void setBoost(double factor) {boost_ = factor; update();}
  void setActive(bool active) {active_ = active; update();}
  bool kmh() const {return kmh_;}
  void setKmh(bool kmh) {kmh_ = kmh; update();}
  QSize sizeHint() const override {return QSize(220, 190);}

Q_SIGNALS:
  void unitsToggled(bool kmh);

protected:
  void paintEvent(QPaintEvent *) override;
  void mousePressEvent(QMouseEvent *) override;

private:
  double speed_ = 0.0, command_ = 0.0, max_speed_ = 10.0, boost_ = 1.0;
  bool kmh_ = true, active_ = false;
};

// Throttle and brake as vertical bars (0..1)
class PedalBars : public QWidget
{
  Q_OBJECT

public:
  explicit PedalBars(QWidget * parent = nullptr);
  void set(double throttle, double brake, bool reverse) {throttle_ = throttle; brake_ = brake; reverse_ = reverse; update();}
  QSize sizeHint() const override {return QSize(96, 170);}

protected:
  void paintEvent(QPaintEvent *) override;

private:
  double throttle_ = 0.0, brake_ = 0.0;
  bool reverse_ = false;
};

// F1 steering wheel turning with the front wheels (scaled up so small angles are visible)
class SteeringWheel : public QWidget
{
  Q_OBJECT

public:
  explicit SteeringWheel(QWidget * parent = nullptr);
  void set(double angle, double command, double max_angle);  // rad
  QSize sizeHint() const override {return QSize(190, 170);}

protected:
  void paintEvent(QPaintEvent *) override;

private:
  double angle_ = 0.0, command_ = 0.0, max_angle_ = 0.4;
};

// Rev lights across the top of the wheel: green, red, blue; flashing at the limit
class ShiftLights : public QWidget
{
  Q_OBJECT

public:
  explicit ShiftLights(QWidget * parent = nullptr);
  void setFraction(double fraction);
  QSize sizeHint() const override {return QSize(320, 22);}

protected:
  void paintEvent(QPaintEvent *) override;

private:
  double fraction_ = 0.0;
  bool flash_ = false;
  QTimer * timer_ = nullptr;
};

class BatteryGauge : public QWidget
{
  Q_OBJECT

public:
  explicit BatteryGauge(QWidget * parent = nullptr);
  void set(double percent, double voltage, int cells, double current, bool connected);
  QSize sizeHint() const override {return QSize(170, 120);}

protected:
  void paintEvent(QPaintEvent *) override;

private:
  double percent_ = 0.0, voltage_ = 0.0, current_ = 0.0;
  int cells_ = 3;
  bool connected_ = false;
};

// Lateral/longitudinal acceleration with a fading trace
class GForceMeter : public QWidget
{
  Q_OBJECT

public:
  explicit GForceMeter(QWidget * parent = nullptr);
  void addSample(double lateral, double longitudinal);  // g
  void setRange(double g) {range_ = g; update();}
  QSize sizeHint() const override {return QSize(150, 150);}

protected:
  void paintEvent(QPaintEvent *) override;

private:
  std::deque<QPointF> trace_;
  double range_ = 1.5;
};

// Lap number, current/last/best times, sectors (purple/green/yellow like F1 timing) and track progress
class TimingTower : public QWidget
{
  Q_OBJECT

public:
  explicit TimingTower(QWidget * parent = nullptr);
  void setLap(const LapState & lap);
  void reset();
  QSize sizeHint() const override {return QSize(320, 250);}
  QSize minimumSizeHint() const override {return QSize(240, 210);}

protected:
  void paintEvent(QPaintEvent *) override;

private:
  LapState lap_;
  bool have_lap_ = false;
  uint32_t last_lap_count_ = 0;
  double sector_start_ = 0.0;
  int sector_ = 0;                 // sector the car is in (0..2)
  double current_[3] = {0, 0, 0};  // this lap's sector times (0 = not yet)
  double previous_[3] = {0, 0, 0};
  double best_[3] = {0, 0, 0};
  double previous_best_lap_ = 0.0;
  QColor sector_color_[3];
};

// Row of indicator chips (deadman, kill switch, locks, ...)
class FlagStrip : public QWidget
{
  Q_OBJECT

public:
  explicit FlagStrip(QWidget * parent = nullptr);
  void setFlag(const QString & key, const QString & text, const QColor & color, bool lit);
  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent *) override;

private:
  struct Flag
  {
    QString key, text;
    QColor color;
    bool lit;
  };
  QVector<Flag> flags_;
};
}  // namespace f1ui
