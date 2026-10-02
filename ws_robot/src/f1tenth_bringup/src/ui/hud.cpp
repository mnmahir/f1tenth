#include "hud.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QTimer>

#include <algorithm>
#include <cmath>

#include "theme.hpp"

namespace f1ui
{
namespace
{
QColor mix(const QColor & a, const QColor & b, double t)
{
  t = std::clamp(t, 0.0, 1.0);
  return QColor::fromRgbF(a.redF() + (b.redF() - a.redF()) * t, a.greenF() + (b.greenF() - a.greenF()) * t,
    a.blueF() + (b.blueF() - a.blueF()) * t, a.alphaF() + (b.alphaF() - a.alphaF()) * t);
}

QString lapTime(double seconds)
{
  if (seconds <= 0.0) {
    return "-:--.---";
  }
  int minutes = static_cast<int>(seconds / 60.0);
  double rest = seconds - minutes * 60.0;
  return QString("%1:%2").arg(minutes).arg(rest, 6, 'f', 3, QChar('0'));
}

QFont spaced(int size, int weight, double spacing = 1.2)
{
  QFont f = theme::font(size, weight);
  f.setLetterSpacing(QFont::AbsoluteSpacing, spacing);
  return f;
}
}  // namespace

// ---- SpeedGauge ----

SpeedGauge::SpeedGauge(QWidget * parent)
: QWidget(parent)
{
  setCursor(Qt::PointingHandCursor);
  setToolTip("Click to switch km/h and m/s");
  setMinimumSize(180, 160);
}

void SpeedGauge::setSpeed(double speed, double command)
{
  speed_ = speed;
  command_ = command;
  update();
}

void SpeedGauge::mousePressEvent(QMouseEvent *)
{
  kmh_ = !kmh_;
  update();
  Q_EMIT unitsToggled(kmh_);
}

void SpeedGauge::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  double r = std::min(width() / 2.0 - 14.0, (height() - 16.0) / 1.86);
  QPointF c(width() / 2.0, r + 12.0);
  QRectF arc(c.x() - r, c.y() - r, 2 * r, 2 * r);
  const double start = 225.0, sweep = 270.0;
  double thickness = std::max(6.0, r * 0.11);

  p.setPen(QPen(theme::bg3, thickness, Qt::SolidLine, Qt::FlatCap));
  p.drawArc(arc, static_cast<int>(start * 16), static_cast<int>(-sweep * 16));

  // Value arc in short segments, white turning red near the top speed
  double fraction = std::clamp(std::abs(speed_) / max_speed_, 0.0, 1.0);
  const int segments = 90;
  int lit = static_cast<int>(std::round(fraction * segments));
  for (int i = 0; i < lit; ++i) {
    double t = static_cast<double>(i) / segments;
    QColor color = active_ ? mix(theme::text, theme::redBright, std::pow(t, 1.6)) : theme::text3;
    p.setPen(QPen(color, thickness, Qt::SolidLine, Qt::FlatCap));
    p.drawArc(arc, static_cast<int>((start - sweep * t) * 16), static_cast<int>(-sweep / segments * 16 - 8));
  }

  // Ticks and scale
  double display_max = kmh_ ? max_speed_ * 3.6 : max_speed_;
  QFont tick_font = theme::font(std::max(9, static_cast<int>(r * 0.11)), QFont::DemiBold);
  p.setFont(tick_font);
  for (int i = 0; i <= 10; ++i) {
    double a = (start - sweep * i / 10.0) * M_PI / 180.0;
    double inner = r - thickness / 2.0 - (i % 5 == 0 ? 10.0 : 5.0);
    double outer = r - thickness / 2.0 - 2.0;
    p.setPen(QPen(i % 5 == 0 ? theme::text2 : theme::text3, i % 5 == 0 ? 2.0 : 1.0));
    p.drawLine(QPointF(c.x() + inner * std::cos(a), c.y() - inner * std::sin(a)),
      QPointF(c.x() + outer * std::cos(a), c.y() - outer * std::sin(a)));
    if (i == 0 || i == 10) {
      double lr = inner - 12.0;
      QPointF at(c.x() + lr * std::cos(a), c.y() - lr * std::sin(a));
      p.setPen(theme::text3);
      p.drawText(QRectF(at.x() - 20, at.y() - 8, 40, 16), Qt::AlignCenter,
        QString::number(display_max * i / 10.0, 'f', 0));
    }
  }

  // Commanded speed marker
  if (active_ && std::abs(command_) > 0.01) {
    double t = std::clamp(std::abs(command_) / max_speed_, 0.0, 1.0);
    double a = (start - sweep * t) * M_PI / 180.0;
    double ro = r + thickness / 2.0 + 1.0;
    QPointF tip(c.x() + (r + thickness / 2.0 - 2.0) * std::cos(a), c.y() - (r + thickness / 2.0 - 2.0) * std::sin(a));
    QPointF left(c.x() + (ro + 7) * std::cos(a + 0.05), c.y() - (ro + 7) * std::sin(a + 0.05));
    QPointF right(c.x() + (ro + 7) * std::cos(a - 0.05), c.y() - (ro + 7) * std::sin(a - 0.05));
    QPainterPath marker;
    marker.moveTo(tip);
    marker.lineTo(left);
    marker.lineTo(right);
    marker.closeSubpath();
    p.fillPath(marker, theme::cyan);
  }

  // Digits
  double shown = kmh_ ? std::abs(speed_) * 3.6 : std::abs(speed_);
  QString text = active_ ? (kmh_ ? QString::number(shown, 'f', 0) : QString::number(shown, 'f', 1)) : "--";
  p.setFont(theme::font(static_cast<int>(r * 0.62), QFont::Black));
  p.setPen(active_ ? theme::text : theme::text3);
  theme::drawTabular(p, QRectF(c.x() - r, c.y() - r * 0.4, 2 * r, r * 0.62), text, Qt::AlignCenter);
  p.setFont(spaced(std::max(10, static_cast<int>(r * 0.13)), QFont::Bold, 1.5));
  p.setPen(theme::text3);
  p.drawText(QRectF(c.x() - r, c.y() + r * 0.22, 2 * r, r * 0.2), Qt::AlignCenter, kmh_ ? "KM/H" : "M/S");

  // Gear: D / R / N
  QString gear = !active_ ? "-" : (speed_ > 0.05 ? "D" : (speed_ < -0.05 ? "R" : "N"));
  QRectF gear_box(c.x() - r * 0.19, c.y() + r * 0.5, r * 0.38, r * 0.34);
  theme::fillSlanted(p, gear_box, gear == "R" ? theme::yellow : theme::bg3, gear_box.height() * 0.3);
  p.setFont(theme::font(static_cast<int>(gear_box.height() * 0.75), QFont::Black, true));
  p.setPen(gear == "R" ? QColor(Qt::black) : theme::text);
  p.drawText(gear_box, Qt::AlignCenter, gear);

  // Boost, shown like DRS: lit when the LT trigger multiplies the speed
  bool boosting = active_ && boost_ > 1.05;
  QRectF boost_box(c.x() - r * 0.4, c.y() - r * 0.66, r * 0.8, r * 0.22);
  p.setPen(Qt::NoPen);
  p.setBrush(boosting ? theme::purple : Qt::transparent);
  p.drawRoundedRect(boost_box, 3, 3);
  if (!boosting) {
    p.setPen(QPen(theme::bg3, 1.2));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(boost_box, 3, 3);
  }
  p.setFont(spaced(std::max(9, static_cast<int>(boost_box.height() * 0.55)), QFont::Black, 1.0));
  p.setPen(boosting ? theme::text : theme::text3);
  p.drawText(boost_box, Qt::AlignCenter, boosting ? QString("BOOST x%1").arg(boost_, 0, 'f', 1) : "BOOST");
}

// ---- PedalBars ----

PedalBars::PedalBars(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(80, 120);
}

void PedalBars::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  double bar_w = std::min(28.0, width() / 2.0 - 10.0);
  double top = 20.0, bottom = height() - 22.0;
  struct Bar
  {
    QString label;
    double value;
    QColor color;
  };
  Bar bars[2] = {{reverse_ ? "REV" : "THR", throttle_, reverse_ ? theme::yellow : theme::green},
    {"BRK", brake_, theme::redBright}};
  for (int i = 0; i < 2; ++i) {
    double x = width() / 2.0 + (i == 0 ? -bar_w - 6 : 6);
    QRectF track(x, top, bar_w, bottom - top);
    p.setPen(Qt::NoPen);
    p.setBrush(theme::bg2);
    p.drawRoundedRect(track, 3, 3);
    double v = std::clamp(bars[i].value, 0.0, 1.0);
    QRectF fill(track.left(), track.bottom() - track.height() * v, track.width(), track.height() * v);
    QLinearGradient g(fill.bottomLeft(), track.topLeft());
    g.setColorAt(0.0, bars[i].color.darker(170));
    g.setColorAt(1.0, bars[i].color);
    p.setBrush(g);
    p.drawRoundedRect(fill, 3, 3);
    // Segment lines, like LED bars
    p.setPen(QPen(theme::bg0, 1.5));
    for (int s = 1; s < 10; ++s) {
      double y = track.top() + track.height() * s / 10.0;
      p.drawLine(QPointF(track.left(), y), QPointF(track.right(), y));
    }
    p.setPen(theme::text2);
    p.setFont(theme::font(12, QFont::Bold));
    theme::drawTabular(p, QRectF(x - 10, 0, bar_w + 20, top - 2), QString::number(v * 100.0, 'f', 0),
      Qt::AlignHCenter | Qt::AlignBottom);
    p.setFont(spaced(10, QFont::Bold));
    p.setPen(theme::text3);
    p.drawText(QRectF(x - 10, bottom + 3, bar_w + 20, 16), Qt::AlignCenter, bars[i].label);
  }
}

// ---- SteeringWheel ----

SteeringWheel::SteeringWheel(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(150, 130);
}

void SteeringWheel::set(double angle, double command, double max_angle)
{
  angle_ = angle;
  command_ = command;
  max_angle_ = std::max(0.05, max_angle);
  update();
}

void SteeringWheel::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  // The wheel turns up to 100 degrees, so it has to fit its box sideways too
  double s = std::min(width(), static_cast<int>(height() - 24.0)) / 196.0;
  QPointF c(width() / 2.0, (height() - 24.0) / 2.0 + 2.0);

  // Command arc above the wheel: where the steering is being asked to go
  double arc_r = 92.0 * s;
  QRectF arc(c.x() - arc_r, c.y() - arc_r, 2 * arc_r, 2 * arc_r);
  p.setPen(QPen(theme::bg3, 3.0 * s, Qt::SolidLine, Qt::FlatCap));
  p.drawArc(arc, 45 * 16, 90 * 16);
  auto marker = [&](double angle, const QColor & color, double len) {
      double t = std::clamp(angle / max_angle_, -1.0, 1.0);
      double a = (90.0 + 45.0 * t) * M_PI / 180.0;  // left turn (positive) moves the marker left
      p.setPen(QPen(color, 3.0 * s, Qt::SolidLine, Qt::RoundCap));
      p.drawLine(QPointF(c.x() + (arc_r - len) * std::cos(a), c.y() - (arc_r - len) * std::sin(a)),
        QPointF(c.x() + (arc_r + len) * std::cos(a), c.y() - (arc_r + len) * std::sin(a)));
    };
  marker(command_, theme::cyan, 6.0 * s);
  marker(angle_, theme::text, 4.0 * s);

  // The wheel, turned like the driver's hands would (front wheels x ~3)
  double visual = std::clamp(angle_ / max_angle_, -1.0, 1.0) * 100.0;
  p.save();
  p.translate(c);
  p.rotate(-visual);
  p.scale(s, s);
  QPainterPath grips;
  grips.addRoundedRect(QRectF(-92, -34, 34, 84), 15, 15);
  grips.addRoundedRect(QRectF(58, -34, 34, 84), 15, 15);
  QLinearGradient grip_gradient(0, -34, 0, 50);
  grip_gradient.setColorAt(0, QColor(40, 40, 50));
  grip_gradient.setColorAt(1, QColor(18, 18, 24));
  p.setPen(QPen(QColor(70, 70, 88), 1.2));
  p.setBrush(grip_gradient);
  p.drawPath(grips);
  QPainterPath body;
  body.addRoundedRect(QRectF(-72, -40, 144, 70), 18, 18);
  QLinearGradient body_gradient(0, -40, 0, 30);
  body_gradient.setColorAt(0, QColor(46, 46, 58));
  body_gradient.setColorAt(1, QColor(24, 24, 31));
  p.setBrush(body_gradient);
  p.drawPath(body);
  // Display
  QRectF screen(-34, -30, 68, 36);
  p.setPen(QPen(QColor(80, 80, 100), 1.0));
  p.setBrush(QColor(6, 6, 9));
  p.drawRoundedRect(screen, 4, 4);
  p.setPen(theme::text);
  p.setFont(theme::font(17, QFont::Bold));
  theme::drawTabular(p, screen.adjusted(0, -3, 0, -3),
    QString("%1°").arg(angle_ * 180.0 / M_PI, 0, 'f', 1), Qt::AlignCenter);
  // Buttons and rotaries
  const QColor buttons[] = {theme::red, theme::blue, theme::yellow, theme::green};
  const QPointF spots[] = {{-56, -22}, {56, -22}, {-52, 14}, {52, 14}};
  p.setPen(Qt::NoPen);
  for (int i = 0; i < 4; ++i) {
    p.setBrush(buttons[i]);
    p.drawEllipse(spots[i], 5.5, 5.5);
  }
  p.setBrush(QColor(60, 60, 75));
  p.drawEllipse(QPointF(-18, 18), 6.5, 6.5);
  p.drawEllipse(QPointF(18, 18), 6.5, 6.5);
  p.setPen(QPen(theme::text3, 1.5));
  p.drawLine(QPointF(-18, 18), QPointF(-18, 13));
  p.drawLine(QPointF(18, 18), QPointF(22, 15));
  // Top centre mark
  p.setPen(Qt::NoPen);
  p.setBrush(theme::red);
  p.drawRect(QRectF(-3, -40, 6, 5));
  p.restore();

  p.setFont(spaced(10, QFont::Bold));
  p.setPen(theme::text3);
  p.drawText(QRectF(0, height() - 18, width(), 16), Qt::AlignCenter,
    QString("STEER  CMD %1°").arg(command_ * 180.0 / M_PI, 0, 'f', 1));
}

// ---- ShiftLights ----

ShiftLights::ShiftLights(QWidget * parent)
: QWidget(parent)
{
  setMinimumHeight(16);
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() {flash_ = !flash_; update();});
}

void ShiftLights::setFraction(double fraction)
{
  fraction_ = std::clamp(fraction, 0.0, 1.0);
  bool limiter = fraction_ > 0.97;
  if (limiter && !timer_->isActive()) {
    timer_->start(90);
  } else if (!limiter && timer_->isActive()) {
    timer_->stop();
    flash_ = false;
  }
  update();
}

void ShiftLights::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  const int n = 15;
  double gap = 5.0;
  double d = std::min(height() - 4.0, (width() - gap * (n - 1)) / n);
  double total = n * d + (n - 1) * gap;
  double x0 = (width() - total) / 2.0;
  int lit = static_cast<int>(std::round(fraction_ * n));
  bool limiter = fraction_ > 0.97;
  for (int i = 0; i < n; ++i) {
    QColor color = i < 5 ? theme::green : (i < 10 ? theme::redBright : theme::blue);
    if (limiter) {
      color = flash_ ? theme::blue : theme::bg2;
    }
    bool on = limiter ? flash_ : i < lit;
    QPointF center(x0 + i * (d + gap) + d / 2.0, height() / 2.0);
    if (on) {
      QRadialGradient glow(center, d);
      QColor halo = color;
      halo.setAlpha(110);
      glow.setColorAt(0, halo);
      glow.setColorAt(1, Qt::transparent);
      p.setPen(Qt::NoPen);
      p.setBrush(glow);
      p.drawEllipse(center, d, d);
    }
    p.setPen(QPen(on ? color.lighter(130) : theme::bg3, 1.0));
    p.setBrush(on ? color : QColor(26, 26, 34));
    p.drawEllipse(center, d / 2.0, d / 2.0);
  }
}

// ---- BatteryGauge ----

BatteryGauge::BatteryGauge(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(140, 100);
}

void BatteryGauge::set(double percent, double voltage, int cells, double current, bool connected)
{
  percent_ = percent;
  voltage_ = voltage;
  cells_ = std::max(1, cells);
  current_ = current;
  connected_ = connected;
  update();
}

void BatteryGauge::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  QColor level = percent_ > 50 ? theme::green : (percent_ > 20 ? theme::yellow : theme::redBright);
  if (!connected_) {
    level = theme::text3;
  }
  p.setFont(spaced(11, QFont::Bold));
  p.setPen(theme::text3);
  p.drawText(QRectF(0, 0, width(), 16), Qt::AlignLeft | Qt::AlignVCenter, "BATTERY");

  QRectF body(0, 22, width() - 12, std::min(54.0, height() - 58.0));
  p.setPen(QPen(theme::line, 2));
  p.setBrush(theme::bg1);
  p.drawRoundedRect(body, 6, 6);
  p.setPen(Qt::NoPen);
  p.setBrush(theme::line);
  p.drawRoundedRect(QRectF(body.right() + 2, body.center().y() - 9, 7, 18), 2, 2);
  double fraction = connected_ ? std::clamp(percent_ / 100.0, 0.0, 1.0) : 0.0;
  QRectF fill = body.adjusted(4, 4, -4, -4);
  fill.setWidth(fill.width() * fraction);
  QLinearGradient g(fill.topLeft(), fill.bottomLeft());
  g.setColorAt(0, level);
  g.setColorAt(1, level.darker(160));
  p.setBrush(g);
  p.drawRoundedRect(fill, 3, 3);
  p.setPen(theme::text);
  p.setFont(theme::font(static_cast<int>(body.height() * 0.6), QFont::Black));
  theme::drawTabular(p, body, connected_ ? QString("%1%").arg(percent_, 0, 'f', 0) : "--", Qt::AlignCenter);

  p.setFont(theme::font(13, QFont::DemiBold));
  p.setPen(theme::text2);
  QString detail = connected_ ?
    QString("%1 V  ·  %2 V/cell").arg(voltage_, 0, 'f', 2).arg(voltage_ / cells_, 0, 'f', 2) : "VESC offline";
  theme::drawTabular(p, QRectF(0, body.bottom() + 4, width(), 18), detail, Qt::AlignLeft | Qt::AlignVCenter);
  p.setPen(theme::text3);
  p.setFont(theme::font(12));
  if (connected_) {
    theme::drawTabular(p, QRectF(0, body.bottom() + 22, width(), 16),
      QString("%1 A drawn").arg(current_, 0, 'f', 1), Qt::AlignLeft | Qt::AlignVCenter);
  }
}

// ---- GForceMeter ----

GForceMeter::GForceMeter(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(110, 110);
}

void GForceMeter::addSample(double lateral, double longitudinal)
{
  trace_.push_back(QPointF(lateral, longitudinal));
  while (trace_.size() > 45) {
    trace_.pop_front();
  }
  update();
}

void GForceMeter::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  double r = std::min(width(), height() - 18) / 2.0 - 4.0;
  QPointF c(width() / 2.0, r + 4.0);
  p.setPen(QPen(theme::bg3, 1.0));
  p.setBrush(QColor(16, 16, 22));
  p.drawEllipse(c, r, r);
  p.setBrush(Qt::NoBrush);
  for (double g : {0.5, 1.0}) {
    if (g < range_) {
      p.setPen(QPen(theme::bg3, 1.0, Qt::DashLine));
      p.drawEllipse(c, r * g / range_, r * g / range_);
    }
  }
  p.setPen(QPen(theme::bg3, 1.0));
  p.drawLine(QPointF(c.x() - r, c.y()), QPointF(c.x() + r, c.y()));
  p.drawLine(QPointF(c.x(), c.y() - r), QPointF(c.x(), c.y() + r));
  auto to_screen = [&](const QPointF & g) {
      // Lateral right is +x on screen; braking pushes forward (up)
      double x = std::clamp(g.x() / range_, -1.0, 1.0), y = std::clamp(g.y() / range_, -1.0, 1.0);
      return QPointF(c.x() + x * r, c.y() - y * r);
    };
  for (size_t i = 1; i < trace_.size(); ++i) {
    QColor color = theme::redBright;
    color.setAlphaF(static_cast<double>(i) / trace_.size() * 0.6);
    p.setPen(QPen(color, 2.0, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(to_screen(trace_[i - 1]), to_screen(trace_[i]));
  }
  QPointF g = trace_.empty() ? QPointF() : trace_.back();
  QPointF dot = to_screen(g);
  QRadialGradient glow(dot, 10);
  glow.setColorAt(0, QColor(255, 42, 31, 150));
  glow.setColorAt(1, Qt::transparent);
  p.setPen(Qt::NoPen);
  p.setBrush(glow);
  p.drawEllipse(dot, 10, 10);
  p.setBrush(theme::redBright);
  p.drawEllipse(dot, 4, 4);
  p.setFont(theme::font(11, QFont::Bold));
  p.setPen(theme::text2);
  double total = std::hypot(g.x(), g.y());
  theme::drawTabular(p, QRectF(0, height() - 16, width(), 16), QString("%1 G").arg(total, 0, 'f', 2),
    Qt::AlignCenter);
}

// ---- TimingTower ----

TimingTower::TimingTower(QWidget * parent)
: QWidget(parent)
{
  reset();
}

void TimingTower::reset()
{
  for (int i = 0; i < 3; ++i) {
    current_[i] = previous_[i] = best_[i] = 0.0;
    sector_color_[i] = theme::bg3;
  }
  sector_ = 0;
  last_lap_count_ = 0;
  previous_best_lap_ = 0.0;
  update();
}

void TimingTower::setLap(const LapState & lap)
{
  auto evaluate = [this](int i) {
      double t = current_[i];
      if (t <= 0.0) {
        return;
      }
      if (best_[i] <= 0.0 || t <= best_[i]) {
        best_[i] = t;
        sector_color_[i] = theme::purple;
      } else if (previous_[i] > 0.0 && t < previous_[i]) {
        sector_color_[i] = theme::green;
      } else {
        sector_color_[i] = theme::yellow;
      }
    };
  if (have_lap_ && lap.lap < last_lap_count_) {
    reset();  // lap times were reset
  } else if (have_lap_ && lap.current_lap_time + 0.5f < lap_.current_lap_time) {
    // Crossed the line: close sector 3 of the lap just finished
    if (lap.lap > last_lap_count_ && current_[0] > 0.0 && current_[1] > 0.0 && lap.last_lap_time > 0.0f) {
      current_[2] = lap.last_lap_time - current_[0] - current_[1];
      evaluate(2);
    }
    if (lap_.best_lap_time > 0.0f) {
      previous_best_lap_ = lap_.best_lap_time;
    }
    for (int i = 0; i < 3; ++i) {
      previous_[i] = current_[i];
      current_[i] = 0.0;
    }
    sector_ = 0;
  }
  if (lap.active && lap.current_lap_time > 0.0f) {
    int s = std::min(2, static_cast<int>(lap.progress * 3.0f));
    if (s == sector_ + 1) {
      double before = 0.0;
      for (int i = 0; i < sector_; ++i) {
        before += current_[i];
      }
      current_[sector_] = lap.current_lap_time - before;
      if (sector_ == 0) {
        sector_color_[1] = sector_color_[2] = theme::bg3;
      }
      evaluate(sector_);
      sector_ = s;
    }
  }
  last_lap_count_ = lap.lap;
  lap_ = lap;
  have_lap_ = true;
  update();
}

void TimingTower::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  double w = width();
  bool timing = have_lap_ && lap_.active;

  // Header: lap number on a red slanted block
  QRectF head(0, 0, w * 0.46, 30);
  theme::fillSlanted(p, head, timing ? theme::red : theme::bg3, 10);
  p.setPen(theme::text);
  p.setFont(spaced(16, QFont::Black, 1.5));
  QString lap_text = !timing ? "NO TIMING" :
    (lap_.current_lap_time <= 0.0f ? "OUT LAP" : QString("LAP %1").arg(lap_.lap + 1));
  p.drawText(head, Qt::AlignCenter, lap_text);
  p.setFont(theme::font(13, QFont::Bold));
  p.setPen(theme::text2);
  if (timing) {
    theme::drawTabular(p, QRectF(w * 0.5, 0, w * 0.5, 30), QString("%1%").arg(lap_.progress * 100.0, 0, 'f', 0),
      Qt::AlignRight | Qt::AlignVCenter);
  }

  if (!timing) {
    p.setFont(theme::font(13));
    p.setPen(theme::text3);
    p.drawText(QRectF(0, 40, w, height() - 40), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap,
      "Laps are timed on the selected path once the car is localized on the map.\n\n"
      "Pick a map and a path on HOME, then set the car's pose in the 3D view if needed.");
    return;
  }

  // Current lap
  p.setFont(theme::font(std::min(52, static_cast<int>(height() * 0.2)), QFont::Black));
  bool running = lap_.current_lap_time > 0.0f;
  p.setPen(running ? theme::text : theme::text3);
  double y = 36;
  double big = std::min(56.0, height() * 0.22);
  theme::drawTabular(p, QRectF(0, y, w, big), running ? lapTime(lap_.current_lap_time) : QString("0:00.000"),
    Qt::AlignLeft | Qt::AlignVCenter);
  y += big + 4;

  auto row = [&](const QString & label, double time, const QColor & color, const QString & extra,
    const QColor & extra_color) {
      p.setFont(spaced(11, QFont::Bold));
      p.setPen(theme::text3);
      p.drawText(QRectF(0, y, 60, 24), Qt::AlignLeft | Qt::AlignVCenter, label);
      p.setFont(theme::font(19, QFont::Bold));
      p.setPen(color);
      theme::drawTabular(p, QRectF(58, y, w * 0.5, 24), lapTime(time), Qt::AlignLeft | Qt::AlignVCenter);
      if (!extra.isEmpty()) {
        p.setFont(theme::font(15, QFont::Bold));
        p.setPen(extra_color);
        theme::drawTabular(p, QRectF(0, y, w, 24), extra, Qt::AlignRight | Qt::AlignVCenter);
      }
      y += 26;
    };
  QString delta;
  QColor delta_color = theme::text2;
  if (lap_.last_lap_time > 0.0f && previous_best_lap_ > 0.0) {
    double d = lap_.last_lap_time - previous_best_lap_;
    delta = QString("%1%2").arg(d <= 0 ? "-" : "+").arg(std::abs(d), 0, 'f', 3);
    delta_color = d <= 0 ? theme::green : theme::yellow;
  }
  row("LAST", lap_.last_lap_time, theme::text, delta, delta_color);
  row("BEST", lap_.best_lap_time, lap_.best_lap_time > 0.0f ? theme::purple : theme::text3, QString(), QColor());

  // Sectors
  y += 4;
  double sw = (w - 8) / 3.0;
  for (int i = 0; i < 3; ++i) {
    QRectF box(i * (sw + 4), y, sw, 30);
    bool running = i == sector_;
    QColor fill = (current_[i] > 0.0 || previous_[i] > 0.0) ? sector_color_[i] : theme::bg3;
    if (running && current_[i] <= 0.0) {
      fill = theme::bg2;
    }
    p.setPen(Qt::NoPen);
    p.setBrush(fill);
    p.drawRoundedRect(QRectF(box.left(), box.top(), box.width(), 5), 2, 2);
    p.setFont(spaced(10, QFont::Bold));
    p.setPen(running ? theme::text : theme::text3);
    p.drawText(QRectF(box.left(), box.top() + 7, 24, 20), Qt::AlignLeft | Qt::AlignVCenter, QString("S%1").arg(i + 1));
    double t = current_[i] > 0.0 ? current_[i] : previous_[i];
    p.setFont(theme::font(13, QFont::Bold));
    p.setPen(t > 0.0 ? theme::text : theme::text3);
    theme::drawTabular(p, QRectF(box.left(), box.top() + 7, box.width() - 2, 20),
      t > 0.0 ? QString::number(t, 'f', 2) : "--", Qt::AlignRight | Qt::AlignVCenter);
  }
  y += 36;

  // Progress along the track and distance from the line
  QRectF track(0, y + 4, w, 4);
  p.setPen(Qt::NoPen);
  p.setBrush(theme::bg3);
  p.drawRoundedRect(track, 2, 2);
  for (int i = 1; i < 3; ++i) {
    p.fillRect(QRectF(track.left() + track.width() * i / 3.0 - 1, track.top() - 3, 2, 10), theme::text3);
  }
  double px = track.left() + track.width() * std::clamp(static_cast<double>(lap_.progress), 0.0, 1.0);
  p.setBrush(theme::red);
  p.drawRoundedRect(QRectF(track.left(), track.top(), px - track.left(), track.height()), 2, 2);
  p.setBrush(theme::text);
  p.drawEllipse(QPointF(px, track.center().y()), 5, 5);
  y += 16;
  p.setFont(spaced(10, QFont::Bold));
  p.setPen(theme::text3);
  QColor xte = lap_.cross_track_error < 0.2f ? theme::green : (lap_.cross_track_error < 0.5f ? theme::yellow :
    theme::redBright);
  p.drawText(QRectF(0, y, w, 18), Qt::AlignLeft | Qt::AlignVCenter, "OFF LINE");
  p.setFont(theme::font(13, QFont::Bold));
  p.setPen(xte);
  theme::drawTabular(p, QRectF(0, y, w, 18), QString("%1 m").arg(lap_.cross_track_error, 0, 'f', 2),
    Qt::AlignRight | Qt::AlignVCenter);
}

// ---- FlagStrip ----

FlagStrip::FlagStrip(QWidget * parent)
: QWidget(parent)
{
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
}

void FlagStrip::setFlag(const QString & key, const QString & text, const QColor & color, bool lit)
{
  for (auto & flag : flags_) {
    if (flag.key == key) {
      if (flag.text != text || flag.color != color || flag.lit != lit) {
        flag.text = text;
        flag.color = color;
        flag.lit = lit;
        update();
      }
      return;
    }
  }
  flags_.push_back({key, text, color, lit});
  updateGeometry();
  update();
}

QSize FlagStrip::sizeHint() const
{
  QFontMetrics fm(spaced(10, QFont::Bold, 1.0));
  int w = 0;
  for (const auto & flag : flags_) {
    w += fm.horizontalAdvance(flag.text) + 22;
  }
  return QSize(w, 24);
}

void FlagStrip::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  QFont f = spaced(10, QFont::Bold, 1.0);
  p.setFont(f);
  QFontMetrics fm(f);
  double x = 0;
  for (const auto & flag : flags_) {
    double w = fm.horizontalAdvance(flag.text) + 16;
    if (x + w > width()) {
      break;
    }
    QRectF box(x, 2, w, 20);
    if (flag.lit) {
      p.setPen(Qt::NoPen);
      p.setBrush(flag.color);
      p.drawRoundedRect(box, 3, 3);
      p.setPen(flag.color.lightness() > 150 ? QColor(Qt::black) : theme::text);
    } else {
      p.setPen(QPen(theme::bg3, 1.2));
      p.setBrush(Qt::NoBrush);
      p.drawRoundedRect(box.adjusted(0.5, 0.5, -0.5, -0.5), 3, 3);
      p.setPen(theme::text3);
    }
    p.drawText(box, Qt::AlignCenter, flag.text);
    x += w + 6;
  }
}
}  // namespace f1ui
