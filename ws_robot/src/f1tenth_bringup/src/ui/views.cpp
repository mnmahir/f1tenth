#include "views.hpp"

#include <QDateTime>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QTimer>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

#include "theme.hpp"

namespace f1ui
{
// ---- StripChart ----

StripChart::StripChart(const QString & title, const QString & unit, QWidget * parent)
: QWidget(parent), title_(title.toUpper()), unit_(unit)
{
}

int StripChart::addSeries(const QString & name, const QColor & color)
{
  series_.push_back({name, color, {}});
  return series_.size() - 1;
}

void StripChart::addPoint(int series, double time, double value)
{
  if (paused_ || series < 0 || series >= series_.size() || !std::isfinite(value)) {
    return;
  }
  auto & points = series_[series].points;
  points.push_back(QPointF(time, value));
  while (!points.empty() && points.front().x() < time - 120.0) {
    points.pop_front();
  }
  now_ = std::max(now_, time);
}

void StripChart::clear()
{
  for (auto & s : series_) {
    s.points.clear();
  }
  update();
}

void StripChart::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  QFont caption = theme::font(11, QFont::Bold);
  caption.setLetterSpacing(QFont::AbsoluteSpacing, 1.2);
  p.setFont(caption);
  p.setPen(theme::text2);
  p.drawText(QRectF(0, 0, width(), 18), Qt::AlignLeft | Qt::AlignVCenter, title_);

  // Legend with latest values, right aligned
  double x = width();
  p.setFont(theme::font(12, QFont::Bold));
  for (int i = series_.size() - 1; i >= 0; --i) {
    const auto & s = series_[i];
    QString text = s.name + " " + (s.points.empty() ? QString("--") :
      QString::number(s.points.back().y(), 'f', std::abs(s.points.back().y()) < 10 ? 2 : 1)) +
      (unit_.isEmpty() ? QString() : " " + unit_);
    double w = p.fontMetrics().horizontalAdvance(text);
    x -= w;
    p.setPen(s.color);
    p.drawText(QRectF(x, 0, w, 18), Qt::AlignVCenter, text);
    x -= 14;
  }

  QRectF plot(38, 24, width() - 42, height() - 42);
  double lo = fixed_min_, hi = fixed_max_;
  if (lo >= hi) {
    lo = std::numeric_limits<double>::max();
    hi = std::numeric_limits<double>::lowest();
    for (const auto & s : series_) {
      for (const auto & pt : s.points) {
        if (pt.x() >= now_ - window_) {
          lo = std::min(lo, pt.y());
          hi = std::max(hi, pt.y());
        }
      }
    }
    if (lo > hi) {
      lo = hi = 0.0;
    }
    if (hi - lo < min_span_) {
      double mid = (hi + lo) / 2.0;
      lo = mid - min_span_ / 2.0;
      hi = mid + min_span_ / 2.0;
    }
    double margin = (hi - lo) * 0.08;
    lo -= margin;
    hi += margin;
  }
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(15, 15, 21));
  p.drawRect(plot);
  p.setFont(theme::font(10));
  for (int i = 0; i <= 4; ++i) {
    double y = plot.bottom() - plot.height() * i / 4.0;
    p.setPen(QPen(QColor(32, 32, 42), 1.0));
    p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
    p.setPen(theme::text3);
    double v = lo + (hi - lo) * i / 4.0;
    p.drawText(QRectF(0, y - 8, 34, 16), Qt::AlignRight | Qt::AlignVCenter,
      QString::number(v, 'f', std::abs(hi - lo) < 5 ? 1 : 0));
  }
  if (lo < 0.0 && hi > 0.0) {
    double y0 = plot.bottom() - plot.height() * (0.0 - lo) / (hi - lo);
    p.setPen(QPen(theme::bg3, 1.0));
    p.drawLine(QPointF(plot.left(), y0), QPointF(plot.right(), y0));
  }
  for (int i = 0; i <= 3; ++i) {
    double xs = plot.left() + plot.width() * i / 3.0;
    p.setPen(theme::text3);
    p.drawText(QRectF(xs - 30, plot.bottom() + 2, 60, 14), Qt::AlignCenter,
      i == 3 ? QString("now") : QString("-%1s").arg(window_ * (3 - i) / 3.0, 0, 'f', 0));
  }

  p.setClipRect(plot);
  for (const auto & s : series_) {
    QPolygonF line;
    for (const auto & pt : s.points) {
      if (pt.x() < now_ - window_ - 1.0) {
        continue;
      }
      double px = plot.left() + plot.width() * (pt.x() - (now_ - window_)) / window_;
      double py = plot.bottom() - plot.height() * (pt.y() - lo) / (hi - lo);
      line << QPointF(px, py);
    }
    p.setPen(QPen(s.color, 1.8));
    p.drawPolyline(line);
  }
}

// ---- TrackMap ----

TrackMap::TrackMap(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(160, 120);
}

void TrackMap::setMap(const QImage & image, double resolution, double origin_x, double origin_y)
{
  bool same_size = !image_.isNull() && image.size() == image_.size();
  image_ = image;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  // Fit the view to what was mapped, not the whole (mostly unknown) map canvas
  int c0 = image.width(), c1 = -1, r0 = image.height(), r1 = -1;
  for (int row = 0; row < image.height(); ++row) {
    auto * line = reinterpret_cast<const QRgb *>(image.constScanLine(row));
    for (int col = 0; col < image.width(); ++col) {
      if (qAlpha(line[col]) > 0) {
        c0 = std::min(c0, col);
        c1 = std::max(c1, col);
        r0 = std::min(r0, row);
        r1 = std::max(r1, row);
      }
    }
  }
  content_ = c1 < 0 ? QRectF() : QRectF(origin_x + c0 * resolution, origin_y + (image.height() - r1 - 1) * resolution,
    (c1 - c0 + 1) * resolution, (r1 - r0 + 1) * resolution);
  if (!same_size) {
    fit();
  }
  update();
}

void TrackMap::setPreview(const QImage & gray, double resolution, double origin_x, double origin_y)
{
  QImage image(gray.size(), QImage::Format_ARGB32);
  QRgb free = QColor(36, 39, 48).rgb(), wall = QColor(236, 238, 242).rgb();
  for (int row = 0; row < gray.height(); ++row) {
    const uchar * in = gray.constScanLine(row);
    auto * out = reinterpret_cast<QRgb *>(image.scanLine(row));
    for (int col = 0; col < gray.width(); ++col) {
      out[col] = in[col] < 64 ? wall : (in[col] > 192 ? free : qRgba(0, 0, 0, 0));
    }
  }
  setMap(image, resolution, origin_x, origin_y);
}

void TrackMap::clearMap()
{
  image_ = QImage();
  update();
}

void TrackMap::setPath(const QVector<QPointF> & points, const QVector<float> & speeds)
{
  path_ = points;
  speeds_ = speeds;
  if (image_.isNull()) {
    fit();
  }
  update();
}

void TrackMap::clearPath()
{
  path_.clear();
  speeds_.clear();
  update();
}

void TrackMap::setCar(bool valid, double x, double y, double yaw)
{
  car_valid_ = valid;
  car_x_ = x;
  car_y_ = y;
  car_yaw_ = yaw;
  update();
}

void TrackMap::fit()
{
  zoom_ = 1.0;
  pan_ = QPointF();
  update();
}

QRectF TrackMap::worldBounds() const
{
  QRectF bounds;
  if (!image_.isNull()) {
    bounds = content_.isValid() ? content_ :
      QRectF(origin_x_, origin_y_, image_.width() * resolution_, image_.height() * resolution_);
  }
  if (!path_.isEmpty()) {
    double x0 = path_[0].x(), x1 = x0, y0 = path_[0].y(), y1 = y0;
    for (const auto & pt : path_) {
      x0 = std::min(x0, pt.x());
      x1 = std::max(x1, pt.x());
      y0 = std::min(y0, pt.y());
      y1 = std::max(y1, pt.y());
    }
    QRectF path_bounds(x0, y0, x1 - x0, y1 - y0);
    bounds = bounds.isValid() ? bounds.united(path_bounds) : path_bounds;
  }
  if (!bounds.isValid()) {
    return QRectF(-5, -5, 10, 10);
  }
  return bounds.adjusted(-0.5, -0.5, 0.5, 0.5);
}

QPointF TrackMap::toScreen(const QPointF & world) const
{
  QRectF b = worldBounds();
  double scale = std::min((width() - 16) / b.width(), (height() - 16) / b.height()) * zoom_;
  QPointF c = b.center();
  return QPointF(width() / 2.0 + (world.x() - c.x()) * scale + pan_.x(),
    height() / 2.0 - (world.y() - c.y()) * scale + pan_.y());
}

void TrackMap::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.fillRect(rect(), QColor(13, 14, 19));
  // Faint 1 m grid
  if (!image_.isNull() || !path_.isEmpty()) {
    QRectF b = worldBounds();
    p.setPen(QPen(QColor(28, 30, 38), 1.0));
    for (double gx = std::floor(b.left()); gx <= b.right(); gx += 1.0) {
      p.drawLine(toScreen(QPointF(gx, b.top())), toScreen(QPointF(gx, b.bottom())));
    }
    for (double gy = std::floor(b.top()); gy <= b.bottom(); gy += 1.0) {
      p.drawLine(toScreen(QPointF(b.left(), gy)), toScreen(QPointF(b.right(), gy)));
    }
  }
  if (image_.isNull() && path_.isEmpty()) {
    p.setPen(theme::text3);
    p.setFont(theme::font(13));
    p.drawText(rect(), Qt::AlignCenter | Qt::TextWordWrap, placeholder_);
    return;
  }
  if (!image_.isNull()) {
    QRectF b(origin_x_, origin_y_, image_.width() * resolution_, image_.height() * resolution_);
    QPointF tl = toScreen(QPointF(b.left(), b.bottom()));
    QPointF br = toScreen(QPointF(b.right(), b.top()));
    p.setRenderHint(QPainter::SmoothPixmapTransform, (br.x() - tl.x()) < image_.width());
    p.drawImage(QRectF(tl, br), image_);
  }
  if (path_.size() >= 2) {
    double max_speed = 0.0;
    for (float v : speeds_) {
      max_speed = std::max(max_speed, static_cast<double>(v));
    }
    bool by_speed = speeds_.size() == path_.size() && max_speed > 0.0;
    double min_speed = max_speed;
    for (float v : speeds_) {
      min_speed = std::min(min_speed, static_cast<double>(v));
    }
    for (int i = 1; i < path_.size(); ++i) {
      QColor color = theme::purple;
      if (by_speed) {
        double span = std::max(0.1, max_speed - min_speed);
        color = theme::heat((speeds_[i] - min_speed) / span);
      }
      p.setPen(QPen(color, 3.0, Qt::SolidLine, Qt::RoundCap));
      p.drawLine(toScreen(path_[i - 1]), toScreen(path_[i]));
    }
    // Start/finish line across the first point, chequered
    QPointF a = toScreen(path_[0]), b = toScreen(path_[std::min(3, static_cast<int>(path_.size()) - 1)]);
    QPointF dir = b - a;
    double len = std::hypot(dir.x(), dir.y());
    if (len > 0.1) {
      QPointF n(-dir.y() / len, dir.x() / len);
      for (int k = -3; k < 3; ++k) {
        p.setPen(QPen(k % 2 ? QColor(Qt::white) : QColor(Qt::black), 4.0, Qt::SolidLine, Qt::FlatCap));
        p.drawLine(a + n * (k * 3.0), a + n * ((k + 1) * 3.0));
      }
    }
  }
  if (car_valid_) {
    QPointF c = toScreen(QPointF(car_x_, car_y_));
    p.save();
    p.translate(c);
    p.rotate(-car_yaw_ * 180.0 / M_PI);
    QPainterPath arrow;
    arrow.moveTo(10, 0);
    arrow.lineTo(-7, 6);
    arrow.lineTo(-4, 0);
    arrow.lineTo(-7, -6);
    arrow.closeSubpath();
    QRadialGradient glow(QPointF(0, 0), 16);
    glow.setColorAt(0, QColor(225, 6, 0, 120));
    glow.setColorAt(1, Qt::transparent);
    p.setPen(Qt::NoPen);
    p.setBrush(glow);
    p.drawEllipse(QPointF(0, 0), 16, 16);
    p.setPen(QPen(Qt::white, 1.5));
    p.setBrush(theme::red);
    p.drawPath(arrow);
    p.restore();
  }
}

void TrackMap::wheelEvent(QWheelEvent * event)
{
  if (!interactive_) {
    return;
  }
  double factor = event->angleDelta().y() > 0 ? 1.2 : 1.0 / 1.2;
  QPointF mouse = event->position() - QPointF(width() / 2.0, height() / 2.0);
  // Zoom about the cursor
  pan_ = mouse - (mouse - pan_) * factor;
  zoom_ *= factor;
  update();
}

void TrackMap::mousePressEvent(QMouseEvent * event)
{
  drag_start_ = event->pos();
  pan_start_ = pan_;
}

void TrackMap::mouseMoveEvent(QMouseEvent * event)
{
  if (interactive_ && (event->buttons() & Qt::LeftButton)) {
    pan_ = pan_start_ + QPointF(event->pos() - drag_start_);
    update();
  }
}

// ---- CameraView ----

CameraView::CameraView(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(160, 90);
  setCursor(Qt::PointingHandCursor);
  stale_timer_ = new QTimer(this);
  connect(stale_timer_, &QTimer::timeout, this, [this]() {update();});
  stale_timer_->start(1000);
}

void CameraView::setFrame(const QImage & image, double fps)
{
  frame_ = image;
  fps_ = fps;
  last_frame_ms_ = QDateTime::currentMSecsSinceEpoch();
  update();
}

void CameraView::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.setRenderHint(QPainter::SmoothPixmapTransform);
  p.fillRect(rect(), QColor(5, 5, 8));
  bool live = !frame_.isNull() && QDateTime::currentMSecsSinceEpoch() - last_frame_ms_ < 1500;
  if (!frame_.isNull()) {
    QSize size = frame_.size().scaled(this->size(), Qt::KeepAspectRatio);
    QRect target(QPoint((width() - size.width()) / 2, (height() - size.height()) / 2), size);
    p.drawImage(target, frame_);
    if (!live) {
      p.fillRect(target, QColor(0, 0, 0, 170));
    }
  }
  // Corner brackets
  p.setPen(QPen(QColor(255, 255, 255, 90), 2.0));
  int m = 8, l = 14;
  QRect r = rect().adjusted(m, m, -m, -m);
  p.drawLine(r.topLeft(), r.topLeft() + QPoint(l, 0));
  p.drawLine(r.topLeft(), r.topLeft() + QPoint(0, l));
  p.drawLine(r.topRight(), r.topRight() - QPoint(l, 0));
  p.drawLine(r.topRight(), r.topRight() + QPoint(0, l));
  p.drawLine(r.bottomLeft(), r.bottomLeft() + QPoint(l, 0));
  p.drawLine(r.bottomLeft(), r.bottomLeft() - QPoint(0, l));
  p.drawLine(r.bottomRight(), r.bottomRight() - QPoint(l, 0));
  p.drawLine(r.bottomRight(), r.bottomRight() - QPoint(0, l));

  QFont f = theme::font(11, QFont::Bold);
  f.setLetterSpacing(QFont::AbsoluteSpacing, 1.2);
  p.setFont(f);
  QString osd = caption_ + (live ? QString("  ·  %1 FPS").arg(fps_, 0, 'f', 0) : QString());
  QRectF osd_box(m + 6, m + 4, p.fontMetrics().horizontalAdvance(osd) + 26, 20);
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(0, 0, 0, 150));
  p.drawRoundedRect(osd_box, 3, 3);
  p.setBrush(live ? theme::redBright : theme::text3);
  p.drawEllipse(QPointF(osd_box.left() + 10, osd_box.center().y()), 3.5, 3.5);
  p.setPen(theme::text);
  p.drawText(osd_box.adjusted(18, 0, 0, 0), Qt::AlignVCenter | Qt::AlignLeft, osd);

  if (!live) {
    p.setPen(theme::text3);
    p.setFont(theme::iconFont(34));
    p.drawText(QRectF(0, height() / 2.0 - 40, width(), 40), Qt::AlignCenter, QString(theme::icon::videocamOff));
    p.setFont(theme::font(13, QFont::Bold));
    p.drawText(QRectF(0, height() / 2.0 + 2, width(), 20), Qt::AlignCenter, "NO SIGNAL");
  }
}
}  // namespace f1ui
