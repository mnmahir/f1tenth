// 2D views: telemetry strip charts, the track map (map preview + path + car) and the camera feed.
#pragma once

#include <QColor>
#include <QImage>
#include <QPointF>
#include <QString>
#include <QVector>
#include <QWidget>

#include <deque>
#include <limits>

class QTimer;

namespace f1ui
{
// Scrolling time-series chart with several series
class StripChart : public QWidget
{
  Q_OBJECT

public:
  StripChart(const QString & title, const QString & unit, QWidget * parent = nullptr);
  int addSeries(const QString & name, const QColor & color);
  void addPoint(int series, double time, double value);  // time in seconds
  void setWindow(double seconds) {window_ = seconds; update();}
  // Fixed y range; with min >= max the range follows the data
  void setRange(double min, double max) {fixed_min_ = min; fixed_max_ = max; update();}
  // Smallest y span when following the data
  void setMinSpan(double span) {min_span_ = span; update();}
  void setPaused(bool paused) {paused_ = paused;}
  void clear();
  QSize sizeHint() const override {return QSize(420, 180);}
  QSize minimumSizeHint() const override {return QSize(200, 120);}

protected:
  void paintEvent(QPaintEvent *) override;

private:
  struct Series
  {
    QString name;
    QColor color;
    std::deque<QPointF> points;
  };
  QString title_, unit_;
  QVector<Series> series_;
  double window_ = 30.0, now_ = 0.0;
  double fixed_min_ = 0.0, fixed_max_ = 0.0, min_span_ = 1.0;
  bool paused_ = false;
};

// Map preview with a path (coloured by speed), start line and the car
class TrackMap : public QWidget
{
  Q_OBJECT

public:
  explicit TrackMap(QWidget * parent = nullptr);
  // image: row 0 is the top (largest y); origin is the world position of the lower-left corner
  void setMap(const QImage & image, double resolution, double origin_x, double origin_y);
  // Map preview from the supervisor (mono8: 0 occupied, 255 free, 127 unknown)
  void setPreview(const QImage & gray, double resolution, double origin_x, double origin_y);
  void clearMap();
  void setPath(const QVector<QPointF> & points, const QVector<float> & speeds = {});
  void clearPath();
  void setCar(bool valid, double x = 0.0, double y = 0.0, double yaw = 0.0);
  void setPlaceholder(const QString & text) {placeholder_ = text; update();}
  void setInteractive(bool interactive) {interactive_ = interactive;}
  void fit();
  bool hasMap() const {return !image_.isNull();}
  QSize sizeHint() const override {return QSize(360, 260);}

protected:
  void paintEvent(QPaintEvent *) override;
  void wheelEvent(QWheelEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseDoubleClickEvent(QMouseEvent *) override {fit();}

private:
  QPointF toScreen(const QPointF & world) const;
  QRectF worldBounds() const;

  QImage image_;
  double resolution_ = 0.05, origin_x_ = 0.0, origin_y_ = 0.0;
  QRectF content_;  // world rectangle of the mapped (not unknown) part
  QVector<QPointF> path_;
  QVector<float> speeds_;
  bool car_valid_ = false;
  double car_x_ = 0.0, car_y_ = 0.0, car_yaw_ = 0.0;
  QString placeholder_ = "No map";
  bool interactive_ = true;
  double zoom_ = 1.0;
  QPointF pan_;  // screen px
  QPoint drag_start_;
  QPointF pan_start_;
};

// Camera feed, letterboxed, with a small on-screen display
class CameraView : public QWidget
{
  Q_OBJECT

public:
  explicit CameraView(QWidget * parent = nullptr);
  void setFrame(const QImage & image, double fps);
  void setCaption(const QString & caption) {caption_ = caption; update();}
  // Degrees to turn the picture anticlockwise, for a camera mounted on its side
  void setRotation(int degrees) {rotation_ = ((degrees % 360) + 360) % 360;}
  QSize sizeHint() const override {return QSize(400, 225);}
  int heightForWidth(int width) const override {return width * 9 / 16;}
  bool hasHeightForWidth() const override {return true;}

Q_SIGNALS:
  void clicked();

protected:
  void paintEvent(QPaintEvent *) override;
  void mouseReleaseEvent(QMouseEvent *) override {Q_EMIT clicked();}

private:
  QImage frame_;
  int rotation_ = 0;
  double fps_ = 0.0;
  qint64 last_frame_ms_ = 0;
  QString caption_ = "ONBOARD CAMERA";
  QTimer * stale_timer_ = nullptr;
};
}  // namespace f1ui
