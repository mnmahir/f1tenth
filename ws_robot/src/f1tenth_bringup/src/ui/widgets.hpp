// Small building blocks shared by the pages.
#pragma once

#include <QAbstractButton>
#include <QFrame>
#include <QWheelEvent>
#include <QIcon>
#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include <utility>

class QHBoxLayout;
class QVBoxLayout;
class QTimer;

namespace f1ui
{
// Icon from the Material Icons font; the checked state can use another colour
QIcon glyphIcon(QChar glyph, const QColor & color, int size = 18, const QColor & checked_color = QColor());

QPushButton * makeButton(const QString & text, QChar glyph = QChar(), const QString & role = QString());
QLabel * makeLabel(const QString & text, int pixel_size = 13, int weight = 50, const QColor & color = QColor());
// Small uppercase caption with letter spacing ("SPEED", "BATTERY"...)
QLabel * makeCaption(const QString & text);
// Re-applies the stylesheet after a dynamic property change (e.g. role)
void repolish(QWidget * widget);

// A slider, spin box or combo box inside a scrolling panel: the wheel scrolls the panel unless the control was
// clicked first, so scrolling past it can't change a setting by accident
template<class Control>
class FocusWheel : public Control
{
public:
  template<typename ... Args>
  explicit FocusWheel(Args && ... args)
  : Control(std::forward<Args>(args)...)
  {
    this->setFocusPolicy(Qt::StrongFocus);
  }

protected:
  void wheelEvent(QWheelEvent * event) override
  {
    if (this->hasFocus()) {
      Control::wheelEvent(event);
    } else {
      event->ignore();
    }
  }
};

// Label that shortens long text with "..." at the end to fit its width
class ElidedLabel : public QLabel
{
  Q_OBJECT

public:
  using QLabel::QLabel;
  void setFullText(const QString & text);

protected:
  void resizeEvent(QResizeEvent * event) override;

private:
  void refresh();
  QString full_;
};

// Dark panel with an F1-style title (red tick + uppercase caption) and an optional header widget
class Card : public QFrame
{
  Q_OBJECT

public:
  explicit Card(const QString & title = QString(), QWidget * parent = nullptr);
  QVBoxLayout * body() const {return body_;}
  QHBoxLayout * header() const {return header_;}
  void setTitle(const QString & title);

private:
  QLabel * title_ = nullptr;
  QHBoxLayout * header_ = nullptr;
  QVBoxLayout * body_ = nullptr;
};

class StatusLed : public QWidget
{
  Q_OBJECT

public:
  explicit StatusLed(QWidget * parent = nullptr, int diameter = 10);
  void setColor(const QColor & color);
  void setBlinking(bool blinking);
  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent *) override;

private:
  QColor color_;
  int diameter_;
  bool blinking_ = false, phase_ = true;
  QTimer * timer_ = nullptr;
};

// On/off switch with a label
class Toggle : public QAbstractButton
{
  Q_OBJECT

public:
  explicit Toggle(const QString & text = QString(), QWidget * parent = nullptr);
  void setAccent(const QColor & color) {accent_ = color; update();}
  // Shows the switch as busy (request in flight)
  void setPending(bool pending) {pending_ = pending; update();}
  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent *) override;

private:
  QColor accent_;
  bool pending_ = false;
};

// Left navigation rail entry: icon above an uppercase label
class NavButton : public QAbstractButton
{
  Q_OBJECT

public:
  NavButton(QChar glyph, const QString & text, QWidget * parent = nullptr);
  void setBadge(const QColor & color) {badge_ = color; update();}  // invalid colour hides it
  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent *) override;
  void enterEvent(QEvent *) override {hover_ = true; update();}
  void leaveEvent(QEvent *) override {hover_ = false; update();}

private:
  QChar glyph_;
  QColor badge_;
  bool hover_ = false;
};

// Slanted coloured label, like the team boxes on F1 timing graphics
class SkewBadge : public QWidget
{
  Q_OBJECT

public:
  explicit SkewBadge(QWidget * parent = nullptr);
  void set(const QString & text, const QColor & color, const QColor & text_color = Qt::white);
  void setPixelSize(int size) {pixel_size_ = size; updateGeometry(); update();}
  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent *) override;

private:
  QString text_;
  QColor color_, text_color_;
  int pixel_size_ = 13;
};

// Caption, big value and unit, with an optional level bar underneath
class ValueTile : public QWidget
{
  Q_OBJECT

public:
  explicit ValueTile(const QString & caption, const QString & unit = QString(), QWidget * parent = nullptr);
  void setValue(const QString & value, const QColor & color = QColor());
  void setBar(double fraction, const QColor & color);  // fraction < 0 hides the bar
  void setDetail(const QString & detail) {detail_ = detail; update();}
  void setValuePixelSize(int size) {value_size_ = size; updateGeometry(); update();}
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

protected:
  void paintEvent(QPaintEvent *) override;

private:
  QString caption_, unit_, value_ = "--", detail_;
  QColor value_color_;
  double bar_ = -1.0;
  QColor bar_color_;
  int value_size_ = 26;
};
}  // namespace f1ui
