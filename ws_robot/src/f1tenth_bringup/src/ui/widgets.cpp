#include "widgets.hpp"

#include <QHBoxLayout>
#include <QPainter>
#include <QPainterPath>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>
#include <QVariant>

#include <algorithm>

#include "theme.hpp"

namespace f1ui
{
namespace
{
QPixmap glyphPixmap(QChar glyph, const QColor & color, int size, qreal dpr)
{
  QPixmap pm(QSize(size, size) * dpr);
  pm.setDevicePixelRatio(dpr);
  pm.fill(Qt::transparent);
  QPainter p(&pm);
  p.setRenderHint(QPainter::TextAntialiasing);
  p.setFont(theme::iconFont(size));
  p.setPen(color);
  p.drawText(QRect(0, 0, size, size), Qt::AlignCenter, QString(glyph));
  return pm;
}
}  // namespace

QIcon glyphIcon(QChar glyph, const QColor & color, int size, const QColor & checked_color)
{
  QIcon icon;
  for (qreal dpr : {1.0, 2.0}) {
    icon.addPixmap(glyphPixmap(glyph, color, size, dpr), QIcon::Normal, QIcon::Off);
    icon.addPixmap(glyphPixmap(glyph, checked_color.isValid() ? checked_color : color, size, dpr),
      QIcon::Normal, QIcon::On);
    icon.addPixmap(glyphPixmap(glyph, QColor(84, 84, 99), size, dpr), QIcon::Disabled, QIcon::Off);
    icon.addPixmap(glyphPixmap(glyph, QColor(84, 84, 99), size, dpr), QIcon::Disabled, QIcon::On);
  }
  return icon;
}

QPushButton * makeButton(const QString & text, QChar glyph, const QString & role)
{
  auto * button = new QPushButton(text);
  if (!glyph.isNull()) {
    button->setIcon(glyphIcon(glyph, theme::text, 16));
    button->setIconSize(QSize(16, 16));
  }
  if (!role.isEmpty()) {
    button->setProperty("role", role);
  }
  button->setCursor(Qt::PointingHandCursor);
  button->setFont(role == "seg" ? theme::font(12, QFont::Bold) : theme::font(13, QFont::DemiBold));
  return button;
}

QLabel * makeLabel(const QString & text, int pixel_size, int weight, const QColor & color)
{
  auto * label = new QLabel(text);
  label->setFont(theme::font(pixel_size, weight));
  if (color.isValid()) {
    label->setStyleSheet(QString("color: %1;").arg(theme::css(color)));
  }
  return label;
}

QLabel * makeCaption(const QString & text)
{
  auto * label = new QLabel(text.toUpper());
  QFont f = theme::font(11, QFont::Bold);
  f.setLetterSpacing(QFont::AbsoluteSpacing, 1.2);
  label->setFont(f);
  label->setStyleSheet(QString("color: %1;").arg(theme::css(theme::text3)));
  return label;
}

void repolish(QWidget * widget)
{
  widget->style()->unpolish(widget);
  widget->style()->polish(widget);
  widget->update();
}

// ---- ElidedLabel ----

void ElidedLabel::setFullText(const QString & text)
{
  full_ = text;
  setToolTip(text);
  refresh();
}

void ElidedLabel::resizeEvent(QResizeEvent * event)
{
  QLabel::resizeEvent(event);
  refresh();
}

void ElidedLabel::refresh()
{
  setText(fontMetrics().elidedText(full_, Qt::ElideRight, std::max(0, width() - 4)));
}

// ---- Card ----

Card::Card(const QString & title, QWidget * parent)
: QFrame(parent)
{
  setObjectName("Card");
  auto * outer = new QVBoxLayout(this);
  outer->setContentsMargins(14, 10, 14, 12);
  outer->setSpacing(8);
  header_ = new QHBoxLayout();
  header_->setSpacing(8);
  auto * tick = new QFrame();
  tick->setFixedSize(3, 13);
  tick->setStyleSheet(QString("background: %1; border: none;").arg(theme::css(theme::red)));
  title_ = makeCaption(title);
  title_->setStyleSheet(QString("color: %1;").arg(theme::css(theme::text2)));
  header_->addWidget(tick);
  header_->addWidget(title_);
  header_->addStretch(1);
  if (title.isEmpty()) {
    tick->hide();
    title_->hide();
  }
  outer->addLayout(header_);
  body_ = new QVBoxLayout();
  body_->setSpacing(8);
  outer->addLayout(body_, 1);
}

void Card::setTitle(const QString & title)
{
  title_->setText(title.toUpper());
}

// ---- StatusLed ----

StatusLed::StatusLed(QWidget * parent, int diameter)
: QWidget(parent), color_(theme::text3), diameter_(diameter)
{
  setFixedSize(diameter + 6, diameter + 6);
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() {phase_ = !phase_; update();});
}

void StatusLed::setColor(const QColor & color)
{
  if (color != color_) {
    color_ = color;
    update();
  }
}

void StatusLed::setBlinking(bool blinking)
{
  if (blinking == blinking_) {
    return;
  }
  blinking_ = blinking;
  phase_ = true;
  if (blinking) {
    timer_->start(450);
  } else {
    timer_->stop();
  }
  update();
}

QSize StatusLed::sizeHint() const {return QSize(diameter_ + 6, diameter_ + 6);}

void StatusLed::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  QColor c = (blinking_ && !phase_) ? color_.darker(300) : color_;
  QPointF center = rect().center() + QPointF(0.5, 0.5);
  QRadialGradient glow(center, diameter_ * 0.9);
  QColor halo = c;
  halo.setAlpha(90);
  glow.setColorAt(0.0, halo);
  glow.setColorAt(1.0, Qt::transparent);
  p.setPen(Qt::NoPen);
  p.setBrush(glow);
  p.drawEllipse(center, diameter_ * 0.9, diameter_ * 0.9);
  p.setBrush(c);
  p.drawEllipse(center, diameter_ / 2.0, diameter_ / 2.0);
}

// ---- Toggle ----

Toggle::Toggle(const QString & text, QWidget * parent)
: QAbstractButton(parent), accent_(theme::green)
{
  setText(text);
  setCheckable(true);
  setCursor(Qt::PointingHandCursor);
  setFont(theme::font(13, QFont::DemiBold));
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
}

QSize Toggle::sizeHint() const
{
  int text_width = text().isEmpty() ? 0 : fontMetrics().horizontalAdvance(text()) + 10;
  return QSize(40 + text_width, 26);
}

void Toggle::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  QRectF track(1, (height() - 18) / 2.0, 36, 18);
  QColor on = isEnabled() ? accent_ : accent_.darker(250);
  QColor off = isEnabled() ? theme::bg3 : theme::bg2;
  if (pending_) {
    on = on.darker(160);
    off = off.lighter(130);
  }
  p.setPen(Qt::NoPen);
  p.setBrush(isChecked() ? on : off);
  p.drawRoundedRect(track, 9, 9);
  double knob_x = isChecked() ? track.right() - 16 : track.left() + 2;
  p.setBrush(isEnabled() ? QColor(Qt::white) : theme::text3);
  p.drawEllipse(QRectF(knob_x, track.top() + 2, 14, 14));
  if (!text().isEmpty()) {
    p.setPen(isEnabled() ? theme::text : theme::text3);
    p.setFont(font());
    p.drawText(QRectF(track.right() + 10, 0, width() - track.right() - 10, height()),
      Qt::AlignVCenter | Qt::AlignLeft, text());
  }
}

// ---- NavButton ----

NavButton::NavButton(QChar glyph, const QString & text, QWidget * parent)
: QAbstractButton(parent), glyph_(glyph)
{
  setText(text);
  setCheckable(true);
  setCursor(Qt::PointingHandCursor);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
}

QSize NavButton::sizeHint() const {return QSize(84, 62);}

void NavButton::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  if (isChecked()) {
    p.fillRect(rect(), theme::bg2);
    p.fillRect(QRect(0, 0, 3, height()), theme::red);
  } else if (hover_) {
    p.fillRect(rect(), theme::bg1);
  }
  QColor c = isChecked() ? theme::text : (hover_ ? theme::text2 : theme::text3);
  p.setPen(c);
  p.setFont(theme::iconFont(24));
  p.drawText(QRect(0, 8, width(), 28), Qt::AlignCenter, QString(glyph_));
  QFont f = theme::font(10, QFont::Bold);
  f.setLetterSpacing(QFont::AbsoluteSpacing, 1.0);
  p.setFont(f);
  p.drawText(QRect(0, 38, width(), 16), Qt::AlignCenter, text().toUpper());
  if (badge_.isValid()) {
    p.setPen(Qt::NoPen);
    p.setBrush(badge_);
    p.drawEllipse(QPointF(width() / 2.0 + 16, 12), 4.5, 4.5);
  }
}

// ---- SkewBadge ----

SkewBadge::SkewBadge(QWidget * parent)
: QWidget(parent), color_(theme::bg3), text_color_(Qt::white)
{
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

void SkewBadge::set(const QString & text, const QColor & color, const QColor & text_color)
{
  text_ = text.toUpper();
  color_ = color;
  text_color_ = text_color;
  updateGeometry();
  update();
}

QSize SkewBadge::sizeHint() const
{
  QFont f = theme::font(pixel_size_, QFont::Black, true);
  f.setLetterSpacing(QFont::AbsoluteSpacing, 1.0);
  QFontMetrics fm(f);
  return QSize(fm.horizontalAdvance(text_) + 34, fm.height() + 8);
}

void SkewBadge::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  theme::fillSlanted(p, rect(), color_, height() * 0.35);
  QFont f = theme::font(pixel_size_, QFont::Black, true);
  f.setLetterSpacing(QFont::AbsoluteSpacing, 1.0);
  p.setFont(f);
  p.setPen(text_color_);
  p.drawText(rect(), Qt::AlignCenter, text_);
}

// ---- ValueTile ----

ValueTile::ValueTile(const QString & caption, const QString & unit, QWidget * parent)
: QWidget(parent), caption_(caption.toUpper()), unit_(unit), value_color_(theme::text)
{
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
}

void ValueTile::setValue(const QString & value, const QColor & color)
{
  value_ = value;
  value_color_ = color.isValid() ? color : theme::text;
  update();
}

void ValueTile::setBar(double fraction, const QColor & color)
{
  bar_ = fraction;
  bar_color_ = color;
  update();
}

QSize ValueTile::sizeHint() const {return QSize(140, 18 + QFontMetrics(theme::font(value_size_, QFont::Bold)).height() + 26);}
QSize ValueTile::minimumSizeHint() const {return QSize(90, sizeHint().height());}

void ValueTile::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  QFont caption = theme::font(11, QFont::Bold);
  caption.setLetterSpacing(QFont::AbsoluteSpacing, 1.2);
  p.setFont(caption);
  p.setPen(theme::text3);
  p.drawText(QRect(0, 0, width(), 16), Qt::AlignLeft | Qt::AlignVCenter, caption_);

  QFont value = theme::font(value_size_, QFont::Bold);
  p.setFont(value);
  QFontMetrics fm(value);
  int baseline = 18 + fm.ascent() - 2;
  p.setPen(value_color_);
  theme::drawTabular(p, QRectF(0, 18, width(), fm.height()), value_, Qt::AlignLeft | Qt::AlignTop);
  double value_width = 0;
  {
    QFontMetricsF fmf(value);
    double digit = fmf.horizontalAdvance('0');
    for (QChar c : value_) {
      value_width += c.isDigit() ? std::max(digit, fmf.horizontalAdvance(c)) : fmf.horizontalAdvance(c);
    }
  }
  if (!unit_.isEmpty()) {
    p.setFont(theme::font(13, QFont::DemiBold));
    p.setPen(theme::text2);
    p.drawText(QPointF(value_width + 5, baseline), unit_);
  }
  int y = 18 + fm.height();
  if (!detail_.isEmpty()) {
    p.setFont(theme::font(11));
    p.setPen(theme::text3);
    p.drawText(QRect(0, y, width(), 14), Qt::AlignLeft | Qt::AlignVCenter, detail_);
  }
  if (bar_ >= 0.0) {
    QRectF track(0, y + 18, width(), 4);
    p.setPen(Qt::NoPen);
    p.setBrush(theme::bg3);
    p.drawRoundedRect(track, 2, 2);
    p.setBrush(bar_color_);
    p.drawRoundedRect(QRectF(track.left(), track.top(), track.width() * std::clamp(bar_, 0.0, 1.0),
      track.height()), 2, 2);
  }
}
}  // namespace f1ui
