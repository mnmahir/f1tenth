#include "theme.hpp"

#include <QFontDatabase>
#include <QFontMetricsF>
#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>

namespace f1ui
{
namespace theme
{
QColor modeColor(const QString & mode)
{
  if (mode == "manual") {return red;}
  if (mode == "mapping") {return cyan;}
  if (mode == "path") {return purple;}
  return text3;
}

QString modeTitle(const QString & mode)
{
  if (mode == "manual") {return "MANUAL DRIVE";}
  if (mode == "mapping") {return "MAPPING";}
  if (mode == "path") {return "PATH & RACELINE";}
  if (mode == "idle") {return "STANDBY";}
  return "OFFLINE";
}

QColor levelColor(int level)
{
  switch (level) {
    case 0: return green;
    case 1: return yellow;
    case 2: return red;
    default: return text3;
  }
}

QColor logColor(int level)
{
  if (level >= 50) {return purple;}
  if (level >= 40) {return redBright;}
  if (level >= 30) {return yellow;}
  if (level >= 20) {return text2;}
  return text3;
}

QColor heat(double t)
{
  static const QColor stops[] = {blue, cyan, green, yellow, redBright};
  t = std::clamp(t, 0.0, 1.0) * 4.0;
  int i = std::min(static_cast<int>(t), 3);
  double f = t - i;
  const QColor & a = stops[i];
  const QColor & b = stops[i + 1];
  return QColor::fromRgbF(
    a.redF() + (b.redF() - a.redF()) * f,
    a.greenF() + (b.greenF() - a.greenF()) * f,
    a.blueF() + (b.blueF() - a.blueF()) * f);
}

QString css(const QColor & c) {return c.name(QColor::HexRgb);}

QString styleSheet()
{
  QString s = R"(
QWidget { color: @text; }
QMainWindow, QWidget#Root, QStackedWidget#Pages > QWidget { background: @bg0; }
QToolTip { background: @bg2; color: @text; border: 1px solid @line; padding: 4px 6px; }

QFrame#Card { background: @bg1; border: 1px solid #23232f; border-radius: 6px; }
QFrame#Inset { background: #101016; border: 1px solid #23232f; border-radius: 4px; }
QLabel { background: transparent; }
QLabel[muted="true"] { color: @text3; }
QLabel[dim="true"] { color: @text2; }

QPushButton {
  background: @bg2; border: 1px solid @line; border-radius: 4px;
  padding: 6px 14px; color: @text; min-height: 18px;
}
QPushButton:hover { background: @bg3; border-color: #4c4c63; }
QPushButton:pressed { background: @bg1; }
QPushButton:disabled { color: #545463; background: #17171f; border-color: #25252f; }
QPushButton:checked { background: #3a1214; border-color: @red; color: @text; }
QPushButton::menu-indicator { image: none; width: 0px; }
QPushButton[role="primary"] { background: @red; border-color: @redBright; }
QPushButton[role="primary"]:hover { background: @redBright; }
QPushButton[role="primary"]:disabled { background: #3a1a1a; border-color: #4a2020; color: #8a6a6a; }
QPushButton[role="go"] { background: #0d3b22; border-color: @green; }
QPushButton[role="go"]:hover { background: #125a33; }
QPushButton[role="go"]:disabled { background: #111a15; border-color: #1f3328; color: #4d6657; }
QPushButton[role="ghost"] { background: transparent; border: 1px solid transparent; color: @text2; }
QPushButton[role="ghost"]:hover { color: @text; background: @bg2; }
QPushButton[role="seg"] {
  background: @bg1; border: 1px solid @line; border-radius: 0px; padding: 5px 12px;
  color: @text2;
}
QPushButton[role="seg"]:hover { color: @text; background: @bg2; }
QPushButton[role="seg"]:checked { background: @red; border-color: @red; color: @text; }

QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
  background: #0f0f15; border: 1px solid @line; border-radius: 4px; padding: 5px 8px;
  selection-background-color: @red; color: @text; min-height: 18px;
}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus { border-color: @red; }
QLineEdit:disabled, QComboBox:disabled { color: #545463; }
QComboBox QAbstractItemView {
  background: @bg1; border: 1px solid @line; selection-background-color: @bg3; outline: 0; padding: 2px;
}

QScrollBar:vertical { background: transparent; width: 10px; margin: 0; }
QScrollBar::handle:vertical { background: @line; border-radius: 3px; min-height: 30px; margin: 2px; }
QScrollBar::handle:vertical:hover { background: #4c4c63; }
QScrollBar:horizontal { background: transparent; height: 10px; margin: 0; }
QScrollBar::handle:horizontal { background: @line; border-radius: 3px; min-width: 30px; margin: 2px; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: none; }

QTableView, QListView, QTreeView, QPlainTextEdit, QTextEdit {
  background: #0f0f15; border: 1px solid #23232f; border-radius: 4px; gridline-color: #1c1c26;
  alternate-background-color: #121219; selection-background-color: #3a1214; selection-color: @text; outline: 0;
}
QListView::item { padding: 6px 8px; border-radius: 3px; }
QListView::item:selected { background: #3a1214; }
QListView::item:hover:!selected { background: @bg2; }
QHeaderView { background: @bg1; }
QHeaderView::section {
  background: @bg1; color: @text3; border: none; border-bottom: 1px solid @line;
  padding: 6px 8px; font-weight: 700; font-size: 11px;
}
QTableCornerButton::section { background: @bg1; border: none; }

QTabWidget::pane { border: none; }
QTabBar::tab {
  background: transparent; color: @text3; padding: 8px 18px; font-weight: 700;
  border: none; border-bottom: 2px solid transparent;
}
QTabBar::tab:selected { color: @text; border-bottom: 2px solid @red; }
QTabBar::tab:hover:!selected { color: @text2; }

QSlider::groove:horizontal { height: 4px; background: @bg3; border-radius: 2px; }
QSlider::sub-page:horizontal { background: @red; border-radius: 2px; }
QSlider::handle:horizontal { background: @text; width: 14px; height: 14px; margin: -5px 0; border-radius: 7px; }

QCheckBox { spacing: 8px; }
QCheckBox::indicator { width: 15px; height: 15px; border: 1px solid #4c4c63; border-radius: 3px; background: #0f0f15; }
QCheckBox::indicator:checked { background: @red; border-color: @redBright; }

QMenu { background: @bg1; border: 1px solid @line; padding: 4px; }
QMenu::item { padding: 6px 24px 6px 12px; border-radius: 3px; }
QMenu::item:selected { background: @bg3; }
QMenu::separator { height: 1px; background: @line; margin: 4px 6px; }
QMenu::indicator { width: 14px; height: 14px; margin-left: 4px; }

QSplitter::handle { background: @bg0; }
QProgressBar { background: #0f0f15; border: 1px solid @line; border-radius: 3px; text-align: center; height: 8px; }
QProgressBar::chunk { background: @red; border-radius: 2px; }
QDialog { background: @bg1; }
)";
  const std::pair<const char *, QColor> colors[] = {
    {"@bg0", bg0}, {"@bg1", bg1}, {"@bg2", bg2}, {"@bg3", bg3}, {"@line", line}, {"@text3", text3},
    {"@text2", text2}, {"@text", text}, {"@redBright", redBright}, {"@red", red}, {"@green", green}};
  for (const auto & [key, color] : colors) {
    s.replace(key, css(color));
  }
  return s;
}

void loadFonts()
{
  for (const char * file : {
      ":/fonts/TitilliumWeb-Regular.ttf", ":/fonts/TitilliumWeb-SemiBold.ttf", ":/fonts/TitilliumWeb-Bold.ttf",
      ":/fonts/TitilliumWeb-Black.ttf", ":/fonts/IBMPlexMono-Regular.ttf", ":/fonts/IBMPlexMono-SemiBold.ttf",
      ":/fonts/MaterialIcons-Regular.ttf"})
  {
    QFontDatabase::addApplicationFont(file);
  }
}

QFont font(int pixel_size, int weight, bool italic)
{
  QFont f("Titillium Web");
  f.setPixelSize(pixel_size);
  f.setWeight(weight);
  f.setItalic(italic);
  return f;
}

QFont mono(int pixel_size, int weight)
{
  QFont f("IBM Plex Mono");
  f.setPixelSize(pixel_size);
  f.setWeight(weight);
  f.setStyleHint(QFont::Monospace);
  return f;
}

QFont iconFont(int pixel_size)
{
  QFont f("Material Icons");
  f.setPixelSize(pixel_size);
  return f;
}

void drawTabular(QPainter & p, const QRectF & rect, const QString & text, Qt::Alignment align)
{
  QFontMetricsF fm(p.font());
  double digit = 0.0;
  for (QChar c : QStringLiteral("0123456789")) {
    digit = std::max(digit, fm.horizontalAdvance(c));
  }
  auto advance = [&](QChar c) {return c.isDigit() ? digit : fm.horizontalAdvance(c);};
  double width = 0.0;
  for (QChar c : text) {
    width += advance(c);
  }
  double x = rect.left();
  if (align & Qt::AlignRight) {
    x = rect.right() - width;
  } else if (align & Qt::AlignHCenter) {
    x = rect.center().x() - width / 2.0;
  }
  double y;
  if (align & Qt::AlignTop) {
    y = rect.top() + fm.ascent();
  } else if (align & Qt::AlignBottom) {
    y = rect.bottom() - fm.descent();
  } else {
    y = rect.center().y() + (fm.ascent() - fm.descent()) / 2.0;
  }
  for (QChar c : text) {
    double w = advance(c);
    double offset = c.isDigit() ? (w - fm.horizontalAdvance(c)) / 2.0 : 0.0;
    p.drawText(QPointF(x + offset, y), QString(c));
    x += w;
  }
}

void fillSlanted(QPainter & p, const QRectF & r, const QColor & color, double slant)
{
  QPainterPath path;
  path.moveTo(r.left() + slant, r.top());
  path.lineTo(r.right(), r.top());
  path.lineTo(r.right() - slant, r.bottom());
  path.lineTo(r.left(), r.bottom());
  path.closeSubpath();
  p.fillPath(path, color);
}
}  // namespace theme
}  // namespace f1ui
