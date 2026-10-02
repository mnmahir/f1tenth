// F1 broadcast look: carbon blacks, F1 red, timing-screen purple/green/yellow, Titillium Web.
#pragma once

#include <QColor>
#include <QFont>
#include <QRectF>
#include <QString>

class QPainter;

namespace f1ui
{
namespace theme
{
// Surfaces, darkest first
inline const QColor bg0{0x0a, 0x0b, 0x0f};
inline const QColor bg1{0x15, 0x15, 0x1e};
inline const QColor bg2{0x1f, 0x1f, 0x2b};
inline const QColor bg3{0x2a, 0x2a, 0x39};
inline const QColor line{0x38, 0x38, 0x4b};
// Text
inline const QColor text{0xff, 0xff, 0xff};
inline const QColor text2{0xb4, 0xb4, 0xc3};
inline const QColor text3{0x74, 0x74, 0x86};
// Accents
inline const QColor red{0xe1, 0x06, 0x00};
inline const QColor redBright{0xff, 0x2a, 0x1f};
inline const QColor purple{0xb1, 0x38, 0xdd};   // fastest / session best
inline const QColor green{0x00, 0xd2, 0x5a};    // ok / personal best
inline const QColor yellow{0xff, 0xd1, 0x2e};   // warning / slower
inline const QColor orange{0xff, 0x87, 0x00};
inline const QColor cyan{0x00, 0xe1, 0xff};     // lidar, information
inline const QColor blue{0x2d, 0x7f, 0xf9};

// Session modes
QColor modeColor(const QString & mode);
QString modeTitle(const QString & mode);
// diagnostic_msgs/DiagnosticStatus level (OK, WARN, ERROR, STALE)
QColor levelColor(int level);
// rcl_interfaces/Log level (10 debug ... 50 fatal)
QColor logColor(int level);
// Blue -> cyan -> green -> yellow -> red, for 0..1
QColor heat(double t);

QString css(const QColor & c);
QString styleSheet();

// Registers the bundled fonts (call once, after QApplication)
void loadFonts();
QFont font(int pixel_size, int weight = QFont::Normal, bool italic = false);
QFont mono(int pixel_size, int weight = QFont::Normal);
QFont iconFont(int pixel_size);

// Material Icons code points
namespace icon
{
inline const QChar home{0xe88a};
inline const QChar speed{0xe9e4};
inline const QChar map{0xe55b};
inline const QChar route{0xeacd};
inline const QChar timeline{0xe922};
inline const QChar insights{0xf092};
inline const QChar monitor{0xeaa2};
inline const QChar tune{0xe429};
inline const QChar terminal{0xeb8e};
inline const QChar garage{0xf011};
inline const QChar warning{0xe002};
inline const QChar error{0xe000};
inline const QChar check{0xe86c};
inline const QChar play{0xe037};
inline const QChar stop{0xe047};
inline const QChar pause{0xe034};
inline const QChar refresh{0xe5d5};
inline const QChar restart{0xf053};
inline const QChar save{0xe161};
inline const QChar trash{0xe872};
inline const QChar edit{0xe3c9};
inline const QChar videocam{0xe04b};
inline const QChar videocamOff{0xe04c};
inline const QChar flag{0xf06e};
inline const QChar gamepad{0xea28};
inline const QChar battery{0xe1a4};
inline const QChar thermostat{0xf076};
inline const QChar wifi{0xe63e};
inline const QChar bolt{0xea0b};
inline const QChar power{0xe8ac};
inline const QChar pose{0xe55c};
inline const QChar measure{0xe41c};
inline const QChar layers{0xe53b};
inline const QChar orbit{0xe84d};
inline const QChar record{0xe061};
inline const QChar car{0xe531};
inline const QChar timer{0xe425};
inline const QChar memory{0xe322};
inline const QChar storage{0xe1db};
inline const QChar hub{0xe9f4};
inline const QChar list{0xe896};
inline const QChar search{0xe8b6};
inline const QChar magic{0xe663};
inline const QChar emergency{0xe1eb};
inline const QChar block{0xe14b};
inline const QChar radar{0xf04e};
inline const QChar sensors{0xe51e};
inline const QChar link{0xe157};
inline const QChar linkOff{0xe16f};
inline const QChar info{0xe88e};
inline const QChar swap{0xe8d4};
inline const QChar expand{0xf1ce};
inline const QChar collapse{0xf1cf};
inline const QChar camera{0xe3b0};
inline const QChar rocket{0xeb9b};
inline const QChar navigation{0xe55d};
inline const QChar focus{0xe3b4};
inline const QChar grid{0xe3ec};
inline const QChar visibility{0xe8f4};
}  // namespace icon

// Painting helpers shared by the custom widgets
// Text with fixed-width digits, so changing numbers don't jitter (Titillium Web digits are proportional)
void drawTabular(QPainter & p, const QRectF & rect, const QString & text, Qt::Alignment align);
// F1-style slanted box (parallelogram leaning right)
void fillSlanted(QPainter & p, const QRectF & rect, const QColor & color, double slant = 8.0);
}  // namespace theme
}  // namespace f1ui
