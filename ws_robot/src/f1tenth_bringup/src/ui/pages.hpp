// The UI's pages. Each page talks to the car only through RosBridge and reports results with message().
#pragma once

#include <QMap>
#include <QPointer>
#include <QSet>
#include <QWidget>

#include <map>
#include <vector>

#include "ros_bridge.hpp"
#include "scene_view.hpp"

class QAbstractButton;
class QButtonGroup;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QListWidgetItem;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QSlider;
class QStackedWidget;
class QTableView;
class QTableWidget;
class QTabWidget;

namespace f1ui
{
class BatteryGauge;
class CameraView;
class ElidedLabel;
class Card;
class FlagStrip;
class GForceMeter;
class PedalBars;
class ShiftLights;
class SkewBadge;
class SpeedGauge;
class StatusLed;
class SteeringWheel;
class StripChart;
class TimingTower;
class Toggle;
class TrackMap;
class ValueTile;

enum class Level {Info, Ok, Warn, Error};

// Paths usable on a map: those made on it, then those whose map is unknown (older files), labelled as such.
// Returns (name, label) pairs.
std::vector<std::pair<QString, QString>> pathsForMap(const SupervisorState & state, const QString & map);
// The map a path was made on, '' if unknown
QString pathMap(const SupervisorState & state, const QString & path);

// Base for pages: reports outcomes to the status bar
class Page : public QWidget
{
  Q_OBJECT

public:
  explicit Page(RosBridge * ros, QWidget * parent = nullptr);

Q_SIGNALS:
  void message(f1ui::Level level, const QString & text);

protected:
  // Reply callback that reports the outcome
  ReplyFn report(const QString & context = QString());
  RosBridge * ros_;
};

// ---- HOME: session selection ----

class ModeTile;

class HomePage : public Page
{
  Q_OBJECT

public:
  explicit HomePage(RosBridge * ros, QWidget * parent = nullptr);
  // The car software is installed here: the UI runs on the car itself
  static bool isCar();
  void selectMap(const QString & map);
  void selectPath(const QString & path);

Q_SIGNALS:
  void sessionStarted(const QString & mode);

private:
  void onSupervisor(const SupervisorState & state);
  void onLink(bool car, bool supervisor);
  void onTelemetry(const Telemetry & t);
  void selectMode(const QString & mode);
  void startCarSoftware();
  void refreshPreview();
  void validate();
  void start();

  QString mode_ = "manual";
  std::map<QString, ModeTile *> tiles_;
  QComboBox * map_combo_ = nullptr;
  QComboBox * path_combo_ = nullptr;
  TrackMap * preview_ = nullptr;
  QLabel * info_ = nullptr;
  QLabel * hint_ = nullptr;
  QPushButton * start_ = nullptr;
  StatusLed * link_led_ = nullptr;
  QLabel * link_text_ = nullptr;
  QLabel * battery_text_ = nullptr;
  QPushButton * start_car_ = nullptr;
  SkewBadge * running_mode_ = nullptr;
  QLabel * running_detail_ = nullptr;
  QWidget * components_ = nullptr;
  QStringList maps_;
  QString path_map_filter_;   // map the path list was last built for
  SupervisorState state_;
  bool have_state_ = false;
  QString preview_map_, preview_path_;
  std::map<QString, MapPreview> preview_cache_;
};

// ---- Session (DRIVE / MAPPING / PATH): 3D scene, camera, mode panel and the HUD ----

// Whether the car knows where it is on the map, and the three ways to tell it: START LINE (the car stands on the
// selected path's start), FIND CAR (the car searches the map for where its lidar scan fits) and SET POSE (by hand)
class LocalizationControls : public QWidget
{
  Q_OBJECT

public:
  explicit LocalizationControls(RosBridge * ros, QWidget * parent = nullptr);
  bool localized() const {return localized_;}

Q_SIGNALS:
  void setPoseRequested();
  void changed();
  void message(f1ui::Level level, const QString & text);

private:
  void onLocalization(const LocalizationState & state);
  void refresh();

  RosBridge * ros_;
  StatusLed * led_ = nullptr;
  QLabel * status_ = nullptr;
  QLabel * match_ = nullptr;
  QProgressBar * bar_ = nullptr;
  QLabel * hint_ = nullptr;
  QPushButton * start_line_ = nullptr;
  QPushButton * find_ = nullptr;
  QPushButton * set_pose_ = nullptr;
  bool localized_ = false, active_ = false, has_path_ = false, busy_ = false;
  double match_value_ = 0.0;
  QString state_message_;
  qint64 last_ms_ = 0;
};

class ChecklistRow;

class HudStrip : public QWidget
{
  Q_OBJECT

public:
  explicit HudStrip(RosBridge * ros, QWidget * parent = nullptr);
  void setMaxSpeed(double speed);

private:
  void onTelemetry(const Telemetry & t);
  void onImu(double ax, double ay, double az);

  RosBridge * ros_;
  qint64 localization_ms_ = 0;
  ShiftLights * lights_ = nullptr;
  PedalBars * pedals_ = nullptr;
  SpeedGauge * speed_ = nullptr;
  SteeringWheel * wheel_ = nullptr;
  BatteryGauge * battery_ = nullptr;
  ValueTile * temp_fet_ = nullptr;
  ValueTile * motor_current_ = nullptr;
  ValueTile * brake_current_ = nullptr;
  ValueTile * distance_ = nullptr;
  GForceMeter * g_meter_ = nullptr;
  FlagStrip * flags_ = nullptr;
  double max_boost_ = 5.0;
  double g_lat_ = 0.0, g_lon_ = 0.0;
  bool imu_seen_ = false;
};

class DrivePanel : public Page
{
  Q_OBJECT

public:
  explicit DrivePanel(RosBridge * ros, QWidget * parent = nullptr);
  void refreshParameters();
  LocalizationControls * localization() const {return localization_;}

private:
  void onDrive(const DriveState & d);
  void updateChecklist();
  void onParameterEvent(const ParameterEvent & event);
  void applyParameter(const QString & node, const rclcpp::Parameter & p);

  SkewBadge * source_ = nullptr;
  QLabel * drive_message_ = nullptr;
  QPushButton * autonomous_ = nullptr;
  QLabel * autonomous_hint_ = nullptr;
  ChecklistRow * track_row_ = nullptr;
  ChecklistRow * follower_row_ = nullptr;
  ChecklistRow * joystick_row_ = nullptr;
  LocalizationControls * localization_ = nullptr;
  QString map_, path_;
  bool joystick_ = false;
  bool estop_ = false;
  Toggle * assist_ = nullptr;
  Toggle * safety_ = nullptr;
  Toggle * avoidance_ = nullptr;
  QSlider * auto_speed_ = nullptr;
  QLabel * auto_speed_value_ = nullptr;
  QSlider * boost_ = nullptr;
  QLabel * boost_value_ = nullptr;
  TimingTower * timing_ = nullptr;
  DriveState drive_;
};

class MappingPanel : public Page
{
  Q_OBJECT

public:
  explicit MappingPanel(RosBridge * ros, QWidget * parent = nullptr);
  void setMapInfo(int width, int height, double resolution, double known_fraction);

Q_SIGNALS:
  void openParameters(const QString & node);
  void openGarage();

private:
  void onSupervisor(const SupervisorState & state);

  StatusLed * led_ = nullptr;
  QLabel * status_ = nullptr;
  ValueTile * size_ = nullptr;
  ValueTile * explored_ = nullptr;
  ValueTile * resolution_ = nullptr;
  QPushButton * pause_ = nullptr;
  QLineEdit * name_ = nullptr;
  QPushButton * save_ = nullptr;
  QLabel * saved_count_ = nullptr;
  bool paused_ = false;
};

class PathPanel : public Page
{
  Q_OBJECT

public:
  explicit PathPanel(RosBridge * ros, QWidget * parent = nullptr);
  LocalizationControls * localization() const {return localization_;}

Q_SIGNALS:
  void showRaceline();

private:
  void onSupervisor(const SupervisorState & state);
  void onRecorder(const RecorderState & state);
  void optimize();

  LocalizationControls * localization_ = nullptr;
  QPushButton * record_ = nullptr;
  StatusLed * rec_led_ = nullptr;
  ValueTile * points_ = nullptr;
  ValueTile * length_ = nullptr;
  ValueTile * top_speed_ = nullptr;
  QLineEdit * record_name_ = nullptr;
  QComboBox * source_ = nullptr;
  QDoubleSpinBox * margin_ = nullptr;
  QDoubleSpinBox * max_speed_ = nullptr;
  QDoubleSpinBox * lateral_ = nullptr;
  QDoubleSpinBox * accel_ = nullptr;
  QDoubleSpinBox * decel_ = nullptr;
  QPushButton * optimize_ = nullptr;
  QLabel * result_ = nullptr;
  QLineEdit * raceline_name_ = nullptr;
  QPushButton * save_raceline_ = nullptr;
  QPushButton * drive_it_ = nullptr;
  bool recording_ = false;
  QString map_;
  QString saved_raceline_;
};

class SessionPage : public Page
{
  Q_OBJECT

public:
  SessionPage(RosBridge * ros, const std::string & rviz_node_name, QWidget * parent = nullptr);
  void initializeScene();
  SceneView * scene() const {return scene_;}
  QString title() const;
  void setActive(bool active);  // page visible (camera decoding follows it)

Q_SIGNALS:
  void titleChanged(const QString & title);
  void openParameters(const QString & node);
  void openGarage();
  void openHome();

private:
  void onSupervisor(const SupervisorState & state);
  void setMode(const QString & mode);
  void swapCamera();
  void startPoseOnMap();
  void updateCameraWanted();
  void applyLayerDefaults(const QString & mode);

  SceneView * scene_ = nullptr;
  QStackedWidget * big_ = nullptr;
  QStackedWidget * small_ = nullptr;
  CameraView * camera_big_ = nullptr;
  CameraView * camera_small_ = nullptr;
  TrackMap * minimap_ = nullptr;
  QStackedWidget * panels_ = nullptr;
  DrivePanel * drive_ = nullptr;
  MappingPanel * mapping_ = nullptr;
  PathPanel * path_ = nullptr;
  QWidget * idle_ = nullptr;
  HudStrip * hud_ = nullptr;
  ElidedLabel * status_ = nullptr;
  std::map<SceneView::View, QPushButton *> view_buttons_;
  QPushButton * follow_ = nullptr;
  QPushButton * pose_ = nullptr;
  QPushButton * measure_ = nullptr;
  QString mode_;
  QString map_;
  QString session_path_;
  bool camera_big_shown_ = false;
  bool active_ = false;
  std::map<SceneView::Layer, bool> user_layers_;  // layers the user toggled by hand
};

// ---- GARAGE: maps and paths on the car ----

class GaragePage : public Page
{
  Q_OBJECT

public:
  explicit GaragePage(RosBridge * ros, QWidget * parent = nullptr);
  void setActive(bool active);

Q_SIGNALS:
  void useMap(const QString & map);
  void usePath(const QString & path);

private:
  void onSupervisor(const SupervisorState & state);
  void showMap(const QString & name);
  void showPath(const QString & name);
  void fetchThumbnails();
  void rename();
  void remove();

  QListWidget * maps_ = nullptr;
  QListWidget * paths_ = nullptr;
  TrackMap * preview_ = nullptr;
  QLabel * title_ = nullptr;
  QLabel * info_ = nullptr;
  QPushButton * use_ = nullptr;
  QPushButton * rename_ = nullptr;
  QPushButton * delete_ = nullptr;
  QString kind_;   // map | path
  QString selected_;
  QString shown_map_;
  QStringList map_names_, path_names_;
  QSet<QString> thumbnails_;
  bool active_ = false;
  SupervisorState state_;
};

// ---- TELEMETRY: charts and session stats ----

class TelemetryPage : public Page
{
  Q_OBJECT

public:
  explicit TelemetryPage(RosBridge * ros, QWidget * parent = nullptr);

private:
  void onTelemetry(const Telemetry & t);
  void onLap(const LapState & lap);

  StripChart * speed_ = nullptr;
  StripChart * steering_ = nullptr;
  StripChart * current_ = nullptr;
  StripChart * battery_ = nullptr;
  StripChart * duty_ = nullptr;
  StripChart * temps_ = nullptr;
  int speed_actual_ = 0, speed_cmd_ = 0, steer_actual_ = 0, steer_cmd_ = 0;
  int motor_ = 0, input_ = 0, brake_ = 0, volts_ = 0, duty_series_ = 0, fet_ = 0, motor_temp_ = 0;
  GForceMeter * g_meter_ = nullptr;
  ValueTile * top_speed_ = nullptr;
  ValueTile * distance_ = nullptr;
  ValueTile * energy_ = nullptr;
  ValueTile * peak_current_ = nullptr;
  QTableWidget * laps_ = nullptr;
  double start_time_ = -1.0;
  double top_speed_value_ = 0.0, peak_current_value_ = 0.0;
  double start_distance_ = -1.0, start_energy_ = -1.0;
  uint32_t lap_count_ = 0;
  double best_lap_ = 0.0;
  bool paused_ = false;
};

// ---- SYSTEM: components, nodes, health ----

class SystemPage : public Page
{
  Q_OBJECT

public:
  explicit SystemPage(RosBridge * ros, QWidget * parent = nullptr);

Q_SIGNALS:
  void healthChanged(int level);  // worst of components and diagnostics

private:
  void onSupervisor(const SupervisorState & state);
  void onDiagnostics(const DiagnosticArray & diagnostics);
  void onNodes(const QStringList & nodes);
  void updateHealth();

  StatusLed * supervisor_led_ = nullptr;
  QLabel * supervisor_text_ = nullptr;
  Toggle * auto_restart_ = nullptr;
  Toggle * bag_ = nullptr;
  QTableWidget * components_ = nullptr;
  std::map<QString, ValueTile *> tiles_;
  QTableWidget * rates_ = nullptr;
  QListWidget * nodes_ = nullptr;
  QLabel * nodes_count_ = nullptr;
  QStringList missing_;
  QStringList graph_;
  int component_level_ = 0, diagnostic_level_ = 0;
  bool standby_ = false;
};

// ---- PARAMS: live parameters of any node, saved to the car ----

class ParamsPage : public Page
{
  Q_OBJECT

public:
  explicit ParamsPage(RosBridge * ros, QWidget * parent = nullptr);
  void showNode(const QString & node);

private:
  void onNodes(const QStringList & nodes);
  void load(const QString & node);
  void fill(const std::vector<ParamInfo> & params);
  void onParameterEvent(const ParameterEvent & event);
  void commit(int row);
  void filterRows();

  QLineEdit * node_filter_ = nullptr;
  QListWidget * node_list_ = nullptr;
  QLabel * node_title_ = nullptr;
  QLineEdit * param_filter_ = nullptr;
  QTableWidget * table_ = nullptr;
  QPushButton * refresh_ = nullptr;
  QPushButton * save_ = nullptr;
  QLabel * state_ = nullptr;
  QString node_;
  std::vector<ParamInfo> params_;
  bool filling_ = false;
};

// ---- LOG: /rosout and the components' terminal output ----

class LogModel;

class LogPage : public Page
{
  Q_OBJECT

public:
  explicit LogPage(RosBridge * ros, QWidget * parent = nullptr);
  void loadHistory();

private:
  void onRosout(const std::vector<Log> & batch);
  void onConsole(const std::vector<Log> & batch);
  void appendConsole(const Log & line);
  void rebuildConsole();

  QTabWidget * tabs_ = nullptr;
  LogModel * model_ = nullptr;
  QTableView * table_ = nullptr;
  QComboBox * level_ = nullptr;
  QComboBox * node_ = nullptr;
  QLineEdit * search_ = nullptr;
  QPushButton * pause_ = nullptr;
  QPlainTextEdit * console_ = nullptr;
  QComboBox * component_ = nullptr;
  std::vector<Log> console_lines_;
  bool history_loaded_ = false;
};
}  // namespace f1ui

Q_DECLARE_METATYPE(f1ui::Level)
