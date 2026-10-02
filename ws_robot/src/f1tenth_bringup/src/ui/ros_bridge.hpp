// The UI's link to the car. ROS runs on its own executor threads; every signal here is emitted on the
// GUI thread (high-rate topics are coalesced to the latest message), and every reply callback runs on the
// GUI thread too. All car-side actions go through services, so the UI works the same on a laptop.
#pragma once

#include <QImage>
#include <QObject>
#include <QPointF>
#include <QSize>
#include <QStringList>
#include <QVector>

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "f1tenth_bringup/msg/drive_state.hpp"
#include "f1tenth_bringup/msg/lap_state.hpp"
#include "f1tenth_bringup/msg/localization_state.hpp"
#include "f1tenth_bringup/msg/recorder_state.hpp"
#include "f1tenth_bringup/msg/supervisor_state.hpp"
#include "f1tenth_bringup/msg/telemetry.hpp"
#include "f1tenth_bringup/srv/optimize_raceline.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"

namespace f1ui
{
using Telemetry = f1tenth_bringup::msg::Telemetry;
using DriveState = f1tenth_bringup::msg::DriveState;
using LapState = f1tenth_bringup::msg::LapState;
using LocalizationState = f1tenth_bringup::msg::LocalizationState;
using RecorderState = f1tenth_bringup::msg::RecorderState;
using SupervisorState = f1tenth_bringup::msg::SupervisorState;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using Log = rcl_interfaces::msg::Log;
using ParameterEvent = rcl_interfaces::msg::ParameterEvent;

using ReplyFn = std::function<void(bool ok, const QString & message)>;

struct MapPreview
{
  bool ok = false;
  QString message;
  QImage image;              // grayscale, top row = +y; 0 occupied, 255 free, 127 unknown
  double resolution = 0.05;  // m per image pixel
  double origin_x = 0.0;     // world position of the image's lower-left corner
  double origin_y = 0.0;
};

struct PathData
{
  bool ok = false;
  QString message;
  QVector<QPointF> points;   // map frame, m
  QVector<float> speeds;     // m/s
  double length() const;
};

struct RacelineResult
{
  bool ok = false;
  QString message;
  double length = 0.0;
  double lap_time = 0.0;
  double min_width = 0.0;
};

struct ParamInfo
{
  rclcpp::Parameter value;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
};

class RosBridge : public QObject
{
  Q_OBJECT

public:
  explicit RosBridge(QObject * parent = nullptr);
  ~RosBridge() override;

  rclcpp::Node::SharedPtr node() const {return node_;}

  // Session and files (supervisor on the car)
  void setMode(const QString & mode, const QString & map, const QString & path, ReplyFn done);
  void componentCommand(const QString & name, const QString & action, ReplyFn done);
  void fileOp(const QString & kind, const QString & action, const QString & name, const QString & new_name,
    ReplyFn done);
  void setAutoRestart(bool on, ReplyFn done);
  void setBagRecording(bool on, ReplyFn done);
  void saveMap(const QString & name, ReplyFn done);
  void saveParams(const QString & node, ReplyFn done);
  void mapPreview(const QString & name, int max_size, std::function<void(const MapPreview &)> done);
  void path(const QString & name, std::function<void(const PathData &)> done);
  void consoleHistory(int max_lines, std::function<void(const std::vector<Log> &)> done);
  void rosoutHistory(int max_lines, std::function<void(const std::vector<Log> &)> done);

  // Driving
  void setAutonomous(bool on, ReplyFn done);
  void setAssist(bool on, ReplyFn done);
  void setEstop(bool engaged);
  bool estopHeldHere() const {return estop_local_;}

  // Path recording, raceline, SLAM, laps
  void recorder(const QString & action, ReplyFn done);  // start | stop | clear
  void saveRecording(const QString & name, ReplyFn done);
  void optimizeRaceline(const f1tenth_bringup::srv::OptimizeRaceline::Request & request,
    std::function<void(const RacelineResult &)> done);
  void saveRaceline(const QString & name, ReplyFn done);
  void toggleMappingPause(ReplyFn done);
  // Localization on the map: put the car on the selected path's start line, or let the car find itself
  void placeAtStartLine(ReplyFn done);
  void locateCar(ReplyFn done);
  void resetLap(ReplyFn done);

  // Parameters of any node ("/name" or "/ns/name")
  void loadParameters(const QString & node,
    std::function<void(bool ok, const QString & error, const std::vector<ParamInfo> & params)> done);
  void getParameters(const QString & node, const QStringList & names,
    std::function<void(bool ok, const std::vector<rclcpp::Parameter> & values)> done);
  void setParameter(const QString & node, const rclcpp::Parameter & param, ReplyFn done);

  QStringList nodes() const {return nodes_;}
  bool carLinked() const {return car_linked_;}
  bool supervisorLinked() const {return supervisor_linked_;}
  double telemetryRate() const {return telemetry_rate_;}

  // Camera frames are only decoded while a camera view is visible, scaled down to fit max_size
  void setCameraWanted(bool wanted, QSize max_size);

Q_SIGNALS:
  void telemetry(const Telemetry & msg);
  void driveState(const DriveState & msg);
  void lapState(const LapState & msg);
  void localization(const LocalizationState & msg);
  void supervisorState(const SupervisorState & msg);
  void recorderState(const RecorderState & msg);
  void diagnostics(const DiagnosticArray & msg);
  void rosout(const std::vector<Log> & batch);
  void console(const std::vector<Log> & batch);
  void parameterEvent(const ParameterEvent & msg);
  void cameraFrame(const QImage & image, double fps);
  void imu(double ax, double ay, double az);
  void nodesChanged(const QStringList & nodes);
  void estopChanged(bool engaged, bool held_here);
  void linkChanged(bool car, bool supervisor);

private:
  template<typename T>
  struct Mailbox
  {
    std::mutex mutex;
    T value;
    std::atomic<bool> pending{false};
  };

  // Keeps only the newest value; the GUI picks it up once per event loop pass
  template<typename T, typename Emit>
  void post(Mailbox<T> & box, T value, Emit emit_fn);
  // Keeps every value (logs)
  void postLogs(std::vector<Log> & buffer, std::atomic<bool> & pending, const Log & msg, bool is_console);

  template<typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr client(const std::string & name);
  // Sends a request; on_response or on_error runs on the GUI thread (never both)
  template<typename ServiceT>
  void call(const std::string & service, std::shared_ptr<typename ServiceT::Request> request,
    std::function<void(std::shared_ptr<typename ServiceT::Response>)> on_response,
    std::function<void(const QString &)> on_error, int timeout_ms = 5000);
  // Runs go once the service is in the graph, or fail after max_wait_ms
  void whenReady(std::function<bool()> ready, std::function<void()> go, std::function<void()> fail,
    int max_wait_ms = 5000);
  void trigger(const std::string & service, ReplyFn done, int timeout_ms = 5000);
  void setBool(const std::string & service, bool value, ReplyFn done, int timeout_ms = 5000);
  void saveFile(const std::string & service, const QString & name, ReplyFn done, int timeout_ms = 10000);
  std::shared_ptr<rclcpp::AsyncParametersClient> parameterClient(const QString & node);
  void onGuiTick();
  void publishEstop(bool engaged);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  rclcpp::CallbackGroup::SharedPtr camera_group_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  std::map<std::string, std::shared_ptr<rclcpp::ClientBase>> clients_;
  std::vector<std::function<void()>> pruners_;
  std::map<std::string, std::shared_ptr<rclcpp::AsyncParametersClient>> parameter_clients_;

  Mailbox<Telemetry> telemetry_box_;
  Mailbox<DriveState> drive_box_;
  Mailbox<LapState> lap_box_;
  Mailbox<LocalizationState> localization_box_;
  Mailbox<SupervisorState> supervisor_box_;
  Mailbox<RecorderState> recorder_box_;
  Mailbox<std::array<double, 3>> imu_box_;
  Mailbox<std::pair<QImage, double>> camera_box_;
  Mailbox<QStringList> nodes_box_;
  Mailbox<bool> estop_box_;

  std::mutex log_mutex_;
  std::vector<Log> rosout_buffer_, console_buffer_;
  std::atomic<bool> rosout_pending_{false}, console_pending_{false};

  std::atomic<bool> camera_wanted_{false};
  std::mutex camera_mutex_;
  QSize camera_max_size_{960, 540};
  std::chrono::steady_clock::time_point camera_last_;
  double camera_fps_ = 0.0;

  // Link watchdog (GUI thread)
  std::chrono::steady_clock::time_point telemetry_time_, supervisor_time_;
  bool car_linked_ = false, supervisor_linked_ = false;
  int telemetry_count_ = 0;
  double telemetry_rate_ = 0.0;
  QStringList nodes_;
  SupervisorState last_supervisor_;

  bool estop_local_ = false;  // E-stop engaged from this UI (re-sent until released)
  int estop_release_repeats_ = 0;
};
}  // namespace f1ui
