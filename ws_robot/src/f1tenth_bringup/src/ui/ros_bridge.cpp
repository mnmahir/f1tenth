#include "ros_bridge.hpp"

#include <unistd.h>

#include <QBuffer>
#include <QImageReader>
#include <QMetaObject>
#include <QTimer>

#include <algorithm>
#include <cctype>
#include <cmath>

#include "f1tenth_bringup/srv/component_command.hpp"
#include "f1tenth_bringup/srv/file_op.hpp"
#include "f1tenth_bringup/srv/get_console.hpp"
#include "f1tenth_bringup/srv/get_map_preview.hpp"
#include "f1tenth_bringup/srv/get_path.hpp"
#include "f1tenth_bringup/srv/save_file.hpp"
#include "f1tenth_bringup/srv/set_mode.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace f1ui
{
namespace
{
constexpr char kSupervisor[] = "/f1tenth/supervisor";

std::string str(const QString & s) {return s.toStdString();}
QString qstr(const std::string & s) {return QString::fromStdString(s);}

// "f1tenth_ui_<host>": one UI per computer, valid as a ROS name
std::string uiNodeName()
{
  char host[256] = {0};
  gethostname(host, sizeof(host) - 1);
  std::string name = "f1tenth_ui_";
  for (const char * c = host; *c; ++c) {
    name += std::isalnum(static_cast<unsigned char>(*c)) ? *c : '_';
  }
  return name;
}
}  // namespace

double PathData::length() const
{
  double total = 0.0;
  for (int i = 1; i < points.size(); ++i) {
    total += std::hypot(points[i].x() - points[i - 1].x(), points[i].y() - points[i - 1].y());
  }
  return total;
}

RosBridge::RosBridge(QObject * parent)
: QObject(parent)
{
  node_ = std::make_shared<rclcpp::Node>(uiNodeName());
  camera_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sensor_qos = rclcpp::SensorDataQoS();
  subscriptions_.push_back(node_->create_subscription<Telemetry>("/f1tenth/telemetry", sensor_qos,
    [this](Telemetry::ConstSharedPtr msg) {
      post(telemetry_box_, *msg, [this](const Telemetry & m) {
        telemetry_time_ = std::chrono::steady_clock::now();
        telemetry_count_++;
        Q_EMIT telemetry(m);
      });
    }));
  subscriptions_.push_back(node_->create_subscription<DriveState>("/f1tenth/drive_state", 10,
    [this](DriveState::ConstSharedPtr msg) {
      post(drive_box_, *msg, [this](const DriveState & m) {Q_EMIT driveState(m);});
    }));
  subscriptions_.push_back(node_->create_subscription<LapState>("/f1tenth/lap", 10,
    [this](LapState::ConstSharedPtr msg) {
      post(lap_box_, *msg, [this](const LapState & m) {Q_EMIT lapState(m);});
    }));
  subscriptions_.push_back(node_->create_subscription<LocalizationState>("/f1tenth/localization", 10,
    [this](LocalizationState::ConstSharedPtr msg) {
      post(localization_box_, *msg, [this](const LocalizationState & m) {Q_EMIT localization(m);});
    }));
  subscriptions_.push_back(node_->create_subscription<SupervisorState>("/f1tenth/supervisor/state", 10,
    [this](SupervisorState::ConstSharedPtr msg) {
      post(supervisor_box_, *msg, [this](const SupervisorState & m) {
        supervisor_time_ = std::chrono::steady_clock::now();
        last_supervisor_ = m;
        Q_EMIT supervisorState(m);
      });
    }));
  subscriptions_.push_back(node_->create_subscription<RecorderState>("/f1tenth/recorder/state", 10,
    [this](RecorderState::ConstSharedPtr msg) {
      post(recorder_box_, *msg, [this](const RecorderState & m) {Q_EMIT recorderState(m);});
    }));
  subscriptions_.push_back(node_->create_subscription<DiagnosticArray>("/diagnostics", 20,
    [this](DiagnosticArray::ConstSharedPtr msg) {
      DiagnosticArray copy = *msg;
      QMetaObject::invokeMethod(this, [this, copy]() {Q_EMIT diagnostics(copy);}, Qt::QueuedConnection);
    }));
  subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::Imu>("/sensors/imu/raw", sensor_qos,
    [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      std::array<double, 3> a{msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
      post(imu_box_, a, [this](const std::array<double, 3> & v) {Q_EMIT imu(v[0], v[1], v[2]);});
    }));
  subscriptions_.push_back(node_->create_subscription<std_msgs::msg::Bool>("/mux_bool/ui_estop", 10,
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      post(estop_box_, static_cast<bool>(msg->data), [this](const bool & engaged) {
        if (!engaged && estop_local_) {
          estop_local_ = false;  // released from another UI
        }
        Q_EMIT estopChanged(engaged, estop_local_);
      });
    }));
  // The car's /rosout as relayed by the supervisor (history comes from ~/get_rosout). Not /rosout itself: a
  // transient-local /rosout subscription makes every node on the car replay its log cache to this computer,
  // which can stall a node that logs meanwhile.
  subscriptions_.push_back(node_->create_subscription<Log>(std::string(kSupervisor) + "/rosout", rclcpp::QoS(500),
    [this](Log::ConstSharedPtr msg) {postLogs(rosout_buffer_, rosout_pending_, *msg, false);}));
  subscriptions_.push_back(node_->create_subscription<Log>("/f1tenth/supervisor/console", rclcpp::QoS(500),
    [this](Log::ConstSharedPtr msg) {postLogs(console_buffer_, console_pending_, *msg, true);}));
  subscriptions_.push_back(node_->create_subscription<ParameterEvent>("/parameter_events",
    rclcpp::ParameterEventsQoS(),
    [this](ParameterEvent::ConstSharedPtr msg) {
      ParameterEvent copy = *msg;
      QMetaObject::invokeMethod(this, [this, copy]() {Q_EMIT parameterEvent(copy);}, Qt::QueuedConnection);
    }));

  rclcpp::SubscriptionOptions camera_options;
  camera_options.callback_group = camera_group_;
  subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::CompressedImage>(
    "/camera/image_raw/compressed", sensor_qos,
    [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
      if (!camera_wanted_ || msg->data.empty()) {
        return;
      }
      QSize max_size;
      {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        max_size = camera_max_size_;
      }
      QByteArray bytes = QByteArray::fromRawData(reinterpret_cast<const char *>(msg->data.data()),
        static_cast<int>(msg->data.size()));
      QBuffer buffer(&bytes);
      buffer.open(QIODevice::ReadOnly);
      QImageReader reader(&buffer);
      QSize size = reader.size();
      if (size.isValid() && max_size.isValid() &&
        (size.width() > max_size.width() || size.height() > max_size.height()))
      {
        reader.setScaledSize(size.scaled(max_size, Qt::KeepAspectRatio));  // libjpeg decodes at the smaller size
      }
      QImage image = reader.read();
      if (image.isNull()) {
        return;
      }
      auto now = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(now - camera_last_).count();
      camera_last_ = now;
      if (dt > 0.0 && dt < 2.0) {
        camera_fps_ = camera_fps_ > 0.0 ? 0.9 * camera_fps_ + 0.1 / dt : 1.0 / dt;
      }
      post(camera_box_, std::make_pair(image, camera_fps_),
        [this](const std::pair<QImage, double> & frame) {Q_EMIT cameraFrame(frame.first, frame.second);});
    }, camera_options));

  estop_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/mux_bool/ui_estop", 10);

  graph_timer_ = node_->create_wall_timer(std::chrono::seconds(2), [this]() {
    QStringList names;
    for (const auto & name : node_->get_node_names()) {
      names << qstr(name);
    }
    names.removeDuplicates();
    names.sort();
    post(nodes_box_, names, [this](const QStringList & list) {
      if (list != nodes_) {
        nodes_ = list;
        Q_EMIT nodesChanged(nodes_);
      }
    });
  });

  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 3);
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {executor_->spin();});

  auto * tick = new QTimer(this);
  connect(tick, &QTimer::timeout, this, &RosBridge::onGuiTick);
  tick->start(500);
}

RosBridge::~RosBridge()
{
  if (estop_local_) {
    publishEstop(true);  // leave it engaged on the car: closing the UI must not release an E-stop
  }
  executor_->cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  executor_->remove_node(node_);
  parameter_clients_.clear();
  clients_.clear();
  subscriptions_.clear();
}

template<typename T, typename Emit>
void RosBridge::post(Mailbox<T> & box, T value, Emit emit_fn)
{
  {
    std::lock_guard<std::mutex> lock(box.mutex);
    box.value = std::move(value);
  }
  if (!box.pending.exchange(true)) {
    QMetaObject::invokeMethod(this, [&box, emit_fn]() {
        T latest;
        {
          std::lock_guard<std::mutex> lock(box.mutex);
          latest = box.value;
          box.pending = false;
        }
        emit_fn(latest);
      }, Qt::QueuedConnection);
  }
}

void RosBridge::postLogs(std::vector<Log> & buffer, std::atomic<bool> & pending, const Log & msg, bool is_console)
{
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    buffer.push_back(msg);
    if (buffer.size() > 5000) {
      buffer.erase(buffer.begin(), buffer.begin() + 1000);
    }
  }
  if (!pending.exchange(true)) {
    QMetaObject::invokeMethod(this, [this, &buffer, &pending, is_console]() {
        std::vector<Log> batch;
        {
          std::lock_guard<std::mutex> lock(log_mutex_);
          batch.swap(buffer);
          pending = false;
        }
        if (is_console) {
          Q_EMIT console(batch);
        } else {
          Q_EMIT rosout(batch);
        }
      }, Qt::QueuedConnection);
  }
}

void RosBridge::onGuiTick()
{
  auto now = std::chrono::steady_clock::now();
  bool car = std::chrono::duration<double>(now - telemetry_time_).count() < 1.5;
  bool supervisor = std::chrono::duration<double>(now - supervisor_time_).count() < 3.0;
  telemetry_rate_ = 0.5 * telemetry_rate_ + 0.5 * telemetry_count_ / 0.5;
  telemetry_count_ = 0;
  if (car != car_linked_ || supervisor != supervisor_linked_) {
    car_linked_ = car;
    supervisor_linked_ = supervisor;
    Q_EMIT linkChanged(car, supervisor);
  }
  // E-stop: re-sent while held here (covers a restarted mux), release sent a few times
  if (estop_local_) {
    publishEstop(true);
  } else if (estop_release_repeats_ > 0) {
    estop_release_repeats_--;
    publishEstop(false);
  }
  for (const auto & prune : pruners_) {
    prune();
  }
}

void RosBridge::publishEstop(bool engaged)
{
  std_msgs::msg::Bool msg;
  msg.data = engaged;
  estop_pub_->publish(msg);
}

void RosBridge::setEstop(bool engaged)
{
  estop_local_ = engaged;
  estop_release_repeats_ = engaged ? 0 : 3;
  publishEstop(engaged);
  if (engaged) {
    setAutonomous(false, [](bool, const QString &) {});
  }
  Q_EMIT estopChanged(engaged, engaged);
}

void RosBridge::setCameraWanted(bool wanted, QSize max_size)
{
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    camera_max_size_ = max_size;
  }
  camera_wanted_ = wanted;
}

// ---- service plumbing ----

template<typename ServiceT>
typename rclcpp::Client<ServiceT>::SharedPtr RosBridge::client(const std::string & name)
{
  auto it = clients_.find(name);
  if (it != clients_.end()) {
    return std::static_pointer_cast<rclcpp::Client<ServiceT>>(it->second);
  }
  auto c = node_->create_client<ServiceT>(name);
  clients_[name] = c;
  std::weak_ptr<rclcpp::Client<ServiceT>> weak = c;
  pruners_.push_back([weak]() {
    if (auto locked = weak.lock()) {
      locked->prune_requests_older_than(std::chrono::system_clock::now() - std::chrono::minutes(2));
    }
  });
  return c;
}

template<typename ServiceT>
void RosBridge::call(const std::string & service, std::shared_ptr<typename ServiceT::Request> request,
  std::function<void(std::shared_ptr<typename ServiceT::Response>)> on_response,
  std::function<void(const QString &)> on_error, int timeout_ms)
{
  auto c = client<ServiceT>(service);
  QString what = qstr(service);
  auto finished = std::make_shared<std::atomic<bool>>(false);
  whenReady([c]() {return c->service_is_ready();},
    [this, c, request, finished, on_response]() {
      c->async_send_request(request,
        [this, finished, on_response](typename rclcpp::Client<ServiceT>::SharedFuture future) {
          auto response = future.get();
          QMetaObject::invokeMethod(this, [finished, on_response, response]() {
              if (!finished->exchange(true)) {
                on_response(response);
              }
            }, Qt::QueuedConnection);
        });
    },
    [finished, on_error, what]() {
      if (!finished->exchange(true)) {
        on_error(what + " is not available (is the car running?)");
      }
    });
  QTimer::singleShot(timeout_ms, this, [finished, on_error, what]() {
      if (!finished->exchange(true)) {
        on_error(what + " did not answer");
      }
    });
}

void RosBridge::whenReady(std::function<bool()> ready, std::function<void()> go, std::function<void()> fail)
{
  if (ready()) {
    go();
    return;
  }
  // Discovery can lag right after start-up: give the service up to 2 s to appear
  auto * timer = new QTimer(this);
  auto waited = std::make_shared<int>(0);
  connect(timer, &QTimer::timeout, this, [timer, waited, ready, go, fail]() {
      if (ready()) {
        timer->deleteLater();
        timer->stop();
        go();
      } else if ((*waited += 100) >= 2000) {
        timer->deleteLater();
        timer->stop();
        fail();
      }
    });
  timer->start(100);
}

void RosBridge::trigger(const std::string & service, ReplyFn done, int timeout_ms)
{
  using Srv = std_srvs::srv::Trigger;
  call<Srv>(service, std::make_shared<Srv::Request>(),
    [done](Srv::Response::SharedPtr res) {done(res->success, qstr(res->message));},
    [done](const QString & error) {done(false, error);}, timeout_ms);
}

void RosBridge::setBool(const std::string & service, bool value, ReplyFn done, int timeout_ms)
{
  using Srv = std_srvs::srv::SetBool;
  auto req = std::make_shared<Srv::Request>();
  req->data = value;
  call<Srv>(service, req,
    [done](Srv::Response::SharedPtr res) {done(res->success, qstr(res->message));},
    [done](const QString & error) {done(false, error);}, timeout_ms);
}

void RosBridge::saveFile(const std::string & service, const QString & name, ReplyFn done, int timeout_ms)
{
  using Srv = f1tenth_bringup::srv::SaveFile;
  auto req = std::make_shared<Srv::Request>();
  req->name = str(name);
  call<Srv>(service, req,
    [done](Srv::Response::SharedPtr res) {done(res->success, qstr(res->message));},
    [done](const QString & error) {done(false, error);}, timeout_ms);
}

// ---- supervisor ----

void RosBridge::setMode(const QString & mode, const QString & map, const QString & path, ReplyFn done)
{
  using Srv = f1tenth_bringup::srv::SetMode;
  auto req = std::make_shared<Srv::Request>();
  req->mode = str(mode);
  req->map = str(map);
  req->path = str(path);
  call<Srv>(std::string(kSupervisor) + "/set_mode", req,
    [done](Srv::Response::SharedPtr res) {done(res->success, qstr(res->message));},
    [done](const QString & error) {done(false, error);});
}

void RosBridge::componentCommand(const QString & name, const QString & action, ReplyFn done)
{
  using Srv = f1tenth_bringup::srv::ComponentCommand;
  auto req = std::make_shared<Srv::Request>();
  req->name = str(name);
  req->action = str(action);
  call<Srv>(std::string(kSupervisor) + "/component", req,
    [done](Srv::Response::SharedPtr res) {done(res->success, qstr(res->message));},
    [done](const QString & error) {done(false, error);});
}

void RosBridge::fileOp(const QString & kind, const QString & action, const QString & name,
  const QString & new_name, ReplyFn done)
{
  using Srv = f1tenth_bringup::srv::FileOp;
  auto req = std::make_shared<Srv::Request>();
  req->kind = str(kind);
  req->action = str(action);
  req->name = str(name);
  req->new_name = str(new_name);
  call<Srv>(std::string(kSupervisor) + "/file_op", req,
    [done](Srv::Response::SharedPtr res) {done(res->success, qstr(res->message));},
    [done](const QString & error) {done(false, error);});
}

void RosBridge::setAutoRestart(bool on, ReplyFn done)
{
  setBool(std::string(kSupervisor) + "/set_auto_restart", on, done);
}

void RosBridge::setBagRecording(bool on, ReplyFn done)
{
  setBool(std::string(kSupervisor) + "/set_bag_recording", on, done);
}

void RosBridge::saveMap(const QString & name, ReplyFn done)
{
  saveFile(std::string(kSupervisor) + "/save_map", name, done, 30000);
}

void RosBridge::saveParams(const QString & node, ReplyFn done)
{
  saveFile(std::string(kSupervisor) + "/save_params", node, done, 15000);
}

void RosBridge::mapPreview(const QString & name, int max_size, std::function<void(const MapPreview &)> done)
{
  using Srv = f1tenth_bringup::srv::GetMapPreview;
  auto req = std::make_shared<Srv::Request>();
  req->name = str(name);
  req->max_size = static_cast<uint32_t>(max_size);
  call<Srv>(std::string(kSupervisor) + "/get_map_preview", req,
    [done](Srv::Response::SharedPtr res) {
      MapPreview preview;
      preview.ok = res->success;
      preview.message = qstr(res->message);
      const auto & img = res->image;
      if (res->success && img.width > 0 && img.height > 0 && img.data.size() >= img.step * img.height) {
        QImage image(static_cast<int>(img.width), static_cast<int>(img.height), QImage::Format_Grayscale8);
        for (uint32_t row = 0; row < img.height; ++row) {
          std::copy_n(img.data.data() + row * img.step, img.width, image.scanLine(static_cast<int>(row)));
        }
        preview.image = image;
        preview.resolution = res->resolution;
        preview.origin_x = res->origin.position.x;
        preview.origin_y = res->origin.position.y;
      } else if (res->success) {
        preview.ok = false;
        preview.message = "Empty map preview";
      }
      done(preview);
    },
    [done](const QString & error) {
      MapPreview preview;
      preview.message = error;
      done(preview);
    }, 10000);
}

void RosBridge::path(const QString & name, std::function<void(const PathData &)> done)
{
  using Srv = f1tenth_bringup::srv::GetPath;
  auto req = std::make_shared<Srv::Request>();
  req->name = str(name);
  call<Srv>(std::string(kSupervisor) + "/get_path", req,
    [done](Srv::Response::SharedPtr res) {
      PathData data;
      data.ok = res->success;
      data.message = qstr(res->message);
      for (const auto & pose : res->path.poses) {
        data.points.push_back(QPointF(pose.pose.position.x, pose.pose.position.y));
      }
      for (float v : res->speeds) {
        data.speeds.push_back(v);
      }
      done(data);
    },
    [done](const QString & error) {
      PathData data;
      data.message = error;
      done(data);
    });
}

void RosBridge::consoleHistory(int max_lines, std::function<void(const std::vector<Log> &)> done)
{
  using Srv = f1tenth_bringup::srv::GetConsole;
  auto req = std::make_shared<Srv::Request>();
  req->max_lines = static_cast<uint32_t>(max_lines);
  call<Srv>(std::string(kSupervisor) + "/get_console", req,
    [done](Srv::Response::SharedPtr res) {done(res->lines);},
    [done](const QString &) {done({});});
}

void RosBridge::rosoutHistory(int max_lines, std::function<void(const std::vector<Log> &)> done)
{
  using Srv = f1tenth_bringup::srv::GetConsole;
  auto req = std::make_shared<Srv::Request>();
  req->max_lines = static_cast<uint32_t>(max_lines);
  call<Srv>(std::string(kSupervisor) + "/get_rosout", req,
    [done](Srv::Response::SharedPtr res) {done(res->lines);},
    [done](const QString &) {done({});});
}

// ---- driving ----

void RosBridge::setAutonomous(bool on, ReplyFn done)
{
  setBool("/drive_coordinator/set_autonomous", on, done);
}

void RosBridge::setAssist(bool on, ReplyFn done)
{
  setBool("/drive_coordinator/set_assist", on, done);
}

void RosBridge::recorder(const QString & action, ReplyFn done)
{
  trigger("/path_recorder/" + str(action), done);
}

void RosBridge::saveRecording(const QString & name, ReplyFn done)
{
  saveFile("/path_recorder/save", name, done);
}

void RosBridge::optimizeRaceline(const f1tenth_bringup::srv::OptimizeRaceline::Request & request,
  std::function<void(const RacelineResult &)> done)
{
  using Srv = f1tenth_bringup::srv::OptimizeRaceline;
  call<Srv>("/raceline_optimizer/optimize", std::make_shared<Srv::Request>(request),
    [done](Srv::Response::SharedPtr res) {
      RacelineResult r;
      r.ok = res->success;
      r.message = qstr(res->message);
      r.length = res->length;
      r.lap_time = res->lap_time;
      r.min_width = res->min_width;
      done(r);
    },
    [done](const QString & error) {
      RacelineResult r;
      r.message = error;
      done(r);
    }, 60000);
}

void RosBridge::saveRaceline(const QString & name, ReplyFn done)
{
  saveFile("/raceline_optimizer/save", name, done);
}

void RosBridge::toggleMappingPause(ReplyFn done)
{
  // The supervisor tracks the pause state (slam_toolbox only has a toggle)
  setBool(std::string(kSupervisor) + "/set_mapping_paused", !last_supervisor_.mapping_paused, done);
}

void RosBridge::placeAtStartLine(ReplyFn done)
{
  trigger("/localization_monitor/place_at_start", done);
}

void RosBridge::locateCar(ReplyFn done)
{
  trigger("/localization_monitor/locate", done, 20000);
}

void RosBridge::resetLap(ReplyFn done)
{
  trigger("/lap_timer/reset", done);
}

// ---- parameters ----

std::shared_ptr<rclcpp::AsyncParametersClient> RosBridge::parameterClient(const QString & node)
{
  std::string name = str(node);
  auto it = parameter_clients_.find(name);
  if (it != parameter_clients_.end()) {
    return it->second;
  }
  auto c = std::make_shared<rclcpp::AsyncParametersClient>(node_, name);
  parameter_clients_[name] = c;
  return c;
}

void RosBridge::loadParameters(const QString & node,
  std::function<void(bool ok, const QString & error, const std::vector<ParamInfo> & params)> done)
{
  auto c = parameterClient(node);
  auto finished = std::make_shared<std::atomic<bool>>(false);
  auto fail = [this, finished, done](const QString & error) {
      QMetaObject::invokeMethod(this, [finished, done, error]() {
          if (!finished->exchange(true)) {
            done(false, error, {});
          }
        }, Qt::QueuedConnection);
    };
  // list -> get + describe, then hand everything to the GUI thread
  auto describe = [this, finished, done](std::shared_ptr<rclcpp::AsyncParametersClient> client,
      std::vector<std::string> names, std::vector<rclcpp::Parameter> values) {
      client->describe_parameters(names,
        [this, values, finished, done](
          std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>> described) {
          std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors;
          try {
            descriptors = described.get();
          } catch (const std::exception &) {
            // descriptions are optional
          }
          std::vector<ParamInfo> params;
          for (size_t i = 0; i < values.size(); ++i) {
            ParamInfo info;
            info.value = values[i];
            if (i < descriptors.size()) {
              info.descriptor = descriptors[i];
            }
            params.push_back(info);
          }
          QMetaObject::invokeMethod(this, [finished, done, params]() {
              if (!finished->exchange(true)) {
                done(true, QString(), params);
              }
            }, Qt::QueuedConnection);
        });
    };
  auto list = [c, fail, describe]() {
      std::weak_ptr<rclcpp::AsyncParametersClient> weak = c;
      c->list_parameters({}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE,
        [weak, fail, describe](std::shared_future<rcl_interfaces::msg::ListParametersResult> listed) {
          std::vector<std::string> names;
          try {
            names = listed.get().names;
          } catch (const std::exception & e) {
            fail(QString("list_parameters failed: %1").arg(e.what()));
            return;
          }
          auto client = weak.lock();
          if (!client) {
            return;
          }
          std::sort(names.begin(), names.end());
          client->get_parameters(names,
            [weak, names, fail, describe](std::shared_future<std::vector<rclcpp::Parameter>> got) {
              std::vector<rclcpp::Parameter> values;
              try {
                values = got.get();
              } catch (const std::exception & e) {
                fail(QString("get_parameters failed: %1").arg(e.what()));
                return;
              }
              if (auto client = weak.lock()) {
                describe(client, names, values);
              }
            });
        });
    };
  whenReady([c]() {return c->service_is_ready();}, list,
    [finished, done, node]() {
      if (!finished->exchange(true)) {
        done(false, node + " has no parameter services", {});
      }
    });
  QTimer::singleShot(8000, this, [finished, done, node]() {
      if (!finished->exchange(true)) {
        done(false, node + " did not answer", {});
      }
    });
}

void RosBridge::getParameters(const QString & node, const QStringList & names,
  std::function<void(bool ok, const std::vector<rclcpp::Parameter> & values)> done)
{
  auto c = parameterClient(node);
  std::vector<std::string> list;
  for (const auto & name : names) {
    list.push_back(str(name));
  }
  auto finished = std::make_shared<std::atomic<bool>>(false);
  auto get = [this, c, list, finished, done]() {
      c->get_parameters(list, [this, finished, done](std::shared_future<std::vector<rclcpp::Parameter>> got) {
          std::vector<rclcpp::Parameter> values;
          bool ok = true;
          try {
            values = got.get();
          } catch (const std::exception &) {
            ok = false;
          }
          QMetaObject::invokeMethod(this, [finished, done, ok, values]() {
              if (!finished->exchange(true)) {
                done(ok, values);
              }
            }, Qt::QueuedConnection);
        });
    };
  whenReady([c]() {return c->service_is_ready();}, get,
    [finished, done]() {
      if (!finished->exchange(true)) {
        done(false, {});
      }
    });
  QTimer::singleShot(5000, this, [finished, done]() {
      if (!finished->exchange(true)) {
        done(false, {});
      }
    });
}

void RosBridge::setParameter(const QString & node, const rclcpp::Parameter & param, ReplyFn done)
{
  auto c = parameterClient(node);
  auto finished = std::make_shared<std::atomic<bool>>(false);
  QString what = qstr(param.get_name());
  auto set = [this, c, param, finished, done, what]() {
      c->set_parameters({param},
        [this, finished, done, what](
          std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> result) {
          bool ok = false;
          QString message;
          try {
            auto results = result.get();
            ok = !results.empty() && results.front().successful;
            message = ok ? what + " updated" :
            what + ": " + (results.empty() ? QString("rejected") : qstr(results.front().reason));
          } catch (const std::exception & e) {
            message = what + ": " + e.what();
          }
          QMetaObject::invokeMethod(this, [finished, done, ok, message]() {
              if (!finished->exchange(true)) {
                done(ok, message);
              }
            }, Qt::QueuedConnection);
        });
    };
  whenReady([c]() {return c->service_is_ready();}, set,
    [finished, done, node]() {
      if (!finished->exchange(true)) {
        done(false, node + " is not running");
      }
    });
  QTimer::singleShot(5000, this, [finished, done, node]() {
      if (!finished->exchange(true)) {
        done(false, node + " did not answer");
      }
    });
}
}  // namespace f1ui
