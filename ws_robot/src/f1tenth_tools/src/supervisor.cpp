// Session supervisor: runs on the car and turns UI requests into running software, so the UI can be remote.
//  - Modes (idle, manual, mapping, path) map to launch components that it starts and stops as process groups
//  - Restarts components that crash or whose expected nodes vanish (auto_restart), with growing back-off
//  - Saves maps (slam_toolbox), lists/previews/renames/deletes maps and paths, persists parameter overrides,
//    records bags, and publishes the selected path
//  - Publishes what the components print (~/console) and keeps the latest lines (~/get_console) for the UI
//  - Announces the car (name, ROS domain, mode, battery) on the network so UIs can list cars and connect
#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <functional>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "f1tenth_bringup/msg/supervisor_state.hpp"
#include "f1tenth_bringup/srv/component_command.hpp"
#include "f1tenth_bringup/srv/file_op.hpp"
#include "f1tenth_bringup/srv/get_console.hpp"
#include "f1tenth_bringup/srv/get_map_preview.hpp"
#include "f1tenth_bringup/srv/get_path.hpp"
#include "f1tenth_bringup/srv/save_file.hpp"
#include "f1tenth_bringup/srv/set_mode.hpp"
#include "f1tenth_bringup/msg/telemetry.hpp"
#include "f1tenth_tools/data_io.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "slam_toolbox/srv/pause.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace ft = f1tenth_tools;
using namespace f1tenth_bringup;
using Clock = std::chrono::steady_clock;

namespace
{
double seconds_since(Clock::time_point t) {return std::chrono::duration<double>(Clock::now() - t).count();}

std::string timestamp()
{
  std::time_t t = std::time(nullptr);
  std::ostringstream ss;
  ss << std::put_time(std::localtime(&t), "%Y%m%d-%H%M%S");
  return ss.str();
}

// Absolute path of an executable on PATH (resolved before fork: exec in the child must not allocate)
std::string which(const std::string & exe)
{
  const char * path = std::getenv("PATH");
  std::stringstream ss(path ? path : "");
  std::string dir;
  while (std::getline(ss, dir, ':')) {
    std::string candidate = dir + "/" + exe;
    if (access(candidate.c_str(), X_OK) == 0) {
      return candidate;
    }
  }
  return exe;
}
}  // namespace

// Lets UIs find cars on the network: answers "F1TENTH?" on UDP 47820 (works across routed networks such as a
// VPN) and broadcasts the same JSON to UDP 47821 on every interface each second (local network).
class Beacon
{
public:
  static constexpr uint16_t kQueryPort = 47820;
  static constexpr uint16_t kAnnouncePort = 47821;

  explicit Beacon(rclcpp::Logger logger)
  : logger_(logger)
  {
    fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    int on = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(kQueryPort);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (fd_ < 0 || bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      RCLCPP_WARN(logger_, "Car beacon: UDP port %d unavailable (%s); UIs won't list this car",
        kQueryPort, std::strerror(errno));
    }
    thread_ = std::thread([this]() {serve();});
  }

  ~Beacon()
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  void set_payload(const std::string & json)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    payload_ = json;
  }

  void announce()
  {
    std::string payload;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      payload = payload_;
    }
    ifaddrs * list = nullptr;
    if (fd_ < 0 || payload.empty() || getifaddrs(&list) != 0) {
      return;
    }
    for (ifaddrs * ifa = list; ifa; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET && ifa->ifa_broadaddr &&
        (ifa->ifa_flags & IFF_UP) && (ifa->ifa_flags & IFF_BROADCAST) && !(ifa->ifa_flags & IFF_LOOPBACK))
      {
        sockaddr_in to = *reinterpret_cast<sockaddr_in *>(ifa->ifa_broadaddr);
        to.sin_port = htons(kAnnouncePort);
        sendto(fd_, payload.data(), payload.size(), 0, reinterpret_cast<sockaddr *>(&to), sizeof(to));
      }
    }
    freeifaddrs(list);
  }

private:
  void serve()
  {
    char buf[64];
    while (running_ && fd_ >= 0) {
      pollfd p{fd_, POLLIN, 0};
      if (poll(&p, 1, 300) <= 0) {
        continue;
      }
      sockaddr_in from{};
      socklen_t len = sizeof(from);
      ssize_t n = recvfrom(fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&from), &len);
      if (n >= 8 && std::string(buf, 8) == "F1TENTH?") {
        std::lock_guard<std::mutex> lock(mutex_);
        sendto(fd_, payload_.data(), payload_.size(), 0, reinterpret_cast<sockaddr *>(&from), len);
      }
    }
  }

  rclcpp::Logger logger_;
  int fd_ = -1;
  std::atomic<bool> running_{true};
  std::thread thread_;
  std::mutex mutex_;
  std::string payload_;
};

std::string json_string(const std::string & s)
{
  std::string out = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    if (static_cast<unsigned char>(c) >= 0x20) {
      out += c;
    }
  }
  return out + "\"";
}

// Component output for the UI's terminal view. Shared with the reader threads, which can outlive the node.
class Console
{
public:
  using Log = rcl_interfaces::msg::Log;

  void set_publisher(rclcpp::Publisher<Log>::SharedPtr pub)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pub_ = std::move(pub);
  }

  void add(const std::string & name, const std::string & line)
  {
    Log msg;
    msg.stamp = clock_.now();
    msg.level = level_of(line);
    msg.name = name;
    msg.msg = line;
    std::lock_guard<std::mutex> lock(mutex_);
    history_.push_back(msg);
    if (history_.size() > 2000) {
      history_.pop_front();
    }
    if (pub_) {
      try {
        pub_->publish(msg);
      } catch (const std::exception &) {
        // shutting down
      }
    }
  }

  std::vector<Log> latest(size_t max_lines)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t n = max_lines == 0 ? history_.size() : std::min<size_t>(max_lines, history_.size());
    return std::vector<Log>(history_.end() - static_cast<std::ptrdiff_t>(n), history_.end());
  }

private:
  static uint8_t level_of(const std::string & line)
  {
    auto has = [&line](const char * s) {return line.find(s) != std::string::npos;};
    if (has("[FATAL]")) {return Log::FATAL;}
    if (has("[ERROR]") || has("Traceback") || has("terminate called") || has("what():")) {return Log::ERROR;}
    if (has("[WARN]") || has("[WARNING]")) {return Log::WARN;}
    if (has("[DEBUG]")) {return Log::DEBUG;}
    return Log::INFO;
  }

  std::mutex mutex_;
  rclcpp::Publisher<Log>::SharedPtr pub_;
  rclcpp::Clock clock_{RCL_SYSTEM_TIME};
  std::deque<Log> history_;
};

struct Component
{
  std::string name;
  std::vector<std::string> expected_nodes;
  std::string log_file;

  // What the session wants
  bool desired = false;
  std::vector<std::string> command;

  // What is running
  std::vector<std::string> running_command;
  pid_t pid = -1;
  std::string status = "stopped";
  int restarts = 0;
  double backoff = 2.0;
  bool stopping = false;
  bool restart_pending = false;
  bool user_stopped = false;  // stopped from the UI: stays off (and listed) until the session changes
  Clock::time_point started, stop_requested, next_start = Clock::now();
  std::vector<std::string> missing;
  std::vector<std::string> unresponsive;
  Clock::time_point missing_since;
  std::thread reader;
  // Set while stopping: shutdown noise (e.g. tracebacks from nodes on SIGINT) is not reported as errors
  std::shared_ptr<std::atomic<bool>> quiet = std::make_shared<std::atomic<bool>>(false);
};

class Supervisor : public rclcpp::Node
{
public:
  Supervisor() : Node("supervisor")
  {
    data_dir_ = ft::expand_user(declare_parameter<std::string>("data_dir", "~/f1tenth/data"));
    maps_dir_ = data_dir_ + "/maps";
    paths_dir_ = data_dir_ + "/paths";
    params_dir_ = data_dir_ + "/params";
    log_dir_ = ft::expand_user(declare_parameter<std::string>("log_dir", "~/.ros/f1tenth"));
    auto_restart_ = declare_parameter<bool>("auto_restart", true);
    start_core_ = declare_parameter<bool>("start_core", false);  // run the core in standby too
    car_name_ = declare_parameter<std::string>("car_name", "");
    router_port_ = static_cast<int>(declare_parameter<int>("router_port", 7447));  // 0: no Zenoh router
    if (car_name_.empty()) {
      char host[256] = {0};
      gethostname(host, sizeof(host) - 1);
      car_name_ = host;
    }
    startup_grace_ = declare_parameter<double>("startup_grace", 25.0);
    missing_timeout_ = declare_parameter<double>("missing_node_timeout", 15.0);
    // Hung nodes: each expected node's parameter service is pinged every probe_period; one that misses
    // probe_failures answers in a row is restarted with its component. Nodes that may legitimately block for
    // long (map serialization, raceline optimization) are not probed.
    probe_period_ = declare_parameter<double>("probe_period", 3.0);
    probe_timeout_ = declare_parameter<double>("probe_timeout", 2.5);
    probe_failures_ = static_cast<int>(declare_parameter<int>("probe_failures", 5));
    for (const auto & n : declare_parameter<std::vector<std::string>>("probe_exclude",
      std::vector<std::string>{"/slam_toolbox", "/raceline_optimizer"}))
    {
      probe_exclude_.insert(n);
    }
    bag_topics_ = declare_parameter<std::vector<std::string>>("bag_topics", std::vector<std::string>{
      "/scan", "/tf", "/tf_static", "/odom/wheel", "/odom/filtered", "/sensors/core", "/sensors/imu/raw",
      "/ackermann_cmd", "/ackermann_cmd_filtered", "/joy", "/f1tenth/telemetry", "/f1tenth/drive_state",
      "/f1tenth/lap", "/map", "/amcl_pose", "/f1tenth/selected_path"});
    for (const auto & dir : {maps_dir_, paths_dir_, params_dir_, log_dir_, data_dir_ + "/bags"}) {
      std::error_code ec;
      ft::fs::create_directories(dir, ec);
    }
    ros2_ = which("ros2");
    // Orphaned node processes (after their launch exits) become our children, so we can reap them
    prctl(PR_SET_CHILD_SUBREAPER, 1);

    add_component("core", {"/joy", "/joy_teleop", "/teleop_speed_multiplier", "/drive_coordinator",
      "/vesc_driver_node", "/ackermann_to_vesc_node", "/vesc_to_odom_node", "/ackermann_mux",
      "/robot_state_publisher", "/vehicle_joint_state_publisher", "/urg_node2", "/camera",
      "/cmd_cutoff_and_brake_control", "/autonomous_safety_brake", "/utility_visualizer", "/ekf_filter_node",
      "/avoidance_controller_node", "/telemetry_aggregator", "/imu_drift_corrector"});
    add_component("localization", {"/map_server", "/amcl", "/lifecycle_manager_localization",
      "/base_footprint_to_map_odom", "/lap_timer", "/localization_monitor"});
    add_component("slam", {"/slam_toolbox"});
    add_component("follower", {"/pure_pursuit", "/waypoint_visualizer_node"});
    add_component("path_tools", {"/path_recorder", "/raceline_optimizer"});
    add_component("bag", {});

    rclcpp::QoS latched(1);
    latched.transient_local();
    state_pub_ = create_publisher<msg::SupervisorState>("~/state", 10);
    console_->set_publisher(create_publisher<rcl_interfaces::msg::Log>("~/console", rclcpp::QoS(200)));
    console_srv_ = create_service<srv::GetConsole>("~/get_console",
      [this](srv::GetConsole::Request::SharedPtr req, srv::GetConsole::Response::SharedPtr res) {
        res->lines = console_->latest(req->max_lines);
      });
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/f1tenth/selected_path", latched);
    // /rosout for the UIs: relayed (and kept since boot) here, so a UI never subscribes to /rosout itself. A
    // transient-local /rosout subscription from a laptop makes every node on the car replay its log cache over
    // the network, and a node that logs while that is in flight can block in its log call (seen with
    // pure_pursuit, which then stopped). This subscription is volatile: nothing is replayed.
    rosout_pub_ = create_publisher<rcl_interfaces::msg::Log>("~/rosout", rclcpp::QoS(500));
    rosout_sub_ = create_subscription<rcl_interfaces::msg::Log>("/rosout", rclcpp::QoS(1000).reliable(),
      [this](rcl_interfaces::msg::Log::ConstSharedPtr msg) {
        if (msg->name.rfind("f1tenth_ui", 0) == 0) {
          return;  // a UI's own messages stay on its computer
        }
        rosout_history_.push_back(*msg);
        if (rosout_history_.size() > 5000) {
          rosout_history_.pop_front();
        }
        rosout_pub_->publish(*msg);
      });
    rosout_srv_ = create_service<srv::GetConsole>("~/get_rosout",
      [this](srv::GetConsole::Request::SharedPtr req, srv::GetConsole::Response::SharedPtr res) {
        size_t n = req->max_lines == 0 ? rosout_history_.size() :
        std::min<size_t>(req->max_lines, rosout_history_.size());
        res->lines.assign(rosout_history_.end() - static_cast<std::ptrdiff_t>(n), rosout_history_.end());
      });

    set_mode_srv_ = create_service<srv::SetMode>("~/set_mode",
      [this](srv::SetMode::Request::SharedPtr req, srv::SetMode::Response::SharedPtr res) {
        res->success = set_mode(req->mode, req->map, req->path, res->message);
      });
    component_srv_ = create_service<srv::ComponentCommand>("~/component",
      [this](srv::ComponentCommand::Request::SharedPtr req, srv::ComponentCommand::Response::SharedPtr res) {
        res->success = component_command(req->name, req->action, res->message);
      });
    file_srv_ = create_service<srv::FileOp>("~/file_op",
      [this](srv::FileOp::Request::SharedPtr req, srv::FileOp::Response::SharedPtr res) {
        res->success = file_op(*req, res->message);
      });
    auto_restart_srv_ = create_service<std_srvs::srv::SetBool>("~/set_auto_restart",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
        auto_restart_ = req->data;
        set_parameter(rclcpp::Parameter("auto_restart", auto_restart_));
        update_desired();
        res->success = true;
        res->message = auto_restart_ ? "Auto restart on" : "Auto restart off";
        event(res->message);
      });
    bag_srv_ = create_service<std_srvs::srv::SetBool>("~/set_bag_recording",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
        bag_ = req->data;
        bag_name_ = bag_ ? data_dir_ + "/bags/" + timestamp() : "";
        update_desired();
        res->success = true;
        res->message = bag_ ? "Recording bag " + bag_name_ : "Bag recording stopped";
        event(res->message);
      });
    preview_srv_ = create_service<srv::GetMapPreview>("~/get_map_preview",
      [this](srv::GetMapPreview::Request::SharedPtr req, srv::GetMapPreview::Response::SharedPtr res) {
        map_preview(*req, *res);
      });
    get_path_srv_ = create_service<srv::GetPath>("~/get_path",
      [this](srv::GetPath::Request::SharedPtr req, srv::GetPath::Response::SharedPtr res) {
        std::vector<ft::PathPoint> points;
        res->success = load_path(req->name, points, res->message);
        if (res->success) {
          res->path = to_path(points);
          for (const auto & p : points) {
            res->speeds.push_back(static_cast<float>(p.v));
          }
        }
      });
    save_map_srv_ = create_service<srv::SaveFile>("~/save_map",
      [this](std::shared_ptr<rclcpp::Service<srv::SaveFile>> service, std::shared_ptr<rmw_request_id_t> header,
      srv::SaveFile::Request::SharedPtr req) {save_map(service, header, req->name);});
    save_params_srv_ = create_service<srv::SaveFile>("~/save_params",
      [this](std::shared_ptr<rclcpp::Service<srv::SaveFile>> service, std::shared_ptr<rmw_request_id_t> header,
      srv::SaveFile::Request::SharedPtr req) {save_params(service, header, req->name);});
    serialize_client_ = create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");
    pause_client_ = create_client<slam_toolbox::srv::Pause>("/slam_toolbox/pause_new_measurements");
    pause_srv_ = create_service<std_srvs::srv::SetBool>("~/set_mapping_paused",
      [this](std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> service, std::shared_ptr<rmw_request_id_t> header,
      std_srvs::srv::SetBool::Request::SharedPtr req) {set_mapping_paused(service, header, req->data);});

    std::string message;
    set_mode("idle", "", "", message);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {tick();});
    telemetry_sub_ = create_subscription<msg::Telemetry>("/f1tenth/telemetry", rclcpp::SensorDataQoS(),
      [this](msg::Telemetry::ConstSharedPtr t) {
        battery_ = t->vesc_connected ? t->battery_percent : -1.0;
        telemetry_time_ = Clock::now();
      });
    beacon_ = std::make_unique<Beacon>(get_logger());
    beacon_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        bool fresh = seconds_since(telemetry_time_) < 3.0;
        char battery[16];
        std::snprintf(battery, sizeof(battery), "%.0f", fresh ? battery_ : -1.0);
        char host[256] = {0};
        gethostname(host, sizeof(host) - 1);
        beacon_->set_payload("{\"f1tenth\": 1, \"name\": " + json_string(car_name_) + ", \"host\": " +
          json_string(host) + ", \"domain\": " + std::to_string(get_node_base_interface()->get_context()->get_domain_id()) +
          ", \"mode\": " + json_string(mode_) + ", \"map\": " + json_string(map_) + ", \"path\": " +
          json_string(path_) + ", \"battery\": " + battery + ", \"router\": " + std::to_string(router_port_) + "}");
        beacon_->announce();
      });
    event("Supervisor ready. Data in " + data_dir_);
  }

  ~Supervisor() override
  {
    for (auto & [name, c] : components_) {
      c.desired = false;
      stop(c);
    }
    for (int i = 0; i < 100 && any_running(); ++i) {
      reap();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    for (auto & [name, c] : components_) {
      if (c.pid > 0) {
        kill(-c.pid, SIGKILL);
      }
    }
    reap();
    for (auto & [name, c] : components_) {
      if (c.reader.joinable()) {
        c.reader.detach();
      }
    }
    console_->set_publisher(nullptr);
  }

private:
  void add_component(const std::string & name, const std::vector<std::string> & nodes)
  {
    auto & c = components_[name];
    c.name = name;
    c.expected_nodes = nodes;
    c.log_file = log_dir_ + "/" + name + ".log";
  }

  void event(const std::string & message)
  {
    message_ = message;
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    console_->add("supervisor", message);
  }

  std::vector<std::string> launch(const std::string & file, std::vector<std::string> args) const
  {
    std::vector<std::string> cmd{ros2_, "launch", "f1tenth_bringup", file};
    args.push_back(std::string("respawn:=") + (auto_restart_ ? "true" : "false"));
    args.push_back("params_dir:=" + params_dir_);
    cmd.insert(cmd.end(), args.begin(), args.end());
    return cmd;
  }

  bool load_path(const std::string & name, std::vector<ft::PathPoint> & points, std::string & message) const
  {
    std::string file = ft::find_path_file(name, paths_dir_);
    if (file.empty()) {
      message = "Path '" + name + "' not found";
      return false;
    }
    return ft::load_path_csv(file, points, message);
  }

  nav_msgs::msg::Path to_path(const std::vector<ft::PathPoint> & points)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    for (const auto & p : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  std::vector<std::string> list_maps() const {return ft::list_stems(maps_dir_, ".yaml");}

  // The map a path was made on, '' if unknown (cached by file time: this runs for every state message)
  std::string path_map(const std::string & name)
  {
    std::string csv = ft::find_path_file(name, paths_dir_);
    if (csv.empty()) {
      return "";
    }
    std::error_code ec;
    auto stamp = ft::fs::last_write_time(ft::path_info_file(csv), ec);
    auto it = path_maps_.find(csv);
    if (it == path_maps_.end() || it->second.first != stamp) {
      std::string map = ec ? std::string() : ft::load_path_info(csv).map;
      it = path_maps_.insert_or_assign(csv, std::make_pair(stamp, map)).first;
    }
    return it->second.second;
  }

  std::vector<std::string> list_paths() const
  {
    std::set<std::string> names;
    for (const auto & dir : ft::path_dirs(paths_dir_)) {
      for (const auto & n : ft::list_stems(dir, ".csv")) {
        names.insert(n);
      }
    }
    return {names.begin(), names.end()};
  }

  bool set_mode(const std::string & mode, const std::string & map, const std::string & path, std::string & message)
  {
    static const std::set<std::string> modes{"idle", "manual", "mapping", "path"};
    if (!modes.count(mode)) {
      message = "Unknown mode '" + mode + "'";
      return false;
    }
    if (!map.empty() && !ft::fs::exists(maps_dir_ + "/" + map + ".yaml")) {
      message = "Map '" + map + "' not found";
      return false;
    }
    if (!path.empty() && ft::find_path_file(path, paths_dir_).empty()) {
      message = "Path '" + path + "' not found";
      return false;
    }
    if (mode == "path" && map.empty()) {
      message = "Path recording needs a map to localize in";
      return false;
    }
    if (!path.empty() && mode != "mapping") {
      if (map.empty()) {
        message = "A path needs the map it was made on";
        return false;
      }
      std::string made_on = path_map(path);
      if (!made_on.empty() && made_on != map) {
        message = "Path '" + path + "' was made on map '" + made_on + "', not '" + map + "'";
        return false;
      }
    }
    mode_ = mode;
    map_ = mode == "mapping" ? "" : map;
    path_ = mode == "mapping" ? "" : path;
    for (auto & [name, c] : components_) {
      c.user_stopped = false;  // a new session runs everything it needs
    }
    update_desired();

    std::vector<ft::PathPoint> points;
    std::string error;
    path_pub_->publish(!path_.empty() && load_path(path_, points, error) ? to_path(points) : to_path({}));

    message = "Session: " + mode_ + (map_.empty() ? "" : ", map " + map_) + (path_.empty() ? "" : ", path " + path_);
    event(message);
    return true;
  }

  // Which components the session needs, and with what arguments
  void update_desired()
  {
    for (auto & [name, c] : components_) {
      c.desired = false;
    }
    auto want = [this](const std::string & name, std::vector<std::string> command) {
      components_[name].desired = true;
      components_[name].command = std::move(command);
    };
    if (start_core_ || mode_ != "idle") {
      want("core", launch("core_launch.py", {}));
    }
    std::string map_file = maps_dir_ + "/" + map_ + ".yaml";
    if ((mode_ == "manual" || mode_ == "path") && !map_.empty()) {
      want("localization", launch("localization_launch.py", {"map:=" + map_file}));
    }
    if (mode_ == "manual" && !map_.empty() && !path_.empty()) {
      want("follower", launch("follower_launch.py", {"path:=" + ft::find_path_file(path_, paths_dir_)}));
    }
    if (mode_ == "mapping") {
      want("slam", launch("slam_launch.py", {}));
    }
    if (mode_ == "path") {
      want("path_tools", launch("path_tools_launch.py",
        {"maps_dir:=" + maps_dir_, "paths_dir:=" + paths_dir_, "map:=" + map_}));
    }
    if (bag_) {
      std::vector<std::string> cmd{ros2_, "bag", "record", "-o", bag_name_};
      cmd.insert(cmd.end(), bag_topics_.begin(), bag_topics_.end());
      want("bag", cmd);
    }
    for (auto & [name, c] : components_) {
      if (c.user_stopped) {
        c.desired = false;
      }
    }
  }

  bool component_command(const std::string & name, const std::string & action, std::string & message)
  {
    if (name == "car") {
      return car_power(action, message);
    }
    auto it = components_.find(name);
    if (it == components_.end()) {
      message = "Unknown component '" + name + "'";
      return false;
    }
    auto & c = it->second;
    if (action == "restart") {
      if (!c.desired) {
        message = name + " is not part of this session";
        return false;
      }
      if (c.pid > 0) {
        c.restart_pending = true;
        stop(c);
      } else {  // not running (e.g. failed with auto restart off): start on the next tick
        c.status = "stopped";
        c.next_start = Clock::now();
      }
      message = "Restarting " + name;
    } else if (action == "stop") {
      c.user_stopped = true;
      c.desired = false;
      stop(c);
      message = "Stopped " + name + " (until the session changes)";
    } else if (action == "start") {
      c.user_stopped = false;
      update_desired();
      message = c.desired ? "Starting " + name : name + " is not part of this session";
    } else {
      message = "Unknown action '" + action + "'";
      return false;
    }
    event(message);
    return true;
  }

  // Reboots or powers off the car from the UI: stops the session, then runs systemctl through sudo (which must
  // not need a password, as the supervisor has no terminal)
  bool car_power(const std::string & action, std::string & message)
  {
    if (action != "poweroff" && action != "reboot") {
      message = "Unknown action '" + action + "' (poweroff or reboot)";
      return false;
    }
    if (std::system("sudo -n true > /dev/null 2>&1") != 0) {
      const char * user = std::getenv("USER");
      message = std::string("Can't ") + action + ": " + (user ? user : "this user") + " needs passwordless sudo";
      return false;
    }
    std::string ignored;
    set_mode("idle", "", "", ignored);
    power_action_ = action;
    power_requested_ = Clock::now();
    message = action == "poweroff" ? "Powering off the car: stopping everything first" :
      "Rebooting the car: stopping everything first";
    event(message);
    return true;
  }

  // Runs a command in its own process group (so stopping it stops everything it started); its output goes to
  // log_file through a reader thread. Returns the pid, or -1.
  pid_t spawn(std::vector<std::string> command, const std::string & name, const std::string & log_file,
    std::shared_ptr<std::atomic<bool>> quiet, std::thread & reader)
  {
    int fds[2];
    if (pipe2(fds, O_CLOEXEC) != 0) {
      event("Cannot start " + name + ": " + std::strerror(errno));
      return -1;
    }
    std::vector<char *> argv;
    for (auto & arg : command) {
      argv.push_back(arg.data());
    }
    argv.push_back(nullptr);
    pid_t pid = fork();
    if (pid == 0) {
      setsid();
      dup2(fds[1], STDOUT_FILENO);
      dup2(fds[1], STDERR_FILENO);
      execv(argv[0], argv.data());
      _exit(127);
    }
    close(fds[1]);
    if (pid < 0) {
      close(fds[0]);
      event("Cannot start " + name + ": fork failed");
      return -1;
    }
    if (reader.joinable()) {
      reader.detach();
    }
    reader = std::thread(forward_output, fds[0], name, log_file, get_logger(), quiet, console_);
    return pid;
  }

  void start(Component & c)
  {
    if (c.name == "slam") {
      mapping_paused_ = false;  // a fresh slam_toolbox takes scans
    }
    c.quiet->store(false);
    pid_t pid = spawn(c.command, c.name, c.log_file, c.quiet, c.reader);
    if (pid < 0) {
      return;
    }
    c.pid = pid;
    c.running_command = c.command;
    c.status = "starting";
    c.stopping = false;
    c.started = Clock::now();
    c.missing.clear();
    c.unresponsive.clear();
    c.missing_since = Clock::time_point();
    for (const auto & n : c.expected_nodes) {
      probes_.erase(n);  // judged afresh once the new process answers
    }
    RCLCPP_INFO(get_logger(), "Started %s (pid %d)", c.name.c_str(), pid);
  }

  // Keeps the component's output in its log file and surfaces errors in the supervisor's log.
  // Static with its own logger copy: it can outlive the node while orphaned nodes still hold the pipe.
  static void forward_output(int fd, std::string name, std::string log_file, rclcpp::Logger logger,
    std::shared_ptr<std::atomic<bool>> quiet, std::shared_ptr<Console> console)
  {
    std::error_code ec;
    if (ft::fs::file_size(log_file, ec) > 20u * 1024 * 1024 && !ec) {
      ft::fs::rename(log_file, log_file + ".1", ec);  // keep one previous log
    }
    std::ofstream log(log_file, std::ios::app);
    log << "==== " << timestamp() << " start\n";
    std::string pending;
    char buf[4096];
    ssize_t n;
    while ((n = read(fd, buf, sizeof(buf))) > 0) {
      pending.append(buf, static_cast<size_t>(n));
      size_t pos;
      while ((pos = pending.find('\n')) != std::string::npos) {
        std::string line = pending.substr(0, pos);
        pending.erase(0, pos + 1);
        log << line << '\n';
        console->add(name, line);
        if (!quiet->load() && (line.find("process has died") != std::string::npos ||
          line.find("Traceback") != std::string::npos || line.find("[FATAL]") != std::string::npos))
        {
          RCLCPP_ERROR(logger, "[%s] %s", name.c_str(), line.c_str());
        }
      }
      log.flush();
    }
    close(fd);
  }

  void stop(Component & c)
  {
    if (c.pid > 0 && !c.stopping) {
      c.quiet->store(true);
      kill(-c.pid, SIGINT);
      c.stopping = true;
      c.status = "stopping";
      c.stop_requested = Clock::now();
    }
  }

  bool any_running() const
  {
    for (const auto & [name, c] : components_) {
      if (c.pid > 0) {
        return true;
      }
    }
    return false;
  }

  // Collects exited processes: our launches and orphaned nodes re-parented to us
  void reap()
  {
    int st;
    pid_t pid;
    while ((pid = waitpid(-1, &st, WNOHANG)) > 0) {
      auto job = jobs_.find(pid);
      if (job != jobs_.end()) {
        auto done = job->second;
        jobs_.erase(job);
        done(WIFEXITED(st) && WEXITSTATUS(st) == 0);
        continue;
      }
      for (auto & [name, c] : components_) {
        if (c.pid != pid) {
          continue;
        }
        bool expected = c.stopping || !c.desired;
        c.pid = -1;
        c.stopping = false;
        if (expected || c.restart_pending) {
          c.status = "stopped";
          c.backoff = 2.0;
          if (c.restart_pending) {
            c.restart_pending = false;
            c.next_start = Clock::now();
          }
        } else {
          c.status = "failed";
          ++c.restarts;
          c.next_start = Clock::now() + std::chrono::milliseconds(static_cast<int>(c.backoff * 1000));
          RCLCPP_ERROR(get_logger(), "%s exited unexpectedly (%s)%s", c.name.c_str(),
            WIFSIGNALED(st) ? strsignal(WTERMSIG(st)) : ("code " + std::to_string(WEXITSTATUS(st))).c_str(),
            auto_restart_ ? (", restarting in " + std::to_string(static_cast<int>(c.backoff)) + " s").c_str() : "");
          c.backoff = std::min(c.backoff * 2.0, 30.0);
        }
      }
    }
  }

  void tick()
  {
    reap();
    // Power off / reboot once the session has stopped (or after 15 s)
    if (!power_action_.empty() && (!any_running() || seconds_since(power_requested_) > 15.0)) {
      std::string action = power_action_;
      power_action_.clear();
      std::thread reader;
      pid_t pid = spawn({which("sudo"), "-n", which("systemctl"), action}, "power", log_dir_ + "/power.log",
        std::make_shared<std::atomic<bool>>(false), reader);
      reader.detach();
      if (pid > 0) {
        jobs_[pid] = [this, action](bool ok) {
            if (!ok) {
              event("The " + action + " command failed (see " + log_dir_ + "/power.log)");
            }
          };
      }
    }
    // Graph check every other tick
    std::set<std::string> nodes;
    bool check_nodes = (++ticks_ % 2) == 0;
    if (check_nodes) {
      for (const auto & n : get_node_names()) {
        nodes.insert(n);
      }
    }
    for (auto & [name, c] : components_) {
      if (c.pid > 0) {
        if (!c.desired || c.running_command != c.command) {
          if (c.desired) {
            c.restart_pending = true;  // arguments changed (e.g. another map): restart with the new ones
          }
          stop(c);
        }
        if (c.stopping) {
          double waited = seconds_since(c.stop_requested);
          if (waited > 15.0) {
            kill(-c.pid, SIGKILL);
          } else if (waited > 10.0) {
            kill(-c.pid, SIGTERM);
          }
        } else if (check_nodes) {
          check_component_nodes(c, nodes);
        }
      } else if (c.desired && Clock::now() >= c.next_start && (c.status != "failed" || auto_restart_)) {
        start(c);
      }
    }
    publish_state();
  }

  // Pings a node's parameter service (answered by its executor, so it fails when the node is stuck).
  // Returns true when the node has stopped answering.
  bool probe_hung(const std::string & node)
  {
    if (probe_exclude_.count(node)) {
      return false;
    }
    auto & p = probes_[node];
    if (!p.client) {
      p.client = create_client<rcl_interfaces::srv::GetParameters>(node + "/get_parameters");
    }
    if (p.pending >= 0 && seconds_since(p.sent) > probe_timeout_) {
      p.client->remove_pending_request(p.pending);
      p.pending = -1;
      ++p.failures;
    }
    if (p.pending < 0 && seconds_since(p.sent) >= probe_period_ && p.client->service_is_ready()) {
      p.sent = Clock::now();
      p.pending = p.client->async_send_request(std::make_shared<rcl_interfaces::srv::GetParameters::Request>(),
        [this, node](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture) {
          auto & q = probes_[node];
          q.pending = -1;
          q.failures = 0;
          q.answered = true;
        }).request_id;
    }
    return p.answered && p.failures >= probe_failures_;
  }

  // True when most probed nodes fail at once: then the problem is more likely this node's own connection
  bool probes_failing_everywhere() const
  {
    size_t judged = 0, failing = 0;
    for (const auto & [name, p] : probes_) {
      if (p.answered) {
        ++judged;
        failing += p.failures >= 2 ? 1 : 0;
      }
    }
    return judged >= 4 && failing * 2 > judged;
  }

  void check_component_nodes(Component & c, const std::set<std::string> & nodes)
  {
    c.missing.clear();
    c.unresponsive.clear();
    for (const auto & n : c.expected_nodes) {
      if (!nodes.count(n)) {
        c.missing.push_back(n);
      } else if (probe_hung(n)) {
        c.unresponsive.push_back(n);
      }
    }
    if (!c.unresponsive.empty() && c.missing.empty()) {
      c.status = "degraded";
      if (probes_failing_everywhere()) {
        return;
      }
      if (auto_restart_) {
        ++c.restarts;
        event("Restarting " + c.name + ": " + c.unresponsive.front() + " stopped responding");
        c.restart_pending = true;
        stop(c);
      }
      return;
    }
    if (c.missing.empty()) {
      c.status = "running";
      c.missing_since = Clock::time_point();
      c.backoff = 2.0;
      return;
    }
    if (seconds_since(c.started) < startup_grace_) {
      return;  // still starting: nodes may take a while to appear
    }
    c.status = "degraded";
    if (c.missing_since == Clock::time_point()) {
      c.missing_since = Clock::now();
    } else if (auto_restart_ && seconds_since(c.missing_since) > missing_timeout_) {
      ++c.restarts;
      event("Restarting " + c.name + ": " + std::to_string(c.missing.size()) + " node(s) missing (" + c.missing.front() + ")");
      c.restart_pending = true;
      c.missing_since = Clock::time_point();
      stop(c);
    }
  }

  void publish_state()
  {
    msg::SupervisorState state;
    state.header.stamp = now();
    state.mode = mode_;
    state.map = map_;
    state.path = path_;
    state.auto_restart = auto_restart_;
    state.bag_recording = bag_;
    state.mapping_paused = mapping_paused_;
    for (const auto & [name, c] : components_) {
      if (!c.desired && !c.user_stopped && c.pid <= 0 && c.status == "stopped" && c.restarts == 0) {
        continue;  // not part of this session
      }
      msg::ComponentState cs;
      cs.name = name;
      cs.status = c.status;
      cs.pid = c.pid;
      cs.restarts = c.restarts;
      cs.uptime = c.pid > 0 ? static_cast<float>(seconds_since(c.started)) : 0.0f;
      for (size_t i = 3; i < c.running_command.size(); ++i) {
        cs.args += (i > 3 ? " " : "") + c.running_command[i];
      }
      cs.missing_nodes = c.missing;
      cs.unresponsive_nodes = c.unresponsive;
      state.components.push_back(cs);
    }
    state.maps = list_maps();
    state.paths = list_paths();
    for (const auto & p : state.paths) {
      state.path_maps.push_back(path_map(p));
    }
    state.maps_dir = maps_dir_;
    state.paths_dir = paths_dir_;
    state.message = message_;
    state_pub_->publish(state);
  }

  void map_preview(const srv::GetMapPreview::Request & req, srv::GetMapPreview::Response & res)
  {
    ft::OccupancyMap map;
    if (!ft::load_map(maps_dir_ + "/" + req.name + ".yaml", map, res.message)) {
      res.success = false;
      return;
    }
    int max_size = req.max_size > 0 ? static_cast<int>(req.max_size) : 512;
    int factor = std::max(1, (std::max(map.width, map.height) + max_size - 1) / max_size);
    int w = (map.width + factor - 1) / factor, h = (map.height + factor - 1) / factor;
    res.image.width = w;
    res.image.height = h;
    res.image.encoding = "mono8";
    res.image.step = w;
    res.image.data.assign(static_cast<size_t>(w) * h, 127);
    for (int r = 0; r < h; ++r) {
      for (int col = 0; col < w; ++col) {
        bool occupied = false, free = false;
        for (int dr = 0; dr < factor && r * factor + dr < map.height; ++dr) {
          for (int dc = 0; dc < factor && col * factor + dc < map.width; ++dc) {
            int8_t v = map.cells[static_cast<size_t>(r * factor + dr) * map.width + col * factor + dc];
            occupied |= v == 100;
            free |= v == 0;
          }
        }
        res.image.data[static_cast<size_t>(r) * w + col] = occupied ? 0 : (free ? 255 : 127);
      }
    }
    res.resolution = static_cast<float>(map.resolution * factor);
    res.origin.position.x = map.origin_x;
    res.origin.position.y = map.origin_y;
    res.origin.orientation.z = std::sin(map.origin_yaw / 2.0);
    res.origin.orientation.w = std::cos(map.origin_yaw / 2.0);
    res.success = true;
  }

  // slam_toolbox only has a pause toggle, so the supervisor tracks the state
  void set_mapping_paused(std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> service,
    std::shared_ptr<rmw_request_id_t> header, bool paused)
  {
    auto reply = [service, header](bool ok, const std::string & message) {
        std_srvs::srv::SetBool::Response res;
        res.success = ok;
        res.message = message;
        service->send_response(*header, res);
      };
    if (mode_ != "mapping" || !pause_client_->service_is_ready()) {
      reply(false, "Mapping is not running");
      return;
    }
    if (paused == mapping_paused_) {
      reply(true, paused ? "Mapping paused" : "Mapping running");
      return;
    }
    pause_client_->async_send_request(std::make_shared<slam_toolbox::srv::Pause::Request>(),
      [this, reply, paused](rclcpp::Client<slam_toolbox::srv::Pause>::SharedFuture) {
        mapping_paused_ = paused;
        event(paused ? "Mapping paused: new scans are ignored" : "Mapping resumed");
        reply(true, message_);
      });
  }

  void save_map(std::shared_ptr<rclcpp::Service<srv::SaveFile>> service, std::shared_ptr<rmw_request_id_t> header,
    const std::string & name)
  {
    auto reply = [service, header, this](bool ok, const std::string & message) {
      srv::SaveFile::Response res;
      res.success = ok;
      res.message = message;
      service->send_response(*header, res);
      if (ok) {event(message);} else {RCLCPP_WARN(get_logger(), "%s", message.c_str());}
    };
    if (mode_ != "mapping") {
      return reply(false, "Maps are saved from mapping mode");
    }
    if (!ft::valid_name(name)) {
      return reply(false, "Use letters, digits, '-' or '_' for the map name");
    }
    if (ft::fs::exists(maps_dir_ + "/" + name + ".yaml")) {
      return reply(false, "A map named '" + name + "' already exists");
    }
    // map_saver_cli directly rather than slam_toolbox/save_map: that one uses map_saver's default 2 s timeout,
    // which a fresh node over Zenoh needs longer than to receive the map
    std::string base = maps_dir_ + "/" + name;
    std::thread reader;
    pid_t pid = spawn({ros2_, "run", "nav2_map_server", "map_saver_cli", "-f", base, "--ros-args",
        "-p", "save_map_timeout:=10.0", "-p", "map_subscribe_transient_local:=true"},
      "map_saver", log_dir_ + "/map_saver.log", std::make_shared<std::atomic<bool>>(false), reader);
    reader.detach();
    if (pid < 0) {
      return reply(false, "Could not start map_saver_cli");
    }
    jobs_[pid] = [this, reply, base, name](bool ok) {
        if (!ok || !ft::fs::exists(base + ".yaml")) {
          return reply(false, "Could not save the map (no map from slam_toolbox yet?); see " + log_dir_ + "/map_saver.log");
        }
        // Also keep the pose graph, so mapping can continue from this map later
        if (!serialize_client_->service_is_ready()) {
          return reply(true, "Saved map '" + name + "' (pose graph not saved)");
        }
        auto ser = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();
        ser->filename = base;
        serialize_client_->async_send_request(ser,
          [reply, name](rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedFuture f) {
            bool graph = f.get()->result == slam_toolbox::srv::SerializePoseGraph::Response::RESULT_SUCCESS;
            reply(true, "Saved map '" + name + "'" + (graph ? " with its pose graph" : " (pose graph not saved)"));
          });
      };
  }

  // Writes all current parameters of a node to params_dir/<node>.yaml; launch files load it as an override
  void save_params(std::shared_ptr<rclcpp::Service<srv::SaveFile>> service, std::shared_ptr<rmw_request_id_t> header,
    const std::string & node)
  {
    auto reply = [service, header, this](bool ok, const std::string & message) {
      srv::SaveFile::Response res;
      res.success = ok;
      res.message = message;
      service->send_response(*header, res);
      if (ok) {event(message);} else {RCLCPP_WARN(get_logger(), "%s", message.c_str());}
    };
    std::string base = node.substr(node.find_last_of('/') + 1);
    if (!ft::valid_name(base)) {
      return reply(false, "Invalid node name '" + node + "'");
    }
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(this, node);
    if (!client->wait_for_service(std::chrono::seconds(3))) {
      return reply(false, node + " is not running");
    }
    client->list_parameters({}, 0, [this, client, reply, node, base](std::shared_future<rcl_interfaces::msg::ListParametersResult> listed) {
      std::vector<std::string> names;
      for (const auto & n : listed.get().names) {
        if (n.rfind("qos_overrides.", 0) != 0 && n != "use_sim_time") {
          names.push_back(n);
        }
      }
      client->get_parameters(names, [this, client, reply, node, base](std::shared_future<std::vector<rclcpp::Parameter>> got) {
        std::string file = params_dir_ + "/" + base + ".yaml";
        std::ofstream out(file);
        if (!out) {
          return reply(false, "Cannot write " + file);
        }
        out << "# Saved from the F1TENTH UI on " << timestamp() << "; delete this file to go back to the defaults\n";
        out << base << ":\n  ros__parameters:\n";
        for (const auto & p : got.get()) {
          out << "    " << p.get_name() << ": " << yaml_value(p) << "\n";
        }
        reply(true, "Saved " + std::to_string(got.get().size()) + " parameters of " + node + " to the car");
      });
    });
  }

  static std::string yaml_value(const rclcpp::Parameter & p)
  {
    switch (p.get_type()) {
      case rclcpp::ParameterType::PARAMETER_STRING: {
          std::string s = p.as_string(), escaped;
          for (char ch : s) {
            if (ch == '"' || ch == '\\') {escaped += '\\';}
            escaped += ch;
          }
          return "\"" + escaped + "\"";
        }
      case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: {
          std::string out = "[";
          for (size_t i = 0; i < p.as_string_array().size(); ++i) {
            out += (i ? ", \"" : "\"") + p.as_string_array()[i] + "\"";
          }
          return out + "]";
        }
      case rclcpp::ParameterType::PARAMETER_DOUBLE: {
          std::ostringstream ss;
          ss << std::setprecision(10) << p.as_double();
          std::string s = ss.str();
          return s.find_first_of(".en") == std::string::npos ? s + ".0" : s;  // keep it a double
        }
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
          std::ostringstream ss;
          ss << "[";
          for (size_t i = 0; i < p.as_double_array().size(); ++i) {
            std::ostringstream v;
            v << std::setprecision(10) << p.as_double_array()[i];
            std::string s = v.str();
            ss << (i ? ", " : "") << (s.find_first_of(".en") == std::string::npos ? s + ".0" : s);
          }
          ss << "]";
          return ss.str();
        }
      default:
        return p.value_to_string();
    }
  }

  bool file_op(const srv::FileOp::Request & req, std::string & message)
  {
    bool is_map = req.kind == "map", is_path = req.kind == "path", is_params = req.kind == "params";
    if (!is_map && !is_path && !is_params) {
      message = "Unknown kind '" + req.kind + "'";
      return false;
    }
    if (mode_ != "idle" && ((is_map && req.name == map_) || (is_path && req.name == path_))) {
      message = "'" + req.name + "' is in use; end the session first";
      return false;
    }
    std::string dir = is_map ? maps_dir_ : (is_path ? paths_dir_ : params_dir_);
    std::vector<std::string> exts = is_map ? std::vector<std::string>{".yaml", ".pgm", ".posegraph", ".data"} :
      (is_path ? std::vector<std::string>{".csv", ".yaml"} : std::vector<std::string>{".yaml"});
    if (!ft::fs::exists(dir + "/" + req.name + exts.front())) {
      message = "'" + req.name + "' not found" + (is_path ? " (paths shipped with ugv_race can't be changed here)" : "");
      return false;
    }
    std::error_code ec;
    if (req.action == "delete") {
      // Recoverable: moved to a trash folder
      std::string trash = dir + "/.trash";
      ft::fs::create_directories(trash, ec);
      std::string stamp = timestamp();
      for (const auto & ext : exts) {
        std::string src = dir + "/" + req.name + ext;
        if (ft::fs::exists(src)) {
          ft::fs::rename(src, trash + "/" + req.name + "-" + stamp + ext, ec);
        }
      }
      message = "Moved '" + req.name + "' to " + trash;
    } else if (req.action == "rename") {
      if (!ft::valid_name(req.new_name)) {
        message = "Use letters, digits, '-' or '_' for the new name";
        return false;
      }
      if (ft::fs::exists(dir + "/" + req.new_name + exts.front())) {
        message = "'" + req.new_name + "' already exists";
        return false;
      }
      for (const auto & ext : exts) {
        std::string src = dir + "/" + req.name + ext;
        if (ft::fs::exists(src)) {
          ft::fs::rename(src, dir + "/" + req.new_name + ext, ec);
        }
      }
      if (is_map) {  // the YAML names its image file
        std::string yaml = dir + "/" + req.new_name + ".yaml";
        std::ifstream in(yaml);
        std::stringstream content;
        std::string line;
        while (std::getline(in, line)) {
          content << (line.rfind("image:", 0) == 0 ? "image: " + req.new_name + ".pgm" : line) << "\n";
        }
        in.close();
        std::ofstream(yaml) << content.str();
        // Paths made on this map follow it
        for (const auto & path : ft::list_stems(paths_dir_, ".csv")) {
          std::string csv = paths_dir_ + "/" + path + ".csv";
          auto info = ft::load_path_info(csv);
          if (info.map == req.name) {
            info.map = req.new_name;
            ft::save_path_info(csv, info);
          }
        }
      }
      message = "Renamed '" + req.name + "' to '" + req.new_name + "'";
    } else {
      message = "Unknown action '" + req.action + "'";
      return false;
    }
    if (ec) {
      message += " (with errors: " + ec.message() + ")";
    }
    event(message);
    return true;
  }

  std::string data_dir_, maps_dir_, paths_dir_, params_dir_, log_dir_, ros2_;
  bool auto_restart_, start_core_, bag_ = false;
  double startup_grace_, missing_timeout_;
  std::vector<std::string> bag_topics_;
  std::string bag_name_, mode_ = "idle", map_, path_, message_;
  std::map<std::string, Component> components_;
  std::map<pid_t, std::function<void(bool)>> jobs_;  // one-shot processes (map saving) and what to do when they end
  std::map<std::string, std::pair<ft::fs::file_time_type, std::string>> path_maps_;
  std::shared_ptr<Console> console_ = std::make_shared<Console>();
  std::string car_name_;
  int router_port_ = 7447;
  double battery_ = -1.0;
  Clock::time_point telemetry_time_;
  rclcpp::Subscription<msg::Telemetry>::SharedPtr telemetry_sub_;
  std::unique_ptr<Beacon> beacon_;
  rclcpp::TimerBase::SharedPtr beacon_timer_;
  std::string power_action_;  // poweroff | reboot, pending until the session has stopped
  Clock::time_point power_requested_;
  rclcpp::Service<srv::GetConsole>::SharedPtr console_srv_, rosout_srv_;
  rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr rosout_pub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
  std::deque<rcl_interfaces::msg::Log> rosout_history_;
  uint64_t ticks_ = 0;

  struct Probe
  {
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client;
    int64_t pending = -1;
    Clock::time_point sent;
    int failures = 0;
    bool answered = false;  // only nodes that answered once are judged
  };
  std::map<std::string, Probe> probes_;
  std::set<std::string> probe_exclude_;
  double probe_period_, probe_timeout_;
  int probe_failures_;

  rclcpp::Publisher<msg::SupervisorState>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<srv::ComponentCommand>::SharedPtr component_srv_;
  rclcpp::Service<srv::FileOp>::SharedPtr file_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr auto_restart_srv_, bag_srv_;
  rclcpp::Service<srv::GetMapPreview>::SharedPtr preview_srv_;
  rclcpp::Service<srv::GetPath>::SharedPtr get_path_srv_;
  rclcpp::Service<srv::SaveFile>::SharedPtr save_map_srv_, save_params_srv_;
  rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedPtr serialize_client_;
  rclcpp::Client<slam_toolbox::srv::Pause>::SharedPtr pause_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_srv_;
  bool mapping_paused_ = false;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    // Two supervisors would both start the car's nodes: refuse to run if one is already up. (Launch remaps the
    // node name for the whole process, so this probe is called supervisor too: look for a second one.)
    auto probe = std::make_shared<rclcpp::Node>("supervisor");
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    auto names = probe->get_node_names();
    std::string self = probe->get_fully_qualified_name();
    if (std::count(names.begin(), names.end(), self) > 1) {
      RCLCPP_FATAL(probe->get_logger(), "Another supervisor is already running (car service or a manual launch)");
      rclcpp::shutdown();
      return 1;
    }
  }
  auto node = std::make_shared<Supervisor>();
  rclcpp::spin(node);
  node.reset();  // stops all components
  rclcpp::shutdown();
  return 0;
}
