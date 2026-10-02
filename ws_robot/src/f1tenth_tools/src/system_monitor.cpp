// Car health on /diagnostics: Jetson CPU/GPU load and temperatures, memory, disk, Wi-Fi signal, and the
// publish rate of key topics against what they should be. Rates are measured here on the car so a remote UI
// does not have to subscribe to every sensor stream.
#include <sys/statvfs.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace fs = std::filesystem;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

namespace
{
std::string read_line(const std::string & file)
{
  std::ifstream in(file);
  std::string line;
  std::getline(in, line);
  return line;
}

double read_number(const std::string & file, double fallback = -1.0)
{
  try {
    return std::stod(read_line(file));
  } catch (...) {
    return fallback;
  }
}

KeyValue kv(const std::string & key, const std::string & value)
{
  KeyValue k;
  k.key = key;
  k.value = value;
  return k;
}

std::string fmt(double value, int decimals = 1)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
  return buf;
}
}  // namespace

class SystemMonitor : public rclcpp::Node
{
public:
  SystemMonitor() : Node("system_monitor")
  {
    hardware_id_ = declare_parameter<std::string>("hardware_id", "f1tenth");
    wifi_interface_ = declare_parameter<std::string>("wifi_interface", "wlP1p1s0");
    auto topics = declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{
      "/scan", "/sensors/core", "/sensors/imu/raw", "/odom/wheel", "/odom/filtered", "/joy",
      "/camera/image_raw/compressed", "/f1tenth/telemetry"});
    auto rates = declare_parameter<std::vector<double>>("expected_rates", std::vector<double>{
      40.0, 50.0, 50.0, 50.0, 50.0, 20.0, 30.0, 30.0});
    for (size_t i = 0; i < topics.size(); ++i) {
      monitors_[topics[i]].expected = i < rates.size() ? rates[i] : 0.0;
    }

    pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    last_tick_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {tick();});
  }

private:
  struct TopicMonitor
  {
    double expected = 0.0;
    double rate = 0.0;             // smoothed, Hz
    std::shared_ptr<rclcpp::GenericSubscription> sub;
    std::shared_ptr<std::atomic<uint64_t>> count = std::make_shared<std::atomic<uint64_t>>(0);
  };

  DiagnosticStatus status(const std::string & name, uint8_t level, const std::string & message)
  {
    DiagnosticStatus s;
    s.name = "f1tenth/" + name;
    s.hardware_id = hardware_id_;
    s.level = level;
    s.message = message;
    return s;
  }

  void tick()
  {
    auto t = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(t - last_tick_).count();
    last_tick_ = t;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(cpu());
    array.status.push_back(gpu());
    array.status.push_back(memory());
    array.status.push_back(disk());
    array.status.push_back(wifi());
    array.status.push_back(thermal());
    topic_rates(array, dt);
    pub_->publish(array);
  }

  DiagnosticStatus cpu()
  {
    // Overall usage from /proc/stat deltas
    std::ifstream in("/proc/stat");
    std::string label;
    uint64_t user, nice, system, idle, iowait, irq, softirq, steal;
    in >> label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
    uint64_t busy = user + nice + system + irq + softirq + steal, total = busy + idle + iowait;
    double usage = total > cpu_total_ ? 100.0 * (busy - cpu_busy_) / (total - cpu_total_) : 0.0;
    cpu_busy_ = busy;
    cpu_total_ = total;
    double temp = zone_temp("cpu-thermal");
    uint8_t level = temp > 95 ? DiagnosticStatus::ERROR : (temp > 80 || usage > 90 ? DiagnosticStatus::WARN : DiagnosticStatus::OK);
    auto s = status("cpu", level, fmt(usage, 0) + "% " + fmt(temp, 0) + " C");
    s.values = {kv("usage", fmt(usage)), kv("temperature", fmt(temp)),
      kv("cores", std::to_string(std::thread::hardware_concurrency())), kv("load_1min", read_line("/proc/loadavg").substr(0, 4))};
    return s;
  }

  DiagnosticStatus gpu()
  {
    double load = -1.0;
    for (const char * f : {"/sys/devices/platform/gpu.0/load", "/sys/devices/platform/17000000.gpu/load"}) {
      load = read_number(f);
      if (load >= 0) {
        break;
      }
    }
    double temp = zone_temp("gpu-thermal");
    if (load < 0) {
      return status("gpu", DiagnosticStatus::OK, "not available");
    }
    load /= 10.0;  // reported in 0.1 %
    uint8_t level = temp > 95 ? DiagnosticStatus::ERROR : (temp > 80 ? DiagnosticStatus::WARN : DiagnosticStatus::OK);
    auto s = status("gpu", level, fmt(load, 0) + "% " + fmt(temp, 0) + " C");
    s.values = {kv("usage", fmt(load)), kv("temperature", fmt(temp))};
    return s;
  }

  DiagnosticStatus memory()
  {
    std::ifstream in("/proc/meminfo");
    std::string key;
    double value, total = 0, available = 0;
    std::string unit;
    while (in >> key >> value >> unit) {
      if (key == "MemTotal:") {total = value / 1024.0;}
      if (key == "MemAvailable:") {available = value / 1024.0;}
    }
    double used = total - available, percent = total > 0 ? 100.0 * used / total : 0.0;
    uint8_t level = percent > 95 ? DiagnosticStatus::ERROR : (percent > 85 ? DiagnosticStatus::WARN : DiagnosticStatus::OK);
    auto s = status("memory", level, fmt(used / 1024.0) + " / " + fmt(total / 1024.0) + " GB");
    s.values = {kv("usage", fmt(percent)), kv("used_mb", fmt(used, 0)), kv("total_mb", fmt(total, 0))};
    return s;
  }

  DiagnosticStatus disk()
  {
    struct statvfs st{};
    if (statvfs("/", &st) != 0) {
      return status("disk", DiagnosticStatus::WARN, "not available");
    }
    double total = static_cast<double>(st.f_blocks) * st.f_frsize / 1e9;
    double free = static_cast<double>(st.f_bavail) * st.f_frsize / 1e9;
    double percent = total > 0 ? 100.0 * (total - free) / total : 0.0;
    uint8_t level = percent > 95 ? DiagnosticStatus::ERROR : (percent > 90 ? DiagnosticStatus::WARN : DiagnosticStatus::OK);
    auto s = status("disk", level, fmt(free, 0) + " GB free");
    s.values = {kv("usage", fmt(percent)), kv("free_gb", fmt(free)), kv("total_gb", fmt(total))};
    return s;
  }

  DiagnosticStatus wifi()
  {
    std::ifstream in("/proc/net/wireless");
    std::string line;
    while (std::getline(in, line)) {
      std::istringstream ss(line);
      std::string iface;
      ss >> iface;
      if (iface == wifi_interface_ + ":") {
        std::string status_field;
        double quality, level_dbm;
        ss >> status_field >> quality >> level_dbm;
        uint8_t lvl = level_dbm < -80 ? DiagnosticStatus::ERROR : (level_dbm < -70 ? DiagnosticStatus::WARN : DiagnosticStatus::OK);
        auto s = status("wifi", lvl, fmt(level_dbm, 0) + " dBm");
        s.values = {kv("interface", wifi_interface_), kv("signal_dbm", fmt(level_dbm, 0)), kv("quality", fmt(quality, 0) + "/70")};
        return s;
      }
    }
    return status("wifi", DiagnosticStatus::WARN, wifi_interface_ + " not connected");
  }

  DiagnosticStatus thermal()
  {
    auto s = status("thermal", DiagnosticStatus::OK, "");
    double hottest = 0.0;
    for (const auto & zone : thermal_zones()) {
      double t = read_number(zone.second) / 1000.0;
      hottest = std::max(hottest, t);
      s.values.push_back(kv(zone.first, fmt(t)));
    }
    s.level = hottest > 95 ? DiagnosticStatus::ERROR : (hottest > 85 ? DiagnosticStatus::WARN : DiagnosticStatus::OK);
    s.message = "max " + fmt(hottest, 0) + " C";
    return s;
  }

  // name -> temp file, cached
  const std::vector<std::pair<std::string, std::string>> & thermal_zones()
  {
    if (zones_.empty()) {
      std::error_code ec;
      for (const auto & entry : fs::directory_iterator("/sys/class/thermal", ec)) {
        if (entry.path().filename().string().rfind("thermal_zone", 0) == 0) {
          zones_.emplace_back(read_line((entry.path() / "type").string()), (entry.path() / "temp").string());
        }
      }
      std::sort(zones_.begin(), zones_.end());
    }
    return zones_;
  }

  double zone_temp(const std::string & type)
  {
    for (const auto & zone : thermal_zones()) {
      if (zone.first == type) {
        return read_number(zone.second) / 1000.0;
      }
    }
    return -1.0;
  }

  void topic_rates(diagnostic_msgs::msg::DiagnosticArray & array, double dt)
  {
    // Subscribe once a topic shows up in the graph (its type is needed for a generic subscription)
    auto graph = get_topic_names_and_types();
    for (auto & [topic, mon] : monitors_) {
      if (!mon.sub) {
        auto it = graph.find(topic);
        if (it != graph.end() && !it->second.empty()) {
          auto count = mon.count;
          mon.sub = create_generic_subscription(topic, it->second.front(), rclcpp::SensorDataQoS(),
              [count](std::shared_ptr<const rclcpp::SerializedMessage>) {(*count)++;});
        }
      }
      double instant = mon.sub && dt > 0 ? mon.count->exchange(0) / dt : 0.0;
      // Smooth over a few seconds so one slow second (or a camera dropping frames in dim light) doesn't flag
      mon.rate = instant == 0.0 ? 0.0 : (mon.rate > 0.0 ? 0.7 * mon.rate + 0.3 * instant : instant);
      double rate = mon.rate;
      uint8_t level = DiagnosticStatus::OK;
      std::string message = fmt(rate) + " Hz";
      if (!mon.sub) {
        level = DiagnosticStatus::STALE;
        message = "not published";
      } else if (rate == 0.0) {
        level = DiagnosticStatus::ERROR;
        message = "no messages";
      } else if (mon.expected > 0 && rate < 0.6 * mon.expected) {
        level = DiagnosticStatus::WARN;
      }
      auto s = status("topic" + topic, level, message);
      s.values = {kv("rate", fmt(rate)), kv("expected", fmt(mon.expected))};
      array.status.push_back(s);
    }
  }

  std::string hardware_id_, wifi_interface_;
  std::map<std::string, TopicMonitor> monitors_;
  std::vector<std::pair<std::string, std::string>> zones_;
  uint64_t cpu_busy_ = 0, cpu_total_ = 0;
  std::chrono::steady_clock::time_point last_tick_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemMonitor>());
  rclcpp::shutdown();
  return 0;
}
