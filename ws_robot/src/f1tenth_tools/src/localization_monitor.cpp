// Checks the localization and sets it for the UI:
//  - /f1tenth/localization: how well the lidar scan lines up with the map's walls at the estimated pose
//    ("match"), with hysteresis into localized / not localized
//  - ~/place_at_start: tells AMCL the car stands on the selected path's start line, facing along the path
//  - ~/locate: searches the whole map for the pose where the current scan fits best, then tells AMCL
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "f1tenth_bringup/msg/localization_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using LocalizationState = f1tenth_bringup::msg::LocalizationState;
using PoseWithCovariance = geometry_msgs::msg::PoseWithCovarianceStamped;
using Trigger = std_srvs::srv::Trigger;

namespace
{
double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

struct Candidate
{
  double score, x, y, yaw;  // laser pose in the grid frame (m, rad)
};
}  // namespace

class LocalizationMonitor : public rclcpp::Node
{
public:
  LocalizationMonitor() : Node("localization_monitor")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    tolerance_ = declare_parameter<double>("tolerance", 0.15);        // m: a lidar point this close to a wall matches
    match_good_ = declare_parameter<double>("match_good", 0.6);       // localized from this match up
    match_lost_ = declare_parameter<double>("match_lost", 0.4);       // and lost below this (hysteresis)
    max_range_ = declare_parameter<double>("max_range", 12.0);        // m, farther points are ignored
    locate_min_fit_ = declare_parameter<double>("locate_min_fit", 0.5);
    double rate = declare_parameter<double>("rate", 5.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    rclcpp::QoS latched(1);
    latched.transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", latched,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr map) {set_map(*map);});
    path_sub_ = create_subscription<nav_msgs::msg::Path>("/f1tenth/selected_path", latched,
      [this](nav_msgs::msg::Path::SharedPtr path) {path_ = *path;});
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr scan) {scan_ = scan;});
    amcl_sub_ = create_subscription<PoseWithCovariance>("/amcl_pose", 10, [this](PoseWithCovariance::SharedPtr p) {
        const auto & c = p->pose.covariance;
        std_xy_ = std::sqrt(std::max(c[0], c[7]));
        std_yaw_ = std::sqrt(c[35]);
      });
    initial_pub_ = create_publisher<PoseWithCovariance>("/initialpose", 10);
    state_pub_ = create_publisher<LocalizationState>("/f1tenth/localization", 10);
    start_srv_ = create_service<Trigger>("~/place_at_start",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        res->success = place_at_start(res->message);
      });
    locate_srv_ = create_service<Trigger>("~/locate",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        auto start = std::chrono::steady_clock::now();
        res->success = locate(res->message);
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        RCLCPP_INFO(get_logger(), "Locate: %s (%.0f ms)", res->message.c_str(), ms);
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / rate)), [this]() {update();});
  }

private:
  // ---- map ----

  void set_map(const nav_msgs::msg::OccupancyGrid & map)
  {
    width_ = static_cast<int>(map.info.width);
    height_ = static_cast<int>(map.info.height);
    resolution_ = map.info.resolution;
    origin_x_ = map.info.origin.position.x;
    origin_y_ = map.info.origin.position.y;
    origin_yaw_ = yaw_of(map.info.origin.orientation);
    size_t cells = static_cast<size_t>(width_) * height_;
    if (cells == 0 || cells > 25'000'000 || map.data.size() < cells || resolution_ <= 0.0) {
      have_map_ = false;
      return;
    }
    // Distance to the nearest wall: two-pass chamfer transform (in cells)
    const float inf = std::numeric_limits<float>::max() / 4;
    std::vector<float> d(cells, inf);
    free_.assign(cells, 0);
    for (size_t i = 0; i < cells; ++i) {
      int8_t v = map.data[i];
      if (v >= 65) {
        d[i] = 0.0f;
      }
      free_[i] = v >= 0 && v < 25;
    }
    const float diag = 1.41421356f;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        float & c = d[y * width_ + x];
        if (x > 0) {c = std::min(c, d[y * width_ + x - 1] + 1.0f);}
        if (y > 0) {
          c = std::min(c, d[(y - 1) * width_ + x] + 1.0f);
          if (x > 0) {c = std::min(c, d[(y - 1) * width_ + x - 1] + diag);}
          if (x + 1 < width_) {c = std::min(c, d[(y - 1) * width_ + x + 1] + diag);}
        }
      }
    }
    for (int y = height_ - 1; y >= 0; --y) {
      for (int x = width_ - 1; x >= 0; --x) {
        float & c = d[y * width_ + x];
        if (x + 1 < width_) {c = std::min(c, d[y * width_ + x + 1] + 1.0f);}
        if (y + 1 < height_) {
          c = std::min(c, d[(y + 1) * width_ + x] + 1.0f);
          if (x + 1 < width_) {c = std::min(c, d[(y + 1) * width_ + x + 1] + diag);}
          if (x > 0) {c = std::min(c, d[(y + 1) * width_ + x - 1] + diag);}
        }
      }
    }
    // Wall distance in metres, and a smooth likelihood of "this point is on a wall" for the search
    distance_.resize(cells);
    likelihood_.resize(cells);
    const double sigma = 0.1;
    for (size_t i = 0; i < cells; ++i) {
      double m = d[i] >= inf ? 1e3 : d[i] * resolution_;
      distance_[i] = static_cast<float>(m);
      likelihood_[i] = static_cast<float>(std::exp(-m * m / (2.0 * sigma * sigma)));
    }
    have_map_ = true;
  }

  // Map frame -> cell index, or -1 outside the map
  int cell(double wx, double wy) const
  {
    double dx = wx - origin_x_, dy = wy - origin_y_;
    double c = std::cos(-origin_yaw_), s = std::sin(-origin_yaw_);
    return grid_cell(c * dx - s * dy, s * dx + c * dy);
  }

  int grid_cell(double gx, double gy) const
  {
    int i = static_cast<int>(std::floor(gx / resolution_));
    int j = static_cast<int>(std::floor(gy / resolution_));
    return (i < 0 || j < 0 || i >= width_ || j >= height_) ? -1 : j * width_ + i;
  }

  // ---- state ----

  bool fresh_scan() const
  {
    return scan_ && (now() - rclcpp::Time(scan_->header.stamp, get_clock()->get_clock_type())).seconds() < 1.0;
  }

  void update()
  {
    LocalizationState msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.active = have_map_;
    msg.std_xy = static_cast<float>(std_xy_);
    msg.std_yaw = static_cast<float>(std_yaw_);
    if (!have_map_) {
      msg.message = "No map";
      localized_ = false;
      state_pub_->publish(msg);
      return;
    }
    geometry_msgs::msg::TransformStamped base, laser;
    try {
      base = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      msg.message = "No pose on the map yet";
      localized_ = false;
      state_pub_->publish(msg);
      return;
    }
    msg.x = static_cast<float>(base.transform.translation.x);
    msg.y = static_cast<float>(base.transform.translation.y);
    msg.yaw = static_cast<float>(yaw_of(base.transform.rotation));
    double match = 0.0;
    if (!fresh_scan()) {
      msg.message = "No lidar data";
      localized_ = false;
    } else {
      try {
        laser = tf_buffer_->lookupTransform(map_frame_, scan_->header.frame_id, tf2::TimePointZero);
        match = scan_match(*scan_, laser);
        if (match >= match_good_) {
          localized_ = true;
        } else if (match < match_lost_) {
          localized_ = false;
        }
        msg.message = localized_ ? "Localized" :
          (match < 0.2 ? "Lost: set the car's pose" : "Not sure: check the car's pose");
      } catch (const tf2::TransformException &) {
        msg.message = "No lidar position";
        localized_ = false;
      }
    }
    msg.match = static_cast<float>(match);
    msg.localized = localized_;
    state_pub_->publish(msg);
  }

  // Share of lidar points within tolerance of a wall
  double scan_match(const sensor_msgs::msg::LaserScan & scan, const geometry_msgs::msg::TransformStamped & laser)
  {
    double lx = laser.transform.translation.x, ly = laser.transform.translation.y;
    double lyaw = yaw_of(laser.transform.rotation);
    int total = 0, hits = 0;
    double max_range = std::min(static_cast<double>(scan.range_max), max_range_);
    for (size_t k = 0; k < scan.ranges.size(); ++k) {
      double r = scan.ranges[k];
      if (!std::isfinite(r) || r <= scan.range_min || r >= max_range) {
        continue;
      }
      double a = lyaw + scan.angle_min + k * scan.angle_increment;
      int c = cell(lx + r * std::cos(a), ly + r * std::sin(a));
      if (c < 0) {
        continue;
      }
      total++;
      if (distance_[c] <= tolerance_) {
        hits++;
      }
    }
    return total > 20 ? static_cast<double>(hits) / total : 0.0;
  }

  // ---- setting AMCL's pose ----

  void set_pose(double x, double y, double yaw, double std_xy, double std_yaw)
  {
    PoseWithCovariance pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = now();
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.pose.orientation.w = std::cos(yaw / 2.0);
    pose.pose.covariance[0] = pose.pose.covariance[7] = std_xy * std_xy;
    pose.pose.covariance[35] = std_yaw * std_yaw;
    initial_pub_->publish(pose);
    localized_ = false;  // until the scan agrees
  }

  bool place_at_start(std::string & message)
  {
    if (path_.poses.size() < 2) {
      message = "No path selected: start a manual session with a map and a path";
      return false;
    }
    const auto & p0 = path_.poses.front().pose.position;
    size_t k = 1;
    while (k + 1 < path_.poses.size() &&
      std::hypot(path_.poses[k].pose.position.x - p0.x, path_.poses[k].pose.position.y - p0.y) < 0.3)
    {
      ++k;
    }
    const auto & pk = path_.poses[k].pose.position;
    set_pose(p0.x, p0.y, std::atan2(pk.y - p0.y, pk.x - p0.x), 0.2, 0.15);
    message = "Pose set on the start line, facing along the path";
    return true;
  }

  // Mean likelihood of the scan points for a laser pose in the grid frame
  double fit(const std::vector<std::pair<double, double>> & points, double gx, double gy, double yaw) const
  {
    double c = std::cos(yaw), s = std::sin(yaw), sum = 0.0;
    for (const auto & [px, py] : points) {
      int idx = grid_cell(gx + c * px - s * py, gy + s * px + c * py);
      if (idx >= 0) {
        sum += likelihood_[idx];
      }
    }
    return sum / points.size();
  }

  bool locate(std::string & message)
  {
    if (!have_map_) {
      message = "No map loaded";
      return false;
    }
    if (!fresh_scan()) {
      message = "No lidar data";
      return false;
    }
    auto scan = scan_;
    geometry_msgs::msg::TransformStamped mount;
    try {
      mount = tf_buffer_->lookupTransform(base_frame_, scan->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      message = std::string("Where is the lidar on the car? ") + e.what();
      return false;
    }
    // Scan points in the lidar frame, about 90 of them
    std::vector<std::pair<double, double>> all, points;
    double max_range = std::min(static_cast<double>(scan->range_max), max_range_);
    for (size_t k = 0; k < scan->ranges.size(); ++k) {
      double r = scan->ranges[k];
      if (std::isfinite(r) && r > scan->range_min && r < max_range) {
        double a = scan->angle_min + k * scan->angle_increment;
        all.emplace_back(r * std::cos(a), r * std::sin(a));
      }
    }
    if (all.size() < 30) {
      message = "Too few lidar points to search with";
      return false;
    }
    size_t step = std::max<size_t>(1, all.size() / 90);
    for (size_t k = 0; k < all.size(); k += step) {
      points.push_back(all[k]);
    }

    // 1. Coarse: free cells every ~0.2 m, 72 headings; keep each spot's best heading
    const int headings = 72;
    int stride = std::max(1, static_cast<int>(std::lround(0.2 / resolution_)));
    std::vector<std::vector<std::pair<double, double>>> rotated(headings);
    for (int h = 0; h < headings; ++h) {
      double yaw = 2.0 * M_PI * h / headings, c = std::cos(yaw), s = std::sin(yaw);
      for (const auto & [px, py] : points) {
        rotated[h].emplace_back(c * px - s * py, s * px + c * py);
      }
    }
    std::vector<Candidate> coarse;
    for (int j = 0; j < height_; j += stride) {
      for (int i = 0; i < width_; i += stride) {
        if (!free_[j * width_ + i]) {
          continue;
        }
        double gx = (i + 0.5) * resolution_, gy = (j + 0.5) * resolution_;
        Candidate best{-1.0, gx, gy, 0.0};
        for (int h = 0; h < headings; ++h) {
          double sum = 0.0;
          for (const auto & [rx, ry] : rotated[h]) {
            int idx = grid_cell(gx + rx, gy + ry);
            if (idx >= 0) {
              sum += likelihood_[idx];
            }
          }
          double score = sum / points.size();
          if (score > best.score) {
            best = {score, gx, gy, 2.0 * M_PI * h / headings};
          }
        }
        coarse.push_back(best);
      }
    }
    if (coarse.empty()) {
      message = "The map has no free space to search";
      return false;
    }
    std::sort(coarse.begin(), coarse.end(), [](const Candidate & a, const Candidate & b) {return a.score > b.score;});

    // 2. Refine the 12 best distinct spots
    std::vector<Candidate> refined;
    for (const auto & c : coarse) {
      if (refined.size() >= 12) {
        break;
      }
      bool near = std::any_of(refined.begin(), refined.end(), [&c](const Candidate & r) {
          return std::hypot(r.x - c.x, r.y - c.y) < 0.5;
        });
      if (near) {
        continue;
      }
      Candidate best = c;
      for (double scale : {1.0, 0.25}) {
        Candidate center = best;
        for (int dx = -5; dx <= 5; ++dx) {
          for (int dy = -5; dy <= 5; ++dy) {
            for (int da = -6; da <= 6; ++da) {
              double x = center.x + dx * 0.04 * scale, y = center.y + dy * 0.04 * scale;
              double yaw = center.yaw + da * (M_PI / 180.0) * scale;
              double score = fit(points, x, y, yaw);
              if (score > best.score) {
                best = {score, x, y, yaw};
              }
            }
          }
        }
      }
      refined.push_back(best);
    }
    std::sort(refined.begin(), refined.end(), [](const Candidate & a, const Candidate & b) {return a.score > b.score;});
    const Candidate & best = refined.front();
    if (best.score < locate_min_fit_) {
      message = "Couldn't find the car on this map (best fit " + std::to_string(static_cast<int>(best.score * 100)) +
        "%). Is it the right map? Set the pose by hand.";
      return false;
    }
    for (size_t k = 1; k < refined.size(); ++k) {
      double apart = std::hypot(refined[k].x - best.x, refined[k].y - best.y);
      if (apart > 1.0 && refined[k].score > best.score - 0.04) {
        message = "Several places on the map fit equally well: put the car on the start line or set the pose by hand";
        return false;
      }
    }
    // Lidar pose in the grid frame -> car pose in the map frame
    double c = std::cos(origin_yaw_), s = std::sin(origin_yaw_);
    double lx = origin_x_ + c * best.x - s * best.y, ly = origin_y_ + s * best.x + c * best.y;
    double laser_yaw = best.yaw + origin_yaw_;
    double mx = mount.transform.translation.x, my = mount.transform.translation.y;
    double yaw = laser_yaw - yaw_of(mount.transform.rotation);
    double x = lx - (std::cos(yaw) * mx - std::sin(yaw) * my);
    double y = ly - (std::sin(yaw) * mx + std::cos(yaw) * my);
    set_pose(x, y, std::atan2(std::sin(yaw), std::cos(yaw)), 0.1, 0.05);
    message = "Found the car (fit " + std::to_string(static_cast<int>(best.score * 100)) + "%)";
    return true;
  }

  std::string map_frame_, base_frame_;
  double tolerance_, match_good_, match_lost_, max_range_, locate_min_fit_;
  bool have_map_ = false, localized_ = false;
  int width_ = 0, height_ = 0;
  double resolution_ = 0.05, origin_x_ = 0.0, origin_y_ = 0.0, origin_yaw_ = 0.0;
  std::vector<float> distance_, likelihood_;
  std::vector<uint8_t> free_;
  nav_msgs::msg::Path path_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  double std_xy_ = 0.0, std_yaw_ = 0.0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<PoseWithCovariance>::SharedPtr amcl_sub_;
  rclcpp::Publisher<PoseWithCovariance>::SharedPtr initial_pub_;
  rclcpp::Publisher<LocalizationState>::SharedPtr state_pub_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_, locate_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationMonitor>());
  rclcpp::shutdown();
  return 0;
}
