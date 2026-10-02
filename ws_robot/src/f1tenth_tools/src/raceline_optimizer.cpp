// Raceline optimizer: turns a driven path into a minimum-curvature racing line inside the track walls of a map,
// with a speed profile from grip and acceleration limits.
//  1. Resample and smooth the source path (the reference line)
//  2. Measure the free width left and right of each point in the occupancy map
//  3. Shift each point sideways (along its normal) to minimise the summed squared curvature (second differences),
//     keeping the car inside the walls minus half its width and a margin: a box-constrained QP solved with FISTA
//  4. Speed: lateral-grip limit from curvature, then forward (acceleration) and backward (braking) passes
// ~/optimize previews the result on /f1tenth/raceline/*, ~/save writes it as a path CSV (x,y,speed).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "f1tenth_bringup/srv/optimize_raceline.hpp"
#include "f1tenth_bringup/srv/save_file.hpp"
#include "f1tenth_tools/data_io.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ft = f1tenth_tools;
using OptimizeRaceline = f1tenth_bringup::srv::OptimizeRaceline;

namespace
{
struct Vec2
{
  double x, y;
};

// Uniform resampling by arc length; closed lines include the segment back to the start
std::vector<Vec2> resample(const std::vector<Vec2> & in, double ds, bool closed)
{
  std::vector<Vec2> pts = in;
  if (closed) {
    pts.push_back(in.front());
  }
  std::vector<double> s(pts.size(), 0.0);
  for (size_t i = 1; i < pts.size(); ++i) {
    s[i] = s[i - 1] + std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  }
  double total = s.back();
  int n = std::max(2, static_cast<int>(std::round(total / ds)));
  double step = total / n;
  std::vector<Vec2> out;
  size_t j = 1;
  for (int k = 0; k < (closed ? n : n + 1); ++k) {
    double target = std::min(k * step, total);
    while (j < s.size() - 1 && s[j] < target) {
      ++j;
    }
    double seg = s[j] - s[j - 1];
    double t = seg > 1e-9 ? (target - s[j - 1]) / seg : 0.0;
    out.push_back({pts[j - 1].x + t * (pts[j].x - pts[j - 1].x), pts[j - 1].y + t * (pts[j].y - pts[j - 1].y)});
  }
  return out;
}

// Moving average (endpoints of open lines stay put)
std::vector<Vec2> smooth(const std::vector<Vec2> & in, int half_window, bool closed)
{
  int n = static_cast<int>(in.size());
  std::vector<Vec2> out = in;
  for (int i = 0; i < n; ++i) {
    if (!closed && (i < half_window || i >= n - half_window)) {
      continue;
    }
    double sx = 0, sy = 0;
    for (int k = -half_window; k <= half_window; ++k) {
      const Vec2 & p = in[((i + k) % n + n) % n];
      sx += p.x;
      sy += p.y;
    }
    out[i] = {sx / (2 * half_window + 1), sy / (2 * half_window + 1)};
  }
  return out;
}

std::vector<Vec2> left_normals(const std::vector<Vec2> & p, bool closed)
{
  int n = static_cast<int>(p.size());
  std::vector<Vec2> normals(n);
  for (int i = 0; i < n; ++i) {
    int a = closed ? (i - 1 + n) % n : std::max(i - 1, 0);
    int b = closed ? (i + 1) % n : std::min(i + 1, n - 1);
    double tx = p[b].x - p[a].x, ty = p[b].y - p[a].y, len = std::hypot(tx, ty);
    normals[i] = len > 1e-9 ? Vec2{-ty / len, tx / len} : Vec2{0, 0};
  }
  return normals;
}

// Distance from p along dir to the first occupied or unknown cell
double free_distance(const ft::OccupancyMap & map, Vec2 p, Vec2 dir, double max_dist)
{
  double step = map.resolution * 0.5;
  for (double d = 0.0; d <= max_dist; d += step) {
    if (map.at_world(p.x + dir.x * d, p.y + dir.y * d) != 0) {
      return d;
    }
  }
  return max_dist;
}

std::vector<double> signed_curvature(const std::vector<Vec2> & r, bool closed)
{
  int n = static_cast<int>(r.size());
  std::vector<double> k(n, 0.0);
  for (int i = 0; i < n; ++i) {
    if (!closed && (i == 0 || i == n - 1)) {
      continue;
    }
    const Vec2 & a = r[(i - 1 + n) % n];
    const Vec2 & b = r[i];
    const Vec2 & c = r[(i + 1) % n];
    double cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    double d = std::hypot(b.x - a.x, b.y - a.y) * std::hypot(c.x - b.x, c.y - b.y) * std::hypot(c.x - a.x, c.y - a.y);
    k[i] = d > 1e-12 ? 2.0 * cross / d : 0.0;
  }
  if (!closed && n > 2) {
    k[0] = k[1];
    k[n - 1] = k[n - 2];
  }
  return k;
}
}  // namespace

class RacelineOptimizer : public rclcpp::Node
{
public:
  RacelineOptimizer() : Node("raceline_optimizer")
  {
    maps_dir_ = ft::expand_user(declare_parameter<std::string>("maps_dir", "~/f1tenth/data/maps"));
    paths_dir_ = ft::expand_user(declare_parameter<std::string>("paths_dir", "~/f1tenth/data/paths"));
    vehicle_width_ = declare_parameter<double>("vehicle_width", 0.30);
    opt_spacing_ = declare_parameter<double>("optimization_spacing", 0.20);
    out_spacing_ = declare_parameter<double>("output_spacing", 0.10);
    smoothing_ = declare_parameter<double>("smoothing", 0.6);  // m, moving-average window on the source path
    iterations_ = declare_parameter<int>("iterations", 6000);
    max_track_width_ = declare_parameter<double>("max_track_width", 6.0);
    // Defaults used when a request leaves a limit at 0
    margin_ = declare_parameter<double>("margin", 0.10);
    max_speed_ = declare_parameter<double>("max_speed", 6.0);
    max_lat_ = declare_parameter<double>("max_lateral_accel", 5.0);
    max_acc_ = declare_parameter<double>("max_accel", 3.0);
    max_dec_ = declare_parameter<double>("max_decel", 5.0);

    rclcpp::QoS latched(1);
    latched.transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/f1tenth/raceline/path", latched);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/f1tenth/raceline/markers", latched);

    optimize_srv_ = create_service<OptimizeRaceline>(
      "~/optimize", [this](OptimizeRaceline::Request::SharedPtr req, OptimizeRaceline::Response::SharedPtr res) {
        auto start = std::chrono::steady_clock::now();
        res->success = optimize(*req, *res);
        if (res->success) {
          double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
          RCLCPP_INFO(get_logger(), "%s (%.0f ms)", res->message.c_str(), ms);
        } else {
          RCLCPP_WARN(get_logger(), "Raceline: %s", res->message.c_str());
        }
      });
    save_srv_ = create_service<f1tenth_bringup::srv::SaveFile>(
      "~/save", [this](f1tenth_bringup::srv::SaveFile::Request::SharedPtr req, f1tenth_bringup::srv::SaveFile::Response::SharedPtr res) {
        res->success = save(req->name, res->message);
      });
  }

private:
  bool optimize(const OptimizeRaceline::Request & req, OptimizeRaceline::Response & res)
  {
    double margin = req.margin > 0 ? req.margin : margin_;
    double v_max = req.max_speed > 0 ? req.max_speed : max_speed_;
    double a_lat = req.max_lateral_accel > 0 ? req.max_lateral_accel : max_lat_;
    double a_acc = req.max_accel > 0 ? req.max_accel : max_acc_;
    double a_dec = req.max_decel > 0 ? req.max_decel : max_dec_;

    std::string error;
    std::string path_file = ft::find_path_file(req.path, paths_dir_);
    std::vector<ft::PathPoint> source;
    if (path_file.empty() || !ft::load_path_csv(path_file, source, error)) {
      res.message = path_file.empty() ? "path '" + req.path + "' not found" : error;
      return false;
    }
    ft::OccupancyMap map;
    if (!ft::load_map(maps_dir_ + "/" + req.map + ".yaml", map, error)) {
      res.message = error;
      return false;
    }

    // 1. Reference line
    std::vector<Vec2> pts;
    for (const auto & p : source) {
      if (pts.empty() || std::hypot(p.x - pts.back().x, p.y - pts.back().y) > 1e-3) {
        pts.push_back({p.x, p.y});
      }
    }
    double length = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
      length += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    }
    if (length < 2.0) {
      res.message = "path is too short to optimize";
      return false;
    }
    closed_ = std::hypot(pts.front().x - pts.back().x, pts.front().y - pts.back().y) < std::max(1.0, 0.05 * length);
    if (closed_ && std::hypot(pts.front().x - pts.back().x, pts.front().y - pts.back().y) < 1e-3) {
      pts.pop_back();
    }
    auto ref = resample(pts, opt_spacing_, closed_);
    int half_window = std::max(1, static_cast<int>(std::round(smoothing_ / (2 * opt_spacing_))));
    for (int pass = 0; pass < 3; ++pass) {
      ref = smooth(ref, half_window, closed_);
    }
    ref = resample(ref, opt_spacing_, closed_);
    int n = static_cast<int>(ref.size());
    auto normals = left_normals(ref, closed_);

    // 2. Track width and the corridor the car's centre may use
    double half_width = vehicle_width_ / 2.0;
    std::vector<double> lower(n), upper(n), wall_left(n), wall_right(n);
    double min_width = 1e9;
    int narrow = 0;
    for (int i = 0; i < n; ++i) {
      wall_left[i] = free_distance(map, ref[i], normals[i], max_track_width_);
      wall_right[i] = free_distance(map, ref[i], {-normals[i].x, -normals[i].y}, max_track_width_);
      min_width = std::min(min_width, wall_left[i] + wall_right[i]);
      upper[i] = wall_left[i] - half_width - margin;
      lower[i] = -(wall_right[i] - half_width - margin);
      if (lower[i] > upper[i]) {  // narrower than the car plus margins: stay in the middle of the gap
        upper[i] = lower[i] = (wall_left[i] - wall_right[i]) / 2.0;
        ++narrow;
      }
    }
    if (!closed_) {  // open lines keep their ends
      lower[0] = upper[0] = lower[n - 1] = upper[n - 1] = 0.0;
    }

    // 3. Minimum curvature: minimise sum |r[i-1] - 2 r[i] + r[i+1]|^2, r[i] = ref[i] + alpha[i] * normal[i]
    std::vector<double> alpha(n, 0.0), y(n, 0.0), next(n), grad(n);
    std::vector<Vec2> e(n);
    auto residuals = [&](const std::vector<double> & a) {
      for (int i = 0; i < n; ++i) {
        if (!closed_ && (i == 0 || i == n - 1)) {
          e[i] = {0, 0};
          continue;
        }
        int im = (i - 1 + n) % n, ip = (i + 1) % n;
        e[i] = {
          ref[im].x + a[im] * normals[im].x - 2 * (ref[i].x + a[i] * normals[i].x) + ref[ip].x + a[ip] * normals[ip].x,
          ref[im].y + a[im] * normals[im].y - 2 * (ref[i].y + a[i] * normals[i].y) + ref[ip].y + a[ip] * normals[ip].y};
      }
    };
    const double step = 1.0 / 32.0;  // 1/L: the Hessian's largest eigenvalue is at most 2 * 16
    double t = 1.0;
    for (int it = 0; it < iterations_; ++it) {
      residuals(y);
      for (int k = 0; k < n; ++k) {
        const Vec2 & em = e[(k - 1 + n) % n];
        const Vec2 & ep = e[(k + 1) % n];
        double gx = (closed_ || k > 0 ? em.x : 0.0) - 2 * e[k].x + (closed_ || k < n - 1 ? ep.x : 0.0);
        double gy = (closed_ || k > 0 ? em.y : 0.0) - 2 * e[k].y + (closed_ || k < n - 1 ? ep.y : 0.0);
        grad[k] = 2.0 * (normals[k].x * gx + normals[k].y * gy);
      }
      double t_next = (1.0 + std::sqrt(1.0 + 4.0 * t * t)) / 2.0;
      for (int k = 0; k < n; ++k) {
        next[k] = std::clamp(y[k] - step * grad[k], lower[k], upper[k]);
        y[k] = next[k] + ((t - 1.0) / t_next) * (next[k] - alpha[k]);
      }
      alpha.swap(next);
      t = t_next;
    }
    std::vector<Vec2> line(n);
    for (int i = 0; i < n; ++i) {
      line[i] = {ref[i].x + alpha[i] * normals[i].x, ref[i].y + alpha[i] * normals[i].y};
    }

    // 4. Speed profile on the output spacing
    raceline_ = resample(line, out_spacing_, closed_);
    int m = static_cast<int>(raceline_.size());
    auto curvature = signed_curvature(raceline_, closed_);
    std::vector<double> ds(m, 0.0), v(m);
    for (int i = 0; i < m; ++i) {
      int ip = (i + 1) % m;
      ds[i] = std::hypot(raceline_[ip].x - raceline_[i].x, raceline_[ip].y - raceline_[i].y);
    }
    for (int i = 0; i < m; ++i) {
      double k = (std::abs(curvature[(i - 1 + m) % m]) + 2 * std::abs(curvature[i]) + std::abs(curvature[(i + 1) % m])) / 4.0;
      v[i] = std::min(v_max, std::sqrt(a_lat / std::max(k, 1e-6)));
    }
    int rounds = closed_ ? 2 : 1;
    for (int r = 0; r < rounds; ++r) {
      for (int i = 1; i <= (closed_ ? m : m - 1); ++i) {
        int cur = i % m, prev = i - 1;
        v[cur] = std::min(v[cur], std::sqrt(v[prev] * v[prev] + 2.0 * a_acc * ds[prev]));
      }
    }
    for (int r = 0; r < rounds; ++r) {
      for (int i = (closed_ ? m : m - 1) - 1; i >= 0; --i) {
        int nxt = (i + 1) % m;
        v[i] = std::min(v[i], std::sqrt(v[nxt] * v[nxt] + 2.0 * a_dec * ds[i]));
      }
    }
    speeds_ = v;

    double total = 0.0, lap_time = 0.0;
    for (int i = 0; i < (closed_ ? m : m - 1); ++i) {
      int ip = (i + 1) % m;
      total += ds[i];
      lap_time += ds[i] / std::max(0.05, (v[i] + v[ip]) / 2.0);
    }
    publish(ref, normals, wall_left, wall_right);

    res.length = static_cast<float>(total);
    res.lap_time = static_cast<float>(lap_time);
    res.min_width = static_cast<float>(min_width);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s raceline: %.1f m, est. %s %.2f s, %.1f-%.1f m/s%s",
      closed_ ? "Closed" : "Open", total, closed_ ? "lap" : "time", lap_time,
      *std::min_element(v.begin(), v.end()), *std::max_element(v.begin(), v.end()),
      narrow ? (", " + std::to_string(narrow) + " points narrower than the car").c_str() : "");
    res.message = buf;
    result_map_ = req.map;
    result_source_ = req.path;
    has_result_ = true;
    return true;
  }

  void publish(const std::vector<Vec2> & ref, const std::vector<Vec2> & normals,
    const std::vector<double> & wall_left, const std::vector<double> & wall_right)
  {
    auto stamp = now();
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = stamp;
    for (const auto & p : raceline_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);

    visualization_msgs::msg::MarkerArray markers;
    auto strip = [&](int id, const std::string & ns, double width) {
      visualization_msgs::msg::Marker mk;
      mk.header = path.header;
      mk.ns = ns;
      mk.id = id;
      mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
      mk.action = visualization_msgs::msg::Marker::ADD;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = width;
      mk.color.a = 1.0;
      return mk;
    };
    // Speed-coloured raceline: blue (slow) to red (fast), like F1 telemetry traces
    auto line = strip(0, "raceline", 0.06);
    double vmin = *std::min_element(speeds_.begin(), speeds_.end());
    double vmax = *std::max_element(speeds_.begin(), speeds_.end());
    for (size_t i = 0; i <= raceline_.size(); ++i) {
      if (i == raceline_.size() && !closed_) {
        break;
      }
      size_t k = i % raceline_.size();
      geometry_msgs::msg::Point pt;
      pt.x = raceline_[k].x;
      pt.y = raceline_[k].y;
      pt.z = 0.06;  // above the UI's driving trail
      line.points.push_back(pt);
      double f = vmax > vmin ? (speeds_[k] - vmin) / (vmax - vmin) : 0.5;
      std_msgs::msg::ColorRGBA c;
      c.r = static_cast<float>(std::min(1.0, 2.0 * f));
      c.g = static_cast<float>(1.0 - std::abs(2.0 * f - 1.0)) * 0.8f;
      c.b = static_cast<float>(std::min(1.0, 2.0 * (1.0 - f)));
      c.a = 1.0f;
      line.colors.push_back(c);
    }
    markers.markers.push_back(line);
    // Track walls as measured from the map
    for (int side = 0; side < 2; ++side) {
      auto wall = strip(1 + side, "walls", 0.03);
      wall.color.r = wall.color.g = wall.color.b = 0.85f;
      wall.color.a = 0.8f;
      for (size_t i = 0; i <= ref.size(); ++i) {
        if (i == ref.size() && !closed_) {
          break;
        }
        size_t k = i % ref.size();
        double d = side == 0 ? wall_left[k] : -wall_right[k];
        geometry_msgs::msg::Point pt;
        pt.x = ref[k].x + normals[k].x * d;
        pt.y = ref[k].y + normals[k].y * d;
        pt.z = 0.05;
        wall.points.push_back(pt);
      }
      markers.markers.push_back(wall);
    }
    markers_pub_->publish(markers);
  }

  bool save(const std::string & name, std::string & message)
  {
    if (!has_result_) {
      message = "Optimize a path first";
      return false;
    }
    if (!ft::valid_name(name)) {
      message = "Use letters, digits, '-' or '_' for the name";
      return false;
    }
    std::error_code ec;
    ft::fs::create_directories(paths_dir_, ec);
    std::string file = paths_dir_ + "/" + name + ".csv";
    if (ft::fs::exists(file)) {
      message = "A path named '" + name + "' already exists";
      return false;
    }
    std::vector<ft::PathPoint> points;
    for (size_t i = 0; i < raceline_.size(); ++i) {
      points.push_back({raceline_[i].x, raceline_[i].y, speeds_[i]});
    }
    if (!ft::save_path_csv(file, points, message)) {
      return false;
    }
    ft::save_path_info(file, {result_map_, "raceline", result_source_, ""});
    message = "Saved raceline '" + name + "' (" + std::to_string(points.size()) + " points, on " + result_map_ + ")";
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  std::string maps_dir_, paths_dir_;
  double vehicle_width_, opt_spacing_, out_spacing_, smoothing_, max_track_width_;
  double margin_, max_speed_, max_lat_, max_acc_, max_dec_;
  int iterations_;
  bool closed_ = false, has_result_ = false;
  std::string result_map_, result_source_;
  std::vector<Vec2> raceline_;
  std::vector<double> speeds_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Service<OptimizeRaceline>::SharedPtr optimize_srv_;
  rclcpp::Service<f1tenth_bringup::srv::SaveFile>::SharedPtr save_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacelineOptimizer>());
  rclcpp::shutdown();
  return 0;
}
