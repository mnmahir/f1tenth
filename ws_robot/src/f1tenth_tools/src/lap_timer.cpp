// Times laps on the selected path (/f1tenth/selected_path): a lap ends when the car's progress along the
// path wraps from the end back to the start. The first crossing starts the first timed lap (out lap before).
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "f1tenth_bringup/msg/lap_state.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LapTimer : public rclcpp::Node
{
public:
  LapTimer() : Node("lap_timer")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    min_lap_time_ = declare_parameter<double>("min_lap_time", 3.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    rclcpp::QoS latched(1);
    latched.transient_local();
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/f1tenth/selected_path", latched, [this](nav_msgs::msg::Path::SharedPtr msg) {set_path(*msg);});
    pub_ = create_publisher<f1tenth_bringup::msg::LapState>("/f1tenth/lap", 10);
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset", [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        lap_ = 0;
        last_ = best_ = 0.0;
        timing_ = false;
        res->success = true;
        res->message = "Lap times reset";
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {update();});
  }

private:
  void set_path(const nav_msgs::msg::Path & path)
  {
    xs_.clear();
    ys_.clear();
    cumulative_.clear();
    double s = 0.0;
    for (size_t i = 0; i < path.poses.size(); ++i) {
      double x = path.poses[i].pose.position.x, y = path.poses[i].pose.position.y;
      if (i > 0) {
        s += std::hypot(x - xs_.back(), y - ys_.back());
      }
      xs_.push_back(x);
      ys_.push_back(y);
      cumulative_.push_back(s);
    }
    total_ = s;
    nearest_ = -1;
    lap_ = 0;
    last_ = best_ = 0.0;
    timing_ = false;
  }

  void update()
  {
    f1tenth_bringup::msg::LapState state;
    state.header.stamp = now();
    geometry_msgs::msg::TransformStamped tf;
    bool localized = false;
    if (xs_.size() >= 2 && total_ > 0.0) {
      try {
        tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
        localized = true;
      } catch (const tf2::TransformException &) {
      }
    }
    if (localized) {
      double x = tf.transform.translation.x, y = tf.transform.translation.y;
      // Search near the last match, or everywhere when lost
      int n = static_cast<int>(xs_.size());
      int lo = 0, hi = n;
      if (nearest_ >= 0) {
        lo = nearest_ - 50;
        hi = nearest_ + 50;
      }
      double best_d = std::numeric_limits<double>::max();
      int best_i = 0;
      for (int k = lo; k < hi; ++k) {
        int i = ((k % n) + n) % n;
        double d = std::hypot(x - xs_[i], y - ys_[i]);
        if (d < best_d) {
          best_d = d;
          best_i = i;
        }
      }
      if (best_d > 2.0 && nearest_ >= 0) {
        nearest_ = -1;  // lost: full search next time
      } else {
        double progress = cumulative_[best_i] / total_;
        auto t = now();
        if (nearest_ >= 0 && progress_ > 0.9 && progress < 0.1) {
          if (timing_ && (t - lap_start_).seconds() > min_lap_time_) {
            last_ = (t - lap_start_).seconds();
            best_ = best_ > 0.0 ? std::min(best_, last_) : last_;
            ++lap_;
            RCLCPP_INFO(get_logger(), "Lap %u: %.3f s (best %.3f s)", lap_, last_, best_);
          }
          timing_ = true;
          lap_start_ = t;
        }
        nearest_ = best_i;
        progress_ = progress;
        state.active = true;
        state.progress = static_cast<float>(progress);
        state.cross_track_error = static_cast<float>(best_d);
        state.current_lap_time = timing_ ? static_cast<float>((t - lap_start_).seconds()) : 0.0f;
      }
    }
    state.lap = lap_;
    state.last_lap_time = static_cast<float>(last_);
    state.best_lap_time = static_cast<float>(best_);
    pub_->publish(state);
  }

  std::string map_frame_, base_frame_;
  double min_lap_time_;
  std::vector<double> xs_, ys_, cumulative_;
  double total_ = 0.0, progress_ = 0.0, last_ = 0.0, best_ = 0.0;
  int nearest_ = -1;
  uint32_t lap_ = 0;
  bool timing_ = false;
  rclcpp::Time lap_start_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<f1tenth_bringup::msg::LapState>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LapTimer>());
  rclcpp::shutdown();
  return 0;
}
