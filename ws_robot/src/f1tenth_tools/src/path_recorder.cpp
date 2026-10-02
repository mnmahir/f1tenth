// Records the driven path in the map frame (from TF) while driving, then saves it as a raceline CSV
// (x,y,speed) in paths_dir. Speeds are what the car actually did, or a constant.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "f1tenth_bringup/msg/recorder_state.hpp"
#include "f1tenth_bringup/srv/save_file.hpp"
#include "f1tenth_tools/data_io.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace ft = f1tenth_tools;
using Trigger = std_srvs::srv::Trigger;

class PathRecorder : public rclcpp::Node
{
public:
  PathRecorder() : Node("path_recorder")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    spacing_ = declare_parameter<double>("spacing", 0.10);
    // Read when saving, so the Pit Wall can set them just before
    declare_parameter<std::string>("speed_mode", "recorded");  // recorded | constant
    declare_parameter<double>("constant_speed", 2.0);
    declare_parameter<double>("min_speed", 0.5);
    paths_dir_ = ft::expand_user(declare_parameter<std::string>("paths_dir", "~/f1tenth/data/paths"));
    map_name_ = declare_parameter<std::string>("map_name", "");  // the session's map, saved with each path
    double rate = declare_parameter<double>("rate", 20.0);
    param_cb_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name() == "speed_mode" && (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
          (p.as_string() != "recorded" && p.as_string() != "constant")))
        {
          result.successful = false;
          result.reason = "speed_mode must be recorded or constant";
        } else if ((p.get_name() == "constant_speed" || p.get_name() == "min_speed") &&
          (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || p.as_double() <= 0.0))
        {
          result.successful = false;
          result.reason = p.get_name() + " must be above 0 m/s";
        }
      }
      return result;
    });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/wheel", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {speed_ = msg->twist.twist.linear.x;});

    rclcpp::QoS latched(1);
    latched.transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/f1tenth/recorded_path", latched);
    state_pub_ = create_publisher<f1tenth_bringup::msg::RecorderState>("/f1tenth/recorder/state", 10);

    start_srv_ = create_service<Trigger>("~/start", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      recording_ = true;
      reply(res, true, points_.empty() ? "Recording" : "Recording resumed");
    });
    stop_srv_ = create_service<Trigger>("~/stop", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      recording_ = false;
      reply(res, true, "Recording paused");
    });
    clear_srv_ = create_service<Trigger>("~/clear", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      points_.clear();
      length_ = 0.0;
      publish_path();
      reply(res, true, "Recording cleared");
    });
    save_srv_ = create_service<f1tenth_bringup::srv::SaveFile>(
      "~/save", [this](f1tenth_bringup::srv::SaveFile::Request::SharedPtr req, f1tenth_bringup::srv::SaveFile::Response::SharedPtr res) {
        res->success = save(req->name, res->message);
      });

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {sample();});
    status_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {publish_state();});
  }

private:
  void reply(Trigger::Response::SharedPtr res, bool ok, const std::string & message)
  {
    message_ = message;
    res->success = ok;
    res->message = message;
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
  }

  void sample()
  {
    if (!recording_) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No pose yet: %s", e.what());
      return;
    }
    double x = tf.transform.translation.x, y = tf.transform.translation.y;
    if (!points_.empty()) {
      double d = std::hypot(x - points_.back().x, y - points_.back().y);
      if (d < spacing_) {
        return;
      }
      length_ += d;
    }
    points_.push_back({x, y, std::abs(speed_)});
    max_speed_ = std::max(max_speed_, std::abs(speed_));
    if (points_.size() % 5 == 0) {
      publish_path();
    }
  }

  void publish_path()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    for (const auto & p : points_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  void publish_state()
  {
    f1tenth_bringup::msg::RecorderState state;
    state.header.stamp = now();
    state.recording = recording_;
    state.points = static_cast<uint32_t>(points_.size());
    state.length = static_cast<float>(length_);
    state.max_speed = static_cast<float>(max_speed_);
    state.message = message_;
    state_pub_->publish(state);
  }

  bool save(const std::string & name, std::string & message)
  {
    if (!ft::valid_name(name)) {
      message = "Use letters, digits, '-' or '_' for the name";
      return false;
    }
    if (points_.size() < 10) {
      message = "Record a longer path first";
      return false;
    }
    std::error_code ec;
    ft::fs::create_directories(paths_dir_, ec);
    std::string file = paths_dir_ + "/" + name + ".csv";
    if (ft::fs::exists(file)) {
      message = "A path named '" + name + "' already exists";
      return false;
    }
    bool constant = get_parameter("speed_mode").as_string() == "constant";
    double constant_speed = get_parameter("constant_speed").as_double();
    double min_speed = get_parameter("min_speed").as_double();
    auto points = points_;
    for (auto & p : points) {
      p.v = constant ? constant_speed : std::max(p.v, min_speed);
    }
    if (!ft::save_path_csv(file, points, message)) {
      return false;
    }
    ft::save_path_info(file, {map_name_, "recorded", "", ""});
    char speeds[48];
    std::snprintf(speeds, sizeof(speeds), constant ? "%.1f m/s throughout" : "speeds as driven", constant_speed);
    message = "Saved lap '" + name + "' (" + std::to_string(points.size()) + " points, " + speeds +
      (map_name_.empty() ? "" : ", on " + map_name_) + ")";
    message_ = message;
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  std::string map_frame_, base_frame_, paths_dir_, map_name_, message_;
  double spacing_;
  bool recording_ = false;
  double speed_ = 0.0, length_ = 0.0, max_speed_ = 0.0;
  std::vector<ft::PathPoint> points_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<f1tenth_bringup::msg::RecorderState>::SharedPtr state_pub_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_, stop_srv_, clear_srv_;
  rclcpp::Service<f1tenth_bringup::srv::SaveFile>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathRecorder>());
  rclcpp::shutdown();
  return 0;
}
