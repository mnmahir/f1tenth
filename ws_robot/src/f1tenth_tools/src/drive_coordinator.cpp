// Decides what reaches ackermann_mux:
//  - joystick commands pass through, with the path follower's steering when assist is on (user keeps the speed)
//  - path follower commands go to cmd_auto/drive only while autonomous is engaged
// Autonomous is engaged from the UI (~/set_autonomous) or the joystick, only while the car is localized on the
// map, and drops when the follower goes stale, localization is lost, or the kill switch (teleop lock) or the UI
// E-stop is engaged, so releasing a lock never resumes driving by itself.
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "f1tenth_bringup/msg/drive_state.hpp"
#include "f1tenth_bringup/msg/localization_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

using AckermannStamped = ackermann_msgs::msg::AckermannDriveStamped;
using DriveState = f1tenth_bringup::msg::DriveState;

class DriveCoordinator : public rclcpp::Node
{
public:
  DriveCoordinator() : Node("drive_coordinator")
  {
    assist_enabled_ = declare_parameter<bool>("assist_enabled", true);
    follower_timeout_ = declare_parameter<double>("follower_timeout", 0.3);
    engage_button_ = declare_parameter<int>("engage_button", 0);       // A
    disengage_button_ = declare_parameter<int>("disengage_button", 1);  // B
    require_localization_ = declare_parameter<bool>("require_localization", true);
    localization_timeout_ = declare_parameter<double>("localization_timeout", 1.5);  // s lost before disengaging

    teleop_pub_ = create_publisher<AckermannStamped>("cmd_teleop/joy", 10);
    auto_pub_ = create_publisher<AckermannStamped>("cmd_auto/drive", 10);
    state_pub_ = create_publisher<DriveState>("/f1tenth/drive_state", 10);

    teleop_sub_ = create_subscription<AckermannStamped>(
      "cmd_teleop/joy_user", 10, [this](AckermannStamped::SharedPtr msg) {on_teleop(*msg);});
    follower_sub_ = create_subscription<AckermannStamped>(
      "/f1tenth/follower/drive", 10, [this](AckermannStamped::SharedPtr msg) {on_follower(*msg);});
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, [this](sensor_msgs::msg::Joy::SharedPtr msg) {on_joy(*msg);});
    lock_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/mux_bool/teleop_cutoff_and_brake", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && autonomous_) {
          set_autonomous(false, "kill switch engaged");
        }
      });
    localization_sub_ = create_subscription<f1tenth_bringup::msg::LocalizationState>(
      "/f1tenth/localization", 10, [this](f1tenth_bringup::msg::LocalizationState::SharedPtr msg) {
        localization_time_ = now();
        if (msg->localized) {
          localized_time_ = localization_time_;
        }
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/mux_bool/ui_estop", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
        estop_ = msg->data;
        if (estop_ && autonomous_) {
          set_autonomous(false, "E-stop");
        }
      });

    autonomous_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/set_autonomous",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
        res->success = set_autonomous(req->data, "requested by UI");
        res->message = message_;
      });
    assist_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/set_assist",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
        set_parameter(rclcpp::Parameter("assist_enabled", req->data));
        res->success = true;
        res->message = req->data ? "Steering assist on" : "Steering assist off";
      });
    param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
      for (const auto & p : params) {
        if (p.get_name() == "assist_enabled") {assist_enabled_ = p.as_bool();}
        if (p.get_name() == "follower_timeout") {follower_timeout_ = p.as_double();}
        if (p.get_name() == "engage_button") {engage_button_ = static_cast<int>(p.as_int());}
        if (p.get_name() == "disengage_button") {disengage_button_ = static_cast<int>(p.as_int());}
        if (p.get_name() == "require_localization") {require_localization_ = p.as_bool();}
        if (p.get_name() == "localization_timeout") {localization_timeout_ = p.as_double();}
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

    follower_time_ = teleop_time_ = localization_time_ = localized_time_ = message_time_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {publish_state();});
  }

private:
  bool follower_fresh() {return (now() - follower_time_).seconds() < follower_timeout_;}
  // Localized now (fresh report) / within the grace period
  bool localized() {return !require_localization_ || (now() - localized_time_).seconds() < 0.5;}
  bool localization_lost() {return require_localization_ && (now() - localized_time_).seconds() > localization_timeout_;}

  void say(const std::string & message, bool warn = false)
  {
    message_ = message;
    message_time_ = now();
    if (warn) {
      RCLCPP_WARN(get_logger(), "%s", message_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "%s", message_.c_str());
    }
  }

  bool set_autonomous(bool on, const std::string & why)
  {
    if (on && estop_) {
      say("Release the E-stop first", true);
      return false;
    }
    if (on && !localized()) {
      say("Not localized: set the car's pose on the map first (DRIVE > Autonomous)", true);
      return false;
    }
    if (on && !follower_fresh()) {
      say("Autonomous needs a map and a path (path follower not running)", true);
      return false;
    }
    if (on != autonomous_) {
      autonomous_ = on;
      say(std::string(on ? "Autonomous engaged" : "Autonomous disengaged") + " (" + why + ")");
    }
    return true;
  }

  void on_teleop(AckermannStamped msg)
  {
    teleop_time_ = now();
    assisting_ = assist_enabled_ && follower_fresh() && localized();
    if (assisting_) {
      msg.drive.steering_angle = follower_.drive.steering_angle;
    }
    teleop_pub_->publish(msg);
  }

  void on_follower(const AckermannStamped & msg)
  {
    follower_ = msg;
    follower_time_ = now();
    if (autonomous_) {
      auto_pub_->publish(msg);
    }
  }

  void on_joy(const sensor_msgs::msg::Joy & joy)
  {
    auto pressed = [&joy](int button) {
      return button >= 0 && button < static_cast<int>(joy.buttons.size()) && joy.buttons[button] != 0;
    };
    bool engage = pressed(engage_button_), disengage = pressed(disengage_button_);
    if (engage && !engage_was_) {
      set_autonomous(true, "joystick");
    }
    if (disengage && !disengage_was_) {
      set_autonomous(false, "joystick");
    }
    engage_was_ = engage;
    disengage_was_ = disengage;
  }

  void publish_state()
  {
    if (autonomous_ && !follower_fresh()) {
      set_autonomous(false, "path follower stopped");
    }
    if (autonomous_ && localization_lost()) {
      set_autonomous(false, "localization lost");
    }
    DriveState state;
    state.header.stamp = now();
    bool teleop_active = (now() - teleop_time_).seconds() < 0.2;
    if (teleop_active) {
      state.source = assisting_ ? DriveState::ASSIST : DriveState::MANUAL;
    } else if (autonomous_) {
      state.source = DriveState::AUTONOMOUS;
    } else {
      state.source = DriveState::IDLE;
    }
    state.follower_available = follower_fresh();
    state.assist_enabled = assist_enabled_;
    state.autonomous_enabled = autonomous_;
    if ((now() - message_time_).seconds() > 10.0) {
      message_.clear();  // events, not states: don't let an old refusal linger
    }
    state.message = message_;
    state_pub_->publish(state);
  }

  bool assist_enabled_, autonomous_ = false, assisting_ = false, estop_ = false;
  bool require_localization_;
  double localization_timeout_;
  rclcpp::Time localization_time_, localized_time_, message_time_;
  rclcpp::Subscription<f1tenth_bringup::msg::LocalizationState>::SharedPtr localization_sub_;
  bool engage_was_ = false, disengage_was_ = false;
  double follower_timeout_;
  int engage_button_, disengage_button_;
  std::string message_;
  AckermannStamped follower_;
  rclcpp::Time follower_time_, teleop_time_;

  rclcpp::Publisher<AckermannStamped>::SharedPtr teleop_pub_, auto_pub_;
  rclcpp::Publisher<DriveState>::SharedPtr state_pub_;
  rclcpp::Subscription<AckermannStamped>::SharedPtr teleop_sub_, follower_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lock_sub_, estop_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr autonomous_srv_, assist_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveCoordinator>());
  rclcpp::shutdown();
  return 0;
}
