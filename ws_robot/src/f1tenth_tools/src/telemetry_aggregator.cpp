// Collects the car's vitals (VESC state, commands, joystick, locks) into one f1tenth_bringup/Telemetry
// message for the UI, so a remote UI needs a single small topic instead of many.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "f1tenth_bringup/msg/telemetry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

using namespace std::chrono_literals;

class TelemetryAggregator : public rclcpp::Node
{
public:
  TelemetryAggregator() : Node("telemetry_aggregator")
  {
    // Shared with the VESC nodes (vesc.yaml)
    speed_gain_ = declare_parameter<double>("speed_to_erpm_gain", 4564.0);
    speed_offset_ = declare_parameter<double>("speed_to_erpm_offset", 0.0);
    servo_gain_ = declare_parameter<double>("steering_angle_to_servo_gain", -0.895);
    servo_offset_ = declare_parameter<double>("steering_angle_to_servo_offset", 0.5);
    battery_cells_ = declare_parameter<int>("battery_cells", 3);
    // Joystick layout (Xbox One S over Bluetooth)
    deadman_button_ = declare_parameter<int>("deadman_button", 6);
    throttle_axis_ = declare_parameter<int>("throttle_axis", 1);
    steer_axis_ = declare_parameter<int>("steer_axis", 2);
    brake_axis_ = declare_parameter<int>("brake_axis", 4);
    boost_axis_ = declare_parameter<int>("boost_axis", 5);
    double rate = declare_parameter<double>("rate", 30.0);

    pub_ = create_publisher<f1tenth_bringup::msg::Telemetry>("/f1tenth/telemetry", 10);
    vesc_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
      "/sensors/core", 10, [this](vesc_msgs::msg::VescStateStamped::SharedPtr msg) {
        vesc_ = msg->state;
        vesc_time_ = now();
      });
    brake_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/commands/motor/brake", 10, [this](std_msgs::msg::Float64::SharedPtr msg) {
        brake_ = std::abs(msg->data);
        brake_time_ = now();
      });
    cmd_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/ackermann_cmd_filtered", 10, [this](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        cmd_ = msg->drive;
        cmd_time_ = now();
      });
    servo_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/sensors/servo_position_command", 10, [this](std_msgs::msg::Float64::SharedPtr msg) {
        steering_angle_ = (msg->data - servo_offset_) / servo_gain_;
      });
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, [this](sensor_msgs::msg::Joy::SharedPtr msg) {
        joy_ = *msg;
        joy_time_ = now();
      });
    teleop_lock_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/mux_bool/teleop_cutoff_and_brake", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {teleop_lock_ = msg->data;});
    auto_lock_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/mux_bool/autonomous_cutoff_and_brake", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {autonomous_lock_ = msg->data;});
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/mux_bool/ui_estop", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {estop_ = msg->data;});

    vesc_time_ = brake_time_ = cmd_time_ = joy_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {publish();});
  }

private:
  static double age(const rclcpp::Time & now, const rclcpp::Time & t) {return (now - t).seconds();}

  double axis(int index) const
  {
    return index >= 0 && index < static_cast<int>(joy_.axes.size()) ? joy_.axes[index] : 0.0;
  }

  // Trigger axes rest at 1.0 and read -1.0 fully pressed
  double trigger(int index) const
  {
    if (index < 0 || index >= static_cast<int>(joy_.axes.size())) {
      return 0.0;
    }
    return std::clamp((1.0 - joy_.axes[index]) / 2.0, 0.0, 1.0);
  }

  // Typical LiPo resting curve, % per cell voltage
  static double lipo_percent(double cell_voltage)
  {
    static const std::array<std::pair<double, double>, 12> curve{{
      {3.27, 0}, {3.61, 5}, {3.69, 10}, {3.73, 20}, {3.77, 30}, {3.80, 40},
      {3.84, 50}, {3.87, 60}, {3.95, 70}, {4.02, 80}, {4.11, 90}, {4.20, 100}}};
    if (cell_voltage <= curve.front().first) {
      return 0.0;
    }
    for (size_t i = 1; i < curve.size(); ++i) {
      if (cell_voltage <= curve[i].first) {
        double t = (cell_voltage - curve[i - 1].first) / (curve[i].first - curve[i - 1].first);
        return curve[i - 1].second + t * (curve[i].second - curve[i - 1].second);
      }
    }
    return 100.0;
  }

  static std::string fault_name(int code)
  {
    static const char * names[] = {
      "", "OVER VOLTAGE", "UNDER VOLTAGE", "DRV", "ABS OVER CURRENT", "OVER TEMP FET",
      "OVER TEMP MOTOR", "GATE DRIVER OVER VOLTAGE", "GATE DRIVER UNDER VOLTAGE", "MCU UNDER VOLTAGE",
      "WATCHDOG RESET", "ENCODER SPI", "ENCODER SINCOS BELOW MIN", "ENCODER SINCOS ABOVE MAX",
      "FLASH CORRUPTION", "HIGH OFFSET CURRENT 1", "HIGH OFFSET CURRENT 2", "HIGH OFFSET CURRENT 3",
      "UNBALANCED CURRENTS", "BRK", "RESOLVER LOT", "RESOLVER DOS", "RESOLVER LOS"};
    if (code >= 0 && code < static_cast<int>(sizeof(names) / sizeof(names[0]))) {
      return names[code];
    }
    return "FAULT " + std::to_string(code);
  }

  void publish()
  {
    auto t = now();
    f1tenth_bringup::msg::Telemetry msg;
    msg.header.stamp = t;

    msg.vesc_connected = age(t, vesc_time_) < 0.5;
    msg.erpm = vesc_.speed;
    msg.speed = (vesc_.speed - speed_offset_) / speed_gain_;
    msg.duty_cycle = vesc_.duty_cycle;
    msg.battery_voltage = vesc_.voltage_input;
    msg.battery_percent = battery_cells_ > 0 ? lipo_percent(vesc_.voltage_input / battery_cells_) : 0.0;
    msg.motor_current = vesc_.current_motor;
    msg.input_current = vesc_.current_input;
    msg.temp_fet = vesc_.temp_fet;
    msg.temp_motor = vesc_.temp_motor;
    msg.energy_drawn = vesc_.energy_drawn;
    // 6 tachometer steps per electrical revolution; erpm per m/s = gain, so steps per metre = gain / 10
    msg.distance = speed_gain_ > 0 ? vesc_.distance_traveled * 10.0 / speed_gain_ : 0.0;
    msg.fault_code = vesc_.fault_code;
    msg.fault = fault_name(vesc_.fault_code);

    msg.brake_current = age(t, brake_time_) < 0.5 ? brake_ : 0.0;
    bool cmd_fresh = age(t, cmd_time_) < 0.5;
    msg.speed_command = cmd_fresh ? cmd_.speed : 0.0;
    msg.steering_command = cmd_fresh ? cmd_.steering_angle : 0.0;
    msg.steering_angle = steering_angle_;

    msg.joystick_connected = age(t, joy_time_) < 1.0;
    if (msg.joystick_connected) {
      msg.deadman = deadman_button_ >= 0 && deadman_button_ < static_cast<int>(joy_.buttons.size()) &&
        joy_.buttons[deadman_button_] != 0;
      msg.throttle = axis(throttle_axis_);
      msg.steer_stick = axis(steer_axis_);
      msg.brake_trigger = trigger(brake_axis_);
      msg.boost = trigger(boost_axis_);
    }
    msg.teleop_lock = teleop_lock_;
    msg.autonomous_lock = autonomous_lock_;
    msg.estop = estop_;
    pub_->publish(msg);
  }

  double speed_gain_, speed_offset_, servo_gain_, servo_offset_;
  int battery_cells_, deadman_button_, throttle_axis_, steer_axis_, brake_axis_, boost_axis_;

  vesc_msgs::msg::VescState vesc_;
  ackermann_msgs::msg::AckermannDrive cmd_;
  sensor_msgs::msg::Joy joy_;
  double brake_ = 0.0, steering_angle_ = 0.0;
  bool teleop_lock_ = false, autonomous_lock_ = false, estop_ = false;
  rclcpp::Time vesc_time_, brake_time_, cmd_time_, joy_time_;

  rclcpp::Publisher<f1tenth_bringup::msg::Telemetry>::SharedPtr pub_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr brake_sub_, servo_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr teleop_lock_sub_, auto_lock_sub_, estop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetryAggregator>());
  rclcpp::shutdown();
  return 0;
}
