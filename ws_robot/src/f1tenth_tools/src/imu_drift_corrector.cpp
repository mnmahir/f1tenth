// Keeps the IMU's yaw from drifting while the car stands still, for the EKF (/sensors/imu/corrected).
// The VESC's IMU integrates a biased gyro (about 0.4 deg/s at rest). A car with Ackermann steering can't turn
// without its wheels turning, so while the wheels are stopped the yaw is held and the bias is learned; while
// driving, the IMU's yaw changes are used minus that bias. Also reports the angular velocity in rad/s (the VESC
// driver publishes deg/s).
// "Stopped" comes from the motor speed in the VESC's state, which arrives from power-up. (/odom/wheel would do,
// but vesc_to_odom publishes nothing until the first steering command, so a car that hasn't driven yet drifted.)
#include <algorithm>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

namespace
{
double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
}

class ImuDriftCorrector : public rclcpp::Node
{
public:
  ImuDriftCorrector() : Node("imu_drift_corrector")
  {
    stationary_erpm_ = declare_parameter<double>("stationary_erpm", 100.0);  // motor ERPM, about 0.02 m/s
    stationary_time_ = declare_parameter<double>("stationary_time", 0.5);     // s stopped before holding the yaw
    bias_time_constant_ = declare_parameter<double>("bias_time_constant", 10.0);  // s
    max_bias_ = declare_parameter<double>("max_bias", 0.05);                  // rad/s, larger is not a bias
    pub_ = create_publisher<sensor_msgs::msg::Imu>("/sensors/imu/corrected", 20);
    vesc_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>("/sensors/core", rclcpp::SensorDataQoS(),
      [this](vesc_msgs::msg::VescStateStamped::SharedPtr vesc) {
        auto t = now();
        wheel_time_ = t;
        if (std::abs(vesc->state.speed) > stationary_erpm_) {
          moving_time_ = t;
        }
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/sensors/imu/raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr imu) {on_imu(*imu);});
    wheel_time_ = moving_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

private:
  void on_imu(sensor_msgs::msg::Imu imu)
  {
    const auto & q = imu.orientation;
    double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    double pitch = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
    double raw_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    rclcpp::Time stamp(imu.header.stamp, get_clock()->get_clock_type());
    if (!started_) {
      started_ = true;
      yaw_ = raw_yaw;
    } else {
      double dt = (stamp - last_stamp_).seconds();
      double change = wrap(raw_yaw - last_raw_yaw_);
      auto t = now();
      // No motor speed (VESC not connected): don't guess, pass the IMU through
      bool wheels_known = (t - wheel_time_).seconds() < 0.5;
      bool stopped = wheels_known && (t - moving_time_).seconds() > stationary_time_;
      if (dt > 0.0 && dt < 0.5) {
        if (stopped) {
          double rate = change / dt;
          if (std::abs(rate) < max_bias_) {
            bias_ += (rate - bias_) * std::min(1.0, dt / bias_time_constant_);
          }
          rate_ = 0.0;
        } else {
          yaw_ = wrap(yaw_ + change - bias_ * dt);
          rate_ = change / dt - bias_;
        }
      }
    }
    last_raw_yaw_ = raw_yaw;
    last_stamp_ = stamp;

    double cr = std::cos(roll / 2), sr = std::sin(roll / 2), cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    double cy = std::cos(yaw_ / 2), sy = std::sin(yaw_ / 2);
    imu.orientation.w = cr * cp * cy + sr * sp * sy;
    imu.orientation.x = sr * cp * cy - cr * sp * sy;
    imu.orientation.y = cr * sp * cy + sr * cp * sy;
    imu.orientation.z = cr * cp * sy - sr * sp * cy;
    constexpr double kDegToRad = M_PI / 180.0;
    imu.angular_velocity.x *= kDegToRad;
    imu.angular_velocity.y *= kDegToRad;
    imu.angular_velocity.z = rate_;
    pub_->publish(imu);
  }

  double stationary_erpm_, stationary_time_, bias_time_constant_, max_bias_;
  bool started_ = false;
  double yaw_ = 0.0, last_raw_yaw_ = 0.0, bias_ = 0.0, rate_ = 0.0;
  rclcpp::Time last_stamp_, wheel_time_, moving_time_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuDriftCorrector>());
  rclcpp::shutdown();
  return 0;
}
