#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

// Publishes the URDF's steering and wheel joints from what the car is doing, so the model steers in RViz.
// Steering comes from the servo position the VESC applied, split into left/right angles with Ackermann
// geometry; wheel rotation integrates the wheel odometry speed.
class VehicleJointStatePublisher : public rclcpp::Node
{
public:
    VehicleJointStatePublisher() : Node("vehicle_joint_state_publisher")
    {
        // Servo mapping shared with ackermann_to_vesc (vesc.yaml): servo = gain * steering_angle + offset
        this->declare_parameter<double>("steering_angle_to_servo_gain", -0.895);
        this->declare_parameter<double>("steering_angle_to_servo_offset", 0.5);
        this->declare_parameter<double>("wheelbase", 0.324);
        this->declare_parameter<double>("steering_track", 0.192);      // distance between the front steering pivots
        this->declare_parameter<double>("wheel_radius", 0.05);
        this->declare_parameter<double>("max_steering_angle", 0.5);    // URDF steer joint limit
        this->declare_parameter<double>("odom_timeout", 0.5);          // wheels stop spinning after this long without odometry (s)
        this->declare_parameter<double>("rate", 30.0);

        this->get_parameter("steering_angle_to_servo_gain", servo_gain_);
        this->get_parameter("steering_angle_to_servo_offset", servo_offset_);
        this->get_parameter("wheelbase", wheelbase_);
        this->get_parameter("steering_track", steering_track_);
        this->get_parameter("wheel_radius", wheel_radius_);
        this->get_parameter("max_steering_angle", max_steering_angle_);
        this->get_parameter("odom_timeout", odom_timeout_);
        double rate = this->get_parameter("rate").as_double();

        servo_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "sensors/servo_position_command", 10,
            std::bind(&VehicleJointStatePublisher::servo_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom/wheel", 10,
            std::bind(&VehicleJointStatePublisher::odom_callback, this, std::placeholders::_1));
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        last_publish_ = this->now();
        last_odom_ = this->now();
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate),
                                         std::bind(&VehicleJointStatePublisher::publish_joint_states, this));

        joint_state_.name = {"front_left_steer_joint", "front_right_steer_joint",
                             "front_left_wheel_joint", "front_right_wheel_joint",
                             "rear_left_wheel_joint", "rear_right_wheel_joint"};
        joint_state_.position.assign(joint_state_.name.size(), 0.0);

        RCLCPP_INFO(this->get_logger(), "Vehicle joint state publisher started.");
    }

private:
    void servo_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        steering_angle_ = (msg->data - servo_offset_) / servo_gain_;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        speed_ = msg->twist.twist.linear.x;
        last_odom_ = this->now();
    }

    void publish_joint_states()
    {
        auto now = this->now();
        double dt = (now - last_publish_).seconds();
        last_publish_ = now;
        double speed = (now - last_odom_).seconds() > odom_timeout_ ? 0.0 : speed_;

        // Ackermann: both front wheels point at the same turning centre on the rear axle line
        double left = 0.0, right = 0.0;
        if (std::abs(std::tan(steering_angle_)) > 1e-6) {
            double turning_radius = wheelbase_ / std::tan(steering_angle_);
            left = std::atan(wheelbase_ / (turning_radius - steering_track_ / 2.0));
            right = std::atan(wheelbase_ / (turning_radius + steering_track_ / 2.0));
        }

        wheel_angle_ = std::fmod(wheel_angle_ + speed * dt / wheel_radius_, 2.0 * M_PI);

        joint_state_.header.stamp = now;
        joint_state_.position = {std::clamp(left, -max_steering_angle_, max_steering_angle_),
                                 std::clamp(right, -max_steering_angle_, max_steering_angle_),
                                 wheel_angle_, wheel_angle_, wheel_angle_, wheel_angle_};
        joint_state_pub_->publish(joint_state_);
    }

    double servo_gain_, servo_offset_, wheelbase_, steering_track_, wheel_radius_, max_steering_angle_, odom_timeout_;
    double steering_angle_ = 0.0;
    double speed_ = 0.0;
    double wheel_angle_ = 0.0;
    rclcpp::Time last_publish_, last_odom_;
    sensor_msgs::msg::JointState joint_state_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleJointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
