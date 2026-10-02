#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

// Scales joystick teleop speed by how far a trigger is pressed: 1x when released, max_multiplier fully pressed.
// Sits between joy_teleop (cmd_teleop/joy_raw) and ackermann_mux (cmd_teleop/joy).
class TeleopSpeedMultiplier : public rclcpp::Node
{
public:
    TeleopSpeedMultiplier() : Node("teleop_speed_multiplier")
    {
        this->declare_parameter<std::string>("joy_topic", "/joy");
        this->declare_parameter<int>("multiplier_axis", 5);         // trigger axis: 1.0 released, -1.0 fully pressed
        this->declare_parameter<double>("max_multiplier", 5.0);
        this->get_parameter("multiplier_axis", multiplier_axis_);
        this->get_parameter("max_multiplier", max_multiplier_);

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("cmd_teleop/joy", 10);
        drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "cmd_teleop/joy_raw", 10, std::bind(&TeleopSpeedMultiplier::drive_callback, this, std::placeholders::_1));
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            this->get_parameter("joy_topic").as_string(), 10,
            std::bind(&TeleopSpeedMultiplier::joy_callback, this, std::placeholders::_1));

        // Both parameters can be changed live (e.g. from the UI)
        param_cb_ = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto & p : params) {
                if (p.get_name() == "max_multiplier") {
                    if (p.as_double() < 1.0) {
                        result.successful = false;
                        result.reason = "max_multiplier must be at least 1.0";
                    } else {
                        max_multiplier_ = p.as_double();
                    }
                } else if (p.get_name() == "multiplier_axis") {
                    multiplier_axis_ = static_cast<int>(p.as_int());
                }
            }
            return result;
        });

        RCLCPP_INFO(this->get_logger(), "Teleop speed multiplier started (axis %d, up to %.1fx).", multiplier_axis_, max_multiplier_);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (multiplier_axis_ < 0 || multiplier_axis_ >= static_cast<int>(msg->axes.size())) {
            return;
        }
        double pressed = std::clamp((1.0 - msg->axes[multiplier_axis_]) / 2.0, 0.0, 1.0);
        multiplier_ = 1.0 + (max_multiplier_ - 1.0) * pressed;
    }

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        auto scaled = *msg;
        scaled.drive.speed *= multiplier_;
        drive_pub_->publish(scaled);
    }

    int multiplier_axis_;
    double max_multiplier_;
    double multiplier_ = 1.0;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopSpeedMultiplier>());
    rclcpp::shutdown();
    return 0;
}
