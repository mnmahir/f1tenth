#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class CmdVelToAckermann : public rclcpp::Node
{
public:
    CmdVelToAckermann() : Node("cmd_vel_to_ackermann")
    {
        // Declare and get the wheelbase parameter
        this->declare_parameter<double>("wheelbase", 1.0);
        this->get_parameter("wheelbase", wheelbase_);

        // Initialize publisher
        ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10);

        // Initialize subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CmdVelToAckermann::cmdVelCallback, this, std::placeholders::_1));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Create Ackermann message
        auto ackermann_msg = ackermann_msgs::msg::AckermannDriveStamped();
        ackermann_msg.header.stamp = this->now();
        ackermann_msg.header.frame_id = "base_link";

        // Recalculate steering angle and speed
        ackermann_msg.drive.steering_angle = atan2(msg->angular.z * wheelbase_, msg->linear.x);
        ackermann_msg.drive.speed = msg->linear.x;

        // Publish Ackermann message
        ackermann_pub_->publish(ackermann_msg);
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    double wheelbase_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToAckermann>());
    rclcpp::shutdown();
    return 0;
}