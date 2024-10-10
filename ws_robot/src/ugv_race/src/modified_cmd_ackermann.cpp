#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class AckermannRepublisher : public rclcpp::Node
{
public:
    AckermannRepublisher()
    : Node("ackermann_republisher")
    {
        this->declare_parameter<double>("min_speed_threshold", 0.5);
        this->get_parameter("min_speed_threshold", min_speed_threshold_);


        // Create a subscriber to the /ackermann_cmd topic
        subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/ackermann_cmd", 10,
            std::bind(&AckermannRepublisher::ackermann_callback, this, std::placeholders::_1));

        // Create a publisher to the /ackermann_cmd_republished topic
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd_pro", 10);
    }

private:
    void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        if (std::abs(msg->drive.speed) == 0.0)
        {
            msg->drive.speed = 0.0;
        }
        else if (std::abs(msg->drive.speed) < min_speed_threshold_)
        {
            msg->drive.speed = (msg->drive.speed > 0) ? min_speed_threshold_ : -min_speed_threshold_;
        }
        publisher_->publish(*msg);
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    double min_speed_threshold_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannRepublisher>());
    rclcpp::shutdown();
    return 0;
}