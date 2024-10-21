#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

class CutoffAndBrakeControlNode : public rclcpp::Node
{
public:
    CutoffAndBrakeControlNode()
    : Node("cutoff_and_brake_control_node")
    {
        this->declare_parameter<std::string>("joy_topic", "/joy");
        this->declare_parameter<int>("joy_axis", 4);               // axis for cutoff and brake
        this->declare_parameter<double>("braking_current", -10.0);   // threshold for instantaneous time to collision (iTTC)
        this->declare_parameter<std::string>("brake_publisher_topic", "/commands/motor/brake");
        this->declare_parameter<std::string>("bool_publisher_topic", "/mux_bool/teleop_cutoff_and_brake");

        this->get_parameter("joy_axis", joy_axis_);
        this->get_parameter("braking_current", braking_current_);

        toggle_cb_ = false;
        brake_value_ = 0.0;

        // Subscribers
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            this->get_parameter("joy_topic").as_string(), 10,
            std::bind(&CutoffAndBrakeControlNode::joy_callback, this, std::placeholders::_1));

        // Publishers
        brake_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            this->get_parameter("brake_publisher_topic").as_string(), 10);
        cb_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            this->get_parameter("bool_publisher_topic").as_string(), 10);

        RCLCPP_INFO(this->get_logger(), "Cutoff and brake control node started.");
        RCLCPP_INFO(this->get_logger(), "Maximum braking current: %.2f A", braking_current_);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes[joy_axis_] < 1.0)
        {
            if (!toggle_cb_)
            {
                toggle_cb_ = true;
                RCLCPP_INFO(this->get_logger(), "\033[1;31mCutoff and braking...");
            }
            toggle_cb_ = true;
            auto bool_msg = std_msgs::msg::Bool();
            bool_msg.data = true;
            cb_pub_->publish(bool_msg);

            brake_value_ = (1 - msg->axes[joy_axis_]) * braking_current_ / 2;
            auto float_msg = std_msgs::msg::Float64();
            float_msg.data = brake_value_;
            brake_pub_->publish(float_msg);
        }
        else if (toggle_cb_)
        {
            toggle_cb_ = false;
            auto float_msg = std_msgs::msg::Float64();
            float_msg.data = 0.0;
            brake_pub_->publish(float_msg);
            auto bool_msg = std_msgs::msg::Bool();
            bool_msg.data = false;
            cb_pub_->publish(bool_msg);
            RCLCPP_INFO(this->get_logger(), "\033[1;32mBrake released.");
        }
    }

    int joy_axis_;
    double braking_current_;
    bool toggle_cb_;
    double brake_value_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cb_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CutoffAndBrakeControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}