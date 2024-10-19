#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include <chrono>


class TransformToOdom : public rclcpp::Node {
private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped transform_;
    // geometry_msgs::msg::TransformStamped test_tf_;
    std::string source_frame_;
    std::string target_frame_;
    std::string odom_topic_;
    nav_msgs::msg::Odometry odom_msg_;
    double rate_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishOdometry() {
        try {
            transform_ = tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);

            odom_msg_.header.stamp = this->get_clock()->now();
            odom_msg_.header.frame_id = target_frame_;
            odom_msg_.child_frame_id = source_frame_;
            odom_msg_.pose.pose.position.x = transform_.transform.translation.x;
            odom_msg_.pose.pose.position.y = transform_.transform.translation.y;
            odom_msg_.pose.pose.position.z = transform_.transform.translation.z;
            odom_msg_.pose.pose.orientation = transform_.transform.rotation;

            odom_pub_->publish(odom_msg_);



        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", source_frame_.c_str(), target_frame_.c_str(), ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));     // Wait before retrying
        }
    }

public:
    TransformToOdom(const std::string & name):Node(name) {
        // Declare parameters
        this->declare_parameter<std::string>("source_frame", "base_footprint");     // Frame to compare
        this->declare_parameter<std::string>("target_frame", "map");    // w.r.t. this frame
        this->declare_parameter<std::string>("odom_topic", "odom/map");
        this->declare_parameter<double>("rate", 50.0);                      // Rate to publish map frame

        // Get parameters
        this->get_parameter("source_frame", source_frame_);
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("rate", rate_);

        // Create a publisher for the odometry topic
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // Create a timer to call publishOdometry at the specified rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate_)),
            std::bind(&TransformToOdom::publishOdometry, this)
        );
        RCLCPP_INFO(this->get_logger(), "%s node has started. Publishing odometry %.2f Hz.", name.c_str(), rate_);
    }
    ~TransformToOdom() {}
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TransformToOdom>("transform_to_odom_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}