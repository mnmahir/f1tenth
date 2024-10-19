#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class SafetyNode : public rclcpp::Node
{
public:
    SafetyNode() : Node("safety_node")
    {
        // Declare parameters
        this->declare_parameter("ittc_threshold", 0.2);
        this->declare_parameter("ittc_foward_drive_scan_width", 0.3);
        this->declare_parameter("ittc_foward_drive_x_scan_offset", -0.3);
        this->declare_parameter("force_stop_rectangular_region", std::vector<double>{0.0, 0.16, -0.14, 0.14});
        this->declare_parameter("force_stop_min_ray", 5);
        this->declare_parameter("braking_current", -10.0);
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("odom_topic", "/odom");
        this->declare_parameter("brake_publisher_topic", "/commands/motor/brake");
        this->declare_parameter("bool_publisher_topic", "/mux_bool/autonomous_cutoff_and_brake");
        this->declare_parameter("bypass_teleop_topic", "/joy");
        this->declare_parameter("bypass_teleop_button", 6);

        // Get parameters
        braking_current_ = this->get_parameter("braking_current").as_double();
        ittc_threshold_ = this->get_parameter("ittc_threshold").as_double();
        fstop_min_ray_ = this->get_parameter("force_stop_min_ray").as_int();
        auto force_stop_rect = this->get_parameter("force_stop_rectangular_region").as_double_array();
        fstop_rect_x_min_ = force_stop_rect[0];
        fstop_rect_x_max_ = force_stop_rect[1];
        fstop_rect_y_min_ = force_stop_rect[2];
        fstop_rect_y_max_ = force_stop_rect[3];
        bypass_teleop_button_ = this->get_parameter("bypass_teleop_button").as_int();
        ittc_foward_drive_scan_width_ = this->get_parameter("ittc_foward_drive_scan_width").as_double();
        ittc_foward_drive_x_scan_offset_ = this->get_parameter("ittc_foward_drive_x_scan_offset").as_double();

        // Create publishers
        brake_pub_ = this->create_publisher<std_msgs::msg::Float64>(this->get_parameter("brake_publisher_topic").as_string(), 10);
        safe_pub_ = this->create_publisher<std_msgs::msg::Bool>(this->get_parameter("bool_publisher_topic").as_string(), 10);
        force_stop_boundary_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/safety/force_stop_boundary", 10);
        force_ittc_stop_boundary_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/safety/ittc_stop_boundary", 10);

        // Create subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(this->get_parameter("scan_topic").as_string(), 10, std::bind(&SafetyNode::scan_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(this->get_parameter("odom_topic").as_string(), 10, std::bind(&SafetyNode::odom_callback, this, std::placeholders::_1));
        cmd_ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10, std::bind(&SafetyNode::cmd_ackermann_callback, this, std::placeholders::_1));
        bypass_teleop_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(this->get_parameter("bypass_teleop_topic").as_string(), 10, std::bind(&SafetyNode::bypass_teleop_callback, this, std::placeholders::_1));

        // Create timer
        timer_ = this->create_wall_timer(1s, std::bind(&SafetyNode::publish_force_stop_boundary, this));

        // Initialize variables
        bypass_teleop_state_ = false;
        speed_ = 0.0;
        steering_angle_ = 0.0;
        toggle_emergency_brake_ = false;
        ittc_ = std::numeric_limits<double>::infinity();
        ittc_idx_ = 0;

        RCLCPP_INFO(this->get_logger(), "Safety node started.");
        RCLCPP_INFO(this->get_logger(), "Maximum braking current: %f A", braking_current_);
    }

private:
    void publish_force_stop_boundary()
    {
        auto polygon = geometry_msgs::msg::PolygonStamped();
        polygon.header.frame_id = "lidar_1_link";
        polygon.header.stamp = this->get_clock()->now();

        std::vector<std::vector<double>> points = {
            {fstop_rect_x_min_, fstop_rect_y_min_, 0.0},
            {fstop_rect_x_min_, fstop_rect_y_max_, 0.0},
            {fstop_rect_x_max_, fstop_rect_y_max_, 0.0},
            {fstop_rect_x_max_, fstop_rect_y_min_, 0.0}};

        for (const auto &point : points)
        {
            geometry_msgs::msg::Point32 p;
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            polygon.polygon.points.push_back(p);
        }

        force_stop_boundary_pub_->publish(polygon);
    }

    void publish_ittc_foward_drive_scan_boundary()
    {
        auto polygon = geometry_msgs::msg::PolygonStamped();
        polygon.header.frame_id = "lidar_1_link";
        polygon.header.stamp = this->get_clock()->now();

        std::vector<std::vector<double>> points = {
            {ittc_foward_drive_x_scan_offset_, -(ittc_foward_drive_scan_width_ / 2) - std::abs((ittc_foward_drive_scan_width_ * steering_angle_)), 0.0},
            {ittc_foward_drive_x_scan_offset_, (ittc_foward_drive_scan_width_ / 2) + std::abs((ittc_foward_drive_scan_width_ * steering_angle_)), 0.0},
            {1.0, -(ittc_foward_drive_scan_width_ / 2) - std::abs((ittc_foward_drive_scan_width_ * steering_angle_)), 0.0},
            {1.0, (ittc_foward_drive_scan_width_ / 2) + std::abs((ittc_foward_drive_scan_width_ * steering_angle_)), 0.0}};

        for (const auto &point : points)
        {
            geometry_msgs::msg::Point32 p;
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            polygon.polygon.points.push_back(p);
        }

        force_ittc_stop_boundary_pub_->publish(polygon);
    }

    void apply_emergency_brake()
    {
        if (!toggle_emergency_brake_)
        {
            toggle_emergency_brake_ = true;
            RCLCPP_INFO(this->get_logger(), "Applying emergency brake...");
        }
        auto bool_msg = std_msgs::msg::Bool();
        bool_msg.data = true;
        safe_pub_->publish(bool_msg);

        auto brake_msg = std_msgs::msg::Float64();
        brake_msg.data = braking_current_;
        brake_pub_->publish(brake_msg);
    }

    void release_emergency_brake()
    {
        toggle_emergency_brake_ = false;
        ittc_ = std::numeric_limits<double>::infinity();

        auto brake_msg = std_msgs::msg::Float64();
        brake_msg.data = 0.0;
        brake_pub_->publish(brake_msg);

        auto bool_msg = std_msgs::msg::Bool();
        bool_msg.data = false;
        safe_pub_->publish(bool_msg);

        RCLCPP_INFO(this->get_logger(), "Emergency brake released.");
    }

    double compute_ittc(const std::vector<double> &ray_range, const std::vector<double> &ray_angle)
    {
        std::vector<double> r_dot(ray_angle.size());
        for (size_t i = 0; i < ray_angle.size(); ++i)
        {
            r_dot[i] = std::cos(ray_angle[i]) * speed_;
            if (r_dot[i] < 1e-3)
            {
                r_dot[i] = 0;
            }
        }

        std::vector<double> time_to_collision(ray_range.size(), std::numeric_limits<double>::infinity());
        for (size_t i = 0; i < ray_range.size(); ++i)
        {
            if (r_dot[i] != 0)
            {
                time_to_collision[i] = ray_range[i] / r_dot[i];
            }
        }

        ittc_ = *std::min_element(time_to_collision.begin(), time_to_collision.end());
        ittc_idx_ = std::distance(time_to_collision.begin(), std::min_element(time_to_collision.begin(), time_to_collision.end()));
        return ittc_;
    }

    void bypass_teleop_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        bypass_teleop_state_ = joy_msg->buttons[bypass_teleop_button_];
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        speed_ = odom_msg->twist.twist.linear.x;
    }

    void cmd_ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr ackermann_msg)
    {
        steering_angle_ = ackermann_msg->drive.steering_angle;
        publish_ittc_foward_drive_scan_boundary();
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        std::vector<double> ray_range(scan_msg->ranges.begin(), scan_msg->ranges.end());
        std::vector<double> ray_angle(ray_range.size());
        double angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / ray_range.size();
        for (size_t i = 0; i < ray_range.size(); ++i)
        {
            ray_angle[i] = scan_msg->angle_min + i * angle_increment;
        }

        std::vector<double> x(ray_range.size());
        std::vector<double> y(ray_range.size());
        for (size_t i = 0; i < ray_range.size(); ++i)
        {
            x[i] = ray_range[i] * std::cos(ray_angle[i]);
            y[i] = ray_range[i] * std::sin(ray_angle[i]);
        }

        std::vector<bool> within_rect(ray_range.size());
        for (size_t i = 0; i < ray_range.size(); ++i)
        {
            within_rect[i] = (fstop_rect_x_min_ <= x[i]) && (x[i] <= fstop_rect_x_max_) && (fstop_rect_y_min_ <= y[i]) && (y[i] <= fstop_rect_y_max_);
        }

        std::vector<bool> within_foward_drive_boundary(ray_range.size());
        for (size_t i = 0; i < ray_range.size(); ++i)
        {
            within_foward_drive_boundary[i] = (x[i] >= ittc_foward_drive_x_scan_offset_) && (std::abs(y[i]) <= ((ittc_foward_drive_scan_width_ / 2) + (ittc_foward_drive_scan_width_ * std::abs(steering_angle_))));
        }

        if (std::count(within_rect.begin(), within_rect.end(), true) >= fstop_min_ray_ && !bypass_teleop_state_)
        {
            apply_emergency_brake();
            RCLCPP_WARN(this->get_logger(), "An object is in the scan boundary! Vehicle force stopped. Use teleop to move the vehicle.");
            std::this_thread::sleep_for(1s);
        }
        else
        {
            if (speed_ > 0)
            {
                std::vector<double> filtered_ray_range;
                std::vector<double> filtered_ray_angle;
                for (size_t i = 0; i < ray_range.size(); ++i)
                {
                    if (within_foward_drive_boundary[i])
                    {
                        filtered_ray_range.push_back(ray_range[i]);
                        filtered_ray_angle.push_back(ray_angle[i]);
                    }
                }
                compute_ittc(filtered_ray_range, filtered_ray_angle);
            }
            else
            {
                std::vector<double> filtered_ray_range;
                std::vector<double> filtered_ray_angle;
                for (size_t i = 0; i < ray_range.size(); ++i)
                {
                    if (ray_angle[i] > -2.268928028 && ray_angle[i] < 2.268928028)
                    {
                        filtered_ray_range.push_back(ray_range[i]);
                        filtered_ray_angle.push_back(ray_angle[i]);
                    }
                }
                compute_ittc(filtered_ray_range, filtered_ray_angle);
            }

            if (ittc_ < ittc_threshold_ && !bypass_teleop_state_)
            {
                apply_emergency_brake();
                RCLCPP_WARN(this->get_logger(), "iTTC: %f s (< %f s) to object at %f° with approaching speed of %f m/s (%f m/s)", ittc_, ittc_threshold_, ray_angle[ittc_idx_] * 180 / M_PI, ray_range[ittc_idx_] / ittc_, speed_);
                RCLCPP_WARN(this->get_logger(), "Emergency brake applied %f m before collision to object.", ray_range[ittc_idx_]);
                std::this_thread::sleep_for(500ms);
            }
            else if (toggle_emergency_brake_)
            {
                release_emergency_brake();
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safe_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr force_stop_boundary_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr force_ittc_stop_boundary_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_ackermann_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr bypass_teleop_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double braking_current_;
    double ittc_threshold_;
    int fstop_min_ray_;
    double fstop_rect_x_min_;
    double fstop_rect_x_max_;
    double fstop_rect_y_min_;
    double fstop_rect_y_max_;
    int bypass_teleop_button_;
    double ittc_foward_drive_scan_width_;
    double ittc_foward_drive_x_scan_offset_;
    bool bypass_teleop_state_;
    double speed_;
    double steering_angle_;
    bool toggle_emergency_brake_;
    double ittc_;
    size_t ittc_idx_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}