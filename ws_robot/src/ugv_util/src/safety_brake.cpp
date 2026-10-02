#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class SafetyNode : public rclcpp::Node {
public:
    SafetyNode() : Node("safety_node") {
        // Declare parameters
        this->declare_parameter("ittc_threshold", 0.2);
        this->declare_parameter("ittc_foward_drive_scan_width", 0.3);
        this->declare_parameter("ittc_foward_drive_x_scan_offset", -0.3);
        this->declare_parameter("ittc_brake_release_delay", 500);
        this->declare_parameter("force_stop_rectangular_region", std::vector<double>{0.0, 0.16, -0.14, 0.14});
        this->declare_parameter("force_stop_min_ray", 5);
        this->declare_parameter("enable_recovery", false);
        this->declare_parameter("enabled", true);   // false: never brake (switchable from the UI)
        this->declare_parameter("recovery_timeout", 3000);
        this->declare_parameter("recovery_backup_speed", -1.0);
        this->declare_parameter("recovery_backup_duration", 1000);  
        this->declare_parameter("braking_current", -10.0);
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("odom_topic", "/odom");
        this->declare_parameter("brake_publisher_topic", "/commands/motor/brake");
        this->declare_parameter("bool_publisher_topic", "/mux_bool/autonomous_cutoff_and_brake");
        this->declare_parameter("bypass_teleop_topic", "/joy");
        this->declare_parameter("bypass_teleop_button", 6);

        // Initialize publishers
        brake_pub_ = this->create_publisher<std_msgs::msg::Float64>(this->get_parameter("brake_publisher_topic").as_string(), 10);
        safe_pub_ = this->create_publisher<std_msgs::msg::Bool>(this->get_parameter("bool_publisher_topic").as_string(), 10);
        cmd_ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd_auto/recovery", 10);  

        // Initialize subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(this->get_parameter("scan_topic").as_string(), 10, std::bind(&SafetyNode::scan_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(this->get_parameter("odom_topic").as_string(), 10, std::bind(&SafetyNode::odom_callback, this, std::placeholders::_1));
        bypass_teleop_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(this->get_parameter("bypass_teleop_topic").as_string(), 10, std::bind(&SafetyNode::bypass_teleop_callback, this, std::placeholders::_1));

        // Initialize variables
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
        ittc_brake_release_delay_ = this->get_parameter("ittc_brake_release_delay").as_int();
        enable_recovery_ = this->get_parameter("enable_recovery").as_bool();
        enabled_ = this->get_parameter("enabled").as_bool();
        recovery_timeout_ = this->get_parameter("recovery_timeout").as_int();
        recovery_backup_speed_ = this->get_parameter("recovery_backup_speed").as_double();
        recovery_backup_duration_ = this->get_parameter("recovery_backup_duration").as_int();
        bypass_teleop_state_ = false;
        speed_ = 0.0;
        steering_angle_ = 0.0;
        toggle_emergency_brake_ = false;
        ittc_ = std::numeric_limits<double>::infinity();
        ittc_idx_ = 0;
        scan_frame_id_ = "";
        force_stop_start_time_ = this->get_clock()->now();
        backup_recovery_in_progress_ = false;

        RCLCPP_INFO(this->get_logger(), "Safety node started.");
        RCLCPP_INFO(this->get_logger(), "Maximum braking current: \033[1;33m%.2f A", braking_current_);
        RCLCPP_INFO(this->get_logger(), "iTTC brake release delay: \033[1;33m%d ms", static_cast<int>(ittc_brake_release_delay_));
        RCLCPP_INFO(this->get_logger(), "Recovery mode: \033[1;33m%s", enable_recovery_ ? "enabled" : "disabled");

        // Initialize dynamic configuration timer
        dyn_conf_timer_ = this->create_wall_timer(2s, std::bind(&SafetyNode::dyn_conf_timer_callback, this));
    }

private:
    void apply_emergency_brake() {
        if (!toggle_emergency_brake_) {
            toggle_emergency_brake_ = true;
            force_stop_start_time_ = this->get_clock()->now();
            RCLCPP_INFO(this->get_logger(), "\033[1;31mApplying emergency brake...");
        }
        auto bool_msg = std_msgs::msg::Bool();
        bool_msg.data = true;
        safe_pub_->publish(bool_msg);

        auto float_msg = std_msgs::msg::Float64();
        float_msg.data = braking_current_;
        brake_pub_->publish(float_msg);
    }

    void release_emergency_brake() {
        toggle_emergency_brake_ = false;
        ittc_ = std::numeric_limits<double>::infinity();

        auto float_msg = std_msgs::msg::Float64();
        float_msg.data = 0.0;
        brake_pub_->publish(float_msg);

        auto bool_msg = std_msgs::msg::Bool();
        bool_msg.data = false;
        safe_pub_->publish(bool_msg);

        RCLCPP_INFO(this->get_logger(), "\033[1;32mEmergency brake released.");
    }

    double compute_ittc(const std::vector<double>& ray_range, const std::vector<double>& ray_angle) {
        std::vector<double> r_dot(ray_angle.size());
        for (size_t i = 0; i < ray_angle.size(); ++i) {
            r_dot[i] = std::cos(ray_angle[i]) * speed_;
            if (r_dot[i] < 1e-3) r_dot[i] = 0;
        }

        std::vector<double> time_to_collision(ray_range.size(), std::numeric_limits<double>::infinity());
        for (size_t i = 0; i < ray_range.size(); ++i) {
            if (r_dot[i] != 0) {
                time_to_collision[i] = ray_range[i] / r_dot[i];
            }
        }

        ittc_ = *std::min_element(time_to_collision.begin(), time_to_collision.end());
        ittc_idx_ = std::distance(time_to_collision.begin(), std::min_element(time_to_collision.begin(), time_to_collision.end()));
        return ittc_;
    }

    void bypass_teleop_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        bypass_teleop_state_ = joy_msg->buttons[bypass_teleop_button_];
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        speed_ = odom_msg->twist.twist.linear.x;
    }

    void cmd_ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr ackermann_msg) {
        steering_angle_ = ackermann_msg->drive.steering_angle;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        scan_frame_id_ = scan_msg->header.frame_id;
        if (!enabled_) {
            if (toggle_emergency_brake_) {
                release_emergency_brake();
            }
            return;
        }

        std::vector<double> ray_range(scan_msg->ranges.begin(), scan_msg->ranges.end());
        std::vector<double> ray_angle(ray_range.size());
        for (size_t i = 0; i < ray_range.size(); ++i) {
            ray_angle[i] = scan_msg->angle_min + i * (scan_msg->angle_max - scan_msg->angle_min) / (ray_range.size() - 1);
        }

        std::vector<double> x(ray_range.size()), y(ray_range.size());
        for (size_t i = 0; i < ray_range.size(); ++i) {
            x[i] = ray_range[i] * std::cos(ray_angle[i]);
            y[i] = ray_range[i] * std::sin(ray_angle[i]);
        }

        // Check if any point is within the force stop rectangular region
        bool within_rect = false;
        for (size_t i = 0; i < x.size(); ++i) {
            if (fstop_rect_x_min_ <= x[i] && x[i] <= fstop_rect_x_max_ && fstop_rect_y_min_ <= y[i] && y[i] <= fstop_rect_y_max_) {
                within_rect = true;
                break;
            }
        }
        
        if (within_rect && !bypass_teleop_state_) {     // Check if an object is within the force stop rectangular region and teleop is not bypassed
            apply_emergency_brake();
            if (!enable_recovery_){
                RCLCPP_INFO(this->get_logger(), "\033[1;31mAn object is in the force stop scan boundary! \033[1;33mUse teleop to move the vehicle.");
                std::this_thread::sleep_for(1s);
            } else {
                RCLCPP_INFO(this->get_logger(), "\033[1;31mAn object is in the force stop scan boundary! \033[1;33mWill attempt backup recovery...");
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(recovery_timeout_/5.0)));
                if ((this->get_clock()->now() - force_stop_start_time_).seconds() * 1000 > recovery_timeout_) {
                    start_backup_recovery();
                }
            }
        } else {
            if (speed_ > 0) {
                std::vector<double> filtered_ray_range, filtered_ray_angle;
                for (size_t i = 0; i < ray_range.size(); ++i) {
                    if (x[i] >= ittc_foward_drive_x_scan_offset_ && std::abs(y[i]) <= (ittc_foward_drive_scan_width_ / 2)) {
                        filtered_ray_range.push_back(ray_range[i]);
                        filtered_ray_angle.push_back(ray_angle[i]);
                    }
                }
                ray_range = filtered_ray_range;
                ray_angle = filtered_ray_angle;
            } else {
                std::vector<double> filtered_ray_range, filtered_ray_angle;
                for (size_t i = 0; i < ray_range.size(); ++i) {
                    if (ray_angle[i] > -2.268928028 && ray_angle[i] < 2.268928028) {
                        filtered_ray_range.push_back(ray_range[i]);
                        filtered_ray_angle.push_back(ray_angle[i]);
                    }
                }
                ray_range = filtered_ray_range;
                ray_angle = filtered_ray_angle;
            }

            compute_ittc(ray_range, ray_angle);
            if (ittc_ < ittc_threshold_ && !bypass_teleop_state_) {
                apply_emergency_brake();
                RCLCPP_INFO(this->get_logger(), "iTTC: %.3f s (< %.3f s) to object at %.2f° with approaching speed of %.3f m/s (%.3f m/s)", ittc_, ittc_threshold_, ray_angle[ittc_idx_] * 180 / M_PI, ray_range[ittc_idx_] / ittc_, speed_);
                RCLCPP_INFO(this->get_logger(), "Emergency brake applied %.3f m before collision to object.", ray_range[ittc_idx_]);
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(ittc_brake_release_delay_)));  // 1000 milliseconds = 1 second;
            } else if (toggle_emergency_brake_) {
                release_emergency_brake();
            }
        }
    }

    void start_backup_recovery() {
        if (!backup_recovery_in_progress_) {
            backup_recovery_in_progress_ = true;
            RCLCPP_INFO(this->get_logger(), "\033[1;33mStarting backup recovery...");

            auto ackermann_msg = ackermann_msgs::msg::AckermannDriveStamped();
            ackermann_msg.drive.speed = recovery_backup_speed_;

            auto start_time = std::chrono::steady_clock::now();
            auto end_time = start_time + std::chrono::milliseconds(static_cast<int>(recovery_backup_duration_));

            while (std::chrono::steady_clock::now() < end_time) {
                cmd_ackermann_pub_->publish(ackermann_msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Adjust the sleep duration as needed
            }

            stop_backup_recovery();
        }
    }

    void stop_backup_recovery() {
        backup_recovery_in_progress_ = false;
        RCLCPP_INFO(this->get_logger(), "\033[1;33mStopping backup recovery...");

        // Publish stop command
        auto ackermann_msg = ackermann_msgs::msg::AckermannDriveStamped();
        ackermann_msg.drive.speed = 0.0;  // Stop the vehicle
        cmd_ackermann_pub_->publish(ackermann_msg);
    }

    void dyn_conf_timer_callback()
    {
        braking_current_ = this->get_parameter("braking_current").as_double();
        ittc_threshold_ = this->get_parameter("ittc_threshold").as_double();
        fstop_min_ray_ = this->get_parameter("force_stop_min_ray").as_int();
        auto force_stop_rect = this->get_parameter("force_stop_rectangular_region").as_double_array();
        bypass_teleop_button_ = this->get_parameter("bypass_teleop_button").as_int();
        ittc_foward_drive_scan_width_ = this->get_parameter("ittc_foward_drive_scan_width").as_double();
        ittc_foward_drive_x_scan_offset_ = this->get_parameter("ittc_foward_drive_x_scan_offset").as_double();
        ittc_brake_release_delay_ = this->get_parameter("ittc_brake_release_delay").as_int();
        enable_recovery_ = this->get_parameter("enable_recovery").as_bool();
        enabled_ = this->get_parameter("enabled").as_bool();
        recovery_timeout_ = this->get_parameter("recovery_timeout").as_int();
        recovery_backup_speed_ = this->get_parameter("recovery_backup_speed").as_double();
        recovery_backup_duration_ = this->get_parameter("recovery_backup_duration").as_int();
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safe_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_ackermann_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr bypass_teleop_sub_;
    rclcpp::TimerBase::SharedPtr dyn_conf_timer_;

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
    double ittc_brake_release_delay_;
    double enable_recovery_;
    double recovery_timeout_;
    double recovery_backup_speed_;
    int recovery_backup_duration_;
    bool bypass_teleop_state_;
    bool enabled_;
    double speed_;
    double steering_angle_;
    bool toggle_emergency_brake_;
    double ittc_;
    size_t ittc_idx_;
    std::string scan_frame_id_;
    rclcpp::Time force_stop_start_time_;
    bool backup_recovery_in_progress_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}