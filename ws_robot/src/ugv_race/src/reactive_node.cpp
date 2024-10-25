#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class ReactiveFollowGap : public rclcpp::Node
{
public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        // Create ROS subscribers and publishers
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
        
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/cmd_auto/drive";

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    const float max_steering_angle = 24.0; // Max steering angle in degrees
    const float bubble_radius = 0.5; // Safety bubble radius in meters

    // Preprocess LiDAR data
    void preprocess_lidar(std::vector<float>& ranges)
    {
        // Set high range values (> 3 meters) to zero to reject far points
        for (auto& range : ranges) {
            if (range > 3.0) {
                range = 0.0;
            }
        }
    }

    // Find the start and end indices of the largest gap in the LiDAR scan data
    std::pair<int, int> find_max_gap(const std::vector<float>& ranges)
    {
        int max_gap_start = -1;
        int max_gap_end = -1;
        int gap_start = -1;
        int gap_end = -1;

        for (int i = 0; i < ranges.size(); ++i) {
            if (ranges[i] > 0) {
                if (gap_start == -1) {
                    gap_start = i;
                }
                gap_end = i;
            } else {
                if (gap_end - gap_start > max_gap_end - max_gap_start) {
                    max_gap_start = gap_start;
                    max_gap_end = gap_end;
                }
                gap_start = -1;
            }
        }

        return {max_gap_start, max_gap_end};
    }

    // Find the best point within the largest gap (furthest point in the gap)
    int find_best_point(const std::vector<float>& ranges, int gap_start, int gap_end)
    {
        int best_point_idx = gap_start;
        float max_range = 0.0;

        for (int i = gap_start; i <= gap_end; ++i) {
            if (ranges[i] > max_range) {
                max_range = ranges[i];
                best_point_idx = i;
            }
        }

        return best_point_idx;
    }

    // LiDAR callback function
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        // Copy LiDAR ranges
        std::vector<float> ranges = scan_msg->ranges;

        // Preprocess LiDAR data
        preprocess_lidar(ranges);

        // Find the closest point
        auto min_it = std::min_element(ranges.begin(), ranges.end());
        int closest_idx = std::distance(ranges.begin(), min_it);
        float closest_range = *min_it;

        // Clear points inside the safety bubble around the closest obstacle
        float angle_increment = scan_msg->angle_increment;
        float min_angle = scan_msg->angle_min + closest_idx * angle_increment;

        for (int i = 0; i < ranges.size(); ++i) {
            float angle = scan_msg->angle_min + i * angle_increment;
            if (std::abs(angle - min_angle) <= bubble_radius) {
                ranges[i] = 0.0; // mark within bubble as obstacle
            }
        }

        // Find the largest gap in the free space
        auto [gap_start, gap_end] = find_max_gap(ranges);

        // Find the best point in the largest gap
        int best_point_idx = find_best_point(ranges, gap_start, gap_end);

        // Compute the angle corresponding to the best point
        float goal_angle = scan_msg->angle_min + best_point_idx * angle_increment;

        // Publish the drive command (steering angle capped at ±24 degrees)
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.speed = 1.0; // set constant speed
        drive_msg.drive.steering_angle = std::clamp(static_cast<float>(goal_angle * 180.0 / M_PI), -max_steering_angle, max_steering_angle); // convert radians to degrees and clamp

        // Log speed and steering angle
        RCLCPP_INFO(this->get_logger(), "Speed: %.2f, Steering Angle: %.2f degrees", drive_msg.drive.speed, drive_msg.drive.steering_angle);

        // Publish the drive message
        drive_publisher_->publish(drive_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
