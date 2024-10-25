/*
This node is for 2D LiDAR mounted at the center width of the vehicle.
It will republish the drive message with adjusted steering angle to avoid obstacles.
This node adds virtual fences to the vehicle to avoid obstacles.
When obstacle is detected on the side of the vehicle and steering angle direction, it will counter steer and car will go straight avoiding the car turning towards the obstacle.
When obstacle is detected in front of the vehicle, the triangle wall will steer the car away from the obstacle. Which direction the car will steer depends on the side of the obstacle. If both sides have obstacles, the car will steer towards the side with the most space using follow gap algorithm. After follow gap make the decision, it will lock the dicision of which side to steer until no more obstacles is detected.
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <cmath>
#include <vector>
#include <chrono>
#include <thread>
#include <memory>

class AvoidanceController : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr visualize_avoidance_boundary_;

    // timer initialisation
    rclcpp::TimerBase::SharedPtr dyn_conf_timer_;

    // variables
    double max_steering_angle_;

    // drive message callback
    double original_steering_angle_;
    double side_avoidance_offset_steering_angle_;
    double front_avoidance_offset_steering_angle_;

    // wall parameters
    double side_wall_width_;
    double side_wall_length_;
    double side_wall_length_offset_;
    double front_triangle_wall_width_;
    double front_triangle_wall_length_;
    double front_triangle_wall_distance_offset_;

    std::string scan_frame_id_;

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr drive_msg)
    {
        auto modified_drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        modified_drive_msg = *drive_msg;

        // Get the original steering angle
        original_steering_angle_ = drive_msg->drive.steering_angle;

        // Modify the steering angle
        if (drive_msg->drive.speed < 0)
        {
            modified_drive_msg.drive.steering_angle = std::clamp(original_steering_angle_ - front_avoidance_offset_steering_angle_, -max_steering_angle_, max_steering_angle_);
        } else {
            modified_drive_msg.drive.steering_angle = std::clamp(original_steering_angle_ + side_avoidance_offset_steering_angle_ + front_avoidance_offset_steering_angle_, -max_steering_angle_, max_steering_angle_);
        }
          
        // Publish the modified drive message
        drive_pub_->publish(modified_drive_msg);
    }

    void visualize_avoidance_boundary()
    {
        /*
        This function will visualize the avoidance boundary of the vehicle.
        The avoidance boundary will be a polygon that represents the side wall and the front triangle wall.
        */
        auto avoidance_boundary = geometry_msgs::msg::PolygonStamped();
        avoidance_boundary.header.frame_id = scan_frame_id_;
        avoidance_boundary.header.stamp = this->get_clock()->now();

        // Define the side wall points with offset
        std::vector<std::vector<double>> side_wall_points = {
            {side_wall_length_offset_, -side_wall_width_ / 2, 0.0},
            {side_wall_length_offset_ + side_wall_length_, -side_wall_width_ / 2, 0.0},
            {side_wall_length_offset_ + side_wall_length_, side_wall_width_ / 2, 0.0},
            {side_wall_length_offset_, side_wall_width_ / 2, 0.0},
            {side_wall_length_offset_, -side_wall_width_ / 2, 0.0}};

        // Define the front triangle wall points
        std::vector<std::vector<double>> front_triangle_wall_points = {
            {front_triangle_wall_distance_offset_, -front_triangle_wall_width_ / 2, 0.0},
            {front_triangle_wall_distance_offset_ + front_triangle_wall_length_, 0.0, 0.0},
            {front_triangle_wall_distance_offset_, front_triangle_wall_width_ / 2, 0.0},
            {front_triangle_wall_distance_offset_, -front_triangle_wall_width_ / 2, 0.0}};

        // Add the side wall points to the avoidance boundary
        for (const auto &point : side_wall_points)
        {
            geometry_msgs::msg::Point32 p;
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            avoidance_boundary.polygon.points.push_back(p);
        }

        // Add the front triangle wall points to the avoidance boundary
        for (const auto &point : front_triangle_wall_points)
        {
            geometry_msgs::msg::Point32 p;
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            avoidance_boundary.polygon.points.push_back(p);
        }

        // Publish the avoidance boundary
        visualize_avoidance_boundary_->publish(avoidance_boundary);
    }


    void front_wall_avoidance(const std::vector<double> &x, const std::vector<double> &y)
    {
        /*
        Front wall avoidance will check if there is any obstacle on the front left of the triangle and if there is, it will steer the car to the right. Same for the right side.
        */
        // Check if there is any obstacle in front of the vehicle
        bool left_obstacle = false;
        bool right_obstacle = false;

        // Define the boundaries for the front triangle wall
        double front_triangle_wall_min_x = front_triangle_wall_distance_offset_;
        double front_triangle_wall_max_x = front_triangle_wall_distance_offset_ + front_triangle_wall_length_;
        double front_triangle_wall_half_width = front_triangle_wall_width_ / 2;

        // Iterate through the scan points and check if any points fall within the triangular boundaries
        for (size_t i = 0; i < x.size(); ++i)
        {
            if (x[i] >= front_triangle_wall_min_x && x[i] <= front_triangle_wall_max_x)
            {
                double max_y = front_triangle_wall_half_width * (1 - (x[i] - front_triangle_wall_min_x) / (front_triangle_wall_max_x - front_triangle_wall_min_x));
                if (y[i] >= -max_y && y[i] <= max_y)
                {
                    if (y[i] > 0)
                    {
                        left_obstacle = true;
                    }
                    if (y[i] < 0)
                    {
                        right_obstacle = true;
                    }
                }
            }
        }
        // If both sides have obstacles, steer towards the side with the most space
        if (left_obstacle && right_obstacle)
        {
            // Count the number of points on the left and right side of the front triangle wall. The side with the most space will have the least number of points.
            long left_space = 0;
            long right_space = 0;
            for (size_t i = 0; i < x.size(); ++i)
            {
                if (x[i] >= front_triangle_wall_min_x && x[i] <= front_triangle_wall_max_x)
                {
                    double max_y = front_triangle_wall_half_width * (1 - (x[i] - front_triangle_wall_min_x) / (front_triangle_wall_max_x - front_triangle_wall_min_x));
                    if (y[i] >= -max_y && y[i] <= max_y)
                    {
                        if (y[i] > 0)
                        {
                            left_space += 1;
                        }
                        if (y[i] < 0)
                        {
                            right_space += 1;
                        }
                    }
                }
            }
            if (left_space > right_space)
            {
                front_avoidance_offset_steering_angle_ = -max_steering_angle_ * 2;
            }
            else
            {
                front_avoidance_offset_steering_angle_ = max_steering_angle_ * 2;
            }

        }
        else if (left_obstacle)
        {
            front_avoidance_offset_steering_angle_ = -max_steering_angle_ * 2;
        }
        else if (right_obstacle)
        {
            front_avoidance_offset_steering_angle_ = max_steering_angle_ * 2;
        }
        else
        {
            front_avoidance_offset_steering_angle_ = 0.0;
        }
    }

    void side_wall_avoidance(const std::vector<double> &x, const std::vector<double> &y)
    {
        /*
        Side wall avoidance will check if there is any obstacle on the left side of the vehicle and if the steering angle is towards the left side, it will make the car go straight. Same for the right side.
        */
        // Check if there is any obstacle on the side of the vehicle
        bool left_obstacle = false;
        bool right_obstacle = false;

        // Define the boundaries for the left and right side walls
        double left_boundary_min_x = side_wall_length_offset_;
        double left_boundary_max_x = side_wall_length_offset_ + side_wall_length_;
        double left_boundary_min_y = -side_wall_width_ / 2;
        double left_boundary_max_y = 0;

        double right_boundary_min_x = side_wall_length_offset_;
        double right_boundary_max_x = side_wall_length_offset_ + side_wall_length_;
        double right_boundary_min_y = 0;
        double right_boundary_max_y = side_wall_width_ / 2;

        // Iterate through the scan points and check if any points fall within the boundaries
        for (size_t i = 0; i < x.size(); ++i)
        {
            if (x[i] >= left_boundary_min_x && x[i] <= left_boundary_max_x &&
                y[i] >= left_boundary_min_y && y[i] <= left_boundary_max_y)
            {
                right_obstacle = true;
            }

            if (x[i] >= right_boundary_min_x && x[i] <= right_boundary_max_x &&
                y[i] >= right_boundary_min_y && y[i] <= right_boundary_max_y)
            {
                left_obstacle = true;
            }
        }

        // Check the steering angle direction
        bool left_steering = original_steering_angle_ > 0;
        bool right_steering = original_steering_angle_ < 0;

        // If there is an obstacle on the left side and the steering angle is towards the left side, make the car go straight
        if ((left_obstacle && left_steering) || (right_obstacle && right_steering))
        {
            side_avoidance_offset_steering_angle_ = -original_steering_angle_;
        }
        else
        {
            side_avoidance_offset_steering_angle_ = 0.0;
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // Get the frame id of the scan
        scan_frame_id_ = scan_msg->header.frame_id;

        // Get the range and angle of the scan
        std::vector<double> ray_range(scan_msg->ranges.begin(), scan_msg->ranges.end());
        std::vector<double> ray_angle(ray_range.size());
        for (size_t i = 0; i < ray_range.size(); ++i)
        {
            ray_angle[i] = scan_msg->angle_min + i * (scan_msg->angle_max - scan_msg->angle_min) / (ray_range.size() - 1);
        }

        // Get the x and y coordinates of the scan
        std::vector<double> x(ray_range.size()), y(ray_range.size());
        for (size_t i = 0; i < ray_range.size(); ++i)
        {
            x[i] = ray_range[i] * std::cos(ray_angle[i]);
            y[i] = ray_range[i] * std::sin(ray_angle[i]);
        }

        // Check if there is any obstacle in front of the vehicle
        front_wall_avoidance(x, y);

        // Check if there is any obstacle on the side of the vehicle
        side_wall_avoidance(x, y);

        // Visualize the avoidance boundary
        visualize_avoidance_boundary();
    }

    void dyn_conf_timer_callback()
    {
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double() * M_PI / 180.0;
        side_wall_width_ = this->get_parameter("side_wall_width").as_double();
        side_wall_length_ = this->get_parameter("side_wall_length").as_double();
        side_wall_length_offset_ = this->get_parameter("side_wall_length_offset").as_double();
        front_triangle_wall_width_ = this->get_parameter("front_triangle_wall_width").as_double();
        front_triangle_wall_length_ = this->get_parameter("front_triangle_wall_length").as_double();
        front_triangle_wall_distance_offset_ = this->get_parameter("front_triangle_wall_distance_offset").as_double();
    }

public:
    AvoidanceController() : Node("avoidance_controller")
    {
        // Declare parameters
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("drive_topic_sub", "/ackermann_cmd");
        this->declare_parameter("drive_topic_pub", "/ackermann_cmd_filtered");
        this->declare_parameter("max_steering_angle", 22.9);                 // maximum steering angle (degrees)
        this->declare_parameter("side_wall_width", 0.35);                    // width of the side wall from LiDAR
        this->declare_parameter("side_wall_length", 0.424);                  // length of the side wall
        this->declare_parameter("side_wall_length_offset", -0.324);          // distance from y axis of lidar
        this->declare_parameter("front_triangle_wall_width", 0.35);          // triangle wall base width (from LiDAR)
        this->declare_parameter("front_triangle_wall_length", 0.2324);       // triangle wall length from lidar to tip
        this->declare_parameter("front_triangle_wall_distance_offset", 0.1); // distance from x axis of lidar

        this->declare_parameter("side_wall_avoidance_boundary_topic", "/side_wall_avoidance_boundary");

        // Initialize subscribers
        drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(this->get_parameter("drive_topic_sub").as_string(), 10, std::bind(&AvoidanceController::drive_callback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(this->get_parameter("scan_topic").as_string(), 10, std::bind(&AvoidanceController::scan_callback, this, std::placeholders::_1));

        // Initialize publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->get_parameter("drive_topic_pub").as_string(), 10);
        visualize_avoidance_boundary_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(this->get_parameter("side_wall_avoidance_boundary_topic").as_string(), 10);

        // Initialize variables
        original_steering_angle_ = 0.0;
        side_avoidance_offset_steering_angle_ = 0.0;
        front_avoidance_offset_steering_angle_ = 0.0;
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double() * M_PI / 180.0;
        side_wall_width_ = this->get_parameter("side_wall_width").as_double();
        side_wall_length_ = this->get_parameter("side_wall_length").as_double();
        side_wall_length_offset_ = this->get_parameter("side_wall_length_offset").as_double();
        front_triangle_wall_width_ = this->get_parameter("front_triangle_wall_width").as_double();
        front_triangle_wall_length_ = this->get_parameter("front_triangle_wall_length").as_double();
        front_triangle_wall_distance_offset_ = this->get_parameter("front_triangle_wall_distance_offset").as_double();

        // Initialize the dynamic configuration timer
        dyn_conf_timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&AvoidanceController::dyn_conf_timer_callback, this));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AvoidanceController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}