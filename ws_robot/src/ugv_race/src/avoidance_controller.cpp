#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
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
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr best_point_marker_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr visualize_avoidance_boundary_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr visualize_scan_rejection_boundary_;

    // timer initialisation
    rclcpp::TimerBase::SharedPtr dyn_conf_timer_;

    // variables
    double max_steering_angle_;

    // gap finding parameters
    double bubble_radius_;
    double scan_rejection_distance_;
    double angle_increment_;
    double angle_min_;
    double lidar_ray_fov_;

    // drive message callback
    double original_steering_angle_;
    double side_avoidance_offset_steering_angle_;
    double front_avoidance_offset_steering_angle_;

    // wall parameters
    double side_wall_width_;
    double side_wall_length_;
    double side_wall_length_offset_;
    double front_rectangle_wall_width_;
    double front_rectangle_wall_length_;
    double front_rectangle_wall_distance_offset_;

    std::string scan_frame_id_;

    void visualize_avoidance_boundary()
    {
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

        // Define the front rectangle wall points
        std::vector<std::vector<double>> front_rectangle_wall_points = {
            {front_rectangle_wall_distance_offset_, -front_rectangle_wall_width_ / 2, 0.0},
            {front_rectangle_wall_distance_offset_ + front_rectangle_wall_length_, 0.0, 0.0},
            {front_rectangle_wall_distance_offset_, front_rectangle_wall_width_ / 2, 0.0},
            {front_rectangle_wall_distance_offset_, -front_rectangle_wall_width_ / 2, 0.0}};

        // Add the side wall points to the avoidance boundary
        for (const auto &point : side_wall_points)
        {
            geometry_msgs::msg::Point32 p;
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            avoidance_boundary.polygon.points.push_back(p);
        }

        // Add the front rectangle wall points to the avoidance boundary
        for (const auto &point : front_rectangle_wall_points)
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

    void visualize_scan_rejection_boundary()
    {
        // Visualize the pie boundary from the LiDAR based on the scan rejection distance and FOV
        auto boundary_msg = geometry_msgs::msg::PolygonStamped();
        boundary_msg.header.frame_id = scan_frame_id_;
        boundary_msg.header.stamp = this->now();

        // Create a pie boundary with points
        int num_points = 360; // Number of points to represent the pie
        double angle_increment = lidar_ray_fov_ / num_points;

        // Add the center point of the pie
        geometry_msgs::msg::Point32 center_point;
        center_point.x = 0.0;
        center_point.y = 0.0;
        center_point.z = 0.0;
        boundary_msg.polygon.points.push_back(center_point);

        // Add the boundary points of the larger pie
        for (int i = 0; i <= num_points; ++i)
        {
            geometry_msgs::msg::Point32 point;
            double angle = -lidar_ray_fov_ / 2 + i * angle_increment;
            point.x = scan_rejection_distance_ * std::cos(angle);
            point.y = scan_rejection_distance_ * std::sin(angle);
            point.z = 0.0;
            boundary_msg.polygon.points.push_back(point);
        }

        // Add the last point to close the larger pie shape
        boundary_msg.polygon.points.push_back(center_point);

        // Add the boundary points of the smaller pie
        double smaller_pie_radius = front_rectangle_wall_length_ + front_rectangle_wall_distance_offset_;
        for (int i = 0; i <= num_points; ++i)
        {
            geometry_msgs::msg::Point32 point;
            double angle = -lidar_ray_fov_ / 2 + i * angle_increment;
            point.x = smaller_pie_radius * std::cos(angle);
            point.y = smaller_pie_radius * std::sin(angle);
            point.z = 0.0;
            boundary_msg.polygon.points.push_back(point);
        }

        // Add the last point to close the smaller pie shape
        boundary_msg.polygon.points.push_back(center_point);

        visualize_scan_rejection_boundary_->publish(boundary_msg);
    }

    void publish_best_point_marker(int best_point_idx, const std::vector<double> &ranges, const std::vector<double> &angles)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = scan_frame_id_;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "best_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        double angle = angles[best_point_idx];
        marker.pose.position.x = (front_rectangle_wall_length_ + front_rectangle_wall_distance_offset_) * std::cos(angle);
        marker.pose.position.y = (front_rectangle_wall_length_ + front_rectangle_wall_distance_offset_) * std::sin(angle);
        marker.pose.position.z = 0.0;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        best_point_marker_pub_->publish(marker);
    }

    // Preprocess LiDAR data
    void preprocess_lidar(std::vector<double> &ranges, const std::vector<double> &angles)
    {
        // Remove points outside the LiDAR field of view
        for (int i = 0; i < ranges.size(); ++i)
        {
            if (std::abs(angles[i]) > lidar_ray_fov_ / 2)
            {
                ranges[i] = 0.0;
            }
        }
        // Set high range values to zero to reject far points
        for (auto &range : ranges)
        {
            if (range > scan_rejection_distance_)
            {
                range = scan_rejection_distance_ - 0.1;
            }
        }
    }

    // Find the start and end indices of the largest gap in the LiDAR scan data
    std::pair<int, int> find_max_gap(const std::vector<double> &ranges)
    {
        int max_gap_start = -1;
        int max_gap_end = -1;
        int gap_start = -1;
        int gap_end = -1;

        for (int i = 0; i < ranges.size(); ++i)
        {
            if (ranges[i] > 0)
            {
                if (gap_start == -1)
                {
                    gap_start = i;
                }
                gap_end = i;
            }
            else
            {
                if (gap_start != -1)
                {
                    if (gap_end - gap_start > max_gap_end - max_gap_start)
                    {
                        max_gap_start = gap_start;
                        max_gap_end = gap_end;
                    }
                    gap_start = -1;
                }
            }
        }

        // Check the last gap
        if (gap_start != -1 && gap_end - gap_start > max_gap_end - max_gap_start)
        {
            max_gap_start = gap_start;
            max_gap_end = gap_end;
        }

        // RCLCPP_INFO(this->get_logger(), "Max gap start: %d, end: %d", max_gap_start, max_gap_end);

        return {max_gap_start, max_gap_end};
    }

    // Find the best point within the largest and furthest point
    int find_best_point(const std::vector<double> &ranges, int gap_start, int gap_end)
    {
        int mid_idx = (gap_start + gap_end) / 2;
        double left_total_distance = 0.0;
        double right_total_distance = 0.0;

        // Calculate total distances for left and right sides
        for (int i = gap_start; i <= mid_idx; ++i)
        {
            left_total_distance += ranges[i];
        }
        for (int i = mid_idx + 1; i <= gap_end; ++i)
        {
            right_total_distance += ranges[i];
        }

        // Determine the side with the greater total distance
        int best_point_idx = gap_start;
        double max_range = 0.0;

        if (left_total_distance > right_total_distance)
        {
            // Choose the best point from the left side
            for (int i = gap_start; i <= mid_idx; ++i)
            {
                if (ranges[i] > max_range)
                {
                    max_range = ranges[i];
                    best_point_idx = i;
                }
            }
        }
        else
        {
            // Choose the best point from the right side
            for (int i = mid_idx + 1; i <= gap_end; ++i)
            {
                if (ranges[i] > max_range)
                {
                    max_range = ranges[i];
                    best_point_idx = i;
                }
            }
        }

        return best_point_idx;
    }

    void front_wall_avoidance_rect(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> ranges, double angle_increment, double angle_min, const std::vector<double> &ray_angle)
    {
        bool obstacle_exist = false; // Any obstacle in the boundary?

        // Define the boundaries for the front rectangle wall
        double front_rectangle_wall_min_x = front_rectangle_wall_distance_offset_;
        double front_rectangle_wall_max_x = front_rectangle_wall_distance_offset_ + front_rectangle_wall_length_;
        double front_rectangle_wall_half_width = front_rectangle_wall_width_ / 2;

        // Turning factor
        double closest_obstacle_x = front_rectangle_wall_max_x;
        double turning_factor = 0.0;

        // Obstacle left and right space
        bool obs_left_space = false;
        bool obs_right_space = false;
        double obs_left_closest_x = front_rectangle_wall_max_x;
        double obs_right_closest_x = front_rectangle_wall_max_x;

        // Iterate through the scan points and check if any points fall within the rectangular boundaries
        for (size_t i = 0; i < x.size(); ++i)
        {
            if (x[i] >= front_rectangle_wall_min_x && x[i] <= front_rectangle_wall_max_x)
            {
                if (y[i] >= -front_rectangle_wall_half_width && y[i] <= front_rectangle_wall_half_width)
                {
                    obstacle_exist = true;
                    if (x[i] < closest_obstacle_x)
                    {
                        closest_obstacle_x = x[i];
                    }
                    // RCLCPP_INFO(this->get_logger(), "Obstacle at x: %f y: %f", x[i], y[i]);
                    if (y[i] > 0 && x[i] < (front_rectangle_wall_distance_offset_ + front_rectangle_wall_length_) / 2)
                    {
                        obs_left_space = true;
                        if (x[i] < obs_left_closest_x)
                        {
                            obs_left_closest_x = x[i];
                        }
                    }
                    if (y[i] < 0 && x[i] < (front_rectangle_wall_distance_offset_ + front_rectangle_wall_length_) / 2)
                    {
                        obs_right_space = true;
                        if (x[i] < obs_right_closest_x)
                        {
                            obs_right_closest_x = x[i];
                        }
                    }
                }
            }
        }
        turning_factor = (front_rectangle_wall_length_ - (closest_obstacle_x - front_rectangle_wall_min_x)) / front_rectangle_wall_length_;

        if (obstacle_exist && !obs_left_space && !obs_right_space)
        // if (obstacle_exist)
        {
            // Preprocess LiDAR data
            preprocess_lidar(ranges, ray_angle);

            // Find closest point to LiDAR
            auto min_it = std::min_element(ranges.begin(), ranges.end());
            int closest_idx = std::distance(ranges.begin(), min_it);
            double closest_range = *min_it;

            // Eliminate all points inside 'bubble' (set them to zero)
            double angle_increment = angle_increment_;
            double min_angle = angle_min_ + closest_idx * angle_increment;

            for (int i = 0; i < ranges.size(); ++i)
            {
                double angle = angle_min_ + i * angle_increment;
                if (std::abs(angle - min_angle) <= bubble_radius_)
                {
                    ranges[i] = 0.0;
                }
            }

            // Find max length gap in the LiDAR scan data
            auto [gap_start, gap_end] = find_max_gap(ranges);

            // Find the best point in the gap
            int best_point_idx = find_best_point(ranges, gap_start, gap_end);
            publish_best_point_marker(best_point_idx, ranges, ray_angle);

            // If the best point is on the left side of the vehicle, steer left and vice versa
            if (best_point_idx < ranges.size() / 2)
            {
                front_avoidance_offset_steering_angle_ = -max_steering_angle_ * turning_factor * 2;
                RCLCPP_INFO(this->get_logger(), "STEER \033[0;33mRIGHT!, \033[0moffset: %f", front_avoidance_offset_steering_angle_);
            }
            else
            {
                front_avoidance_offset_steering_angle_ = max_steering_angle_ * turning_factor * 2;
                RCLCPP_INFO(this->get_logger(), "STEER \033[0;33mLEFT!, \033[0moffset: %f", front_avoidance_offset_steering_angle_);
            }
        }
        else if (obs_left_closest_x < obs_right_closest_x)
        {
            front_avoidance_offset_steering_angle_ = -max_steering_angle_ * 2;
            RCLCPP_INFO(this->get_logger(), "FORCE STEER \033[0;33mRIGHT!, \033[0moffset: %f", front_avoidance_offset_steering_angle_);
        }
        else if (obs_right_closest_x < obs_left_closest_x)
        {
            front_avoidance_offset_steering_angle_ = max_steering_angle_ * 2;
            RCLCPP_INFO(this->get_logger(), "FORCE STEER \033[0;33mLEFT!, \033[0moffset: %f", front_avoidance_offset_steering_angle_);
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
        // Get info from the scan message
        scan_frame_id_ = scan_msg->header.frame_id;
        angle_increment_ = scan_msg->angle_increment;
        angle_min_ = scan_msg->angle_min;
        // float max_angle = scan_msg->angle_max;

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
        // front_wall_avoidance(x, y);
        front_wall_avoidance_rect(x, y, ray_range, angle_increment_, angle_min_, ray_angle);

        // Check if there is any obstacle on the side of the vehicle
        side_wall_avoidance(x, y);

        // Visualize the avoidance boundary
        visualize_avoidance_boundary();
        visualize_scan_rejection_boundary();
    }

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr drive_msg)
    {
        auto modified_drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        modified_drive_msg = *drive_msg;

        // Get the original steering angle
        original_steering_angle_ = drive_msg->drive.steering_angle;

        // Modify the steering angle
        if (drive_msg->drive.speed >= 0)
        {
            original_steering_angle_ = std::clamp(original_steering_angle_ + front_avoidance_offset_steering_angle_, -max_steering_angle_, max_steering_angle_);
            modified_drive_msg.drive.steering_angle = std::clamp(original_steering_angle_ + side_avoidance_offset_steering_angle_, -max_steering_angle_, max_steering_angle_);
        }
        else
        {
            modified_drive_msg.drive.steering_angle = original_steering_angle_;
        }

        // Publish the modified drive message
        drive_pub_->publish(modified_drive_msg);
    }

    void dyn_conf_timer_callback()
    {
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double() * M_PI / 180.0;
        side_wall_width_ = this->get_parameter("side_wall_width").as_double();
        side_wall_length_ = this->get_parameter("side_wall_length").as_double();
        side_wall_length_offset_ = this->get_parameter("side_wall_length_offset").as_double();
        front_rectangle_wall_width_ = this->get_parameter("front_rectangle_wall_width").as_double();
        front_rectangle_wall_length_ = this->get_parameter("front_rectangle_wall_length").as_double();
        front_rectangle_wall_distance_offset_ = this->get_parameter("front_rectangle_wall_distance_offset").as_double();
    }

public:
    AvoidanceController() : Node("avoidance_controller")
    {
        // Declare parameters
        this->declare_parameter("scan_topic", "/scan");
        this->declare_parameter("drive_topic_sub", "/ackermann_cmd");
        this->declare_parameter("drive_topic_pub", "/ackermann_cmd_filtered");
        this->declare_parameter("max_steering_angle", 22.9);                  // maximum steering angle (degrees)
        this->declare_parameter("bubble_radius", 0.281);                      // radius of the safety bubble around the closest obstacle
        this->declare_parameter("scan_rejection_distance", 3.0);              // reject far points (meters)
        this->declare_parameter("lidar_ray_fov", 180.0);                      // width of the side wall from LiDAR
        this->declare_parameter("side_wall_width", 0.35);                     // width of the side wall from LiDAR
        this->declare_parameter("side_wall_length", 0.424);                   // length of the side wall
        this->declare_parameter("side_wall_length_offset", -0.324);           // distance from y axis of lidar
        this->declare_parameter("front_rectangle_wall_width", 0.35);          // rectangle wall width (from LiDAR)
        this->declare_parameter("front_rectangle_wall_length", 0.2324);       // rectangle wall length from lidar
        this->declare_parameter("front_rectangle_wall_distance_offset", 0.1); // distance from x axis of lidar

        this->declare_parameter("visualize_boundary_topic", "/avoidance_control_boundary");
        this->declare_parameter("visualize_scan_rejection_boundary_topic", "/scan_rejection_boundary");
        this->declare_parameter("visualize_best_point_topic", "/best_point_marker");

        // Initialize subscribers
        drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(this->get_parameter("drive_topic_sub").as_string(), 10, std::bind(&AvoidanceController::drive_callback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(this->get_parameter("scan_topic").as_string(), 10, std::bind(&AvoidanceController::scan_callback, this, std::placeholders::_1));

        // Initialize publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->get_parameter("drive_topic_pub").as_string(), 10);
        visualize_avoidance_boundary_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(this->get_parameter("visualize_boundary_topic").as_string(), 10);
        visualize_scan_rejection_boundary_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(this->get_parameter("visualize_scan_rejection_boundary_topic").as_string(), 10);
        best_point_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(this->get_parameter("visualize_best_point_topic").as_string(), 10);

        // Initialize variables
        original_steering_angle_ = 0.0;
        side_avoidance_offset_steering_angle_ = 0.0;
        front_avoidance_offset_steering_angle_ = 0.0;
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double() * M_PI / 180.0;
        bubble_radius_ = this->get_parameter("bubble_radius").as_double();
        lidar_ray_fov_ = this->get_parameter("lidar_ray_fov").as_double() * M_PI / 180.0;
        scan_rejection_distance_ = this->get_parameter("scan_rejection_distance").as_double();
        side_wall_width_ = this->get_parameter("side_wall_width").as_double();
        side_wall_length_ = this->get_parameter("side_wall_length").as_double();
        side_wall_length_offset_ = this->get_parameter("side_wall_length_offset").as_double();
        front_rectangle_wall_width_ = this->get_parameter("front_rectangle_wall_width").as_double();
        front_rectangle_wall_length_ = this->get_parameter("front_rectangle_wall_length").as_double();
        front_rectangle_wall_distance_offset_ = this->get_parameter("front_rectangle_wall_distance_offset").as_double();

        // Initialize dynamic configuration timer
        dyn_conf_timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&AvoidanceController::dyn_conf_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "AvoidanceController node has started.");
        RCLCPP_INFO(this->get_logger(), "max_steering_angle: \033[1;33m%f", max_steering_angle_);
        RCLCPP_INFO(this->get_logger(), "bubble_radius: \033[1;33m%f", bubble_radius_);
        RCLCPP_INFO(this->get_logger(), "scan_rejection_distance: \033[1;33m%f", scan_rejection_distance_);
        RCLCPP_INFO(this->get_logger(), "lidar_ray_fov: \033[1;33m%f", lidar_ray_fov_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "side_wall_width: \033[1;33m%f", side_wall_width_);
        RCLCPP_INFO(this->get_logger(), "side_wall_length: \033[1;33m%f", side_wall_length_);
        RCLCPP_INFO(this->get_logger(), "side_wall_length_offset: \033[1;33m%f", side_wall_length_offset_);
        RCLCPP_INFO(this->get_logger(), "front_rectangle_wall_width: \033[1;33m%f", front_rectangle_wall_width_);
        RCLCPP_INFO(this->get_logger(), "front_rectangle_wall_length: \033[1;33m%f", front_rectangle_wall_length_);
        RCLCPP_INFO(this->get_logger(), "front_rectangle_wall_distance_offset: \033[1;33m%f", front_rectangle_wall_distance_offset_);
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