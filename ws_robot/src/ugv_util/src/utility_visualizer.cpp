#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

class UtilityVisualizer : public rclcpp::Node
{
public:
    UtilityVisualizer() : Node("utility_visualizer")
    {
        // Declare and get parameters
        this->declare_parameter("visualization_update_rate", 30);  // Hz
        
        this->declare_parameter<std::string>("base_frame", "base_footprint");
        this->declare_parameter<std::string>("laser_scan_frame", "lidar_1_link");

        this->declare_parameter<double>("wheelbase", 0.324);
        this->declare_parameter("ittc_foward_drive_scan_width", 0.3);
        this->declare_parameter("ittc_foward_drive_x_scan_offset", -0.3);
        this->declare_parameter("force_stop_rectangular_region", std::vector<double>{0.0, 0.15, -0.11, 0.11});

        this->declare_parameter<std::string>("viz_drive_topic", "/ackermann_cmd_filtered");

        this->declare_parameter<std::string>("trajectory_topic", "trajectory_marker");
        this->declare_parameter<std::string>("force_stop_boundary_topic", "safety/force_stop_boundary");
        this->declare_parameter<std::string>("ittc_stop_boundary_topic", "safety/ittc_stop_boundary");

        visualization_update_rate_ = this->get_parameter("visualization_update_rate").as_int();

        base_frame_ = this->get_parameter("base_frame").as_string();
        scan_frame_id_ = this->get_parameter("laser_scan_frame").as_string();

        wheelbase_ = this->get_parameter("wheelbase").as_double();
        ittc_foward_drive_scan_width_ = this->get_parameter("ittc_foward_drive_scan_width").as_double();
        ittc_foward_drive_x_scan_offset_ = this->get_parameter("ittc_foward_drive_x_scan_offset").as_double();
        auto force_stop_rect = this->get_parameter("force_stop_rectangular_region").as_double_array();
        fstop_rect_x_min_ = force_stop_rect[0];
        fstop_rect_x_max_ = force_stop_rect[1];
        fstop_rect_y_min_ = force_stop_rect[2];
        fstop_rect_y_max_ = force_stop_rect[3];


        // Create a subscriber to the car's state
        state_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(this->get_parameter("viz_drive_topic").as_string(), 10, std::bind(&UtilityVisualizer::stateCallback, this, std::placeholders::_1));

        // Create a publisher for the visualization
        trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(this->get_parameter("trajectory_topic").as_string(), 10);
        force_stop_boundary_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(this->get_parameter("force_stop_boundary_topic").as_string(), 10);
        force_ittc_stop_boundary_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(this->get_parameter("ittc_stop_boundary_topic").as_string(), 10);

        // Create a timer to publish the visualization
        publish_visualization_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / visualization_update_rate_), std::bind(&UtilityVisualizer::publish_visualization, this));

        RCLCPP_INFO(this->get_logger(), "UtilityVisualizer node has started.");
    }
    ~UtilityVisualizer()
    {
        RCLCPP_INFO(this->get_logger(), "UtilityVisualizer node has been terminated.");
    }

private:
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr state_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr force_stop_boundary_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr force_ittc_stop_boundary_pub_;
    rclcpp::TimerBase::SharedPtr publish_visualization_timer_;
    double wheelbase_;
    std::string base_frame_;
    std::string scan_frame_id_;
    int visualization_update_rate_;

    double ittc_foward_drive_scan_width_;
    double ittc_foward_drive_x_scan_offset_;
    double fstop_rect_x_min_;
    double fstop_rect_x_max_;
    double fstop_rect_y_min_;
    double fstop_rect_y_max_;

    double state_speed_ = 0.0;           // until the first command arrives
    double state_steering_angle_ = 0.0;
    

    void publish_trajectory()
    {
        // Create a marker for the trajectory
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_frame_;
        marker.header.stamp = this->now();
        marker.ns = "trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1; // Line width
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        // Calculate the trajectory points based on the car's state and parameters
        geometry_msgs::msg::Point p;
        double x = 0.0, y = 0.0, theta = 0.0;
        double time_interval = 0.1;                         // Time interval between points
        double total_distance = std::abs(state_speed_); // Total distance to be covered
        int num_points = std::isfinite(total_distance) ? std::min(static_cast<int>(total_distance / time_interval), 200) : 0;

        for (int i = 0; i < num_points; ++i)
        {
            double distance = state_speed_ * time_interval;
            x += distance * cos(theta);
            y += distance * sin(theta);
            if (state_speed_ > 0)
                theta += (state_steering_angle_ * time_interval) / wheelbase_;
            else
                theta -= (state_steering_angle_ * time_interval) / wheelbase_;
            p.x = x;
            p.y = y;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        // Publish the marker
        trajectory_pub_->publish(marker);
    }

    void publish_force_stop_boundary()
    {
        auto polygon = geometry_msgs::msg::PolygonStamped();
        polygon.header.frame_id = scan_frame_id_;
        polygon.header.stamp = this->now();

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
        polygon.header.frame_id = scan_frame_id_; // Fetch the frame_id from scan
        polygon.header.stamp = this->now();

        // Define the rectangular region points
        std::vector<std::vector<double>> points = {
            {ittc_foward_drive_x_scan_offset_, -(ittc_foward_drive_scan_width_ / 2), 0.0},
            {ittc_foward_drive_x_scan_offset_, (ittc_foward_drive_scan_width_ / 2), 0.0},
            {1.0, (ittc_foward_drive_scan_width_ / 2), 0.0},
            {ittc_foward_drive_x_scan_offset_, (ittc_foward_drive_scan_width_ / 2), 0.0},
            {ittc_foward_drive_x_scan_offset_, -(ittc_foward_drive_scan_width_ / 2), 0.0},
            {1.0, -(ittc_foward_drive_scan_width_ / 2), 0.0}};

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

    void publish_visualization()
    {
        publish_trajectory();
        publish_force_stop_boundary();
        publish_ittc_foward_drive_scan_boundary();
    }

    void stateCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        // speed and steering angle
        state_speed_ = msg->drive.speed;
        state_steering_angle_ = msg->drive.steering_angle;
    }

    void dyn_conf_timer_callback()
    {
        base_frame_ = this->get_parameter("base_frame").as_string();
        scan_frame_id_ = this->get_parameter("laser_scan_frame").as_string();

        wheelbase_ = this->get_parameter("wheelbase").as_double();
        ittc_foward_drive_scan_width_ = this->get_parameter("ittc_foward_drive_scan_width").as_double();
        ittc_foward_drive_x_scan_offset_ = this->get_parameter("ittc_foward_drive_x_scan_offset").as_double();
        auto force_stop_rect = this->get_parameter("force_stop_rectangular_region").as_double_array();
        fstop_rect_x_min_ = force_stop_rect[0];
        fstop_rect_x_max_ = force_stop_rect[1];
        fstop_rect_y_min_ = force_stop_rect[2];
        fstop_rect_y_max_ = force_stop_rect[3];
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UtilityVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}