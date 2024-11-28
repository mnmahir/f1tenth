#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <chrono>

class AvoidanceControllerVisualizer : public rclcpp::Node
{
public:
    AvoidanceControllerVisualizer() : Node("avoidance_controller_visualizer")
    {
        // Declare and get parameters
        this->declare_parameter("visualization_update_rate", 30); // Hz

        this->declare_parameter<std::string>("base_frame", "base_footprint");
        this->declare_parameter<std::string>("laser_scan_frame", "lidar_1_link");

        this->declare_parameter<double>("wheelbase", 0.324);
        this->declare_parameter("scan_rejection_distance", 3.0);
        this->declare_parameter("side_wall_width", 0.35);
        this->declare_parameter("side_wall_length", 0.424);
        this->declare_parameter("side_wall_length_offset", -0.324);
        this->declare_parameter("front_rectangle_wall_width", 0.35);
        this->declare_parameter("front_rectangle_wall_length", 0.2324);
        this->declare_parameter("front_rectangle_wall_distance_offset", 0.1);

        this->declare_parameter("drive_topic", "/ackermann_cmd");

        this->declare_parameter<std::string>("visualize_trajectory_topic", "control_trajectory");
        this->declare_parameter("visualize_boundary_topic", "/avoidance_control_boundary");
        this->declare_parameter("visualize_scan_rejection_boundary_topic", "/scan_rejection_boundary");

        visualization_update_rate_ = this->get_parameter("visualization_update_rate").as_int();

        base_frame_ = this->get_parameter("base_frame").as_string();
        scan_frame_id_ = this->get_parameter("laser_scan_frame").as_string();

        wheelbase_ = this->get_parameter("wheelbase").as_double();

        // Create a subscriber to the car's state
        state_subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(this->get_parameter("drive_topic").as_string(), 10, std::bind(&AvoidanceControllerVisualizer::stateCallback, this, std::placeholders::_1));

        // Create a publisher for the visualization
        visualize_avoidance_boundary_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(this->get_parameter("visualize_boundary_topic").as_string(), 10);
        visualize_scan_rejection_boundary_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(this->get_parameter("visualize_scan_rejection_boundary_topic").as_string(), 10);

        // Create a timer to publish the visualization
        publish_visualization_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / visualization_update_rate_), std::bind(&AvoidanceControllerVisualizer::publish_visualization, this));

        RCLCPP_INFO(this->get_logger(), "AvoidanceControllerVisualizer node has started.");
    }
    ~AvoidanceControllerVisualizer()
    {
        RCLCPP_INFO(this->get_logger(), "AvoidanceControllerVisualizer node has been terminated.");
    }

private:
    rclcpp::TimerBase::SharedPtr publish_visualization_timer_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr state_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr visualize_avoidance_boundary_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr visualize_scan_rejection_boundary_;

    double wheelbase_;
    std::string base_frame_;
    std::string scan_frame_id_;
    int visualization_update_rate_;

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
        // Visualize the circular boundary from the LiDAR based on the scan rejection distance
        auto boundary_msg = geometry_msgs::msg::PolygonStamped();
        boundary_msg.header.frame_id = scan_frame_id_;
        boundary_msg.header.stamp = this->now();

        // Create a circular boundary with points
        int num_points = 360; // Number of points to represent the circle
        double angle_increment = 2 * M_PI / num_points;

        for (int i = 0; i < num_points; ++i)
        {
            geometry_msgs::msg::Point32 point;
            double angle = i * angle_increment;
            point.x = scan_rejection_distance_ * std::cos(angle);
            point.y = scan_rejection_distance_ * std::sin(angle);
            point.z = 0.0;
            boundary_msg.polygon.points.push_back(point);
        }

        visualize_scan_rejection_boundary_->publish(boundary_msg);
    }

    void publish_visualization()
    {
    }

    void dyn_conf_timer_callback()
    {
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AvoidanceControllerVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}