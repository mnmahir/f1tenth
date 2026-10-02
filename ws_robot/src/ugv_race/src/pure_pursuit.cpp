/*
Modified version of the pure_pursuit.cpp file from https://github.com/CL2-UWaterloo/f1tenth_ws c20cf63d04b9841ffdb6b2f963bd737d78074136
*/

#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>
#include <Eigen/Eigen>
#include <chrono>
#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit() : Node("pure_pursuit_node")
    {   
        // Boolean
        this->declare_parameter("read_frame_pose", true);
        this->declare_parameter("reverse_waypoints_order", false);
        read_frame_pose = this->get_parameter("read_frame_pose").as_bool();
        reverse_waypoints_order = this->get_parameter("reverse_waypoints_order").as_bool();

        // Odometry
        this->declare_parameter("odom_topic", "/odom");
        odom_topic = this->get_parameter("odom_topic").as_string();
        
        // Frames
        this->declare_parameter("base_frame", "base_footprint");
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("read_frame_pose_rate", 50);     // Rate lookup transform between base_frame and map_frame
        base_frame = this->get_parameter("base_frame").as_string();
        map_frame = this->get_parameter("map_frame").as_string();
        read_frame_pose_rate = this->get_parameter("read_frame_pose_rate").as_int();
        
        // Waypoints
        this->declare_parameter("waypoints_path", "waypoint.csv");
        this->declare_parameter("rviz_current_waypoint_topic", "/current_waypoint");
        this->declare_parameter("rviz_lookahead_waypoint_topic", "/lookahead_waypoint");
        waypoints_path = this->get_parameter("waypoints_path").as_string();
        // Relative paths are relative to this package's share directory, e.g. "racelines/f1tenth_route.csv"
        if (std::filesystem::path(waypoints_path).is_relative()) {
            waypoints_path = ament_index_cpp::get_package_share_directory("ugv_race") + "/" + waypoints_path;
        }
        rviz_current_waypoint_topic = this->get_parameter("rviz_current_waypoint_topic").as_string();
        rviz_lookahead_waypoint_topic = this->get_parameter("rviz_lookahead_waypoint_topic").as_string();

        // Control
        this->declare_parameter("min_lookahead", 0.5);
        this->declare_parameter("max_lookahead", 1.0);
        this->declare_parameter("lookahead_ratio", 8.0);
        this->declare_parameter("k_p", 0.5);
        this->declare_parameter("steering_limit", 25.0);
        this->declare_parameter("velocity_percentage", 0.6);
        this->declare_parameter("drive_topic", "/drive");
        min_lookahead = this->get_parameter("min_lookahead").as_double();
        max_lookahead = this->get_parameter("max_lookahead").as_double();
        lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
        k_p = this->get_parameter("k_p").as_double();
        steering_limit = this->get_parameter("steering_limit").as_double();
        velocity_percentage = this->get_parameter("velocity_percentage").as_double();
        drive_topic = this->get_parameter("drive_topic").as_string();

        // Topic Publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);
        vis_current_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(rviz_current_waypoint_topic, 10);
        vis_lookahead_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(rviz_lookahead_waypoint_topic, 10);

        // Topic Subscriptions
        if (!read_frame_pose) {
            RCLCPP_INFO(this->get_logger(), "Using odom topic for pose");
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 25, std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));
        }
        // Callback Timer
        dyn_conf_timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&PurePursuit::dyn_conf_timer_callback, this));
        if (read_frame_pose) {
            RCLCPP_INFO(this->get_logger(), "Using frame lookup for pose");
            get_pose_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/read_frame_pose_rate), std::bind(&PurePursuit::get_pose_on_map, this));
        }
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        load_waypoints();

        RCLCPP_INFO(this->get_logger(), "Pure pursuit node started");
        RCLCPP_INFO(this->get_logger(), "Using waypoint: %s", waypoints_path.c_str());
    }

    // void pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr &pose_msg)
    // {
    //     // TODO: find the current waypoint to track using methods mentioned in lecture

    //     // TODO: transform goal point to vehicle frame of reference

    //     // TODO: calculate curvature/steering angle

    //     // TODO: publish drive message, don't forget to limit the steering angle.
    // }

    ~PurePursuit() {}   

private:
    void load_waypoints()
    {
        csvFile_waypoints.open(waypoints_path, std::ios::in);

        if (!csvFile_waypoints.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", waypoints_path.c_str());
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "CSV File Opened");
        }

        // std::vector<std::string> row;
        std::string line, word, temp;

        while (!csvFile_waypoints.eof())
        {
            std::getline(csvFile_waypoints, line, '\n');
            std::stringstream s(line);

            int j = 0;
            while (getline(s, word, ','))
            {
                if (!word.empty())
                {
                    if (j == 0)
                    {
                        waypoints.X.push_back(std::stod(word));
                    }
                    else if (j == 1)
                    {
                        waypoints.Y.push_back(std::stod(word));
                    }
                    else if (j == 2)
                    {
                        waypoints.V.push_back(std::stod(word));
                    }
                }
                j++;
            }
        }

        csvFile_waypoints.close();
        num_waypoints = waypoints.X.size();
        RCLCPP_INFO(this->get_logger(), "Finished loading %d waypoints from %s", num_waypoints, waypoints_path.c_str());

        // reverse the order of waypoints
        if (reverse_waypoints_order)
        {
            std::reverse(waypoints.X.begin(), waypoints.X.end());
            std::reverse(waypoints.Y.begin(), waypoints.Y.end());
            std::reverse(waypoints.V.begin(), waypoints.V.end());
        }

        double average_dist_between_waypoints = 0.0;
        for (int i = 0; i < num_waypoints - 1; i++)
        {
            average_dist_between_waypoints += p2pdist(waypoints.X[i], waypoints.X[i + 1], waypoints.Y[i], waypoints.Y[i + 1]);
        }
        average_dist_between_waypoints /= num_waypoints;
        RCLCPP_INFO(this->get_logger(), "Average distance between waypoints: %f", average_dist_between_waypoints);
    }

    

    void get_waypoint()
    {
        // Main logic: Search within the next 500 points
        double longest_distance = 0;
        int final_i = -1;
        int start = waypoints.index;
        int end = (waypoints.index + 50) % num_waypoints;

        // Lookahead needs to be between the min_lookhead and the max_lookahead
        double lookahead = std::min(std::max(min_lookahead, max_lookahead * curr_velocity / lookahead_ratio), max_lookahead);

        if (end < start)
        { // If we need to loop around
            for (int i = start; i < num_waypoints; i++)
            {
                if (p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y) <= lookahead && p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y) >= longest_distance)
                {
                    longest_distance = p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y);
                    final_i = i;
                }
            }
            for (int i = 0; i < end; i++)
            {
                if (p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y) <= lookahead && p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y) >= longest_distance)
                {
                    longest_distance = p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y);
                    final_i = i;
                }
            }
        }
        else
        {
            for (int i = start; i < end; i++)
            {
                if (p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y) <= lookahead && p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y) >= longest_distance)
                {
                    longest_distance = p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y);
                    final_i = i;
                }
            }
        }

        if (final_i == -1)
        { // if we haven't found anything, search from the beginning
            final_i = 0;
            for (int i = 0; i < num_waypoints; i++)
            {
                if (p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y) <= lookahead && p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y) >= longest_distance)
                {
                    longest_distance = p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y);
                    final_i = i;
                }
            }
        }

        // Find the closest point to the car, and use the velocity index for that
        double shortest_distance = p2pdist(waypoints.X[0], base_pose_x, waypoints.Y[0], base_pose_y);
        int velocity_i = 0;
        for (int i = 0; i < num_waypoints; i++)
        {
            if (p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y) <= shortest_distance)
            {
                shortest_distance = p2pdist(waypoints.X[i], base_pose_x, waypoints.Y[i], base_pose_y);
                velocity_i = i;
            }
        }

        // If a waypoint is not found within our radius, then waypoints.index = 0
        waypoints.index = final_i;
        waypoints.velocity_index = velocity_i;
    }

    

    void transformandinterp_waypoint()
    { // pass old waypoint here
        // initialise vectors
        waypoints.lookahead_point_world << waypoints.X[waypoints.index], waypoints.Y[waypoints.index], 0.0;
        waypoints.current_point_world << waypoints.X[waypoints.velocity_index], waypoints.Y[waypoints.velocity_index], 0.0;

        visualize_lookahead_point(waypoints.lookahead_point_world);
        visualize_current_point(waypoints.current_point_world);

        // look up transformation at that instant from tf_buffer_
        geometry_msgs::msg::TransformStamped transformStamped;

        try
        {
            // Get the transform from the base_link reference to world reference frame
            transformStamped = tf_buffer_->lookupTransform(base_frame, map_frame, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform. Error: %s", ex.what());
        }

        // transform points (rotate first and then translate)
        Eigen::Vector3d translation_v(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
        quat_to_rot(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

        waypoints.lookahead_point_car = (rotation_m * waypoints.lookahead_point_world) + translation_v;
    }

    double p_controller()
    {
        double r = waypoints.lookahead_point_car.norm(); // r = sqrt(x^2 + y^2)
        double y = waypoints.lookahead_point_car(1);
        double angle = k_p * 2 * y / pow(r, 2); // Calculated from https://docs.google.com/presentation/d/1jpnlQ7ysygTPCi8dmyZjooqzxNXWqMgO31ZhcOlKVOE/edit#slide=id.g63d5f5680f_0_33

        return angle;
    }

    double get_velocity(double steering_angle)
    {
        double velocity = 0;

        if (waypoints.V[waypoints.velocity_index])
        {
            velocity = waypoints.V[waypoints.velocity_index] * velocity_percentage;
        }
        else
        { // For waypoints loaded without velocity profiles
            if (abs(steering_angle) >= to_radians(0.0) && abs(steering_angle) < to_radians(10.0))
            {
                velocity = 6.0 * velocity_percentage;
            }
            else if (abs(steering_angle) >= to_radians(10.0) && abs(steering_angle) <= to_radians(20.0))
            {
                velocity = 2.5 * velocity_percentage;
            }
            else
            {
                velocity = 2.0 * velocity_percentage;
            }
        }

        return velocity;
    }

    void publish_drive(double steering_angle)
    {
        auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
        if (steering_angle < 0.0)
        {
            drive_msgObj.drive.steering_angle = std::max(steering_angle, -to_radians(steering_limit)); // ensure steering angle is dynamically capable
        }
        else
        {
            drive_msgObj.drive.steering_angle = std::min(steering_angle, to_radians(steering_limit)); // ensure steering angle is dynamically capable
        }

        curr_velocity = get_velocity(drive_msgObj.drive.steering_angle);
        drive_msgObj.drive.speed = curr_velocity;

        RCLCPP_INFO(this->get_logger(), "Idx: %d | Distance: %.2fm | Speed: %.2fm/s | Steering angle: %.2f", waypoints.index, p2pdist(waypoints.X[waypoints.index], base_pose_x, waypoints.Y[waypoints.index], base_pose_y), drive_msgObj.drive.speed, to_degrees(drive_msgObj.drive.steering_angle));

        drive_pub_->publish(drive_msgObj);
    }

    // Pose from odom callback
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj)
    {
        // log the this function is running
        base_pose_x = odom_submsgObj->pose.pose.position.x;
        base_pose_y = odom_submsgObj->pose.pose.position.y;
        // interpolate between different way-points
        get_waypoint();

        // use tf2 transform the goal point
        transformandinterp_waypoint();

        // Calculate curvature/steering angle
        double steering_angle = p_controller();

        // publish object and message: AckermannDriveStamped on drive topic
        publish_drive(steering_angle);
    }

    // Pose from frame lookup
    void get_pose_on_map()
    {
        // Get the current pose of the car in the world frame
        geometry_msgs::msg::TransformStamped transformStamped;

        try
        {
            transformStamped = tf_buffer_->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform. Error: %s", ex.what());
        }

        base_pose_x = transformStamped.transform.translation.x;
        base_pose_y = transformStamped.transform.translation.y;

        get_waypoint();

        // use tf2 transform the goal point
        transformandinterp_waypoint();

        // Calculate curvature/steering angle
        double steering_angle = p_controller();

        // publish object and message: AckermannDriveStamped on drive topic
        publish_drive(steering_angle);
    }


    // Visualisation functions
    void visualize_lookahead_point(Eigen::Vector3d &point)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.281;
        marker.scale.y = 0.281;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.b = 1.0;

        marker.pose.position.x = point(0);
        marker.pose.position.y = point(1);
        marker.id = 1;
        vis_lookahead_point_pub_->publish(marker);
    }

    void visualize_current_point(Eigen::Vector3d &point)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.281;
        marker.scale.y = 0.281;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.b = 1.0;
        marker.color.r = 1.0;

        marker.pose.position.x = point(0);
        marker.pose.position.y = point(1);
        marker.id = 1;
        vis_current_point_pub_->publish(marker);
    }
    

    // Utility functions
    double to_radians(double degrees)
    {
        double radians;
        return radians = degrees * M_PI / 180.0;
    }

    double to_degrees(double radians)
    {
        double degrees;
        return degrees = radians * 180.0 / M_PI;
    }

    double p2pdist(double &x1, double &x2, double &y1, double &y2)
    {
        double dist = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
        return dist;
    }

    void quat_to_rot(double q0, double q1, double q2, double q3)
    {
        double r00 = (double)(2.0 * (q0 * q0 + q1 * q1) - 1.0);
        double r01 = (double)(2.0 * (q1 * q2 - q0 * q3));
        double r02 = (double)(2.0 * (q1 * q3 + q0 * q2));

        double r10 = (double)(2.0 * (q1 * q2 + q0 * q3));
        double r11 = (double)(2.0 * (q0 * q0 + q2 * q2) - 1.0);
        double r12 = (double)(2.0 * (q2 * q3 - q0 * q1));

        double r20 = (double)(2.0 * (q1 * q3 - q0 * q2));
        double r21 = (double)(2.0 * (q2 * q3 + q0 * q1));
        double r22 = (double)(2.0 * (q0 * q0 + q3 * q3) - 1.0);

        rotation_m << r00, r01, r02, r10, r11, r12, r20, r21, r22;
    }

    void dyn_conf_timer_callback()
    {
        // Periodically check parameters and update for dynamic reconfigure
        k_p = this->get_parameter("k_p").as_double();
        velocity_percentage = this->get_parameter("velocity_percentage").as_double();
        min_lookahead = this->get_parameter("min_lookahead").as_double();
        max_lookahead = this->get_parameter("max_lookahead").as_double();
        lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
        steering_limit = this->get_parameter("steering_limit").as_double();
    }


    // Initialise variables
    // global static (to be shared by all objects) and dynamic variables (each instance gets its own copy -> managed on the stack)
    struct csvFileData {
        std::vector<double> X;
        std::vector<double> Y;
        std::vector<double> V;

        int index;
        int velocity_index;

        Eigen::Vector3d lookahead_point_world;  // from world reference frame (usually `map`)
        Eigen::Vector3d lookahead_point_car;    // from car reference frame
        Eigen::Vector3d current_point_world;    // Locks on to the closest waypoint, which gives a velocity profile
    };
    
    Eigen::Matrix3d rotation_m;
    
    double base_pose_x;     // Position of the car in the world frame
    double base_pose_y;

    std::string odom_topic;
    std::string base_frame;
    std::string drive_topic;
    std::string map_frame;
    std::string rviz_current_waypoint_topic;
    std::string rviz_lookahead_waypoint_topic;
    std::string waypoints_path;
    double k_p;
    double min_lookahead;
    double max_lookahead;
    double lookahead_ratio;
    double steering_limit;
    double velocity_percentage;
    double curr_velocity = 0.0;
    int16_t read_frame_pose_rate;
    bool read_frame_pose;
    bool reverse_waypoints_order;


    // file object
    std::fstream csvFile_waypoints;

    // struct initialisation
    csvFileData waypoints;
    int num_waypoints;

    // Timer initialisation
    rclcpp::TimerBase::SharedPtr dyn_conf_timer_;
    rclcpp::TimerBase::SharedPtr get_pose_timer_;

    // declare subscriber sharedpointer obj
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // declare publisher sharedpointer obj
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_current_point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_lookahead_point_pub_;

    // declare tf shared pointers
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}