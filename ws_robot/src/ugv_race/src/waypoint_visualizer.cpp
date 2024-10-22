/*
Modified version of the waypoint_visualizer.cpp file from https://github.com/CL2-UWaterloo/f1tenth_ws c20cf63d04b9841ffdb6b2f963bd737d78074136
*/
#include <math.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class WaypointVisualizer : public rclcpp::Node
{
public:
    WaypointVisualizer() : Node("waypoint_visualizer_node")
    {
        this->declare_parameter("waypoints_path", "waypoint.csv");
        this->declare_parameter("rviz_waypoints_line_topic", "/waypoints_line");
        this->declare_parameter("rviz_waypoints_arrow_topic", "/waypoints_arrow");
        this->declare_parameter("waypoint_line_width", 0.281);
        this->declare_parameter("waypoint_arrow_length", 0.15);
        this->declare_parameter("waypoint_arrow_width", 0.05);
        this->declare_parameter("waypoint_arrow_height", 0.03);
        this->declare_parameter<std::vector<double>>("waypoint_arrow_color", {1.0, 0.0, 0.0, 1.0});
        this->declare_parameter("reverse_waypoints_order", false);

        waypoints_path = this->get_parameter("waypoints_path").as_string();
        rviz_waypoints_line_topic = this->get_parameter("rviz_waypoints_line_topic").as_string();
        rviz_waypoints_arrow_topic = this->get_parameter("rviz_waypoints_arrow_topic").as_string();
        waypoint_line_width = this->get_parameter("waypoint_line_width").as_double();
        waypoint_arrow_length = this->get_parameter("waypoint_arrow_length").as_double();
        waypoint_arrow_width = this->get_parameter("waypoint_arrow_width").as_double();
        waypoint_arrow_height = this->get_parameter("waypoint_arrow_height").as_double();
        waypoint_arrow_color = this->get_parameter("waypoint_arrow_color").as_double_array();
        reverse_waypoints_order = this->get_parameter("reverse_waypoints_order").as_bool();

        vis_path_line_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(rviz_waypoints_line_topic, 1000);
        vis_path_arrow_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(rviz_waypoints_arrow_topic, 1000);
        visualize_waypoints_line_timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&WaypointVisualizer::visualize_waypoints_line, this));
        visualize_waypoints_arrow_timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&WaypointVisualizer::visualize_waypoints_arrow, this));

        RCLCPP_INFO(this->get_logger(), "this node has been launched");
        download_waypoints();
    }

private:
    void download_waypoints()
    {
        // Load waypoints from CSV
        csvFile_waypoints.open(waypoints_path, std::ios::in);

        RCLCPP_INFO(this->get_logger(), "%s", (csvFile_waypoints.is_open() ? "fileOpened" : "fileNOTopened"));

        std::string line, word;

        while (std::getline(csvFile_waypoints, line))
        {
            std::stringstream s(line);
            int j = 0;
            double x, y, v;

            while (getline(s, word, ','))
            {
                if (!word.empty())
                {
                    if (j == 0)
                    {
                        x = std::stod(word);
                    }
                    else if (j == 1)
                    {
                        y = std::stod(word);
                    }
                    else if (j == 2)
                    {
                        v = std::stod(word);
                    }
                }
                j++;
            }

            waypoints.X.push_back(x);
            waypoints.Y.push_back(y);
            waypoints.V.push_back(v);
        }

        csvFile_waypoints.close();

        // reverse the order of waypoints
        if (reverse_waypoints_order)
        {
            std::reverse(waypoints.X.begin(), waypoints.X.end());
            std::reverse(waypoints.Y.begin(), waypoints.Y.end());
            std::reverse(waypoints.V.begin(), waypoints.V.end());
        }
    }

    // vizualize waypoints as a colored line (color represents velocity)
    void visualize_waypoints_line()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = waypoint_line_width;

        double min_speed = *std::min_element(waypoints.V.begin(), waypoints.V.end());
        double max_speed = *std::max_element(waypoints.V.begin(), waypoints.V.end());

        RCLCPP_INFO(this->get_logger(), "Minimum speed: %f m/s", min_speed);
        RCLCPP_INFO(this->get_logger(), "Maximum speed: %f m/s", max_speed);

        for (unsigned int i = 0; i < waypoints.X.size(); ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = waypoints.X[i];
            p.y = waypoints.Y[i];
            p.z = 0.0; // Assuming z is 0 for 2D waypoints
            marker.points.push_back(p);

            std_msgs::msg::ColorRGBA color;
            double normalized_speed = (waypoints.V[i] - min_speed) / (max_speed - min_speed);

            // Interpolate color: green (0, 1, 0) to yellow (1, 1, 0) to red (1, 0, 0)
            if (normalized_speed < 0.5)
            {
                // Green to Yellow
                color.r = 2 * normalized_speed;
                color.g = 1.0;
                color.b = 0.0;
            }
            else
            {
                // Yellow to Red
                color.r = 1.0;
                color.g = 1.0 - 2 * (normalized_speed - 0.5);
                color.b = 0.0;
            }
            color.a = 1.0;
            marker.colors.push_back(color);
        }

        marker_array.markers.push_back(marker);
        vis_path_line_pub->publish(marker_array);
    }

    // vizualize waypoints as arrows
    void visualize_waypoints_arrow()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = waypoint_arrow_length; // Shaft diameter
        marker.scale.y = waypoint_arrow_width;  // Head diameter
        marker.scale.z = waypoint_arrow_height; // Head length
        marker.color.r = waypoint_arrow_color[0];
        marker.color.g = waypoint_arrow_color[1];
        marker.color.b = waypoint_arrow_color[2];
        marker.color.a = waypoint_arrow_color[3];

        for (unsigned int i = 0; i < waypoints.X.size() - 1; ++i)
        {
            marker.pose.position.x = waypoints.X[i];
            marker.pose.position.y = waypoints.Y[i];
            marker.id = i;

            // Calculate orientation to point to the next waypoint
            double dx = waypoints.X[i + 1] - waypoints.X[i];
            double dy = waypoints.Y[i + 1] - waypoints.Y[i];
            double yaw = atan2(dy, dx);

            tf2::Quaternion quat;
            quat.setRPY(0, 0, yaw);
            marker.pose.orientation = tf2::toMsg(quat);

            marker_array.markers.push_back(marker);
        }

        vis_path_arrow_pub->publish(marker_array);
    }
    // Initialise variables
    // global static (to be shared by all objects) and dynamic variables (each instance gets its own copy -> managed on the stack)
    struct csvFileData
    {
        std::vector<double> X;
        std::vector<double> Y;
        std::vector<double> V;
    };

    // variables
    double waypoint_line_width;
    double waypoint_arrow_length;
    double waypoint_arrow_width;
    double waypoint_arrow_height;
    std::vector<double> waypoint_arrow_color;
    bool reverse_waypoints_order;

    // topic names
    std::string waypoints_path;
    std::string rviz_waypoints_line_topic;
    std::string rviz_waypoints_arrow_topic;

    // file object
    std::fstream csvFile_waypoints;

    // struct initialisation
    csvFileData waypoints;

    // Publisher initialisation
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_path_line_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_path_arrow_pub;

    // Timer initialisation
    rclcpp::TimerBase::SharedPtr visualize_waypoints_line_timer_;
    rclcpp::TimerBase::SharedPtr visualize_waypoints_arrow_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointVisualizer>());
    rclcpp::shutdown();
    return 0;
}
