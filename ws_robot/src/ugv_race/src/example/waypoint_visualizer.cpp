#include "waypoint_visualizer.hpp"

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

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

WaypointVisualizer::WaypointVisualizer() : Node("waypoint_visualizer_node") {
    this->declare_parameter("waypoints_path", "/sim_ws/src/pure_pursuit/racelines/e7_floor5.csv");
    this->declare_parameter("rviz_waypoints_topic", "/waypoints");

    waypoints_path = this->get_parameter("waypoints_path").as_string();
    rviz_waypoints_topic = this->get_parameter("rviz_waypoints_topic").as_string();

    vis_path_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(rviz_waypoints_topic, 1000);
    timer_ = this->create_wall_timer(2000ms, std::bind(&WaypointVisualizer::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "this node has been launched");
    download_waypoints();
}

void WaypointVisualizer::download_waypoints() {  // put all data in vectors
    csvFile_waypoints.open(waypoints_path, std::ios::in);

    RCLCPP_INFO(this->get_logger(), "%s", (csvFile_waypoints.is_open() ? "fileOpened" : "fileNOTopened"));

    std::string line, word;

    while (std::getline(csvFile_waypoints, line)) {
        std::stringstream s(line);
        int j = 0;
        double x, y, v;

        while (getline(s, word, ',')) {
            if (!word.empty()) {
                if (j == 0) {
                    x = std::stod(word);
                } else if (j == 1) {
                    y = std::stod(word);
                } else if (j == 2) {
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
}

void WaypointVisualizer::visualize_points() {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.281;  // Line width

    double min_speed = *std::min_element(waypoints.V.begin(), waypoints.V.end());
    double max_speed = *std::max_element(waypoints.V.begin(), waypoints.V.end());

    RCLCPP_INFO(this->get_logger(), "Minimum speed: %f m/s", min_speed);
    RCLCPP_INFO(this->get_logger(), "Maximum speed: %f m/s", max_speed);

    for (unsigned int i = 0; i < waypoints.X.size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = waypoints.X[i];
        p.y = waypoints.Y[i];
        p.z = 0.0;  // Assuming z is 0 for 2D waypoints
        marker.points.push_back(p);

        std_msgs::msg::ColorRGBA color;
        double normalized_speed = (waypoints.V[i] - min_speed) / (max_speed - min_speed);

        // Interpolate color: green (0, 1, 0) to yellow (1, 1, 0) to red (1, 0, 0)
        if (normalized_speed < 0.5) {
            // Green to Yellow
            color.r = 2 * normalized_speed;
            color.g = 1.0;
            color.b = 0.0;
        } else {
            // Yellow to Red
            color.r = 1.0;
            color.g = 1.0 - 2 * (normalized_speed - 0.5);
            color.b = 0.0;
        }
        color.a = 1.0;
        marker.colors.push_back(color);
    }

    marker_array.markers.push_back(marker);
    vis_path_pub->publish(marker_array);
}

void WaypointVisualizer::timer_callback() {
    visualize_points();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<WaypointVisualizer>();  // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}