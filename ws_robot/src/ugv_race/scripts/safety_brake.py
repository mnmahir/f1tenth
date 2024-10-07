#!/usr/bin/env python3
from yaml import scan
import rclpy
from rclpy.node import Node
import time

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan, Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PolygonStamped, Point32



class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.declare_parameter('ittc_threshold', 0.2)        # threshold for instantaneous time to collision (iTTC)
        self.declare_parameter('ittc_foward_drive_scan_width', 0.3)
        self.declare_parameter('ittc_foward_drive_x_scan_offset', -0.3)
        self.declare_parameter('force_stop_rectangular_region', [0.0, 0.16, -0.14, 0.14])   # rectangular region to stop the vehicle (x_min, x_max, y_min, y_max)
        self.declare_parameter('force_stop_min_ray', 5)     # minimum number of rays reporting less than threshold to stop the vehicle
        self.declare_parameter('braking_current', -10.0)    # maximum braking current
        self.declare_parameter('scan_topic', "/scan")
        self.declare_parameter('odom_topic', "/odom")
        self.declare_parameter('brake_publisher_topic', "/commands/motor/brake")
        self.declare_parameter('bool_publisher_topic', "/mux_bool/autonomous_cutoff_and_brake")
        self.declare_parameter('bypass_teleop_topic', "/joy")
        self.declare_parameter('bypass_teleop_button', 6)
        
        
        # Publisher
        self.brake_pub = self.create_publisher(Float64, self.get_parameter('brake_publisher_topic').value, 10)
        self.safe_pub = self.create_publisher(Bool, self.get_parameter('bool_publisher_topic').value, 10)
        self.force_stop_boundary_pub = self.create_publisher(PolygonStamped, '/safety/force_stop_boundary', 10)
        self.force_ittc_stop_boundary_pub = self.create_publisher(PolygonStamped, '/safety/ittc_stop_boundary', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 10)
        self.cmd_ackermann_sub = self.create_subscription(AckermannDriveStamped, "/ackermann_cmd", self.cmd_ackermann_callback, 10)
        self.bypass_teleop_sub = self.create_subscription(Joy, self.get_parameter('bypass_teleop_topic').value, self.bypass_teleop_callback, 10)
        
        # Timed publisher
        self.create_timer(1.0, self.publish_force_stop_boundary)
        
        # Initialize variables
        self.braking_current = self.get_parameter('braking_current').value
        self.ittc_threshold = self.get_parameter('ittc_threshold').value
        self.fstop_min_ray = self.get_parameter('force_stop_min_ray').value
        self.fstop_rect_x_min = self.get_parameter('force_stop_rectangular_region').value[0]
        self.fstop_rect_x_max = self.get_parameter('force_stop_rectangular_region').value[1]
        self.fstop_rect_y_min = self.get_parameter('force_stop_rectangular_region').value[2]
        self.fstop_rect_y_max = self.get_parameter('force_stop_rectangular_region').value[3]
        self.bypass_teleop_button = self.get_parameter('bypass_teleop_button').value
        self.ittc_foward_drive_scan_width = self.get_parameter('ittc_foward_drive_scan_width').value
        self.ittc_foward_drive_x_scan_offset = self.get_parameter('ittc_foward_drive_x_scan_offset').value
        self.bypass_teleop_state = False
        self.speed = 0.0
        self.steering_angle = 0.0
        self.toggle_emergency_brake = False
        self.ittc = np.inf
        self.ittc_idx = 0
        
        
        self._logger.info(f"Safety node started. ")
        self._logger.info(f"Maximum braking current: {self.braking_current} A")


    def publish_force_stop_boundary(self):
        polygon = PolygonStamped()
        polygon.header.frame_id = "lidar_1_link"  # Adjust the frame as needed
        polygon.header.stamp = self.get_clock().now().to_msg()

        # Define the rectangular region points
        points = [
            [self.fstop_rect_x_min, self.fstop_rect_y_min, 0.0],
            [self.fstop_rect_x_min, self.fstop_rect_y_max, 0.0],
            [self.fstop_rect_x_max, self.fstop_rect_y_max, 0.0],
            [self.fstop_rect_x_max, self.fstop_rect_y_min, 0.0]
        ]

        for point in points:
            p = Point32()
            p.x, p.y, p.z = point
            polygon.polygon.points.append(p)

        self.force_stop_boundary_pub.publish(polygon)
        
    def publish_ittc_foward_drive_scan_boundary(self):
        polygon = PolygonStamped()
        polygon.header.frame_id = "lidar_1_link"  # Adjust the frame as needed
        polygon.header.stamp = self.get_clock().now().to_msg()
        
        # Define the rectangular region points
        point_RL = [self.ittc_foward_drive_x_scan_offset, -(self.ittc_foward_drive_scan_width / 2) - np.abs((self.ittc_foward_drive_scan_width * self.steering_angle)), 0.0]
        point_RR = [self.ittc_foward_drive_x_scan_offset, (self.ittc_foward_drive_scan_width / 2) + np.abs((self.ittc_foward_drive_scan_width * self.steering_angle)), 0.0]
        point_FL = [1.0, -(self.ittc_foward_drive_scan_width / 2) - np.abs((self.ittc_foward_drive_scan_width * self.steering_angle)), 0.0]
        point_FR = [1.0, (self.ittc_foward_drive_scan_width / 2) + np.abs((self.ittc_foward_drive_scan_width * self.steering_angle)), 0.0]
        points = [
            point_RL,
            point_RR,
            point_FR,
            point_RR,
            point_RL,
            point_FL,
        ]
        
        for point in points:
            p = Point32()
            p.x, p.y, p.z = point
            polygon.polygon.points.append(p)
            
        self.force_ittc_stop_boundary_pub.publish(polygon)
    
    
    
    def apply_emergency_brake(self):
        if not self.toggle_emergency_brake:
            self.toggle_emergency_brake = True
            self._logger.info(f"Applying emergency brake...")
        self.safe_pub.publish(Bool(data=True))
        self.brake_pub.publish(Float64(data=self.braking_current))
        
    def release_emergency_brake(self):
        self.toggle_emergency_brake = False
        self.ittc = np.inf
        self.brake_pub.publish(Float64(data=0.0))
        self.safe_pub.publish(Bool(data=False))
        self._logger.info("Emergency brake released.")
        
    
    def compute_ittc(self, ray_range, ray_angle):
        r_dot = np.cos(ray_angle) * self.speed  # Calculate range rate (using vehicle's current longitudinal velocity)
        r_dot[r_dot < 1e-3] = 0 # Set small r_dot value to and value less than 0 to 0
        with np.errstate(divide='ignore', invalid='ignore'):
            time_to_collision = np.where(r_dot != 0, np.divide(ray_range, r_dot), np.inf)
            self.ittc = np.min(time_to_collision)  # Calculate time to collision
            self.ittc_idx = np.argmin(time_to_collision)
        return self.ittc

    def bypass_teleop_callback(self, joy_msg):
        self.bypass_teleop_state = joy_msg.buttons[self.bypass_teleop_button]
        
    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x  # current speed of the vehicle
        
    def cmd_ackermann_callback(self, ackermann_msg):
        self.steering_angle = ackermann_msg.drive.steering_angle  # current steering angle of the vehicle
        self.publish_ittc_foward_drive_scan_boundary()
            

    def scan_callback(self, scan_msg):
        # Extract the range and angle of each ray
        ray_range = np.array(scan_msg.ranges)
        ray_angle = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ray_range))
        
        # Convert polar coordinates to Cartesian coordinates
        x = ray_range * np.cos(ray_angle)
        y = ray_range * np.sin(ray_angle)

        # Check if any point is within the defined rectangular region
        within_rect = (self.fstop_rect_x_min <= x) & (x <= self.fstop_rect_x_max) & (self.fstop_rect_y_min <= y) & (y <= self.fstop_rect_y_max)
        within_foward_drive_boundary= (x >= self.ittc_foward_drive_x_scan_offset) & (np.abs(y) <= ((self.ittc_foward_drive_scan_width / 2) + (self.ittc_foward_drive_scan_width * np.abs(self.steering_angle ))))
        print(f"within_foward_drive_boundary: {within_foward_drive_boundary}")
        
        # Stop the vehicle if the more than min_rays reported distance is less than the threshold
        if np.sum(within_rect) >= self.fstop_min_ray and not self.bypass_teleop_state:
            self.apply_emergency_brake()
            self._logger.warn(f"An object is in {self.get_parameter('force_stop_rectangular_region').value} m scan boundary! Vehicle force stopped. Use teleop to move the vehicle.")
            time.sleep(1.0)
        else:
            # clip the range to only see objects in front of the vehicle
            if self.speed > 0:
                ray_range = ray_range[within_foward_drive_boundary]
                ray_angle = ray_angle[within_foward_drive_boundary]
            else:   # remove the noises from rear of the scan
                ray_range = ray_range[(ray_angle > -2.268928028) & (ray_angle < 2.268928028)]   
                ray_angle = ray_angle[(ray_angle > -2.268928028) & (ray_angle < 2.268928028)]
            self.compute_ittc(ray_range, ray_angle)
            if self.ittc < self.ittc_threshold and not self.bypass_teleop_state:
                self.apply_emergency_brake()
                self._logger.warn(f"iTTC: {self.ittc:.3f} s (< {self.ittc_threshold:.2f}s) to object at {(ray_angle[self.ittc_idx]*180/np.pi):.2f}° with approaching speed of {(ray_range[self.ittc_idx]/self.ittc):.2f} m/s ({self.speed:.2f} m/s)")
                self._logger.warn(f"Emergency brake applied {ray_range[self.ittc_idx]:.2f} m before collision to object.")
                time.sleep(0.5)
                
            elif self.toggle_emergency_brake:
                self.release_emergency_brake()          
            else:
                return
            
        

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()