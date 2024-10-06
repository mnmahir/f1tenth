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


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.declare_parameter('ittc_threshold', 0.2)        # threshold for instantaneous time to collision (iTTC)
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
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 10)
        self.bypass_teleop_sub = self.create_subscription(Joy, self.get_parameter('bypass_teleop_topic').value, self.bypass_teleop_callback, 10)
        
        # Initialize variables
        self.braking_current = self.get_parameter('braking_current').value
        self.ittc_threshold = self.get_parameter('ittc_threshold').value
        self.fstop_min_ray = self.get_parameter('force_stop_min_ray').value
        self.fstop_rect_x_min = self.get_parameter('force_stop_rectangular_region').value[0]
        self.fstop_rect_x_max = self.get_parameter('force_stop_rectangular_region').value[1]
        self.fstop_rect_y_min = self.get_parameter('force_stop_rectangular_region').value[2]
        self.fstop_rect_y_max = self.get_parameter('force_stop_rectangular_region').value[3]
        self.bypass_teleop_button = self.get_parameter('bypass_teleop_button').value
        self.bypass_teleop_state = False
        self.speed = 0.0
        self.toggle_cb = False
        self.ittc = np.inf
        
        
        
        self._logger.info(f"Safety node started. ")
        self._logger.info(f"Maximum braking current: {self.braking_current} A")

    def apply_emergency_brake(self):
        if not self.toggle_cb:
            self.toggle_cb = True
            self._logger.info(f"Applying emergency brake...")
        self.safe_pub.publish(Bool(data=True))
        self.brake_pub.publish(Float64(data=self.braking_current))
        
    def release_emergency_brake(self):
        self.toggle_cb = False
        self.ittc = np.inf
        self.brake_pub.publish(Float64(data=0.0))
        self.safe_pub.publish(Bool(data=False))
        self._logger.info("Emergency brake released.")
        
    
    def compute_ittc(self, ray_range, ray_angle):
        r_dot = np.cos(ray_angle) * self.speed  # Calculate range rate (using vehicle's current longitudinal velocity)
        r_dot[r_dot < 1e-3] = 0 # Set small r_dot value to and value less than 0 to 0
        with np.errstate(divide='ignore', invalid='ignore'):
            self.ittc  = np.min(np.where(r_dot != 0, np.divide(ray_range, r_dot), np.inf))    # Calculate time to collision
        # self.ittc   = np.min(np.divide(ray_range[r_dot > 0] , r_dot[r_dot > 0]))
        return self.ittc

    def bypass_teleop_callback(self, joy_msg):
        self.bypass_teleop_state = joy_msg.buttons[self.bypass_teleop_button]
        
    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x  # current speed of the vehicle
            

    def scan_callback(self, scan_msg):
        # Extract the range and angle of each ray
        ray_range = np.array(scan_msg.ranges)
        ray_angle = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ray_range))
        
        # Convert polar coordinates to Cartesian coordinates
        x = ray_range * np.cos(ray_angle)
        y = ray_range * np.sin(ray_angle)

        # Check if any point is within the defined rectangular region
        within_rect = (self.fstop_rect_x_min <= x) & (x <= self.fstop_rect_x_max) & (self.fstop_rect_y_min <= y) & (y <= self.fstop_rect_y_max)
        
        # Stop the vehicle if the more than min_rays reported distance is less than the threshold
        if np.sum(within_rect) >= self.fstop_min_ray and not self.bypass_teleop_state:
            if not self.toggle_cb:
                self._logger.warn(f"An object is within {self.get_parameter('force_stop_rectangular_region').value}m scan boundary! Force stopping the vehicle. Use teleop to move the vehicle.")
            self.apply_emergency_brake()
            
        elif self.compute_ittc(ray_range, ray_angle) < self.ittc_threshold and not self.bypass_teleop_state:
            self._logger.warn(f"iTTC: {self.ittc:.3f} s (< {self.ittc_threshold:.2f}s). Applying emergency brake...")
            self.apply_emergency_brake()
            time.sleep(0.5)
            
        elif self.toggle_cb:
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