#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
 
 
class CutoffAndBrakeControlNode(Node):
    def __init__(self):
        super().__init__("cutoff_and_brake_control_node")
        self.declare_parameter('joy_topic', "/joy")
        self.declare_parameter('joy_axis', 4)               # axis for cutoff and brake
        self.declare_parameter('braking_current', -10.0)      # maximum braking current
        self.declare_parameter('brake_publisher_topic', "/commands/motor/brake")
        self.declare_parameter('bool_publisher_topic', "/mux_bool/teleop_cutoff_and_brake")
        
        # Subscribers
        self.joy_sub = self.create_subscription(Joy, self.get_parameter('joy_topic').value, self.joy_callback, 10)
        
        # Publishers
        self.brake_pub = self.create_publisher(Float64, self.get_parameter('brake_publisher_topic').value, 10)
        self.cb_pub = self.create_publisher(Bool, self.get_parameter('bool_publisher_topic').value, 10)
        
        # Initialize variables
        self.joy_axis = self.get_parameter('joy_axis').value
        self.braking_current = self.get_parameter('braking_current').value
        self.toggle_cb = False
        self.brake_value = 0.0
        
        self._logger.info(f"Cutoff and brake control node started. ")
        self._logger.info(f"Maximum braking current: {self.braking_current} A")



    def joy_callback(self, msg):
        if msg.axes[self.joy_axis] < 1.0:
            if not self.toggle_cb:
                self.toggle_cb = True
                self._logger.info(f"Cutoff and braking...")
            self.toggle_cb = True
            self.cb_pub.publish(Bool(data=True))
            
            self.brake_value = (1 - msg.axes[self.joy_axis]) * self.braking_current / 2
            self.brake_pub.publish(Float64(data=self.brake_value))
            
            
        elif self.toggle_cb:
            self.toggle_cb = False
            self.brake_pub.publish(Float64(data=0.0))
            self.cb_pub.publish(Bool(data=False))
            self._logger.info("Brake released.")
            


def main(args=None):
    rclpy.init(args=args)
    node = CutoffAndBrakeControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()

