"""The F1TENTH driver UI ("pit wall"). Runs on the car's screen or on any computer that sees the car's ROS graph.

The car runs car_launch.py (its supervisor starts and stops everything else when the UI asks).
The UI makes two nodes, f1tenth_ui_<hostname> and ..._scene, so it is not given a node name here.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='f1tenth_bringup', executable='f1tenth_ui', output='screen', emulate_tty=True),
    ])
