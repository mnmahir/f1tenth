#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    cmd_cutoff_and_brake_node = Node(
        package='ugv_race',
        executable='cutoff_and_brake_control',
        name='cmd_cutoff_and_brake_control',
        parameters=[
            {'braking_current': -60.0},
            ],
    )
    
    autonomous_safety_brake_node = Node(
        package='ugv_race',
        executable='safety_brake.py',
        name='autonomous_safety_brake',
        parameters=[
            {'braking_current': -60.0},
            {'ittc_threshold': 0.4},
            {'odom_topic': '/odom/wheel'},
            ],
    )

    return LaunchDescription([
            LogInfo(msg="STARTING SAFETY COMPONENTS..."),
            cmd_cutoff_and_brake_node,
            autonomous_safety_brake_node,
        ]
    )