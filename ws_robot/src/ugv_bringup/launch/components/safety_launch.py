#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    safety_config = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'util', 'safety.yaml')
    
    safety_config_arg = DeclareLaunchArgument('safety_config', default_value=safety_config, description='Path to safety config file')
    
    cmd_cutoff_and_brake_node = Node(
        package='ugv_util',
        executable='cutoff_and_brake_control',
        name='cmd_cutoff_and_brake_control',
        parameters=[LaunchConfiguration('safety_config')],
    )
    
    autonomous_safety_brake_node = Node(
        package='ugv_util',
        executable='safety_brake',
        name='autonomous_safety_brake',
        parameters=[LaunchConfiguration('safety_config')],
    )

    return LaunchDescription([
            LogInfo(msg="STARTING SAFETY COMPONENTS..."),
            safety_config_arg,
            cmd_cutoff_and_brake_node,
            autonomous_safety_brake_node,
        ]
    )