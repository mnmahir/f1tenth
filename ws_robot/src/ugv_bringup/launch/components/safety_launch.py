#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    util_config = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'util.yaml')
    
    util_config_arg = DeclareLaunchArgument('util_config', default_value=util_config, description='Path to utility config file')
    
    cmd_cutoff_and_brake_node = Node(
        package='ugv_util',
        executable='cutoff_and_brake_control',
        name='cmd_cutoff_and_brake_control',
        parameters=[LaunchConfiguration('util_config')],
    )
    
    autonomous_safety_brake_node = Node(
        package='ugv_util',
        executable='safety_brake',
        name='autonomous_safety_brake',
        parameters=[LaunchConfiguration('util_config')],
    )
    
    visualizer_node = Node(
        package='ugv_util',
        executable='utility_visualizer',
        name='utility_visualizer',
        # parameters=[LaunchConfiguration('util_config')],      # Cannot use.. Need to check
    )

    return LaunchDescription([
            LogInfo(msg="STARTING SAFETY COMPONENTS..."),
            util_config_arg,
            cmd_cutoff_and_brake_node,
            autonomous_safety_brake_node,
            visualizer_node,
        ]
    )