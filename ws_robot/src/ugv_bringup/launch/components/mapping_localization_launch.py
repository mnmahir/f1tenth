#!/usr/bin/python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ekf_yaml_file_path = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'localization', 'ekf.yaml')
    declare_ekf_params_file_cmd =   DeclareLaunchArgument('ekf_params_file',    default_value=ekf_yaml_file_path, description='Full path to the ROS2 parameters file to use for all launched nodes')

    ekf_params_file = LaunchConfiguration('ekf_params_file')
    
    

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[('/odometry/filtered', '/odom/filtered')]
    )
   
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_ekf_params_file_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(ekf_node)

    return ld
