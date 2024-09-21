#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import xacro


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',            default_value='false',      description='Use simulation clock if true')
    robot_state_publisher_arg = DeclareLaunchArgument('robot_state',    default_value='true',      description='Publish robot state')
    joint_state_publisher_arg = DeclareLaunchArgument('joint_state',    default_value='true',      description='Publish joint state based on URDF')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_state = LaunchConfiguration('robot_state')
    joint_state = LaunchConfiguration('joint_state')

    urdf_path = os.path.join(get_package_share_directory('ugv_description'), 'urdf', 'robot.urdf.xacro')
    
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': xacro.process_file(urdf_path).toxml(),
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(robot_state)
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(joint_state)
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_arg,
        joint_state_publisher_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])