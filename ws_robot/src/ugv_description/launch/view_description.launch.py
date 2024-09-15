#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',            default_value='true',      description='Use simulation clock if true')

    urdf_path = os.path.join(get_package_share_directory('ugv_description'), 'urdf', 'robot.urdf.xacro')
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': xacro.process_file(urdf_path).toxml(),
        'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )



    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        # parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]    # Use_sim_time set to false in joint_state_publisher_node to avoid error. Hence commented out.
    )



    rviz_config_dir = os.path.join(get_package_share_directory('ugv_description'), 'config', 'rviz', 'view_description.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )



    return LaunchDescription([
            use_sim_time_arg,

            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
