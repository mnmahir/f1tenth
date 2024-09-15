#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    joy_teleop_config = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'controller', 'joy_teleop.yaml')
    vesc_config = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'controller', 'vesc.yaml')
    mux_config = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'controller', 'mux.yaml')

    joy_teleop_config_arg = DeclareLaunchArgument('joy_config', default_value=joy_teleop_config,    description='Descriptions for joy and joy_teleop configs')
    vesc_config_arg = DeclareLaunchArgument('vesc_config',      default_value=vesc_config,          description='Descriptions for vesc configs')
    mux_config_arg = DeclareLaunchArgument('mux_config',        default_value=mux_config,           description='Descriptions for ackermann mux configs')


    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )


    return LaunchDescription([
            joy_config_arg,
            vesc_config_arg,
            mux_config_arg,

            joy_node,
            joy_teleop_node,
            ackermann_to_vesc_node,
            vesc_to_odom_node,
            vesc_driver_node,
            # throttle_interpolator_node,
            ackermann_mux_node,
            static_tf_node
        ]
    )
