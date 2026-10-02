#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    joy_teleop_config = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'controller', 'joy_teleop.yaml')
    vesc_config = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'controller', 'vesc.yaml')
    mux_config = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'controller', 'mux.yaml')
    urdf_path = os.path.join(get_package_share_directory('ugv_description'), 'urdf', 'robot.urdf.xacro')

    joy_teleop_config_arg = DeclareLaunchArgument('joy_config', default_value=joy_teleop_config,    description='Descriptions for joy and joy_teleop configs')
    vesc_config_arg = DeclareLaunchArgument('vesc_config',      default_value=vesc_config,          description='Descriptions for vesc configs')
    mux_config_arg = DeclareLaunchArgument('mux_config',        default_value=mux_config,           description='Descriptions for ackermann mux configs')
    urdf_path_arg = DeclareLaunchArgument('urdf_path',          default_value=urdf_path,            description='Path to urdf file')

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

    # LT scales joystick teleop speed (cmd_teleop/joy_raw -> cmd_teleop/joy)
    teleop_speed_multiplier_node = Node(
        package='ugv_util',
        executable='teleop_speed_multiplier',
        name='teleop_speed_multiplier',
        parameters=[LaunchConfiguration('joy_config')]
    )

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')],
        remappings=[('/ackermann_cmd', '/ackermann_cmd_filtered')]
    )

    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')],
        remappings=[('/odom', '/odom/wheel')]
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
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': xacro.process_file(urdf_path).toxml(),}]
    )

    # Steering and wheel joints from the VESC servo command and wheel odometry (the model steers in RViz)
    joint_state_publisher_node = Node(
        package='ugv_util',
        executable='vehicle_joint_state_publisher',
        name='vehicle_joint_state_publisher',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')],
    )



    return LaunchDescription([
            LogInfo(msg="STARTING ROBOT COMPONENTS..."),
            joy_teleop_config_arg,
            vesc_config_arg,
            mux_config_arg,

            joy_node,
            joy_teleop_node,
            teleop_speed_multiplier_node,
            ackermann_to_vesc_node,
            vesc_to_odom_node,
            vesc_driver_node,
            # throttle_interpolator_node,
            ackermann_mux_node,
            robot_state_publisher_node,
            joint_state_publisher_node,
        ]
    )
