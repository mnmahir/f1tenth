"""Runs on the car: the session supervisor, the system monitor and a Zenoh router. Users never start robot nodes
by hand.

The car service (scripts/install_car_service.bash) runs this at boot. Nothing else runs until a session is
started from the UI (ui_launch.py, on the car's screen or any computer on the network); the supervisor then
starts and stops the launch files in launch/components for that session.

The supervisor announces the car (name, ROS domain, mode, battery) on UDP so UIs can list the cars around them.
The router gives UIs outside the car's local network (e.g. over a VPN) a fixed address: tcp/<car>:7447.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    tools_config = os.path.join(get_package_share_directory('f1tenth_tools'), 'config', 'tools.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('data_dir', default_value='~/f1tenth/data',
                              description='Where maps, paths, parameter overrides and bags are kept'),
        DeclareLaunchArgument('car_name', default_value=EnvironmentVariable('F1TENTH_CAR_NAME', default_value=''),
                              description='Name shown in the UI (default: the hostname)'),
        DeclareLaunchArgument('auto_restart', default_value='true', description='Restart crashed nodes'),
        DeclareLaunchArgument('start_core', default_value='false',
                              description='Run the core (VESC, sensors, joystick, safety) in standby too'),
        DeclareLaunchArgument('router', default_value='true',
                              description='Run a Zenoh router on port 7447 for UIs on other networks'),
        Node(package='f1tenth_tools', executable='supervisor', name='supervisor', namespace='f1tenth',
             output='screen', respawn=True, respawn_delay=5.0,
             parameters=[{'data_dir': LaunchConfiguration('data_dir'),
                          'car_name': LaunchConfiguration('car_name'),
                          'auto_restart': ParameterValue(LaunchConfiguration('auto_restart'), value_type=bool),
                          'start_core': ParameterValue(LaunchConfiguration('start_core'), value_type=bool)}]),
        Node(package='f1tenth_tools', executable='system_monitor', name='system_monitor', respawn=True,
             respawn_delay=2.0, parameters=[tools_config]),
        # Finds the car's nodes by multicast like they find each other, and routes for UIs that connect to it
        ExecuteProcess(cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'], name='zenoh_router',
                       additional_env={'ZENOH_CONFIG_OVERRIDE': 'scouting/multicast/enabled=true'},
                       respawn=True, respawn_delay=5.0, condition=IfCondition(LaunchConfiguration('router'))),
    ])
