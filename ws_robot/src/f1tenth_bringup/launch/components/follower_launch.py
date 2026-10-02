"""Path follower: pure pursuit on the selected path, publishing to /f1tenth/follower/drive.

The drive coordinator uses its steering for assist and forwards it to the mux only while autonomous is engaged.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from common import common_arguments, params, respawn, share  # noqa: E402


def launch_setup(context):
    race_config = share('ugv_race', 'config', 'race.yaml')
    overrides = {'waypoints_path': context.launch_configurations['path'],
                 'reverse_waypoints_order': context.launch_configurations['reverse'].lower() == 'true',
                 'drive_topic': '/f1tenth/follower/drive'}
    r = respawn(context)
    return [
        # race.yaml first, then the session's path, then the UI's saved overrides
        Node(package='ugv_race', executable='pure_pursuit', name='pure_pursuit', respawn=r, respawn_delay=2.0,
             parameters=[race_config, overrides] + params(context, 'pure_pursuit')),
        Node(package='ugv_race', executable='waypoint_visualizer', name='waypoint_visualizer_node', respawn=r,
             respawn_delay=2.0, parameters=[race_config, overrides]),
    ]


def generate_launch_description():
    return LaunchDescription(common_arguments() + [
        DeclareLaunchArgument('path', description='Path CSV (x,y,speed) to follow'),
        DeclareLaunchArgument('reverse', default_value='false', description='Follow the path backwards'),
        OpaqueFunction(function=launch_setup),
    ])
