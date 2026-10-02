"""Path mode tools: path recorder and raceline optimizer."""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from common import common_arguments, params, respawn, share  # noqa: E402


def launch_setup(context):
    tools_config = share('f1tenth_tools', 'config', 'tools.yaml')
    dirs = {'maps_dir': context.launch_configurations['maps_dir'],
            'paths_dir': context.launch_configurations['paths_dir']}
    r = respawn(context)
    return [
        Node(package='f1tenth_tools', executable='path_recorder', name='path_recorder', respawn=r, respawn_delay=2.0,
             parameters=params(context, 'path_recorder', tools_config) + [
                 {'paths_dir': dirs['paths_dir'], 'map_name': context.launch_configurations['map']}]),
        Node(package='f1tenth_tools', executable='raceline_optimizer', name='raceline_optimizer', respawn=r,
             respawn_delay=2.0, parameters=params(context, 'raceline_optimizer', tools_config) + [dirs]),
    ]


def generate_launch_description():
    return LaunchDescription(common_arguments() + [
        DeclareLaunchArgument('maps_dir', default_value=os.path.expanduser('~/f1tenth/data/maps')),
        DeclareLaunchArgument('paths_dir', default_value=os.path.expanduser('~/f1tenth/data/paths')),
        DeclareLaunchArgument('map', default_value='', description='Name of the session map (saved with each path)'),
        OpaqueFunction(function=launch_setup),
    ])
