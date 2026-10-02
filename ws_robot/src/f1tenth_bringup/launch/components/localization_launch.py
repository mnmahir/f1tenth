"""Localization on a saved map: map_server + AMCL (map -> odom), map-frame odometry, the lap timer and the
localization monitor (scan-to-map match, setting the pose from the UI)."""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from common import common_arguments, params, respawn, share  # noqa: E402


def launch_setup(context):
    map_file = context.launch_configurations['map']
    amcl_config = share('ugv_bringup', 'config', 'localization', 'amcl.yaml')
    tools_config = share('f1tenth_tools', 'config', 'tools.yaml')
    r = respawn(context)
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server', respawn=r, respawn_delay=2.0,
             parameters=params(context, 'map_server') + [{'yaml_filename': map_file, 'frame_id': 'map', 'topic_name': 'map'}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', respawn=r, respawn_delay=2.0,
             parameters=params(context, 'amcl', amcl_config)),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl'],
                          'attempt_respawn_reconnection': True}]),
        Node(package='ugv_util', executable='transform_to_odom', name='base_footprint_to_map_odom', respawn=r,
             respawn_delay=2.0, parameters=[{'source_frame': 'base_footprint', 'target_frame': 'map',
                                             'odom_topic': 'odom/map', 'rate': 50.0}]),
        Node(package='f1tenth_tools', executable='lap_timer', name='lap_timer', respawn=r, respawn_delay=2.0,
             parameters=params(context, 'lap_timer', tools_config)),
        Node(package='f1tenth_tools', executable='localization_monitor', name='localization_monitor', respawn=r,
             respawn_delay=2.0, parameters=params(context, 'localization_monitor', tools_config)),
    ]


def generate_launch_description():
    return LaunchDescription(common_arguments() + [
        DeclareLaunchArgument('map', description='Map YAML to localize in'),
        OpaqueFunction(function=launch_setup),
    ])
