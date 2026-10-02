#!/usr/bin/python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    amcl_params_file_path = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'localization', 'amcl.yaml')
    map_yaml_file_path = os.path.join(get_package_share_directory('ugv_race'), 'maps', 'f1tenth', 'map.yaml')
    ekf_yaml_file_path = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'localization', 'ekf.yaml')

    stdout_linebuf_envvar =         SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    declare_namespace_cmd =         DeclareLaunchArgument('namespace',          default_value='',                           description='Top-level namespace')
    declare_map_yaml_cmd =          DeclareLaunchArgument('map',                default_value=map_yaml_file_path,           description='Full path to map yaml file to load')
    declare_use_sim_time_cmd =      DeclareLaunchArgument('use_sim_time',       default_value='false',                      description='Use simulation (Gazebo) clock if true')
    declare_amcl_params_file_cmd =  DeclareLaunchArgument('amcl_params_file',   default_value=amcl_params_file_path,   description='Full path to the ROS2 parameters file to use for all launched nodes')
    declare_autostart_cmd =         DeclareLaunchArgument('autostart',          default_value='true',           description='Automatically startup the nav2 stack')
    declare_use_composition_cmd =   DeclareLaunchArgument('use_composition',    default_value='False',          description='Use composed bringup if True')
    declare_container_name_cmd =    DeclareLaunchArgument('container_name',     default_value='nav2_container', description='the name of conatiner that nodes will load in if use composition')
    declare_use_respawn_cmd =       DeclareLaunchArgument('use_respawn',        default_value='False',          description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    declare_log_level_cmd =         DeclareLaunchArgument('log_level',          default_value='info',           description='log level')
    declare_ekf_params_file_cmd =   DeclareLaunchArgument('ekf_params_file',    default_value=ekf_yaml_file_path, description='Full path to the ROS2 parameters file to use for all launched nodes')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    amcl_params_file = LaunchConfiguration('amcl_params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    
    
    lifecycle_nodes = [
        'map_server', 
        'amcl'
    ]
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[('/odometry/filtered', '/odom/filtered')]
    )
    
    base_footprint_to_map_odom_node = Node(
        package='ugv_util',
        executable='transform_to_odom',
        name='base_footprint_to_map_odom',
        output='screen',
        parameters=[{'source_frame': 'base_footprint',
                     'target_frame': 'map',
                     'odom_topic' : 'odom/map',
                     'rate': 50.0
                     }],
    )
    
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    {'yaml_filename': map_yaml_file},
                    {'use_sim_time': use_sim_time},
                    {'frame_id': 'map'},
                    {'topic_name': 'map'},
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[amcl_params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[
                    {'yaml_filename': map_yaml_file},
                    {'use_sim_time': use_sim_time},
                    {'frame_id': 'map'},
                    {'topic_name': 'map'},
                ],
                remappings=remappings),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[amcl_params_file],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    # map_server and amcl need a map; without one, say how to make it instead of failing the lifecycle bringup
    def map_localization(context):
        map_file = map_yaml_file.perform(context)
        if not os.path.isfile(map_file):
            return [LogInfo(msg=f'No map at {map_file}: map_server and amcl not started. '
                                 'Make one with mapping_launch.py and save it there, or pass map:=<path to map.yaml>.')]
        return [load_nodes, load_composable_nodes]

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_amcl_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_ekf_params_file_cmd)
    ld.add_action(base_footprint_to_map_odom_node)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(ekf_node)
    ld.add_action(OpaqueFunction(function=map_localization))

    return ld
