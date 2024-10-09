#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    lidar_hokuyo_ust10lx_config = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'sensor', 'lidar_hokuyo_ust10lx.yaml')
    lidar_robotis_lds01_config = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'sensor', 'lidar_robotis_lds01.yaml')

    lidar_hokuyo_ust10lx_config_arg = DeclareLaunchArgument('lidar_hokuyo_ust10lx_config', default_value=lidar_hokuyo_ust10lx_config, description='Hokuyo UST-10LX configs')
    lidar_robotis_lds01_config_arg =  DeclareLaunchArgument('lidar_robotis_lds01_config',  default_value=lidar_robotis_lds01_config,  description='Robotis LDS-01 configs')

    lidar_hokuyo_urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('lidar_hokuyo_ust10lx_config')]
    )

    lidar_robotis_lds01_node = Node(
        package='hls_lfcd_lds_driver',
        executable='hlds_laser_publisher',
        name='hlds_laser_publisher_node',
        parameters=[LaunchConfiguration('lidar_robotis_lds01_config')],
        output='screen'
    )



    return LaunchDescription([
            LogInfo(msg="STARTING SENSOR COMPONENTS..."),
            lidar_hokuyo_ust10lx_config_arg,
            # lidar_robotis_lds01_config_arg,

            lidar_hokuyo_urg_node,
            # lidar_robotis_lds01_node,
        ]
    )