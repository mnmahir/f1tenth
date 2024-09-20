import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node



def generate_launch_description():
    robot_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'robot_bringup_launch.py')]),
    )
    
    sensor_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'sensor_bringup_launch.py')]),
    )


    # test_node = ExecuteProcess(
    #     cmd=['ros2', 'launch', os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'sensor_bringup.launch.py')],
    #     output='screen'
    # )


    return LaunchDescription([       
            robot_bringup_node,
            sensor_bringup_node,
        ]
    )