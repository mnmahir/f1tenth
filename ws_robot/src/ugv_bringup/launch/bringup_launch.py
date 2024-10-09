import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    robot_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'components', 'robot_launch.py')]),
    )
    
    sensor_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'components', 'sensor_launch.py')]),
    )
    
    safety_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'components', 'safety_launch.py')]),
    )


    # test_node = ExecuteProcess(
    #     cmd=['ros2', 'launch', os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'sensor_bringup.launch.py')],
    #     output='screen'
    # )


    return LaunchDescription([       
            robot_nodes,
            sensor_nodes,
            safety_nodes,
        ]
    )