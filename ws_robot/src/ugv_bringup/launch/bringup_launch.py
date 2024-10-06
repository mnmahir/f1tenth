import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    robot_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'components', 'robot_bringup_launch.py')]),
    )
    
    sensor_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'components', 'sensor_bringup_launch.py')]),
    )
    
    safety_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'components', 'safety_bringup_launch.py')]),
    )


    # test_node = ExecuteProcess(
    #     cmd=['ros2', 'launch', os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'sensor_bringup.launch.py')],
    #     output='screen'
    # )


    return LaunchDescription([       
            robot_bringup_node,
            sensor_bringup_node,
            safety_bringup_node,
        ]
    )