from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('ugv_bringup'),
        'config',
        'race.yaml'
    )

    
    avoidance_controller_node = Node(
        package='ugv_race',
        executable='avoidance_controller',
        name='avoidance_controller_node',
        parameters=[config]
    )

    ld.add_action(avoidance_controller_node)

    return ld
