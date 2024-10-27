from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('ugv_bringup'),
        'config',
        'race.yaml'
    )
    
    reactive_follow_gap = Node(
        package='ugv_race',
        executable='reactive_node',
        name='reactive_follow_gap',
        parameters=[config]
    )

    pure_pursuit = Node(
        package='ugv_race',
        executable='pure_pursuit',
        name='pure_pursuit',
        parameters=[config]
    )

    waypoint_visualizer_node = Node(
        package='ugv_race',
        executable='waypoint_visualizer',
        name='waypoint_visualizer_node',
        parameters=[config]
    )
    
    # avoidance_controller_node = Node(
    #     package='ugv_race',
    #     executable='avoidance_controller',
    #     name='avoidance_controller_node',
    #     parameters=[config]
    # )
    
    race_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'components', 'race_controller_launch.py')]),
    )

    # finalize
    # ld.add_action(reactive_follow_gap)
    ld.add_action(pure_pursuit)
    ld.add_action(waypoint_visualizer_node)
    # ld.add_action(avoidance_controller_node)
    ld.add_action(race_controller)

    return ld
