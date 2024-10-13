#!/usr/bin/python3
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler, LogInfo)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    lidar_hokuyo_ust10lx_config = os.path.join(get_package_share_directory('ugv_bringup'), 'config', 'sensor', 'lidar_hokuyo_ust10lx.yaml')

    lidar_hokuyo_ust10lx_config_arg = DeclareLaunchArgument('lidar_hokuyo_ust10lx_config', default_value=lidar_hokuyo_ust10lx_config, description='Hokuyo UST-10LX configs')
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='true', description='Automatically start the lifecycle node')
    
    
    # lidar_hokuyo_urg_node = Node(
    #     package='urg_node',
    #     executable='urg_node_driver',
    #     name='urg_node',
    #     parameters=[LaunchConfiguration('lidar_hokuyo_ust10lx_config')]
    # )
    
    # urg_node2をライフサイクルノードとして起動
    lidar_hokuyo_urg_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='urg_node2',
        parameters=[LaunchConfiguration('lidar_hokuyo_ust10lx_config')],
        namespace='',
        output='screen',
    )

    # Unconfigure状態からInactive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lidar_hokuyo_urg_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lidar_hokuyo_urg_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Inactive状態からActive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lidar_hokuyo_urg_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lidar_hokuyo_urg_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    return LaunchDescription([
            LogInfo(msg="STARTING SENSOR COMPONENTS..."),
            lidar_hokuyo_ust10lx_config_arg,
            auto_start_arg,

            lidar_hokuyo_urg_node,
            urg_node2_node_configure_event_handler,
            urg_node2_node_activate_event_handler,
        ]
    )