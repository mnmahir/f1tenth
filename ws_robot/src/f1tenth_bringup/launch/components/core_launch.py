"""Core of every session: VESC, joystick, mux, robot model, LiDAR, camera, safety, EKF, avoidance and telemetry.

Teleop chain: joy_teleop -> cmd_teleop/joy_raw -> teleop_speed_multiplier (LT boost) -> cmd_teleop/joy_user
-> drive_coordinator (steering assist / autonomous gate) -> cmd_teleop/joy -> ackermann_mux.
"""
import os
import sys

import xacro
from launch import LaunchDescription
from launch.actions import EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from common import common_arguments, params, respawn, share  # noqa: E402


def launch_setup(context):
    joy_config = share('ugv_bringup', 'config', 'controller', 'joy_teleop.yaml')
    vesc_config = share('ugv_bringup', 'config', 'controller', 'vesc.yaml')
    mux_config = share('ugv_bringup', 'config', 'controller', 'mux.yaml')
    lidar_config = share('ugv_bringup', 'config', 'sensor', 'lidar_hokuyo_ust10lx.yaml')
    util_config = share('ugv_bringup', 'config', 'util.yaml')
    ekf_config = share('ugv_bringup', 'config', 'localization', 'ekf.yaml')
    race_config = share('ugv_race', 'config', 'race.yaml')
    tools_config = share('f1tenth_tools', 'config', 'tools.yaml')
    urdf = share('ugv_description', 'urdf', 'robot.urdf.xacro')
    camera_device = '/dev/sensors/camera-see3cam' if os.path.exists('/dev/sensors/camera-see3cam') else '/dev/video0'
    r = respawn(context)

    def node(package, executable, name, files, **kwargs):
        return Node(package=package, executable=executable, name=name, respawn=r, respawn_delay=2.0,
                    parameters=params(context, name, *files) + kwargs.pop('extra_params', []), **kwargs)

    lidar = LifecycleNode(package='urg_node2', executable='urg_node2_node', name='urg_node2', namespace='',
                          parameters=params(context, 'urg_node2', lidar_config), respawn=r, respawn_delay=2.0,
                          output='screen')
    # urg_node2 is a lifecycle node: configure then activate on every (re)start
    lidar_configure = RegisterEventHandler(OnProcessStart(target_action=lidar, on_start=[EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(lidar), transition_id=Transition.TRANSITION_CONFIGURE))]))
    lidar_activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=lidar, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(lidar),
                                              transition_id=Transition.TRANSITION_ACTIVATE))]))

    return [
        # Driver input
        node('joy', 'joy_node', 'joy', [joy_config]),
        node('joy_teleop', 'joy_teleop', 'joy_teleop', [joy_config]),
        node('ugv_util', 'teleop_speed_multiplier', 'teleop_speed_multiplier', [joy_config],
             remappings=[('cmd_teleop/joy', 'cmd_teleop/joy_user')]),
        node('f1tenth_tools', 'drive_coordinator', 'drive_coordinator', [tools_config]),
        node('ackermann_mux', 'ackermann_mux', 'ackermann_mux', [mux_config]),
        # VESC
        node('vesc_driver', 'vesc_driver_node', 'vesc_driver_node', [vesc_config]),
        node('vesc_ackermann', 'ackermann_to_vesc_node', 'ackermann_to_vesc_node', [vesc_config],
             remappings=[('/ackermann_cmd', '/ackermann_cmd_filtered')]),
        node('vesc_ackermann', 'vesc_to_odom_node', 'vesc_to_odom_node', [vesc_config],
             remappings=[('/odom', '/odom/wheel')]),
        # Robot model
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
             respawn=r, respawn_delay=2.0, parameters=[{'robot_description': xacro.process_file(urdf).toxml()}]),
        node('ugv_util', 'vehicle_joint_state_publisher', 'vehicle_joint_state_publisher', [vesc_config]),
        # Sensors
        lidar, lidar_configure, lidar_activate,
        node('f1tenth_tools', 'mjpeg_camera', 'camera', [tools_config], extra_params=[{'device': camera_device}]),
        # Safety
        node('ugv_util', 'cutoff_and_brake_control', 'cmd_cutoff_and_brake_control', [util_config]),
        node('ugv_util', 'safety_brake', 'autonomous_safety_brake', [util_config]),
        node('ugv_util', 'utility_visualizer', 'utility_visualizer', []),
        node('ugv_race', 'avoidance_controller', 'avoidance_controller_node', [race_config]),
        # State estimation (odom -> base_footprint). The IMU's yaw goes through the drift corrector first: it holds
        # still while the wheels do (the VESC gyro's bias would otherwise turn the car on the map at rest).
        node('f1tenth_tools', 'imu_drift_corrector', 'imu_drift_corrector', [tools_config]),
        node('robot_localization', 'ekf_node', 'ekf_filter_node', [ekf_config],
             remappings=[('/odometry/filtered', '/odom/filtered')],
             extra_params=[{'imu0': 'sensors/imu/corrected'}]),
        # UI telemetry
        node('f1tenth_tools', 'telemetry_aggregator', 'telemetry_aggregator', [vesc_config, tools_config]),
    ]


def generate_launch_description():
    return LaunchDescription(common_arguments() + [OpaqueFunction(function=launch_setup)])
