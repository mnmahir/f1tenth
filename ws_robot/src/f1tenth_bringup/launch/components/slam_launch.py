"""Mapping: slam_toolbox (online async) builds the occupancy map and publishes map -> odom."""
import os
import sys

from launch import LaunchDescription
from launch.actions import EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from common import common_arguments, params, respawn, share  # noqa: E402


def launch_setup(context):
    slam_config = share('ugv_bringup', 'config', 'slam', 'f1tenth_online_async.yaml')
    slam = LifecycleNode(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                         namespace='', output='screen', respawn=respawn(context), respawn_delay=2.0,
                         parameters=params(context, 'slam_toolbox', slam_config) + [{'use_lifecycle_manager': False}])
    # Since Jazzy, slam_toolbox does nothing until configured and activated
    configure = RegisterEventHandler(OnProcessStart(target_action=slam, on_start=[EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam), transition_id=Transition.TRANSITION_CONFIGURE))]))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                              transition_id=Transition.TRANSITION_ACTIVATE))]))
    return [slam, configure, activate]


def generate_launch_description():
    return LaunchDescription(common_arguments() + [OpaqueFunction(function=launch_setup)])
