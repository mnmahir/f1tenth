"""Helpers shared by the f1tenth_bringup component launch files (loaded by path, not installed as a module)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument


def share(package, *path):
    return os.path.join(get_package_share_directory(package), *path)


def common_arguments():
    return [
        DeclareLaunchArgument('respawn', default_value='true', description='Restart nodes that exit'),
        DeclareLaunchArgument('params_dir', default_value=os.path.expanduser('~/f1tenth/data/params'),
                              description='Parameter overrides saved from the UI (<node>.yaml)'),
    ]


def params(context, node_name, *files):
    """Parameter files for a node, plus its override saved from the UI if there is one."""
    override = os.path.join(context.launch_configurations['params_dir'], node_name + '.yaml')
    return [*files, override] if os.path.isfile(override) else list(files)


def respawn(context):
    return context.launch_configurations['respawn'].lower() == 'true'
