import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gamefield_barrier'), 'config', 'barrier.yaml')

    return LaunchDescription([
        Node(
            package='gamefield_barrier',
            executable='barrier_node',
            name='gamefield_barrier',
            output='screen',
            parameters=[config],
        ),
    ])