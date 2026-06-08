"""Launch monitoring node only (no GUI)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb3_monitor',
            executable='monitor_node',
            name='tb3_monitor',
            output='screen',
            emulate_tty=True,
        ),
    ])
