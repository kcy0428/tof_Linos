"""Launch monitoring node, RViz2, and rqt_plot."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('tb3_monitor')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'tb3_monitor.rviz')

    return LaunchDescription([
        # Monitoring node
        Node(
            package='tb3_monitor',
            executable='monitor_node',
            name='tb3_monitor',
            output='screen',
            emulate_tty=True,
        ),

        # ToF 8x8 Heatmap
        Node(
            package='tb3_monitor',
            executable='tof_heatmap',
            name='tof_heatmap',
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),

        # rqt_plot
        ExecuteProcess(
            cmd=[
                'rqt_plot',
                '/tof/front_distance/data',
                '/obstacle/distance/data',
                '/gap/width/data',
            ],
            output='screen',
        ),
    ])
