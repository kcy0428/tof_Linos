"""Bring up the full shared_control stack on TurtleBot3 Waffle.

이 launch 파일이 담당하는 노드:
  * joy_node          (8BitDo Micro 블루투스 컨트롤러 → /joy)
  * tof_publisher     (VL53L8CX 8x8 그리드 → /tof/distances, /tof/status)
  * teleop_twist_joy  (/joy → /cmd_vel_manual)
  * tof_bridge        (/tof/distances → /tof/front_distance)
  * obstacle_detector (ToF + LiDAR 융합 → /obstacle/detected)
  * gap_analyzer      (LiDAR 갭 폭 → /gap/width, /gap/passable)
  * avoidance         (회피 명령 → /cmd_vel_auto)
  * arbitrator        (상태머신 → /cmd_vel, /robot/state)

별도로 실행되어야 하는 것 (TurtleBot3 bringup):
  TURTLEBOT3_MODEL=waffle LDS_MODEL=LDS-02
  ros2 launch turtlebot3_bringup robot.launch.py
  → /scan (LiDAR), /cmd_vel (base) 제공

주의: assisted_teleop 은 함께 실행하면 /cmd_vel 충돌 발생.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('shared_control')

    params = PathJoinSubstitution([pkg_share, 'config',
                                   'shared_control_params.yaml'])
    joy_yaml = PathJoinSubstitution([pkg_share, 'config',
                                     'teleop_twist_joy.yaml'])

    return LaunchDescription([
        # ── 입력 드라이버 ─────────────────────────────────────────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
        ),
        Node(
            package='tof_publisher',
            executable='tof_publisher',
            name='tof_publisher',
            output='screen',
            parameters=[{
                'port': '/dev/tof_sensor',   # 필요시 /dev/tof 등으로 변경
            }],
        ),

        # ── 조이스틱 → /cmd_vel_manual ───────────────────────────
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_yaml],
            remappings=[('/cmd_vel', '/cmd_vel_manual')],
        ),

        # ── shared_control 노드들 ────────────────────────────────
        Node(
            package='shared_control',
            executable='tof_bridge',
            name='tof_bridge',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='shared_control',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='shared_control',
            executable='gap_analyzer',
            name='gap_analyzer',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='shared_control',
            executable='avoidance',
            name='avoidance',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='shared_control',
            executable='arbitrator',
            name='arbitrator',
            output='screen',
            parameters=[params],
        ),
    ])
