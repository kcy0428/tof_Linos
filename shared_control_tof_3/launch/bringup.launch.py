"""Bring up the ToF-based Assisted Teleoperation stack on TurtleBot3 Waffle.

이 launch 파일이 담당하는 노드:
  * joy_node          (블루투스 컨트롤러 → /joy)
  * teleop_twist_joy  (/joy → /cmd_vel_manual)
  * tof_array         (3개 VL53L8CX 시리얼 → /tof/front, /tof/left, /tof/right)
  * assist_controller (조이스틱 + 3 ToF 융합 → /cmd_vel, /robot/state)

별도로 실행되어야 하는 것 (TurtleBot3 bringup, 모터 구동용):
  TURTLEBOT3_MODEL=waffle
  ros2 launch turtlebot3_bringup robot.launch.py
  → /cmd_vel(TwistStamped) 를 받아 OpenCR 로 모터 제어

참고:
  * LiDAR 는 사용하지 않는다 (ToF 3개만으로 회피).
  * assisted_teleop / 기존 LiDAR 스택과 /cmd_vel 이 충돌하므로 함께 실행 금지.
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

        # ── 조이스틱 → /cmd_vel_manual ───────────────────────────
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_yaml],
            remappings=[('/cmd_vel', '/cmd_vel_manual')],
        ),

        # ── 3개 ToF 시리얼 → /tof/front,left,right (Range) ───────
        Node(
            package='shared_control',
            executable='tof_array',
            name='tof_array',
            output='screen',
            parameters=[params],
        ),

        # ── Assisted Teleop 컨트롤러 → /cmd_vel, /robot/state ────
        Node(
            package='shared_control',
            executable='assist_controller',
            name='assist_controller',
            output='screen',
            parameters=[params],
        ),
    ])
