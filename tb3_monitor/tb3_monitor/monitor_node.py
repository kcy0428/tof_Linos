#!/usr/bin/env python3
"""TurtleBot3 monitoring node with terminal display and RViz markers."""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


class TB3Monitor(Node):
    def __init__(self):
        super().__init__('tb3_monitor')

        # --- State storage ---
        self.robot_state = 'UNKNOWN'
        self.tof_distance = 0.0
        self.obstacle_detected = False
        self.obstacle_distance = 0.0
        self.gap_width = 0.0
        self.gap_passable = False
        self.cmd_vel_linear = 0.0
        self.cmd_vel_angular = 0.0
        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0

        # --- QoS ---
        qos_default = QoSProfile(depth=10)
        qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # --- Subscribers ---
        self.create_subscription(Float32, '/tof/front_distance', self._cb_tof, qos_default)
        self.create_subscription(Bool, '/obstacle/detected', self._cb_obs_det, qos_default)
        self.create_subscription(Float32, '/obstacle/distance', self._cb_obs_dist, qos_default)
        self.create_subscription(Float32, '/gap/width', self._cb_gap_w, qos_default)
        self.create_subscription(Bool, '/gap/passable', self._cb_gap_p, qos_default)
        self.create_subscription(String, '/robot/state', self._cb_state, qos_default)
        self.create_subscription(LaserScan, '/scan', self._cb_scan, qos_sensor)
        self.create_subscription(TwistStamped, '/cmd_vel', self._cb_cmd, qos_default)

        # --- Publishers ---
        self.marker_pub = self.create_publisher(MarkerArray, '/monitor/markers', 10)

        # --- Timers ---
        self.create_timer(0.5, self._publish_markers)
        self.create_timer(1.0, self._print_status)

        self.get_logger().info('TB3 Monitor node started')

    # ---- Callbacks ----
    def _cb_tof(self, msg):
        self.tof_distance = msg.data

    def _cb_obs_det(self, msg):
        self.obstacle_detected = msg.data

    def _cb_obs_dist(self, msg):
        self.obstacle_distance = msg.data

    def _cb_gap_w(self, msg):
        self.gap_width = msg.data

    def _cb_gap_p(self, msg):
        self.gap_passable = msg.data

    def _cb_state(self, msg):
        self.robot_state = msg.data

    def _cb_scan(self, msg):
        self.scan_ranges = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment

    def _cb_cmd(self, msg):
        self.cmd_vel_linear = msg.twist.linear.x
        self.cmd_vel_angular = msg.twist.angular.z

    # ---- Terminal display ----
    def _print_status(self):
        state_color = {
            'MANUAL': '\033[92m',      # green
            'AVOIDANCE': '\033[93m',    # yellow
            'STOPPED': '\033[91m',      # red
        }
        c = state_color.get(self.robot_state, '\033[97m')
        rst = '\033[0m'

        obs_str = f'\033[91mYES\033[0m' if self.obstacle_detected else f'\033[92mNO\033[0m'
        gap_str = f'\033[92mYES\033[0m' if self.gap_passable else f'\033[91mNO\033[0m'

        print('\033[2J\033[H', end='')  # clear screen
        print('=' * 50)
        print('     TB3 WAFFLE MONITOR')
        print('=' * 50)
        print(f'  Robot State     : {c}{self.robot_state}{rst}')
        print(f'  ToF Distance    : {self.tof_distance:.3f} m')
        print('-' * 50)
        print(f'  Obstacle Detect : {obs_str}')
        print(f'  Obstacle Dist   : {self.obstacle_distance:.3f} m')
        print(f'  Gap Width       : {self.gap_width:.3f} m')
        print(f'  Gap Passable    : {gap_str}')
        print('-' * 50)
        print(f'  Cmd Vel Linear  : {self.cmd_vel_linear:+.3f} m/s')
        print(f'  Cmd Vel Angular : {self.cmd_vel_angular:+.3f} rad/s')
        print('-' * 50)
        print(f'  LiDAR Rays      : {len(self.scan_ranges)}')
        print('=' * 50)

    # ---- Marker publishing ----
    def _publish_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 1) Obstacle distance sphere (red)
        if self.obstacle_distance > 0.01:
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = stamp
            m.ns = 'obstacle'
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = self.obstacle_distance
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.r = 1.0
            m.color.a = 0.9
            m.lifetime = Duration(sec=1)
            ma.markers.append(m)

        # 2) Obstacle distance text
        mt = Marker()
        mt.header.frame_id = 'base_link'
        mt.header.stamp = stamp
        mt.ns = 'obstacle_text'
        mt.id = 1
        mt.type = Marker.TEXT_VIEW_FACING
        mt.action = Marker.ADD
        mt.pose.position.x = self.obstacle_distance
        mt.pose.position.z = 0.3
        mt.pose.orientation.w = 1.0
        mt.scale.z = 0.12
        mt.color.r = 1.0
        mt.color.g = 1.0
        mt.color.b = 1.0
        mt.color.a = 1.0
        mt.text = f'Obs: {self.obstacle_distance:.2f}m'
        mt.lifetime = Duration(sec=1)
        ma.markers.append(mt)

        # 3) Gap center marker (green cylinder)
        if self.gap_width > 0.01 and self.scan_ranges:
            mg = Marker()
            mg.header.frame_id = 'base_link'
            mg.header.stamp = stamp
            mg.ns = 'gap'
            mg.id = 2
            mg.type = Marker.CYLINDER
            mg.action = Marker.ADD
            mg.pose.position.x = self.obstacle_distance
            mg.pose.orientation.w = 1.0
            mg.scale.x = float(self.gap_width)
            mg.scale.y = 0.05
            mg.scale.z = 0.3
            if self.gap_passable:
                mg.color.g = 1.0
            else:
                mg.color.r = 1.0
                mg.color.g = 0.5
            mg.color.a = 0.7
            mg.lifetime = Duration(sec=1)
            ma.markers.append(mg)

        # 4) Gap width text
        mgt = Marker()
        mgt.header.frame_id = 'base_link'
        mgt.header.stamp = stamp
        mgt.ns = 'gap_text'
        mgt.id = 3
        mgt.type = Marker.TEXT_VIEW_FACING
        mgt.action = Marker.ADD
        mgt.pose.position.x = self.obstacle_distance
        mgt.pose.position.z = 0.5
        mgt.pose.orientation.w = 1.0
        mgt.scale.z = 0.10
        mgt.color.r = 0.0
        mgt.color.g = 1.0
        mgt.color.b = 0.0
        mgt.color.a = 1.0
        mgt.text = f'Gap: {self.gap_width:.2f}m {"[PASS]" if self.gap_passable else "[BLOCK]"}'
        mgt.lifetime = Duration(sec=1)
        ma.markers.append(mgt)

        # 5) Robot state text above robot
        ms = Marker()
        ms.header.frame_id = 'base_link'
        ms.header.stamp = stamp
        ms.ns = 'robot_state'
        ms.id = 4
        ms.type = Marker.TEXT_VIEW_FACING
        ms.action = Marker.ADD
        ms.pose.position.z = 0.7
        ms.pose.orientation.w = 1.0
        ms.scale.z = 0.15
        if self.robot_state == 'MANUAL':
            ms.color.g = 1.0
        elif self.robot_state == 'AVOIDANCE':
            ms.color.r = 1.0
            ms.color.g = 1.0
        else:
            ms.color.r = 1.0
        ms.color.a = 1.0
        ms.text = f'[{self.robot_state}]'
        ms.lifetime = Duration(sec=1)
        ma.markers.append(ms)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = TB3Monitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
