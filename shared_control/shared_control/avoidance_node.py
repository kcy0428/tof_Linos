"""Reactive avoidance: when an obstacle is detected, drive through the gap if
passable; otherwise turn toward the freer side. Publishes /cmd_vel_auto."""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('avoidance')

        self.declare_parameter('v_forward', 0.08)
        self.declare_parameter('w_turn', 0.4)
        self.declare_parameter('back_v', -0.05)
        self.declare_parameter('side_window_deg', 60.0)
        self.declare_parameter('blocked_threshold', 0.25)
        self.declare_parameter('rate', 20.0)

        self._v_fwd = float(self.get_parameter('v_forward').value)
        self._w_turn = float(self.get_parameter('w_turn').value)
        self._back_v = float(self.get_parameter('back_v').value)
        self._side_win = math.radians(float(self.get_parameter('side_window_deg').value))
        self._blocked_th = float(self.get_parameter('blocked_threshold').value)
        rate = float(self.get_parameter('rate').value)

        self._obstacle = False
        self._gap_passable = False
        self._gap_width = 0.0
        self._scan = None

        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Bool, '/obstacle/detected', self._on_obstacle, 10)
        self.create_subscription(Bool, '/gap/passable', self._on_gap_pass, 10)
        self.create_subscription(Float32, '/gap/width', self._on_gap_width, 10)

        self._pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'avoidance: v_fwd={self._v_fwd} w_turn={self._w_turn} '
            f'side_window=±{math.degrees(self._side_win):.0f}°'
        )

    def _on_scan(self, msg: LaserScan):
        self._scan = msg

    def _on_obstacle(self, msg: Bool):
        self._obstacle = bool(msg.data)

    def _on_gap_pass(self, msg: Bool):
        self._gap_passable = bool(msg.data)

    def _on_gap_width(self, msg: Float32):
        self._gap_width = float(msg.data)

    @staticmethod
    def _norm(a):
        """각도를 [-π, +π] 범위로 정규화 (0~2π 입력 대응)."""
        return math.atan2(math.sin(a), math.cos(a))

    def _side_scores(self):
        scan = self._scan
        if scan is None or not scan.ranges:
            return 0.0, 0.0
        left_score = 0.0
        right_score = 0.0
        ang = scan.angle_min
        inc = scan.angle_increment
        for r in scan.ranges:
            na = self._norm(ang)
            if -self._side_win <= na <= self._side_win:
                if math.isfinite(r) and r > 0.01:
                    # 측정값 그대로 사용 (0.0 clamp 제거 → 가까운 벽도 낮은 점수 유지)
                    r_clip = min(5.0, r)
                elif not math.isfinite(r) or r == 0.0:
                    # inf/nan/0 → 측정 불가 = 열린 공간으로 간주
                    r_clip = 5.0
                else:
                    r_clip = 5.0
                if na > 0:
                    left_score += r_clip
                elif na < 0:
                    right_score += r_clip
            ang += inc
        return left_score, right_score

    def _min_front(self, half_angle):
        scan = self._scan
        if scan is None or not scan.ranges:
            return float('inf')
        best = float('inf')
        ang = scan.angle_min
        inc = scan.angle_increment
        for r in scan.ranges:
            if -half_angle <= self._norm(ang) <= half_angle:
                # 0.01m 이상이면 유효 측정값으로 처리 (기존 0.05m 기준은 LiDAR 맹점 유발)
                if math.isfinite(r) and r > 0.01 and r < best:
                    best = r
            ang += inc
        return best

    def _tick(self):
        cmd = Twist()
        if not self._obstacle:
            self._pub.publish(cmd)  # zero twist while idle
            return

        # If the gap directly ahead is wide enough, push slowly through it.
        if self._gap_passable:
            cmd.linear.x = self._v_fwd
            cmd.angular.z = 0.0
            self._pub.publish(cmd)
            return

        # 좌우 여유공간 비교 → 더 열린 쪽으로 회전
        left, right = self._side_scores()
        front_min = self._min_front(math.radians(20.0))

        # 전방이 너무 가까우면 제자리 회전, 아니면 저속 전진 + 회전
        if front_min < self._blocked_th:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = max(0.0, self._v_fwd * 0.3)

        # 양쪽 점수가 같으면 왼쪽 우선
        cmd.angular.z = self._w_turn if left >= right else -self._w_turn
        self._pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
