"""2단계 장애물 감지 + 측면 대각 보호.

1단계 (ToF 선행 감지):
  tof_front < enter_threshold  → tof_alert = True

2단계 (LiDAR 정면 확인):
  lidar_front_min < lidar_confirm_threshold  → lidar_alert = True

AVOIDANCE 진입 조건:
  (tof_alert AND lidar_alert)   ← 정면 2단계 확인
  OR side_alert                 ← 측면/대각 벽 감지 (즉시 진입)

MANUAL 복귀 조건:
  tof_front > exit_threshold
  AND lidar_front_min > exit_threshold
  AND NOT side_alert
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.declare_parameter('enter_threshold', 0.5)
        self.declare_parameter('exit_threshold', 0.8)
        self.declare_parameter('sector_deg', 30.0)
        self.declare_parameter('lidar_min_range', 0.12)
        self.declare_parameter('lidar_confirm_threshold', 0.5)  # LiDAR 정면 확인 거리
        self.declare_parameter('side_sector_lo_deg', 55.0)      # 측면 섹터 시작 (전방 기준)
        self.declare_parameter('side_sector_hi_deg', 120.0)     # 측면 섹터 끝
        self.declare_parameter('side_threshold', 0.30)          # 측면 경보 거리
        self.declare_parameter('rate', 20.0)

        self._enter   = float(self.get_parameter('enter_threshold').value)
        self._exit    = float(self.get_parameter('exit_threshold').value)
        self._sector  = math.radians(float(self.get_parameter('sector_deg').value))
        self._lidar_min = float(self.get_parameter('lidar_min_range').value)
        self._lidar_confirm = float(self.get_parameter('lidar_confirm_threshold').value)
        self._side_lo = math.radians(float(self.get_parameter('side_sector_lo_deg').value))
        self._side_hi = math.radians(float(self.get_parameter('side_sector_hi_deg').value))
        self._side_th = float(self.get_parameter('side_threshold').value)
        rate = float(self.get_parameter('rate').value)

        self._tof_front      = float('inf')
        self._lidar_front    = float('inf')
        self._lidar_side_min = float('inf')  # 좌우 측면 최솟값
        self._latched = False

        self.create_subscription(Float32,   '/tof/front_distance', self._on_tof,  10)
        self.create_subscription(LaserScan, '/scan',               self._on_scan, 10)

        self._pub_detected = self.create_publisher(Bool,    '/obstacle/detected', 10)
        self._pub_distance = self.create_publisher(Float32, '/obstacle/distance', 10)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'obstacle_detector: enter<{self._enter}m exit>{self._exit}m '
            f'lidar_confirm<{self._lidar_confirm}m '
            f'side[{math.degrees(self._side_lo):.0f}°~{math.degrees(self._side_hi):.0f}°]'
            f'<{self._side_th}m'
        )

    # ── 콜백 ──────────────────────────────────────────────────────────────
    def _on_tof(self, msg: Float32):
        self._tof_front = float(msg.data) if math.isfinite(msg.data) else float('inf')

    def _on_scan(self, msg: LaserScan):
        if not msg.ranges:
            self._lidar_front = self._lidar_side_min = float('inf')
            return

        inc      = msg.angle_increment
        rmin_cfg = max(self._lidar_min, msg.range_min)
        rmax_cfg = msg.range_max if msg.range_max > 0 else float('inf')

        best_front = float('inf')
        best_side  = float('inf')
        ang = msg.angle_min

        for r in msg.ranges:
            na = math.atan2(math.sin(ang), math.cos(ang))  # 정규화 → [-π, +π]
            abs_na = abs(na)

            if math.isfinite(r) and rmin_cfg <= r <= rmax_cfg:
                # 정면 섹터
                if abs_na <= self._sector:
                    if r < best_front:
                        best_front = r
                # 측면 섹터 (좌우 대칭)
                if self._side_lo <= abs_na <= self._side_hi:
                    if r < best_side:
                        best_side = r

            ang += inc

        self._lidar_front    = best_front
        self._lidar_side_min = best_side

    # ── 판단 로직 ─────────────────────────────────────────────────────────
    def _tick(self):
        # 1단계: ToF 선행 감지
        tof_alert = math.isfinite(self._tof_front) and self._tof_front < self._enter

        # 2단계: LiDAR 정면 확인 (3가지 경우 모두 "확인됨"으로 처리)
        #  a) LiDAR 가 정상 범위에서 장애물 측정
        lidar_measured  = math.isfinite(self._lidar_front) and self._lidar_front < self._lidar_confirm
        #  b) ToF 가 경보 중인데 LiDAR 값이 inf → 최소측정거리(0.12m) 이내에 있는 것
        #     즉, 너무 가까워서 LiDAR 가 못 잡는 상태 = 확실한 장애물
        lidar_blind_zone = tof_alert and not math.isfinite(self._lidar_front)
        lidar_alert = lidar_measured or lidar_blind_zone

        # 측면 대각 감지 (즉시 진입, ToF 확인 불필요)
        side_alert = math.isfinite(self._lidar_side_min) and self._lidar_side_min < self._side_th

        # AVOIDANCE 진입: (ToF + LiDAR 모두 확인) 또는 측면 벽 감지
        enter_cond = (tof_alert and lidar_alert) or side_alert
        # MANUAL 복귀: 정면 양쪽 모두 멀어지고 측면도 안전
        exit_cond  = (self._tof_front   > self._exit and
                      self._lidar_front > self._exit and
                      not side_alert)

        if not self._latched and enter_cond:
            self._latched = True
            reason = 'SIDE' if side_alert else f'FRONT(ToF={self._tof_front:.2f} LiDAR={self._lidar_front:.2f})'
            self.get_logger().info(f'obstacle ENTER [{reason}]')
        elif self._latched and exit_cond:
            self._latched = False
            self.get_logger().info(
                f'obstacle EXIT tof={self._tof_front:.2f} lidar={self._lidar_front:.2f}m')

        fused = min(self._tof_front, self._lidar_front, self._lidar_side_min)

        b = Bool()
        b.data = self._latched
        self._pub_detected.publish(b)

        d = Float32()
        d.data = float(fused) if math.isfinite(fused) else float('inf')
        self._pub_distance.publish(d)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
