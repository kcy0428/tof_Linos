"""Convert VL53L8CX 8x8 grid (/tof/distances + /tof/status) to a single
/tof/front_distance Float32 (meters)."""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, UInt8MultiArray


class TofBridgeNode(Node):
    def __init__(self):
        super().__init__('tof_bridge')

        self.declare_parameter('roi_rows', [2, 5])
        self.declare_parameter('roi_cols', [2, 5])
        self.declare_parameter('valid_status', [5, 9])
        self.declare_parameter('stale_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        roi_rows = self.get_parameter('roi_rows').value
        roi_cols = self.get_parameter('roi_cols').value
        self._row_lo, self._row_hi = int(roi_rows[0]), int(roi_rows[1])
        self._col_lo, self._col_hi = int(roi_cols[0]), int(roi_cols[1])
        self._valid_status = set(int(s) for s in self.get_parameter('valid_status').value)
        self._stale_timeout = float(self.get_parameter('stale_timeout').value)
        rate = float(self.get_parameter('publish_rate').value)

        self._last_distances = None
        self._last_status = None
        self._last_stamp = None

        self.create_subscription(Float32MultiArray, '/tof/distances',
                                 self._on_distances, 10)
        self.create_subscription(UInt8MultiArray, '/tof/status',
                                 self._on_status, 10)
        self._pub = self.create_publisher(Float32, '/tof/front_distance', 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'tof_bridge: ROI rows[{self._row_lo}..{self._row_hi}] '
            f'cols[{self._col_lo}..{self._col_hi}] valid={sorted(self._valid_status)}'
        )

    def _on_distances(self, msg: Float32MultiArray):
        self._last_distances = list(msg.data)
        self._last_stamp = self.get_clock().now()

    def _on_status(self, msg: UInt8MultiArray):
        self._last_status = list(msg.data)

    def _extract_front_distance(self):
        dist = self._last_distances
        if dist is None or len(dist) < 64:
            return float('inf')
        status = self._last_status if (self._last_status and len(self._last_status) >= 64) else None
        vals = []
        for r in range(self._row_lo, self._row_hi + 1):
            for c in range(self._col_lo, self._col_hi + 1):
                idx = r * 8 + c
                d_mm = dist[idx]
                # NaN(측정 불가) 또는 0 이하 제외
                if d_mm is None or not math.isfinite(d_mm) or d_mm <= 0.0:
                    continue
                # status가 있으면 확실히 잘못된 코드(255=없음)만 제외;
                # 0(초기값)·5·6·9·10·13 모두 수용
                if status is not None and int(status[idx]) == 255:
                    continue
                vals.append(d_mm / 1000.0)
        return min(vals) if vals else float('inf')

    def _tick(self):
        out = Float32()
        if self._last_stamp is None:
            out.data = float('inf')
        else:
            age = (self.get_clock().now() - self._last_stamp).nanoseconds * 1e-9
            if age > self._stale_timeout:
                out.data = float('inf')
            else:
                out.data = self._extract_front_distance()
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TofBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
