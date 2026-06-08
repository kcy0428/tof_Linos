"""Analyze LiDAR front sector (-30° to +30°), find the widest passable gap,
publish /gap/width (m) and /gap/passable (Bool)."""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class GapAnalyzerNode(Node):
    def __init__(self):
        super().__init__('gap_analyzer')

        self.declare_parameter('sector_deg', 30.0)
        self.declare_parameter('robot_width', 0.281)
        self.declare_parameter('margin', 0.10)
        self.declare_parameter('required_gap', 0.381)
        self.declare_parameter('rate', 10.0)

        self._sector = math.radians(float(self.get_parameter('sector_deg').value))
        self._robot_w = float(self.get_parameter('robot_width').value)
        self._margin = float(self.get_parameter('margin').value)
        self._required = float(self.get_parameter('required_gap').value)
        rate = float(self.get_parameter('rate').value)

        self._free_thresh = self._robot_w + self._margin  # 0.381 m default

        self._last_scan = None
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self._pub_width = self.create_publisher(Float32, '/gap/width', 10)
        self._pub_pass = self.create_publisher(Bool, '/gap/passable', 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'gap_analyzer: sector=±{math.degrees(self._sector):.0f}° '
            f'required_gap={self._required:.3f} m free_thresh={self._free_thresh:.3f} m'
        )

    def _on_scan(self, msg: LaserScan):
        self._last_scan = msg

    def _compute_gap(self, scan: LaserScan):
        if scan is None or not scan.ranges:
            return 0.0
        n = len(scan.ranges)
        ang = scan.angle_min
        inc = scan.angle_increment

        rmin_cfg = scan.range_min if scan.range_min > 0 else 0.05
        rmax_cfg = scan.range_max if scan.range_max > 0 else 12.0

        free_flags = []
        ranges_in_sector = []
        for i in range(n):
            na = math.atan2(math.sin(ang), math.cos(ang))
            if -self._sector <= na <= self._sector:
                r = scan.ranges[i]
                if math.isfinite(r) and rmin_cfg <= r <= rmax_cfg:
                    free_flags.append(r > self._free_thresh)
                    ranges_in_sector.append(r)
                else:
                    free_flags.append(True)
                    ranges_in_sector.append(rmax_cfg)
            ang += inc

        if not free_flags:
            return 0.0

        # longest contiguous True run
        best_len = 0
        best_lo = -1
        best_hi = -1
        run_lo = -1
        for i, f in enumerate(free_flags):
            if f:
                if run_lo < 0:
                    run_lo = i
                run_hi = i
                run_len = run_hi - run_lo + 1
                if run_len > best_len:
                    best_len = run_len
                    best_lo, best_hi = run_lo, run_hi
            else:
                run_lo = -1

        if best_len <= 0:
            return 0.0

        run_angle = best_len * inc
        r_min_in_run = min(ranges_in_sector[best_lo:best_hi + 1])
        # chord of arc subtended by run_angle at radius r_min_in_run
        width = 2.0 * r_min_in_run * math.sin(run_angle / 2.0)
        return width

    def _tick(self):
        width = self._compute_gap(self._last_scan)
        w_msg = Float32()
        w_msg.data = float(width)
        self._pub_width.publish(w_msg)
        p_msg = Bool()
        p_msg.data = bool(width >= self._required)
        self._pub_pass.publish(p_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GapAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
