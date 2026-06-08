"""Command arbitration & state machine.

States: MANUAL | AVOIDANCE | STOPPED.

  MANUAL    -- obstacle_detected=True ------------------> AVOIDANCE
  AVOIDANCE -- obstacle_detected=False -----------------> MANUAL
  AVOIDANCE -- /cmd_vel_auto stale ---------------------> STOPPED
  STOPPED   -- fresh /cmd_vel_auto received ------------> AVOIDANCE

Publishes:
  /cmd_vel     (TwistStamped)  ← Jazzy turtlebot3_node 요구 사항
  /robot/state (String)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, Float32, String


STATE_MANUAL = 'MANUAL'
STATE_AVOIDANCE = 'AVOIDANCE'
STATE_STOPPED = 'STOPPED'


class ArbitratorNode(Node):
    def __init__(self):
        super().__init__('arbitrator')

        self.declare_parameter('cmd_timeout', 0.3)
        self.declare_parameter('rate', 20.0)

        self._cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        rate = float(self.get_parameter('rate').value)

        self._manual_cmd = Twist()
        self._auto_cmd = Twist()
        self._manual_stamp = None
        self._auto_stamp = None
        self._obstacle = False
        self._state = STATE_MANUAL
        self._last_published_state = None

        self.create_subscription(Twist, '/cmd_vel_manual', self._on_manual, 10)
        self.create_subscription(Twist, '/cmd_vel_auto', self._on_auto, 10)
        self.create_subscription(Bool, '/obstacle/detected', self._on_obstacle, 10)
        self.create_subscription(Float32, '/tof/front_distance', self._on_tof, 10)

        self._pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self._pub_state = self.create_publisher(String, '/robot/state', 10)

        self.create_timer(1.0 / rate, self._tick)
        # 2 Hz heartbeat publishes current state even when unchanged
        self.create_timer(0.5, self._heartbeat_state)

        self.get_logger().info(
            f'arbitrator: initial state={self._state} cmd_timeout={self._cmd_timeout}s'
        )

    def _now(self):
        return self.get_clock().now()

    def _on_manual(self, msg: Twist):
        self._manual_cmd = msg
        self._manual_stamp = self._now()

    def _on_auto(self, msg: Twist):
        self._auto_cmd = msg
        self._auto_stamp = self._now()

    def _on_obstacle(self, msg: Bool):
        self._obstacle = bool(msg.data)

    def _on_tof(self, msg: Float32):
        # Telemetry only — obstacle_detector owns the latching logic.
        pass

    def _is_stale(self, stamp):
        if stamp is None:
            return True
        age = (self._now() - stamp).nanoseconds * 1e-9
        return age > self._cmd_timeout

    def _publish_state(self, new_state: str, *, force: bool = False):
        if force or new_state != self._last_published_state:
            s = String()
            s.data = new_state
            self._pub_state.publish(s)
            self._last_published_state = new_state
            self.get_logger().info(f'state -> {new_state}')

    def _heartbeat_state(self):
        s = String()
        s.data = self._state
        self._pub_state.publish(s)
        self._last_published_state = self._state

    def _tick(self):
        prev = self._state

        if self._obstacle:
            if self._is_stale(self._auto_stamp):
                self._state = STATE_STOPPED
            else:
                self._state = STATE_AVOIDANCE
        else:
            self._state = STATE_MANUAL

        if self._state != prev:
            self._publish_state(self._state)

        twist = Twist()
        if self._state == STATE_MANUAL:
            if not self._is_stale(self._manual_stamp):
                twist = self._manual_cmd
        elif self._state == STATE_AVOIDANCE:
            twist = self._auto_cmd
        # STOPPED → zero Twist (기본값 유지)

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = twist
        self._pub_cmd.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArbitratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
