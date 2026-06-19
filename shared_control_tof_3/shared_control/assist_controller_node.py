"""Assisted Teleoperation 컨트롤러 (ToF 3개 기반, LiDAR 미사용).

조이스틱 입력(/cmd_vel_manual)을 기본으로 하되, 전방 ToF 가 장애물을
감지하면 좌/우 ToF 를 비교해 더 넓은 쪽으로 조향하여 충돌만 회피한다.
완전 자율주행이 아니라 "사용자 조작 의도 유지 + 충돌 방지" 가 목표.

구독:
  /cmd_vel_manual (geometry_msgs/Twist) - teleop_twist_joy 출력
  /tof/front      (sensor_msgs/Range)   - 정면 거리
  /tof/left       (sensor_msgs/Range)   - 좌 45° 거리
  /tof/right      (sensor_msgs/Range)   - 우 45° 거리

발행:
  /cmd_vel     (geometry_msgs/TwistStamped) - 최종 모터 명령 (Jazzy turtlebot3_node)
  /robot/state (std_msgs/String) - NORMAL | AVOID_LEFT | AVOID_RIGHT | STOP

핵심 알고리즘 (프롬프트 + 개선안):

    manual = 최신 /cmd_vel_manual (timeout 지나면 0)
    engaged = 조이스틱에 의미있는 입력이 있는가
    forward = 사용자가 전진을 명령했는가 (manual.linear.x > eps)

    obstacle = front 막힘  OR  측면 벽 회피중(side_escaping)

    if not engaged:                      → STOP        (입력 없으면 정지)
    elif not obstacle:                   → NORMAL      (조이스틱 그대로: 전진/후진/회전 수동)
    elif not forward:                    → NORMAL      (전진 의도 없으면 회피 안 함)
    elif 좌·우 모두 side_min 이내:        → STOP        (양쪽 벽 → 갈 곳 없음)
    elif 측면 벽 회피중:                  → 더 가까운 벽에서 멀어지게 회전(전진0), side_clear 까지
    else:  # 전방 장애물 + 측면 여유
        left/right 상대 비교해 '더 열린 쪽'으로 비례 조향하며 전진 (빈 공간 탐색)
        front 가까울수록 회전↑(w_min→w_avoid)·전진↓, front_stop 이하면 전진0(제자리 회전)

  개선안:
    1) 히스테리시스: front<=enter 로 진입, front>exit(>enter) 까지 회피 유지 → 경계 떨림 방지
    2) 더 넓은 쪽 선택 + 동률 시 좌측 우선, "둘 다 막힘"일 때만 정지 (한쪽만 열려도 통과 시도)
    3) 자율 후진 금지: 로봇이 회피를 위해 스스로 후진하지 않는다(AVOID 분기는 항상 전진+회전).
       단, 사용자가 컨트롤러로 직접 후진하는 것은 NORMAL 에서 그대로 허용한다.
    4) 회피 전진속도는 사용자 명령과 v_avoid 중 작은 값 → 사용자 의도(느리게 가면 느리게) 유지
    5) 비례 조향: 전방 거리에 비례해 회전·전진 조절 → 부드러운 회피
    6) 측면 벽 회피: 좌/우 45° ToF 가 side_min(200mm) 이내면 그 벽에서 멀어지도록 회전(전진정지),
       side_clear 이상 벌어질 때까지 유지(히스테리시스) → 회피 중 옆구리 충돌 방지
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String


STATE_NORMAL = 'NORMAL'
STATE_AVOID_LEFT = 'AVOID_LEFT'
STATE_AVOID_RIGHT = 'AVOID_RIGHT'
STATE_STOP = 'STOP'


class AssistControllerNode(Node):
    def __init__(self):
        super().__init__('assist_controller')

        # 임계값
        self.declare_parameter('front_enter_threshold', 0.50)   # m  front<=이 값 → 회피 진입 (500mm)
        self.declare_parameter('front_exit_threshold', 0.65)    # m  front>이 값 → 회피 종료 (히스테리시스)
        # 측면(45° ToF)이 이 값 이내면 '벽이 너무 가까움'으로 보고, 그 벽에서 멀어지도록
        # 반대로 회전(전진 정지)한다. side_min + side_clear_margin 이상 벌어지면 해제(히스테리시스).
        # 양쪽 모두 이 값 이내면 갈 곳 없음 → 정지.
        self.declare_parameter('side_min_distance', 0.20)       # m (200mm)
        self.declare_parameter('side_clear_margin', 0.05)       # m  해제 여유
        # 전방이 이 값 이하이면 전진을 멈추고 제자리 회전으로 빈 공간을 탐색(충돌 방지).
        self.declare_parameter('front_stop_distance', 0.25)     # m
        # 회피 속도 (비례 조향)
        self.declare_parameter('v_avoid', 0.10)                 # m/s 회피 시 최대 전진속도
        self.declare_parameter('w_avoid', 0.6)                  # rad/s 최대 회전(가장 근접 시)
        self.declare_parameter('w_min', 0.15)                   # rad/s 최소 회전(막 감지 시)
        # 조이스틱 판정
        self.declare_parameter('forward_eps', 0.02)             # m/s 이 값 초과면 "전진 의도"
        self.declare_parameter('engage_eps', 0.02)              # 입력 유무 판정 (linear/angular 공통)
        self.declare_parameter('cmd_timeout', 0.3)              # s  manual stale 판정
        # 센서 무효(데이터 없음/미연결) 처리: True 면 그 방향을 '막힘'으로 간주(안전).
        # tof_array 는 stale 채널을 range=0.0(무효)으로 발행한다.
        self.declare_parameter('treat_stale_as_blocked', True)
        self.declare_parameter('rate', 20.0)

        self._enter = float(self.get_parameter('front_enter_threshold').value)
        self._exit = float(self.get_parameter('front_exit_threshold').value)
        self._side_min = float(self.get_parameter('side_min_distance').value)
        self._side_clear = self._side_min + float(self.get_parameter('side_clear_margin').value)
        self._front_stop = float(self.get_parameter('front_stop_distance').value)
        self._v_avoid = float(self.get_parameter('v_avoid').value)
        self._w_avoid = float(self.get_parameter('w_avoid').value)
        self._w_min = float(self.get_parameter('w_min').value)
        self._fwd_eps = float(self.get_parameter('forward_eps').value)
        self._eng_eps = float(self.get_parameter('engage_eps').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self._stale_blocked = bool(self.get_parameter('treat_stale_as_blocked').value)
        rate = float(self.get_parameter('rate').value)

        # 센서/입력 최신값
        self._front = float('inf')
        self._left = float('inf')
        self._right = float('inf')
        self._manual = Twist()
        self._manual_stamp = None

        # 상태
        self._state = STATE_STOP
        self._avoiding = False       # 전방 히스테리시스 래치
        self._side_escaping = False  # 측면 벽 회피 래치 (히스테리시스 on/off)

        # 구독
        self.create_subscription(Twist, '/cmd_vel_manual', self._on_manual, 10)
        self.create_subscription(Range, '/tof/front', self._on_front, 10)
        self.create_subscription(Range, '/tof/left', self._on_left, 10)
        self.create_subscription(Range, '/tof/right', self._on_right, 10)

        # 발행
        self._pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self._pub_state = self.create_publisher(String, '/robot/state', 10)

        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'assist_controller: enter<{self._enter:.2f}m exit>{self._exit:.2f}m '
            f'side_min<{self._side_min:.2f}m front_stop<{self._front_stop:.2f}m '
            f'v_avoid={self._v_avoid} w={self._w_min}~{self._w_avoid}(비례)'
        )

    # ── 콜백 ─────────────────────────────────────────────────────────────
    def _on_manual(self, msg: Twist):
        self._manual = msg
        self._manual_stamp = self.get_clock().now()

    def _range_m(self, msg: Range) -> float:
        r = float(msg.range)
        if not math.isfinite(r) or r <= 0.0:
            # 무효(데이터 없음/미연결). 안전 옵션이면 '막힘'(0.0), 아니면 '열림'(inf).
            return 0.0 if self._stale_blocked else float('inf')
        return r

    def _on_front(self, msg: Range):
        self._front = self._range_m(msg)

    def _on_left(self, msg: Range):
        self._left = self._range_m(msg)

    def _on_right(self, msg: Range):
        self._right = self._range_m(msg)

    # ── 유틸 ─────────────────────────────────────────────────────────────
    def _manual_fresh(self) -> bool:
        if self._manual_stamp is None:
            return False
        age = (self.get_clock().now() - self._manual_stamp).nanoseconds * 1e-9
        return age <= self._cmd_timeout

    def _front_blocked(self) -> bool:
        """히스테리시스 적용 전방 장애물 판정."""
        if self._front <= self._enter:
            self._avoiding = True
        elif self._front > self._exit:
            self._avoiding = False
        # enter~exit 사이면 이전 상태 유지
        return self._avoiding

    # ── 메인 루프 ────────────────────────────────────────────────────────
    def _tick(self):
        manual = self._manual if self._manual_fresh() else Twist()
        engaged = (abs(manual.linear.x) > self._eng_eps
                   or abs(manual.angular.z) > self._eng_eps)
        forward = manual.linear.x > self._fwd_eps

        out = Twist()

        front_block = self._front_blocked()
        left_close = self._left < self._side_min
        right_close = self._right < self._side_min
        nearest_side = min(self._left, self._right)

        # ── 측면 벽 회피 래치 (히스테리시스) ──────────────────────────────
        # 측면이 side_min(200mm) 이내로 들어오면 회피 시작, side_clear 이상
        # 벌어질 때까지 계속 벽에서 멀어지도록 회전한다.
        if nearest_side < self._side_min:
            self._side_escaping = True
        elif nearest_side > self._side_clear:
            self._side_escaping = False

        obstacle = front_block or self._side_escaping

        if not engaged:
            # 조이스틱 입력 없음 → 정지 (조이스틱 우선 원칙)
            state = STATE_STOP

        elif not obstacle:
            # 전방·측면 모두 안전 → 조이스틱 그대로 (전진/후진/회전 수동)
            state = STATE_NORMAL
            out = manual

        elif not forward:
            # 장애물 있으나 전진 의도 없음 (제자리 회전/후진 등) → 조이스틱대로
            state = STATE_NORMAL
            out = manual

        elif left_close and right_close:
            # 양쪽 벽 모두 200mm 이내 → 갈 곳 없음 → 정지
            state = STATE_STOP

        elif self._side_escaping:
            # 측면 벽이 가까움 → 더 가까운 벽에서 멀어지도록 회전, 전진 정지.
            #   side_clear 이상 벌어질 때까지 계속 회전(부딪힘 방지).
            if self._left <= self._right:
                state = STATE_AVOID_RIGHT          # 좌측 벽 → 우회전
                out.angular.z = -self._w_avoid
            else:
                state = STATE_AVOID_LEFT           # 우측 벽 → 좌회전
                out.angular.z = self._w_avoid
            out.linear.x = 0.0

        else:
            # 전방 장애물 + 측면 여유 → 더 열린 쪽으로 비례 조향(부드러운 회피).
            #   t = 근접도 0~1 (front=enter→0 약하게, front<=front_stop→1 최대+전진0)
            turn_left = self._left >= self._right       # 동률은 좌측 우선
            denom = max(1e-3, self._enter - self._front_stop)
            t = (self._enter - self._front) / denom
            t = max(0.0, min(1.0, t))

            w_mag = self._w_min + (self._w_avoid - self._w_min) * t
            if turn_left:
                state = STATE_AVOID_LEFT
                out.angular.z = w_mag        # +z = 좌회전(CCW)
            else:
                state = STATE_AVOID_RIGHT
                out.angular.z = -w_mag       # -z = 우회전(CW)
            # 가까울수록 전진속도 감소 (1-t), front_stop 이하면 전진 0 → 제자리 회전
            out.linear.x = min(manual.linear.x, self._v_avoid) * (1.0 - t)

        # 자율 후진 금지: AVOID 분기는 linear.x=min(manual>0, v_avoid) 라 구조적으로
        # 음수가 될 수 없다. NORMAL 에서는 사용자가 명령한 후진을 그대로 허용한다.

        # 발행
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.twist = out
        self._pub_cmd.publish(ts)

        if state != self._state:
            self.get_logger().info(
                f'{self._state} -> {state}  '
                f'(F={self._front:.2f} L={self._left:.2f} R={self._right:.2f})'
            )
            self._state = state
        self._pub_state.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = AssistControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
