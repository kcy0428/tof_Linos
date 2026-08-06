#!/usr/bin/env python3
"""mqtt.py — MQTT → ROS2 변환 브리지 (라파2).

tof.py 가 MQTT 로 발행한 제어 명령을 받아 ROS2 토픽으로 옮긴다.
**판단하지 않는다.** 받은 값을 그대로 Twist 로 옮기는 변환기다.
유일한 예외가 워치독(§8.3)인데, 이는 판단이 아니라 통신 장애 시 안전 정지다.

동작 순서:
    1. MQTT Subscribe   robot/cmd
    2. JSON Parsing     + 스키마 검증
    3. TwistStamped 생성
    4. ROS2 Topic Publish  /cmd_vel

⚠ geometry_msgs/Twist 가 아니라 TwistStamped 를 쓴다.
  ROS2 Jazzy 의 turtlebot3_node 는 TwistStamped 를 구독한다.
  plain Twist 로 발행하면 타입 불일치로 모터가 전혀 반응하지 않으며,
  에러도 없이 조용히 실패하므로 디버깅이 매우 어렵다.

실행:
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=<라파1과 동일>
    python3 mqtt.py

colcon 빌드가 필요 없다 (ROS 패키지 미생성 원칙). 수정하면 즉시 반영된다.
"""

import argparse
import json
import os
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


CONFIG = {
    'mqtt_host': '127.0.0.1',
    'mqtt_port': 1883,
    'mqtt_topic': 'robot/cmd',
    'mqtt_qos': 0,

    'ros_topic': '/cmd_vel',
    'frame_id': 'base_link',
    'publish_rate': 20.0,       # Hz
    'watchdog_timeout': 0.5,    # s 초과 시 정지 발행
}


def make_mqtt_client(client_id):
    """paho-mqtt v1/v2 양쪽 API 를 지원하는 클라이언트 생성."""
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print('paho-mqtt 미설치 — pip install paho-mqtt '
              '(또는 apt install python3-paho-mqtt)', file=sys.stderr)
        sys.exit(1)
    try:  # paho-mqtt >= 2.0
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except (AttributeError, TypeError):  # paho-mqtt 1.x
        return mqtt.Client(client_id=client_id)


class CmdBridge(Node):
    """MQTT 명령을 /cmd_vel (TwistStamped) 로 중계한다."""

    def __init__(self, cfg):
        super().__init__('tof_cmd_bridge')
        self.cfg = cfg

        self._lock = threading.Lock()
        self._lin = 0.0
        self._ang = 0.0
        self._mode = 'STOP'
        self._last_rx = 0.0        # 마지막 유효 수신 시각
        self._last_seq = None
        self._dropped = 0
        self._stale = True         # 워치독 상태 (시작 시 정지)
        self._warned = False

        self._pub = self.create_publisher(TwistStamped, cfg['ros_topic'], 10)
        self.create_timer(1.0 / cfg['publish_rate'], self._tick)

        self.get_logger().info(
            f'bridge: MQTT {cfg["mqtt_host"]}:{cfg["mqtt_port"]}/{cfg["mqtt_topic"]} '
            f'→ ROS2 {cfg["ros_topic"]} (TwistStamped) '
            f'watchdog={cfg["watchdog_timeout"]}s'
        )

    # ── MQTT 콜백 ────────────────────────────────────────────────────────
    def on_message(self, payload_bytes):
        """2. JSON Parsing + 스키마 검증."""
        try:
            data = json.loads(payload_bytes.decode('utf-8'))
        except (ValueError, UnicodeDecodeError) as e:
            self.get_logger().warn(f'JSON 파싱 실패: {e}')
            return

        if not isinstance(data, dict):
            self.get_logger().warn('JSON 최상위가 object 가 아님')
            return

        try:
            lin = float(data['linear_x'])
            ang = float(data['angular_z'])
        except (KeyError, TypeError, ValueError) as e:
            self.get_logger().warn(f'linear_x/angular_z 없음 또는 형변환 실패: {e}')
            return

        # NaN/inf 는 모터 드라이버를 망가뜨릴 수 있으므로 거부한다.
        if not (lin == lin and ang == ang) or abs(lin) == float('inf') or abs(ang) == float('inf'):
            self.get_logger().warn('NaN/inf 명령 거부')
            return

        seq = data.get('seq')
        with self._lock:
            if isinstance(seq, int) and self._last_seq is not None:
                gap = seq - self._last_seq
                if gap > 1:
                    self._dropped += gap - 1
                elif gap <= 0:
                    # 순서 역전/중복 — 오래된 명령은 버린다
                    return
            if isinstance(seq, int):
                self._last_seq = seq
            self._lin = lin
            self._ang = ang
            self._mode = str(data.get('mode', ''))
            self._last_rx = time.time()

    # ── 주기 발행 ────────────────────────────────────────────────────────
    def _tick(self):
        """3. TwistStamped 생성 → 4. ROS2 Publish (워치독 적용)."""
        now = time.time()
        with self._lock:
            age = now - self._last_rx if self._last_rx > 0 else float('inf')
            expired = age > self.cfg['watchdog_timeout']
            lin = 0.0 if expired else self._lin
            ang = 0.0 if expired else self._ang
            mode = self._mode

        # 워치독 상태 전이 로그 (매 프레임 찍지 않는다)
        if expired and not self._stale:
            self.get_logger().error(
                f'⚠ MQTT 명령 두절 ({age:.2f}s) — 정지 발행. tof.py 가 살아있는지 확인하세요.')
        elif not expired and self._stale:
            self.get_logger().info('MQTT 명령 복구 — 정상 발행 재개')
        self._stale = expired

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg['frame_id']
        msg.twist.linear.x = lin
        msg.twist.angular.z = ang
        self._pub.publish(msg)

        _ = mode  # 상태값은 로깅용으로만 보관

    def publish_stop(self, count=5):
        """정지 명령을 여러 번 발행한다 (종료 시 로봇이 계속 달리는 것 방지).

        rclpy 컨텍스트가 이미 내려간 뒤라면 발행이 불가능하므로 조용히 포기한다.
        (컨텍스트가 죽었다는 건 프로세스가 끝나간다는 뜻이고, 그때는 mqtt.py 가
         /cmd_vel 발행을 멈추므로 turtlebot3_node 쪽 타임아웃에 맡길 수밖에 없다.)
        """
        if not rclpy.ok():
            return False
        stop = TwistStamped()
        stop.header.frame_id = self.cfg['frame_id']
        try:
            for _ in range(count):
                stop.header.stamp = self.get_clock().now().to_msg()
                self._pub.publish(stop)
                time.sleep(0.02)
            return True
        except Exception:
            return False

    def stats(self):
        with self._lock:
            return self._last_seq, self._dropped


def main():
    ap = argparse.ArgumentParser(description='MQTT → ROS2 /cmd_vel 브리지')
    ap.add_argument('--mqtt-host', default=CONFIG['mqtt_host'])
    ap.add_argument('--mqtt-port', type=int, default=CONFIG['mqtt_port'])
    ap.add_argument('--topic', default=CONFIG['mqtt_topic'],
                    help='구독할 MQTT 토픽')
    ap.add_argument('--ros-topic', default=CONFIG['ros_topic'],
                    help='발행할 ROS2 토픽')
    ap.add_argument('--watchdog', type=float, default=CONFIG['watchdog_timeout'],
                    help='이 시간(초) 넘게 수신 없으면 정지 발행')
    args = ap.parse_args()

    cfg = dict(CONFIG)
    cfg['mqtt_host'] = args.mqtt_host
    cfg['mqtt_port'] = args.mqtt_port
    cfg['mqtt_topic'] = args.topic
    cfg['ros_topic'] = args.ros_topic
    cfg['watchdog_timeout'] = args.watchdog

    rclpy.init()
    node = CmdBridge(cfg)

    # ── 1. MQTT Subscribe ────────────────────────────────────────────────
    client = make_mqtt_client(f'bridge-{os.getpid()}')

    def on_connect(_client, _userdata, _flags, reason_code, *_rest):
        # paho v1 은 rc(int), v2 는 ReasonCode — 양쪽 모두 0/Success 가 정상
        ok = (reason_code == 0) or (getattr(reason_code, 'is_failure', False) is False)
        if ok:
            node.get_logger().info(f'MQTT 연결됨 — subscribe {cfg["mqtt_topic"]}')
            _client.subscribe(cfg['mqtt_topic'], qos=cfg['mqtt_qos'])
        else:
            node.get_logger().error(f'MQTT 연결 실패: {reason_code}')

    def on_disconnect(_client, _userdata, *_rest):
        # 재연결까지 워치독이 로봇을 정지시킨다.
        node.get_logger().warn('MQTT 연결 끊김 — 재연결 시도 중 (워치독이 정지 유지)')

    def on_message(_client, _userdata, msg):
        node.on_message(msg.payload)

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=5)

    try:
        client.connect(cfg['mqtt_host'], cfg['mqtt_port'], keepalive=10)
    except Exception as e:
        node.get_logger().error(
            f'MQTT 연결 실패 {cfg["mqtt_host"]}:{cfg["mqtt_port"]} — {e}\n'
            f'mosquitto 실행 여부를 확인하세요: systemctl status mosquitto')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    client.loop_start()

    # SIGTERM(kill, systemd stop, timeout) 으로 죽을 때도 정지 명령을 남긴다.
    # 기본 동작은 즉시 종료라서, 아무 조치 없으면 로봇이 마지막 명령으로 계속 달린다.
    def _on_term(_sig, _frm):
        node.publish_stop(3)
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_term)

    stopped = False
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stopped = node.publish_stop()
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if not stopped:
            # 컨텍스트가 살아 있으면 여기서라도 정지를 남긴다
            stopped = node.publish_stop()

        seq, dropped = node.stats()
        msg = f'종료 — 마지막 seq={seq}, 유실 추정={dropped}'
        if not stopped:
            msg += ' / ⚠ 정지 명령 발행 실패 (ROS 컨텍스트 종료됨)'
        print(msg, flush=True)

        client.loop_stop()
        client.disconnect()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
