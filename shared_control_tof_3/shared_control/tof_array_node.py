"""멀티 ToF 퍼블리셔.

3개의 VL53L8CX (각각 Raspberry Pi Pico 2, USB 시리얼) 를 동시에 읽어
방향별 단일 거리값을 sensor_msgs/Range 로 발행한다.

  /tof/front  (sensor_msgs/Range)  - 정면
  /tof/left   (sensor_msgs/Range)  - 정면 기준 좌 45°
  /tof/right  (sensor_msgs/Range)  - 정면 기준 우 45°

각 센서는 8x8 거리 그리드(mm)를 시리얼 텍스트로 출력하며,
중앙 ROI(rows 2~5, cols 2~5)의 최소거리를 그 방향의 대표 거리(m)로 사용한다.
(VL53L8CX 펌웨어 출력 포맷은 tof_publisher 패키지와 동일하다고 가정)

파라미터:
  port_front / port_left / port_right : 시리얼 장치 경로 (udev 심볼릭 링크 권장)
  baud            : 115200
  roi_rows        : [2, 5]   중앙 행 범위
  roi_cols        : [2, 5]   중앙 열 범위
  field_of_view   : 0.79 rad (VL53L8CX ~45°)
  min_range       : 0.02 m
  max_range       : 4.0 m
  stale_timeout   : 0.5 s    프레임 없으면 max_range(=clear) 발행
  publish_rate    : 20.0 Hz
"""

import math
import re
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


ROW_RE = re.compile(r'\s*R(\d)\s+(.*)')


class _SerialReader(threading.Thread):
    """한 ToF 시리얼 포트를 읽어 중앙 ROI 최소거리(m)를 유지하는 스레드."""

    def __init__(self, node, name, port, baud, roi, logger):
        super().__init__(daemon=True)
        self._node = node
        self.name_tag = name
        self._port = port
        self._baud = baud
        self._row_lo, self._row_hi, self._col_lo, self._col_hi = roi
        self._logger = logger

        self._lock = threading.Lock()
        self._min_m = float('inf')      # 최신 중앙 ROI 최소거리 (m)
        self._grid = [float('nan')] * 64  # 최신 8x8 거리 그리드 (mm), 무효는 NaN
        self._stamp = None              # 마지막 프레임 시각 (ns)
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def read(self):
        with self._lock:
            return self._min_m, self._stamp

    def read_grid(self):
        with self._lock:
            return list(self._grid)

    def run(self):
        try:
            import serial
        except ImportError:
            self._logger.error('pyserial 미설치: pip3 install pyserial')
            return

        while not self._stop.is_set():
            try:
                ser = serial.Serial(self._port, self._baud, timeout=1)
            except Exception as e:  # noqa: BLE001
                self._logger.warn(f'[{self.name_tag}] 포트 열기 실패 {self._port}: {e} (3초 후 재시도)')
                self._stop.wait(3.0)
                continue

            self._logger.info(f'[{self.name_tag}] 연결됨: {self._port}')
            buffer = []
            in_frame = False
            try:
                while not self._stop.is_set():
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    if 'Frame #' in line:
                        if in_frame and buffer:
                            self._consume(buffer)
                        in_frame = True
                        buffer = [line]
                    elif in_frame:
                        buffer.append(line)
                        if line.startswith('---'):
                            self._consume(buffer)
                            in_frame = False
                            buffer = []
            except Exception as e:  # noqa: BLE001
                self._logger.warn(f'[{self.name_tag}] 읽기 오류: {e} (재연결)')
            finally:
                try:
                    ser.close()
                except Exception:
                    pass

    def _consume(self, lines):
        rows = []
        for line in lines:
            m = ROW_RE.match(line)
            if not m:
                continue
            rows.append(m.group(2).split())
            if len(rows) >= 8:
                break
        if len(rows) < 8:
            return

        # 전체 8x8 그리드(mm) 구성 + 중앙 ROI 최소거리 계산
        grid = [float('nan')] * 64
        best = None
        for r in range(8):
            if r >= len(rows):
                continue
            for c in range(8):
                if c >= len(rows[r]):
                    continue
                v = rows[r][c]
                if v == '----':
                    continue
                try:
                    mm = float(v)
                except ValueError:
                    continue
                if mm <= 0:
                    continue
                grid[r * 8 + c] = mm
                # 중앙 ROI 안에서만 최소거리 갱신
                if (self._row_lo <= r <= self._row_hi
                        and self._col_lo <= c <= self._col_hi):
                    if best is None or mm < best:
                        best = mm

        with self._lock:
            self._grid = grid
            self._min_m = (best / 1000.0) if best is not None else float('inf')
            self._stamp = self._node.get_clock().now().nanoseconds


class TofArrayNode(Node):
    def __init__(self):
        super().__init__('tof_array')

        self.declare_parameter('port_front', '/dev/tof_front')
        self.declare_parameter('port_left', '/dev/tof_left')
        self.declare_parameter('port_right', '/dev/tof_right')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('roi_rows', [2, 4])   # 아래 3줄(바닥) 제외: R2~R4 (3행)
        self.declare_parameter('roi_cols', [0, 7])   # 전체 8열 → 8x3 영역
        self.declare_parameter('field_of_view', 0.79)
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('stale_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        baud = int(self.get_parameter('baud').value)
        rr = self.get_parameter('roi_rows').value
        rc = self.get_parameter('roi_cols').value
        roi = (int(rr[0]), int(rr[1]), int(rc[0]), int(rc[1]))
        self._fov = float(self.get_parameter('field_of_view').value)
        self._min_r = float(self.get_parameter('min_range').value)
        self._max_r = float(self.get_parameter('max_range').value)
        self._stale = float(self.get_parameter('stale_timeout').value)
        rate = float(self.get_parameter('publish_rate').value)

        log = self.get_logger()
        # (토픽, 파라미터명, frame_id)
        specs = [
            ('/tof/front', 'port_front', 'tof_front_link', 'front'),
            ('/tof/left', 'port_left', 'tof_left_link', 'left'),
            ('/tof/right', 'port_right', 'tof_right_link', 'right'),
        ]
        self._channels = []
        for topic, param, frame_id, tag in specs:
            port = self.get_parameter(param).value
            reader = _SerialReader(self, tag, port, baud, roi, log)
            pub = self.create_publisher(Range, topic, 10)
            grid_pub = self.create_publisher(Float32MultiArray, topic + '/grid', 10)
            reader.start()
            self._channels.append((reader, pub, grid_pub, frame_id))

        self.create_timer(1.0 / rate, self._tick)
        log.info(
            f'tof_array: front={specs[0]} 등 3채널 @ {rate:.0f}Hz '
            f'ROI rows[{roi[0]}..{roi[1]}] cols[{roi[2]}..{roi[3]}]'
        )

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        stamp = self.get_clock().now().to_msg()
        for reader, pub, grid_pub, frame_id in self._channels:
            dist, t = reader.read()
            stale = (t is None or (now_ns - t) * 1e-9 > self._stale)
            if stale:
                # 데이터 없음/오래됨(센서 미연결·펌웨어 이상) → 무효 센티넬 0.0
                # (sensor_msgs/Range 관례상 min_range 미만은 "유효하지 않은 측정")
                # 다운스트림(assist_controller)이 무효를 어떻게 다룰지 결정한다.
                dist = 0.0
            elif not math.isfinite(dist):
                dist = self._max_r
            else:
                dist = max(self._min_r, min(self._max_r, dist))

            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = self._fov
            msg.min_range = self._min_r
            msg.max_range = self._max_r
            msg.range = float(dist)
            pub.publish(msg)

            # 8x8 원본 그리드(mm) — 시각화/진단용
            grid = reader.read_grid()
            gmsg = Float32MultiArray()
            gmsg.layout.dim = [
                MultiArrayDimension(label='row', size=8, stride=64),
                MultiArrayDimension(label='col', size=8, stride=8),
            ]
            gmsg.layout.data_offset = 0
            gmsg.data = grid
            grid_pub.publish(gmsg)

    def destroy_node(self):
        for reader, *_ in self._channels:
            reader.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TofArrayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
