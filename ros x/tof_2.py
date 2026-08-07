#!/usr/bin/env python3
"""tof_2.py — tof.py 의 장애물 회피를 3D 기하 기반으로 바꾼 실험 버전 (라파2).

**tof.py 는 건드리지 않는다.** 이 파일만 바꿔가며 비교한다.
`--geom` 으로 두 방식의 거리를 나란히 볼 수 있다.

전동 휠체어 프로젝트(`전동 휠체어 운전자 보조 시스템/`)를 분석해서 가져온 것:

  1. **셀 → 3D 점군 변환** (`SensorGeometry`). tof.py 는 센서마다 ROI 최소값
     하나로 줄였는데, 그 탓에 두 가지가 깨져 있었다:
       · 바닥이 장애물로 잡힌다 — front R4 가 바닥을 619mm 에서 만나
         회피 임계 500mm 까지 여유가 119mm 뿐이었다 (STATUS.md §2.4c).
       · 측면 거리가 사선 거리였다 — 45° 로 붙은 센서의 raw 값을
         side_min_distance 와 비교하는 건 단위가 다르다.
     높이(y)로 바닥을 거르면 첫째가, |x| 로 재면 둘째가 사라진다.
  2. **정면 거리 = 좁은 각도 밴드의 가까운 N개 평균** — 셀 하나의 최소값보다
     노이즈에 강하다.
  3. **측면 거리 = 실제 옆거리 |x|** (정면 원뿔 제외).

가져오지 **않은** 것과 이유:
  · 그쪽의 단차 대응(speed_scale 0.4 로 감속만, 전진 차단 없음) — 우리 요구사항
    "전진을 막고 회전·후진은 허용"이 더 강하고, 실측으로 검증까지 끝냈다.
  · 통로 탐지(passage scan) — 문틀 통과에는 유리하지만 튜닝 부담이 크다.
    지금 구조에 얹으려면 별도 검증이 필요해 보류했다.

바뀐 것이 하나 더 있다 — `side_min_distance` 의 **기준점**이 달라졌다.
tof.py 에서는 센서가 읽은 사선 거리, 여기서는 차체 옆구리에서의 여유다.
Controller 가 robot_half_width_mm 을 더해 환산한다. ⚠ 실주행 재튜닝 대상.

모든 판단을 이 프로세스 하나에서 수행하고, 결과를 MQTT 로 발행한다.
ROS2 는 전혀 사용하지 않는다 (발행은 mqtt.py 담당).

  ToF ×3 (시리얼)  ─┐
  컨트롤러 (evdev) ─┴─► [판단] ─► MQTT robot/cmd   (제어 명령, 20Hz)
                              └─► 내장 웹서버      (8x8 실시간 화면, 10Hz)
                                     http://<라파2 IP>:8080

판단 우선순위 (위에서부터, 먼저 걸리면 확정):
  1. 낭떠러지 감지      → 전진 차단, 회전·후진 허용        [최우선]
  2. 조이스틱 입력 없음  → 완전 정지
  3. 장애물 없음        → 조이스틱 그대로
  4. 전진 의도 없음      → 조이스틱 그대로
  5. 좌·우 모두 막힘     → 정면 뚫림이면 직진, 막혔으면 정지
  6. 측면 벽 근접       → 벽 반대로 회전, 전진 0
  7. 전방 장애물        → 더 열린 쪽으로 비례 조향

회피 로직은 shared_control_tof/shared_control/assist_controller_node.py 를
임계값·수식까지 그대로 이식한 것이다 (STATUS.md §2.2).

웹 화면은 이 프로세스에 내장돼 있지만 **제어 루프와 분리된 스레드**에서 돈다.
제어 루프가 하는 일은 최신 프레임 참조를 락 안에서 대입하는 것뿐이고(JSON 직렬화·
네트워크 전송은 전부 웹 스레드 몫), 브라우저가 몇 개 붙든 느리든 끊기든 루프는
기다리지 않는다. 포트를 못 열어도 경고만 남기고 제어는 그대로 진행한다.

사용법:
    python3 tof.py                 # 정상 동작 (MQTT 발행 + 웹 화면)
    python3 tof.py --dry-run       # MQTT 없이 콘솔 출력만 (배선 확인용, 웹은 동작)
    python3 tof.py --no-web        # 웹 화면 끄기
    python3 tof.py --web-port 8000 # 포트 변경
    python3 tof.py --list-joy      # 입력 장치 목록만 출력하고 종료
"""

import argparse
import json
import math
import os
import re
import select
import signal
import socket
import statistics
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ─────────────────────────────────────────────────────────────────────────────
# 설정
# ─────────────────────────────────────────────────────────────────────────────

CONFIG = {
    # ── ToF 입력 ──────────────────────────────────────────────────────────
    'port_front': '/dev/tof_front',
    'port_left': '/dev/tof_left',
    'port_right': '/dev/tof_right',
    'baud': 115200,
    'obstacle_roi_rows': [2, 4],    # 장애물: 바닥 제외 상단 3행
    'obstacle_roi_cols': [0, 7],
    # 낭떠러지 ROI 는 하단 2행만 쓴다. R5 는 바닥과 배경의 경계라 장면에 따라
    # 원경이 섞여 들어와 중앙값을 흔든다 (실측: front R5 395 / R6 309 / R7 247).
    # R6·R7 은 항상 바닥만 보므로 판정이 훨씬 안정적이다.
    'cliff_roi_rows': [6, 7],       # 낭떠러지: 바닥만 확실히 보는 하단 2행
    'cliff_roi_cols': [0, 7],
    # 채널별 예외. 측면 센서는 45° 바깥을 향해 **R6 이 바닥을 지나쳐 배경을 본다**
    # (실측: left R6 유효 3/8 에 1379mm 배경 혼입, right 5/8, front 는 8/8 정상).
    # 신뢰할 수 있는 R7 만 쓰면 평지 유효율이 8/8 로 올라가 여유가 훨씬 커진다.
    'cliff_roi_rows_left': [7, 7],
    'cliff_roi_rows_right': [7, 7],
    'stale_timeout': 0.5,           # s 프레임 없으면 무효 처리

    # ── 장애물 회피 (기존 프로젝트 값 그대로) ──────────────────────────────
    'front_enter_threshold': 0.50,  # m  이하 → 회피 진입
    'front_exit_threshold': 0.65,   # m  초과 → 회피 해제 (히스테리시스)
    # tof_2: **차체 옆구리에서의 여유**로 의미가 바뀌었다 (tof.py 는 센서가 읽은
    # 사선 거리였다). Controller 가 robot_half_width_mm 을 더해서 중심축 기준으로
    # 환산한다. ⚠ 실주행에서 재튜닝 대상 — 값의 기준점이 달라졌다.
    'side_min_distance': 0.15,      # m  측면 여유 (차체 옆면 기준)
    'robot_half_width_mm': 140.0,   # TurtleBot3 Waffle 폭 281mm 의 절반
    'side_clear_margin': 0.05,      # m  측면 회피 해제 여유
    'front_stop_distance': 0.20,    # m  정면 완전 막힘
    'v_avoid': 0.10,                # m/s 회피 최대 전진속도
    'w_avoid': 0.6,                 # rad/s 최대 회전 (가장 근접)
    'w_min': 0.15,                  # rad/s 최소 회전 (막 감지)
    # ── 3D 기하 (tof_2 신규) ──────────────────────────────────────────────
    # 좌우 장착각 — **실측 보정값** (2026-08-06, `--calib-wall`).
    # CLAUDE.md/README 는 ±45° 라고 적고 있으나 **실물은 그렇지 않다.**
    # 두 가지 독립 방법이 일치했다:
    #   ① 벽 평면 역산: 좌 -10.2° / 우 +14.0°, 잔차 91→21 / 85→19mm 로 급감
    #      (front 는 정의상 0° 여야 하는데 -0.2° 로 나와 장면 유효성도 확인)
    #   ② 최소거리 비: 좌 0.988 / 우 0.965 (±45° 라면 1.082 여야 함)
    # 재측정: `python3 tof_2.py --calib-wall` (평평한 벽 정면, 0.4~1.0m)
    'mount_phi_front': 0.0,
    'mount_phi_left': -10.2,
    'mount_phi_right': +14.0,
    # 상하 장착각(아래로 기울면 음수). STATUS.md §2.4b 형상 역산에서 유도:
    #   행 중심각 = (3.5 - row) × 5.625°,  R6 실측 내림각 - 14.06° = 장착각
    #   front 21.6°-14.06° = 7.5° / left 15.1° → 1.0° / right 16.2° → 2.1°
    'mount_theta_front': -7.5,
    'mount_theta_left': -1.0,
    'mount_theta_right': -2.1,
    # 바닥에서 센서까지 높이 [mm] (§2.4b 역산값)
    'sensor_height_front': 111.0,
    'sensor_height_left': 105.0,
    'sensor_height_right': 102.0,
    # 이 높이 이하의 점은 바닥으로 보고 버린다. 문턱처럼 낮은 장애물을 살리려면
    # 낮추고, 바닥 반사가 장애물로 잡히면 올린다.
    'floor_margin_mm': 45.0,
    # 정면 거리: 이 반각 안의 점들 중 가까운 N개 평균
    'front_band_half_deg': 8.5,
    'front_n_closest': 3,
    # 옆거리를 볼 전방 z 범위 [mm] — 뒤나 먼 벽은 제외
    'lateral_z_range': (0.0, 700.0),
    # 옆거리에서 제외할 정면 원뿔 반각 [도]. 이걸 안 빼면 바로 앞 장애물이
    # x 부호만으로 '옆벽 3.8cm' 가 되어 상시 측면 회피가 걸린다 (실측 확인).
    'lateral_min_az_deg': 20.0,
    # 좌/우 채널을 무엇으로 잴 것인가.
    #   'sector'  — 좌/우 섹터의 전방거리 min z. **현재 장착(±12°) 기본값.**
    #               시야가 69° 뿐이라 진짜 옆거리는 관측되지 않는다.
    #   'lateral' — 실제 옆거리 |x|. 센서를 ±45° 로 재장착하면 이쪽으로 바꿀 것.
    'side_metric': 'sector',
    # 좌우 섹터에서 **정면 원뿔을 제외**하는 반각. 0 으로 두면 정면 장애물이
    # 좌우 양쪽 섹터에 다 들어가서, 중앙에서 조금만 치우쳐도 "그쪽이 막혔다"가
    # 되어 **막힌 쪽으로 조향**한다. 정확히 중앙이면 매 프레임 좌우가 뒤집혀
    # 제자리에서 떤다 (실주행에서 4번 중 1번만 성공한 원인).
    # 정면 밴드(±8.5°) 바깥으로 잡는다.
    'sector_dead_deg': 12.0,
    # 조향 방향을 한 번 정하면, 반대쪽이 이만큼 더 열려야 바꾼다 [m].
    # 좌우 거리차가 작을 때 방향이 떠는 것을 막는다 (실측 중앙값 40mm).
    'side_prefer_margin': 0.12,
    # 3D 계산에서 제외할 행 (천장/먼 배경만 보는 최상단)
    'geom_skip_rows': (0,),

    # 프레임 자체가 안 오는 채널(미연결/고장)을 '막힘'으로 간주 (안전).
    'treat_stale_as_blocked': True,
    # 프레임은 정상인데 장애물 ROI 셀이 전부 무효인 경우의 해석.
    #   True  → '사거리 내 물체 없음' = 열림 (정상 동작에 필요)
    #   False → 막힘 (가장 보수적. 검은색/흡수성 표면을 놓칠 위험을 없애지만,
    #           빈 공간을 향한 센서가 영구히 막힘으로 잡혀 그쪽 회피가 불가능해진다)
    # 센서가 프레임을 보내고 있다는 것 자체가 '살아있다'는 증거이므로 True 가 기본.
    'empty_roi_as_open': True,

    # ── 낭떠러지 ──────────────────────────────────────────────────────────
    # 절대 기준. 평지 바닥이 약 300~390mm 이므로 800mm 는 2배 이상 여유가 있다.
    # 실측 변동폭이 10mm 이내라 오탐 여지가 거의 없고, 판정 지점이 장면·기준선과
    # 무관하게 항상 같아서 예측 가능하다.
    'cliff_abs_mm': 800.0,          # 이 거리를 넘으면 낭떠러지
    # 기준선 대비 상대 판정. None = 사용 안 함.
    # 절대 기준과 같이 켜면 (delta 120 → 약 420mm 에서 트립) 상대 쪽이 항상 먼저
    # 걸려 절대 기준이 무의미해진다. 그래서 기본은 절대 기준 단독이다.
    'cliff_delta_mm': None,
    # 유효셀 비율이 이 미만이면 낭떠러지. **실측상 이쪽이 주 감지 경로다** —
    # 낭떠러지에서는 빔이 허공으로 나가 셀이 하나씩 무효가 되는데, 남은 셀은
    # 여전히 바닥을 보므로 중앙값은 잘 안 오른다. 실측 6회 전부 이 경로로 잡혔다.
    # 0.4 → 0.6 상향: 평지 유효율이 front 16/16, 측면 8/8 이라 여유가 충분하고
    # front 기준 감지가 1.71초 빨라진다 (실측).
    'cliff_min_valid_ratio': 0.6,
    'cliff_baseline_alpha': 0.05,   # 기준선 EMA 계수
    'cliff_on_frames': 3,           # 연속 이만큼 → 감지 확정 (빠른 진입)
    'cliff_off_frames': 10,         # 연속 이만큼 정상 → 해제 (느린 해제)
    'cliff_warmup_frames': 20,      # 기준선 학습 대기 프레임
    'cliff_enabled': True,

    # ── 조이스틱 ──────────────────────────────────────────────────────────
    'joy_device': 'auto',           # 'auto' 또는 '/dev/input/eventN'
    'axis_linear': 'auto',          # 'auto' | 'ABS_Y' | 'ABS_HAT0Y' ...
    'axis_angular': 'auto',
    'invert_linear': True,          # 스틱 위 = 음수인 장치가 대부분
    'invert_angular': True,         # 스틱 오른쪽 = 양수 → 우회전은 음수
    # 낭떠러지 감지 실측 12cm 기준 제동 여유 (STATUS.md §6):
    #   0.10 m/s → 84~89mm  /  0.15 m/s → 58~70mm  /  0.20 m/s → 28~48mm
    # 0.15 는 절충값. 낭떠러지 근처에서 더 여유가 필요하면 0.10 으로 낮출 것.
    'scale_linear': 0.15,           # m/s 최대 전진속도
    'scale_angular': 1.0,           # rad/s 최대 회전속도
    'deadzone': 0.05,
    'engage_eps': 0.02,             # 입력 유무 판정
    'forward_eps': 0.02,            # 전진 의도 판정
    # 리더 스레드가 죽거나 장치가 사라지면 이 시간 뒤 입력 0 으로 간주한다.
    # '조작이 없는 시간'이 아니다 — 스틱을 고정해도 하트비트가 계속 갱신된다
    # (JoystickReader._pump 주석 참조). 하트비트 주기 0.15s 의 3배 여유.
    'joy_timeout': 0.5,

    # ── 최종 안전 클램프 ───────────────────────────────────────────────────
    'v_max': 0.22,                  # m/s 전진 상한
    'v_min': -0.15,                 # m/s 후진 하한 (음수)
    'w_max': 1.5,                   # rad/s 회전 상한

    # ── MQTT ──────────────────────────────────────────────────────────────
    'mqtt_host': '127.0.0.1',
    'mqtt_port': 1883,
    'mqtt_topic': 'robot/cmd',
    'mqtt_grid_topic': 'robot/tof/grid',
    'mqtt_qos': 0,
    'grid_rate': 10.0,              # Hz 시각화 갱신 주기 (웹 / grid 토픽 공통)

    # ── 내장 웹 화면 ───────────────────────────────────────────────────────
    'web_enabled': True,
    'web_host': '0.0.0.0',          # 0.0.0.0 = 같은 네트워크 어디서나 접속
    'web_port': 8080,
    # 셀 색 구간. 임계값을 제어 파라미터에서 그대로 끌어오므로 색 경계 = 로봇의
    # 실제 판정 경계다. 색이 예뻐 보이려고 나눈 게 아니라 '이 셀이 무슨 뜻인지'를
    # 나타낸다. 마지막 두 구간만 표시 목적으로 임의 지정한다.
    'web_band_far_mm': 1500,        # 이 위는 '안전', 아래는 '여유'
    'web_stale_sec': 1.0,           # 이 시간 갱신 없으면 '멈춤'으로 표시
    'sysmon_interval': 1.0,         # s CPU·메모리 샘플 주기

    'rate': 20.0,                   # Hz 메인 루프
}

MODE_CLIFF = 'CLIFF_BLOCK'
MODE_STOP = 'STOP'
MODE_NORMAL = 'NORMAL'
MODE_AVOID_LEFT = 'AVOID_LEFT'
MODE_AVOID_RIGHT = 'AVOID_RIGHT'

CHANNELS = ('front', 'left', 'right')
ROW_RE = re.compile(r'\s*R(\d)\s+(.*)')


def log(msg):
    print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# ─────────────────────────────────────────────────────────────────────────────
# ToF 시리얼 리더
# ─────────────────────────────────────────────────────────────────────────────

class SerialReader(threading.Thread):
    """ToF 시리얼 포트 하나를 읽어 8x8 그리드(mm)를 유지하는 스레드.

    프레임 포맷 (VL53L8CX 펌웨어):
        Frame #123
        R0  120  130  ---- ...
        ...
        R7  ...
        --------
    '----' 또는 0 이하 값은 무효(None)로 둔다.
    포트가 끊기면 3초 후 자동 재연결한다.
    """

    def __init__(self, tag, port, baud):
        super().__init__(daemon=True)
        self.tag = tag
        self._port = port
        self._baud = baud
        self._lock = threading.Lock()
        self._grid = [None] * 64
        self._stamp = 0.0
        self._connected = False
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def snapshot(self):
        """(grid, stamp, connected) 를 원자적으로 반환."""
        with self._lock:
            return list(self._grid), self._stamp, self._connected

    def run(self):
        try:
            import serial
        except ImportError:
            log(f'[{self.tag}] pyserial 미설치 — pip install pyserial')
            return

        while not self._stop.is_set():
            try:
                ser = serial.Serial(self._port, self._baud, timeout=1)
            except Exception as e:
                with self._lock:
                    self._connected = False
                log(f'[{self.tag}] 포트 열기 실패 {self._port}: {e} (3초 후 재시도)')
                self._stop.wait(3.0)
                continue

            with self._lock:
                self._connected = True
            log(f'[{self.tag}] 연결됨: {self._port}')

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
            except Exception as e:
                log(f'[{self.tag}] 읽기 오류: {e} (재연결)')
            finally:
                with self._lock:
                    self._connected = False
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

        grid = [None] * 64
        for r in range(8):
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

        with self._lock:
            self._grid = grid
            self._stamp = time.time()


def roi_values(grid, rows, cols):
    """ROI 안의 유효값(mm) 리스트와 ROI 전체 셀 수를 반환."""
    r_lo, r_hi = rows
    c_lo, c_hi = cols
    vals = []
    total = 0
    for r in range(r_lo, r_hi + 1):
        for c in range(c_lo, c_hi + 1):
            total += 1
            v = grid[r * 8 + c]
            if v is not None:
                vals.append(v)
    return vals, total


def row_medians(grid, rows, cols):
    """ROI 각 행의 중앙값 리스트 (유효값 없는 행은 None)."""
    r_lo, r_hi = rows
    c_lo, c_hi = cols
    out = []
    for r in range(r_lo, r_hi + 1):
        vals = [grid[r * 8 + c] for c in range(c_lo, c_hi + 1)
                if grid[r * 8 + c] is not None]
        out.append(statistics.median(vals) if vals else None)
    return out


# ─────────────────────────────────────────────────────────────────────────────
# 3D 점군 기하 (tof_2 신규)
# ─────────────────────────────────────────────────────────────────────────────
#
# tof.py 는 센서마다 ROI 최소값 하나로 줄여서 썼다. 그게 두 가지 문제를 만든다:
#
#   1) **바닥이 장애물로 잡힌다.** front 의 R4 는 이 장착각에서 바닥을 619mm 에서
#      만나므로 빈 복도에서도 front 거리가 619mm 를 못 넘는다. 회피 진입 임계
#      500mm 까지 여유가 119mm 뿐이라, 센서를 2.5° 만 더 숙이면 상시 오탐이 된다.
#      (STATUS.md §2.4c)
#   2) **측면 거리가 사선 거리다.** 45° 로 붙은 센서의 raw 거리는 '옆으로 얼마나
#      떨어졌나'가 아니다. side_min_distance 를 그 값과 비교하는 건 단위가 다르다.
#
# 두 문제 모두 셀을 3D 좌표로 펴면 사라진다. 셀마다 방향이 정해져 있으므로
# 단위벡터를 **기동 시 한 번만** 계산해 두면, 매 프레임 곱셈 192번이면 끝이다
# (휠체어 프로젝트는 매 프레임 삼각함수를 다시 돈다 — numpy 를 써도 그쪽이 더 무겁다).
#
# 좌표계 (로봇 기준, 원점은 **바닥**):
#     x = 좌우 (오른쪽 +)   y = 높이 (위 +, 바닥이 0)   z = 전방 (앞 +)

class SensorGeometry:
    """센서 하나의 64셀 → 로봇 좌표 단위벡터. 기동 시 1회 계산."""

    def __init__(self, mount_phi_deg, mount_theta_deg, height_mm,
                 fov_phi_deg=45.0, fov_theta_deg=45.0, offset=(0.0, 0.0)):
        self.height = float(height_mm)
        self.off_x, self.off_z = offset
        hp = math.radians(fov_phi_deg / 2.0)
        ht = math.radians(fov_theta_deg / 2.0)
        mp = math.radians(mount_phi_deg)
        mt = math.radians(mount_theta_deg)
        self.unit = []          # [(ux, uy, uz), ...] 64개, row-major
        self.abs_az = []        # 로봇 기준 절대 방위각 [도]
        for i in range(8):
            for j in range(8):
                # 셀 중심의 센서 로컬 각도 (FoV 를 8칸에 균등 분할)
                phi = hp * (j * 2 - 7) / 7.0            # 좌우, 오른쪽 +
                theta = ht * (-i * 2 + 7) / 7.0 + mt    # 상하, 위 +  (+장착 기울기)
                # 센서 로컬 → 정면이 +z
                lx = math.sin(phi) * math.cos(theta)
                ly = math.sin(theta)
                lz = math.cos(phi) * math.cos(theta)
                # 좌우 장착각만큼 회전 (+z 를 +x 쪽으로)
                self.unit.append((lx * math.cos(mp) + lz * math.sin(mp),
                                  ly,
                                  -lx * math.sin(mp) + lz * math.cos(mp)))
                self.abs_az.append(math.degrees(phi) + mount_phi_deg)

    def points(self, grid, floor_margin_mm, skip_rows=()):
        """유효 셀을 (x, y, z, abs_az) 로 편다. y 는 **바닥 기준 높이**.

        바닥 높이 이하의 점은 버린다 — 이게 '바닥을 장애물로 보는' 문제의
        근본 해결이다. 행 번호로 자르는 방식과 달리 장착각이 바뀌어도 따라간다.
        """
        out = []
        for idx, mm in enumerate(grid):
            if mm is None:
                continue
            if (idx >> 3) in skip_rows:
                continue
            ux, uy, uz = self.unit[idx]
            y = self.height + mm * uy          # 바닥 기준 높이
            if y <= floor_margin_mm:
                continue                        # 바닥(또는 바닥 반사)
            out.append((mm * ux + self.off_x, y, mm * uz + self.off_z,
                        self.abs_az[idx]))
        return out


def front_distance_3d(points, half_angle_deg, n_closest):
    """정면 좁은 각도 밴드에서 가장 가까운 N개 z 의 평균 [m].

    tof.py 는 셀 하나의 최소값을 썼다. 노이즈 셀 하나에 회피가 걸릴 수 있고,
    실제로 바닥 셀 하나가 계속 최소값을 잡고 있었다. 평균이 훨씬 안정적이다.
    """
    zs = [z for (_x, _y, z, az) in points if abs(az) <= half_angle_deg]
    if not zs:
        return None
    zs.sort()
    n = min(n_closest, len(zs))
    return sum(zs[:n]) / n / 1000.0


def lateral_distances_3d(points, z_range, min_az_deg):
    """좌/우 **실제 옆거리** |x| 의 최소값 [m]. 사선 거리가 아니다.

    두 가지로 걸러야 의미가 생긴다:
      · z_range — 전방 이 범위 안. 뒤나 아주 먼 벽은 지금 판단과 무관하다.
      · min_az_deg — **정면 원뿔은 제외한다.** 이걸 빼먹으면 바로 앞의 장애물이
        x 가 조금만 양수여도 '오른쪽 벽 3.8cm' 로 잡힌다(실제로 그랬다).
        정면 물체는 front 거리가 담당하고, 여기는 '옆을 스칠 위험'만 본다.
    """
    lo, hi = z_range
    left = right = None
    for (x, _y, z, az) in points:
        if not (lo <= z <= hi) or abs(az) < min_az_deg:
            continue
        if x < 0:
            d = -x
            left = d if left is None else min(left, d)
        elif x > 0:
            right = x if right is None else min(right, x)
    return (None if left is None else left / 1000.0,
            None if right is None else right / 1000.0)


def sector_distances_3d(points, dead_deg=0.0):
    """좌/우 **섹터 전방거리** min z [m]. '그쪽으로 틀면 얼마나 갈 수 있나'.

    왜 옆거리(|x|) 대신 이걸 쓰는가 — 실물 장착이 ±12° 라 전체 시야가 69° 뿐이고,
    로봇 바로 옆(z≈0)은 세 센서 모두 못 본다. 그 상태에서 |x| 를 재면 '옆 물체'가
    아니라 **정면 벽의 가장자리**가 잡힌다 (정면 벽 0.58m → 옆거리 0.21m 로 보고).
    임계 0.29m 와 비교하면 정면에 벽만 있어도 상시 측면 회피가 걸린다.

    섹터 최소 z 는 시야가 좁아도 의미가 유지되고, 단위가 전방거리라
    front 임계값들과 같은 척도다. 센서를 ±45° 로 재장착하면 |x| 쪽이 더 낫다
    (`side_metric` 으로 전환).
    """
    left = right = None
    for (_x, _y, z, az) in points:
        if az < -dead_deg:
            left = z if left is None else min(left, z)
        elif az > dead_deg:
            right = z if right is None else min(right, z)
    return (None if left is None else left / 1000.0,
            None if right is None else right / 1000.0)


def floor_geometry(meds):
    """행 중앙값이 '바닥을 내려다보는' 모양인지 판정.

    센서가 바닥을 비스듬히 보면 아래 행일수록 거리가 짧아진다(단조 감소).
    벽·배경을 보고 있으면 행마다 거의 같거나 들쭉날쭉하다.
    절대 거리로는 장착 높이·각도를 모르니 판정할 수 없어서 **모양**으로 본다.

    반환: (ok, 사유)
    """
    if any(m is None for m in meds):
        return False, '유효값 없는 행이 있음'
    if len(meds) < 2:
        # 기울기를 만들 행이 없으면 형상 판정 자체가 성립하지 않는다.
        # (호출부가 검사용으로 한 행 위까지 넓혀서 넘겨주므로 보통 여기 안 온다)
        return True, ''
    drops = [meds[i] - meds[i + 1] for i in range(len(meds) - 1)]
    if any(d <= 0 for d in drops):
        return False, '아래 행이 더 멀다 (바닥이 아니라 배경을 보는 중)'
    # 바닥이라면 위아래 행 사이에 뚜렷한 차이가 있어야 한다.
    if (meds[0] - meds[-1]) < 30.0:
        return False, f'행간 차이 {meds[0] - meds[-1]:.0f}mm 로 너무 평평함'
    return True, ''


# ─────────────────────────────────────────────────────────────────────────────
# 낭떠러지 감지
# ─────────────────────────────────────────────────────────────────────────────

class CliffDetector:
    """하단 3행의 바닥 거리 변화로 낭떠러지를 판정한다 (STATUS.md §2.4).

    평지에서 하단 행은 바닥을 일정 거리로 안정되게 읽는다. 낭떠러지에서는
      (a) 거리가 급격히 증가하거나
      (b) 반사가 없어 무효값이 급증한다.
    센서 장착 각도·높이가 로봇마다 달라 절대 임계값을 못 쓰므로,
    평지 바닥 거리를 EMA 로 학습한 '적응형 기준선' 대비 변화량으로 판정한다.

    진입은 빠르게(3프레임), 해제는 느리게(10프레임) — 안전 방향 비대칭.
    """

    def __init__(self, cfg, tag):
        self.tag = tag
        self._abs = (None if cfg.get('cliff_abs_mm') is None
                     else float(cfg['cliff_abs_mm']))
        self._delta = (None if cfg.get('cliff_delta_mm') is None
                       else float(cfg['cliff_delta_mm']))
        self._min_valid = float(cfg['cliff_min_valid_ratio'])
        self._alpha = float(cfg['cliff_baseline_alpha'])
        self._on_frames = int(cfg['cliff_on_frames'])
        self._off_frames = int(cfg['cliff_off_frames'])
        self._warmup = int(cfg['cliff_warmup_frames'])

        self.baseline = None
        self.latched = False
        self._on_count = 0
        self._off_count = 0
        self._warm = []
        self.last_median = None
        self.last_ratio = 0.0
        self.floor_ok = None        # 워밍업 시 바닥을 본 게 맞는가 (None=미판정)
        self.floor_reason = ''

    @property
    def ready(self):
        return self.baseline is not None

    def update(self, values, total, meds=None):
        """ROI 유효값(mm) 리스트로 상태를 갱신하고 latched 를 반환.

        meds: ROI 행별 중앙값. 워밍업이 실제로 바닥을 봤는지 검증하는 데만 쓴다.
        """
        ratio = (len(values) / total) if total else 0.0
        self.last_ratio = ratio
        med = statistics.median(values) if values else None
        self.last_median = med

        # 유효셀이 너무 적음 = 빔이 바닥을 못 만남 → 낭떠러지 신호
        raw = ratio < self._min_valid

        if not raw and med is not None:
            # 절대 기준은 기준선과 무관하므로 워밍업 중에도 적용한다.
            # 낭떠러지 위에서 켜도 잡히고, 그 값을 평지로 학습하지도 않는다.
            if self._abs is not None and med > self._abs:
                raw = True
            elif self.baseline is None:
                # 워밍업: 평지라고 가정하고 기준선을 학습한다.
                self._warm.append(med)
                if len(self._warm) >= self._warmup:
                    self.baseline = statistics.median(self._warm)
                    log(f'[{self.tag}] 낭떠러지 기준선 학습 완료: {self.baseline:.0f}mm')
                    # 학습한 게 정말 바닥인지 확인한다. 로봇을 들거나 기울인 채로
                    # 켜면 배경(벽)을 '평지'로 학습해버리는데, 그러면 낭떠러지
                    # 판정이 통째로 무의미해지면서도 아무 에러가 안 난다.
                    if meds is not None:
                        self.floor_ok, self.floor_reason = floor_geometry(meds)
                        if not self.floor_ok:
                            rowtxt = ' '.join('--' if m is None else f'{m:.0f}'
                                              for m in meds)
                            log(f'[{self.tag}] ⚠ 기준선이 바닥이 아닐 수 있음 — '
                                f'{self.floor_reason}')
                            log(f'[{self.tag}]   낭떠러지 ROI 행중앙값: {rowtxt} mm '
                                f'(바닥이면 아래로 갈수록 짧아져야 함)')
                            log(f'[{self.tag}]   로봇을 평지에 내려놓고 tof.py 를 '
                                f'다시 시작하세요.')
                    # 바닥이 절대 임계값에 너무 가까우면 여유가 없다는 뜻이다.
                    # (센서를 더 높이 달았거나 각도가 얕아진 경우)
                    if self._abs is not None and self.baseline > self._abs * 0.7:
                        log(f'[{self.tag}] ⚠ 바닥({self.baseline:.0f}mm)이 낭떠러지 '
                            f'임계값({self._abs:.0f}mm)에 근접 — 오탐 위험. '
                            f'센서 각도를 낮추거나 cliff_abs_mm 을 올리세요.')
            elif self._delta is not None:
                raw = (med - self.baseline) > self._delta

        # 디바운스 (비대칭 히스테리시스)
        if raw:
            self._on_count += 1
            self._off_count = 0
            if self._on_count >= self._on_frames:
                if not self.latched:
                    if ratio < self._min_valid:
                        why = f'유효셀 {ratio:.0%} < {self._min_valid:.0%} (반사 없음)'
                    elif self._abs is not None and med is not None and med > self._abs:
                        why = f'{round(med)}mm > 임계 {round(self._abs)}mm'
                    else:
                        why = (f'{round(med)}mm, 기준선 대비 '
                               f'+{round(med - self.baseline)}mm')
                    log(f'[{self.tag}] ⚠ 낭떠러지 감지 — {why}')
                self.latched = True
        else:
            self._off_count += 1
            self._on_count = 0
            if self._off_count >= self._off_frames:
                if self.latched:
                    log(f'[{self.tag}] 낭떠러지 해제')
                self.latched = False
            # 정상 상태에서만 기준선을 천천히 따라간다 (드리프트 보정)
            if not self.latched and self.baseline is not None and med is not None:
                self.baseline = (1 - self._alpha) * self.baseline + self._alpha * med

        return self.latched


# ─────────────────────────────────────────────────────────────────────────────
# 조이스틱
# ─────────────────────────────────────────────────────────────────────────────

class JoystickReader(threading.Thread):
    """evdev 로 게임패드를 직접 읽어 (linear_x, angular_z) 를 유지한다.

    장치가 없거나 빠지면 값을 0 으로 두고 계속 재탐색한다 (fail-safe).
    """

    POLL = 0.15     # s 이벤트가 없을 때 장치 생존을 확인하는 주기

    def __init__(self, cfg):
        super().__init__(daemon=True)
        self._cfg = cfg
        self._lock = threading.Lock()
        self._lin = 0.0
        self._ang = 0.0
        self._stamp = 0.0
        self._name = None
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def read(self):
        """(linear_x, angular_z, engaged) — 타임아웃 적용."""
        cfg = self._cfg
        with self._lock:
            lin, ang, stamp, = self._lin, self._ang, self._stamp
        if stamp <= 0 or (time.time() - stamp) > cfg['joy_timeout']:
            return 0.0, 0.0, False
        engaged = (abs(lin) > cfg['engage_eps'] or abs(ang) > cfg['engage_eps'])
        return lin, ang, engaged

    @property
    def device_name(self):
        return self._name

    def run(self):
        try:
            from evdev import InputDevice, ecodes, list_devices
        except ImportError:
            log('evdev 미설치 — sudo apt install python3-evdev '
                '(조이스틱 입력 없음 → 로봇은 정지 상태 유지)')
            return

        while not self._stop.is_set():
            dev = self._open(InputDevice, ecodes, list_devices)
            if dev is None:
                self._stop.wait(2.0)
                continue
            try:
                self._pump(dev, ecodes)
            except Exception as e:
                log(f'조이스틱 읽기 오류: {e} (재탐색)')
            finally:
                with self._lock:
                    self._lin = self._ang = 0.0
                    self._stamp = 0.0
                self._name = None
                try:
                    dev.close()
                except Exception:
                    pass

    def _open(self, InputDevice, ecodes, list_devices):
        want = self._cfg['joy_device']
        paths = [want] if want != 'auto' else list_devices()
        for path in paths:
            try:
                dev = InputDevice(path)
            except Exception:
                continue
            caps = dev.capabilities()
            abs_codes = [c for c, _ in caps.get(ecodes.EV_ABS, [])]
            if ecodes.ABS_X in abs_codes or ecodes.ABS_HAT0X in abs_codes:
                self._name = f'{dev.name} ({dev.path})'
                log(f'조이스틱 연결됨: {self._name}')
                return dev
            dev.close()
        return None

    def _resolve_axes(self, dev, ecodes):
        """설정에 따라 (linear축, 각속도축) 코드를 결정한다."""
        caps = dev.capabilities()
        abs_codes = [c for c, _ in caps.get(ecodes.EV_ABS, [])]

        def pick(name, fallbacks):
            if name != 'auto':
                return getattr(ecodes, name, None)
            for fb in fallbacks:
                code = getattr(ecodes, fb)
                if code in abs_codes:
                    return code
            return None

        lin = pick(self._cfg['axis_linear'], ['ABS_Y', 'ABS_HAT0Y'])
        ang = pick(self._cfg['axis_angular'], ['ABS_X', 'ABS_HAT0X'])
        return lin, ang

    def _pump(self, dev, ecodes):
        cfg = self._cfg
        lin_code, ang_code = self._resolve_axes(dev, ecodes)
        if lin_code is None or ang_code is None:
            log('조이스틱 축을 찾지 못함 — axis_linear/axis_angular 를 지정하세요')
            self._stop.wait(3.0)
            return

        info = {}
        for code in (lin_code, ang_code):
            try:
                a = dev.absinfo(code)
                mid = (a.min + a.max) / 2.0
                half = (a.max - a.min) / 2.0
                info[code] = (mid, half if half > 0 else 1.0)
            except Exception:
                info[code] = (0.0, 1.0)

        raw = {lin_code: 0.0, ang_code: 0.0}
        dz = cfg['deadzone']

        # 연결 시점에 스틱이 이미 꺾여 있으면, 놓았다 다시 잡기 전까지는
        # evdev 가 이벤트를 안 줘서 로봇이 안 움직인다. 값을 그대로 가져다 쓰면
        # 실행하자마자 로봇이 튀어나가므로(재연결 시엔 더 위험) 그렇게 하지 않고,
        # 사용자에게 왜 안 움직이는지 알려준다.
        for code, name in ((lin_code, '전진'), (ang_code, '회전')):
            try:
                mid, half = info[code]
                n = abs(clamp((dev.absinfo(code).value - mid) / half, -1.0, 1.0))
            except Exception:
                continue
            if n > dz:
                log(f'⚠ 연결 시점에 {name} 스틱이 중립이 아닙니다 ({n:.0%} 꺾임). '
                    f'중립으로 놓았다가 다시 조작하세요 — 그전까지는 정지 상태입니다.')

        def norm(code, value):
            mid, half = info[code]
            n = clamp((value - mid) / half, -1.0, 1.0)
            if abs(n) < dz:
                return 0.0
            # 데드존 바깥을 다시 0~1 로 펴서 불연속을 없앤다
            return math.copysign((abs(n) - dz) / (1.0 - dz), n)

        def store():
            lin = raw[lin_code] * cfg['scale_linear']
            ang = raw[ang_code] * cfg['scale_angular']
            if cfg['invert_linear']:
                lin = -lin
            if cfg['invert_angular']:
                ang = -ang
            with self._lock:
                self._lin = lin
                self._ang = ang
                self._stamp = time.time()

        # ── read_loop() 대신 select 로 도는 이유 ──────────────────────────
        # evdev 는 값이 '바뀔 때만' 이벤트를 낸다. 스틱을 한 방향으로 고정하면
        # 아무 이벤트도 안 온다. 이벤트 도착 시각으로만 타임아웃을 재면
        # '가만히 잡고 있는 것'을 '컨트롤러가 끊긴 것'으로 오판해서, 조금씩
        # 흔들어야만 움직이는 현상이 생긴다.
        #
        # 그래서 이벤트가 없는 동안에도 **장치가 살아있으면** 하트비트로
        # 타임스탬프를 갱신한다. joy_timeout 은 이제 '입력이 없다'가 아니라
        # '리더 스레드가 죽었거나 장치가 사라졌다'를 뜻한다 — 원래 의도한 것.
        store()   # 첫 하트비트 (아직 조작 전이어도 장치는 살아있다)
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select([dev.fd], [], [], self.POLL)
            except (OSError, ValueError):
                return                      # fd 가 닫힘 = 장치 사라짐
            if ready:
                for event in dev.read():
                    if event.type != ecodes.EV_ABS or event.code not in raw:
                        continue
                    raw[event.code] = norm(event.code, event.value)
                store()
            else:
                # 이벤트가 없다 → 장치 존재를 직접 확인하고 하트비트만 갱신
                if not os.path.exists(dev.path):
                    log('조이스틱 연결 끊김 — 재탐색')
                    return
                with self._lock:
                    self._stamp = time.time()


def list_input_devices():
    try:
        from evdev import InputDevice, ecodes, list_devices
    except ImportError:
        print('evdev 미설치 — sudo apt install python3-evdev')
        return
    found = False
    for path in list_devices():
        try:
            dev = InputDevice(path)
        except Exception as e:
            print(f'{path}: 열기 실패 ({e})')
            continue
        caps = dev.capabilities(verbose=False)
        abs_codes = [c for c, _ in caps.get(ecodes.EV_ABS, [])]
        names = [ecodes.ABS.get(c, str(c)) for c in abs_codes]
        print(f'{dev.path}  {dev.name}')
        if names:
            print(f'    ABS 축: {", ".join(str(n) for n in names)}')
        found = True
        dev.close()
    if not found:
        print('입력 장치를 찾지 못했습니다. 컨트롤러 연결과 input 그룹 권한을 확인하세요.')


# ─────────────────────────────────────────────────────────────────────────────
# 제어 로직 (기존 assist_controller_node.py 이식)
# ─────────────────────────────────────────────────────────────────────────────

class Controller:
    def __init__(self, cfg):
        self.cfg = cfg
        self._enter = cfg['front_enter_threshold']
        self._exit = cfg['front_exit_threshold']
        # tof_2: 옆거리가 **로봇 중심축 기준 |x|** 로 바뀌었다. tof.py 에서는
        # 센서가 읽은 사선 거리였다. 그래서 임계값에 로봇 반폭을 더해야
        # side_min_distance 가 원래 의도대로 '내 옆구리에서 얼마나 남았나'가 된다.
        # 이걸 빼먹으면 0.15 가 '중심에서 15cm' = 차체에서 1cm 가 되어버린다.
        # 'lateral' 은 로봇 중심축 기준 |x| 라 반폭을 더해야 '옆구리 여유'가 된다.
        # 'sector' 는 전방거리라 front 임계들과 같은 척도 — 보정하지 않는다.
        half = (cfg['robot_half_width_mm'] / 1000.0
                if cfg.get('side_metric') == 'lateral' else 0.0)
        self._side_min = cfg['side_min_distance'] + half
        self._side_clear = self._side_min + cfg['side_clear_margin']
        self._front_stop = cfg['front_stop_distance']
        self._avoiding = False        # 전방 히스테리시스 래치
        self._side_escaping = False   # 측면 벽 회피 래치
        self._turn_dir = None         # 조향 방향 커밋 ('left'/'right')
        self._prefer_margin = cfg.get('side_prefer_margin', 0.0)

    def _pick_dir(self, left, right):
        """조향 방향을 정하되, 한 번 정하면 반대쪽이 확실히 더 열릴 때까지 유지한다.

        좌우 거리차가 작으면(실측 중앙값 40mm) 노이즈로 방향이 매 프레임 뒤집혀
        로봇이 제자리에서 떤다. 커밋 + 마진으로 그걸 막는다.
        """
        mg = self._prefer_margin
        if self._turn_dir is None:
            self._turn_dir = 'left' if left >= right else 'right'
        elif self._turn_dir == 'left' and right > left + mg:
            self._turn_dir = 'right'
        elif self._turn_dir == 'right' and left > right + mg:
            self._turn_dir = 'left'
        return self._turn_dir == 'left'

    def _front_blocked(self, front):
        if front <= self._enter:
            self._avoiding = True
        elif front > self._exit:
            self._avoiding = False
        # enter~exit 사이면 이전 상태 유지
        return self._avoiding

    def step(self, dist, cliff_any, manual_lin, manual_ang, engaged):
        """(linear_x, angular_z, mode) 를 반환."""
        cfg = self.cfg
        front, left, right = dist['front'], dist['left'], dist['right']
        forward = manual_lin > cfg['forward_eps']

        # ── 1. 낭떠러지 (최우선) ──────────────────────────────────────────
        # min(x, 0) 한 줄로 규칙 4개가 모두 성립한다:
        #   전진(양수)→0 차단 / 후진(음수)→통과 / 회전→그대로 / 래치 유지
        if cliff_any:
            if not engaged:
                return 0.0, 0.0, MODE_STOP
            return min(manual_lin, 0.0), manual_ang, MODE_CLIFF

        # ── 2. 조이스틱 입력 없음 ─────────────────────────────────────────
        if not engaged:
            return 0.0, 0.0, MODE_STOP

        front_block = self._front_blocked(front)
        front_fully_blocked = front <= self._front_stop
        left_close = left < self._side_min
        right_close = right < self._side_min
        nearest_side = min(left, right)

        # 측면 벽 회피 래치 (히스테리시스)
        if nearest_side < self._side_min:
            self._side_escaping = True
        elif nearest_side > self._side_clear:
            self._side_escaping = False

        obstacle = front_block or self._side_escaping

        # ── 3. 장애물 없음 ────────────────────────────────────────────────
        if not obstacle:
            self._turn_dir = None        # 다음 회피 때 방향을 새로 정한다
            return manual_lin, manual_ang, MODE_NORMAL

        # ── 4. 전진 의도 없음 (제자리 회전·후진) ──────────────────────────
        if not forward:
            return manual_lin, manual_ang, MODE_NORMAL

        # ── 5. 좌·우 모두 막힘 ────────────────────────────────────────────
        if left_close and right_close:
            if front_fully_blocked:
                return 0.0, 0.0, MODE_STOP          # 갈 곳 없음
            # 정면은 뚫림 → 회전 없이 직진 통과 (좁은 통로)
            return min(manual_lin, cfg['v_avoid']), 0.0, MODE_NORMAL

        # ── 6. 측면 벽 근접 → 벽에서 멀어지게 회전, 전진 0 ────────────────
        if self._side_escaping:
            if self._pick_dir(left, right):
                return 0.0, cfg['w_avoid'], MODE_AVOID_LEFT     # 좌측이 열림 → 좌회전
            return 0.0, -cfg['w_avoid'], MODE_AVOID_RIGHT

        # ── 7. 전방 장애물 → 더 열린 쪽으로 비례 조향 ─────────────────────
        turn_left = self._pick_dir(left, right)      # 커밋 + 마진 (떨림 방지)
        denom = max(1e-3, self._enter - self._front_stop)
        t = clamp((self._enter - front) / denom, 0.0, 1.0)
        w_mag = cfg['w_min'] + (cfg['w_avoid'] - cfg['w_min']) * t
        lin = min(manual_lin, cfg['v_avoid']) * (1.0 - t)

        if turn_left:
            return lin, w_mag, MODE_AVOID_LEFT       # +z = 좌회전(CCW)
        return lin, -w_mag, MODE_AVOID_RIGHT


# ─────────────────────────────────────────────────────────────────────────────
# 시스템 모니터 (CPU·메모리)
# ─────────────────────────────────────────────────────────────────────────────

class SysMonitor(threading.Thread):
    """1초마다 /proc 를 읽어 tof.py·mqtt.py 와 시스템 전체 상태를 추적한다.

    psutil 같은 외부 의존성 없이 /proc 파일 몇 개만 읽는다. 1Hz 라 비용이
    사실상 0 이고, 제어 루프와 무관한 별도 스레드라 20Hz 주기에 영향이 없다.

    CPU% 는 top 과 같은 기준이다 — **코어 1개 100% 기준**. 라파5 는 4코어이므로
    한 프로세스가 400% 까지 나올 수 있고, 400% 를 다 쓰는 게 아니라
    50% 는 '4코어 중 half core' 를 뜻한다.

    mqtt.py 가 안 보이면 `procs['mqtt.py']` 가 None 이 된다. 이건 단순 정보가
    아니라 **안전 신호**다 — mqtt.py 가 죽으면 라파1 에 명령이 전혀 안 간다.
    """

    HZ = os.sysconf('SC_CLK_TCK') if hasattr(os, 'sysconf') else 100
    PAGE_KB = (os.sysconf('SC_PAGE_SIZE') // 1024) if hasattr(os, 'sysconf') else 4

    def __init__(self, interval=1.0, watch=('mqtt.py',)):
        super().__init__(daemon=True)
        self._interval = interval
        self._watch = tuple(watch)
        self._lock = threading.Lock()
        self._snap = None
        self._prev_proc = {}     # name -> (pid, cpu_ticks, wall)
        self._prev_sys = None    # (busy, total)
        self._stop = threading.Event()
        self._cpu_count = os.cpu_count() or 1

    def stop(self):
        self._stop.set()

    def snapshot(self):
        with self._lock:
            return self._snap

    # ── /proc 읽기 헬퍼 ───────────────────────────────────────────────────
    @staticmethod
    def _read(path):
        try:
            with open(path) as f:
                return f.read()
        except OSError:
            return None

    def _find_pid(self, script):
        """script 를 실행 중인 **파이썬** 프로세스의 pid. 자기 자신은 제외.

        단순히 cmdline 에 'mqtt.py' 가 들어있는지만 보면 안 된다. 그 이름을
        인자로 달고 있는 셸 래퍼(`bash -c '... python3 mqtt.py'`)나 편집기까지
        잡혀서, 정작 mqtt.py 가 죽어도 살아있는 것처럼 보인다.
        그래서 (1) 실행 파일이 python 이고 (2) 인자 중 basename 이 정확히
        일치하는 것이 있을 때만 인정한다.
        """
        me = os.getpid()
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            pid = int(entry)
            if pid == me:
                continue
            comm = self._read(f'/proc/{pid}/comm')
            if not comm or not comm.strip().startswith('python'):
                continue
            cmd = self._read(f'/proc/{pid}/cmdline')
            if not cmd:
                continue
            if any(os.path.basename(a) == script
                   for a in cmd.split('\0') if a):
                return pid
        return None

    def _proc_stat(self, name, pid, now):
        """(pid, cpu%, rss_mb, threads) 또는 None."""
        stat = self._read(f'/proc/{pid}/stat')
        if stat is None:
            return None
        # comm 에 공백·괄호가 들어갈 수 있어 마지막 ')' 뒤부터 자른다
        try:
            fields = stat[stat.rindex(')') + 2:].split()
            utime, stime = int(fields[11]), int(fields[12])
            threads = int(fields[17])
            rss_pages = int(fields[21])
        except (ValueError, IndexError):
            return None

        ticks = utime + stime
        cpu = 0.0
        prev = self._prev_proc.get(name)
        if prev and prev[0] == pid and now > prev[2]:
            cpu = (ticks - prev[1]) / self.HZ / (now - prev[2]) * 100.0
        self._prev_proc[name] = (pid, ticks, now)
        return {
            'pid': pid,
            'cpu_pct': round(max(0.0, cpu), 1),
            'rss_mb': round(rss_pages * self.PAGE_KB / 1024.0, 1),
            'threads': threads,
        }

    def _system(self):
        out = {'cpu_pct': 0.0, 'mem_used_mb': 0, 'mem_total_mb': 0,
               'mem_pct': 0.0, 'load1': 0.0, 'temp_c': None}

        stat = self._read('/proc/stat')
        if stat:
            f = [int(x) for x in stat.split('\n', 1)[0].split()[1:]]
            idle = f[3] + (f[4] if len(f) > 4 else 0)     # idle + iowait
            total = sum(f)
            busy = total - idle
            if self._prev_sys:
                db, dt = busy - self._prev_sys[0], total - self._prev_sys[1]
                if dt > 0:
                    out['cpu_pct'] = round(db / dt * 100.0, 1)
            self._prev_sys = (busy, total)

        mem = self._read('/proc/meminfo')
        if mem:
            kv = {}
            for line in mem.split('\n'):
                if ':' in line:
                    k, v = line.split(':', 1)
                    kv[k] = v.strip().split()[0]
            total_kb = int(kv.get('MemTotal', 0))
            avail_kb = int(kv.get('MemAvailable', 0))
            if total_kb:
                out['mem_total_mb'] = total_kb // 1024
                out['mem_used_mb'] = (total_kb - avail_kb) // 1024
                out['mem_pct'] = round((total_kb - avail_kb) / total_kb * 100.0, 1)

        load = self._read('/proc/loadavg')
        if load:
            try:
                out['load1'] = float(load.split()[0])
            except ValueError:
                pass

        # 라즈베리파이 온도 — 스로틀링 여부를 보려면 필요하다
        t = self._read('/sys/class/thermal/thermal_zone0/temp')
        if t:
            try:
                out['temp_c'] = round(int(t.strip()) / 1000.0, 1)
            except ValueError:
                pass
        return out

    def run(self):
        while not self._stop.is_set():
            now = time.time()
            procs = {}

            # 자기 자신은 실제 스크립트 이름으로 보고한다 (tof.py / tof_2.py 구분)
            selfname = os.path.basename(sys.argv[0]) or 'tof.py'
            procs[selfname] = self._proc_stat(selfname, os.getpid(), now)

            for script in self._watch:
                cached = self._prev_proc.get(script)
                pid = cached[0] if cached else None
                # 캐시된 pid 가 사라졌을 때만 다시 스캔한다 (/proc 전체 순회 회피)
                if pid is None or not os.path.exists(f'/proc/{pid}'):
                    pid = self._find_pid(script)
                st = self._proc_stat(script, pid, now) if pid else None
                if st is None:
                    # 죽은 프로세스의 마지막 값이 남아 되살아난 것처럼 보이면 안 된다
                    self._prev_proc.pop(script, None)
                procs[script] = st

            snap = {
                'ts': round(now, 2),
                'cpu_count': self._cpu_count,
                'system': self._system(),
                'procs': procs,
            }
            with self._lock:
                self._snap = snap
            self._stop.wait(self._interval)


# ─────────────────────────────────────────────────────────────────────────────
# 내장 웹 화면
# ─────────────────────────────────────────────────────────────────────────────
#
# 제어 루프를 지키는 규칙 세 가지 — 시각화가 안전 로직을 방해하면 안 된다:
#   1) 루프는 최신 프레임 '참조'만 락 안에서 대입한다. JSON 직렬화(64셀×3)와
#      네트워크 쓰기는 전부 웹 스레드에서 한다.
#   2) 루프는 브라우저를 절대 기다리지 않는다. notify_all() 은 블로킹하지 않고,
#      느린 클라이언트는 자기 스레드에서만 막힌다.
#   3) 웹이 실패해도(포트 점유·예외) 경고만 남기고 제어는 계속한다.
#
# 프레임은 쌓지 않고 최신 한 벌만 유지한다. 실시간 뷰에서 밀린 과거 프레임은
# 가치가 없고, 큐를 두면 느린 클라이언트가 메모리를 밀어 올린다.

class WebView:
    """8x8 그리드를 브라우저로 내보내는 내장 HTTP 서버 (SSE)."""

    def __init__(self, cfg, sysmon=None):
        self.cfg = cfg
        self.sysmon = sysmon
        self._cv = threading.Condition()
        self._version = 0
        self._snap = None          # (grids, payload, baselines)
        self._at = 0.0
        self._hz = 0.0             # 제어 루프 실측 주파수
        self._prev = None
        self._httpd = None

    # ── 제어 루프에서 호출되는 유일한 지점 (락 + 대입만) ──────────────────
    def publish(self, grids, payload, baselines):
        now = time.time()
        with self._cv:
            if self._prev is not None:
                dseq = payload['seq'] - self._prev[0]
                dt = payload['stamp'] - self._prev[1]
                if dt > 0:
                    inst = dseq / dt
                    self._hz = inst if self._hz == 0 else (0.7 * self._hz + 0.3 * inst)
            self._prev = (payload['seq'], payload['stamp'])
            self._snap = (grids, payload, baselines)
            self._at = now
            self._version += 1
            self._cv.notify_all()

    # ── 아래는 전부 웹 스레드에서만 호출된다 ──────────────────────────────
    def bands(self):
        """거리 → 색 구간. 임계값은 전부 제어 파라미터에서 가져온다.

        색 배치 근거 (dataviz 검증 완료, 어두운 패널 #171a21 기준):
          · 명도가 거리에 따라 단조 증가 — 흑백으로 봐도, 색각 이상이어도 순서가 읽힌다
            (인접 구간 ΔL ≥ 0.06, 표면 대비 전 구간 ≥ 3.7:1)
          · 가장 중요한 경계인 **빨강↔주황(정지 vs 회피)** 이 가장 크게 벌어져 있다
            (CVD ΔE 13.3 / 정상시야 16.2 — 두 기준 모두 통과)
          · 주황↔노랑은 ΔE 7.5 로 경계 구간이라 색만으로는 부족하다. 그래서 셀마다
            mm 숫자를 찍고 범례에 거리 구간을 명시한다 (색 단독 의존 금지).
        """
        cfg = self.cfg
        far = cfg['web_band_far_mm']
        return [
            {'max': cfg['front_stop_distance'] * 1000,
             'bg': '#d33c42', 'fg': '#ffffff', 'name': '정지'},
            {'max': cfg['front_enter_threshold'] * 1000,
             'bg': '#e88434', 'fg': '#12151c', 'name': '회피'},
            {'max': cfg.get('cliff_abs_mm') or 800.0,
             'bg': '#deb433', 'fg': '#12151c', 'name': '주의'},
            {'max': far,
             'bg': '#a8ddf2', 'fg': '#12151c', 'name': '여유'},
            {'max': None,
             'bg': '#dff7df', 'fg': '#12151c', 'name': '안전'},
        ]

    def render(self):
        with self._cv:
            snap, at, hz = self._snap, self._at, self._hz
        now = time.time()
        sysinfo = self.sysmon.snapshot() if self.sysmon is not None else None
        if snap is None:
            return {'ready': False, 'stale': True, 'hz': 0.0, 'age': None,
                    'sys': sysinfo}
        grids, payload, baselines = snap
        return {
            'sys': sysinfo,
            'ready': True,
            'stale': (now - at) > self.cfg['web_stale_sec'],
            'age': round(now - at, 2),
            'hz': round(hz, 1),
            'seq': payload['seq'],
            'mode': payload['mode'],
            # mm 정수 변환도 여기서 — 루프에서 하면 20Hz × 192셀을 헛돈다
            'grids': {k: [None if v is None else int(v) for v in g]
                      for k, g in grids.items()},
            'dist': payload['tof'],
            'sensor': payload['sensor'],
            'cliff': payload['cliff'],
            'baseline': baselines,
            'floor_ok': payload.get('floor_ok'),
            # 행 라벨 색을 페이지에 하드코딩하면 설정을 바꿀 때 어긋난다
            'obstacle_roi': self.cfg['obstacle_roi_rows'],
            # 행 라벨 색은 공통값 기준. 채널별 예외는 아래 per_ch 로 따로 알린다.
            'cliff_roi': self.cfg['cliff_roi_rows'],
            'cliff_roi_per_ch': {t: (self.cfg.get(f'cliff_roi_rows_{t}')
                                     or self.cfg['cliff_roi_rows'])
                                 for t in CHANNELS},
            'cliff_abs_mm': self.cfg.get('cliff_abs_mm'),
            'bands': self.bands(),
            'self_name': os.path.basename(sys.argv[0]) or 'tof.py',
            'joy': payload['joy'],
            'out': {'linear_x': payload['linear_x'],
                    'angular_z': payload['angular_z']},
        }

    def wait(self, last_version, timeout):
        with self._cv:
            if self._version != last_version:
                return self._version
            self._cv.wait(timeout)
            return self._version

    def start(self):
        global WEB
        WEB = self
        cfg = self.cfg
        try:
            httpd = ThreadingHTTPServer((cfg['web_host'], cfg['web_port']), _Handler)
        except OSError as e:
            # 시각화가 안 뜬다고 제어를 멈추지 않는다
            log(f'⚠ 웹 화면 비활성 — 포트 {cfg["web_port"]} 열기 실패: {e}')
            log(f'  이미 실행 중인지 확인: ss -ltnp | grep {cfg["web_port"]}')
            return False
        httpd.daemon_threads = True
        self._httpd = httpd
        threading.Thread(target=httpd.serve_forever,
                         kwargs={'poll_interval': 0.3}, daemon=True).start()
        log(f'웹 화면 → http://{_local_ip()}:{cfg["web_port"]}')
        return True

    def stop(self):
        with self._cv:
            self._cv.notify_all()      # SSE 스레드들을 깨워 빠져나가게 한다
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
                self._httpd.server_close()
            except Exception:
                pass


WEB = None              # WebView 인스턴스 (핸들러가 참조)
WEB_STOP = threading.Event()


def _local_ip():
    """접속 주소 안내용. UDP connect 라 패킷은 실제로 나가지 않는다."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


class _Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'tof/1.0'

    def log_message(self, *_a):
        pass    # 접속 로그가 제어 콘솔을 덮어쓰지 않게 막는다

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/':
            self._send(PAGE.encode('utf-8'), 'text/html; charset=utf-8')
        elif path == '/state':
            self._send(json.dumps(WEB.render()).encode('utf-8'),
                       'application/json; charset=utf-8')
        elif path == '/sys':
            # 리소스만 따로 — 터미널에서 watch 로 보기 좋게
            body = WEB.sysmon.snapshot() if WEB.sysmon is not None else None
            self._send(json.dumps(body).encode('utf-8'),
                       'application/json; charset=utf-8')
        elif path == '/healthz':
            self._send(b'ok\n', 'text/plain; charset=utf-8')
        elif path == '/stream':
            self._stream()
        else:
            self.send_error(404)

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _stream(self):
        """SSE — 새 프레임이 오면 즉시 push. 폴링보다 지연이 낮다."""
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Accel-Buffering', 'no')
        self.send_header('Connection', 'close')   # SSE 는 Content-Length 가 없다
        self.end_headers()
        last = -1
        try:
            while not WEB_STOP.is_set():
                last = WEB.wait(last, timeout=1.0)
                # 변화가 없어도 1초마다 보낸다: 경과시간·멈춤 표시를 갱신해야 하고
                # 중간 프록시가 유휴 연결을 끊는 것도 막는다.
                self.wfile.write(f'data: {json.dumps(WEB.render())}\n\n'.encode('utf-8'))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass    # 브라우저 탭이 닫힘 — 정상 종료


# ── 페이지 (단일 파일, 외부 CDN·라이브러리 0개 → 오프라인 로봇에서 그대로 동작) ──

PAGE = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ToF 8x8 실시간 뷰</title>
<style>
  :root{
    --bg:#0f1115; --panel:#171a21; --line:#2a2f3a;
    --fg:#e6e9ef; --dim:#8b93a3;
    --ok:#3fb950; --warn:#d29922; --bad:#f85149; --accent:#58a6ff;
    --nodata:#3a3f4b;           /* 무효 셀 — 램프와 겹치지 않는 중립 회색 */
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);
       font:14px/1.45 system-ui,-apple-system,"Noto Sans KR",sans-serif}
  header{display:flex;flex-wrap:wrap;gap:10px;align-items:center;
         padding:10px 14px;border-bottom:1px solid var(--line);
         position:sticky;top:0;background:var(--bg);z-index:5}
  h1{font-size:15px;margin:0 12px 0 0;font-weight:600;letter-spacing:.2px}
  .badge{padding:3px 10px;border-radius:999px;font-size:12px;font-weight:600;
         border:1px solid var(--line);background:var(--panel);white-space:nowrap}
  .badge.ok{color:var(--ok);border-color:#1f6f33}
  .badge.warn{color:var(--warn);border-color:#7a5a12}
  .badge.bad{color:var(--bad);border-color:#8b2a26}
  .spacer{flex:1}
  .meta{color:var(--dim);font-size:12px;font-variant-numeric:tabular-nums}

  /* 상단 고정 리소스 스트립 — ToF 격자와 항상 같이 보이도록 헤더에 둔다.
     아래 상세 카드는 스크롤해야 보여서 실주행 중에는 못 본다. */
  .sysbar{display:inline-flex;gap:10px;align-items:center;flex-wrap:wrap;
          font-size:12px;font-variant-numeric:tabular-nums}
  .sysbar .it{display:inline-flex;gap:4px;align-items:baseline;
              padding:2px 8px;border-radius:6px;background:var(--panel);
              border:1px solid var(--line)}
  .sysbar .k{color:var(--dim);font-size:11px}
  .sysbar .v{font-weight:600}
  .sysbar .it.warn{border-color:#7a5a12} .sysbar .it.warn .v{color:var(--warn)}
  .sysbar .it.bad{border-color:#8b2a26}  .sysbar .it.bad .v{color:var(--bad)}
  .sysbar .dot{width:7px;height:7px;border-radius:50%;background:var(--ok);
               display:inline-block}
  .sysbar .dot.off{background:var(--bad)}

  #alert{display:none;margin:10px 14px 0;padding:10px 14px;border-radius:8px;
         background:#3d1416;border:1px solid var(--bad);color:#ffb4ae;
         font-weight:700}
  #alert.on{display:block}

  main{padding:14px;display:flex;flex-wrap:wrap;gap:14px;align-items:flex-start;
       transition:opacity .2s,filter .2s}
  /* 갱신이 끊기면 마지막 화면이 그대로 남는다. 그걸 실시간 값으로 오해하지
     않도록 흐리게+탈색해서 '멈춘 화면'임을 색이 아닌 형태로도 알린다. */
  main.stale{opacity:.4;filter:grayscale(.7)}
  .panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;
         padding:12px;flex:1 1 300px;min-width:280px}
  .panel.cliff{border-color:var(--bad);box-shadow:0 0 0 1px var(--bad) inset}
  .ptitle{display:flex;align-items:baseline;gap:8px;margin-bottom:8px}
  .ptitle b{font-size:14px}
  .dist{margin-left:auto;font-size:20px;font-weight:700;
        font-variant-numeric:tabular-nums}
  .dist.small{font-size:14px;font-weight:600;color:var(--dim)}

  .gridwrap{display:grid;grid-template-columns:auto 1fr;gap:4px}
  .rows{display:grid;grid-template-rows:repeat(8,1fr);gap:2px}
  .rlab{display:flex;align-items:center;justify-content:flex-end;
        font-size:10px;color:var(--dim);padding-right:4px;
        border-right:3px solid transparent}
  .rlab.obs{border-right-color:var(--accent)}
  .rlab.clf{border-right-color:#a371f7}
  .grid{display:grid;grid-template-columns:repeat(8,1fr);gap:2px}
  .cell{aspect-ratio:1/1;border-radius:3px;display:flex;
        align-items:center;justify-content:center;
        font-size:11px;font-variant-numeric:tabular-nums;
        background:var(--nodata);color:#7d8492}
  /* 범례 — 색만으로 뜻을 전달하지 않기 위해 이름과 거리 구간을 함께 적는다 */
  .legend{display:flex;gap:14px;flex-wrap:wrap;align-items:center;
          padding:10px 14px 0;font-size:12px}
  .lgi{display:inline-flex;align-items:center;gap:5px;white-space:nowrap}
  .lgi b{font-weight:600;color:var(--fg)}
  .lgi .rng{color:var(--dim);font-variant-numeric:tabular-nums}
  .key{display:inline-block;width:12px;height:12px;border-radius:3px}

  .bars{padding:0 14px 18px}
  .bar{display:flex;align-items:center;gap:10px;margin:6px 0}
  .bar .lab{width:130px;color:var(--dim);font-size:12px}
  .track{position:relative;flex:1;height:14px;background:#11141a;
         border:1px solid var(--line);border-radius:7px;overflow:hidden}
  .track::after{content:"";position:absolute;left:50%;top:0;bottom:0;
                width:1px;background:var(--line)}
  .fill{position:absolute;top:0;bottom:0;background:var(--accent);opacity:.85}
  .fill.out{background:var(--ok)}
  .val{width:70px;text-align:right;font-variant-numeric:tabular-nums;
       font-size:12px}

  /* 리소스 패널 — 제어 화면과 섞이지 않게 구분선 아래로 내린다 */
  .sys{margin:0 14px 18px;border-top:1px solid var(--line);padding-top:12px}
  .sys h2{font-size:12px;color:var(--dim);font-weight:600;margin:0 0 8px;
          text-transform:uppercase;letter-spacing:.6px}
  .cards{display:flex;flex-wrap:wrap;gap:10px}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:8px;
        padding:10px 12px;min-width:170px;flex:1 1 170px}
  .card.down{border-color:var(--bad)}
  .card .t{font-size:12px;color:var(--dim);display:flex;gap:6px;align-items:center}
  .card .big{font-size:19px;font-weight:700;font-variant-numeric:tabular-nums;
             margin-top:2px}
  .card .sub{font-size:11px;color:var(--dim);font-variant-numeric:tabular-nums}
  .meter{height:5px;border-radius:3px;background:#11141a;margin-top:6px;
         overflow:hidden}
  .meter i{display:block;height:100%;background:var(--accent);width:0}
  .meter i.warn{background:var(--warn)} .meter i.bad{background:var(--bad)}
  footer{padding:0 14px 24px;color:var(--dim);font-size:12px}
</style>
</head>
<body>
<header>
  <h1>ToF 8×8 실시간 뷰</h1>
  <span class="badge" id="mode">—</span>
  <span class="badge" id="link">연결 대기</span>
  <span class="spacer"></span>
  <span class="sysbar" id="sysbar"></span>
  <span class="meta" id="meta">—</span>
</header>

<div id="alert">⚠ 낭떠러지 감지 — 전진 차단 중 (<span id="alertwho"></span>)</div>

<div class="legend" id="legend"></div>

<main id="panels"></main>

<div class="bars">
  <div class="bar"><span class="lab">조이스틱 전진</span>
    <span class="track"><i class="fill" id="jl"></i></span>
    <span class="val" id="jlv">0.00</span></div>
  <div class="bar"><span class="lab">조이스틱 회전</span>
    <span class="track"><i class="fill" id="ja"></i></span>
    <span class="val" id="jav">0.00</span></div>
  <div class="bar"><span class="lab">출력 전진 (m/s)</span>
    <span class="track"><i class="fill out" id="ol"></i></span>
    <span class="val" id="olv">0.00</span></div>
  <div class="bar"><span class="lab">출력 회전 (rad/s)</span>
    <span class="track"><i class="fill out" id="oa"></i></span>
    <span class="val" id="oav">0.00</span></div>
</div>

<section class="sys">
  <h2>리소스 <span class="meta" id="sysmeta"></span></h2>
  <div class="cards" id="cards"></div>
</section>

<footer>
  행 라벨의 색 막대 = ROI (<span style="color:var(--accent)">파랑</span> 장애물,
  <span style="color:#a371f7">보라</span> 낭떠러지) — <span id="roitxt">…</span>.
  숫자는 셀 거리(mm). <b>open</b> = 센서는 정상인데 사거리 안에 물체 없음,
  <b>off</b> = 프레임이 안 옴(미연결/고장, 안전을 위해 막힘으로 취급).
  파란 막대(조이스틱)와 초록 막대(출력)의 차이가 회피 로직이 개입한 양이다.
</footer>

<script>
const CH = ["left","front","right"];
const NAME = {left:"왼쪽 45°", front:"정면", right:"오른쪽 45°"};
const MODE_CLS = {
  NORMAL:"ok", STOP:"warn", CLIFF_BLOCK:"bad",
  AVOID_LEFT:"warn", AVOID_RIGHT:"warn"
};
let roiDone = false;   // ROI 행 색칠은 최초 1회만

// 거리 → 색 구간. 경계값은 tof.py 의 실제 판정 임계값에서 온다 (WebView.bands).
// 빨강=정지 / 주황=회피 / 노랑=주의 / 하늘=여유 / 연초록=안전.
// 거리가 멀수록 명도가 올라가므로 흑백·색각이상에서도 순서가 읽힌다.
let BANDS = null;
function cellColor(mm){
  if (!BANDS) return {bg:"", fg:""};
  for (const b of BANDS) if (b.max === null || mm <= b.max) return b;
  return BANDS[BANDS.length-1];
}

// DOM 은 한 번만 만들고 이후에는 값만 갈아끼운다 (10Hz × 192셀)
const cells = {}, panels = {}, dists = {}, rowLabels = {};
const root = document.getElementById("panels");
for (const ch of CH){
  const p = document.createElement("section");
  p.className = "panel";
  const head = document.createElement("div");
  head.className = "ptitle";
  head.innerHTML = `<b>${NAME[ch]}</b><span class="meta" id="base-${ch}"></span>
                    <span class="dist" id="d-${ch}">—</span>`;
  const wrap = document.createElement("div");
  wrap.className = "gridwrap";
  const rows = document.createElement("div");
  rows.className = "rows";
  rowLabels[ch] = [];
  for (let r=0;r<8;r++){
    const l = document.createElement("div");
    l.className = "rlab";
    l.textContent = "R"+r;
    rows.appendChild(l);
    rowLabels[ch].push(l);
  }
  const g = document.createElement("div");
  g.className = "grid";
  cells[ch] = [];
  for (let i=0;i<64;i++){
    const c = document.createElement("div");
    c.className = "cell";
    c.textContent = "·";
    g.appendChild(c);
    cells[ch].push(c);
  }
  wrap.appendChild(rows); wrap.appendChild(g);
  p.appendChild(head); p.appendChild(wrap);

  root.appendChild(p);
  panels[ch] = p;
  dists[ch] = document.getElementById("d-"+ch);
}

function setBar(id, v, max){
  const el = document.getElementById(id);
  const t = Math.max(-1, Math.min(1, v/max));
  el.style.left  = (t >= 0 ? 50 : 50 + t*50) + "%";
  el.style.width = Math.abs(t)*50 + "%";
}

function render(s){
  const link = document.getElementById("link");
  if (!s.ready){
    link.textContent = "데이터 대기 중"; link.className = "badge warn";
  } else if (s.stale){
    link.textContent = "제어 루프 멈춤"; link.className = "badge bad";
  } else {
    link.textContent = "수신 중"; link.className = "badge ok";
  }

  // 끊긴 동안 남아 있는 마지막 프레임을 실시간으로 오인하지 않게 한다
  root.className = s.stale ? "stale" : "";

  // 색 구간·범례도 tof.py 설정에서 받아 만든다 (하드코딩하면 임계값과 어긋난다)
  if (s.bands && !BANDS){
    BANDS = s.bands;
    let lo = 0;
    document.getElementById("legend").innerHTML =
      BANDS.map(b => {
        const range = b.max === null ? `${lo}mm 이상`
                    : `${lo}~${Math.round(b.max)}mm`;
        const r = `<span class="lgi"><span class="key" style="background:${b.bg}"></span>
                   <b>${b.name}</b> <span class="rng">${range}</span></span>`;
        lo = Math.round(b.max ?? lo);
        return r;
      }).join("") +
      `<span class="lgi"><span class="key" style="background:var(--nodata)"></span>
       <b>무효</b> <span class="rng">반사 없음</span></span>`;
  }

  // 행 라벨 색은 tof.py 의 실제 ROI 설정을 따른다 (하드코딩하면 어긋난다)
  if (s.obstacle_roi && s.cliff_roi && !roiDone){
    const [o0,o1] = s.obstacle_roi, [c0,c1] = s.cliff_roi;
    for (const ch of CH) rowLabels[ch].forEach((el,r) => {
      el.className = "rlab" + (r>=o0&&r<=o1 ? " obs" : (r>=c0&&r<=c1 ? " clf" : ""));
    });
    document.getElementById("roitxt").textContent =
      `R${o0}–R${o1} 장애물 판정, R${c0}–R${c1} 낭떠러지 판정` +
      (s.cliff_abs_mm ? ` (${s.cliff_abs_mm}mm 초과 시 낭떠러지)` : '');
    roiDone = true;
  }

  document.getElementById("meta").textContent = s.ready
    ? `seq ${s.seq} · 제어 ${s.hz.toFixed(1)}Hz · ${s.age}s 전`
    : "tof.py 데이터 대기 중";

  const mode = s.mode || "—";
  const mb = document.getElementById("mode");
  mb.textContent = mode;
  mb.className = "badge " + (MODE_CLS[mode] || "");

  const cliff = s.cliff || {};
  const who = CH.filter(c => cliff[c]);
  document.getElementById("alert").className = who.length ? "on" : "";
  if (who.length) document.getElementById("alertwho").textContent =
    who.map(c => NAME[c]).join(", ");

  if (s.ready){
    for (const ch of CH){
      const grid = (s.grids && s.grids[ch]) || [];
      const cs = cells[ch];
      for (let i=0;i<64;i++){
        const v = grid[i], c = cs[i];
        if (v === null || v === undefined){
          if (c.dataset.v !== "x"){
            c.dataset.v = "x";
            c.style.background = ""; c.style.color = "";
            c.textContent = "·";
          }
        } else {
          const col = cellColor(v);
          c.dataset.v = "o";
          c.style.background = col.bg;
          c.style.color = col.fg;
          c.textContent = v;
        }
      }
      panels[ch].className = "panel" + (cliff[ch] ? " cliff" : "");

      // 측정값이 없으면 null 로 오므로 open/off 를 구분해 보여준다
      const d = s.dist ? s.dist[ch] : null;
      const sensor = s.sensor ? s.sensor[ch] : null;
      const el = dists[ch];
      if (d === null || d === undefined){
        el.className = "dist small";
        el.textContent = sensor === "offline" ? "off (미연결)" : "open (물체 없음)";
      } else {
        el.className = "dist";
        el.textContent = d.toFixed(2) + " m";
      }
      // 기준선이 바닥에서 학습된 게 아니면 낭떠러지 판정이 통째로 무의미하다.
      // 값 자체는 멀쩡해 보이므로 여기서 명시적으로 알린다.
      const b = s.baseline ? s.baseline[ch] : null;
      const fok = s.floor_ok ? s.floor_ok[ch] : null;
      const be = document.getElementById("base-"+ch);
      if (b === null || b === undefined){
        be.textContent = "기준선 학습 중"; be.style.color = "";
      } else if (fok === false){
        be.textContent = `기준선 ${b}mm ⚠ 바닥 아님`;
        be.style.color = "var(--bad)";
      } else {
        be.textContent = `기준선 ${b}mm`; be.style.color = "";
      }
    }
  }

  const j = s.joy || {linear_x:0, angular_z:0};
  const o = s.out || {linear_x:0, angular_z:0};
  setBar("jl", j.linear_x, 0.22); document.getElementById("jlv").textContent = j.linear_x.toFixed(2);
  setBar("ja", j.angular_z, 1.5); document.getElementById("jav").textContent = j.angular_z.toFixed(2);
  setBar("ol", o.linear_x, 0.22); document.getElementById("olv").textContent = o.linear_x.toFixed(2);
  setBar("oa", o.angular_z, 1.5); document.getElementById("oav").textContent = o.angular_z.toFixed(2);

  renderSys(s.sys, s.hz, s.self_name);
}

// ── 리소스 카드 ─────────────────────────────────────────────────────────
const cardsEl = document.getElementById("cards");
function card(cls, title, big, sub, meter){
  const m = meter === null || meter === undefined ? "" :
    `<div class="meter"><i class="${meter>85?"bad":meter>60?"warn":""}"
       style="width:${Math.min(100,meter)}%"></i></div>`;
  return `<div class="card ${cls||""}"><div class="t">${title}</div>
          <div class="big">${big}</div><div class="sub">${sub}</div>${m}</div>`;
}
// 헤더 스트립 — 격자를 보면서 동시에 읽히도록 한 줄로 압축한다
function renderSysBar(sys){
  const el = document.getElementById("sysbar");
  if (!sys){ el.innerHTML = ""; return; }
  const sv = sys.system, p = sys.procs || {};
  const lv = (v, warn, bad) => v >= bad ? "bad" : (v >= warn ? "warn" : "");
  const item = (k, v, cls) =>
    `<span class="it ${cls||""}"><span class="k">${k}</span><span class="v">${v}</span></span>`;
  // 프로세스 생사는 색 점 + 이름으로 (색만으로 뜻을 전하지 않는다)
  const dots = Object.entries(p).map(([n, d]) =>
    `<span class="it ${d ? "" : "bad"}"><span class="dot ${d ? "" : "off"}"></span>` +
    `<span class="k">${n.replace(/\.py$/, "")}</span>` +
    `<span class="v">${d ? d.cpu_pct.toFixed(0) + "%" : "없음"}</span></span>`).join("");
  el.innerHTML =
    dots +
    item("CPU", sv.cpu_pct.toFixed(0) + "%", lv(sv.cpu_pct, 70, 90)) +
    item("MEM", sv.mem_pct.toFixed(0) + "%", lv(sv.mem_pct, 80, 92)) +
    (sv.temp_c === null ? "" :
      item("온도", sv.temp_c.toFixed(0) + "°C", lv(sv.temp_c, 70, 80)));
}

function renderSys(sys, hz, selfName){
  renderSysBar(sys);
  if (!sys){ cardsEl.innerHTML = card("", "리소스 모니터", "꺼짐", "--no-sysmon", null); return; }
  const p = sys.procs || {}, sv = sys.system || {};
  const n = sys.cpu_count || 1;
  const proc = (name, label, extra) => {
    const d = p[name];
    if (!d) return card("down", label, "실행 안 됨",
                        name === "mqtt.py" ? "⚠ 라파1에 명령이 가지 않는다" : "—", null);
    // CPU% 는 top 기준(코어 1개 = 100%). 미터는 전체 코어 대비로 채운다.
    return card("", label + ` <span class="sub">pid ${d.pid}</span>`,
                d.cpu_pct.toFixed(1) + "%",
                `${(d.cpu_pct/n).toFixed(1)}% of ${n}코어 · RSS ${d.rss_mb}MB · ${d.threads}스레드${extra||""}`,
                d.cpu_pct / n);
  };
  cardsEl.innerHTML =
    // 제어 프로세스 이름은 서버가 알려준다 (tof.py / tof_2.py 구분)
    proc(selfName || "tof.py", (selfName || "tof.py") + " (제어)",
         ` · ${hz ? hz.toFixed(1) : "?"}Hz`) +
    proc("mqtt.py", "mqtt.py (ROS2 브리지)", "") +
    card("", "시스템 CPU", sv.cpu_pct.toFixed(1) + "%",
         `${n}코어 · load ${sv.load1.toFixed(2)}`, sv.cpu_pct) +
    card("", "메모리", sv.mem_pct.toFixed(1) + "%",
         `${sv.mem_used_mb} / ${sv.mem_total_mb} MB`, sv.mem_pct) +
    (sv.temp_c === null ? "" :
      card(sv.temp_c > 80 ? "down" : "", "CPU 온도", sv.temp_c.toFixed(1) + "°C",
           sv.temp_c > 80 ? "⚠ 스로틀링 구간" : "정상", sv.temp_c));
  document.getElementById("sysmeta").textContent = "1초 주기 · /proc";
}

// EventSource 는 끊기면 브라우저가 알아서 재접속한다 — 별도 재시도 코드 불필요
const es = new EventSource("/stream");
es.onmessage = e => { try { render(JSON.parse(e.data)); } catch(_){} };
es.onerror = () => {
  const link = document.getElementById("link");
  link.textContent = "tof.py 끊김 — 재접속 중"; link.className = "badge bad";
  document.getElementById("panels").className = "stale";
};
</script>
</body>
</html>
"""


# ─────────────────────────────────────────────────────────────────────────────
# MQTT
# ─────────────────────────────────────────────────────────────────────────────

def make_mqtt_client(client_id):
    """paho-mqtt v1/v2 양쪽 API 를 지원하는 클라이언트 생성."""
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        log('paho-mqtt 미설치 — pip install paho-mqtt (또는 apt install python3-paho-mqtt)')
        sys.exit(1)
    try:  # paho-mqtt >= 2.0
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except (AttributeError, TypeError):  # paho-mqtt 1.x
        return mqtt.Client(client_id=client_id)


# ─────────────────────────────────────────────────────────────────────────────
# 메인
# ─────────────────────────────────────────────────────────────────────────────

def run_geom_diag(cfg, seconds=12.0):
    """3D 기하 진단 — 기존(ROI 최소값)과 신규(3D) 거리를 나란히 보여준다.

    벽에서 정확히 아는 거리에 세워 두고 돌리면 장착각이 맞는지 바로 확인된다.
    """
    readers = {t: SerialReader(t, cfg[f'port_{t}'], cfg['baud']) for t in CHANNELS}
    for r in readers.values():
        r.start()
    geom = {t: SensorGeometry(cfg[f'mount_phi_{t}'], cfg[f'mount_theta_{t}'],
                              cfg[f'sensor_height_{t}']) for t in CHANNELS}
    print('\n=== 장착 파라미터 ===')
    for t in CHANNELS:
        print(f'  {t:6s} 좌우 {cfg[f"mount_phi_{t}"]:+6.1f}°  상하 '
              f'{cfg[f"mount_theta_{t}"]:+5.1f}°  높이 {cfg[f"sensor_height_{t}"]:5.1f}mm')
    print(f'  바닥 컷오프 {cfg["floor_margin_mm"]:.0f}mm 이하 제외, '
          f'정면 밴드 ±{cfg["front_band_half_deg"]:.1f}°, '
          f'옆거리 z {cfg["lateral_z_range"][0]:.0f}~{cfg["lateral_z_range"][1]:.0f}mm '
          f'(정면 ±{cfg["lateral_min_az_deg"]:.0f}° 제외)')
    print(f'\n{seconds:.0f}초간 비교 (0.5초 간격)\n')
    print(f'  {"":8} | {"기존 ROI 최소값 (m)":^26} | {"신규 3D (m)":^26}')
    print(f'  {"t":>8} | {"front":>8} {"left":>8} {"right":>8} | '
          f'{"front":>8} {"left":>8} {"right":>8}  바닥제외')
    t0 = time.time()
    while time.time() - t0 < seconds:
        now = time.time()
        old, pts_of, dropped = {}, {}, 0
        for tag, rd in readers.items():
            grid, stamp, conn = rd.snapshot()
            fresh = conn and stamp > 0 and (now - stamp) <= cfg['stale_timeout']
            if not fresh:
                old[tag] = None
                pts_of[tag] = []
                continue
            vals, _ = roi_values(grid, cfg['obstacle_roi_rows'],
                                 cfg['obstacle_roi_cols'])
            old[tag] = (min(vals) / 1000.0) if vals else None
            pts = geom[tag].points(grid, cfg['floor_margin_mm'],
                                   cfg['geom_skip_rows'])
            pts_of[tag] = pts
            n_valid = sum(1 for v in grid if v is not None)
            dropped += max(0, n_valid - len(pts))
        allp = pts_of['front'] + pts_of['left'] + pts_of['right']
        nf = front_distance_3d(allp, cfg['front_band_half_deg'],
                               cfg['front_n_closest'])
        if cfg['side_metric'] == 'lateral':
            nl, nr = lateral_distances_3d(allp, cfg['lateral_z_range'],
                                          cfg['lateral_min_az_deg'])
        else:
            nl, nr = sector_distances_3d(allp, cfg['sector_dead_deg'])
        f = lambda v: '   --  ' if v is None else f'{v:7.3f}'
        print(f'  {now-t0:8.1f} | {f(old["front"])} {f(old["left"])} {f(old["right"])} | '
              f'{f(nf)} {f(nl)} {f(nr)}  {dropped:3d}셀')
        time.sleep(0.5)
    for r in readers.values():
        r.stop()
    print('\n해석:')
    print('  · 신규 front 가 기존보다 **크게** 나오면 바닥 셀이 빠진 것 — 의도한 개선.')
    print('  · 신규 left/right 는 사선이 아니라 실제 옆거리다. 벽에서 30cm 옆에')
    print('    세웠을 때 0.300 근처가 나와야 좌우 장착각이 맞다.')
    print('  · 셋 다 -- 면 그 방향에 (바닥 아닌) 물체가 사거리 안에 없다는 뜻.')
    return 0


def run_wall_calib(cfg, seconds=8.0):
    """평평한 벽을 정면으로 마주본 상태에서 좌우 장착각을 역산한다.

    원리: 벽이 평면이므로 **모든 점의 z(전방거리)가 같아야 한다.**
    좌/우 센서는 비스듬히 보므로 사선 거리는 열마다 다르지만, 장착각이 맞으면
    3D 변환 후 z 는 전부 벽까지 거리로 모인다. 각도가 틀리면 z 가 부채꼴로
    퍼진다. 그래서 **z 의 흩어짐이 가장 작아지는 각도**를 찾으면 그게 정답이다.

    자로 옆거리를 재는 것보다 훨씬 쉽고 정확하다 — 벽을 마주보게만 세우면 된다.
    """
    readers = {t: SerialReader(t, cfg[f'port_{t}'], cfg['baud']) for t in CHANNELS}
    for r in readers.values():
        r.start()
    print('\n=== 좌우 장착각 역산 ===')
    print('  로봇을 **평평한 벽을 정면으로 마주보게** 세우세요.')
    print('  벽까지 0.4~1.0m 가 적당합니다. 옆 물건은 치우세요.\n')
    print(f'  {seconds:.0f}초간 수집 중...')
    time.sleep(2.0)

    samples = {t: [] for t in CHANNELS}     # (d_mm, phi_local_rad, theta_rad)
    t0 = time.time()
    while time.time() - t0 < seconds:
        now = time.time()
        for tag, rd in readers.items():
            grid, stamp, conn = rd.snapshot()
            if not (conn and stamp > 0 and (now - stamp) <= cfg['stale_timeout']):
                continue
            g = geom_local_angles(cfg, tag)
            for idx, mm in enumerate(grid):
                row = idx >> 3
                if mm is None or row in cfg['geom_skip_rows']:
                    continue
                phi, theta = g[idx]
                # 바닥으로 보이는 점은 제외 (현재 추정각 기준, 대략만 걸러도 충분)
                if cfg[f'sensor_height_{tag}'] + mm * math.sin(theta) <= cfg['floor_margin_mm']:
                    continue
                samples[tag].append((mm, phi, theta))
        time.sleep(0.05)
    for r in readers.values():
        r.stop()

    print(f'\n  {"센서":6} {"점 수":>7} {"현재각":>8} {"역산각":>8} {"차이":>7}  '
          f'{"z 흩어짐(현재→역산)":>22}')
    result = {}
    railed = {}
    resid_of = {}
    for tag in CHANNELS:
        sm = samples[tag]
        if len(sm) < 50:
            print(f'  {tag:6} {len(sm):7d}   점이 너무 적음 — 벽이 안 보입니다')
            continue
        cur = cfg[f'mount_phi_{tag}']

        def spread(mu_deg):
            mu = math.radians(mu_deg)
            zs = [d * math.cos(phi + mu) * math.cos(th) for d, phi, th in sm]
            m = sum(zs) / len(zs)
            return math.sqrt(sum((z - m) ** 2 for z in zs) / len(zs))

        # 넓게 훑는다. 좁게 잡으면 최적값이 범위 밖일 때 경계에 붙어버리고,
        # 그 값을 정답으로 착각한다 (실제로 겪었다 — ±25° 범위에서 둘 다 경계인
        # ±20.0° 가 나왔다). 경계에 붙으면 아래에서 걸러낸다.
        lo, hi, step = -85.0, 85.0, 0.25
        best, best_s = cur, spread(cur)
        x = lo
        while x <= hi:
            s = spread(x)
            if s < best_s:
                best, best_s = x, s
            x += step
        railed[tag] = abs(best - lo) < step * 1.5 or abs(best - hi) < step * 1.5
        result[tag] = best
        resid_of[tag] = best_s
        print(f'  {tag:6} {len(sm):7d} {cur:+8.1f}° {best:+8.1f}° {best-cur:+7.1f}°  '
              f'{spread(cur):8.1f}mm → {best_s:6.1f}mm')

    # ── 장면 유효성 검사 ─────────────────────────────────────────────────
    # front 의 좌우 장착각은 **정의상 0°** 다 — 로봇의 전방축을 그것으로 정하니까.
    # 역산에서 0 이 안 나오면 각도가 틀린 게 아니라 **장면이 평평한 정면 벽이
    # 아니라는 뜻**이다. 이 검사 없이 결과를 믿으면 멀쩡한 설정을 망친다.
    if 'front' not in result:
        print('\n  ⚠ front 표본 부족 — 판정 불가.')
        return 1
    front_off = result['front']
    print('\n판정:')
    bad = [t for t, r in railed.items() if r]
    if bad:
        print(f'  ❌ 탐색 경계에 붙은 채널: {", ".join(bad)} — 결과를 믿을 수 없습니다.')
        print('     이 센서들이 벽을 거의 못 보고 있을 가능성이 큽니다.')
        return 1
    if abs(front_off) > 5.0:
        print(f'  ❌ 장면이 유효하지 않습니다 — front 가 {front_off:+.1f}° 로 나왔습니다.')
        print('     front 의 좌우 장착각은 정의상 0° 이므로, 0 이 아니면 각도 문제가')
        print('     아니라 **평평한 벽을 정면으로 마주보고 있지 않다**는 뜻입니다.')
        print('     로봇을 벽에 정면으로 맞추고(0.4~1.0m) 옆 물건을 치운 뒤 다시 하세요.')
        print('     ※ 지금 결과로 CONFIG 를 바꾸지 마세요.')
        return 1

    ok = True
    for tag, best in result.items():
        if tag == 'front':
            continue
        diff = abs(best - cfg[f'mount_phi_{tag}'])
        if diff > 3.0:
            ok = False
            print(f'  ⚠ {tag}: 현재 설정이 {diff:.1f}° 틀렸습니다. '
                  f"CONFIG 의 mount_phi_{tag} 를 {best:+.1f} 로 바꾸세요."
                  f'  (잔차 {resid_of[tag]:.1f}mm)')
    if ok:
        print('  ✅ 좌우 장착각이 맞습니다 (전부 3° 이내).')
    print(f'  (장면 검증 통과: front {front_off:+.1f}° ≈ 0°)')
    print('\n  · 역산각의 z 흩어짐이 현재각보다 크게 작아졌다면 그 각도가 맞습니다.')
    print('  · 흩어짐이 역산 후에도 크면(>15mm) 벽이 평평하지 않은 겁니다.')
    return 0


def geom_local_angles(cfg, tag):
    """셀별 (phi_local, theta) — 장착 좌우각을 뺀 센서 로컬 방위각."""
    hp = math.radians(45.0 / 2.0)
    ht = math.radians(45.0 / 2.0)
    mt = math.radians(cfg[f'mount_theta_{tag}'])
    out = []
    for i in range(8):
        for j in range(8):
            out.append((hp * (j * 2 - 7) / 7.0, ht * (-i * 2 + 7) / 7.0 + mt))
    return out


def main():
    ap = argparse.ArgumentParser(description='ToF Assisted Teleop 제어 연산 (라파2)')
    ap.add_argument('--dry-run', action='store_true',
                    help='MQTT 발행 없이 콘솔에만 출력')
    ap.add_argument('--list-joy', action='store_true',
                    help='입력 장치 목록 출력 후 종료')
    ap.add_argument('--geom', action='store_true',
                    help='3D 기하 진단 — 기존(ROI 최소값) vs 신규(3D) 거리 비교 후 종료')
    ap.add_argument('--calib-wall', action='store_true',
                    help='평평한 벽을 마주본 상태에서 좌우 장착각 역산 후 종료')
    ap.add_argument('--mqtt-host', default=CONFIG['mqtt_host'])
    ap.add_argument('--mqtt-port', type=int, default=CONFIG['mqtt_port'])
    ap.add_argument('--topic', default=CONFIG['mqtt_topic'])
    ap.add_argument('--no-web', action='store_true',
                    help='내장 웹 화면 끄기')
    ap.add_argument('--no-sysmon', action='store_true',
                    help='CPU·메모리 모니터 끄기')
    ap.add_argument('--web-host', default=CONFIG['web_host'],
                    help='웹 바인드 주소 (기본 0.0.0.0 = 외부 접속 허용)')
    ap.add_argument('--web-port', type=int, default=CONFIG['web_port'])
    ap.add_argument('--grid', action='store_true',
                    help='8x8 원본을 MQTT 로도 발행 (원격 구독용. 웹 화면에는 불필요)')
    ap.add_argument('--no-cliff', action='store_true',
                    help='낭떠러지 감지 끄기 (실내 평지 테스트 전용)')
    ap.add_argument('--quiet', action='store_true', help='상태 출력 줄이기')
    args = ap.parse_args()

    if args.list_joy:
        list_input_devices()
        return 0

    if args.geom:
        return run_geom_diag(dict(CONFIG))

    if args.calib_wall:
        return run_wall_calib(dict(CONFIG))

    cfg = dict(CONFIG)
    cfg['mqtt_host'] = args.mqtt_host
    cfg['mqtt_port'] = args.mqtt_port
    cfg['mqtt_topic'] = args.topic
    cfg['web_host'] = args.web_host
    cfg['web_port'] = args.web_port
    if args.no_cliff:
        cfg['cliff_enabled'] = False
        log('⚠ 낭떠러지 감지가 꺼져 있습니다 (--no-cliff)')

    # ── 스레드 기동 ───────────────────────────────────────────────────────
    readers = {
        tag: SerialReader(tag, cfg[f'port_{tag}'], cfg['baud'])
        for tag in CHANNELS
    }
    for r in readers.values():
        r.start()

    joy = JoystickReader(cfg)
    joy.start()

    cliffs = {tag: CliffDetector(cfg, tag) for tag in CHANNELS}
    ctrl = Controller(cfg)

    # 리소스 모니터 (1Hz 별도 스레드 — 제어 주기와 무관)
    sysmon = None
    if not args.no_sysmon:
        sysmon = SysMonitor(interval=cfg['sysmon_interval'])
        sysmon.start()

    # 웹 화면 (실패해도 제어는 계속한다)
    web = None
    if not args.no_web:
        w = WebView(cfg, sysmon)
        if w.start():
            web = w

    # ── MQTT 연결 ─────────────────────────────────────────────────────────
    client = None
    if not args.dry_run:
        client = make_mqtt_client(f'tof-{os.getpid()}')
        try:
            client.connect(cfg['mqtt_host'], cfg['mqtt_port'], keepalive=10)
        except Exception as e:
            log(f'MQTT 연결 실패 {cfg["mqtt_host"]}:{cfg["mqtt_port"]} — {e}')
            log('mosquitto 가 실행 중인지 확인하세요: systemctl status mosquitto')
            return 1
        client.loop_start()
        log(f'MQTT 연결됨 → {cfg["mqtt_host"]}:{cfg["mqtt_port"]} '
            f'topic={cfg["mqtt_topic"]}')

    running = {'v': True}

    def on_signal(_sig, _frm):
        running['v'] = False

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    period = 1.0 / cfg['rate']
    grid_period = 1.0 / cfg['grid_rate']
    next_grid = 0.0
    seq = 0
    last_print = 0.0
    last_mode = None

    obs_rows = cfg['obstacle_roi_rows']
    obs_cols = cfg['obstacle_roi_cols']
    cliff_cols = cfg['cliff_roi_cols']
    # 센서 장착 기하 (기동 시 1회 — 매 프레임 삼각함수를 돌지 않는다)
    geom = {t: SensorGeometry(cfg[f'mount_phi_{t}'], cfg[f'mount_theta_{t}'],
                              cfg[f'sensor_height_{t}']) for t in CHANNELS}
    for t in CHANNELS:
        log(f'[{t}] 장착 좌우 {cfg[f"mount_phi_{t}"]:+.1f}° 상하 '
            f'{cfg[f"mount_theta_{t}"]:+.1f}° 높이 {cfg[f"sensor_height_{t}"]:.0f}mm')

    # 채널별 낭떠러지 ROI (지정 없으면 공통값)
    cliff_rows_of = {t: cfg.get(f'cliff_roi_rows_{t}') or cfg['cliff_roi_rows']
                     for t in CHANNELS}
    for t in CHANNELS:
        if cliff_rows_of[t] != cfg['cliff_roi_rows']:
            log(f'[{t}] 낭떠러지 ROI 행 {cliff_rows_of[t]} (공통값과 다름)')

    log('메인 루프 시작 (Ctrl+C 로 종료)')

    while running['v']:
        loop_start = time.time()
        now = loop_start

        grids = {}
        dist = {}       # 제어용 거리 (무효는 안전 대체값으로 치환됨)
        valid = {}      # 실제로 측정값이 있었는가 (표시·보고용)
        status = {}     # ok | empty(물체 없음) | offline(센서 미연결)
        cliff_flags = {}

        # ── 3D 점군 (tof_2): 세 센서를 로봇 좌표 하나로 합친다 ────────────
        fresh_of = {}
        pts_of = {}
        for tag, reader in readers.items():
            grid, stamp, connected = reader.snapshot()
            fresh = connected and stamp > 0 and (now - stamp) <= cfg['stale_timeout']
            fresh_of[tag] = fresh
            grids[tag] = grid if fresh else [None] * 64
            pts_of[tag] = (geom[tag].points(grid, cfg['floor_margin_mm'],
                                            cfg['geom_skip_rows'])
                           if fresh else [])
        all_pts = pts_of['front'] + pts_of['left'] + pts_of['right']

        # 정면은 각도 밴드(세 센서 합산), 옆은 실제 |x| 최소값
        d_front = front_distance_3d(all_pts, cfg['front_band_half_deg'],
                                    cfg['front_n_closest'])
        if cfg['side_metric'] == 'lateral':
            d_left, d_right = lateral_distances_3d(
                all_pts, cfg['lateral_z_range'], cfg['lateral_min_az_deg'])
        else:
            d_left, d_right = sector_distances_3d(all_pts, cfg['sector_dead_deg'])
        d_of = {'front': d_front, 'left': d_left, 'right': d_right}

        for tag, reader in readers.items():
            fresh = fresh_of[tag]
            grid = grids[tag]

            # ── 장애물: 3D 기반 거리 (tof.py 의 ROI 최소값을 대체) ─────────
            d = d_of[tag] if fresh else None
            valid[tag] = d is not None
            if d is None:
                # 두 가지 무효를 구분한다 (같이 취급하면 안 된다):
                #   1) 프레임 자체가 안 옴(fresh=False) = 센서 미연결/고장 → 막힘
                #   2) 프레임은 오는데 ROI 셀이 전부 무효 = 사거리 내 물체 없음 → 열림
                # 2를 막힘으로 처리하면, 빈 공간을 향한 센서가 영구히 '막힘'으로
                # 잡혀 그 방향으로는 절대 회피하지 못하게 된다.
                if not fresh:
                    d = 0.0 if cfg['treat_stale_as_blocked'] else float('inf')
                else:
                    d = float('inf') if cfg['empty_roi_as_open'] else 0.0
            dist[tag] = d
            status[tag] = 'ok' if valid[tag] else ('empty' if fresh else 'offline')

            # ── 낭떠러지: 하단 3행 ────────────────────────────────────────
            if cfg['cliff_enabled'] and fresh:
                rows_t = cliff_rows_of[tag]
                vals, total = roi_values(grid, rows_t, cliff_cols)
                # 행중앙값은 워밍업 검증에만 쓰이므로 학습 전에만 계산한다.
                # ROI 가 한 행뿐이면 기울기를 못 만드니 검사용으로만 한 행 위까지 넓힌다.
                if not cliffs[tag].ready:
                    chk = rows_t if rows_t[0] < rows_t[1] else [max(0, rows_t[0]-1), rows_t[1]]
                    meds = row_medians(grid, chk, cliff_cols)
                else:
                    meds = None
                cliff_flags[tag] = cliffs[tag].update(vals, total, meds)
            else:
                cliff_flags[tag] = False

        cliff_any = any(cliff_flags.values())
        manual_lin, manual_ang, engaged = joy.read()

        lin, ang, mode = ctrl.step(dist, cliff_any, manual_lin, manual_ang, engaged)

        # 최종 안전 클램프
        lin = clamp(lin, cfg['v_min'], cfg['v_max'])
        ang = clamp(ang, -cfg['w_max'], cfg['w_max'])

        seq += 1
        payload = {
            'seq': seq,
            'stamp': round(now, 3),
            'linear_x': round(lin, 4),
            'angular_z': round(ang, 4),
            'mode': mode,
            'cliff_detected': cliff_any,
            'obstacle_detected': mode in (MODE_AVOID_LEFT, MODE_AVOID_RIGHT),
            # 측정값이 없으면 null — 안전 대체값(0.0)을 실제 측정처럼 보고하지 않는다
            'tof': {k: (round(v, 3) if valid[k] else None) for k, v in dist.items()},
            # ok=측정됨 / empty=센서 정상이나 사거리 내 물체 없음 / offline=프레임 없음
            'sensor': status,
            'cliff': cliff_flags,
            'joy': {'engaged': engaged,
                    'linear_x': round(manual_lin, 4),
                    'angular_z': round(manual_ang, 4)},
        }

        if client is not None:
            client.publish(cfg['mqtt_topic'], json.dumps(payload), qos=cfg['mqtt_qos'])

        # ── 시각화 (10Hz — 제어 20Hz 보다 낮게) ───────────────────────────
        # 여기서 하는 일은 참조 대입뿐이다. 8x8×3 직렬화는 웹 스레드가 한다.
        want_mqtt_grid = client is not None and args.grid
        if now >= next_grid and (web is not None or want_mqtt_grid):
            next_grid = now + grid_period
            baselines = {k: (None if cliffs[k].baseline is None
                             else int(cliffs[k].baseline)) for k in CHANNELS}
            if web is not None:
                payload['floor_ok'] = {k: cliffs[k].floor_ok for k in CHANNELS}
                web.publish(grids, payload, baselines)
            if want_mqtt_grid:
                client.publish(cfg['mqtt_grid_topic'], json.dumps({
                    'seq': seq,
                    'stamp': round(now, 3),
                    'grids': {k: [None if v is None else int(v) for v in g]
                              for k, g in grids.items()},
                    'dist': payload['tof'],
                    'cliff': cliff_flags,
                    'baseline': baselines,
                    'mode': mode,
                    'obstacle_roi': [obs_rows, obs_cols],
                    'cliff_roi': [cliff_rows_of, cliff_cols],
                }), qos=0)

        # ── 콘솔 출력 ─────────────────────────────────────────────────────
        if mode != last_mode:
            log(f'mode: {last_mode} -> {mode}')
            last_mode = mode
        if not args.quiet and (now - last_print) >= 0.5:
            last_print = now
            def fmt(tag):
                # 세 상태를 구분: 측정값 / 물체 없음(open) / 센서 미연결(off)
                if status[tag] == 'offline':
                    return ' off '
                if status[tag] == 'empty':
                    return ' open'
                return f'{dist[tag]:5.2f}'
            cl = ''.join('C' if cliff_flags[t] else '.' for t in CHANNELS)
            print(f'\rF{fmt("front")} L{fmt("left")} R{fmt("right")} '
                  f'| cliff[{cl}] | joy {manual_lin:+.2f}/{manual_ang:+.2f} '
                  f'{"ON " if engaged else "OFF"} '
                  f'| out {lin:+.2f}/{ang:+.2f} | {mode:<12}',
                  end='', flush=True)

        elapsed = time.time() - loop_start
        if elapsed < period:
            time.sleep(period - elapsed)

    # ── 종료: 정지 명령을 여러 번 보내고 끝낸다 ───────────────────────────
    print()
    log('종료 중 — 정지 명령 발행')
    if client is not None:
        for _ in range(5):
            seq += 1
            client.publish(cfg['mqtt_topic'], json.dumps({
                'seq': seq, 'stamp': round(time.time(), 3),
                'linear_x': 0.0, 'angular_z': 0.0, 'mode': MODE_STOP,
                'cliff_detected': False, 'obstacle_detected': False,
            }), qos=cfg['mqtt_qos'])
            time.sleep(0.05)
        client.loop_stop()
        client.disconnect()

    WEB_STOP.set()
    if web is not None:
        web.stop()
    if sysmon is not None:
        sysmon.stop()
    for r in readers.values():
        r.stop()
    joy.stop()
    log('종료 완료')
    return 0


if __name__ == '__main__':
    sys.exit(main())
