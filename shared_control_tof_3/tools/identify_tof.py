#!/usr/bin/env python3
"""ToF 센서 식별 도구 (ROS 불필요, 단독 실행).

3개의 Raspberry Pi Pico 2 (VL53L8CX ToF) 시리얼 포트를 동시에 열고,
각 포트의 8x8 그리드에서 '중앙 최소거리'를 실시간 표로 출력한다.

사용법:
    python3 identify_tof.py

화면을 보면서 손을 각 센서 앞 약 10~20cm 에 가까이 대면
해당 포트의 min_mm 값이 뚝 떨어진다. 그렇게 어느 시리얼번호가
front / left / right 인지 확정한 뒤, 아래처럼 알려주면 udev 규칙을 만든다.

    front  = 3979...
    left   = 753A...
    right  = A45D...

종료: Ctrl+C

참고:
  - VID 0x2E8A (Raspberry Pi) 인 tty 포트를 자동으로 모두 찾는다.
  - 펌웨어가 다르면(예: product 000b) 프레임 파싱이 안 될 수 있는데,
    그 경우 raw 한 줄을 같이 보여줘서 포맷을 진단할 수 있게 한다.
"""

import re
import sys
import threading
import time

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    print("pyserial 이 필요합니다:  pip3 install pyserial  (또는 sudo apt install python3-serial)")
    sys.exit(1)

BAUD = 115200
PI_VID = 0x2E8A            # Raspberry Pi (Pico/Pico2)
ROW_RE = re.compile(r'\s*R(\d)\s+(.*)')

# 중앙 ROI (8x8 중 rows 2..5, cols 2..5) — 정면 판단용
ROI_ROWS = range(2, 6)
ROI_COLS = range(2, 6)


class PortReader(threading.Thread):
    def __init__(self, device, serial_no):
        super().__init__(daemon=True)
        self.device = device
        self.serial_no = serial_no
        self.min_mm = None          # 중앙 ROI 최소거리 (mm)
        self.valid_cells = 0        # 유효 셀 개수
        self.last_raw = ''          # 최근 raw 라인 (진단용)
        self.frames = 0
        self.error = None
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            ser = serial.Serial(self.device, BAUD, timeout=1)
        except Exception as e:  # noqa: BLE001
            self.error = str(e)
            return

        buffer = []
        in_frame = False
        while not self._stop.is_set():
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception as e:  # noqa: BLE001
                self.error = str(e)
                break
            if not line:
                continue
            self.last_raw = line

            if 'Frame #' in line:
                if in_frame and buffer:
                    self._parse(buffer)
                in_frame = True
                buffer = [line]
            elif in_frame:
                buffer.append(line)
                if line.startswith('---'):
                    self._parse(buffer)
                    in_frame = False
                    buffer = []
        try:
            ser.close()
        except Exception:
            pass

    def _parse(self, lines):
        """첫 8개 R 라인 = 거리(mm). 중앙 ROI 최소거리/유효셀 갱신."""
        rows = []
        for line in lines:
            m = ROW_RE.match(line)
            if not m:
                continue
            vals = m.group(2).split()
            rows.append(vals)
            if len(rows) >= 8:        # 거리 행 8개면 충분
                break
        if len(rows) < 8:
            return

        best = None
        count = 0
        for r in ROI_ROWS:
            for c in ROI_COLS:
                if r >= len(rows) or c >= len(rows[r]):
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
                count += 1
                if best is None or mm < best:
                    best = mm
        self.min_mm = best
        self.valid_cells = count
        self.frames += 1


def find_pico_ports():
    ports = []
    for p in list_ports.comports():
        if p.vid == PI_VID:
            ports.append((p.device, p.serial_number or '?', p.pid))
    # 시리얼번호 기준 정렬 (일관된 표시)
    ports.sort(key=lambda x: x[1])
    return ports


def main():
    ports = find_pico_ports()
    if not ports:
        print("Raspberry Pi Pico 포트를 찾지 못했습니다. (VID 0x2E8A)")
        print("연결을 확인하세요:  ls -l /dev/serial/by-id/")
        sys.exit(1)

    print(f"발견된 Pico 포트 {len(ports)}개:")
    for dev, sn, pid in ports:
        print(f"  {dev}  serial={sn}  pid=0x{pid:04x}")
    print()
    print("이제 각 센서 앞 10~20cm 에 손을 대보세요. min_mm 이 뚝 떨어지는 포트가 그 센서입니다.")
    print("Ctrl+C 로 종료.\n")
    time.sleep(1.0)

    readers = [PortReader(dev, sn) for dev, sn, _ in ports]
    for r in readers:
        r.start()

    try:
        while True:
            time.sleep(0.5)
            cols = []
            for r in readers:
                short = r.serial_no[:8]
                if r.error:
                    cell = f"{short}: ERR({r.error[:18]})"
                elif r.frames == 0:
                    raw = r.last_raw[:22] if r.last_raw else '대기중'
                    cell = f"{short}: no-frame [{raw}]"
                else:
                    mm = r.min_mm
                    dist = f"{mm/1000:5.2f}m" if mm is not None else " ----  "
                    cell = f"{short}: {dist} (cells={r.valid_cells:2d}, f={r.frames})"
                cols.append(cell)
            line = "  |  ".join(cols)
            sys.stdout.write("\r" + line[:200].ljust(200))
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n\n종료합니다.")
        for r in readers:
            r.stop()
        # 최종 요약
        print("\n최종 시리얼번호 매핑을 알려주세요 (가장 가까이 손을 댔을 때 값이 떨어진 포트):")
        for r in readers:
            print(f"  serial={r.serial_no}  device={r.device}")


if __name__ == '__main__':
    main()
