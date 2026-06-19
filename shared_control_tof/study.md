# shared_control 프로젝트 공부 노트

이 파일은 프로젝트를 이해하는 데 필요한 개념들을  
**처음 보는 사람도 이해할 수 있도록** 설명합니다.

---

## 목차

1. [ROS2 가 뭔가요?](#1-ros2-가-뭔가요)
2. [노드·토픽·메시지 — ROS2 의 핵심 3요소](#2-노드토픽메시지--ros2-의-핵심-3요소)
3. [Publisher 와 Subscriber](#3-publisher-와-subscriber)
4. [타이머 (Timer)](#4-타이머-timer)
5. [파라미터 (Parameter)](#5-파라미터-parameter)
6. [센서 이해 — LiDAR](#6-센서-이해--lidar)
7. [센서 이해 — ToF (Time of Flight)](#7-센서-이해--tof-time-of-flight)
8. [로봇 좌표계와 각도 방향](#8-로봇-좌표계와-각도-방향)
9. [각도 정규화 — atan2 트릭](#9-각도-정규화--atan2-트릭)
10. [상태 머신 (State Machine)](#10-상태-머신-state-machine)
11. [히스테리시스 (Hysteresis)](#11-히스테리시스-hysteresis)
12. [2단계 감지와 LiDAR 맹점](#12-2단계-감지와-lidar-맹점)
13. [공유 제어 (Shared Control) 개념](#13-공유-제어-shared-control-개념)
14. [갭 탐색 알고리즘 (Gap Seeking)](#14-갭-탐색-알고리즘-gap-seeking)
15. [하이브리드 점수 (Hybrid Scoring)](#15-하이브리드-점수-hybrid-scoring)
16. [비례 조향 (Proportional Steering)](#16-비례-조향-proportional-steering)
17. [이 프로젝트의 5개 노드 흐름](#17-이-프로젝트의-5개-노드-흐름)
18. [코드 패턴 읽는 법 (rclpy)](#18-코드-패턴-읽는-법-rclpy)
19. [파라미터 튜닝 가이드](#19-파라미터-튜닝-가이드)
20. [자주 보는 ROS2 명령어 모음](#20-자주-보는-ros2-명령어-모음)

---

## 1. ROS2 가 뭔가요?

**ROS2 (Robot Operating System 2)** 는 로봇 소프트웨어를 만들기 위한  
**프레임워크(틀)** 입니다. 운영체제가 아니라, Ubuntu 위에서 동작하는  
로봇 전용 미들웨어입니다.

이 프로젝트는 ROS2 **Jazzy Jalisco** 버전을 사용합니다 (Ubuntu 24.04 와 호환).

---

## 2. 노드·토픽·메시지 — ROS2 의 핵심 3요소

### 노드 (Node)

ROS2 에서 돌아가는 **하나의 프로그램 단위**.  
이 프로젝트는 5개의 노드로 구성됩니다.

```
tof_bridge        → ToF 데이터 변환
obstacle_detector → 장애물 감지 (ToF + LiDAR)
gap_analyzer      → 갭 탐색
avoidance         → 회피 명령 생성
arbitrator        → 최종 명령 선택
```

### 토픽 (Topic)

노드끼리 데이터를 주고받는 **이름 있는 채널**.

```
/scan              ← LiDAR 가 발행
/cmd_vel           ← 모터가 구독
/robot/state       ← 모니터링용
```

### 메시지 (Message)

토픽에서 주고받는 **데이터의 형식**.

```
geometry_msgs/Twist          → linear.x, angular.z (속도 명령)
geometry_msgs/TwistStamped   → Twist + 시간 정보 (Jazzy 기본)
sensor_msgs/LaserScan        → LiDAR 거리 배열
std_msgs/Float32             → 숫자 하나
std_msgs/Bool                → True/False
```

---

## 3. Publisher 와 Subscriber

### Publisher (발행자)

```python
self._pub = self.create_publisher(Float32, '/tof/front_distance', 10)

msg = Float32()
msg.data = 0.45
self._pub.publish(msg)
```

### Subscriber (구독자)

```python
self.create_subscription(Float32, '/tof/front_distance', self._on_tof, 10)

def _on_tof(self, msg: Float32):
    print(f"ToF: {msg.data} m")
```

숫자 `10` = **큐 크기**. 처리가 느려도 최대 10개 메시지를 쌓아둠.

---

## 4. 타이머 (Timer)

일정 주기마다 함수를 자동 실행.

```python
self.create_timer(1.0 / 20.0, self._tick)   # 20 Hz

def _tick(self):
    # 1초에 20번 자동 실행
    ...
```

| Hz | 의미 |
|---|---|
| 10 Hz | gap_analyzer (계산 무거움) |
| 20 Hz | 대부분의 노드 (빠른 응답) |

---

## 5. 파라미터 (Parameter)

코드를 수정하지 않고 동작 값을 바꾸는 설정.

```yaml
obstacle_detector:
  ros__parameters:
    enter_threshold: 0.4
```

```python
self.declare_parameter('enter_threshold', 0.4)
value = self.get_parameter('enter_threshold').value
```

---

## 6. 센서 이해 — LiDAR

레이저를 360° 쏘아 각 방향 거리를 측정.

### LaserScan 메시지

```
angle_min        = 0 또는 -π  (스캔 시작 각도)
angle_max        = 2π 또는 +π
angle_increment  = 0.01745 rad (≈ 1°)
ranges = [1.2, 1.1, 0.8, inf, ...]  ← 각 방향 거리 (m)
```

i번째 빔의 각도 = `angle_min + i × angle_increment`  
`inf` = 측정 안 됨, `nan` = 오류

### 이 프로젝트의 LDS-02

`angle_min = 0`, `angle_max = 2π` 로 발행 → **0~360° 범위**.  
ROS REP-103 의 [-π, +π] 가 아니므로 **각도 정규화 필요** (9장 참조).

---

## 7. 센서 이해 — ToF (Time of Flight)

빛이 돌아오는 **시간**으로 거리 측정.

### VL53L8CX (이 프로젝트 사용 센서)

**8×8 = 64개 픽셀**을 동시에 측정.

```
     col0  col1  col2  col3  col4  col5  col6  col7
row0 [---] [---] [---] [---] [---] [---] [---] [---]
row1 [---] [---] [---] [---] [---] [---] [---] [---]
row2 [---] [---] [300] [280] [310] [290] [---] [---]  ← 중앙 4×4
row3 [---] [---] [295] [270] [305] [285] [---] [---]
row4 [---] [---] [305] [290] [295] [280] [---] [---]
row5 [---] [---] [295] [285] [300] [275] [---] [---]
row6 [---] [---] [---] [---] [---] [---] [---] [---]
row7 [---] [---] [---] [---] [---] [---] [---] [---]
```

`roi_rows: [2, 5]`, `roi_cols: [2, 5]` = 중앙 16개 셀만 사용.

### Status 코드

| 코드 | 의미 |
|---|---|
| 0 | 측정 없음 / 초기값 |
| 5 | 유효한 측정 |
| 6 | 유효 (큰 펄스) |
| 9 | 유효 (wrap-around 없음) |
| 255 | 완전 무효 |

이 프로젝트는 **255 만 제외** 하고 나머지는 모두 수용 (관용적).

---

## 8. 로봇 좌표계와 각도 방향

ROS2 의 표준 좌표계 (REP-103):

```
         앞 (+x)
          ↑
          │
왼쪽(+y) ←┼→ 오른쪽(-y)
          │
          ▼
         뒤 (-x)
```

- `angular.z > 0` → 왼쪽 회전 (반시계)
- `angular.z < 0` → 오른쪽 회전 (시계)

### Twist 명령 예시

```python
cmd = Twist()
cmd.linear.x  = 0.1   # 전진 0.1 m/s
cmd.angular.z = 0.4   # 왼쪽 회전 0.4 rad/s
```

---

## 9. 각도 정규화 — atan2 트릭

이 프로젝트의 LDS-02 는 각도를 `0 → 2π` 로 발행합니다.  
하지만 우리 코드는 `-π → +π` 를 가정합니다.

### 문제

```python
ang = 5.5  # rad (≈ 315°, 오른쪽 방향)
if -0.524 <= ang <= 0.524:   # ±30° 체크
    process(ang)
# 5.5 > 0.524 → 무시됨!
# 오른쪽 빔이 모두 누락
```

### 해결: atan2 정규화

`atan2(sin(x), cos(x))` 는 입력 범위에 상관없이 항상 `[-π, +π]` 로 변환.

```python
import math

ang = 5.5
na = math.atan2(math.sin(ang), math.cos(ang))
# na ≈ -0.78 rad (≈ -45°) ← 올바른 오른쪽 방향
```

### 코드에서

```python
for r in scan.ranges:
    na = math.atan2(math.sin(ang), math.cos(ang))
    if -self._sector <= na <= self._sector:
        process(na, r)
    ang += inc
```

이 한 줄이 빠지면 **장애물의 절반(오른쪽)을 놓치게** 됩니다.

---

## 10. 상태 머신 (State Machine)

현재 상태를 명확히 관리하는 설계 패턴.

```
MANUAL ← → AVOIDANCE ← → STOPPED
```

```python
if obstacle_detected:
    if auto_cmd_stale:
        state = "STOPPED"
    else:
        state = "AVOIDANCE"
else:
    state = "MANUAL"
```

각 상태마다 다른 명령을 발행합니다.

---

## 11. 히스테리시스 (Hysteresis)

진입 조건과 탈출 조건을 다르게 두어 **경계 진동 방지**.

```
AVOIDANCE 진입: 거리 < 0.4 m
AVOIDANCE 탈출: 거리 > 0.5 m

0.4 ~ 0.5 m 구간 = "현재 상태 유지"
```

이 폭이 좁으면 경계에서 상태가 빠르게 진동합니다.

---

## 12. 2단계 감지와 LiDAR 맹점

### 왜 2단계인가?

ToF 단독으로 결정하면 **오감지** 위험이 있습니다.

```
ToF 가 바닥의 작은 자국, 햇빛 반사 등을 잘못 측정
→ 단독 사용 시 갑자기 AVOIDANCE 진입
→ 사용자 불편
```

해결: ToF 가 알리고, **LiDAR 가 확인** 해야 진입.

```
1단계: ToF < 0.4 m       → tof_alert = True
2단계: LiDAR < 0.4 m     → lidar_alert = True
       둘 다 True 일 때만 AVOIDANCE 진입
```

### LiDAR 맹점 문제

LiDAR 는 **최소 측정 거리** 가 있습니다 (LDS-02 = 0.12 m).  
0.12 m 보다 가까이 있는 물체는 inf 또는 0 으로 나옵니다.

```
물체가 0.05 m 까지 접근
→ ToF: 0.05 m (정상 감지)
→ LiDAR: inf (맹점!)
→ 2단계 확인 실패
→ AVOIDANCE 미진입 → 충돌!
```

### 맹점 보정 트릭

ToF 가 가까운 물체를 감지하는데 LiDAR 가 inf 면,  
**"너무 가까워서 LiDAR 가 못 보는 것"** 으로 간주.

```python
tof_alert = (tof_front < 0.4)
lidar_normal = (lidar_front < 0.4)
lidar_blind = tof_alert and (lidar_front == inf)
lidar_alert = lidar_normal or lidar_blind

if tof_alert and lidar_alert:
    enter_avoidance()
```

이게 12 단원의 핵심입니다. "맹점도 신호로 활용".

---

## 13. 공유 제어 (Shared Control) 개념

사람과 자동화 시스템이 제어권을 **나눠 가짐**.

```
완전 수동:  사람 100% → 실수하면 충돌
완전 자동:  로봇 100% → 사람 의도 무시
공유 제어:  평소 사람, 위험 시 로봇이 개입
```

자동차의 자동 긴급제동(AEB)와 같은 개념.

```
MANUAL    → 조이스틱 → /cmd_vel
AVOIDANCE → 자동 회피 → /cmd_vel  (조이스틱 차단)
```

전환은 **arbitrator** 가 담당합니다.

---

## 14. 갭 탐색 알고리즘 (Gap Seeking)

벽이 막아도 **통과할 수 있는 공간(갭)** 을 찾는 알고리즘.

### 1단계: 자유 빔 표시

```
빔의 거리가 robot_width + margin (0.381 m) 보다 멀면 "자유"

각도:  -150° ... -30° ... 0° ... +30° ... +150°
거리:   0.2     1.5   1.8  0.3   0.4    2.0
자유:    ✗       ✓     ✓    ✗    ✗     ✓
```

### 2단계: 연속 자유 구간 (갭) 찾기

```
자유 빔이 연속으로 이어진 구간 → 하나의 갭
→ 여러 개 갭이 생길 수 있음
```

### 3단계: 갭 폭 계산

호의 현(chord) 공식:

```
width = 2 × r_min × sin(span / 2)

r_min: 갭 내 가장 가까운 거리
span:  갭이 차지하는 전체 각도
```

### 4단계: 통과 가능 갭만 선택

```
width >= required_gap (0.381 m) 인 갭만 후보
→ 로봇이 통과할 수 있는 크기 보장
```

### 5단계: 가장 좋은 갭 선택

→ **하이브리드 점수** 사용 (15장)

---

## 15. 하이브리드 점수 (Hybrid Scoring)

여러 갭 중 어느 것을 고를까?

### 옵션 1: 정면에 가장 가까운 갭

```
score = |angle|
→ 의도를 잘 반영하지만, 좁아서 위험한 갭도 선택
```

### 옵션 2: 가장 넓은 갭

```
score = -width
→ 안전하지만, 의도와 다른 옆길로 빠짐
```

### 옵션 3: 하이브리드 (이 프로젝트)

```
score = |angle(°)| − width_weight × width(m) × 100

→ 정면에 가까울수록 좋고, 폭이 넓을수록 좋음
→ 작을수록 좋은 갭
```

### 예시 (width_weight = 0.3)

```
케이스 A: 정면 5° 폭 0.4m vs 옆 40° 폭 0.8m
  정면: 5  - 0.3 × 40 = -7
  옆길: 40 - 0.3 × 80 = 16
  → 정면 선택 (의도 존중)

케이스 B: 정면 5° 폭 0.39m vs 옆 15° 폭 1.2m
  정면: 5  - 0.3 × 39 = -6.7
  옆길: 15 - 0.3 × 120 = -21
  → 옆길 선택 (안전한 넓은 길)
```

`width_weight` 가 커질수록 폭을 더 중시합니다.

---

## 16. 비례 조향 (Proportional Steering)

오차에 비례한 회전 명령을 생성하는 제어 기법.

### 단순 회전 (이전 방식)

```python
if 갭이 왼쪽:
    angular_z = +0.4  # 고정 속도로 왼쪽 회전
elif 갭이 오른쪽:
    angular_z = -0.4
```

**문제**: 갭이 5° 옆에 있어도 40° 옆에 있어도 같은 속도로 회전.  
→ 작은 오차에서 오버슈팅, 큰 오차에서 너무 느림.

### 비례 제어 (현재 방식)

```python
angular_z = k_angular × gap_angle  # 오차에 비례
angular_z = clip(angular_z, ±w_turn)  # 최대 속도 제한

# 예: k_angular = 1.5
gap_angle = 0.087 rad (5°)  → angular_z = 0.13 rad/s (천천히)
gap_angle = 0.7 rad (40°)   → angular_z = 1.05 → 0.4 rad/s (최대로 제한)
```

**장점**: 오차가 작으면 부드럽게, 크면 빠르게 회전.

### 정렬 후 전진

```python
if abs(gap_angle) < align_threshold (20°):
    linear_x = v_forward       # 정렬됐으니 전진
else:
    linear_x = v_forward × 0.2  # 아직 회전 중이니 매우 느리게
```

이러면 회전과 전진이 자연스럽게 결합됩니다.

---

## 17. 이 프로젝트의 5개 노드 흐름

### 단계 1: 센서 → tof_bridge

```
VL53L8CX → /tof/distances (64개 mm)
        → /tof/status     (64개 코드)
              │
              ▼
       [tof_bridge]
       중앙 16개 셀, status≠255, NaN 제외, 최솟값
              │
              ▼
       /tof/front_distance (Float32, m)
```

### 단계 2: 감지 (obstacle_detector)

```
/tof/front_distance ──┐
/scan ─────────────────►[obstacle_detector]
                       │
                       ├─ 1단계: ToF < 0.4 m?
                       ├─ 2단계: LiDAR 정면 < 0.4 m? (or 맹점 보정)
                       └─ 측면: LiDAR 55°~120° < 0.3 m?
                       │
                       ▼
              /obstacle/detected (Bool)
              /obstacle/distance (Float32)
```

### 단계 3: 갭 탐색 (gap_analyzer)

```
/scan ──►[gap_analyzer]
         │
         ├─ ±150° 범위에서 자유 빔 표시
         ├─ 연속 자유 구간 → 갭 후보
         ├─ 폭 계산 → 통과 가능 갭만 필터
         └─ 하이브리드 점수 → 최적 갭 선택
         │
         ▼
    /gap/width    (Float32, m)
    /gap/passable (Bool)
    /gap/angle    (Float32, rad)
```

### 단계 4: 회피 명령 (avoidance)

```
/obstacle/detected ──┐
/gap/passable ────────►[avoidance]
/gap/angle ───────────┤
/scan ────────────────┘
                      │
                      ├─ 장애물 없음 → zero twist
                      ├─ 갭 있음 → 비례 조향 + 정렬 시 전진
                      └─ 갭 없음 → 제자리 회전 (더 열린 쪽)
                      │
                      ▼
              /cmd_vel_auto (Twist)
```

### 단계 5: 중재 (arbitrator)

```
/cmd_vel_manual ──┐
/cmd_vel_auto ─────►[arbitrator]
/obstacle/detected ┘
                    │
                    ├─ MANUAL    → /cmd_vel = /cmd_vel_manual
                    ├─ AVOIDANCE → /cmd_vel = /cmd_vel_auto
                    └─ STOPPED   → /cmd_vel = 0
                    │
                    ▼
              /cmd_vel (TwistStamped)
              /robot/state (String)
                    │
                    ▼
              turtlebot3_node → DXL 모터
```

---

## 18. 코드 패턴 읽는 법 (rclpy)

모든 노드는 같은 구조를 가집니다.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # 1. 파라미터 선언
        self.declare_parameter('my_value', 1.0)
        self._value = self.get_parameter('my_value').value

        # 2. Publisher 생성
        self._pub = self.create_publisher(Float32, '/my_topic', 10)

        # 3. Subscriber 생성
        self.create_subscription(Float32, '/input', self._on_input, 10)

        # 4. 타이머 생성
        self.create_timer(0.05, self._tick)   # 20 Hz

    def _on_input(self, msg: Float32):
        # 데이터 오면 자동 호출
        self._latest = msg.data

    def _tick(self):
        # 20 Hz 마다 자동 호출
        out = Float32()
        out.data = self._latest * self._value
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 핵심 메서드

| 메서드 | 역할 |
|---|---|
| `self.declare_parameter(이름, 기본값)` | 파라미터 등록 |
| `self.get_parameter(이름).value` | 값 읽기 |
| `self.create_publisher(타입, 토픽, 큐)` | 발행자 |
| `self.create_subscription(타입, 토픽, 콜백, 큐)` | 구독자 |
| `self.create_timer(주기초, 콜백)` | 타이머 |
| `self.get_clock().now()` | 현재 시간 |
| `self.get_logger().info("...")` | 로그 출력 |

---

## 19. 파라미터 튜닝 가이드

### 자주 묻는 시나리오

**"장애물에 너무 가까이 가서야 회피한다"**
```yaml
obstacle_detector:
  enter_threshold: 0.4 → 0.55
  lidar_confirm_threshold: 0.4 → 0.55
```

**"멀리서부터 자꾸 회피해서 부자연스럽다"**
```yaml
obstacle_detector:
  enter_threshold: 0.4 → 0.3
```

**"옆길로 자주 새서 의도와 다르게 간다"**
```yaml
gap_analyzer:
  width_weight: 0.3 → 0.15
```

**"좁은 길 통과는 위험하니 넓은 길로 가게 하고 싶다"**
```yaml
gap_analyzer:
  width_weight: 0.3 → 0.5
  margin: 0.10 → 0.15
  required_gap: 0.381 → 0.431  # = 0.281 + 0.15
```

**"회전이 너무 급격하다"**
```yaml
avoidance:
  k_angular: 1.5 → 0.8
  w_turn: 0.4 → 0.3
```

**"대각으로 벽에 접근하면 옆구리가 부딪힌다"**
```yaml
obstacle_detector:
  side_threshold: 0.30 → 0.40
  side_sector_lo_deg: 55.0 → 45.0  # 더 정면 가까이까지 측면으로 봄
```

**"갭이 측면에 있을 때 못 본다"**
```yaml
gap_analyzer:
  search_sector_deg: 150.0 → 180.0  # 거의 360° 가까이
```

---

## 20. 자주 보는 ROS2 명령어 모음

### 토픽 관련

```bash
# 활성 토픽 목록
ros2 topic list

# 데이터 실시간 확인
ros2 topic echo /robot/state
ros2 topic echo /tof/front_distance
ros2 topic echo /gap/angle

# 발행 주파수
ros2 topic hz /cmd_vel

# 토픽 타입 확인
ros2 topic info /cmd_vel

# 토픽에 직접 발행 (테스트)
ros2 topic pub /cmd_vel geometry_msgs/TwistStamped \
  "{twist: {linear: {x: 0.1}, angular: {z: 0.0}}}" --once
```

### 노드 관련

```bash
ros2 node list
ros2 node info /arbitrator
```

### 파라미터 관련

```bash
ros2 param list /obstacle_detector
ros2 param get /obstacle_detector enter_threshold

# 실행 중 변경 (재시작 없이)
ros2 param set /obstacle_detector enter_threshold 0.5
```

### 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select shared_control
source install/setup.bash
```

### 실행

```bash
# 전체 launch
ros2 launch shared_control bringup.launch.py

# 단일 노드 (테스트)
ros2 run shared_control tof_bridge
ros2 run shared_control gap_analyzer
```

---

## 개념 연결 지도

```
[8BitDo 컨트롤러]
       │ 블루투스
       ▼
  [joy_node]  →  /joy  →  [teleop_twist_joy]
                                  │
                                  ▼
                         /cmd_vel_manual (Twist)
                                  │
                                  ▼
[VL53L8CX]──[tof_publisher]──/tof/distances
                  │
            [tof_bridge]
                  │
                  ▼
       /tof/front_distance (Float32)
                  │
[LDS-02]──/scan ──┤  ← atan2 각도 정규화
                  ▼
         [obstacle_detector]   ← 2단계 감지 + 측면 보호
                  │
            /obstacle/detected
                  │
       ┌──────────┤
       ▼          ▼
[gap_analyzer]   [avoidance]    ← 비례 조향
  ±150° 탐색     /cmd_vel_auto
  하이브리드 점수      │
       │              │
       └──┬───────────┘
          ▼
    [arbitrator]    ← 상태 머신
          │
          ▼
    /cmd_vel (TwistStamped)
          │
          ▼
    [turtlebot3_node]
          │
          ▼
       DXL 모터
```

---

> 핵심 한 줄 요약:  
> **"센서가 위험을 알리면 LiDAR 로 가장 좋은 통로를 찾아 자동으로 통과시키는 로봇"**
