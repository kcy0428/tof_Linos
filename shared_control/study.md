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
9. [상태 머신 (State Machine)](#9-상태-머신-state-machine)
10. [히스테리시스 (Hysteresis)](#10-히스테리시스-hysteresis)
11. [공유 제어 (Shared Control) 개념](#11-공유-제어-shared-control-개념)
12. [이 프로젝트의 5개 노드를 순서대로 이해하기](#12-이-프로젝트의-5개-노드를-순서대로-이해하기)
13. [코드 패턴 읽는 법 (rclpy)](#13-코드-패턴-읽는-법-rclpy)
14. [파라미터 튜닝 — 어떤 값을 왜 바꾸나](#14-파라미터-튜닝--어떤-값을-왜-바꾸나)
15. [자주 보는 ROS2 명령어 모음](#15-자주-보는-ros2-명령어-모음)

---

## 1. ROS2 가 뭔가요?

**ROS2 (Robot Operating System 2)** 는 로봇 소프트웨어를 만들기 위한  
**프레임워크(틀)** 입니다. 운영체제가 아니라, Ubuntu 위에서 동작하는  
로봇 전용 미들웨어입니다.

### 왜 ROS2 를 사용하나요?

로봇에는 수많은 부품이 있습니다.

```
[LiDAR] [ToF 센서] [카메라] [모터] [컨트롤러]
```

이것들이 서로 데이터를 주고받아야 하는데, 각 부품마다 통신 방식을  
직접 구현하면 너무 복잡합니다. ROS2 는 이 통신을 표준화해서  
"토픽"이라는 공통 채널로 모든 부품이 대화할 수 있게 합니다.

### ROS2 Jazzy 가 뭔가요?

ROS2 는 버전마다 이름이 있습니다. **Jazzy Jalisco** 는 2024년 출시된  
버전으로 Ubuntu 24.04 와 함께 쓰입니다. 이 프로젝트가 사용하는 버전입니다.

---

## 2. 노드·토픽·메시지 — ROS2 의 핵심 3요소

### 노드 (Node)

**노드**는 ROS2 에서 돌아가는 **하나의 프로그램 단위**입니다.

이 프로젝트의 노드들:

```
tof_bridge        → ToF 데이터 변환 담당
obstacle_detector → 장애물 감지 담당
gap_analyzer      → 갭 분석 담당
avoidance         → 회피 명령 생성 담당
arbitrator        → 최종 명령 선택 담당
```

각 노드는 **한 가지 일만** 합니다. 이렇게 나누면 고장이 났을 때  
어떤 노드가 문제인지 쉽게 찾을 수 있습니다.

### 토픽 (Topic)

**토픽**은 노드들이 데이터를 주고받는 **채널(통로)** 입니다.  
이름이 있는 메일함이라고 생각하세요.

```
/scan              ← LiDAR 가 데이터를 이 토픽에 넣어줌
/cmd_vel           ← 이 토픽에 명령을 넣으면 모터가 움직임
/robot/state       ← 현재 상태를 이 토픽에 넣어서 알림
```

### 메시지 (Message)

**메시지**는 토픽에서 주고받는 **데이터의 형식**입니다.  
택배 박스의 내용물 구조라고 생각하세요.

```
geometry_msgs/Twist       → linear.x, angular.z 같은 속도값을 담는 형식
std_msgs/Float32          → 숫자 하나를 담는 형식
std_msgs/Bool             → True/False 를 담는 형식
sensor_msgs/LaserScan     → LiDAR 거리 배열을 담는 형식
```

### 세 가지 관계 정리

```
노드 A                  토픽                  노드 B
(발행자)  ──데이터──►  /어떤_토픽  ──데이터──►  (구독자)

예시:
tof_bridge ──Float32──► /tof/front_distance ──Float32──► obstacle_detector
```

---

## 3. Publisher 와 Subscriber

토픽에서 데이터를 **보내는 쪽**이 Publisher(발행자),  
**받는 쪽**이 Subscriber(구독자)입니다.

### 코드로 보기

```python
# Publisher 만들기 (데이터를 보낼 준비)
self.pub = self.create_publisher(Float32, '/tof/front_distance', 10)

# 실제로 데이터 보내기
msg = Float32()
msg.data = 0.45   # 0.45 m 라는 거리값
self.pub.publish(msg)
```

```python
# Subscriber 만들기 (데이터를 받을 준비)
self.create_subscription(Float32, '/tof/front_distance', self._on_tof, 10)

# 데이터가 오면 자동으로 이 함수가 호출됨
def _on_tof(self, msg: Float32):
    print(f"ToF 거리: {msg.data} m")
```

숫자 `10` 은 **큐 크기(queue size)** 입니다. 처리가 느릴 때 최대 10개까지  
메시지를 쌓아두겠다는 뜻입니다. 보통 10 으로 두면 됩니다.

---

## 4. 타이머 (Timer)

**타이머**는 일정 주기마다 함수를 자동으로 실행시켜 줍니다.

```python
# 0.05초마다 (= 20 Hz) _tick 함수를 자동 실행
self.create_timer(1.0 / 20.0, self._tick)

def _tick(self):
    # 이 함수가 초당 20번 실행됨
    self.publish_data()
```

### Hz (헤르츠) 란?

**1 Hz = 1초에 1번**. 20 Hz = 1초에 20번.

```
1 Hz  → 1초에 1번  → 느린 센서나 상태 발행용
10 Hz → 1초에 10번 → 중간 (gap_analyzer)
20 Hz → 1초에 20번 → 빠른 제어 루프 (대부분의 노드)
```

이 프로젝트에서 `publish_rate: 20.0` 이라는 파라미터는  
"1초에 20번 발행하라"는 뜻입니다.

---

## 5. 파라미터 (Parameter)

**파라미터**는 코드를 수정하지 않고 **동작 값을 바꿀 수 있는 설정값**입니다.  
게임에서 난이도나 옵션을 설정하는 것과 비슷합니다.

```yaml
# config/shared_control_params.yaml

obstacle_detector:
  ros__parameters:
    enter_threshold: 0.5   # ← 이 값을 바꾸면 감지 민감도가 달라짐
    exit_threshold: 0.8
```

코드 안에서는 이렇게 읽습니다:

```python
self.declare_parameter('enter_threshold', 0.5)   # 기본값 0.5 로 등록
value = self.get_parameter('enter_threshold').value  # 실제 값 읽기
```

yaml 파일에서 값을 바꾸면 코드를 건드리지 않아도 동작이 달라집니다.  
**코드 수정 없이 튜닝할 수 있다**는 게 파라미터의 핵심 장점입니다.

---

## 6. 센서 이해 — LiDAR

**LiDAR (Light Detection And Ranging)** 은 레이저를 360° 빠르게 쏘면서  
각 방향의 거리를 측정하는 센서입니다.

### LaserScan 메시지 구조

```
/scan 토픽으로 오는 데이터:

angle_min     = -π (-180°)   ← 스캔 시작 각도
angle_max     = +π (+180°)   ← 스캔 끝 각도
angle_increment = 0.00174 rad  ← 빔 사이 간격 (약 0.1°)
ranges = [1.2, 1.1, 0.8, inf, 1.5, ...]  ← 각 방향의 거리 (m)
```

`ranges` 배열에서 i번째 값은:
- `angle_min + i × angle_increment` 방향의 거리입니다.
- `inf` 는 그 방향에 아무것도 없음 (측정 불가)
- `nan` 은 오류

### 코드에서 전방 ±30° 만 보는 법

```python
ang = scan.angle_min     # 시작 각도
for r in scan.ranges:
    if -0.524 <= ang <= 0.524:   # ±30° = ±0.524 rad
        # 이 빔은 전방 섹터에 있음
        process(r)
    ang += scan.angle_increment  # 다음 빔으로
```

---

## 7. 센서 이해 — ToF (Time of Flight)

**ToF 센서**는 빛을 쏘고 돌아오는 **시간**으로 거리를 측정합니다.  
이 프로젝트의 VL53L8CX 는 8×8 = **64개 픽셀**을 동시에 측정하는  
2D 배열형 ToF 센서입니다.

### 8×8 그리드 구조

```
센서 전면에서 바라본 측정 격자:

     col0  col1  col2  col3  col4  col5  col6  col7
row0 [---] [---] [---] [---] [---] [---] [---] [---]
row1 [---] [---] [---] [---] [---] [---] [---] [---]
row2 [---] [---] [300] [280] [310] [290] [---] [---]  ← 전방 중앙
row3 [---] [---] [295] [270] [305] [285] [---] [---]  ← 전방 중앙
row4 [---] [---] [305] [290] [295] [280] [---] [---]  ← 전방 중앙
row5 [---] [---] [295] [285] [300] [275] [---] [---]  ← 전방 중앙
row6 [---] [---] [---] [---] [---] [---] [---] [---]
row7 [---] [---] [---] [---] [---] [---] [---] [---]

값: mm 단위 거리 (---는 NaN 또는 측정 불가)
```

`roi_rows: [2, 5]`, `roi_cols: [2, 5]` 설정은  
"중앙 4×4 영역(16개 셀)만 보겠다"는 뜻입니다.

### Status 코드란?

각 픽셀마다 측정의 신뢰도를 나타내는 코드가 함께 옵니다:

| 코드 | 의미 |
|---|---|
| 0 | 초기값 / 측정 없음 |
| 5 | 유효한 측정 |
| 6 | 유효 (큰 펄스) |
| 9 | 유효 (wrap-around 없음) |
| 255 | 완전 무효 |

tof_bridge 는 `255` 인 픽셀만 제외하고 나머지는 모두 사용합니다.

---

## 8. 로봇 좌표계와 각도 방향

ROS2 는 **REP-103** 이라는 표준 좌표계를 따릅니다.

```
         앞 (+x)
          ↑
          │
왼쪽(+y) ←┼→ 오른쪽(-y)
          │
          ↓
         뒤 (-x)
```

- **+x**: 앞으로 이동
- **+y**: 왼쪽으로 이동 (로봇 기준)
- **+z 회전**: 반시계 방향 (왼쪽으로 회전)
- **-z 회전**: 시계 방향 (오른쪽으로 회전)

### Twist 메시지와 로봇 동작

```python
cmd = Twist()
cmd.linear.x = 0.1   # 앞으로 0.1 m/s
cmd.angular.z = 0.0  # 회전 없음  → 직진

cmd.linear.x = 0.0
cmd.angular.z = 0.4  # 왼쪽으로 0.4 rad/s 회전 → 제자리 좌회전

cmd.linear.x = -0.05
cmd.angular.z = 0.0  # 후진
```

### LiDAR 각도와 방향

LiDAR 도 같은 좌표계를 따릅니다:

```
angle = 0      → 정면
angle = +π/2   → 왼쪽 90°
angle = -π/2   → 오른쪽 90°
angle = ±π     → 정후방
```

그래서 코드에서 `ang > 0` 이면 왼쪽, `ang < 0` 이면 오른쪽 방향입니다.

---

## 9. 상태 머신 (State Machine)

**상태 머신**은 "현재 어떤 상태인지"를 명확하게 관리하는 설계 패턴입니다.  
신호등을 생각하면 이해하기 쉽습니다.

```
신호등 상태 머신:
  빨간불 ──일정 시간 후──► 초록불
  초록불 ──일정 시간 후──► 노란불
  노란불 ──일정 시간 후──► 빨간불
```

### 이 프로젝트의 상태 머신

```
MANUAL ← → AVOIDANCE ← → STOPPED
```

상태가 **3개** 있고, 각 상태에서 조건이 만족되면 다른 상태로 전환됩니다.

```python
# 코드 구조
if obstacle_detected:
    if auto_cmd_stale:
        state = "STOPPED"
    else:
        state = "AVOIDANCE"
else:
    state = "MANUAL"
```

### 왜 상태 머신을 쓰나요?

단순히 `if obstacle: auto_mode` 로 하면 안 될까요?

안 됩니다. 예를 들어:
- 장애물이 감지됐다가 사라졌다가를 빠르게 반복하면  
  모드가 계속 바뀌어서 로봇이 불안정하게 동작합니다.
- 상태 머신 + 히스테리시스를 쓰면 이런 문제를 방지합니다.

---

## 10. 히스테리시스 (Hysteresis)

**히스테리시스**는 진입 조건과 탈출 조건을 **다르게** 설정해서  
경계 근처에서 상태가 빠르게 왔다 갔다 하는 것을 막는 기법입니다.

### 예시: 온도 조절기

```
냉방 ON  조건: 온도 > 26°C
냉방 OFF 조건: 온도 < 24°C  ← 진입(26)과 탈출(24)이 다름
```

25°C 에서 냉방이 계속 켜졌다 꺼졌다 하지 않습니다.

### 이 프로젝트에서

```
AVOIDANCE 진입: 거리 < 0.5 m
AVOIDANCE 탈출: 거리 > 0.8 m
```

```
        거리
  ─────────────────────────────────────────────►
  0   0.3  0.5  0.6  0.7  0.8  1.0

  ←────────────────┤                   AVOIDANCE 진입 임계
                             ├────────► MANUAL 복귀 임계
              ├──────────────┤
              이 구간에서는 현재 상태 유지 (히스테리시스 영역)
```

0.5 m 가까이 접근하면 AVOIDANCE 로 들어가고,  
0.8 m 이상 멀어져야 MANUAL 로 돌아옵니다.  
0.5~0.8 m 사이에서는 **현재 상태를 그대로 유지**합니다.

---

## 11. 공유 제어 (Shared Control) 개념

**공유 제어**는 사람과 자동화 시스템이 **제어권을 나눠 갖는** 방식입니다.

### 완전 수동 vs 완전 자동 vs 공유 제어

```
완전 수동:  사람이 100% 조종 → 실수하면 충돌
완전 자동:  로봇이 100% 결정 → 사람의 의도를 무시
공유 제어:  평소엔 사람이 조종, 위험 시에만 로봇이 개입
```

### 이 프로젝트의 공유 제어

```
평상시 (MANUAL):
  사람 명령  ──────────────────────────► 모터
  로봇 회피  ✗ (무시됨)

위험 시 (AVOIDANCE):
  사람 명령  ✗ (차단됨)
  로봇 회피  ──────────────────────────► 모터
```

**중재기(arbitrator)** 가 이 전환을 담당합니다.  
마치 자동차의 자동 긴급제동(AEB) 처럼 평소엔 사람이 운전하다가  
위험할 때만 시스템이 개입합니다.

---

## 12. 이 프로젝트의 5개 노드를 순서대로 이해하기

데이터가 센서에서 모터까지 어떤 경로로 흐르는지 단계별로 봅니다.

### 단계 1: 센서 → tof_bridge

```
VL53L8CX 센서
     │
     │ 직렬통신 (/dev/tof_sensor)
     ▼
[tof_publisher]  →  /tof/distances (64개 숫자, mm)
                     /tof/status   (64개 상태코드)
     │
     ▼
[tof_bridge]
  "64개 중 중앙 16개만 보고, 유효한 것 중 가장 가까운 값을 뽑자"
     │
     ▼
/tof/front_distance (숫자 1개, m)  예: 0.42
```

**왜 이게 필요한가?**  
센서는 항상 64개 숫자를 주는데, 우리가 필요한 건  
"전방에 뭔가가 얼마나 가까이 있는가?"라는 단일 값입니다.

### 단계 2: ToF + LiDAR → obstacle_detector

```
/tof/front_distance  → 0.42 m (ToF 로 잰 전방 거리)
/scan                → 0.55 m (LiDAR 전방 ±30° 최솟값)

obstacle_detector:
  fused = min(0.42, 0.55) = 0.42 m  ← 둘 중 더 가까운 값 사용
  0.42 < 0.5 m → 장애물 감지!

     │
     ▼
/obstacle/detected  = True
/obstacle/distance  = 0.42
```

**왜 두 센서를 합치나요?**  
ToF 는 좁은 영역을, LiDAR 는 넓은 영역을 봅니다.  
둘 중 하나라도 장애물을 감지하면 알아야 안전합니다.

### 단계 3: LiDAR → gap_analyzer

```
/scan (전방 ±30° 빔들)

gap_analyzer:
  빔마다 "0.381 m 보다 멀면 = 통과 가능"을 판단
  
  예시 (각 | 는 하나의 빔):
  
  각도:  -30° ... -10° ... 0° ... +10° ... +30°
  거리:   0.2   0.8  0.9  0.7   0.8   0.9   0.3
  통과:    ✗     ✓    ✓    ✓    ✓     ✓     ✗
  
  가장 긴 연속 "통과 가능" 구간 → 갭 폭 계산
  
     │
     ▼
/gap/width    = 0.8 m
/gap/passable = True  (0.8 >= 0.381)
```

**갭 폭 계산 공식**:

```
width = 2 × r_min × sin(구간 각도 / 2)

r_min: 갭 안에서 가장 가까운 거리 (가장 보수적인 추정)
구간 각도: 연속 구간이 차지하는 전체 각도
```

왜 `sin` 을 쓰나요? 거리 r 에서 각도 θ 를 가진 호의 폭(현, chord)이  
`2r × sin(θ/2)` 이기 때문입니다.

### 단계 4: → avoidance

```
/obstacle/detected = True
/gap/passable      = True

avoidance:
  "갭이 충분히 넓으니 그냥 전진"
  linear.x = 0.08 m/s
  angular.z = 0.0

     │
     ▼
/cmd_vel_auto → Twist(linear.x=0.08, angular.z=0)
```

```
/obstacle/detected = True
/gap/passable      = False
/scan (좌우 비교)  → 왼쪽이 더 열려 있음

avoidance:
  "갭이 좁으니 더 열린 왼쪽으로 회전"
  angular.z = +0.4  (양수 = 왼쪽 회전)

     │
     ▼
/cmd_vel_auto → Twist(linear.x=0, angular.z=0.4)
```

**좌우 판단 방법**:

```python
# 각 방향의 거리를 합산
# 거리가 길수록 점수가 높음 = 더 열려 있음
left_score  = sum(좌측 빔 거리들)   # 클수록 왼쪽이 열림
right_score = sum(우측 빔 거리들)   # 클수록 오른쪽이 열림

if left_score >= right_score:
    turn_left()    # 왼쪽이 더 열려 있음
else:
    turn_right()
```

### 단계 5: arbitrator (최종 명령 선택)

```
상태: MANUAL
  /cmd_vel_manual → (조이스틱에서 온 명령)
  → 그대로 /cmd_vel 로 전달

상태: AVOIDANCE
  /cmd_vel_auto → (회피 알고리즘이 만든 명령)
  → 그대로 /cmd_vel 로 전달
  (조이스틱 명령은 무시)

상태: STOPPED
  → Twist(0,0) 발행 (정지)
```

모터는 `/cmd_vel` 만 봅니다. 누가 보냈는지는 신경 쓰지 않습니다.

---

## 13. 코드 패턴 읽는 법 (rclpy)

모든 노드는 거의 같은 구조를 가집니다.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MyNode(Node):         # Node 클래스를 상속
    def __init__(self):
        super().__init__('my_node')  # 노드 이름 등록

        # 1. 파라미터 선언
        self.declare_parameter('my_value', 1.0)
        self._value = self.get_parameter('my_value').value

        # 2. Publisher 만들기
        self._pub = self.create_publisher(Float32, '/my_topic', 10)

        # 3. Subscriber 만들기
        self.create_subscription(Float32, '/input_topic', self._on_input, 10)

        # 4. 타이머 만들기 (주기적 실행)
        self.create_timer(0.05, self._tick)  # 20 Hz

    def _on_input(self, msg: Float32):
        # 구독 토픽에서 데이터가 오면 자동 호출
        self._latest = msg.data

    def _tick(self):
        # 20 Hz 마다 자동 호출
        out = Float32()
        out.data = self._latest * self._value
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)          # ROS2 초기화
    node = MyNode()
    rclpy.spin(node)               # 노드 실행 (Ctrl+C 까지 계속)
    node.destroy_node()
    rclpy.shutdown()
```

### 핵심 메서드 정리

| 메서드 | 하는 일 |
|---|---|
| `self.declare_parameter(이름, 기본값)` | 파라미터 등록 |
| `self.get_parameter(이름).value` | 파라미터 값 읽기 |
| `self.create_publisher(타입, 토픽명, 큐)` | 발행자 생성 |
| `self.create_subscription(타입, 토픽명, 콜백, 큐)` | 구독자 생성 |
| `self.create_timer(주기초, 콜백)` | 타이머 생성 |
| `self.get_clock().now()` | 현재 시간 가져오기 |
| `self.get_logger().info("메시지")` | 로그 출력 |

---

## 14. 파라미터 튜닝 — 어떤 값을 왜 바꾸나

### 장애물 감지가 너무 민감할 때 (자꾸 AVOIDANCE 로 들어감)

```yaml
obstacle_detector:
  ros__parameters:
    enter_threshold: 0.5  →  0.35  # 더 가까이 와야 감지
```

### 장애물에서 벗어났는데도 MANUAL 로 안 돌아올 때

```yaml
obstacle_detector:
  ros__parameters:
    exit_threshold: 0.8  →  0.6  # 덜 멀어도 복귀
```

### 회피할 때 회전이 너무 빠를 때 (로봇이 급하게 돔)

```yaml
avoidance:
  ros__parameters:
    w_turn: 0.4  →  0.25  # 회전 속도 낮춤
```

### 갭을 통과할 때 전진이 너무 빠를 때

```yaml
avoidance:
  ros__parameters:
    v_forward: 0.08  →  0.05  # 전진 속도 낮춤
```

### 좁은 통로도 통과하게 하고 싶을 때 (위험할 수 있음!)

```yaml
gap_analyzer:
  ros__parameters:
    margin: 0.10  →  0.05   # 안전 마진 줄임
    required_gap: 0.381  →  0.331  # = robot_width + margin
```

### 조이스틱 최대 속도를 높이고 싶을 때

```yaml
# config/teleop_twist_joy.yaml
teleop_twist_joy_node:
  ros__parameters:
    scale_linear.x: 0.20  →  0.30   # 일반 전진 빠르게
    scale_angular.yaw: 1.0  →  1.2  # 회전도 빠르게
```

---

## 15. 자주 보는 ROS2 명령어 모음

### 토픽 관련

```bash
# 현재 활성화된 모든 토픽 목록
ros2 topic list

# 특정 토픽의 데이터를 출력 (Ctrl+C 로 종료)
ros2 topic echo /robot/state
ros2 topic echo /tof/front_distance

# 토픽 발행 주파수 확인
ros2 topic hz /cmd_vel

# 토픽의 메시지 타입 확인
ros2 topic info /cmd_vel

# 토픽에 직접 데이터 보내기 (테스트용)
ros2 topic pub /cmd_vel geometry_msgs/TwistStamped \
  "{twist: {linear: {x: 0.1}, angular: {z: 0.0}}}" --once
```

### 노드 관련

```bash
# 현재 실행 중인 노드 목록
ros2 node list

# 특정 노드가 어떤 토픽을 구독/발행하는지 확인
ros2 node info /arbitrator
ros2 node info /obstacle_detector
```

### 파라미터 관련

```bash
# 실행 중인 노드의 파라미터 목록
ros2 param list /obstacle_detector

# 파라미터 현재 값 확인
ros2 param get /obstacle_detector enter_threshold

# 실행 중에 파라미터 값 바꾸기 (재실행 없이)
ros2 param set /obstacle_detector enter_threshold 0.4
```

### 빌드 관련

```bash
# 패키지 빌드
colcon build --packages-select shared_control

# 빌드 결과 환경 적용 (터미널마다 해야 함)
source install/setup.bash

# 빌드 + 적용 한 번에
colcon build --packages-select shared_control && source install/setup.bash
```

### 실행 관련

```bash
# launch 파일로 전체 스택 실행
ros2 launch shared_control bringup.launch.py

# 노드 하나만 단독 실행 (테스트용)
ros2 run shared_control tof_bridge
ros2 run shared_control obstacle_detector
ros2 run shared_control arbitrator
```

---

## 개념 연결 지도

```
[8BitDo 컨트롤러]
       │ 블루투스
       ▼
  [joy_node]  →  /joy  →  [teleop_twist_joy]
                                  │
                    (Twist: 속도명령으로 변환)
                                  │
                                  ▼
                         /cmd_vel_manual
                                  │
                                  ▼
[VL53L8CX]──►[tof_publisher]──►/tof/distances
                     │
               [tof_bridge]        이 두 거리 중
                     │         ┌── 더 가까운 걸 씀
                     ▼         │
            /tof/front_distance┘
                               │
[LDS LiDAR]──────────────►/scan┤
                               ▼
                    [obstacle_detector]
                         │        │
                  /obstacle/   /obstacle/
                  detected     distance
                         │
                    (히스테리시스)
                         │
          ┌──────────────┤
          ▼              ▼
   [gap_analyzer]   [avoidance]
     /gap/width       /cmd_vel_auto
     /gap/passable ───────►┘
                           │
                           ▼
                     [arbitrator]  ← 상태머신
                    (MANUAL / AVOIDANCE / STOPPED)
                           │
                    조건에 따라 선택:
                    cmd_vel_manual 또는 cmd_vel_auto
                           │
                           ▼
                       /cmd_vel  (TwistStamped)
                           │
                           ▼
                      [turtlebot3_node]
                           │
                           ▼
                        DXL 모터 구동
```

---

> 이 프로젝트를 한 문장으로 요약하면:  
> **"센서 데이터를 보고 위험하면 자동으로 피하고, 안전하면 사람이 운전하는 로봇"**
