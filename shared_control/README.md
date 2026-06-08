# shared_control

TurtleBot3 Waffle (Raspberry Pi 5 · Ubuntu Server 24.04 · ROS2 Jazzy) 용  
**공유 제어(Shared Control) 장애물 회피 패키지**

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [하드웨어 구성](#2-하드웨어-구성)
3. [전체 토픽 흐름](#3-전체-토픽-흐름)
4. [상태 머신](#4-상태-머신)
5. [패키지 구조](#5-패키지-구조)
6. [노드별 동작 설명](#6-노드별-동작-설명)
7. [파라미터 설정 방법](#7-파라미터-설정-방법)
8. [컨트롤러 버튼 설정](#8-컨트롤러-버튼-설정)
9. [빌드 방법](#9-빌드-방법)
10. [실행 방법](#10-실행-방법)
11. [VMware 원격 모니터링](#11-vmware-원격-모니터링)
12. [트러블슈팅](#12-트러블슈팅)

---

## 1. 시스템 개요

평상시에는 사용자가 **8BitDo Micro 블루투스 컨트롤러**로 로봇을 직접 조종합니다.  
전방 ToF 센서 또는 LiDAR가 **장애물을 감지하면 사용자 입력을 차단**하고 자동 회피 알고리즘이 로봇을 제어합니다.  
장애물이 사라지면 **자동으로 수동 조종으로 복귀**합니다.

```
사용자 입력  ──┐
               ▼
         [ 중재기 (arbitrator) ]  ──► /cmd_vel ──► 로봇
               ▲
자동 회피  ────┘
```

---

## 2. 하드웨어 구성

| 구성요소 | 사양 |
|---|---|
| 로봇 | TurtleBot3 Waffle |
| 컴퓨터 | Raspberry Pi 5 |
| OS | Ubuntu Server 24.04 |
| ROS | ROS2 Jazzy |
| LiDAR | LDS-02 (360° · `/scan`) |
| ToF 센서 | VL53L8CX 8×8 그리드 (전방 · `/tof/distances`) |
| 컨트롤러 | 8BitDo Micro (블루투스) |

---

## 3. 전체 토픽 흐름

```
[8BitDo Micro]
      │ 블루투스
      ▼
[joy_node] ──────────────────────────► /joy  (sensor_msgs/Joy)
      │
      ▼
[teleop_twist_joy] ──────────────────► /cmd_vel_manual  (geometry_msgs/Twist)
      │ (리매핑: /cmd_vel → /cmd_vel_manual)


[VL53L8CX 센서] ─ /dev/tof_sensor ─►
[tof_publisher] ─────────────────────► /tof/distances  (Float32MultiArray 8×8, mm)
                                        /tof/status     (UInt8MultiArray  8×8)
      │
      ▼
[tof_bridge] ────────────────────────► /tof/front_distance  (Float32, m)


[LDS-02 LiDAR] ──────────────────────► /scan  (sensor_msgs/LaserScan)


/tof/front_distance ──┐
/scan ─────────────────►[obstacle_detector]──► /obstacle/detected  (Bool)
                                            └─► /obstacle/distance  (Float32, m)

/scan ──────────────────►[gap_analyzer] ─────► /gap/width    (Float32, m)
                                            └─► /gap/passable (Bool)

/scan ─────────────────┐
/obstacle/detected ────►[avoidance] ─────────► /cmd_vel_auto  (geometry_msgs/Twist)
/gap/width ────────────┤
/gap/passable ─────────┘

/cmd_vel_manual ───────┐
/cmd_vel_auto ─────────►[arbitrator] ────────► /cmd_vel      (geometry_msgs/TwistStamped)
/obstacle/detected ────┘                   └─► /robot/state  (String)
```

### 전체 토픽 목록

| 토픽 | 타입 | 방향 | 설명 |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | 입력 | 컨트롤러 원시 데이터 |
| `/cmd_vel_manual` | `geometry_msgs/Twist` | 입력 | 조이스틱 변환 명령 |
| `/tof/distances` | `std_msgs/Float32MultiArray` | 입력 | ToF 8×8 그리드 (mm) |
| `/tof/status` | `std_msgs/UInt8MultiArray` | 입력 | ToF 셀별 상태 코드 |
| `/scan` | `sensor_msgs/LaserScan` | 입력 | LiDAR 360° 스캔 |
| `/tof/front_distance` | `std_msgs/Float32` | 내부 | 전방 ToF 거리 (m) |
| `/obstacle/detected` | `std_msgs/Bool` | 출력 | 장애물 감지 여부 |
| `/obstacle/distance` | `std_msgs/Float32` | 출력 | 융합 장애물 거리 (m) |
| `/gap/width` | `std_msgs/Float32` | 출력 | 통과 가능 갭 폭 (m) |
| `/gap/passable` | `std_msgs/Bool` | 출력 | 갭 통과 가능 여부 |
| `/robot/state` | `std_msgs/String` | 출력 | 현재 상태 (MANUAL/AVOIDANCE/STOPPED) |
| `/cmd_vel_auto` | `geometry_msgs/Twist` | 내부 | 자동 회피 명령 |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | 출력 | 최종 모터 명령 |

---

## 4. 상태 머신

```
                  초기 상태
                      │
                      ▼
              ┌───────────────┐
              │    MANUAL     │◄──────────────────────────┐
              │               │   ToF > 0.8m              │
              │ /cmd_vel =    │   AND LiDAR 전방 청결      │
              │ /cmd_vel_     │                           │
              │   manual      │                           │
              └───────────────┘                           │
                      │                                   │
              ToF < 0.5m                                  │
              (장애물 감지)                                │
                      │                                   │
                      ▼                                   │
              ┌───────────────┐                           │
              │   AVOIDANCE   │───────────────────────────┘
              │               │
              │ /cmd_vel =    │
              │ /cmd_vel_auto │
              └───────────────┘
                      │
              /cmd_vel_auto 가
              0.3초 이상 미수신
                      │
                      ▼
              ┌───────────────┐
              │    STOPPED    │
              │               │
              │ /cmd_vel = 0  │
              └───────────────┘
                      │
              새 /cmd_vel_auto 수신
                      │
                      └──► AVOIDANCE 로 복귀
```

### 전이 조건 요약

| 전이 | 조건 |
|---|---|
| MANUAL → AVOIDANCE | 전방 거리 < **0.5 m** |
| AVOIDANCE → MANUAL | 전방 거리 > **0.8 m** (히스테리시스) |
| AVOIDANCE → STOPPED | `/cmd_vel_auto` 가 **0.3 초** 이상 도착 안 함 |
| STOPPED → AVOIDANCE | 새로운 `/cmd_vel_auto` 수신 |

> **히스테리시스**: 진입(0.5 m)과 탈출(0.8 m) 임계값이 달라서  
> 경계 근처에서 상태가 빠르게 왔다 갔다 하는 것을 방지합니다.

---

## 5. 패키지 구조

```
src/shared_control/
├── package.xml                          # 패키지 메타데이터 및 의존성
├── setup.py                             # ament_python 빌드 설정
├── setup.cfg
├── resource/shared_control              # ament 인덱스 마커
├── README.md                            # 이 파일
│
├── shared_control/                      # Python 모듈
│   ├── __init__.py
│   ├── tof_bridge_node.py               # ToF 그리드 → 단일 거리값
│   ├── obstacle_detector_node.py        # 장애물 감지 (ToF+LiDAR 융합)
│   ├── gap_analyzer_node.py             # 전방 갭 분석
│   ├── avoidance_node.py                # 회피 명령 생성
│   └── arbitrator_node.py              # 상태머신 + 명령 중재
│
├── launch/
│   └── bringup.launch.py               # 전체 스택 한 번에 기동
│
└── config/
    ├── shared_control_params.yaml       # 모든 노드 파라미터 ← 여기서 튜닝
    └── teleop_twist_joy.yaml            # 컨트롤러 축/버튼 매핑
```

---

## 6. 노드별 동작 설명

### 6.1 tof_bridge_node

**역할**: VL53L8CX가 보내는 8×8 = 64개 셀 격자에서 전방 중앙 영역만 추출해  
단일 거리값(`/tof/front_distance`)으로 변환합니다.

```
/tof/distances (64개 float, mm)
/tof/status    (64개 uint8)
        │
        ▼ ROI: rows 2~5, cols 2~5 (중앙 4×4 = 16셀)
        │
        ├─ status == 255 인 셀 제외 (완전 무효 측정)
        ├─ NaN 또는 0 이하인 셀 제외
        └─ 남은 셀 중 최솟값 → m 단위로 변환
        │
        ▼
/tof/front_distance (Float32, m)
```

센서 데이터가 0.5초 이상 오지 않으면 `inf` 를 발행합니다.

---

### 6.2 obstacle_detector_node

**역할**: ToF 거리와 LiDAR 전방 섹터 최소거리를 융합하여  
장애물 감지 여부를 판단합니다. **히스테리시스 로직을 소유**합니다.

```
/tof/front_distance ──┐
                      ├─► fused = min(tof, lidar_front_min)
/scan (±30° 최솟값) ──┘
        │
        ├─ fused < 0.5 m  → latched = True  → /obstacle/detected = True
        └─ fused > 0.8 m  → latched = False → /obstacle/detected = False
```

LiDAR 전방 섹터는 기본값 **±30°** 입니다.  
ToF 단독으로도, LiDAR 단독으로도 감지되면 장애물로 판단합니다.

---

### 6.3 gap_analyzer_node

**역할**: LiDAR 전방 ±30° 범위에서 로봇이 통과할 수 있는 갭의 폭을 계산합니다.

```
알고리즘:
1. /scan 에서 ±30° 범위의 빔만 추출
2. 각 빔이 robot_width + margin (= 0.381 m) 보다 먼지 판단
3. "먼" 빔의 가장 긴 연속 구간을 찾음
4. 그 구간의 호(arc) 폭을 추정:
   width = 2 × r_min × sin(구간_각도 / 2)
5. width >= 0.381 m 이면 /gap/passable = True
```

**로봇 폭 계산**:
- TurtleBot3 Waffle 폭: 281 mm
- 안전 마진: 100 mm
- 필요 갭: **381 mm**

---

### 6.4 avoidance_node

**역할**: 장애물이 감지됐을 때 `/cmd_vel_auto`를 생성합니다.

```
obstacle_detected == False
  → Twist(0,0) 발행 (아이들 상태)

obstacle_detected == True
  ├─ gap_passable == True
  │     → 직진 (v_forward = 0.08 m/s)
  │
  ├─ gap_passable == False
  │   ├─ 좌우 자유공간 비교
  │   │   (LiDAR ±60° 범위에서 왼쪽/오른쪽 거리 합산)
  │   ├─ 전방이 너무 가까우면 (< 0.25 m) → 제자리 회전
  │   └─ 아니면 저속(0.08 × 0.3 m/s) + 회전 (0.4 rad/s)
  │
  └─ 양쪽 모두 막힘 (좌우 합계 < 0.5)
        → 후진 (back_v = -0.05 m/s)
```

**좌우 판단 방법**: ROS 좌표계에서 양수 각도 = 왼쪽.  
좌측/우측 각각 거리 합산 → 더 큰 쪽(더 열린 쪽)으로 회전.

---

### 6.5 arbitrator_node

**역할**: 상태 머신을 구동하며, 상태에 따라 수동/자동 명령 중 하나를  
`/cmd_vel` (TwistStamped) 로 발행합니다.

```
MANUAL    → /cmd_vel_manual 의 최신값을 /cmd_vel 로 전달
             (조이스틱 무입력 시 0.3초 후 zero Twist)

AVOIDANCE → /cmd_vel_auto 의 최신값을 /cmd_vel 로 전달

STOPPED   → zero TwistStamped 발행
```

`/robot/state` 는 상태 변경 시 즉시 + **2 Hz 하트비트**로 항상 발행됩니다.

> **왜 TwistStamped인가?**  
> ROS2 Jazzy 기준 TurtleBot3 패키지의 `turtlebot3_node` 가  
> `geometry_msgs/TwistStamped` 를 구독합니다.  
> plain `Twist` 를 보내면 타입 불일치로 모터가 반응하지 않습니다.

---

## 7. 파라미터 설정 방법

모든 파라미터는 **하나의 파일**에서 관리합니다:

```
config/shared_control_params.yaml
```

빌드 없이 파일만 수정 후 재실행하면 즉시 반영됩니다.  
(단, `colcon build` 는 한 번 더 실행해야 install 폴더에 복사됩니다.)

### 전체 파라미터 표

#### tof_bridge (ToF 브리지)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `roi_rows` | `[2, 5]` | 사용할 행 범위 (0~7 중, 2행~5행) |
| `roi_cols` | `[2, 5]` | 사용할 열 범위 (0~7 중, 2열~5열) |
| `valid_status` | `[5, 9]` | *현재 미사용* (255만 제외) |
| `stale_timeout` | `0.5` (초) | 이 시간 이상 데이터 없으면 inf 발행 |
| `publish_rate` | `20.0` (Hz) | 발행 주기 |

> `roi_rows/cols` 를 `[0, 7]` 로 바꾸면 전체 8×8 영역을 사용합니다.  
> 센서 장착 위치에 따라 실제 전방에 해당하는 행/열을 맞춰야 합니다.

#### obstacle_detector (장애물 감지)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `enter_threshold` | `0.5` (m) | 이 거리 이하면 AVOIDANCE 진입 |
| `exit_threshold` | `0.8` (m) | 이 거리 이상이면 MANUAL 복귀 |
| `sector_deg` | `30.0` (°) | LiDAR 검사 범위 (±이 값) |
| `lidar_min_range` | `0.12` (m) | LiDAR 유효 최소 거리 |
| `rate` | `20.0` (Hz) | 감지 주기 |

> `enter_threshold` 를 낮추면 더 가까이 와야 회피 시작.  
> `exit_threshold` 와의 차이(0.3 m)가 히스테리시스 폭입니다.  
> 너무 좁히면 경계에서 상태가 진동합니다.

#### gap_analyzer (갭 분석)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `sector_deg` | `30.0` (°) | 분석할 LiDAR 전방 섹터 (±이 값) |
| `robot_width` | `0.281` (m) | 로봇 폭 (TurtleBot3 Waffle 실측) |
| `margin` | `0.10` (m) | 안전 여유 공간 |
| `required_gap` | `0.381` (m) | 통과 판정 최소 폭 (= 폭 + 마진) |
| `rate` | `10.0` (Hz) | 분석 주기 |

> `required_gap` 은 `robot_width + margin` 과 일치시켜야 합니다.  
> 좁은 공간에서 자주 막힌다면 `margin` 을 줄여 보세요.

#### avoidance (회피)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `v_forward` | `0.08` (m/s) | 갭 통과 시 전진 속도 |
| `w_turn` | `0.4` (rad/s) | 회전 각속도 |
| `back_v` | `-0.05` (m/s) | 양쪽 막힘 시 후진 속도 |
| `side_window_deg` | `60.0` (°) | 좌우 비교 범위 (±이 값) |
| `blocked_threshold` | `0.25` (m) | 이 거리 이하면 제자리 회전 |
| `rate` | `20.0` (Hz) | 명령 발행 주기 |

> 회피가 너무 급하면 `w_turn` 을 낮추세요 (예: 0.3).  
> 좁은 복도에서 전진이 너무 빠르면 `v_forward` 를 줄이세요.

#### arbitrator (중재기)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `cmd_timeout` | `0.3` (초) | 이 시간 이상 명령 미수신 시 stale 처리 |
| `rate` | `20.0` (Hz) | 상태머신 실행 주기 |

> `cmd_timeout` 이 너무 짧으면 일시적 지연으로 STOPPED 가 될 수 있습니다.

### 파라미터 수정 예시

```bash
# 파일 열어서 수정
nano ~/ros2_ws/src/shared_control/config/shared_control_params.yaml

# 수정 후 반드시 재빌드 (install 폴더에 복사됨)
cd ~/ros2_ws
colcon build --packages-select shared_control

# 재실행
source install/setup.bash
ros2 launch shared_control bringup.launch.py
```

---

## 8. 컨트롤러 버튼 설정

컨트롤러 설정 파일:

```
config/teleop_twist_joy.yaml
```

현재 설정은 **8BitDo Micro** 기준입니다.

### 현재 버튼/축 매핑

| 항목 | 값 | 설명 |
|---|---|---|
| `axis_linear.x` | `1` | D-pad 상하 = 전진/후진 |
| `axis_angular.yaw` | `0` | D-pad 좌우 = 회전 |
| `scale_linear.x` | `0.20` | 일반 전진 최대속도 (m/s) |
| `scale_linear_turbo.x` | `0.22` | 터보 전진 최대속도 (m/s) |
| `scale_angular.yaw` | `1.0` | 일반 회전 최대속도 (rad/s) |
| `scale_angular_turbo.yaw` | `1.5` | 터보 회전 최대속도 (rad/s) |
| `require_enable_button` | `false` | 활성화 버튼 없이 바로 이동 |
| `enable_turbo_button` | `7` | R 버튼 = 터보 모드 |

### 다른 컨트롤러 사용 시

`/joy` 토픽을 `ros2 topic echo /joy` 로 확인하면서 버튼을 눌러  
실제 인덱스를 확인하고 yaml 파일을 수정하세요.

```bash
# 컨트롤러 축/버튼 인덱스 확인
ros2 topic echo /joy
# axes: [0.0, 0.0, ...] 순서로 D-pad/스틱이 바뀌는 위치가 인덱스
# buttons: [0, 0, 1, ...] 순서로 눌린 버튼이 1 이 되는 위치가 인덱스
```

---

## 9. 빌드 방법

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select shared_control
source install/setup.bash
```

> `config/` 파일을 수정했을 때도 반드시 재빌드해야  
> `install/` 폴더에 복사됩니다.

---

## 10. 실행 방법

### 터미널 1 — TurtleBot3 Bringup (로봇 기본 구동)

```bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export LDS_MODEL=LDS-02
ros2 launch turtlebot3_bringup robot.launch.py
```

이 터미널이 `/scan` (LiDAR) 과 `/cmd_vel` 수신(모터)을 담당합니다.

### 터미널 2 — shared_control 전체 스택

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch shared_control bringup.launch.py
```

아래 노드들이 한꺼번에 기동됩니다:

| 노드 | 패키지 |
|---|---|
| `joy_node` | `joy` |
| `tof_publisher` | `tof_publisher` |
| `teleop_twist_joy_node` | `teleop_twist_joy` |
| `tof_bridge` | `shared_control` |
| `obstacle_detector` | `shared_control` |
| `gap_analyzer` | `shared_control` |
| `avoidance` | `shared_control` |
| `arbitrator` | `shared_control` |

> **주의**: `assisted_teleop` 은 `/cmd_vel` 을 공유하므로  
> 함께 실행하면 충돌이 발생합니다. 반드시 종료 후 실행하세요.

### ToF 시리얼 포트 확인

기본 포트는 `/dev/tof_sensor` 입니다. 다를 경우:

```bash
ls /dev/tof*          # 실제 장치명 확인
ls /dev/ttyUSB*       # 또는 ttyUSB 로 잡히는 경우
```

`launch/bringup.launch.py` 에서 `port` 파라미터를 수정하거나:

```bash
ros2 launch shared_control bringup.launch.py   # 기본 실행
```

---

## 11. VMware 원격 모니터링

VMware의 Ubuntu VM에서 로봇 상태를 실시간으로 확인할 수 있습니다.

### 네트워크 설정 (필수)

로봇(라즈베리파이)과 VMware VM이 **같은 네트워크 서브넷**에 있어야 합니다.  
ROS_DOMAIN_ID 도 같아야 합니다 (기본값: 0).

```bash
# VMware VM 에서 실행
export ROS_DOMAIN_ID=0                    # 로봇과 동일한 값
source /opt/ros/jazzy/setup.bash
```

### 모니터링 토픽

```bash
# 현재 상태 확인 (MANUAL / AVOIDANCE / STOPPED)
ros2 topic echo /robot/state

# 전방 ToF 거리 (m)
ros2 topic echo /tof/front_distance

# 장애물 감지 여부
ros2 topic echo /obstacle/detected

# 융합 장애물 거리
ros2 topic echo /obstacle/distance

# 갭 폭 및 통과 가능 여부
ros2 topic echo /gap/width
ros2 topic echo /gap/passable

# 발행 주파수 확인
ros2 topic hz /cmd_vel
ros2 topic hz /robot/state
```

### rqt 시각화

```bash
# 여러 토픽 동시 그래프
rqt_plot /tof/front_distance/data /obstacle/distance/data /gap/width/data

# 전체 토픽 브라우저
rqt
```

### LiDAR 시각화 (RViz2)

```bash
rviz2
# Add > LaserScan > Topic: /scan
# Fixed Frame: base_link
```

---

## 12. 트러블슈팅

### `/tof/front_distance` 가 inf 로만 나올 때

```bash
# tof_publisher 가 실제 데이터를 보내는지 확인
ros2 topic echo /tof/distances
ros2 topic echo /tof/status
```

- `/tof/distances` 에 유효값이 없으면 → 시리얼 포트 확인 (`/dev/tof_sensor`)
- 모든 status 가 `255` → 센서와 물체 사이 거리가 너무 멀거나 측정 불가 상태
- `roi_rows/cols` 범위가 실제 전방 방향과 맞지 않을 수 있음 → 값 조정

### DXL(모터)이 안 움직일 때

```bash
# 중재기가 /cmd_vel 을 발행하는지 확인
ros2 topic echo /cmd_vel

# TurtleBot3 bringup 이 켜져 있는지 확인
ros2 node list | grep turtlebot3
```

- `/cmd_vel` 이 비어 있다면 → `arbitrator` 노드가 실행 중인지 확인
- TurtleBot3 bringup 이 없으면 → 터미널 1 확인

### `/obstacle/detected` 가 항상 false 일 때

```bash
ros2 topic echo /tof/front_distance   # inf 이면 ToF 문제
ros2 topic echo /obstacle/distance    # 실제 거리 확인
```

- ToF 와 LiDAR 모두 감지 거리 밖이면 정상적으로 false
- 전방 0.5 m 이내에 물체를 놓고 다시 확인

### 상태가 MANUAL ↔ AVOIDANCE 를 빠르게 왔다 갔다 할 때

`shared_control_params.yaml` 에서 히스테리시스 폭을 키우세요:

```yaml
obstacle_detector:
  ros__parameters:
    enter_threshold: 0.4   # 낮춤 (더 가까워야 진입)
    exit_threshold: 0.9    # 높임 (더 멀어야 탈출)
```

### 컨트롤러가 반응하지 않을 때

```bash
ros2 topic echo /joy   # 컨트롤러 입력 확인
```

- `/joy` 에 변화가 없으면 → 블루투스 재페어링 또는 `joy_node` 재시작
- `require_enable_button: true` 이면 활성화 버튼을 누른 채로 조종해야 함
