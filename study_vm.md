# ROS2 모니터링 시스템 공부 가이드

> TurtleBot3 Waffle + Raspberry Pi 5 + VMware 모니터링 PC 환경 기준

---

## 목차

1. [전체 시스템 구조](#1-전체-시스템-구조)
2. [ROS2 기초 개념](#2-ros2-기초-개념)
3. [토픽(Topic) 이해하기](#3-토픽topic-이해하기)
4. [우리 프로젝트의 토픽들](#4-우리-프로젝트의-토픽들)
5. [DDS 통신 이해하기](#5-dds-통신-이해하기)
6. [VMware 네트워크 설정](#6-vmware-네트워크-설정)
7. [시각화 도구들](#7-시각화-도구들)
8. [SLAM이란?](#8-slam이란)
9. [Navigation2란?](#9-navigation2란)
10. [Launch 파일 이해하기](#10-launch-파일-이해하기)
11. [QoS 이해하기](#11-qos-이해하기)
12. [마커(Marker) 이해하기](#12-마커marker-이해하기)
13. [TF(Transform) 이해하기](#13-tftransform-이해하기)
14. [ToF 8x8 센서와 거리값 이해하기](#14-tof-8x8-센서와-거리값-이해하기)
15. [유용한 명령어 모음](#15-유용한-명령어-모음)
16. [자주 만나는 문제와 해결](#16-자주-만나는-문제와-해결)

---

## 1. 전체 시스템 구조

```
┌─────────────────────────┐         WiFi/유선          ┌─────────────────────────┐
│   Raspberry Pi 5        │ ◄══════════════════════► │   VMware (Ubuntu 24.04)  │
│   (로봇에 탑재)           │        DDS 통신            │   (모니터링 PC)           │
│                         │                           │                         │
│  ┌───────────────────┐  │                           │  ┌───────────────────┐  │
│  │ LiDAR 센서        │  │  /scan ───────────────►  │  │ RViz2             │  │
│  │ ToF 센서          │  │  /tof/front_distance ─►  │  │  - LiDAR 시각화    │  │
│  │ 블루투스 컨트롤러    │  │  /cmd_vel ──────────►   │  │  - 마커 시각화      │  │
│  │ 장애물 감지 노드    │  │  /obstacle/* ────────►  │  │                   │  │
│  │ 갭 감지 노드       │  │  /gap/* ─────────────►  │  │ rqt_plot          │  │
│  │ 상태 관리 노드     │  │  /robot/state ───────►  │  │  - 거리 그래프      │  │
│  └───────────────────┘  │                           │  │                   │  │
│                         │                           │  │ monitor_node      │  │
│                         │                           │  │  - 터미널 출력      │  │
│                         │                           │  │  - 마커 발행       │  │
│                         │                           │  └───────────────────┘  │
└─────────────────────────┘                           └─────────────────────────┘
```

### 핵심 포인트

- **Pi5**는 센서 데이터를 수집하고, 로봇을 제어함 (퍼블리셔)
- **VMware PC**는 그 데이터를 받아서 화면에 보여줌 (서브스크라이버)
- 둘 사이는 **DDS**라는 프로토콜로 자동 연결됨 (같은 네트워크 + 같은 DOMAIN_ID)

---

## 2. ROS2 기초 개념

### ROS2가 뭔가?

ROS2(Robot Operating System 2)는 **로봇 소프트웨어 프레임워크**이다.
운영체제가 아니라, 로봇 프로그램들이 서로 **데이터를 주고받는 규칙**을 정해놓은 것이다.

### 핵심 3가지

```
┌──────────┐    토픽(Topic)     ┌──────────┐
│          │ ─── 메시지 ────►  │          │
│  노드 A   │                   │  노드 B   │
│ (Publisher)                  │(Subscriber)
└──────────┘                   └──────────┘
```

| 개념 | 비유 | 설명 |
|------|------|------|
| **노드(Node)** | 사람 | 하나의 프로그램. 각자 맡은 일을 함 |
| **토픽(Topic)** | 게시판 | 데이터를 올리는 장소. 이름이 있음 (예: `/scan`) |
| **메시지(Message)** | 편지 | 토픽에 올리는 데이터의 형식 (예: `Float32`) |

### 노드(Node)란?

```
예시: 우리 시스템의 노드들

[LiDAR 드라이버 노드]  → /scan 토픽에 레이저 데이터 발행
[ToF 센서 노드]        → /tof/front_distance 토픽에 거리 발행
[장애물 감지 노드]      → /obstacle/detected 토픽에 결과 발행
[모니터 노드]          → 위 토픽들을 전부 구독해서 화면에 표시
```

- 각 노드는 **독립적인 프로그램**이다
- 하나가 죽어도 다른 노드는 계속 동작한다
- `ros2 node list` 명령으로 실행 중인 노드를 볼 수 있다

### Publisher와 Subscriber

```
Publisher(발행자)                  Subscriber(구독자)
─────────────                    ──────────────
"나 데이터 있어, 올릴게"           "그 데이터 필요해, 볼게"

예:                               예:
ToF 센서 노드가                    모니터 노드가
/tof/front_distance에             /tof/front_distance를
거리값을 계속 올림                  계속 읽어서 화면에 표시
```

---

## 3. 토픽(Topic) 이해하기

### 토픽은 이름이 있는 데이터 통로

```
토픽 이름: /tof/front_distance
메시지 타입: std_msgs/msg/Float32
데이터 예시: 1.877 (미터)

    [ToF 노드] ──── 1.877 ────► /tof/front_distance ────► [모니터 노드]
                                                    ────► [다른 노드도 구독 가능]
```

### 메시지 타입 정리

| 타입 | 설명 | 데이터 예시 |
|------|------|-------------|
| `std_msgs/msg/Float32` | 소수점 숫자 1개 | `data: 1.877` |
| `std_msgs/msg/Bool` | 참/거짓 | `data: true` |
| `std_msgs/msg/String` | 문자열 | `data: "MANUAL"` |
| `sensor_msgs/msg/LaserScan` | LiDAR 데이터 (거리 배열) | `ranges: [0.5, 0.7, ...]` |
| `geometry_msgs/msg/TwistStamped` | 속도 명령 + 시간 | `twist.linear.x: 0.2` |

### 토픽 확인하는 법

```bash
# 현재 활성 토픽 목록
ros2 topic list

# 특정 토픽의 데이터 1개 보기
ros2 topic echo /tof/front_distance --once

# 토픽의 메시지 타입 확인
ros2 topic info /cmd_vel --verbose

# 토픽이 초당 몇 번 오는지 확인
ros2 topic hz /scan
```

---

## 4. 우리 프로젝트의 토픽들

### 전체 토픽 흐름도

```
[블루투스 컨트롤러]
        │
        ▼
   /joy (조이스틱 raw 데이터)
        │
        ▼
┌───────────────┐     /cmd_vel_manual
│  텔레옵 노드   │ ──────────────────┐
└───────────────┘                    │
                                     ▼
                               ┌──────────┐     /cmd_vel        ┌──────────┐
                               │ 중재 노드  │ ──────────────────► │ 모터 구동  │
                               └──────────┘                     └──────────┘
                                     ▲
┌───────────────┐     /cmd_vel_auto  │
│  회피 노드     │ ──────────────────┘
└───────────────┘
        ▲
        │ 참조
        │
   /obstacle/detected
   /obstacle/distance
   /gap/width
   /gap/passable
        ▲
        │
┌───────────────┐          ┌───────────────┐
│  장애물 감지    │ ◄─────── │   LiDAR       │ ──► /scan
│  노드          │          │   센서 노드    │
└───────────────┘          └───────────────┘
        ▲
        │
┌───────────────┐
│  ToF 센서 노드  │ ──► /tof/front_distance
└───────────────┘

┌───────────────┐
│  상태 관리 노드  │ ──► /robot/state (MANUAL / AVOIDANCE / STOPPED)
└───────────────┘
```

### 각 토픽 상세 설명

#### `/tof/front_distance` (Float32)

```
무엇: 로봇 정면의 ToF(Time of Flight) 센서 거리값
단위: 미터(m)
예시: 1.877 → 정면 1.877m 앞에 물체가 있음
용도: 정면 장애물까지의 정밀 거리 측정
센서: VL53L8CX (8x8 존 센서, 그 중 정면 값만 추출)
```

#### `/obstacle/detected` (Bool)

```
무엇: 장애물이 감지되었는지 여부
예시: true → 장애물 있음, false → 장애물 없음
용도: 장애물 회피 모드 진입 여부 판단
```

#### `/obstacle/distance` (Float32)

```
무엇: 감지된 장애물까지의 거리
단위: 미터(m)
예시: 0.35 → 0.35m 앞에 장애물
용도: 얼마나 가까운지 판단하여 회피 강도 결정
```

#### `/gap/width` (Float32)

```
무엇: 장애물 사이 통과 가능한 틈의 너비
단위: 미터(m)
예시: 0.45 → 45cm 틈
용도: 로봇이 그 틈을 통과할 수 있는지 판단
      (TurtleBot3 Waffle 폭 ≈ 30cm)
```

#### `/gap/passable` (Bool)

```
무엇: 틈이 로봇이 통과할 수 있을 만큼 넓은지
예시: true → 통과 가능, false → 통과 불가
용도: 회피 경로 선택
```

#### `/robot/state` (String)

```
무엇: 현재 로봇의 동작 모드
값:
  "MANUAL"    → 사용자가 컨트롤러로 직접 조종 중
  "AVOIDANCE" → 장애물 감지 → 자동 회피 중
  "STOPPED"   → 정지 상태
```

#### `/scan` (LaserScan)

```
무엇: LiDAR(라이다) 센서의 360도 거리 데이터
내용:
  - ranges: [0.5, 0.7, 1.2, ...] → 각 방향의 거리(m) 배열
  - angle_min ~ angle_max: 스캔 각도 범위
  - angle_increment: 각 측정점 사이 각도
용도: 주변 환경 인식, 장애물 감지, SLAM
```

#### `/cmd_vel` (TwistStamped)

```
무엇: 로봇에게 보내는 속도 명령
내용:
  - twist.linear.x  → 직진 속도 (m/s). 양수=전진, 음수=후진
  - twist.angular.z  → 회전 속도 (rad/s). 양수=반시계, 음수=시계
예시:
  linear.x = 0.2, angular.z = 0.0  → 0.2m/s로 직진
  linear.x = 0.0, angular.z = 0.5  → 제자리 반시계 회전
  linear.x = 0.1, angular.z = -0.3 → 전진하면서 우회전
```

---

## 5. DDS 통신 이해하기

### DDS가 뭔가?

```
DDS = Data Distribution Service (데이터 분배 서비스)

ROS1: 중앙 마스터(roscore)가 필요했음
       노드A ──► roscore ──► 노드B
       roscore 죽으면 전부 멈춤

ROS2: DDS를 사용하여 마스터 없이 통신
       노드A ◄────────────► 노드B
       서로 자동으로 발견하고 직접 통신
```

### DDS 작동 원리

```
1단계: 디스커버리 (서로 찾기)
───────────────────────────
Pi5의 노드: "나 /scan 토픽 발행하고 있어!" (멀티캐스트)
VMware 노드: "나 /scan 토픽 구독하고 싶어!" (멀티캐스트)
→ 두 노드가 같은 네트워크 + 같은 DOMAIN_ID이면 자동 매칭

2단계: 데이터 전송 (유니캐스트)
───────────────────────────
Pi5 ────── /scan 데이터 ──────► VMware 모니터 노드
(매칭 후에는 1:1 직접 전송)
```

### ROS_DOMAIN_ID

```
같은 네트워크에 여러 로봇이 있을 때 구분하는 번호

예:
  로봇1팀: export ROS_DOMAIN_ID=1   → ID 1인 노드끼리만 통신
  로봇2팀: export ROS_DOMAIN_ID=2   → ID 2인 노드끼리만 통신

우리 설정:
  Pi5:    export ROS_DOMAIN_ID=0  (기본값)
  VMware: export ROS_DOMAIN_ID=0  (기본값)
  → 같은 숫자여야 서로 보임!
```

### CycloneDDS

```
DDS는 규격(표준)이고, 실제 구현체가 여러 개 있음

┌─────────────────────────────────────────────┐
│ DDS 구현체         │ 설명                     │
├─────────────────────────────────────────────┤
│ CycloneDDS         │ ROS2 Jazzy 기본. 가볍고 빠름 │
│ FastDDS            │ 이전 버전 기본. 무거움       │
│ Connext DDS        │ 상용 제품                  │
└─────────────────────────────────────────────┘

중요: Pi5와 VMware가 같은 DDS 구현체를 써야 안정적!
→ 둘 다 CycloneDDS 사용 권장
→ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## 6. VMware 네트워크 설정

### VMware 네트워크 모드 3가지

```
┌────────────────────────────────────────────────────────────────┐
│                                                                │
│  1. NAT 모드 (기본값) ← DDS 안 됨!                              │
│  ──────────────────────────────                                │
│  호스트 PC 뒤에 숨어서 인터넷 사용                                │
│  VM이 별도 IP를 가지지만, 외부에서 VM에 직접 접근 불가              │
│                                                                │
│    [Pi5: 192.168.0.100] ──X──► [VM: 192.168.xx.yy (NAT)]     │
│    서로 다른 네트워크 → DDS 디스커버리 실패                        │
│                                                                │
│  2. Bridged 모드 ← DDS 가능! 이것을 사용                        │
│  ──────────────────────────                                    │
│  VM이 실제 네트워크에 직접 연결된 것처럼 동작                       │
│  Pi5와 같은 네트워크 대역의 IP를 받음                              │
│                                                                │
│    [Pi5: 192.168.0.100] ◄───► [VM: 192.168.0.57 (Bridged)]   │
│    같은 네트워크 → DDS 디스커버리 성공!                            │
│                                                                │
│  3. Host-Only 모드                                              │
│  ──────────────────                                            │
│  VM과 호스트 PC끼리만 통신. 외부 네트워크 접근 불가                  │
│  Pi5와는 통신 불가                                               │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

### Bridged 모드 설정 방법

```
VMware 메뉴:
1. VM → Settings → Network Adapter
2. "Bridged: Connected directly to the physical network" 선택
3. "Replicate physical network connection state" 체크
4. OK

확인:
$ ip addr show ens33
  inet 192.168.0.57/24   ← Pi5와 같은 192.168.0.x 대역이면 성공
```

### 멀티캐스트 확인

```bash
# DDS는 멀티캐스트로 서로를 찾음
# VMware Bridged 모드에서 멀티캐스트가 되는지 확인:

ros2 multicast receive &     # 터미널1: 수신 대기
ros2 multicast send           # 터미널2: 전송

# "Received from xxx" 메시지가 나오면 성공
```

---

## 7. 시각화 도구들

### RViz2

```
RViz2 = ROS Visualization 2

무엇: ROS2 데이터를 3D 공간에 시각화하는 도구
비유: 로봇이 보는 세상을 우리 눈으로 볼 수 있게 해주는 창

우리가 RViz2에서 보는 것:
┌─────────────────────────────────────────┐
│                                         │
│    초록 점들 = LiDAR 스캔 (/scan)        │
│          ·  · ·                         │
│        ·        ·                       │
│       ·    ●     ·    ← 빨간 구 = 장애물 │
│        ·  [MANUAL] ·  ← 상태 텍스트      │
│          ·  · ·                         │
│           로봇                           │
│                                         │
└─────────────────────────────────────────┘

표시 항목:
  - Grid: 바닥 격자선 (기준점 파악용)
  - LaserScan: /scan 토픽의 LiDAR 포인트 (초록 점)
  - MarkerArray: 우리 monitor_node가 발행하는 마커들
    - 빨간 구: 장애물 위치
    - 텍스트: 장애물 거리, 갭 폭, 로봇 상태
    - 초록 실린더: 통과 가능한 갭
  - TF: 좌표계 화살표 (로봇의 방향)
  - RobotModel: 로봇 3D 모델
```

### rqt_plot

```
rqt_plot = 실시간 그래프 도구

무엇: 토픽의 숫자값을 시간에 따라 그래프로 그려줌

우리가 rqt_plot에서 보는 것:
┌─────────────────────────────────────────┐
│  거리(m)                                 │
│  2.0 ┤                                  │
│      │ ~~~~/tof/front_distance           │
│  1.5 ┤     ~~~~                          │
│      │         ~~~~                      │
│  1.0 ┤ ──── /obstacle/distance           │
│      │  ────────                         │
│  0.5 ┤          ────                     │
│      │ .... /gap/width                   │
│  0.0 ┤............................        │
│      └───────────────────── 시간 ──►     │
└─────────────────────────────────────────┘

용도: 센서값 변화 추이를 한눈에 파악
  - 장애물에 다가가면 거리값이 줄어드는 것을 실시간으로 봄
  - 갭이 생기면 gap/width가 올라가는 것을 봄
```

### rqt

```
rqt = ROS2 Qt 기반 GUI 도구 모음

포함된 도구들:
  - rqt_plot: 숫자 그래프
  - rqt_graph: 노드/토픽 연결 관계 시각화
  - rqt_console: 로그 메시지 보기
  - rqt_topic: 토픽 목록 및 데이터 보기
  - rqt_image_view: 카메라 이미지 보기

실행: rqt 명령어 → 메뉴에서 플러그인 선택
```

---

## 8. SLAM이란?

### 개념

```
SLAM = Simultaneous Localization And Mapping
      (동시적 위치추정 및 지도작성)

쉽게: "내가 어디 있는지 모르는 상태에서,
       돌아다니면서 지도를 만들고,
       동시에 지도 위에서 내 위치를 파악하는 것"

비유: 눈을 감고 새 건물에 들어가서
      손으로 벽을 더듬으며 머릿속으로 지도를 그리는 것
```

### SLAM이 필요한 데이터

```
SLAM 입력:
  1. /scan (LiDAR) → 주변에 벽이나 물체가 어디에 있는지
  2. /odom (주행거리) → 로봇이 얼마나 이동했는지
  3. /tf (좌표 변환) → 센서와 로봇의 위치 관계

SLAM 출력:
  1. /map → 2D 점유 격자 지도 (Occupancy Grid Map)
  2. /tf (map → odom) → 지도 위에서 로봇의 정확한 위치
```

### SLAM 결과물

```
점유 격자 지도 (Occupancy Grid Map):

  ■ = 벽/장애물 (occupied)
  □ = 빈 공간 (free)
  ? = 아직 모르는 영역 (unknown)

  ■■■■■■■■■■■■■■
  ■□□□□□□□□□□□□■
  ■□□□□□□□□□□□□■
  ■□□□■■■□□□□□□■
  ■□□□□□□□□□□□□■
  ■□□□□□□□■■□□□■
  ■□□□□□□□□□□□□■
  ■■■■■■■■■■■■■■
```

### 우리 시스템에 SLAM이 없는 이유

```
현재 우리 시스템:
  ✅ /scan 데이터 있음
  ✅ /odom 데이터 있음
  ❌ SLAM 노드 실행 안 함

왜?
  → 현재 목적은 "모니터링"이지 "지도 생성"이 아님
  → 장애물 감지/회피는 SLAM 없이도 가능 (LiDAR 직접 사용)
  → SLAM은 자율 주행(Navigation)을 할 때 필요

추가하고 싶다면:
  sudo apt install ros-jazzy-slam-toolbox
  ros2 launch slam_toolbox online_async_launch.py
  → RViz2에서 지도가 실시간으로 그려지는 것을 볼 수 있음
```

---

## 9. Navigation2란?

### 개념

```
Navigation2 (Nav2) = ROS2 자율 주행 스택

SLAM으로 지도를 만든 후,
"이 지도 위에서 A 지점에서 B 지점으로 가라" 명령을 수행

단계:
  1. SLAM으로 지도 생성 → map.yaml 저장
  2. Nav2 실행 → 지도 로드
  3. RViz2에서 목표 지점 클릭
  4. Nav2가 경로 계획 → 장애물 회피하며 이동
```

### 우리 시스템과의 관계

```
현재 우리 시스템:                    Nav2 시스템:
──────────────                     ──────────
블루투스 컨트롤러 → 수동 조종         목표 지점 설정 → 자동 이동
장애물 감지 → 단순 회피               전역 경로 계획 + 동적 장애물 회피
지도 없음                            지도 기반 경로 탐색

우리 시스템은 "원격 조종 + 단순 회피" 방식
Nav2는 "완전 자율 주행" 방식
→ 서로 다른 목적!
```

---

## 10. Launch 파일 이해하기

### Launch 파일이 뭔가?

```
여러 노드를 한 번에 실행하는 스크립트

Launch 없이:
  터미널1: ros2 run 패키지1 노드1
  터미널2: ros2 run 패키지2 노드2
  터미널3: ros2 run 패키지3 노드3
  → 터미널 3개 열어야 함

Launch 사용:
  터미널1: ros2 launch 패키지 launch파일.py
  → 한 번에 전부 실행!
```

### 우리 monitor.launch.py 분석

```python
# 이 Launch 파일이 실행하는 것 3가지:

return LaunchDescription([

    # 1. monitor_node: 터미널 출력 + 마커 발행
    Node(
        package='tb3_monitor',        # 패키지 이름
        executable='monitor_node',    # 실행 파일 이름
        name='tb3_monitor',           # 노드 이름
        output='screen',              # 출력을 화면에 표시
        emulate_tty=True,             # 색상 출력 지원
    ),

    # 2. RViz2: 3D 시각화
    Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],  # 설정 파일 적용
    ),

    # 3. rqt_plot: 그래프
    ExecuteProcess(
        cmd=['rqt_plot', '/tof/front_distance/data', ...],
    ),
])
```

---

## 11. QoS 이해하기

### QoS가 뭔가?

```
QoS = Quality of Service (서비스 품질)

토픽 통신할 때 "얼마나 신뢰성 있게 보낼 것인가" 설정

비유:
  일반우편(Best Effort) vs 등기우편(Reliable)
```

### 주요 QoS 설정

```
┌──────────────────────────────────────────────────────────┐
│ 설정            │ 옵션              │ 설명               │
├──────────────────────────────────────────────────────────┤
│ Reliability     │ RELIABLE          │ 데이터 반드시 도착   │
│ (신뢰성)        │                   │ (느릴 수 있음)      │
│                 │ BEST_EFFORT       │ 빠르게, 유실 허용   │
├──────────────────────────────────────────────────────────┤
│ Durability      │ TRANSIENT_LOCAL   │ 늦게 구독해도       │
│ (지속성)        │                   │ 마지막 값 받음      │
│                 │ VOLATILE          │ 구독 시점부터만 받음  │
└──────────────────────────────────────────────────────────┘
```

### 우리 코드에서의 QoS

```python
# 일반 토픽 (상태, 거리 등) → Reliable
qos_default = QoSProfile(depth=10)

# 센서 토픽 (/scan 등) → Best Effort (LiDAR는 빠른 전송 우선)
qos_sensor = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
```

### QoS 불일치 문제

```
흔한 에러: "토픽이 있는데 데이터가 안 와요!"

원인:
  Publisher가 BEST_EFFORT로 보냄
  Subscriber가 RELIABLE로 받으려 함
  → QoS 불일치 → 데이터 안 옴!

규칙:
  Publisher    Subscriber   결과
  ─────────   ──────────   ─────
  RELIABLE    RELIABLE     ✅ 동작
  RELIABLE    BEST_EFFORT  ✅ 동작
  BEST_EFFORT BEST_EFFORT  ✅ 동작
  BEST_EFFORT RELIABLE     ❌ 안 됨!

확인 방법:
  ros2 topic info /scan --verbose
  → QoS 정보가 나옴 → Subscriber를 맞춰줘야 함
```

---

## 12. 마커(Marker) 이해하기

### Marker가 뭔가?

```
RViz2에 원하는 도형/텍스트를 표시하는 메시지

비유: RViz2 3D 공간에 스티커를 붙이는 것

종류:
  SPHERE     → 구 (우리는 장애물 위치 표시용)
  CYLINDER   → 원기둥 (갭 표시용)
  TEXT_VIEW_FACING → 항상 카메라를 향하는 텍스트
  ARROW      → 화살표
  CUBE       → 정육면체
  LINE_STRIP → 선
```

### 우리 monitor_node의 마커들

```
┌──────────────────────────────────────────────────────┐
│  마커           │ 타입       │ 색상    │ 위치          │
├──────────────────────────────────────────────────────┤
│  장애물 구       │ SPHERE     │ 빨강    │ 전방 거리     │
│  장애물 거리 텍스트│ TEXT       │ 흰색    │ 장애물 위     │
│  갭 실린더       │ CYLINDER   │ 초록/주황│ 장애물 위치   │
│  갭 정보 텍스트   │ TEXT       │ 초록    │ 갭 위         │
│  로봇 상태 텍스트  │ TEXT       │ 상태별  │ 로봇 위      │
└──────────────────────────────────────────────────────┘

마커 코드 예시:
  m = Marker()
  m.header.frame_id = 'base_link'   ← 로봇 기준 좌표
  m.type = Marker.SPHERE            ← 구 모양
  m.pose.position.x = 1.5           ← 로봇 앞 1.5m 위치
  m.color.r = 1.0                   ← 빨간색
  m.scale.x = 0.15                  ← 크기 15cm
  m.lifetime = Duration(sec=1)      ← 1초 후 자동 삭제
```

---

## 13. TF(Transform) 이해하기

### TF가 뭔가?

```
TF = Transform (좌표 변환)

로봇에는 여러 부품이 있고, 각각의 위치가 다름
TF는 "이 부품은 저 부품에서 얼마만큼 떨어져 있다"를 알려줌

TurtleBot3 TF 트리:

  map (세계 기준점) ← SLAM이 있을 때만
   └── odom (출발점 기준)
        └── base_footprint (로봇 바닥)
             └── base_link (로봇 중심)
                  ├── base_scan (LiDAR 위치)
                  ├── imu_link (IMU 위치)
                  └── wheel_left_link, wheel_right_link
```

### 왜 중요한가?

```
LiDAR가 "정면 1m에 물체 있음"이라고 할 때,
이 "정면 1m"은 LiDAR 센서 기준임.

로봇 중심(base_link)에서 LiDAR(base_scan)가 10cm 앞에 있다면:
→ 로봇 중심 기준으로는 "정면 1.1m에 물체 있음"

TF가 이 좌표 변환을 자동으로 해줌!

RViz2에서 Fixed Frame = "base_link"로 설정하면:
→ 모든 데이터가 로봇 중심 기준으로 변환되어 표시됨
```

---

## 14. ToF 8x8 센서와 거리값 이해하기

### VL53L8CX 센서란?

```
VL53L8CX = ST마이크로 사의 ToF(Time of Flight) 센서

원리: 적외선 빛을 쏘고, 반사되어 돌아오는 시간으로 거리를 계산
특징: 한 번에 8x8 = 64개 영역을 동시에 측정 (멀티존)

비유: 눈(카메라)은 색을 보지만, ToF는 거리를 본다
      그리고 8x8 격자로 나눠서 각 칸마다 거리를 측정

        ┌──┬──┬──┬──┬──┬──┬──┬──┐
        │  │  │  │  │  │  │  │  │  ← 각 칸이 하나의 측정 존(zone)
        ├──┼──┼──┼──┼──┼──┼──┼──┤     총 64개 존
        │  │  │  │  │  │  │  │  │
        ├──┼──┼──┼──┼──┼──┼──┼──┤     각 존에서 독립적으로
        │  │  │  │  │  │  │  │  │     거리를 측정 (단위: mm)
        ... (8행)
        └──┴──┴──┴──┴──┴──┴──┴──┘

스펙:
  - 측정 범위: ~10mm ~ 4000mm (약 4m)
  - 시야각(FOV): 약 45도 x 45도
  - 출력: 8x8 거리 배열 (mm) + 각 존의 상태값
  - NaN: 측정 실패 (너무 멀거나, 반사가 안 되거나, 상태 불량)
```

### 센서에서 토픽까지 데이터 흐름

```
[VL53L8CX 하드웨어]
        │
        │ I2C/SPI 통신
        ▼
┌──────────────────┐
│  tof_publisher    │  (Pi5에서 실행)
│  노드             │
│                  │  8x8 거리값을 읽어서 ROS2 토픽으로 발행
│  발행:            │
│  /tof/distances  │ → Float32MultiArray (64개 값, mm 단위)
│  /tof/status     │ → UInt8MultiArray (64개 상태값)
└──────────────────┘
        │
        │ /tof/distances 구독
        ▼
┌──────────────────┐
│  tof_bridge       │  (Pi5에서 실행)
│  노드             │
│                  │  8x8 중에서 ROI(관심 영역)만 추출
│                  │  유효한 값만 평균 계산
│                  │  mm → m 변환
│  발행:            │
│  /tof/front_distance │ → Float32 (단일 값, m 단위)
└──────────────────┘
        │
        │ /tof/front_distance 구독
        ▼
┌──────────────────┐
│  obstacle_detector│  (Pi5에서 실행)
│  노드             │
│                  │  ToF + LiDAR를 종합 판단
│  발행:            │
│  /obstacle/distance  │ → Float32 (m)
│  /obstacle/detected  │ → Bool
└──────────────────┘
```

### front_distance가 정확히 뭔가?

```
/tof/distances (8x8 raw 데이터, mm):

         C0    C1    C2    C3    C4    C5    C6    C7
    R0 │ NaN   NaN   461   944   726   599   495   433 │
    R1 │ NaN   NaN  1153   883   694   573   488   427 │
    R2 │ NaN  1633 ╔1081   868   675╗  571   478   418 │
    R3 │ NaN  1451 ║1041   817   662║  555   476   414 │ ← ROI 영역
    R4 │ NaN  1353 ║1010   NaN   637║  548   465   409 │    (노란 테두리)
    R5 │1950  1300 ╚ 968   755   630╝  527   454   400 │
    R6 │ NaN  1227   936   737   593   520   446   391 │
    R7 │ 533   457   876   707   575   497   435   384 │

tof_bridge 파라미터:
    roi_rows = [2, 5]     → Row 2, 3, 4 사용 (인덱스 2 이상 ~ 5 미만)
    roi_cols = [2, 5]     → Col 2, 3, 4 사용 (인덱스 2 이상 ~ 5 미만)
    valid_status = [5, 9] → 상태값이 5 또는 9인 셀만 "유효"로 인정

계산 과정:
    1. 8x8에서 ROI 영역(R2~R4, C2~C4) = 3x3 = 9개 셀 추출
    2. 그 중 status가 5 또는 9인 셀만 선택 (NaN, 불량 제외)
    3. 유효한 셀들의 평균값 계산
    4. mm → m 변환 (÷ 1000)
    5. /tof/front_distance로 발행

결과: front_distance ≈ 유효셀 평균 = 0.529m

왜 ROI를 쓰나?
    → 8x8 전체가 아니라 "로봇 정면 중앙"만 관심 있기 때문
    → 가장자리 셀은 옆이나 위아래를 보고 있어서 정면 거리와 무관
```

### 왜 거리값이 3개나 있고, 왜 서로 다른가?

```
┌─────────────────────┬──────────┬───────────────────────────────────┐
│ 토픽                 │ 실측값   │ 어떻게 계산되나                     │
├─────────────────────┼──────────┼───────────────────────────────────┤
│ /tof/front_distance │ 0.529m   │ ToF ROI 3x3 유효셀 평균           │
│ /obstacle/distance  │ 0.526m   │ ToF + LiDAR 종합 판단             │
│ /scan (LiDAR 정면)   │ 다양     │ 레이저 360도 중 정면 부근 값        │
└─────────────────────┴──────────┴───────────────────────────────────┘
```

#### 이유 1: 센서 자체가 다르다

```
ToF (VL53L8CX)              LiDAR (LD-08)
───────────────             ──────────────
적외선(IR) 빛 사용           레이저 사용
위에서 아래까지 45도 시야     수평 360도 한 줄만 스캔
로봇 앞쪽에 장착              로봇 위쪽에 장착
높이가 다름!                  높이가 다름!

같은 물체를 봐도:
  ToF: 물체의 "몸통" 거리 (낮은 위치)
  LiDAR: 물체의 "허리/머리" 거리 (높은 위치)
  → 물체가 기울어져 있으면 거리가 다름
```

#### 이유 2: 측정 영역이 다르다

```
위에서 본 시야(FOV) 비교:

                 ToF 시야 (~45도)
                 ┌─────────┐
                 │ ┌─────┐ │
                 │ │ ROI │ │ ← front_distance: 이 작은 영역의 "평균"
                 │ └─────┘ │
                 └─────────┘

    LiDAR 시야 (정면 ±30도 부채꼴)
    ──────────\           /──────────
               \         /
                \       /
                 \     /
                  \ ● /   ← obstacle_distance: 이 부채꼴의 "최솟값"
                   \_/
                  로봇

ToF: 좁은 영역의 "평균" → 여러 셀의 중간값
LiDAR: 넓은 영역의 "최솟값" → 가장 가까운 점
→ 당연히 다른 값이 나옴!
```

#### 이유 3: 계산 방법이 다르다

```
front_distance:
    = ToF ROI 셀들의 평균
    예: (1081 + 868 + 675 + 1041 + 817 + 662 + 1010 + 637) / 8
    = 848.9mm = 0.849m
    (NaN인 셀은 제외하고 평균)

obstacle_distance:
    = obstacle_detector가 ToF와 LiDAR를 종합 판단한 결과
    파라미터:
      enter_threshold = 0.4m  ← 이 거리 이하면 "장애물 진입"
      exit_threshold  = 0.5m  ← 이 거리 이상이면 "장애물 해제"
      lidar_confirm_threshold = 0.4m ← LiDAR로 교차 확인
      sector_deg = 30.0  ← LiDAR 정면 ±30도 범위 사용

    판단 로직 (추정):
      1. ToF front_distance가 enter_threshold(0.4m) 이하인가?
      2. LiDAR 정면 ±30도에서 최솟값이 lidar_confirm(0.4m) 이하인가?
      3. 둘 다 만족 → obstacle/detected = true
      4. obstacle/distance = 두 센서의 종합 거리값
```

### 전체 비교 그림

```
              같은 장애물을 3가지 방식으로 측정:

    ┌─────────────────────────────────────────────────┐
    │                                                 │
    │              ██████████  ← 장애물               │
    │              ██████████                          │
    │                                                 │
    │         ┌ - - - - ┐                             │
    │         : ToF ROI :  → "평균 0.529m"            │
    │         └ - - - - ┘                             │
    │        /            \                           │
    │       / LiDAR ±30도  \  → "최솟값에 기반"        │
    │      /                \                         │
    │              ●                                  │
    │            로봇                                  │
    │                                                 │
    │  obstacle_detector가 위 두 값을 종합 → 0.526m    │
    │                                                 │
    └─────────────────────────────────────────────────┘
```

### ToF 8x8 히트맵에서 보이는 패턴

```
가까운 물체가 있을 때:
    ┌──────────────────────┐
    │ 파  파  파  파  파  파  파  파 │  ← 먼 쪽 (위쪽 행)
    │ 파  파  파  파  파  파  파  파 │
    │ 파  파  주  빨  빨  주  파  파 │  ← ROI: 가까운 값
    │ 파  파  빨  빨  빨  빨  파  파 │  ← ROI: 가장 가까움
    │ 파  파  빨  빨  빨  빨  파  파 │  ← ROI
    │ 파  파  주  빨  빨  주  파  파 │
    │ 파  파  파  파  파  파  파  파 │
    │ 파  파  파  파  파  파  파  파 │  ← 먼 쪽 (아래쪽 행)
    └──────────────────────┘
    빨강 = 가까움, 파랑 = 멀음

읽는 법:
    - 중앙이 빨갈수록 정면에 물체가 가까이 있음
    - 한쪽만 빨간면 물체가 정면이 아니라 옆에 있음
    - NaN이 많으면 센서 시야 밖이거나 표면이 반사 안 됨
    - 전체적으로 파란면 앞이 탁 트인 상태
```

### 정리: 어떤 값을 언제 봐야 하나?

```
┌──────────────────────┬────────────────────────────────────┐
│ 상황                  │ 봐야 할 값                          │
├──────────────────────┼────────────────────────────────────┤
│ 정면 물체 정밀 거리     │ /tof/front_distance               │
│ 장애물 회피 판단       │ /obstacle/detected + distance      │
│ 360도 주변 환경        │ /scan (LiDAR)                     │
│ 정면 물체의 형태/크기   │ ToF 8x8 히트맵 전체                │
│ 센서 고장 진단         │ /tof/status (NaN 패턴 확인)        │
└──────────────────────┴────────────────────────────────────┘

값이 서로 다른 것은 정상!
→ 다른 센서, 다른 위치, 다른 높이, 다른 계산 방식
→ 오히려 여러 센서를 조합해야 더 정확한 판단 가능
   (이것을 "센서 퓨전"이라 함)
```

---

## 15. 유용한 명령어 모음

### 토픽 관련

```bash
# 토픽 목록
ros2 topic list

# 토픽 데이터 보기 (한 번)
ros2 topic echo /robot/state --once

# 토픽 데이터 보기 (계속)
ros2 topic echo /tof/front_distance

# 토픽 주파수(Hz) 확인
ros2 topic hz /scan

# 토픽 상세 정보 (퍼블리셔/서브스크라이버/QoS)
ros2 topic info /cmd_vel --verbose

# 토픽 대역폭 확인
ros2 topic bw /scan
```

### 노드 관련

```bash
# 실행 중인 노드 목록
ros2 node list

# 노드 상세 정보 (어떤 토픽을 구독/발행하는지)
ros2 node info /tb3_monitor
```

### 디버깅

```bash
# ROS2 전체 시스템 진단
ros2 doctor

# 멀티캐스트 테스트 (DDS 통신 확인)
ros2 multicast receive    # 터미널1
ros2 multicast send       # 터미널2

# ROS2 데몬 재시작 (토픽 목록이 이상할 때)
ros2 daemon stop && ros2 daemon start

# TF 트리 확인
ros2 run tf2_tools view_frames
# → frames.pdf 파일 생성됨

# 환경 변수 확인
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
```

### 빌드 관련

```bash
# 특정 패키지만 빌드
colcon build --packages-select tb3_monitor --symlink-install

# 빌드 후 환경 적용 (반드시!)
source install/setup.bash

# 전체 빌드
colcon build --symlink-install
```

---

## 16. 자주 만나는 문제와 해결

### "토픽이 안 보여요"

```
원인 1: ROS_DOMAIN_ID 불일치
  확인: echo $ROS_DOMAIN_ID (양쪽 다)
  해결: 둘 다 같은 값으로 설정

원인 2: VMware가 NAT 모드
  확인: ip addr show → Pi5와 같은 대역인지?
  해결: VMware → Bridged 모드로 변경

원인 3: 방화벽
  확인: sudo ufw status
  해결: sudo ufw allow 7400:7500/udp

원인 4: DDS 구현체 불일치
  확인: echo $RMW_IMPLEMENTATION (양쪽 다)
  해결: 둘 다 rmw_cyclonedds_cpp 사용
```

### "토픽은 보이는데 데이터가 안 와요"

```
원인: QoS 불일치 (거의 100%)
  확인: ros2 topic info /scan --verbose
  → Publisher QoS와 Subscriber QoS 비교
  해결: Subscriber QoS를 Publisher에 맞춤
  (특히 /scan은 BEST_EFFORT로 맞춰야 함)
```

### "RViz2에 아무것도 안 보여요"

```
원인 1: Fixed Frame 설정 오류
  해결: RViz2 좌측 → Global Options → Fixed Frame = "base_link"

원인 2: 토픽 구독 안 됨
  해결: RViz2에서 해당 Display의 Topic 확인

원인 3: TF 없음
  해결: ros2 topic echo /tf --once → 데이터 오는지 확인
```

### "colcon build 에러"

```
원인: 의존성 패키지 없음
  해결: rosdep install --from-paths src --ignore-src -r -y

원인: source 안 함
  해결: source /opt/ros/jazzy/setup.bash
        source install/setup.bash
```

---

## 부록: 용어 사전

| 용어 | 의미 |
|------|------|
| **노드(Node)** | 하나의 ROS2 프로그램 |
| **토픽(Topic)** | 데이터를 주고받는 이름 있는 채널 |
| **퍼블리셔(Publisher)** | 토픽에 데이터를 올리는 쪽 |
| **서브스크라이버(Subscriber)** | 토픽의 데이터를 읽는 쪽 |
| **메시지(Message)** | 토픽으로 전달되는 데이터 구조 |
| **DDS** | 노드 간 통신 프로토콜 |
| **QoS** | 통신 품질 설정 (신뢰성, 속도 등) |
| **TF** | 좌표계 간 변환 정보 |
| **SLAM** | 동시 위치추정 및 지도작성 |
| **Nav2** | ROS2 자율 주행 스택 |
| **LiDAR** | 레이저로 360도 거리 측정하는 센서 |
| **ToF** | 빛의 비행시간으로 거리 측정하는 센서 |
| **IMU** | 가속도/각속도 측정 센서 |
| **Odom** | 바퀴 회전으로 추정한 이동 거리 |
| **RViz2** | 3D 시각화 도구 |
| **rqt** | GUI 기반 도구 모음 |
| **colcon** | ROS2 빌드 도구 |
| **Launch** | 여러 노드를 한 번에 실행하는 스크립트 |
| **frame_id** | 좌표계 이름 (예: base_link, odom) |
| **Marker** | RViz2에 표시하는 시각적 도형 |
| **ROI** | Region of Interest, 관심 영역. 전체 중 필요한 부분만 추출 |
| **센서 퓨전** | 여러 센서 데이터를 종합하여 더 정확한 판단을 하는 것 |
| **FOV** | Field of View, 센서의 시야각 |
| **멀티존(Multi-zone)** | 하나의 센서가 여러 영역을 동시에 측정하는 방식 |
| **VL53L8CX** | ST마이크로의 8x8 멀티존 ToF 센서 |
