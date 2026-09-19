# multibot_control_ui

Pinky Pro 2대의 지도, 위치, Nav2 목표, 수동 주행과 병목 통과를 관리하는 ROS 2
관제 UI 패키지.

현재 구성은 **robot1(domain 21), robot2(domain 19), 관제 PC(domain 22)** 를
전제로 한다. 로봇 대수·이름·도메인·토픽을 바꾸려면
`multibot_control_ui/robot_config.py`와 domain bridge 설정을 함께 수정해야 한다.

## 문서 안내

- 이 README: 설치, 실행, 기본 운영, 빠른 점검
- [현재 구현 사양](docs/CURRENT_IMPLEMENTATION.md): 상태 전이, 안전 조건, 파라미터,
  알려진 제약
- [코드 구조와 개발 가이드](docs/ARCHITECTURE.md): 모듈 책임, 주요 호출 흐름,
  상태 소유권, 변경 시 확인할 곳

## 공유 전제 조건

이 저장소의 관제 패키지만으로 로봇 측 기능이 자동 설치되지는 않는다. 팀원 환경에
다음 항목이 준비되어 있어야 한다.

- ROS 2 Jazzy와 `domain_bridge`, Nav2 메시지, Tkinter, PyYAML
- 같은 워크스페이스의 `pinky_interfaces` (`FleetPermit`, `RobotHeartbeat`)
- 각 로봇의 pose reporter, velocity gate, LED server, Nav2 구성
- 관제 PC에서 domain 19, 21, 22 DDS discovery가 가능한 네트워크/방화벽 설정

## 주요 기능

- robot1·robot2 위치를 공유 지도에 표시
- 로봇별 AMCL 초기 위치 전송
- 지도 드래그로 Nav2 `NavigateToPose` 목표 전송
- 선택 로봇으로 관제 PC `/cmd_vel` 전달
- 병목 구역 capacity 1 제어
  - 선행 로봇: `RUN`
  - 후행 로봇: 확장 경계에서 `HOLD`
  - 선행 로봇 이탈 후: 후행 로봇 자동 재출발
- 로봇 로컬 velocity gate를 통한 최종 `/cmd_vel` 차단
- heartbeat·pose 통신 상태 감시
- LED 주행 상태 표시
  - 초록: 주행 허용
  - 노랑: 일시 대기
  - 빨강: E-STOP 또는 permit 단절

## 시스템 구성

| 장치 | `ROS_DOMAIN_ID` | 역할 |
|---|---:|---|
| robot1 | 21 | Nav2, 모터, 위치 보고, velocity gate, LED |
| robot2 | 19 | Nav2, 모터, 위치 보고, velocity gate, LED |
| 관제 PC | 22 | UI, 병목 판단, permit 발행, 수동 명령 입력 |

- 일반 토픽: `domain_bridge`로 전달
- Nav2 액션: UI가 각 robot domain에 직접 연결
- 로봇 ID: 각 로봇의 `bringup_robot.launch.xml`에 명시
  - robot1 PC: `robot_id=robot1`
  - robot2 PC: `robot_id=robot2`

## 통신 구조

### 수동 속도 명령

```text
/cmd_vel (domain 22)
  -> multibot_control_ui
  -> /robot1/cmd_vel 또는 /robot2/cmd_vel
  -> domain_bridge
  -> /cmd_vel_candidate (robot domain)
  -> fleet_velocity_gate
  -> /cmd_vel
  -> motor driver
```

### Nav2 속도 명령

```text
Nav2 controller / behavior server
  -> cmd_vel_nav
  -> velocity_smoother
  -> cmd_vel_candidate
  -> fleet_velocity_gate
  -> cmd_vel
```

### Nav2 목표

```text
UI 지도 드래그
  -> robot1 ActionClient (domain 21) -> /navigate_to_pose
  -> robot2 ActionClient (domain 19) -> /navigate_to_pose
```

- `NavigateToPose` 액션: domain bridge를 통과하지 않음
- UI 프로세스 내부: domain 21·19 전용 ROS context 생성
- goal, feedback, result, cancel: 각 Nav2 서버와 직접 송수신
- 실제 모터 출력: velocity gate를 통과하므로 병목 제한 유지

## 병목 제어

### 기본 동작

- 실제 로봇 pose가 먼저 병목과 겹친 로봇을 점유자로 지정
- 목표 경로를 이용한 사전 예약: 사용하지 않음
- 병목 점유자: 확장 경계를 벗어날 때까지 `RUN`
- 후행 로봇: 점유 중인 병목의 확장 경계에 도착하면 `HOLD`
- 점유자 이탈: 후행 로봇을 `RUN`으로 자동 변경
- `HOLD` 중: Nav2 goal 유지
- Nav2 goal 중단: 저장된 최신 goal 자동 재전송
- 병목 경계 충돌: 전역 E-STOP으로 전환하지 않음

### 안전거리

```text
점유 진입 거리 = 병목 폴리곤 + robot_radius_m
점유 해제 거리 = 병목 폴리곤 + robot_radius_m + clearance_margin_m
```

- 기본 로봇 반경: `0.09 m`
- 기본 추가 마진: `0.01 m`
- 기본 최종 확장 거리: `0.10 m`
- UI 라벨 표시
  - 병목 사각형 `x/y` 범위
  - 로봇 반경 `R`
  - 추가 마진 `M`
  - 최종 판정 거리 `R + M`

### 구역 상태

| 상태 | UI 색상 | 의미 |
|---|---|---|
| `FREE` | 초록 | 점유자 없음 |
| `OCCUPIED` | 주황 | 선행 로봇이 병목 점유 중 |
| `CLEARING` | 노랑 | 점유자가 본 구역을 나왔지만 확장 경계 안에 있음 |
| `CONFLICT` | 보라 | 후행 로봇도 경계에 진입, 후행 로봇만 HOLD |
| `LOCKED` | 빨강 | E-STOP으로 구역 잠김 |

### 병목 구역 설정

- UI에서 `관제 일시정지` 선택
- `병목 구역 지정` 선택
- 지도에서 사각형의 한 모서리부터 반대쪽 모서리까지 드래그
- 표시된 좌표와 안전거리 확인
- `관제 시작 / 상태 재확인` 선택
- UI 생성 구역: runtime 구역 하나로 적용
- UI 종료 시 삭제: 영구 구역은 `config/bottleneck_zones.yaml`에 등록

## HOLD와 E-STOP

### HOLD

- 의미: 정상적으로 자동 복구되는 일시 정지
- 적용 조건
  - 다른 로봇이 병목을 점유하고 후행 로봇이 확장 경계에 도착
  - 로봇 gate heartbeat stale
  - 활성 병목이 있고 로봇 pose stale
  - 관제 시작 전 또는 관제 일시정지
- 원인 해제: 자동 `RUN` 복귀
- Nav2 goal: 유지

### E-STOP

- 적용 조건
  - UI의 `전체 정지 / 선택 해제` 선택
  - 병목 점유자의 pose stale
  - UI 정상 종료
- 처리 내용
  - 두 로봇 `MODE_ESTOP`
  - 실행 중인 모든 Nav2 goal 일시 취소
  - 로봇별 최신 goal 좌표 유지
  - 수동 선택 제거
  - LED 빨강 표시
- 복구 순서
  - 원인 해소
  - `관제 시작 / 상태 재확인` 선택
  - 저장된 최신 Nav2 goal 자동 재전송

## 빌드

### 관제 PC

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  pinky_interfaces multibot_control_ui
source install/setup.bash
```

### 로봇 PC

```bash
cd ~/pinky
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  pinky_interfaces pinky_led pinky_fleet_safety \
  pinky_bringup pinky_navigation
source install/setup.bash
```

## 실행

### 1. robot1 bringup

```bash
cd ~/pinky
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=21
ros2 launch pinky_bringup bringup_robot.launch.xml
```

- 모터·센서 실행
- `fleet_velocity_gate` 실행
- `fleet_pose_reporter` 실행
- `pinky_led led_server` 실행
- 별도의 `ros2 run pinky_led led_server` 실행 불필요

### 2. robot1 Nav2

새 터미널:

```bash
cd ~/pinky
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=21
ros2 launch pinky_navigation bringup_launch.xml
```

### 3. robot2

- robot1과 동일한 순서로 bringup·Nav2 실행
- 두 터미널 모두 `ROS_DOMAIN_ID=19` 사용
- robot2 PC의 `bringup_robot.launch.xml`: `robot_id=robot2` 적용 확인

### 4. Domain bridge

관제 PC 터미널 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
BRIDGE_CONFIG="$(ros2 pkg prefix --share multibot_control_ui)/config/domain_bridge.yaml"
ROS_DOMAIN_ID=22 ros2 run domain_bridge domain_bridge "$BRIDGE_CONFIG"
```

- `pinky_interfaces not found`: 관제 overlay source 여부 확인
- 기본 설정 파일: `config/domain_bridge.yaml`
- 로봇 domain이나 토픽을 바꾸면 이 파일과 `robot_config.py`를 함께 수정

### 5. 관제 UI

관제 PC 터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 launch multibot_control_ui control_ui.launch.py
```

안전거리 변경 예시:

```bash
ros2 launch multibot_control_ui control_ui.launch.py \
  runtime_zone_robot_radius_m:=0.09 \
  runtime_zone_clearance_margin_m:=0.01
```

### 6. 키보드 수동 조작

관제 PC 터미널 3:

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
ROS_DOMAIN_ID=22 ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- UI에서 조작할 로봇 선택
- 선택 전 `/cmd_vel`: 폐기
- 로봇 변경: 이전 로봇에 정지 명령 발행
- 관제 명령 0.5초 단절: 자동 정지
- 병목 `HOLD`: 수동 명령도 차단

## UI 사용 순서

1. 두 로봇의 map, pose, heartbeat 확인
2. 필요 시 로봇별 AMCL 초기 위치 입력 후 `설정` 선택
3. 병목 구역 지정 후 관제 시작
4. robot1 또는 robot2 선택
5. 주행 방식 선택
   - Nav2: 지도에서 목표 위치를 누르고 도착 방향으로 10 px 이상 드래그
   - 수동: domain 22의 `/cmd_vel` 입력
6. UI에서 goal, 남은 거리, gate, 병목 상태 확인

## AMCL 초기 위치 기본값

| 로봇 | x (m) | y (m) | yaw (deg) |
|---|---:|---:|---:|
| robot1 / domain 21 | `1.3` | `1.0` | `-176.0` |
| robot2 / domain 19 | `-0.2` | `0.6` | `-90.0` |

- UI 입력 단위: degree
- 내부 변환: quaternion `z`, `w`
- frame: `map`
- 실행 중 자유롭게 수정 가능
- `fleet_pose_reporter`: 정지 중에도 현재 pose를 약 5 Hz로 전달

## 주요 파라미터

| 파라미터 | 기본값 | 용도 |
|---|---:|---|
| `input_cmd_vel_topic` | `/cmd_vel` | 관제 공용 속도 입력 |
| `command_timeout_sec` | `0.5` | 관제 수동 명령 timeout |
| `permit_ttl_sec` | `0.5` | 로봇 gate permit 유효시간 |
| `permit_publish_rate_hz` | `10.0` | permit 발행 주기 |
| `heartbeat_stale_sec` | `1.5` | gate heartbeat stale 판정 |
| `pose_stale_sec` | `3.0` | 로봇 pose stale 판정 |
| `runtime_zone_robot_radius_m` | `0.09` | UI 병목 로봇 반경 |
| `runtime_zone_clearance_margin_m` | `0.01` | UI 병목 추가 마진 |

## 점검 명령

관제 PC:

```bash
source ~/colcon_ws/install/setup.bash
ROS_DOMAIN_ID=22 ros2 topic list | grep -E 'robot[12]|cmd_vel'
ROS_DOMAIN_ID=22 ros2 topic hz /robot1/amcl_pose
ROS_DOMAIN_ID=22 ros2 topic hz /robot2/amcl_pose
ROS_DOMAIN_ID=22 ros2 topic echo /robot1/fleet/heartbeat
ROS_DOMAIN_ID=22 ros2 topic echo /robot2/fleet/heartbeat
```

Nav2 액션 직접 연결:

```bash
ROS_DOMAIN_ID=21 ros2 action info /navigate_to_pose
ROS_DOMAIN_ID=19 ros2 action info /navigate_to_pose
```

로봇 PC:

```bash
ros2 topic hz /fleet/pose
ros2 topic echo /fleet/heartbeat
ros2 topic echo /fleet/permit
ros2 topic echo /cmd_vel_candidate
ros2 topic echo /cmd_vel
```

## 파일 구성

```text
multibot_control_ui/
├── config/
│   ├── bottleneck_zones.yaml          # 영구 병목 설정
│   └── domain_bridge.yaml              # 3개 domain 토픽 bridge 예제
├── docs/ARCHITECTURE.md                # 코드 구조와 변경 가이드
├── docs/CURRENT_IMPLEMENTATION.md      # 현재 구현 상세 문서
├── launch/control_ui.launch.py         # 관제 domain 및 파라미터
├── multibot_control_ui/
│   ├── control_ui.py                   # Tk UI
│   ├── control_node.py                 # ROS 토픽과 permit
│   ├── fleet_coordinator.py            # 병목 상태·주행 정책
│   ├── navigation_client.py            # domain별 Nav2 액션
│   ├── robot_config.py                 # 로봇·domain 등록
│   ├── zone_config.py                  # 병목 지오메트리
│   └── map_math.py                     # 지도 좌표 변환
├── test/                               # 자동 테스트
├── package.xml
└── setup.py
```

## 테스트

```bash
cd ~/colcon_ws/src/pinky-fleet-control/multibot_control_ui
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
pytest -q
```

- 최신 확인 결과는 [현재 구현 사양](docs/CURRENT_IMPLEMENTATION.md)의 테스트 절을 참고
- 코드 변경 후: 빌드와 테스트 재실행 권장

## 상세 문서

- [현재 구현 상태](docs/CURRENT_IMPLEMENTATION.md)
- [코드 구조와 개발 가이드](docs/ARCHITECTURE.md)
