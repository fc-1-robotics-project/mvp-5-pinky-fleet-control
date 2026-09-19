# 멀티로봇 병목 관제 현재 구현 상태

기준일: 2026-09-19

이 문서는 현재 실행 코드와 패키지의 `config/domain_bridge.yaml`을 기준으로 정리한
현행 사양이다.
향후 동작이나 설정을 변경하면 이 문서를 함께 갱신한다.

## 1. 구현 범위

현재 시스템은 동일한 맵을 사용하는 Pinky Pro 2대를 한 관제 UI에서 다음과 같이
운영한다.

- 로봇 위치와 맵을 관제 PC에서 표시한다.
- UI에서 AMCL 초기 위치와 Nav2 목표를 로봇별로 전송한다.
- 관제 PC의 공용 `/cmd_vel`을 UI에서 선택한 한 로봇에 전달한다.
- 지도에 capacity 1 병목 구역을 지정하고 실제 로봇 위치로 점유자를 정한다.
- 선행 로봇은 통과시키고 후행 로봇만 병목 확장 경계에서 `HOLD`한다.
- 관제 명령이 끊기거나 안전 조건이 충족되지 않으면 로봇 로컬 PC의 velocity
  gate가 최종 `/cmd_vel`을 0으로 만든다.
- 로봇 LED로 주행 허용, 일시 대기, 비상·통신 단절 상태를 표시한다.

## 2. 시스템 구성과 ROS 도메인

| 장치 | ROS domain | 역할 |
|---|---:|---|
| robot1 | 21 | Nav2, 위치 보고, velocity gate, 모터, LED |
| robot2 | 19 | Nav2, 위치 보고, velocity gate, 모터, LED |
| 관제 PC | 22 | `multibot_control_ui`, 병목 판단, permit 발행 |

관제 토픽은 `domain_bridge`가 전달하지만 Nav2 `NavigateToPose` 액션은 브리지하지
않는다. UI 프로세스가 domain 21과 19에 별도 ROS context와 액션 클라이언트를
생성해 각 로봇의 `/navigate_to_pose`에 직접 접속한다.

속도 명령의 최종 경로는 다음과 같다.

```text
Nav2: controller/behavior -> cmd_vel_nav -> velocity_smoother
수동: 관제 /cmd_vel -> 선택된 /robotN/cmd_vel -> domain_bridge
                                      |
                                      v
로봇 로컬:                   /cmd_vel_candidate
                                      |
                              fleet_velocity_gate
                                      |
                              /cmd_vel -> motor
```

관제와 안전 상태의 왕복 경로는 다음과 같다.

```text
map -> base_footprint TF -> /fleet/pose (5 Hz)
                            -> bridge -> /robotN/amcl_pose -> 관제

관제 /robotN/fleet/permit (10 Hz)
                            -> bridge -> /fleet/permit -> velocity gate

velocity gate /fleet/heartbeat (5 Hz)
                            -> bridge -> /robotN/fleet/heartbeat -> 관제
```

## 3. 구성 요소별 역할

### 관제 PC

| 파일 | 역할 |
|---|---|
| `multibot_control_ui/control_ui.py` | Tk UI, 지도·로봇·목표·병목 상태 표시와 사용자 입력 |
| `multibot_control_ui/control_node.py` | 토픽 송수신, 수동 명령 라우팅, permit 반복 발행, freshness 판단 |
| `multibot_control_ui/fleet_coordinator.py` | 병목 점유 상태와 로봇별 `RUN/HOLD/E-STOP` 결정 |
| `multibot_control_ui/navigation_client.py` | domain 21/19의 Nav2 액션에 직접 목표 전송·취소 |
| `multibot_control_ui/zone_config.py` | 병목 YAML 로딩과 폴리곤·확장 경계 계산 |
| `multibot_control_ui/robot_config.py` | 로봇 ID, domain, 토픽, 초기 위치 등록 |
| `config/bottleneck_zones.yaml` | 시작 시 로드할 영구 병목 구역 설정 |
| `config/domain_bridge.yaml` | 관제 domain과 두 robot domain 사이의 토픽 bridge 예제 |
| `launch/control_ui.launch.py` | 관제 domain 22와 기본 파라미터 설정 |

모듈 사이의 호출 방향과 상태 소유권은
[코드 구조와 개발 가이드](ARCHITECTURE.md)에 별도로 정리했다.

### 로봇 로컬 PC

로봇 측 구현은 별도 `pinky_pro` 소스 저장소에서 관리한다. 실제 로봇 PC에는 관제
저장소와 호환되는 버전의 변경 사항을 적용해야 한다.

| 패키지·파일 | 역할 |
|---|---|
| `pinky_fleet_safety/velocity_gate.py` | permit과 candidate 명령이 모두 유효할 때만 최종 `/cmd_vel` 출력 |
| `pinky_fleet_safety/gate_policy.py` | fail-closed 판정 및 LED 상태 결정 |
| `pinky_fleet_safety/pose_reporter.py` | `map -> base_footprint` TF를 `/fleet/pose`로 5 Hz 발행 |
| `pinky_navigation/launch/navigation_launch.xml` | Nav2 출력을 `cmd_vel_nav`, smoother 출력을 `cmd_vel_candidate`로 remap |
| `pinky_bringup/launch/bringup_robot.launch.xml` | 모터, 센서, LED server, velocity gate, pose reporter 실행 |
| `pinky_interfaces` | `FleetPermit`, `RobotHeartbeat`, `SetLed` 인터페이스 제공 |

최종 모터용 `/cmd_vel`은 velocity gate만 발행해야 한다. 추가 수동 조작 노드나
비전 제어 노드를 붙일 때에도 출력 목적지는 `/cmd_vel_candidate`로 설정한다.

## 4. Domain bridge 설정

공유 가능한 기본 설정 파일은 `config/domain_bridge.yaml`이다. 빌드 후에는 패키지
share 경로에도 함께 설치된다.

| 관제 PC 토픽(domain 22) | 방향 | 로봇 로컬 토픽 | robot domain |
|---|---|---|---:|
| `/robot1/map` | <- | `/map` | 21 |
| `/robot2/map` | <- | `/map` | 19 |
| `/robot1/amcl_pose` | <- | `/fleet/pose` | 21 |
| `/robot2/amcl_pose` | <- | `/fleet/pose` | 19 |
| `/robot1/cmd_vel` | -> | `/cmd_vel_candidate` | 21 |
| `/robot2/cmd_vel` | -> | `/cmd_vel_candidate` | 19 |
| `/robot1/fleet/permit` | -> | `/fleet/permit` | 21 |
| `/robot2/fleet/permit` | -> | `/fleet/permit` | 19 |
| `/robot1/fleet/heartbeat` | <- | `/fleet/heartbeat` | 21 |
| `/robot2/fleet/heartbeat` | <- | `/fleet/heartbeat` | 19 |
| `/robot1/initialpose` | -> | `/initialpose` | 21 |
| `/robot2/initialpose` | -> | `/initialpose` | 19 |

`/map`은 latched map을 받을 수 있도록 reliable/transient-local QoS를 사용한다.
LED 서비스는 로봇 내부에서만 사용하므로 브리지하지 않는다.

브리지를 실행할 때 `pinky_interfaces`를 찾지 못하는 오류가 나지 않도록 관제
워크스페이스를 반드시 source해야 한다.

## 5. 병목 구역 판정

### 구역 생성

- YAML에서 `enabled: true`인 폴리곤을 시작 시 로드할 수 있다.
- 기본 예제 구역은 `enabled: false`이므로 초기에는 활성 병목이 없다.
- UI에서 `병목 구역 지정`을 누르고 지도에서 대각선 방향으로 드래그하면 축 정렬
  사각형 하나를 만든다.
- 구역 수정과 삭제는 관제를 일시정지한 상태에서만 가능하다.
- UI로 새 사각형을 만들면 현재 runtime 구역 전체를 그 사각형 하나로 교체한다.
- UI에서 만든 구역은 YAML에 저장되지 않으므로 UI를 재시작하면 사라진다.
- 사각형은 x, y 방향 모두 최소 0.10 m 이상이어야 한다.

### 로봇 footprint와 확장 경계

병목 점유는 로봇 중심점만으로 판단하지 않는다.

```text
점유 진입 판정 = 병목 폴리곤 + robot_radius_m
점유 해제 및 후행 HOLD 판정 = 병목 폴리곤
                              + robot_radius_m
                              + clearance_margin_m
```

현재 UI runtime 기본값은 `robot_radius_m=0.09 m`,
`clearance_margin_m=0.01 m`이며 최종 확장 거리는 0.10 m이다. UI 라벨에 실제
사각형의 `x/y` 범위와 `R`, `M`, 합계가 표시된다.

### 점유 상태

| 상태 | UI 색상 | 의미 |
|---|---|---|
| `FREE` | 초록 | 확장 구역에 기존 점유자가 없음 |
| `OCCUPIED` | 주황 | 소유 로봇 footprint가 병목과 겹침 |
| `CLEARING` | 노랑 | 소유 로봇은 본 구역에서 나왔지만 확장 경계 안에 있음 |
| `CONFLICT` | 보라 | 후행 로봇도 병목 경계에 닿음. 소유자는 RUN, 후행만 HOLD |
| `LOCKED` | 빨강 | E-STOP 등으로 구역 잠김 |

실제 pose가 먼저 병목과 겹친 로봇이 소유자가 된다. 두 로봇이 같은 갱신 주기에
동시에 겹치면 중심이 폴리곤 안에 있는 로봇을 우선하고, 그래도 둘이면 구역 중심에
더 가까운 로봇을 선택한다.

소유 로봇은 확장 경계를 완전히 벗어날 때까지 소유권을 유지한다. 다른 로봇은
소유 중인 구역의 확장 경계에 들어왔을 때만 `HOLD`된다. 후행 로봇이 제동 지연으로
본 구역에 조금 진입해도 전역 E-STOP으로 바꾸지 않으며, 기존 소유자가 빠져나오면
그 후행 로봇으로 소유권을 자동 인계해 복구한다.

## 6. Nav2 목표와 수동 조작

### Nav2 목표

- 병목 점유 여부와 관계없이 새 목표를 항상 받을 수 있다.
- 지도에서 목표 지점을 누르고 도착 방향으로 10 px 이상 드래그해 전송한다.
- Nav2 액션 서버가 준비되지 않았으면 로봇별 최신 목표 하나를 보관하고 1초마다
  다시 전송한다.
- 병목에서 `HOLD`되어도 Nav2 goal은 취소하지 않는다. permit이 `RUN`으로 바뀌면
  기존 goal로 계속 주행한다.
- 대기 중 Nav2 goal이 더 이상 `ACTIVE/PENDING`이 아니면 저장된 최신 목표를 다시
  전송한다.
- 새 goal을 보낼 때 같은 로봇의 수동 명령 선택은 해제된다.
- 이전 goal의 늦은 callback이 최신 goal 상태를 덮어쓰지 않도록 goal generation을
  구분한다.

### 수동 `/cmd_vel`

- UI에서 robot1 또는 robot2를 선택하면 필요할 경우 관제가 자동으로 시작된다.
- 선택 로봇에 fresh heartbeat가 있어야 하며, 활성 병목이 있으면 fresh pose도
  있어야 한다.
- 관제 domain의 `/cmd_vel`은 선택된 로봇에만 전달된다.
- 선택을 바꾸면 기존 선택 로봇에 0 속도를 먼저 발행한다.
- 관제 PC의 명령이 0.5초 이상 끊기면 0 속도를 발행한다.
- 로봇 로컬 gate가 `HOLD/E-STOP`이면 수동 명령도 Nav2와 동일하게 차단된다.
- 수동 조작을 선택하면 해당 로봇에 저장된 Nav2 goal을 취소하고 제거한다.

## 7. Velocity gate와 fail-closed 동작

관제 PC는 `FleetPermit`을 기본 10 Hz로 반복 발행하며 TTL은 0.5초다. 로봇 gate는
다음 두 조건을 모두 만족할 때에만 candidate 속도를 최종 `/cmd_vel`로 내보낸다.

1. 자기 `robot_id`와 일치하는 fresh `MODE_RUN` permit
2. 0.25초 이내에 수신한 fresh `/cmd_vel_candidate`

| gate 상태 | 최종 출력 |
|---|---|
| `RUN` | candidate 속도 통과 |
| `COMMAND_TIMEOUT` | 0 속도 |
| `HOLD` | 0 속도 |
| `E_STOP` | 0 속도 |
| `NO_PERMIT` / `PERMIT_TIMEOUT` | 0 속도 |

로봇은 gate 상태를 `/fleet/heartbeat`로 기본 5 Hz 발행한다. heartbeat에는 로봇 ID,
gate mode, permit·command freshness, 출력 허용 여부, status, controller ID, 마지막
permit sequence, 활성 lease ID가 포함된다.

## 8. HOLD와 E-STOP의 구분

`HOLD`는 정상적으로 복구 가능한 일시 정지다.

- 다른 로봇이 병목을 점유한 상태에서 후행 로봇이 확장 경계에 도착함
- 해당 로봇의 heartbeat가 1.5초보다 오래됨
- 활성 병목이 있는데 해당 로봇 pose가 3.0초보다 오래됨
- 관제 시작 전 또는 관제 일시정지

위 조건이 해제되면 로봇별 permit은 자동으로 `RUN`으로 돌아간다. 병목 경계 충돌은
전역 E-STOP 조건이 아니다.

현재 E-STOP 조건은 다음과 같다.

- 사용자가 UI의 `전체 정지 / 선택 해제`를 누름: `OPERATOR_ESTOP`
- 병목 소유 로봇의 pose가 stale 상태가 됨: `OWNER_POSE_STALE`
- UI가 정상 종료됨: `UI_CLOSED`

E-STOP은 두 로봇의 실행 중인 Nav2 goal을 취소하고 수동 선택을 지우며 양쪽 gate에
`MODE_ESTOP`을 보낸다. 단, 로봇별 최신 목표 좌표는 관제 coordinator에 유지한다.
원인을 해소한 뒤 UI에서 `관제 시작 / 상태 재확인`을 누르면 저장된 목표를 각
Nav2에 자동 재전송한다. 재시작 시 로봇이 병목 HOLD 대상이어도 goal은 전송하고
velocity gate만 `HOLD`를 유지한다.

## 9. LED 상태 표시

`pinky_bringup`이 로봇 로컬의 `pinky_led led_server`를 자동 실행하며 velocity gate가
`/set_led` 서비스를 호출한다. 동일 로봇에서 LED server를 수동으로 하나 더 실행하지
않는다.

| permit 상태 | LED | 의미 |
|---|---|---|
| fresh `MODE_RUN` | 초록 `(0, 255, 0)` | 이동 허용 |
| fresh `MODE_HOLD` | 노랑 `(255, 180, 0)` | 병목 대기 또는 관제 일시정지 |
| `MODE_ESTOP` 또는 permit 없음·만료 | 빨강 `(255, 0, 0)` | fail-closed 정지 |

`COMMAND_TIMEOUT`은 이동 허가 자체는 유효하므로 LED는 초록색을 유지한다. LED
서비스가 준비되지 않아도 gate의 속도 차단 기능은 계속 동작하며 서비스 연결을
재시도한다.

## 10. 현재 기본 설정

### 관제 파라미터

| 파라미터 | 기본값 |
|---|---:|
| `input_cmd_vel_topic` | `/cmd_vel` |
| `command_timeout_sec` | 0.5 s |
| `permit_ttl_sec` | 0.5 s |
| `permit_publish_rate_hz` | 10 Hz |
| `heartbeat_stale_sec` | 1.5 s |
| `pose_stale_sec` | 3.0 s |
| `runtime_zone_robot_radius_m` | 0.09 m |
| `runtime_zone_clearance_margin_m` | 0.01 m |

### 로봇 gate 파라미터

| 파라미터 | 기본값 |
|---|---:|
| candidate timeout | 0.25 s |
| 최대 permit TTL | 1.0 s |
| 출력 주기 | 20 Hz |
| heartbeat 주기 | 5 Hz |
| pose report 주기 | 5 Hz |

### UI 초기 위치 입력 기본값

아래 값은 현재 `robot_config.py`의 입력 기본값이며 UI에서 언제든 바꿀 수 있다.

| 로봇 | x (m) | y (m) | yaw (deg) |
|---|---:|---:|---:|
| robot1 / domain 21 | 1.3 | 1.0 | -176.0 |
| robot2 / domain 19 | -0.2 | 0.6 | -90.0 |

## 11. 빌드와 실행

### 로봇 PC 최초 적용 또는 변경 후 빌드

각 로봇 PC에서 다음을 실행한다.

```bash
cd ~/pinky
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  pinky_interfaces pinky_led pinky_fleet_safety \
  pinky_bringup pinky_navigation
source install/setup.bash
```

`bringup_robot.launch.xml`의 `robot_id`는 launch 인자가 아니라 PC별로 명시되어 있다.
robot1 PC는 `robot1`, robot2 PC에 복사한 파일은 반드시 `robot2`로 바꿔야 한다.

robot1은 서로 다른 두 터미널에서 다음을 실행한다.

```bash
cd ~/pinky
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=21
ros2 launch pinky_bringup bringup_robot.launch.xml
```

```bash
cd ~/pinky
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=21
ros2 launch pinky_navigation bringup_launch.xml
```

robot2는 동일하게 실행하되 두 터미널 모두 `ROS_DOMAIN_ID=19`를 사용한다.

### 관제 PC 빌드

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  pinky_interfaces multibot_control_ui
source install/setup.bash
```

### 관제 PC 실행

터미널 1에서 브리지를 실행한다. 이 터미널에서도 overlay를 source해야
`pinky_interfaces`를 찾을 수 있다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
BRIDGE_CONFIG="$(ros2 pkg prefix --share multibot_control_ui)/config/domain_bridge.yaml"
ROS_DOMAIN_ID=22 ros2 run domain_bridge domain_bridge "$BRIDGE_CONFIG"
```

터미널 2에서 UI를 실행한다. launch 파일이 domain 22를 설정한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 launch multibot_control_ui control_ui.launch.py
```

필요하면 runtime 안전 거리를 launch 인자로 바꾼다.

```bash
ros2 launch multibot_control_ui control_ui.launch.py \
  runtime_zone_robot_radius_m:=0.09 \
  runtime_zone_clearance_margin_m:=0.01
```

터미널 3에서 키보드 수동 명령을 사용할 수 있다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
ROS_DOMAIN_ID=22 ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 12. 운영 순서

1. 각 로봇에서 bringup을 먼저 실행한다.
2. 각 로봇에서 Nav2를 실행하고 localization과 Nav2 lifecycle 활성화를 확인한다.
3. 관제 PC에서 overlay를 source한 뒤 domain bridge를 실행한다.
4. 관제 UI를 실행하고 두 로봇의 map, pose, heartbeat를 확인한다.
5. 필요하면 UI에서 AMCL 초기 위치를 전송한다.
6. `관제 일시정지` 상태에서 병목 사각형을 지정한다.
7. 라벨에 표시된 사각형 좌표와 `R + M` 값을 확인한다.
8. `관제 시작 / 상태 재확인`을 누른다.
9. 로봇을 선택해 Nav2 목표를 보내거나 수동 `/cmd_vel`을 입력한다.

## 13. 점검 명령

관제 PC:

```bash
source ~/colcon_ws/install/setup.bash
ROS_DOMAIN_ID=22 ros2 topic list | grep -E 'robot[12]|cmd_vel'
ROS_DOMAIN_ID=22 ros2 topic hz /robot1/amcl_pose
ROS_DOMAIN_ID=22 ros2 topic hz /robot2/amcl_pose
ROS_DOMAIN_ID=22 ros2 topic echo /robot1/fleet/heartbeat
ROS_DOMAIN_ID=22 ros2 topic echo /robot2/fleet/heartbeat
ROS_DOMAIN_ID=22 ros2 topic echo /robot1/fleet/permit
```

각 로봇 PC:

```bash
ros2 topic hz /fleet/pose
ros2 topic echo /fleet/heartbeat
ros2 topic echo /fleet/permit
ros2 topic echo /cmd_vel_candidate
ros2 topic echo /cmd_vel
```

정상 상태에서는 정지 중에도 `/fleet/pose`가 약 5 Hz로 계속 발행되어야 한다.

## 14. 확인된 테스트 상태

- 관제 패키지 테스트: 31 passed, 1 skipped (2026-09-19)
- Tk UI 생성 smoke test 통과
- 패키지에 설치된 `domain_bridge.yaml` 로딩 확인
- 로봇 safety 패키지 테스트: 7 passed
- `pinky_interfaces`, `pinky_led`, `pinky_fleet_safety`, `pinky_bringup` 빌드 확인

위 숫자는 기준일 당시 결과다. 코드 변경 후에는 다시 빌드와 테스트를 수행한다.

## 15. 현재 제약과 향후 개선 후보

- UI에서 그린 병목 사각형은 파일에 영구 저장되지 않는다.
- runtime UI 편집은 한 번에 병목 구역 하나만 유지한다. YAML은 여러 폴리곤을
  정의할 수 있다.
- 정적 거리 기반 제어이며 로봇 속도와 제동 거리에 따라 확장 경계를 자동 보정하지
  않는다.
- 병목 진입 순서의 공정한 대기열, 우선순위, starvation 방지 정책은 없다.
- 목적지 경로를 미리 예약하지 않고 실제 pose 진입을 기준으로만 소유권을 정한다.
- 병목 밖에서 발생하는 로봇 간 충돌은 이 기능의 제어 대상이 아니다.
- 구역이 벽이나 Nav2 장애물과 겹치는지 자동 검증하지 않는다.
- 깊게 진입한 후행 로봇을 후진시키는 별도 원격 복구 모드는 없다.
- 병목 소유자 pose stale은 전역 E-STOP이지만 최신 Nav2 목표 좌표는 유지되며 관제
  재시작 시 자동 재전송된다.
- `FleetPermit`에는 HOLD reason 문자열이 포함되지 않아 로봇 LED만으로는 병목 대기와
  heartbeat 문제를 구분할 수 없다. 상세 이유는 관제 UI에서 확인한다.
- robot ID는 로봇별 launch 파일에 하드코딩되어 있어 배포 시 값을 확인해야 한다.
- Nav2 액션은 domain bridge가 아닌 직접 DDS 연결이므로 관제 PC와 각 robot domain의
  discovery·방화벽·네트워크 통신이 가능해야 한다.
- 각 로봇에는 LED service server를 하나만 실행해야 한다.
