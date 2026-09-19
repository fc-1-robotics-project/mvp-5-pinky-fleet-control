# 코드 구조와 개발 가이드

기준일: 2026-09-19

이 문서는 `multibot_control_ui`를 처음 수정하는 팀원이 코드의 책임 경계와 주요
실행 흐름을 빠르게 파악하기 위한 문서다. 운영 중 보이는 동작과 안전 조건은
[현재 구현 사양](CURRENT_IMPLEMENTATION.md), 실행 명령은 [README](../README.md)를
먼저 참고한다.

## 1. 설계 요약

관제 프로세스는 UI, ROS 토픽 I/O, 병목 정책, Nav2 액션을 분리한다. 의존 방향은
다음과 같다.

```text
control_ui.py (화면·사용자 이벤트·프로세스 조립)
  ├── control_node.py (domain 22 토픽 I/O, freshness, permit 발행)
  ├── fleet_coordinator.py (RUN/HOLD/E-STOP과 병목 소유권 결정)
  │     └── zone_config.py (구역 모델과 2D 기하 계산)
  ├── navigation_client.py (domain 21/19 Nav2 액션 클라이언트)
  ├── robot_config.py (로봇별 정적 설정)
  └── map_math.py (map/grid/화면 좌표 계산에 쓰는 순수 함수)
```

중요한 원칙은 세 가지다.

1. UI는 안전 정책을 직접 결정하지 않는다. 버튼과 지도 이벤트를 coordinator에
   전달하고 결과를 표시한다.
2. coordinator는 ROS publisher를 직접 만들지 않는다. node의 메서드를 통해 permit
   상태를 바꾸고 navigation facade를 통해 goal을 관리한다.
3. 관제에서 `RUN`이어도 로봇의 velocity gate가 최종 명령 freshness를 다시 검사한다.
   모터용 `/cmd_vel`을 관제에서 직접 발행하지 않는다.

## 2. 시작과 종료 흐름

`control_ui.py:main()`이 composition root다.

```text
ros2 launch
  -> create_node()                         # domain 22, rclpy 기본 context
  -> FleetNavigationClients()             # robot별 context + executor thread
  -> load_zones(zone_config_file)          # enabled YAML 구역만 로드
  -> FleetCoordinator(...)                 # 시작 상태는 전체 HOLD
  -> MultiBotControlUI(...).run()          # Tk mainloop
```

Tk mainloop 안에서는 두 주기가 돈다.

- 20 ms: `rclpy.spin_once()`로 관제 node의 토픽 callback과 timer 처리
- 100 ms: coordinator `tick()`, 맵/마커/상태 라벨 갱신

Nav2 액션은 각 robot domain의 별도 executor thread가 처리한다. callback에서 공유하는
상태는 `RobotNavigationClient.lock`으로 보호한다. Tk widget은 Tk main thread에서만
접근한다.

창을 정상 종료하면 coordinator가 `UI_CLOSED` E-STOP permit을 즉시 발행하고 수동
속도를 0으로 만든 뒤 Nav2 client, node, rclpy context 순으로 정리한다. 프로세스 강제
종료처럼 마지막 permit을 보내지 못하는 상황은 로봇 gate의 permit TTL 만료가
fail-closed 정지를 담당한다.

## 3. 모듈별 책임

### `robot_config.py`

`ROBOTS`가 로봇 목록의 단일 진입점이다. 로봇 이름, domain, bridge 이후 관제 토픽,
Nav2 액션 이름, UI 초기 위치, 표시 색을 가진다. `ROBOT_BY_NAME`은 입력 검증과 빠른
조회에 사용한다.

로봇을 추가하거나 이름을 바꾸면 이 파일만 고쳐서는 안 된다. 다음 항목도 함께
수정해야 한다.

- `config/domain_bridge.yaml`의 map, pose, cmd_vel, permit, heartbeat, initialpose
- 로봇 측 `robot_id`와 `ROS_DOMAIN_ID`
- 2대만 가정한 화면 폭과 병목 정책 테스트

### `control_node.py`

domain 22에서 동작하는 ROS node다. 책임은 다음 네 종류다.

- 수신: robot별 map, pose, gate heartbeat
- 송신: robot별 수동 velocity, AMCL initial pose, `FleetPermit`
- 캐시: 최신 map/pose/heartbeat와 수신 시각
- 감시: 수동 명령 timeout, pose/heartbeat freshness

`set_gate_mode()`는 원하는 상태를 메모리에 기록하고, `publish_permits_now()`가
짧은 TTL을 가진 permit을 반복 발행한다. 상태가 바뀔 때만 sequence를 증가시키며,
같은 상태라도 TTL 갱신을 위해 메시지는 계속 발행한다.

두 로봇이 같은 맵을 쓴다는 전제 때문에 robot1의 맵을 우선한다. robot1 맵을 한 번
받은 뒤에는 robot2 맵 callback을 무시한다. 이 전제가 바뀌면 `_map_callback()`과 UI의
단일 `latest_map` 구조를 같이 수정해야 한다.

### `fleet_coordinator.py`

관제 정책의 중심이다. ROS 메시지 callback이나 Tk widget을 알지 않고, node와
navigation 객체의 작은 인터페이스만 사용하므로 단위 테스트에서 fake 객체로 검증할
수 있다.

소유하는 주요 상태는 다음과 같다.

| 상태 | 의미 |
|---|---|
| `zones` | 구역별 `FREE/OCCUPIED/CLEARING/CONFLICT/LOCKED`, owner, lease |
| `requests` | robot별 최신 Nav2 목적지와 전송 상태 |
| `enabled` | 관제 정책을 주기적으로 평가하는지 여부 |
| `emergency` | 운영자 또는 안전 조건에 의한 E-STOP 여부 |
| `manual_robot` | 공용 `/cmd_vel`의 현재 대상 |
| `blocked_robots` | 현재 `HOLD` permit을 받는 robot 집합 |

매 tick의 정책 순서는 고정되어 있다.

```text
Nav2 request 상태 정리/재시도
  -> 실제 pose로 구역 owner와 구역 상태 갱신
  -> owner pose stale이면 전체 E-STOP
  -> robot별 heartbeat/pose/타 구역 owner 경계 검사
  -> RUN 또는 HOLD permit 상태 기록
  -> 화면용 summary 갱신
```

구역 owner는 경로 예약이 아니라 실제 footprint 겹침으로 정한다. owner가 확장 경계를
나갈 때까지 lease를 유지한다. 나머지 robot만 owner 구역의 확장 경계 안에서 HOLD한다.

### `navigation_client.py`

Nav2 액션은 domain bridge를 거치지 않는다. `RobotNavigationClient` 하나가 robot 하나의
ROS context, node, action client, executor thread를 소유한다. UI thread를 막지 않도록
goal 전송·취소·결과 수신은 모두 비동기다.

`_goal_generation`은 이전 goal의 늦은 callback이 최신 goal 상태를 덮어쓰지 못하게
한다. callback을 추가할 때도 generation 확인을 유지해야 한다. 화면 문구는 `status()`,
coordinator 판단은 안정된 영문 상태값을 돌려주는 `state()`를 사용한다.

### `control_ui.py`

UI 코드는 역할에 따라 다음 묶음으로 읽으면 된다.

- `_build_*_panel()`: widget 생성과 배치
- `_select_robot()`부터 `_clear_zones()`: 버튼 이벤트
- `_on_goal_*()`, `_canvas_to_world()`: 지도 drag 입력
- `_poll_ros()`, `_refresh_ui()`: Tk와 ROS 주기 연결
- `_render_map()`, `_scale_map_to_canvas()`, `_draw_robot_markers()`: 지도와 overlay
- `_refresh_*_status()`: 화면 문구 갱신
- `_on_close()`: 안전 종료

지도에서 목표를 drag하면 시작점이 목적지 `(x, y)`, drag 방향이 yaw가 된다. 병목 편집
모드에서는 같은 pointer event를 사각형 두 모서리 입력으로 사용한다.

### `zone_config.py`와 `map_math.py`

두 파일은 ROS node나 Tk 상태에 의존하지 않는 계산 코드다. `zone_config.py`는 YAML
검증, 사각형 생성, point/polygon/segment 거리 계산을 담당한다. `map_math.py`는 회전된
occupancy grid origin까지 고려해 world와 grid 좌표를 양방향 변환한다.

기하 또는 좌표 변환을 바꿀 때는 UI에서 먼저 시험하지 말고 순수 함수 테스트를 추가한
뒤 overlay를 확인하는 편이 안전하다.

## 4. 주요 사용자 흐름

### 수동 주행 선택

```text
robot 버튼
  -> node.select_robot()                  # 이전 robot에 zero Twist
  -> coordinator.select_manual_robot()
       -> 필요하면 관제 start
       -> heartbeat/pose freshness 확인
       -> 기존 Nav2 goal 취소·삭제
       -> motion policy 평가
  -> RUN일 때만 node가 공용 /cmd_vel을 robot 토픽으로 전달
```

node와 robot-local gate가 모두 RUN을 확인하므로, UI 선택만으로 모터 출력이 허용되지는
않는다.

### Nav2 목표 전송

```text
지도 drag
  -> canvas 좌표를 map 좌표와 yaw로 변환
  -> coordinator.submit_goal()
       -> 최신 목적지 저장
       -> domain별 navigation client에 비동기 전송
       -> 병목/통신 상태에 맞춰 permit 재평가
  -> 수동 command target 해제
```

Nav2 server가 준비되지 않았으면 목적지를 보관하고 1초 간격으로 재전송한다. 병목 HOLD는
goal을 취소하지 않고 velocity gate만 닫는다.

### 병목 통과

```text
pose가 병목 + robot radius에 처음 겹침
  -> owner와 lease 생성
  -> owner RUN
  -> 다른 robot이 확장 경계에 닿으면 그 robot만 HOLD
  -> owner가 확장 경계를 완전히 이탈
  -> owner 해제, 대기 robot이 겹쳐 있으면 새 owner로 인계
```

### E-STOP 복구

E-STOP은 활성 Nav2 goal을 취소하지만 목적지 좌표는 `requests`에 남긴다. 원인을 해소한
후 `관제 시작 / 상태 재확인`을 누르면 occupancy와 freshness를 다시 평가하고 저장된
목표를 전송한다. 운영자가 목표 자체를 버리려면 robot을 선택하고 목표 취소를 사용한다.

## 5. 설정 위치

| 변경 대상 | 수정 위치 |
|---|---|
| 로봇 이름/domain/토픽/색상/초기 pose | `multibot_control_ui/robot_config.py` |
| 기본 timeout, permit 주기, runtime 여유 거리 | `launch/control_ui.launch.py` |
| parameter 선언과 유효성 검사 | `multibot_control_ui/control_node.py` |
| 시작 시 영구 병목 | `config/bottleneck_zones.yaml` |
| domain 간 topic remap과 QoS | `config/domain_bridge.yaml` |
| 병목 소유권과 HOLD/E-STOP 정책 | `multibot_control_ui/fleet_coordinator.py` |
| UI 문구·배치·지도 표현 | `multibot_control_ui/control_ui.py` |

launch 기본값과 node의 parameter 기본값은 launch 없이 직접 실행할 때도 같은 동작을
하도록 함께 유지한다. 문서의 파라미터 표도 같이 갱신한다.

## 6. 테스트와 변경 원칙

빠른 단위/정적 검사는 패키지 루트에서 실행한다.

```bash
pytest -q
```

테스트 범위는 다음과 같다.

- `test_map_math.py`: quaternion, world/grid 좌표, occupancy 색
- `test_zone_config.py`: YAML/사각형 구역과 기하 판정
- `test_fleet_coordinator.py`: 병목 owner, HOLD, stale, E-STOP, goal 재전송
- `test_flake8.py`, `test_pep257.py`, `test_copyright.py`: ROS 2 Python 품질 검사

ROS 통합 확인은 별도로 필요하다. 특히 다음은 단위 테스트만으로 보장되지 않는다.

- 세 domain 사이 DDS discovery와 bridge QoS
- Nav2 action server 연결·goal 수락·취소
- map/pose frame 일치
- robot-local velocity gate와 LED 동작
- 실제 제동 거리와 병목 여유 거리

기능을 바꿀 때는 정책을 UI callback에 직접 추가하기보다 coordinator 또는 순수 계산
모듈에 넣고 단위 테스트를 먼저 추가한다. 새로운 모터 명령 소스도 최종 `/cmd_vel`이
아니라 반드시 `/cmd_vel_candidate`로 연결한다.
