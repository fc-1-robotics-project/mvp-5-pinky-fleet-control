# multibot_control_ui

ROS 2 Jazzy의 Nav2·차선 임무·병목 관제 UI. 공유 기준은 **`codex/lane-field-20261001` / 2026-10-03**입니다.

## 설치와 실행

새 PC는 [팀 설치 가이드](../TEAM_LANE_GUIDE.md), 로봇과 함께 사용하는 순서는 [공통 실행·UI 가이드](../UI_INTEGRATION.md)를 따릅니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
ros2 launch multibot_control_ui control_ui.launch.py
```

launch가 UI와 domain_bridge를 하나씩 실행합니다. 외부 브리지를 따로 운용할 때만 `start_bridge:=false`를 추가합니다. UI 창이 닫히거나 브리지가 종료되면 같은 launch도 종료됩니다.

## 자주 쓰는 UI 기능

| 기능 | 조작 |
|---|---|
| 패널 폭 | 좌우 경계선을 드래그, 아래 제어는 세로 스크롤 |
| AMCL 초기 위치 | 로봇 행 `지도 선택` → 위치/방향 드래그 → `설정` |
| 통합 시연 좌표·waypoint | `두 로봇 통합 시연 · waypoint 설정` → `지도 선택` → 드래그 → `지정`/`추가` |
| 지도 선택 종료 | `선택 취소 (Esc)`; 일반 지도 드래그로 복귀 |
| 차선 시험 | 현장 준비 확인 → `차선 단독 테스트` |
| 두 로봇 연속 주행 | 통합 시연 창에서 A/B 설정 → 준비 확인 → `통합 시연 시작` |
| 수동 차선 완료 | `차선 구간 완료`; 통합 시연은 자동 종료 조건도 사용 |
| 잠시 정지 / 취소 | `일시정지 / 재개` / `선택 임무 중단` |
| 관제 HOLD / 긴급 정지 | `관제 일시정지` / `전체 정지 / 선택 해제` |

지도 선택 모드는 입력란만 채웁니다. 모드 밖에서 로봇 선택 후 지도 클릭·10px 이상 드래그하면 Nav2 목표가 전송됩니다. 입력 단위는 m/도, map 좌표계입니다. 초기 위치 기본 숫자는 실제 배치를 확인한 값이 아닙니다.

## 패키지 구성

| 파일 | 담당 |
|---|---|
| `control_ui.py` | Tk 레이아웃·지도 이벤트·버튼·주기 갱신·종료 |
| `control_node.py` | domain 22 토픽·permit·명령 라우팅 |
| `fleet_coordinator.py` | 병목 소유권·임무 진행·HOLD/취소·모드 전환 |
| `navigation_client.py` | 로봇 domain별 Nav2 액션 |
| `lane_routes.py` | 방향별 좌표 검사·저장 |
| `robot_config.py`, `zone_config.py`, `map_math.py` | 로봇 등록·구역·좌표 계산 |
| `../vision_control/vision_control/lane_client.py` | 차선 액션·텔레메트리·AMCL 전환 |

## domain과 저장 위치

robot1=21, robot2=19, 관제=22. 로봇별 `/navigate_to_pose`와 `/follow_lane` 액션은 별도 context로 직접 연결합니다. map/pose/permit/모드 요청 등은 `config/domain_bridge.yaml`로 브리지합니다. 로봇 ID/domain을 바꾸면 이 파일과 `robot_config.py`, 로봇 실행 인자를 함께 맞춥니다.

통합 시연 설정은 `~/.config/pinky_fleet_control/two_robot_demo.json`에 저장됩니다. 기존 3개 좌표 연속 임무 입력은 UI에서 제거했습니다. 예전 `lane_routes.json` 파일은 보존합니다. 병목 영구 설정은 `config/bottleneck_zones.yaml`; UI에서 지정한 사각형은 현재 실행에만 유지됩니다.

현재 로봇 차선 설정은 0.06m/s·횡단보도 정지 OFF·차선 라이다 물체 정지 OFF입니다. 자세한 설정과 검증 범위는 공통 실행 가이드에 있습니다.

## 개발

빌드와 동작 테스트 명령은 [팀 가이드](../TEAM_LANE_GUIDE.md#7-코드-검사검증-범위)에 있습니다. 모듈 관계는 [ARCHITECTURE.md](docs/ARCHITECTURE.md), 실제 명령 경로와 제한은 [CURRENT_IMPLEMENTATION.md](docs/CURRENT_IMPLEMENTATION.md)를 참고합니다.

## 두 로봇 차선↔Nav2 통합 시연

[설정·waypoint 편집·자동 종료 사용법](docs/TWO_ROBOT_DEMO.md)을 참고하세요.
