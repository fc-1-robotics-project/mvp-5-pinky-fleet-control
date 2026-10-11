# multibot_control_ui

ROS 2 Jazzy의 Nav2·차선 임무·병목 관제 UI. 현재 작업 브랜치는 **`feature/mission-recovery-position-20261009`**입니다.

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

| 기능 | 조작 / 실제 동작 |
|---|---|
| 패널 폭 | 좌우 경계선을 드래그, 아래 제어는 세로 스크롤 |
| AMCL 초기 위치 | 로봇 행 `지도 선택` → 위치/방향 드래그 → `초기 위치 적용`. 진행 임무·수동 조작·시연 중에는 적용 불가 |
| 통합 시연 설정 | 별도 설정 창에서 `지도 선택` → 드래그 → `좌표 반영` 또는 `대기점 추가` → `설정 저장` |
| 통합 시연 실행 | 메인 UI에서 현장 확인 → `통합 시연 시작`. `시연 일시정지`/`시연 재개`는 두 로봇 전체에 적용 |
| 지도 선택 종료 | `좌표 선택 취소 (Esc)`는 입력 모드만 해제하며 주행을 취소하지 않음 |
| 개별/전체 차선 시험 | 현장 준비 확인 → `선택 로봇 차선 시험 시작` / `전체 차선 시험 시작`. 전체 시험은 모든 로봇이 준비됐을 때만 시작 |
| 수동 차선 완료 | `선택 로봇 차선 완료`는 선택한 차선 임무를 정상 종료. 시연에서는 다음 단계로 이어질 수 있음 |
| 선택 임무 정지/재개 | `선택 임무 일시정지` ↔ `선택 임무 재개`. 전역 HOLD는 별도로 해제해야 함 |
| 임무 취소 | `선택 임무 취소`. 시연 참여 로봇 선택 시 `통합 시연 중단 (두 대)`로 표시하며 두 임무 모두 취소 |
| 수동 조작 | `선택 로봇 수동 조작` ↔ `선택 로봇 자율주행 복귀`. 수동 전환 시 전역 관제도 RUN이 되며 기존 자율 임무는 유지 |
| 관제 RUN/HOLD | `관제 시작 / 재개 (RUN)`은 주행 허가, `전체 일시정지 (HOLD)`는 목표 유지·전체 정지. 이미 RUN일 때 재실행 불가 |
| 비상정지 | `전체 비상정지`. 해제는 `비상정지 해제 · 관제 RUN`. 개별 Nav2 목표는 재개될 수 있고 차선 시험/시연은 새로 시작해야 함 |
| 병목 구역 | `병목 구역 지정 / 교체`는 현재 구역을 드래그한 사각형 하나로 교체. `병목 구역 삭제`는 모든 병목 구역을 제거. 관제 정지 중에만 편집 |

시연 중 개별 시험·수동 전환·초기 위치 적용·개별 일시정지는 비활성화됩니다. 시연 영역의 정지/재개 버튼을 사용하세요. 비상정지 상태는 일시정지나 임무 취소로 해제되지 않습니다.

지도 선택 모드는 입력란만 채웁니다. 모드 밖에서 로봇 선택 후 지도 클릭·10px 이상 드래그하면 Nav2 목표가 전송됩니다. 입력 단위는 m/도, map 좌표계입니다. 초기 위치 기본 숫자는 실제 배치를 확인한 값이 아닙니다.

## 패키지 구성

| 파일 | 담당 |
|---|---|
| `control_ui.py` | Tk 레이아웃·지도 이벤트·버튼·주기 갱신·종료 |
| `control_node.py` | domain 22 토픽·permit·명령 라우팅 |
| `demo_mission.py`, `demo_panel.py` | 두 로봇 시연 상태·waypoint·완료/복구 검사·별도 편집 창 |
| `fleet_coordinator.py` | 병목 소유권·임무 진행·HOLD/취소·모드 전환 |
| `navigation_client.py` | 로봇 domain별 Nav2 액션 |
| `lane_routes.py` | 방향별 좌표 검사·저장 |
| `robot_config.py`, `zone_config.py`, `map_math.py` | 로봇 등록·구역·좌표 계산 |
| `../vision_control/vision_control/lane_client.py` | 차선 액션·텔레메트리·AMCL 전환 |

## domain과 저장 위치

robot1=21, robot2=19, 관제=22. 로봇별 `/navigate_to_pose`와 `/follow_lane` 액션은 별도 context로 직접 연결합니다. map/pose/permit/모드 요청 등은 `config/domain_bridge.yaml`로 브리지합니다. 로봇 ID/domain을 바꾸면 이 파일과 `robot_config.py`, 로봇 실행 인자를 함께 맞춥니다.

통합 시연 설정은 `~/.config/pinky_fleet_control/two_robot_demo.json`에 저장됩니다. 기존 3개 좌표 연속 임무 입력은 UI에서 제거했습니다. 예전 `lane_routes.json` 파일은 보존합니다. 병목 영구 설정은 `config/bottleneck_zones.yaml`; UI에서 지정한 사각형은 현재 실행에만 유지됩니다.

현재 저장소의 기본 차선 설정은 cruise/max/fallback 0.09m/s, blind/approach 0.06m/s·횡단보도 정지 ON·라이다 물체 정지 ON입니다. 횡단보도는 정지 명령과 odom 정지 확인 후 안전 조건이 유지되면 2초 WAIT 뒤 최대 0.06m/s로 재출발합니다. 기체의 실제 운용 JSON에서 `behavior.crosswalk_stop=true`와 `wait_s=2.0`, 업데이트된 설치 코드 사용 여부를 각각 확인합니다. 자세한 설정과 기체별 실행 경로·검증 범위는 공통 실행 가이드에 있습니다.

## 개발

빌드와 동작 테스트 명령은 [팀 가이드](../TEAM_LANE_GUIDE.md#7-코드-검사검증-범위)에 있습니다. 모듈 관계는 [ARCHITECTURE.md](docs/ARCHITECTURE.md), 실제 명령 경로와 제한은 [CURRENT_IMPLEMENTATION.md](docs/CURRENT_IMPLEMENTATION.md)를 참고합니다.

## 두 로봇 차선↔Nav2 통합 시연

[설정·waypoint 편집·자동 종료 사용법](docs/TWO_ROBOT_DEMO.md)을 참고하세요. 현재 전체 복구 대기가 다른 로봇의 최종 Nav2까지 멈추는 문제가 남아 있으며 전체 완주는 미확인입니다.
