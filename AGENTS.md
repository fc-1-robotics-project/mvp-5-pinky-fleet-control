# AGENTS.md

This file provides guidance to coding agents (Claude Code, Codex) working in this repository. `CLAUDE.md` is a symlink to it.

## 개요

Pinky 로봇 플릿의 **관제 PC 쪽** ROS 2 Jazzy 패키지 모음 (Ubuntu 24.04, Python 3.12). Tk UI 하나로 Nav2 이동, 차선 연속 임무, 차선 단독 시험, 병목 구역 관제를 한다. 로봇 쪽 소스(임무 서버, watchdog, velocity gate, YOLO 추론)는 별도 저장소 `jsh0116/pinky-lane-driving`에 있으며, **두 저장소의 같은 브랜치(`codex/lane-field-20261001`)를 함께 빌드·업데이트**해야 한다. 문서는 한국어다.

패키지 3개 (colcon workspace의 `src/` 아래에 이 저장소를 clone):
- `pinky_interfaces` — ament_cmake. `FollowLane.action`, `FleetPermit`/`RobotHeartbeat` msg, 램프·LED 등 srv. 로봇 저장소에도 같은 이름의 패키지가 있으므로 **같은 workspace에 두 저장소를 같이 clone하면 중복**된다. 인터페이스를 바꾸면 양쪽을 함께 수정·빌드한다.
- `vision_control` — 로봇 domain별 `FollowLane` 액션 클라이언트·상태 수신 (`lane_client.py`).
- `multibot_control_ui` — UI·Nav2 client·모드/병목 정책.

## 빌드·테스트

```bash
cd ~/colcon_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to multibot_control_ui
source install/setup.bash
ros2 launch multibot_control_ui control_ui.launch.py   # GUI 세션 필요, domain_bridge 동반 실행 (start_bridge:=false로 끔)
```

하드웨어 없이 도는 동작 테스트 (TEAM_LANE_GUIDE.md §7, 저장소 루트에서 실행):

```bash
PYTHONPATH="$PWD/multibot_control_ui:$PWD/vision_control:${PYTHONPATH:-}" \
python3 -m pytest -q multibot_control_ui/test/test_fleet_coordinator.py   # 단일 파일
# -k <이름> 으로 단일 테스트
```

`test_copyright/flake8/pep257.py`는 ament 린트 테스트이고, 위 가이드의 73개 동작 검사 목록에는 포함되지 않는다. 빌드·동작 검사·실주행 검증은 각각 구분해서 기록한다 (0.06m/s 실주행은 미검증).

## 아키텍처 (여러 파일을 읽어야 보이는 부분)

**domain 분리**: robot1=21, robot2=19, 관제=22. 두 경로로 로봇과 통신한다.
1. `config/domain_bridge.yaml` (domain_bridge) — map/pose/permit/heartbeat/모드 요청 토픽. 같은 원본 토픽 키가 로봇 domain별로 반복되므로 일반 dict로 합치면 안 된다.
2. 로봇별 별도 rclpy context/executor로 직접 연결하는 액션 클라이언트 — `/navigate_to_pose`(`navigation_client.py`)와 `/follow_lane`(`vision_control/lane_client.py`).

**모듈 책임**:
- `control_ui.py` (Tk, 가장 큼) — 화면·지도 이벤트·`main()` 조립. Tk main thread에서 20ms마다 관제 ROS callback, 100ms마다 정책+화면 갱신. 위젯은 Tk thread에서만 갱신, 클라이언트 공유 상태는 lock 경유.
- `control_node.py` — domain 22 토픽 I/O와 permit 발행.
- `fleet_coordinator.py` — 병목 소유권, Nav2 입구 도착 → 차선 액션 → 출구 확인 → 다음 Nav2 목표로 이어지는 임무 진행, HOLD/취소. **안전 정책은 UI callback이 아니라 여기에 구현하고 단위 테스트를 추가**한다.
- `lane_routes.py` — 방향별(로봇별 아님) 입구·출구·다음 목표 검증·저장 (`~/.config/pinky_fleet_control/lane_routes.json`). `validate_route`는 세 항목 모두 요구.
- `robot_config.py`, `zone_config.py`(`config/bottleneck_zones.yaml`), `map_math.py`(회전된 map 원점 포함 좌표 변환).

**안전 불변식**:
- 관제는 최종 모터 토픽 `/cmd_vel`을 직접 발행하지 않는다. 명령은 로봇의 `/cmd_vel_candidate` → velocity gate를 거치며, 관제는 단명(ttl) `FleetPermit`을 반복 발행해 허가한다. 시작 관제 상태는 HOLD.
- 임무 완료 후에는 취소/정지와 permit 해제를 확인한다. UI 종료 시 임무·permit 정리, 클라이언트·ROS context close (닫힌 context에 spin 금지).
- 지도 선택 모드(AMCL/경로 입력)는 입력란만 채우고 `설정`/`지정`으로 확정한다. 모드가 없을 때의 지도 드래그는 Nav2 목표 전송. 드래그 중 대상 로봇/지도 변환이 바뀌면 진행 중 드래그를 폐기한다 (`test_ui_map_capture.py`).

**함께 수정해야 하는 변경**:
| 변경 | 같이 확인 |
|---|---|
| 로봇 이름/domain/토픽 | `robot_config.py` + `config/domain_bridge.yaml` + 로봇 실행 인자 (관제 domain 22는 launch·bridge에도 있음) |
| launch 파라미터 기본값 | `control_node.py`의 직접 실행 기본값 + 문서 |
| 액션/서비스 정의 | 양쪽 저장소 `pinky_interfaces`, 로봇 임무 서버, `vision_control` |

## 규칙

- `main`에서 직접 개발하지 않고 작업별 브랜치(`feat/`, `fix/`, `docs/`, `refactor/`, `test/`). PR 하나에 한 목적. 안전 동작·토픽 구성 변경 PR은 팀원 1명 이상 리뷰.
- 공유 브랜치에 모델, 현장 영상, 지도, 자격증명, 빌드 산출물을 추가하지 않는다.
- 상세 문서: `TEAM_LANE_GUIDE.md`(설치·업데이트·문제 확인), `UI_INTEGRATION.md`(실행·UI 사용), `multibot_control_ui/docs/{ARCHITECTURE,CURRENT_IMPLEMENTATION}.md`.
