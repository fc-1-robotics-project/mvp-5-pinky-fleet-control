# 관제 코드 구조

기준일: **2026-10-06**, 작업 브랜치: `codex/two-robot-demo-20261005`, 병합 대상: 팀 `develop`.
설치·실행은 [팀 가이드](../../TEAM_LANE_GUIDE.md) / [UI 사용법](../../UI_INTEGRATION.md),
시연 정책과 미해결 문제는 [통합 시연 가이드](TWO_ROBOT_DEMO.md)를 확인한다.

## 모듈과 책임

| 코드 | 역할 |
|---|---|
| `control_ui.py`, `demo_panel.py` | 메인 실행 버튼·지도 입력·별도 시연 설정 창·진행 표시 |
| `demo_mission.py` | A/B 단계, waypoint 목록, 완료 조건, 시작 검사, 재검사·재시도 |
| `fleet_coordinator.py`, `zone_config.py` | 임무 요청·모드 선택·permit 정책·병목 소유권 |
| `control_node.py`, `navigation_client.py` | domain 22 토픽 I/O 및 로봇 domain별 Nav2 액션 |
| `vision_control/lane_client.py` | 로봇 domain별 FollowLane·임무 상태·AMCL 위치 검증 |

`robot_config.py`는 로봇 ID/domain 등록, `map_math.py`는 픽셀↔map 변환을 담당한다.
`lane_routes.py`와 예전 연속 임무 API는 호환용으로 남아 있지만 옛 세 좌표 입력 UI는 제거했다.
새 시연 설정은 `two_robot_demo.json`으로 분리한다.

## ROS 연결과 명령 경로

```text
Tk UI -> FleetCoordinator -> TwoRobotDemo (진행 상태)
                 |
                 +-> 로봇별 Nav2/FollowLane 액션 (각 domain에 직접 연결)
                 +-> drive mode 요청 + fleet permit (domain bridge)

로봇: Nav2 / lane_watchdog / manual 후보 명령
                    -> drive_mode_mux
                    -> fleet_velocity_gate
                    -> /cmd_vel -> 모터
```

`NavigateToPose`와 `FollowLane` 액션은 domain bridge를 통하지 않는다.
map·fleet pose·heartbeat·permit·initial pose·mode 요청·차선 완료 Bool은 bridge를 통한다.
차선 임무 상태는 per-domain 차선 클라이언트에서 직접 구독한다.
UI는 모터 `/cmd_vel`을 직접 발행하지 않으며 영상 추론은 로봇에서 수행한다.

Tk thread는 ROS callback과 정책·화면 갱신을 처리한다. 로봇 domain 액션은 별도
executor/context를 사용하고 공유 상태는 클라이언트 lock으로 읽는다.
Tk 위젯은 Tk thread에서만 갱신한다.

## 시연 상태와 완료 처리

```text
A_LANE (B 대기)
  -> NAV_ROUTES (A/B 각 waypoint 순서대로, 병목 정책 적용)
  -> FINAL (A 최종 Nav2, B 차선 입구 Nav2)
  -> B_LANE (B 차선 주행, A 목표가 아직 진행 중이면 계속 검사)
  -> COMPLETE (두 역할 모두 완료, 전체 HOLD)
```

- `validate_plan`은 지도 식별값·서로 다른 A/B·네 고정 pose·방향·목록 확정 값을 검사한다.
- 빈 waypoint 목록도 확정되어 있으면 완료로 인정한다. 두 역할 목록을 모두 마쳐야 FINAL로 간다.
- Nav2 단계는 **NavigateToPose 성공**으로 완료한다. 관제에서 거리·yaw 오차를 재검사하지 않는다.
- 차선은 유효한 AMCL 끝 반경 **20cm 안에서 0.5초** 도착이 유지되면 기존 종료 Bool을 보낸다.
  종료 yaw·차선 소실·3초 정지는 필수 조건이 아니다. 최신 센서와 허가 조건은 유지한다.
- 종료 후 FollowLane 성공·STOP·로컬 허가 OFF 정리를 확인한다. AMCL 초기 위치를 덮어쓰지 않는다.

## 오류 처리와 현재 제한

`wait_for_recovery`는 단계·목표·목록·병목 소유권을 보존한다.
현재 구현은 `sequence.recovering` 동안 **두 로봇 모두 STOP/HOLD**로 만든다.
두 로봇 상태가 1초 정상이고 마지막 시도에서 3초가 지났으면 `_recover`가 모든 역할을 검사한다.
완료한 Nav2 목표는 재전송하지 않고, 종료된 실패 액션의 현재 목표만 재시도한다.
운영자 pause는 명시적인 재개를 기다리고 cancel/ESTOP은 자동 재시도하지 않는다.

**미해결:** B 차선이 시작 준비 상태로 돌아오지 않으면 A 최종 Nav2도 계속 HOLD된다.
또한 HOLD 중 살아 있는 Nav2 액션을 취소하지 않아 progress checker가 실패할 수 있다.
로봇별 복구 분리와 긴 의도적 HOLD에서 목표 보존·액션 종료·재전송은 아직 구현되지 않았다.
해당 동작을 완료된 기능으로 문서화하거나 모의 테스트 통과로 현장 검증을 대신하지 않는다.

## UI 입력과 종료

지도 선택은 입력란만 채운다. AMCL은 `초기 위치 적용`, 시연 pose는 `좌표 반영`,
waypoint는 `대기점 추가`로 확정하고 재실행용 설정은 `설정 저장`한다.
선택 모드 밖 일반 드래그는 Nav2 목표를 즉시 보낸다.
지도 크기·대상 변경은 진행 중 드래그를 무효화한다.

`control_ui.launch.py`는 UI와 bridge를 하나씩 시작한다.
UI 종료 시 임무·permit·클라이언트·ROS context를 정리하고 함께 시작한 bridge도 종료한다.
강제 종료나 통신 단절에는 로봇 permit/로컬 허가 TTL이 적용된다.

## 변경 시 함께 확인할 코드

| 변경 | 확인 대상 |
|---|---|
| 시연 전환·복구 | `demo_mission.py`, coordinator 정책, `test_demo_mission.py` |
| 시작·AMCL 기준 | demo 상수, lane client `localization_status`, 상태 검사 테스트 |
| 로봇 ID/domain | `robot_config.py`, bridge YAML, 로봇 실행 인자 |
| 차선 속도·기능 | 실제 운용 JSON, 로봇 launch watchdog 상한, mission server 기대값 |
| 완료·취소·종료 | 액션 terminal 상태, STOP·허가 OFF, UI signal/context 정리 |

모델·현장 영상·자격증명·빌드 산출물은 커밋하지 않는다.
현장 지도 `261003.yaml/pgm`은 로봇 저장소에 포함되어 있으며 실제 장소에 맞는지 별도 확인한다.
