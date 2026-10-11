# 관제 코드 구조

기준일: **2026-10-09**, 작업 브랜치: `feature/mission-recovery-position-20261009`, 기준: `origin/develop`.
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
  x/y 분산은 각각 `lane_exit_position_variance`(기본 0.0025m²) 이하로 제한한다. 시연 시작의 0.30m²와 구분한다.
  종료 yaw·차선 소실·3초 정지는 필수 조건이 아니다. 최신 센서와 허가 조건은 유지한다.
- 종료 후 FollowLane 성공·STOP·로컬 허가 OFF 정리를 확인한다. AMCL 초기 위치를 덮어쓰지 않는다.

## 오류 처리

`wait_for_recovery`는 단계·목표·목록·병목 소유권을 보존하고 `sequence.recovering`에 로봇별 복구 상태를 저장한다.
해당 로봇만 STOP/HOLD하며 정상 로봇의 요청 갱신·완료·다음 waypoint 진행은 기존 병목 정책 안에서 계속 처리한다.
해당 로봇 상태가 1초 정상이고 마지막 시도에서 3초가 지났으면 `_recover`가 그 역할만 검사한다.
병목 소유자의 위치가 오래되어 점유가 불확실하면 공유 안전 오류로 두 로봇을 HOLD하고 기존 소유자·lease를 유지한다.
소유자 위치가 다시 유효해지기 전에는 복구를 진행하지 않는다.

복구 HOLD가 3초 이상 지속되면 해당 Nav2 액션 취소를 요청하고 현재 목표를 보존한다.
취소 요청 승인만으로 재전송하지 않으며, 취소 중 오류가 나도 종료 결과 확인 전에는 새 목표를 보내지 않는다.
복구 중 성공한 목표는 완료 처리하고, 종료된 실패·취소 목표만 해당 로봇에 재전송한다.
늦게 도착한 취소 응답이나 오류가 이미 확정된 도착 성공을 덮어쓰지 않도록 액션 세대와 현재 핸들을 확인한다.
운영자 전체 pause는 두 로봇을 HOLD하고 명시적인 재개를 기다리며 cancel/ESTOP은 자동 재시도하지 않는다.

`lane_client.py`는 로봇 상태와 `/fleet/pose`의 원본 시각에 최대 100ms 앞섬을 허용한다.
수신 지연과 원본의 과거 지연은 각각 1.5초 이내여야 하며, 더 큰 시계 차이·지연·비정상 값은 거부한다.
최신 `origin/develop`의 `7a84a49` 위에서 기능 테스트 231개 통과(lint 3개 제외)와 실제 PC 작업공간 3개 패키지 빌드를 확인했다. UI·주행 노드를 실행하지 않은 오프라인 검증이며, 이전 전체 복구 대기의 현장 문제와 현재의 실제 전체 완주 미확인 상태는 [통합 시연 가이드](TWO_ROBOT_DEMO.md)에 구분해 기록한다.

운영자·병목 HOLD는 coordinator가 목표를 보존하고 취소하며, 시연 건강 오류의 3초 지연 취소는 `TwoRobotDemo`가 담당한다. 수동 선택 중 자동 목표 재전송은 보류한다. TF 원본 시각과 수신 시각 중 오래된 값으로 관제 위치 신선도를 판단한다. [수정 기록](MISSION_RECOVERY_FIXES.md)에 상세 검증 범위를 기록한다.

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
| 시연 전환·복구 | `demo_mission.py`, coordinator 정책, `test_demo_mission.py`, `test_demo_recovery.py` |
| 시작·AMCL 기준 | demo 상수, lane client `localization_status`, 상태 검사 테스트 |
| 로봇 ID/domain | `robot_config.py`, bridge YAML, 로봇 실행 인자 |
| 차선 속도·기능 | 실제 운용 JSON, 로봇 launch watchdog 상한, mission server 기대값 |
| 완료·취소·종료 | 액션 terminal 상태, STOP·허가 OFF, UI signal/context 정리 |

모델·현장 영상·자격증명·빌드 산출물은 커밋하지 않는다.
현장 지도 `261003.yaml/pgm`은 로봇 저장소에 포함되어 있으며 실제 장소에 맞는지 별도 확인한다.
