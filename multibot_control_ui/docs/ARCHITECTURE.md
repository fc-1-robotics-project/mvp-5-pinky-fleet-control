# 코드 구조와 개발 가이드

기준일: **2026-10-03**, 공유 브랜치: `codex/lane-field-20261001`.
[현재 구현 상태](CURRENT_IMPLEMENTATION.md)와 [실행·UI 사용법](../../UI_INTEGRATION.md)을 함께 봅니다.

## 구성 요소

```text
control_ui.py                         화면, 지도 입력, main 조립
  ├── control_node.py                 domain 22 토픽 I/O, permit
  ├── fleet_coordinator.py            병목 정책, Nav2↔차선 임무 진행
  │     └── zone_config.py            구역과 경계 계산
  ├── navigation_client.py            로봇별 Nav2 context / executor
  ├── vision_control/lane_client.py   로봇별 FollowLane / 상태 / 출구 위치
  ├── lane_routes.py                  방향별 경로 유효성·저장
  ├── robot_config.py                 이름·domain·토픽·초기 표시값
  └── map_math.py                     회전 지도를 포함한 좌표 변환

로봇 저장소
  ├── lane_mission_server.py          로컬 임무·허가 갱신·정리
  ├── mission_guard.py                준비·센서·속도·이동 감시
  ├── lane_control / lane_watchdog    경로 기반 속도 제안·유효성 검사
  └── drive_mode_mux / velocity_gate 최종 주행 소스 선택·permit 적용
```

## 책임 경계

UI는 숫자 입력·지도 이벤트·상태 표시를 담당합니다. 임무 시작/취소는 coordinator를 통합니다. ROS 토픽과 액션 I/O는 node/client가 담당합니다. 카메라 추론·차선 경로·속도 제어는 로봇 측에 있습니다. 관제에 YOLO를 추가하거나 원본 영상을 전송할 필요가 없습니다.

Tk main thread는 20ms 주기로 관제 ROS callback을 처리하고 100ms 주기로 정책과 화면을 갱신합니다. 로봇 도메인의 액션은 별도 executor/context에서 처리합니다. 공유 상태는 클라이언트의 lock을 통해 접근하고 Tk 위젯은 Tk thread에서 갱신합니다.

## 시작·종료

`control_ui.launch.py`가 domain_bridge와 UI를 만들며 둘 중 필수 프로세스가 종료되면 launch를 닫습니다. `start_bridge:=false`는 외부 브리지를 쓰는 구성에만 적용합니다.

`main()`은 node·Nav2 clients·lane clients·coordinator·Tk UI를 조립합니다. 시작 관제 상태는 HOLD입니다. UI 종료는 임무/permit을 정리하고 클라이언트와 ROS context를 닫습니다. `_poll_ros`, `_refresh_ui`는 종료 요청/ROS context 상태를 확인해 닫힌 context에 spin하지 않습니다.

## 좌표 입력 흐름

1. AMCL/경로의 `지도 선택`이 `(종류, 대상)` 입력 모드를 설정합니다.
2. 마우스 press에서 대상·지도 위치를 고정하고 drag로 방향을 표시합니다.
3. release에서 map 원점 회전을 반영해 x/y/yaw를 해당 입력란에 채웁니다. 선택 모드는 반복 수정할 수 있도록 유지됩니다.
4. `설정`은 AMCL 전송, `지정`은 해당 경로 항목 저장을 수행합니다. Esc·방향 변경·대상 변경은 진행 중 선택을 정리합니다.
5. 선택 모드가 없을 때의 드래그는 기존 Nav2 목표 동작입니다.

드래그 도중 로봇 선택이 바뀌어도 처음 정한 대상이 바뀌지 않습니다. 지도 resize/update로 픽셀 변환이 바뀌면 진행 중 드래그를 폐기합니다. 회귀 검사는 `test_ui_map_capture.py`에 있습니다.

## 경로·임무 흐름

`lane_routes.py`는 방향별 일부 좌표 저장을 허용하지만 `validate_route`는 입구·출구·다음 목표 세 항목을 요구합니다. UI는 저장 좌표와 편집 중 값, 지도 식별값을 대조한 뒤 연속 임무를 요청합니다. 같은 PC의 두 로봇은 방향별 경로를 공유합니다.

coordinator는 Nav2 입구 도착 이후 차선 액션을 실행합니다. 임무 서버는 정지·센서·워치독 설정을 검증하고 로컬 허가를 갱신합니다. 완료 Bool 이후 취소/정지·권한 해제를 확인하며, 연속 임무만 출구 위치 확인을 거쳐 다음 Nav2로 넘어갑니다. 한 시점에 서로 다른 명령 소스가 모터 토픽을 직접 발행하도록 만들지 않습니다.

## 변경 시 확인할 파일

| 변경 | 함께 확인 |
|---|---|
| 로봇 ID/domain | `robot_config.py`, bridge YAML, 로봇 실행 인자 |
| 액션/서비스 정의 | 두 저장소 `pinky_interfaces`, 로봇 임무 서버, `vision_control` |
| 차선 속도 | 운용 JSON, 로봇 launch watchdog 상한, mission server 기대값, mission guard |
| 새 UI 좌표 입력 | 지도 선택 모드·적용 구분, 좌표 변환, Tk resize, 저장 검증 |
| 종료/취소 | UI signal 처리, action 취소 확인, 로컬 허가 해제, bridge 종료 |

검사 명령은 [팀 설치 가이드](../../TEAM_LANE_GUIDE.md#7-코드-검사검증-범위)에 있습니다. 빌드·동작 검사·실주행 검증은 각각 구분해 기록합니다. 공유 브랜치에는 모델·현장 영상·지도·자격증명·빌드 산출물을 추가하지 않습니다.
