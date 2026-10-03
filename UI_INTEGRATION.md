# UI 기반 Nav2·차선 통합 운용 (2026-10-03)

브랜치: `codex/nav2-lane-ui-20261003`. 로봇과 관제 코드를 함께 적용합니다.

## 처음 한 번 실행

로봇은 센서·Nav2·차선 임무 서버를 함께 실행하되 **차선 허가는 꺼진 상태**로 시작합니다. 현재 접속 주소는 `pinky@192.168.1.139`입니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/setup.bash
source ~/pinky_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=21
export LD_LIBRARY_PATH="/usr/local/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
# 추론을 별도 venv에 설치한 경우 해당 site-packages의 PYTHONPATH도 설정
ros2 launch pinky_robot_system robot_system.launch.py \
  robot_id:=robot1 map:="$HOME/260916_map.yaml" \
  start_motors:=true start_battery:=false start_nav2:=true \
  start_lane_control:=true \
  lane_control_config:="$HOME/pinky_calibration/lane_control_lane_only_verified_20261001.json" \
  lane_dry_run:=false hardware_watchdog_confirmed:=true \
  lane_start_enabled:=false perception_imgsz:=448
```

위 지도·설정 경로는 현재 시험 로봇의 실제 경로입니다. 다른 기체에서는 해당 기체의 지도와 보정 파일로 바꿉니다. 모터 통신 단절 정지를 확인한 장비에서만 `hardware_watchdog_confirmed:=true`를 사용합니다.

관제 PC의 GUI 터미널에서 **한 번** 실행합니다. 이 launch가 bridge와 UI를 함께 실행하므로 별도 bridge 명령은 필요 없습니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 launch multibot_control_ui control_ui.launch.py
```

외부에서 bridge를 이미 운용하는 구성에서만 `start_bridge:=false`를 사용합니다. 같은 UI·bridge를 중복 실행하지 않습니다.

## UI에서 운용

1. **로봇 선택·초기 위치 확인:** Nav2 지도와 로봇 위치를 확인합니다. 초기 위치 설정은 기존 패널을 사용합니다.
2. **연속 임무 경로 설정:** A_to_B 또는 B_to_A를 고릅니다. 입구·출구·다음 목표의 `지정`을 누른 뒤 지도에서 클릭하고 주행 방향으로 드래그합니다. `방향별 경로 저장`으로 저장합니다. 출구 좌표는 완료 버튼을 누를 실제 지점과 방향입니다. 초기 위치와 별개입니다.
3. **시작:** 현장 감시·즉시 정지 준비를 확인한 뒤 `연속 임무 시작` 또는 `차선 단독 테스트`를 누릅니다. 단독 테스트는 현재 배치 위치에서 시작하므로 경로 좌표 저장이 필요 없습니다.
4. **완료:** 실제 출구에서 `차선 구간 완료`를 누릅니다. 연속 임무는 출구 확인 창을 표시하고, STOP·로컬 허가 해제·새 AMCL 위치 확인 후 다음 Nav2 목표를 진행합니다. 단독 테스트는 STOP으로 끝나며 AMCL 위치를 덮어쓰지 않습니다.
5. **정지/재개:** `일시정지 / 재개`는 선택 임무와 목표를 유지합니다. `선택 임무 중단`은 임무를 취소합니다. 기존 전체 비상정지도 유지됩니다. 다음 시험은 같은 launch에서 다시 시작할 수 있습니다.

경로는 PC의 `~/.config/pinky_fleet_control/lane_routes.json`에 방향별로 저장됩니다. 저장 당시 지도의 내용·geometry가 달라지면 좌표를 확인하고 다시 저장해야 연속 임무를 시작할 수 있습니다. A/B는 전진 주행 방향을 구분하며 후진 명령을 뜻하지 않습니다.

## 통합된 점검과 감시

| 위치 | 담당 |
|---|---|
| 로봇 차선 임무 서버 | STOP·cmd_vel=0, 경로/라이다/odom/영상 신선도, watchdog 설정 확인; 허가 갱신; 정체·고장 감시; 종료 정지 확인 |
| 로봇 로컬 허가 | 1초 유효 기간. 임무 서버의 갱신이 끊기면 해제 |
| 관제 coordinator | Nav2 입구 도착/방향 확인, 차선 액션, 출구 위치 검증, 다음 Nav2 목표, 병목 통행 허가 |
| UI | 준비 상태, 실제 모드, 차선 거리·정지 이유, 방향별 설정, 시작/완료/중단 |

정상적인 중앙 HOLD·수동 조작은 차선 정체 타이머에서 제외합니다. 관제 연결 소실, 실제 주행 속도 초과, 센서 고장 지속, 주행 명령이 있지만 이동 없음은 임무를 종료합니다. 일시적인 인식 소실은 기존 차선 알고리즘의 제한된 유지 주행을 따릅니다. 차선 인식 소실을 정상 출구로 해석하지 않습니다.

UI를 닫으면 임무를 취소하고 관제 허가를 중지하며 함께 실행한 bridge도 종료합니다. 프로세스 강제 종료·통신 단절에서는 관제 permit과 로컬 주행 허가의 유효 기간이 정지를 담당합니다. 통신 복구 후에는 상태를 확인하고 새로 시작합니다.

차선 추론은 기존 448, 속도 상한은 0.03m/s입니다. 횡단보도 정지는 현재 시험용 설정에서 계속 꺼져 있습니다. 원본 카메라·scan을 PC로 추가 전송하지 않고 `/lane/mission_status`의 작은 상태 메시지만 수신합니다. 영상/rosbag/CSV 자동 저장 기능은 없습니다.

## 기존 시험 명령의 변경

기존 `pinky_start_full_lane.py`, `pinky_dual_domain_lane_monitor.py`의 역할은 정식 임무 서버·관제로 옮겼습니다. **이 버전과 구형 시작 스크립트를 함께 사용하지 마세요.** `/lane/set_enabled=true` 한 번만으로는 새 1초 허가가 유지되지 않습니다. 정지 probe는 선택적인 진단 도구로 남아 있고 정상 UI 운용에는 필요 없습니다.

## 검증 범위

로컬 검증: 로봇 알고리즘 132개, 격리 ROS 임무·로컬 허가 8개, 관제 52개 테스트 통과. 로봇·관제 패키지 빌드와 가짜 클라이언트를 사용한 Tk 버튼·경로 저장 이벤트도 통과했습니다. 로봇 패키지 검사에는 기존 skip 1개가 있습니다. 이번 통합 코드로 실제 Nav2와 차선 주행을 연속 실행한 검증은 아직 하지 않았습니다. 적용 후에는 정지 점검 → UI 차선 단독 시험 → A_to_B·B_to_A 연속 임무 순서로 확인합니다. Nav2와 YOLO 동시 실행 시 발열·부하는 실제 장비에서 추가 확인해야 합니다.
