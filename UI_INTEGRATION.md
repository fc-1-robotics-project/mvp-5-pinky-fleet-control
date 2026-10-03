# 로봇·관제 실행 및 UI 사용법

두 저장소 모두 **`codex/lane-field-20261001`** 사용. 최신 정리: 2026-10-03.

처음 설치하는 장비는 [로봇 설치](https://github.com/jsh0116/pinky-lane-driving/blob/codex/lane-field-20261001/TEAM_LANE_GUIDE.md) / [관제 PC 설치](https://github.com/INYUP-BAEK/pinky-fleet-control/blob/codex/lane-field-20261001/TEAM_LANE_GUIDE.md)부터 진행합니다.

## 현재 공유 설정

| 항목 | 값 |
|---|---|
| 기본·최대·한쪽 경계 보완·차선 소실 유지 속도 | **0.06m/s** |
| 차선 각속도 상한 / 최소 추종 거리 / 조향 배율 | 0.6rad/s / 0.10m / 1.2 |
| 카메라 / YOLO 입력 | 640×480 / **448** |
| 횡단보도 자동 정지 / 라이다 물체 자동 정지 | **둘 다 OFF**인 차선 시험 프로필 |
| 일시 인식 소실 | 측정했던 경로를 odom으로 변환해 최대 1.5초·6cm 안에서만 사용 |

라이다는 Nav2·센서 신선도 점검에 계속 사용됩니다. `lidar_obstacle_stop_enabled=false`는 차선 제어의 물체 판정 정지만 끕니다. Nav2 장애물 회피 설정을 끄는 옵션은 아닙니다. 비상정지·관제 permit·로컬 허가 만료·영상/센서 오류 점검은 유지됩니다. 이 프로필은 현장 감시와 즉시 정지가 가능한 차선 시험용입니다.

**0.06m/s는 적용·빌드·단위/모의 ROS 검사까지 완료했고 실제 주행은 아직 검증하지 않았습니다.** 이전 0.03m/s 주행에서 사용자의 실제 출구 도착 확인이 있었으며, 이를 새 속도의 완주 검증으로 간주하지 않습니다.

## 1. 로봇 SSH 터미널 — 한 번 실행

새 로봇 설치 가이드대로 준비한 경로입니다. `robot1`은 domain 21, `robot2`는 domain 19를 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/setup.bash
source ~/pinky_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=21
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
export LD_LIBRARY_PATH="/usr/local/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"

# 설치 가이드의 추론 venv를 사용한 경우에만 필요합니다.
if [ -d "$HOME/pinky_inference/lib/python3.12/site-packages" ]; then
  export PYTHONPATH="$HOME/pinky_inference/lib/python3.12/site-packages:${PYTHONPATH:-}"
fi

export PINKY_MAP="$HOME/pinky_maps/site.yaml"
export PINKY_LANE_CONFIG="$HOME/pinky_calibration/lane_control_lane_only.json"
# 기존 시험 로봇(192.168.1.139)에서는 위 두 줄 대신 아래 경로를 사용합니다.
# export PINKY_MAP="$HOME/260916_map.yaml"
# export PINKY_LANE_CONFIG="$HOME/pinky_calibration/lane_control_lane_only_verified_20261001.json"

test -f "$PINKY_MAP" && test -f "$PINKY_LANE_CONFIG" && \
ros2 launch pinky_robot_system robot_system.launch.py \
  robot_id:=robot1 map:="$PINKY_MAP" \
  start_motors:=true start_battery:=false start_nav2:=true \
  start_lane_control:=true lane_control_config:="$PINKY_LANE_CONFIG" \
  lane_dry_run:=false hardware_watchdog_confirmed:=true \
  lane_start_enabled:=false perception_imgsz:=448
```

`hardware_watchdog_confirmed:=true`는 모터 통신 단절 시 실제로 정지함을 확인한 기체에 사용합니다. 해당 확인 전 장비의 소프트웨어 준비 점검은 `start_motors:=false lane_dry_run:=true hardware_watchdog_confirmed:=false`로 실행합니다. 이 상태는 주행 준비 완료 판정을 위한 실제 정지 점검을 대체하지 않습니다.

통합 launch가 센서·Nav2·차선 임무 서버·모드 선택기·velocity gate를 실행합니다. **차선 로컬 허가는 OFF로 시작**하며 UI에서 시작해야 주행합니다. 같은 기체의 기존 bringup/Nav2/시험 서비스를 중복 실행하지 않습니다.

## 2. 관제 PC GUI 터미널 — 한 번 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=22
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
ros2 launch multibot_control_ui control_ui.launch.py
```

이 명령이 **브리지와 UI를 각각 하나씩** 실행합니다. 별도 `domain_bridge` 명령이나 `/tmp` 시험 스크립트는 필요 없습니다. 기존 외부 브리지를 의도적으로 사용하고 있을 때만 `start_bridge:=false`를 추가합니다.

## 3. UI 준비 및 좌표 입력

1. **robot1/robot2 선택.** 맵·위치·heartbeat·차선 서버 상태를 확인합니다. 시작 전 실제 모드 STOP, 로컬 허가 해제, 로봇 정지를 확인합니다.
2. **AMCL 초기 위치:** 해당 로봇 행의 `지도 선택` → 지도에서 실제 위치를 누르고 실제 방향으로 10px 이상 드래그 → x/y/yaw 확인 → `설정`.
3. **연속 임무 경로:** A_to_B/B_to_A 선택 → 입구·출구·다음 목표 각각 `지도 선택` → 클릭·드래그 → 해당 행 `지정`. 세 항목 입력 후 `방향별 경로 저장`으로 한 번에 저장해도 됩니다.
4. **화면 폭:** 좌우 패널 사이 분할선을 드래그합니다. 왼쪽은 버튼이 잘리지 않는 최소 폭을 유지하고 세로로 스크롤됩니다.
5. **현장 준비:** 코스·출구가 비어 있고 계속 감시·즉시 비상정지가 가능한지 확인한 뒤 준비 체크박스를 선택합니다.

`지도 선택` 모드에서는 입력란만 바뀝니다. 다시 드래그해 조정한 뒤 `설정`/`지정`으로 확정합니다. `선택 취소 (Esc)`는 선택 모드를 종료하며, 이미 입력·저장한 숫자를 되돌리지는 않습니다. **선택 모드 밖의 일반 지도 드래그는 기존처럼 선택 로봇에 Nav2 목표를 전송합니다.**

좌표는 클릭을 시작한 위치, 방향은 드래그 방향입니다. 단위는 m/도, 좌표계는 Nav2 `map`입니다. 시작 시 표시된 기본 AMCL 숫자는 실제 로봇 위치를 뜻하지 않습니다.

## 4. 주행 방식 선택

| 목적 | 조작 | 완료 후 |
|---|---|---|
| 현재 위치에서 차선만 시험 | 차선에 배치 → 준비 체크 → `차선 단독 테스트` | `차선 구간 완료`로 STOP. AMCL 위치를 덮어쓰지 않음 |
| Nav2 → 차선 → Nav2 연속 | 세 좌표 저장 → 준비 체크 → `연속 임무 시작` | 입구 Nav2 도착 → 차선 주행 → 완료/출구 확인 → 정지·위치 확인 → 다음 Nav2 목표 |
| 일반 Nav2 이동 | 지도 선택 모드 종료 → 로봇 선택 → 목표 클릭·드래그 | Nav2 도착 처리 |
| 진행 임무 잠시 멈춤 | `일시정지 / 재개` | 임무·목표 유지 |
| 진행 임무 취소 | `선택 임무 중단` | 취소·정지 처리 |

차선 단독 시험은 저장 경로가 필요 없습니다. 연속 임무의 출구 좌표는 **사용자가 실제로 완료 버튼을 누를 위치와 방향**입니다. 단순 차선 인식 소실은 출구 완료로 간주하지 않습니다. 실제 출구에서 `차선 구간 완료`를 누르세요. 연속 임무에서는 출구 확인 창 이후 다음 Nav2 목표로 움직일 수 있습니다.

시작 버튼은 연결·준비 조건이 충족되어야 활성화됩니다. 전체 비상정지 후에는 원인을 해소하고 `관제 시작 / 상태 재확인`으로 관제 상태를 복구합니다. 저장된 Nav2 목표가 있다면 재개 가능성을 확인한 뒤 누릅니다.

## 5. 종료

1. 진행 중이면 `선택 임무 중단`을 누릅니다. 출구에 도착했다면 먼저 `차선 구간 완료`로 정상 종료할 수 있습니다.
2. `관제 일시정지` → 로봇 STOP·로컬 허가 OFF·관제 HOLD를 확인합니다.
3. UI 창을 닫거나 관제 터미널에서 `Ctrl+C`를 누릅니다. 함께 실행한 브리지도 종료됩니다.
4. 로봇 실행 터미널에서 `Ctrl+C`를 누릅니다.

긴급 상황에서는 현장 비상정지와 UI의 `전체 정지 / 선택 해제`를 사용합니다. 구형 `/lane/set_enabled` 수동 호출이나 시험 시작 스크립트를 현재 임무 서버와 함께 사용하지 않습니다.

## 6. 저장 위치·속도 수정·점검

- PC 경로: `~/.config/pinky_fleet_control/lane_routes.json`. 방향별로 저장되며 **모든 로봇이 같은 방향 설정을 공유**합니다. 같은 지도·같은 코스가 전제입니다. 다른 사이트의 경로 파일을 그대로 사용하지 않습니다. 지도 내용/geometry가 바뀌면 세 좌표를 확인하고 다시 저장합니다.
- UI의 병목 사각형은 현재 실행에만 적용됩니다. 영구 설정은 `multibot_control_ui/config/bottleneck_zones.yaml`을 수정하고 재빌드합니다.
- 기체별 카메라/차체 보정은 로봇의 JSON·URDF·카메라 YAML에 있습니다. 저장소 업데이트가 홈의 운용 JSON을 자동 변경하지는 않습니다.
- 네 가지 속도 필드는 `behavior.cruise_speed`, `control.max_speed`, `path.fallback_speed`, `path.blind_speed`이며 현재 모두 `0.06`입니다. 워치독 `max_speed_mps=0.06`, 임무 서버 기대값 `0.06`, 감시 임계값 `0.0605`(허용 오차 포함)도 맞춰져 있습니다. JSON만 임의 증속하면 거부/중단될 수 있습니다. 변경 후 로봇 launch를 재시작합니다.
- 거리·종료 이유는 UI와 작은 임무 상태 메시지로 확인합니다. 원본 영상·rosbag·CSV 자동 수집은 없습니다.

읽기 전용 진단 예시(해당 터미널에서 ROS/워크스페이스 source 후):

```bash
# 관제 PC: robot1의 실제 도메인에 직접 조회
ROS_DOMAIN_ID=21 ros2 action info /follow_lane
ROS_DOMAIN_ID=21 ros2 action info /navigate_to_pose
ROS_DOMAIN_ID=21 ros2 topic echo /lane/mission_status --once
# 브리지를 통한 관제 상태
ROS_DOMAIN_ID=22 ros2 topic echo /robot1/fleet/heartbeat --once
# 로봇에서 조회
ROS_DOMAIN_ID=21 ros2 param get /lane_watchdog max_speed_mps
ROS_DOMAIN_ID=21 ros2 topic echo /drive/mode_status --once
ROS_DOMAIN_ID=21 ros2 topic echo /cmd_vel --once
```

`single_boundary`는 한쪽 경계 사용, `recent_path_no_boundaries`는 이전 측정 경로의 제한적 재사용입니다. `no_current_lane`·`lane_observation_stale`가 지속되면 현재 경로/영상 신선도를 확인합니다. `velocity_limit_exceeded` 또는 `watchdog_setting_mismatch`는 속도 설정·설치 코드 버전을 대조합니다. `no_odom_progress`는 주행 명령 대비 실제 이동을 확인합니다.
