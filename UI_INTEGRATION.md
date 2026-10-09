# 로봇·관제 실행 및 UI 사용법

PC 복구 수정 브랜치: **`feature/mission-recovery-position-20261009`** (`origin/develop` 기반). 로봇도 `feature/mission-recovery-position-20261009`를 함께 적용한다. 최신 정리: 2026-10-09.

처음 설치하는 장비는 [로봇 설치](https://github.com/fc-1-robotics-project/mvp-5-pinky-lane-driving/blob/codex/two-robot-demo-20261005/TEAM_LANE_GUIDE.md) / [관제 PC 설치](https://github.com/fc-1-robotics-project/mvp-5-pinky-fleet-control/blob/codex/two-robot-demo-20261005/TEAM_LANE_GUIDE.md)부터 진행합니다.

## 현재 공유 설정

| 항목 | 값 |
|---|---|
| 기본·최대·한쪽 경계 보완 속도 | **0.09m/s** |
| 차선 소실 유지 / 횡단보도 감속 속도 | **0.06m/s** |
| 차선 각속도 상한 / 최소 추종 거리 / 조향 배율 | 0.6rad/s / 0.10m / 1.2 |
| 카메라 / YOLO 입력 | 640×480 / **448** |
| 횡단보도 / 라이다 물체 자동 정지 | 인식·감속 ON, 횡단보도 정지 OFF / **라이다 정지 ON** |
| 일시 인식 소실 | 측정했던 경로를 odom으로 변환해 최대 1.5초·6cm 안에서만 사용 |

저장소의 `lane_control.json`과 `lane_control_lane_only.json`은 현재 같은 운용 값을 갖습니다. 파일 이름으로 기능 ON/OFF를 판단하지 않습니다. 두 프로필 모두 `crosswalk_control_enabled=true`, `behavior.crosswalk_stop=false`, `lidar_obstacle_stop_enabled=true`입니다. 횡단보도는 감속해 통과하고 라이다 물체 판정은 정지시킬 수 있습니다. PR #4는 조향 경로를 유지하면서 직진 반응 구간만 차로 내부 점으로 검사합니다. Nav2 장애물 회피는 별도 설정입니다.

로봇 홈의 JSON은 Git 갱신으로 바뀌지 않습니다. `lane_control_config`로 지정한 실제 파일을 먼저 확인합니다. 비상정지·관제 permit·로컬 허가 만료·영상/센서 오류 점검은 유지됩니다.

**전체 통합 시연 완주는 미확인입니다.** 2026-10-06 이전 버전 현장 시험에서는 관제 전체 복구 대기가 A의 최종 Nav2까지 멈췄습니다. 현재 PC 수정은 로봇별 복구를 분리해 정상 로봇이 기존 병목 허가 안에서 계속 진행하도록 합니다. 병목 점유 위치가 불확실하면 전체 HOLD를 유지합니다. 이번 수정은 오프라인 검증 범위이며 자세한 동작과 이전 현장 기록은 [통합 시연 가이드](multibot_control_ui/docs/TWO_ROBOT_DEMO.md)에 있습니다.

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
# 실제 설치한 지도와 기체별 보정 JSON 경로로 바꿉니다.

test -f "$PINKY_MAP" && test -f "$PINKY_LANE_CONFIG" && \
ros2 launch pinky_robot_system robot_system.launch.py \
  robot_id:=robot1 map:="$PINKY_MAP" \
  start_motors:=true start_battery:=false start_nav2:=true \
  start_lane_control:=true lane_control_config:="$PINKY_LANE_CONFIG" \
  lane_dry_run:=false hardware_watchdog_confirmed:=true \
  lane_start_enabled:=false perception_imgsz:=448 perception_cpu_threads:=1
```

`hardware_watchdog_confirmed:=true`는 모터 통신 단절 시 실제로 정지함을 확인한 기체에 사용합니다. 해당 확인 전 장비의 소프트웨어 준비 점검은 `start_motors:=false lane_dry_run:=true hardware_watchdog_confirmed:=false`로 실행합니다. 이 상태는 주행 준비 완료 판정을 위한 실제 정지 점검을 대체하지 않습니다.

통합 launch가 센서·Nav2·차선 임무 서버·모드 선택기·velocity gate를 실행합니다. **차선 로컬 허가는 OFF로 시작**하며 UI에서 시작해야 주행합니다. 같은 기체의 기존 bringup/Nav2/시험 서비스를 중복 실행하지 않습니다.

## 2. 관제 PC GUI 터미널 — 한 번 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=22
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY PINKY_FLEET_ROBOTS
ros2 launch multibot_control_ui control_ui.launch.py
```

이 명령이 **브리지와 UI를 각각 하나씩** 실행합니다. 별도 `domain_bridge` 명령이나 `/tmp` 시험 스크립트는 필요 없습니다. 기존 외부 브리지를 의도적으로 사용하고 있을 때만 `start_bridge:=false`를 추가합니다.

## 3. UI 준비 및 좌표 입력

1. **robot1/robot2 선택.** 맵·위치·heartbeat·차선 서버 상태를 확인합니다. 시작 전 실제 모드 STOP, 로컬 허가 해제, 로봇 정지를 확인합니다.
2. **AMCL 초기 위치:** 해당 로봇 행의 `지도 선택` → 지도에서 실제 위치를 누르고 실제 방향으로 10px 이상 드래그 → x/y/yaw 확인 → `초기 위치 적용`.
3. **통합 시연 설정:** 메인 UI의 `통합 시연 설정 · waypoint 편집` → A/B 배정 → 네 고정 좌표를 `지도 선택` → 드래그 → `좌표 반영`. A/B waypoint를 `대기점 추가`하고 `설정 저장`합니다. 상세 조건은 [통합 시연 가이드](multibot_control_ui/docs/TWO_ROBOT_DEMO.md)를 확인하세요.
4. **화면 폭:** 좌우 패널 사이 분할선을 드래그합니다. 왼쪽은 버튼이 잘리지 않는 최소 폭을 유지하고 세로로 스크롤됩니다.
5. **현장 준비:** 코스·출구가 비어 있고 계속 감시·즉시 비상정지가 가능한지 확인한 뒤 준비 체크박스를 선택합니다.

`지도 선택` 모드에서는 입력란만 바뀝니다. 다시 드래그해 조정한 뒤 `초기 위치 적용` 또는 `좌표 반영`으로 확정합니다. 시연 설정은 `설정 저장`도 필요합니다. `좌표 선택 취소 (Esc)`는 선택 모드를 종료하며, 이미 입력·저장한 숫자를 되돌리지는 않습니다. **선택 모드 밖의 일반 지도 드래그는 기존처럼 선택 로봇에 Nav2 목표를 전송합니다.**

좌표는 클릭을 시작한 위치, 방향은 드래그 방향입니다. 단위는 m/도, 좌표계는 Nav2 `map`입니다. 시작 시 표시된 기본 AMCL 숫자는 실제 로봇 위치를 뜻하지 않습니다.

## 4. 주행 방식 선택

| 목적 | 조작 | 완료 후 |
|---|---|---|
| 현재 위치에서 차선만 시험 | 차선에 배치 → 준비 체크 → `선택 로봇 차선 시험 시작` | `선택 로봇 차선 완료`로 STOP. AMCL 위치를 덮어쓰지 않음 |
| 두 로봇 통합 시연 | 별도 설정 창에서 저장 → 메인 UI에서 현장 확인 → `통합 시연 시작` | A 차선 → A/B Nav2 waypoint → A 최종 정지 / B 차선 → 전체 HOLD |
| 일반 Nav2 이동 | 지도 선택 모드 종료 → 로봇 선택 → 목표 클릭·드래그 | Nav2 도착 처리 |
| 진행 임무 잠시 멈춤 | `선택 임무 일시정지` / `선택 임무 재개` | 임무·목표 유지 |
| 진행 임무 취소 | `선택 임무 취소` | 취소·정지 처리 |

차선 단독 시험은 저장 경로가 필요 없으며 실제 출구에서 `선택 로봇 차선 완료`로 끝냅니다. 통합 시연은 AMCL상 끝 지점 반경 20cm 안에서 0.5초 도착을 확인해 자동 종료합니다. 차선 유무·3초 정지 대기는 필요 없으며, STOP·로컬 허가 OFF 확인 뒤 다음 단계로 넘어갑니다. 수동 완료 버튼도 사용할 수 있으며 이후 다음 단계로 움직일 수 있습니다. 시연 중 취소 버튼은 `통합 시연 중단 (두 대)`로 바뀌고 두 로봇을 함께 중단합니다.

시작 버튼은 연결·준비 조건이 충족되어야 활성화됩니다. 전체 비상정지 후에는 원인을 해소하고 `비상정지 해제 · 관제 RUN`으로 관제 상태를 복구합니다. 개별 Nav2의 저장 목표는 재개될 수 있습니다. 차선 시험/시연은 새로 시작해야 하며, 일시정지·임무 취소로 비상정지가 풀리지는 않습니다. 전체 버튼의 적용 범위는 [UI 기능표](multibot_control_ui/README.md#자주-쓰는-ui-기능)에 정리되어 있습니다.

## 5. 종료

1. 진행 중이면 `선택 임무 취소` 또는 `시연 중단`을 누릅니다. 출구에 도착했다면 먼저 `선택 로봇 차선 완료`로 정상 종료할 수 있습니다.
2. `전체 일시정지 (HOLD)` → 로봇 STOP·로컬 허가 OFF·관제 HOLD를 확인합니다.
3. UI 창을 닫거나 관제 터미널에서 `Ctrl+C`를 누릅니다. 함께 실행한 브리지도 종료됩니다.
4. 로봇 실행 터미널에서 `Ctrl+C`를 누릅니다.

긴급 상황에서는 현장 비상정지와 UI의 `전체 비상정지`를 사용합니다. 구형 `/lane/set_enabled` 수동 호출이나 시험 시작 스크립트를 현재 임무 서버와 함께 사용하지 않습니다.

## 6. 저장 위치·속도 수정·점검

- PC 시연 설정: `~/.config/pinky_fleet_control/two_robot_demo.json`. A/B 로봇·고정 좌표·waypoint를 저장합니다. 같은 지도·같은 코스가 전제이며 지도 해시가 다르면 시작을 거부합니다. 예전 `lane_routes.json`은 보존하지만 메인 UI의 옛 세 좌표 연속 임무 입력은 제거했습니다.
- UI의 병목 사각형은 현재 실행에만 적용됩니다. 영구 설정은 `multibot_control_ui/config/bottleneck_zones.yaml`을 수정하고 재빌드합니다.
- 기체별 카메라/차체 보정은 로봇의 JSON·URDF·카메라 YAML에 있습니다. 저장소 업데이트가 홈의 운용 JSON을 자동 변경하지는 않습니다.
- 속도는 `behavior.cruise_speed=0.09`, `control.max_speed=0.09`, `path.fallback_speed=0.09`, `path.blind_speed=0.06`, `behavior.approach_speed=0.06`입니다. 차선 launch의 워치독 `max_speed_mps`와 임무 서버 기대값은 `0.09`입니다. JSON만 임의 증속하면 거부/중단될 수 있습니다. 변경 후 로봇 launch를 재시작합니다.
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

## 7. 최신 시험 환경의 경로 예시

| 장치 | robot_id / domain | map | lane_control_config |
|---|---|---|---|
| A | robot1 / 21 | `/home/pinky/261003.yaml` | `/home/pinky/pinky_calibration/lane_control_lane_only_verified_20261001.json` |
| B | robot2 / 19 | `/home/pinky/pinky_robot_ws/src/pinky-lane-driving/pinky_navigation/map/261003.yaml` | `/home/pinky/pinky_calibration/lane_control_lane_only.json` |
| 관제 | domain 22 | 로봇 지도 수신 | PC는 차선 JSON을 실행하지 않음 |

위 경로는 현재 시험 장비의 예시입니다. 새 기체는 실제 지도 위치와 자체 보정을 사용합니다. B 실행 시 1절의 `ROS_DOMAIN_ID=19`, `robot_id:=robot2`를 함께 변경합니다. `PINKY_FLEET_ROBOTS`를 해제하면 등록된 두 로봇을 모두 표시합니다.
