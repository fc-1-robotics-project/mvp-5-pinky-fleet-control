# 팀원용 관제 PC 설치·차선 시험 가이드

**로봇과 관제 저장소에서 모두 `codex/lane-field-20261001` 브랜치를 사용하세요.**

로봇 설치·보정·실행은 [로봇 가이드](https://github.com/jsh0116/pinky-lane-driving/blob/codex/lane-field-20261001/TEAM_LANE_GUIDE.md)를 따릅니다. 기준은 2026-10-01 현장 적용본이며 ROS 2 Jazzy / Python 3.12 환경입니다.

| 구성 | 기본 값 |
|---|---|
| 시험 로봇 | robot1 / ROS_DOMAIN_ID=21 |
| 관제 PC | ROS_DOMAIN_ID=22 |
| 추가 등록 로봇 | robot2 / domain 19; 아래 시험 스크립트 대상 아님 |
| 포함 패키지 | `pinky_interfaces`, `vision_control`, `multibot_control_ui` |
| 시험 조건 | 속도 0.03m/s, YOLO 448, Nav2 끔, 횡단보도 정지 끔 |

## 1. 관제 PC 설치

ROS Jazzy가 설치된 Ubuntu 환경의 새 워크스페이스에서 실행합니다. 기존 같은 이름 패키지가 있으면 별도의 새 워크스페이스를 사용하세요. 특히 `vision_control`을 바깥에서 또 clone하면 패키지가 중복됩니다.

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-rosdep \
  python3-tk python3-pytest ros-jazzy-domain-bridge
source /opt/ros/jazzy/setup.bash
mkdir -p ~/colcon_ws/src
git clone --branch codex/lane-field-20261001 \
  https://github.com/INYUP-BAEK/pinky-fleet-control.git \
  ~/colcon_ws/src/pinky-fleet-control
# rosdep 초기화가 안 된 PC에서만: sudo rosdep init
rosdep update
cd ~/colcon_ws
rosdep install --from-paths src/pinky-fleet-control --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-up-to multibot_control_ui
source ~/colcon_ws/install/setup.bash
python3 -c 'from pinky_interfaces.action import FollowLane; from vision_control.lane_client import FleetLaneClients; import tkinter; print("imports OK")'
```

UI는 GUI 세션의 터미널에서 실행합니다. SSH 터미널에 DISPLAY가 없으면 Tk 창이 열리지 않습니다. 로봇과 PC가 서로 DDS 통신 가능한 같은 네트워크에 있어야 합니다. PC는 bridge/UI 외에도 action client와 시험 도구가 로봇 domain 21에 직접 참가합니다. SSH 접속 성공만으로 ROS 통신이 확인되는 것은 아닙니다. 각 PC/로봇 시간 동기화 상태도 확인합니다.

## 2. 정지 상태로 시스템 준비

먼저 로봇 가이드의 launch를 `lane_start_enabled:=false`로 실행하고 로봇에서 정지 probe를 실행합니다. 원본 카메라 영상은 PC로 상시 전송하지 않습니다.

### PC 터미널 A — domain bridge 1개

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=22
ros2 run domain_bridge domain_bridge \
  ~/colcon_ws/install/multibot_control_ui/share/multibot_control_ui/config/domain_bridge.yaml
```

### PC 터미널 B — UI 1개

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch multibot_control_ui control_ui.launch.py
```

UI에서 robot1을 선택합니다. Nav2가 꺼진 차선 단독 시험에서는 지도/AMCL 표시가 없을 수 있습니다. 아직 UI의 출발 버튼을 누르지 않습니다. 아래 시작 스크립트가 출발을 담당합니다.

### PC 터미널 C — 감시기 먼저

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
python3 ~/colcon_ws/src/pinky-fleet-control/tools/field_test/pinky_dual_domain_lane_monitor.py \
  --wait-active-s 120 --max-active-s 360 \
  --no-progress-s 15 --lane-fault-hold-s 15
```

시작 대기는 최대 120초, 주행 시간은 최대 360초입니다. 감시기가 종료됐다면 다시 실행한 뒤 출발합니다. 감시기는 robot1/domain21과 PC/domain22에 접속하도록 고정되어 있습니다. 낮은 부하의 기존 텍스트 감시 도구이며 영상/rosbag 수집 기능은 없습니다. 센서/통신 고장, 제한 초과, 지속 정지 시 중앙·로컬 허가를 해제합니다. 제어 노드 자체의 센서/장애물 정지는 이 15초 대기와 별개로 계속 작동합니다.

## 3. 실제 출발

**실행 즉시 로봇이 움직일 수 있습니다.** 로봇이 차선 안에 있고 코스·출구가 비었으며, 현장 작업자가 계속 감시·즉시 비상정지·출구 종료 Bool 조작을 할 수 있는 상태에서만 다음 명령을 실행합니다.

### PC 터미널 D — 시작 요청 1회

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
python3 ~/colcon_ws/src/pinky-fleet-control/tools/field_test/pinky_start_full_lane.py
```

`LANE_START_SUCCESS`가 나오면 로컬 `/lane/set_enabled=true`와 중앙 `/robot1/lane/test=true`가 승인된 것입니다. 스크립트는 watchdog 설정, 신선한 경로·scan·odom, STOP, cmd_vel 0, 초기 로컬 estop를 검사합니다. `LANE_START_FAILED`이면 원인을 해결하고 정지 상태에서 다시 검사합니다. 실패한 출발을 수동 허가로 우회하지 않습니다.

주행 중에는 안쪽/바깥쪽 선 밟음, 중앙 복귀, 멈칫거림, 실제 끝단 도달 여부를 관찰합니다. 감시기의 `lane_reason`, `control_reason`, 이동 거리도 함께 봅니다. 샘플 빈도는 진단 콜백 횟수 기준이고 서로 독립적인 주행 사건 횟수가 아닙니다. 카메라 추론 주기보다 진단 출력이 빠를 수 있습니다.

## 4. 종료와 권한 해제

1. 출구에서 UI의 **선택 로봇 차선 종료 Bool 전송** 버튼을 누릅니다. 선택 대상은 robot1이어야 합니다.
2. 감시기 `TRIAL_SUMMARY`의 종료 원인과 cleanup 내 중앙/로컬 권한 해제 응답이 모두 `ok: true`인지 확인합니다. 그 뒤 STOP과 cmd_vel 0을 확인합니다.
3. 고장·지속 정지 시에도 두 허가를 모두 해제합니다. 원격 명령이 응답하지 않으면 현장 비상정지로 정지 상태를 확보합니다.
4. 로봇 launch를 Ctrl+C로 종료하고 PC UI/bridge도 Ctrl+C로 종료합니다. 다음 시험은 launch를 새로 시작한 후 정지 점검부터 반복합니다.

감시기를 잃었거나 수동 해제가 필요할 때는 UI가 켜져 있는 동안 다음을 실행합니다. `success: true` 응답을 확인하고 모드를 확인하세요.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
ROS_DOMAIN_ID=22 ros2 service call /robot1/lane/test std_srvs/srv/SetBool '{data: false}'
ROS_DOMAIN_ID=21 ros2 service call /lane/set_enabled std_srvs/srv/SetBool '{data: false}'
ROS_DOMAIN_ID=21 ros2 topic pub --once /drive/mode_request std_msgs/msg/String '{data: STOP}'
ROS_DOMAIN_ID=21 ros2 topic echo --once /drive/mode_status std_msgs/msg/String
ROS_DOMAIN_ID=21 ros2 topic echo --once /cmd_vel geometry_msgs/msg/Twist
```

세 명령은 순서대로 실행하며, 한 서비스가 응답하지 않아 명령이 대기하면 해당 명령을 Ctrl+C로 끝내고 현장 정지를 확보한 상태에서 나머지 허가도 해제합니다. PC 네트워크가 끊긴 경우 위 원격 명령은 정지를 보증하지 않습니다.

## 5. 다른 로봇·PC에 적용할 때

- 다른 PC에서도 같은 두 저장소 브랜치로 빌드합니다. 사용자 홈과 IP는 새 장비에 맞게 쓰고 기존 사용자 SSH 키는 복사하지 않습니다.
- 다른 기체를 **단독 robot1**으로 시험할 때는 domain 21을 쓰는 기존 기체를 종료합니다. 로봇 카메라/차체 보정은 해당 기체에서 검증해야 합니다.
- 동시에 robot2를 운용하려면 로봇 `robot_id`/domain, `multibot_control_ui/multibot_control_ui/robot_config.py`, `multibot_control_ui/config/domain_bridge.yaml`을 함께 맞춥니다. 제공 시험 스크립트는 robot1 고정이므로 robot2에 그대로 사용하면 안 됩니다. UI launch도 domain 22 고정입니다.
- 두 저장소에 포함된 `pinky_interfaces`를 함께 빌드하세요. 오래된 interface overlay를 source하면 action/service 타입이 맞지 않을 수 있습니다.
- 현장에서는 10cm 설정으로 중앙 주행이 개선됐지만, 최종 수정 조합의 전체 곡선 코스 연속 완주는 아직 검증하지 않았습니다. 마지막 사용자는 완전한 끝단 전 종료했으며 코스 끝을 직선으로 늘릴 예정입니다.

코드 검사:

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
cd ~/colcon_ws/src/pinky-fleet-control
PYTHONPATH="$PWD/multibot_control_ui:$PWD/vision_control:${PYTHONPATH:-}" \
  python3 -m pytest multibot_control_ui/test/test_fleet_coordinator.py
```

### 문제별 첫 확인

| 증상 | 첫 확인 |
|---|---|
| `No module named vision_control` | 이 브랜치의 포함 패키지 빌드 및 install/setup.bash source |
| UI가 안 열림 | GUI 세션, DISPLAY, python3-tk |
| 시작 서비스 없음 | UI가 하나 실행 중인지, domain21/22 통신 여부 |
| 차선/영상 stale | 로봇 온도·추론 지연·카메라, 448 입력·1.1초 설정 |
| 벽 근처 정지 / 곡선 끝 정지 | 실제 물체와 차체 간격, scan_hit, 경로 길이·lane_reason 대조; 장애물 검사 전체를 끄지 않음 |

### 공유 브랜치 포장 시 검증

현재 PC의 별도 빌드 경로에서 `pinky_interfaces`, `vision_control`, `multibot_control_ui` 3개 빌드와 import 검사를 통과했습니다. 관제 coordinator 테스트 22개가 통과했고 두 시험 스크립트의 `--help` 실행을 확인했습니다. 다른 PC의 새 설치 또는 실제 로봇을 이용한 이번 공유본 재시험은 수행하지 않았습니다.
