# 팀원용 관제 PC 설치·차선 시험 가이드

**로봇과 관제 저장소에서 모두 `codex/nav2-lane-ui-20261003` 브랜치를 사용하세요.**

로봇 설치·보정·실행은 [로봇 가이드](https://github.com/jsh0116/pinky-lane-driving/blob/codex/nav2-lane-ui-20261003/TEAM_LANE_GUIDE.md)를 따릅니다. 2026-10-01 현장 차선 알고리즘에 2026-10-03 UI 통합 코드를 추가했으며 ROS 2 Jazzy / Python 3.12 환경입니다.

| 구성 | 기본 값 |
|---|---|
| 시험 로봇 | robot1 / ROS_DOMAIN_ID=21 |
| 관제 PC | ROS_DOMAIN_ID=22 |
| 추가 등록 로봇 | robot2 / domain 19; 해당 기체에도 동일 통합 코드 필요 |
| 포함 패키지 | `pinky_interfaces`, `vision_control`, `multibot_control_ui` |
| 시험 조건 | 차선 속도 0.03m/s, YOLO 448, Nav2 통합, 횡단보도 정지 끔 |

## 1. 관제 PC 설치

ROS Jazzy가 설치된 Ubuntu 환경의 새 워크스페이스에서 실행합니다. 기존 같은 이름 패키지가 있으면 별도의 새 워크스페이스를 사용하세요. 특히 `vision_control`을 바깥에서 또 clone하면 패키지가 중복됩니다.

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-rosdep \
  python3-tk python3-pytest ros-jazzy-domain-bridge
source /opt/ros/jazzy/setup.bash
mkdir -p ~/colcon_ws/src
git clone --branch codex/nav2-lane-ui-20261003 \
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

UI는 GUI 세션의 터미널에서 실행합니다. SSH 터미널에 DISPLAY가 없으면 Tk 창이 열리지 않습니다. 로봇과 PC가 서로 DDS 통신 가능한 같은 네트워크에 있어야 합니다. PC는 bridge/UI 외에도 action client가 로봇 domain 21에 직접 참가합니다. SSH 접속 성공만으로 ROS 통신이 확인되는 것은 아닙니다. 각 PC/로봇 시간 동기화 상태도 확인합니다.

## 2. 실행·주행·종료

이 브랜치부터 bridge·점검·감시·모드 전환은 통합되었습니다. 로봇과 PC에서 각각 한 번 실행한 뒤 UI로 운용합니다. **[UI_INTEGRATION.md](UI_INTEGRATION.md)**의 명령과 운용 순서를 사용하세요.

## 3. 다른 로봇·PC에 적용할 때

- 다른 PC에서도 같은 두 저장소 브랜치로 빌드합니다. 사용자 홈과 IP는 새 장비에 맞게 쓰고 기존 사용자 SSH 키는 복사하지 않습니다.
- 다른 기체를 **단독 robot1**으로 시험할 때는 domain 21을 쓰는 기존 기체를 종료합니다. 로봇 카메라/차체 보정은 해당 기체에서 검증해야 합니다.
- 동시에 robot2를 운용하려면 로봇 `robot_id`/domain, `multibot_control_ui/multibot_control_ui/robot_config.py`, `multibot_control_ui/config/domain_bridge.yaml`을 함께 맞춥니다. 임무 클라이언트는 등록된 각 로봇의 domain에 연결합니다. UI launch는 domain 22입니다.
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

현재 PC의 별도 빌드 경로에서 `pinky_interfaces`, `vision_control`, `multibot_control_ui` 3개 빌드와 import 검사를 통과했습니다. coordinator·경로 설정·액션 취소·AMCL 전환 등을 하드웨어 없는 테스트로 검사했습니다. 별도 시작/감시 스크립트는 제거했고 Tk 버튼과 경로 저장 이벤트도 가짜 ROS 클라이언트로 확인했습니다. 다른 PC의 새 설치 또는 실제 로봇을 이용한 이번 공유본 재시험은 수행하지 않았습니다.
