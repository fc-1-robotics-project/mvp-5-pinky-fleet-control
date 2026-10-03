# 새 관제 PC 설치·기존 PC 업데이트

**두 저장소에서 `codex/lane-field-20261001` 브랜치를 함께 사용합니다.**

[로봇 설치 가이드](https://github.com/jsh0116/pinky-lane-driving/blob/codex/lane-field-20261001/TEAM_LANE_GUIDE.md) → 이 문서 → [실행·UI 사용법](UI_INTEGRATION.md) 순서입니다. 기준일: 2026-10-03.

## 1. 준비

| 항목 | 기준 |
|---|---|
| PC | Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, GUI 세션 |
| 로봇 | 같은 공유 브랜치의 통합 스택과 기체별 보정·지도·모델 설치 |
| 기본 domain | robot1=21, robot2=19, 관제=22 |
| 포함 패키지 | `pinky_interfaces`, `vision_control`, `multibot_control_ui` |
| 현재 차선 프로필 | **0.06m/s / YOLO 448 / 횡단보도 정지 OFF / 차선 라이다 물체 자동 정지 OFF** |

ROS 미설치 PC는 [ROS Jazzy 공식 Ubuntu 설치 안내](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)에 따라 먼저 설치합니다. PC에서는 `ros-jazzy-desktop`을 사용할 수 있습니다. 이 PC에는 차선 추론용 torch/YOLO·카메라 드라이버·모델이 필요하지 않습니다. 차선 인식은 로봇에서 수행합니다.

로봇과 PC는 서로 DDS 통신 가능한 네트워크에 연결합니다. 관제는 bridge뿐 아니라 로봇 domain에 직접 참여하는 액션 클라이언트도 사용합니다. SSH 연결 성공만으로 ROS 통신까지 확인된 것은 아닙니다. `timedatectl status`로 시간 동기화를 확인하고, [DDS 발견 범위](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html)를 SUBNET으로 맞춥니다. 무선 AP의 클라이언트 격리/멀티캐스트 차단이나 방화벽 설정도 확인합니다.

## 2. 새 PC 설치·빌드

같은 이름 패키지가 없는 workspace를 사용합니다. 이 저장소에 `vision_control`이 포함되어 있으므로 별도 clone하지 않습니다. 로봇 저장소까지 같은 workspace에 clone하면 `pinky_interfaces`가 중복됩니다.

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

실행 파일이 어느 workspace에서 오는지도 확인합니다.

```bash
ros2 pkg prefix pinky_interfaces
ros2 pkg prefix vision_control
ros2 pkg prefix multibot_control_ui
ros2 pkg prefix domain_bridge
```

앞의 세 패키지는 새 `~/colcon_ws/install` 경로여야 합니다. 다른 과거 workspace를 `.bashrc`에서 source하고 있다면 깨끗한 터미널에서 ROS → 이번 install 순서로 다시 source합니다. 두 저장소의 `pinky_interfaces` action/service 정의는 함께 업데이트·빌드해야 합니다.

## 3. 실행·UI 운용

로봇을 [공통 실행 가이드](UI_INTEGRATION.md)의 통합 launch로 시작한 뒤, **PC의 GUI 터미널**에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=22
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
ros2 launch multibot_control_ui control_ui.launch.py
```

브리지와 UI를 함께 시작합니다. 별도 브리지를 이미 관리하는 경우에만 `start_bridge:=false`를 추가합니다. DISPLAY가 없는 SSH 세션에서 Tk 창은 열리지 않습니다.

1. robot1 또는 robot2를 선택하고 맵·현재 위치·heartbeat·차선 서버를 확인합니다.
2. AMCL 행의 `지도 선택` → 실제 위치/방향 드래그 → `설정`.
3. 차선 시험은 현장 준비 확인 → `차선 단독 테스트`. 연속 임무는 입구·출구·다음 목표의 `지도 선택` → 드래그 → `지정`을 먼저 완료합니다.
4. 실제 출구에서 `차선 구간 완료`. 연속 임무에서는 출구 확인 후 다음 Nav2 목표가 진행됩니다.
5. 마무리는 `선택 임무 중단`(진행 중일 때) → `관제 일시정지` → STOP·허가 해제 확인 → PC와 로봇 launch 종료.

좌우 분할선을 드래그해 조작 패널 폭을 조절합니다. 지도 선택은 해당 입력란만 채우며 `설정`/`지정`으로 확정합니다. `Esc`로 선택 모드를 종료합니다. 일반 지도 드래그는 선택 로봇의 Nav2 목표를 전송합니다. 자세한 버튼 동작·정지/재개 차이는 [UI 사용법](UI_INTEGRATION.md)을 따릅니다.

## 4. 다른 로봇·PC로 옮길 때

| 항목 | 단독 robot1 교체 | 두 번째 robot2 추가 |
|---|---|---|
| 로봇 ID/domain | robot1 / 21, 기존 robot1은 종료 | robot2 / 19 |
| 로봇 실행 | `robot_id:=robot1`, `ROS_DOMAIN_ID=21` | `robot_id:=robot2`, `ROS_DOMAIN_ID=19` |
| 관제 등록 | 기본 제공 | 기본 제공 |
| 현장 파일 | 새 기체 보정·현장 지도 준비 | 기체별 보정, 두 로봇이 같은 map 좌표계 사용 |

다른 이름/domain이 필요하면 로봇 실행 인자와 `multibot_control_ui/multibot_control_ui/robot_config.py`, `multibot_control_ui/config/domain_bridge.yaml`을 함께 맞춘 뒤 재빌드합니다. 관제 domain 변경 시 UI launch와 bridge의 22도 함께 수정합니다. bridge YAML의 같은 원본 토픽 키 반복은 각 로봇 도메인별 항목이므로 일반 dict 변환으로 합치면 안 됩니다.

PC의 `~/.config/pinky_fleet_control/lane_routes.json`은 입구·출구·다음 목표를 저장합니다. 다른 PC로 복사할 수 있지만 같은 지도/코스인지 확인하고 좌표를 다시 확인합니다. 경로는 로봇별이 아닌 **방향별 공용**입니다. 새 지도에서는 UI로 다시 지정합니다. 병목 영역 영구 설정은 `multibot_control_ui/config/bottleneck_zones.yaml`에 있고, UI에서 그린 영역은 종료 시 없어집니다.

IP는 SSH 접속 시 해당 로봇 주소를 사용합니다. 사용자 홈 경로·SSH 개인키·GitHub 자격증명을 다른 팀원과 공유할 필요가 없습니다.

## 5. 기존 PC 업데이트

UI와 브리지를 종료한 상태에서 실행합니다. 작업 중인 수정은 먼저 보관/커밋합니다.

```bash
cd ~/colcon_ws/src/pinky-fleet-control
git status --short
git fetch origin
git switch codex/lane-field-20261001
git pull --ff-only origin codex/lane-field-20261001
source /opt/ros/jazzy/setup.bash
cd ~/colcon_ws
colcon build --symlink-install --packages-up-to multibot_control_ui
source ~/colcon_ws/install/setup.bash
```

예전 개발 PC처럼 `vision_control`이 별도 `src/vision_control`에 있다면, 새 저장소 내부 패키지와 중복되지 않게 한 사본만 빌드 대상으로 둡니다. 기존 코드가 있으면 백업/비교 후 정리합니다. `colcon list`에서 패키지 이름 중복을 해결한 뒤 빌드합니다. 로봇도 같은 브랜치로 업데이트해야 최신 `FollowLane` 임무·허가 갱신 동작이 맞습니다.

## 6. 문제별 첫 확인

| 증상 | 확인할 내용 |
|---|---|
| `vision_control`/`FollowLane` import 실패 | 세 패키지 빌드 및 올바른 install source, 중복/오래된 interface overlay |
| Tk 창이 열리지 않음 | GUI 세션의 DISPLAY와 `python3-tk` |
| 맵·heartbeat 없음 | domain 21/19/22, bridge 한 개, SUBNET/멀티캐스트/방화벽 |
| 차선 시작 버튼 비활성 | 로봇 선택·현장 준비 체크·서버 연결·진행 임무/취소 상태 |
| `watchdog_setting_mismatch` / `velocity_limit_exceeded` | 로봇 JSON 네 속도 0.06, 설치 워치독/임무 서버 버전 일치 |
| `lane_observation_stale` / 센서 오류 | 로봇 카메라·추론 지연·온도, 448 입력, 영상/scan/odom 신선도 |
| 곡선에서 감속·끝에서 정지 | 현재 측정 경로 길이, 곡률/각속도 상한, 실제 차선 끝 여부 |
| 가까운 벽에서 정지 | Nav2 정지인지 차선 모드인지와 종료 이유 구분. 현재 차선 물체 정지는 OFF이나 센서 오류와 Nav2 장애물 판정은 별개 |

## 7. 코드 검사·검증 범위

하드웨어 연결 없이 실행하는 관제 동작 검사:

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
cd ~/colcon_ws/src/pinky-fleet-control
PYTHONPATH="$PWD/multibot_control_ui:$PWD/vision_control:${PYTHONPATH:-}" \
python3 -m pytest -q \
  multibot_control_ui/test/test_map_math.py \
  multibot_control_ui/test/test_zone_config.py \
  multibot_control_ui/test/test_fleet_coordinator.py \
  multibot_control_ui/test/test_lane_routes.py \
  multibot_control_ui/test/test_lane_client_lifecycle.py \
  multibot_control_ui/test/test_ui_shutdown.py \
  multibot_control_ui/test/test_ui_map_capture.py
```

관제 동작 검사 **73개 통과**. ROS 연결 없는 Tk 창에서 1180×740/1600×900 크기의 잘림·분할선 최소 폭·다섯 지도 선택 버튼을 확인했습니다. 현재 로봇 코드와 운용 JSON의 차선 속도는 0.06m/s이고, 로봇 단위/격리 ROS 검사까지 확인했습니다. **0.06 실주행, 새 로봇·새 PC 조합의 현장 주행은 미검증**입니다.

관제에 별도 데이터 수집 기능은 없습니다. 정상 운용에는 `/tmp`의 옛 시험 스크립트가 필요하지 않습니다.
