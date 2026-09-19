# Pinky Fleet Control

Pinky Pro 2대를 하나의 관제 PC에서 운영하기 위한 ROS 2 Jazzy 패키지 모음이다.
공유 지도 표시, Nav2 목표 전송, 수동 명령 라우팅, velocity permit, capacity 1 병목
관제를 제공한다.

## 포함 패키지

| 패키지 | 역할 |
|---|---|
| `pinky_interfaces` | 관제 PC와 로봇 gate가 공유하는 permit·heartbeat 메시지 |
| `multibot_control_ui` | Tk 관제 UI, ROS 토픽 I/O, Nav2 client, 병목 정책 |

로봇 측 `pinky_fleet_safety`, `pinky_led`, `pinky_bringup`, `pinky_navigation`은 이
저장소에 포함되지 않는다. 로봇 배포 환경에서 호환되는 버전을 별도로 준비해야 한다.

## 빌드

저장소를 ROS 2 workspace의 `src` 아래에 clone한다.

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/INYUP-BAEK/pinky-fleet-control.git

cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  pinky_interfaces multibot_control_ui
source install/setup.bash
```

## 실행과 설계 문서

- [관제 UI 사용·실행 방법](multibot_control_ui/README.md)
- [현재 구현 사양](multibot_control_ui/docs/CURRENT_IMPLEMENTATION.md)
- [코드 구조와 개발 가이드](multibot_control_ui/docs/ARCHITECTURE.md)
- [기여 방법](CONTRIBUTING.md)

## 테스트

```bash
cd ~/colcon_ws/src/pinky-fleet-control/multibot_control_ui
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
pytest -q
```

실제 로봇에 배포하기 전에는 domain bridge, Nav2 action, pose frame, velocity gate,
E-STOP과 실제 제동 거리를 통합 환경에서 별도로 확인해야 한다.

## 라이선스

Apache License 2.0. 자세한 내용은 [LICENSE](LICENSE)를 참고한다.
