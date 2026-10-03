# Pinky Fleet Control

**공유 브랜치: `codex/lane-field-20261001` · 기준일: 2026-10-03**

Pinky의 Nav2 이동·차선 연속 임무·차선 단독 시험을 하나의 Tk UI에서 제어합니다.

1. **새 PC 설치:** [TEAM_LANE_GUIDE.md](TEAM_LANE_GUIDE.md)
2. **실행 명령·버튼 사용·종료:** [UI_INTEGRATION.md](UI_INTEGRATION.md)
3. **로봇 설치:** [같은 브랜치의 로봇 가이드](https://github.com/jsh0116/pinky-lane-driving/blob/codex/lane-field-20261001/TEAM_LANE_GUIDE.md)

## 구성

| 패키지 | 역할 |
|---|---|
| `pinky_interfaces` | permit·heartbeat·FollowLane action·공유 서비스 |
| `vision_control` | 로봇 domain별 차선 임무 클라이언트·상태 수신 |
| `multibot_control_ui` | UI·Nav2 client·모드/병목 관제·지도 좌표 선택 |

기본 domain: robot1=21, robot2=19, 관제=22. 브리지는 UI launch가 함께 실행합니다.
로봇 소스는 [pinky-lane-driving](https://github.com/jsh0116/pinky-lane-driving/tree/codex/lane-field-20261001)에 있습니다. 두 저장소의 같은 브랜치를 함께 빌드합니다.

## 설치된 PC에서 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
ros2 launch multibot_control_ui control_ui.launch.py
```

설치·빌드는 위 팀 가이드에 있습니다. 현재 차선 프로필은 **0.06m/s·YOLO 448·횡단보도 정지 OFF·차선 라이다 물체 정지 OFF**이며, 0.06m/s 실주행은 아직 검증하지 않았습니다. 새 장비에서는 현장 감시·즉시 정지가 가능한 상태에서 점검합니다.

## 개발 문서

- [관제 패키지 안내](multibot_control_ui/README.md)
- [현재 구현 사양](multibot_control_ui/docs/CURRENT_IMPLEMENTATION.md)
- [코드 구조](multibot_control_ui/docs/ARCHITECTURE.md)
- [기여 방법](CONTRIBUTING.md)

Apache License 2.0 — [LICENSE](LICENSE)
