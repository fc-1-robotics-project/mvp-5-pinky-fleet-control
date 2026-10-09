# Pinky Fleet Control

**작업 브랜치: `feature/mission-recovery-position-20261009` · 기준: `origin/develop` · 기준일: 2026-10-09**

Pinky의 Nav2 이동·두 로봇 통합 시연·차선 단독 시험을 하나의 Tk UI에서 제어합니다.

1. **새 PC 설치:** [TEAM_LANE_GUIDE.md](TEAM_LANE_GUIDE.md)
2. **실행 명령·버튼 사용·종료:** [UI_INTEGRATION.md](UI_INTEGRATION.md)
3. **로봇 설치:** [기존 통합 시연의 로봇 가이드](https://github.com/fc-1-robotics-project/mvp-5-pinky-lane-driving/blob/feature/mission-recovery-position-20261009/TEAM_LANE_GUIDE.md)

## 구성

| 패키지 | 역할 |
|---|---|
| `pinky_interfaces` | permit·heartbeat·FollowLane action·공유 서비스 |
| `vision_control` | 로봇 domain별 차선 임무 클라이언트·상태 수신 |
| `multibot_control_ui` | UI·Nav2 client·모드/병목 관제·지도 좌표 선택 |

기본 domain: robot1=21, robot2=19, 관제=22. 브리지는 UI launch가 함께 실행합니다.
로봇 소스는 [pinky-lane-driving](https://github.com/fc-1-robotics-project/mvp-5-pinky-lane-driving/tree/feature/mission-recovery-position-20261009)에 있습니다. 이번 수정은 관제 PC와 두 로봇의 같은 feature 브랜치를 함께 적용합니다. 로봇 쪽 의도적 대기·TF 원본 시각 보존 수정도 포함합니다.

## 설치된 PC에서 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
ros2 launch multibot_control_ui control_ui.launch.py
```

설치·빌드는 위 팀 가이드에 있습니다. 현재 저장소의 차선 프로필은 기본/최대/한쪽 보완 **0.09m/s**, 차선 소실 유지/횡단보도 감속 **0.06m/s**, YOLO **448**, 라이다 물체 정지 **ON**입니다. 기체 홈의 운용 JSON은 별도 파일이므로 실행 인자와 실제 내용을 확인합니다.

**로봇별 복구 대기를 분리해 정상 로봇은 기존 병목 허가 안에서 계속 진행합니다.** 병목 점유 위치가 불확실하면 전체 HOLD를 유지하며, 긴 복구 대기의 Nav2 목표는 종료 확인 후 해당 로봇만 재전송합니다. 원본 시각의 최대 100ms 앞섬을 허용하고 수신·원본 지연 상한 1.5초는 유지합니다. 기능 테스트 231개 통과(lint 3개 제외)와 실제 PC 작업공간 3개 패키지 빌드를 확인했습니다. 검증은 오프라인 범위이며 실제 전체 통합 시연 완주는 미확인입니다. [통합 시연 가이드](multibot_control_ui/docs/TWO_ROBOT_DEMO.md)에 현재 동작과 이전 버전의 현장 결과를 구분해 기록했습니다.

## 개발 문서

- [두 로봇 통합 시연](multibot_control_ui/docs/TWO_ROBOT_DEMO.md)
- [관제 패키지 안내](multibot_control_ui/README.md)
- [현재 구현 사양](multibot_control_ui/docs/CURRENT_IMPLEMENTATION.md)
- [코드 구조](multibot_control_ui/docs/ARCHITECTURE.md)
- [기여 방법](CONTRIBUTING.md)

Apache License 2.0 — [LICENSE](LICENSE)
