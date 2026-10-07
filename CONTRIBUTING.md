# 기여 가이드

공유 설치·실행 기준은 [TEAM_LANE_GUIDE.md](TEAM_LANE_GUIDE.md)와 [UI_INTEGRATION.md](UI_INTEGRATION.md)입니다. 현장 공유 브랜치는 `codex/lane-field-20261001`이며, 아래는 별도 신규 개발 작업의 일반 흐름입니다.

## 기본 작업 흐름

`main`에서 직접 개발하지 않고 작업별 브랜치를 사용한다.

```bash
git switch main
git pull --ff-only
git switch -c feat/short-description
```

권장 브랜치 접두사는 `feat/`, `fix/`, `docs/`, `refactor/`, `test/`다. 한 Pull
Request에는 한 가지 목적의 변경만 포함한다.

## 변경 전 확인

- 로봇 이름, domain 또는 토픽 변경 시 `robot_config.py`와
  `config/domain_bridge.yaml`을 함께 수정한다.
- launch parameter 기본값을 변경하면 `control_node.py`의 직접 실행 기본값과 문서를
  함께 수정한다.
- 안전 정책은 UI callback이 아니라 `fleet_coordinator.py`에 구현하고 단위 테스트를
  추가한다.
- 최종 모터 토픽 `/cmd_vel`을 직접 발행하지 않는다. 새 명령 소스는 로봇의
  `/cmd_vel_candidate`와 velocity gate를 거쳐야 한다.

## 로컬 검증

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  pinky_interfaces vision_control multibot_control_ui

source install/setup.bash
cd src/pinky-fleet-control/multibot_control_ui
pytest -q
```

## 커밋과 Pull Request

```bash
git status
git diff
git add multibot_control_ui/multibot_control_ui/control_ui.py  # 실제 수정 파일 선택
git commit -m "간결한 변경 목적"
git push -u origin HEAD
```

Pull Request에는 변경 목적, 주요 변경점, 실행한 테스트, 실제 로봇 검증 여부를 적는다.
안전 동작이나 토픽 구성을 바꾸는 PR은 다른 팀원 1명 이상의 리뷰 후 병합한다.
