"""Two-robot demo sequencing; all motion authority stays in FleetCoordinator."""

from copy import deepcopy
import json
import math
from pathlib import Path
import time

from .lane_routes import validate_pose
from .robot_config import ROBOT_BY_NAME

POSES = ('a_exit', 'a_final', 'b_entry', 'b_exit')
EXIT_RADIUS_M = .07
EXIT_STATIONARY_S = 3.


def validate_plan(data):
    if not isinstance(data, dict) or data.get('version') != 1:
        raise ValueError('지원하지 않는 통합 시연 설정입니다.')
    plan = deepcopy(data)
    if (plan.get('a') not in ROBOT_BY_NAME or plan.get('b') not in ROBOT_BY_NAME
            or plan['a'] == plan['b']):
        raise ValueError('A와 B에 서로 다른 활성 로봇을 선택하세요.')
    if not isinstance(plan.get('map_key'), str) or not plan['map_key']:
        raise ValueError('시연 지도 정보가 없습니다.')
    for field in POSES:
        plan[field] = validate_pose(plan.get(field), field)
    if not isinstance(plan.get('waypoints'), dict) or not isinstance(plan.get('closed'), dict):
        raise ValueError('로봇별 waypoint 목록과 확정 상태가 필요합니다.')
    for role in ('A', 'B'):
        route = plan.get(role.lower() + '_route')
        if not isinstance(route, str) or not route.strip():
            raise ValueError('차선 route_id가 필요합니다.')
        queue = plan.get('waypoints', {}).get(role)
        if not isinstance(queue, list):
            raise ValueError('로봇별 waypoint 목록이 필요합니다.')
        plan['waypoints'][role] = [validate_pose(p, role) for p in queue]
        if type(plan.get('closed', {}).get(role)) is not bool:
            raise ValueError('로봇별 Nav2 목록 확정 여부가 필요합니다.')
    return plan


def save_plan(path, plan):
    checked = validate_plan(plan)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(checked, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


class LaneExitCondition:
    """No-line exit or actual 3 s standstill, only inside the 7 cm map region."""

    def __init__(self):
        self.since = None

    def reset(self):
        self.since = None

    def evaluate(self, now, pose, target, telemetry, allowed):
        status = telemetry.get('exit_status')
        if not isinstance(status, dict):
            self.reset()
            return ''
        bounds = status.get('boundaries')
        age, transport = status.get('observation_age_s'), telemetry.get('received_age_s', 0.)
        numeric = lambda v: type(v) in (int, float) and math.isfinite(v)
        valid = (allowed and pose is not None and math.dist(pose[:2], target[:2]) <= EXIT_RADIUS_M
                 and telemetry.get('fresh') is True and status.get('version') == 1
                 and telemetry.get('active') is True and telemetry.get('mode') == 'LANE'
                 and telemetry.get('permission_enabled') is True
                 and status.get('gate_run') is True and status.get('sensors_ok') is True
                 and status.get('obstacle') is False
                 and isinstance(bounds, dict)
                 and all(type(bounds.get(key)) is bool for key in ('left_visible', 'right_visible'))
                 and numeric(age) and numeric(transport) and 0 <= age + transport <= 1.1
                 and status.get('control_reason') in {
                     'follow', 'tracking', 'stop_requested_or_short_path', 'sensor_failure'})
        if not valid:
            self.reset()
            return ''
        if not bounds['left_visible'] and not bounds['right_visible']:
            return '양쪽 차선 소실 · 종료 반경 안'
        stationary = status.get('stationary_s')
        if not numeric(stationary) or stationary <= 0:
            self.reset()
            return ''
        if self.since is None:
            self.since = now
        if now - self.since >= EXIT_STATIONARY_S and stationary >= EXIT_STATIONARY_S:
            return '종료 반경 안 실제 정지 3초'
        return ''


class TwoRobotDemo:
    """Reuse individual goal lifecycles; never publish a separate velocity/permit."""

    def __init__(self, coordinator, clock=time.monotonic):
        self.fleet = coordinator
        coordinator.sequence = self
        self.clock = clock
        self.active = False
        self.stage = 'IDLE'
        self.detail = '시연 설정 대기'
        self.plan = {}
        self.pending = {'A': [], 'B': []}
        self.closed = {'A': False, 'B': False}
        self.done = {'A': [], 'B': []}
        self.tasks = {}
        self.route_done = set()
        self.exits = {role: LaneExitCondition() for role in ('A', 'B')}

    def owns(self, robot):
        return self.active and robot in (self.plan['a'], self.plan['b'])

    def start(self, plan, map_key):
        if self.active:
            raise ValueError('통합 시연이 이미 진행 중입니다.')
        plan = validate_plan(plan)
        f = self.fleet
        if plan['map_key'] != map_key:
            raise ValueError('저장된 지도와 현재 지도가 다릅니다. 좌표를 다시 확인하세요.')
        if f.emergency or f.manual_robot or f.lane_navigation is None:
            raise ValueError('긴급 정지·수동 모드를 해제하고 차선 서버를 연결하세요.')
        for name in (plan['a'], plan['b']):
            status = f.lane_navigation.telemetry(name)
            if (name in f.requests or not f.node.heartbeat_is_fresh(name)
                    or f.lane_navigation.current_localization(name) is None
                    or not status.get('fresh') or status.get('active') is not False
                    or status.get('mode') != 'STOP' or status.get('permission_enabled') is not False
                    or status.get('cleanup_ok') is not True
                    or not isinstance(status.get('exit_status'), dict)
                    or status['exit_status'].get('version') != 1):
                raise ValueError(f'{name}: STOP·허가 OFF·AMCL·새 로봇 상태 정보 확인이 필요합니다.')
        if not f.lane_navigation.telemetry(plan['a']).get('ready'):
            raise ValueError('A 로봇의 차선 시작 점검이 준비되지 않았습니다.')
        self.plan, self.pending, self.closed = plan, deepcopy(plan['waypoints']), dict(plan['closed'])
        self.done, self.tasks, self.route_done = {'A': [], 'B': []}, {}, set()
        for condition in self.exits.values():
            condition.reset()
        self.active, self.stage, self.detail = True, 'A_LANE', 'A 차선 출발 · B 대기'
        for name in (plan['a'], plan['b']):
            self._hold(name, 'DEMO_WAIT')
        self._lane('A')

    def replace_pending(self, role, points, closed):
        if not self.active or self.stage not in {'A_LANE', 'NAV_ROUTES'} or role in self.route_done:
            raise ValueError('이 로봇의 Nav2 대기 목록 편집 단계가 아닙니다.')
        if type(closed) is not bool:
            raise ValueError('목록 확정 상태가 필요합니다.')
        self.pending[role] = [validate_pose(p, role) for p in points]
        self.closed[role] = closed

    def _hold(self, name, reason):
        self.fleet.sequence_holds[name] = reason
        self.fleet._request_drive_mode(name, 'STOP')

    def cancel(self, detail='운영자 시연 중단', failed=False):
        if not self.active:
            return
        self.active = False
        self.stage, self.detail = ('FAILED' if failed else 'CANCELLED'), detail
        for role in ('A', 'B'):
            self.exits[role].reset()
            self._hold(self.plan[role.lower()], self.stage)
        for role in ('A', 'B'):
            name = self.plan[role.lower()]
            self.fleet.cancel_robot(name, _owner=self)
        if not self.fleet.emergency:
            self.fleet.pause()

    def _nav(self, role, goal):
        name = self.plan[role.lower()]
        ok, detail = self.fleet.submit_goal(name, *goal, _owner=self)
        if not ok:
            self.cancel(detail, failed=True)
            return
        self.tasks[role] = dict(kind='NAV', goal=goal, request=self.fleet.requests[name],
                                started=self.clock())

    def _lane(self, role):
        name = self.plan[role.lower()]
        ok, detail = self.fleet.submit_lane_test(name, self.plan[role.lower() + '_route'], _owner=self)
        if not ok:
            self.cancel(detail, failed=True)
            return
        self.tasks[role] = dict(kind='LANE', request=self.fleet.requests[name],
                                started=self.clock(), finish_sent=None)

    def _task_complete(self, role, now):
        task = self.tasks.get(role)
        if task is None:
            return True
        f, name = self.fleet, self.plan[role.lower()]
        request = task['request']
        if request.paused or name in f.blocked_robots:
            self.exits[role].reset()
            return False
        if request.phase == 'SUCCEEDED':
            if task['kind'] == 'LANE':
                status = f.lane_navigation.telemetry(name)
                if not (status.get('fresh') and status.get('mission_id') == request.mission_id
                        and status.get('active') is False and status.get('cleanup_ok') is True
                        and status.get('mode') == 'STOP' and status.get('permission_enabled') is False):
                    if now - task.setdefault('cleanup_wait', now) > 5.:
                        self.cancel('차선 종료 후 STOP·허가 해제 확인 실패', failed=True)
                    return False
            elif not f.node.arrival_is_close(name, task['goal']):
                self.cancel(f'{name}: Nav2 도착 위치/방향 불일치', failed=True)
                return False
            self._hold(name, 'DEMO_STEP_COMPLETE')
            del self.tasks[role]
            return True
        if f.requests.get(name) is not request:
            self.cancel(f'{name}: 시연 목표 변경/취소 감지', failed=True)
            return False
        state = (f.navigation if task['kind'] == 'NAV' else f.lane_navigation).state(name)
        if (request.phase == 'LANE_FAILED'
                or state in {'ABORTED', 'REJECTED', 'CANCELED', 'ERROR'}
                and name not in f.blocked_robots):
            self.cancel(f'{name}: {state} / {request.phase}', failed=True)
            return False
        if now - task['started'] > 900.:
            self.cancel(f'{name}: 구간 제한 시간 15분 초과', failed=True)
            return False
        if task['kind'] == 'LANE':
            status = f.lane_navigation.telemetry(name)
            allowed = (name not in f.blocked_robots and name not in f.sequence_holds
                       and request.phase in {'LANE_ACTIVE', 'SAFETY_HOLD'}
                       and status.get('mission_id') == request.mission_id)
            reason = self.exits[role].evaluate(now, f.lane_navigation.current_localization(name),
                        self.plan[role.lower() + '_exit'], status, allowed)
            if reason and task['finish_sent'] is None:
                f.node.publish_lane_finish(name)
                task['finish_sent'] = now
                self.detail = f'{role}: {reason} · STOP 확인 중'
            if task['finish_sent'] is not None and now - task['finish_sent'] > 5.:
                self.cancel(f'{name}: 차선 종료 응답 시간 초과', failed=True)
        return False

    def tick(self):
        if not self.active:
            return
        f, now = self.fleet, self.clock()
        if f.emergency or f.manual_robot:
            self.cancel('긴급 정지 또는 수동 모드로 시연 종료', failed=True)
            return
        if not f.enabled:
            for condition in self.exits.values():
                condition.reset()
            return
        for role in ('A', 'B'):
            name = self.plan[role.lower()]
            if (not f.node.heartbeat_is_fresh(name)
                    or f.lane_navigation.current_localization(name) is None):
                self.cancel(f'{name}: heartbeat 또는 AMCL 유효성 상실', failed=True)
                return
        if self.stage == 'A_LANE' and self._task_complete('A', now) and self.active:
            self.stage, self.detail = 'NAV_ROUTES', 'A·B Nav2 waypoint 진행'
        if self.stage == 'NAV_ROUTES':
            for role in ('A', 'B'):
                task = self.tasks.get(role)
                if not self._task_complete(role, now) or not self.active:
                    continue
                if task is not None:
                    self.done[role].append(task['goal'])
                if self.pending[role]:
                    self._nav(role, self.pending[role].pop(0))
                elif self.closed[role]:
                    self.route_done.add(role)
                else:
                    self.detail = f'{role}: 다음 waypoint 또는 목록 확정 대기'
            if self.active and len(self.route_done) == 2:
                self.stage, self.detail = 'FINAL', 'A 최종 목적지 · B 차선 입구'
                self._nav('A', self.plan['a_final'])
                if self.active:
                    self._nav('B', self.plan['b_entry'])
        elif self.stage == 'FINAL':
            self._task_complete('A', now)
            if self.active and self._task_complete('B', now):
                self.stage, self.detail = 'B_LANE', 'B 차선 주행 · A 최종 도착 확인'
                self._lane('B')
        elif self.stage == 'B_LANE':
            a_done = self._task_complete('A', now)
            b_done = self._task_complete('B', now) if self.active else False
            if self.active and a_done and b_done:
                self.active, self.stage, self.detail = False, 'COMPLETE', 'A·B 최종 도착 · STOP/HOLD'
                f.pause()
