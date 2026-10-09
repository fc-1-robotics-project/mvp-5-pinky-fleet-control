"""Two-robot demo sequencing; all motion authority stays in FleetCoordinator."""

from copy import deepcopy
import json
import math
from pathlib import Path
import time

from .lane_routes import validate_pose
from .robot_config import ROBOT_BY_NAME

POSES = ('a_exit', 'a_final', 'b_entry', 'b_exit')
EXIT_RADIUS_M = .20
EXIT_DWELL_S = .5
RECOVERY_DELAY_S = 3.
RECOVERY_STABLE_S = 1.
POSE_POSITION_VARIANCE_LIMIT = .30
POSE_YAW_VARIANCE_LIMIT = math.radians(20.) ** 2


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
    """Confirm arrival near the lane endpoint before requesting the stop handshake."""

    def __init__(self):
        self.since = None

    def reset(self):
        self.since = None

    def evaluate(self, now, pose, target, telemetry, allowed):
        status = telemetry.get('exit_status')
        if not isinstance(status, dict):
            self.reset()
            return ''
        transport = telemetry.get('received_age_s')
        numeric = lambda v: type(v) in (int, float) and math.isfinite(v)
        valid = (allowed and pose is not None and math.dist(pose[:2], target[:2]) <= EXIT_RADIUS_M
                 and telemetry.get('fresh') is True and status.get('version') == 1
                 and telemetry.get('active') is True and telemetry.get('mode') == 'LANE'
                 and telemetry.get('permission_enabled') is True
                 and status.get('gate_run') is True
                 and status.get('obstacle') is False
                 and numeric(transport) and 0 <= transport <= 1.5
                 and status.get('control_reason') in {
                     'follow', 'tracking', 'crosswalk', 'stop_requested_or_short_path', 'sensor_failure'})
        if not valid:
            self.reset()
            return ''
        if self.since is None:
            self.since = now
        # Camera delay may postpone the request, but does not erase valid AMCL
        # arrival evidence. Never finish with a current sensor fault.
        if now - self.since >= EXIT_DWELL_S and status.get('sensors_ok') is True:
            return f'차선 끝 {EXIT_RADIUS_M * 100:g}cm 안 도착 확인'
        return ''


class TwoRobotDemo:
    """Reuse individual goal lifecycles; never publish a separate velocity/permit."""

    def __init__(self, coordinator, clock=time.monotonic):
        self.fleet = coordinator
        coordinator.sequence = self
        self.clock = clock
        self.active = False
        self.recovering = {}
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

    def is_recovering(self, robot):
        return self.active and robot in self.recovering

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
            checks = (
                (name not in f.requests, '진행 중인 기존 임무를 먼저 정리하세요.'),
                (f.node.heartbeat_is_fresh(name), 'heartbeat 수신 대기/지연'),
                (status.get('fresh') is True, '로봇 상태 메시지 수신 대기/지연'),
                (status.get('active') is False, '이전 차선 임무 종료 확인 중'),
                (status.get('mode') == 'STOP', f"STOP 모드 확인 필요 (현재 {status.get('mode')})"),
                (status.get('permission_enabled') is False, '차선 로컬 주행 허가 OFF 확인 중'),
                (status.get('cleanup_ok') is True, '이전 임무 STOP·허가 정리 확인 중'),
                (isinstance(status.get('exit_status'), dict)
                 and status['exit_status'].get('version') == 1, '로봇 시연 상태 코드 버전 확인 필요'),
            )
            for passed, reason in checks:
                if not passed:
                    raise ValueError(f'{name}: {reason}')
            localization = self._pose_status(name)
            if localization['pose'] is None:
                raise ValueError(f"{name}: {localization['reason']}")
        status = f.lane_navigation.telemetry(plan['a'])
        if not status.get('ready'):
            raise ValueError(f"{plan['a']}: 차선 시작 점검 · {status.get('readiness_reason', '준비 대기')}")
        self.plan, self.pending, self.closed = plan, deepcopy(plan['waypoints']), dict(plan['closed'])
        self.done, self.tasks, self.route_done = {'A': [], 'B': []}, {}, set()
        self.recovering = {}
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
        self.recovering = {}
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
        self.tasks[role] = dict(kind='NAV', goal=goal, request=None,
                                started=self.clock(), retries=0)
        if not self.is_recovering(self.plan[role.lower()]):
            self._submit_task(role)

    def _lane(self, role):
        self.tasks[role] = dict(kind='LANE', request=None, started=self.clock(),
                                finish_sent=None, retries=0)
        if not self.is_recovering(self.plan[role.lower()]):
            self._submit_task(role)

    def _submit_task(self, role):
        task, name = self.tasks[role], self.plan[role.lower()]
        if task['kind'] == 'NAV':
            ok, detail = self.fleet.submit_goal(name, *task['goal'], _owner=self)
        else:
            ok, detail = self.fleet.submit_lane_test(
                name, self.plan[role.lower() + '_route'], _owner=self)
        if not ok:
            self.wait_for_recovery(f'{name}: {detail}', robot=name)
            return False
        task['request'] = self.fleet.requests[name]
        task['started'] = self.clock()
        return True

    def wait_for_recovery(self, reason, *, robot=None, reset_health=False):
        """Hold affected robots; an unspecified robot denotes a shared safety fault."""
        if not self.active:
            return
        now = self.clock()
        for role in ('A', 'B'):
            name = self.plan[role.lower()]
            if robot is not None and name != robot:
                continue
            recovery = self.recovering.setdefault(name, dict(
                healthy_since=None, last_attempt=now, since=now, reason=''))
            self.exits[role].reset()
            if reset_health:
                recovery['healthy_since'] = None
            if recovery['reason'] != reason:
                recovery['reason'] = reason
                logger = getattr(self.fleet.node, 'get_logger', None)
                if logger:
                    logger().warning(f'통합 시연 복구 대기 [{self.stage}] {name}: {reason}')
            self._suspend_recovery_nav(role, now)
        self._recovery_detail()
        self.fleet._apply_motion_policy()
        self.fleet.node.publish_permits_now()

    def _recovery_detail(self):
        if self.recovering:
            reasons = dict.fromkeys(item['reason'] for item in self.recovering.values())
            self.detail = '복구 대기 · ' + ' / '.join(reasons) + ' · 해당 로봇 목표 유지'

    def _cancel_recovery_nav(self, role, now):
        task, name = self.tasks[role], self.plan[role.lower()]
        task['retry_requested'] = task['recovery_cancel_pending'] = True
        state = self.fleet.navigation.state(name)
        if state == 'CANCELLING':
            return
        if now - task.get('last_cancel_attempt', -math.inf) >= RECOVERY_DELAY_S:
            task['last_cancel_attempt'] = now
            self.fleet.navigation.cancel_goal(name)

    def _suspend_recovery_nav(self, role, now):
        """Avoid running Nav2's progress checker throughout a prolonged HOLD."""
        task, name = self.tasks.get(role), self.plan[role.lower()]
        if (not task or task['kind'] != 'NAV' or task['request'] is None
                or task['request'].phase == 'SUCCEEDED'
                or now - self.recovering[name]['since'] < RECOVERY_DELAY_S):
            return
        state = self.fleet.navigation.state(name)
        if state in {'SUCCEEDED', 'CANCELED', 'ABORTED', 'REJECTED'}:
            return
        if state in {'ACTIVE', 'PENDING'} or task.get('recovery_cancel_pending'):
            self._cancel_recovery_nav(role, now)

    def _pose_status(self, name):
        return self.fleet.lane_navigation.localization_status(name,
            position_variance_limit=POSE_POSITION_VARIANCE_LIMIT,
            yaw_variance_limit=POSE_YAW_VARIANCE_LIMIT)

    def _current_pose(self, name):
        return self._pose_status(name)['pose']

    def _health_error(self, name):
        if not self.fleet.node.heartbeat_is_fresh(name):
            return f'{name}: heartbeat 수신 재확인 중'
        if not self.fleet.node.pose_is_fresh(name):
            return f'{name}: 관제 지도 위치 수신 재확인 중'
        localization = self._pose_status(name)
        if localization['pose'] is None:
            return f"{name}: {localization['reason']} · 재확인 중"
        if not self.fleet.lane_navigation.telemetry(name).get('fresh'):
            return f'{name}: 로봇 상태 수신 재확인 중'
        return ''

    @staticmethod
    def _lane_released(status, request):
        return (status.get('fresh') and status.get('mission_id') == request.mission_id
                and status.get('active') is False and status.get('cleanup_ok') is True
                and status.get('mode') == 'STOP' and status.get('permission_enabled') is False)

    def _retry_task(self, role, now):
        task, f = self.tasks[role], self.fleet
        name, request = self.plan[role.lower()], task['request']
        if request is not None and request.paused:
            return False
        current = f.requests.get(name)
        if current is not None and current is not request:
            self.detail = f'복구 대기 · {name}: 다른 목표 정리 필요'
            return False
        if request is None:
            if task['kind'] == 'LANE' and not self._lane_start_ready(name):
                return False
            return self._submit_task(role)
        client = f.navigation if task['kind'] == 'NAV' else f.lane_navigation
        active_goal = getattr(client, 'has_active_goal', None)
        # Read the state after the handle check so a just-completed goal is
        # recognized as success before deciding to resend an ERROR request.
        goal_pending = active_goal(name) if active_goal is not None else None
        state = client.state(name)
        status = f.lane_navigation.telemetry(name)
        # A result may arrive while the fleet is held. Preserve that success;
        # never resend a waypoint which Nav2 already completed.
        completed = request.phase == 'SUCCEEDED' or (
            state == 'SUCCEEDED' and (request.delivered if task['kind'] == 'NAV' else request.lane_sent))
        if (task['kind'] == 'LANE' and task.get('finish_sent') is not None
                and status.get('state') == 'COMPLETE' and self._lane_released(status, request)):
            completed = True  # Matched robot success report recovers a lost action response.
        if completed:
            request.phase = 'SUCCEEDED'
            if task['kind'] == 'LANE' and not self._lane_released(status, request):
                return False
            if f.requests.get(name) is request:
                f.requests.pop(name)
            self._hold(name, 'DEMO_STEP_COMPLETE')
            return True
        if (task['kind'] == 'NAV' and state in {'ERROR', 'UNAVAILABLE'}
                and goal_pending is True):
            self._cancel_recovery_nav(role, now)
            return False
        if task.get('recovery_cancel_pending'):
            # A cancel RPC failure is not proof that the old action ended.
            ended = state in {'CANCELED', 'ABORTED', 'REJECTED'} or (
                state == 'ERROR' and goal_pending is False)
            if not ended:
                self._cancel_recovery_nav(role, now)
                return False
            task.pop('recovery_cancel_pending')
        if task['kind'] == 'LANE' and task.get('finish_sent') is not None:
            if status.get('active') is True and status.get('mission_id') == request.mission_id:
                f.node.publish_lane_finish(name)
                task['finish_sent'] = now
            self.detail = f'복구 대기 · {name}: 차선 완료 응답/STOP·허가 OFF 재확인'
            return False
        retry = (task.get('retry_requested') or now - task['started'] > 900.
                 or request.phase == 'LANE_FAILED'
                 or state in {'ABORTED', 'REJECTED', 'CANCELED', 'ERROR', 'UNAVAILABLE'}
                 or current is None)
        if not retry:
            return True
        if state in {'ACTIVE', 'PENDING', 'WAITING_FOR_LANE', 'FOLLOWING', 'SAFETY_HOLD',
                     'STOPPING', 'CANCELLING'}:
            if state not in {'STOPPING', 'CANCELLING'}:
                if task['kind'] == 'NAV':
                    self._cancel_recovery_nav(role, now)
                else:
                    client.cancel_goal(name)
            return False  # Wait for the old action to end before sending another.
        if task['kind'] == 'LANE':
            if not self._lane_start_ready(name):
                return False
            if current is request:
                f.requests.pop(name)
            if not self._submit_task(role):
                return False
        else:
            f.requests[name] = request
            if not f._send_request(request):
                return False
        task['started'] = now
        task.pop('retry_requested', None)
        task['retries'] += 1
        return True

    def _lane_start_ready(self, name):
        status = self.fleet.lane_navigation.telemetry(name)
        ready = (status.get('fresh') and status.get('active') is False
                 and status.get('mode') == 'STOP' and status.get('permission_enabled') is False
                 and status.get('cleanup_ok') is True and status.get('ready') is True
                 and isinstance(status.get('exit_status'), dict)
                 and status['exit_status'].get('version') == 1)
        if not ready:
            self.detail = f'복구 대기 · {name}: 차선 시작 점검/STOP·허가 OFF 재확인'
        return ready

    def _recover(self, role, now):
        name = self.plan[role.lower()]
        recovery = self.recovering[name]
        self._suspend_recovery_nav(role, now)
        if recovery['healthy_since'] is None:
            recovery['healthy_since'] = now
        if (now - recovery['healthy_since'] < RECOVERY_STABLE_S
                or now - recovery['last_attempt'] < RECOVERY_DELAY_S):
            return False
        recovery['last_attempt'] = now
        if role in self.tasks and not self._retry_task(role, now):
            return False
        self.recovering.pop(name)
        task = self.tasks.get(role)
        if task:
            request = task['request']
            if request.phase != 'SUCCEEDED':
                self.fleet._request_drive_mode(name,
                    'NAV2' if task['kind'] == 'NAV' else self.fleet._autonomous_mode_for(request.robot_name))
        self.detail = '복구 확인 완료 · 이전 구간 이어서 진행'
        self.fleet._apply_motion_policy()
        self.fleet.node.publish_permits_now()
        return True

    def _task_complete(self, role, now):
        if self.is_recovering(self.plan[role.lower()]):
            return False
        task = self.tasks.get(role)
        if task is None:
            return True
        f, name = self.fleet, self.plan[role.lower()]
        request = task['request']
        if request is None:
            self.wait_for_recovery(f'{name}: 목표 전송 재확인', robot=name)
            return False
        if request.paused or name in f.blocked_robots:
            self.exits[role].reset()
            return False
        if request.phase == 'SUCCEEDED':
            if task['kind'] == 'LANE':
                status = f.lane_navigation.telemetry(name)
                if not self._lane_released(status, request):
                    if now - task.setdefault('cleanup_wait', now) > 5.:
                        self.wait_for_recovery(f'{name}: 차선 종료 후 STOP·허가 해제 재확인', robot=name)
                    return False
            # NavigateToPose success is the sole Nav2 arrival decision.
            # The fleet still enforces fresh localization and bottleneck permits.
            if f.requests.get(name) is request:
                f.requests.pop(name)
            self._hold(name, 'DEMO_STEP_COMPLETE')
            del self.tasks[role]
            return True
        if f.requests.get(name) is not request:
            self.wait_for_recovery(f'{name}: 시연 목표 변경/취소 재확인', robot=name)
            return False
        state = (f.navigation if task['kind'] == 'NAV' else f.lane_navigation).state(name)
        if (request.phase == 'LANE_FAILED'
                or state in {'ABORTED', 'REJECTED', 'CANCELED', 'ERROR', 'UNAVAILABLE'}
                and name not in f.blocked_robots):
            self.wait_for_recovery(f'{name}: {state} / {request.phase}', robot=name)
            return False
        if now - task['started'] > 900.:
            task['retry_requested'] = True
            self.wait_for_recovery(f'{name}: 구간 제한 시간 15분 초과 · 같은 구간 재시도', robot=name)
            return False
        if task['kind'] == 'LANE':
            status = f.lane_navigation.telemetry(name)
            allowed = (name not in f.blocked_robots and name not in f.sequence_holds
                       and request.phase in {'LANE_ACTIVE', 'SAFETY_HOLD'}
                       and status.get('mission_id') == request.mission_id)
            reason = self.exits[role].evaluate(now, self._current_pose(name),
                        self.plan[role.lower() + '_exit'], status, allowed)
            if reason and task['finish_sent'] is None:
                f.node.publish_lane_finish(name)
                task['finish_sent'] = now
                self.detail = f'{role}: {reason} · STOP 확인 중'
            if task['finish_sent'] is not None and now - task['finish_sent'] > 5.:
                self.wait_for_recovery(f'{name}: 차선 종료 응답 재확인', robot=name)
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
            reason = self._health_error(name)
            if reason:
                self.wait_for_recovery(reason, robot=name, reset_health=True)
            elif self.is_recovering(name):
                self._recover(role, now)
        if self.stage == 'A_LANE' and self._task_complete('A', now) and self.active:
            self.stage, self.detail = 'NAV_ROUTES', 'A·B Nav2 waypoint 진행'
        if self.stage == 'NAV_ROUTES':
            for role in ('A', 'B'):
                if self.is_recovering(self.plan[role.lower()]):
                    continue
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
            if self.active and not self.recovering and len(self.route_done) == 2:
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
            if self.active and not self.recovering and a_done and b_done:
                self.active, self.stage, self.detail = False, 'COMPLETE', 'A·B 최종 도착 · STOP/HOLD'
                f.pause()
        self._recovery_detail()
