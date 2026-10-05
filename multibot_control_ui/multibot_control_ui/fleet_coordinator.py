"""Position-based capacity-one bottleneck control for the desktop UI."""

from dataclasses import dataclass
import math
import time
from typing import Dict, List, Optional, Set, Tuple
import uuid

from pinky_interfaces.msg import FleetPermit

from .robot_config import ROBOTS
from .zone_config import BottleneckZone, rectangular_zone


Goal = Tuple[float, float, float]


@dataclass
class NavigationRequest:
    """The latest UI goal, retained while a robot is held at a boundary."""

    robot_name: str
    goal: Goal
    phase: str = 'ACTIVE'
    delivered: bool = False
    last_send_attempt: float = 0.0
    lane_after_arrival: bool = False
    lane_exit_pose: Optional[Goal] = None
    mission_id: str = ''
    route_id: str = 'right_lane'
    lane_sent: bool = False
    transition_started: float = 0.0
    next_goal: Optional[Goal] = None
    paused: bool = False
    lane_only: bool = False
    lane_duration_sec: float = 300.0


@dataclass
class ZoneRuntime:
    """Observed occupancy state for one capacity-one zone."""

    config: BottleneckZone
    state: str = 'FREE'
    owner: Optional[str] = None
    lease_id: str = ''
    detail: str = ''


class FleetCoordinator:
    """Send goals freely and gate motion from observed robot positions."""

    _RUNNING_NAV_STATES = {'ACTIVE', 'PENDING'}

    def __init__(
        self,
        node,
        navigation,
        zones: List[BottleneckZone],
        lane_navigation=None,
    ) -> None:
        self.node = node
        self.navigation = navigation
        self.lane_navigation = lane_navigation
        self.zones: Dict[str, ZoneRuntime] = {
            zone.zone_id: ZoneRuntime(zone)
            for zone in zones
        }
        self.requests: Dict[str, NavigationRequest] = {}
        self.sequence = None
        self.sequence_holds = {}
        self.enabled = False
        self.emergency = False
        self.manual_robot: Optional[str] = None
        self.blocked_robots: Set[str] = set()
        self.summary = '관제 정지 · 모든 로봇 HOLD'
        self.robot_details = {
            robot.name: 'HOLD · 관제 시작 대기'
            for robot in ROBOTS
        }
        self._hold_all('STARTUP')

    # Public coordination API ---------------------------------------------

    def start(self) -> Tuple[bool, str]:
        """Enable supervision; unavailable robots remain held individually."""
        self.enabled = True
        self.emergency = False
        for runtime in self.zones.values():
            runtime.owner = None
            runtime.lease_id = ''
            runtime.state = 'FREE'
            runtime.detail = ''
        if not self._update_occupancy():
            return False, self.summary
        self._apply_motion_policy()
        self.summary = self._make_summary()
        self.node.publish_permits_now()
        return True, self.summary

    def pause(self) -> None:
        """Hold every robot without discarding its current Nav2 goal."""
        self.enabled = False
        self.emergency = False
        self.blocked_robots = {robot.name for robot in ROBOTS}
        for request in self.requests.values():
            if not request.phase.startswith('LANE') and request.phase not in {
                'WAITING_FOR_LANE', 'SAFETY_HOLD', 'RELOCALIZING',
            }:
                request.phase = 'PAUSED'
        for runtime in self.zones.values():
            runtime.detail = '관제 일시정지'
        self._hold_all('PAUSED')
        self.summary = '관제 일시정지 · 목표 유지 · 모든 로봇 HOLD'
        self.node.publish_permits_now()

    def emergency_stop(self, reason: str = 'OPERATOR_ESTOP') -> None:
        """Cancel active goals, retain their destinations, and force E-STOP."""
        self.enabled = False
        self.emergency = True
        self.manual_robot = None
        self.blocked_robots = {robot.name for robot in ROBOTS}
        for request in self.requests.values():
            request.phase = 'LANE_FAILED' if request.lane_only or request.lane_after_arrival else 'E_STOP_HOLD'
            request.delivered = False
        for robot in ROBOTS:
            self.navigation.cancel_goal(robot.name)
            if self.lane_navigation is not None:
                self.lane_navigation.cancel_goal(robot.name)
            self._request_drive_mode(robot.name, 'STOP')
            self._set_manual_routing(robot.name, False)
            self.node.set_gate_mode(
                robot.name,
                FleetPermit.MODE_ESTOP,
                reason=reason,
            )
            self.robot_details[robot.name] = f'E-STOP · {reason}'
        for runtime in self.zones.values():
            runtime.state = 'LOCKED'
            runtime.detail = reason
        retained = len(self.requests)
        self.summary = f'긴급 정지 · {reason} · 저장 목표 {retained}개 유지'
        self.node.publish_permits_now()

    def set_runtime_rectangle(
        self,
        first: Tuple[float, float],
        second: Tuple[float, float],
    ) -> BottleneckZone:
        """Replace configuration with one UI-defined rectangular zone."""
        if self.enabled:
            raise ValueError('관제를 일시정지한 뒤 구역을 변경하세요.')
        zone = rectangular_zone(
            first,
            second,
            robot_radius_m=getattr(
                self.node,
                'runtime_zone_robot_radius_m',
                0.09,
            ),
            clearance_margin_m=getattr(
                self.node,
                'runtime_zone_clearance_margin_m',
                0.01,
            ),
        )
        self.zones = {zone.zone_id: ZoneRuntime(zone)}
        self.summary = (
            f'병목 구역 설정 완료 · {self._zone_geometry(zone)} · '
            '관제 시작을 누르세요.'
        )
        return zone

    def clear_zones(self) -> None:
        """Remove all runtime zones while coordination is paused."""
        if self.enabled:
            raise ValueError('관제를 일시정지한 뒤 구역을 삭제하세요.')
        self.zones.clear()
        self.summary = '병목 구역 없음 · 주행 제한 없이 사용 가능합니다.'

    def select_manual_robot(self, robot_name: str) -> Tuple[bool, str]:
        """Enter manual mode for compatibility with older UI callers."""
        if self.sequence is not None and self.sequence.owns(robot_name):
            return False, '통합 시연을 중단한 뒤 수동 조작으로 전환하세요.'
        if self.emergency:
            return False, '긴급 정지를 먼저 해제하세요.'
        if not self.enabled:
            started, detail = self.start()
            if not started:
                return False, detail
        if not self.node.heartbeat_is_fresh(robot_name):
            return False, f'{robot_name} gate heartbeat가 오래되었습니다.'
        if self.zones and not self.node.pose_is_fresh(robot_name):
            return False, f'{robot_name} 위치가 없거나 오래되었습니다.'
        self.sequence_holds.pop(robot_name, None)
        self.manual_robot = robot_name
        self._request_drive_mode(robot_name, 'MANUAL')
        self._set_manual_routing(robot_name, True)

        self._update_occupancy()
        if self.emergency:
            return False, self.summary
        self._apply_motion_policy()
        self.node.publish_permits_now()
        if robot_name in self.blocked_robots:
            return False, self.robot_details[robot_name]
        self.robot_details[robot_name] = 'RUN · 수동 조작'
        return True, f'{robot_name} 수동 cmd_vel 허용'

    def toggle_manual(self, robot_name: str) -> Tuple[bool, str]:
        """Toggle manual input without discarding an autonomous mission."""
        if self.sequence is not None and self.sequence.owns(robot_name):
            return False, '통합 시연을 중단한 뒤 수동 조작으로 전환하세요.'
        if self.emergency:
            return False, '긴급 정지를 먼저 해제하세요.'
        if not self.enabled:
            started, detail = self.start()
            if not started:
                return False, detail
        if self.manual_robot == robot_name:
            self.manual_robot = None
            self._set_manual_routing(robot_name, False)
            self._request_drive_mode(
                robot_name, self._autonomous_mode_for(robot_name),
            )
            self._apply_motion_policy()
            self.node.publish_permits_now()
            return True, f'{robot_name} 자율주행 복귀'
        if not self.node.heartbeat_is_fresh(robot_name):
            return False, f'{robot_name} gate heartbeat가 오래되었습니다.'
        if self.zones and not self.node.pose_is_fresh(robot_name):
            return False, f'{robot_name} 위치가 없거나 오래되었습니다.'
        if self.manual_robot is not None:
            self._set_manual_routing(self.manual_robot, False)
            self._request_drive_mode(
                self.manual_robot,
                self._autonomous_mode_for(self.manual_robot),
            )
        self.sequence_holds.pop(robot_name, None)
        self.manual_robot = robot_name
        self._set_manual_routing(robot_name, True)
        self._request_drive_mode(robot_name, 'MANUAL')
        self._apply_motion_policy()
        self.node.publish_permits_now()
        return True, f'{robot_name} 수동 조작 전환'

    def submit_goal(
        self,
        robot_name: str,
        x: float,
        y: float,
        yaw_degrees: float,
        *, _owner=None,
    ) -> Tuple[bool, str]:
        """Submit the latest goal while the velocity gate handles conflicts."""
        if self.sequence is not None and self.sequence.owns(robot_name) and _owner is not self.sequence:
            return False, '통합 시연 중입니다. 대기 waypoint를 편집하거나 시연을 중단하세요.'
        if self.emergency:
            return False, '긴급 정지를 먼저 해제하세요.'

        current = self.requests.get(robot_name)
        if current is not None and (current.lane_after_arrival or current.lane_only):
            return False, '진행 중인 차선 임무를 중단한 뒤 새 목표를 지정하세요.'
        if self.lane_navigation is not None and self.lane_navigation.state(robot_name) == 'CANCELLING':
            return False, '차선 임무 중단 확인 중입니다.'
        goal = (x, y, yaw_degrees)
        if not all(math.isfinite(value) for value in goal):
            return False, '잘못된 목표 좌표입니다.'

        self.sequence_holds.pop(robot_name, None)
        request = NavigationRequest(robot_name, goal)
        self.requests[robot_name] = request
        sent = self._send_request(request)
        if self.manual_robot == robot_name:
            self.manual_robot = None
            self._set_manual_routing(robot_name, False)
        self._request_drive_mode(robot_name, 'NAV2')
        if not self.enabled:
            self.start()
        elif self._update_occupancy():
            self._apply_motion_policy()
        self.node.publish_permits_now()

        if robot_name in self.blocked_robots:
            return True, f'{robot_name} 목표 전송 완료 · 병목 경계에서 대기'
        if not sent:
            return True, (
                f'{robot_name} 목표 저장 완료 · Nav2 연결 후 자동 전송'
            )
        return True, f'{robot_name} 목표 전송 완료'

    def submit_lane_entry_goal(
        self,
        robot_name: str,
        x: float,
        y: float,
        yaw_degrees: float,
        exit_pose: Goal,
        route_id: str = 'right_lane',
        next_goal: Optional[Goal] = None,
    ) -> Tuple[bool, str]:
        """Send a Nav2 entry goal that continues as a FollowLane mission."""
        if self.lane_navigation is None:
            return False, '차선 액션 클라이언트가 구성되지 않았습니다.'
        if (len(exit_pose) != 3
                or not all(math.isfinite(value) for value in exit_pose)):
            return False, '잘못된 차선 출구 pose입니다.'
        if next_goal is not None and (len(next_goal) != 3 or not all(math.isfinite(v) for v in next_goal)):
            return False, '잘못된 차선 이후 Nav2 목표입니다.'
        if not route_id:
            return False, 'route_id가 비어 있습니다.'
        accepted, detail = self.submit_goal(
            robot_name, x, y, yaw_degrees,
        )
        if not accepted:
            return accepted, detail
        request = self.requests[robot_name]
        request.lane_after_arrival = True
        request.lane_exit_pose = tuple(exit_pose)
        request.mission_id = f'{robot_name}-{uuid.uuid4().hex}'
        request.route_id = route_id
        request.next_goal = tuple(next_goal) if next_goal is not None else None
        return True, f'{detail} · {route_id} 차선 연속 임무 예약'

    def cancel_robot(
        self,
        robot_name: str,
        *,
        release_unentered: bool = False,
        _owner=None,
    ) -> None:
        """Cancel one navigation request without changing zone ownership."""
        if self.sequence is not None and self.sequence.owns(robot_name) and _owner is not self.sequence:
            self.sequence.cancel('개별 목표 취소로 시연 중단')
            return
        del release_unentered  # Kept for compatibility with older UI calls.
        self.navigation.cancel_goal(robot_name)
        if self.lane_navigation is not None:
            self.lane_navigation.cancel_goal(robot_name)
        self.requests.pop(robot_name, None)
        if self.manual_robot == robot_name:
            self.manual_robot = None
            self._set_manual_routing(robot_name, False)
        self._request_drive_mode(robot_name, 'STOP')
        if self.enabled and not self.emergency:
            self._apply_motion_policy()
        else:
            self.node.set_gate_mode(
                robot_name,
                FleetPermit.MODE_HOLD,
                reason='GOAL_CANCELLED',
            )
        self.robot_details[robot_name] = '목표 취소'
        self.node.publish_permits_now()

    def submit_lane_test(self, robot_name: str, route_id: str = 'lane_test', *, _owner=None) -> Tuple[bool, str]:
        """Start a supervised lane-only test without Nav2 or a guessed exit pose."""
        if self.sequence is not None and self.sequence.owns(robot_name) and _owner is not self.sequence:
            return False, '통합 시연 중입니다. 시연을 중단한 뒤 개별 시험을 시작하세요.'
        if self.emergency:
            return False, '긴급 정지를 먼저 해제하세요.'
        if self.lane_navigation is None:
            return False, '차선 액션 클라이언트가 구성되지 않았습니다.'
        if not self.node.heartbeat_is_fresh(robot_name):
            return False, '로봇 gate heartbeat가 오래되었습니다.'
        if robot_name in self.requests:
            return False, '기존 목표를 취소한 뒤 차선 테스트를 시작하세요.'
        self.sequence_holds.pop(robot_name, None)
        self.navigation.cancel_goal(robot_name)
        self._request_drive_mode(robot_name, 'STOP')
        if self.manual_robot == robot_name:
            self.manual_robot = None
            self._set_manual_routing(robot_name, False)
        request = NavigationRequest(
            robot_name, (0.0, 0.0, 0.0), phase='LANE_PENDING',
            mission_id=f'{robot_name}-lane-test-{uuid.uuid4().hex}',
            route_id=route_id, lane_only=True, lane_duration_sec=900.0,
        )
        self.requests[robot_name] = request
        self._send_lane_request(request)
        if not self.enabled:
            self.start()
        else:
            self._apply_motion_policy()
        self.node.publish_permits_now()
        return True, '차선 단독 테스트 · 출구에서 종료 Bool 전송 · 최대 15분'

    def pause_robot(self, robot_name, paused):
        request = self.requests.get(robot_name)
        if request is None:
            return False, '진행 중인 임무가 없습니다.'
        request.paused = bool(paused)
        if self.enabled and not self.emergency:
            self._apply_motion_policy()
        self.node.publish_permits_now()
        return True, '임무 일시정지' if paused else '임무 재개'

    def tick(self) -> None:
        """Refresh occupancy and velocity permits from actual positions."""
        if not self.enabled or self.emergency:
            if self.sequence is not None:
                self.sequence.tick()
            return
        self._refresh_navigation_requests()
        if not self._update_occupancy():
            return
        self._apply_motion_policy()
        if self.sequence is not None:
            self.sequence.tick()
            if not self.enabled or self.emergency:
                return
            self._apply_motion_policy()
        self.summary = self._make_summary()

    def zone_list(self) -> List[ZoneRuntime]:
        """Return current zone runtimes for UI rendering."""
        return list(self.zones.values())

    # Policy evaluation ----------------------------------------------------

    def _update_occupancy(self) -> bool:
        """Assign ownership only after a robot is physically observed inside."""
        for runtime in self.zones.values():
            if runtime.owner and not self.node.pose_is_fresh(runtime.owner):
                runtime.state = 'LOCKED'
                runtime.detail = f'{runtime.owner} pose stale'
                self.emergency_stop('OWNER_POSE_STALE')
                return False

            overlapping = [
                robot.name
                for robot in ROBOTS
                if self.node.pose_is_fresh(robot.name)
                and self.node.robot_position(robot.name) is not None
                and runtime.config.robot_overlaps(
                    self.node.robot_position(robot.name),
                )
            ]
            if runtime.owner is not None:
                owner_position = self.node.robot_position(runtime.owner)
                owner_in_clearance = (
                    owner_position is not None
                    and runtime.config.robot_in_clearance(owner_position)
                )
                if owner_in_clearance:
                    intruders = [
                        name for name in overlapping
                        if name != runtime.owner
                    ]
                    if intruders:
                        runtime.state = 'CONFLICT'
                        runtime.detail = (
                            f'{runtime.owner} 우선 · '
                            f'{",".join(intruders)} 경계 HOLD'
                        )
                    elif runtime.owner in overlapping:
                        runtime.state = 'OCCUPIED'
                        runtime.detail = f'{runtime.owner} 점유 중'
                    else:
                        runtime.state = 'CLEARING'
                        runtime.detail = f'{runtime.owner} 이탈 확인 중'
                    continue

                runtime.owner = None
                runtime.lease_id = ''

            if not overlapping:
                runtime.state = 'FREE'
                runtime.detail = ''
                continue

            occupant = self._choose_observed_owner(runtime, overlapping)
            runtime.owner = occupant
            runtime.lease_id = f'observed-{uuid.uuid4().hex[:8]}'
            if len(overlapping) > 1:
                waiting = [name for name in overlapping if name != occupant]
                runtime.state = 'CONFLICT'
                runtime.detail = (
                    f'{occupant} 우선 · {",".join(waiting)} 경계 HOLD'
                )
            else:
                runtime.state = 'OCCUPIED'
                runtime.detail = f'{occupant} 점유 중'
        return True

    def _choose_observed_owner(
        self,
        runtime: ZoneRuntime,
        overlapping: List[str],
    ) -> str:
        """Choose one deterministic owner if entries arrive in one tick."""
        centers_inside = [
            name
            for name in overlapping
            if runtime.config.contains_center(
                self.node.robot_position(name),
            )
        ]
        candidates = centers_inside or overlapping
        min_x, min_y, max_x, max_y = runtime.config.bounds
        center_x = (min_x + max_x) / 2.0
        center_y = (min_y + max_y) / 2.0
        return min(
            candidates,
            key=lambda name: (
                (self.node.robot_position(name)[0] - center_x) ** 2
                + (self.node.robot_position(name)[1] - center_y) ** 2
            ),
        )

    def _apply_motion_policy(self) -> None:
        """Hold only robots at an occupied zone's expanded boundary."""
        previous_blocked = set(self.blocked_robots)
        blocked: Set[str] = set()
        reasons: Dict[str, str] = {}

        for robot in ROBOTS:
            name = robot.name
            if not self.node.heartbeat_is_fresh(name):
                blocked.add(name)
                reasons[name] = 'GATE_HEARTBEAT_STALE'
                continue
            if self.zones and not self.node.pose_is_fresh(name):
                blocked.add(name)
                reasons[name] = 'ROBOT_POSE_STALE'
                continue
            position = self.node.robot_position(name)
            if position is None:
                continue
            for runtime in self.zones.values():
                if runtime.owner in (None, name):
                    continue
                if runtime.config.robot_in_clearance(position):
                    blocked.add(name)
                    reasons[name] = (
                        f'{runtime.config.zone_id}_OWNED_BY_{runtime.owner}'
                    )
                    break

        self.blocked_robots = blocked
        for robot in ROBOTS:
            name = robot.name
            request = self.requests.get(name)
            if name in self.sequence_holds:
                self.node.set_gate_mode(name, FleetPermit.MODE_HOLD, reason=self.sequence_holds[name])
                self.robot_details[name] = 'HOLD · ' + self.sequence_holds[name]
                continue
            if request is not None and request.paused:
                self.node.set_gate_mode(name, FleetPermit.MODE_HOLD, reason='OPERATOR_PAUSED')
                self.robot_details[name] = '일시정지 · 목표 유지'
                continue
            if request is not None and request.phase in {
                'LANE_PENDING', 'WAITING_FOR_LANE', 'RELOCALIZING',
                'LANE_FAILED', 'LANE_STOPPING',
            } and self.manual_robot != name:
                self.node.set_gate_mode(
                    name,
                    FleetPermit.MODE_HOLD,
                    reason=request.phase,
                )
                self.robot_details[name] = f'HOLD · {request.phase}'
                continue
            if name in blocked:
                if (
                    request is not None
                    and request.phase == 'E_STOP_HOLD'
                ):
                    self._send_request(request)
                self.node.set_gate_mode(
                    name,
                    FleetPermit.MODE_HOLD,
                    reason=reasons[name],
                )
                if request is not None:
                    if not request.phase.startswith('LANE') and request.phase not in {
                        'WAITING_FOR_LANE', 'SAFETY_HOLD', 'RELOCALIZING',
                    }:
                        request.phase = 'BOUNDARY_HOLD'
                self.robot_details[name] = f'HOLD · {reasons[name]}'
                continue

            if self.manual_robot == name:
                self.node.set_gate_mode(
                    name,
                    FleetPermit.MODE_RUN,
                    reason='MANUAL',
                )
                self.robot_details[name] = 'RUN · 수동 조작'
                continue

            owned_zones = [
                runtime.config.zone_id
                for runtime in self.zones.values()
                if runtime.owner == name
            ]
            lease_id = next(
                (
                    runtime.lease_id
                    for runtime in self.zones.values()
                    if runtime.owner == name
                ),
                '',
            )
            self.node.set_gate_mode(
                name,
                FleetPermit.MODE_RUN,
                lease_id=lease_id,
                allowed_zone_ids=owned_zones,
                reason='POSITION_CLEAR',
            )

            if request is not None:
                if request.phase in {'LANE_ACTIVE', 'SAFETY_HOLD'}:
                    self.robot_details[name] = (
                        'RUN · 차선 주행'
                        if request.phase == 'LANE_ACTIVE'
                        else 'RUN · 차선 안전 정지'
                    )
                    continue
                if name in previous_blocked:
                    nav_state = self.navigation.state(name)
                    if (
                        not request.delivered
                        or nav_state not in self._RUNNING_NAV_STATES
                    ):
                        self._send_request(request)
                request.phase = (
                    'ACTIVE' if request.delivered else 'NAV_PENDING'
                )
                self.robot_details[name] = (
                    'RUN · Nav2 목표 주행'
                    if request.delivered
                    else 'RUN · Nav2 연결 후 목표 자동 전송'
                )
            elif self.manual_robot == name:
                self.robot_details[name] = 'RUN · 수동 조작'
            else:
                self.robot_details[name] = 'RUN · 주행 가능'

    # Retained Nav2 request lifecycle -------------------------------------

    def _refresh_navigation_requests(self) -> None:
        """Retry retained goals and discard goals that have completed."""
        for name, request in list(self.requests.items()):
            if request.paused:
                continue
            if request.phase in {
                'LANE_PENDING', 'WAITING_FOR_LANE', 'LANE_ACTIVE',
                'SAFETY_HOLD', 'RELOCALIZING', 'LANE_FAILED', 'LANE_STOPPING',
            }:
                self._refresh_lane_request(name, request)
                continue
            nav_state = self.navigation.state(name)
            if request.delivered and nav_state == 'SUCCEEDED':
                if request.lane_after_arrival:
                    if self.manual_robot == name:
                        continue
                    checker = getattr(self.node, 'arrival_is_close', None)
                    if checker is not None and not checker(name, request.goal):
                        request.phase = 'LANE_FAILED'
                        self._request_drive_mode(name, 'STOP')
                        self.robot_details[name] = '입구 위치/방향 확인 실패 · 재배치 후 재시작'
                        continue
                    request.phase = 'LANE_PENDING'
                    request.transition_started = time.monotonic()
                    self._request_drive_mode(name, 'STOP')
                    self.node.set_gate_mode(
                        name, FleetPermit.MODE_HOLD, reason='LANE_TRANSITION',
                    )
                    self._send_lane_request(request)
                else:
                    request.phase = 'SUCCEEDED'
                    self.requests.pop(name, None)
                    if self.sequence is not None and self.sequence.owns(name):
                        self.sequence_holds[name] = 'DEMO_STEP_COMPLETE'
                        self._request_drive_mode(name, 'STOP')
                continue
            if (
                not request.delivered
                and time.monotonic() - request.last_send_attempt >= 1.0
            ):
                self._send_request(request)

    def _send_lane_request(self, request: NavigationRequest) -> bool:
        """Try to start a retained FollowLane goal without blocking."""
        if self.lane_navigation is None:
            request.phase = 'LANE_FAILED'
            return False
        request.last_send_attempt = time.monotonic()
        request.lane_sent = self.lane_navigation.send_goal(
            request.robot_name,
            request.mission_id,
            request.route_id,
            max_duration_sec=request.lane_duration_sec,
        )
        request.phase = 'WAITING_FOR_LANE' if request.lane_sent else 'LANE_PENDING'
        return request.lane_sent

    def _refresh_lane_request(
        self,
        name: str,
        request: NavigationRequest,
    ) -> None:
        """Advance lane action, relocalization, and return-to-Nav2 states."""
        if request.phase == 'RELOCALIZING':
            ready = (self.node.pose_is_fresh(name) and
                     self.lane_navigation.localization_ready(name, request.transition_started, request.lane_exit_pose))
            if ready and self.manual_robot != name:
                if request.next_goal is None:
                    self._request_drive_mode(name, 'NAV2')
                    self.requests.pop(name, None)
                    self.robot_details[name] = '출구 위치 확인 · Nav2 준비됨'
                else:
                    request.goal = request.next_goal
                    request.next_goal = None
                    request.lane_after_arrival = False
                    request.phase, request.delivered = 'NAV_PENDING', False
                    self._request_drive_mode(name, 'NAV2')
                    self._send_request(request)
            elif time.monotonic() - request.transition_started > 20.:
                request.phase = 'LANE_FAILED'
                self._request_drive_mode(name, 'STOP')
                self.robot_details[name] = '출구 위치 추정 확인 시간 초과 · STOP'
            return
        if request.phase == 'LANE_FAILED':
            return
        if not request.lane_sent:
            if time.monotonic() - request.last_send_attempt >= 1.0:
                self._send_lane_request(request)
            return
        status_reader = getattr(self.lane_navigation, 'telemetry', None)
        if status_reader is not None and request.lane_sent and time.monotonic() - request.last_send_attempt > 3.:
            if not status_reader(name).get('fresh'):
                self.lane_navigation.cancel_goal(name)
                self._request_drive_mode(name, 'STOP')
                request.phase = 'LANE_FAILED'
                self.robot_details[name] = '차선 임무 서버 통신 끊김 · STOP'
                return
        state = self.lane_navigation.state(name)
        if state == 'WAITING_FOR_LANE' or state == 'PENDING':
            request.phase = 'WAITING_FOR_LANE'
        elif state == 'FOLLOWING':
            request.phase = 'LANE_ACTIVE'
            if self.manual_robot != name:
                self._request_drive_mode(name, 'LANE')
        elif state == 'SAFETY_HOLD':
            request.phase = 'SAFETY_HOLD'
        elif state in {'STOPPING', 'CANCELLING'}:
            request.phase = 'LANE_STOPPING'
            self._request_drive_mode(name, 'STOP')
        elif state == 'SUCCEEDED':
            if request.lane_only:
                self._request_drive_mode(name, 'STOP')
                self.node.set_gate_mode(name, FleetPermit.MODE_HOLD, reason='LANE_TEST_COMPLETE')
                request.phase = 'SUCCEEDED'
                self.requests.pop(name, None)
                if self.sequence is not None and self.sequence.owns(name):
                    self.sequence_holds[name] = 'DEMO_LANE_COMPLETE'
                self.robot_details[name] = '차선 테스트 완료 · STOP · AMCL 유지'
                return
            if request.lane_exit_pose is None:
                request.phase = 'LANE_FAILED'
                return
            if self.manual_robot != name:
                self._request_drive_mode(name, 'STOP')
            self.node.set_gate_mode(
                name, FleetPermit.MODE_HOLD, reason='LANE_COMPLETE',
            )
            self.lane_navigation.mark_localization_reset(name)
            self.node.publish_initial_pose(
                name,
                *request.lane_exit_pose,
                position_variance=self.node.lane_exit_position_variance,
                yaw_variance=self.node.lane_exit_yaw_variance,
            )
            request.phase = 'RELOCALIZING'
            request.transition_started = time.monotonic()
        elif state in {'ABORTED', 'CANCELED', 'REJECTED', 'ERROR'}:
            self._request_drive_mode(name, 'STOP')
            request.phase = 'LANE_FAILED'

    def _request_drive_mode(self, robot_name: str, mode: str) -> None:
        """Use mode control when available while preserving test doubles."""
        request = getattr(self.node, 'request_drive_mode', None)
        if request is not None:
            request(robot_name, mode)

    def _set_manual_routing(self, robot_name: str, enabled: bool) -> None:
        """Keep shared teleop routing aligned with the coordinator mode."""
        setter = getattr(self.node, 'set_manual_routing', None)
        if setter is not None:
            setter(robot_name, enabled)

    def _autonomous_mode_for(self, robot_name: str) -> str:
        """Return the absolute autonomous mode hidden by manual override."""
        request = self.requests.get(robot_name)
        if request is None:
            return 'NAV2'
        if request.phase in {'LANE_ACTIVE', 'SAFETY_HOLD'}:
            return 'LANE'
        if request.phase in {
            'LANE_PENDING', 'WAITING_FOR_LANE', 'RELOCALIZING',
            'LANE_FAILED', 'LANE_STOPPING',
        }:
            return 'STOP'
        return 'NAV2'

    def _send_request(self, request: NavigationRequest) -> bool:
        """Attempt one nonblocking Nav2 send and retain failure for retry."""
        request.last_send_attempt = time.monotonic()
        request.delivered = self.navigation.send_goal(
            request.robot_name,
            *request.goal,
        )
        request.phase = 'ACTIVE' if request.delivered else 'NAV_PENDING'
        return request.delivered

    def _hold_all(self, reason: str) -> None:
        for robot in ROBOTS:
            self.node.set_gate_mode(
                robot.name,
                FleetPermit.MODE_HOLD,
                reason=reason,
            )
            self.robot_details[robot.name] = f'HOLD · {reason}'

    def _make_summary(self) -> str:
        if not self.zones:
            return '관제 실행 중 · 병목 구역 없음'
        parts = []
        for runtime in self.zones.values():
            owner = runtime.owner or '-'
            parts.append(
                f'{runtime.config.zone_id}: {runtime.state} owner={owner} · '
                f'{self._zone_geometry(runtime.config)}',
            )
        if self.blocked_robots:
            parts.append(f'HOLD={",".join(sorted(self.blocked_robots))}')
        return ' | '.join(parts)

    @staticmethod
    def _zone_geometry(zone: BottleneckZone) -> str:
        min_x, min_y, max_x, max_y = zone.bounds
        return (
            f'x=[{min_x:.2f},{max_x:.2f}] '
            f'y=[{min_y:.2f},{max_y:.2f}] '
            f'R={zone.robot_radius_m:.2f}m '
            f'M={zone.clearance_margin_m:.2f}m '
            f'판정={zone.effective_clearance_m:.2f}m'
        )
