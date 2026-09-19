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

    def __init__(self, node, navigation, zones: List[BottleneckZone]) -> None:
        self.node = node
        self.navigation = navigation
        self.zones: Dict[str, ZoneRuntime] = {
            zone.zone_id: ZoneRuntime(zone)
            for zone in zones
        }
        self.requests: Dict[str, NavigationRequest] = {}
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
            request.phase = 'E_STOP_HOLD'
            request.delivered = False
        for robot in ROBOTS:
            self.navigation.cancel_goal(robot.name)
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
        """Select a teleoperation target and enable supervision if needed."""
        if self.emergency:
            return False, '긴급 정지를 먼저 해제하세요.'
        if not self.enabled:
            started, detail = self.start()
            if not started:
                return False, detail
        if robot_name in self.requests:
            self.navigation.cancel_goal(robot_name)
            self.requests.pop(robot_name, None)
        self.manual_robot = robot_name
        if not self.node.heartbeat_is_fresh(robot_name):
            return False, f'{robot_name} gate heartbeat가 오래되었습니다.'
        if self.zones and not self.node.pose_is_fresh(robot_name):
            return False, f'{robot_name} 위치가 없거나 오래되었습니다.'

        self._update_occupancy()
        if self.emergency:
            return False, self.summary
        self._apply_motion_policy()
        self.node.publish_permits_now()
        if robot_name in self.blocked_robots:
            return False, self.robot_details[robot_name]
        self.robot_details[robot_name] = 'RUN · 수동 조작'
        return True, f'{robot_name} 수동 cmd_vel 허용'

    def submit_goal(
        self,
        robot_name: str,
        x: float,
        y: float,
        yaw_degrees: float,
    ) -> Tuple[bool, str]:
        """Submit the latest goal while the velocity gate handles conflicts."""
        if self.emergency:
            return False, '긴급 정지를 먼저 해제하세요.'

        goal = (x, y, yaw_degrees)
        if not all(math.isfinite(value) for value in goal):
            return False, '잘못된 목표 좌표입니다.'

        request = NavigationRequest(robot_name, goal)
        self.requests[robot_name] = request
        sent = self._send_request(request)
        if self.manual_robot == robot_name:
            self.manual_robot = None
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

    def cancel_robot(
        self,
        robot_name: str,
        *,
        release_unentered: bool = False,
    ) -> None:
        """Cancel one navigation request without changing zone ownership."""
        del release_unentered  # Kept for compatibility with older UI calls.
        self.navigation.cancel_goal(robot_name)
        self.requests.pop(robot_name, None)
        if self.manual_robot == robot_name:
            self.manual_robot = None
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

    def tick(self) -> None:
        """Refresh occupancy and velocity permits from actual positions."""
        if not self.enabled or self.emergency:
            return
        self._refresh_navigation_requests()
        if not self._update_occupancy():
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
                    request.phase = 'BOUNDARY_HOLD'
                self.robot_details[name] = f'HOLD · {reasons[name]}'
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
            nav_state = self.navigation.state(name)
            if request.delivered and nav_state == 'SUCCEEDED':
                self.requests.pop(name, None)
                continue
            if (
                not request.delivered
                and time.monotonic() - request.last_send_attempt >= 1.0
            ):
                self._send_request(request)

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
