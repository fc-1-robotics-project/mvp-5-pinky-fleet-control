"""Tests for position-based bottleneck motion gating."""

from multibot_control_ui.fleet_coordinator import FleetCoordinator
from multibot_control_ui.zone_config import rectangular_zone
from pinky_interfaces.msg import FleetPermit
import pytest


class FakeNode:

    def __init__(self):
        self.positions = {'robot1': (-1.0, 0.25), 'robot2': (2.0, 0.25)}
        self.heartbeat_fresh = {'robot1': True, 'robot2': True}
        self.pose_fresh = {'robot1': True, 'robot2': True}
        self.runtime_zone_robot_radius_m = 0.09
        self.runtime_zone_clearance_margin_m = 0.01
        self.modes = {}
        self.permit_count = 0

    def heartbeat_is_fresh(self, robot_name):
        return self.heartbeat_fresh[robot_name]

    def pose_is_fresh(self, robot_name):
        return self.pose_fresh[robot_name]

    def robot_position(self, robot_name):
        return self.positions.get(robot_name)

    def set_gate_mode(
        self,
        robot_name,
        mode,
        *,
        lease_id='',
        allowed_zone_ids=None,
        reason='',
    ):
        self.modes[robot_name] = (
            mode,
            lease_id,
            list(allowed_zone_ids or []),
            reason,
        )

    def publish_permits_now(self):
        self.permit_count += 1


class FakeNavigation:

    def __init__(self):
        self.sent = []
        self.cancelled = []
        self.available = True
        self.states = {'robot1': 'READY', 'robot2': 'READY'}

    def send_goal(self, robot_name, x, y, yaw):
        if not self.available:
            self.states[robot_name] = 'UNAVAILABLE'
            return False
        self.sent.append((robot_name, x, y, yaw))
        self.states[robot_name] = 'ACTIVE'
        return True

    def cancel_goal(self, robot_name):
        self.cancelled.append(robot_name)
        self.states[robot_name] = 'CANCELED'
        return True

    def status(self, robot_name):
        return self.states[robot_name]

    def state(self, robot_name):
        return self.states[robot_name]


def make_coordinator():
    node = FakeNode()
    navigation = FakeNavigation()
    zone = rectangular_zone((0.0, 0.0), (1.0, 0.5))
    coordinator = FleetCoordinator(node, navigation, [zone])
    assert coordinator.start()[0]
    return coordinator, node, navigation


def test_goals_are_sent_without_pre_reserving_the_zone() -> None:
    coordinator, node, navigation = make_coordinator()

    assert coordinator.submit_goal('robot1', 2.0, 0.25, 0.0)[0]
    assert coordinator.submit_goal('robot2', -1.0, 0.25, 180.0)[0]

    runtime = coordinator.zone_list()[0]
    assert runtime.owner is None
    assert runtime.state == 'FREE'
    assert navigation.sent == [
        ('robot1', 2.0, 0.25, 0.0),
        ('robot2', -1.0, 0.25, 180.0),
    ]
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN
    assert node.modes['robot2'][0] == FleetPermit.MODE_RUN


def test_ui_rectangle_uses_runtime_clearance_parameters() -> None:
    node = FakeNode()
    coordinator = FleetCoordinator(node, FakeNavigation(), [])

    zone = coordinator.set_runtime_rectangle((1.2, -0.4), (0.2, 0.6))

    assert zone.bounds == (0.2, -0.4, 1.2, 0.6)
    assert zone.robot_radius_m == 0.09
    assert zone.clearance_margin_m == 0.01
    assert zone.effective_clearance_m == pytest.approx(0.10)
    assert 'x=[0.20,1.20]' in coordinator.summary
    assert '판정=0.10m' in coordinator.summary


def test_later_robot_holds_only_when_it_reaches_occupied_boundary() -> None:
    coordinator, node, _ = make_coordinator()
    node.positions['robot1'] = (0.5, 0.25)
    node.positions['robot2'] = (1.5, 0.25)
    coordinator.tick()

    runtime = coordinator.zone_list()[0]
    assert runtime.owner == 'robot1'
    assert runtime.state == 'OCCUPIED'
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN
    assert node.modes['robot2'][0] == FleetPermit.MODE_RUN

    node.positions['robot2'] = (1.095, 0.25)
    coordinator.tick()
    assert node.modes['robot2'][0] == FleetPermit.MODE_HOLD
    assert 'OWNED_BY_robot1' in node.modes['robot2'][3]


def test_waiting_robot_resumes_existing_goal_after_owner_clears() -> None:
    coordinator, node, navigation = make_coordinator()
    coordinator.submit_goal('robot2', -1.0, 0.25, 180.0)
    node.positions['robot1'] = (0.5, 0.25)
    node.positions['robot2'] = (1.095, 0.25)
    coordinator.tick()
    assert coordinator.requests['robot2'].phase == 'BOUNDARY_HOLD'

    node.positions['robot1'] = (1.11, 0.25)
    coordinator.tick()

    assert coordinator.zone_list()[0].state == 'FREE'
    assert node.modes['robot2'][0] == FleetPermit.MODE_RUN
    assert coordinator.requests['robot2'].phase == 'ACTIVE'
    assert navigation.sent.count(('robot2', -1.0, 0.25, 180.0)) == 1


def test_aborted_goal_is_reissued_when_boundary_hold_is_released() -> None:
    coordinator, node, navigation = make_coordinator()
    coordinator.submit_goal('robot2', -1.0, 0.25, 180.0)
    node.positions['robot1'] = (0.5, 0.25)
    node.positions['robot2'] = (1.095, 0.25)
    coordinator.tick()
    navigation.states['robot2'] = 'ABORTED'

    node.positions['robot1'] = (1.11, 0.25)
    coordinator.tick()

    assert navigation.sent.count(('robot2', -1.0, 0.25, 180.0)) == 2


def test_paused_goal_is_reissued_if_nav2_aborts_before_resume() -> None:
    coordinator, _, navigation = make_coordinator()
    coordinator.submit_goal('robot1', 2.0, 0.25, 0.0)
    coordinator.pause()
    navigation.states['robot1'] = 'ABORTED'

    coordinator.start()

    assert navigation.sent.count(('robot1', 2.0, 0.25, 0.0)) == 2


def test_estop_retains_goal_and_reissues_it_on_restart() -> None:
    coordinator, node, navigation = make_coordinator()
    goal = ('robot1', 2.0, 0.25, 0.0)
    coordinator.submit_goal(*goal)

    coordinator.emergency_stop()

    request = coordinator.requests['robot1']
    assert request.goal == goal[1:]
    assert request.phase == 'E_STOP_HOLD'
    assert not request.delivered
    assert node.modes['robot1'][0] == FleetPermit.MODE_ESTOP
    assert navigation.cancelled == ['robot1', 'robot2']

    success, _ = coordinator.start()

    assert success
    assert navigation.sent.count(goal) == 2
    assert coordinator.requests['robot1'].phase == 'ACTIVE'
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN


def test_estop_restart_reissues_goal_even_if_robot_remains_held() -> None:
    coordinator, node, navigation = make_coordinator()
    goal = ('robot2', -1.0, 0.25, 180.0)
    coordinator.submit_goal(*goal)
    node.positions['robot1'] = (0.5, 0.25)
    node.positions['robot2'] = (1.095, 0.25)
    coordinator.tick()

    coordinator.emergency_stop()
    success, _ = coordinator.start()

    assert success
    assert navigation.sent.count(goal) == 2
    assert coordinator.requests['robot2'].phase == 'BOUNDARY_HOLD'
    assert node.modes['robot2'][0] == FleetPermit.MODE_HOLD


def test_new_goal_is_sent_and_replaced_even_while_held() -> None:
    coordinator, node, navigation = make_coordinator()
    node.positions['robot1'] = (0.5, 0.25)
    node.positions['robot2'] = (1.095, 0.25)
    coordinator.tick()

    sent, _ = coordinator.submit_goal('robot2', -2.0, 0.5, 90.0)

    assert sent
    assert navigation.sent[-1] == ('robot2', -2.0, 0.5, 90.0)
    assert coordinator.requests['robot2'].goal == (-2.0, 0.5, 90.0)
    assert node.modes['robot2'][0] == FleetPermit.MODE_HOLD


def test_stale_pose_does_not_prevent_goal_submission() -> None:
    coordinator, node, navigation = make_coordinator()
    node.pose_fresh['robot1'] = False

    sent, _ = coordinator.submit_goal('robot1', 2.0, 0.25, 0.0)

    assert sent
    assert navigation.sent[-1] == ('robot1', 2.0, 0.25, 0.0)
    assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD


def test_unavailable_nav2_retains_latest_goal_for_retry() -> None:
    coordinator, _, navigation = make_coordinator()
    navigation.available = False

    accepted, detail = coordinator.submit_goal('robot1', 2.0, 0.25, 0.0)

    assert accepted
    assert '자동 전송' in detail
    assert coordinator.requests['robot1'].phase == 'NAV_PENDING'

    navigation.available = True
    coordinator.requests['robot1'].last_send_attempt = 0.0
    coordinator.tick()
    assert navigation.sent[-1] == ('robot1', 2.0, 0.25, 0.0)
    assert coordinator.requests['robot1'].delivered


def test_manual_selection_enables_control_and_run_gate() -> None:
    node = FakeNode()
    coordinator = FleetCoordinator(node, FakeNavigation(), [])

    allowed, _ = coordinator.select_manual_robot('robot1')

    assert allowed
    assert coordinator.enabled
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN


def test_missing_heartbeat_holds_only_that_robot() -> None:
    node = FakeNode()
    node.heartbeat_fresh['robot2'] = False
    coordinator = FleetCoordinator(
        node,
        FakeNavigation(),
        [rectangular_zone((0.0, 0.0), (1.0, 0.5))],
    )

    success, _ = coordinator.start()

    assert success
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN
    assert node.modes['robot2'][0] == FleetPermit.MODE_HOLD


def test_boundary_intrusion_holds_only_later_robot() -> None:
    coordinator, node, navigation = make_coordinator()
    coordinator.submit_goal('robot1', 2.0, 0.25, 0.0)
    coordinator.submit_goal('robot2', -1.0, 0.25, 180.0)
    node.positions['robot1'] = (0.4, 0.25)
    node.positions['robot2'] = (1.08, 0.25)

    coordinator.tick()

    runtime = coordinator.zone_list()[0]
    assert not coordinator.emergency
    assert runtime.state == 'CONFLICT'
    assert runtime.owner == 'robot1'
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN
    assert node.modes['robot2'][0] == FleetPermit.MODE_HOLD
    assert navigation.states['robot1'] == 'ACTIVE'
    assert navigation.states['robot2'] == 'ACTIVE'


def test_owner_transfers_to_waiting_intruder_after_first_robot_clears() -> None:
    coordinator, node, _ = make_coordinator()
    node.positions['robot1'] = (0.4, 0.25)
    node.positions['robot2'] = (1.08, 0.25)
    coordinator.tick()

    node.positions['robot1'] = (-0.11, 0.25)
    coordinator.tick()

    runtime = coordinator.zone_list()[0]
    assert not coordinator.emergency
    assert runtime.owner == 'robot2'
    assert runtime.state == 'OCCUPIED'
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN
    assert node.modes['robot2'][0] == FleetPermit.MODE_RUN


def test_same_tick_entry_selects_one_owner_without_estop() -> None:
    coordinator, node, _ = make_coordinator()
    node.positions['robot1'] = (0.4, 0.25)
    node.positions['robot2'] = (0.6, 0.25)

    coordinator.tick()

    runtime = coordinator.zone_list()[0]
    assert not coordinator.emergency
    assert runtime.state == 'CONFLICT'
    assert runtime.owner == 'robot1'
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN
    assert node.modes['robot2'][0] == FleetPermit.MODE_HOLD
