"""HOLD must stop action timeouts and wait for terminal results before retry."""
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from pinky_interfaces.msg import FleetPermit

from multibot_control_ui.fleet_coordinator import FleetCoordinator
from multibot_control_ui.navigation_client import RobotNavigationClient
from multibot_control_ui.zone_config import rectangular_zone
from test_fleet_coordinator import FakeNode, FakeNavigation, FakeLaneNavigation


class AsyncNavigation(FakeNavigation):
    def cancel_goal(self, name):
        self.cancelled.append(name)
        self.states[name] = 'CANCELLING'
        return True


@pytest.mark.parametrize('hold', ['global', 'robot', 'heartbeat'])
def test_hold_cancels_once_and_resume_waits_for_terminal_result(hold):
    node, nav = FakeNode(), AsyncNavigation()
    fleet = FleetCoordinator(node, nav, [])
    goal = ('robot1', 2., .25, 0.)
    fleet.submit_goal(*goal)
    original = fleet.requests['robot1']
    if hold == 'global':
        fleet.pause()
        fleet.start()
    elif hold == 'robot':
        fleet.pause_robot('robot1', True)
        fleet.pause_robot('robot1', False)
    else:
        node.heartbeat_fresh['robot1'] = False
        fleet.tick()
        node.heartbeat_fresh['robot1'] = True
    for _ in range(4):
        fleet.tick()
    assert nav.cancelled == ['robot1']
    assert nav.sent == [goal]
    assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD
    nav.states['robot1'] = 'CANCELED'
    fleet.tick()
    fleet.tick()
    assert nav.sent == [goal, goal]
    assert fleet.requests['robot1'] is original
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN


def test_success_arriving_during_hold_is_never_resent():
    node, nav = FakeNode(), AsyncNavigation()
    fleet = FleetCoordinator(node, nav, [])
    fleet.submit_goal('robot1', 2., 0., 0.)
    fleet.pause()
    nav.states['robot1'] = 'SUCCEEDED'
    fleet.start()
    fleet.tick()
    assert len(nav.sent) == 1
    assert 'robot1' not in fleet.requests


def nav_client(state='PENDING'):
    client = RobotNavigationClient.__new__(RobotNavigationClient)
    client.lock = Lock()
    client._state, client._status = state, ''
    client._goal_generation = 1
    client._goal_handle = None
    client._cancel_requested = False
    client._last_goal = None
    client.action_client = Mock()
    client.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=Time)))
    return client


def test_pending_nav_goal_is_cancelled_on_acceptance_then_waits_for_result():
    client = nav_client()
    assert client.cancel_goal()
    assert client.state() == 'CANCELLING'
    client._cancel_done_callback(SimpleNamespace(result=lambda: SimpleNamespace(goals_canceling=[1])), 1)
    assert client.state() == 'CANCELLING'
    handle = Mock(accepted=True)
    client._goal_response_callback(SimpleNamespace(result=lambda: handle), 1)
    handle.cancel_goal_async.assert_called_once()
    handle.get_result_async.assert_called_once()
    assert client.state() == 'CANCELLING'
    assert not client.send_goal(1., 2., 0.)
    response = SimpleNamespace(status=GoalStatus.STATUS_CANCELED,
                               result=SimpleNamespace(error_code=NavigateToPose.Result.NONE))
    client._result_callback(SimpleNamespace(result=lambda: response), 1)
    assert client.state() == 'CANCELED'
    assert client.send_goal(1., 2., 0.)




def test_boundary_hold_resumes_only_after_cancel_result_and_does_not_repeat():
    node, nav = FakeNode(), AsyncNavigation()
    fleet = FleetCoordinator(node, nav, [rectangular_zone((0., 0.), (1., .5))])
    goal = ('robot2', -1., .25, 180.)
    fleet.submit_goal(*goal)
    node.positions.update(robot1=(.5, .25), robot2=(1.095, .25))
    fleet.tick()
    node.positions['robot1'] = (1.2, .25)
    fleet.tick()
    assert nav.sent == [goal]
    assert node.modes['robot2'][0] == FleetPermit.MODE_HOLD
    nav.states['robot2'] = 'CANCELED'
    fleet.tick()
    fleet.tick()
    assert nav.sent == [goal, goal]
    assert nav.cancelled == ['robot2']


@pytest.mark.parametrize('explicit_stop', ['cancel', 'estop'])
def test_explicit_stop_while_cancelling_cannot_automatically_restart(explicit_stop):
    node, nav = FakeNode(), AsyncNavigation()
    fleet = FleetCoordinator(node, nav, [])
    fleet.submit_goal('robot1', 2., 0., 0.)
    fleet.pause_robot('robot1', True)
    if explicit_stop == 'cancel':
        fleet.cancel_robot('robot1')
    else:
        fleet.emergency_stop()
    nav.states['robot1'] = 'CANCELED'
    for _ in range(4):
        fleet.tick()
    assert len(nav.sent) == 1


def test_goal_replacement_waits_for_old_result_before_single_send():
    node, nav = FakeNode(), AsyncNavigation()
    fleet = FleetCoordinator(node, nav, [])
    old, new = ('robot1', 2., 0., 0.), ('robot1', 3., 1., 90.)
    fleet.submit_goal(*old)
    fleet.submit_goal(*new)
    fleet.tick()
    assert nav.sent == [old]
    assert nav.cancelled == ['robot1']
    assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD
    nav.states['robot1'] = 'CANCELED'
    fleet.tick()
    fleet.tick()
    assert nav.sent == [old, new]




@pytest.mark.parametrize('failure', ['rejected', 'exception'])
def test_failed_cancel_never_allows_new_goal_until_terminal_result(failure):
    client = nav_client('ACTIVE')
    client._goal_handle = Mock()
    assert client.cancel_goal()
    def response():
        if failure == 'exception':
            raise RuntimeError('cancel transport failed')
        return SimpleNamespace(goals_canceling=[])
    client._cancel_done_callback(SimpleNamespace(result=response), 1)
    assert client.state() == 'CANCELLING'
    assert client._goal_handle is not None
    assert not client.send_goal(1., 2., 0.)


def test_late_cancel_exception_cannot_overwrite_terminal_success():
    client = nav_client('ACTIVE')
    client._goal_handle = Mock()
    assert client.cancel_goal()
    response = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,
                               result=SimpleNamespace(error_code=NavigateToPose.Result.NONE))
    client._result_callback(SimpleNamespace(result=lambda: response), 1)
    def failed_cancel():
        raise RuntimeError('late cancel transport error')
    client._cancel_done_callback(SimpleNamespace(result=failed_cancel), 1)
    assert client.state() == 'SUCCEEDED'
    assert client.status() == '목표 도착 완료'


@pytest.mark.parametrize('operation', ['send', 'cancel'])
def test_synchronous_action_transport_error_holds_without_retransmission(operation):
    client = nav_client('READY' if operation == 'send' else 'ACTIVE')
    if operation == 'send':
        client.action_client.send_goal_async.side_effect = RuntimeError('send failed')
        assert not client.send_goal(1., 2., 0.)
    else:
        client._goal_handle = Mock()
        client._goal_handle.cancel_goal_async.side_effect = RuntimeError('cancel failed')
        assert client.cancel_goal()
    assert client.state() == 'CANCELLING'
    assert not client.send_goal(1., 2., 0.)


def test_successful_lane_entry_during_hold_clears_nav_suspension_before_lane_run():
    node, nav, lane = FakeNode(), AsyncNavigation(), FakeLaneNavigation()
    fleet = FleetCoordinator(node, nav, [], lane)
    fleet.submit_lane_entry_goal('robot1', 2., 0., 0., (3., 0., 0.))
    fleet.pause()
    fleet.start()
    assert nav.states['robot1'] == 'CANCELLING'
    assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD
    assert not lane.sent
    nav.states['robot1'] = 'SUCCEEDED'  # Success races with the cancellation request.
    fleet.tick()
    assert len(nav.sent) == 1
    assert len(lane.sent) == 1
    lane.states['robot1'] = 'FOLLOWING'
    fleet.tick()
    assert fleet.requests['robot1'].phase == 'LANE_ACTIVE'
    assert not fleet.requests['robot1'].nav_suspended
    assert node.drive_modes['robot1'] == 'LANE'
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN


@pytest.mark.parametrize('manual_api', ['select_manual_robot', 'toggle_manual'])
@pytest.mark.parametrize('terminal_before_select', [True, False])
@pytest.mark.parametrize('terminal', ['CANCELED', 'SUCCEEDED'])
def test_manual_after_nav_hold_never_sends_nav_and_restores_manual_mode(
        manual_api, terminal_before_select, terminal):
    node, nav = FakeNode(), AsyncNavigation()
    fleet = FleetCoordinator(node, nav, [])
    goal = ('robot1', 2., 0., 0.)
    fleet.submit_goal(*goal)
    fleet.pause()
    if terminal_before_select:
        nav.states['robot1'] = terminal
    ok, detail = getattr(fleet, manual_api)('robot1')
    assert ok
    assert nav.sent == [goal]
    if not terminal_before_select:
        assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD
        assert '확인' in detail
        nav.states['robot1'] = terminal
    for _ in range(3):
        fleet.tick()
    assert fleet.manual_robot == 'robot1'
    assert node.manual_routing['robot1'] is True
    assert node.drive_modes['robot1'] == 'MANUAL'
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN
    assert nav.sent == [goal]
    fleet.toggle_manual('robot1')
    fleet.tick()
    assert node.manual_routing['robot1'] is False
    assert nav.sent == ([goal, goal] if terminal == 'CANCELED' else [goal])


@pytest.mark.parametrize('manual_api', ['select_manual_robot', 'toggle_manual'])
def test_manual_off_before_cancel_result_keeps_hold_then_resumes_once(manual_api):
    node, nav = FakeNode(), AsyncNavigation()
    fleet = FleetCoordinator(node, nav, [])
    goal = ('robot1', 2., 0., 0.)
    fleet.submit_goal(*goal)
    fleet.pause()
    getattr(fleet, manual_api)('robot1')
    ok, detail = fleet.toggle_manual('robot1')
    assert ok and '확인' in detail
    fleet.tick()
    assert fleet.manual_robot is None
    assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD
    assert nav.sent == [goal]
    nav.states['robot1'] = 'CANCELED'
    fleet.tick()
    fleet.tick()
    assert nav.sent == [goal, goal]
    assert node.drive_modes['robot1'] == 'NAV2'


@pytest.mark.parametrize('next_action', ['cancel', 'new_goal', 'heartbeat_stale'])
def test_manual_cancel_pending_keeps_explicit_action_and_safety_policy(next_action):
    node, nav = FakeNode(), AsyncNavigation()
    fleet = FleetCoordinator(node, nav, [])
    old, new = ('robot1', 2., 0., 0.), ('robot1', 3., 1., 90.)
    fleet.submit_goal(*old)
    fleet.pause()
    fleet.toggle_manual('robot1')
    if next_action == 'cancel':
        fleet.cancel_robot('robot1')
    elif next_action == 'new_goal':
        fleet.submit_goal(*new)
    else:
        node.heartbeat_fresh['robot1'] = False
    nav.states['robot1'] = 'CANCELED'
    fleet.tick()
    fleet.tick()
    if next_action == 'new_goal':
        assert fleet.manual_robot is None
        assert nav.sent == [old, new]
        assert node.drive_modes['robot1'] == 'NAV2'
    else:
        assert nav.sent == [old]
        assert node.drive_modes['robot1'] in {'STOP', 'MANUAL'}
        if next_action == 'cancel':
            assert fleet.manual_robot is None and not node.manual_routing['robot1']
            assert 'robot1' not in fleet.requests
        else:
            assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD
