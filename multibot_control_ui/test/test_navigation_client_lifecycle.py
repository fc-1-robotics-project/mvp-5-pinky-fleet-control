"""Cancellation callbacks must preserve terminal Nav2 action results."""

from concurrent.futures import Future
from threading import Lock
from types import SimpleNamespace

import pytest

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from multibot_control_ui.navigation_client import FleetNavigationClients, RobotNavigationClient


class Handle:
    def __init__(self):
        self.cancel_response = Future()
        self.cancel_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_response


def client():
    c = RobotNavigationClient.__new__(RobotNavigationClient)
    c.lock = Lock()
    c._goal_generation = 1
    c._goal_handle = Handle()
    c._state, c._status = 'ACTIVE', '주행 중'
    return c


def finish(c, status=GoalStatus.STATUS_SUCCEEDED):
    result = Future()
    result.set_result(SimpleNamespace(
        status=status, result=SimpleNamespace(error_code=NavigateToPose.Result.NONE)))
    c._result_callback(result, c._goal_generation)


class ResultAtUnlock:
    """Deliver a ROS result as soon as cancellation releases the client lock."""

    def __init__(self, callback):
        self.lock = Lock()
        self.callback = callback

    def __enter__(self):
        self.lock.acquire()
        return self

    def __exit__(self, *_):
        self.lock.release()
        callback, self.callback = self.callback, None
        if callback is not None:
            callback()


def test_terminal_result_between_handle_capture_and_cancel_rpc_is_preserved():
    c = client()
    handle = c._goal_handle
    handle.cancel_response.set_result(SimpleNamespace(goals_canceling=[]))
    c.lock = ResultAtUnlock(lambda: finish(c))
    assert c.cancel_goal()
    assert handle.cancel_calls == 1
    assert c.state() == 'SUCCEEDED' and c.status() == '목표 도착 완료'
    assert not c.has_active_goal()


def test_late_cancel_rpc_exception_cannot_overwrite_success():
    c = client()
    handle = c._goal_handle
    assert c.cancel_goal()
    finish(c)
    handle.cancel_response.set_exception(RuntimeError('late cancellation transport error'))
    assert c.state() == 'SUCCEEDED' and c.status() == '목표 도착 완료'
    assert not c.has_active_goal()


@pytest.mark.parametrize('response', ['accepted', 'rejected', 'error'])
def test_obsolete_cancel_callback_cannot_change_new_goal(response):
    c = client()
    old_handle = c._goal_handle
    assert c.cancel_goal()
    c._goal_generation += 1
    new_handle = c._goal_handle = Handle()
    c._state, c._status = 'ACTIVE', 'new goal running'
    if response == 'error':
        old_handle.cancel_response.set_exception(RuntimeError('old RPC failed'))
    else:
        old_handle.cancel_response.set_result(SimpleNamespace(
            goals_canceling=[object()] if response == 'accepted' else []))
    assert c.state() == 'ACTIVE' and c.status() == 'new goal running'
    assert c._goal_handle is new_handle and new_handle.cancel_calls == 0


def test_rejected_cancel_retains_handle_and_can_retry_until_terminal_result():
    c = client()
    handle = c._goal_handle
    assert c.cancel_goal()
    handle.cancel_response.set_result(SimpleNamespace(goals_canceling=[]))
    assert c.state() == 'ERROR' and c.has_active_goal()
    assert c._goal_handle is handle

    handle.cancel_response = Future()
    assert c.cancel_goal() and handle.cancel_calls == 2
    handle.cancel_response.set_result(SimpleNamespace(goals_canceling=[object()]))
    assert c.state() == 'CANCELLING' and c.has_active_goal()
    finish(c, GoalStatus.STATUS_CANCELED)
    assert c.state() == 'CANCELED' and not c.has_active_goal()


def test_fleet_reports_retained_error_handle_until_terminal_result():
    c = client()
    fleet = FleetNavigationClients.__new__(FleetNavigationClients)
    fleet.clients = {'robot1': c}
    c._state = 'ERROR'
    assert fleet.has_active_goal('robot1')
    finish(c)
    assert not fleet.has_active_goal('robot1')
