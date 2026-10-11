"""Rebased demo retries failed cancellation RPCs while retaining motion HOLD."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from pinky_interfaces.msg import FleetPermit
from test_demo_mission import finish_lane
from test_demo_mission import scene as scene
from test_hold_recovery import nav_client


@pytest.mark.parametrize('failure', ['rejected', 'future_exception', 'sync_exception', 'accepted'])
def test_demo_cancel_rpc_failure_retries_without_releasing_old_goal(scene, failure):
    fleet, demo, node, navigation, lane, clock = scene
    finish_lane(scene, 'A')
    assert demo.stage == 'NAV_ROUTES'
    client = nav_client('ACTIVE')
    handle = Mock()
    callbacks = []
    pending_future = SimpleNamespace(add_done_callback=callbacks.append)
    handle.cancel_goal_async.return_value = pending_future
    if failure == 'sync_exception':
        handle.cancel_goal_async.side_effect = [RuntimeError('cancel send failed'), pending_future]
    client._goal_handle = handle
    original_state = navigation.state
    original_cancel = navigation.cancel_goal
    navigation.state = lambda name: client.state() if name == 'robot1' else original_state(name)
    navigation.cancel_goal = lambda name: (
        client.cancel_goal() if name == 'robot1' else original_cancel(name))
    navigation.has_active_goal = lambda name: client.has_active_goal() if name == 'robot1' else False
    sent_before = list(navigation.sent)

    node.heartbeat_fresh['robot1'] = False
    fleet.tick()
    clock[0] += 3.1
    fleet.tick()
    assert handle.cancel_goal_async.call_count == 1
    if failure != 'sync_exception':
        def failed_cancel_result():
            if failure == 'future_exception':
                raise RuntimeError('cancel response lost')
            return SimpleNamespace(goals_canceling=[object()] if failure == 'accepted' else [])
        callbacks[0](SimpleNamespace(result=failed_cancel_result))
    assert client.state() == 'CANCELLING'
    assert client.has_active_goal()

    node.heartbeat_fresh['robot1'] = True
    for _ in range(4):
        clock[0] += 3.1
        fleet.tick()
    assert handle.cancel_goal_async.call_count == (1 if failure == 'accepted' else 2)
    assert client.state() == 'CANCELLING'
    assert client.has_active_goal()
    assert demo.is_recovering('robot1')
    assert navigation.sent == sent_before
    assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD
    assert node.drive_modes['robot1'] == 'STOP'


def test_sync_cancel_rpc_failure_allows_explicit_retry_with_handle_preserved():
    client = nav_client('ACTIVE')
    handle = Mock()
    handle.cancel_goal_async.side_effect = [RuntimeError('send failed'), Mock()]
    client._goal_handle = handle
    assert client.cancel_goal()
    assert client.state() == 'CANCELLING'
    assert client.has_active_goal()
    assert client.cancel_goal()
    assert handle.cancel_goal_async.call_count == 2
    assert client.state() == 'CANCELLING'
    assert client.has_active_goal()
