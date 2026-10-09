"""Regression cases for independent robot recovery and held Nav2 actions."""

import pytest

from multibot_control_ui.fleet_coordinator import ZoneRuntime
from multibot_control_ui.zone_config import rectangular_zone
from pinky_interfaces.msg import FleetPermit
from test_demo_mission import finish_lane, finish_nav, recover
from test_demo_mission import scene as scene


def reach_final(scene):
    finish_lane(scene, 'A')
    finish_nav(scene, 'A')
    finish_nav(scene, 'B')
    assert scene[1].stage == 'FINAL'


def asynchronous_cancels(nav):
    cancellations = []

    def cancel(name):
        cancellations.append(name)
        nav.cancelled.append(name)
        nav.states[name] = 'CANCELLING'
        return True

    nav.cancel_goal = cancel
    return cancellations


def test_failed_B_lane_does_not_hold_or_replay_A_final_goal(scene):
    fleet, demo, node, nav, lane, _ = scene
    reach_final(scene)
    finish_nav(scene, 'B')
    sent = list(nav.sent)
    cancelled = list(nav.cancelled)
    lane.states['robot2'] = 'ABORTED'
    lane.statuses['robot2'].update(
        active=False, mode='STOP', permission_enabled=False,
        cleanup_ok=False, ready=False)
    fleet.tick()
    recover(scene)

    assert demo.stage == 'B_LANE' and demo.is_recovering('robot2')
    assert not demo.is_recovering('robot1')
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN
    assert node.drive_modes['robot1'] == 'NAV2'
    assert nav.state('robot1') == 'ACTIVE' and nav.cancelled == cancelled
    finish_nav(scene, 'A')
    assert 'A' not in demo.tasks and demo.is_recovering('robot2')
    assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD

    lane.statuses['robot2'].update(cleanup_ok=True, ready=True)
    recover(scene)
    assert not demo.is_recovering('robot2')
    assert nav.sent == sent  # Completed A must not be replayed when B restarts.
    assert len(lane.sent) == 3


def test_one_robot_recovers_while_peer_health_keeps_failing(scene):
    fleet, demo, node, nav, lane, _ = scene
    finish_lane(scene, 'A')
    sent = list(nav.sent)
    peer_request = demo.tasks['B']['request']
    nav.states['robot1'] = 'ABORTED'
    lane.statuses['robot2']['fresh'] = False
    fleet.tick()
    assert demo.is_recovering('robot1') and demo.is_recovering('robot2')
    recover(scene)

    assert not demo.is_recovering('robot1') and demo.is_recovering('robot2')
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN
    assert node.modes['robot2'][0] == FleetPermit.MODE_HOLD
    assert nav.sent == sent + [sent[0]]
    assert demo.tasks['B']['request'] is peer_request


def test_failed_A_final_goal_does_not_delay_healthy_B_lane_entry(scene):
    fleet, demo, node, nav, lane, _ = scene
    reach_final(scene)
    nav.states['robot1'] = 'ABORTED'
    fleet.tick()
    sent = list(nav.sent)
    assert demo.is_recovering('robot1')
    finish_nav(scene, 'B')

    assert demo.stage == 'B_LANE'
    assert demo.is_recovering('robot1') and not demo.is_recovering('robot2')
    assert demo.tasks['B']['kind'] == 'LANE'
    assert len(lane.sent) == 2 and lane.sent[-1][0] == 'robot2'
    assert nav.sent == sent
    lane.states['robot2'] = 'FOLLOWING'
    fleet.tick()
    assert node.modes['robot2'][0] == FleetPermit.MODE_RUN


def test_peer_health_fault_does_not_stop_next_waypoint_dispatch(scene):
    fleet, demo, node, nav, lane, _ = scene
    goals = [(2., 0., 0.), (3., 1., 90.)]
    demo.replace_pending('A', goals, True)
    finish_lane(scene, 'A')
    peer_request = demo.tasks['B']['request']
    lane.statuses['robot2']['fresh'] = False
    fleet.tick()
    finish_nav(scene, 'A')

    assert demo.is_recovering('robot2') and not demo.is_recovering('robot1')
    assert demo.done['A'] == [goals[0]]
    assert demo.tasks['A']['goal'] == goals[1]
    assert nav.sent[-1] == ('robot1', *goals[1])
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN
    assert demo.tasks['B']['request'] is peer_request
    finish_nav(scene, 'A')
    assert demo.done['A'] == goals and 'A' in demo.route_done
    assert demo.stage == 'NAV_ROUTES' and demo.is_recovering('robot2')


def test_stale_zone_owner_preserves_lease_and_holds_peer_at_boundary(scene):
    fleet, demo, node, _, _, clock = scene
    finish_lane(scene, 'A')
    zone = rectangular_zone((0., 0.), (1., .5))
    fleet.zones[zone.zone_id] = ZoneRuntime(zone)
    node.positions.update(robot1=(1.5, .25), robot2=(.5, .25))
    fleet.tick()
    runtime = fleet.zones[zone.zone_id]
    lease = runtime.lease_id
    assert runtime.owner == 'robot2' and lease

    node.positions['robot1'] = (1.05, .25)
    node.pose_fresh['robot2'] = False
    fleet.tick()
    clock[0] += 10.
    fleet.tick()
    assert (runtime.owner, runtime.lease_id) == ('robot2', lease)
    assert runtime.state == 'LOCKED' and not fleet.emergency
    assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD
    assert demo.is_recovering('robot2')

    node.pose_fresh['robot2'] = True
    recover(scene)
    assert (runtime.owner, runtime.lease_id) == ('robot2', lease)
    assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD


def test_long_health_hold_cancels_only_affected_nav_once_and_waits_for_result(scene):
    fleet, demo, node, nav, _, clock = scene
    finish_lane(scene, 'A')
    sent = list(nav.sent)
    request = demo.tasks['A']['request']
    cancellations = asynchronous_cancels(nav)
    node.heartbeat_fresh['robot1'] = False
    fleet.tick()
    clock[0] += 2.9
    fleet.tick()
    assert not cancellations
    clock[0] += .2
    fleet.tick()
    assert cancellations == ['robot1']
    assert nav.state('robot1') == 'CANCELLING'
    assert nav.state('robot2') == 'ACTIVE'
    assert node.modes['robot2'][0] == FleetPermit.MODE_RUN

    node.heartbeat_fresh['robot1'] = True
    recover(scene)
    recover(scene)
    assert cancellations == ['robot1'] and nav.sent == sent
    assert demo.is_recovering('robot1')
    nav.states['robot1'] = 'CANCELED'
    clock[0] += 3.1
    fleet.tick()
    assert nav.sent == sent + [sent[0]]
    assert demo.tasks['A']['request'] is request
    assert not demo.is_recovering('robot1')
    assert node.modes['robot1'][0] == FleetPermit.MODE_RUN
    assert nav.state('robot2') == 'ACTIVE'


def test_late_nav_success_after_hold_cancellation_is_preserved(scene):
    fleet, demo, node, nav, _, clock = scene
    finish_lane(scene, 'A')
    sent = list(nav.sent)
    goal = demo.tasks['A']['goal']
    cancellations = asynchronous_cancels(nav)
    node.heartbeat_fresh['robot1'] = False
    fleet.tick()
    clock[0] += 3.1
    fleet.tick()
    assert cancellations == ['robot1'] and nav.state('robot1') == 'CANCELLING'

    nav.states['robot1'] = 'SUCCEEDED'
    node.heartbeat_fresh['robot1'] = True
    recover(scene)
    assert nav.sent == sent and cancellations == ['robot1']
    assert demo.done['A'] == [goal] and 'A' in demo.route_done
    assert 'A' not in demo.tasks and not demo.is_recovering('robot1')
    assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD
    assert node.modes['robot2'][0] == FleetPermit.MODE_RUN


def test_cancel_rpc_error_does_not_allow_new_goal_before_terminal_result(scene):
    fleet, demo, node, nav, _, clock = scene
    finish_lane(scene, 'A')
    sent = list(nav.sent)
    cancellations = []

    def failed_cancel(name):
        cancellations.append(name)
        nav.states[name] = 'ERROR'  # The RPC failed; the old goal may still run.
        return True

    nav.cancel_goal = failed_cancel
    node.heartbeat_fresh['robot1'] = False
    fleet.tick()
    clock[0] += 3.1
    fleet.tick()
    assert cancellations == ['robot1'] and nav.state('robot1') == 'ERROR'
    node.heartbeat_fresh['robot1'] = True
    recover(scene)
    assert demo.is_recovering('robot1') and nav.sent == sent
    assert set(cancellations) == {'robot1'}
    assert node.modes['robot2'][0] == FleetPermit.MODE_RUN

    nav.states['robot1'] = 'ABORTED'
    clock[0] += 3.1
    fleet.tick()
    assert not demo.is_recovering('robot1')
    assert nav.sent == sent + [sent[0]]


def test_result_transport_error_with_live_handle_is_cancelled_before_resend(scene):
    fleet, demo, node, nav, _, clock = scene
    finish_lane(scene, 'A')
    sent = list(nav.sent)
    active_handles = {'robot1': True, 'robot2': True}
    nav.has_active_goal = lambda name: active_handles[name]
    cancellations = asynchronous_cancels(nav)
    nav.states['robot1'] = 'ERROR'  # Result transport failed, handle still exists.
    fleet.tick()
    recover(scene)
    assert cancellations == ['robot1'] and nav.sent == sent
    assert demo.is_recovering('robot1')
    assert node.modes['robot2'][0] == FleetPermit.MODE_RUN

    active_handles['robot1'] = False
    nav.states['robot1'] = 'ABORTED'
    clock[0] += 3.1
    fleet.tick()
    assert nav.sent == sent + [sent[0]]
    assert not demo.is_recovering('robot1')


def test_pending_nav_acceptance_cannot_be_replaced_before_cancel_result(scene):
    fleet, demo, node, nav, _, clock = scene
    finish_lane(scene, 'A')
    sent = list(nav.sent)
    accepted = [False]
    nav.states['robot1'] = 'PENDING'

    def cancel(name):
        nav.cancelled.append(name)
        if not accepted[0]:
            return False
        nav.states[name] = 'CANCELLING'
        return True

    nav.cancel_goal = cancel
    node.heartbeat_fresh['robot1'] = False
    fleet.tick()
    clock[0] += 3.1
    fleet.tick()
    node.heartbeat_fresh['robot1'] = True
    recover(scene)
    assert demo.is_recovering('robot1') and nav.sent == sent

    accepted[0] = True
    nav.states['robot1'] = 'ACTIVE'
    clock[0] += 3.1
    fleet.tick()
    assert nav.states['robot1'] == 'CANCELLING' and nav.sent == sent
    nav.states['robot1'] = 'CANCELED'
    clock[0] += 3.1
    fleet.tick()
    assert nav.sent == sent + [sent[0]]
    assert not demo.is_recovering('robot1')


def test_result_arriving_during_active_handle_check_is_not_replayed(scene):
    fleet, demo, node, nav, _, _ = scene
    finish_lane(scene, 'A')
    sent = list(nav.sent)
    goal = demo.tasks['A']['goal']
    nav.states['robot1'] = 'ERROR'
    fleet.tick()

    def complete_during_check(name):
        if name == 'robot1':
            nav.states[name] = 'SUCCEEDED'
            return False
        return True

    nav.has_active_goal = complete_during_check
    recover(scene)
    assert nav.sent == sent
    assert demo.done['A'] == [goal] and 'A' in demo.route_done
    assert not demo.is_recovering('robot1')


@pytest.mark.parametrize('stop', ['pause', 'estop'])
def test_operator_stop_during_partial_recovery_never_restarts_automatically(scene, stop):
    fleet, demo, node, nav, _, clock = scene
    finish_lane(scene, 'A')
    nav.states['robot1'] = 'ABORTED'
    fleet.tick()
    assert demo.is_recovering('robot1') and not demo.is_recovering('robot2')
    sent = list(nav.sent)
    if stop == 'pause':
        fleet.pause()
    else:
        fleet.emergency_stop()
    clock[0] += 20.
    fleet.tick()

    assert nav.sent == sent
    mode = FleetPermit.MODE_HOLD if stop == 'pause' else FleetPermit.MODE_ESTOP
    assert all(value[0] == mode for value in node.modes.values())
    if stop == 'pause':
        assert demo.active and demo.is_recovering('robot1')
    else:
        assert not demo.active and not fleet.requests
        fleet.start()
        fleet.tick()
        assert nav.sent == sent and not demo.active
