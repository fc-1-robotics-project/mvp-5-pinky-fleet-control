"""Offline sequencing tests using the real fleet policy and fake ROS clients."""
from copy import deepcopy
import math

import pytest

from multibot_control_ui.demo_mission import LaneExitCondition, TwoRobotDemo, validate_plan, save_plan
from multibot_control_ui.fleet_coordinator import FleetCoordinator
from pinky_interfaces.msg import FleetPermit
from test_fleet_coordinator import FakeNode, FakeNavigation, FakeLaneNavigation


def telemetry():
    return dict(fresh=True, received_age_s=0., active=True, mode='LANE', permission_enabled=True,
                cleanup_ok=False, ready=True,
                exit_status=dict(version=1, boundaries=dict(left_visible=False, right_visible=False),
                                 observation_age_s=.1, stationary_s=0., sensors_ok=True,
                                 obstacle=False, gate_run=True, control_reason='follow'))


def test_exit_confirms_twenty_cm_arrival_while_lines_visible_and_robot_moving():
    c, status = LaneExitCondition(), telemetry()
    status['exit_status'].update(boundaries=dict(left_visible=True, right_visible=True), stationary_s=None)
    assert not c.evaluate(0., (.2, 0., 180.), (0., 0., 0.), status, True)
    assert not c.evaluate(.49, (.18, 0., 180.), (0., 0., 0.), status, True)
    assert c.evaluate(.5, (.16, 0., 180.), (0., 0., 0.), status, True)
    assert not c.evaluate(.6, (.200001, 0., 0.), (0., 0., 0.), status, True)
    assert not c.evaluate(.7, (.19, 0., 0.), (0., 0., 0.), status, True)
    assert c.evaluate(1.21, (.19, 0., 0.), (0., 0., 0.), status, True)


@pytest.mark.parametrize('field,value', [('fresh', False), ('active', False), ('mode', 'MANUAL'),
                                       ('permission_enabled', False), ('received_age_s', 1.51),
                                       ('received_age_s', math.nan)])
def test_exit_rejects_stale_paused_manual_or_inactive_status(field, value):
    c, status = LaneExitCondition(), telemetry()
    assert not c.evaluate(0., (0., 0., 0.), (0., 0., 0.), status, True)
    status[field] = value
    assert not c.evaluate(5., (0., 0., 0.), (0., 0., 0.), status, True)
    assert c.since is None


@pytest.mark.parametrize('field,value', [('gate_run', False), ('obstacle', True),
        ('obstacle', None), ('version', 0), ('control_reason', 'clear_hold'),
        ('control_reason', 'emergency')])
def test_exit_resets_on_safety_holds(field, value):
    c, status = LaneExitCondition(), telemetry()
    assert not c.evaluate(0., (0., 0., 0.), (0., 0., 0.), status, True)
    status['exit_status'][field] = value
    assert not c.evaluate(5., (0., 0., 0.), (0., 0., 0.), status, True)
    assert c.since is None


def test_camera_delay_waits_for_sensor_recovery_without_erasing_arrival():
    c, status = LaneExitCondition(), telemetry()
    status['exit_status'].update(boundaries=dict(left_visible=True, right_visible=True), stationary_s=None)
    def evaluate(now):
        return c.evaluate(now, (.015, 0., 0.), (0., 0., 0.), status, True)
    assert not evaluate(0.)
    status['exit_status'].update(observation_age_s=1.3, sensors_ok=False)
    assert not evaluate(.4)
    assert not evaluate(.6)
    assert c.since == 0.
    status['exit_status'].update(observation_age_s=.8, sensors_ok=True)
    assert evaluate(.7)


def test_arrival_does_not_treat_visibility_or_stationarity_as_completion_evidence():
    c, status = LaneExitCondition(), telemetry()
    status['exit_status'].update(boundaries=None, observation_age_s=None, stationary_s=None)
    assert not c.evaluate(0., (.15, 0., 0.), (0., 0., 0.), status, True)
    assert c.evaluate(.5, (.15, 0., 0.), (0., 0., 0.), status, True)
    assert not c.evaluate(.6, None, (0., 0., 0.), status, True)
    assert not c.evaluate(1., (0., 0., 0.), (0., 0., 0.), status, True)
    assert not c.evaluate(2., (0., 0., 0.), (0., 0., 0.), status, False)
    assert not c.evaluate(3., (0., 0., 0.), (0., 0., 0.), status, True)
    assert c.evaluate(3.5, (0., 0., 0.), (0., 0., 0.), status, True)


def plan():
    return dict(version=1, map_key='map', a='robot1', b='robot2', a_route='A_to_B', b_route='B_to_A',
                a_exit=(1., 0., 0.), a_final=(4., 0., 0.), b_entry=(-1., 0., 180.), b_exit=(-2., 0., 180.),
                waypoints={'A': [(2., 0., 0.)], 'B': [(0., 0., 180.)]}, closed={'A': True, 'B': True})


class Lane(FakeLaneNavigation):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.statuses = {}
        for name in ('robot1', 'robot2'):
            self.statuses[name] = telemetry()
            self.statuses[name].update(active=False, mode='STOP', permission_enabled=False, cleanup_ok=True)

    def telemetry(self, name):
        return self.statuses[name]

    def current_localization(self, name):
        return (*self.node.positions[name], 0.) if self.node.pose_fresh[name] else None

    def localization_status(self, name, **limits):
        return dict(pose=self.current_localization(name), reason='test localization unavailable')

    def send_goal(self, name, mission_id, route_id, **kwargs):
        self.statuses[name].update(mission_id=mission_id, active=True, mode='LANE',
                                   permission_enabled=True, cleanup_ok=False)
        return super().send_goal(name, mission_id, route_id, **kwargs)


@pytest.fixture
def scene():
    node, nav = FakeNode(), FakeNavigation()
    node.finished = []
    node.publish_lane_finish = node.finished.append
    node.arrival_is_close = lambda name, goal: math.dist(node.positions[name], goal[:2]) < .15
    lane = Lane(node)
    f = FleetCoordinator(node, nav, [], lane)
    clock = [100.]
    demo = TwoRobotDemo(f, clock=lambda: clock[0])
    demo.start(plan(), 'map')
    return f, demo, node, nav, lane, clock


def finish_lane(scene, role):
    f, demo, node, nav, lane, _ = scene
    name = demo.plan[role.lower()]
    lane.states[name] = 'SUCCEEDED'
    lane.statuses[name].update(active=False, mode='STOP', permission_enabled=False, cleanup_ok=True)
    f.tick()


def finish_nav(scene, role):
    f, demo, node, nav, lane, _ = scene
    name = demo.plan[role.lower()]
    node.positions[name] = demo.tasks[role]['goal'][:2]
    nav.states[name] = 'SUCCEEDED'
    f.tick()


def test_full_demo_A_first_both_routes_then_final_and_B_lane(scene):
    f, demo, node, nav, lane, clock = scene
    assert demo.stage == 'A_LANE'
    assert [goal[0] for goal in lane.sent] == ['robot1']
    assert not nav.sent
    assert node.modes['robot2'][0] == FleetPermit.MODE_HOLD
    lane.states['robot1'] = 'FOLLOWING'
    node.positions['robot1'] = (1., 0.)
    f.tick()
    assert not node.finished
    clock[0] += .5
    f.tick()
    assert node.finished == ['robot1']
    assert not nav.sent  # Bool delivery alone is not a completed/released action.
    f.tick()
    assert node.finished == ['robot1']
    finish_lane(scene, 'A')
    assert demo.stage == 'NAV_ROUTES'
    assert len(nav.sent) == 2
    assert node.initial_poses == []
    finish_nav(scene, 'A')
    assert demo.stage == 'NAV_ROUTES'
    assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD
    finish_nav(scene, 'B')
    assert demo.stage == 'FINAL'
    finish_nav(scene, 'B')
    assert demo.stage == 'B_LANE'
    assert len(lane.sent) == 2
    finish_nav(scene, 'A')
    assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD
    node.positions['robot2'] = (-2., 0.)
    lane.states['robot2'] = 'FOLLOWING'
    f.tick()
    clock[0] += .5
    f.tick()
    assert node.finished == ['robot1', 'robot2']
    finish_lane(scene, 'B')
    assert demo.stage == 'COMPLETE'
    assert not f.enabled
    assert all(node.modes[n][0] == FleetPermit.MODE_HOLD for n in ('robot1', 'robot2'))
    assert node.initial_poses == []


def test_cleanup_and_current_AMCL_required_before_nav(scene):
    f, demo, node, nav, lane, _ = scene
    lane.states['robot1'] = 'SUCCEEDED'
    lane.statuses['robot1']['cleanup_ok'] = False
    f.tick()
    assert not nav.sent
    assert demo.stage == 'A_LANE'
    finish_lane(scene, 'A')
    assert len(nav.sent) == 2


def test_empty_open_queue_waits_and_pending_edit_does_not_replace_active_goal(scene):
    f, demo, node, nav, lane, _ = scene
    demo.replace_pending('A', [(3., 1., 90.)], False)
    finish_lane(scene, 'A')
    active = demo.tasks['A']['request']
    demo.replace_pending('A', [(3., 2., 0.), (3., 3., 90.)], False)
    assert demo.tasks['A']['request'] is active
    assert active.goal == (3., 1., 90.)
    demo.replace_pending('A', [], False)
    finish_nav(scene, 'A')
    finish_nav(scene, 'B')
    assert demo.stage == 'NAV_ROUTES'
    assert 'A' not in demo.route_done
    assert node.modes['robot1'][0] == FleetPermit.MODE_HOLD
    demo.replace_pending('A', [], True)
    f.tick()
    assert demo.stage == 'FINAL'


def test_conflicting_individual_goal_and_manual_are_rejected_but_cancel_stops_both(scene):
    f, demo, node, nav, lane, _ = scene
    assert not f.submit_goal('robot1', 10., 1., 0.)[0]
    assert not f.submit_lane_test('robot2')[0]
    assert not f.toggle_manual('robot2')[0]
    f.cancel_robot('robot1')
    assert demo.stage == 'CANCELLED'
    assert not f.enabled and not f.requests
    assert set(lane.cancelled) == {'robot1', 'robot2'}
    assert f.submit_goal('robot1', 1., 2., 0.)[0]
    assert 'robot1' not in f.sequence_holds


def test_pause_and_emergency_never_finish_or_downgrade_estop(scene):
    f, demo, node, nav, lane, _ = scene
    lane.states['robot1'] = 'FOLLOWING'
    node.positions['robot1'] = (1., 0.)
    f.pause()
    f.tick()
    assert not node.finished
    f.emergency_stop()
    f.tick()
    assert demo.stage == 'FAILED' and f.emergency
    assert all(node.modes[n][0] == FleetPermit.MODE_ESTOP for n in ('robot1', 'robot2'))


def test_nav_failure_retains_stage_and_goal_until_rechecked(scene):
    f, demo, node, nav, lane, _ = scene
    finish_lane(scene, 'A')
    nav.states['robot1'] = 'ABORTED'
    f.tick()
    assert demo.active and demo.stage == 'NAV_ROUTES' and demo.recovering
    assert f.enabled
    assert all(value[0] == FleetPermit.MODE_HOLD for value in node.modes.values())


def test_map_and_robot_validation_and_atomic_save(tmp_path):
    p = plan()
    path = tmp_path / 'demo.json'
    save_plan(path, p)
    saved = path.read_text()
    p['a'] = p['b']
    with pytest.raises(ValueError):
        save_plan(path, p)
    assert path.read_text() == saved
    p = plan()
    p['b_exit'] = [float('inf'), 0., 0.]
    with pytest.raises(ValueError):
        validate_plan(p)


def test_bottleneck_hold_retains_waypoint_and_original_observed_owner(scene):
    from multibot_control_ui.fleet_coordinator import ZoneRuntime
    from multibot_control_ui.zone_config import rectangular_zone
    f, demo, node, nav, lane, _ = scene
    finish_lane(scene, 'A')
    zone = rectangular_zone((0., 0.), (1., .5))
    f.zones[zone.zone_id] = ZoneRuntime(zone)
    node.positions.update(robot1=(.5, .25), robot2=(1.5, .25))
    f.tick()
    assert f.zones[zone.zone_id].owner == 'robot1'
    node.positions['robot2'] = (1.095, .25)
    request = demo.tasks['B']['request']
    sent = list(nav.sent)
    f.tick()
    assert request.phase == 'BOUNDARY_HOLD'
    assert node.modes['robot2'][0] == FleetPermit.MODE_HOLD
    assert demo.tasks['B']['request'] is request
    assert nav.sent == sent
    node.positions['robot1'] = (1.2, .25)
    f.tick()
    assert f.zones[zone.zone_id].owner is None
    assert node.modes['robot2'][0] == FleetPermit.MODE_RUN
    assert demo.tasks['B']['request'] is request


def test_cleanup_timeout_waits_for_confirmation_without_advancing(scene):
    f, demo, node, nav, lane, clock = scene
    lane.states['robot1'] = 'SUCCEEDED'
    f.tick()
    clock[0] += 5.1
    f.tick()
    assert demo.active and demo.stage == 'A_LANE' and demo.recovering
    assert not nav.sent
    lane.statuses['robot1'].update(active=False, mode='STOP', permission_enabled=False, cleanup_ok=True)
    recover(scene)
    assert demo.stage == 'NAV_ROUTES' and not demo.recovering
    assert len(nav.sent) == 2


def test_stale_waiting_robot_pose_does_not_start_either_nav_route(scene):
    f, demo, node, nav, lane, clock = scene
    node.pose_fresh['robot2'] = False
    f.tick()
    assert demo.active and demo.recovering and demo.stage == 'A_LANE'
    assert not nav.sent and f.enabled
    original = demo.tasks['A']['request']
    node.pose_fresh['robot2'] = True
    recover(scene)
    assert not demo.recovering
    assert demo.tasks['A']['request'] is original
    assert not nav.sent


def test_estop_terminates_demo_immediately_before_any_ui_tick(scene):
    fleet, demo, node, nav, lane, _ = scene
    finish_lane(scene, 'A')
    sent = list(nav.sent)
    fleet.emergency_stop()
    assert not demo.active and demo.stage == 'FAILED'
    assert not fleet.requests
    assert all(value[0] == FleetPermit.MODE_ESTOP for value in node.modes.values())
    # Clicking RUN before a periodic tick must not continue the old demo.
    fleet.start()
    fleet.tick()
    assert nav.sent == sent
    assert not demo.active
    assert all(value[0] == FleetPermit.MODE_HOLD for value in node.modes.values())


def recover(scene):
    fleet, _, _, _, _, clock = scene
    clock[0] += 3.1
    fleet.tick()
    clock[0] += 1.1
    fleet.tick()


def test_nav2_success_advances_without_a_second_arrival_check(scene):
    fleet, demo, node, nav, _, _ = scene
    def forbidden_check(*_):
        raise AssertionError('Nav2 success must be the arrival decision')
    node.arrival_is_close = forbidden_check
    finish_lane(scene, 'A')
    # Leave the PC's cached coordinates far from the goals deliberately.
    nav.states['robot1'] = 'SUCCEEDED'
    fleet.tick()
    nav.states['robot2'] = 'SUCCEEDED'
    fleet.tick()
    assert demo.stage == 'FINAL'
    nav.states['robot2'] = 'SUCCEEDED'
    fleet.tick()
    assert demo.stage == 'B_LANE'
    nav.states['robot1'] = 'SUCCEEDED'
    fleet.tick()
    assert 'A' not in demo.tasks


@pytest.mark.parametrize('state', ['ABORTED', 'REJECTED', 'CANCELED', 'ERROR', 'UNAVAILABLE'])
def test_nav_errors_retry_only_failed_goal_and_preserve_queues(scene, state):
    fleet, demo, node, nav, _, _ = scene
    demo.replace_pending('A', [(2., 0., 0.), (3., 1., 90.)], True)
    finish_lane(scene, 'A')
    original = demo.tasks['A']['request']
    other = demo.tasks['B']['request']
    pending = deepcopy(demo.pending)
    sent = list(nav.sent)
    nav.states['robot1'] = state
    fleet.tick()
    assert demo.active and demo.recovering
    assert demo.tasks['A']['request'] is original
    assert fleet.requests['robot1'] is original
    assert demo.pending == pending
    assert all(value[0] == FleetPermit.MODE_HOLD for value in node.modes.values())
    for _ in range(4):
        fleet.tick()
    assert nav.sent == sent
    recover(scene)
    assert demo.active and not demo.recovering and demo.stage == 'NAV_ROUTES'
    assert nav.sent == sent + [('robot1', *original.goal)]
    assert demo.tasks['B']['request'] is other
    assert demo.pending == pending
    finish_nav(scene, 'A')
    assert nav.sent[-1] == ('robot1', 3., 1., 90.)


@pytest.mark.parametrize('fault', ['heartbeat', 'localization', 'status'])
def test_transient_data_fault_holds_then_resumes_same_lane_action(scene, fault):
    fleet, demo, node, nav, lane, _ = scene
    request = demo.tasks['A']['request']
    sent = list(lane.sent)
    if fault == 'heartbeat':
        node.heartbeat_fresh['robot2'] = False
    elif fault == 'localization':
        node.pose_fresh['robot2'] = False
    else:
        lane.statuses['robot2']['fresh'] = False
    fleet.tick()
    assert demo.active and demo.recovering and demo.stage == 'A_LANE'
    assert not nav.sent
    node.heartbeat_fresh['robot2'] = node.pose_fresh['robot2'] = True
    lane.statuses['robot2']['fresh'] = True
    recover(scene)
    assert demo.tasks['A']['request'] is request
    assert lane.sent == sent and not nav.sent
    assert not demo.recovering


def test_recovery_health_must_be_continuous_and_operator_pause_prevents_retry(scene):
    fleet, demo, node, nav, _, clock = scene
    finish_lane(scene, 'A')
    nav.states['robot1'] = 'ABORTED'
    fleet.tick()
    sent = list(nav.sent)
    clock[0] += 3.1
    fleet.tick()
    node.heartbeat_fresh['robot2'] = False
    clock[0] += .8
    fleet.tick()
    node.heartbeat_fresh['robot2'] = True
    fleet.tick()
    clock[0] += .8
    fleet.tick()
    assert nav.sent == sent and demo.recovering
    fleet.pause()
    clock[0] += 20.
    fleet.tick()
    assert nav.sent == sent and demo.recovering
    assert demo.active
    fleet.start()
    fleet.tick()
    assert not demo.recovering and len(nav.sent) == len(sent) + 1


def test_lane_failure_waits_for_cleanup_and_readiness_before_retry(scene):
    fleet, demo, node, nav, lane, clock = scene
    request = demo.tasks['A']['request']
    lane.states['robot1'] = 'ABORTED'
    lane.statuses['robot1'].update(active=False, mode='STOP', permission_enabled=False,
                                  cleanup_ok=False, ready=False)
    fleet.tick()
    recover(scene)
    assert demo.active and demo.recovering and len(lane.sent) == 1
    assert not nav.sent and demo.tasks['A']['request'] is request
    lane.statuses['robot1'].update(cleanup_ok=True, ready=True)
    clock[0] += 3.1
    fleet.tick()
    assert not demo.recovering and len(lane.sent) == 2
    assert demo.tasks['A']['request'].mission_id != request.mission_id
    assert demo.stage == 'A_LANE' and not nav.sent


def test_finish_timeout_repeats_bool_and_accepts_matched_robot_success(scene):
    fleet, demo, node, nav, lane, clock = scene
    request = demo.tasks['A']['request']
    node.positions['robot1'] = (1., 0.)
    lane.states['robot1'] = 'FOLLOWING'
    fleet.tick()
    clock[0] += .5
    fleet.tick()
    assert node.finished == ['robot1']
    clock[0] += 5.1
    fleet.tick()
    assert demo.active and demo.recovering and not nav.sent
    recover(scene)
    assert node.finished == ['robot1', 'robot1']
    assert demo.recovering and not nav.sent and len(lane.sent) == 1
    lane.states['robot1'] = 'ERROR'  # Result lost; robot reports the same mission succeeded.
    lane.statuses['robot1'].update(state='COMPLETE', mission_id=request.mission_id,
                                  active=False, mode='STOP', permission_enabled=False, cleanup_ok=True)
    clock[0] += 3.1
    fleet.tick()
    assert not demo.recovering and demo.stage == 'NAV_ROUTES'
    assert len(nav.sent) == 2 and len(lane.sent) == 1


def test_wrong_mission_success_cannot_release_lane_finish_recovery(scene):
    fleet, demo, node, nav, lane, clock = scene
    node.positions['robot1'] = (1., 0.)
    lane.states['robot1'] = 'FOLLOWING'
    fleet.tick()
    clock[0] += .5
    fleet.tick()
    clock[0] += 5.1
    fleet.tick()
    lane.states['robot1'] = 'ERROR'
    lane.statuses['robot1'].update(state='COMPLETE', mission_id='different-mission',
                                  active=False, mode='STOP', permission_enabled=False, cleanup_ok=True)
    recover(scene)
    assert demo.active and demo.recovering and not nav.sent


def test_completed_peer_is_not_resent_when_other_robot_recovers(scene):
    fleet, demo, _, nav, _, _ = scene
    finish_lane(scene, 'A')
    nav.states['robot1'] = 'ABORTED'
    fleet.tick()
    nav.states['robot2'] = 'SUCCEEDED'
    sent = list(nav.sent)
    recover(scene)
    assert demo.done['B'] == [plan()['waypoints']['B'][0]]
    assert nav.sent == sent + [('robot1', *plan()['waypoints']['A'][0])]
    assert 'B' in demo.route_done


def test_segment_timeout_waits_for_cancellation_before_retry(scene):
    fleet, demo, node, nav, _, clock = scene
    finish_lane(scene, 'A')
    sent = list(nav.sent)
    clock[0] += 901.
    demo.tasks['B']['started'] = clock[0]  # Isolate the expired A segment.
    fleet.tick()
    assert demo.active and demo.recovering
    def asynchronous_cancel(name):
        nav.cancelled.append(name)
        nav.states[name] = 'CANCELLING'
        return True
    nav.cancel_goal = asynchronous_cancel
    recover(scene)
    assert demo.recovering and nav.sent == sent
    clock[0] += 3.1
    fleet.tick()
    assert nav.sent == sent
    nav.states['robot1'] = 'CANCELED'
    clock[0] += 3.1
    fleet.tick()
    assert nav.sent[-1] == sent[0]
    assert not demo.recovering


def test_stale_bottleneck_owner_keeps_lease_and_recovers_without_estop(scene):
    from multibot_control_ui.fleet_coordinator import ZoneRuntime
    from multibot_control_ui.zone_config import rectangular_zone
    fleet, demo, node, nav, _, _ = scene
    finish_lane(scene, 'A')
    zone = rectangular_zone((0., 0.), (1., .5))
    fleet.zones[zone.zone_id] = ZoneRuntime(zone)
    node.positions.update(robot1=(.5, .25), robot2=(1.5, .25))
    fleet.tick()
    runtime = fleet.zones[zone.zone_id]
    owner, lease = runtime.owner, runtime.lease_id
    node.pose_fresh['robot1'] = False
    fleet.tick()
    assert demo.active and demo.recovering and not fleet.emergency
    assert runtime.state == 'LOCKED' and (runtime.owner, runtime.lease_id) == (owner, lease)
    assert all(value[0] == FleetPermit.MODE_HOLD for value in node.modes.values())
    node.pose_fresh['robot1'] = True
    recover(scene)
    assert not demo.recovering and demo.active
    assert (runtime.owner, runtime.lease_id) == (owner, lease)


@pytest.mark.parametrize('explicit_stop', ['cancel', 'estop'])
def test_explicit_stop_during_recovery_cannot_restart_automatically(scene, explicit_stop):
    fleet, demo, node, nav, lane, clock = scene
    finish_lane(scene, 'A')
    nav.states['robot1'] = 'ABORTED'
    fleet.tick()
    sent = list(nav.sent)
    if explicit_stop == 'cancel':
        demo.cancel()
    else:
        fleet.emergency_stop()
    clock[0] += 20.
    fleet.tick()
    assert not demo.active and not demo.recovering and not fleet.requests
    assert nav.sent == sent
    mode = FleetPermit.MODE_ESTOP if explicit_stop == 'estop' else FleetPermit.MODE_HOLD
    assert all(value[0] == mode for value in node.modes.values())


def test_failed_final_goal_submission_retains_both_destinations(scene):
    fleet, demo, node, nav, lane, clock = scene
    finish_lane(scene, 'A')
    finish_nav(scene, 'A')
    submit = fleet.submit_goal
    fleet.submit_goal = lambda *args, **kwargs: (False, 'temporary server fault')
    finish_nav(scene, 'B')
    assert demo.active and demo.recovering and demo.stage == 'FINAL'
    assert demo.tasks['A']['goal'] == plan()['a_final']
    assert demo.tasks['B']['goal'] == plan()['b_entry']
    assert demo.tasks['A']['request'] is demo.tasks['B']['request'] is None
    assert len(lane.sent) == 1  # Never skip B's entry and start its lane action.
    fleet.submit_goal = submit
    recover(scene)
    assert demo.active and not demo.recovering and demo.stage == 'FINAL'
    assert nav.sent[-2:] == [('robot1', *plan()['a_final']), ('robot2', *plan()['b_entry'])]
    assert all(task['request'] is not None for task in demo.tasks.values())
    finish_nav(scene, 'B')
    assert demo.stage == 'B_LANE' and len(lane.sent) == 2


def reset_start_scene(scene):
    _, demo, _, _, lane, _ = scene
    demo.cancel()
    for status in lane.statuses.values():
        status.update(active=False, mode='STOP', permission_enabled=False, cleanup_ok=True, ready=True)


def test_initial_pose_default_covariance_starts_and_does_not_immediately_recover(scene):
    from test_lane_client_lifecycle import pose_client
    fleet, demo, node, nav, lane, clock = scene
    reset_start_scene(scene)
    clients = {name: pose_client() for name in ('robot1', 'robot2')}
    lane.localization_status = lambda name, **limits: clients[name].localization_status(**limits)
    demo.start(plan(), 'map')
    assert demo.active and demo.stage == 'A_LANE' and not demo.recovering
    fleet.tick()
    assert not demo.recovering and len(lane.sent) == 2
    assert not nav.sent


def test_start_reports_actual_excessive_covariance_instead_of_generic_not_ready(scene):
    from test_lane_client_lifecycle import pose_client
    fleet, demo, node, nav, lane, clock = scene
    reset_start_scene(scene)
    c = pose_client(.31)
    lane.localization_status = lambda name, **limits: c.localization_status(**limits)
    sent = list(lane.sent)
    with pytest.raises(ValueError, match='robot1: 위치 분산 범위 초과'):
        demo.start(plan(), 'map')
    assert not demo.active and lane.sent == sent and not nav.sent


@pytest.mark.parametrize('fault,message', [
    ('heartbeat', 'heartbeat 수신'), ('status', '상태 메시지 수신'),
    ('active', '이전 차선 임무 종료'), ('mode', 'STOP 모드 확인'),
    ('permission', '로컬 주행 허가 OFF'), ('cleanup', 'STOP·허가 정리'),
    ('version', '코드 버전'), ('lane_ready', '차선 시작 점검')])
def test_start_reports_remaining_required_conditions(scene, fault, message):
    fleet, demo, node, nav, lane, clock = scene
    reset_start_scene(scene)
    status = lane.statuses['robot1']
    if fault == 'heartbeat':
        node.heartbeat_fresh['robot1'] = False
    elif fault == 'status':
        status['fresh'] = False
    elif fault == 'active':
        status['active'] = True
    elif fault == 'mode':
        status['mode'] = 'NAV2'
    elif fault == 'permission':
        status['permission_enabled'] = True
    elif fault == 'cleanup':
        status['cleanup_ok'] = False
    elif fault == 'version':
        status['exit_status']['version'] = 0
    else:
        status.update(ready=False, readiness_reason='camera_stale')
    with pytest.raises(ValueError, match=message):
        demo.start(plan(), 'map')
    assert not demo.active
