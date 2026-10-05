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


def test_exit_requires_radius_and_genuine_no_boundary_observation():
    condition = LaneExitCondition()
    status = telemetry()
    assert condition.evaluate(0., (.07, 0., 180.), (0., 0., 0.), status, True)
    assert not condition.evaluate(0., (.070001, 0., 0.), (0., 0., 0.), status, True)
    status['exit_status']['boundaries']['left_visible'] = True
    assert not condition.evaluate(0., (0., 0., 0.), (0., 0., 0.), status, True)


@pytest.mark.parametrize('field,value', [('fresh', False), ('active', False), ('mode', 'MANUAL'),
                                       ('permission_enabled', False), ('received_age_s', 1.1)])
def test_exit_rejects_stale_paused_manual_or_inactive_status(field, value):
    status = telemetry()
    status[field] = value
    assert not LaneExitCondition().evaluate(5., (0., 0., 0.), (0., 0., 0.), status, True)


@pytest.mark.parametrize('field,value', [('boundaries', None), ('observation_age_s', None),
        ('observation_age_s', math.nan), ('sensors_ok', False), ('gate_run', False),
        ('obstacle', True), ('version', 0), ('control_reason', 'crosswalk'),
        ('control_reason', 'clear_hold'), ('boundaries', {'left_visible': None, 'right_visible': False})])
def test_exit_rejects_invalid_frames_and_safety_holds(field, value):
    status = telemetry()
    status['exit_status'][field] = value
    assert not LaneExitCondition().evaluate(5., (0., 0., 0.), (0., 0., 0.), status, True)


def test_stationary_requires_three_seconds_inside_and_resets_on_hold_motion_or_exit():
    c, status = LaneExitCondition(), telemetry()
    status['exit_status'].update(boundaries=dict(left_visible=True, right_visible=True), stationary_s=10.)
    def evaluate(now, allowed=True, x=0.):
        return c.evaluate(now, (x, 0., 0.), (0., 0., 0.), status, allowed)
    assert not evaluate(0.)
    assert not evaluate(2.99)
    assert evaluate(3.)
    assert not evaluate(4., False)
    assert not evaluate(5.)
    assert not evaluate(7.99)
    assert evaluate(8.)
    assert not evaluate(9., x=.08)
    assert not evaluate(10.)
    status['exit_status']['stationary_s'] = 0.
    assert not evaluate(13.)
    status['exit_status']['stationary_s'] = 2.
    assert not evaluate(15.)
    assert not evaluate(18.)
    status['exit_status']['stationary_s'] = 3.
    assert evaluate(18.1)


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
    f, demo, node, nav, lane, _ = scene
    assert demo.stage == 'A_LANE'
    assert [goal[0] for goal in lane.sent] == ['robot1']
    assert not nav.sent
    assert node.modes['robot2'][0] == FleetPermit.MODE_HOLD
    lane.states['robot1'] = 'FOLLOWING'
    node.positions['robot1'] = (1., 0.)
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


def test_nav_failure_and_stale_localization_stop_demo(scene):
    f, demo, node, nav, lane, _ = scene
    finish_lane(scene, 'A')
    nav.states['robot1'] = 'ABORTED'
    f.tick()
    assert demo.stage == 'FAILED'
    assert not f.enabled


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


def test_broken_amcl_and_cleanup_timeout_cancel_without_advancing(scene):
    f, demo, node, nav, lane, clock = scene
    lane.states['robot1'] = 'SUCCEEDED'
    f.tick()
    clock[0] += 5.1
    f.tick()
    assert demo.stage == 'FAILED'
    assert not nav.sent


def test_stale_waiting_robot_pose_does_not_start_either_nav_route(scene):
    f, demo, node, nav, lane, clock = scene
    node.pose_fresh['robot2'] = False
    f.tick()
    assert demo.stage == 'FAILED'
    assert not nav.sent and not f.enabled


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
