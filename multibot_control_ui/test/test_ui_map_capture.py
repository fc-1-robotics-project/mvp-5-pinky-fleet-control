"""Map pose input must stay separate from commands until explicit apply."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from multibot_control_ui.control_ui import MultiBotControlUI
from multibot_control_ui.lane_routes import POSES, load_routes


class Value:
    def __init__(self, value=''):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


@pytest.fixture
def ui(tmp_path):
    app = MultiBotControlUI.__new__(MultiBotControlUI)
    app.node = Mock()
    app.node.latest_map = SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        info=SimpleNamespace(width=100, height=100, resolution=0.1,
                             origin='rotated-map'),
        data=[0] * 10000,
    )
    app.coordinator = Mock(enabled=False)
    app.coordinator.submit_goal.return_value = (True, 'goal sent')
    app.canvas = Mock()
    app.capture_cancel_button = Mock()
    app.map_geometry = (1.0, 2.0, math.pi / 2)
    app.map_display = (20.0, 10.0, 2.0)
    app.map_photo = None
    app.map_capture = None
    app.goal_drag_start = app.goal_drag_target = None
    app.zone_drag_start = None
    app.zone_edit_mode = False
    app.selected_robot = Value()
    app.lane_direction = Value('A_to_B')
    app.field_ready = Value(True)
    app.goal_status = Value()
    app.routing_status = Value()
    app.route_status = Value()
    app.initial_pose_status = Value()
    app.route_values = {field: [Value() for _ in range(3)] for field in POSES}
    app.initial_pose_values = {
        name: {key: Value('0') for key in ('x', 'y', 'yaw')}
        for name in ('robot1', 'robot2')
    }
    app.routes = {}
    app.route_path = tmp_path / 'routes.json'
    app.goal_markers = {}
    app._draw_robot_markers = Mock()
    return app


def event(x=120, y=110):
    return SimpleNamespace(x=x, y=y)


def drag(ui, start=None, end=None):
    ui._on_goal_press(start or event())
    ui._on_goal_drag(end or event(150, 110))
    ui._on_goal_release(end or event(150, 110))


def assert_no_command(ui):
    ui.coordinator.submit_goal.assert_not_called()
    ui.coordinator.submit_lane_entry_goal.assert_not_called()
    ui.node.publish_initial_pose.assert_not_called()


def test_initial_pose_targets_its_row_and_waits_for_apply(ui):
    ui.selected_robot.set('robot1')
    ui._capture_initial_pose('robot2')
    drag(ui)
    assert [float(v.get()) for v in ui.initial_pose_values['robot2'].values()] == [-4, 7, 90]
    assert [v.get() for v in ui.initial_pose_values['robot1'].values()] == ['0'] * 3
    assert_no_command(ui)
    assert ui.map_capture == ('initial', 'robot2')
    ui._set_initial_pose('robot2')
    ui.node.publish_initial_pose.assert_called_once_with('robot2', -4, 7, 90)
    ui.coordinator.submit_goal.assert_not_called()
    assert ui.map_capture is None


@pytest.mark.parametrize('field', POSES)
def test_route_capture_without_selected_robot_and_individual_apply(ui, field):
    ui._capture_route_pose(field)
    drag(ui)
    assert [float(v.get()) for v in ui.route_values[field]] == [-4, 7, 90]
    assert not ui.route_path.exists()
    assert_no_command(ui)
    # A second drag adjusts this field, even if a robot has since been selected.
    ui.selected_robot.set('robot1')
    drag(ui, end=event(120, 80))
    assert [float(v.get()) for v in ui.route_values[field]] == [-4, 7, -180]
    assert_no_command(ui)
    ui._apply_route_pose(field)
    assert load_routes(ui.route_path)['A_to_B'] == {
        field: (-4, 7, -180), 'map_key': ui._map_key()}
    assert ui.map_capture is None
    # Individually saved coordinates do not bypass the complete-route gate.
    ui._start_continuous()
    assert_no_command(ui)


@pytest.mark.parametrize('cancel', ['escape', 'resize', 'target', 'direction', 'zone'])
def test_interrupting_a_drag_never_applies_or_sends_it(ui, cancel):
    ui.selected_robot.set('robot1')
    ui._capture_initial_pose('robot2')
    ui._on_goal_press(event())
    if cancel == 'escape':
        ui._cancel_map_capture()
    elif cancel == 'resize':
        ui._on_canvas_resize(None)
    elif cancel == 'target':
        ui._capture_route_pose('exit')
    elif cancel == 'direction':
        ui.lane_direction.set('B_to_A')
        ui._load_route_fields()
    else:
        ui._begin_zone_edit()
    ui._on_goal_release(event(150, 110))
    assert_no_command(ui)
    assert [v.get() for v in ui.initial_pose_values['robot2'].values()] == ['0'] * 3
    assert all(v.get() == '' for row in ui.route_values.values() for v in row)


@pytest.mark.parametrize('start,end', [((0, 0), (100, 100)), ((120, 110), (123, 112))])
def test_outside_map_or_short_drag_does_not_fill_fields(ui, start, end):
    ui._capture_initial_pose('robot1')
    drag(ui, event(*start), event(*end))
    assert_no_command(ui)
    assert [v.get() for v in ui.initial_pose_values['robot1'].values()] == ['0'] * 3


def test_regular_nav_goal_keeps_robot_selected_at_press(ui):
    ui.selected_robot.set('robot1')
    ui._on_goal_press(event())
    ui.selected_robot.set('robot2')
    ui._on_goal_drag(event(150, 110))
    ui._on_goal_release(event(150, 110))
    ui.coordinator.submit_goal.assert_called_once_with('robot1', -4, 7, 90)
    ui.node.publish_initial_pose.assert_not_called()
    assert ui.goal_markers == {'robot1': (-4, 7, 90)}


def test_capture_cannot_begin_without_map(ui):
    ui.node.latest_map = None
    ui._capture_route_pose('entry')
    drag(ui)
    assert ui.map_capture is None
    assert_no_command(ui)


def test_apply_keeps_other_coordinates_and_rejects_changed_map(ui):
    for field in POSES:
        ui._capture_route_pose(field)
        drag(ui)
        ui._apply_route_pose(field)
    saved = load_routes(ui.route_path)
    assert set(saved['A_to_B']) == {*POSES, 'map_key'}
    ui.node.latest_map.info.origin = 'different-map'
    ui.route_values['entry'][0].set('99')
    ui._apply_route_pose('entry')
    assert load_routes(ui.route_path) == saved
    assert '지도가 변경' in ui.route_status.get()


def test_partial_apply_does_not_save_other_unconfirmed_edits(ui):
    ui.routes = {'B_to_A': {'entry': (1, 2, 3), 'map_key': ui._map_key()}}
    for field in ('entry', 'exit'):
        ui._capture_route_pose(field)
        drag(ui)
    ui._apply_route_pose('entry')
    saved = load_routes(ui.route_path)
    assert saved['B_to_A']['entry'] == (1, 2, 3)
    assert 'entry' in saved['A_to_B']
    assert 'exit' not in saved['A_to_B']
    assert_no_command(ui)
