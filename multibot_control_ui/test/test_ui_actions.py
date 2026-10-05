"""UI commands must address the advertised robot and preserve mode ownership."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from multibot_control_ui.demo_panel import DemoPanel
from multibot_control_ui.demo_mission import POSES, validate_plan
from test_demo_mission import plan
from test_ui_map_capture import Value, ui  # noqa: F401 (shared fixture)


def test_manual_reselects_routing_target_after_autonomous_start(ui):
    ui.selected_robot.set('robot1')
    ui.node.clear_selection()
    def enter_manual(_robot):
        ui.coordinator.manual_robot = 'robot1'
        return True, 'manual'
    ui.coordinator.toggle_manual.side_effect = enter_manual
    ui._toggle_manual()
    ui.node.select_robot.assert_called_once_with('robot1')


def test_failed_manual_switch_does_not_enable_routing(ui):
    ui.selected_robot.set('robot1')
    ui.coordinator.toggle_manual.return_value = False, 'no heartbeat'
    ui._toggle_manual()
    ui.node.select_robot.assert_not_called()
    assert 'no heartbeat' in ui.goal_status.get()


def test_demo_owns_individual_pause_and_cancel_targets_both(ui):
    ui.selected_robot.set('robot2')
    ui.demo = SimpleNamespace(owns=lambda name: name in ('robot1', 'robot2'))
    ui._pause_selected_mission()
    ui.coordinator.pause_robot.assert_not_called()
    assert not ui._lane_start_available('robot2')
    ui._cancel_selected_goal()
    ui.coordinator.cancel_robot.assert_called_once_with('robot2')
    assert '두 로봇' in ui.goal_status.get()


@pytest.mark.parametrize('mode', ['mission', 'manual', 'demo'])
def test_initial_pose_cannot_jump_during_active_control(ui, mode):
    if mode == 'mission':
        ui.coordinator.requests['robot1'] = object()
    elif mode == 'manual':
        ui.coordinator.manual_robot = 'robot1'
    else:
        ui.demo = SimpleNamespace(owns=lambda name: name == 'robot1')
    ui._set_initial_pose('robot1')
    ui.node.publish_initial_pose.assert_not_called()
    assert '중단' in ui.initial_pose_status.get()


def test_run_while_already_running_does_not_reset_zone_ownership(ui):
    ui.coordinator.enabled = True
    ui._start_fleet()
    ui.coordinator.start.assert_not_called()


def test_popup_save_refuses_unapplied_coordinates_and_preserves_precision(tmp_path):
    panel = DemoPanel.__new__(DemoPanel)
    panel.demo = SimpleNamespace(active=False)
    panel.path = tmp_path / 'demo.json'
    panel.plan = validate_plan(plan())
    panel.plan['a_exit'] = (1.123456789, 2., 0.)
    panel.values = {field: [Value() for _ in range(3)] for field in POSES}
    panel.robots = {role: Value() for role in ('A', 'B')}
    panel.routes = {role: Value() for role in ('A', 'B')}
    panel.status, panel.ui = Value(), Mock()
    panel._fill_configuration()
    panel._save()
    saved = panel.path.read_text()
    assert '1.123456789' in saved
    panel.values['a_exit'][0].set('9')
    with pytest.raises(ValueError, match='좌표 반영'):
        panel._save()
    assert panel.path.read_text() == saved
    panel.ui._map_key.return_value = 'map'
    panel._apply('a_exit')
    panel._save()
    assert panel._configuration()['a_exit'] == (9., 2., 0.)
