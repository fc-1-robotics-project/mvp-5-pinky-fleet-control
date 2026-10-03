"""The Tk loop must exit on signals, including after ROS context shutdown."""
from types import SimpleNamespace
import pytest
from multibot_control_ui import control_ui


@pytest.mark.parametrize('callback', ['_poll_ros', '_refresh_ui'])
@pytest.mark.parametrize('ros_alive', [True, False])
def test_shutdown_closes_window_and_never_queries_closed_ros(monkeypatch, callback, ros_alive):
    events=[]
    ui=control_ui.MultiBotControlUI.__new__(control_ui.MultiBotControlUI)
    ui.closing=False
    ui.shutdown_requested=ros_alive
    ui.root=SimpleNamespace(destroy=lambda:events.append('destroy'))
    ui.node=SimpleNamespace(stop_all=lambda:events.append('zero'))
    ui.coordinator=SimpleNamespace(emergency_stop=lambda reason:events.append('stop'))
    monkeypatch.setattr(control_ui.rclpy, 'ok', lambda:ros_alive)
    getattr(ui,callback)()
    ui._on_close()
    assert ui.closing
    assert events==(['stop','zero','destroy'] if ros_alive else ['destroy'])
