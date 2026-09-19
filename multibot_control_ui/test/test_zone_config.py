"""Tests for bottleneck geometry and UI rectangle generation."""

from multibot_control_ui.zone_config import point_in_polygon, rectangular_zone
import pytest


def test_point_in_polygon_includes_boundary() -> None:
    polygon = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))
    assert point_in_polygon((0.5, 0.5), polygon)
    assert point_in_polygon((1.0, 0.5), polygon)
    assert not point_in_polygon((1.5, 0.5), polygon)


def test_route_intersection_accounts_for_robot_radius() -> None:
    zone = rectangular_zone((0.0, 0.0), (1.0, 0.5))
    assert zone.route_intersects([(-1.0, 0.25), (2.0, 0.25)])
    assert zone.route_intersects([(-1.0, -0.05), (2.0, -0.05)])
    assert not zone.route_intersects([(-1.0, -0.5), (2.0, -0.5)])


def test_rectangle_uses_boundary_clearance_without_hold_points() -> None:
    zone = rectangular_zone((0.0, 0.0), (1.0, 0.5))
    assert zone.effective_clearance_m == pytest.approx(0.10)
    assert zone.robot_in_clearance((-0.10, 0.25))
    assert not zone.robot_in_clearance((-0.11, 0.25))


def test_tiny_rectangle_is_rejected() -> None:
    with pytest.raises(ValueError):
        rectangular_zone((0.0, 0.0), (0.05, 1.0))
