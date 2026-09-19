"""Tests for map coordinate and colour conversion."""

import math

from multibot_control_ui.map_math import grid_to_world, occupancy_color
from multibot_control_ui.map_math import quaternion_to_yaw, world_to_grid
from multibot_control_ui.map_math import yaw_degrees_to_quaternion
import pytest


def test_quaternion_to_yaw() -> None:
    yaw = quaternion_to_yaw(0.0, 0.0, math.sin(math.pi / 4),
                            math.cos(math.pi / 4))
    assert yaw == pytest.approx(math.pi / 2)


@pytest.mark.parametrize(
    ('yaw_degrees', 'expected_z', 'expected_w'),
    [
        (155.17060734362803, 0.9766171667419288, 0.21498583587056974),
        (-112.00334746205931, -0.829053907409978, 0.5591686852893746),
    ],
)
def test_yaw_degrees_to_quaternion(
    yaw_degrees: float,
    expected_z: float,
    expected_w: float,
) -> None:
    z, w = yaw_degrees_to_quaternion(yaw_degrees)
    assert z == pytest.approx(expected_z)
    assert w == pytest.approx(expected_w)


def test_world_to_grid_without_rotation() -> None:
    grid_x, grid_y = world_to_grid(1.0, 2.0, -1.0, -2.0, 0.0, 0.05)
    assert grid_x == pytest.approx(40.0)
    assert grid_y == pytest.approx(80.0)


def test_world_to_grid_with_rotated_origin() -> None:
    grid_x, grid_y = world_to_grid(0.0, 1.0, 0.0, 0.0,
                                   math.pi / 2, 1.0)
    assert grid_x == pytest.approx(1.0)
    assert grid_y == pytest.approx(0.0, abs=1e-12)


def test_grid_world_conversion_round_trip() -> None:
    world_x, world_y = grid_to_world(21.5, 8.25, -2.0, 1.5, 0.4, 0.05)
    grid_x, grid_y = world_to_grid(
        world_x,
        world_y,
        -2.0,
        1.5,
        0.4,
        0.05,
    )
    assert grid_x == pytest.approx(21.5)
    assert grid_y == pytest.approx(8.25)


@pytest.mark.parametrize(
    ('value', 'expected'),
    [(-1, '#bdbdbd'), (0, '#ffffff'), (100, '#000000')],
)
def test_occupancy_color(value: int, expected: str) -> None:
    assert occupancy_color(value) == expected
