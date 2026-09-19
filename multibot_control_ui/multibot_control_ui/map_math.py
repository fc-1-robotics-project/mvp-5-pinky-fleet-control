"""Coordinate helpers for displaying ROS occupancy grids in Tkinter."""

import math
from typing import Tuple


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return the planar yaw represented by a quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def yaw_degrees_to_quaternion(yaw_degrees: float) -> Tuple[float, float]:
    """Convert a planar yaw in degrees into quaternion z and w values."""
    yaw = math.radians(yaw_degrees)
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def world_to_grid(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    resolution: float,
) -> Tuple[float, float]:
    """Convert map-frame metres into occupancy-grid cell coordinates."""
    if resolution <= 0.0:
        raise ValueError('Map resolution must be positive.')

    delta_x = x - origin_x
    delta_y = y - origin_y
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    grid_x = (cosine * delta_x + sine * delta_y) / resolution
    grid_y = (-sine * delta_x + cosine * delta_y) / resolution
    return grid_x, grid_y


def grid_to_world(
    grid_x: float,
    grid_y: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    resolution: float,
) -> Tuple[float, float]:
    """Convert occupancy-grid cell coordinates into map-frame metres."""
    if resolution <= 0.0:
        raise ValueError('Map resolution must be positive.')

    local_x = grid_x * resolution
    local_y = grid_y * resolution
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    world_x = origin_x + cosine * local_x - sine * local_y
    world_y = origin_y + sine * local_x + cosine * local_y
    return world_x, world_y


def occupancy_color(value: int) -> str:
    """Map a ROS occupancy value to a Tk-compatible RGB colour."""
    if value < 0:
        return '#bdbdbd'
    bounded = min(100, max(0, value))
    shade = 255 - round(255 * bounded / 100)
    return f'#{shade:02x}{shade:02x}{shade:02x}'
