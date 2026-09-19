"""Bottleneck-zone configuration and planar geometry helpers."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import yaml


Point = Tuple[float, float]


@dataclass(frozen=True)
class BottleneckZone:
    """One capacity-one region and its position-based safety margins."""

    zone_id: str
    polygon: Tuple[Point, ...]
    robot_radius_m: float = 0.09
    clearance_margin_m: float = 0.01

    @property
    def effective_clearance_m(self) -> float:
        """Return the footprint radius plus the configured safety margin."""
        return self.robot_radius_m + self.clearance_margin_m

    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        """Return the map-frame axis-aligned polygon bounds."""
        x_values = [point[0] for point in self.polygon]
        y_values = [point[1] for point in self.polygon]
        return min(x_values), min(y_values), max(x_values), max(y_values)

    def robot_overlaps(self, point: Point) -> bool:
        """Return whether a circular robot footprint overlaps the polygon."""
        return point_near_polygon(point, self.polygon, self.robot_radius_m)

    def contains_center(self, point: Point) -> bool:
        """Return whether the robot center is inside the drawn polygon."""
        return point_in_polygon(point, self.polygon)

    def robot_in_clearance(self, point: Point) -> bool:
        """Return whether a robot still overlaps the expanded exit region."""
        # Entry uses the physical footprint radius. Exit/HOLD adds a small
        # margin, producing hysteresis so ownership does not flicker.
        return point_near_polygon(
            point,
            self.polygon,
            self.effective_clearance_m,
        )

    def route_intersects(self, points: Sequence[Point]) -> bool:
        """Return whether any route segment intersects the protected region."""
        if len(points) == 1:
            return self.robot_overlaps(points[0])
        return any(
            segment_near_polygon(start, end, self.polygon, self.robot_radius_m)
            for start, end in zip(points, points[1:])
        )


def load_zones(path: str) -> List[BottleneckZone]:
    """Load and validate enabled bottleneck zones from YAML."""
    if not path:
        return []
    config_path = Path(path).expanduser()
    if not config_path.exists():
        raise ValueError(f'Zone configuration does not exist: {config_path}')
    with config_path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}

    zones = []
    seen_ids = set()
    for raw in document.get('zones', []):
        if not raw.get('enabled', True):
            continue
        zone_id = str(raw.get('id', '')).strip()
        if not zone_id or zone_id in seen_ids:
            raise ValueError(f'Invalid or duplicate zone id: {zone_id!r}')
        polygon = tuple(_point(value, 'polygon') for value in raw['polygon'])
        if len(polygon) < 3:
            raise ValueError(f'{zone_id}: polygon needs at least three points')
        if abs(polygon_area(polygon)) < 1e-6:
            raise ValueError(f'{zone_id}: polygon area must be non-zero')
        radius = _positive(raw.get('robot_radius_m', 0.09), 'robot_radius_m')
        clearance = _positive(
            raw.get('clearance_margin_m', 0.01),
            'clearance_margin_m',
            allow_zero=True,
        )
        zones.append(BottleneckZone(
            zone_id=zone_id,
            polygon=polygon,
            robot_radius_m=radius,
            clearance_margin_m=clearance,
        ))
        seen_ids.add(zone_id)
    return zones


def rectangular_zone(
    first: Point,
    second: Point,
    *,
    zone_id: str = 'ui_bottleneck',
    robot_radius_m: float = 0.09,
    clearance_margin_m: float = 0.01,
) -> BottleneckZone:
    """Create an axis-aligned position-supervised zone from a map drag."""
    min_x, max_x = sorted((first[0], second[0]))
    min_y, max_y = sorted((first[1], second[1]))
    if max_x - min_x < 0.10 or max_y - min_y < 0.10:
        raise ValueError('Bottleneck rectangle must be at least 0.10 m per side')
    polygon = (
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y),
    )
    return BottleneckZone(
        zone_id=zone_id,
        polygon=polygon,
        robot_radius_m=robot_radius_m,
        clearance_margin_m=clearance_margin_m,
    )


def point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    """Return whether a point is inside or on the boundary of a polygon."""
    x, y = point
    inside = False
    for start, end in _edges(polygon):
        if distance_to_segment(point, start, end) < 1e-9:
            return True
        x1, y1 = start
        x2, y2 = end
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
    return inside


def point_near_polygon(
    point: Point,
    polygon: Sequence[Point],
    margin: float,
) -> bool:
    """Return whether a point is inside or within margin of a polygon."""
    if point_in_polygon(point, polygon):
        return True
    return any(
        distance_to_segment(point, start, end) <= margin + 1e-9
        for start, end in _edges(polygon)
    )


def segment_near_polygon(
    start: Point,
    end: Point,
    polygon: Sequence[Point],
    margin: float,
) -> bool:
    """Conservatively test a route segment against an expanded polygon."""
    if point_near_polygon(start, polygon, margin):
        return True
    if point_near_polygon(end, polygon, margin):
        return True
    if any(segments_intersect(start, end, a, b) for a, b in _edges(polygon)):
        return True
    return any(
        distance_to_segment(vertex, start, end) <= margin
        for vertex in polygon
    )


def segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    """Return whether two closed line segments intersect."""
    def orientation(p: Point, q: Point, r: Point) -> float:
        return (
            (q[0] - p[0]) * (r[1] - p[1])
            - (q[1] - p[1]) * (r[0] - p[0])
        )

    values = (
        orientation(a, b, c),
        orientation(a, b, d),
        orientation(c, d, a),
        orientation(c, d, b),
    )
    if values[0] * values[1] < 0.0 and values[2] * values[3] < 0.0:
        return True
    return any(
        abs(value) < 1e-9 and distance_to_segment(point, start, end) < 1e-9
        for value, point, start, end in (
            (values[0], c, a, b),
            (values[1], d, a, b),
            (values[2], a, c, d),
            (values[3], b, c, d),
        )
    )


def distance_to_segment(point: Point, start: Point, end: Point) -> float:
    """Return the Euclidean distance from point to a closed segment."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-18:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    ratio = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / length_squared
    ratio = max(0.0, min(1.0, ratio))
    nearest = (start[0] + ratio * dx, start[1] + ratio * dy)
    return math.hypot(point[0] - nearest[0], point[1] - nearest[1])


def polygon_area(polygon: Sequence[Point]) -> float:
    """Return the signed area of a polygon."""
    return sum(
        start[0] * end[1] - end[0] * start[1]
        for start, end in _edges(polygon)
    ) / 2.0


def _edges(polygon: Sequence[Point]) -> Iterable[Tuple[Point, Point]]:
    return zip(polygon, polygon[1:] + polygon[:1])


def _point(raw: Sequence[float], label: str) -> Point:
    if len(raw) != 2:
        raise ValueError(f'{label} point must contain x and y')
    point = (float(raw[0]), float(raw[1]))
    if not all(math.isfinite(value) for value in point):
        raise ValueError(f'{label} values must be finite')
    return point


def _positive(value: float, label: str, allow_zero: bool = False) -> float:
    number = float(value)
    valid = number >= 0.0 if allow_zero else number > 0.0
    if not math.isfinite(number) or not valid:
        raise ValueError(f'{label} must be a valid positive value')
    return number
