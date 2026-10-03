"""Persist explicit lane entry/exit/next-goal poses without invented defaults."""

import json
import math
from pathlib import Path

DIRECTIONS = ('A_to_B', 'B_to_A')
POSES = ('entry', 'exit', 'next')


def validate_route(route):
    result = {}
    for field in POSES:
        values = route.get(field)
        if not isinstance(values, (list, tuple)) or len(values) != 3:
            raise ValueError(f'{field}: x, y, yaw 입력이 필요합니다.')
        if any(isinstance(v, bool) for v in values):
            raise ValueError('좌표는 유한한 숫자여야 합니다.')
        try:
            values = tuple(float(v) for v in values)
        except (TypeError, ValueError) as error:
            raise ValueError(f'{field}: 좌표 입력을 확인하세요.') from error
        if not all(math.isfinite(v) for v in values):
            raise ValueError('좌표는 유한한 숫자여야 합니다.')
        result[field] = values
    return result


def load_routes(path):
    path = Path(path)
    if not path.exists():
        return {}
    data = json.loads(path.read_text())
    if not isinstance(data, dict) or data.get('version') != 1:
        raise ValueError('지원하지 않는 차선 경로 설정입니다.')
    result = {}
    routes = data.get('routes', {})
    if not isinstance(routes, dict):
        raise ValueError('차선 경로 목록 형식이 잘못되었습니다.')
    for direction, item in routes.items():
        if direction not in DIRECTIONS or not isinstance(item, dict):
            raise ValueError('잘못된 차선 방향 설정입니다.')
        result[direction] = dict(validate_route(item), map_key=item.get('map_key', ''))
    return result


def save_routes(path, routes):
    path = Path(path)
    checked = {}
    for direction, item in routes.items():
        if direction not in DIRECTIONS:
            raise ValueError('잘못된 차선 방향입니다.')
        checked[direction] = dict(validate_route(item), map_key=item.get('map_key', ''))
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(dict(version=1, routes=checked), indent=2, allow_nan=False) + '\n')
    temporary.replace(path)
