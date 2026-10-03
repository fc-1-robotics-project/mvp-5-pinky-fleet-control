import pytest
from multibot_control_ui.lane_routes import load_routes, save_routes, validate_route


def test_empty_routes_require_operator_coordinates(tmp_path):
    assert load_routes(tmp_path / 'routes.json') == {}
    with pytest.raises(ValueError):
        validate_route({})


def test_roundtrip_two_independent_directions(tmp_path):
    route = dict(entry=[1,2,90], exit=[3,4,-90], next=[5,6,0], map_key='map-a')
    reverse = dict(entry=[3,4,90], exit=[1,2,-90], next=[0,0,0], map_key='map-a')
    path = tmp_path / 'routes.json'
    save_routes(path, dict(A_to_B=route, B_to_A=reverse))
    loaded = load_routes(path)
    assert loaded['B_to_A']['entry'] == (3.,4.,90.)
    assert loaded['A_to_B']['map_key'] == 'map-a'


@pytest.mark.parametrize('value', [float('nan'), float('inf'), True, 'x'])
def test_invalid_pose_rejected(value):
    with pytest.raises(ValueError):
        validate_route(dict(entry=[0,0,value], exit=[1,1,0], next=[2,2,0]))


def test_partial_pose_roundtrip_still_requires_complete_mission(tmp_path):
    path = tmp_path / 'routes.json'
    save_routes(path, {'A_to_B': {'entry': [1, 2, 90], 'map_key': 'map-a'}})
    loaded = load_routes(path)
    assert loaded == {'A_to_B': {'entry': (1, 2, 90), 'map_key': 'map-a'}}
    with pytest.raises(ValueError, match='exit'):
        validate_route(loaded['A_to_B'])


def test_invalid_partial_save_preserves_existing_file(tmp_path):
    path = tmp_path / 'routes.json'
    save_routes(path, {'A_to_B': {'entry': [1, 2, 90]}})
    before = path.read_bytes()
    with pytest.raises(ValueError):
        save_routes(path, {'A_to_B': {'entry': [1, 2, float('nan')]}})
    assert path.read_bytes() == before
